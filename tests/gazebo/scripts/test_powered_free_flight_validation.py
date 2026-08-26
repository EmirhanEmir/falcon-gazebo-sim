#!/usr/bin/env python3
"""
FALCON V2 - POWERED_TRIM_AND_FREE_FLIGHT_VALIDATION, Phase 2 (gazebo-testing,
2026-08-23).

Full free 6-DOF validation of the trim candidate selected by
tests/gazebo/scripts/test_powered_trim_search.py (Phase 1):
  throttle = 0.5125 (both motors), elevator = +6.0deg physical joint angle
  (both left_elevator_joint/right_elevator_joint -> delta_e_aero=-6.0deg via
  the verified elevator_sign=-1.0 mapping), ailerons/rudder = 0 (neutral).

Procedure (exactly matching the task brief / Phase 1's already-validated
technique - see test_powered_trim_search.py's header comment for the
empirical justification of holding BOTH linear AND angular rate during the
short settling phase, not linear-only):
  1. One-time teleport (Model.set_world_pose_cmd) to altitude=100m, level
     attitude.
  2. hold_step() both linear (u=18.14534, v=0, w=-0.78335 m/s, i.e.
     V=18.162 m/s, alpha=2.472deg - the same common baseline Phase 1 searched
     from) AND angular (p=q=r=0) targets for HOLD_STEPS=800 (0.8s) - long
     enough for RPM to settle under the real rotor ODE, short enough to keep
     the P-only angular hold's steady-state residual-rate attitude drift
     small (Phase 1 measured this drift as ~0.2-2deg over an 0.8s hold,
     depending on candidate - see that script's header note).
  3. RELEASE EVERYTHING - no velocity hold, no position hold, no
     P-controller, no attitude controller, no artificial stabilizer, no
     external holding force, no kinematic intervention, for the entire
     remainder of the run (20s / 20000 steps). Gravity, FalconV2Aerodynamics,
     FalconV2Propulsion all ON, exactly as attached in model/model.sdf,
     unmodified. Elevator/ailerons/rudder held at their commanded angle only
     via pin_control_surface_joints() (the same TEMPORARY_NUMERICAL_MASS
     joint-position technique this whole test suite already uses for the 5
     massless/undamped/unactuated control-surface joints - this does NOT
     apply any force/torque to base_link itself, it only keeps the surface
     link at its commanded angle so AerodynamicsSystem reads the correct
     control input, exactly as documented in
     tests/gazebo/worlds/falcon_v2_powered_free_flight_gui_world.sdf's own
     header comment about its equivalent JointPositionController mechanism).
     Throttle republished every tick (topics are not latched, per
     plugins/propulsion/README.md).

No aircraft physics parameter is read for any purpose other than
loading the existing config, and none is modified anywhere in this script.
"""
import json
import math
import sys

import propulsion_lib as PL
import aero_lib as AL

PL.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = f"{REPO_ROOT}/tests/gazebo/results"
WORLD = f"{REPO_ROOT}/tests/gazebo/worlds/falcon_v2_freefall_world.sdf"

CFG = AL.load_config()
MASS = 5.9348
I_DIAG = (0.7284, 0.2507, 0.9523)
KP_LIN = 150.0
KP_ANG = 400.0
HOLD_STEPS = 800
RELEASE_STEPS = 30000  # 30 s of genuinely free 6-DOF flight (cheap: ~7s wall-clock for 20s sim in this pass's own timing check, so extended to the task brief's full 30s ceiling to capture ~2-3 phugoid cycles per the documented ~10s period)
ALTITUDE_M = 100.0
U_HOLD, W_HOLD = 18.14534, -0.78335

# ---------------- Selected trim candidate (Phase 1 result) ----------------
# tests/gazebo/scripts/test_powered_trim_search.py's full 4-round search
# (coarse grid -> refine 1 -> refine 2 -> settled/tail-window check (round 3)
# -> directional throttle extrapolation (round 4)) selected this candidate:
# round 4's own live measurement already shows a near-zero SETTLED tail-
# window vertical speed (+0.0191 m/s over the last ~3.6s of a 12s check) -
# this Phase 2 run re-verifies that over the full, genuinely free 15-30s
# window with complete telemetry, per the task brief.
THROTTLE = 0.4915
ELEVATOR_AERO_DEG = -5.5
ELEVATOR_THETA_RAD = math.radians(-ELEVATOR_AERO_DEG)  # +5.5deg physical

TELEMETRY_EVERY_N = 1000  # 1 Hz (1000 steps @ 1ms)


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    model_e = world.model_by_name(ecm, "falcon_v2")
    return sim.Model(model_e)


def quat_rpy(rot):
    qx, qy, qz, qw = rot.x(), rot.y(), rot.z(), rot.w()
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    log("FALCON V2 - POWERED_TRIM_AND_FREE_FLIGHT_VALIDATION - Phase 2 free flight (gazebo-testing, 2026-08-23)")
    log(f"World: {WORLD}")
    log(f"Selected trim candidate: throttle={THROTTLE} elevator_aero={ELEVATOR_AERO_DEG}deg "
        f"(theta={math.degrees(ELEVATOR_THETA_RAD):+.2f}deg physical)")
    log(f"HOLD_STEPS={HOLD_STEPS} ({HOLD_STEPS*AL.STEP}s) at u={U_HOLD} w={W_HOLD}, "
        f"then RELEASE_STEPS={RELEASE_STEPS} ({RELEASE_STEPS*AL.STEP}s) genuinely free 6-DOF flight.")
    log("")

    state = {"n": 0, "teleported": False, "series": [], "any_nan": False,
             "throttle_cmd": None, "prop_diag": None, "nan_step": None}

    def on_pre(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, ALTITUDE_M, 0, 0, 0))
            state["teleported"] = True
            state["throttle_cmd"] = PL.ThrottleCommander()
        state["throttle_cmd"].set(left=THROTTLE, right=THROTTLE)
        state["throttle_cmd"].tick()
        PL.pin_control_surface_joints(model, ecm, sim, positions={
            "left_elevator_joint": ELEVATOR_THETA_RAD,
            "right_elevator_joint": ELEVATOR_THETA_RAD})
        n = state["n"]
        if n < HOLD_STEPS:
            AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(U_HOLD, 0, W_HOLD),
                         gm.Vector3d(0, 0, 0), kp_lin=KP_LIN, kp_ang=KP_ANG)
        # else: fully released - no controller, no stabilizer, no holding force at all.

    def on_post(info, ecm):
        if state["prop_diag"] is None:
            try:
                state["prop_diag"] = PL.DiagSubscriber()
            except Exception:
                pass
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        wpose = base.world_pose(ecm)
        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        if wpose is None or lv is None or av is None:
            state["any_nan"] = True
            if state["nan_step"] is None:
                state["nan_step"] = n
            state["n"] += 1
            return
        rot = wpose.rot()
        lv_b = rot.rotate_vector_reverse(lv)
        av_b = rot.rotate_vector_reverse(av)
        raw_vals = [lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z(),
                    wpose.pos().z(), lv.z()]
        if any(math.isnan(x) or math.isinf(x) for x in raw_vals):
            state["any_nan"] = True
            if state["nan_step"] is None:
                state["nan_step"] = n

        if n >= HOLD_STEPS and (n - HOLD_STEPS) % TELEMETRY_EVERY_N == 0:
            roll, pitch, yaw = quat_rpy(rot)
            V = math.sqrt(lv_b.x() ** 2 + lv_b.y() ** 2 + lv_b.z() ** 2)
            alpha = math.atan2(-lv_b.z(), lv_b.x())
            beta = math.atan2(lv_b.y(), math.hypot(lv_b.x(), lv_b.z()))
            prop = state["prop_diag"].latest() if state["prop_diag"] else None
            state["series"].append(dict(
                n=n, t_release=(n - HOLD_STEPS) * AL.STEP,
                V=V, alt=wpose.pos().z(), world_vz=lv.z(),
                roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch), yaw_deg=math.degrees(yaw),
                alpha_deg=math.degrees(alpha), beta_deg=math.degrees(beta),
                p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z()),
                left_rpm=(prop["left"]["rpm"] if prop else None),
                right_rpm=(prop["right"]["rpm"] if prop else None),
                left_thrust_N=(prop["left"]["thrust_N"] if prop else None),
                right_thrust_N=(prop["right"]["thrust_N"] if prop else None),
            ))
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, HOLD_STEPS + RELEASE_STEPS, False)

    log(f"Total steps run: {state['n']} (expected {HOLD_STEPS + RELEASE_STEPS})")
    log(f"any_nan: {state['any_nan']} (first at step {state['nan_step']})" if state["any_nan"]
        else "any_nan: False")
    log("")
    log(f"{'t(s)':>6} {'V(m/s)':>8} {'alt(m)':>8} {'vz(m/s)':>8} {'roll':>7} {'pitch':>8} "
        f"{'yaw':>7} {'alpha':>7} {'beta':>7} {'p(d/s)':>8} {'q(d/s)':>8} {'r(d/s)':>8} "
        f"{'L_rpm':>8} {'R_rpm':>8} {'L_T(N)':>7} {'R_T(N)':>7}")
    for s in state["series"]:
        log(f"{s['t_release']:6.1f} {s['V']:8.3f} {s['alt']:8.2f} {s['world_vz']:8.3f} "
            f"{s['roll_deg']:7.2f} {s['pitch_deg']:8.2f} {s['yaw_deg']:7.2f} "
            f"{s['alpha_deg']:7.2f} {s['beta_deg']:7.2f} "
            f"{s['p_deg_s']:8.2f} {s['q_deg_s']:8.2f} {s['r_deg_s']:8.2f} "
            f"{s['left_rpm']:8.1f} {s['right_rpm']:8.1f} {s['left_thrust_N']:7.3f} {s['right_thrust_N']:7.3f}")

    series = state["series"]
    start, end = series[0], series[-1]
    max_abs_pitch = max(abs(s["pitch_deg"]) for s in series)
    max_abs_roll = max(abs(s["roll_deg"]) for s in series)
    max_abs_yaw_rate_from_start = max(abs(s["yaw_deg"] - start["yaw_deg"]) for s in series)
    max_abs_alpha = max(abs(s["alpha_deg"]) for s in series)

    log("")
    log("=" * 78)
    log("SUMMARY")
    log("=" * 78)
    log(f"Duration observed: {end['t_release']:.1f} s (of {RELEASE_STEPS*AL.STEP:.1f} s commanded)")
    log(f"Airspeed:  start={start['V']:.3f} m/s  end={end['V']:.3f} m/s  drift={end['V']-start['V']:+.3f} m/s")
    log(f"Altitude:  start={start['alt']:.2f} m  end={end['alt']:.2f} m  drift={end['alt']-start['alt']:+.2f} m")
    log(f"Vert.spd:  start={start['world_vz']:+.3f} m/s  end={end['world_vz']:+.3f} m/s")
    log(f"Roll:      start={start['roll_deg']:+.2f}deg  end={end['roll_deg']:+.2f}deg  max|roll|={max_abs_roll:.2f}deg")
    log(f"Pitch:     start={start['pitch_deg']:+.2f}deg  end={end['pitch_deg']:+.2f}deg  max|pitch|={max_abs_pitch:.2f}deg")
    log(f"Yaw:       start={start['yaw_deg']:+.2f}deg  end={end['yaw_deg']:+.2f}deg  "
        f"max|yaw-start|={max_abs_yaw_rate_from_start:.2f}deg")
    log(f"Alpha:     start={start['alpha_deg']:+.2f}deg  end={end['alpha_deg']:+.2f}deg  max|alpha|={max_abs_alpha:.2f}deg")
    log(f"Beta:      start={start['beta_deg']:+.3f}deg  end={end['beta_deg']:+.3f}deg")
    log(f"RPM:       L start={start['left_rpm']:.1f} end={end['left_rpm']:.1f} | "
        f"R start={start['right_rpm']:.1f} end={end['right_rpm']:.1f}")
    log(f"Thrust:    L start={start['left_thrust_N']:.3f}N end={end['left_thrust_N']:.3f}N | "
        f"R start={start['right_thrust_N']:.3f}N end={end['right_thrust_N']:.3f}N")

    with open(f"{RESULTS_DIR}/powered_free_flight_validation_result.json", "w") as f:
        json.dump({
            "candidate": dict(throttle=THROTTLE, elevator_aero_deg=ELEVATOR_AERO_DEG,
                               elevator_theta_deg=math.degrees(ELEVATOR_THETA_RAD)),
            "hold_steps": HOLD_STEPS, "release_steps": RELEASE_STEPS,
            "any_nan": state["any_nan"], "nan_step": state["nan_step"],
            "telemetry": series,
            "summary": dict(
                duration_s=end["t_release"],
                airspeed_start=start["V"], airspeed_end=end["V"],
                altitude_start=start["alt"], altitude_end=end["alt"],
                vertical_speed_start=start["world_vz"], vertical_speed_end=end["world_vz"],
                roll_start=start["roll_deg"], roll_end=end["roll_deg"], max_abs_roll=max_abs_roll,
                pitch_start=start["pitch_deg"], pitch_end=end["pitch_deg"], max_abs_pitch=max_abs_pitch,
                yaw_start=start["yaw_deg"], yaw_end=end["yaw_deg"],
                max_abs_yaw_drift=max_abs_yaw_rate_from_start,
                alpha_start=start["alpha_deg"], alpha_end=end["alpha_deg"], max_abs_alpha=max_abs_alpha,
            ),
        }, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/powered_free_flight_validation_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return state["any_nan"]


if __name__ == "__main__":
    any_nan = run()
    sys.exit(1 if any_nan else 0)

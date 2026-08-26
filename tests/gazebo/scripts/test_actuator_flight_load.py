#!/usr/bin/env python3
"""
FALCON V2 - ACTUATOR_SERVO_MODEL_V1 live flight-load test (gazebo-testing,
2026-08-23), task section 17.

Small-deflection (+/-5deg, +/-10deg) actuator/aero response check for each
control-surface class (elevator symmetric, aileron differential, rudder), in
the established powered-trim FLOW region (V~=18 m/s, throttle~=0.4915 both
motors - reusing the exact trim state found by POWERED_TRIM_AND_FREE_FLIGHT_
VALIDATION, tests/gazebo/results/powered_trim_search_result.json /
docs/test_results/2026-08-23_powered_trim_and_free_flight_validation_report.md:
V=18.165 m/s, alpha=2.461deg at the selected trim candidate). Per task
instruction, this is explicitly NOT a CONTROL_AUTHORITY_EFFECTIVENESS_
VALIDATION pass (no roll/pitch/yaw performance measurement/tuning) - it only
checks the ACTUATOR's own physical response (smooth motion, settling,
stability) and confirms the aerodynamics plugin reacts with the correct
sign, at small deflections representative of real stick input around trim.

Method: teleport to 100m altitude (matching the established trim-search
technique, tests/gazebo/scripts/test_powered_trim_search.py - confirmed
clean/non-freezing, unlike Link.set_linear_velocity()/set_angular_velocity()),
propulsion_lib.ThrottleCommander set to 0.4915/0.4915 (both motors,
republished every tick), aero_lib.hold_step() holding BOTH linear (u,w set
to the trim V/alpha; v=0) AND angular (p=q=r=0) rate throughout the ENTIRE
run - per task instruction ("teleport+hold_step is fine here since this is
testing actuator/aero response, not free-flight dynamics"), never released.
Elevator/aileron/rudder actuator commands are swept from a NEUTRAL (0rad)
baseline, not layered on top of the trim elevator offset (+5.5deg) - since
hold_step actively holds body rate/attitude regardless of control input in
this harness, layering a trim offset underneath would not change what this
specific actuator-response check can observe, and testing from a clean
neutral baseline keeps each case's interpretation unambiguous (documented
design choice, not an attempt to re-validate trim itself, which
POWERED_TRIM_AND_FREE_FLIGHT_VALIDATION already did).

Real actuator commands are sent via actuator_lib.ActuatorCommander (gz-
transport Double per surface -> FalconV2Actuators -> Joint::SetForce()) -
NEVER reset_position()/pin injection for the joint(s) under test this pass
(the other, non-tested control-surface joints + both prop joints ARE pinned
via actuator_lib.pin_other_child_joints(), the same isolation technique
already validated throughout this suite).

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
control-sign mapping, motor/prop constant, actuator max_rate/max_effort/
kp/kd, hinge geometry) is modified anywhere in this script.

RE-TEST, 2026-08-24 (ACTUATOR_SERVO_MODEL_V1_GRAVITY_DROOP_FIX): re-run
unmodified in case set against controls-integration's PID (integral-action)
fix to ActuatorModel.hh/ActuatorSystem.{cc,hh} for the gravity-droop bug
this script originally found (8/12 FAIL, all 4 ELEVATOR_SYMMETRIC_*/
AILERON_DIFFERENTIAL_* cases). Two changes from the original version, both
test-infrastructure-only (no aircraft physics parameter touched):
  1. SETTLE_STEPS/TAIL_STEPS widened (see their definitions below) to give
     the new integral term's predicted ~3s settling time (per
     actuator_v1_config.yaml's integral_derivation_note) genuine room to
     reach steady state before the tail window is sampled.
  2. sign_ok bug fix (see run_case()'s "Sign / aero reaction check" comment
     below): the original sign_ok comparison used the raw commanded
     PHYSICAL joint sign as its expected value for all 3 surface classes.
     This was silently wrong for elevator specifically (elevator_sign =
     -1.0, VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST, aero_v1_config.yaml) -
     masked pre-fix because ELEVATOR_SYMMETRIC_* cases already failed on
     settle_ok for the (unrelated) gravity-droop reason, and invisible for
     aileron/rudder only because aileron_sign/rudder_sign both happen to be
     +1.0. Fixed to match the already-correct convention
     test_actuator_aero_integration.py's own sign checks use
     (expected_deg = CFG.<surface>Sign * commanded_deg) - a test-methodology
     correction, not a change to any aero/actuator sign convention or
     physical value.

RE-TEST ROUND 2, 2026-08-24 (setpoint-weighting overshoot fix): re-run unmodified in case set
against controls-integration's round-2 fix (spWeightB=0.7 setpoint weighting on the P-term only,
ActuatorConfig/ActuatorModel.hh, actuator_v1_config.yaml) for the transient-overshoot finding the
round-1 re-test above surfaced (6/12 cases, 1.96-3.46deg peak overshoot). One test-infrastructure-
only addition this round (no physics/control-law parameter touched, no pass/fail criterion
changed): decimated (every DECIMATE_STEPS ticks) raw theta(t) time-series capture for the worst
round-1 overshoot cases (TIMESERIES_CASE_NAMES below), saved into the result JSON, so the
overshoot claim can be verified directly from saved data rather than an ad hoc diagnostic
(validation's round-1 gap). SETTLE_STEPS/TAIL_STEPS reused unchanged from the round-1 re-test
(4500/500) per task instruction - no need to widen further.
"""
import json
import math
import sys

import actuator_lib as ACT
import aero_lib as AL
import propulsion_lib as PL

ACT.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
CFG = AL.load_config()
ACFG = ACT.load_actuator_config()

MASS = 5.9348
I_DIAG = (0.7284, 0.2507, 0.9523)
KP_LIN = 150.0
KP_ANG = 150.0
ALTITUDE_M = 100.0

# Established trim condition, cited verbatim from
# docs/test_results/2026-08-23_powered_trim_and_free_flight_validation_report.md
# ("Selected trim candidate" table): V=18.165 m/s, alpha=2.461deg, throttle
# (L=R)=0.4915. u/w derived here (u=V*cos(alpha), w=-V*sin(alpha), FLU/
# body-frame, standard alpha=atan2(-w,u) convention matching aero_lib.py) -
# READ-ONLY reuse of that already-established result, not a re-derivation.
TRIM_V = 18.165
TRIM_ALPHA_DEG = 2.461
TRIM_THROTTLE = 0.4915
_alpha_rad = math.radians(TRIM_ALPHA_DEG)
TRIM_U = TRIM_V * math.cos(_alpha_rad)
TRIM_W = -TRIM_V * math.sin(_alpha_rad)
LIN_TARGET = gm.Vector3d(TRIM_U, 0.0, TRIM_W)
ANG_TARGET = gm.Vector3d(0.0, 0.0, 0.0)

WARM_STEPS = 300      # 0.3s - trim flow + throttle spin-up settle before any command
# SETTLE_STEPS widened 2026-08-24 (ACTUATOR_SERVO_MODEL_V1_GRAVITY_DROOP_FIX re-test) from the
# original 600 steps (0.6s) to 4500 steps (4.5s): controls-integration's ki derivation
# (actuator_v1_config.yaml integral_derivation_note) predicts the gravity-droop residual decays via
# a dominant pole at -2.09 rad/s (tau~0.48s) to <0.06deg by t=2s and <0.01deg by t=3s after the
# command step (rate-limited setpoint itself reaches target within tens of ms, confirmed by
# actuator_model_selftest.cc's ACTUATOR_POSITIVE_STEP_TEST, so t=3s-from-command is effectively
# t=3s-from-disturbance-onset too). 4.5s gives ~1.5s of margin past the predicted <0.01deg point so
# the tail window below is unambiguously sampled AFTER full settling, not cut short.
SETTLE_STEPS = 4500
TAIL_STEPS = 500       # 0.5s - last N steps of the run used for settled-state stats; spans
                        # t=4.0-4.5s after the command step, well past the t=3s predicted full-settle
                        # point above (was 150 steps/0.15s pre-fix; widened for the same reason)

WORLD_SDF = "/home/emirhan/Desktop/FalconV2/tests/gazebo/worlds/falcon_v2_freefall_world.sdf"

# Decimated raw time-series capture, added 2026-08-24 round-2 re-test
# (ACTUATOR_SERVO_MODEL_V1, controls-integration's setpoint-weighting/spWeightB=0.7 fix).
# validation flagged, in round-1's review, that the transient-overshoot claim was demonstrated
# only via an ad hoc, unrecorded diagnostic (not saved to any result file). For the worst
# round-1 overshoot cases (docs/test_results/2026-08-24_actuator_servo_model_v1_test_report.md,
# section "New finding: transient overshoot..."), save a decimated (every DECIMATE_STEPS ticks,
# not full per-tick) theta(t) series into the result JSON so the before/after overshoot claim can
# be verified directly from saved data. Test-infrastructure-only addition: does not change any
# pass/fail criterion, does not touch any physics/control-law parameter, and does not affect the
# other 8 cases (raw_timeseries_decimated stays null for them, keeping the file size reasonable).
TIMESERIES_CASE_NAMES = {
    "AILERON_DIFFERENTIAL_-10DEG",  # round-1 single worst overshoot: 3.4645deg
    "AILERON_DIFFERENTIAL_-5DEG",   # round-1: 2.7704deg
    "ELEVATOR_SYMMETRIC_-5DEG",     # round-1 worst elevator overshoot: 1.9757deg
    "ELEVATOR_SYMMETRIC_-10DEG",    # round-1: 1.9621deg
}
DECIMATE_STEPS = 10  # every 10 ticks = 10ms resolution; round-1's diagnosis placed the peak
                      # ~50-150ms post-command, so this comfortably resolves the peak and decay shape


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    model_e = world.model_by_name(ecm, "falcon_v2")
    return sim.Model(model_e)


def run_case(log, case_name, commands_rad, joints_of_interest, delta_formula, delta_label,
             expected_delta_deg_signed, commanded_deg_signed):
    total_steps = WARM_STEPS + SETTLE_STEPS + 5
    max_step_allowed = ACFG["max_rate_rad_s"] * ACT.STEP * 1.5  # 1.5x safety margin over the documented rate limit

    state = {"n": 0, "teleported": False, "cmd": None, "thr": None,
             "actuator_diag": None, "aero_diag": None, "any_nan": False,
             "theta": {jn: [] for _, jn in joints_of_interest},
             "rate": {jn: [] for _, jn in joints_of_interest},
             "diag_flags": [], "t": []}

    def on_pre(info, ecm):
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, ALTITUDE_M, 0, 0, 0))
            state["teleported"] = True
            state["cmd"] = ACT.ActuatorCommander()
            state["thr"] = PL.ThrottleCommander()
        state["thr"].set(left=TRIM_THROTTLE, right=TRIM_THROTTLE)
        state["thr"].tick()
        if n >= WARM_STEPS:
            state["cmd"].set(**commands_rad)
        state["cmd"].tick()
        leave_free = [jn for _, jn in joints_of_interest]
        ACT.pin_other_child_joints(model, ecm, sim, leave_free_joints=leave_free)
        AL.hold_step(base, ecm, MASS, I_DIAG, LIN_TARGET, ANG_TARGET,
                     kp_lin=KP_LIN, kp_ang=KP_ANG, ang_axis_mask=(True, True, True))

    def on_post(info, ecm):
        if state["actuator_diag"] is None:
            try:
                state["actuator_diag"] = ACT.DiagSubscriber()
            except Exception:
                pass
        if state["aero_diag"] is None:
            try:
                state["aero_diag"] = AL.DiagSubscriber()
            except Exception:
                pass
        n = state["n"]
        model = get_model(ecm)
        for surf, jn in joints_of_interest:
            th, rt = ACT.read_joint_state(model, ecm, sim, jn)
            state["theta"][jn].append(th if th is not None else float("nan"))
            state["rate"][jn].append(rt if rt is not None else float("nan"))
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        if lv is None or av is None:
            state["any_nan"] = True
        else:
            vals = [lv.x(), lv.y(), lv.z(), av.x(), av.y(), av.z()]
            if any(math.isnan(v) or math.isinf(v) for v in vals):
                state["any_nan"] = True
        ad = state["actuator_diag"].latest() if state["actuator_diag"] else None
        if ad:
            flags = {surf: (ad[surf]["target_clamp_active"], ad[surf]["effort_clamp_active"])
                     for surf, _ in joints_of_interest}
            state["diag_flags"].append(flags)
        state["t"].append(n * ACT.STEP)
        state["n"] += 1

    fixture = sim.TestFixture(WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    # ---- Smooth motion: no per-tick jump beyond max_rate*dt*1.5 margin ----
    max_jump = {}
    for _, jn in joints_of_interest:
        series = state["theta"][jn]
        jumps = [abs(series[i] - series[i - 1]) for i in range(1, len(series))
                 if not (math.isnan(series[i]) or math.isnan(series[i - 1]))]
        max_jump[jn] = max(jumps) if jumps else 0.0
    smooth_ok = all(v <= max_step_allowed for v in max_jump.values())

    # ---- Overshoot: actual angle should never exceed target beyond a small margin ----
    overshoot_deg = {}
    for surf, jn in joints_of_interest:
        target = commands_rad.get(surf, 0.0)
        series = state["theta"][jn][WARM_STEPS:]
        if target >= 0:
            worst = max((v - target for v in series if not math.isnan(v)), default=0.0)
        else:
            worst = max((target - v for v in series if not math.isnan(v)), default=0.0)
        overshoot_deg[jn] = math.degrees(max(worst, 0.0))
    overshoot_ok = all(v < 1.0 for v in overshoot_deg.values())  # < 1 deg overshoot tolerance

    # ---- Settling: tail-window mean close to target, low residual ----
    tail_mean = {}
    tail_noise = {}
    tail_rate_absmax = {}
    for surf, jn in joints_of_interest:
        tail_theta = state["theta"][jn][-TAIL_STEPS:]
        tail_rate = state["rate"][jn][-TAIL_STEPS:]
        tail_mean[jn] = sum(tail_theta) / len(tail_theta)
        tail_noise[jn] = max(tail_theta) - min(tail_theta)
        tail_rate_absmax[jn] = max(abs(v) for v in tail_rate)
    residual_deg = {jn: math.degrees(abs(tail_mean[jn] - commands_rad.get(surf, 0.0)))
                     for surf, jn in joints_of_interest}
    settle_ok = all(v < 1.0 for v in residual_deg.values())  # < 1 deg residual

    # ---- No oscillation/chatter in the settled tail ----
    no_chatter_ok = (all(math.degrees(v) < 0.3 for v in tail_noise.values()) and
                      all(v < 0.02 for v in tail_rate_absmax.values()))  # rad/s

    # ---- effort/target clamp flags over the run (watch-item) ----
    any_target_clamp = False
    any_effort_clamp_tail = False
    if state["diag_flags"]:
        for flags in state["diag_flags"]:
            for surf, (tc, ec) in flags.items():
                if tc > 0.5:
                    any_target_clamp = True
        for flags in state["diag_flags"][-30:]:
            for surf, (tc, ec) in flags.items():
                if ec > 0.5:
                    any_effort_clamp_tail = True

    # ---- Sign / aero reaction check ----
    # sign_ok bug fix, 2026-08-24 (ACTUATOR_SERVO_MODEL_V1_GRAVITY_DROOP_FIX re-test,
    # gazebo-testing, test-methodology finding, NOT an actuator/aero physics change): this
    # comparison must be against the AERO-CONVENTION expected sign (CFG.<surface>Sign *
    # commanded_deg, exactly the convention test_actuator_aero_integration.py's own
    # expected_deg=CFG.elevatorSign*ele_deg already uses correctly), not the raw physical
    # commanded joint sign. aileron_sign/rudder_sign are both VERIFIED_BY_GAZEBO_GEOMETRY_
    # SIGN_TEST = +1.0 (docs/source_of_truth/aerodynamics/aero_v1_config.yaml), so this bug was
    # invisible for those two surface classes; elevator_sign = -1.0 (also VERIFIED_BY_GAZEBO_
    # GEOMETRY_SIGN_TEST - a real, correct, already-validated aero sign convention, not a
    # defect), so comparing against the raw commanded sign was always wrong for elevator and
    # produced sign_ok=False on every ELEVATOR_SYMMETRIC_* case regardless of the PID fix -
    # masked pre-fix only because settle_ok was already False there for the unrelated gravity-
    # droop reason. Confirmed by direct comparison against the already-passing
    # test_actuator_aero_integration.py, which uses the corrected convention below.
    thetas = {surf: tail_mean[jn] for surf, jn in joints_of_interest}
    computed_delta_deg = math.degrees(delta_formula(thetas))
    sign_ok = ((computed_delta_deg > 0) == (expected_delta_deg_signed > 0)
               if expected_delta_deg_signed != 0 else True)

    aero_diag_latest = state["aero_diag"].latest() if state["aero_diag"] else None

    # ---- Decimated raw time-series (worst round-1 overshoot cases only, see comment above) ----
    raw_timeseries_decimated = None
    if case_name in TIMESERIES_CASE_NAMES:
        idx = list(range(0, len(state["t"]), DECIMATE_STEPS))
        raw_timeseries_decimated = dict(
            decimate_every_n_steps=DECIMATE_STEPS, dt_s=ACT.STEP * DECIMATE_STEPS, n_samples=len(idx),
            t_s=[state["t"][i] for i in idx],
            theta_deg={jn: [(math.degrees(state["theta"][jn][i])
                              if not math.isnan(state["theta"][jn][i]) else None) for i in idx]
                       for _, jn in joints_of_interest})

    log(f"--- {case_name} ---")
    log(f"Commanded (rad): {commands_rad}; trim flow held: u={TRIM_U:.4f} w={TRIM_W:.4f} "
        f"(V={TRIM_V}, alpha={TRIM_ALPHA_DEG}deg), throttle(L=R)={TRIM_THROTTLE}")
    for surf, jn in joints_of_interest:
        log(f"  {jn}: tail_mean={math.degrees(tail_mean[jn]):+.4f}deg "
            f"residual={residual_deg[jn]:.4f}deg tail_noise(pp)={math.degrees(tail_noise[jn]):.5f}deg "
            f"tail_|rate|_max={tail_rate_absmax[jn]:.6f} rad/s "
            f"max_per_tick_jump={math.degrees(max_jump[jn]):.5f}deg "
            f"(limit {math.degrees(max_step_allowed):.5f}deg) overshoot={overshoot_deg[jn]:.4f}deg")
    log(f"{delta_label} = {computed_delta_deg:+.4f}deg (commanded {commanded_deg_signed:+.1f}deg physical; "
        f"expected sign = aero-convention {expected_delta_deg_signed:+.1f}deg)")
    log(f"target_clamp_active ever set: {any_target_clamp} (expected False, well within +/-45deg mechanical limit)")
    log(f"effort_clamp_active in tail (last 30 diag msgs): {any_effort_clamp_tail} (expected False once settled - "
        f"zero opposing hinge load per actuator_v1_config.yaml's no_opposing_aero_torque_caveat)")
    if aero_diag_latest:
        log(f"Live aero diagnostics (latest): CL={aero_diag_latest['CL']:.5f} CD={aero_diag_latest['CD']:.5f} "
            f"CY={aero_diag_latest['CY']:.5f} Cl={aero_diag_latest['Cl']:.5f} Cm={aero_diag_latest['Cm']:.5f} "
            f"Cn={aero_diag_latest['Cn']:.5f}")

    overall = bool(smooth_ok and overshoot_ok and settle_ok and no_chatter_ok and sign_ok
                   and not state["any_nan"])
    log(f"smooth_ok={smooth_ok} overshoot_ok={overshoot_ok} settle_ok={settle_ok} "
        f"no_chatter_ok={no_chatter_ok} sign_ok={sign_ok} finite_ok={not state['any_nan']}")
    if raw_timeseries_decimated is not None:
        log(f"raw_timeseries_decimated captured for {case_name}: "
            f"{raw_timeseries_decimated['n_samples']} samples (every {DECIMATE_STEPS} ticks = "
            f"{DECIMATE_STEPS * ACT.STEP * 1000:.0f}ms resolution), saved to result.json")
    log(f"{case_name}: {'PASS' if overall else 'FAIL'}\n")

    return dict(case=case_name, commands_rad=commands_rad,
                tail_mean_deg={jn: math.degrees(v) for jn, v in tail_mean.items()},
                residual_deg=residual_deg, tail_noise_deg={jn: math.degrees(v) for jn, v in tail_noise.items()},
                tail_rate_absmax=tail_rate_absmax, max_jump_deg={jn: math.degrees(v) for jn, v in max_jump.items()},
                max_step_allowed_deg=math.degrees(max_step_allowed), overshoot_deg=overshoot_deg,
                computed_delta_deg=computed_delta_deg, commanded_deg_signed=commanded_deg_signed,
                expected_delta_deg_signed=expected_delta_deg_signed, any_target_clamp=any_target_clamp,
                any_effort_clamp_tail=any_effort_clamp_tail, aero_diag_latest=aero_diag_latest,
                smooth_ok=smooth_ok, overshoot_ok=overshoot_ok, settle_ok=settle_ok,
                no_chatter_ok=no_chatter_ok, sign_ok=sign_ok, any_nan=state["any_nan"],
                raw_timeseries_decimated=raw_timeseries_decimated,
                pass_fail="PASS" if overall else "FAIL")


def run():
    log_lines = []

    def log(msg):
        print(msg, flush=True)
        log_lines.append(msg)

    log("FALCON V2 - ACTUATOR_SERVO_MODEL_V1 live flight-load test (gazebo-testing, 2026-08-23)")
    log(f"Trim flow region: V={TRIM_V} m/s, alpha={TRIM_ALPHA_DEG}deg (u={TRIM_U:.5f}, w={TRIM_W:.5f}), "
        f"throttle(L=R)={TRIM_THROTTLE} - reused read-only from POWERED_TRIM_AND_FREE_FLIGHT_VALIDATION.")
    log("Actuator commands swept +/-5deg / +/-10deg from a neutral baseline (see module docstring for rationale).")
    log(f"actuator_v1_config.yaml (read-only): max_rate_rad_s={ACFG['max_rate_rad_s']} "
        f"({math.degrees(ACFG['max_rate_rad_s']):.1f} deg/s), max_effort_nm={ACFG['max_effort_nm']}, "
        f"min/max_angle_rad={ACFG['min_angle_rad']:.4f}/{ACFG['max_angle_rad']:.4f}\n")

    results = {}
    for mag_deg in (5.0, 10.0):
        for sign, sign_label in ((1, "+"), (-1, "-")):
            deg = sign * mag_deg
            rad = math.radians(deg)

            name = f"ELEVATOR_SYMMETRIC_{sign_label}{mag_deg:.0f}DEG"
            results[name] = run_case(
                log, name, dict(left_elevator=rad, right_elevator=rad),
                [("left_elevator", "left_elevator_joint"), ("right_elevator", "right_elevator_joint")],
                lambda th: CFG.elevatorSign * 0.5 * (th["left_elevator"] + th["right_elevator"]),
                "delta_e_aero", CFG.elevatorSign * deg, deg)

            name = f"AILERON_DIFFERENTIAL_{sign_label}{mag_deg:.0f}DEG"
            results[name] = run_case(
                log, name, dict(left_aileron=-rad, right_aileron=rad),
                [("left_aileron", "left_aileron_joint"), ("right_aileron", "right_aileron_joint")],
                lambda th: CFG.aileronSign * 0.5 * (th["right_aileron"] - th["left_aileron"]),
                "delta_a_aero", CFG.aileronSign * deg, deg)

            name = f"RUDDER_{sign_label}{mag_deg:.0f}DEG"
            results[name] = run_case(
                log, name, dict(rudder=rad),
                [("rudder", "rudder_joint")],
                lambda th: CFG.rudderSign * th["rudder"],
                "delta_r_aero", CFG.rudderSign * deg, deg)

    overall = all(r["pass_fail"] == "PASS" for r in results.values())

    log("=" * 78)
    log("SUMMARY - flight-load test, all 12 cases (3 surface classes x 2 signs x 2 magnitudes)")
    log("=" * 78)
    for name, r in results.items():
        log(f"{name}: {r['pass_fail']}")
    log(f"\nOVERALL: {'PASS' if overall else 'FAIL'}")

    with open(f"{RESULTS_DIR}/actuator_flight_load_result.json", "w") as f:
        json.dump(dict(trim_condition=dict(V=TRIM_V, alpha_deg=TRIM_ALPHA_DEG, u=TRIM_U, w=TRIM_W,
                                            throttle=TRIM_THROTTLE),
                        cases=results, overall=overall), f, indent=2,
                   default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/actuator_flight_load_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

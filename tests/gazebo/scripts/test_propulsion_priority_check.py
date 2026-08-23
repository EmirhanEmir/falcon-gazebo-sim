#!/usr/bin/env python3
"""
FALCON V2 - PROPULSION_V1_IMPLEMENTATION, PRIORITY CHECK (gazebo-testing,
2026-08-23).

Investigates the specific, plausible duplicated-force risk `propulsion`
self-flagged: PropulsionSystem.cc's StepMotor() BOTH (a) applies an
explicit, analytically-computed reaction torque (Q_reaction = -Q_prop) to
base_link via Link.AddWorldWrench() at the real hub, AND (b) separately
drives left_prop_joint/right_prop_joint's angular velocity every tick via
Joint.SetVelocity() (to keep the visual mesh in sync with the plugin's own
integrated omega state). Concern: gz-sim's constraint solver may need to
apply its OWN internal joint torque to force the prop link (real, nonzero
SDF inertia I=2.7349e-4 kg*m^2) to track that commanded velocity, and that
internal constraint torque could ALSO react onto base_link through the
joint - i.e. the reaction torque could be counted twice.

METHOD (per task spec): command a steady throttle on ONE motor only (left
=0.6, right=0.0), hold base_link COMPLETELY still (all 6 DOF, via
aero_lib.hold_step()) while the rotor ODE settles to equilibrium
(domega/dt~0, Q_motor~=Q_prop), pin the 5 control-surface joints (never the
2 prop joints - propulsion_lib.pin_control_surface_joints()), then RELEASE
ONLY the roll (body X) rotational DOF for a short window while continuing
to actively hold the other 5 DOF (pitch, yaw, and all 3 linear). Because
the thrust force is purely along the shared body +X axis and both hubs'
Y/Z offsets only produce PITCH/YAW moments (r x F has zero X-component for
F=(T,0,0) at any r), base_link's real, live-measured roll angular
acceleration during the release window is driven ENTIRELY by whatever
reaction-torque mechanism(s) actually act on base_link about X - i.e. this
isolates exactly the quantity in question. Ixz cross-coupling
(base_link's inertia tensor has ixz=0.01485, non-zero) is analytically
negligible here (correction factor Ixz^2/(Ixx*Izz) ~ 0.03%, see this
script's own derivation printed in the log) and pitch/yaw are actively held
at 0 throughout the release window regardless.

Compares:
  measured Mx = Ixx_base_link * (linear-regression slope of p(t) over the
    release window)
against:
  expected_single_Mx = rotation_left_sign*(-Q_prop_left_signed) +
    rotation_right_sign*(-Q_prop_right_signed), using the diagnostics
    topic's own live Q_prop values (averaged over the release window) and
    propulsion_v1_config.yaml's own rotation-sign block (loaded fresh, read
    -only).

If measured/expected_single is close to 1.0 (tolerance below), no
duplication. If close to 2.0, this is a duplication bug and is reported to
`propulsion` verbatim - not fixed here.

No aircraft physics parameter is modified anywhere in this script.
"""
import json
import math
import sys

import aero_lib as AL
import propulsion_lib as PL

PL.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
CFG = PL.load_config()

LEFT_THROTTLE = 0.6
RIGHT_THROTTLE = 0.0
SETTLE_STEPS = 5000    # 5.0 s @ 1ms - generous margin for domega/dt -> ~0
RELEASE_STEPS = 150    # 0.15 s @ 1ms - short window, negligible RPM drift
KP_LIN = 150.0
KP_ANG = 150.0


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    model_e = world.model_by_name(ecm, "falcon_v2")
    return sim.Model(model_e)


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    log("=" * 78)
    log("PRIORITY CHECK: duplicated reaction-torque test (SetVelocity constraint")
    log("torque vs explicit AddWorldWrench reaction torque)")
    log("=" * 78)
    log(f"Config (fresh load, propulsion_v1_config.yaml): rotation.left_sign="
        f"{CFG.rotation_left_sign} rotation.right_sign={CFG.rotation_right_sign} "
        f"i_rotor_kg_m2={CFG.i_rotor_kg_m2} thrust_axis_body={CFG.thrust_axis_body}")
    log(f"Command: left_throttle={LEFT_THROTTLE}, right_throttle={RIGHT_THROTTLE}")
    log(f"Phase 1 (settle, all 6 DOF held at 0): {SETTLE_STEPS} steps "
        f"({SETTLE_STEPS*PL.STEP:.3f} s)")
    log(f"Phase 2 (release ROLL ONLY, pitch/yaw/linear still held at 0): "
        f"{RELEASE_STEPS} steps ({RELEASE_STEPS*PL.STEP:.3f} s)")

    diag = PL.DiagSubscriber()
    thr = PL.ThrottleCommander()
    thr.set(left=LEFT_THROTTLE, right=RIGHT_THROTTLE)

    state = {"n": 0, "phase": 1, "inertia": None,
             "settle_omega_left": [], "settle_omega_right": [],
             "release_p": [], "release_t": [],
             "release_qprop_left": [], "release_qprop_right": []}

    def on_pre(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        thr.tick()
        PL.pin_control_surface_joints(model, ecm, sim)

        if state["inertia"] is None and state["n"] > 5:
            state["inertia"] = AL.read_base_link_inertia(model, ecm, sim)
        i_diag = ((state["inertia"]["ixx"], state["inertia"]["iyy"], state["inertia"]["izz"])
                  if state["inertia"] else (0.7284, 0.2507, 0.9523))

        if state["phase"] == 1:
            AL.hold_step(base, ecm, 5.9348, i_diag,
                         gm.Vector3d(0, 0, 0), gm.Vector3d(0, 0, 0),
                         kp_lin=KP_LIN, kp_ang=KP_ANG, ang_axis_mask=(True, True, True))
        else:
            # Release ROLL (x) only; keep pitch/yaw + all linear DOF actively held.
            AL.hold_step(base, ecm, 5.9348, i_diag,
                         gm.Vector3d(0, 0, 0), gm.Vector3d(0, 0, 0),
                         kp_lin=KP_LIN, kp_ang=KP_ANG, ang_axis_mask=(False, True, True))

    def on_post(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        wpose = base.world_pose(ecm)
        av = base.world_angular_velocity(ecm)
        n = state["n"]

        if wpose is not None and av is not None:
            rot = wpose.rot()
            av_b = rot.rotate_vector_reverse(av)
            if state["phase"] == 2:
                state["release_p"].append(av_b.x())
                state["release_t"].append(n * PL.STEP)

        d = diag.latest()
        if d is not None:
            if state["phase"] == 1 and n > SETTLE_STEPS - 500:
                state["settle_omega_left"].append(d["left"]["omega_rad_s"])
                state["settle_omega_right"].append(d["right"]["omega_rad_s"])
            if state["phase"] == 2:
                state["release_qprop_left"].append(d["left"]["Q_prop_Nm"])
                state["release_qprop_right"].append(d["right"]["Q_prop_Nm"])

        state["n"] += 1
        if state["phase"] == 1 and state["n"] >= SETTLE_STEPS:
            state["phase"] = 2
            state["n"] = 0

    fixture = sim.TestFixture(PL.WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, SETTLE_STEPS + RELEASE_STEPS + 10, False)

    log(f"\nDiagnostics messages received total: {diag.count()}")
    inertia = state["inertia"]
    log(f"base_link inertia (queried live): {inertia}")
    ixx = inertia["ixx"]
    ixz_correction_pct = 100.0 * (inertia.get("ixz", 0.0) ** 2) / (inertia["ixx"] * inertia["izz"])
    log(f"Ixz cross-coupling correction estimate: Ixz^2/(Ixx*Izz) = {ixz_correction_pct:.4f}% "
        f"(negligible for this test's purpose)")

    settle_omega_left = state["settle_omega_left"]
    settle_omega_right = state["settle_omega_right"]
    if settle_omega_left:
        omega_l_span = max(settle_omega_left) - min(settle_omega_left)
        log(f"Left motor omega over final 500 diag samples of settle phase: "
            f"min={min(settle_omega_left):.6f} max={max(settle_omega_left):.6f} "
            f"span={omega_l_span:.6f} rad/s (near-zero span -> settled)")
    else:
        omega_l_span = float("nan")

    qprop_left_vals = state["release_qprop_left"]
    qprop_right_vals = state["release_qprop_right"]
    qprop_left_avg = sum(qprop_left_vals) / len(qprop_left_vals) if qprop_left_vals else float("nan")
    qprop_right_avg = sum(qprop_right_vals) / len(qprop_right_vals) if qprop_right_vals else float("nan")
    log(f"\nQ_prop (signed, own-frame) during release window: "
        f"left avg={qprop_left_avg:.6f} N*m (n={len(qprop_left_vals)}), "
        f"right avg={qprop_right_avg:.6f} N*m (n={len(qprop_right_vals)})")

    expected_single_mx = (CFG.rotation_left_sign * (-qprop_left_avg) +
                           CFG.rotation_right_sign * (-qprop_right_avg))
    log(f"Analytically expected (SINGLE reaction-torque source) Mx = "
        f"left_sign*(-Qprop_left) + right_sign*(-Qprop_right) = "
        f"{CFG.rotation_left_sign}*({-qprop_left_avg:.6f}) + "
        f"{CFG.rotation_right_sign}*({-qprop_right_avg:.6f}) = {expected_single_mx:.6f} N*m")

    p_series = state["release_p"]
    t_series = state["release_t"]
    log(f"\nRelease-window p(t) samples: n={len(p_series)}")
    if len(p_series) >= 5:
        n_pts = len(p_series)
        t_mean = sum(t_series) / n_pts
        p_mean = sum(p_series) / n_pts
        num = sum((t_series[i] - t_mean) * (p_series[i] - p_mean) for i in range(n_pts))
        den = sum((t_series[i] - t_mean) ** 2 for i in range(n_pts))
        slope = num / den if den > 0 else float("nan")  # dp/dt, rad/s^2
        p_start, p_end = p_series[0], p_series[-1]
        endpoint_slope = (p_end - p_start) / (t_series[-1] - t_series[0])
        log(f"p(t=0)={p_start:.8f} rad/s, p(t={t_series[-1]:.3f})={p_end:.8f} rad/s")
        log(f"Linear-regression slope dp/dt = {slope:.6f} rad/s^2 "
            f"(endpoint-difference cross-check: {endpoint_slope:.6f} rad/s^2)")
        measured_mx = ixx * slope
        log(f"Measured Mx = Ixx * dp/dt = {ixx:.4f} * {slope:.6f} = {measured_mx:.6f} N*m")

        ratio = measured_mx / expected_single_mx if expected_single_mx != 0 else float("nan")
        log(f"\nRATIO measured/expected_single = {ratio:.4f}")

        near_one = abs(ratio - 1.0) <= 0.25
        near_two = abs(ratio - 2.0) <= 0.35
        if near_one and not near_two:
            verdict = "NO_DUPLICATION"
        elif near_two and not near_one:
            verdict = "DUPLICATION_CONFIRMED"
        else:
            verdict = "INCONCLUSIVE"
        log(f"\nVERDICT: {verdict} (ratio={ratio:.4f}; ~1.0 => single reaction-torque source only, "
            f"~2.0 => joint-constraint torque duplicates the explicit AddWorldWrench reaction torque)")
    else:
        measured_mx = float("nan")
        ratio = float("nan")
        verdict = "INCONCLUSIVE_INSUFFICIENT_DATA"
        log("Insufficient release-window samples to compute a slope.")

    result = dict(
        left_throttle=LEFT_THROTTLE, right_throttle=RIGHT_THROTTLE,
        settle_steps=SETTLE_STEPS, release_steps=RELEASE_STEPS,
        inertia=inertia, ixz_correction_pct=ixz_correction_pct,
        settle_omega_left_span=omega_l_span,
        qprop_left_avg=qprop_left_avg, qprop_right_avg=qprop_right_avg,
        expected_single_mx=expected_single_mx,
        measured_mx=measured_mx, ratio=ratio, verdict=verdict,
        rotation_left_sign=CFG.rotation_left_sign, rotation_right_sign=CFG.rotation_right_sign,
    )
    with open(f"{RESULTS_DIR}/propulsion_priority_check_result.json", "w") as f:
        json.dump(result, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/propulsion_priority_check_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return verdict


if __name__ == "__main__":
    v = run()
    sys.exit(0 if v == "NO_DUPLICATION" else 1)

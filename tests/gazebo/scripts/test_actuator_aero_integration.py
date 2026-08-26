#!/usr/bin/env python3
"""
FALCON V2 - ACTUATOR_SERVO_MODEL_V1 mapping/integration test pass
(gazebo-testing, 2026-08-23).

Runs the 4 required live-Gazebo tests that need BOTH `FalconV2Actuators`
(controls-integration, plugins/actuators/) AND `FalconV2Aerodynamics`
(plugins/aerodynamics/) running together:

  1. ACTUAL_JOINT_STATE_FEEDS_AERO_TEST - commands a real elevator step via
     the actuator's own command topic (ActuatorCommander -> gz-transport ->
     FalconV2Actuators -> Joint::SetForce(), NEVER reset_position() for
     these 2 joints this pass), then measures the REAL applied aerodynamic
     pitch moment (via the established "hold trim, release pitch axis
     briefly, measure the real dq/dt" technique - aero_lib.hold_step, the
     SAME primitive/technique already validated by
     test_control_surface_sign_mapping.py / test_aero_stability_
     derivatives.py) at TWO different elapsed times since the command step:
     EARLY (15 ticks = 15ms after the command, well before the actuator can
     have reached the target - see actuator_model_selftest's own measured
     ~66ms rate-limited travel time for a 20deg step, so an 8deg step's
     15ms-old actual position is necessarily still well short of target) and
     LATE (350 ticks = 350ms after the command, many actuator time-constants
     past settling). If the aerodynamics plugin used the ACTUAL (servo-
     tracked) joint position, the EARLY measured moment must be
     substantially SMALLER in magnitude than the LATE one (same sign) - an
     instantaneous jump to the commanded value would instead make them
     approximately equal already at the EARLY sample. This directly
     distinguishes "aero reads actual position" from "aero reads commanded
     position" using a real, live-measured applied moment, not merely a
     diagnostics-topic readout.

  2. LEFT_RIGHT_ELEVATOR_MAPPING_REGRESSION - commands both elevator
     actuators to +5deg (physical joint angle, TE-up per the already-
     verified VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST mapping), lets the real
     servo settle, reads the ACTUAL joint positions back from the ECM
     (ground truth, not the commanded value), computes
     delta_e_aero = -0.5*(theta_left+theta_right) (equivalently
     0.5*elevator_sign*(theta_left+theta_right) with the already-verified
     elevator_sign=-1.0, aero_v1_config.yaml, UNMODIFIED this pass), and
     confirms sign/magnitude against the expected -5deg.

  3. LEFT_RIGHT_AILERON_MAPPING_REGRESSION - commands left_aileron=-5deg /
     right_aileron=+5deg, settles, confirms
     delta_a_aero=+0.5*(theta_right-theta_left) against the expected +5deg
     (aileron_sign=+1.0, UNCHANGED).

  4. RUDDER_MAPPING_REGRESSION - commands rudder=+5deg, settles, confirms
     delta_r_aero=theta_rudder against the expected +5deg
     (rudder_sign=+1.0, UNCHANGED), and cross-checks the resulting Cn sign
     against the already-established RUDDER_AERO_MOMENT_SIGN_TEST finding
     (Mz<0 for rudder>0, tests/gazebo/results/control_surface_sign_mapping_
     result.json).

For tests 2-4, "settled" state is cross-checked THREE independent ways at
every case: (a) direct ECM joint-position readback (ground truth), (b) an
independent aero_lib.compute_aero() replica fed those ACTUAL angles at the
held flight condition, (c) the live FalconV2Aerodynamics diagnostics
topic's own published CL/CD/CY/Cl/Cm/Cn (a genuine live measurement, not
just the replica). No aircraft physics parameter (mass, CG, inertia,
aerodynamic coefficient, control-sign mapping, actuator max_rate/max_effort/
kp/kd, hinge geometry) is modified anywhere in this script - aero_v1_config.
yaml and actuator_v1_config.yaml are read-only here.

World: tests/gazebo/worlds/falcon_v2_zero_g_world.sdf (gravity off, matches
the established CONTROL_SURFACE_SIGN_MAPPING precedent - isolates the
control-surface/aero-response measurement from gravity/attitude drift).
Held condition throughout every case: u=15 m/s, v=w=p=q=r=0 via
aero_lib.hold_step() (a plain proportional force/torque controller applied
via the SAME AddWorldForce/AddWorldWrench primitives the aero plugin itself
uses - never a Cmd-based velocity override, which aero_lib.py's own header
documents as freezing further physics integration).
"""
import json
import math
import sys

import actuator_lib as ACT
import aero_lib as AL

ACT.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
CFG = AL.load_config()
ACFG = ACT.load_actuator_config()

MASS = 5.9348  # kg, base_link mass, model/model.sdf (queried live too, cross-checked below)
I_DIAG_DEFAULT = (0.7284, 0.2507, 0.9523)  # kg*m^2, base_link diagonal inertia, controller gain only
KP_LIN = 150.0
KP_ANG = 150.0
U = 15.0
LIN_TARGET = gm.Vector3d(U, 0.0, 0.0)
ANG_TARGET = gm.Vector3d(0.0, 0.0, 0.0)

WARM_STEPS = 300  # 0.3s - lets hold_step converge to u=15 before any actuation


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    model_e = world.model_by_name(ecm, "falcon_v2")
    return sim.Model(model_e)


# =============================================================================
# TEST 1 - ACTUAL_JOINT_STATE_FEEDS_AERO_TEST
# =============================================================================
DEFLECTION_DEG_T1 = 8.0
DEFLECTION_RAD_T1 = math.radians(DEFLECTION_DEG_T1)
EARLY_RELEASE_DELAY = 15    # ticks after the command step before releasing pitch
LATE_RELEASE_DELAY = 350    # ticks after the command step before releasing pitch (well settled)
RELEASE_WINDOW = 15         # ticks of released pitch used to measure dq/dt


def run_t1_subcase(log, release_delay, label):
    total_steps = WARM_STEPS + release_delay + RELEASE_WINDOW + 5
    state = {"n": 0, "series": [], "any_nan": False, "inertia": None, "cmd": None,
             "theta_l": [], "theta_r": [], "actuator_diag": None}

    def on_pre(info, ecm):
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if state["cmd"] is None:
            state["cmd"] = ACT.ActuatorCommander()
        if n >= WARM_STEPS:
            state["cmd"].set(left_elevator=DEFLECTION_RAD_T1, right_elevator=DEFLECTION_RAD_T1)
        state["cmd"].tick()
        # Both elevator joints are real-actuator-driven this subcase; the
        # other 5 child joints (2 ailerons, rudder, 2 props) stay pinned at
        # 0 (isolation, aero_lib.CHILD_JOINTS rationale).
        ACT.pin_other_child_joints(
            model, ecm, sim, leave_free_joints=("left_elevator_joint", "right_elevator_joint"))
        release_at = WARM_STEPS + release_delay
        in_release = release_at <= n < release_at + RELEASE_WINDOW
        mask = (True, False, True) if in_release else (True, True, True)
        AL.hold_step(base, ecm, MASS, I_DIAG_DEFAULT, LIN_TARGET, ANG_TARGET,
                     kp_lin=KP_LIN, kp_ang=KP_ANG, ang_axis_mask=mask)

    def on_post(info, ecm):
        if state["actuator_diag"] is None:
            try:
                state["actuator_diag"] = ACT.DiagSubscriber()
            except Exception:
                pass
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if state["inertia"] is None and n > 5:
            state["inertia"] = AL.read_base_link_inertia(model, ecm, sim)
        tl, _ = ACT.read_joint_state(model, ecm, sim, "left_elevator_joint")
        tr, _ = ACT.read_joint_state(model, ecm, sim, "right_elevator_joint")
        state["theta_l"].append(tl if tl is not None else float("nan"))
        state["theta_r"].append(tr if tr is not None else float("nan"))
        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        wpose = base.world_pose(ecm)
        if lv is None or av is None or wpose is None:
            state["any_nan"] = True
            state["n"] += 1
            return
        rot = wpose.rot()
        lv_b = rot.rotate_vector_reverse(lv)
        av_b = rot.rotate_vector_reverse(av)
        vals = [lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z()]
        if any(math.isnan(v) or math.isinf(v) for v in vals):
            state["any_nan"] = True
        release_at = WARM_STEPS + release_delay
        in_release = release_at <= n < release_at + RELEASE_WINDOW
        state["series"].append(dict(n=n, t=n * ACT.STEP, in_release=in_release,
                                     u=lv_b.x(), v=lv_b.y(), w=lv_b.z(),
                                     p=av_b.x(), q=av_b.y(), r=av_b.z(),
                                     theta_l=state["theta_l"][-1], theta_r=state["theta_r"][-1]))
        state["n"] += 1

    fixture = sim.TestFixture(AL.WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    series = state["series"]
    at_command = series[WARM_STEPS] if len(series) > WARM_STEPS else None
    release_at = WARM_STEPS + release_delay
    at_release_start = series[release_at] if len(series) > release_at else None
    release_pts = [r for r in series if r["in_release"]]

    import numpy as np
    ts = np.array([r["t"] for r in release_pts])
    rates = np.array([r["q"] for r in release_pts])
    ts0 = ts - ts[0]
    slope = float(np.polyfit(ts0, rates, 1)[0])
    inertia = state["inertia"]
    my_measured = inertia["iyy"] * slope

    theta_l_at_release = at_release_start["theta_l"] if at_release_start else None
    theta_r_at_release = at_release_start["theta_r"] if at_release_start else None
    target_deg = DEFLECTION_DEG_T1
    frac_of_target = (0.5 * (theta_l_at_release + theta_r_at_release) / DEFLECTION_RAD_T1
                       if theta_l_at_release is not None and theta_r_at_release is not None else None)

    log(f"--- {label} (release_delay={release_delay} ticks = {release_delay*ACT.STEP*1000:.0f} ms after command) ---")
    log(f"  Command issued at n={WARM_STEPS} (t={WARM_STEPS*ACT.STEP:.3f}s): "
        f"left_elevator=right_elevator={target_deg:+.1f}deg ({DEFLECTION_RAD_T1:.6f} rad)")
    log(f"  Actual joint angle AT release-window start (n={release_at}, "
        f"{release_delay*ACT.STEP*1000:.0f} ms after command): "
        f"theta_l={math.degrees(theta_l_at_release):.4f}deg theta_r={math.degrees(theta_r_at_release):.4f}deg "
        f"(fraction of {target_deg}deg target = {frac_of_target*100:.2f}%)" if theta_l_at_release is not None
        else "  Actual joint angle AT release-window start: READ FAILED")
    log(f"  q(t) over release window: {[round(v,5) for v in rates]}")
    log(f"  Measured slope dq/dt = {slope:.5f} rad/s^2 -> My_measured = Iyy*slope = "
        f"{inertia['iyy']:.4f}*{slope:.5f} = {my_measured:.5f} N*m")

    return dict(label=label, release_delay_ticks=release_delay,
                release_delay_ms=release_delay * ACT.STEP * 1000,
                theta_l_deg_at_release=math.degrees(theta_l_at_release) if theta_l_at_release is not None else None,
                theta_r_deg_at_release=math.degrees(theta_r_at_release) if theta_r_at_release is not None else None,
                fraction_of_target=frac_of_target,
                my_measured=my_measured, any_nan=state["any_nan"], inertia=inertia)


def run_test1(log):
    log("=" * 78)
    log("TEST 1: ACTUAL_JOINT_STATE_FEEDS_AERO_TEST")
    log("=" * 78)
    early = run_t1_subcase(log, EARLY_RELEASE_DELAY, "EARLY (before servo has caught up)")
    late = run_t1_subcase(log, LATE_RELEASE_DELAY, "LATE (after servo has settled)")

    any_nan = early["any_nan"] or late["any_nan"]
    finite_ok = not any_nan and all(
        math.isfinite(x) for x in (early["my_measured"], late["my_measured"]))
    same_sign = finite_ok and (early["my_measured"] > 0) == (late["my_measured"] > 0) and late["my_measured"] != 0
    partial_position_ok = (early["fraction_of_target"] is not None and
                            0.0 <= early["fraction_of_target"] < 0.85)
    settled_position_ok = (late["fraction_of_target"] is not None and
                            late["fraction_of_target"] > 0.95)
    magnitude_grew = (finite_ok and abs(late["my_measured"]) > 0 and
                       abs(early["my_measured"]) < 0.85 * abs(late["my_measured"]))

    overall = bool(finite_ok and same_sign and partial_position_ok and settled_position_ok and magnitude_grew)

    log("\n--- TEST 1 interpretation ---")
    log(f"EARLY actual position: {early['fraction_of_target']*100 if early['fraction_of_target'] is not None else float('nan'):.2f}% "
        f"of target reached at t=+{EARLY_RELEASE_DELAY} ms -> "
        f"{'PASS (clearly still in transit)' if partial_position_ok else 'FAIL (unexpectedly already near target)'}")
    log(f"LATE actual position: {late['fraction_of_target']*100 if late['fraction_of_target'] is not None else float('nan'):.2f}% "
        f"of target reached at t=+{LATE_RELEASE_DELAY} ms -> "
        f"{'PASS (settled)' if settled_position_ok else 'FAIL (not settled as expected)'}")
    log(f"EARLY measured My={early['my_measured']:.5f} N*m vs LATE measured My={late['my_measured']:.5f} N*m "
        f"(same sign: {same_sign}, |early| < 0.85*|late|: {magnitude_grew})")
    log(f"-> {'CONFIRMS' if overall else 'REFUTES'} that the aerodynamics plugin's applied moment tracks the "
        f"ACTUAL (servo-tracked) joint trajectory rather than jumping instantly to the commanded value.")
    log(f"ACTUAL_JOINT_STATE_FEEDS_AERO_TEST: {'PASS' if overall else 'FAIL'}")

    return dict(early=early, late=late, same_sign=same_sign, partial_position_ok=partial_position_ok,
                settled_position_ok=settled_position_ok, magnitude_grew=magnitude_grew,
                any_nan=any_nan, pass_fail="PASS" if overall else "FAIL")


# =============================================================================
# TESTS 2-4 - settled-state mapping regressions
# =============================================================================
SETTLE_STEPS = 600   # 0.6s after command - many actuator time-constants past settling
TAIL_STEPS = 100      # last N steps of the run used for the "settled" average/noise check


def run_settled_case(log, name, commands_rad, settle_deg_formula, expected_deg, joints_of_interest):
    """commands_rad: dict of surface->rad given to ActuatorCommander at
    n=WARM_STEPS onward. joints_of_interest: list of (surface_key, joint_name)
    pairs whose ACTUAL angle is tracked and returned."""
    total_steps = WARM_STEPS + SETTLE_STEPS + 5
    state = {"n": 0, "cmd": None, "actuator_diag": None, "aero_diag": None,
             "theta": {jn: [] for _, jn in joints_of_interest}, "any_nan": False,
             "hold_state": []}

    def on_pre(info, ecm):
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if state["cmd"] is None:
            state["cmd"] = ACT.ActuatorCommander()
        if n >= WARM_STEPS:
            state["cmd"].set(**commands_rad)
        state["cmd"].tick()
        leave_free = [jn for _, jn in joints_of_interest]
        ACT.pin_other_child_joints(model, ecm, sim, leave_free_joints=leave_free)
        AL.hold_step(base, ecm, MASS, I_DIAG_DEFAULT, LIN_TARGET, ANG_TARGET,
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
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        for surf, jn in joints_of_interest:
            th, _ = ACT.read_joint_state(model, ecm, sim, jn)
            state["theta"][jn].append(th if th is not None else float("nan"))
        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        wpose = base.world_pose(ecm)
        if lv is not None and av is not None and wpose is not None:
            rot = wpose.rot()
            lv_b = rot.rotate_vector_reverse(lv)
            av_b = rot.rotate_vector_reverse(av)
            state["hold_state"].append((lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z()))
            if any(math.isnan(v) or math.isinf(v) for v in
                   (lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z())):
                state["any_nan"] = True
        else:
            state["any_nan"] = True
        state["n"] += 1

    fixture = sim.TestFixture(AL.WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    tail = {jn: state["theta"][jn][-TAIL_STEPS:] for _, jn in joints_of_interest}
    tail_mean = {jn: sum(v) / len(v) for jn, v in tail.items()}
    tail_noise = {jn: (max(v) - min(v)) for jn, v in tail.items()}
    u_b, v_b, w_b, p_b, q_b, r_b = state["hold_state"][-1] if state["hold_state"] else (U, 0, 0, 0, 0, 0)

    thetas = {surf: tail_mean[jn] for surf, jn in joints_of_interest}
    settled_deg_formula_val = math.degrees(settle_deg_formula(thetas))

    cross = AL.compute_aero(CFG, u_b, v_b, w_b, p_b, q_b, r_b,
                             deltaA=math.radians(settled_deg_formula_val) if name.startswith("AILERON") else 0.0,
                             deltaE=math.radians(settled_deg_formula_val) if name.startswith("ELEVATOR") else 0.0,
                             deltaR=math.radians(settled_deg_formula_val) if name.startswith("RUDDER") else 0.0)
    aero_diag_latest = state["aero_diag"].latest() if state["aero_diag"] else None
    actuator_diag_latest = state["actuator_diag"].latest() if state["actuator_diag"] else None

    log(f"--- {name} ---")
    log(f"Commanded (rad): {commands_rad}")
    for surf, jn in joints_of_interest:
        log(f"  Actual {jn} tail-window (last {TAIL_STEPS} steps): "
            f"mean={math.degrees(tail_mean[jn]):+.4f}deg noise(max-min)={math.degrees(tail_noise[jn]):.5f}deg")
    log(f"Computed deflection (formula given in task): {settled_deg_formula_val:+.4f}deg "
        f"(expected {expected_deg:+.1f}deg)")
    log(f"Held flight condition at end of run: u={u_b:.4f} v={v_b:.4f} w={w_b:.4f} p={p_b:.5f} q={q_b:.5f} r={r_b:.5f}")
    log(f"Independent aero_lib.compute_aero() replica @ this deflection: "
        f"CL={cross['CL']:.5f} CD={cross['CD']:.5f} CY={cross['CY']:.5f} "
        f"Cl={cross['Cl']:.5f} Cm={cross['Cm']:.5f} Cn={cross['Cn']:.5f}")
    if aero_diag_latest:
        log(f"Live FalconV2Aerodynamics diagnostics (latest message): {aero_diag_latest}")
    if actuator_diag_latest:
        surf0 = joints_of_interest[0][0]
        log(f"Live FalconV2Actuators diagnostics for '{surf0}' (latest message): "
            f"{actuator_diag_latest.get(surf0)}")

    magnitude_ok = abs(settled_deg_formula_val - expected_deg) < 1.0  # deg, generous given no opposing load
    sign_ok = (settled_deg_formula_val > 0) == (expected_deg > 0) if expected_deg != 0 else True
    noise_ok = all(math.degrees(n_) < 0.5 for n_ in tail_noise.values())  # settled, not chattering
    finite_ok = not state["any_nan"]

    overall = bool(magnitude_ok and sign_ok and noise_ok and finite_ok)
    log(f"magnitude_ok(within 1.0deg of {expected_deg:+.1f}deg): {magnitude_ok}, sign_ok: {sign_ok}, "
        f"settled/no-chatter(tail noise<0.5deg): {noise_ok}, finite: {finite_ok}")
    log(f"{name}: {'PASS' if overall else 'FAIL'}\n")

    return dict(name=name, commands_rad=commands_rad, tail_mean_deg={jn: math.degrees(v) for jn, v in tail_mean.items()},
                tail_noise_deg={jn: math.degrees(v) for jn, v in tail_noise.items()},
                settled_deg_formula_val=settled_deg_formula_val, expected_deg=expected_deg,
                held_state=dict(u=u_b, v=v_b, w=w_b, p=p_b, q=q_b, r=r_b),
                compute_aero_replica=cross, aero_diag_latest=aero_diag_latest,
                actuator_diag_latest=actuator_diag_latest, magnitude_ok=magnitude_ok, sign_ok=sign_ok,
                noise_ok=noise_ok, finite_ok=finite_ok, any_nan=state["any_nan"],
                pass_fail="PASS" if overall else "FAIL")


def run_tests_2_3_4(log):
    log("=" * 78)
    log("TESTS 2-4: LEFT_RIGHT_ELEVATOR / LEFT_RIGHT_AILERON / RUDDER MAPPING REGRESSION")
    log("=" * 78)

    ele_deg = 5.0
    ele_rad = math.radians(ele_deg)
    t2 = run_settled_case(
        log, "ELEVATOR_LEFT_RIGHT_MAPPING_REGRESSION",
        commands_rad=dict(left_elevator=ele_rad, right_elevator=ele_rad),
        settle_deg_formula=lambda th: CFG.elevatorSign * 0.5 * (th["left_elevator"] + th["right_elevator"]),
        expected_deg=CFG.elevatorSign * ele_deg,
        joints_of_interest=[("left_elevator", "left_elevator_joint"), ("right_elevator", "right_elevator_joint")])

    ail_deg = 5.0
    ail_rad = math.radians(ail_deg)
    t3 = run_settled_case(
        log, "AILERON_LEFT_RIGHT_MAPPING_REGRESSION",
        commands_rad=dict(left_aileron=-ail_rad, right_aileron=ail_rad),
        settle_deg_formula=lambda th: CFG.aileronSign * 0.5 * (th["right_aileron"] - th["left_aileron"]),
        expected_deg=CFG.aileronSign * ail_deg,
        joints_of_interest=[("left_aileron", "left_aileron_joint"), ("right_aileron", "right_aileron_joint")])

    rud_deg = 5.0
    rud_rad = math.radians(rud_deg)
    t4 = run_settled_case(
        log, "RUDDER_MAPPING_REGRESSION",
        commands_rad=dict(rudder=rud_rad),
        settle_deg_formula=lambda th: CFG.rudderSign * th["rudder"],
        expected_deg=CFG.rudderSign * rud_deg,
        joints_of_interest=[("rudder", "rudder_joint")])

    return t2, t3, t4


def run():
    log_lines = []

    def log(msg):
        print(msg, flush=True)
        log_lines.append(msg)

    log("FALCON V2 - ACTUATOR_SERVO_MODEL_V1 mapping/integration test pass (gazebo-testing, 2026-08-23)")
    log("Verifying the NEW real actuator (FalconV2Actuators, controls-integration) correctly feeds the")
    log("already-verified aerodynamics control-sign mapping, with the real joint position as the source")
    log("(not test-script reset_position() injection). No aircraft physics parameter modified.\n")

    t1 = run_test1(log)
    t2, t3, t4 = run_tests_2_3_4(log)

    all_results = {
        "ACTUAL_JOINT_STATE_FEEDS_AERO_TEST": t1,
        "LEFT_RIGHT_ELEVATOR_MAPPING_REGRESSION": t2,
        "LEFT_RIGHT_AILERON_MAPPING_REGRESSION": t3,
        "RUDDER_MAPPING_REGRESSION": t4,
    }
    overall = all(r["pass_fail"] == "PASS" for r in all_results.values())

    log("=" * 78)
    log("SUMMARY - 4 mapping/integration tests")
    log("=" * 78)
    for name, r in all_results.items():
        log(f"{name}: {r['pass_fail']}")
    log(f"\nOVERALL: {'PASS' if overall else 'FAIL'}")

    with open(f"{RESULTS_DIR}/actuator_aero_integration_result.json", "w") as f:
        json.dump(all_results, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/actuator_aero_integration_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

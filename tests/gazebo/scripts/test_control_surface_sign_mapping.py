#!/usr/bin/env python3
"""
FALCON V2 - CONTROL_SURFACE_SIGN_MAPPING live-Gazebo verification pass
(gazebo-testing, 2026-08-22).

Executes the 9 tests `controls-integration` specified for this pass, to
independently verify (not just trust) their geometric determination of how
each control-surface joint's positive angle maps to physical trailing-edge
(TE) direction, and how that maps through aero_v1_config.yaml's
aileron_sign/rudder_sign to a real aerodynamic moment.

  PART A - kinematic TE-direction tests (pure geometry, no aero plugin
  needed, joint driven directly via Joint.reset_position()):
    1. LEFT_AILERON_JOINT_SIGN_TEST
    2. RIGHT_AILERON_JOINT_SIGN_TEST
    5. LEFT_ELEVATOR_JOINT_SIGN_TEST
    6. RIGHT_ELEVATOR_JOINT_SIGN_TEST
    7. RUDDER_JOINT_SIGN_TEST

  Method: for each surface, command the joint to -5deg/0deg/+5deg in turn
  (all other 4 control-surface joints + both prop joints pinned to 0 via
  aero_lib.pin_child_joints - the same isolation technique already validated
  reliable by test_aero_control_surfaces.py/test_control_joint_limits_and_
  motion.py), run a few physics steps for the pose graph to update, then
  read the target link's real world_pose(ecm) and transform the SAME local
  TE reference point `controls-integration` specified (that link's own
  collision-box center, local frame, taken directly from model/model.sdf's
  <inertial><pose>/<collision><pose> for that link, unmodified) into world
  frame: world_point = wpose.pos() + wpose.rot().rotate_vector(local_point).
  Aileron/elevator: compare world Z across the 3 angles (TE-up hypothesis
  means Z(+5) > Z(0) > Z(-5)). Rudder: compare world Y (TE-right hypothesis
  means Y(+5) < Y(0) < Y(-5), i.e. more negative = more to the right in
  FLU).

  PART B - dynamic aero-moment sign tests (real applied wrench via the live
  FalconV2Aerodynamics plugin, "hold steady condition then release one
  rotational axis and measure the real resulting angular acceleration"
  technique, identical to test_aero_control_surfaces.py/aero_lib.hold_step -
  re-run FRESH this pass, not reusing old log data, per task instruction):
    3. AILERON_DIFFERENTIAL_SIGN_TEST  (theta_left=-8deg, theta_right=+8deg,
       release roll axis, measure real Mx)
    4. AILERON_COMMON_MODE_REJECTION_TEST (theta_left=theta_right=+8deg,
       release roll axis, measure real Mx, expect much smaller |Mx| than
       test 3)
    8. AILERON_AERO_MOMENT_SIGN_TEST - explicitly the SAME measurement run
       as test 3 (task brief: "same as test 3 essentially"), reported here
       as its own named verdict against the aileron_sign=+1 hypothesis
       (Mx>0 expected), not a second independent run.
    9. RUDDER_AERO_MOMENT_SIGN_TEST (theta_rudder=+8deg, release yaw axis,
       measure real Mz, expect Mz<0)

Explicitly NOT run this pass (excluded by task instruction, pending the
elevator_sign config fix): ELEVATOR_SYMMETRIC_MAPPING_TEST,
ELEVATOR_AERO_MOMENT_SIGN_TEST.

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
control authority, motor thrust, or any joint/hinge geometry in model.sdf)
is modified by this script under any circumstance. This script only READS
joint/link state and drives joints via reset_position() for isolation/
testing purposes, exactly like the existing validated test suite.
"""
import json
import math
import sys

import aero_lib as AL

AL.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
CFG = AL.load_config()

# =============================================================================
# PART A - kinematic TE-direction tests
# =============================================================================

# TE reference local points, taken verbatim from model/model.sdf's
# <inertial><pose>/<collision><pose> for each link (that link's own
# collision-box center, local frame) - the exact points controls-integration
# cited in the task brief.
KINEMATIC_SURFACES = [
    dict(test="LEFT_AILERON_JOINT_SIGN_TEST", joint="left_aileron_joint",
         link="left_aileron", local_point=(-0.025962, 0.236107, 0.007807),
         check_axis="z", hypothesis="TE-up for +angle (Z higher at +5deg)"),
    dict(test="RIGHT_AILERON_JOINT_SIGN_TEST", joint="right_aileron_joint",
         link="right_aileron", local_point=(-0.025962, -0.236107, 0.007807),
         check_axis="z", hypothesis="SAME sign as left (TE-up for +angle, not mirrored)"),
    dict(test="LEFT_ELEVATOR_JOINT_SIGN_TEST", joint="left_elevator_joint",
         link="left_elevator", local_point=(-0.020211, 0.094700, -0.004031),
         check_axis="z", hypothesis="TE-up for +angle (Z higher at +5deg)"),
    dict(test="RIGHT_ELEVATOR_JOINT_SIGN_TEST", joint="right_elevator_joint",
         link="right_elevator", local_point=(-0.020211, -0.094700, -0.004031),
         check_axis="z", hypothesis="SAME direction as left (TE-up for +angle, not mirrored)"),
    dict(test="RUDDER_JOINT_SIGN_TEST", joint="rudder_joint",
         link="rudder", local_point=(-0.019247, 0.0, 0.084500),
         check_axis="y", hypothesis="TE toward -Y (right in FLU) for +angle (Y lower at +5deg)"),
]

KIN_ANGLES_DEG = [-5.0, 0.0, 5.0]
KIN_WARM_STEPS = 3
KIN_HOLD_STEPS = 8


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    model_e = world.model_by_name(ecm, "falcon_v2")
    return sim.Model(model_e)


def measure_te_world_point(joint_name, link_name, local_point_xyz, angle_rad):
    """Fresh TestFixture per (joint, angle): pin all 7 child joints via
    aero_lib.pin_child_joints with the target joint overridden to
    angle_rad, run warm+hold steps, then read the target link's real
    world_pose(ecm) and transform local_point_xyz into world frame."""
    fixture = sim.TestFixture(AL.WORLD_SDF)
    state = {"n": 0, "wpose": None, "readback": None, "any_none": False}

    def on_pre(info, ecm):
        model = get_model(ecm)
        AL.pin_child_joints(model, ecm, sim, positions={joint_name: angle_rad})

    def on_post(info, ecm):
        model = get_model(ecm)
        link = sim.Link(model.link_by_name(ecm, link_name))
        j = sim.Joint(model.joint_by_name(ecm, joint_name))
        j.enable_position_check(ecm, True)
        wpose = link.world_pose(ecm)
        pos = j.position(ecm)
        state["n"] += 1
        if state["n"] >= KIN_WARM_STEPS + KIN_HOLD_STEPS:
            if wpose is None or pos is None:
                state["any_none"] = True
            else:
                state["wpose"] = wpose
                state["readback"] = pos[0]

    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, KIN_WARM_STEPS + KIN_HOLD_STEPS + 2, False)

    wpose = state["wpose"]
    if wpose is None:
        return None
    local_vec = gm.Vector3d(*local_point_xyz)
    world_vec = wpose.pos() + wpose.rot().rotate_vector(local_vec)
    return dict(
        readback_rad=state["readback"],
        readback_deg=math.degrees(state["readback"]) if state["readback"] is not None else None,
        world_point=(world_vec.x(), world_vec.y(), world_vec.z()),
        link_world_pos=(wpose.pos().x(), wpose.pos().y(), wpose.pos().z()),
    )


def run_part_a(log):
    log("=" * 78)
    log("PART A - KINEMATIC TE-DIRECTION TESTS (tests 1, 2, 5, 6, 7)")
    log("=" * 78)
    results = {}
    for surf in KINEMATIC_SURFACES:
        name = surf["test"]
        log(f"\n--- {name} ---")
        log(f"joint={surf['joint']} link={surf['link']} local_TE_point={surf['local_point']} "
            f"check_axis={surf['check_axis']} hypothesis: {surf['hypothesis']}")
        per_angle = {}
        for deg in KIN_ANGLES_DEG:
            rad = math.radians(deg)
            m = measure_te_world_point(surf["joint"], surf["link"], surf["local_point"], rad)
            per_angle[deg] = m
            if m is None:
                log(f"  angle={deg:+.1f}deg: FAILED TO READ wpose/joint position (None)")
            else:
                log(f"  angle={deg:+.1f}deg (readback={m['readback_deg']:.4f}deg): "
                    f"world_TE_point={tuple(round(c,6) for c in m['world_point'])} "
                    f"link_world_pos={tuple(round(c,6) for c in m['link_world_pos'])}")

        ok_readback = all(
            per_angle[d] is not None and abs(per_angle[d]["readback_deg"] - d) < 0.05
            for d in KIN_ANGLES_DEG)

        axis_idx = {"x": 0, "y": 1, "z": 2}[surf["check_axis"]]
        v_neg = per_angle[-5.0]["world_point"][axis_idx] if per_angle[-5.0] else None
        v_zero = per_angle[0.0]["world_point"][axis_idx] if per_angle[0.0] else None
        v_pos = per_angle[5.0]["world_point"][axis_idx] if per_angle[5.0] else None

        if surf["check_axis"] == "z":
            # TE-up hypothesis: Z(+5) > Z(0) > Z(-5)
            monotonic_ok = (v_pos is not None and v_zero is not None and v_neg is not None
                             and v_pos > v_zero > v_neg)
            direction = "TE-up-for-positive-angle" if monotonic_ok else (
                "TE-down-for-positive-angle" if (v_pos is not None and v_zero is not None and v_neg is not None and v_pos < v_zero < v_neg)
                else "NOT-MONOTONIC/INCONCLUSIVE")
        else:  # y, rudder
            # TE-right hypothesis: Y(+5) < Y(0) < Y(-5)
            monotonic_ok = (v_pos is not None and v_zero is not None and v_neg is not None
                             and v_pos < v_zero < v_neg)
            direction = "TE-right(-Y)-for-positive-angle" if monotonic_ok else (
                "TE-left(+Y)-for-positive-angle" if (v_pos is not None and v_zero is not None and v_neg is not None and v_pos > v_zero > v_neg)
                else "NOT-MONOTONIC/INCONCLUSIVE")

        delta_pos = (v_pos - v_zero) if (v_pos is not None and v_zero is not None) else None
        delta_neg = (v_neg - v_zero) if (v_neg is not None and v_zero is not None) else None

        overall = bool(ok_readback and monotonic_ok)
        verdict = "CONFIRMS-HYPOTHESIS" if (overall and (
            (surf["check_axis"] == "z" and direction == "TE-up-for-positive-angle") or
            (surf["check_axis"] == "y" and direction == "TE-right(-Y)-for-positive-angle")
        )) else ("REFUTES-HYPOTHESIS" if overall else "FAIL")

        log(f"  readback fidelity ok: {ok_readback}")
        log(f"  {surf['check_axis'].upper()}(-5)={v_neg}, {surf['check_axis'].upper()}(0)={v_zero}, "
            f"{surf['check_axis'].upper()}(+5)={v_pos}")
        log(f"  delta at +5deg = {delta_pos}, delta at -5deg = {delta_neg}")
        log(f"  observed direction: {direction}")
        log(f"  {name}: {verdict}")

        results[name] = dict(
            per_angle={str(d): per_angle[d] for d in KIN_ANGLES_DEG},
            readback_ok=ok_readback,
            monotonic_ok=monotonic_ok,
            observed_direction=direction,
            delta_at_plus5=delta_pos,
            delta_at_minus5=delta_neg,
            verdict=verdict,
            pass_fail="PASS" if overall else "FAIL",
        )
    return results


# =============================================================================
# PART B - dynamic aero-moment sign tests (real applied wrench, live plugin)
# =============================================================================
MASS = 5.9348
I_EFF = 0.5
KP_LIN = 150.0
KP_ANG = 150.0
U = 15.0
HOLD_STEPS = 400
RELEASE_STEPS = 20
DEFLECTION_DEG = 8.0
DEFLECTION_RAD = math.radians(DEFLECTION_DEG)

LIN_TARGET = gm.Vector3d(U, 0.0, 0.0)
ANG_TARGET = gm.Vector3d(0.0, 0.0, 0.0)

DYNAMIC_PHASES = [
    dict(name="AILERON_DIFFERENTIAL_SIGN_TEST", released_axis="x", mask=(False, True, True),
         joint_positions={"left_aileron_joint": -DEFLECTION_RAD,
                            "right_aileron_joint": +DEFLECTION_RAD},
         hypothesis="aileron_sign=+1 confirmed if measured Mx > 0",
         also_reports=["AILERON_AERO_MOMENT_SIGN_TEST"]),
    dict(name="AILERON_COMMON_MODE_REJECTION_TEST", released_axis="x", mask=(False, True, True),
         joint_positions={"left_aileron_joint": +DEFLECTION_RAD,
                            "right_aileron_joint": +DEFLECTION_RAD},
         hypothesis="|Mx| much smaller than AILERON_DIFFERENTIAL_SIGN_TEST's result (near-zero, "
                    "bounded only by residual coupling)",
         also_reports=[]),
    dict(name="RUDDER_AERO_MOMENT_SIGN_TEST", released_axis="z", mask=(True, True, False),
         joint_positions={"rudder_joint": DEFLECTION_RAD},
         hypothesis="rudder_sign=+1 confirmed if measured Mz < 0 (nose-right)",
         also_reports=[]),
]


def run_part_b(log):
    log("\n" + "=" * 78)
    log("PART B - DYNAMIC AERO-MOMENT SIGN TESTS (tests 3, 4, 8, 9)")
    log("=" * 78)

    bounds = []
    t = 0
    for ph in DYNAMIC_PHASES:
        start = t
        hold_end = start + HOLD_STEPS
        rel_end = hold_end + RELEASE_STEPS
        bounds.append((start, hold_end, rel_end))
        t = rel_end
    total_steps = t

    state = {"n": 0, "phase_series": [[] for _ in DYNAMIC_PHASES], "any_nan": False, "inertia": None}

    def current_phase_idx(n):
        for i, (s, h, r) in enumerate(bounds):
            if s <= n < r:
                return i, n < h
        return len(DYNAMIC_PHASES) - 1, False

    def on_pre(info, ecm):
        n = state["n"]
        idx, in_hold = current_phase_idx(n)
        ph = DYNAMIC_PHASES[idx]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        mask = (True, True, True) if in_hold else ph["mask"]
        AL.hold_step(base, ecm, MASS, I_EFF, LIN_TARGET, ANG_TARGET,
                     kp_lin=KP_LIN, kp_ang=KP_ANG, ang_axis_mask=mask)
        AL.pin_child_joints(model, ecm, sim, positions=ph["joint_positions"])

    def on_post(info, ecm):
        n = state["n"]
        idx, in_hold = current_phase_idx(n)
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if state["inertia"] is None and n > 5:
            state["inertia"] = AL.read_base_link_inertia(model, ecm, sim)
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
        state["phase_series"][idx].append(dict(
            n=n, t=n * AL.STEP, in_hold=in_hold,
            u=lv_b.x(), v=lv_b.y(), w=lv_b.z(),
            p=av_b.x(), q=av_b.y(), r=av_b.z()))
        state["n"] += 1

    fixture = sim.TestFixture(AL.WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    log(f"Total steps run: {state['n']} (expected {total_steps})")
    log(f"Any NaN/Inf observed anywhere: {state['any_nan']}")
    inertia = state["inertia"]
    log(f"base_link inertia (queried at runtime): {inertia}")

    I_AXIS = {"x": inertia["ixx"], "y": inertia["iyy"], "z": inertia["izz"]}
    RATE_KEY = {"x": "p", "y": "q", "z": "r"}
    REFLEN = {"x": CFG.b, "y": CFG.c_ref, "z": CFG.b}
    C_LABEL = {"x": "Cl (Clda)", "y": "Cm (Cmde)", "z": "Cn (Cndr)"}

    results = {}
    for idx, ph in enumerate(DYNAMIC_PHASES):
        name = ph["name"]
        axis = ph["released_axis"]
        rate_key = RATE_KEY[axis]
        series = state["phase_series"][idx]
        hold_last = [r for r in series if r["in_hold"]][-1]
        free = [r for r in series if not r["in_hold"]]

        log(f"\n--- {name} ---")
        log(f"Hypothesis: {ph['hypothesis']}")
        log(f"Commanded joint positions: {ph['joint_positions']}")
        log(f"Hold-end state (converged): u={hold_last['u']:.4f} v={hold_last['v']:.4f} w={hold_last['w']:.4f} "
            f"p={hold_last['p']:.5f} q={hold_last['q']:.5f} r={hold_last['r']:.5f}")

        import numpy as np
        ts = np.array([r["t"] for r in free])
        rates = np.array([r[rate_key] for r in free])
        ts0 = ts - ts[0]
        slope = float(np.polyfit(ts0, rates, 1)[0])
        moment_measured = I_AXIS[axis] * slope

        jp = ph["joint_positions"]
        theta_la = jp.get("left_aileron_joint", 0.0)
        theta_ra = jp.get("right_aileron_joint", 0.0)
        theta_le = jp.get("left_elevator_joint", 0.0)
        theta_re = jp.get("right_elevator_joint", 0.0)
        theta_r = jp.get("rudder_joint", 0.0)
        delta_a = 0.5 * CFG.aileronSign * (theta_ra - theta_la)
        delta_e = 0.5 * CFG.elevatorSign * (theta_le + theta_re)
        delta_r = CFG.rudderSign * theta_r
        cross = AL.compute_aero(CFG, hold_last["u"], hold_last["v"], hold_last["w"],
                                 hold_last["p"], hold_last["q"], hold_last["r"],
                                 deltaA=delta_a, deltaE=delta_e, deltaR=delta_r)
        expected_moment = {"x": cross["Mx"], "y": cross["My"], "z": cross["Mz"]}[axis]

        log(f"delta_a={math.degrees(delta_a):.2f}deg delta_e={math.degrees(delta_e):.2f}deg "
            f"delta_r={math.degrees(delta_r):.2f}deg (aileron_sign={CFG.aileronSign} "
            f"elevator_sign={CFG.elevatorSign} rudder_sign={CFG.rudderSign}, aero_v1_config.yaml, UNMODIFIED)")
        log(f"Released axis: {axis} ({rate_key}). {rate_key}(t) over release window: "
            f"{[round(v,5) for v in rates]}")
        log(f"Measured slope d{rate_key}/dt = {slope:.5f} rad/s^2 over {len(free)} free steps "
            f"-> moment_measured = I_{axis}{axis}*slope = {I_AXIS[axis]:.4f}*{slope:.5f} = {moment_measured:.5f} N*m")
        log(f"Independent replica (from hold-end state + same delta mapping): "
            f"{C_LABEL[axis]}-driven M{axis}_expected = {expected_moment:.5f} N*m "
            f"(qbar={cross['qbar']:.3f}, ref_len={REFLEN[axis]})")

        nonzero_ok = bool(abs(moment_measured) > 1e-3)
        finite_ok = bool(not (math.isnan(moment_measured) or math.isinf(moment_measured)))
        sane_ok = bool(expected_moment != 0 and 0.2 < abs(moment_measured / expected_moment) < 5.0)
        same_sign = bool((moment_measured > 0) == (expected_moment > 0))

        results[name] = dict(moment_measured=moment_measured, moment_expected_replica=expected_moment,
                               nonzero_ok=nonzero_ok, finite_ok=finite_ok, sane_ok=sane_ok,
                               same_sign_as_replica=same_sign,
                               commanded=jp, hold_last=hold_last)
        log(f"Nonzero: {'PASS' if nonzero_ok else 'FAIL'}; Finite: {'PASS' if finite_ok else 'FAIL'}; "
            f"Sane magnitude (within 5x of formula replica): {'PASS' if sane_ok else 'FAIL'}; "
            f"Same sign as replica (config unmodified, checks plumbing self-consistency): "
            f"{'yes' if same_sign else 'no'}")
        log(f"Measured moment_measured={moment_measured:.5f} N*m")

    # ---- Interpret against controls-integration's specific hypotheses ----
    diff = results["AILERON_DIFFERENTIAL_SIGN_TEST"]
    common = results["AILERON_COMMON_MODE_REJECTION_TEST"]
    rudder = results["RUDDER_AERO_MOMENT_SIGN_TEST"]

    diff_verdict = "CONFIRMS-HYPOTHESIS" if diff["moment_measured"] > 0 else "REFUTES-HYPOTHESIS"
    diff_pf = "PASS" if diff["nonzero_ok"] and diff["finite_ok"] and diff["sane_ok"] and diff["moment_measured"] > 0 else "FAIL"

    ratio = abs(common["moment_measured"] / diff["moment_measured"]) if diff["moment_measured"] != 0 else float("inf")
    common_rejection_ok = ratio < 0.15  # "much smaller" - common-mode Mx should be a small residual, not comparable in magnitude to the differential case
    common_pf = "PASS" if common["finite_ok"] and common_rejection_ok else "FAIL"

    rudder_verdict = "CONFIRMS-HYPOTHESIS" if rudder["moment_measured"] < 0 else "REFUTES-HYPOTHESIS"
    rudder_pf = "PASS" if rudder["nonzero_ok"] and rudder["finite_ok"] and rudder["sane_ok"] and rudder["moment_measured"] < 0 else "FAIL"

    log("\n--- Interpretation against controls-integration's hypotheses ---")
    log(f"AILERON_DIFFERENTIAL_SIGN_TEST: measured Mx={diff['moment_measured']:.5f} N*m -> {diff_verdict} "
        f"(aileron_sign=+1 hypothesis needs Mx>0) -> {diff_pf}")
    log(f"AILERON_AERO_MOMENT_SIGN_TEST (same measurement run as AILERON_DIFFERENTIAL_SIGN_TEST): "
        f"measured Mx={diff['moment_measured']:.5f} N*m -> {diff_verdict} -> {diff_pf}")
    log(f"AILERON_COMMON_MODE_REJECTION_TEST: measured Mx={common['moment_measured']:.5f} N*m vs "
        f"differential Mx={diff['moment_measured']:.5f} N*m, |ratio|={ratio:.4f} "
        f"(expect << 1, threshold 0.15) -> {common_pf}")
    log(f"RUDDER_AERO_MOMENT_SIGN_TEST: measured Mz={rudder['moment_measured']:.5f} N*m -> {rudder_verdict} "
        f"(rudder_sign=+1 hypothesis needs Mz<0) -> {rudder_pf}")

    results["AILERON_DIFFERENTIAL_SIGN_TEST"].update(verdict=diff_verdict, pass_fail=diff_pf)
    results["AILERON_AERO_MOMENT_SIGN_TEST"] = dict(
        moment_measured=diff["moment_measured"], moment_expected_replica=diff["moment_expected_replica"],
        note="Same measurement run as AILERON_DIFFERENTIAL_SIGN_TEST, per task instruction ('same as test 3 essentially').",
        verdict=diff_verdict, pass_fail=diff_pf)
    results["AILERON_COMMON_MODE_REJECTION_TEST"].update(
        ratio_to_differential=ratio, verdict="CONFIRMS-HYPOTHESIS" if common_rejection_ok else "REFUTES-HYPOTHESIS",
        pass_fail=common_pf)
    results["RUDDER_AERO_MOMENT_SIGN_TEST"].update(verdict=rudder_verdict, pass_fail=rudder_pf)

    return results, state["any_nan"], inertia


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    log("FALCON V2 - CONTROL_SURFACE_SIGN_MAPPING test pass (gazebo-testing, 2026-08-22)")
    log("Verifying controls-integration's geometric TE-direction/aero-moment-sign hypotheses.")
    log("No aircraft physics parameter modified by this script.\n")

    part_a_results = run_part_a(log)
    part_b_results, any_nan, inertia = run_part_b(log)

    all_results = {**part_a_results, **part_b_results}
    overall = (not any_nan) and all(
        r.get("pass_fail") == "PASS" for r in all_results.values())

    log("\n" + "=" * 78)
    log("SUMMARY - all 9 tests")
    log("=" * 78)
    for name in ["LEFT_AILERON_JOINT_SIGN_TEST", "RIGHT_AILERON_JOINT_SIGN_TEST",
                 "AILERON_DIFFERENTIAL_SIGN_TEST", "AILERON_COMMON_MODE_REJECTION_TEST",
                 "LEFT_ELEVATOR_JOINT_SIGN_TEST", "RIGHT_ELEVATOR_JOINT_SIGN_TEST",
                 "RUDDER_JOINT_SIGN_TEST", "AILERON_AERO_MOMENT_SIGN_TEST",
                 "RUDDER_AERO_MOMENT_SIGN_TEST"]:
        r = all_results.get(name, {})
        log(f"{name}: {r.get('pass_fail','?')} ({r.get('verdict','?')})")
    log(f"\nOVERALL: {'PASS' if overall else 'FAIL'}")

    with open(f"{RESULTS_DIR}/control_surface_sign_mapping_result.json", "w") as f:
        json.dump({"part_a": part_a_results, "part_b": part_b_results,
                    "any_nan": any_nan, "inertia": inertia, "overall": overall}, f, indent=2,
                   default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/control_surface_sign_mapping_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

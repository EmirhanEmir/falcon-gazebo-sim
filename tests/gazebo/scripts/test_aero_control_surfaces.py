#!/usr/bin/env python3
"""
FALCON V2 - AERODYNAMICS_V1 live-Gazebo test pass (gazebo-testing, 2026-08-22).

Covers:
  - AILERON_ROLL_SIGN_TEST
  - RUDDER_YAW_SIGN_TEST
  - ELEVATOR_PITCH_SIGN_TEST

Per the task brief, the joint-sign-to-deflection-sign mapping
(aileron_sign/elevator_sign/rudder_sign in aero_v1_config.yaml) is still
ASSUMPTION-tagged pending `controls-integration`'s own sign tests
(CONTROLS.md sec 4). This script therefore reports what is OBSERVED
(commanded joint deflection -> real, measured roll/yaw/pitch moment applied
to base_link, direction and magnitude), and does NOT assert that observed
direction is the final "correct" physical control sense - that determination
belongs to controls-integration. What this script DOES conclusively check:
the plumbing from commanded joint position -> aero_v1_config.yaml's
delta_a/delta_e/delta_r mapping -> Clda/Cldr/Cmde-driven moment -> real
applied wrench on base_link is present, produces a NONZERO, FINITE,
sane-magnitude response, and is internally self-consistent with the
documented formula (independent python replica cross-check).

Method: for each surface, the relevant joint(s) are held at a fixed non-zero
deflection every step via gz.sim8 Joint.reset_position() (validated
reliable by this suite's prior structural-V1 test pass,
test_control_joint_limits_and_motion.py), while base_link is held at a
steady alpha=beta=0, p=q=r=0 forward-flight condition via
aero_lib.hold_step() (same technique as test_aero_stability_derivatives.py).
The OTHER, non-commanded control-surface/prop joints are pinned to 0 via
aero_lib.pin_child_joints() throughout (isolation technique, see that
function's docstring - found empirically necessary this pass to avoid a
test-harness artifact from these lightweight, undamped joints contaminating
a rate measurement). After convergence, the relevant rotational axis
(roll for aileron, yaw for rudder, pitch for elevator) is released (the
other two angular axes and translation stay controlled) and the real
resulting angular acceleration is measured directly from live ECM state -
exactly the same "release and measure" technique validated in
test_aero_stability_derivatives.py.

No aircraft physics parameter is modified by this script. No control-surface
joint sign is changed - only READ and driven via reset_position for testing
purposes, per this plugin's own read-only design (it never writes joint
commands itself, see AerodynamicsSystem.hh header).
"""
import json
import math
import sys

import numpy as np

import aero_lib as AL

AL.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
CFG = AL.load_config()

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

PHASES = [
    dict(name="AILERON_ROLL", released_axis="x", mask=(False, True, True),
         joint_positions={"left_aileron_joint": -DEFLECTION_RAD,
                            "right_aileron_joint": +DEFLECTION_RAD},
         note=(f"AILERON_ROLL_SIGN_TEST: left_aileron_joint={-DEFLECTION_DEG}deg, "
               f"right_aileron_joint={+DEFLECTION_DEG}deg (differential; per "
               f"aero_v1_config.yaml aileron_differential_convention this maps to "
               f"delta_a=0.5*aileron_sign*(right-left)={DEFLECTION_DEG}deg). Roll-axis "
               f"released after convergence; roll moment (Clda) checked for "
               f"nonzero/sane magnitude - direction NOT asserted as physically "
               f"'correct' (aileron_sign is ASSUMPTION-tagged, controls-integration's "
               f"call).")),
    dict(name="RUDDER_YAW", released_axis="z", mask=(True, True, False),
         joint_positions={"rudder_joint": DEFLECTION_RAD},
         note=(f"RUDDER_YAW_SIGN_TEST: rudder_joint={DEFLECTION_DEG}deg (maps to "
               f"delta_r=rudder_sign*theta={DEFLECTION_DEG}deg). Yaw-axis released "
               f"after convergence; yaw moment (Cndr) checked for nonzero/sane "
               f"magnitude - direction NOT asserted as physically 'correct' "
               f"(rudder_sign is ASSUMPTION-tagged).")),
    dict(name="ELEVATOR_PITCH", released_axis="y", mask=(True, False, True),
         joint_positions={"left_elevator_joint": DEFLECTION_RAD,
                            "right_elevator_joint": DEFLECTION_RAD},
         note=(f"ELEVATOR_PITCH_SIGN_TEST: left_elevator_joint=right_elevator_joint="
               f"{DEFLECTION_DEG}deg (symmetric; maps to "
               f"delta_e=0.5*elevator_sign*(left+right)={DEFLECTION_DEG}deg). "
               f"Pitch-axis released after convergence; pitch moment (Cmde) checked "
               f"for nonzero/sane magnitude - direction NOT asserted as physically "
               f"'correct' (elevator_sign is ASSUMPTION-tagged). RE-TEST NOTE "
               f"(2026-08-22, post Cm-to-My fix): Cmde is part of the STATIC group "
               f"(Cm0+Cma*alpha+Cmde*deltaE) that aerodynamics' scoped fix now negates "
               f"when computing the applied My torque - so this test's measured sign is "
               f"EXPECTED to be FLIPPED relative to the pre-fix run's result, and that is "
               f"NOT a new bug (see AeroModel.hh 'RESOLVED FINDING' comment and "
               f"aero_lib.compute_aero()'s docstring, which mirrors the same scoped "
               f"correction, so the replica cross-check below already reflects it too).")),
]


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    bounds = []
    t = 0
    for ph in PHASES:
        start = t
        hold_end = start + HOLD_STEPS
        rel_end = hold_end + RELEASE_STEPS
        bounds.append((start, hold_end, rel_end))
        t = rel_end
    total_steps = t

    state = {"n": 0, "phase_series": [[] for _ in PHASES], "any_nan": False, "inertia": None}

    def get_model(ecm):
        world = sim.World(sim.world_entity(ecm))
        model_e = world.model_by_name(ecm, "falcon_v2")
        return sim.Model(model_e)

    def current_phase_idx(n):
        for i, (s, h, r) in enumerate(bounds):
            if s <= n < r:
                return i, n < h
        return len(PHASES) - 1, False

    def on_pre(info, ecm):
        n = state["n"]
        idx, in_hold = current_phase_idx(n)
        ph = PHASES[idx]
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
    log("")

    I_AXIS = {"x": inertia["ixx"], "y": inertia["iyy"], "z": inertia["izz"]}
    RATE_KEY = {"x": "p", "y": "q", "z": "r"}
    REFLEN = {"x": CFG.b, "y": CFG.c_ref, "z": CFG.b}
    C_LABEL = {"x": "Cl (Clda)", "y": "Cm (Cmde)", "z": "Cn (Cndr)"}

    results = {}
    overall = not state["any_nan"]

    for idx, ph in enumerate(PHASES):
        name = ph["name"]
        axis = ph["released_axis"]
        rate_key = RATE_KEY[axis]
        series = state["phase_series"][idx]
        hold_last = [r for r in series if r["in_hold"]][-1]
        free = [r for r in series if not r["in_hold"]]

        log(f"=== {name} ===")
        log(ph["note"])
        log(f"Commanded joint positions: {ph['joint_positions']}")
        log(f"Hold-end state (converged): u={hold_last['u']:.4f} v={hold_last['v']:.4f} w={hold_last['w']:.4f} "
            f"p={hold_last['p']:.5f} q={hold_last['q']:.5f} r={hold_last['r']:.5f}")

        ts = np.array([r["t"] for r in free])
        rates = np.array([r[rate_key] for r in free])
        ts0 = ts - ts[0]
        slope = float(np.polyfit(ts0, rates, 1)[0])
        moment_measured = I_AXIS[axis] * slope

        # Independent replica cross-check: mirror the same joint->delta_x
        # mapping as AerodynamicsSystem.cc, using the config's sign
        # parameters (still ASSUMPTION-tagged, but this at least confirms
        # the plugin's INTERNAL math is self-consistent for whatever sign is
        # currently configured).
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
            f"elevator_sign={CFG.elevatorSign} rudder_sign={CFG.rudderSign}, aero_v1_config.yaml)")
        log(f"Released axis: {axis} ({rate_key}). {rate_key}(t) over release window: "
            f"{[round(v,5) for v in rates]}")
        log(f"Measured slope d{rate_key}/dt = {slope:.5f} rad/s^2 over {len(free)} free steps "
            f"-> moment_measured = I_{axis}{axis}*slope = {I_AXIS[axis]:.4f}*{slope:.5f} = {moment_measured:.5f} N*m")
        log(f"Independent replica (from hold-end state + same delta mapping): "
            f"{C_LABEL[axis]}-driven M{axis}_expected = {expected_moment:.5f} N*m "
            f"(qbar={cross['qbar']:.3f}, ref_len={REFLEN[axis]})")

        nonzero_ok = bool(abs(moment_measured) > 1e-3)
        finite_ok = bool(not (math.isnan(moment_measured) or math.isinf(moment_measured)))
        # "Sane magnitude": same order of magnitude as the independent
        # formula replica (catches gross plumbing errors - wrong axis, wrong
        # reference length, factor-of-10/100 bugs - without asserting a
        # specific physical sign convention).
        sane_ok = bool(expected_moment != 0 and 0.2 < abs(moment_measured / expected_moment) < 5.0)
        same_sign = bool((moment_measured > 0) == (expected_moment > 0))
        ok = nonzero_ok and finite_ok and sane_ok
        log(f"Nonzero: {'PASS' if nonzero_ok else 'FAIL'}; Finite: {'PASS' if finite_ok else 'FAIL'}; "
            f"Sane magnitude (within 5x of formula replica): {'PASS' if sane_ok else 'FAIL'}; "
            f"Same sign as replica (both use the SAME, still-ASSUMPTION-tagged config sign, "
            f"so this checks plumbing self-consistency, not real-world direction): "
            f"{'yes' if same_sign else 'no'}")
        log(f"{name} overall (plumbing/magnitude, NOT a physical-direction claim): {'PASS' if ok else 'FAIL'}")
        results[name] = dict(moment_measured=moment_measured, moment_expected_replica=expected_moment,
                               nonzero_ok=nonzero_ok, finite_ok=finite_ok, sane_ok=sane_ok,
                               same_sign_as_replica=same_sign, overall=ok)
        overall = overall and ok
        log("")

    log(f"=== test_aero_control_surfaces.py OVERALL (plumbing/magnitude only - physical "
        f"sign direction is controls-integration's determination): {'PASS' if overall else 'FAIL'} ===")

    with open(f"{RESULTS_DIR}/aero_control_surfaces_result.json", "w") as f:
        json.dump({"results": results, "overall": overall, "any_nan": state["any_nan"],
                    "inertia": inertia}, f, indent=2,
                   default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/aero_control_surfaces_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

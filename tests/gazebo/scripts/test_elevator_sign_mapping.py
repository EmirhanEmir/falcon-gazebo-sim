#!/usr/bin/env python3
"""
FALCON V2 - CONTROL_SURFACE_SIGN_MAPPING follow-up pass (gazebo-testing,
2026-08-22).

Runs the 2 tests deferred by the original CONTROL_SURFACE_SIGN_MAPPING pass
(tests/gazebo/scripts/test_control_surface_sign_mapping.py), now that
`aerodynamics` has applied the elevator_sign config fix (+1.0 -> -1.0) in
docs/source_of_truth/aerodynamics/aero_v1_config.yaml:

  10. ELEVATOR_SYMMETRIC_MAPPING_TEST - command theta_left=theta_right=+8deg
      (the SAME physical TE-up command as the pre-fix run), release the
      pitch axis, measure the REAL applied My via the live plugin, using the
      identical "hold steady condition then release one rotational axis and
      measure the real resulting angular acceleration" technique already
      validated by test_aero_control_surfaces.py / test_aero_stability_
      derivatives.py / test_control_surface_sign_mapping.py. Cross-checked
      against a fresh independent Python replica (aero_lib.compute_aero,
      which mirrors AeroModel.hh + the now-corrected elevator_sign).
      Expected: My flips sign to NEGATIVE (nose-up) relative to the pre-fix
      measurement (+1.128 N*m, nose-down) - this flip is the expected,
      correct CONSEQUENCE of the config fix, not a regression.

  11. ELEVATOR_AERO_MOMENT_SIGN_TEST - same live measurement as test 10
      (explicitly the same run, per the coordinator's own framing - "this
      may effectively be the same live run as test 1, that's fine, just
      report both explicitly"), interpreted against the hypothesis that the
      CORRECT physical elevator command (TE-up, commanded via
      elevator_sign=-1.0) now produces the textbook-correct pitch-moment
      direction: TE-up should produce a NOSE-UP (My<0 in this plugin's own
      established FLU sign convention, see AeroModel.hh "RESOLVED FINDING")
      moment, consistent with Cmde~-0.73/rad and the already-fixed static-
      group Cm-to-My negation.

Also includes an OPTIONAL cheap regression re-run of
test_aero_stability_derivatives.py (Cma_RESTORING_SIGN_TEST,
Cmq_DAMPING_SIGN_TEST, among others) to confirm this elevator_sign-only
config change did not disturb those unrelated, already-passing tests -
requested by the coordinator as a "cheap confirmation", not itself part of
controls-integration's original 9/11-test spec.

No aircraft physics parameter is modified by this script. aero_v1_config.yaml
is read-only here (already edited by `aerodynamics` before this script runs);
model/model.sdf is unmodified.
"""
import json
import math
import subprocess
import sys

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

PRE_FIX_MY = 1.128  # N*m, nose-down, cited from tests/gazebo/results/aero_control_surfaces_log.txt (elevator_sign=+1.0, pre-fix)


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    model_e = world.model_by_name(ecm, "falcon_v2")
    return sim.Model(model_e)


def run_elevator_phase(log):
    log("=" * 78)
    log("ELEVATOR_SYMMETRIC_MAPPING_TEST / ELEVATOR_AERO_MOMENT_SIGN_TEST")
    log("(same live measurement run, reported against both named tests, per")
    log(" coordinator instruction)")
    log("=" * 78)
    log(f"Config check: elevator_sign (loaded fresh from aero_v1_config.yaml this run) = {CFG.elevatorSign}")
    assert CFG.elevatorSign == -1.0, (
        f"Expected elevator_sign=-1.0 (post-fix), got {CFG.elevatorSign} - "
        f"aero_v1_config.yaml does not reflect the expected fix, aborting.")

    joint_positions = {"left_elevator_joint": DEFLECTION_RAD, "right_elevator_joint": DEFLECTION_RAD}
    state = {"n": 0, "series": [], "any_nan": False, "inertia": None}
    hold_end = HOLD_STEPS
    total_steps = HOLD_STEPS + RELEASE_STEPS

    def on_pre(info, ecm):
        n = state["n"]
        in_hold = n < hold_end
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        mask = (True, True, True) if in_hold else (True, False, True)  # release pitch (y) only
        AL.hold_step(base, ecm, MASS, I_EFF, LIN_TARGET, ANG_TARGET,
                     kp_lin=KP_LIN, kp_ang=KP_ANG, ang_axis_mask=mask)
        AL.pin_child_joints(model, ecm, sim, positions=joint_positions)

    def on_post(info, ecm):
        n = state["n"]
        in_hold = n < hold_end
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
        state["series"].append(dict(
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
    log(f"Any NaN/Inf observed: {state['any_nan']}")
    inertia = state["inertia"]
    log(f"base_link inertia (queried at runtime): {inertia}")

    series = state["series"]
    hold_last = [r for r in series if r["in_hold"]][-1]
    free = [r for r in series if not r["in_hold"]]

    log(f"Commanded joint positions: {joint_positions} (same physical TE-up command as the pre-fix run)")
    log(f"Hold-end state (converged): u={hold_last['u']:.4f} v={hold_last['v']:.4f} w={hold_last['w']:.4f} "
        f"p={hold_last['p']:.5f} q={hold_last['q']:.5f} r={hold_last['r']:.5f}")

    import numpy as np
    ts = np.array([r["t"] for r in free])
    rates = np.array([r["q"] for r in free])
    ts0 = ts - ts[0]
    slope = float(np.polyfit(ts0, rates, 1)[0])
    Iyy = inertia["iyy"]
    my_measured = Iyy * slope

    theta_le = joint_positions["left_elevator_joint"]
    theta_re = joint_positions["right_elevator_joint"]
    delta_e = 0.5 * CFG.elevatorSign * (theta_le + theta_re)
    cross = AL.compute_aero(CFG, hold_last["u"], hold_last["v"], hold_last["w"],
                             hold_last["p"], hold_last["q"], hold_last["r"],
                             deltaA=0.0, deltaE=delta_e, deltaR=0.0)
    my_expected = cross["My"]

    log(f"delta_e = 0.5*elevator_sign*(theta_left+theta_right) = 0.5*{CFG.elevatorSign}*"
        f"({math.degrees(theta_le):.1f}+{math.degrees(theta_re):.1f})deg = {math.degrees(delta_e):.2f}deg "
        f"(elevator_sign={CFG.elevatorSign}, aero_v1_config.yaml, POST-FIX)")
    log(f"Released axis: y (q). q(t) over release window: {[round(v,5) for v in rates]}")
    log(f"Measured slope dq/dt = {slope:.5f} rad/s^2 over {len(free)} free steps "
        f"-> My_measured = I_yy*slope = {Iyy:.4f}*{slope:.5f} = {my_measured:.5f} N*m")
    log(f"Independent replica (fresh aero_lib.compute_aero, mirrors AeroModel.hh + corrected "
        f"elevator_sign): My_expected = {my_expected:.5f} N*m "
        f"(cmStatic={cross['cmStatic']:.5f}, cmRate={cross['cmRate']:.5f}, "
        f"qbar={cross['qbar']:.3f}, c_ref={CFG.c_ref})")

    nonzero_ok = bool(abs(my_measured) > 1e-3)
    finite_ok = bool(not (math.isnan(my_measured) or math.isinf(my_measured)))
    sane_ok = bool(my_expected != 0 and 0.2 < abs(my_measured / my_expected) < 5.0)
    same_sign_as_replica = bool((my_measured > 0) == (my_expected > 0))
    flipped_vs_pre_fix = bool((my_measured > 0) != (PRE_FIX_MY > 0))
    nose_up_confirmed = bool(my_measured < 0)  # this plugin's My<0 convention = nose-up (AeroModel.hh "RESOLVED FINDING")

    log(f"Nonzero: {'PASS' if nonzero_ok else 'FAIL'}; Finite: {'PASS' if finite_ok else 'FAIL'}; "
        f"Sane magnitude (within 5x of replica): {'PASS' if sane_ok else 'FAIL'}; "
        f"Same sign as replica: {'yes' if same_sign_as_replica else 'no'}")
    log(f"Sign flip vs pre-fix measurement (My={PRE_FIX_MY:+.3f} N*m, nose-down, elevator_sign=+1.0 pre-fix): "
        f"{'YES, flipped as expected' if flipped_vs_pre_fix else 'NO FLIP OBSERVED'}")
    log(f"Physical interpretation: My_measured={my_measured:+.5f} N*m -> "
        f"{'NOSE-UP (My<0)' if my_measured < 0 else 'NOSE-DOWN (My>0)'} "
        f"(this plugin's established FLU My convention, AeroModel.hh 'RESOLVED FINDING')")

    verdict_10 = ("CONFIRMS-HYPOTHESIS" if (nonzero_ok and finite_ok and sane_ok and flipped_vs_pre_fix)
                  else "REFUTES-HYPOTHESIS")
    verdict_11 = ("CONFIRMS-HYPOTHESIS" if (nonzero_ok and finite_ok and sane_ok and nose_up_confirmed)
                  else "REFUTES-HYPOTHESIS")
    pf_10 = "PASS" if verdict_10 == "CONFIRMS-HYPOTHESIS" else "FAIL"
    pf_11 = "PASS" if verdict_11 == "CONFIRMS-HYPOTHESIS" else "FAIL"

    log(f"\nELEVATOR_SYMMETRIC_MAPPING_TEST (expects My to flip sign vs pre-fix +1.128 N*m): "
        f"{verdict_10} -> {pf_10}")
    log(f"ELEVATOR_AERO_MOMENT_SIGN_TEST (expects textbook-correct nose-up My<0 for physical TE-up command): "
        f"{verdict_11} -> {pf_11}")

    result = dict(
        elevator_sign_used=CFG.elevatorSign,
        commanded=joint_positions,
        hold_last=hold_last,
        my_measured=my_measured,
        my_expected_replica=my_expected,
        pre_fix_my=PRE_FIX_MY,
        nonzero_ok=nonzero_ok, finite_ok=finite_ok, sane_ok=sane_ok,
        same_sign_as_replica=same_sign_as_replica,
        flipped_vs_pre_fix=flipped_vs_pre_fix,
        nose_up_confirmed=nose_up_confirmed,
        any_nan=state["any_nan"], inertia=inertia,
    )
    return {
        "ELEVATOR_SYMMETRIC_MAPPING_TEST": {**result, "verdict": verdict_10, "pass_fail": pf_10},
        "ELEVATOR_AERO_MOMENT_SIGN_TEST": {**result, "verdict": verdict_11, "pass_fail": pf_11},
    }, state["any_nan"]


def run_regression_check(log):
    log("\n" + "=" * 78)
    log("OPTIONAL REGRESSION CHECK: re-run test_aero_stability_derivatives.py")
    log("(Cma_RESTORING_SIGN_TEST, Cmq_DAMPING_SIGN_TEST, etc. - should be")
    log(" unaffected by an elevator_sign-only config change)")
    log("=" * 78)
    try:
        proc = subprocess.run(
            [sys.executable, "/home/emirhan/Desktop/FalconV2/tests/gazebo/scripts/test_aero_stability_derivatives.py"],
            capture_output=True, text=True, timeout=600)
        out = proc.stdout + "\n" + proc.stderr
        for line in out.splitlines():
            if any(k in line for k in ("Cma_RESTORING_SIGN_TEST", "Cmq_DAMPING_SIGN_TEST",
                                        "Clp_DAMPING_SIGN_TEST", "Cnr_DAMPING_SIGN_TEST",
                                        "OVERALL", "overall")):
                log(f"  {line}")
        ok = proc.returncode == 0
        log(f"test_aero_stability_derivatives.py re-run exit code: {proc.returncode} -> "
            f"{'PASS (script-level)' if ok else 'FAIL (script-level)'}")
        return ok, out
    except Exception as e:
        log(f"Regression re-run FAILED TO EXECUTE: {e}")
        return False, str(e)


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    log("FALCON V2 - CONTROL_SURFACE_SIGN_MAPPING follow-up pass (gazebo-testing, 2026-08-22)")
    log("Post elevator_sign fix (+1.0 -> -1.0). No aircraft physics parameter modified by this script.\n")

    elevator_results, any_nan = run_elevator_phase(log)
    regression_ok, regression_raw = run_regression_check(log)

    overall = (not any_nan) and all(
        r["pass_fail"] == "PASS" for r in elevator_results.values())

    log("\n" + "=" * 78)
    log("SUMMARY")
    log("=" * 78)
    for name, r in elevator_results.items():
        log(f"{name}: {r['pass_fail']} ({r['verdict']})")
    log(f"Regression re-run (test_aero_stability_derivatives.py): {'PASS' if regression_ok else 'FAIL'}")
    log(f"\nOVERALL (2 required tests only, regression check informational): {'PASS' if overall else 'FAIL'}")

    with open(f"{RESULTS_DIR}/elevator_sign_mapping_result.json", "w") as f:
        json.dump({"tests": elevator_results, "any_nan": any_nan,
                    "regression_check_pass": regression_ok, "overall": overall}, f, indent=2,
                   default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/elevator_sign_mapping_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")
    with open(f"{RESULTS_DIR}/elevator_sign_mapping_regression_raw.txt", "w") as f:
        f.write(regression_raw)

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

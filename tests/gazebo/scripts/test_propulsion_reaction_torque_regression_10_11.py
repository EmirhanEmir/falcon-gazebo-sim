#!/usr/bin/env python3
"""
FALCON V2 - PROPULSION_V1_IMPLEMENTATION targeted re-test (gazebo-testing,
2026-08-23, post `propulsion` SetVelocity->ResetPosition fix).

Regression-check ONLY (item 4 of this re-test pass, NOT the full 6-test
tests/gazebo/scripts/test_propulsion_reaction_torque.py suite - the
coordinator's task explicitly scoped this pass to the tests affected by the
fix plus one new test, and these two steady-state checks are quick
confirmations that the fix did not break already-passing steady-state
behavior):

  10. PROP_REACTION_TORQUE_SIGN_TEST
  11. COUNTER_ROTATION_CANCELLATION_TEST

Both are re-run with the EXACT SAME method/parameters as the original pass
(imports and reuses measure_full_release()/expected_moments()/
measured_moments()/cmp_pct() directly from test_propulsion_reaction_torque.py
- no logic duplicated/forked) against the rebuilt plugin
(plugins/propulsion/build/libFalconV2Propulsion.so, ResetPosition-based).
HUB_FORCE_APPLICATION_TEST/DIFFERENTIAL_THRUST_MOMENT_TEST/engine-out tests
12-15 are NOT re-run here (unaffected by the SetVelocity->ResetPosition
change per the coordinator's scoping - the fix only touches the cosmetic
visual-joint-drive path, not force/torque application or hub geometry).

Updates tests/gazebo/results/propulsion_reaction_torque_result.json IN
PLACE for only these two keys (leaving the other 4 untouched from the
original pass), and adds a top-level "_regression_recheck_2026-08-23b"
metadata block documenting this re-test.

No aircraft physics parameter is modified anywhere in this script.
"""
import json
import sys

import test_propulsion_reaction_torque as RT

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
RESULT_JSON = f"{RESULTS_DIR}/propulsion_reaction_torque_result.json"


def run():
    log_lines = []

    def log(msg):
        print(msg, flush=True)
        log_lines.append(msg)

    log("=" * 78)
    log("REGRESSION RE-CHECK (post ResetPosition fix): tests 10/11 only")
    log("=" * 78)

    # ---- TEST 10: PROP_REACTION_TORQUE_SIGN_TEST ----
    log("=" * 78)
    log("TEST 10: PROP_REACTION_TORQUE_SIGN_TEST (regression re-check)")
    log("=" * 78)
    r10 = RT.measure_full_release(log, "REACTION_TORQUE_SIGN_REGRESSION",
                                   left_throttle=0.6, right_throttle=0.0,
                                   ang_hold_mask=(False, True, True))
    meas10 = RT.measured_moments(r10)
    exp10 = RT.expected_moments(r10["cg"], r10["left_diag_avg"]["thrust_N"], r10["right_diag_avg"]["thrust_N"],
                                 r10["left_diag_avg"]["Q_prop_Nm"], r10["right_diag_avg"]["Q_prop_Nm"])
    sign_ok = (meas10["Mx"] * exp10["Mx"]) > 0
    mag_pct = RT.cmp_pct(meas10["Mx"], exp10["Mx"])
    mag_ok = abs(mag_pct) <= 25.0
    pass10 = sign_ok and mag_ok
    log(f"Measured Mx={meas10['Mx']:.6f} N*m, Expected Mx={exp10['Mx']:.6f} N*m, "
        f"sign_ok={sign_ok}, mag_err={mag_pct:+.2f}%")
    log(f"PROP_REACTION_TORQUE_SIGN_TEST: {'PASS' if pass10 else 'FAIL'}")

    # ---- TEST 11: COUNTER_ROTATION_CANCELLATION_TEST ----
    log("=" * 78)
    log("TEST 11: COUNTER_ROTATION_CANCELLATION_TEST (regression re-check)")
    log("=" * 78)
    r11 = RT.measure_full_release(log, "COUNTER_ROTATION_CANCELLATION_REGRESSION",
                                   left_throttle=0.6, right_throttle=0.6,
                                   ang_hold_mask=(False, True, True))
    meas11 = RT.measured_moments(r11)
    exp11 = RT.expected_moments(r11["cg"], r11["left_diag_avg"]["thrust_N"], r11["right_diag_avg"]["thrust_N"],
                                 r11["left_diag_avg"]["Q_prop_Nm"], r11["right_diag_avg"]["Q_prop_Nm"])
    single_motor_scale = abs(r11["left_diag_avg"]["Q_prop_Nm"])
    cancel_tol = 0.15 * single_motor_scale
    measured_near_zero = abs(meas11["Mx"]) < cancel_tol
    expected_near_zero = abs(exp11["Mx"]) < cancel_tol
    pass11 = measured_near_zero and expected_near_zero
    log(f"Measured Mx={meas11['Mx']:.6f} N*m, Expected Mx={exp11['Mx']:.6f} N*m, "
        f"cancel_tol={cancel_tol:.6f} N*m")
    log(f"COUNTER_ROTATION_CANCELLATION_TEST: {'PASS' if pass11 else 'FAIL'}")

    overall = pass10 and pass11
    log("=" * 78)
    log(f"SUMMARY: PROP_REACTION_TORQUE_SIGN_TEST={'PASS' if pass10 else 'FAIL'}, "
        f"COUNTER_ROTATION_CANCELLATION_TEST={'PASS' if pass11 else 'FAIL'}")

    # ---- Update the existing result JSON in place (only these 2 keys) ----
    with open(RESULT_JSON) as f:
        existing = json.load(f)

    existing["PROP_REACTION_TORQUE_SIGN_TEST"] = dict(
        pass_=pass10, measured_Mx=meas10["Mx"], expected_Mx=exp10["Mx"],
        sign_ok=sign_ok, mag_err_pct=mag_pct,
        priority_check_cross_reference="tests/gazebo/results/propulsion_priority_check_result.json",
        regression_recheck_2026_08_23b=True)
    existing["COUNTER_ROTATION_CANCELLATION_TEST"] = dict(
        pass_=pass11, measured_Mx=meas11["Mx"], expected_Mx=exp11["Mx"], cancel_tol=cancel_tol,
        regression_recheck_2026_08_23b=True)
    existing["_regression_recheck_2026-08-23b"] = dict(
        reason=("propulsion rebuilt PropulsionSystem to drive left_prop_joint/right_prop_joint "
                "via Joint::ResetPosition() instead of Joint::SetVelocity() (duplicated-reaction-"
                "torque risk fix). Only tests 10/11 (steady-state PROP_REACTION_TORQUE_SIGN_TEST, "
                "COUNTER_ROTATION_CANCELLATION_TEST) re-run here as a quick regression check that "
                "the fix did not disturb already-passing steady-state force/torque application "
                "behavior (which is unchanged by this fix - only the cosmetic visual-joint-drive "
                "path changed). Tests 12-15 (HUB_FORCE_APPLICATION_TEST, "
                "DIFFERENTIAL_THRUST_MOMENT_TEST, LEFT/RIGHT_ENGINE_OUT_TEST) NOT re-run - out of "
                "scope for this targeted re-test pass."),
        overall_pass=overall,
        script="tests/gazebo/scripts/test_propulsion_reaction_torque_regression_10_11.py",
        log_file="tests/gazebo/results/propulsion_reaction_torque_regression_10_11_log.txt")

    with open(RESULT_JSON, "w") as f:
        json.dump(existing, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/propulsion_reaction_torque_regression_10_11_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

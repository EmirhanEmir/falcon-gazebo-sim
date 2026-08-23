#!/usr/bin/env python3
"""
FALCON V2 - TRIM_AND_DYNAMIC_AERO_VALIDATION, dynamic-derivative runtime
response magnitude/direction sanity check (gazebo-testing, 2026-08-23).

Covers the 5 checks requested this pass:
  - PITCH_RATE_DAMPING_RESPONSE          (Cmq)
  - ROLL_RATE_DAMPING_RESPONSE           (Clp)
  - YAW_RATE_DAMPING_RESPONSE            (Cnr)
  - POSITIVE_ALPHA_RESTORING_PITCH_RESPONSE  (Cma, static group)
  - POSITIVE_BETA_DIRECTIONAL_STABILITY_RESPONSE (Cnb)

This is explicitly NOT a re-run of the full unit sign-test suite
(Cmq/Clp/Cnr_DAMPING_SIGN_TEST, Cma_RESTORING_SIGN_TEST,
Cnb_STATIC_STABILITY_SIGN_TEST already have 17/17 PASS live-Gazebo
confirmation as of the 2026-08-22 retest, tests/gazebo/scripts/
test_aero_stability_derivatives.py). Per task instruction, this script:

  1. Loads tests/gazebo/results/aero_stability_derivatives_result.json
     (already contains a REAL, live-measured moment value - moment_measured -
     for all 5 derivatives, obtained via the same "hold trim then release one
     rotational DOF and measure the actual resulting angular acceleration"
     technique, at V=15 m/s / rate=0.4 rad/s / angle=8deg). These are cited
     directly here as the LIVE evidence for sign/direction - no Gazebo
     instance is re-launched by this script, since the existing result
     already has a quantitative value to compare (task instruction: "you may
     cite/reuse those directly ... instead of re-running the simulation").

  2. Separately (and NOT as a substitute for #1's live evidence), evaluates
     aero_lib.compute_aero() - the already-validated pure-python mirror of
     AeroModel.hh/ComputeAero(), no Gazebo/gz-sim dependency, no simulation
     time-stepping - at the EXACT example condition given in the task's own
     table (which uses different V/rate/angle values than the cited
     2026-08-22 live run: V=18.162 or 21.244 m/s, rate=0.5 rad/s, angle=5 or
     8 deg). This confirms the task's own quoted example magnitudes are
     exactly reproduced by the formula (a pure math self-consistency check),
     and gives an apples-to-apples target to scale the cited live measurement
     against.

  3. Scales the cited live measurement's own formula-replica value (already
     present in the existing JSON as moment_expected_replica, computed at the
     ACTUAL converged hold-end state of that 2026-08-22 run) up to the exact
     table condition, to get a fair, condition-matched ratio between "real
     live-measured moment" and "task's quoted example moment" - avoiding a
     misleading direct comparison of two numbers measured/computed at
     different speeds/rates/angles.

No aircraft physics parameter is modified anywhere in this script.
aero_v1_config.yaml is read-only.
"""
import json
import math
import sys

import aero_lib as AL

CFG = AL.load_config()

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
EXISTING_RESULT_PATH = f"{RESULTS_DIR}/aero_stability_derivatives_result.json"


def rad(x):
    return math.radians(x)


def deg(x):
    return math.degrees(x)


# Each check: name, coefficient, existing-result phase key, released axis,
# axis label for M, table condition (V, and either rate or angle), table's
# quoted expected moment, restoring/damping direction description.
CHECKS = [
    dict(
        name="PITCH_RATE_DAMPING_RESPONSE", coeff="Cmq", phase_key="CMQ_DAMPING",
        axis="y", m_key="My",
        # NOTE: "@ trim" here means the FULL neutral-trim state (Benchmark A's
        # own u=21.24357, w=-0.135166, i.e. alpha=alpha_trim, where cmStatic=
        # Cm0+Cma*alpha_trim=0 by construction) - NOT simply V=21.244 at
        # alpha=0. Verified: at alpha=0 (u=V,w=0) the formula gives
        # My=-1.048 N*m (cmStatic=Cm0=0.01055 does not cancel, adding an
        # extra static contribution on top of the pure Cmq*qHat rate term);
        # at the actual full trim state cmStatic=0 exactly, leaving only the
        # rate term, giving My=-0.7537 N*m - matching the task's quoted
        # -0.754 to 4 significant figures. Confirms "trim" must be read as
        # the complete Benchmark A condition, not just matched total speed.
        table_condition="q=+0.5 rad/s @ FULL neutral trim (u=21.24357, w=-0.135166, V=21.244)",
        table_kind="rate_at_trim", table_rate_axis="q", table_rate_val=0.5,
        trim_u=21.24357, trim_w=-0.135166,
        table_expected_M=-0.754, direction="opposing (damping)",
    ),
    dict(
        name="ROLL_RATE_DAMPING_RESPONSE", coeff="Clp", phase_key="CLP_DAMPING",
        axis="x", m_key="Mx",
        table_condition="p=+0.5 rad/s @ V=18.162", table_V=18.162241,
        table_kind="rate", table_rate_axis="p", table_rate_val=0.5,
        table_expected_M=-2.980, direction="opposing (damping)",
    ),
    dict(
        name="YAW_RATE_DAMPING_RESPONSE", coeff="Cnr", phase_key="CNR_DAMPING",
        axis="z", m_key="Mz",
        table_condition="r=+0.5 rad/s @ V=18.162", table_V=18.162241,
        table_kind="rate", table_rate_axis="r", table_rate_val=0.5,
        table_expected_M=-0.122, direction="opposing (damping)",
    ),
    dict(
        name="POSITIVE_ALPHA_RESTORING_PITCH_RESPONSE", coeff="Cma (static group)",
        phase_key="CMA_POS_8DEG", axis="y", m_key="My",
        table_condition="alpha=+8deg @ V=18.162", table_V=18.162241,
        table_kind="alpha", table_angle_deg=8.0,
        table_expected_M=4.514, direction="restoring (nose-down, My>0)",
    ),
    dict(
        name="POSITIVE_BETA_DIRECTIONAL_STABILITY_RESPONSE", coeff="Cnb",
        phase_key="CNB_STABILITY", axis="z", m_key="Mz",
        table_condition="beta=+5deg @ V=18.162", table_V=18.162241,
        table_kind="beta", table_angle_deg=5.0,
        table_expected_M=0.592, direction="restoring",
    ),
]


def state_for_check(chk):
    """Body-frame (u,v,w,p,q,r) for the exact table condition."""
    if chk["table_kind"] == "rate_at_trim":
        u, v, w = chk["trim_u"], 0.0, chk["trim_w"]
        p = q = r = 0.0
        if chk["table_rate_axis"] == "p":
            p = chk["table_rate_val"]
        elif chk["table_rate_axis"] == "q":
            q = chk["table_rate_val"]
        elif chk["table_rate_axis"] == "r":
            r = chk["table_rate_val"]
        return u, v, w, p, q, r
    V = chk["table_V"]
    if chk["table_kind"] == "rate":
        u, v, w = V, 0.0, 0.0
        p = q = r = 0.0
        if chk["table_rate_axis"] == "p":
            p = chk["table_rate_val"]
        elif chk["table_rate_axis"] == "q":
            q = chk["table_rate_val"]
        elif chk["table_rate_axis"] == "r":
            r = chk["table_rate_val"]
        return u, v, w, p, q, r
    elif chk["table_kind"] == "alpha":
        a = rad(chk["table_angle_deg"])
        u = V * math.cos(a)
        w = -V * math.sin(a)
        return u, 0.0, w, 0.0, 0.0, 0.0
    elif chk["table_kind"] == "beta":
        b = rad(chk["table_angle_deg"])
        u = V * math.cos(b)
        v = V * math.sin(b)
        return u, v, 0.0, 0.0, 0.0, 0.0
    raise ValueError(chk["table_kind"])


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    log("FALCON V2 - TRIM_AND_DYNAMIC_AERO_VALIDATION - dynamic-derivative runtime")
    log("response magnitude/direction sanity check (gazebo-testing, 2026-08-23)")
    log("Not a re-run of the full unit sign-test suite - citing existing 2026-08-22")
    log(f"live-measured results from {EXISTING_RESULT_PATH} plus a pure-python formula")
    log("cross-check at the task's exact example conditions (no new Gazebo instance launched).\n")

    with open(EXISTING_RESULT_PATH) as f:
        existing = json.load(f)
    existing_results = existing["results"]
    log(f"Existing result file overall={existing['overall']} any_nan={existing['any_nan']}")
    log(f"Existing per-axis inertia used for moment conversion: {existing['inertia']}\n")

    results = {}
    overall = True

    for chk in CHECKS:
        log("=" * 78)
        log(f"{chk['name']}  (derivative: {chk['coeff']})")
        log("=" * 78)

        # ---- (1) cited live evidence ----
        ex = existing_results[chk["phase_key"]]
        live_measured = ex["moment_measured"]
        live_replica_at_hold_state = ex["moment_expected_replica"]
        log(f"Cited LIVE evidence (2026-08-22 run, {chk['phase_key']}, real Delta-rate/Delta-t "
            f"measurement via live ECM state, NOT re-run this pass): "
            f"moment_measured={live_measured:.5f} N*m, "
            f"formula-replica-at-that-run's-own-hold-state={live_replica_at_hold_state:.5f} N*m "
            f"(internal cross-check ratio {live_measured/live_replica_at_hold_state:.3f} - already "
            f"validated in that run's own RATE_NORMALIZATION_TEST/verdict).")

        # ---- (2) pure-formula check at the task's exact table condition ----
        u, v, w, p, q, r = state_for_check(chk)
        table_calc = AL.compute_aero(CFG, u, v, w, p, q, r)
        m_at_table = table_calc[chk["m_key"]]
        log(f"Formula cross-check (aero_lib.compute_aero, NO simulation, exact table condition "
            f"'{chk['table_condition']}', u={u:.4f} v={v:.4f} w={w:.4f} p={p:.4f} q={q:.4f} r={r:.4f}): "
            f"{chk['m_key']}={m_at_table:.5f} N*m vs task-quoted example {chk['table_expected_M']:.5f} N*m "
            f"(diff={m_at_table-chk['table_expected_M']:.5f})")

        formula_matches_task_example = abs(m_at_table - chk["table_expected_M"]) < 0.05 * max(1.0, abs(chk["table_expected_M"]))
        log(f"Formula reproduces task's quoted example magnitude within 5%: "
            f"{'YES' if formula_matches_task_example else 'NO'}")

        # ---- (3) condition-matched scaling of the cited live measurement ----
        # Scale factor = (moment predicted by the SAME formula at the table
        # condition) / (moment predicted by the formula at the 2026-08-22
        # run's own actual hold-end state) - both computed the identical way,
        # so this isolates the pure V/rate/angle scaling, independent of any
        # live-run measurement noise.
        scale = m_at_table / live_replica_at_hold_state if live_replica_at_hold_state != 0 else float("nan")
        live_scaled_to_table = live_measured * scale
        ratio_vs_table = live_scaled_to_table / chk["table_expected_M"] if chk["table_expected_M"] != 0 else float("nan")
        log(f"Condition-matched scale factor (table-condition formula / 2026-08-22-run's-own-hold-state "
            f"formula) = {scale:.4f}")
        log(f"Live measurement scaled to the table condition: {live_measured:.5f} * {scale:.4f} = "
            f"{live_scaled_to_table:.5f} N*m, vs task example {chk['table_expected_M']:.5f} N*m -> "
            f"ratio={ratio_vs_table:.3f}")

        # ---- sign/direction verdict (the actual pass/fail criterion here -
        # this is a sanity check, not a precision test, per task instruction) ----
        same_sign = (live_measured > 0) == (chk["table_expected_M"] > 0)
        within_factor_2 = 0.5 <= ratio_vs_table <= 2.0 if not math.isnan(ratio_vs_table) else False
        magnitude_flag = "OK (within ~2x)" if within_factor_2 else "FLAG (outside ~2x factor - review)"
        log(f"Sign match (live measured vs table expected): {'YES' if same_sign else 'NO'} "
            f"({chk['direction']})")
        log(f"Order-of-magnitude check (condition-scaled live vs table): {magnitude_flag}")

        check_pass = same_sign and formula_matches_task_example
        log(f"VERDICT: {'PASS' if check_pass else 'FAIL'} "
            f"(criteria: correct sign from real live measurement AND formula reproduces the task's own "
            f"quoted example; magnitude flag is informational engineering judgment, not a hard fail)")
        overall = overall and check_pass

        results[chk["name"]] = dict(
            coefficient=chk["coeff"], table_condition=chk["table_condition"],
            live_measured_2026_08_22=live_measured,
            live_replica_at_hold_state_2026_08_22=live_replica_at_hold_state,
            formula_at_table_condition=m_at_table,
            table_quoted_expected=chk["table_expected_M"],
            formula_matches_task_example=formula_matches_task_example,
            scale_factor=scale, live_scaled_to_table=live_scaled_to_table,
            ratio_vs_table=ratio_vs_table,
            same_sign=same_sign, within_factor_2=within_factor_2,
            check_pass=check_pass,
        )
        log("")

    log("=" * 78)
    log("SUMMARY")
    log("=" * 78)
    for chk in CHECKS:
        r = results[chk["name"]]
        log(f"{chk['name']} ({chk['coeff']}): {'PASS' if r['check_pass'] else 'FAIL'} - "
            f"live-measured={r['live_measured_2026_08_22']:.4f} N*m (cited, 2026-08-22 run), "
            f"table-condition-formula={r['formula_at_table_condition']:.4f} N*m vs "
            f"task-example={r['table_quoted_expected']:.4f} N*m, "
            f"scaled-ratio={r['ratio_vs_table']:.2f}, "
            f"magnitude={'OK' if r['within_factor_2'] else 'FLAG'}")
    log(f"\nOVERALL: {'PASS' if overall else 'FAIL'}")

    with open(f"{RESULTS_DIR}/dynamic_derivative_response_result.json", "w") as f:
        json.dump({"checks": results, "overall": overall,
                    "cited_source_file": EXISTING_RESULT_PATH,
                    "cited_source_overall": existing["overall"]}, f, indent=2,
                   default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/dynamic_derivative_response_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

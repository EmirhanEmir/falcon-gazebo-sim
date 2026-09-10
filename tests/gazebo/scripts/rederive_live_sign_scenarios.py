#!/usr/bin/env python3
"""FALCON V2 - re-derive the Part 2 live sign-scenario verdicts OFFLINE from
the already-captured timeseries, and report the TRANSIENT PEAK deflections
that the steady-window verdict does not show.

CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY.
Owner: gazebo-testing. Created 2026-09-10.

WHY THIS EXISTS
---------------
1. Scenarios C..F were captured BEFORE the sign-character harness fix in
   test_control_surface_live_sign_scenarios.py (the Cm / Cn checks were
   silently left UNJUDGED whenever the pre-registered expectation string
   carried explanatory prose after the sign character). analyse_scenario() is
   a pure function of the captured windows, which are stored verbatim in
   *_timeseries.json, so the verdicts can be recomputed WITHOUT re-flying -
   no new simulation, no new data, no parameter of any kind.
2. The verdict is computed on the STEADY portion of the input window. In FBWA
   a roll demand settles to a commanded bank angle, at which point the
   ailerons return to near-neutral and Cl returns to near-zero, so the steady
   window understates the control input that was actually applied. The
   TRANSIENT PEAK is therefore reported here alongside it. This ADDS
   information; it does not replace or relax any check.

CHANGES NO PHYSICS PARAMETER. Reads results/timeseries JSON only.

USAGE
    python3 rederive_live_sign_scenarios.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import manual_takeoff_live_lib as L  # noqa: E402
import test_control_surface_live_sign_scenarios as S2  # noqa: E402

RES = L.RESULTS_DIR
PREFIX = "control_surface_live_sign"
DEG = 57.29577951308232


def peak_of(recs, path):
    """Largest-magnitude value of recs[*][path...] (signed)."""
    best = None
    for r in recs:
        v = r
        for k in path:
            v = v.get(k) if isinstance(v, dict) else None
            if v is None:
                break
        if isinstance(v, (int, float)):
            if best is None or abs(v) > abs(best):
                best = v
    return best


def main():
    out = {
        "stage": "MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION",
        "part": "2 - re-derived offline from captured timeseries",
        "owner": "gazebo-testing",
        "physics_parameters_changed": "NONE",
        "note": "Verdicts recomputed with the corrected sign-character "
                "evaluation. No simulation was re-run to produce these "
                "numbers; the captured windows are unchanged.",
        "scenarios": {},
    }
    for k in sorted(S2.SCENARIOS):
        tsp = os.path.join(RES, f"{PREFIX}_scenario_{k}_timeseries.json")
        if not os.path.exists(tsp):
            out["scenarios"][k] = {"status": "NO_TIMESERIES (bring-up never "
                                            "produced a measurement)"}
            continue
        with open(tsp, "r", encoding="utf-8") as fh:
            ts = json.load(fh)
        w = ts["windows"]
        pre = dict(L.PRE_REGISTERED_EXPECTATIONS[S2.SCENARIOS[k]["expect_key"]])
        r = S2.analyse_scenario(k, pre, w["before"], w["during"], w["after"])
        during = w["during"]
        transient = {
            "peak_delta_a_deg": (peak_of(during, ["aero_input_rad", "delta_a"]) or 0.0) * DEG,
            "peak_delta_e_deg": (peak_of(during, ["aero_input_rad", "delta_e"]) or 0.0) * DEG,
            "peak_delta_r_deg": (peak_of(during, ["aero_input_rad", "delta_r"]) or 0.0) * DEG,
            "peak_left_aileron_deg": (peak_of(during, ["actual_rad", "left_aileron"]) or 0.0) * DEG,
            "peak_right_aileron_deg": (peak_of(during, ["actual_rad", "right_aileron"]) or 0.0) * DEG,
            "peak_left_elevator_deg": (peak_of(during, ["actual_rad", "left_elevator"]) or 0.0) * DEG,
            "peak_right_elevator_deg": (peak_of(during, ["actual_rad", "right_elevator"]) or 0.0) * DEG,
            "peak_rudder_deg": (peak_of(during, ["actual_rad", "rudder"]) or 0.0) * DEG,
            "peak_Cl": peak_of(during, ["aero", "Cl"]),
            "peak_Cm": peak_of(during, ["aero", "Cm"]),
            "peak_Cn": peak_of(during, ["aero", "Cn"]),
            "note": "Largest-magnitude sample inside the input window, "
                    "SIGNED. Reported because the verdict above is computed "
                    "on the post-transient steady portion, where a settled "
                    "FBWA bank/pitch demand needs almost no control "
                    "deflection.",
        }
        out["scenarios"][k] = {
            "name": r["name"],
            "verdict": r["verdict"],
            "n_checks_pass": r["n_checks_pass"],
            "n_checks_total": r["n_checks_total"],
            "checks": r["checks"],
            "delta_vs_baseline": r["delta_vs_baseline"],
            "peak_response": r["peak_response"],
            "transient_peaks_during_input": transient,
        }
    dst = os.path.join(RES, f"{PREFIX}_rederived_summary.json")
    with open(dst, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    print("wrote", dst)
    for k, v in out["scenarios"].items():
        if "verdict" not in v:
            print(f"  {k} {v['status']}")
            continue
        print(f"  {k} {v['name']:12s} {v['verdict']:12s} "
              f"{v['n_checks_pass']}/{v['n_checks_total']}")
        for c in v["checks"]:
            if c.get("pass") is False:
                print(f"      FAIL {c['check']}: measured={c.get('measured')} "
                      f"sign={c.get('measured_sign')} expected={c.get('expected_sign')} "
                      f"floor={c.get('floor')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

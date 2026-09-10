#!/usr/bin/env python3
"""FALCON V2 - RE-JUDGEMENT of the Part 2 live control-surface SIGN scenarios
A-F, on the ALREADY-CAPTURED data.

CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY.
Stage: MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION.
Owner: gazebo-testing. Created 2026-09-10.

NOTHING IS RE-FLOWN. This module reads
  tests/gazebo/results/control_surface_live_sign_scenario_{A..F}_timeseries.json
  tests/gazebo/results/control_surface_live_sign_scenario_{A..F}_result.json
and writes ONE new artifact. It changes no threshold, no floor, no
pre-registered expectation, no aerodynamic/propulsion coefficient, no
mass/CG/inertia, no actuator limit, no ArduPilot parameter, no SDF and no
config. The floors it uses are IMPORTED from the original harness's own TH
block so they cannot drift.

=============================================================================
WHY A RE-JUDGEMENT IS NEEDED (the defect is in the OBSERVABLE, not the model)
=============================================================================
`aerodynamics` independently verified the plugin against a C++ oracle built
around the unmodified AeroModel.hh (bit-exact, 1486 states) and reported
RUDDER_CONTROL_SIGN = PASS: the plugin's Cn, the applied Mz, the rudder
lookup sign, rudder_sign=+1 and the pre-registered statement
"Cn<0 <=> Mz<0 <=> nose right (FLU)" are all correct and mutually consistent.

The Part 2 harness nevertheless judged the yaw scenarios on
`delta["Cn"]` = (mean Cn over the LAST 75% of the input window) - baseline.
That is a CLOSED-LOOP EQUILIBRIUM quantity. By the time it is measured the
airframe has developed sideslip and the weathercock term Cnb*beta dominates
the rudder term, so the TOTAL Cn no longer carries the sign of the control
input under test. The same structural problem makes the roll/pitch STEADY
coefficients decay below the 1e-4 reporting floor (sign "0", pass False),
which is why A/B passed only through their transient-peak fields.

Only yaw actually REVERSES: roll and pitch feedback are RATE/damping terms
(Clp, Cmq) that decay the control term toward zero but can never flip its
sign, whereas yaw feedback is a STIFFNESS term (Cnb*beta) whose overshoot
does flip it, and |Cnr| = 0.02227 is ~1/24 of |Clp| = 0.54187 so the Dutch
roll rings for the whole window.

=============================================================================
THE THREE CORRECTED OBSERVABLES (all three are reported, none replaces the
raw data, and the ORIGINAL judgement is reported next to every one of them)
=============================================================================
O1  AT CONTROL ONSET.
    The first sample of the input window at which the axis's own aero
    deflection (delta_a / delta_e / delta_r, taken from the actuator
    plugin's ACTUAL joint angles) has moved more than the harness's OWN
    min_surface_deflection_deg floor away from its pre-input baseline,
    subject to the airframe not yet having built up sideslip:
    |beta| < BETA_GUARD_DEG. The coefficient delta vs the SAME baseline is
    judged there, against the SAME pre-registered sign, with the SAME
    min_coefficient_magnitude floor.
O2  SIGNED EXTREMUM NEAREST THE STEP.
    The extremum of the coefficient delta over the input window taken
    FORWARD from control onset up to the first zero crossing of that delta -
    i.e. the first excursion the control produced - instead of
    max(..., key=abs) over the whole window, which selects the LATER
    airframe-feedback overshoot whenever that overshoot is larger.
O3  ISOLATED CONTROL TERM.
    dCn_rudder = interp_linear(ctrlBreakpointsRad, ctrlRuddCn, delta_r), and
    the corresponding dCl_aileron / dCm_elevator lookups, evaluated with
    aero_lib (now a re-synced, oracle-verified mirror of the plugin). This
    isolates exactly the quantity under test with no airframe feedback in it
    at all.

BETA_GUARD_DEG is a WINDOW-SELECTION guard, not a pass/fail threshold: it
only decides WHICH SAMPLE the pre-registered sign test is applied to. It is
reported, and O2/O3 do not use it, so a reader who dislikes it can read the
verdict off O2 or O3 instead.

USAGE
    python3 rejudge_control_surface_sign_scenarios.py
"""
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import manual_takeoff_live_lib as L                       # noqa: E402
import test_control_surface_live_sign_scenarios as P2      # noqa: E402

RES = L.RESULTS_DIR
PREFIX = "control_surface_live_sign_scenario"

# IMPORTED, never redefined here, so a future change to the harness's floors
# automatically changes this re-judgement too.
DEFL_FLOOR_DEG = P2.TH["min_surface_deflection_deg"]["value"]   # 0.5
RATE_FLOOR_DEG_S = P2.TH["min_rate_response_deg_s"]["value"]    # 2.0
COEF_FLOOR = P2.TH["min_coefficient_magnitude"]["value"]        # 1e-4

BETA_GUARD_DEG = 2.0
BETA_GUARD_BASIS = (
    "WINDOW SELECTION ONLY - not a pass/fail threshold. Bounds how much "
    "sideslip the airframe may have built before the pre-registered sign "
    "test is applied, so the sample being judged is one where the CONTROL "
    "term still dominates the weathercock term Cnb*beta. With Cnb=+0.03554 "
    "(CLAUDE.md reference point) a 2 deg sideslip contributes "
    "Cnb*beta = +0.00124, while the measured rudder contribution at onset is "
    "~-0.0096; the control term is therefore still ~8x the feedback term at "
    "the guard limit. O2 and O3 below use no beta guard at all.")

AXIS_DEFL_KEY = {"roll": "delta_a", "pitch": "delta_e", "yaw": "delta_r"}
AXIS_COEF = {"roll": "Cl", "pitch": "Cm", "yaw": "Cn"}
AXIS_EXPECT = {"roll": "expected_Cl_sign", "pitch": "expected_Cm_sign",
               "yaw": "expected_Cn_sign"}


def mean(xs):
    xs = [x for x in xs if isinstance(x, (int, float)) and not math.isnan(x)]
    return sum(xs) / len(xs) if xs else float("nan")


def get(rec, path):
    v = rec
    for k in path:
        if not isinstance(v, dict):
            return None
        v = v.get(k)
    return v


def sgn(x, floor):
    return P2.sgn_str(x, floor)


def rejudge(key):
    ts_path = os.path.join(RES, f"{PREFIX}_{key}_timeseries.json")
    rs_path = os.path.join(RES, f"{PREFIX}_{key}_result.json")
    if not (os.path.exists(ts_path) and os.path.exists(rs_path)):
        return {"scenario": key, "status": "MISSING_ARTIFACT",
                "timeseries_present": os.path.exists(ts_path),
                "result_present": os.path.exists(rs_path)}
    ts = json.load(open(ts_path, encoding="utf-8"))
    rs = json.load(open(rs_path, encoding="utf-8"))
    before = ts["windows"]["before"]
    during = ts["windows"]["during"]
    axis = P2.SCENARIOS[key]["axis"]
    dkey = AXIS_DEFL_KEY[axis]
    ckey = AXIS_COEF[axis]
    pre = dict(L.PRE_REGISTERED_EXPECTATIONS[P2.SCENARIOS[key]["expect_key"]])
    expected_sign = pre[AXIS_EXPECT[axis]][0]

    # ---- baselines: EXACTLY the harness's own definition (mean over the
    # settle window immediately before the input) ----
    base_defl_deg = math.degrees(mean([get(r, ["aero_input_rad", dkey])
                                       for r in before]))
    base_coef = mean([get(r, ["aero", ckey]) for r in before])
    base_beta_deg = math.degrees(mean([get(r, ["aero", "beta"]) for r in before]))

    # ---- the ORIGINAL judgement ------------------------------------------
    # TWO forms are reported, because they differ and hiding either would
    # misrepresent the record:
    #  (a) AS_STORED: the check list actually written into the
    #      *_result.json at capture time. At that moment the harness passed
    #      the whole "- (Mz<0 = nose right)" string to check(), which only
    #      judges a plain "+"/"-", so the coefficient check was recorded
    #      UNJUDGED (pass=None) and never counted toward the verdict.
    #  (b) CURRENT_HARNESS_CODE: what the harness as it stands TODAY decides
    #      on the SAME captured windows. The [0] sign-character fix landed
    #      after these runs, so the coefficient check is now judged - and on
    #      the closed-loop steady observable it FAILS. Reproduced here by
    #      calling the harness's own analyse_scenario() on the stored data.
    #      NOTHING IS RE-FLOWN.
    stored = next((c for c in rs["result"]["checks"] if c["check"] == ckey), None)
    stored_verdict = rs["result"]["verdict"]
    replay = P2.analyse_scenario(key, pre, before, during,
                                 ts["windows"].get("after", []))
    old = next((c for c in replay["checks"] if c["check"] == ckey), None)
    old_verdict = replay["verdict"]
    old_checks = replay["checks"]

    # ---- O1: at control onset -------------------------------------------
    onset_i = None
    for i, r in enumerate(during):
        d = get(r, ["aero_input_rad", dkey])
        b = get(r, ["aero", "beta"])
        if d is None or b is None:
            continue
        if abs(math.degrees(d) - base_defl_deg) > DEFL_FLOOR_DEG \
                and abs(math.degrees(b)) < BETA_GUARD_DEG:
            onset_i = i
            break
    o1 = {"status": "NO_SAMPLE_MET_THE_ONSET_CONDITION"}
    if onset_i is not None:
        r = during[onset_i]
        dc = get(r, ["aero", ckey]) - base_coef
        o1 = {
            "status": "JUDGED",
            "sample_index_in_input_window": onset_i,
            "t_since_input_start_s": r["t_wall"] - during[0]["t_wall"],
            f"{dkey}_deg": math.degrees(get(r, ["aero_input_rad", dkey])),
            f"{dkey}_delta_vs_baseline_deg": math.degrees(
                get(r, ["aero_input_rad", dkey])) - base_defl_deg,
            "beta_deg": math.degrees(get(r, ["aero", "beta"])),
            f"{ckey}_raw": get(r, ["aero", ckey]),
            f"delta_{ckey}_vs_baseline": dc,
            "measured_sign": sgn(dc, COEF_FLOOR),
            "expected_sign": expected_sign,
            "floor": COEF_FLOOR,
            "pass": sgn(dc, COEF_FLOOR) == expected_sign,
            "body_rates_at_this_sample_deg_s": {
                "roll": get(r, ["attitude", "rollspeed_deg_s"]),
                "pitch": get(r, ["attitude", "pitchspeed_deg_s"]),
                "yaw_MAVLINK_FRD": get(r, ["attitude", "yawspeed_deg_s"]),
            },
        }

    # ---- O2: signed extremum of the FIRST excursion after onset ----------
    o2 = {"status": "NOT_APPLICABLE_NO_ONSET"}
    if onset_i is not None:
        series = []
        for r in during[onset_i:]:
            c = get(r, ["aero", ckey])
            if c is None:
                continue
            series.append((r["t_wall"] - during[0]["t_wall"], c - base_coef))
        if series:
            s0 = series[0][1]
            first_sign = 1.0 if s0 >= 0 else -1.0
            seg = []
            for t, v in series:
                if v * first_sign < 0:
                    break
                seg.append((t, v))
            ext = max(seg, key=lambda tv: abs(tv[1])) if seg else series[0]
            o2 = {
                "status": "JUDGED",
                "definition": "extremum of the coefficient delta over the "
                              "FIRST excursion after control onset (up to "
                              "its first zero crossing), i.e. the excursion "
                              "the control produced - not max(|.|) over the "
                              "whole window",
                "t_since_input_start_s": ext[0],
                "n_samples_in_first_excursion": len(seg),
                f"extremum_delta_{ckey}": ext[1],
                "measured_sign": sgn(ext[1], COEF_FLOOR),
                "expected_sign": expected_sign,
                "pass": sgn(ext[1], COEF_FLOOR) == expected_sign,
                "harness_max_by_abs_over_whole_window": max(
                    (v for _, v in [(r["t_wall"], get(r, ["aero", ckey]) - base_coef)
                                    for r in during
                                    if get(r, ["aero", ckey]) is not None]),
                    key=abs, default=None),
            }

    # ---- O3: isolated control term from the plugin's own lookup ----------
    o3 = {"status": "BLOCKED"}
    try:
        import aero_lib as AL
        cfg = AL.load_config()
        bp = cfg.ctrlBreakpointsRad
        table = {"roll": cfg.ctrlAileCl, "pitch": cfg.ctrlElevDCm,
                 "yaw": cfg.ctrlRuddCn}[axis]
        sign_scalar = {"roll": cfg.aileronSign, "pitch": cfg.elevatorSign,
                       "yaw": cfg.rudderSign}[axis]
        vals = []
        for r in during:
            d = get(r, ["aero_input_rad", dkey])
            if d is None:
                continue
            vals.append(AL.interp_linear(bp, table, d))
        steady = vals[max(1, len(vals) // 4):] if len(vals) > 4 else vals
        at_onset = vals[onset_i] if (onset_i is not None and onset_i < len(vals)) else None
        o3 = {
            "status": "JUDGED",
            "definition": f"interp_linear(ctrlBreakpointsRad, "
                          f"ctrl<{axis}>{ckey} table, {dkey}) - the control "
                          f"term ONLY, no airframe feedback in it at all",
            "lookup_sign_scalar_from_config": sign_scalar,
            "value_at_control_onset": at_onset,
            "mean_over_steady_75pct_of_input_window": mean(steady),
            "max_abs_over_input_window": max((abs(v) for v in vals), default=None),
            "measured_sign_at_onset": sgn(at_onset, COEF_FLOOR),
            "measured_sign_steady": sgn(mean(steady), COEF_FLOOR),
            "expected_sign": expected_sign,
            "pass_at_onset": (sgn(at_onset, COEF_FLOOR) == expected_sign)
                             if at_onset is not None else None,
            "pass_steady": sgn(mean(steady), COEF_FLOOR) == expected_sign,
        }
    except Exception as e:
        o3 = {"status": "BLOCKED", "reason": repr(e)}

    # ---- feedback decomposition, so the mechanism is visible in numbers ---
    decomp = {"status": "NOT_APPLICABLE"}
    if axis == "yaw":
        try:
            import aero_lib as AL
            cfg = AL.load_config()
            n = len(during)
            steady_recs = during[max(1, n // 4):] if n > 4 else during
            cnb_beta = mean([cfg.Cnb * get(r, ["aero", "beta"]) for r in steady_recs
                             if get(r, ["aero", "beta"]) is not None])
            dcn_r = mean([AL.interp_linear(cfg.ctrlBreakpointsRad, cfg.ctrlRuddCn,
                                           get(r, ["aero_input_rad", "delta_r"]))
                          for r in steady_recs
                          if get(r, ["aero_input_rad", "delta_r"]) is not None])
            cn_tot = mean([get(r, ["aero", "Cn"]) for r in steady_recs])
            decomp = {"status": "COMPUTED",
                      "window": "the SAME steady 75% window the harness judged",
                      "Cnb_times_beta_mean": cnb_beta,
                      "dCn_rudder_lookup_mean": dcn_r,
                      "Cn_total_published_mean": cn_tot,
                      "Cnb_used": cfg.Cnb,
                      "note": "Shows directly why the STEADY TOTAL Cn does not "
                              "carry the rudder's sign: the weathercock term "
                              "is the larger of the two."}
        except Exception as e:
            decomp = {"status": "BLOCKED", "reason": repr(e)}

    # ---- O1b: the DEFLECTION checks, re-judged at the SAME onset sample ---
    #
    # ITEM 2 of the correction. The aileron-joint and delta_a/delta_e/delta_r
    # checks use the SAME closed-loop steady observable as the coefficient
    # check and fail for the SAME reason: under FBWA the surface returns
    # toward neutral once the commanded bank/pitch is captured, so by the
    # last 75% of the input window the deflection delta has decayed below the
    # 0.5 deg floor (sign "0") or reversed. They are re-judged at control
    # onset, against the SAME pre-registered signs and the SAME floor.
    #
    # DECLARED CIRCULARITY: control onset is DEFINED by the axis deflection
    # exceeding DEFL_FLOOR_DEG, so at that sample the MAGNITUDE part of the
    # axis's own deflection check is satisfied by construction. Only its SIGN
    # is genuinely under test there. The per-surface joint checks and the
    # opposite/same-direction pair checks are NOT circular, because onset is
    # set by the aero-input deflection, not by the individual joints.
    o1b = {"status": "NOT_APPLICABLE_NO_ONSET"}
    if onset_i is not None:
        r = during[onset_i]

        def jdeg(name):
            v = get(r, ["actual_rad", name])
            b = mean([get(x, ["actual_rad", name]) for x in before])
            return (math.degrees(v) - math.degrees(b)) if v is not None else None

        def adeg(name):
            v = get(r, ["aero_input_rad", name])
            b = mean([get(x, ["aero_input_rad", name]) for x in before])
            return (math.degrees(v) - math.degrees(b)) if v is not None else None
        o1b = {"status": "JUDGED", "sample_index_in_input_window": onset_i,
               "joint_delta_deg": {n: jdeg(n) for n in L.SURFACES},
               "aero_input_delta_deg": {n: adeg(n)
                                        for n in ("delta_a", "delta_e", "delta_r")},
               "circularity_declaration":
                   f"onset was defined by |{dkey}| exceeding "
                   f"{DEFL_FLOOR_DEG} deg, so the MAGNITUDE of the {dkey} "
                   f"check is satisfied by construction here; only its SIGN "
                   f"is under test."}

    # ---- corrected verdict ------------------------------------------------
    # Every check whose observable is a STEADY closed-loop quantity is
    # re-evaluated at control onset. The RATE checks are left exactly as the
    # harness computed them (they are transient peaks already, and roll/pitch
    # feedback is a damping term that cannot reverse a control sign).
    STEADY_OBSERVABLE_CHECKS = {
        "roll": {"left_aileron_joint_delta_deg": ("joint", "left_aileron"),
                 "right_aileron_joint_delta_deg": ("joint", "right_aileron"),
                 "delta_a_aero_deg": ("aero_in", "delta_a"),
                 "ailerons_deflect_in_OPPOSITE_directions": ("pair_opp", None)},
        "pitch": {"delta_e_aero_deg": ("aero_in", "delta_e"),
                  "elevators_deflect_in_the_SAME_direction": ("pair_same", None)},
        "yaw": {"rudder_joint_delta_deg": ("joint", "rudder"),
                "delta_r_aero_deg": ("aero_in", "delta_r")},
    }[axis]
    corrected = []
    for c in old_checks:
        if c.get("pass") is None:
            continue
        nm = c["check"]
        if nm == ckey:
            corrected.append({"check": ckey + " (RE-JUDGED at control onset)",
                              "old_pass": c["pass"], "pass": o1.get("pass"),
                              "value": o1.get(f"delta_{ckey}_vs_baseline")})
        elif nm in STEADY_OBSERVABLE_CHECKS and o1b.get("status") == "JUDGED":
            kind, arg = STEADY_OBSERVABLE_CHECKS[nm]
            if kind == "joint":
                v = o1b["joint_delta_deg"][arg]
                ok = sgn(v, DEFL_FLOOR_DEG) == c["expected_sign"]
            elif kind == "aero_in":
                v = o1b["aero_input_delta_deg"][arg]
                ok = sgn(v, DEFL_FLOOR_DEG) == c["expected_sign"]
            elif kind == "pair_opp":
                a = o1b["joint_delta_deg"]["left_aileron"]
                b = o1b["joint_delta_deg"]["right_aileron"]
                v = [a, b]
                ok = bool(a * b < 0.0)
            else:  # pair_same
                a = o1b["joint_delta_deg"]["left_elevator"]
                b = o1b["joint_delta_deg"]["right_elevator"]
                v = [a, b]
                ok = bool(a * b > 0.0)
            corrected.append({"check": nm + " (RE-JUDGED at control onset)",
                              "old_pass": c["pass"], "pass": ok, "value": v,
                              "expected_sign": c.get("expected_sign")})
        else:
            corrected.append({"check": nm, "old_pass": c["pass"],
                              "pass": c["pass"], "value": c.get("measured")})
    new_verdict = ("PASS" if corrected and all(c["pass"] for c in corrected)
                   else "SEE_CHECKS")

    return {
        "scenario": key,
        "name": P2.SCENARIOS[key]["name"],
        "axis": axis,
        "status": "RE_JUDGED",
        "re_flown": False,
        "source_timeseries": ts_path,
        "source_result": rs_path,
        "pre_registered_expected_sign": expected_sign,
        "pre_registered_statement": pre.get(AXIS_EXPECT[axis]),
        "baseline": {f"{dkey}_deg": base_defl_deg, f"{ckey}": base_coef,
                     "beta_deg": base_beta_deg},
        "ORIGINAL_judgement": {
            "observable": "mean coefficient over the LAST 75% of the input "
                          "window, minus baseline (closed-loop equilibrium)",
            "AS_STORED_in_result_json": {
                "check": stored,
                "scenario_verdict": stored_verdict,
                "n_checks_pass": rs["result"]["n_checks_pass"],
                "n_checks_total": rs["result"]["n_checks_total"],
                "note": "the coefficient check was UNJUDGED (pass=None) at "
                        "capture time and did not count toward this verdict",
            },
            "CURRENT_HARNESS_CODE_on_the_same_data": {
                "check": old,
                "scenario_verdict": old_verdict,
                "n_checks_pass": replay["n_checks_pass"],
                "n_checks_total": replay["n_checks_total"],
            },
        },
        "O1_at_control_onset": o1,
        "O1b_deflections_at_control_onset": o1b,
        "O2_signed_extremum_of_first_excursion": o2,
        "O3_isolated_control_term_lookup": o3,
        "yaw_feedback_decomposition": decomp,
        "CORRECTED_scenario_verdict": new_verdict,
        "corrected_check_list": corrected,
        "beta_guard_deg": BETA_GUARD_DEG,
        "beta_guard_basis": BETA_GUARD_BASIS,
        "floors_used": {"min_surface_deflection_deg": DEFL_FLOOR_DEG,
                        "min_rate_response_deg_s": RATE_FLOOR_DEG_S,
                        "min_coefficient_magnitude": COEF_FLOOR,
                        "source": "IMPORTED from "
                                  "test_control_surface_live_sign_scenarios.TH "
                                  "- unchanged"},
    }


def aero_cross_check_status():
    """The cross-check the previous report listed as BLOCKED. aero_lib was
    re-synced by `aerodynamics`; re-run it and report whatever comes back."""
    try:
        import aero_lib as AL
        cfg = AL.load_config()
    except Exception as e:
        return {"status": "STILL_BLOCKED", "reason": repr(e)}
    out = {"status": "RUNNABLE", "load_config": "OK", "per_scenario": {}}
    for key in sorted(P2.SCENARIOS):
        p = os.path.join(RES, f"{PREFIX}_{key}_timeseries.json")
        if not os.path.exists(p):
            continue
        ts = json.load(open(p, encoding="utf-8"))
        rl, rm, rn = [], [], []
        for r in ts["windows"]["during"]:
            a = r.get("aero")
            att = r.get("attitude")
            d = r.get("aero_input_rad")
            if not a or not att or not d or a.get("V") is None or a["V"] < 1.0:
                continue
            V, al, be = a["V"], a["alpha"], a["beta"]
            u = V * math.cos(al) * math.cos(be)
            v = V * math.sin(be)
            w = -V * math.sin(al) * math.cos(be)
            pr = math.radians(att["rollspeed_deg_s"])
            q = -math.radians(att["pitchspeed_deg_s"])
            rr = -math.radians(att["yawspeed_deg_s"])
            got = AL.compute_aero(cfg, u, v, w, pr, q, rr,
                                  deltaA=d["delta_a"], deltaE=d["delta_e"],
                                  deltaR=d["delta_r"])
            rl.append(got["Cl"] - a["Cl"])
            rm.append(got["Cm"] - a["Cm"])
            rn.append(got["Cn"] - a["Cn"])
        if not rl:
            out["per_scenario"][key] = {"status": "NO_SAMPLE_ABOVE_1_MS_FLOOR"}
            continue

        def stat(xs):
            return {"max_abs": max(abs(x) for x in xs), "mean": mean(xs),
                    "rms": math.sqrt(sum(x * x for x in xs) / len(xs))}
        out["per_scenario"][key] = {"n": len(rl), "residual_Cl": stat(rl),
                                    "residual_Cm": stat(rm),
                                    "residual_Cn": stat(rn)}
    out["method"] = (
        "aero_lib.compute_aero() fed with the aerodynamics plugin's OWN "
        "published V/alpha/beta and the deflections reconstructed from the "
        "actuator plugin's ACTUAL joint angles, compared against the same "
        "message's published Cl/Cm/Cn. Body rates come from MAVLink ATTITUDE "
        "(FRD) converted to FLU - an EKF-filtered estimate, not the plugin's "
        "own instantaneous rate - so a small residual from the rate terms is "
        "EXPECTED and is not evidence of a mapping error.")
    return out


def main():
    runs = [rejudge(k) for k in sorted(P2.SCENARIOS)]
    table = []
    for r in runs:
        if r.get("status") != "RE_JUDGED":
            table.append({"scenario": r["scenario"], "status": r.get("status")})
            continue
        table.append({
            "scenario": r["scenario"], "name": r["name"],
            "coefficient": AXIS_COEF[r["axis"]],
            "expected_sign": r["pre_registered_expected_sign"],
            "OLD_observable_value": (r["ORIGINAL_judgement"]["CURRENT_HARNESS_CODE_on_the_same_data"]["check"] or {}).get("measured"),
            "OLD_sign": (r["ORIGINAL_judgement"]["CURRENT_HARNESS_CODE_on_the_same_data"]["check"] or {}).get("measured_sign"),
            "OLD_pass": (r["ORIGINAL_judgement"]["CURRENT_HARNESS_CODE_on_the_same_data"]["check"] or {}).get("pass"),
            "OLD_scenario_verdict": r["ORIGINAL_judgement"]["CURRENT_HARNESS_CODE_on_the_same_data"]["scenario_verdict"],
            "AS_STORED_pass": (r["ORIGINAL_judgement"]["AS_STORED_in_result_json"]["check"] or {}).get("pass"),
            "AS_STORED_verdict": r["ORIGINAL_judgement"]["AS_STORED_in_result_json"]["scenario_verdict"],
            "NEW_O1_value": r["O1_at_control_onset"].get(
                "delta_%s_vs_baseline" % AXIS_COEF[r["axis"]]),
            "NEW_O1_sign": r["O1_at_control_onset"].get("measured_sign"),
            "NEW_O1_pass": r["O1_at_control_onset"].get("pass"),
            "NEW_O2_pass": r["O2_signed_extremum_of_first_excursion"].get("pass"),
            "NEW_O3_pass_at_onset": r["O3_isolated_control_term_lookup"].get("pass_at_onset"),
            "NEW_O3_pass_steady": r["O3_isolated_control_term_lookup"].get("pass_steady"),
            "CORRECTED_scenario_verdict": r["CORRECTED_scenario_verdict"],
        })
    out = {
        "stage": "MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION",
        "part": "2 - RE-JUDGEMENT of scenarios A-F on existing captured data",
        "owner": "gazebo-testing",
        "re_flown": False,
        "physics_parameters_changed": "NONE",
        "thresholds_changed": "NONE (all floors imported from the original "
                              "harness's TH block)",
        "expectations_changed": "NONE (all signs read from "
                                "manual_takeoff_live_lib."
                                "PRE_REGISTERED_EXPECTATIONS)",
        "why": "The original judgement used a CLOSED-LOOP EQUILIBRIUM "
               "observable. `aerodynamics` verified the plugin itself is "
               "correct against a C++ oracle; the defect is the harness's "
               "choice of observable, which is gazebo-testing's to fix.",
        "summary_table": table,
        "aero_deflection_cross_check": aero_cross_check_status(),
        "known_unfixed_defect_reported_not_touched": {
            "file": "tests/gazebo/scripts/test_control_authority_effectiveness.py",
            "line": 629,
            "symptom": "prints CFG.controlDeflectionClamp, an attribute that "
                       "no longer exists since the 2026-08-26 retirement of "
                       "control_deflection_clamp_deg from "
                       "docs/source_of_truth/aerodynamics/aero_v1_config.yaml. "
                       "It previously died earlier inside load_config(); now "
                       "that load_config() works again it will reach this "
                       "line and raise AttributeError.",
            "action_taken": "NONE - reported only, out of scope for this stage.",
        },
        "scenarios": runs,
    }
    dst = os.path.join(RES, "control_surface_live_sign_rejudgement.json")
    with open(dst, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    print("wrote", dst)
    print(json.dumps(table, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

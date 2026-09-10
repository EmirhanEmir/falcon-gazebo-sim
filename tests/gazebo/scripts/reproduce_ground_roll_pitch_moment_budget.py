#!/usr/bin/env python3
"""Reproduce the ground-roll pitch-rotation moment budget of stage
MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION (2026-09-10)
from the stored TAKEOFF timeseries.

Why this script exists
----------------------
`validation` finding m3: the moment budget quoted in
docs/test_results/2026-09-10_manual_takeoff_and_live_control_surface_behavior_validation.md
(section 3.2a) was computed ad hoc and never persisted, so the numbers
0.695189 m / 47.24 N.m / 45.66 N.m could not be re-derived from the repository.
This script persists that derivation and additionally adds the D'Alembert
(linear-acceleration) term that the original budget omitted.

Scope and limits
----------------
- READ ONLY with respect to every engineering artifact. No simulation is run,
  no parameter is read-modified-written, nothing under docs/source_of_truth/ or
  model/ or plugins/ is touched.
- This is an ANALYSIS of an existing measurement. It is NOT the primary
  evidence for the "no rotation" result. The primary evidence is the measured
  pitch trace (std 0.28-0.35 deg for 13.9 s of a 14.3 s roll under a saturated
  nose-up elevator). This budget is only "consistent with" that measurement --
  see the margin-vs-sensitivity comparison it prints.

Physics / sign convention
-------------------------
Body frame is FLU (+X fwd, +Y left, +Z up), per CLAUDE.md.
In FLU a POSITIVE moment about +Y is NOSE DOWN
(plugins/aerodynamics/AeroModel.hh, axis-rotation table at line 316).
Everything below is expressed as a NOSE-UP magnitude, i.e. as (-M_y), so a
positive number always means "helps the nose come up".

Pivot: the aircraft has no landing gear. It rests on flat-bottomed fuselage
collision boxes whose bottom face is at link z = 0 and whose aft edge is at
link x = -0.526880 m. That aft edge is the only pivot available for a nose-up
rotation, so all moments are taken about
    P = (-0.526880, 0, 0)   [link frame]

With r = CG - P and M_y = r_z*F_x - r_x*F_z, the nose-up magnitude (-M_y) of
each contribution is:
    lift        L    at CoM      ->  +L  * dx
    weight      -m g at CoM      ->  -m g* dx        (nose down)
    drag        -D   at CoM      ->  +D  * dz
    thrust      +T   at hub      ->  -T  * h_hub     (nose down)
    aero pitching moment (pure couple, frame independent)
    D'Alembert  -m a_x at CoM    ->  +m a_x * dz     (nose up while accelerating)

The aero force is applied at the link centre of mass and the aero moment as a
pure couple (AerodynamicsSystem.cc AddWorldWrench with zero offset), so the
aero force arm is exactly dx and the couple needs no arm.

Cm sign correction (validation finding M2)
------------------------------------------
The stage report's section 0 stated "My = qbar*S*c_ref*Cm, positive Cm = nose
up". That is wrong twice. AeroModel.hh:705 actually computes

    my = qbar * S * c_ref * (-cmStatic + cmRate)

with out.Cm = cmStatic + cmRate (AeroModel.hh:646-648), and +My is NOSE DOWN.
So the correct nose-up aero pitching moment is

    nose_up_aero = -my = qbar * S * c_ref * (cmStatic - cmRate)
                       = qbar * S * c_ref * (Cm - 2*cmRate)

The two errors (the negation, and the +My sense) cancel for the static part,
which is why the published Cm-only budget is numerically almost right. This
script computes BOTH forms and prints the difference so the cancellation is
visible rather than assumed. cmRate = Cmq * qHat is reconstructed from the
configured Cmq and the measured pitch rate.

Provenance of every constant used
---------------------------------
  mass 6.000 kg                        CLAUDE.md "Aircraft mass"
  g 9.81 m/s^2                         world SDF <gravity>, echoed in each
                                       timeseries under world_constants
  S 0.4514 m^2                         CLAUDE.md manufacturer geometry
  c_ref 0.224 m                        docs/source_of_truth/aerodynamics/
                                       aero_v1_config.yaml reference_chord_c_ref_m
  Cmq -10.22875                        same file, CONFIRMED
  CG (0.168309, 0, 0.100000) m         CLAUDE.md "Current Gazebo/CAD CG"
  pivot x -0.526880 m                  model/model.sdf fuselage collision boxes,
                                       aft edge (read only)
  hub height 0.1271 m                  model/model.sdf motor/prop hub z (read only)
  a_x                                  DERIVED here: 0.5 s central difference of
                                       VFR_HUD groundspeed. Declared ASSUMPTION:
                                       the along-track acceleration is used as
                                       the body-X acceleration. Valid while the
                                       aircraft is tracking straight; during the
                                       yawed slide it is an approximation, and
                                       it is reported as such.

Usage
-----
  python3 tests/gazebo/scripts/reproduce_ground_roll_pitch_moment_budget.py

Writes tests/gazebo/results/ground_roll_pitch_moment_budget.json
Owner: controls-integration.
"""

from __future__ import annotations

import json
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
RESULTS = os.path.join(REPO, "tests", "gazebo", "results")

# --- constants, all with provenance in the module docstring -----------------
MASS_KG = 6.0
G_MPS2 = 9.81
S_M2 = 0.4514
C_REF_M = 0.224
CMQ = -10.22875
CG_LINK_M = (0.168309, 0.0, 0.100000)
PIVOT_LINK_M = (-0.526880, 0.0, 0.0)
HUB_HEIGHT_M = 0.1271

DX_M = CG_LINK_M[0] - PIVOT_LINK_M[0]   # 0.695189 m, CG forward of the pivot
DZ_M = CG_LINK_M[2] - PIVOT_LINK_M[2]   # 0.100000 m, CG above the pivot

# in-contact test: belly flat on the ground. The roll-tip lifts the link origin
# by 0.28*sin(roll), so any z above this floor means the tip has begun.
CONTACT_Z_FLOOR_M = 1.0e-4
AX_WINDOW_S = 0.5                       # central-difference half-width*2

RUNS = (1, 2, 3)


def _load(run: int):
    path = os.path.join(RESULTS, f"manual_takeoff_ground_roll_run{run}_TAKEOFF_timeseries.json")
    with open(path) as fh:
        return path, json.load(fh)


def _window(times, i, i_first, window_s=AX_WINDOW_S):
    """Indices bracketing a `window_s` central difference around sample i.

    The pose/telemetry stream is wall-clock sampled with jitter, so a 2-sample
    difference is unusably noisy. A 0.5 s window is used for every rate derived
    in this script, for both a_x and q.
    """
    t0 = times[i]
    lo = min(range(i_first, i + 1), key=lambda j: abs(times[j] - (t0 - window_s / 2.0)))
    hi = min(range(i, len(times)), key=lambda j: abs(times[j] - (t0 + window_s / 2.0)))
    return lo, hi


def _central_diff_ax(samples, times, i, i_first, window_s=AX_WINDOW_S):
    """0.5 s central difference of VFR_HUD groundspeed -> along-track a_x."""
    lo, hi = _window(times, i, i_first, window_s)
    if times[hi] == times[lo]:
        return 0.0
    return (samples[hi]["vfr"]["groundspeed"] - samples[lo]["vfr"]["groundspeed"]) / (times[hi] - times[lo])


def _central_diff_q(samples, times, i, i_first, window_s=AX_WINDOW_S):
    """0.5 s central difference of the Gazebo pose pitch -> q in rad/s (FLU).

    Gazebo pose pitch is the negative of true nose-up pitch; the plugin reads the
    same raw FLU q, so no sign flip is applied here.
    """
    lo, hi = _window(times, i, i_first, window_s)
    if times[hi] == times[lo]:
        return 0.0
    d_deg = samples[hi]["gz_rpy_deg"][1] - samples[lo]["gz_rpy_deg"][1]
    return d_deg * 3.141592653589793 / 180.0 / (times[hi] - times[lo])


def analyse_run(run: int):
    path, doc = _load(run)
    samples = doc["samples"]
    times = [s["t_rel_sim"] for s in samples]

    i_thr = next(i for i, s in enumerate(samples) if s["vfr"]["throttle_pct"] > 0)
    i_end = next(i for i in range(i_thr, len(samples))
                 if samples[i]["gz_pos_world_m"][2] > CONTACT_Z_FLOOR_M)
    t_thr = times[i_thr]

    rows = []
    for i in range(i_thr, i_end):
        s = samples[i]
        aero = s["aero"]
        V = max(aero["V"], 1e-6)

        q_rad_s = _central_diff_q(samples, times, i, i_thr)
        cm_rate = CMQ * (q_rad_s * C_REF_M / (2.0 * V))
        cm_static = aero["Cm"] - cm_rate

        qSc = aero["qbar"] * S_M2 * C_REF_M
        # what the stage report used (literal published Cm):
        m_aero_report = qSc * aero["Cm"]
        # what AeroModel.hh:705 actually applies, as a nose-up magnitude:
        m_aero_correct = qSc * (cm_static - cm_rate)

        L = s["lift_N"]
        D = s["drag_N"]
        T = s["thrust_total_N"]

        nose_up_static = m_aero_report + L * DX_M + D * DZ_M
        nose_down = MASS_KG * G_MPS2 * DX_M + T * HUB_HEIGHT_M
        margin_static = nose_up_static - nose_down

        a_x = _central_diff_ax(samples, times, i, i_thr)
        m_dalembert = MASS_KG * a_x * DZ_M
        margin_with_inertia = margin_static + m_dalembert

        rows.append({
            "index": i,
            "t_thr_s": times[i] - t_thr,
            "V_ms": aero["V"],
            "Cm_published": aero["Cm"],
            "cm_rate": cm_rate,
            "aero_moment_report_Nm": m_aero_report,
            "aero_moment_AeroModel_correct_Nm": m_aero_correct,
            "aero_moment_correction_Nm": m_aero_correct - m_aero_report,
            "lift_N": L,
            "drag_N": D,
            "thrust_total_N": T,
            "a_x_mps2": a_x,
            "nose_up_available_static_Nm": nose_up_static,
            "nose_down_required_Nm": nose_down,
            "margin_static_only_Nm": margin_static,
            "dalembert_nose_up_Nm": m_dalembert,
            "margin_with_inertia_Nm": margin_with_inertia,
        })

    best_static = max(rows, key=lambda r: r["margin_static_only_Nm"])
    best_inertia = max(rows, key=lambda r: r["margin_with_inertia_Nm"])

    # single-parameter sensitivities, evaluated at the best inertia-corrected
    # sample (the least unfavourable instant of the whole roll)
    b = best_inertia
    sens = {
        "pivot_moved_aft_1cm_Nm": (b["lift_N"] - MASS_KG * G_MPS2) * 0.01,
        "hub_height_raised_1cm_Nm": -b["thrust_total_N"] * 0.01,
        "Cm_plus_5pct_Nm": 0.05 * b["aero_moment_report_Nm"],
        "a_x_plus_10pct_Nm": 0.10 * b["dalembert_nose_up_Nm"],
    }

    early = min((r for r in rows if r["t_thr_s"] >= 1.0), key=lambda r: r["t_thr_s"])
    max_dalembert = max(rows, key=lambda r: r["dalembert_nose_up_Nm"])

    return {
        "run_index": run,
        "source_timeseries": os.path.relpath(path, REPO),
        "n_in_contact_samples": len(rows),
        "in_contact_window": {
            "throttle_onset_index": i_thr,
            "first_z_above_floor_index": i_end,
            "t_thr_end_s": times[i_end] - t_thr,
            "contact_z_floor_m": CONTACT_Z_FLOOR_M,
        },
        "best_case_static_only": best_static,
        "best_case_with_dalembert": best_inertia,
        "dalembert_early_sample": {"t_thr_s": early["t_thr_s"], "a_x_mps2": early["a_x_mps2"],
                                   "dalembert_nose_up_Nm": early["dalembert_nose_up_Nm"]},
        "dalembert_max_over_roll": {"t_thr_s": max_dalembert["t_thr_s"],
                                    "a_x_mps2": max_dalembert["a_x_mps2"],
                                    "dalembert_nose_up_Nm": max_dalembert["dalembert_nose_up_Nm"]},
        "max_abs_aero_moment_correction_Nm": max(abs(r["aero_moment_correction_Nm"]) for r in rows),
        "max_abs_cm_rate": max(abs(r["cm_rate"]) for r in rows),
        "single_parameter_sensitivities_at_best_case": sens,
        "margin_negative_at_every_sample_static_only": all(r["margin_static_only_Nm"] < 0 for r in rows),
        "margin_negative_at_every_sample_with_dalembert": all(r["margin_with_inertia_Nm"] < 0 for r in rows),
    }


def main() -> int:
    out = {
        "analysis": "ground_roll_pitch_rotation_moment_budget",
        "stage": "MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION",
        "date": "2026-09-10",
        "owner": "controls-integration",
        "reason": "validation finding m3 - persist the section 3.2a budget; validation finding M3 - add the omitted D'Alembert term",
        "physics_parameters_changed": False,
        "simulation_rerun": False,
        "frames": {
            "body": "FLU (+X fwd, +Y left, +Z up), CLAUDE.md",
            "moment_sign": "reported as NOSE-UP magnitude = -M_y; +M_y is NOSE DOWN in FLU (AeroModel.hh:316)",
        },
        "constants": {
            "mass_kg": MASS_KG, "g_mps2": G_MPS2, "S_m2": S_M2, "c_ref_m": C_REF_M,
            "Cmq": CMQ, "cg_link_m": list(CG_LINK_M), "pivot_link_m": list(PIVOT_LINK_M),
            "dx_m": DX_M, "dz_m": DZ_M, "hub_height_m": HUB_HEIGHT_M,
            "provenance": "see module docstring; every value is CAD/CLAUDE.md/aero_v1_config.yaml, none is new",
        },
        "a_x_method": {
            "definition": "0.5 s central difference of VFR_HUD groundspeed",
            "label": "ASSUMPTION - along-track acceleration used as body-X acceleration; exact while tracking straight, approximate during the yawed slide",
        },
        "interpretation": (
            "The budget is CONSISTENT WITH the measured absence of rotation; it is not "
            "decisive on its own. With the D'Alembert term included the best-case margin "
            "is 1.5-2.8% of the ~47 N.m budget, which is inside the span of the "
            "single-parameter sensitivities reported per run. The primary evidence for "
            "24MPS_LIFTOFF = GROUND_MODEL_ARTIFACT is the measured pitch trace, not this budget."
        ),
        "runs": [analyse_run(r) for r in RUNS],
    }

    dest = os.path.join(RESULTS, "ground_roll_pitch_moment_budget.json")
    with open(dest, "w") as fh:
        json.dump(out, fh, indent=1)

    print(f"wrote {dest}")
    for r in out["runs"]:
        bs = r["best_case_static_only"]
        bi = r["best_case_with_dalembert"]
        print(f"run {r['run_index']}: n_in_contact={r['n_in_contact_samples']}  dx={DX_M:.6f} m")
        print(f"  best case, STATIC ONLY   : t-thr {bs['t_thr_s']:.2f} s  "
              f"up {bs['nose_up_available_static_Nm']:.2f}  down {bs['nose_down_required_Nm']:.2f}  "
              f"margin {bs['margin_static_only_Nm']:+.3f} N.m")
        print(f"  best case, + D'ALEMBERT  : t-thr {bi['t_thr_s']:.2f} s  "
              f"static {bi['margin_static_only_Nm']:+.3f}  dalembert {bi['dalembert_nose_up_Nm']:+.3f}  "
              f"margin {bi['margin_with_inertia_Nm']:+.3f} N.m")
        print(f"  D'Alembert early (t-thr {r['dalembert_early_sample']['t_thr_s']:.2f} s): "
              f"{r['dalembert_early_sample']['dalembert_nose_up_Nm']:+.3f} N.m ; "
              f"max over roll {r['dalembert_max_over_roll']['dalembert_nose_up_Nm']:+.3f} N.m")
        print(f"  Cm-formula correction (M2): max |delta My| = "
              f"{r['max_abs_aero_moment_correction_Nm']:.2e} N.m, max |cmRate| = {r['max_abs_cm_rate']:.2e}")
        s = r["single_parameter_sensitivities_at_best_case"]
        print(f"  sensitivities: pivot +1cm {s['pivot_moved_aft_1cm_Nm']:+.3f} | "
              f"hub +1cm {s['hub_height_raised_1cm_Nm']:+.3f} | "
              f"Cm +5% {s['Cm_plus_5pct_Nm']:+.3f} | a_x +10% {s['a_x_plus_10pct_Nm']:+.3f} N.m")
        print(f"  margin < 0 at EVERY in-contact sample: static {r['margin_negative_at_every_sample_static_only']}, "
              f"with D'Alembert {r['margin_negative_at_every_sample_with_dalembert']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
FALCON V2 - re-derive the published step-5 figures FROM THE COMMITTED
EXCERPTS ALONE. Owner: gazebo-testing, 2026-09-13.

This script deliberately does NOT read the 1 kHz scratchpad originals and
does NOT import analyze_airborne_limit_cycle_runtime.py's loader. It opens
EXACTLY three files - tests/gazebo/results/actuator_fix_airborne_{MANUAL,FBWA,
YAW}_1khz_excerpt.json - and nothing else (the YAW excerpt embeds its own
phase markers so the ladder segmentation needs no companion file). It inverts
the delta/fixed-point encoding with the routine documented inside those
files, and recomputes the headline numbers of
docs/test_results/2026-09-12_actuator_limit_cycle_fix_verification.md.

Its purpose is to answer one question: if the scratchpad disappears, can the
published result still be checked against raw data? Every value it prints is
compared against the figure published in the report, and the difference is
shown. MEASUREMENT-ONLY, READ-ONLY.
"""
import json
import math
import os

import numpy as np

RES = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
MAX_RATE_RAD_S = 5.2360          # actuator_v1_config.yaml, V1_PROVISIONAL
RATE_PIN_FRACTION = 0.999        # imported verbatim, 2026-09-10 offline analysis
CMD_CALM_P2P_DEG = 1.0           # imported verbatim
ACT_OSC_P2P_DEG = 3.0            # imported verbatim
CALM_WINDOW_S = 1.0              # imported verbatim
LEADIN_S = 1.0                   # the criterion added in report section 7.5


def deq(d):
    """Inverse of build_actuator_fix_1khz_excerpts.quantize(), written out
    from the `encoding` string stored in the file itself."""
    if "constant" in d:
        return np.full(d["n"], d["constant"], float)
    q = np.empty(d["n"], np.int64)
    q[0] = d["q0"]
    if d["n"] > 1:
        q[1:] = d["q0"] + np.cumsum(np.asarray(d["dq"], np.int64))
    return q * d["scale"]


def load(tag):
    p = os.path.join(RES, f"actuator_fix_airborne_{tag}_1khz_excerpt.json")
    E = json.load(open(p))
    t = deq(E["actuator_t_sim"])
    ch = {s: {f: deq(v) for f, v in E["actuator"][s].items()}
          for s in E["actuator_field_order"]["surfaces"]}
    return E, t, ch, E["actuator"]


def p2p(x):
    return float(np.max(x) - np.min(x))


ROWS = []


def cmp(label, derived, published, tol, unit="", chan=None, chan_deg=False):
    """`tol` is the precision the report itself quotes the figure to. Where a
    channel's own DOCUMENTED quantization is coarser than that, the channel's
    `worst_case_roundtrip_abs_error` - read out of the committed file, not
    chosen here - is used as the floor instead. That is a tolerance derived
    from the artifact, not one tuned until a check passed."""
    floor = 0.0
    if chan is not None and "worst_case_roundtrip_abs_error" in chan:
        floor = chan["worst_case_roundtrip_abs_error"]
        if chan_deg:
            floor = math.degrees(floor)
    tol = max(tol, floor)
    d = abs(derived - published)
    ok = d <= tol
    ROWS.append((label, derived, published, d, ok, unit))
    print(f"{'OK ' if ok else 'FAIL'}  {label:<58} "
          f"derived={derived:.6f}{unit}  published={published:.6f}{unit}  "
          f"diff={d:.2e}  tol={tol:.2e}")
    return ok


def main():
    allok = True
    print("=" * 118)
    print("RE-DERIVATION FROM THE COMMITTED EXCERPTS ONLY "
          "(scratchpad originals not opened)")
    print("=" * 118)

    # ---------------- MANUAL ----------------
    E, t, ch, meta = load("MANUAL")
    print(f"\n-- MANUAL  window {E['window_sim_s']}  "
          f"n_act={E['n_actuator_samples']}  "
          f"rate={E['achieved_actuator_rate_hz']:.1f} Hz")
    allok &= cmp("P4 left_aileron uncommanded actual p2p",
                 math.degrees(p2p(ch["left_aileron"]["actual_angle_rad"])),
                 0.3905, 5e-4, " deg")
    allok &= cmp("P4 left_elevator uncommanded actual p2p",
                 math.degrees(p2p(ch["left_elevator"]["actual_angle_rad"])),
                 0.6081, 5e-4, " deg")
    allok &= cmp("P11 rudder uncommanded actual p2p",
                 math.degrees(p2p(ch["rudder"]["actual_angle_rad"])),
                 0.0025, 5e-4, " deg")
    allok &= cmp("P4 left_aileron cmd unique values (bit-constant command)",
                 float(np.unique(np.round(ch["left_aileron"]["cmd_rad"], 12)).size),
                 1.0, 0.0, "")
    pin = float(np.mean(np.abs(ch["left_elevator"]["actual_rate_rad_s"])
                        >= RATE_PIN_FRACTION * MAX_RATE_RAD_S)) * 100.0
    allok &= cmp("P6 left_elevator rate-pinned duty", pin, 0.000, 1e-9, " %")
    allok &= cmp("left_elevator max |joint rate|",
                 math.degrees(float(np.max(np.abs(
                     ch["left_elevator"]["actual_rate_rad_s"])))),
                 0.266, 5e-3, " deg/s")
    allok &= cmp("P13 left_elevator effort_clamp duty",
                 float(np.mean(ch["left_elevator"]["effort_clamp_active"] > 0.5)) * 100.0,
                 0.0, 1e-9, " %")
    g = np.asarray(E["imu_gyro_rad_s"], float)
    allok &= cmp("P14 body pitch-rate RMS (MANUAL, information)",
                 math.degrees(float(np.sqrt(np.mean(g[:, 1] ** 2)))),
                 31.824, 5e-3, " deg/s")
    V = deq(E["aero_V_ms_full_rate"])
    allok &= cmp("section 8 airspeed max", float(V.max()), 48.441, 5e-3, " m/s")
    allok &= cmp("section 8 airspeed min", float(V.min()), 18.497, 5e-3, " m/s")
    nn = sum(int(np.sum(~np.isfinite(v))) for s in ch for v in ch[s].values())
    nn += int(np.sum(~np.isfinite(g)))
    allok &= cmp("NaN/Inf count", float(nn), 0.0, 0.0, "")

    # ---------------- FBWA ----------------
    E, t, ch, meta = load("FBWA")
    print(f"\n-- FBWA  window {E['window_sim_s']}  "
          f"n_act={E['n_actuator_samples']}")
    allok &= cmp("P5 left_aileron uncommanded actual p2p",
                 math.degrees(p2p(ch["left_aileron"]["actual_angle_rad"])),
                 0.0988, 5e-4, " deg")
    allok &= cmp("P5 left_aileron cmd p2p",
                 math.degrees(p2p(ch["left_aileron"]["cmd_rad"])),
                 0.0643, 5e-4, " deg")
    allok &= cmp("P11 rudder uncommanded actual p2p",
                 math.degrees(p2p(ch["rudder"]["actual_angle_rad"])),
                 0.0015, 5e-4, " deg")
    allok &= cmp("left_elevator tracking-error p2p",
                 math.degrees(p2p(ch["left_elevator"]["actual_angle_rad"]
                                  - ch["left_elevator"]["setpoint_rad"])),
                 0.4919, 1e-3, " deg")
    pin = float(np.mean(np.abs(ch["left_elevator"]["actual_rate_rad_s"])
                        >= RATE_PIN_FRACTION * MAX_RATE_RAD_S)) * 100.0
    allok &= cmp("P6 left_elevator rate-pinned duty", pin, 0.000, 1e-9, " %")
    g = np.asarray(E["imu_gyro_rad_s"], float)
    allok &= cmp("P14 body pitch-rate RMS (FBWA, the stated prediction)",
                 math.degrees(float(np.sqrt(np.mean(g[:, 1] ** 2)))),
                 1.490, 5e-3, " deg/s")

    # ---------------- YAW ----------------
    E, t, ch, meta = load("YAW")
    print(f"\n-- YAW  window {E['window_sim_s']}  "
          f"n_act={E['n_actuator_samples']}")
    a = np.asarray(E["imu_accel_ms2"], float)
    g = np.asarray(E["imu_gyro_rad_s"], float)
    allok &= cmp("yaw excitation: lateral specific force |f_y| max",
                 float(np.max(np.abs(a[:, 1]))), 4.854, 5e-3, " m/s^2")
    allok &= cmp("yaw excitation: yaw-rate p2p",
                 math.degrees(p2p(g[:, 2])), 342.537, 5e-2, " deg/s")
    allok &= cmp("rudder max |joint rate| over the ladder",
                 math.degrees(float(np.max(np.abs(
                     ch["rudder"]["actual_rate_rad_s"])))),
                 247.912, 5e-2, " deg/s")
    pin = float(np.mean(np.abs(ch["rudder"]["actual_rate_rad_s"])
                        >= RATE_PIN_FRACTION * MAX_RATE_RAD_S)) * 100.0
    allok &= cmp("rudder rate-pinned duty over the ladder", pin, 0.000,
                 1e-9, " %")
    allok &= cmp("rudder travel extreme (max)",
                 math.degrees(float(np.max(ch["rudder"]["actual_angle_rad"]))),
                 45.000000, 1e-4, " deg",
                 chan=meta["rudder"]["actual_angle_rad"], chan_deg=True)
    allok &= cmp("rudder travel extreme (min)",
                 math.degrees(float(np.min(ch["rudder"]["actual_angle_rad"]))),
                 -45.000000, 1e-4, " deg",
                 chan=meta["rudder"]["actual_angle_rad"], chan_deg=True)
    beta = np.asarray(E["aero_other_fields_decimated"]["fields"]["beta"], float)
    allok &= cmp("sideslip |beta| max (100 Hz decimated channel)",
                 math.degrees(float(np.max(np.abs(beta)))), 57.10, 5e-2, " deg")

    # the section 7.5 command-calm census, rebuilt from the excerpt
    err = ch["rudder"]["actual_angle_rad"] - ch["rudder"]["setpoint_rad"]
    cmd = ch["rudder"]["cmd_rad"]
    mk = {m["label"]: m["t_sim"] for m in E["phase_markers_sim_s"]}
    t0, t1 = mk["ladder_start"], mk["ladder_end"]
    worst_raw = worst_lead = -1.0
    n_raw = n_lead = n_over = 0
    tt = t0
    while tt + CALM_WINDOW_S <= t1:
        m = (t >= tt) & (t < tt + CALM_WINDOW_S)
        mL = (t >= tt - LEADIN_S) & (t < tt + CALM_WINDOW_S)
        if m.sum() > 32:
            e = math.degrees(p2p(err[m]))
            if math.degrees(p2p(cmd[m])) <= CMD_CALM_P2P_DEG:
                n_raw += 1
                worst_raw = max(worst_raw, e)
            if mL.sum() and math.degrees(p2p(cmd[mL])) <= CMD_CALM_P2P_DEG:
                n_lead += 1
                worst_lead = max(worst_lead, e)
                if e > ACT_OSC_P2P_DEG:
                    n_over += 1
        tt += 0.25
    allok &= cmp("section 7.5 worst RAW-criterion calm tracking-error p2p",
                 worst_raw, 11.603, 5e-3, " deg")
    allok &= cmp("section 7.5 worst LEAD-IN calm tracking-error p2p",
                 worst_lead, 1.335, 5e-3, " deg")
    allok &= cmp("section 7.5 lead-in calm windows over the 3.0 deg bar",
                 float(n_over), 0.0, 0.0, "")
    allok &= cmp("section 7.5 lead-in calm window count", float(n_lead),
                 21.0, 0.0, "")

    print("\n" + "=" * 118)
    print(f"{sum(1 for r in ROWS if r[4])}/{len(ROWS)} re-derived figures "
          f"match the published report to their stated tolerance.")
    print("=" * 118)
    return 0 if allok else 1


if __name__ == "__main__":
    raise SystemExit(main())

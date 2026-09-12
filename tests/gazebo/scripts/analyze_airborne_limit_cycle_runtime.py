#!/usr/bin/env python3
"""FALCON V2 - AIRBORNE_ACTUATOR_LIMIT_CYCLE_ROOT_CAUSE, RUNTIME VERIFICATION.

Owner: gazebo-testing. Created 2026-09-11.
CLASSIFICATION: MEASUREMENT-ONLY / READ-ONLY.

Reads the records produced by
tests/gazebo/scripts/run_airborne_limit_cycle_runtime_verification.sh (this
stage's OWN live runs) and independently re-measures the claims of
docs/test_results/2026-09-10_airborne_actuator_limit_cycle_root_cause.md.

CHANGES NO PARAMETER. Launches no simulation. Writes exactly one result JSON.

EVERY THRESHOLD BELOW IS IMPORTED VERBATIM FROM THE 2026-09-10 OFFLINE
ANALYSIS SO THE TWO ARE DIRECTLY COMPARABLE. None was chosen by this stage,
and none was changed after a result was seen. The one criterion this stage
must define for itself (the airborne window) is stated up front and is
reported at three values so its sensitivity is visible.
"""
import json
import math
import os
import sys

import numpy as np

REPO = "/home/emirhan/Desktop/FalconV2"
RES = os.path.join(REPO, "tests/gazebo/results")
sys.path.insert(0, os.path.join(REPO, "tests/gazebo/scripts"))

# ---- constants READ from their documented sources (never invented here) ----
import yaml  # noqa: E402
_CFG = yaml.safe_load(open(os.path.join(
    REPO, "docs/source_of_truth/controls/actuator_v1_config.yaml")))
MAX_RATE_RAD_S = float(_CFG["surfaces"]["left_aileron"]["max_rate_rad_s"])
MAX_EFFORT_NM = float(_CFG["surfaces"]["left_aileron"]["max_effort_nm"])
MAX_ANGLE_RAD = float(_CFG["surfaces"]["left_aileron"]["max_angle_rad"])
KP = float(_CFG["control_law"]["kp_nm_per_rad"])

SURFACES = ["left_aileron", "right_aileron", "left_elevator",
            "right_elevator", "rudder"]
ACT_FIELDS = ["cmd_rad", "target_clamped_rad", "setpoint_rad",
              "actual_angle_rad", "actual_rate_rad_s", "target_clamp_active",
              "effort_clamp_active"]
AERO_FIELDS = ["V", "alpha", "beta", "qbar", "CL", "CD", "CY", "Cl", "Cm", "Cn"]

# ---- thresholds, IMPORTED from the 2026-09-10 offline analysis -------------
RATE_PIN_FRACTION = 0.999     # its ASSUMPTION T1
CLAMP_FLAG_TRUE = 0.5         # its ASSUMPTION T2
CALM_WINDOW_S = 1.0           # its ASSUMPTION T4
CMD_CALM_P2P_DEG = 1.0        # its ASSUMPTION T4
ACT_OSC_P2P_DEG = 3.0         # its ASSUMPTION T5

# ---- the ONE criterion this stage defines: the airborne window ------------
# ASSUMPTION (gazebo-testing, fixed before any result of this stage was
# inspected): airborne = Gazebo world z of the falcon_v2 model at or above
# AIRBORNE_Z_M, from the first such sample to the end of the record. 1.0 m is
# 10x the existing harness's own liftoff_z_margin_m = 0.10 m (which its
# author justified as "well above the contact-solver penetration/settling
# scale seen at rest"), so it cannot be satisfied by belly contact. Reported
# at 0.5 / 1.0 / 5.0 m so the sensitivity of every airborne-scoped number is
# visible.
AIRBORNE_Z_M = 1.0
AIRBORNE_Z_SENS = (0.5, 1.0, 5.0)


def p2p(x):
    x = np.asarray(x, float)
    return float(x.max() - x.min()) if x.size else float("nan")


def rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x * x))) if x.size else float("nan")


def dom_freq(t, y):
    """Dominant non-DC frequency of a uniformly sampled series, plus the
    fraction of total AC variance carried by that bin."""
    y = np.asarray(y, float)
    n = y.size
    if n < 16:
        return None, None, None
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        return None, None, None
    yy = y - y.mean()
    w = np.hanning(n)
    Y = np.abs(np.fft.rfft(yy * w))
    f = np.fft.rfftfreq(n, dt)
    Y[0] = 0.0
    k = int(np.argmax(Y))
    tot = float(np.sum(Y ** 2))
    return float(f[k]), (float(Y[k] ** 2 / tot) if tot > 0 else None), 1.0 / (2 * dt)


def zero_cross_freq(t, y):
    """Frequency from positive-going mean crossings - independent of the FFT."""
    y = np.asarray(y, float) - np.mean(y)
    idx = np.where((y[:-1] <= 0) & (y[1:] > 0))[0]
    if idx.size < 3:
        return None, None, int(idx.size)
    tc = []
    for i in idx:
        d = y[i + 1] - y[i]
        tc.append(t[i] + (0 - y[i]) / d * (t[i + 1] - t[i]) if d != 0 else t[i])
    per = np.diff(np.asarray(tc))
    return float(1.0 / np.mean(per)), float(np.std(per)), int(idx.size)


def band_energy(t, y, lo, hi):
    y = np.asarray(y, float)
    n = y.size
    dt = float(np.median(np.diff(t)))
    yy = (y - y.mean()) * np.hanning(n)
    P = np.abs(np.fft.rfft(yy)) ** 2
    f = np.fft.rfftfreq(n, dt)
    P[0] = 0.0
    tot = float(P.sum())
    if tot <= 0:
        return None
    m = (f >= lo) & (f < hi)
    return float(P[m].sum() / tot)


# ---------------------------------------------------------------------------
def load_highrate(path):
    """Returns dict with sim-time-stamped actuator / aero / imu arrays."""
    R = json.load(open(path))
    pairs = np.asarray(R["clock_pairs_wall_sim"], float)   # (wall, sim)
    # ---- wall -> sim map --------------------------------------------------
    # Gazebo publishes /clock ONCE PER PHYSICS STEP, so `pairs` is a dense
    # (1 kHz) sampling of the true wall<->sim relation. A single global
    # affine fit is NOT adequate here: the real-time factor is not constant
    # over a 260 s run, and the affine fit's own residual (computed and
    # reported below) reaches several seconds. The map actually used is
    # therefore a MONOTONE PIECEWISE-LINEAR INTERPOLATION of the clock
    # stream. The affine fit is retained only as the reported diagnostic
    # that justifies not using it.
    w0 = float(pairs[0, 0])
    A = np.vstack([pairs[:, 0] - w0, np.ones(pairs.shape[0])]).T
    coef, *_ = np.linalg.lstsq(A, pairs[:, 1], rcond=None)
    resid = pairs[:, 1] - A @ coef

    cw = np.maximum.accumulate(pairs[:, 0])   # enforce monotone wall axis
    cs = pairs[:, 1]

    def w2s(tw):
        return np.interp(np.asarray(tw, float), cw, cs)

    # self-consistency of the interpolating map: re-map the clock's own wall
    # stamps and compare against its own sim stamps, after the monotone
    # enforcement. Any residual here is transport re-ordering, not RTF drift.
    interp_resid = w2s(pairs[:, 0]) - cs
    clockmap = {
        "map_used": "monotone piecewise-linear interpolation of the /clock "
                    "stream (published once per physics step)",
        "n_clock_msgs": int(pairs.shape[0]),
        "clock_msg_rate_hz_measured": float(
            (pairs.shape[0] - 1) / (pairs[-1, 1] - pairs[0, 1])),
        "interp_residual_rms_s": float(np.sqrt(np.mean(interp_resid ** 2))),
        "interp_residual_max_abs_s": float(np.max(np.abs(interp_resid))),
        "rejected_global_affine_fit": {
            "slope_sim_per_wall": float(coef[0]),
            "intercept_sim_s_at_wall_origin": float(coef[1]),
            "wall_origin_unix_s": w0,
            "residual_rms_s": float(np.sqrt(np.mean(resid ** 2))),
            "residual_max_abs_s": float(np.max(np.abs(resid))),
            "why_rejected": "residual too large - the real-time factor "
                            "varies over the run, so a single line cannot "
                            "represent the wall<->sim relation.",
        },
    }

    act_tw = np.asarray([r["t_wall"] for r in R["actuator"]], float)
    act_v = np.asarray([r["v"] for r in R["actuator"]], float)
    aero_tw = np.asarray([r["t_wall"] for r in R["aero"]], float)
    aero_v = np.asarray([r["v"] for r in R["aero"]], float)
    imu_ts = np.asarray([r["t_stamp_s"] for r in R["imu"]], float)
    imu_g = np.asarray([r["gyro_rad_s"] for r in R["imu"]], float)
    imu_a = np.asarray([r["accel_ms2"] for r in R["imu"]], float)

    return {"clockmap": clockmap, "w2s": w2s,
            "act_t": w2s(act_tw), "act_v": act_v, "act_tw": act_tw,
            "aero_t": w2s(aero_tw), "aero_v": aero_v,
            "imu_t": imu_ts, "imu_gyro": imu_g, "imu_acc": imu_a,
            "subscribe_ok": R["subscribe_ok"]}


def surf(act_v, name, field):
    i = SURFACES.index(name)
    j = ACT_FIELDS.index(field)
    return act_v[:, i * 7 + j]


def analyse_one(tag, ts_path, hr_path):
    out = {"tag": tag, "timeseries": ts_path, "highrate_raw": hr_path}
    TS = json.load(open(ts_path))
    S = TS["samples"]
    ts_t = np.asarray([s["t_sim"] for s in S], float)
    ts_z = np.asarray([s["gz_pos_world_m"][2] for s in S], float)

    H = load_highrate(hr_path)
    out["clock_map"] = H["clockmap"]
    out["subscribe_ok"] = H["subscribe_ok"]

    # ---- measured publish rates -------------------------------------------
    def rate_of(t):
        if t.size < 3:
            return None
        return float((t.size - 1) / (t[-1] - t[0]))
    out["measured_publish_rate_hz"] = {
        "actuator_diagnostics": rate_of(H["act_t"]),
        "aero_diagnostics": rate_of(H["aero_t"]),
        "imu_sensor": rate_of(H["imu_t"]),
        "clock": H["clockmap"]["clock_msg_rate_hz_measured"],
    }
    dact = np.diff(H["act_t"])
    nomp = 1.0 / out["measured_publish_rate_hz"]["actuator_diagnostics"]
    out["actuator_delivery"] = {
        "n_messages": int(H["act_t"].size),
        "sim_span_s": float(H["act_t"][-1] - H["act_t"][0]),
        "median_gap_s": float(np.median(dact)),
        "p99_gap_s": float(np.percentile(dact, 99)),
        "max_gap_s": float(np.max(dact)),
        "gaps_over_3x_median": int(np.sum(dact > 3 * np.median(dact))),
        "note": "A message DROP by gz-transport would show as a gap that is "
                "an integer multiple of the median. Reported, not assumed "
                "absent.",
    }

    # ---- airborne window ---------------------------------------------------
    air = {}
    for zthr in AIRBORNE_Z_SENS:
        k = np.where(ts_z >= zthr)[0]
        air[str(zthr)] = (float(ts_t[k[0]]), float(ts_t[-1])) if k.size else None
    out["airborne_window_sim_s_by_z_threshold"] = air
    win = air[str(AIRBORNE_Z_M)]
    if win is None:
        out["status"] = "NO_AIRBORNE_SEGMENT"
        return out
    t0, t1 = win
    out["status"] = "AIRBORNE_SEGMENT_FOUND"
    out["airborne_window_sim_s"] = [t0, t1]
    out["airborne_duration_s"] = t1 - t0
    out["max_altitude_m"] = float(np.max(ts_z))

    am = (H["act_t"] >= t0) & (H["act_t"] <= t1)
    at = H["act_t"][am]
    av = H["act_v"][am, :]
    out["n_airborne_actuator_samples"] = int(at.size)

    im = (H["imu_t"] >= t0) & (H["imu_t"] <= t1)
    it = H["imu_t"][im]
    ig = H["imu_gyro"][im, :]

    em = (H["aero_t"] >= t0) & (H["aero_t"] <= t1)
    et = H["aero_t"][em]
    ev = H["aero_v"][em, :]

    # ---- per-surface census ------------------------------------------------
    pin_thr = RATE_PIN_FRACTION * MAX_RATE_RAD_S
    per = {}
    for s in SURFACES:
        ang = surf(av, s, "actual_angle_rad")
        rate = surf(av, s, "actual_rate_rad_s")
        cmd = surf(av, s, "cmd_rad")
        tcl = surf(av, s, "target_clamp_active")
        ecl = surf(av, s, "effort_clamp_active")
        f_fft, varfrac, nyq = dom_freq(at, ang)
        f_zc, zsd, nzc = zero_cross_freq(at, ang)
        pinned = int(np.sum(np.abs(rate) >= pin_thr))
        d = {
            "n_samples": int(ang.size),
            "cmd_p2p_deg": math.degrees(p2p(cmd)),
            "cmd_min_deg": math.degrees(float(cmd.min())),
            "cmd_max_deg": math.degrees(float(cmd.max())),
            "cmd_unique_values": int(np.unique(cmd).size),
            "actual_p2p_deg": math.degrees(p2p(ang)),
            "actual_min_deg": math.degrees(float(ang.min())),
            "actual_max_deg": math.degrees(float(ang.max())),
            "actual_rate_abs_max_deg_s": math.degrees(float(np.abs(rate).max())),
            "rate_pinned_samples": pinned,
            "rate_pinned_duty_frac": pinned / ang.size,
            "effort_clamp_samples": int(np.sum(ecl > CLAMP_FLAG_TRUE)),
            "target_clamp_samples": int(np.sum(tcl > CLAMP_FLAG_TRUE)),
            "hits_mechanical_limit": bool(np.max(np.abs(ang)) >= MAX_ANGLE_RAD),
            "freq_fft_hz": f_fft,
            "freq_fft_peak_variance_fraction": varfrac,
            "nyquist_hz": nyq,
            "freq_zero_crossing_hz": f_zc,
            "zero_crossing_period_sd_s": zsd,
            "n_zero_crossings": nzc,
        }
        # spectral content ABOVE the old 20 Hz diagnostics Nyquist (10 Hz) -
        # the specific thing a 20 Hz record could not see
        if nyq and nyq > 12.0:
            d["spectral_fraction_0_10hz"] = band_energy(at, ang, 0.0, 10.0)
            d["spectral_fraction_10_50hz"] = band_energy(at, ang, 10.0, 50.0)
            d["spectral_fraction_above_50hz"] = band_energy(at, ang, 50.0, nyq)
        # f vs the fully-rate-saturated triangle prediction R/(2P)
        P = p2p(ang)
        if P > 0 and f_fft:
            d["f_pred_triangle_R_over_2P_hz"] = MAX_RATE_RAD_S / (2 * P)
            d["f_measured_over_f_pred"] = f_fft / (MAX_RATE_RAD_S / (2 * P))
        # restoring torque the control law can produce at the observed peak
        d["kp_restoring_torque_at_peak_nm"] = KP * float(np.abs(ang).max())
        d["kp_restoring_torque_as_frac_of_max_effort"] = (
            KP * float(np.abs(ang).max()) / MAX_EFFORT_NM)
        per[s] = d
    out["per_surface_airborne"] = per

    # ---- Q2: actual oscillates while the command is calm -------------------
    nwin = int((t1 - t0) // CALM_WINDOW_S)
    calm = {s: {"windows": 0, "qualifying": 0, "worst_actual_p2p_deg": 0.0,
                "worst_cmd_p2p_deg": None} for s in SURFACES}
    for k in range(nwin):
        a, b = t0 + k * CALM_WINDOW_S, t0 + (k + 1) * CALM_WINDOW_S
        m = (at >= a) & (at < b)
        if m.sum() < 4:
            continue
        for s in SURFACES:
            c = math.degrees(p2p(surf(av[m], s, "cmd_rad")))
            g = math.degrees(p2p(surf(av[m], s, "actual_angle_rad")))
            calm[s]["windows"] += 1
            if c <= CMD_CALM_P2P_DEG and g > ACT_OSC_P2P_DEG:
                calm[s]["qualifying"] += 1
                if g > calm[s]["worst_actual_p2p_deg"]:
                    calm[s]["worst_actual_p2p_deg"] = g
                    calm[s]["worst_cmd_p2p_deg"] = c
    out["calm_command_oscillating_joint"] = calm

    # ---- airframe rates ----------------------------------------------------
    body = {}
    for i, nm in enumerate(("p_roll", "q_pitch", "r_yaw")):
        f, vf, nyq = dom_freq(it, ig[:, i])
        body[nm] = {"rms_deg_s": math.degrees(rms(ig[:, i])),
                    "p2p_deg_s": math.degrees(p2p(ig[:, i])),
                    "freq_fft_hz": f, "peak_variance_fraction": vf,
                    "nyquist_hz": nyq}
    dt_i = float(np.median(np.diff(it)))
    alpha_y = np.gradient(ig[:, 1], dt_i)
    body["pitch_angular_accel_rms_rad_s2"] = rms(alpha_y)
    body["imu_sample_rate_hz"] = 1.0 / dt_i
    out["airframe_rates_airborne"] = body

    # ---- aero coefficients -------------------------------------------------
    aero = {}
    for nm in ("Cl", "Cm", "Cn"):
        y = ev[:, AERO_FIELDS.index(nm)]
        f, vf, nyq = dom_freq(et, y)
        aero[nm] = {"p2p": p2p(y), "rms": rms(y - y.mean()),
                    "freq_fft_hz": f, "peak_variance_fraction": vf}
    aero["V_mean_ms"] = float(np.mean(ev[:, AERO_FIELDS.index("V")]))
    aero["qbar_mean_pa"] = float(np.mean(ev[:, AERO_FIELDS.index("qbar")]))
    out["aero_airborne"] = aero

    # ---- correlation: joint motion vs body pitch angular acceleration ------
    # both resampled onto the coarser (IMU) grid
    corr = {}
    for s in SURFACES:
        ang_i = np.interp(it, at, surf(av, s, "actual_angle_rad"))
        rat_i = np.interp(it, at, surf(av, s, "actual_rate_rad_s"))
        for nm, y in (("angle_vs_alpha_y", ang_i), ("rate_vs_alpha_y", rat_i)):
            if np.std(y) > 0 and np.std(alpha_y) > 0:
                corr.setdefault(s, {})[nm] = float(
                    np.corrcoef(y, alpha_y)[0, 1])
            else:
                corr.setdefault(s, {})[nm] = None
        corr[s]["angle_vs_q_pitch"] = (
            float(np.corrcoef(ang_i, ig[:, 1])[0, 1])
            if np.std(ang_i) > 0 else None)
    out["correlation_with_airframe"] = corr

    # ---- left/right phase, from the raw traces -----------------------------
    la = surf(av, "left_aileron", "actual_angle_rad")
    ra = surf(av, "right_aileron", "actual_angle_rad")
    le = surf(av, "left_elevator", "actual_angle_rad")
    re_ = surf(av, "right_elevator", "actual_angle_rad")
    out["left_right_agreement"] = {
        "aileron_max_abs_diff_deg": math.degrees(float(np.max(np.abs(la - ra)))),
        "elevator_max_abs_diff_deg": math.degrees(float(np.max(np.abs(le - re_)))),
        "delta_a_aero_p2p_deg": math.degrees(p2p(0.5 * (ra - la))),
        "delta_e_aero_p2p_deg": math.degrees(p2p(-0.5 * (le + re_))),
        "note": "delta_a = 0.5*(theta_R - theta_L), delta_e = "
                "-0.5*(theta_LE + theta_RE), per "
                "docs/source_of_truth/controls/CONTROLS.md sec 10.",
    }

    # ---- per-1 s-window f vs the R/(2P) triangle prediction ----------------
    # The 2026-09-10 analysis evaluated this PER BURST. This stage has no
    # burst detector, so it evaluates it on fixed 1 s windows (same
    # CALM_WINDOW_S) to give a comparable window-scoped population instead of
    # one whole-segment number.
    tri = {}
    for s_ in SURFACES:
        ang_all = surf(av, s_, "actual_angle_rad")
        rate_all = surf(av, s_, "actual_rate_rad_s")
        ratios, duties = [], []
        for k in range(nwin):
            a, b = t0 + k * CALM_WINDOW_S, t0 + (k + 1) * CALM_WINDOW_S
            m = (at >= a) & (at < b)
            if m.sum() < 16:
                continue
            duty = float(np.mean(np.abs(rate_all[m]) >= pin_thr))
            if duty < 0.20:
                continue
            P = p2p(ang_all[m])
            f, _, _ = dom_freq(at[m], ang_all[m])
            if not f or P <= 0:
                continue
            ratios.append(f / (MAX_RATE_RAD_S / (2 * P)))
            duties.append(duty)
        tri[s_] = {
            "n_windows_with_duty_ge_20pct": len(ratios),
            "f_over_f_pred_min": (min(ratios) if ratios else None),
            "f_over_f_pred_median": (float(np.median(ratios)) if ratios else None),
            "f_over_f_pred_max": (max(ratios) if ratios else None),
            "mean_duty": (float(np.mean(duties)) if duties else None),
        }
    out["triangle_prediction_per_1s_window"] = tri

    # ---- DECIMATION TEST ---------------------------------------------------
    # The single question this stage was asked to settle: is the 20 Hz
    # rate-pinned duty a materially biased LOWER BOUND of the true duty?
    # Answered WITHIN THE SAME RECORD by decimating this run's own samples,
    # so no run-to-run difference can confound it. Only meaningful when the
    # record is actually high-rate.
    src_hz = out["measured_publish_rate_hz"]["actuator_diagnostics"]
    dec = {"source_rate_hz": src_hz}
    if src_hz > 100:
        for target in (20.0, 50.0, 100.0, 200.0):
            step = max(1, int(round(src_hz / target)))
            sub = av[::step, :]
            dec[f"decimated_to_{target:.0f}hz"] = {
                "effective_rate_hz": src_hz / step,
                "n_samples": int(sub.shape[0]),
                "rate_pinned_duty_frac": {
                    s_: float(np.mean(np.abs(surf(sub, s_, "actual_rate_rad_s"))
                                       >= pin_thr)) for s_ in SURFACES},
                "effort_clamp_samples": {
                    s_: int(np.sum(surf(sub, s_, "effort_clamp_active")
                                    > CLAMP_FLAG_TRUE)) for s_ in SURFACES},
                "actual_p2p_deg": {
                    s_: math.degrees(p2p(surf(sub, s_, "actual_angle_rad")))
                    for s_ in SURFACES},
            }
        dec["full_rate"] = {
            "effective_rate_hz": src_hz,
            "n_samples": int(av.shape[0]),
            "rate_pinned_duty_frac": {
                s_: per[s_]["rate_pinned_duty_frac"] for s_ in SURFACES},
        }
    else:
        dec["note"] = ("Record is not high-rate; decimation test not "
                       "applicable to it.")
    out["decimation_test"] = dec

    # ---- PRE-LIFTOFF (ground-roll) census and onset ordering ---------------
    # Uses the SAME 1 kHz record, on the segment BEFORE the airborne window.
    # Thresholds reused from the existing harness's own TH block
    # (test_manual_takeoff_ground_roll_reproduction.py): surface_onset_deg
    # = 0.5 deg, yaw/attitude rate onset = 1.0 deg/s. Nothing new invented.
    SURFACE_ONSET_DEG = 0.5
    RATE_ONSET_DEG_S = 1.0
    gm = H["act_t"] < t0
    gt = H["act_t"][gm]
    gv = H["act_v"][gm, :]
    ground = {"n_samples_before_airborne": int(gt.size),
              "window_sim_s": [float(gt[0]), float(gt[-1])] if gt.size else None}
    for s_ in SURFACES:
        if gt.size == 0:
            ground[s_] = "DATA_REQUIRED"
            continue
        ang = surf(gv, s_, "actual_angle_rad")
        rate = surf(gv, s_, "actual_rate_rad_s")
        pinned = np.abs(rate) >= pin_thr
        i_dev = np.where(np.abs(np.degrees(ang)) > SURFACE_ONSET_DEG)[0]
        i_pin = np.where(pinned)[0]
        ground[s_] = {
            "actual_p2p_deg": math.degrees(p2p(ang)),
            "rate_pinned_samples": int(pinned.sum()),
            "rate_pinned_duty_frac": float(pinned.mean()),
            "first_deflection_over_0p5deg_t_sim": (float(gt[i_dev[0]])
                                                    if i_dev.size else None),
            "first_rate_pin_t_sim": (float(gt[i_pin[0]]) if i_pin.size else None),
        }
    ig_all = H["imu_gyro"]
    it_all = H["imu_t"]
    qm = np.abs(np.degrees(ig_all[:, 1])) > RATE_ONSET_DEG_S
    pre = it_all < t0
    iq = np.where(qm & pre)[0]
    ground["first_body_pitch_rate_over_1deg_s_t_sim"] = (
        float(it_all[iq[0]]) if iq.size else None)
    ground["airborne_window_start_t_sim"] = t0
    out["pre_liftoff_ground_census"] = ground

    # ---- ArduPilot servo PWM over the airborne segment ----------------------
    pw = [s["servo_pwm"] for s in S
          if s.get("servo_pwm") and t0 <= s["t_sim"] <= t1]
    if pw:
        A = np.asarray(pw, float)
        out["ardupilot_servo_pwm_airborne"] = {
            "n": int(A.shape[0]),
            "aileron_c1": {"min": float(A[:, 0].min()), "max": float(A[:, 0].max()),
                            "unique": int(np.unique(A[:, 0]).size)},
            "elevator_c2": {"min": float(A[:, 1].min()), "max": float(A[:, 1].max()),
                             "unique": int(np.unique(A[:, 1]).size)},
            "rudder_c4": {"min": float(A[:, 3].min()), "max": float(A[:, 3].max()),
                           "unique": int(np.unique(A[:, 3]).size)},
        }
    else:
        out["ardupilot_servo_pwm_airborne"] = "DATA_REQUIRED"
    return out


def main():
    tags = sys.argv[1:] or ["baseline_MANUAL", "baseline_FBWA",
                            "highrate_MANUAL", "highrate_FBWA"]
    raw = os.environ.get(
        "RAW_DIR",
        "/tmp/claude-1000/-home-emirhan-Desktop-FalconV2/"
        "76fa60fb-30e2-4feb-bf64-2f73f0e53cde/scratchpad/highrate_raw")
    runs = []
    for tag in tags:
        ts = os.path.join(RES, f"airborne_limit_cycle_runtime_{tag}_timeseries.json")
        hr = os.path.join(raw, f"airborne_limit_cycle_{tag}_highrate.json")
        if not (os.path.exists(ts) and os.path.exists(hr)):
            runs.append({"tag": tag, "status": "MISSING_RECORD",
                         "timeseries_exists": os.path.exists(ts),
                         "highrate_exists": os.path.exists(hr)})
            continue
        runs.append(analyse_one(tag, ts, hr))

    doc = {
        "stage": "AIRBORNE_ACTUATOR_LIMIT_CYCLE_ROOT_CAUSE - RUNTIME VERIFICATION",
        "owner": "gazebo-testing",
        "date": "2026-09-11",
        "classification": "MEASUREMENT-ONLY. No physics parameter was read for "
                          "modification or modified. No pre-existing captured "
                          "record was altered.",
        "cross_reference": "docs/test_results/"
                           "2026-09-10_airborne_actuator_limit_cycle_root_cause.md",
        "constants_read": {
            "max_rate_rad_s": MAX_RATE_RAD_S,
            "max_rate_deg_s": math.degrees(MAX_RATE_RAD_S),
            "max_effort_nm": MAX_EFFORT_NM,
            "max_angle_rad": MAX_ANGLE_RAD,
            "kp_nm_per_rad": KP,
            "source": "docs/source_of_truth/controls/actuator_v1_config.yaml",
        },
        "thresholds_imported_from_2026_09_10_analysis": {
            "RATE_PIN_FRACTION": RATE_PIN_FRACTION,
            "CLAMP_FLAG_TRUE": CLAMP_FLAG_TRUE,
            "CALM_WINDOW_S": CALM_WINDOW_S,
            "CMD_CALM_P2P_DEG": CMD_CALM_P2P_DEG,
            "ACT_OSC_P2P_DEG": ACT_OSC_P2P_DEG,
        },
        "threshold_defined_by_this_stage": {
            "AIRBORNE_Z_M": AIRBORNE_Z_M,
            "tag": "ASSUMPTION",
            "basis": "10x the existing harness's own liftoff_z_margin_m = "
                     "0.10 m, so it cannot be satisfied by belly contact. "
                     "Sensitivity reported at 0.5 / 1.0 / 5.0 m.",
        },
        "runs": runs,
    }
    outp = os.path.join(RES, "airborne_limit_cycle_runtime_verification.json")
    with open(outp, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, indent=1)
    print("wrote", outp)
    for r in runs:
        print("\n===", r["tag"], r.get("status"))
        if r.get("status") != "AIRBORNE_SEGMENT_FOUND":
            continue
        print(" rates Hz:", {k: (round(v, 2) if v else v)
                             for k, v in r["measured_publish_rate_hz"].items()})
        print(" airborne %.1f s (%.1f..%.1f), n_act=%d, max alt %.1f m" % (
            r["airborne_duration_s"], r["airborne_window_sim_s"][0],
            r["airborne_window_sim_s"][1], r["n_airborne_actuator_samples"],
            r["max_altitude_m"]))
        for s in SURFACES:
            d = r["per_surface_airborne"][s]
            print("  %-15s cmd_p2p=%7.3f deg  act_p2p=%7.3f deg  "
                  "[%7.2f..%7.2f]  pin=%6.2f%%  eff_clamp=%d  f_fft=%s  f_zc=%s"
                  % (s, d["cmd_p2p_deg"], d["actual_p2p_deg"],
                     d["actual_min_deg"], d["actual_max_deg"],
                     100 * d["rate_pinned_duty_frac"], d["effort_clamp_samples"],
                     ("%.3f" % d["freq_fft_hz"]) if d["freq_fft_hz"] else "-",
                     ("%.3f" % d["freq_zero_crossing_hz"]) if d["freq_zero_crossing_hz"] else "-"))
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
FALCON V2 - ACTUATOR_CONTROL_LIMIT_CYCLE_FIX_AND_VALIDATION step 5 analysis.
Owner: gazebo-testing, 2026-09-12.

CLASSIFICATION: MEASUREMENT-ONLY / READ-ONLY. Launches no simulation, writes
no parameter, writes exactly one result JSON plus a log.

Re-measures, on THIS stage's own post-fix flights, exactly the quantities the
2026-09-11 pre-fix runtime verification measured, so the two are directly
comparable, and judges them against the falsifiable predictions P4/P5/P6/P11/
P13/P14 of docs/test_results/2026-09-12_actuator_limit_cycle_fix_
implementation.md.

THRESHOLDS are imported verbatim from
tests/gazebo/scripts/analyze_airborne_limit_cycle_runtime.py, which in turn
imported them verbatim from the 2026-09-10 offline analysis. None was chosen
here and none was changed after a result was seen.

CORRECTIONS CARRIED OUT HERE, from docs/validation/2026-09-11_airborne_
actuator_limit_cycle_root_cause_validation.md (items routed to gazebo-testing)
---------------------------------------------------------------------------
m5  There is NO solver overshoot of the velocity limit. 5.2360 rad/s is
    300.00070153 deg/s EXACTLY; the configured limit IS 300.0007 deg/s. This
    script reports `max_abs_rate_minus_limit_deg_s` (which is <= 0 by
    construction when the limit holds) and never describes a rounding
    artifact as a solver tolerance.
m7  dom_freq() previously returned raw FFT bin centres. Here the peak is
    refined by 3-point parabolic (Quinn/Jacobsen-style log-magnitude)
    interpolation AND the raw bin spacing df is reported alongside every
    frequency, so no frequency is quoted more precisely than it is known.
m13 The observability-only claim for the 1 kHz model covers BOTH changed
    plugins; see that model's header and this stage's report.
m14 np.interp evaluated at its own nodes returns 0 by construction. That
    number is NOT reported here as evidence of anything. The genuinely
    uncontrolled term - per-topic transport latency between /clock and the
    actuator topic - is declared UNMEASURED.
m19 Every number quoted in the report is read back out of this JSON.
M4  The 20 Hz duty estimator is PROVABLY UNBIASED (validation's 50-phase
    sweep, +0.00 pp). Duty figures are NOT described as lower bounds
    anywhere. Amplitude from a 20 Hz record IS a lower bound - but every run
    analysed here is a 1 kHz run, so that caveat does not apply to these
    numbers either.
I15 Actuator/aero sample times are RECONSTRUCTED (gz.msgs.Double_V carries no
    header); only the IMU has a true publisher stamp. Declared in the output.
T1  The published aero `Cm` is in XFLR5's UNFLIPPED convention and is NOT the
    driver of My (AeroModel.hh applies -cmStatic + cmRate). This script
    therefore reports Cm as a COEFFICIENT only and never forms My from it.
    Cl and Cn ARE the applied convention (mx = qbar*S*b*Cl, mz = qbar*S*b*Cn,
    AeroModel.hh lines 704/706), so rolling and yawing moments ARE formed.

APPLIED EFFORT: still not a published channel (DATA_REQUIRED as a first-class
field). It is reconstructed here exactly as in the standalone bench - see
test_actuator_standalone_step_response.py's header, method E1 - by running
the committed control law forward over the 1 kHz diagnostics stream from the
documented initial integral 0.0. The reconstruction predicts
`effort_clamp_active`, which IS published, so it is falsifiable whenever that
flag is ever true; when the flag is constant-false the agreement is
consistency only and is labelled as such.
"""
import json
import math
import os
import sys

import numpy as np
import yaml

REPO = "/home/emirhan/Desktop/FalconV2"
RES = os.path.join(REPO, "tests/gazebo/results")
sys.path.insert(0, os.path.join(REPO, "tests/gazebo/scripts"))

import analyze_airborne_limit_cycle_runtime as PRE  # thresholds + loader

SURFACES = PRE.SURFACES
ACT_FIELDS = PRE.ACT_FIELDS
AERO_FIELDS = PRE.AERO_FIELDS

# thresholds, imported verbatim
RATE_PIN_FRACTION = PRE.RATE_PIN_FRACTION
CLAMP_FLAG_TRUE = PRE.CLAMP_FLAG_TRUE
CALM_WINDOW_S = PRE.CALM_WINDOW_S
CMD_CALM_P2P_DEG = PRE.CMD_CALM_P2P_DEG
ACT_OSC_P2P_DEG = PRE.ACT_OSC_P2P_DEG
AIRBORNE_Z_M = PRE.AIRBORNE_Z_M
AIRBORNE_Z_SENS = PRE.AIRBORNE_Z_SENS

_ACFG = yaml.safe_load(open(os.path.join(
    REPO, "docs/source_of_truth/controls/actuator_v1_config.yaml")))
_AEROCFG = yaml.safe_load(open(os.path.join(
    REPO, "docs/source_of_truth/aerodynamics/aero_v1_config.yaml")))
S_REF = float(_AEROCFG["reference_geometry"]["wing_area_S_m2"])
B_REF = float(_AEROCFG["reference_geometry"]["wingspan_b_m"])

MAX_RATE = float(_ACFG["surfaces"]["left_aileron"]["max_rate_rad_s"])
MAX_RATE_DEG_S = math.degrees(MAX_RATE)


def surf_cfg(name):
    law = _ACFG["control_law"]
    n = _ACFG["surfaces"][name]
    return dict(kp=n.get("kp_nm_per_rad", law["kp_nm_per_rad"]),
                kd=n.get("kd_nm_per_rad_s", law["kd_nm_per_rad_s"]),
                ki=n.get("ki_nm_per_rad_s", law.get("ki_nm_per_rad_s", 0.0)),
                sp_weight_b=law.get("sp_weight_b", 1.0),
                max_effort_nm=n["max_effort_nm"],
                max_rate_rad_s=n["max_rate_rad_s"],
                min_angle_rad=n["min_angle_rad"], max_angle_rad=n["max_angle_rad"],
                hinge_inertia_kg_m2=n.get("hinge_inertia_kg_m2"))


RATE_SAT_DETECT_FRACTION = 0.999  # kRateSaturationDetectFraction, ActuatorModel.hh


def pid_effort(sp, ang, rate, prev_i, dt, tgt_clamp, c):
    error = sp - ang
    errp = c["sp_weight_b"] * sp - ang
    raw0 = c["kp"] * errp + c["ki"] * prev_i - c["kd"] * rate
    ws = c["max_effort_nm"] > 0 and abs(raw0) >= c["max_effort_nm"]
    ss = (raw0 >= 0 and error >= 0) or (raw0 <= 0 and error <= 0)
    rs = c["max_rate_rad_s"] > 0 and abs(rate) >= RATE_SAT_DETECT_FRACTION * c["max_rate_rad_s"]
    rss = (rate >= 0 and error >= 0) or (rate <= 0 and error <= 0)
    freeze = bool(tgt_clamp) or (ws and ss) or (rs and rss)
    integ = prev_i + error * dt if (not freeze and dt > 0) else prev_i
    raw = c["kp"] * errp + c["ki"] * integ - c["kd"] * rate
    cl = max(-c["max_effort_nm"], min(c["max_effort_nm"], raw))
    return cl, (cl != raw), integ, freeze, (rs and rss)


def dom_freq_interp(t, y):
    """Dominant non-DC frequency with 3-point parabolic peak interpolation on
    the log-magnitude spectrum, PLUS the raw bin spacing so the honest
    resolution is always visible (validation finding m7)."""
    y = np.asarray(y, float)
    n = y.size
    if n < 32:
        return None
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        return None
    yy = (y - y.mean()) * np.hanning(n)
    Y = np.abs(np.fft.rfft(yy))
    f = np.fft.rfftfreq(n, dt)
    Y[0] = 0.0
    k = int(np.argmax(Y))
    df = float(f[1] - f[0])
    f_bin = float(f[k])
    f_interp = f_bin
    if 0 < k < Y.size - 1 and Y[k - 1] > 0 and Y[k + 1] > 0:
        a, b, c = np.log(Y[k - 1]), np.log(Y[k]), np.log(Y[k + 1])
        denom = (a - 2 * b + c)
        if denom != 0:
            delta = 0.5 * (a - c) / denom
            if abs(delta) <= 1.0:
                f_interp = f_bin + delta * df
    tot = float(np.sum(Y ** 2))
    return dict(f_hz_parabolic=f_interp, f_hz_raw_bin=f_bin,
                fft_bin_spacing_hz=df, n_samples=int(n),
                record_length_s=float(n * dt),
                peak_bin_variance_fraction=(float(Y[k] ** 2 / tot) if tot > 0 else None),
                nyquist_hz=1.0 / (2 * dt))


def p2p(x):
    x = np.asarray(x, float)
    return float(x.max() - x.min()) if x.size else float("nan")


def rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x * x))) if x.size else float("nan")


def sl(t, v, t0, t1):
    m = (t >= t0) & (t <= t1)
    return v[m], m


def per_surface(H, t0, t1, out_key="per_surface"):
    """Per-surface census over the sim-time window [t0, t1]."""
    act_t = H["act_t"]
    act_v = H["act_v"]
    m = (act_t >= t0) & (act_t <= t1)
    tt = act_t[m]
    res = {}
    for i, name in enumerate(SURFACES):
        c = surf_cfg(name)
        col = lambda f: act_v[m, i * 7 + ACT_FIELDS.index(f)]  # noqa: E731
        cmd = col("cmd_rad")
        sp = col("setpoint_rad")
        ang = col("actual_angle_rad")
        rate = col("actual_rate_rad_s")
        tclamp = col("target_clamp_active")
        eclamp = col("effort_clamp_active")
        track_err = ang - sp

        # ---- E1 effort reconstruction over the FULL record (not the window)
        # so the integral state entering the window is the real one.
        allv = act_v
        colf = lambda f: allv[:, i * 7 + ACT_FIELDS.index(f)]  # noqa: E731
        a_sp, a_ang, a_rt, a_tc = colf("setpoint_rad"), colf("actual_angle_rad"), \
            colf("actual_rate_rad_s"), colf("target_clamp_active")
        dt_all = np.diff(act_t, prepend=act_t[0] - 0.001)
        integ = 0.0
        tau = np.empty(allv.shape[0])
        eclam_pred = np.empty(allv.shape[0], dtype=bool)
        for k in range(allv.shape[0]):
            dt = float(dt_all[k])
            if not np.isfinite(dt) or dt <= 0 or dt > 0.05:
                dt = 0.001
            tk, ec, integ, _, _ = pid_effort(a_sp[k], a_ang[k], a_rt[k], integ,
                                             dt, a_tc[k] > CLAMP_FLAG_TRUE, c)
            tau[k] = tk
            eclam_pred[k] = ec
        tau_w = tau[m]
        agree = float(np.mean(eclam_pred == (allv[:, i * 7 + ACT_FIELDS.index("effort_clamp_active")] > CLAMP_FLAG_TRUE)))

        pinned = np.abs(rate) >= RATE_PIN_FRACTION * MAX_RATE
        res[name] = {
            "n_samples": int(tt.size),
            "cmd_unique_values": int(np.unique(np.round(cmd, 12)).size),
            "cmd_p2p_deg": math.degrees(p2p(cmd)),
            "setpoint_p2p_deg": math.degrees(p2p(sp)),
            "actual_p2p_deg": math.degrees(p2p(ang)),
            "actual_min_deg": math.degrees(float(ang.min())),
            "actual_max_deg": math.degrees(float(ang.max())),
            "tracking_error_p2p_deg": math.degrees(p2p(track_err)),
            "tracking_error_rms_deg": math.degrees(rms(track_err)),
            "max_abs_rate_deg_s": math.degrees(float(np.abs(rate).max())),
            "configured_rate_limit_deg_s": MAX_RATE_DEG_S,
            "max_abs_rate_minus_limit_deg_s": math.degrees(float(np.abs(rate).max())) - MAX_RATE_DEG_S,
            "rate_pinned_duty_frac": float(np.mean(pinned)),
            "target_clamp_duty_frac": float(np.mean(tclamp > CLAMP_FLAG_TRUE)),
            "effort_clamp_duty_frac": float(np.mean(eclamp > CLAMP_FLAG_TRUE)),
            "hits_mechanical_limit": bool(np.abs(ang).max() >= c["max_angle_rad"]),
            "effort_recon_E1": {
                "max_abs_nm": float(np.abs(tau_w).max()),
                "mean_abs_nm": float(np.abs(tau_w).mean()),
                "rms_nm": float(rms(tau_w)),
                "frac_of_max_effort": float(np.abs(tau_w).max() / c["max_effort_nm"]),
                "predicted_vs_published_effort_clamp_agreement_frac": agree,
                "n_published_effort_clamp_true_whole_record": int(
                    np.sum(allv[:, i * 7 + ACT_FIELDS.index("effort_clamp_active")] > CLAMP_FLAG_TRUE)),
                "falsifiable": bool(np.sum(allv[:, i * 7 + ACT_FIELDS.index("effort_clamp_active")] > CLAMP_FLAG_TRUE) > 0),
            },
            "angle_spectrum": dom_freq_interp(tt, ang),
            "tracking_error_spectrum": dom_freq_interp(tt, track_err),
        }
    return res


def body_and_aero(H, t0, t1):
    it = H["imu_t"]
    m = (it >= t0) & (it <= t1)
    g = H["imu_gyro"][m]
    a = H["imu_acc"][m]
    at = H["aero_t"]
    ma = (at >= t0) & (at <= t1)
    av = H["aero_v"][ma]
    idx = {f: AERO_FIELDS.index(f) for f in AERO_FIELDS}
    qbar = av[:, idx["qbar"]]
    out = {
        "imu_n": int(m.sum()),
        "p_rms_deg_s": math.degrees(rms(g[:, 0])) if m.sum() else None,
        "q_rms_deg_s": math.degrees(rms(g[:, 1])) if m.sum() else None,
        "r_rms_deg_s": math.degrees(rms(g[:, 2])) if m.sum() else None,
        "p_p2p_deg_s": math.degrees(p2p(g[:, 0])) if m.sum() else None,
        "q_p2p_deg_s": math.degrees(p2p(g[:, 1])) if m.sum() else None,
        "r_p2p_deg_s": math.degrees(p2p(g[:, 2])) if m.sum() else None,
        "f_y_rms_ms2": rms(a[:, 1]) if m.sum() else None,
        "f_y_absmax_ms2": float(np.abs(a[:, 1]).max()) if m.sum() else None,
        "f_z_rms_ms2": rms(a[:, 2]) if m.sum() else None,
        "f_z_absmax_ms2": float(np.abs(a[:, 2]).max()) if m.sum() else None,
        "q_rate_spectrum": dom_freq_interp(it[m], g[:, 1]) if m.sum() > 64 else None,
        "r_rate_spectrum": dom_freq_interp(it[m], g[:, 2]) if m.sum() > 64 else None,
        "aero_n": int(ma.sum()),
        "V_min_ms": float(av[:, idx["V"]].min()) if ma.sum() else None,
        "V_max_ms": float(av[:, idx["V"]].max()) if ma.sum() else None,
        "V_mean_ms": float(av[:, idx["V"]].mean()) if ma.sum() else None,
        "alpha_p2p_deg": math.degrees(p2p(av[:, idx["alpha"]])) if ma.sum() else None,
        "beta_p2p_deg": math.degrees(p2p(av[:, idx["beta"]])) if ma.sum() else None,
        "beta_absmax_deg": math.degrees(float(np.abs(av[:, idx["beta"]]).max())) if ma.sum() else None,
        "Cm_coefficient_p2p": p2p(av[:, idx["Cm"]]) if ma.sum() else None,
        "Cm_convention_note": "PUBLISHED Cm IS XFLR5-UNFLIPPED AND IS NOT THE My "
                              "DRIVER (AeroModel.hh applies -cmStatic + cmRate). "
                              "Reported as a coefficient only; My is NOT formed "
                              "from it here.",
        "rolling_moment_L_Nm_p2p": p2p(qbar * S_REF * B_REF * av[:, idx["Cl"]]) if ma.sum() else None,
        "rolling_moment_L_Nm_absmax": float(np.abs(qbar * S_REF * B_REF * av[:, idx["Cl"]]).max()) if ma.sum() else None,
        "yawing_moment_N_Nm_p2p": p2p(qbar * S_REF * B_REF * av[:, idx["Cn"]]) if ma.sum() else None,
        "yawing_moment_N_Nm_absmax": float(np.abs(qbar * S_REF * B_REF * av[:, idx["Cn"]]).max()) if ma.sum() else None,
    }
    return out


def nan_scan(H):
    bad = {}
    for k in ("act_v", "aero_v", "imu_gyro", "imu_acc"):
        a = np.asarray(H[k], float)
        bad[k] = dict(n_nan=int(np.sum(np.isnan(a))), n_inf=int(np.sum(np.isinf(a))))
    return bad



LEADIN_S = 1.0  # see calm_windows() docstring, LEAD-IN REQUIREMENT


def calm_windows(H, t0, t1, win_s=CALM_WINDOW_S, stride_s=0.25):
    """Sliding fixed-length windows over [t0, t1]. For each window and each
    surface: the COMMAND peak-to-peak, the TRACKING ERROR (actual - setpoint)
    peak-to-peak, and the lateral/normal specific force in the same window.

    A window is 'command-calm' when cmd p2p <= CMD_CALM_P2P_DEG (1.0 deg) -
    the criterion imported verbatim from the 2026-09-10 offline analysis and
    reused unchanged by the 2026-09-11 runtime pass. Inside such a window any
    joint motion is uncommanded by construction, so the tracking error is the
    right quantity to test for a limit cycle even when the surface IS being
    commanded elsewhere in the record.

    This is what makes the YAW_RESPONSE_TEST answerable: the rudder is
    commanded hard during the ladder, so its raw joint angle is supposed to
    move; only its command-calm tracking error can show a limit cycle.

    LEAD-IN REQUIREMENT (added here, and the reason it had to be added).
    The imported criterion was written for records in which the command NEVER
    changed, so a 1 s look-back was not needed. In the yaw ladder it is: a
    window can start 60 ms after a 45 deg command STEP and still satisfy
    "cmd p2p <= 1 deg" for its own duration, because the command is already
    at its new value. Such a window contains the setpoint-rate-limiter slew
    and the integral unwinding of that step - a deterministic, fully expected
    transient - and scoring it as "uncommanded motion" would be wrong. This
    was observed directly in the raw waveform (rudder, window t0 = 58.37 s:
    setpoint slewing 36.3 -> 0.3 deg at the 300.0007 deg/s limiter while the
    joint lags, peak error 12.93 deg, then a monotone decay with tau ~ 0.5 s).
    `command_calm_with_leadin` additionally requires the command to have been
    calm for the preceding LEADIN_S as well. BOTH flags are reported; the
    stricter one is the one any limit-cycle judgement should use."""
    act_t, act_v = H["act_t"], H["act_v"]
    it, ia = H["imu_t"], H["imu_acc"]
    rows = []
    t = t0
    while t + win_s <= t1:
        m = (act_t >= t) & (act_t < t + win_s)
        mi = (it >= t) & (it < t + win_s)
        if m.sum() > 32:
            row = {"t_start_sim_s": float(t),
                   "f_y_rms_ms2": rms(ia[mi, 1]) if mi.sum() else None,
                   "f_y_absmax_ms2": float(np.abs(ia[mi, 1]).max()) if mi.sum() else None,
                   "f_z_rms_ms2": rms(ia[mi, 2]) if mi.sum() else None,
                   "surfaces": {}}
            for i, name in enumerate(SURFACES):
                mL = (act_t >= t - LEADIN_S) & (act_t < t + win_s)
                cmdL = act_v[mL, i * 7 + ACT_FIELDS.index("cmd_rad")]
                cmd = act_v[m, i * 7 + ACT_FIELDS.index("cmd_rad")]
                sp = act_v[m, i * 7 + ACT_FIELDS.index("setpoint_rad")]
                ang = act_v[m, i * 7 + ACT_FIELDS.index("actual_angle_rad")]
                rate = act_v[m, i * 7 + ACT_FIELDS.index("actual_rate_rad_s")]
                row["surfaces"][name] = {
                    "cmd_p2p_deg": math.degrees(p2p(cmd)),
                    "actual_p2p_deg": math.degrees(p2p(ang)),
                    "track_err_p2p_deg": math.degrees(p2p(ang - sp)),
                    "max_abs_rate_deg_s": math.degrees(float(np.abs(rate).max())),
                    "rate_pinned_duty_frac": float(np.mean(
                        np.abs(rate) >= RATE_PIN_FRACTION * MAX_RATE)),
                    "command_calm": bool(math.degrees(p2p(cmd)) <= CMD_CALM_P2P_DEG),
                    "command_calm_with_leadin": bool(
                        cmdL.size > 0 and
                        math.degrees(p2p(cmdL)) <= CMD_CALM_P2P_DEG),
                    "cmd_p2p_with_leadin_deg": (math.degrees(p2p(cmdL))
                                                 if cmdL.size else None),
                }
            rows.append(row)
        t += stride_s
    summ = {}
    for name in SURFACES:
        calm_raw = [r for r in rows if r["surfaces"][name]["command_calm"]]
        calm = [r for r in rows if r["surfaces"][name]["command_calm_with_leadin"]]
        summ[name] = {
            "n_windows": len(rows),
            "n_command_calm_windows_raw_imported_criterion": len(calm_raw),
            "worst_raw_calm_track_err_p2p_deg": (
                max(r["surfaces"][name]["track_err_p2p_deg"] for r in calm_raw)
                if calm_raw else None),
            "n_command_calm_windows": len(calm),
            "worst_calm_track_err_p2p_deg": (
                max(r["surfaces"][name]["track_err_p2p_deg"] for r in calm) if calm else None),
            "worst_calm_actual_p2p_deg": (
                max(r["surfaces"][name]["actual_p2p_deg"] for r in calm) if calm else None),
            "n_calm_windows_over_ACT_OSC_P2P_DEG": sum(
                1 for r in calm if r["surfaces"][name]["track_err_p2p_deg"] > ACT_OSC_P2P_DEG),
            "max_f_y_rms_among_calm_windows_ms2": (
                max(r["f_y_rms_ms2"] for r in calm if r["f_y_rms_ms2"] is not None)
                if calm else None),
            "calm_window_track_err_vs_f_y": [
                [r["f_y_rms_ms2"], r["surfaces"][name]["track_err_p2p_deg"]]
                for r in calm if r["f_y_rms_ms2"] is not None],
        }
    return {"window_s": win_s, "stride_s": stride_s,
            "criterion": f"command-calm = cmd p2p <= {CMD_CALM_P2P_DEG} deg "
                         f"(imported verbatim); oscillation bar "
                         f"{ACT_OSC_P2P_DEG} deg (imported verbatim)",
            "summary": summ, "windows": rows}


def envelope_vs_airspeed(H, t0, t1, n_bins=8):
    """Bins the airborne record by TRUE AIRSPEED (the aerodynamics plugin's
    own V channel, 1 kHz) and reports, per bin, the actuator quantities that
    would grow if the elevator servoelastic loop were approaching its
    predicted V_crit. This addresses the residual risk stated in section 'P1/
    P3b' of the fix implementation record: max|L| grows as V^2, so if any
    amplitude growth is present it must appear in the top bins."""
    at, av = H["aero_t"], H["aero_v"]
    ma = (at >= t0) & (at <= t1)
    V = av[ma, AERO_FIELDS.index("V")]
    tV = at[ma]
    act_t, act_v = H["act_t"], H["act_v"]
    if V.size < 64:
        return None
    lo, hi = float(V.min()), float(V.max())
    edges = np.linspace(lo, hi, n_bins + 1)
    # map each actuator sample to an airspeed by nearest aero sample in time
    Vi = np.interp(act_t, tV, V)
    sel = (act_t >= t0) & (act_t <= t1)
    out = {"V_min_ms": lo, "V_max_ms": hi, "bins": []}
    for k in range(n_bins):
        m = sel & (Vi >= edges[k]) & (Vi < edges[k + 1] if k < n_bins - 1 else Vi <= edges[k + 1])
        if m.sum() < 32:
            continue
        b = {"V_lo_ms": float(edges[k]), "V_hi_ms": float(edges[k + 1]),
             "n_samples": int(m.sum()), "surfaces": {}}
        for i, name in enumerate(SURFACES):
            sp = act_v[m, i * 7 + ACT_FIELDS.index("setpoint_rad")]
            ang = act_v[m, i * 7 + ACT_FIELDS.index("actual_angle_rad")]
            rate = act_v[m, i * 7 + ACT_FIELDS.index("actual_rate_rad_s")]
            b["surfaces"][name] = {
                "track_err_p2p_deg": math.degrees(p2p(ang - sp)),
                "track_err_rms_deg": math.degrees(rms(ang - sp)),
                "actual_p2p_deg": math.degrees(p2p(ang)),
                "max_abs_rate_deg_s": math.degrees(float(np.abs(rate).max())),
                "rate_pinned_duty_frac": float(np.mean(
                    np.abs(rate) >= RATE_PIN_FRACTION * MAX_RATE)),
            }
        out["bins"].append(b)
    return out


def analyse_flight(tag, ts_path, hr_path):
    out = {"tag": tag, "timeseries": ts_path, "highrate_raw": hr_path}
    TS = json.load(open(ts_path))
    S = TS["samples"]
    ts_t = np.asarray([s["t_sim"] for s in S], float)
    ts_z = np.asarray([s["gz_pos_world_m"][2] for s in S], float)
    H = PRE.load_highrate(hr_path)
    out["clock_map_used"] = H["clockmap"]["map_used"]
    out["timebase_caveat"] = (
        "I15: gz.msgs.Double_V carries no header, so actuator/aero sample "
        "times are RECONSTRUCTED from wall arrival time through the /clock "
        "map. Only the IMU carries a true publisher stamp. The per-topic "
        "transport latency DIFFERENCE between /clock and the actuator topic "
        "is UNMEASURED and is the real uncontrolled term (m14).")
    out["measured_publish_rate_hz"] = {
        "actuator_diagnostics": float((H["act_t"].size - 1) / (H["act_t"][-1] - H["act_t"][0])),
        "aero_diagnostics": float((H["aero_t"].size - 1) / (H["aero_t"][-1] - H["aero_t"][0])),
        "imu_sensor": float((H["imu_t"].size - 1) / (H["imu_t"][-1] - H["imu_t"][0])),
    }
    dact = np.diff(H["act_t"])
    out["actuator_delivery"] = {
        "n_messages": int(H["act_t"].size),
        "median_gap_s": float(np.median(dact)), "p99_gap_s": float(np.percentile(dact, 99)),
        "max_gap_s": float(np.max(dact)),
        "gaps_over_3x_median": int(np.sum(dact > 3 * np.median(dact))),
    }
    air = {}
    for z in AIRBORNE_Z_SENS:
        k = np.where(ts_z >= z)[0]
        air[str(z)] = (float(ts_t[k[0]]), float(ts_t[-1])) if k.size else None
    out["airborne_window_sim_s_by_z_threshold"] = air
    win = air[str(AIRBORNE_Z_M)]
    if win is None:
        out["status"] = "NO_AIRBORNE_SEGMENT"
        return out
    t0, t1 = win
    out["airborne_window_sim_s"] = [t0, t1]
    out["airborne_duration_s"] = t1 - t0
    out["max_altitude_m"] = float(ts_z.max())
    out["per_surface_airborne"] = per_surface(H, t0, t1)
    out["body_and_aero_airborne"] = body_and_aero(H, t0, t1)
    out["envelope_vs_airspeed_airborne"] = envelope_vs_airspeed(H, t0, t1)
    out["calm_windows_airborne"] = calm_windows(H, t0, t1)
    out["nan_inf_scan_whole_record"] = nan_scan(H)
    out["status"] = "OK"
    return out


def analyse_yaw(tag, yaw_path, hr_path):
    out = {"tag": tag, "yaw_result": yaw_path, "highrate_raw": hr_path}
    Y = json.load(open(yaw_path))
    H = PRE.load_highrate(hr_path)
    w2s = H["w2s"]
    out["clock_map_used"] = H["clockmap"]["map_used"]
    out["overall_from_flight_script"] = Y.get("overall")
    marks = Y["phase_markers_wall"]
    ms = [{"label": m["label"], "t_sim": float(w2s(m["t_wall"])),
           "rc4_us": m.get("rc4_us"), "hold_s": m.get("hold_s")} for m in marks]
    out["phase_markers_sim_s"] = ms
    by = {m["label"]: m for m in ms}

    def window(label):
        for k, m in enumerate(ms):
            if m["label"] == label:
                t0 = m["t_sim"]
                t1 = ms[k + 1]["t_sim"] if k + 1 < len(ms) else t0 + 1.0
                return t0, t1
        return None

    segs = {}
    for m in ms:
        if not m["label"].startswith("ladder_") or m["label"] in ("ladder_start", "ladder_end"):
            continue
        w = window(m["label"])
        if w is None or w[1] - w[0] < 0.2:
            continue
        segs[m["label"]] = dict(
            window_sim_s=list(w), rc4_us=m["rc4_us"],
            per_surface=per_surface(H, w[0], w[1]),
            body_and_aero=body_and_aero(H, w[0], w[1]))
    # in-flight baseline: the neutral-stick stabilise phase
    wb = window("stabilise_start")
    if wb:
        segs["IN_FLIGHT_BASELINE_stabilise"] = dict(
            window_sim_s=list(wb), rc4_us=1500,
            per_surface=per_surface(H, wb[0], wb[1]),
            body_and_aero=body_and_aero(H, wb[0], wb[1]))
    # whole ladder
    if "ladder_start" in by and "ladder_end" in by:
        t0, t1 = by["ladder_start"]["t_sim"], by["ladder_end"]["t_sim"]
        segs["WHOLE_LADDER"] = dict(
            window_sim_s=[t0, t1], rc4_us=None,
            per_surface=per_surface(H, t0, t1),
            body_and_aero=body_and_aero(H, t0, t1))
    out["segments"] = segs
    if "ladder_start" in by and "ladder_end" in by:
        out["calm_windows_ladder"] = calm_windows(
            H, by["ladder_start"]["t_sim"], by["ladder_end"]["t_sim"])
    out["nan_inf_scan_whole_record"] = nan_scan(H)
    out["status"] = "OK"
    return out


def main():
    raw = os.environ.get(
        "RAW_DIR",
        "/tmp/claude-1000/-home-emirhan-Desktop-FalconV2/"
        "d791f088-f001-45d8-802a-39118ca168e8/scratchpad/actuator_fix_highrate_raw")
    out = {"analysis": "ACTUATOR_FIX_AIRBORNE_REGRESSION", "date": "2026-09-12",
           "owner": "gazebo-testing",
           "thresholds_imported_verbatim": {
               "RATE_PIN_FRACTION": RATE_PIN_FRACTION,
               "CLAMP_FLAG_TRUE": CLAMP_FLAG_TRUE,
               "CALM_WINDOW_S": CALM_WINDOW_S,
               "CMD_CALM_P2P_DEG": CMD_CALM_P2P_DEG,
               "ACT_OSC_P2P_DEG": ACT_OSC_P2P_DEG,
               "AIRBORNE_Z_M": AIRBORNE_Z_M},
           "config_per_surface": {s: surf_cfg(s) for s in SURFACES},
           "runs": {}}
    for tag in ("MANUAL", "FBWA"):
        ts = os.path.join(RES, f"actuator_fix_airborne_{tag}_timeseries.json")
        hr = os.path.join(raw, f"actuator_fix_airborne_{tag}_highrate.json")
        if os.path.exists(ts) and os.path.exists(hr):
            out["runs"][tag] = analyse_flight(tag, ts, hr)
            print(f"analysed {tag}", flush=True)
    yw = os.path.join(RES, "actuator_fix_yaw_response_FBWA_result.json")
    hr = os.path.join(raw, "actuator_fix_airborne_YAW_highrate.json")
    if os.path.exists(yw) and os.path.exists(hr):
        out["runs"]["YAW"] = analyse_yaw("YAW", yw, hr)
        print("analysed YAW", flush=True)
    p = os.path.join(RES, "actuator_fix_airborne_regression_analysis.json")
    with open(p, "w") as f:
        json.dump(out, f, indent=1)
    print("wrote", p)
    return 0


if __name__ == "__main__":
    sys.exit(main())

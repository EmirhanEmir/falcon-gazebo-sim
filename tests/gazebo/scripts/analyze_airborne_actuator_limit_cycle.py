#!/usr/bin/env python3
"""FALCON V2 - AIRBORNE_ACTUATOR_LIMIT_CYCLE_ROOT_CAUSE.

CLASSIFICATION: MEASUREMENT-ONLY, READ-ONLY, OFFLINE.
Stage : AIRBORNE_ACTUATOR_LIMIT_CYCLE_ROOT_CAUSE (2026-09-10)
Owner : controls-integration
Input : ALREADY-CAPTURED records only. No simulation is launched, no
        parameter of any kind is written. Every file this script opens is
        opened read-only; the only file it writes is its own result JSON
        under tests/gazebo/results/.

WHY THIS SCRIPT EXISTS
----------------------
The 2026-09-10 manual-takeoff / live-control-surface stage closed with one
deferred defect, routed to controls-integration:

  docs/test_results/2026-09-10_manual_takeoff_and_live_control_surface_
      behavior_validation.md  sec 7b row i3
  docs/validation/2026-09-10_manual_takeoff_and_live_control_surface_
      behavior_validation.md  finding i3

described as an "airborne actuator limit cycle": 0 rate-pinned samples in
every ground roll, 0 of 4890-4905 in every A-F sign scenario, non-zero only
after liftoff in the three TAKEOFF-mode runs. Nothing beyond the pinned-sample
count was ever measured. This script measures it.

WHAT IT MEASURES, AND ON WHICH RECORD
-------------------------------------
Two independent, already-captured views of the SAME runs are used, because
neither alone can answer the frequency question:

  (A) tests/gazebo/results/manual_takeoff_ground_roll_run{1,2,3}_TAKEOFF_
      dataflash/00000001.BIN  - ArduPilot's OWN log.
        IMU  (GyrX/Y/Z)      50.00 Hz  -> Nyquist 25.00 Hz
        RCOU (C1..C5, PWM)   25.00 Hz  -> Nyquist 12.50 Hz
        PIDR (roll rate PID) 25.00 Hz  -> Nyquist 12.50 Hz
        ATT                  25.00 Hz
        SIM2 (sim truth)     50.00 Hz
      Rates are MEASURED per file by this script, not assumed.

  (B) tests/gazebo/results/manual_takeoff_ground_roll_run{1,2,3}_TAKEOFF_
      timeseries.json - the stage harness capture. It is the ONLY record
      that carries the actuator plugin's own per-surface diagnostics
      (cmd_rad / target_clamped_rad / setpoint_rad / actual_angle_rad /
      actual_rate_rad_s / target_clamp_active / effort_clamp_active) and the
      aero plugin's Cl/Cm/Cn, on one timestamp per sample.
      The actuator diagnostics topic publishes at
      docs/source_of_truth/controls/actuator_v1_config.yaml
      command_interface.diagnostics_rate_hz = 20.0 Hz -> Nyquist 10.0 Hz.
      The harness samples faster than that, so consecutive samples REPEAT the
      same diagnostics message. This script de-duplicates to DISTINCT
      diagnostics arrivals before any spectral work, and timestamps each
      channel at t_wall + skew_s[channel] (skew_s is defined by
      manual_takeoff_live_lib.py as arrival_wall[ch] - max(arrival_wall), so
      t_wall + skew_s is that channel's own arrival time).

CHANNEL AVAILABILITY - stated, not silently worked around
---------------------------------------------------------
  ArduPilot servo output / PWM ............ AVAILABLE (RCOU + servo_pwm)
  actuator cmd_rad ........................ AVAILABLE (actuator diag)
  actual joint angle ...................... AVAILABLE (actuator diag)
  actual joint rate ....................... AVAILABLE (actuator diag)
  rate-clamp flag ......................... DERIVED - see note below
  effort-clamp flag ....................... AVAILABLE (effort_clamp_active)
  roll/pitch/yaw + p/q/r .................. AVAILABLE (IMU 50 Hz, ATT 25 Hz)
  aero Cl/Cm/Cn ........................... AVAILABLE (aero diag)
  actuator effort / torque ................ DATA_REQUIRED
      The 35-field diagnostics layout documented in actuator_v1_config.yaml
      (command_interface.diagnostics_field_order_per_surface) does NOT
      publish the commanded torque, only the saturation FLAG. The applied
      joint torque is therefore not recoverable from any existing record.
      Reported as DATA_REQUIRED, never inferred.
  rate-clamp flag ......................... DATA_REQUIRED as a FLAG.
      The plugin publishes no rate-clamp flag. What IS published is the
      measured joint rate, and the velocity limit is a physics-engine hard
      constraint (gz::sim::Joint::SetVelocityLimits, see
      actuator_v1_config.yaml placeholder_inertia_note). This script
      therefore counts samples whose measured |actual_rate_rad_s| sits ON
      that limit and labels the count RATE_PINNED (an observation), never
      "rate_clamp_active" (a flag that does not exist). Because the
      observation is made at 20 Hz on a limit that is enforced at the 1 kHz
      physics tick, every pinned count in this file is a LOWER BOUND.

CONSTANTS - every one cited or explicitly tagged
------------------------------------------------
See the CONST block below. No threshold in this file was chosen after seeing
a result; the burst detector is additionally reported at three multipliers so
its sensitivity is visible.

USAGE
    python3 analyze_airborne_actuator_limit_cycle.py
"""
import json
import math
import os
import sys

import numpy as np
from pymavlink import mavutil

REPO = "/home/emirhan/Desktop/FalconV2"
RES = os.path.join(REPO, "tests/gazebo/results")
SCRIPTS = os.path.join(REPO, "tests/gazebo/scripts")
sys.path.insert(0, SCRIPTS)
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

# The de-aliased liftoff detector and its three criterion numbers are REUSED
# unchanged from the stage harness rather than restated here.
import analyze_manual_takeoff_ground_roll_evidence as EV  # noqa: E402

# ---------------------------------------------------------------------------
# CONST - declared before any result is looked at.
# ---------------------------------------------------------------------------
SURFACES = ["left_aileron", "right_aileron", "left_elevator",
            "right_elevator", "rudder"]

# Servo mechanical/servo-model limits. READ from
# docs/source_of_truth/controls/actuator_v1_config.yaml (V1_PROVISIONAL).
MAX_RATE_RAD_S = 5.2360          # surfaces.*.max_rate_rad_s  (= 300.00 deg/s)
MAX_EFFORT_NM = 0.20             # surfaces.*.max_effort_nm
MIN_ANGLE_RAD = -0.7853981634    # surfaces.*.min_angle_rad   (= -45 deg)
MAX_ANGLE_RAD = 0.7853981634     # surfaces.*.max_angle_rad   (= +45 deg)

# Servo control law. READ from the same file, control_law block.
KP_NM_PER_RAD = 0.0034788        # control_law.kp_nm_per_rad
KD_NM_PER_RAD_S = 6.9576e-05     # control_law.kd_nm_per_rad_s
KI_NM_PER_RAD_S = 6.9576e-03     # control_law.ki_nm_per_rad_s
SP_WEIGHT_B = 0.7                # control_law.sp_weight_b
I_REF_KG_M2 = 3.4788e-07         # control_law.derivation_note reference inertia
                                 # (model.sdf aileron <iyy>, the LARGEST of the
                                 # three placeholder values; the gains were
                                 # pole-placed against exactly this number)

# Diagnostics publish rate. READ from actuator_v1_config.yaml
# command_interface.diagnostics_rate_hz. Used only to state the Nyquist limit.
DIAG_RATE_HZ_CONFIG = 20.0

# Body/geometry constants, READ from their cited sources - used only for
# order-of-magnitude cross-checks, never to replace a measurement.
IXX_KG_M2 = 0.7284      # base-link Ixx, quoted in
                        # tests/gazebo/scripts/manual_takeoff_live_lib.py
                        # sign-convention header (from model/model.sdf)
WING_AREA_M2 = 0.4514   # CLAUDE.md, manufacturer geometry
SPAN_M = 2.093          # CLAUDE.md, manufacturer geometry

# PWM -> joint radians. READ from manual_takeoff_live_lib.py, which reads it
# from model/model.sdf's ArduPilotPlugin <control> blocks at run time.
PWM_MULT_RAD = 1.5707963268
PWM_SERVO_MIN = 800
PWM_SERVO_MAX = 2200
PWM_QUANTUM_DEG = math.degrees(PWM_MULT_RAD / (PWM_SERVO_MAX - PWM_SERVO_MIN))

# --- thresholds, all ASSUMPTION-tagged, all fixed before any result --------
# ASSUMPTION T1: a sample is called RATE_PINNED when |actual_rate| is within
# 0.1% of the configured limit. 0.1% is a float-comparison tolerance, not a
# physical bound; the plugin clamps to exactly MAX_RATE_RAD_S so any looser
# band would also admit merely-fast samples.
RATE_PIN_FRACTION = 0.999
# ASSUMPTION T2: effort/target clamp flags are booleans published as floats.
CLAMP_FLAG_TRUE = 0.5
# ASSUMPTION T3: burst detector. A 0.5 s window is called a BURST when its
# roll-rate (GyrX) RMS exceeds BURST_RMS_MULT x the MEDIAN airborne 0.5 s
# window RMS of the SAME run. A self-normalising multiple is used instead of
# an absolute rad/s number so the detector cannot be tuned to a run. The
# headline multiplier is 5.0; 3.0 and 10.0 are also reported so the
# sensitivity of every burst-scoped number is visible.
BURST_WINDOW_S = 0.5
BURST_RMS_MULT = 5.0
BURST_RMS_MULT_SENS = (3.0, 5.0, 10.0)
# ASSUMPTION T4: "command is calm" = commanded joint angle peak-to-peak below
# 1.0 deg over a 1.0 s window. 1.0 deg is ~15.6 PWM microseconds, i.e. ~15x
# the 0.0643 deg transport quantum, so a calm window is genuinely calm and not
# merely quantisation-limited.
CALM_WINDOW_S = 1.0
CMD_CALM_P2P_DEG = 1.0
# ASSUMPTION T5: "actual joint is oscillating" = actual joint angle
# peak-to-peak above 3.0 deg in the same window, i.e. 3x the calm-command
# bar, so a flagged window cannot be explained by the command at all.
ACT_OSC_P2P_DEG = 3.0

MODE_TAKEOFF = "TAKEOFF"
RUNS = [(1, "TAKEOFF"), (2, "TAKEOFF"), (3, "TAKEOFF"),
        (1, "MANUAL"), (2, "MANUAL"), (3, "MANUAL"),
        (1, "FBWA"), (2, "FBWA"), (3, "FBWA")]


# ---------------------------------------------------------------------------
# Small numeric helpers (SI throughout; degrees only at report boundaries)
# ---------------------------------------------------------------------------
def p2p(x):
    x = np.asarray(x, dtype=float)
    return float(x.max() - x.min()) if x.size else float("nan")


def rms(x):
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x * x))) if x.size else float("nan")


def sine_fit(t, y, f_hz):
    """Least-squares fit y ~ c0 + a*cos(2*pi*f*t) + b*sin(2*pi*f*t).

    Returns (amplitude, phase_rad, offset, residual_fraction). Phase is the
    argument of a - j*b, i.e. y ~ c0 + A*cos(2*pi*f*t + phase). Works on the
    non-uniformly sampled gz records as well as the uniform dataflash ones,
    which is why a plain FFT is not used for the phase comparison.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    if t.size < 4:
        return float("nan"), float("nan"), float("nan"), float("nan")
    w = 2.0 * math.pi * f_hz
    M = np.column_stack([np.ones_like(t), np.cos(w * t), np.sin(w * t)])
    coef, *_ = np.linalg.lstsq(M, y, rcond=None)
    c0, a, b = coef
    amp = math.hypot(a, b)
    ph = math.atan2(-b, a)
    resid = y - M @ coef
    denom = float(np.var(y))
    frac = float(np.var(resid) / denom) if denom > 0 else float("nan")
    return float(amp), float(ph), float(c0), frac


def dft_peak(t, y, f_lo, f_hi, n=4096):
    """Peak frequency of a Lomb-style least-squares periodogram.

    Uniform FFT is not usable on the gz records (non-uniform arrivals), so the
    same single-frequency least-squares fit is swept over [f_lo, f_hi] and the
    frequency that explains the most variance is returned. On a uniform record
    this is equivalent to a zero-padded DFT magnitude peak.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float) - float(np.mean(y))
    if t.size < 8:
        return float("nan"), float("nan")
    freqs = np.linspace(f_lo, f_hi, n)
    best_f, best_p = float("nan"), -1.0
    w = 2.0 * math.pi * freqs[:, None] * t[None, :]
    C = np.cos(w) @ y
    S = np.sin(w) @ y
    CC = np.einsum("ij,ij->i", np.cos(w), np.cos(w))
    SS = np.einsum("ij,ij->i", np.sin(w), np.sin(w))
    power = np.where(CC > 0, C * C / CC, 0.0) + np.where(SS > 0, S * S / SS, 0.0)
    i = int(np.argmax(power))
    best_f = float(freqs[i])
    tot = float(np.sum(y * y))
    best_p = float(power[i] / tot) if tot > 0 else float("nan")
    return best_f, best_p


def zero_cross_freq(t, y):
    """Mean-removed positive-going zero-crossing frequency + its scatter."""
    t = np.asarray(t, float)
    y = np.asarray(y, float) - float(np.mean(y))
    xs = []
    for i in range(1, y.size):
        if y[i - 1] < 0.0 <= y[i]:
            dy = y[i] - y[i - 1]
            frac = (-y[i - 1] / dy) if dy != 0 else 0.0
            xs.append(t[i - 1] + frac * (t[i] - t[i - 1]))
    if len(xs) < 3:
        return {"n_crossings": len(xs), "f_hz": None, "period_std_s": None}
    per = np.diff(np.array(xs))
    return {"n_crossings": len(xs),
            "f_hz": float(1.0 / np.mean(per)),
            "period_mean_s": float(np.mean(per)),
            "period_std_s": float(np.std(per, ddof=1)) if per.size > 1 else 0.0}


def actuator_model_response(f_hz):
    """Closed-loop |theta/cmd| and phase of the DOCUMENTED servo model.

    Derived here (not read from a file) from actuator_v1_config.yaml's own
    control_law_form and derivation_note:
        torque = kp*(b*r - th) + ki*Integral(r - th) - kd*th_dot
        I*th_ddot = torque
    =>  th/r (s) = (kp*b*s + ki) / (I*s^3 + kd*s^2 + kp*s + ki)
    with I = I_ref, the exact inertia the gains were pole-placed against.
    This is the servo model AS DESIGNED, used only as the reference the
    MEASURED joint is compared against. It is not a new parameter.
    """
    s = 1j * 2.0 * math.pi * f_hz
    num = KP_NM_PER_RAD * SP_WEIGHT_B * s + KI_NM_PER_RAD_S
    den = (I_REF_KG_M2 * s ** 3 + KD_NM_PER_RAD_S * s ** 2
           + KP_NM_PER_RAD * s + KI_NM_PER_RAD_S)
    h = num / den
    return float(abs(h)), float(math.degrees(np.angle(h)))


def wrap_deg(x):
    return (x + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------------
# Record loaders - READ ONLY
# ---------------------------------------------------------------------------
def load_dataflash(path):
    m = mavutil.mavlink_connection(path)
    out = {"IMU0": [], "RCOU": [], "PIDR": [], "ATT": [], "SIM2": []}
    while True:
        msg = m.recv_match()
        if msg is None:
            break
        t = msg.get_type()
        if t == "IMU" and getattr(msg, "I", 0) == 0:
            out["IMU0"].append((msg.TimeUS / 1e6, msg.GyrX, msg.GyrY, msg.GyrZ))
        elif t == "RCOU":
            out["RCOU"].append((msg.TimeUS / 1e6, msg.C1, msg.C2, msg.C3,
                                msg.C4, msg.C5))
        elif t == "PIDR":
            out["PIDR"].append((msg.TimeUS / 1e6, msg.Tar, msg.Act,
                                msg.P, msg.I, msg.D, msg.SRate))
        elif t == "ATT":
            out["ATT"].append((msg.TimeUS / 1e6, msg.DesRoll, msg.Roll,
                               msg.DesPitch, msg.Pitch, msg.Yaw))
        elif t == "SIM2":
            out["SIM2"].append((msg.TimeUS / 1e6, msg.PN, msg.PE, msg.PD))
    return {k: np.array(v, dtype=float) for k, v in out.items()}


def measured_rate(arr):
    if arr.shape[0] < 2:
        return float("nan")
    return float((arr.shape[0] - 1) / (arr[-1, 0] - arr[0, 0]))


def gz_channel(samples, getter, skew_key):
    """De-duplicated (t, value...) for one gz publisher.

    Consecutive harness samples that carry the SAME underlying message are
    collapsed to their first occurrence, and the timestamp used is that
    publisher's own arrival time t_wall + skew_s[skew_key].
    """
    ts, vals = [], []
    last = None
    for s in samples:
        try:
            v = getter(s)
        except (KeyError, TypeError, IndexError):
            continue
        if v is None:
            continue
        key = tuple(v) if isinstance(v, (list, tuple)) else v
        if key == last:
            continue
        last = key
        sk = (s.get("skew_s") or {}).get(skew_key)
        ts.append(s["t_sim"] + (sk if isinstance(sk, (int, float)) else 0.0))
        vals.append(v)
    return np.array(ts, float), np.array(vals, float)


# ---------------------------------------------------------------------------
# Per-run analysis
# ---------------------------------------------------------------------------
def analyse_run(idx, mode):
    ts_path = os.path.join(
        RES, f"manual_takeoff_ground_roll_run{idx}_{mode}_timeseries.json")
    df_path = os.path.join(
        RES, f"manual_takeoff_ground_roll_run{idx}_{mode}_dataflash",
        "00000001.BIN")
    if not os.path.exists(ts_path):
        return {"run_index": idx, "mode": mode, "status": "TIMESERIES_MISSING"}

    with open(ts_path) as fh:
        rec = json.load(fh)
    S = rec["samples"]

    # ---- airborne window, using the stage harness's own de-aliased test ----
    vz = EV.dealiased_vertical_rate(S)
    resting = None
    for s in S:
        if s["phase"] == "settle":
            resting = s["gz_pos_world_m"][2]
    if resting is None:
        resting = S[0]["gz_pos_world_m"][2]
    lo_i = EV.find_liftoff(S, resting, vz)
    air = S[lo_i:] if lo_i is not None else []
    t0 = S[0]["t_wall"]

    out = {
        "run_index": idx,
        "mode": mode,
        "source_timeseries": ts_path,
        "source_dataflash": df_path if os.path.exists(df_path) else
                            "DATA_REQUIRED (no dataflash for this run)",
        "n_samples_total": len(S),
        "liftoff_sample_index": lo_i,
        "liftoff_t_rel_s": (S[lo_i]["t_wall"] - t0) if lo_i is not None else None,
        "n_airborne_samples": len(air),
    }
    if not air:
        out["status"] = "NO_AIRBORNE_SEGMENT"
        return out

    # ---- de-duplicated actuator / aero / attitude channels ----------------
    chans = {}
    for name in SURFACES:
        t, v = gz_channel(
            air,
            lambda s, n=name: [s["surface_full"][n]["cmd_rad"],
                               s["surface_full"][n]["setpoint_rad"],
                               s["surface_full"][n]["actual_angle_rad"],
                               s["surface_full"][n]["actual_rate_rad_s"],
                               s["surface_full"][n]["target_clamp_active"],
                               s["surface_full"][n]["effort_clamp_active"]],
            "actuator_diag")
        chans[name] = (t, v)
    t_aero, v_aero = gz_channel(
        air, lambda s: [s["aero"]["Cl"], s["aero"]["Cm"], s["aero"]["Cn"],
                        s["aero"]["V"], s["aero"]["qbar"]], "aero_diag")
    t_pwm, v_pwm = gz_channel(
        air, lambda s: list(s["servo_pwm"]), "servo_output_raw")

    t_ref = air[0]["t_sim"]
    span = air[-1]["t_sim"] - air[0]["t_sim"]
    out["airborne_window_s"] = float(span)
    out["effective_publish_rate_hz"] = {
        "actuator_diag": float((len(chans["left_aileron"][0]) - 1) / span),
        "aero_diag": float((len(t_aero) - 1) / span),
        "servo_output_raw": float((len(t_pwm) - 1) / span),
        "config_diagnostics_rate_hz": DIAG_RATE_HZ_CONFIG,
        "actuator_diag_nyquist_hz": float((len(chans["left_aileron"][0]) - 1)
                                          / span / 2.0),
    }

    # ---- Q4 / Q5 : pinned-rate and clamp census over the whole airborne leg
    census = {}
    for name in SURFACES:
        t, v = chans[name]
        rate = v[:, 3]
        pin = np.abs(rate) >= RATE_PIN_FRACTION * MAX_RATE_RAD_S
        census[name] = {
            "n_distinct_diag_samples": int(t.size),
            "n_rate_pinned": int(pin.sum()),
            "pct_rate_pinned": float(100.0 * pin.sum() / t.size) if t.size else None,
            "rate_pinned_is_LOWER_BOUND": (
                "observed at ~20 Hz diagnostics on a limit enforced at the "
                "1 kHz physics tick"),
            "n_effort_clamp_active": int((v[:, 5] >= CLAMP_FLAG_TRUE).sum()),
            "n_target_clamp_active": int((v[:, 4] >= CLAMP_FLAG_TRUE).sum()),
            "max_abs_rate_rad_s": float(np.abs(rate).max()),
            "max_abs_rate_deg_s": float(math.degrees(np.abs(rate).max())),
            "actual_angle_deg_min": float(math.degrees(v[:, 2].min())),
            "actual_angle_deg_max": float(math.degrees(v[:, 2].max())),
            "cmd_deg_min": float(math.degrees(v[:, 0].min())),
            "cmd_deg_max": float(math.degrees(v[:, 0].max())),
            "actual_hits_mechanical_limit": bool(
                (v[:, 2] <= MIN_ANGLE_RAD + 1e-9).any()
                or (v[:, 2] >= MAX_ANGLE_RAD - 1e-9).any()),
        }
    out["airborne_census"] = census

    # ---- dataflash (authoritative frequency view) -------------------------
    if not os.path.exists(df_path):
        out["status"] = "NO_DATAFLASH"
        return out
    df = load_dataflash(df_path)
    imu = df["IMU0"]
    out["dataflash_measured_rates_hz"] = {
        k: measured_rate(v) for k, v in df.items() if v.size}
    out["dataflash_nyquist_hz"] = {
        k: measured_rate(v) / 2.0 for k, v in df.items() if v.size}

    rcou = df["RCOU"]
    # Align the two clocks. The harness records Gazebo sim time (t_sim) and
    # the dataflash records ArduPilot sim time (TimeUS); in lockstep SITL both
    # advance with the same physics tick, so the mapping is a pure offset. The
    # offset is MEASURED, not assumed: it is the shift that maximises the
    # correlation between the harness's Gazebo world z and the log's own
    # SIM2 ground-truth altitude (-PD). Altitude is used rather than a servo
    # channel because it always varies, including in MANUAL where the aileron
    # output is constant and a servo correlation is degenerate.
    sim2 = df["SIM2"]
    gzt = np.array([sm["t_sim"] for sm in S], float)
    gzz = np.array([sm["gz_pos_world_m"][2] for sm in S], float)
    lag, corr = None, None
    if sim2.shape[0] > 20 and gzt.size > 20:
        a = gzz - gzz.mean()
        na = float(np.linalg.norm(a))
        best = (-2.0, None)
        for step, half in ((0.05, 60.0), (0.001, 0.2)):
            centre = best[1] if best[1] is not None else 0.0
            for off in np.arange(centre - half, centre + half, step):
                y = np.interp(gzt + off, sim2[:, 0], -sim2[:, 3])
                b = y - y.mean()
                nb = float(np.linalg.norm(b))
                c = float(np.dot(a, b) / (na * nb)) if na > 0 and nb > 0 else -2.0
                if c > best[0]:
                    best = (c, float(off))
        corr, lag = best[0], best[1]
    out["clock_alignment"] = {
        "method": "max correlation of Gazebo world z against dataflash SIM2 -PD",
        "offset_s_added_to_gz_t_sim": lag,
        "pearson_r": corr,
        "sim_time_vs_wall_time": (
            "channel timestamps are t_sim + skew_s[channel]; skew_s is a wall-"
            "clock age and the runs execute at RTF 0.99993-0.99996, so the "
            "wall/sim conversion error over a 0.5 s burst window is < 1e-4 s"),
    }
    if lag is None or (corr is not None and corr < 0.9):
        out["status"] = "CLOCK_ALIGNMENT_FAILED"
        return out

    def gz_t_to_df(t_sim):
        return np.asarray(t_sim, float) + lag

    air_lo_df = gz_t_to_df(air[0]["t_sim"])
    air_hi_df = gz_t_to_df(air[-1]["t_sim"])

    # ---- burst detection on the 50 Hz gyro, airborne only ------------------
    mask = (imu[:, 0] >= air_lo_df) & (imu[:, 0] <= air_hi_df)
    gt, gx, gy, gz_ = imu[mask, 0], imu[mask, 1], imu[mask, 2], imu[mask, 3]
    wins = []
    if gt.size:
        w0 = gt[0]
        while w0 + BURST_WINDOW_S <= gt[-1]:
            sel = (gt >= w0) & (gt < w0 + BURST_WINDOW_S)
            if sel.sum() >= 8:
                wins.append((w0, float(np.std(gx[sel]))))
            w0 += BURST_WINDOW_S
    med = float(np.median([w[1] for w in wins])) if wins else float("nan")
    out["airborne_gyroX_window_rms_median_rad_s"] = med
    out["burst_detector"] = {
        "window_s": BURST_WINDOW_S,
        "rule": "window GyrX RMS > mult x median airborne window RMS",
        "median_rad_s": med,
        "sensitivity": {},
    }
    bursts_by_mult = {}
    for mult in BURST_RMS_MULT_SENS:
        hot = [w for w in wins if w[1] > mult * med]
        merged = []
        for w0, r in hot:
            if merged and abs(w0 - merged[-1][1]) < 1e-9:
                merged[-1][1] = w0 + BURST_WINDOW_S
                merged[-1][2] = max(merged[-1][2], r)
            else:
                merged.append([w0, w0 + BURST_WINDOW_S, r])
        bursts_by_mult[mult] = merged
        out["burst_detector"]["sensitivity"][f"mult_{mult}"] = {
            "n_bursts": len(merged),
            "total_s": float(sum(b[1] - b[0] for b in merged)),
            "windows_df_t": [[round(b[0], 3), round(b[1], 3)] for b in merged],
        }
    bursts = bursts_by_mult[BURST_RMS_MULT]
    out["n_bursts"] = len(bursts)

    # ---- per-burst spectral / phase analysis ------------------------------
    burst_reports = []
    for (b0, b1, brms) in bursts:
        rep = {"df_window": [round(b0, 3), round(b1, 3)],
               "duration_s": round(b1 - b0, 3),
               "peak_window_rms_rad_s": brms,
               "peak_window_rms_deg_s": math.degrees(brms),
               "ABSOLUTE_SIZE_ANNOTATION": (
                   "the burst detector is RELATIVE (a multiple of this run's "
                   "own median airborne window RMS). peak_window_rms_deg_s is "
                   "the absolute size and must be read alongside it: a run "
                   "whose median is 0.004 deg/s will report 'bursts' that are "
                   "physically negligible.")}

        sel = (imu[:, 0] >= b0) & (imu[:, 0] <= b1)
        bt, bgx = imu[sel, 0], imu[sel, 1]
        if bt.size < 12:
            continue
        f_gx, pw = dft_peak(bt, bgx, 0.3, measured_rate(imu) / 2.0)
        zc = zero_cross_freq(bt, bgx)
        rep["aircraft_roll_rate_GyrX"] = {
            "source": "dataflash IMU[0].GyrX (50 Hz, Nyquist 25 Hz)",
            "peak_freq_hz": f_gx,
            "peak_variance_fraction": pw,
            "zero_crossing": zc,
            "p2p_rad_s": p2p(bgx),
            "p2p_deg_s": math.degrees(p2p(bgx)),
            "rms_rad_s": float(np.std(bgx)),
        }
        f0 = f_gx  # every phase below is referenced to this one frequency

        for label, arr, col in (("GyrY_pitch_rate", imu, 2),
                                ("GyrZ_yaw_rate", imu, 3)):
            s2 = (arr[:, 0] >= b0) & (arr[:, 0] <= b1)
            if s2.sum() >= 12:
                ff, pp = dft_peak(arr[s2, 0], arr[s2, col], 0.3,
                                  measured_rate(arr) / 2.0)
                A, ph, _, res = sine_fit(arr[s2, 0], arr[s2, col], f0)
                rep[label] = {"peak_freq_hz": ff, "peak_variance_fraction": pp,
                              "amp_at_f0_deg_s": math.degrees(A),
                              "phase_at_f0_deg": math.degrees(ph),
                              "unexplained_variance_at_f0": res,
                              "p2p_deg_s": math.degrees(p2p(arr[s2, col]))}

        # ArduPilot side
        s3 = (rcou[:, 0] >= b0) & (rcou[:, 0] <= b1)
        if s3.sum() >= 8:
            y = rcou[s3, 1]
            ff, pp = dft_peak(rcou[s3, 0], y, 0.3, measured_rate(rcou) / 2.0)
            A, ph, c0, res = sine_fit(rcou[s3, 0], y, f0)
            rep["ardupilot_RCOU_C1_aileron_pwm"] = {
                "source": "dataflash RCOU.C1 (25 Hz, Nyquist 12.5 Hz)",
                "peak_freq_hz": ff, "peak_variance_fraction": pp,
                "p2p_pwm_us": p2p(y),
                "p2p_equivalent_joint_deg": p2p(y) * PWM_QUANTUM_DEG,
                "amp_at_f0_pwm_us": A,
                "amp_at_f0_equivalent_joint_deg": A * PWM_QUANTUM_DEG,
                "phase_at_f0_deg": math.degrees(ph),
                "unexplained_variance_at_f0": res,
                "trim_pwm_us": c0,
            }
        pidr = df["PIDR"]
        s4 = (pidr[:, 0] >= b0) & (pidr[:, 0] <= b1)
        if s4.sum() >= 8:
            rep["ardupilot_PIDR_roll_rate_loop"] = {
                "source": "dataflash PIDR (25 Hz, Nyquist 12.5 Hz)",
                "target_p2p_deg_s": p2p(pidr[s4, 1]),
                "actual_p2p_deg_s": p2p(pidr[s4, 2]),
                "target_amp_at_f0_deg_s": sine_fit(pidr[s4, 0], pidr[s4, 1], f0)[0],
                "actual_amp_at_f0_deg_s": sine_fit(pidr[s4, 0], pidr[s4, 2], f0)[0],
                "target_phase_at_f0_deg": math.degrees(
                    sine_fit(pidr[s4, 0], pidr[s4, 1], f0)[1]),
                "actual_phase_at_f0_deg": math.degrees(
                    sine_fit(pidr[s4, 0], pidr[s4, 2], f0)[1]),
                "P_p2p": p2p(pidr[s4, 3]), "I_p2p": p2p(pidr[s4, 4]),
                "D_p2p": p2p(pidr[s4, 5]),
                "note": ("Tar is the roll-rate demand the outer roll-angle "
                         "loop asks for; Act is the rate the autopilot "
                         "measures. Act oscillating while Tar does not means "
                         "the oscillation is in the vehicle/plant, not in the "
                         "autopilot's own demand."),
            }

        # ---- gz actuator channels inside the same burst, same f0 ----------
        surf_rep = {}
        for name in SURFACES:
            t, v = chans[name]
            tdf = gz_t_to_df(t)
            s5 = (tdf >= b0) & (tdf <= b1)
            if s5.sum() < 8:
                continue
            tt = tdf[s5]
            cmd, sp, act, rate = v[s5, 0], v[s5, 1], v[s5, 2], v[s5, 3]
            Ac, phc, _, rc = sine_fit(tt, cmd, f0)
            As, phs, _, rs = sine_fit(tt, sp, f0)
            Aa, pha, _, ra = sine_fit(tt, act, f0)
            Ar, phr, _, rr = sine_fit(tt, rate, f0)
            fa, pa = dft_peak(tt, act, 0.3, 0.5 * (s5.sum() - 1) / (b1 - b0))
            # sub-sample chatter test: does the reported instantaneous rate
            # carry far more energy than the sampled angle trace can explain?
            dif = np.diff(act) / np.diff(tt)
            mdl_gain, mdl_phase = actuator_model_response(f0)
            surf_rep[name] = {
                "n_samples": int(s5.sum()),
                "cmd": {"amp_at_f0_deg": math.degrees(Ac),
                        "phase_at_f0_deg": math.degrees(phc),
                        "p2p_deg": math.degrees(p2p(cmd)),
                        "unexplained_variance_at_f0": rc},
                "setpoint_after_rate_limiter": {
                    "amp_at_f0_deg": math.degrees(As),
                    "phase_at_f0_deg": math.degrees(phs),
                    "p2p_deg": math.degrees(p2p(sp)),
                    "max_abs_diff_from_cmd_deg": math.degrees(
                        float(np.abs(sp - cmd).max())),
                    "unexplained_variance_at_f0": rs},
                "actual_angle": {"amp_at_f0_deg": math.degrees(Aa),
                                 "phase_at_f0_deg": math.degrees(pha),
                                 "p2p_deg": math.degrees(p2p(act)),
                                 "peak_freq_hz": fa,
                                 "peak_variance_fraction": pa,
                                 "unexplained_variance_at_f0": ra},
                "actual_rate": {"amp_at_f0_deg_s": math.degrees(Ar),
                                "phase_at_f0_deg": math.degrees(phr),
                                "p2p_deg_s": math.degrees(p2p(rate)),
                                "rms_deg_s": math.degrees(rms(rate)),
                                "unexplained_variance_at_f0": rr},
                "closed_loop_measured": {
                    "gain_actual_over_cmd": (Aa / Ac) if Ac > 0 else None,
                    "phase_actual_minus_cmd_deg": wrap_deg(
                        math.degrees(pha - phc)),
                },
                "closed_loop_documented_model": {
                    "gain": mdl_gain, "phase_deg": mdl_phase,
                    "basis": ("actuator_v1_config.yaml control_law with "
                              "I_ref=3.4788e-07 kg m^2")},
                "sub_sample_chatter_test": {
                    "rms_reported_rate_deg_s": math.degrees(rms(rate)),
                    "rms_finite_difference_rate_deg_s": math.degrees(rms(dif)),
                    "ratio_finitediff_over_reported": (
                        float(rms(dif) / rms(rate)) if rms(rate) > 0 else None),
                    "rate_amp_implied_by_angle_at_f0_deg_s": math.degrees(
                        2.0 * math.pi * f0 * Aa),
                    "rate_amp_measured_at_f0_deg_s": math.degrees(Ar),
                    "interpretation_rule": (
                        "If the joint carried large motion ABOVE the ~20 Hz "
                        "diagnostics Nyquist, the reported instantaneous rate "
                        "would be far larger than 2*pi*f0*angle_amplitude and "
                        "the finite-difference rate would be far smaller than "
                        "the reported rate. Both ratios near 1 mean the motion "
                        "is fully resolved at f0."),
                },
                "rate_saturated_triangle_test": {
                    "f_predicted_hz": (math.degrees(MAX_RATE_RAD_S)
                                       / (2.0 * math.degrees(p2p(act)))
                                       if p2p(act) > 0 else None),
                    "f_measured_hz": fa,
                    "relation": ("a fully slew-rate-saturated triangular limit "
                                 "cycle of peak-to-peak P at slew rate R covers "
                                 "2P of travel per period, so f = R/(2P). "
                                 "Agreement means the amplitude and frequency "
                                 "are set by the joint VELOCITY LIMIT, not by "
                                 "the command."),
                    "duty_rate_pinned_pct": float(
                        100.0 * (np.abs(rate)
                                 >= RATE_PIN_FRACTION * MAX_RATE_RAD_S).mean()),
                },
                "n_rate_pinned_in_burst": int(
                    (np.abs(rate) >= RATE_PIN_FRACTION * MAX_RATE_RAD_S).sum()),
                "n_effort_clamp_in_burst": int(
                    (v[s5, 5] >= CLAMP_FLAG_TRUE).sum()),
                "n_target_clamp_in_burst": int(
                    (v[s5, 4] >= CLAMP_FLAG_TRUE).sum()),
            }
        rep["surfaces"] = surf_rep

        # left/right phase relationships
        pr = {}
        for a_, b_, expect in (("left_aileron", "right_aileron",
                                "ANTI_PHASE (differential roll command)"),
                               ("left_elevator", "right_elevator",
                                "IN_PHASE (symmetric pitch command)")):
            if a_ in surf_rep and b_ in surf_rep:
                d = wrap_deg(surf_rep[a_]["actual_angle"]["phase_at_f0_deg"]
                             - surf_rep[b_]["actual_angle"]["phase_at_f0_deg"])
                pr[f"{a_}_minus_{b_}"] = {
                    "actual_angle_phase_diff_deg": d,
                    "amp_ratio": (surf_rep[a_]["actual_angle"]["amp_at_f0_deg"]
                                  / surf_rep[b_]["actual_angle"]["amp_at_f0_deg"]
                                  if surf_rep[b_]["actual_angle"]["amp_at_f0_deg"]
                                  else None),
                    "commanded_relationship": expect,
                    "classification": ("ANTI_PHASE" if abs(abs(d) - 180.0) < 45.0
                                       else "IN_PHASE" if abs(d) < 45.0
                                       else "NEITHER_INDEPENDENT_LOOKING"),
                }
        rep["left_right_phase"] = pr

        # aero moments at the same f0
        s6 = (gz_t_to_df(t_aero) >= b0) & (gz_t_to_df(t_aero) <= b1)
        if s6.sum() >= 8:
            ta = gz_t_to_df(t_aero)[s6]
            aero = {}
            for j, nm in ((0, "Cl"), (1, "Cm"), (2, "Cn")):
                A, ph, _, r_ = sine_fit(ta, v_aero[s6, j], f0)
                ff, pp = dft_peak(ta, v_aero[s6, j], 0.3,
                                  0.5 * (s6.sum() - 1) / (b1 - b0))
                aero[nm] = {"amp_at_f0": A, "phase_at_f0_deg": math.degrees(ph),
                            "p2p": p2p(v_aero[s6, j]), "peak_freq_hz": ff,
                            "peak_variance_fraction": pp,
                            "unexplained_variance_at_f0": r_}
            qbar = float(np.mean(v_aero[s6, 4]))
            aero["qbar_mean_pa"] = qbar
            aero["Mx_amp_at_f0_Nm"] = (aero["Cl"]["amp_at_f0"] * qbar
                                       * WING_AREA_M2 * SPAN_M)
            aero["implied_roll_accel_amp_rad_s2"] = (
                aero["Mx_amp_at_f0_Nm"] / IXX_KG_M2)
            aero["measured_roll_accel_amp_rad_s2"] = (
                2.0 * math.pi * f0
                * (rep["aircraft_roll_rate_GyrX"]["p2p_rad_s"] / 2.0))
            aero["consistency_note"] = (
                "implied vs measured roll acceleration amplitude - an "
                "order-of-magnitude closure check on Ixx=0.7284 kg m^2, not a "
                "pass/fail criterion")
            rep["aero_moments"] = aero
        burst_reports.append(rep)
    out["bursts"] = burst_reports

    # ---- Q2: does the ACTUAL joint move while the COMMAND is calm? --------
    calm = {}
    for name in SURFACES:
        t, v = chans[name]
        t_rel = t - t_ref
        hits, n_win = [], 0
        w0 = 0.0
        while w0 + CALM_WINDOW_S <= (t_rel[-1] if t_rel.size else 0.0):
            sel = (t_rel >= w0) & (t_rel < w0 + CALM_WINDOW_S)
            if sel.sum() >= 8:
                n_win += 1
                cp = math.degrees(p2p(v[sel, 0]))
                ap = math.degrees(p2p(v[sel, 2]))
                if cp <= CMD_CALM_P2P_DEG and ap > ACT_OSC_P2P_DEG:
                    hits.append({"t_rel_s": round(w0, 2),
                                 "cmd_p2p_deg": cp, "actual_p2p_deg": ap})
            w0 += CALM_WINDOW_S
        calm[name] = {
            "n_airborne_windows": n_win,
            "n_windows_cmd_calm_and_actual_oscillating": len(hits),
            "worst": max(hits, key=lambda h: h["actual_p2p_deg"]) if hits else None,
            "criterion": (f"cmd p2p <= {CMD_CALM_P2P_DEG} deg AND actual p2p "
                          f"> {ACT_OSC_P2P_DEG} deg over {CALM_WINDOW_S} s"),
        }
    out["cmd_calm_actual_oscillating"] = calm

    # ---- driver characterisation: joint motion vs BASE angular acceleration
    # The four surfaces whose hinge axes are +Y-dominant (model/model.sdf
    # left/right_aileron_joint <axis><xyz> = (0.000994, 0.999899, 0.014146) and
    # left/right_elevator_joint = (0.002983, 0.999993, -0.002072)) share the
    # body PITCH axis; rudder_joint's axis is (0.010378, 0.0, 0.999946), i.e.
    # +Z-dominant. The rudder is therefore the built-in control surface for a
    # +Y-axis driver, in the same record, under the same command.
    drive = {"axis_provenance": {
        "left/right_aileron_joint": "model/model.sdf <axis><xyz> 0.000994 0.999899 0.014146 (+Y dominant)",
        "left/right_elevator_joint": "model/model.sdf <axis><xyz> 0.002983 0.999993 -0.002072 (+Y dominant)",
        "rudder_joint": "model/model.sdf <axis><xyz> 0.010378 0.000000 0.999946 (+Z dominant)",
        "surface link mass/inertia": ("model/model.sdf: every control-surface "
                                      "link mass = 0.001 kg TEMPORARY_NUMERICAL_MASS; "
                                      "left_aileron <iyy> = 3.4788e-07 kg m^2 "
                                      "(the I_ref the gains were pole-placed "
                                      "against); rudder <izz> = 1.7775e-07"),
        "joint damping/friction": ("model/model.sdf declares NO <dynamics> "
                                   "block on any of the five joints - zero "
                                   "mechanical damping and zero friction; the "
                                   "only damping in the loop is kd"),
    }, "windows": []}
    gyv = imu
    w0 = air_lo_df
    while w0 + CALM_WINDOW_S <= air_hi_df:
        m2 = (gyv[:, 0] >= w0) & (gyv[:, 0] < w0 + CALM_WINDOW_S)
        if m2.sum() >= 12:
            gy = gyv[m2, 2]
            tg = gyv[m2, 0]
            alpha_y = np.diff(gy) / np.diff(tg)
            row = {"df_t": round(float(w0), 2),
                   "gyroY_rms_rad_s": float(np.std(gy)),
                   "base_pitch_ang_accel_rms_rad_s2": float(np.std(alpha_y)),
                   "gyroX_rms_rad_s": float(np.std(gyv[m2, 1])),
                   "surfaces": {}}
            for name in SURFACES:
                t, v = chans[name]
                tdf = gz_t_to_df(t)
                m3 = (tdf >= w0) & (tdf < w0 + CALM_WINDOW_S)
                if m3.sum() >= 8:
                    row["surfaces"][name] = {
                        "cmd_p2p_deg": math.degrees(p2p(v[m3, 0])),
                        "actual_p2p_deg": math.degrees(p2p(v[m3, 2])),
                        "pct_rate_pinned": float(100.0 * (
                            np.abs(v[m3, 3])
                            >= RATE_PIN_FRACTION * MAX_RATE_RAD_S).mean()),
                    }
            drive["windows"].append(row)
        w0 += CALM_WINDOW_S
    out["driver_characterisation"] = drive

    # ---- Q1: onset ordering ------------------------------------------------
    if bursts:
        b0 = bursts[0][0]
        pre_lo, pre_hi = b0 - 6.0, b0 - 4.0
        onset = {"first_burst_df_t": round(b0, 3),
                 "baseline_window_df_t": [round(pre_lo, 3), round(pre_hi, 3)],
                 "rule": ("for each channel: the first time after the baseline "
                          "window at which |signal - baseline mean| exceeds "
                          "6x the baseline standard deviation of that SAME "
                          "channel, searched forward to the burst start"),
                 "channels": {}}

        def onset_of(t, y, label, unit):
            bsel = (t >= pre_lo) & (t <= pre_hi)
            if bsel.sum() < 6:
                return None
            mu, sd = float(np.mean(y[bsel])), float(np.std(y[bsel]))
            if sd <= 0:
                return None
            ssel = (t > pre_hi) & (t <= b0 + BURST_WINDOW_S)
            tt, yy = t[ssel], y[ssel]
            for i in range(tt.size):
                if abs(yy[i] - mu) > 6.0 * sd:
                    return {"label": label, "unit": unit,
                            "onset_df_t": float(tt[i]),
                            "baseline_mean": mu, "baseline_std": sd,
                            "value_at_onset": float(yy[i])}
            return {"label": label, "unit": unit, "onset_df_t": None,
                    "baseline_mean": mu, "baseline_std": sd}

        ch_list = [
            ("IMU_GyrX_roll_rate", imu[:, 0], imu[:, 1], "rad/s"),
            ("PIDR_target_roll_rate", df["PIDR"][:, 0], df["PIDR"][:, 1], "deg/s"),
            ("PIDR_actual_roll_rate", df["PIDR"][:, 0], df["PIDR"][:, 2], "deg/s"),
            ("RCOU_C1_aileron_pwm", rcou[:, 0], rcou[:, 1], "us"),
            ("RCOU_C2_elevator_pwm", rcou[:, 0], rcou[:, 2], "us"),
        ]
        for name in SURFACES:
            t, v = chans[name]
            ch_list.append((f"actuator_{name}_cmd", gz_t_to_df(t), v[:, 0], "rad"))
            ch_list.append((f"actuator_{name}_actual", gz_t_to_df(t), v[:, 2], "rad"))
        for label, t, y, unit in ch_list:
            r = onset_of(np.asarray(t, float), np.asarray(y, float), label, unit)
            if r:
                onset["channels"][label] = r
        ordered = sorted(
            [(v["onset_df_t"], k) for k, v in onset["channels"].items()
             if v.get("onset_df_t") is not None])
        onset["ordering_earliest_first"] = [
            {"channel": k, "onset_df_t": round(t, 3),
             "s_before_burst": round(b0 - t, 3)} for t, k in ordered]
        out["onset_ordering"] = onset
    out["status"] = "OK"
    return out


def analyse_sign_scenarios():
    """Control condition: the six A-F live sign scenarios.

    The 2026-09-10 stage reported 0 of 4890-4905 rate-pinned samples in every
    one of these. They are re-measured here with the SAME pin criterion so the
    negative result is this stage's own, not a quotation.
    """
    out = []
    for sc in "ABCDEF":
        path = os.path.join(
            RES, f"control_surface_live_sign_scenario_{sc}_timeseries.json")
        if not os.path.exists(path):
            out.append({"scenario": sc, "status": "MISSING"})
            continue
        with open(path) as fh:
            rec = json.load(fh)
        rows = []
        for w in ("before", "during", "after"):
            rows.extend(rec["windows"].get(w, []))
        per = {}
        for name in SURFACES:
            ang, rate, cmd, eff = [], [], [], 0
            for r in rows:
                a = r["actuator_full"][name]
                ang.append(a["actual_angle_rad"])
                rate.append(a["actual_rate_rad_s"])
                cmd.append(a["cmd_rad"])
                eff += 1 if a["effort_clamp_active"] >= CLAMP_FLAG_TRUE else 0
            rate = np.asarray(rate, float)
            per[name] = {
                "n_samples": len(rows),
                "n_rate_pinned": int(
                    (np.abs(rate) >= RATE_PIN_FRACTION * MAX_RATE_RAD_S).sum()),
                "n_effort_clamp_active": eff,
                "max_abs_rate_deg_s": math.degrees(float(np.abs(rate).max())),
                "actual_p2p_deg": math.degrees(p2p(ang)),
                "cmd_p2p_deg": math.degrees(p2p(cmd)),
            }
        vs = [r["aero"]["V"] for r in rows if "aero" in r]
        out.append({"scenario": sc, "name": rec.get("name"),
                    "source": path,
                    "mean_airspeed_ms": float(np.mean(vs)) if vs else None,
                    "per_surface": per})
    return out


def main():
    result = {
        "stage": "AIRBORNE_ACTUATOR_LIMIT_CYCLE_ROOT_CAUSE",
        "date": "2026-09-10",
        "owner": "controls-integration",
        "classification": "MEASUREMENT_ONLY_READ_ONLY_OFFLINE",
        "simulation_launched": False,
        "parameters_changed": "NONE",
        "defect_under_investigation": {
            "test_report": ("docs/test_results/2026-09-10_manual_takeoff_and_"
                            "live_control_surface_behavior_validation.md sec 7b "
                            "row i3"),
            "validation": ("docs/validation/2026-09-10_manual_takeoff_and_live_"
                           "control_surface_behavior_validation.md finding i3"),
        },
        "constants": {
            "MAX_RATE_RAD_S": MAX_RATE_RAD_S,
            "MAX_RATE_DEG_S": math.degrees(MAX_RATE_RAD_S),
            "MAX_EFFORT_NM": MAX_EFFORT_NM,
            "KP_NM_PER_RAD": KP_NM_PER_RAD,
            "KD_NM_PER_RAD_S": KD_NM_PER_RAD_S,
            "KI_NM_PER_RAD_S": KI_NM_PER_RAD_S,
            "SP_WEIGHT_B": SP_WEIGHT_B,
            "I_REF_KG_M2": I_REF_KG_M2,
            "IXX_KG_M2": IXX_KG_M2,
            "PWM_QUANTUM_DEG": PWM_QUANTUM_DEG,
            "provenance": {
                "servo limits + control law":
                    "docs/source_of_truth/controls/actuator_v1_config.yaml",
                "IXX_KG_M2":
                    "model/model.sdf base link, quoted in "
                    "tests/gazebo/scripts/manual_takeoff_live_lib.py header",
                "PWM mapping":
                    "model/model.sdf ArduPilotPlugin <control> blocks, via "
                    "manual_takeoff_live_lib.py",
                "wing area / span": "CLAUDE.md manufacturer geometry",
            },
        },
        "thresholds_ASSUMPTION": {
            "RATE_PIN_FRACTION": RATE_PIN_FRACTION,
            "CLAMP_FLAG_TRUE": CLAMP_FLAG_TRUE,
            "BURST_WINDOW_S": BURST_WINDOW_S,
            "BURST_RMS_MULT": BURST_RMS_MULT,
            "BURST_RMS_MULT_SENS": list(BURST_RMS_MULT_SENS),
            "CALM_WINDOW_S": CALM_WINDOW_S,
            "CMD_CALM_P2P_DEG": CMD_CALM_P2P_DEG,
            "ACT_OSC_P2P_DEG": ACT_OSC_P2P_DEG,
            "note": ("all fixed before any result was inspected; the burst "
                     "detector is reported at three multipliers so its "
                     "sensitivity is visible"),
        },
        "channel_availability": {
            "ardupilot_servo_output_pwm": "AVAILABLE",
            "actuator_cmd_rad": "AVAILABLE",
            "actual_joint_angle": "AVAILABLE",
            "actual_joint_rate": "AVAILABLE",
            "effort_clamp_flag": "AVAILABLE",
            "target_clamp_flag": "AVAILABLE",
            "rate_clamp_FLAG": ("DATA_REQUIRED - the plugin publishes no "
                                "rate-clamp flag; RATE_PINNED counts below are "
                                "an OBSERVATION of |rate| sitting on the limit, "
                                "sampled at ~20 Hz, and are LOWER BOUNDS"),
            "actuator_effort_torque_Nm": (
                "DATA_REQUIRED - not present in the 35-field diagnostics "
                "layout documented in actuator_v1_config.yaml"),
            "body_rates_p_q_r": "AVAILABLE (dataflash IMU 50 Hz)",
            "aero_Cl_Cm_Cn": "AVAILABLE (aero diagnostics ~20 Hz)",
        },
        "runs": [],
    }
    result["sign_scenario_control_condition"] = analyse_sign_scenarios()
    for idx, mode in RUNS:
        print(f"[analyse] run{idx} {mode} ...", flush=True)
        result["runs"].append(analyse_run(idx, mode))
    # ---- roll-up used verbatim by the stage report -----------------------
    tot_surface_samples = 0
    tot_effort_clamp = 0
    tot_pinned = 0
    per_mode = {}
    for r in result["runs"]:
        if r.get("status") != "OK":
            continue
        mode = r["mode"]
        agg = per_mode.setdefault(mode, {"n_runs": 0, "surfaces": {}})
        agg["n_runs"] += 1
        for name, c in r["airborne_census"].items():
            d0 = agg["surfaces"].setdefault(name, {"n": 0, "pinned": 0,
                                                   "effort": 0, "target": 0,
                                                   "act_min": 1e9,
                                                   "act_max": -1e9})
            d0["n"] += c["n_distinct_diag_samples"]
            d0["pinned"] += c["n_rate_pinned"]
            d0["effort"] += c["n_effort_clamp_active"]
            d0["target"] += c["n_target_clamp_active"]
            d0["act_min"] = min(d0["act_min"], c["actual_angle_deg_min"])
            d0["act_max"] = max(d0["act_max"], c["actual_angle_deg_max"])
            tot_surface_samples += c["n_distinct_diag_samples"]
            tot_effort_clamp += c["n_effort_clamp_active"]
            tot_pinned += c["n_rate_pinned"]
    for sc in result["sign_scenario_control_condition"]:
        for name, c in sc.get("per_surface", {}).items():
            tot_surface_samples += c["n_samples"]
            tot_effort_clamp += c["n_effort_clamp_active"]
            tot_pinned += c["n_rate_pinned"]
    for mode, agg in per_mode.items():
        for name, d0 in agg["surfaces"].items():
            d0["pct_pinned"] = 100.0 * d0["pinned"] / d0["n"] if d0["n"] else None
    result["rollup"] = {
        "total_surface_samples_examined": tot_surface_samples,
        "total_effort_clamp_active_samples": tot_effort_clamp,
        "total_rate_pinned_samples": tot_pinned,
        "per_mode_airborne": per_mode,
    }
    outp = os.path.join(RES, "airborne_actuator_limit_cycle_analysis.json")
    with open(outp, "w") as fh:
        json.dump(result, fh, indent=1)
    print("wrote", outp)


if __name__ == "__main__":
    main()

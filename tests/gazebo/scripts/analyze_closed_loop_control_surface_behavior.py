#!/usr/bin/env python3
"""FALCON V2 - OFFLINE closed-loop control-surface behaviour analysis.

CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY / OFFLINE.
Stage: MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION (Part 3).
Owner: controls-integration.  Created 2026-09-10.

WHAT THIS IS
------------
A pure post-processor over ALREADY-RECORDED navigation/FBWA timeseries. It
launches NO simulation, opens NO transport, and writes NOTHING outside
tests/gazebo/results/. It changes NO physics parameter of any kind (no aero
coefficient, no propulsion coefficient, no mass/CG/inertia, no actuator PID /
rate / effort limit, no +/-45 deg mapping, no PTCH_TRIM_DEG, no TECS_PTCH_DAMP,
no roll/pitch PID, no sensor model, no collision/friction value).

INPUTS (read-only, pre-existing):
  tests/gazebo/results/ardupilot_navigation_scenario_{A,B,C,D}_timeseries.json
  tests/gazebo/results/ardupilot_fbwa_level_pitch_reference_correction_result.json
  tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_timeseries_ptchdamp06.json
  docs/source_of_truth/aerodynamics/aero_v1_config.yaml  (sign scalars only)

SIGN CONVENTION USED (NOT invented here - cited, see report):
  docs/source_of_truth/controls/CONTROLS.md sec 10:
      delta_e_aero = -0.5 * (theta_left_elevator + theta_right_elevator)
      delta_a_aero = +0.5 * (theta_right_aileron - theta_left_aileron)
      delta_r_aero =        theta_rudder
  Physical meaning (VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST, 2026-08-22):
      + joint angle -> trailing edge UP  (both ailerons, both elevators)
      + joint angle -> rudder trailing edge toward -Y (RIGHT in FLU)
      delta_a_aero > 0 -> Mx > 0 (FLU) -> roll RIGHT
      delta_e_aero < 0 -> nose UP
      delta_r_aero > 0 -> Mz < 0 (FLU) -> nose RIGHT

NO MAGIC NUMBERS: every threshold below is declared in THRESHOLDS with a
stated basis. All are ANALYSIS-REPORTING thresholds (they select which events
to tabulate); none is a pass/fail gate on aircraft physics and none feeds back
into any simulation parameter.
"""
import json
import math
import os
import sys

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = os.path.join(REPO_ROOT, "tests/gazebo/results")
AERO_CFG = os.path.join(REPO_ROOT, "docs/source_of_truth/aerodynamics/aero_v1_config.yaml")
OUT_JSON = os.path.join(RESULTS_DIR, "closed_loop_control_surface_behavior_analysis.json")

SURFACES = ["left_aileron", "right_aileron", "left_elevator", "right_elevator", "rudder"]

# ---------------------------------------------------------------------------
# Reporting thresholds. Each has an explicit documented basis. These only
# choose WHICH samples get tabulated as "events"; they are not physics.
# ---------------------------------------------------------------------------
THRESHOLDS = {
    "rudder_deadband_deg": {
        "value": 0.5,
        "basis": "ASSUMPTION (reporting only). 0.5 deg is ~1.1% of the "
                 "+/-45 deg mechanical travel (actuator_v1_config.yaml "
                 "min/max_angle_rad) and comfortably above the observed "
                 "neutral-hold droop of the servo model (~0.06 deg, see "
                 "actuator PID droop note in actuator_v1_config.yaml). Used "
                 "ONLY to compute 'percent of time the rudder is doing "
                 "something' - no physics depends on it.",
    },
    "roll_demand_onset_deg": {
        "value": 5.0,
        "basis": "ASSUMPTION (reporting only). Picks nav_roll_deg excursions "
                 "clearly above the ~0.05 deg wings-level command noise seen "
                 "in the settle_fbwa segments, so 'onset' events are real "
                 "commanded turns and not quantisation.",
    },
    "pitch_demand_onset_deg": {
        "value": 2.0,
        "basis": "ASSUMPTION (reporting only). Same rationale as "
                 "roll_demand_onset_deg, scaled to the much smaller pitch "
                 "demand range ArduPlane uses in level cruise.",
    },
    "reversal_min_amplitude_deg": {
        "value": 2.0,
        "basis": "ASSUMPTION (reporting only). A sign change of the aero "
                 "deflection is only counted as a 'reversal event' if the "
                 "deflection reaches this magnitude on BOTH sides of the zero "
                 "crossing, so zero-crossing chatter is not reported as a "
                 "reversal.",
    },
}


# Reliability gate for the actuator-lag regression. Both are ANALYSIS-quality
# gates on the ESTIMATOR, not on the aircraft.
TAU_FIT_MIN_R2 = 0.30        # ASSUMPTION (reporting only): below this the
                             # dact-vs-error regression explains too little
                             # variance to quote a tau at all.
TAU_FIT_MIN_CMD_RMS_DEG = 1.0  # ASSUMPTION (reporting only): a command whose
                             # RMS is under 1 deg is comparable to the PWM
                             # quantisation step of this transport
                             # (1 PWM us -> 1.5707963268/1400 rad = 0.0643 deg,
                             # model.sdf ArduPilotPlugin <control> multiplier
                             # and servo_min/servo_max), so a tau fitted to it
                             # is quantisation noise, not servo dynamics.


def load_json(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def read_sign_scalars():
    """Parse the three control_mapping sign scalars out of aero_v1_config.yaml
    WITHOUT importing yaml (keeps this analyser dependency-free) and WITHOUT
    modifying the file. Values are asserted against CONTROLS.md sec 10 so a
    silent drift in the source of truth is caught rather than absorbed."""
    signs = {}
    with open(AERO_CFG, "r", encoding="utf-8") as fh:
        for line in fh:
            stripped = line.strip()
            for key in ("aileron_sign", "elevator_sign", "rudder_sign"):
                if stripped.startswith(key + ":"):
                    if key in signs:
                        continue
                    val = stripped.split(":", 1)[1].split("#")[0].strip()
                    signs[key] = float(val)
    missing = {"aileron_sign", "elevator_sign", "rudder_sign"} - set(signs)
    if missing:
        raise RuntimeError(f"could not parse sign scalars {missing} from {AERO_CFG}")
    expected = {"aileron_sign": 1.0, "elevator_sign": -1.0, "rudder_sign": 1.0}
    if signs != expected:
        raise RuntimeError(
            f"control_mapping signs {signs} differ from the CONTROLS.md sec 10 "
            f"VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST values {expected}. This "
            f"analyser refuses to silently reinterpret a changed convention.")
    return signs


# ---------------------------------------------------------------------------
# Small dependency-free statistics helpers (no numpy requirement).
# ---------------------------------------------------------------------------
def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def pearson(xs, ys):
    n = min(len(xs), len(ys))
    if n < 3:
        return float("nan")
    mx, my = mean(xs[:n]), mean(ys[:n])
    sxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    sxx = sum((xs[i] - mx) ** 2 for i in range(n))
    syy = sum((ys[i] - my) ** 2 for i in range(n))
    if sxx <= 0.0 or syy <= 0.0:
        return float("nan")
    return sxy / math.sqrt(sxx * syy)


def best_lag_correlation(xs, ys, max_shift):
    """Return (best_r, best_shift) maximising |corr(x[k], y[k+shift])|.
    A POSITIVE shift means ys lags xs by `shift` samples (ys responds later)."""
    best = (float("nan"), 0)
    n = len(xs)
    for shift in range(-max_shift, max_shift + 1):
        if shift >= 0:
            a, b = xs[:n - shift] if shift else xs, ys[shift:]
        else:
            a, b = xs[-shift:], ys[:n + shift]
        m = min(len(a), len(b))
        if m < 10:
            continue
        r = pearson(list(a[:m]), list(b[:m]))
        if math.isnan(r):
            continue
        if math.isnan(best[0]) or abs(r) > abs(best[0]):
            best = (r, shift)
    return best


# ---------------------------------------------------------------------------
# Per-sample engineering extraction
# ---------------------------------------------------------------------------
REQUIRED_MAV_FIELDS = (
    "nav_roll_deg", "att_roll_deg", "nav_pitch_deg", "att_pitch_deg",
    "rollspeed_deg_s", "pitchspeed_deg_s", "yawspeed_deg_s", "att_yaw_deg")


def usable(smp):
    """A sample is usable only if every MAVLink field this analysis
    differences is present. Early samples in a segment can predate the first
    NAV_CONTROLLER_OUTPUT message, so nav_roll_deg/nav_pitch_deg are None.
    Those are DROPPED and COUNTED - never defaulted to 0, which would
    manufacture a fake 'wings level demand'."""
    mv = smp.get("mav") or {}
    return all(mv.get(k) is not None for k in REQUIRED_MAV_FIELDS)


def extract(samples, signs):
    """Turn raw samples into flat arrays of engineering quantities, in degrees
    where an angle, using ONLY the CONTROLS.md sec 10 mapping."""
    out = {k: [] for k in (
        "t", "nav_roll", "att_roll", "roll_err", "rollspeed",
        "nav_pitch", "att_pitch", "pitch_err", "pitchspeed", "yawspeed",
        "da_cmd", "da_act", "de_cmd", "de_act", "dr_cmd", "dr_act",
        "Cl", "Cm", "Cn", "V", "beta", "servo_ail", "servo_ele", "servo_rud",
        "custom_mode", "xtrack", "nav_bearing", "target_bearing", "yaw")}
    per_surface_cmd = {s: [] for s in SURFACES}
    per_surface_act = {s: [] for s in SURFACES}
    for smp in samples:
        mv, ac, ae = smp["mav"], smp["actuators"], smp.get("aero", {})
        deg = math.degrees
        la = ac["left_aileron"]; ra = ac["right_aileron"]
        le = ac["left_elevator"]; re_ = ac["right_elevator"]; rd = ac["rudder"]
        for s in SURFACES:
            per_surface_cmd[s].append(ac[s]["cmd_rad"])
            per_surface_act[s].append(ac[s]["actual_angle_rad"])
        # CONTROLS.md sec 10 mapping, applied identically to the COMMANDED
        # angles and to the ACTUAL joint angles.
        da_cmd = 0.5 * signs["aileron_sign"] * (ra["cmd_rad"] - la["cmd_rad"])
        da_act = 0.5 * signs["aileron_sign"] * (ra["actual_angle_rad"] - la["actual_angle_rad"])
        de_cmd = 0.5 * signs["elevator_sign"] * (le["cmd_rad"] + re_["cmd_rad"])
        de_act = 0.5 * signs["elevator_sign"] * (le["actual_angle_rad"] + re_["actual_angle_rad"])
        dr_cmd = signs["rudder_sign"] * rd["cmd_rad"]
        dr_act = signs["rudder_sign"] * rd["actual_angle_rad"]
        srv = mv.get("servo_raw") or [None] * 5
        out["t"].append(smp["t"])
        out["nav_roll"].append(mv["nav_roll_deg"])
        out["att_roll"].append(mv["att_roll_deg"])
        out["roll_err"].append(mv["nav_roll_deg"] - mv["att_roll_deg"])
        out["rollspeed"].append(mv["rollspeed_deg_s"])
        out["nav_pitch"].append(mv["nav_pitch_deg"])
        out["att_pitch"].append(mv["att_pitch_deg"])
        out["pitch_err"].append(mv["nav_pitch_deg"] - mv["att_pitch_deg"])
        out["pitchspeed"].append(mv["pitchspeed_deg_s"])
        out["yawspeed"].append(mv["yawspeed_deg_s"])
        out["yaw"].append(mv["att_yaw_deg"])
        out["da_cmd"].append(deg(da_cmd)); out["da_act"].append(deg(da_act))
        out["de_cmd"].append(deg(de_cmd)); out["de_act"].append(deg(de_act))
        out["dr_cmd"].append(deg(dr_cmd)); out["dr_act"].append(deg(dr_act))
        out["Cl"].append(ae.get("Cl", float("nan")))
        out["Cm"].append(ae.get("Cm", float("nan")))
        out["Cn"].append(ae.get("Cn", float("nan")))
        out["V"].append(ae.get("V", float("nan")))
        out["beta"].append(math.degrees(ae["beta"]) if ae.get("beta") is not None else float("nan"))
        out["servo_ail"].append(srv[0]); out["servo_ele"].append(srv[1])
        out["servo_rud"].append(srv[3])
        out["custom_mode"].append(mv.get("custom_mode"))
        out["xtrack"].append(mv.get("xtrack_error_m"))
        out["nav_bearing"].append(mv.get("nav_bearing_deg"))
        out["target_bearing"].append(mv.get("target_bearing_deg"))
    out["_cmd"] = per_surface_cmd
    out["_act"] = per_surface_act
    return out


def sample_period(ts):
    if len(ts) < 2:
        return float("nan")
    dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    dts = [d for d in dts if d > 0]
    return mean(dts) if dts else float("nan")


# ---------------------------------------------------------------------------
# Analyses
# ---------------------------------------------------------------------------
def analyse_tracking(d, dt):
    """Aileron-vs-roll and elevator-vs-pitch closed-loop tracking."""
    max_shift = max(1, int(round(2.0 / dt)))  # +/-2.0 s search window
    res = {}
    res["aileron"] = {
        "corr_rollerr_vs_da_act": pearson(d["roll_err"], d["da_act"]),
        "corr_navroll_vs_da_act": pearson(d["nav_roll"], d["da_act"]),
        "corr_attroll_vs_da_act": pearson(d["att_roll"], d["da_act"]),
        "corr_da_act_vs_Cl": pearson(d["da_act"], d["Cl"]),
        "corr_da_act_vs_rollspeed": pearson(d["da_act"], d["rollspeed"]),
        "da_act_deg_rms": rms(d["da_act"]),
        "da_act_deg_max_abs": max(abs(x) for x in d["da_act"]),
    }
    r, sh = best_lag_correlation(d["da_act"], d["rollspeed"], max_shift)
    res["aileron"]["best_corr_da_act_to_rollspeed"] = r
    res["aileron"]["best_lag_da_act_to_rollspeed_s"] = sh * dt
    r, sh = best_lag_correlation(d["nav_roll"], d["att_roll"], max_shift)
    res["aileron"]["best_corr_navroll_to_attroll"] = r
    res["aileron"]["attroll_lag_behind_navroll_s"] = sh * dt

    res["elevator"] = {
        "corr_pitcherr_vs_de_act": pearson(d["pitch_err"], d["de_act"]),
        "corr_navpitch_vs_de_act": pearson(d["nav_pitch"], d["de_act"]),
        "corr_attpitch_vs_de_act": pearson(d["att_pitch"], d["de_act"]),
        "corr_de_act_vs_Cm": pearson(d["de_act"], d["Cm"]),
        "corr_de_act_vs_pitchspeed": pearson(d["de_act"], d["pitchspeed"]),
        "de_act_deg_rms": rms(d["de_act"]),
        "de_act_deg_mean": mean(d["de_act"]),
        "de_act_deg_max_abs": max(abs(x) for x in d["de_act"]),
    }
    r, sh = best_lag_correlation(d["de_act"], d["pitchspeed"], max_shift)
    res["elevator"]["best_corr_de_act_to_pitchspeed"] = r
    res["elevator"]["best_lag_de_act_to_pitchspeed_s"] = sh * dt
    r, sh = best_lag_correlation(d["nav_pitch"], d["att_pitch"], max_shift)
    res["elevator"]["best_corr_navpitch_to_attpitch"] = r
    res["elevator"]["attpitch_lag_behind_navpitch_s"] = sh * dt
    return res


def analyse_rudder(d, dt, deadband_deg):
    n = len(d["dr_act"])
    beyond = [abs(x) > deadband_deg for x in d["dr_act"]]
    res = {
        "deadband_deg": deadband_deg,
        "dr_act_deg_rms": rms(d["dr_act"]),
        "dr_act_deg_mean": mean(d["dr_act"]),
        "dr_act_deg_max_abs": max(abs(x) for x in d["dr_act"]),
        "dr_cmd_deg_rms": rms(d["dr_cmd"]),
        "dr_cmd_deg_max_abs": max(abs(x) for x in d["dr_cmd"]),
        "pct_time_beyond_deadband": 100.0 * sum(beyond) / n,
        "pct_of_full_travel_rms": 100.0 * rms(d["dr_act"]) / 45.0,
        # Which hypothesis does rudder motion follow?
        "corr_dr_act_vs_yawspeed": pearson(d["dr_act"], d["yawspeed"]),
        "corr_dr_act_vs_beta": pearson(d["dr_act"], d["beta"]),
        "corr_dr_act_vs_rollspeed": pearson(d["dr_act"], d["rollspeed"]),
        "corr_dr_act_vs_attroll": pearson(d["dr_act"], d["att_roll"]),
        "corr_dr_act_vs_da_act": pearson(d["dr_act"], d["da_act"]),
        "corr_dr_act_vs_Cn": pearson(d["dr_act"], d["Cn"]),
    }
    max_shift = max(1, int(round(2.0 / dt)))
    r, sh = best_lag_correlation(d["yawspeed"], d["dr_act"], max_shift)
    res["best_corr_yawspeed_to_dr_act"] = r
    res["dr_act_lag_behind_yawspeed_s"] = sh * dt
    return res


def analyse_actuator_lag(d, dt):
    """cmd_rad -> actual_angle_rad transport lag / first-order time constant.

    Three estimators, deliberately reported separately, never averaged:
      (1) cross-correlation lag - integer sample resolution, so it CANNOT
          resolve anything faster than one sample period (~53 ms here). It is
          reported to make that resolution floor explicit, not as an answer.
      (2) one-parameter fit  d(actual)/dt = (cmd - actual)/tau.
          KNOWN BIAS: this is contaminated by the servo model's documented
          steady-state droop (actuator_v1_config.yaml integral_derivation_note
          - a constant non-zero (cmd - actual) with d(actual)/dt ~ 0 drives
          tau -> infinity). Kept only so the bias is visible, not hidden.
      (3) two-parameter fit  d(actual)/dt = (1/tau)*(cmd - actual) + beta,
          which separates the constant droop (beta) from the lag (tau). This
          is the estimator to quote.

    SAMPLE-RATE CAVEAT (must be carried into any conclusion): the source
    records are ~19 Hz MAVLink-paced captures of a 1 kHz actuator loop. Any
    tau below ~2 sample periods (~0.11 s) is at or beyond this data's
    resolution and must be treated as an UPPER BOUND, not a measurement. A
    dedicated high-rate step test (Deliverable B1) is required to resolve it
    properly - DATA_REQUIRED for a sub-100 ms figure."""
    res = {}
    max_shift = max(1, int(round(0.5 / dt)))
    for s in SURFACES:
        cmd = d["_cmd"][s]
        act = d["_act"][s]
        n = len(cmd)
        err = [cmd[i] - act[i] for i in range(n)]
        dact = [(act[i + 1] - act[i]) / (d["t"][i + 1] - d["t"][i])
                for i in range(n - 1) if d["t"][i + 1] > d["t"][i]]
        m = min(len(err) - 1, len(dact))
        num = sum(err[i] * err[i] for i in range(m))
        den = sum(err[i] * dact[i] for i in range(m))
        tau1 = (num / den) if den != 0.0 else float("nan")
        # Two-parameter least squares: dact = alpha*err + beta
        me, md = mean(err[:m]), mean(dact[:m])
        sxx = sum((err[i] - me) ** 2 for i in range(m))
        sxy = sum((err[i] - me) * (dact[i] - md) for i in range(m))
        syy = sum((dact[i] - md) ** 2 for i in range(m))
        r2 = (sxy * sxy / (sxx * syy)) if (sxx > 0.0 and syy > 0.0) else float("nan")
        alpha = (sxy / sxx) if sxx > 0.0 else float("nan")
        beta = md - alpha * me if not math.isnan(alpha) else float("nan")
        tau2 = (1.0 / alpha) if (alpha and not math.isnan(alpha) and alpha != 0.0) \
            else float("nan")
        droop = (-beta / alpha) if (alpha and not math.isnan(alpha) and alpha != 0.0) \
            else float("nan")
        r, sh = best_lag_correlation(cmd, act, max_shift)
        res[s] = {
            "tracking_error_deg_rms": math.degrees(rms(err)),
            "tracking_error_deg_max_abs": math.degrees(max(abs(e) for e in err)),
            "first_order_tau_s_1param_BIASED": tau1,
            "first_order_tau_s_2param": tau2,
            "steady_state_droop_deg_2param": math.degrees(droop)
            if not math.isnan(droop) else None,
            "tau_fit_r2": r2,
            "tau_fit_reliable": bool(
                (not math.isnan(r2)) and r2 >= TAU_FIT_MIN_R2
                and math.degrees(rms(cmd)) >= TAU_FIT_MIN_CMD_RMS_DEG),
            "tau_at_or_below_sample_resolution":
                bool(not math.isnan(tau2) and tau2 < 2.0 * dt),
            "xcorr_best_r": r,
            "xcorr_lag_s": sh * dt,
            "xcorr_lag_samples": sh,
            "sample_period_s": dt,
            "cmd_deg_rms": math.degrees(rms(cmd)),
            "act_deg_rms": math.degrees(rms(act)),
        }
    return res


def find_reversals(d, key_act, key_cmd, min_amp_deg):
    """Locate sign reversals of the COMMANDED aero deflection and check the
    ACTUAL deflection reverses the same way, with the same sign."""
    events = []
    cmd, act, t = d[key_cmd], d[key_act], d["t"]
    n = len(cmd)
    i = 1
    while i < n:
        if cmd[i - 1] * cmd[i] < 0.0:
            # look back / forward for amplitude on both sides
            back = max(0, i - 60)
            fwd = min(n, i + 60)
            amp_before = max(abs(x) for x in cmd[back:i]) if i > back else 0.0
            amp_after = max(abs(x) for x in cmd[i:fwd]) if fwd > i else 0.0
            if amp_before >= min_amp_deg and amp_after >= min_amp_deg:
                # actual reversal index
                j = i
                while j < n and act[j] * act[i - 1] > 0.0:
                    j += 1
                events.append({
                    "t_cmd_reversal_s": t[i],
                    "t_act_reversal_s": t[j] if j < n else None,
                    "act_reversal_delay_s": (t[j] - t[i]) if j < n else None,
                    "cmd_before_deg": cmd[i - 1],
                    "cmd_after_deg": cmd[i],
                    "act_before_deg": act[i - 1],
                    "act_after_deg": act[min(j, n - 1)],
                    "cmd_amp_before_deg": amp_before,
                    "cmd_amp_after_deg": amp_after,
                    "actual_followed_command_sign":
                        (act[min(j, n - 1)] * cmd[i] > 0.0) if j < n else None,
                })
                i = fwd
                continue
        i += 1
    return events


def find_damping_dominant_events(d, min_amp_deg):
    """Find samples where the AILERON deflection has the OPPOSITE sign to the
    roll ATTITUDE ERROR (nav_roll - att_roll).

    Why this matters: ArduPlane's roll controller is error-proportional PLUS a
    rate-damping term. When the rate term dominates (aircraft rolling fast
    TOWARD the target), the commanded surface can oppose the remaining
    attitude error - i.e. to a human watching the GUI the surface appears to
    move 'backwards'. This function locates such samples WITHOUT asserting
    they are wrong; it reports them with the rate that explains them."""
    events = []
    n = len(d["t"])
    for i in range(n):
        e = d["roll_err"][i]
        da = d["da_act"][i]
        if abs(e) < min_amp_deg or abs(da) < min_amp_deg:
            continue
        # da_act > 0 commands roll RIGHT. A positive roll error (target right
        # of current) SHOULD, on the proportional term alone, command da > 0.
        if e * da < 0.0:
            events.append({
                "t_s": d["t"][i],
                "nav_roll_deg": d["nav_roll"][i],
                "att_roll_deg": d["att_roll"][i],
                "roll_err_deg": e,
                "rollspeed_deg_s": d["rollspeed"][i],
                "da_act_deg": da,
                "da_cmd_deg": d["da_cmd"][i],
                "Cl": d["Cl"][i],
                "explained_by_rate_term":
                    (d["rollspeed"][i] * e > 0.0),
            })
    return events


# One PWM microsecond, expressed in aero-differential degrees, for this exact
# transport: model.sdf ArduPilotPlugin <control channel="0"> has
# multiplier=+/-1.5707963268 rad and servo_min/servo_max = 800/2200, so
# 1 us -> 1.5707963268/1400 rad on each aileron joint; the differential
# delta_a = 0.5*(theta_right - theta_left) with opposite multipliers gives the
# same 1.5707963268/1400 rad. DERIVED, not assumed.
PWM_QUANTUM_DEG = math.degrees(1.5707963268 / 1400.0)
# Smallest roll attitude error worth classifying. ASSUMPTION (reporting only):
# ArduPlane reports nav_roll_deg quantised to 0.01 deg and att_roll_deg is
# EKF output; 0.3 deg keeps the classification clear of both.
ATT_ERR_MIN_DEG = 0.3


def find_unwind_episodes(d):
    """Find the VISIBLE form of the damping phenomenon: contiguous stretches
    where the roll attitude error keeps ONE sign (aircraft still needs to roll
    that way) while the commanded aileron differential moves MONOTONICALLY the
    other way by a large amount. This is what a human watching the GUI would
    describe as 'the aileron went back / went the wrong way while I was still
    short of the bank angle'.

    Reports the total travel of the command during the episode so the reader
    can judge visibility for themselves. Asserts nothing about correctness."""
    episodes = []
    n = len(d["t"])
    i = 1
    while i < n:
        e0 = d["roll_err"][i]
        if abs(e0) < ATT_ERR_MIN_DEG:
            i += 1
            continue
        sgn = 1.0 if e0 > 0 else -1.0
        j = i
        while j + 1 < n and d["roll_err"][j + 1] * sgn > 0.0:
            j += 1
        if j > i:
            # Largest BACK-OFF inside the run: the biggest drop from a running
            # extreme (in the direction the error is asking for) to a later
            # sample. Net start-to-end travel would miss the common case where
            # the command first builds up and then unwinds inside one run.
            best = None
            run_ext_idx = i
            for k in range(i, j + 1):
                if d["da_cmd"][k] * sgn > d["da_cmd"][run_ext_idx] * sgn:
                    run_ext_idx = k
                back = (d["da_cmd"][k] - d["da_cmd"][run_ext_idx]) * sgn
                if back < 0.0 and (best is None or back < best[0]):
                    best = (back, run_ext_idx, k)
            if best is not None:
                back, a, b = best
                episodes.append({
                    "t_error_run_start_s": round(d["t"][i], 3),
                    "t_error_run_end_s": round(d["t"][j], 3),
                    "t_cmd_peak_s": round(d["t"][a], 3),
                    "t_cmd_backoff_s": round(d["t"][b], 3),
                    "backoff_duration_s": round(d["t"][b] - d["t"][a], 3),
                    "roll_error_sign": "positive (needs RIGHT roll)"
                    if sgn > 0 else "negative (needs LEFT roll)",
                    "roll_error_at_cmd_peak_deg": round(d["roll_err"][a], 3),
                    "roll_error_at_backoff_deg": round(d["roll_err"][b], 3),
                    "cmd_delta_a_peak_deg": round(d["da_cmd"][a], 4),
                    "cmd_delta_a_after_backoff_deg": round(d["da_cmd"][b], 4),
                    "backoff_magnitude_deg": round(-back, 4),
                    "roll_rate_at_cmd_peak_deg_s": round(d["rollspeed"][a], 3),
                    "roll_rate_at_backoff_deg_s": round(d["rollspeed"][b], 3),
                    "actual_delta_a_peak_deg": round(d["da_act"][a], 4),
                    "actual_delta_a_after_backoff_deg": round(d["da_act"][b], 4),
                })
        i = j + 1
    episodes.sort(key=lambda e: -e["backoff_magnitude_deg"])
    return episodes[:10]


def analyse_roll_controller_structure(d, rll2srv_tconst, min_amp_deg):
    """Answer, with evidence, the 'does the damping term make the surface look
    like it is moving BACKWARDS?' question.

    ArduPlane's AP_RollController is NOT an attitude-proportional controller.
    It is a cascade:
        desired_roll_rate = (nav_roll - att_roll) / RLL2SRV_TCONST
        surface_demand    = ratePID(desired_roll_rate - measured_roll_rate)
    So the surface follows the RATE error, not the ATTITUDE error. Whenever the
    aircraft is already rolling toward the target FASTER than the demanded
    rate, the rate error flips sign while the attitude error has NOT yet
    reached zero - and the aileron correctly commands the opposite way. To a
    human watching the GUI that reads as 'the surface moved backwards'.

    RLL2SRV_TCONST is READ FROM THE LIVE PARAM CAPTURE of the same run, never
    hard-coded here. This function CHANGES NOTHING - it only classifies
    samples that already exist in the record."""
    del min_amp_deg  # superseded by the PWM-quantum threshold below
    if rll2srv_tconst is None or rll2srv_tconst <= 0.0:
        return {"data_required": "RLL2SRV_TCONST not present in params_live; "
                                 "cannot reconstruct the demanded roll rate"}
    n = len(d["t"])
    opposes_attitude_error = []
    opposes_rate_error = []
    min_amp = PWM_QUANTUM_DEG
    for i in range(n):
        e_att = d["roll_err"][i]
        p_des = e_att / rll2srv_tconst
        e_rate = p_des - d["rollspeed"][i]
        # COMMAND domain: what ArduPlane actually asked for, free of the
        # servo's own tracking lag. This is the domain the controller-structure
        # question is about.
        da = d["da_cmd"][i]
        if abs(da) < min_amp or abs(e_att) < ATT_ERR_MIN_DEG:
            continue
        if e_att * da < 0.0:
            rec = {
                "t_s": round(d["t"][i], 3),
                "nav_roll_deg": round(d["nav_roll"][i], 3),
                "att_roll_deg": round(d["att_roll"][i], 3),
                "roll_attitude_error_deg": round(e_att, 4),
                "demanded_roll_rate_deg_s": round(p_des, 4),
                "measured_roll_rate_deg_s": round(d["rollspeed"][i], 4),
                "roll_rate_error_deg_s": round(e_rate, 4),
                "cmd_delta_a_deg": round(da, 4),
                "actual_delta_a_deg": round(d["da_act"][i], 4),
                "Cl": d["Cl"][i],
                "surface_agrees_with_RATE_error": bool(e_rate * da > 0.0),
            }
            opposes_attitude_error.append(rec)
        if e_rate * da < 0.0:
            opposes_rate_error.append(round(d["t"][i], 3))
    tot = sum(1 for i in range(n)
              if abs(d["da_cmd"][i]) >= min_amp
              and abs(d["roll_err"][i]) >= ATT_ERR_MIN_DEG)
    explained = sum(1 for r in opposes_attitude_error
                    if r["surface_agrees_with_RATE_error"])
    return {
        "rll2srv_tconst_from_params_live": rll2srv_tconst,
        "domain": "COMMAND (da_cmd), not the lagged actual angle",
        "min_amplitude_deg": min_amp,
        "min_attitude_error_deg": ATT_ERR_MIN_DEG,
        "unwind_episodes": find_unwind_episodes(d),
        "n_samples_with_deflection_above_min_amp": tot,
        "n_surface_opposes_ATTITUDE_error": len(opposes_attitude_error),
        "pct_surface_opposes_ATTITUDE_error":
            (100.0 * len(opposes_attitude_error) / tot) if tot else None,
        "n_of_those_explained_by_RATE_error": explained,
        "pct_of_those_explained_by_RATE_error":
            (100.0 * explained / len(opposes_attitude_error))
            if opposes_attitude_error else None,
        "n_surface_opposes_RATE_error": len(opposes_rate_error),
        "pct_surface_opposes_RATE_error":
            (100.0 * len(opposes_rate_error) / tot) if tot else None,
        "examples_surface_opposes_attitude_error":
            opposes_attitude_error[:25],
    }


def analyse_rudder_aileron_mix_domain(x, y):
    n = len(x)
    mx, my = mean(x), mean(y)
    sxx = sum((x[i] - mx) ** 2 for i in range(n))
    sxy = sum((x[i] - mx) * (y[i] - my) for i in range(n))
    syy = sum((y[i] - my) ** 2 for i in range(n))
    k = (sxy / sxx) if sxx > 0 else float("nan")
    c = my - k * mx if not math.isnan(k) else float("nan")
    r2 = (sxy * sxy / (sxx * syy)) if (sxx > 0 and syy > 0) else float("nan")
    resid = [y[i] - (k * x[i] + c) for i in range(n)] if not math.isnan(k) else []
    return {"slope_dr_per_da": k, "intercept_deg": c, "r_squared": r2,
            "residual_deg_rms": rms(resid) if resid else float("nan")}


def analyse_rudder_aileron_mix(d):
    """Quantify how much of the rudder motion is simply ArduPlane's
    KFF_RDDRMIX roll-to-rudder feedforward, i.e. a scaled copy of the aileron
    demand, rather than an independent yaw-controller action.

    Least squares dr_act = k * da_act + c. Under the documented
    delta-mapping, a pure KFF_RDDRMIX mix with both SERVO1 and SERVO4
    REVERSED=1 and identical MIN/MAX/TRIM (falcon_v2_sitl.parm) predicts
    k = KFF_RDDRMIX exactly. Reported, never asserted."""
    cmd = analyse_rudder_aileron_mix_domain(d["da_cmd"], d["dr_cmd"])
    x, y = d["da_act"], d["dr_act"]
    n = len(x)
    mx, my = mean(x), mean(y)
    sxx = sum((x[i] - mx) ** 2 for i in range(n))
    sxy = sum((x[i] - mx) * (y[i] - my) for i in range(n))
    syy = sum((y[i] - my) ** 2 for i in range(n))
    k = (sxy / sxx) if sxx > 0 else float("nan")
    c = my - k * mx if not math.isnan(k) else float("nan")
    r2 = (sxy * sxy / (sxx * syy)) if (sxx > 0 and syy > 0) else float("nan")
    resid = [y[i] - (k * x[i] + c) for i in range(n)] \
        if not math.isnan(k) else []
    return {
        "slope_dr_per_da": k,
        "intercept_deg": c,
        "r_squared": r2,
        "residual_deg_rms": rms(resid) if resid else float("nan"),
        "residual_deg_max_abs": max((abs(v) for v in resid), default=float("nan")),
        "command_domain_fit": cmd,
        "interpretation_note":
            "slope ~= KFF_RDDRMIX and r_squared ~= 1 would mean the rudder is "
            "almost entirely the roll-to-rudder feedforward (turn "
            "coordination), with almost nothing left over for an independent "
            "yaw-damper/sideslip action.",
    }


def event_row(d, i, label, scenario, segment):
    return {
        "label": label,
        "scenario": scenario,
        "segment": segment,
        "t_s": round(d["t"][i], 3),
        "target_nav_roll_deg": round(d["nav_roll"][i], 3),
        "target_nav_pitch_deg": round(d["nav_pitch"][i], 3),
        "servo_raw_ail_pwm": d["servo_ail"][i],
        "servo_raw_ele_pwm": d["servo_ele"][i],
        "servo_raw_rud_pwm": d["servo_rud"][i],
        "cmd_delta_a_deg": round(d["da_cmd"][i], 4),
        "cmd_delta_e_deg": round(d["de_cmd"][i], 4),
        "cmd_delta_r_deg": round(d["dr_cmd"][i], 4),
        "actual_delta_a_deg": round(d["da_act"][i], 4),
        "actual_delta_e_deg": round(d["de_act"][i], 4),
        "actual_delta_r_deg": round(d["dr_act"][i], 4),
        "att_roll_deg": round(d["att_roll"][i], 3),
        "att_pitch_deg": round(d["att_pitch"][i], 3),
        "rollspeed_deg_s": round(d["rollspeed"][i], 3),
        "pitchspeed_deg_s": round(d["pitchspeed"][i], 3),
        "yawspeed_deg_s": round(d["yawspeed"][i], 3),
        "Cl": d["Cl"][i], "Cm": d["Cm"][i], "Cn": d["Cn"][i],
    }


def pick_events(d, scenario, segment):
    """Pick the concrete timestamps the stage asked for."""
    rows = []
    n = len(d["t"])
    ro = THRESHOLDS["roll_demand_onset_deg"]["value"]
    po = THRESHOLDS["pitch_demand_onset_deg"]["value"]

    # roll input onset: first index where |nav_roll| first crosses the onset
    for i in range(1, n):
        if abs(d["nav_roll"][i]) >= ro and abs(d["nav_roll"][i - 1]) < ro:
            rows.append(event_row(d, i, "roll_input_onset", scenario, segment))
            break
    # approaching roll target: |roll_err| minimal while |nav_roll| still large
    cand = [i for i in range(n) if abs(d["nav_roll"][i]) >= ro]
    if cand:
        i = min(cand, key=lambda k: abs(d["roll_err"][k]))
        rows.append(event_row(d, i, "approaching_roll_target", scenario, segment))
        # roll overshoot + damping: max |att_roll| - |nav_roll| beyond target
        j = max(cand, key=lambda k: abs(d["att_roll"][k]) - abs(d["nav_roll"][k]))
        rows.append(event_row(d, j, "roll_overshoot_and_damping", scenario, segment))
        # turn entry / exit from the sustained-demand block
        rows.append(event_row(d, cand[0], "turn_entry", scenario, segment))
        rows.append(event_row(d, cand[-1], "turn_exit", scenario, segment))
    # pitch climb onset: largest positive nav_pitch
    i = max(range(n), key=lambda k: d["nav_pitch"][k])
    if d["nav_pitch"][i] >= po:
        rows.append(event_row(d, i, "pitch_climb_onset", scenario, segment))
    # pitch level-off: first return of nav_pitch below onset after that peak
    for k in range(i, n):
        if d["nav_pitch"][k] < po:
            rows.append(event_row(d, k, "pitch_level_off", scenario, segment))
            break
    return rows


def analyse_segment(all_samples, signs, scenario, segment, params_live=None):
    samples = [s for s in all_samples if usable(s)]
    n_dropped = len(all_samples) - len(samples)
    if len(samples) < 20:
        return {"scenario": scenario, "segment": segment,
                "data_required": "fewer than 20 usable samples "
                                 f"({len(samples)} of {len(all_samples)})"}
    d = extract(samples, signs)
    dt = sample_period(d["t"])
    out = {
        "scenario": scenario,
        "segment": segment,
        "n_samples": len(samples),
        "n_samples_dropped_missing_nav_fields": n_dropped,
        "t_start_s": d["t"][0],
        "t_end_s": d["t"][-1],
        "sample_period_s": dt,
        "sample_rate_hz": (1.0 / dt) if dt and dt > 0 else float("nan"),
        "tracking": analyse_tracking(d, dt),
        "rudder": analyse_rudder(d, dt, THRESHOLDS["rudder_deadband_deg"]["value"]),
        "actuator_lag": analyse_actuator_lag(d, dt),
        "aileron_reversals": find_reversals(
            d, "da_act", "da_cmd", THRESHOLDS["reversal_min_amplitude_deg"]["value"]),
        "elevator_reversals": find_reversals(
            d, "de_act", "de_cmd", THRESHOLDS["reversal_min_amplitude_deg"]["value"]),
        "damping_dominant_aileron_events":
            find_damping_dominant_events(
                d, THRESHOLDS["reversal_min_amplitude_deg"]["value"]),
        "event_table": pick_events(d, scenario, segment),
        "roll_controller_structure": analyse_roll_controller_structure(
            d, (params_live or {}).get("RLL2SRV_TCONST"),
            THRESHOLDS["reversal_min_amplitude_deg"]["value"]),
        "rudder_aileron_mix": analyse_rudder_aileron_mix(d),
    }
    ev = out["damping_dominant_aileron_events"]
    out["damping_dominant_summary"] = {
        "count": len(ev),
        "pct_of_samples": 100.0 * len(ev) / len(samples),
        "pct_explained_by_rate_term":
            (100.0 * sum(1 for e in ev if e["explained_by_rate_term"]) / len(ev))
            if ev else None,
    }
    return out


def main():
    signs = read_sign_scalars()
    report = {
        "stage": "MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION",
        "part": "3 - offline closed-loop control-surface behaviour analysis",
        "classification": "TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY / OFFLINE",
        "owner": "controls-integration",
        "physics_parameters_changed": "NONE",
        "simulation_runs_performed": 0,
        "sign_convention_source": [
            "docs/source_of_truth/controls/CONTROLS.md sec 10",
            "docs/test_results/2026-08-22_control_surface_sign_mapping_test_report.md",
            "docs/source_of_truth/aerodynamics/aero_v1_config.yaml control_mapping",
        ],
        "sign_scalars_used": signs,
        "thresholds": THRESHOLDS,
        "segments": [],
    }
    sources = []
    for sc in ("A", "B", "C", "D"):
        path = os.path.join(RESULTS_DIR, f"ardupilot_navigation_scenario_{sc}_timeseries.json")
        if not os.path.exists(path):
            report.setdefault("data_required", []).append(f"missing input: {path}")
            continue
        sources.append(path)
        data = load_json(path)
        for segname, seg in data["segments"].items():
            report["segments"].append(
                analyse_segment(seg["samples"], signs, sc, segname,
                                data.get("params_live")))
    report["input_files"] = sources
    with open(OUT_JSON, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    print(f"wrote {OUT_JSON}")
    for s in report["segments"]:
        print(f"  {s['scenario']}/{s['segment']:20s} n={s['n_samples']:5d} "
              f"{s['sample_rate_hz']:.1f}Hz "
              f"ail_r={s['tracking']['aileron']['corr_rollerr_vs_da_act']:+.3f} "
              f"ele_r={s['tracking']['elevator']['corr_pitcherr_vs_de_act']:+.3f} "
              f"rud_rms={s['rudder']['dr_act_deg_rms']:.3f}deg "
              f"damp_ev={s['damping_dominant_summary']['count']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

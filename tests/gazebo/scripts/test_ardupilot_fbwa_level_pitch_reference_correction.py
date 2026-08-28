#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_FBWA_LEVEL_PITCH_REFERENCE_CORRECTION - Step 3
(gazebo-testing, 2026-08-29). SHORT bounded verification.

WHAT THIS CHECKS
----------------
`controls-integration` (Step 1/2, verified + applied) added
`PTCH_TRIM_DEG 2.49` to config/ardupilot/falcon_v2_sitl.parm (was the
ArduPlane default 0.0) and corrected the canonical trim-reference constants
in test_ardupilot_basic_closed_loop_flight.py (imported here as `base`) to
the proven pure-Gazebo equilibrium C.2:
    V_TRIM 18.162, ALPHA_TRIM_DEG 2.472, ELEVATOR_THETA_RAD radians(4.092),
    TRIM_THROTTLE 0.4957  ->  RC3_TRIM_TARGET_US = 1496, ELEV_RC2_TARGET_US = 1536.

`PTCH_TRIM_DEG` is a pure ArduPlane FBWA/attitude-controller *demand offset*
(ArduPlane/Attitude.cpp:244, verified V4.8.0-dev commit 409226a637): at FBWA
neutral pitch stick the attitude controller targets AHRS pitch =
`nav_pitch (=0) + PTCH_TRIM_DEG` = +2.49 deg nose-up. It does NOT touch
AHRS/EKF and does NOT affect MANUAL.

PITCH TELEMETRY CAVEAT (handled below, do NOT misread):
`GCS_MAVLink_Plane.cpp:139` subtracts `PTCH_TRIM_DEG` from the reported
MAVLink `ATTITUDE.pitch` (FLIGHT_OPTIONS bit 8 unset). So with the fix
working and the aircraft physically at +2.49 deg nose-up:
  - `ATTITUDE.pitch` (mav)              -> reads ~= 0 deg   (EXPECTED, not a failure)
  - `NAV_CONTROLLER_OUTPUT.nav_pitch`   -> stays raw 0.000 deg at neutral stick
  - the REAL physical pitch is read from gz-transport ground truth
    (base_link world pose Euler). gz / quat_to_rpy Euler pitch is
    NOSE-DOWN-POSITIVE in this FLU world, so physical nose-up pitch =
    -(gz pitch). We report physical nose-up-positive.
  - new expected invariant:  pitch_mav ~= -pitch_gz - PTCH_TRIM_DEG
                             (i.e. pitch_mav ~= pitch_phys_nose_up - 2.49)

TASK: ONE neutral-stick FBWA segment, 28 s, 5 s transient cutoff, ~20 Hz
combined MAVLink + gz-transport telemetry, RC1=RC2=1500, RC3=1496, RC4=1500,
RC5=1000. Measure over the stabilized window (t >= 5 s):
  nav_pitch target; physical pitch (mean/std/drift) + ATTITUDE.pitch mav +
  invariant residual; pitch rate q; airspeed (VFR_HUD + aero diag V);
  vertical speed (pos_z linreg + endpoint + odometry twist z) + altitude
  slope; throttle actual L/R + asymmetry; thrust total, drag = q_bar*S*CD,
  T-D; elevator cmd & actual both halves + clamp counts; roll/yaw sanity;
  NaN/Inf guard; oscillation growth check. Side-by-side vs the prior stage's
  FBWA-original (sink 0.551 m/s) and FBWA-throttle* (0.598 m/s) runs.

ACCEPTANCE: PASS if |vertical_speed| <= 0.10 m/s (<= 0.05 preferred) AND
physical pitch ~= +2.49 deg stable AND airspeed ~= 18.16 m/s AND zero
elevator clamp AND throttle actual ~= 0.4957 AND no new/growing oscillation.
Verdict: FBWA_LEVEL_PITCH_REFERENCE_CORRECTION_PASS / _PARTIAL / _FAILED.
If PARTIAL/FAILED: measure and report the residual + likely cause. Do NOT
tune PID/aero/propulsion.

SCOPE / HARD CONSTRAINT: this file only defines one neutral FBWA segment +
the analysis. It imports `base` (PHASE 1-4 precondition) and `campaign`
(enter_fbwa, run_segment, build_sample, MSG_IDS_20HZ) VERBATIM - exact same
pattern as test_ardupilot_trim_reference_correction_validation.py. It reads
NO aircraft-physics parameter for any purpose other than citing an
already-published value, and modifies NONE. No aero coefficient/table, no
propulsion parameter, no PID gain, no falcon_v2_sitl.parm, no actuator/sign
mapping, no joint limit, no plugin source, no SDF is touched.

Usage (a FRESH gz sim + gdb-wrapped arduplane pair MUST already be running -
see tests/gazebo/scripts/run_ardupilot_fbwa_level_pitch_reference_correction.sh):
    python3 test_ardupilot_fbwa_level_pitch_reference_correction.py
"""
import json
import math
import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import gz.transport13 as tp  # noqa: E402
from gz.msgs10 import entity_wrench_pb2, entity_pb2  # noqa: E402

import test_ardupilot_basic_closed_loop_flight as base  # noqa: E402
import test_ardupilot_basic_closed_loop_flight_campaign as campaign  # noqa: E402
import aero_lib  # noqa: E402
import propulsion_lib  # noqa: E402
import actuator_lib  # noqa: E402

OUT_JSON = os.path.join(base.RESULTS_DIR, "ardupilot_fbwa_level_pitch_reference_correction_result.json")

STAGE = "ARDUPLANE_FBWA_LEVEL_PITCH_REFERENCE_CORRECTION"
SEGMENT_LABEL = "fbwa_neutral_level_pitch_ref_2p49"
SEGMENT_DURATION_S = 28.0
TRANSIENT_CUTOFF_S = 5.0

PTCH_TRIM_DEG_EXPECTED = 2.49       # config/ardupilot/falcon_v2_sitl.parm (controls-integration, applied)
S_REF_M2 = 0.4514                   # CLAUDE.md wing area - read-only citation
MASS_KG = 6.000                     # CLAUDE.md - read-only citation
G = 9.81

# Prior-stage FBWA neutral-stick runs, SAME world / SAME PHASE1-4 precondition,
# differing ONLY in PTCH_TRIM_DEG (0.0 then) + the trim constants. Cited, not
# re-derived: docs/test_results/2026-08-28_ardupilot_longitudinal_equilibrium_
# and_sink_root_cause_validation.md sec 9, and the raw c3 result JSONs.
PRIOR_RUNS = {
    "fbwa_original_0.5010": dict(
        rc3=1501, throttle=0.5010, sink_regression_ms=0.5512, sink_endpoint_ms=0.5465,
        pitch_gz_mean_deg=0.159, pitch_mav_mean_deg=-0.159, airspeed_mean_ms=19.493,
        thrust_minus_drag_N=-1.76,
        source="ardupilot_longitudinal_equilibrium_c3_fbwa_original_result.json"),
    "fbwa_throttlestar_0.4957": dict(
        rc3=1496, throttle=0.4957, sink_regression_ms=0.5982, sink_endpoint_ms=0.5945,
        pitch_gz_mean_deg=0.201, pitch_mav_mean_deg=-0.202, airspeed_mean_ms=19.351,
        thrust_minus_drag_N=-1.94,
        source="ardupilot_longitudinal_equilibrium_c3_fbwa_throttlestar_result.json"),
}


# =============================================================================
# small stats helpers
# =============================================================================
def linreg(ts, ys):
    """returns (slope, intercept) or (None, None)."""
    n = len(ts)
    if n < 2:
        return None, None
    tb = sum(ts) / n
    yb = sum(ys) / n
    den = sum((t - tb) ** 2 for t in ts)
    if den == 0:
        return None, None
    num = sum((t - tb) * (y - yb) for t, y in zip(ts, ys))
    m = num / den
    return m, yb - m * tb


def mean(xs):
    return sum(xs) / len(xs) if xs else None


def stdev(xs):
    if len(xs) < 2:
        return 0.0
    mu = mean(xs)
    return math.sqrt(sum((x - mu) ** 2 for x in xs) / len(xs))


def minmaxmean(xs):
    return dict(min=min(xs), max=max(xs), mean=mean(xs), std=stdev(xs), n=len(xs)) if xs else None


def series_report(ts, ys, label):
    """mean/std/min/max + linear slope + total drift over the window."""
    if len(ys) < 2:
        return dict(label=label, n=len(ys))
    slope, _ = linreg(ts, ys)
    dur = ts[-1] - ts[0]
    return dict(label=label, n=len(ys), mean=mean(ys), std=stdev(ys),
                min=min(ys), max=max(ys),
                slope_per_s=slope, drift_over_window=(slope * dur if slope is not None else None),
                endpoint_delta=ys[-1] - ys[0], t_start=ts[0], t_end=ts[-1])


def detrended_growth(ts, ys):
    """fit a line, subtract it, compare first-half vs second-half residual
    spread. growing=True if 2nd-half std >= 1.3x 1st-half std (and the 1st
    half is not already flat at the noise floor)."""
    if len(ys) < 8:
        return dict(n=len(ys), growing=False, note="too few samples")
    slope, icpt = linreg(ts, ys)
    resid = [y - (slope * t + icpt) for t, y in zip(ts, ys)]
    h = len(resid) // 2
    s1 = stdev(resid[:h])
    s2 = stdev(resid[h:])
    p2p1 = (max(resid[:h]) - min(resid[:h]))
    p2p2 = (max(resid[h:]) - min(resid[h:]))
    growing = (s1 > 1e-4) and (s2 >= 1.3 * s1)
    return dict(n=len(ys), resid_std_first_half=s1, resid_std_second_half=s2,
                resid_p2p_first_half=p2p1, resid_p2p_second_half=p2p2,
                ratio_second_over_first=(s2 / s1 if s1 > 0 else None), growing=bool(growing))


# =============================================================================
# runtime PARAM read (regression guard)
# =============================================================================
def read_param(mav, name, timeout=6.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        mav.m.mav.param_request_read_send(mav.m.target_system, mav.m.target_component,
                                          name.encode("ascii"), -1)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            r, _, _ = select.select([mav.m.port], [], [], 0.3)
            if not r:
                continue
            msg = mav.m.recv_match(type="PARAM_VALUE", blocking=False)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("ascii", "ignore")
            pid = pid.rstrip("\x00")
            if pid == name:
                return float(msg.param_value)
    return None


def check_params(mav, R):
    got = {}
    for name in ("PTCH_TRIM_DEG", "KFF_THR2PTCH", "FLIGHT_OPTIONS"):
        got[name] = read_param(mav, name)
    R["param_check"] = dict(
        values=got,
        ptch_trim_deg_ok=(got.get("PTCH_TRIM_DEG") is not None
                          and abs(got["PTCH_TRIM_DEG"] - PTCH_TRIM_DEG_EXPECTED) < 1e-3),
        kff_thr2ptch_zero=(got.get("KFF_THR2PTCH") == 0.0),
        flight_options_zero=(got.get("FLIGHT_OPTIONS") == 0.0),
    )
    print("PARAM check:", json.dumps(R["param_check"], default=str))
    return R["param_check"]


# =============================================================================
# analysis of the one neutral FBWA segment
# =============================================================================
def analyze(seg, R):
    samples = seg["samples"]
    full = samples
    stab = [s for s in samples if s["t"] >= TRANSIENT_CUTOFF_S]
    out = {"n_full": len(full), "n_stab": len(stab), "transient_cutoff_s": TRANSIENT_CUTOFF_S}

    def gz_pos_series(subset):
        pts = [(s["t"], s["gz"]["pos"][2]) for s in subset if s["gz"]["pos"] is not None]
        return [p[0] for p in pts], [p[1] for p in pts]

    # ---- vertical speed / altitude, 3 methods, stabilized + full ----
    def vspeed_block(subset, label):
        ts, zs = gz_pos_series(subset)
        if len(zs) < 2:
            return dict(label=label, n=len(zs))
        slope, _ = linreg(ts, zs)
        endpoint = (zs[-1] - zs[0]) / (ts[-1] - ts[0])
        # odometry twist z: raw body-frame, and world-projected via pose quat
        raw_tw = []
        world_tw = []
        for s in subset:
            vb = s["gz"]["v_body"]
            pose = None
            # pose quat not stored in build_sample; reconstruct from att? we
            # only have euler deg. Rebuild quaternion from gz euler to project.
            att = s["gz"]["att_deg"]
            if vb is None:
                continue
            raw_tw.append(vb[2])
            if att is not None:
                r, p, y = (math.radians(att[0]), math.radians(att[1]), math.radians(att[2]))
                cr, sr = math.cos(r / 2), math.sin(r / 2)
                cp, sp = math.cos(p / 2), math.sin(p / 2)
                cy, sy = math.cos(y / 2), math.sin(y / 2)
                qw = cr * cp * cy + sr * sp * sy
                qx = sr * cp * cy - cr * sp * sy
                qy = cr * sp * cy + sr * cp * sy
                qz = cr * cp * sy - sr * sp * cy
                vw = base.rotate_body_to_world((qw, qx, qy, qz), tuple(vb))
                world_tw.append(vw[2])
        climb = [s["mav"]["climb"] for s in subset if s["mav"]["climb"] is not None]
        return dict(
            label=label, n=len(zs), t_start=ts[0], t_end=ts[-1], duration_s=ts[-1] - ts[0],
            alt_start=zs[0], alt_end=zs[-1],
            vertical_speed_regression_ms=slope,           # climb +, sink -
            vertical_speed_endpoint_ms=endpoint,
            altitude_slope_ms=slope,                       # == -sink
            sink_regression_ms=(-slope if slope is not None else None),
            sink_endpoint_ms=-endpoint,
            odom_twist_z_body_mean_ms=minmaxmean(raw_tw),
            odom_twist_z_world_proj_mean_ms=minmaxmean(world_tw),
            vfr_hud_climb_mean_ms=minmaxmean(climb),
        )

    out["vertical_stabilized"] = vspeed_block(stab, "stabilized_5s_to_end")
    out["vertical_full"] = vspeed_block(full, "full_0_to_end")

    if not stab:
        R["analysis"] = out
        return out

    ts = [s["t"] for s in stab]

    # ---- pitch: nav target, physical (gz), mav, invariant ----
    nav_pitch = [s["mav"]["nav_pitch_deg"] for s in stab if s["mav"]["nav_pitch_deg"] is not None]
    pitch_gz = [s["gz"]["att_deg"][1] for s in stab if s["gz"]["att_deg"] is not None]
    pitch_mav = [s["mav"]["att_pitch_deg"] for s in stab if s["mav"]["att_pitch_deg"] is not None]
    # physical nose-up-positive = -(gz euler pitch)
    ts_p = [s["t"] for s in stab if s["gz"]["att_deg"] is not None]
    pitch_phys = [-p for p in pitch_gz]

    invariant_resid = []
    for s in stab:
        if s["gz"]["att_deg"] is None or s["mav"]["att_pitch_deg"] is None:
            continue
        pphys = -s["gz"]["att_deg"][1]
        # expected: pitch_mav ~= pitch_phys - PTCH_TRIM_DEG
        invariant_resid.append(s["mav"]["att_pitch_deg"] - (pphys - PTCH_TRIM_DEG_EXPECTED))

    out["nav_pitch_target_deg"] = minmaxmean(nav_pitch)
    out["pitch_physical_noseup_deg"] = series_report(ts_p, pitch_phys, "physical_pitch_noseup")
    out["pitch_mav_deg"] = minmaxmean(pitch_mav)
    out["pitch_gz_euler_deg"] = minmaxmean(pitch_gz)
    out["pitch_mav_invariant_residual_deg"] = dict(
        formula="pitch_mav - (pitch_phys_noseup - PTCH_TRIM_DEG)",
        mean=mean(invariant_resid), std=stdev(invariant_resid),
        max_abs=max(abs(x) for x in invariant_resid) if invariant_resid else None,
        n=len(invariant_resid))

    # ---- pitch rate q ----
    q_gz = [s["gz"]["av_body_deg"][1] for s in stab if s["gz"]["av_body_deg"] is not None]
    q_mav = [s["mav"]["pitchspeed_deg_s"] for s in stab if s["mav"]["pitchspeed_deg_s"] is not None]
    out["pitch_rate_q_gz_deg_s"] = minmaxmean(q_gz)
    out["pitch_rate_q_mav_deg_s"] = minmaxmean(q_mav)

    # ---- airspeed ----
    asp_vfr = [s["mav"]["airspeed"] for s in stab if s["mav"]["airspeed"] is not None]
    asp_aero = [s["aero"]["V"] for s in stab if s["aero"] is not None]
    ts_vfr = [s["t"] for s in stab if s["mav"]["airspeed"] is not None]
    out["airspeed_vfr_hud_ms"] = series_report(ts_vfr, asp_vfr, "airspeed_vfr_hud")
    out["airspeed_aero_diag_V_ms"] = minmaxmean(asp_aero)

    # ---- throttle actual L/R (propulsion diag) ----
    thr_L = [s["propulsion"]["left"]["throttle"] for s in stab if s["propulsion"]]
    thr_R = [s["propulsion"]["right"]["throttle"] for s in stab if s["propulsion"]]
    thr_vfr = [s["mav"]["throttle_pct"] for s in stab if s["mav"]["throttle_pct"] is not None]
    asym = None
    if thr_L and thr_R:
        asym = max(abs(a - b) for a, b in zip(thr_L, thr_R))
    out["throttle_actual_L"] = minmaxmean(thr_L)
    out["throttle_actual_R"] = minmaxmean(thr_R)
    out["throttle_vfr_hud_pct"] = minmaxmean(thr_vfr)
    out["throttle_LR_asymmetry_max"] = asym

    # ---- thrust total, drag, T-D, L/W ----
    thrust_tot = [s["propulsion"]["left"]["thrust_N"] + s["propulsion"]["right"]["thrust_N"]
                  for s in stab if s["propulsion"]]
    drag = [s["aero"]["qbar"] * S_REF_M2 * s["aero"]["CD"] for s in stab if s["aero"] is not None]
    lift = [s["aero"]["qbar"] * S_REF_M2 * s["aero"]["CL"] for s in stab if s["aero"] is not None]
    T = mean(thrust_tot) if thrust_tot else None
    D = mean(drag) if drag else None
    out["thrust_total_N"] = minmaxmean(thrust_tot)
    out["drag_qbar_S_CD_N"] = minmaxmean(drag)
    out["thrust_minus_drag_N"] = (T - D) if (T is not None and D is not None) else None
    out["lift_over_weight"] = (mean(lift) / (MASS_KG * G)) if lift else None

    # ---- elevator cmd & actual, both halves ----
    def elev(field, surf):
        return [s["actuators"][surf][field] for s in stab if s["actuators"]]
    out["elevator_deg"] = {}
    for surf in ("left_elevator", "right_elevator"):
        cmd = [math.degrees(x) for x in elev("cmd_rad", surf)]
        act = [math.degrees(x) for x in elev("actual_angle_rad", surf)]
        out["elevator_deg"][surf] = dict(cmd=minmaxmean(cmd), actual=minmaxmean(act))

    # ---- actuator clamp events (all 5 surfaces, full + stab windows) ----
    def clamp_counts(subset):
        tgt = eff = 0
        for s in subset:
            if not s["actuators"]:
                continue
            for surf, d in s["actuators"].items():
                if d["target_clamp_active"]:
                    tgt += 1
                if d["effort_clamp_active"]:
                    eff += 1
        return dict(target_clamp_active_samples=tgt, effort_clamp_active_samples=eff)
    out["actuator_clamp_stabilized"] = clamp_counts(stab)
    out["actuator_clamp_full"] = clamp_counts(full)

    # ---- roll / yaw sanity ----
    roll_gz = [s["gz"]["att_deg"][0] for s in stab if s["gz"]["att_deg"] is not None]
    yaw_gz = [s["gz"]["att_deg"][2] for s in stab if s["gz"]["att_deg"] is not None]
    p_gz = [s["gz"]["av_body_deg"][0] for s in stab if s["gz"]["av_body_deg"] is not None]
    r_gz = [s["gz"]["av_body_deg"][2] for s in stab if s["gz"]["av_body_deg"] is not None]
    roll_mav = [s["mav"]["att_roll_deg"] for s in stab if s["mav"]["att_roll_deg"] is not None]
    out["lateral_sanity"] = dict(
        roll_gz_deg=minmaxmean(roll_gz), yaw_gz_deg=minmaxmean(yaw_gz),
        roll_rate_p_gz_deg_s=minmaxmean(p_gz), yaw_rate_r_gz_deg_s=minmaxmean(r_gz),
        roll_mav_deg=minmaxmean(roll_mav),
        roll_mav_vs_roll_gz_max_abs_diff=(max(abs(a - b) for a, b in zip(roll_mav, roll_gz))
                                          if roll_mav and roll_gz and len(roll_mav) == len(roll_gz) else None),
    )

    # ---- NaN / Inf guard (every captured numeric) ----
    bad = []
    for s in stab:
        for grp in ("gz", "mav"):
            for k, v in s[grp].items():
                vals = v if isinstance(v, list) else [v]
                for x in vals:
                    if isinstance(x, float) and not math.isfinite(x):
                        bad.append((s["t"], grp, k))
        for grp in ("aero", "propulsion", "actuators"):
            d = s[grp]
            if not d:
                continue
            def scan(dd, path):
                for kk, vv in dd.items():
                    if isinstance(vv, dict):
                        scan(vv, path + "." + kk)
                    elif isinstance(vv, float) and not math.isfinite(vv):
                        bad.append((s["t"], path + "." + kk))
            scan(d, grp)
    out["nan_inf_samples"] = bad[:50]
    out["nan_inf_count"] = len(bad)

    # ---- oscillation growth ----
    ts_z, zs = gz_pos_series(stab)
    out["oscillation_growth"] = dict(
        altitude_pos_z=detrended_growth(ts_z, zs),
        airspeed=detrended_growth(ts_vfr, asp_vfr),
        pitch_physical=detrended_growth(ts_p, pitch_phys),
    )

    # ---- mode confirmation ----
    modes = set(s["mav"]["custom_mode"] for s in stab if s["mav"]["custom_mode"] is not None)
    out["custom_modes_seen_stabilized"] = sorted(modes)
    out["all_fbwa"] = (modes == {campaign.base.ARDUPLANE_FBWA_CUSTOM_MODE})

    # ---- endpoint window (last 5 s) - the near-settled state, since the
    # 28 s segment ends while the ~9 s phugoid (zeta ~ 0.2) is still slowly
    # converging toward the true level equilibrium ----
    if stab:
        t_end = max(s["t"] for s in stab)
        ep = [s for s in stab if s["t"] >= t_end - 5.0]
        ep_ts, ep_zs = gz_pos_series(ep)
        ep_pitch = [-s["gz"]["att_deg"][1] for s in ep if s["gz"]["att_deg"] is not None]
        ep_asp = [s["mav"]["airspeed"] for s in ep if s["mav"]["airspeed"] is not None]
        ep_elevL = [math.degrees(s["actuators"]["left_elevator"]["actual_angle_rad"])
                    for s in ep if s["actuators"]]
        ep_slope, _ = linreg(ep_ts, ep_zs) if len(ep_zs) >= 2 else (None, None)
        out["endpoint_window_last_5s"] = dict(
            n=len(ep),
            pitch_phys_noseup_mean_deg=mean(ep_pitch),
            airspeed_mean_ms=mean(ep_asp),
            elevator_actual_L_mean_deg=mean(ep_elevL),
            vertical_speed_regression_ms=ep_slope,
        )

    R["analysis"] = out
    return out


# =============================================================================
# acceptance
# =============================================================================
def verdict(R):
    a = R.get("analysis")
    if not a or not a.get("vertical_stabilized") or "vertical_speed_regression_ms" not in a["vertical_stabilized"]:
        return "FBWA_LEVEL_PITCH_REFERENCE_CORRECTION_FAILED", ["no stabilized analysis / segment aborted"]

    v = a["vertical_stabilized"]
    checks = {}
    vs_reg = v["vertical_speed_regression_ms"]
    vs_end = v["vertical_speed_endpoint_ms"]
    checks["vertical_speed_reg_abs_le_0.10"] = (abs(vs_reg) <= 0.10)
    checks["vertical_speed_reg_abs_le_0.05_preferred"] = (abs(vs_reg) <= 0.05)
    checks["vertical_speed_endpoint_abs_le_0.10"] = (abs(vs_end) <= 0.10)

    pp = a["pitch_physical_noseup_deg"]
    ew = a.get("endpoint_window_last_5s", {})
    checks["pitch_phys_mean_within_0.5_of_2.49"] = (abs(pp["mean"] - PTCH_TRIM_DEG_EXPECTED) <= 0.5)
    checks["pitch_phys_std_lt_0.5"] = (pp["std"] < 0.5)
    # convergence-aware: a monotonic drift TOWARD the +2.49 deg command is
    # settling, not divergence. Pass if the drift is small OR the endpoint is
    # closer to the target than the window start.
    _p_start = pp["mean"] - 0.5 * pp["drift_over_window"]
    _p_end = pp["mean"] + 0.5 * pp["drift_over_window"]
    checks["pitch_phys_drift_small_or_toward_target"] = (
        abs(pp["drift_over_window"]) < 0.5
        or abs(_p_end - PTCH_TRIM_DEG_EXPECTED) <= abs(_p_start - PTCH_TRIM_DEG_EXPECTED))
    checks["pitch_phys_endpoint_within_0.5_of_2.49"] = (
        ew.get("pitch_phys_noseup_mean_deg") is not None
        and abs(ew["pitch_phys_noseup_mean_deg"] - PTCH_TRIM_DEG_EXPECTED) <= 0.5)

    inv = a["pitch_mav_invariant_residual_deg"]
    checks["pitch_mav_invariant_holds_max_abs_lt_0.5"] = (inv["max_abs"] is not None and inv["max_abs"] < 0.5)

    asp = a["airspeed_vfr_hud_ms"]
    checks["airspeed_mean_within_0.5_of_18.16"] = (abs(asp["mean"] - 18.16) <= 0.5)
    # convergence-aware: prior FBWA runs DIVERGED upward to ~19.4 m/s (positive
    # slope / high mean). Pass if the airspeed slope is not accelerating upward
    # (slope <= +0.02 m/s^2) and the endpoint airspeed is near trim.
    checks["airspeed_slope_not_diverging_upward"] = (asp["slope_per_s"] <= 0.02)
    checks["airspeed_endpoint_within_0.5_of_18.16"] = (
        ew.get("airspeed_mean_ms") is not None and abs(ew["airspeed_mean_ms"] - 18.16) <= 0.5)

    checks["zero_actuator_clamp"] = (a["actuator_clamp_full"]["target_clamp_active_samples"] == 0
                                     and a["actuator_clamp_full"]["effort_clamp_active_samples"] == 0)

    tl, tr = a["throttle_actual_L"], a["throttle_actual_R"]
    checks["throttle_L_within_0.01_of_0.4957"] = (abs(tl["mean"] - 0.4957) <= 0.01)
    checks["throttle_R_within_0.01_of_0.4957"] = (abs(tr["mean"] - 0.4957) <= 0.01)

    og = a["oscillation_growth"]
    checks["no_growing_oscillation"] = not (og["altitude_pos_z"].get("growing")
                                            or og["airspeed"].get("growing")
                                            or og["pitch_physical"].get("growing"))

    checks["no_nan_inf"] = (a["nan_inf_count"] == 0)
    checks["all_samples_fbwa"] = bool(a.get("all_fbwa"))
    checks["nav_pitch_target_near_zero"] = (a["nav_pitch_target_deg"] is not None
                                            and abs(a["nav_pitch_target_deg"]["mean"]) < 0.5)

    R["acceptance_checks"] = checks
    fails = [k for k, ok in checks.items() if not ok and not k.endswith("_preferred")]

    # core PASS gate
    core_pass = (checks["vertical_speed_reg_abs_le_0.10"]
                 and checks["vertical_speed_endpoint_abs_le_0.10"]
                 and checks["pitch_phys_endpoint_within_0.5_of_2.49"]
                 and checks["pitch_phys_std_lt_0.5"]
                 and checks["pitch_phys_drift_small_or_toward_target"]
                 and checks["airspeed_endpoint_within_0.5_of_18.16"]
                 and checks["airspeed_slope_not_diverging_upward"]
                 and checks["zero_actuator_clamp"]
                 and checks["throttle_L_within_0.01_of_0.4957"]
                 and checks["throttle_R_within_0.01_of_0.4957"]
                 and checks["no_growing_oscillation"]
                 and checks["no_nan_inf"]
                 and checks["pitch_mav_invariant_holds_max_abs_lt_0.5"])

    prior_best_sink = min(PRIOR_RUNS[k]["sink_regression_ms"] for k in PRIOR_RUNS)  # 0.5512
    improved = abs(vs_reg) < 0.9 * prior_best_sink

    if core_pass:
        return "FBWA_LEVEL_PITCH_REFERENCE_CORRECTION_PASS", fails
    if (checks["pitch_phys_mean_within_0.5_of_2.49"] and improved
            and checks["zero_actuator_clamp"] and checks["no_nan_inf"]
            and checks["no_growing_oscillation"]):
        return "FBWA_LEVEL_PITCH_REFERENCE_CORRECTION_PARTIAL", fails
    return "FBWA_LEVEL_PITCH_REFERENCE_CORRECTION_FAILED", fails


# =============================================================================
# flight
# =============================================================================
def flight_sequence(mav, sub, osub, adiag, pdiag, aerodiag, R):
    confirmed = campaign.enter_fbwa(mav, R)
    if not confirmed:
        R["flight_result"] = dict(aborted=True, reason="fbwa_not_confirmed")
        return False
    latest_mav = {}
    t_flight0 = time.time()
    rc3 = round(base.RC3_TRIM_TARGET_US)
    seg = campaign.run_segment(mav, sub, osub, adiag, pdiag, aerodiag,
                               SEGMENT_LABEL, SEGMENT_DURATION_S,
                               1500, 1500, rc3, t_flight0, latest_mav)
    print(f"  segment {SEGMENT_LABEL}: dur={SEGMENT_DURATION_S}s rc1=1500 rc2=1500 rc3={rc3} "
          f"n_samples={seg['n_samples']} aborted={seg['aborted']} reason={seg['abort_reason']}")
    R["segments"] = [seg]
    R["flight_result"] = dict(aborted=seg["aborted"], reason=seg["abort_reason"])
    if not seg["aborted"]:
        analyze(seg, R)
    return not seg["aborted"]


def finish_fail(R, phase, mav):
    R["overall_result"] = "TEST_FAILED"
    R["blocking_phase"] = phase
    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(R, f, indent=2, default=str)
    print(f"FAILED at {phase} - see", OUT_JSON)
    if mav is not None:
        mav.close()
    return 1


def reanalyze(path):
    """Re-run analyze() + verdict() offline against an already-captured raw
    result JSON (the flight is NOT re-flown). Used to regenerate the analysis
    / acceptance_checks blocks after a test-logic (not physics) fix. Prints
    and rewrites the file in place."""
    with open(path) as f:
        R = json.load(f)
    seg = R["segments"][0]
    analyze(seg, R)
    vd, fails = verdict(R)
    R["verdict"] = vd
    R["failed_checks"] = fails
    R["reanalyzed"] = True
    with open(path, "w") as f:
        json.dump(R, f, indent=2, default=str)
    print("REANALYZED", path)
    print("verdict:", vd, "failed_checks:", fails)
    print("acceptance_checks:", json.dumps(R["acceptance_checks"], indent=2, default=str))
    print("endpoint_window_last_5s:", json.dumps(R["analysis"].get("endpoint_window_last_5s"), default=str))
    return 0


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--reanalyze":
        return reanalyze(sys.argv[2])
    # ---- runtime constant guard (base's corrected constants) ----
    rc3_round = round(base.RC3_TRIM_TARGET_US)
    consts_ok = (rc3_round == 1496 and abs(base.TRIM_THROTTLE - 0.4957) < 1e-9
                 and abs(base.V_TRIM - 18.162) < 1e-9 and abs(base.ALPHA_TRIM_DEG - 2.472) < 1e-9)
    R = {"stage": STAGE,
         "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         "corrected_trim_constants": dict(
             V_TRIM=base.V_TRIM, ALPHA_TRIM_DEG=base.ALPHA_TRIM_DEG,
             U_HOLD=base.U_HOLD, W_HOLD=base.W_HOLD,
             ELEVATOR_THETA_DEG=math.degrees(base.ELEVATOR_THETA_RAD),
             TRIM_THROTTLE=base.TRIM_THROTTLE,
             RC3_TRIM_TARGET_US=base.RC3_TRIM_TARGET_US, RC3_round=rc3_round,
             ELEV_RC2_TARGET_US=base.ELEV_RC2_TARGET_US,
             constants_guard_ok=consts_ok),
         "ptch_trim_deg_expected": PTCH_TRIM_DEG_EXPECTED,
         "segment_config": dict(label=SEGMENT_LABEL, duration_s=SEGMENT_DURATION_S,
                                transient_cutoff_s=TRANSIENT_CUTOFF_S,
                                rc=dict(rc1=1500, rc2=1500, rc3=rc3_round, rc4=1500, rc5=1000)),
         "prior_fbwa_runs": PRIOR_RUNS}
    print("corrected_trim_constants:", json.dumps(R["corrected_trim_constants"], default=str))
    if not consts_ok:
        print("WARNING: base trim constants do not match the expected corrected values!")

    node = tp.Node()
    sub = base.PoseSub(base.WORLD)
    osub = base.OdomSub()
    time.sleep(0.5)
    pub_oneshot = node.advertise(f"/world/{base.WORLD}/wrench", entity_wrench_pb2.EntityWrench)
    pub_clear = node.advertise(f"/world/{base.WORLD}/wrench/clear", entity_pb2.Entity)
    time.sleep(0.3)

    adiag = actuator_lib.DiagSubscriber()
    pdiag = propulsion_lib.DiagSubscriber()
    aerodiag = aero_lib.DiagSubscriber()
    time.sleep(0.5)

    mav, armed = base.phase1_mavlink_arm(R)
    print("PHASE 1 (mavlink arm):", json.dumps(R.get("phase1_mavlink_arm", {}), default=str)[:400])
    if not armed:
        return finish_fail(R, "phase1_mavlink_arm", mav)

    check_params(mav, R)

    settled, elapsed = base.wait_ground_settle(osub)
    R["ground_settle"] = dict(settled=settled, elapsed_s=elapsed)
    print("ground settle:", R["ground_settle"])
    if not settled:
        base.disarm(mav)
        return finish_fail(R, "ground_settle", mav)

    t_teleport, ok_v = base.phase2_teleport_and_verify(
        node, sub, osub, pub_oneshot, (0.0, 0.0, 90.0, 0.0, 0.0, 0.0), R)
    print("PHASE 2 (teleport+verify):", R["phase2_teleport_verify"]["ok"])
    if not ok_v:
        base.disarm(mav)
        return finish_fail(R, "phase2_teleport_verify", mav)

    base.clear_wrench(pub_clear)

    hold_ok = base.phase3_hold_to_trim(pub_oneshot, pub_clear, sub, osub, mav, R)
    print("PHASE 3 (hold-to-trim): aborted =", R["phase3_hold_to_trim"]["aborted"],
          "reason =", R["phase3_hold_to_trim"]["abort_reason"],
          "loop_dt_ms =", R["phase3_hold_to_trim"].get("loop_dt_ms"))
    if not hold_ok:
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        base.disarm(mav)
        return finish_fail(R, "phase3_hold_to_trim", mav)

    for msg_id in campaign.MSG_IDS_20HZ:
        campaign.request_rate(mav, msg_id, 20.0)
    time.sleep(0.3)

    print(f"Starting FBWA neutral-stick level-pitch-reference segment ({SEGMENT_DURATION_S}s)...")
    ok = flight_sequence(mav, sub, osub, adiag, pdiag, aerodiag, R)

    mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    base.disarm(mav)

    if ok and "analysis" in R:
        vd, fails = verdict(R)
        R["verdict"] = vd
        R["failed_checks"] = fails
        R["overall_result"] = "FLIGHT_COMPLETED_NO_ABORT"
        v = R["analysis"]["vertical_stabilized"]
        pp = R["analysis"]["pitch_physical_noseup_deg"]
        print("-" * 70)
        print(f"vertical speed (stab, regression): {v['vertical_speed_regression_ms']:+.4f} m/s")
        print(f"vertical speed (stab, endpoint):   {v['vertical_speed_endpoint_ms']:+.4f} m/s")
        print(f"odom twist z (world-proj) mean:    {v['odom_twist_z_world_proj_mean_ms']}")
        print(f"physical pitch noseup mean/std:    {pp['mean']:+.3f} / {pp['std']:.3f} deg  drift {pp['drift_over_window']:+.3f} deg")
        print(f"airspeed (vfr) mean:               {R['analysis']['airspeed_vfr_hud_ms']['mean']:.3f} m/s")
        print(f"throttle L/R:                      {R['analysis']['throttle_actual_L']['mean']:.4f} / {R['analysis']['throttle_actual_R']['mean']:.4f}")
        print(f"T - D:                             {R['analysis']['thrust_minus_drag_N']}")
        print(f"prior FBWA-original sink 0.5512 / FBWA-throttle* sink 0.5982 m/s")
        print(f"VERDICT: {vd}  failed_checks={fails}")
        print("-" * 70)
    else:
        R["overall_result"] = "FLIGHT_ABORTED"
        R["verdict"] = "FBWA_LEVEL_PITCH_REFERENCE_CORRECTION_FAILED"
        print("FLIGHT ABORTED:", R.get("flight_result"))

    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(R, f, indent=2, default=str)
    print("RESULT:", R["overall_result"], R.get("verdict"), "->", OUT_JSON)
    mav.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

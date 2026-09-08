#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_NAVIGATION_AND_AUTOTUNE_VALIDATION : PRE-FLIGHT CHECK
(controls-integration, 2026-09-07).

PURPOSE
-------
A SHORT, live, one-shot readback + sanity pass executed BEFORE the navigation
campaign (test_ardupilot_navigation_validation.py) is ever flown. It answers
exactly one question: "is the SITL/Gazebo pair in the state the navigation
stage assumes it is in?" - by LIVE PARAMETER READBACK over MAVLink, never by
parsing config/ardupilot/falcon_v2_sitl.parm.

WHAT IT CHECKS (all gating unless marked report-only)
  1. TECS_PTCH_DAMP == 0.6                       (adopted SIM-specific value)
  2. RLL_RATE_P/I/D/FF == 0.25/0.125/0.002/0.125
     PTCH_RATE_P/I/D/FF == 0.25/0.125/0.002/0.125
  3. AIRSPEED_MIN=16, AIRSPEED_CRUISE=18, AIRSPEED_MAX=28,
     ARSPD_USE=1, ARSPD_TYPE=100
  4. PTCH_TRIM_DEG == 2.49
  5. SITL/Gazebo stable + NORMAL arming succeeds (DCM/EKF gate, then
     MAV_CMD_COMPONENT_ARM_DISARM - no force-arm, no arming-check bypass)
  6. No NaN/Inf in the attitude / airspeed / position streams (both MAVLink
     and Gazebo ground truth), on the ground AND airborne
  7. Roll SIGN sanity ONLY (short): in FBWA, a right-roll stick command must
     produce a right bank. This is NOT a re-run of the control-surface
     sign-mapping suite; it is a single-axis regression tripwire.

WHAT IT DELIBERATELY DOES NOT DO
  * No long legacy suite, no TECS campaign, no navigation mode, no AUTOTUNE.
  * WRITES NO PARAMETER of any kind. Reads only.
  * Changes no SDF, plugin, aero/propulsion/actuator/sensor value.

SIGN CONVENTION UNDER TEST (stated explicitly, verified - never assumed)
  Body frame is FLU (+X fwd, +Y left, +Z up), CLAUDE.md. A positive rotation
  about +X by the right-hand rule carries +Y (left wing) toward +Z (up), i.e.
  POSITIVE ROLL ANGLE == RIGHT WING DOWN == RIGHT BANK. ArduPilot's own FRD
  convention independently also gives positive roll == right wing down, so
  Gazebo ground-truth roll and MAVLink ATTITUDE.roll must agree in SIGN.
  COMMAND: RC1 > RC1_TRIM is the roll-right command
  (campaign.rc1_for_roll_deg: rc1 = 1500 + 400*bank/ROLL_LIMIT_DEG, with
  SERVO1_REVERSED=1 already accounted for on the servo side by
  falcon_v2_sitl.parm). The check below requires MEASURED roll > 0 for
  RC1 > trim and MEASURED roll < 0 for RC1 < trim. A failure is reported as
  a control-direction finding and is NEVER "adapted" away.

PITCH TELEMETRY CAVEAT (same as every prior ArduPlane stage here)
  GCS ATTITUDE.pitch = (true_pitch - PTCH_TRIM_DEG), GCS_MAVLink_Plane.cpp:139.
  Gazebo Euler pitch is nose-DOWN-positive in this FLU world, so physical
  nose-up pitch = -(gz euler pitch). Roll needs no such correction.

USAGE (a fresh gz sim + gdb-wrapped arduplane pair MUST already be running -
see tests/gazebo/scripts/run_ardupilot_navigation_preflight.sh):
    python3 test_ardupilot_navigation_preflight.py
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import gz.transport13 as tp  # noqa: E402
from gz.msgs10 import entity_wrench_pb2, entity_pb2  # noqa: E402

import test_ardupilot_basic_closed_loop_flight as base  # noqa: E402
import test_ardupilot_basic_closed_loop_flight_campaign as campaign  # noqa: E402
import test_ardupilot_fbwa_level_pitch_reference_correction as fbwa  # noqa: E402
import aero_lib  # noqa: E402
import propulsion_lib  # noqa: E402
import actuator_lib  # noqa: E402

read_param = fbwa.read_param
mean = fbwa.mean

STAGE = "ARDUPLANE_NAVIGATION_AND_AUTOTUNE_VALIDATION"
OUT_JSON = os.path.join(base.RESULTS_DIR,
                        "ardupilot_navigation_preflight_result.json")

# -----------------------------------------------------------------------------
# EXPECTED VALUES - every one of these is a LOCKED project value quoted from
# its own source of truth. This file asserts them; it never sets them.
# -----------------------------------------------------------------------------
EXPECTED = {
    # config/ardupilot/falcon_v2_sitl.parm, section
    # FALCON_V2_SIM_VALIDATED_TECS_PITCH_DAMPING (adopted 2026-09-05,
    # docs/source_of_truth/controls/ardupilot_tecs_pitch_damping_loop.yaml)
    "TECS_PTCH_DAMP": 0.6,
    # falcon_v2_sitl.parm FALCON_V2_MANUFACTURER_INITIAL_PID
    # (Titan Dynamics Falcon V2 Build & User Manual Rev 1.0)
    "RLL_RATE_P": 0.25, "RLL_RATE_I": 0.125, "RLL_RATE_D": 0.002, "RLL_RATE_FF": 0.125,
    "PTCH_RATE_P": 0.25, "PTCH_RATE_I": 0.125, "PTCH_RATE_D": 0.002, "PTCH_RATE_FF": 0.125,
    # falcon_v2_sitl.parm, SIMULATION_DERIVED_FROM_GAZEBO_ENVELOPE
    # (docs/test_results/2026-08-27_flight_envelope_validation.md)
    "AIRSPEED_MIN": 16.0, "AIRSPEED_CRUISE": 18.0, "AIRSPEED_MAX": 28.0,
    # falcon_v2_sitl.parm SITL_REQUIRED / SIMULATION_ADDED_PITOT_SENSOR
    "ARSPD_USE": 1.0, "ARSPD_TYPE": 100.0,
    # falcon_v2_sitl.parm, measured straight-and-level pitch attitude
    # (docs/test_results/2026-08-28_ardupilot_longitudinal_equilibrium_...md)
    "PTCH_TRIM_DEG": 2.49,
}
# Absolute tolerances. RATE_D=0.002 is transported as a float32 PARAM_VALUE;
# 1e-6 is ~3 orders below the smallest expected value and well above float32
# representation error for these magnitudes.
TOL = 1e-6

# Additional parameters read for the record / used by the navigation harness.
EXTRA_PARAMS = [
    "AUTOTUNE_LEVEL", "ROLL_LIMIT_DEG", "PTCH_LIM_MAX_DEG", "PTCH_LIM_MIN_DEG",
    "RC1_MIN", "RC1_MAX", "RC1_TRIM", "RC1_DZ", "RC1_REVERSED",
    "RC2_MIN", "RC2_MAX", "RC2_TRIM", "RC2_DZ", "RC2_REVERSED",
    "RC3_MIN", "RC3_MAX", "RC3_TRIM", "RC3_DZ", "RC3_REVERSED",
    "RC4_MIN", "RC4_MAX", "RC4_TRIM", "RC4_DZ", "RC4_REVERSED",
    "SERVO1_MIN", "SERVO1_MAX", "SERVO1_TRIM", "SERVO1_REVERSED",
    "SERVO2_MIN", "SERVO2_MAX", "SERVO2_TRIM", "SERVO2_REVERSED",
    "SERVO4_MIN", "SERVO4_MAX", "SERVO4_TRIM", "SERVO4_REVERSED",
    "THR_MIN", "THR_MAX", "TRIM_THROTTLE", "THR_SLEWRATE",
    "STICK_MIXING", "FLIGHT_OPTIONS", "FENCE_ENABLE", "FENCE_ACTION",
    "WP_LOITER_RAD", "WP_RADIUS", "NAVL1_PERIOD", "NAVL1_DAMPING",
    "RTL_ALTITUDE", "RTL_AUTOLAND", "ALT_HOLD_RTL", "LIM_ROLL_CD",
    "GUIDED_OPTIONS", "LOITER_RAD", "TERRAIN_FOLLOW",
    "SIM_WIND_SPD", "SIM_WIND_DIR", "SIM_WIND_TURB",
    "SIM_OPOS_LAT", "SIM_OPOS_LNG", "SIM_OPOS_ALT", "SIM_OPOS_HDG",
    "TECS_CLMB_MAX", "TECS_SINK_MIN", "TECS_SINK_MAX", "TECS_TIME_CONST",
    "TECS_THR_DAMP", "TECS_INTEG_GAIN", "TECS_SPDWEIGHT", "TECS_PITCH_MAX",
    "TECS_PITCH_MIN", "TECS_HDEM_TCONST", "TECS_RLL2THR",
    "AHRS_EKF_TYPE", "ARSPD_OPTIONS", "MIN_GROUNDSPEED", "AIRSPEED_STALL",
    "KFF_THR2PTCH", "ARMING_CHECK", "ARMING_REQUIRE",
]

# --- short sign-sanity flight window ---
AIRBORNE_ALT_M = 90.0        # identical to every prior ArduPlane stage here
SETTLE_S = 5.0               # neutral-stick settle after FBWA handoff
ROLL_CMD_S = 5.0             # per roll direction
ROLL_CMD_DEG = 20.0          # well inside ROLL_LIMIT_DEG=45; a clear signal
ROLL_SIGN_MIN_DEG = 5.0      # response must exceed this magnitude to count
ATT_ABORT_DEG = 60.0


def finite(x):
    return x is not None and isinstance(x, (int, float)) and math.isfinite(x)


def nonfinite_scan(samples):
    """Scan every attitude / airspeed / position channel of every sample for
    NaN/Inf. Returns (n_bad, detail). A value that is absent (None, e.g. a
    MAVLink message not yet received) is NOT counted as non-finite; it is
    counted separately so a silent telemetry dropout cannot masquerade as a
    clean pass."""
    bad = []
    missing = {}
    for s in samples:
        chans = {
            "mav.att_roll_deg": s["mav"]["att_roll_deg"],
            "mav.att_pitch_deg": s["mav"]["att_pitch_deg"],
            "mav.att_yaw_deg": s["mav"]["att_yaw_deg"],
            "mav.airspeed": s["mav"]["airspeed"],
            "mav.groundspeed": s["mav"]["groundspeed"],
            "mav.vfr_alt": s["mav"]["vfr_alt"],
            "mav.relative_alt_m": s["mav"]["relative_alt_m"],
        }
        if s["gz"]["att_deg"] is not None:
            for i, n in enumerate(("gz.roll", "gz.pitch", "gz.yaw")):
                chans[n] = s["gz"]["att_deg"][i]
        else:
            missing["gz.att_deg"] = missing.get("gz.att_deg", 0) + 1
        if s["gz"]["pos"] is not None:
            for i, n in enumerate(("gz.x", "gz.y", "gz.z")):
                chans[n] = s["gz"]["pos"][i]
        else:
            missing["gz.pos"] = missing.get("gz.pos", 0) + 1
        if s["gz"]["v_body"] is not None:
            for i, n in enumerate(("gz.u", "gz.v", "gz.w")):
                chans[n] = s["gz"]["v_body"][i]
        else:
            missing["gz.v_body"] = missing.get("gz.v_body", 0) + 1
        for name, v in chans.items():
            if v is None:
                missing[name] = missing.get(name, 0) + 1
            elif not math.isfinite(v):
                bad.append(dict(t=s["t"], channel=name, value=str(v)))
    return len(bad), dict(nonfinite=bad[:50], nonfinite_count=len(bad),
                          missing_counts=missing, n_samples=len(samples))


def sample_window(mav, sub, osub, adiag, pdiag, aerodiag, label, duration_s,
                  rc1, rc2, rc3, t0_flight, latest):
    """20 Hz combined telemetry, RC republished every 0.1 s (RC_OVERRIDE_TIME
    is 3.0 s). Same rates/structure as campaign.run_segment."""
    samples = []
    aborted, abort_reason = False, None
    t0 = time.time()
    last_rc = last_s = -1.0
    while time.time() - t0 < duration_s:
        tnow = time.time() - t0
        campaign.drain_mavlink(mav, latest)
        if tnow - last_rc >= campaign.RC_REFRESH_PERIOD:
            mav.send_rc_override(rc1=int(round(rc1)), rc2=int(round(rc2)),
                                 rc3=int(round(rc3)), rc4=1500, rc5=1000)
            last_rc = tnow
        if tnow - last_s >= campaign.SAMPLE_PERIOD:
            s = campaign.build_sample(time.time() - t0_flight, latest, sub, osub,
                                      adiag, pdiag, aerodiag)
            s["t_seg"] = tnow
            samples.append(s)
            last_s = tnow
            att = s["gz"]["att_deg"]
            if att is not None and (not math.isfinite(att[0]) or not math.isfinite(att[1])
                                    or abs(att[0]) > ATT_ABORT_DEG or abs(att[1]) > ATT_ABORT_DEG):
                aborted = True
                abort_reason = dict(reason="attitude_envelope_or_nonfinite", att_deg=att, t=s["t"])
                break
        time.sleep(0.005)
    return dict(label=label, rc1=rc1, rc2=rc2, rc3=rc3, n_samples=len(samples),
                samples=samples, aborted=aborted, abort_reason=abort_reason,
                actual_duration_s=time.time() - t0)


def roll_stats(seg):
    gz, mv = [], []
    for s in seg["samples"]:
        if s["gz"]["att_deg"] is not None and math.isfinite(s["gz"]["att_deg"][0]):
            gz.append(s["gz"]["att_deg"][0])
        if finite(s["mav"]["att_roll_deg"]):
            mv.append(s["mav"]["att_roll_deg"])
    # use the LAST 50 % of the window: the first half is the command transient
    def tail(xs):
        return xs[len(xs) // 2:] if len(xs) >= 4 else xs
    return dict(gz_roll_deg_mean_tail=mean(tail(gz)), gz_roll_deg_max_abs=(max(abs(x) for x in gz) if gz else None),
                mav_roll_deg_mean_tail=mean(tail(mv)), n_gz=len(gz), n_mav=len(mv))


def main():
    R = {"stage": STAGE,
         "check": "PRE_FLIGHT",
         "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         "writes_parameters": False,
         "expected_values": EXPECTED,
         "expected_value_tolerance_abs": TOL,
         "sign_convention_under_test": (
             "FLU body frame (CLAUDE.md). Positive roll about +X carries +Y (left "
             "wing) toward +Z (up) => POSITIVE ROLL = RIGHT BANK. RC1 > RC1_TRIM is "
             "the commanded-right-roll input. Gating check requires measured roll > "
             "+%.1f deg for right command and < -%.1f deg for left command, in BOTH "
             "Gazebo ground truth and MAVLink ATTITUDE." % (ROLL_SIGN_MIN_DEG, ROLL_SIGN_MIN_DEG)),
         "scope_note": ("Roll axis only, single short window. This is a regression "
                        "tripwire, NOT a re-run of test_control_surface_sign_mapping.py "
                        "or the CONTROL_SURFACE_DIRECTION_TEST suite.")}
    checks = {}

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

    # ---- 1. link up + NORMAL arming ----
    mav, armed = base.phase1_mavlink_arm(R)
    checks["mavlink_link_up"] = bool(R.get("phase1_mavlink_arm", {}).get("heartbeat"))
    checks["dcm_ekf_converged"] = bool(R.get("phase1_mavlink_arm", {}).get("dcm_ekf_ready"))
    checks["normal_arming_succeeds"] = bool(armed)
    print("arming:", checks["normal_arming_succeeds"])
    if mav is None:
        R["checks"] = checks
        R["overall_result"] = "PREFLIGHT_FAILED"
        R["blocking"] = "no_mavlink_link"
        write(R)
        return 1

    # ---- 2. LIVE parameter readback ----
    bulk = {}
    try:
        bulk = mav.fetch_all_params(timeout=45, idle_cutoff=3.0)
    except Exception as exc:                      # noqa: BLE001 - reported
        R["param_bulk_error"] = str(exc)
    live = {}
    for name in list(EXPECTED) + EXTRA_PARAMS:
        v = bulk.get(name)
        if v is None:
            v = read_param(mav, name)
        live[name] = v
    R["live_params"] = live
    R["param_bulk_count"] = len(bulk)
    R["params_unreadable"] = sorted(k for k in EXPECTED if live.get(k) is None)
    R["param_readback_source"] = ("MAVLink PARAM_REQUEST_LIST / PARAM_REQUEST_READ "
                                  "from the running vehicle - NOT parsed from the .parm file")

    per_param = {}
    for name, exp in EXPECTED.items():
        got = live.get(name)
        per_param[name] = dict(expected=exp, live=got,
                               ok=(got is not None and abs(got - exp) <= TOL))
    R["expected_param_readback"] = per_param
    checks["tecs_ptch_damp_0p6"] = per_param["TECS_PTCH_DAMP"]["ok"]
    checks["roll_rate_pid_unchanged"] = all(per_param[k]["ok"] for k in
                                            ("RLL_RATE_P", "RLL_RATE_I", "RLL_RATE_D", "RLL_RATE_FF"))
    checks["pitch_rate_pid_unchanged"] = all(per_param[k]["ok"] for k in
                                             ("PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D", "PTCH_RATE_FF"))
    checks["airspeed_envelope_16_18_28"] = all(per_param[k]["ok"] for k in
                                               ("AIRSPEED_MIN", "AIRSPEED_CRUISE", "AIRSPEED_MAX"))
    checks["airspeed_sensor_enabled"] = all(per_param[k]["ok"] for k in ("ARSPD_USE", "ARSPD_TYPE"))
    checks["ptch_trim_deg_2p49"] = per_param["PTCH_TRIM_DEG"]["ok"]
    checks["zero_wind"] = (live.get("SIM_WIND_SPD") is None or abs(live["SIM_WIND_SPD"]) < 1e-6)
    checks["atmosphere_datum_sim_opos_alt_zero"] = (live.get("SIM_OPOS_ALT") is not None
                                                    and abs(live["SIM_OPOS_ALT"]) < 1e-6)
    for k, v in per_param.items():
        print(f"  {k:16s} expected {v['expected']!s:>8s}  live {v['live']!s:>10s}  {'OK' if v['ok'] else 'MISMATCH'}")

    # The bulk dump takes tens of seconds with no RC_CHANNELS_OVERRIDE traffic;
    # re-confirm the armed state instead of silently proceeding disarmed.
    if not base.is_armed(mav):
        base.arm(mav)
        R["rearmed_after_param_dump"] = base.is_armed(mav)
        if not R["rearmed_after_param_dump"]:
            R["checks"] = checks
            R["overall_result"] = "PREFLIGHT_FAILED"
            R["blocking"] = "rearm_after_param_dump"
            write(R)
            mav.close()
            return 1

    # ---- 3. ground telemetry NaN/Inf scan ----
    for mid in campaign.MSG_IDS_20HZ:
        campaign.request_rate(mav, mid, 20.0)
    time.sleep(0.3)
    latest = {}
    t0_flight = time.time()
    ground = sample_window(mav, sub, osub, adiag, pdiag, aerodiag, "ground_stream",
                           5.0, 1500, 1500, 1000, t0_flight, latest)
    n_bad_g, det_g = nonfinite_scan(ground["samples"])
    R["ground_stream_scan"] = det_g
    checks["ground_telemetry_finite"] = (n_bad_g == 0 and ground["n_samples"] > 20)

    settled, elapsed = base.wait_ground_settle(osub)
    R["ground_settle"] = dict(settled=settled, elapsed_s=elapsed)
    checks["gazebo_ground_settle"] = bool(settled)
    if not settled:
        base.disarm(mav)
        R["checks"] = checks
        R["overall_result"] = "PREFLIGHT_FAILED"
        R["blocking"] = "ground_settle"
        write(R)
        mav.close()
        return 1

    # ---- 4. safe airborne initial condition (the project's proven, already
    # validated bring-up: teleport -> hold-to-trim -> FULL wrench release).
    # Everything after the release is free 6-DOF; no pose/velocity forcing. ----
    _, ok_v = base.phase2_teleport_and_verify(
        node, sub, osub, pub_oneshot, (0.0, 0.0, AIRBORNE_ALT_M, 0.0, 0.0, 0.0), R)
    checks["airborne_teleport_verified"] = bool(ok_v)
    base.clear_wrench(pub_clear)
    if not ok_v:
        base.disarm(mav)
        R["checks"] = checks
        R["overall_result"] = "PREFLIGHT_FAILED"
        R["blocking"] = "phase2_teleport_verify"
        write(R)
        mav.close()
        return 1

    hold_ok = base.phase3_hold_to_trim(pub_oneshot, pub_clear, sub, osub, mav, R)
    checks["hold_to_trim_ok"] = bool(hold_ok)
    if not hold_ok:
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        base.disarm(mav)
        R["checks"] = checks
        R["overall_result"] = "PREFLIGHT_FAILED"
        R["blocking"] = "phase3_hold_to_trim"
        write(R)
        mav.close()
        return 1

    # ---- 5. FBWA handoff + roll sign sanity ----
    fbwa_ok = campaign.enter_fbwa(mav, R)
    checks["fbwa_handoff_confirmed"] = bool(fbwa_ok)
    rc3 = int(round(base.RC3_TRIM_TARGET_US))
    segs = {}
    segs["settle"] = sample_window(mav, sub, osub, adiag, pdiag, aerodiag, "settle",
                                   SETTLE_S, 1500, 1500, rc3, t0_flight, latest)
    rc1_right = campaign.rc1_for_roll_deg(+ROLL_CMD_DEG)
    rc1_left = campaign.rc1_for_roll_deg(-ROLL_CMD_DEG)
    segs["roll_right_cmd"] = sample_window(mav, sub, osub, adiag, pdiag, aerodiag,
                                           "roll_right_cmd", ROLL_CMD_S, rc1_right, 1500,
                                           rc3, t0_flight, latest)
    segs["roll_left_cmd"] = sample_window(mav, sub, osub, adiag, pdiag, aerodiag,
                                          "roll_left_cmd", ROLL_CMD_S, rc1_left, 1500,
                                          rc3, t0_flight, latest)
    segs["recover"] = sample_window(mav, sub, osub, adiag, pdiag, aerodiag, "recover",
                                    3.0, 1500, 1500, rc3, t0_flight, latest)

    mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    base.disarm(mav)

    air_samples = [s for k in segs for s in segs[k]["samples"]]
    n_bad_a, det_a = nonfinite_scan(air_samples)
    R["airborne_stream_scan"] = det_a
    checks["airborne_telemetry_finite"] = (n_bad_a == 0 and len(air_samples) > 100)
    checks["no_flight_abort"] = not any(segs[k]["aborted"] for k in segs)

    rs_r = roll_stats(segs["roll_right_cmd"])
    rs_l = roll_stats(segs["roll_left_cmd"])
    R["roll_sign_sanity"] = dict(
        commanded_bank_deg=ROLL_CMD_DEG,
        rc1_right_us=rc1_right, rc1_left_us=rc1_left,
        rc1_trim_us=live.get("RC1_TRIM"),
        min_response_deg=ROLL_SIGN_MIN_DEG,
        right=rs_r, left=rs_l,
        abort_right=segs["roll_right_cmd"]["abort_reason"],
        abort_left=segs["roll_left_cmd"]["abort_reason"])
    gz_r, gz_l = rs_r["gz_roll_deg_mean_tail"], rs_l["gz_roll_deg_mean_tail"]
    mv_r, mv_l = rs_r["mav_roll_deg_mean_tail"], rs_l["mav_roll_deg_mean_tail"]
    checks["roll_sign_gazebo_correct"] = (
        finite(gz_r) and finite(gz_l) and gz_r > ROLL_SIGN_MIN_DEG and gz_l < -ROLL_SIGN_MIN_DEG)
    checks["roll_sign_mavlink_correct"] = (
        finite(mv_r) and finite(mv_l) and mv_r > ROLL_SIGN_MIN_DEG and mv_l < -ROLL_SIGN_MIN_DEG)
    checks["roll_sign_gz_mav_agree"] = (
        finite(gz_r) and finite(mv_r) and finite(gz_l) and finite(mv_l)
        and (gz_r * mv_r > 0) and (gz_l * mv_l > 0))
    print(f"roll right cmd -> gz {gz_r} deg / mav {mv_r} deg")
    print(f"roll left  cmd -> gz {gz_l} deg / mav {mv_l} deg")

    R["checks"] = checks
    failed = sorted(k for k, v in checks.items() if not v)
    R["failed_checks"] = failed
    R["overall_result"] = "PREFLIGHT_PASS" if not failed else "PREFLIGHT_FAILED"
    R["segments_summary"] = {k: {kk: vv for kk, vv in v.items() if kk != "samples"}
                             for k, v in segs.items()}
    write(R)
    print("-" * 70)
    for k in sorted(checks):
        print(f"  {'PASS' if checks[k] else 'FAIL'}  {k}")
    print("RESULT:", R["overall_result"], "failed:", failed, "->", OUT_JSON)
    mav.close()
    return 0 if not failed else 1


def write(R):
    """Write the pre-flight record. The bring-up phases (teleport / hold-to-
    trim) each carry a few hundred raw samples that are NOT evidence for any
    check in this file; they are summarised out so the pre-flight record stays
    small and readable. Nothing that a check depends on is dropped."""
    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    out = dict(R)
    for key in ("phase2_teleport_verify", "phase3_hold_to_trim"):
        blk = out.get(key)
        if isinstance(blk, dict):
            out[key] = {k: v for k, v in blk.items() if k not in ("samples", "attempts")}
            out[key]["samples_omitted"] = True
    with open(OUT_JSON, "w") as f:
        json.dump(out, f, indent=2, default=str)


if __name__ == "__main__":
    sys.exit(main())

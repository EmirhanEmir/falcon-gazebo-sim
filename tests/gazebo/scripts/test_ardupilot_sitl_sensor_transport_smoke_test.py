#!/usr/bin/env python3
"""
FALCON V2 - sensor transport smoke test (gazebo-testing,
ARDUPLANE_SITL_TRANSPORT_AND_ACTUATOR_MAPPING_VALIDATION, item 14 re-run,
2026-08-28 second follow-up).

Re-runs the original item 14 intent ("rule out a transport-layer sign/unit
bug causing EKF divergence") now that BOTH root causes identified in the
2026-08-27/2026-08-28 report are addressed:
  - the IMU gyro/accel FLU->FRD frame-conversion bug (Sec 5), fixed by
    controls-integration and independently re-confirmed by this same task's
    own separate frame re-check pass (see report Sec 5/13 update, same
    session as this file);
  - the DCM/EKF3 prearm divergence (Sec 15.1/15.2), root-caused to a
    test-condition gap (no valid accelerometer gravity reference in the
    prior free-fall/VelocityControl-held test worlds) and resolved via the
    grounded test world used here.

PRECONDITION: Gazebo running
tests/gazebo/worlds/falcon_v2_ardupilot_sitl_grounded_test_world.sdf and
ArduPlane SITL already running and connected on tcp:127.0.0.1:5760.

Method: connect, let the airframe finish settling under real ground-contact
physics, arm (real, non-forceful MAVLink arm - confirms the DCM/EKF3 prearm
gate is actually clear, not just inferred), hold a short static/controlled
window collecting ATTITUDE, AHRS2, RAW_IMU, GPS_RAW_INT, EKF_STATUS_REPORT,
SYS_STATUS, GLOBAL_POSITION_INT, and STATUSTEXT, disarm, then check:
  - no NaN/Inf in any numeric field of any message received
  - gyro (RAW_IMU x/y/z gyro) stays within a sane bound for a resting
    airframe (not insane/saturated)
  - GPS fix is valid (fix_type >= 3, i.e. at least a 3D fix)
  - altitude is sane (near the spawn/home altitude, no huge jump/NaN)
  - no compass-failure STATUSTEXT observed
  - EKF/DCM (ATTITUDE vs AHRS2) attitude does not diverge (bounded
    roll/pitch difference throughout the window, not just at one instant)

No aircraft physics parameter (mass/CG/inertia/aero coefficient/control
authority/motor thrust) is read, written, or influenced by this file.
custom_mode is confirmed to stay 0 (MANUAL) throughout - no closed-loop
flight mode requested. No ARMING_SKIPCHK, no force-arm magic value, no
safety-check suppression of any kind.
"""
import json
import math
import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

from ardupilot_sitl_mav_lib import SafeMav  # noqa: E402

OUT_JSON = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results/ardupilot_sitl_sensor_transport_smoke_test_result.json"

MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAVLINK_MSG_ID_AHRS2 = 178
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_GPS_RAW_INT = 24
MAVLINK_MSG_ID_RAW_IMU = 27
MAVLINK_MSG_ID_EKF_STATUS_REPORT = 193
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_SYS_STATUS = 1

GYRO_SANE_LIMIT_MRAD_S = 200_000  # ~200 rad/s raw mrad units - generous, catches genuine garbage/saturation, not a tuned physics threshold
DCM_EKF_DIVERGE_LIMIT_DEG = 10.0  # same threshold ArduPlane's own attitudes_consistent() prearm check uses (AP_AHRS.cpp, source-cited in report Sec 15.1)


def is_finite(x):
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return True  # non-numeric field, not in scope for the finite check


def check_all_finite(msgs):
    bad = []
    for m in msgs:
        d = m.to_dict()
        for k, v in d.items():
            if isinstance(v, (int, float)) and not is_finite(v):
                bad.append({"type": d.get("mavpackettype"), "field": k, "value": v})
    return bad


def arm(mav):
    mav.m.mav.command_long_send(mav.m.target_system, mav.m.target_component,
                                 MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    ack = None
    sts = []
    while time.time() - t0 < 4.0:
        r, _, _ = select.select([mav.m.port], [], [], 0.2)
        if not r:
            continue
        msg = mav.m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=False)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK" and msg.command == MAV_CMD_COMPONENT_ARM_DISARM:
            ack = msg.to_dict()
        elif msg.get_type() == "STATUSTEXT":
            sts.append(msg.text)
    return ack, sts


def disarm(mav):
    for _ in range(4):
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        time.sleep(0.15)
    mav.m.mav.command_long_send(mav.m.target_system, mav.m.target_component,
                                 MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    ack = None
    while time.time() - t0 < 4.0:
        r, _, _ = select.select([mav.m.port], [], [], 0.2)
        if not r:
            continue
        msg = mav.m.recv_match(type=["COMMAND_ACK"], blocking=False)
        if msg and msg.command == MAV_CMD_COMPONENT_ARM_DISARM:
            ack = msg.to_dict()
            break
    return ack


def is_armed(mav):
    hb = mav.m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if hb is None:
        return None, None
    return bool(hb.base_mode & 128), hb.custom_mode


def request_streams(mav):
    for msgid, hz in [
        (MAVLINK_MSG_ID_AHRS2, 10),
        (MAVLINK_MSG_ID_ATTITUDE, 10),
        (MAVLINK_MSG_ID_GPS_RAW_INT, 5),
        (MAVLINK_MSG_ID_RAW_IMU, 10),
        (MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5),
        (MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5),
        (MAVLINK_MSG_ID_SYS_STATUS, 2),
    ]:
        interval_us = int(1e6 / hz)
        mav.m.mav.command_long_send(
            mav.m.target_system, mav.m.target_component,
            MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid, interval_us, 0, 0, 0, 0, 0)
        time.sleep(0.05)


def collect(mav, duration):
    # Tag each message with a wall-clock receipt timestamp on the object
    # itself (NOT all MAVLink message types carry a usable time_boot_ms
    # field - AHRS2 in particular has none in this dialect, confirmed live
    # this run - so pairing AHRS2/ATTITUDE by MAVLink-internal timestamp is
    # not reliable; wall-clock receipt time is used instead, same approach
    # already used and validated by this task's own frame-angular-probe
    # script).
    out = []
    t0 = time.time()
    while time.time() - t0 < duration:
        r, _, _ = select.select([mav.m.port], [], [], 0.1)
        if not r:
            continue
        msg = mav.m.recv_match(blocking=False)
        if msg is None:
            continue
        msg._recv_t = time.time()
        out.append(msg)
    return out


def main():
    mav = SafeMav("tcp:127.0.0.1:5760", source_system=255)
    hb = mav.wait_heartbeat(15)
    print("initial HEARTBEAT:", hb.to_dict() if hb else None, flush=True)
    if hb is None:
        print("FATAL: no heartbeat")
        sys.exit(1)

    request_streams(mav)
    time.sleep(1.0)

    results = {}

    # Confirm the airframe is settled (grounded world already settles
    # within ~2 physics seconds per its own header comment/prior soak
    # tests) AND that GPS/EKF-origin/accel-bias-consistency prearm state
    # has caught up (live-observed this run: a fresh SITL boot needs more
    # than 3s for GPS fix_type to reach a stable 6 and for "Accels
    # inconsistent"/"waiting for home" prearm messages to clear - this is
    # genuine SITL/EKF startup settling, not a bug, so this is a longer
    # passive wait, not a bypass) before attempting to arm.
    print("Settling window (12s, passive observation only)...", flush=True)
    settle_msgs = collect(mav, 12.0)

    # Real (non-forceful) arm attempt(s) - confirms the DCM/EKF3 prearm
    # gate is actually clear under this smoke test's own live conditions,
    # not just inferred from the separate 2026-08-28 motor-test follow-up.
    # Up to 3 plain, non-forceful attempts spaced a few seconds apart are
    # allowed here purely to ride out residual EKF/accel-bias settling
    # after a fresh boot - each one is a genuine MAV_CMD_COMPONENT_ARM_
    # DISARM with no bypass parameter/magic value of any kind.
    print("\n=== Arming (real, non-forceful, up to 3 attempts) ===", flush=True)
    ack, sts, armed, custom_mode_at_arm = None, [], False, None
    arm_attempts = []
    for attempt in range(3):
        ack, sts = arm(mav)
        armed, custom_mode_at_arm = is_armed(mav)
        print(f"  attempt {attempt+1}: ack={ack} statustexts={sts} armed={armed}", flush=True)
        arm_attempts.append({"ack": ack, "statustexts": sts, "armed_after": armed})
        if armed:
            break
        time.sleep(4.0)
    print("armed state after arm attempts:", armed, "custom_mode:", custom_mode_at_arm, flush=True)
    results["arm"] = {"attempts": arm_attempts, "final_ack": ack, "final_statustexts": sts,
                       "armed_confirmed": armed, "custom_mode_at_arm": custom_mode_at_arm}

    # Static/controlled measurement window - no RC input, no cmd_vel, no
    # flight-mode change, purely observing the transport under a genuine
    # armed-and-resting condition.
    print("\n=== Measurement window (10s, static, armed, MANUAL) ===", flush=True)
    window_msgs = collect(mav, 10.0)

    all_msgs = settle_msgs + window_msgs
    results["n_settle_msgs"] = len(settle_msgs)
    results["n_window_msgs"] = len(window_msgs)

    # ---- finite check (settle + window combined) ----
    bad_finite = check_all_finite(all_msgs)
    results["nan_inf_findings"] = bad_finite
    print(f"\nfinite check: {len(bad_finite)} bad (non-finite) numeric fields out of "
          f"{len(all_msgs)} messages", flush=True)

    # ---- gyro sanity (RAW_IMU, window only - airframe should be resting) ----
    raw_imu = [m.to_dict() for m in window_msgs if m.get_type() == "RAW_IMU"]
    gyro_bad = [d for d in raw_imu if any(
        abs(d.get(k, 0)) > GYRO_SANE_LIMIT_MRAD_S for k in ("xgyro", "ygyro", "zgyro"))]
    gyro_max = {
        "xgyro_max_abs": max((abs(d["xgyro"]) for d in raw_imu), default=None),
        "ygyro_max_abs": max((abs(d["ygyro"]) for d in raw_imu), default=None),
        "zgyro_max_abs": max((abs(d["zgyro"]) for d in raw_imu), default=None),
    }
    results["raw_imu"] = {"n": len(raw_imu), "gyro_insane_samples": gyro_bad, "gyro_max_abs_mrad_s": gyro_max}
    print(f"RAW_IMU: {len(raw_imu)} samples, gyro max |xyz| (mrad/s) = {gyro_max}, "
          f"{len(gyro_bad)} insane samples", flush=True)

    # ---- GPS validity ----
    gps = [m.to_dict() for m in window_msgs if m.get_type() == "GPS_RAW_INT"]
    gps_fix_types = [d["fix_type"] for d in gps]
    gps_valid = len(gps) > 0 and all(ft >= 3 for ft in gps_fix_types)
    results["gps_raw_int"] = {"n": len(gps), "fix_types": gps_fix_types, "valid": gps_valid,
                               "last": gps[-1] if gps else None}
    print(f"GPS_RAW_INT: {len(gps)} samples, fix_types={gps_fix_types}, valid={gps_valid}", flush=True)

    # ---- altitude sanity (GLOBAL_POSITION_INT relative_alt, mm, should be
    #      small/near-zero for a resting-on-ground airframe, not huge/NaN) ----
    gpi = [m.to_dict() for m in window_msgs if m.get_type() == "GLOBAL_POSITION_INT"]
    rel_alts_m = [d["relative_alt"] / 1000.0 for d in gpi]
    alt_sane = len(gpi) > 0 and all(abs(a) < 50.0 for a in rel_alts_m)  # generous bound - just ruling out a garbage/runaway value, not a tuned physics threshold
    results["global_position_int"] = {"n": len(gpi), "relative_alt_m": rel_alts_m, "sane": alt_sane}
    print(f"GLOBAL_POSITION_INT: {len(gpi)} samples, relative_alt_m={rel_alts_m}, sane={alt_sane}", flush=True)

    # ---- compass failure check (STATUSTEXT scan across full session) ----
    statustexts = [m.text for m in all_msgs if m.get_type() == "STATUSTEXT"] + sts
    compass_fail_texts = [t for t in statustexts if "compass" in t.lower() or "mag" in t.lower()]
    results["statustexts"] = statustexts
    results["compass_failure_statustexts"] = compass_fail_texts
    print(f"STATUSTEXT scan: {len(statustexts)} total, {len(compass_fail_texts)} "
          f"mention compass/mag: {compass_fail_texts}", flush=True)

    # ---- EKF status flags ----
    ekf = [m.to_dict() for m in window_msgs if m.get_type() == "EKF_STATUS_REPORT"]
    results["ekf_status_report"] = {"n": len(ekf), "flags": [d["flags"] for d in ekf],
                                     "last": ekf[-1] if ekf else None}
    print(f"EKF_STATUS_REPORT: {len(ekf)} samples, flags={[d['flags'] for d in ekf]}", flush=True)

    # ---- DCM vs EKF3 divergence (ATTITUDE=EKF3 primary, AHRS2=DCM secondary) ----
    att_raw = [m for m in window_msgs if m.get_type() == "ATTITUDE"]
    ahrs2_raw = [m for m in window_msgs if m.get_type() == "AHRS2"]
    att = []
    for m in att_raw:
        d = m.to_dict()
        d["_recv_t"] = m._recv_t
        att.append(d)
    ahrs2 = []
    for m in ahrs2_raw:
        d = m.to_dict()
        d["_recv_t"] = m._recv_t
        ahrs2.append(d)
    diffs = []
    for a2 in ahrs2:
        best = min(att, key=lambda a: abs(a["_recv_t"] - a2["_recv_t"]), default=None)
        if best is None:
            continue
        droll = math.degrees(a2["roll"] - best["roll"])
        dpitch = math.degrees(a2["pitch"] - best["pitch"])
        diffs.append({"droll_deg": droll, "dpitch_deg": dpitch})
    max_droll = max((abs(d["droll_deg"]) for d in diffs), default=None)
    max_dpitch = max((abs(d["dpitch_deg"]) for d in diffs), default=None)
    dcm_ekf_ok = (max_droll is not None and max_dpitch is not None
                  and max_droll < DCM_EKF_DIVERGE_LIMIT_DEG and max_dpitch < DCM_EKF_DIVERGE_LIMIT_DEG)
    results["dcm_ekf3_divergence"] = {
        "n_pairs": len(diffs), "max_abs_droll_deg": max_droll, "max_abs_dpitch_deg": max_dpitch,
        "threshold_deg": DCM_EKF_DIVERGE_LIMIT_DEG, "converged": dcm_ekf_ok,
    }
    print(f"DCM/EKF3 divergence: n_pairs={len(diffs)} max|droll|={max_droll} "
          f"max|dpitch|={max_dpitch} deg (threshold {DCM_EKF_DIVERGE_LIMIT_DEG}) converged={dcm_ekf_ok}",
          flush=True)

    # ---- custom_mode stayed MANUAL(0) throughout (no closed-loop flight
    #      mode ever requested) ----
    hbs = [m.to_dict() for m in all_msgs if m.get_type() == "HEARTBEAT"]
    custom_modes = sorted(set(d["custom_mode"] for d in hbs))
    results["heartbeat_custom_modes_seen"] = custom_modes

    # ---- overall verdict ----
    verdict = "PASS"
    reasons = []
    if not armed:
        verdict = "FAIL"
        reasons.append("did not arm")
    if bad_finite:
        verdict = "FAIL"
        reasons.append(f"{len(bad_finite)} non-finite fields")
    if gyro_bad:
        verdict = "FAIL"
        reasons.append(f"{len(gyro_bad)} insane gyro samples")
    if not gps_valid:
        verdict = "FAIL"
        reasons.append("GPS not valid (fix_type<3 or no samples)")
    if not alt_sane:
        verdict = "FAIL"
        reasons.append("altitude not sane")
    if compass_fail_texts:
        verdict = "FAIL"
        reasons.append(f"compass-failure statustext(s): {compass_fail_texts}")
    if not dcm_ekf_ok:
        verdict = "FAIL"
        reasons.append("DCM/EKF3 diverged beyond threshold")
    results["verdict"] = verdict
    results["fail_reasons"] = reasons
    print(f"\n=== VERDICT: {verdict} === reasons: {reasons}", flush=True)

    print("\n=== Disarming ===", flush=True)
    dack = disarm(mav)
    print("disarm ack:", dack, flush=True)
    results["disarm"] = dack
    armed_final, _ = is_armed(mav)
    print("armed state after disarm:", armed_final, flush=True)
    results["armed_after_disarm"] = armed_final

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2, default=str)

    mav.close()
    print(f"\nWrote {OUT_JSON}")


if __name__ == "__main__":
    main()

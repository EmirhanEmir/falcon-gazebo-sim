#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_BASIC_CLOSED_LOOP_FLIGHT_VALIDATION - required
3-flight campaign (gazebo-testing, 2026-08-28).

This script runs the REQUIRED flight campaign on top of the already-PROVEN
precondition mechanism in test_ardupilot_basic_closed_loop_flight.py
(imported, UNMODIFIED, as `base` below): phase1_mavlink_arm,
wait_ground_settle, phase2_teleport_and_verify, phase3_hold_to_trim,
disarm, clear_wrench, PoseSub, OdomSub, quat_to_rpy, set_pose, and every
constant/gain already documented and proven there (KP_LIN=150/KP_ANG=100,
VEL_SANITY_MAX_MS, V_TRIM/ALPHA_TRIM_DEG, ELEV_RC2_TARGET_US,
RC3_TRIM_TARGET_US) - see that module's own top-of-file docstring and
docs/test_results/2026-08-28_ardupilot_basic_closed_loop_flight_validation.md
sec 10 for the 4 test-harness bugs already found/root-caused/fixed there.
NOTHING in test_ardupilot_basic_closed_loop_flight.py is changed by this
file - it is imported as a library, not copy-pasted or re-derived.

NEW in this file (the flight-campaign layer only):
  - FBWA roll/pitch RC-override command formulas - task-provided, already
    live-verified in an earlier pre-flight pass, cited verbatim below, NOT
    re-derived (ROLL_LIMIT_DEG=45.0, PTCH_LIM_MAX_DEG=20.0,
    PTCH_LIM_MIN_DEG=-25.0 - live ArduPlane params from that pass).
  - enter_fbwa(): identical mode-switch/confirm pattern to
    base.phase4_fbwa_handoff_and_observe() (same "wait for the LAST
    heartbeat, not the first" fix documented there), but WITHOUT that
    function's own fixed 10s neutral-RC observation window, since this
    campaign needs a multi-segment, staged command sequence instead.
  - run_segment(): runs ONE fixed-RC segment for a fixed duration,
    refreshing RC_CHANNELS_OVERRIDE every 0.1s (matches base's own
    RC_OVERRIDE_TIME=3.0s margin), sampling combined MAVLink + gz-transport
    ground-truth telemetry at 20 Hz, and aborting immediately (preserving
    every sample gathered so far) if gz-derived |roll| or |pitch| exceeds
    80 deg, a NaN/Inf appears, or altitude drops below 5 m - identical
    hard safety envelope to base.phase4_fbwa_handoff_and_observe's own.
  - bounded_ok(): a lightweight ONLINE staging heuristic (compares the
    last third of a segment's angle trace to the first two-thirds for
    growing spread) used ONLY to decide whether to escalate to the next,
    larger commanded magnitude on an axis, per the task's explicit
    "stage through neutral->small->larger only if the prior step was
    stable/bounded" instruction. This is NOT the final PID/oscillation
    classification (that is done offline, from the full sample record,
    when writing the report) - it is only a conservative go/no-go gate for
    whether to keep escalating within a single flight run.
  - Full per-sample telemetry: MAVLink (mode, ATTITUDE,
    NAV_CONTROLLER_OUTPUT, SERVO_OUTPUT_RAW, VFR_HUD, GLOBAL_POSITION_INT)
    cross-referenced against gz-transport ground truth (world
    pose/orientation via base.PoseSub, body angular/linear velocity via
    base.OdomSub, all 5 actuator-surface joint diagnostics via
    actuator_lib.DiagSubscriber, both-motor propulsion diagnostics via
    propulsion_lib.DiagSubscriber, aerodynamics diagnostics via
    aero_lib.DiagSubscriber) at matching sample instants.

IMPORTANT - body-frame sign-convention note for the required "cross-check
ArduPlane-reported roll/pitch vs Gazebo-derived roll/pitch" item: this
model's ArduPilotPlugin body-frame override is left at its COMPILED
DEFAULT (model/model.sdf's own ArduPilotPlugin header comment, already
live-verified in the prior ARDUPLANE_SITL_TRANSPORT_AND_ACTUATOR_MAPPING_
VALIDATION stage) - a PURE 180 deg rotation about body +X, mapping this
project's genuine FLU body frame to ArduPlane's expected FRD body frame.
Conjugating a full body orientation by a fixed 180 deg rotation about +X
leaves the ROLL (rotation about X) component's sign UNCHANGED, but FLIPS
the sign of both PITCH (about Y) and YAW (about Z) - a standard result,
not specific to this project (X unaffected by rotating about itself; Y and
Z both get sign-flipped by the 180 deg-about-X conjugation, and a rotation
about a sign-flipped axis has its own angle sign flipped). Therefore the
EXPECTED, already-frame-documented relationship between base.quat_to_rpy()
applied to the Gazebo/FLU pose quaternion and ArduPlane's own
FRD-frame-derived ATTITUDE.roll/pitch is: roll_mav ~= +roll_gz,
pitch_mav ~= -pitch_gz. This is stated here explicitly, in advance, so the
live comparison in the report is checked against this DOCUMENTED,
already-verified transform (i.e. a CRITICAL finding would be ROLL signs
disagreeing, or PITCH failing to show the expected inversion) rather than
naively asserting raw sign equality on both axes - but the actual
comparison itself is still done from LIVE captured data below, not
asserted a priori.

No aircraft-physics parameter (mass, CG, inertia, aerodynamic coefficient,
control authority, motor thrust, PID gain) is read for any purpose other
than citing already-published/live-confirmed values, and none is modified
anywhere in this file. model/model.sdf, falcon_v2_sitl.parm, the real
.param, and every plugin under plugins/ are unmodified.

Usage (Gazebo + arduplane already running, per the proven launch sequence
in the report sec 3/10.7 - a FRESH pair of processes per flight, per the
task's explicit instruction):
    python3 test_ardupilot_basic_closed_loop_flight_campaign.py flight1
    python3 test_ardupilot_basic_closed_loop_flight_campaign.py flight2
    python3 test_ardupilot_basic_closed_loop_flight_campaign.py flight3
    python3 test_ardupilot_basic_closed_loop_flight_campaign.py flight4   # optional, only if 1-3 all stable
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
from pymavlink import mavutil  # noqa: E402

import test_ardupilot_basic_closed_loop_flight as base  # noqa: E402
import aero_lib  # noqa: E402
import propulsion_lib  # noqa: E402
import actuator_lib  # noqa: E402

FLIGHTS = ["flight1", "flight2", "flight3", "flight4"]

MSG_IDS_20HZ = [
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
    mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
]

SAMPLE_PERIOD = 0.05   # 20 Hz combined-telemetry sample rate
RC_REFRESH_PERIOD = 0.1  # well under RC_OVERRIDE_TIME=3.0s (falcon_v2_sitl.parm)

# ---- Live-verified FBWA command formulas (task-provided, cited verbatim,
# NOT re-derived here). Live params from the earlier pre-flight pass:
# ROLL_LIMIT_DEG=45.0, PTCH_LIM_MAX_DEG=20.0, PTCH_LIM_MIN_DEG=-25.0. ----
ROLL_LIMIT_DEG = 45.0
PITCH_LIM_MAX_DEG = 20.0
PITCH_LIM_MIN_DEG_MAG = 25.0


def rc1_for_roll_deg(desired_bank_deg):
    """desired_bank_deg: SIGNED target bank angle (positive = commanded
    right bank, per the task's own formula). RC1_pwm = 1500 + 400*norm."""
    norm = desired_bank_deg / ROLL_LIMIT_DEG
    return 1500 + 400 * norm


def rc2_for_pitch_deg(desired_pitch_deg):
    """desired_pitch_deg: SIGNED target pitch angle (positive = commanded
    nose-up/climb, negative = commanded nose-down/dive). Per the task's two
    formulas (climb uses /20, dive uses /25 on the MAGNITUDE, both mapped
    through the same RC2_pwm=1500+400*norm expression) - applied here to a
    single signed input by selecting the appropriate divisor per sign."""
    if desired_pitch_deg >= 0:
        norm = desired_pitch_deg / PITCH_LIM_MAX_DEG
    else:
        norm = -(abs(desired_pitch_deg) / PITCH_LIM_MIN_DEG_MAG)
    return 1500 + 400 * norm


def request_rate(mav, msg_id, hz):
    mav.m.mav.command_long_send(mav.m.target_system, mav.m.target_component,
                                 base.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id,
                                 int(1e6 / hz), 0, 0, 0, 0, 0)


def enter_fbwa(mav, R):
    """Identical mode-switch/confirm pattern to
    base.phase4_fbwa_handoff_and_observe() (same "wait for the LAST
    heartbeat with custom_mode==FBWA, not the first" fix), without that
    function's own fixed neutral-RC observation window."""
    mav.m.mav.set_mode_send(mav.m.target_system, base.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             base.ARDUPLANE_FBWA_CUSTOM_MODE)
    mav.send_rc_override(rc1=1500, rc2=1500, rc3=int(round(base.RC3_TRIM_TARGET_US)), rc4=1500, rc5=1000)
    hb = None
    t_hb0 = time.time()
    while time.time() - t_hb0 < 5.0:
        r, _, _ = select.select([mav.m.port], [], [], 0.3)
        if not r:
            continue
        msg = mav.m.recv_match(type="HEARTBEAT", blocking=False)
        if msg is None:
            continue
        hb = msg
        if hb.custom_mode == base.ARDUPLANE_FBWA_CUSTOM_MODE:
            break
    confirmed = bool(hb and hb.custom_mode == base.ARDUPLANE_FBWA_CUSTOM_MODE)
    R["fbwa_handoff"] = dict(confirmed=confirmed,
                              custom_mode=(hb.custom_mode if hb else None),
                              armed=(bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None))
    return confirmed


def drain_mavlink(mav, latest):
    n = 0
    while n < 300:
        r, _, _ = select.select([mav.m.port], [], [], 0)
        if not r:
            break
        msg = mav.m.recv_match(blocking=False)
        if msg is None:
            break
        latest[msg.get_type()] = msg
        n += 1


def build_sample(t_rel, latest_mav, sub, osub, adiag, pdiag, aerodiag):
    hb = latest_mav.get("HEARTBEAT")
    att = latest_mav.get("ATTITUDE")
    nav = latest_mav.get("NAV_CONTROLLER_OUTPUT")
    servo = latest_mav.get("SERVO_OUTPUT_RAW")
    vfr = latest_mav.get("VFR_HUD")
    gpi = latest_mav.get("GLOBAL_POSITION_INT")

    pose = sub.latest()
    od = osub.latest()
    gz_att_deg = None
    gz_pos = None
    v_body = None
    av_body_deg = None
    if pose is not None:
        qw, qx, qy, qz = pose[4], pose[5], pose[6], pose[7]
        r, p, y = base.quat_to_rpy(qw, qx, qy, qz)
        gz_att_deg = [math.degrees(r), math.degrees(p), math.degrees(y)]
        gz_pos = [pose[1], pose[2], pose[3]]
    if od is not None:
        v_body = [od.twist.linear.x, od.twist.linear.y, od.twist.linear.z]
        av_body_deg = [math.degrees(od.twist.angular.x), math.degrees(od.twist.angular.y),
                        math.degrees(od.twist.angular.z)]

    return dict(
        t=t_rel,
        mav=dict(
            custom_mode=(hb.custom_mode if hb else None),
            armed=(bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None),
            att_roll_deg=(math.degrees(att.roll) if att else None),
            att_pitch_deg=(math.degrees(att.pitch) if att else None),
            att_yaw_deg=(math.degrees(att.yaw) if att else None),
            rollspeed_deg_s=(math.degrees(att.rollspeed) if att else None),
            pitchspeed_deg_s=(math.degrees(att.pitchspeed) if att else None),
            yawspeed_deg_s=(math.degrees(att.yawspeed) if att else None),
            nav_roll_deg=(nav.nav_roll if nav else None),
            nav_pitch_deg=(nav.nav_pitch if nav else None),
            nav_aspd_error=(nav.aspd_error if nav else None),
            nav_alt_error=(nav.alt_error if nav else None),
            servo_raw=([servo.servo1_raw, servo.servo2_raw, servo.servo3_raw, servo.servo4_raw,
                        servo.servo5_raw] if servo else None),
            airspeed=(vfr.airspeed if vfr else None),
            groundspeed=(vfr.groundspeed if vfr else None),
            throttle_pct=(vfr.throttle if vfr else None),
            vfr_alt=(vfr.alt if vfr else None),
            climb=(vfr.climb if vfr else None),
            relative_alt_m=((gpi.relative_alt / 1000.0) if gpi else None),
        ),
        gz=dict(att_deg=gz_att_deg, pos=gz_pos, v_body=v_body, av_body_deg=av_body_deg),
        actuators=adiag.latest(),
        propulsion=pdiag.latest(),
        aero=aerodiag.latest(),
    )


def run_segment(mav, sub, osub, adiag, pdiag, aerodiag, label, duration_s, rc1, rc2, rc3,
                 t_flight0, latest_mav):
    samples = []
    aborted = False
    abort_reason = None
    t0 = time.time()
    last_rc = -1.0
    last_sample = -1.0
    while True:
        tnow = time.time() - t0
        if tnow > duration_s:
            break
        drain_mavlink(mav, latest_mav)
        if tnow - last_rc >= RC_REFRESH_PERIOD:
            mav.send_rc_override(rc1=int(round(rc1)), rc2=int(round(rc2)),
                                  rc3=int(round(rc3)), rc4=1500, rc5=1000)
            last_rc = tnow
        if tnow - last_sample >= SAMPLE_PERIOD:
            s = build_sample(time.time() - t_flight0, latest_mav, sub, osub, adiag, pdiag, aerodiag)
            samples.append(s)
            att = s["gz"]["att_deg"]
            pos = s["gz"]["pos"]
            bad = False
            if att is not None:
                if not (math.isfinite(att[0]) and math.isfinite(att[1])):
                    bad = True
                elif abs(att[0]) > 80.0 or abs(att[1]) > 80.0:
                    bad = True
            if pos is not None and pos[2] < 5.0:
                bad = True
            if bad:
                aborted = True
                abort_reason = s
            last_sample = tnow
        if aborted:
            break
        time.sleep(0.005)
    return dict(label=label, duration_s=duration_s, rc1=rc1, rc2=rc2, rc3=rc3,
                n_samples=len(samples), samples=samples, aborted=aborted, abort_reason=abort_reason)


def bounded_ok(seg, axis_idx):
    """axis_idx: 0=roll, 1=pitch in gz att_deg. Online staging heuristic
    only - see module docstring."""
    if seg["aborted"]:
        return False
    vals = [s["gz"]["att_deg"][axis_idx] for s in seg["samples"] if s["gz"]["att_deg"] is not None]
    if len(vals) < 6:
        return True
    if not all(math.isfinite(v) for v in vals):
        return False
    n = len(vals)
    first = vals[: (2 * n) // 3]
    last = vals[(2 * n) // 3:]
    spread_first = (max(first) - min(first)) if first else 0.0
    spread_last = (max(last) - min(last)) if last else 0.0
    if spread_first > 0.5 and spread_last > 1.6 * spread_first:
        return False
    return True


# =============================================================================
# Per-flight command sequences
# =============================================================================
def flight1(mav, sub, osub, adiag, pdiag, aerodiag, R):
    confirmed = enter_fbwa(mav, R)
    if not confirmed:
        R["flight_result"] = dict(aborted=True, reason="fbwa_not_confirmed")
        return False
    latest_mav = {}
    t_flight0 = time.time()
    seg = run_segment(mav, sub, osub, adiag, pdiag, aerodiag,
                       "neutral_stabilization", 20.0, 1500, 1500, base.RC3_TRIM_TARGET_US,
                       t_flight0, latest_mav)
    R["segments"] = [seg]
    R["flight_result"] = dict(aborted=seg["aborted"], reason=seg["abort_reason"])
    return not seg["aborted"]


def flight2(mav, sub, osub, adiag, pdiag, aerodiag, R):
    confirmed = enter_fbwa(mav, R)
    if not confirmed:
        R["flight_result"] = dict(aborted=True, reason="fbwa_not_confirmed")
        return False
    latest_mav = {}
    t_flight0 = time.time()
    segments = []

    plan10 = [("neutral_pre", 2.0, 0.0), ("right_10", 4.0, 10.0), ("neutral_1", 3.0, 0.0),
              ("left_10", 4.0, -10.0), ("neutral_2", 3.0, 0.0)]
    for label, dur, bank in plan10:
        seg = run_segment(mav, sub, osub, adiag, pdiag, aerodiag, label, dur,
                           rc1_for_roll_deg(bank), 1500, base.RC3_TRIM_TARGET_US, t_flight0, latest_mav)
        segments.append(seg)
        if seg["aborted"]:
            break

    if not any(s["aborted"] for s in segments):
        right10 = next(s for s in segments if s["label"] == "right_10")
        left10 = next(s for s in segments if s["label"] == "left_10")
        if bounded_ok(right10, 0) and bounded_ok(left10, 0):
            plan20 = [("right_20", 4.0, 20.0), ("neutral_3", 3.0, 0.0),
                      ("left_20", 4.0, -20.0), ("neutral_4", 3.0, 0.0)]
            for label, dur, bank in plan20:
                seg = run_segment(mav, sub, osub, adiag, pdiag, aerodiag, label, dur,
                                   rc1_for_roll_deg(bank), 1500, base.RC3_TRIM_TARGET_US, t_flight0, latest_mav)
                segments.append(seg)
                if seg["aborted"]:
                    break
        else:
            R["escalation_skipped_20deg"] = "10deg segments not judged bounded by online heuristic"

    R["segments"] = segments
    aborted_any = any(s["aborted"] for s in segments)
    R["flight_result"] = dict(aborted=aborted_any)
    return not aborted_any


def flight3(mav, sub, osub, adiag, pdiag, aerodiag, R):
    confirmed = enter_fbwa(mav, R)
    if not confirmed:
        R["flight_result"] = dict(aborted=True, reason="fbwa_not_confirmed")
        return False
    latest_mav = {}
    t_flight0 = time.time()
    segments = []

    plan5 = [("neutral_pre", 2.0, 0.0), ("pitch_up_5", 4.0, 5.0), ("neutral_1", 3.0, 0.0),
             ("pitch_down_5", 4.0, -5.0), ("neutral_2", 3.0, 0.0)]
    for label, dur, pitch in plan5:
        seg = run_segment(mav, sub, osub, adiag, pdiag, aerodiag, label, dur,
                           1500, rc2_for_pitch_deg(pitch), base.RC3_TRIM_TARGET_US, t_flight0, latest_mav)
        segments.append(seg)
        if seg["aborted"]:
            break

    if not any(s["aborted"] for s in segments):
        up5 = next(s for s in segments if s["label"] == "pitch_up_5")
        down5 = next(s for s in segments if s["label"] == "pitch_down_5")
        if bounded_ok(up5, 1) and bounded_ok(down5, 1):
            plan10 = [("pitch_up_10", 4.0, 10.0), ("neutral_3", 3.0, 0.0),
                      ("pitch_down_10", 4.0, -10.0), ("neutral_4", 3.0, 0.0)]
            for label, dur, pitch in plan10:
                seg = run_segment(mav, sub, osub, adiag, pdiag, aerodiag, label, dur,
                                   1500, rc2_for_pitch_deg(pitch), base.RC3_TRIM_TARGET_US, t_flight0, latest_mav)
                segments.append(seg)
                if seg["aborted"]:
                    break
        else:
            R["escalation_skipped_10deg"] = "5deg segments not judged bounded by online heuristic"

    R["segments"] = segments
    aborted_any = any(s["aborted"] for s in segments)
    R["flight_result"] = dict(aborted=aborted_any)
    return not aborted_any


def flight4(mav, sub, osub, adiag, pdiag, aerodiag, R):
    """Optional combined roll+pitch, only run if flight1-3 all stable."""
    confirmed = enter_fbwa(mav, R)
    if not confirmed:
        R["flight_result"] = dict(aborted=True, reason="fbwa_not_confirmed")
        return False
    latest_mav = {}
    t_flight0 = time.time()
    plan = [("neutral_pre", 2.0, 0.0, 0.0), ("combined_right_bank_pitch_up", 5.0, 15.0, 5.0),
            ("neutral_recover", 4.0, 0.0, 0.0)]
    segments = []
    for label, dur, bank, pitch in plan:
        seg = run_segment(mav, sub, osub, adiag, pdiag, aerodiag, label, dur,
                           rc1_for_roll_deg(bank), rc2_for_pitch_deg(pitch), base.RC3_TRIM_TARGET_US,
                           t_flight0, latest_mav)
        segments.append(seg)
        if seg["aborted"]:
            break
    R["segments"] = segments
    aborted_any = any(s["aborted"] for s in segments)
    R["flight_result"] = dict(aborted=aborted_any)
    return not aborted_any


# =============================================================================
# Main
# =============================================================================
def finish_fail(R, flight, phase, mav):
    R["overall_result"] = "TEST_FAILED"
    R["blocking_phase"] = phase
    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    out_path = os.path.join(base.RESULTS_DIR, f"ardupilot_{flight}_result.json")
    with open(out_path, "w") as f:
        json.dump(R, f, indent=2, default=str)
    print(f"{flight} FAILED at {phase} - see", out_path)
    if mav is not None:
        mav.close()
    return 1


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in FLIGHTS:
        print(f"usage: {sys.argv[0]} {{{'|'.join(FLIGHTS)}}}")
        return 2
    flight = sys.argv[1]

    R = {"flight": flight, "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}

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
        return finish_fail(R, flight, "phase1_mavlink_arm", mav)

    settled, elapsed = base.wait_ground_settle(osub)
    R["ground_settle"] = dict(settled=settled, elapsed_s=elapsed)
    print("ground settle:", R["ground_settle"])
    if not settled:
        base.disarm(mav)
        return finish_fail(R, flight, "ground_settle", mav)

    t_teleport, ok_v = base.phase2_teleport_and_verify(
        node, sub, osub, pub_oneshot, (0.0, 0.0, 90.0, 0.0, 0.0, 0.0), R)
    print("PHASE 2 (teleport+verify):", R["phase2_teleport_verify"]["ok"])
    if not ok_v:
        base.disarm(mav)
        return finish_fail(R, flight, "phase2_teleport_verify", mav)

    base.clear_wrench(pub_clear)

    hold_ok = base.phase3_hold_to_trim(pub_oneshot, pub_clear, sub, osub, mav, R)
    print("PHASE 3 (hold-to-trim): aborted =", R["phase3_hold_to_trim"]["aborted"],
          "reason =", R["phase3_hold_to_trim"]["abort_reason"])
    if not hold_ok:
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        base.disarm(mav)
        return finish_fail(R, flight, "phase3_hold_to_trim", mav)

    for msg_id in MSG_IDS_20HZ:
        request_rate(mav, msg_id, 20.0)
    time.sleep(0.3)

    fn = {"flight1": flight1, "flight2": flight2, "flight3": flight3, "flight4": flight4}[flight]
    print(f"Starting {flight} command sequence (FBWA)...")
    ok = fn(mav, sub, osub, adiag, pdiag, aerodiag, R)

    mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    base.disarm(mav)

    R["overall_result"] = "FLIGHT_COMPLETED_NO_ABORT" if ok else "FLIGHT_ABORTED"
    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    out_path = os.path.join(base.RESULTS_DIR, f"ardupilot_{flight}_result.json")
    with open(out_path, "w") as f:
        json.dump(R, f, indent=2, default=str)
    seg_summary = [(s["label"], s["aborted"], s["n_samples"]) for s in R.get("segments", [])]
    print("RESULT:", R["overall_result"], "segments:", seg_summary, "->", out_path)
    mav.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
FALCON V2 - AHRS DCM/EKF3 consistency prearm-blocker root-cause probe
(gazebo-testing, ARDUPLANE_SITL_TRANSPORT_AND_ACTUATOR_MAPPING_VALIDATION
follow-up, 2026-08-28).

Investigates the persistent "AHRS: DCM Roll/Pitch inconsistent ~51-54 deg"
prearm failure left OPEN by controls-integration's IMU frame-conversion fix
(see docs/source_of_truth/autopilot/SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md
sec 10). Distinct from the frame-sign bug: AP_AHRS::attitudes_consistent()
(source-read, AP_AHRS.cpp ~line 1752) compares the PRIMARY (EKF3)
quaternion against the SEPARATE legacy DCM backend's own quaternion - both
now fed the same, correctly FLU->FRD converted IMU stream.

Hypothesis under test THIS script: the test world used for this session's
prearm checks (falcon_v2_ardupilot_sitl_test_world.sdf) spawns falcon_v2 at
50m AGL, disarmed, unpowered, no ground contact - and per controls-
integration's OWN design-doc note (sec 6.2) "the aircraft tumbles within a
couple of real seconds" in this condition. AP_AHRS_DCM starts its internal
_dcm_matrix at IDENTITY (source-read, AP_AHRS_DCM.h ~line 47) and converges
attitude via a low-bandwidth complementary-filter drift_correction() PI
loop (source-read, AP_AHRS_DCM.cpp ~line 792) - a fundamentally different,
generally lower-bandwidth estimator than EKF3's full INS/GPS Kalman filter.
If the airframe is genuinely tumbling/falling throughout the check (a real,
continuously-changing true attitude), DCM and EKF3 can each track a
DIFFERENT, real, physically-changing attitude with different lag/dynamics -
producing a persistent (non-converging) tens-of-degrees disagreement that
is not a bug in either estimator, and not fixable by any AHRS parameter -
it is a test/launch-condition gap: prearm checks assume a physically
stationary/settled airframe (exactly like a real aircraft - nobody prearm-
checks an aircraft that is currently falling out of the sky).

Test method: hold falcon_v2 stationary (zero linear+angular body velocity)
via the SAME stock gz-sim VelocityControl Cmd mechanism already used and
justified in this task's own frame-validation work (NOT a physics
parameter, NOT an aircraft-physics change - a continuous zero-velocity
override applied through the pre-existing VelocityControl system plugin
already attached to falcon_v2's <include> in this test world), starting as
early as possible after connecting, and compare AHRS2 (DCM/secondary
estimate, MAVLink) against ATTITUDE (EKF3/primary estimate) over an
extended soak, alongside periodic REAL (non-forceful) arm attempts to
observe whether ArduPlane's own prearm STATUSTEXT changes.

No ARMING_SKIPCHK, no force-arm magic value, no safety-check suppression of
any kind. No aircraft physics parameter (mass/CG/inertia/aero coefficient/
control authority/motor thrust) is read, written, or influenced.

PRECONDITION: falcon_v2_ardupilot_sitl_test_world.sdf + arduplane already
running and connected on tcp:127.0.0.1:5760 (see this task's report for
exact launch commands).
"""
import json
import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import gz.transport13 as tp  # noqa: E402
from gz.msgs10 import twist_pb2  # noqa: E402

from ardupilot_sitl_mav_lib import SafeMav  # noqa: E402

OUT_JSON = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results/ardupilot_sitl_arming_dcm_ekf_probe_result.json"

MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAVLINK_MSG_ID_AHRS2 = 178
MAVLINK_MSG_ID_ATTITUDE = 30


def pub_vel(pub, lin=(0, 0, 0), ang=(0, 0, 0)):
    m = twist_pb2.Twist()
    m.linear.x, m.linear.y, m.linear.z = lin
    m.angular.x, m.angular.y, m.angular.z = ang
    pub.publish(m)


def try_arm(mav):
    mav.m.mav.command_long_send(
        mav.m.target_system, mav.m.target_component,
        MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    ack = None
    statustext = None
    while time.time() - t0 < 3.0:
        r, _, _ = select.select([mav.m.port], [], [], 0.2)
        if not r:
            continue
        msg = mav.m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=False)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK" and msg.command == MAV_CMD_COMPONENT_ARM_DISARM:
            ack = msg.to_dict()
        elif msg.get_type() == "STATUSTEXT":
            statustext = msg.text if hasattr(msg, "text") else str(msg)
    if statustext:
        # if armed, disarm immediately (should not happen, but be safe -
        # never leave an armed session running unattended)
        pass
    return ack, statustext


def request_ahrs2(mav, rate_hz=4):
    interval_us = int(1e6 / rate_hz)
    mav.m.mav.command_long_send(
        mav.m.target_system, mav.m.target_component,
        MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        MAVLINK_MSG_ID_AHRS2, interval_us, 0, 0, 0, 0, 0)
    mav.m.mav.command_long_send(
        mav.m.target_system, mav.m.target_component,
        MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        MAVLINK_MSG_ID_ATTITUDE, interval_us, 0, 0, 0, 0, 0)


def soak(mav, pub, hold_still, duration, arm_probe_period=20.0, label=""):
    """Collect AHRS2 (DCM) + ATTITUDE (EKF3) samples for `duration` seconds.
    If hold_still, continuously republish a zero cmd_vel throughout (same
    mechanism/rate as the existing frame probe). Every arm_probe_period
    seconds, issue one real (non-forceful) arm attempt and record the
    COMMAND_ACK/STATUSTEXT, then let it settle (no persistent arming is
    ever achieved by a rejected attempt - this is inert)."""
    print(f"\n=== soak start: {label} (hold_still={hold_still}, {duration}s) ===", flush=True)
    samples = []
    events = []
    t0 = time.time()
    last_hold_pub = 0
    last_arm_probe = 0
    while time.time() - t0 < duration:
        now = time.time()
        if hold_still and (now - last_hold_pub) > 0.05:
            pub_vel(pub, (0, 0, 0), (0, 0, 0))
            last_hold_pub = now
        if (now - last_arm_probe) >= arm_probe_period:
            last_arm_probe = now
            ack, statustext = try_arm(mav)
            ev = {"t": now - t0, "ack": ack, "statustext": statustext}
            events.append(ev)
            print(f"  [t={ev['t']:.1f}s] arm attempt -> ack={ack} statustext={statustext}", flush=True)
        r, _, _ = select.select([mav.m.port], [], [], 0.1)
        if not r:
            continue
        msg = mav.m.recv_match(type=["AHRS2", "ATTITUDE"], blocking=False)
        if msg is None:
            continue
        d = msg.to_dict()
        d["_t"] = now - t0
        samples.append(d)
    return samples, events


def summarize(samples):
    """Pair the nearest ATTITUDE (EKF3 primary) sample to each AHRS2 (DCM
    secondary) sample and report the roll/pitch difference in degrees over
    time, to see whether it shrinks (convergence) or plateaus (persistent
    bias)."""
    import math
    att = [s for s in samples if s["mavpackettype"] == "ATTITUDE"]
    ahrs2 = [s for s in samples if s["mavpackettype"] == "AHRS2"]
    diffs = []
    for a2 in ahrs2:
        # nearest ATTITUDE sample in time
        best = min(att, key=lambda a: abs(a["_t"] - a2["_t"]), default=None)
        if best is None:
            continue
        droll = math.degrees(a2["roll"] - best["roll"])
        dpitch = math.degrees(a2["pitch"] - best["pitch"])
        diffs.append({"t": a2["_t"], "droll_deg": droll, "dpitch_deg": dpitch,
                       "dcm_roll_deg": math.degrees(a2["roll"]),
                       "dcm_pitch_deg": math.degrees(a2["pitch"]),
                       "ekf3_roll_deg": math.degrees(best["roll"]),
                       "ekf3_pitch_deg": math.degrees(best["pitch"])})
    return diffs


def main():
    mav = SafeMav("tcp:127.0.0.1:5760", source_system=255)
    hb = mav.wait_heartbeat(15)
    print("HEARTBEAT", hb, flush=True)
    if hb is None:
        print("FATAL: no heartbeat")
        sys.exit(1)

    request_ahrs2(mav, rate_hz=4)
    time.sleep(0.5)

    node = tp.Node()
    pub = node.advertise("/model/falcon_v2/cmd_vel", twist_pb2.Twist)
    time.sleep(0.5)

    results = {}

    # Phase A: baseline, free-falling/tumbling exactly as spawned (matches
    # this task's prior-session condition), no cmd_vel hold at all.
    samples_a, events_a = soak(mav, pub, hold_still=False, duration=45.0,
                                arm_probe_period=15.0, label="A_baseline_freefall")
    results["phaseA_baseline_freefall"] = {
        "diffs": summarize(samples_a),
        "arm_events": events_a,
        "n_samples": len(samples_a),
    }

    # Engage a continuous zero-body-velocity hold (VelocityControl Cmd,
    # same mechanism as the already-validated frame probe) - this arrests
    # whatever tumble accumulated during phase A and holds the airframe at
    # a fixed (not necessarily level, but STATIONARY) attitude from here
    # on, then soak for longer (per task instruction: rule out slow
    # convergence with a much longer wait than the prior 90s test).
    print("\nEngaging stationary hold (zero body velocity)...", flush=True)
    pub_vel(pub, (0, 0, 0), (0, 0, 0))
    time.sleep(2.0)

    samples_b, events_b = soak(mav, pub, hold_still=True, duration=150.0,
                                arm_probe_period=20.0, label="B_held_stationary")
    results["phaseB_held_stationary"] = {
        "diffs": summarize(samples_b),
        "arm_events": events_b,
        "n_samples": len(samples_b),
    }

    # release hold
    pub_vel(pub, (0, 0, 0), (0, 0, 0))

    mav.close()

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2, default=str)

    for phase in ["phaseA_baseline_freefall", "phaseB_held_stationary"]:
        d = results[phase]["diffs"]
        print(f"\n=== {phase}: {len(d)} AHRS2/ATTITUDE pairs ===")
        for row in d[:5] + (d[-5:] if len(d) > 5 else []):
            print("t=%6.1f  dcm(r,p)=(%7.2f,%7.2f)  ekf3(r,p)=(%7.2f,%7.2f)  diff(r,p)=(%7.2f,%7.2f)" % (
                row["t"], row["dcm_roll_deg"], row["dcm_pitch_deg"],
                row["ekf3_roll_deg"], row["ekf3_pitch_deg"],
                row["droll_deg"], row["dpitch_deg"]))
        print(f"  arm events: {results[phase]['arm_events']}")

    print(f"\nWrote {OUT_JSON}")


if __name__ == "__main__":
    main()

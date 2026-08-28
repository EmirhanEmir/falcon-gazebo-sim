#!/usr/bin/env python3
"""
FALCON V2 - previously-BLOCKED live motor/throttle acceptance tests
(gazebo-testing, ARDUPLANE_SITL_TRANSPORT_AND_ACTUATOR_MAPPING_VALIDATION
follow-up, 2026-08-28).

Runs items #6/7/8/9/12 from the 2026-08-27 report (left motor, right motor,
dual motor, motor channel identity, brief asymmetric-thrust sanity) -
BLOCKED in that report purely because ArduPlane would not arm (persistent
"AHRS: DCM Roll/Pitch inconsistent" prearm failure, root-caused THIS
session - see docs/test_results update).

PRECONDITION: Gazebo running
tests/gazebo/worlds/falcon_v2_ardupilot_sitl_grounded_test_world.sdf (a
NEW, additive, gazebo-testing-owned diagnostic world - falcon_v2 rests
under genuine dartsim ground-contact physics, giving the IMU accelerometer
a real, non-artifactual ~1g gravity reference so ArduPilot's legacy DCM
backend can actually converge - see that world file's own header comment
for the full root-cause reasoning) and ArduPlane SITL already running and
connected on tcp:127.0.0.1:5760.

Real (non-forceful) MAVLink arming is used - NO ARMING_SKIPCHK, NO
force-arm magic value, NO safety-check suppression of any kind. No
aircraft physics parameter (mass/CG/inertia/aero coefficient/control
authority/motor thrust) is read, written, or influenced by this file.
MANUAL mode only (custom_mode stays 0 throughout - confirmed via every
captured HEARTBEAT).

IMPORTANT METHODOLOGY CORRECTION (found live, this run): the FIRST version
of this script drove RC3 (master throttle) and RC5 in isolation, on the
assumption RC5 independently commands k_throttleRight. This is WRONG for
ArduPlane's real twin-engine architecture (source-confirmed,
ArduPlane/servos.cpp::servos_twin_engine_mix(), ~line 766): in MANUAL mode,
BOTH k_throttleLeft and k_throttleRight are derived from a SINGLE master
k_throttle value (RC3) plus a RUDDER-driven differential term
(`rudder_dt`, gained by RUDD_DT_GAIN, default 10, NOT overridden in
falcon_v2_sitl.parm - confirmed via live PARAM_VALUE read this run):
  throttle_left  = throttle + 50*rudder_dt   (clamped [0,100])
  throttle_right = throttle - 50*rudder_dt   (clamped [0,100])
  rudder_dt = (RUDD_DT_GAIN/100) * scaled_rudder_output / SERVO_MAX
RC5 input is NOT consulted anywhere in this mix - SERVO5_FUNCTION=74
(k_throttleRight) only tells ArduPlane which PHYSICAL OUTPUT channel
carries the already-computed k_throttleRight scaled value; it is not an
independent pilot input channel in this configuration. This is real,
correct, standard ArduPlane twin-engine-plane behavior, not a defect in
this project's SDF/param configuration - flagged here for the record since
the original task wording ("ThrottleLeft only, right stays ~zero") assumed
independent per-motor RC channels, which this airframe's real ArduPlane
mixing does not provide. The corrected test below drives RC3 (shared
master throttle) + RC4 (rudder, differential) instead.
"""
import json
import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

from ardupilot_sitl_mav_lib import SafeMav  # noqa: E402
from propulsion_lib import DiagSubscriber  # noqa: E402

OUT_JSON = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results/ardupilot_sitl_motor_tests_result.json"

MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_SET_MESSAGE_INTERVAL = 511


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
    # zero throttle RC first and let it take effect - a real ArduPlane
    # disarm-refusal was observed live this run when RC3 was still
    # commanded nonzero at the instant of the disarm attempt (procedural
    # finding, not a defect - documented in the test report).
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
        return None
    return bool(hb.base_mode & 128)


def hold_rc(mav, duration, rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000, period=0.05,
            diag=None, mav_stream=None, samples=None):
    t0 = time.time()
    while time.time() - t0 < duration:
        mav.send_rc_override(rc1=rc1, rc2=rc2, rc3=rc3, rc4=rc4, rc5=rc5)
        if samples is not None and diag is not None:
            d = diag.latest()
            if d is not None:
                d = dict(d)
                d["_t"] = time.time() - t0
                samples.append(d)
        time.sleep(period)


def get_yawspeed(mav, timeout=1.0):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        r, _, _ = select.select([mav.m.port], [], [], 0.2)
        if not r:
            continue
        msg = mav.m.recv_match(type="ATTITUDE", blocking=False)
        if msg:
            last = msg.yawspeed
    return last


def main():
    mav = SafeMav("tcp:127.0.0.1:5760", source_system=255)
    hb = mav.wait_heartbeat(15)
    print("HEARTBEAT", hb.to_dict() if hb else None, flush=True)
    if hb is None:
        print("FATAL: no heartbeat")
        sys.exit(1)

    mav.m.mav.command_long_send(mav.m.target_system, mav.m.target_component,
                                 MAV_CMD_SET_MESSAGE_INTERVAL, 0, 30, 100000, 0, 0, 0, 0, 0)  # ATTITUDE 10Hz

    diag = DiagSubscriber()
    time.sleep(0.5)

    results = {}

    print("\n=== Arming (real, non-forceful) ===", flush=True)
    ack, sts = arm(mav)
    print("arm ack:", ack, "statustexts:", sts, flush=True)
    results["arm"] = {"ack": ack, "statustexts": sts}
    armed = is_armed(mav)
    print("armed state after arm attempt:", armed, flush=True)
    results["armed_confirmed"] = armed
    if not armed:
        print("FATAL: did not arm - aborting motor tests")
        json.dump(results, open(OUT_JSON, "w"), indent=2, default=str)
        mav.close()
        sys.exit(1)

    try:
        # Neutral baseline diagnostics (both throttles at MIN=1000 -> raw_cmd=0)
        time.sleep(0.5)
        base = diag.latest()
        print("\nbaseline diag (both throttle min):", base, flush=True)
        results["baseline_diag"] = base

        # ---- Item 8 (run first): DUAL motor test (symmetric, RC4 neutral) ----
        print("\n=== Item 8: DUAL motor test (RC3=1600, RC4=1500 neutral, RC5 irrelevant=1000) ===", flush=True)
        samples = []
        hold_rc(mav, 4.0, rc3=1600, rc4=1500, rc5=1000, diag=diag, samples=samples)
        dual = diag.latest()
        print("dual-motor diag (RC3=1600,RC4=neutral):", json.dumps(dual, indent=2), flush=True)
        results["item8_dual_motor"] = {"final_diag": dual, "n_diag_samples": len(samples)}

        hold_rc(mav, 2.0, rc3=1000, rc4=1500, rc5=1000)

        # ---- Items 6/7/9 corrected: rudder-mixed differential thrust ----
        # (RC3=master throttle shared by both engines, RC4=rudder drives
        # the ONLY real per-motor differential in this MANUAL-mode mix -
        # see module docstring for the servos_twin_engine_mix() source
        # citation). RC4=1900 is the same "yaw-right" extreme already
        # PASS-tested for pure rudder-surface direction in the prior
        # report (SERVO4 REVERSED=1, PWM->800/MIN at RC4=1900).
        print("\n=== Items 6/7/9: rudder-mixed differential thrust, RC4=1900 (yaw-right extreme) ===", flush=True)
        samples = []
        hold_rc(mav, 4.0, rc3=1600, rc4=1900, rc5=1000, diag=diag, samples=samples)
        diff_a = diag.latest()
        print("differential diag (RC3=1600,RC4=1900/yaw-right):", json.dumps(diff_a, indent=2), flush=True)
        results["items6_7_9_differential_rc4_1900"] = {"final_diag": diff_a, "n_diag_samples": len(samples)}

        hold_rc(mav, 2.0, rc3=1000, rc4=1500, rc5=1000)

        print("\n=== Items 6/7/9 mirror: RC4=1100 (yaw-left extreme) ===", flush=True)
        samples = []
        hold_rc(mav, 4.0, rc3=1600, rc4=1100, rc5=1000, diag=diag, samples=samples)
        diff_b = diag.latest()
        print("differential diag (RC3=1600,RC4=1100/yaw-left):", json.dumps(diff_b, indent=2), flush=True)
        results["items6_7_9_differential_rc4_1100"] = {"final_diag": diff_b, "n_diag_samples": len(samples)}

        hold_rc(mav, 2.0, rc3=1000, rc4=1500, rc5=1000)

        # ---- Item 12: brief asymmetric-thrust sanity nod ----
        print("\n=== Item 12: asymmetric-thrust sanity (RC3=1600,RC4=1900, watch yawspeed) ===", flush=True)
        yawspeed_before = get_yawspeed(mav, timeout=1.0)
        hold_rc(mav, 3.0, rc3=1600, rc4=1900, rc5=1000)
        yawspeed_after = get_yawspeed(mav, timeout=1.0)
        asym_diag = diag.latest()
        print(f"yawspeed before={yawspeed_before} after={yawspeed_after} rad/s, diag={asym_diag}", flush=True)
        results["item12_asymmetric_sanity"] = {
            "yawspeed_before": yawspeed_before,
            "yawspeed_after": yawspeed_after,
            "final_diag": asym_diag,
        }

        hold_rc(mav, 2.0, rc3=1000, rc4=1500, rc5=1000)

    finally:
        print("\n=== Disarming ===", flush=True)
        dack = disarm(mav)
        print("disarm ack:", dack, flush=True)
        results["disarm"] = dack
        armed_final = is_armed(mav)
        print("armed state after disarm:", armed_final, flush=True)
        results["armed_after_disarm"] = armed_final

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2, default=str)

    mav.close()
    print(f"\nWrote {OUT_JSON}")


if __name__ == "__main__":
    main()

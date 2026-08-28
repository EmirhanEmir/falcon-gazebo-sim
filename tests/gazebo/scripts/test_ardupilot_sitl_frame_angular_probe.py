#!/usr/bin/env python3
"""
FALCON V2 - focused 3-axis IMU angular-rate frame-conversion probe
(gazebo-testing, ARDUPLANE_SITL_TRANSPORT_AND_ACTUATOR_MAPPING_VALIDATION,
2026-08-27).

Follow-up to test_ardupilot_sitl_transport_actuator_mapping.py's own frame
validation phase (which confirmed the position/velocity/orientation leg of
ArduPilotPlugin's frame conversion is correct, but left an anomaly in the
angular-rate leg worth isolating cleanly). Commands a pure body-frame FLU
angular rate on ONE axis at a time (via the same VelocityControl/cmd_vel
mechanism, body-frame Cmd component) from a clean, freshly-reset near-
identity pose (one `set_pose` service call used only as test SETUP, not as
the measurement method itself), then reads RAW_IMU/ATTITUDE for 4s per
axis. See docs/test_results/2026-08-27_ardupilot_sitl_transport_actuator_
mapping_validation.md sec 5.2 for the full analysis and root-cause finding
(ArduPilotPlugin.cc's CreateStateJSON() never applies
modelXYZToAirplaneXForwardZDown to gyro/accel data - source-confirmed).

PRECONDITION: same as the main test script - Gazebo (falcon_v2_ardupilot_
sitl_test_world.sdf) and ArduPlane SITL already running and connected on
tcp:127.0.0.1:5760.

No aircraft physics parameter is read, written, or influenced by this file.
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

OUT_JSON = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results/ardupilot_sitl_frame_angular_probe_result.json"


def pub_vel(pub, lin, ang):
    m = twist_pb2.Twist()
    m.linear.x, m.linear.y, m.linear.z = lin
    m.angular.x, m.angular.y, m.angular.z = ang
    pub.publish(m)


def run_axis(mav, pub, name, ang, duration=4.0):
    print(f"\n--- axis test: {name}, angular={ang} ---", flush=True)
    pub_vel(pub, (0, 0, 0), (0, 0, 0))
    time.sleep(1.5)
    pub_vel(pub, (0, 0, 0), ang)
    t0 = time.time()
    samples = []
    while time.time() - t0 < duration:
        r, _, _ = select.select([mav.m.port], [], [], 0.1)
        if not r:
            continue
        msg = mav.m.recv_match(type=["ATTITUDE", "RAW_IMU"], blocking=False)
        if msg is None:
            continue
        d = msg.to_dict()
        d["_t"] = time.time() - t0
        samples.append(d)
    pub_vel(pub, (0, 0, 0), (0, 0, 0))
    time.sleep(1.0)
    return samples


def main():
    mav = SafeMav("tcp:127.0.0.1:5760", source_system=255)
    hb = mav.wait_heartbeat(15)
    print("HEARTBEAT", hb, flush=True)

    node = tp.Node()
    pub = node.advertise("/model/falcon_v2/cmd_vel", twist_pb2.Twist)
    time.sleep(0.5)

    results = {}
    results["roll_x_plus"] = run_axis(mav, pub, "body +X (roll) 0.5 rad/s", (0.5, 0, 0))
    results["pitch_y_plus"] = run_axis(mav, pub, "body +Y (pitch) 0.5 rad/s", (0, 0.5, 0))
    results["yaw_z_plus"] = run_axis(mav, pub, "body +Z (yaw) 0.5 rad/s", (0, 0, 0.5))

    mav.close()

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2, default=str)

    for axis, samples in results.items():
        print(f"\n=== {axis} ===")
        att = [s for s in samples if s["mavpackettype"] == "ATTITUDE"]
        imu = [s for s in samples if s["mavpackettype"] == "RAW_IMU"]
        for a in att[:3] + att[-3:]:
            print("ATT t=%.2f roll=%.4f pitch=%.4f yaw=%.4f rs=%.4f ps=%.4f ys=%.4f" % (
                a["_t"], a["roll"], a["pitch"], a["yaw"],
                a["rollspeed"], a["pitchspeed"], a["yawspeed"]))
        for g in imu[:3] + imu[-3:]:
            print("IMU t=%.2f xgyro=%s ygyro=%s zgyro=%s" % (
                g["_t"], g["xgyro"], g["ygyro"], g["zgyro"]))

    print(f"\nWrote {OUT_JSON}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
FALCON V2 - POWERED_FREE_FLIGHT_SMOKE_TEST GUI reproduction, runtime driver
(gazebo-testing, 2026-08-23).

Companion to tests/gazebo/worlds/falcon_v2_powered_free_flight_gui_world.sdf
and tests/gazebo/scripts/run_powered_free_flight_gui.sh. Read that world
file's header comment first - it explains why this script does NOT need to
run a hold_step()-style body-velocity convergence controller (the world
file's own <state><link><velocity> element already gives base_link its
exact initial body velocity at spawn, with zero transient/error) and does
NOT need a one-time teleport call (the world file's <include><pose> already
places the model at the target altitude).

This script's only job, once the live `gz sim -r` server named by that
world file is up, is to act as an ordinary gz-transport CLIENT against it
(it does NOT load any System plugin itself, so it does not need
GZ_SIM_SYSTEM_PLUGIN_PATH - only PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=
python, via propulsion_lib.setup_env(), for the same protobuf-version
reason documented in that module):

  1. Continuously publish left/right throttle = 0.50/0.50 to the exact
     topics FalconV2Propulsion already subscribes to (model/model.sdf,
     unmodified) - reusing propulsion_lib.ThrottleCommander AS-IS (same
     "republish every tick, never a one-shot publish" technique already
     established and documented there, since gz-transport topics are not
     latched).
  2. Continuously publish the 5 control-surface target angles to the 5
     JointPositionController topics the world file attaches (servo-hold
     representation only - see that file's header comment). Same
     repeated-publish rationale as (1).
  3. Print a short startup info block (initial V, altitude, elevator angle,
     throttle L/R).
  4. Optionally print ~1 Hz telemetry (time, airspeed, altitude,
     roll/pitch/yaw, left/right RPM, left/right thrust) by subscribing to
     the aerodynamics/propulsion diagnostics topics (aero_lib.DiagSubscriber
     / propulsion_lib.DiagSubscriber, reused as-is) plus a small local
     Pose_V subscriber for altitude/attitude (gz-transport's standard world
     pose topic - no new infrastructure invented, this is the same topic
     already used by the GUI's own 3D view).

No aircraft physics parameter is read for any purpose other than
display/logging in this script, and none is modified anywhere in this file
or by any message this script publishes (throttle and control-surface
*commands* are control INPUTS, exactly like a pilot's stick/throttle -
not physics parameters).
"""
import math
import os
import signal
import sys
import threading
import time

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
SCRIPTS_DIR = os.path.join(REPO_ROOT, "tests/gazebo/scripts")
sys.path.insert(0, SCRIPTS_DIR)

import aero_lib  # noqa: E402
import propulsion_lib  # noqa: E402

# ---------------------------------------------------------------------------
# Reference scenario constants - bit-for-bit identical to the values already
# validated by tests/gazebo/scripts/test_trim_benchmarks.py's Benchmark B and
# by the (deleted-per-its-own-task-requirement) original smoke test driver.
# Not re-derived here.
# ---------------------------------------------------------------------------
U_TARGET = 18.14534   # m/s, body x (forward)
V_TARGET = 0.0        # m/s, body y (left)
W_TARGET = -0.78335   # m/s, body z (up)
ALTITUDE_M = 85.0
ELEVATOR_THETA_RAD = 0.13962634  # +8.0 deg physical joint angle
THROTTLE_LEFT = 0.50
THROTTLE_RIGHT = 0.50

CONTROL_SURFACE_TOPICS = {
    "left_aileron_joint": "/model/falcon_v2/control_surface/left_aileron/cmd_pos",
    "right_aileron_joint": "/model/falcon_v2/control_surface/right_aileron/cmd_pos",
    "left_elevator_joint": "/model/falcon_v2/control_surface/left_elevator/cmd_pos",
    "right_elevator_joint": "/model/falcon_v2/control_surface/right_elevator/cmd_pos",
    "rudder_joint": "/model/falcon_v2/control_surface/rudder/cmd_pos",
}
CONTROL_SURFACE_TARGETS = {
    "left_aileron_joint": 0.0,
    "right_aileron_joint": 0.0,
    "left_elevator_joint": ELEVATOR_THETA_RAD,
    "right_elevator_joint": ELEVATOR_THETA_RAD,
    "rudder_joint": 0.0,
}

WORLD_NAME = "falcon_v2_powered_free_flight_gui"
WORLD_POSE_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"

PUBLISH_HZ = 20.0
TELEMETRY_HZ = 1.0


class ControlSurfaceCommander:
    """Publishes gz.msgs.Double target-position commands to each control
    surface's JointPositionController topic every tick - mirrors
    propulsion_lib.ThrottleCommander's own repeated-publish pattern exactly
    (same module, same rationale, not duplicated logic, just applied to a
    different topic set)."""

    def __init__(self, topics):
        import gz.transport13 as tp
        from gz.msgs10 import double_pb2
        self._double_pb2 = double_pb2
        self.node = tp.Node()
        self.pubs = {jn: self.node.advertise(t, double_pb2.Double)
                     for jn, t in topics.items()}

    def tick(self, targets):
        for jn, pub in self.pubs.items():
            m = self._double_pb2.Double()
            m.data = targets[jn]
            pub.publish(m)


class WorldPoseSubscriber:
    """Minimal Pose_V subscriber for falcon_v2's world pose (altitude +
    orientation), used only for telemetry display. Same raw subscribe_raw +
    manual-parse pattern aero_lib.DiagSubscriber / propulsion_lib.
    DiagSubscriber already use for their own diagnostics topics."""

    def __init__(self, topic):
        import gz.transport13 as tp
        self._tp = tp
        self.node = tp.Node()
        self.lock = threading.Lock()
        self.latest_pose = None
        ok = self.node.subscribe_raw(
            topic, self._cb, "gz.msgs.Pose_V", tp.SubscribeOptions())
        if not ok:
            raise RuntimeError(f"Failed to subscribe to {topic}")

    def _cb(self, data, info):
        from gz.msgs10 import pose_v_pb2
        m = pose_v_pb2.Pose_V()
        m.ParseFromString(data)
        for p in m.pose:
            if p.name == "falcon_v2":
                with self.lock:
                    self.latest_pose = dict(
                        x=p.position.x, y=p.position.y, z=p.position.z,
                        qx=p.orientation.x, qy=p.orientation.y,
                        qz=p.orientation.z, qw=p.orientation.w)
                break

    def latest(self):
        with self.lock:
            return dict(self.latest_pose) if self.latest_pose else None


def quat_to_euler_rpy(qx, qy, qz, qw):
    """Standard quaternion -> roll/pitch/yaw (radians), body FLU / world
    convention consistent with the rest of this repo. Pure display-only
    math, not used to drive anything."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def print_startup_info():
    v_target = math.sqrt(U_TARGET ** 2 + V_TARGET ** 2 + W_TARGET ** 2)
    alpha_deg = math.degrees(math.atan2(-W_TARGET, U_TARGET))
    print("=" * 70)
    print("FALCON V2 - POWERED_FREE_FLIGHT_SMOKE_TEST (GUI reproduction)")
    print("=" * 70)
    print(f"  Initial altitude:      {ALTITUDE_M:.1f} m")
    print(f"  Initial body velocity: u={U_TARGET:.5f} v={V_TARGET:.5f} "
          f"w={W_TARGET:.5f} m/s  (V={v_target:.3f} m/s, alpha={alpha_deg:.3f} deg)")
    print(f"  Elevators (L=R):       +{math.degrees(ELEVATOR_THETA_RAD):.2f} deg "
          f"physical joint angle ({ELEVATOR_THETA_RAD:.8f} rad)")
    print("  Ailerons:               0.0 deg (neutral)")
    print("  Rudder:                 0.0 deg (neutral)")
    print(f"  Throttle L/R:           {THROTTLE_LEFT:.2f} / {THROTTLE_RIGHT:.2f}")
    print("  Gravity: ON | Aerodynamics plugin: ON | Propulsion plugin: ON")
    print("  No velocity hold / P-controller / stabilization is ever applied")
    print("  to base_link - genuinely free 6-DOF flight from spawn onward.")
    print("=" * 70)
    print("Commanding throttle + control-surface holds continuously "
          "(Ctrl+C here or close the GUI window to stop).")
    print()


def main():
    propulsion_lib.setup_env()  # PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

    print_startup_info()

    throttle = propulsion_lib.ThrottleCommander()
    surfaces = ControlSurfaceCommander(CONTROL_SURFACE_TOPICS)
    throttle.set(left=THROTTLE_LEFT, right=THROTTLE_RIGHT)

    aero_diag = None
    prop_diag = None
    world_pose = None
    try:
        aero_diag = aero_lib.DiagSubscriber()
    except Exception as exc:  # pragma: no cover - telemetry is best-effort
        print(f"[telemetry] aero diagnostics subscription unavailable: {exc}")
    try:
        prop_diag = propulsion_lib.DiagSubscriber()
    except Exception as exc:  # pragma: no cover
        print(f"[telemetry] propulsion diagnostics subscription unavailable: {exc}")
    try:
        world_pose = WorldPoseSubscriber(WORLD_POSE_TOPIC)
    except Exception as exc:  # pragma: no cover
        print(f"[telemetry] world pose subscription unavailable: {exc}")

    stop = threading.Event()

    def handle_sigint(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    t_start = time.monotonic()
    period = 1.0 / PUBLISH_HZ
    next_telemetry = t_start + (1.0 / TELEMETRY_HZ)

    while not stop.is_set():
        throttle.tick()
        surfaces.tick(CONTROL_SURFACE_TARGETS)

        now = time.monotonic()
        if now >= next_telemetry:
            next_telemetry += 1.0 / TELEMETRY_HZ
            t_elapsed = now - t_start
            aero = aero_diag.latest() if aero_diag else None
            prop = prop_diag.latest() if prop_diag else None
            pose = world_pose.latest() if world_pose else None

            parts = [f"t={t_elapsed:6.1f}s"]
            if aero:
                parts.append(f"V={aero['V']:.2f}m/s")
                parts.append(f"alpha={math.degrees(aero['alpha']):.2f}deg")
            if pose:
                roll, pitch, yaw = quat_to_euler_rpy(
                    pose["qx"], pose["qy"], pose["qz"], pose["qw"])
                parts.append(f"alt={pose['z']:.1f}m")
                parts.append(
                    f"rpy=({math.degrees(roll):.1f},"
                    f"{math.degrees(pitch):.1f},{math.degrees(yaw):.1f})deg")
            if prop:
                parts.append(f"L_rpm={prop['left']['rpm']:.0f}")
                parts.append(f"R_rpm={prop['right']['rpm']:.0f}")
                parts.append(f"L_T={prop['left']['thrust_N']:.2f}N")
                parts.append(f"R_T={prop['right']['thrust_N']:.2f}N")
            if len(parts) > 1:
                print("  " + " | ".join(parts))

        stop.wait(period)

    print("\n[powered_free_flight_gui_setup] stopped (control publishing "
          "halted; GUI/server left running until closed).")


if __name__ == "__main__":
    main()

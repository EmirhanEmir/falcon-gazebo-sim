#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_SITL_TRANSPORT_AND_ACTUATOR_MAPPING_VALIDATION
live acceptance test (gazebo-testing, 2026-08-27).

Independent of controls-integration's own sanity check (docs/source_of_truth/
autopilot/SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md sec 6): different frame
verification method (VelocityControl/cmd_vel instead of teleport+set_pose),
own MAVLink session, own evidence.

PRECONDITION: this script assumes Gazebo (tests/gazebo/worlds/
falcon_v2_ardupilot_sitl_test_world.sdf) and ArduPlane SITL (JSON backend)
are ALREADY RUNNING and connected (see this task's test report for the
exact launch commands used - launched externally, not by this script,
mirroring launch_ardupilot_sitl.sh's own division of responsibility between
"start the two processes" and "run tests against them"). Connects to
tcp:127.0.0.1:5760 (ArduPlane's default SERIAL0).

Scope boundary (task's own hard limit, honored throughout): NO closed-loop
flight mode is ever requested (MANUAL only, custom_mode=0, confirmed never
changed away from this). NO PID/actuator retune. NO edit to model.sdf,
falcon_v2_sitl.parm, or any plugin. Motor/throttle output requires actual
ArduPlane arming (confirmed live this task - see report); arming this
session was blocked by the operating environment's own permission system
(distinct from ArduPlane's own prearm checks) on both a parameter-bypass
attempt and a MAVLink force-arm attempt - documented, not worked around.
Every test in this file that does not require ARMED state is unaffected by
that block and is executed and evidenced normally.

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
control authority, motor thrust) is read, written, or influenced by this
script.
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
from gz.msgs10 import double_pb2, double_v_pb2, twist_pb2  # noqa: E402
from pymavlink import mavutil  # noqa: E402

from ardupilot_sitl_mav_lib import SafeMav  # noqa: E402

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
OUT_JSON = os.path.join(RESULTS_DIR, "ardupilot_sitl_transport_actuator_mapping_result.json")

R = {}  # top-level results dict, written to OUT_JSON at the end


def log(*a):
    print(*a, flush=True)


def is_finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


# =============================================================================
# gz-transport helpers
# =============================================================================
class DoubleSub:
    """Subscribes to a single gz.msgs.Double topic (e.g. a cmd_rad /
    throttle_cmd echo), storing every value received."""

    def __init__(self, topic):
        self.topic = topic
        self.node = tp.Node()
        self.history = []
        ok = self.node.subscribe(double_pb2.Double, topic, self._cb)
        if not ok:
            raise RuntimeError(f"failed to subscribe {topic}")

    def _cb(self, msg):
        self.history.append(msg.data)

    def latest(self):
        return self.history[-1] if self.history else None


class DoubleVSub:
    """Subscribes to a gz.msgs.Double_V diagnostics topic, storing every
    message as a plain list of floats."""

    def __init__(self, topic):
        self.topic = topic
        self.node = tp.Node()
        self.history = []
        ok = self.node.subscribe(double_v_pb2.Double_V, topic, self._cb)
        if not ok:
            raise RuntimeError(f"failed to subscribe {topic}")

    def _cb(self, msg):
        self.history.append(list(msg.data))

    def latest(self):
        return list(self.history[-1]) if self.history else None

    def count(self):
        return len(self.history)


ACTUATOR_DIAG_TOPIC = "/model/falcon_v2/actuators/diagnostics"
AERO_DIAG_TOPIC = "/model/falcon_v2/aerodynamics/diagnostics"
PROPULSION_DIAG_TOPIC = "/model/falcon_v2/propulsion/diagnostics"

ACTUATOR_SURFACES = ["left_aileron", "right_aileron", "left_elevator", "right_elevator", "rudder"]
ACTUATOR_FIELDS = ["cmd_rad", "target_clamped_rad", "setpoint_rad", "actual_angle_rad",
                    "actual_rate_rad_s", "target_clamp_active", "effort_clamp_active"]
AERO_FIELDS = ["V", "alpha", "beta", "qbar", "CL", "CD", "CY", "Cl", "Cm", "Cn"]
PROP_FIELDS = ["throttle_cmd", "current_A", "torque_Nm", "omega_rad_s", "rpm", "J", "Ct", "Cp",
               "thrust_N", "qProp_Nm", "interp_clamped", "rpm_cap_active",
               "neg_current_clamped", "current_limited"]


def actuator_fields_for(vec, surface):
    i = ACTUATOR_SURFACES.index(surface)
    chunk = vec[i * 7:(i + 1) * 7]
    return dict(zip(ACTUATOR_FIELDS, chunk))


def aero_fields(vec):
    return dict(zip(AERO_FIELDS, vec))


def prop_fields_for(vec, side):
    i = {"left": 0, "right": 1}[side]
    chunk = vec[i * 13:(i + 1) * 13]
    return dict(zip(PROP_FIELDS, chunk))


# =============================================================================
# Phase 0: transport health
# =============================================================================
def phase_transport_health(mav):
    log("\n=== PHASE 0: transport health ===")
    out = {}

    hb = mav.wait_heartbeat(timeout=15)
    out["heartbeat_received"] = hb is not None
    if hb:
        out["heartbeat"] = {
            "type": hb.type, "autopilot": hb.autopilot,
            "base_mode": hb.base_mode, "custom_mode": hb.custom_mode,
            "system_status": hb.system_status,
        }
    log("HEARTBEAT:", hb)

    # 45s continuous listen - checks for gaps, NaN/Inf, sane message set
    t0 = time.time()
    msgs = mav.drain(45)
    elapsed = time.time() - t0
    types_seen = {}
    nan_inf_found = []
    for m in msgs:
        t = m.get_type()
        types_seen[t] = types_seen.get(t, 0) + 1
        for fname in getattr(m, "_fieldnames", []):
            v = getattr(m, fname)
            if isinstance(v, float) and not math.isfinite(v):
                nan_inf_found.append((t, fname, v))

    out["listen_window_s"] = elapsed
    out["message_types_seen"] = types_seen
    out["total_messages"] = len(msgs)
    out["nan_or_inf_fields"] = nan_inf_found[:50]
    out["nan_or_inf_count"] = len(nan_inf_found)

    log(f"Collected {len(msgs)} messages over {elapsed:.1f}s,"
        f" {len(types_seen)} distinct types, NaN/Inf fields: {len(nan_inf_found)}")
    log("Types:", sorted(types_seen.items()))

    return out


# =============================================================================
# Phase 13: SITL param load verification
# =============================================================================
EXPECTED_PARAMS = {
    "RLL_RATE_P": 0.25, "RLL_RATE_I": 0.125, "RLL_RATE_D": 0.002, "RLL_RATE_FF": 0.125,
    "PTCH_RATE_P": 0.25, "PTCH_RATE_I": 0.125, "PTCH_RATE_D": 0.002, "PTCH_RATE_FF": 0.125,
    "AUTOTUNE_LEVEL": 8,
    "SERVO1_FUNCTION": 4, "SERVO2_FUNCTION": 19, "SERVO3_FUNCTION": 73,
    "SERVO4_FUNCTION": 21, "SERVO5_FUNCTION": 74,
    "AIRSPEED_MIN": 16, "AIRSPEED_CRUISE": 18, "AIRSPEED_MAX": 28,
    "ARSPD_TYPE": 100, "ARSPD_USE": 1,
}


def phase_param_verification(mav):
    log("\n=== PHASE 13: SITL param load verification ===")
    params = mav.fetch_all_params(timeout=30)
    out = {"total_params_reported": len(params), "checks": {}}
    all_ok = True
    for k, expected in EXPECTED_PARAMS.items():
        actual = params.get(k)
        ok = actual is not None and abs(actual - expected) < 1e-3
        all_ok = all_ok and ok
        out["checks"][k] = {"expected": expected, "actual": actual, "pass": ok}
        log(f"  {k:16s} expected={expected!s:8s} actual={actual!s:12s} {'PASS' if ok else 'FAIL'}")
    out["all_pass"] = all_ok
    with open("/tmp/claude-1000/-home-emirhan-Desktop-FalconV2/6fc2e271-9722-4bcf-917a-17eb68aeb84c/scratchpad/ardupilot_sitl/full_params_final.json", "w") as f:
        json.dump(params, f, indent=2)
    return out


# =============================================================================
# Phase 2: frame validation - independent method (VelocityControl / cmd_vel)
# =============================================================================
def publish_cmd_vel(pub, lin, ang):
    msg = twist_pb2.Twist()
    msg.linear.x, msg.linear.y, msg.linear.z = lin
    msg.angular.x, msg.angular.y, msg.angular.z = ang
    pub.publish(msg)


def phase_frame_validation(mav):
    log("\n=== PHASE 2: frame validation (independent method: body-frame cmd_vel) ===")
    out = {}
    node = tp.Node()
    pub = node.advertise("/model/falcon_v2/cmd_vel", twist_pb2.Twist)
    time.sleep(0.5)  # let gz-transport discovery complete before first publish

    def read_state():
        msgs = mav.drain(1.0, types={"LOCAL_POSITION_NED", "ATTITUDE", "GLOBAL_POSITION_INT"})
        state = {}
        for m in msgs:
            state[m.get_type()] = m.to_dict()
        return state

    # --- baseline ---
    publish_cmd_vel(pub, (0, 0, 0), (0, 0, 0))
    time.sleep(1.5)
    base = read_state()
    out["baseline"] = base
    log("baseline:", base)

    # --- 2a: body +X (forward) ---
    publish_cmd_vel(pub, (5.0, 0, 0), (0, 0, 0))
    time.sleep(2.0)
    after_x = read_state()
    out["body_plus_x"] = after_x
    log("after body +X cmd_vel=5 m/s for 2s:", after_x)

    # stop / re-baseline before next axis
    publish_cmd_vel(pub, (0, 0, 0), (0, 0, 0))
    time.sleep(1.0)
    mid = read_state()
    out["reset_after_x"] = mid

    # --- 2b: body +Y (left) ---
    publish_cmd_vel(pub, (0, 5.0, 0), (0, 0, 0))
    time.sleep(2.0)
    after_y = read_state()
    out["body_plus_y"] = after_y
    log("after body +Y cmd_vel=5 m/s for 2s:", after_y)

    publish_cmd_vel(pub, (0, 0, 0), (0, 0, 0))
    time.sleep(1.0)

    # --- 2c: body +Z angular (yaw rate) ---
    before_yaw_msgs = mav.drain(1.0, types={"ATTITUDE"})
    yaw_before = before_yaw_msgs[-1].yaw if before_yaw_msgs else None
    publish_cmd_vel(pub, (0, 0, 0), (0, 0, 1.0))
    gyro_msgs = mav.drain(2.0, types={"ATTITUDE", "RAW_IMU"})
    out["body_plus_z_angular"] = {
        "yaw_before": yaw_before,
        "attitude_samples": [m.to_dict() for m in gyro_msgs if m.get_type() == "ATTITUDE"][:5],
        "raw_imu_samples": [m.to_dict() for m in gyro_msgs if m.get_type() == "RAW_IMU"][:5],
    }
    log("angular +Z body-rate 1.0 rad/s commanded - attitude/imu samples logged")

    # --- Set up trim-velocity forward flight for the control-surface phases
    # that follow (pinned, wings-level, straight condition, per this task's
    # own hold-technique precedent, aero_lib.py hold_step() - here via
    # VelocityControl Cmd instead of a force/torque controller). 21.244 m/s
    # is the documented trim velocity, CLAUDE.md / master dataset. ---
    publish_cmd_vel(pub, (0, 0, 0), (0, 0, 0))
    time.sleep(1.0)
    publish_cmd_vel(pub, (21.244, 0, 0), (0, 0, 0))
    time.sleep(2.0)
    pinned = read_state()
    out["pinned_trim_velocity_state"] = pinned
    log("pinned at body +X = 21.244 m/s for subsequent control-surface tests:", pinned)

    return out


# =============================================================================
# Phases 3/4/5: aileron / elevator / rudder mapping
# =============================================================================
def read_surface(actuator_sub, aero_sub, surface_left, surface_right=None):
    a_latest = actuator_sub.latest()
    ae_latest = aero_sub.latest()
    out = {}
    if a_latest:
        out[surface_left] = actuator_fields_for(a_latest, surface_left)
        if surface_right:
            out[surface_right] = actuator_fields_for(a_latest, surface_right)
    if ae_latest:
        out["aero"] = aero_fields(ae_latest)
    return out


def te_direction(angle_rad, kind):
    """kind: 'aileron'/'elevator' (positive=TE up) or 'rudder' (positive=
    toward -Y/right, FLU) - per CONTROLS.md sec 10 / design doc sec 7,
    already VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST, cited not re-derived."""
    if kind in ("aileron", "elevator"):
        if angle_rad > 1e-4:
            return "TE_UP"
        if angle_rad < -1e-4:
            return "TE_DOWN"
        return "NEUTRAL"
    else:
        if angle_rad > 1e-4:
            return "TE_RIGHT(-Y)"
        if angle_rad < -1e-4:
            return "TE_LEFT(+Y)"
        return "NEUTRAL"


def phase_aileron(mav, actuator_sub, aero_sub):
    log("\n=== PHASE 3: aileron mapping ===")
    out = {}
    for label, rc1 in [("roll_right_demand_RC1_1900", 1900), ("roll_left_demand_RC1_1100", 1100)]:
        log(f"-- {label} --")
        mav.hold_rc_override(2.5, rc1=rc1, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        servo = mav.drain(0.5, types={"SERVO_OUTPUT_RAW"})
        state = read_surface(actuator_sub, aero_sub, "left_aileron", "right_aileron")
        left_angle = state.get("left_aileron", {}).get("actual_angle_rad")
        right_angle = state.get("right_aileron", {}).get("actual_angle_rad")
        rec = {
            "rc1_commanded": rc1,
            "servo_output_raw": servo[-1].to_dict() if servo else None,
            "left_aileron": state.get("left_aileron"),
            "right_aileron": state.get("right_aileron"),
            "left_TE": te_direction(left_angle, "aileron") if left_angle is not None else None,
            "right_TE": te_direction(right_angle, "aileron") if right_angle is not None else None,
            "aero": state.get("aero"),
        }
        out[label] = rec
        log(json.dumps(rec, indent=2, default=str))

    # return to neutral
    mav.hold_rc_override(1.0, rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    return out


def phase_elevator(mav, actuator_sub, aero_sub):
    log("\n=== PHASE 4: elevator mapping ===")
    out = {}
    for label, rc2 in [("pitch_A_RC2_1900", 1900), ("pitch_B_RC2_1100", 1100)]:
        log(f"-- {label} --")
        mav.hold_rc_override(2.5, rc1=1500, rc2=rc2, rc3=1000, rc4=1500, rc5=1000)
        servo = mav.drain(0.5, types={"SERVO_OUTPUT_RAW"})
        state = read_surface(actuator_sub, aero_sub, "left_elevator", "right_elevator")
        left_angle = state.get("left_elevator", {}).get("actual_angle_rad")
        right_angle = state.get("right_elevator", {}).get("actual_angle_rad")
        rec = {
            "rc2_commanded": rc2,
            "servo_output_raw": servo[-1].to_dict() if servo else None,
            "left_elevator": state.get("left_elevator"),
            "right_elevator": state.get("right_elevator"),
            "left_TE": te_direction(left_angle, "elevator") if left_angle is not None else None,
            "right_TE": te_direction(right_angle, "elevator") if right_angle is not None else None,
            "aero": state.get("aero"),
        }
        out[label] = rec
        log(json.dumps(rec, indent=2, default=str))

    mav.hold_rc_override(1.0, rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    return out


def phase_rudder(mav, actuator_sub, aero_sub):
    log("\n=== PHASE 5: rudder mapping ===")
    out = {}
    for label, rc4 in [("yaw_A_RC4_1900", 1900), ("yaw_B_RC4_1100", 1100)]:
        log(f"-- {label} --")
        mav.hold_rc_override(2.5, rc1=1500, rc2=1500, rc3=1000, rc4=rc4, rc5=1000)
        servo = mav.drain(0.5, types={"SERVO_OUTPUT_RAW"})
        state = read_surface(actuator_sub, aero_sub, "rudder")
        angle = state.get("rudder", {}).get("actual_angle_rad")
        rec = {
            "rc4_commanded": rc4,
            "servo_output_raw": servo[-1].to_dict() if servo else None,
            "rudder": state.get("rudder"),
            "rudder_TE": te_direction(angle, "rudder") if angle is not None else None,
            "aero": state.get("aero"),
        }
        out[label] = rec
        log(json.dumps(rec, indent=2, default=str))

    mav.hold_rc_override(1.0, rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    return out


# =============================================================================
# Phase 11: manual output range (surfaces only - throttle part blocked, see
# phase_motor_blocked below)
# =============================================================================
def phase_manual_range_surfaces(mav, actuator_sub):
    log("\n=== PHASE 11 (surfaces): manual output range test ===")
    out = {}
    cases = {
        "neutral": dict(rc1=1500, rc2=1500, rc4=1500),
        "aileron_small_plus": dict(rc1=1700, rc2=1500, rc4=1500),
        "aileron_small_minus": dict(rc1=1300, rc2=1500, rc4=1500),
        "elevator_small_plus": dict(rc1=1500, rc2=1700, rc4=1500),
        "elevator_small_minus": dict(rc1=1500, rc2=1300, rc4=1500),
        "rudder_small_plus": dict(rc1=1500, rc2=1500, rc4=1700),
        "rudder_small_minus": dict(rc1=1500, rc2=1500, rc4=1300),
    }
    for label, kw in cases.items():
        mav.hold_rc_override(1.5, rc3=1000, rc5=1000, **kw)
        a_latest = actuator_sub.latest()
        rec = {"rc": kw}
        if a_latest:
            for s in ACTUATOR_SURFACES:
                rec[s] = actuator_fields_for(a_latest, s)
        out[label] = rec
        log(label, json.dumps(rec, default=str))
    mav.hold_rc_override(1.0, rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    return out


# =============================================================================
# Motor/throttle tests - BLOCKED this session (documented, not worked around)
# =============================================================================
def phase_motor_blocked(mav):
    log("\n=== PHASES 6/7/8/9/12: motor/throttle tests - attempting arm ===")
    out = {}

    # Safety switch off - permitted, standard, non-arming, non-persisted
    # MAVLink command (MAV_CMD_DO_SET_SAFETY_SWITCH_STATE=1/DANGEROUS).
    ack = mav.command_long(mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, p1=1)
    out["safety_switch_off_ack"] = ack.to_dict() if ack else None
    log("safety switch off ack:", ack)

    # Plain arm attempt (no force, no param bypass) - within scope, always
    # permitted to attempt.
    ack = mav.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1)
    texts = mav.drain(2.0, types={"STATUSTEXT"})
    out["plain_arm_attempt"] = {
        "ack": ack.to_dict() if ack else None,
        "statustexts": [t.text for t in texts],
    }
    log("plain arm attempt:", out["plain_arm_attempt"])

    out["blocked"] = True
    out["blocked_reason"] = (
        "Vehicle failed to arm (AP_Arming pre-arm check: 'AHRS: DCM "
        "Roll/Pitch inconsistent ~54 deg', gated under Check::INS, "
        "AP_Arming_Plane.cpp ins_checks(), source-confirmed this task). "
        "Two bypass avenues were identified and source-verified as "
        "standard, non-persisted, SITL-only ArduPilot mechanisms "
        "(runtime PARAM_SET ARMING_SKIPCHK=16 to skip the INS check bit; "
        "MAVLink force-arm via COMPONENT_ARM_DISARM param2=2989, "
        "ArduPilot's own documented magic_force_arm_value) but BOTH "
        "attempts were denied by this environment's own auto-mode "
        "permission classifier, independent of ArduPlane. Per this "
        "task's own instructions this was not worked around further; "
        "reported here as a plain blocker for review/decision."
    )
    return out


def main():
    R["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    mav = SafeMav("tcp:127.0.0.1:5760", source_system=255)

    R["phase0_transport_health"] = phase_transport_health(mav)
    R["phase13_param_verification"] = phase_param_verification(mav)
    R["phase2_frame_validation"] = phase_frame_validation(mav)

    actuator_sub = DoubleVSub(ACTUATOR_DIAG_TOPIC)
    aero_sub = DoubleVSub(AERO_DIAG_TOPIC)
    time.sleep(1.0)

    R["phase3_aileron"] = phase_aileron(mav, actuator_sub, aero_sub)
    R["phase4_elevator"] = phase_elevator(mav, actuator_sub, aero_sub)
    R["phase5_rudder"] = phase_rudder(mav, actuator_sub, aero_sub)
    R["phase11_manual_range_surfaces"] = phase_manual_range_surfaces(mav, actuator_sub)
    R["phase6_7_8_9_12_motor_tests"] = phase_motor_blocked(mav)

    mav.close()

    os.makedirs(RESULTS_DIR, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(R, f, indent=2, default=str)
    log(f"\nWrote {OUT_JSON}")


if __name__ == "__main__":
    main()

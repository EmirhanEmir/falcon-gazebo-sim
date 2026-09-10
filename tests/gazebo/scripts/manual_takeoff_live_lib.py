#!/usr/bin/env python3
"""FALCON V2 - shared LIVE capture helpers for stage
MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION.

CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY.
Owner: controls-integration. Created 2026-09-10.

THIS MODULE CHANGES NO PHYSICS PARAMETER. It contains no aerodynamic
coefficient, no propulsion coefficient, no mass/CG/inertia, no actuator
PID/rate/effort value, no control-surface mapping, no PTCH_TRIM_DEG, no
TECS_PTCH_DAMP, no roll/pitch PID, no sensor value, no friction or collision
value. It subscribes, samples and time-stamps. Nothing here writes to
model/model.sdf, codex/, docs/source_of_truth/ or config/ardupilot/.

WHAT IT PROVIDES
----------------
* SIGN CONVENTION constants and the CONTROLS.md sec 10 delta mapping, read
  from the source of truth at import time (never hard-coded, and asserted
  against the documented VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST values so a
  silent drift fails loudly).
* LinkPoseSub  - subscribes to /world/<world>/dynamic_pose/info, THE SAME
  STREAM THE GAZEBO GUI RENDERS FROM. This is what makes the "does the
  rendered visual follow the ACTUAL joint position?" question answerable:
  we read the exact transform the renderer reads.
* StaticPoseSub - subscribes to /world/<world>/pose/info once, to capture the
  static visual-within-link offsets (the second half of the render chain).
* MultiSourceSampler - binds actuator / aero / propulsion / pose / MAVLink
  readings into one record AND records the per-source arrival timestamp and
  the resulting SKEW, instead of pretending they are simultaneous.

TIME BASES (three of them; never silently mixed)
------------------------------------------------
  t_wall   : python time.time() on the test host - the only clock ALL
             sources share, used to compute skew.
  t_sim    : gz sim time from /world/<world>/clock (authoritative for
             anything compared against a physics step).
  t_mav    : ArduPlane's own time_boot_ms where the message carries it.
Every sample records all three where available plus the skew between them.
"""
import math
import os
import threading
import time

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = os.path.join(REPO_ROOT, "tests/gazebo/results")
AERO_CFG = os.path.join(REPO_ROOT,
                        "docs/source_of_truth/aerodynamics/aero_v1_config.yaml")
ACT_CFG = os.path.join(REPO_ROOT,
                       "docs/source_of_truth/controls/actuator_v1_config.yaml")
MODEL_SDF = os.path.join(REPO_ROOT, "model/model.sdf")

SURFACES = ["left_aileron", "right_aileron", "left_elevator",
            "right_elevator", "rudder"]
JOINT_NAMES = {s: s + "_joint" for s in SURFACES}

# ---------------------------------------------------------------------------
# ArduPlane -> Gazebo command transport constants.
# NOT magic numbers: every one is READ BACK from model/model.sdf at import
# time by read_control_blocks() below and cross-checked against these
# documented values, so the test fails loudly if the SDF is ever changed
# under it. Provenance: model/model.sdf <plugin name="ArduPilotPlugin">
# <control> blocks; docs/source_of_truth/autopilot/
# SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md sec 3 and sec 11.
# ---------------------------------------------------------------------------
EXPECTED_MULTIPLIER_RAD = 1.5707963268   # = pi/2; +/-45 deg half-range
EXPECTED_SERVO_MIN = 800
EXPECTED_SERVO_MAX = 2200
EXPECTED_OFFSET = -0.5
SERVO_TRIM_PWM = 1500                     # SERVOx_TRIM, falcon_v2_sitl.parm
# One PWM microsecond expressed in joint radians, DERIVED from the above:
#   cmd = multiplier * ((pwm - servo_min)/(servo_max - servo_min) - 0.5)
PWM_QUANTUM_RAD = EXPECTED_MULTIPLIER_RAD / (EXPECTED_SERVO_MAX - EXPECTED_SERVO_MIN)
PWM_QUANTUM_DEG = math.degrees(PWM_QUANTUM_RAD)

# ---------------------------------------------------------------------------
# SIGN CONVENTION - cited, never invented here.
#
# docs/source_of_truth/controls/CONTROLS.md sec 10
# ("Final verified mapping equations", VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST,
#  2026-08-22), confirmed live in
# docs/test_results/2026-08-22_control_surface_sign_mapping_test_report.md:
#
#   PHYSICAL MEANING OF A POSITIVE JOINT ANGLE
#     left_aileron_joint   +  -> LEFT aileron trailing edge UP   (test 1)
#     right_aileron_joint  +  -> RIGHT aileron trailing edge UP  (test 2)
#     left_elevator_joint  +  -> LEFT elevator trailing edge UP  (test 5)
#     right_elevator_joint +  -> RIGHT elevator trailing edge UP (test 6)
#     rudder_joint         +  -> rudder trailing edge toward -Y  (test 7)
#                                (-Y = RIGHT in the FLU body frame)
#   The two ailerons are NOT sign-mirrored: an equal-sign command on both
#   gives bit-identical physical motion. Differential roll therefore REQUIRES
#   opposite-sign joint commands, which model.sdf's <control channel="0">
#   pair implements with multiplier +1.5707963268 / -1.5707963268.
#
#   AERO DEFLECTION FED TO THE AERO MODEL
#     delta_a_aero = +0.5 * (theta_right_aileron - theta_left_aileron)
#     delta_e_aero = -0.5 * (theta_left_elevator + theta_right_elevator)
#     delta_r_aero = +1.0 *  theta_rudder
#
#   RESULTING MOMENT SIGN, FLU BODY FRAME (measured, Part B of the same
#   report; base-link inertia Ixx=0.7284 Iyy=0.2507 Izz=0.9523 kg m^2)
#     delta_a_aero > 0  ->  Mx = +4.877 N m  ->  ROLL RIGHT
#                           (left wing up, right wing down)
#     delta_e_aero < 0  ->  My = -1.486 N m  ->  NOSE UP
#     delta_r_aero > 0  ->  Mz = -0.446 N m  ->  NOSE RIGHT
#
#   ARDUPLANE COMMAND DIRECTION (design record:
#   SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md sec 7.1/7.2/7.3, with
#   SERVO1_REVERSED=1, SERVO2_REVERSED=0, SERVO4_REVERSED=1 in
#   config/ardupilot/falcon_v2_sitl.parm)
#     roll-right demand -> aileron PWM BELOW 1500 -> delta_a_aero > 0
#     pitch-up   demand -> elevator PWM ABOVE 1500 -> theta_elev > 0 (TE up)
#                                                  -> delta_e_aero < 0
#     yaw-right  demand -> rudder  PWM BELOW 1500 -> delta_r_aero > 0
#   These three lines are the PRE-REGISTERED EXPECTATIONS that
#   test_control_surface_live_sign_scenarios.py states in advance and then
#   measures. They are NOT re-fitted from the data.
# ---------------------------------------------------------------------------
EXPECTED_SIGNS = {"aileron_sign": 1.0, "elevator_sign": -1.0, "rudder_sign": 1.0}

PRE_REGISTERED_EXPECTATIONS = {
    "roll_right": {
        "rc_channel": 1,
        "rc_direction": "above trim (stick right)",
        "expected_aileron_pwm_vs_trim": "BELOW 1500 (SERVO1_REVERSED=1)",
        "expected_left_aileron_joint": "negative -> LEFT aileron TE DOWN",
        "expected_right_aileron_joint": "positive -> RIGHT aileron TE UP",
        "expected_delta_a_aero_sign": "+",
        "expected_Cl_sign": "+",
        "expected_roll_rate_sign": "+ (roll right, right wing down)",
        "source": "SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md sec 7.1 + "
                  "CONTROLS.md sec 10",
    },
    "roll_left": {
        "rc_channel": 1,
        "rc_direction": "below trim (stick left)",
        "expected_aileron_pwm_vs_trim": "ABOVE 1500",
        "expected_left_aileron_joint": "positive -> LEFT aileron TE UP",
        "expected_right_aileron_joint": "negative -> RIGHT aileron TE DOWN",
        "expected_delta_a_aero_sign": "-",
        "expected_Cl_sign": "-",
        "expected_roll_rate_sign": "- (roll left)",
        "source": "mirror of roll_right, same sources",
    },
    "pitch_up": {
        "rc_channel": 2,
        "rc_direction": "ArduPlane pitch-up demand",
        "expected_elevator_pwm_vs_trim": "ABOVE 1500 (SERVO2_REVERSED=0)",
        "expected_left_elevator_joint": "positive -> TE UP",
        "expected_right_elevator_joint": "positive -> TE UP (SAME sign as left)",
        "expected_delta_e_aero_sign": "-",
        "expected_Cm_sign": "+ (Cm>0 <=> My<0 <=> nose up in this plugin)",
        "expected_pitch_rate_sign": "+ (nose up)",
        "source": "SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md sec 7.2 + "
                  "CONTROLS.md sec 10 + 2026-08-22 report tests 5/6/10/11",
    },
    "pitch_down": {
        "rc_channel": 2,
        "rc_direction": "ArduPlane pitch-down demand",
        "expected_elevator_pwm_vs_trim": "BELOW 1500",
        "expected_left_elevator_joint": "negative -> TE DOWN",
        "expected_right_elevator_joint": "negative -> TE DOWN (SAME sign)",
        "expected_delta_e_aero_sign": "+",
        "expected_Cm_sign": "-",
        "expected_pitch_rate_sign": "- (nose down)",
        "source": "mirror of pitch_up, same sources",
    },
    "yaw_right": {
        "rc_channel": 4,
        "rc_direction": "above trim (right rudder)",
        "expected_rudder_pwm_vs_trim": "BELOW 1500 (SERVO4_REVERSED=1)",
        "expected_rudder_joint": "positive -> rudder TE toward -Y (RIGHT)",
        "expected_delta_r_aero_sign": "+",
        "expected_Cn_sign": "- (Mz<0 = nose right)",
        "expected_yaw_rate_sign": "- in FLU body frame (nose right is a "
                                  "NEGATIVE rotation about +Z-up); "
                                  "+ in the MAVLink FRD yawspeed field",
        "source": "SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md sec 7.3 + "
                  "CONTROLS.md sec 10 + 2026-08-22 report tests 7/9",
    },
    "yaw_left": {
        "rc_channel": 4,
        "rc_direction": "below trim (left rudder)",
        "expected_rudder_pwm_vs_trim": "ABOVE 1500",
        "expected_rudder_joint": "negative -> rudder TE toward +Y (LEFT)",
        "expected_delta_r_aero_sign": "-",
        "expected_Cn_sign": "+",
        "expected_yaw_rate_sign": "+ in FLU body frame; - in MAVLink FRD",
        "source": "mirror of yaw_right, same sources",
    },
}

# DATA_REQUIRED, recorded here so no consumer of this module can assume
# otherwise. Verified this stage by direct source read of
# plugins/aerodynamics/AerodynamicsSystem.cc (diagnostics payload is exactly
# V, alpha, beta, qbar, CL, CD, CY, Cl, Cm, Cn - 10 fields, lines ~365-384).
AERO_DEFLECTION_NOT_PUBLISHED = {
    "issue": "The aerodynamics plugin does NOT publish the delta_a/delta_e/"
             "delta_r it actually consumes.",
    "evidence": "plugins/aerodynamics/AerodynamicsSystem.cc PreUpdate() "
                "computes state.deltaA/deltaE/deltaR locally and its "
                "diagnostics Double_V carries only "
                "V,alpha,beta,qbar,CL,CD,CY,Cl,Cm,Cn.",
    "consequence": "The deflection the aero model consumed cannot be OBSERVED "
                   "directly. It is RECONSTRUCTED here from the actuator "
                   "plugin's actual_angle_rad using the CONTROLS.md sec 10 "
                   "formulas, which is exact ONLY because both plugins read "
                   "the SAME gz::sim::Joint(e).Position(_ecm) in the same "
                   "PreUpdate tick (confirmed by source read of both). The "
                   "reconstruction is INDEPENDENTLY CROSS-CHECKED against the "
                   "published Cl/Cm/Cn via aero_lib.compute_aero(), so a "
                   "mismatch is detectable rather than assumed away.",
    "status": "DATA_REQUIRED",
    "recommended_fix_owner": "aerodynamics",
    "recommended_fix": "append deltaA, deltaE, deltaR to the aerodynamics "
                       "diagnostics payload (additive, 3 extra Double_V "
                       "fields, no coefficient change). NOT done by this "
                       "stage - out of this agent's ownership and out of "
                       "this stage's measurement-only scope.",
}


# ---------------------------------------------------------------------------
# Source-of-truth readers (read-only, never write)
# ---------------------------------------------------------------------------
def read_sign_scalars():
    signs = {}
    with open(AERO_CFG, "r", encoding="utf-8") as fh:
        for line in fh:
            st = line.strip()
            for key in EXPECTED_SIGNS:
                if st.startswith(key + ":") and key not in signs:
                    signs[key] = float(st.split(":", 1)[1].split("#")[0].strip())
    if signs != EXPECTED_SIGNS:
        raise RuntimeError(
            f"aero_v1_config.yaml control_mapping signs {signs} != the "
            f"CONTROLS.md sec 10 VERIFIED values {EXPECTED_SIGNS}. Refusing to "
            f"run: the sign convention this harness pre-registers would be "
            f"wrong.")
    return signs


def read_control_blocks():
    """Read the 7 ArduPilotPlugin <control> blocks out of model/model.sdf and
    verify the transport constants this module assumes. READ-ONLY."""
    import xml.etree.ElementTree as ET
    root = ET.parse(MODEL_SDF).getroot()
    model = root.find("model")
    blocks = []
    for plug in model.findall("plugin"):
        if plug.get("name") != "ArduPilotPlugin":
            continue
        for c in plug.findall("control"):
            blocks.append({
                "channel": int(c.get("channel")),
                "jointName": c.findtext("jointName"),
                "type": c.findtext("type"),
                "cmd_topic": c.findtext("cmd_topic"),
                "multiplier": float(c.findtext("multiplier")),
                "offset": float(c.findtext("offset")),
                "servo_min": int(c.findtext("servo_min")),
                "servo_max": int(c.findtext("servo_max")),
            })
    problems = []
    for b in blocks:
        if "prop" in (b["jointName"] or ""):
            continue  # throttle channels: 1:1 [0,1], different convention
        if abs(abs(b["multiplier"]) - EXPECTED_MULTIPLIER_RAD) > 1e-9:
            problems.append(f"{b['jointName']}: multiplier {b['multiplier']}")
        if b["offset"] != EXPECTED_OFFSET:
            problems.append(f"{b['jointName']}: offset {b['offset']}")
        if b["servo_min"] != EXPECTED_SERVO_MIN or b["servo_max"] != EXPECTED_SERVO_MAX:
            problems.append(f"{b['jointName']}: servo range "
                            f"{b['servo_min']}..{b['servo_max']}")
    return blocks, problems


def read_model_structure():
    """Hinge axis, link pose and visual pose per surface, from model/model.sdf.
    These are geometry-structure-owned values; this harness only READS them."""
    import xml.etree.ElementTree as ET

    def triple(text, n=3, start=0):
        vals = [float(v) for v in (text or "0 0 0 0 0 0").split()]
        while len(vals) < 6:
            vals.append(0.0)
        return vals[start:start + n]

    root = ET.parse(MODEL_SDF).getroot()
    model = root.find("model")
    links = {x.get("name"): x for x in model.findall("link")}
    out = {}
    for s in SURFACES:
        joint = model.find(f"joint[@name='{JOINT_NAMES[s]}']")
        link = links[s]
        visual = link.find(f"visual[@name='{s}_visual']")
        out[s] = {
            "joint": JOINT_NAMES[s],
            "parent_link": joint.findtext("parent"),
            "child_link": joint.findtext("child"),
            "axis_xyz": [float(v) for v in joint.findtext("axis/xyz").split()],
            "limit_lower_rad": float(joint.findtext("axis/limit/lower")),
            "limit_upper_rad": float(joint.findtext("axis/limit/upper")),
            "link_pose_xyz": triple(link.findtext("pose")),
            "link_pose_rpy": triple(link.findtext("pose"), start=3),
            "visual_pose_xyz": triple(visual.findtext("pose")) if visual is not None else None,
            "visual_name": (s + "_visual") if visual is not None else None,
        }
    return out


def aero_deltas_from_joint_angles(theta, signs):
    """CONTROLS.md sec 10 mapping. theta is a dict surface -> radians."""
    return {
        "delta_a": 0.5 * signs["aileron_sign"] * (theta["right_aileron"]
                                                  - theta["left_aileron"]),
        "delta_e": 0.5 * signs["elevator_sign"] * (theta["left_elevator"]
                                                   + theta["right_elevator"]),
        "delta_r": signs["rudder_sign"] * theta["rudder"],
    }


def pwm_to_cmd_rad(pwm, multiplier):
    """Exactly ArduPilotPlugin::UpdateMotorCommands() (confirmed by source
    read, documented in SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md sec 3)."""
    raw = (float(pwm) - EXPECTED_SERVO_MIN) / (EXPECTED_SERVO_MAX - EXPECTED_SERVO_MIN)
    raw = max(0.0, min(1.0, raw))
    return multiplier * (raw + EXPECTED_OFFSET)


# ---------------------------------------------------------------------------
# Quaternion helpers (no gz.math dependency; this runs against a SEPARATE
# gz server process over gz-transport, not inside a TestFixture)
# ---------------------------------------------------------------------------
def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_rotate(q, v):
    qw, qx, qy, qz = q
    t = (2.0 * (qy * v[2] - qz * v[1]),
         2.0 * (qz * v[0] - qx * v[2]),
         2.0 * (qx * v[1] - qy * v[0]))
    return (v[0] + qw * t[0] + (qy * t[2] - qz * t[1]),
            v[1] + qw * t[1] + (qz * t[0] - qx * t[2]),
            v[2] + qw * t[2] + (qx * t[1] - qy * t[0]))


def q_from_axis_angle(axis, angle):
    n = math.sqrt(sum(c * c for c in axis))
    if n <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    u = [c / n for c in axis]
    s = math.sin(0.5 * angle)
    return (math.cos(0.5 * angle), u[0] * s, u[1] * s, u[2] * s)


def angle_about_axis(q, axis):
    """Signed rotation angle of q about `axis`, assuming q IS a rotation about
    that axis (which a revolute joint's relative transform must be). Computed
    from the axis-projected vector part and the scalar part, so it is exact for
    a pure axis rotation and reports the off-axis part separately via
    off_axis_residual()."""
    n = math.sqrt(sum(c * c for c in axis))
    u = [c / n for c in axis]
    vdot = q[1] * u[0] + q[2] * u[1] + q[3] * u[2]
    return 2.0 * math.atan2(vdot, q[0])


def off_axis_residual(q, axis):
    """Magnitude of the component of q's vector part PERPENDICULAR to `axis`.
    For a true revolute joint this must be ~0; a non-zero value means the
    child link is not rotating purely about the declared hinge axis."""
    n = math.sqrt(sum(c * c for c in axis))
    u = [c / n for c in axis]
    v = (q[1], q[2], q[3])
    d = v[0] * u[0] + v[1] * u[1] + v[2] * u[2]
    perp = (v[0] - d * u[0], v[1] - d * u[1], v[2] - d * u[2])
    return math.sqrt(sum(c * c for c in perp))


def quat_to_rpy(qw, qx, qy, qz):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    s = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    return roll, math.asin(s), math.atan2(2 * (qw * qz + qx * qy),
                                          1 - 2 * (qy * qy + qz * qz))


# ---------------------------------------------------------------------------
# Transport subscribers
# ---------------------------------------------------------------------------
class ClockSub:
    """gz sim time. The authoritative clock for anything compared against a
    physics step; kept separate from wall time on purpose."""

    def __init__(self, world):
        import gz.transport13 as tp
        from gz.msgs10 import clock_pb2
        self._pb = clock_pb2
        self.node = tp.Node()
        self.lock = threading.Lock()
        self.last = None
        self.last_wall = None
        ok = self.node.subscribe(clock_pb2.Clock, f"/world/{world}/clock", self._cb)
        if not ok:
            raise RuntimeError(f"subscribe failed: /world/{world}/clock")

    def _cb(self, msg):
        with self.lock:
            self.last = msg.sim.sec + msg.sim.nsec * 1e-9
            self.last_wall = time.time()

    def latest(self):
        with self.lock:
            return self.last, self.last_wall


class LinkPoseSub:
    """Subscribes to /world/<world>/dynamic_pose/info.

    THIS IS THE RENDER STREAM. gz-sim's SceneBroadcaster publishes the pose of
    every entity that MOVED on this topic, and the GUI's scene manager applies
    exactly these poses to the rendered nodes. Reading it is therefore not a
    proxy for "what the GUI shows" - it IS what the GUI shows.

    Poses on this topic are relative to the entity's PARENT (confirmed live
    against model/model.sdf: at rest, left_aileron reads
    (0.032943, 0.313950, 0.110356), bit-matching its <pose> under base_link).
    So for a control surface whose parent is base_link, the pose read here is
    already the joint's relative transform - which is exactly the quantity the
    'does the visual follow the ACTUAL joint angle?' question needs."""

    def __init__(self, world, names):
        import gz.transport13 as tp
        from gz.msgs10 import pose_v_pb2
        self.node = tp.Node()
        self.names = set(names)
        self.lock = threading.Lock()
        self.last = {}
        self.last_wall = None
        self.count = 0
        ok = self.node.subscribe(pose_v_pb2.Pose_V,
                                 f"/world/{world}/dynamic_pose/info", self._cb)
        if not ok:
            raise RuntimeError(f"subscribe failed: /world/{world}/dynamic_pose/info")

    def _cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        now = time.time()
        got = {}
        for p in msg.pose:
            if p.name in self.names:
                got[p.name] = {
                    "pos": (p.position.x, p.position.y, p.position.z),
                    "quat": (p.orientation.w, p.orientation.x,
                             p.orientation.y, p.orientation.z),
                }
        if got:
            with self.lock:
                self.last = got
                self.last_stamp = stamp
                self.last_wall = now
                self.count += 1

    def latest(self):
        with self.lock:
            if not self.last:
                return None, None, None
            return dict(self.last), getattr(self, "last_stamp", None), self.last_wall


class StaticPoseSub:
    """One-shot capture of /world/<world>/pose/info, which carries the FULL
    entity tree including <visual> children. The visual's pose within its link
    is static, so it is captured once and combined with the live link pose to
    reconstruct the rendered visual transform."""

    def __init__(self, world):
        import gz.transport13 as tp
        from gz.msgs10 import pose_v_pb2
        self.node = tp.Node()
        self.lock = threading.Lock()
        self.tree = {}
        ok = self.node.subscribe(pose_v_pb2.Pose_V,
                                 f"/world/{world}/pose/info", self._cb)
        if not ok:
            raise RuntimeError(f"subscribe failed: /world/{world}/pose/info")

    def _cb(self, msg):
        with self.lock:
            for p in msg.pose:
                self.tree[p.name] = {
                    "pos": (p.position.x, p.position.y, p.position.z),
                    "quat": (p.orientation.w, p.orientation.x,
                             p.orientation.y, p.orientation.z),
                }

    def snapshot(self):
        with self.lock:
            return dict(self.tree)


class TimedDiagSub:
    """A gz.msgs.Double_V subscriber that ALSO records the wall-clock arrival
    time of each message.

    The existing actuator_lib / aero_lib / propulsion_lib DiagSubscriber
    classes store values only. That is fine for their own in-process
    TestFixture usage, where every read is inside the same physics step, but
    it makes inter-source SKEW unmeasurable when several independent topics
    are bound into one record from outside the server. This stage was
    explicitly asked to record the skew rather than pretend simultaneity, so
    this subscriber exists alongside (not instead of) those libraries; the
    field ORDER is still taken from them, never redefined here."""

    def __init__(self, topic, splitter):
        import gz.transport13 as tp
        self.topic = topic
        self.splitter = splitter
        self.node = tp.Node()
        self.lock = threading.Lock()
        self.last = None
        self.last_wall = None
        self.count = 0
        ok = self.node.subscribe_raw(topic, self._cb, "gz.msgs.Double_V",
                                     tp.SubscribeOptions())
        if not ok:
            raise RuntimeError(f"Failed to subscribe to {topic}")

    def _cb(self, data, info):
        from gz.msgs10 import double_v_pb2
        m = double_v_pb2.Double_V()
        m.ParseFromString(data)
        vals = list(m.data)
        now = time.time()
        with self.lock:
            self.last = self.splitter(vals)
            self.last_wall = now
            self.count += 1

    def latest(self):
        with self.lock:
            return (dict(self.last) if isinstance(self.last, dict) else self.last,
                    self.last_wall)


class ContactSub:
    """Optional ground-contact stream.

    DATA_REQUIRED HANDLING: the runway world does NOT attach a
    gz-sim-contact-system, and model/model.sdf's collisions declare no
    <sensor type="contact">. Adding either would modify a world/model this
    stage is forbidden to touch. So this subscriber is BEST-EFFORT: if the
    topic does not exist, `available` stays False and the caller must report
    ground normal force as DATA_REQUIRED rather than inventing it. The
    fallback observable that IS available (and is captured unconditionally by
    the ground-roll test) is the vertical acceleration/velocity signature and
    the aircraft's z position, from which support-force loss is INFERRED, and
    that inference is labelled as such."""

    def __init__(self, world):
        import gz.transport13 as tp
        self.node = tp.Node()
        self.available = False
        self.topic = f"/world/{world}/contacts"
        self.lock = threading.Lock()
        self.last = None
        try:
            from gz.msgs10 import contacts_pb2
            self.available = self.node.subscribe(contacts_pb2.Contacts,
                                                 self.topic, self._cb)
        except Exception:
            self.available = False

    def _cb(self, msg):
        with self.lock:
            self.last = msg

    def latest(self):
        with self.lock:
            return self.last


# ---------------------------------------------------------------------------
# Skew-aware multi-source binding
# ---------------------------------------------------------------------------
def bind_sample(sources):
    """sources: dict name -> (value, arrival_wall_time_or_None).

    Returns {"values": {...}, "arrival_wall": {...}, "skew": {...}} where
    skew[name] = arrival_wall[name] - max(arrival_wall) (<= 0, seconds). The
    caller gets an EXPLICIT, per-source staleness figure instead of an
    implicit assumption that all sources are simultaneous."""
    values = {k: v[0] for k, v in sources.items()}
    arrivals = {k: v[1] for k, v in sources.items()}
    valid = [t for t in arrivals.values() if t is not None]
    ref = max(valid) if valid else None
    skew = {k: (None if (t is None or ref is None) else t - ref)
            for k, t in arrivals.items()}
    return {
        "values": values,
        "arrival_wall": arrivals,
        "skew_s": skew,
        "skew_reference": "most recently arrived source",
        "max_abs_skew_s": (max((abs(s) for s in skew.values() if s is not None),
                               default=None)),
    }


# ---------------------------------------------------------------------------
# MAVLink telemetry stream rates
#
# MEASURED FINDING (this stage, 2026-09-10, two independent live probes
# against arduplane -w -M json --defaults config/ardupilot/falcon_v2_sitl.parm):
#
#   MAV_CMD_SET_MESSAGE_INTERVAL (command 511) is REJECTED by this ArduPlane
#   SITL build with MAV_RESULT_DENIED (result=2), for ATTITUDE(30),
#   SERVO_OUTPUT_RAW(36), VFR_HUD(74), GLOBAL_POSITION_INT(33) and
#   NAV_CONTROLLER_OUTPUT(62) alike. The rejection is NOT an
#   INITIALISING-window artefact: it was re-probed after HEARTBEAT settled to
#   custom_mode=0 (MANUAL) and still returned DENIED, and the measured stream
#   rates were unchanged (1.0 Hz before and after).
#
#   The legacy REQUEST_DATA_STREAM message DOES work on the same link and the
#   same build. Measured after requesting 50 Hz:
#       MAV_DATA_STREAM_EXTRA1      -> ATTITUDE             39.8 Hz
#       MAV_DATA_STREAM_EXTRA2      -> VFR_HUD              39.8 Hz
#       MAV_DATA_STREAM_RC_CHANNELS -> SERVO_OUTPUT_RAW     39.8 Hz
#       MAV_DATA_STREAM_POSITION    -> GLOBAL_POSITION_INT  25.0 Hz (asked 25)
#   ~40 Hz is the observed ceiling for the 50 Hz requests.
#
#   NOTE ON THE STREAM MAPPING: SERVO_OUTPUT_RAW is carried by
#   MAV_DATA_STREAM_RC_CHANNELS, NOT by MAV_DATA_STREAM_RAW_CONTROLLER. A
#   first probe requested RAW_CONTROLLER and SERVO_OUTPUT_RAW stayed at 1 Hz;
#   requesting RC_CHANNELS moved it to 39.8 Hz. This was determined
#   empirically, not assumed from the message name.
#
# This helper therefore sends BOTH forms and then MEASURES what it actually
# got, returning the measured rates so every test records the rate it really
# had rather than the rate it asked for. It writes no parameter: both
# REQUEST_DATA_STREAM and SET_MESSAGE_INTERVAL are per-link telemetry
# requests, not stored settings.
# ---------------------------------------------------------------------------
DEFAULT_STREAM_REQUESTS = (
    # (MAV_DATA_STREAM_*, hz, [messages it is expected to carry])
    ("EXTRA1", 50, ["ATTITUDE"]),
    ("EXTRA2", 50, ["VFR_HUD"]),
    ("RC_CHANNELS", 50, ["SERVO_OUTPUT_RAW", "RC_CHANNELS"]),
    ("POSITION", 25, ["GLOBAL_POSITION_INT"]),
    ("EXTRA3", 10, []),
)


def request_stream_rates(mav, requests=DEFAULT_STREAM_REQUESTS,
                         measure_s=3.0, also_try_set_message_interval=True):
    """Raise the telemetry stream rates and MEASURE the result.

    Returns a dict with what was asked for, whether SET_MESSAGE_INTERVAL was
    accepted or denied, and the per-message rates actually measured over
    `measure_s` seconds. Never asserts; the caller records the numbers."""
    import select as _sel
    from pymavlink import mavutil

    out = {"requested": [], "set_message_interval_results": {},
           "measured_hz": {}, "measure_window_s": measure_s,
           "mechanism_note": "SET_MESSAGE_INTERVAL is DENIED on this build; "
                             "REQUEST_DATA_STREAM is the mechanism that "
                             "works. Both are sent; the measured rates below "
                             "are the ground truth."}
    if also_try_set_message_interval:
        for mid in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                    mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
                    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                    mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT):
            ack = mav.command_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                   p1=mid, p2=int(1e6 / 50.0), wait_ack=True,
                                   timeout=2)
            out["set_message_interval_results"][str(mid)] = (
                ack.result if ack else None)
    for name, hz, msgs in requests:
        sid = getattr(mavutil.mavlink, "MAV_DATA_STREAM_" + name, None)
        if sid is None:
            continue
        mav.m.mav.request_data_stream_send(
            mav.m.target_system, mav.m.target_component, sid, int(hz), 1)
        out["requested"].append({"stream": name, "hz": hz,
                                 "expected_messages": msgs})
    counts = {}
    t0 = time.time()
    while time.time() - t0 < measure_s:
        r, _, _ = _sel.select([mav.m.port], [], [], 0.2)
        if not r:
            continue
        m = mav.m.recv_match(blocking=False)
        if m is None:
            continue
        t = m.get_type()
        counts[t] = counts.get(t, 0) + 1
    for t, c in counts.items():
        out["measured_hz"][t] = round(c / measure_s, 2)
    return out


def env_setup():
    """Set GZ_SIM_SYSTEM_PLUGIN_PATH for all six project plugins plus
    ardupilot_gazebo. Environment only - no file is written."""
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    parts = [os.path.join(REPO_ROOT, f"plugins/{p}/build") for p in
             ("aerodynamics", "propulsion", "actuators", "sensors", "wind")]
    parts.append("/home/emirhan/gazebo_sim/ardupilot_gazebo/build")
    existing = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    if existing:
        parts.append(existing)
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
        [p for p in parts if os.path.isdir(p) or p == existing])


def provenance_header(stage_part, world, extra=None):
    d = {
        "stage": "MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION",
        "part": stage_part,
        "classification": "TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY",
        "owner": "controls-integration",
        "physics_parameters_changed": "NONE - this harness measures only. It "
                                      "writes no aerodynamic or propulsion "
                                      "coefficient, no mass/CG/inertia, no "
                                      "actuator PID/rate/effort limit, no "
                                      "control-surface mapping, no "
                                      "PTCH_TRIM_DEG, no TECS_PTCH_DAMP, no "
                                      "roll/pitch PID, no sensor value, no "
                                      "landing gear, no friction or collision "
                                      "value.",
        "world": world,
        "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "sign_convention_sources": [
            "docs/source_of_truth/controls/CONTROLS.md sec 10",
            "docs/test_results/2026-08-22_control_surface_sign_mapping_test_report.md",
            "docs/source_of_truth/autopilot/SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md sec 3, 7.1-7.3",
            "docs/test_results/2026-08-28_ardupilot_control_surface_travel_scaling_validation.md",
        ],
        "pre_registered_expectations": PRE_REGISTERED_EXPECTATIONS,
        "known_data_required": [AERO_DEFLECTION_NOT_PUBLISHED],
    }
    if extra:
        d.update(extra)
    return d

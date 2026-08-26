#!/usr/bin/env python3
"""
FALCON V2 - shared helper library for the ACTUATOR_SERVO_MODEL_V1 live-Gazebo
test suite (gazebo-testing, 2026-08-23).

Mirrors tests/gazebo/scripts/aero_lib.py / propulsion_lib.py's structure and
conventions exactly, applied to plugins/actuators/ (the FalconV2Actuators
System plugin, controls-integration) instead of plugins/aerodynamics/ or
plugins/propulsion/. Reuses aero_lib.py / propulsion_lib.py directly for
everything not actuator-specific: hold_step() (holding a body-frame
linear/angular-velocity condition via an external force/torque controller,
the SAME primitive the aero/propulsion plugins themselves use to apply
force/torque), read_base_link_inertia(), aero_lib.compute_aero() (the
already-validated pure-Python mirror of AeroModel.hh, used here only as an
independent cross-check of the LIVE aerodynamics plugin's response to the
REAL, actuator-driven joint angle - never as a substitute for a live
measurement), aero_lib.DiagSubscriber (aerodynamics diagnostics topic),
propulsion_lib.ThrottleCommander (for the powered-trim-region flight-load
test, which needs BOTH motors at the established trim throttle).

Provides here, actuator-specific:
  1. setup_env() - GZ_SIM_SYSTEM_PLUGIN_PATH covering ALL THREE plugin build
     dirs (actuators + propulsion + aerodynamics), since model/model.sdf now
     attaches all three System plugins to the same model and this task's
     tests need all three running together (the actuator drives the real
     joint; the aerodynamics plugin reads that real joint's ACTUAL position
     every tick, unmodified; propulsion is along for the ride in the
     flight-load test's trim-region setup).
  2. load_actuator_config() - read-only parse of
     docs/source_of_truth/controls/actuator_v1_config.yaml (max_rate_rad_s,
     max_effort_nm, min/max_angle_rad, kp/kd) - used only to cross-check live
     diagnostics against the documented V1_PROVISIONAL constants, never
     modified.
  3. ActuatorCommander - a gz-transport Node with 5 per-surface Double
     publishers (one per docs/source_of_truth/controls/actuator_v1_config.yaml
     "command_interface.topics"), republished EVERY tick (never a one-shot
     publish) - mirrors propulsion_lib.ThrottleCommander's documented
     discovery-timing rationale (a single one-shot CLI-style publish can race
     gz-transport's background discovery window and be silently dropped,
     since these topics are not latched).
  4. DiagSubscriber - raw gz-transport subscriber for
     /model/falcon_v2/actuators/diagnostics (gz.msgs.Double_V, 35 fields: 7
     per surface x 5 surfaces, field order per ActuatorSystem.cc's Configure()
     gzmsg line / actuator_v1_config.yaml's
     command_interface.diagnostics_field_order_per_surface: cmd_rad,
     target_clamped_rad, setpoint_rad, actual_angle_rad, actual_rate_rad_s,
     target_clamp_active, effort_clamp_active, repeated for left_aileron,
     right_aileron, left_elevator, right_elevator, rudder in that order).
  5. read_joint_state() - direct ECM readback (ground truth, independent of
     the diagnostics topic's own periodic publish timing) of a single
     joint's (position, velocity), mirroring the enable_position_check() /
     enable_velocity_check() pattern already used throughout this suite
     (e.g. test_propulsion_transient_reaction_torque.py's
     lj.enable_velocity_check(ecm, True) usage).
  6. pin_other_control_surface_and_prop_joints() - pins every one of the 7
     lightweight/child joints (5 control surfaces + 2 props) EXCEPT one
     explicitly named "leave free" joint, to isolate a single actuator-driven
     surface's real dynamics from the other 6 un-actuated/free-spinning
     joints - the same empirically-justified isolation rationale as
     aero_lib.pin_child_joints()'s own header comment (a sustained nonzero
     base_link body rate can otherwise drag these near-massless placeholder
     joints to extreme relative velocities), specialized here to leave
     exactly one joint under the real actuator's own SetForce() control
     instead of forcing it via reset_position()/reset_velocity() every tick
     (which would fight the actuator plugin, defeating the entire point of
     testing it).
  7. surface_collision_geometry() - the per-surface collision-box local
     pose/size constants (link name, collision local pose, box size, and the
     nearest fixed base_link-frame collision box to compare against for the
     +/-45deg mechanical/mesh sanity check), taken VERBATIM from
     model/model.sdf's own <collision><pose>/<box><size> elements (cited
     inline per constant) - read-only reference data for a geometric
     plausibility check, never used to alter model.sdf.

No aircraft physics parameter is modified anywhere in this file.
"""
import math
import os
import threading

import aero_lib as AL
import propulsion_lib as PL

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
ACTUATOR_PLUGIN_BUILD_DIR = os.path.join(REPO_ROOT, "plugins/actuators/build")
ACTUATOR_CONFIG_YAML_PATH = os.path.join(
    REPO_ROOT, "docs/source_of_truth/controls/actuator_v1_config.yaml")

STEP = AL.STEP  # 0.001 s, matches every test world's <max_step_size>

DIAG_TOPIC = "/model/falcon_v2/actuators/diagnostics"
SURFACES = ["left_aileron", "right_aileron", "left_elevator", "right_elevator", "rudder"]
JOINT_NAMES = {
    "left_aileron": "left_aileron_joint", "right_aileron": "right_aileron_joint",
    "left_elevator": "left_elevator_joint", "right_elevator": "right_elevator_joint",
    "rudder": "rudder_joint",
}
# Per docs/source_of_truth/controls/actuator_v1_config.yaml "command_interface.topics".
CMD_TOPICS = {
    "left_aileron": "/model/falcon_v2/actuators/left_aileron/cmd_rad",
    "right_aileron": "/model/falcon_v2/actuators/right_aileron/cmd_rad",
    "left_elevator": "/model/falcon_v2/actuators/left_elevator/cmd_rad",
    "right_elevator": "/model/falcon_v2/actuators/right_elevator/cmd_rad",
    "rudder": "/model/falcon_v2/actuators/rudder/cmd_rad",
}
# Per actuator_v1_config.yaml "command_interface.diagnostics_field_order_per_surface".
DIAG_FIELDS = ["cmd_rad", "target_clamped_rad", "setpoint_rad", "actual_angle_rad",
               "actual_rate_rad_s", "target_clamp_active", "effort_clamp_active"]

CHILD_JOINTS = list(AL.CHILD_JOINTS)  # 5 control surfaces + 2 props, aero_lib.py


def setup_env():
    """Must be called BEFORE `import gz.sim8` / creating any TestFixture.
    Adds all THREE plugin build dirs to GZ_SIM_SYSTEM_PLUGIN_PATH."""
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    existing = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    parts = [ACTUATOR_PLUGIN_BUILD_DIR, PL.PROP_PLUGIN_BUILD_DIR, AL.PLUGIN_BUILD_DIR]
    if existing:
        parts.append(existing)
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(parts)


def load_actuator_config():
    """Read-only parse of actuator_v1_config.yaml - used only to cross-check
    live diagnostics against the documented V1_PROVISIONAL constants
    (max_rate_rad_s, max_effort_nm, min/max_angle_rad). Never modified."""
    import yaml
    with open(ACTUATOR_CONFIG_YAML_PATH) as f:
        root = yaml.safe_load(f)
    surf = root["surfaces"]["left_aileron"]  # identical across all 5 surfaces in V1
    law = root["control_law"]
    return dict(
        min_angle_rad=surf["min_angle_rad"], max_angle_rad=surf["max_angle_rad"],
        max_rate_rad_s=surf["max_rate_rad_s"], max_effort_nm=surf["max_effort_nm"],
        kp=law["kp_nm_per_rad"], kd=law["kd_nm_per_rad_s"])


class ActuatorCommander:
    """Publishes gz.msgs.Double per surface EVERY tick (never a one-shot
    publish) - see module docstring for the discovery-timing rationale
    (mirrors propulsion_lib.ThrottleCommander). commands[s] in radians;
    defaults to 0.0 (neutral) for any surface never explicitly set() this
    run, matching the plugin's own documented neutral-start/hold-last-valid
    semantics."""

    def __init__(self):
        import gz.transport13 as tp
        from gz.msgs10 import double_pb2
        self._double_pb2 = double_pb2
        self.node = tp.Node()
        self.pubs = {s: self.node.advertise(CMD_TOPICS[s], double_pb2.Double) for s in SURFACES}
        self.commands = {s: 0.0 for s in SURFACES}

    def set(self, **kwargs):
        """set(left_aileron=rad, right_elevator=rad, rudder=rad, ...)."""
        for k, v in kwargs.items():
            assert k in SURFACES, f"unknown surface '{k}', expected one of {SURFACES}"
            self.commands[k] = v

    def tick(self):
        for s in SURFACES:
            m = self._double_pb2.Double()
            m.data = self.commands[s]
            self.pubs[s].publish(m)


class DiagSubscriber:
    """Subscribes to /model/falcon_v2/actuators/diagnostics (Double_V, 35
    fields = 7 fields x 5 surfaces, order per DIAG_FIELDS/SURFACES above)."""

    def __init__(self):
        import gz.transport13 as tp
        self._tp = tp
        self.node = tp.Node()
        self.lock = threading.Lock()
        self.history = []
        ok = self.node.subscribe_raw(
            DIAG_TOPIC, self._cb, "gz.msgs.Double_V", tp.SubscribeOptions())
        if not ok:
            raise RuntimeError(f"Failed to subscribe to {DIAG_TOPIC}")

    def _cb(self, data, info):
        from gz.msgs10 import double_v_pb2
        m = double_v_pb2.Double_V()
        m.ParseFromString(data)
        vals = list(m.data)
        with self.lock:
            self.history.append(vals)

    def count(self):
        with self.lock:
            return len(self.history)

    @classmethod
    def _split(cls, vals):
        out = {}
        for i, s in enumerate(SURFACES):
            out[s] = dict(zip(DIAG_FIELDS, vals[i * 7:(i + 1) * 7]))
        return out

    def latest(self):
        with self.lock:
            if not self.history:
                return None
            return self._split(self.history[-1])

    def all_split(self):
        with self.lock:
            return [self._split(v) for v in self.history]


def read_joint_state(model, ecm, sim, joint_name):
    """Direct ECM readback (ground truth) of (position_rad, velocity_rad_s)
    for a single joint - independent of the diagnostics topic's own
    publish-rate timing."""
    je = model.joint_by_name(ecm, joint_name)
    if je is None:
        return None, None
    j = sim.Joint(je)
    j.enable_position_check(ecm, True)
    j.enable_velocity_check(ecm, True)
    pos = j.position(ecm)
    vel = j.velocity(ecm)
    return (pos[0] if pos else None), (vel[0] if vel else None)


def pin_other_child_joints(model, ecm, sim, leave_free_joints, positions=None):
    """Pins every one of the 7 CHILD_JOINTS (aero_lib.CHILD_JOINTS: 5
    control-surface + 2 prop joints) to `positions.get(jn, 0.0)` EXCEPT the
    joint(s) named in `leave_free_joints` (a single joint-name string, or an
    iterable of joint-name strings), which are left completely untouched (no
    reset_position/reset_velocity call at all) so the real actuator plugin's
    own SetForce()-driven dynamics on those joints are never fought/
    overridden by a kinematic pin. See module docstring point 6."""
    positions = positions or {}
    if isinstance(leave_free_joints, str):
        leave_free_joints = {leave_free_joints}
    else:
        leave_free_joints = set(leave_free_joints)
    for jn in CHILD_JOINTS:
        if jn in leave_free_joints:
            continue
        je = model.joint_by_name(ecm, jn)
        if je is None:
            continue
        j = sim.Joint(je)
        j.reset_position(ecm, [positions.get(jn, 0.0)])
        j.reset_velocity(ecm, [0.0])


# =============================================================================
# Collision-box geometry constants for the +/-45deg mechanical/mesh sanity
# check - taken VERBATIM from model/model.sdf's own <collision><pose>/
# <box><size> elements (grep-verified against that file for this task, cited
# by line-content, not line-number, since line numbers shift). Local pose is
# relative to the owning link's own frame; collision box rotation is
# identity in every case below (SDF <collision><pose> has zero rpy for all 5
# control surfaces and the two reference boxes), so box edges are aligned
# with the owning link's own local axes - corner computation below only
# needs to add/subtract half-extents in local frame before transforming by
# the link's world pose.
# =============================================================================
SURFACE_COLLISION = {
    "left_aileron": dict(link="left_aileron", local_pose=(-0.025962, 0.236107, 0.007807),
                          size=(0.061119, 0.479366, 0.020953),
                          ref_name="left_wing_collision", ref_local_pose=(0.105743, 0.566014, 0.127019),
                          ref_size=(0.266510, 0.972027, 0.046956)),
    "right_aileron": dict(link="right_aileron", local_pose=(-0.025962, -0.236107, 0.007807),
                           size=(0.061119, 0.479366, 0.020953),
                           ref_name="right_wing_collision", ref_local_pose=(0.105743, -0.566014, 0.127019),
                           ref_size=(0.266510, 0.972027, 0.046956)),
    "left_elevator": dict(link="left_elevator", local_pose=(-0.020211, 0.094700, -0.004031),
                           size=(0.044973, 0.189400, 0.009066),
                           ref_name="fuselage_collision_tail", ref_local_pose=(-0.499782, 0, 0.165240),
                           ref_size=(0.054196, 0.560000, 0.330480)),
    "right_elevator": dict(link="right_elevator", local_pose=(-0.020211, -0.094700, -0.004031),
                            size=(0.044973, 0.189400, 0.009066),
                            ref_name="fuselage_collision_tail", ref_local_pose=(-0.499782, 0, 0.165240),
                            ref_size=(0.054196, 0.560000, 0.330480)),
    "rudder": dict(link="rudder", local_pose=(-0.019247, 0.0, 0.084500),
                    size=(0.045316, 0.008911, 0.169500),
                    ref_name="fuselage_collision_tail", ref_local_pose=(-0.499782, 0, 0.165240),
                    ref_size=(0.054196, 0.560000, 0.330480)),
}

# Link ORIGIN local pose (relative to base_link), i.e. the joint pivot point
# for each of the 5 control-surface joints - SDF joints below have no
# separate <joint><pose> element, so the joint frame coincides exactly with
# the child link's own (undisplaced) frame; this point must stay fixed in
# base_link's frame at every commanded angle (pure rotation about the hinge
# axis never translates its own origin) - the "hinge break" invariant check.
LINK_ORIGIN_LOCAL = {
    "left_aileron": (0.032943, 0.313950, 0.110356),
    "right_aileron": (0.032943, -0.313950, 0.110356),
    "left_elevator": (-0.474959, 0.050600, 0.087119),
    "right_elevator": (-0.474959, -0.050600, 0.087119),
    "rudder": (-0.476094, 0.000000, 0.130500),
}


def box_world_corners(gm, wpose, local_center, size):
    """8 corners of an axis-aligned (in LOCAL frame) box, transformed to
    world frame via wpose (gz.math7.Pose3d)."""
    cx, cy, cz = local_center
    sx, sy, sz = size
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    corners = []
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            for dz in (-hz, hz):
                local = gm.Vector3d(cx + dx, cy + dy, cz + dz)
                world = wpose.pos() + wpose.rot().rotate_vector(local)
                corners.append((world.x(), world.y(), world.z()))
    return corners


def aabb_of(corners):
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    zs = [c[2] for c in corners]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def aabb_overlap(a, b):
    """a, b = ((xmin,xmax),(ymin,ymax),(zmin,zmax)). Returns
    (overlap_bool, overlap_extent_xyz) - overlap_extent is the positive
    overlap length on each axis (0 or negative = no overlap on that axis)."""
    ext = []
    ok = True
    for (amin, amax), (bmin, bmax) in zip(a, b):
        e = min(amax, bmax) - max(amin, bmin)
        ext.append(e)
        if e <= 0:
            ok = False
    return ok, tuple(ext)

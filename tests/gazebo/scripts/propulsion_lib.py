#!/usr/bin/env python3
"""
FALCON V2 - shared helper library for the PROPULSION_V1_IMPLEMENTATION
live-Gazebo test suite (gazebo-testing, 2026-08-23).

Mirrors tests/gazebo/scripts/aero_lib.py's structure/conventions exactly,
applied to plugins/propulsion/ instead of plugins/aerodynamics/. Reuses
aero_lib.py directly (imported below) for the pieces that are not
propulsion-specific: setup_env()'s plugin-path mechanism (extended here to
add BOTH plugin build dirs, since model/model.sdf now carries both the
aerodynamics AND propulsion plugins), hold_step() (holding a body-frame
velocity condition via an external force/torque controller), and
read_base_link_inertia(). A NEW, propulsion-specific pin helper
(pin_control_surface_joints) is added here rather than reusing
aero_lib.pin_child_joints() as-is, because that helper pins ALL 7 child
joints INCLUDING the two prop joints - which would fight the propulsion
plugin's own per-tick Joint.SetVelocity() call on those exact two joints.
Propulsion tests that need the 5 lightweight, unactuated, undamped
control-surface joints held still (same empirically-justified rationale as
aero_lib's own header comment) use pin_control_surface_joints() instead,
leaving left_prop_joint/right_prop_joint entirely owned by the plugin.

Provides:
  1. setup_env() - GZ_SIM_SYSTEM_PLUGIN_PATH covering both plugin build
     dirs, protobuf python-implementation env var (same requirement as
     aero_lib.py).
  2. load_config() - parses
     docs/source_of_truth/propulsion/propulsion_v1_config.yaml into a
     PropulsionConfig object (motor/battery/propeller/physics/geometry/
     rotation/interfaces), read-only, never modified.
  3. load_apc_table() - pure-Python, Gazebo-independent re-parse of the
     REAL parsed APC CSV (apc_13x65e_parsed.csv), mirroring
     PropulsionModel.hh::LoadApcTableCsv() line-for-line (grouping by RPM,
     verifying strictly-increasing J per slice, sorting slices by RPM).
  4. A pure-Python mirror of the rest of PropulsionModel.hh (motor_kt,
     motor_electrical, apply_rpm_cap, integrate_rotor_step, advance_ratio,
     lookup_ct_cp [2-stage interpolation], prop_thrust, prop_power,
     prop_torque_from_power, prop_aero_load) - used ONLY as an independent
     hand-calculation cross-check against live-measured plugin behavior,
     exactly the same role aero_lib.compute_aero() plays for the
     aerodynamics suite. Never used as a substitute for a live measurement.
  5. DiagSubscriber - raw gz-transport subscriber for
     /model/falcon_v2/propulsion/diagnostics (gz.msgs.Double_V, 28 fields:
     14 per motor, left then right, field order per PropulsionSystem.cc's
     Configure() gzmsg line / model.sdf's propulsion <plugin> comment).
  6. ThrottleCommander - a gz-transport Node with left/right throttle
     Double publishers. Verified empirically this pass (see test report):
     publishing from this SAME Python process, repeatedly, inside a
     TestFixture's on_pre_update callback (i.e. across the real wall-clock
     duration of a server.run() call, not as a single fire-and-forget
     message) reliably reaches the plugin's subscriber once gz-transport's
     background discovery completes - the DiagSubscriber pattern already
     established by aero_lib.py demonstrates the same discovery-then-
     delivery timing in the opposite (plugin->script) direction. A single
     one-shot publish (e.g. the CLI `gz topic pub` one-shot documented in
     plugins/propulsion/README.md) can race that discovery window and be
     silently dropped (gz-transport topics are not latched) - this is the
     investigated explanation for `propulsion`'s reported "left throttle
     one-shot publish did not appear to take effect" observation (see this
     suite's LEFT_PROP_VISUAL_OMEGA_TEST report section).
  7. pin_control_surface_joints() - pins ONLY the 5 control-surface joints
     (never the 2 prop joints, see class-level docstring above).

No aircraft physics parameter is modified anywhere in this file.
"""
import csv
import io
import math
import os
import threading

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
AERO_PLUGIN_BUILD_DIR = os.path.join(REPO_ROOT, "plugins/aerodynamics/build")
PROP_PLUGIN_BUILD_DIR = os.path.join(REPO_ROOT, "plugins/propulsion/build")
CONFIG_YAML_PATH = os.path.join(
    REPO_ROOT, "docs/source_of_truth/propulsion/propulsion_v1_config.yaml")
WORLD_SDF = os.path.join(
    REPO_ROOT, "tests/gazebo/worlds/falcon_v2_zero_g_world.sdf")
DIAG_TOPIC = "/model/falcon_v2/propulsion/diagnostics"
LEFT_THROTTLE_TOPIC = "/model/falcon_v2/propulsion/left/throttle_cmd"
RIGHT_THROTTLE_TOPIC = "/model/falcon_v2/propulsion/right/throttle_cmd"

STEP = 0.001  # s, matches falcon_v2_zero_g_world.sdf's <max_step_size>


def setup_env():
    """Must be called BEFORE `import gz.sim8` / creating any TestFixture in
    the calling script. Adds BOTH plugin build dirs to
    GZ_SIM_SYSTEM_PLUGIN_PATH since model/model.sdf attaches both the
    aerodynamics and propulsion plugins to the same model."""
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    existing = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    parts = [PROP_PLUGIN_BUILD_DIR, AERO_PLUGIN_BUILD_DIR]
    if existing:
        parts.append(existing)
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(parts)


# =============================================================================
# Config loader - mirrors PropulsionSystem.cc::LoadConfig()'s YAML keys.
# =============================================================================
class PropulsionConfig:
    pass


def load_config():
    import yaml
    with open(CONFIG_YAML_PATH) as f:
        root = yaml.safe_load(f)

    cfg = PropulsionConfig()
    motor = root["motor"]
    cfg.kv_rpm_per_v = motor["kv_rpm_per_v"]
    cfg.resistance_ohm = motor["resistance_ohm"]
    cfg.no_load_current_A = motor["no_load_current_A"]
    cfg.no_load_current_ref_voltage_V = motor["no_load_current_ref_voltage_V"]
    cfg.current_limit_A = motor["current_limit_A"]
    cfg.power_limit_W = motor["power_limit_W"]

    battery = root["battery"]
    cfg.v_nominal_V = battery["v_nominal_V"]
    cfg.v1_operating_voltage_V = battery["v1_operating_voltage_V"]

    prop = root["propeller"]
    cfg.diameter_m = prop["diameter_m"]
    cfg.apc_parsed_csv_path = os.path.join(REPO_ROOT, prop["apc_parsed_csv_path"])
    cfg.apc_raw_data_path = os.path.join(REPO_ROOT, prop["apc_raw_data_path"])
    cfg.rpm_cap_v1 = prop["rpm_cap_v1"]
    cfg.rpm_table_min = prop["rpm_table_min"]
    cfg.rpm_table_max = prop["rpm_table_max"]

    physics = root["physics"]
    cfg.rho = physics["rho_kg_m3"]
    cfg.n_safe_floor_rev_s = physics["n_safe_floor_rev_s"]
    cfg.i_rotor_kg_m2 = physics["i_rotor_kg_m2"]

    geom = root["geometry"]
    cfg.left_hub_m = tuple(geom["left_hub_m"])
    cfg.right_hub_m = tuple(geom["right_hub_m"])
    cfg.thrust_axis_body = tuple(geom["thrust_axis_body"])

    rotation = root["rotation"]
    cfg.rotation_left_sign = rotation["left_sign"]
    cfg.rotation_right_sign = rotation["right_sign"]

    interfaces = root["interfaces"]
    cfg.left_throttle_topic = interfaces["left_throttle_topic"]
    cfg.right_throttle_topic = interfaces["right_throttle_topic"]
    cfg.diagnostics_topic = interfaces["diagnostics_topic"]
    cfg.diagnostics_rate_hz = interfaces["diagnostics_rate_hz"]

    return cfg


# =============================================================================
# Pure-Python mirror of PropulsionModel.hh - independent cross-check only.
# =============================================================================
def load_apc_table(csv_path):
    """Mirrors PropulsionModel.hh::LoadApcTableCsv(). Returns a list of
    slice dicts {rpm, J:[...], Ct:[...], Cp:[...]} sorted ascending by rpm,
    each slice's J array verified strictly increasing (raises AssertionError
    otherwise, matching the C++ load-time sanity check)."""
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        rows = list(reader)
    header, data_rows = rows[0], rows[1:]

    order = []
    raw = {}
    for row in data_rows:
        if not row:
            continue
        vals = [float(x) for x in row]
        rpm = vals[0]
        j, ct, cp = vals[2], vals[3], vals[4]
        if rpm not in raw:
            order.append(rpm)
            raw[rpm] = {"J": [], "Ct": [], "Cp": []}
        raw[rpm]["J"].append(j)
        raw[rpm]["Ct"].append(ct)
        raw[rpm]["Cp"].append(cp)

    slices = []
    for rpm in order:
        s = raw[rpm]
        for i in range(1, len(s["J"])):
            assert s["J"][i] > s["J"][i - 1], (
                f"non-monotonic J within RPM slice {rpm} at index {i}")
        slices.append({"rpm": rpm, "J": s["J"], "Ct": s["Ct"], "Cp": s["Cp"]})
    slices.sort(key=lambda s: s["rpm"])
    return slices


def motor_kt(kv_rpm_per_v):
    if kv_rpm_per_v <= 0.0:
        return 0.0
    return 60.0 / (2.0 * math.pi * kv_rpm_per_v)


def motor_electrical(cfg, throttle01, v_battery, omega):
    """Returns dict(current_A, torque_Nm, negative_current_clamped,
    current_limited). Mirrors MotorElectrical()."""
    throttle = max(0.0, min(1.0, throttle01))
    kt = motor_kt(cfg.kv_rpm_per_v)
    ke = kt
    v_esc = throttle * v_battery
    i_raw = 0.0
    if cfg.resistance_ohm > 0.0:
        i_raw = (v_esc - ke * omega) / cfg.resistance_ohm
    i_used = i_raw
    neg_clamped = False
    if i_used < 0.0:
        i_used = 0.0
        neg_clamped = True
    cur_limited = False
    if cfg.current_limit_A > 0.0 and i_used > cfg.current_limit_A:
        i_used = cfg.current_limit_A
        cur_limited = True
    torque_Nm = kt * (i_used - cfg.no_load_current_A)
    return dict(current_A=i_used, torque_Nm=torque_Nm,
                negative_current_clamped=neg_clamped, current_limited=cur_limited)


def apply_rpm_cap(omega, rpm_cap):
    if rpm_cap <= 0.0:
        return omega, False
    omega_cap = rpm_cap * 2.0 * math.pi / 60.0
    if omega > omega_cap:
        return omega_cap, True
    if omega < -omega_cap:
        return -omega_cap, True
    return omega, False


def integrate_rotor_step(omega_prev, q_motor, q_prop_signed, i_rotor, dt, rpm_cap):
    omega_new = omega_prev
    if i_rotor > 0.0:
        omega_new = omega_prev + dt * (q_motor - q_prop_signed) / i_rotor
    omega_capped, cap_active = apply_rpm_cap(omega_new, rpm_cap)
    rpm = omega_capped * 60.0 / (2.0 * math.pi)
    return dict(omega=omega_capped, rpm=rpm, rpm_cap_active=cap_active)


def advance_ratio(v_axial, n, diameter_m, n_safe_floor):
    n_safe = max(n, n_safe_floor)
    if diameter_m <= 0.0:
        return 0.0
    return v_axial / (n_safe * diameter_m)


def _interp_within_slice(slice_, j):
    js, cts, cps = slice_["J"], slice_["Ct"], slice_["Cp"]
    if not js:
        return 0.0, 0.0, False
    if len(js) == 1:
        return cts[0], cps[0], False
    jq = j
    j_clamped = False
    if jq < js[0]:
        jq = js[0]
        j_clamped = True
    elif jq > js[-1]:
        jq = js[-1]
        j_clamped = True
    hi = 0
    while hi < len(js) and js[hi] < jq:
        hi += 1
    if hi == 0:
        return cts[0], cps[0], j_clamped
    if hi >= len(js):
        return cts[-1], cps[-1], j_clamped
    lo = hi - 1
    j_lo, j_hi = js[lo], js[hi]
    span = j_hi - j_lo
    f = (jq - j_lo) / span if span > 0.0 else 0.0
    ct = cts[lo] + f * (cts[hi] - cts[lo])
    cp = cps[lo] + f * (cps[hi] - cps[lo])
    return ct, cp, j_clamped


def lookup_ct_cp(slices, rpm_query, j):
    """Mirrors LookupCtCp()'s 2-stage interpolation exactly."""
    if not slices:
        return 0.0, 0.0, False
    rpm = rpm_query
    interp_clamped = False
    min_rpm, max_rpm = slices[0]["rpm"], slices[-1]["rpm"]
    if rpm < min_rpm:
        rpm = min_rpm
        interp_clamped = True
    elif rpm > max_rpm:
        rpm = max_rpm
        interp_clamped = True

    hi = 0
    while hi < len(slices) and slices[hi]["rpm"] < rpm:
        hi += 1
    if hi == 0:
        lo = 0
    elif hi >= len(slices):
        hi = len(slices) - 1
        lo = hi
    else:
        lo = hi - 1

    ct_lo, cp_lo, jc_lo = _interp_within_slice(slices[lo], j)
    ct_hi, cp_hi, jc_hi = _interp_within_slice(slices[hi], j)
    rpm_lo, rpm_hi = slices[lo]["rpm"], slices[hi]["rpm"]
    span = rpm_hi - rpm_lo
    f = (rpm - rpm_lo) / span if span > 0.0 else 0.0
    ct = ct_lo + f * (ct_hi - ct_lo)
    cp = cp_lo + f * (cp_hi - cp_lo)
    return ct, cp, (interp_clamped or jc_lo or jc_hi)


def prop_thrust(ct, rho, n, diameter_m):
    d2 = diameter_m * diameter_m
    return ct * rho * n * n * (d2 * d2)


def prop_power(cp, rho, n, diameter_m):
    d2 = diameter_m * diameter_m
    return cp * rho * n * n * n * (d2 * d2 * diameter_m)


def prop_torque_from_power(p_prop, omega_abs, omega_safe_floor):
    omega_safe = max(omega_abs, omega_safe_floor)
    return p_prop / omega_safe


def prop_aero_load(slices, omega, v_axial, diameter_m, rho, n_safe_floor_rev_s):
    """Mirrors PropAeroLoad(). Returns a dict with the same fields as
    PropLoadResult."""
    omega_abs = abs(omega)
    n = omega_abs / (2.0 * math.pi)
    rpm = omega * 60.0 / (2.0 * math.pi)
    j = advance_ratio(v_axial, n, diameter_m, n_safe_floor_rev_s)
    rpm_query = n * 60.0
    ct, cp, interp_clamped = lookup_ct_cp(slices, rpm_query, j)
    thrust_N = prop_thrust(ct, rho, n, diameter_m)
    p_prop_W = prop_power(cp, rho, n, diameter_m)
    omega_safe_floor = 2.0 * math.pi * n_safe_floor_rev_s
    q_prop_mag = prop_torque_from_power(p_prop_W, omega_abs, omega_safe_floor)
    sign = 1.0 if omega >= 0.0 else -1.0
    q_prop_signed = sign * q_prop_mag
    return dict(n=n, rpm=rpm, J=j, Ct=ct, Cp=cp, thrust_N=thrust_N,
                pProp_W=p_prop_W, qPropMagnitude_Nm=q_prop_mag,
                qPropSigned_Nm=q_prop_signed, interpClamped=interp_clamped)


# =============================================================================
# Diagnostics topic subscriber (28 fields: 14 per motor, left then right).
# =============================================================================
class DiagSubscriber:
    MOTOR_FIELDS = ["throttle", "current_A", "Q_motor_Nm", "omega_rad_s", "rpm",
                     "J", "Ct", "Cp", "thrust_N", "Q_prop_Nm", "interpClamped",
                     "rpmCapActive", "negativeCurrentClamped", "currentLimited"]

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
        left = dict(zip(cls.MOTOR_FIELDS, vals[0:14]))
        right = dict(zip(cls.MOTOR_FIELDS, vals[14:28]))
        return {"left": left, "right": right}

    def latest(self):
        with self.lock:
            if not self.history:
                return None
            return self._split(self.history[-1])

    def all_split(self):
        with self.lock:
            return [self._split(v) for v in self.history]


# =============================================================================
# Throttle commander - see module docstring re: repeated-publish-during-run
# discovery timing rationale.
# =============================================================================
class ThrottleCommander:
    def __init__(self, left_topic=LEFT_THROTTLE_TOPIC, right_topic=RIGHT_THROTTLE_TOPIC):
        import gz.transport13 as tp
        from gz.msgs10 import double_pb2
        self._double_pb2 = double_pb2
        self.node = tp.Node()
        self.pub_left = self.node.advertise(left_topic, double_pb2.Double)
        self.pub_right = self.node.advertise(right_topic, double_pb2.Double)
        self.left = 0.0
        self.right = 0.0

    def set(self, left=None, right=None):
        if left is not None:
            self.left = left
        if right is not None:
            self.right = right

    def tick(self):
        """Call every on_pre_update tick - publishes the CURRENT commanded
        left/right throttle values every step (never a one-shot publish),
        so gz-transport discovery timing can never cause a dropped
        command (see module docstring)."""
        ml = self._double_pb2.Double()
        ml.data = self.left
        self.pub_left.publish(ml)
        mr = self._double_pb2.Double()
        mr.data = self.right
        self.pub_right.publish(mr)


# =============================================================================
# Pin ONLY the 5 control-surface joints (never the 2 prop joints - see
# module docstring for why aero_lib.pin_child_joints() is not reused here).
# =============================================================================
CONTROL_SURFACE_JOINTS = ["left_aileron_joint", "right_aileron_joint",
                           "left_elevator_joint", "right_elevator_joint",
                           "rudder_joint"]


def pin_control_surface_joints(model, ecm, sim, positions=None):
    positions = positions or {}
    for jn in CONTROL_SURFACE_JOINTS:
        je = model.joint_by_name(ecm, jn)
        if je is None:
            continue
        j = sim.Joint(je)
        j.reset_position(ecm, [positions.get(jn, 0.0)])
        j.reset_velocity(ecm, [0.0])

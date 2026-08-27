#!/usr/bin/env python3
"""
FALCON V2 - SENSOR_MODEL_AND_ARDUPLANE_SITL_PREPARATION live validation
(gazebo-testing, 2026-08-27).

Live-Gazebo confirmation of the new IMU/GPS/baro/mag native sensors + the
custom FalconV2Pitot plugin (`controls-integration`, this stage) - reusing
`tests/gazebo/worlds/falcon_v2_sensors_selftest_world.sdf` (the world that
loads the native gz-sim8 Imu/NavSat/AirPressure/Magnetometer systems, per
SENSORS.md sec 5) plus the actuator/propulsion/aero/wind plugin build dirs
already used throughout this suite.

Scope, per task brief: LIVE VALIDATION ONLY. No aircraft physics parameter
is read for any purpose other than reusing already-validated trim/hold
constants (verbatim from docs/test_results/2026-08-26_updated_powered_trim_
high_deflection_validation.md and test_wind_gust_disturbance.py's own
already-validated U_HOLD/W_HOLD), and none is modified anywhere in this
script. No closed-loop ArduPlane flight (no FBWA/AUTO/LOITER/RTL/takeoff/
autotune) is exercised - every "hold"/"release" technique below is the same
plain Python proportional force/torque controller (`aero_lib.hold_step()`)
already used throughout this test suite, never ArduPilotPlugin/MAVLink.

Techniques reused verbatim, not reinvented:
  - setup_env() / ActuatorCommander / ThrottleCommander / DiagSubscriber
    (actuator_lib.py, propulsion_lib.py, aero_lib.py).
  - hold_step() proportional force/torque controller for "hold a body-frame
    condition, then sample" cases (same primitive AerodynamicsSystem.cc
    itself uses - AddWorldForce/AddWorldWrench).
  - WindCommander pattern (mirrors test_wind_gust_disturbance.py's own
    WindCommander - re-declared here rather than imported, since that file
    is a standalone script, not a shared library, matching this suite's
    existing convention of small per-script copies of this exact class).

New technique this script adds:
  - RawSub: a single generic gz-transport raw-subscribe wrapper parameterized
    by (topic, module, class) instead of one bespoke class per message type
    (IMU/NavSat/FluidPressure/Magnetometer/Double all handled by one class).
  - Timing-paired sampling for the baro free-fall regression and the GPS
    free-fall/forward-motion cases: the TRUE (ECM-queried) state is recorded
    at the SAME on_post_update tick a NEW sensor message is observed
    (sub.count() increments), avoiding the exact 20Hz-vs-per-tick staleness
    artifact `validation` already found and `gazebo-testing` fixed in the
    2026-08-26 pulse-test report (see that report's Part 14) - not
    reinvented blind, a known pitfall in this exact codebase deliberately
    avoided here from the start.

New world file this script uses (created this task, additive only):
  tests/gazebo/worlds/falcon_v2_sensors_selftest_altmag_world.sdf - an EXACT
  copy of controls-integration's falcon_v2_sensors_selftest_world.sdf with
  ONLY <magnetic_field> scaled 100x, used for the MAG_FIELD_SOURCE_REVIEW_
  REQUIRED live diagnostic (does the sensor's output scale with the world's
  declared field or not).
"""
import json
import math
import os
import sys
import threading

import actuator_lib as ACT
import aero_lib as AL
import propulsion_lib as PL

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
SENSORS_PLUGIN_BUILD_DIR = os.path.join(REPO_ROOT, "plugins/sensors/build")


def setup_env():
    """Must be called BEFORE `import gz.sim8` / creating any TestFixture.
    Extends actuator_lib.setup_env() (actuators+propulsion+aerodynamics)
    with the new plugins/wind and plugins/sensors build dirs - the FULL set
    of model-level plugins `model/model.sdf` now declares."""
    ACT.setup_env()
    existing = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    wind_dir = os.path.join(REPO_ROOT, "plugins/wind/build")
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
        [SENSORS_PLUGIN_BUILD_DIR, wind_dir] +
        ([existing] if existing else []))


setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

RESULTS_DIR = f"{REPO_ROOT}/tests/gazebo/results"
WORLDS_DIR = f"{REPO_ROOT}/tests/gazebo/worlds"
WORLD = f"{WORLDS_DIR}/falcon_v2_sensors_selftest_world.sdf"
WORLD_ALTMAG = f"{WORLDS_DIR}/falcon_v2_sensors_selftest_altmag_world.sdf"

IMU_TOPIC = "/model/falcon_v2/sensors/imu"
GPS_TOPIC = "/model/falcon_v2/sensors/gps"
BARO_TOPIC = "/model/falcon_v2/sensors/baro"
MAG_TOPIC = "/model/falcon_v2/sensors/mag"
PITOT_V_TOPIC = "/model/falcon_v2/sensors/pitot/airspeed_mps"
PITOT_P_TOPIC = "/model/falcon_v2/sensors/pitot/differential_pressure_pa"
WIND_STEADY_TOPIC = "/model/falcon_v2/wind/steady_cmd"

# =============================================================================
# Nominal trim, reused verbatim (NOT re-searched) from
# docs/test_results/2026-08-26_updated_powered_trim_high_deflection_validation.md
# and test_wind_gust_disturbance.py's own already-validated U_HOLD/W_HOLD.
# =============================================================================
THROTTLE = 0.5010
ELEV_THETA_DEG = 4.50
U_HOLD, W_HOLD = 18.14534, -0.78335
MASS = 5.9348  # kg, base_link mass - controller gain only, read-only
I_DIAG = (0.7284, 0.2507, 0.9523)  # kg*m^2, base_link diagonal inertia - controller gain only
KP_LIN = 150.0
KP_ANG = 1500.0  # reused verbatim from test_updated_powered_trim_high_deflection.py / test_wind_gust_disturbance.py
STEP = AL.STEP  # 0.001 s

DECLARED_MAG_FIELD_T = (5.5645e-6, 22.8758e-6, -42.3884e-6)
ALTMAG_FIELD_T = (5.5645e-4, 22.8758e-4, -42.3884e-4)
P0_PA = 101325.0


def isa_pressure_pa(h_m):
    """Standard ISA barometric formula, exactly as specified in the task
    brief: P(h) = P0*(1 - 2.25577e-5*h)^5.25588, P0=101325 Pa."""
    return P0_PA * (1.0 - 2.25577e-5 * h_m) ** 5.25588


def vec3_norm(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    model_e = world.model_by_name(ecm, "falcon_v2")
    return sim.Model(model_e)


def log_open(name):
    path = f"{RESULTS_DIR}/{name}_log.txt"
    f = open(path, "w")

    def log(msg=""):
        print(msg)
        f.write(str(msg) + "\n")
        f.flush()
    return log, f


# =============================================================================
# Generic raw gz-transport subscriber, parameterized by (topic, proto module,
# proto class name) - one class handles IMU/NavSat/FluidPressure/
# Magnetometer/Double instead of one bespoke class per message type.
# =============================================================================
class RawSub:
    def __init__(self, topic, module_name, class_name):
        import gz.transport13 as tp
        import importlib
        mod = importlib.import_module(module_name)
        self._cls = getattr(mod, class_name)
        self._msg_type = f"gz.msgs.{class_name}"
        self.node = tp.Node()
        self.lock = threading.Lock()
        self.history = []
        ok = self.node.subscribe_raw(
            topic, self._cb, self._msg_type, tp.SubscribeOptions())
        if not ok:
            raise RuntimeError(f"Failed to subscribe to {topic} ({self._msg_type})")

    def _cb(self, data, info):
        m = self._cls()
        m.ParseFromString(data)
        with self.lock:
            self.history.append(m)

    def latest(self):
        with self.lock:
            return self.history[-1] if self.history else None

    def count(self):
        with self.lock:
            return len(self.history)

    def tail(self, n):
        with self.lock:
            return list(self.history[-n:])


def imu_sub():
    return RawSub(IMU_TOPIC, "gz.msgs10.imu_pb2", "IMU")


def gps_sub():
    return RawSub(GPS_TOPIC, "gz.msgs10.navsat_pb2", "NavSat")


def baro_sub():
    return RawSub(BARO_TOPIC, "gz.msgs10.fluid_pressure_pb2", "FluidPressure")


def mag_sub():
    return RawSub(MAG_TOPIC, "gz.msgs10.magnetometer_pb2", "Magnetometer")


def pitot_v_sub():
    return RawSub(PITOT_V_TOPIC, "gz.msgs10.double_pb2", "Double")


def pitot_p_sub():
    return RawSub(PITOT_P_TOPIC, "gz.msgs10.double_pb2", "Double")


class WindCommander:
    """Same pattern as test_wind_gust_disturbance.py's own WindCommander -
    re-declared here (standalone-script convention already used throughout
    this suite, not a shared-library import)."""

    def __init__(self):
        import gz.transport13 as tp
        from gz.msgs10 import vector3d_pb2
        self._v3 = vector3d_pb2
        self.node = tp.Node()
        self.pub = self.node.advertise(WIND_STEADY_TOPIC, vector3d_pb2.Vector3d)
        self.steady = (0.0, 0.0, 0.0)

    def set(self, x, y, z):
        self.steady = (x, y, z)

    def tick(self):
        m = self._v3.Vector3d()
        m.x, m.y, m.z = self.steady
        self.pub.publish(m)


# =============================================================================
# PART A/B/C/E - "hold at a body-frame condition, then sample" runner.
# Reused for BARO two-altitude comparison, MAG heading sweep + field-scale
# test, IMU static + known-rate cases, and PITOT wind-response cases.
# =============================================================================
def run_held_case(log, label, world, *, altitude=100.0, rpy=(0.0, 0.0, 0.0),
                   throttle=0.0, elev_deg=0.0, wind=(0.0, 0.0, 0.0),
                   lin_target=(0.0, 0.0, 0.0), ang_target=(0.0, 0.0, 0.0),
                   settle_steps=2000, tail_steps=400,
                   need_imu=False, need_gps=False, need_baro=False,
                   need_mag=False, need_pitot=False, need_aero_diag=False):
    state = {"n": 0, "teleported": False, "cmd": None, "thr": None, "wind_cmd": None,
             "subs": {}, "any_nan": False}

    def on_pre(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            r, p, y = rpy
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, altitude, r, p, y))
            state["teleported"] = True
            state["cmd"] = ACT.ActuatorCommander()
            state["thr"] = PL.ThrottleCommander()
            state["wind_cmd"] = WindCommander()
        state["thr"].set(left=throttle, right=throttle)
        state["thr"].tick()
        state["cmd"].set(left_elevator=math.radians(elev_deg), right_elevator=math.radians(elev_deg),
                          left_aileron=0.0, right_aileron=0.0, rudder=0.0)
        state["cmd"].tick()
        state["wind_cmd"].set(*wind)
        state["wind_cmd"].tick()
        AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(*lin_target), gm.Vector3d(*ang_target),
                     kp_lin=KP_LIN, kp_ang=KP_ANG)

    def on_post(info, ecm):
        if not state["subs"]:
            def try_sub(key, fn):
                try:
                    state["subs"][key] = fn()
                except Exception as e:
                    log(f"  [WARN] subscriber init failed for {key}: {e}")
            if need_imu:
                try_sub("imu", imu_sub)
            if need_gps:
                try_sub("gps", gps_sub)
            if need_baro:
                try_sub("baro", baro_sub)
            if need_mag:
                try_sub("mag", mag_sub)
            if need_pitot:
                try_sub("pitot_v", pitot_v_sub)
                try_sub("pitot_p", pitot_p_sub)
            if need_aero_diag:
                try_sub("aero", AL.DiagSubscriber)
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        wpose = base.world_pose(ecm)
        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        if wpose is not None and lv is not None and av is not None:
            vals = [wpose.pos().z(), lv.x(), lv.y(), lv.z(), av.x(), av.y(), av.z()]
            if any(math.isnan(v) or math.isinf(v) for v in vals):
                state["any_nan"] = True
            state["last_body"] = (wpose, lv, av)
        state["n"] += 1

    fixture = sim.TestFixture(world)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, settle_steps, False)

    result = {"label": label, "any_nan": state["any_nan"], "n_steps": state["n"]}
    subs = state["subs"]
    wpose, lv, av = state.get("last_body", (None, None, None))
    result["final_z"] = wpose.pos().z() if wpose else None
    result["final_lv"] = (lv.x(), lv.y(), lv.z()) if lv else None
    result["final_av"] = (av.x(), av.y(), av.z()) if av else None

    if "imu" in subs:
        tail = subs["imu"].tail(tail_steps)
        result["imu_count"] = subs["imu"].count()
        if tail:
            m = tail[-1]
            result["imu_last"] = dict(
                orientation=(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w),
                angular_velocity=(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z),
                linear_acceleration=(m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
            avz = [mm.angular_velocity.z for mm in tail]
            aax = [mm.linear_acceleration.x for mm in tail]
            aay = [mm.linear_acceleration.y for mm in tail]
            aaz = [mm.linear_acceleration.z for mm in tail]
            result["imu_tail_avg_angvel_z"] = sum(avz) / len(avz)
            result["imu_tail_avg_specforce"] = (sum(aax) / len(aax), sum(aay) / len(aay), sum(aaz) / len(aaz))
    if "baro" in subs:
        tail = subs["baro"].tail(tail_steps)
        result["baro_count"] = subs["baro"].count()
        result["baro_pressures"] = [m.pressure for m in tail]
        result["baro_avg"] = (sum(result["baro_pressures"]) / len(result["baro_pressures"])
                               if result["baro_pressures"] else None)
    if "mag" in subs:
        tail = subs["mag"].tail(tail_steps)
        result["mag_count"] = subs["mag"].count()
        if tail:
            xs = [m.field_tesla.x for m in tail]
            ys = [m.field_tesla.y for m in tail]
            zs = [m.field_tesla.z for m in tail]
            result["mag_avg"] = (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))
            result["mag_last"] = (tail[-1].field_tesla.x, tail[-1].field_tesla.y, tail[-1].field_tesla.z)
    if "gps" in subs:
        tail = subs["gps"].tail(tail_steps)
        result["gps_count"] = subs["gps"].count()
        if tail:
            m = tail[-1]
            result["gps_last"] = dict(lat=m.latitude_deg, lon=m.longitude_deg, alt=m.altitude,
                                       ve=m.velocity_east, vn=m.velocity_north, vu=m.velocity_up)
    if "pitot_v" in subs:
        tv = subs["pitot_v"].tail(tail_steps)
        tp_ = subs["pitot_p"].tail(tail_steps)
        result["pitot_v_count"] = subs["pitot_v"].count()
        result["pitot_v_avg"] = (sum(m.data for m in tv) / len(tv)) if tv else None
        result["pitot_p_avg"] = (sum(m.data for m in tp_) / len(tp_)) if tp_ else None
        result["pitot_v_tail"] = [m.data for m in tv]
    if "aero" in subs:
        ta = subs["aero"].history[-tail_steps:]
        if ta:
            result["aero_count"] = subs["aero"].count()
            result["aero_V_avg"] = sum(m["V"] for m in ta) / len(ta)
            result["aero_V_tail"] = [m["V"] for m in ta]
    return result


# =============================================================================
# PART D - free (unheld) motion runner for GPS. Timing-paired sampling:
# records the TRUE ECM state at the SAME on_post tick a NEW gps/baro message
# is observed (avoids the known 20Hz-vs-per-tick staleness pitfall this
# codebase already found and fixed once - 2026-08-26 report Part 14).
# =============================================================================
def run_free_motion_case(log, label, *, altitude=150.0, throttle=0.0, elev_deg=0.0,
                          release_hold_steps=0, total_steps=3000, need_gps=True,
                          need_baro=False):
    state = {"n": 0, "teleported": False, "cmd": None, "thr": None,
             "gps": None, "baro": None, "last_gps_n": 0, "last_baro_n": 0,
             "paired_gps": [], "paired_baro": [], "any_nan": False}

    def on_pre(info, ecm):
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, altitude, 0, 0, 0))
            state["teleported"] = True
            state["cmd"] = ACT.ActuatorCommander()
            state["thr"] = PL.ThrottleCommander()
        state["thr"].set(left=throttle, right=throttle)
        state["thr"].tick()
        state["cmd"].set(left_elevator=math.radians(elev_deg), right_elevator=math.radians(elev_deg),
                          left_aileron=0.0, right_aileron=0.0, rudder=0.0)
        state["cmd"].tick()
        if n < release_hold_steps:
            lin_target = gm.Vector3d(U_HOLD, 0.0, W_HOLD)
            AL.hold_step(base, ecm, MASS, I_DIAG, lin_target, gm.Vector3d(0, 0, 0),
                         kp_lin=KP_LIN, kp_ang=KP_ANG)
        # else: fully free - no hold_step call at all

    def on_post(info, ecm):
        if need_gps and state["gps"] is None:
            try:
                state["gps"] = gps_sub()
            except Exception as e:
                log(f"  [WARN] gps subscriber init failed: {e}")
        if need_baro and state["baro"] is None:
            try:
                state["baro"] = baro_sub()
            except Exception as e:
                log(f"  [WARN] baro subscriber init failed: {e}")
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        wpose = base.world_pose(ecm)
        lv = base.world_linear_velocity(ecm)
        if wpose is None or lv is None:
            state["n"] += 1
            return
        z, vx, vy, vz = wpose.pos().z(), lv.x(), lv.y(), lv.z()
        if any(math.isnan(v) or math.isinf(v) for v in [z, vx, vy, vz]):
            state["any_nan"] = True
        t = state["n"] * STEP
        if need_gps and state["gps"] is not None:
            c = state["gps"].count()
            if c > state["last_gps_n"]:
                state["last_gps_n"] = c
                state["paired_gps"].append((t, z, vx, vy, vz, state["gps"].latest()))
        if need_baro and state["baro"] is not None:
            c = state["baro"].count()
            if c > state["last_baro_n"]:
                state["last_baro_n"] = c
                state["paired_baro"].append((t, z, state["baro"].latest().pressure))
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    return state


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    log, f = log_open("sensor_model_live_validation")
    all_results = {}

    log("=" * 78)
    log("FALCON V2 - SENSOR_MODEL_AND_ARDUPLANE_SITL_PREPARATION live validation")
    log("gazebo-testing, 2026-08-27")
    log("=" * 78)
    log(f"World: {WORLD}")
    log(f"Declared world magnetic_field (T): {DECLARED_MAG_FIELD_T}, "
        f"magnitude={vec3_norm(*DECLARED_MAG_FIELD_T):.6e} T")
    log("")

    # =========================================================================
    # PART A: BARO_ALTITUDE_RESPONSE_REVIEW_REQUIRED
    # =========================================================================
    log("=" * 78)
    log("PART A: BARO_ALTITUDE_RESPONSE_REVIEW_REQUIRED")
    log("=" * 78)

    log("--- A1: free-fall descent regression (spawn Z=150m, unheld, 3000 steps=3s) ---")
    st = run_free_motion_case(log, "A1_BARO_FREEFALL", altitude=150.0, throttle=0.0,
                               elev_deg=0.0, release_hold_steps=0, total_steps=3000,
                               need_gps=False, need_baro=True)
    pb = st["paired_baro"]
    log(f"any_nan={st['any_nan']} n_baro_paired_samples={len(pb)}")
    a1 = dict(any_nan=st["any_nan"], n_samples=len(pb))
    if pb:
        first_t, first_z, first_p = pb[0]
        last_t, last_z, last_p = pb[-1]
        log(f"  first sample: t={first_t:.3f}s z={first_z:.4f}m pressure={first_p:.4f}Pa "
            f"(ISA-predicted={isa_pressure_pa(first_z):.4f}Pa)")
        log(f"  last  sample: t={last_t:.3f}s z={last_z:.4f}m pressure={last_p:.4f}Pa "
            f"(ISA-predicted={isa_pressure_pa(last_z):.4f}Pa)")
        # least-squares slope of pressure vs z across the whole descent
        n = len(pb)
        zs = [s[1] for s in pb]
        ps = [s[2] for s in pb]
        zbar = sum(zs) / n
        pbar = sum(ps) / n
        num = sum((z - zbar) * (p - pbar) for z, p in zip(zs, ps))
        den = sum((z - zbar) ** 2 for z in zs)
        slope = num / den if den > 0 else None
        isa_slope = (isa_pressure_pa(last_z) - isa_pressure_pa(first_z)) / (last_z - first_z) if last_z != first_z else None
        log(f"  observed dP/dz (least-squares over full descent): {slope:.6f} Pa/m")
        log(f"  ISA-predicted dP/dz over the same z-range: {isa_slope:.6f} Pa/m")
        log(f"  ratio observed/ISA-predicted slope: {(slope / isa_slope) if isa_slope else None}")
        a1.update(first=dict(t=first_t, z=first_z, pressure=first_p, isa_pred=isa_pressure_pa(first_z)),
                  last=dict(t=last_t, z=last_z, pressure=last_p, isa_pred=isa_pressure_pa(last_z)),
                  observed_slope_pa_per_m=slope, isa_slope_pa_per_m=isa_slope,
                  slope_ratio=(slope / isa_slope) if isa_slope else None,
                  full_series=[(t, z, p) for t, z, p in pb])
    all_results["A1_baro_freefall_regression"] = a1
    log("")

    log("--- A2/A3: two independent spawn-altitude comparison (Z=10m vs Z=300m, ~0.3s settle) ---")
    a2 = run_held_case(log, "A2_BARO_Z10", WORLD, altitude=10.0, settle_steps=300, tail_steps=100,
                        need_baro=True)
    log(f"[A2 Z=10m ] any_nan={a2['any_nan']} baro_count={a2.get('baro_count')} "
        f"baro_avg={a2.get('baro_avg')} ISA_pred={isa_pressure_pa(10.0):.4f} final_z={a2.get('final_z')}")
    a3 = run_held_case(log, "A3_BARO_Z300", WORLD, altitude=300.0, settle_steps=300, tail_steps=100,
                        need_baro=True)
    log(f"[A3 Z=300m] any_nan={a3['any_nan']} baro_count={a3.get('baro_count')} "
        f"baro_avg={a3.get('baro_avg')} ISA_pred={isa_pressure_pa(300.0):.4f} final_z={a3.get('final_z')}")
    log(f"  delta observed (Z300-Z10) = {(a3.get('baro_avg') or 0) - (a2.get('baro_avg') or 0):.4f} Pa "
        f"vs ISA-predicted delta = {isa_pressure_pa(300.0) - isa_pressure_pa(10.0):.4f} Pa")
    all_results["A2_baro_z10"] = a2
    all_results["A3_baro_z300"] = a3
    log("")

    # =========================================================================
    # PART B: MAG_FIELD_SOURCE_REVIEW_REQUIRED
    # =========================================================================
    log("=" * 78)
    log("PART B: MAG_FIELD_SOURCE_REVIEW_REQUIRED")
    log("=" * 78)
    headings_deg = [0.0, 90.0, 180.0, 270.0]
    mag_heading_results = {}
    for hdg in headings_deg:
        label = f"B_MAG_HDG_{int(hdg)}"
        r = run_held_case(log, label, WORLD, altitude=100.0, rpy=(0.0, 0.0, math.radians(hdg)),
                           settle_steps=1500, tail_steps=300, need_mag=True)
        mavg = r.get("mag_avg")
        mag_mag = vec3_norm(*mavg) if mavg else None
        log(f"[{label}] any_nan={r['any_nan']} mag_count={r.get('mag_count')} "
            f"mag_avg={mavg} |mag|={mag_mag}")
        r["mag_magnitude"] = mag_mag
        mag_heading_results[label] = r
    all_results["B_mag_heading_sweep"] = mag_heading_results
    declared_mag = vec3_norm(*DECLARED_MAG_FIELD_T)
    log(f"declared world |magnetic_field| = {declared_mag:.6e} T")
    ratios = [ (r["mag_magnitude"] / declared_mag) for r in mag_heading_results.values() if r.get("mag_magnitude")]
    log(f"observed/declared magnitude ratios across headings: {ratios}")
    log("")

    log("--- B2: field-scale test (100x world magnetic_field, altmag world) ---")
    b2 = run_held_case(log, "B2_MAG_ALTMAG", WORLD_ALTMAG, altitude=100.0, rpy=(0.0, 0.0, 0.0),
                        settle_steps=1500, tail_steps=300, need_mag=True)
    b2_mag = vec3_norm(*b2["mag_avg"]) if b2.get("mag_avg") else None
    baseline_0deg = mag_heading_results["B_MAG_HDG_0"]
    log(f"[B2 altmag] any_nan={b2['any_nan']} mag_avg={b2.get('mag_avg')} |mag|={b2_mag}")
    log(f"[baseline 0deg, original world] mag_avg={baseline_0deg.get('mag_avg')} "
        f"|mag|={baseline_0deg.get('mag_magnitude')}")
    if b2_mag and baseline_0deg.get("mag_magnitude"):
        ratio = b2_mag / baseline_0deg["mag_magnitude"]
        log(f"ratio |mag|(altmag=100x world field) / |mag|(original world field) = {ratio:.4f} "
            f"(expect ~100 if sensor scales with declared world field, ~1 if it ignores it)")
        b2["scale_ratio_vs_original"] = ratio
    all_results["B2_mag_altmag_scale_test"] = b2
    log("")

    # =========================================================================
    # PART C: IMU
    # =========================================================================
    log("=" * 78)
    log("PART C: IMU STATIC + KNOWN-RATE")
    log("=" * 78)
    log("--- C1: static/at-rest under gravity (held zero body velocity, throttle=0) ---")
    c1 = run_held_case(log, "C1_IMU_STATIC", WORLD, altitude=100.0, settle_steps=2000, tail_steps=400,
                        need_imu=True)
    log(f"any_nan={c1['any_nan']} imu_count={c1.get('imu_count')} "
        f"orientation_last={c1.get('imu_last', {}).get('orientation')} "
        f"specific_force_tail_avg={c1.get('imu_tail_avg_specforce')}")
    all_results["C1_imu_static"] = c1
    log("")

    log("--- C2: known constant yaw rate (r_target=+0.3 rad/s, held via hold_step angular target) ---")
    c2 = run_held_case(log, "C2_IMU_KNOWN_RATE", WORLD, altitude=100.0, ang_target=(0.0, 0.0, 0.3),
                        settle_steps=2000, tail_steps=400, need_imu=True)
    log(f"any_nan={c2['any_nan']} imu_count={c2.get('imu_count')} "
        f"commanded_r=0.3 measured_angvel_z_tail_avg={c2.get('imu_tail_avg_angvel_z')} "
        f"final_av={c2.get('final_av')}")
    all_results["C2_imu_known_rate"] = c2
    log("")

    # =========================================================================
    # PART D: GPS
    # =========================================================================
    log("=" * 78)
    log("PART D: GPS known-motion")
    log("=" * 78)
    log("--- D1: pure vertical free-fall (spawn Z=150m, unheld, throttle=0, 2000 steps) ---")
    stD1 = run_free_motion_case(log, "D1_GPS_FREEFALL", altitude=150.0, throttle=0.0, elev_deg=0.0,
                                 release_hold_steps=0, total_steps=2000, need_gps=True)
    pg = stD1["paired_gps"]
    log(f"any_nan={stD1['any_nan']} n_gps_paired_samples={len(pg)}")
    d1 = dict(any_nan=stD1["any_nan"], n_samples=len(pg))
    if pg:
        first = pg[0]
        last = pg[-1]
        def fmt(sample):
            t, z, vx, vy, vz, m = sample
            return dict(t=t, true_z=z, true_v=(vx, vy, vz),
                        gps_alt=m.altitude, gps_v=(m.velocity_east, m.velocity_north, m.velocity_up),
                        lat=m.latitude_deg, lon=m.longitude_deg)
        log(f"  first: {fmt(first)}")
        log(f"  last : {fmt(last)}")
        d1["first"] = fmt(first)
        d1["last"] = fmt(last)
        d1["full_series"] = [fmt(s) for s in pg]
    all_results["D1_gps_freefall"] = d1
    log("")

    log("--- D2: powered forward trim, released after 500-step hold (2500 steps total) ---")
    stD2 = run_free_motion_case(log, "D2_GPS_FORWARD", altitude=100.0, throttle=THROTTLE, elev_deg=ELEV_THETA_DEG,
                                 release_hold_steps=500, total_steps=2500, need_gps=True)
    pg2 = stD2["paired_gps"]
    log(f"any_nan={stD2['any_nan']} n_gps_paired_samples={len(pg2)}")
    d2 = dict(any_nan=stD2["any_nan"], n_samples=len(pg2))
    if pg2:
        first = pg2[0]
        last = pg2[-1]
        def fmt2(sample):
            t, z, vx, vy, vz, m = sample
            return dict(t=t, true_z=z, true_v=(vx, vy, vz),
                        gps_alt=m.altitude, gps_v=(m.velocity_east, m.velocity_north, m.velocity_up),
                        lat=m.latitude_deg, lon=m.longitude_deg)
        log(f"  first (near release): {fmt2(first)}")
        log(f"  last                : {fmt2(last)}")
        d2["first"] = fmt2(first)
        d2["last"] = fmt2(last)
        d2["full_series"] = [fmt2(s) for s in pg2]
    all_results["D2_gps_forward_trim"] = d2
    log("")

    # =========================================================================
    # PART E: PITOT wind response (the priority check)
    # =========================================================================
    log("=" * 78)
    log("PART E: PITOT wind response at nominal trim (V~18.166m/s, throttle=0.5010, elevator=+4.5deg)")
    log("=" * 78)
    wind_cases = [
        ("E1_ZERO_WIND", (0.0, 0.0, 0.0)),
        ("E2_HEADWIND_5", (-5.0, 0.0, 0.0)),   # air mass moving -X while aircraft flies +X => headwind
        ("E3_TAILWIND_5", (5.0, 0.0, 0.0)),    # air mass moving +X (same as flight direction) => tailwind
        ("E4_CROSSWIND_5", (0.0, 5.0, 0.0)),   # air mass moving +Y (world) => crosswind
    ]
    pitot_results = {}
    for label, wind in wind_cases:
        r = run_held_case(log, label, WORLD, altitude=100.0, throttle=THROTTLE, elev_deg=ELEV_THETA_DEG,
                           wind=wind, lin_target=(U_HOLD, 0.0, W_HOLD), ang_target=(0.0, 0.0, 0.0),
                           settle_steps=2500, tail_steps=500, need_pitot=True, need_aero_diag=True)
        log(f"[{label}] wind_cmd={wind} any_nan={r['any_nan']} "
            f"pitot_v_avg={r.get('pitot_v_avg')} pitot_p_avg={r.get('pitot_p_avg')} "
            f"aero_V_avg={r.get('aero_V_avg')} final_lv={r.get('final_lv')}")
        pitot_results[label] = r
    all_results["E_pitot_wind_response"] = pitot_results
    base = pitot_results["E1_ZERO_WIND"]
    log("")
    log("Pitot sign-check summary (vs E1 zero-wind baseline):")
    for label, _ in wind_cases[1:]:
        r = pitot_results[label]
        dv = (r.get("pitot_v_avg") or 0) - (base.get("pitot_v_avg") or 0)
        log(f"  [{label}] pitot_v_avg={r.get('pitot_v_avg')} delta_vs_zero_wind={dv:+.4f} "
            f"aero_diag_V={r.get('aero_V_avg')} pitot_vs_aero_diff={(r.get('pitot_v_avg') or 0) - (r.get('aero_V_avg') or 0):+.6f}")
    log("")

    # Save JSON
    def jsonable(o):
        if isinstance(o, dict):
            return {k: jsonable(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [jsonable(v) for v in o]
        if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
            return str(o)
        return o

    with open(f"{RESULTS_DIR}/sensor_model_live_validation_result.json", "w") as jf:
        json.dump(jsonable(all_results), jf, indent=2)

    log("=" * 78)
    log("DONE. Results: tests/gazebo/results/sensor_model_live_validation_{log.txt,result.json}")
    log("=" * 78)
    f.close()


if __name__ == "__main__":
    main()

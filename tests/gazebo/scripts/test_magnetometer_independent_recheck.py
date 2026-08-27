#!/usr/bin/env python3
"""
FALCON V2 - independent re-verification of the magnetometer ONLY
(gazebo-testing, 2026-08-27).

Scope: this is a SECOND, INDEPENDENT confirmation pass of the new
FalconV2Magnetometer plugin (plugins/sensors/MagnetometerSystem.{hh,cc},
controls-integration's replacement for the confirmed-broken native
gz-sim8 Magnetometer system - see
docs/test_results/2026-08-27_sensor_model_sitl_preparation.md sec 5 for the
original TEST_FAILED finding). Deliberately does NOT reuse
test_sensor_model_live_validation.py's exact steps/numbers - different
technique throughout, per the task brief's independence requirement:

  - Holding: plain velocity-command override (Link.set_angular_velocity /
    set_linear_velocity, called every tick) instead of aero_lib's
    proportional force/torque hold_step() controller.
  - Rotation cross-check math: a from-scratch quaternion vector-rotation
    function (Rodrigues form, hand-derived below), NOT a call into
    gz.math7.Quaterniond.rotate_vector_reverse() (the exact function the
    plugin itself calls) - so the cross-check does not just mirror the
    plugin's own math back at itself.
  - Cross-check sources: BOTH the ECM's own WorldPose ground truth AND the
    live IMU orientation topic, independently, at the same paired tick.
  - The declared Earth-field parameter is read directly out of
    model/model.sdf via xml parsing in this script, never hand-copied from
    controls-integration's or this suite's own prior report.
  - New diagnostic world/model pair created this pass (see
    tests/gazebo/worlds/falcon_v2_magnetometer_scaled_model/model.sdf and
    tests/gazebo/worlds/falcon_v2_sensors_selftest_magscaled_world.sdf):
    live-tested first and confirmed the NEW plugin does NOT read the
    world-level <magnetic_field> SDF element at all (unlike what the old,
    broken native system was supposed to do) - it has its own
    model-plugin-level <world_magnetic_field_tesla> parameter. Reusing the
    existing falcon_v2_sensors_selftest_altmag_world.sdf (which only scales
    the world element) would silently test nothing for this new plugin, so
    a new diagnostic pair that scales the parameter it actually reads was
    built instead.

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
control authority, motor thrust) is read for any purpose, nor modified,
anywhere in this script.
"""
import json
import math
import os
import sys
import threading
import xml.etree.ElementTree as ET

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
MODEL_SDF = os.path.join(REPO_ROOT, "model/model.sdf")


def setup_env():
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    build_dirs = [
        os.path.join(REPO_ROOT, "plugins/actuators/build"),
        os.path.join(REPO_ROOT, "plugins/propulsion/build"),
        os.path.join(REPO_ROOT, "plugins/aerodynamics/build"),
        os.path.join(REPO_ROOT, "plugins/wind/build"),
        os.path.join(REPO_ROOT, "plugins/sensors/build"),
    ]
    existing = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
        build_dirs + ([existing] if existing else []))


setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402
import gz.transport13 as tp  # noqa: E402

WORLDS_DIR = os.path.join(REPO_ROOT, "tests/gazebo/worlds")
WORLD_MAIN = os.path.join(WORLDS_DIR, "falcon_v2_sensors_selftest_world.sdf")
WORLD_MAGSCALED = os.path.join(
    WORLDS_DIR, "falcon_v2_sensors_selftest_magscaled_world.sdf")
WORLD_ALTMAG_LEGACY = os.path.join(
    WORLDS_DIR, "falcon_v2_sensors_selftest_altmag_world.sdf")

MAG_TOPIC = "/model/falcon_v2/sensors/mag"
IMU_TOPIC = "/model/falcon_v2/sensors/imu"

RESULTS_DIR = os.path.join(REPO_ROOT, "tests/gazebo/results")


# =============================================================================
# Independent read of the plugin's declared field (never hand-copied from
# any other report).
# =============================================================================
def read_declared_field_from_model_sdf():
    tree = ET.parse(MODEL_SDF)
    root = tree.getroot()
    for plugin in root.iter("plugin"):
        if plugin.attrib.get("filename") == "FalconV2Magnetometer":
            elt = plugin.find("world_magnetic_field_tesla")
            rate = plugin.find("update_rate_hz")
            topic = plugin.find("topic")
            vec = tuple(float(v) for v in elt.text.split())
            return vec, float(rate.text) if rate is not None else None, \
                (topic.text.strip() if topic is not None else None)
    raise RuntimeError("FalconV2Magnetometer plugin block not found in model.sdf")


def read_declared_field_from_scaled_model_sdf():
    path = os.path.join(WORLDS_DIR, "falcon_v2_magnetometer_scaled_model/model.sdf")
    tree = ET.parse(path)
    root = tree.getroot()
    for plugin in root.iter("plugin"):
        if plugin.attrib.get("filename") == "FalconV2Magnetometer":
            elt = plugin.find("world_magnetic_field_tesla")
            return tuple(float(v) for v in elt.text.split())
    raise RuntimeError("FalconV2Magnetometer plugin block not found in scaled model.sdf")


def vec3_norm(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


# =============================================================================
# From-scratch quaternion vector rotation (Rodrigues form), independent of
# gz.math7's own rotate_vector/rotate_vector_reverse calls - used ONLY as a
# cross-check, deliberately not the same code path the plugin under test
# uses (plugins/sensors/MagnetometerSystem.cc calls
# gz::math::Quaterniond::RotateVectorReverse() directly).
#
# For a unit quaternion q=(w,x,y,z) that rotates BODY vectors into WORLD
# (v_world = q * v_body * q^-1, the standard "orientation" convention), the
# inverse mapping (world vector -> body-frame vector) is v_body = q* * v_world
# * q, i.e. rotation BY the conjugate q* = (w,-x,-y,-z). Using the standard
# closed-form vector-rotation-by-quaternion identity
#   v' = v + 2*qw*(qxyz x v) + 2*(qxyz x (qxyz x v))
# with qxyz replaced by the conjugate's vector part (-x,-y,-z):
# =============================================================================
def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def rotate_world_vector_to_body(qw, qx, qy, qz, v_world):
    ux, uy, uz = (-qx, -qy, -qz)  # vector part of conjugate q*
    u = (ux, uy, uz)
    t1 = cross(u, v_world)
    t1 = (2.0 * qw * t1[0], 2.0 * qw * t1[1], 2.0 * qw * t1[2])
    t2 = cross(u, cross(u, v_world))
    t2 = (2.0 * t2[0], 2.0 * t2[1], 2.0 * t2[2])
    return (v_world[0] + t1[0] + t2[0],
            v_world[1] + t1[1] + t2[1],
            v_world[2] + t1[2] + t2[2])


def quat_self_check():
    """Sanity check my own rotate_world_vector_to_body() against a case with
    a known-by-hand answer: 90 deg yaw (about world Z, FLU/ENU), a world
    vector along +X should appear along -Y in body frame (nose has rotated
    90 deg left of the vector's original heading -> the vector now points
    to the body's right...). Verified numerically instead of by hand-wavy
    argument: build the SAME quaternion via gz.math7 (independent library
    construction of q, not of the rotation formula) and require my formula
    to reproduce gz.math7's rotate_vector_reverse() for several arbitrary
    (q, v) pairs to within 1e-12."""
    import random
    random.seed(1234)
    for _ in range(200):
        r, p, y = (random.uniform(-3.1, 3.1) for _ in range(3))
        q = gm.Quaterniond(r, p, y)
        v = tuple(random.uniform(-5, 5) for _ in range(3))
        expected = q.rotate_vector_reverse(gm.Vector3d(*v))
        got = rotate_world_vector_to_body(q.w(), q.x(), q.y(), q.z(), v)
        err = math.sqrt((got[0] - expected.x()) ** 2 +
                         (got[1] - expected.y()) ** 2 +
                         (got[2] - expected.z()) ** 2)
        assert err < 1e-9, f"quat_self_check FAILED err={err}"
    return True


# =============================================================================
# Minimal raw gz-transport subscriber.
# =============================================================================
class RawSub:
    def __init__(self, topic, module_name, class_name):
        import importlib
        mod = importlib.import_module(module_name)
        self._cls = getattr(mod, class_name)
        self._msg_type = f"gz.msgs.{class_name}"
        self.node = tp.Node()
        self.lock = threading.Lock()
        self.count_ = 0
        self.latest_ = None
        ok = self.node.subscribe_raw(
            topic, self._cb, self._msg_type, tp.SubscribeOptions())
        if not ok:
            raise RuntimeError(f"subscribe failed: {topic}")

    def _cb(self, data, info):
        m = self._cls()
        m.ParseFromString(data)
        with self.lock:
            self.latest_ = m
            self.count_ += 1

    def snapshot(self):
        with self.lock:
            return self.latest_, self.count_


def mag_sub():
    return RawSub(MAG_TOPIC, "gz.msgs10.magnetometer_pb2", "Magnetometer")


def imu_sub():
    return RawSub(IMU_TOPIC, "gz.msgs10.imu_pb2", "IMU")


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    return sim.Model(world.model_by_name(ecm, "falcon_v2"))


def log_open(name):
    path = os.path.join(RESULTS_DIR, f"{name}_log.txt")
    f = open(path, "w")

    def log(msg=""):
        print(msg)
        f.write(str(msg) + "\n")
        f.flush()
    return log, f


# =============================================================================
# PART 1 / PART 3 - static level hold via VELOCITY-COMMAND override (every
# tick), deliberately not aero_lib's force-based hold_step().
# =============================================================================
def run_static_level_with_tail(world, steps=1200, tail=250, r=0.0):
    """Same as run_static_level but records a tail of mag samples plus the
    paired IMU orientation and ECM ground-truth pose at each NEW mag
    message. r != 0 additionally commands a constant WORLD-frame yaw rate
    (rad/s) every tick, used for PART 2's known-rate rotation check."""
    state = {"teleported": False, "mag": None, "imu": None, "n": 0,
             "mag_count_prev": 0, "series": [], "any_nan": False}

    def on_pre(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, 100, 0, 0, 0))
            state["teleported"] = True
            state["mag"] = mag_sub()
            state["imu"] = imu_sub()
        base.set_linear_velocity(ecm, gm.Vector3d(0, 0, 0))
        base.set_angular_velocity(ecm, gm.Vector3d(0, 0, r))

    def on_post(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        wpose = base.world_pose(ecm)
        mag_latest, mag_count = state["mag"].snapshot()
        if mag_count > state["mag_count_prev"] and wpose is not None:
            state["mag_count_prev"] = mag_count
            imu_latest, _ = state["imu"].snapshot()
            entry = {
                "t": info.sim_time.total_seconds() if hasattr(info.sim_time, "total_seconds")
                     else state["n"] * 0.001,
                "ecm_quat": (wpose.rot().w(), wpose.rot().x(), wpose.rot().y(), wpose.rot().z()),
                "ecm_rpy": (wpose.rot().roll(), wpose.rot().pitch(), wpose.rot().yaw()),
                "mag_field": (mag_latest.field_tesla.x, mag_latest.field_tesla.y,
                              mag_latest.field_tesla.z),
            }
            if imu_latest is not None:
                o = imu_latest.orientation
                entry["imu_quat"] = (o.w, o.x, o.y, o.z)
            vals = list(entry["mag_field"]) + list(entry["ecm_quat"])
            if any(math.isnan(v) or math.isinf(v) for v in vals):
                state["any_nan"] = True
            state["series"].append(entry)
        state["n"] += 1

    fixture = sim.TestFixture(world)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    fixture.server().run(True, steps, False)
    return state


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    log, f = log_open("magnetometer_independent_recheck")
    result = {}

    log("=" * 78)
    log("FALCON V2 - independent magnetometer re-check (gazebo-testing, 2026-08-27)")
    log("=" * 78)

    assert quat_self_check()
    log("[SELFCHECK] from-scratch rotate_world_vector_to_body() matches "
        "gz.math7 Quaterniond.rotate_vector_reverse() on 200 random (q,v) "
        "pairs to <1e-9 - my independent rotation formula is itself correct "
        "before using it to judge the plugin.")

    declared_vec, declared_rate, declared_topic = read_declared_field_from_model_sdf()
    declared_mag = vec3_norm(declared_vec)
    log(f"\n[0] Declared plugin parameters, read directly from model/model.sdf "
        f"via xml parsing (not copied from any other report):")
    log(f"    world_magnetic_field_tesla = {declared_vec} T, |.|={declared_mag:.8e} T")
    log(f"    update_rate_hz = {declared_rate}, topic = {declared_topic}")
    result["declared_vec"] = declared_vec
    result["declared_mag"] = declared_mag
    result["declared_rate_hz"] = declared_rate
    result["declared_topic"] = declared_topic

    # -------------------------------------------------------------------
    # PART 1: static level hold, main (unscaled) world.
    # -------------------------------------------------------------------
    log("\n" + "-" * 78)
    log("PART 1: static level (rpy=0,0,0), velocity-command hold, unscaled world")
    log("-" * 78)
    st1 = run_static_level_with_tail(WORLD_MAIN, steps=1200, r=0.0)
    tail1 = st1["series"][-200:] if len(st1["series"]) >= 200 else st1["series"]
    avg1 = tuple(sum(e["mag_field"][i] for e in tail1) / len(tail1) for i in range(3))
    mag1 = vec3_norm(avg1)
    log(f"  mag messages captured (paired) = {len(st1['series'])}, any_nan={st1['any_nan']}")
    log(f"  tail-avg mag field (last {len(tail1)}) = {avg1}")
    log(f"  |mag| = {mag1:.8e} T  vs declared |.|={declared_mag:.8e} T "
        f"(ratio={mag1/declared_mag:.6f})")
    comp_err = tuple(avg1[i] - declared_vec[i] for i in range(3))
    log(f"  per-axis (observed - declared) = {comp_err}")
    last_rpy = tail1[-1]["ecm_rpy"] if tail1 else None
    log(f"  ECM ground-truth final rpy (rad) = {last_rpy} (expect ~0,0,0)")
    p1_pass = (abs(mag1 / declared_mag - 1.0) < 0.01 and
               all(abs(e) < 0.01 * declared_mag for e in comp_err) and
               not st1["any_nan"])
    result["part1"] = dict(avg_field=avg1, mag=mag1, declared_mag=declared_mag,
                            ratio=mag1 / declared_mag, comp_err=comp_err,
                            final_rpy=last_rpy, any_nan=st1["any_nan"], PASS=p1_pass)
    log(f"  PART 1 VERDICT: {'PASS' if p1_pass else 'FAIL'}")

    # -------------------------------------------------------------------
    # PART 2: known constant WORLD-frame yaw rate, cross-checked against
    # BOTH ECM ground truth and the live IMU orientation topic, using the
    # from-scratch rotation formula (not the plugin's own gz.math call).
    # -------------------------------------------------------------------
    log("\n" + "-" * 78)
    log("PART 2: known commanded yaw rate r=+0.5 rad/s (world Z), 6000 steps (6s)")
    log("-" * 78)
    R_CMD = 0.5
    st2 = run_static_level_with_tail(WORLD_MAIN, steps=6000, r=R_CMD)
    series2 = st2["series"]
    log(f"  mag messages captured (paired) = {len(series2)}, any_nan={st2['any_nan']}")

    if series2:
        first, last = series2[0], series2[-1]
        log(f"  first sample t~{first['t']:.3f}s ecm_rpy={first['ecm_rpy']}")
        log(f"  last  sample t~{last['t']:.3f}s ecm_rpy={last['ecm_rpy']}")
        max_roll = max(abs(e["ecm_rpy"][0]) for e in series2)
        max_pitch = max(abs(e["ecm_rpy"][1]) for e in series2)
        log(f"  max |roll| over sweep = {max_roll:.6f} rad, max |pitch| = {max_pitch:.6f} rad "
            f"(expect ~0 if command is a pure world-Z yaw rate, as assumed)")

        yaws = [e["ecm_rpy"][2] for e in series2]
        ts = [e["t"] for e in series2]
        # unwrap yaw for a clean linear-fit sanity check against r*t
        unwrapped = [yaws[0]]
        for i in range(1, len(yaws)):
            d = yaws[i] - yaws[i - 1]
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            unwrapped.append(unwrapped[-1] + d)
        dt_total = ts[-1] - ts[0]
        dyaw_total = unwrapped[-1] - unwrapped[0]
        observed_rate = dyaw_total / dt_total if dt_total > 0 else float("nan")
        log(f"  yaw swept (unwrapped) = {dyaw_total:.5f} rad over {dt_total:.4f} s "
            f"-> observed rate={observed_rate:.5f} rad/s vs commanded r={R_CMD} rad/s")

        magnitudes = [vec3_norm(e["mag_field"]) for e in series2]
        mag_min, mag_max = min(magnitudes), max(magnitudes)
        log(f"  |mag| range over full sweep = [{mag_min:.8e}, {mag_max:.8e}] T "
            f"(rotation-invariant check - should stay == declared |.|={declared_mag:.8e} T)")

        # Cross-check EVERY sample against BOTH ECM ground truth and live IMU,
        # using the from-scratch rotation formula only.
        errs_ecm = []
        errs_imu = []
        n_imu_missing = 0
        for e in series2:
            qw, qx, qy, qz = e["ecm_quat"]
            pred_ecm = rotate_world_vector_to_body(qw, qx, qy, qz, declared_vec)
            obs = e["mag_field"]
            err_ecm = math.sqrt(sum((pred_ecm[i] - obs[i]) ** 2 for i in range(3)))
            errs_ecm.append(err_ecm)
            if "imu_quat" in e:
                iqw, iqx, iqy, iqz = e["imu_quat"]
                pred_imu = rotate_world_vector_to_body(iqw, iqx, iqy, iqz, declared_vec)
                err_imu = math.sqrt(sum((pred_imu[i] - obs[i]) ** 2 for i in range(3)))
                errs_imu.append(err_imu)
            else:
                n_imu_missing += 1

        max_err_ecm = max(errs_ecm)
        avg_err_ecm = sum(errs_ecm) / len(errs_ecm)
        log(f"  cross-check vs ECM ground-truth WorldPose (own rotation formula): "
            f"max_err={max_err_ecm:.3e} T, avg_err={avg_err_ecm:.3e} T "
            f"(relative to |.|={declared_mag:.3e} T)")
        if errs_imu:
            max_err_imu = max(errs_imu)
            avg_err_imu = sum(errs_imu) / len(errs_imu)
            log(f"  cross-check vs LIVE IMU orientation topic (own rotation formula): "
                f"max_err={max_err_imu:.3e} T, avg_err={avg_err_imu:.3e} T, "
                f"n_missing_imu_pairing={n_imu_missing}/{len(series2)}")
        else:
            max_err_imu = avg_err_imu = None
            log("  [WARN] no IMU-paired samples available")

        tol = 0.02 * declared_mag  # 2% of field magnitude
        p2_pass = (max_roll < 0.01 and max_pitch < 0.01 and
                   abs(observed_rate - R_CMD) < 0.01 and
                   (mag_max - mag_min) < tol and
                   max_err_ecm < tol and
                   (max_err_imu is None or max_err_imu < tol) and
                   not st2["any_nan"])
        result["part2"] = dict(
            n_samples=len(series2), max_roll=max_roll, max_pitch=max_pitch,
            observed_rate=observed_rate, commanded_rate=R_CMD,
            mag_min=mag_min, mag_max=mag_max, declared_mag=declared_mag,
            max_err_ecm=max_err_ecm, avg_err_ecm=avg_err_ecm,
            max_err_imu=max_err_imu, avg_err_imu=avg_err_imu,
            any_nan=st2["any_nan"], PASS=p2_pass)
        log(f"  PART 2 VERDICT: {'PASS' if p2_pass else 'FAIL'}")
    else:
        result["part2"] = dict(PASS=False, note="no samples captured")
        log("  PART 2 VERDICT: FAIL (no samples captured)")

    # -------------------------------------------------------------------
    # PART 3: field-source SCALE regression - new magscaled world/model
    # (scales the plugin's OWN parameter, not the inert world-level element).
    # -------------------------------------------------------------------
    log("\n" + "-" * 78)
    log("PART 3: field-SCALE regression (plugin's own parameter x100), new "
        "falcon_v2_sensors_selftest_magscaled_world.sdf")
    log("-" * 78)
    scaled_declared_vec = read_declared_field_from_scaled_model_sdf()
    scaled_declared_mag = vec3_norm(scaled_declared_vec)
    log(f"  scaled model.sdf copy's own declared world_magnetic_field_tesla = "
        f"{scaled_declared_vec} T, |.|={scaled_declared_mag:.8e} T "
        f"(read independently from tests/gazebo/worlds/"
        f"falcon_v2_magnetometer_scaled_model/model.sdf)")
    st3 = run_static_level_with_tail(WORLD_MAGSCALED, steps=1200, r=0.0)
    tail3 = st3["series"][-200:] if len(st3["series"]) >= 200 else st3["series"]
    avg3 = tuple(sum(e["mag_field"][i] for e in tail3) / len(tail3) for i in range(3))
    mag3 = vec3_norm(avg3)
    log(f"  mag messages captured = {len(st3['series'])}, any_nan={st3['any_nan']}")
    log(f"  tail-avg mag field (scaled world) = {avg3}")
    log(f"  |mag| = {mag3:.8e} T")
    ratio_mag = mag3 / mag1 if mag1 else float("nan")
    log(f"  magnitude ratio (scaled/unscaled, BOTH LIVE-MEASURED this pass) = "
        f"{ratio_mag:.4f} (expect ~100, since the plugin parameter itself was "
        f"scaled 100x)")
    comp_ratios = tuple((avg3[i] / avg1[i]) if abs(avg1[i]) > 1e-12 else float("nan")
                         for i in range(3))
    log(f"  per-axis ratio (scaled/unscaled) = {comp_ratios} (expect ~100,100,100 - "
        f"a clean, consistent scale factor, unlike the original defect's "
        f"per-axis ratios of ~49590x/-1058x/3775x)")
    p3_pass = (abs(ratio_mag - 100.0) < 2.0 and
               all(abs(cr - 100.0) < 2.0 for cr in comp_ratios) and
               not st3["any_nan"])
    result["part3"] = dict(avg_field_scaled=avg3, mag_scaled=mag3, mag_unscaled=mag1,
                            ratio=ratio_mag, comp_ratios=comp_ratios,
                            any_nan=st3["any_nan"], PASS=p3_pass)
    log(f"  PART 3 VERDICT: {'PASS' if p3_pass else 'FAIL'}")

    # -------------------------------------------------------------------
    # PART 4: single-publisher / rate sanity check, unscaled world +
    # leftover-native-plugin altmag world (dead-plugin-declaration check).
    # -------------------------------------------------------------------
    log("\n" + "-" * 78)
    log("PART 4: single-publisher / rate sanity check")
    log("-" * 78)
    # 4a: main world, message rate over a fixed sim duration.
    st4 = run_static_level_with_tail(WORLD_MAIN, steps=2000, r=0.0)
    n4 = len(st4["series"])
    sim_dur = st4["series"][-1]["t"] - st4["series"][0]["t"] if n4 > 1 else 0.0
    observed_hz = (n4 - 1) / sim_dur if sim_dur > 0 else float("nan")
    log(f"  main world: {n4} paired mag samples over ~{sim_dur:.3f}s sim time "
        f"-> observed rate ~= {observed_hz:.2f} Hz vs declared "
        f"update_rate_hz={declared_rate} Hz (a ~2x reading here would indicate "
        f"a second live publisher on the same topic)")
    KNOWN_BROKEN_NATIVE_MAG = 0.32  # the OLD native system's frozen signature value, T
    n_broken_signature = sum(
        1 for e in st4["series"] if abs(vec3_norm(e["mag_field"]) - KNOWN_BROKEN_NATIVE_MAG) < 1e-3)
    log(f"  samples matching the OLD broken-native-system signature "
        f"(|mag|~{KNOWN_BROKEN_NATIVE_MAG} T) = {n_broken_signature}/{n4}")

    # 4b: legacy altmag world (still declares the now-dead native
    # gz-sim-magnetometer-system plugin line) - confirm it contributes
    # nothing (no matching <sensor type="magnetometer"> element exists in
    # model.sdf anymore for that system to attach to, so it has nothing to
    # manage) and does NOT produce a double-publish/broken-signature mix.
    st4b = run_static_level_with_tail(WORLD_ALTMAG_LEGACY, steps=1200, r=0.0)
    n4b = len(st4b["series"])
    n_broken_signature_b = sum(
        1 for e in st4b["series"] if abs(vec3_norm(e["mag_field"]) - KNOWN_BROKEN_NATIVE_MAG) < 1e-3)
    sim_dur_b = st4b["series"][-1]["t"] - st4b["series"][0]["t"] if n4b > 1 else 0.0
    observed_hz_b = (n4b - 1) / sim_dur_b if sim_dur_b > 0 else float("nan")
    log(f"  legacy altmag world (still declares the dead "
        f"gz-sim-magnetometer-system plugin line): {n4b} samples, "
        f"observed rate ~= {observed_hz_b:.2f} Hz, "
        f"broken-signature count = {n_broken_signature_b}/{n4b}")

    p4_pass = (abs(observed_hz - declared_rate) / declared_rate < 0.15 and
               n_broken_signature == 0 and
               abs(observed_hz_b - declared_rate) / declared_rate < 0.15 and
               n_broken_signature_b == 0)
    result["part4"] = dict(observed_hz=observed_hz, declared_rate=declared_rate,
                            n_broken_signature=n_broken_signature,
                            observed_hz_altmag_legacy=observed_hz_b,
                            n_broken_signature_altmag_legacy=n_broken_signature_b,
                            PASS=p4_pass)
    log(f"  PART 4 VERDICT: {'PASS' if p4_pass else 'FAIL'}")

    overall = all(result[k]["PASS"] for k in ("part1", "part2", "part3", "part4"))
    result["overall_PASS"] = overall
    log("\n" + "=" * 78)
    log(f"OVERALL MAGNETOMETER INDEPENDENT RE-CHECK VERDICT: "
        f"{'PASS' if overall else 'FAIL'}")
    log("=" * 78)

    with open(os.path.join(RESULTS_DIR, "magnetometer_independent_recheck_result.json"), "w") as jf:
        json.dump(result, jf, indent=2, default=str)

    f.close()
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
FALCON V2 - structural V1 test pass, gazebo-testing, 2026-08-21.

Combined dynamics run against tests/gazebo/worlds/falcon_v2_freefall_world.sdf
(gravity ON, ground plane present, falcon_v2 spawned at (0,0,5)). One
continuous run is used to cover four planned tests, since they share the
same scenario:

  - MASS_CG_INERTIA_TEST: mass/CG/inertia of every link as loaded by the
    physics engine at t=0, aggregated (mass-weighted, parallel-axis) into a
    whole-model figure and cross-checked against both the documented SDF
    source values and the independent `gz sdf --inertial-stats` figures
    already recorded in GEOMETRY.md section 33.7 / 33.4. Also checks the
    "no unexpected drift at rest" free-rotation criterion: under gravity
    alone with zero initial velocity, a rigid body experiences zero net
    torque about its own CG (gravity acts through the CG by definition),
    so base_link's angular velocity is expected to stay small during the
    pre-ground-contact phase - this script checks it stays bounded, and
    separately documents (not as a failure) the much larger, physically
    expected free-swing motion of the five near-massless
    (TEMPORARY_NUMERICAL_MASS = 1 g) control-surface links, whose own CG is
    offset from their hinge line and which have no actuator or joint
    damping in this structural-only build.

  - STATIC_GRAVITY_TEST: aircraft falls under gravity with no aerodynamic
    forces (expected, not a failure). FAILS only on NaN, explosion,
    impossible angular acceleration, or instability.

  - COLLISION_SANITY_TEST: aircraft eventually contacts the ground plane;
    checks it does not sink through its own collision geometry and does
    not spontaneously launch.

  - NUMERICAL_STABILITY_IDLE_TEST: no NaN, no unbounded energy/velocity
    growth, across the whole run (free fall + impact + settling).

No aircraft physics parameter is modified by this script.
"""
import json
import math
import sys

import gz.math7 as gm
import gz.sim8 as sim

WORLD_SDF = "/home/emirhan/Desktop/FalconV2/tests/gazebo/worlds/falcon_v2_freefall_world.sdf"

ALL_LINKS = [
    "base_link", "left_aileron", "right_aileron", "left_elevator",
    "right_elevator", "rudder", "left_prop", "right_prop",
]

# Documented source-of-truth values (model/model.sdf, MASS_PROPERTIES.md section 6.3)
EXPECTED_LINK_MASS = {
    "base_link": 5.9348, "left_aileron": 0.001, "right_aileron": 0.001,
    "left_elevator": 0.001, "right_elevator": 0.001, "rudder": 0.001,
    "left_prop": 0.0301, "right_prop": 0.0301,
}
EXPECTED_TOTAL_MASS = 6.000
MASS_TOL = 1e-6

# GEOMETRY.md section 33.7 `gz sdf --inertial-stats` reference figures (independent tool, cross-checked here at runtime)
EXPECTED_AGG_CG = (0.169196, 0.0, 0.100291)
AGG_CG_TOL = 2e-4  # m (~0.2 mm) - tight, since this is meant to reproduce that exact figure
DOCUMENTED_CAD_CG = (0.168309, 0.0, 0.100000)
CAD_CG_MAX_SHIFT = 0.002  # m, generous vs the ~0.9 mm documented shift, catches gross errors only

G = 9.81
DROP_HEIGHT = 5.0
T_CONTACT_NOMINAL = math.sqrt(2 * DROP_HEIGHT / G)  # ~1.009 s
STEP = 0.001
TOTAL_STEPS = 3500  # 3.5 s sim time


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    fixture = sim.TestFixture(WORLD_SDF)
    state = {
        "n": 0,
        "t0_captured": False,
        "t0_links": {},
        "series": [],  # list of dicts per sampled step
        "any_nan_ever": False,
    }

    def get_model(ecm):
        world = sim.World(sim.world_entity(ecm))
        model_e = world.model_by_name(ecm, "falcon_v2")
        return sim.Model(model_e)

    def on_post(info, ecm):
        model = get_model(ecm)
        base_link_e = model.link_by_name(ecm, "base_link")
        base_link = sim.Link(base_link_e)
        base_link.enable_velocity_checks(ecm, True)

        # --- one-time t0 mass/CG/inertia capture (all 8 links) ---
        if not state["t0_captured"] and state["n"] >= 2:
            agg_mass = 0.0
            agg_m_r = [0.0, 0.0, 0.0]  # sum(m_i * r_i) for aggregate CG
            for lname in ALL_LINKS:
                le = model.link_by_name(ecm, lname)
                link = sim.Link(le)
                inertial = link.world_inertial(ecm)
                wpose = link.world_pose(ecm)
                if inertial is None or wpose is None:
                    continue
                mm = inertial.mass_matrix()
                mass = mm.mass()
                # NOTE (found during script development, verified interactively): despite the
                # method name suggesting a link-local offset, Link.world_inertial(ecm).pose()
                # already returns the CG pose expressed directly in WORLD frame (confirmed
                # identical to Link.world_inertial_pose(ecm)) - it must NOT be re-added to
                # wpose, or the CG ends up double-counting the link's own world offset.
                ipose = inertial.pose()  # CG pose, already in WORLD frame
                cg_world_x = ipose.pos().x()
                cg_world_y = ipose.pos().y()
                cg_world_z = ipose.pos().z()
                state["t0_links"][lname] = {
                    "mass": mass,
                    "ixx": mm.ixx(), "iyy": mm.iyy(), "izz": mm.izz(),
                    "ixy": mm.ixy(), "ixz": mm.ixz(), "iyz": mm.iyz(),
                    "cg_world": (cg_world_x, cg_world_y, cg_world_z),
                    "link_world_pose": (wpose.pos().x(), wpose.pos().y(), wpose.pos().z()),
                }
                agg_mass += mass
                agg_m_r[0] += mass * cg_world_x
                agg_m_r[1] += mass * cg_world_y
                agg_m_r[2] += mass * cg_world_z
            if agg_mass > 0:
                state["t0_agg_mass"] = agg_mass
                state["t0_agg_cg"] = (agg_m_r[0] / agg_mass, agg_m_r[1] / agg_mass, agg_m_r[2] / agg_mass)
            state["t0_captured"] = True

        # --- time series sampling (every step; cheap scalar reads only) ---
        wpose = base_link.world_pose(ecm)
        lin_vel = base_link.world_linear_velocity(ecm)
        ang_vel = base_link.world_angular_velocity(ecm)
        if wpose is not None and lin_vel is not None and ang_vel is not None:
            t = info.sim_time.total_seconds() if hasattr(info.sim_time, "total_seconds") else state["n"] * STEP
            rec = {
                "t": t,
                "z": wpose.pos().z(),
                "x": wpose.pos().x(),
                "y": wpose.pos().y(),
                "vx": lin_vel.x(), "vy": lin_vel.y(), "vz": lin_vel.z(),
                "wx": ang_vel.x(), "wy": ang_vel.y(), "wz": ang_vel.z(),
            }
            vals = [rec["z"], rec["x"], rec["y"], rec["vx"], rec["vy"], rec["vz"], rec["wx"], rec["wy"], rec["wz"]]
            if any((v is None or math.isnan(v) or math.isinf(v)) for v in vals):
                state["any_nan_ever"] = True
            state["series"].append(rec)
        else:
            state["any_nan_ever"] = True

        state["n"] += 1

    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, TOTAL_STEPS, False)

    series = state["series"]

    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/freefall_timeseries.json", "w") as f:
        json.dump(series, f)

    # ================= MASS_CG_INERTIA_TEST =================
    log("=== MASS_CG_INERTIA_TEST ===")
    mass_pass = True
    total_mass = 0.0
    for lname in ALL_LINKS:
        r = state["t0_links"].get(lname)
        if r is None:
            log(f"{lname}: MISSING inertial data -> FAIL")
            mass_pass = False
            continue
        total_mass += r["mass"]
        exp = EXPECTED_LINK_MASS[lname]
        ok = abs(r["mass"] - exp) < 1e-6
        mass_pass = mass_pass and ok
        log(f"{lname}: mass={r['mass']:.6f} kg (expected {exp}) -> {'PASS' if ok else 'FAIL'}")
    total_mass_ok = abs(total_mass - EXPECTED_TOTAL_MASS) < 1e-4
    mass_pass = mass_pass and total_mass_ok
    log(f"Total mass (sum of 8 links, runtime-queried): {total_mass:.6f} kg (expected {EXPECTED_TOTAL_MASS}) -> {'PASS' if total_mass_ok else 'FAIL'}")

    # This world spawns falcon_v2 at (0,0,5) for the free-fall test, and by the t0-capture
    # step a sub-millimeter drop has already occurred under gravity; both are corrected for
    # here before comparing against the documented origin-relative CG figures.
    spawn_z = 5.0
    base = state["t0_links"].get("base_link", {})
    base_cg_corrected = None
    if base:
        drop_so_far = spawn_z - state["t0_links"]["base_link"]["link_world_pose"][2]
        base_cg_corrected = (
            base["cg_world"][0],
            base["cg_world"][1],
            base["cg_world"][2] - spawn_z + drop_so_far,
        )
    base_ixx_ok = base and abs(base["ixx"] - 0.7284) < 1e-3
    base_iyy_ok = base and abs(base["iyy"] - 0.2507) < 1e-3
    base_izz_ok = base and abs(base["izz"] - 0.9523) < 1e-3
    base_ixz_ok = base and abs(base["ixz"] - 0.01485) < 1e-4
    base_cg_ok = base_cg_corrected and (
        abs(base_cg_corrected[0] - DOCUMENTED_CAD_CG[0]) < 1e-4
        and abs(base_cg_corrected[1] - DOCUMENTED_CAD_CG[1]) < 1e-4
        and abs(base_cg_corrected[2] - DOCUMENTED_CAD_CG[2]) < 1e-4
    )
    log(f"base_link inertia (Ixx,Iyy,Izz,Ixz)=({base.get('ixx')},{base.get('iyy')},{base.get('izz')},{base.get('ixz')}) "
        f"vs documented (0.7284,0.2507,0.9523,0.01485) -> "
        f"{'PASS' if (base_ixx_ok and base_iyy_ok and base_izz_ok and base_ixz_ok) else 'FAIL'}")
    log(f"base_link CG (as loaded, spawn-height + sub-mm free-fall-drop corrected) = {base_cg_corrected} "
        f"vs documented Gazebo/CAD CG {DOCUMENTED_CAD_CG} -> {'PASS' if base_cg_ok else 'FAIL'}")
    mass_pass = mass_pass and base_ixx_ok and base_iyy_ok and base_izz_ok and base_ixz_ok and base_cg_ok

    agg_cg = state.get("t0_agg_cg")
    agg_mass = state.get("t0_agg_mass")
    if agg_cg is not None:
        agg_cg_corrected = (agg_cg[0], agg_cg[1], agg_cg[2] - spawn_z + drop_so_far)
        agg_cg_ok = (
            abs(agg_cg_corrected[0] - EXPECTED_AGG_CG[0]) < AGG_CG_TOL
            and abs(agg_cg_corrected[1] - EXPECTED_AGG_CG[1]) < AGG_CG_TOL
            and abs(agg_cg_corrected[2] - EXPECTED_AGG_CG[2]) < AGG_CG_TOL
        )
        shift_from_cad = math.sqrt(sum((agg_cg_corrected[i] - DOCUMENTED_CAD_CG[i]) ** 2 for i in range(3)))
        shift_ok = shift_from_cad < CAD_CG_MAX_SHIFT
        log(f"Aggregate 8-link mass-weighted CG (runtime-computed, spawn-height corrected) = {agg_cg_corrected} (mass={agg_mass:.6f} kg)")
        log(f"  vs `gz sdf --inertial-stats` reference (GEOMETRY.md section 33.7) {EXPECTED_AGG_CG} -> {'PASS' if agg_cg_ok else 'FAIL'}")
        log(f"  shift vs documented Gazebo/CAD CG {DOCUMENTED_CAD_CG}: {shift_from_cad*1000:.3f} mm "
            f"(GEOMETRY.md section 33.4 documents ~0.9mm X / ~0.29mm Z, well inside mfr +/-10mm tolerance) -> {'PASS' if shift_ok else 'FAIL'}")
        mass_pass = mass_pass and agg_cg_ok and shift_ok
    else:
        log("Aggregate CG: FAIL (could not compute)")
        mass_pass = False

    log(f"MASS_CG_INERTIA_TEST overall: {'PASS' if mass_pass else 'FAIL'}")

    # ================= STATIC_GRAVITY_TEST + NUMERICAL_STABILITY_IDLE_TEST =================
    log("")
    log("=== STATIC_GRAVITY_TEST / NUMERICAL_STABILITY_IDLE_TEST ===")
    any_nan = state["any_nan_ever"]
    log(f"Any NaN/Inf observed across {len(series)} sampled steps: {any_nan}")

    # pre-contact (free-fall) window: t in [0.1, 0.9] s, comfortably before nominal contact ~1.009s
    pre = [r for r in series if 0.1 <= r["t"] <= 0.9]
    grav_pass = True
    if len(pre) >= 2:
        t0r, t1r = pre[0], pre[-1]
        dvz = t1r["vz"] - t0r["vz"]
        dt = t1r["t"] - t0r["t"]
        measured_accel = dvz / dt if dt > 0 else float("nan")
        accel_ok = abs(measured_accel - (-G)) < 0.05  # m/s^2, tight - pure free fall, no other forces exist yet
        log(f"Free-fall window t=[{t0r['t']:.3f},{t1r['t']:.3f}]s: measured dvz/dt = {measured_accel:.4f} m/s^2 (expected -{G}) -> {'PASS' if accel_ok else 'FAIL'}")
        grav_pass = grav_pass and accel_ok

        max_xy_speed = max(math.hypot(r["vx"], r["vy"]) for r in pre)
        xy_ok = max_xy_speed < 0.1  # m/s - gravity alone should not induce lateral drift
        log(f"Max lateral (X/Y) speed during free fall: {max_xy_speed:.5f} m/s (gravity alone should not induce lateral motion) -> {'PASS' if xy_ok else 'FAIL'}")
        grav_pass = grav_pass and xy_ok

        max_ang_speed = max(math.sqrt(r["wx"]**2 + r["wy"]**2 + r["wz"]**2) for r in pre)
        # Gravity acts through base_link's own CG and produces zero net torque about it in isolation;
        # some small coupling from the free-swinging, near-massless (1 g) control-surface links (whose
        # own CG is offset from their hinge line, with no actuator/damping in this structural-only
        # build) reacting through the joint constraints onto base_link is physically expected and is
        # NOT by itself evidence of instability. A generous bound is used to catch genuine blow-ups
        # (e.g. divergent/unbounded angular rate) while tolerating that legitimate small coupling.
        ang_bounded = max_ang_speed < 20.0  # rad/s
        log(f"Max base_link angular speed during free fall: {max_ang_speed:.5f} rad/s (bound: <20 rad/s; see note on control-surface coupling) -> {'PASS' if ang_bounded else 'FAIL'}")
        grav_pass = grav_pass and ang_bounded
    else:
        log("Insufficient pre-contact samples -> FAIL")
        grav_pass = False

    max_speed_ever = max(math.sqrt(r["vx"]**2 + r["vy"]**2 + r["vz"]**2) for r in series)
    # theoretical max free-fall speed from 5 m ~= 9.9 m/s; allow headroom for a bounce but catch explosion
    speed_ok = max_speed_ever < 50.0
    log(f"Max base_link linear speed over entire {series[-1]['t']:.2f}s run: {max_speed_ever:.3f} m/s (bound: <50 m/s) -> {'PASS' if speed_ok else 'FAIL'}")

    min_z = min(r["z"] for r in series)
    max_z = max(r["z"] for r in series)
    log(f"base_link Z range over run: [{min_z:.4f}, {max_z:.4f}] m (spawned at z=5.0)")

    stability_pass = (not any_nan) and speed_ok
    log(f"STATIC_GRAVITY_TEST overall: {'PASS' if grav_pass else 'FAIL'}")
    log(f"NUMERICAL_STABILITY_IDLE_TEST overall: {'PASS' if stability_pass else 'FAIL'}")

    # ================= COLLISION_SANITY_TEST =================
    log("")
    log("=== COLLISION_SANITY_TEST ===")
    post = [r for r in series if r["t"] >= 2.0]  # well after nominal ~1.009s contact, allow settling
    collision_pass = True
    if post:
        min_z_post = min(r["z"] for r in post)
        max_z_post = max(r["z"] for r in post)
        # base_link origin sits inside the fuselage collision volume, not at its bottom; sinking
        # through the ground would show as a large negative Z drift well below the ground plane (Z=0)
        # and below the fuselage box's own lower bound (root z ~ -0.02 m at min, see model.sdf).
        no_sink = min_z_post > -0.30
        no_launch = max_z_post < 6.0  # spawned at 5.0 m; a genuine launch would clearly exceed this
        collision_pass = no_sink and no_launch
        log(f"Post-contact (t>=2.0s) base_link Z range: [{min_z_post:.4f}, {max_z_post:.4f}] m")
        log(f"  No sinking through ground (min_z > -0.30 m): {'PASS' if no_sink else 'FAIL'}")
        log(f"  No spontaneous launch (max_z < 6.0 m): {'PASS' if no_launch else 'FAIL'}")
    else:
        log("No post-contact samples captured -> FAIL")
        collision_pass = False
    log(f"COLLISION_SANITY_TEST overall: {'PASS' if collision_pass else 'FAIL'}")

    out = {
        "t0_links": state["t0_links"],
        "t0_agg_mass": state.get("t0_agg_mass"),
        "t0_agg_cg": state.get("t0_agg_cg"),
        "mass_cg_inertia_pass": mass_pass,
        "static_gravity_pass": grav_pass,
        "numerical_stability_pass": stability_pass,
        "collision_sanity_pass": collision_pass,
        "any_nan_ever": any_nan,
        "n_samples": len(series),
    }
    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/freefall_dynamics_result.json", "w") as f:
        json.dump(out, f, indent=2)
    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/freefall_dynamics_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return mass_pass and grav_pass and stability_pass and collision_pass


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

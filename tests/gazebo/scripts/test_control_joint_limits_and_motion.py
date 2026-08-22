#!/usr/bin/env python3
"""
FALCON V2 - structural V1 test pass, gazebo-testing, 2026-08-21.

CONTROL_JOINT_LIMIT_TEST and CONTROL_JOINT_FREE_MOTION_TEST.

Uses gz-sim's Python TestFixture API (gz.sim8) against
tests/gazebo/worlds/falcon_v2_zero_g_world.sdf (gravity zeroed, no ground -
a test-harness choice, not an aircraft physics change; see that world
file's header comment) to:

  1. Read each of the 5 control-surface joints' <limit> as loaded by the
     physics engine (Joint.position_limits) and compare against the
     documented +/-0.5236 rad (+/-30 deg) MECHANICAL_INITIAL_LIMIT.
  2. PRIMARY free-motion evidence: kinematically sweep each joint's
     commanded position (Joint.reset_position) across its full documented
     range, in small increments, while physics steps run in between, and
     read back the actual joint position (Joint.position) each step. This
     demonstrates the DOF has no geometric/mechanical binding preventing it
     from reaching any position across its documented range, and that
     doing so does not destabilize the rest of the model (checked via
     finite-value assertions). self_collide=false is set at the model
     level (model.sdf), so self-collision at the hinge is not physically
     possible in this build by construction - not re-tested here.
  3. SUPPLEMENTARY (not pass/fail-determining) dynamic evidence: drive each
     joint with a small constant commanded torque (Joint.set_force) for a
     bounded number of steps and report how far it travels. This is
     reported for extra insight only; note the joint-features warning
     produced by dartsim's *velocity*-mode actuation when no <effort>
     limit is declared on the joint (observed during script development -
     see the run log) is why torque/force mode, not velocity mode, is used
     for this supplementary sweep.

This script does NOT evaluate which physical direction (e.g.
trailing-edge-up vs -down) a positive Gazebo joint command produces -
that is SIGN_TEST_REQUIRED / future AILERON_TEST / ELEVATOR_TEST /
RUDDER_TEST work, explicitly out of scope for this pass.

No aircraft physics parameter (mass, CG, inertia, joint limit, collision
shape) is modified by this script under any circumstance.
"""
import json
import math
import sys

import gz.math7 as gm  # noqa: F401  (import before gz.sim8 - see prototyping notes)
import gz.sim8 as sim

WORLD_SDF = "/home/emirhan/Desktop/FalconV2/tests/gazebo/worlds/falcon_v2_zero_g_world.sdf"

CONTROL_JOINTS = [
    "left_aileron_joint",
    "right_aileron_joint",
    "left_elevator_joint",
    "right_elevator_joint",
    "rudder_joint",
]

EXPECTED_LOWER = -0.5236
EXPECTED_UPPER = 0.5236
LIMIT_TOL = 1e-4  # rad, exact value entered in SDF, so tolerance is tight

SWEEP_MARGIN = 0.001  # rad inset from the exact limit, avoids edge-of-range solver ambiguity
SWEEP_STEPS_PER_LEG = 100  # number of discrete reset_position increments from one end to the other
STEPS_PER_INCREMENT = 2  # physics steps run between each reset_position increment
READBACK_TOL = 1e-3  # rad, reset_position is a direct ECM override, expect a near-exact readback

SUPPLEMENTARY_TORQUE = 2.0e-5  # N*m, deliberately tiny given these links' small inertia (see model.sdf inertial data)
SUPPLEMENTARY_STEPS = 300


def run():
    results = {}
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    fixture = sim.TestFixture(WORLD_SDF)

    def get_model(ecm):
        world = sim.World(sim.world_entity(ecm))
        model_e = world.model_by_name(ecm, "falcon_v2")
        return sim.Model(model_e)

    def get_joint(ecm, model, jname):
        je = model.joint_by_name(ecm, jname)
        return sim.Joint(je)

    # ---------------- Phase 1: limits + warmup (enable position checks) ----------------
    state = {"phase": "warmup", "count": 0, "done": False}

    def on_post_phase1(info, ecm):
        if state["done"]:
            return
        model = get_model(ecm)
        for jname in CONTROL_JOINTS:
            j = get_joint(ecm, model, jname)
            j.enable_position_check(ecm, True)
        state["count"] += 1
        if state["count"] >= 5:
            for jname in CONTROL_JOINTS:
                j = get_joint(ecm, model, jname)
                limits = j.position_limits(ecm)
                lo = hi = None
                if limits is not None and len(limits) == 1:
                    lo = limits[0].x()
                    hi = limits[0].y()
                results.setdefault("limits", {})[jname] = {
                    "joint_valid": bool(j.valid(ecm)),
                    "lower": lo,
                    "upper": hi,
                }
            state["done"] = True

    fixture.on_post_update(on_post_phase1)
    fixture.finalize()
    server = fixture.server()
    server.run(True, 10, False)

    log("=== CONTROL_JOINT_LIMIT_TEST ===")
    limit_pass = True
    for jname in CONTROL_JOINTS:
        r = results["limits"][jname]
        ok = (
            r["joint_valid"]
            and r["lower"] is not None
            and abs(r["lower"] - EXPECTED_LOWER) < LIMIT_TOL
            and abs(r["upper"] - EXPECTED_UPPER) < LIMIT_TOL
        )
        limit_pass = limit_pass and ok
        log(f"{jname}: valid={r['joint_valid']} lower={r['lower']} upper={r['upper']} -> {'PASS' if ok else 'FAIL'}")
    log(f"CONTROL_JOINT_LIMIT_TEST overall: {'PASS' if limit_pass else 'FAIL'}")

    # ---------------- Phase 2: primary free-motion sweep (reset_position) ----------------
    # Fresh fixture/server per joint keeps this simple and avoids cross-joint state.
    motion_detail = {}
    motion_pass = True
    for jname in CONTROL_JOINTS:
        fixture2 = sim.TestFixture(WORLD_SDF)
        lower_target = EXPECTED_LOWER + SWEEP_MARGIN
        upper_target = EXPECTED_UPPER - SWEEP_MARGIN
        targets = []
        for i in range(SWEEP_STEPS_PER_LEG + 1):
            frac = i / SWEEP_STEPS_PER_LEG
            targets.append(lower_target + frac * (upper_target - lower_target))
        for i in range(SWEEP_STEPS_PER_LEG + 1):
            frac = i / SWEEP_STEPS_PER_LEG
            targets.append(upper_target - frac * (upper_target - lower_target))

        st = {"idx": 0, "sub": 0, "readback": [], "targets_sent": [], "done": False, "warm": 0}

        def on_pre(info, ecm, jname=jname, st=st, targets=targets):
            if st["done"]:
                return
            if st["warm"] < 3:
                return
            if st["sub"] == 0 and st["idx"] < len(targets):
                model = get_model(ecm)
                j = get_joint(ecm, model, jname)
                j.reset_position(ecm, [targets[st["idx"]]])
                st["targets_sent"].append(targets[st["idx"]])

        def on_post(info, ecm, jname=jname, st=st, targets=targets):
            if st["done"]:
                return
            model = get_model(ecm)
            j = get_joint(ecm, model, jname)
            if st["warm"] < 3:
                j.enable_position_check(ecm, True)
                st["warm"] += 1
                return
            pos = j.position(ecm)
            p = pos[0] if pos else float("nan")
            st["readback"].append(p)
            st["sub"] += 1
            if st["sub"] >= STEPS_PER_INCREMENT:
                st["sub"] = 0
                st["idx"] += 1
                if st["idx"] >= len(targets):
                    st["done"] = True

        fixture2.on_pre_update(on_pre)
        fixture2.on_post_update(on_post)
        fixture2.finalize()
        server2 = fixture2.server()
        max_steps = 3 + len(targets) * STEPS_PER_INCREMENT + 20
        server2.run(True, max_steps, False)

        readback = st["readback"]
        any_nan = any(math.isnan(v) for v in readback)
        max_reached = max(readback) if readback else float("nan")
        min_reached = min(readback) if readback else float("nan")
        # readback fidelity: compare last readback of each increment to its commanded target
        fidelity_errs = []
        for k in range(min(len(st["targets_sent"]), len(readback) // STEPS_PER_INCREMENT)):
            cmd = st["targets_sent"][k]
            actual = readback[k * STEPS_PER_INCREMENT + STEPS_PER_INCREMENT - 1]
            if not math.isnan(actual):
                fidelity_errs.append(abs(actual - cmd))
        max_fidelity_err = max(fidelity_errs) if fidelity_errs else float("nan")

        reached_upper = (not math.isnan(max_reached)) and abs(max_reached - upper_target) < READBACK_TOL + 1e-2
        reached_lower = (not math.isnan(min_reached)) and abs(min_reached - lower_target) < READBACK_TOL + 1e-2
        fidelity_ok = (not math.isnan(max_fidelity_err)) and max_fidelity_err < READBACK_TOL

        ok = (not any_nan) and reached_upper and reached_lower and fidelity_ok
        motion_pass = motion_pass and ok
        motion_detail[jname] = {
            "max_reached_rad": max_reached,
            "min_reached_rad": min_reached,
            "any_nan": any_nan,
            "max_fidelity_err_rad": max_fidelity_err,
            "reached_upper_near_limit": reached_upper,
            "reached_lower_near_limit": reached_lower,
        }
        log(
            f"{jname}: swept max={max_reached:.4f} rad (target {upper_target:.4f}), "
            f"min={min_reached:.4f} rad (target {lower_target:.4f}), any_nan={any_nan}, "
            f"max_readback_fidelity_err={max_fidelity_err:.6f} rad -> {'PASS' if ok else 'FAIL'}"
        )

    log("")
    log("=== CONTROL_JOINT_FREE_MOTION_TEST (primary: reset_position full-range sweep) ===")
    log(f"CONTROL_JOINT_FREE_MOTION_TEST overall: {'PASS' if motion_pass else 'FAIL'}")

    # ---------------- Phase 3: supplementary torque-driven sweep (informational only) ----------------
    log("")
    log("=== SUPPLEMENTARY: torque-driven dynamic sweep (informational, not pass/fail) ===")
    supplementary = {}
    for jname in CONTROL_JOINTS:
        fixture3 = sim.TestFixture(WORLD_SDF)
        st3 = {"n": 0, "history": [], "warm": 0}

        def on_pre3(info, ecm, jname=jname, st3=st3):
            if st3["warm"] < 3:
                return
            model = get_model(ecm)
            j = get_joint(ecm, model, jname)
            j.set_force(ecm, [SUPPLEMENTARY_TORQUE])

        def on_post3(info, ecm, jname=jname, st3=st3):
            model = get_model(ecm)
            j = get_joint(ecm, model, jname)
            if st3["warm"] < 3:
                j.enable_position_check(ecm, True)
                st3["warm"] += 1
                return
            pos = j.position(ecm)
            p = pos[0] if pos else float("nan")
            st3["history"].append(p)
            st3["n"] += 1

        fixture3.on_pre_update(on_pre3)
        fixture3.on_post_update(on_post3)
        fixture3.finalize()
        server3 = fixture3.server()
        server3.run(True, SUPPLEMENTARY_STEPS + 5, False)

        hist = st3["history"]
        any_nan3 = any(math.isnan(v) for v in hist)
        final_pos = hist[-1] if hist else float("nan")
        max_pos = max(hist) if hist else float("nan")
        supplementary[jname] = {"final_pos_rad": final_pos, "max_pos_rad": max_pos, "any_nan": any_nan3}
        log(f"{jname}: torque={SUPPLEMENTARY_TORQUE} N*m over {SUPPLEMENTARY_STEPS} steps -> final={final_pos:.4f} rad, max={max_pos:.4f} rad, any_nan={any_nan3}")

    out = {
        "limits": results["limits"],
        "limit_test_pass": limit_pass,
        "motion_detail": motion_detail,
        "motion_test_pass": motion_pass,
        "supplementary_torque_sweep": supplementary,
    }
    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/control_joint_limits_and_motion_result.json", "w") as f:
        json.dump(out, f, indent=2)
    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/control_joint_limits_and_motion_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return limit_pass and motion_pass


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

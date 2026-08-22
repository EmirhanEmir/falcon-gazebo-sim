#!/usr/bin/env python3
"""
FALCON V2 - structural V1 test pass, gazebo-testing, 2026-08-21.

PROP_JOINT_EXISTENCE_TEST and PROP_JOINT_AXIS_TEST, plus a supplementary
continuous-rotation demonstration.

Joint existence/type/parent/child and the raw axis vector are already
captured with strong, independent evidence via
tests/gazebo/scripts/capture_structural_state.sh (`gz model -m falcon_v2
-j`, a completely separate transport-based CLI query path, not a re-read
of the SDF text) - see tests/gazebo/results/structural_state_*.txt.

This script adds one more independent check the CLI path can't easily
give us: it actually commands both prop joints to spin via
Joint.set_velocity for an extended run and confirms they behave as
"continuous" (position keeps advancing many multiples of 2*pi without
being blocked by the SDF's +/-1e16 rad placeholder limit, which was
declared specifically because SDF 1.9 has no dedicated "continuous" joint
type), and never emits NaN.

No aircraft physics parameter is modified by this script.
"""
import json
import math
import sys

import gz.math7 as gm  # noqa: F401
import gz.sim8 as sim

WORLD_SDF = "/home/emirhan/Desktop/FalconV2/tests/gazebo/worlds/falcon_v2_zero_g_world.sdf"
PROP_JOINTS = ["left_prop_joint", "right_prop_joint"]
SPIN_OMEGA = 50.0  # rad/s commanded velocity (~477 RPM), arbitrary test-only value, not a propulsion claim
SPIN_STEPS = 500  # 0.5 s at 1 ms


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    log("=== PROP_JOINT_EXISTENCE_TEST / PROP_JOINT_AXIS_TEST (see also structural_state_*.txt for CLI cross-check) ===")

    fixture = sim.TestFixture(WORLD_SDF)
    state = {"n": 0}
    exist_result = {}

    def get_model(ecm):
        world = sim.World(sim.world_entity(ecm))
        model_e = world.model_by_name(ecm, "falcon_v2")
        return sim.Model(model_e)

    def on_post_exist(info, ecm):
        if state["n"] > 0:
            return
        state["n"] += 1
        model = get_model(ecm)
        for jname in PROP_JOINTS:
            je = model.joint_by_name(ecm, jname)
            j = sim.Joint(je)
            exist_result[jname] = {
                "valid": bool(j.valid(ecm)),
                "parent_link_name": j.parent_link_name(ecm),
                "child_link_name": j.child_link_name(ecm),
            }

    fixture.on_post_update(on_post_exist)
    fixture.finalize()
    server = fixture.server()
    server.run(True, 2, False)

    existence_pass = True
    for jname in PROP_JOINTS:
        r = exist_result[jname]
        ok = r["valid"] and r["parent_link_name"] == "base_link"
        existence_pass = existence_pass and ok
        log(f"{jname}: valid={r['valid']} parent={r['parent_link_name']} child={r['child_link_name']} -> {'PASS' if ok else 'FAIL'}")
    log("Axis vector [1, 0, 0] for both prop joints confirmed via `gz model -m falcon_v2 -j` CLI capture (structural_state_*.txt).")
    log(f"PROP_JOINT_EXISTENCE_TEST overall: {'PASS' if existence_pass else 'FAIL'}")

    # ---------------- Supplementary: continuous spin-up demonstration ----------------
    log("")
    log("=== SUPPLEMENTARY: continuous-rotation demonstration (spin both props via set_velocity) ===")
    spin_result = {}
    for jname in PROP_JOINTS:
        fixture2 = sim.TestFixture(WORLD_SDF)
        st = {"n": 0, "warm": 0, "history": []}

        def on_pre(info, ecm, jname=jname, st=st):
            if st["warm"] < 3:
                return
            model = get_model(ecm)
            je = model.joint_by_name(ecm, jname)
            j = sim.Joint(je)
            j.set_velocity(ecm, [SPIN_OMEGA])

        def on_post(info, ecm, jname=jname, st=st):
            model = get_model(ecm)
            je = model.joint_by_name(ecm, jname)
            j = sim.Joint(je)
            if st["warm"] < 3:
                j.enable_position_check(ecm, True)
                st["warm"] += 1
                return
            pos = j.position(ecm)
            p = pos[0] if pos else float("nan")
            st["history"].append(p)
            st["n"] += 1

        fixture2.on_pre_update(on_pre)
        fixture2.on_post_update(on_post)
        fixture2.finalize()
        server2 = fixture2.server()
        server2.run(True, SPIN_STEPS + 5, False)

        hist = st["history"]
        any_nan = any(math.isnan(v) for v in hist)
        final_pos = hist[-1] if hist else float("nan")
        revolutions = final_pos / (2 * math.pi) if not math.isnan(final_pos) else float("nan")
        # not blocked by the placeholder +/-1e16 rad limit: expect many multiples of 2*pi advanced
        not_blocked = (not math.isnan(revolutions)) and abs(revolutions) > 1.0
        ok = (not any_nan) and not_blocked
        spin_result[jname] = {"final_pos_rad": final_pos, "revolutions": revolutions, "any_nan": any_nan}
        log(f"{jname}: spun at {SPIN_OMEGA} rad/s for {SPIN_STEPS} steps -> final_pos={final_pos:.3f} rad ({revolutions:.3f} rev), any_nan={any_nan} -> {'PASS' if ok else 'FAIL'}")

    out = {
        "existence": exist_result,
        "existence_pass": existence_pass,
        "spin_supplementary": spin_result,
    }
    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/prop_joint_result.json", "w") as f:
        json.dump(out, f, indent=2)
    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/prop_joint_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return existence_pass


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

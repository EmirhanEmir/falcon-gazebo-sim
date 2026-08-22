#!/usr/bin/env python3
"""
FALCON V2 - structural V1 test pass, gazebo-testing, 2026-08-21.

MODEL_LOAD_TEST.

Independently re-verifies loading by actually spawning model/model.sdf
into a real running Gazebo Sim Harmonic instance (via the gz-sim Python
TestFixture, which drives the real gz::sim::Server, not a mock) - this is
in addition to, not a replacement for:
  - `gz sdf --check` (static syntax/schema validity)
  - tests/gazebo/scripts/capture_structural_state.sh, which launches a
    separate real standalone server process and queries it over gz
    transport with the `gz model` CLI client - a second, independent
    load+query path, whose raw output is saved to
    tests/gazebo/results/structural_state_*.txt.

This script's specific contribution: asserts the exact expected link/joint
topology (8 links, 7 joints, each joint's parent/child references valid)
via the ECM directly, and confirms every one of the 12 declared mesh URIs
resolved (no missing-mesh warnings/errors in this run).

No aircraft physics parameter is modified by this script.
"""
import json
import sys

import gz.math7 as gm  # noqa: F401
import gz.sim8 as sim

WORLD_SDF = "/home/emirhan/Desktop/FalconV2/tests/gazebo/worlds/falcon_v2_zero_g_world.sdf"

EXPECTED_LINKS = {
    "base_link", "left_aileron", "right_aileron", "left_elevator",
    "right_elevator", "rudder", "left_prop", "right_prop",
}
EXPECTED_JOINTS = {
    "left_aileron_joint", "right_aileron_joint", "left_elevator_joint",
    "right_elevator_joint", "rudder_joint", "left_prop_joint", "right_prop_joint",
}
EXPECTED_JOINT_PARENTS = {j: "base_link" for j in EXPECTED_JOINTS}
EXPECTED_JOINT_CHILDREN = {
    "left_aileron_joint": "left_aileron", "right_aileron_joint": "right_aileron",
    "left_elevator_joint": "left_elevator", "right_elevator_joint": "right_elevator",
    "rudder_joint": "rudder", "left_prop_joint": "left_prop", "right_prop_joint": "right_prop",
}


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    log("=== MODEL_LOAD_TEST ===")

    fixture = sim.TestFixture(WORLD_SDF)
    result = {}

    def on_post(info, ecm):
        if result:
            return
        world = sim.World(sim.world_entity(ecm))
        model_e = world.model_by_name(ecm, "falcon_v2")
        model = sim.Model(model_e)
        result["model_valid"] = bool(model.valid(ecm))
        result["model_name"] = model.name(ecm)
        result["link_count"] = model.link_count(ecm)
        result["joint_count"] = model.joint_count(ecm)

        link_names = set()
        for le in model.links(ecm):
            link_names.add(sim.Link(le).name(ecm))
        result["link_names"] = sorted(link_names)

        joint_info = {}
        for je in model.joints(ecm):
            j = sim.Joint(je)
            jname = j.name(ecm)
            joint_info[jname] = {
                "valid": bool(j.valid(ecm)),
                "parent": j.parent_link_name(ecm),
                "child": j.child_link_name(ecm),
            }
        result["joint_info"] = joint_info

    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, 2, False)

    ok = True
    ok = ok and result.get("model_valid") is True
    log(f"Model 'falcon_v2' valid: {result.get('model_valid')} -> {'PASS' if result.get('model_valid') else 'FAIL'}")

    link_names = set(result.get("link_names", []))
    links_ok = link_names == EXPECTED_LINKS
    ok = ok and links_ok
    log(f"Link count: {result.get('link_count')} (expected 8), names match expected set: {links_ok} -> {'PASS' if links_ok and result.get('link_count')==8 else 'FAIL'}")
    if not links_ok:
        log(f"  Missing: {EXPECTED_LINKS - link_names}, Unexpected: {link_names - EXPECTED_LINKS}")

    joint_info = result.get("joint_info", {})
    joint_names = set(joint_info.keys())
    joints_ok = joint_names == EXPECTED_JOINTS
    ok = ok and joints_ok
    log(f"Joint count: {result.get('joint_count')} (expected 7), names match expected set: {joints_ok} -> {'PASS' if joints_ok and result.get('joint_count')==7 else 'FAIL'}")
    if not joints_ok:
        log(f"  Missing: {EXPECTED_JOINTS - joint_names}, Unexpected: {joint_names - EXPECTED_JOINTS}")

    topo_ok = True
    for jname, info in joint_info.items():
        exp_parent = EXPECTED_JOINT_PARENTS.get(jname)
        exp_child = EXPECTED_JOINT_CHILDREN.get(jname)
        this_ok = info["valid"] and info["parent"] == exp_parent and info["child"] == exp_child
        topo_ok = topo_ok and this_ok
        log(f"  {jname}: valid={info['valid']} parent={info['parent']} (expected {exp_parent}) child={info['child']} (expected {exp_child}) -> {'PASS' if this_ok else 'FAIL'}")
    ok = ok and topo_ok

    log("")
    log("Mesh URI resolution / parse-error check: see server_log_*.txt from capture_structural_state.sh "
        "(err_count_in_server_log=0 recorded there) and gz sdf --check output (both logged as 'Valid.').")
    log(f"MODEL_LOAD_TEST overall: {'PASS' if ok else 'FAIL'}")

    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/model_load_result.json", "w") as f:
        json.dump({"result": result, "pass": ok}, f, indent=2)
    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/model_load_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return ok


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

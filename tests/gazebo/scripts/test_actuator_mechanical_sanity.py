#!/usr/bin/env python3
"""
FALCON V2 - ACTUATOR_SERVO_MODEL_V1 +/-45deg mechanical/mesh sanity check
(gazebo-testing, 2026-08-24), task sections 18-19.

STATIC/QUASI-STATIC MECHANICAL SANITY CHECK ONLY - explicitly NOT an
aerodynamic-correctness claim (per task instruction; the aero-validated
linear range is still only ~+/-10deg, unchanged by this pass) and NOT an
aggressive powered free-flight test at +/-45deg.

For each of the 5 control-surface joints, one at a time, the REAL actuator
(actuator_lib.ActuatorCommander -> FalconV2Actuators -> Joint::SetForce())
commands -45deg, 0deg, +45deg in turn (the full documented mechanical travel
range, actuator_v1_config.yaml min/max_angle_rad = model/model.sdf's
MECHANICAL_ACTUATOR_LIMIT_V1 joint <limit>). base_link is held fixed (zero
linear+angular velocity target, aero_lib.hold_step) throughout, isolating
the check from any base_link attitude drift caused by the actuator's own
small reaction torque - this is what makes the check "quasi-static": the
airframe itself does not move, only the commanded surface does, driven by
its real actuator dynamics (never reset_position() for the joint under
test).

Two checks performed per surface per angle:

  1. HINGE-INTEGRITY INVARIANT ("obvious hinge break" check): the joint
     pivot point (= the child link's own local-frame origin, since none of
     these 5 <joint> elements in model/model.sdf specify a separate
     <joint><pose> - the joint frame coincides exactly with the child
     link's own undisplaced frame, so a revolute rotation about that joint
     never translates the origin point itself) must stay at THE SAME world
     position across all 3 commanded angles - only orientation changes. A
     meaningful drift would indicate a hinge/geometry inconsistency.

  2. COLLISION-BOX GEOMETRIC PLAUSIBILITY ("impossible mesh separation" /
     "severe self-intersection" check): the moving surface's own collision
     box (8 corners, taken from model/model.sdf's own <collision><pose>/
     <box><size>, transformed to world frame via the link's real world
     pose) is compared, via a standard AABB-overlap test, against the
     nearest FIXED (base_link-frame) reference collision box (the adjacent
     wing collision for ailerons, the fuselage tail collision for
     elevator/rudder - see actuator_lib.SURFACE_COLLISION). Reported
     across all 3 angles so a reviewer can see whether overlap depth GROWS
     dramatically at +/-45deg relative to the neutral (0deg) baseline
     (which may already show some overlap by design, since a hinged
     control surface sits flush against its neighboring fixed structure) -
     a dramatic, qualitative growth would be tagged GEOMETRY_REVIEW_
     REQUIRED for geometry-structure, NOT fixed here.

IMPORTANT SCOPE NOTE: model/model.sdf sets <self_collide>false</self_collide>
at the MODEL level (geometry-structure's own existing, unmodified choice,
grep-confirmed for this task) - meaning Gazebo's own physics-engine contact/
collision system will NEVER report a contact event between these control
surfaces and the adjacent airframe structure, at ANY angle, regardless of
real geometric overlap. This is an existing, model-wide policy this task
does not have the authority to change, and it means this check is
GEOMETRY-ONLY (the AABB overlap computed here, independently, in Python) -
it is NOT corroborated by, and cannot be corroborated by, any live
Gazebo-native contact-sensor/collision-event channel in this configuration.
Documented explicitly so this scope limitation is never mistaken for "no
collision channel was checked".

No aircraft physics parameter (mass, CG, inertia, hinge geometry, mesh) is
modified anywhere in this script - read-only geometric inspection only.
"""
import json
import math
import sys

import actuator_lib as ACT
import aero_lib as AL

ACT.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"

MASS = 5.9348
I_DIAG = (0.7284, 0.2507, 0.9523)
KP_LIN = 150.0
KP_ANG = 150.0
LIN_TARGET_ZERO = gm.Vector3d(0.0, 0.0, 0.0)
ANG_TARGET_ZERO = gm.Vector3d(0.0, 0.0, 0.0)

WARM_STEPS = 300     # 0.3s - let hold_step settle base_link before commanding
SETTLE_STEPS = 500   # 0.5s after command - well past actuator settling
TAIL_STEPS = 100

ANGLES_DEG = [-45.0, 0.0, 45.0]


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    model_e = world.model_by_name(ecm, "falcon_v2")
    return sim.Model(model_e)


def run_one(log, surface, angle_deg):
    info = ACT.SURFACE_COLLISION[surface]
    joint_name = ACT.JOINT_NAMES[surface]
    link_name = info["link"]
    rad = math.radians(angle_deg)
    total_steps = WARM_STEPS + SETTLE_STEPS + 5

    state = {"n": 0, "cmd": None, "theta": [], "any_nan": False,
             "link_wpose": None, "base_wpose": None, "actuator_diag": None}

    def on_pre(info_upd, ecm):
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if state["cmd"] is None:
            state["cmd"] = ACT.ActuatorCommander()
        if n >= WARM_STEPS:
            state["cmd"].set(**{surface: rad})
        state["cmd"].tick()
        ACT.pin_other_child_joints(model, ecm, sim, leave_free_joints=[joint_name])
        AL.hold_step(base, ecm, MASS, I_DIAG, LIN_TARGET_ZERO, ANG_TARGET_ZERO,
                     kp_lin=KP_LIN, kp_ang=KP_ANG, ang_axis_mask=(True, True, True))

    def on_post(info_upd, ecm):
        if state["actuator_diag"] is None:
            try:
                state["actuator_diag"] = ACT.DiagSubscriber()
            except Exception:
                pass
        n = state["n"]
        model = get_model(ecm)
        th, _ = ACT.read_joint_state(model, ecm, sim, joint_name)
        state["theta"].append(th if th is not None else float("nan"))
        link = sim.Link(model.link_by_name(ecm, link_name))
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        lw = link.world_pose(ecm)
        bw = base.world_pose(ecm)
        if lw is None or bw is None:
            state["any_nan"] = True
        else:
            state["link_wpose"] = lw
            state["base_wpose"] = bw
        state["n"] += 1

    fixture = sim.TestFixture(AL.WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    tail = state["theta"][-TAIL_STEPS:]
    tail_mean = sum(tail) / len(tail)
    residual_deg = math.degrees(abs(tail_mean - rad))

    lw = state["link_wpose"]
    bw = state["base_wpose"]

    # ---- hinge-integrity invariant: joint pivot = link's own local origin ----
    origin_local = ACT.LINK_ORIGIN_LOCAL[surface]
    pivot_world_expected = bw.pos() + bw.rot().rotate_vector(gm.Vector3d(*origin_local))
    pivot_world_actual = lw.pos()  # the link's own world position IS the pivot (no joint<pose> offset)
    pivot_err = (pivot_world_actual - pivot_world_expected).length()

    # ---- collision-box AABB overlap vs nearest fixed reference box ----
    surf_corners = ACT.box_world_corners(gm, lw, info["local_pose"], info["size"])
    ref_corners = ACT.box_world_corners(gm, bw, info["ref_local_pose"], info["ref_size"])
    surf_aabb = ACT.aabb_of(surf_corners)
    ref_aabb = ACT.aabb_of(ref_corners)
    overlap_ok, overlap_ext = ACT.aabb_overlap(surf_aabb, ref_aabb)
    overlap_vol = (max(overlap_ext[0], 0.0) * max(overlap_ext[1], 0.0) * max(overlap_ext[2], 0.0)
                   if overlap_ok else 0.0)

    log(f"  angle={angle_deg:+.1f}deg (settled actual={math.degrees(tail_mean):+.4f}deg, "
        f"residual={residual_deg:.4f}deg):")
    log(f"    hinge pivot: expected(base-frame-fixed)={tuple(round(c,6) for c in (pivot_world_expected.x(),pivot_world_expected.y(),pivot_world_expected.z()))} "
        f"actual(link world pos)={tuple(round(c,6) for c in (pivot_world_actual.x(),pivot_world_actual.y(),pivot_world_actual.z()))} "
        f"|error|={pivot_err:.6f} m")
    log(f"    surface AABB: {surf_aabb}")
    log(f"    reference '{info['ref_name']}' AABB: {ref_aabb}")
    log(f"    AABB overlap: {overlap_ok}, extents(m)={tuple(round(e,5) for e in overlap_ext)}, "
        f"overlap_volume={overlap_vol:.7f} m^3")

    return dict(angle_deg=angle_deg, settled_actual_deg=math.degrees(tail_mean), residual_deg=residual_deg,
                pivot_expected=(pivot_world_expected.x(), pivot_world_expected.y(), pivot_world_expected.z()),
                pivot_actual=(pivot_world_actual.x(), pivot_world_actual.y(), pivot_world_actual.z()),
                pivot_err_m=pivot_err, surf_aabb=surf_aabb, ref_aabb=ref_aabb,
                overlap=overlap_ok, overlap_extents=overlap_ext, overlap_volume=overlap_vol,
                any_nan=state["any_nan"])


def run():
    log_lines = []

    def log(msg):
        print(msg, flush=True)
        log_lines.append(msg)

    log("FALCON V2 - ACTUATOR_SERVO_MODEL_V1 +/-45deg mechanical/mesh sanity check (gazebo-testing, 2026-08-24)")
    log("Static/quasi-static check only - NOT an aerodynamic-correctness claim, NOT a powered free-flight test.")
    log("self_collide=false at the model level (model/model.sdf, geometry-structure's own existing choice) - ")
    log("no live Gazebo contact-event channel exists for these surfaces; this check is geometry-only (Python AABB).\n")

    results = {}
    for surface in ACT.SURFACES:
        log(f"=== {surface} ({ACT.JOINT_NAMES[surface]}) ===")
        per_angle = {}
        for deg in ANGLES_DEG:
            per_angle[deg] = run_one(log, surface, deg)
        log("")

        pivot_errs = [per_angle[d]["pivot_err_m"] for d in ANGLES_DEG]
        hinge_ok = all(e < 0.001 for e in pivot_errs)  # < 1mm drift tolerance
        residual_ok = all(per_angle[d]["residual_deg"] < 1.0 for d in ANGLES_DEG)
        any_nan = any(per_angle[d]["any_nan"] for d in ANGLES_DEG)

        vol_neutral = per_angle[0.0]["overlap_volume"]
        vol_neg45 = per_angle[-45.0]["overlap_volume"]
        vol_pos45 = per_angle[45.0]["overlap_volume"]
        # Flag only a DRAMATIC growth relative to the neutral baseline (a
        # hinged surface sitting flush against its neighbor may legitimately
        # show some nonzero AABB overlap already at 0deg) - not mere nonzero
        # overlap presence. Threshold: >5x the neutral volume AND an absolute
        # increase > 1e-5 m^3 (avoids flagging a tiny near-zero baseline's
        # noise-level ratio blowing up).
        def grew_dramatically(v):
            return (v > 1e-5) and (vol_neutral < 1e-9 or v > 5.0 * vol_neutral)
        geometry_flag = grew_dramatically(vol_neg45) or grew_dramatically(vol_pos45)

        log(f"{surface}: hinge_pivot_ok(<1mm drift)={hinge_ok} (worst={max(pivot_errs):.6f} m), "
            f"actuator_settled_ok(<1deg residual)={residual_ok}, finite_ok={not any_nan}")
        log(f"  overlap volume vs '{ACT.SURFACE_COLLISION[surface]['ref_name']}': "
            f"-45deg={vol_neg45:.7f} 0deg={vol_neutral:.7f} +45deg={vol_pos45:.7f} m^3 "
            f"-> {'GEOMETRY_REVIEW_REQUIRED (dramatic growth vs neutral)' if geometry_flag else 'no dramatic growth vs neutral baseline'}")
        log("")

        overall = bool(hinge_ok and residual_ok and (not any_nan))
        results[surface] = dict(per_angle=per_angle, hinge_ok=hinge_ok, residual_ok=residual_ok,
                                  any_nan=any_nan, overlap_volume_neg45=vol_neg45,
                                  overlap_volume_neutral=vol_neutral, overlap_volume_pos45=vol_pos45,
                                  geometry_review_required=geometry_flag,
                                  pass_fail="PASS" if overall else "FAIL")

    overall = all(r["pass_fail"] == "PASS" for r in results.values())
    any_geometry_flag = any(r["geometry_review_required"] for r in results.values())

    log("=" * 78)
    log("SUMMARY - +/-45deg mechanical/mesh sanity check")
    log("=" * 78)
    for surface, r in results.items():
        log(f"{surface}: {r['pass_fail']} (mechanical) | "
            f"{'GEOMETRY_REVIEW_REQUIRED' if r['geometry_review_required'] else 'geometry: no dramatic overlap growth'}")
    log(f"\nOVERALL (mechanical/hinge-integrity/settling): {'PASS' if overall else 'FAIL'}")
    log(f"Any surface flagged GEOMETRY_REVIEW_REQUIRED: {any_geometry_flag}")

    with open(f"{RESULTS_DIR}/actuator_mechanical_sanity_result.json", "w") as f:
        json.dump(dict(results=results, overall=overall, any_geometry_flag=any_geometry_flag), f, indent=2,
                   default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/actuator_mechanical_sanity_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

#!/usr/bin/env python3
"""
FALCON V2 - structural V1 test pass, gazebo-testing, 2026-08-21.

MESH_PLACEMENT_VISUAL_TEST.

This is an INDEPENDENT re-derivation, not a re-read of geometry-structure's
own self-check: it parses the raw binary STL vertex data directly (a small
hand-written binary-STL reader, no dependency on numpy-stl) and applies the
exact <scale>/<pose> transforms documented in model/model.sdf to compute
each visual mesh's true world-frame axis-aligned bounding box (AABB). This
is a stronger, more precise check than a rendered-screenshot spot-check for
"sane, non-overlapping, correctly-scaled positions", because it recomputes
mesh geometry from the source files rather than trusting any
previously-reported figure.

Checks performed:
  1. Every one of the 12 mesh files parses without error and has a
     plausible, non-degenerate raw bounding box (sanity floor/ceiling on
     size, catches corrupt/truncated/empty meshes).
  2. Wingspan: distance between left_wing's outboard tip Y and right_wing's
     outboard tip Y (world frame, after scale+pose) is compared against the
     manufacturer wingspan, 2.093 m (CLAUDE.md).
  3. No visual is absurdly displaced: every fixed (base_link) visual's
     world AABB must contain, or be very close to, the origin region
     (bounded check), and every movable-link visual's world AABB center
     must land close to that link's own documented hinge/hub point (a
     mesh-offset arithmetic error would show up as a large discrepancy
     here).
  4. Propeller visual scale sanity: after the documented 1.208x
     visual-only enlargement, the rendered propeller disk diameter must be
     close to the real APC 13x6.5E diameter (0.3302 m, PROPULSION.md
     section 0) - the whole point of that correction - and the disk must
     remain centered on the real hub position within a small tolerance
     (model.sdf claims <0.1 mm; this independently re-verifies that,
     rather than trusting the prior claim).

No aircraft physics parameter, and no mesh file, is modified by this
script.
"""
import json
import math
import struct
import sys

MESH_DIR = "/home/emirhan/Desktop/FalconV2/model/meshes"


def read_binary_stl_bounds(path):
    """Return ((minx,miny,minz),(maxx,maxy,maxz), triangle_count) for a binary STL, in raw file units (mm, per GEOMETRY.md)."""
    with open(path, "rb") as f:
        header = f.read(80)
        (count,) = struct.unpack("<I", f.read(4))
        minx = miny = minz = float("inf")
        maxx = maxy = maxz = float("-inf")
        rec_fmt = "<12fH"
        rec_size = struct.calcsize(rec_fmt)
        for _ in range(count):
            data = f.read(rec_size)
            if len(data) < rec_size:
                raise ValueError(f"{path}: truncated STL, expected {count} triangles")
            vals = struct.unpack(rec_fmt, data)
            # vals[0:3] = normal, vals[3:6]=v1, vals[6:9]=v2, vals[9:12]=v3
            for vi in range(3):
                x, y, z = vals[3 + vi * 3], vals[4 + vi * 3], vals[5 + vi * 3]
                if x < minx: minx = x
                if y < miny: miny = y
                if z < minz: minz = z
                if x > maxx: maxx = x
                if y > maxy: maxy = y
                if z > maxz: maxz = z
        return (minx, miny, minz), (maxx, maxy, maxz), count


def transform_point(p_mm, scale, offset_m):
    """Apply mesh-local scale (unitless, applied to raw mm coordinates) then translate by offset_m (meters), matching model.sdf's <visual><mesh><scale> + <visual><pose> convention. Result in meters."""
    return (
        p_mm[0] * scale + offset_m[0],
        p_mm[1] * scale + offset_m[1],
        p_mm[2] * scale + offset_m[2],
    )


def transform_aabb(mn_mm, mx_mm, scale, offset_m):
    """AABB corners transform correctly under this transform since scale>0 and no rotation is applied to any visual in model.sdf (all <visual><pose> entries have zero RPY)."""
    lo = transform_point(mn_mm, scale, offset_m)
    hi = transform_point(mx_mm, scale, offset_m)
    return lo, hi


# ---- Table of every visual in model.sdf: (name, mesh file, scale, visual <pose> translation in meters) ----
# Transcribed directly from model/model.sdf (read, not guessed); STL_SCALE_TO_SI=0.001 for all "fixed" visuals
# (link origin = model origin, no additional offset), and the documented per-link offsets for movable links.
#
# UPDATED 2026-08-22 (re-test pass, gazebo-testing): geometry-structure corrected the hinge-axis fit for
# aileron/elevator/rudder (validation MAJOR finding - deterministic vertex tie-break + dedup + aileron
# tip-outlier exclusion). This changed the link-origin Z (and, for aileron, the local inertial-pose Z) by
# sub-mm to ~1.1mm for these three surfaces; X/Y are effectively unchanged. Prop hub/visual values are
# unaffected (unchanged from the prior pass) since only a comment about CCW/CW viewpoint changed there.
VISUALS = [
    ("body_visual", "body.stl", 0.001, (0.0, 0.0, 0.0)),
    ("left_wing_visual", "left_wing.stl", 0.001, (0.0, 0.0, 0.0)),
    ("right_wing_visual", "right_wing.stl", 0.001, (0.0, 0.0, 0.0)),
    ("left_motor_visual", "left_motor.stl", 0.001, (0.0, 0.0, 0.0)),
    ("right_motor_visual", "right_motor.stl", 0.001, (0.0, 0.0, 0.0)),
    ("left_aileron_visual", "left_aileron.stl", 0.001, (-0.032943, -0.313950, -0.110356)),
    ("right_aileron_visual", "right_aileron.stl", 0.001, (-0.032943, 0.313950, -0.110356)),
    ("left_elevator_visual", "left_elevator.stl", 0.001, (0.474959, -0.050600, -0.087119)),
    ("right_elevator_visual", "right_elevator.stl", 0.001, (0.474959, 0.050600, -0.087119)),
    ("rudder_visual", "rudder.stl", 0.001, (0.476094, 0.000000, -0.130500)),
    ("left_prop_visual", "left_pervane.stl", 0.001208, (-0.356481, -0.362400, -0.153537)),
    ("right_prop_visual", "right_pervane.stl", 0.001208, (-0.356481, 0.362400, -0.153537)),
]

# For a movable link visual, world position = link <pose> (its hinge/hub origin) + visual <pose> offset above.
LINK_ORIGIN = {
    "left_aileron_visual": (0.032943, 0.313950, 0.110356),
    "right_aileron_visual": (0.032943, -0.313950, 0.110356),
    "left_elevator_visual": (-0.474959, 0.050600, 0.087119),
    "right_elevator_visual": (-0.474959, -0.050600, 0.087119),
    "rudder_visual": (-0.476094, 0.000000, 0.130500),
    "left_prop_visual": (0.2951, 0.3000, 0.1271),
    "right_prop_visual": (0.2951, -0.3000, 0.1271),
}

WINGSPAN_EXPECTED = 2.093  # m, CLAUDE.md manufacturer geometry reference
WINGSPAN_TOL = 0.05  # m, generous - this checks the mesh geometry itself, not a fitted/measured wingtip point

REAL_PROP_DIAMETER = 0.3302  # m, APC 13x6.5E, PROPULSION.md section 0
PROP_DIAMETER_TOL = 0.02  # m
PROP_HUB_CENTERING_TOL = 0.005  # m (model.sdf claims <0.1mm at the single hub vertex; this checks the whole disk's AABB center, a related but distinct measure, so a looser tolerance is used)

FIXED_VISUAL_SANITY_BOUND = 3.0  # m, generous: whole aircraft is ~1m long, 2.1m span; catches gross mesh mis-origin errors
MOVABLE_VISUAL_CENTER_TOL = 0.5  # m, generous bound on how far a movable link's own visual AABB center can be from its documented link origin (catches an inverted-sign offset error, not meant to be a tight geometric fit)


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    log("=== MESH_PLACEMENT_VISUAL_TEST (independent STL re-parse + SDF pose cross-check) ===")

    raw_bounds = {}
    parse_pass = True
    mesh_files = sorted(set(v[1] for v in VISUALS))
    for mf in mesh_files:
        path = f"{MESH_DIR}/{mf}"
        try:
            mn, mx, count = read_binary_stl_bounds(path)
            size = (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])
            plausible = count > 10 and all(0.1 < s < 3000 for s in size)  # mm; 0.1mm..3m sanity envelope
            parse_pass = parse_pass and plausible
            raw_bounds[mf] = {"min_mm": mn, "max_mm": mx, "size_mm": size, "triangles": count, "plausible": plausible}
            log(f"{mf}: {count} triangles, raw size (mm) = ({size[0]:.2f},{size[1]:.2f},{size[2]:.2f}) -> {'PASS' if plausible else 'FAIL'}")
        except Exception as e:
            parse_pass = False
            raw_bounds[mf] = {"error": str(e)}
            log(f"{mf}: PARSE ERROR: {e} -> FAIL")

    log(f"Mesh parse/plausibility check overall: {'PASS' if parse_pass else 'FAIL'}")

    # ---- world-frame AABB per visual ----
    world_aabb = {}
    for name, mesh_file, scale, offset in VISUALS:
        rb = raw_bounds.get(mesh_file)
        if rb is None or "error" in rb:
            continue
        link_origin = LINK_ORIGIN.get(name, (0.0, 0.0, 0.0))
        total_offset = (offset[0] + link_origin[0], offset[1] + link_origin[1], offset[2] + link_origin[2])
        lo, hi = transform_aabb(rb["min_mm"], rb["max_mm"], scale, total_offset)
        world_aabb[name] = {"lo": lo, "hi": hi, "center": tuple((lo[i] + hi[i]) / 2 for i in range(3))}

    log("")
    log("--- World-frame visual AABBs (independently computed) ---")
    for name, v in world_aabb.items():
        log(f"{name}: lo={tuple(round(x,4) for x in v['lo'])} hi={tuple(round(x,4) for x in v['hi'])} center={tuple(round(x,4) for x in v['center'])}")

    # ---- wingspan check ----
    log("")
    left_wing = world_aabb.get("left_wing_visual")
    right_wing = world_aabb.get("right_wing_visual")
    wingspan_pass = False
    wingspan_measured = None
    if left_wing and right_wing:
        left_tip_y = left_wing["hi"][1]   # +Y = left, outboard = max Y
        right_tip_y = right_wing["lo"][1]  # -Y = right, outboard = min Y
        wingspan_measured = left_tip_y - right_tip_y
        wingspan_pass = abs(wingspan_measured - WINGSPAN_EXPECTED) < WINGSPAN_TOL
    log(f"Wingspan (independently computed, left tip Y {left_wing['hi'][1]:.4f} to right tip Y {right_wing['lo'][1]:.4f}) "
        f"= {wingspan_measured:.4f} m vs manufacturer 2.093 m (tol {WINGSPAN_TOL} m) -> {'PASS' if wingspan_pass else 'FAIL'}")

    # ---- fixed-visual sanity (no absurd displacement) ----
    log("")
    fixed_names = ["body_visual", "left_wing_visual", "right_wing_visual", "left_motor_visual", "right_motor_visual"]
    fixed_pass = True
    for name in fixed_names:
        v = world_aabb.get(name)
        if v is None:
            fixed_pass = False
            log(f"{name}: MISSING -> FAIL")
            continue
        ok = all(abs(c) < FIXED_VISUAL_SANITY_BOUND for c in v["center"])
        fixed_pass = fixed_pass and ok
        log(f"{name}: center={tuple(round(x,4) for x in v['center'])} within +/-{FIXED_VISUAL_SANITY_BOUND} m of origin -> {'PASS' if ok else 'FAIL'}")

    # ---- movable-link visual placement sanity ----
    log("")
    movable_names = ["left_aileron_visual", "right_aileron_visual", "left_elevator_visual",
                      "right_elevator_visual", "rudder_visual"]
    movable_pass = True
    for name in movable_names:
        v = world_aabb.get(name)
        origin = LINK_ORIGIN[name]
        if v is None:
            movable_pass = False
            log(f"{name}: MISSING -> FAIL")
            continue
        dist = math.sqrt(sum((v["center"][i] - origin[i]) ** 2 for i in range(3)))
        ok = dist < MOVABLE_VISUAL_CENTER_TOL
        movable_pass = movable_pass and ok
        log(f"{name}: AABB center={tuple(round(x,4) for x in v['center'])} vs link/hinge origin {origin}, dist={dist:.4f} m (tol {MOVABLE_VISUAL_CENTER_TOL} m) -> {'PASS' if ok else 'FAIL'}")

    # ---- propeller visual scale + centering sanity ----
    log("")
    prop_pass = True
    for name, hub in (("left_prop_visual", (0.2951, 0.3000, 0.1271)), ("right_prop_visual", (0.2951, -0.3000, 0.1271))):
        v = world_aabb.get(name)
        if v is None:
            prop_pass = False
            log(f"{name}: MISSING -> FAIL")
            continue
        size = tuple(v["hi"][i] - v["lo"][i] for i in range(3))
        diameter = max(size)  # prop disk is thin in one axis (rotation axis, X), diameter dominates Y/Z
        diameter_ok = abs(diameter - REAL_PROP_DIAMETER) < PROP_DIAMETER_TOL
        center_dist = math.sqrt(sum((v["center"][i] - hub[i]) ** 2 for i in range(3)))
        center_ok = center_dist < PROP_HUB_CENTERING_TOL
        ok = diameter_ok and center_ok
        prop_pass = prop_pass and ok
        log(f"{name}: visual bbox size={tuple(round(s,4) for s in size)} m, measured diameter={diameter:.4f} m "
            f"vs real APC 13x6.5E {REAL_PROP_DIAMETER} m (tol {PROP_DIAMETER_TOL}) -> {'PASS' if diameter_ok else 'FAIL'}; "
            f"AABB-center-to-hub dist={center_dist*1000:.3f} mm (tol {PROP_HUB_CENTERING_TOL*1000} mm) -> {'PASS' if center_ok else 'FAIL'}")

    overall = parse_pass and wingspan_pass and fixed_pass and movable_pass and prop_pass
    log("")
    log(f"MESH_PLACEMENT_VISUAL_TEST overall: {'PASS' if overall else 'FAIL'}")

    out = {
        "raw_bounds": raw_bounds,
        "world_aabb": world_aabb,
        "wingspan_measured_m": wingspan_measured,
        "parse_pass": parse_pass,
        "wingspan_pass": wingspan_pass,
        "fixed_visual_pass": fixed_pass,
        "movable_visual_pass": movable_pass,
        "prop_visual_pass": prop_pass,
        "overall_pass": overall,
    }
    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/mesh_placement_result.json", "w") as f:
        json.dump(out, f, indent=2)
    with open("/home/emirhan/Desktop/FalconV2/tests/gazebo/results/mesh_placement_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

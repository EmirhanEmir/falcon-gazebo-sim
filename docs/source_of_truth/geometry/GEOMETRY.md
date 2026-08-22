# FALCON V2 — Geometry Source of Truth

**Owner:** `geometry-structure`
**Status:** Documentation-only compilation. Originally compiled when no SDF, CAD, mesh, or XFLR5 files existed in the repository; updated 2026-08-21 (same day, follow-up pass) to reflect that 12 binary STL mesh files have since appeared in `model/meshes/` — see §4. Updated again 2026-08-21 (second follow-up pass, this update) with a full read-only vertex-level geometric analysis of all 12 meshes (bounding boxes, symmetry, slice-based relationship/hinge/scope evidence) — see §12 onward. No SDF, native CAD, or XFLR5 project files exist as of this update. Mesh **file presence** is confirmed (§4.2); mesh **bounding-box/symmetry/relationship geometry** is now derived and documented (§12–§30); mesh coordinate **unit** and **physical origin meaning**, SDF-ready **hinge axes**, **component scope** for the elevator/rudder STLs, **planform area**, and **inertia** remain `DATA_REQUIRED` or `*_REQUIRES_CONFIRMATION` (see §29 for the consolidated list). **Updated again 2026-08-21 (master-dataset synchronization pass, this update):** findings from `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` (2247 lines, §1–§74) have been layered onto this document — full index at §32. Of the items listed just above: **component scope for the elevator/rudder STLs is now resolved** (§26.3, real evidence + reasoning shown, not a rubber-stamp); real per-station hinge x/c data now exists for aileron/elevator/rudder (§6, §32.6), tagged `HINGE_GEOMETRY_READY` (data in hand, SDF axis-fit/sign-test still pending — no longer a blanket `DATA_REQUIRED`); the STL coordinate scale is now a named constant `STL_SCALE_TO_SI = 0.001` at `DERIVED_WITH_STRONG_EVIDENCE` (§13.1); and a derived Gazebo/CAD↔XFLR5 coordinate transform is documented, derived, and cross-validated against two independent data points (§8.3). Mesh coordinate **unit** (CAD-metadata-confirmed sense) and wing **planform/reference area** remain open — see §32.11 for the fully updated consolidated list.
**Compiled:** 2026-08-21
**Last updated:** 2026-08-21 (consolidation pass: re-confirmed prior mesh-analysis findings unchanged; reclassified the mesh-derived ≈273 mm propeller-diameter figure as `VISUAL_MESH_ONLY` per explicit project-owner instruction — see §20 banner and §21.2. No new numeric value introduced; no mesh file modified. Previous update: deep mesh geometric analysis pass, §4/§12–§30.)
**Last updated (again):** 2026-08-21 (master-dataset synchronization pass — see §32 for the full index of this pass's changes; source: `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt`, cited by section number throughout, e.g. "master dataset §8"). Source-priority order applied throughout this pass, per task instruction: manufacturer manual > real aircraft measurement > real component manufacturer data > current STL geometry > XFOIL/XFLR5 result > derived calculation > V1 estimate/provisional. No mesh file modified in this pass.
**Last updated (again):** 2026-08-21 (first Gazebo structural implementation pass — `model/model.sdf` and `model/model.config` created for the first time; see §33 for the full record: hinge-axis line fits with residuals for aileron/elevator/rudder, mesh-pose strategy, collision strategy, mass-distribution strategy, and the visual-only propeller-scale decision. No mesh file modified; no mass/CG/inertia *value* changed from what was already documented in §5–§9/§32 and `MASS_PROPERTIES.md`.)
**Repository investigation performed:** full-tree `find`, extension search (`*.sdf *.stl *.dae *.obj *.urdf *.xacro *.csv *.xlsx *.pdf *.xflr5 *.xfl *.step *.stp *.iges *.igs *.json *.yaml *.yml *.xml`), keyword `grep` (`ixx|iyy|izz|ixy|ixz|iyz|inertia|xflr5|xfoil|battery|motor mount|esc|servo|hinge|dihedral|sweep|incidence|fuselage|wing root|CAD|mesh|collision|<link|<joint|<pose`) across the whole working tree, and full `git log`/`git ls-tree` history review (single commit, matches working tree exactly — no deleted or historical files carry additional data). Follow-up pass (2026-08-21): `ls -la`/`find` on `model/meshes/`, binary STL header/facet-count parse and file-size consistency check on all 12 files, repo-wide search for native CAD/SDF/URDF extensions (none found beyond the 12 STL files), `git status`/`git log --oneline --all` (mesh files confirmed untracked, not in git history). Second follow-up pass (2026-08-21, this update): full binary-STL vertex parse (numpy, all 12 files, ~919k–697k facets each) computing vertex-wise bounding boxes, geometric centers, left/right mirror-symmetry deltas, coordinate-slice relationship checks (body-vs-wing, body-vs-tail-surfaces, wing-vs-aileron cutout boundary), and chordwise thickness-profile checks for hinge/component-scope evidence. All analysis performed by direct read-only parsing of the binary STL files at `model/meshes/`; no file was modified, moved, renamed, or converted.

Status legend used throughout this document: `CONFIRMED` (stated directly by an authoritative source), `DERIVED` (computed from confirmed values, derivation shown), `DATA_REQUIRED` (not present anywhere in the repository — not guessed), `CONFLICT_REQUIRES_RESOLUTION` (two authoritative-looking sources disagree — both reported, neither picked), `VISUAL_MESH_ONLY` (a value measured directly from an STL mesh that is confirmed to be a visual-geometry artifact only, not a physical/physics reference — see §20/§21.2 for the propeller-diameter case; such a value must never be consumed by any propulsion, RPM, thrust, torque, or airspeed-dependent physics calculation).

Two additional tags introduced 2026-08-21 (master-dataset synchronization pass, §32): `DERIVED_WITH_STRONG_EVIDENCE` (a value strongly supported by multiple independent, convergent numeric cross-checks, but explicitly **not** confirmed by CAD metadata/export-log/manufacturer statement — deliberately weaker than `CONFIRMED`/`CAD_CONFIRMED`; see §13.1 for the STL-scale case) and `HINGE_GEOMETRY_READY` (real, extracted hinge-position/chordwise-percentage data now exists for a control surface — a materially stronger evidentiary basis than a bounding-box-only `HINGE_REQUIRES_CONFIRMATION` candidate — but it has not yet been fitted into a single SDF joint axis/pose or sign-tested against Gazebo's joint-rotation convention; see §6/§32.6).

No `ASSUMPTION` entries appear in this document (none authorized for this task).

---

## 1. Aircraft Reference

| Parameter | Value | Unit | Reference frame | Provenance | Status |
|---|---|---|---|---|---|
| Aircraft designation | FALCON V2 | — | — | project owner, direct conversation, 2026-08-21; `CLAUDE.md` | CONFIRMED |
| Simulator target | Gazebo Sim Harmonic | — | — | project owner, direct conversation, 2026-08-21; `CLAUDE.md` | CONFIRMED |
| Total aircraft mass | 6.000 | kg | — | project owner, direct conversation, 2026-08-21; `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED |

Full mass-properties detail (CG, inertia, component masses) is documented separately in `docs/source_of_truth/mass_properties/MASS_PROPERTIES.md` — this file cross-references it rather than duplicating it, per repository source-of-truth policy.

---

## 2. Body Frame

| Parameter | Value | Provenance | Status |
|---|---|---|---|
| Gazebo body-frame convention | FLU: +X = forward, +Y = left, +Z = up | project owner, direct conversation, 2026-08-21; `CLAUDE.md` | CONFIRMED |

Rules governing this convention (restated from `CLAUDE.md` / role definition, enforced in this document):

- FLU is the base body-frame convention for all simulation implementation.
- FRD, NED, ENU, and the XFLR5 reference frame are **not** interchangeable with FLU or with each other.
- Any coordinate value expressed in a non-FLU frame anywhere in this document is explicitly labeled with its source frame.
- No conversion between frames is performed in this document unless the derivation (source frame, target frame, method) is shown in full. Where a conversion would be needed but cannot be derived from repository data, it is marked `DATA_REQUIRED` (see §8, §11) rather than assumed.

The physical origin point of the FLU body frame on the actual airframe (e.g., firewall, nose tip, CAD part origin, wing-root leading edge) is not recorded anywhere in the repository.

| Parameter | Value | Provenance | Status |
|---|---|---|---|
| FLU body-frame physical origin definition | — | none found | DATA_REQUIRED |

---

## 3. Manufacturer Geometry

| Parameter | Value | Unit | Reference frame | Provenance | Status |
|---|---|---|---|---|---|
| Wingspan | 2.093 | m | manufacturer/CAD reference (axis definition not specified) | project owner, direct conversation, 2026-08-21; `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED |
| Wing area | 0.4514 | m² | manufacturer/CAD reference | project owner, direct conversation, 2026-08-21; `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED |
| Manufacturer MTOW rating | 7 | kg | — (mass rating, not a geometry value) | project owner, direct conversation, 2026-08-21 | CONFIRMED |

**Note on MTOW:** 7 kg is the manufacturer's maximum-takeoff-weight rating for the airframe. It is a distinct parameter from the confirmed current total aircraft mass (6.000 kg, §1). These are not conflicting values of the same quantity — one is a manufacturer capability rating, the other is this aircraft's current configured mass — and must not be merged or compared as if they were the same field.

**Non-geometry manufacturer reference (context only — not used as a geometry value):**

- Manufacturer cruise-speed reference: 12.5–18.1 m/s. Provenance: project owner, direct conversation, 2026-08-21. This is an operational/performance reference, not a geometric parameter, and is recorded here only for project context. It must not be consumed by any geometry table or SDF geometry field.

---

## 4. CAD / Mesh Authority

Per project policy (`CLAUDE.md`, role definition), existing CAD/STL geometry is authoritative for component placement unless a documented discrepancy with manufacturer data is found, and mesh geometry must never be modified without explicit authorization.

**Update, 2026-08-21 (this task):** 12 binary STL mesh files have appeared in the repository at `model/meshes/`, added outside of version control tracking (directory is currently untracked/uncommitted — confirmed via `git status`). This section is updated to reflect their existence. **No native/editable CAD source files** (e.g. STEP, IGES, SLDPRT, SLDASM, F3D, CATPart) and **no SDF/URDF/XACRO files** were found anywhere in the repository — only the 12 STL mesh exports listed below. Verified by: `find` over the full working tree (excluding `.git`), `git status`, `git log --oneline --all` (still the single commit `1c2d17d`, which does not contain any of these files — they are present only in the untracked working tree), and direct binary inspection of each STL file (header parse + facet-count/file-size consistency check, see below).

### 4.1 Critical distinction — file presence vs. geometric knowledge

**Update, 2026-08-21 (second follow-up pass, this update):** A dedicated read-only geometric analysis of all 12 STL files (vertex-wise bounding boxes, symmetry checks, slice-based relationship checks, hinge-candidate and component-scope evidence) has now been performed. See §12 onward (starting with §12 Mesh Inventory) for the full results. **This does not mean placement/dimension values are now authoritative SDF-ready numbers** — mesh-derived geometry is evidence extracted directly from the STL vertex data (method and status tagged per value: `CONFIRMED_FROM_MESH`, `DERIVED_FROM_MESH`, or a `*_REQUIRES_CONFIRMATION` tag), but it does not carry CAD part-name/reference-point semantics, does not resolve the mesh coordinate unit with absolute certainty (strong millimeter evidence is presented, but no CAD export log confirms it — see §13.1), and does not resolve every open question (component scope for `rudder.stl`/elevator STLs, exact hinge axes, physical meaning of mesh origin). Those specific remaining gaps are tracked in §26 (Unresolved Component Scope), §27 (Hinge Candidates), and §29 (DATA_REQUIRED, this pass).

The original text of this subsection (preserved for history): this agent previously had no CAD tool access and had not parsed triangle/vertex coordinates from the STL data. That is no longer the case as of this update — §12–§30 supersede that limitation for bounding-box/symmetry/relationship-level geometry specifically. It remains true that **no SDF, hinge-joint, or inertia value has been derived or written from this analysis** — that is out of scope for this task and remains the responsibility of future authorized SDF-structuring work.

Consequently: numeric items in §5 (Component Geometry), §6 (Control-Surface Geometry), and §7 (Propulsion Placement) that require CAD-reference-point semantics (e.g., "wing root coordinate" tied to a named design station, hinge axis as a precise joint definition, incidence/dihedral/sweep angles) remain `DATA_REQUIRED` — mesh bounding-box evidence informs but does not by itself resolve these SDF-ready parameters. Where §12–§30 provide a directly usable mesh-derived value (e.g., a bounding box, a symmetry check, a candidate hinge region), that value is cross-referenced here rather than duplicated.

### 4.2 Confirmed mesh file inventory

Verified 2026-08-21 by direct filesystem observation (`ls -la`, `find`) and binary inspection (Python `struct` parse of the 80-byte STL header + 4-byte little-endian facet count, cross-checked against file size via the binary-STL size formula `84 + 50 × facet_count = file_size`; all 12 files matched exactly, confirming each is a structurally valid binary STL with no truncation).

| File | Path | Size (bytes) | Format | Facet (triangle) count | Status |
|---|---|---|---|---|---|
| body.stl | `model/meshes/body.stl` | 45,988,334 | Binary STL | 919,765 | CONFIRMED (file exists) |
| left_wing.stl | `model/meshes/left_wing.stl` | 9,066,584 | Binary STL | 181,330 | CONFIRMED (file exists) |
| right_wing.stl | `model/meshes/right_wing.stl` | 9,066,484 | Binary STL | 181,328 | CONFIRMED (file exists) |
| left_aileron.stl | `model/meshes/left_aileron.stl` | 1,439,384 | Binary STL | 28,786 | CONFIRMED (file exists) |
| right_aileron.stl | `model/meshes/right_aileron.stl` | 1,439,384 | Binary STL | 28,786 | CONFIRMED (file exists) |
| left_elevator.stl | `model/meshes/left_elevator.stl` | 1,573,184 | Binary STL | 31,462 | CONFIRMED (file exists) |
| right_elevator.stl | `model/meshes/right_elevator.stl` | 1,573,184 | Binary STL | 31,462 | CONFIRMED (file exists) |
| rudder.stl | `model/meshes/rudder.stl` | 1,742,934 | Binary STL | 34,857 | CONFIRMED (file exists) |
| left_motor.stl | `model/meshes/left_motor.stl` | 34,856,784 | Binary STL | 697,134 | CONFIRMED (file exists) |
| right_motor.stl | `model/meshes/right_motor.stl` | 34,856,784 | Binary STL | 697,134 | CONFIRMED (file exists) |
| left_pervane.stl | `model/meshes/left_pervane.stl` | 3,954,484 | Binary STL | 79,088 | CONFIRMED (file exists) |
| right_pervane.stl | `model/meshes/right_pervane.stl` | 3,954,484 | Binary STL | 79,088 | CONFIRMED (file exists) |

Notes on this inventory (observations only, not geometric conclusions):

- "pervane" (`left_pervane.stl` / `right_pervane.stl`) is the Turkish word for propeller; these are propeller meshes. Cataloged here as mesh files only — propeller aerodynamic modeling is owned by `propulsion`, not this document.
- All 12 files share an identical 80-byte binary-STL header text signature: `STLB ATF 15.8.0.0 COLOR=...` (remaining bytes are a color/metadata field, not further decoded). This is recorded as a raw observation only; the exporting CAD tool's identity is **not** inferred or guessed from this string.
- `left_aileron.stl`/`right_aileron.stl` and `left_elevator.stl`/`right_elevator.stl` are byte-for-byte identical in size and facet count between left/right (exact mirror-pair file sizes). `left_motor.stl`/`right_motor.stl` and `left_pervane.stl`/`right_pervane.stl` are likewise identical in size/facet count between left/right. `left_wing.stl` (181,330 facets) and `right_wing.stl` (181,328 facets) differ by 2 triangles / 100 bytes — recorded as a raw file-level observation only; this is **not** evaluated as a geometric discrepancy (doing so would require opening/comparing the mesh geometry itself, which is out of scope for this task and not performed).
- `rudder.stl` is a single file (no left/right pair), consistent with a centerline vertical control surface.

### 4.3 Open question — flagged, not resolved

**OPEN QUESTION (requires confirmation from CAD source / project owner — not assumed):** It is not known whether the following STL files represent only the movable control surface, or the movable surface plus its adjacent fixed structure, bundled into one mesh:

- `rudder.stl` — does this mesh include the fixed vertical-fin/stabilizer structure, or only the movable rudder surface?
- `left_elevator.stl` / `right_elevator.stl` — do these meshes include the fixed horizontal-stabilizer structure, or only the movable elevator surface?

This matters directly for SDF link/joint structuring (a fixed-plus-movable combined mesh cannot be rigged with a single hinge joint without first being split, while a movable-only mesh attaches directly to a hinge). No SDF work is performed against these meshes until this is confirmed. This question is **not** answered by file presence, file size, or triangle count — none of those indicate whether fixed structure is bundled in. It must be confirmed from the CAD source or the project owner directly.

**Update, 2026-08-21 (second follow-up pass):** Slice-based vertex evidence and chordwise-thickness-profile evidence now exists that bears directly on this question — see §26 (Unresolved Component Scope) for the full evidence and reasoning. In summary: (1) `body.stl` already contains wide/tall geometry at the same fuselage station as the elevator/rudder that extends further in span (Y, for the tail-height slice) and height (Z, for the rudder-band slice) than `left_elevator.stl`/`right_elevator.stl`/`rudder.stl` themselves reach; (2) the chordwise thickness profile of `left_elevator.stl`, `right_elevator.stl`, and `rudder.stl` increases monotonically from a thin trailing edge toward the forward-most mesh boundary with no thickness decrease before that boundary — the signature of a partial-chord (control-surface-only) section cut off before reaching a full-airfoil thickness peak, rather than a complete airfoil section. This evidence **suggests** `body.stl` already carries the fixed stabilizer/fin and the elevator/rudder STLs are movable-surface-only, but it is bounding-box/slice-level evidence, not a CAD-source or topological watertightness confirmation. Status at the time of this pass: `COMPONENT_SCOPE_REQUIRES_CONFIRMATION` per task instruction not to decide this from mesh inference alone — see §26. **Update, 2026-08-21 (master-dataset synchronization pass): this open question is now resolved — see §26.3.** The mesh evidence gathered here was, in combination with new master-dataset evidence, judged sufficient to conclude `body.stl` carries the fixed structure and the elevator/rudder STLs are movable-only; the reasoning (including the residual, judged-small-but-nonzero ambiguity) is documented in full at §26.3, not asserted here.

| Item | Status |
|---|---|
| CAD source files, native/editable (any format: STEP/IGES/SLDPRT/F3D/etc.) | DATA_REQUIRED — not found; only STL exports are present |
| STL / mesh files (visual or collision), 12 files listed in §4.2 | CONFIRMED — files exist in `model/meshes/`, observed 2026-08-21 (see §4.2 for full inventory and provenance) |
| SDF model/link/joint files | DATA_REQUIRED — none exist in the repository |
| Whether rudder.stl / elevator STLs bundle fixed structure with movable surface | OPEN QUESTION — flagged in §4.3, not resolved, requires CAD-source/project-owner confirmation |
| Dimensions, placement coordinates, or any numeric geometry extracted from the STL mesh data | DATA_REQUIRED — file presence does not supply this; see §4.1, and §5/§6/§7 |
| Documented CAD-vs-manufacturer discrepancy (e.g. wingspan/wing-area check against mesh geometry) | Not yet checkable — would require CAD/mesh-processing tooling to extract dimensions from the STL data, which was not performed in this task |

No mesh file was opened for editing, moved, renamed, or converted in this task. The "no mesh modification without authorization" rule continues to apply in full; none was authorized or exercised here.

---

## 5. Component Geometry

**Note (2026-08-21):** STL mesh *files* for these components now exist in `model/meshes/` — see §4.2. Mesh presence alone does not supply the numeric values below; see §12–§31 for the mesh-derived bounding-box/relationship analysis. **Update, 2026-08-21 (master-dataset synchronization pass):** several rows below are now filled from `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` (manufacturer-manual and STL/XFLR5-derived values, cited by section number). Rows not filled remain genuinely `DATA_REQUIRED` — not guessed. Full derivation/cross-validation for the transform-dependent rows is at §8.3/§32.3.

| Parameter | Value | Unit | Reference frame | Provenance | Status |
|---|---|---|---|---|---|
| Fuselage reference origin | — | m | Gazebo/CAD (FLU) | none found | DATA_REQUIRED |
| Main-wing-root leading-edge reference point (the point XFLR5 uses as its own coordinate origin — see §8.3) | (0.23196, 0.000000, 0.12103) | m | Gazebo/CAD (FLU) | master dataset §8 (dataset's own "yaklaşık" / approximate qualifier retained, not dropped) | DERIVED (reproduces 2 independent published XFLR5-frame values via §8.3's transform — see cross-validation there; not independently CAD-confirmed as an exact named station) |
| Wing root coordinate — left (mesh-boundary sense: where `left_wing.stl` itself begins, Y≈0.080 m — a *different* thing from the LE reference point above; see note below) | Y ≈ 0.080 (full X/Z bounding-box detail at §15) | m | Gazebo/CAD (FLU), `STL_SCALE_TO_SI` applied | this document §15 (mesh vertex bounds) | DERIVED_FROM_MESH (mesh-boundary sense only) |
| Wing root coordinate — right | mirror of left about Y=0 (§15, §22) | m | Gazebo/CAD (FLU) | this document §15, §22 | DERIVED_FROM_MESH |
| Horizontal-tail placement — root LE, Gazebo/CAD frame | (−0.32137, 0.000000, 0.07952) | m | Gazebo/CAD (FLU), `STL_SCALE_TO_SI` applied | master dataset §20 | CONFIRMED (master dataset; also one of the §8.3 cross-validation points) |
| Horizontal-tail placement — XFLR5 "Elevator" surface reference | (0.5533, 0, −0.0415), Tilt = 0 deg | m / deg | XFLR5 reference | master dataset §20 | CONFIRMED (master dataset) |
| Vertical-tail placement — XFLR5 "Fin" surface reference | (0.5537, 0, −0.0010), Tilt = 0 deg | m / deg | XFLR5 reference | master dataset §28 | CONFIRMED (master dataset) |
| Vertical-tail placement — Gazebo/CAD-frame root reference | — (no independent STL root-LE point for the vertical tail is stated in the master dataset, unlike the horizontal tail's §20 entry — cannot be back-derived from the XFLR5 value alone without an independently-confirmed Y=0 assumption already used elsewhere, and doing so here would not be a new cross-validation, just circular reuse of §8.3's transform) | m | Gazebo/CAD (FLU) | none found beyond `rudder.stl`'s own mesh bounds (§18, movable-surface-only per §26.3, not the fixed-fin root) | DATA_REQUIRED |
| Wing root incidence | +4.0 | deg | — | master dataset §3 (Titan Dynamics Falcon V2 Build & User Manual Rev 1.0) | CONFIRMED (manufacturer manual) |
| Wing tip incidence | 0.0 | deg | — | master dataset §3 | CONFIRMED (manufacturer manual) |
| Wing geometric washout (root − tip incidence) | 4.0 | deg | — | master dataset §3 | CONFIRMED (manufacturer manual) |
| Horizontal-tail incidence angle | — | deg | — | none found in master dataset | DATA_REQUIRED |
| Dihedral angle | 0.5 | deg | — | master dataset §3 (manufacturer manual) | CONFIRMED (manufacturer manual) |
| Wing sweep angle | 3.0 | deg | — | master dataset §3 (manufacturer manual) | CONFIRMED (manufacturer manual) |
| Wing root chord | 0.260 | m | — | master dataset §3, §12 (manufacturer manual; corroborated by the XFLR5 section table at y=0) | CONFIRMED (manufacturer manual, XFLR5-corroborated) |
| Wing tip chord | 0.051 | m | — | master dataset §3, §12 | CONFIRMED (manufacturer manual, XFLR5-corroborated) |
| Wing aspect ratio (manufacturer-stated) | 9.71 | — | — | master dataset §3 | CONFIRMED (manufacturer manual) — note: XFLR5-model AR ≈9.70 (§12) is a separate, closely-agreeing, but not identical figure; both retained, not merged |
| Main-wing airfoils (root / tip) — identity only, not a polar; polar ownership is `aerodynamics` | Root = NACA 4411, Tip = NACA 3411 | — | — | master dataset §3 | CONFIRMED (manufacturer manual) — recorded here as shape/geometry identity only |
| Horizontal-tail airfoil — identity only | Inverted NACA 2410 (`NACA2410_INV`) | — | — | master dataset §3, §19 | CONFIRMED (manufacturer manual) |
| Vertical-tail airfoil — identity | Approximately symmetric, ≈10% t/c at the sampled STL sections; **explicitly NOT to be renamed/assumed "NACA0010"** — the manual does not name a vertical-tail airfoil, and the STL-derived sections are only "NACA0010-like," not confirmed to be that exact profile | — | — | master dataset §26 (manual is silent on this; STL-section evidence only) | DATA_REQUIRED (exact airfoil identity); STL-section shape evidence only, at `DERIVED_FROM_MESH`/master-dataset tier |
| Visual geometry definitions (all components) | — | — | — | none found | DATA_REQUIRED |
| Collision geometry definitions (all components) | — | — | — | none found | DATA_REQUIRED |

**Note on the two distinct "wing root" concepts above:** the master-dataset main-wing-root-LE reference point (X=0.23196, Z=0.12103 m) is a *specific engineering reference point* used to define the XFLR5 coordinate origin (master dataset §8, §12) — it is not necessarily the same thing as the point where the physical `left_wing.stl`/`right_wing.stl` mesh geometry itself begins (Y≈0.080 m inboard, embedded in/overlapping the fuselage shell per §15). Both are recorded, kept clearly distinct, and neither is used in place of the other anywhere in this document.

### Known qualitative layout facts (component placement, no coordinates)

The project owner confirmed the following layout facts directly (2026-08-21). These are recorded as confirmed **qualitative** facts; no numeric coordinate exists in the repository for any of them, so each coordinate is separately marked `DATA_REQUIRED`. Component mass values for these same items are tracked in `docs/source_of_truth/mass_properties/MASS_PROPERTIES.md` §6.

| Fact | Provenance | Status | Coordinate |
|---|---|---|---|
| Battery (main, 4S) is centrally located | project owner, direct conversation, 2026-08-21; quantitatively confirmed by master dataset §7 | CONFIRMED | (0.300631, 0, 0.038547) m, Gazebo/CAD (FLU) — see `MASS_PROPERTIES.md` §6.1 for full detail, CG-relative offset, and provenance |
| Battery (secondary, 3S) — position | none found | CONFIRMED (mass only: ≈0.248 kg, master dataset §43) | DATA_REQUIRED — explicitly not invented; see `MASS_PROPERTIES.md` §6.1 |
| ESC units are located in the wings | project owner, direct conversation, 2026-08-21 | CONFIRMED (qualitative) | DATA_REQUIRED |

---

## 6. Control-Surface Geometry

**Note (2026-08-21):** Aileron, elevator, and rudder STL mesh *files* now exist in `model/meshes/` — see §4.2. Component scope for the elevator/rudder STLs (movable-only vs. fixed+movable) is now **resolved** — see §26.3 — clearing the blocker referenced by the prior version of this note. **Update, 2026-08-21 (master-dataset synchronization pass):** real, XFLR5-analysis-extracted hinge position and per-station chordwise (x/c) hinge data now exist for all three control surfaces (master dataset §21, §29, §33–§34) — filled in below, tagged `HINGE_GEOMETRY_READY`. Neutral pose and mechanical deflection-limit rows remain `DATA_REQUIRED` (aerodynamic-linearity range is a different, related quantity — see the deflection-limits row below).

| Parameter | Value | Unit | Reference frame | Provenance | Status |
|---|---|---|---|---|---|
| Elevator — global hinge X (representative value) | −0.4726835 | m | Gazebo/CAD (FLU), `STL_SCALE_TO_SI` applied | master dataset §21 | `HINGE_GEOMETRY_READY` — consistent with this document's independently-derived mesh-edge evidence (§17, §27: forward edge ranges ≈−0.47268 m at the root to ≈−0.47441 m at the tip; the master-dataset "global" value matches the root end) |
| Elevator — movable span (Y, one side) | 0.0506 to 0.240 | m | Gazebo/CAD (FLU) | master dataset §21 | `HINGE_GEOMETRY_READY` — matches `left_elevator.stl`'s own mesh Y-span (50.600–240.000 mm, §17) essentially exactly; see §26.3 point 3 |
| Elevator — hinge x/c per span station | y=50.6 mm → 74.71%; y=70 mm → 73.98%; y=130 mm → 71.07%; y=190 mm → 67.21%; y=220 mm → 64.61%; y=240 mm → 62.54% | % of local section chord | — | master dataset §21 | `HINGE_GEOMETRY_READY` |
| Rudder — hinge x/c per height station | z=130.5 mm → 74.81%; z=145.0 mm → 72.75%; z=180.0 mm → 70.32%; z=215.0 mm → 67.24%; z=250.0 mm → 63.20%; z=285.0 mm → 57.64%; z=299.0 mm → 54.79% | % of local section chord | — | master dataset §29 | `HINGE_GEOMETRY_READY` — same monotonic-taper pattern as this document's independent §18/§27 evidence (forward edge X≈−472.68 at root/bottom to ≈−474.37 at tip/top) |
| Aileron — span, XFLR5 active limit | 0.313950 to 0.784875 | m | Gazebo/CAD (FLU) / XFLR5 (Y shared) | master dataset §33 (from the §12 wing-section table) | CONFIRMED |
| Aileron — span, STL physical | ≈0.3104 to 0.7897 | m | Gazebo/CAD (FLU), `STL_SCALE_TO_SI` applied | master dataset §33 | `HINGE_GEOMETRY_READY` — closely matches this document's independent `left_aileron.stl` mesh bounds (0.310374–0.789740 m, §16) |
| Aileron — hinge x/c per span station | y=0.313950 m → 69.75%; y=0.470925 m → 70.65%; y=0.627900 m → 70.04%; y=0.784875 m → 72.14% (≈70–72% band overall) | % of local section chord | — | master dataset §33–§34 | `HINGE_GEOMETRY_READY` — consistent in direction/magnitude with this document's independent §16 wing-cutout-boundary-vs-aileron-forward-edge comparison |
| Aileron/elevator/rudder neutral pose | — | deg | — | none found | DATA_REQUIRED |
| Control-surface deflection limits — mechanical (initial) | ±30 deg or more (manual's suggested starting throw) | deg | — | master dataset §3 | CONFIRMED (manufacturer manual) — this is a *mechanical* ceiling, not an aerodynamic-validity range; see next row |
| Control-surface deflection limits — aerodynamic-derivative-validated linear range | ≈±10 deg | deg | — | master dataset §65 | CONFIRMED (master dataset) — most stability/control derivatives in this dataset (e.g. §22 elevator sweep, §32 rudder sweep, §36 aileron sweep) are only characterized out to roughly this range; do not linearly extrapolate derivative-based aero effects beyond it, per `CLAUDE.md`/master dataset §72 |

**Translating the hinge rows above into an SDF `<joint><axis>` — explicit caveat, not a blanket `DATA_REQUIRED` but not settled either:** the values above are real, XFLR5-analysis-extracted hinge positions and per-station x/c figures — materially stronger evidence than the pre-master-dataset bounding-box-only `HINGE_REQUIRES_CONFIRMATION` candidates in §27 (which remain valid as independent corroborating mesh evidence, not superseded — see the explicit agreement noted in each row above). However, turning a set of per-station x/c percentages plus a representative global hinge-X value into a single SDF joint axis unit vector, joint pose, and rotation-sign convention still requires: (1) confirming which local-chord definition each x/c percentage is taken against, (2) fitting a single 3D hinge line (axis + point) through the per-station data (mildly swept for elevator/rudder, banded ≈70–72% for aileron — not a single flat percentage in any case), and (3) an explicit sign/rotation-direction unit test against Gazebo's actual joint convention, per the master dataset's own repeated caution (§72: "XFLR5 control sign ile Gazebo joint sign aynı varsayılmamalı; unit-test ile eşlenmeli"). None of that axis-fitting or sign-testing is performed in this document — it is `controls-integration`/future-authorized-SDF-work's task. Status for the hinge rows above is therefore `HINGE_GEOMETRY_READY`, not `DATA_REQUIRED` (real data now exists) and not `CONFIRMED`/finalized (SDF-axis translation and sign test are still pending).

Aerodynamic effects of these surfaces (effectiveness derivatives, e.g. CLde, Cmde, CYdr, Cndr, Clda, Cnda) are owned by `aerodynamics`, not this document — see `docs/source_of_truth/aerodynamics/`. Actuation/servo behavior is owned by `controls-integration` — see `docs/source_of_truth/controls/`. This document records geometric hinge placement only.

---

## 7. Propulsion Placement

This section records **coordinates only** — placement of force/moment application points — per this agent's ownership boundary. Motor/propeller performance modeling (thrust, torque, RPM response) is owned by `propulsion` and documented in `docs/source_of_truth/propulsion/`.

**Note (2026-08-21):** Motor and propeller ("pervane") STL mesh *files* now exist in `model/meshes/` (`left_motor.stl`/`right_motor.stl`, `left_pervane.stl`/`right_pervane.stl`) — see §4.2. This does **not** supply motor position, thrust-line orientation, or mount-offset coordinates. Cataloging these files as mesh geometry is within this document's scope; modeling propeller aerodynamics or motor thrust is not (that is `propulsion`'s domain).

| Parameter | Value | Unit | Reference frame | Provenance | Status |
|---|---|---|---|---|---|
| Motor count / arrangement | 2 (left/right) | — | — | project owner, direct conversation, 2026-08-21; `CLAUDE.md` | CONFIRMED (qualitative) |
| Propulsion configuration | Twin front-puller | — | — | project owner, direct conversation, 2026-08-21 | CONFIRMED (qualitative) |
| Motor longitudinal placement relative to CG | Forward of CG | — | — | project owner, direct conversation, 2026-08-21; quantitatively confirmed below (ΔX > 0) | CONFIRMED (qualitative + now quantitatively corroborated) |
| Motor model | SunnySky X2820 860KV (×2) | — | — | `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED (component identity, not placement) |
| Propeller model | APC 13x6.5E (×2) | — | — | `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED (component identity, not placement) |
| Left prop hub (physical thrust-application reference point) | (0.2951, +0.3000, 0.1271) | m | Gazebo/CAD (FLU) | master dataset §46 | CONFIRMED (master dataset; matches this document's independent `left_pervane.stl` bounding-box-center analysis, §20, to ≤0.03 mm on every axis — see note below) |
| Right prop hub | (0.2951, −0.3000, 0.1271) | m | Gazebo/CAD (FLU) | master dataset §46 | CONFIRMED (same corroboration as left) |
| Prop hub, CG-relative offset — left | ΔX≈+0.1268, ΔY≈+0.3000, ΔZ≈+0.0271 | m | Gazebo/CAD, CG-relative | master dataset §47 (= master §46 hub position − Gazebo/CAD CG, §6 of this document — arithmetic reproduced and confirmed below) | CONFIRMED |
| Prop hub, CG-relative offset — right | ΔX≈+0.1268, ΔY≈−0.3000, ΔZ≈+0.0271 | m | Gazebo/CAD, CG-relative | master dataset §47 | CONFIRMED |
| Left motor position (coordinate; physical reference — distinct from the raw mesh bounding-box center, see note below) | (0.2623, +0.3000, 0.1269) | m | Gazebo/CAD (FLU) | master dataset §46 | CONFIRMED (master dataset value) — differs from this document's independent `left_motor.stl` bounding-box center (§19: 0.269577, 0.299974, 0.126975 m) by ≈7.28 mm in X; Y/Z agree to ≤0.03 mm; see note below, not silently unified |
| Right motor position (coordinate) | (0.2623, −0.3000, 0.1269) | m | Gazebo/CAD (FLU) | master dataset §46 | CONFIRMED (same ΔX note as left) |
| Motor thrust-line orientation/axis | Nominal +X; measured prop-normal unit vector (+0.999996, +0.000018, −0.002668), i.e. ≈0.153° off pure +X in the vertical plane | — (unit vector) | Gazebo/CAD (FLU) | master dataset §30, §48 | CONFIRMED (master dataset, directly measured/stated vector) — a materially stronger form of evidence than this document's own §19/§20 elongation/coaxiality shape evidence, which remains `THRUST_AXIS_REQUIRES_CONFIRMATION` as independent corroboration only, not superseded |
| Motor mount / firewall offset | — | m | Gazebo/CAD (FLU) | none found | DATA_REQUIRED |

"Forward of CG" is a qualitative direction; the numeric hub/motor coordinates above (ΔX≈+0.1268 m forward of the Gazebo/CAD CG, master dataset §47) now give it a quantitative value usable as an SDF force-application-point offset.

**Physical hub/shaft reference vs. raw mesh bounding-box center — explicit distinction (2026-08-21, master-dataset synchronization pass, per task instruction not to silently conflate the two):**

- **Prop hub:** master dataset value (0.2951, ±0.3000, 0.1271 m) vs. this document's independently-computed `left_pervane.stl`/`right_pervane.stl` bounding-box center (0.295084, ±0.300007/∓0.299993, 0.127073 m, §20, `STL_SCALE_TO_SI` applied) — agreement to ≤0.03 mm on every axis. For a thin, rotation-symmetric propeller disc, the bounding-box center and the physical hub/rotation-axis point are expected to coincide closely, so this tight agreement is unsurprising and mutually corroborating, not a coincidence requiring explanation.
- **Motor center:** master dataset value (0.2623, ±0.3000, 0.1269 m) vs. this document's independently-computed `left_motor.stl`/`right_motor.stl` bounding-box center (0.269577, ±0.299974/∓0.300026, 0.126975 m, §19) — Y and Z agree to ≤0.03 mm, but **X differs by ≈7.28 mm** (0.2623 m vs. 0.269577 m). This is flagged explicitly rather than silently resolved: the two figures plausibly measure *different things* — a full-mesh bounding-box center necessarily includes whatever the mesh's entire 66.485 mm X-extent covers (§19; possibly a mount adapter, shaft stub, or other protrusion beyond the motor "can" itself), whereas the master dataset's "motor center" more likely refers to the motor body's own effective center (e.g., relevant to a rotor-inertia contribution). No CAD part-tree or component-boundary definition is available in this repository to confirm which is "correct" for which purpose — **both values are recorded here, neither silently discarded**, and the ≈7.3 mm difference should be kept in mind by any future consumer of "motor position" data (it is a non-trivial fraction of the ≈27.1 mm CG-to-hub vertical offset used in the pitch-moment estimate at master dataset §59).
- **For thrust force application specifically** (this agent's ownership: force/moment application *locations*, not the force model itself): the **prop hub** coordinate is the relevant point, per master dataset §47's own explicit instruction ("Thrust CG'ye uygulanmamalı, gerçek hub/thrust-line noktasında uygulanmalı" — thrust must not be applied at the CG, but at the real hub/thrust-line point) — and it is the more tightly corroborated of the two figures above. This is provided to `propulsion` as the force-application coordinate; modeling the thrust force magnitude/direction physics itself remains `propulsion`'s domain, not this document's.

---

## 8. Coordinate and Reference Definitions

Two distinct, non-interchangeable coordinate/reference systems are referenced across this project's data. Both are used elsewhere in this repository (CG values, aerodynamic reference point), so their definitions are recorded here for geometry consistency even though the CG magnitudes themselves live in `MASS_PROPERTIES.md`.

### 8.1 Gazebo/CAD reference frame

| Property | Value | Provenance | Status |
|---|---|---|---|
| Axis convention | FLU (+X forward, +Y left, +Z up) | project owner; `CLAUDE.md` | CONFIRMED |
| Physical origin location on airframe | — | none found | DATA_REQUIRED |
| Units | meters (SI) | project owner; `CLAUDE.md` engineering rules | CONFIRMED |

### 8.2 XFLR5 reference frame

| Property | Value | Provenance | Status |
|---|---|---|---|
| Axis convention | — | none found | DATA_REQUIRED |
| Physical origin location on airframe | — | none found | DATA_REQUIRED |
| Units | Not stated in repo (assumed SI by XFLR5 convention only — not confirmed for this project) | — | DATA_REQUIRED |

### 8.3 Conversion between Gazebo/CAD and XFLR5 frames

**Update, 2026-08-21 (master-dataset synchronization pass):** a specific, derived X/Z transform is now documented below, sourced from master dataset §2, §6, §8, §12, §20, and cross-validated against two independent published coordinate pairs. This supersedes the prior blanket "no conversion performed/possible" position **for the specific transform stated below only** — it does not establish general-purpose full knowledge of either frame's origin/axis convention. See "What this does NOT establish" below for the explicit limits.

**Derivation**

1. Master dataset §12 records that the XFLR5 Plane Editor was configured with `Main Wing X = 0`, `Main Wing Z = 0`, `Tilt = 0 deg` — i.e., by construction, the XFLR5 coordinate origin was placed at the main-wing-root leading-edge (LE) point, with no additional plane-level rotation (the wing's own +4°/0° root/tip incidence is carried in the per-section twist values instead, per master dataset §12's own note).
2. Master dataset §8 records that same main-wing-root-LE point's location in STL/Gazebo-CAD coordinates: X ≈ +231.96 mm, Z ≈ +121.03 mm (Y = 0, root/centerline station).
3. Master dataset §2 records the sign relationship between the frames' X axes: "XFLR5 +X yönü STL +X yönüne ters / kuyruğa doğru kabul edildi" — XFLR5's +X is taken as reversed relative to STL's +X, pointing toward the tail. No reversal is stated for Z (treated as same-sense in both frames); no statement at all is made about Y.

Combining these three facts gives the candidate transform (STL/Gazebo-CAD meters → XFLR5 meters), using the main-wing-root-LE point as the shared origin and a 180°-about-Z-ish (X-reversing) relationship between the frames:

```
XFLR5_X = -(STL_X - 0.23196)  =  0.23196 - STL_X
XFLR5_Z =   STL_Z - 0.12103
XFLR5_Y =   STL_Y                (untested — see caveats below; not asserted as confirmed)
```

**Cross-validation 1 — Gazebo/CAD CG → XFLR5 CG**

Using the Gazebo/CAD CG (`MASS_PROPERTIES.md` §3.1: STL_X = 0.168309 m, STL_Z = 0.100000 m):

```
XFLR5_X = 0.23196 - 0.168309 = 0.063651 m   vs. documented XFLR5 CG X = 0.0637 m   → Δ = 0.00005 m
XFLR5_Z = 0.100000 - 0.12103 = -0.02103 m   vs. documented XFLR5 CG Z = -0.0210 m  → Δ = 0.00003 m
```

Both components reproduce the documented XFLR5 CG (`MASS_PROPERTIES.md` §3.2; master dataset §6, §8) to within 0.05 mm — i.e., to the precision the master dataset itself reports these figures. Note: this is essentially the same arithmetic master dataset §8 performs to justify its own XFLR5 CG entry, so this check confirms the transform is *internally consistent* with how the project's own XFLR5 CG figure was produced — it is corroborating, but it is not an independent external measurement of the CG.

**Cross-validation 2 — Horizontal-tail root LE, STL → XFLR5 (an independent point, different surface, not the CG)**

Using the horizontal-tail root LE in STL/Gazebo-CAD coordinates (master dataset §20): STL_X = -0.32137 m, STL_Z = 0.07952 m:

```
XFLR5_X = 0.23196 - (-0.32137) = 0.55333 m   vs. documented XFLR5 Elevator X = 0.5533 m   → Δ = 0.00003 m
XFLR5_Z =  0.07952 - 0.12103   = -0.04151 m  vs. documented XFLR5 Elevator Z = -0.0415 m  → Δ = 0.00001 m
```

This is a genuinely independent check — a different physical point, on a different lifting surface, not used to define the CG — and it closes to within 0.01–0.03 mm.

**Conclusion:** the candidate transform is corroborated by two independent check points, both closing to sub-0.1 mm — the precision the master dataset itself reports. It is documented here as `DERIVED` (derivation shown, both cross-validation points cited), replacing the prior blanket `DATA_REQUIRED` for this specific X/Z relationship.

| Parameter | Value | Provenance | Status |
|---|---|---|---|
| Gazebo/CAD → XFLR5 transform, X | `XFLR5_X = 0.23196 - STL_X` (meters) | master dataset §2, §6, §8, §12, §20 (derivation + 2 cross-validation points, this section) | `DERIVED` |
| Gazebo/CAD → XFLR5 transform, Z | `XFLR5_Z = STL_Z - 0.12103` (meters) | master dataset §2, §6, §8, §12, §20 | `DERIVED` |
| Gazebo/CAD → XFLR5 transform, Y | Not derived — carried as `XFLR5_Y = STL_Y` by unverified assumption only (see caveat 1 below) | — | `DATA_REQUIRED` |
| Rotation (roll/pitch) between the two frames beyond the stated X-axis sign reversal | Not stated by the master dataset and not derived here | — | `DATA_REQUIRED` |

**What this transform does NOT establish (stated explicitly, not glossed over):**

1. **Y-axis behavior is untested.** All three points used above (Gazebo/CAD CG, XFLR5 CG, horizontal-tail root LE) have Y = 0. `XFLR5_Y = STL_Y` is carried here as a plausible, unverified consequence of the model (Y being the shared left/right axis in a conventional XFLR5-and-FLU setup), **not** a validated result. No point with nonzero Y (a wingtip, an aileron/elevator/rudder hinge station, a motor hub) should be run through this transform until Y-axis behavior is independently confirmed with a Y≠0 cross-validation point.
2. **No rotational/bank alignment beyond the stated X-sign reversal is confirmed.** Master dataset §2 states only that XFLR5's +X is reversed relative to STL's +X; it says nothing about roll (bank) alignment between the frames' Y/Z axes beyond an implied "Z appears same-sense" reading of the dataset's own wording — that Z claim is not independently re-derived from a third data point in this document (both cross-validation points above use the same X-reversed/Z-same-sense model; neither tests an alternative rotation hypothesis against it).
3. **The physical/CAD meaning of the shared origin point (main-wing-root LE, STL X=0.23196 m, Z=0.12103 m) is still unconfirmed.** It is accepted here as *a* valid shared reference point because it reproduces two independently documented values, not because its identity as a named CAD datum (e.g. a specific wing-rib/monocoque station) has been verified against a CAD source. This point is also explicitly **not** the same thing as the raw mesh-file (0,0,0) origin discussed in §13.2, whose physical meaning remains separately `DATA_REQUIRED` — the two must not be conflated.
4. **This does not fully resolve `MASS_PROPERTIES.md` §4's remaining `DATA_REQUIRED` items** for the XFLR5 frame's general axis-convention/physical-origin documentation — it resolves the specific numeric X/Z relationship between the two documented CG/reference-point values for this aircraft, using the evidence available, not a general CAD-level characterization of the XFLR5 frame.

Full cross-validation arithmetic and this subsection's relationship to the STL-scale question (§13.1) are also indexed at §32.3. See `MASS_PROPERTIES.md` §3.5 for the mass-properties-side restatement of this same derived transform.

---

## 9. Provenance Table

Consolidated provenance for every value used in this document.

| Source | What it provides | Used for |
|---|---|---|
| Project owner, direct conversation, 2026-08-21 | Aircraft identity, simulator target, wingspan, wing area, MTOW rating, cruise-speed reference, body-frame convention, twin front-puller/L-R motor arrangement, motor-forward-of-CG fact, battery-central fact, ESC-in-wings fact | §1, §2, §3, §5, §7 |
| `CLAUDE.md` (repository root) | Restates the above owner-provided values; project engineering rules | Cross-check for all sections |
| `docs/source_of_truth/README.md` | Restates wingspan, wing area, mass; explicitly flags CG origin/reference-frame definitions as `DATA_REQUIRED` | Cross-check for §3, §8 |
| Full-repository search (this task) | Confirms absence of CAD/SDF/mesh/XFLR5 files and absence of any geometry coordinate data | Basis for all `DATA_REQUIRED` entries in §4–§8 |

**Source hierarchy** (per role definition — used only to *characterize* a conflict if one is found, never to silently pick a winner):
1. Confirmed current FALCON V2 CAD-derived data
2. Confirmed measured aircraft data
3. Manufacturer data
4. Current FALCON V2 engineering calculations
5. Current FALCON V2 XFLR5/XFOIL data
6. Older project notes

No case in this document required applying this hierarchy, because no conflicting values were found (see §10).

---

## 10. Conflicts

**No conflicts found.** Every geometry value obtained from the project owner (Step 2 of this task) was cross-checked against `CLAUDE.md` and `docs/source_of_truth/README.md`, the only other places these values are recorded in the repository:

| Parameter | Project owner (this task) | `CLAUDE.md` | `docs/source_of_truth/README.md` | Result |
|---|---|---|---|---|
| Wingspan | 2.093 m | 2.093 m | 2.093 m | Match |
| Wing area | 0.4514 m² | 0.4514 m² | 0.4514 m² | Match |
| Total aircraft mass | 6.000 kg | 6.000 kg | 6.000 kg | Match |
| Body-frame convention | FLU | FLU | (not restated) | Match |

Manufacturer MTOW (7 kg) and manufacturer cruise speed (12.5–18.1 m/s) are new in this task and were not previously recorded anywhere in the repository — there is nothing to conflict with. No CAD/SDF file exists to compare against manufacturer geometry, so the "documented discrepancy" check required by policy (§4) could not surface any discrepancy — there was nothing to check.

**Update, 2026-08-21 (this task):** 12 STL mesh files were added to `model/meshes/` (see §4). This still does not enable the manufacturer-wingspan/wing-area discrepancy check described below — no dimension has been extracted from the STL mesh data (no CAD/mesh-processing tool was used; see §4.1). This check remains open. Two raw file-level observations were made during mesh cataloging, neither evaluated as a geometric conflict because no geometric data was extracted:
- `left_wing.stl` (181,330 facets) and `right_wing.stl` (181,328 facets) differ by 2 triangles / 100 bytes in the raw binary file. This is reported as an observation only — it is not resolved into a "match" or "conflict" determination in this document.
- Whether `rudder.stl` / the elevator STLs bundle fixed structure with the movable control surface is an open question (§4.3), not a conflict — no competing values exist to conflict, just unresolved scope of what the mesh contains.

This section will require updating once CAD-dimension-extraction is performed (STL/mesh measurement, or native CAD file) and compared against the manufacturer wingspan/wing-area references.

---

## 11. Missing Data

**Update, 2026-08-21 (this task):** STL mesh files (item, formerly "STL/mesh files, visual and collision") are **no longer missing as files** — 12 binary STL files are confirmed present in `model/meshes/`, see §4.2. This line item is removed from the list below. It is **not** replaced with "resolved" — the dimensional/placement data that would be *extracted from* those files remains fully `DATA_REQUIRED` and is tracked under items 5–30 below exactly as before. "CAD source files, any format" is retained below because only STL exports were found — no native/editable CAD file (STEP/IGES/SLDPRT/F3D/etc.) is present.

Full list of `DATA_REQUIRED` items from this document, for tracking. **Update, 2026-08-21 (master-dataset synchronization pass): items resolved this pass are marked `[RESOLVED]` below with a pointer to the new content; they are retained in this numbered list, not deleted, so the tracking history stays intact.**

1. FLU body-frame physical origin definition on the airframe (§2)
2. CAD source files, native/editable, any format — STEP/IGES/SLDPRT/F3D/etc. (§4) — note: STL mesh *exports* are present (§4.2), this item concerns native CAD source only
3. SDF model/link/joint files (§4)
4. Fuselage reference origin coordinate (§5)
5. Wing root coordinate — left (§5) — `[PARTIALLY RESOLVED]` a main-wing-root-LE *reference point* is now documented (§5, §8.3: 0.23196, 0, 0.12103 m), but this is a different concept from the mesh's own Y≈0.080 m boundary (§15); neither is a named CAD station, so this item is not fully closed
6. Wing root coordinate — right (§5) — same partial-resolution note as item 5
7. Horizontal-tail placement coordinate (§5) — `[RESOLVED]` both Gazebo/CAD-frame (−0.32137, 0, 0.07952 m) and XFLR5-frame (0.5533, 0, −0.0415 m) values now documented, master dataset §19–§20
8. Vertical-tail placement coordinate (§5) — `[PARTIALLY RESOLVED]` XFLR5-frame value now documented (0.5537, 0, −0.0010 m, master dataset §28); the Gazebo/CAD-frame equivalent is still `DATA_REQUIRED` (no independent STL root-LE point stated for the vertical tail in the master dataset)
9. Wing incidence angle (§5) — `[RESOLVED]` root +4°, tip 0°, washout 4°, master dataset §3
10. Tail incidence angle (§5) — still `DATA_REQUIRED`; not stated for the horizontal or vertical tail anywhere in the master dataset
11. Dihedral angle (§5) — `[RESOLVED]` 0.5°, master dataset §3
12. Wing sweep angle (§5) — `[RESOLVED]` 3°, master dataset §3
13. Visual geometry definitions for all components (§5) — mesh files exist (§4.2) but are not yet wired into any SDF visual element; still `DATA_REQUIRED` in the SDF-ready sense
14. Collision geometry definitions for all components (§5) — still `DATA_REQUIRED`
15. Battery center coordinate (§5) — `[RESOLVED, main battery]` (0.300631, 0, 0.038547) m, master dataset §7; see `MASS_PROPERTIES.md` §6.1 for full detail. Secondary (3S) battery position remains `DATA_REQUIRED` — explicitly not invented, per task instruction
16. ESC center coordinate(s) — left/right wing (§5) — still `DATA_REQUIRED`; master dataset §42 confirms only that ESCs are wing-mounted and gives mass (~80 g each), not position
17. Aileron hinge position — left (§6) — `[RESOLVED, as `HINGE_GEOMETRY_READY`]` span + per-station x/c now documented, master dataset §33–§34; SDF-axis fit/sign-test still pending
18. Aileron hinge position — right (§6) — same as item 17 (mirror)
19. Aileron hinge axis (§6) — still not a fitted joint-axis vector; `HINGE_GEOMETRY_READY` (real data, translation pending), not a blanket `DATA_REQUIRED`
20. Elevator hinge position (§6) — `[RESOLVED, as `HINGE_GEOMETRY_READY`]` global hinge X + span + per-station x/c, master dataset §21
21. Elevator hinge axis (§6) — `HINGE_GEOMETRY_READY` (real data, axis-fit/sign-test pending)
22. Rudder hinge position (§6) — `[RESOLVED, as `HINGE_GEOMETRY_READY`]` per-station x/c, master dataset §29
23. Rudder hinge axis (§6) — `HINGE_GEOMETRY_READY` (real data, axis-fit/sign-test pending)
24. Aileron/elevator/rudder neutral poses (§6) — still `DATA_REQUIRED`; nothing in the master dataset defines a neutral pose independent of the as-exported mesh pose
25. Control-surface deflection limits (§6) — `[RESOLVED]` mechanical initial ≈±30° (master dataset §3) vs. aero-derivative-validated linear range ≈±10° (master dataset §65) — two distinct, non-conflicting quantities, both now documented
26. Left motor position coordinate (§7) — `[RESOLVED]` prop hub (0.2951, 0.3000, 0.1271 m) and motor center (0.2623, 0.3000, 0.1269 m), master dataset §46 — see §7's explicit note on the ≈7.3 mm motor-center-vs-bounding-box-center discrepancy
27. Right motor position coordinate (§7) — `[RESOLVED]` mirror of item 26
28. Motor thrust-line orientation/axis (§7) — `[RESOLVED]` nominal +X with measured normal vector (+0.999996, +0.000018, −0.002668), ≈0.153° offset, master dataset §30/§48
29. Motor mount / firewall offset (§7) — still `DATA_REQUIRED`; no dedicated mount/pylon/strut mesh or dimension exists anywhere in the repository or master dataset
30. XFLR5 reference-frame axis convention (§8.2) — `[PARTIALLY RESOLVED]` X-axis is reversed relative to Gazebo/CAD +X, Z is same-sense (master dataset §2); Y-axis behavior is untested (§8.3 caveat 1) and roll/bank alignment is unconfirmed (§8.3 caveat 2) — not a full axis-convention characterization
31. XFLR5 reference-frame physical origin (§8.2) — `[PARTIALLY RESOLVED]` identified as the main-wing-root-LE point (§8.3), reproduced by 2 independent cross-validation points; not independently CAD-confirmed as a named datum (§8.3 caveat 3)
32. Gazebo/CAD reference-frame physical origin (§8.1, duplicate of item 1 — same underlying gap) — unchanged, still `DATA_REQUIRED` as a named CAD datum
33. Gazebo/CAD ↔ XFLR5 transform (§8.3) — `[RESOLVED for X/Z]` `XFLR5_X = 0.23196 - STL_X`, `XFLR5_Z = STL_Z - 0.12103`, derived and cross-validated to sub-0.1 mm against 2 independent points (§8.3). Y-axis component and any rotation beyond the stated X-sign reversal remain `DATA_REQUIRED` (§8.3 caveats 1–2)

**Open question, previously tracked here (see §4.3) — now resolved:** whether `rudder.stl` includes the fixed vertical-fin structure or only the movable rudder surface, and whether `left_elevator.stl`/`right_elevator.stl` include the fixed horizontal-stabilizer structure or only the movable elevator surface. **`[RESOLVED]` 2026-08-21, master-dataset synchronization pass — see §26.3: concluded movable-surface-only for all three meshes, with `body.stl` carrying the fixed structure.** Reasoning, evidence, and the explicitly-acknowledged residual (small, non-zero) ambiguity are documented in full at §26.3 — not asserted here without justification.

None of the still-open items above are estimated, guessed, or filled with placeholder numeric values anywhere in this document (§1–§11, §32).

---

# Deep Mesh Geometric Analysis (2026-08-21, second follow-up pass)

**Scope of this part of the document (§12–§30):** read-only geometric analysis of the 12 binary STL mesh files at `model/meshes/`. Method for every value: direct parse of the binary STL vertex data (80-byte header skipped, little-endian uint32 facet count read and cross-checked against `(filesize − 84) / 50`, then all vertex triples read via `numpy.frombuffer`). No mesh file was opened for editing, modified, moved, renamed, or converted. No SDF, joint, or inertia value is created in this part of the document. All coordinate values below are **raw mesh-file coordinates as extracted** — see §13.1 for why these are evidenced-but-not-CAD-confirmed to be millimeters, and never silently treated as meters anywhere in this document.

**Distinctions maintained throughout §12–§30 (per task instruction, stated explicitly rather than assumed away):**
- A bounding-box center is a geometric center of the mesh's vertices — it is **not** a mass center / CG. It is never substituted for the Gazebo/CAD CG (0.168309, 0, 0.100000 m) or the XFLR5 CG (0.0637, 0, −0.0210 m) anywhere below.
- A bounding-box edge or slice boundary is **not** automatically a hinge line. Candidates are labeled `HINGE_REQUIRES_CONFIRMATION` unless noted otherwise.
- STL triangle surface area is **not** computed or used anywhere below as a stand-in for aerodynamic planform/reference area.
- A motor mesh's long bounding-box axis is **not** automatically its thrust axis; the corroborating evidence for treating it as one is stated explicitly and the item is still tagged `THRUST_AXIS_REQUIRES_CONFIRMATION`.

---

## 12. Mesh Inventory (this pass)

This is a dimension-analysis-focused inventory. For full file-existence provenance (git status, header signature, size-formula cross-check), see §4.2 — not repeated here.

| File | Facet count (header) | Facet count check `(size−84)/50` | Format |
|---|---|---|---|
| body.stl | 919,765 | 919,765.0 — match | Binary STL |
| left_wing.stl | 181,330 | 181,330.0 — match | Binary STL |
| right_wing.stl | 181,328 | 181,328.0 — match | Binary STL |
| left_aileron.stl | 28,786 | 28,786.0 — match | Binary STL |
| right_aileron.stl | 28,786 | 28,786.0 — match | Binary STL |
| left_elevator.stl | 31,462 | 31,462.0 — match | Binary STL |
| right_elevator.stl | 31,462 | 31,462.0 — match | Binary STL |
| rudder.stl | 34,857 | 34,857.0 — match | Binary STL |
| left_motor.stl | 697,134 | 697,134.0 — match | Binary STL |
| right_motor.stl | 697,134 | 697,134.0 — match | Binary STL |
| left_pervane.stl | 79,088 | 79,088.0 — match | Binary STL |
| right_pervane.stl | 79,088 | 79,088.0 — match | Binary STL |

**Format confirmation method:** each file's 80-byte header was checked; none begin with the ASCII STL literal `"solid"` in a way that also satisfies an ASCII-file size profile — all 12 satisfy the binary-STL size formula `84 + 50 × facet_count = file_size` exactly, confirming binary format with no truncation. `Source: model/meshes/*.stl`. `Method: STL header parse + file-size cross-check`. `Status: CONFIRMED_FROM_MESH` (format only; not a dimension value).

All 12 files were successfully parsed and are included in every table below — **all 12 mesh files were analyzed**, not a subset.

---

## 13. Mesh Coordinate Extents

Vertex-wise axis-aligned bounding box for every mesh (i.e., computed from all 3 vertices of every facet, not just one vertex per facet). Units as extracted from the file (see §13.1 for the unit question). `Source: model/meshes/<file>.stl` for every row. `Method: STL vertex bounds`. `Status: DERIVED_FROM_MESH`.

| File | min X | max X | min Y | max Y | min Z | max Z | size X | size Y | size Z | center (X,Y,Z) |
|---|---|---|---|---|---|---|---|---|---|---|
| body.stl | −526.880 | 526.880 | −280.000 | 280.000 | 0.000 | 330.480 | 1053.761 | 559.999 | 330.480 | (0.000, 0.000, 165.240) |
| left_wing.stl | −27.512 | 238.998 | 80.000 | 1052.027 | 103.541 | 150.497 | 266.510 | 972.027 | 46.956 | (105.743, 566.014, 127.019) |
| right_wing.stl | −27.512 | 238.998 | −1052.027 | −80.000 | 103.541 | 150.497 | 266.510 | 972.027 | 46.956 | (105.743, −566.014, 127.019) |
| left_aileron.stl | −23.578 | 37.541 | 310.374 | 789.740 | 107.686 | 128.639 | 61.119 | 479.366 | 20.953 | (6.981, 550.057, 118.163) |
| right_aileron.stl | −23.578 | 37.541 | −789.740 | −310.374 | 107.686 | 128.639 | 61.119 | 479.366 | 20.953 | (6.981, −550.057, 118.163) |
| left_elevator.stl | −517.657 | −472.684 | 50.600 | 240.000 | 78.555 | 87.621 | 44.973 | 189.400 | 9.066 | (−495.170, 145.300, 83.088) |
| right_elevator.stl | −517.657 | −472.684 | −240.000 | −50.600 | 78.555 | 87.621 | 44.973 | 189.400 | 9.066 | (−495.170, −145.300, 83.088) |
| rudder.stl | −517.999 | −472.684 | −4.456 | 4.455 | 130.250 | 299.750 | 45.316 | 8.911 | 169.500 | (−495.341, ~0.000, 215.000) |
| left_motor.stl | 236.335 | 302.819 | 280.862 | 319.087 | 107.856 | 146.093 | 66.485 | 38.225 | 38.236 | (269.577, 299.974, 126.975) |
| right_motor.stl | 236.335 | 302.819 | −319.138 | −280.913 | 107.856 | 146.093 | 66.485 | 38.225 | 38.236 | (269.577, −300.026, 126.975) |
| left_pervane.stl | 286.617 | 303.550 | 163.298 | 436.717 | 116.485 | 137.661 | 16.932 | 273.419 | 21.176 | (295.084, 300.007, 127.073) |
| right_pervane.stl | 286.617 | 303.550 | −436.702 | −163.283 | 116.485 | 137.661 | 16.932 | 273.419 | 21.176 | (295.084, −299.993, 127.073) |

### 13.1 Unit evidence

No CAD export log, unit metadata field, or project-owner statement confirms the coordinate unit of these STL files. The 80-byte binary header text (`"STLB ATF 15.8.0.0 COLOR=..."`) does not encode a unit. The following is an evidence-based observation, not an applied conversion:

- Taking the raw numbers **as meters**: `body.stl` would be 1053.8 m long, 560.0 m wide, 330.5 m tall — physically impossible for an aircraft with a manufacturer-stated 2.093 m wingspan. This interpretation is rejected on physical-implausibility grounds.
- Taking the raw numbers **as millimeters**: `body.stl` is 1.054 m long × 0.560 m wide × 0.330 m tall, and the wing tip-to-tip span (§21) computes to 2.104 m — closely matching the manufacturer's 2.093 m wingspan reference (§21, `GEOMETRIC_CHECK`, +0.53%). This interpretation is physically plausible and corroborated by an independent manufacturer reference.

`Source: model/meshes/*.stl` (magnitude comparison), manufacturer wingspan reference (`CLAUDE.md`, §3 of this document). `Method: dimensional comparison`. **Status: `DATA_REQUIRED` for the CAD-confirmed unit.** The millimeter interpretation is strongly evidenced but is not treated as authoritatively confirmed anywhere in this document — every subsequent section that performs a millimeter→meter comparison states this explicitly as "using the millimeter interpretation" rather than asserting it as fact. No silent conversion is applied to §5/§6/§7 (which remain `DATA_REQUIRED` for SDF-ready values), and this section does not overwrite or reinterpret the CONFIRMED Gazebo/CAD CG or manufacturer values elsewhere in this document, which remain in meters as originally provided.

**Update, 2026-08-21 (master-dataset synchronization pass):** the millimeter interpretation is formalized as a named, citable constant for downstream mesh-loading/SDF work:

```
STL_SCALE_TO_SI = 0.001   (dimensionless multiplier, mesh-file units -> meters)
```

**Status: `DERIVED_WITH_STRONG_EVIDENCE`** — deliberately **not** `CAD_CONFIRMED`/`CONFIRMED`. No CAD export log, STL unit-metadata field, or project-owner statement of "the mesh is authored in millimeters" exists anywhere in the repository or the master dataset — that specific piece of evidence is still absent. What has changed is the *number of independent convergent numeric checks* that are consistent only with the millimeter interpretation: (1) the physical-implausibility rejection of the meters interpretation (this section, original); (2) the wingspan cross-check, §21.1, +0.53% agreement; (3) the CG-based coordinate-transform cross-check, §8.3/§32.3, closing to ≤0.05 mm; (4) the horizontal-tail-placement coordinate-transform cross-check, §8.3/§32.3, closing to ≤0.03 mm; (5) the full §11-vs-mesh bounding-dimension cross-check, §32.10, matching on all 7 components to the master dataset's own reported precision. Five independent numeric agreements at sub-percent/sub-millimeter precision is strong convergent evidence, but convergent evidence from arithmetic checks is still not the same class of evidence as a CAD tool's own unit metadata — hence `DERIVED_WITH_STRONG_EVIDENCE` rather than `CONFIRMED`. `STL_SCALE_TO_SI = 0.001` is the constant this document uses, by name, for every mesh-unit→meter conversion performed from this pass onward (see §32); it must be applied explicitly (never silently/implicitly) by any implementation code that loads these meshes into SDF, and the underlying unit remains formally `DATA_REQUIRED` in the strict CAD-metadata sense. `Source: master dataset §4 (structural/manual section, general engineering-judgment context); this document §13.1 (original evidence), §21.1, §8.3, §32.3, §32.10 (the four cross-validation points enumerated above)`.

### 13.2 Origin-placement evidence

- `body.stl`: X range is symmetric about 0 (−526.880 / +526.880) and Y range is symmetric about 0 (−280.000 / +280.000) to within ~0.0004 mm (float32 rounding noise). Z range starts at 0.000 and only extends positive (0 to 330.480). This is evidence that the mesh-file origin (0,0,0) sits at the body's geometric X-center and Y-center (consistent with a centerline/symmetry-plane placement), but at the **bottom** of the body in Z (Z=0 is the lowest vertex of the entire mesh), not the vertical center. `Source: model/meshes/body.stl`. `Method: STL vertex bounds`. `Status: DERIVED_FROM_MESH` (placement pattern only — the *physical* meaning of this origin, e.g. "ground line," "firewall," or a specific CAD reference station, is **not** determinable from geometry alone and is `DATA_REQUIRED`).
- All other 11 meshes (wing, aileron, elevator, rudder, motor, pervane) are **not** centered near their own local (0,0,0) — e.g. `left_wing.stl` sits entirely between Y=80 and Y=1052, nowhere near Y=0. This is evidence that all 12 meshes share **one common assembly coordinate system** (each part pre-positioned as it sits in the assembled aircraft) rather than each having an independent per-part origin. `Source: model/meshes/*.stl`. `Method: STL vertex bounds comparison across files`. `Status: DERIVED_FROM_MESH`. This is a necessary (not sufficient) condition for the 12 meshes being placeable directly into one SDF model without individual per-link re-registration — but it does **not** by itself confirm what physical point that shared origin represents on the real airframe (still `DATA_REQUIRED`, consistent with §2 of this document).

---

## 14. Body Geometry

`Source: model/meshes/body.stl` for all values in this section. `Method: STL vertex bounds / argmax-argmin point query`. `Status: DERIVED_FROM_MESH` unless noted.

| Quantity | Value (raw mesh units) | Notes |
|---|---|---|
| Overall size | 1053.761 × 559.999 × 330.480 | X × Y × Z |
| Nose-most point (max X) | (526.880, −11.562, 53.857) | Single vertex at max X; not exactly on Y=0 — see note below |
| Tail-most point (min X) | (−526.880, −5.719, 84.284) | Single vertex at min X |
| Left-most point (max Y) | (−498.262, 279.9996, 85.136) | Occurs near the tail X-region (X≈−498), not amidships — see §14.1 |
| Right-most point (min Y) | (−498.262, −279.9996, 85.136) | Mirrors left-most point exactly in X and Z |
| Lowest point (min Z) | (193.709, 32.500, ~0.000) | Located under the wing/motor X-region, near centerline in Y |
| Highest point (max Z) | (−501.825, ~0.000, 330.480) | Located in the tail X-region, at Y≈0 (centerline) — see §14.1 / §17 / §26 |
| Bounding-box center | (0.000, 0.000, 165.240) | Geometric center of vertex bounds — **not** the CG (§3.1/§3.2 of this document use separately-documented CG values) |
| Possible symmetry plane | Y = 0 | X range and Y range both symmetric about 0 to ≤0.0004 mm; this is the aircraft's evidenced left/right symmetry plane, consistent with FLU's Y-axis convention |

**Nose/tail identification reasoning:** `left_elevator.stl`/`right_elevator.stl`/`rudder.stl` (the empennage control surfaces) cluster tightly around X ≈ −473 to −518, immediately adjacent to the body's X-minimum extreme (−526.880). This places the tail unambiguously at the −X end of the body and, by elimination, the nose at the +X end. `Source: model/meshes/body.stl, left_elevator.stl, right_elevator.stl, rudder.stl`. `Method: STL vertex bounds comparison`. `Status: DERIVED_FROM_MESH`.

### 14.1 Body-mesh observations bearing on tail structure (see §17, §26 for full analysis)

- The body's **widest point** (Y = ±280.000, the global max/min Y of the *entire* mesh) occurs at X≈−498 — inside the tail X-region, not amidships, and at the same Z-height band (Z≈85) as the elevator meshes (Z 78.6–87.6). This is a raw geometric observation; whether it represents a fixed horizontal-stabilizer structure built into `body.stl` is analyzed in §26.
- The body's **tallest point** (Z = 330.480, the global max Z of the entire mesh) occurs at X≈−502, Y≈0 — inside the tail X-region, at centerline. This is taller than `rudder.stl`'s own max Z (299.750). Analyzed in §26.

Both observations are `DERIVED_FROM_MESH` (vertex bounds + argmax point query); their *interpretation* (fixed tail-surface structure bundled into `body.stl`) was `COMPONENT_SCOPE_REQUIRES_CONFIRMATION` per §26 at the time this section was written — this section reports the raw coordinates only. **Resolved 2026-08-21, master-dataset synchronization pass — see §26.3.**

---

## 15. Wing Geometry

`Source: model/meshes/left_wing.stl, model/meshes/right_wing.stl`. `Method: STL vertex bounds / slice-based comparison`. `Status: DERIVED_FROM_MESH` unless noted.

| Quantity | left_wing.stl | right_wing.stl |
|---|---|---|
| Chordwise extent (X) | −27.512 to 238.998 (266.510) | −27.512 to 238.998 (266.510) |
| Spanwise extent (Y) | 80.000 to 1052.027 (972.027) | −1052.027 to −80.000 (972.027) |
| Thickness extent (Z) | 103.541 to 150.497 (46.956) | 103.541 to 150.497 (46.956) |
| Bounding-box center | (105.743, 566.014, 127.019) | (105.743, −566.014, 127.019) |

**Root vs. tip:** the Y=80.000 end of `left_wing.stl` sits closest to the body/centerline; the Y=1052.027 end is farthest away. Cross-checked against body geometry: at the wing's own X/Z footprint, `body.stl`'s local outer surface reaches to Y=94.649 (see §17). Since the wing mesh begins at Y=80.000 — *inboard* of the body's local outer surface at Y=94.649 by 14.649 mm — the wing root is evidenced to sit embedded in/overlapping the fuselage shell (a normal wing-carry-through/fairing relationship, not a floating gap). Therefore: **root = Y≈80 end (closer to body), tip = Y≈1052 end (farther from body).** `Status: DERIVED_FROM_MESH`.

**Approximate mesh-to-mesh wingspan:** computed tip-to-tip as `left_wing.stl` max Y minus `right_wing.stl` min Y = 1052.027 − (−1052.027) = 2104.054 (raw mesh units). Method chosen: tip-to-tip (both tip Y-extremes are well-defined single mesh values; a "body centerline to one tip ×2" method was considered but rejected because the wing root is embedded in the fuselage and does not start at Y=0, so doubling a single-side span would not equal the tip-to-tip value — see §21 for the full manufacturer comparison using the millimeter interpretation). `Source: model/meshes/left_wing.stl, right_wing.stl`. `Method: STL vertex bounds`. `Status: DERIVED_FROM_MESH` (raw mesh units); comparison against manufacturer reference is in §21 tagged `GEOMETRIC_CHECK`.

**Chordwise (taper) profile across span:** a 10-band spanwise slice of `left_wing.stl`'s own vertices shows local X-range (apparent chord) decreasing outboard from ≈266 mm near the root toward ≈184 mm near the tip, but **not monotonically** — several mid-span bands show an apparent chord smaller than expected. This irregularity is very likely an artifact of the aileron cutout (§16): where `left_aileron.stl` occupies the wing's trailing-edge region (Y 310–790), the wing mesh itself is missing that trailing-edge material, so `left_wing.stl`'s own local X-range in that band underrepresents the true local aerodynamic chord (which would include the aileron). This profile is reported as a raw observation, not as a resolved taper schedule — a true taper/twist schedule would require combining wing+aileron local geometry per span station, which is beyond a simple bounding-box method and is not attempted here. `Source: model/meshes/left_wing.stl`. `Method: STL vertex bounds, sub-region filtered by spanwise (Y) coordinate range`. `Status: DERIVED_FROM_MESH`, with the caveat stated explicitly.

**Wing area — explicitly not computed.** Per task instruction, STL triangle surface area is not computed or reported as a stand-in for aerodynamic planform/reference area. A clean top-down projected-area method was considered and rejected as insufficiently defensible for this pass: (1) the wing mesh root is truncated at Y=80 (embedded in the fuselage — §15 above), so any projection would omit the unknown carry-through area inside the fuselage without a documented method to recover it; (2) a convex-hull projection would not correctly represent the trailing-edge aileron cutout (§16) without first re-merging wing+aileron geometry, which was not done. **Status: `DATA_REQUIRED`** for wing planform/reference area. No number is reported that could be mistaken for the manufacturer's 0.4514 m² reference.

---

## 16. Aileron Geometry

`Source: model/meshes/left_aileron.stl, model/meshes/right_aileron.stl, model/meshes/left_wing.stl`. `Method: STL vertex bounds / slice-based comparison / gap scan`. `Status: DERIVED_FROM_MESH` unless noted.

| Quantity | left_aileron.stl | right_aileron.stl |
|---|---|---|
| Chordwise extent (X) | −23.578 to 37.541 (61.119) | −23.578 to 37.541 (61.119) |
| Spanwise extent (Y) | 310.374 to 789.740 (479.366) | −789.740 to −310.374 (479.366) |
| Thickness extent (Z) | 107.686 to 128.639 (20.953) | 107.686 to 128.639 (20.953) |
| Bounding-box center | (6.981, 550.057, 118.163) | (6.981, −550.057, 118.163) |

**Placement relative to wing:** the aileron's Y-span (310.374–789.740) is a sub-interval of `left_wing.stl`'s own Y-span (80.000–1052.027) — i.e. the aileron occupies an outboard-of-root, inboard-of-tip section of the wing span. `Status: DERIVED_FROM_MESH`.

**Trailing-edge / cutout evidence:** within the aileron's exact Y-footprint (310.374–789.740), `left_wing.stl`'s own local minimum X is +25.297 — i.e. the wing mesh does **not** have material at X < 25.297 in this Y-range. This is a real chordwise gap/cutout in the wing mesh at the aileron's spanwise location (not an artifact of a coarse Y-bin — verified using the aileron's exact Y-range as the slice boundary). `Source: model/meshes/left_wing.stl`. `Method: STL vertex bounds, sub-region filtered by spanwise (Y) coordinate range matching the aileron's own Y extent`. `Status: DERIVED_FROM_MESH`.

**Hinge-region candidate:** see §27 for the per-band comparison of the wing's local cutout boundary (≈25–35 mm, varies with span) against the aileron's own forward-most (hinge-side) edge (≈33–38 mm, varies with span). The two are close (within roughly −2 to +12 mm across 6 spanwise bands), consistent with the aileron's forward edge sitting at or just overlapping the wing's cutout boundary — a plausible hinge-region location. Tagged `HINGE_REQUIRES_CONFIRMATION` (§27); no single precise hinge axis line is asserted.

**Thickness/nose-shape evidence:** a chordwise thickness (Z-range) profile at a representative mid-span band (Y 470–550) increases **monotonically** from 3.69 mm at the aft (X≈−21 to −14, trailing-edge side) to 13.28 mm at the forward (X≈30 to 37, hinge side) end, with no decrease before the mesh boundary. This is the same signature seen in the elevator and rudder chordwise profiles (§26) — consistent with a partial-chord movable-surface mesh whose forward edge is a hinge cut, not a full-airfoil leading edge. `Source: model/meshes/left_aileron.stl`. `Method: STL vertex bounds, sub-region filtered by chordwise (X) coordinate range`. `Status: DERIVED_FROM_MESH`.

**Gap/overlap vs. wing, quantitatively (6 spanwise bands, `left_aileron.stl` vs `left_wing.stl`):**

| Y-band | wing local cutout X (min X in band) | aileron hinge-side X (max X in band) | (aileron − wing) mm |
|---|---|---|---|
| 310.4–390.3 | 32.801 | 37.541 | +4.740 |
| 390.3–470.2 | 33.424 | 33.089 | −0.335 |
| 470.2–550.1 | 25.297 | 37.371 | +12.074 |
| 550.1–630.0 | 34.373 | 37.351 | +2.978 |
| 630.0–709.8 | 34.767 | 33.348 | −1.419 |
| 709.8–789.7 | 35.122 | 33.431 | −1.691 |

`Source: model/meshes/left_wing.stl, model/meshes/left_aileron.stl`. `Method: STL vertex bounds, sub-region filtered by spanwise (Y) coordinate range, per band`. `Status: DERIVED_FROM_MESH`. The sign variation across bands (small mm-scale, both positive and negative) is consistent with binning/vertex-density noise rather than a fundamentally different relationship — no single straight hinge line is fit or asserted from this data; see §27.

**Neutral-position relationship:** no deflection-angle or actuation data exists to define "neutral" independent of the mesh's own as-exported pose. The as-exported pose is assumed by this analysis to represent the aileron's neutral (undeflected) position only insofar as no other pose data exists to compare against — this is **not asserted as confirmed neutral**, it is simply the only pose present in the file. `Status: DATA_REQUIRED` for an authoritative neutral-pose definition.

---

## 17. Horizontal Tail / Elevator Geometry

`Source: model/meshes/left_elevator.stl, model/meshes/right_elevator.stl, model/meshes/body.stl`. `Method: STL vertex bounds / slice-based comparison / gap scan / chordwise thickness profile`. `Status: DERIVED_FROM_MESH` unless noted.

| Quantity | left_elevator.stl | right_elevator.stl |
|---|---|---|
| Chordwise extent (X) | −517.657 to −472.684 (44.973) | −517.657 to −472.684 (44.973) |
| Spanwise extent (Y) | 50.600 to 240.000 (189.400) | −240.000 to −50.600 (189.400) |
| Thickness extent (Z) | 78.555 to 87.621 (9.066) | 78.555 to 87.621 (9.066) |
| Bounding-box center | (−495.170, 145.300, 83.088) | (−495.170, −145.300, 83.088) |

**Placement relative to body:** located at the tail X-region, immediately adjacent to (within 4.973 mm of) the body's tail-most X extreme (−526.880). Z-band (78.6–87.6) sits low in the body's overall Z range (0–330.5) — i.e. this is a low-mounted horizontal tail relative to the body's own vertical extent, and notably *lower* than the wing's Z-band (103.5–150.5). Reported as a raw observation; no aerodynamic implication is drawn. `Status: DERIVED_FROM_MESH`.

**Chordwise taper (8 spanwise bands):** root-side chord ≈44.97 mm tapering to tip-side chord ≈37.4 mm — mild, roughly monotonic taper, no internal chordwise gaps detected at any band (`gap_scan` empty at every tested band). `Status: DERIVED_FROM_MESH`.

**Component-scope evidence (full detail in §26):**
1. At the elevator's own X-band (−517.657 to −472.684) and Z-band (78.555 to 87.621), `body.stl` itself spans Y from −280.000 to +280.000 — i.e. the body already has material across the **full** 280 mm half-width at this exact tail station/height, well beyond the elevator's own 240 mm half-span. This is the same station identified in §14.1 as the body's global-widest point.
2. Chordwise thickness profile of `left_elevator.stl` (10 X-bands) increases **monotonically** from 1.35 mm at the aft/trailing edge (X≈−518 to −513) to 9.03 mm at the forward-most mesh boundary (X≈−477 to −473), with no decrease before that boundary.

Both observations, taken together, suggest `body.stl` already includes a fixed horizontal-stabilizer-like structure at this station, and `left_elevator.stl`/`right_elevator.stl` are movable-surface-only sections whose forward edge is a hinge cut (not a full-airfoil leading edge). This is `DERIVED_FROM_MESH` evidence; the scope determination was `COMPONENT_SCOPE_REQUIRES_CONFIRMATION` (§26) pending CAD-source or project-owner confirmation at the time this section was written — **now resolved, see §26.3** (master-dataset synchronization pass, 2026-08-21): concluded movable-surface-only, with the reasoning and residual ambiguity documented there.

**Possible hinge line:** the forward-most edge of each elevator band ranges X≈−472.68 (root) to X≈−474.41 (tip) — a nearly straight, mildly swept edge. See §27. `Status: HINGE_REQUIRES_CONFIRMATION`.

**Symmetry:** `left_elevator.stl` and `right_elevator.stl` are exact Y-mirrors (0.000 mm delta in X, Y, and Z bounds; identical facet counts, 31,462 each). `Status: CONFIRMED_FROM_MESH`.

---

## 18. Vertical Tail / Rudder Geometry

`Source: model/meshes/rudder.stl, model/meshes/body.stl`. `Method: STL vertex bounds / slice-based comparison / gap scan / chordwise thickness profile`. `Status: DERIVED_FROM_MESH` unless noted.

| Quantity | Value |
|---|---|
| Chordwise extent (X) | −517.999 to −472.684 (45.316) |
| Lateral (thickness) extent (Y) | −4.456 to 4.455 (8.911) |
| Vertical extent (Z) | 130.250 to 299.750 (169.500) |
| Bounding-box center | (−495.341, ~0.000, 215.000) |

**Center-plane relationship:** Y range (−4.456 to +4.455) is symmetric about Y=0 to sub-mm precision — `rudder.stl` sits on the aircraft's Y=0 centerline, consistent with a single (non-paired) vertical control surface. `Status: CONFIRMED_FROM_MESH`.

**Placement relative to body:** X-band immediately adjacent to the body's tail-most extreme (within 9.196 mm of −526.880), same region as the elevator meshes. `Status: DERIVED_FROM_MESH`.

**Component-scope evidence (full detail in §26):**
1. At the rudder's own X-band and Y-band (the ±4.456 mm centerline strip), `body.stl` spans Z from 66.322 to 330.480 — taller than the rudder mesh's own max Z of 299.750 by 30.730 mm. `body.stl`'s single tallest point anywhere in the entire mesh (Z=330.480) falls inside this X/Y band, at Y≈0.
2. Chordwise (height-wise) thickness profile of `rudder.stl` (10 Z-bands) increases **monotonically** from 1.405 mm at the aft/trailing edge (X≈−518 to −513) to 8.911 mm at the forward-most mesh boundary (X≈−477 to −473), with no decrease before that boundary — same signature as the elevator and aileron.

Together, this suggests `body.stl` already includes a fixed vertical-fin structure taller than `rudder.stl` itself, and `rudder.stl` is a movable-surface-only mesh set into that fin. `DERIVED_FROM_MESH` evidence; was formally `COMPONENT_SCOPE_REQUIRES_CONFIRMATION` (§26), **now resolved, see §26.3** (master-dataset synchronization pass, 2026-08-21): concluded movable-surface-only.

**Chordwise (height) taper:** root (bottom, Z≈130–147) chord ≈44.97 mm tapering to tip (top, Z≈283–300) chord ≈38.24 mm — mild, roughly monotonic taper, no internal gaps detected in any height band (`gap_scan` empty). `Status: DERIVED_FROM_MESH`.

**Possible hinge line:** forward-most edge ranges X≈−472.68 (bottom) to X≈−474.37 (top) — nearly straight, mildly swept, same pattern as the elevator's candidate hinge edge. `Status: HINGE_REQUIRES_CONFIRMATION` (§27).

---

## 19. Motor Geometry

`Source: model/meshes/left_motor.stl, model/meshes/right_motor.stl`. `Method: STL vertex bounds / symmetry comparison`. `Status: DERIVED_FROM_MESH` unless noted.

| Quantity | left_motor.stl | right_motor.stl |
|---|---|---|
| Extent X | 236.335 to 302.819 (66.485) | 236.335 to 302.819 (66.485) |
| Extent Y | 280.862 to 319.087 (38.225) | −319.138 to −280.913 (38.225) |
| Extent Z | 107.856 to 146.093 (38.236) | 107.856 to 146.093 (38.236) |
| Bounding-box center | (269.577, 299.974, 126.975) | (269.577, −300.026, 126.975) |

**Shape observation bearing on thrust axis:** the X extent (66.485 mm) is substantially larger than the Y extent (38.225 mm) and Z extent (38.236 mm), and Y/Z extents are nearly equal (0.011 mm apart) — consistent with an elongated body with a roughly circular cross-section in the Y-Z plane, i.e. a cylindrical motor "can" whose long axis is X and whose rotation-symmetric cross-section is Y-Z. This is corroborating (not conclusive) evidence for a thrust axis parallel to local X. **Status: `THRUST_AXIS_REQUIRES_CONFIRMATION`** — a mesh's long bounding-box axis is not proof of the actual motor shaft/thrust direction (per task instruction); no propulsion physics is modeled or asserted here.

**Placement relative to body:** motor X-range (236.3–302.8) sits inboard of the body's nose extreme (526.880) by roughly 224–290 mm, and Y-center (≈±300) is well outboard of the body's own half-width (280.000) — i.e. the motors sit **outboard of the fuselage**, within the wing's spanwise range (wing Y 80–1052), not on the fuselage nose. This is consistent with a twin wing-mounted nacelle configuration rather than a single fuselage-nose-mounted motor, and reconciles the CLAUDE.md qualitative fact "twin front-puller" with a fuselage nose (X=526.880) that extends well forward of both motors — the nose is simply the fuselage's own nose cone, geometrically unrelated in this analysis to the motor mounting location. `Status: DERIVED_FROM_MESH`.

**Placement relative to the current Gazebo/CAD CG (0.168309, 0, 0.100000 m):** this comparison requires two unconfirmed assumptions, both stated explicitly rather than applied silently: (a) mesh coordinates are millimeters (§13.1, evidenced but not CAD-confirmed), and (b) the mesh coordinate origin coincides with the origin used for the documented Gazebo/CAD CG (**not confirmed anywhere in this repository** — CG duality rules in `CLAUDE.md`/§8 of this document forbid assuming frame/origin equivalence without derivation). With both caveats stated: if both held, motor X (0.236–0.303 m) would be forward of CG X (0.168309 m) by 0.068–0.134 m, and motor Z (0.108–0.146 m) would be moderately above CG Z (0.100 m) — a plausible, not contradictory, relationship consistent with the qualitative "motor forward of CG" fact in §7 of this document. This is reported strictly as a **plausibility cross-check**, not as a confirmed coordinate relationship. `Status: DATA_REQUIRED` for the actual mesh-origin-to-CG-origin transform (same underlying gap already tracked in §8.3/§9 of this document).

**Symmetry:** `left_motor.stl`/`right_motor.stl` are Y-mirrors to within 0.052 mm (X and Z deltas exactly 0.000 mm); identical facet counts (697,134 each). `Status: CONFIRMED_FROM_MESH`.

---

## 20. Propeller Geometry

`Source: model/meshes/left_pervane.stl, model/meshes/right_pervane.stl, model/meshes/left_motor.stl, model/meshes/right_motor.stl`. `Method: STL vertex bounds / geometric center calculation / max radial distance from bbox-center in the Y-Z plane`. `Status: DERIVED_FROM_MESH` unless noted. ("Pervane" = Turkish for propeller; cataloged here as a mesh-geometry item only — propeller aerodynamic/performance modeling is `propulsion`'s domain, not addressed here.)

> ### `VISUAL_MESH_ONLY` — read before using any number in this section
>
> **Every propeller-diameter figure derived from `left_pervane.stl`/`right_pervane.stl` in this section and in §21.2 (≈273.4–273.5 mm raw mesh units / ≈0.2734–0.2735 m under the millimeter interpretation) is confirmed to be a visual-mesh artifact only. It is NOT the physical propeller and must NEVER be used as a physics input.**
>
> Explicitly forbidden uses of this mesh-derived ≈273 mm figure — RPM calculations, thrust calculations, torque calculations, advance-ratio calculations, propeller disk-area calculations, blade-tip-speed calculations, motor-load calculations, throttle→RPM mapping, RPM→thrust mapping, and airspeed-dependent-thrust modeling. None of these may consume this number, in this document or in any downstream implementation.
>
> **The real, physical propeller for all propulsion/physics calculations is the APC 13x6.5E:** nominal diameter **D = 0.3302 m** (13 in), nominal pitch **0.1651 m** (6.5 in). Source: `CLAUDE.md` propulsion reference / project owner. Applying this value inside any propulsion model (RPM, thrust, torque, advance ratio, disk area, tip speed, etc.) is `propulsion`'s domain — not this document's — and is not performed here.
>
> This reclassification does not remove or alter the STL measurement below — it remains a valid, confirmed geometric observation about the *mesh as exported*. It is labeled so it can never be mistaken for a propulsion/physics reference by a future reader.

| Quantity | left_pervane.stl | right_pervane.stl |
|---|---|---|
| Extent X (thin/rotation axis) | 286.617 to 303.550 (16.932) | 286.617 to 303.550 (16.932) |
| Extent Y | 163.298 to 436.717 (273.419) | −436.702 to −163.283 (273.419) |
| Extent Z | 116.485 to 137.661 (21.176) | 116.485 to 137.661 (21.176) |
| Bounding-box center | (295.084, 300.007, 127.073) | (295.084, −299.993, 127.073) |

**Diameter estimate:** the X extent (16.932 mm) is much smaller than the Y or Z extents, consistent with a thin, roughly-planar propeller disc whose rotation axis is local X (matching the motor's evidenced long/thrust axis, §19). Diameter was estimated as 2× the maximum radial distance from the bounding-box center in the Y-Z (candidate rotation) plane — this method is robust to whatever azimuthal (blade-pointing) orientation the mesh happens to be posed in, unlike simply reading the raw Y or Z bbox size:

| Method | left_pervane.stl | right_pervane.stl |
|---|---|---|
| Max radius from pervane's own bbox-center (Y,Z) | 136.719 mm → diameter 273.438 mm | 136.719 mm → diameter 273.438 mm |
| Max radius from motor's bbox-center (Y,Z) axis | 136.751 mm → diameter 273.503 mm | 136.751 mm → diameter 273.503 mm |

Both methods agree closely (≤0.065 mm apart) and both sides match. **Mesh-derived diameter ≈ 273.4–273.5 mm** (raw mesh units). `Status: DERIVED_FROM_MESH` — and, as of this task, additionally tagged **`VISUAL_MESH_ONLY`: this ≈273.4–273.5 mm figure is a visual-mesh artifact, confirmed not to be the physical propeller, and must never be used in any RPM, thrust, torque, advance-ratio, disk-area, tip-speed, motor-load, throttle→RPM, RPM→thrust, or airspeed-dependent-thrust calculation. The real physical propeller diameter for all such calculations is D = 0.3302 m (APC 13x6.5E, 13 in nominal) — see the banner at the top of §20.** Comparison against the APC 13x6.5E nominal 13-inch diameter is in §21.2, tagged `GEOMETRIC_CHECK` and `VISUAL_MESH_ONLY` — **this is a geometric sanity check only, not propulsion performance data**, and pitch is not derived or guessed from the mesh anywhere in this document.

**Placement relative to motor:** propeller X-range (286.617–303.550) overlaps the motor's forward portion (motor X max = 302.819) and extends 0.731 mm beyond it — i.e. the propeller sits at the front (max-X, nose-facing) end of the motor, protruding slightly forward, consistent with a front-mounted (tractor) propeller on a shaft protruding from the motor can. Y and Z centers of the propeller (300.007, 127.073) match the motor's Y/Z center (299.974, 126.975) to within 0.13 mm — strong evidence of a coaxial mount. `Status: CONFIRMED_FROM_MESH` (relative coaxial placement); the *thrust direction implied by this* still carries the same `THRUST_AXIS_REQUIRES_CONFIRMATION` caveat as §19, since coaxial placement confirms alignment between the two parts, not the absolute thrust vector's meaning in the aircraft frame.

**Apparent rotation axis:** local X, by the same elongation/coaxiality evidence as above. `Status: DERIVED_FROM_MESH` for the geometric observation; `THRUST_AXIS_REQUIRES_CONFIRMATION` for treating it as the actual propulsive thrust axis.

**Symmetry:** `left_pervane.stl`/`right_pervane.stl` are Y-mirrors to within 0.015 mm (X and Z deltas exactly 0.000 mm); identical facet counts (79,088 each). `Status: CONFIRMED_FROM_MESH`.

---

## 21. Geometry vs. Manufacturer Checks

### 21.1 Wingspan

| Quantity | Value |
|---|---|
| Mesh-derived tip-to-tip span (raw mesh units) | 2104.054 |
| Mesh-derived tip-to-tip span, millimeter interpretation | 2.104054 m |
| Manufacturer wingspan reference | 2.093 m |
| Absolute difference | +0.011054 m (mesh larger) |
| Percentage difference | +0.528% |

`Source: model/meshes/left_wing.stl, right_wing.stl` (mesh); `CLAUDE.md` / `docs/source_of_truth/README.md` (manufacturer reference). `Method: STL vertex bounds, dimensional comparison`. **Status: `GEOMETRIC_CHECK`.** This uses the millimeter interpretation of §13.1 explicitly — it is not an applied/silent unit conversion elsewhere in this document, and the mesh coordinate unit itself remains formally `DATA_REQUIRED`. No geometry is altered as a result of this check, per task instruction and per `CLAUDE.md`'s "no mesh modification without authorization" rule.

### 21.2 Propeller diameter (geometric sanity check only — not propulsion data) — `VISUAL_MESH_ONLY`

**RECLASSIFICATION (2026-08-21, this task, explicit project-owner instruction — non-negotiable):** the ≈273 mm figure below is confirmed to be a visual mesh artifact only. It is **not** the physical propeller and must **never** be used in any RPM, thrust, torque, advance-ratio, propeller-disk-area, tip-speed, motor-load, throttle→RPM, RPM→thrust, or airspeed-dependent-thrust calculation, in this document or anywhere downstream. The real physical propeller for all physics/propulsion calculations is the **APC 13x6.5E** — nominal diameter **D = 0.3302 m** (13 in), nominal pitch **0.1651 m** (6.5 in) — which is `propulsion`'s domain to apply. The STL measurement itself is preserved below unmodified; only its classification is changed.

| Quantity | Value | Status |
|---|---|---|
| Mesh-derived diameter (raw mesh units) | ≈273.4–273.5 | `VISUAL_MESH_ONLY` — mesh artifact, not a physics input |
| Mesh-derived diameter, millimeter interpretation | ≈0.2734–0.2735 m | `VISUAL_MESH_ONLY` — mesh artifact, not a physics input |
| **APC 13x6.5E nominal diameter (13 in) — the real physical value for all propulsion/physics use** | **0.3302 m** | `CONFIRMED` (manufacturer data, `CLAUDE.md`) |
| APC 13x6.5E nominal pitch (6.5 in) — recorded for completeness, not derived from the mesh | 0.1651 m | `CONFIRMED` (manufacturer data, `CLAUDE.md`) |
| Absolute difference (mesh vs. nominal) | ≈−0.0567 m (mesh smaller) | `GEOMETRIC_CHECK` only |
| Percentage difference (mesh vs. nominal) | ≈−17.2% | `GEOMETRIC_CHECK` only |

`Source: model/meshes/left_pervane.stl, right_pervane.stl` (mesh); `CLAUDE.md` propulsion reference (APC 13x6.5E). `Method: geometric center calculation + max radial distance in the Y-Z (candidate rotation) plane, dimensional comparison`. **Status: `GEOMETRIC_CHECK` + `VISUAL_MESH_ONLY`.** This is a geometric sanity check only — it is explicitly **not** presented as propeller performance data, and no pitch value is derived or guessed from the mesh (the 0.1651 m pitch above is manufacturer data, not a mesh measurement). The ≈17% discrepancy between the mesh figure and the real APC 13x6.5E diameter is exactly why the mesh figure is reclassified `VISUAL_MESH_ONLY` in this task — no cause for the discrepancy is asserted (e.g. it is not concluded here whether the mesh represents a different prop, a simplified/scaled visual model, or a units artifact); regardless of cause, the mesh value is not to be used for physics. This is left for `propulsion`/the project owner to resolve if the underlying cause matters for future mesh work, consistent with this agent's ownership boundary (mesh geometry only, not propeller performance modeling). For all propulsion/physics calculations, use D = 0.3302 m.

### 21.3 Full-inventory bounding-dimension cross-check against master dataset §11 (2026-08-21, master-dataset synchronization pass)

Master dataset §11 independently states approximate bounding dimensions for 7 of the 12 meshes (body, wing, aileron, elevator, rudder, motor, prop STL), in millimeters. Compared against this document's own §13 vertex-derived bounding boxes:

| Component | Master dataset §11 (mm) | This document, §13 (mm) | Result |
|---|---|---|---|
| body | ≈1053.8 × 560.0 × 330.5 | 1053.761 × 559.999 × 330.480 | Match |
| each wing | ≈266.5 × 972.0 × 47.0 | 266.510 × 972.027 × 46.956 | Match |
| each aileron | ≈61.1 × 479.4 × 21.0 | 61.119 × 479.366 × 20.953 | Match |
| each elevator | ≈45.0 × 189.4 × 9.1 | 44.973 × 189.400 × 9.066 | Match |
| rudder | ≈45.3 × 8.9 × 169.5 | 45.316 × 8.911 × 169.500 | Match |
| each motor | ≈66.5 × 38.2 × 38.2 | 66.485 × 38.225 × 38.236 | Match |
| each prop STL | ≈16.9 × 273.4 × 21.2 | 16.932 × 273.419 × 21.176 | Match (this is the same `VISUAL_MESH_ONLY`-tagged mesh diameter as §20/§21.2 — the cross-check here is purely about raw-mesh-dimension consistency between the two documents, not a re-endorsement of the figure for physics use) |
| body bounds (min/max) | X=−0.5269→+0.5269, Y=−0.2800→+0.2800, Z=0→+0.3305 m | X=−0.526880→+0.526880, Y=−0.280000→+0.280000, Z=0.000→+0.330480 m | Match |

**Result: all 7 cross-checked components match to the precision the master dataset itself reports (≤0.06 mm / ≤0.06% in every case).** No discrepancy was found — this fully confirms item 9 of this task's assignment (cross-check the existing STL bounding numbers against master dataset §11). `Source: model/meshes/*.stl` (this document's own independent §13 analysis); master dataset §11. `Method: side-by-side dimensional comparison`. `Status: GEOMETRIC_CHECK` (confirmatory; also serves as one of the convergent cross-checks supporting `STL_SCALE_TO_SI`'s `DERIVED_WITH_STRONG_EVIDENCE` status, §13.1).

---

## 22. Symmetry Checks

All five left/right mesh pairs were checked for mirror symmetry about Y=0 by comparing each pair's Y-range against the other's Y-range negated (mirrored), and independently confirming X and Z ranges match exactly (i.e. the parts differ *only* in Y-sign, not in independent X/Z placement). `Source: model/meshes/left_*.stl, right_*.stl` (5 pairs). `Method: symmetry comparison (mirror Y, compare X/Y/Z bounds and facet counts)`. `Status: CONFIRMED_FROM_MESH` for all 5 pairs (deltas are all ≤0.052 mm, consistent with float32 rounding, not a real asymmetry).

| Pair | Y-mirror delta (min/max) | X delta | Z delta | Facet count (L / R) | Facet count delta |
|---|---|---|---|---|---|
| left_wing.stl / right_wing.stl | 0.000 mm / 0.000 mm | 0.000 mm | 0.000 mm | 181,330 / 181,328 | 2 |
| left_aileron.stl / right_aileron.stl | 0.000 mm / 0.000 mm | 0.000 mm | 0.000 mm | 28,786 / 28,786 | 0 |
| left_elevator.stl / right_elevator.stl | 0.000 mm / 0.000 mm | 0.000 mm | 0.000 mm | 31,462 / 31,462 | 0 |
| left_motor.stl / right_motor.stl | −0.052 mm / −0.052 mm | 0.000 mm | 0.000 mm | 697,134 / 697,134 | 0 |
| left_pervane.stl / right_pervane.stl | +0.015 mm / +0.015 mm | 0.000 mm | 0.000 mm | 79,088 / 79,088 | 0 |

The 2-facet difference between `left_wing.stl` (181,330) and `right_wing.stl` (181,328) is a raw file-level observation (already noted in §4.2); it does not appear as a bounding-box or center discrepancy at the precision reported here (both wings' X/Y/Z bounds and centers match to the same tolerance as the other, facet-count-identical pairs). No geometric conflict is asserted from the 2-facet difference — it is far too small a fraction of ~181,000 facets to draw a conclusion from bounding-box data alone.

`rudder.stl` has no left/right counterpart (single centerline part, §18) — not applicable to this section.

---

## 23. Frame Compatibility

Checked whether the 12 meshes' coordinate data is compatible with the project's FLU convention (+X forward, +Y left, +Z up) without any axis swap or sign flip.

| Check | Evidence | Result |
|---|---|---|
| Long/nose-tail axis is X | Tail control surfaces (elevator, rudder) cluster at one X extreme (§14); nose is unambiguously the other X extreme | Consistent with +X = forward/aft axis |
| Left/right split is clean across Y=0 | All 5 L/R pairs mirror about Y=0 to ≤0.052 mm (§22); body itself is Y-symmetric to ≤0.0004 mm | Consistent with Y as the left/right axis |
| "Left" parts are at positive Y | `left_wing.stl`, `left_aileron.stl`, `left_elevator.stl`, `left_motor.stl`, `left_pervane.stl` are all at Y>0 | Consistent with +Y = left (FLU), not −Y = left |
| Up axis is Z | Vertical-fin-height structure (rudder region) is the tallest Z region of the body (§14.1, §18); body's lowest point is Z≈0 (§14); wing sits at moderate Z, well below the fin peak | Consistent with +Z = up |

**Result: no `FRAME_MAPPING_REQUIRED` condition was found for axis orientation** — the 12 meshes' relative geometry (nose/tail, left/right, up/down) is directly compatible with the FLU convention as-exported, with no evidenced need for an axis swap or sign flip. `Source: model/meshes/*.stl` (all 12). `Method: STL vertex bounds, symmetry comparison, argmax/argmin point query`. `Status: CONFIRMED_FROM_MESH` for relative axis orientation only.

**What this does *not* confirm:** (1) the mesh coordinate **unit** (§13.1, `DATA_REQUIRED`); (2) whether the mesh coordinate **origin** is the same physical/reference origin as the one used for the documented Gazebo/CAD CG (§19, `DATA_REQUIRED` — CG-frame-duality rules in `CLAUDE.md` explicitly forbid assuming this); (3) any SDF-ready hinge axis, incidence, dihedral, or sweep angle. Axis-orientation compatibility and origin/unit confirmation are separate questions, and only the former is resolved by this section.

---

## 24. Geometry Relationships

Per task instruction, each relationship below is tagged `CONFIRMED_FROM_MESH` (clear, well-evidenced from the coordinate data) or `REQUIRES_CONFIRMATION` (ambiguous, or not fully derivable from bounding-box/slice data alone).

| Relationship | Finding | Status |
|---|---|---|
| Wing-to-body | Wing root (Y≈80) sits inboard of the body's local outer surface (Y≈94.65) at the matching X/Z station — root is embedded in/adjacent to the fuselage shell, no floating gap (§15, §17.1 analog for wing in §25) | `CONFIRMED_FROM_MESH` (placement); exact CAD wing-root reference station is `DATA_REQUIRED` |
| Aileron-to-wing | Aileron spans an outboard sub-section of the wing's Y-range, occupies a real chordwise cutout in the wing mesh at that Y-range, with a plausible (not precisely fit) hinge-region boundary (§16) | `CONFIRMED_FROM_MESH` (relative placement / cutout existence); precise hinge axis is `HINGE_REQUIRES_CONFIRMATION` (§27) |
| Elevator-to-body | Elevator sits at the tail X-region, at a Z-band where the body itself already has wide (full-half-width) structure (§17) | `CONFIRMED_FROM_MESH` (relative placement); component scope (fixed+movable vs. movable-only) resolved 2026-08-21 as movable-only — see §26.3 |
| Rudder-to-body | Rudder sits at the tail X-region, on the Y=0 centerline, at a height-band where the body itself already has tall (up to global-max-Z) structure (§18) | `CONFIRMED_FROM_MESH` (relative placement); component scope resolved 2026-08-21 as movable-only — see §26.3 |
| Motor-to-body | Motor sits outboard of the fuselage (Y≈±300, beyond body half-width 280), within the wing's spanwise range, inboard of the aileron's own span start — consistent with a wing-mounted nacelle, not a fuselage-nose mount (§19) | `CONFIRMED_FROM_MESH` (relative placement); no dedicated mount/pylon/strut mesh exists among the 12 files, so the mechanical mounting interface itself is `DATA_REQUIRED` |
| Propeller-to-motor | Propeller sits coaxially at the motor's forward (max-X) end, Y/Z centers matching to ≤0.13 mm, X-overlap consistent with a shaft-mounted tractor prop (§20) | `CONFIRMED_FROM_MESH` (relative coaxial placement); absolute thrust-axis meaning is `THRUST_AXIS_REQUIRES_CONFIRMATION` |

---

## 25. Body Slice Evidence (supporting data for §15/§17/§18/§24)

Raw slice results referenced above, collected here for traceability. `Source: model/meshes/body.stl` (all rows). `Method: STL vertex bounds, sub-region filtered by coordinate range`. `Status: DERIVED_FROM_MESH`.

| Slice region on body.stl | Restriction | Result |
|---|---|---|
| Wing-root station | X ∈ [−27.512, 238.998] (wing's own X-range) | Y range [−94.649, 94.649] |
| Wing-root station, further restricted | X as above, Z ∈ [103.541, 150.497] (wing's own Z-range) | Y range [−94.649, 94.649], 224,487 vertices in slice |
| Tail station (elevator/rudder X-range) | X ∈ [−517.999/−517.657, −472.684] | Y range [−280.000, 280.000]; Z range [66.322, 330.480] |
| Tail station at elevator's Z-band | X as above, Z ∈ [78.555, 87.621] (elevator's own Z-range) | Y range [−280.000, 280.000], 414,380 vertices in slice |
| Tail station at rudder's Y-band | X as above, Y ∈ [−4.456, 4.455] (rudder's own Y-range) | Z range [66.322, 330.480], 192,481 vertices in slice |

---

## 26. Unresolved Component Scope

Per task instruction (at the time this section was written), this was not decided from mesh inference alone — both items were formally `COMPONENT_SCOPE_REQUIRES_CONFIRMATION`. The evidence gathered in that pass is summarized here (§26.1, §26.2, unchanged below). **Resolution reached 2026-08-21 (master-dataset synchronization pass) — see §26.3, which weighs this evidence together with new master-dataset evidence and concludes movable-surface-only for all three meshes, with an explicitly-stated residual (judged small, not zero) ambiguity.**

### 26.1 `left_elevator.stl` / `right_elevator.stl` — movable elevator only, or fixed stabilizer + movable elevator combined?

Evidence gathered:
1. `body.stl` already spans the full ±280 mm half-width at the elevator's exact X-band and Z-band (§17, §25) — wider than the elevator mesh's own ±240 mm half-span. The body's single widest point (anywhere in the entire 920k-facet mesh) occurs at this same station.
2. Chordwise thickness profile of the elevator mesh increases monotonically from a thin (1.35 mm) trailing edge to a thick (9.03 mm) forward boundary, with **no decrease** before that boundary — the mesh appears to be cut off before reaching a full-airfoil thickness peak, i.e. consistent with a partial-chord (movable-only) section.
3. No internal chordwise gap was found within the elevator mesh itself at any tested spanwise band (a single continuous surface, not two visibly disjoint pieces stitched together).

Interpretation: (1) and (2) together suggest `body.stl` carries the fixed horizontal-stabilizer geometry and `left_elevator.stl`/`right_elevator.stl` are movable-surface-only. (3) means this is *not* proven by finding an explicit split within the elevator file itself — the evidence is comparative (elevator vs. body), not internal to the elevator mesh. **Status: `COMPONENT_SCOPE_REQUIRES_CONFIRMATION`** — not decided here.

### 26.2 `rudder.stl` — movable rudder only, or fixed vertical fin + movable rudder combined?

Evidence gathered:
1. `body.stl` already spans Z 66.322–330.480 at the rudder's exact X-band and Y-band (§18, §25) — taller than the rudder mesh's own max Z of 299.750 by 30.730 mm. The body's single tallest point occurs inside this same X/Y band, at Y≈0.
2. Chordwise (height-wise) thickness profile of the rudder mesh increases monotonically from a thin (1.405 mm) trailing edge to a thick (8.911 mm) forward boundary, with no decrease before that boundary — same signature as the elevator.
3. No internal gap was found within the rudder mesh itself at any tested height band.

Interpretation: same reasoning as §26.1 — suggests `body.stl` carries the fixed vertical-fin geometry and `rudder.stl` is movable-surface-only, but not proven by an internal split. **Status (prior to §26.3): `COMPONENT_SCOPE_REQUIRES_CONFIRMATION`** — not decided here at the time this subsection was written.

**Why this still isn't a final determination (as originally written):** bounding-box/slice evidence cannot distinguish "body.stl has a wide/tall tail fairing that happens not to be an aerodynamic surface" from "body.stl has an actual fixed stabilizer/fin." No airfoil-camber-line extraction, mesh-region labeling, or CAD part-tree inspection was performed (none of these are available from bounding-box/slice analysis of an unlabeled watertight mesh). This was, at the time, left to be confirmed from the CAD source or the project owner before any SDF link-splitting decision — see §26.3 for the resolution reached in the master-dataset synchronization pass.

### 26.3 Resolution reached (2026-08-21, master-dataset synchronization pass)

**Status change: `COMPONENT_SCOPE_REQUIRES_CONFIRMATION` → resolved.** Conclusion: `left_elevator.stl` / `right_elevator.stl` / `rudder.stl` are movable-surface-only meshes; `body.stl` carries the fixed horizontal-stabilizer and vertical-fin structure. This is reached by weighing the §26.1/§26.2 mesh-geometry evidence (unchanged, still valid, not superseded) together with new evidence from the master dataset — explicitly reasoned through below, not cleared merely because instructed to check it:

1. **Prior mesh evidence (§26.1, §26.2, unchanged):** at the elevator's/rudder's own X-band, `body.stl` already has wider/taller structure than the elevator/rudder meshes themselves reach, and the elevator/rudder meshes' own chordwise thickness profile increases monotonically to their forward boundary with no peak-then-taper — the signature of a partial-chord section cut off at a hinge line, not a complete airfoil.
2. **Master dataset §66 ("MOVABLE LINKS / JOINTS")** explicitly lists `left_aileron`, `right_aileron`, `left_elevator`, `right_elevator`, `rudder`, `left_prop`, `right_prop` as the components with "mesh hazır" (mesh ready) for a separate link/joint — i.e., the project's own working engineering assumption, stated independently of this document's mesh analysis, is that these five control-surface STL files are each already scoped as an individual movable part suitable for a single hinge joint, not a combined fixed+movable assembly.
3. **Master dataset §21 ("GERÇEK ELEVATOR HINGE")** states a "movable elevator span" (y ≈ 50.60–240.00 mm) as a specific, named quantity. Cross-checked directly against this document's own §17 mesh measurement: `left_elevator.stl`'s own mesh Y-span is 50.600–240.000 mm — an exact match. If the STL file contained additional fixed structure inboard of the movable panel, the mesh's own Y-span would be expected to extend further inboard than the stated "movable" span; it does not. This is a direct, checkable confirmation, not just a suggestive naming choice.
4. **The STL parts list itself (master dataset §10, cross-checked against the 12 files actually present, §4.2) contains no separate mesh for a fixed horizontal-stabilizer or vertical-fin structure.** Every other movable surface in the package has its own distinct fixed-structure counterpart as a *separate* mesh file (aileron ↔ `left_wing.stl`/`right_wing.stl`). No analogous separate fixed-tail mesh exists among the 12 files. Combined with point 1 (body.stl already has the requisite extra material at exactly the matching station), the more parsimonious reading is that the fixed tail structure is modeled as part of `body.stl`, not omitted or hidden elsewhere.

**Genuine residual ambiguity, stated plainly:** none of the four points above is a literal CAD part-tree readout or a project-owner statement of intent for this exact question. It remains conceivable that `body.stl`'s extra tail-station material is fairing/blend geometry rather than a true aerodynamic fixed-stabilizer/fin surface, and that no fixed-tail-surface geometry is represented in the mesh package at all. This residual possibility is judged small — not zero — given the convergence of four independent lines of evidence (two from direct mesh geometry, two from the master dataset's own stated engineering treatment, one of which — point 3 — is a precise numeric match, not just a qualitative reading). It is judged small enough to no longer block SDF link/joint structuring work on this specific question, but it is **not** formally CAD-confirmed. If a CAD source or the project owner later states otherwise, this conclusion must be revisited and this section updated (not silently overwritten).

**Practical consequence for future SDF work:** `left_elevator.stl`, `right_elevator.stl`, and `rudder.stl` may be treated as single-link movable control surfaces, each attachable to a hinge joint at the candidate/real hinge region already identified (§27, §6/§32.6). The fixed horizontal-stabilizer and vertical-fin surfaces are treated as part of the fixed `body.stl` link — no additional fixed-tail mesh/link needs to be sourced or split out for V1. This is a geometry/SDF-structuring conclusion only; it changes no mass, CG, or mesh file, and no mesh file was modified to reach it.

---

## 27. Hinge Candidates

All items in this section are `HINGE_REQUIRES_CONFIRMATION` — none are asserted as a confirmed hinge axis. Reported as candidate regions with supporting evidence only.

| Control surface | Candidate hinge-region evidence | Status |
|---|---|---|
| Aileron (left/right) | Wing's local trailing-edge cutout boundary (X≈25–35 mm, varies by span) closely tracks the aileron's own forward (hinge-side) edge (X≈33–38 mm, varies by span) — see §16 table for the 6-band comparison (deltas −1.7 to +12.1 mm). Aileron's forward-edge thickness increases monotonically with no peak-then-taper, consistent with a hinge cut rather than a full-chord leading edge. | `HINGE_REQUIRES_CONFIRMATION` |
| Elevator (left/right) | Forward-most mesh edge ranges X≈−472.68 (root) to X≈−474.41 (tip) — a nearly straight, mildly swept candidate line. Thickness at this edge is the mesh's local maximum and still increasing at the boundary (§17). | `HINGE_REQUIRES_CONFIRMATION` |
| Rudder | Forward-most mesh edge ranges X≈−472.68 (root/bottom) to X≈−474.37 (tip/top) — nearly straight, mildly swept, same pattern as the elevator. Thickness at this edge is the mesh's local maximum and still increasing at the boundary (§18). | `HINGE_REQUIRES_CONFIRMATION` |

None of these candidates include a fitted 3D line equation, a joint-axis unit vector, or a confirmed rotation point — only the coordinate region where the evidence points. Precise hinge-axis definition for SDF `<joint><axis>` use requires either CAD-source confirmation or an explicit, authorized follow-on geometric fitting exercise, neither of which is performed in this task.

---

## 28. Confirmed Values (this pass)

Consolidated list of items reaching `CONFIRMED_FROM_MESH` status in §12–§27 (full detail and provenance in the referenced section — not repeated here):

1. All 12 mesh files are valid, untruncated binary STL (§12).
2. Body's left/right symmetry plane is Y=0, to ≤0.0004 mm (§14, §23).
3. All 5 left/right mesh pairs are true Y-mirrors, to ≤0.052 mm, with matching or near-matching facet counts (§22).
4. Mesh coordinate data is directly compatible with the FLU axis convention (+X forward, +Y left, +Z up) with no evidenced need for an axis swap or sign flip (§23).
5. Wing root end (Y≈80/−80) sits closer to the body than the tip end (Y≈1052/−1052) (§15).
6. Rudder sits on the Y=0 centerline (§18).
7. Propeller is coaxially mounted at the motor's forward end, Y/Z centers matching to ≤0.13 mm (§20).
8. Aileron/elevator/rudder relative placement to their parent structures (wing/body) (§24 table).

---

## 29. Derived Geometric Values (this pass)

Consolidated list of `DERIVED_FROM_MESH` numeric values computed in §12–§27 (full detail and provenance in the referenced section — not repeated here):

1. Vertex-wise bounding box (min/max/size/center) for all 12 meshes (§13).
2. Mesh-to-mesh tip-to-tip wingspan: 2104.054 raw mesh units (§15, §21.1).
3. Mesh-derived propeller diameter: ≈273.4–273.5 raw mesh units, two independent methods (§20, §21.2). **`VISUAL_MESH_ONLY` (reclassified 2026-08-21, this task) — mesh artifact, not the physical propeller; never use for RPM/thrust/torque/advance-ratio/disk-area/tip-speed/motor-load/throttle→RPM/RPM→thrust/airspeed-dependent-thrust physics. The real physical propeller for those calculations is the APC 13x6.5E, D = 0.3302 m (13 in nominal), which is `propulsion`'s domain to apply.**
4. Body slice extents at the wing-root, elevator, and rudder stations (§25).
5. Aileron/wing cutout-boundary comparison, 6 spanwise bands (§16).
6. Chordwise thickness profiles for aileron, elevator, and rudder showing the monotonic-increase-to-forward-boundary signature (§16, §17, §18).
7. Candidate hinge-region coordinates for aileron, elevator, and rudder (§27).

---

## 30. DATA_REQUIRED (this pass)

New or continuing `DATA_REQUIRED` / `*_REQUIRES_CONFIRMATION` items introduced or reaffirmed by this pass. Items already tracked in §11 (e.g. inertia, incidence/dihedral/sweep angles, battery/ESC placement) are not repeated here except where this pass adds new evidence to them.

1. **Mesh coordinate unit** — strongly evidenced as millimeters (§13.1), not CAD-confirmed. `DATA_REQUIRED`.
2. **Physical meaning of the shared mesh-assembly origin** (0,0,0) — e.g. whether it corresponds to a firewall, a specific CAD datum, or another named reference point. `DATA_REQUIRED` (§13.2).
3. **Whether the mesh coordinate origin is the same origin used for the documented Gazebo/CAD CG** (0.168309, 0, 0.100000 m) — not assumed anywhere in this document; a plausibility-only cross-check was performed for the motor position (§19), not a confirmed relationship. `DATA_REQUIRED`.
4. **Component scope of `left_elevator.stl`/`right_elevator.stl`** (movable-only vs. fixed+movable) — mesh evidence gathered (§26.1). **RESOLVED 2026-08-21 (master-dataset synchronization pass) — see §26.3: concluded movable-only.** No longer `COMPONENT_SCOPE_REQUIRES_CONFIRMATION`.
5. **Component scope of `rudder.stl`** (movable-only vs. fixed+movable) — mesh evidence gathered (§26.2). **RESOLVED 2026-08-21 — see §26.3: concluded movable-only.** No longer `COMPONENT_SCOPE_REQUIRES_CONFIRMATION`.
6. **Precise hinge axis (as a joint-ready line/vector) for aileron, elevator, and rudder** — candidate regions identified (§27); real per-station hinge x/c and position data now added from the master dataset (§6, §32.6), tagged `HINGE_GEOMETRY_READY`. Still not a fitted single SDF axis/pose, and not sign-tested — `HINGE_REQUIRES_CONFIRMATION`/`HINGE_GEOMETRY_READY` (data exists; axis-fit + sign-test remain).
7. **Wing planform/aerodynamic reference area** — explicitly not derived from the mesh this pass; reasoning stated in §15. `DATA_REQUIRED`.
8. **Motor thrust-axis confirmation** — strong corroborating shape/coaxiality evidence (§19, §20), not a confirmed thrust vector. `THRUST_AXIS_REQUIRES_CONFIRMATION`.
9. **Mechanical motor-mount/pylon/strut geometry** — no such part exists among the 12 mesh files; motor-to-wing/body attachment interface is unmodeled. `DATA_REQUIRED`.
10. **Neutral (undeflected) pose confirmation for aileron/elevator/rudder** — the as-exported mesh pose is the only pose available; it is not confirmed to represent a defined neutral position. `DATA_REQUIRED` (§16).
11. **CAD-named reference points** (e.g. "wing root station," "firewall," "spinner tip") tying mesh coordinates to engineering drawing callouts — bounding-box geometry alone does not supply named reference semantics. `DATA_REQUIRED`.
12. **Cause of the ≈17% mesh-vs-nominal propeller diameter discrepancy** (§21.2) — reported as a raw `GEOMETRIC_CHECK` finding; no cause is asserted. `DATA_REQUIRED` (for `propulsion`/project-owner follow-up, outside this agent's ownership boundary to resolve). **Not a blocker for propulsion work**: regardless of cause, the mesh-derived ≈273 mm figure is reclassified `VISUAL_MESH_ONLY` (§20, §21.2, this task, 2026-08-21) and must never be used in physics calculations — the real physical diameter to use is the APC 13x6.5E's D = 0.3302 m.

None of these are estimated, guessed, or filled with placeholder numeric values anywhere in this document.

---

## 31. Validation Findings

**Reviewer:** `validation` (independent, tool-based re-derivation — not a read-through review).
**Scope reviewed:** §12–§31 (full mesh geometric analysis pass).
**Overall verdict: sound.**

Validation independently re-parsed all 12 STL files from scratch (rather than trusting figures already in this document) and re-derived every numeric claim it checked, including: all 12 bounding boxes, all 5 left/right symmetry pairs, the wingspan `GEOMETRIC_CHECK` (§21.1), both propeller-diameter calculation methods (§20, §21.2), the 6-band aileron hinge-candidate table, body-slice evidence, body argmax/argmin vertices, and the FLU-compatibility axis assignment. All of these matched this document to float32 precision.

**CRITICAL findings:** none.

**MAJOR findings:** none.

**MINOR findings:**
- **MINOR-1** (fixed by this same closeout pass, no longer open): §21.2 reported the mesh-vs-nominal propeller diameter percentage difference as "≈−17.1%". Validation independently recomputed the same figure from the mesh-derived diameter range (0.2734–0.2735 m) against the APC 13x6.5E nominal (0.3302 m) and obtained −17.17% to −17.19%, which rounds to **−17.2%**, not −17.1%. Corrected in §21.2 as part of this closeout; no other value in the document depended on the incorrect figure, so no downstream correction was required.

**INFO findings:**
- **INFO-1**: The body-slice X-range notation in §25 has a slice-vertex-count traceability ambiguity — cosmetic only, not a substantive numeric defect. Optionally tighten the notation in a future pass; not required now.

**Explicitly confirmed by validation (no violation found):**
- No CG-duality violation anywhere in the document — the Gazebo/CAD CG and the XFLR5 reference CG are never conflated; the single CG usage in §19 is explicitly labeled an unconfirmed-assumption plausibility check only, not an authoritative geometric claim.
- Unit handling is honest: the millimeter interpretation of mesh coordinates is confined to explicitly-labeled comparison sections (§21.1, §21.2) and is never silently substituted for meters elsewhere in the document.
- The bounding-box-center-vs-CG distinction is consistently maintained throughout.
- Every hinge-related claim is correctly tagged `HINGE_REQUIRES_CONFIRMATION`, with no SDF-ready hinge axis asserted as settled.
- The elevator/rudder component-scope question remains genuinely open, tagged `COMPONENT_SCOPE_REQUIRES_CONFIRMATION`, framed as "evidence suggests" rather than a settled conclusion.
- The motor thrust axis is correctly left `THRUST_AXIS_REQUIRES_CONFIRMATION`.
- The provenance triplet format (Source / Method / Status) is consistent across the spot-checked sections.
- No unconfirmed value (e.g. the millimeter-interpretation wingspan figure, or the "fixed stabilizer/fin in body.stl" evidence) was found being silently reused downstream as if it were settled fact.

**File-scope / ownership-boundary check:** confirmed via `git status` and STL file modification times that only `GEOMETRY.md` and `README.md` were modified in the mesh-analysis pass. No `.stl` file was altered, moved, or renamed. No SDF, joint, or inertia file was created.

**Note added 2026-08-21 (master-dataset synchronization pass) — this note is additive, the §31 validation record above is left unedited as the historical record of that specific review:** the component-scope question that this validation pass explicitly found "genuinely open" has since been resolved — see §26.3. The resolution was reached using the same mesh evidence this validation pass reviewed and confirmed as sound, combined with new master-dataset evidence; it does not contradict or require revisiting this validation pass's findings about the mesh-analysis arithmetic itself, which remain valid.

---

## 32. Master Dataset Synchronization Pass (2026-08-21)

**Scope:** docs-only. No `model.sdf`, plugin, world, or launch file created or modified; no `.stl` file under `model/meshes/` modified (read-only, per task instruction). Source: `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` (2247 lines, §1–§74), read directly in full, not from any summary. Source-priority order applied throughout: manufacturer manual > real aircraft measurement > real component manufacturer data > current STL geometry > XFOIL/XFLR5 result > derived calculation > V1 estimate/provisional. The master dataset's own status qualifiers (confirmed/final, V1, provisional, approximate/"yaklaşık", estimate, validation target, V2-future-improvement) are preserved — no V1/provisional value is promoted to final/`CONFIRMED` status in this document beyond what the master dataset itself claims for it.

This section is a navigation index only — the substantive content, derivations, and cross-validation arithmetic all live in the topically-relevant sections listed below (added or updated in this pass); nothing here duplicates them at length.

| # | Topic | Where the full content lives | Outcome |
|---|---|---|---|
| 32.1 | STL coordinate scale, formalized as a named constant | §13.1 | `STL_SCALE_TO_SI = 0.001`, status `DERIVED_WITH_STRONG_EVIDENCE` (explicitly not `CAD_CONFIRMED`) |
| 32.3 | Gazebo/CAD ↔ XFLR5 coordinate transform (numbered 32.3, not 32.2, to match the cross-reference anchor `§32.3` used throughout §8.3/§13.1/`MASS_PROPERTIES.md`) | §8.3 (full derivation + 2 cross-validations); restated in `MASS_PROPERTIES.md` §3.5 | `DERIVED` for X/Z (`XFLR5_X = 0.23196 - STL_X`, `XFLR5_Z = STL_Z - 0.12103`); Y-axis and rotation-beyond-X-reversal remain `DATA_REQUIRED` |
| 32.4 | Manufacturer wing/tail planform (incidence, dihedral, sweep, root/tip chord, AR, airfoil identities) | §5 | `CONFIRMED` (manufacturer manual, master dataset §3) for the items now filled in; tail incidence remains `DATA_REQUIRED` |
| 32.5 | Horizontal/vertical tail placement | §5 | Horizontal tail: `CONFIRMED` in both Gazebo/CAD and XFLR5 frames. Vertical tail: `CONFIRMED` in XFLR5 frame only; Gazebo/CAD-frame root point remains `DATA_REQUIRED` (no independent STL reference point stated for it in the master dataset, unlike the horizontal tail) |
| 32.6 | Hinge geometry — elevator, rudder, aileron | §6 | `HINGE_GEOMETRY_READY` for all three (real position/x-c data now in hand); SDF axis-fit + sign-test explicitly still pending, not asserted as done |
| 32.7 | Component-scope resolution — elevator/rudder | §26.3 (reasoning), with pointers updated at §4.3, §17, §18, §24, §30 | Resolved: `left_elevator.stl`/`right_elevator.stl`/`rudder.stl` = movable-surface-only; `body.stl` carries the fixed tail structure. Judged-small-but-nonzero residual ambiguity stated explicitly, not CAD-confirmed |
| 32.8 | Motor/propeller physical hub/shaft reference vs. raw mesh bounding-box center | §7 | Prop hub: `CONFIRMED`, tightly corroborated (≤0.03 mm) by independent mesh bounding-box analysis. Motor center: `CONFIRMED` (master dataset value) but explicitly **not** the same number as this document's own motor-mesh bounding-box center (≈7.28 mm apart in X) — both recorded, neither discarded |
| 32.9 | Main battery position | `MASS_PROPERTIES.md` §6.1 (primary content, mass-properties' natural home); pointer in this document's §5 qualitative-facts table | `CONFIRMED`: (0.300631, 0, 0.038547) m, CG-relative ΔX≈+0.132322 m (forward), ΔZ≈−0.061453 m (below). Secondary 3S battery position: `DATA_REQUIRED`, not invented |
| 32.10 | Cross-check of existing STL bounding-dimension figures against master dataset §11 | §21.3 | All 7 cross-checked components match to the master dataset's own reported precision; no discrepancy found |
| 32.11 | Updated consolidated `DATA_REQUIRED`/resolved-item ledger | §11 (updated in place, `[RESOLVED]`/`[PARTIALLY RESOLVED]` tags added, nothing deleted), §30 (items 4–6 updated) | See those sections for the full itemized list |

**Inertia** (master dataset §9: V1 provisional tensor) is **not** a geometry-file item — it is documented in `docs/source_of_truth/mass_properties/MASS_PROPERTIES.md` §5, per this repository's existing file-ownership split (this document cross-references it rather than duplicating the tensor here, consistent with §1 of this document).

**What this pass explicitly did not do:** it did not create, modify, or touch `model.sdf`, any plugin source, any world/launch file, or any ArduPilot config file (out of scope per task instruction). It did not modify any `.stl` file (read-only per task instruction and per `CLAUDE.md`'s "no mesh modification without authorization" rule). It did not fit a single SDF joint axis for any control surface, and it did not perform a Gazebo joint-sign unit test — both remain explicitly open (`HINGE_GEOMETRY_READY`, §6). It did not change the aircraft's mass, CG, or any aerodynamic coefficient. Where the master dataset's own wording carried a "yaklaşık" (approximate)/V1/provisional qualifier, that qualifier is preserved in this document's status tags rather than silently upgraded.

---

## 33. First Gazebo Structural Implementation Pass (2026-08-21)

**Scope:** `model/model.sdf` and `model/model.config` created for the first time — the first physical skeleton of FALCON V2 in Gazebo Sim Harmonic. Structural only: links, joints, mass/CG/inertia placement, visual mesh placement, collision primitives. No aerodynamic, propulsion-force, or control-actuation model; no plugins. Validated with `gz sdf --check` (`Valid.`) and `gz sdf --inertial-stats` (see §33.7). No `.stl` file modified. No mass, CG, or inertia *source value* changed from what was already documented in §5–§9/§32 and `MASS_PROPERTIES.md` §3/§5 — this section records how those already-documented values were *placed into SDF*, and the new hinge-axis-fitting/collision/mass-split engineering work performed to make that placement possible.

### 33.1 Hinge-axis line fits — method, data, and residuals

**Revised 2026-08-22 (post-`validation` MAJOR-1 correction pass).** The original version of this subsection (written 2026-08-21) reported a hinge-line fit whose **primary/chordwise (X) component was independently re-confirmed correct by `validation`**, but whose **secondary (tilt) component did not reproduce** under `validation`'s independent re-implementation of the stated method, and whose characterization of the rudder's lateral (Y) position as "noise" was found not to hold up. Root cause, corrected method, corrected values, and newly-reported secondary-axis residuals are documented in full below — nothing from the original pass is silently dropped; the discrepancy and its resolution are recorded explicitly.

Per-station %chord data already existed (`HINGE_GEOMETRY_READY`, §6) but had not been fitted into a single SDF-ready joint axis. This pass performed that fit directly from the STL mesh vertex data (read-only re-parse, not a re-derivation of the %chord figures themselves).

**Root cause of the original non-reproducibility (diagnosed this pass):** the original method took the mesh's forward-most (max-X) vertex within a ±3 mm station band, then **averaged every vertex within a fixed radial tolerance (0.5 mm) of that maximum** to estimate the secondary-axis (Y or Z) coordinate. Direct re-inspection of the raw vertex data (band-width/tolerance sensitivity scan, all three surfaces) showed the true "hinge cut" at a given station is not always a single point — it can be a short near-vertical/near-planar edge, or (for the rudder specifically) a genuine **mirror-symmetric pair** of distinct vertices (the thin skin's left and right surfaces meeting almost, but not exactly, at the same X). A fixed-radius average over such a cluster pulls in a triangulation-density-dependent, tolerance-dependent mix of nearby-but-distinct points, which is why the secondary axis was unstable under re-implementation even though the primary (X) axis — which changes slowly and smoothly with span — was not visibly affected by the same instability.

**Corrected method:** for each control surface, at each master-dataset-documented span/height station, the mesh is sliced into a narrow band (±3 mm) centered on that station. Within each band: (1) find the single greatest X value in the band, `xmax`; (2) collect every vertex within 0.001 mm of `xmax` — this is a **deterministic, exactly-reproducible tie-breaking rule**, and at this sub-micron tolerance the collected vertices are true float32-duplicate copies of the same physical point(s), an artifact of binary STL's per-triangle unshared vertex storage (each triangle stores its own copy of every vertex it touches, so a single physical point shared by *n* triangles appears *n* times in the raw vertex stream); (3) round each collected vertex to 3 decimal mm and take the **unique** spatial points (not multiplicity-weighted — the number of triangles referencing a given physical point varies with local mesh topology and must not bias an average); (4) average those unique point(s) to get that station's (X, Y, Z). The station's own independent (regression) variable is taken as this point's **own actual coordinate** (Y for aileron/elevator, Z for rudder), not the nominal master-dataset target — e.g. the nominal "y=70 mm" station's true extremal vertex sits at y=72.616 mm, a few mm off nominal, which is expected since a real mesh feature within a search band is being located, not assumed to sit exactly at the band center. A 2D least-squares line (X, and the secondary axis, each regressed linearly against the station's own actual coordinate) is then fit through these points. This is the same edge already identified, from independent bounding-box/thickness evidence, as the physical hinge cut (§16–§18: chordwise thickness increases monotonically to this boundary with no full-airfoil thickness peak — a partial-chord section cut at a hinge line, not a complete airfoil).

**Boundary-station exclusion (elevator, rudder) — reconfirmed under the corrected method:** the two stations sitting exactly at the movable surface's own mesh Y/Z boundary (elevator y=50.6/240 mm; rudder z=130.5/299.0 mm) both independently return X = −472.684 mm — exactly the mesh's own global bounding-box max-X vertex — at *both* the root and tip band, roughly 2 mm off the smooth trend defined by the interior stations. This reproduces under the new deterministic method (it is not an artifact of the old tolerance-average method), confirming it as a real end-cap/boundary-face contamination artifact (the flat face closing the mesh at its own Y/Z extremity), not the true local hinge chord position. These two raw samples per surface are reported but excluded from the fit.

**Aileron tip-station exclusion — new finding this pass (secondary/Z axis only):** station y=784.875 mm's extremal point (actual coordinates Y=782.582, Z=128.564 mm) is a local outlier roughly 12 mm above the smooth Z(Y) trend established over Y=700–780 mm (which rises smoothly from Z≈115.8 to Z≈116.9 mm across that range — checked via a dedicated band-width sensitivity scan at multiple Y centers). Station y=784.875 sits only 4.865 mm from the aileron mesh's own tip boundary (Y=789.740 mm, §16), consistent with this being an isolated tip-cap/closing-face artifact vertex, not part of the smooth hinge line. Critically, this station's **primary (X) value (33.396 mm) is not similarly affected** — it stays fully consistent with the smooth X(Y) trend — so the aileron's X-fit still safely uses all 4 given stations, while its Z-fit (secondary/tilt axis) uses only the 3 non-tip stations (313.950, 470.925, 627.900 mm), with the exclusion and its evidence stated explicitly rather than silently dropped.

**Rudder lateral (Y) position — corrected characterization:** the original pass characterized per-station Y values as "±0.1–0.3 mm noise consistent with Y=0" — re-investigation under the corrected method shows this was an oversimplification of a more specific and stronger finding, not a wrong conclusion. At every interior station, the true extremal-X point is a **mirror-symmetric pair of two distinct vertices** (Y ≈ +d and Y ≈ −d, essentially identical X and Z — the hinge cut face's two skin surfaces, separated by the rudder's own thin Y-thickness of ≈8–9 mm, meet at almost exactly the same X). Averaging the two **unique** points of each pair gives **Y = 0.0000 mm exactly, at all 5 of 5 interior stations** — a direct, exact, per-station geometric result, not an assumption or a "noise is small so we forced it to zero" simplification. This also explains why two different imperfect methods produced two different-looking wrong answers for Y: the original tolerance-average method (this document's first version) picked up an uneven, triangulation-density-biased mix from both sides of the pair and got small-but-nonzero values (a few tenths of a mm); a naive single-vertex argmax with no tie-breaking rule (independently tried by `validation` during its review) arbitrarily picks *one* member of the tied pair and gets a value close to that member's own ±d (of the order 3.6–4.4 mm, matching the per-station |d| values found here) — both are symptoms of the same unhandled tie, not evidence against Y=0.

**Residuals — PRIMARY axis (X for all three surfaces), corrected deterministic method:**

| Surface | Stations used (mm) | RMS residual | Max residual |
|---|---|---|---|
| Elevator | 4 interior: y=70, 130, 190, 220 (y=50.6, 240 excluded, boundary artifact) | 0.0096 mm | 0.0153 mm |
| Rudder | 5 interior: z=145, 180, 215, 250, 285 (z=130.5, 299.0 excluded, boundary artifact) | 0.0263 mm | 0.0349 mm |
| Aileron | 4: y=313.95, 470.93, 627.90, 784.88 | 0.0123 mm | 0.0204 mm |

**Residuals — SECONDARY/tilt axis (Z for elevator/aileron, Y for rudder), reported for the first time this pass, per `validation`'s explicit request:**

| Surface | Stations used (mm) | Fitted slope | Tilt angle | RMS residual | Max residual |
|---|---|---|---|---|---|
| Elevator, Z(Y) | 4 interior: y=70, 130, 190, 220 | dZ/dY = −0.002072 | ≈ −0.12° | 0.0003 mm | 0.0004 mm |
| Rudder, Y(Z) | 5 interior: z=145, 180, 215, 250, 285 | dY/dZ = 0 (exact, by mirror-pair construction) | 0° | 0.0000 mm | 0.0000 mm |
| Aileron, Z(Y) | 3 non-tip-outlier: y=313.95, 470.93, 627.90 (y=784.88 excluded, tip artifact) | dZ/dY = +0.014147 | ≈ +0.81° | 0.0020 mm | 0.0028 mm |

For traceability: the original (incorrect) pass had reported dZ/dY ≈ −0.022208 (≈ −1.27°) for the elevator — roughly an order of magnitude larger than the corrected −0.002072 (≈ −0.12°), and now understood to have been a tolerance-average artifact, not a real feature. It had reported dZ/dY ≈ +0.019252 (≈ +1.10°) for the aileron, computed by including the tip-outlier station — the corrected, outlier-excluded value is +0.014147 (≈ +0.81°). `validation`'s own independent rough estimate for the aileron (≈0.025–0.036, noted by `validation` itself as varying with "band-width/vertex-selection") is consistent with that same tip-proximity artifact affecting a less-deterministic re-derivation, not a materially different underlying geometry.

All three PRIMARY (X) fits remain sub-0.04 mm RMS — effectively a straight line in each case, quantifying and confirming the prior qualitative "nearly straight, mildly swept" characterization (§17/§18/§27). The SECONDARY (tilt) axis is now also reproducibly fit, with residuals reported for the first time, rather than asserted from an unstable, tolerance-dependent average. **Status: `DERIVED_FROM_MESH`, method (including the tie-breaking rule) and residuals for both axis components fully reported.** This is a materially stronger evidentiary basis than the pre-fit `HINGE_GEOMETRY_READY`/`HINGE_REQUIRES_CONFIRMATION` state, but the **Gazebo joint-rotation sign** is still explicitly **not** resolved by this fit (see §33.1.1) — fitting the axis *direction* (a line has no inherent sign) is a distinct question from which physical rotation a positive Gazebo joint command produces.

**Fitted hinge lines (Gazebo/CAD frame, meters, left side; right side is an exact Y-mirror — see §33.1.2):**

| Surface | Link/joint origin (root of movable span) | Axis unit vector (dY or dZ = +1 sense) |
|---|---|---|
| Left aileron | (0.032943, 0.313950, 0.110356) | (0.000994, 0.999899, 0.014146) |
| Left elevator | (−0.474959, 0.050600, 0.087119) | (0.002983, 0.999993, −0.002072) |
| Rudder | (−0.476094, 0.000000, 0.130500) | (0.010378, 0.000000, 0.999946) |

Changes from the original (incorrect) pass are small for X/Y in absolute terms (link-origin X shifted ≤0.02 mm for rudder, ≤0.001 m for aileron/elevator; Y unchanged) but material for Z (elevator origin Z shifted ≈1.1 mm; aileron origin Z shifted ≈0.5 mm) and for the axis direction vectors' secondary component (see the tilt-angle table above) — small in absolute magnitude (sub-degree to ~1°), which is why `validation` rated this MAJOR (a real provenance/reproducibility gap on a load-bearing constant) rather than CRITICAL (nothing currently consumes these axes for force/moment computation).

### 33.1.1 Sign convention — explicitly not resolved

Per `CLAUDE.md`/master dataset §72 ("XFLR5 control sign ile Gazebo joint sign aynı varsayılmamalı"), the axis vectors above define a rotation **axis**, not a rotation **sign**. Which physical deflection direction (e.g. trailing-edge-up vs. -down) a positive Gazebo joint angle produces is `SIGN_TEST_REQUIRED` for all three surfaces (`AILERON_SIGN_TEST_REQUIRED` / `ELEVATOR_SIGN_TEST_REQUIRED` / `RUDDER_SIGN_TEST_REQUIRED`, `CONTROLS.md` §4) — to be resolved by `gazebo-testing`'s `AILERON_TEST`/`ELEVATOR_TEST`/`RUDDER_TEST` once an actuator exists. Not asserted or guessed here.

### 33.1.2 Left/right mirror symmetry check

Aileron and elevator right-side hinge lines were derived by an exact Y-mirror of the left-side fit (right Y = −left Y; right dX/dY = −left dX/dY; right dZ/dY = −left dZ/dY), rather than by an independent right-side re-fit — justified by the already-`CONFIRMED` exact Y-mirror symmetry of the underlying meshes (§22: deltas ≤0.052 mm for every L/R pair). Verified programmatically on the final `model.sdf`: link-origin Y-mirror delta and joint-axis mirror delta are both exactly `[0, 0, 0]` for both the aileron and elevator pairs (by construction, since the right-side numbers were computed via the mirror formula, not independently re-fit). An independent right-side STL re-fit, as an optional cross-check, is left for `validation`. Rudder has no left/right pair (single centerline part, §18).

### 33.2 Mesh-pose strategy — decision

**Chosen: (B) for all 7 movable links, (A) for all fixed `base_link` visuals**, exactly as the task's guidance anticipated:

- **Fixed `base_link` visuals** (body, left/right wing, left/right motor): strategy (A) — link origin = model origin = (0,0,0), mesh kept at its own raw global/assembly coordinates, only `STL_SCALE_TO_SI = 0.001` applied, no additional visual `<pose>` offset. No local rotation axis is needed for a fixed part, so there is no reason to re-origin it.
- **Movable links** (aileron ×2, elevator ×2, rudder, prop ×2): strategy (B) — link origin placed at the physical hinge/hub point (§33.1 for control surfaces; the `CONFIRMED` prop hub, §7, for the props), with the mesh's `<visual>`/`<collision>` `<pose>` offset by exactly the negative of that same point (in meters), so that `link_origin + mesh_offset` reproduces the mesh's own authored global position with no visual shift. Verified numerically for every movable link (see `model/model.sdf` per-link header comments); also independently verified via `gz sdf --check` (loads without error) and by construction (the offset is computed as `-1 × origin`, an exact algebraic inverse, not an approximation) for all links except the propellers, where an additional visual-only scale factor is applied — see §33.5 for that specific verification.

### 33.3 Collision strategy — decision, full box inventory

No raw STL mesh is used as collision geometry anywhere in `model.sdf` (primitives only), per task instruction. Full inventory, with provenance for every box:

| Component | Box(es) | Provenance |
|---|---|---|
| Fuselage (3 boxes) | fwd (nose+wing-root band): center (0.249684,0,0.165240) m, size (0.554392,0.189298,0.330480) m | X-bounds from confirmed body/wing mesh bounds (§13); Y=±0.094649 m = `CONFIRMED` wing-root-station body slice (§25); Z = full confirmed body range (conservative, no nose-specific Z slice exists) |
| | mid: center (−0.250098,0,0.165240) m, size (0.445172,0.560000,0.330480) m | X-bounds as above; **Y=±0.280000 m is `ASSUMPTION`** — no mid-fuselage slice exists in the source of truth; the wider, confirmed tail-band half-width is used as a deliberately conservative (oversized, never undersized) stand-in |
| | tail: center (−0.499782,0,0.165240) m, size (0.054196,0.560000,0.330480) m | X-bounds from confirmed body/tail mesh bounds; Y=±0.280000 m = `CONFIRMED` tail-station body slice (§25); Z = full confirmed body range (body's own global max Z occurs in this X-band, §14.1) |
| Horizontal/vertical tail (fixed portions) | *(none — see reasoning)* | Not given a separate primitive: per §26.3 the fixed h-stab/v-fin structure is part of `body.stl`, and the fuselage tail box above already envelops that region (Y=±0.280 m exceeds the elevator's 240 mm half-span; Z up to 0.330480 m exceeds the rudder's 299.75 mm max height) — a second primitive there would duplicate collision volume |
| Left/right wing | 1 box each: center (0.105743,±0.566014,0.127019) m, size (0.266510,0.972027,0.046956) m | Each wing's own full mesh bounding box (§13/§15). Deliberately coarse — a constant-chord box is oversized outboard relative to the real tapered planform (root chord 0.260 m vs. tip chord 0.051 m); a simplification, not a tapered/multi-segment fit |
| Left/right motor | 1 box each: center (0.269577,±0.299975,0.126974) m, size (0.066484,0.038225,0.038237) m | Each motor's own full mesh bounding box (§13/§19). Not explicitly required by the task's named collision list (fuselage/wing/h-tail/v-tail); added as a low-cost completeness extension |
| Aileron / elevator / rudder (each, on its own link) | 1 thin box each, matching the part's own mesh bounding-box size, expressed in the link's own local frame (offset from the hinge-point link origin) | §13/§16–§18 mesh bounding boxes; local-frame offset computed the same way as the visual-mesh offset (§33.2) |
| Propellers | *(none — deliberate)* | A rotating thin disk at high RPM has no current physical use-case in this structural-only pass (no ground-contact/self-collision scenario involves it specifically); adding one risks spurious/self-collision artifacts once a propulsion plugin exists. May be revisited once propulsion/ArduPilot integration exists |

`self_collide` is set `false` at the model level: several movable-surface meshes intentionally abut/slightly overlap their parent structure at the hinge line (e.g. aileron vs. wing cutout, §16) — enabling self-collision would produce spurious contact forces there with no benefit for this structural-only pass.

### 33.4 Mass-distribution strategy — decision (cross-referenced from `MASS_PROPERTIES.md`)

**Hybrid of (A) and (B)**, not a pure (B), chosen and fully documented in `MASS_PROPERTIES.md` §7 (new section, this pass) and in `model/model.sdf`'s header comment — summarized here for cross-reference only, not duplicated in full. In brief: `left_prop`/`right_prop` get their real `CONFIRMED` component mass (0.0301 kg each, master dataset §44); the 5 control-surface links get a `TEMPORARY_NUMERICAL_MASS` of 0.001 kg each (no component-level mass data exists for them); `base_link` carries the remainder, 5.9348 kg, so total mass across all 8 links is exactly 6.000 kg (verified: `gz sdf --inertial-stats` reports `Total mass of the model: 6`).

**Known, documented limitation:** `base_link`'s `<inertial><inertia>` uses the `V1_PROVISIONAL` whole-aircraft tensor (master dataset §9/§70) **unmodified**, while `base_link`'s own mass (5.9348 kg) no longer exactly matches the tensor's original 6.000 kg documentation basis. This was a deliberate choice, not an oversight — see `model/model.sdf` header comment for the full reasoning (rigorously subtracting a parallel-axis point-mass contribution for the 7 child links would fabricate a precision the externally-supplied V1 tensor does not have). **Quantified consequence**, measured via `gz sdf --inertial-stats` on the actual final `model.sdf`: the multi-link system's aggregate mass-weighted CG is (0.169196, 0, 0.100291) m versus the documented Gazebo/CAD CG (0.168309, 0, 0.100000) m — a shift of ≈0.89 mm in X and ≈0.29 mm in Z, well inside the manufacturer's own ≈±10 mm CG tolerance (master dataset §3). The aggregate moment-of-inertia matrix is Ixx=0.735117, Iyy=0.253645, Izz=0.961294, Ixz=0.0147044 (kg·m²) versus the documented base tensor 0.7284/0.2507/0.9523/0.01485 — differences of roughly 0.9–1.5%, attributable entirely to the small (≈1.09% of total mass) known point masses on the movable links pulling the aggregate figures slightly via the parallel-axis effect. This is flagged for `validation`, not silently treated as exact.

### 33.5 Visual-only propeller scale — decision and isolation verification

**Decision: applied.** An additional 1.208× visual-only enlargement (`0.001 × 1.208 = 0.001208`) is applied to the propeller `<visual><mesh><scale>` only, so the rendered model displays at a realistic size (STL mesh propeller diameter ≈0.2734 m vs. real APC 13x6.5E D=0.3302 m, §20–§21.2). Reasoning: no physics in this structural-only pass consumes propeller mesh geometry at all (no propulsion plugin exists), so there is zero risk of this scale leaking into physics; it improves visual/inspection fidelity for the project owner and future specialists at no cost.

**Isolation:** the scale is applied only inside `left_prop_visual`/`right_prop_visual`'s `<mesh><scale>` element. No `<collision>` exists for either propeller link (§33.3), so there is no collision geometry this scale could contaminate. `PROPULSION.md` §0 and the `model.sdf` header comment both restate, in loud terms, that any future propulsion/physics code must use the real D=0.3302 m constant directly, never this visual scale factor or the resulting ≈0.2734 m mesh dimension.

**Pivot-point math (so the enlarged disk stays centered on the real hub, not the mesh's own distant local origin):** a naive uniform `<scale>` multiplies every raw mesh vertex coordinate, which would also inflate the mesh's *position* relative to the shared assembly origin (the mesh's local (0,0,0) sits near the fuselage nose region, far from the propeller itself) — using `0.001208` with no compensating offset would shift the rendered hub roughly 60 mm from its true position. The `<visual><pose>` translation was therefore computed as `T = -scale_ratio × hub_position_m` (`scale_ratio = 1.208`), which scales the mesh geometry by 1.208× **about the hub point** rather than about the mesh's own local origin:

```
left_prop:  T = -1.208 × (0.2951, 0.3000, 0.1271) = (-0.356481, -0.362400, -0.153537) m
right_prop: T = -1.208 × (0.2951, -0.3000, 0.1271) = (-0.356481, 0.362400, -0.153537) m
```

Verified numerically: applying this scale+offset to the `left_pervane.stl` mesh's own bounding-box-center vertex (295.084, 300.007, 127.073 mm raw) reproduces (0.29508, 0.30001, 0.12707) m — within 0.1 mm of the true hub (0.2951, 0.3000, 0.1271) m. `Status: DERIVED, verified by direct computation, not asserted without checking.`

### 33.6 Self-check results (performed this pass)

All checks below were run against the final `model/model.sdf`, not asserted from memory:

| Check | Method | Result |
|---|---|---|
| XML/SDF syntax validity | `gz sdf --check model/model.sdf` | `Valid.` |
| Mesh URIs resolve | Programmatic parse of every `<uri>`, checked against `model/meshes/` | All 12 mesh files resolve |
| Total mass = 6.000 kg exactly | `gz sdf --inertial-stats` + independent per-link summation | `Total mass of the model: 6` (exact); per-link sum = 6.000000000000002 (float rounding only) |
| CG consistency | `gz sdf --inertial-stats` aggregate CG vs. documented Gazebo/CAD CG | Aggregate (0.169196,0,0.100291) m vs. documented (0.168309,0,0.100000) m — ≈0.89/0/0.29 mm delta, quantified and flagged, §33.4 |
| Inertia positive-definiteness (all 8 links) | Eigenvalue decomposition of each link's 3×3 inertia matrix | All 8 links positive-definite (smallest eigenvalue > 0 in every case) |
| Inertia symmetry | By construction (Ixy/Ixz/Iyz entered directly, matrix built symmetric) | Symmetric by construction for all 8 links |
| Link/joint graph validity (no disconnected movable link) | `gz sdf --graph pose` and `--graph frame` | All 7 movable links attach to `base_link` via their joint; no orphaned link |
| Hinge left/right symmetry | Programmatic mirror-delta check (link pose + joint axis) | Exact `[0,0,0]` delta for both aileron and elevator L/R pairs (right side constructed as an exact mirror, §33.1.2) |
| Prop hub left/right symmetry | Direct comparison of link `<pose>` | Y-mirror exact by construction: (0.2951,±0.3000,0.1271) |
| No duplicate visual/collision names | Programmatic per-link name-uniqueness check | No duplicates found on any of the 8 links |
| No double-counted mass | Manual accounting, §33.4 + `model.sdf` header comment | Motor mass (0.286 kg) stays in `base_link` only, not duplicated; propeller mass (0.0602 kg) subtracted from `base_link` exactly once, matching its assignment to `left_prop`/`right_prop` |
| Joint axis vectors are unit vectors | Programmatic norm check on all 7 `<axis><xyz>` | All 7 norms = 1.000000 |
| Joint parent/child references valid | Programmatic cross-check against the 8 declared link names | All 7 joints reference existing links |

### 33.7 `gz sdf --inertial-stats` raw output (for traceability)

```
Total mass of the model: 6
Centre of mass in model frame:
X: 0.169196
Y: 0
Z: 0.100291
Moment of inertia matrix:
0.735117     -2.1684e-19  0.0147044
-2.1684e-19  0.253645     1.35525e-19
0.0147044    1.35525e-19  0.961294
```

### 33.8 Residual open items after this pass (not resolved here, not silently dropped)

1. Control-surface joint rotation **sign** (§33.1.1) — `SIGN_TEST_REQUIRED` for all 5 control-surface joints; `gazebo-testing`/`controls-integration` own this next.
2. Propeller rotation **direction** (left CCW / right CW, master dataset §44) is documented in `model.sdf` comments only — not enforced by the structural joint itself (no propulsion plugin exists to command a signed velocity yet).
3. Ixz sign convention vs. SDF's expected off-diagonal sign — still explicitly unverified (`MASS_PROPERTIES.md` §5.1); entered into `model.sdf` exactly as documented, flagged, not resolved.
4. The mid-fuselage collision box's Y half-width (§33.3) is `ASSUMPTION`-tagged, not measured — a future slice measurement at that X-band would let it be tightened.
5. The known base_link-mass-vs-inertia-tensor-basis inconsistency (§33.4) is flagged for `validation`, not resolved.
6. No propulsion, aerodynamic, or control-actuation plugin exists — `model.sdf` is structure only, exactly as scoped.

# FALCON V2 — Mass Properties Source of Truth

**Owner:** `geometry-structure`
**Status:** Documentation-only compilation. STL mesh export files now exist (see `docs/source_of_truth/geometry/GEOMETRY.md` §4); no mass-property-bearing CAD file, SDF file, or measured mass data exists in this repository as of this writing. File presence does not equal mass-property knowledge — inertia, component masses, and all other values not listed in §1 remain `DATA_REQUIRED` throughout this document.
**Compiled:** 2026-08-21
**Last updated:** 2026-08-21 (consolidation pass: re-confirmed the CG-duality discipline in §3 was intact; made frame/origin/axis-convention/intended-usage explicit and separate for both the Gazebo/CAD CG and the XFLR5 reference CG, §3.1/§3.2; added an explicit designation statement, §3.4, naming the Gazebo/CAD CG as the value for the SDF `<inertial>` block with no XFLR5→Gazebo conversion applied. No numeric value changed; both CG origins remain `DATA_REQUIRED`, not resolved by this pass.)

**Last updated (again):** 2026-08-21 (master-dataset synchronization pass — source: `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt`, §1–§74, read in full, cited by section number; source-priority order applied: manufacturer manual > real aircraft measurement > real component manufacturer data > current STL geometry > XFOIL/XFLR5 result > derived calculation > V1 estimate/provisional; the master dataset's own status qualifiers, e.g. "V1"/"provisional"/"yaklaşık", are preserved, not silently promoted). Three substantive changes this pass, all detailed in the sections they touch: (1) a V1-provisional inertia tensor is now documented at §5, replacing the prior "no inertia data of any kind" finding — it does not block Gazebo V1 use, per the master dataset's own explicit statement; (2) main battery mass and center coordinate are now documented at §6.1 (secondary 3S battery position remains `DATA_REQUIRED`, not invented); (3) a derived Gazebo/CAD↔XFLR5 coordinate transform (X/Z only) is documented at §3.5, cross-referencing the full derivation in `GEOMETRY.md` §8.3. No mass or CG value was changed — only inertia (previously entirely absent) and battery mass/position (previously entirely absent) were added, and both are geometry/mass-distribution facts explicitly within this agent's ownership, not tuning changes.
**Repository investigation performed:** full-tree `find`, extension search (`*.sdf *.stl *.dae *.obj *.urdf *.xacro *.csv *.xlsx *.pdf *.xflr5 *.xfl *.step *.stp *.iges *.igs *.json *.yaml *.yml *.xml`), keyword `grep` for `ixx|iyy|izz|ixy|ixz|iyz|inertia|xflr5|xfoil|battery|motor mount|esc|servo|hinge|CAD` (case-insensitive) across the whole working tree, and full `git log`/`git ls-tree` history review (single commit, `1c2d17d`, matches the working tree exactly — no deleted or historical files carry additional mass-property data).

Status legend: `CONFIRMED`, `DERIVED` (derivation shown), `DATA_REQUIRED` (not found anywhere in repo), `CONFLICT_REQUIRES_RESOLUTION` (two authoritative sources disagree, both reported), `UNVERIFIED` (found but revision/applicability to current FALCON V2 cannot be confirmed — excluded from authoritative tables), `REVISION_REQUIRES_CONFIRMATION` (component value found but which aircraft revision it belongs to is ambiguous — excluded from authoritative tables).

`V1_PROVISIONAL` (introduced 2026-08-21, master-dataset synchronization pass): real, usable numeric data supplied as a working V1 input (e.g. to make an XFLR5 dynamic-stability analysis runnable, or as an interim engineering value) that the source itself explicitly labels as not final. Distinct from `DATA_REQUIRED` (nothing exists) and from `DERIVED`/`CONFIRMED` (would imply a settled, traceable-to-first-principles or authoritative value) — usable now, expected to be superseded later by a "V2" value, and never to be silently treated as final in the interim. See §5.1 for the inertia-tensor usage of this tag.

No `ASSUMPTION` entries appear in this document.

---

## 1. Authoritative Values

Summary of every value in this document that carries `CONFIRMED` status. Full detail and provenance for each is in the sections that follow.

| Parameter | Value | Unit | Reference frame | Status |
|---|---|---|---|---|
| Total aircraft mass | 6.000 | kg | — | CONFIRMED |
| Gazebo/CAD reference CG | (0.168309, 0.000000, 0.100000) | m | Gazebo/CAD reference | CONFIRMED |
| XFLR5 reference CG | (+0.0637, 0.0000, -0.0210) | m | XFLR5 reference | CONFIRMED |

**Update, 2026-08-21 (master-dataset synchronization pass):** the full inertia tensor (§5.1) and the main battery mass/position (§6.1) are no longer purely `DATA_REQUIRED` — both now carry real values at `V1_PROVISIONAL` (inertia) or `CONFIRMED` (battery, master dataset) status. Neither is added to the table above, which is reserved for values with unqualified `CONFIRMED` status from the original project-owner-provided set; `V1_PROVISIONAL` is a deliberately distinct, weaker status (see legend) and is kept out of this specific summary table so it is never mistaken for the same tier of authority as mass/CG. See §5.1 and §6.1 directly.

Everything else in this document — the CG-to-CG conversion beyond the derived X/Z transform (§3.5), remaining component mass distribution, and remaining frame origin definitions — is `DATA_REQUIRED`, `UNVERIFIED`, or `REVISION_REQUIRES_CONFIRMATION`; none of it is treated as authoritative.

---

## 2. Total Aircraft Mass

| Parameter | Value | Unit | Provenance | Status |
|---|---|---|---|---|
| Total aircraft mass (current configuration) | 6.000 | kg | project owner, direct conversation, 2026-08-21; `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED |
| Manufacturer MTOW rating | 7 | kg | project owner, direct conversation, 2026-08-21 | CONFIRMED |

The manufacturer MTOW (7 kg) is the airframe's rated maximum takeoff weight — a capability ceiling from the manufacturer, not a measurement of this aircraft's current mass. It is a different parameter from the 6.000 kg total aircraft mass and must not be substituted for it anywhere in the simulation. Both are recorded here so the distinction stays explicit; see also `docs/source_of_truth/geometry/GEOMETRY.md` §3.

Per `CLAUDE.md` simulation-tuning policy: mass must never be changed to alter simulation behavior, and no unauthorized change to mass is made or implied by this document.

---

## 3. Center of Gravity

Two CG values exist in this project, using **different, non-interchangeable reference definitions**. They are documented under fully separate headings below and must never be merged, averaged, or substituted for one another. See §4 for what is and is not known about each frame's physical origin, and `GEOMETRY.md` §8.3 for why no conversion between them is computed in this repository.

### 3.1 Gazebo/CAD Reference CG

| Axis | Value | Unit | Reference frame | Provenance | Status |
|---|---|---|---|---|---|
| X | 0.168309 | m | Gazebo/CAD reference | project owner, direct conversation, 2026-08-21; `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED |
| Y | 0.000000 | m | Gazebo/CAD reference | same as above | CONFIRMED |
| Z | 0.100000 | m | Gazebo/CAD reference | same as above | CONFIRMED |

**Frame / origin / axis / usage, stated explicitly for this CG value:**

| Property | Value | Status |
|---|---|---|
| Frame | Gazebo/CAD reference | CONFIRMED |
| Physical origin on airframe (what the (0,0,0) point corresponds to) | Not documented anywhere in the repository | `DATA_REQUIRED` (re-confirmed this task — see §4; not guessed) |
| Axis convention | FLU: +X forward, +Y left, +Z up | CONFIRMED (`CLAUDE.md`) |
| Intended usage | This is the CG value used for the Gazebo `model.sdf` `<inertial><pose>` block position, because it is already expressed in the Gazebo/CAD FLU frame that Gazebo/SDF operates in. See §3.4 for the full designation statement. | Stated here for clarity, not a new value |

### 3.2 XFLR5 Reference CG

| Axis | Value | Unit | Reference frame | Provenance | Status |
|---|---|---|---|---|---|
| X | +0.0637 | m | XFLR5 reference | project owner, direct conversation, 2026-08-21; `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED |
| Y | 0.0000 | m | XFLR5 reference | same as above | CONFIRMED |
| Z | -0.0210 | m | XFLR5 reference | same as above | CONFIRMED |

**Frame / origin / axis / usage, stated explicitly for this CG value:**

| Property | Value | Status |
|---|---|---|
| Frame | XFLR5 reference | CONFIRMED |
| Physical origin on airframe (what the (0,0,0) point corresponds to) | Not documented anywhere in the repository | `DATA_REQUIRED` (re-confirmed this task — see §4; not guessed) |
| Axis convention | Not documented anywhere in the repository (XFLR5's own internal convention is not necessarily FLU and must not be assumed to be) | `DATA_REQUIRED` (re-confirmed this task — see §4; not guessed) |
| Intended usage | This is the CG value that XFLR5-derived aerodynamic/stability reference data is referenced against — e.g. the full-aircraft reference-point quantities in `CLAUDE.md` (XNP = 0.132 m, XCP = 0.064 m, and the associated stability derivatives). It is owned/consumed by `aerodynamics` for XFLR5-frame aerodynamic work and must **never** be used for the SDF `<inertial>` block. | Stated here for clarity, not a new value |

### 3.3 Explicit warning (restated from `CLAUDE.md` and role definition)

- These two CG triples are **not** expressed in the same coordinate system. Their origins and axis conventions have not been documented anywhere in this repository (see §4).
- Do not subtract, average, or otherwise combine §3.1 and §3.2 as if they described the same point.
- Do not use §3.1 in place of §3.2 (or vice versa) in any XFLR5-derived aerodynamic calculation, stability derivative, or SDF `<inertial><pose>` field.
- A derived conversion between the two is only permitted once both frame origins are documented from source data (CAD file, XFLR5 project file) — see §9, item "Gazebo/CAD ↔ XFLR5 CG conversion."

### 3.4 Designated CG for the SDF `<inertial>` block (explicit statement, this task)

**The Gazebo/CAD CG (§3.1: 0.168309, 0.000000, 0.100000 m) is the value designated for the `model.sdf` `<inertial><pose>` block**, because it is already expressed in the Gazebo/CAD FLU frame that Gazebo/SDF operates in — no frame change is needed to use it directly as an SDF pose offset from the link origin.

**The XFLR5 reference CG (§3.2) is NOT used for this purpose**, and **no conversion from the XFLR5 CG to the Gazebo/CAD CG is applied, or needed, to produce this designation.** The Gazebo/CAD CG value is used directly, as-is, exactly as documented in §3.1 — it is not derived from, checked against, or reconciled with the XFLR5 CG. No origin-to-origin transform between the two reference frames exists in this repository (§3.3, §4, `GEOMETRY.md` §8.3), and this designation does not require one: the SDF-target CG is simply read from the frame that already matches the SDF's own coordinate system.

This designation is a documentation clarification only — it does not itself constitute SDF implementation (no SDF file is created or edited by this document) and does not change, guess, or supersede either CG value recorded in §3.1/§3.2.

### 3.5 Gazebo/CAD ↔ XFLR5 CG conversion — derived, 2026-08-21 (master-dataset synchronization pass)

**A specific, derived X/Z transform between the two frames now exists.** Full derivation and cross-validation arithmetic live in `GEOMETRY.md` §8.3 — not duplicated at length here; this subsection restates the result and its direct consequence for the two CG values in §3.1/§3.2.

```
XFLR5_X = 0.23196 - Gazebo/CAD_X   (meters)
XFLR5_Z = Gazebo/CAD_Z - 0.12103   (meters)
```

derived from master dataset §2 (X-axis sign reversal between the two frames), §8 (main-wing-root-LE reference point, shared origin), and §12 (XFLR5 Plane Editor set up at that same point, X=Z=Tilt=0). Applying it to the Gazebo/CAD CG (§3.1: 0.168309, 0, 0.100000 m) reproduces the documented XFLR5 CG (§3.2: 0.0637, 0, −0.0210 m) to within 0.00005 m in X and 0.00003 m in Z — this is, in effect, the master dataset's own §8 arithmetic, restated and independently cross-checked against a second, unrelated point (the horizontal-tail root LE, `GEOMETRY.md` §8.3, §20 of the master dataset) which also closes to sub-0.1 mm.

**Status: `DERIVED`** for the X and Z components specifically, replacing the prior blanket `DATA_REQUIRED` transform item (§9, item 5) for this specific pair of values.

**What this does NOT change or establish, restated from `GEOMETRY.md` §8.3 for this document's own context:**
- It does **not** license substituting §3.1 for §3.2 or vice versa anywhere in this document or downstream — §3.3's warning stands in full. The transform is a documented, derived *relationship* between the two values, not a license to treat them interchangeably without applying it.
- It does **not** resolve Y-axis behavior (untested — all three points used have Y=0) or any rotation beyond the stated X-axis sign reversal (§4 below, `GEOMETRY.md` §8.3 caveats 1–2).
- It does **not** establish a CAD-confirmed physical identity for either frame's origin in general terms — see §4.
- It applies only to CG/reference-point-style single coordinates; it is **not** a substitute for, and does not by itself transform, the inertia tensor (§5) — the inertia tensor is documented directly about the Gazebo/CAD CG (master dataset §9) and no transform of it into an XFLR5-referenced form is performed or needed here.

---

## 4. Reference Frames

| Property | Gazebo/CAD reference | XFLR5 reference |
|---|---|---|
| Axis convention | FLU (+X forward, +Y left, +Z up) — CONFIRMED, `CLAUDE.md` | Update 2026-08-21: **partially resolved** — X is reversed relative to Gazebo/CAD +X, Z is same-sense (master dataset §2); Y-axis behavior is untested (no Y≠0 cross-validation point available, `GEOMETRY.md` §8.3 caveat 1) and roll/bank alignment beyond the stated X-reversal is unconfirmed (§8.3 caveat 2) — DATA_REQUIRED for a complete axis-convention characterization |
| Physical origin on airframe | Not documented as a named CAD datum — DATA_REQUIRED | Update 2026-08-21: **partially resolved** — identified as the main-wing-root leading-edge point, Gazebo/CAD-frame (0.23196, 0, 0.12103) m (master dataset §8, §12); reproduces 2 independent cross-validation points to sub-0.1 mm (`GEOMETRY.md` §8.3) but is not independently CAD-confirmed as an exact named station — DATA_REQUIRED for that stronger sense |
| Units | meters (SI) — CONFIRMED, `CLAUDE.md` engineering rules | Not explicitly confirmed for this project's XFLR5 files — DATA_REQUIRED (though the §3.5 transform's sub-0.1-mm agreement is only consistent with both frames using meters, this is corroborating evidence, not a direct statement, and is not treated as a formal confirmation) |

`docs/source_of_truth/README.md` independently flags both origin/reference-frame definitions as `DATA_REQUIRED` — as of 2026-08-21 this is updated to reflect the partial resolution above (see `README.md` mass_properties/ bullets).

**Update, 2026-08-21 (master-dataset synchronization pass):** a specific X/Z transform between the two frames is now derived — see §3.5 for the full statement and `GEOMETRY.md` §8.3 for the complete derivation and cross-validation. This is narrower than "both origins are now fully documented" — it resolves the numeric X/Z relationship between this aircraft's two specific documented CG/reference values, using the specific evidence available, not a general CAD-level characterization of either frame. Y-axis behavior and any rotation beyond the stated X-reversal remain `DATA_REQUIRED`, carried forward as such, not filled by assumption.

---

## 5. Inertia Tensor

**Update, 2026-08-21 (master-dataset synchronization pass): a V1/provisional inertia tensor now exists, replacing the prior "no inertia data of any kind" finding.** The finding below (from the pre-master-dataset investigation) is preserved for history, followed by the new tensor and its explicit status.

**Prior finding (preserved, no longer current):** "no inertia data of any kind — confirmed, unverified, or otherwise — exists anywhere in this repository, including git history." The keyword search for `Ixx|Iyy|Izz|Ixy|Ixz|Iyz|inertia|inertia origin|inertia coordinate frame|CAD-computed inertia` returned zero matches outside of rule statements in `CLAUDE.md`, the two agent-definition files, and `docs/source_of_truth/README.md` — all of which stated that the inertia tensor was `DATA_REQUIRED`, and none of which contained any numeric inertia value. This remains true of the pre-master-dataset repository state; the tensor below comes from the newly-added master dataset, not from any file that existed at the time of that search.

### 5.1 V1 Provisional Inertia Tensor (master dataset §9)

| Component | Value | Unit | Associated mass | Associated CG / origin | Reference frame | Status |
|---|---|---|---|---|---|---|
| Ixx | 0.7284 | kg·m² | 6.000 kg (total aircraft, §2) | Gazebo/CAD CG (0.168309, 0, 0.100000 m, §3.1) | Gazebo/CAD (FLU) | `V1_PROVISIONAL` |
| Iyy | 0.2507 | kg·m² | 6.000 kg | Gazebo/CAD CG | Gazebo/CAD (FLU) | `V1_PROVISIONAL` |
| Izz | 0.9523 | kg·m² | 6.000 kg | Gazebo/CAD CG | Gazebo/CAD (FLU) | `V1_PROVISIONAL` |
| Ixy | 0 | kg·m² | 6.000 kg | Gazebo/CAD CG | Gazebo/CAD (FLU) | `V1_PROVISIONAL` |
| Iyz | 0 | kg·m² | 6.000 kg | Gazebo/CAD CG | Gazebo/CAD (FLU) | `V1_PROVISIONAL` |
| Ixz | 0.01485 | kg·m² | 6.000 kg | Gazebo/CAD CG | Gazebo/CAD (FLU) | `V1_PROVISIONAL` |
| Inertia reference origin | Gazebo/CAD CG (0.168309, 0, 0.100000 m) | — | — | — | Gazebo/CAD (FLU) | `V1_PROVISIONAL` (stated by master dataset §70 summary block, consistent with §9) |
| Inertia coordinate frame | Gazebo/CAD (FLU) | — | — | — | — | `V1_PROVISIONAL` |

**Provenance and method (master dataset §9):** these values were entered directly into XFLR5's Type-7 "Mean inertia" field with **`Use Plane Inertia = OFF`**. The master dataset explicitly records that `Use Plane Inertia = ON` produced zero inertia and eigenvalue problems in XFLR5 — i.e., this tensor was not computed by XFLR5 from the modeled geometry/mass distribution; it was supplied as a fixed external input to make XFLR5's stability/dynamic-mode analysis runnable (this is also the tensor implicitly used to produce the short-period/phugoid/Dutch-roll/roll/spiral mode results elsewhere in the master dataset, e.g. §25, §31). The master dataset does not state a first-principles derivation (e.g., a component-mass-distribution hand calculation) for these six numbers — they are recorded here exactly as given, with no re-derivation, re-scaling, or "correction" performed by this document.

**Status: `V1_PROVISIONAL`** (a new tag introduced this pass, deliberately distinct from `DATA_REQUIRED`, `DERIVED`, and `CONFIRMED`): this is real, usable numeric data — not a placeholder — but it is explicitly **not final**, per the master dataset's own repeated statement ("Final değildir" / "Not final") and per `CLAUDE.md`'s general rule that a value can be a legitimate placeholder for V1 while still requiring a documented follow-up. **This status explicitly means: usable for Gazebo V1 now; must be replaced by Inertia V2 later; must not be silently treated as a CAD-confirmed or final tensor in the interim.**

**Symmetry/format check (this document, informational only — not a re-derivation):** the tensor is presented in symmetric form (Ixy = Iyz = 0, Ixz = 0.01485 kg·m², matching the expected zero-Ixy/zero-Iyz pattern for a conventional aircraft with left-right (Y=0) mass symmetry and a nonzero Ixz product of inertia from the typical nose-up/tail-down mass asymmetry along X-Z). This is consistent with, not a substitute for, the "inertia symmetry must be checked/enforced" validation rule already stated in §10 rule 3 of this document.

**Sign-convention caveat (unresolved, flagged per §10 rule 5 of this document):** the master dataset does not state whether the Ixz value as given already matches the sign convention Gazebo/SDF's `<inertia>` element expects for its off-diagonal products of inertia, or whether it was entered into XFLR5 using XFLR5's own internal sign convention (which is not necessarily the same). **This must be verified before this tensor's Ixz value is entered into any SDF `<inertial><inertia>` block** — per `CLAUDE.md`'s general rule ("XFLR5 control sign ile Gazebo joint sign aynı varsayılmamalı" — the same discipline applies to inertia sign conventions, not just control-surface sign conventions) and per this document's own pre-existing §10 rule 5. This document does not perform that verification — it is out of scope for a docs-only synchronization pass and is flagged here as a prerequisite for whoever writes the SDF `<inertial>` block.

**This does NOT block Gazebo V1, per the master dataset's own explicit statement (§9):** "Gazebo V1 için kullanılabilir" — usable for Gazebo V1. **Inertia V2 is still needed later**, to be computed from the real mass distribution of: spars, servos, ESCs, avionics, GPS, flight controller, batteries, wiring, and motor/prop mass distribution (master dataset §9, listing these components explicitly) — none of which currently has a documented mass/position in this repository beyond the battery data added in §6.1 below. Until Inertia V2 exists, this V1 tensor is the one to use; it must not be blocked on, and it must not be silently "improved" or re-derived without the real component mass distribution behind it.

The full V2 inertia tensor must eventually be sourced from CAD (preferred, mass-property export at the documented CG/origin) or a derived hand calculation from the real component mass distribution (§6) — this remains true; it is simply no longer a blocker for V1 work.

---

## 6. Component Mass Distribution

**Update, 2026-08-21 (master-dataset synchronization pass):** the master dataset supplies mass values (and, for the main battery and the two motors, a center coordinate) for several components that were previously entirely `DATA_REQUIRED`. These are added below; items with no data in the master dataset remain genuinely `DATA_REQUIRED` — not guessed. The project owner's original **qualitative** layout facts (2026-08-21) remain recorded as confirmed facts alongside the new quantitative data.

### 6.1 Main Battery — position and mass (item 3 of this pass's assignment)

| Parameter | Value | Unit | Reference frame | Provenance | Status |
|---|---|---|---|---|---|
| Main battery type | 4S 22000 mAh 25C | — | — | master dataset §43 | CONFIRMED |
| Main battery nominal/full voltage | 14.8 V nominal / 16.8 V full | V | — | master dataset §43 | CONFIRMED |
| Main battery mass | ≈1.666 | kg | — | master dataset §43 (dataset's own "≈" qualifier retained) | CONFIRMED (master dataset; approximate per source's own notation) |
| Main battery center coordinate | (0.300631, 0.000000, 0.038547) | m | Gazebo/CAD (FLU) | master dataset §7 | CONFIRMED |
| Main battery center, CG-relative offset | ΔX ≈ +0.132322 m (forward of CG), ΔY = 0, ΔZ ≈ −0.061453 m (below CG) | m | Gazebo/CAD, CG-relative | master dataset §7 (arithmetic reproduced and confirmed: 0.300631 − 0.168309 = 0.132322; 0.038547 − 0.100000 = −0.061453) | CONFIRMED |
| Secondary battery type | 3S 3300 mAh | — | — | master dataset §43 | CONFIRMED |
| Secondary battery mass | ≈0.248 | kg | — | master dataset §43 | CONFIRMED (approximate per source's own notation) |
| Secondary battery center coordinate | — | m | Gazebo/CAD (FLU) | none found — master dataset §43 explicitly states "exact position unknown" | `DATA_REQUIRED` — explicitly not invented, per task instruction |

See `GEOMETRY.md` §5 (qualitative-facts table) for the geometry-side cross-reference to this same data.

### 6.2 Other Components

| Component | Qualitative fact | Mass | Center coordinate | Provenance | Status |
|---|---|---|---|---|---|
| Main battery (4S 22000 mAh) | Centrally located | ≈1.666 kg | (0.300631, 0, 0.038547) m | project owner (qualitative); master dataset §7, §43 (quantitative) | CONFIRMED — see §6.1 |
| Secondary battery (3S 3300 mAh) | — | ≈0.248 kg | DATA_REQUIRED | master dataset §43 | Mass CONFIRMED; position DATA_REQUIRED |
| ESC (×2, Hobbywing Skywalker 80A, one per side) | Located in the wings | ≈0.080 kg each / ≈0.160 kg total | DATA_REQUIRED | project owner (qualitative, wing location); master dataset §42 (identity + mass) | Mass CONFIRMED (approximate, master dataset "≈" retained); position DATA_REQUIRED — master dataset §42 itself states "Kesin konumlar bilinmiyor" (exact positions unknown) |
| Motors (×2, SunnySky X2820 860KV) | Forward of CG, twin front-puller, left/right arrangement | ≈0.143 kg each / ≈0.286 kg total | (0.2623, +0.3000, 0.1269) m (left); (0.2623, −0.3000, 0.1269) m (right) | project owner (qualitative); master dataset §41 (mass), §46 (position, cross-referenced `GEOMETRY.md` §7) | CONFIRMED — see `GEOMETRY.md` §7 for the explicit note distinguishing this "motor center" value from the raw motor-mesh bounding-box center (≈7.3 mm difference in X, not silently unified) |
| Propellers (×2, APC 13x6.5E) | Mounted on the two motors (front-puller) | ≈0.0301 kg each (≈1.06 oz) / ≈0.0601 kg total | (0.2951, +0.3000, 0.1271) m (left hub); (0.2951, −0.3000, 0.1271) m (right hub) | `CLAUDE.md` (identity); master dataset §44 (mass), §46 (hub position, cross-referenced `GEOMETRY.md` §7) | CONFIRMED |
| Servos (×7, Emax ES08MAII, conventional-tail set per manual) | Identity only — no mass given | DATA_REQUIRED | DATA_REQUIRED | master dataset §4 (identity only) | Identity CONFIRMED; mass/position DATA_REQUIRED |
| Payload | No data found in repository | DATA_REQUIRED | DATA_REQUIRED | none found | DATA_REQUIRED |
| Airframe / structure (wing, tail, fuselage, printed/LWPLA parts, spars) | No numeric mass found in repository | DATA_REQUIRED | DATA_REQUIRED | master dataset §4 lists structural member *dimensions* (spar OD/ID/length) but no mass; none found | DATA_REQUIRED |
| Wiring / avionics / GPS / flight controller | No data found in repository | DATA_REQUIRED | DATA_REQUIRED | none found | DATA_REQUIRED |

No component mass value was found anywhere in the repository that could be flagged `REVISION_REQUIRES_CONFIRMATION` — there is nothing of that kind present at all (not even from an older/different aircraft), so this category is currently empty. It is kept as an explicit heading here for future updates, per task instructions.

**Sum check (partial, informational only — not a validation pass):**

```
main battery      ≈ 1.666 kg
secondary battery ≈ 0.248 kg
motors (×2)       ≈ 0.286 kg
ESCs (×2)         ≈ 0.160 kg
propellers (×2)   ≈ 0.060 kg (≈0.0601 kg)
                  ---------
known subtotal    ≈ 2.420 kg

total aircraft mass (§2)         = 6.000 kg
remaining, unaccounted           ≈ 3.580 kg
  (airframe structure, servos, wiring, avionics/GPS/flight-controller,
   any payload — all still individually DATA_REQUIRED)
```

This is a running total for tracking purposes only — it does **not** validate or cross-check anything (the ≈3.58 kg remainder is exactly what §9 item 18 below, "Airframe structural mass," plus servos/wiring/avionics, must eventually account for), and no component mass is inferred, backed-out, or estimated from this arithmetic. The full component-sum-equals-6.000-kg validation rule (§10 rule 1) remains **not yet checkable** in full — only ≈40% of the total mass is currently itemized.

---

## 7. Provenance

Consolidated provenance for every value used in this document.

| Source | What it provides | Used for |
|---|---|---|
| Project owner, direct conversation, 2026-08-21 | Total mass, both CG triples, MTOW rating, battery/ESC/motor qualitative layout facts | §1, §2, §3, §6 |
| `CLAUDE.md` (repository root) | Restates total mass, both CG triples, body-frame convention, motor/propeller model identity | Cross-check for §2, §3, §6 |
| `docs/source_of_truth/README.md` | Restates total mass, both CG triples; independently flags CG origin/reference-frame definitions and inertia tensor as `DATA_REQUIRED` | Cross-check for §3, §4, §5 |
| Full-repository search + git history review (this task) | Confirms absence of any inertia data, component mass data, or CAD mass-property export anywhere in the repository, past or present | Basis for §5, §6 findings |

**Source hierarchy** (per role definition — for characterizing a conflict if found, never for silently picking a winner):
1. Confirmed current FALCON V2 CAD-derived data
2. Confirmed measured aircraft data
3. Manufacturer data
4. Current FALCON V2 engineering calculations
5. Current FALCON V2 XFLR5/XFOIL data
6. Older project notes

No case in this document required applying this hierarchy, because no conflicting values were found (see §8).

---

## 8. Conflicts

**No conflicts found.** The total mass and both CG triples supplied by the project owner (Step 6 of this task) were cross-checked against `CLAUDE.md` and `docs/source_of_truth/README.md`:

| Parameter | Project owner (this task) | `CLAUDE.md` | `docs/source_of_truth/README.md` | Result |
|---|---|---|---|---|
| Total aircraft mass | 6.000 kg | 6.000 kg | 6.000 kg | Match |
| Gazebo/CAD CG | (0.168309, 0.000000, 0.100000) m | (0.168309, 0, 0.100000) m | (0.168309, 0, 0.100000) m | Match |
| XFLR5 CG | (+0.0637, 0.0000, -0.0210) m | (0.0637, 0, -0.0210) m | (0.0637, 0, -0.0210) m | Match |

Manufacturer MTOW (7 kg) is new in this task and was not previously recorded anywhere in the repository, so there is nothing for it to conflict with (and, per §2, it is not the same parameter as total aircraft mass in the first place). No inertia value or component mass value exists anywhere in the repository, so none could conflict with anything.

**Update, 2026-08-21 (master-dataset synchronization pass):** the newly-added inertia tensor (§5.1) and battery/motor/ESC/propeller mass data (§6) were cross-checked against every other numeric value already in this document (total mass, both CG triples, motor/prop identity) — no conflict found. The Gazebo/CAD↔XFLR5 transform derived in §3.5 was cross-validated against two independent published data points (the documented XFLR5 CG, and — via `GEOMETRY.md` §8.3 — the horizontal-tail placement) and closed to sub-0.1 mm on both; this is treated as successful cross-validation, not a conflict. No `CONFLICT_REQUIRES_RESOLUTION` tag was needed anywhere in this pass.

---

## 9. Missing Data

Full list of `DATA_REQUIRED` items from this document, for tracking. **Update, 2026-08-21 (master-dataset synchronization pass): items resolved (fully or partially) this pass are marked below; retained in the numbered list, not deleted, so tracking history stays intact.**

1. Gazebo/CAD reference-frame physical origin on the airframe (§4) — `[PARTIALLY RESOLVED]` a candidate shared reference point (main-wing-root LE, 0.23196/0/0.12103 m) now exists via §3.5/`GEOMETRY.md` §8.3, but is not independently CAD-confirmed as a named datum
2. XFLR5 reference-frame axis convention (§4) — `[PARTIALLY RESOLVED]` X reversed vs. Gazebo/CAD +X, Z same-sense (master dataset §2); Y-axis untested
3. XFLR5 reference-frame physical origin on the airframe (§4) — `[PARTIALLY RESOLVED]` identified as the main-wing-root-LE point, same caveat as item 1
4. XFLR5 units confirmation (§4) — still `DATA_REQUIRED`; the §3.5 transform's sub-0.1mm agreement is only consistent with meters in both frames, but this is corroborating evidence, not a direct statement
5. Gazebo/CAD ↔ XFLR5 CG conversion (transform) (§3.3, §3.5, §4) — `[RESOLVED for X/Z]` `XFLR5_X = 0.23196 - Gazebo/CAD_X`, `XFLR5_Z = Gazebo/CAD_Z - 0.12103`, derived and cross-validated to sub-0.1mm (§3.5, `GEOMETRY.md` §8.3). Y-axis component remains `DATA_REQUIRED`
6. Ixx, Iyy, Izz, Ixy, Ixz, Iyz — full inertia tensor (§5) — `[RESOLVED, as V1_PROVISIONAL]` master dataset §9; see §5.1 for the full tensor, method, and explicit non-final status. Inertia V2 (real component mass distribution) remains a future requirement, not a current blocker
7. Inertia reference origin (§5) — `[RESOLVED]` Gazebo/CAD CG (0.168309, 0, 0.100000 m), per §5.1
8. Inertia coordinate frame (§5) — `[RESOLVED]` Gazebo/CAD (FLU), per §5.1. Sign convention for the off-diagonal Ixz term is explicitly **not** resolved — flagged in §5.1, must be verified before SDF use
9. Battery mass (§6) — `[RESOLVED, main battery]` ≈1.666 kg, master dataset §43; secondary battery mass also resolved, ≈0.248 kg
10. Battery center coordinate (§6) — `[RESOLVED, main battery]` (0.300631, 0, 0.038547) m, master dataset §7. Secondary battery center remains `DATA_REQUIRED` — explicitly not invented, per task instruction
11. ESC mass (each unit) (§6) — `[RESOLVED]` ≈0.080 kg each / ≈0.160 kg total, master dataset §42
12. ESC center coordinate (each unit, left/right wing) (§6) — still `DATA_REQUIRED`; master dataset §42 itself states exact positions are unknown
13. Motor mass (each of 2 motors — SunnySky X2820 860KV) (§6) — `[RESOLVED]` ≈0.143 kg each / ≈0.286 kg total, master dataset §41
14. Motor center coordinate (left, right) (§6) — `[RESOLVED]` (0.2623, ±0.3000, 0.1269) m, master dataset §46; see `GEOMETRY.md` §7 for the explicit motor-center-vs-bounding-box-center distinction (≈7.3 mm difference, not silently unified)
15. Propeller mass (each of 2 — APC 13x6.5E) (§6) — `[RESOLVED]` ≈0.0301 kg each / ≈0.0601 kg total, master dataset §44
16. Servo existence/count/mass/location (§6) — `[PARTIALLY RESOLVED]` existence/count/identity now known (7 × Emax ES08MAII, conventional-tail set, master dataset §4); mass and location remain `DATA_REQUIRED`
17. Payload mass/location (§6) — still `DATA_REQUIRED`
18. Airframe structural mass (wing, tail, fuselage, spars, printed/LWPLA structure) (§6) — still `DATA_REQUIRED`; master dataset §4 gives structural-member *dimensions* (spar OD/ID/length) but no mass value

None of these are estimated, guessed, or filled with placeholder numeric values anywhere in this document.

---

## 10. Validation Requirements

The following rules govern any future use of this document's data in SDF/simulation implementation. They are stated explicitly here so `gazebo-testing` and `validation` (and any implementing specialist) can check against them directly.

1. **Mass closure:** the sum of all component masses (battery + ESCs + motors + propellers + servos + payload + airframe structure + any other component) must equal the total aircraft mass of 6.000 kg exactly. This cannot currently be checked — see §6, §9 — because component masses are `DATA_REQUIRED`.
2. **Inertia reference origin:** the inertia tensor must be used only together with the reference origin it was computed about (§5). An inertia tensor without its associated origin, CG, and coordinate frame is not usable and must not be entered into any SDF `<inertial>` block.
3. **Inertia symmetry:** the inertia tensor is symmetric (Ixy = Iyx, Ixz = Izx, Iyz = Izy) — this must be checked/enforced when a real tensor is obtained, not assumed correct by construction.
4. **SI units:** all inertia values must be in SI units (kg·m²), per `CLAUDE.md` engineering rules.
5. **Products-of-inertia sign convention:** the sign convention used for the off-diagonal products of inertia (Ixy, Ixz, Iyz) must be explicitly documented (e.g., whether they are entered as computed, or already negated per the SDF/Gazebo `<inertia>` element's expected sign convention) before use — never assumed silently.
6. **Parallel-axis transformations:** if the inertia tensor must be translated from a CAD-computed origin to the SDF link origin or the CG, the parallel-axis theorem application must be shown explicitly (mass, offset vector, resulting tensor) — not applied silently inside implementation code.
7. **CG frame separation:** the Gazebo/CAD CG (§3.1) and the XFLR5 reference CG (§3.2) must never be mixed. Any SDF `<inertial><pose>` uses the Gazebo/CAD CG only; any XFLR5-referenced aerodynamic calculation uses the XFLR5 CG only.
8. **Coordinate transformation documentation:** any coordinate transformation performed anywhere (frame-to-frame, origin-to-origin, or otherwise) must be documented with source frame, target frame, and derivation — per `CLAUDE.md` and this document's §3.3/§4.
9. **Revision consistency:** the inertia tensor and component mass data used in implementation must correspond to the same FALCON V2 revision as the geometry data in `GEOMETRY.md`. Any value whose revision cannot be confirmed must be marked `REVISION_REQUIRES_CONFIRMATION` (§6) or `UNVERIFIED` (§5) and excluded from authoritative use — never assumed current by default.
10. **No tuning via mass properties:** no mass, CG, or inertia value may be changed merely to improve simulation behavior (e.g., to make the aircraft "fly better" or pass a test). Per `CLAUDE.md` simulation-tuning policy, mass/CG/inertia changes require explicit authorization and documented justification, never a test-driven shortcut.

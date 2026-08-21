# FALCON V2 — Controls Source of Truth

**Owner:** `controls-integration`
**Status:** Documentation sync against the newly-added project master dataset. No SDF joint, actuator, servo, or ArduPilot integration code has been written or is written by this document (docs-only pass, per task instruction). This revision **removes a now-stale blanket framing** from the previous version of this file (see §0 below) and adds: (a) an explicit, two-concept treatment of control-surface deflection limits, (b) per-surface sign-convention documentation with required future sign-verification tests, (c) the first servo-model data point found in the repository, (d) an updated ArduPilot/SITL status. It does **not** derive, assert, or invent any hinge axis, deflection-limit number beyond what the master dataset states, sign value, or channel number — those remain either cited from the master dataset (with the master dataset's own status label preserved) or `DATA_REQUIRED`.
**Compiled:** 2026-08-21 (original). **Updated:** 2026-08-21 (master-dataset sync pass). **Refreshed:** 2026-08-21 (small follow-up pass — `geometry-structure` has since resolved the elevator/rudder component-scope question in `GEOMETRY.md` §26.3; that conclusion is now cited correctly in §0–§2/§7/§8 below, superseding the "still being reassessed" framing from the master-dataset sync pass. No other content in this document changed in this follow-up pass — the SDF-ready hinge-axis fit and all four sign-test requirements remain open exactly as previously documented).
**Source documents consulted:** `CLAUDE.md`; `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` (full read, all 74 sections, §1–§74); `docs/source_of_truth/geometry/GEOMETRY.md` — read twice: first as it stood at 2026-08-21 14:46 (pre-master-dataset revision, informed the master-dataset sync pass), then re-read for this follow-up pass specifically at §26.3 ("Resolution reached (2026-08-21, master-dataset synchronization pass)") to obtain `geometry-structure`'s current, authoritative conclusion on elevator/rudder component scope; `docs/source_of_truth/aerodynamics/AERODYNAMICS.md` (existence/scope check only, not duplicated here); repository-wide `find`/`grep` for ArduPilot/SITL/MAVLink/servo/channel-mapping artifacts (§6).

**Master-dataset status-label discipline (governs every citation below):** the master dataset (`docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt`) explicitly marks some of its own values `V1`, `provisional`, or an estimate (its own preamble: *"'V1', 'yaklaşık', 'provisional', 'tahmin' etiketli değerler final değildir"* — V1/approximate/provisional/estimate-labeled values are not final). Every value pulled from it into this document keeps that same non-final status; nothing is promoted to `CONFIRMED` here just because it is written down in the master dataset.

Status legend (carried forward from the prior version of this document, extended where noted):
- `CONFIRMED_FROM_MESH` — a geometric fact directly measured from STL vertex data (per `GEOMETRY.md`).
- `HINGE_REQUIRES_CONFIRMATION` — a candidate hinge region identified from mesh evidence; not a confirmed, joint-ready hinge axis. (Geometry-owned; not re-derived here.)
- `COMPONENT_SCOPE_REQUIRES_CONFIRMATION` — it is not established whether a given STL is movable-surface-only or fixed-structure-plus-movable-surface. (Geometry-owned.) **No longer applicable to `left_elevator.stl`/`right_elevator.stl`/`rudder.stl` as of this pass** — `geometry-structure` resolved this in `GEOMETRY.md` §26.3 (movable-surface-only, `body.stl` carries the fixed tail structure, small stated residual ambiguity). Retained in this legend only as a general-purpose tag definition; see §2 for the current, resolved status.
- `SERVO_ALLOCATION_REQUIRES_CONFIRMATION` — **new tag, this pass.** A servo model/quantity is named by the manufacturer manual, but which physical surface(s)/joint(s) each individual unit actuates is not stated. See §5.
- `DATA_REQUIRED` — not present anywhere in the repository; not guessed, not filled in.
- `ELEVATOR_SIGN_TEST_REQUIRED` / `AILERON_SIGN_TEST_REQUIRED` / `RUDDER_SIGN_TEST_REQUIRED` — **new tags, this pass.** A per-surface flag meaning: an aerodynamic-model (XFLR5) sign convention exists and is documented (§4), but the mapping from that convention to the actual Gazebo joint's positive-rotation direction is **not yet established** and must be confirmed by a dedicated test once the joint exists, per the `controls-integration` rule that a positive-command-to-physical-direction mapping is never assumed.

No `ASSUMPTION` or `TEMPORARY` entries appear in this document (none authorized for this task; none needed).

---

## 0. What Changed This Pass — Stale Claims Removed

The prior version of this document (compiled before the master dataset existed) stated, in its Consolidated Control-Surface Table, that hinge status for **all five** control surfaces was `HINGE_REQUIRES_CONFIRMATION` with "no single hinge line fitted" as the entire finding, and separately treated elevator/rudder **component scope** as an open question with no further data ("It is not known whether these meshes represent the movable elevator surface only, or the movable surface plus the adjacent fixed horizontal-stabilizer structure bundled into one mesh" / same for rudder) — i.e. the blanket framing was "nothing beyond a candidate mesh region is known for any surface."

**This blanket framing is now obsolete and is removed by this revision.** The master dataset (§21 elevator hinge, §29 rudder hinge, §34 aileron hinge, §66 movable-links list) shows that a materially more advanced hinge-extraction effort has since been performed: real per-span hinge chordwise-location (%chord) data exists for the elevator, rudder, and aileron, and §66 explicitly lists `left_aileron`, `right_aileron`, `left_elevator`, `right_elevator`, and `rudder` as **movable links with mesh ready for a separate joint** — not as "unknown-scope" meshes. This document no longer repeats "hinge entirely unconfirmed, no further data exists" as if nothing had changed.

**What is preserved, not removed:** the hinge axis is still not a fitted, SDF-ready 3D line/vector (still `HINGE_REQUIRES_CONFIRMATION`) — that is `geometry-structure`'s ownership, not re-derived here.

**Update (this follow-up pass):** at the time the paragraph above was first written, elevator/rudder component scope was cited as still `COMPONENT_SCOPE_REQUIRES_CONFIRMATION`, "being actively reassessed." `geometry-structure` has since concluded this reassessment: `GEOMETRY.md` §26.3 ("Resolution reached, 2026-08-21, master-dataset synchronization pass") states the status change explicitly — `COMPONENT_SCOPE_REQUIRES_CONFIRMATION` → resolved — concluding `left_elevator.stl`/`right_elevator.stl`/`rudder.stl` are **movable-surface-only**, with `body.stl` carrying the fixed horizontal-stabilizer and vertical-fin structure. `geometry-structure` states this reasoning in full at §26.3 (four converging lines of evidence, one of which — the movable elevator span in master dataset §21 matching `left_elevator.stl`'s own mesh Y-span to the mm — is a precise numeric match, not just a qualitative reading) and explicitly acknowledges a **residual, judged-small-but-nonzero ambiguity** (it remains conceivable, though judged unlikely, that `body.stl`'s extra tail-station material is fairing/blend geometry rather than a true fixed aerodynamic surface; not CAD-confirmed). See §2 below for the full citation. This document does not re-derive or second-guess that conclusion — `geometry-structure` remains the authority on this question — it only updates its own citation of it (superseding this section's and §2's prior "still being reassessed" framing).

---

## 1. Consolidated Control-Surface Table

| Column | Left Aileron | Right Aileron | Left Elevator | Right Elevator | Rudder |
|---|---|---|---|---|---|
| **Movable-link status** | `CONFIRMED` movable link, mesh ready (master dataset §66; mesh file `left_aileron.stl` confirmed present, `GEOMETRY.md` §4.2) | Same (master dataset §66; `right_aileron.stl` confirmed present) | Same (master dataset §66; `left_elevator.stl` confirmed present) | Same (master dataset §66; `right_elevator.stl` confirmed present) | Same (master dataset §66; `rudder.stl` confirmed present) |
| **Hinge chordwise location (%chord)** | Master dataset §34: ≈69.75%–72.14% across 4 sampled span stations (y=0.314–0.785 m) | Mirror of left (master dataset §34, §33 symmetry) | Master dataset §21: ≈74.71% (root, y=50.6 mm) tapering to ≈62.54% (tip, y=240 mm) | Mirror of left (master dataset §21; `GEOMETRY.md` confirms exact Y-mirror) | Master dataset §29: ≈74.81% (root, Z=130.5 mm) tapering to ≈54.79% (tip, Z=299 mm) |
| **Hinge as SDF-ready joint axis (position + direction vector)** | Not yet — `HINGE_REQUIRES_CONFIRMATION`. This is a `geometry-structure`-owned derivation from the %chord data above plus mesh evidence; not re-derived in this document. See §2. | Same | Same | Same | Same |
| **Component scope (movable-only vs. fixed-structure-plus-movable)** | Not flagged as an open question anywhere in the repository (aileron STLs were never flagged — carried forward unchanged) | Same | **Resolved** — movable-surface-only, `body.stl` carries the fixed horizontal-stabilizer structure (`GEOMETRY.md` §26.3, `geometry-structure`-owned conclusion; small stated residual ambiguity, not CAD-confirmed). See §2. | Same | **Resolved** — movable-surface-only, `body.stl` carries the fixed vertical-fin structure (`GEOMETRY.md` §26.3; same residual-ambiguity caveat). See §2. |
| **Neutral pose status** | `DATA_REQUIRED` for an authoritative 0°-deflection definition; the as-exported mesh pose is the only pose present (`GEOMETRY.md`) | Same | Same | Same | Same |
| **Sign convention (XFLR5 side, documented)** | WF1..WF6 = +1,+1,+1,−1,−1,−1 differential gain pattern (master dataset §35) — see §4.3 | Same pattern, opposite physical side | + = trailing-edge-down, − = trailing-edge-up (master dataset §22) — see §4.1 | Same | All rudder-flap gains = 1 in the Type 7 sweep (master dataset §32) — see §4.2 |
| **Sign convention (Gazebo joint, physical actuator)** | `AILERON_SIGN_TEST_REQUIRED` | `AILERON_SIGN_TEST_REQUIRED` | `ELEVATOR_SIGN_TEST_REQUIRED` | `ELEVATOR_SIGN_TEST_REQUIRED` | `RUDDER_SIGN_TEST_REQUIRED` |
| **Mechanical joint-limit provenance** | Manufacturer manual initial recommendation only: ±30° or more (master dataset §3, §65) — see §3 | Same | Same | Same | Same |
| **Aero-derivative validated linear range** | ≈±10° (master dataset §65, and the actual XFLR5 Type 7 sweep range run, §36: −10 to +10) — see §3 | Same | ≈±10° (§65; sweep range run, §22: −10 to +10) | Same | ≈±10° (§65; sweep range run, §32: −10 to +10) |
| **Servo model** | `DATA_REQUIRED` — not named for the wing/aileron in the manual excerpt captured (see §5) | Same | Emax ES08MAII named for "conventional tail" (master dataset §4), but exact per-surface allocation among the 7 units is `SERVO_ALLOCATION_REQUIRES_CONFIRMATION` — see §5 | Same | Same |
| **ArduPilot channel mapping** | `DATA_REQUIRED` — no SITL parameter file exists anywhere in the repository (§6) | Same | Same | Same | Same |

---

## 2. Movable-Link Scope and Component-Scope Cross-Reference (not re-derived here)

**Full movable-link set (master dataset §66, cross-checked):** `left_aileron`, `right_aileron`, `left_elevator`, `right_elevator`, `rudder`, `left_prop`, `right_prop` — seven movable links, each with "mesh hazır" (mesh ready) per the master dataset. Cross-check performed for this document: all seven correspond to distinct STL files already inventoried in `GEOMETRY.md` §4.2 (`left_aileron.stl`, `right_aileron.stl`, `left_elevator.stl`, `right_elevator.stl`, `rudder.stl`) and master dataset §10's 12-part STL package list, where "prop" = "pervane" (Turkish for propeller) → `left_pervane.stl` / `right_pervane.stl`. **Confirmed: this is the full movable-link set with mesh present for each — no additional movable control surface is named anywhere in the master dataset (§10, §66) or in `GEOMETRY.md`'s mesh inventory that is missing from this list, and no member of this list lacks a mesh file.** `left_motor.stl` / `right_motor.stl` are **not** in the movable-link list — consistent with the motor housings being fixed to the airframe while only the propeller mesh rotates.

**Ownership split on `left_prop`/`right_prop`:** these are continuous-rotation joints, not deflection-limited control surfaces. Per this agent's ownership boundary, getting the ArduPilot throttle output correctly connected to that joint's command/velocity interface in Gazebo is `controls-integration`'s responsibility; the joint's mechanical definition is `geometry-structure`'s; the RPM→thrust/torque physics response is `propulsion`'s (`docs/source_of_truth/propulsion/PROPULSION.md`). This document does not model or assert any prop-joint control-loop numeric value.

**Component scope of `left_elevator.stl`/`right_elevator.stl`/`rudder.stl` — resolved by `geometry-structure`, cited (not re-derived) here:** `GEOMETRY.md` §26.3 ("Resolution reached, 2026-08-21, master-dataset synchronization pass") states: *"Status change: `COMPONENT_SCOPE_REQUIRES_CONFIRMATION` → resolved. Conclusion: `left_elevator.stl` / `right_elevator.stl` / `rudder.stl` are movable-surface-only meshes; `body.stl` carries the fixed horizontal-stabilizer and vertical-fin structure."* The reasoning combines (1) the prior mesh-geometry evidence already in `GEOMETRY.md` §26.1/§26.2 (unchanged, still valid — `body.stl` already has wider/taller structure than the elevator/rudder meshes reach at their exact station, and the elevator/rudder meshes' own chordwise-thickness profile shows the signature of a partial-chord section cut at a hinge line, not a full airfoil) with (2) new master-dataset evidence: §66's movable-links list treating these as individually-scoped movable parts, §21's stated "movable elevator span" (y≈50.60–240.00 mm) matching `left_elevator.stl`'s own mesh Y-span (50.600–240.000 mm) essentially exactly, and the absence of any separate fixed-tail-structure mesh among the 12 STL files (unlike the aileron, which has a distinct `left_wing.stl`/`right_wing.stl` counterpart).

`geometry-structure` explicitly states a **residual ambiguity, not eliminated**, per `GEOMETRY.md` §26.3: *"none of the four points above is a literal CAD part-tree readout or a project-owner statement of intent for this exact question. It remains conceivable that `body.stl`'s extra tail-station material is fairing/blend geometry rather than a true aerodynamic fixed-stabilizer/fin surface... This residual possibility is judged small — not zero... It is judged small enough to no longer block SDF link/joint structuring work on this specific question, but it is not formally CAD-confirmed."* This document reproduces that qualification rather than dropping it — the scope question is resolved for practical SDF-structuring purposes, not closed with zero remaining uncertainty, and if a CAD source or the project owner later states otherwise, `geometry-structure` (not this document) will revisit and update it.

`geometry-structure` also states the practical consequence for future SDF work (`GEOMETRY.md` §26.3): `left_elevator.stl`, `right_elevator.stl`, and `rudder.stl` may be treated as single-link movable control surfaces, each attachable to a hinge joint at the candidate/real hinge region already identified; the fixed horizontal-stabilizer and vertical-fin surfaces are treated as part of the fixed `body.stl` link, with no additional fixed-tail mesh/link needing to be sourced or split out for V1.

**This document does not re-derive, second-guess, or independently confirm this conclusion — `geometry-structure` remains the sole authority on this question.** It is cited here only because it directly changes an item this document previously listed as blocking (§7, §8).

**Controls-integration implication, updated for the resolved scope:** because each of `left_elevator.stl`/`right_elevator.stl`/`rudder.stl` is now treated as a single, movable-only rigid body for SDF purposes, a hinge joint attached to the whole mesh will actuate the intended control surface only, without incorrectly rotating fixed tail structure — the specific failure mode this document previously flagged as the reason component scope blocked joint definition no longer applies. **What remains blocking (unchanged):** the hinge axis itself is still `HINGE_REQUIRES_CONFIRMATION` (a candidate region, not a fitted SDF-ready line/vector — `geometry-structure`-owned, §8 item 2), and neutral pose, mesh unit/origin, and all four sign-test requirements (§4) remain fully open regardless of the scope resolution. Component-scope resolution removes one blocking item from the dependency chain in §8; it does not resolve any of the others.

---

## 3. Control Surface Limits — Two Distinct Concepts (master dataset §65, §72)

The master dataset is explicit that these are **two different concepts and must not be confused**:

| Concept | Value | Source | What it is / is not |
|---|---|---|---|
| **Mechanical joint/servo throw** (a physical/CAD travel limit — the number that would eventually populate an SDF `<joint><limit><lower>/<upper>`) | ±30° or more ("veya daha fazla" — manual states this as a floor, not a firm upper bound) | Manufacturer manual initial setup recommendation (Titan Dynamics Falcon V2 Build & User Manual Rev 1.0), master dataset §3 ("control surfaces için başlangıç throw ±30 deg veya daha fazla"), restated §65 | A **manufacturer starting-point recommendation for initial ArduPlane setup**, not a measured servo-horn/linkage travel limit, not a CAD hard-stop, and not derived from any hinge geometry in this repository. Still `DATA_REQUIRED` as an exact, surface-specific mechanical limit — the manual gives one shared approximate figure, not five distinct measured limits. |
| **Aerodynamic-derivative high-confidence linear range** (the range over which the XFLR5-derived force/moment coefficients — Cmde, CYdr, Cndr, Cldr, Clda, Cnda, CYda — are directly validated by swept data, not extrapolated) | ≈±10° | Master dataset §65 ("Analiz derivatives: çoğunlukla ±10 deg aralığında... Dolayısıyla en güvenilir lineer range: ≈±10 deg"), corroborated by the actual XFLR5 Type 7 sweep ranges run: elevator §22 (−10° to +10°, with +10° explicitly noted as producing negative lift and trim being skipped), rudder §32 (−10° to +10°), aileron §36 (−10° to +10°) | This is a statement about where the **aerodynamic model** (owned by `aerodynamics`, see `AERODYNAMICS.md`) is validated by direct sweep data, not a statement about what a servo or joint can physically achieve. |

**Why this document states both instead of picking one:** if a future SDF joint limit is set to the mechanical ±30° figure (reasonable for the joint itself) while the aerodynamic force model is linearly extrapolated past ±10° for any commanded deflection between ±10° and ±30°, the aerodynamic output in that range is unvalidated and must not be silently trusted as accurate — this is `aerodynamics`' concern for how the force/moment model saturates or is looked up beyond ±10° (master dataset §72: "±10 deg control derivative range en güvenilir analiz bandı... stall öncesi linear derivatives sonsuza kadar extrapolate edilmemeli"), not something `controls-integration` resolves by narrowing the mechanical joint limit. Per the project's simulation-tuning policy, control authority (i.e., the mechanical limit) must not be adjusted to compensate for a separate concern (aerodynamic-model validity range) — the two are tracked here explicitly so neither agent accidentally uses the other's number for the wrong purpose.

**Exact per-surface deflection limits beyond these two ranges:** `DATA_REQUIRED`. Neither the manual excerpt captured in the master dataset nor any other repository document gives a distinct mechanical stop angle for the aileron vs. elevator vs. rudder individually — only the shared ±30°-or-more starting recommendation.

---

## 4. Sign Conventions — Per Surface

**Governing rule for this entire section (master dataset §72, "kritik uygulama uyarıları"):** *"XFLR5 control sign ile Gazebo joint sign aynı varsayılmamalı; unit-test ile eşlenmeli"* — the XFLR5 control sign must not be assumed equal to the Gazebo joint sign; it must be mapped by a unit test. This is the same rule already codified in this agent's ownership boundary (never assume positive command = expected physical direction). Every sign statement below is an **XFLR5-side aerodynamic-model convention**, documented here because it is control-surface-sign data and therefore this agent's ownership to record — **it is not a statement about the physical Gazebo joint's positive-rotation direction**, which does not exist yet (no SDF joint has been created) and cannot be asserted without a test once it does.

### 4.1 Elevator

XFLR5 sign convention (master dataset §22): **+ = trailing-edge-down, − = trailing-edge-up.**

Practical trim table from the Type 7 stability-polar sweep (master dataset §22; cross-reference `AERODYNAMICS.md` for the underlying CLa/Cma/Cmq/NP/CMde derivative values at each point — not restated here beyond what is needed for sign context):

| Trim airspeed | Elevator deflection (XFLR5 sign) |
|---|---|
| ≈18.2 m/s | ≈−8° |
| ≈19.1 m/s | ≈−6° |
| ≈20.0 m/s | ≈−4° |
| ≈20.8 m/s | ≈−2° |
| ≈21.24 m/s (neutral trim) | ≈0° |

Within this same sweep, `Cmde` is consistently negative (≈−0.60 to −0.73 /rad, master dataset §22–§23) — i.e., a positive (trailing-edge-down) elevator deflection produces a negative (nose-down) pitching-moment contribution, which is the textbook-expected sign relationship for a conventional-tail elevator and is internally consistent within XFLR5's own output. This is recorded here only as context for the sign convention; the derivative magnitudes themselves are `aerodynamics`' data.

**What is not established:** whether commanding "+" on the eventual Gazebo elevator joint (however that command is defined — PWM, radians, normalized authority) produces trailing-edge-down or trailing-edge-up. **`ELEVATOR_SIGN_TEST_REQUIRED`** before any elevator joint sign is treated as correct, to be executed as `ELEVATOR_TEST` / `CONTROL_SURFACE_DIRECTION_TEST` per the standard workflow once `geometry-structure` defines the joint.

### 4.2 Rudder

Master dataset §32: the Type 7 rudder control sweep applied **gain = 1 to all rudder-flap segments** (i.e., a single uniform, non-differential gain — unlike the aileron's differential WF1–WF6 pattern in §4.3). Response is described as "approximately linear and symmetric" (*"Rudder response yaklaşık lineer ve simetrik"*) across the swept ±10° range, with the dominant effect being yaw + side-force and only a small roll-coupling term.

**What the master dataset does not state:** the physical trailing-edge direction (left vs. right) that corresponds to XFLR5's positive rudder-deflection convention. Only the resulting sign pattern of the derivatives is given (CYdr≈+0.085/rad, Cndr≈−0.025/rad, Cldr≈+0.0007/rad near center — `aerodynamics`-owned data, `AERODYNAMICS.md`), not a stated "TE-left" or "TE-right" definition for "+". This is an additional reason a sign test is required, not just a Gazebo-mapping question but also a confirmation of what XFLR5's own "+" meant physically.

**`RUDDER_SIGN_TEST_REQUIRED`** before any rudder joint sign is treated as correct, to be executed as `RUDDER_TEST` / `CONTROL_SURFACE_DIRECTION_TEST` per the standard workflow once `geometry-structure` defines the joint.

### 4.3 Aileron

Master dataset §35: XFLR5's aileron differential mapping across its six wing-flap segments is:

`WF1=+1, WF2=+1, WF3=+1, WF4=-1, WF5=-1, WF6=-1`

The master dataset states this mapping "produces symmetric roll moment and is accepted as functioning within XFLR5" (*"Bu mapping simetrik roll moment ürettiği için çalışır kabul edildi"*) — i.e., this is a statement that the mapping works correctly **as an internal XFLR5 aerodynamic-model bookkeeping convention** for generating a symmetric differential-roll sweep, not a statement about which physical wing (left or right) or which physical trailing-edge direction WF1–WF3 vs. WF4–WF6 correspond to on the real airframe.

**Explicit warning carried forward from the task instruction (this is not this agent inventing caution — it is the correct reading of master dataset §35 and the general rule in §72):** this XFLR5 flap-gain sign convention **must not be automatically equated with the eventual Gazebo joint-sign convention** for the physical left/right aileron actuators. That mapping requires its own explicit sign test — it is not derivable from the XFLR5 gain pattern alone, and treating "WF1–WF3 = +1" as "this is what a positive left-aileron-servo command means" without verification would violate the sign-convention-discipline rule.

The master dataset itself makes the same caution for a related, downstream question (§36): *"Adverse/proverse etiketi physical side/deflection mapping kesinleşmeden yalnızca işaretten verilmemeli"* — the adverse/proverse yaw label must not be assigned from sign alone until the physical side/deflection mapping is finalized. This document records that caution as directly applicable to any future adverse-yaw compensation or aileron-rudder mixing decision, which must wait for the sign test, not be guessed from the XFLR5 derivative sign (Cn_delta_a ≈ +0.00144/rad near-linear, +0.00165/rad near center — `aerodynamics`-owned data).

**`AILERON_SIGN_TEST_REQUIRED`** before any aileron joint sign (left or right, independently) is treated as correct, to be executed as `AILERON_TEST` / `CONTROL_SURFACE_DIRECTION_TEST` per the standard workflow once `geometry-structure` defines the joints. Note this must be verified **per side** — a correct left-aileron sign mapping does not by itself confirm the right-aileron mapping, since they are separate physical actuators/joints even though the XFLR5 model ties them together with a single differential gain pattern.

### 4.4 Summary — required future tests, this pass

No sign value in §4.1–§4.3 is to be hardcoded into any actuator/plugin/mixer configuration from the XFLR5 naming or gain pattern alone. The following tests are required (owned by `gazebo-testing`, to run only once `geometry-structure` has defined the corresponding joint and `controls-integration` has wired a command interface to it), with `validation` reviewing the result, per the standard workflow:

- `ELEVATOR_SIGN_TEST_REQUIRED` → `ELEVATOR_TEST`
- `AILERON_SIGN_TEST_REQUIRED` → `AILERON_TEST` (left and right independently)
- `RUDDER_SIGN_TEST_REQUIRED` → `RUDDER_TEST`
- General cross-check: `CONTROL_SURFACE_DIRECTION_TEST`, followed by `ROLL_RESPONSE_TEST` / `PITCH_RESPONSE_TEST` / `YAW_RESPONSE_TEST` to confirm the full command→deflection→moment→rate chain, not just the deflection direction in isolation.

---

## 5. Servo / Actuator Data

**Update, this pass:** the master dataset provides the **first servo-model data point found anywhere in this repository** (previously, the entire category was `DATA_REQUIRED` with a negative search result as the only finding).

Master dataset §4 ("MANUAL YAPISAL BİLGİLER" / manual structural information), verbatim in substance:

> Conventional tail: 1× 4×500 mm hstab support, 1× 3×500 mm elevator hinge, 1× 4×170 mm vstab support, 1× 3×220 mm rudder hinge.
> Servolar (servos): **7 × Emax ES08MAII** for the conventional tail.

**What this does and does not establish:**
- **Servo model identity:** Emax ES08MAII is named by the manufacturer manual as the servo used "for the conventional tail." Provenance: manufacturer manual (Titan Dynamics Falcon V2 Build & User Manual Rev 1.0), via master dataset §4. This is a real, citable model identity — not invented.
- **Quantity/allocation ambiguity — new tag `SERVO_ALLOCATION_REQUIRES_CONFIRMATION`:** the manual states 7 units "for the conventional tail" as a single figure, immediately following the tail hinge-hardware list. It does **not** break this down into a per-surface count (e.g., how many drive the elevator — one shared unit, or one per side if elevator is split-actuated per `left_elevator`/`right_elevator` links per §2 — versus the rudder, versus any other tail-adjacent mechanism). Seven is a larger count than a minimal elevator(1)+rudder(1) or independent-elevator(2)+rudder(1) allocation would require, so this document does **not** guess at an allocation (e.g., it is not assumed that this means 2 elevator + 2 elevator-trim-tab + 2 rudder + 1 spare, or any other specific split) — that would be inventing data. Status: `SERVO_ALLOCATION_REQUIRES_CONFIRMATION`, to be resolved from the CAD source or the project owner, in the same category of open question as component scope (§2).
- **Aileron servo model:** not named in the master-dataset excerpt of the manual (the manual's servo line item explicitly scopes to "conventional tail"). Status: `DATA_REQUIRED`.
- **Servo performance specs (torque, speed, PWM/voltage range, travel-per-µs):** no datasheet for the Emax ES08MAII has been ingested into this repository. Per the project's provenance rule, this document does **not** fill these in from general outside-repository knowledge — doing so without a citable datasheet in the repository would be an undocumented magic number. Status: `DATA_REQUIRED` — if a datasheet is added to the repository, it should be cited here (`docs/source_of_truth/controls/` per the source-of-truth policy), not embedded directly in plugin code.
- **Control gains / mixer values for actuation (e.g., PWM-to-radians scale, trim offsets, expo/rate curves):** none exist anywhere in the repository. Status: `DATA_REQUIRED`.

---

## 6. ArduPilot / SITL / MAVLink Status

**Manual recommendations captured (master dataset §3), recorded here as manufacturer-recommended *starting points* — none of these are implemented as an actual parameter file, and none should be read as if they were:**

- Firmware: ArduPlane recommended by the manual ("ArduPlane öneriliyor").
- Initial control-surface throw: ±30° or more — already covered in §3 above; repeated here only to note it appears in the same manual passage as the firmware recommendation.
- CG tolerance: ≈±10 mm (mass-properties concern, not controls — cross-reference `MASS_PROPERTIES.md`, not duplicated here).
- Propeller rotation: left CCW, right CW (propulsion concern — cross-reference `PROPULSION.md`, not duplicated here; recorded in the manual alongside the control-surface notes).
- `AUTOTUNE_LEVEL=8` (or similar) mentioned as an initial-setup suggestion ("AUTOTUNE_LEVEL=8 vb. başlangıç önerileri"). This is a **manual recommendation for a specific ArduPlane parameter name**, not a value this document adopts or writes into any parameter file — no `.parm` file exists in this repository (confirmed by repository search below), and creating one is out of scope for this docs-only pass per task instruction.

**Repository-wide search performed for this document (2026-08-21, this pass):** repeated the same method as the prior version of this document (`find` for `*.parm`/`*ardupilot*`/`*sitl*`/`*mavlink*`/`*servo*`/`*channel*` filenames; `grep -niE` for `ardupilot|sitl|mavlink|servo[0-9]|rcin|channel map|SERVO_OUT` across all tracked text file types) against the current repository state, which now additionally includes `model/meshes/*.stl`, `docs/source_of_truth/{geometry,aerodynamics,propulsion,mass_properties,master}/*`, and `docs/architecture/GAZEBO_READINESS.md` (none of these existed at the time of the prior version of this document).

**Result: still none found.** No `.sdf`, `.urdf`, `.parm`, or plugin source file exists anywhere in the repository. The only matches for the search keywords are: (a) this document and the master dataset describing the *need* for ArduPilot/SITL/channel-mapping work, and (b) the project's own agent-role definitions (`CLAUDE.md`, `.claude/agents/controls-integration.md`, `docs/architecture/AGENT_WORKFLOW.md`) describing this as `controls-integration`'s future ownership. No channel number, SITL parameter value, or MAVLink configuration is stated or invented anywhere in the repository.

**Conclusion (unchanged from the prior version of this document, reaffirmed by this pass's repeat search):** ArduPilot channel mapping, the SITL parameter file, and control gains/mixer values remain fully `DATA_REQUIRED`. Nothing is invented here to fill the gap.

---

## 7. Summary — `DATA_REQUIRED` / Open-Status Items for Control-Surface Work

1. SDF-ready hinge axis (position + direction vector) for left/right aileron, left/right elevator, rudder — `HINGE_REQUIRES_CONFIRMATION`; `geometry-structure`-owned, informed by master-dataset %chord data (§21, §29, §34 of the master dataset) but not yet a fitted joint axis (§2).
2. Component scope of `left_elevator.stl`/`right_elevator.stl`/`rudder.stl` — **resolved** (not `DATA_REQUIRED` / not open): `geometry-structure` concluded movable-surface-only in `GEOMETRY.md` §26.3, with a small stated residual (non-CAD-confirmed) ambiguity (§2). No longer a blocking item for joint-scoping purposes.
3. Neutral (0°-deflection) pose definition for all five control surfaces — `DATA_REQUIRED`; the as-exported mesh pose is not confirmed to represent a defined neutral position.
4. Gazebo joint positive-rotation-direction (physical sign) for all five control surfaces — `ELEVATOR_SIGN_TEST_REQUIRED` / `AILERON_SIGN_TEST_REQUIRED` / `RUDDER_SIGN_TEST_REQUIRED` (§4); cannot be established without a joint and a verifying test, neither of which exists yet.
5. Exact per-surface mechanical deflection limit beyond the shared manufacturer ±30°-or-more starting recommendation — `DATA_REQUIRED` (§3).
6. Servo model for aileron — `DATA_REQUIRED`; for elevator/rudder, model is named (Emax ES08MAII) but exact per-surface allocation among the manual's "7 units for conventional tail" is `SERVO_ALLOCATION_REQUIRES_CONFIRMATION` (§5).
7. Servo performance specs (torque, speed, PWM/voltage range) — `DATA_REQUIRED`; no datasheet in repository (§5).
8. Control gains / mixer values for aileron/elevator/rudder actuation — `DATA_REQUIRED` (§5).
9. ArduPilot SITL parameter file — does not exist (§6).
10. MAVLink / plugin interface configuration for Gazebo↔ArduPilot integration — does not exist (§6).
11. ArduPilot RC/servo channel mapping for aileron, elevator, rudder, throttle — `DATA_REQUIRED`; no channel number invented here (§6).
12. Mesh coordinate unit and mesh-origin-to-Gazebo/CAD-CG-origin relationship (`GEOMETRY.md` §13.1, §19/§30 item 3 as of the revision read for this document) — still open; any future hinge-position value derived from mesh coordinates inherits both of these unresolved dependencies. `geometry-structure`-owned.

---

## 8. What Is Blocking Future SDF Joint / ArduPilot Mapping Work

In dependency order (updated from the prior version of this document to reflect the master-dataset sync, and updated again in this follow-up pass now that item 1 below is resolved):

1. ~~**Component scope resolution**~~ — **RESOLVED** (§2): `geometry-structure` concluded in `GEOMETRY.md` §26.3 that `left_elevator.stl`/`right_elevator.stl`/`rudder.stl` are movable-surface-only, with `body.stl` carrying the fixed tail structure (small stated residual ambiguity, not CAD-confirmed). This item no longer blocks joint-scoping work — a hinge joint may be attached to the whole mesh of each without concern that fixed structure would be incorrectly rotated along with it.
2. **Hinge axis confirmation** — the master dataset's %chord candidate regions (§21, §29, §34) and `GEOMETRY.md`'s mesh-derived candidate regions are not yet a fitted 3D line/joint-axis vector. `geometry-structure`-owned. **Now the first remaining blocking item**, since item 1 is resolved.
3. **Neutral pose confirmation** — needed to know what 0° deflection means relative to the as-exported mesh pose, before any deflection angle (limit or commanded) can be expressed meaningfully.
4. **Mesh unit and origin-frame confirmation** — any hinge position pulled from mesh coordinates is only usable once the unit (evidenced but not CAD-confirmed as millimeters) and the mesh-origin-to-CG-origin relationship are resolved. `geometry-structure`-owned.
5. Only after 2–4 are resolved (item 1 is already resolved, see above) can an SDF joint be defined by `geometry-structure`, at which point `controls-integration` sets the joint limit from the manufacturer ±30°-or-more starting recommendation (§3, pending an exact per-surface number) and verifies sign convention against the actual joint axis with `ELEVATOR_TEST`/`AILERON_TEST`/`RUDDER_TEST`/`CONTROL_SURFACE_DIRECTION_TEST` (§4).
6. **Servo allocation** (§5, `SERVO_ALLOCATION_REQUIRES_CONFIRMATION`) and the **ArduPilot SITL parameter file / channel mapping** (§6) are independently `DATA_REQUIRED` and not blocked by items 2–4, but no channel mapping can be meaningfully assigned until it is known which physical surfaces will actually have independent joints. Note that the component-scope resolution (item 1) settles *what each mesh physically contains*, not *whether left/right elevator are commanded by one shared actuator or two independent ones* — that is a separate actuation/servo-allocation question, still tracked as `SERVO_ALLOCATION_REQUIRES_CONFIRMATION` (§5), not resolved by §26.3.

No SDF joint, actuator, servo, or ArduPilot integration code was created or modified by this task, per task constraints. No mesh file was read, modified, or re-derived by this document — all mesh-derived figures are cited from `GEOMETRY.md` and the master dataset, not recomputed here.

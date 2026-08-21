# FALCON V2 — Source of Truth

This directory holds the **authoritative engineering input data** for the FALCON V2 Gazebo Sim Harmonic simulation: geometry, mass properties, aerodynamics, propulsion, and control surfaces.

## Rules

- Every value here must be traceable to CAD, manufacturer data, XFOIL, XFLR5, measured test data, a derived calculation, or an explicitly documented assumption.
- Implementation code (SDF, plugins, controllers) may read this data. It must never silently change it.
- Missing data is marked `DATA_REQUIRED` — never filled in with a guess.
- Values marked `ASSUMPTION` or `TEMPORARY` must state why, and by whom/when authorized.

## Status

As of 2026-08-21, this repository was set up from an empty directory. The values below were provided directly by the project owner in the setup conversation. Update, same day (mesh-file pass): 12 binary STL mesh files have since appeared at `model/meshes/` (see `geometry/` below and `docs/source_of_truth/geometry/GEOMETRY.md` §4) — these are visual/collision mesh geometry, not CAD source or XFLR5 project files. Update, same day (deep mesh geometric analysis pass): the 12 mesh files have now been geometrically analyzed — vertex-wise bounding boxes, left/right symmetry checks, wing/body/tail/motor/propeller coordinate relationships, hinge-region and component-scope evidence — see `docs/source_of_truth/geometry/GEOMETRY.md` §12–§31 for full detail (not duplicated here). **This analysis measures mesh geometry; it does not resolve every geometry parameter** — see the note below. No native CAD source files, XFOIL/XFLR5 project files, or manufacturer datasheets have been added to this repository — those remain `DATA_REQUIRED` until placed in the relevant subdirectory below.

**Update, same day (pre-`model.sdf` source-of-truth consolidation pass):** each of the four remaining domains now has its own dedicated document — `aerodynamics/AERODYNAMICS.md`, `propulsion/PROPULSION.md`, `controls/CONTROLS.md` were created for the first time in this pass, and `geometry/GEOMETRY.md` / `mass_properties/MASS_PROPERTIES.md` were updated. **Most important outcome of this pass:** the ≈273 mm propeller diameter measured from the STL mesh (`left_pervane.stl`/`right_pervane.stl`) is formally reclassified `VISUAL_MESH_ONLY` — it is a mesh artifact, not a physical propulsion reference, and must never be used in any RPM/thrust/torque/advance-ratio/disk-area/tip-speed calculation. The real physical propeller for all physics is the APC 13x6.5E: **D = 0.3302 m, pitch = 0.1651 m** — see `propulsion/PROPULSION.md` §0 and `geometry/GEOMETRY.md` §20–§21.2. No SDF, plugin, or Gazebo implementation has started.

**Update, 2026-08-21 (master-dataset synchronization pass, `geometry`/`mass_properties` scope):** `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` (2247 lines, §1–§74) was read in full and layered onto `geometry/GEOMETRY.md` and `mass_properties/MASS_PROPERTIES.md` — see the `geometry/` and `mass_properties/` bullets below for the itemized outcome. Source-priority order applied: manufacturer manual > real aircraft measurement > real component manufacturer data > current STL geometry > XFOIL/XFLR5 result > derived calculation > V1 estimate/provisional; the master dataset's own status qualifiers ("V1", "provisional", "yaklaşık"/approximate, etc.) are preserved, not silently promoted to final. No mass, CG, or mesh file was changed.

---

## geometry/

- Wingspan: **2.093 m** (manufacturer/CAD reference)
- Wing area: **0.4514 m²** (manufacturer/CAD reference)
- CAD/STL mesh files: 12 binary STL files are present at `model/meshes/` (confirmed 2026-08-21). File **presence** is confirmed, and as of the 2026-08-21 deep-analysis pass, each file's **bounding box, symmetry, and coordinate relationships to the other meshes have also been measured and documented** — see `docs/source_of_truth/geometry/GEOMETRY.md` §12–§31 for the full inventory, mesh-derived dimensions, symmetry checks, manufacturer-comparison `GEOMETRIC_CHECK`s (wingspan, propeller diameter), and hinge/component-scope evidence.
- **Update, 2026-08-21 (master-dataset synchronization pass):** a master dataset (`docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt`) was layered onto `GEOMETRY.md` — full index at its §32. Several previously-open items are now resolved or partially resolved:
  - **STL coordinate scale** is now a named constant `STL_SCALE_TO_SI = 0.001`, status `DERIVED_WITH_STRONG_EVIDENCE` (multiple independent cross-checks converge on millimeters; still not CAD-metadata-confirmed) — `GEOMETRY.md` §13.1.
  - **Gazebo/CAD ↔ XFLR5 coordinate transform** is now `DERIVED` for X/Z (`XFLR5_X = 0.23196 - STL_X`, `XFLR5_Z = STL_Z - 0.12103`), cross-validated to sub-0.1 mm against two independent points — `GEOMETRY.md` §8.3. Y-axis behavior and any rotation beyond the stated X-sign reversal remain `DATA_REQUIRED`.
  - **Component scope for `rudder.stl`/the elevator STLs is now resolved**: concluded movable-surface-only, with `body.stl` carrying the fixed horizontal-stabilizer/vertical-fin structure — `GEOMETRY.md` §26.3 (reasoning shown, explicitly-acknowledged small residual ambiguity, not a rubber-stamp).
  - **Real hinge-geometry data** (position + per-station chordwise x/c) now exists for aileron, elevator, and rudder, tagged `HINGE_GEOMETRY_READY` — data in hand, SDF-axis-fit and Gazebo-sign-test still pending — `GEOMETRY.md` §6.
  - **Horizontal- and vertical-tail placement** are now documented in both the Gazebo/CAD and XFLR5 frames (vertical tail: XFLR5 frame only; Gazebo/CAD-frame root point remains `DATA_REQUIRED`) — `GEOMETRY.md` §5.
  - **Manufacturer wing/tail planform** (root/tip incidence, washout, dihedral, sweep, root/tip chord) is now documented from the manufacturer manual — `GEOMETRY.md` §5.
  - **Motor/propeller physical hub/shaft reference points** (force-application locations) are now documented, explicitly distinguished from the raw mesh bounding-box centers computed in the earlier deep-mesh-analysis pass (a ≈7.3 mm difference exists for the motor-center figure specifically — not silently unified) — `GEOMETRY.md` §7.
- **What is still open** (see `GEOMETRY.md` §11/§30/§32.11 for the full, itemized ledger): the mesh coordinate unit in the strict CAD-metadata-confirmed sense; the mesh origin's physical/CAD meaning; wing planform/aerodynamic reference area (explicitly not derived from mesh surface data); the motor thrust-axis as an *independently mesh-derived* confirmation (a directly-stated measured vector now exists from the master dataset, §7, which is stronger evidence than the shape-only `THRUST_AXIS_REQUIRES_CONFIRMATION` mesh evidence it sits alongside); precise SDF joint-axis fitting and sign-testing for all three hinge-equipped control surfaces; ESC and secondary (3S) battery positions; horizontal/vertical-tail incidence angle; motor mount/firewall/pylon geometry; servo mass/location.
- Full SDF-ready geometry breakdown (per-link dimensions, finalized hinge joint axes with sign-tested rotation direction, collision geometry): `DATA_REQUIRED`

## mass_properties/

- Aircraft mass: **6.000 kg**
- Gazebo/CAD reference CG: **(0.168309, 0, 0.100000) m** — origin/reference-frame definition as a named CAD datum: `DATA_REQUIRED`; a candidate shared reference point (main-wing-root LE) is now identified — see transform note below. **This is the CG designated for the SDF `<inertial><pose>`** (see `MASS_PROPERTIES.md` §3.4) — no conversion from the XFLR5 CG is applied to it.
- XFLR5 reference CG: **(0.0637, 0, -0.0210) m** — axis convention: **partially resolved** (X reversed vs. Gazebo/CAD +X, Z same-sense, Y untested; master dataset §2) — origin: **partially resolved** (identified as the main-wing-root-LE point, master dataset §8/§12). Used only for XFLR5-referenced aerodynamic/stability data (XNP, XCP, derivatives) — see `aerodynamics/AERODYNAMICS.md`.

**These two CG values use different reference frames/origins and remain non-interchangeable as general values** — `MASS_PROPERTIES.md` §3.3's warning stands in full. **Update, 2026-08-21 (master-dataset synchronization pass): a specific, derived X/Z transform between the two frames now exists** — `XFLR5_X = 0.23196 - Gazebo/CAD_X`, `XFLR5_Z = Gazebo/CAD_Z - 0.12103` (meters) — derived from the master dataset and cross-validated to sub-0.1 mm against two independent data points (the documented XFLR5 CG itself, and the horizontal-tail placement). This does **not** license substituting one CG for the other directly — it is a documented relationship that must be explicitly applied, not an equivalence. Y-axis behavior and any rotation beyond the stated X-sign reversal remain `DATA_REQUIRED`. Full derivation: `GEOMETRY.md` §8.3; mass-properties-side restatement: `MASS_PROPERTIES.md` §3.5.

- Inertia tensor (Ixx, Iyy, Izz, Ixy, Ixz, Iyz): **`V1_PROVISIONAL`** — Ixx=0.7284, Iyy=0.2507, Izz=0.9523, Ixy=0, Iyz=0, Ixz=0.01485 kg·m², about the Gazebo/CAD CG, Gazebo/CAD (FLU) frame (master dataset §9). Entered directly into XFLR5's Type-7 Mean-inertia field with Use Plane Inertia=OFF (Use Plane Inertia=ON produced zero inertia/eigenvalue problems in XFLR5) — not derived from a component mass distribution. **Usable for Gazebo V1 now; Inertia V2 (from real spar/servo/ESC/avionics/GPS/flight-controller/battery/wiring/motor/prop mass distribution) is still needed later but does not block V1.** Ixz sign-convention-vs-Gazebo/SDF is unverified and flagged as a prerequisite before SDF `<inertial>` use. Full detail: `MASS_PROPERTIES.md` §5.1.
- Main battery (4S 22000 mAh, 25C): mass ≈1.666 kg, center **(0.300631, 0, 0.038547) m** (Gazebo/CAD frame), CG-relative offset ΔX≈+0.132 m forward / ΔZ≈−0.061 m below (master dataset §7, §43). Secondary battery (3S 3300 mAh): mass ≈0.248 kg, position `DATA_REQUIRED` — not invented. Motor (≈0.143 kg each), ESC (≈0.080 kg each), and propeller (≈0.0301 kg each) masses are also now documented — see `MASS_PROPERTIES.md` §6. Known-component running subtotal ≈2.42 kg of the 6.000 kg total; airframe structure, servos, wiring, and avionics/GPS/flight-controller mass remain `DATA_REQUIRED`.

## aerodynamics/

Full consolidated reference: **`aerodynamics/AERODYNAMICS.md`** (created 2026-08-21; updated 2026-08-21 same day, master-dataset sync pass). Source analyses referenced as existing/expected for this project: XFOIL, XFLR5, full-aircraft stability analysis, wing analysis, horizontal tail analysis, vertical tail analysis, elevator analysis, rudder analysis, aileron analysis — **as of the master-dataset sync pass, all of these now have real content in the repository**, via `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` (2247 lines, §1–§74, ingested 2026-08-21). This supersedes the prior "nothing beyond the single trim point exists" finding. Raw XFOIL/XFLR5 project files themselves are still not in the repository (only the master dataset's summary/derived numeric content is) — remaining gaps are tracked in `AERODYNAMICS.md` §17.

Current full-aircraft / neutral-vertical-fin reference point (cross-checked against `CLAUDE.md` and, now, against the master dataset §30/§37/§70 — exact match, see `AERODYNAMICS.md` §14):

| Quantity | Value | Notes |
|---|---|---|
| mass | 6.000 kg | |
| trim velocity | 21.244 m/s | |
| trim alpha | 0.364 deg | |
| CL | 0.47167 | |
| XNP (neutral point) | 0.132 m | XFLR5 frame; qualitative XFLR5↔Gazebo axis convention now documented (X reversed vs. Gazebo/CAD FLU, origin at main-wing-root LE) but not yet a validated general transform — see `AERODYNAMICS.md` §9 |
| XCP (center of pressure) | 0.064 m | same frame/status as XNP above |
| CYb | -0.13216 | |
| Clb | -0.00717 | |
| Cnb | +0.03554 | |
| CYp | -0.04567 | |
| Clp | -0.54187 | |
| Cnp | -0.05878 | |
| CYr | +0.08776 | |
| Clr | +0.10586 | |
| Cnr | -0.02227 | |

This is still not, on its own, a complete aerodynamic model, but it is no longer the only aerodynamic data in the repository. **Corrected claim (was stale as of the 2026-08-21 initial pass): longitudinal derivatives, a full-aircraft CD value, and a mean aerodynamic chord now exist** — `CLa=+5.44594/rad`, `Cma=-1.65805/rad`, `CLq=+9.48457`, `Cmq=-10.22875`, a V1-calibrated full-aircraft drag polar `CD=CD0+k·CL²` (`CD0≈0.0351`, `k≈0.0528`, status `V1_CALIBRATED`, not flight-measured), and an aerodynamic reference chord `c_ref≈0.224 m` (distinct from the manufacturer manual's 0.176 m average chord — different definitions, do not conflate). `Cmα̇` remains the one longitudinal derivative still entirely absent. Control-surface derivatives now exist too: `Cmδe≈-0.73/rad` (neutral trim point, full ±10° sweep tabulated), `Clδa≈+0.308/rad`, `Cnδa≈+0.00144/rad`, `CYδa≈+0.0254/rad`, `CYδr≈+0.085/rad`, `Cnδr≈-0.025/rad`, `Clδr≈+0.0007/rad` — all reliable only within the analyzed ±10° deflection band (mechanical throw may be ±30° or more; no extrapolation past ±10°). Longitudinal and lateral-directional dynamic modes (short-period, phugoid, roll subsidence, Dutch roll, and a **mildly unstable spiral mode**, reported explicitly not hidden) are also now documented as `VALIDATION_TARGET`s. Full detail, all provenance, and the remaining `DATA_REQUIRED` items (`Cmα̇`, `CL0`/`Cm0` anchors, XFLR5↔Gazebo sign-convention unit-test resolution, rudder/aileron physical deflection-sign mapping, stall/post-stall model choice — explicitly deferred pending review, not decided here): see `AERODYNAMICS.md` §6–§18.

## propulsion/

Full consolidated reference: **`propulsion/PROPULSION.md`** (created 2026-08-21; synced 2026-08-21 against `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` §41–§63/§70–§72).

- 2 × SunnySky X2820 860KV motors — KV=860 rpm/V, internal resistance R≈0.0258 Ω, no-load current I0≈1.3 A @10V, mass≈0.143 kg/motor, max current≈65 A/30s, max power≈960 W/motor — all `CONFIRMED` manufacturer/web-collected component data (master dataset §41). Motor rotational inertia remains `DATA_REQUIRED` (V1-estimate/bench-calibration path only, master dataset §53).
- 2 × Hobbywing Skywalker 80A ESCs, ≈0.080 kg each, located inside the wings (qualitative only — exact positions `DATA_REQUIRED`). V1 model: `V_ESC = throttle × V_battery` (master dataset §42).
- 2 × APC 13x6.5E propellers — **nominal diameter D = 0.3302 m, nominal pitch = 0.1651 m — this is the only diameter used in propulsion physics.** Mass ≈0.0301 kg each. **Counter-rotating: left = CCW, right = CW (`CONFIRMED`, master dataset §44)** — the physically-reversed side requires a genuine reverse-handed APC 13x6.5EP part; reversing a normal 13x6.5E's electrical rotation direction alone is **not** an equivalent physical reverse propeller.
- 4S electrical system — main battery: **4S 22000 mAh, 25C**, Vnom=14.8 V, Vfull=16.8 V, mass≈1.666 kg, center **(0.300631, 0, 0.038547) m** (Gazebo/CAD frame), CG-relative offset ΔX≈+0.132 m forward / ΔZ≈−0.061 m below (master dataset §7/§43; published in `mass_properties/MASS_PROPERTIES.md` §6.1). Secondary battery: 3S 3300 mAh, mass≈0.248 kg, position unknown (`DATA_REQUIRED`, not invented).

**⚠ VISUAL_MESH_ONLY:** the propeller STL mesh (`model/meshes/left_pervane.stl` / `right_pervane.stl`) measures ≈273 mm diameter. This is a mesh artifact, not a physical propulsion reference, and must never be used in RPM, thrust, torque, advance ratio (J), disk area, tip speed, motor load, throttle→RPM, RPM→thrust, or airspeed-dependent-thrust calculations. `D = 0.3302 m` is used everywhere, always. See `PROPULSION.md` §0 and `geometry/GEOMETRY.md` §20–§21.2.

Target model chain (full form, master dataset §1 intro): `throttle → ESC duty-cycle/voltage → motor electrical model (Kt≈0.0111 N·m/A, V=I·R+Ke·ω) → motor current/torque → motor+propeller angular dynamics (I_rotor·dω/dt = Q_motor − Q_prop) → RPM(t) → advance ratio J → APC Ct(J)/Cp(J) → thrust + propeller torque → reaction torque → force/moment at the real hub location → Gazebo 6-DOF`. RPM is a solved ODE state, never set instantaneously; reaction torque must always be applied; differential-thrust yaw arises naturally from `r×F` at the real hub positions (±0.300 m lateral), not from a coded control derivative. Motor/prop hub coordinates (left (0.2951,+0.300,0.1271) m, right (0.2951,−0.300,0.1271) m) and thrust axis (≈+X, measured normal (0.999996,0.000018,−0.002668), ~0.15° vertical offset) are now published by `geometry-structure` in `geometry/GEOMETRY.md` §7, explicitly distinguished there from raw mesh bounding-box centers.

Static validation references (single operating points, not a full coefficient table): SunnySky bench ≈9230 RPM → ≈32.85 N/motor, ≈63.2 A, ≈935 W/motor; independent APC-official data interpolated to the same RPM → ≈31.32 N (≈4.7–5% difference between the two, both retained as references, not forced to match exactly).

Still `DATA_REQUIRED` (tagged `PROPULSION_DATA_REQUIRED`/`APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION` in `PROPULSION.md`): the real APC 13x6.5E Ct(J)/Cp(J) coefficient table (architecture is fully defined; only the numeric table is missing), motor/propeller rotational inertia, motor electrical/mechanical time constant, ESC dynamic-response characteristics (ramp/delay/efficiency/current-limiting — V2), exact ESC positions, exact secondary-battery position, battery internal resistance/SOC curve (V2), motor efficiency/thermal/friction-loss model, mounting angle beyond the ≈0.15° thrust-axis offset, and APC 13x6.5EP-specific performance data. **No longer** `DATA_REQUIRED` (corrected this pass): motor KV/internal resistance/no-load current/mass/max current/max power, ESC identity/mass/V1 model, battery Vnom/Vfull/mass, propeller rotation direction (left CCW / right CW).

## controls/

Full consolidated reference: **`controls/CONTROLS.md`** (created 2026-08-21; updated 2026-08-21 same day, master-dataset sync pass — 196 lines, up from 121) — a consolidated table covering all five control surfaces (left/right aileron, left/right elevator, rudder) plus the two continuous-rotation prop joints.

**Corrected claim (was stale): "all five control surfaces have `HINGE_REQUIRES_CONFIRMATION` with no data beyond a candidate mesh region" is obsolete.** The master dataset (§21 elevator, §29 rudder, §34 aileron, §66 movable-links list) provides real per-station hinge chordwise-location (%chord) data for all three deflecting surfaces, and confirms the full seven-member movable-link set (`left_aileron`, `right_aileron`, `left_elevator`, `right_elevator`, `rudder`, `left_prop`, `right_prop`) with mesh ready for each — see `CONTROLS.md` §0–§2 and `geometry/GEOMETRY.md` §6 (`HINGE_GEOMETRY_READY`, cross-owned).

Still genuinely open, unchanged in kind: a fitted SDF-ready hinge axis (position + direction vector, `geometry-structure`-owned, `CONTROLS.md` §2/§7/§8); elevator/rudder component scope (`geometry-structure` concluded movable-surface-only in this same pass — see `geometry/GEOMETRY.md` §26.3 — `CONTROLS.md` still shows the pre-conclusion framing and should be treated as pending a small follow-up sync); neutral (0°) pose for all five surfaces; the physical Gazebo joint sign for all five, now tracked per-surface as `ELEVATOR_SIGN_TEST_REQUIRED` / `AILERON_SIGN_TEST_REQUIRED` / `RUDDER_SIGN_TEST_REQUIRED` (`CONTROLS.md` §4); exact per-surface mechanical deflection limit beyond the shared manufacturer ±30°-or-more figure (`CONTROLS.md` §3, explicitly distinguished there from the ≈±10° aerodynamic-derivative high-confidence linear range — two different concepts, never conflated); aileron servo model, and per-surface allocation of the tail's "7× Emax ES08MAII" (new tag `SERVO_ALLOCATION_REQUIRES_CONFIRMATION`, `CONTROLS.md` §5); servo performance specs; control gains/mixer values; and the ArduPilot SITL parameter file / MAVLink channel mapping (`CONTROLS.md` §6, repository-wide search repeated, still zero hits).

---

## How this maps to agents

- `geometry-structure` reads/writes `geometry/` and `mass_properties/`.
- `aerodynamics` reads/writes `aerodynamics/`.
- `propulsion` reads/writes `propulsion/`.
- `controls-integration` reads/writes `controls/`.
- `validation` reads all of the above to cross-check implementation; it does not write here.

No implementation work (SDF, plugins, controllers) has started. This README will be updated as real source files are added to each subdirectory.

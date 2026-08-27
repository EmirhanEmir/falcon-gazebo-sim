# FALCON V2 — Aerodynamics Source of Truth

**Owner:** `aerodynamics`
**Status:** Updated 2026-08-21 (master-dataset sync pass). This document was first compiled 2026-08-21 from `CLAUDE.md`/`docs/source_of_truth/README.md` only (the single full-aircraft/neutral-vertical-fin reference point). It is now updated against the newly-added master dataset at `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` (2247 lines, Turkish, sections §1–§74, dated 2026-08-21), read directly and in full for this update. **This is a docs-only pass: no `model.sdf`, plugin/source code, world file, launch file, or ArduPilot config was created or modified, and no STL file was touched, in producing this update.**

**Master dataset citation convention used throughout this document:** `MD §NN` refers to a section of `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt`. All numeric values pulled from it are reproduced verbatim (no rounding changes, no unit conversions beyond what is explicitly shown).

**The master dataset's own governing rules are treated as binding on how its content is used in this document** (`MD` "VERİ ÖNCELİĞİ" preamble, lines 11–24):
1. Source priority: manufacturer manual > measured aircraft data > real motor/prop/component manufacturer data > STL geometry > XFOIL/XFLR5 analysis > calculation/V1 estimate.
2. Data from other Falcon/Talon projects must never be mixed in (none was — this document only draws from the FALCON V2 master dataset, `CLAUDE.md`, and `docs/source_of_truth/README.md`).
3. **Values labeled "V1", "yaklaşık" (approximate), "provisional", or "tahmin" (estimate) in the master dataset are explicitly *not final*.** This document preserves those labels wherever they appear — a V1/provisional/approximate/estimate value is never re-stated here as `CONFIRMED` final truth. Where the master dataset itself calls a value "V1_CALIBRATED", "PROVISIONAL", "APPROXIMATE", "ESTIMATE", or a "VALIDATION_TARGET" (not something to force-fit or re-tune to), that label is carried into this document's status column.

Status legend (extends the original legend used throughout this document, `GEOMETRY.md`, `MASS_PROPERTIES.md`):
- `CONFIRMED` — stated directly by an authoritative source, cross-checked, not qualified as provisional by that source.
- `V1_CALIBRATED` / `PROVISIONAL` / `APPROXIMATE` / `ESTIMATE` — carried verbatim from the master dataset's own qualification; usable in a Gazebo V1 model but explicitly not flight-validated truth, and never to be "tuned" to make a test pass (`CLAUDE.md` simulation tuning policy).
- `VALIDATION_TARGET` — a benchmark number to check a future implementation *against*, not a parameter to force-fit into the simulation.
- `V2_FUTURE_IMPROVEMENT` — explicitly deferred by the master dataset to a later modeling pass; not implemented, not estimated as a placeholder.
- `DERIVED` — computed from confirmed/labeled values, derivation shown.
- `DATA_REQUIRED` — not present anywhere in the repository — not guessed, not estimated.
- `CONFLICT_REQUIRES_RESOLUTION` — two authoritative-looking sources disagree.

No `ASSUMPTION` or `TEMPORARY` value is introduced anywhere in this document. Where the master dataset itself recommends a modeling choice (e.g., lookup/saturation instead of pure linear `CL`) that choice is reported as a recommendation requiring separate implementation review, not adopted here.

---

## 1. Purpose and Scope

This document is the aerodynamics-domain counterpart to `docs/source_of_truth/geometry/GEOMETRY.md` and `docs/source_of_truth/mass_properties/MASS_PROPERTIES.md` (both owned by `geometry-structure`), and to `docs/source_of_truth/propulsion/PROPULSION.md` and `docs/source_of_truth/controls/CONTROLS.md`. It catalogs all aerodynamic source data now available for FALCON V2 — manufacturer geometry, XFOIL airfoil-level data, XFLR5 per-surface and full-aircraft data, control-surface derivatives, dynamic modes, and the intended V1 coefficient build-up architecture — and separates confirmed/labeled source data from what remains `DATA_REQUIRED`. **It does not create any new aerodynamic coefficient, curve, or derivative not already present in `CLAUDE.md`, `docs/source_of_truth/README.md`, or the master dataset.** Ownership-boundary items (control-surface hinge geometry, CG, mesh placement) are cited, not re-derived or duplicated — see the per-section notes below.

### 1.1 Stale claim removed (first, per task instruction)

The prior version of this document (compiled 2026-08-21, before the master dataset was added) stated, in its former §5.1:

> "The nine derivatives listed (CYb, Clb, Cnb, CYp, Clp, Cnp, CYr, Clr, Cnr) are entirely lateral-directional... **No longitudinal stability derivative exists anywhere in the repository — no CLα, no Cmα, no CLq, no Cmq, no Cmα̇, no CD0, no induced-drag factor.**"

and correspondingly in the former §14 (`DATA_REQUIRED` list, item 16):

> "All longitudinal stability derivatives — CLα, Cmα, CLq, Cmq, Cmα̇ (§5.1) — none exist; only lateral-directional derivatives are present"

and `docs/source_of_truth/README.md`'s `aerodynamics/` bullet stated the same thing verbatim: *"no longitudinal derivative exists at all (no CLα, Cmα, CLq, Cmq, Cmα̇), no CD value, and no mean aerodynamic chord."*

**This is now obsolete and incorrect.** The master dataset provides `CLa`, `Cma`, `CLq`, `Cmq`, a full-aircraft `CD0`/`k` drag polar, and an aerodynamic reference chord `c_ref` (`MD §37`, `MD §70`, `MD §24`, `MD §38` — full values in §6 below). It does **not** provide `Cmα̇` (rate of change of pitching moment with angle-of-attack rate) — that specific derivative remains `DATA_REQUIRED` and is not fabricated here. This correction is carried through every section below and into `docs/source_of_truth/README.md`.

---

## 2. Manufacturer Geometry Reference

Unchanged from the prior version of this document — still `CONFIRMED`, still cross-checked, now additionally corroborated by the master dataset itself (`MD §3`, citing "Titan Dynamics Falcon V2 Build & User Manual Rev 1.0" as its own source):

| Parameter | Value | Unit | Provenance | Status |
|---|---|---|---|---|
| Wingspan | 2.093 | m | `CLAUDE.md`; `README.md`; `MD §3` (manual) | CONFIRMED |
| Wing area | 0.4514 | m² | `CLAUDE.md`; `README.md`; `MD §3` (manual) | CONFIRMED |

### 2.1 New from the master dataset — wing planform chord data (geometry-structure's domain, cited only)

`MD §3` additionally records manufacturer-manual wing planform values not previously in this document: root chord 0.260 m, tip chord 0.051 m, manual average chord 0.176 m, aspect ratio 9.71, sweep 3°, dihedral 0.5°, root incidence +4°, tip incidence 0°, geometric washout 4°, root airfoil NACA4411, tip airfoil NACA3411. **These are wing planform/CAD geometry values, owned by `geometry-structure`** (`CLAUDE.md` ownership boundary: "You do not... move geometry... You consume geometry (reference area/chord/span...) from it; you don't set it"). This document does not assert them as its own authoritative record — they are cited here only because §4/§5 below (airfoil identity, spanwise section model) need them for context. The authoritative recording location for wing planform geometry is `docs/source_of_truth/geometry/GEOMETRY.md` §15 (Wing Geometry); this document flags that a sync of these manual values into `GEOMETRY.md` is `geometry-structure`'s task, not performed here.

### 2.2 Aerodynamic reference chord `c_ref` — distinct from the manufacturer average chord (this agent's domain)

The master dataset explicitly and repeatedly warns these are **not the same quantity, and must not be conflated** (`MD §24`, line 748–749):

> "XFLR5 c_ref≈0.224 m, manual average chord 0.176 m ile aynı tanım değildir." (XFLR5's c_ref ≈0.224 m is not the same definition as the manual's average chord of 0.176 m.)

| Quantity | Value | Definition | Used for | Provenance | Status |
|---|---|---|---|---|---|
| `c_ref` (aerodynamic/stability reference chord) | ≈0.224 m | XFLR5's internal reference chord for the full-aircraft stability analysis and derivative non-dimensionalization | Static margin (§6.2), `q_hat` normalization, `My = qbar·S·c_ref·Cm` (§8) | `MD §24`, `MD §37`, `MD §70` | CONFIRMED (value), but its precise XFLR5-internal geometric definition — e.g. wing MAC vs. some other reference — is not spelled out beyond "XFLR5 c_ref" in the master dataset; treated as an opaque analysis-tool reference chord, not re-derived here |
| Manual average chord | 0.176 m | `(root chord + tip chord)/2`-style manufacturer manual average, wing-planform geometry | Not used in any derivative/force equation in this document | `MD §3` | CONFIRMED — geometry-structure's value, cited only |

**`c_ref = 0.224 m` is the value this document uses everywhere a "reference chord" appears in a force/moment equation (§8). The manufacturer's 0.176 m average chord is never substituted for it.**

---

## 3. Source Documents for This Update

| Source | What it provides |
|---|---|
| `CLAUDE.md` | Wingspan/wing area; the single full-aircraft/neutral-vertical-fin trim reference point (mass, trim V, trim alpha, CL, XNP, XCP, 9 lateral-directional derivatives); coordinate-system and simulation-tuning rules |
| `docs/source_of_truth/README.md` | Restates the same values; independently flagged the same gaps this update now partially resolves |
| `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` (`MD`, this update's primary new input) | Full XFOIL/XFLR5 analysis chain: wing 2D XFOIL, main-wing 3D VLM2, wing viscous drag, horizontal-tail geometry+airfoil, elevator Type7 trim sweep, longitudinal dynamic modes, vertical-tail geometry+XFOIL, full-aircraft neutral-fin stability derivatives, lateral dynamic modes, rudder Type7 sweep, aileron geometry+Type7 sweep, the Gazebo V1 derivative set, full-aircraft drag V1 model, 18 m/s benchmark, and explicit reliability/extrapolation limits |
| `docs/source_of_truth/geometry/GEOMETRY.md` | CAD/mesh cross-checks; control-surface hinge candidates (`geometry-structure`'s domain) — cited, not duplicated |
| `docs/source_of_truth/mass_properties/MASS_PROPERTIES.md` | Gazebo/CAD CG and XFLR5 CG, and the explicit rule against conflating them — cited, not duplicated |

**Cross-check result:** every value in `CLAUDE.md`/`README.md`'s existing reference point (mass, trim V, trim alpha, CL, XNP, XCP, and all 9 lateral-directional derivatives) is reproduced **exactly** inside `MD §30` (the "Neutral Fin Type 7 — Tam Stability Derivatives" section) and `MD §37`/`§70` (the Gazebo derivative set). No discrepancy found — see §14 (Cross-Check) below for the full side-by-side table.

---

## 4. XFOIL Airfoil-Level Data

### 4.1 Wing airfoils (`MD §13`)

Wing XFOIL run: 48 polar groups, 5986 points total (some low-point groups noted: `NACA2410_INV` at Re=300k and Re=550k, 95 points each). **Only representative/summary values are reproduced in the master dataset text — the full point-by-point Cl/Cd/Cm-vs-alpha tables are not themselves included in the master dataset file**, so a full multi-point polar curve is still not literally present in this repository; only the summary statistics below are.

| Airfoil | Re | CLmax | alpha at CLmax | CDmin | alpha0 (zero-lift) | Status |
|---|---|---|---|---|---|---|
| NACA4411 (wing root) | 150k | ≈1.424 | ≈13.25° | ≈0.0120 | ≈-4.02° | CONFIRMED (representative point) |
| NACA4411 | 200k | ≈1.430 | ≈15.25° | ≈0.00981 | ≈-4.37° | CONFIRMED (representative point) |
| NACA4411 | 300k | ≈1.427 | ≈14.75° | ≈0.00775 | ≈-4.55° | CONFIRMED (representative point) |
| NACA3411 (wing tip) | 40k | ≈1.254 | — | — | — | CONFIRMED (representative point) |
| NACA3411 | 50k | ≈1.288 | — | — | — | CONFIRMED (representative point) |
| NACA3411 | 60k | ≈1.299 | — | — | — | CONFIRMED (representative point) |
| NACA3411 | 75k | ≈1.330 | — | — | — | CONFIRMED (representative point) |
| NACA3411 | 100k | ≈1.356 | — | — | — | CONFIRMED (representative point); alpha0 ≈-3.3 to -3.5°, CLalpha ≈6.0–6.2/rad (2D) |
| NACA2410_INV (horizontal tail, inverted) | ≈200k (representative) | — | — | — | zero-lift ≈+1.8 to +2.1° | CL(-10°)≈-1.17, CL(-5°)≈-0.79, CL(0°)≈-0.30, CL(+5°)≈+0.44 — CONFIRMED (representative points) |

2D lift-curve slope for the cruise region, all wing airfoils: `CLalpha_2D ≈ 6.0–6.2 /rad` (`MD §13`).

The wing's 3D model is built from 9 spanwise stations transitioning root NACA4411 → morph sections `FALCON_MORPH_M3p850` … `FALCON_MORPH_M3p070` → tip NACA3411 (`MD §12`). The full chord/twist schedule at each station is wing planform geometry, owned by `geometry-structure`; not duplicated here beyond this airfoil-identity chain, which is needed to interpret the 2D polars above.

**Explicit caveat carried from the master dataset (`MD §13`, line 397): "Post-stall XFOIL verileri doğrudan güvenilir kabul edilmedi"** — post-stall XFOIL data is explicitly *not* treated as directly reliable. This document does not use any post-stall XFOIL value for anything, consistent with the project's stall/post-stall review requirement.

### 4.2 Vertical-tail airfoil (`MD §26`, `MD §27`)

Manufacturer manual does not name a vertical-tail airfoil. STL sections show an approximately symmetric section (root chord≈164.39 mm t/c≈10%, mid chord≈129.31 mm t/c≈9.99%, tip chord≈94.22 mm t/c≈10%) — **explicitly not labeled NACA0010** by the master dataset, only "yaklaşık simetrik / NACA0010-benzeri fakat NACA0010 olarak kabul edilmedi" (approximately symmetric / NACA0010-like, but not accepted as NACA0010). This document preserves that explicit non-naming; the vertical-tail airfoil identity itself remains `DATA_REQUIRED` beyond "approximately symmetric, ~10% t/c."

Clean re-derived XFOIL-ready profiles were built after raw-STL XFOIL problems (poor input coordinate distribution, excessive panel angle ≈-80.6°, solver NaN issues): `VTAIL_Z120.0_FIXED`, `VTAIL_Z130.5_TE_74.81`, `VTAIL_Z145.0_TE_72.75`, `VTAIL_Z180.0_TE_70.32`, `VTAIL_Z215.0_TE_67.24`, `VTAIL_Z250.0_TE_63.20`, `VTAIL_Z285.0_TE_57.64`, `VTAIL_Z299.0_TE_54.79` (`MD §26`).

Vertical-tail XFOIL run: 37 polar groups, 4167 points (low-point groups: `TEST_VTAIL_ROOT_CLEAN` Re=150k → 21 pts, `VTAIL_TIP_CLEAN` Re=225k → 99 pts).

| Quantity | Value | Provenance | Status |
|---|---|---|---|
| CLalpha | ≈8.2 /rad @ Re≈150k | `MD §27` | CONFIRMED (representative) |
| alpha0 | ≈0° | `MD §27` | CONFIRMED (representative) |
| CL(-10°) | ≈-0.885 | `MD §27` | CONFIRMED (representative) |
| CL(+10°) | ≈+0.885 | `MD §27` | CONFIRMED (representative) |

CDmin vs. Reynolds number trend:

| Re | CDmin |
|---|---|
| 50k | ≈0.0181 |
| 100k | ≈0.0130 |
| 150k | ≈0.0107 |
| 200k | ≈0.0096 |
| 250k | ≈0.0088 |
| 300k | ≈0.0076 |
| 350k | ≈0.0068 |

### 4.3 Still `DATA_REQUIRED` (§4)

- Full point-by-point Cl/Cd/Cm-vs-alpha tables for any airfoil (only summary/representative statistics exist in-repo — see §4.1/§4.2).
- Definitive vertical-tail airfoil identity (only "approximately symmetric, ~10% t/c, not NACA0010" is confirmed).
- Any airfoil polar beyond the wing (NACA4411/NACA3411/morph sections), horizontal tail (NACA2410_INV), and vertical tail (approx. symmetric) — i.e. this is now resolved at the identity level, not the full-curve level.

---

## 5. XFLR5 Wing/Tail-Level (Per-Surface) Data

Previously all `DATA_REQUIRED`. Now substantially populated:

### 5.1 Main wing 3D, Type 1 VLM2 (`MD §14`, `MD §15`, `MD §16`)

Conditions: Type 1 fixed-speed VLM2, thin surfaces, Neumann BC, viscous correction; speeds 12.5/15/18/20 m/s.

Representative CL vs. alpha (sparse — not an evenly-stepped polar; reproduced exactly as given, no interpolation performed by this document):

| alpha (deg) | CL |
|---|---|
| -8 | -0.193686 |
| -6 | -0.019100 |
| -5.5 | +0.024601 |
| 0 | 0.504441 |
| 5 | 0.934221 |
| 9 | 1.269198 |

3D `CLalpha ≈ 4.94 /rad`; zero-lift alpha ≈ -5.8°. **Note the 3D wing lift-curve slope (4.94/rad) is markedly lower than the 2D section slope (6.0–6.2/rad, §4.1) — this is the expected finite-aspect-ratio reduction (AR≈9.71) and is not a conflict.**

**High-alpha reliability limit (`MD §15`):** viscous-interpolation warnings begin around alpha ≈9.5°. The master dataset's own conclusion: results at **alpha ≤ ≈9°** are the reliable attached-flow XFLR5 range; **alpha ≥ ≈9.5° must not be used directly for post-stall validation.** This document adopts that same limit — no wing CL/CD value above ≈9° is treated as reliable anywhere in this document, and none is extrapolated past it.

Wing-only viscous drag near alpha≈0 (`MD §16`) — **explicitly wing-only, never substituted for full-aircraft drag (§6.5)**:

| V (m/s) | CD | CDv |
|---|---|---|
| 12.5 | 0.019408 | 0.010732 |
| 15 | 0.018264 | 0.009587 |
| 18 | 0.017356 | 0.008680 |
| 20 | 0.016914 | 0.008237 |

`CDi ≈ 0.008677` (wing-only induced drag, representative). `CDmin ≈ 0.01435` @ alpha≈-2.5° (CL≈0.287) at 12.5 m/s. Wing-only `L/Dmax ≈ 26` @ alpha≈0.5°.

### 5.2 6 kg required-CL and stall speed (`MD §17`, `MD §18`)

Using rho=1.225 kg/m³, S=0.4514 m², W≈58.86 N:

| V (m/s) | CLreq | Approx. required alpha |
|---|---|---|
| 12.5 | ≈1.36 | >9° / critical region (past the §5.1 reliability limit) |
| 15 | ≈0.946 | ≈5.1° |
| 18 | ≈0.657 | ≈1.8° |
| 20 | ≈0.532 | ≈0.3° |

Stall speed, using manual `CLmax=1.42` (`CLmax` itself is explicitly a manufacturer-manual performance-calc input, not a flight measurement — `MD §3`): `Vstall ≈12.24 m/s` (≈44.1 km/h). With a 20% margin: `Vmin_safe ≈14.69 m/s` (≈52.9 km/h).

### 5.3 Horizontal tail (`MD §19`)

Airfoil: **NACA2410_INV** (inverted), confirmed manufacturer-manual value (`MD §3`, `MD §19`).

Aerodynamic-reference shape (from the final XFLR5 model, `MD §19`): span ≈0.560 m, area ≈0.07875 m², MAC ≈0.1501 m, AR ≈3.98, root-to-tip sweep ≈20.02°.

**Spatial placement (root LE STL coordinates, XFLR5 Plane-Editor X/Z location, tail arm, tail volume — `MD §20`) is placement/position data. Per this agent's ownership boundary, this document does not restate or assert those position numbers as its own authoritative record — see `docs/source_of_truth/geometry/GEOMETRY.md` §17 (Horizontal Tail / Elevator Geometry) for the CAD/mesh-side placement record, and `MD §20` for the master dataset's own XFLR5-side placement numbers.** (For context only: `MD §20` reports an XFLR5-computed tail volume coefficient ≈0.42 and elevator lever arm ≈0.54 m, cross-checked against an independent STL-based tail-arm estimate of ≈0.549 m — a non-dimensional/derived aerodynamic design parameter, reported here only because it is not itself a coordinate.)

### 5.4 Vertical tail (`MD §26`)

Aerodynamic-reference shape, final exposed model: span ≈0.1790 m, area ≈0.02385 m², MAC ≈0.13941 m, AR ≈1.344. (An older, cruder STL-based estimate of area≈0.0301 m² included body-blend material and is superseded by this final exposed-model figure, per the master dataset's own priority rule.)

Spatial placement (Fin X≈0.5537 m, Y=0, Z≈-0.0010 m, Fin Tilt=0°, `MD §28`) is again placement data — cited to `MD §28`, authoritative CAD-side record is `GEOMETRY.md` §18 (Vertical Tail / Rudder Geometry), not duplicated here.

### 5.5 Aileron geometry (`MD §33`) — reproduced directly, per this agent's need to interpret the Type 7 sweep in §7.3

| Quantity | Value |
|---|---|
| Each aileron span | ≈0.4794 m |
| Each aileron area | ≈0.0270 m² |
| Total aileron area (both) | ≈0.0540 m² |
| Average aileron chord | ≈0.0566 m |
| STL actual span (y-range) | ≈310.4 → 789.7 mm |
| XFLR5 active boundary (y-range) | 0.313950 → 0.784875 m |

### 5.6 Still `DATA_REQUIRED` (§5)

- Per-surface downwash/sidewash interaction data (e.g. tail-in-wing-wake effects) — not present anywhere in the master dataset.
- Full (non-representative, dense) CL/CD/Cm-vs-alpha curves for wing-only, horizontal-tail-only, or vertical-tail-only analyses beyond the representative points given.
- Per-surface local lift/drag span-load distribution.

---

## 6. XFLR5 Full-Aircraft Stability Analysis (Neutral Vertical-Fin Configuration)

### 6.1 The original single reference point — restated, now with full context

The full-aircraft/neutral-vertical-fin trim/reference point from `CLAUDE.md`/`README.md` (mass 6.000 kg, trim V 21.244 m/s, trim alpha 0.364°, CL 0.47167, XNP 0.132 m, XCP 0.064 m, and the 9 lateral-directional derivatives CYb/Clb/Cnb/CYp/Clp/Cnp/CYr/Clr/Cnr) is reproduced exactly, at higher precision, inside `MD §30` ("Neutral Fin Type 7 — Tam Stability Derivatives", V∞=21.244 m/s, alpha=0.364°, XNP=0.132 m, XCP=0.064 m, CL=0.47167) — see §14 for the full cross-check table. **It remains, as before, a single trim/reference operating point from a full-aircraft analysis, not by itself a complete polar** — but it is no longer the *only* aerodynamic data in the repository; §6.2–§6.10 below is what the master dataset adds around it.

### 6.2 Full-aircraft stability-derivative set (`MD §30`, `MD §37`, `MD §70` — all three internally consistent, no conflict)

Reference quantities used for every coefficient below: `S = 0.4514 m²`, `b = 2.093 m`, `c_ref ≈ 0.224 m` (§2.2). **This resolves a previously-open question in this document (former §9/§10): it is now `CONFIRMED` that these are the exact reference values XFLR5 used internally for this derivative set** (`MD §37`, `MD §70`), not merely the manufacturer wingspan/area asserted to match by assumption.

**Longitudinal:**

| Derivative | Value | Meaning |
|---|---|---|
| `CLa` | +5.44594 /rad | Lift-curve slope, full aircraft |
| `Cma` | -1.65805 /rad | Pitching-moment slope w.r.t. alpha (static longitudinal stability) |
| `CLq` | +9.48457 | Lift due to pitch rate (`q_hat`-normalized) |
| `Cmq` | -10.22875 | Pitch damping |
| `CXa` | +0.31835 | Body-axis X-force derivative w.r.t. alpha |
| `CXq` | +0.39295 | Body-axis X-force derivative w.r.t. pitch rate |
| `Cm_delta_e` (neutral) | ≈-0.73 /rad | Elevator pitching-moment effectiveness at the neutral trim point (see §7.1 for the full deflection-dependent sweep) |

**Not carried into the "Gazebo main derivative set" by the master dataset itself** (present in the raw XFLR5 output at `MD §30` but explicitly excluded from `MD §37`'s "Longitudinal" list and from `MD §70`'s summary block) — reported here for provenance completeness only, not adopted into any V1 equation in §8:

| Derivative | Value |
|---|---|
| `CXu` | -0.01645 |
| `CLu` | -0.00009 |
| `Cmu` | 0 |

**Note on mixed force-axis convention (flagged, not resolved):** `CXa`/`CXq` are **body-axis** X-force derivatives, while `CL` (lift) and the drag polar (§6.5) are conventionally **wind-axis** quantities. The master dataset presents both `CXa`/`CXq` and `CLa`/`CLq`/the CD0+k·CL² polar side by side (`MD §37`) without stating how the two axis systems are to be reconciled in a single force model. **This is `DATA_REQUIRED`/an open modeling question, not silently resolved here** — before both are used together in a Gazebo force model, whether `CXa`/`CXq` are meant as an alternative body-axis path (bypassing CL/CD entirely) or a supplementary term must be explicitly decided and documented; this document does not choose one.

**Sideforce / Roll / Yaw (identical values to `CLAUDE.md`'s existing reference point, cross-checked — see §14):**

| | CYβ / Clβ / Cnβ | CYp / Clp / Cnp | CYr / Clr / Cnr | CY𝛿a / Cl𝛿a / Cn𝛿a | CY𝛿r / Cl𝛿r / Cn𝛿r |
|---|---|---|---|---|---|
| Sideforce | CYb = -0.13216 | CYp = -0.04567 | CYr = +0.08776 | CYda ≈ +0.0254 | CYdr ≈ +0.085 |
| Roll | Clb = -0.00717 | Clp = -0.54187 | Clr = +0.10586 | Clda ≈ +0.308 | Cldr ≈ +0.0007 |
| Yaw | Cnb = +0.03554 | Cnp = -0.05878 | Cnr = -0.02227 | Cnda ≈ +0.00144 | Cndr ≈ -0.025 |

Physical interpretation stated directly in the master dataset (`MD §30`): Cnβ>0 → directionally statically stable; Cnr<0 → yaw damping stable; Clβ<0 → weak positive dihedral effect; Clp<0 → strong roll damping.

### 6.3 Static margin, neutral point, center of pressure (`MD §23`, `MD §24`)

At the neutral trim point: `XNP ≈ 0.1319 m`, `XCP ≈ 0.0635–0.064 m`, `CGx = 0.0637 m` — **all three in the same XFLR5 reference frame** (this matters — see §9 for why mixing this with the Gazebo/CAD-frame CG would be an error).

`SM = (XNP - XCG)/c_ref ≈ (0.13186 - 0.0637)/0.224 ≈ 0.304 ≈ 30.4%` — a strong positive longitudinal static margin. **`c_ref` here is the 0.224 m XFLR5 aerodynamic reference chord (§2.2), explicitly not the manufacturer's 0.176 m average chord** — the master dataset itself calls this out (`MD §24`) as a reminder not to conflate the two chord definitions.

### 6.4 Longitudinal and lateral-directional dynamic modes

**Longitudinal (`MD §25`):**

| Mode | λ (1/s) | Frequency | Damping ζ | Notes |
|---|---|---|---|---|
| Short-period | -5.67462 ± 13.25078i | fn≈2.294 Hz, fd≈2.109 Hz | ≈0.394 | Stable |
| Phugoid | -0.00185 ± 0.61576i | f≈0.098 Hz | ≈0.003 | Stable but very lightly damped |
| Phugoid (Phillips method cross-check) | -0.00253 + 0.60270i | fn≈0.096 Hz | ≈0.004 | Cross-check, same conclusion |

**Lateral-directional (`MD §31`):**

| Mode | λ (1/s) | Frequency / time constant | Damping ζ | Notes |
|---|---|---|---|---|
| Roll subsidence | -9.46425 | τ≈0.106 s | — | Stable |
| Dutch roll | -0.30549 ± 3.20273i | fn≈0.512 Hz, fd≈0.510 Hz | ≈0.095 | Stable, lightly damped, period≈1.96 s |
| Dutch roll (Phillips cross-check) | -0.31706 + 3.17052i | fn≈0.507 Hz, fd≈0.505 Hz | ≈0.100 | Cross-check, same conclusion |
| **Spiral** | **+0.08227** | doubling time ≈8.43 s | — | **Mildly unstable** — reported explicitly, not hidden. A slow, mild spiral divergence is common and often acceptable for small fixed-wing UAVs; this is a `VALIDATION_TARGET` characteristic of the aircraft as analyzed, not a defect to "fix" by adjusting a derivative (`CLAUDE.md` simulation tuning policy: never adjust a coefficient to make the aircraft "stable"). |

All dynamic-mode values above are `VALIDATION_TARGET`s for a future linearized-model check against the implemented Gazebo simulation — they are not parameters to be coded directly into the physics engine, and are not to be force-fit if a future simulation doesn't reproduce them exactly.

### 6.5 Full-aircraft drag model — V1 (`MD §38`, `MD §39`)

**Critical distinction, stated repeatedly by the master dataset and preserved here:** the Type 7 control-sweep CD values referenced throughout this document (elevator §7.1, rudder §7.2, aileron §7.3) are **inviscid (CDv=0, CD=CDi only)** — they are XFLR5 induced-drag-only values from an inviscid VLM control sweep, and **must never be treated as real total-aircraft drag.**

The master dataset's separate, explicitly-labeled **V1 calibrated** full-aircraft drag polar:

```
CD = CD0 + k·CL²
CD0 ≈ 0.0351
k   ≈ 0.0528
```
Status: **`V1_CALIBRATED`** — a calibration model, not flight-measured truth, per the master dataset's own label.

Cross-check against the neutral-fin inviscid XFLR5 point (CL≈0.47167, CDi≈0.008226): back-solving gives `k_XFLR5 ≈ 0.0370`, `e_XFLR5 ≈ 0.887` (inviscid-only Oswald efficiency) vs. the manual-calibrated V1's implied `e ≈ 0.62` order of magnitude. The master dataset attributes this gap explicitly to: fuselage drag, motor/nacelle drag, antenna/protrusions, surface roughness, printed-structure excrescence drag, trim/interference losses, and XFLR5's inviscid/ideal assumptions (`MD §38`) — i.e. `CD0`/`k` folds in everything the inviscid VLM sweep cannot see. This is **not** presented as more "correct" than the inviscid number in an absolute sense — it is a separate calibration model for a separate purpose (total-aircraft drag), and is labeled as such everywhere it is used.

### 6.6 6 kg level-flight drag table — V1 (`MD §39`)

| V (m/s) | CLreq | CD | Parasite drag (N) | Induced drag (N) | Total drag (N) | Aero power (W) |
|---|---|---|---|---|---|---|
| 12.5 | ≈1.363 | ≈0.133 | ≈1.52 | ≈4.23 | ≈5.75 | ≈71.9 |
| 15 | ≈0.946 | ≈0.082 | ≈2.18 | ≈2.94 | ≈5.12 | ≈76.9 |
| 18 | ≈0.657 | ≈0.058 | ≈3.14 | ≈2.04 | ≈5.19 | ≈93.4 |
| 20 | ≈0.532 | ≈0.050 | ≈3.88 | ≈1.65 | ≈5.54 | ≈110.7 |
| 22 | ≈0.440 | ≈0.045 | ≈4.70 | ≈1.37 | ≈6.06 | ≈133.4 |
| 25 | ≈0.341 | ≈0.041 | ≈6.07 | ≈1.06 | ≈7.12 | ≈178.1 |

Best L/D / minimum drag: ≈15–17 m/s, `Dmin ≈ 5.0–5.1 N`, `L/Dmax ≈ 11.5–11.7`. Status: `V1_CALIBRATED`, same caveats as §6.5.

### 6.7 Nominal 18 m/s validation benchmark (`MD §40`)

Mass=6.0 kg, V≈18 m/s, CL≈0.657, drag≈5.19 N, required total thrust≈5.19 N (≈2.595 N per motor), elevator trim≈-8°. XFLR5 cross-check at delta_e=-8°: V≈18.162 m/s, alpha≈2.472° (`MD §22`'s own elevator sweep table, delta_e=-8 row).

**Status: `VALIDATION_TARGET`.** This is registered here as the primary future straight-and-level Gazebo benchmark — **it is not run as part of this docs-only pass**; running/scoring it is `gazebo-testing`'s responsibility once implementation exists.

### 6.8 Consolidated dynamic-mode + trim benchmark table (for future `validation` use)

All entries below are `VALIDATION_TARGET`s, consolidated from §6.4/§6.7/§6.2 for convenience — none of them is a tuning target to force-fit:

| Benchmark | Target value |
|---|---|
| 18 m/s straight-level: CLreq | ≈0.657 |
| 18 m/s straight-level: drag / required thrust | ≈5.19 N total, ≈2.595 N/motor |
| 18 m/s straight-level: elevator trim | ≈-8° |
| Neutral-elevator trim: V, alpha, CL | ≈21.244 m/s, ≈0.365°, ≈0.4717 |
| Longitudinal: Cma, Cmq | ≈-1.658, ≈-10.229 |
| Short-period ζ | ≈0.394 |
| Phugoid ζ | ≈0.003 |
| Lateral: Cnβ, Cnr, Clp | ≈+0.03554, ≈-0.02227, ≈-0.54187 |
| Dutch roll: fn, ζ, period | ≈0.512 Hz, ≈0.095, ≈1.96 s |
| Roll subsidence: λ, τ | ≈-9.464, ≈0.106 s |
| Spiral: λ, doubling time | ≈+0.08227, ≈8.43 s (mildly unstable — expected, not a bug) |

---

## 7. Control-Surface Derivative/Polar Data

Previously entirely `DATA_REQUIRED` beyond the single trim point. Now substantially populated for elevator, rudder, and aileron.

### 7.1 Elevator — Type 7 trim sweep (`MD §21`–`§24`)

**Hinge/geometry context (geometry-structure's domain, cited only):** global hinge X ≈ -472.6835 mm; movable elevator span y≈50.60→240.00 mm; hinge x/c varies from ≈74.71% (root) to ≈62.54% (tip). TE flap, LE flap OFF, hinge at 50% thickness. See `GEOMETRY.md` §17 for the CAD-side record.

**XFLR5 sign convention, stated explicitly (`MD §22`, lines 601–603): `+ = trailing edge down`, `- = trailing edge up`.** This is XFLR5's own convention for the `delta_e` values below. **Mapping it to an actual Gazebo elevator joint rotation sign is now RESOLVED** — task `CONTROL_SURFACE_SIGN_MAPPING` (§19.13): `elevator_sign` corrected `+1.0→-1.0`, `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST`.

Full trim-sweep table. **Each row is a distinct trimmed operating point** (a different `delta_e` requires a different trim `alpha`/`V` for 1g level flight at 6 kg) — `CLa`, `Cma`, `Cmq`, `NP`, `CMde` are each a *local* linearization evaluated at that trim point, not a single global constant reused across the sweep:

| delta_e (deg) | alpha (deg) | V (m/s) | CL | CLa (/rad) | Cma (/rad) | Cmq | NP (m) | CMde (/rad) |
|---|---|---|---|---|---|---|---|---|
| -10 | 3.22252 | 17.35812 | 0.706701 | 5.3001 | -1.4719 | -8.1193 | 0.12588 | -0.6014 |
| -8 | 2.47198 | 18.16224 | 0.645357 | 5.3193 | -1.4704 | -8.2518 | 0.12559 | -0.60631 |
| -6 | 1.75144 | 19.05573 | 0.586190 | 5.3389 | -1.4746 | -8.4249 | 0.12554 | -0.61392 |
| -4 | 1.09089 | 20.00777 | 0.531726 | 5.3609 | -1.4904 | -8.6828 | 0.12594 | -0.62752 |
| -2 | 0.59568 | 20.82652 | 0.490769 | 5.3925 | -1.5402 | -9.1941 | 0.12764 | -0.65898 |
| 0 (neutral) | 0.36455 | 21.24412 | 0.471685 | 5.4459 | -1.6581 | -10.229 | 0.13186 | -0.7282 |
| +2 | -1.17453 | 24.89660 | 0.343639 | 5.407 | -1.4944 | -9.1398 | 0.12558 | -0.65551 |
| +4 | -2.40748 | 29.77044 | 0.240542 | 5.3898 | -1.4068 | -8.6303 | 0.12214 | -0.62568 |
| +6 | -3.49448 | 37.80907 | 0.149303 | 5.3814 | -1.3531 | -8.3751 | 0.11999 | -0.61394 |
| +8 | -4.55164 | 59.54707 | 0.060317 | 5.3745 | -1.3098 | -8.2005 | 0.11826 | -0.60812 |
| +10 | zero-moment alpha≈-5.60981 | — | negative-lift trim skipped | — | — | — | — | — |

**The master dataset's own chosen single V1 constant for the Gazebo derivative set is the neutral (`delta_e=0`) row's `Cm_delta_e ≈ -0.73 /rad`** (rounded from -0.7282) — i.e., the sweep's point-to-point variation (-0.60 to -0.73 across the table) is deliberately collapsed to one representative constant for the linear V1 model, not carried through as a `delta_e`-dependent function. This document preserves that choice and flags it explicitly rather than silently treating `Cmde` as a universal constant.

Practical trim table (`MD §22`): ≈18.2 m/s → elevator≈-8°; ≈19.1 m/s → ≈-6°; ≈20.0 m/s → ≈-4°; ≈20.8 m/s → ≈-2°; ≈21.24 m/s → 0°.

### 7.2 Rudder — Type 7 sweep (`MD §29`, `MD §32`)

**Hinge/geometry context (cited only, geometry-structure's domain):** global hinge X approximately constant; x/c varies with taper from ≈74.81% (Z=130.5mm) to ≈54.79% (Z=299.0mm). TE flap, LE OFF, hinge at 50% thickness. See `GEOMETRY.md` §18.

Conditions: all rudder flap gains=1, elevator controls=0, Type7 VLM2 **inviscid** (CD values are CDi only — see §6.5 caveat).

| delta_r (deg) | CY | Cl | Cn |
|---|---|---|---|
| -10 | -0.014982 | -0.000133 | +0.004437 |
| -8 | -0.011838 | -0.000101 | +0.003505 |
| -6 | -0.008817 | -0.000072 | +0.002610 |
| -4 | -0.005860 | -0.000045 | +0.001734 |
| -2 | -0.002943 | -0.000020 | +0.000870 |
| 0 | -0.000020 | 0 | +0.000007 |
| +2 | +0.002908 | +0.000020 | -0.000859 |
| +4 | +0.005827 | +0.000045 | -0.001723 |
| +6 | +0.008785 | +0.000071 | -0.002599 |
| +8 | +0.011798 | +0.000099 | -0.003491 |
| +10 | +0.014916 | +0.000128 | -0.004416 |

Approximate derivatives (full ±10° range): `CY_delta_r ≈ +0.085 /rad`, `Cn_delta_r ≈ -0.025 /rad`, `Cl_delta_r ≈ +0.0007 /rad`. Near-center (±2°): `CYdr≈+0.084`, `Cndr≈-0.0248`, `Cldr≈+0.0006` — response is "yaklaşık lineer ve simetrik" (approximately linear and symmetric), main effect is yaw+sideforce, roll coupling very small. Longitudinal trim barely changes with rudder deflection (neutral alpha≈0.364°/V≈21.244 m/s vs. ±10° rudder alpha≈0.329–0.330°/V≈21.31 m/s).

### 7.3 Aileron — geometry + Type 7 sweep (`MD §33`–`§36`)

Geometry: see §5.5 (span/area/chord). Hinge: y=0.313950→70.65% chord peak, ranging ≈69.75–72.14% chord band (`MD §34`); TE flap, LE OFF, hinge at 50% thickness. XFLR5 flap-group differential mapping (`MD §35`): `WF1=+1, WF2=+1, WF3=+1, WF4=-1, WF5=-1, WF6=-1` — accepted by the master dataset as producing a symmetric roll moment; **the physical (which-side-up, which-side-down) meaning of this sign mapping is not finalized** and is not asserted here.

Full Type7 sweep (inviscid VLM2, -10→+10°, step 2 — CDi shown is inviscid-only, §6.5 caveat applies):

| delta_a (deg) | alpha (deg) | V (m/s) | CL | CDi | CY | Cl | Cn |
|---|---|---|---|---|---|---|---|
| -10 | 1.565 | 20.5907 | 0.501949 | 0.013975 | -0.004436 | -0.053400 | -0.000242 |
| -8 | 1.489 | 20.6144 | 0.500838 | 0.012223 | -0.003556 | -0.042925 | -0.000194 |
| -6 | 1.396 | 20.6416 | 0.499550 | 0.010894 | -0.002678 | -0.032459 | -0.000146 |
| -4 | 1.268 | 20.6841 | 0.497519 | 0.009991 | -0.001803 | -0.021996 | -0.000099 |
| -2 | 1.005 | 20.8120 | 0.491440 | 0.008942 | -0.000946 | -0.011504 | -0.000051 |
| 0 | 0.421 | 21.1887 | 0.474152 | 0.008303 | -0.000011 | 0 | +0.000006 |
| +2 | 1.005 | 20.8120 | 0.491439 | 0.008984 | +0.000923 | +0.011505 | +0.000064 |
| +4 | 1.268 | 20.6841 | 0.497519 | 0.009987 | +0.001782 | +0.021996 | +0.000112 |
| +6 | 1.396 | 20.6416 | 0.499551 | 0.010885 | +0.002658 | +0.032459 | +0.000159 |
| +8 | 1.489 | 20.6144 | 0.500838 | 0.012212 | +0.003536 | +0.042926 | +0.000207 |
| +10 | 1.565 | 20.5907 | 0.501949 | 0.013962 | +0.004416 | +0.053400 | +0.000255 |

Approximate derivatives (full ±10° range): `Cl_delta_a ≈ +0.308 /rad`, `Cn_delta_a ≈ +0.00144 /rad`, `CY_delta_a ≈ +0.0254 /rad`. Near-center: `Clda≈+0.330`, `Cnda≈+0.00165`, `CYda≈+0.0268`. Strong roll control; yaw coupling small but non-zero; sweep symmetric. **Adverse/proverse labeling requires a finalized physical side/deflection mapping and is explicitly not asserted here from sign alone** (`MD §36`).

### 7.4 Still `DATA_REQUIRED` (§7)

- Control-surface hinge-moment data (any surface).
- Control-effectiveness variation vs. Reynolds number/airspeed (the sweeps above are single-condition/near-trim-speed only).
- A validated, ±30°-mechanical-range aerodynamic model — see §10 (reliability limits) for why this is explicitly *not* extrapolated from the ±10° sweep data above.
- ~~The XFLR5-to-Gazebo sign mapping for rudder and aileron deflection (only elevator's `+ = TE down` is explicitly stated in the master dataset).~~ — **RESOLVED** for the Gazebo joint-to-deflection direction of all three surfaces (task `CONTROL_SURFACE_SIGN_MAPPING`, §19.13); the XFLR5-internal WF1..WF6 adverse/proverse physical labeling question (§7.3, `MD §36`) is a separate, still-open item, unrelated to the Gazebo joint sign.

---

## 8. Aerodynamic Coefficient Build-Up Architecture — Intended V1 (`MD §37`)

This is the master dataset's own stated intended architecture, reproduced here as the aerodynamics-domain design reference. **No plugin/code implements any of this yet — this is documentation of the intended V1 approach, not a report of running code.**

### 8.1 Relative wind, dynamic pressure, angles

```
Vrel = Vbody - Vwind
V    = |Vrel|
qbar = 0.5 * rho * V^2
alpha = atan2(w, u)
beta  = asin(v/V)   (or a numerically safer atan2 form)
```

where `u, v, w` are the components of `Vrel` in the body frame. **Explicit, unresolved flag — the master dataset itself states this must be verified, not assumed (`MD §37`, line 1271: "Gazebo sign convention unit test ile doğrulanmalı" — must be validated with a unit test):**

The `alpha = atan2(w,u)` / `beta = asin(v/V)` forms as written are the standard textbook forms for an **FRD** body axis (X forward, Y right, Z down). FALCON V2's body frame is **FLU** (X forward, Y left, Z up — `CLAUDE.md`). Both the Z axis (up vs. down) and the Y axis (left vs. right) are flipped relative to FRD. A literal, unadjusted copy of these formulas into an FLU implementation risks an inverted sign for both `alpha` and `beta` relative to the standard "positive alpha = nose up relative to the wind" / "positive beta = wind from the right" convention. **This is flagged here, not fixed here** — resolving it is exactly what `AOA_SIGN_TEST`/`SIDESLIP_SIGN_TEST` (`gazebo-testing`) exist for, per the project workflow, and this document does not pre-judge the outcome.

### 8.2 Normalized rates — preserve exactly (`MD §37`)

```
p_hat = p * b / (2V)
q_hat = q * c_ref / (2V)
r_hat = r * b / (2V)
```

`b = 2.093 m`, `c_ref ≈ 0.224 m` (§2.2). Degrees/radians must not be mixed — all control-surface and rate derivatives above are per-radian (`MD §72`).

### 8.3 Coefficient build-up (V1, as stated in the master dataset)

```
CY = CYb*beta + CYp*p_hat + CYr*r_hat + CYda*delta_a + CYdr*delta_r
Cl = Clb*beta + Clp*p_hat + Clr*r_hat + Clda*delta_a + Cldr*delta_r
Cn = Cnb*beta + Cnp*p_hat + Cnr*r_hat + Cnda*delta_a + Cndr*delta_r
Cm = Cm0 + Cma*alpha + Cmq*q_hat + Cmde*delta_e
CL = CL0 + CLa*alpha + CLq*q_hat + (elevator lift contribution)
```

**`CL0` and `Cm0` are not given as standalone constants anywhere in the master dataset.** They are not fabricated here. A `CL0` could in principle be *derived* from the neutral trim point (`CL=0.471685` at `alpha=0.36455°`, with `CLa=5.4459/rad`, §7.1's `delta_e=0` row) via `CL0 = CL_trim - CLa * alpha_trim(rad)`, but **that derivation has not been performed or authorized in this pass** — it is flagged here as a `DATA_REQUIRED`/derivation-pending item for whoever implements the plugin, not silently assumed to be zero or computed and inserted here.

**The master dataset itself flags the pure-linear `CL` form as a risk, not a recommendation (`MD §37`, line 1284): "Ancak V1 için lookup/saturation daha güvenli olabilir"** (a lookup/saturation approach may be safer for V1). This document surfaces that recommendation; it does not implement either the linear form or a lookup/saturation form — per the project's rule that stall/post-stall and any related modeling decision requires explicit review (by the user and `validation`) before implementation, not unilateral action by this agent.

### 8.4 Forces and moments

```
L  = qbar * S * CL
D  = qbar * S * CD
Y  = qbar * S * CY

Mx = qbar * S * b     * Cl
My = qbar * S * c_ref * Cm
Mz = qbar * S * b     * Cn
```

`CD` here is the full-aircraft V1 calibrated drag polar (§6.5) — **not** any of the inviscid Type7 sweep CD/CDi values (§6.5/§7.1–§7.3).

---

## 9. Reference-Frame Distinction — XFLR5 vs. Gazebo/CAD

The project's existing strict rule against conflating Gazebo/CAD-frame and XFLR5-frame quantities (`CLAUDE.md`; `MASS_PROPERTIES.md` §3.3; `GEOMETRY.md` §8.3) still applies in full. The master dataset adds documented, but still only partially validated, context:

- **`MD §2` states explicitly: "XFLR5 +X yönü STL +X yönüne ters / kuyruğa doğru kabul edildiği için CG ve tail dönüşümlerinde bu işaret farkı dikkate alındı."** — XFLR5's +X direction is treated as *reversed* relative to STL/Gazebo +X (i.e., XFLR5 +X points toward the tail, opposite of the Gazebo/CAD FLU +X-forward convention), and this sign difference was accounted for in the CG and tail conversions the master dataset performed.
- **`MD §8` shows the specific arithmetic**: main-wing root LE in STL/Gazebo frame ≈ (X=+231.96 mm, Z=+121.03 mm); Gazebo/CAD CG = (0.168309, 0, 0.100000) m (`MD §6`, matches `MASS_PROPERTIES.md` §3.1 exactly); STL-frame delta (CG − wing-root-LE) ≈ (ΔX=-63.65 mm, ΔZ=-21.03 mm); the XFLR5 CG actually used = (+0.0637, 0, -0.0210) m. The X-component sign flips (STL ΔX is negative, XFLR5 CoG_x is positive, same magnitude) — consistent with the stated X-axis reversal; the Z-component does **not** flip sign (both negative) — consistent with Z (up) being unreversed between the two frames.
- **What this does and does not resolve:** this documents a **qualitative** XFLR5 axis convention (X reversed relative to Gazebo/CAD FLU, origin at the main-wing-root leading edge, Y and Z unreversed) and shows it is *arithmetically self-consistent* for this one specific point (the CG). **It is not yet a general, validated coordinate transform for arbitrary points** (e.g. a future thrust-application point, hinge point, or a general XNP/XCP-to-Gazebo-frame conversion), and the master dataset's own repeated caution applies: *"XFLR5 control sign ile Gazebo joint sign aynı varsayılmamalı; unit-test ile eşlenmeli"* (`MD §72`) — XFLR5 sign conventions must not be assumed identical to Gazebo joint/force signs; they must be matched via unit test. **This document treats the XFLR5↔Gazebo frame relationship as `PARTIALLY_DOCUMENTED`, not `CONFIRMED`/resolved** — a full validated transform remains a prerequisite before XNP/XCP or any position-like XFLR5 value is used to compute a Gazebo-frame force-application point, static margin in Gazebo coordinates, or similar.
- **Separately unresolved:** whether the "X reversed" note describes only the XFLR5 *Plane-Editor geometry-input frame* (how CG/tail position numbers were entered) or *also* the axis convention XFLR5 uses internally when it reports stability derivatives (CYb, Clb, Cnb, CLa, Cma, etc. — a distinct "stability/results" axis system in XFLR5, not necessarily identical to the geometry-input frame). The master dataset does not state this explicitly. **This document does not assume the two are the same** — resolving whether the derivative signs above need any adjustment for Gazebo FLU remains an explicit `DATA_REQUIRED`/unit-test item (§10).
- The XNP (0.132 m)/XCP (0.064 m) values are still XFLR5-frame position values, not directly substitutable for the Gazebo/CAD CG (0.168309, 0, 0.100000) m — this has not changed.

---

## 10. Sign-Convention Status (alpha, beta, and every derivative)

| Item | Status | Detail |
|---|---|---|
| Gazebo body frame | CONFIRMED | FLU: +X forward, +Y left, +Z up (`CLAUDE.md`) |
| `alpha = atan2(w,u)`, `beta = asin(v/V)` as literally stated in the master dataset | **FLAGGED — requires `AOA_SIGN_TEST`/`SIDESLIP_SIGN_TEST` before use** | These are standard FRD-body-axis forms; FLU flips both the Z axis (alpha) and Y axis (beta) relative to FRD, so a naive implementation risks an inverted sign on both angles (§8.1). The master dataset's own text flags this ("Gazebo sign convention unit test ile doğrulanmalı") — this document amplifies, not resolves, that flag. |
| Elevator `delta_e` sign | CONFIRMED (XFLR5's own convention) + Gazebo joint-rotation mapping RESOLVED | `+ = trailing edge down`, `- = trailing edge up` (`MD §22`). Gazebo joint-rotation sign mapping: `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST` (task `CONTROL_SURFACE_SIGN_MAPPING`, §19.13); `elevator_sign` corrected `+1.0→-1.0`. |
| Rudder `delta_r` sign | RESOLVED (Gazebo joint-to-deflection physical direction) | The Type7 sweep table (§7.2) gives numeric `CY/Cl/Cn` vs. `delta_r` in XFLR5's own convention. The physical Gazebo joint direction (`rudder_sign=+1.0`, confirmed unchanged) was established by task `CONTROL_SURFACE_SIGN_MAPPING`, §19.13. |
| Aileron `delta_a` sign | RESOLVED (Gazebo joint-to-deflection physical direction) | The physical Gazebo joint direction (`aileron_sign=+1.0`, confirmed unchanged) was established by task `CONTROL_SURFACE_SIGN_MAPPING`, §19.13. **Separately, and still open:** the XFLR5-internal WF1..WF6 flap-group adverse/proverse physical labeling (`MD §35`/`§36`) remains not finalized — an unrelated question about the master dataset's own convention, not the Gazebo joint sign. |
| XFLR5 lateral-directional derivative sign convention (CYb/Clb/Cnb/etc.) vs. Gazebo FLU | DATA_REQUIRED — partially informed but not resolved | See §9, last bullet — the "X reversed" geometry-input-frame note is not confirmed to also describe the stability-derivative results frame. |
| XNP/XCP frame origin | PARTIALLY_DOCUMENTED | See §9 — qualitative convention shown, general transform not validated. |

**No sign flip, axis remap, or "XFLR5 and FLU agree" shortcut is applied anywhere in this document.** Establishing and unit-testing these mappings remains a prerequisite for using any of §6/§7's derivatives inside a Gazebo force/moment model, per this agent's role rules and the project's workflow (`gazebo-testing` runs `AOA_SIGN_TEST`/`SIDESLIP_SIGN_TEST`/`TRIM_TEST` after implementation; `validation` reviews independently).

---

## 11. Stall / Post-Stall — Explicit Non-Action (per project rule)

The master dataset (`MD §64`) explicitly states the stall model is **not final**:

> "XFLR5 attached-flow güvenilirliği≈9–9.5 deg civarında düşüyor. Manual CLmax≈1.42. Gazebo'da lineer CL sonsuza kadar devam ettirilmemeli. V1: lookup / smooth saturation. V2: stall onset, post-stall CL drop, CD rise, Cm variation, aerodynamic-center migration."

**Consistent with this agent's rule ("Stall/post-stall modeling requires explicit review before implementation — do not add stall or post-stall aerodynamic behavior on your own initiative"), no stall or post-stall model, lookup table, or saturation function is added, chosen, or implemented in this document or anywhere else in this pass.** This is surfaced to the user and to `validation` as an open architecture decision, not resolved unilaterally. The reliability limits that bound *when* this decision becomes relevant are, however, documented and preserved:

- Wing attached-flow linear model: reliable to **≤ ≈9°** alpha; **≥9.5°** not to be used for validation (`MD §15`).
- Control-surface derivatives (elevator/rudder/aileron): most reliable within **≈±10°** deflection, even though the mechanical/manual joint-limit starting point is **±30° or more** (`MD §65`, `MD §72`). No control derivative in §7 is extrapolated past ±10° anywhere in this document.
- Manual `CLmax = 1.42` is a manufacturer performance-calculation input, not a flight measurement (`MD §3`) — used only for the stall-speed calculation in §5.2, not as a validated post-stall boundary condition.

### 11.1 Observed emergent behavior at 12.5 m/s — CL/Cm coupling near `alpha_transition` (informational, no model change)

`FLIGHT_ENVELOPE_VALIDATION` (2026-08-27, `gazebo-testing`) found that the V1 trim solver reports `AERODYNAMIC_NO_TRIM_MOMENT` at V=12.5 m/s (alpha≈11.4°, 2.15° past `alpha_transition=9.25°`): no elevator deflection zeros the pitching moment without the corresponding trim alpha demanding a `CL` above the `CLmax=1.42` asymptote that `SaturatedCL()` is built to never exceed. Root cause, traced by `aerodynamics` on interpretation only (no coefficient, table, or limiter code changed to produce or fix this): the elevator wide-deflection lookup's `dCm` and `dCL` columns (`control_surface_lookup.elevator`, `AeroModel.hh` `ComputeAero()`) are coupled by construction — the same `delta_e` drives both. The trailing-edge-up deflection needed to supply enough nose-down restoring moment at this alpha also subtracts lift (`dCL` becomes more negative); near the `CLmax` asymptote no further alpha increase can make up that lift deficit, so no self-consistent `(alpha, delta_e)` pair exists. This is assessed as an **expected, self-consistent consequence of the current V1 no-stall architecture**, not a defect — it appears exactly at/past `alpha_transition`, the same boundary this section already documents as outside the model's validated attached-flow region with no stall/post-stall physics implemented. It is arguably a useful property (the model reports infeasibility rather than silently accepting an inconsistent trim), but it is also a real gap worth carrying into any future stall/post-stall modeling pass: a true post-stall model would need a reduced effective `CLmax` and/or reduced elevator effectiveness in this same region, and the current V1 model has no graceful degradation there — it simply has no solution. No fix proposed or made here; logged for `validation`/a future dedicated review only.

---

## 12. Directly Gazebo-Usable Parameters (updated)

| Parameter | Status | Caveat still open |
|---|---|---|
| S=0.4514 m², b=2.093 m, c_ref≈0.224 m | CONFIRMED — now confirmed as XFLR5's own internal reference values (§2.2, §6.2), not merely assumed to match | None remaining on the *reference-quantity* question; sign/frame caveats (§9/§10) still apply to any position-like value used alongside them |
| CLa, Cma, CLq, Cmq, CXa, CXq (§6.2) | CONFIRMED (single reference operating point; §9/§10 sign caveats apply) | Valid at/near the neutral trim condition; not confirmed alpha-independent across the full envelope beyond the elevator sweep's own trim-point range (§7.1) |
| CYb/Clb/Cnb/CYp/Clp/Cnp/CYr/Clr/Cnr (§6.2) | CONFIRMED (unchanged from original reference point) | Sign convention vs. Gazebo FLU unresolved (§10) |
| CYda/Clda/Cnda, CYdr/Cldr/Cndr, Cmde (§7) | CONFIRMED within ±10° (§11) | Physical Gazebo joint-to-deflection sign mapping (elevator/aileron/rudder) now RESOLVED (§10, §19.13, task `CONTROL_SURFACE_SIGN_MAPPING`); do not extrapolate past ±10° |
| CD0≈0.0351, k≈0.0528 (§6.5) | V1_CALIBRATED | Calibration model, not flight-measured; distinct from any inviscid Type7 CD |
| Dynamic-mode / trim benchmarks (§6.8) | VALIDATION_TARGET | For checking a future implementation, not for tuning it |

---

## 13. Parameters Requiring Conversion / Confirmation Before Gazebo Use

| Parameter | What's needed | Status |
|---|---|---|
| XNP/XCP → Gazebo/CAD frame | Full validated coordinate transform (§9) | PARTIALLY_DOCUMENTED — qualitative convention only |
| Lateral-directional derivative sign convention → Gazebo FLU | Unit-test confirmation (`SIDESLIP_SIGN_TEST` et al.) | DATA_REQUIRED |
| `alpha`/`beta` formula sign in FLU | Unit-test confirmation (`AOA_SIGN_TEST`) | FLAGGED, not resolved (§8.1/§10) |
| ~~Rudder/aileron/elevator deflection physical-direction sign~~ | ~~Project-owner or controls-integration confirmation~~ | **RESOLVED** (task `CONTROL_SURFACE_SIGN_MAPPING`, §19.13; also §10) |
| CL0/Cm0 constants for the linear build-up | Explicit derivation (candidate method shown, §8.3) or a lookup/saturation decision (§11) | DATA_REQUIRED / pending review |
| CXa/CXq vs. CL/CD axis reconciliation | Explicit modeling decision | DATA_REQUIRED (§6.2 note) |
| Stall/post-stall behavior | Explicit review (user + `validation`) before any implementation | Not implemented (§11) |
| Control-surface effectiveness beyond ±10° / vs. Reynolds/airspeed | New XFOIL/XFLR5 analysis | DATA_REQUIRED |
| Control-surface hinge-moment data | New analysis or test data | DATA_REQUIRED |
| Vertical-tail definitive airfoil identity | CAD/manufacturer confirmation | DATA_REQUIRED (only "approx. symmetric ~10% t/c" confirmed) |
| Per-surface downwash/sidewash interaction | New analysis | DATA_REQUIRED |
| Cmα̇ | New analysis | DATA_REQUIRED — explicitly still missing (§1.1) |

---

## 14. Cross-Check of Values

All 17 values from the original `CLAUDE.md`/`README.md` reference point were re-checked against `MD §30` (and, redundantly, `MD §37`/`§70`):

| Parameter | `CLAUDE.md`/`README.md` | `MD §30` | `MD §37`/`§70` | Result |
|---|---|---|---|---|
| Mass | 6.000 kg | 6.000 kg | 6.000 kg | Match |
| Trim velocity | 21.244 m/s | 21.244 m/s | (implicit, same point) | Match |
| Trim alpha | 0.364° | 0.364° | — | Match |
| CL | 0.47167 | 0.47167 | — | Match |
| XNP | 0.132 m | 0.132 m | — | Match |
| XCP | 0.064 m | 0.064 m | — | Match |
| CYb | -0.13216 | -0.13216 | -0.13216 | Match |
| Clb | -0.00717 | -0.00717 | -0.00717 | Match |
| Cnb | +0.03554 | +0.03554 | +0.03554 | Match |
| CYp | -0.04567 | -0.04567 | -0.04567 | Match |
| Clp | -0.54187 | -0.54187 | -0.54187 | Match |
| Cnp | -0.05878 | -0.05878 | -0.05878 | Match |
| CYr | +0.08776 | +0.08776 | +0.08776 | Match |
| Clr | +0.10586 | +0.10586 | +0.10586 | Match |
| Cnr | -0.02227 | -0.02227 | -0.02227 | Match |
| Wingspan | 2.093 m | — (`MD §3`: 2.093 m) | b=2.093 m | Match |
| Wing area | 0.4514 m² | — (`MD §3`: 0.4514 m²) | S=0.4514 m² | Match |

**Result: all values match exactly, no discrepancy.** The master dataset is confirmed to be an extension of the same underlying analysis, not a competing or conflicting source.

---

## 15. Conflicts

**No conflicts found.** The master dataset's full-aircraft neutral-fin derivative set (`MD §30`) is byte-for-byte consistent (to the precision given) with `CLAUDE.md`/`README.md`'s existing reference point (§14). No other document in the repository contains a competing aerodynamic numeric value.

---

## 16. Provenance Table

| Source | Provides | Used for |
|---|---|---|
| `CLAUDE.md` | Wingspan/wing area; single full-aircraft reference point; coordinate-system and tuning rules | §2, §6.1, §9 |
| `docs/source_of_truth/README.md` | Restates the same; prior gap list (now partially resolved) | Cross-check throughout |
| `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` | Full XFOIL/XFLR5 analysis chain, §1–§74 | §4–§11 primarily |
| `docs/source_of_truth/geometry/GEOMETRY.md` | CAD/mesh cross-checks; control-surface hinge candidates | Cited in §5, §7 (not duplicated) |
| `docs/source_of_truth/mass_properties/MASS_PROPERTIES.md` | Gazebo/CAD CG, XFLR5 CG, frame-separation rule | §9 |

Source hierarchy (unchanged, restated from `GEOMETRY.md`/`MASS_PROPERTIES.md`, matching the master dataset's own "VERİ ÖNCELİĞİ" list): (1) manufacturer manual, (2) measured aircraft data, (3) real motor/prop/component manufacturer data, (4) STL geometry, (5) XFOIL/XFLR5 analysis, (6) calculation/V1 estimate. No case in this document required resolving a conflict using this hierarchy (§15).

---

## 17. Missing Data — Full `DATA_REQUIRED` List (post-master-dataset-sync)

Substantially shorter than the prior 28-item list, since most of it is now resolved. Remaining:

1. Full point-by-point (non-representative) Cl/Cd/Cm-vs-alpha XFOIL polar curves, any airfoil (§4.1, §4.3).
2. Definitive vertical-tail airfoil identity beyond "approximately symmetric, ~10% t/c, not NACA0010" (§4.2).
3. Per-surface downwash/sidewash interaction data (§5.6).
4. Full dense (non-representative) CL/CD/Cm-vs-alpha curves for any single surface in isolation (§5.6).
5. `Cmα̇` (rate of pitching-moment change with angle-of-attack rate) — the one longitudinal derivative still entirely absent (§1.1, §13).
6. ~~`CL0`/`Cm0` constant-offset anchors for the linear build-up equations~~ — **RESOLVED** (task `AERODYNAMICS_V1_IMPLEMENTATION`, §19.2/§19.3): `CL0=0.437035`, `Cm0=0.010550`, both `DERIVED` per the candidate method already sketched here, full arithmetic shown.
7. Reconciliation of the `CXa`/`CXq` body-axis force derivatives with the `CL`/`CD`-based wind-axis force model (§6.2).
8. A full, validated Gazebo-frame coordinate transform for XNP/XCP and any other XFLR5-frame position value (§9) — qualitative convention only exists so far.
9. Confirmation of whether the "XFLR5 X-axis reversed" note applies to the stability-derivative results frame, or only the geometry-input frame (§9).
10. `AOA_SIGN_TEST`/`SIDESLIP_SIGN_TEST` resolution of the FRD-vs-FLU sign risk in the `alpha=atan2(w,u)`/`beta=asin(v/V)` formulas (§8.1, §10).
11. ~~Physical (nose-left/right, surface-up/down) deflection-direction mapping for rudder and aileron `delta` sign~~ — **RESOLVED** (task `CONTROL_SURFACE_SIGN_MAPPING`, §19.13): `controls-integration`'s geometric determination plus `gazebo-testing`'s live kinematic/aero-moment confirmation established the physical joint-to-deflection mapping for all three surfaces (elevator, aileron, rudder) — `elevator_sign` corrected `+1.0→-1.0`, `aileron_sign`/`rudder_sign` confirmed `+1.0`. See `aero_v1_config.yaml` `control_mapping` for the resulting config.
12. Control-surface hinge-moment data, any surface (§7.4).
13. Control-surface effectiveness variation vs. Reynolds number/airspeed, and beyond the ±10° reliable band (§7.4, §11).
14. Stall/post-stall model choice (lookup/saturation vs. any other approach) — explicitly deferred pending review, not a data gap to fill unilaterally (§11).
15. XFLR5 project's own internal reference-geometry confirmation beyond S/b/c_ref (e.g. its exact panel/mesh definition) — not needed for the current derivative set but not in-repo either.

None of these are estimated, guessed, interpolated, or filled with a placeholder value anywhere in this document.

---

## 18. Interpolation Methods

**No interpolation has been implemented in any code — this section documents the intended method and valid range for a future implementation, consistent with the multi-point tables now available (§7.1–§7.3, §5.1).**

| Dataset | Recommended method | Valid range | Do not extrapolate past |
|---|---|---|---|
| Elevator Type7 trim sweep (`delta_e` → alpha/V/CLa/Cma/Cmq/NP/CMde, §7.1) | Linear interpolation between adjacent table rows (master dataset itself characterizes the response as smoothly varying, not spline-fit) | delta_e ∈ [-10°, +8°] (usable rows); +10° row has no valid trim (negative-lift skip) | delta_e beyond ±10° — "most reliable" band per `MD §65` |
| Rudder Type7 sweep (`delta_r` → CY/Cl/Cn, §7.2) | Linear interpolation; master dataset itself describes the response as "yaklaşık lineer ve simetrik" (approximately linear and symmetric), so a single-slope model is also a defensible simplification if a full table lookup is not implemented | delta_r ∈ [-10°, +10°] | delta_r beyond ±10° |
| Aileron Type7 sweep (`delta_a` → CY/Cl/Cn/CDi, §7.3) | Linear interpolation between adjacent rows | delta_a ∈ [-10°, +10°] | delta_a beyond ±10° |
| Main-wing 3D CL vs. alpha (§5.1) | Linear interpolation between the given points; **table is sparse/uneven** (points at -8, -6, -5.5, 0, 5, 9° only) — no spline is justified without more points, and none is applied here | alpha ∈ [-8°, 9°], with alpha > ≈9° explicitly outside the reliable attached-flow range (`MD §15`) even though it is the last tabulated point | alpha > ≈9° (reliability limit, not the table's own edge) |
| Full-aircraft drag polar `CD=CD0+k·CL²` (§6.5) | Closed-form quadratic, not a table — no interpolation needed | Whatever CL range the V1 calibration was fit against (not explicitly bounded in the master dataset) | Any CL requiring extrapolation of the manual's own 6 kg performance-graph range is not separately validated |

No Reynolds-number interpolation is implemented or recommended yet — the 2D airfoil CLmax/CDmin/alpha0 data (§4.1) is reported only at discrete Re values with no stated interpolation method in the master dataset itself, and none is invented here.

---

## 19. `AERODYNAMICS_V1_IMPLEMENTATION` (2026-08-22) — First working force/moment model

**Scope of this pass:** implements the V1 aerodynamic force/moment model as a Gazebo Sim Harmonic C++ System plugin, attached to `model/model.sdf`. This is a code-and-config pass (unlike §1–§18, which were docs-only) — `model/model.sdf` gained one additive `<plugin>` block (no structural element modified), `plugins/aerodynamics/` was created, and `docs/source_of_truth/aerodynamics/aero_v1_config.yaml` was created as the structured coefficient dataset. Files: `plugins/aerodynamics/{AeroModel.hh, AerodynamicsSystem.hh, AerodynamicsSystem.cc, CMakeLists.txt, README.md, test/aero_model_selftest.cc}`, `docs/source_of_truth/aerodynamics/aero_v1_config.yaml`.

### 19.1 Architecture

C++ Gazebo System plugin (`ISystemConfigure` + `ISystemPreUpdate`), chosen over any Classic-era or Python-only approach because this repository's environment has the full `gz-sim8`/`gz-plugin2`/`gz-transport13`/`gz-msgs10`/`gz-math7` development headers installed (verified directly: `pkg-config --modversion gz-sim8` → 8.14.0, etc.) and a System plugin is the only Harmonic-native way to apply a per-timestep force/torque inside the physics loop with full ECM access (link velocity, joint position, quaternion pose) — a Python `TestFixture`-based approach (as used by the existing structural test scripts) is a *test* harness, not a way to permanently attach continuous physics to the model.

The plugin is split into a pure-math header (`AeroModel.hh`, no Gazebo dependency beyond `gz::math::Vector3d`) and a thin ECM-glue class (`AerodynamicsSystem`). This split let the same formulas be exercised by a standalone, Gazebo-independent self-test executable (`aero_model_selftest`) without needing a live Gazebo instance — see §19.9.

All coefficients live in `docs/source_of_truth/aerodynamics/aero_v1_config.yaml` (loaded at `Configure()` time via `yaml-cpp`), not hardcoded in the C++ source, per `CLAUDE.md`'s source-of-truth policy. Every numeric field in that file carries a provenance comment tracing to `CLAUDE.md`/this document/a documented derivation.

### 19.2 `CL0` derivation (`DERIVED`)

Per §8.3's own candidate method, now performed:

```
CL0 = CL_trim - CLa * alpha_trim(rad)
```

Using the neutral-trim (`delta_e=0`) row of the elevator sweep (§7.1): `CL_trim = 0.471685`, `alpha_trim = 0.36455°`, and the full-precision `CLa = 5.44594 /rad` (§6.2, per the task brief's "use exactly" derivative set):

```
alpha_trim(rad) = 0.36455 * pi/180 = 0.0063626 rad
CL0 = 0.471685 - 5.44594 * 0.0063626 = 0.437035
```

Verified by direct computation (Python, reproduced in the plugin's self-test `MakeFalconV2Config()`). Sanity check: `CL0 + CLa*alpha_trim = 0.437035 + 0.034650 = 0.471685` reproduces `CL_trim` exactly, by construction. Independent plausibility check: the implied zero-lift angle (`-CL0/CLa = -4.60°`) falls in the same order of magnitude as the wing airfoils' own 2D zero-lift alpha (`≈-4.0° to -4.6°`, §4.1) and the 3D wing's zero-lift alpha (`≈-5.8°`, §5.1) — not identical (a full-aircraft CL0 differs from a wing-alone value due to the tail's contribution), but not implausible either. **Status: `DERIVED`.**

### 19.3 `Cm0` derivation (`DERIVED`)

At the same neutral-trim anchor, `Cm_trim ≈ 0`, `delta_e = 0` (by definition of the neutral row), `q_hat = 0` (steady 1g level flight, no pitch rate). From `Cm = Cm0 + Cma*alpha + Cmq*q_hat + Cmde*delta_e`:

```
0 = Cm0 + Cma * alpha_trim(rad)
Cm0 = -Cma * alpha_trim(rad) = -(-1.65805) * 0.0063626 = 0.010550
```

Using the full-precision `Cma = -1.65805 /rad` (§6.2). **Status: `DERIVED`.** Sign sanity check: a positive `Cm0` is the textbook-expected sign for a longitudinally-stable aircraft trimmed at a small positive alpha with `Cma<0` (see §19.7 for a related, more consequential sign finding about how this `Cm` is subsequently converted into a Gazebo torque).

### 19.4 `CLde` (elevator lift derivative) — omitted, with evidence

The task brief authorized either deriving `CLde` from the Type-7 elevator sweep table (§7.1) or omitting it with documented justification. A finite-difference extraction was attempted: for each adjacent pair of sweep rows, `CLde_est = (ΔCL - CLa_avg·Δalpha) / Δdelta_e`. Result (full arithmetic reproducible from the table in §7.1):

| delta_e pair (deg) | CLde_est (/rad) |
|---|---|
| -10 → -8 | 0.235 |
| -8 → -6 | 0.225 |
| -6 → -4 | 0.207 |
| -4 → -2 | 0.158 |
| -2 → 0 | 0.080 |
| 0 → 2 | 0.508 |
| 2 → 4 | 0.374 |
| 4 → 6 | 0.313 |
| 6 → 8 | 0.293 |

This ranges over **0.080–0.508 /rad, a >6x spread, with a discontinuity straddling `delta_e=0`** — not a stable, extractable constant. Root cause: each row of the Type-7 sweep is a distinct **trim search result** (a different `V` and `alpha` satisfying a separate 1g moment/lift balance for that `delta_e`, §7.1's own framing), not a fixed-speed, fixed-alpha direct control sweep — so a naive finite difference conflates the elevator's direct lift contribution with the entire trim-point shift (including `CLa`'s own row-to-row variation, 5.30–5.45/rad). Fabricating a `CLde` from this unstable extraction would violate the project's no-fabrication rule. **`CLde` is therefore omitted from the V1 `CL` build-up** (`CL = CL0 + CLa*alpha + CLq*q_hat` only, no elevator term), consistent with the task brief's "(supported control contribution only, per above)" qualifier and its explicit "omit it with documented justification" option. `docs/source_of_truth/aerodynamics/aero_v1_config.yaml`'s `longitudinal.CLde` note reproduces this evidence.

### 19.5 High-alpha smooth-saturation limiter (`V1_SMOOTH_SATURATION`)

Formula (C1-continuous — matches value **and** slope at the transition, so there is no kink and no unexplained free parameter beyond the already-given `CLa`/`CL0` and the two data-traced constants `CLmax`, `alpha_transition`):

```
alpha_transition = 9.25°  [DERIVED: midpoint of the XFLR5-stated 9-9.5° attached-flow reliability band, §5.1/§11]
CLmax = 1.42               [CONFIRMED as a manufacturer performance-calc input, MD sec 3; NOT flight-measured, §5.2]

for |alpha| <= alpha_transition:
    CL = CL0 + CLa*alpha                                  (exact linear model, unchanged)

for alpha > alpha_transition (positive side, traces to real data):
    headroom_pos = CLmax - (CL0 + CLa*alpha_transition) = 0.103757
    k_pos = CLa / headroom_pos = 52.4876 /rad
    CL = CLmax - headroom_pos * exp(-k_pos*(alpha - alpha_transition))

for alpha < -alpha_transition (negative side, ASSUMPTION - see below):
    A_neg = (CL0 - CLa*alpha_transition) + CLmax = 0.977826
    k_neg = CLa / A_neg = 5.5694 /rad
    CL = -CLmax + A_neg * exp(k_neg*(alpha + alpha_transition))
```

Numeric table (full range, computed by the self-test, `HIGH_ALPHA_LIMITER_TEST`):

| alpha (deg) | CL (linear, unclamped) | CL (saturated) |
|---|---|---|
| 0 | 0.437 | 0.437 |
| 9.25 | 1.316 | 1.316 (exact match, boundary) |
| 10 | 1.388 | 1.368 |
| 20 | 2.338 | 1.420 |
| 90 | 8.992 | 1.420 |
| -9.25 | -0.442 | -0.442 (exact match, boundary) |
| -20 | -1.464 | -1.076 |
| -90 | -8.117 | -1.420 |

**Positive side: traces to real data** (manufacturer `CLmax=1.42`, XFLR5-stated 9–9.5° reliability boundary). **Negative side: `ASSUMPTION`.** No full-aircraft negative-alpha `CLmin`/stall data exists anywhere in the source of truth (the only negative-alpha data present is the wing-*only* 3D VLM table down to -8°, §5.1, with no stated stall onset). Per the task brief's explicit instruction not to invent symmetry without source support, the negative-side bound uses a symmetric magnitude (`-CLmax`, `-alpha_transition`) **purely as a numerical safety measure** to prevent unbounded negative lift/drag growth (the literal V1 requirement — "just prevent unbounded linear growth"), not as a claim of validated negative-alpha stall physics. True negative-alpha stall behavior remains `DATA_REQUIRED`. This is tagged `ASSUMPTION` explicitly in `aero_v1_config.yaml` and in `AeroModel.hh`.

`CD = CD0 + k*CL²` (§6.5, `V1_CALIBRATED`, unchanged) is fed the **saturated** `CL`, not the raw linear value, so induced drag also stays bounded at extreme alpha — the same "prevent unbounded growth" requirement applied consistently.

### 19.6 Angle of attack / sideslip — FLU re-derivation (not a blind FRD copy)

Both formulas were re-derived from first principles (not copied from the FRD textbook forms) by explicit rotation-matrix construction, verified numerically (Python) and again via the compiled self-test. Full derivation and axis-rotation reference table live in `AeroModel.hh`'s header comments (reproduced in summary here):

**Confirmed axis-rotation physical meanings in FLU** (found by asking "where does a body unit vector go, in world coordinates, under a positive rotation about this body axis", using the standard right-handed rotation matrices — identical algebra to FRD, only the physical axis labels differ):

| Axis | Positive-rotation physical meaning in FLU | vs. FRD |
|---|---|---|
| +X (roll) | LEFT wingtip moves UP | Same physical roll sense as FRD's "right wing down" (Y and Z both flip meaning between frames — an even number of flips, so roll handedness is unchanged) |
| +Y (pitch) | NOSE DOWN | **Opposite** of the traditional aerospace "q>0/Cm>0 = nose up" shorthand (only Z flips meaning for this axis pair — an odd number of flips) |
| +Z (yaw) | NOSE LEFT | **Opposite** of FRD's "r>0 = nose right" (only Y flips meaning — an odd number of flips) |

**Angle of attack:** `alpha = atan2(-w, u)`. Derived by placing a nose-up-by-theta rotation (physically, θ IS the angle of attack by definition of this scenario) at the correct sign relative to the "+Y rotation → nose down" finding above (i.e. nose-up = rotation of `-theta` about +Y), then computing the resulting body-frame relative wind: `u=V·cos(theta)`, `w=-V·sin(theta)`. Alpha's physical meaning ("nose up relative to the wind = positive") has **no free convention** — unlike beta below, there is only one physically sensible definition, and it was verified by an explicit 5°/90°-rotation numeric check, not verbal analogy. **`AOA_SIGN_TEST` (gazebo-testing, live Gazebo) is still required** before this is treated as final against XFLR5's own internal convention.

**Sideslip:** `beta = atan2(v, hypot(u,w))` — chosen over the candidate `atan2(-v, hypot(u,w))` (the literal FRD-textbook-preserving form) after checking both candidates against the **given, unmodified** `Cnb=+0.03554` and `CYb=-0.13216` and their stated physical meanings (§6.2: "Cnβ>0 → directionally statically stable"). For a nose-slipped-left/wind-from-right disturbance: the `atan2(-v,...)` candidate produces a **destabilizing** yaw moment (amplifies the disturbance) under the confirmed "+Z rotation → nose left" axis meaning; the adopted `atan2(+v,...)` candidate produces a **restoring** yaw moment, matching the stated `Cnb` interpretation, and `CYb` under the same candidate produces a sideforce pushing the aircraft away from a right-side crosswind (basic, reliable physics). **Physical meaning of the adopted convention: positive beta = relative wind from the aircraft's LEFT side** (the opposite of the standard FRD "wind from the right" convention — expected, since FLU's yaw-axis handedness is flipped relative to FRD, see table above). This is a **documented inference** from matching two independent, reliable physical checks against the fixed/unmodified `Cnb`/`CYb` signs — `Cnb`/`CYb` themselves are never touched, only the sideslip-angle sign convention (a genuine free choice, unlike alpha) is selected. A third possible check (`Clb`, "dihedral effect") was attempted but **not used as evidence either way** — this document does not assert a verified physical direction for the dihedral-sideslip roll coupling from first principles (subtle fluid-dynamics mechanism, not reliably re-derivable by rotation-matrix bookkeeping alone). **`SIDESLIP_SIGN_TEST` and `Cnb_STATIC_STABILITY_SIGN_TEST` (gazebo-testing, live Gazebo) are still required** before this convention is treated as final.

### 19.7 Force-axis transformation (wind → body, not the naive `Fx=-D,Fz=L` shortcut)

Built from the *same* rotation `R(alpha,beta) = Ry(alpha)·Rz(beta)` implied by the alpha/beta formulas above (guaranteeing self-consistency — verified by reconstructing `R(alpha,beta)·(1,0,0) = Vrel/V` to ~1e-16 over thousands of random trials, and confirming `R(0,0)=identity`):

```
Fx = -D·cos(a)·cos(b) - Y·cos(a)·sin(b) + L·sin(a)
Fy = -D·sin(b) + Y·cos(b)
Fz =  D·sin(a)·cos(b) + Y·sin(a)·sin(b) + L·cos(a)
```

At `alpha=beta=0`: reduces exactly to `(Fx,Fy,Fz)=(-D,Y,L)`, matching the naive case as the correct zero-angle special case (not a coincidence — the derivation guarantees it). Tested (self-test `LIFT_SIGN_TEST`/`DRAG_SIGN_TEST`/rotation-sanity) at `alpha=0`, `alpha=+10°`, and `alpha=-10°`: `Fx` and `Fz` both vary continuously with alpha in both directions, confirming a proper rotation is applied, not a constant `-D`/`L` split.

**`Cma`/`My` pitch-axis sign finding — RESOLVED this pass, see §19.12 for the full root-cause/fix record.** Unlike the rate/damping terms (`Cmq·q_hat`, `Clp·p_hat`, `Cnr·r_hat`), which are mathematically self-referential (a negative coefficient times a rate always produces a moment opposing that *same* rate, about the *same* axis — true in any frame/handedness; verified algebraically and via the self-test, all three pass), the static terms `Cma·alpha`, `Cm0`, and `Cmde·delta_e` relate an independently-defined angle to a moment about a *different* reference. Given the confirmed "+Y rotation → NOSE DOWN" FLU finding (§19.6 table) — the opposite of the "positive Cm = nose up" shorthand that is only self-consistent with strict right-hand-rule rotation in FRD, not FLU — a literal, unmodified application of `My = qbar·S·c_ref·Cm` to the static group produced, for a nose-up disturbance, a moment that was **destabilizing** rather than the textbook-expected restoring (nose-down) behavior. This was first reported (not silently patched) pending live confirmation, then independently confirmed by `gazebo-testing`'s live measurement (`My=-2.83 N·m at alpha=+8°`, reinforcing not restoring) and root-caused by `validation`'s independent re-derivation. **The precisely-scoped fix (§19.12) is now applied**: only the static group (`Cm0 + Cma·alpha + Cmde·delta_e`) is negated when computing `My`; the rate group (`Cmq·q_hat`) is left unflipped (it was already correct — flipping it would have broken the passing `Cmq_DAMPING_SIGN_TEST`).

Moments (`Mx=qbar·S·b·Cl`, `My=qbar·S·c_ref·Cm`, `Mz=qbar·S·b·Cn`) are applied directly, with **no** wind→body rotation (§8.4, unchanged) — they are already body-axis roll/pitch/yaw moment coefficients per the given architecture.

### 19.8 Force-application point and double-counting analysis

**Decision:** apply the net aerodynamic force at `base_link`'s center of mass (which is exactly the documented Gazebo/CAD CG, `MASS_PROPERTIES.md` §3.1, since `base_link`'s `<inertial><pose>` is `(0.168309, 0, 0.100000)` m), and apply the net moment as a pure torque, both on `base_link` only.

**Implementation (verified against the actual installed `gz-sim8` `Link.hh` API, not assumed):**
- `Link::AddWorldForce(ecm, force_world)` — the 2-argument overload applies the given world-frame force **at the link's center of mass** (per the header's own documentation: "Add a force expressed in world coordinates and applied at the center of mass of the link"), contributing **zero** additional moment by construction (`r=0` relative to CoM).
- `Link::AddWorldWrench(ecm, gz::math::Vector3d::Zero, moment_world)` — force argument is exactly zero, so this reduces to a **pure torque** application, which is frame/offset-independent (no `r×F` term exists when `F=0`).

No other force is applied anywhere else in the plugin. This means: the fully-formed `Mx/My/Mz` (which already include every `Cl`/`Cm`/`Cn` derivative contribution — `Clb`, `Clp`, `Clr`, `Clda`, `Cldr`, etc.) are applied once, directly, as a moment; the lift/drag/sideforce are applied once, as a force at the CoM producing zero extra moment. There is no `r×F` double-counting. This required no coordination with `geometry-structure` beyond reading the already-published `base_link` `<inertial><pose>` value (no ambiguity found).

### 19.9 Self-test results (superseded by §19.12's post-fix re-run — kept here as the historical pre-fix record)

Two levels of self-testing were performed (both by `aerodynamics`, neither is a substitute for `gazebo-testing`'s formal, independently-executed and independently-reviewed test suite):

**(a) Standalone, Gazebo-independent executable** (`plugins/aerodynamics/build/aero_model_selftest`, compiled and run this pass, BEFORE the §19.12 fix):

```
[PASS] ZERO_AIRSPEED_AERO_TEST                  V=0 qbar=0 F=(0,0,0) M=(0,0,0), all finite
[PASS] AOA_SIGN_TEST (math-level)               nose-up 5deg -> alpha=+5.000deg
[PASS] AOA_SIGN_TEST negative-alpha case        nose-down -> alpha negative
[PASS] SIDESLIP_SIGN_TEST (formula self-consistency)
[PASS] LIFT_SIGN_TEST (alpha=beta=0)            Fz=+10 (lift up)
[PASS] DRAG_SIGN_TEST (alpha=beta=0)            Fx=-2 (drag aft)
[PASS] Wind-to-body rotation sanity (+/-alpha)  Fx/Fz vary continuously with alpha
[FAIL] Cma_RESTORING_SIGN_TEST                  DESTABILIZING under literal formula - see sec 19.7, NOT patched
[PASS] Cmq_DAMPING_SIGN_TEST
[PASS] Clp_DAMPING_SIGN_TEST
[PASS] Cnr_DAMPING_SIGN_TEST
[PASS] Cnb_STATIC_STABILITY_SIGN_TEST
[INFO] AILERON_ROLL_SIGN (algebraic only)       needs live AILERON_TEST for physical direction
[INFO] RUDDER_YAW_SIGN (algebraic only)         needs live RUDDER_TEST
[INFO] ELEVATOR_PITCH_SIGN (algebraic only)     needs live ELEVATOR_TEST
[PASS] RATE_NORMALIZATION_TEST (p_hat=p*b/2V)
[PASS] RATE_NORMALIZATION_TEST (V=0, rates nonzero -> no NaN/Inf)
[PASS] DRAG_POLAR_TEST + TRIM_BENCHMARK          CL=0.471685 (expect ~0.4717), CD=CD0+k*CL^2 exact
[PASS] HIGH_ALPHA_LIMITER_TEST                   bounded, monotonic, exact-linear within +/-9.25deg

SUMMARY: 15 PASS, 1 FAIL (honest, documented, sec 19.7), 3 INFO (need live Gazebo)
```

**Note: `CLq` was loaded from config but not yet referenced in `SaturatedCL()`/`ComputeAero()` at the time of this pre-fix run** — a second, independent finding (`validation`, MAJOR) not caught by this self-test suite at the time, since no test in the (a) run above specifically exercised `q≠0` against `CL`. Fixed alongside the `Cma`/`My` fix in §19.12; a dedicated regression check was added to the self-test to prevent recurrence.

**(b) Live Gazebo smoke test** (this pass, `gz sim -s -r`, `GZ_SIM_SYSTEM_PLUGIN_PATH` pointed at the built `.so`, against the existing `tests/gazebo/worlds/falcon_v2_freefall_world.sdf`): the plugin loads (`Loaded system [falcon_v2_aero::AerodynamicsSystem]`), configures successfully (logs `S=0.4514 b=2.093 c_ref=0.224`, correct diagnostics/wind topics), and ran 3000 physics steps (1 ms each) with no error/crash/NaN. Diagnostics topic (`/model/falcon_v2/aerodynamics/diagnostics`) was echoed live and showed physically sane values throughout a multi-second free-fall: `alpha` converges toward ≈90° (correct — a body falling nose-forward but moving nearly straight down has near-maximal angle of attack in the vertical plane, not a bug), `CL` saturates at exactly `1.42` (the limiter engaging correctly at high alpha), `CD` matches `CD0+k·CL²` exactly, and `Cm`/`Mx`/`My`/`Mz` remain finite and smoothly varying throughout. **This is a smoke test only** — it confirms the plugin loads, reads real ECM state, and produces finite, plausible output over a real physics run; it is explicitly **not** a substitute for `gazebo-testing`'s formal `ZERO_AIRSPEED_AERO_TEST`/`AOA_SIGN_TEST`/etc. with proper pass/fail criteria, result files, and independent `validation` review.

**Tests requiring `gazebo-testing` to execute in a live Gazebo instance** (full list, all 16 named in the task): `AOA_SIGN_TEST`, `SIDESLIP_SIGN_TEST`, `Cma_RESTORING_SIGN_TEST` (**highest priority — known, documented, honestly-reported discrepancy, §19.7**), `Cnb_STATIC_STABILITY_SIGN_TEST` (re-confirm against real quaternion/ECM plumbing, not just the pure-math core), `AILERON_ROLL_SIGN_TEST`, `RUDDER_YAW_SIGN_TEST`, `ELEVATOR_PITCH_SIGN_TEST` (all three gated on `controls-integration`'s not-yet-built command interface — see §19.10), and `TRIM_TEST`/`STRAIGHT_LEVEL_FLIGHT_TEST`-class dynamic tests (require a full free-flight scenario, out of scope for this pass per the task brief). `ZERO_AIRSPEED_AERO_TEST`, `LIFT_SIGN_TEST`, `DRAG_SIGN_TEST`, `Cmq_DAMPING_SIGN_TEST`, `Clp_DAMPING_SIGN_TEST`, `Cnr_DAMPING_SIGN_TEST`, `RATE_NORMALIZATION_TEST`, `DRAG_POLAR_TEST`, `HIGH_ALPHA_LIMITER_TEST` were exercised at the pure-math level here (self-test, above) but should still be re-run by `gazebo-testing` against the live plugin (ECM plumbing, quaternion rotation, joint reads) for formal sign-off.

### 19.10 Control-joint-to-deflection-sign mapping

`AerodynamicsSystem` reads the 5 real joint positions (`left_aileron_joint`, `right_aileron_joint`, `left_elevator_joint`, `right_elevator_joint`, `rudder_joint`) from the ECM every step — there is no parallel/disconnected control-state variable. Mapping to `delta_a`/`delta_e`/`delta_r` (documented in full in `aero_v1_config.yaml`'s `control_mapping` block and `AerodynamicsSystem.cc`):

```
delta_a = 0.5 * aileron_sign  * (theta_right_aileron - theta_left_aileron)   [differential]
delta_e = 0.5 * elevator_sign * (theta_left_elevator  + theta_right_elevator) [symmetric]
delta_r =       rudder_sign   *  theta_rudder
```

**Status update (2026-08-22, task `CONTROL_SURFACE_SIGN_MAPPING`, §19.13): all three sign parameters are now `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST`, no longer `ASSUMPTION`.** `aileron_sign=+1.0` and `rudder_sign=+1.0` were confirmed correct unchanged; `elevator_sign` was corrected from `+1.0` to `-1.0` (was backward — see §19.13 for the full evidence and provenance). The aileron **differential** (not symmetric-sum) combination and the elevator **symmetric-sum** (not differential) combination — originally derived from `CONTROLS.md` §9.3's finding that the left/right aileron and elevator joint axes share the same +Y-dominant sign sense (not true sign-flipped mirrors) — are likewise now `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST`, directly confirmed by live kinematic measurement (§19.13). All three sign parameters remain exposed as YAML config values (not hardcoded), so any future correction can be applied by flipping a config value, no code change required. Control deflections are additionally clamped to `±10°` (`control_deflection_clamp_deg`) before being used in the linear coefficient formulas — `V1_CONSERVATIVE_CLAMP`, tracing to `CONTROLS.md` §3 / this document §7.4/§11/§18's `±10°` aero-validated-range statement (the mechanical joint limit is `±30°`, per `geometry-structure`'s SDF; the aerodynamic model is not extrapolated past its validated range even though the joint can mechanically travel further).

### 19.11 `DATA_REQUIRED` items unaffected by this pass

This pass did not resolve, and does not claim to resolve: `Cmα̇`, the `CXa`/`CXq` axis-reconciliation question (still correctly unused — the given V1 equation set never references them), the full XNP/XCP Gazebo-frame coordinate transform, control-surface effectiveness beyond `±10°`, hinge-moment data, vertical-tail airfoil identity, or downwash/sidewash interaction. See §17 for the full list, unchanged except for item 6 (`CL0`/`Cm0`, now resolved, §19.2/§19.3) and item 11 (physical nose-left/right, TE-up/down meaning of a positive rudder/aileron/elevator deflection — now resolved by live measurement, task `CONTROL_SURFACE_SIGN_MAPPING`, §19.13).

### 19.12 Post-review fix pass (2026-08-22) — `Cma`/`My` axis-handedness bug + missing `CLq` term

Following §19.9's self-reported `Cma_RESTORING_SIGN_TEST` failure, `gazebo-testing` independently confirmed the same behavior via a live measurement (`My=-2.83 N·m at alpha=+8°`, reinforcing not restoring), and `validation` performed a full root-cause review, re-deriving the axis-handedness argument independently (rotation-matrix construction on both the `+X` and `+Z` body axes, in addition to the `+Y` analysis already in §19.7) and identifying one additional MAJOR finding (`CLq` unused). Both are fixed in this sub-pass, in `plugins/aerodynamics/AeroModel.hh`'s `ComputeAero()` only — no coefficient value in `aero_v1_config.yaml` was changed.

**Fix 1 — scoped `Cm`-to-`My` sign correction (CRITICAL).** `validation`'s independent re-derivation confirmed and refined the original diagnosis: only the **static/angle-derived** terms need the axis-handedness correction; the **rate** term does not.

- **Static group** (`Cm0`, `Cma·alpha`, `Cmde·delta_e`): each relates an independently-defined angle (alpha, or a control deflection) to a moment — not self-referential, so it inherits the FLU `+Y`-axis-handedness mismatch identified in §19.7. **Negated** when computing `My`.
- **Rate group** (`Cmq·q_hat`): self-referential — a negative coefficient times a rate always opposes that *same* rate about the *same* axis, algebraically, regardless of frame handedness. `validation` additionally traced through *why* this term was already correct despite the same nominal `+Y` mirroring: the mirroring affects **both** `q` (read raw from the FLU ECM, itself subject to the same axis relabeling) **and** `My`'s resulting sign, identically — a double-cancellation, not a coincidence, and exactly consistent with `Cmq_DAMPING_SIGN_TEST` having already passed pre-fix. **Left unflipped.**
- `Cmde` is included in the negated static group deliberately — it is a geometric control-deflection angle, not a rate, so it has the same axis-handedness exposure as `Cma·alpha`, **independently of** the separate, still-open `ELEVATOR_SIGN_TEST` question of whether a positive Gazebo elevator joint command physically means trailing-edge-down. **Consequence, flagged for `gazebo-testing`/`controls-integration`: `ELEVATOR_PITCH_SIGN_TEST`'s measured `My` sign will be flipped relative to any pre-fix measurement — expected, not a regression.**
- `Cm0` and `Cma` are always flipped **together**, never independently: `Cm0` was specifically derived (§19.3) so that `Cm0 + Cma·alpha_trim = 0` at the neutral trim point; flipping only one would break that zero-moment trim condition.
- `out.Cm` (the diagnostics-facing value, e.g. published on the `/model/falcon_v2/aerodynamics/diagnostics` topic) is left in **XFLR5's own, unflipped convention** — only the internal `My` computation applies the correction. This keeps `Cm` comparable against the source-of-truth sweep tables (§7.1) for diagnostic purposes.

Implementation (`AeroModel.hh::ComputeAero()`):
```cpp
const double cmStatic = cfg.Cm0 + cfg.Cma * out.alpha + cfg.Cmde * deltaE;
const double cmRate   = cfg.Cmq * qHat;
out.Cm = cmStatic + cmRate;                                          // XFLR5's own convention, diagnostics only
const double my = out.qbar * cfg.S * cfg.c_ref * (-cmStatic + cmRate); // only the static group is negated
```

`validation` also confirmed **no equivalent fix is needed for `Mx` (roll) or `Mz` (yaw)**, closing out the question of whether this class of bug is isolated to pitch:
- **Roll (`+X`)**: does not flip handedness between FRD and FLU at all (both `Y` and `Z` flip meaning for this axis pair — an even number of flips) — `Cl`→`Mx` needs no correction. Independently re-derived by `validation` via rotation-matrix construction on `+X`, matching the axis-rotation table already in `AeroModel.hh`/§19.6.
- **Yaw (`+Z`)**: *does* flip handedness (an odd number of flips, same class of issue as pitch), but — unlike alpha — `beta` had a genuine, free sign-convention choice available (§19.6's Candidate A/B analysis), and Candidate B was selected specifically because it made the given, unmodified `Cnb`/`CYb` signs behave correctly through the literal, unmodified `Mz=qbar·S·b·Cn` formula. That choice already absorbed the needed compensation for the *entire* `Cn` quantity (including `Cnda·delta_a` and `Cndr·delta_r`, which share the same "Cn" convention as `Cnb`, all summed into one `Cn` before one `Mz=qbar·S·b·Cn` conversion) — so no separate `Mz`-side correction is needed or applied. Alpha had no equivalent free choice (its sign is physically unambiguous — "nose up" has one meaning), which is precisely why pitch was left exposed and yaw was not.

**Fix 2 — missing `CLq·q_hat` term (MAJOR).** `aero_v1_config.yaml` documents `CLq=9.48457` as `CONFIRMED` and §19.4 already stated the intended V1 formula as `CL = CL0 + CLa·alpha + CLq·q_hat`, and `AerodynamicsSystem.cc` correctly loads `CLq` into `AeroConfig`, but `SaturatedCL()`/`ComputeAero()` never referenced it — a real, non-negligible omission (`validation`: ≈16% of `CL0` at `q=1 rad/s`, `V=15 m/s`), not dangerous (no NaN/instability) but a documented-vs-implemented mismatch. **Fixed**: the high-alpha saturation is applied to the alpha-driven static term only (`SaturatedCL(cfg, alpha)`, unchanged — the source data's `9–9.5°`/`CLmax` reliability boundary characterizes alpha, not `q_hat`), and `cfg.CLq * qHat` is added on top, unsaturated — mirroring the same static/rate split adopted for `Cm` in Fix 1:

```cpp
out.CL = SaturatedCL(cfg, out.alpha) + cfg.CLq * qHat;
```

`CD = CD0 + k·CL²` is fed this full (static+rate) `CL`, so induced drag reflects the complete lift coefficient.

**Verification performed this sub-pass:**
- Plugin rebuilt clean (`-Wall -Wextra -Wpedantic`, no warnings).
- Standalone self-test re-run: **17 PASS, 0 FAIL, 3 INFO** (up from 15/1/3) — `Cma_RESTORING_SIGN_TEST` now passes (`My(trim)=-0.000014` ≈ 0 as expected at the zero-moment trim point, `My(+2° nose-up)=+1.617668`, restoring); a new regression check (`RATE_NORMALIZATION_TEST (CLq*q_hat now included in CL)`) confirms `CL(q=1)-CL(q=0)` matches `CLq·q_hat` exactly; `Cmq_DAMPING_SIGN_TEST`/`Clp_DAMPING_SIGN_TEST`/`Cnr_DAMPING_SIGN_TEST` all still pass unchanged (regression-clean — the rate group was correctly left untouched); the neutral-trim `DRAG_POLAR_TEST`/`TRIM_BENCHMARK` (`q_hat=0` there) is numerically unaffected, still `CL=0.471685`.
- Live Gazebo smoke test re-run (`gz sim -s -r`, freefall world, 20 iterations): plugin loads, configures, runs with no error/crash/NaN.

**Requested from `gazebo-testing` (per the coordinating review):** re-run at minimum `Cma_RESTORING_SIGN_TEST`, `Cmq_DAMPING_SIGN_TEST` (regression check), `ELEVATOR_PITCH_SIGN_TEST` (expected sign flip, not a regression — see Fix 1's `Cmde` note), and ideally `ZERO_AIRSPEED_AERO_TEST`/`DRAG_POLAR_TEST`/`HIGH_ALPHA_LIMITER_TEST` (since the `CL` computation changed). `validation` re-review follows before this can be considered `AERODYNAMICS_V1_READY`.

### 19.13 `CONTROL_SURFACE_SIGN_MAPPING` (2026-08-22) — `elevator_sign` corrected, all three sign parameters now `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST`

Scope: a narrow, config-only change to `docs/source_of_truth/aerodynamics/aero_v1_config.yaml`'s `control_mapping` block (§19.10). No aerodynamic coefficient (`CYb`/`Clb`/`Cnb`/etc.), mass, CG, inertia, or structural value was touched — this section does not re-open or restate any of those derivations.

**Provenance:** `controls-integration` performed a full geometric determination of the physical trailing-edge direction produced by a positive joint angle on all 5 control-surface joints. `gazebo-testing` independently confirmed every part of it with live kinematic measurements (world-frame position of a TE reference point vs. commanded joint angle) *and* live aero-moment measurements (commanded joint angle → real applied `Mx`/`My`/`Mz`, read back from the running `FalconV2Aerodynamics` plugin) — 9/9 `CONFIRMS-HYPOTHESIS`, no refutations. Full records: `docs/test_results/2026-08-22_control_surface_sign_mapping_test_report.md`, `tests/gazebo/results/control_surface_sign_mapping_*`.

**Result:**

| Parameter | Old value/status | New value/status | Evidence |
|---|---|---|---|
| `elevator_sign` | `+1.0`, `ASSUMPTION` | **`-1.0`** (changed — was backward), `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST` | Kinematic: `+`joint angle moves TE **up** on both `left_elevator_joint`/`right_elevator_joint` (bit-identical between sides — confirms the pre-existing `elevator_symmetric_convention` SUM formula, §19.10, was already structurally correct, only the sign scalar was wrong). XFLR5 convention is `+delta_e = TE-down` (`CONTROLS.md` §4.1), so physical TE-up is XFLR5-negative. Aero-moment cross-check under the pre-fix sign: commanding a physical TE-up motion (`theta_left=theta_right=+8°`) measured `My=+1.128 N·m` (nose-down, using this plugin's own §19.12-resolved `My` convention) — backward from the textbook TE-up→nose-up relationship, i.e. numerically consistent with the sign being wrong in exactly the direction now corrected. |
| `aileron_sign` | `+1.0`, `ASSUMPTION` | `+1.0` (unchanged), `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST` | Kinematic: both aileron joints share the same TE-up-for-`+`-angle sense (common-mode, not naturally differential — confirms the `aileron_differential_convention` DIFFERENCE formula, §19.10, was already structurally correct). Aero-moment: `theta_left=-8°/theta_right=+8°` → `Mx=+4.877 N·m` (reproduced twice, 4 sig figs); common-mode `theta_left=theta_right=+8°` → `Mx=0.0` exactly, directly confirming the differential-extraction formula correctly rejects common-mode input. |
| `rudder_sign` | `+1.0`, `ASSUMPTION` | `+1.0` (unchanged), `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST` | Kinematic: `+`joint angle moves TE toward `-Y` (right). Aero-moment: `theta_rudder=+8°` → `Mz=-0.446 N·m` (nose-right, reproduced twice). |

**Change applied:** `aero_v1_config.yaml`'s `control_mapping.elevator_sign` changed from `1.0` to `-1.0`; `aileron_sign`/`rudder_sign` values unchanged; all three status tags (plus `aileron_differential_convention`/`elevator_symmetric_convention`) updated from `ASSUMPTION` to `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST` with the above provenance recorded inline. No plugin C++ source changed (the sign values are config-loaded, not hardcoded) — the built `.so` needed no rebuild for this fix; only `plugins/aerodynamics/test/aero_model_selftest.cc`'s hardcoded config mirror (used solely because the standalone self-test has no YAML loader) was updated to `elevatorSign=-1.0` to stay a faithful copy of the real config, and its `ELEVATOR_PITCH_SIGN`/`AILERON_ROLL_SIGN`/`RUDDER_YAW_SIGN` `INFO` messages were updated to reference the now-confirmed live evidence instead of "needs live test".

**Verification:** plugin rebuilt clean; standalone self-test re-run, still **17 PASS, 0 FAIL** (the sign fields are not exercised by `AeroModel.hh::ComputeAero()` itself — the joint→`delta_x` sign mapping happens upstream in `AerodynamicsSystem.cc`, so this is a no-regression confirmation, not a new pass/fail path); live Gazebo smoke test re-run (`gz sim -s -r`, freefall world, 20 iterations) confirms the plugin loads and parses the corrected YAML with no error.

**Closed, per the coordinating review:** `gazebo-testing` has since run `ELEVATOR_SYMMETRIC_MAPPING_TEST` and `ELEVATOR_AERO_MOMENT_SIGN_TEST` against the corrected config — both passed live (post-fix `My=-1.4856 N·m`, nose-up, correct). `validation` has since completed its final independent review and given a `CONTROL_SURFACE_SIGN_MAPPING_READY` verdict, with 0 CRITICAL/MAJOR findings (only 4 MINOR documentation-currency items, closed by a subsequent `DOCUMENTATION_CLEANUP` pass).

**Follow-up item logged, NOT investigated or fixed this pass (out of scope per explicit instruction):** `controls-integration` separately flagged that `Cldr` (`+0.0007/rad`, roll-due-to-rudder coupling, §7.2) appears inconsistent in sign with the classical vertical-tail-height×`CYdr` mechanism, given the rudder's position above the CG in the Gazebo/CAD Z-up frame. This is related to this document's own already-open question (§19.6/§19.9, and the general `Cl`-family axis-convention risk noted in §9/§10/§13) about whether roll-axis-adjacent derivatives coupling through a vertical moment arm need the same kind of scrutiny given to `Cma`/`My` in §19.12 — `Cldr` specifically was **not** covered by that analysis (§19.12 only examined `Cm`/`My`, and separately confirmed no `Mx`/`Mz` axis-handedness correction is needed for the `beta`/rate/aileron/rudder terms already in the model — but that confirmation did not specifically re-derive the physical sign expectation for a *rudder-induced roll* coupling term via the vertical-tail-moment-arm mechanism `controls-integration` is describing, which is a different physical mechanism than the roll-axis-handedness question already closed). Logged here as a candidate item for a future `aerodynamics` pass (not this one — no coefficient may be changed under `CONTROL_SURFACE_SIGN_MAPPING`'s scope), pending its own dedicated review. **This item is directly picked up in §20.3 below** (task `HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION`) — the vertical-tail-moment-arm mechanism `controls-integration` raised is exactly the kind of "decisive geometric reason" that §20.3 searched for and did not find documented anywhere in the repository, which is part of why §20.3 concluded `UNRESOLVED_KEEP_CURRENT` rather than force-adopting either sign.

---

## 20. `HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION` (2026-08-26) — Wide-deflection control-surface lookup model, 1A/1B resolutions

**Scope of this pass:** follows `CONTROL_AUTHORITY_EFFECTIVENESS_VALIDATION` (2026-08-26, `docs/test_results/2026-08-26_control_authority_effectiveness_validation.md`, and its own comparison doc `control_surface_analysis/CONTROL_AUTHORITY_EFFECTIVENESS_COMPARISON.md`), which is read as full prior context and not re-derived here. This pass (a) resolves the two open items that prior stage explicitly left for this stage (`CY_delta_a`'s 5.7× gap, `Cl_delta_r`'s sign conflict), (b) applies the Part-2 unconditional/conditional small-signal constant updates, and (c) replaces the old linear-coefficient + generic `±10°` clamp control-surface model with a bounded (`±45°`) piecewise-linear wide-deflection lookup, built from a new XFLR5 Type-1 fixed-condition wide-deflection sweep. **Not touched:** `CL0`/`Cm0`/`CD0`/`dragK`, the high-alpha limiter, `plugins/actuators/`, `docs/source_of_truth/controls/actuator_v1_config.yaml`.

Files touched: `plugins/aerodynamics/AeroModel.hh`, `plugins/aerodynamics/AerodynamicsSystem.cc`, `docs/source_of_truth/aerodynamics/aero_v1_config.yaml`, `plugins/aerodynamics/test/aero_model_selftest.cc`, this document.

### 20.1 Source data verification

Every number transcribed into `aero_v1_config.yaml`'s new `control_surface_lookup` block was checked line-by-line against `docs/source_of_truth/aerodynamics/control_surface_analysis/FALCON_V2_CONTROL_SURFACE_WIDE_DEFLECTION_RESULTS.txt` (XFLR5, Type 1 fixed-speed, 3D-Panels/VLM2, viscous OFF, mass 6.000 kg, primary operating point V∞=18.162 m/s, alpha=2.472°, beta=0°, only the tested surface deflected per sweep) before use — no value in this pass was taken from a secondary transcription without independent verification against this source file. The raw source file itself was not modified.

### 20.2 Resolution 1A — `CY_delta_a`: `RESOLVED_NEW_VALUE_VALID`

**Question:** current Gazebo/config `CYda_per_rad ≈ +0.0254/rad` (from the OLD Type-7 aileron sweep, §7.3) vs. the new fixed-condition sweep's `≈ +0.0045/rad` — a ~5.7× gap, much larger than the ~26–30% gaps seen on `Cl_delta_a`/`Cn_delta_a` from the same two datasets.

**Verification performed (not re-derived from the prior stage's flag alone):**

1. **Old-sweep trim contamination, confirmed directly:** re-read §7.3's own table — `alpha` runs 0.421° (`delta_a=0`) → 1.565° (`delta_a=±10°`), `V` runs 21.19→20.59 m/s across the same range. This is a Type-7 **re-trimmed** sweep (each row solves a separate 1g trim), not a fixed-condition sweep — confirmed, not assumed.
2. **New-sweep methodology, confirmed directly:** the new file's aileron section states `V∞=18.162 m/s`, `alpha=2.472°`, `beta=0°`, `Elevator=0°`, `Rudder=0°` for every row — a true Type-1 fixed-condition, single-surface-isolated sweep. Internal consistency check: `CYda` computed from the ±2/±5/±10° central-difference windows gives 0.00458/0.00447/0.00458 /rad — agreement to 2 significant figures, confirming this is a genuine small-signal linear derivative, not a sweep artifact.
3. **Mechanical/definitional bug check (required by the task before accepting trim-contamination as sufficient) — performed, RULED OUT:** compared whether the two sessions use the same `delta_a` normalization (differential half-angle vs. full WF-gain angle). The new file's own "Mapping validation" line (`delta_a=-10° → Cl=-0.07230`, `delta_a=+10° → Cl=+0.07231`) uses the identical differential convention as the old table's `Cl(±10°)=±0.0534` (same sign, same order of magnitude, both scaling the same way with `delta_a`). Quantitatively: `Clda` ratio new:old = 0.414/0.308 ≈ 1.34×; `Cnda` ratio = 0.0017/0.00144 ≈ 1.18×. Neither is close to a 2×/0.5× jump, which is what a shared definitional/factor-of-two bug across all three coefficients (`Cl`/`Cn`/`CY` all derive from the same `delta_a` input in both sessions) would be expected to produce **uniformly**. Since only `CY` shows an anomalously large (5.7×) gap while `Cl`/`Cn` show modest (1.18–1.34×) gaps from the identical input, a universal mapping/sign/factor-of-two bug is not a consistent explanation — this rules out a live mechanical bug as the primary cause.
4. **Honest counter-evidence also checked and reported (not hidden):** within the OLD dataset alone, its own near-center (`±2°`) vs. full-range (`±10°`) window instability is *not* larger in relative terms for `CYda` (0.0254→0.0268, ~5.5%) than for `Clda` (0.308→0.330, ~7%). This means the simple "smallness of the coefficient causes proportionally larger trim contamination" argument is *suggestive*, not quantitatively proven, purely from within-dataset evidence.

**Decision: `RESOLVED_NEW_VALUE_VALID`.** `CYda_per_rad` is updated to `+0.0045/rad`. Reasoning: the new value is adopted primarily because (a) its extraction methodology is unambiguously superior (fixed-condition, single-surface-isolated, vs. a re-trimmed sweep with confirmed real alpha/V excursion) for isolating a small-signal control derivative, (b) the most likely alternative explanation for the outsized gap — a mechanical/definitional bug — was actively checked for and ruled out, since it would have produced a uniform effect across `Cl`/`Cn`/`CY` and did not, and (c) the new value is highly self-consistent internally. The magnitude-based "small coefficients are more trim-sensitive" argument is offered only as a *plausible contributing mechanism* (sideforce is generated substantially through fuselage/vertical-tail sidewash response to the aileron-deflected wake, a coupling more sensitive to operating alpha than the locally-dominated `Cl`/`Cn` effects), not as the sole basis for the decision — the counter-evidence in point 4 is recorded so this is not misrepresented as a proven mechanism.

### 20.3 Resolution 1B — `Cl_delta_r`: `UNRESOLVED_KEEP_CURRENT`

**Question:** current/old `Cldr_per_rad ≈ +0.0007/rad` (§7.2, Type-7 sweep at the neutral-vertical-fin trim point, V=21.244 m/s/alpha=0.364°) vs. the new fixed-condition sweep's `≈ -0.00065/rad` (V=18.162 m/s/alpha=2.472°) — opposite sign, comparable (very small, tertiary) magnitude.

**Search performed for a decisive geometric/methodological reason to prefer one session:**

- **Viscous setting:** both sessions explicitly state Viscous = OFF (checked directly in both source documents) — no difference.
- **Vertical-tail/rudder geometry:** the master dataset's §26–§29 (vertical-tail airfoil, XFLR5 placement, rudder hinge x/c-vs-Z taper schedule) describes a single vertical-tail/rudder geometry and hinge schedule for this aircraft, used throughout the "neutral fin" full-aircraft configuration referenced everywhere in the master dataset (§30's own section title, "NEUTRAL FIN TYPE 7"). No second, differently-rigged or differently-toed vertical-fin configuration is described anywhere in the master dataset or the new wide-deflection file — "neutral fin" is this aircraft's one documented vertical-fin configuration, not one of several being compared between the two sessions.
- **Panel count / VLM mesh density:** not stated numerically in either source document for either session — cannot be compared quantitatively.
- **The one confirmed difference between the two sessions is the operating point itself** (alpha 0.364° vs. 2.472°, V 21.244 vs. 18.162 m/s). `Cl_delta_r` is ~2 orders of magnitude smaller than `CY_delta_r`/`Cn_delta_r` (a tertiary rudder-to-roll coupling). `controls-integration`'s earlier flag (§19.13's follow-up item, reproduced above) — that `Cldr`'s sign looks inconsistent with the classical vertical-tail-height × `CYdr` rolling-moment-arm mechanism — is exactly the kind of geometric mechanism that, if a documented vertical CP height or fin-rigging difference existed between the two sessions, could decisively explain a sign flip. No such documented difference was found in either source file.

**Decision: `UNRESOLVED_KEEP_CURRENT`.** No decisive geometric or methodological reason was found in the available documentation to prefer either session's sign over the other. Per the task's explicit instruction not to force a resolution that cannot be defended, `Cldr_per_rad` **keeps its old value, `+0.0007/rad`**. This is not a claim that the old value is "known correct" — only that neither sign has been shown superior with the evidence in this repository. A future pass with direct access to both underlying XFLR5 project files (not just their exported text summaries) is needed to close this properly.

**Consistency handling for the wide-deflection `Cl(delta_r)` lookup (required by the task, applied here):** the new wide-deflection file's `Cl(delta_r)` table is **not** loaded into the plugin at all. It is reproduced in `aero_v1_config.yaml` under the explicit key `Cl_NOT_LOADED_disputed_sign_reference_only` purely for provenance/traceability (so the raw new-session data remains inspectable), and `AerodynamicsSystem.cc`'s loader does not read it. Instead, `AeroModel.hh`'s `AeroConfig::Prepare()` builds the runtime rudder-roll lookup (`ctrlRuddCl`) as a **bounded linear extension of the OLD `Cldr_per_rad` constant** across the full `±45°` breakpoint grid (`ctrlRuddCl[i] = Cldr * ctrlBreakpointsRad[i]`) — verified exactly (max error 0 at machine precision) by the self-test's `RUDDER_CL_LINEAR_EXTENSION_TEST`. This avoids silently injecting the disputed new sign into the wide-deflection model while still giving the rudder-roll coupling *some* bounded, non-extrapolating (in the "clamped at ±45°" sense) representation across the full mechanical range. **The true wide-deflection `Cl(delta_r)` shape — whether it is actually linear out to ±45°, and which sign is correct — remains an open item / `DATA_REQUIRED`** for a future pass.

### 20.4 Part-2 small-signal constant updates applied

| Constant | Old | New | Basis |
|---|---|---|---|
| `CL_delta_e` | *(did not exist)* | `+0.414/rad` | New fixed-condition data (unconditional, new field `CLde_per_rad`) |
| `Cm_delta_e` | `-0.73/rad` | `-1.000/rad` | New fixed-condition data (unconditional) |
| `Cl_delta_a` | `0.308/rad` | `0.414/rad` | New fixed-condition data (unconditional) |
| `Cn_delta_a` | `0.00144/rad` | `0.0017/rad` | New fixed-condition data (unconditional) |
| `CY_delta_r` | `0.085/rad` | `0.0916/rad` | New fixed-condition data (unconditional) |
| `Cn_delta_r` | `-0.025/rad` | `-0.0272/rad` | New fixed-condition data (unconditional) |
| `CY_delta_a` | `0.0254/rad` | `0.0045/rad` | §20.2, `RESOLVED_NEW_VALUE_VALID` |
| `Cl_delta_r` | `0.0007/rad` | `0.0007/rad` (unchanged) | §20.3, `UNRESOLVED_KEEP_CURRENT` |

All eight are now `SUPERSEDED_BY_LOOKUP` for the actual force/moment formula (see §20.5) **except** `Cl_delta_r`, which remains functionally used as the source constant for the linear-extension rudder-roll lookup (§20.3). All eight remain in `aero_v1_config.yaml` as documented small-signal reference constants, each verified by the self-test's `SMALL_SIGNAL_RECOVERY` checks to be closely reproduced by a central difference of the corresponding lookup table near `delta=0` (§20.7).

### 20.5 Architecture: bounded piecewise-linear wide-deflection lookup

Elevator, aileron, and rudder each now get a **full-lookup-replaces-static-term** piecewise-linear table over the shared 15-point breakpoint grid `[-45,-35,-25,-15,-10,-5,-2,0,+2,+5,+10,+15,+25,+35,+45]°` (`AeroModel.hh`'s `InterpLinear()`, `kNumCtrlBreakpoints=15`, `kCtrlZeroIndex=7`):

- **Elevator:** `CL = SaturatedCL(alpha) + CLq·q̂ + LookupDCL_e(delta_e)`; `cmStatic = Cm0 + Cma·alpha + LookupDCm_e(delta_e)` (replaces `Cmde·deltaE` exactly — not added on top). `LookupDCD_e(delta_e)` feeds §20.6. Source table is already baseline-differenced in the source file (delta_e=0 row is exactly 0), so no additional differencing was needed.
- **Aileron:** `LookupCl_a`/`LookupCn_a`/`LookupCY_a` (raw source values, tiny ~1e-5 residual baseline at `delta_a=0` preserved as given, not zeroed, per the no-fabrication rule) replace `Clda·deltaA`/`Cnda·deltaA`/`CYda·deltaA` exactly, added to the untouched `Clb·beta+Clp·p̂+Clr·r̂` etc. base terms. The source `CD`/`CL`/`Cm` rows are **full values** at fixed alpha (a `delta_a` sweep, not baseline-differenced in the source file) — `AeroConfig::Prepare()` subtracts the `delta_a=0` row (`kCtrlZeroIndex`) to produce `dCD_a`/`dCL_a`/`dCm_a`, the even/symmetric secondary corrections. `dCL_a`/`dCm_a` are included in the `CL`/`Cm` build-up (optional per the task brief, included since the data gives them cleanly at no extra cost); `dCD_a` feeds §20.6.
- **Rudder:** `LookupCY_r`/`LookupCn_r` (raw source values) replace `CYdr·deltaR`/`Cndr·deltaR` exactly. `LookupCl_r` is the §20.3 linear-extension-of-`Cldr` table, not the new source table. `dCD_r` (from the full-value `CD` table, differenced the same way as aileron) feeds §20.6. Rudder's small `CL`/`Cm` secondary correction (present in the source data, explicitly marked optional by the source file itself — "may be retained if desired") was **not** implemented this pass, since the task's Part-3 spec for rudder only specified `CY`/`Cn`/`Cl`/`CD` — left as a documented, non-fabricated future option, not silently added or silently omitted.

**Domain bound:** `InterpLinear()` clamps any input at or beyond the first/last breakpoint to that breakpoint's value (no extrapolation) — the domain is exactly `[-45°, +45°]`, matching the actuator's mechanical range (`docs/source_of_truth/controls/actuator_v1_config.yaml`, `±0.7853981634 rad`) exactly. This **replaces** the old generic `control_deflection_clamp_deg=10.0` (`V1_CONSERVATIVE_CLAMP`), which is removed from `aero_v1_config.yaml` (key deleted, not merely widened) and no longer read by `AerodynamicsSystem.cc`'s loader. This is not a re-introduction of the old "clamp then extrapolate nothing" policy under a different name — it is a genuinely wider *and* real-data-backed bound: every point in `[-45°,+45°]` is now backed by an actual XFLR5 data point at one of the 15 breakpoints, with linear interpolation (not extrapolation) between them. The separate `high_alpha_limiter` block (an angle-of-attack concept) is untouched and unrelated.

**No double-counting confirmed:** for every one of the eight superseded scalar constants (§20.4), the corresponding old linear term (`Cxda·deltaA`, `Cxdr·deltaR`, `Cmde·deltaE`) was removed from `AeroModel.hh::ComputeAero()`'s coefficient build-up in the same edit that added the replacing lookup call — verified by direct code inspection (no line in `ComputeAero()` references `cfg.Clda`, `cfg.Cnda`, `cfg.CYda`, `cfg.CYdr`, `cfg.Cndr`, or `cfg.Cmde`/`cfg.CLde` anymore; `cfg.Cldr` is referenced only inside `Prepare()`, to build `ctrlRuddCl`, never directly inside `ComputeAero()`).

### 20.6 Drag integration (Part 4)

```
CD_total = CD0 + dragK·CL² + dCD_e(delta_e) + dCD_a(delta_a) + dCD_r(delta_r)
```

This is an explicit, documented **V1 additive approximation** for simultaneous multi-surface deflection (tagged `V1_ADDITIVE_MULTI_SURFACE_DRAG_APPROXIMATION` in code comments): each `dCD_x` was measured with *only that one surface* deflected (single-surface isolated sweeps); no combined-deflection XFLR5 sweep exists in the source of truth to validate simple linear superposition when multiple surfaces are deflected together. Not fabricated interaction physics — a documented linear-superposition assumption, flagged as unvalidated for combined deflection.

**Negative-CD floor:** `CD_total` is floored at `CD0` (`out.CD = max(cdRaw, CD0)`), not at `0`. Chosen because `CD0` is the aircraft's own documented, real parasite-drag constant (§6.5) that always physically exists regardless of control-surface deflection — flooring at exactly `0` would imply a physically implausible zero-drag airframe, which is less defensible than flooring at the aircraft's own baseline. This matters because several `dCD_e` values are slightly negative near small negative elevator deflections (e.g. `-0.00034` at `delta_e=-5°`) — aileron/rudder `dCD` are drag-increasing at every nonzero deflection in the source data, so only elevator can drive `cdRaw` below `CD0`. Verified by the self-test's `DRAG_FLOOR_TEST`.

### 20.7 Self-test results

`plugins/aerodynamics/test/aero_model_selftest.cc` extended with: `BREAKPOINTS_SORTED_TEST`, `LOOKUP_EXACT_BREAKPOINT_TEST` (all 13 lookup curves across all 3 surfaces return the exact table value at all 15 breakpoints — 195 checks, max error < 1e-12), `BASELINE_DIFFERENCE_ZERO_AT_ORIGIN_TEST`, `RUDDER_CL_LINEAR_EXTENSION_TEST` (confirms §20.3's linear-extension derivation exactly, max error 0), 8× `SMALL_SIGNAL_RECOVERY` checks (central-difference slope of each lookup near `delta=0` at `±2°/±5°/±10°` windows, compared against the §20.4 reference constants — all within 2–10% depending on coefficient magnitude/nonlinearity, see code for exact tolerances and rationale), `LOOKUP_NO_EXTRAPOLATION_TEST` (9 curves × 10 out-of-domain inputs up to `±1000°` — all clamp exactly to the nearest edge value, all finite), and `DRAG_FLOOR_TEST`. All pre-existing tests (unaffected in intent by this pass, since they mostly use `deltaA=deltaE=deltaR=0`) were re-verified to still pass. **Result: 31 PASS, 0 FAIL, 3 INFO** (the 3 `INFO` items — `AILERON_ROLL_SIGN`/`RUDDER_YAW_SIGN`/`ELEVATOR_PITCH_SIGN` — remain algebraic-only by design, same as before this pass; the underlying joint-sign-mapping chain they describe was not touched by this pass).

### 20.8 Trim-impact diagnostic (report only — no retrim performed)

At the existing validated trim point (throttle=0.4915, elevator physical trim +5.50°L/R → `delta_e_aero≈-5.4995°`, V≈18.165 m/s, alpha≈2.46°), computed via the actual `ComputeAero()` code path (both the pre-pass OLD formula, hand-evaluated with the old `Cmde=-0.73`/no-`CLde` linear model at this unclamped `delta_e`, and the NEW lookup-based model):

| Quantity | OLD | NEW | Δ (NEW−OLD) |
|---|---|---|---|
| `CL` | 0.670857 | 0.631166 | −0.039691 |
| `Cm` (diagnostic, XFLR5-unflipped) | 0.009430 | 0.035168 | +0.025738 |
| Lift (N), `qbar=202.10` | 61.20 | 57.58 | **−3.62 N** |
| `My` (N·m, static-only, `q̂=0`) | −0.1927 | −0.7187 | **−0.526 N·m** |

This is **diagnostic only** — no coefficient, trim search, or actuator command was adjusted to compensate. Physically: at this exact trim deflection/alpha, the new model (a) predicts noticeably less lift than the old model, because `CL_delta_e` was previously entirely absent and this specific trim uses a sizeable (~5.5°) TE-up elevator deflection, and (b) predicts a stronger nose-up pitching tendency (`My` more negative) than the old model at the same point — meaning, if this trim point were re-searched under the new model (not done here), a somewhat different elevator/alpha combination would likely be needed to re-achieve level flight at the same speed. This is exactly the kind of finding the diagnostic is meant to surface, per the task's explicit "report only, do not retrim" instruction — routed to `gazebo-testing`/`validation` for the next stage's trim re-verification, not resolved here.

### 20.9 `DATA_REQUIRED` / open items after this pass

- The true sign and shape of `Cl_delta_r` beyond small-signal (§20.3) — needs direct access to both underlying XFLR5 project files, not just their exported text summaries.
- Rudder's small `CL`/`Cm` secondary wide-deflection correction (data exists, not implemented this pass — see §20.5).
- Multi-surface simultaneous-deflection drag/force interaction (§20.6's additive approximation is unvalidated for combined deflection).
- Everything already listed as `DATA_REQUIRED` in §7.4/§11/§18 that this pass did not touch (hinge-moment data, Reynolds/airspeed variation of control effectiveness, etc.).

## 21. Beta-saturation modeling gap — assessed, not fixed (`UPDATED_POWERED_TRIM_AND_HIGH_DEFLECTION_FLIGHT_VALIDATION`, 2026-08-27)

`gazebo-testing`'s aileron ±15°/±25° high-deflection pulse tests (`tests/gazebo/results/high_deflection_flight_result.json`) produced sideslip excursions of 15.0–32.0 deg (`max_abs_beta_deg`), while `CYb`/`Clb`/`Cnb` are applied in `AeroModel.hh`'s `ComputeAero()` as unbounded linear terms in `beta`, with no analog of the `high_alpha_limiter`'s saturation or the ±45° control-deflection domain bound. This is judged a REAL modeling gap (CYb/Clb/Cnb are small-disturbance derivatives from a beta=0 reference condition, MD/§6.2, with no documented validity claim at 15–32° sideslip), but assessed as MINOR severity for that specific validation pass and NOT invalidating any number already reported there, for three reasons: (a) `CL`, `Cm`, and `CD` have zero beta-dependence anywhere in `ComputeAero()` — only `CY`/`Cl`/`Cn` carry a beta term, so those three coefficients are structurally immune to this gap regardless of beta magnitude; (b) back-solving beta from the measured `CY` at the EARLY (near-static, t=0.08–0.18s post-pulse-onset) sample that anchors gazebo-testing's headline control-effectiveness numbers (Cl_delta_a antisymmetry, control-drag deltas, 10–25° smoothness checks) gives beta≈0.37–0.39 deg at ±15° and beta≈0.47–0.57 deg at ±25° (aileron +25°=+0.5344°, aileron -25°=-0.5718°, rudder +25°=+0.4692°, rudder -25°=-0.5070°; full EARLY-window span across ±15°/±25° ≈0.37–0.57 deg) for both the aileron and rudder pulses at that sampling instant (`Clb*beta` of order -5e-5 to -4e-5, negligible against the reported `Cl` values of 0.06–0.11) — the large 15–32° beta excursions only develop later, well into the SETTLED/tail-return phase, by which point roll has already passed 90–180 deg (a large-angle departure from small-disturbance conditions for reasons that have nothing to do with beta specifically); (c) the SETTLED-window `Cl` values (already self-labeled by gazebo-testing as "DYNAMIC steady state — includes rate damping", never presented as a clean small-signal `Cl_delta_a`) are the only place a non-negligible `Clb*beta` contribution is plausible, and that is an already-acknowledged limitation of using a dynamic pulse to probe a static derivative, not a new contamination discovered by this review. No fix (beta clamp/saturation model) is proposed here — logged as a candidate item for a future dedicated review, analogous in spirit to the existing `high_alpha_limiter`.

---

## 22. Flight Envelope Classification — Interpretation Record (`FLIGHT_ENVELOPE_VALIDATION`, 2026-08-27)

**Scope: interpretation/classification only.** No coefficient, lookup table, or the `high_alpha_limiter` was retuned or touched to produce this section. Data source: `tests/gazebo/results/flight_envelope_result.json` (8-speed trim sweep, 3 free-flight runs, control-authority-vs-speed sweep, and its own `final_classification_reconciliation` per-speed reasoning) and `tests/gazebo/results/flight_envelope_log.txt`, both produced by `gazebo-testing`. This record persists the classification `aerodynamics` reported back to the coordinator for this stage, so it is reviewable in-repo rather than only existing in a chat transcript.

### 22.1 Three-tier envelope classification (8 speeds)

| V (m/s) | Tier | Reasoning |
|---|---|---|
| 12.5 | `OUTSIDE_VALIDATED_ENVELOPE` | Analytical `AERODYNAMIC_NO_TRIM_MOMENT` (real infeasibility, not search noise): My=+1.55 Nm residual (~2.4x the next-largest), mean\|q\|=7.4°/s (vs ≤2.7°/s everywhere else), alpha=11.40° is 2.15° past `alpha_transition=9.25°`, and V is below both the master dataset's `Vstall≈12.24 m/s` and `Vmin_safe≈14.69 m/s` (§5.2). |
| 14.0 | `PROVISIONAL_EDGE_ENVELOPE` | Corrected trim self-consistent (Lift/W=0.988, T/D=0.997) and free-flight `PASS` (14s, bounded, roll≤0.06°), but free-flight max\|alpha\|=9.23° sits essentially *at* `alpha_transition=9.25°`, and V=14.0 is still below the real-aircraft `Vmin_safe=14.69 m/s` reference — real corroboration exists, but at the edge of both the model's own reliability boundary and the cited safety margin. |
| 16.0 | `PROVISIONAL_EDGE_ENVELOPE` | Self-consistent trim (Lift/W=0.986, T/D=0.974, alpha=4.74° well below transition), but no direct free-flight run this stage. |
| 18.166 | `VALIDATED_CORE_ENVELOPE` | Reference trim, smallest My/mean\|q\| of all 8 points, independently validated by a prior 25s free flight + pulse tests, reconfirmed here. |
| 21.0 | `PROVISIONAL_EDGE_ENVELOPE` | Excellent trim numbers (smallest mean\|q\|=0.53°/s of all 8, T/D=1.077 matching the accepted 18.166 precedent), but no free-flight cross-validation this stage. |
| 24.0 | `VALIDATED_CORE_ENVELOPE` | Automated 5s-window classifier's `NO_VALID_TRIM` (tail_vz=-0.98 m/s) is corroborated as short-window phugoid-transient noise, not real divergence, by an actual 14s free-flight `PASS` at this exact point (bounded ~-0.8 m/s altitude drift, max\|pitch\|=4.39°); My=+0.095 Nm is 2nd-smallest of all 8. |
| 28.0 | `PROVISIONAL_EDGE_ENVELOPE` | Self-consistent, aerodynamically feasible, no actuator/propulsion limit, but growing residual (My=+0.392 Nm) and no free-flight test. |
| 30.0 | `PROVISIONAL_EDGE_ENVELOPE` | Largest residual (My=+0.645 Nm) among feasible points and no free-flight test, but RPM (9491/11500) and current (23.4/65A) stay comfortably inside propulsion limits — not propulsion-limited, just least-corroborated at the fast end. |

### 22.2 Model-predicted low-speed limit vs. real-aircraft stall-speed reference

The model's own trim search is moment-*infeasible* at 12.5 m/s (`AERODYNAMIC_NO_TRIM_MOMENT`) despite CL=1.34 being individually reachable on the no-stall saturation curve, and becomes moment-*feasible* with reasonable pitch behavior (mean\|q\| dropping from 7.4°/s to ≤2.7°/s) from 14 m/s upward — so the **model-predicted practical low-speed limit is ≈14 m/s**, explicitly a boundary artifact of this V1 no-stall/no-post-stall simulation architecture, not a claim about the real aircraft. It is a distinct, independently-derived number from the master dataset's real-aircraft references, `Vstall≈12.24 m/s` (manufacturer-CLmax-based) and `Vmin_safe≈14.69 m/s` (20% margin, §5.2) — the ~14 m/s model limit and the ~14.69 m/s real safety margin are numerically close but that is **coincidental agreement between two unrelated derivations, not cross-validation of either.**

### 22.3 qbar-scaling confirmation

Confirmed against the actual `flight_envelope_result.json` control-authority-vs-speed numbers: aileron +5° `dMx` grows 2.80→4.80→9.50 Nm across 14→18.166→24 m/s (ratio ≈1.7x/3.4x, tracking `qbar∝V²` ratios of 1.68x/2.94x once the differing actual achieved deflections are accounted for), while `dCl` stays roughly flat (0.0239→0.0247→0.0286, <20% variation) — exactly the expected `M = qbar·S·L·C(deflection)` physics with speed-independent lookup coefficients, not a derivative/model change.

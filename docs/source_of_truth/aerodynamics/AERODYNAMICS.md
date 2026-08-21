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

**XFLR5 sign convention, stated explicitly (`MD §22`, lines 601–603): `+ = trailing edge down`, `- = trailing edge up`.** This is XFLR5's own convention for the `delta_e` values below — **mapping it to an actual Gazebo elevator joint rotation sign is `controls-integration`'s task and a `DATA_REQUIRED`/unit-test item, not resolved in this document.**

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
- The XFLR5-to-Gazebo sign mapping for rudder and aileron deflection (only elevator's `+ = TE down` is explicitly stated in the master dataset).

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
| Elevator `delta_e` sign | CONFIRMED (XFLR5's own convention only) | `+ = trailing edge down`, `- = trailing edge up` (`MD §22`). Mapping to a Gazebo joint-rotation sign is `controls-integration`'s task — `DATA_REQUIRED` until a unit test confirms it. |
| Rudder `delta_r` sign | DATA_REQUIRED (physical direction not stated) | The Type7 sweep table (§7.2) gives numeric `CY/Cl/Cn` vs. `delta_r`, but the master dataset does not state which physical direction (nose left/right) a positive `delta_r` corresponds to. |
| Aileron `delta_a` sign / WF1..WF6 mapping | DATA_REQUIRED (physical direction not finalized) | `MD §35`/`§36` explicitly state the physical side/deflection mapping for adverse/proverse labeling is not finalized — not assumed here. |
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

---

## 12. Directly Gazebo-Usable Parameters (updated)

| Parameter | Status | Caveat still open |
|---|---|---|
| S=0.4514 m², b=2.093 m, c_ref≈0.224 m | CONFIRMED — now confirmed as XFLR5's own internal reference values (§2.2, §6.2), not merely assumed to match | None remaining on the *reference-quantity* question; sign/frame caveats (§9/§10) still apply to any position-like value used alongside them |
| CLa, Cma, CLq, Cmq, CXa, CXq (§6.2) | CONFIRMED (single reference operating point; §9/§10 sign caveats apply) | Valid at/near the neutral trim condition; not confirmed alpha-independent across the full envelope beyond the elevator sweep's own trim-point range (§7.1) |
| CYb/Clb/Cnb/CYp/Clp/Cnp/CYr/Clr/Cnr (§6.2) | CONFIRMED (unchanged from original reference point) | Sign convention vs. Gazebo FLU unresolved (§10) |
| CYda/Clda/Cnda, CYdr/Cldr/Cndr, Cmde (§7) | CONFIRMED within ±10° (§11) | Physical deflection-sign mapping (rudder/aileron) unresolved (§10); do not extrapolate past ±10° |
| CD0≈0.0351, k≈0.0528 (§6.5) | V1_CALIBRATED | Calibration model, not flight-measured; distinct from any inviscid Type7 CD |
| Dynamic-mode / trim benchmarks (§6.8) | VALIDATION_TARGET | For checking a future implementation, not for tuning it |

---

## 13. Parameters Requiring Conversion / Confirmation Before Gazebo Use

| Parameter | What's needed | Status |
|---|---|---|
| XNP/XCP → Gazebo/CAD frame | Full validated coordinate transform (§9) | PARTIALLY_DOCUMENTED — qualitative convention only |
| Lateral-directional derivative sign convention → Gazebo FLU | Unit-test confirmation (`SIDESLIP_SIGN_TEST` et al.) | DATA_REQUIRED |
| `alpha`/`beta` formula sign in FLU | Unit-test confirmation (`AOA_SIGN_TEST`) | FLAGGED, not resolved (§8.1/§10) |
| Rudder/aileron deflection physical-direction sign | Project-owner or controls-integration confirmation | DATA_REQUIRED |
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
6. `CL0`/`Cm0` constant-offset anchors for the linear build-up equations — not given explicitly; candidate derivation shown but not performed/authorized (§8.3).
7. Reconciliation of the `CXa`/`CXq` body-axis force derivatives with the `CL`/`CD`-based wind-axis force model (§6.2).
8. A full, validated Gazebo-frame coordinate transform for XNP/XCP and any other XFLR5-frame position value (§9) — qualitative convention only exists so far.
9. Confirmation of whether the "XFLR5 X-axis reversed" note applies to the stability-derivative results frame, or only the geometry-input frame (§9).
10. `AOA_SIGN_TEST`/`SIDESLIP_SIGN_TEST` resolution of the FRD-vs-FLU sign risk in the `alpha=atan2(w,u)`/`beta=asin(v/V)` formulas (§8.1, §10).
11. Physical (nose-left/right, surface-up/down) deflection-direction mapping for rudder and aileron `delta` sign (§10) — only elevator's `+ = TE down` is explicitly stated.
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

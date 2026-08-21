# FALCON V2 — Propulsion Source of Truth

**Owner:** `propulsion`
**Status:** Architecture-definition pass, now synced against the project master dataset (see update note below). **No propulsion plugin/code has been implemented anywhere in this repository as of this writing.** This document defines the target physics model, inventories what data exists vs. is missing, and records data-sourcing strategy. It contains no fabricated coefficients, curves, or performance numbers beyond what the master dataset records (and every such value keeps the master dataset's own status label — V1/provisional/estimate values are never promoted to final measured truth here).
**Compiled:** 2026-08-21
**Updated:** 2026-08-21 (master-dataset sync pass). `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` (§41–§63, §70–§72) was added to the repository and is now the primary source for this document. This pass (a) records confirmed motor/ESC/battery/propeller component data that previously read as unknown, (b) records the propeller counter-rotation direction (previously unknown) together with the reverse-handed-propeller caveat, (c) records the SI motor/propeller equation set and the static bench + APC-official validation references, and (d) explicitly preserves what the master dataset itself still marks incomplete (rotational inertia, the real APC Ct(J)/Cp(J) table, ESC/secondary-battery exact positions, battery internal resistance/SOC curve, motor efficiency/thermal/friction model). Nothing in this pass is promoted beyond the status the master dataset itself assigns it.
**Repository investigation performed (original pass):** full-text search of `CLAUDE.md`, `docs/source_of_truth/README.md`, `docs/source_of_truth/geometry/GEOMETRY.md`, `docs/source_of_truth/mass_properties/MASS_PROPERTIES.md`, `docs/architecture/`, `.claude/agents/`, and the working tree (`model/`, `tests/`) for propulsion-relevant keywords (`battery`, `mAh`, `Rm`, `no-load`, `I0`, `KV`, `Ct`, `Cq`, `advance ratio`, `coefficient`, `ESC`, `RPM`, `thrust`, `torque`, `motor mount`).

Status legend (consistent with `GEOMETRY.md`/`MASS_PROPERTIES.md`): `CONFIRMED` (stated directly by an authoritative source), `DERIVED` (computed from confirmed values, derivation shown), `ASSUMPTION` (explicitly documented, temporary), `V1`/`PROVISIONAL`/`ESTIMATE` (the master dataset's own non-final labels — carried through unchanged, never promoted), `PROPULSION_DATA_REQUIRED` (propulsion-specific data not present anywhere in the repository — not guessed), `APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION` (a narrower, distinct tag — the propeller-loading *architecture* is fully defined, but the real numeric Ct(J)/Cp(J) table is not yet in the repository), `VISUAL_MESH_ONLY` (a mesh-derived value confirmed to be a visual-geometry artifact, forbidden as a physics input).

**Update, second follow-up pass (same day):** motor/propeller hub and mount coordinates and the thrust-axis vector are now `CONFIRMED` directly by `geometry-structure` in `GEOMETRY.md` §7, and the main battery center/mass/CG-relative offset are now `CONFIRMED` directly by `geometry-structure` in `MASS_PROPERTIES.md` §6.1 — both published after this document's initial master-dataset sync pass. Every `PENDING_GEOMETRY_SYNC` tag used in the prior version of this document (the numbers were already known from the master dataset, but `geometry-structure` had not yet republished them as its own authoritative record) is replaced below with `CONFIRMED`, citing both `GEOMETRY.md` §7 / `MASS_PROPERTIES.md` §6.1 and the original master dataset section. The underlying numeric values are unchanged — this pass only removes the "not yet authoritative" caveat, per `geometry-structure`'s publication.

---

## 0. CRITICAL — the STL-vs-real-propeller distinction (read first, applies everywhere in this document and downstream)

> **The propeller STL meshes (`model/meshes/left_pervane.stl`, `model/meshes/right_pervane.stl`) measure at ≈273.4–273.5 mm (≈0.2734–0.2735 m) diameter. This figure is `VISUAL_MESH_ONLY` — a visual/collision-mesh artifact, confirmed by `geometry-structure` in `docs/source_of_truth/geometry/GEOMETRY.md` §20–§21.2, and it is explicitly NOT a physical propulsion reference.**
>
> **This ≈273 mm figure must NEVER be used, anywhere in this project, for:**
> - RPM calculations
> - thrust calculations
> - torque calculations
> - advance ratio (J) calculations
> - propeller disk-area calculations
> - blade-tip-speed calculations
> - motor-load calculations
> - throttle → RPM mapping
> - RPM → thrust mapping
> - airspeed-dependent-thrust modeling
>
> **The real, physical propeller for every propulsion physics calculation in this project is the APC 13x6.5E:**
>
> | Quantity | Value | Source |
> |---|---|---|
> | Nominal diameter | 13 in = 330.2 mm = **D = 0.3302 m** | Manufacturer nominal (APC 13x6.5E), per `CLAUDE.md` propulsion reference / project owner |
> | Nominal pitch | 6.5 in = 165.1 mm = **0.1651 m** | Manufacturer nominal (APC 13x6.5E), per `CLAUDE.md` propulsion reference / project owner |
>
> **`D = 0.3302 m` is used everywhere, always, in this project's propulsion physics — no exceptions.** The ≈17.2% mesh-vs-nominal discrepancy documented in `GEOMETRY.md` §21.2 is a geometric sanity-check finding only; no cause is asserted there (mesh authored at non-flight scale, simplified visual model, unit artifact, etc. are all left unresolved), and it has no bearing on which diameter this document uses — the mesh figure is never a candidate.

This document does not consume STL geometry directly anywhere below. Where mesh-derived data would be relevant (e.g. motor/propeller mount coordinates for force application points), it is cited as `geometry-structure`'s output and cross-referenced, not recomputed here.

**Related but distinct caveat (see §1, §7 for detail):** the left and right propellers are counter-rotating and require physically different-handed APC parts (13x6.5E and a reverse-handed 13x6.5EP counterpart) — this is a separate hardware-identity issue from the STL-diameter issue above, and is not resolved by anything in this section.

---

## 1. Confirmed propulsion configuration

### 1.1 Motor (SunnySky X2820 860KV) — component/electrical data

**Update, this pass:** the master dataset (§41) records collected manufacturer/web data for this exact motor. This clears the previous "motor electrical parameters unknown" framing specifically for KV, internal resistance, no-load current, mass, max current, and max power — these are now `CONFIRMED` component/manufacturer data, not derived or assumed. What is *not* cleared by this: motor rotational inertia, motor efficiency map, thermal model, and no-load/friction current-vs-RPM behavior beyond the single reference point — those remain open (§4, §8).

| Parameter | Value | Provenance | Status |
|---|---|---|---|
| Motor count / arrangement | 2 (left, right) | `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED |
| Motor model | SunnySky X2820 860KV | `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED (component identity) |
| Motor KV | 860 rpm/V (no-load, nominal) | Manufacturer nameplate spec, per `CLAUDE.md`; master dataset §41 | CONFIRMED |
| Motor internal resistance (R) | ≈0.0258 Ω (25.8 mΩ) | Manufacturer/web-collected data, master dataset §41 | CONFIRMED (component data — supersedes the prior `PROPULSION_DATA_REQUIRED` tag on Rm) |
| Motor no-load current (I0) | ≈1.3 A @ 10 V | Manufacturer/web-collected data, master dataset §41 | CONFIRMED (component data — supersedes the prior `PROPULSION_DATA_REQUIRED` tag on I0; note this is a reference point at 10 V, not a full no-load-current-vs-voltage curve — see §4) |
| Motor mass | ≈0.143 kg/motor (0.286 kg total, 2 motors) | Manufacturer/web-collected data, master dataset §41 | CONFIRMED |
| Motor max current | ≈65 A / 30 s | Manufacturer/web-collected data, master dataset §41 | CONFIRMED (rating, not a continuous-duty figure) |
| Motor max power | ≈960 W/motor | Manufacturer/web-collected data, master dataset §41 | CONFIRMED (rating) |
| Motor electrical compatibility | 4S-compatible; APC 13x6.5 listed as a recommended propeller pairing | Manufacturer/web-collected data, master dataset §41 | CONFIRMED |
| Motor rotational inertia (I_rotor, motor-only component) | — | Master dataset §53: "Exact rotational inertia henüz bilinmiyor. V1'de estimate veya bench spin-up kalibrasyonu gerekir." | `PROPULSION_DATA_REQUIRED` — genuinely open; only a V1-estimate/bench-calibration *path* exists, no number (§4, §8) |

### 1.2 ESC (Hobbywing Skywalker 80A)

| Parameter | Value | Provenance | Status |
|---|---|---|---|
| ESC model | 2 × Hobbywing Skywalker 80A | Master dataset §42 | CONFIRMED (component identity) |
| ESC mass | ≈0.080 kg each (0.160 kg total) | Master dataset §42 | CONFIRMED |
| ESC placement (qualitative) | In wings | `GEOMETRY.md` §9 provenance table ("ESC-in-wings fact"); master dataset §42 ("Kanat içinde oldukları biliniyor") | CONFIRMED (qualitative only) |
| ESC exact position (per side) | — | Master dataset §42: "Kesin konumlar bilinmiyor." | `PROPULSION_DATA_REQUIRED` — explicitly still unknown; do not invent (§8) |
| ESC V1 electrical model | `V_ESC = throttle × V_battery` (throttle ∈ [0,1]) | Master dataset §42 | CONFIRMED as the V1 target model (architecture, not yet implemented) |
| ESC V2 future work | PWM response, delay, efficiency, current limiting | Master dataset §42 | Explicitly labeled V2/future — not required for V1, not modeled here |

### 1.3 Batteries

| Parameter | Value | Provenance | Status |
|---|---|---|---|
| Main propulsion battery | 4S 22000 mAh, 25C | Master dataset §43 (supersedes the prior attribution to "project owner, direct conversation, 2026-08-21," now formally recorded in the repository's master dataset) | CONFIRMED |
| Main battery nominal voltage (Vnom) | 14.8 V | Master dataset §43 — recorded as this specific pack's spec, not merely a generic 4S LiPo chemistry convention | CONFIRMED |
| Main battery full-charge voltage (Vfull) | 16.8 V | Master dataset §43 | CONFIRMED |
| Main battery mass | ≈1.666 kg | Master dataset §43 | CONFIRMED |
| Main battery center coordinate | (0.300631, 0, 0.038547) m; CG-relative offset ΔX≈+0.132322 m, ΔY=0, ΔZ≈−0.061453 m | Master dataset §43/§7; now formally published by `geometry-structure` in `MASS_PROPERTIES.md` §6.1. **Placement/coordinate ownership is `geometry-structure`'s** — cited here for propulsion-model awareness, not re-derived | CONFIRMED — `MASS_PROPERTIES.md` §6.1 |
| Secondary battery | 3S 3300 mAh | Master dataset §43 | CONFIRMED (component identity/capacity) |
| Secondary battery mass | ≈0.248 kg | Master dataset §43 | CONFIRMED |
| Secondary battery position | — | Master dataset §43: "exact position unknown" | `PROPULSION_DATA_REQUIRED` — genuinely open; not invented (§8) |
| Battery V2 model (voltage sag under load) | `V_battery = V_oc(SOC) − I_total·R_internal` | Master dataset §43, §63 | Explicitly labeled future work (V2); battery internal resistance and SOC-OCV curve are `PROPULSION_DATA_REQUIRED` (§8) |

### 1.4 Propeller (APC 13x6.5E) and counter-rotation

| Parameter | Value | Provenance | Status |
|---|---|---|---|
| Propeller model | APC 13x6.5E | `CLAUDE.md`; `docs/source_of_truth/README.md` | CONFIRMED (component identity) |
| Propeller diameter (physical, for all physics) | 0.3302 m (13 in) | Manufacturer nominal, per `CLAUDE.md` and §0 above | CONFIRMED |
| Propeller pitch (physical) | 0.1651 m (6.5 in) | Manufacturer nominal, per `CLAUDE.md` and §0 above | CONFIRMED |
| Propeller mass | ≈0.0301 kg each (≈30.1 g), ≈0.0602 kg total (2 props) | Master dataset §44 | CONFIRMED |
| Motor/propeller rotation direction (per side) | **Left = CCW, Right = CW** | Master dataset §44 | **CONFIRMED — supersedes the prior `PROPULSION_DATA_REQUIRED` tag on rotation direction.** This is a genuine correction: the previous version of this document explicitly stated rotation direction was undocumented; it is now recorded directly by the master dataset. See §7 for the reaction-torque architecture this enables. |
| Reverse-handed propeller requirement | The physically-reversed (CW) side requires a **physically reverse-handed propeller part** — APC 13x6.5EP (reverse-left-hand rotation) — not a normal 13x6.5E spun backward electrically | Master dataset §44: "Physical counter-rotating systemde reverse-handed counterpart gerekir... Aynı handed prop sadece motor ters döndürülerek fiziksel eşdeğer sayılmamalı." | CONFIRMED as an architectural/hardware-identity requirement. **Explicit caveat, carried through unmodified:** reversing a normal (13x6.5E) propeller's electrical rotation direction alone is NOT an equivalent physical reverse propeller — the blade airfoil/twist is handed, so a true reverse-rotation installation needs the APC 13x6.5EP (or equivalent reverse-hand) part, not just a re-wired motor. |
| APC 13x6.5EP performance data (Ct/Cp, if it differs from the 13x6.5E) | — | Not in the master dataset or this repository | `PROPULSION_DATA_REQUIRED` — the E and EP variants are conventionally treated as mirror-image aerodynamic equivalents by the manufacturer product line, but this document does not assert that as confirmed; independently confirming EP performance data is open, and no Ct(J)/Cp(J) table exists for either variant yet regardless (§5, §8) |

### 1.5 Motor/propeller mount coordinates and thrust axis (geometry-structure's ownership — cited, not re-derived)

Per this agent's ownership boundary, force-application-point **coordinates** and the thrust-axis **direction** are `geometry-structure`'s output; propulsion physics consumes those points/axis as given, it does not set them. These values were first recorded in the master dataset (§46–§48, §70) and are now formally published as `CONFIRMED` by `geometry-structure` in `GEOMETRY.md` §7:

| Parameter | Value | Status here |
|---|---|---|
| Left prop hub | (0.2951, +0.3000, 0.1271) m | CONFIRMED — `GEOMETRY.md` §7; master dataset §46 |
| Right prop hub | (0.2951, −0.3000, 0.1271) m | CONFIRMED — `GEOMETRY.md` §7; master dataset §46 |
| Left motor center | (0.2623, +0.3000, 0.1269) m | CONFIRMED — `GEOMETRY.md` §7; master dataset §46 |
| Right motor center | (0.2623, −0.3000, 0.1269) m | CONFIRMED — `GEOMETRY.md` §7; master dataset §46 |
| Prop hub position relative to CG (CG = 0.168309, 0, 0.100000) | Left: ΔX≈+0.1268, ΔY≈+0.3000, ΔZ≈+0.0271 m; Right: ΔX≈+0.1268, ΔY≈−0.3000, ΔZ≈+0.0271 m | CONFIRMED (derived quantity) — `GEOMETRY.md` §7; master dataset §47 |
| Motor thrust-line orientation/axis | Measured prop-face normal (+0.999996, +0.000018, −0.002668), i.e. essentially pure +X with a ≈0.15° vertical offset. Master dataset §48: "Gazebo V1: thrust axis=+X kabul edilebilir." | CONFIRMED — `GEOMETRY.md` §7; master dataset §48 |

`GEOMETRY.md` §7 now publishes these as its own authoritative record (superseding the prior `DATA_REQUIRED`/`THRUST_AXIS_REQUIRES_CONFIRMATION` status this document previously had to cite there); the numeric values are unchanged from the master dataset. **Note, carried over from `GEOMETRY.md` §7:** the "motor center" value above is explicitly distinguished there from the raw `left_motor.stl`/`right_motor.stl` mesh bounding-box center — the two differ by ≈7.3 mm in X (both values are kept by `geometry-structure`, not silently unified); the prop-hub value, by contrast, agrees with its mesh bounding-box center to ≤0.03 mm. This document uses the master-dataset/`GEOMETRY.md` "motor center" and "prop hub" values above, not the raw mesh bounding-box centers, for any future force-application-point implementation. **Thrust must never be applied at the CG — only at the real hub/thrust-line point** (master dataset §47: "Thrust CG'ye uygulanmamalı. Gerçek hub/thrust-line noktasında uygulanmalı.").

Motor/propeller mounting angle (incidence/toe/downthrust) beyond the ≈0.15° thrust-axis offset noted above is not separately documented anywhere in the master dataset or `GEOMETRY.md`; remains `PROPULSION_DATA_REQUIRED`/geometry-owned if a distinct mounting angle (as opposed to thrust-axis direction) is later found to matter.

### 1.6 Static bench / validation reference points

**These are validation references for the eventual physics model, not a lookup table and not a substitute for the throttle→electrical→RPM→propeller-loading chain.**

| Source | Condition | RPM | Thrust | Current | Power |
|---|---|---|---|---|---|
| SunnySky X2820 860KV + APC 13x6.5 + 4S, representative bench (master dataset §49) | Static bench | ≈9230 | ≈3350 gf ≈ 32.85 N/motor | ≈63.2 A | ≈935 W/motor |
| APC official data, independent cross-check (master dataset §50) | Static, 9000 RPM | 9000 | 29.664 N | — | — |
| APC official data, independent cross-check (master dataset §50) | Static, 10000 RPM | 10000 | 36.877 N | — | — |
| APC official data, interpolated to 9230 RPM (master dataset §50) | Static, interpolated | 9230 | ≈31.32 N | — | — |

Two motors static (bench reference): ≈65.7 N ≈ 6.70 kgf combined; for the 6.000 kg aircraft this gives a static thrust-to-weight ratio ≈1.12 (master dataset §49). The bench current (≈63.2 A) is close to the 65 A/30 s rating — **this is a burst/full-static data point, not a cruise operating point**, and must not be treated as representative of sustained/cruise current draw.

The SunnySky bench figure (≈32.85 N at 9230 RPM) and the APC-official interpolation (≈31.32 N at the same RPM) differ by ≈4.7–5%. Per master dataset §50, these are **two independent datasets agreeing reasonably well** — both are retained as validation references. **The eventual implementation must not be tuned to force exact agreement with both simultaneously**; a genuine Ct(J)/Cp(J) table (§5) will not exactly reproduce either single bench number, and that is expected, not an error to "fix" by adjusting motor thrust (see `CLAUDE.md` simulation tuning policy — motor thrust is never a tuning knob for an unrelated discrepancy).

### 1.7 RPM safety limit — three distinct concepts (do not conflate)

| Concept | Value | Provenance | Note |
|---|---|---|---|
| Physical/manufacturer safety limit (APC "Thin Electric" rule) | RPM_max ≈ 150000 / D[in] ≈ 11538 RPM for a 13 in prop | Master dataset §51 | This is a propeller structural/manufacturer guideline, not a motor limit |
| Gazebo V1 hard-cap / numerical clamp candidate | ≈11500 RPM | Master dataset §51 | An implementation-level clamp derived from (slightly below) the physical limit above — a numerical safety measure, not itself a new physical fact |
| Theoretical no-load RPM (KV × V, unloaded) | 860×14.8 ≈ 12728 RPM (Vnom); 860×16.8 ≈ 14448 RPM (Vfull) | Master dataset §51, §52 | **Explicitly NOT an allowable operating point** — it exceeds the physical safety limit above. Loaded RPM (under real propeller torque) is always lower than this no-load figure (§4) |

These three concepts must be kept distinct in any future implementation: the no-load figure is a theoretical upper bound on unloaded motor speed (never actually reached once a real propeller load is applied), the physical safety limit is a hardware constraint independent of the electrical model, and the numerical clamp is an implementation safeguard that should sit at or below the physical limit.

---

## 2. Target propulsion model chain

This is the architecture the eventual implementation must follow. No implementation is performed in this document.

```
throttle → electrical/motor response → motor RPM → propeller aerodynamic loading → thrust, torque
```

**Full expanded chain (master dataset §1 introduction — this is the authoritative, non-negotiable architecture for FALCON V2's propulsion, restated here in full because it is the frame every section below fits into):**

```
throttle
→ ESC duty-cycle / effective applied voltage
→ motor electrical model
→ motor current
→ motor electromagnetic torque
→ motor + propeller angular dynamics
→ RPM(t)
→ propeller advance ratio J
→ APC Ct(J), Cp(J)
→ thrust + aerodynamic propeller torque
→ reaction torque
→ force/moment applied at the real propeller hub location
→ Gazebo 6-DOF dynamics
```

Expanded SI-unit equation form (master dataset §52–§56, §70 "core equations" block — physics relationships, not code; **no implementation performed in this document**):

```
Kt   = 60 / (2π × KV)                 (motor torque constant, N·m/A; KV in rpm/V)
Ke   ≈ Kt (numerically, SI units)     (back-EMF constant, V/(rad/s))
V_ESC = throttle × V_battery          (V1 ESC model, §1.2)
I    = (V_ESC − Ke × ω) / R           (motor current; ω = motor angular rate, rad/s; R = motor internal resistance)
Q_motor = Kt × (I − I0_effective)     (motor electromagnetic torque)
I_rotor × dω/dt = Q_motor − Q_prop    (angular dynamics — RPM is a solved state, never set instantaneously)

n    = RPM / 60                       (revolutions per second; RPM derived from the ω state above)
J    = V_axial / (n × D)              (advance ratio; V_axial = local axial airflow at that motor/prop disk)
T    = Ct(J) × ρ × n² × D⁴            (thrust)
P_prop = Cp(J) × ρ × n³ × D⁵          (propeller shaft power)
ω    = 2π × n
Q_prop = P_prop / ω                   (propeller aerodynamic torque — the load term in the angular-dynamics ODE above)
Q_reaction = −Q_prop                  (reaction torque applied to the airframe, §7)
```

Where, for every occurrence in this project:
- **D = 0.3302 m always** (APC 13x6.5E nominal diameter — never the ≈273 mm STL-mesh figure; see §0).
- ρ = local air density (standard atmosphere model or environment-provided value — not defined in this document; source `DATA_REQUIRED` if a non-ISA atmosphere is needed).
- Ct(J), Cp(J) = propeller thrust/power coefficients as functions of advance ratio — see §5 for how these should be sourced; **no numeric Ct/Cp values are given anywhere in this document.** (Note: an equivalent torque-coefficient form `Q = Cq(J) × ρ × n² × D⁵` is algebraically identical via `Cq = Cp / (2π)`; the master dataset's own equations use the power-coefficient `Cp` form shown above, so this document now follows that convention rather than a separately-invented `Cq` curve — no new physical assumption is introduced by this notational alignment.)
- **RPM must never be set instantaneously.** It is a state variable solved through the `I_rotor·dω/dt = Q_motor − Q_prop` ODE (master dataset §53) — this is the mechanism that produces realistic spin-up/spin-down and airspeed-dependent RPM sag, and it is what prevents this chain from ever collapsing into a static throttle→RPM lookup.
- `I_rotor` (combined motor-rotor + propeller rotational inertia) is **not known exactly** — master dataset §53 explicitly states this needs a V1 estimate or bench spin-up calibration. This document does not invent a fixed numeric value for it (§4, §8).

**This chain must never be collapsed into `throttle × maximum_thrust`.** Per `CLAUDE.md` and the `propulsion` agent definition, that simplification is permitted only as a temporary diagnostic, only with explicit authorization, and must be labeled `TEMPORARY_TEST_MODEL` everywhere it appears (code, config, documentation). No such authorization has been given as of this document, and no such model exists anywhere in this repository. Master dataset §72 restates the same rule independently: "throttle sabit thrust lookup ana model olarak kullanılmamalı."

Each of the four top-level stages (throttle→electrical response, electrical response→RPM, RPM→propeller operating point, operating point→thrust/torque) is expanded further in §3–§5 below.

---

## 3. Existing-data inventory — found vs. missing

Searched for in `CLAUDE.md`, `docs/source_of_truth/`, `docs/architecture/`, `.claude/agents/`, `model/`, `tests/`, the master dataset (`docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt`, §41–§63/§70–§72), and full git history.

| Data item | Found in repository? | Value / detail | Status |
|---|---|---|---|
| Motor KV | Yes | 860 rpm/V (SunnySky X2820 860KV) | CONFIRMED — manufacturer nameplate spec, per `CLAUDE.md`; master dataset §41 |
| Motor count/model | Yes | 2 × SunnySky X2820 860KV | CONFIRMED |
| Propeller model | Yes | APC 13x6.5E (+ reverse-handed 13x6.5EP counterpart required for the reversed side, §1.4) | CONFIRMED |
| Propeller diameter/pitch (nominal) | Yes | D = 0.3302 m, pitch = 0.1651 m | CONFIRMED — manufacturer nominal figures for the 13x6.5E part number |
| Electrical architecture | Yes | 4S | CONFIRMED |
| Battery capacity | Yes | 4S 22000 mAh, 25C | CONFIRMED — master dataset §43 (§1.3) |
| **Motor internal resistance (R)** | **Yes (this pass)** | **≈0.0258 Ω** | **CONFIRMED — master dataset §41. Supersedes the prior `PROPULSION_DATA_REQUIRED` status; this was a stale gap, now closed by manufacturer/web-collected component data (§1.1).** |
| **Motor no-load current (I0)** | **Yes (this pass)** | **≈1.3 A @ 10 V** | **CONFIRMED — master dataset §41 (§1.1). Same correction as above.** |
| Motor max current / max power / mass | Yes (this pass) | ≈65 A/30 s, ≈960 W/motor, ≈0.143 kg/motor | CONFIRMED — master dataset §41 (§1.1) |
| No-load RPM (theoretical, KV×V) | Yes, as a labeled theoretical figure — not a measured operating point | 860×14.8≈12728 RPM; 860×16.8≈14448 RPM | Master dataset §51/§52. Remains a *derived* theoretical bound, **explicitly not an allowable operating point** (§1.7) — this framing is unchanged from before, only now cross-referenced against an explicit physical safety limit |
| Battery nominal/full-charge voltage | Yes (this pass) | Vnom=14.8 V, Vfull=16.8 V | CONFIRMED as this specific pack's recorded spec — master dataset §43. (Previously this document could only cite the generic 4S LiPo chemistry convention as a `DERIVED` stand-in; the master dataset now records it directly as pack data, closing that gap — §1.3.) |
| Motor winding/pole count, mechanical time constant | No | — | `PROPULSION_DATA_REQUIRED` (unchanged — not in the master dataset) |
| **ESC identity, mass, V1 electrical model** | **Yes (this pass)** | **2× Hobbywing Skywalker 80A, ≈0.080 kg each, `V_ESC = throttle × V_battery`** | **CONFIRMED — master dataset §42 (§1.2). ESC dynamic-response characteristics (PWM ramp, delay, efficiency, current limiting) remain `PROPULSION_DATA_REQUIRED`, explicitly labeled V2/future work by the master dataset — not conflated with the now-resolved identity/V1-model items.** |
| ESC exact position (per side) | No | — | `PROPULSION_DATA_REQUIRED` — genuinely unknown; master dataset §42 states this explicitly ("Kesin konumlar bilinmiyor") (§1.2) |
| APC 13x6.5E Ct(J)/Cp(J) coefficient data or equivalent performance tables | No | — | `APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION` — distinct from "architecture unknown": the propeller-loading architecture (T=Ct·ρ·n²·D⁴, P=Cp·ρ·n³·D⁵, Q=P/ω) **is** fully defined (§2, §5, master dataset §54–§56); only the numeric coefficient table itself is missing |
| Static/dynamic bench validation reference points | Yes (this pass) | SunnySky bench (9230 RPM, 32.85 N/motor, 63.2 A, 935 W/motor) + APC-official cross-check (9000/10000 RPM points, ≈4.7–5% delta at interpolated 9230 RPM) | CONFIRMED as **validation reference points**, master dataset §49–§50 (§1.6) — **not** a full Ct(J)/Cp(J) table and not a substitute for one; the `APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION` gap above is not closed by these two static points |
| Motor/propeller combined dyno or bench data across a throttle/airspeed sweep | Partial — first-estimate validation table only, not measured bench data | Master dataset §61 throttle/RPM/thrust-vs-airspeed table (§6 below) | Labeled `V1_VALIDATION_ESTIMATES` by this document, matching the master dataset's own "Bu değerler sabit lookup yapılmayacak; fizik modelinden beklenen denge noktalarıdır" framing — expected model outputs, not measured dyno data, and explicitly not a lookup table |
| Battery internal resistance / discharge curve / sag characteristics | No | — | `PROPULSION_DATA_REQUIRED` (unchanged) — master dataset §43/§63 explicitly defers this to V2 |
| Motor/propeller mount coordinates (per side) | Yes | (0.2951, ±0.3000, 0.1271) m per side (hub); (0.2623, ±0.3000, 0.1269) m per side (motor center) | CONFIRMED — `GEOMETRY.md` §7 (geometry-structure's ownership); master dataset §46, §70 (§1.5) |
| Motor thrust-axis orientation | Yes | ≈+X (measured normal ≈0.999996, ≈0.15° vertical offset) | CONFIRMED — `GEOMETRY.md` §7; master dataset §48, §70 (§1.5) |
| **Motor/propeller rotation direction (CW/CCW, per side)** | **Yes (this pass)** | **Left = CCW, Right = CW** | **CONFIRMED — master dataset §44 (§1.4). This is the primary stale claim corrected in this pass: rotation direction is no longer undocumented.** |
| Motor/propeller mounting angle (incidence/toe/downthrust), beyond the ≈0.15° thrust-axis offset | No | — | `PROPULSION_DATA_REQUIRED` (unchanged; not separately given in the master dataset) |
| Motor/propeller rotational inertia (I_rotor) | No — only a V1-estimate/bench-calibration *path*, not a number | — | `PROPULSION_DATA_REQUIRED` (unchanged, explicitly per master dataset §53 — "Exact rotational inertia henüz bilinmiyor") |
| Motor efficiency map / thermal model / friction-loss model | No | — | `PROPULSION_DATA_REQUIRED` (unchanged; master dataset §71 lists these as explicit V2 items) |

### What level of propulsion model can actually be built now (updated this pass)

**Now architecturally and partially numerically specified (with the master dataset):**
- Component identity, top-level architecture, and the full SI motor/propeller equation set (§2) — this is a substantial change from the prior "architecture only, no equations" state.
- The motor electrical model is now numerically parameterizable: KV, R, I0, Kt are all known (§1.1, §52). Given an assumed/calibrated `I_rotor` (still open) and a real Ct(J)/Cp(J) table (still open), the full chain in §2 could be implemented.
- Static-thrust validation references now exist at one operating point from two independent sources (§1.6).
- Motor/prop hub and mount coordinates and the thrust-axis vector are now `CONFIRMED` by `geometry-structure` in `GEOMETRY.md` §7, and the main battery center/mass are now `CONFIRMED` by `geometry-structure` in `MASS_PROPERTIES.md` §6.1 — the force-application-point geometry that this document previously had to wait on is no longer a blocker (§1.5).

**Still currently blocked (genuinely open, not filled with guesses):**
- Any numeric RPM→thrust/torque relationship across the full advance-ratio range (needs the real APC Ct(J)/Cp(J) table — `APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION`, §5).
- Full airspeed-dependent thrust variation (same blocker).
- Spin-up/spin-down dynamic response (needs `I_rotor`, §1.1/§4).
- ESC exact positions, secondary-battery position, battery internal resistance/SOC curve, motor efficiency/thermal/friction model (all explicitly deferred to V2 by the master dataset itself, §8).

No numeric Ct/Cp curve is written anywhere in this document to fill the remaining gap.

---

## 4. Throttle → RPM model architecture

Target structure (architectural definition; no code, no numeric implementation):

```
throttle command → effective motor voltage → target (no-load) RPM
                                                    │
                                                    ▼
                                    motor first-order electromechanical response
                                       (time constant, torque-balance dynamics)
                                                    │
                                                    ▼
                                              actual RPM (state variable)
```

Key architectural points:

1. **`RPM_no_load ≈ KV × V_effective` is a no-load approximation only.** It describes the RPM the motor would settle at with zero propeller load. It must **not** be used directly as the loaded operating RPM in this project — propeller aerodynamic load (torque `Q_prop`, from §5) reduces actual RPM below this no-load figure. **Update, this pass: the amount of reduction is now numerically computable**, since motor R and I0 are `CONFIRMED` (§1.1, master dataset §41): `I = (V_ESC − Ke·ω)/R`, `Q_motor = Kt·(I − I0_effective)` (master dataset §52). What remains open is the propeller's own torque-vs-RPM-and-airspeed relationship (`APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION`, §5) and the rotational inertia that sets how fast RPM settles into that balance (`I_rotor`, point 3 below).

2. **Actual RPM must be modeled as a state variable that settles from a torque balance**, not as a direct static function of voltage: `I_rotor·dω/dt = Q_motor − Q_prop` (master dataset §53). Motor electromagnetic torque is now a fully specified function of current, voltage, back-EMF, and RPM (R, I0, Kt all known, §1.1/§52); the propeller absorbs torque `Q_prop(RPM, V_local)` per §5; RPM evolves until motor output torque and propeller load torque balance. This is the mechanism by which airspeed, throttle, and instantaneous RPM interact — it is not a lookup table indexed by throttle alone, and **RPM must never be set instantaneously** (master dataset §60, §72).

3. **The rotational-inertia term `I_rotor` (combined motor-rotor + propeller inertia) governs the dynamic lag in the ODE above** and is the one genuinely open parameter blocking this section's full numeric implementation. Master dataset §53 is explicit: "Exact rotational inertia henüz bilinmiyor. V1'de estimate veya bench spin-up kalibrasyonu gerekir." This document does not supply a number for it — only the calibration *path* (estimate, or bench spin-up-time measurement) is documented, consistent with `CLAUDE.md`'s rule against inventing plausible-looking constants.

4. **Effective motor voltage** should account for battery voltage (which itself sags under load and depletes with state of charge — battery internal resistance/SOC-curve behavior is `PROPULSION_DATA_REQUIRED`, explicitly deferred to V2 by master dataset §43/§63) and the ESC's V1 throttle-to-voltage mapping, now `CONFIRMED` as `V_ESC = throttle × V_battery` (§1.2, master dataset §42) — ESC *dynamic* response characteristics (ramp rate, delay, efficiency, current limiting) remain V2/`PROPULSION_DATA_REQUIRED`. Note: **the throttle command itself, and how it is sourced/actuated from the autopilot, is `controls-integration`'s domain** — this document only concerns what happens once a throttle command reaches the motor/electrical model (per this agent's ownership boundary in `CLAUDE.md`).

### Parameters requiring calibration before this architecture is numerically implementable

| Parameter | Needed for | Status |
|---|---|---|
| ~~Motor Rm (internal resistance)~~ | Current draw, output torque, loaded-RPM torque balance | **Resolved this pass: ≈0.0258 Ω, CONFIRMED — master dataset §41 (§1.1)** |
| ~~Motor I0 (no-load current)~~ | Torque balance, efficiency at low load | **Resolved this pass: ≈1.3 A @ 10 V, CONFIRMED — master dataset §41 (§1.1)** |
| Motor/propeller-assembly rotational inertia (I_rotor) | Spin-up/spin-down dynamics, angular-dynamics ODE time constant | `PROPULSION_DATA_REQUIRED` — genuinely open; master dataset §53 provides only a V1-estimate/bench-calibration path, no number |
| Motor electrical/mechanical time constant (if modeled as a lumped first-order lag, as an alternative/addition to the explicit ODE above) | Throttle-step response shape | `PROPULSION_DATA_REQUIRED` |
| ESC dynamic response characteristics (ramp rate, PWM-to-voltage mapping beyond the V1 linear model, current limiting) | Effective voltage delivered to motor as a function of throttle, transient shaping | `PROPULSION_DATA_REQUIRED` — explicitly V2 per master dataset §42 |
| Battery voltage-under-load / discharge behavior | Effective voltage available at a given throttle/current/state-of-charge | `PROPULSION_DATA_REQUIRED` — explicitly V2 per master dataset §43/§63; V1 uses fixed Vnom=14.8 V (§1.3) |
| APC 13x6.5E Ct(J)/Cp(J) coefficient table | Propeller load torque `Q_prop(RPM, V_local)` in the ODE above | `APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION` (§5) |

---

## 5. RPM → thrust model — data-source options for Ct(J)/Cp(J)

The propeller physics must always use **D = 0.3302 m** (§0). This section evaluates *how* the eventual implementation should source the coefficient functions Ct(J) and Cp(J) — no numeric coefficients are given here. (Notation update, this pass: the master dataset's own equations (§54–§56) express the torque-side coefficient as a power coefficient `Cp(J)`, with `Q_prop = P_prop/ω` — algebraically equivalent to a direct torque coefficient `Cq(J) = Cp(J)/(2π)`. This document now follows the master dataset's `Cp` convention for consistency; nothing about the underlying missing-data status changes.)

**Distinct from "propulsion architecture unknown":** the architecture for this stage — `T = Ct(J)·ρ·n²·D⁴`, `P_prop = Cp(J)·ρ·n³·D⁵`, `Q_prop = P_prop/ω` — is **fully defined** (§2, master dataset §54–§56). Only the real numeric Ct(J)/Cp(J) table for the APC 13x6.5E is missing. This document tags that specific, narrower gap `APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION` to keep it distinct from any claim that the model structure itself is undefined.

The static bench/APC-official validation reference points recorded in §1.6 (master dataset §49–§50) are single operating-point data — they do not by themselves constitute a Ct(J)/Cp(J) table across the advance-ratio range the aircraft will encounter, and do not close this gap on their own; they are retained as calibration/sanity-check references for whichever option below eventually supplies the full table.

### Option A — Real APC published performance data / coefficient tables for the 13x6.5E prop
If obtainable (e.g. from APC's published performance data, which many APC propellers have — static and dynamic RPM/thrust/torque/power tables at various RPM and airspeed points, sometimes with derived Ct/Cq or power/thrust coefficient curves), this captures the actual blade geometry, airfoil section, and twist distribution of the real product. This is manufacturer test data, directly traceable, and requires no separate validation of the underlying physical assumptions (only of the transcription/digitization into this project's data files).

### Option B — Experimental thrust-RPM test data (project-owner test stand measurement)
If the project owner has or can obtain a bench/test-stand measurement (thrust and torque vs. RPM, ideally at more than one airspeed or at least static + one forward-flight-representative condition), this is measured test data specific to the exact hardware combination (this motor + this ESC + this propeller + this battery), which can differ from generic manufacturer curves due to manufacturing tolerance, mounting, and installation effects. This is the most directly traceable option for *this specific aircraft's* hardware.

### Option C — Physical Ct/Cp approximation (blade-element theory or simplified propeller theory) + calibration against whatever limited data is available
A blade-element-momentum (BEMT) or simplified propeller-theory model can generate a physically-motivated Ct(J)/Cp(J) shape without requiring a manufacturer table or test stand, but it requires blade geometry inputs (chord/twist distribution, airfoil sections along the span) that are not currently in this repository, and any simplified/analytical approximation not built from the real blade geometry risks being an "arbitrary thrust curve" — explicitly disallowed by `CLAUDE.md` and the `propulsion` agent's rules unless clearly labeled and calibrated against real data. If used, the static bench/APC-official reference points in §1.6 are the natural calibration anchor.

### Recommendation

**Preference order: A > B > C**, with the following reasoning:

- **A (real APC data) is preferred first** because it is manufacturer-sourced, directly traceable, and reflects the actual blade geometry and airfoil performance of the specific product (APC 13x6.5E) used on this aircraft — no physical modeling assumptions are introduced by this project.
- **B (test-stand data) is preferred second**, and in practice may be *more* representative than A for this specific airframe, since it captures the actual installed motor+ESC+propeller+battery combination rather than an idealized/generic manufacturer curve. It is ranked after A only because A is typically easier to obtain without new hardware testing and covers a broader operating envelope (multiple RPM/airspeed points) than a single test stand session might; if B were available with equivalent coverage (multiple airspeeds/advance ratios, not just static thrust), it should be weighted equal to or above A. **A and B are not mutually exclusive — B should also be used, where available, to validate/calibrate A**, and either can independently satisfy the "traceable to manufacturer/test data" requirement in `CLAUDE.md`.
- **C (physical approximation) is the fallback** when neither A nor B is available. It is acceptable *only* if clearly labeled as an approximation (not manufacturer/test data), and only if calibrated against whatever real data becomes available later (even a single static-thrust data point is better than an uncalibrated theoretical curve). An uncalibrated BEMT/simplified-propeller-theory curve, presented as if it were the aircraft's real performance, would violate the "no arbitrary thrust curves" rule in `CLAUDE.md` and the `propulsion` agent definition.

**Current status: none of A, B, or C (the full coefficient table) exist in this repository yet.** Ct(J)/Cp(J), under any sourcing option, is tagged `APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION` (§3) — distinct from the model architecture, which is fully defined (§2). The single-operating-point static bench/APC-official validation references now exist (§1.6, master dataset §49–§50) and should be used to calibrate/sanity-check whichever of A/B/C eventually supplies the table, but they are not themselves the table. No numeric coefficient curve is fabricated in this document to fill the gap.

---

## 6. Airspeed-dependent thrust — per-motor computation

Because FALCON V2 is a twin-engine aircraft, **each motor's thrust and torque must be computed independently**, not lumped into a single combined value. This matters specifically because asymmetric conditions — one engine out, sideslip, yaw, differential throttle — require the two motors to see different local operating conditions and produce different force/torque outputs.

Target per-motor computation, applied separately to the left motor and the right motor:

```
for each motor m in {left, right}:
    V_local(m)   = local airspeed at motor m's propeller disk
                   (freestream airspeed combined with aircraft body rates / sideslip —
                    each motor can see a different local flow, e.g. under sideslip or yaw,
                    since they are laterally offset from the CG)
    RPM(m)       = actual RPM of motor m (from §4's torque-balance state, independent per motor —
                    e.g. under engine-out, one motor's RPM can be zero/free-spinning while the
                    other operates normally)
    n(m)         = RPM(m) / 60
    J(m)         = V_local(m) / (n(m) × D)          [D = 0.3302 m, always — §0]
    T(m)         = Ct(J(m)) × ρ × n(m)² × D⁴
    P_prop(m)    = Cp(J(m)) × ρ × n(m)³ × D⁵
    Q_prop(m)    = P_prop(m) / (2π × n(m))
    apply T(m), Q_prop(m)-derived reaction torque at motor m's force-application point
    (coordinates owned by geometry-structure; CONFIRMED in GEOMETRY.md §7, §1.5)
```

`V_local` should be evaluated at (or as close as practical to) each motor's actual location, not the aircraft's single freestream/CG airspeed — this is what allows the model to represent, e.g., a sideslip condition where one propeller sees a different effective inflow than the other, or an engine-out condition where the windmilling/stopped propeller on one side no longer produces the same thrust as the operating one on the other side. The precise method for computing per-motor local airspeed (freestream + rotational-rate correction at each motor's offset from CG) is an implementation-phase decision, not resolved numerically in this document, and now has the motor mount coordinates it needs `CONFIRMED` by `geometry-structure` (`GEOMETRY.md` §7, §1.5).

If the underlying Ct(J)/Cp(J) data (§5, `APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION`) does not cover the full range of J the aircraft can encounter (e.g. very low J at high RPM/low airspeed near static thrust, or high J near windmilling/engine-out), that gap must be reported for the affected J range rather than extrapolated with an assumed static-thrust-like curve.

### 6.1 Propulsion validation estimates — `V1_VALIDATION_ESTIMATES`, not a lookup table

Master dataset §61 explicitly frames the following as **expected physics-model equilibrium points, not a fixed lookup table**: "Bu değerler sabit lookup yapılmayacak; fizik modelinden beklenen denge noktalarıdır." This document labels the whole set `V1_VALIDATION_ESTIMATES` and **explicitly forbids implementing "throttle → precomputed thrust" as the primary propulsion model** using these numbers — they exist to check that an eventual torque-balance implementation lands in the right neighborhood, not to be interpolated as the model itself.

Trimmed-flight throttle/RPM/thrust vs. airspeed (`V1_VALIDATION_ESTIMATES`, master dataset §61):

| Airspeed | Throttle | RPM | Thrust/motor | Total thrust (2 motors) |
|---|---|---|---|---|
| 12.5 m/s | ≈43.2% | ≈4800 | ≈2.88 N | ≈5.75 N |
| 15 m/s | ≈46.9% | ≈5262 | ≈2.56 N | ≈5.12 N |
| 18 m/s | ≈53.6% | ≈6021 | ≈2.59 N | ≈5.19 N |
| 20 m/s | ≈58.2% | ≈6533 | ≈2.77 N | ≈5.54 N |
| 22 m/s | ≈63.8% | ≈7125 | ≈3.03 N | ≈6.06 N |
| 25 m/s | ≈72.4% | ≈8022 | ≈3.56 N | ≈7.12 N |

Fixed-14.8 V throttle sweep, first estimates (`V1_VALIDATION_ESTIMATES`, master dataset §61):

| Throttle | Estimated airspeed | Estimated RPM |
|---|---|---|
| 45% | ≈13.7 m/s | ≈5030 |
| 50% | ≈16.6 m/s | ≈5624 |
| 55% | ≈18.7 m/s | ≈6179 |
| 60% | ≈20.7 m/s | ≈6724 |
| 65% | ≈22.5 m/s | ≈7257 |
| 70% | ≈24.3 m/s | ≈7778 |
| 75% | ≈26.0 m/s | ≈8296 |
| 80% | ≈27.6 m/s | ≈8803 |
| 90% | ≈30.9 m/s | ≈9801 |
| 100% | theoretical ≈34 m/s | ≈10780 |

Master dataset §61 flags 80%+ throttle as **lower-confidence**, since battery sag, ESC/motor efficiency, propeller unloading, drag, and prop-wing interference all become more significant there and are not fully modeled at V1. This caveat is preserved unchanged.

### 6.2 Cruise electrical-consumption first estimate

Master dataset §62 provides a **first estimate, not a validated figure**: at 18 m/s / ≈6020 RPM, APC shaft power ≈78 W/motor; with a first motor-efficiency estimate ≈0.73, electrical input ≈107 W/motor (≈214 W total); at 14.8 V this is ≈14.4 A total current draw. Against the 22 Ah main pack, this gives a theoretical propulsion-only endurance of ≈1.53 h, or ≈1.22 h at 80% usable capacity. Master dataset §62 itself notes real flight endurance may be lower. This is carried through as a first estimate only — it is not a measured or validated flight-test figure, and no attempt is made here to sharpen it.

### 6.3 Visual RPM must equal physics RPM

Master dataset §60/§72: there must be a **single** `omega_left`/`omega_right` state per motor, and that same state must drive (a) the visual propeller joint rotation, (b) thrust, (c) propeller shaft power, (d) propeller aerodynamic torque, and (e) reaction torque. No separate, arbitrary "visual RPM" state may be introduced — this would risk the visual/animation RPM silently diverging from the physics RPM actually producing forces/torques.

---

## 7. Propeller reaction torque and rotation direction

**Update, this pass: rotation direction is now `CONFIRMED` — Left = CCW, Right = CW (master dataset §44, §1.4).** This corrects the prior version of this document, which stated rotation direction was undocumented; that was a stale gap, not an assumption this document is now making. **Mounting angle (incidence/toe/downthrust) beyond the ≈0.15° thrust-axis offset noted in §1.5/§1.7 remains genuinely undocumented** and is not assumed here.

### Whether the eventual model applies propeller shaft reaction torque to the airframe

Yes — master dataset §57/§72 state this explicitly and non-negotiably: "reaction torque mutlaka uygulanmalı" (reaction torque must always be applied). Propeller shaft reaction torque, `Q_reaction(m) = −Q_prop(m)` (§2, §6), is a real physical effect applied to the airframe at each motor's mount point (once `geometry-structure` publishes those coordinates, §1.5), consistent with the "no duplicated forces" and "force applied at wrong point" concerns in `CLAUDE.md`'s tuning policy. This is qualitatively different from thrust (a force along the thrust axis) — reaction torque is a moment about the same axis.

### Counter-rotation architecture (now confirmed, not just qualitatively discussed)

Because the motors counter-rotate (left CCW, right CW — §1.4), **the two reaction torques largely cancel at the airframe level under symmetric throttle** (equal RPM/torque on both sides), per master dataset §57: "Sol/sağ ters döndüğü için eşit RPM'de büyük ölçüde birbirini götürür." This is not necessarily instantaneous per motor during transients — unequal RPM/throttle (asymmetric throttle input, one-engine-out) or differing local airspeed per §6 causing differing instantaneous torque even at equal throttle command will produce a **naturally-arising residual moment**, which is the correct physical behavior, not an error to suppress (master dataset §57: "Unequal RPM/throttle veya motor arızasında residual moment doğal oluşur").

The same-direction (non-counter-rotating) case discussed in the prior version of this document is now moot — it does not apply to FALCON V2's actual hardware and is removed from this document rather than kept as a live alternative, since rotation direction is no longer an open question.

### 7.1 Differential thrust and yaw — no fake control derivative

Master dataset §58: the two motors sit at ≈±0.300 m lateral offset from the CG (§1.5). The resulting yaw moment from differential thrust, `Mz ≈ 0.300 × (T_right − T_left)`, **must not be coded as a separate, artificial yaw-control derivative.** Per `CLAUDE.md`'s "no duplicated forces" concern and the master dataset's own instruction ("Bunu ayrıca yapay control derivative olarak kodlamaya gerek yok. Gerçek force doğru hub noktasında uygulanırsa Gazebo r×F ile üretir."): if thrust `T(m)` is applied as a real force at the real hub position for each motor, Gazebo's own `r × F` rigid-body dynamics will produce the correct yaw moment naturally. This is documented here as the **intended architecture** — nothing is implemented in this document, and no separate differential-thrust yaw coefficient is to be introduced anywhere downstream.

### 7.2 Thrust-line pitch moment (future unit-test item)

Master dataset §59: the propeller hubs sit ≈27.1 mm above the CG (per the CG-relative hub position in §1.5). At 18 m/s (≈2.59 N/motor per §6.1), this gives a combined pitch-moment magnitude ≈2 × 0.0271 m × 2.59 N ≈ 0.14 N·m. **The sign of this moment must be verified against Gazebo's FLU convention** — master dataset §59: "İşaret Gazebo FLU convention ile doğrulanmalı." This document does not assert a sign here; it is recorded as a future unit-test item (per master dataset §72's general instruction that force/moment signs be verified with short unit tests), consistent with `gazebo-testing`/`validation`'s later role, not resolved by this document.

---

## 8. Consolidated open-items list (updated this pass)

### 8.1 Resolved this pass (no longer open — kept here only to show what changed)

- ~~Motor internal resistance (Rm)~~ — CONFIRMED ≈0.0258 Ω, master dataset §41 (§1.1).
- ~~Motor no-load current (I0)~~ — CONFIRMED ≈1.3 A @ 10 V, master dataset §41 (§1.1).
- ~~Motor mass, max current, max power~~ — CONFIRMED, master dataset §41 (§1.1).
- ~~ESC identity, mass, V1 electrical model~~ — CONFIRMED (Hobbywing Skywalker 80A, `V_ESC=throttle×V_battery`), master dataset §42 (§1.2).
- ~~Battery Vnom/Vfull as pack-specific spec~~ — CONFIRMED for this specific 4S 22000 mAh pack, master dataset §43 (§1.3) — no longer only a generic chemistry convention.
- ~~Motor/propeller rotation direction, per side~~ — CONFIRMED Left=CCW, Right=CW, master dataset §44 (§1.4). **This was the primary stale claim in the prior version of this document and is the main correction made in this pass.**
- ~~Static-thrust validation reference~~ — CONFIRMED, two independent single-point sources (SunnySky bench + APC official), master dataset §49–§50 (§1.6) — note this resolves "no static reference point exists," not the full Ct/Cp table (§5 remains open, see below).
- ~~Motor mount coordinates (left/right hub, left/right motor center) and thrust-axis direction~~ — CONFIRMED, `GEOMETRY.md` §7 (published by `geometry-structure`'s own follow-up sync pass), master dataset §46–§48, §70 (§1.5).
- ~~Main battery center coordinate and CG-relative offset~~ — CONFIRMED, `MASS_PROPERTIES.md` §6.1 (published by `geometry-structure`'s own follow-up sync pass), master dataset §43, §7 (§1.3).

### 8.2 Genuinely still open (unchanged in kind, not conflated with the resolved items above)

1. **`APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION`** — the real APC 13x6.5E Ct(J)/Cp(J) coefficient table (or equivalent performance data across the advance-ratio range) — §5. Distinct from "architecture unknown": the architecture is fully defined (§2).
2. Motor/propeller-assembly rotational inertia (I_rotor) — only a V1-estimate/bench-calibration path exists, no number, master dataset §53 — §1.1, §4.
3. Motor electrical/mechanical time constant, if modeled as a lumped first-order lag — §4.
4. ESC dynamic response characteristics (ramp rate, PWM-to-voltage mapping beyond the V1 linear model, current limiting) — explicitly V2 per master dataset §42 — §1.2, §4.
5. Exact ESC positions (per side) — explicitly unknown per master dataset §42 ("Kesin konumlar bilinmiyor") — §1.2.
6. Exact secondary (3S 3300 mAh) battery position — explicitly unknown per master dataset §43 — §1.3.
7. Battery internal resistance / SOC-OCV curve / temperature effects — explicitly V2 per master dataset §43, §63 — §1.3.
8. Motor efficiency map, no-load/friction loss model, thermal model — explicitly V2 per master dataset §71 — §1.1, §6.2.
9. APC 13x6.5EP (reverse-handed) propeller's own performance data, if it differs from the 13x6.5E — §1.4.
10. Motor/propeller mounting angle (incidence/toe/downthrust) beyond the ≈0.15° thrust-axis offset — §1.5, §7.
11. Exact avionics/servo masses and positions — not this agent's section, but referenced by cross-section CG/mass work; flagged per master dataset §71 in case a future PROPULSION.md revision needs it.

### 8.3 (Closed) — previously "pending another agent's action"

This category previously tracked items whose numeric values were already known from the master dataset but had not yet been republished as authoritative by the owning agent. As of `geometry-structure`'s own follow-up sync pass, both items that were ever recorded here (motor/prop mount coordinates + thrust axis, and the main battery center coordinate) are `CONFIRMED` in `GEOMETRY.md` §7 and `MASS_PROPERTIES.md` §6.1 respectively, and are now listed in §8.1 instead. This subsection heading is kept, empty, as a record that the category existed and was subsequently closed — not deleted outright, so the document's own history stays visible.

None of the items above are filled with a guessed, estimated, or "plausible-looking" value anywhere in this document; every value that *is* now recorded (§8.1) carries a section citation into the master dataset (and, where applicable, into `GEOMETRY.md`/`MASS_PROPERTIES.md`), and every value still open (§8.2) keeps whatever status the master dataset itself gives it.

---

## 9. Provenance table

| Source | What it provides | Used for |
|---|---|---|
| `CLAUDE.md` (project owner, setup conversation, 2026-08-21) | Motor model/KV, propeller model/nominal diameter/pitch, electrical architecture (4S), target model chain, engineering rules (no arbitrary curves, `TEMPORARY_TEST_MODEL` labeling requirement, tuning policy) | §0, §1, §2, §3 |
| `docs/source_of_truth/master/FALCON_V2_MASTER_DATASET_BEFORE_GAZEBO.txt` §41–§63, §70–§72 (project owner-authored master dataset, added to the repository 2026-08-21) | Motor electrical parameters (KV/R/I0/mass/max current/max power, §41), ESC identity/mass/V1 model (§42), battery details incl. Vnom/Vfull/mass/center and secondary battery (§43), propeller mass and counter-rotation direction + reverse-handed-prop requirement (§44), STL-vs-real diameter restatement (§45), motor/prop STL placement and thrust axis (§46–§48), static bench + APC-official validation references (§49–§50), RPM safety limit concepts (§51), motor SI equations (§52), angular dynamics ODE (§53), advance ratio/thrust/power equations (§54–§56), reaction torque (§57), differential thrust architecture (§58), thrust-line pitch moment (§59), visual-RPM=physics-RPM rule (§60), V1 validation-estimate tables (§61), cruise consumption first estimate (§62), battery V2 path (§63), consolidated V1 parameter block (§70), explicit not-yet-final/V2 list (§71), critical implementation warnings (§72) | §0, §1, §2, §3, §4, §5, §6, §7, §8 (essentially the entire document as revised in this pass) |
| `docs/source_of_truth/geometry/GEOMETRY.md` §7, §9, §19–§21.2 (authored by `geometry-structure`) | Twin front-puller configuration (qualitative), motor-forward-of-CG (qualitative), battery-central/ESC-in-wings (qualitative), motor/propeller mount coordinates (now `CONFIRMED` in §7, following `geometry-structure`'s own follow-up sync pass), thrust-axis vector (now `CONFIRMED` in §7, superseding the earlier `THRUST_AXIS_REQUIRES_CONFIRMATION` mesh-shape-only status), `VISUAL_MESH_ONLY` tag and ≈273 mm mesh-diameter finding, ≈−17.2% mesh-vs-nominal geometric-check finding, motor-center-vs-mesh-bounding-box-center ≈7.3 mm discrepancy note | §0, §1.5, §3, §6, §7, §8.1 |
| `docs/source_of_truth/mass_properties/MASS_PROPERTIES.md` §6.1 (authored by `geometry-structure`) | Main battery center coordinate, mass, and CG-relative offset — now `CONFIRMED` there, following `geometry-structure`'s own follow-up sync pass | §1.3, §8.1 |
| Manufacturer nominal APC 13x6.5E dimensions (13 in diameter / 6.5 in pitch) | D = 0.3302 m, pitch = 0.1651 m | §0, §1, §2 |

---

## 10. Summary — what this document does and does not establish

**Established, this pass (in addition to everything already established in the prior version):**
- Motor electrical parameters (KV, R, I0, mass, max current, max power) are `CONFIRMED` component/manufacturer data, not derived or assumed — the prior "motor electrical parameters unknown" framing is corrected (§1.1, §3, §8.1).
- ESC identity, mass, and the V1 `V_ESC = throttle × V_battery` model are `CONFIRMED` (§1.2).
- Battery Vnom/Vfull are now this specific pack's recorded spec (not merely a generic chemistry convention), plus battery mass/center and a secondary-battery identity (§1.3).
- Propeller mass, and — critically — **rotation direction (Left=CCW, Right=CW) is now `CONFIRMED`**, together with the explicit caveat that the physically-reversed side requires a genuinely reverse-handed APC 13x6.5EP part, not a re-wired 13x6.5E (§1.4).
- The full SI motor + propeller equation set (Kt, Ke, V=IR+Keω, Q_motor=Kt(I−I0), the I_rotor·dω/dt angular-dynamics ODE, n/J/T/P_prop/Q_prop/Q_reaction) is now recorded, replacing the previous architecture-only description with an equation-complete (though not yet numerically closed) model (§2, §4).
- Static-thrust validation references now exist from two independent sources and are recorded as calibration anchors, not as a substitute for a full Ct(J)/Cp(J) table (§1.6, §5).
- A distinct `APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION` tag now separates "the coefficient table is missing" from "the architecture is undefined" (§3, §5, §8.2).
- The reaction-torque architecture is now confirmed to be the counter-rotating case, with the differential-thrust (`r×F`, no fake yaw derivative) and thrust-line pitch-moment items documented as intended architecture, not implemented (§7).
- Motor/prop mount coordinates and thrust axis, and the main battery center coordinate, are now `CONFIRMED` — published by `geometry-structure` in `GEOMETRY.md` §7 and `MASS_PROPERTIES.md` §6.1 respectively, following its own follow-up sync pass. This document's earlier "value known but not yet authoritative" caveat no longer applies (§1.3, §1.5, §8.1).

**Not established (intentionally, per task scope — unchanged from before):**
- No propulsion plugin, controller, or simulation code.
- No numeric Ct(J)/Cp(J) coefficient table, thrust curve, or torque curve — the architecture is defined, the table is not (`APC_CT_CP_TABLE_REQUIRED_FOR_IMPLEMENTATION`).
- No motor/propeller rotational inertia number (only a V1-estimate/bench-calibration path).
- No battery internal resistance/SOC curve, ESC dynamic-response model, or motor efficiency/thermal model — all explicitly V2 per the master dataset itself.
- No edits to `docs/source_of_truth/README.md`'s structure beyond its `propulsion/` bullet section (updated separately, per this task), and no edits to any `.stl`, `model.sdf`, plugin, world, launch, or ArduPilot config file.

# FALCON V2 — Control Authority Effectiveness Validation: Gazebo vs. New XFLR5 Fixed-Condition Data

Task: `CONTROL_AUTHORITY_EFFECTIVENESS_VALIDATION` (comparison/classification stage)
Owner: `aerodynamics` (read-only analysis for this task — see confirmation below)
Date: 2026-08-26

## 0. Scope and no-edit confirmation

This document is a **read-only comparison and classification**. No aerodynamic
coefficient, config value, or plugin code was changed to produce it.

Confirmed by direct inspection at the time of writing:

```
$ git status --porcelain -- plugins/aerodynamics/AeroModel.hh docs/source_of_truth/aerodynamics/aero_v1_config.yaml
(no output — files are clean, no modifications, no untracked changes)
$ git diff -- plugins/aerodynamics/AeroModel.hh docs/source_of_truth/aerodynamics/aero_v1_config.yaml
(no output — zero diff against HEAD)
```

Neither `plugins/aerodynamics/AeroModel.hh` nor
`docs/source_of_truth/aerodynamics/aero_v1_config.yaml` was touched by this
task. `CL0`/`Cm0`/`CD0`/the drag model/the high-alpha limiter and
`aero_v1_config.yaml` are unmodified. No lookup table or nonlinear model was
implemented. Stall/post-stall behavior was not touched.

## 1. Data sources

- Quasi-static: `tests/gazebo/results/control_authority_quasi_static_result.json`
  (21 points; 7 elevator Δ-increments relative to trim, 7 aileron absolute, 7
  rudder absolute), `..._log.txt` for narrative.
- Free flight: `tests/gazebo/results/control_authority_free_flight_result.json`
  (6 runs, Δ=±5° each channel, 2.5 s, `any_nan=False` on all 6),
  `..._log.txt` for narrative. All quoted free-flight numbers below were
  independently re-extracted from the raw `series`/`early_window`
  arrays (t=0.3 s snapshot) in the JSON, not just taken from the log.
- Force/moment model: `plugins/aerodynamics/AeroModel.hh` (`ComputeAero()`,
  the `cmStatic`/`cmRate` split, and `my = qbar*S*c_ref*(-cmStatic+cmRate)`).
- Current config: `docs/source_of_truth/aerodynamics/aero_v1_config.yaml`
  (`Cmde_per_rad=-0.73`, `Clda_per_rad=0.308`, `Cnda_per_rad=0.00144`,
  `CYda_per_rad=0.0254`, `CYdr_per_rad=0.085`, `Cndr_per_rad=-0.025`,
  `Cldr_per_rad=0.0007`, `control_deflection_clamp_deg=10.0`).
- New XFLR5 data: `docs/source_of_truth/aerodynamics/control_surface_analysis/FALCON_V2_CONTROL_SURFACE_WIDE_DEFLECTION_RESULTS.txt`
  (Type-1 fixed-speed VLM2, viscous OFF, V∞=18.162 m/s, alpha=2.472°, β=0°).
- Old master dataset comparison values (`AERODYNAMICS.md` §7.2/§7.3, Type-7
  sweeps at the neutral-vertical-fin trim point, V=21.244 m/s, alpha=0.364°)
  were also consulted, because they are the direct provenance of the current
  `aero_v1_config.yaml` lateral-directional control derivatives and are
  needed to correctly diagnose the Cl_delta_r sign question in §4.3.

Reference geometry used for all N·m/N conversions below (CONFIRMED,
`aero_v1_config.yaml`): S = 0.4514 m², b = 2.093 m, c_ref = 0.224 m.

## 2. Method note (interpolation/derivative-extraction method used in this task)

No new interpolation was introduced. Two methods are used, both already
present in the quasi-static JSON or trivially derived from it:

1. **Central difference** (`central_diff` field, computed by `gazebo-testing`):
   `slope = (Cx(+Δ) - Cx(-Δ)) / (delta_actual(+Δ) - delta_actual(-Δ))`,
   using the *actual achieved* (not commanded) deflection as the x-axis, at
   three windows (w2/w5/w10 ≈ ±2°/±5°/±10° actual deflection). Valid only
   where both bracketing points are unclamped (see §3.1 for elevator).
2. **Linear least-squares fit** (this task, elevator only, to strengthen the
   single w2 estimate): ordinary least squares of `Cm` vs. actual `delta_e`
   (rad) across the 5 confirmed-unclamped quasi-static elevator points
   (Δcmd = −2, 0, +2, +5, +10°). Valid range: actual `delta_e` ∈
   [−7.4995°, +4.5027°], i.e. strictly inside the ±10° clamp. Not
   extrapolated beyond this range.

## 3. Elevator

### 3.1 The clamp-saturation finding (confirmed numerically)

Trim: `delta_e_aero_trim = -5.4995°` (elevator_theta = +5.5° both sides,
`elevator_sign=-1.0`). The quasi-static elevator sweep tests Δcmd ∈
{−10,−5,−2,0,+2,+5,+10}° as increments *relative to this trim*, so the
absolute `delta_e` fed into `AeroModel.hh`'s linear formulas is
`delta_e_aero_trim + Δcmd`, clamped to ±10° by `controlDeflectionClamp`
before the coefficient build-up (`V1_CONSERVATIVE_CLAMP`, "no silent
extrapolation").

Actual absolute `delta_e` achieved (from `actual_delta_e_deg` in the JSON):

| Δcmd (deg) | actual delta_e (deg) | inside ±10° clamp? |
|---|---|---|
| −10 | −15.4987 | **NO** — clamped to −10 |
| −5 | −10.4994 | **NO** — clamped to −10 |
| −2 | −7.4995 | yes |
| 0 (trim) | −5.4995 | yes |
| +2 | −3.4993 | yes |
| +5 | −0.4988 | yes |
| +10 | +4.5027 | yes |

Confirmed directly in the JSON: `Cm` (diagnostic, XFLR5-unflipped convention)
at `ELEVATOR_DELTA_M10DEG` = 0.0671620480 and at `ELEVATOR_DELTA_M5DEG` =
0.0671622408 — a difference of 1.9e-7 despite the commanded increment
differing by 5° and the actual joint deflection differing by ~5° (−15.4987°
vs −10.4994°). This is only possible if both hit the same internal −10°
clamp before the coefficient formula runs. `tracking_error_deg` on both
points is ~0.001–0.0013°, and `actuator_limited_response=false`,
`smooth_ok=true` — the actuator/joint itself tracked the full −15.5°
commanded deflection accurately; the clamp is entirely inside
`AeroModel.hh`, not an actuator effect.

Consequence: `central_diff`'s w5 (−0.6954/rad) and w10 (−0.5307/rad) windows
for `Cm_delta_e_GZ` are **not valid small-signal derivative measurements** —
they average a clamped and an unclamped point (w5: M5DEG clamped vs P5DEG
unclamped) or two differently-clamped points (w10: M10DEG clamped at −10°
vs P10DEG unclamped at +4.5°), which is a clamp-boundary artifact, not
aerodynamic-model nonlinearity.

### 3.2 Measured Gazebo value used

The w2 window (M2DEG=−7.4995° vs P2DEG=−3.4993°, both unclamped) is the
correct minimal estimate. To strengthen it, this task also fit a line
through all 5 confirmed-unclamped points (§2, method 2):

| Method | Cm_delta_e_GZ (/rad) | CL_delta_e_GZ (/rad) |
|---|---|---|
| w2 central diff (2 points) | −0.7319946 | +0.0006738 |
| 5-point OLS fit (this task) | −0.7319839 | +0.0006380 |

Agreement between the 2-point and 5-point estimates is 0.001% — the
elevator model is genuinely linear across the entire unclamped range; the
w5/w10 "nonlinearity" is 100% attributable to the clamp, not to any curvature
in the Cm(delta_e) relationship itself.

**Value used for comparison below: Cm_delta_e_GZ = −0.7320/rad**
(matches the configured `Cmde=-0.73/rad` to within 0.3% — i.e. the
implementation faithfully executes its own configured value).

### 3.3 Comparison

| Quantity | Gazebo (measured) | XFLR5 new (fixed-condition) | Abs. diff | % diff |
|---|---|---|---|---|
| Cm_delta_e (/rad) | −0.7320 | −1.000 | +0.268 | 26.8% (of XFLR5 new); GZ is 36.6% too small in magnitude |
| CL_delta_e (/rad) | ~+0.0006 (no CLde term — see below) | +0.414 | +0.413 | GZ omits ~100% of this derivative |

`CLde` is **deliberately omitted** from the current V1 CL build-up (not
just zero-valued — no term exists), per `aero_v1_config.yaml`'s own note: an
earlier finite-difference attempt from the old trim-sweep table gave an
unstable 0.080–0.508/rad spread and was rejected as unfabricatable. The tiny
nonzero measured `CL_delta_e_GZ` (~0.0005–0.0007/rad, itself not stable
across w2/w5/w10) is not a real elevator lift derivative — it is a
second-order artifact of alpha shifting slightly across the qstatic
hold-controller's trim points, confirmed by its own instability across
windows (unlike every genuine derivative in this report, which agrees to
4+ significant figures across w2/w5/w10).

### 3.4 Classifications

- **LINEARITY_CHECK (Cm_delta_e): `OTHER`** — not `MILD_NONLINEARITY` (that
  would misattribute the w5/w10 spread to the aero model's own behavior when
  it is actually a clamp-saturation artifact from combining a wide test
  window with a nonzero trim offset) and not `ACTUATOR_LIMITED_RESPONSE`
  (the actuator was not the limiting factor — `tracking_error_deg` and
  `smooth_ok=true` on the −15.5° point confirm the joint tracked the full
  commanded deflection smoothly; the aero model's own deliberate input clamp
  is what limited the *aerodynamic* response, not the mechanism).
- **LINEARITY_CHECK (CL_delta_e): `OTHER`** — near-zero-magnitude secondary
  coupling artifact, not a real control derivative to begin with (no CLde
  term exists), so "linear vs nonlinear" does not meaningfully apply.
- **Overall channel classification: `KNOWN_IMPLEMENTATION_GAP`** — both gaps
  (Cmde 27% low in magnitude, CLde entirely absent) are pre-existing,
  self-documented V1 limitations in `aero_v1_config.yaml`, now quantified
  against real new fixed-condition data rather than being unexplained.

### 3.5 Practical +5° authority

Actual achieved increment: Δcmd=+5° → `delta_e` goes from trim −5.4995° to
−0.4988° (an actual change of +5.0007°, unclamped both ends).

- ΔCL = +5.69e−5 (`delta_vs_baseline.CL` at `ELEVATOR_DELTA_P5DEG`) —
  negligible, consistent with no CLde term.
- ΔCm (diagnostic, XFLR5-unflipped) = −0.063887 (`delta_vs_baseline.Cm`).
- **ΔMy (applied, sign-corrected per `my = qbar·S·c_ref·(−cmStatic+cmRate)`,
  assuming cmRate≈0 at this quasi-static, rate-held point):**
  `= −qbar·S·c_ref·ΔCm_diag = −(202.178)(0.4514)(0.224)(−0.063887)`
  **≈ +1.306 N·m** (positive = nose-down, per `AeroModel.hh`'s FLU
  convention). This is physically correct: +delta_e in XFLR5's convention is
  TE-down, which for a tail aft of the CG produces a diving (nose-down)
  moment.
- Free-flight cross-check (`elevator_plus5`, t=0.3 s after release): q =
  +2.838°/s (nose-down). Sign matches ΔMy>0 exactly — no rate-coupling
  competing term to reconcile here (Cmq is a pure damping term, cannot flip
  the early-time sign).

## 4. Aileron

### 4.1 Measured Gazebo values

All three aileron derivatives are confirmed **linear** — w2/w5/w10 agree to
5–6 significant figures in the JSON `central_diff` block (no clamp issue:
trim is neutral, so the full ±10° absolute test range never approaches the
±10° clamp boundary):

| Derivative | w2 | w5 | w10 |
|---|---|---|---|
| Cl_delta_a (/rad) | 0.3063356 | 0.3063356 | 0.3063356 |
| Cn_delta_a (/rad) | 0.0011965 | 0.0011965 | 0.0011966 |
| CY_delta_a (/rad) | 0.0254783 | 0.0254781 | 0.0254778 |

### 4.2 Comparison

| Derivative | Gazebo (measured) | Config value | XFLR5 new | Abs diff | % diff (of new) |
|---|---|---|---|---|---|
| Cl_delta_a | 0.30634 | 0.308 | 0.414 | +0.1077 | 26.0% |
| Cn_delta_a | 0.0011965 | 0.00144 | 0.0017 | +0.0005035 | 29.6% |
| CY_delta_a | 0.025478 | 0.0254 | 0.0045 | −0.020978 | **−466%** (GZ is ~5.7× larger than new value) |

### 4.3 Classifications

- **LINEARITY_CHECK: `LINEAR`** for all three (confirmed above).
- **Overall channel classification: mixed.**
  - Cl_delta_a, Cn_delta_a: **`KNOWN_IMPLEMENTATION_GAP`** — same pattern as
    elevator: current config derives from the older `AERODYNAMICS.md` §7.3
    Type-7 sweep (full-range Clda≈0.308, Cnda≈0.00144), and the new
    fixed-condition sweep gives systematically ~26–30% larger values in the
    *same direction/sign*. Not unexpected in magnitude — this is exactly the
    kind of "older/lower-fidelity sweep vs. newer dedicated wide-deflection
    sweep" gap already anticipated by the project's iterative data-collection
    approach.
  - CY_delta_a: **`UNEXPLAINED_MISMATCH`** (flagged, secondary priority
    after §5's Cl_delta_r item). A 5.7× factor is far larger than the 26–30%
    gaps seen on Cl_delta_a/Cn_delta_a from the same two datasets, and is not
    explained by "older/newer sweep" alone (if it were a common delta_a
    scaling/definition difference between the two XFLR5 sessions, Cl_delta_a
    would show the same factor, and it does not). CY_delta_a is a small,
    secondary/tertiary coefficient (~40× smaller than Cl_delta_a), plausibly
    more sensitive to viscous/inviscid settings or fuselage/tail interaction
    modeling differences between the two XFLR5 sessions, but this is not
    confirmed — recommend dedicated review before touching either value.

### 4.4 Practical +5° authority

Actual achieved: `delta_a` = +4.9997° (unclamped).

- ΔCl = +0.026731, ΔCn = +0.0001044, ΔCY = +0.002223
  (`delta_vs_baseline` at `AILERON_DELTA_P5DEG`, qbar=202.158).
- **ΔMx (applied) = qbar·S·b·ΔCl = (202.158)(0.4514)(2.093)(0.026731)
  ≈ +5.106 N·m.**
- ΔMz (adverse-yaw coupling, zero body-rate) = qbar·S·b·ΔCn ≈ +0.0199 N·m
  (positive = nose-left tendency, the classic adverse-yaw direction for this
  roll sense, per `AeroModel.hh`'s "+Z rotation → nose left" convention).
- Free-flight cross-check (`aileron_plus5`, t=0.3 s): p=+37.427°/s,
  roll=+7.227°, beta=+0.100°, **r=−4.648°/s** (nose-*right*, opposite sign
  from the zero-rate ΔMz above). This is expected, not a defect: by t=0.3 s,
  p has built up to 37.4°/s, and `p_hat = p·b/(2V) ≈ 0.0374`; the roll-rate-
  into-yaw derivative `Cnp = −0.05878` (CLAUDE.md reference point) then
  contributes `Cnp·p_hat·qbar·S·b ≈ −0.42 N·m` — roughly 20× larger than the
  +0.0199 N·m static Cnda coupling and opposite in sign. The measured
  free-flight r sign is dominated by Cnp once rate builds up, not by the
  small static aileron-yaw coupling seen in the (zero-rate) quasi-static
  sweep. Both signs are internally consistent with the model's own
  coefficients; this is not a sign error.

## 5. Rudder

### 5.1 Measured Gazebo values (all linear, no clamp issue — neutral trim)

| Derivative | w2 | w5 | w10 |
|---|---|---|---|
| CY_delta_r (/rad) | 0.0849169 | 0.0849169 | 0.0849169 |
| Cn_delta_r (/rad) | −0.0249784 | −0.0249784 | −0.0249784 |
| Cl_delta_r (/rad) | +0.0006724 | +0.0006724 | +0.0006724 |

### 5.2 Comparison

| Derivative | Gazebo (measured) | Config value | XFLR5 new | Abs diff | % diff (of new) |
|---|---|---|---|---|---|
| CY_delta_r | 0.084917 | 0.085 | 0.0916 | +0.006683 | 7.3% |
| Cn_delta_r | −0.024978 | −0.025 | −0.0272 | −0.002222 | 8.2% |
| **Cl_delta_r** | **+0.0006724** | **+0.0007** | **−0.00065** | **−0.0013224** | **203%** (opposite sign) |

### 5.3 SIGN FINDING — Cl_delta_r (top-priority open item for the next stage)

Measured Gazebo `Cl_delta_r ≈ +0.0006724/rad`, matching the configured
`Cldr=+0.0007/rad` almost exactly — the implementation is correctly and
faithfully reproducing its own configured/source value. The new XFLR5
fixed-condition sweep gives `Cl_delta_r = −0.00065/rad` — opposite sign,
comparable magnitude.

This was traced one level further back than the task brief's raw numbers, to
the two underlying XFLR5 source tables, and the finding is more specific
than "unknown, could be either side":

- **Old master dataset** (`AERODYNAMICS.md` §7.2, Type-7 sweep, `MD §29/§32`,
  at the neutral-vertical-fin trim point V=21.244 m/s / alpha=0.364°): at
  delta_r=+10°, Cl=+0.000128; at delta_r=−10°, Cl=−0.000133. **Positive**
  slope, smooth and antisymmetric through zero. This table is the direct,
  cited provenance of the current `Cldr_per_rad=+0.0007` config value — the
  config is not a transcription error; it correctly encodes this table.
- **New fixed-condition sweep** (this task's input file, V=18.162 m/s /
  alpha=2.472°): at delta_r=+10°, Cl=−0.00012; at delta_r=−10°, Cl=+0.00012.
  **Negative** slope, also smooth and antisymmetric through zero.

Both tables are individually well-behaved (smoothly varying, antisymmetric,
internally self-consistent, no noise/outliers) — this is a genuine
**disagreement between two separate XFLR5 analysis sessions at two different
operating points**, not a Gazebo implementation bug, not a read/transcription
error, and not an actuator effect. `AeroModel.hh`/`aero_v1_config.yaml` are
correctly executing the (old) source data they were given.

Plausible cause (not confirmed): `Cl_delta_r` is two orders of magnitude
smaller than `CY_delta_r`/`Cn_delta_r` (a tertiary rudder→roll coupling,
likely from asymmetric vertical-tail/rudder loading interacting with
dihedral/wing effects at nonzero alpha). Coefficients of this magnitude are
plausibly sensitive to differences in operating alpha/V or vertical-fin/
rudder rigging state between the two XFLR5 sessions (the old dataset is
explicitly the "neutral vertical-fin" full-aircraft reference point per
`CLAUDE.md`, run at a different alpha/V than the new fixed-condition sweep).
A sign-convention bookkeeping difference in how each XFLR5 session defines
positive rudder deflection cannot be ruled out either, without directly
re-opening both XFLR5 project files to compare their rudder positive-TE
direction definitions — that direct comparison was not performed as part of
this read-only task.

**Classification: `UNEXPLAINED_MISMATCH` (sign-mismatch sub-case).** Not
`KNOWN_IMPLEMENTATION_GAP` (the current implementation is not "known wrong"
— it faithfully encodes a real, cited source table) and not
`ACTUATOR_LIMITED_RESPONSE` (rudder tracked commands accurately in both
quasi-static and free-flight tests). **This is flagged as the single most
important open item for `HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION`** to
resolve, since that is the stage where any coefficient/sign change would
actually be made — no change is made here.

### 5.4 Other classifications

- **LINEARITY_CHECK: `LINEAR`** for all three rudder derivatives (5–6 sig
  fig agreement across w2/w5/w10, confirmed above).
- **CY_delta_r, Cn_delta_r overall classification: `ACCEPTABLE_DIFFERENCE`**
  (~7–8% gaps, same sign, same order of magnitude as the elevator/aileron
  "older vs. newer sweep" pattern — small enough that no urgent action is
  implied, though the recommendations list below still notes them as
  optional refinements).

### 5.5 Practical +5° authority

Actual achieved: `delta_r` = +4.9998° (unclamped).

- ΔCY = +0.007410, ΔCn = −0.0021797, ΔCl = +0.00005867
  (`delta_vs_baseline` at `RUDDER_DELTA_P5DEG`, qbar=202.158).
- **ΔMz (applied) = qbar·S·b·ΔCn = (202.158)(0.4514)(2.093)(−0.0021797)
  ≈ −0.4163 N·m** (negative = nose-right, per the FLU "+Z=nose-left"
  convention).
- ΔMx (coupling, zero rate) = qbar·S·b·ΔCl ≈ +0.0112 N·m.
- ΔFy = qbar·S·ΔCY ≈ +0.676 N.
- Free-flight cross-check (`rudder_plus5`, t=0.3 s): **r=−4.652°/s**
  (nose-right) — matches ΔMz<0 directly, no competing-term reconciliation
  needed this early (the applied yaw torque is the dominant term at
  release). beta=+0.769°, p=−0.562°/s: this p sign is *opposite* the
  zero-rate ΔMx=+0.0112 N·m static Cl_delta_r coupling above, but is
  explained by the dihedral term `Clb·beta`: with `Clb=−0.00717` (CLAUDE.md
  reference point) and beta already at 0.769°=0.01342 rad by t=0.3 s,
  `Clb·beta ≈ −9.6e−5` — larger in magnitude than and opposite in sign to
  the direct `Cldr·delta_r` term (~+6.1e−5 at this deflection) — so the net
  roll torque has already flipped sign by the time beta builds up. Both
  quasi-static and free-flight roll-sign behavior are internally consistent
  with the model's own Clb/Cldr coefficients; not a defect.

## 6. Summary classification table

| Channel | Derivative | LINEARITY_CHECK | Classification |
|---|---|---|---|
| Elevator | Cm_delta_e | OTHER (clamp-saturation artifact in w5/w10; genuinely linear within unclamped range) | KNOWN_IMPLEMENTATION_GAP |
| Elevator | CL_delta_e | OTHER (no real term exists) | KNOWN_IMPLEMENTATION_GAP |
| Aileron | Cl_delta_a | LINEAR | KNOWN_IMPLEMENTATION_GAP |
| Aileron | Cn_delta_a | LINEAR | KNOWN_IMPLEMENTATION_GAP |
| Aileron | CY_delta_a | LINEAR | **UNEXPLAINED_MISMATCH** (5.7× factor, secondary priority) |
| Rudder | CY_delta_r | LINEAR | ACCEPTABLE_DIFFERENCE |
| Rudder | Cn_delta_r | LINEAR | ACCEPTABLE_DIFFERENCE |
| Rudder | Cl_delta_r | LINEAR | **UNEXPLAINED_MISMATCH — sign flip, TOP PRIORITY** |

## 7. Recommended changes for `HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION` (recommendations only — nothing implemented here)

1. **Resolve the Cl_delta_r sign conflict first** (§5.3) — re-open both
   underlying XFLR5 project files (old neutral-vertical-fin trim sweep vs.
   new fixed-condition sweep) and directly compare their rudder
   positive-deflection/geometry definitions before adopting either sign.
2. Investigate the CY_delta_a 5.7× discrepancy (§4.3) before changing
   `CYda_per_rad` — check for a viscous/inviscid or delta_a-normalization
   difference between the two XFLR5 sessions specific to this coefficient.
3. Add a `CL_delta_e` term to the CL build-up (currently entirely omitted) —
   the new fixed-condition data gives a stable +0.414/rad across ±2/±5/±10°
   and across two operating points (this file's primary and secondary sweep),
   unlike the old trim-sweep-derived attempt that was rejected as unstable.
4. Consider updating `Cmde_per_rad` from −0.73 toward −1.000/rad (27% low in
   magnitude vs. the new fixed-condition value).
5. Consider updating `Clda_per_rad` from 0.308 toward 0.414/rad (26% low).
6. Consider updating `Cnda_per_rad` from 0.00144 toward 0.0017/rad (measured
   effective value is 30% low vs. new; config itself is also below new).
7. Lower priority: consider updating `CYdr_per_rad`/`Cndr_per_rad` toward
   0.0916/−0.0272 (~7–8% gaps, `ACCEPTABLE_DIFFERENCE`, optional refinement).
8. Document explicitly, project-wide, which operating point (old
   neutral-vertical-fin trim point, V=21.244 m/s/alpha=0.364°, vs. new
   fixed-condition point, V=18.162 m/s/alpha=2.472°) is the authoritative V1
   reference condition going forward, since several of the gaps above may
   partly reflect the two source analyses being evaluated at different
   operating points rather than pure control-derivative disagreement.

No coefficient, config value, or code was changed as part of producing this
document.

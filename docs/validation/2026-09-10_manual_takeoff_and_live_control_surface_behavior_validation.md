# VALIDATION REVIEW RECORD — MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION (2026-09-10)

**Verdict: READY**, conditional on five MAJOR record corrections.

Independent read-only review by `validation`. This file is the persistent record of that review;
it was written by the coordinator because `validation` has no write access. It **persists** the
verdict and findings — it does not re-run, re-analyse, re-interpret or extend them, and it makes
no new engineering decision. Authorship convention follows
`docs/validation/2026-09-08_ardupilot_navigation_and_autotune_validation.md`.

The stage record itself (execution, evidence, timelines, root cause) is
`docs/test_results/2026-09-10_manual_takeoff_and_live_control_surface_behavior_validation.md`.

---

## 1. Stage and ruling

| | |
|---|---|
| Stage | `MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION` |
| `validation` ruling | **READY**, conditional on five MAJOR record corrections |
| CRITICAL findings | **none** |
| Verdicts audited | 6, **all CONFIRMED** on independent re-derivation |
| Tuning / physics change | **none**, independently confirmed (§5) |

Every required correction is a **document/record fix**. None requires a simulation re-run and none
changes an engineering parameter.

## 2. Agent chain

`controls-integration` (stage primary — harness, offline closed-loop analysis, root cause,
verdicts, report) → `gazebo-testing` (live campaign: visual/joint consistency, six sign scenarios,
6 ground-roll runs MANUAL/FBWA, 3 runs TAKEOFF, user-dataflash extraction, sign re-judgement) →
`aerodynamics` (called in **only** for the rudder `Cn` question and the `aero_lib` re-sync, because
the root cause rested on that model) → `validation` (independent read-only review).

`general-purpose` was **not** used, per CLAUDE.md's orchestration rule.

## 3. Verdicts

| Verdict | Value | `validation` ruling |
|---|---|---|
| `VISUAL_SURFACE_KINEMATICS` | PASS | **CONFIRMED** — survives without the rate correction (ratio 33–34 vs pre-registered gate 10) and without the well-bound subset; 89.5 deg hypothesis separation |
| `AILERON_CONTROL_SIGN` | PASS | **CONFIRMED** — O1/O2/O3 all pass; floors imported not changed; ground-roll antisymmetry corroborates |
| `ELEVATOR_CONTROL_SIGN` | PASS | **CONFIRMED** — `Cm` never negative under nose-up demand (+0.4859…+3.1743 verified); both surfaces same-way; nose-up motion once free |
| `RUDDER_CONTROL_SIGN` | PASS | **CONFIRMED** — on the live O1/O2/O3 observables and the yaw budget, **not** on the oracle (see M1) |
| `TAKEOFF_LEFT_DRIFT` | CONTROL_CAUSED | **CONFIRMED** — onset ordering and the MANUAL/FBWA control condition are each independently sufficient |
| `24MPS_LIFTOFF` | GROUND_MODEL_ARTIFACT | **CONFIRMED with qualification** — carried by measurement; reason 1 is dataset-specific (M4), the moment budget is marginal not decisive (M3). `GROUND_MODEL_ARTIFACT`, not `MIXED`, is correct |

## 4. The user's six review questions

**Q1 — Is "visual follows the ACTUAL joint" sound? YES.** The rate correction is not doing the
work and the subset is not cherry-picked. Three tiers are all reported: well-bound + rate-corrected
660–722, well-bound uncorrected 205–230, **all samples uncorrected 33–34**, against a pre-registered
gate of 10. `residual_vs_CMD` is essentially identical across all three tiers (median 3.33–3.66 deg)
— subsetting moves only the *actual*-hypothesis residual, the signature of a time-binding artefact
rather than selection bias. The subset criterion (`dt <= 0.006 s`) is independent of the hypothesis
under test. `max_cmd_minus_actual_deg` 89.25–89.52; `max_off_axis_residual` 1.4e-09;
`geometry_self_test worst_error` 1.1e-16.

**Q2 — Are the control-sign interpretations correct? YES.** Thresholds were **not** changed:
`rejudge_control_surface_sign_scenarios.py:91-93` imports the floors directly from the original
harness (`0.5 deg / 2.0 deg/s / 1e-4`) and reads expectations from
`manual_takeoff_live_lib.PRE_REGISTERED_EXPECTATIONS`. The onset definition is a *magnitude* test;
the sign is checked separately, so the test is non-circular. The verdicts do not depend on the beta
guard (O2 and O3 pass 6/6 without it). Scenario D's O3-at-onset FAIL was reproduced and confirmed
as an absolute-vs-delta definitional artefact, not an excuse. The structural yaw argument is
predictive, not post-hoc: at steady state `Cnb*beta = +0.012051` genuinely exceeds and reverses
`dCn_rudder = -0.009946`.

**Q3 — Is the drift attribution evidenced? YES, and it stands without the L1 arithmetic.** Two
independent legs: (i) onset ordering — surface command +6.900 s, yaw rate +8.217 s, lateral
+8.736 s, lead 1.317 s, with `|Y| = 3.33e-07 m` and `yaw = 5.63e-05 deg` at command onset, and
`RCIN` C1/C2/C4 constant at 1500 us (max departure 0) while `RCOU` C1 departs 237 us; (ii) the
control condition — 6/6 MANUAL/FBWA runs with aileron and rudder `cmd_max_abs_deg = 0.0000` exactly
and `max|Y| = 1.74e-05 … 1.76e-04 m` on the identical aircraft, contact, friction and propulsion.
ArduPilot source references verified at commit `409226a6`. The L1 prediction is corroboration and
the report correctly hedges its residual as "to within the log sampling rate, not a bit-exact
match"; the nav_roll agreement should not be quoted as quantitative confirmation on its own.

**Q4 — Was 24 m/s misinterpreted by ignoring the missing landing gear? NO**, but the verdict is
oversold in two specific ways (M3, M4). The report never validates 24 m/s as `TAKEOFF_SPEED`, labels
it only `CURRENT_BELLY_CONTACT_TEST_LIFTOFF_SPEED`, and states that even that label attaches to the
wrong number. The roll-tip identity holds: `max|z - 0.28*sin(roll)|` = 2.94e-06 / 2.07e-06 /
3.13e-06 m, and a 1 mm-resolution best-fit half-width search returns **exactly 0.280 m**, the
independently-derived `model.sdf` value. `pitch_std_in_contact` 0.354 / 0.309 / 0.282 deg over
13.9 s of a 14.3 s roll with `actual_angle_rad = 0.7853981634` (= pi/4, the SDF joint limit) and
both clamp flags 0. The moment budget was reproduced bit-for-bit.

**Q5 — Was any tuning or physics change made? NO.** See §5.

**Q6 — High-J propeller limitation.** `PROPULSION_HIGH_J_WINDMILLING` (APC 13x6.5E `Ct/Cp` has no
data above J ≈ 0.64; `docs/test_results/2026-09-03_*` §9, MINOR / `DATA_REQUIRED` / OPEN) is **not
relevant to the decisive segments** but **was silently dropped** from this stage's report — MINOR
finding m1. Ground-roll J never exceeds 0.402 / 0.411 / 0.402, and the 22–28 `interpClamped`
ground-roll samples all occur in the first 0.104–0.133 s at `J = 0.0000`, RPM 54–977 — the
**low-RPM** table bound, not high-J, with thrust ≈ 0. It is active airborne (J up to 1.05 / 1.49 /
1.06; scenario D at 15.4% of samples), but no verdict depends on the post-departure tumble and the
clamp is symmetric between motors, so it cannot flip a control-surface sign. Conclusions unaffected;
it should have been stated and dismissed rather than omitted.

## 5. Independent confirmation: no tuning, no physics change

`validation` established this from five independent lines rather than from any agent's assertion:

1. `git status` clean for `model/`, `plugins/`, `config/`, `docs/source_of_truth/`. The only
   modified tracked file in the repository is `tests/gazebo/scripts/aero_lib.py` (+123 / −18).
2. mtimes of every engineering file predate the stage: `model/model.sdf` 2026-09-02;
   `aero_v1_config.yaml` 2026-08-26; `propulsion_v1_config.yaml` 2026-08-23;
   `falcon_v2_sitl.parm` 2026-09-05; `AeroModel.hh` and `AerodynamicsSystem.cc` 2026-08-26.
3. **No plugin was rebuilt** — all six `.so` binaries date 2026-08-23…2026-08-27, so the physics
   executed in this stage is byte-identical to prior stages.
4. No runtime parameter writes: zero `param_set` / `PARAM_SET` in any stage script.
5. `aero_lib.py` audited line by line. It defines **no coefficient value**; every number is read
   from `aero_v1_config.yaml`. The only introduced literals are `CTRL_ZERO_INDEX = 7` (verified
   against `AeroModel.hh:72 constexpr int kCtrlZeroIndex = 7`) and interpolation bookkeeping.

`aero_lib.py` could not have affected any prior verdict: the pre-stage version raises
`KeyError: 'control_deflection_clamp_deg'` on `load_config()` and has been unloadable since
2026-08-26.

**No CG-duality violation.** The XFLR5 CG `(0.0637, 0, -0.0210)` appears in **zero** stage scripts;
`model/model.sdf:400-403` documents the rule and uses only the Gazebo/CAD CG.

**Supporting integrity checks all pass.** The stage world SDF differs from the codex original in
**exactly two places** — a prepended header comment and the single `<include><uri>` line.
`verify_world()` genuinely gates and aborts the run on failure. `codex/model/model.sdf` has 84 added
lines, 0 removed, 0 changed, all visual. D1's criterion numbers are unchanged (0.10 m / 0.5 m/s /
0.30 s). The scenario retry is at the runner level and never re-runs a scenario that produced a
measurement. No `DATA_REQUIRED` was silently filled with an estimate; `Cmα̇` remains `DATA_REQUIRED`.

## 6. Findings

### CRITICAL — none

### MAJOR

| # | Finding | Owner |
|---|---|---|
| **M1** | The oracle claim is logically circular **as stated**. The oracle is real and was re-run independently (n=1486; residuals 0.000e+00 for V/α/qbar/CL/CD/Cl/Cm/Mx/My), and `oracle.cc` genuinely `#include`s the unmodified `AeroModel.hh` on the unmodified YAML. But `oracle_cmp.py` compares `aero_lib.compute_aero` against `AeroModel.hh` — proving the **Python mirror** is faithful, not that the plugin's physics is correct. §5.4's inference does not follow. `RUDDER_CONTROL_SIGN = PASS` stands on the live O1/O2/O3 observables and the yaw budget. | `aerodynamics` (claim), `controls-integration` (wording) |
| **M2** | Reference-frame/sign **error** in the report's conventions table. It states `My = qbar*S*c_ref*Cm` with "positive `Cm` = nose UP". `AeroModel.hh:705` computes `my = qbar*S*c_ref*(-cmStatic + cmRate)`, and `AeroModel.hh` documents "+Y (pitch) rotation → NOSE DOWN" in FLU. Wrong twice: the static group is negated, and `+My` is nose-**down**. Numerically harmless where used here (`cmRate = -3.0e-05` at the budget samples; `\|My\|` differs by +0.002 N·m) but `max\|cmRate\| = 0.667` over the full roll, comparable to `min(Cm) = 0.486`, so materially wrong wherever `q != 0`. In a stage whose purpose is sign conventions, this must be corrected. | `controls-integration` (report), `aerodynamics` (confirm) |
| **M3** | The moment budget **omits the D'Alembert term** `m*a_x*Δz`, which is nose-up and material: +2.08 N·m early, +0.89…+0.97 N·m at t-thr ≈ 9.2 s. Recomputed best case: run 1 −1.962 → **−1.071**; run 2 −1.656 → **−0.716**; run 3 −2.299 → **−1.334** (42–57% of the deficit erased). The sign survives but the margin is 1.5–2.8% of the 47 N·m budget, inside the span of single-parameter sensitivities (pivot ±1 cm → ∓0.33 N·m; hub height ±1 cm → ±0.5 N·m; `Cm` ±5% → ±0.8 N·m). §3.3's "…is 2-4% short … **so no nose-up rotation occurs**" asserts a causal certainty the budget cannot support. The verdict is carried by the **measurement**; that framing must lead. | `controls-integration` (wording), `geometry-structure` (pivot/hub uncertainty) |
| **M4** | **MANUAL/FBWA liftoff speeds are in the stage's own data and never reported**, over-scoping verdict F reason 1. MANUAL ×3: 25.15 / 25.68 / 26.09 m/s, pitch change 0.852 deg (`exceeded_1_deg = False`), lift=weight at 21.3–21.9 m/s. FBWA ×3: 22.85 / 23.07 / 23.40 m/s, pitch change 2.19–2.30 deg, lift=weight at 21.0–21.5 m/s. In the clean, straight, non-drifting roll the aircraft **does** lift off at 25–26 m/s without rotating — a *stronger* ground-model-artefact result, but it narrows reason 1 to the TAKEOFF dataset. | `gazebo-testing` (publish), `controls-integration` (re-word) |
| **M5** | Part 5 hypothesis-1 propulsion numbers are **not reproducible**. §2.4 #1 claims, for the throttle-onset → first-surface-command window, `max\|ΔRPM\| <= 5.1e-10`, `max\|ΔT\| <= 1.7e-11 N`, `max\|Mz\| <= 5.1e-12 N·m`. Measured at that exact window (index 1552→2223, n=672): **5.2e-09 / 5.2e-10 N / 1.6e-10 N·m** — 10–30× larger, with the ΔRPM/ΔT labels apparently swapped. No alternative window reproduces the published figures. **The ruling is entirely unaffected** (1.6e-10 N·m against an O(1 N·m) rudder moment). The second window reproduces exactly. | `gazebo-testing` |

### MINOR

| # | Finding | Owner |
|---|---|---|
| m1 | High-J limitation dropped from the report (see §4 Q6). Not relevant to the decisive segments; should have been stated and dismissed. | `propulsion`, `gazebo-testing` |
| m2 | D1's scope understated — the report says "all 6 MANUAL/FBWA runs"; **all nine** stock result files report `NO_LIFTOFF_WITHIN_WINDOW`, including the 3 TAKEOFF runs. | `gazebo-testing` |
| m3 | The moment budget is not persisted — no script or JSON in the repository produces `0.695189` / `47.24` / `45.66`. Reproduced independently, but not re-runnable from the repo. | `controls-integration` |
| m4 | The oracle lives only in an ephemeral session scratchpad, outside the repository, yet is cited as primary evidence. | `aerodynamics` |
| m5 | `BETA_GUARD_DEG = 2.0` is a new constant with a documented derivation but **no `ASSUMPTION` tag**, contrary to CLAUDE.md's labelling rule. | `gazebo-testing` |
| m6 | O3-at-onset uses **absolute** deflection. D's FAIL is correctly explained, but A/B/C/E/F's O3-at-onset *passes* are equally dependent on the trim point carrying the right sign. Report O3 as a delta or discount its at-onset passes. | `gazebo-testing` |
| m7 | "rendered" overstates `dynamic_pose/info` — it is the scene-graph transform, not the framebuffer. Sound proxy in gz-sim; say so precisely. | `controls-integration` |
| m8 | Oracle `Mz` residual quoted "~1e-19"; actual `max_abs_diff` is 4.163e-17. | `aerodynamics` |
| m9 | §3.1 #7 speeds "20.07 / 21.10 / 20.23" are min / max / run-1, not runs 1/2/3 in order. | `controls-integration` |

### INFO

- **i1** Elevator `actual` reaches 45.00000000020525 deg while `cmd_max = 44.614` — actuator
  overshoot into the SDF joint limit (±0.7853981634), both clamp flags 0. The report describes this
  accurately. Mapping `1.5707963268 rad × (x − 0.5)` ⇒ ±45 deg confirmed.
- **i2** `aero_lib.py` was unloadable from 2026-08-26 until this stage; **no prior verdict can have
  depended on it.**
- **i3** The airborne actuator limit cycle is **exactly 0 samples during every ground roll** and
  0/4890–4905 in every sign scenario — airborne only. **It contaminates no verdict.** Real defect,
  correctly routed to `controls-integration` for a future stage.
- **i4** D3 confirmed real — `contact_messages_ever_received = 0` while all 7358 samples carry
  `normal_force_source = "MEASURED /contacts"`. Correctly scoped, correctly corrected downstream,
  `DATA_REQUIRED` properly declared.
- **i5** All five of the user's 2026-09-08 logs lifted off in mode **TAKEOFF** — the stage's dataset
  choice and its characterisation of the user's flights are correct.

## 7. What the agents got wrong or overstated

1. The oracle proves the mirror, not the physics (M1).
2. `My = qbar*S*c_ref*Cm` is wrong and inverts the FLU pitch sign (M2).
3. The moment budget omits a real nose-up term of the same order as its own deficit, and "cannot
   rotate" is asserted beyond what it supports (M3).
4. MANUAL/FBWA liftoff at 25–26 m/s is in the stage's own data and is never reported (M4).
5. Hypothesis-1 propulsion figures are 10–30× off with labels apparently swapped (M5).
6. D1's scope is understated by three runs (m2).
7. The moment budget and the oracle are both unreproducible from the repository (m3, m4).

Nothing was left unverifiable except the moment budget and the oracle **as repository artifacts** —
`validation` reproduced both independently, so the numbers themselves are confirmed.

## 8. Key artifacts

| Path | Role |
|---|---|
| `docs/test_results/2026-09-10_manual_takeoff_and_live_control_surface_behavior_validation.md` | stage record under review |
| `tests/gazebo/results/manual_takeoff_ground_roll_TAKEOFF_independent_analysis.json` | decisive TAKEOFF-mode dataset |
| `tests/gazebo/results/manual_takeoff_ground_roll_independent_analysis.json` | MANUAL/FBWA control condition; holds the unreported liftoff speeds (M4) |
| `tests/gazebo/results/control_surface_live_sign_rejudgement.json` | corrected-observable A–F re-judgement, OLD and NEW side by side |
| `tests/gazebo/results/control_surface_visual_joint_consistency_live_result.json` | three-tier visual/joint discrimination |
| `tests/gazebo/results/codex_manual_takeoff_dataflash_extraction.json` | the user's own 2026-09-08 flights |
| `plugins/aerodynamics/AeroModel.hh:705` | the `My` relation contradicting the report's conventions table (M2) |
| `tests/gazebo/scripts/aero_lib.py` | the only modified tracked file in the repository |
| `tests/gazebo/scripts/rejudge_control_surface_sign_scenarios.py:91-93` | imported floors, proving thresholds unchanged |

---

## 9. Confirmation pass — corrections applied (same day)

`controls-integration` applied the five MAJOR corrections and the minors it owns.
`validation` re-reviewed the corrections only (not the stage) and ruled:

**READY TO CLOSE.** No CRITICAL, no MAJOR. Carry m10, i5, i6, i7 forward as non-blocking.

### 9.1 M2 confirmed

Report §0 rows now match `AeroModel.hh:705` (`my = qbar*S*c_ref*(-cmStatic + cmRate)`) and `:316`
("+Y (pitch) rotation → NOSE DOWN") verbatim. All five affected sites fixed (§0 table, §0 correction
note, §3.1 #3, §3.2a, §5.3); a grep of every `c_ref` and `Cm` occurrence across the 1009-line report
found no surviving instance of the wrong relation outside an explicitly flagged historical quote.

"No Part 6 number changes" is **demonstrated, not asserted**: `validation` regenerated
`ground_roll_pitch_moment_budget.json` in an isolated sandbox and got a **byte-identical** artifact.
`cmRate` at the three best-case budget samples = −2.473e-08 / −1.887e-06 / −5.865e-10, giving
`|ΔMy|` = 1.888e-06 / 1.454e-04 / 4.448e-08 N·m against a ~47 N·m budget.

**The cancellation reasoning is correct.** The original form was `qSc*(cmStatic + cmRate)`, the
correct form `qSc*(cmStatic − cmRate)`; the error is exactly `2*qSc*cmRate` and vanishes as
`cmRate → 0`. The two sign errors compose to identity on the static term, so "static-dominated
`Cm > 0` ⇒ nose up" was right **by coincidence** and survives. `validation` additionally audited
every arm and sign in the budget (`-My = r_z F_x − r_x F_z` for lift, weight, drag, thrust-at-hub
and D'Alembert); all six contributions carry the correct sense.

### 9.2 M5 adjudicated — `validation`'s original finding was WRONG on the label swap

`controls-integration` refused to apply the claimed ΔRPM/ΔT label swap and recorded the refusal
rather than complying. On independent re-derivation from the raw timeseries:

| run | window | max\|ΔRPM\| | max\|ΔT\| [N] | max\|Mz\| [N·m] |
|---|---|---|---|---|
| 1 | 1552→2218 (n=667) | 5.148e-10 | 1.701e-11 | 5.103e-12 |
| 2 | 1553→2215 (n=663) | 5.020e-10 | 4.444e-12 | 1.333e-12 |
| 3 | 1555→2224 (n=670) | **1.766e-09** | **1.241e-10** | **3.722e-11** |

`max|ΔRPM|` exceeds `max|ΔT|` by 14–30× in every window and every run — the ordering both the
original and corrected tables already show. **`validation` withdraws the swap sub-claim**: applying
it would have introduced a real error into a previously correct record, and refusing it was correct.

The *other* half of M5 stands: the published figures were run 1 quoted with a `<=` implying an
all-run bound. Run 3 is binding; `1.8e-09 / 1.2e-10 N / 3.7e-11 N·m` is the true bound and §2.4 #1
now carries it. Hypothesis 1 remains `NOT_SUPPORTED`, not close.

### 9.3 Budget script confirmed

`tests/gazebo/scripts/reproduce_ground_roll_pitch_moment_budget.py` re-ran from a clean sandbox and
reproduced §3.2a to the last published digit (static −1.590/−1.088/−1.957; with D'Alembert
−1.072/−0.716/−1.335; `a_x` 1.483/1.566/1.607; `Cm+5%` sensitivity +1.257…+1.272; deficit fractions
45/57/42 %). Repo-relative paths, no arguments, no simulation, no network. Every constant traces to
`CLAUDE.md`, `aero_v1_config.yaml` or `model.sdf`; **no new engineering value is invented**; `a_x`
is tagged `ASSUMPTION` in both the docstring and the JSON.

M3's re-framing confirmed: §3.2a now opens *"The measurement leads. The moment budget is CONSISTENT
WITH it, and is not by itself decisive"*, and §3.3 leads with `MEASURED RESULT:` before
`CONSISTENT WITH THAT MEASUREMENT:`.

### 9.4 Verdicts and prohibited files confirmed

All six verdicts byte-match §3 of this record. `git status --porcelain model/ plugins/ config/
docs/source_of_truth/` returns **zero** entries and `find` over those four trees returns **no** file
modified on or after 2026-09-10. The only modified tracked file repo-wide remains
`tests/gazebo/scripts/aero_lib.py` (mtime 07:20:49, from the campaign — the correction pass at
19:15–19:22 did not touch it).

### 9.5 New non-blocking findings

| # | Finding | Owner |
|---|---|---|
| m10 | §0 says "6-10 orders of magnitude below the ~47 N·m budget"; actual is **5.5 / 7.4 / 9.0** — run 2 is 5.5, not 6. Should read "5-9 orders". Non-material; even 5.5 orders is decisive. | `controls-integration` |
| i5 | The script derives a **second** quantity, not one: `q` is a 0.5 s central difference of Gazebo pose pitch (the timeseries publishes no `q`), and `cmRate = Cmq*q̂` is built from it. Documented in the docstring but, unlike `a_x`, carries no `ASSUMPTION` tag in the JSON. Immaterial (max `\|ΔMy\|` over the roll is 1.8e-04 N·m); the tag should be symmetric. | `controls-integration` |
| i6 | §0's comparison of `max\|cmRate\| = 0.451-0.455` against `min(Cm) = 0.486` is loose — the large `cmRate` values occur at near-zero airspeed where `q̂ = qc/2V` blows up but `qbar → 0`, so `qSc*cmRate ∝ V` and the moment stays tiny. The comparison is **conservative** and the underlying warning is valid. No action. | — |
| i7 | The new script lives in `tests/gazebo/scripts/` (a `gazebo-testing`-owned tree) but is self-declared `Owner: controls-integration`. Declared, not hidden. `gazebo-testing` should formally adopt or acknowledge it. | `gazebo-testing` |

No CG-duality violation: the script uses the Gazebo/CAD CG `(0.168309, 0, 0.100000)` throughout with
correct provenance; the XFLR5 reference CG appears nowhere in it.

### 9.6 Workflow ruling — no further round required

`controls-integration` flagged that per CLAUDE.md the M2 sign correction is a "non-trivial change"
that should route to `gazebo-testing` then `validation` before closure. `validation`'s ruling: **no
further round is required.**

The CLAUDE.md rule gates *implementation* changes. M2 changed **no implementation file** — git proves
`model/`, `plugins/`, `config/` and `docs/source_of_truth/` are untouched. M2 corrected a
*description of* `AeroModel.hh` in a test report; `AeroModel.hh` was always right. Step 1 is
therefore vacuous: no simulation input changed, so a re-run would be bit-identical by construction
and would produce no information. Step 2 is the confirmation pass recorded here, performed against
`AeroModel.hh:705` and `:316` directly rather than against `validation`'s own prior text.

`validation` placed one qualification on the record: **"the correction was specified by `validation`"
is not itself a valid reason to skip review.** M5 is the proof — `validation` specified a swap that
was wrong, and the correct outcome was for `controls-integration` to refuse it. The reason no further
round is needed is narrower and purely structural: nothing testable changed. `controls-integration`'s
instinct to flag it was good practice, not overcaution.

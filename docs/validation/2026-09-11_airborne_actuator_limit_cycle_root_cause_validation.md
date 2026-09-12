# VALIDATION — AIRBORNE_ACTUATOR_LIMIT_CYCLE_ROOT_CAUSE (2026-09-11)

Reviewer: `validation` (read-only). Persisted by the main session verbatim from the agent's return —
`validation` has no Write/Edit tool by design and edited nothing.

Audited artifacts:
- `docs/test_results/2026-09-10_airborne_actuator_limit_cycle_root_cause.md` (`controls-integration`, offline)
- `docs/test_results/2026-09-11_airborne_actuator_limit_cycle_runtime_verification.md` (`gazebo-testing`, live)
- `tests/gazebo/scripts/analyze_airborne_actuator_limit_cycle.py`, `tests/gazebo/results/airborne_actuator_limit_cycle_analysis.json`
- `tests/gazebo/scripts/{run_airborne_limit_cycle_runtime_verification.sh,record_actuator_highrate.py,analyze_airborne_limit_cycle_runtime.py}`
- `tests/gazebo/results/airborne_limit_cycle_runtime_*.json`
- `tests/gazebo/models/falcon_v2_highrate_diag/model.sdf`, `tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_highrate_world.sdf`

## Plain answers

**Root-cause classification: SUPPORTED**, with one qualification. `ACTUATOR_CONTROL_LIMIT_CYCLE` is correct
and the alternatives are refuted by direct measurement, not preference. `MIXED` is correctly rejected.
**However, the specific *driving mechanism* asserted in §4 item 2 is not established and is quantitatively
implausible as written** (finding M3). The classification survives because it does not depend on which
hinge-torque term dominates.

**Blocker severity CRITICAL: JUSTIFIED.** Reproduced independently from the raw 1 kHz records: 44–85 %
rate-pinned duty, 34–43° uncommanded p2p, `δe` p2p 38–43° driving `Cm` p2p ≈ 0.96 and 126–148 °/s RMS pitch
rate. Any airborne dynamic measurement from these records is meaningless.

**Both agents asserted beyond their data** in several places — findings M2, M3, m5, m6, m7, m12, m13, m14.

## What was verified as correct (passes)

- **CG duality: clean.** `model/model.sdf` base_link uses the Gazebo/CAD CG `(0.168309, 0, 0.100000)` and
  explicitly documents that the XFLR5 CG is never used for SDF purposes. IMU/GPS poses use the same CG.
  No interchange anywhere in this stage.
- **Command → actual causality: ESTABLISHED, not merely consistent.** Verified against raw records, not
  prose. All four runs, `per_surface_airborne`: MANUAL has `cmd_unique_values = 1`, `cmd_p2p_deg = 0.0` on
  all five surfaces, and `SERVO_OUTPUT_RAW` C1/C2/C4 `unique: 1` at 1500 µs across 5091/5105 samples.
  Independent recomputation from the 1 kHz excerpt gives `cmd_uniq = 1` on all five while the four `+Y`
  joints swing 33.8–34.1° at exactly 300.0007 °/s. This is bit-level, and it is the strongest evidence in
  the stage.
- **Provenance chain closes exactly.** `kp = I_ref·ω_n² = 3.4788e-07 × 100² = 0.0034788` (exact);
  `kd = 2ζω_n I_ref = 6.9576e-05` (exact); `ki = kp/Ti = 6.9576e-03` (exact); roots of
  `I s³ + kd s² + kp s + ki` = **−113.287, −84.627, −2.086**, matching the documented −113.29, −84.63,
  −2.09. `I_ref` equals `model/model.sdf` `left_aileron <iyy>` exactly; link mass `0.001 kg` is labelled
  `TEMPORARY_NUMERICAL_MASS`. The `placeholder_inertia_note` quote is verbatim, not paraphrased.
  Restoring-torque arithmetic (6.07e-5 N·m/deg; 0.46 %; 0.53–0.71 %) all reproduce.
- **Zero tracked engineering files modified.** `git diff --stat HEAD` is empty across
  `model/ plugins/ config/ docs/source_of_truth/ tests/`. Timestep, physics, gains, aero, mass and mapping
  all untouched.
- **Every published runtime number is derived under the corrected clock mapping.** The rejected affine
  fit's `coef` populates only the diagnostic dict (lines 164–165) and is never applied to any data.
- **FLU pitch-axis sign is handled correctly** — see the trap in T1 below.
- **Liftoff artefact confirmed** as the same documented no-landing-gear belly-contact detector limitation.
  The runtime pass uses its own airborne window (`z ≥ 1.0 m`, sensitivity at 0.5/1.0/5.0 m), so the
  detector feeds no published number.

## Findings

### M1 — MAJOR — The primary recommended fix rests on a false mass-dependence → `controls-integration` + `geometry-structure`

The offline report §7 item 1 calls replacing `TEMPORARY_NUMERICAL_MASS` and re-deriving `kp/kd/ki` "the
primary item; everything below is secondary to it," and says "the servo bandwidth is pinned by a fictitious
1-gram link."

The quasi-static deflection under an acceleration load through the link CoM offset is

```
θ_eq = m·a·d / kp = m·a·d / (m·k_g²·ω_n²) = a·d / (k_g²·ω_n²)
```

**This is independent of link mass.** Re-deriving the gains against a real inertia at the same
`ω_n = 100 rad/s` changes the deflection by exactly zero. Bandwidth `ω_n` is a design choice bounded by the
1 kHz timestep, not something "pinned by" the link mass.

Cross-validated against an independent pre-existing measurement: the model predicts
`T_gravity = m·g·d = 2.66e-4 N·m` against the config's documented `~1.6–2.5e-4 N·m`, and a gravity droop of
**4.38°** against the 2026-08-24 measured **2.6–4.2°**. At `a = 40 m/s²` it predicts **±17.9°**, against the
measured ±19°. The model closes on three independent data points.

The real levers are geometry (`d`, `k_g` — `geometry-structure`), bandwidth `ω_n` (bounded by
`max_step_size`), and real hinge damping — **not** mass. A fix stage that replaces the mass and re-derives
at `ω_n = 100` will find the defect unchanged.

### M2 — MAJOR — "There is no measured actuator τ ≈ 0.25 s in this repository" is false → `controls-integration`

The user's recollection is grounded. `tests/gazebo/results/closed_loop_control_surface_behavior_analysis.json`,
segment 0:

- `left_aileron.first_order_tau_s_2param = 0.240018552563389`
- `right_aileron.first_order_tau_s_2param = 0.2397643902376466`

The offline report searched only `docs/` (it says so) and never searched `tests/gazebo/results/`. It then
asserted the figure was "almost certainly" a confusion with `RLL_RATE_P` — an assertion beyond its evidence,
and wrong.

The substantive conclusion is unaffected, and the caveats matter: both aileron fits carry
`tau_fit_reliable = False` with r² ≈ 0.13, and all 11 aileron segments scatter 0.15–2.94 s. But the
*reliable* fits (all 11 elevator segments, `tau_fit_reliable = True`) are **bimodal: 0.058–0.069 s and
0.298–0.343 s** — an amplitude-dependent time constant, which is the classic signature of a rate-limited
actuator. This is corroborating evidence for the stage's own conclusion that the report missed by not
searching results. Q9 and the `DATA_REQUIRED` marker both need correction.

### M3 — MAJOR — §4 item 2's asserted driver is not established and is ~50× too weak → `controls-integration`

§4 is headed "**Measured, not inferred**" and item 2 asserts "**The driver is base pitch angular
acceleration**." What was measured is a correlation and a binning, not a torque.

Order of magnitude: inertial reaction to base angular acceleration is `I·α_y = 3.4788e-07 × 66 = 2.3e-5 N·m`,
against a restoring torque `kp·θ = 1.2e-3 N·m` at the observed ±19°. **It is ~50× too small** to produce the
observed excursion. The CoM-offset term `m·a·d` (with `d_perp = 0.0271 m` from the aileron's hinge-to-CoM
offset) is an order of magnitude larger and closes the amplitude, as shown in M1.

The report's own closing paragraph concedes the (a)/(b)/(c) split "cannot be separated from the existing
records and no split is asserted" — which directly contradicts the section heading. The `DATA_REQUIRED`
marker is correctly placed; the heading overstates it. The runtime report inherits the phrasing in §2.4.

### M4 — MAJOR (original claim) / INFO (adjudication) — D1 adjudicated in `gazebo-testing`'s favour, more strongly than it argued

The offline "every pinned count here is a **LOWER BOUND**" is **refuted**, and §1.1/§Q4 need correction.

`gazebo-testing`'s refutation is correct but under-argued: its decimation test uses a single phase offset
(`av[::step]` from index 0), so "−3.85 to +1.52 pp" is four single draws. Sweeping **all 50 phases** on the
1 kHz excerpts:

| run / surface | 1 kHz truth | 20 Hz phase-sweep mean | bias | SD | range |
|---|---|---|---|---|---|
| MANUAL aileron | 47.50 % | 47.50 % | **+0.00 pp** | 1.55 | 5.00 |
| MANUAL elevator | 51.72 % | 51.72 % | **+0.00 pp** | 1.10 | 5.00 |
| FBWA aileron | 73.87 % | 73.87 % | **+0.00 pp** | 1.27 | 5.00 |
| FBWA elevator | 85.02 % | 85.02 % | **+0.00 pp** | 1.26 | 4.17 |

The 20 Hz duty estimator is **provably unbiased**, not merely "scatter-limited" — systematic point-sampling
of a duty fraction recovers the exact measure on phase-average. The large per-phase scatter is because 20 Hz
samples a ~3.97 Hz cycle at nearly 5:1, i.e. near phase-locking.

**D1b is correct and is a mathematical certainty**, not an empirical finding: a subsample's max−min can never
exceed the full set's. Verified `True` in all 200 phase draws. Amplitude quoted from 20 Hz records genuinely
is a lower bound.

### m5 — MINOR — The "+0.0007 °/s overshoot" is a unit-rounding artifact, not solver tolerance → `gazebo-testing`

§2.1 states the joint "does slightly overshoot its own limit, by a fixed +0.0007 °/s ... it is the solver's
constraint tolerance."

`5.2360 rad/s × 180/π = 300.00070153 °/s` exactly. The configured limit *is* 300.0007 °/s; exactly
300.0000 °/s would be 5.23598776 rad/s. `gazebo-testing`'s own JSON stores
`max_rate_deg_s: 300.00070153049904`. **There is no overshoot** — the joint hits its limit exactly and never
exceeds it. A rounding artifact was attributed to solver physics.

### m6 — MINOR — `peak_variance_fraction` does not support "not a pure tone" → `controls-integration`

Offline §Q8 argues "the airframe signal is not a pure tone (`peak_variance_fraction` 0.31–0.59)." Running
their own estimator on known signals:

| signal | `peak_variance_fraction` |
|---|---|
| perfectly pure 3.97 Hz sinusoid | **0.527** |
| ideal 3.97 Hz triangle | **0.519** |

A pure tone scores ~0.53 under a Hann window with single-bin power and no peak interpolation. So 0.52–0.59 is
*consistent with* a pure tone; only the 0.31 end indicates real spreading. The inference is unsupported by
that metric.

### m7 — MINOR — Over-precision in quoted frequencies → `gazebo-testing`

`dom_freq()` returns raw FFT bin centres with no peak interpolation. Over 52.5 s, Δf ≈ 0.019 Hz; on a
synthetic 3.97 Hz tone the estimator returns 3.9619 Hz (0.008 Hz error). Quoting baseline vs high-rate
agreement as "3.9657 / 3.9656 Hz" (0.1 mHz) is meaningless — they agree because they land in the same bin.
For MANUAL the whole-segment peak is not a well-defined quantity at all (see I10).

### m12 — MINOR — "differs on exactly two lines, verified by machine diff" is literally inaccurate → `gazebo-testing`

The real diff is `1461c1497` and `1595c1631` plus a 36-line comment header — not the quoted
`1460c1460 / 1594c1594`. The world likewise differs by the `<uri>` line **plus** a header. The substance is
verified correct: two functional lines, propulsion left at 20.0, world `<name>`/gravity/`max_step_size`/spawn
pose identical, and `meshes` is a symlink to the real `model/meshes` so no geometry can diverge. The copy's
own header discloses the comment block; the report's diff excerpt does not.

### m13 — MINOR — Observability-only argument covered only one of the two changed plugins → `gazebo-testing`

The argument cites `ActuatorSystem.cc::PreUpdate()` only. The second changed line is the **aerodynamics**
plugin. Verified independently that `AerodynamicsSystem.cc` also computes `ComputeAero()` and applies
`AddWorldForce`/`AddWorldWrench` every step (lines 349–363), throttling only `Publish()` (lines 365–385).
**The claim holds for both lines** — but the report did not establish it, and if the aero plugin had gated
force application on the diagnostics rate the high-rate numbers would have been contaminated.

### m14 — MINOR — "residual 0.0 s" is a tautology → `gazebo-testing`

`np.interp` evaluated at its own nodes returns the node values, so `interp_residual` is 0 by construction
except where `np.maximum.accumulate` altered the axis. The JSON confirms `rms = 0, max = 0` exactly,
alongside the genuine rejected-affine residual of 13.27 s. The correction to monotone interpolation is the
right call, but 0.0 s is not evidence of its accuracy. The real uncontrolled term — the per-topic transport
latency difference between `/clock` and the actuator topic — is unmeasured.

### m19 — MINOR — Transcription: §2.1 says 378 unique `cmd` values for the high-rate FBWA elevator; the JSON says 376 → `gazebo-testing`

### I8 — INFO (resolved) — D2 is an estimator mismatch, not a discrepancy

`gazebo-testing` recorded the 15–30× roll-rate gap as "not resolved." It resolves. The offline statistic is
`airborne_gyroX_window_rms_median_rad_s` — the **median of per-0.5 s-window RMS**. The runtime statistic is
`p_roll.rms_deg_s` — a **whole-segment RMS**. Different estimators. On the *same* 6 s of FBWA data the two
differ by **9.8×**; the residual factor is plausibly the channel difference (ArduPilot dataflash IMU vs raw
Gazebo IMU). No conclusion is affected in either report.

### I9 — INFO (resolved) — D3: `f/f_pred > 1` is structurally impossible and is a scoping artifact

`f = R/(2P)` is an upper bound for a rate-saturated triangle; exceeding it requires `|θ̇| > R`, which never
occurs. Values of 1.049/1.084 arise because the 1 s-window `P` is inflated by mean-wander, which makes
`f_pred` too small — and the maximum occurs in `highrate_MANUAL`, exactly the run whose mean wanders. Neither
"never above 1.016" nor "max 1.084" is a physical result. The qualitative conclusion (median 0.81–0.97, below
1) stands; the ratio should not be quoted to three decimals by either agent.

### I10 — INFO (resolved) — The MANUAL 3.966 vs 4.689 Hz gap is non-stationarity, not a discrepancy

In a common 6 s window, joint angle, body pitch rate `q` and `Cm` peak in the **same bin** — 4.5 Hz in
MANUAL, 4.0 Hz in FBWA. The whole-segment mismatch is because MANUAL's frequency drifts across the phugoid
(V ranges ~19–37 m/s within the airborne segment), smearing the peak. This *supports* the closed-loop story
the runtime report left unexplained.

### I11 — INFO — Independent corroboration of the rate-saturated triangle model

The FBWA joint-angle spectrum shows a 3rd harmonic at 11.833 Hz at **10.5 %** of the fundamental amplitude.
An ideal triangle wave's 3rd harmonic is exactly 1/9 = 11.1 %. This is direct spectral evidence that the rate
limit shapes the waveform — stronger than the `f ≈ R/2P` ratio argument either report used.

### I15 — INFO — Actuator timestamps are reconstructed, not publisher-stamped

`gz::msgs::Double_V` carries no header, so `record_actuator_highrate.py` stores only `t_wall` for
actuator/aero; only the IMU has a true `t_stamp_s`. Measured non-uniformity after mapping:
`dt ∈ [0.000131, 0.001922] s`. Impact: duty, p2p and clamp counts are value-based and **immune**; frequency
is essentially immune. The correlations cross two time bases (IMU header stamp vs reconstructed) — a
systematic offset would depress `|corr|` but cannot flip its sign, so the `+Y`/`+Z` sign contrast is safe.

### I18 — INFO — The headline severity numbers derive from an unsourced constant

`max_rate_rad_s = 5.2360` is `V1_PROVISIONAL` with no ES08MAII datasheet (`DATA_REQUIRED`). Since
`f ≈ R/2P`, **both** the ~4 Hz frequency and the 35–43° amplitude are set by it. The defect is real and
reproducible; its quantitative magnitude is provisional. Likewise the `EFFORT_LIMIT_INTERACTION` refutation
is conditional on `max_effort_nm = 0.20 N·m` (also `V1_PROVISIONAL`) — though it is robust, since measured
torques are ~0.7 % of it.

## The four named items — verdicts

1. **Command → actual causality: ESTABLISHED.** Bit-level, verified against raw records and the result JSON,
   not the prose. Strongest evidence in the stage.
2. **Frequency/amplitude: CONSISTENT, but over-precise.** Offline 3.72–4.53 Hz spans 9 runs at 3 modes and
   airspeeds; runtime 3.966–3.997 Hz spans 2 high-rate runs — nested, not contradictory. Nyquist arguments
   are sound for the fundamental (the 3rd harmonic at ~11.9 Hz does alias in the 20 Hz records, but carries
   ~1 % of power, and the 1 kHz spectral census bounds it at 0.85–1.23 % in 10–50 Hz). Windowing is Hann,
   correctly applied. Defects: m6, m7, I9, I10.
3. **Clamp interpretation: the inference is SOUND.** With no rate-clamp flag, "pinned" is correctly defined
   as an *observation* (`|actual_rate| ≥ 0.999 × 5.2360`), never as a flag, in both reports — this is the
   right call and both are disciplined about it. `DATA_REQUIRED` is correctly declared for both the applied
   joint torque and the rate-clamp flag; confirmed at `ActuatorSystem.cc:309–317` that `eff.torqueNm` is
   applied via `SetForce` but is **not** among the 7 published fields. `SetVelocityLimits` is confirmed at
   line **127**, exactly as cited. The "lower bound" characterisation is the one thing wrong here (M4), and
   the "overshoot" gloss (m5).
4. **Correlation: REAL, sign-correct, but mislabelled and direction-agnostic.** Numbers reproduced (rate vs
   `α_y`: −0.81…−0.92; rudder +0.04…+0.09). The signs are consistent with correct FLU handling (T1). **But
   the prose is loose**: it is the joint *rate* that tracks `α_y` and equivalently the joint *angle* that
   tracks `q` (−0.85…−0.96). The joint *angle* does **not** track `α_y` — measured only −0.19 to −0.52. The
   runtime report's tables are right; §2.4's sentence "the joints track the base pitch angular acceleration
   almost perfectly" is not. Separately, **a correlation cannot establish causal direction** — in a closed
   feedback loop it appears regardless of which end initiates. The directional claim rests on the other
   evidence (the `+Y`/`+Z` discriminator, the zero-command case, the ignition ordering), which is strong; the
   correlation is not independent support for it.

## What a future fix stage must NOT take on faith

- **T1 — the `Cm` diagnostic is not the pitching moment.** `AeroModel.hh` computes
  `my = qbar·S·c_ref·(−cmStatic + cmRate)` while `out.Cm = cmStatic + cmRate` is published in XFLR5's
  **unflipped** convention. Computing `My = +Cm·q̄·S·c_ref` from the published channel gives the **wrong
  sign**. Magnitude closure confirmed (|α_pred|/|α_meas| = 1.26–1.34) and the negative `Cm`↔`α_y`
  correlation is exactly what correct FLU handling predicts. Neither report misused this, but neither
  flagged it.
- **M1 — replacing the placeholder mass will not fix this by itself.** `θ_eq = a·d/(k_g²·ω_n²)` is
  mass-independent.
- **M3 — the driving hinge torque is still unmeasured.** Do not treat "base pitch angular acceleration" as
  the established driver.
- **M4 — do not build any argument on "the true duty must be higher than this."** The 20 Hz estimator is
  unbiased.
- **I18 — do not treat 4 Hz / 35–43° as physical constants of the airframe.** They are set by an unsourced
  `V1_PROVISIONAL` rate limit.
- **I9 — do not carry `f/f_pred` precision forward** in either direction.

## Routing

| To | Items |
|---|---|
| `controls-integration` | M1 (with `geometry-structure`), M2, M3, M4 (correct §1.1/§Q4 "lower bound"), m6 |
| `gazebo-testing` | m5, m7, m12, m13, m14, m19 |
| `geometry-structure` | M1 — real per-surface CoM offset `d` and radius of gyration `k_g`; note the placeholder `k_g² = 3.4788e-4 m²` (k_g = 18.7 mm) sits inconsistently against the placeholder CoM offset `d = 27.1 mm`, which implies a larger `k_g` for any plausible surface geometry |
| `aerodynamics` | T1 — document that the published `Cm` diagnostic is unflipped and is not the `My` driver |

No finding rises to `CRITICAL`. Nothing in this stage changed an engineering parameter, and no defect found
produces wrong physics *in the simulation* — the defects are in characterisation, provenance of a negative
claim, and a fix recommendation that would not work. The `CRITICAL` blocker severity that both reports assign
to the *underlying defect* is separately justified and concurred with.

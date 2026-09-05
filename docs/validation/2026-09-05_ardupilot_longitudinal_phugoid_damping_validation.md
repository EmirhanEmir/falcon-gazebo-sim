# STAGE RECORD — ARDUPLANE_LONGITUDINAL_PHUGOID_DAMPING_VALIDATION (2026-09-05)

Stage documentation artifact. Written by `controls-integration` (stage primary) after
`gazebo-testing` executed the campaign and `validation` completed its independent read-only
review.

**Authorship convention for this file.** Unlike `docs/validation/2026-09-02_*.md` and
`docs/validation/2026-09-03_*.md`, which are `validation`'s own review reports, this file is the
*stage record*: it consolidates the measurement, the root-cause derivation and the declared
limitations. Every claim that originates with `validation` is attributed inline as
"(`validation`, independent)". Every claim that originates with the test harness is attributed to
the artifact it came from. Nothing in this file was taken on trust from a summary; each number
cited below was re-read out of the named JSON artifact or re-derived from ArduPlane source while
writing it.

---

## 0. Bottom line

| | |
|---|---|
| Stage verdict | **PASS**, 0 stage-blocking findings (`validation`) |
| Wrong physics found (sign / frame / unit / duplicated force or damping / CG conflation) | **NONE** |
| Forbidden parameter changed | **NONE** — `config/ardupilot/falcon_v2_sitl.parm` md5 `180e56711bdf18c658f6ded8031421f1`, unchanged; it sets no `TECS_*` value |
| Root cause of the weak longitudinal damping | **IDENTIFIED and independently verified in `AP_TECS.cpp`** — see §2 |
| Is the observed mode the bare-airframe phugoid? | **NO.** It is a TECS-generated closed-loop energy mode — see §3 |
| `TECS_PTCH_DAMP = 0.6` | **VALIDATED_NOT_ADOPTED.** Measured, corroborated by four independent estimators, no performance regression — but **NOT written to any parameter file by this stage**. Adoption gate: §8 |
| Carried MAJOR `CLOSED_LOOP_LONGITUDINAL_DAMPING_WEAKER_THAN_FREE_AIRFRAME` | **STAYS OPEN** at firmware defaults. Its magnitude is corrected downward from 1.24x to ~1.06–1.13x — see §4 |
| Parameter/physics changes made by this stage | **NONE.** No `TECS_*`, no PID, no `PTCH_TRIM_DEG`, no aero, no propulsion, no actuator, no sensor, no mass/CG/inertia, no `.parm` |

**Scope constraint enforced.** `TECS_PTCH_DAMP = 0.6` exists in this repository only as a
*runtime* `PARAM_SET` issued by the test harness for the duration of two measurement runs. It is
recorded here as `VALIDATED_NOT_ADOPTED`. `docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml:183`
still correctly records the firmware default `TECS_PTCH_DAMP = 0.3`, sourced to
`libraries/AP_TECS/AP_TECS.cpp:107`, and was not edited. `validation` was explicit that the
runtime-flag-only state is the CORRECT state and that documentation does not yet support adoption.

---

## 1. Configuration under test

| Item | Value | Source |
|---|---|---|
| Firmware | ArduPlane V4.8.0-dev, ArduPilot commit `409226a637` | `/home/emirhan/gazebo_sim/ardupilot`, `ArduPlane/version.h:10` |
| Mode | FBWB, `custom_mode = 6` | `ArduPlane/mode.h:45` |
| TECS parameters | firmware defaults throughout, except the single runtime `TECS_PTCH_DAMP` write in the candidate runs | `tecs_baseline_params_live` in every result JSON |
| Wind | zero | `SIM_WIND_SPD/DIR/TURB` read live |
| `AHRS_EKF_TYPE` | 10 (SITL EKF), read live | `analysis.tecs_energy_loop_gains.ahrs_ekf_type_live` |
| Excitation | ONE full-up FBWB pitch-stick pulse, RC2 1500 → 1900 µs, 4.0028 s, then RC2 back to trim for the whole ring-down | `analysis.excitation`, `analysis.phase_plan` |
| Ring-down | 65 s segment; analysed window `t_seg >= 10 s` → **54.951 s** of free decay with CONSTANT height and airspeed demands | `analysis.ringdown.primary_window_span_s` |
| Why the demands are constant | the pitch stick passing back through zero calls `set_target_altitude_current()`, locking the height demand to the current altitude | `ArduPlane/navigation.cpp:418-424`, `ArduPlane/altitude.cpp:191-195` |

Test artifacts (all absolute paths):

- `/home/emirhan/Desktop/FalconV2/tests/gazebo/scripts/test_ardupilot_longitudinal_phugoid_damping.py`
- `/home/emirhan/Desktop/FalconV2/tests/gazebo/scripts/run_ardupilot_longitudinal_phugoid_damping.sh`
- `/home/emirhan/Desktop/FalconV2/tests/gazebo/scripts/test_ardupilot_tecs_ptch_damp_regression.py`
- `/home/emirhan/Desktop/FalconV2/tests/gazebo/scripts/run_ardupilot_tecs_ptch_damp_regression.sh`
- `/home/emirhan/Desktop/FalconV2/tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_result_{baseline,ptchdamp06}_reanalyzed.json`
- `/home/emirhan/Desktop/FalconV2/tests/gazebo/results/ardupilot_tecs_ptch_damp_regression_result_{defaults,ptchdamp06}.json`

---

## 2. ROOT CAUSE — derived from `AP_TECS.cpp`, independently verified

`validation` re-derived this from the firmware source at commit `409226a637` and confirmed that
**every cited line resolves**. The lines were re-read again while writing this record.

### 2.1 The algebra

During the ring-down the height demand is locked and the airspeed demand is constant, therefore
`_hgt_rate_dem = 0` and `d(SEB_dem)/dt = 0`.

Let `S = SEB_error = SEB_dem - SEB_est` (`AP_TECS.cpp:1033`). Then:

```
SEBdot_dem   (:1036) = _hgt_rate_dem*g*SPE_w + S/Tc          =  S/Tc          (since _hgt_rate_dem = 0)
SEBdot_est   (:1050) = _SPEdot*SPE_w - _SKEdot*SKE_w
SEBdot_error (:1051) = SEBdot_dem - SEBdot_est               =  Sdot + S/Tc   (exactly)
```

The last step uses `Sdot = d(SEB_dem - SEB_est)/dt = -SEBdot_est`, which holds only because the
demand is constant — that is why the campaign is built around a locked-demand free decay.

Substituting into the pitch-demand numerator:

```
SEBdot_dem_total (:1062) = SEBdot_dem + SEBdot_error * pitch_damp
                         = S*(1 + Kd)/Tc + Kd*Sdot
_integSEBdot     (:1086, :1095) = INTEGRAL[ Ki*(Sdot + S/Tc) ] dt
                         = Ki*S + (Ki/Tc)*INTEGRAL[S] dt
_pitch_dem_unc   (:1108) = (SEBdot_dem_total + _integSEBdot + _integKE) / (TAS*g)      [gainInv at :1065]
```

so the gains the mode actually sees are

```
Kp_eff = (1 + TECS_PTCH_DAMP)/TECS_TIME_CONST + TECS_INTEG_GAIN     [1/s]
Kd_eff = TECS_PTCH_DAMP                                             [-]
Ki_true = TECS_INTEG_GAIN / TECS_TIME_CONST                         [1/s^2]
```

### 2.2 Four consequences, each one a finding

**(a) `TECS_INTEG_GAIN` is damping-DEGRADING here.** `_integSEBdot` integrates a signal that
already contains a *derivative* term. The integral of a derivative returns **pure proportional
stiffness**: `Ki*S`, with **no phase lead**. At firmware defaults that term is `0.30 / 0.56 =
53.6 %` — roughly 54 % — of the total loop stiffness, contributed with zero damping. This is the
single largest reason the loop is stiff but under-damped.

**(b) `TECS_PTCH_DAMP` is the ONLY derivative term in the pitch/energy loop.** No other term in
the `_pitch_dem_unc` numerator carries `Sdot`. `_integKE` (`:1096`) integrates
`(SKE_est - SKE_dem)*SKE_w` with gain `1/timeConstant()` — **not** `TECS_INTEG_GAIN` — and adds
phase **lag**, not lead.

**(c) `TECS_THR_DAMP` is near-blind to this mode.** It multiplies `STEdot_error` (`:740`), the
**total** energy-rate error, after a 0.5 s first-order filter (`:744-746`). An energy-**balance**
oscillation has `SPEdot ≈ -SKEdot`, so `STE_error` and `STEdot_error` are ≈ 0 through it. The
throttle loop cannot damp the mode even though throttle is nowhere near saturation.

**(d) `TECS_SPDWEIGHT = 1.0` is architecture, not a damping knob.** It sets
`w_SPE = w_SKE = 1.0` (`:1003`, `:1024`, `:1027-1028`), which is exactly what makes SEB the *pure*
energy-balance coordinate. Changing it would change what is being controlled, not how well it is
damped.

### 2.3 `TECS_HGT_OMEGA` is a DEAD CODE PATH in this configuration

`_hgtCompFiltOmega` appears at exactly three places — `AP_TECS.cpp:357`, `:359`, `:367` — and all
three are inside the `else` of `if (_ahrs.get_velocity_NED(velned))` at **`:344`**. With an EKF
vertical velocity available (`AHRS_EKF_TYPE = 10`, read live) that branch **never executes**;
`_climb_rate = -velned.z` is taken at `:346`, and `_height` comes from
`get_relative_position_D_home` at **`:327-328`**. `TECS_HGT_OMEGA` therefore cannot influence this
result and is not a candidate fix.

**Two cosmetic citation corrections recorded (`validation`, independent).** The result artifacts
carry the earlier, slightly off citations `_height` at `:330-331` and the EKF branch at
`:343-345`. The correct lines are `_height` at **`:327-328`** and the EKF branch condition at
**`:344`**. Both were re-verified against the source while writing this record. Neither affects
any number, conclusion or gate; they are recorded so the citation trail is exact.

### 2.4 Loop-gain model corroboration — REPORT_ONLY, never gated

The report-only diagnostic `analysis.tecs_energy_loop_gains` linearises the loop on the
constant-total-energy manifold (`SPEdot = -SKEdot`, on which `S = -2*g*dh`) — tagged
`ASSUMPTION: TECS_SEB_MANIFOLD_LINEARISATION`. It predicts a closed-loop energy-mode frequency
`omega ≈ 2*Kp_eff`:

| | firmware defaults | `TECS_PTCH_DAMP = 0.6` |
|---|---|---|
| `Kp_eff` [1/s] | 0.5600 | 0.6200 |
| `Kd_eff` [-] | 0.3000 | 0.6000 |
| fraction of `Kp_eff` from `TECS_INTEG_GAIN` | 0.5357 | 0.4839 |
| predicted `omega` [rad/s] | 1.1200 | 1.2400 |
| predicted period [s] | **5.610** | **5.067** |
| MEASURED period [s] | **5.6474** | **4.7317** |
| model error | **0.7 %** | **6.5 %** |
| PD corner `Kp_eff/Kd_eff` [rad/s] | **1.8667** | **1.0333** |
| MEASURED mode `omega_d` [rad/s] | **1.1126** | **1.3211** |
| PD corner vs mode | corner is **ABOVE** the mode | corner is **BELOW** the mode |

**The regime change is the point.** At firmware defaults the PD corner (1.8667 rad/s) sits *above*
the mode (1.1126 rad/s), so to that mode the loop looks essentially **proportional**: it stiffens
without damping. At `TECS_PTCH_DAMP = 0.6` the corner (1.0333 rad/s) drops *below* the mode
(1.3211 rad/s), so the PD pair delivers **real phase lead at the mode frequency**. A 0.7 %
first-principles prediction of the baseline period from firmware gains alone is strong evidence
that the identified mechanism is the actual mechanism.

This diagnostic is `REPORT_ONLY_DIAGNOSTIC`, `never_gated: true` in the artifact, and must stay
that way: it rests on a linearisation assumption.

---

## 3. MODE IDENTIFICATION — the observed mode is NOT the bare airframe phugoid

This reframes the carried MAJOR. The thing that was measured as "1.24x slower than the free
airframe" is a **TECS-generated closed-loop energy mode**, not a free-airframe eigenvalue.

### 3.1 The decisive evidence

**A pure TECS gain change moved the period.** With the aircraft, mass, CG, inertia, aerodynamic
tables, propulsion tables, actuator model and every PID byte-identical, and with exactly one
FUNCTIONAL parameter different, the mode period moved

```
5.6474 s  ->  4.7317 s     (-16.21 %)
```

(raw all-extrema periods, both runs, before any SNR truncation — so this comparison is
truncation-independent.) **A controller gain cannot move a free-airframe eigenvalue.** If the
observed mode were the bare phugoid, its period would have been invariant to `TECS_PTCH_DAMP`.

**Estimator-free corroboration (`validation`, independent).** `validation` computed zero-crossing
periods on the high-SNR early part of the ring-down, using no envelope estimator, no extremum
detection and no truncation:

| window | firmware defaults | `TECS_PTCH_DAMP = 0.6` |
|---|---|---|
| first 16 s | 5.58 s | 4.67 s |
| first 20 s | 5.68 s | 4.54 s |
| first 25 s | 5.65 s | 4.68 s |

A robust **≈ −18 %** shift, stable across all three windows, from a method that shares no code with
the harness's estimator.

### 3.2 Two caveats recorded honestly (`validation`)

**(i) The DFT confirmation is weak on its own.** Over a 55 s window the frequency bin spacing is
`1/55 = 0.01818 Hz`; at the mode frequency `f = 1/5.647 = 0.1771 Hz` that maps to a period
resolution of `df/f^2 = 0.58 s` — **wider than the shift being claimed**. The DFT is consistent
with the shift but cannot by itself resolve it. The claim is carried by the full-window fit and by
the zero-crossing periods, not by the DFT.

**(ii) "The free-airframe reference itself was unchanged" is NOT evidence.** `T_ref` moved only
8.1193 s → 8.1153 s between the two runs, but `T_ref = pi*sqrt(2)*V/g` depends **only on V and g**.
Since both runs trimmed at essentially the same airspeed, its near-constancy is an arithmetic
near-tautology and proves nothing about the mode. It is recorded here so that no future reader
mistakes it for corroboration.

### 3.3 Consequence

The correct name for what the ring-down measures is *the FALCON V2 + ArduPlane-TECS closed-loop
longitudinal energy mode*, and it must not be described as "the phugoid" in future stages. The
free-airframe Lanchester phugoid remains the **reference** the closed-loop mode is compared
against, nothing more.

---

## 4. PRIOR-STAGE CORRECTION — the carried MAJOR's "1.24x" figure is superseded

`CLOSED_LOOP_LONGITUDINAL_DAMPING_WEAKER_THAN_FREE_AIRFRAME` was raised by the 2026-09-03 energy
stage with `tau_closed/tau_free = 1.24`. That figure is now superseded.

### 4.1 Decomposition

| step | `tau_closed` [s] | `tau_ref` [s] | ratio | what changed |
|---|---|---|---|---|
| 2026-09-03 as published | 25.9086 | 20.8947 | **1.240** | — |
| legacy estimator, this stage's 55 s free-decay data | 22.9135 | 20.8732 | **1.098** | **data + window** (55 s purpose-built free decay vs a 30 s settle tail) |
| corrected estimator, same data | 22.0741 | 20.8732 | **1.0575** | **estimator** (see §6) |
| `validation` independent full-window damped-sinusoid fit | 23.67 | 20.8732 | **1.134** | independent method |

### 4.2 The shift is data/window, not code — proven

The new harness's estimator module, run on **the prior stage's own raw timeseries**
(`tests/gazebo/results/ardupilot_tecs_climb_descent_energy_timeseries.json`, segment `P3_settle`,
window `t_seg >= 10 s`), reproduces the 2026-09-03 published numbers **exactly**:

```
T     = 5.632851 s     (published 5.632851)
zeta  = 0.0345816      (published 0.0345816)
tau   = 25.9086 s      (published 25.909)
n_extrema = 11, analysed span 29.939 s
```

This was re-run independently while writing this record. Because the legacy code reproduces the
legacy answer bit-for-bit on the legacy data, the 1.240 → 1.098 movement is attributable to the
**measurement** (a longer, cleaner, purpose-built free decay), not to any change in analysis code.

### 4.3 Ruling — the MAJOR STAYS OPEN

The true ratio is **~1.06–1.13** depending on estimator, and it is **> 1 under EVERY estimator
tried**. The closed loop still damps the longitudinal energy mode *more slowly* than the free
airframe would. The MAJOR is therefore **NOT closed**; only its magnitude is corrected downward.
It remains **non-gating** — gating a baseline measurement on the quantity the baseline exists to
measure would be circular and would invite exactly the tuning CLAUDE.md's simulation tuning policy
forbids.

---

## 5. MEASUREMENTS

### 5.1 Baseline — firmware defaults

`tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_result_baseline_reanalyzed.json`,
verdict `PHUGOID_DAMPING_BASELINE_MEASURED`, `failed_checks = []`, **35 / 35 acceptance checks
PASS**.

| Quantity | Value | Note |
|---|---|---|
| excitation | single 4.0028 s full-up pulse, RC2 1900 µs | pulse altitude gain 2.655 m, inside the 1.301–10.408 m admitted band |
| free ring-down analysed | 54.951 s | `t_seg >= 10 s`, constant demands throughout |
| period `T` | **5.6474 s** | |
| `zeta` | **0.039196** | legacy estimator |
| `tau` log-decrement | **22.9135 s** | legacy estimator |
| `tau` from `zeta` | **22.9312 s** | legacy estimator |
| `tau` LSQ envelope fit | **22.5931 s**, `r2 = 0.99212` | 20 extrema |
| **corrected `tau`** | **22.0741 s** | pooled mean-of-logs, §6 |
| **corrected `zeta`** | **0.040684** | |
| `omega_d` | 1.11259 rad/s | |
| `tau_ref` (Lanchester free airframe) | **20.8732 s** | `V = 17.9276 m/s`, `L/D = 11.4219` |
| `T_ref` | 8.1193 s | |
| **`tau` ratio closed/free** | **1.0575** | carried MAJOR indicator, non-gating |
| **period ratio closed/free** | **0.69555** | mode is 1.44x FASTER than the free phugoid → stiffened by the loop |
| window / `tau` | **2.489** | `measurability.window_over_tau`; = 2.398 if computed against the legacy `tau` 22.914 s. Gate `TH_MIN_WINDOW_TAU = 2.0` — PASS either way |
| extrema admitted | 20 of 20, **0 rejected** | SNR truncation is INERT on the baseline |

### 5.2 Candidate — `TECS_PTCH_DAMP = 0.6` (runtime `PARAM_SET` only)

`tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_result_ptchdamp06_reanalyzed.json`.

| Quantity | Value |
|---|---|
| corrected `tau` | **8.6516 s** |
| corrected `zeta` | **0.087160** |
| period `T` | **4.7561 s** (SNR-admitted) / 4.7317 s (raw all-extrema) |
| `omega_d` | 1.32109 rad/s |
| `tau_ref` | 20.8640 s |
| **`tau` ratio closed/free** | **0.41466** — now decaying **2.4x FASTER** than the free airframe |
| extrema admitted | 8 of 23 (15 rejected below the 3-sigma floor) |
| window / `tau` | 6.346 (full analysed span) |

**Reported honestly:** this run's own harness verdict is `PHUGOID_DAMPING_BASELINE_FAILED` with
`failed_checks = ['is_firmware_default_baseline']`. That is **by design and is the correct
behaviour** — the part-1 harness gates that the aircraft is flying at firmware defaults, and the
candidate run deliberately is not. It is not an aircraft or physics failure. Every other one of
the 35 checks passed on that run.

### 5.3 Improvement — four independent methods, most conservative first

| # | Method | defaults | 0.6 | improvement |
|---|---|---|---|---|
| 1 | harness pooled log-decrement `tau` | 22.0741 s | 8.6516 s | **2.55x** |
| 2 | estimator-independent 2nd-half / 1st-half residual std ratio (altitude) | 0.30524 | 0.10498 | **2.91x** |
| 3 | LSQ regression over admitted extrema `tau` | 22.5931 s | 7.1501 s | **3.16x** |
| 4 | `validation` independent full-window nonlinear damped-sinusoid fit (no extrema, no truncation) | 23.67 s | 6.70 s | **3.53x** |

Method 2 uses no extremum detection, no truncation and no logarithm — it is simply the ratio of
detrended residual scatter between the two halves of the same window — and it is the *most
conservative* of the four.

**Honest precision statement.** The damping improvement is **roughly 2.5x to 3.5x faster envelope
decay**. `validation` assessed the originally claimed `+/-20 %` uncertainty on `tau` as *slightly
optimistic* and ruled **`+/-25–30 %` defensible**. That is the figure of record. There is **n = 1
run per setting**, so no run-to-run scatter estimate exists for the damping measurement itself
(§7).

---

## 6. ESTIMATOR CORRECTION — a genuine defect in the analysis code, not a threshold change

### 6.1 The defect

The legacy `energy.damping_estimate()`
(`tests/gazebo/scripts/test_ardupilot_tecs_climb_descent_energy.py:713`, read-only, closed stage)
computes

```python
ratios = [amps[i + 1] / amps[i] for i in range(len(amps) - 1) if amps[i] > 1e-9]
r_half = mean(ratios)          # ARITHMETIC mean of successive amplitude ratios
```

The **arithmetic** mean of ratios is a **biased** estimator of exponential decay for **any**
dataset. By Jensen's inequality `AM >= GM`, so `r_half` is biased **high**, the logarithmic
decrement biased **low**, and `tau` biased **long** — i.e. the legacy estimator systematically
reports a system as *less damped than it is*. This is a defect in the estimator, independent of
which aircraft or which run it is applied to.

### 6.2 The replacement

The new harness replaces it with the **definitional pooled logarithmic decrement**

```
delta = (1/(n-1)) * SUM_i ln(A_i / A_{i+1})
```

(equivalently the geometric mean of the ratios), plus an **a-priori 3-sigma SNR truncation**: an
extremum enters the estimate only if its amplitude is at least `TH_SNR_DETECTION_MULTIPLE = 3.0`
times the **incoherent** noise floor of that channel, where the floor is measured from the run's
own data as the RMS remaining after removing a linear trend *and* the best-fit sinusoid at the
mode period over the last `TH_NOISE_TAIL_CYCLES = 3.0` periods. Tag:
`ASSUMPTION_EXTREMUM_SNR_DETECTION_THRESHOLD`. 3 sigma is the conventional limit of detection
(ISO 11843 / IUPAC). `DATA_REQUIRED` to remove the assumption entirely: an independently
characterised altitude measurement-noise spectrum for the Gazebo/SITL sensor chain.

### 6.3 No acceptance threshold was changed — verified

`validation` verified, and the artifact confirms:

- threshold key count **29 → 34**, with **0 CHANGED** and **0 REMOVED**;
- all 5 additions are inside a block explicitly named
  `ESTIMATOR_SETTINGS_NOT_ACCEPTANCE_THRESHOLDS`
  (`TH_SNR_DETECTION_MULTIPLE`, `TH_NOISE_TAIL_CYCLES`, `TH_MIN_ADMITTED_EXTREMA`, plus a `note`
  and an `assumption_tag`);
- the baseline acceptance dict is **bit-identical**, 35/35;
- the prior stage reanalyzes **byte-identical** (§4.2).

The legacy estimator is still imported **unchanged** and its result is retained in every artifact
under `corrected_log_decrement.legacy_arithmetic_mean_estimator`, so both numbers are visible side
by side and nothing was quietly replaced.

### 6.4 `validation` MAJOR-1, recorded honestly (non-blocking)

`validation` raised, and this record accepts, that the "baseline unchanged" argument validates the
**mean-of-logs** change but **NOT the truncation step**. The truncation is *inert on the baseline*
(0 of 20 extrema rejected) and *load-bearing on the candidate* (15 of 23 rejected), so a baseline
that is unchanged says nothing about it.

The truncation is instead justified on two independent grounds, both of which are recorded rather
than asserted:

1. the rejected tail is demonstrably **non-monotone floor noise** — the raw amplitude sequence
   stops decaying and starts wandering, which is what a floor looks like and is not a measurement
   of a decay rate;
2. **the conclusion survives without truncation**: the estimator-independent residual-ratio method
   (§5.3 method 2) uses no truncation at all and still gives 2.91x, and `validation`'s full-window
   fit uses no truncation and gives 3.53x.

The `snr_sensitivity` sweep over `TH_SNR_DETECTION_MULTIPLE ∈ {2,3,4,5}` and
`TH_NOISE_TAIL_CYCLES ∈ {2,3}` is recorded in full in both result artifacts, so the sensitivity is
visible rather than hidden. `gazebo-testing` is adding the corresponding explicit reporting to the
harness in parallel; `controls-integration` did **not** edit
`tests/gazebo/scripts/test_ardupilot_longitudinal_phugoid_damping.py` in this stage.

---

## 7. PERFORMANCE REGRESSION — `TECS_PTCH_DAMP = 0.6` costs nothing measurable

Two runs of the climb/descent/energy profile,
`tests/gazebo/scripts/test_ardupilot_tecs_ptch_damp_regression.py`, with **exactly one FUNCTIONAL
parameter differing**.

### 7.1 Anti-tuning provenance

- **Defaults were taken FIRST**, verified by mtime: `..._result_defaults.json` at
  `2026-09-04 23:22:20`, `..._result_ptchdamp06.json` at `2026-09-05 08:35:46`.
- The **regression harness itself was frozen at `2026-09-04 23:17:40`** —
  *before either run existed*. Its gates therefore could not have been tuned to the outcome.
- Both runs: verdict `TECS_PTCH_DAMP_NO_PERFORMANCE_REGRESSION`, `failed_checks = []`,
  `regression_gates_failed = []`, **36 / 36 regression gates PASS** and **66 / 66 acceptance
  checks PASS**.
  *(Recorded as measured: the artifact's `analysis.regression_gates` list contains **36** entries,
  one per `regression_*` acceptance check; a count of 37 elsewhere would include the aggregate.)*

### 7.2 Measured deltas

| metric | defaults | `PTCH_DAMP 0.6` | direction |
|---|---|---|---|
| cruise airspeed std [m/s] | 0.18579 | **0.06596** | better (2.8x) |
| altitude-hold p2p max [m] | 2.1477 | **0.6345** | better (3.4x) |
| per-window hold p2p [m] (R1/R3/R5) | 1.0113 / 2.1477 / 0.9587 | **0.4122 / 0.6345 / 0.4224** | better in all three |
| achieved climb [m] | 10.4319 | 10.3843 | unchanged |
| achieved descent [m] | −11.0114 | −10.9168 | unchanged |
| roundtrip altitude residual [m] | −0.5795 | −0.5324 | slightly better |
| descent ramp duration [s] | 9.2831 | **8.8895** | better |
| whole-flight airspeed min [m/s] | 16.8674 | **17.1171** | better (further from `AIRSPEED_MIN` 16.0) |
| whole-flight airspeed max [m/s] | 19.2719 | 18.9178 | better |
| max abs elevator [deg] | 6.4059 | 6.2895 | better |
| cruise drag mean [N] | 5.1452 | 5.1487 | **identical** — trim/aero state unchanged |
| **hold `vz` max abs [m/s]** | **0.010399** | **0.012577** | **the ONE metric moving the wrong way** |

**On the one adverse metric.** Both values sit far below the gate `TH_REG_HOLD_VZ_MAX_MS =
0.028935 m/s`, **and both sit below the harness's own declared LSQ slope-bias floor of
0.020401 m/s** — the worst-case slope a residual oscillation of the observed amplitude can imprint
on a least-squares fit over a 24 s window. In other words the metric **cannot resolve a change at
that level**; the 0.0022 m/s difference is inside its own stated noise. It is recorded, not
explained away, and not compensated for.

**Tightest margin.** `descent_ramp_vz_peak` 1.7563 m/s against a floor of 1.6857 m/s — a **4.2 %**
margin. Note that the descent ramp *duration* nevertheless **improved** (9.2831 → 8.8895 s) with
the achieved descent essentially unchanged: a smoother, less-overshooting ramp, which is exactly
what higher damping should produce.

### 7.3 Gate construction, and why it is legitimate

Every derived regression tolerance is a **band or one-sided limit taken over TWO independently
recorded reference realisations** (2026-09-02 cruise stage and 2026-09-03 energy stage), with the
statistical tolerance added **once**:

```
band: min(A,B) - TOL <= x <= max(A,B) + TOL
max : x <= max(A,B) + TOL
min : x >= min(A,B) - TOL
```

`validation` ruled this **legitimate use of measured run-to-run scatter, not tolerance widening**,
because the `min/max` over two real realisations *is* the observed scatter and is not a chosen
number. **Caveat recorded:** `n = 2` is a coarse scatter estimate. The harness itself flags this
in the `cruise_airspeed_std_ms` gate derivation ("RESOLUTION LIMITATION … More repeats are
DATA_REQUIRED").

---

## 8. `TECS_PTCH_DAMP = 0.6` — status `VALIDATED_NOT_ADOPTED`

**Not adopted by this stage.** No `.parm`, no SOT parameter table and no default records the
value 0.6 as a Falcon V2 setting. It exists only as a runtime `PARAM_SET` in two measurement runs.
`docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml:183` still records the firmware
default `TECS_PTCH_DAMP = 0.3` from `AP_TECS.cpp:107` and is correct as written.

A future **adoption stage** must satisfy all five of the following (`validation`'s gate):

1. **Source-of-truth entry** carrying the `AP_TECS` algebraic derivation (§2.1), the
   PD-corner-versus-mode-frequency argument (§2.4) and both measured runs, with provenance class
   `DERIVED_CALCULATION` + `GAZEBO_VALIDATION`. — *satisfied by
   `docs/source_of_truth/controls/ardupilot_tecs_pitch_damping_loop.yaml`, written by this stage.*
2. **Repeat runs** to establish run-to-run scatter. Currently `n = 1` per setting on the damping
   measurement and `n = 2` total realisations on the regression references. — **NOT satisfied.**
3. **Envelope coverage.** Currently one airspeed (18 m/s), one step size (+/-10 m), zero wind, one
   mass/CG. The wind/gust model is already validated (commit `efb5624`) and should be exercised. —
   **NOT satisfied.**
4. **Explicit record of the real-aircraft divergence.**
   `docs/source_of_truth/autopilot/real_aircraft/yeni_pixhawk.param:1035` carries
   `TECS_PTCH_DAMP,0.3`. Per `docs/source_of_truth/autopilot/SITL_PARAM_MIGRATION.md` §3 the whole
   `TECS_*` group is classified `REVIEW_AND_ADAPT` and explicitly *"not authoritative Falcon V2
   truth"*, so this is **NOT a blocker** — but 0.6 would be the project's **first Falcon-specific
   TECS value**, and the divergence must be explicit rather than silent. — *recorded here and in
   the SOT file; the adoption stage must restate it.*
5. **The reduced pitch-demand margin declared as a limitation** (§9). — *declared here.*

---

## 9. DECLARED LIMITATIONS — all NON-GATING

### 9.1 `PROPULSION_HIGH_J_WINDMILLING` — carried unchanged, owner `propulsion`

`DATA_REQUIRED`: measured or defensibly extrapolated APC 13x6.5E `Ct/Cp` beyond the zero-thrust
advance ratio (~J 0.64). Ct is clamped to the table end, flooring thrust at 0 N where a real
fixed-pitch propeller would windmill and produce negative thrust. **High-J descent performance is
therefore NOT absolute high-fidelity truth.** Direction, controllability, settling and the
kinematic energy bookkeeping remain valid.

**The apparent worsening is an artifact, verified.** Whole-flight advance-ratio max in the
regression pair moved `J 1.3063 → 1.7572`. `validation` verified independently that this is a
`t = 0.00 s` FBWB-entry **spin-down** artifact: **only 2 of 2315 samples** in RUN 2 exceed RUN 1's
maximum, and **both are inside the discarded 12 s transient**. Excluding the transient, RUN 2's
J max (**0.5525**) is *LOWER* than RUN 1's (**0.5759**), and the interp-clamped fraction *improved*
`0.041667 → 0.039309`. Counted and reported in every artifact's `high_j` block; never gated, never
compensated for.

### 9.2 MINOR-1 — `window_over_tau` is computed on the full span, not the SNR-admitted span

`ringdown.measurability.window_over_tau` uses the **full analysed span**, not the span actually
admitted by the SNR truncation. For the candidate: full span `54.906 / 8.652 = 6.35 tau` → PASS,
but the SNR-admitted span is only `16.646 s = 1.92 tau`, **below** the documented intent of
`TH_MIN_WINDOW_TAU = 2.0`.

**Status: OPEN, owner `controls-integration`, deferred to a future stage.** It was deliberately
**NOT changed post-hoc**. Redefining a measurability gate after seeing which run it would have
caught is exactly the after-the-fact adjustment CLAUDE.md's simulation tuning policy forbids. The
baseline is unaffected (0 extrema rejected, so admitted span = full span).

### 9.3 Reduced pitch-demand margin

Margin from the peak TECS pitch demand to `TECS_PITCH_MAX = 15 deg`, regression runs, R1 cruise:

```
defaults      nav_pitch_raw max  8.71 deg  ->  margin 6.29 deg
PTCH_DAMP 0.6 nav_pitch_raw max 10.69 deg  ->  margin 4.31 deg
```

Still **4.3x** the required `TH_PITCH_DEMAND_MARGIN_DEG = 1.0 deg`, and `pitch_demand_not_clipped`
is `true` in every window of every run. But the trend is real and **will matter at larger altitude
steps or higher gains**. Declared, not gated.

### 9.4 `ASSUMPTION: PHUGOID_REFERENCE_IS_LANCHESTER` — retained

The free-airframe reference is the Lanchester approximation
`tau_ref = V*(L/D)/g`, `T_ref = pi*sqrt(2)*V/g`, `zeta_ref = 1/(sqrt(2)*(L/D))`.

`validation` verified the derivation **algebraically** and checked it **numerically**:
back-solving `tau_ref = 20.873 s` at `V = 17.9276 m/s`, `g = 9.81` gives `L/D = 11.42`, which
matches the aerodynamics plugin's own measured lift-to-drag ratio over the ring-down window
(`CL = 0.66009`, `CD = 0.057792`, `L/D = 11.4219`, recorded in
`ringdown.phugoid_reference_free_airframe`). `validation`'s own window gave `CL/CD ≈ 0.6789/0.0591
= 11.49`, i.e. the same value to better than 0.7 %. A third, independent level-flight identity
`L/D = m*g/T_thrust` gives **11.4906** (report-only; affected by §9.1). Three routes to `L/D`
agree, so the Lanchester reference is at least self-consistent — it is still an ASSUMPTION about
the *mode*, not a measurement of it.

**`DATA_REQUIRED`:** a **measured stick-fixed phugoid** (period *and* damping) from an open-loop
fixed-stick, fixed-throttle run would replace this assumption directly.

### 9.5 `n = 1` per setting

There is **one** phugoid run and **one** regression run per `TECS_PTCH_DAMP` setting. **No
run-to-run scatter estimate exists for the damping measurement itself.** This is the reason
adoption requirement §8.2 is unsatisfied, and it is why the improvement is quoted as a range
(2.5x–3.5x) rather than a point value.

---

## 10. Bookkeeping — what this stage changed

| | |
|---|---|
| `config/ardupilot/falcon_v2_sitl.parm` | **UNCHANGED**, md5 `180e56711bdf18c658f6ded8031421f1`; contains no `TECS_*` assignment |
| `docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml` | **UNCHANGED**; line 183 still records `TECS_PTCH_DAMP: 0.3` from `AP_TECS.cpp:107` |
| `model/model.sdf`, `plugins/**`, aero tables, propulsion tables, actuator config, sensor config | **UNCHANGED** |
| Mass / CG / inertia | **UNCHANGED** — no CG value is used by this stage at all; neither `(0.168309, 0, 0.100000)` nor `(0.0637, 0, -0.0210)` appears in anything this stage wrote |
| PIDs, `PTCH_TRIM_DEG` | **UNCHANGED** |
| Written by this stage | this file, and `docs/source_of_truth/controls/ardupilot_tecs_pitch_damping_loop.yaml` |

---

## 11. Handoff

| Item | Owner | Status |
|---|---|---|
| `CLOSED_LOOP_LONGITUDINAL_DAMPING_WEAKER_THAN_FREE_AIRFRAME` | `controls-integration` | **OPEN**, magnitude corrected to ~1.06–1.13x, non-gating |
| `PROPULSION_HIGH_J_WINDMILLING` | `propulsion` | **OPEN**, `DATA_REQUIRED`, unchanged |
| MINOR-1 `window_over_tau` on admitted span | `controls-integration` | **OPEN**, future stage |
| SNR-truncation reporting in the harness | `gazebo-testing` | in progress in parallel |
| `TECS_PTCH_DAMP = 0.6` adoption | future stage | **BLOCKED** on §8 items 2 and 3 |
| Measured stick-fixed phugoid (replaces `PHUGOID_REFERENCE_IS_LANCHESTER`) | `gazebo-testing` + `aerodynamics` | `DATA_REQUIRED` |

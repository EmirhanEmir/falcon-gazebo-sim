# ACTUATOR_CONTROL_LIMIT_CYCLE_FIX — implementation record (step 3)

> **Not.** Bu dosya projenin iç doğrulama kaydından alınmıştır ve teknik
> içeriği değiştirilmeden korunmuştur. Metinde atıf yapılan `tests/gazebo/`
> test koşum altyapısı ile `docs/source_of_truth/` mühendislik veri ağacı bu
> repository ile dağıtılmaz; o yollar tarihsel kayıt olarak bırakılmıştır.

**Stage:** `ACTUATOR_CONTROL_LIMIT_CYCLE_FIX_AND_VALIDATION`, step 3 (implementation).
**Area:** control / actuation. **Date:** 2026-09-12.
**Authorization:** main session, this stage — F1 (mandatory), F2 (conditional on provenance discipline), F3.
**Companion document (root cause, steps 1–2):** `docs/test_results/2026-09-12_actuator_limit_cycle_root_cause_and_fix_proposal.md`.
**Binding prior review:** `docs/validation/2026-09-11_airborne_actuator_limit_cycle_root_cause_validation.md`.
**No Gazebo flight test was run** — that is the simulation-test stage's job, and runs next.

---

## 1. Files changed — complete diffstat

```
 docs/source_of_truth/controls/actuator_v1_config.yaml | 488 ++++++++++++++++++++-
 plugins/actuators/ActuatorModel.hh                    | 128 +++++-
 plugins/actuators/ActuatorSystem.cc                   |  27 +-
 plugins/actuators/test/actuator_model_selftest.cc     | 133 ++++--
 4 files changed, 697 insertions(+), 79 deletions(-)
```

`git diff --name-only HEAD` returns **exactly these four files and nothing else**. Specifically **not** touched:
`model/model.sdf` (no mass, CG, inertia, mesh, hinge axis, joint `<limit>`, plugin block),
`docs/source_of_truth/aerodynamics/*`, `docs/source_of_truth/propulsion/*`, `config/falcon_v2_sitl.parm`,
`plugins/aerodynamics/*`, `plugins/propulsion/*`, `plugins/sensors/*`, `tests/*`.

| file | what changed |
|---|---|
| `docs/source_of_truth/controls/actuator_v1_config.yaml` | F1 per-surface `kp/kd/ki` + `hinge_inertia_kg_m2`; `control_law` demoted to documented fallback; 6 new provenance notes; SUPERSEDED-IN-PART banners on the 3 stale notes; `control_law_form` documents F3 |
| `plugins/actuators/ActuatorModel.hh` | **F3 only** (the sole behavioural code change): `kRateSaturationDetectFraction`, the rate-saturation anti-windup clause, 2 informational `EffortResult` flags; plus comment corrections for F1/F2. **Function signatures unchanged.** |
| `plugins/actuators/ActuatorSystem.cc` | `LoadConfig()` reads per-surface `kp/kd/ki` with fallback to `control_law`. No other logic touched. |
| `plugins/actuators/test/actuator_model_selftest.cc` | gains + reference inertia updated to the corrected left-aileron values; anti-windup case (c) rescaled; disturbance-rejection bar replaced by the analytic droop law |

---

## 2. F1 — parallel-axis inertia correction (implemented)

### 2.1 The error, named explicitly, corrected in place

The pre-fix `derivation_note` stated its inertia reference as *"`I_ref = 3.4788e-07 kg*m^2` … `model/model.sdf left_aileron <inertial><inertia><iyy>`, **about the hinge-ish axis**"*.
An SDF `<inertial><inertia>` tensor is about an axis through the link's **centre of mass**, at `<inertial><pose>`. None of the 5 joints has a `<pose>`, so the joint frame **is** the child link frame and the hinge axis passes through the link **origin**, while the CoM sits `d_perp` = 20.1–26.6 mm away perpendicular to it. The correct plant inertia is

```
I_hinge = n·(I_com + m(|d|²E − d dᵀ))·n = I_com,nn + m·d_perp²,
d_perp² = |d|² − (n̂·d)²,   n̂ = normalised <axis><xyz>,   d = <inertial><pose> translation
```

| surface | `I_com,nn` (SDF) | `m·d_perp²` (omitted) | `I_hinge` | ratio | `d_perp` |
|---|---|---|---|---|---|
| aileron (`iyy`) | 3.4788e-07 | **7.062178e-07** | **1.054098e-06** | 3.03006 | 26.5748 mm |
| elevator (`iyy`) | 1.7540e-07 | **4.346859e-07** | **6.100859e-07** | 3.47826 | 20.8491 mm |
| rudder (`izz`) | 1.7775e-07 | **4.049313e-07** | **5.826813e-07** | 3.27809 | 20.1229 mm |

The SDF axis vectors are not exactly unit length (`|n|` = 0.99999955 / 0.99999960 / 0.99999985); `n` is normalised before use — skipping that changes `d_perp` in the 7th significant figure only.

`derivation_note`, `integral_derivation_note` and `placeholder_inertia_note` are **left unrewritten** as the historical record, each now carrying a `!! SUPERSEDED IN PART, 2026-09-12` banner naming the specific false statements and pointing at `hinge_inertia_derivation_note`. Nothing was quietly overwritten.

### 2.2 Sub-finding (2), called out separately as required

The previous config applied **one shared** `kp/kd/ki` — derived from the **aileron's** `I_com` — to the elevator and rudder as well, and `derivation_note` argued this was *"if anything mildly conservative/gentler for those three surfaces"*. Gentler is the wrong direction for this defect, and the two errors partly cancelled for the elevator/rudder while compounding for the aileron. Gains are now per-surface.

### 2.3 What the running system's dynamics actually were (stale claims corrected)

| surface | config **claimed** | **actual** `ω_n` | **actual** `ζ` | **actual** cubic roots |
|---|---|---|---|---|
| aileron | `ω_n=100, ζ=1.0`, roots `−113.29, −84.63, −2.09`, *"no oscillatory (complex-conjugate) mode is introduced at all"* | **57.448** | **0.5745** | **−31.9606 ± 46.3202j**, −2.0841 |
| elevator | same claim | **75.513** | **0.7551** | **−55.9788 ± 48.3223j**, −2.0854 |
| rudder | same claim | **77.268** | **0.7727** | **−58.6606 ± 47.7975j**, −2.0855 |

The claimed roots described a plant that does not exist. The Routh-Hurwitz conclusion survives unharmed because that criterion is inertia-independent here (`kd·kp > I·ki ⟺ 2ζω_n > 1/Ti`, now 400 > 2). All of this is recorded in `hinge_inertia_derivation_note`.

### 2.4 F1 alone is insufficient — recorded in the config as the reason F2 exists

Stated verbatim in `hinge_inertia_derivation_note`: at the unchanged `ω_n = 100`, F1 raises the elevator's `kp` by only **1.7537×** (its own `I_hinge` is *smaller* than the aileron-`I_com`-derived shared value), moving `V_crit` from 22.5 to **30.6 m/s** — below both flown records (32.9 and 37.5 m/s equivalent from `q̄`).

---

## 3. F2 — bandwidth `ω_n` 100 → 200 rad/s (implemented)

Written into `bandwidth_requirement_note` in the **required order**: requirement first, number second.

**REQUIREMENT ASI-1 (stated up front, before any `ω_n` is quoted).** The closed hinge/airframe loop
`L(s) = [m·c_z·|r_x|·(q̇/δ)(s)] / (I_h s² + kd s + kp + ki/s)` shall satisfy
**(a)** `max|L| ≤ 0.32` (≥ 10 dB) at the highest airspeed present in this repository's own recorded flight data, **33 m/s**;
**(b)** `max|L| ≤ 0.13` (≥ 18 dB) at the documented trim airspeed, **21.244 m/s** (CLAUDE.md).
10 dB is the conventional minimum gain margin for a servoelastic loop; the larger 18 dB applies at the design point the aircraft actually spends its time at. Envelope `VNE` is `DATA_REQUIRED`, so 33 m/s is *observed*, not assumed. **This requirement would have been correct to state before the defect was known; it is not back-solved from "the oscillation stopped".**

**Solving it.** `|L| ∝ q̄/kp ∝ V²/ω_n²`. At `ω_n = 100` with the F1-corrected inertia the elevator loop gives `max|L| = 1.153` at 33 m/s, i.e. 3.7× over (a); therefore `ω_n ≥ 100·√(1.153/0.32) = 189.8 rad/s`. **⇒ `ω_n = 200 rad/s`**, the smallest round value meeting ASI-1.

**`dt` ceiling and the actual margin to it.** Torque is recomputed once per tick (`SetForce` from `PreUpdate`, `max_step_size = 0.001 s`, dartsim) and the joint velocity advanced semi-implicitly. The binding condition is the damping term: `2ζω_n·dt < 2 ⟺ ω_n < 1/(ζ·dt) = **1000 rad/s**` at `ζ = 1.0`. (The stiffness condition `ω_n²dt² < 4`, i.e. `ω_n < 2000`, is slacker and does not bind.) This project's own self-test independently found divergence at `ω_n = 2566` (`ω_n·dt = 2.566`), consistent with that order.
**`ω_n = 200` ⇒ `ω_n·dt = 0.200`, `2ζω_n·dt = 0.400` — margin to the ceiling 5.0×** (the previous 100 sat 10× inside it).

**Labels, exactly as required.** `ω_n = 200 rad/s` is labelled **`ASSUMPTION`** in the config and explicitly *"is NOT a measurement and must not be quoted as one"*. The physical argument recorded is directional only: a real geared servo's back-drive resistance is set by gear ratio and gearbox friction, not by position-loop bandwidth, and for an ES08MAII-class part it is effectively rigid against the ~1e-3 N·m disturbances measured here — so "stiffer than 100 rad/s" is defensible while the *value* is bounded by the timestep, not by the servo. **`DATA_REQUIRED`** is restated and still open: ES08MAII (or the actually-fitted servo) stall torque, no-load speed, gear ratio, **back-drive/holding stiffness**; real per-surface surface mass; real joint damping/friction (all 5 joints still have **no `<dynamics>` element at all** — zero damping, zero friction).

### 3.1 As-implemented gains (verified by reading the committed YAML through yaml-cpp, exactly as the plugin does)

| surface | `hinge_inertia_kg_m2` | `kp_nm_per_rad` | `kd_nm_per_rad_s` | `ki_nm_per_rad_s` | ×kp vs before |
|---|---|---|---|---|---|
| left/right aileron | 1.054098e-06 | **4.216391e-02** | 4.216391e-04 | 8.432782e-02 | 12.1202 |
| left/right elevator | 6.100859e-07 | **2.440344e-02** | 2.440344e-04 | 4.880687e-02 | 7.0149 |
| rudder | 5.826813e-07 | **2.330725e-02** | 2.330725e-04 | 4.661450e-02 | 6.6998 |
| `control_law` (fallback only, unreachable) | — | 2.330725e-02 | 2.330725e-04 | 4.661450e-02 | — |

Back-computed from the committed file, all five surfaces:
**`ω_n` = 199.999976 / 200.000016 / 199.999991 rad/s, `ζ` = 1.000000, `Ti` = 0.500000 s** — the designed properties now hold **exactly, on every surface, for the first time**.
Cubic roots `−219.108, −178.850, −2.0415` on all five (real, negative, identical). The slow integral pole moves only −2.086 → −2.041, so the 2026-08-24 droop-rejection dynamics are preserved.
Effort at a full-travel error (1.5707963268 rad): 0.066231 / 0.038333 / 0.036611 N·m = **33.1 / 19.2 / 18.3 %** of the unchanged `max_effort_nm = 0.20`.

The rudder is included **not** because it oscillated (it did not — 0.002° p2p) but because its immunity is excitation-dependent, not structural; recorded in `rudder_immunity_is_excitation_dependent_note`.

---

## 4. F3 — rate-saturation anti-windup (implemented)

Justification accepted by the coordinator and recorded in `rate_saturation_antiwindup_note`: effort-clamp duty and integral-freeze duty were **0.0 % in every run, on every surface, in both flight modes** — the pre-existing anti-windup never engaged once — while the joint sat at the `SetVelocityLimits` bound **44–85 %** of the time with the integrator accumulating unopposed (`ki·I` reached 9.00e-04 N·m = 79 % of peak `kp·θ`), and the integral path's measured mean mechanical power was **positive** (+1.26e-04 to +3.04e-04 W). A correctness defect in the anti-windup logic, not a tuning knob.

Implemented in `PidEffort()` as a third freeze clause:

```cpp
const bool rateSaturated = cfg.maxRateRadS > 0.0 &&
    std::abs(actualRateRadS) >= kRateSaturationDetectFraction * cfg.maxRateRadS;
const bool rateSameSignAsError =
    (actualRateRadS >= 0.0 && error >= 0.0) || (actualRateRadS <= 0.0 && error <= 0.0);
const bool rateSatFreeze = rateSaturated && rateSameSignAsError;

const bool freeze =
    targetClampActive || (wouldSaturate && sameSignAsError) || rateSatFreeze;
```

- **Direction-conditional**, exactly like the existing effort clause, so the integrator is never prevented from *unwinding*.
- **`kRateSaturationDetectFraction = 0.999`** is a *detection threshold*, not a physical coefficient. Provenance: it is the identical definition of "rate-pinned" already used by `tests/gazebo/scripts/analyze_airborne_limit_cycle_runtime.py` and both limit-cycle test reports, so the plugin's internal notion of saturation and the harness's published duty statistics refer to the same event.
- **Signature unchanged.** The condition is derived from arguments `PidEffort()` already received (`actualRateRadS`, `cfg.maxRateRadS`), so **no call site changed** anywhere.
- **Separately identifiable**, as required: `EffortResult` gained `rateSatFreezeActive` (freeze caused specifically by F3) and `rateSaturated` (observed at the limit regardless of freeze). Both informational — **neither is published**, so the 35-field diagnostics layout the simulation-test stage's `actuator_lib.py` depends on is byte-identical.

---

## 5. Verification performed

### 5.1 Standalone self-test — **13 PASS, 0 FAIL, 1 INFO**

`cmake --build plugins/actuators/build` (clean) then `./plugins/actuators/build/actuator_model_selftest`:

| test | result |
|---|---|
| `ACTUATOR_NEUTRAL_START_TEST` | **PASS** — setpoint 0.000000000 rad, no flags |
| `ACTUATOR_POSITIVE_STEP_TEST` | **PASS** — 20° in 66 steps (0.0660 s), monotonic |
| `ACTUATOR_NEGATIVE_STEP_TEST` | **PASS** — −20° in 66 steps, monotonic |
| `ACTUATOR_RATE_LIMIT_TEST` | **PASS** — worst overshoot **0.000e+00** rad |
| `ACTUATOR_POSITIVE_45_LIMIT_TEST` | **PASS** — cmd 60° → 0.785398 rad, never exceeded |
| `ACTUATOR_NEGATIVE_45_LIMIT_TEST` | **PASS** — cmd −60° → −0.785398 rad, never exceeded |
| `ACTUATOR_OVERCOMMAND_CLAMP_TEST` (+ and −) | **PASS** — cmd ±1000° → exactly ±0.785398163 rad |
| `ACTUATOR_NO_TELEPORT_TEST` | **PASS** — 450 steps, worst step 5.236000e-03 = exactly `max_rate·dt` |
| `PD_EFFORT_CLAMP_TEST` (bonus) | PASS |
| `ACTUATOR_INTEGRAL_ANTI_WINDUP_TEST` (bonus) | PASS — integral settles 1.672000 vs analytic 1.6717 rad·s |
| `ACTUATOR_INTEGRAL_DISTURBANCE_REJECTION_INFO` (bonus) | PASS — ki=0 residual **−0.2718°** vs analytic `−T/kp` = **−0.2718°**; ki on → −0.0000° |
| `ACTUATOR_SETPOINT_STEP_OVERSHOOT_TEST` (bonus) | PASS — 0.1628° → 0.0000° (100 % reduction), final residual −0.00023° |
| `CLOSED_LOOP_TRACKING_INFO` | INFO — `stable=1`, converges to target |

**All 8 required pure-math tests still pass.** The first-run failure was in my *own new* check (predicted droop sign): the residual is `sp − angle = −T/kp`, negative for a positive disturbance. Corrected to the signed prediction; the old `abs(...) > 1.0` bar could never have caught a sign error, so this is strictly stronger.

### 5.2 Self-test changes — disclosed, as required

The self-test **does** hardcode gains and a reference inertia. Four changes, each consistent with the config:
1. `MakeV1Config()` gains → the **left-aileron** per-surface set (4.216391e-02 / 4.216391e-04 / 8.432782e-02). Chosen so gains and reference inertia stay a matched pair. `spWeightB = 0.7` **unchanged**; travel, rate and effort limits **unchanged**.
2. `refInertia` (3 occurrences) `3.4788e-07` → **`1.054098e-06`**, the aileron's inertia about its **hinge**. Using the bare `<iyy>` in the self-test was the *same* error F1 fixes.
3. Anti-windup case (c) sustained error 10 rad → **2 rad**, because at 12.1× `kp` the P-term alone would now saturate on tick 1 (`kp·0.7·10 = 0.295 > 0.20`), collapsing case (c) into a duplicate of case (b). The *form* of the analytic fixed point is unchanged: `(max_effort − kp·b·err)/ki`, now 1.6717 rad·s. Cases (a)/(b) untouched.
4. `ACTUATOR_INTEGRAL_DISTURBANCE_REJECTION_INFO`'s hardcoded `> 1.0 deg` bar replaced by the **analytic law** `residual = −T/kp` matched within 10 % (plus a `> 0.1°` non-triviality floor). The old bar was a proxy that stops being meaningful once `kp` changes — it would have failed for the right reason and hidden the mechanism. `disturbanceNm = 2.0e-4` is **unchanged** (it is a physical torque, not a tuning parameter).

F3 is inert in all three anti-windup cases — every call there passes `rate = 0.0`.

### 5.3 Direct arithmetic proof: travel clamp and rate limiter byte-for-byte unchanged

Differential harness compiling the **git-HEAD** `ActuatorModel.hh` (renamespaced) alongside the **new** one and comparing bit-for-bit:

| exercised | cases | **bitwise differences** |
|---|---|---|
| `StepSetpoint()` = Stage 1 travel clamp + Stage 2 rate limiter, `cmd ∈ [−20, 20]` rad (incl. gross overcommand), `prevSetpoint ∈ [−1.6, 1.6]` rad, `dt ∈ {0, 1e-6, 1e-4, 1e-3, 2e-3, 1.0}` s — all four output fields | **2 663 040** | **0** |
| `PidEffort()`, identical gains, **unsaturated** (`\|rate\| < 0.999·max_rate`) — `torqueNm`, `effortClamped`, `integralRadS`, `integralFrozen` | **348 704** | **0** |
| `PidEffort()`, identical gains, **rate-saturated** — F3 acting by design | 51 296 | 24 957 |

F3's freeze fired on **25 639 / 51 296 = 49.98 %** of saturated cases — exactly the 50 % the direction-conditional test predicts for randomly signed rate/error, confirming it is conditional and not a blanket freeze.

**Conclusion: the ±45° travel clamp and the rate limiter are provably unchanged (0 differences in 2 663 040 cases), and F3 is provably inert whenever the joint is not at its velocity limit (0 differences in 348 704 cases).**

### 5.4 Config load verified through yaml-cpp

A harness mirroring `LoadConfig()` exactly confirms the anchors/aliases resolve and every surface reads its own gains: `ω_n` = 199.999976 / 199.999976 / 200.000016 / 200.000016 / 199.999991 rad/s, `ζ` = 1.000000 on all five, `Ti` = 0.500000 s on all five, `min/max_angle_rad = ∓0.785398163`, `max_rate_rad_s = 5.236`, `max_effort_nm = 0.20` — the last three **unchanged**.

---

## 6. Falsifiable predictions, recomputed against the AS-IMPLEMENTED numbers

Loop model fed the gains read from the **committed** YAML. Elevator uses the worst-case measured `dCm/dθ = 1.3934` (FBWA); aileron 1.5083.

| # | quantity | before | **as implemented** |
|---|---|---|---|
| P1 | Elevator `V_crit` (`max\|L\| = 1`) | **22.54 m/s** (below trim 21.24!) | **61.72 m/s** |
| P2 | Elevator `max\|L\|` @ 33.0 m/s | 2.117 | **0.298 (10.5 dB)** — meets ASI-1(a) |
| P3 | Elevator `max\|L\|` @ 21.244 m/s (trim) | 0.889 | **0.124 (18.1 dB)** — meets ASI-1(b) |
| P3b | Elevator `max\|L\|` @ 37.5 m/s / 47.5 m/s | 2.708 / 4.217 | 0.383 (8.3 dB) / 0.606 (4.3 dB) |
| P3c | Aileron `V_crit`; `max\|L\|` @ 33 m/s | 40.11 m/s; 0.670 | **176.65 m/s; 0.050 (26.0 dB)** |
| **P4** | **MANUAL, bit-constant zero command: uncommanded aileron/elevator p2p** | **33.80–34.09°** | **< 2.0°** |
| **P5** | **FBWA: uncommanded aileron p2p** (cmd p2p 0.064°) | **33.74°** | **< 2.5°** |
| **P6** | **Velocity-clamp (`\|θ̇\| ≥ 0.999·R`) duty, airborne** | **44–85 %** | **< 5 %** |
| P7 | **frequency of any residual oscillation** | 4.0–4.5 Hz | **still ≈ 4.2–4.8 Hz. NOT an acceptance criterion — see §7.** |
| P8 | `θ/f_z = m·c_z/kp` [°/(m/s²)] | ail **0.4314**, elev 0.3375, rud 0 | ail **0.0356** (12.1×), elev **0.0481** (7.0×), rud **0** |
| P9 | uncommanded deflection in a 3 g pull-up (`f_z ≈ 30 m/s²`) | ail 12.94°, elev 10.13° | **ail 1.07°, elev 1.44°** |
| P10 | transient gravity droop before the integrator catches (steady state stays 0) | ail 4.23°, elev 3.31° (measured 2.6–4.2°) | **ail 0.35°, elev 0.47°** |
| P11 | **rudder** uncommanded p2p, longitudinal excitation | 0.002° | **≤ 0.002° — MUST NOT REGRESS** |
| P12 | elevator small-signal actuator τ | 0.058 s (measured) | **≈ 0.010–0.020 s** (dominant pole −178.85 vs −55.98) |
| P13 | `effort_clamp_active` duty | 0.0 % | **0.0 %** in normal flight; may become briefly nonzero on a full-travel step (aileron peaks at 33.1 % of `max_effort`, plus the integral) — physically a servo stalling, but worth logging |
| P14 | pitch-rate RMS, FBWA airborne | 126–148 °/s | **< 25 °/s** |
| P15 | **F3 specifically** (separately identifiable) | integral-freeze duty 0.0 % while rate-pinned 44–85 % | after F1+F2 the joint should rarely velocity-saturate, so F3 should be **largely dormant**; if rate-pinned duty is non-trivial, freeze duty must now track it |

**P4, P5 and P6 are the acceptance criteria.**

---

## 7. Acceptance-criterion trap — carried forward explicitly

> **The limit-cycle frequency is set by the airframe short period, not by `kp`.** Measured `f_LC ≈ 1.18 × f_shortperiod` at *both* recorded flight conditions (4.500 vs 3.820 Hz; 4.000 vs 3.352 Hz). The as-implemented loop model still peaks at ≈ 4.22 Hz at 33 m/s — essentially unmoved by a 7–12× change in `kp`.
> **Acceptance must be judged on AMPLITUDE (uncommanded peak-to-peak) and RATE-PINNED DUTY. Never on frequency.** A residual or re-excited oscillation would appear at very nearly the same ~4 Hz and must not be mistaken for "unchanged behaviour". This is also recorded in the config's `limit_cycle_fix_scope_note`.

---

## 8. Geometry: why no the structural layer change was made

Validation (2026-09-11) routed an item suggesting `k_g = 18.7 mm < d = 27.1 mm` is geometrically implausible. Checked directly before assuming any geometry change was needed; **no geometry change is required.** Arithmetic, also recorded in the config's `geometry_consistency_note`:

1. **Each surface's SDF inertia reproduces its own collision box as a uniform solid box, to 5 significant figures:**
   aileron box (0.061119, 0.479366, 0.020953): `(x²+z²)/12 = 3.4788003e-04` vs `iyy/m = 3.4788e-04`;
   elevator box (0.044973, 0.189400, 0.009066): `(x²+z²)/12 = 1.7540e-04` vs `iyy/m = 1.7540e-04`;
   rudder box (0.045316, 0.008911, 0.169500): `(x²+y²)/12 = 1.7774e-04` vs `izz/m = 1.7775e-04`.
2. **`k_g < d_perp` is expected, not anomalous,** for a surface hinged near its leading edge. For a uniform box of chord `c` hinged at the LE, `k_g = c/√12 = 0.2887c` while `d_perp = c/2`, so `k_g/d_perp = 0.577 < 1` **by construction**. The SDF values place the hinge 4.0 / 1.6 / 2.5 mm aft of each box's front face — essentially leading-edge hinged, exactly as a plain hinged control surface should be.
3. **Validation's own flat-plate reference `d/k² = 1.5/c` is the hinge-referenced form** (`k² = c²/3` about the LE ⇒ `(c/2)/(c²/3) = 1.5/c`). For the aileron it predicts `1.5/0.061119 = 24.54 1/m`; the SDF's hinge-referenced value is `d_perp/(I_hinge/m) = 0.0265748/1.054098e-03 = 25.21 1/m` — **agreeing to 2.8 %**.

**Validation's sanity check was correct, and was implicitly computed *with* the parallel-axis term included. It is the gain derivation, not the geometry, that disagreed with it — the apparent inconsistency was a symptom of the very defect F1 fixes.** No mesh, collision box, inertia, mass, CoM or hinge axis was touched. If any future finding shows a genuine geometry change is needed, that is the structural layer's call and I will stop and report rather than make it.

---

## 9. Explicitly unchanged (forbidden list, confirmed by diff)

`max_rate_rad_s = 5.2360` · `max_effort_nm = 0.20` · `min/max_angle_rad = ∓0.7853981634` · SDF joint `<limit>` ±45° · PWM→rad transport (`multiplier = π/2`, `offset = −0.5`, 800–2200 µs, `SERVOx_TRIM = 1500`) · every verified sign mapping (`CONTROLS.md` §10: `+left_aileron_joint` → left aileron TE **up**; `+left/right_elevator_joint` → elevator TE **up**; `+rudder_joint` → rudder TE toward **−Y** = right in FLU; `δe = −0.5(θ_LE+θ_RE)`, `δa = +0.5(θ_RA−θ_LA)`, `δr = θ_R`) · `sp_weight_b = 0.7` · `TEMPORARY_NUMERICAL_MASS = 0.001 kg` (per M1, replacing it would change the compliance by exactly zero) · mass/CG/inertia/meshes/hinge axes · aero coefficients · propulsion · sensors · ArduPilot roll/pitch PID · TECS · the 35-field diagnostics layout · `diagnostics_rate_hz = 20`.

---

## 10. Handoff

**the simulation-test stage (next):** `CONTROL_SURFACE_DIRECTION_TEST`, `AILERON_TEST`, `ELEVATOR_TEST`, `RUDDER_TEST`, `ROLL_RESPONSE_TEST`, `PITCH_RESPONSE_TEST`, **`YAW_RESPONSE_TEST`** (the rudder's immunity is excitation-dependent — this is the one axis never yet excited), plus a re-run of the airborne limit-cycle runtime verification (MANUAL **and** FBWA, 1 kHz high-rate) against **P4/P5/P6**, and the `ACTUATOR_FLIGHT_LOAD` / gravity-droop steady-state check against **P10**. The plugin `.so` has been rebuilt at `plugins/actuators/build/libFalconV2Actuators.so`.

**the independent review (after):** independent review. Specific attention requested on (i) whether REQUIREMENT ASI-1 in `bandwidth_requirement_note` is genuinely requirement-driven rather than outcome-driven, (ii) the `ω_n = 200` `ASSUMPTION` label and the 5.0× `dt`-ceiling margin, and (iii) the §8 geometry adjudication, which the coordinator has flagged for independent review.

**Residual risk:** `max|L|` rises as `V²`; above ≈ 62 m/s the elevator loop is predicted unstable again. The MANUAL record reached 47.5 m/s (itself a symptom of the uncontrolled dive), where the margin is 4.3 dB. `VNE` is `DATA_REQUIRED`. If any legitimate test flies above ~55 m/s this must be revisited.

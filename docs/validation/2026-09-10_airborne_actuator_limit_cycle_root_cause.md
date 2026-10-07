# AIRBORNE_ACTUATOR_LIMIT_CYCLE_ROOT_CAUSE — measurement and root cause

> **Not.** Bu dosya projenin iç doğrulama kaydından alınmıştır ve teknik
> içeriği değiştirilmeden korunmuştur. Metinde atıf yapılan `tests/gazebo/`
> test koşum altyapısı ile `docs/source_of_truth/` mühendislik veri ağacı bu
> repository ile dağıtılmaz; o yollar tarihsel kayıt olarak bırakılmıştır.

**Stage:** `AIRBORNE_ACTUATOR_LIMIT_CYCLE_ROOT_CAUSE`
**Date:** 2026-09-10 (analysis pass ran 2026-09-10 → 2026-09-11 local)
**Area:** control / actuation
**Classification:** MEASUREMENT-ONLY / READ-ONLY / OFFLINE.
**Parameters changed: NONE.** No simulation was launched. No PID/TECS parameter,
actuator gain, rate limit, effort limit, aerodynamic coefficient, propulsion
value, mass/CG/inertia, sensor setting, control mapping, timestep or physics
setting was read for modification or modified. No existing captured record was
altered. **No fix is applied by this stage — the defect is described, not
repaired.**

**Defect under investigation (routed here, deferred, by the previous stage):**

- `docs/test_results/2026-09-10_manual_takeoff_and_live_control_surface_behavior_validation.md` §7b row `i3`
- `docs/validation/2026-09-10_manual_takeoff_and_live_control_surface_behavior_validation.md` finding `i3`

---

## 0. Verdict

| Question | Answer |
|---|---|
| Reproduced? | **YES** — in already-captured data, no new flight needed. |
| Root-cause class | **`ACTUATOR_CONTROL_LIMIT_CYCLE`** |
| Blocker severity | **CRITICAL** for any airborne dynamics result obtained after liftoff in these runs. |
| Was `i3` correctly scoped? | **NO.** `i3` was measured only on the three TAKEOFF-mode runs (0.5–3.0 % rate-pinned). On the six MANUAL/FBWA runs in the *same* stage the figure is **43.6–86.0 %** of all airborne samples, and in FBWA it is a **continuous, whole-flight** limit cycle, not a transient burst. |

---

## 1. What was measured, on which record

No new run was flown. Every number below comes from records that already
existed before this stage started.

| Record | Role | Channels used | Rate (MEASURED per file) |
|---|---|---|---|
| `tests/gazebo/results/manual_takeoff_ground_roll_run{1,2,3}_{TAKEOFF,MANUAL,FBWA}_timeseries.json` | actuator + aero + attitude, one timestamp per sample | `surface_full.*` (`cmd_rad`, `target_clamped_rad`, `setpoint_rad`, `actual_angle_rad`, `actual_rate_rad_s`, `target_clamp_active`, `effort_clamp_active`), `aero.Cl/Cm/Cn/V/qbar`, `servo_pwm`, `gz_rpy_deg`, `attitude_mav` | actuator/aero diagnostics **19.98–20.02 Hz** (Nyquist ≈ 10.0 Hz) |
| `tests/gazebo/results/manual_takeoff_ground_roll_run{1,2,3}_{TAKEOFF,MANUAL,FBWA}_dataflash/00000001.BIN` | ArduPilot's own log — the only high-rate view | `IMU[0].GyrX/Y/Z`, `RCOU.C1..C5`, `PIDR.Tar/Act/P/I/D`, `ATT`, `SIM2` | IMU **50.00 Hz** (Nyquist 25.0 Hz); RCOU / PIDR / ATT **25.00 Hz**; SIM2 **50.00 Hz** |
| `tests/gazebo/results/control_surface_live_sign_scenario_{A..F}_timeseries.json` | healthy control condition | same actuator channels | ~20 Hz |
| `model/model.sdf`, `docs/source_of_truth/controls/actuator_v1_config.yaml`, `plugins/actuators/*` | **read only**, for provenance of every constant and for the joint-axis / link-inertia facts quoted in §5 | — | — |

**Clock alignment (measured, not assumed).** The harness records Gazebo sim
time; the dataflash records ArduPilot sim time. The offset was obtained by
maximising the correlation between the harness's Gazebo world `z` and the
log's own `SIM2 -PD`: offset **−0.016 to −0.018 s**, Pearson **r = 0.999998 to
1.000000** on every run. Per-channel timestamps use `t_sim + skew_s[channel]`
(`skew_s` is defined by `manual_takeoff_live_lib.py` as
`arrival_wall[ch] − max(arrival_wall)`, i.e. that channel's own arrival time).
The runs execute at RTF 0.99993–0.99996, so the wall↔sim conversion error over
a 0.5 s window is < 1e-4 s.

### 1.1 Channel availability — stated, not worked around

| Required channel | Status |
|---|---|
| ArduPilot servo output / PWM | AVAILABLE (`RCOU.C1..C5` at 25 Hz; `servo_pwm` over MAVLink at 31–40 Hz) |
| actuator `cmd_rad` | AVAILABLE |
| actual joint angle | AVAILABLE |
| actual joint rate | AVAILABLE |
| effort-clamp flag | AVAILABLE (`effort_clamp_active`) |
| travel-clamp flag | AVAILABLE (`target_clamp_active`) |
| roll/pitch/yaw and p/q/r | AVAILABLE (IMU 50 Hz, ATT 25 Hz) |
| aero `Cl`/`Cm`/`Cn` | AVAILABLE |
| **rate-clamp FLAG** | **`DATA_REQUIRED`.** The plugin publishes no rate-clamp flag — the velocity limit is a physics-engine hard constraint (`ActuatorSystem.cc:127` `SetVelocityLimits`), not a plugin decision. This report therefore counts samples whose **measured** `\|actual_rate_rad_s\|` sits on that limit and calls them `RATE_PINNED` (an observation), never `rate_clamp_active` (a flag that does not exist). Because the observation is made at 20 Hz on a limit enforced at the 1 kHz physics tick, **every pinned count here is a LOWER BOUND.** |
| **actuator effort / torque (N·m)** | **`DATA_REQUIRED`.** The 35-field diagnostics layout documented in `actuator_v1_config.yaml` (`command_interface.diagnostics_field_order_per_surface`) publishes only the saturation flag, not the applied torque. The applied joint torque is not recoverable from any existing record and is **not inferred anywhere in this report.** |

### 1.2 Thresholds — fixed before any result was inspected

| Symbol | Value | Tag | Basis |
|---|---|---|---|
| `RATE_PIN_FRACTION` | 0.999 | `ASSUMPTION` | float-comparison tolerance on an exact clamp, not a physical bound |
| `CLAMP_FLAG_TRUE` | 0.5 | `ASSUMPTION` | booleans published as floats |
| `BURST_WINDOW_S` | 0.5 s | `ASSUMPTION` | ≥ 2 periods of any candidate ≥ 4 Hz oscillation |
| `BURST_RMS_MULT` | 5.0 (also reported at 3.0 and 10.0) | `ASSUMPTION` | a *self-normalising multiple* of each run's own median airborne window RMS, so the detector cannot be tuned per run; reported at three multipliers so the sensitivity of every burst-scoped number is visible |
| `CALM_WINDOW_S` | 1.0 s | `ASSUMPTION` | ≥ 4 periods |
| `CMD_CALM_P2P_DEG` | 1.0° | `ASSUMPTION` | ≈ 15.6 PWM µs ≈ 15× the 0.0643° transport quantum |
| `ACT_OSC_P2P_DEG` | 3.0° | `ASSUMPTION` | 3× the calm-command bar |

No threshold was changed after a result was seen. The burst detector is
purely *relative*; because of that its output is always reported next to the
absolute window RMS (`peak_window_rms_deg_s`), so a run whose median is
0.004 °/s cannot make a physically negligible ripple look like a burst.

---

## 2. Answers to the nine questions

### Q1 — Where does the oscillation FIRST originate?

**In the actuator/joint layer.** Established by a case in the captured data
where every other candidate is *bit-exactly* excluded:

In all three **MANUAL** runs the aileron command is **exactly
`cmd_rad` = 0.000000 rad** over the entire airborne segment
(`RCOU.C1` pinned at 1500 µs). In all three **FBWA** runs it spans
0.00 … 0.06°, i.e. **one PWM microsecond**, the smallest value the transport
can express. Over that same segment:

(all figures below: aileron, whole airborne segment, per-run)

| Mode | aileron `cmd` p2p, full segment | worst 1 s-window aileron `actual` p2p | per-burst aileron `actual` p2p range | rate-pinned | airframe roll rate `GyrX` median window RMS |
|---|---|---|---|---|---|
| FBWA ×3 | **0.00–0.06°** (one PWM µs) | **33.85 / 34.16 / 34.29°** | 28.15 – 34.05° | **75.8 %** | **0.0043 °/s** |
| MANUAL ×3 | **exactly 0.00°** | **33.39 / 34.47 / 34.51°** | 29.46 – 33.71° | **43.6 %** | 0.034 °/s |

The aircraft is **not rolling** (FBWA `GyrX` RMS = 0.0043 °/s, roll attitude
0.0°, steady wings-level 33.7 m/s cruise) and **nothing is commanding the
ailerons**, yet both aileron joints swing ~±16° at ~4 Hz with the joint
velocity on its 300 °/s limit three quarters of the time. ArduPilot,
the command path and the roll axis of the airframe are therefore all
excluded as the origin.

**Causal ordering at the ignition event** (FBWA run 1, resolved at 50 Hz):

| `t_sim` | event |
|---|---|
| ≤ 33.05 s | quiet. `GyrY` = 0.0005 rad/s, `LAact` = −0.067°, `LArate` = −0.1 °/s, `z` = 0.00 m, V = 20.3 m/s |
| 33.079 s | airframe pitch rate begins to grow (`GyrY` 0.0010 → 0.192 rad/s by 33.279 s) — this is the **real liftoff rotation**, `AccZ` −9.82 → −12.96 m/s², `z` starts rising |
| 33.104 s | first joint deviation (`LAact` −0.114°, `LArate` −1.5 °/s) |
| 33.26 s → | `LArate` −9.5 → −14.7 → ±300 °/s; limit cycle established |
| 33.3 s … 85.2 s | **limit cycle never stops** (52 s, to end of record) |

So a genuine physical transient (the liftoff pitch-up) *triggers* it, and the
actuator layer then *sustains* it indefinitely with no command.

**In the TAKEOFF runs the ArduPilot roll-rate loop is entrained, but as a
follower.** `PIDR` shows the outer roll-angle loop's *demand* barely moving at
the oscillation frequency while the *measured* rate does all the moving:

| run/burst | `PIDR.Tar` amp @ f₀ | `PIDR.Act` amp @ f₀ | P-term p2p | I-term p2p | D-term p2p |
|---|---|---|---|---|---|
| 1 / 67.0–70.0 s | 1.70 °/s | **28.30 °/s** | 15.20 | 0.32 | 2.77 |
| 2 / 60.8–63.8 s | 2.40 °/s | **33.03 °/s** | 22.70 | 0.56 | 3.59 |
| 3 / 61.7–64.2 s | 3.78 °/s | **44.59 °/s** | 24.68 | 0.62 | 3.83 |

The rate PID is *reacting* to an oscillation it did not ask for. Its output
then feeds back into the same joints and enlarges them — see Q7 — but it is
not the origin, and the same ~4 Hz oscillation exists with the loop fully open
(MANUAL) and with the aileron channel bit-constant (FBWA).

### Q2 — Does the actual joint oscillate while the command is calm/constant?

**Yes, decisively.** Criterion (fixed in advance): command p2p ≤ 1.0° AND
actual p2p > 3.0° over a 1.0 s window.

| Mode | qualifying windows / airborne windows | worst case |
|---|---|---|
| FBWA ×3 | **53/53, 53/53, 53/53** (ailerons) | cmd p2p **0.000°**, actual p2p **34.29°** |
| MANUAL ×3 | **40/52, 41/53, 40/52** (both ailerons **and** both elevators) | cmd p2p **0.000°**, actual p2p **36.03°** |
| TAKEOFF ×3 | 0/45 | — (in TAKEOFF the command is never calm for a whole second) |
| rudder, all 9 runs | **0** | — |

### Q3 — Frequency and peak-to-peak amplitude

Frequency from the 50 Hz IMU (Nyquist 25 Hz — the oscillation is comfortably
resolved, not aliased), cross-checked by positive-going zero crossings and by
the joint's own 20 Hz trace:

| Quantity | Value |
|---|---|
| **Oscillation frequency** | **3.72 – 4.53 Hz** across all nine runs |
| zero-crossing period, best-resolved burst | 0.2264 s ± 0.0147 s (n = 11 crossings) → **4.42 Hz** |
| joint angle peak-to-peak, developed cycle | **28.1 – 37.1°** (i.e. roughly **±14 to ±18.5°**) |
| worst single actual-angle excursion | **left/right elevator, FBWA: −23.12° … +18.88°** |
| worst aileron excursion | **−19.46° … +18.71°** (MANUAL) |
| airframe roll rate p2p, TAKEOFF bursts | 126.2 – 258.4 °/s |
| airframe pitch rate RMS, FBWA steady cruise | **2.63 rad/s = 150.7 °/s**, while mean pitch attitude holds −2.5° |

The reported observation *"actual joint angle swinging roughly −23°…+20°"* is
**confirmed exactly**: the FBWA elevator actual range is −23.12° … +18.88°.

### Q4 — Is the 300 °/s rate limit genuinely and repeatedly hit?

**Yes.** `max_rate_rad_s = 5.2360 rad/s = 300.00 °/s`
(`docs/source_of_truth/controls/actuator_v1_config.yaml`, `V1_PROVISIONAL`).

Airborne rate-pinned counts (`|actual_rate| ≥ 0.999 × 5.2360 rad/s`), summed
over three runs per mode:

| Surface | TAKEOFF ×3 | MANUAL ×3 | FBWA ×3 |
|---|---|---|---|
| left aileron | 50 / 2732 (**1.83 %**) | 1390 / 3184 (**43.66 %**) | 2426 / 3201 (**75.79 %**) |
| right aileron | 41 / 2732 (1.50 %) | 1388 / 3184 (43.59 %) | 2425 / 3201 (75.76 %) |
| left elevator | 0 (0.00 %) | 1669 / 3184 (**52.42 %**) | 2753 / 3201 (**86.00 %**) |
| right elevator | 0 (0.00 %) | 1669 / 3184 (52.42 %) | 2753 / 3201 (86.00 %) |
| **rudder** | **0 (0.00 %)** | **0 (0.00 %)** | **0 (0.00 %)** |

Within individual bursts the duty reaches **100.0 %** (FBWA elevators). Total
across the whole study: **16 564 rate-pinned surface-samples out of 74 975
examined.** All counts are lower bounds (20 Hz observation of a 1 kHz limit).

The limit is not merely touched — it *sets* the motion. For a fully
slew-rate-saturated triangular limit cycle of peak-to-peak `P` at slew rate
`R`, one period covers `2P` of travel, so `f = R/(2P)`. Measured against that:

Over all 188 surface-bursts with ≥ 20 % rate-pinned duty:

| population | f_measured / f_predicted (`R/2P`) |
|---|---|
| all 188 | min **0.722**, median **0.813**, max **1.016** |
| the 147 with duty ≥ 70 % | min 0.752, median 0.806, max **1.016** |
| best-agreeing pairs (MANUAL ailerons/elevators, duty 71–84 %) | **0.95 – 1.02** |

The measured frequency is consistently **0–28 % below** the pure-triangle
prediction and never above it by more than 1.6 %. That is the expected
direction and the expected size: a real cycle is not perfectly triangular —
part of each half-cycle is spent off the velocity limit — so `R/(2P)` is an
upper bound that is approached, not exceeded. The amplitude and frequency of
this oscillation are therefore set by the joint **velocity limit**, not by any
command.

### Q5 — Does the effort limit / clamp engage?

**No. Never. Not once.**

`effort_clamp_active = 0` in **all 74 975 surface-samples** examined —
9 runs × 5 surfaces airborne (45 585) plus 6 sign scenarios × 5 surfaces
(29 390). `target_clamp_active` (the ±45° travel clamp) is likewise 0 in all
of them, and no joint reaches the ±45° mechanical limit
(`actual_hits_mechanical_limit = False` everywhere).

`max_effort_nm = 0.20 N·m` is therefore **never the binding constraint**. The
binding constraint is `max_rate_rad_s`. `EFFORT_LIMIT_INTERACTION` is refuted
by direct measurement.

### Q6 — Phase relationship, left vs right

Measured as the phase of a single-frequency least-squares sinusoid fit of each
joint's *actual* angle at the burst's own f₀:

| Pair | commanded relationship | measured phase difference | amplitude ratio |
|---|---|---|---|
| left − right **aileron** (TAKEOFF, 4 bursts) | ANTI-PHASE (differential) | **−178.8°, −178.5°, −176.9°, −175.8°, +179.1°** | 0.99 – 1.05 |
| left − right **aileron** (FBWA / MANUAL, `cmd ≡ 0`) | none — no command at all | **IN-PHASE**; `actual_angle` agrees to **≤ 0.01°** sample by sample | 1.000 |
| left − right **elevator** (TAKEOFF) | IN-PHASE (`cmd` bit-identical) | **−154.2°, −153.9°, −148.4°, −63.3°, +179.0°** | 0.83 – 3.31 |
| left − right **elevator** (FBWA / MANUAL, `cmd ≡ 0`) | none | IN-PHASE, agree to ≤ 0.01° | 1.000 |

Two things follow, and they are not the same thing:

1. **This is not independent per-joint numerical chatter.** In the
   zero-command case the two ailerons, and the two elevators, are *identical
   to three or four decimal places* while being separate joints with separate
   PID states. They are driven by a **common** input.
2. **In the TAKEOFF case the two elevators, given a bit-identical command,
   move differently** (phase differing by up to 154°, amplitude ratio up to
   3.3). The elevator pair is not tracking its shared command; each joint is
   carrying its own share of a disturbance-driven cycle on top of it.

### Q7 — Is the joint oscillation at the same frequency as the airframe vibration?

**Yes — same frequency, and the airframe vibration is a consequence of it.**

| Channel | peak frequency, TAKEOFF run 1 burst | run 2 | run 3 |
|---|---|---|---|
| airframe roll rate `IMU.GyrX` (50 Hz) | 4.371 Hz | 3.853 / 4.414 Hz | 3.786 / 3.720 Hz |
| ArduPilot aileron output `RCOU.C1` (25 Hz) | 4.361 Hz | 3.869 / 4.435 Hz | 3.818 / 3.774 Hz |
| left aileron **actual** joint angle (20 Hz) | 4.40 Hz | 3.92 / 4.45 Hz | 3.81 / 3.82 Hz |
| aero `Cl` (20 Hz) | 4.38 Hz | 3.93 / 4.44 Hz | 3.84 / 3.81 Hz |

All four agree to within the resolution of the coarsest channel. In the
zero-command FBWA/MANUAL cases the joints run at 3.86–4.53 Hz and the airframe
*pitch* rate runs at the same frequency, while the airframe *roll* rate stays
at 0.004 °/s — because the two ailerons move **in phase**, so
`delta_a = 0.5·(θ_right − θ_left) ≈ 0` and the aileron cycle produces almost no
rolling moment (`Cl` amplitude **6e-5 to 1.8e-4**). The elevator cycle does
**not** cancel (`delta_e = −0.5·(θ_LE + θ_RE) = −θ_E`), which is why the
airframe shakes in pitch and not in roll.

### Q8 — Do the aero moments actually follow this oscillation?

**Yes**, and by exactly the path the sign convention predicts.

TAKEOFF bursts (where the ailerons *are* differential, so `delta_a ≠ 0`):

| run/burst | `Cl` amp @ f₀ | `Cl` p2p | `Cl` peak freq | `Mx` amp = `Cl·q̄·S·b` |
|---|---|---|---|---|
| 1 / 67.0–70.0 s | 0.0401 | 0.174 | 4.38 Hz | **9.81 N·m** |
| 2 / 60.8–63.8 s | 0.0375 | 0.232 | 3.93 Hz | 8.86 N·m |
| 2 / 66.8–69.3 s | 0.0425 | 0.166 | 4.44 Hz | 10.73 N·m |
| 3 / 61.7–64.2 s | 0.0490 | 0.232 | 3.84 Hz | **13.20 N·m** |
| 3 / 67.7–70.7 s | 0.0433 | 0.220 | 3.81 Hz | 10.63 N·m |

Against `Ixx = 0.7284 kg·m²` (`model/model.sdf` base link, quoted in
`tests/gazebo/scripts/manual_takeoff_live_lib.py`) that is a roll-acceleration
amplitude of **12.2 – 18.1 rad/s²** at f₀, in the same band as the measured
value; the closure is order-of-magnitude only and is *not* used as a criterion
here, because `Cl` also carries the `Clp` roll-damping term at the same
frequency and the airframe signal is not a pure tone
(`peak_variance_fraction` 0.31–0.59).

In the zero-aileron-command FBWA/MANUAL cases the two coefficients separate by
three orders of magnitude, exactly as the two extraction conventions predict:
across the 47 bursts in those six runs, `Cm` peak-to-peak is **0.749 – 0.943**
(amplitude at f₀ up to 0.399) while `Cl` peak-to-peak is
**2.4e-5 – 1.4e-3** (amplitude at f₀ 3e-6 – 1.9e-4). The elevator cycle drives
the ±150 °/s pitch vibration; the aileron cycle cancels in `delta_a` and
produces essentially nothing.

### Q9 — Does any of this contradict the previously measured actuator τ ≈ 0.25 s?

**There is no measured actuator τ ≈ 0.25 s in this repository.** A grep of
`docs/` finds `0.25` only as `RLL_RATE_P` / `PTCH_RATE_P` = 0.25 (the ArduPlane
*roll/pitch rate proportional gain* in `config/ardupilot/falcon_v2_sitl.parm`),
which is a controller gain, not an actuator time constant. That value is
almost certainly the source of the "0.25" in the reported observation, and it
is **not** an actuator time constant. Recorded as `DATA_REQUIRED` if a real
τ ≈ 0.25 s measurement is believed to exist elsewhere.

The actuator time constants that **are** on record contradict nothing and in
fact predict this failure:

- `docs/test_results/2026-08-24_actuator_servo_model_v1_test_report.md` §"Settling-time
  sanity check": peak deviation at **t + 0.06 … 0.15 s**, settling to a
  0.1–0.15° band in **0.84 – 1.11 s**, dominated by the deliberately slow
  integral pole at **−2.09 rad/s (τ ≈ 0.48 s)**.
- `actuator_v1_config.yaml` `control_law.derivation_note`: the PD pair was
  pole-placed at `ω_n = 100 rad/s`, `ζ = 1.0`; `integral_derivation_note`:
  closed-loop poles **−113.29, −84.63, −2.09 rad/s**, all real.

Two consequences, both measured here:

1. The **dominant** actuator mode is the −2.09 rad/s (0.33 Hz) integral tail.
   The disturbance driving this defect is at **~4 Hz = ~26 rad/s**, an order of
   magnitude above it. The integral action that fixed the 2026-08-24 gravity
   droop cannot follow a 4 Hz disturbance, by its own design.
2. The measured closed-loop response of the real joint at f₀ does **not** match
   the documented servo model. Evaluating the documented law
   `θ/r (s) = (kp·b·s + ki)/(I·s³ + kd·s² + kp·s + ki)` at f₀ with
   `I = I_ref = 3.4788e-07 kg·m²` gives **gain 0.68–0.69, phase −29° to −34°**.
   The TAKEOFF measurement gives **gain 1.44–1.70, phase −60° to −85°** — about
   **2.2× the gain and 2.1× the lag** the model predicts. That excess is *not*
   a servo transfer function: it is the limit cycle riding on top of the
   commanded motion (the f₀ sinusoid fit leaves 33–65 % of the actual-angle
   variance unexplained). It is reported because it is exactly what the
   ArduPilot roll loop sees, and it is destabilising to that loop.

---

## 3. Root-cause classification

### `ACTUATOR_CONTROL_LIMIT_CYCLE`

The actuator control loop cannot hold its own joint against a hinge
disturbance that varies faster than its dominant pole. Once excited it enters
a self-sustaining, amplitude-saturated limit cycle whose amplitude and
frequency are set by the joint **velocity limit**, and which is completely
independent of the command.

Chosen over the alternatives on evidence, not preference:

| Candidate | Verdict | Evidence |
|---|---|---|
| `ARDUPILOT_COMMAND_OSCILLATION` | **REFUTED as origin** | MANUAL: aileron `cmd_rad` ≡ 0.000000 rad and `RCOU.C1` ≡ 1500 µs for the whole airborne segment; FBWA: aileron `cmd` p2p ≤ 0.06° = one PWM µs for 52 s — while in both the aileron joints run ±16° at 300 °/s |
| `EFFORT_LIMIT_INTERACTION` | **REFUTED** | `effort_clamp_active = 0` in 74 975/74 975 samples; `max_effort_nm` never binds |
| `GAZEBO_SOLVER_NUMERICAL` | **REFUTED as the oscillation itself** | the motion is fully resolved at ~4 Hz: RMS(finite-difference rate)/RMS(reported rate) = **0.78 – 0.98**, and the rate amplitude measured at f₀ is within ~10 % of `2π·f₀·A_angle`. If large motion existed above the 10 Hz diagnostics Nyquist those two ratios would be far from 1. (See §5 for the one genuinely numerical *contributing condition*.) |
| `RATE_LIMIT_INTERACTION` | **CONTRIBUTING, not the cause** | the velocity limit sets the amplitude and frequency (`f ≈ R/2P` to within 2 % at high duty) and is on 43–86 % of samples — but removing it would make the excursions **larger**, not smaller. It bounds the failure; it does not create it. |
| `MIXED` | rejected | the FBWA/MANUAL zero-command case isolates a single origin cleanly; calling it MIXED would blur a result that is not blurred |
| `NOT_REPRODUCED` | rejected | reproduced in 9 of 9 airborne records |

---

## 4. The mechanism, as far as the data supports it

**Measured, not inferred:**

1. **Hinge-axis discrimination.** From `model/model.sdf`:
   `left_aileron_joint` axis `(0.000994, 0.999899, 0.014146)` and
   `left_elevator_joint` axis `(0.002983, 0.999993, −0.002072)` are **+Y
   dominant** — the body pitch axis. `rudder_joint` axis
   `(0.010378, 0.000000, 0.999946)` is **+Z dominant**. The four +Y-axis
   surfaces oscillate; the one +Z-axis surface does not, in the same record,
   under the same command (rudder airborne actual range across all six
   zero-command runs: **−0.03° … +0.03°**, 0 rate-pinned samples out of 6385).
   The rudder is a built-in control experiment.
2. **The driver is base pitch angular acceleration.** Binning every 1 s
   airborne window across all nine runs by the base pitch angular
   acceleration RMS `α_y` (finite difference of the 50 Hz `GyrY`):

   | `α_y` RMS (rad/s²) | n windows | left-aileron `cmd` p2p | left-aileron **actual** p2p | rate-pinned | **rudder** actual p2p |
   |---|---|---|---|---|---|
   | 0 – 1 | 148 | 0.69° | **1.39°** | 0.1 % | 0.278° |
   | 1 – 3 | 17 | 6.60° | 10.88° | 10.9 % | 1.637° |
   | 3 – 10 | 9 | 9.48° | 16.97° | 6.1 % | 2.918° |
   | 10 – 30 | 20 | **0.00°** | 20.25° | 6.5 % | **0.005°** |
   | 30 – 60 | 52 | **0.00°** | **31.77°** | 52.3 % | **0.009°** |
   | 60 – 200 | 207 | **0.01°** | **29.72°** | 78.0 % | **0.007°** |

   (The larger rudder numbers in the first three bins are the TAKEOFF runs,
   where the rudder is genuinely commanded.)
3. **It saturates, so it is a limit cycle and not a forced response.** As
   `α_y` rises from the 30–60 band to the 60–200 band — a factor > 2 in the
   driver — the joint amplitude does **not** grow (31.8° → 29.7°). Only the
   rate-limit duty rises (52 % → 78 %).
4. **The sustaining feedback path is through the elevator only.**
   `delta_e = −0.5·(θ_LE + θ_RE)` does not cancel, so the elevator cycle
   produces a real `Cm` oscillation (p2p 0.75 – 0.94) → real base pitch angular
   acceleration → back into the +Y elevator hinges. The ailerons are passive
   passengers: their common-mode motion cancels in
   `delta_a = 0.5·(θ_R − θ_L)`, giving `Cl` ≈ 1e-4 and airframe roll rate
   0.004 °/s.
5. **The actuator has essentially no authority to resist it.**
   `kp = 0.0034788 N·m/rad` (`actuator_v1_config.yaml`) is a restoring torque
   of **6.07e-5 N·m per degree** — at a full 15° excursion, 9.1e-4 N·m,
   i.e. **0.46 % of the configured `max_effort_nm = 0.20 N·m`**. The joints
   carry **no `<dynamics><damping>` and no `<friction>` element at all** in
   `model/model.sdf`, so the only damping in the loop is
   `kd = 6.9576e-05 N·m/(rad/s)` — 3.6e-4 N·m even at the full 300 °/s.

**Why the gains are that small — the documented, self-declared cause:**
`kp`, `kd` and `ki` were pole-placed against
`I_ref = 3.4788e-07 kg·m²`, which is `model/model.sdf`'s
`left_aileron <inertial><inertia><iyy>` for a link whose mass is
**`0.001 kg`, explicitly labelled `TEMPORARY_NUMERICAL_MASS`** — a
the structural layer-owned numerical placeholder, not a physical claim. The
config file says so itself, in `control_law.placeholder_inertia_note`:

> "Once real per-surface component mass/inertia data exists
> (geometry-structure-owned), kp/kd/ki here MUST be re-derived against that
> real inertia … reusing these specific numbers unmodified once real inertia
> data lands would be an error, not a convenience."

This stage is the first time that warning has been cashed in by a measurement.

**Explicitly `DATA_REQUIRED` — not inferred here:** the *applied joint torque*
is not published (§1.1), and the hinge-to-CoM offset of each control-surface
link is the structural layer data that this stage did not measure. Therefore
the split of the driving hinge torque between (a) inertial reaction to base
angular acceleration, (b) inertial reaction to base linear acceleration through
the link CoM offset, and (c) gravity, **cannot be separated from the existing
records** and no split is asserted. What is measured is the +Y/+Z axis
discrimination, the driver correlation, the amplitude saturation, and the
rate-limit duty.

---

## 5. One genuinely numerical contributing condition (named, not blamed)

The oscillation itself is not solver noise (§3). But the *conditions* are
numerically hostile and this should be on the record for the structural layer
and the simulation-test stage:

- control-surface link mass `0.001 kg` vs. aircraft mass `6.000 kg` — a mass
  ratio of **6000 : 1**;
- link `iyy` `3.4788e-07 kg·m²` vs. base-link `Iyy` `0.2507 kg·m²` — an inertia
  ratio of **7.2e5 : 1** about the shared hinge axis;
- the joint velocity limit is enforced as a physics-engine constraint
  (`ActuatorSystem.cc:127`) on that ill-conditioned pair, at
  `max_step_size = 0.001 s` with `dartsim`.

That is an environment in which a very soft joint controller and a hard
velocity constraint will interact badly. It is a contributing condition, not
the measured cause.

---

## 6. Scope correction to `i3`

`i3` recorded "**exactly 0 samples during every ground roll** and 0 of
4890–4905 samples in every sign scenario — airborne only". Both halves are
**re-confirmed here independently**:

- ground roll: this stage's airborne census starts at the de-aliased liftoff
  sample, and the whole-flight pinned counts above are airborne-only;
- sign scenarios A–F re-measured with this stage's own pin criterion:
  **0 rate-pinned and 0 effort-clamped in all 4890–4905 surface-samples of
  each of the six** (978–981 timestamps × 5 surfaces),
  max rate 0.10–15.0 °/s on the four wing/tail surfaces, and `actual` p2p
  tracking `cmd` p2p to within 0.06–0.36° (e.g. A: cmd 3.664° → actual 3.344°).
  Mean airspeed 17.4 – 20.3 m/s. **This is what a healthy actuator looks like
  in this model, in the same repository, on the same day.**

What `i3` got wrong is **scope**, not fact: it quantified only the three
TAKEOFF-mode runs (0.5–3.0 % pinned, transient bursts) and therefore described
the defect as an intermittent burst. On the six MANUAL/FBWA runs *from the same
stage* it is 43.6–86.0 % of airborne samples, and in FBWA it is **continuous
from within 0.25 s of the liftoff rotation to the end of the record (52 s)** — the aircraft cruises
straight and level at 33.7 m/s with a steady −2.5° pitch attitude while its
pitch rate oscillates at 150.7 °/s RMS. That is the vibration/judder in the
reported observation, and it invalidates any airborne dynamic measurement taken
from those records.

---

## 7. Recommended FIX DIRECTION — **not applied**

No parameter was changed by this stage. The following is a direction for a
future, separately-authorised stage, listed in the order `CLAUDE.md` §"Simulation
tuning policy" requires (verify before tuning), and explicitly **not** a
control-authority adjustment made to pass a test.

1. **Re-derive `kp`/`kd`/`ki` against real control-surface inertia — the
   documented prerequisite.** The gains are correct for the plant they were
   designed against and wrong for the plant that exists. This requires
   the structural layer to replace `TEMPORARY_NUMERICAL_MASS = 0.001 kg` with
   real per-surface mass/inertia first; until then the servo bandwidth is
   pinned by a fictitious 1-gram link. `DATA_REQUIRED`: real control-surface
   mass, CoM and inertia. **This is the primary item; everything below is
   secondary to it.**
2. **Give the actuator a disturbance-rejection specification, then check the
   gains against it.** The present design specifies only a step response
   (`ω_n = 100 rad/s`, `ζ = 1`, `Ti = 0.5 s`). It has never been specified or
   tested against an oscillating hinge disturbance. The measured failure is
   entirely in that untested regime.
3. **Make the anti-windup rate-limit-aware.** `PidEffort()`
   (`plugins/actuators/ActuatorModel.hh:344`) freezes the integrator on the
   ±45° travel clamp or on effort saturation. Neither ever occurs here
   (0/74 975). The condition that *does* occur — the joint velocity sitting on
   `max_rate_rad_s` — is invisible to the anti-windup because that limit is
   enforced by the physics engine, not by the plugin.
4. **Publish the applied joint torque in the diagnostics.** The single most
   important channel for diagnosing this is the one that does not exist
   (§1.1). Adding a 36th field per surface would have made this stage a
   direct measurement instead of an inference. Note `actuator_v1_config.yaml`
   deliberately avoided this to keep `actuator_lib.py`'s fixed-35-field parser
   working, so it is a coordinated the controls layer + the simulation-test stage
   change.
5. **Consider declaring real joint `<dynamics><damping>` from servo data.**
   The joints currently have none. `DATA_REQUIRED`: servo gearbox
   damping/friction. This is the structural layer's element to write and must
   come from a datasheet or measurement, not from a value chosen to damp the
   oscillation.
6. **Raise the diagnostics rate for actuator-dynamics work.** 20 Hz
   (Nyquist 10 Hz) is adequate for this 4 Hz cycle but leaves no margin, and
   it is the reason every pinned count here is a lower bound. the simulation-test stage
   owns the harness side.

**Explicitly NOT recommended:** raising `max_rate_rad_s` (it is
`V1_PROVISIONAL` and would enlarge the excursions), lowering it, changing
`max_effort_nm` (never binds), touching any ArduPilot PID, or touching mass,
CG, inertia or any aerodynamic coefficient. None of those is the cause and all
of them are forbidden shortcuts under `CLAUDE.md`.

---

## 8. Provenance and change record

**Files created by this stage — exactly two:**

- `tests/gazebo/scripts/analyze_airborne_actuator_limit_cycle.py` — read-only
  offline analysis, launches no simulation, writes only its own result JSON.
- `tests/gazebo/results/airborne_actuator_limit_cycle_analysis.json` — full
  per-run, per-burst, per-surface output backing every number above.

**Files modified:** exactly one — this document (new).

**Read only, never written:** `codex/` (untouched entirely),
`docs/source_of_truth/`, `model/model.sdf`, `plugins/`,
`config/ardupilot/falcon_v2_sitl.parm`, `docs/validation/`, every world file,
every pre-existing test script, and every pre-existing result JSON and
dataflash log.

**Constants used, with provenance:**

| Constant | Value | Source |
|---|---|---|
| `max_rate_rad_s` | 5.2360 rad/s (300.00 °/s) | `docs/source_of_truth/controls/actuator_v1_config.yaml`, `V1_PROVISIONAL` |
| `max_effort_nm` | 0.20 N·m | same, `V1_PROVISIONAL` |
| `min/max_angle_rad` | ∓0.7853981634 rad | same, `USER_CONFIRMED_MECHANICAL_CAPABILITY` |
| `kp`, `kd`, `ki`, `sp_weight_b` | 0.0034788, 6.9576e-05, 6.9576e-03, 0.7 | same, `control_law` |
| `I_ref` | 3.4788e-07 kg·m² | same, `control_law.derivation_note`; equals `model/model.sdf` `left_aileron <iyy>` |
| control-surface link mass | 0.001 kg `TEMPORARY_NUMERICAL_MASS` | `model/model.sdf` |
| joint axes | as quoted in §4 | `model/model.sdf` |
| `Ixx` | 0.7284 kg·m² | `model/model.sdf` base link, quoted in `tests/gazebo/scripts/manual_takeoff_live_lib.py` |
| wing area / span | 0.4514 m² / 2.093 m | `CLAUDE.md`, manufacturer geometry |
| PWM → joint radians | `1.5707963268 × ((pwm−800)/1400 − 0.5)` | `model/model.sdf` `ArduPilotPlugin <control>`, via `manual_takeoff_live_lib.py` |
| `RLL_RATE_P/I/D/FF` | 0.25 / 0.125 / 0.002 / 0.125 | `config/ardupilot/falcon_v2_sitl.parm`, `FALCON_V2_MANUFACTURER_INITIAL_PID` |
| diagnostics rate | 20.0 Hz | `actuator_v1_config.yaml` `command_interface.diagnostics_rate_hz` |

**Units:** SI throughout the analysis; degrees appear only at report
boundaries, always labelled.

**Markers used:** `DATA_REQUIRED` — actuator effort/torque channel;
rate-clamp flag; a real actuator τ ≈ 0.25 s measurement; real control-surface
mass/CoM/inertia; servo gearbox damping. `ASSUMPTION` — the seven analysis
thresholds in §1.2, each with its stated basis. `TEMPORARY` — the
`TEMPORARY_NUMERICAL_MASS = 0.001 kg` link mass this defect turns out to
depend on, carried forward from `model/model.sdf` unchanged.

**Next in the chain:** simulation test, then independent engineering review.

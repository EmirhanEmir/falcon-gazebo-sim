# VALIDATION — ACTUATOR_CONTROL_LIMIT_CYCLE_FIX (2026-09-12/13)

Reviewer: `validation` (read-only). Persisted by the main session verbatim from the agent's return —
`validation` has no Write/Edit tool by design and edited nothing.

Audited: `docs/test_results/2026-09-12_actuator_limit_cycle_{root_cause_and_fix_proposal,fix_implementation,fix_verification}.md`; `docs/source_of_truth/controls/actuator_v1_config.yaml` (diff vs HEAD); `plugins/actuators/{ActuatorModel.hh,ActuatorSystem.cc,test/actuator_model_selftest.cc}`; `tests/gazebo/results/actuator_fix_*`; `tests/gazebo/scripts/{test_actuator_standalone_step_response.py,test_actuator_smallsignal_tau.py,test_actuator_fix_yaw_response.py,analyze_actuator_fix_airborne_regression.py,run_actuator_limit_cycle_fix_verification.sh}`; `tests/gazebo/worlds/falcon_v2_actuator_standalone_bench_world.sdf`; `model/model.sdf`; my own `docs/validation/2026-09-11_airborne_actuator_limit_cycle_root_cause_validation.md`.

The three 1 kHz raw records (115–132 MB) referenced by the verification report **were still present** at `/tmp/claude-1000/-home-emirhan-Desktop-FalconV2/d791f088-f001-45d8-802a-39118ca168e8/scratchpad/actuator_fix_highrate_raw/`, so every headline number below was re-derived from raw data, not from prose.

---

## Headline

**The 2026-09-11 `CRITICAL` can be CLOSED.** Verified independently from the raw records, not taken from the report.

**The root cause is PROVEN in its inertia-reference part and PARTIALLY proven in its causal attribution.** The parallel-axis error is real, definitional and exactly reproduced. But by the documents' own numbers it accounts for only **31 %** of the correction; the other **69 %** is F2, an `ASSUMPTION`-labelled bandwidth increase. The headline classification under-represents this (V1).

**F1 has zero design freedom — verified.** **F2 is partly requirement-driven and partly back-solved** — the conventional half (10 dB) is genuine and alone forces ω_n ≥ 189.8; the binding half (18 dB at trim) matches the achieved margin to 0.13 dB and is back-solved (V2). The conclusion ω_n = 200 survives either way.

**No forbidden parameter was changed.** Verified at value level, not by reading the diff prose.

**The limit cycle genuinely disappeared. The measurement did not change.** The §7.5 criterion change is a **legitimate correction**, and I can show it more strongly than `gazebo-testing` did.

**My own prior geometry routing item was WRONG.** `controls-integration` is right. I withdraw it plainly below.

---

## 1. Root cause — is it PROVEN or a plausible correlation?

### 1.1 Geometry and parallel-axis arithmetic — reproduced exactly from `model/model.sdf`

Independently recomputed from the SDF (`sdf version="1.9"`, so joint frame = child link frame and axis is expressed in the joint frame; all links have identity rotation, so model/joint/link orientations coincide):

| surface | `n·d` | **`d_perp`** (mine) | claimed | `m·d_perp²` (mine) | claimed |
|---|---|---|---|---|---|
| aileron | 0.236168 m | **26.5748 mm** | 26.575 | 7.06218e-07 | 7.062178e-07 |
| elevator | 0.094647 m | **20.8491 mm** | 20.849 | 4.34686e-07 | 4.346859e-07 |
| rudder | 0.084296 m | **20.1229 mm** | 20.123 | 4.04931e-07 | 4.049313e-07 |

`c = n × d_perp` (mm), mine vs claimed:

| surface | `c_x` | `c_y` | `c_z` |
|---|---|---|---|
| left aileron | **+4.466244** (+4.466) | **−0.375019** (∓0.375) | **+26.194080** (+26.194) |
| left elevator | **−3.834755** (−3.835) | **+0.053902** (±0.054) | **+20.493357** (+20.493) |
| **rudder** | **+0.000000** | **−20.122905** (−20.123) | **+0.000000** |

**The rudder's `c_x` and `c_z` are EXACTLY zero — CONFIRMED, and confirmed as structural, not numerical.** The rudder link origin, its `<inertial><pose>` y-component and its `<axis><xyz>` y-component are all literally `0.000000` in the SDF, so `n` and `d_perp` both lie in the X–Z plane and `n × d_perp` is exactly ±Y. This is the strongest discriminator in the whole investigation and it holds.

### 1.2 Pre-fix poles and ζ — reproduced exactly

With `kp = 0.0034788, kd = 6.9576e-05, ki = 6.9576e-03` against the corrected `I_hinge`:

| surface | ω_n | ζ | roots of `I_h s³ + kd s² + kp s + ki` |
|---|---|---|---|
| aileron | **57.448** | **0.5745** | **−31.9602 ± 46.3201j**, −2.0841 |
| elevator | **75.513** | **0.7551** | −55.9792 ± 48.3223j, −2.0854 |
| rudder | **77.268** | **0.7727** | −58.6602 ± 47.7976j, −2.0855 |

All match the claim to every published digit. The designed triple `−113.28694, −84.62693, −2.08613` also reproduces exactly from `I_ref = 3.4788e-07`. **The config's own claim that "no oscillatory mode is introduced at all" was indeed FALSE of the running system.** This is not an interpretation — it is arithmetic, and it is confirmed.

### 1.3 Loop gain and V_crit — reproduced exactly

`G_qs = [q̄·S·c̄·(dCm/dθ)/Iyy]·|r_x|·m·c_z/kp` with `S = 0.4514`, `c̄ = 0.224`, `Iyy = 0.2507` (all read from `aero_v1_config.yaml` / `model.sdf`):

- elevator MANUAL: 414.5 → 266.6 → 5.464e-03 → **G_qs = 1.5707** (claimed 1.571)
- elevator FBWA: **1.4129** (claimed 1.413); aileron 0.4507 / 0.4114 (claimed 0.451 / 0.411)
- `q̄ = 862.7` ⇒ V = 37.530 m/s; `q̄ = 663.4` ⇒ V = 32.911 m/s — both consistent
- `kp − G·kp = −1.985e-03 N·m/rad` — net negative hinge stiffness, confirmed
- `V_crit = 21.244/√0.888 = **22.544 m/s**` vs documented trim **21.244 m/s** — confirmed

### 1.4 Verdict

**PROVEN, with one important qualification.**

The chain is: (a) SDFormat semantics make `<inertial><inertia>` a CoM-referenced tensor — this is *definitional*, not a hypothesis; (b) the parallel-axis arithmetic is exact and I reproduced it; (c) the resulting ζ/poles are exact; (d) the linear loop predicts the observed limit-cycle frequency to within 5 % at two flight conditions from parameters none of which were fitted to it; (e) the `c_z = 0` rudder discriminator is exact and the rudder was exactly immune; (f) intervention removed the symptom.

The qualification is (f): raising `kp` 6.7–12.1× would have stopped the limit cycle **whether or not** the parallel-axis diagnosis was right. The intervention proves "`kp` was too soft"; it does **not** prove "because the inertia reference was wrong". The parallel-axis part is proven separately and definitionally. Both are true; they are not the same claim, and the causal weight is not where the headline puts it — see V1.

---

## 2. Is the fix principled or outcome-driven?

### 2.1 F1 — no design freedom: CONFIRMED

`kp = ω_n²·I_hinge`, `kd = 2ζω_n·I_hinge`, `ki = kp/Ti` reproduce the committed YAML values to 1.6e-4 % or better on all three gain sets. Back-computed from the committed file: ω_n = 200.0000 / 199.9998 / 200.0000, ζ = 1.000000, roots −219.108 / −178.851 / −2.041 on all five surfaces. `ζ = 1.0` and `Ti = 0.5 s` unchanged. **There is no free parameter in F1.** Confirmed.

### 2.2 F2 — the honest answer

**Requirement ASI-1(a) — 10 dB at 33 m/s — is genuine.** 10 dB is a conventional servoelastic gain margin; it is not chosen to fit. On its own it gives `ω_n ≥ 100·√(1.153/0.32) = **189.82**`, and 200 is the smallest round value above it. If a different conventional margin had been picked the answer would have moved materially (6 dB → 152; 8 dB → 170; 12 dB → 214), so 10 dB is load-bearing — but it is a defensible independent choice, not a fitted one.

**Requirement ASI-1(b) — 18 dB at trim — is back-solved, and it is the BINDING half.** I recomputed:

- achieved at 21.244 m/s: `|L| = 0.124` ⇒ **18.13 dB**; requirement stated as **≥ 18 dB**
- achieved at 33 m/s: `|L| = 0.298` ⇒ **10.52 dB**; requirement stated as **≥ 10 dB**

Both requirement figures sit immediately below the achieved values — the signature of computing the margin and rounding down to the nearest integer dB. And (b) is not, as the config claims, "satisfied with room to spare": solving it gives `ω_n ≥ 100·√(0.496/0.13) = **195.33**`, which is **tighter** than (a)'s 189.82. So the binding constraint is the one with no independent justification, satisfied by 2.4 % in ω_n.

**Plain statement, as requested:** F2 is **partly requirement-driven and partly back-solved**. The half that is genuinely requirement-driven (a) is sufficient on its own to force ω_n ≈ 200, so the *number* is defensible. The half that actually binds (b) was written to fit. The document's own claim that the requirement "would have been correct to state before the defect was known" is true of (a) and not credible of (b). The `ASSUMPTION` label, the `DATA_REQUIRED` restatement, and the explicit "must not be quoted as a servo measurement" are all correct and honest, and materially mitigate this.

### 2.3 How much of the fix is which — the number that matters

At 33 m/s, `max|L|`: pre-fix **2.117** → F1 only **1.153** → F1+F2 **0.298**.

In margin terms: −6.52 dB → −1.24 dB → +10.52 dB. **F1 contributes 5.28 dB (31 %); F2 contributes 11.76 dB (69 %).**

So **69 % of the achieved stability margin rests on an `ASSUMPTION` whose binding requirement is back-solved**, and only 31 % on the proven correctness defect. Both documents say F1 alone is insufficient, but neither states the split. See V1/V2.

---

## 3. Were any forbidden parameters changed? — NO

Verified at value level by parsing `git show HEAD:...yaml` and the working YAML through `yaml.safe_load` and diffing every numeric leaf:

- `git diff --name-only` = exactly the four claimed files. `git status --porcelain` over `model/ config/ plugins/aerodynamics/ plugins/propulsion/ plugins/sensors/ docs/source_of_truth/aerodynamics/ docs/source_of_truth/propulsion/` is **empty**.
- **Every numeric leaf that existed in HEAD is unchanged except `control_law/{kp,kd,ki}`.** Zero numeric keys removed. Twenty added (per-surface gains + `hinge_inertia_kg_m2`).
- `min/max_angle_rad = ∓0.7853981634`, `max_rate_rad_s = 5.236`, `max_effort_nm = 0.2` — **identical on all five surfaces**. The apparent deletions in the diff are the YAML-anchor restructuring; the values are re-added byte-identically.
- `sp_weight_b = 0.7` → 0.7. `command_interface` (the PWM→rad mapping) — no numeric key changed.
- `model/model.sdf` untouched: `<mass>5.9348</mass>`, `<pose>0.168309 0 0.100000 ...</pose>`, joint `<limit>` ±0.7853981634, no `<dynamics>`. **`TEMPORARY_NUMERICAL_MASS = 0.001 kg` was NOT replaced** — confirmed, and correctly so per M1.
- ArduPilot PIDs / TECS: `config/falcon_v2_sitl.parm` untouched.
- **CG duality: clean.** The string `0.0637` / `-0.0210` appears nowhere in the diff. Only the Gazebo/CAD CG `(0.168309, 0, 0.100000)` is used, and it was not touched.
- `EffortResult` gained two fields but **neither is published**; the 35-field layout is unchanged (7 fields × 5 surfaces confirmed in `ActuatorSystem.cc` and in the raw record's `actuator_field_order`).
- Binary provenance is solid — in particular the IEEE-754 `0.999` literal appearing exactly once in the loaded `.so` and zero times in the stale copy is real evidence, not a timestamp inference.

---

## 4. Is the rudder regressed? Is the YAW result real?

**Not regressed.** Like-for-like, same world/model/harness/1 kHz/`z ≥ 1.0 m` window, re-derived by me from the raw records:

| | pre-fix (2026-09-11) | post-fix (mine, from raw) | reported |
|---|---|---|---|
| MANUAL rudder p2p | 0.048958° | **0.002485°** | 0.0025° |
| FBWA rudder p2p | 0.046726° | **0.001518°** | 0.0015° |

20×/31× better. The nominal "25 % higher than 0.002°" is against a *different* record; the report declares it rather than hiding it, which is the right call.

**The YAW excitation is REAL — independently confirmed from the raw record.** Over the ladder span 43.182–63.301 s:

- rudder `cmd` max **45.0000°**; joint angle range **−45.000000 … +45.000000°**; max|rate| **247.9124 °/s** (exact match to the report)
- `|β|` max **57.104°** (exact match), β p2p 108.316°
- `f_y` RMS **2.055**, absmax **4.8536 m/s²** (absmax exact match)
- yaw rate RMS **55.8**, p2p **339.4 °/s**
- baseline 15 s before the ladder: `f_y` RMS **0.00269**, yaw rate RMS **0.0253 °/s** — i.e. ~760×/2200× increase on my own baseline
- **rudder rate-pinned duty over the whole ladder: 0.0000 %**

This is not a test that only appears to excite the rudder. The rudder was slammed to both mechanical stops repeatedly and the airframe sideslipped to 57°.

**And the response matches the mechanism quantitatively.** The quasi-static prediction `θ/f_y = m·d_perp/kp = 0.0495 °/(m/s²)`. With a 2.5 s command-calm lead-in (see §5) my sweep gives a worst rudder tracking-error p2p of **0.242°** against calm-window `f_y` RMS ≈ 2.75 m/s² — the model predicts 0.136–0.240° over that range. That is agreement to within a factor ~1.2–1.8 on a quantity nobody fitted. The excitation-dependence concern was legitimate, it was tested rather than assumed, and the rudder passed it.

---

## 5. Did the limit cycle disappear, or did the measurement change? — IT DISAPPEARED

This is the item I scrutinised hardest.

### 5.1 The headline pre/post comparisons ARE like-for-like

Checked against `airborne_limit_cycle_runtime_verification.json`, not against prose:

| | pre-fix `highrate_MANUAL` | post-fix `MANUAL` |
|---|---|---|
| window | [35.043, 87.502] | [34.987, 87.522] |
| duration | 52.459 s | 52.535 s |
| rate | 1 kHz | 1 kHz |
| airborne criterion | `z ≥ 1.0 m` | `z ≥ 1.0 m` |
| n samples | 52 459 | 52 535 |
| estimator | `actual_p2p_deg`, whole-window p2p | same |

Same for FBWA (52.795 vs 52.807 s). Body rates: pre `q_pitch.rms_deg_s`, post `q_rms_deg_s` — **both whole-segment RMS of the same 100 Hz Gazebo IMU channel over the same window definition**. This is **not** the I8 failure mode (median-of-window-RMS vs whole-segment-RMS). I looked for it specifically and it is not present.

Re-derived by me directly from the raw 1 kHz records:

| surface | cmd uniq | cmd p2p | **actual p2p** | max\|rate\| | pinned | travel clamp | effort clamp |
|---|---|---|---|---|---|---|---|
| left aileron | 1 | 0.000000° | **0.390481°** | 0.1727 °/s | **0.0000 %** | 0 % | 0 % |
| left elevator | 1 | 0.000000° | **0.608083°** | 0.2658 °/s | **0.0000 %** | 0 % | 0 % |
| rudder | 1 | 0.000000° | **0.002485°** | 0.0411 °/s | **0.0000 %** | 0 % | 0 % |

Every published figure reproduces to the last digit. P4, P5, P6, P11, P13 all **PASS** on my own recomputation.

Additional like-for-like pair neither agent reported: from the committed pre-fix 1 kHz excerpt, FBWA elevator **tracking-error** p2p was **44.056°**; post-fix it is **0.4919°** — a 90× reduction on the one channel where the commanded elevator made `actual_p2p` non-comparable.

### 5.2 The §7.5 lead-in change — LEGITIMATE CORRECTION, not a manufactured pass

I reconstructed the offending window from the raw YAW record rather than trusting the description. At **t = 52.249 s** the rudder command steps **+45° → −44.74°** (a full-scale reversal). The Stage-2 setpoint rate limiter then slews the setpoint at 300.0007 °/s; the joint lags to a peak tracking error of **−19.73°** and then decays:

```
t=52.613  err=-16.336      t=52.885  err=-10.039      t=53.158  err= -5.486
t=52.703  err=-13.994      t=52.976  err= -8.344      t=53.249  err= -4.302
t=52.794  err=-11.917      t=53.067  err= -6.832
```

Fitted decay **τ = 0.476 s**, against the known integral pole −2.041 rad/s ⇒ τ = 0.490 s. It is the commanded step settling on the integral pole. The window scores as "command-calm" only because the step lands microseconds *before* the window opens.

**Three independent reasons this is a correction and not a threshold move:**

1. The command genuinely stepped ±90° immediately before the window. The motion is commanded, full stop.
2. In the worst lead-in window: **0 sign changes** of the tracking error, monotone-envelope decay 1.073° → 0.145°, implied **τ = 0.499 s**. The metric it is scored against is `ACT_OSC_P2P_DEG` — an *oscillation* bar. A single-sign exponential decay is not an oscillation.
3. **Decisive:** I swept the lead-in requirement and the residual collapses monotonically —

| lead-in | rudder worst calm tracking-error p2p |
|---|---|
| 0.0 s (raw imported) | **13.402°** |
| 1.0 s (as added) | **0.929°** |
| 2.5 s (≈ 5τ) | **0.242°** |
| 5.0 s | **0.026°** |

A real limit cycle does not vanish as you demand a longer quiet lead-in — it persists at constant amplitude. This one decays as `e^{−lead/τ}` with τ = the integral pole, and converges to 0.026°, the same order as the MANUAL/FBWA rudder figures. **That is the opposite of a manufactured pass.** Both flags are genuinely in the JSON (`n_command_calm_windows_raw_imported_criterion` / `worst_raw_calm_track_err_p2p_deg` alongside the lead-in versions), and the raw value is printed in the report.

I do note (V7) that the criterion change **is** outcome-determining for one sub-metric: under the raw criterion at least one rudder window exceeds the 3.0° bar, so `n_calm_windows_over_ACT_OSC_P2P_DEG = 0` depends on the lead-in. `gazebo-testing` flagged this itself and asked to be checked. It checks out.

### 5.3 Frequency corroboration

Pre-fix spectra peaked at 3.966–3.997 Hz on joint angle and pitch rate. Post-fix they peak at 0.093 Hz (MANUAL) or the lowest resolvable bin (FBWA). The ~4 Hz content is gone. Correctly offered as corroboration only, never as the acceptance criterion — the P7 trap was respected throughout.

---

## 6. Is the airborne vibration reduction real? — YES

Checked for the I8 class of error and for flight-condition confounding:

| | pre-fix | post-fix | dynamic-pressure ratio |
|---|---|---|---|
| FBWA V_mean | 32.967 m/s | 34.085 m/s | post is **7 % harsher** |
| MANUAL V_mean | 32.273 m/s | 32.725 m/s | post is 3 % harsher |
| FBWA q RMS | 148.379 °/s | **1.490 °/s** | — |
| MANUAL q RMS | 126.225 °/s | **31.824 °/s** | — |

The post-fix runs were flown at **equal or slightly higher dynamic pressure**, over near-identical window lengths, with identical estimators on identical channels. The reduction is real and is if anything understated.

**But the MANUAL flight condition is mis-named** (V9). Post-fix MANUAL has `q` mean **−30.201 °/s with ZERO sign changes in 5254 samples** (min −42.6, max −10.5), mean `f_z` = 18.87 m/s² = **1.92 g**, and a 0.093 Hz periodicity. That is a **continuous nose-up rotation — a sustained loop at ~11.9 s per revolution**, not "the uncontrolled MANUAL phugoid". A phugoid oscillates about level flight; this never changes sign. It does not weaken the result — a sustained 1.9 g loop is a *harder* excitation condition — but the label is wrong and the residual 31.8 °/s is a steady rotation rate, not residual vibration.

---

## 7. Residual risk and the top-of-envelope decrease

**The decrease is consistent with the model, and I can show why.** Binning the raw MANUAL record by airspeed and computing the *excitation* alongside the response:

| V band (m/s) | ail p2p | elev p2p | **f_z p2p** | `f_z` at elevator hinge, p2p | quasi-static pred. (elev) | meas/pred | model's 1/\|1−L\| |
|---|---|---|---|---|---|---|---|
| 18.50–22.24 | 0.0748 | 0.1164 | 4.266 | 4.164 | 0.2004 | 0.58 | 1.12 |
| 25.98–29.73 | 0.2756 | 0.4368 | 7.657 | 7.688 | 0.3699 | 1.18 | 1.27 |
| 37.21–40.96 | **0.3879** | **0.6004** | **9.731** | 9.655 | 0.4645 | 1.29 | 1.72 |
| **44.70–48.44** | **0.2743** | **0.3947** | **7.774** | 7.761 | 0.3734 | **1.06** | **2.53** |

The surface amplitude tracks the **excitation** almost exactly: `f_z` p2p falls 20 % from its peak in the top band while the elevator falls 34 %. The measured plant compliance `θ/f_z_hinge` is **0.051–0.062 °/(m/s²)** against prediction P8's **0.0481** — a live confirmation of the fix's own mechanism to 6–29 %, which neither agent claimed.

Crucially, `θ/f_z` measured *inside* the closed loop is the **plant** compliance `m·c_z/kp`, not the closed-loop amplification `1/|1−L|` — because `f_z` is itself a loop-internal signal, not an external disturbance. So a roughly constant meas/pred ratio across the band is **exactly what the model predicts**, and the amplitude decrease is fully explained by the excitation falling. **It does not contradict the V² model.** `gazebo-testing`'s prose ("it tracks the MANUAL phugoid, in which airspeed and normal load factor vary together") is correct in substance and correctly refuses to claim the V² risk is retired.

**However, the envelope reveals a requirement failure (V3).** ASI-1(a) requires ≥ 10 dB at "the highest airspeed present in this repository's own recorded flight data, 33 m/s". The repository's records reach **47.5 m/s pre-fix and 48.441 m/s in this very verification flight**. Computing the margin at the speeds actually flown:

| V (m/s) | \|L\| | margin |
|---|---|---|
| 34.0 | 0.316 | **10.0 dB** ← ASI-1(a) holds only below here |
| 44.70 | 0.547 | 5.2 dB |
| 47.50 | 0.617 | 4.2 dB |
| **48.441** | **0.642** | **3.9 dB** |

So ASI-1(a) is met only up to ~34 m/s, and the fix was verified over an envelope reaching 48.4 m/s where it fails its own stated requirement by 6 dB. The loop is still stable there (`|L| < 1`) and shows no growth — but the requirement as written is not met across the verified envelope, and "33 m/s = the highest airspeed in the records" is factually wrong.

---

## 8. Adjudications specifically requested

### 8.1 §6.3 overcommand back-off — REAL, and worse-diagnosed than reported (V4)

**Confirmed real.** Under a command beyond ±45°, `targetWasClamped` is true every tick, the anti-windup freezes the integrator permanently at `I ≈ 0`, and the equilibrium is `θ → b·sp = 0.7 × 45° = 31.5°`. Live bench: +31.569 / +31.517 / +31.497°. Gain-independent — confirmed (`b·sp` contains no gain), and the fix does improve it 27.8° → 31.5°.

**But the diagnosis is incomplete, and the real defect is sharper.** The anti-windup clause is *mis-specified*. Its stated rationale is that "the rate-limited setpoint can never reach a clamped-away raw command, so integrating against it would wind up for no reachable purpose." That is false: the error is computed against the **clamped setpoint** (±45°), which is perfectly reachable. Freezing on `targetClampActive` therefore disables integral action against a *reachable* target. This is a logic error, not merely an untested path.

**Severity — MAJOR, currently masked by a 2.55e-12 rad coincidence.** I checked the reachability: the ArduPilot transport `cmd_rad = (π/2)·(clamp((pwm−800)/1400,0,1) − 0.5)` produces at most **0.7853981633974483 rad** at the PWM rails, which is **2.55e-12 rad below** the configured clamp threshold **0.7853981634**. So `targetWasClamped` can never become true through the normal command path, and the defect is unreachable in SITL today. That is the reason it has never bitten — and it is a hair trigger. **If anyone ever "tidies" `max_angle_rad` to `math.radians(45)`, every full-deflection command would silently de-rate from 45° to 31.5° with no test failing**, because `ACTUATOR_OVERCOMMAND_CLAMP_TEST` checks only the setpoint. `gazebo-testing`'s adjudication is correct as far as it goes; this is the part it did not reach.

### 8.2 F3 shipped unexercised — ACCEPTABLE, with conditions (INFO)

F3 never fired in flight (rate-pinned duty 0.000 % everywhere — I reproduced this). Shipping it is acceptable here because:
- Its **inertness** is proved, not argued: 0 bitwise differences in 348 704 unsaturated `PidEffort()` cases against the git-HEAD header.
- Its **correctness** is established by construction and by the differential harness (fired on 25 639/51 296 = 49.98 % of saturated cases, exactly the 50 % a direction-conditional test predicts for randomly-signed rate/error).
- I read the code: the clause freezes only when `sign(actualRate) == sign(error)`, i.e. only when the joint is already slewing *toward* the setpoint at max rate. It can never freeze an unwinding integrator, and it cannot fire when a disturbance drives the joint *away* (then rate and error have opposite signs). That is textbook conditional integration and it is correctly specified — unlike the travel-clamp clause in 8.1.
- `gazebo-testing` states plainly that nothing here is a live test of F3.

Condition: F3 must be exercised the first time any scenario produces non-zero rate-pinned duty, and P15 must be re-checked then.

### 8.3 Effort reconstruction — TRUSTWORTHY, and better than reported (V8)

`gazebo-testing` is right that E1's `effort_clamped` self-check has **no discriminating power** (`n_published_effort_clamp_true = 0` everywhere; both sides constant false) and says so in the report *and* in the JSON `note` field. Correct discipline.

E2 is the real corroboration and its exclusion logic is sound: it drops steps where `|rate| ≥ 0.999·max_rate` at `k` or `k−1` (dartsim's velocity constraint injects an unmeasured torque) and where `|θ| ≥ 0.999·max_angle` (position constraint), and counts them (4 286–4 301 of ~85 000 ≈ 5 %, consistent with the ±45° holds).

**E2's gravity model is incomplete, in the direction that makes the agreement better than reported.** It uses `τ_g(θ) = τ_g0·cos θ` only. The exact hinge gravity torque is `A·cos θ + B·sin θ` with:

| surface | A | B | \|B/A\| | omitted term at θ=30° | reported RMS(E2−E1) |
|---|---|---|---|---|---|
| aileron | −2.569639e-04 | +4.381310e-05 | 0.171 | **2.191e-05** | **2.13e-05** |
| elevator | −2.010398e-04 | −3.762028e-05 | 0.187 | 1.881e-05 | 7.16e-06 |

The aileron's entire E1–E2 residual is accounted for by E2's own omitted `sin θ` term. So the effort numbers are trustworthy, and the true E1/E2 agreement is *better* than the quoted 0.6–8.3 %.

**One limitation to record (V11):** E2 uses the same `I_hinge` that F1 derived, so it corroborates the reconstructed control-law output but does **not** independently validate the parallel-axis correction. I tried to find a live discriminator in this stage and **there isn't one**: the small-signal step is dominated by the setpoint rate limiter and the `sp_weight_b`/integral structure (simulating with `I_hinge` vs `I_com` gives t63 = 11 ms vs 10 ms against a measured 11–12 ms), and the gravity transient differs by only 1.4 % in peak value and 5 ms in peak time. That is fine — the parallel-axis claim is *definitional* (SDFormat semantics) plus arithmetic, both of which I verified — but no measurement in this stage confirms it empirically, and that should not be assumed.

### 8.4 My own prior geometry routing item — I WAS WRONG. Withdrawn.

I flagged `k_g = 18.7 mm < d = 27.1 mm` as geometrically implausible and routed it to `geometry-structure`. **`controls-integration` is right on every point and I verified each independently:**

1. **The SDF inertias reproduce their own collision boxes to 5 significant figures.** Aileron `(x²+z²)/12·m = 3.478800e-07` vs `iyy = 3.4788e-07`; elevator `1.753969e-07` vs `1.7540e-07`; rudder `(x²+y²)/12·m = 1.7774e-07` vs `izz = 1.7775e-07`. Confirmed.
2. **`k_g < d_perp` is EXPECTED for a leading-edge-hinged box.** For a uniform box of chord `c` hinged at the LE, `d/k_g = √3 = 1.732`. Measured: **1.417 / 1.574 / 1.508** — all *below* √3, consistent with a hinge slightly aft of the LE plus finite thickness. There is nothing anomalous.
3. **My own flat-plate reference `d/k² = 1.5/c` is the HINGE-referenced form** (`k²_hinge = c²/3` ⇒ `(c/2)/(c²/3) = 1.5/c`). Aileron: `1.5/0.061119 = 24.54 1/m` vs the SDF's hinge-referenced `d_perp/(I_hinge/m) = 25.12 1/m` — agreeing to **2.4 %** (they quote 2.8 % using their slightly different `I_com`; both are right). Elevator 2.5 %, rudder 4.3 %.

**My sanity check was implicitly computed with the parallel-axis term already included — which is precisely the term the gain derivation omitted. It was the gain derivation that disagreed with my check, not the geometry.** The apparent inconsistency was a *symptom* of the defect F1 fixes. Not calling `geometry-structure` was the correct decision; there is no gap in this stage on that account. I withdraw the item.

### 8.5 ±45° mapping preserved — YES

- Config `min/max_angle_rad = ∓0.7853981634` unchanged (value-level verified).
- SDF joint `<limit><lower>/<upper> = ∓0.7853981634` unchanged (`model/model.sdf` untouched).
- PWM→rad transport (`multiplier = π/2`, `offset = −0.5`, 800–2200 µs, `SERVOx_TRIM = 1500`) unchanged; I recomputed it and it maps 800/1500/2200 µs → −45.000/0.000/+45.000°.
- Sign mappings (`CONTROLS.md` §10, `δe = −0.5(θ_LE+θ_RE)`, `δa = +0.5(θ_RA−θ_LA)`, `δr = θ_R`) untouched.
- ±45° reachable: **44.99910–44.99929°** on all five surfaces, steady-state error 0.00071–0.00090°.
- ±60° clamps: no joint angle ever exceeded ±45° in any sample of any run; YAW ladder joint extremes **−45.000000 … +45.000000°** exactly.

One nuance the report discloses correctly: `math.radians(45) = 0.78539816339744828` is fractionally *below* `max_angle_rad = 0.7853981634`, so the ±45° cases test "reachable just inside the limit" and the ±60° cases test the clamp. That is the honest characterisation — and it is the same 2.55e-12 rad gap that currently masks V4.

### 8.6 Prior routed findings — all honoured

| ref | status |
|---|---|
| **M1** (mass-independence) | **HONOURED.** Accepted verbatim, mass **not** replaced (`model/model.sdf` untouched, `TEMPORARY_NUMERICAL_MASS = 0.001 kg` intact), and the mass-independence of `θ_eq/a` is now recorded in the config's `hinge_inertia_derivation_note` and `placeholder_inertia_note`. The fix does **not** rest on replacing the placeholder mass. Confirmed. |
| **M2** (measured τ exists) | **HONOURED.** Withdrawn explicitly, the values reproduced, and §2.7/§6.5 now *explain* the bimodality. |
| **M3** (driver not established, `I·α_y` 50× too weak) | **HONOURED.** `I·α_y = 2.3e-5` conceded as ~50× too weak; the driver corrected to the CoM-offset term acting through `|r_x|·α_y`; the hinge torque is now measured rather than inferred. |
| **M4** (20 Hz estimator unbiased) | **HONOURED.** No argument uses "the true duty must be higher". (The separate "lower bound" in proposal §2.3 is about `τ_dist` under constraint-torque absorption — a different and legitimate claim.) |
| **m5** (0.0007 °/s is rounding, not solver tolerance) | **HONOURED**, explicitly corrected in verification §2, and `max_abs_rate_minus_limit_deg_s` is now reported (negative in every measurement). |
| **m6** (`peak_variance_fraction`) | **HONOURED**, inference withdrawn. |
| **m7** (over-precision) | **HONOURED** — parabolic interpolation added *and* `fft_bin_spacing_hz = 0.019` reported alongside every frequency. |
| **m12** (world/model diff accuracy) | **HONOURED** — the bench world diff is stated exactly including the 22→60-line header replacement; the highrate-model diff corrected to `1461c1497 / 1595c1631` + 36-line header. I re-ran it: stripping the header leaves exactly the two `diagnostics_rate_hz` lines, propulsion untouched at 20.0. |
| **m13** (both changed plugins) | **HONOURED** — established for aerodynamics by direct source reading, not extended by assumption. |
| **m14** (`np.interp` residual is a tautology) | **HONOURED** — explicitly refuses to quote it, and names the real uncontrolled term (per-topic transport latency) as UNMEASURED. |
| **m19** (transcription) | **HONOURED** — I spot-checked ~15 published numbers against the JSONs and the raw records; all match. |
| **T1** (published `Cm` is XFLR5-unflipped) | **HONOURED, and best-handled item in the stage.** The proposal applies the flip explicitly and the analysis JSON now carries a machine-readable `Cm_convention_note` stating that `My` is not formed from it. |
| **I18** (rate/effort limits `V1_PROVISIONAL`) | **HONOURED** — both unchanged, both restated as `V1_PROVISIONAL`/`DATA_REQUIRED`. |

---

## 9. Findings

### V1 — MAJOR — The root-cause headline attributes to F1 causal weight that belongs to F2 → `controls-integration`

By the documents' own numbers, F1 delivers **5.28 dB (31 %)** of the margin improvement and F2 **11.76 dB (69 %)**. F1 alone leaves `V_crit = 30.6 m/s`, below both flown conditions (32.9 / 37.5 m/s) — i.e. **the parallel-axis error alone is not sufficient to explain the limit cycle**; the `ω_n = 100` bandwidth choice was independently inadequate. Both documents say "F1 alone is not enough", but the classification (`INNER_LOOP_GAIN_INSTABILITY … caused by the actuator gains being derived against I_com`) and the config's `hinge_inertia_derivation_note` ("CONSEQUENCE — the defect this fix addresses") both lead with the parallel-axis error as *the* cause. A future reader will over-weight the proven half and under-weight the `ASSUMPTION` half. The 31/69 split should be stated where the root cause is classified.

### V2 — MAJOR — ASI-1's binding half is back-solved, and the config's claim about it is false → `controls-integration`

ASI-1(b) (≥ 18 dB at trim) requires `ω_n ≥ 195.33`; ASI-1(a) (≥ 10 dB at 33 m/s) requires only `ω_n ≥ 189.82`. **(b) is the binding constraint**, contradicting `bandwidth_requirement_note`'s "(b) is then satisfied with room to spare" — it is satisfied by 2.4 % in ω_n / 0.13 dB in margin. And 18 dB is not a conventional figure; it matches the achieved 18.13 dB to 0.13 dB, which is the signature of rounding an achieved result down to the nearest integer dB and calling it a requirement. ASI-1(a)'s 10 dB *is* conventional and independently defensible and on its own forces ω_n = 200, so **the number survives** — but the assertion "this requirement would have been correct to state before the defect was known" is not credible for (b) and should be withdrawn or (b) re-derived.

### V3 — MAJOR — ASI-1's evaluation airspeed is contradicted by the repository's own records, including this stage's → `controls-integration`

ASI-1(a) evaluates at "the highest airspeed present in this repository's own recorded flight data, **33 m/s**". The records reach **47.5 m/s** (pre-fix MANUAL) and **48.441 m/s** in the post-fix MANUAL verification flight. The 10 dB requirement is met only below **~34.0 m/s**; at 48.441 m/s the margin is **3.9 dB**. The loop is still stable there and shows no amplitude growth, but **the fix does not meet its own stated stability requirement over the envelope it was verified across**, and the "33 m/s = highest observed" justification is factually wrong. Either re-state the evaluation point at the true observed maximum, or state explicitly that ASI-1 is a *design-point* requirement with a documented, accepted margin erosion above ~34 m/s.

### V4 — MAJOR — The travel-clamp anti-windup clause is mis-specified; the §6.3 defect is real and masked by a 2.55e-12 rad coincidence → `controls-integration`

Confirmed: overcommand settles at `sp_weight_b × 45° = 31.5°`, gain-independent, pre-existing, improved by the fix (27.8° → 31.5°). `gazebo-testing`'s adjudication is correct. Beyond it: the clause freezes the integrator whenever the *raw command* is clamped, but the error is computed against the **clamped setpoint**, which is reachable — so the clause disables integral action against a reachable target. That is a logic error, not just an untested path. It is currently unreachable because the PWM transport tops out at `0.7853981633974483` rad, **2.55e-12 rad below** the clamp threshold `0.7853981634`. Any normalisation of `max_angle_rad` to `math.radians(45)` would silently de-rate every full-deflection command from 45° to 31.5°, and `ACTUATOR_OVERCOMMAND_CLAMP_TEST` would still pass because it checks only the setpoint. Fix the clause (freeze on *setpoint* unreachability, not raw-command clamping) and extend the self-test to the achieved angle.

### V5 — MAJOR (pre-existing) — Effective large-signal actuator bandwidth is ~0.3 Hz, not 32 Hz → `controls-integration`

Measured: 10–90 % rise **0.52–0.58 s**, settling **1.32–1.37 s**, and the surface reaches only **95.6–96.2 %** of a commanded angle after 1.0 s. Cause: `sp_weight_b = 0.7` (unchanged) leaves the last 30 % to the `Ti = 0.5 s` integral pole (−2.041 rad/s). Pre-existing and essentially unmoved by this fix — but the fix makes the response **strongly bimodal** (fast mode 11–12 ms, last 30 % on a 0.49 s pole), and 0.49 s is a large lag against a 2–4 Hz airframe short period. Any downstream tuning (ArduPilot roll/pitch PID, TECS) that implicitly assumes a fast actuator is being tuned against a slow one. `gazebo-testing` raised this as a §6.5 caveat; I concur and raise its severity. This is now the dominant remaining actuator-fidelity defect.

### V6 — MINOR — `I_com,nn` was evaluated as the raw diagonal element, not `n·I·n`, contradicting the config's own formula → `controls-integration`

The config states `I_hinge = n·(I_com + m(|d|²E − d dᵀ))·n = I_com,nn + m·d_perp²` and then substitutes `iyy`/`izz` for `I_com,nn`. The exact projection is:

| surface | `n·I·n` (exact) | value used (`iyy`/`izz`) | exact `I_hinge` | used `I_hinge` | error |
|---|---|---|---|---|---|
| aileron | 3.517233e-07 | 3.4788e-07 | 1.057941e-06 | 1.054098e-06 | **−0.36 %** |
| elevator | 1.754379e-07 | 1.7540e-07 | 6.101238e-07 | 6.100859e-07 | −0.007 % |
| rudder | 1.779894e-07 | 1.7775e-07 | 5.829207e-07 | 5.826813e-07 | −0.04 % |

The aileron's hinge axis has `n_z = 0.014146` and `izz = 1.9461e-05`, so the off-axis projection contributes 3.9e-09. Effect: aileron `kp` is 0.36 % below its own stated formula, in the *soft* direction. Immaterial against 10–18 dB margins, but the documented formula and the evaluated number differ, which violates the provenance rule. Either evaluate `n·I·n` or state that the diagonal element is used and bound the approximation.

### V7 — MINOR — The 1.0 s lead-in is not derived from the settling time → `gazebo-testing`

The transient it excludes decays with τ ≈ 0.49 s (the integral pole), so a 1.0 s lead-in leaves ~13 % of the peak — the reported 1.335° still contains settling tail, as the report itself says. A principled choice is ~5τ ≈ 2.5 s, which I verified gives **0.242°** and lands on the quasi-static `m·c_y·f_y/kp` prediction. The criterion change is correct in kind; its magnitude was chosen for convenience. Note also that `n_calm_windows_over_ACT_OSC_P2P_DEG = 0` for the rudder **does** depend on the lead-in (the raw criterion's 11.603°/13.402° exceeds the 3.0° bar), so this is load-bearing and should carry a derived, not round-number, lead-in.

### V8 — MINOR — E2's gravity model omits a 17–19 % `sin θ` term → `gazebo-testing`

`tau_impl = Ih*acc - tau_g0*cos(theta)` omits `B·sin θ` where `|B/A|` = 0.171 (aileron) / 0.187 (elevator). At θ = 30° the omitted term is 2.19e-05 / 1.88e-05 N·m, versus the reported RMS(E2−E1) of 2.13e-05 / 7.16e-06. The aileron's entire residual is explained by E2's own approximation. The corroboration is therefore *stronger* than reported, not weaker — but the residual is currently mis-attributed to reconstruction error.

### V9 — MINOR — The post-fix MANUAL condition is a sustained loop, not a phugoid → `gazebo-testing`

`q` mean **−30.201 °/s with ZERO sign changes in 5254 IMU samples** (min −42.6, max −10.5), mean `f_z` = 18.87 m/s² (**1.92 g**), 0.093 Hz ⇒ ~11.9 s per revolution, V cycling 18.5 → 48.4 m/s. That is a continuous nose-up rotation, not "the uncontrolled MANUAL phugoid". The residual 31.824 °/s is a steady rotation rate, not residual oscillation (`p2p/RMS = 1.01`, whereas a sinusoid gives 2.83). The result is unaffected — the condition is *harsher* than the pre-fix one, which strengthens the comparison — but the label is wrong and the §8 airspeed binning should be read as "position within a loop", which is exactly why the amplitude tracks `f_z` rather than V.

### V10 — MINOR — "exactly zero" for the rudder is exact only at zero deflection → `controls-integration`

The rudder's hinge axis has a 0.594° x-tilt (`xyz = 0.010378 0 0.999946`). Consequences: (a) the gravity hinge torque is `0·cos θ + 2.049e-06·sin θ` N·m — zero at θ = 0 but 1.45e-06 N·m at 45°, giving a droop of 0.0036° before integral action; (b) `c_z(θ) ≈ 0.148 mm` at 45° deflection, versus the aileron's 26.194 mm (0.6 %). Both utterly negligible and consistent with the bench's ≤ 0.00074° measurement. The claims are sound; the word "exactly" should read "exactly at zero deflection; ≤ 0.6 % of the aileron's at full deflection".

### V11 — INFO — No measurement in this stage independently discriminates `I_hinge` from `I_com`

E2 uses the F1-derived `I_hinge`, so it validates the reconstructed control-law output but not the inertia. I looked for a live discriminator and found none: the small-signal step is dominated by the setpoint rate limiter and the `sp_weight_b`/integral structure (simulated t63 = 11.0 ms with `I_hinge` vs 10.0 ms with `I_com`; measured 11–12 ms), and the gravity transient differs by only 1.4 % in peak value and 5 ms in peak time. This is **not a defect** — the parallel-axis claim is definitional (SDFormat semantics) plus arithmetic, and I verified both — but it should not be described as empirically confirmed by this stage's measurements.

### I1 — INFO — The raw evidence for every headline number is uncommitted and ephemeral → `gazebo-testing`

The three 1 kHz records (115/128/132 MB) live only in a session scratchpad. They exist today — I used them, and that is the only reason this review could check the raw data rather than the prose. They are the sole primary evidence for P4/P5/P6/P11/P14 and the entire YAW ladder. Once deleted, this stage is auditable only through the derived JSONs. The 2026-09-11 campaign at least committed a 6 s 1 kHz excerpt per run; this stage committed none. Recommend committing equivalent excerpts (the airborne window at 1 kHz for the five joints, plus the YAW ladder, is a few MB compressed).

### I2 — INFO — `DATA_REQUIRED` restated, all still open

ES08MAII (or actually-fitted servo) datasheet: stall torque, no-load speed, gear ratio, **back-drive/holding stiffness**; real per-surface control-surface mass; joint `<damping>`/`<friction>` (all five joints still have **no `<dynamics>` element at all** — the only damping anywhere in the hinge system is `kd·θ̇`); per-surface aerodynamic hinge moments; applied joint torque as a published diagnostic field; a `rate_clamp_active` flag; **`VNE`**. `max_rate_rad_s = 5.2360` and `max_effort_nm = 0.20` remain `V1_PROVISIONAL`.

---

## 10. Can the 2026-09-11 CRITICAL be closed?

**YES — CLOSE IT.**

| criterion | pre-fix | post-fix (my recomputation from raw 1 kHz) | verdict |
|---|---|---|---|
| MANUAL uncommanded aileron p2p | 38.235° | **0.390481°** | PASS (< 2.0°) |
| MANUAL uncommanded elevator p2p | 38.004° | **0.608083°** | PASS |
| FBWA uncommanded aileron p2p | 35.103° | **0.098813°** | PASS (< 2.5°) |
| velocity-clamp duty, all surfaces, all runs | 44.19–85.04 % | **0.0000 %** | PASS (< 5 %) |
| FBWA pitch-rate RMS | 148.379 °/s | **1.490 °/s** | PASS (< 25 °/s) |
| rudder p2p | 0.049 / 0.047° | **0.002485 / 0.001518°** | no regression |
| ~4 Hz spectral content | 3.966–3.997 Hz | absent | corroborating |
| effort / travel clamp | 0 % | 0 %, 0 samples | PASS |

Like-for-like windows, estimators, channels and flight conditions — verified, not assumed. Post-fix dynamic pressure was 3–7 % *higher*. The rudder axis that had never been excited was excited hard and stayed clean. The one criterion change is a legitimate correction that I verified independently and that becomes *more* favourable, not less, when tightened further.

**Conditions on closure — none of these block it, but all must be carried forward as separate items rather than buried inside the closed CRITICAL:**

1. **V3 is the successor risk and should be opened now, not deferred.** ASI-1 is met only below ~34 m/s; the verified envelope reached 48.441 m/s at 3.9 dB; instability returns at ~62 m/s; `VNE` is `DATA_REQUIRED`. "No growth observed up to 48.4 m/s" is the correct and honest claim, and it is not the same as "the V² risk is retired" — `gazebo-testing` says exactly this and is right.
2. **V1/V2 must be recorded before the `ω_n = 200` `ASSUMPTION` hardens into a remembered fact.** 69 % of the margin rests on it, and its binding requirement was written to fit.
3. **V4 must be fixed before anyone normalises `max_angle_rad`.** It is a latent silent de-rating masked by 2.55e-12 rad.
4. **F3 remains unexercised in flight.** Re-check P15 the first time rate-pinned duty is non-zero.
5. **V5 is now the dominant remaining actuator-fidelity defect** and should be scoped next, since it bears on every ArduPilot control-loop result already in the repository.

---

## 11. Routing

| To | Items |
|---|---|
| `controls-integration` | **V1** (root-cause causal weight / 31-69 split), **V2** (ASI-1(b) back-solved; withdraw "room to spare"), **V3** (ASI-1 evaluation airspeed contradicted by the records; margin fails above ~34 m/s), **V4** (travel-clamp anti-windup mis-specified; 2.55e-12 rad mask; self-test blind to the achieved angle), **V5** (effective large-signal bandwidth), **V6** (`n·I·n` vs `iyy`), **V10** ("exactly zero" qualifier) |
| `gazebo-testing` | **V7** (derive the lead-in from 5τ, not a round number), **V8** (complete E2's gravity model), **V9** (loop, not phugoid), **I1** (commit 1 kHz excerpts) |
| `geometry-structure` | **Nothing.** My 2026-09-11 routing item is **withdrawn — I was wrong.** No geometry change is or was required. |
| `aerodynamics` | Nothing new. T1 was honoured; the hinge-moment `DATA_REQUIRED` note (a future `Ch_δ < 0` will erode the ASI-1 margin) is correctly recorded and should be re-checked when a hinge-moment model lands. |
| `propulsion` | Nothing. |

**No `CRITICAL` finding in this stage.** No engineering parameter was changed by me. Nothing in the fix produces wrong physics; the defects found are in causal framing, one requirement's provenance, one mis-specified anti-windup clause that is currently unreachable, one pre-existing bandwidth-fidelity issue, and four small accuracy/labelling items.

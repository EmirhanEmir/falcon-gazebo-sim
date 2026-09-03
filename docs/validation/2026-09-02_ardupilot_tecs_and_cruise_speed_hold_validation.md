# VALIDATION REVIEW — ARDUPLANE_TECS_AND_CRUISE_SPEED_HOLD_VALIDATION

**Reviewer:** `validation` (independent, read-only)
**Date:** 2026-09-02
**Verdict reviewed:** `TECS_CRUISE_SPEED_HOLD_PASS`, 43/43 acceptance criteria
**Review verdict:** measured results upheld; **3 MAJOR items open** → stage does not meet the
"0 unresolved CRITICAL/MAJOR" READY criterion.

> Persisted by the main session on behalf of `validation`, which is read-only by design and
> cannot author files. Content is `validation`'s review verbatim in substance.
> `docs/validation/` is NOT gitignored (unlike `docs/test_results/*`, `.gitignore:54`),
> so this review is tracked.

---

## 1. Independent reproduction

Every headline number was recomputed from the raw 20 Hz record rather than trusting `analyze()`.
**All reproduce exactly.**

| Claim | Reported | Recomputed | Source |
|---|---|---|---|
| A_hold airspeed mean / σ / min | 17.926 / 0.187 / 17.413 | 17.9260 / 0.1874 / 17.4126 | timeseries |
| A_hold speed slope | +0.000088 | +0.000088 m/s² | timeseries |
| A_hold vz / p2p | +0.00103 / 1.296 | +0.001031 / 1.2963 | gz ground truth |
| A_hold throttle | 0.4911 | 0.49109 | propulsion diag |
| Physical pitch / demand | +2.663 / +2.713 | +2.6632 / +2.7129 | gz + MAVLink |
| TECS target airspeed | 17.92 | 17.92147 | MAVLink |
| TECS authority Δ | 0.2936 | 0.1975 → 0.4911, Δ = 0.2936 | recomputed from RC3 = 1258 |

**Cross-check against ArduPlane's own dataflash** (`00000001.BIN`, time-aligned to 0.017 m RMS on
altitude) — the MAVLink-derived numbers agree with TECS's internal state:

| `TECS` field | Dataflash | External measurement | Δ |
|---|---|---|---|
| `h` | 89.1662 | gz z = 89.1623 | 0.004 m |
| `hdem` / `hin` | 89.1802 / 89.1800 (constant) | target locked | — |
| `th` | 0.4916 | Gazebo motor throttle 0.49109 | 0.0005 |
| `ph` | +0.0039 rad = +0.223° | `nav_pitch` +0.2229° | 0.000° |
| `spdem` / `sp` | 18.5131 / 18.5197 | — | −0.0066 |
| `f` (flags) | 0 throughout | no bad-descent / underspeed | — |

`th` agreeing with the Gazebo-side motor throttle to 0.1 % independently proves the
TECS → SRV → ArduPilotPlugin → propulsion-plugin chain is faithful end to end.

---

## 2. Findings

### CRITICAL — none

No sign error, no wrong force application point, no duplicated force or damping, no CG misuse.
**CG duality check: clean** — neither CG value appears in any file touched this stage;
`MASS_KG = 6.000` and `S_REF_M2 = 0.4514` are cited to `CLAUDE.md` and used only for post-hoc
diagnostics, never fed into a physics path.

---

### MAJOR-1 — Atmosphere / altitude-datum mismatch between ArduPlane and Gazebo (undocumented)

**Owner:** `controls-integration`. Not noticed by either preceding agent.

ArduPlane's own log: `Home: -35.363261 149.165230 alt=584.000000m hdg=0.000000` — the **CMAC
default**, *not* the `-O 0,0,0,0` requested at
`tests/gazebo/scripts/run_ardupilot_tecs_cruise_speed_hold.sh:83`. Dataflash `ORGN.Alt = 584.0`,
`POS.Alt = 673.16 m`, `GPS.Lat/Lng = −35.363262 / 149.169`. The Gazebo world declares
`<elevation>0.0</elevation>`, lat/lon 0
(`falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf:58-65`), and the aero plugin uses a fixed
ρ = 1.225 (`aero_v1_config.yaml:70`; confirmed 2·q̄/V² = 1.22500 exactly in the run).

Measured consequence chain:

- ArduPlane believes it is at 673 m AMSL → ISA ρ = 1.148 kg/m³ → `EAS2TAS = 1.0331`
  (`AP_Baro_atmosphere.cpp:235-243`, `_get_EAS2TAS` at `:299`).
- Gazebo true airspeed **17.938 m/s**; `ARSP.Airspeed` (EAS to TECS) **17.931 m/s**;
  TECS `sp` (TAS) **18.520 m/s**.
- TECS's internal TAS is **3.3 % above physical truth**, and specific kinetic energy (½·TAS²)
  **6.7 % above**.

**Does it invalidate this stage? No.** `_TAS_dem` and `_TAS_state` are both scaled by the same
factor, so the speed loop closes on EAS and the aircraft genuinely held ≈17.93 m/s *true* airspeed.
But SKE is inflated while SPE = g·h is not, so the speed/height energy split is skewed ~6.7 % —
exactly the quantity the *next* stage (TECS tuning / `TECS_CLMB_MAX`–`SINK_MIN` sizing) depends on.

`-O 0,0,0,0` has silently failed in **every** SITL launch script since the SITL-prep stage
(`launch_ardupilot_sitl.sh:174`, `run_ardupilot_longitudinal_equilibrium_c3.sh:64`,
`run_ardupilot_fbwa_level_pitch_reference_correction.sh:63`) — pre-existing, not introduced here.
Nothing in `docs/source_of_truth/` mentions 584 m, CMAC, or EAS2TAS.

---

### MAJOR-2 — Project `PitotSystem` plugin is not in the control loop; `SIM_WIND_*` can never act

**Owner:** `controls-integration`. Honestly disclosed by both preceding agents; confirmed here,
with the forward risk raised.

Verified: `grep airspeed` in `ardupilot_gazebo/src/ArduPilotPlugin.cc` returns **nothing** → the
`AIRSPEED` bit (`SIM_JSON.h:188`) is never set → the else-branch at `SIM_JSON.cpp:443-455` runs,
**`wind_ef.zero()` unconditionally**, airspeed from `update_eas_airspeed()`
(`SIM_Aircraft.cpp:1409-1411`). `model/model.sdf:1701` `FalconV2Pitot` publishes to
`/model/falcon_v2/sensors/pitot/airspeed_mps`, which nothing consumes.

**Benign for this stage** — the world has no wind system, so true airspeed = ground speed
(measured 17.9260 vs 17.9348, Δ 0.009 m/s), and the SITL-derived airspeed matches the aero
plugin's V to 0.06 %.

**Not benign later:** `wind_ef` is zeroed *unconditionally*, so in any future wind-in-the-loop test
the Gazebo wind plugin will push the airframe while ArduPlane's airspeed stays wind-blind. That
would silently corrupt the result. **Recorded as a blocker on any wind + TECS stage.**

---

### MAJOR-3 — APC 13x6.5E table domain: no windmilling (negative) thrust; descent is affected

**Owner:** `propulsion`.

The parsed table ends at J ≈ 0.635–0.644 with Ct ≈ 0 (manufacturer PER3 data stops at zero
thrust). `InterpWithinSlice` (`plugins/propulsion/PropulsionModel.hh:~460`) clamps to the last J
and returns `Ct.back()`, so for J > J_max thrust is floored at ≈0 rather than going negative.

| Segment | motor-samples | `interpClamped` | thrust ≡ 0 N | clamped J range |
|---|---|---|---|---|
| A_baseline | 1720 | 10 (0.6 %) | 4 | 0.691–1.457 |
| B_climb / B_hold | 334 / 1340 | 0 / 0 | 0 | — |
| **C_descent_ramp** | 342 | **112 (32.7 %)** | **110** | 0.644–0.769 |
| C_hold | 1152 | 54 (4.7 %) | 54 | 0.654–0.785 |
| **Total** | 4888 | 176 (3.60 %) | 168 | thrust < 0: **0** |

**Throttle equilibrium: not affected** (0.6 % in A_hold). **Cruise and climb conclusions: not
affected.** **Descent: materially affected** — a third of the descent ramp had zero retarding
thrust where a real fixed-pitch prop would windmill, so the model understates descent drag and
overstates TECS's throttle-down braking authority.

The code behaves correctly (it does not extrapolate, and flags `interpClamped`), so this is a
**data-domain gap, not a coding error** — but `PROPULSION.md:347` explicitly *requires* that such a
gap "be reported for the affected J range rather than extrapolated". The affected range
**J ∈ [0.644, 1.457]** is not yet recorded anywhere as `DATA_REQUIRED`. This will corrupt the
planned climb/sink-performance stage that is supposed to size `TECS_SINK_MIN` / `TECS_SINK_MAX`.

---

### MINOR-1 — `SITL_PARAM_MIGRATION.md:225` TECS row is misleading

**Owner:** `controls-integration`.

`controls-integration`'s correction is **right, and verified at source**: `AP_TECS.h:66-68`
`get_pitch_demand()` returns `int32_t(_pitch_dem * 5729.5781f)` — raw, no trim; `pitch_trim_deg` is
referenced in exactly one place, `AP_TECS.cpp:939` inside `_update_throttle_without_airspeed()`,
reached only via the `else` at `AP_TECS.cpp:1355` when `!use_airspeed()` — not taken with
`ARSPD_USE = 1`. The single addition is `Attitude.cpp:244`. The doc's "AUTO/CRUISE/FBWB handle the
same offset consistently" implies TECS accounts for it in the pitch demand; it does not. Its
*conclusion* (no double-count) is correct.

**Empirically closed:** dataflash `TECS.ph` = +0.223° == MAVLink `nav_pitch` +0.2229°, physical
demand +2.713°, measured pitch +2.663° → **exactly one count of `PTCH_TRIM_DEG`, no double-count
and no missing count.** Also verified `KFF_THR2PTCH = 0` (`Parameters.cpp:62`, absent from the
`.parm`), so the third term in `demanded_pitch` is exactly zero.

---

### MINOR-2 — Test's command inversion does not model the `int16_t` truncation

**Owner:** `controls-integration` (author). Already self-reported by `gazebo-testing`.

**Verified exactly at source:** `RC_Channel.h:99/542` `int16_t control_in`; `RC_Channel.cpp:316`
`control_in = pwm_to_range();` truncates the float from `pwm_to_range_dz()` (`:388-402`).
100·(1258 − 1130)/770 = 16.623 → 16 → `target = 12·16 + 1600 = 1792 cm/s = 17.92 m/s`. Dataflash
confirms: `spdem` / EAS2TAS = 18.5131 / 1.0328 = **17.9248**. `rc3_pwm_for_target_airspeed()`
(line 348) uses a float `ci` and the *unrounded* PWM, predicting 18.000 — optimistic by 0.08 m/s,
absorbed by the 0.4 m/s tolerance.

**Is PASS defensible? Yes.** The physics claim is "TECS holds the speed it is commanded to hold",
and it tracked its own demand to +0.005 m/s. The −0.074 m/s is command quantization (grid =
0.12 m/s; 18.000 is not reachable via RC3 with `AIRSPEED_MIN 16` / `MAX 28`), fully root-caused.
It should nonetheless be fixed before any stage needing an exact demand.

---

### MINOR-3 — Inline magic numbers in `verdict()` outside the recorded threshold block

**Owner:** `controls-integration` / `gazebo-testing`. `0.02` (line 935, speed divergence),
`0.7 * ALT_STEP_M` (968/970), `0.2` (976/980, direction bars). All plausible, but absent from the
`thresholds` dict written to the result JSON — contrary to the file's own "every value justified
here" claim and to `CLAUDE.md`'s no-magic-numbers rule.

### MINOR-4 — Two checks pass vacuously on missing data

**Owner:** `controls-integration` / `gazebo-testing`. Lines 970 and 977 use `x is None or …`, and
the B/C checks at 954-963 are only *created* if the window exists. This contradicts the stated
policy at lines 903-905 ("A missing quantity must FAIL its check … never be silently treated as
passing"). Moot in this run (all windows present), but it is a latent false-PASS path.

### MINOR-5 — Quantization figure in the baseline YAML understates the real grid

**Owner:** `controls-integration`. `test_…py:221` and the YAML's worked example imply
1 µs ≈ 0.0156 m/s of demand. True per-µs sensitivity is 0.0156 m/s, but the *achievable* demand
grid after `int16` truncation is **0.12 m/s** — 7.7× coarser.

---

### INFO

- **EKF3 lane switch — dismissal is defensible.** Confirmed `Subsys 24 = EKF_PRIMARY`
  (`AP_Logger.h:136`), a lane change, **not** a failsafe. Dataflash: 0→1 at boot 33.86 s, 1→0 at
  39.70 s; FBWB entry at 30.56 s → **3.30 s and 9.14 s after entry**, both before the A_hold window
  opens at 12.05 s (2.9 s margin). Only two ERR events in the whole 127 s log; none in any analysis
  window. Crucially, altitude analysis uses **gz ground truth**, not the EKF — and `TECS.h` matches
  gz z to 0.004 m, showing no residual EKF bias. The 12 s cutoff was justified a priori
  (1.5 phugoid periods, 2.4 × `TECS_TIME_CONST`), not reverse-fitted around the switch.
- **Window-choice robustness (reviewer's own added check).** vz for `t_seg ≥ 12/20/30 s` =
  +0.00103 / **−0.00180** / +0.00733 m/s with p2p 1.296 → 0.889 → 0.562 m. The sign flips and the
  magnitude stays ~10⁻³ — the altitude-hold result is residual phugoid, not a trend, and is **not**
  an artifact of the transient cutoff. Monotonically shrinking p2p independently confirms decay
  rather than a limit cycle.
- **"FBWA residual sink closed" is true but not a root-cause fix.** A height-error integrator
  naturally closes a sink that an attitude-only mode leaves. The check name could be read as
  implying the underlying FBWA cause was resolved; it was masked, not resolved. The report's prose
  is correctly worded.
- **`detrended_growth` blind spot.** The `s1 > 1e-4` guard means a signal starting at the noise
  floor and growing to ~10⁻³ would report `growing=False`. Irrelevant here (all four signals ×
  three windows decayed, ratios 0.405–0.653).
- **`stdev()` is population (÷n), not sample (÷n−1).** At n = 630 the difference is 0.08 %.
  Immaterial.
- **Statistics / units / frames are correct.** `linreg` is mean-centred OLS (numerically sound);
  `specific_energy_rate = g·vz + V·dV/dt` is the correct d/dt(gh + V²/2); `gamma = asin(vz/V)` with
  `pitch_phys = −(gz euler pitch)` is consistent FLU handling; cm↔m (`aspd_error/100`, `alt_error`
  already m), cd↔deg (`nav_pitch` already deg), rad↔deg conversions all correct. The
  `pitch − (α+γ)` residual of **+0.0017° mean / 0.118° max** is itself a decisive independent proof
  that the FLU / NED / wind-axis conventions are consistently applied — a sign or frame error
  anywhere in that chain would show up as degrees, not thousandths.

---

## 3. Threshold integrity — not reverse-fitted

Every gating threshold was checked against its measured value. **None is marginal**; the tightest
has 1.8× margin:

| Threshold | Bar | Measured | Margin | Basis |
|---|---|---|---|---|
| `fbwa_residual_sink_closed` | 0.078 m/s | 0.00103 | **76×** | prior FBWA published result — an *a priori*, strictly harder bar (TECS has a height integrator, FBWA does not). Non-circular. |
| alt slope | 0.10 | 0.00103 | 97× | prior FBWA stage |
| alt p2p | 5.0 m | 2.179 | 2.3× | declared `ASSUMPTION` / `DATA_REQUIRED`; = ½ the commanded step |
| throttle band | ±0.05 of 0.4957 | 0.0046 | 11× | declared `ASSUMPTION`. Widest band (10 %) — weakest test here, but honest |
| speed mean tol | 0.5 | 0.074 | 6.8× | prior stage |
| TECS authority | 0.10 | 0.2936 | 2.9× | derived from the FBWA / MANUAL passthrough path |
| elevator in hold | 10° | 5.47° | 1.8× | vs ±45° mechanical travel |
| sat run | 2.0 s | 0.0 s | — | `THR_SLEWRATE 100 %/s` ⇒ ≤ 1.0 s legitimate transient |

Every bar traces to a prior published measurement, an ArduPlane firmware constant, or an
explicitly labelled `ASSUMPTION`. Analysis windows exclude transient and are long enough: A_hold
32.95 s = 4 phugoid periods (T_ph = π√2·V/g = 8.23 s, independently verified) = 6.6 ×
`TECS_TIME_CONST`. **No evidence of reverse-fitting.**

Saturation metric sanity: `longest_run_seconds` reports 0.0 for a 1-sample run, so the raw data was
checked directly — whole-flight throttle stayed in **[0.240, 0.577]**, with **0** samples within
1 % of `THR_MIN` (0) or `THR_MAX` (1). The 0.0 s result is genuine, not a metric artifact.

---

## 4. Physics chain — not bypassed (confirmed independently)

- **Nothing changed this stage.** `git status` shows only new untracked test/result files plus a
  2-line `tests/gazebo/README.md` edit. `git diff HEAD` against `config/ardupilot/`, `model/`,
  `plugins/`, `docs/source_of_truth/` is **empty**; no file under those trees has an mtime after
  Aug 30. `falcon_v2_sitl.parm` is still at commit `248c10e`. Independently corroborated *at
  runtime*: all 16 `param_preconditions` true, including `tecs_at_firmware_defaults` and
  `pids_unchanged` — read live over MAVLink, not assumed.
- **Full free 6-DOF.** `run_seg` / `build_sample` are read-only (pose + odometry + diag
  subscribers); the only egress is `RC_OVERRIDE`. The wrench publishers (lines 1280-1281) are used
  **only** in `phase2_teleport_and_verify` and `phase3_hold_to_trim`, and `clear_wrench` is called
  both at the end of `phase3` and at line 1325 — before `flight_sequence`. No pose locking, no
  forced velocity, no kinematic shortcut in world or script.
- **Force balance closes on aero + thrust + gravity alone.** A_hold: T = 5.1842 N, D = 5.1460 N,
  T − D = +0.038 N (0.7 %); L/W = 0.99879. A sample-wise vertical Newton reconstruction gives a
  residual of +0.159 N against a per-sample σ of 2.15 N — statistically indistinguishable from
  zero, and far too small to be a leftover hold wrench (which would be tens of N).
- **FBWB really is a TECS-only, nav-free mode** — every cited line verified in ArduPlane
  V4.8.0-dev, commit `409226a637` (both confirmed): `mode.h:45` FBWB = 6; `Plane.cpp:635` +
  `mode.h` `ModeFBWB::does_auto_throttle() → true`; `Plane.cpp:669` is the **only**
  `update_pitch_throttle` call site in the vehicle (grep-verified); `Attitude.cpp:510-511` throttle
  ← `get_throttle_demand()`; `Attitude.cpp:635-638` `nav_pitch_cd` ← `get_pitch_demand()`;
  `navigation.cpp:450-451` ends `update_fbwb_speed_height()` with both; `ModeFBWB` has **no**
  `does_auto_navigation()` override (base `mode.h:134` → false) and **no** `navigate()`, setting
  `nav_roll_cd` straight from the stick (`mode_fbwb.cpp:18`); `Attitude.cpp:293-301` excludes
  `mode_fbwb` from stick mixing. The CRUISE rejection is also correct (`mode_cruise.cpp` does call
  `nav_controller->update_waypoint()`). Mode integrity held: `custom_mode 6` on **every** sample.

---

## 5. READY criteria

| Criterion | Status | Evidence |
|---|---|---|
| Speed hold bounded / stable | **MET** | σ 0.187, slope +8.8e-5 m/s², never below 16.78 (limit 16.0 / trigger 14.4) or above 19.34 (max 28) |
| Altitude hold bounded / stable | **MET** | vz +0.00103 / +0.00109 / +0.00829; p2p 1.296 / 2.179 / 0.967 m; robust to window choice |
| Throttle / pitch coordination physically correct | **MET** | climb 0.5398 / +6.68°, level 0.4911 / +2.66°, descent 0.4251 / −1.25°; `pitch − (α+γ)` = +0.0017° |
| No NaN / Inf | **MET** | 0 across all windows, all groups |
| No sustained saturation | **MET** | throttle ∈ [0.240, 0.577], 0 samples near either limit; 0 actuator clamps; elevator max 6.37° of ±45° |
| No growing oscillation | **MET** | 12/12 ratios in 0.405–0.653, all decaying; p2p shrinks monotonically |
| Physics chain not bypassed | **MET** | see §4 |
| 0 unresolved CRITICAL / MAJOR | **NOT MET** | MAJOR-1, MAJOR-2, MAJOR-3 open |

**The TECS baseline claim itself is upheld.** The stage does not get a clean READY sign-off until
the three MAJOR items are recorded and dispositioned. None of the three invalidates this stage's
measured results; all three constrain the *next* stage.

---

## 6. Recommendation on TECS parameters

**Change nothing. Baseline-first is correct and the baseline is adequate.**

Every gating metric passes with ≥ 1.8× (typically ≥ 10×) margin; all four monitored signals decay
in all three windows; there is zero saturation and zero clamping; TECS tracked its own speed demand
to +0.005 m/s and held height to centimetres. There is no observable deficiency for a TECS gain to
fix.

The "slow throttle / altitude settling transient" that the baseline YAML's
`TRIM_THROTTLE_DEFAULT_VS_MEASURED_TRIM` open item pre-authorised as a trigger **was not found** —
the integrator absorbed the 4.6-point feed-forward offset well inside the 12 s transient window
(`TECS.th` 0.4916 vs measured trim 0.4957). So **`TRIM_THROTTLE` should not be changed either**,
even though it is the most defensible candidate. Changing any `TECS_*` now would be tuning against
noise.

**Caveat on record:** `TECS_CLMB_MAX = 5.0` / `TECS_SINK_MIN = 2.0` remain generic firmware
defaults, and `AP_TECS.cpp:759` derives `K_thr2STE` directly from them. The YAML's `ASSUMPTION`
flag on this is correct and appropriately actioned as `NONE` this stage.

---

## 7. Recommended next stage

**`SITL_ATMOSPHERE_DATUM_AND_PROP_HIGH_J_DOMAIN_RECONCILIATION`** — must precede any TECS tuning or
climb/sink characterization:

1. `controls-integration` — root-cause why `-O 0,0,0,0` never takes effect
   (`SITL_cmdline.cpp:646-658` `set_start_location` path), reconcile the ArduPlane home datum with
   the world's `<elevation>0.0</elevation>`, and document the resulting EAS2TAS. Also record the
   `wind_ef.zero()` consequence as a hard blocker on any wind + TECS stage, and correct
   `SITL_PARAM_MIGRATION.md:225`.
2. `propulsion` — record J ∈ [0.644, 1.457] as `DATA_REQUIRED` per `PROPULSION.md:347`, and decide
   (with authorization) whether to source windmilling Ct/Cp data or explicitly bound the model's
   validity to J ≤ 0.64.
3. `controls-integration` — fix the `int16` truncation in the command inversion (MINOR-2) and lift
   the inline literals into the recorded threshold block (MINOR-3), and close the vacuous-pass path
   (MINOR-4).

**Then** `TECS_CLIMB_SINK_PERFORMANCE_CHARACTERIZATION` to size `TECS_CLMB_MAX` /
`TECS_SINK_MIN` / `TECS_SINK_MAX` from measured data — which depends on both (1) and (2) being
fixed first, since sink performance is exactly where the prop-domain clamp and the energy scaling
bite.

---

## 8. Assessment of the preceding agents

Both did notably disciplined work. `controls-integration`'s mode-selection evidence is source-cited
and essentially all of it was verified at the quoted lines; the `PTCH_TRIM_DEG` correction against
the migration doc was proactive and **correct**. `gazebo-testing` self-reported the test's own
optimistic derivation rather than hiding it behind a passing tolerance, and preserved the dataflash
specifically so this review could cross-check internal TECS state — that is exactly right. The
three MAJOR items raised here are all things neither agent looked for, and two of them are
pre-existing rather than introduced in this stage.

---

## 9. Key files

- `docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml`
- `tests/gazebo/scripts/test_ardupilot_tecs_cruise_speed_hold.py`
- `tests/gazebo/scripts/run_ardupilot_tecs_cruise_speed_hold.sh` (line 83, `-O 0,0,0,0`)
- `tests/gazebo/results/ardupilot_tecs_cruise_speed_hold_result.json`
- `tests/gazebo/results/ardupilot_tecs_cruise_speed_hold_arduplane_log.txt` (the `Home:` line)
- `docs/source_of_truth/autopilot/SITL_PARAM_MIGRATION.md` (line 225)
- `plugins/propulsion/PropulsionModel.hh` (J clamping)
- `docs/source_of_truth/propulsion/PROPULSION.md` (line 347)

# VALIDATION — ARDUPLANE_TECS_CLIMB_DESCENT_AND_ENERGY_VALIDATION (2026-09-03)

Independent engineering review by `validation`. Read-only: no engineering parameter, threshold,
SDF, plugin, table or config value was edited by this review. Every number below was
re-derived by `validation` from the raw artifacts and from the ArduPlane / AP_TECS / pymavlink
source — not taken from `controls-integration`'s or `gazebo-testing`'s summaries.

Scope: THIS STAGE ONLY. Earlier READY stages were not re-audited.

## 0. Bottom line

| | |
|---|---|
| Wrong physics found (sign / frame / unit / duplicated force or damping / CG conflation) | **NONE** |
| Forbidden parameter changed | **NONE** |
| Was this a genuine TECS closed-loop, free-6-DOF test | **YES** |
| Is `settles_after_climb` a real aircraft control deficiency | **NO — the criterion is not defensible.** A real but separate, non-blocking damping observation exists (V-2) |
| CRITICAL findings | 0 |
| MAJOR findings | 3 (V-1, V-2, V-3) — **V-1 and V-3 are stage-blocking** |
| MINOR findings | 5 |
| Stage status | **NOT_READY** — blocked on V-1 (criterion re-derivation, `controls-integration`) and V-3 (report correction, `gazebo-testing`). The AIRCRAFT is not implicated by either. |

---

## 1. (A) Was this a genuine TECS closed-loop test? — YES, verified from source and raw data

| Claim | Independent evidence |
|---|---|
| FBWB, `custom_mode = 6`, all phases | `analysis.<phase>_full.custom_modes_seen == [6]` for all five phases. Dataflash `MODE` has exactly two records: MANUAL (0) at AP t=9.08 s and FBWB (6) at AP t=33.08 s, Rsn 2 (GCS). No reversion, no failsafe, `ERR` table empty. |
| RC3 set target AIRSPEED, not throttle | `ArduPlane/navigation.cpp:187-189` maps `get_throttle_input()` to `target_airspeed_cm`; `ArduPlane/Attitude.cpp:510` sets the throttle channel from `TECS_controller.get_throttle_demand()`. Measured: RC3 held at 1258 us for all 143 s (manual-passthrough equivalent 0.1975), measured throttle 0.4909 → delta 0.2934. |
| TECS drove pitch too | `ModeFBWB::update()` sets only `nav_roll_cd` from the stick; pitch comes from TECS. `Attitude.cpp:244`: `demanded_pitch = nav_pitch_cd + pitch_trim*100 + throttle*kff_throttle_to_pitch`, with `KFF_THR2PTCH = 0`. |
| Free 6-DOF, no physics bypass | The 0.8 s hold-to-trim wrench (`phase3_hold_to_trim`) ends with `clear_wrench()`; `cruise.run_seg()` publishes **only** `RC_CHANNELS_OVERRIDE` — no wrench, no pose set, no velocity set, no thrust override, no frozen DOF, for the entire 142.96 s / 2746-sample measured flight. Verified by reading `run_seg` and `phase3_hold_to_trim`, not by trusting the docstrings. |
| Normal arming, safe airborne start | `phase1_mavlink_arm.ok = true`, `ground_settle` settled, `phase2_teleport_verify.ok = true`, `phase3` released at u=18.14 m/s, pitch +0.39 deg. |
| Pitot genuinely in the loop | `arduplane` log JSON key list contains **both** `airspeed` and `velocity_wind` → `DataKey::AIRSPEED` set, the `SIM_JSON.cpp` `wind_ef.zero()` branch is NOT taken. `ARSPD_TYPE=100`, `ARSPD_USE=1` read live. This **closes** the 2026-09-02 open MAJOR. |
| Atmosphere datum correct | `Home: ... alt=0.000000m` in the `arduplane` log; `SIM_OPOS_ALT=0` gated. Dataflash `CTUN.E2T` mean **1.00391** (0.99998 … 1.00497), not 1.0331. **Closes** the second 2026-09-02 open MAJOR. |
| Zero wind | `SIM_WIND_SPD/DIR/TURB` read live = 0; max abs(groundspeed − airspeed) 0.271 m/s in P1 hold. |
| No lockstep / no forced timing | `no_time_sync`, `no_lockstep` in the FDM key list; `Forcing use_time_sync=0`. |

Additional independent frame check (`validation`, not in either agent's report):
Gazebo world z vs ArduPlane home-relative altitude agree to **5 mm** (mean +0.0052 m, sd 0.011 m),
and TECS's own internal `h` matches `GLOBAL_POSITION_INT.relative_alt` to **1.7 cm RMS** across the
whole flight. No altitude-datum conflation anywhere.

**CG duality trap: clean.** Neither `(0.168309, 0, 0.100000)` nor `(0.0637, 0, -0.0210)` appears in
any file this stage created or edited. No CG value is used by this stage at all.

## 2. (B) Was anything forbidden changed? — NO

`git diff --stat HEAD` over the whole tree: exactly two tracked files, both reviewed in §3.
`model/model.sdf`, `config/ardupilot/falcon_v2_sitl.parm`, `plugins/**`, every aerodynamic and
propulsion table, and every `docs/source_of_truth/{geometry,aerodynamics,propulsion}` file are
byte-unmodified vs `HEAD` and have 2026-09-02-or-older mtimes.

`falcon_v2_sitl.parm` sets **no** `TECS_*` value. Every live `TECS_*` value was checked by
`validation` against the compiled defaults in `/home/emirhan/gazebo_sim/ardupilot`
(commit `409226a637`):

`SINK_MIN 2.0` (AP_TECS.cpp:35), `TIME_CONST 5.0` (:43), `THR_DAMP 0.5` (:51),
`INTEG_GAIN 0.3` (:59), `SPDWEIGHT 1.0` (:99), `PTCH_DAMP 0.3` (:107), `SINK_MAX 5.0` (:115),
`CLMB_MAX 5.0` (:27), `PITCH_MAX 15` (:147), `PITCH_MIN 0` (:155), `HDEM_TCONST 3.0` (:292).
**All eleven match the firmware defaults exactly.** `PTCH_TRIM_DEG 2.49`, `RLL_RATE_*`/`PTCH_RATE_*`
= the values committed at `22eb5e4`. Control-surface travel, actuator, propulsion, sensor and
mass/CG/inertia paths untouched. Confirmed: **TECS ran on firmware defaults; nothing forbidden moved.**

## 3. (C) The two pre-existing files edited by `controls-integration` — both are what they claim

### 3.1 `tests/gazebo/scripts/test_ardupilot_tecs_cruise_speed_hold.py` — COMMENT-ONLY: **CONFIRMED**
The diff is +10/-1 lines, all inside a comment block plus one trailing comment on an unchanged
statement (`out["nav_alt_error_m"] = minmaxmean(aerr)`). No expression, threshold, constant or
control-flow token changes. No recorded result is affected.

### 3.2 `docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml` — DOCUMENTATION CORRECTION: **CONFIRMED, AND THE CORRECTION IS FACTUALLY RIGHT**

The old line claimed `NAV_CONTROLLER_OUTPUT.alt_error` is `int16 -> 1 m resolution`. Verified
independently by `validation`:

```
pymavlink MAVLink_nav_controller_output_message
  fieldnames: ['nav_roll','nav_pitch','nav_bearing','target_bearing','wp_dist','alt_error','aspd_error','xtrack_error']
  fieldtypes: ['float','float','int16_t','int16_t','uint16_t','float','float','float']
```
Only `nav_bearing` and `target_bearing` are `int16_t`. `alt_error` is a **float**. The old line was
wrong; the correction is right.

The second half of the edit restates the formula's subtrahend as `adjusted_altitude_cm`. Verified:
- `ArduPlane/GCS_MAVLink_Plane.cpp:240` → `plane.calc_altitude_error_cm() * 0.01`
- `ArduPlane/altitude.cpp:389-399` → `return target_altitude.amsl_cm - adjusted_altitude_cm();`
- `ArduPlane/altitude.cpp:561-564` → `adjusted_altitude_cm() = current_loc.alt - mission_alt_offset()*100`

`mission_alt_offset()` = `g.alt_offset` plus a landing term that is only added in AUTO/LAND
(`altitude.cpp:583-593`) — unreachable in FBWB. With `ALT_OFFSET = 0` (gated) the two forms are
numerically identical, so this is a precision improvement, not a change of meaning.

**No numeric engineering value is altered by the yaml edit.** The diff is +16/-1: a comment block,
one restated prose line, and one *added* derived line. Nothing in the TECS parameter table, the
command mapping numbers or any threshold moved. The empirical claim in the new yaml
(re-deriving the 2026-09-02 target altitudes gives 89.18 / 99.42 / 89.17 m on a 0.01 m grid) is
consistent with what `validation` measured this run: `ap_target` hold-window means land on exact
centimetre values (89.170 / 99.690 / 88.590 m) with sd = 0. A 1 m-resolution field could not do that.

**Verdict on C: both edits are exactly what they claim. The source-of-truth edit is a genuine,
source-verified factual correction and is approved.**

## 4. (D) Physics correctness — CORRECT

### 4.1 Energy definitions match ArduPlane's own TECS, line by line
Verified against `libraries/AP_TECS/AP_TECS.cpp` at commit `409226a637`:
`_SPE_est = _height*GRAVITY_MSS` (:689), `_SKE_est = 0.5*_TAS_state^2` (:690),
`_SPEdot = _climb_rate*GRAVITY_MSS` (:694), `_SKEdot = _TAS_state*(_vel_dot-_vel_dot_lpf)` (:695),
`_SKE_weighting = constrain(_spdWeight,0,2)` (:1003), `SPE_weighting = 2-_SKE_weighting` (:1024),
both `MIN(...,1)` (:1027-1028), `SEB_est = _SPE_est*SPE_w - _SKE_est*_SKE_w` (:1032),
`SEBdot_est = _SPEdot*SPE_w - _SKEdot*_SKE_w` (:1050). The test's formulae are identical.
The high-pass difference in SKEdot is **declared** in the module docstring and the yaml, not hidden.

**The `w_SPE = w_SKE = 1.0` assumption is proven, not assumed.** Dataflash `TECS.f` flag word over
all 1457 records: `Underspeed` **0/1457**, `isGliding` **0/1457**, `AutoLanding` 0/1457. None of the
branches at AP_TECS.cpp:1005-1020 that would change the weighting was ever entered.

### 4.2 Every reported energy rate re-derived independently — all reproduce
`validation` recomputed the regressions from the raw 20 Hz trace:

| Phase | SPEdot | SKEdot | STEdot | reported STEdot |
|---|---|---|---|---|
| P1 cruise | +0.114 | −0.018 | +0.096 | −0.006 (hold window) |
| P2 climb | **+12.768** | −0.864 | **+11.903** | +11.903 |
| P3 settle | −0.331 | −0.006 | −0.337 | −0.054 (hold window) |
| P4 descent | **−12.903** | +1.431 | **−11.471** | −11.471 |
| P5 resettle | +0.400 | +0.071 | +0.471 | +0.103 (hold window) |

Reproduced exactly. Signs correct: climb = altitude up, throttle 0.491→0.540, pitch +2.68→+6.72 deg,
airspeed bounded (EAS 17.29…18.25); descent = altitude down, throttle 0.491→0.420,
pitch +2.68→−1.33 deg, airspeed overshoot above demand only +0.66 m/s (limit 2.0). No duplicated
force, no duplicated damping, no double-counted energy term.

Roundtrip closure −6.413 J/kg on 1035.35 J/kg: cross-checked — the −0.632 m altitude residual alone
accounts for −6.20 J/kg (×9.81), the remaining −0.21 J/kg is kinetic. The books close.

### 4.3 PTCH_TRIM_DEG applied exactly ONCE — proven, not asserted
`Attitude.cpp:244` adds `pitch_trim` once. Measured, P1 hold:
`nav_pitch_raw +0.2330 deg + PTCH_TRIM_DEG 2.49 = +2.7230 deg` demand, against a measured
**physical** pitch of **+2.6841 deg** — a 0.039 deg residual. A double count would put the demand at
+5.21 deg and the residual at 2.5 deg. **Not double counted.**
Longitudinal kinematics `pitch − (alpha + gamma)` residual: mean +0.001 deg. Consistent.

### 4.4 Cross-check against the dataflash TECS internals (the record MAVLink cannot supply)
Aligning the `TECS` message time base to the test clock (see V-3 for the alignment), TECS's own
state agrees with the test's kinematics:

| Window | `TECS.sp` (TAS) | test SKE-implied TAS | `TECS.h` | test gz z | `TECS.th` | measured throttle |
|---|---|---|---|---|---|---|
| P1 hold | 18.0018 | 17.913 | 89.149 | 89.154 | 0.4913 | 0.4909 |
| P3 hold | 18.0350 | 17.951 | 99.670 | 99.675 | 0.4906 | 0.4902 |
| P5 hold | 17.9936 | 17.888 | 88.518 | 88.522 | 0.4913 | 0.4908 |

Height agrees to <1 cm; throttle to 0.0005. The ~0.09 m/s TAS gap is exactly the EAS→TAS scaling
(`CTUN.E2T` 1.0039 at ~89 m), i.e. 17.913 × 1.0039 = 17.983 ≈ 18.00. Units and frames are sound.
`pmin`/`pmax` constant at −25 / +15 deg, never reached (physical pitch spanned −4.73 … +10.12 deg).

**Verdict on D: cruise, climb, descent and the energy bookkeeping are physically correct.**

## 5. (E) THE CENTRAL QUESTION — my independent judgement

### Answer: the 25.0 s threshold is the problem. It is not a defensible criterion. This is NOT a real aircraft control deficiency.

I reproduced the failing metric byte-for-byte from the raw trace before judging it
(altitude-only 8.486 s, airspeed-only 25.269 s, combined 25.269 s, band violation 0.518 m/s peak in
the last out-of-band run, nine out-of-band runs at the times the test reports).

**The threshold's derivation is a category error.** `TH_SETTLE_TIME_MAX_S = 5 x TECS_TIME_CONST`
treats the settling of a *lightly damped second-order mode* as if it were the response of a
*first-order lag*. `TECS_TIME_CONST` sets the first-order gains of the TECS demand-tracking loops
(`_SPEdot_dem = (SPE_dem - SPE_est)/timeConstant()`, AP_TECS.cpp:735; `K_STE2Thr = 1/(timeConst *
K_thr2STE)`, :760). It does not, and cannot, bound the decay envelope of the oscillatory mode the
airframe-plus-TECS actually exhibits. The settling time of that mode is governed by
`t = ln(A0/B) / (zeta * omega_n)`, which contains **no** `TECS_TIME_CONST` term.

**Applying the correct formula shows the threshold sits inside the predicted answer.**
From this run's own measurements (T = 5.633 s, per-cycle amplitude ratio 0.8046, initial airspeed
excursion A0 = 1.598 m/s, band B = 0.5 m/s):

- envelope time constant `tau = T / ln(1/0.8046) = 25.9 s` → `t = tau*ln(A0/B) = 30.1 s`
- envelope fitted directly over the phase (1.598 m/s at t≈5 s → 0.518 m/s at t≈25.3 s):
  `tau = 18.0 s` → `t = 20.9 s`
- the last-exit statistic can only land on an oscillation peak, so add up to `T/2 = 2.8 s`

**Physically predicted settling time: ~21 to ~33 s. The threshold is 25.0 s — inside that band.**
A gate placed in the middle of the predicted range of the quantity it gates cannot discriminate a
healthy aircraft from an unhealthy one.

**Three independent confirmations that it does not discriminate:**

1. **Metric resolution.** The measured out-of-band run end-times are
   3.43, 6.76, 9.27, 12.12, 14.83, 17.53, 20.34, 22.94, 25.27 s — spacing 2.33 … 2.86 s,
   mean 2.73 s = one half-period. The settling time is **quantised on a ~2.7 s grid**. The
   reported exceedance is **0.269 s = 10 % of the metric's own resolution**. Any threshold anywhere
   in (22.94, 25.27) produces the identical FAIL. The criterion has no meaningful resolution here.
2. **Reproducibility.** `validation` applied the identical metric to the 2026-09-02 raw timeseries
   (`B_hold_new_altitude`): **20.138 s** (this confirms the yaml's 20.1 s claim, which I verified
   rather than accepted). Same aircraft, same firmware defaults, same command method, same metric:
   **20.1 s then 25.3 s — a 5.13 s / 26 % run-to-run spread against a hard 25.0 s limit.**
   A criterion whose run-to-run spread is 19x its own margin is not a gate.
3. **The initial condition is uncontrolled by the test.** `t_settle` scales with `ln(A0/B)`, and
   `A0` is set by the ramp exit, not by the aircraft. The FBWB stick ramps ArduPlane's raw demand
   `hin` at 2.0 m/s while the aircraft achieves ~1.16-1.30 m/s, so at stick release
   `hin - h` had grown to **+6.45 m** (climb) / **−6.97 m** (descent). The transient being settled
   is therefore a by-product of the accumulated command lag — A0 was 1.431 m/s in the 2026-09-02
   run and 1.598 m/s here (+12 %), which alone shifts `t_settle` by `tau*ln(1.598/1.431) ≈ +2 s`.
   The criterion is measuring the size of an uncontrolled transient at least as much as the aircraft.

**What I explicitly ruled OUT as the cause:**
- *Sensor noise:* I checked, and it is **not** the explanation. The EAS high-frequency residual is
  only 0.030 m/s sd (vs 0.004 for ground-truth TAS), and repeating the whole settling computation on
  the clean Gazebo TAS gives **25.371 s** — marginally worse, not better. The 0.518 m/s excursion is
  a real physical amplitude, not a measurement artifact. (`SIM_ARSPD_RND` is 2 **Pa**, i.e. ≈0.11 m/s
  peak at 18 m/s, and does not dominate.)
- *A defect in the metric code:* `settling_analysis()` is correctly implemented (last-exit semantics,
  tail-mean reference, `never`-settled handling). I found no bug.
- *High-J windmilling:* P3 has `interpClamped` **0 / 1538**. It cannot be involved.
- *Saturation / clamping / divergence / NaN:* all zero (see §7-§8).

**Is anything real underneath? Yes — but it is a separate, non-blocking observation (V-2).**
The closed-loop longitudinal mode measured here is `T = 5.633 s, zeta ~= 0.035`. The free-airframe
Lanchester estimate computed from **this run's own** measured cruise coefficients
(CL 0.661, CD 0.0579, L/D 11.42, V 17.93 m/s) is `T = 8.12 s, zeta = 0.707*CD/CL = 0.062`.
So with TECS engaged on firmware defaults the mode is ~31 % faster and roughly half as damped as the
aircraft's own phugoid. That is a genuine, quantifiable control-loop characteristic and it is what
makes the airspeed take 21-33 s to come inside 0.5 m/s. It is **not** wrong physics, **not** a sign
or frame error, and **not** an instability: every channel decays (ratios 0.473-0.569 vs a 0.90 limit),
the envelope falls monotonically 1.598 → 0.302 m/s, and `settled_within_segment = true`.
It should be recorded and root-caused, not tuned away (V-2).

### What a defensible criterion looks like, and its derivation

Stated as a **test-criterion correction**, not as tuning the aircraft to pass. My reasoning above
stands on the physics of a second-order mode and on run-to-run reproducibility, and it would be
identical if the measured value had been 24.7 s and the test had "passed" — in that case I would be
reporting that the criterion had passed for the wrong reason. **Relaxing a number to convert a FAIL
into a PASS is forbidden by CLAUDE.md and is not what I am recommending.**

1. **Primary gate should be amplitude-independent decay, which the test already computes and which
   already passes.** `oscillation_decays_after_climb` / `..._after_descent` (all 8 channel ratios
   0.473-0.569 vs 0.90) test the property that actually matters — the mode decays. That gate does not
   depend on the uncontrolled A0 and does not sit on a half-period grid.
2. **If a settling-time gate is retained, derive it from the mode, not from a controller time
   constant:** `t_settle <= k * [ T*ln(A0/B)/ln(1/r) + T/2 ]` with `T`, `r` (per-cycle amplitude
   ratio) and `A0` measured in the same window and `k` a stated, justified margin. This tests the
   aircraft rather than the size of the transient it was handed.
3. **Control A0.** Either release the FBWB stick when ArduPlane's own demand `hin` reaches the
   target (not when the aircraft does), or ramp at a rate the aircraft can follow, so `hin - h` at
   release is bounded. Today it reaches 6.5-7 m and varies run to run.
4. **Derive the 0.5 m/s band from an instantaneous-excursion basis.** It is currently inherited from
   `TH_SPEED_MEAN_TOL_MS`, which is a tolerance on a *window mean* — a different statistic. The band
   and the time limit are coupled through `ln(A0/B)`; neither was derived for this metric.
5. **Until (1)-(4) exist, report the settling time as a measured characteristic rather than gating
   on it.** There is presently no Falcon V2 handling-qualities basis in `docs/source_of_truth/` from
   which an acceptable longitudinal settling time or damping ratio could be derived — that is
   `DATA_REQUIRED`, and it is the actual reason no defensible number can be written today.

The re-derived criterion must be justified **before** the re-run and must still be able to fail.
If a properly derived criterion still fails, that is a real result and must be reported as one.

**Responsible specialist for the criterion: `controls-integration`** (owns the TECS/FBWB integration,
the settle-phase design, the command method and the acceptance thresholds). Then `gazebo-testing`
re-runs, then `validation` re-reviews. **No aerodynamic, mass, CG, inertia, propulsion,
control-surface or `TECS_*` value may be changed to move this number.**

## 6. (F) Saturation / oscillation / decay analysis — CORRECT; excluding the ramp windows is legitimate

`gazebo-testing` flagged `P2_climb_full` airspeed growth ratio 1.234 and `P4_descent_full` throttle
growth ratio 1.416 (the latter with `growing: true`), while `no_growing_oscillation` evaluates only
the three hold windows. I checked whether this masks a growing oscillation. **It does not.**

- `detrended_growth()` fits a **straight line** and compares second-half to first-half residual sd.
  On a ramp window that is a *curvature* statistic, not an oscillation statistic.
- I inspected the raw P4 throttle trace directly: it is **monotone and smooth**
  (0.489, 0.482, 0.470, 0.457, 0.448, 0.441, 0.438, 0.437, 0.440, 0.441, 0.444, 0.445, 0.438, 0.431,
  0.424, 0.413, 0.399, 0.385, 0.370, 0.358, 0.350, 0.342, 0.334) — a convex decay with **no
  oscillation at all**. The 1.416 ratio is purely the residual of a straight-line fit to a curve.
- Same for P2 airspeed: a commanded excitation, not a free response.
- The stability question — does the response keep growing after the excitation stops — is answered by
  P3 and P5, both of which decay on all four channels (0.473-0.569) with monotonically falling
  envelopes. Nothing is masked.

**Conclusion: excluding the ramp windows is methodologically correct.** One transparency defect
remains (V-5): the check is named `no_growing_oscillation` without qualification and silently
discards two windows where the underlying function returned `growing: true`. The raw ratios are in
the JSON, so nothing is hidden, but the check name overstates its scope.

## 7. (G) PROPULSION_HIGH_J_WINDMILLING — honestly preserved: CONFIRMED

- Declared `OPEN_LIMITATION`, owner `propulsion`, `DATA_REQUIRED`, in the test docstring, in
  `high_j_block()`, in the new yaml (§5) and in the report (§9). Not gated anywhere in `verdict()`.
- **P4 descent: 126 / 358 motor-samples clamped = 35.20 %, 124 at exactly 0.000 N.** Reproduced.
  Clamping begins at P4 t_seg 6.25 s as J crosses ~0.65 — consistent with the stated ~0.64
  zero-thrust advance ratio of the APC 13x6.5E table.
- **Never compensated:** no correction factor, no gating, no threshold relaxation anywhere.
- **The descent's absolute drag/sink performance is explicitly NOT claimed as truth** — the
  propulsive-power cross-check is `_preferred` (non-gating) precisely because of this, and both the
  yaml and the report state that only direction, controllability, settling and (kinematic) energy
  bookkeeping are valid.
- **It does NOT touch P3, where the failure occurred: `interpClamped` = 0 / 1538 in P3.** Confirmed.
- Honesty note `validation` adds: the descent clamp episode extends into the **first ~1.56 s of the
  P5 settling window** (62 of 1534 P5 motor-samples). P5 passed, and less prop drag would if anything
  *lengthen* settling, so the direction of the bias is not favourable to a pass. Worth stating
  explicitly; it does not change the conclusion. (INFO)

## 8. (H) NaN/Inf and airspeed floor — verified from the raw data by `validation`

Scanning all 2746 rows x 29 numeric columns of the per-sample trace:
- **non-finite floats: 0. Missing/None entries: 0.**
- **EAS min 16.6509 m/s** vs `AIRSPEED_MIN` 16.0 → margin +0.651 m/s. Never below.
- vs the 14.4 m/s (0.9 x TASmin) underspeed trigger → margin +2.251 m/s. Never below.
- **TAS (ground truth) min 16.6719 m/s**, max 19.5268 m/s. EAS max 19.4974 vs `AIRSPEED_MAX` 28.
- Independently confirmed from the dataflash: TECS `Underspeed` flag **0 / 1457 records**.
- Throttle 0.180 … 0.578 inside `THR_MIN`/`THR_MAX`; per-sample `throttle_saturated` **0 / 2746**;
  per-sample `surface_clamped` **0 / 2746**; `PM.NLon = 0`, `ErrL = 0`, `InE = 0`; `ERR` table empty.

---

## 9. Findings

### CRITICAL — none.

### MAJOR

**V-1 (STAGE-BLOCKING) — `settles_after_climb` gates on a criterion that is not physically well-founded.**
`TH_SETTLE_TIME_MAX_S = 5 x TECS_TIME_CONST = 25.0 s` bounds a first-order loop time constant, not
the decay envelope of the measured second-order mode. The physically predicted settling time from
this run's own measurements is 21-33 s; the threshold lies inside that interval. The metric is
quantised on a 2.73 s half-period grid (exceedance 0.269 s = 10 % of resolution); the identical
metric gave 20.138 s on 2026-09-02 vs 25.269 s here (26 % spread); and its input A0 is set by an
uncontrolled 6.5-7 m command lag at stick release. The companion 0.5 m/s band is inherited from a
*window-mean* tolerance and was not derived for instantaneous excursion. **The FAIL is not evidence
of an aircraft control deficiency.** Owner: **`controls-integration`**. Re-derive per §5, then
`gazebo-testing` re-runs, then `validation` re-reviews. No aircraft parameter to be touched.

**V-2 (MAJOR, open item — record and root-cause; not by itself stage-blocking) — the closed-loop
longitudinal mode is lightly damped and uncharacterised for Falcon V2.**
Measured `T = 5.633 s, zeta ~= 0.035` (P3) / `T = 5.614 s, zeta ~= 0.028` (P5), against a
free-airframe Lanchester estimate of `T = 8.12 s, zeta = 0.062` from this run's own CL/CD. Stable
and decaying in every channel, but poorly damped and never characterised from data. Directly related
and already flagged in the new yaml as `ASSUMPTION TECS_CLMB_MAX_NOT_AIRFRAME_DERIVED`:
`TECS_CLMB_MAX 5.0` / `TECS_SINK_MIN 2.0` set `K_thr2STE = (STEdot_max - STEdot_min)/(THRmax-THRmin)
= 68.6` and hence `K_STE2Thr = 1/(5 x 68.6) = 0.0029` — a very low throttle-to-energy-error gain.
`validation` notes independently that 5 m/s of climb at 18 m/s would need roughly 21 N of total
thrust against the ~5.2 N used in cruise, so `TECS_CLMB_MAX = 5.0` looks optimistic for this
airframe; the aircraft could not even follow the 2.0 m/s FBWB demand (achieved 1.16-1.30 m/s) while
throttle peaked at only 0.578. **DATA_REQUIRED:** there is no Falcon V2 climb/sink performance or
handling-qualities basis in `docs/source_of_truth/` from which `TECS_CLMB_MAX`, `TECS_SINK_MIN` or an
acceptable damping ratio could be derived. Owner: **`controls-integration`** (primary, owns TECS
params), with **`propulsion`** input on the achievable climb rate. Not `aerodynamics`: the measured
mode is a *closed-loop* mode, ~31 % faster than the free phugoid, so it is a loop property, and the
aero model's own cruise CL/CD imply a *better*-damped free phugoid than what is observed.
**No broad tuning. Root-cause first.**

**V-3 (STAGE-BLOCKING for acceptance of the report, not for the aircraft) — `gazebo-testing`'s
dataflash cross-check table (report §4.2) is time-misaligned, and its conclusion is inverted.**
The report aligns the `TECS` log to flight time with an offset of **34.939 s**. The correct offset is
**33.35 s**: at 33.35 s the residual between `TECS.h` and `GLOBAL_POSITION_INT.relative_alt` is
**0.017 m RMS**, at 34.939 s it is **1.214 m RMS** (71x worse). Independently corroborated by the
dataflash `MODE` record — FBWB is entered at AP t = 33.08 s and `enter_fbwb()` sleeps ~0.3 s before
the P1 segment starts, which puts test t = 0 at AP t ≈ 33.35 s, not 34.94 s.
Consequences in §4.2 (recomputed by `validation` with the correct offset):

| | report (off 34.939) | correct (off 33.35) |
|---|---|---|
| P2 climb `dh` (TECS's own climb rate) | +1.543 m/s | **+1.159 m/s** |
| P2 climb `dhdem` | +1.311 | +1.305 |
| P4 descent `dh` | −1.504 | **−1.151** |
| P4 descent `th` | 0.4005 | **0.4199** |
| P2 `hdem` max | 101.05 m | **100.15 m** |

The report concludes "the demand shaping, not the airframe, limits the achieved ramp rate" —
which the misaligned data supports (`dh` 1.543 appears to *exceed* `dhdem` 1.311). With the correct
alignment the aircraft **lags** its own shaped demand (`dh` 1.159 vs `dhdem` 1.305, `hdem - h`
growing to +0.637 m), so the statement as written is not supported. Demand shaping is still the
dominant limiter (raw `hin` ramps at 2.0 m/s), but the airframe/loop contribution is real and was
sign-flipped away. **No acceptance check depends on this** (the test never parses the BIN), so the
verdict is unaffected — but this was one of the four items explicitly handed to `validation` for
cross-check, and it must be corrected. Owner: **`gazebo-testing`** (report §4.2 correction only; no
re-fly needed — the BIN already on disk is sufficient).

### MINOR

**V-4 — `TH_RESETTLE_TIGHT_M = 0.5 m` is unachievable by construction with this command method, and
its supporting latency derivation is incomplete.** The FBWB target latches via
`set_target_altitude_current()` when the stick crosses zero. Measured: at the P4 stop the aircraft
was at z = 88.883 m sinking at **−1.924 m/s** (instantaneous), and the demand latched at exactly
88.590 m — i.e. 0.564 m below `z_ref`, which is the whole −0.632 m roundtrip residual. ArduPlane then
tracked its (slightly low) latched demand to **0.07 m**. The yaml's derivation bounds this latency at
`peak_vz x (0.1 + 0.1) = 0.39 m` but omits `RAMP_STOP_CONSECUTIVE = 3` (0.15 s) and the 20 Hz sample
period; the true bound is ~0.4 s x sink ≈ **0.6-0.8 m**, which exceeds the 0.5 m "preferred" limit.
The preferred check therefore cannot pass regardless of aircraft quality. Not gating. Owner:
`controls-integration` — either add stop-criterion lead compensation or re-derive/retire the
0.5 m preferred bound.

**V-5 — `no_growing_oscillation` check scope is not reflected in its name.** It evaluates only the
three hold windows while discarding two ramp windows where `detrended_growth()` returned
`growing: true`. The exclusion is *correct* (V-6/§6) but the name overstates it. Recommend renaming
to `no_growing_oscillation_in_hold_windows` and reporting the ramp ratios as declared INFO.
Owner: `controls-integration`.

**V-6 — minor source line-citation drift.** The test and yaml cite `AP_TECS.cpp:1031-1032` for SEB
and `:1036-1096` for SEBdot. In this checkout `SEB_est` is at **:1032** (:1031 is the comment) and
`SEBdot_est` is at **:1050**. Also `_STE_error` is at :738, cited as :739-772. All formulae are
correct; only the line anchors drift by 1-15 lines. Owner: `controls-integration`.

**V-7 — report §7 wording.** "the nine out-of-band airspeed runs in P3 decay monotonically" — the
sequence is 0.920, 1.598, 0.883, 0.972, 0.851, 0.717, 0.614, 0.578, 0.518: run 1→2 rises (it is the
partial leading excursion) and run 3→4 rises slightly. Runs 2 and 4→9 are monotone. Wording only;
the table itself is exactly reproducible. Owner: `gazebo-testing`.

**V-8 — report §5 sink-rate figure.** States the aircraft was "still sinking at 1.15 m/s" at the
descent stop; 1.15 m/s is the P4 *window mean*, the instantaneous sink was **1.92 m/s**. The
attribution to ramp-stop latency is correct either way (it changes the implied latency from 0.49 s to
0.15 s). Owner: `gazebo-testing`.

### INFO

- **I-1** Advance ratio J reached **1.98** on 5 P1 spin-up motor-samples (t 0.00-0.21 s, throttle
  0.18-0.44) — far outside the APC 13x6.5E table. These lie inside the 12 s P1 transient that is
  excluded from every analysed window. Recorded so it is not discovered later.
- **I-2** The High-J clamp episode extends into the first ~1.56 s of the P5 settling window
  (62 / 1534 motor-samples). P5 passed; the bias direction (less drag) is not favourable to a pass.
- **I-3** The settling band is taken about the *tail mean* rather than about the TECS demand
  (17.899 vs 17.918 m/s — 0.019 m/s). Immaterial here; worth stating since a regulated variable's
  settling is conventionally measured against its reference.
- **I-4** Gravity: energies use the world 9.81 while ArduPlane uses 9.80665 (0.034 %). Declared and
  recorded in the result JSON, not corrected away. Correct handling.
- **I-5** Command quantisation: RC3 1258 us → TECS demand 17.920 m/s rather than 18.000 (0.12 m/s
  grid). Derived and reported, not hidden.
- **I-6** `..._gz_log.txt` is 0 bytes = zero Gazebo errors/warnings; all five plugins produced live
  diagnostics throughout, so nothing failed to load.
- **I-7** Both 2026-09-02 open MAJORs (584 m atmosphere datum; `wind_ef` zeroed / pitot not in loop)
  are **independently confirmed CLOSED** by this run (§1). The third (High-J prop table) remains open.

---

## 10. Stage disposition

**Unresolved STAGE-BLOCKING findings: YES — two MAJOR, neither of which implicates the aircraft.**

| # | Finding | Class | Blocking | Owner |
|---|---|---|---|---|
| V-1 | `settles_after_climb` criterion not physically well-founded | MAJOR | **YES** | `controls-integration` |
| V-3 | Report §4.2 dataflash alignment error, conclusion inverted | MAJOR | **YES** | `gazebo-testing` |
| V-2 | Lightly damped, uncharacterised closed-loop mode (zeta ~0.035); `TECS_CLMB_MAX`/`SINK_MIN` not airframe-derived | MAJOR | no (record + root-cause) | `controls-integration`, with `propulsion` |
| V-4 | 0.5 m preferred re-settle bound unachievable by construction | MINOR | no | `controls-integration` |
| V-5 | `no_growing_oscillation` scope vs name | MINOR | no | `controls-integration` |
| V-6 | AP_TECS line-citation drift | MINOR | no | `controls-integration` |
| V-7, V-8 | Report wording / sink-rate figure | MINOR | no | `gazebo-testing` |

**Nothing about the simulated aircraft is blocked.** The physics, units, frames, signs, force
directions, energy bookkeeping, PTCH_TRIM_DEG convention, control directions and numerics are all
correct and independently reproduced. No forbidden parameter was changed, no duplicated force or
damping term exists, and the CG duality trap is not present.

**Required loop:** `controls-integration` re-derives the settling criterion (V-1) per §5 and fixes
V-4/V-5/V-6; `gazebo-testing` corrects report §4.2 (V-3) and V-7/V-8; `gazebo-testing` then re-runs
the campaign unmodified; `validation` re-reviews. V-2 is recorded as an open engineering item with an
owner and does not need to be resolved to close this stage, but must not be silently dropped.

**Explicitly forbidden as a response to this stage:** changing any `TECS_*` value, PID,
`PTCH_TRIM_DEG`, control-surface travel, aerodynamic coefficient, actuator or propulsion parameter,
or mass/CG/inertia, in order to move the settling number. My V-1 finding rests on the physics of a
second-order mode and on run-to-run reproducibility, not on the outcome it produces; a properly
re-derived criterion must remain able to fail, and if it still fails, that is a real result.

Reviewed by `validation`, 2026-09-03. Read-only: no engineering parameter was edited.

---
---

# RE-REVIEW — 2026-09-04 (narrow: closure of V-1 and V-3 only)

Independent re-review by `validation`. Read-only: no engineering parameter, threshold, SDF, plugin,
table or config value was edited by this review. **Scope is strictly V-1, V-3 and the new
`SETTLE_LIMIT_NOT_FULLY_FALSIFIABLE_P3_AIRSPEED` limitation.** The flight campaign, the physics, the
energy bookkeeping and all earlier READY stages were NOT re-audited — they were cleared on
2026-09-03 and nothing in this re-review disturbs them. Every number below was re-derived by
`validation` from the raw artifacts (`..._timeseries.json`, `..._per_sample.json`, the untouched
dataflash BIN) and from the ArduPlane source — not taken from either agent's summary.

## R.0 Bottom line

| | |
|---|---|
| V-1 (settling criterion re-derivation, `controls-integration`) | **CLOSED** |
| V-3 (dataflash alignment + inverted conclusion, `gazebo-testing`) | **CLOSED** |
| Is the V-1 replacement a legitimate test-criterion correction, or threshold-shopping? | **LEGITIMATE CORRECTION.** Evidence in R.1.4 |
| `SETTLE_LIMIT_NOT_FULLY_FALSIFIABLE_P3_AIRSPEED` — carry or gate? | **CARRY.** It does not invalidate the P3 airspeed pass. It is also **over-stated** (R.3) |
| Forbidden parameter moved during either fix | **NONE** |
| Re-fly performed | **NO** |
| Raw flight data altered by the `--reanalyze` rewrite | **NO** (byte-exact fixed point, R.4) |
| New findings | 1 MAJOR (non-blocking), 4 MINOR, 3 INFO |
| **Unresolved STAGE-BLOCKING CRITICAL/MAJOR** | **ZERO** |

---

## R.1 V-1 — CLOSED

### R.1.1 The Lanchester derivation and the `TAU_REF = V*(L/D)/g` algebra are CORRECT

Re-derived from first principles, independent of the implementation:

```
omega_n  = sqrt(2) * g / V                          (classical Lanchester phugoid)
zeta     = CD / (sqrt(2)*CL) = 1 / (sqrt(2)*(L/D))  (classical phugoid damping)
zeta*omega_n = [1/(sqrt2*L/D)] * [sqrt2*g/V] = g / (V*(L/D))
TAU_REF  = 1/(zeta*omega_n) = V*(L/D)/g             CORRECT
T_REF    = 2*pi/omega_n     = pi*sqrt(2)*V/g        CORRECT
```

Units check: [m/s]·[-]/[m/s^2] = s. Correct. `G_WORLD = 9.81` is used consistently with the rest of
the energy analysis (the 9.81 vs 9.80665 distinction is already declared, I-4).

Numeric reproduction, P3: V 17.94776, CL 0.661142, CD 0.0578894 -> L/D 11.42078,
`zeta_free` 0.061914, `TAU_REF` **20.8947 s**, `T_REF` 8.1284 s. All reproduce exactly, and they
match the free-airframe figures I derived independently on 2026-09-03 (L/D 11.42, zeta 0.062,
T 8.12 s) — computed then without sight of this implementation.

Independent cross-check of the reference itself: the level-flight identity `L/D = m*g/T_total`
gives 11.458 against the aero-diagnostic 11.421 — **0.3 % agreement**, on a channel the criterion
does not use. `interpClamped` is 0/1538 in P3, so the thrust cross-check is not contaminated by
`PROPULSION_HIGH_J_WINDMILLING` in this window. The reference is sound.

`ASSUMPTION PHUGOID_REFERENCE_IS_LANCHESTER` is declared in the code docstring, in the result JSON
and in the yaml, with the replacing measurement marked `DATA_REQUIRED`. Correct handling.

### R.1.2 Arithmetic of the gate — reproduces exactly

P3 airspeed: `ln(1.5980968/0.5) = 1.1619606`;
`t_limit = 5.406965 + 5.632851/2 + 1.0*20.894744*1.1619606 = 32.50226 s` (JSON: 32.50226);
`tau_implied = (25.268610 - 5.406965 - 2.816426)/1.1619606 = 14.66936 s` (JSON: 14.66936).
`tau_implied/tau_limit = 0.7020`. Verified.

### R.1.3 Nothing else was loosened — verified, not assumed

| Quantity | Value now | Evidence it did not move |
|---|---|---|
| `TH_SETTLE_BAND_M` | 1.5 m | recorded in `thresholds.DERIVED_THIS_STAGE`; same value cited in my 2026-09-03 review |
| `TH_SETTLE_SPEED_BAND_MS` | 0.5 m/s | `= cruise.TH_SPEED_MEAN_TOL_MS`; the cruise module is tracked and its only diff vs `HEAD` is the +10/-1 comment block (re-verified) |
| `SETTLE_TAIL_S` | 10.0 s | decay windows n = 576 / 575 of 769 / 767 samples => 9.65 s of transient excluded at 20 Hz. Consistent |
| `HOLD_TRANSIENT_S` | 10.0 s | inherited from the cruise module (unchanged) |
| P3 / P5 segment length | 40.0 s | **cannot have changed**: the stored samples span 39.980 s and 39.975 s and were recorded at 23:23 on 2026-09-03, before the fix |
| all 24 INHERITED thresholds | unchanged | imported by symbol from the tracked cruise module |
| every other DERIVED threshold | unchanged | `thresholds` block re-read and compared to my 2026-09-03 citations |
| gating structure | unchanged | 81 checks / 67 core; `settles_after_climb` and `settles_after_descent` are **still CORE and still gating**; the only rename is V-5's `no_growing_oscillation` -> `no_growing_oscillation_in_hold_windows` |

The withdrawn `TH_SETTLE_TIME_MAX_S` is not deleted — it is recorded in the result JSON as
`{value: null, status: WITHDRAWN_2026-09-03, was: 25.0, cause: ..., replaced_by: ...}`. Correct
handling: the old criterion remains auditable.

### R.1.4 Is it outcome-independent, or reverse-engineered to pass? — LEGITIMATE

Six independent lines of evidence, in the order I weighted them:

**(a) The criterion has no free parameter left to shop.** Bands inherited; `K = 1.0` is the unique
zero-margin value implied by the stated engineering meaning; `TAU_REF` comes from the phase's own
measured CL/CD/V; `T`, `A0` and `t_peak` are measured. There is nothing left to tune. Any `K > 1`
would need a Falcon V2 handling-qualities basis, which is genuinely `DATA_REQUIRED`. K = 1.0 is the
strictest defensible choice: a stricter `K < 1` would assert "TECS must beat the free airframe by X",
and no basis exists for choosing X — that would be an invented number in the other direction.

**(b) The formula shape was prescribed by `validation` BEFORE any outcome was known.** My
2026-09-03 §5 item 2 recommended `t_settle <= k*[T*ln(A0/B)/ln(1/r) + T/2]`. `controls-integration`
implemented that shape but replaced my measured per-cycle ratio `r` with the free-airframe
`TAU_REF`. That substitution makes it **stricter and non-self-referential**: with the measured `r`
the limit would be 32.93 s (a near-tautology, since the mode is gated against its own decay);
with `TAU_REF` it is 32.50 s and the aircraft can genuinely fail. They chose the harder reference.

**(c) The PASS survives most alternative defensible instantiations.** P3 airspeed, ratio to the
`K = 1.0` gate:

| Instantiation | statistic | ratio | outcome |
|---|---|---|---|
| as implemented (`t_peak` + `T/2` allowances) | 14.669 s | 0.702 | PASS |
| drop the `T/2` quantisation allowance | 17.094 s | 0.818 | PASS |
| drop the `t_peak` offset, keep `T/2` | 19.323 s | 0.925 | PASS |
| drop both (clock from segment start) | 21.747 s | 1.041 | FAIL |
| `validation`'s own §5 shape with `TAU_REF` | limit 27.095 s vs 25.269 s | — | PASS |
| raw peak-to-last-exit envelope (no allowances, measured amplitudes) | 17.630 s | 0.844 | PASS |

Only the naive form that starts the decay clock at the segment start and grants no quantisation
allowance fails, and only by 4 %. Both allowances are physically justified (the envelope clock
starts at the peak; the last-exit statistic can only land on a peak) and both were recommended or
implied by my own finding. The PASS is therefore not an artefact of the two allowance terms.

**(d) The cross-check on an independent run reproduces exactly.** I applied the corrected criterion
myself to the 2026-09-02 `B_hold_new_altitude` samples: settled v 17.9117, `t_settle` **20.1376 s**
(reproducing my 2026-09-03 figure of 20.138 s), A0 1.4308 m/s @ t 5.286 s, T 5.699 s, V 17.9512,
CL 0.66153, CD 0.05791, L/D 11.4237, `TAU_REF` 20.9042 s, limit 30.1143 s,
`tau_implied` **11.4151 s**, ratio **0.5461**. `controls-integration`'s 0.546 is confirmed to four
significant figures. The old fixed limit gave PASS/FAIL for the same manoeuvre; the new one gives
PASS/PASS at 0.546/0.702 — consistent classification with a common normalisation, and neither
marginal.

**(e) The raw measurement is preserved unchanged everywhere.** `settling_time_airspeed_s
25.26861023902893` and `settling_time_altitude_s 8.485944271087646` in the new result JSON are
**bit-identical** to the values printed in `..._log.txt`, which was written at flight time
(23:23:33) and never rewritten. So are all four decay ratios, the achieved steps, the hold-window
means and the throttle/pitch figures. The measurement path did not change; only the acceptance test
did. The withdrawn revision-1 `TEST_FAILED` block is retained verbatim in the `gazebo-testing`
report (§2.2), which is the correct way to withdraw a result.

**(f) The criterion is applied symmetrically.** One code path (`settling_analysis()`), one loop over
`(("altitude", ...), ("airspeed", ...))`, one loop over `(("P3_settle", "after_climb"),
("P5_resettle", "after_descent"))`. No phase-specific or channel-specific branch exists. P5's
negative `tau_implied` is handled by a documented explanatory note, not by a special case — `d["ok"]`
is still `t_set <= limit_s`, evaluated identically.

**Verdict on V-1: CLOSED.** This is a legitimate test-criterion correction, not threshold-shopping.
I would reach the same conclusion had the corrected criterion produced a FAIL.

### R.1.5 What the closure does NOT establish — carried forward to V-2

`tau_implied = 14.669 s` is a **peak-to-band settling** statistic. It is not the mode's asymptotic
envelope. From the report's own log-decrement figures (`T = 5.6329 s`, per-cycle ratio 0.80460,
`zeta = 0.0346`) the closed-loop asymptotic envelope is `tau = T/ln(1/r) = 25.91 s` — cross-checked
as `T/(2*pi*zeta) = 25.92 s` — against `TAU_REF = 20.895 s`. **By that measure the closed-loop mode
decays 1.24x SLOWER than the free-airframe phugoid envelope, not faster.** The gate passes because
the early decay from the peak (17.63 s) is faster than the asymptotic tail decay, which is the
normal behaviour of a transient with a non-modal initial condition.

This is not a defect in the criterion — a check named `settles_*` is entitled to gate a settling
time — but it means the PASS must **not** be read as evidence that the closed-loop damping is
adequate or that it beats the free airframe. That is exactly V-2, which remains open. See R.5 / V-9.

---

## R.2 V-3 — CLOSED

Everything below was re-derived by `validation` from the untouched
`..._dataflash/00000001.BIN` (mtime 2026-09-03 23:23:33, md5 `5b7bcc618b1c9da8f132048b7623d411`)
and the stored samples.

### R.2.1 The offset and its uncertainty — CONFIRMED

Independent RMS minimisation, 0.5 ms grid, 32.0-36.0 s, `TECS.h` (1457 records, 10 Hz) against
`GLOBAL_POSITION_INT.relative_alt` (2746 samples, 20 Hz), whole flight:

| | `validation` (this review) | `gazebo-testing` |
|---|---|---|
| best offset | **33.3395 s** | 33.3425 s |
| RMS at best | **0.0148 m** | 0.0150 m |
| RMS at 34.939 s | **1.2144 m** (82x worse) | 1.2165 m (81x) |

My best differs from theirs by **3 ms**, an order of magnitude inside their stated ±0.031 s
(interpolation-direction difference). The uncertainty definition (interval where RMS stays below
2x its minimum) is a reasonable, stated convention.

Multi-channel agreement: `TECS.h` vs `relative_alt` 33.3395; `TECS.h` vs Gazebo z 33.3625;
`TECS.hin` vs the MAVLink target readback 33.3330 — spread **0.0295 s**, consistent with the claimed
"within 0.028 s". `TECS.th` vs measured throttle gives 33.1575 with a very flat minimum;
`gazebo-testing` **excluded it explicitly as a weak estimator** rather than averaging it in. That is
the correct call and it is disclosed, not hidden.

### R.2.2 The three corroborations — all CONFIRMED

1. **`MODE`.** Exactly two records: MANUAL (0) at AP 9.083 s, FBWB (ModeNum 6, Rsn 2 = GCS) at AP
   **33.083 s**. Offset 33.34 puts test t = 0 at 0.26 s after the mode switch — matching the
   `time.sleep(0.3)` + rate-request latency. 34.939 would demand 1.86 s.
2. **The `reset` bit.** First `TECS` record at AP **33.1428 s** with `f = 128`, the only non-zero
   `f` in 1457 records. I verified the bit position directly in the firmware:
   `libraries/AP_TECS/AP_TECS.h` `struct flags` orders `underspeed, badDescent, is_doing_auto_land,
   reached_speed_takeoff, gliding_requested, is_gliding, propulsion_failed, reset` — **bit 7 = 128 =
   `reset`**, "a reset of airspeed and height states to current is performed on this frame".
   Confirmed.
3. **Causality — decisive.** `TECS.hin` is flat at 89.170 m through AP 78.443 s and first departs at
   AP **78.543 s** (to 89.400). The climb stick was applied at test **45.004 s** (P2 segment start,
   from the stored samples). Hence `offset <= 78.543 - 45.004 = ` **33.539 s**. Reproduced exactly.
   34.939 s would place the command at test 43.604 s — **1.400 s before the stick moved**.
   34.939 s is excluded on causality alone.

### R.2.3 The root cause — CONFIRMED (directionally exact)

I measured the two competing alignments myself:

| pairing | best offset | RMS |
|---|---|---|
| `TECS.hin` vs `relative_alt + nav_alt_error` (unshaped vs unshaped) | **33.333 s** | 0.082 m |
| `TECS.hdem` vs the same readback (shaped vs unshaped) | **35.821 s** | 0.647 m |

So the MAVLink target readback **is** the unshaped `hin`, and matching the `TECS_HDEM_TCONST = 3.0 s`
shaped `hdem` against it absorbs the filter lag into the offset and pushes it ~2.5 s late.
Revision 1's 34.939 s sits inside that spurious range, as claimed. (`gazebo-testing` quotes 35.764 s
for the hdem fit vs my 35.821 s — a 57 ms difference from interpolation direction; immaterial, the
mechanism and its sign are confirmed.) **The lag absorbed into the offset was precisely the quantity
§4.2 then drew a conclusion about — which is why the conclusion inverted.** That explanation is
correct.

### R.2.4 The recomputed metrics — all reproduce

At offset 33.3425 s, over the stored segment windows:

| Quantity | `validation` | report §4.2b |
|---|---|---|
| P2 climb `dh` | **+1.159** | +1.159 |
| P2 climb `dhdem` | **+1.305** | +1.305 |
| P2 climb `th` | **0.5420** | 0.5420 |
| P2 climb in-window `hdem` max | **100.15 m** | 100.155 m |
| P4 descent `dh` | **-1.151** | -1.151 |
| P4 descent `dhdem` | **-1.344** | -1.344 |
| P4 descent `th` | **0.4199** | 0.4199 |
| whole-log `hdem` max / `hin` max | **101.054 / 105.870** | 101.05 / 105.87 |
| `hin - h` at climb release | **+6.290 m** | +6.29 m |
| at the OLD 34.939: P2 `dh` / `dhdem` / `th`, `hdem` max | **+1.543 / +1.311 / 0.5457 / 101.05** | identical |

All reproduce, and the corrected P2/P4 values match the table I published independently in my
2026-09-03 V-3 finding. The `f` word is 0 in 1456/1457 records and `pmin`/`pmax` are constant at
-25 deg / +15 deg, never reached — confirmed.

### R.2.5 Is the corrected conclusion properly hedged? — YES

The rewritten §4.2c: (i) marks the old statement SUPERSEDED and quotes it verbatim; (ii) states the
chain 2.000 -> 1.305 -> 1.159 m/s; (iii) attributes 77-83 % to `HDEM_TCONST` shaping and 17-23 % to
the aircraft-plus-loop; (iv) states the aircraft **lags** its shaped demand and shows the tracking
error growing in the lag direction in both directions; (v) explicitly declines to attribute the
residual lag to either the airframe or the height-loop bandwidth, cites the absence of a demand-rate
sweep, calls the no-saturation evidence "suggestive, not conclusive", marks it `DATA_REQUIRED` and
ties it to V-2. **It does not replace one overreach with the opposite one.** I specifically checked
for the symmetric failure — a claim that "the airframe limits the ramp rate" — and it is not made.

Two small caveats on §4.2c, neither of which changes the conclusion (recorded as V-12, INFO):
- `dhin = 2.000 m/s` is the **commanded** `FBWB_CLIMB_RATE`, not a measured slope; the measured
  in-window raw `hin` slope is 1.915 m/s. Using the measured value shifts the split from 83/17 to
  81/19, inside the report's own stated 77-83 % band. Immaterial, but the label should say
  "commanded".
- The "comparable lag in both directions argues against an excess-thrust limit" argument leans on
  the descent-side number, and the descent is the phase affected by `PROPULSION_HIGH_J_WINDMILLING`
  (35.2 % of P4 motor-samples clamped, thrust floored at 0 N). The bias direction is **favourable to
  the report's conclusion** — missing windmilling drag makes the descent easier, so the true descent
  lag would be at least as large — so the argument survives, but it should carry the cross-reference.

**Verdict on V-3: CLOSED.**

---

## R.3 RULING on `SETTLE_LIMIT_NOT_FULLY_FALSIFIABLE_P3_AIRSPEED`

**Ruling: ACCEPTABLE TO CARRY as a declared, non-gating open limitation. It does NOT invalidate the
P3 airspeed pass. It does not gate the stage.** Reasons:

1. **This run's pass is a completed measurement, not an extrapolation.** Settling occurred at
   25.269 s, 4.7 s inside the 30.0 s reference window and 7.23 s below the 32.502 s limit, with
   14.7 s of continuously in-band tail after it. Nothing was inferred beyond the data.
2. **The flag can never manufacture a pass.** It is computed per channel, stored in the result JSON,
   and is not referenced anywhere in `verdict()`. I checked the gating path directly.
3. **It is honestly declared** — in the source-of-truth yaml with numbers, cause, the remedy, an
   explicit "not a reason to widen the band, lower K, shorten the tail, or re-fly", and the note
   that none of those were done. The equivalent condition on the 2026-09-02 cross-check run (limit
   30.114 s vs a true 25.004 s window) is disclosed too, including the fact that the hard-coded
   `P3_SETTLE_S` makes the flag's reference window wrong when the function is applied to that run.
   That is a specialist reporting a defect that weakens their own result. Correct behaviour.

**However, the limitation is MIS-STATED, and in the conservative direction.** The claim is that a
settling time between 30.0 and 32.502 s "could not be measured — it would be scored *never settled*
(a FAIL)". That is **not what the code does.** In `settling_analysis()`:

```python
never_spd = (rows[-1][0] - t_spd) < 1e-9
```

`never` is true only if the **last sample of the segment** is out of band. A last band exit at
31 s leaves 9 s of in-band samples, so `never` is False, the value is reported, and it is compared
against the 32.502 s limit — and passes. I confirmed this empirically by running the exact
last-exit / `never` logic on synthetic P3-like segments (same 39.98 s length, same 10 s tail,
same 0.5 m/s band, A0 1.60 m/s, T 5.63 s, envelope constant swept):

| envelope tau | last band exit | `never` | scored |
|---|---|---|---|
| 26 s | 28.548 s | False | PASS |
| 28 s | **33.852 s** | False | **FAIL (exceeds the 32.502 s limit)** |
| 30 s | 34.112 s | False | FAIL |
| 34 s | 39.676 s | False | FAIL |
| 40 s | 39.936 s | True | FAIL (never settled) |

**The gate IS falsifiable inside its own 40 s segment**: a settling time in roughly
(32.502, 39.93] s is measurable and produces a FAIL. What the `limit_exceeds_observable_window`
flag legitimately warns about is narrower and milder — for `t_settle > 30 s` the tail-mean
**reference** would be partly defined from an unsettled state, degrading the quality of `A0` and
`t_settle`. That is a real caveat about reference cleanliness, not an unfalsifiable criterion, and
the stated remedy ("a >= 42.5 s segment is required to make it falsifiable") is therefore
mis-targeted: a longer segment would improve reference cleanliness, not restore falsifiability.

So my ruling stands and is if anything stronger than requested: carry it, and correct the text
(V-10 below). No re-fly is warranted, and re-flying to erase a reported limitation would itself be
the kind of after-the-fact adjustment CLAUDE.md forbids — which `controls-integration` correctly
declined to do.

---

## R.4 Bookkeeping — no parameter moved, no re-fly, raw data intact

**No forbidden parameter moved.** `git diff HEAD` over the whole tree is still exactly two tracked
files — the cruise-stage comment block (+10/-1, re-verified as comment-only: the only touched
statement is `out["nav_alt_error_m"] = minmaxmean(aerr)`, unchanged) and the
`ardupilot_fbwb_tecs_baseline.yaml` documentation correction (+15/-1), both reviewed and approved in
§3 on 2026-09-03. `config/ardupilot/falcon_v2_sitl.parm` (mtime 2026-09-02 23:28) and
`model/model.sdf` (2026-09-02 23:45) are byte-identical to `HEAD`. **No file under `plugins/`,
`config/`, `docs/source_of_truth/{geometry,aerodynamics,propulsion}` has an mtime later than
2026-09-03 00:00.** The live parameter dump in the new result JSON carries all 11 `TECS_*` values at
their firmware defaults, `PTCH_TRIM_DEG 2.49`, and the `RLL_RATE_*`/`PTCH_RATE_*` values committed
at `22eb5e4`; all **21** param preconditions are `true`. No PID, `TECS_*`, `PTCH_TRIM_DEG`, aero,
propulsion, actuator, sensor or mass/CG/inertia value moved.

**CG duality trap: still clean.** Neither `(0.168309, 0, 0.100000)` nor `(0.0637, 0, -0.0210)`
appears in anything touched by either fix. No CG value is used by this stage.

**No re-fly.** `..._dataflash/00000001.BIN`, `..._arduplane_log.txt`, `..._gz_log.txt` and
`..._log.txt` all retain their 2026-09-03 23:23:33 mtimes. The result JSON carries
`overall_result: REANALYZED`, `reanalyzed_from: ..._timeseries.json`, and the original flight
`timestamp: 2026-09-03T20:20:39Z`. The `--reanalyze` path takes no Gazebo, SITL or MAVLink action.

**Raw flight data verified unaltered.** Two independent proofs:

1. **Byte-exact fixed point.** I re-serialised `..._timeseries.json` through the exact transform
   `write_outputs()` applies (`json.dump(ts_doc, default=str, separators=(",", ":"))` over the same
   reconstructed document). Result: **md5 `a0ba1fff52576077c4c0a3976586620d`, 8 514 452 bytes —
   identical to the file on disk.** The re-serialisation is therefore idempotent and could not have
   altered, lost or reordered any sample. Segment sample counts 863/168/769/179/767 = **2746**,
   spans 44.997/8.719/39.980/9.260/39.975 s = the same 142.96 s flight.
2. **Full-precision agreement with the pre-fix log.** Every analysed quantity recomputed at 00:02 on
   2026-09-04 reproduces `..._log.txt` (written 23:23:33, never rewritten) to the last float digit:
   settling 25.26861023902893 / 8.485944271087646; decay ratios 0.5121743682491768,
   0.5587564288026494, 0.5556744521362708, 0.4866732023996175; hold means 17.925597 +- 0.239048 /
   17.949750 +- 0.416430 / 17.916526 +- 0.188374; throttles 0.49085624012638307 / 0.5404315476190471
   / 0.4200586592178773; achieved steps +10.521232 / -11.153656 / -0.632424.

`..._per_sample.json`: **2746 rows x 30 columns**, one row per stored sample, spot-checked at rows
0 / 500 / 1500 / 2745 against the timeseries — bit-identical floats. **0 non-finite values.** It is a
deterministic function of the timeseries, which is itself proven unaltered.

Cross-check against the independent, untouched dataflash BIN: `TECS.h` matches the stored samples to
**0.0148 m RMS** across the whole flight at the fitted offset (R.2.1), and hold-window heights agree
to 5 mm. The stored samples and the BIN tell the same story. **`gazebo-testing`'s bookkeeping claim
is verified.**

---

## R.5 New findings

### CRITICAL — none.

### MAJOR

**V-9 (MAJOR, NOT stage-blocking) — `gazebo-testing` report §2.3 over-claims the closed-loop damping.**
The sentence *"the loop damps the mode about 1.4x faster than the free airframe's own Lanchester
phugoid envelope (20.9 s) would"* is true of `tau_implied` (20.895/14.669 = 1.42), which is a
**peak-to-band settling** statistic — but it is written as a claim about how the loop **damps the
mode**, and by that measure it is backwards. From the report's own log-decrement figures the
closed-loop asymptotic envelope is `T/ln(1/r) = 5.6329/0.21735 = ` **25.91 s** (cross-checked
`T/(2*pi*zeta) = 25.92 s`) against `TAU_REF = 20.895 s` — i.e. **1.24x slower**, which is also what
the report's own `zeta = 0.035` (closed loop) vs `0.062` (free airframe) implies two paragraphs
later. The document therefore contains both the claim and its refutation. No acceptance check, no
threshold and no measurement depends on the sentence, and the report immediately and correctly
states "This does not close validation's V-2" — which is why I classify this MAJOR but **not
stage-blocking**, unlike V-3 where the underlying table was numerically wrong and the conclusion
inverted. The risk it creates is real but bounded: a future reader could cite it to argue the
closed-loop damping is adequate and de-prioritise V-2. **It must be corrected before this report is
cited as evidence about damping.** Owner: **`gazebo-testing`** (report wording; the criterion itself
is correct and stays as it is). This finding **reinforces V-2**: the mode is less damped per unit
time than the free airframe, and an acceptable damping ratio for Falcon V2 remains `DATA_REQUIRED`.

### MINOR

**V-10 — the `SETTLE_LIMIT_NOT_FULLY_FALSIFIABLE_P3_AIRSPEED` text misdescribes the code.**
`docs/source_of_truth/controls/ardupilot_tecs_energy_management.yaml`, `limitation_falsifiability`,
states that a settling time in (30.0, 32.502] s "could not be measured" and "would be reported as
never settled ... which the test scores as a FAIL". `never` is set only when the **last sample** of
the segment is out of band, so such a value is in fact measured and compared against the limit
(R.3, with an empirical sweep). The gate is falsifiable inside its own segment: a settling time in
~(32.502, 39.93] s is measurable and FAILs. The real, milder caveat is that for `t_settle > 30 s` the
tail-mean **reference** is defined from an unsettled state. The "a >= 42.5 s segment would make it
falsifiable" remedy is correspondingly mis-targeted. Errs conservatively and gates nothing, but it is
an incorrect statement about acceptance-code behaviour in a source-of-truth file.
Owner: **`controls-integration`**.

**V-11 — the report's revision-2 reproduction command would launch a live flight.**
Report §1.5 prints `python3 tests/gazebo/scripts/test_ardupilot_tecs_climb_descent_energy.py
--reanalyze`. `main()` dispatches on `len(sys.argv) > 2 and sys.argv[1] == "--reanalyze"`, so with
the path argument omitted the guard is false and the script **falls through to the flight path** and
would attempt to arm and fly. The command must include the timeseries path. Owner:
**`gazebo-testing`**.

**V-12 — the report does not surface the falsifiability flag, and two §4.2c labels need tightening.**
(a) `SETTLE_LIMIT_NOT_FULLY_FALSIFIABLE_P3_AIRSPEED` appears in the yaml and as
`settle_limits.airspeed.limit_exceeds_observable_window: true` in the result JSON, but nowhere in the
`gazebo-testing` report — not in §2.3, §7, §11 or §12. It should be listed with the other open
limitations. (b) `dhin = 2.000 m/s` should be labelled the **commanded** `FBWB_CLIMB_RATE`; the
measured in-window raw slope is 1.915 m/s (shifts the split 83/17 -> 81/19, inside the stated
77-83 %). (c) the "comparable lag in both directions" argument should cross-reference
`PROPULSION_HIGH_J_WINDMILLING` on the descent side, noting the bias direction is favourable to the
conclusion. Owner: **`gazebo-testing`**.

**V-13 — the `--reanalyze` result artifact drops provenance blocks the flight artifact carried.**
`reanalyze()` rebuilds `R` from scratch, so the current `..._result.json` no longer contains `mode`,
`parameter_policy`, `open_limitations: ["PROPULSION_HIGH_J_WINDMILLING"]` or `reference_constants`
(mass, S_ref, g, trim references, prior-stage measurements) — all of which the revision-1 artifact
carried. The primary machine-readable artifact for this stage is now weaker in provenance than the
one it replaced. Owner: **`controls-integration`** (owns the `--reanalyze` path).

### INFO

- **I-8 — amplitude normalisation removes only part of the run-to-run spread.** Raw settling time
  went 20.138 -> 25.269 s (+25.5 %) between the two runs; `tau_implied` went 11.415 -> 14.669 s
  (+28.5 %), because `A0` only grew 1.431 -> 1.598 m/s (`ln(A0/B)` +10.5 %). So `A0` explains about
  10 of the 25 percentage points; the rest is genuine run-to-run variation in the decay itself. The
  new gate is still a far better discriminator than the old one — margin to `K = 1.0` is 0.30-0.45
  in normalised units, i.e. roughly 1-1.5 observed run-to-run spreads, against the old criterion
  whose spread was 19x its own margin — but it is not a wide margin, and the residual variability is
  V-2 physics.
- **I-9 — `obs = P3_SETTLE_S - SETTLE_TAIL_S` is hard-coded for both phases.** P5 is evaluated
  against P3's segment length. Numerically identical today (both 40.0 s) and the flag is non-gating.
  `controls-integration` already documented the equivalent issue for the 2026-09-02 cross-check.
  Latent only.
- **I-10 — the V-5 rename is complete and consistent.** `no_growing_oscillation_in_hold_windows` is
  in the core list, the gated/reported window split is recorded in
  `analysis.oscillation_growth_scope` as declared INFO, and the two ramp-window ratios (1.234,
  1.416) are preserved rather than discarded. My 2026-09-03 §6 conclusion is unaffected.

---

## R.6 Stage disposition after re-review

| # | Finding | Class | Blocking | Owner | Status |
|---|---|---|---|---|---|
| V-1 | settling criterion not physically well-founded | MAJOR | was YES | `controls-integration` | **CLOSED** |
| V-3 | dataflash alignment error, conclusion inverted | MAJOR | was YES | `gazebo-testing` | **CLOSED** |
| V-5 | `no_growing_oscillation` scope vs name | MINOR | no | `controls-integration` | **CLOSED** (renamed) |
| V-7 | "monotonically" wording in report §7 | MINOR | no | `gazebo-testing` | **CLOSED** (corrected) |
| V-8 | sink-rate figure (1.15 window mean vs 1.924 instantaneous) | MINOR | no | `gazebo-testing` | **CLOSED** (corrected) |
| V-2 | lightly damped, uncharacterised closed-loop mode; `TECS_CLMB_MAX`/`SINK_MIN` not airframe-derived | MAJOR | no | `controls-integration` + `propulsion` | **OPEN**, reinforced by V-9/R.1.5 |
| V-9 | report §2.3 over-claims closed-loop damping | MAJOR | **no** | `gazebo-testing` | **NEW, OPEN** |
| V-4 | 0.5 m preferred re-settle bound unachievable by construction | MINOR | no | `controls-integration` | OPEN (still false, correctly non-gating) |
| V-6 | AP_TECS line-citation drift | MINOR | no | `controls-integration` | OPEN |
| V-10 | falsifiability limitation text misdescribes the code | MINOR | no | `controls-integration` | **NEW, OPEN** |
| V-11 | reproduction command would launch a live flight | MINOR | no | `gazebo-testing` | **NEW, OPEN** |
| V-12 | falsifiability flag absent from the report; two §4.2c labels | MINOR | no | `gazebo-testing` | **NEW, OPEN** |
| V-13 | `--reanalyze` artifact drops provenance blocks | MINOR | no | `controls-integration` | **NEW, OPEN** |

**UNRESOLVED STAGE-BLOCKING CRITICAL OR MAJOR FINDINGS: ZERO.**

Both stage-blocking MAJORs are closed on their merits, verified independently rather than accepted
on report. The remaining MAJORs (V-2, V-9) are open engineering/reporting items with named owners
and neither implicates the simulated aircraft or gates this stage. The physics, units, frames,
signs, force directions, energy bookkeeping, control directions and numerics cleared on 2026-09-03
are unchanged — the raw flight data is proven byte-identical, so those conclusions carry over
without re-audit.

**Still explicitly forbidden as a response to anything above:** changing any `TECS_*` value, PID,
`PTCH_TRIM_DEG`, control-surface travel, aerodynamic coefficient, actuator or propulsion parameter,
or mass/CG/inertia. V-2 must be root-caused with data, not tuned away, and V-9/V-10/V-11/V-12/V-13
are documentation corrections only — no re-fly, no threshold change and no segment lengthening is
warranted by any of them.

Re-reviewed by `validation`, 2026-09-04. Read-only: no engineering parameter was edited.

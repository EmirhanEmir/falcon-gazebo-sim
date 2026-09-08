# VALIDATION REVIEW RECORD — ARDUPLANE_NAVIGATION_AND_AUTOTUNE_VALIDATION (2026-09-08)

**Verdict: READY.**

Independent read-only review by `validation`. This file is the persistent record of that review;
it was written by the coordinator because `validation` has no write access. It **persists** the
existing verdict and findings from the stage artifacts and logs — it does not re-run, re-analyse,
re-interpret or extend them, and it makes no new engineering decision. Every number below was read
out of the named artifact, not out of an agent summary.

Authorship convention follows `docs/validation/2026-09-02_*.md` and `2026-09-03_*.md`: this is
`validation`'s review report, not a stage record. Claims originating with `gazebo-testing`
(execution) or `controls-integration` (harness) are attributed inline.

---

## 1. Stage and verdict

| | |
|---|---|
| Stage | `ARDUPLANE_NAVIGATION_AND_AUTOTUNE_VALIDATION` |
| Campaign verdict | **`NAVIGATION_PASS`**, 4/4 scenarios, `any_blocker: false` |
| `validation` ruling | **READY** |
| Campaign timestamp | `2026-09-08T03:43:35Z` |
| Source of record | `tests/gazebo/results/ardupilot_navigation_validation_summary.json` |

`failed_checks_by_scenario` and `blocking_checks_by_scenario` are empty for A, B, C and D.

Two review rounds were required. `validation`'s first review returned **NOT_READY** on one MAJOR
harness finding (§6.1); a second MAJOR was introduced by the fix for the first and caught
downstream (§6.2). Both are closed. `validation` recorded that **no aircraft, aerodynamic,
propulsion, geometry, frame, sign or numerical defect exists in this stage** in either round.

## 2. Agent chain

`controls-integration` (stage primary — pre-flight, harness, both fixes) →
`gazebo-testing` (live campaign, then two re-analysis passes, no re-fly) →
`validation` (two independent read-only reviews). `general-purpose` was not used.

Both harness defects were caught downstream of their author: the RTL windowing defect by
`validation`, the oscillation-gate regression by `gazebo-testing`.

## 3. Navigation results

All four flown live, zero wind (`SIM_WIND_SPD` live-asserted 0), safe airborne start, full free
6-DOF, genuine ArduPlane navigation modes. `validation` confirmed mode entry from dataflash `MODE`
messages rather than from claims, and confirmed the airborne teleport ends before every measured
window.

| | Scenario | Verdict | Evidence |
|---|---|---|---|
| A | `GUIDED_SINGLE_TARGET` | **PASS** | 110 s; `wp_dist` 799 → 53 m (closure 0.934); heading error 22.4 → 0.47 deg; nav-sign agreement 0.980 |
| B | `LOITER_ORBIT` | **PASS** | 150 s; 6.55 orbits; radius 60 m commanded; drift −2.32 m over the window; CW direction correct; not spiralling |
| C | `RTL_RETURN_TO_HOME` | **PASS** | home 501 → 53.9 m, arrival t=33.3 s; then 97 s home orbit at radius 60.31 m (cmd 60.0), std 1.16 m, fitted centre offset from home **0.520 m**, rel-alt 100.017 m (`RTL_ALTITUDE` 100.0), CW 4.44 orbits |
| D | `GUIDED_SEQUENTIAL_WAYPOINTS` | **PASS** | 3/3 legs (+45 / −55 / +15 deg turns); commanded +15.0 m altitude step → measured **+15.13 m** |

Scenario C's fitted-centre offset was verified by `gazebo-testing` with four independent
estimators (0.520 / 0.463 / 0.543 / 0.485 m) and by `validation` with a fifth (0.525 m). Two of
those fits run on Gazebo ENU ground truth and never touch lat/lon or ArduPilot geodesy.

Airspeed: campaign minimum **16.43 m/s**, never below `AIRSPEED_MIN = 16`. Roll/pitch tracking,
steady windows: roll RMS 4.18 / 1.58 / 3.82 / 8.22 deg and pitch RMS 2.39 / 1.67 / 2.15 / 7.55 deg
for A/B/C/D.

## 4. AUTOTUNE_REQUIRED = **NO**

AUTOTUNE was **not run**. `autotune.status` in the summary artifact is `DORMANT_NOT_EXECUTED`.
No PID was written; before/after PID values do not exist because nothing changed.

The harness's own decision support raised a review flag on scenario D
(`autotune_decision.indicated_by_scenario` = `{A: false, B: false, C: false, D: true}`,
`any_scenario_indicates_review: true`). Per the harness's recorded policy, the only action that
flag authorises is handing the evidence to an independent review. That review was performed and
concluded **NO**, on these grounds (`validation`, independent; `gazebo-testing` reached the same
conclusion separately):

1. The crossed threshold is uncalibrated. `TH_AUTOTUNE_ROLL_RMS_DEG` and
   `TH_AUTOTUNE_PITCH_RMS_DEG` carry class `ASSUMPTION_DECISION_SUPPORT_ONLY` and are
   `DATA_REQUIRED` — no measured Falcon V2 tracking-error reference exists. Crossing it carries no
   calibrated meaning and gates nothing.
2. RMS is the wrong statistic for this signal: roll mean_abs 1.43 / 0.76 / 1.36 / 3.50 deg against
   RMS 4.18 / 1.58 / 3.82 / 8.22 — transient-dominated, not broadband.
3. The continuous-demand case is decisive. B holds a 28 deg banked orbit for 150 s at roll
   mean_abs **0.758 deg**, p95 error **1.068 deg**, 2.3 m radius drift over 6.55 orbits.
4. D's excursions are demand-side discontinuities, not loop failure: two roll clusters (0.68 s,
   1.31 s) at leg transitions where L1 itself steps `nav_roll` toward `ROLL_LIMIT_DEG`, and five
   pitch clusters (0.43–1.63 s) at the TECS demand reversal on the commanded +15 m step. Each
   recovers in ~1 s.
5. Control authority was never exhausted — the diagnostic that would actually indicate bad gains.
6. No growing oscillation on any gated channel, under either estimator, and none under the
   final detector that is ~27× more sensitive than the one in place when the flag was raised.
7. Provenance: the current gains carry `FALCON_V2_MANUFACTURER_INITIAL_PID` (Titan Dynamics
   Falcon V2 Build & User Manual Rev 1.0). AUTOTUNE would overwrite manufacturer-provenance values
   with Gazebo-derived, explicitly non-transferable ones — the same rule already recorded for
   `TECS_PTCH_DAMP 0.6` — while nothing is broken.

`AUTOTUNE_LEVEL 8` remains recorded in `config/ardupilot/falcon_v2_sitl.parm` under
`MANUFACTURER_RECOMMENDED_AUTOTUNE_LEVEL`, unused by this stage.

## 5. Saturation, NaN/Inf, oscillation

- **No saturation.** `±45 deg` hard-limit duty **0.0000** and longest continuous run **0.000 s**
  on every surface in every scenario. `target_clamp_active` / `effort_clamp_active` never
  asserted. Throttle never 0 % and never 100 %. Surface p95: aileron 0.35–1.74 deg, elevator
  7.1–8.5 deg, rudder ≤0.60 deg.
- **No NaN/Inf.** Zero non-finite values across all four result JSONs, all four timeseries and the
  summary; `validation` independently scanned 10,165 samples and found none. Zero errors, asserts
  or signals in all four `gz` logs and all four `arduplane` logs.
- **No growing oscillation** on any gated channel, under the final detector and under the
  project's previously-validated `detrended_growth` estimator as a second opinion. One
  `growing=True` exists on the **non-gating raw state** channel `D/roll_deg` — see §8.

## 6. MAJOR findings raised and closed

Both are **harness/test-logic** defects. Neither is a physics finding, and neither was fixed by
changing an aircraft parameter.

### 6.1 RTL windowing — `rtl_turned_toward_home` computed over an unwindowed sample set

Raised by `validation` (round 1). `rtl_extra()` computed its bearing / sign / convergence metrics
over the full sample set, including the 97 s post-arrival home loiter. ArduPlane runs
`update_loiter()` in that phase; the aircraft flies tangent to the circle, so
`|bearing_to_home − heading| = 88.79 deg` **by construction**. The harness already applied that
exclusion to scenario B and had simply never classified the post-arrival RTL phase as a loiter.
The consequence was a `NAVIGATION_FAILED` verdict on a correctly flown RTL.

`validation` ruled it a metric error rather than a navigation defect on five independent grounds,
the decisive one being that **the check was broken in both directions**: post-arrival its
*passing* half was equally meaningless (sign agreement 0.9929, because a +88.79 deg bearing error
and a +28.27 deg right bank always agree in a clockwise orbit), and the smoke run had *passed* the
check on two tangent numbers that happened to fall the right way. The verdict tracked window
composition, not aircraft behaviour. `validation` recorded explicitly: *"Had the correctly-windowed
metric failed, my ruling would be the opposite."*

**Closed.** `rtl_extra()` rewritten with three explicit windows — closure on the full set
(unchanged, still sees 501 → 53.9 m), bearing/sign/convergence on `nav_window` (both halves, which
also closed a 0.8058-vs-0.80 thin margin), loiter-at-home on a new `home_loiter_window()`.
`nav_s` is now computed before `extra` at both call sites. `rtl_turned_toward_home` was **retired**
as redundant rather than relaxed; its windowed numbers survive as a non-gating cross-check
(n=517, sign agreement 1.000, heading error 133.11 → 0.150 deg) and tangency is recorded so it
cannot be reinstated by accident. A **stronger** positive check `rtl_loiters_at_home` was added in
its place: five conjoined conditions, circle fitted in the home-relative NE frame, and fewer than
20 post-arrival samples fails rather than passes. `validation` assessed the replacement as
*"strictly stronger — it can fail five independent ways where the old one could only fail on a
tangency artefact."*

### 6.2 Vacuous oscillation gate — coverage regression introduced by a good-faith tightening

Raised by `gazebo-testing` (round 2), against a change made in response to a `validation` MINOR.
An identifiability gate (`max/min` half-cycle spacing ≤ 3.0) made the oscillation gate
**vacuous**: gated channels reported `UNDERDETERMINED` went from 6/19 to **19/19**. Thirteen
channels that had been evaluating and passing on merit (decaying envelopes, σ<0) became ungated,
and `no_growing_oscillation` returned True in all four scenarios because nothing was being
evaluated. It changed no verdict — which is why it nearly shipped. Measured detection floor on the
real scenario-C `roll_err_deg` channel had risen to a **40 deg** growing roll oscillation.

Root cause was two extreme-value statistics with one shared origin, deeper than the initial
diagnosis: the peak-prominence floor was keyed to **peak-to-peak**, so one large transient inflated
the floor and filtered out the ordinary cycles of the very oscillation being measured — which
simultaneously collapsed the peak span *and* manufactured the tiny spacings that made `max/min`
explode. Fixing the spacing statistic alone would have left 16/19 channels ungated.

The author's synthetic verification had passed because those signals contained *nothing but* the
injected oscillation (spacing ratio 1.79 at every amplitude); the same injection superimposed on a
real channel scored 3.66–7.27 and was silently dropped.

**Closed.** Estimator chain replaced end to end with robust statistics: `robust_detrend()`
(Theil–Sen slope, median-residual intercept); prominence `0.25 × median` half-cycle peak
(was `0.05 × peak-to-peak`); coherence `max_gap_frac = max gap ÷ peak span ≤ 0.50`, bounded and
scale-free (was `max/min ≤ 3.0`, unbounded); envelope slope by `theil_sen_slope()`. The old
`max/min` spacing and endpoint ratio are retained as emitted diagnostics so the regression stays
visible — `spacing_ratio_max_over_min` = 224.1 on the C `roll_err_deg` channel is the record of why
the old statistic was the wrong one.

`TH_OSC_MIN_PEAKS = 6` and the peak-span criterion were **sustained** by `validation` in both
rounds, including under the changed prominence floor: minimum peak count constrains observed
half-cycles, which is a property of the signal rather than of the floor.

`TH_OSC_ENDPOINT_RATIO_MIN` was retired to reported-only (class `RETIRED_REPORTED_ONLY`). This was
escalated to `validation` as a loosening; `validation` ruled the premise inverted — removing a
conjunct from a FAIL condition (`growing = env≥1.3 AND endpoint_consistent` → `growing = env≥1.3`)
makes the gate fail in strictly more cases. It removes protection against false *alarms*, not
against *misses*, and cannot produce a missed divergence. Verified load-bearing in **zero** cases
across four scenarios plus three D legs. **Approved.**

## 7. Final robust oscillation detector

| | before regression | during regression | after fix |
|---|---|---|---|
| Detection floor, exp(t/60) T=6.3 s, injected into real `C/roll_err_deg` | 40 deg | 40 deg | **1.5 deg** |
| Detection floor, exp(t/90) T=12 s | not detected ≤40 deg | not detected ≤40 deg | **3 deg** |
| Gated channels evaluated | 13/19 | 0/19 | **17/19** |

Coverage is better than the pre-regression baseline, and sensitivity on real flight data is now
within ~1.5× of the clean-synthetic ideal rather than 40×. `validation` ruled this **not
restored-by-loosening**: a loosening degrades sensitivity, and on a clean oscillation the new
prominence floor (`0.25 A`) is 2.5× *stricter* than the old one (`0.1 A`) — it is more permissive
only where an outlier had inflated the old floor, which is the signature of a robustness fix.

The two remaining ungated channels are genuinely unidentifiable, not conveniently excluded:
`A/xtrack_error_m` (raw peak count 3 — only three half-cycles exist in the signal, so this is not
prominence filtering hiding cycles) and `C/xtrack_error_m` (5 filtered / 9 raw, 2.5 cycles). Both
are monotonic cross-track approach ramps, both were ungated before the regression, and both carry
non-gating provisional σ/envelope figures plus a `detrended_growth` second opinion showing they are
nowhere near growing. Scenario C `roll_err_deg` is now `OSCILLATORY` — evaluated and gated —
48 peaks, σ = −0.03957, envelope ratio 0.00927, not growing **on its merits**, cross-checked
estimator-independently at 0.0423.

`max_gap_frac ≤ 0.50` still rejects the original C 4-peak train (0.711) and two-cluster fixtures
(0.508, 0.719) while accepting and correctly flagging a continuous growing train (0.027). It also
fires on real data (`C/roll_deg`, gap 0.764), so it discriminates rather than permits.

## 8. Remaining MINOR / advisory findings

None gate this stage.

| # | Finding | Owner |
|---|---|---|
| 1 | `growing_channels_non_gating: ['roll_deg']` is emitted with no interpretive note, despite being the only `growing=True` anywhere in the campaign record. Given this project's history of category errors being read as physics, it should be annotated. | `controls-integration` |
| 2 | Per-leg cross-track envelope growth is arguably ill-posed on a closing-**range** channel (`xtrack_error = wp_dist − WP_LOITER_RAD` in GUIDED), the same category as the RTL bearing issue. Consider detrending against expected closing geometry. Non-urgent. | `controls-integration` |
| 3 | D leg1 cross-track envelope ratio **1.2293** against the 1.30 gate — σ = +0.00942 /s against a critical +0.01197 /s, i.e. 79 % of the way to the gate. Thinnest margin in the campaign and the only real channel in that territory. Passes on merit; watch if this stage is re-flown. | watch item |
| 4 | Elevator operating point sits close to the XFLR5-validated boundary: p95 is 7.1–8.5 deg in **every** scenario including benign cruise and loiter (70–85 % of the ±10 deg validated range), with a 1.26 s burst to 11.37 deg in D (24 samples, 1.53 %, 13.7 % beyond). Does not invalidate D — the gate is `p95 ≤ 10` and 8.52 passes — but for that 1.26 s the elevator pitching increment is not backed by source data. An extended XFLR5 elevator sweep is **DATA_REQUIRED**. | `aerodynamics` |

On finding 3, `validation` recorded the reading that a mildly positive envelope slope on a closing
approach is expected geometry, corroborated by the co-located `roll_err` on the same leg
(σ = −0.02639, envelope 0.5791 — strongly decaying), where a genuine lateral divergence would
appear. On finding 1, `validation` recorded that the detected period of 17.43 s **is the leg
cadence** (legs 26.2 / 25.7 / 30.1 s), not an airframe dynamic mode, and that the matching error
channel `D/roll_err_deg` is 0.909 and not growing.

Closed by this stage: the MAJOR of §6.1; the endpoint-guard MINOR by retirement; the coherence
MINOR superseded by the better root-cause fix of §6.2; and the threshold-provenance MINORs by an
emitted machine-readable `class` field plus `threshold_class_census()` with a self-check
(`prose_mentions_assumption_but_class_does_not: []`), so the hand-count drift previously found
cannot recur. Census: 32 thresholds, 27 gating / 5 non-gating.

## 9. Carried-forward open limitation — High-J / windmilling

`HIGH_ADVANCE_RATIO_WINDMILLING_PROPELLER_REGIME` — **OPEN / DATA_REQUIRED / NON-GATING**, owner
`propulsion`. Recorded verbatim in `known_open_limitations` of every result JSON and of the
campaign summary:

> The propulsion model's behaviour in the high-J / windmilling regime is not validated and the
> required data does not exist in this repository. This stage does NOT resolve, work around, or
> tune anything related to it. It is carried forward unchanged. It is explicitly NON-GATING for
> this stage's verdict; where a descent or an idle-throttle segment enters that regime, any
> propulsion number recorded there is reported as-is and must not be read as validated.

Observed in this campaign: propeller thrust clamped to exactly 0.000 N on 0.56 / 1.27 / 0.77 /
1.92 % of motor-samples in A / B / C / D, at advance ratio **J = 0.644–0.797** (above the APC
13×6.5E geometric pitch ratio 0.5), each coincident with `interpClamped = 1`. The model returns
zero rather than negative thrust there, so it slightly under-predicts drag in those brief segments.
`validation` confirmed the limitation was neither hidden nor silently resolved.

## 10. No aircraft or physics defect

`validation` recorded, in both rounds: **there is no aircraft, aerodynamic, propulsion, geometry,
frame, sign or numerical defect in this stage.** The physics and the SITL integration are sound.
The only two MAJORs were test-logic defects (§6), and both are closed.

Independently verified by `validation` (not taken from agent reports):

- **Genuine navigation modes** — dataflash `MODE` messages: A `0→5→15` (GUIDED), B `0→5→12`
  (LOITER), C `0→5→15→11` (RTL), D `0→5→15`. Mode 8 (AUTOTUNE) never entered.
- **Full free 6-DOF, no physics bypass** — wrench publishers appear only in the `bring_up` phases,
  with `clear_wrench` before FBWA handoff; `run_nav_segment` publishes nothing but neutral RC. C's
  measured window starts at t=26.2 s; teleport/hold ended at t<1 s.
- **Locked parameters preserved** — dataflash `PARM` shows single values across all four runs:
  `TECS_PTCH_DAMP 0.6`, `PTCH_TRIM_DEG 2.49`, all eight roll/pitch PIDs at 0.25 / 0.125 / 0.002 /
  0.125. The runner uses `-w --defaults`, i.e. a fresh EEPROM per scenario. Only ArduPilot
  housekeeping parameters changed (`BARO*_GND_PRESS`, `ARSPD_OFFSET`, `STAT_*`, `MIS_TOTAL`).
- **Frames and signs** — maximum heading |difference| between the gz-ENU-derived compass heading
  and ArduPlane heading is **1.657 deg**; roll-sign agreement 1.0000 (A/B/C) and 0.9994 (D, one
  sample) over ~10k samples. The radial-xtrack identity `xtrack_error = wp_dist − WP_LOITER_RAD`
  held (p50 residual 0.24–0.39 m, p95 0.44–0.56 m).
- **Repository integrity** — `model/model.sdf` md5 `52af99ed…` and
  `config/ardupilot/falcon_v2_sitl.parm` md5 `1b5ac47a…` identical to `validation`'s round-0
  values; zero modified tracked files; dataflash BIN mtimes unchanged, confirming no re-fly.
  Timeseries content byte-identical to the flight-run baselines across both re-analysis passes.
- **CG duality** — no CG constant appears anywhere in this stage's files; not applicable, no trap
  triggered.

No engineering parameter was written at any point in this stage, in either round.

## 11. Final verdict

ARDUPLANE_NAVIGATION_AND_AUTOTUNE_VALIDATION_READY

---

### Artifacts of record

- `tests/gazebo/results/ardupilot_navigation_validation_summary.json`
- `tests/gazebo/results/ardupilot_navigation_scenario_{A,B,C,D}_result.json`
- `tests/gazebo/results/ardupilot_navigation_scenario_{A,B,C,D}_timeseries.json`
- `tests/gazebo/results/ardupilot_navigation_scenario_{A,B,C,D}_{log,gz_log,arduplane_log}.txt`
- `tests/gazebo/results/ardupilot_navigation_scenario_{A,B,C,D}_dataflash/00000001.BIN`
- `tests/gazebo/results/ardupilot_navigation_preflight_result.json`
- `tests/gazebo/scripts/test_ardupilot_navigation_validation.py`
- `tests/gazebo/scripts/test_ardupilot_navigation_preflight.py`
- `docs/source_of_truth/controls/ardupilot_navigation_modes.yaml`

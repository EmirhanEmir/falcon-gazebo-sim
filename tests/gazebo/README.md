# FALCON V2 — Gazebo Test Infrastructure

Status: infrastructure/documentation only. **No aircraft physics implementation exists in this repository yet** (no `model.sdf`, no plugin/controller code), so **none of the tests below are implemented or executable** — this file is a registration of planned test scenarios and what each one will check against, for `gazebo-testing` to build out once geometry, aerodynamics, propulsion, and controls implementation exists. Updated 2026-08-21 (post-master-dataset-sync pass) to register 22 additional named scenarios and reconcile them with the prior 19-item list.

**Update, 2026-08-21 (first Gazebo structural implementation pass, `geometry-structure`):** `model/model.sdf` and `model/model.config` now exist for the first time (structural V1: links, joints, mass/CG/inertia, visual/collision geometry — no aerodynamic/propulsion/control plugin). This changes the *executability* of exactly three tests in §1 below — `MODEL_LOAD_TEST`, `GROUND_NO_WIND_TEST`, and `MASS_CG_INERTIA_TEST` can now actually be run against a real file for the first time (a self-check pass by `geometry-structure` already confirms `gz sdf --check` → `Valid.`, correct total mass/CG-consistency/inertia-positive-definiteness/link-graph-connectivity — full detail in `docs/source_of_truth/geometry/GEOMETRY.md` §33.6 — but that self-check is not a substitute for `gazebo-testing` independently running and recording these as formal tests). Every other test below (control-surface sign tests, propulsion tests, aerodynamic/trim/dynamic-mode tests) remains not executable, exactly as before, since no aerodynamic, propulsion, or control-actuation plugin exists yet — this update does not register any new test scenario.

**Update, 2026-08-21 (structural V1 test pass, `gazebo-testing`):** the structural-scope subset of tests is no longer merely planned — it has been implemented and executed against the real `model/model.sdf` in a running Gazebo Sim Harmonic 8.14.0 server (via the `gz-sim` Python `TestFixture` API and, independently, the `gz model`/`gz sdf` CLI tools). 11 tests were run: `MODEL_LOAD_TEST`, `STATIC_GRAVITY_TEST`, `MASS_CG_INERTIA_TEST`, `CONTROL_JOINT_EXISTENCE_TEST`, `CONTROL_JOINT_LIMIT_TEST`, `CONTROL_JOINT_FREE_MOTION_TEST`, `PROP_JOINT_EXISTENCE_TEST`, `PROP_JOINT_AXIS_TEST`, `MESH_PLACEMENT_VISUAL_TEST`, `COLLISION_SANITY_TEST`, `NUMERICAL_STABILITY_IDLE_TEST` — all 11 **PASS**. Full report: `docs/test_results/2026-08-21_structural_v1_test_report.md`. Test worlds: `tests/gazebo/worlds/falcon_v2_freefall_world.sdf`, `tests/gazebo/worlds/falcon_v2_zero_g_world.sdf`. Test scripts and orchestrator: `tests/gazebo/scripts/`. Every other test in this file (control-surface sign tests, propulsion tests, aerodynamic/trim/dynamic-mode tests) remains not executable, exactly as before, since no aerodynamic, propulsion, or control-actuation plugin exists yet.

**Update, 2026-08-22 (structural V1 RE-test pass, `gazebo-testing`, supersedes the 2026-08-21 result):** `validation` raised 2 MAJOR findings against the 2026-08-21 build (hinge secondary/tilt-axis fit not reproducible; unsourced propeller "as viewed from behind" claim). `geometry-structure` corrected both directly in `model/model.sdf` (new deterministic hinge-axis fit changed `<axis><xyz>` and link-origin Z for the 5 control-surface joints by sub-mm to ~1.1 mm; the propeller fix was comment-only, axis/type unaffected). The full 11-test suite was re-run against the corrected file — **all 11 still PASS**, with `MASS_CG_INERTIA_TEST` independently re-confirming mass/CG/inertia were unaffected by the fix (aggregate CG identical to 6 decimal places via a fresh `gz sdf --inertial-stats` run plus an independent runtime ECM query). Full report: `docs/test_results/2026-08-22_structural_v1_retest_hinge_fix.md`. Only one script needed a targeted update, `tests/gazebo/scripts/test_mesh_placement.py` (its hardcoded visual-pose cross-check table now reflects the corrected hinge origins) — every other script queries joint/link state dynamically and required no change. Prior raw evidence archived at `tests/gazebo/results/archive_2026-08-21_pre_hinge_fix/` rather than silently overwritten.

**Update, 2026-08-22 (`AERODYNAMICS_V1_IMPLEMENTATION` pass, `aerodynamics`):** an aerodynamic force/moment System plugin now exists (`plugins/aerodynamics/`, built target `FalconV2Aerodynamics`, attached to `model/model.sdf` via an additive `<plugin>` block — no structural element modified) and is loadable in a live Gazebo instance (`export GZ_SIM_SYSTEM_PLUGIN_PATH=.../plugins/aerodynamics/build:$GZ_SIM_SYSTEM_PLUGIN_PATH` before launching `gz sim`; see `plugins/aerodynamics/README.md`). This changes the executability of §4/§5/§6 below (`ZERO_AIRSPEED_AERO_TEST`, `AOA_SIGN_TEST`, `SIDESLIP_SIGN_TEST`, and the sign-behavior tests) for the first time — they can now be run against real ECM/link/joint state, not just documentation. `aerodynamics` performed (a) a standalone, Gazebo-independent self-test of the pure-math core (`plugins/aerodynamics/build/aero_model_selftest`, 15/16 checks PASS, 1 honestly-reported FAIL) and (b) a live-Gazebo smoke test (plugin loads, configures, runs 3000 physics steps against `tests/gazebo/worlds/falcon_v2_freefall_world.sdf` with no crash/NaN, diagnostics topic confirmed publishing physically sane values) — **neither is a substitute for `gazebo-testing` independently building, running, and recording the formal test suite below.** Full derivation/self-test log: `docs/source_of_truth/aerodynamics/AERODYNAMICS.md` §19.

**Priority flag for `gazebo-testing` (superseded by the 2026-08-22 fix-pass update below — kept as the historical record of the original ask):** `Cma_RESTORING_SIGN_TEST` should be run **first** among the sign tests below. `aerodynamics` found, via a rigorous first-principles FLU pitch-axis analysis (confirmed by both self-tests above), that the literal task-specified `My=qbar·S·c_ref·Cm` formula produces **destabilizing** rather than restoring pitch behavior — a specific, actionable, already-diagnosed finding (`AERODYNAMICS.md` §19.7), not a generic unresolved-sign placeholder. Do not tune/adjust `Cma` or any coefficient to make this pass (`CLAUDE.md` simulation tuning policy) — if confirmed, the fix belongs in the `Cm`-to-`My` axis mapping, and is `aerodynamics`'/`validation`'s decision, not `gazebo-testing`'s.

Diagnostics topic for live inspection while testing: `/model/falcon_v2/aerodynamics/diagnostics` (`gz.msgs.Double_V`, field order `V, alpha_rad, beta_rad, qbar, CL, CD, CY, Cl, Cm, Cn`, published at 20 Hz by default, configurable via the `<diagnostics_rate_hz>` SDF plugin parameter). A wind-input hook exists (`/model/falcon_v2/wind`, `gz.msgs.Vector3d`, world frame) for future use — no publisher exists yet, so wind is always zero in the current build.

**Update, 2026-08-22 (`AERODYNAMICS_V1` live-Gazebo test pass, `gazebo-testing`):** all 16 mandatory aero tests from §4 below (`ZERO_AIRSPEED_AERO_TEST`, `AOA_SIGN_TEST`, `SIDESLIP_SIGN_TEST`, plus `LIFT_SIGN_TEST`/`DRAG_SIGN_TEST`/`Cma_RESTORING_SIGN_TEST`/`Cmq`/`Clp`/`Cnr_DAMPING_SIGN_TEST`/`Cnb_STATIC_STABILITY_SIGN_TEST`/`AILERON_ROLL_SIGN_TEST`/`RUDDER_YAW_SIGN_TEST`/`ELEVATOR_PITCH_SIGN_TEST`/`RATE_NORMALIZATION_TEST`/`DRAG_POLAR_TEST`/`HIGH_ALPHA_LIMITER_TEST`, not previously itemized as separate rows in this file's planned-test tables but named explicitly in this pass's task) were implemented and run against the real `FalconV2Aerodynamics` plugin in a live `gz-sim8` server. **Result: 15/16 PASS, 1 `TEST_FAILED` — `Cma_RESTORING_SIGN_TEST` independently confirms, via real applied-wrench measurement (not just `aerodynamics`' own pure-math self-test), that the pitch axis is destabilizing rather than restoring** — see `docs/test_results/2026-08-22_aerodynamics_v1_test_report.md` for the full writeup, exact numeric evidence, and the two test-methodology findings made along the way (Cmd-based velocity override is unusable for release-and-measure testing in this environment; holding a sustained body rate excites the undamped control-surface/prop joints and must be worked around). Test scripts: `tests/gazebo/scripts/{aero_lib.py, test_aero_kinematic_signs.py, test_aero_stability_derivatives.py, test_aero_control_surfaces.py, run_all_aero_tests.sh}`. Reuses the existing `falcon_v2_zero_g_world.sdf` unmodified (no new world file needed — the plugin is already embedded in `model/model.sdf`). `TRIM_TEST`/`STRAIGHT_LEVEL_FLIGHT_TEST`/dynamic-mode tests remain out of scope for this pass and blocked on the `Cma` fix.

**Update, 2026-08-22 (same day, `AERODYNAMICS_V1` Cm-to-My fix re-test, `gazebo-testing`):** `aerodynamics` applied a scoped Cm-to-My sign correction (negate only the static `Cm0+Cma*alpha+Cmde*deltaE` group when computing applied `My`, leaving the rate group `Cmq*qHat` untouched) plus a previously-missing `CLq*qHat` term in `CL`, both code-only (no `aero_v1_config.yaml` coefficient changed). The full 16-test suite was re-run against the rebuilt plugin, reusing the same scripts/methodology (only the independent Python replica in `aero_lib.py` and the `HIGH_ALPHA_LIMITER_TEST` cross-check were updated to mirror the new formula). **Result: 16/16 PASS — `Cma_RESTORING_SIGN_TEST` now confirms a restoring pitch moment live, closing out the prior `TEST_FAILED`.** `ELEVATOR_PITCH_SIGN_TEST`'s measured sign flipped as `aerodynamics` anticipated (documented, not a regression); every other test is numerically unchanged from the first run, confirming the fix's scope was as claimed. Full writeup: `docs/test_results/2026-08-22_aerodynamics_v1_retest_cma_fix.md` (supersedes `docs/test_results/2026-08-22_aerodynamics_v1_test_report.md`, which remains as the historical record of the original finding and methodology development). Ready for `validation`'s final review.

**Update, 2026-08-22 (`AERODYNAMICS_V1_IMPLEMENTATION` fix sub-pass, `aerodynamics`):** following `gazebo-testing`'s live confirmation above and `validation`'s root-cause review, two fixes were applied to `plugins/aerodynamics/AeroModel.hh`'s `ComputeAero()` (no `aero_v1_config.yaml` coefficient value changed): (1) a precisely-scoped `Cm`-to-`My` sign correction — only the static/angle-derived group (`Cm0 + Cma·alpha + Cmde·delta_e`) is negated when computing `My`; the self-referential rate group (`Cmq·q_hat`) is left untouched (flipping it would have broken the already-correct `Cmq_DAMPING_SIGN_TEST`); (2) the previously-loaded-but-unused `CLq·q_hat` term is now included in the `CL` build-up (MAJOR finding). Full derivation: `AERODYNAMICS.md` §19.12. Plugin rebuilt clean; standalone self-test now **17 PASS, 0 FAIL** (up from 15/1); live-Gazebo smoke-load re-verified (no crash/NaN). **Consequence to expect, not a regression:** `Cmde` moved into the negated group, so `ELEVATOR_PITCH_SIGN_TEST`'s measured `My` sign will flip relative to the 2026-08-22 test-report result above. **Requested from `gazebo-testing`:** re-run at minimum `Cma_RESTORING_SIGN_TEST`, `Cmq_DAMPING_SIGN_TEST` (regression check), `ELEVATOR_PITCH_SIGN_TEST` (expected flip), and ideally `ZERO_AIRSPEED_AERO_TEST`/`DRAG_POLAR_TEST`/`HIGH_ALPHA_LIMITER_TEST` (since `CL` changed) against the rebuilt plugin. `validation` re-review follows before `AERODYNAMICS_V1_READY`.

**Update, 2026-08-22 (`CONTROL_SURFACE_SIGN_MAPPING` test pass, `gazebo-testing`):** independently verified `controls-integration`'s geometric TE-direction/aero-moment-sign determination for the 5 control-surface joints via 9 live-Gazebo tests: `LEFT_AILERON_JOINT_SIGN_TEST`, `RIGHT_AILERON_JOINT_SIGN_TEST`, `AILERON_DIFFERENTIAL_SIGN_TEST`, `AILERON_COMMON_MODE_REJECTION_TEST`, `LEFT_ELEVATOR_JOINT_SIGN_TEST`, `RIGHT_ELEVATOR_JOINT_SIGN_TEST`, `RUDDER_JOINT_SIGN_TEST`, `AILERON_AERO_MOMENT_SIGN_TEST`, `RUDDER_AERO_MOMENT_SIGN_TEST`. **Result: 9/9 PASS, 9/9 CONFIRMS-HYPOTHESIS** — both aileron joints and both elevator joints independently, quantitatively confirmed same-sign (not mirrored, TE-up for +angle on both sides in each pair); rudder confirmed TE-right(−Y) for +angle; `aileron_sign=+1` and `rudder_sign=+1` (unmodified `aero_v1_config.yaml`) both independently confirmed correct via real measured applied moment (Mx=+4.87685 N·m for the differential aileron case, exactly Mx=0 for the common-mode case, Mz=−0.44639 N·m for the rudder case). No REFUTES-HYPOTHESIS result. `ELEVATOR_SYMMETRIC_MAPPING_TEST`/`ELEVATOR_AERO_MOMENT_SIGN_TEST` explicitly deferred, pending `elevator_sign` config fix (`controls-integration`'s coordination item with `aerodynamics`, not executed this pass). Full report: `docs/test_results/2026-08-22_control_surface_sign_mapping_test_report.md`. New script: `tests/gazebo/scripts/test_control_surface_sign_mapping.py` (reuses `aero_lib.py`'s existing helpers, no new technique). No aircraft physics parameter modified.

**Update, 2026-08-22 (same day, `CONTROL_SURFACE_SIGN_MAPPING` follow-up, `gazebo-testing`):** after `aerodynamics` applied the `elevator_sign` fix (`+1.0 -> -1.0` in `aero_v1_config.yaml`), ran the 2 deferred tests: `ELEVATOR_SYMMETRIC_MAPPING_TEST` and `ELEVATOR_AERO_MOMENT_SIGN_TEST` — **both PASS/CONFIRMS-HYPOTHESIS**. Same physical TE-up command (`left_elevator_joint=right_elevator_joint=+8°`) now measures My=-1.48560 N·m (nose-up), flipped from the pre-fix +1.128 N·m (nose-down) exactly as anticipated, and matching the textbook TE-up⇒nose-up expectation for the first time. Optional regression re-check (`test_aero_stability_derivatives.py`, unmodified) confirms `Cma_RESTORING_SIGN_TEST`/`Cmq_DAMPING_SIGN_TEST`/others still PASS, unaffected by the elevator-only config change. **Combined result across both `CONTROL_SURFACE_SIGN_MAPPING` passes: 11/11 CONFIRMS-HYPOTHESIS, 0 REFUTES-HYPOTHESIS.** Full writeup appended to `docs/test_results/2026-08-22_control_surface_sign_mapping_test_report.md` (see "Supplement" section). New script: `tests/gazebo/scripts/test_elevator_sign_mapping.py`. No aircraft physics parameter modified. Ready for `validation`'s final review.

**Update, 2026-08-23 (`TRIM_AND_DYNAMIC_AERO_VALIDATION`, `gazebo-testing`):** live-Gazebo reproduction of `aerodynamics`' desk-computed neutral trim point (Benchmark A, V=21.244 m/s) and the 18 m/s/elevator=-8° trim point (Benchmark B), plus a magnitude/direction sanity check of the 5 dynamic-derivative runtime responses (Cmq/Clp/Cnr damping, Cma restoring, Cnb directional stability). **Result: Benchmark A PASS, Benchmark B PASS vs the V1 model's own desk-computed values (CL/CD/Cm/My all matched to ~1e-6–1e-7), Benchmark B vs the XFLR5/6kg-level-flight reference is KNOWN_V1_LIMITATION (not PASS/FAIL, ~+3.2% CL/+3.6% drag — already root-caused by `aerodynamics` as the expected consequence of the V1 model's omitted `CL_delta_e` term), all 5 dynamic-derivative checks PASS (correct sign, live-measured magnitude within a factor of ~1.4 of the task's quoted example once scaled to a common V/rate condition).** New controller technique (`test_trim_benchmarks.py`, not added to `aero_lib.py`): feedforward + light-P holding, needed because `aero_lib.hold_step()`'s pure-P control has a non-decaying steady-state error against a persistent ~59 N aero force (empirically ~0.06 m/s residual at the standard kp_lin=150, ~300x too large for this task's ±1e-4 CL tolerance) — the feedforward term is computed once from `aero_lib.compute_aero()` at the exact target state and applied through the same `add_world_force`/`add_world_wrench` primitives, so the quantity actually measured (the live plugin's own output) is unaffected. New scripts: `tests/gazebo/scripts/test_trim_benchmarks.py`, `tests/gazebo/scripts/test_dynamic_derivative_response.py` (the latter cites `aero_stability_derivatives_result.json`'s existing live measurements rather than relaunching Gazebo, per task instruction, since that file already had quantitative moment values for all 5 checks). No aircraft physics parameter modified. Full report: `docs/test_results/2026-08-23_trim_and_dynamic_aero_validation_report.md`.

## Directory layout

```
tests/
  gazebo/       # Gazebo Sim Harmonic world / model-load / runtime tests (this directory)
  physics/      # lower-level physics checks (gravity, inertia, numerical stability)
  regression/   # regression suite run after any implementation change
```

Test results are saved to `docs/test_results/`.

## Planned test scenarios

Owned by `gazebo-testing`, independently reviewed by `validation`. **None are implemented yet — this is registration/documentation only, no test was run to produce this table.** Each row cites the specific source-of-truth document/section the test will check its result against once an implementation exists; `gazebo-testing` must never adjust an aircraft physics parameter to make any of these pass (`CLAUDE.md` hard constraint).

### 1. Foundational / load / static

| Test | Validates | Source-of-truth citation |
|---|---|---|
| `MODEL_LOAD_TEST` | FALCON V2 SDF loads into Gazebo Sim Harmonic with no parse/link/joint/mesh-resolution errors. | `docs/source_of_truth/geometry/GEOMETRY.md` §4.2 (12-mesh inventory), §26.3 (movable-link/fixed-structure split); no SDF exists yet — this is a load-time structural check, not a coefficient check. |
| `GROUND_NO_WIND_TEST` | At rest on the ground, throttle=0, zero wind: no spurious motion, no NaN/divergence, model settles under gravity. | Master dataset §68 item 1 ("Ground/no-wind, throttle=0"). |
| `MASS_CG_INERTIA_TEST` (folds prior `STATIC_GRAVITY_TEST` + `CG_BALANCE_TEST`) | SDF `<inertial>` mass=6.000 kg, CG=(0.168309, 0, 0.100000) m, and the V1 inertia tensor produce the expected static balance and free-rotation response (no unexpected pitch/roll/yaw drift at rest). | `docs/source_of_truth/mass_properties/MASS_PROPERTIES.md` §1, §2, §3.1, §3.4 (CG), §5.1 (`V1_PROVISIONAL` inertia tensor, incl. the flagged Ixz-sign-vs-SDF prerequisite). |

### 2. Control-surface sign / direction

| Test | Validates | Source-of-truth citation |
|---|---|---|
| `CONTROL_SURFACE_DIRECTION_TEST` (umbrella) | Each of the 5 control-surface joints and 2 prop joints, commanded in isolation, produces the expected physical motion before any downstream aero/prop force is trusted. General cross-check preceding `ROLL_RESPONSE_TEST`/`PITCH_RESPONSE_TEST`/`YAW_RESPONSE_TEST`. | `docs/source_of_truth/controls/CONTROLS.md` §4 (governing rule: "XFLR5 control sign ile Gazebo joint sign aynı varsayılmamalı"), §4.4. |
| `AILERON_TEST` | Resolves `AILERON_SIGN_TEST_REQUIRED` (left and right independently) — commanded left/right aileron deflection produces the correct physical up/down direction and roll-moment sign consistent with `Clδa≈+0.308/rad`. | `CONTROLS.md` §4.3, §7 item 4; `AERODYNAMICS.md` §7.3, §10. |
| `ELEVATOR_TEST` | Resolves `ELEVATOR_SIGN_TEST_REQUIRED` — commanded elevator deflection produces the correct TE-down(+)/TE-up(−) direction (XFLR5 convention) and pitching-moment sign consistent with `Cmδe≈-0.73/rad`. | `CONTROLS.md` §4.1, §7 item 4; `AERODYNAMICS.md` §7.1, §10. |
| `RUDDER_TEST` | Resolves `RUDDER_SIGN_TEST_REQUIRED` — commanded rudder deflection produces the correct physical left/right direction (not stated by XFLR5 itself) and yaw-moment sign consistent with `Cnδr≈-0.025/rad`. | `CONTROLS.md` §4.2, §7 item 4; `AERODYNAMICS.md` §7.2, §10. |

### 3. Propulsion

| Test | Validates | Source-of-truth citation |
|---|---|---|
| `PROP_SPINUP_TEST` | Throttle step produces RPM rising as a solved state through `I_rotor·dω/dt = Q_motor − Q_prop` — never set instantaneously. | Master dataset §68 item 2; `docs/source_of_truth/propulsion/PROPULSION.md` §2, §4. |
| `STATIC_THRUST_RPM_TEST` | At zero airspeed, per-motor thrust-vs-RPM falls in the validated reference band (SunnySky bench ≈9230 RPM→≈32.85 N; APC-official interpolation ≈31.32 N at the same RPM). | Master dataset §68 item 3; `PROPULSION.md` §1.6. |
| `STATIC_POWER_CURRENT_TEST` | Static current (≈63.2 A) and power (≈935 W/motor) plausibility at the bench reference RPM. | Master dataset §68 item 4; `PROPULSION.md` §1.6. |
| `LEFT_MOTOR_TEST` / `RIGHT_MOTOR_TEST` | Each motor's RPM/thrust/torque is computed independently at its own hub and local airspeed, not lumped into one combined value. | `PROPULSION.md` §6 ("each motor's thrust and torque must be computed independently"). |
| `REACTION_TORQUE_TEST` | Propeller shaft reaction torque `Q_reaction = −Q_prop` is applied to the airframe at each real hub location; sign verified against Gazebo FLU convention. | Master dataset §68 item 5, §72 ("reaction torque mutlaka uygulanmalı"); `PROPULSION.md` §7, §7.2 (pitch-moment-sign flag). |
| `COUNTER_ROTATION_CANCELLATION_TEST` (folds prior `SYMMETRIC_PROPULSION_TEST`) | Equal throttle on both motors (left CCW / right CW) yields a net reaction moment ≈0 at the airframe. | Master dataset §68 item 6; `PROPULSION.md` §1.4, §7. |
| `DIFFERENTIAL_THRUST_YAW_TEST` | Asymmetric throttle produces `Mz ≈ 0.300 × (T_right − T_left)` arising from real `r×F` at the hub positions — no coded yaw-control derivative. | Master dataset §68 item 7; `PROPULSION.md` §7.1. |
| `ENGINE_OUT_TEST` | Stopping one motor produces the expected residual yaw/roll moment and continued (non-diverging) flight response. | Master dataset §68 item 8. |

### 4. Aerodynamic sign & zero-airspeed behavior

| Test | Validates | Source-of-truth citation |
|---|---|---|
| `ZERO_AIRSPEED_AERO_TEST` | At V=0, the aerodynamic force/moment model does not diverge/NaN (`qbar=0`, `J`/advance-ratio edge cases handled). | `docs/source_of_truth/aerodynamics/AERODYNAMICS.md` §8.1, §8.4. |
| `AOA_SIGN_TEST` | `alpha = atan2(w,u)` produces the correct sign for FALCON V2's FLU body frame (flagged risk: the stated formula is a standard FRD form; FLU flips the Z axis relative to FRD). | `AERODYNAMICS.md` §8.1, §10, §13 item 10. |
| `SIDESLIP_SIGN_TEST` | `beta = asin(v/V)` produces the correct sign for FLU (FLU flips the Y axis relative to FRD). | `AERODYNAMICS.md` §8.1, §10, §13 item 10. |

### 5. Trim & straight-level flight

| Test | Validates | Source-of-truth citation |
|---|---|---|
| `TRIM_TEST` (umbrella) | The implemented aircraft can find and hold *a* trimmed condition at all, before checking it against a specific numeric benchmark. | `AERODYNAMICS.md` §6.7, §6.8 (`VALIDATION_TARGET`s). |
| `18MPS_LEVEL_TRIM_TEST` (folds master-dataset benchmark items 9–10) | Trim at V≈18 m/s: `CLreq≈0.657`, total drag/required thrust≈5.19 N (≈2.595 N/motor), elevator≈−8°. | Master dataset §68 items 9–10, §69; `AERODYNAMICS.md` §6.7, §6.8. |
| `NEUTRAL_ELEVATOR_21P24_TEST` | Trim at neutral (0°) elevator: V≈21.244 m/s, alpha≈0.364°, CL≈0.4717 — the project's original single full-aircraft reference point. | Master dataset §68 item 11, §69; `AERODYNAMICS.md` §6.1, §6.8; `CLAUDE.md`. |
| `STRAIGHT_LEVEL_FLIGHT_TEST` | Sustained straight-and-level flight *holds* trim over time (dynamic persistence), not just a one-shot static solve. | `AERODYNAMICS.md` §6.7 (`VALIDATION_TARGET`, "primary future straight-and-level Gazebo benchmark"). |

### 6. Control response (rate-domain)

| Test | Validates | Source-of-truth citation |
|---|---|---|
| `AILERON_ROLL_TEST` | Commanded aileron deflection produces a roll-rate response consistent with `Clδa`/`Clp`, run only after `AILERON_TEST` confirms direction. | Master dataset §68 item 12; `AERODYNAMICS.md` §6.2, §7.3. |
| `RUDDER_YAW_TEST` | Commanded rudder deflection produces a yaw response consistent with `Cnδr`/`Cnr`/`Cnβ`, run only after `RUDDER_TEST` confirms direction. | Master dataset §68 item 13; `AERODYNAMICS.md` §7.2. |
| `ROLL_RESPONSE_TEST` | Full roll-rate step/settle response cross-checked against the roll-subsidence mode (λ≈−9.464 1/s, τ≈0.106 s). | `AERODYNAMICS.md` §6.4, §6.8; `CONTROLS.md` §4.4. |
| `PITCH_RESPONSE_TEST` | Elevator-step → pitch-rate response cross-checked against `Cmq` and the short-period mode. | `AERODYNAMICS.md` §6.2, §6.4. |
| `YAW_RESPONSE_TEST` | Rudder/differential-thrust step → yaw-rate response cross-checked against `Cnr` and the Dutch-roll mode. | `AERODYNAMICS.md` §6.4; `CONTROLS.md` §4.4. |

### 7. Dynamic modes, stall approach, manufacturer comparison

| Test | Validates | Source-of-truth citation |
|---|---|---|
| `DUTCH_ROLL_TEST` | Lateral disturbance excites Dutch roll at fn≈0.512 Hz, ζ≈0.095, period≈1.96 s. | Master dataset §68 item 14, §69; `AERODYNAMICS.md` §6.4, §6.8. |
| `ROLL_SUBSIDENCE_TEST` | Roll mode λ≈−9.464 1/s, τ≈0.106 s. | Master dataset §68 item 15, §69; `AERODYNAMICS.md` §6.4, §6.8. |
| `SPIRAL_MODE_TEST` | Mildly unstable spiral mode λ≈+0.08227 1/s, doubling time≈8.43 s — expected/reported behavior, **not a defect to "fix" by adjusting a derivative**. | Master dataset §68 item 16, §69; `AERODYNAMICS.md` §6.4 (explicit non-tuning instruction). |
| `SHORT_PERIOD_TEST` | Short-period mode ζ≈0.394. | Master dataset §68 item 17, §69; `AERODYNAMICS.md` §6.4, §6.8. |
| `PHUGOID_TEST` | Phugoid mode ζ≈0.003 (stable but very lightly damped). | Master dataset §68 item 18, §69; `AERODYNAMICS.md` §6.4, §6.8. |
| `STALL_APPROACH_TEST` | Behavior approaching stall (alpha→≈9–9.5°, V→≈Vstall≈12.24 m/s) is characterized against the documented V1 linear-model reliability limit — **not** validated against any stall/post-stall model, since none is chosen yet (explicitly deferred, `AERODYNAMICS.md` §11). | Master dataset §68 item 19; `AERODYNAMICS.md` §5.2, §11. |
| `MANUFACTURER_6KG_CURVE_COMPARISON` | Simulated level-flight drag/thrust/RPM/throttle vs. airspeed (12.5–25 m/s) compared against the `V1_CALIBRATED`/`V1_VALIDATION_ESTIMATES` tables. | Master dataset §68 item 20, §69; `AERODYNAMICS.md` §6.6; `PROPULSION.md` §6.1. |

### 8. Numerical stability (cross-cutting)

| Test | Validates | Source-of-truth citation |
|---|---|---|
| `NUMERICAL_STABILITY_TEST` | No NaN/divergence/energy-gain artifact across the integration timestep, for any of the scenarios above (ground, spin-up, trim, dynamic-mode excitation). | `CLAUDE.md` simulation-tuning-policy checklist items 11–12 (integration timestep, numerical stability). |

## Reconciliation with the prior 19-item list

The prior planned-test list (unchanged, still recorded verbatim in `docs/architecture/AGENT_WORKFLOW.md` "Future test categories") is not duplicated above — it is folded in as follows, so nothing is silently dropped:

| Prior list item | Disposition in this pass |
|---|---|
| `MODEL_LOAD_TEST` | Kept as-is (§1). |
| `STATIC_GRAVITY_TEST`, `CG_BALANCE_TEST` | Folded into `MASS_CG_INERTIA_TEST` (§1) — same underlying check (mass/CG/inertia static behavior), one combined name. |
| `CONTROL_SURFACE_DIRECTION_TEST` | Kept as the umbrella for `AILERON_TEST`/`ELEVATOR_TEST`/`RUDDER_TEST` (§2). |
| `AILERON_TEST`, `ELEVATOR_TEST`, `RUDDER_TEST` | Kept, now explicitly tied to the `AILERON_SIGN_TEST_REQUIRED`/`ELEVATOR_SIGN_TEST_REQUIRED`/`RUDDER_SIGN_TEST_REQUIRED` tags from `CONTROLS.md` §4 (§2). |
| `LEFT_MOTOR_TEST`, `RIGHT_MOTOR_TEST` | Kept, now cited against `PROPULSION.md` §6's per-motor-independence rule (§3). |
| `SYMMETRIC_PROPULSION_TEST` | Folded into `COUNTER_ROTATION_CANCELLATION_TEST` (§3) — same check (equal-throttle reaction-torque cancellation), master-dataset-aligned name. |
| `ZERO_AIRSPEED_AERO_TEST` | Kept as-is (§4). |
| `AOA_SIGN_TEST`, `SIDESLIP_SIGN_TEST` | Kept as-is, now cited against the explicit FRD-vs-FLU risk flagged in `AERODYNAMICS.md` §8.1/§10 (§4). |
| `TRIM_TEST` | Kept as the umbrella over the two concrete numeric benchmarks `18MPS_LEVEL_TRIM_TEST` and `NEUTRAL_ELEVATOR_21P24_TEST` (§5). |
| `STRAIGHT_LEVEL_FLIGHT_TEST` | Kept, distinguished from `18MPS_LEVEL_TRIM_TEST` as the dynamic-persistence check vs. the static-benchmark-match check (§5). |
| `ROLL_RESPONSE_TEST`, `PITCH_RESPONSE_TEST`, `YAW_RESPONSE_TEST` | Kept as-is, now explicitly sequenced after the corresponding sign test (§6). |
| `ENGINE_OUT_TEST` | Kept as-is (§3), matches master dataset §68 item 8 exactly. |
| `NUMERICAL_STABILITY_TEST` | Kept as-is, reframed as cross-cutting rather than a single scenario (§8). |

No prior-list item was deleted; every one is either kept verbatim or explicitly folded into a master-dataset-aligned name with the mapping stated above.

## Rules

- `gazebo-testing` may create test worlds, test scripts, launch scripts, automated regression tests, and result reports here.
- `gazebo-testing` must never change aircraft physics parameters (mass, CG, inertia, aerodynamic coefficients, control authority, motor thrust) to make a test pass.
- A failing test is reported as `TEST_FAILED` with: observed behavior, expected behavior, evidence/logs, suspected subsystem, and the responsible specialist agent (`geometry-structure`, `aerodynamics`, `propulsion`, or `controls-integration`).
- `validation` reviews test results independently of `gazebo-testing` — the two roles are never merged.

## Current phase

No test scenarios are implemented yet, since no aircraft physics implementation exists in this repository. This file is registration/documentation only — no Gazebo instance was launched and no test was executed to produce it. See `docs/architecture/AGENT_WORKFLOW.md` for the full agent workflow this test infrastructure supports, and `docs/architecture/GAZEBO_READINESS.md` for the current cross-domain data-readiness assessment.

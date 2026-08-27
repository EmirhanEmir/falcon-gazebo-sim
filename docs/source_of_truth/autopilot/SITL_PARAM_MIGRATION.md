# FALCON V2 — Real .param → ArduPlane SITL Migration

**Owner:** `controls-integration`
**Task:** `SENSOR_MODEL_AND_ARDUPLANE_SITL_PREPARATION` (2026-08-27).

This document classifies every parameter GROUP present in the real aircraft's captured configuration, `docs/source_of_truth/autopilot/real_aircraft/yeni_pixhawk.param` (1105 lines, `NAME,VALUE` CSV — **never modified**, read-only source), into one of: `KEEP_FOR_SITL`, `REVIEW_AND_ADAPT`, or `DROP_OR_DO_NOT_IMPORT`. Groups are prefix-based, not itemized line-by-line (1105 params). The resulting SITL-safe profile is `config/ardupilot/falcon_v2_sitl.parm` — it contains **only** the subset explicitly justified below, never a wholesale copy.

**Provenance tags used below** (per task instruction): `MANUFACTURER_MANUAL`, `USER_CONFIRMED`, `REAL_PARAM_CONFIGURATION`, `GAZEBO_VALIDATION`, `SITL_REQUIRED`, `V1_PROVISIONAL`, `OTHER_AIRCRAFT_TUNING_DO_NOT_IMPORT`, `STALE_REAL_AIRCRAFT_CONFIG`.

Current ArduPlane version referenced throughout: 4.8.0-dev, installed at `/home/emirhan/gazebo_sim/ardupilot`. Every parameter name below was confirmed present/current in that installed source (`ArduPlane/Parameters.cpp`, `libraries/AP_Airspeed/*.cpp`, etc.) — not assumed from memory.

---

## 1. Roll/Pitch rate controller: `RLL_RATE_*` / `PTCH_RATE_*` / `RLL2SRV_*` / `PTCH2SRV_*`

**Real .param values (CONFIRMED PRESENT):**
```
RLL_RATE_P=0.08  RLL_RATE_I=0.15  RLL_RATE_FF=0.345  RLL_RATE_D=0  RLL_RATE_IMAX=0.666  RLL_RATE_SMAX=150  RLL2SRV_TCONST=0.5  RLL2SRV_RMAX=0
PTCH_RATE_P=0.04 PTCH_RATE_I=0.15 PTCH_RATE_FF=0.345 PTCH_RATE_D=0  PTCH_RATE_IMAX=0.666 PTCH_RATE_SMAX=150 PTCH2SRV_TCONST=0.5 PTCH2SRV_RLL=1
AUTOTUNE_LEVEL=6  AUTOTUNE_AXES=7
```

**Classification: the GAIN terms (`RLL_RATE_P/I/D/FF`, `PTCH_RATE_P/I/D/FF`) and `AUTOTUNE_LEVEL` are `OTHER_AIRCRAFT_TUNING_DO_NOT_IMPORT`.** Per the user-confirmed facts governing this task: these values do **not** match the Falcon V2 manufacturer manual's own stated initial PID (`ROLL: P=0.25 I=0.125 D=0.002 FF=0.125`, `PITCH: P=0.25 I=0.125 D=0.002 FF=0.125`, `AUTOTUNE_LEVEL=8`), and this mismatch — together with the real .param's `AUTOTUNE_LEVEL=6` also disagreeing with the manufacturer's recommended 8 — is treated as **decisive corroboration** (not a coincidence to re-litigate) that this specific file's rate-controller tuning reflects a different airframe or a stale/inherited configuration, not Falcon V2 truth.

**Substitution used in `falcon_v2_sitl.parm`:** the Falcon V2 manufacturer manual's stated initial gains, classified `FALCON_V2_MANUFACTURER_INITIAL_PID` / `MANUFACTURER_INITIAL_PID_RECOMMENDATION` (given directly by the project owner for this task; treated as `MANUFACTURER_MANUAL`-sourced per the same convention CLAUDE.md uses for its own owner-supplied constants). **Mapping is DIRECT, current, and unambiguous** — `RLL_RATE_P/I/D/FF` and `PTCH_RATE_P/I/D/FF` are confirmed to be the CURRENT ArduPlane parameter names (present in the real .param itself, which is current-version-generated) — no legacy-name conversion needed, no `PID_MAPPING_REVIEW_REQUIRED` flag warranted for the mapping itself.
- `RLL_RATE_P=0.25, RLL_RATE_I=0.125, RLL_RATE_D=0.002, RLL_RATE_FF=0.125`
- `PTCH_RATE_P=0.25, PTCH_RATE_I=0.125, PTCH_RATE_D=0.002, PTCH_RATE_FF=0.125`
- `AUTOTUNE_LEVEL=8` (comment: `MANUFACTURER_RECOMMENDED_AUTOTUNE_LEVEL` — **not executed this stage**, autotune is never run by this task)

**Never labeled `FINAL_TUNED_PID` or `FLIGHT_VALIDATED_FINAL_PID`** anywhere in this repository — these are initial-setup recommendations only.

**`RLL_RATE_IMAX/SMAX`, `RLL2SRV_TCONST/RMAX`, `PTCH_RATE_IMAX/SMAX`, `PTCH2SRV_TCONST/RLL`, `AUTOTUNE_AXES`: `REVIEW_AND_ADAPT`, NOT silently carried over as if manufacturer-sourced.** These are filter/slew/anti-windup/rate-limit terms, not one of the manufacturer's 4 stated gains — they are not imported into `falcon_v2_sitl.parm` this stage (ArduPlane's own defaults apply). A future dedicated tuning stage should re-derive them against Falcon-specific data if needed, not reuse the other aircraft's values.

**`STEER2SRV_*` / `YAW2SRV_*`:** same `OTHER_AIRCRAFT_TUNING_DO_NOT_IMPORT` classification, same reasoning (ground-steering/yaw-damper gains, not manufacturer-sourced, not imported).

---

## 2. Servo function mapping: `SERVO*`

**Real .param (CONFIRMED, cross-checked against installed ArduPlane 4.8.0-dev `libraries/SRV_Channel/SRV_Channel.h` enum values):**

| Channel | FUNCTION | Meaning | MIN | MAX | TRIM | REVERSED |
|---|---|---|---|---|---|---|
| SERVO1 | 4 | Aileron | 800 | 2200 | 1500 | 1 |
| SERVO2 | 19 | Elevator | 800 | 2200 | 1500 | 0 |
| SERVO3 | 73 | ThrottleLeft | 1000 | 2000 | 1000 | 0 |
| SERVO4 | 21 | Rudder | 800 | 2200 | 1500 | 1 |
| SERVO5 | 74 | ThrottleRight | 1000 | 2000 | 1000 | 0 |
| SERVO6–16 | 0 or -1 | unused/disabled | — | — | — | — |

**Classification: `KEEP_FOR_SITL` / `REAL_PARAM_CONFIGURATION`.** This is real, structural configuration directly attributable to Falcon V2 — not a tuning value. Confirms: exactly **one** logical Aileron output, **one** logical Elevator output, **one** Rudder, and **independent** ThrottleLeft/ThrottleRight — matching this project's existing twin-motor Gazebo physics and 5-surface actuator model (`left_aileron`/`right_aileron`/`left_elevator`/`right_elevator`/`rudder` joints in `model/model.sdf`, `plugins/actuators/ActuatorSystem.cc` + `docs/source_of_truth/controls/actuator_v1_config.yaml`). This also directly corroborates the user-confirmed "one logical Aileron output Y-split to L/R aileron servos, one logical Elevator output Y-split to L/R elevator servos" fact (`PROVISIONAL_WIRING_ASSUMPTION` — corroborating, still kept provisional per the user's own instruction, see sec 7).

**Channel numbering choice for `falcon_v2_sitl.parm`:** the exact same channel numbers 1–5 are kept (not restated onto different channels) — there is no reason to renumber real, structurally-confirmed hardware wiring, and keeping the same numbers minimizes the chance of a future transcription error when this profile is eventually compared against the real aircraft's own file.

**MIN/MAX/TRIM/REVERSED values: kept as-is (`KEEP_FOR_SITL`), with an explicit caveat.** These define the real servo/linkage PWM travel range and center — real, sourced hardware data, not a stale/inherited tuning value like sec 1's rate gains. They are reused structurally in `falcon_v2_sitl.parm`. **Caveat, not resolved this stage:** exactly how ArduPilot's PWM-domain `SERVOx_MIN/MAX/TRIM/REVERSED` normalization interacts with this project's own Gazebo `ActuatorSystem.cc`, which consumes physical-angle-**radians** commands (not PWM) on `/model/falcon_v2/actuators/*/cmd_rad` — is a **future ArduPilotPlugin JSON-bridge-stage** question (task scope item 7 explicitly excludes wiring this stage). See sec 6 for the full mapping-chain statement and the explicit confirmation that `SERVOx_REVERSED` is not being double-applied against the already-validated Gazebo sign mapping.

---

## 3. TECS / NAVL1 (speed/height and navigation controllers)

**Real .param:** `TECS_*` (33 params — climb/sink rates, pitch limits, speed weighting, damping/time-constant terms, land-phase behavior) and `NAVL1_*` (4 params — L1 period/damping/bank-limit/crosstrack-integrator).

**Classification: `REVIEW_AND_ADAPT`, not imported this stage.** None of these are manufacturer-sourced for Falcon V2 (not present anywhere in the master dataset's captured manual excerpt), and — same reasoning as sec 1 — this real .param file's tuning-related fields are demonstrably not authoritative Falcon V2 truth. Some individual values (e.g. `TECS_PITCH_MAX=15`, `NAVL1_PERIOD=16`) may coincidentally be reasonable generic ArduPlane starting points, but this task does not cherry-pick individual "probably-fine" values out of an otherwise-rejected group — that would be an undocumented, unjustified judgment call. `falcon_v2_sitl.parm` leaves this entire group at ArduPlane's own compiled defaults. A future dedicated speed/height-controller tuning stage (out of scope here — this task explicitly must not retune anything) should derive Falcon-specific values from the Gazebo-validated trim/thrust data (`docs/test_results/2026-08-26_updated_powered_trim_high_deflection_validation.md`, `docs/test_results/2026-08-27_flight_envelope_validation.md`) if/when needed.

---

## 4. Airspeed: `AIRSPEED_*` / `ARSPD_*` — OLD (stale, no pitot) vs. NEW (simulated pitot)

**OLD group (real .param, CONFIRMED PRESENT): `STALE_REAL_AIRCRAFT_CONFIG` / `OTHER_AIRCRAFT_OR_STALE` — do NOT import.**
```
AIRSPEED_CRUISE=12  AIRSPEED_MAX=22  AIRSPEED_MIN=9  AIRSPEED_STALL=0
ARSPD_TYPE=0 (None)  ARSPD_PRIMARY=0  ARSPD2_TYPE=0
ARSPD_OFF_PCNT=0  ARSPD_OPTIONS=0  ARSPD_WIND_GATE=5  ARSPD_WIND_MAX=0  ARSPD_WIND_WARN=0
```
Per user-confirmed fact A: the real Falcon V2 had **no** pitot/airspeed sensor when this file was captured (`ARSPD_TYPE=0` confirms this directly — "None"). These `AIRSPEED_*` values are not real-aircraft-measured truth and are not imported anywhere in this task's outputs.

**NEW group (this task, `SIMULATION_ADDED_PITOT_SENSOR` / `SITL_REQUIRED` / `GAZEBO_VALIDATION`):**
- `ARSPD_TYPE=100` — **SITL** backend. CONFIRMED exact integer (not ambiguous — no `PID_MAPPING_REVIEW_REQUIRED`-style flag needed) by direct read of the installed `AP_Airspeed.h` enum: `TYPE_SITL=100`, and the `@Values` comment in `AP_Airspeed_Params.cpp` line 45 lists `100:SITL` explicitly. `AP_Airspeed_SITL::get_differential_pressure()` reads `AP::sitl()->state.airspeed_raw_pressure[instance]` — i.e. whatever value the SITL physics backend (or, at the next stage, the Gazebo/JSON bridge) supplies. **Wiring `plugins/sensors/PitotSystem.cc`'s published differential pressure into that SITL state field is explicitly NEXT-STAGE work, not done by this task** (task scope item 7: "at most confirm the bridge exists/builds if trivial" — confirmed to exist structurally via `AP_Airspeed_SITL.cpp`'s own source; not wired).
- `ARSPD_USE=1` — enables airspeed for automatic-throttle control loops once a real reading exists. `SITL_REQUIRED`/`V1_PROVISIONAL` — not exercised this stage (no closed-loop flight is run), included so the profile is ready for the next stage without a second migration pass.
- `AIRSPEED_MIN=16, AIRSPEED_CRUISE=18, AIRSPEED_MAX=28` — `GAZEBO_VALIDATION`, sourced from `docs/test_results/2026-08-27_flight_envelope_validation.md`'s `SIMULATION_DERIVED_FROM_GAZEBO_ENVELOPE` guidance (`VALIDATED_CORE_ENVELOPE` ≈18.166 and 24.0 m/s; `PROVISIONAL_EDGE_ENVELOPE` 14/16/21/28/30 m/s; `OUTSIDE_VALIDATED_ENVELOPE` 12.5 m/s explicitly excluded as a moment-infeasible trim, never used as MIN). 30 m/s is deliberately **not** used as MAX just because it numerically worked in one test — 28 m/s (a `PROVISIONAL_EDGE_ENVELOPE` point with margin below the outer edge) is used instead, per the task's own explicit guidance.
- `AIRSPEED_STALL=0` — `DATA_REQUIRED`. No validated stall speed exists anywhere in this repository's Gazebo test results (the flight-envelope report identifies 12.5 m/s as "moment-infeasible," not a characterized aerodynamic stall speed). Left at ArduPlane's own default (0 = unused/not configured) rather than inventing a number — the real .param's own value was also `0` (consistent, not a regression).
- `ARSPD_RATIO`, `ARSPD_OFFSET`: **not set** in `falcon_v2_sitl.parm` — left at ArduPlane compiled defaults (`RATIO=2.0`). Whether the raw differential-pressure value `PitotSystem.cc` computes (`0.5*rho*V^2`, `rho=1.225` CITED from `aero_v1_config.yaml`) needs a matching `ARSPD_RATIO` once the JSON bridge exists is an explicit **NEXT-STAGE** calibration question, flagged here, not resolved.

---

## 5. Battery: `BATT_*`

**Real .param (CONFIRMED PRESENT):** `BATT_CAPACITY=13000`, `BATT_MONITOR=4`, `BATT_VOLT_MULT=18.182`, `BATT_AMP_PERVLT=36.364`, `BATT_CURR_PIN=15`, `BATT_VOLT_PIN=14`, plus 8 unused `BATT2..BATT9_MONITOR=0`.

**Classification: `STALE_REAL_AIRCRAFT_CONFIG` — per user-confirmed fact B, do NOT promote to current truth.** `BATT_VOLT_PIN`/`BATT_CURR_PIN`/`BATT_VOLT_MULT`/`BATT_AMP_PERVLT` are Cube-Orange-analog-pin-specific hardware calibration, meaningless in SITL. `BATT_CAPACITY=13000` (mAh) is old config, not re-derived or confirmed against this project's `docs/source_of_truth/propulsion/` data (a 4S/2-motor system's real pack capacity is a separate question from the per-cell/2820-motor propulsion model already validated).

**`falcon_v2_sitl.parm` does NOT set any `BATT_*` value** — per task instruction, the less-invasive of the two allowed options ("omit/use ArduPlane-safe defaults... whichever is less invasive") is used: omission, letting ArduPlane's own compiled default (`BATT_MONITOR=0`/disabled unless configured) apply. Battery capacity for SITL remains `DATA_REQUIRED` if a future stage needs it (e.g. for a battery-failsafe percentage calculation) — not invented here.

---

## 6. EKF / AHRS: `EK3_*` / `AHRS_*`

**Real .param:** `EK3_*` (83 params — IMU/GPS/mag/baro noise and gating, bias-learning limits, GPS-blend/affinity config) and `AHRS_*` (13 params — trim offsets, EKF-type selector, GPS-use gating).

**Classification: `REVIEW_AND_ADAPT`, not imported this stage.** The bulk of `EK3_*` is either (a) generic EKF noise-model tuning tied to the REAL Cube Orange unit's own characterized IMU/GPS/mag/baro noise (not portable to Gazebo's ideal, currently-zero-noise simulated sensors, sec 7 of `SENSORS.md`), or (b) `AHRS_TRIM_X/Y/Z` — a physical IMU-mounting-angle calibration offset specific to the real board's installation, meaningless for a Gazebo sensor mounted at a clean, documented zero-offset orientation (`SENSORS.md` sec 4.1). `AHRS_EKF_TYPE=3` (EKF3) is a reasonable, non-airframe-specific ArduPlane default already and does not need to be imported explicitly (ArduPlane's own default is also EKF3). `falcon_v2_sitl.parm` leaves this entire group at ArduPlane compiled defaults; once real sensor noise is characterized (`SENSORS.md` sec 7, currently zero-noise-by-default V1) a future stage may need to revisit EK3 noise gating, not before.

---

## 7. GPS: `GPS_*` / `GPS1_*`

**Real .param:** `GPS_AUTO_CONFIG`, `GPS_BLEND_MASK`, `GPS_SBAS_MODE`, `GPS_NAVFILTER=8` (AIRBORNE_4G — an airframe-CLASS-appropriate navigation-filter setting for a fixed-wing, not hardware-specific), plus `GPS1_*` (u-blox module-specific antenna/config offsets).

**Classification: `REVIEW_AND_ADAPT` (mixed group).** Most of this group (`GPS_AUTO_CONFIG`, `GPS_SBAS_MODE`, `GPS1_*`) is real u-blox receiver hardware configuration — meaningless for the native `gz-sim` NavSat sensor (`SENSORS.md` sec 4.2), which has no SBAS/auto-config concept. `GPS_NAVFILTER=8` is airframe-class-appropriate but not imported individually this stage either, to avoid the same cherry-picking concern raised in sec 3 — ArduPlane's own default is retained. **Not imported into `falcon_v2_sitl.parm`.**

---

## 8. Compass: `COMPASS_*`

**Real .param (71 params):** device IDs, diagonal/off-diagonal calibration matrices (`COMPASS_DIA_X/Y/Z`, `COMPASS_ODI_X/Y/Z`), per-axis offsets, declination.

**Classification: `DROP_OR_DO_NOT_IMPORT`.** This is real Cube Orange magnetometer hardware calibration (factory + field compass-cal), physically meaningless for Gazebo's simulated magnetometer (`SENSORS.md` sec 4.4, which uses a documented `SIMULATION_ASSUMPTION` Earth field, not a calibrated real-world measurement to correct against). Importing these would silently apply a real sensor's calibration correction to a simulated sensor with entirely different (currently zero) error characteristics.

---

## 9. INS: `INS_*`

**Real .param (72 params):** accelerometer/gyro per-instance offsets, scale factors, calibration temperatures (`INS_ACC2OFFS_*`, `INS_ACC2SCAL_*`, `INS_ACC2_CALTEMP`, etc.), body-fix orientation.

**Classification: `DROP_OR_DO_NOT_IMPORT`.** Real IMU hardware calibration (factory + temperature compensation), meaningless for Gazebo's ideal simulated IMU (`SENSORS.md` sec 4.1, currently zero-noise-by-default, no bias/scale-factor error model exists to correct).

---

## 10. Board/bus/serial hardware IDs: `BRD_*` / `CAN_*` / `SERIAL*`

**Real .param:** `BRD_*` (26 — board type, safety switch behavior, heater PID for the real board's IMU heater, boot delay), `CAN_*` (28 — DroneCAN bus/node config for real CAN peripherals), `SERIAL*` (26 — real UART baud rates/protocols per physical port, e.g. `SERIAL3_PROTOCOL=5` GPS, `SERIAL6_PROTOCOL=22` presumably a companion-computer/MAVLink link).

**Classification: `DROP_OR_DO_NOT_IMPORT`.** All board-specific hardware identifiers and physical-port configuration — SITL has no physical board, CAN bus, or UART hardware in this sense (SITL's own equivalent transport, e.g. the JSON/MAVLink connection to Gazebo, is configured via `sim_vehicle.py` command-line arguments at the next stage, not via `SERIAL*` parameters replicating real hardware ports).

---

## 11. Flight modes: `FLTMODE*`

**Real .param:** `FLTMODE_CH=5` (channel 5 selects flight mode), `FLTMODE1..6` = `{11,10,10,10,0,5}` (mode numbers — 11=RTL region, 10=Auto region, 0=Manual, 5=FBWA, per standard ArduPlane mode numbering).

**Classification: `REVIEW_AND_ADAPT` — not imported this stage.** Per task rule 7, no closed-loop flight (FBWA/AUTO/LOITER/RTL/takeoff) is performed or enabled this stage. `FLTMODE_CH=5` is a real, structural fact (worth carrying forward conceptually into a future closed-loop-flight-preparation stage) but is not written into `falcon_v2_sitl.parm` — importing flight-mode-channel/mode-number configuration now would imply a closed-loop-flight-readiness this task does not establish or claim.

---

## 12. Failsafe: `FENCE_*` / `ARMING_*` / `AFS_*`

**Real .param:** `FENCE_ENABLE=0` (disabled on the real aircraft), `ARMING_CHECK=0` (real aircraft flew with most arming checks disabled — a real-flight operational choice, not necessarily SITL-appropriate), `ARMING_REQUIRE=1`, `AFS_ENABLE=1` with `AFS_HB_PIN=-1`/`AFS_MAN_PIN=-1` (both disabled — the Advanced Failsafe module's real GPIO-pin-based triggers were never wired).

**Classification: `REVIEW_AND_ADAPT` — not imported this stage.** `AFS_*`'s pin-based configuration is real-hardware-specific and mostly self-disabled anyway (`DROP_OR_DO_NOT_IMPORT` in substance). `FENCE_*`/`ARMING_*` are operational-safety choices tied to real flight-test practice at a specific site (e.g. `FENCE_RADIUS=300`, `FENCE_ALT_MAX=100` are site-specific), not portable defaults, and not needed for this task's sensor-and-mapping-preparation scope (no closed-loop or armed flight is run). None of `FENCE_*`/`ARMING_*`/`AFS_*` is written into `falcon_v2_sitl.parm` — ArduPlane compiled defaults apply.

---

## 13. Everything else not itemized above (`ACRO_*`, `ADSB_*`, `LAND_*`, `TKOFF_*`, `GUIDED_*`, `LOG_*`, `NTF_*`, `RPM1_*`/`RPM2_*`, `LGR_*`, `TUNE_*`, `TERRAIN_*`, `THR_*`, `SR0..SR6_*`, `RC1..RC16_*`)

**Classification: `DROP_OR_DO_NOT_IMPORT` (as a blanket group, this stage).** These are either (a) real-hardware/telemetry-stream/RC-receiver-calibration specifics (`SR*` per-link MAVLink stream rates, `RC1..16_MIN/MAX/TRIM/DZ` real transmitter calibration — SITL normally uses a joystick or built-in default RC values, not a real receiver's calibrated PWM range), (b) features entirely out of this task's scope (ADS-B, landing-gear `LGR_*`, RPM sensors, terrain following, takeoff/landing tuning `TKOFF_*`/`LAND_*` — all closed-loop-flight-adjacent, excluded by task rule 7), or (c) logging/notification config with no SITL-preparation relevance. None are written into `falcon_v2_sitl.parm`. If a future stage needs any of these, it should re-justify each individually rather than inherit this blanket exclusion.

---

## 14. Actuator / servo-function mapping table (controls-adjacent, kept here per task instruction — not duplicated in `SENSORS.md`)

| ArduPlane logical function | Real Falcon `SERVOx` | Gazebo actuator cmd topic | Sign conversion | Status |
|---|---|---|---|---|
| Aileron (`k_aileron`=4) | SERVO1 (`REVERSED=1`) | `/model/falcon_v2/actuators/left_aileron/cmd_rad` AND `/model/falcon_v2/actuators/right_aileron/cmd_rad` (Y-split) | See below | `PROVISIONAL_WIRING_ASSUMPTION` (Y-split) + `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST` (Gazebo-side joint sign) |
| Elevator (`k_elevator`=19) | SERVO2 (`REVERSED=0`) | `/model/falcon_v2/actuators/left_elevator/cmd_rad` AND `/model/falcon_v2/actuators/right_elevator/cmd_rad` (Y-split) | See below | `PROVISIONAL_WIRING_ASSUMPTION` (Y-split) + `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST` |
| Rudder (`k_rudder`=21) | SERVO4 (`REVERSED=1`) | `/model/falcon_v2/actuators/rudder/cmd_rad` | See below | `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST` |
| ThrottleLeft (`k_throttleLeft`=73) | SERVO3 (`REVERSED=0`) | `/model/falcon_v2/propulsion/left/throttle_cmd` | 1:1, no sign flip (both are already `0..1`/`0..100%`-normalized magnitude-only throttle commands) | independent per-motor, `KEEP_FOR_SITL` |
| ThrottleRight (`k_throttleRight`=74) | SERVO5 (`REVERSED=0`) | `/model/falcon_v2/propulsion/right/throttle_cmd` | 1:1, no sign flip | independent per-motor, `KEEP_FOR_SITL` |

**The one clean conversion chain, stated explicitly (per task instruction):**
```
ArduPlane normalized surface command (internal, -4500..+4500 centidegrees or -1..+1 depending on stage)
  -> SERVOx_REVERSED / SERVOx_MIN/MAX/TRIM applied by ArduPlane's own SRV_Channel code -> PWM microseconds
  -> [FUTURE ArduPilotPlugin JSON bridge, NOT built this task] -> logical reverse interpretation
     back to a physical-angle-radians target
  -> Gazebo actuator cmd topic (/model/falcon_v2/actuators/*/cmd_rad), consumed by
     plugins/actuators/ActuatorSystem.cc exactly as it already does today (UNCHANGED by this task)
  -> the ALREADY-VERIFIED Gazebo joint sign mapping (docs/source_of_truth/controls/CONTROLS.md sec 10):
       delta_e_aero = -0.5*(theta_left_elevator + theta_right_elevator)
       delta_a_aero = +0.5*(theta_right_aileron - theta_left_aileron)
       delta_r_aero =        theta_rudder
```

**Explicit confirmation `SERVOx_REVERSED` is not double-applied:** `SERVOx_REVERSED` is an ArduPlane-internal PWM-domain sign flip, applied entirely inside ArduPlane's own SRV_Channel output code, upstream of and independent from the Gazebo-side joint-sign mapping above (which was derived and verified purely from Gazebo hinge-axis geometry and live aero-moment sign — `docs/test_results/2026-08-22_control_surface_sign_mapping_test_report.md` — with zero dependency on any ArduPlane parameter). The two sign conventions live in non-overlapping layers (ArduPlane PWM-normalization vs. Gazebo joint-angle-radians physical actuation) and are never combined or cancelled against each other by any code that exists today — the future JSON bridge is the ONE place a translation between them will need to be written, explicitly, once (not built this task, task scope item 7).

**Twin-motor independence:** `ThrottleLeft`/`ThrottleRight` (functions 73/74) map 1:1 to this project's existing independent `/model/falcon_v2/propulsion/{left,right}/throttle_cmd` topics (`plugins/propulsion/PropulsionSystem.cc`, unmodified) — full per-motor independence (needed for the already-validated engine-out/asymmetric-thrust behavior, `docs/test_results/2026-08-27_engine_out_asymmetric_thrust_validation.md`) is preserved end-to-end, with no mixing/averaging introduced anywhere in this chain.

**Y-split (`PROVISIONAL_WIRING_ASSUMPTION`), restated explicitly:** the user's recalled wiring — one logical Aileron output Y-split to L/R aileron servos, one logical Elevator output Y-split to L/R elevator servos — is corroborated by the real .param showing only ONE `SERVO*_FUNCTION=4` (Aileron) and only ONE `SERVO*_FUNCTION=19` (Elevator) channel (sec 2 above), consistent with a single logical output feeding two physical servos via a Y-harness rather than two independently-addressed ArduPlane outputs. This is corroborating evidence, not proof (no CAD/wiring-diagram source confirms it) — the tag remains `PROVISIONAL_WIRING_ASSUMPTION` per the task's own instruction, not upgraded to `CONFIRMED`. **Practical consequence for the future bridge:** a single ArduPlane Aileron/Elevator PWM output will need to be broadcast to BOTH corresponding Gazebo topics (`left_*`/`right_*`) identically (subject to each side's own already-verified Gazebo joint sign, which already differs appropriately between left/right per `CONTROLS.md` sec 10's `delta_a_aero`/`delta_e_aero` formulas) — not resolved by this task, flagged for the bridge-building stage.

---

## 15. Summary — file produced

`config/ardupilot/falcon_v2_sitl.parm` contains exactly: the 8 manufacturer-initial PID gains (sec 1) + `AUTOTUNE_LEVEL=8` (not executed), the 5×5 real `SERVOx_FUNCTION/MIN/MAX/TRIM/REVERSED` values (sec 2), and the new simulated-pitot airspeed group (sec 4: `ARSPD_TYPE`, `ARSPD_USE`, `AIRSPEED_MIN/CRUISE/MAX/STALL`). Every other real-.param group discussed above (secs 3, 5–13) is explicitly excluded, with the reasoning recorded here rather than left as an unexplained omission.

**Format note:** `falcon_v2_sitl.parm` uses ArduPilot's own `Tools/autotest/default_params/*.parm` convention — whitespace-separated `NAME VALUE` pairs with `#`-prefixed comment lines — confirmed by direct read of `pymavlink/mavparm.py`'s `load()` method (`line.split()` after skipping blank/`#`-prefixed lines), the actual parser ArduPilot SITL tooling (`sim_vehicle.py --add-param-file`) uses. This is deliberately **different** from the real aircraft's own `yeni_pixhawk.param` comma-CSV GCS-export format (which is a different, GCS-side export convention, not meant for `--add-param-file`) — not an inconsistency, a deliberate choice matching the file's actual intended consumer.

## 16. Open items carried forward (not resolved by this task)

- `PID_MAPPING_REVIEW_REQUIRED`: none — the manufacturer-PID→`RLL_RATE_*`/`PTCH_RATE_*` mapping is direct and unambiguous (sec 1).
- `REAL_PARAM_VS_SIM_PITOT_REVIEW_REQUIRED`: the exact `ARSPD_RATIO`/`ARSPD_OFFSET` calibration needed once `PitotSystem.cc`'s differential pressure is actually wired into `AP::sitl()->state.airspeed_raw_pressure` (sec 4) — next-stage work.
- `DATA_REQUIRED`: `AIRSPEED_STALL` (no validated stall speed in this repository, sec 4); real battery capacity for SITL if structurally needed later (sec 5); `MAG_FIELD_SOURCE_REVIEW_REQUIRED` and `BARO_ALTITUDE_RESPONSE_REVIEW_REQUIRED` (both `docs/source_of_truth/sensors/SENSORS.md` sec 4.3/4.4 — live-observed, not yet explained).
- Y-split ArduPlane→dual-Gazebo-topic broadcast mechanism (sec 14) and the PWM-domain↔radians-domain translation generally: unbuilt, next-stage (ArduPilotPlugin JSON bridge).

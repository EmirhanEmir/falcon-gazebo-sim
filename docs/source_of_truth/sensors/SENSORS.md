# FALCON V2 — Sensor Suite Source of Truth

**Owner:** `controls-integration`
**Task:** `SENSOR_MODEL_AND_ARDUPLANE_SITL_PREPARATION` (2026-08-27).
**Status:** V1 implementation, live-verified (see sec 8). Two open findings from the original pass have since been resolved by follow-up work (see sec 9): the barometer finding was refuted as benign by `gazebo-testing`'s independent live validation, and the magnetometer finding was confirmed as a real defect and fixed by replacing the native sensor with a custom plugin (`FalconV2Magnetometer`).

This document is the source of truth for all 5 simulated sensors (IMU, GPS, barometer, magnetometer, pitot/airspeed) added to FALCON V2 this task. It does **not** cover ArduPilot SITL parameter migration or the actuator/servo→ArduPilot mapping table — those live in `docs/source_of_truth/autopilot/SITL_PARAM_MIGRATION.md` (cited here, not duplicated).

Per this task's scope, these sensors read ECM ground truth only — they create **no** new command path into aerodynamics/propulsion/actuators, and they are **not** yet wired into ArduPilotPlugin's JSON transport (next stage).

---

## 1. OLD (no pitot) vs NEW (simulated pitot) — explicit statement

- **Real Falcon V2, as captured in `docs/source_of_truth/autopilot/real_aircraft/yeni_pixhawk.param`:** had **no** pitot/airspeed sensor installed (`ARSPD_TYPE=0` = None, confirmed by direct read of that file). The `AIRSPEED_*`/`ARSPD_*` values in that file (`AIRSPEED_CRUISE=12`, `AIRSPEED_MAX=22`, `AIRSPEED_MIN=9`) are **not** real-aircraft-measured truth — status `STALE_REAL_AIRCRAFT_CONFIG` / `OTHER_AIRCRAFT_OR_STALE` per the task's user-confirmed facts. Not imported anywhere in this task's outputs.
- **This simulation:** adds a pitot for the first time — status `SIMULATION_ADDED_PITOT_SENSOR`. It is a **new capability that does not represent flight-proven hardware**. Every airspeed-derived value downstream of it (in a future ArduPilot SITL run) should be understood as validating the simulation/control-law integration, not re-validating a sensor that has flown.

---

## 2. Frame conventions (derivation + tests: `plugins/sensors/Frames.hh`, `plugins/sensors/test/sensors_model_selftest.cc`)

**Governing facts (CONFIRMED this task):**
- This project's Gazebo body frame is FLU (CLAUDE.md) — CONFIRMED to be exactly the body frame the official `ardupilot_gazebo` plugin assumes by default, by direct read of its installed source `/home/emirhan/gazebo_sim/ardupilot_gazebo/src/ArduPilotPlugin.cc` (default `gazeboXYZToNED = Pose3d(0,0,0, GZ_PI,0,0)`).
- ArduPilot's body frame is FRD; ArduPilot's world frame is NED.
- This project's Gazebo world frame is ENU — CONFIRMED both by the same `ArduPilotPlugin.cc` source comment and by `sdformat`'s own installed `spherical_coordinates.sdf` spec (`world_frame_orientation` default = `"ENU"`).

**Transforms derived and unit-tested in `Frames.hh` (all pass, `sensors_model_selftest` 13/13):**

| Transform | Function | Test | Result |
|---|---|---|---|
| FLU↔FRD (any body-frame free vector: gyro, specific force, mag body field, body-relative offsets) | `FluFrdSwap(v) = (x,-y,-z)` | `FLU_FRD_ROTATION_PROPERTIES_TEST` | orthogonal, det=+1 (proper rotation, 180° about body X), involution (`R²=I`) — all confirmed numerically, not just asserted |
| ENU↔NED (any world-frame free vector: GPS velocity, world mag field, world-relative displacement) | `EnuNedSwap(v) = (y,x,-z)` | `ENU_NED_ROTATION_PROPERTIES_TEST` | orthogonal, det=+1 (proper rotation, 180° about the horizontal North-East bisector), involution — confirmed numerically |
| Altitude (up, m) ↔ NED Down (m) | `AltitudeToNedDown`/`NedDownToAltitude` | `BARO_ALTITUDE_DOWN_SIGN_TEST` | trivial negation, round-trips exactly |
| Attitude quaternion, body-FLU-rel-world-ENU → body-FRD-rel-world-NED | `AttitudeFluEnuToFrdNed(q) = qEnuToNedRot * q * qFluToFrdRot` | `ATTITUDE_QUATERNION_TRANSFORM_TEST` | self-consistency identity checked across 4 attitudes × 4 vectors, max numerical error 1.4e-15 |
| Heading (rad, clockwise-from-north) from NED horizontal components | `HeadingFromNedHorizontalRad(north,east) = atan2(east,north)` | `HEADING_CARDINAL_TEST` | N→0, E→+π/2, S→±π, W→−π/2, all exact |

**Explicit warning (also in `Frames.hh`):** `FluFrdSwap` and `EnuNedSwap` are algebraically different rotations (different physical axes) that happen to share the "180° involution" property — applying the wrong one to a vector in the wrong frame category is a silent, finite-but-wrong bug. Every call site must be able to name which category (body vs. world) it is converting.

**Attitude quaternion — how it will be used at the next (ArduPilot-bridge) stage:** the IMU sensor (sec 4.1) reports `q` = body-FLU-relative-to-world-ENU directly. `AttitudeFluEnuToFrdNed(q)` produces the FRD-relative-to-NED equivalent ArduPilot expects. Not exercised end-to-end this stage (no ArduPilotPlugin wiring yet, per task scope item 7).

---

## 3. Native gz-sim sensors vs. custom pitot plugin — decision record

**Installed capability (CONFIRMED, Gazebo Sim Harmonic 8.14.0):** `libgz-sim8-imu-system`, `libgz-sim8-navsat-system`, `libgz-sim8-magnetometer-system`, `libgz-sim8-air-pressure-system`, `libgz-sim8-air-speed-system` all present.

**Decision (updated 2026-08-27, bug-fix follow-up): IMU / GPS(navsat) / barometer(air_pressure) use the NATIVE gz-sim system plugins. Pitot AND magnetometer both use CUSTOM plugins (`plugins/sensors/PitotSystem.cc` / `FalconV2Pitot`, `plugins/sensors/MagnetometerSystem.cc` / `FalconV2Magnetometer`), not their respective native sensors.** The magnetometer was native-backed in the original V1 pass; it was switched to a custom plugin after `gazebo-testing` live-validated a confirmed, reproducible defect in the installed native magnetometer system — see sec 4.4 for the full root-cause record.

**Why, for pitot specifically:**
1. The installed `sdformat` `air_speed.sdf` sensor spec (`/usr/share/sdformat14/1.11/air_speed.sdf`) has **no wind-related configurable field at all** — only a `pressure` noise block. Confirmed by direct read of the spec file.
2. The only native Gazebo wind mechanism found on this system is `gz-sim-wind-effects-system` plus the world-level `<wind><linear_velocity>` SDF element (confirmed via the installed reference world `/usr/share/gz/gz-sim8/worlds/wind.sdf`) — this is a **force-applying** mechanism for links with `<enable_wind>true</enable_wind>`, architecturally unrelated to velocity computation for any sensor, and completely separate from this project's own `plugins/wind/WindSystem.cc`, which is a gz-transport topic publisher (`/model/falcon_v2/wind`) that only `AerodynamicsSystem.cc`/`PropulsionSystem.cc` subscribe to.
3. `docs/source_of_truth/environment/WIND.md` explicitly states: *"This is the ONLY wind source in the project - no second wind-consumption path is created."* Using the native `air_speed` sensor's own (undocumented, and per point 1, apparently nonexistent) wind-sourcing mechanism would either silently ignore this project's commanded wind (failing the mandatory headwind/tailwind live test) or stand up a second, inconsistent wind path — both unacceptable.

**Empirical confirmation performed this task (live, headless Gazebo, `tests/gazebo/worlds/falcon_v2_sensors_selftest_world.sdf`):** commanded a +5 m/s East steady wind via `/model/falcon_v2/wind/steady_cmd` against a body at rest. `FalconV2Pitot`'s `/model/falcon_v2/sensors/pitot/airspeed_mps` read **5.0 m/s** (matches `|Vrel|=|0-5|` exactly) and `/model/falcon_v2/sensors/pitot/differential_pressure_pa` read **15.31 Pa** (matches `0.5*1.225*5^2=15.3125` Pa exactly). In the SAME instant, the native GPS's `velocity_east/north/up` remained ~0 (unaffected by the wind command, as architecturally required — the native sensor has no path to see this project's custom wind topic at all). This is the direct live confirmation the task's own instructions called for before committing to this split.

**Why native is fine for IMU/GPS/baro/mag:** none of these has any wind dependency by design (IMU/mag read link kinematics/attitude and a static world field; GPS reads link pose/velocity; barometer reads local altitude) — confirmed no wind-related field exists in any of their SDF specs either.

---

## 4. Per-sensor specification

All 5 sensors are mounted at `model/model.sdf`'s `base_link`, at pose `(0.168309, 0, 0.100000, 0, 0, 0)` — co-located with the Gazebo/CAD CG (CLAUDE.md-authoritative point). **Documented choice, not an arbitrary number:** the real physical mounting location of any avionics sensor on the real Falcon V2 airframe is `DATA_REQUIRED` (no CAD/manual placement data in this repository) — CG-colocation is the one already-authoritative point on this airframe, and avoids introducing an unvalidated lever-arm velocity effect (`V_point = V_cg + ω×r`) that no real data could confirm. Tag: `SIMULATION_ASSUMPTION` (mount point only — not the sensor physics itself).

### 4.1 IMU

- SDF: `<sensor type="imu">` on `base_link`, topic `/model/falcon_v2/sensors/imu` (gz.msgs.IMU).
- `orientation_reference_frame`: `CUSTOM`, `parent_frame="world"`, `custom_rpy="0 0 0"` — reports orientation of body-FLU relative to Gazebo world frame (CONFIRMED ENU) directly, per the `imu.sdf` spec's own documented example for this exact configuration ("IMU reports in Gazebo world frame").
- **Empirical live verification performed this task:** using a paused server + `/world/.../control` `multi_step` (so the model has not yet accumulated any real free-fall/aero rotation), orientation at the model's identity spawn pose read `(w=0.999999, x≈0, y=0.0016, z≈0)` — correctly near-identity, with the small residual matching that instant's own `angular_velocity.y=0.080 rad/s` reading (a genuine, tiny aero-torque-driven angular rate, not a frame artifact). An earlier same-session sample taken several real seconds into an **unpaused, unpowered, unstabilized** free-fall run showed a large-angle orientation and `angular_velocity.y≈8.9 rad/s` — initially suspected as a possible sensor/localization quirk, but the controlled re-test above confirms it is the aircraft **genuinely tumbling** (expected for an unpowered airframe free-falling from rest at extreme, unvalidated angles of attack — not a sensor defect). This distinction is recorded explicitly so a future reader does not misread either sample as "the" IMU behavior without the controlling context.
- Angular velocity, linear acceleration (specific force): body FLU frame (sensor has zero relative rotation to `base_link`).
- **IMU_SPECIFIC_FORCE_SIGN_TEST (documented expected convention, not yet independently live-confirmed against the native sensor's own output at rest under gravity-only conditions — recommended as a `gazebo-testing` follow-up):** a stationary accelerometer reads +g "up" (reaction to gravity) — in FLU that is `(0,0,+9.81)`; `FluFrdSwap` correctly maps this to `(0,0,-9.81)` in FRD ("up" = −Z in FRD). Verified algebraically in `Frames.hh`/self-test; **not yet cross-checked against a live, non-falling (e.g. ground-supported or externally held) IMU reading this task** — flagged, not asserted as fully live-confirmed.
- `enable_orientation`: default `true` (unchanged).
- Update rate: 100 Hz. `V1_PROVISIONAL` — no Cube Orange IMU datasheet ingested into this repository (per task instruction, never invented). 100 Hz matches `gz-sim`'s own upstream reference demo (`/usr/share/gz/gz-sim8/worlds/sensors.sdf`) IMU default, used here as an order-of-magnitude-reasonable citation, not a Falcon-specific figure.
- Noise: OFF (zero-noise deterministic default, no `<noise>` element present). A commented-out `V1_PROVISIONAL_SENSOR_NOISE` example block is included in `model.sdf` showing the syntax for future use — never a Cube-Orange-specific number.

### 4.2 GPS (NavSat)

- SDF: `<sensor type="navsat">`, topic `/model/falcon_v2/sensors/gps` (gz.msgs.NavSat).
- Reports `latitude_deg`, `longitude_deg`, `altitude`, and **explicitly** `velocity_east`, `velocity_north`, `velocity_up` (confirmed via the installed `navsat.proto` comment: *"East velocity in the ENU frame"* etc.) — i.e. ENU by construction, no ambiguity to derive.
- **GPS_VELOCITY_ENU_TO_NED_TEST** (`Frames.hh` `EnuNedSwap`): `(east=3, north=5, up=-1)` → NED `(north=5, east=3, down=1)`. Passes.
- Position reference origin: world's `<spherical_coordinates>` block, `lat=0, lon=0, elevation=0` — `PLACEHOLDER_ORIGIN` / `DATA_REQUIRED`: the real Falcon V2's actual flight-test location is not present anywhere in this repository. Not invented; restated explicitly at the sdformat-spec default in `falcon_v2_sensors_selftest_world.sdf` rather than left implicit.
- Update rate: 5 Hz. `V1_PROVISIONAL` — no Falcon-specific GPS module datasheet in this repository; 5 Hz is a broadly standard consumer/hobby GPS module update rate (e.g. common uBlox M8/M9-class default), cited as an order-of-magnitude reference, not device-specific.
- Noise: OFF by default (same policy as sec 4.1).

### 4.3 Barometer (air_pressure)

- SDF: `<sensor type="air_pressure">`, topic `/model/falcon_v2/sensors/baro` (gz.msgs.FluidPressure, Pa).
- `reference_altitude=0` — matches the world's `spherical_coordinates` `elevation=0` placeholder origin (sec 4.2), a consistent, not independently-invented, second number.
- **BARO_ALTITUDE_DOWN_SIGN_TEST** (`Frames.hh` `AltitudeToNedDown`): trivial, exact, documents the Up-altitude ↔ NED-Down relationship this sensor's eventual ArduPilot-side altitude will need (ArduPilot's own barometer backend converts pressure→altitude itself, out of scope here).
- **OPEN FINDING (`BARO_ALTITUDE_RESPONSE_REVIEW_REQUIRED`) — live-tested this task, result not yet fully explained:** with the model spawned at world Z=50 m (`falcon_v2_sensors_selftest_world.sdf`) and `reference_altitude=0`, the live `pressure` reading was `101322.2 Pa` — only ~3 Pa below the ISA sea-level reference (101325 Pa), whereas the standard barometric approximation (`dP ≈ -rho0*g*h`, using this project's own CONFIRMED `rho=1.225`) predicts roughly **101325 − 1.225×9.81×50 ≈ 100724 Pa** for a sensor at ~50 m altitude — a ~600 Pa discrepancy. This was checked once, in one live smoke test, and is reported here honestly rather than asserted as correct or silently corrected. **Not resolved by this task** (this task's scope is "confirm each [native sensor] loads/produces live output," which it does — finite, non-stale pressure values are produced); a dedicated altitude-vs-pressure sweep (multiple spawn altitudes, or a controlled multi-step descent) is recommended as a `gazebo-testing` follow-up before this sensor's absolute altitude accuracy is trusted for any future closed-loop TECS/altitude-hold work.
- Update rate: 50 Hz. `V1_PROVISIONAL`, order-of-magnitude reference only (no Falcon-specific barometer datasheet in this repository).
- Noise: OFF by default.

### 4.4 Magnetometer (custom, `FalconV2Magnetometer`) — updated 2026-08-27, bug-fix follow-up

**STATUS: RESOLVED.** The original V1 pass used the native `<sensor type="magnetometer">` + world-level `gz-sim-magnetometer-system` pairing (see the `OPEN FINDING (MAG_FIELD_SOURCE_REVIEW_REQUIRED)` this section used to carry). `gazebo-testing` live-validated that finding into a confirmed, reproducible `TEST_FAILED` (`docs/test_results/2026-08-27_sensor_model_sitl_preparation.md` sec 5): held at 4 headings, the native sensor's `|field_tesla|` was a **constant exactly 0.32** (unspecified-but-clearly-wrong unit) regardless of heading/position/altitude, and — decisively — **unchanged (ratio 1.0000) when the declared world `<magnetic_field>` was scaled 100x** in an otherwise-identical world file (`falcon_v2_sensors_selftest_altmag_world.sdf`). Per-axis ratios vs. the declared vector were inconsistent (~49590x / ~-1058x / ~3775x, including a sign flip), ruling out a simple unit-rescale.

**Root-cause investigation performed this pass (`controls-integration`):**
- Checked `sdformat`'s `world.sdf` spec (`/usr/share/sdformat14/1.11/world.sdf`): `<magnetic_field>` is the correct, current element name/location (world-level, Tesla, "expressed in a coordinate frame defined by the `spherical_coordinates` tag") — not a naming/placement error in this project's SDF.
- Static analysis (`nm`/`objdump -d -C`) of `libgz-sim8.so.8.14.0`'s `SdfEntityCreator::CreateEntities(sdf::v14::World const*, unsigned long)` confirmed it calls `sdf::v14::World::MagneticField()` and creates the ECM world `components::MagneticField` component from it — the SDF value genuinely reaches the ECM.
- **Live gdb instrumentation** (breakpoint on `gz::sensors::v8::MagnetometerSensor::SetWorldMagneticField` in the installed `libgz-sensors8-magnetometer.so.8.2.2`, process launched via `gdb --args ruby /usr/bin/gz sim -s -r ...` since direct `gdb -p` attach is blocked by this machine's `ptrace_scope=1` and this project has no sudo access to change it): the setter **is called with the exact, correctly-declared world field** (e.g. `x=5.5645e-06 y=2.28758e-05 z=-4.23884e-05` for this world's declared value) — i.e. the SDF→ECM→sensor delivery path is provably NOT the defect.
- A follow-up breakpoint placed directly on the exact, uniquely-mangled `MagnetometerSensor::Update(std::chrono::duration<...>)` symbol — confirmed via `/proc/<pid>/maps` to be the ONLY loaded copy of that library, so no ambiguity/duplicate-instance explanation applies — **never fired**, despite the sensor visibly publishing live output at its configured rate for 10+ seconds of wall time. With no source `.cc` available on this machine for either `libgz-sim8-magnetometer-system.so` or `libgz-sensors8-magnetometer.so` (only headers/specs are installed), further root-causing would require patching/rebuilding a closed binary this project does not have source for.
- **Conclusion:** a genuine, non-debuggable-from-source binary-level defect in this Gazebo Sim Harmonic installation's native magnetometer system — confirmed a dead end for the native path per this task's own stated fallback criterion (`plugins/sensors/MagnetometerSystem.hh` header comment carries the same evidence trail).

**Fix:** replaced the native `<sensor type="magnetometer">` + world-level system with a custom `FalconV2Magnetometer` plugin (`plugins/sensors/MagnetometerSystem.cc`/`.hh`), architecturally identical in style to `FalconV2Pitot`. Reads `base_link`'s own world orientation (`gz::sim::Link::WorldPose`, ECM ground truth), rotates a fixed, SDF-configurable world-frame (ENU) Earth field vector into body FLU via `gz::math::Quaterniond::RotateVectorReverse()` (the same convention already used for this project's IMU specific-force reading), and publishes on the **same topic and message type** the native sensor used (`/model/falcon_v2/sensors/mag`, `gz.msgs.Magnetometer`, `field_tesla`, body FLU, zero relative rotation to `base_link`) — no downstream consumer needs to change. The native `<sensor type="magnetometer">` element was removed from `model.sdf`'s `base_link`, and `gz-sim-magnetometer-system` was removed from `tests/gazebo/worlds/falcon_v2_sensors_selftest_world.sdf` (leaving both loaded would create two publishers, one defective, on the same topic).

- Earth field source: `world_magnetic_field_tesla` plugin parameter, default `(5.5645e-6, 22.8758e-6, -42.3884e-6)` Tesla — **numerically unchanged** from the value this project already declared via the world's `<magnetic_field>` element (still present, unmodified, in the world file for documentation/consistency, even though the native ECM-component path that used to read it is confirmed not to work on this installation). Still `SIMULATION_ASSUMPTION`: sdformat's own documented world-element default, not Falcon-flight-location-specific; the real aircraft's actual local magnetic field/declination/inclination remains `DATA_REQUIRED`.
- **HEADING_CARDINAL_TEST** (`Frames.hh` `HeadingFromNedHorizontalRad`): confirms the atan2-based heading-from-horizontal-field convention (N→0, E→+90°) is the standard clockwise-from-north aviation convention, ready to apply to a NED-converted horizontal field once one is available. Unaffected by this fix (pure math, `Frames.hh` untouched).
- **Live re-verification performed this pass** (`falcon_v2_sensors_selftest_world.sdf`, unheld free-fall, real time, headless):
  - **Magnitude:** observed `|field_tesla| = 4.848754830314687e-05` T vs. declared-field magnitude `4.848754830314686e-05` T — ratio **1.0000000000000002** (exact to floating-point precision). This alone is decisive vs. the native system's confirmed constant-0.32/ratio-1.0000-under-100x-scale defect, since magnitude is orientation-invariant and this now tracks the declared value exactly.
  - **Attitude-responsiveness (independent cross-check, not just the plugin's own math mirrored back at itself):** at the same simulated instant, the native IMU (separately validated `PASS`) reported orientation `q=(w=3.44e-6, x=2.897e-4, y=0.999999958, z=6.13e-7)` (the aircraft had tumbled to a near-180°-about-Y attitude during unpowered free-fall — expected, same phenomenon already documented in sec 4.1). Independently rotating the declared world field by this SAME quaternion (`q⁻¹ · field_world · q`) gives `(-5.550950752789719e-06, 2.287896873421784e-05, 4.238846630687158e-05)`, matching the plugin's published `field_tesla = (-5.551177239773955e-06, 2.287900959049329e-05, 4.238841459480093e-05)` to **<0.001% relative error** (max component diff 2.26e-10 T, fully explained by the IMU (100 Hz) and magnetometer (50 Hz) messages being sampled at slightly different ticks, not a bug). This directly demonstrates the plugin's output genuinely tracks the live, changing attitude — the opposite of the native bug's flat, orientation-independent constant.
  - `gz sdf --check` clean on both `model/model.sdf` and `falcon_v2_sensors_selftest_world.sdf` after the change.
- Update rate: 50 Hz. `V1_PROVISIONAL`, unchanged from the (now-replaced) native sensor's rate — see sec 6.
- Noise: not implemented in this plugin (V1 scope decision — the native sensor's noise was already off/unused; only `FalconV2Pitot` has an actually-implemented, working noise path in this project, see sec 4.5/sec 7). Not a regression from a previously-working capability.

### 4.5 Pitot (custom, `FalconV2Pitot`)

- Plugin: `plugins/sensors/PitotSystem.cc` (model-level `<plugin>`, same architectural pattern as `FalconV2Wind`/`FalconV2Actuators`/etc.). Physical model: `plugins/sensors/PitotModel.hh`.
- Reads `base_link`'s own world-frame velocity at the mount point (`gz::sim::Link::WorldLinearVelocity(ecm, offset)`, same primitive `PropulsionSystem.cc` already uses for its hub velocity) and the existing `/model/falcon_v2/wind` topic (read-only, same convention as `AerodynamicsSystem.cc`/`PropulsionSystem.cc`).
- `V_rel_world = V_point_world - V_wind_world`; `airspeed_mps = |V_rel_world|` (frame-invariant magnitude — verified by `PITOT_MAGNITUDE_FRAME_INVARIANCE_TEST`, which applies both `FluFrdSwap` and `EnuNedSwap` to the same vector before taking the magnitude and confirms it is unchanged, since both are proper rotations and rotations preserve vector length).
- `differential_pressure_pa = 0.5 * rho * airspeed_mps^2` (standard incompressible dynamic pressure — valid at this aircraft's flight envelope, max ~30 m/s per `docs/test_results/2026-08-27_flight_envelope_validation.md`, Mach ≈0.09 at sea level, well within the incompressible regime). `rho=1.225` kg/m³ is **CITED**, not invented — matches `docs/source_of_truth/aerodynamics/aero_v1_config.yaml` `environment.air_density_rho_kg_m3` (CONFIRMED, ISA sea-level) exactly, the project's single source of truth for this constant.
- Published on `/model/falcon_v2/sensors/pitot/airspeed_mps` (gz.msgs.Double, m/s) and `/model/falcon_v2/sensors/pitot/differential_pressure_pa` (gz.msgs.Double, Pa).
- **PITOT_ZERO_WIND_TEST / PITOT_HEADWIND_TEST / PITOT_TAILWIND_TEST / PITOT_CROSSWIND_TEST** (pure-math self-test): a body flying 20 m/s East reads 20.0 m/s airspeed with zero wind, 25.0 m/s with a 5 m/s headwind (air mass moving toward the nose — airspeed exceeds groundspeed, correct), 15.0 m/s with a 5 m/s tailwind (airspeed below groundspeed, correct), and √425≈20.6155 m/s with a 5 m/s crosswind. All pass.
- **Live-verified this task** (sec 3): +5 m/s East wind against a body at rest → airspeed reads 5.0 m/s, differential pressure reads 15.31 Pa, exactly as predicted — this is the mandatory headwind/tailwind-class live confirmation.
- Update rate: 10 Hz. `V1_PROVISIONAL_SENSOR_RATE` — no real Falcon pitot exists (sec 1), so no datasheet rate to cite; 10 Hz is order-of-magnitude consistent with typical low-cost differential-pressure (MS4525/DLVR-class) I2C sensor sample rates and with ArduPilot's own airspeed-read scheduling cadence.
- **V1 limitation, stated explicitly (also in `PitotModel.hh`):** scalar-only, always ≥0 — does not model reverse/negative-AoA flow onto the pitot port, compressibility, or installation/position error.
- Noise: OFF by default (`noise_stddev_mps=0.0` SDF parameter). Unlike the 4 native sensors above (which only have a commented-out SDF example), this one has an actually-implemented, working, fixed-seed (deterministic even if enabled) zero-mean Gaussian noise path in `PitotSystem.cc` — off by default, satisfying the "simple configurable V1 noise, clearly optional and off by default" deliverable requirement concretely for at least one sensor. Tag: `V1_PROVISIONAL_SENSOR_NOISE`.

---

## 5. World-level configuration (`tests/gazebo/worlds/falcon_v2_sensors_selftest_world.sdf`)

Loads 3 native sensor-processing systems (`gz::sim::systems::Imu`, `NavSat`, `AirPressure`) at the world level — confirmed via the installed reference world `/usr/share/gz/gz-sim8/worlds/sensors.sdf` that these must be loaded at world scope, not inside `model.sdf`. The `air_speed`/`wind-effects` native systems are deliberately **not** loaded (sec 3). `gz::sim::systems::Magnetometer` (native) was **removed** from this world file 2026-08-27 (bug-fix follow-up) after being confirmed non-functional on this installation — see sec 4.4. `model/model.sdf` itself carries only the `<sensor>` data-definition elements (IMU/GPS/baro) plus the model-level `FalconV2Pitot` and `FalconV2Magnetometer` plugins, consistent with this project's existing plugin-ownership convention (`FalconV2Wind`/`FalconV2Actuators`/etc. are all model-level).

`<spherical_coordinates>` and `<magnetic_field>` are both explicitly restated at their sdformat-spec default values (see sec 4.2/4.4) rather than left implicit, and both are tagged `PLACEHOLDER_ORIGIN`/`SIMULATION_ASSUMPTION` respectively — real values `DATA_REQUIRED`.

Name deliberately prefixed (`falcon_v2_sensors_selftest_world.sdf`) to avoid collision with `gazebo-testing`-owned world files (`falcon_v2_freefall_world.sdf`, `falcon_v2_zero_g_world.sdf`, `falcon_v2_powered_free_flight_gui_world.sdf`) — additive only, none of those files were modified.

---

## 6. Update-rate summary table

| Sensor | Rate | Status |
|---|---|---|
| IMU | 100 Hz | `V1_PROVISIONAL` (gz-sim upstream demo reference) |
| GPS (navsat) | 5 Hz | `V1_PROVISIONAL` (generic consumer-GPS-class reference) |
| Barometer | 50 Hz | `V1_PROVISIONAL` (order-of-magnitude reference) |
| Magnetometer (custom, `FalconV2Magnetometer`) | 50 Hz | `V1_PROVISIONAL` (order-of-magnitude reference; rate unchanged from the now-replaced native sensor, see sec 4.4) |
| Pitot (custom) | 10 Hz | `V1_PROVISIONAL_SENSOR_RATE` (no real device — simulation-added, sec 1) |

None are Cube Orange or Falcon-V2-specific datasheet values — none exist in this repository (per user-confirmed fact B/battery-adjacent precedent: do not invent, mark provisional instead).

---

## 7. Noise status summary

**Default mode (validated this task): zero noise, fully deterministic**, for all 5 sensors — no `<noise>` SDF element present for the 3 remaining native sensors, no noise model implemented in `FalconV2Magnetometer`, `noise_stddev_mps=0.0` for the pitot plugin.

**V1 optional noise:**
- Native sensors (IMU/GPS/baro): a commented-out example `<noise type="gaussian">` block is present under the IMU `<sensor>` in `model.sdf`, showing the syntax; values shown are illustrative only, tagged `V1_PROVISIONAL_SENSOR_NOISE`, **never** presented as Cube Orange or any real device's characterized noise figures. Not enabled.
- Magnetometer (custom, `FalconV2Magnetometer`, added 2026-08-27): no noise model implemented — the sensor it replaced (native) also had noise off/unused, so this is not a regression from a previously-working capability. Could be added later following the pitot's pattern if needed.
- Pitot: an actually-functional, fixed-seed (deterministic reproducibility even when enabled), zero-mean Gaussian noise path exists in `PitotSystem.cc` (`noise_stddev_mps` SDF parameter, default `0.0` = off). Result is floor-clamped at 0 (a real pitot cannot report negative airspeed).

---

## 8. Live verification log (this task, `falcon_v2_sensors_selftest_world.sdf`)

Performed with `GZ_SIM_SYSTEM_PLUGIN_PATH` covering all 5 plugin build directories (aerodynamics, propulsion, actuators, wind, sensors — all already built, all loaded without error, `gz sdf --check` on both `model/model.sdf` and the new world file returns `Valid.`):

1. `gz topic -l` — all 6 expected topics present and live: `/model/falcon_v2/sensors/{imu,gps,baro,mag,pitot/airspeed_mps,pitot/differential_pressure_pa}`.
2. All 5 sensors produce finite, non-NaN output on their topics — confirmed by direct echo.
3. IMU orientation frame convention confirmed correct at near-t=0 via a paused-server single/double-step test (sec 4.1).
4. Pitot wind-response and native-sensor wind-isolation both confirmed live (sec 3): pitot tracks a commanded 5 m/s wind exactly (`airspeed_mps=5.0`, `differential_pressure_pa=15.31`); native GPS `velocity_east/north/up` unaffected by the same wind command.
5. Two open findings requiring `gazebo-testing`/`validation` follow-up, reported honestly rather than hidden or silently "fixed" (per CLAUDE.md — never hide missing information): `BARO_ALTITUDE_RESPONSE_REVIEW_REQUIRED` (sec 4.3) and `MAG_FIELD_SOURCE_REVIEW_REQUIRED` (sec 4.4).

This live log is a **smoke-test-level confirmation performed by this task to satisfy the task's own "confirm each one loads/produces live output" requirement** — it is not a substitute for `gazebo-testing`'s own systematic live validation suite (e.g. a proper `SENSOR_LIVE_OUTPUT_TEST` / `WIND_PITOT_RESPONSE_TEST` with recorded, reproducible results under version control), which is the next step in the standard workflow.

---

## 9. Follow-up disposition of the two sec-8 open findings (2026-08-27)

- **`BARO_ALTITUDE_RESPONSE_REVIEW_REQUIRED` (sec 4.3): CLOSED, BENIGN.** `gazebo-testing` independently live-validated the barometer across a 40 m continuous descent and two independently-spawned altitudes 290 m apart, finding it tracks the ISA pressure-altitude formula to ~0.3% slope / ~1 Pa absolute from the first available sample — directly contradicting the original single-sample smoke test, which is now understood to have been a startup-timing artifact of one manual `gz topic echo`, not a sensor defect. Full evidence: `docs/test_results/2026-08-27_sensor_model_sitl_preparation.md` sec 4. No `controls-integration` change was made or needed.
- **`MAG_FIELD_SOURCE_REVIEW_REQUIRED` (sec 4.4): RESOLVED, FIXED.** `gazebo-testing` confirmed the finding as a real, reproducible defect in the installed native `gz-sim-magnetometer-system` (`docs/test_results/2026-08-27_sensor_model_sitl_preparation.md` sec 5). `controls-integration` root-caused it as far as possible without source access (see sec 4.4 above for the full gdb-verified evidence trail) and replaced the native sensor with a custom `FalconV2Magnetometer` plugin, live re-verified by this agent (magnitude ratio 1.0000000000000002 vs. declared field; independent cross-check against the same-instant native IMU attitude matches to <0.001% relative error). `gazebo-testing` is expected to perform one more targeted re-check of just the magnetometer before this goes to `validation`.

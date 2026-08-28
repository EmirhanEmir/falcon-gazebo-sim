# FALCON V2 — ArduPlane SITL Transport & Actuator-Mapping Design

**Owner:** `controls-integration`
**Task:** `ARDUPLANE_SITL_TRANSPORT_AND_ACTUATOR_MAPPING_VALIDATION` (2026-08-27).

This document is the design/provenance record for the official `ardupilot_gazebo`
`ArduPilotPlugin` block added to `model/model.sdf`, and for the new
ArduPilot-bridge-only IMU relay sensor added alongside it. It does **not**
duplicate `SITL_PARAM_MIGRATION.md` (real `.param` → SITL `.parm`
classification, servo function table, provenance of `falcon_v2_sitl.parm`'s
values) — cross-reference that document for anything not repeated here.

Scope boundary honored throughout (task's own hard limit): transport +
command-mapping only. No closed-loop flight mode, no arming persisted to any
checked-in file, no PID retuning, no change to `plugins/actuators/`,
`plugins/propulsion/`, `plugins/aerodynamics/`, `plugins/sensors/` internals,
no change to mass/CG/inertia/aerodynamic coefficients, no change to the real
`.param` or to `falcon_v2_sitl.parm`.

---

## 1. What was added to `model/model.sdf`

1. A second, ArduPilot-bridge-ONLY IMU sensor (`imu_sensor_ardupilot`, under
   `base_link`, immediately after the pre-existing `imu_sensor`) — see §2 for
   why this is required and could not be avoided by only wiring `<imuName>`
   correctly.
2. One `<plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">` block
   (model level), containing transport settings, one frame override
   (`gazeboXYZToNED`), and 7 `<control type="COMMAND">` blocks (2 aileron + 2
   elevator + 1 rudder + 2 throttle).

Both additions are purely additive — no existing link/joint/mass/CG/inertia/
collision/visual element, and no existing `<plugin>` block (aerodynamics/
propulsion/actuators/wind/pitot/magnetometer), was modified. Full inline
provenance comments are in `model/model.sdf` itself at each block; this
document explains the *reasoning* behind those comments and the live
evidence behind each one, in one place.

---

## 2. IMU wiring — a real bridge gap found and fixed (not "just wire the name")

The task's pre-verified ground truth stated `<imuName>` "IS the sensor
ArduPilotPlugin actually reads" and implied wiring the correct scoped name
would be sufficient. **This was checked live and found incomplete**, for a
reason not evident from reading `ArduPilotPlugin.cc` alone without also
checking gz-sensors' actual runtime topic-assignment behavior:

- `ArduPilotPlugin::PreUpdate()`'s IMU-topic resolution (`LoadImuSensors()`
  only stores the `<imuName>` string; the actual topic subscription happens
  once, lazily, in `PreUpdate()`) computes its subscribe topic as
  `gz::sim::scopedName(imuEntity, ecm) + "/imu"` — **it never reads the
  sensor's own SDF `<topic>` element.** Confirmed by direct source read
  (`ArduPilotPlugin.cc` lines ~1160–1169).
- The pre-existing `imu_sensor` (prior stage, `SENSOR_MODEL_AND_
  ARDUPLANE_SITL_PREPARATION`) declares a **custom** `<topic>
  /model/falcon_v2/sensors/imu</topic>`.
- **Empirically confirmed this task** (`gz topic -l` against the real model
  in `tests/gazebo/worlds/falcon_v2_sensors_selftest_world.sdf`, and again
  against a throwaway probe sensor with no `<topic>` in a scratch copy of
  the model): a gz-sim sensor with an explicit `<topic>` publishes **only**
  on that custom topic — never additionally on the auto-generated default
  (`/world/<world>/model/<model>/link/<link>/sensor/<name>/imu`, which is
  exactly what `scopedName()+"/imu"` computes). So `ArduPilotPlugin`'s
  computed subscribe topic can **never** match `imu_sensor`'s real topic,
  no matter what (correct) `<imuName>` string is given — `imuMsgValid`
  would never become `true` and `CreateStateJSON()` would silently never
  build a packet at all (not "wrong numbers" — **no FDM data sent, ever**).

Per this task's explicit instruction, the prior-stage `imu_sensor` block is
**not** modified to route around this. Instead a second sensor,
`imu_sensor_ardupilot`, was added: same mount point / same
`orientation_reference_frame` (`CUSTOM`, `parent_frame="world"`,
`custom_rpy="0 0 0"`) / same update rate / same zero-noise mode as
`imu_sensor` (it is a transport-layer duplicate of an already-validated
sensor, not a second physical sensor model), but **deliberately with no
`<topic>` element**, so gz-sim's own Sensors system assigns it exactly the
auto-generated default topic `ArduPilotPlugin` independently computes via
the same `scopedName()` call — confirmed to match by construction and
confirmed empirically (`gz topic -l` showed
`/world/<world>/model/falcon_v2/link/base_link/sensor/imu_sensor_ardupilot/imu`
being published, and the live `arduplane` run received real ATTITUDE/
RAW_IMU/gyro data through it — see §6).

**`<imuName>` scoped-name form — also live-verified, not copied blindly
from the zephyr example.** `ArduPilotPlugin.cc` calls
`entitiesFromScopedName(imuName, ecm, /*_relativeTo=*/model.Entity())`.
Per `gz::sim::Util.hh`'s own documented contract, when `_relativeTo` is
given, the scoped name must **not** repeat that entity's own name.
Zephyr's own reference example (`<imuName>zephyr::imu_link::imu_sensor
</imuName>`) is not a counter-example: `zephyr_with_ardupilot` (the model
carrying the plugin) `<include>`s a **nested** sub-model literally named
`zephyr`, so `"zephyr"` there is an intermediate model name, not the
plugin-carrying (`_relativeTo`) model's own name. FALCON V2 is a single
flat model (`<model name="falcon_v2">`, no nesting), so the correct form is
2 segments, not 3. **Confirmed live, this task:** `falcon_v2::base_link::
imu_sensor_ardupilot` and `world::falcon_v2::base_link::
imu_sensor_ardupilot` both failed (`imu_sensor [...] not found, abort
ArduPilot plugin.`); `base_link::imu_sensor_ardupilot` (used in the SDF)
and the bare `imu_sensor_ardupilot` both resolved with no error. Value used:
`base_link::imu_sensor_ardupilot`.

---

## 3. Conversion-chain statement (full pipeline, one place)

```
ArduPlane internal servo demand (SERVO_OUT, centidegrees, function-specific
sign convention)
  -> SERVOx_REVERSED / SERVOx_MIN/MAX/TRIM applied entirely inside
     ArduPlane's own SRV_Channel output code (upstream of, and
     independent from, everything below)                              [ArduPlane-side, real .param]
  -> PWM microseconds, sent as the JSON FDM packet's per-channel PWM array
  -> ArduPilotPlugin::UpdateMotorCommands() (confirmed exact from source):
       raw_cmd = clamp((pwm - servo_min) / (servo_max - servo_min), 0, 1)
       cmd     = multiplier * (raw_cmd + offset)
  -> gz.msgs.Double published on <cmd_topic> (type=COMMAND; confirmed by
     direct source read this NEVER drives the joint via
     force/velocity/position PID — ApplyMotorForces() publishes then
     unconditionally `continue`s for COMMAND-type channels)
  -> the EXISTING, UNMODIFIED Gazebo actuator/propulsion command topic
     (/model/falcon_v2/actuators/*/cmd_rad or
      /model/falcon_v2/propulsion/{left,right}/throttle_cmd)
  -> plugins/actuators/ActuatorSystem.cc / plugins/propulsion/
     PropulsionSystem.cc, exactly as already validated in the prior stages
     (physical-angle-radians / [0,1] throttle, rate-limited/effort-bounded
     servo dynamics, real ESC/motor/prop physics) — UNCHANGED by this task
  -> real joint motion / real RPM -> AerodynamicsSystem / thrust, exactly
     as already validated
```

**`SERVOx_REVERSED` is not double-applied — two separate layers.** This
exact risk was flagged in `SITL_PARAM_MIGRATION.md` §2/§14/§16.
`SERVOx_REVERSED` (real `.param`: SERVO1/4=1, SERVO2/3/5=0) is applied
entirely **inside ArduPlane**, before the JSON packet is ever built — by
the time our `<control>` block sees a PWM value, that correction has
already happened and is invisible to us; all we ever see is a PWM number
in `[servo_min, servo_max]`. Our `<control>` block's own `multiplier`/
`offset` is a **second, independent** conversion (PWM-domain →
physical-radians-domain for our own joint/topic convention) — it is not
"fixing the sign again," it is the *only* place a PWM→radians conversion
happens at all. Confirmed no code path combines or cancels the two.

---

## 4. Frame overrides

### 4.1 Body frame (`modelXYZToAirplaneXForwardZDown`) — left at compiled default

Compiled default: `Pose3d(0,0,0,GZ_PI,0,0)` (pure 180° roll about body X).
For FALCON V2's own genuinely-FLU body frame (CLAUDE.md; every joint/link
in `model/model.sdf` is authored in FLU), this maps
`(X-fwd, Y-left, Z-up) -> (X-fwd, Y-right, Z-down)`, i.e. exactly
FLU → FRD. Verified by direct computation with the exact `gz.math7`
`Quaterniond`/`RotateVector` API the plugin itself calls (not merely
trusted from the in-source comment, which this task's own brief correctly
flagged as unreliable): rotating unit vectors `+X`(fwd)/`+Y`(left)/`+Z`(up)
by this quaternion gives `+X`(fwd)/`-Y`(right)/`-Z`(down) exactly. **No
override needed or added.**

### 4.2 World frame (`gazeboXYZToNED`) — override REQUIRED, contrary to the task's own "CONFIRMED FAVORABLE" hint

The task's own brief flagged this hint as suspicious and asked for live
verification rather than trust. **The hint did not survive verification.**

The compiled default is the **same** `Pose3d(0,0,0,GZ_PI,0,0)` as §4.1. A
pure 180°-roll-about-X is **not** a correct ENU→NED conversion: it flips
Gazebo world Y/Z sign but leaves Gazebo's own `+X`-world axis mapped into
NED's **North** slot — i.e. it silently treats "East" as "North."
Confirmed numerically (`gz.math7.Quaterniond`, exact plugin API): rotating
a pure `+X`-world ("East") unit vector by the default gives `(1,0,0)`,
i.e. it reads out as **pure NED-North**, not NED-East — the swap the
task's brief suspected.

**Fix used:** `<gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>`
— identical to `ardupilot_gazebo`'s own `zephyr_with_ardupilot` reference
example's value for this **same** tag (its own reasoning for choosing it
is not recorded in that example, but the value itself is objectively the
correct one for a standard ENU world, independent of zephyr's own
non-standard body frame — the two overrides in that example serve two
unrelated purposes, see the source-code-analysis note in `model/model.sdf`
for the full reasoning). Verified numerically with this override: a pure
`+X`-world vector → `(0,1,0)` = pure NED-**East**; a pure `+Y`-world
vector → `(1,0,0)` = pure NED-**North**. Correct.

This project's own world files all declare
`<spherical_coordinates><world_frame_orientation>ENU</...>` explicitly
(confirmed by grep across `tests/gazebo/worlds/*.sdf`), so this correction
applies to every world this model is loaded into, not a one-off.

**Live (not just offline-math) confirmation — §6.2.**

---

## 5. GPS / baro / mag / airspeed architecture finding

Investigated per the task's explicit instruction, on both sides of the
bridge:

**Gazebo/plugin side (`ArduPilotPlugin.cc`):**
- The `gpsSensor` member is **literally commented out** of the private data
  class (`// public: sensors::GpsSensorPtr gpsSensor;`), and
  `LoadGpsSensors()`'s entire body is one large `/* NOT MERGED IN MASTER
  YET ... */` comment block — the function does **nothing** when called.
  There is no `gpsName`/`baroName`/`magName` SDF parameter anywhere in
  `ArduPilotPlugin.hh`.
- `CreateStateJSON()`'s emitted JSON keys (confirmed by reading the writer
  code in full) are exactly: `timestamp`, `imu{gyro,accel_body}`,
  `position`, `quaternion`, `velocity`, `rng_1..rng_6`, `windvane
  {direction,speed}` (only if an `<anemometer>` sensor is configured — it
  is not, this stage), `no_time_sync`, `no_lockstep`. **There is no
  `"airspeed"`, `"latitude"/"longitude"/"altitude"`, `"wind_vel"`, or any
  GPS/baro/mag key at all.**

**ArduPilot side (`libraries/SITL/SIM_JSON.cpp`, `libraries/SITL/SIM_JSON.h`,
`libraries/AP_HAL_SITL/sitl_airspeed.cpp`, `libraries/AP_Airspeed/
AP_Airspeed_SITL.cpp`):**
- The JSON parser's `received_bitmask` sets `LATITUDE|LONGITUDE|ALTITUDE`,
  `AIRSPEED`, and `WIND_VEL` bits **only if the corresponding JSON keys are
  present in the packet** (a `keytable` maps JSON key strings to bitmask
  flags). Since our packet never contains those keys, these bits are
  **never** set, so:
  - Position is always taken from the raw `"position"` (local NED-meters-
    from-origin) branch, never lat/lon/alt — consistent with what we send.
  - Airspeed is always taken from the **`else` branch**: `wind_ef.zero()`
    ("wind is not supported yet for JSON sim, assume zero for now" — the
    comment is ArduPilot's own, not this project's), `velocity_air_ef =
    velocity_ef - wind_ef` (i.e. **pure ground-truth velocity, wind never
    subtracted**), then `update_eas_airspeed()`. This EAS value is later
    converted into `AP::sitl()->state.airspeed_raw_pressure[i]` by the
    **generic, backend-agnostic** `SITL_State::_update_airspeed()`
    (`libraries/AP_HAL_SITL/sitl_airspeed.cpp`), via
    `diff_pressure = eas^2 / ARSPD_RATIO` (default `ARSPD_RATIO=2.0`,
    matching the "next-stage calibration" caveat already flagged in
    `SITL_PARAM_MIGRATION.md` §4/§16).
  - GPS/baro/compass: synthesized entirely on the ArduPilot side from the
    same `position`/`velocity`/`quaternion` ground truth (confirmed
    directly for the compass: `SIM_JSON.cpp` line ~608, "as the model does
    not provide mag feild we calculate it from position and attitude").

**Conclusion (industry-standard FDM-backend behavior, not a defect):**
GPS, barometer, magnetometer, and airspeed are **100% ArduPilot-side
synthesis from ground-truth position/velocity/quaternion** — none of this
project's native/custom Gazebo sensors (`gps_sensor`, `baro_sensor`, the
custom `FalconV2Magnetometer`, the custom `FalconV2Pitot`) feed SITL
through this bridge in any way. This is exactly how ArduPilot's JSON/FDM
backend has always worked (every JSON-backed vehicle in `ardupilot_gazebo`
behaves this way) — it is **not** something this project's SDF/plugin
wiring introduced or could fix by wiring an SDF tag differently. Wind is
similarly **not** communicated to SITL's airspeed synthesis at all in this
bridge version (`WIND_VEL` key never emitted). The `windvane` JSON key
**is** wired (via `<anemometer>`, not configured this stage) but feeds the
**separate** `AP_WindVane` subsystem (`WNDVN_TYPE=11`), not `AP_Airspeed`
— confirmed by reading both consumer sites; conflating the two would be
wrong.

**IMU is the one channel confirmed genuinely sensor-in-the-loop** — see §2
and §6.1: `imu.gyro`/`imu.accel_body` are populated straight from the
subscribed Gazebo IMU sensor message, not recomputed from truth.

**What this means for "sensor → SITL validation" this stage:** GPS/baro/
mag/airspeed cannot be, and were not, validated as "live-feeding SITL"
through this bridge, because they structurally do not. Their prior-stage
validation (native-sensor correctness, in-Gazebo) stands on its own and is
unaffected; it simply does not extend through this transport layer. IMU is
the only channel where this bridge's live behavior depends on this
project's own sensor plugin output.

---

## 6. Live sanity check (this task's own quick check — not a substitute for `gazebo-testing`'s rigorous pass)

Performed against `tests/gazebo/worlds/falcon_v2_sensors_selftest_world.sdf`
(unmodified, real model), `GZ_SIM_SYSTEM_PLUGIN_PATH` covering all 6
project plugin build dirs plus `ardupilot_gazebo/build` (matches the
pattern already used by `tests/gazebo/scripts/*_lib.py`), and
`arduplane` launched as `-w -M json -O 0,0,0,0 --defaults
config/ardupilot/falcon_v2_sitl.parm -I 0 --speedup 1` (no `--serial0`
override needed — default TCP port 5760). Connected with `pymavlink`
(`tcp:127.0.0.1:5760`).

### 6.1 Connection / transport / IMU

- `model/model.sdf` loads with **zero errors/warnings** from any plugin,
  including `ArduPilotPlugin` (confirmed after fixing the `<imuName>`
  scoped-name form per §2).
- `HEARTBEAT` received (`type=1` FIXED_WING, `autopilot=3` ARDUPILOTMEGA,
  disarmed throughout this entire session — no arming command was ever
  sent).
- `ATTITUDE`, `RAW_IMU`, `SCALED_IMU2`, `GPS_RAW_INT`, `GPS_GLOBAL_ORIGIN`,
  `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT`, `EKF_STATUS_REPORT`,
  `SCALED_PRESSURE`, `WIND`, `VFR_HUD`, `SERVO_OUTPUT_RAW`, etc. all
  streamed with finite, sane-looking values — confirms the full JSON
  round-trip (Gazebo → arduplane → mavlink) is alive end-to-end.

### 6.2 World-frame override — live confirmation (not just offline math)

Used the `/world/<world>/set_pose` gz service to teleport `falcon_v2` to a
known, level (identity-orientation) pose at a large offset, then read the
very next `LOCAL_POSITION_NED`/`ATTITUDE` message:

| Teleport (Gazebo world frame) | Reported `LOCAL_POSITION_NED` | Reported `ATTITUDE` yaw | Verdict |
|---|---|---|---|
| `x=+200` (pure "East" per ENU labeling), `y=0` | `N(x)=-0.28`, `E(y)=369.1` (climbing toward 200 as the position update settles) | `90.27°` | **CONFIRMS**: East offset reads as NED-East, not North |
| `x=0`, `y=+200` (pure "North") | `N(x)=200.01`, `E(y)=0.01` | `90.0°` (orientation unchanged, as expected — only position was teleported) | **CONFIRMS**: North offset reads as NED-North, not East |

Both results match §4.2's offline derivation exactly (yaw=90° for a
level, nose-`+X-world` aircraft is the textbook-correct NED yaw for
"pointing East"; a `+X`-world position offset reads out on the NED-East
axis, a `+Y`-world offset on the NED-North axis). This is the strongest
evidence in this pass: it exercises the *actual* compiled plugin binary
end-to-end, not just a re-execution of its math offline.

(Aircraft attitude was **not** expected to stay level indefinitely in this
free-running, unpowered/unstabilized, disarmed session — gravity/aero act
immediately and the aircraft tumbles within a couple of real seconds,
exactly the same documented phenomenon already recorded for this project's
native-IMU verification in `model/model.sdf`'s `imu_sensor` header comment.
This is why the position/attitude readings above were taken at the
**first** message immediately following each teleport, not after a
multi-second wait.)

### 6.3 Servo/motor command channels

- `SERVO_OUTPUT_RAW` shows **exactly** the real `falcon_v2_sitl.parm`
  `SERVOx_TRIM` values once RC input was supplied via
  `RC_CHANNELS_OVERRIDE` (a standard, non-arming diagnostic mechanism):
  `servo1=1500, servo2=1500, servo3=1000, servo4=1500, servo5=1000` —
  matching SERVO1/2/4 TRIM=1500 and SERVO3/5 TRIM=1000 (=MIN) exactly.
  Per this task's own mapping formula, `raw_cmd=0.5` at PWM=1500 (MIN=800/
  MAX=2200) → `cmd=0` rad for all 5 non-throttle surfaces, and
  `raw_cmd=0` at PWM=1000 (MIN=MAX-domain 1000/2000) → `cmd=0.0` for both
  throttles — i.e. this **is** the neutral-output state the task asked
  this stage to verify ("Manual/neutral output verification only").
- **Not resolved this session:** the `/model/falcon_v2/actuators/*/cmd_rad`
  and `/model/falcon_v2/propulsion/{left,right}/throttle_cmd` gz topics
  themselves showed **no publish activity** (`gz topic -e`, up to 10s
  windows, zero messages, despite an advertised publisher/subscriber pair
  being present) during this disarmed session, even though
  `SERVO_OUTPUT_RAW` (a separate mavlink telemetry mirror) showed live,
  correct values. This is consistent with `ArduPilotPlugin`'s `outputReady`
  gate (`_pwm[channel] != 0` in `UpdateMotorCommands()`) never becoming
  true for these channels while disarmed — most likely ArduPlane's own
  `hal.rcout()`-level safety-switch gate (`BRD_SAFETYENABLE`, which affects
  the actual PWM array sent to the JSON FDM backend) holding raw PWM at 0
  independent of what `SERVO_OUTPUT_RAW` telemetry reports. A runtime-only
  `PARAM_SET BRD_SAFETYENABLE=0` (not persisted to any file) plus a fresh
  `RC_CHANNELS_OVERRIDE` did not change this within the session time
  available. **Not investigated further this pass** — resolving the exact
  arm/safety-switch sequence needed to observe live nonzero `cmd_rad`/
  `throttle_cmd` values is squarely within `gazebo-testing`'s mandate for
  the "full, rigorous live mapping/sign acceptance tests" that follow this
  task (their harness/expertise for controlled arming is the appropriate
  place for this, not a channel/topic/multiplier defect on this side —
  channel resolution, topic registration, and joint-name resolution all
  succeeded with zero errors for all 7 `<control>` blocks). Flagged
  explicitly: `REVIEW_REQUIRED` for `gazebo-testing`.

---

## 7. Channel / topic / sign table

**SUPERSEDED MAGNITUDE, 2026-08-28 — see §11.** The multiplier *values* in
this section's table and the `±10°`/`±0.349` figures in the prose below and
in §7.1-§7.3 are the **original, now-historical**
`ARDUPLANE_SITL_TRANSPORT_AND_ACTUATOR_MAPPING_VALIDATION`-stage figures,
kept here verbatim for the record because the **sign** reasoning in
§7.1-§7.3 (which of `+0.3490658504`/`-0.3490658504` goes on which surface)
is unchanged and still exactly how the live `model/model.sdf` signs are
derived. The **magnitude** is not current: `model/model.sdf`'s actual
`<multiplier>` values are now `±1.5707963268` (`= ±pi/2`, realizing
`±45°`, not `±10°`) — see §11 for the full derivation, rationale, and
live re-confirmation. Read every `0.3490658504`/`±10°` figure below as
"same sign, superseded magnitude — current magnitude is
`1.5707963268`/`±45°`, §11."

| ArduPlane function | SERVOx (real `.parm`) | channel | Gazebo topic | multiplier (ORIGINAL, superseded — current value in §11) | offset | servo_min/max | Expected physical result of an ArduPlane-increasing (toward SERVO_MAX) command, PRE-REVERSED-by-ArduPlane |
|---|---|---|---|---|---|---|---|
| Aileron | SERVO1, REVERSED=1 | 0 | `.../left_aileron/cmd_rad` | `+0.3490658504` (now `+1.5707963268`) | `-0.5` | 800/2200 | see §7.1 |
| Aileron | SERVO1, REVERSED=1 | 0 | `.../right_aileron/cmd_rad` | `-0.3490658504` (now `-1.5707963268`) | `-0.5` | 800/2200 | see §7.1 |
| Elevator | SERVO2, REVERSED=0 | 1 | `.../left_elevator/cmd_rad` | `+0.3490658504` (now `+1.5707963268`) | `-0.5` | 800/2200 | see §7.2 |
| Elevator | SERVO2, REVERSED=0 | 1 | `.../right_elevator/cmd_rad` | `+0.3490658504` (now `+1.5707963268`) | `-0.5` | 800/2200 | see §7.2 |
| Rudder | SERVO4, REVERSED=1 | 3 | `.../rudder/cmd_rad` | `-0.3490658504` (now `-1.5707963268`) | `-0.5` | 800/2200 | see §7.3 |
| ThrottleLeft | SERVO3, REVERSED=0 | 2 | `.../propulsion/left/throttle_cmd` | `+1.0` (unchanged, out of this task's scope) | `0.0` | 1000/2000 | 1:1, `[0,1]`, no sign question |
| ThrottleRight | SERVO5, REVERSED=0 | 4 | `.../propulsion/right/throttle_cmd` | `+1.0` (unchanged, out of this task's scope) | `0.0` | 1000/2000 | 1:1, `[0,1]`, no sign question |

`multiplier=±0.3490658504 rad = ±2×10°` (ORIGINAL, superseded — see §11
for the current `±1.5707963268 rad = ±2×45°` value and full derivation):
chosen so ArduPlane's full raw PWM swing `[servo_min,servo_max]` mapped
onto a **±10° target range** — a `controls-integration` **design
choice**, not a manufacturer-sourced figure. Was deliberately not the
full `±45°` `MECHANICAL_ACTUATOR_LIMIT_V1` at the time (the original
task's own instruction warned against assuming that). `±10°` was the
*only other* documented boundary anywhere in this project's source of
truth at the time
(`docs/source_of_truth/controls/actuator_v1_config.yaml`'s
`aerodynamic_validity_note`: XFLR5 control-derivative sweeps only
directly cover ≈±10° at small signal — though, as later confirmed in
§11.4, the wide-deflection lookup tables already covered the full ±45°
domain even before this section was written). This kept ArduPlane's
manual-control authority inside the small-signal-validated envelope; the
joint's own `±45°` mechanical clamp (`ActuatorSystem.cc`, untouched)
remained available as a hard ceiling this mapping never reached.
**RESOLVED, §11 (2026-08-28):** per explicit task instruction, the bridge
now commands the full `±45°` mechanical range; `REVIEW_REQUIRED` above is
closed by that section.

### 7.1 Aileron — pre-registered expectation (OPPOSITE signs, per task requirement)

Verified physical facts used (`CONTROLS.md` §10, `aero_v1_config.yaml`
`control_mapping`, all `VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST`):
- Positive `left_aileron_joint`/`right_aileron_joint` angle → TE **up**,
  identically for both joints (they share the same +Y-dominant axis sense,
  confirmed *not* a true sign-mirror).
- `delta_a_aero = 0.5*(theta_right - theta_left)`; a differential command
  `theta_left=-8°(TE down), theta_right=+8°(TE up)` measured live
  `Mx=+4.877 N·m`. In this project's FLU body frame, positive `Mx`
  (right-hand rule about `+X`) moves `+Y`(left wing) toward `+Z`(up) and
  `-Y`(right wing) toward `-Z`(down) — i.e. **left wing up / right wing
  down = roll right** (standard aviation sense, and matches the textbook
  aileron mechanism: TE-down increases lift, TE-up decreases it).

Given `SERVO1_REVERSED=1`, standard `SRV_Channel` convention means a
"more aileron" (roll-right) ArduPlane demand maps to PWM decreasing toward
`SERVO_MIN` (not `SERVO_MAX` — the REVERSED flag flips which end
corresponds to "positive" demand on the real airframe's own installed
servo). With `offset=-0.5`, PWM→`SERVO_MIN` gives `raw_cmd→0`, `cmd→
-0.5×multiplier`.

**Chosen multiplier signs** (`left=+0.349`, `right=-0.349`) give, at
`raw_cmd→0` (i.e. an ArduPlane roll-right demand): `cmd_left = -0.175 rad`
(TE **down**), `cmd_right = +0.175 rad` (TE **up**) — **matching the
Mx>0/roll-right combination above exactly.**

**Pre-registered hypothesis for `gazebo-testing`:** commanding ArduPlane
roll-right (e.g. RC1 above trim with `SERVO1_REVERSED=1` in effect) is
expected to produce **left aileron TE down, right aileron TE up, and a
physical roll to the right** (right wing down). A mismatch here means
either the multiplier signs above need swapping, or (less likely, already
independently verified in a prior stage) the underlying `CONTROLS.md` §10
sign facts are wrong. **This absolute direction (which of the two
`0.349`/`-0.349` signs is correct for which joint) was *not* independently
re-derived from ArduPlane's own real-airframe `SERVO1_REVERSED=1`
calibration** (that value was tuned for the real servo horn/linkage
geometry, which this project has no data on and does not model) — it is
this document's own reasoned, testable choice per the task's explicit
"pre-register, don't assert" instruction, `REVIEW_REQUIRED` until
`gazebo-testing` confirms it live.

### 7.2 Elevator — pre-registered expectation (SAME sign, per task requirement)

Verified: positive `left_elevator_joint`/`right_elevator_joint` angle →
TE **up**, identically for both (same-sign pair, matching the elevator's
own physically-convenient common-mode-for-pure-pitch requirement).
`delta_e_aero = -0.5*(theta_left+theta_right)`; commanding
`theta_left=theta_right=+8°` (TE up, both sides) measured live
`My=-1.4856 N·m`, confirmed **nose-up** in this plugin's own established
FLU convention (`My<0 ⇔ nose-up`).

Given `SERVO2_REVERSED=0`, a "pitch up" ArduPlane demand maps to PWM
increasing toward `SERVO_MAX`. With `offset=-0.5`, PWM→`SERVO_MAX` gives
`raw_cmd→1`, `cmd→+0.5×multiplier`. **Chosen multiplier (`+0.349` on
both blocks — same sign, as required)** gives `cmd_left=cmd_right=
+0.175 rad` (TE up, both sides) at a pitch-up demand — matching the
nose-up result above.

**Pre-registered hypothesis:** commanding ArduPlane pitch-up (RC2 above
trim) is expected to produce **both elevator halves TE up, and a nose-up
pitching moment.** `REVIEW_REQUIRED` until confirmed live, same caveat as
§7.1 about not having the real airframe's own elevator-linkage geometry.

### 7.3 Rudder — pre-registered expectation (single surface)

Verified: positive `rudder_joint` angle → TE toward `-Y` (right, FLU).
`theta_rudder=+8°` measured live `Mz=-0.446 N·m`, confirmed **nose-right**.

Given `SERVO4_REVERSED=1`, a "yaw right" ArduPlane demand maps to PWM
decreasing toward `SERVO_MIN`. With `offset=-0.5`, PWM→`SERVO_MIN` gives
`raw_cmd→0`, `cmd→-0.5×multiplier`. **Chosen multiplier `-0.349`** gives
`cmd_rudder=+0.175 rad` (TE right) at a yaw-right demand — matching the
nose-right result above.

**Pre-registered hypothesis:** commanding ArduPlane yaw-right (RC4 above
trim) is expected to produce **rudder TE right, and a nose-right yawing
moment.** `REVIEW_REQUIRED` until confirmed live, same caveat as §7.1/7.2.

### 7.4 Throttle

1:1, `[0,1]`, no sign ambiguity — confirmed against
`PropulsionSystem.cc::OnThrottleLeft/Right()`'s own
`std::clamp(msg.data(),0.0,1.0)` domain by direct source read. Full
per-motor independence preserved (no mixing/averaging introduced anywhere
in this chain), matching the already-validated engine-out/asymmetric-
thrust behavior.

---

## 8. Launch sequence for `gazebo-testing`

See `tests/gazebo/scripts/launch_ardupilot_sitl.sh` (this task's own
minimal launch helper — coordinate with `gazebo-testing` before either
side duplicates it). Summary:

```
# 1. Gazebo (needs all 6 project plugin build dirs + ardupilot_gazebo/build
#    on GZ_SIM_SYSTEM_PLUGIN_PATH; ardupilot_gazebo/build is already on the
#    path via ~/.bashrc on this machine, confirmed):
gz sim -r tests/gazebo/worlds/falcon_v2_sensors_selftest_world.sdf

# 2. ArduPlane SITL (either form confirmed workable this task):
#    (a) direct binary, no sim_vehicle.py needed for a manual/neutral check:
/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane \
  -w -M json -O 0,0,0,0 \
  --defaults /home/emirhan/Desktop/FalconV2/config/ardupilot/falcon_v2_sitl.parm \
  -I 0 --speedup 1
#    (b) sim_vehicle.py, exactly as the task suggested - CONFIRMED WORKABLE
#    this task (sim_vehicle.py's vehicleinfo lookup falls back gracefully
#    for an unrecognized frame string, printing a harmless
#    "WARNING: no config for frame (json)" and using ret["model"]="json"):
Tools/autotest/sim_vehicle.py -v ArduPlane -f json \
  --add-param-file=/home/emirhan/Desktop/FalconV2/config/ardupilot/falcon_v2_sitl.parm
#    Cleaner alternative avoiding the warning, same effect:
Tools/autotest/sim_vehicle.py -v ArduPlane --model json \
  --add-param-file=/home/emirhan/Desktop/FalconV2/config/ardupilot/falcon_v2_sitl.parm
```

Both `fdm_addr=127.0.0.1`/`fdm_port_in=9002` (Gazebo/plugin side) and
ArduPlane's own JSON backend compiled defaults (`target_ip="127.0.0.1"`,
`control_port=9002`, confirmed from `SIM_JSON.h`) match — **no address/port
arguments are required** for a same-host launch.

---

## 9. Open items

- `REVIEW_REQUIRED` (§6.3): live nonzero `cmd_rad`/`throttle_cmd` values
  were not observed this session while disarmed; likely ArduPlane's own
  safety-switch gate on `hal.rcout()`. `gazebo-testing` should resolve
  this as part of their own controlled-arming test sequence.
- `REVIEW_REQUIRED` (§7.1/7.2/7.3): the absolute multiplier signs are this
  document's own reasoned, testable pre-registration (per the task's
  explicit "pre-register, don't assert" instruction) — not independently
  re-derived from the real airframe's own servo-linkage geometry (not
  available in this repository). `gazebo-testing`'s
  `AILERON_TEST`/`ELEVATOR_TEST`/`RUDDER_TEST`/`ROLL_RESPONSE_TEST`/
  `PITCH_RESPONSE_TEST`/`YAW_RESPONSE_TEST` are the authoritative check.
- `DATA_REQUIRED`: real airframe servo-horn/pushrod linkage geometry (would
  let a future stage double-check `SERVOx_REVERSED`'s real-world meaning
  against this project's own joint-sign convention analytically, instead
  of only empirically).
- **SUPERSEDED, §11 (2026-08-28):** the commanded-range design choice was
  originally `±10°` (§7); per explicit task instruction it is now `±45°`
  (`USER_CONFIRMED_MECHANICAL_TRAVEL_ASSUMPTION`, §11.2), matching the
  actuator's own mechanical ceiling exactly. Still not a manufacturer
  figure / measured linkage ratio — flagged for `validation` to weigh,
  same as every other `V1_PROVISIONAL`/`ASSUMPTION` value in this
  project.
- Wind is not communicated into SITL's airspeed synthesis by this bridge
  version at all (§5) — out of scope this stage (task explicitly limited
  this pass to transport + command-mapping), flagged for whoever later
  needs wind-in-the-loop SITL behavior (would require either an
  `<anemometer>` wiring into `AP_WindVane` — a different subsystem than
  `AP_Airspeed` — or an upstream `ardupilot_gazebo`/ArduPilot-side change
  neither of which exists today).

---

## 10. Bug-fix follow-up (2026-08-28) — IMU gyro/accel FLU→FRD frame fix

**CRITICAL defect found live by `gazebo-testing`** (independent
`VelocityControl`/`cmd_vel`-based per-axis probe, full evidence:
`docs/test_results/2026-08-27_ardupilot_sitl_transport_actuator_mapping_validation.md`
§5), fixed by `controls-integration` the same task.

**Root cause (confirmed by direct source read of the installed
`ardupilot_gazebo` plugin, `ArduPilotPlugin.cc::CreateStateJSON()`):**
`angularVel`/`linearAccel` are read straight from the subscribed IMU sensor
message and written into the JSON `gyro`/`accel_body` fields **verbatim,
with zero rotation applied** — `modelXYZToAirplaneXForwardZDown` (the
FLU→FRD body-frame transform used elsewhere in this same function) is
composed and applied **only** to `position`/`velocity`/`quaternion` in
this plugin version, never to the IMU message. With `imu_sensor_ardupilot`
mounted at zero rotation relative to `base_link` (§1/§2 above), its raw
`angular_velocity()`/`linear_acceleration()` output was genuine
unconverted FLU, reaching ArduPlane's FRD-assuming AHRS/EKF unmodified —
roll (body X) happened to read correctly because FLU +X = FRD +X (no flip
needed on that one axis), but pitch/yaw (Y/Z) were silently unflipped.

**Fix applied:** `imu_sensor_ardupilot`'s own `<pose>` in `model/model.sdf`
now carries a 180°-about-X rotation (`<pose degrees="true">0.168309 0
0.100000 180 0 0</pose>`, position unchanged) — numerically identical to
`modelXYZToAirplaneXForwardZDown`'s own compiled default
(`Pose3d(0,0,0,GZ_PI,0,0)`) and to `plugins/sensors/Frames.hh`'s
already-tested `FluFrdSwap()`/`kQFluToFrdRot` (reused, not re-derived).
Verified correct against gz-sim8/gz-sensors8 source (not assumed) before
applying: for a non-link entity such as a sensor, `Physics.cc`'s
"body angular velocity"/"body linear acceleration" blocks compute the
reported local-frame vector as
`entityWorldPose.Rot().RotateVectorReverse(entityWorldVec)`, where
`entityWorldPose` is the sensor's **fully-composed** world orientation
(parent link rotation chained with the sensor's own `<pose>` rotation) —
so a rotation on this `<pose>` genuinely rotates the raw gyro/accel this
sensor reports, exactly like a physically re-oriented chip would; and
`gz-sensors ImuSensor::Update()` rotates gravity into this same composed
frame for the accelerometer's specific-force term, so `accel_body` gets
the identical, gravity-consistent conversion. Full derivation and
citations recorded inline in `model/model.sdf` next to
`imu_sensor_ardupilot`.

**Live verification (`controls-integration`, own independent per-axis
probe, same `VelocityControl`/`cmd_vel` method `gazebo-testing` used,
`tests/gazebo/worlds/falcon_v2_ardupilot_sitl_test_world.sdf`,
unmodified):**

| Commanded (Gazebo FLU body frame, 0.5 rad/s) | `RAW_IMU` gyro (mrad/s) | `ATTITUDE` rate (rad/s) | Expected FLU→FRD | Verdict |
|---|---|---|---|---|
| roll +X | `xgyro≈+500` | `rollspeed≈+0.501` | `+0.5` (no flip) | matches — roll unaffected, still correct |
| pitch +Y | `ygyro≈-499` | `pitchspeed≈-0.499` | `-0.5` (flip) | matches — now correctly flipped |
| yaw +Z | `zgyro≈-499` | `yawspeed≈-0.499` | `-0.5` (flip) | matches — now correctly flipped |

All three axes now show the correct FRD sign relationship; the roll axis
(already correct pre-fix) was not broken by this change.

**Scope of the fix:** touches only `imu_sensor_ardupilot`'s `<pose>`
rotation in `model/model.sdf`. The original `imu_sensor` (prior-stage
validated, feeds `plugins/sensors/` diagnostics via its own
software-side `Frames.hh::FluFrdSwap()` conversion where needed) is
untouched. No `<control>` block, no aero/propulsion/actuator physics, no
`falcon_v2_sitl.parm`/real `.param`, no mass/CG/inertia was touched.

**Arming-blocker theory (§9's `REVIEW_REQUIRED` on live throttle
testing) — NOT resolved by this fix, reported plainly:** `gazebo-testing`
hypothesized the persistent `"AHRS: DCM Roll/Pitch inconsistent"` prearm
failure blocking arming (and therefore the motor/throttle live tests) was
likely caused by this same gyro frame bug. **Checked live post-fix
(passive observation only — one connect + prearm-status cycle, extended
to a second and third cycle at +30s/+60s to distinguish a converging
transient from a persistent bias; no force-arm, no `ARMING_*` parameter
change):** the warning **still appears**, at essentially the same
magnitude (`~51-54°`) and does **not** shrink over a 90-second soak — a
stable, non-converging offset, not a settling transient. Source read of
`AP_AHRS::attitudes_consistent()` (`AP_AHRS.cpp`) shows this specific
check compares the primary (EKF3) attitude quaternion against the
**separate legacy DCM backend's** own attitude estimate — both consumed
from the same, now-correctly-converted single IMU stream — so a
persistent large divergence between the two backends is not explained by
the fix in this section and is most likely an independent issue. Not
root-caused further here (out of this fix's scope, and this task was
explicitly instructed not to force arming or pursue the arming path
itself — that remains `gazebo-testing`'s mandate next). Flagged
`OPEN`/`REVIEW_REQUIRED` for `gazebo-testing`/`validation`: the
motor/throttle tests blocked in §9 should **not** be assumed unblocked by
this fix alone.

---

## 11. Control-surface travel scaling: `+/-10deg` -> `+/-45deg` (2026-08-28) — `ARDUPLANE_CONTROL_SURFACE_TRAVEL_SCALING_VALIDATION`

**What changed:** the `<multiplier>` on all 5 non-throttle `<control>`
blocks (§7's table) in `model/model.sdf` was changed from
`+/-0.3490658504` rad to `+/-1.5707963268` rad (`= +/-pi/2`), **sign
preserved exactly per surface** (`left_aileron=+`, `right_aileron=-`,
`left_elevator=+`, `right_elevator=+`, `rudder=-` — none flipped).
`<offset>-0.5</offset>` and `<servo_min>800</servo_min>`/
`<servo_max>2200</servo_max>` (the real `SERVOx_MIN/MAX` values) are
**unchanged**. The 2 throttle `<control>` blocks (channels 2/4) are
**untouched**.

### 11.1 The old `+/-10deg` value — retroactively labeled `PROVISIONAL_INTERFACE_LIMIT`, now REMOVED

§7's original rationale (multiplier chosen so ArduPlane's full raw PWM
swing mapped onto `+/-10deg`, deliberately short of the joint's own
`+/-45deg` mechanical ceiling, to stay inside the aero model's
then-only-`+/-10deg`-validated envelope) is now retagged
`PROVISIONAL_INTERFACE_LIMIT` for the historical record and **no longer
applies** — see §11.3 for why the constraint that motivated it no longer
exists. The `+/-10deg` restriction has been fully removed from the
bridge; the joint's own `+/-45deg` mechanical clamp
(`docs/source_of_truth/controls/actuator_v1_config.yaml` `min_angle_rad`/
`max_angle_rad`, `model/model.sdf`'s `MECHANICAL_ACTUATOR_LIMIT_V1` joint
`<limit>` — confirmed still `+/-0.7853981634` rad, **unchanged** by this
task, live-grepped in `model/model.sdf` lines 989-990/1047-1048/
1136-1137/1191-1192/1281-1282) is now the sole binding constraint on
ArduPlane's commanded range.

### 11.2 New value — `USER_CONFIRMED_MECHANICAL_TRAVEL_ASSUMPTION`, not a measured linkage ratio

`+/-45deg` as the new ArduPlane-commanded range is tagged
`USER_CONFIRMED_MECHANICAL_TRAVEL_ASSUMPTION`: it is the project owner's
directed choice (this task's own instruction) to make the bridge command
the full documented mechanical travel range, **not** a real
servo-horn/pushrod linkage ratio, a measured real-airframe PWM->surface-
angle calibration, or a flight-tested maximum control throw — none of
those exist in this repository (same `DATA_REQUIRED` gap already noted in
§9 for the real linkage geometry). The `+/-45deg` figure itself
(`MECHANICAL_ACTUATOR_LIMIT_V1`) is `USER_CONFIRMED_MECHANICAL_CAPABILITY`
per `actuator_v1_config.yaml` (what the servo/joint can physically
reach) — this section's new tag is about a *different* fact: that
ArduPlane's own SERVOx PWM range is now assumed/directed to command that
full range, not some smaller fraction of it. Both remain
non-manufacturer, non-flight-tested figures.

### 11.3 Multiplier derivation (re-derived independently, not copied blindly)

Formula, unchanged from §3 (confirmed exact from `ArduPilotPlugin.cc`'s
`UpdateMotorCommands()`):

```
raw_cmd = clamp((pwm - servo_min) / (servo_max - servo_min), 0, 1)   in [0, 1]
cmd     = multiplier * (raw_cmd + offset)                             offset = -0.5
```

Since `raw_cmd in [0, 1]` and `offset = -0.5`, `(raw_cmd + offset) in
[-0.5, +0.5]` — **the realized output range is `[-0.5*multiplier,
+0.5*multiplier]`, i.e. HALF the multiplier's own numeric value, not the
multiplier itself.** This was true of the old value too (verified as part
of this task, not merely inherited): `0.5 * 0.3490658504 = 0.1745329252`
rad `= 10.0000000°` exactly — confirming §7's old `+/-10deg` label was
already numerically correct, and that reading the raw multiplier
`0.3490658504` itself as "the deflection in radians" would have been
wrong (it is `2x` the realized deflection).

Applying the same relationship for a target `+/-45deg`:
`target_rad = 45 * (pi/180) = 0.7853981633974483` rad;
`multiplier = 2 * target_rad = 1.5707963267948966 rad = pi/2`, rounded to
`1.5707963268` for the SDF (10 significant figures, matching this
project's existing constant-precision convention, e.g. `Cldr_per_rad`,
`0.3490658504` itself). Re-verified: `0.5 * 1.5707963268 =
0.78539816340`, i.e. `45.0000000°` exactly.

**Explicit non-substitution warning (re-derived and confirmed, per this
task's own caution):** using `0.7853981634` (45° expressed directly in
radians) as the multiplier — the naive substitution — would be **wrong**:
it would realize only `0.5 * 0.7853981634 = 0.3926990817` rad `=
22.5000000°`, i.e. HALF of the intended `+/-45deg`. The correct value is
`pi/2`, not `pi/4`.

### 11.4 TXT source re-read / lookup-table cross-check (read-only, no aero edits)

Re-read `docs/source_of_truth/aerodynamics/control_surface_analysis/
FALCON_V2_CONTROL_SURFACE_WIDE_DEFLECTION_RESULTS.txt` (499 lines) in full
and spot-checked it against `docs/source_of_truth/aerodynamics/
aero_v1_config.yaml`'s `control_surface_lookup` block (the table
`plugins/aerodynamics/AeroModel.hh`'s `InterpLinear()` actually reads at
runtime):

- **Value transcription, exact match, no discrepancy found.** Compared
  every one of the 15 breakpoints (`[-45,-35,-25,-15,-10,-5,-2,0,+2,+5,
  +10,+15,+25,+35,+45]` deg) for: elevator `dCL`/`dCD`/`dCm` (TXT "Primary
  elevator baseline-difference table"), aileron `Cl`/`CD_full` (TXT "Full
  aileron wide-deflection sweep"), rudder `CY`/`CD_full` (TXT "Full rudder
  wide-deflection sweep"). All values in `aero_v1_config.yaml` reproduce
  the TXT file's numbers digit-for-digit at every breakpoint checked,
  including at the ±15°/±25°/±45° representative points this task asked
  to spot-check specifically (e.g. elevator `dCm(+45)=-0.78429`, aileron
  `Cl(+25)=+0.18001`, rudder `CY(-35)=-0.05601` — all exact matches). No
  action taken (read-only cross-check, no aero coefficient touched, none
  needed to be).
- **Confidence labeling preserved, not silently dropped.** Both
  `aero_v1_config.yaml` (`control_surface_lookup.confidence_bands`:
  `HIGH_CONFIDENCE_SMALL_SIGNAL <=10deg`,
  `MEDIUM_CONFIDENCE_NONLINEAR_REFERENCE <=25deg`,
  `LOW_CONFIDENCE_HIGH_DEFLECTION_REFERENCE <=45deg`) and
  `plugins/aerodynamics/AeroModel.hh`'s own inline comments (lines
  ~176-178, same three labels/thresholds) carry the identical
  HIGH/MEDIUM/LOW convention as the TXT file's own "Confidence convention"
  section (lines 31-40), word-for-word threshold-for-threshold. The TXT's
  own explicit caveat ("Large-deflection XFLR5/VLM results are not treated
  as direct real-flight truth... separated flow/stall/hinge-region
  nonlinearities are not fully modeled") is likewise preserved in
  `aero_v1_config.yaml`'s header comment above the lookup block
  ("explicitly NOT REAL_FLIGHT_VALIDATED... separated-flow/stall/hinge-
  region nonlinearities are not fully modeled by VLM").
- **High-deflection drag rise confirmed present, not zeroed.** Aileron
  `CD_full` in the YAML: `CD(0)=0.01513`, `CD(+/-25)=0.06757`,
  `CD(+/-35)=0.11664`, `CD(+/-45)=0.18174` — exact match to the TXT and
  to the TXT's own stated `CD(45)/CD(0)~12` ratio. `AeroConfig::Prepare()`
  (`AeroModel.hh` line ~261) differences this against the `delta_a=0` row
  to build `ctrlAileDCD`, which `ComputeAero()` (line 618) reads via
  `InterpLinear()` and adds as an additive drag correction — confirmed
  wired end-to-end, not dropped. Same pattern confirmed for rudder
  `CD_full`/`ctrlRuddDCD` (line 264/623). Elevator's `dCD` is loaded
  directly (already baseline-differenced in the TXT itself) and read via
  `ctrlElevDCD`/line 627 — also confirmed non-zero and wired.
- **Domain bound / no-extrapolation behavior confirmed unaffected by this
  task's bridge change.** `aero_v1_config.yaml`'s
  `control_surface_lookup.domain_bound_deg: 45.0` and `InterpLinear()`'s
  clamp-to-nearest-breakpoint (not extrapolate) behavior were already in
  place from the prior `HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION` stage
  (2026-08-26) and are untouched here — this task's bridge-scaling change
  only means the aero model's already-existing full `+/-45deg` input
  domain is now actually reachable end-to-end from ArduPlane, not that the
  aero model's own domain changed.
- **No genuine discrepancy found requiring escalation to `aerodynamics`.**

### 11.5 Own live sanity check (`controls-integration`, this task — not a substitute for `gazebo-testing`'s rigorous pass)

Method: `tests/gazebo/worlds/falcon_v2_zero_g_world.sdf` (unmodified,
real model with all its own attached plugins, gravity zeroed — a
test-harness isolation choice per that world's own header, not an
aircraft physics change), driving the 5 real `.../actuators/*/cmd_rad`
topics directly via `tests/gazebo/scripts/actuator_lib.py`'s
`ActuatorCommander` (republished every tick) with the exact `cmd_rad`
values the new bridge formula (§11.3) produces at 5 representative PWM
samples (`800, 1000, 1500, 2000, 2200` — spanning the full documented
`SERVOx_MIN/TRIM/MAX` range), then reading back each joint's real
position via the ECM (`Joint.position()`). This exercises the joint side
of the chain directly (`ArduPilotPlugin` itself was not re-exercised here
— it remains independently blocked while disarmed per §6.3/§9, squarely
`gazebo-testing`'s mandate to resolve; the bridge's PWM->`cmd_rad`
arithmetic itself needs no live Gazebo run to verify, since
`ArduPilotPlugin.cc`'s formula was already confirmed byte-exact from
source in the prior stage and re-confirmed by hand in §11.3).

1.0 s settle per step (5-sample sweep, all 5 surfaces simultaneously):
targets from `+/-10deg`-superseding values up to the new `+/-45deg`
extremes were approached to within `0.24-1.85 deg` — consistent with,
not evidence against, this project's own already-documented actuator
settling behavior (`actuator_v1_config.yaml`'s `sp_weight_derivation_note`
explicitly documents a `~1.6 s` 99%-settling time at `spWeightB=0.7` for a
fresh large step, dominated by the intentionally slow `Ti=0.5s` integral
tail — a pre-existing, already-validated `ACTUATOR_SERVO_MODEL_V1`
characteristic, not something this task's multiplier change introduced
or should compensate for).

Supplementary run with a longer 3.0 s settle window at the two extremes
plus neutral (matching that documented ~1.6s 99%-settling time with
margin):

| PWM (all 5 surfaces) | Surface | Target | Actual (3.0s settle) | Error |
|---|---|---|---|---|
| 800 | left_aileron | -45.0000° | -44.9712° | 0.029° |
| 800 | right_aileron | +45.0000° | +44.9712° | 0.029° |
| 800 | left_elevator | -45.0000° | -44.9713° | 0.029° |
| 800 | right_elevator | -45.0000° | -44.9713° | 0.029° |
| 800 | rudder | +45.0000° | +44.9713° | 0.029° |
| 2200 | left_aileron | +45.0000° | +44.9319° | 0.068° |
| 2200 | right_aileron | -45.0000° | -44.9319° | 0.068° |
| 2200 | left_elevator | +45.0000° | +44.9322° | 0.068° |
| 2200 | right_elevator | +45.0000° | +44.9322° | 0.068° |
| 2200 | rudder | -45.0000° | -44.9322° | 0.068° |
| 1500 (neutral) | all 5 | 0.0000° | +/-0.0285-0.0287° | 0.029° |

**Confirms:** (a) no residual `~10deg` clamp anywhere — all 5 surfaces
reach well beyond the old ceiling in both directions; (b) the full
`+/-45deg` range is genuinely reachable at the real joint, converging to
within `<0.07deg` of target given adequate settle time; (c) neutral
(PWM=1500) still maps to `~0deg` (within the same `<0.03deg` residual
band, no bias introduced); (d) aileron L/R opposite preserved (`-44.97°`/
`+44.97°` at PWM=800); (e) elevator L/R same preserved (`-44.9713°` /
`-44.9713°`, identical); (f) rudder sign unchanged (`+44.9713°` at
PWM=800, same sign pattern as the pre-change `-0.3490658504`-multiplier
behavior, only rescaled). No overshoot observed at either extreme
(consistent with the actuator's own critically-damped, `zeta=1.0` P-term
design plus conditional-integration anti-windup — expected, not
incidental).

(Note: this quick check's own `setup_env()` — reused from
`actuator_lib.py`, unmodified — only adds the actuator/propulsion/
aerodynamics plugin build dirs to `GZ_SIM_SYSTEM_PLUGIN_PATH`, not the
wind/pitot/magnetometer sensor plugin dirs; `gz sim` logged 3 harmless
"Failed to load system plugin" errors for those unrelated plugins during
this run. This does not affect control-surface joint actuation — those
plugins are sensor/disturbance-force plugins unrelated to the actuator/
joint chain being checked here — and is not a defect in this task's
change; `gazebo-testing`'s own harness already sets up the full plugin
path correctly for its rigorous pass.)

### 11.6 Clamp-layer agreement — confirmed

Before this task: bridge `+/-10deg` (binding/tighter) vs. actuator
mechanical `+/-45deg` (non-binding, never reached from ArduPlane). After
this task: bridge `+/-45deg` (§11.3, re-derived and live-confirmed §11.5)
vs. actuator mechanical `+/-45deg` (`actuator_v1_config.yaml`
`min/max_angle_rad = +/-0.7853981634` rad, confirmed **unchanged** — this
task touched no actuator config file — and confirmed still present
verbatim in `model/model.sdf`'s 5 joint `<limit>` blocks, §11.1). **Both
layers now agree exactly at `+/-45deg`**, the intended outcome: the
actuator's own mechanical clamp is now the sole binding constraint (by
construction, since `cmd` is a monotonic linear function of `raw_cmd` and
`raw_cmd` is itself hard-clamped to `[0,1]` inside
`ArduPilotPlugin::UpdateMotorCommands()` before the multiplier/offset is
even applied — no separate redundant clamp element exists or was added to
the `<control>` block schema, none needed).

### 11.7 What this section does NOT change

No closed-loop flight mode, no PID/autotune, no aero coefficient/lookup
table value, no mass/CG/inertia, no propulsion model, no sensor plugin,
no throttle `<control>` block, no actuator PID/rate/effort config, no
`falcon_v2_sitl.parm`/real `.param` file. Scope was strictly the 5 surface
`<control>` blocks' `<multiplier>` values in `model/model.sdf` plus this
documentation. `gazebo-testing`'s own rigorous live matrix (9-point
command sweep x 3 surfaces, breakpoint regression at `9.9/10/10.1deg`
etc., actuator tracking, TXT-vs-runtime aero comparison) remains the
authoritative check and was not attempted or substituted for here.

# FALCON V2 - Actuator/Servo Gazebo Sim Harmonic Plugin

Owner: `controls-integration` specialist agent. Task: `ACTUATOR_SERVO_MODEL_V1`.

Scope: the physical actuator/servo command chain for the 5 control-surface
joints (`left_aileron_joint`, `right_aileron_joint`, `left_elevator_joint`,
`right_elevator_joint`, `rudder_joint`) - actuator command (rad) -> finite-
rate, finite-effort servo dynamics -> real Gazebo joint motion. This
REPLACES the previous test-script-only direct position injection as the
actuation mechanism for these 5 joints. It does NOT change how
`plugins/aerodynamics/AerodynamicsSystem.cc` reads those same 5 joint
positions (unmodified - still `gz::sim::Joint(e).Position(_ecm)` every
tick), does not change any aerodynamic coefficient/control-sign mapping, and
does not implement ArduPlane/MAVLink/SITL/TECS - this plugin ends at the
physical actuator interface. See
`docs/source_of_truth/controls/actuator_v1_config.yaml` for every numeric
constant this plugin consumes and its full provenance/derivation record.

## Architecture

```
raw command (rad, gz.msgs.Double, hold-last-valid)
  -> ClampCommand()          [mechanical travel limit, +/-45 deg V1]
  -> RateLimitSetpoint()     [servo slew-rate limit, V1_PROVISIONAL]
  -> PdEffort()              [effort-bounded PD torque, V1_PROVISIONAL gains]
  -> gz::sim::Joint::SetForce()   [REAL joint-space torque command]
  -> gz-sim's own physics engine integrates the REAL joint motion
```

**Why torque/effort-based, not a kinematic position write:** unlike
`plugins/propulsion/PropulsionSystem.cc`'s COSMETIC prop-joint visual sync
(which deliberately uses `Joint::ResetPosition()` specifically to AVOID
engaging real joint dynamics, since that joint's motion is not itself a
physical actuation mechanism), this actuator IS the real, sole actuation
mechanism for these 5 joints. `Joint::SetForce()` commands a genuine
joint-space generalized force/torque that gz-sim's own physics engine
integrates through the joint's real (currently placeholder, see caveat
below) inertia - continuous position/velocity by construction, never an
instantaneous teleport.

**Two independent guarantees, not one:**
1. `ActuatorModel.hh`'s `StepSetpoint()` (`ClampCommand()` +
   `RateLimitSetpoint()`) is a pure, deterministic kinematic function:
   provably, by construction, the internal setpoint can never move by more
   than `max_rate * dt` in one tick, and the clamped target can never leave
   `[min_angle, max_angle]`. This is what the 8 required pure-math
   self-tests exercise directly (Gazebo-independent).
2. `ActuatorSystem.cc` ALSO calls `gz::sim::Joint::SetVelocityLimits()` and
   `SetEffortLimits()` once at `Configure()` (genuine physics-engine-level
   hard constraints on the REAL joint, using the SAME `max_rate`/`max_effort`
   values) - defense in depth, not reliance on PD-gain tuning alone. See
   `ActuatorModel.hh`'s header comment and
   `docs/source_of_truth/controls/actuator_v1_config.yaml`'s
   `placeholder_inertia_note` for why this is necessary given the current
   near-massless placeholder link mass on these 5 joints.

**PLACEHOLDER-INERTIA CAVEAT (read before trusting a live joint-response
test):** `model/model.sdf`'s 5 control-surface links currently carry
`TEMPORARY_NUMERICAL_MASS = 0.001 kg` each (`geometry-structure`'s own
documented near-massless numerical placeholder, not a physical claim). This
project's own standalone self-test (`test/actuator_model_selftest.cc`,
`CLOSED_LOOP_TRACKING_INFO` section) found that a naively-derived PD gain
set (sized only from `max_effort`/a reference tracking-error angle,
independent of inertia) produces a closed-loop natural frequency far above
what a 1 kHz per-tick torque update (`tests/gazebo/worlds/*.sdf`'s
`<max_step_size>=0.001`) can stably control, GIVEN this tiny placeholder
inertia - confirmed by the self-test as a genuine discrete-control
undersampling problem, not a mere numerical-integration artifact. V1's
`kp`/`kd` (see `actuator_v1_config.yaml`) were therefore re-derived directly
against a dt-compatible closed-loop bandwidth using the current placeholder
inertia as the reference plant. **This is expected to need re-derivation
once real per-surface component mass/inertia data replaces the placeholder**
- flagged clearly for whoever does that in the future, and worth
`gazebo-testing` watching for chatter/oscillation in the live diagnostics
regardless (the `SetVelocityLimits`/`SetEffortLimits` hard constraints are
the backstop if it does occur).

**NO-LOAD-HOLDING-CAPABILITY CAVEAT:** `plugins/aerodynamics/
AerodynamicsSystem.cc` applies its computed aerodynamic force/moment to
`base_link` only - it does NOT apply any hinge-moment reaction torque back
onto these 5 joints (per-surface hinge-moment data is `DATA_REQUIRED`
throughout this project). There is currently NO simulated aerodynamic load
pushing back on these joints. A test that commands a deflection and confirms
it "holds" is validating ONLY this actuator's own dynamics against zero
external load - see `actuator_v1_config.yaml`'s
`no_opposing_aero_torque_caveat` for the full statement. Do not cite such a
test as evidence of real load-holding capability.

## Files

- `ActuatorModel.hh` - pure math core (`ClampCommand`, `RateLimitSetpoint`,
  `StepSetpoint`, `PdEffort`, and a self-test-only reference single-DOF
  integrator `IntegrateJointStep`). Zero Gazebo dependency at all (not even
  gz-math - every quantity here is a scalar), matching
  `plugins/propulsion/PropulsionModel.hh`'s "fully Gazebo-independent"
  precedent, one step further than `plugins/aerodynamics/AeroModel.hh`
  (which still needs `gz::math::Vector3d`). Used by both the real plugin and
  the standalone self-test below.
- `ActuatorSystem.hh` / `ActuatorSystem.cc` - the `gz-sim8` System plugin
  (`ISystemConfigure` + `ISystemPreUpdate`). Owns 5 persistent
  (`commandRad`, `setpointRad`) state pairs, subscribes to 5 per-surface
  command topics, reads each joint's actual position/velocity from the ECM,
  calls `ActuatorModel.hh`, drives each joint via `Joint::SetForce()`, and
  publishes diagnostics.
- `test/actuator_model_selftest.cc` - standalone, Gazebo-independent
  self-test executable exercising `ActuatorModel.hh` directly (no Gazebo
  instance required). Covers the 8 required pure-math tests
  (`ACTUATOR_NEUTRAL_START_TEST` through `ACTUATOR_NO_TELEPORT_TEST`) plus
  one bonus clamp check and one informational closed-loop sanity check. Not
  a substitute for `gazebo-testing`'s live-Gazebo suite - the remaining 4
  required tests (`ACTUAL_JOINT_STATE_FEEDS_AERO_TEST`,
  `LEFT_RIGHT_ELEVATOR_MAPPING_REGRESSION`,
  `LEFT_RIGHT_AILERON_MAPPING_REGRESSION`, `RUDDER_MAPPING_REGRESSION`) need
  live Gazebo with both the actuator and aerodynamics plugins running
  together and are `gazebo-testing`'s job next.
- `CMakeLists.txt` - builds both of the above.
- `../../docs/source_of_truth/controls/actuator_v1_config.yaml` - the
  structured dataset (source of truth) the plugin loads at `Configure()`
  time. No coefficient is hardcoded in the C++ source.

## Dependencies

Same as `plugins/aerodynamics/`/`plugins/propulsion/` (already present in
this project's Gazebo Sim Harmonic environment):

- `gz-sim8` (8.14.0), `gz-plugin2` (2.0.4, component `register`),
  `gz-transport13` (13.5.0), `gz-msgs10` (10.3.2), `gz-math7` (7.6.0, needed
  by `ActuatorSystem.cc` for `gz::math::Vector2d` in the
  `SetVelocityLimits`/`SetEffortLimits` calls), `yaml-cpp` (0.7.0).
- `actuator_model_selftest` depends on the C++ standard library ONLY -
  `ActuatorModel.hh` has no gz-math/gz-sim dependency at all.

No dependency outside this list was added.

## Build

```bash
cd /home/emirhan/Desktop/FalconV2/plugins/actuators
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

Produces `build/libFalconV2Actuators.so` and `build/actuator_model_selftest`.
`build/` is git-ignored - rebuild locally, do not commit artifacts.

## Run the standalone self-test (no Gazebo instance needed)

```bash
./build/actuator_model_selftest
```

Exits 0 if every `[PASS]`/`[FAIL]` check passes, 1 otherwise (`[INFO]` lines
are informational, not pass/fail). Covers:
`ACTUATOR_NEUTRAL_START_TEST`, `ACTUATOR_POSITIVE_STEP_TEST`,
`ACTUATOR_NEGATIVE_STEP_TEST`, `ACTUATOR_RATE_LIMIT_TEST`,
`ACTUATOR_POSITIVE_45_LIMIT_TEST`, `ACTUATOR_NEGATIVE_45_LIMIT_TEST`,
`ACTUATOR_OVERCOMMAND_CLAMP_TEST` (+/-), `ACTUATOR_NO_TELEPORT_TEST` - 9
`[PASS]` checks (the overcommand test prints both signs separately) plus one
bonus `PD_EFFORT_CLAMP_TEST` and one informational
`CLOSED_LOOP_TRACKING_INFO` section.

## Run in a live Gazebo instance

`model/model.sdf` already contains the
`<plugin filename="FalconV2Actuators" name="falcon_v2_actuators::ActuatorSystem">`
block with its `<config_path>` and `<diagnostics_rate_hz>` parameters. To
make the compiled `.so` discoverable:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/emirhan/Desktop/FalconV2/plugins/actuators/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -s -r --iterations 20 tests/gazebo/worlds/falcon_v2_freefall_world.sdf
```

Command a surface (physical-angle-radians interface, V1's only implemented
wire format):

```bash
gz topic -t /model/falcon_v2/actuators/left_aileron/cmd_rad -m gz.msgs.Double -p "data: 0.1745"
gz topic -t /model/falcon_v2/actuators/right_aileron/cmd_rad -m gz.msgs.Double -p "data: -0.1745"
```

Diagnostics can be observed live via:
```bash
gz topic -e -t /model/falcon_v2/actuators/diagnostics
```
(35 fields = 7 fields x 5 surfaces, order printed by `Configure()`'s `gzmsg`
line and documented in
`docs/source_of_truth/controls/actuator_v1_config.yaml`'s
`command_interface.diagnostics_field_order_per_surface`: `cmd_rad,
target_clamped_rad, setpoint_rad, actual_angle_rad, actual_rate_rad_s,
target_clamp_active, effort_clamp_active`, repeated for left_aileron,
right_aileron, left_elevator, right_elevator, rudder in that order.)

This README documents build/run steps and the actuator architecture only -
it intentionally does not duplicate the full numeric provenance/derivation
record, which lives in
`docs/source_of_truth/controls/actuator_v1_config.yaml` and
`docs/source_of_truth/controls/CONTROLS.md`.

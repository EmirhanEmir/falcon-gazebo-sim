# FALCON V2 — Actuator/Servo Plugin

Gazebo Sim Harmonic System plugin implementing the physical actuator chain for
the five control-surface joints: `left_aileron_joint`, `right_aileron_joint`,
`left_elevator_joint`, `right_elevator_joint`, `rudder_joint`.

Scope: actuator command → finite-rate, finite-effort servo dynamics → real
Gazebo joint motion. It changes no aerodynamic coefficient or control-sign
mapping, and implements no ArduPlane/MAVLink/TECS logic — this plugin ends at
the physical actuator interface.

## Architecture

```
raw command (rad, gz.msgs.Double, hold-last-valid)
  → ClampCommand()        [mechanical travel limit, ±45°]
  → RateLimitSetpoint()   [servo slew-rate limit]
  → PidEffort()           [effort-bounded PID torque]
  → gz::sim::Joint::SetForce()   [real joint-space torque command]
  → the physics engine integrates the real joint motion
```

**Why torque-based, not a kinematic position write:** a position write would
teleport the joint and bypass the physics engine entirely, making rate and
effort limits unenforceable. `SetForce()` produces genuine joint dynamics.

The PID gains are derived against each surface's inertia about its own hinge
axis (parallel-axis corrected, per surface). Integral action is used rather
than gravity feedforward so that any constant disturbance is rejected without
modelling it; anti-windup freezes accumulation on target clamp, on
same-direction effort saturation, and on rate saturation.

Because the control-surface link masses in `model.sdf` are a deliberate
near-massless numerical placeholder, the plugin additionally applies
`SetVelocityLimits()` / `SetEffortLimits()` at `Configure()` as
physics-engine-level hard constraints, using the same max-rate and max-effort
values.

## Files

| File | Purpose |
| ---- | ------- |
| `ActuatorModel.hh` | Pure-math core: clamp, rate limit, PID with anti-windup. No Gazebo dependency at all. |
| `ActuatorSystem.hh` / `.cc` | The `gz-sim8` System plugin: five command subscriptions, five persistent setpoint states, torque application, diagnostics. |
| `CMakeLists.txt` | Build definition for `libFalconV2Actuators.so`. |

Every numeric constant comes from `config/actuator_v1_config.yaml`.

## Dependencies

- `gz-sim8`, `gz-plugin2` (component `register`), `gz-transport13`,
  `gz-msgs10`, `gz-math7`
- `yaml-cpp` (via `libyaml-cpp-dev`)
- CMake ≥ 3.16, a C++17 compiler

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Produces `build/libFalconV2Actuators.so`. `build/` is git-ignored.

## Runtime interface

Each surface is commanded independently in physical-angle radians:

```bash
gz topic -t /model/falcon_v2/actuators/left_aileron/cmd_rad  -m gz.msgs.Double -p "data:  0.1745"
gz topic -t /model/falcon_v2/actuators/right_aileron/cmd_rad -m gz.msgs.Double -p "data: -0.1745"
```

Command topics: `/model/falcon_v2/actuators/{left_aileron, right_aileron,
left_elevator, right_elevator, rudder}/cmd_rad`. A surface defaults to 0 rad
(neutral) if never commanded, and holds its last valid command otherwise.

Diagnostics on `/model/falcon_v2/actuators/diagnostics` are 35 fields —
7 per surface, in the order left_aileron, right_aileron, left_elevator,
right_elevator, rudder: `cmd_rad, target_clamped_rad, setpoint_rad,
actual_angle_rad, actual_rate_rad_s, target_clamp_active,
effort_clamp_active`.

```bash
gz topic -e -t /model/falcon_v2/actuators/diagnostics
```

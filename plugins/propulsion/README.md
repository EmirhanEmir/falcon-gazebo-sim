# FALCON V2 — Propulsion Plugin

Gazebo Sim Harmonic System plugin implementing the twin motor/ESC/propeller
chain: 2 × SunnySky X2820 860KV motors driving 2 × APC 13x6.5E propellers on
a 4S pack.

Scope: propulsion physics only. No aerodynamic coefficients, no control
actuation, no autopilot logic.

## Model chain

```
throttle → ESC/motor electrical → motor torque
         → rotor angular dynamics (explicit-Euler state, integrated each tick)
         → RPM → advance ratio J → APC Ct(J,RPM) / Cp(J,RPM)
         → thrust + propeller aerodynamic torque → reaction torque
         → force/moment applied to base_link at the real hub
```

The chain is never collapsed into `throttle × maximum_thrust`. The same omega
state drives the propeller joints' angular velocity, so the visual spin can
never diverge from the RPM actually producing forces.

## Files

| File | Purpose |
| ---- | ------- |
| `PropulsionModel.hh` | Pure-math core: Kt/Ke, electrical model, rotor ODE step, advance ratio, APC table parsing and 2-D Ct/Cp interpolation, thrust/power/torque. No Gazebo dependency. |
| `PropulsionSystem.hh` / `.cc` | The `gz-sim8` System plugin: maintains the two omega states, applies the wrench at each hub, drives the prop joints, publishes diagnostics. |
| `CMakeLists.txt` | Build definition for `libFalconV2Propulsion.so`. |

Parameters come from `config/propulsion_v1_config.yaml`. The Ct/Cp data comes
from `config/propulsion_data/apc_13x65e_parsed.csv`, mechanically parsed from
the official APC performance file `PER3_13x65E.dat` — see
`config/propulsion_data/PROVENANCE.md`. No Ct/Cp value is invented: every
value is either a tabulated APC value or a linear interpolation between two.

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

Produces `build/libFalconV2Propulsion.so`. `build/` is git-ignored.

## Runtime interface

| Topic | Type | Direction |
| ----- | ---- | --------- |
| `/model/falcon_v2/propulsion/left/throttle_cmd` | `gz.msgs.Double` | subscribe, clamped to [0,1] |
| `/model/falcon_v2/propulsion/right/throttle_cmd` | `gz.msgs.Double` | subscribe, clamped to [0,1] |
| `/model/falcon_v2/wind` | `gz.msgs.Vector3d` | subscribe |
| `/model/falcon_v2/propulsion/diagnostics` | `gz.msgs.Double_V` | publish |

Throttle defaults to 0 if never commanded. Under ArduPlane SITL these topics
are driven by the ArduPilotPlugin control channels in `model/model.sdf`.

Diagnostics are 28 fields — 14 per motor, left then right:
`throttle, current_A, Qmotor, omega_ownframe, rpm, J, Ct, Cp, thrust_N,
Qprop, interpClamped, rpmCapActive, negCurClamped, curLimited`.

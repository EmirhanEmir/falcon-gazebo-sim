# FALCON V2 — Aerodynamics Plugin

Gazebo Sim Harmonic System plugin implementing the aircraft's aerodynamic
force and moment model.

Scope: aerodynamic physics only. It reads the five control-surface joint
positions; it does not command them, and it implements no propulsion or
autopilot logic.

## Files

| File | Purpose |
| ---- | ------- |
| `AeroModel.hh` | Pure-math core: alpha/beta, dynamic pressure, coefficient build-up, wind→body force rotation, control-surface lookup. Depends only on `gz-math7`. |
| `AerodynamicsSystem.hh` / `.cc` | The `gz-sim8` System plugin: reads Gazebo state, calls `ComputeAero()`, applies the force/moment to `base_link`, publishes diagnostics. |
| `CMakeLists.txt` | Build definition for `libFalconV2Aerodynamics.so`. |

Every coefficient is loaded at `Configure()` time from
`config/aero_v1_config.yaml`. No coefficient is hardcoded in C++.

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

Produces `build/libFalconV2Aerodynamics.so`. `build/` is git-ignored.
`scripts/build_plugins.sh` builds this and the other four plugins together.

## Runtime interface

`model/model.sdf` contains the
`<plugin filename="FalconV2Aerodynamics" name="falcon_v2_aero::AerodynamicsSystem">`
block with its `<config_path>`, `<air_density>`, `<wind_topic>` and
`<diagnostics_rate_hz>` parameters.

| Topic | Type | Direction |
| ----- | ---- | --------- |
| `/model/falcon_v2/wind` | `gz.msgs.Vector3d` | subscribe (world-frame air-mass velocity, m/s) |
| `/model/falcon_v2/aerodynamics/diagnostics` | `gz.msgs.Double_V` | publish |

Diagnostics field order: `V, alpha_rad, beta_rad, qbar, CL, CD, CY, Cl, Cm, Cn`.
Observe them live with:

```bash
gz topic -e -t /model/falcon_v2/aerodynamics/diagnostics
```

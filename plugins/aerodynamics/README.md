# FALCON V2 - Aerodynamics Gazebo Sim Harmonic Plugin

Owner: `aerodynamics` specialist agent. Task: `AERODYNAMICS_V1_IMPLEMENTATION`.

Scope: aerodynamic force/moment physics only. No propulsion, no
control-actuation (reads joint positions, does not write them), no
ArduPilot/SITL. See `docs/source_of_truth/aerodynamics/AERODYNAMICS.md` for
the full architecture, derivations, and provenance record.

## Files

- `AeroModel.hh` — pure math core (no Gazebo dependency beyond gz-math7).
  Used by both the real plugin and the standalone self-test below.
- `AerodynamicsSystem.hh` / `AerodynamicsSystem.cc` — the `gz-sim8` System
  plugin (`ISystemConfigure` + `ISystemPreUpdate`). Reads link
  velocity/pose and the 5 control-surface joint positions from the ECM,
  calls `AeroModel.hh`, applies the resulting force/moment to `base_link`.
- `test/aero_model_selftest.cc` — standalone, Gazebo-independent self-test
  executable exercising `AeroModel.hh` directly (no Gazebo instance
  required). Not a substitute for `gazebo-testing`'s live-Gazebo suite.
- `CMakeLists.txt` — builds both of the above.
- `../../docs/source_of_truth/aerodynamics/aero_v1_config.yaml` — the
  structured coefficient dataset (source of truth) the plugin loads at
  `Configure()` time. No coefficient is hardcoded in the C++ source.

## Dependencies

All available in this project's Gazebo Sim Harmonic environment (verified
present, versions checked during this task):

- `gz-sim8` (8.14.0) — System plugin API
- `gz-plugin2` (2.0.4), component `register`
- `gz-transport13` (13.5.0) — wind-topic subscription, diagnostics publisher
- `gz-msgs10` (10.3.2) — `gz::msgs::Vector3d`, `gz::msgs::Double_V`
- `gz-math7` (7.6.0) — `gz::math::Vector3d`, `Quaterniond`
- `yaml-cpp` (0.7.0, via `libyaml-cpp-dev`) — loads `aero_v1_config.yaml`
- CMake >= 3.16, C++17

No dependency outside this list was added.

## Build

```bash
cd /home/emirhan/Desktop/FalconV2/plugins/aerodynamics
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

Produces `build/libFalconV2Aerodynamics.so` and `build/aero_model_selftest`.
`build/` is git-ignored (repository `.gitignore` already excludes
`build/`, `*.so`, `CMakeFiles/`) — rebuild locally, do not commit artifacts.

## Run the standalone self-test (no Gazebo instance needed)

```bash
./build/aero_model_selftest
```

Exits 0 if every `[PASS]`/`[FAIL]` check passes, 1 otherwise. `[INFO]` lines
are not pass/fail — they require live-Gazebo joint testing
(`AILERON_TEST`/`ELEVATOR_TEST`/`RUDDER_TEST`) to physically confirm. See
`AERODYNAMICS.md` for the full self-test result log and analysis, including
one confirmed, honestly-reported `[FAIL]` (`Cma_RESTORING_SIGN_TEST`) that
is a deliberate, documented finding — not a bug to silently patch. See
`AeroModel.hh`'s "IMPORTANT FINDING" comment.

## Run in a live Gazebo instance

`model/model.sdf` already contains the `<plugin filename="FalconV2Aerodynamics"
name="falcon_v2_aero::AerodynamicsSystem">` block with its `<config_path>`,
`<air_density>`, `<wind_topic>`, `<diagnostics_rate_hz>` parameters. To make
the compiled `.so` discoverable:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/emirhan/Desktop/FalconV2/plugins/aerodynamics/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -s -r --iterations 20 tests/gazebo/worlds/falcon_v2_freefall_world.sdf
```

A successful load prints (via `gzmsg`):
```
[FalconV2Aerodynamics] Configured. config=... rho=1.225 S=0.4514 b=2.093
c_ref=0.224 diagnostics topic=/model/falcon_v2/aerodynamics/diagnostics
(order: V,alpha_rad,beta_rad,qbar,CL,CD,CY,Cl,Cm,Cn) @ 20 Hz. wind
topic=/model/falcon_v2/wind ...
```

Diagnostics can be observed live via:
```bash
gz topic -e -t /model/falcon_v2/aerodynamics/diagnostics
```
(field order as printed in the Configure() message above).

This README documents build/run steps only — it intentionally does not
duplicate the architecture, derivation, or self-test analysis, which live in
`docs/source_of_truth/aerodynamics/AERODYNAMICS.md`.

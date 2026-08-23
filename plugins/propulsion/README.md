# FALCON V2 - Propulsion Gazebo Sim Harmonic Plugin

Owner: `propulsion` specialist agent. Task: `PROPULSION_V1_IMPLEMENTATION`.

Scope: motor/ESC/propeller physics only (throttle -> electrical -> motor
torque -> rotor angular dynamics -> RPM -> advance ratio -> real APC
Ct(J)/Cp(J) -> thrust + propeller aerodynamic torque -> reaction torque ->
force/moment at the real hub). No aerodynamic-coefficient physics
(`plugins/aerodynamics/` is untouched), no control-surface actuation, no
ArduPilot/SITL. See `docs/source_of_truth/propulsion/PROPULSION.md` for the
full architecture/provenance record and
`docs/source_of_truth/propulsion/propulsion_v1_config.yaml` for every
numeric constant this plugin consumes.

## Files

- `PropulsionModel.hh` - pure math core (Kt/Ke, motor electrical, APC-table
  CSV loading + 2D Ct/Cp interpolation, T/P/Q formulas, RPM cap logic, rotor
  ODE step). No Gazebo dependency at all (not even gz-math) - used by both
  the real plugin and the standalone self-test below.
- `PropulsionSystem.hh` / `PropulsionSystem.cc` - the `gz-sim8` System
  plugin (`ISystemConfigure` + `ISystemPreUpdate`). Owns the two explicit
  `omega_left`/`omega_right` rotor-dynamics states, reads `base_link`
  pose/velocity and the wind/throttle topics from the ECM, calls
  `PropulsionModel.hh`, applies the resulting force/wrench to `base_link` at
  each real hub, and drives `left_prop_joint`/`right_prop_joint`'s angular
  velocity from the SAME state.
- `test/propulsion_model_selftest.cc` - standalone, Gazebo-independent
  self-test executable exercising `PropulsionModel.hh` directly against the
  real parsed APC data (no Gazebo instance required). Not a substitute for
  `gazebo-testing`'s live-Gazebo suite (`tests/gazebo/README.md` sec 3:
  `PROP_SPINUP_TEST`, `LEFT_MOTOR_TEST`/`RIGHT_MOTOR_TEST`,
  `COUNTER_ROTATION_CANCELLATION_TEST`, `REACTION_TORQUE_TEST`,
  `DIFFERENTIAL_THRUST_YAW_TEST`, `ENGINE_OUT_TEST`, etc.).
- `CMakeLists.txt` - builds both of the above.
- `../../docs/source_of_truth/propulsion/propulsion_v1_config.yaml` - the
  structured dataset (source of truth) the plugin loads at `Configure()`
  time. No coefficient is hardcoded in the C++ source.
- `../../docs/source_of_truth/propulsion/data/PER3_13x65E.dat` - raw,
  immutable, official APC 13x6.5E performance data.
- `../../docs/source_of_truth/propulsion/data/apc_13x65e_parsed.csv` -
  mechanically parsed from the above by
  `../../docs/source_of_truth/propulsion/data/parse_apc_dat.py` (never
  hand-edited). See `../../docs/source_of_truth/propulsion/data/PROVENANCE.md`.

## Dependencies

Same as `plugins/aerodynamics/` (already present in this project's Gazebo
Sim Harmonic environment):

- `gz-sim8` (8.14.0), `gz-plugin2` (2.0.4, component `register`),
  `gz-transport13` (13.5.0), `gz-msgs10` (10.3.2), `gz-math7` (7.6.0),
  `yaml-cpp` (0.7.0) - all required by `PropulsionSystem.cc`/`CMakeLists.txt`.
- `propulsion_model_selftest` depends on the C++ standard library ONLY -
  `PropulsionModel.hh` has no gz-math/gz-sim dependency at all (a step
  further than `AeroModel.hh`, which still needs `gz::math::Vector3d`).

No dependency outside this list was added.

## Build

```bash
cd /home/emirhan/Desktop/FalconV2/plugins/propulsion
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

Produces `build/libFalconV2Propulsion.so` and `build/propulsion_model_selftest`.
`build/` is git-ignored - rebuild locally, do not commit artifacts.

## Run the standalone self-test (no Gazebo instance needed)

```bash
./build/propulsion_model_selftest
```

Exits 0 if every `[PASS]`/`[FAIL]` check passes, 1 otherwise. Covers:
`KV_TO_KT_KE_TEST`, `APC_DATA_PARSE_TEST`, `APC_STATIC_5000/6000/9000/10000_TEST`,
`APC_FORWARD_FLIGHT_INTERPOLATION_TEST`, `J_CALCULATION_TEST`,
`PROP_POWER_TORQUE_TEST`, `ZERO_RPM_NUMERICAL_TEST`, `RPM_LIMIT_MATH_TEST` -
11 tests total, all against the REAL parsed APC 13x6.5E table (no fabricated
Ct/Cp values). See `docs/source_of_truth/propulsion/data/PROVENANCE.md` for
a documented, non-bug finding regarding a small (~1.5-2%) systematic offset
between this project's mandated `D=0.3302 m`/`rho=1.225 kg/m^3` thrust/power
reconstruction and APC's own tabulated Thrust(N)/PWR(W) columns.

## Run in a live Gazebo instance

`model/model.sdf` already contains the
`<plugin filename="FalconV2Propulsion" name="falcon_v2_propulsion::PropulsionSystem">`
block with its `<config_path>`, `<air_density>`, `<wind_topic>`,
`<left_throttle_topic>`, `<right_throttle_topic>`, `<diagnostics_rate_hz>`
parameters. To make the compiled `.so` discoverable:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/emirhan/Desktop/FalconV2/plugins/propulsion/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -s -r --iterations 20 tests/gazebo/worlds/falcon_v2_freefall_world.sdf
```

Command a throttle (V1 test interface, no ArduPilot/SITL exists yet):

```bash
gz topic -t /model/falcon_v2/propulsion/left/throttle_cmd -m gz.msgs.Double -p "data: 0.5"
gz topic -t /model/falcon_v2/propulsion/right/throttle_cmd -m gz.msgs.Double -p "data: 0.5"
```

Diagnostics can be observed live via:
```bash
gz topic -e -t /model/falcon_v2/propulsion/diagnostics
```
(field order printed by `Configure()`'s `gzmsg` line, and documented in
`model/model.sdf`'s propulsion `<plugin>` block comment - 14 fields per
motor, left then right: `throttle, current_A, Q_motor_Nm,
omega_ownframe_rad_s, rpm, J, Ct, Cp, thrust_N, Q_prop_Nm, interpClamped,
rpmCapActive, negativeCurrentClamped, currentLimited`.)

This README documents build/run steps only - it intentionally does not
duplicate the architecture, derivation, or self-test analysis, which live in
`docs/source_of_truth/propulsion/PROPULSION.md` and
`docs/source_of_truth/propulsion/propulsion_v1_config.yaml`.

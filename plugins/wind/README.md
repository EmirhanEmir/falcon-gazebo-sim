# FALCON V2 - Wind/Gust Generator Gazebo Sim Harmonic Plugin

Owner: `aerodynamics` specialist agent. Task:
`WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION`.

Scope: a minimal, deterministic atmospheric wind/gust **publisher** only. It
computes `V_wind_world = V_steady + V_gust(t)` (world frame, m/s - the
velocity of the AIR MASS, not meteorological "wind coming from" phrasing)
and publishes it every `PreUpdate()` tick on the **existing**
`/model/falcon_v2/wind` topic - the same topic
`plugins/aerodynamics/AerodynamicsSystem.cc` and
`plugins/propulsion/PropulsionSystem.cc` already subscribe to and already
correctly consume as `Vrel = Vbody - Vwind` (confirmed by direct read of
both files for this task; **neither file was modified** by this task). This
is the only wind source in the project - no second consumption path exists.

This plugin **never** applies a direct force/wrench to any link (no
`AddWorldForce`/`AddWorldWrench` anywhere in this plugin) - its only
physical output is the published velocity vector. See
`docs/source_of_truth/environment/WIND.md` for the full convention,
formula, and scope-limitation writeup.

## Files

- `WindModel.hh` - pure math core (1-cosine gust envelope, gust-vector
  evaluation, steady+gust composition, direction normalization). No Gazebo
  dependency beyond `gz::math::Vector3d` (same precedent as
  `plugins/aerodynamics/AeroModel.hh`) - used by both the real plugin and
  the standalone self-test below.
- `WindSystem.hh` / `WindSystem.cc` - the `gz-sim8` System plugin
  (`ISystemConfigure` + `ISystemPreUpdate`). Owns the persistent
  steady-wind and single-gust state, advertises the wind topic, subscribes
  to the two command topics, and publishes the composed wind vector every
  tick.
- `test/wind_model_selftest.cc` - standalone, Gazebo-independent self-test
  executable exercising `WindModel.hh` directly (no Gazebo instance
  required).
- `CMakeLists.txt` - builds both of the above.
- `../../docs/source_of_truth/environment/WIND.md` - convention, formula,
  and scope documentation (source of truth for this plugin).

## Dependencies

Same as `plugins/actuators/` minus `yaml-cpp` (this plugin has no
structured coefficient dataset to load - its only inputs are SDF parameters
and live command topics):

- `gz-sim8` (8.14.0), `gz-plugin2` (2.0.4, component `register`),
  `gz-transport13` (13.5.0), `gz-msgs10` (10.3.2), `gz-math7` (7.6.0).
- `wind_model_selftest` depends on `gz-math7` only (for
  `gz::math::Vector3d`) - no full gz-sim/Gazebo runtime required.

## Build

```bash
cd /home/emirhan/Desktop/FalconV2/plugins/wind
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

Produces `build/libFalconV2Wind.so` and `build/wind_model_selftest`.
`build/` is git-ignored - rebuild locally, do not commit artifacts.

## Run the standalone self-test (no Gazebo instance needed)

```bash
./build/wind_model_selftest
```

Exits 0 if every `[PASS]`/`[FAIL]` check passes, 1 otherwise. Covers:
`GUST_ENVELOPE_BOUNDARY_TEST`, `GUST_ENVELOPE_MIDPOINT_TEST`,
`GUST_ENVELOPE_C1_CONTINUITY_TEST`, `GUST_ENVELOPE_OUTSIDE_WINDOW_TEST`,
`GUST_VECTOR_MIDPOINT_TEST`, `GUST_VECTOR_BOUNDARY_ZERO_TEST`,
`GUST_NOT_SCHEDULED_TEST`, `ZERO_WIND_REGRESSION_TEST`,
`STEADY_GUST_COMPOSITION_NO_CROSS_CONTAMINATION_TEST`,
`NORMALIZE_DIRECTION_TEST`, `GUST_AMPLITUDE_SIGN_TEST`,
`ALL_FINITE_SWEEP_TEST` - 12 tests total, all PASS as of this writing.

## Run in a live Gazebo instance

`model/model.sdf` already contains the
`<plugin filename="FalconV2Wind" name="falcon_v2_wind::WindSystem">` block
with its `<steady_wind_mps>`, `<wind_topic>`, `<steady_cmd_topic>`,
`<gust_cmd_topic>` parameters. To make the compiled `.so` discoverable:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/emirhan/Desktop/FalconV2/plugins/wind/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -s -r --iterations 20 tests/gazebo/worlds/falcon_v2_freefall_world.sdf
```

Command a steady wind (world frame, m/s):

```bash
gz topic -t /model/falcon_v2/wind/steady_cmd -m gz.msgs.Vector3d -p "x: 5.0, y: 0.0, z: 0.0"
```

Schedule a 1-cosine gust (fixed field order:
`[dir_x, dir_y, dir_z, amplitude_mps, start_delay_s, duration_s]`, where
`start_delay_s` is relative to the command's own receipt time, not absolute
sim time - see `WindSystem.cc`'s `OnGustCmd()` and
`docs/source_of_truth/environment/WIND.md`):

```bash
gz topic -t /model/falcon_v2/wind/gust_cmd -m gz.msgs.Double_V \
  -p "data: [1.0, 0.0, 0.0, 8.0, 2.0, 3.0]"
```

(a gust blowing toward world +X, peak 8 m/s, starting 2 s after this
command is received, lasting 3 s).

Observe the published wind live:

```bash
gz topic -e -t /model/falcon_v2/wind
```

This README documents build/run steps only - it intentionally does not
duplicate the convention/formula/scope writeup, which lives in
`docs/source_of_truth/environment/WIND.md`.

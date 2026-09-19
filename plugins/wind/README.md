# FALCON V2 — Wind/Gust Generator Plugin

Gazebo Sim Harmonic System plugin providing a minimal, deterministic
atmospheric wind and gust **publisher**.

It computes `V_wind_world = V_steady + V_gust(t)` — world frame, m/s, the
velocity of the air mass, not the meteorological "wind coming from" phrasing
— and publishes it every `PreUpdate()` tick on `/model/falcon_v2/wind`, the
topic the aerodynamics, propulsion and pitot plugins already consume as
`Vrel = Vbody − Vwind`. This is the only wind source in the project; no
second consumption path exists.

This plugin **never** applies a direct force or wrench to any link — no
`AddWorldForce`/`AddWorldWrench` call exists anywhere in it. Its only physical
output is the published velocity vector.

**Not modelled:** spatial wind gradients (shear or gust fronts varying with
aircraft position), terrain effects, wake effects, and Dryden/Von Karman
stochastic turbulence.

## Files

| File | Purpose |
| ---- | ------- |
| `WindModel.hh` | Pure math: 1-cosine gust envelope, gust-vector evaluation, steady+gust composition, direction normalisation. Depends only on `gz-math7`. |
| `WindSystem.hh` / `.cc` | The `gz-sim8` System plugin: SDF parameter reads, command subscriptions, per-tick publishing. |
| `CMakeLists.txt` | Build definition for `libFalconV2Wind.so`. |

## Dependencies

- `gz-sim8`, `gz-plugin2` (component `register`), `gz-transport13`,
  `gz-msgs10`, `gz-math7`
- CMake ≥ 3.16, a C++17 compiler

No `yaml-cpp`: this plugin has no coefficient dataset to load — its only
inputs are SDF parameters and live command topics.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Produces `build/libFalconV2Wind.so`. `build/` is git-ignored.

## Runtime interface

Set a steady wind:

```bash
gz topic -t /model/falcon_v2/wind/steady_cmd -m gz.msgs.Vector3d -p "x: 5.0, y: 0.0, z: 0.0"
```

Schedule a 1-cosine gust. Field order is fixed:
`[dir_x, dir_y, dir_z, amplitude_mps, start_delay_s, duration_s]`, where
`start_delay_s` is relative to the command's own receipt time, not absolute
sim time:

```bash
gz topic -t /model/falcon_v2/wind/gust_cmd -m gz.msgs.Double_V \
  -p "data: [1.0, 0.0, 0.0, 8.0, 2.0, 3.0]"
```

That is a gust blowing toward world +X, peaking at 8 m/s, starting 2 s after
the command is received and lasting 3 s.

**Limitation:** only one gust may be scheduled or in progress at a time — a
new gust command replaces any existing one.

Observe the published wind:

```bash
gz topic -e -t /model/falcon_v2/wind
```

# FALCON V2 - Wind/Gust Disturbance Model (V1)

Owner: `aerodynamics` specialist agent. Task:
`WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION` (2026-08-27).

This document is the source-of-truth writeup for the wind/gust generator
plugin at `plugins/wind/` (`WindModel.hh`/`WindSystem.hh`/`WindSystem.cc`).
It is a **new capability**, not a retune: no existing aerodynamic
coefficient, control lookup table, actuator parameter, or propulsion
parameter was touched to add it.

## 1. What already existed (confirmed, not re-derived, by this task)

Before this task, `plugins/aerodynamics/AerodynamicsSystem.cc` and
`plugins/propulsion/PropulsionSystem.cc` **already** subscribed to a
`wind_topic` SDF parameter (default `/model/falcon_v2/wind`,
`gz.msgs.Vector3d`, world frame, m/s) via their own `OnWind()` callbacks,
and **already** computed relative airflow as:

```
V_rel_world = V_aircraft_world - V_wind_world
```

before rotating into body axes for the aerodynamic-coefficient/propeller
advance-ratio math (`AerodynamicsSystem.cc` line ~311; `PropulsionSystem.cc`
line ~285, `hubVelRel = *hubVelWorldOpt - this->windWorld`). Both defaulted
`windWorld` to `gz::math::Vector3d::Zero` and never had it updated, because
**no publisher existed** for that topic - wind was always exactly zero.

This task's only job was to build that missing publisher. **No line in
`AerodynamicsSystem.cc` or `PropulsionSystem.cc` was modified.**

## 2. Convention: wind is the velocity of the air mass

`V_wind_world` published on `/model/falcon_v2/wind` is the **world-frame
velocity of the air mass itself**, in the same world/ENU-style Cartesian
frame the rest of this project's world-frame quantities use (m/s). It is
**not** meteorological "wind coming from" phrasing (e.g. "a north wind" in
weather reports means air moving *toward* the south - that convention is
deliberately **not** used here). Concretely:

- A wind vector of `(+Vw, 0, 0)` means the air mass itself is moving in the
  world `+X` direction at `Vw` m/s. If the aircraft is stationary, this
  reduces its relative airspeed along `+X` (a tailwind, in the ordinary
  aviation sense) because `V_rel = V_aircraft - V_wind = 0 - (+Vw, 0, 0)`
  points in `-X`... concretely for level flight along `+X`,
  `V_rel_x = V_ac_x - Vw`, so a `+X`-moving air mass reduces headwind
  component / increases tailwind component along the aircraft's forward
  direction, exactly as intuition expects.
- A wind vector of `(-Vw, 0, 0)` (air mass moving toward `-X`) increases an
  aircraft's relative airspeed when flying along `+X` (a headwind).

This is exactly the convention already assumed by
`AerodynamicsSystem.cc`/`PropulsionSystem.cc`'s pre-existing
`Vrel = Vbody - Vwind` formula - this document does not introduce a new
convention, it documents the one that publisher must match.

## 3. Steady + gust composition

```
V_wind_world(t) = V_steady_world + V_gust_world(t)
```

`V_steady_world` is a constant (until commanded otherwise) vector.
`V_gust_world(t)` is the time-varying output of at most one scheduled
1-cosine gust (see §4). Composition is a **plain vector sum** - the steady
term does not affect the gust term's shape/timing and vice versa
(`WindModel.hh::ComposeWind()`, confirmed by
`STEADY_GUST_COMPOSITION_NO_CROSS_CONTAMINATION_TEST` in the self-test).

## 4. The 1-cosine gust profile

For a gust with unit direction `d` (world frame), peak amplitude `A` (m/s,
may be negative - see §6), absolute start time `t_start` (sim seconds), and
duration `T` (seconds, `T > 0`):

```
                    { d * A * 0.5*(1 - cos(2*pi*(t - t_start)/T))   for t in [t_start, t_start + T]
gust_vec(t) =       {
                    { 0                                              otherwise
```

Implemented exactly as this closed form in `WindModel.hh::GustEnvelope()` /
`EvaluateGust()`.

**C1-continuity ("no jerk").** Let `tau = t - t_start`. The envelope
`e(tau) = 0.5*(1-cos(2*pi*tau/T))` satisfies:

- `e(0) = 0.5*(1-cos(0)) = 0`
- `e(T) = 0.5*(1-cos(2*pi)) = 0`
- `de/dtau = (pi/T)*sin(2*pi*tau/T)`, which is `0` at `tau=0` (`sin(0)=0`)
  and `0` at `tau=T` (`sin(2*pi)=0`).

Since the profile is defined to be identically `0` outside `[0,T]` as well,
both the **value** and the **time-derivative** of the published gust
component are exactly zero at both edges of the window - splicing the gust
onto the (otherwise constant) steady background introduces no
discontinuity in wind value or its rate of change at either boundary.
Verified analytically and numerically (central difference,
`GUST_ENVELOPE_C1_CONTINUITY_TEST`, `plugins/wind/test/wind_model_selftest.cc`).

**Peak value.** At the midpoint `tau = T/2`: `e(T/2) = 0.5*(1-cos(pi)) = 1`,
so the gust reaches exactly `d * A` at its midpoint
(`GUST_ENVELOPE_MIDPOINT_TEST`/`GUST_VECTOR_MIDPOINT_TEST`).

**Domain bound ("no silent extrapolation").** Any `t` outside
`[t_start, t_start+T]`, or a non-positive `T`, evaluates to exactly `0.0`,
never extrapolated (`GUST_ENVELOPE_OUTSIDE_WINDOW_TEST`).

## 5. V1 limitation: one gust at a time (stated explicitly)

`WindSystem` holds exactly **one** `GustState` instance. A new, valid gust
command **unconditionally replaces** any in-progress or still-scheduled
gust - there is no queueing and no superposition of two simultaneous gusts.
This is a deliberate V1 simplification, not a silent gap: if a second gust
command arrives while a previous one is still inside its `[t_start,
t_start+T]` window, the previous gust's remaining contribution is discarded
immediately (the published wind jumps directly to whatever the new gust's
own profile evaluates to at that instant, which is `0` at the new gust's
own `tau=0` if `t_start_new` is in the future, or a non-zero value if
`t_start_new` is already in the past relative to the command's arrival).

## 6. Topics and field orders

All topics default as shown; every one is overridable via the plugin's SDF
parameters (`wind_topic`, `steady_cmd_topic`, `gust_cmd_topic` - see
`model/model.sdf`'s `FalconV2Wind` `<plugin>` block).

### 6.1 Output: `/model/falcon_v2/wind` (`gz.msgs.Vector3d`)

The **existing** topic `AerodynamicsSystem`/`PropulsionSystem` already
subscribe to. World frame, m/s, velocity-of-the-air-mass convention (§2).
Published **every `PreUpdate()` tick**, unthrottled - this is required so a
smoothly time-varying gust is actually seen as smoothly varying by the two
consumers, which each only retain the last-received message.

### 6.2 Command: `/model/falcon_v2/wind/steady_cmd` (`gz.msgs.Vector3d`)

Live-overwrites the steady wind component (world frame, m/s). Hold-last-
valid-command failsafe (same policy as `ActuatorSystem::OnCommand()`): a
non-finite (NaN/Inf) component is rejected and the previous valid steady
value is held.

### 6.3 Command: `/model/falcon_v2/wind/gust_cmd` (`gz.msgs.Double_V`)

**Fixed field order, exactly 6 fields:**

| Index | Field | Units | Notes |
|---|---|---|---|
| 0 | `dir_x` | - | Raw direction vector, world frame. Not required to be pre-normalized - `WindSystem` normalizes internally (`WindModel.hh::NormalizeDirection()`). |
| 1 | `dir_y` | - | (same) |
| 2 | `dir_z` | - | (same) |
| 3 | `amplitude_mps` | m/s | Peak gust speed at the profile midpoint. May be negative (§ below). |
| 4 | `start_delay_s` | s | Relative to **this command's own receipt time**, not absolute sim time - see §7. |
| 5 | `duration_s` | s | Must be `> 0`. |

A message with any field count other than 6, any non-finite field, a
non-positive `duration_s`, or a degenerate (near-zero-norm, `< 1e-9`)
direction vector is **rejected outright** (logged via `gzerr`) - any
previously scheduled/in-progress gust is left completely untouched by a
rejected command.

**Negative amplitude is valid and documented**: it flips the resulting
vector's sense relative to `dir_x/y/z` (i.e. `amplitude=-A` with
`dir=(1,0,0)` produces a gust blowing toward `-X`, identical to
`amplitude=+A` with `dir=(-1,0,0)`) - this lets a caller reuse a fixed
direction convention and flip sign via the amplitude field alone if that is
more convenient for a given test script.

## 7. Why `start_delay_s` is relative to receipt, not absolute sim time

`WindSystem` records `lastSimTimeSec` - the sim time as of the most
recently **completed** `PreUpdate()` tick - once per tick. When a gust
command arrives, its absolute start time is computed as:

```
t_start = lastSimTimeSec (at command receipt) + start_delay_s
```

This was chosen (over requiring the caller to supply an absolute sim time)
so a human operator or test script can say "start a gust 2 seconds from
now" without first querying the simulator's current absolute sim time
out-of-band. The tradeoff, stated explicitly: `lastSimTimeSec` is only
updated once per physics tick, so the effective reference time is accurate
to within one physics timestep of the command's true transport-layer
arrival - the same tick-granularity precedent already accepted throughout
this project for other asynchronous command topics (e.g.
`ActuatorSystem`'s hold-last-valid-command reads, `PropulsionSystem`'s
throttle commands). No mutex guards this shared state, for the same reason
none guards `AerodynamicsSystem::windWorld`/`PropulsionSystem::windWorld`
or any other `On*Cmd()` callback's plain member writes in this project -
this is an already-accepted pattern, not a new gap introduced by this
plugin.

## 8. Default (zero-wind) behavior and regression guarantee

With the plugin loaded but never commanded (SDF `<steady_wind_mps>`
defaulting to `0 0 0`, no gust ever published), `WindSystem` publishes
exactly `(0.0, 0.0, 0.0)` on `/model/falcon_v2/wind` every tick - bit-
identical to the value `AerodynamicsSystem::windWorld`/
`PropulsionSystem::windWorld` already default-initialize to
(`gz::math::Vector3d::Zero`, confirmed by direct read of both headers).
Therefore adding this plugin, by itself, cannot alter any already-validated
result - it is purely additive. Confirmed at the pure-math level by
`ZERO_WIND_REGRESSION_TEST` in `plugins/wind/test/wind_model_selftest.cc`
(12/12 self-test checks pass as of this writing); a live-Gazebo confirmation
of the same claim (with the real plugin loaded, publishing, and consumed by
`AerodynamicsSystem`/`PropulsionSystem`) is `gazebo-testing`'s
responsibility, not re-derived here.

## 9. Explicitly NOT modeled (V1 scope limits)

This plugin is a minimal, deterministic disturbance generator. The
following are **not** modeled anywhere in this plugin, and no claim to the
contrary should be inferred:

- **Spatial wind gradients** - the published wind is a single vector valid
  everywhere in the world simultaneously; it does not vary with the
  aircraft's (or any point's) position (no gust fronts, no wind shear
  layers).
- **Terrain effects** (ground roughness, terrain-following wind speedup,
  valley/ridge channeling, etc.).
- **Rotor-wake / building-wake effects** (no interaction with the
  aircraft's own propeller wash or any other body's wake).
- **Dryden or Von Kármán stochastic turbulence models** - the only
  disturbance shape implemented is the single deterministic 1-cosine gust
  of §4; there is no continuous random turbulence spectrum.
- **Any weather-scale atmospheric model** (fronts, pressure systems,
  diurnal/seasonal variation, altitude-dependent wind profiles).

## 10. Provenance

- The `Vrel = Vbody - Vwind` convention and the `wind_topic` subscription
  mechanism: pre-existing, `plugins/aerodynamics/AerodynamicsSystem.cc`
  (task `AERODYNAMICS_V1_IMPLEMENTATION`, 2026-08-22) and
  `plugins/propulsion/PropulsionSystem.cc` (task
  `PROPULSION_V1_IMPLEMENTATION`, 2026-08-23) - confirmed unmodified by
  this task.
- The 1-cosine gust formula, field orders, and V1 scope limits above: this
  task, `WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION` (2026-08-27),
  `aerodynamics` specialist agent - a standard, textbook discrete-gust
  shape (not a fabricated coefficient; it carries no aircraft-specific
  numeric data at all, only a generic, documented time-domain envelope
  shape applied to a user/test-commanded amplitude/direction/duration).
- No aerodynamic coefficient, derivative, lookup table, actuator parameter,
  or propulsion parameter was read, written, fabricated, or estimated by
  this task.

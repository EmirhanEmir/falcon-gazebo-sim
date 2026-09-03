# FALCON V2 — ArduPlane SITL atmosphere datum & airspeed/wind transport

**Owner:** `controls-integration`
**Stage:** `SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION` (2026-09-02)
**Closes:** `validation` MAJOR-1 and MAJOR-2 of
`docs/validation/2026-09-02_ardupilot_tecs_and_cruise_speed_hold_validation.md`
**Firmware under test:** ArduPlane V4.8.0-dev, ArduPilot commit `409226a637`
(`/home/emirhan/gazebo_sim/ardupilot`)
**Bridge under test:** `ardupilot_gazebo` commit `082a0fe` + this project's patch
(`docs/source_of_truth/autopilot/ardupilot_gazebo_airspeed_wind_bridge.patch`)

This document is the source of truth for two things that were previously implicit and wrong:

1. the relationship between the **Gazebo world altitude datum** and the **ArduPlane SITL
   atmosphere altitude reference**, and
2. how **Falcon V2 air-relative airspeed and wind reach ArduPlane**.

No aerodynamic coefficient, propulsion value, PID, `TECS_*` parameter, `PTCH_TRIM_DEG`,
control-surface scaling, mass, CG or inertia is changed by anything recorded here.

---

## 1. Altitude datum — root cause of the silent 584 m

### 1.1 What was observed

Every SITL launch script passed `-O 0,0,0,0`, yet ArduPlane booted with

```
Home: -35.363261 149.165230 alt=584.000000m hdg=0.000000
```

`ORGN.Alt = 584.0`, `POS.Alt ≈ 673 m`, ISA ρ ≈ 1.148 kg/m³, `EAS2TAS ≈ 1.033` — while the Gazebo
worlds declare `<elevation>0.0</elevation>` and the aerodynamics plugin uses a fixed
ρ = 1.225 kg/m³ (`docs/source_of_truth/aerodynamics/aero_v1_config.yaml`, ISA sea level).

### 1.2 Root cause (source-cited)

`-O` **is** parsed and **is** applied — `SITL_cmdline.cpp:471-472` stores it, `:646-658` calls
`parse_home()` then `sitl_model->set_start_location()`. The defect is inside `parse_home()`:

`libraries/AP_HAL_SITL/SITL_cmdline.cpp:761-766`

```c
if (loc.lat == 0 && loc.lng == 0) {
    // default to CMAC instead of middle of the ocean. This makes
    // SITL in MissionPlanner a bit more useful
    loc.lat = -35.363261*1e7;
    loc.lng = 149.165230*1e7;
    loc.alt = 584*100;          // <-- ALSO overwrites the requested altitude
}
```

A latitude/longitude of exactly `0,0` silently substitutes the CMAC field **including its
584 m elevation**, discarding the requested `alt = 0`. Yaw is parsed *after* this block
(`:769`), which is why the banner showed `hdg=0.000000` — the yaw field was honoured, the
altitude field was not. That asymmetry is exactly why the failure went unnoticed.

### 1.3 How 584 m became 673 m in the atmosphere model

| # | Step | Source |
|---|---|---|
| 1 | `home = origin = CMAC`, `home.alt = 58400 cm` | `SIM_Aircraft.cpp:80-95` `set_start_location()` |
| 2 | EKF origin altitude = 584 m → baro auto-sets `_field_elevation_active = origin.alt*0.01 = 584` | `AP_Baro.cpp:1008-1024` `update_field_elevation()` |
| 3 | `get_altitude_AMSL() = get_altitude() + _field_elevation_active` = 89 + 584 = **673 m** | `AP_Baro.h:106` |
| 4 | `_get_EAS2TAS()` → `get_EAS2TAS_extended(673)` → `get_air_density_for_alt_amsl(673)` = 1.148 → `sqrt(1.225/1.148)` = **1.0331** | `AP_Baro_atmosphere.cpp:299-304`, `:235-243`, `:211` |

`EAS2TAS` under `AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED` is a **pure function of altitude
AMSL** — measured baro temperature does not enter it. Fixing the altitude datum fixes it exactly.

### 1.4 Why the *airspeed* number still looked right before

`Aircraft::eas2tas` (the SITL-side one) is assigned **only** in
`SIM_Aircraft.cpp:729-731` inside `update_dynamics()`, which the JSON backend never calls
(`SIM_JSON::update()` runs no internal dynamics). It therefore stayed at its initialiser
`1.0` (`SIM_Aircraft.h:286`) for the entire run. So `airspeed = velocity_air_ef.length()/1.0`
came out numerically equal to true airspeed, matching Gazebo to 0.06 % — while ArduPlane's
*own* `AP_Baro::_get_EAS2TAS()` = 1.0331 then inflated TECS's internal TAS by 3.3 % and its
specific kinetic energy by 6.7 %. Two independent `eas2tas` values disagreeing by 3.3 % is the
actual defect.

### 1.5 The fix

**Do not pass `-O` at all.** With no `-O`, `home_is_set` stays false and
`Aircraft::update_home()` (`SIM_Aircraft.cpp:694-707`, called every FDM step from
`SITL_State.cpp:250`) builds the origin directly from `SIM_OPOS_LAT/LNG/ALT/HDG` — a code path
with **no CMAC substitution**. Values are set in `config/ardupilot/falcon_v2_sitl.parm`
(new `SIMULATION ENVIRONMENT` section, with full provenance there):

```
SIM_OPOS_LAT    -35.363261
SIM_OPOS_LNG    149.165230
SIM_OPOS_ALT    0
SIM_OPOS_HDG    0
```

**Declared datum relationship (the deliverable):**

> **ArduPlane altitude AMSL (m) == Gazebo world z (m, ENU, +up).**
> Both are referenced to the world's `<elevation>0.0</elevation>`, which is also the datum of
> the fixed ρ = 1.225 kg/m³ (ISA **sea level**) used by the aerodynamics plugin.

### 1.6 Why not lat = 0, lng = 0 (as the worlds declare)

`SIM_OPOS_LAT/LNG/ALT` all zero was **tried and live-reproduced as a hard failure**
(2026-09-02). ArduPlane booted, printed `Home: 0.000000 0.000000 alt=0.000000m hdg=0.000000`
— proving the `SIM_OPOS_*` path honours the request exactly — and then panicked:

```
PANIC: uninitialised location returned by _get_location
```

`AP_Common/Location.h:198` defines `initialised()` as `(lat != 0 || lng != 0 || alt != 0)`, and
`AP_AHRS.cpp:492-494` panics on a SITL build if `_get_location()` returns a non-`initialised()`
location. ArduPilot structurally cannot represent a vehicle at exactly (0, 0, 0).

Horizontal datum therefore stays at the CMAC lat/lng that **every prior validated stage
actually ran at**, so this stage changes the altitude datum *only*. The horizontal datum has no
effect on Falcon V2 physics: aerodynamics, propulsion, actuators and the pitot are all
local-frame, and no Gazebo GPS/NavSat output is in the ArduPilot loop (the JSON FDM backend
synthesises GPS from home + local position).

**Recorded discrepancy (harmless, deliberate):** the test worlds declare
`latitude_deg 0.0 / longitude_deg 0.0` while SITL uses CMAC. Only the altitude datum is
physically meaningful here and it is now aligned exactly. `DATA_REQUIRED`: a real Falcon V2
flight-site georeference. When one exists, set `SIM_OPOS_LAT/LNG/ALT` and the worlds'
`<spherical_coordinates>` from it together.

### 1.7 Residual, quantified and owned

`EAS2TAS` is now `sqrt(1.225 / ρ_ISA(z))` with `z` = the aircraft's true Gazebo altitude,
while the aerodynamics plugin uses ρ = 1.225 at **all** altitudes.

| Aircraft altitude | ρ_ISA | ArduPlane `EAS2TAS` | TAS error vs Gazebo truth | SKE error |
|---|---|---|---|---|
| 0 m | 1.2250 | 1.00000 | 0 % | 0 % |
| 89 m (TECS baseline) | 1.21457 | 1.00428 | **+0.43 %** | +0.86 % |
| 673 m (**before this fix**) | 1.1478 | 1.03310 | **+3.31 %** | +6.72 % |

A **7.7× reduction**. The remaining 0.43 % is the genuine ISA density gradient versus the
aerodynamics plugin's documented constant-density `ASSUMPTION`. It is **not**
`controls-integration`'s to remove: closing it means giving the aerodynamics plugin an
altitude-dependent ρ, which is `aerodynamics`' ownership. Recorded here as an open item.

### 1.8 Live before/after evidence

| Quantity | Before (`-O 0,0,0,0`) | After (`SIM_OPOS_*`) | Source |
|---|---|---|---|
| SITL banner | `Home: -35.363261 149.165230 alt=584.000000m hdg=0.000000` | `Home: -35.363261 149.165230 alt=0.000000m hdg=0.000000` | `arduplane.log` |
| `GPS_GLOBAL_ORIGIN.altitude` | 584.0 m (`ORGN.Alt`) | **0.0 m** | MAVLink msg 49, live |
| `GLOBAL_POSITION_INT.alt` on ground | ≈ 584 m | **0.000 m** | MAVLink, live |
| `SCALED_PRESSURE.press_abs` on ground | ≈ 945 hPa | **1013.25 hPa** (ISA sea level, exact) | MAVLink, live |
| `EAS2TAS` at 89 m | 1.0331 | **1.0043** | `AP_Baro_atmosphere.cpp:299-304` |
| implied local ρ at 89 m | 1.1478 | **1.2146** vs plugin 1.2250 | `:211` |

---

## 2. Airspeed & wind transport — root cause and fix

### 2.1 Root cause (source-cited)

`SIM_JSON.cpp:437-453`:

```c
if ((received_bitmask & AIRSPEED)) {
    airspeed = state.airspeed;
    airspeed_pitot = state.airspeed;
} else {
    // wind is not supported yet for JSON sim, assume zero for now
    wind_ef.zero();                              // <-- UNCONDITIONAL
    velocity_air_ef = velocity_ef - wind_ef;     // == ground velocity
    velocity_air_bf = dcm.transposed() * velocity_air_ef;
    update_eas_airspeed();                       // SIM_Aircraft.cpp:1409-1411
}
```

Stock `ardupilot_gazebo` `ArduPilotPlugin::CreateStateJSON()` emits no `airspeed` key
(`grep airspeed src/ArduPilotPlugin.cc` → nothing), so `DataKey::AIRSPEED` (`SIM_JSON.h:167`,
`1ULL<<19`) was never set, the `else` branch ran on **every frame**, and ArduPlane's airspeed
was **ground speed**. `model/model.sdf` `FalconV2Pitot` published a correct wind-relative
airspeed to `/model/falcon_v2/sensors/pitot/airspeed_mps` that **nothing consumed**.

Note also that `velocity_wind` alone cannot fix this: `SIM_JSON.cpp:456-458` assigns
`wind_ef = state.velocity_wind` **after** the block above, so with no `airspeed` key the wind is
zeroed first and the airspeed is computed before the assignment ever happens.

### 2.2 Fix — the official protocol, not a custom bridge

Both keys used are already part of the official ArduPilot SIM_JSON FDM protocol; **no protocol
extension is invented**:

| JSON key | keytable | DataKey | ArduPilot semantics |
|---|---|---|---|
| `airspeed` | `SIM_JSON.h:139` (`DATA_FLOAT`) | `:167` `AIRSPEED` | **EQUIVALENT airspeed (EAS), m/s** |
| `velocity_wind` | `SIM_JSON.h:136` (`DATA_VECTOR3F`) | `:165` `WIND_VEL` | airmass **velocity**, earth frame **NED**, m/s |

`ArduPilotPlugin.cc` was patched (patch tracked at
`docs/source_of_truth/autopilot/ardupilot_gazebo_airspeed_wind_bridge.patch`) to add two
**optional** SDF elements. Both default to empty, in which case the emitted packet is
byte-identical to upstream:

```xml
<airspeed_topic>/model/falcon_v2/sensors/pitot/airspeed_mps</airspeed_topic>
<wind_topic>/model/falcon_v2/wind</wind_topic>
```

wired in `model/model.sdf` inside the existing `ArduPilotPlugin` block.

### 2.3 Units — why the pitot's scalar output IS the correct EAS

ArduPilot consumes the `airspeed` key as **EAS**, not TAS:

- `SIM_JSON.cpp:439-440` assigns it to `airspeed` / `airspeed_pitot`;
- the value those otherwise hold is `velocity_air_ef.length()/eas2tas` (`SIM_Aircraft.cpp:1411`) — i.e. EAS;
- it reaches the sensor model through `SITL_State.cpp:112` → `sitl_airspeed.cpp:30`
  `void SITL_State::_update_airspeed(float eas)`.

`FalconV2Pitot` publishes `|V_rel|` computed against the **same** fixed ρ = 1.225 kg/m³
(ISA sea level) that every aerodynamic force in this simulation uses. Under that project-wide
constant-density assumption,

```
EAS = TAS · sqrt(ρ / ρ_SSL) = TAS · sqrt(1.225/1.225) = TAS
```

so the pitot's scalar output **is** the correct EAS to publish. This equality is a *consequence*
of the documented constant-density assumption, not an independent claim; if `aerodynamics` ever
adopts an altitude-dependent ρ, this conversion must be revisited **at the same time**.

### 2.4 Frames — wind

`/model/falcon_v2/wind` is a `gz.msgs.Vector3d`, **Gazebo world frame**, m/s, **velocity of the
air mass** (`docs/source_of_truth/environment/WIND.md` §2) — the identical topic and identical
convention already consumed by `AerodynamicsSystem.cc`, `PropulsionSystem.cc` and
`PitotSystem.cc`. **No new wind source and no second wind-consumption path is created.**

The plugin rotates it into ArduPilot's NED world frame with the **same** `wldAToWldG` transform
(from `<gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>`) already applied to the
vehicle velocity in the same function — no second frame convention is introduced. ArduPilot's
`wind_ef` uses the identical airmass-velocity sign convention
(`SIM_Aircraft.cpp:1068`: `velocity_air_ef = velocity_ef - wind_ef`).

### 2.5 The end-to-end data path

```
Gazebo aircraft velocity (world ENU, physics ground truth)
        │
        ├─► FalconV2Wind  ── /model/falcon_v2/wind (world ENU airmass velocity) ──┐
        │                                                                          │
        ▼                                                                          │
FalconV2Pitot (PitotSystem.cc)                                                     │
   V_rel = V_point_world − V_wind_world  ;  airspeed = |V_rel|   (10 Hz)           │
        │                                                                          │
        ▼                                                                          │
/model/falcon_v2/sensors/pitot/airspeed_mps  (gz.msgs.Double, EAS m/s)            │
        │                                                                          │
        ▼                                                                          ▼
ArduPilotPlugin::CreateStateJSON()  ── "airspeed": <EAS>   +   "velocity_wind": [N,E,D]
        │                                (UDP JSON FDM packet, 127.0.0.1:9002)
        ▼
SITL_JSON parse_sensors()  → DataKey::AIRSPEED set  → SIM_JSON.cpp:437-440
        │                    (the wind_ef.zero() else-branch is NOT taken)
        │                    SIM_JSON.cpp:456-458 → wind_ef = velocity_wind
        ▼
Aircraft::airspeed / airspeed_pitot  → fill_fdm() SIM_Aircraft.cpp:422
        ▼
SITL_State.cpp:112 _update_airspeed(eas) → sitl_airspeed.cpp
        │   diff_pressure = eas²/SIM_ARSPD_RATIO (+ SIM_ARSPD_RND noise)
        ▼
AP::sitl()->state.airspeed_raw_pressure[i]
        ▼
AP_Airspeed_SITL::get_differential_pressure()   (ARSPD_TYPE = 100)
        ▼
AP_Airspeed  → ARSP.Airspeed (EAS)   [ARSPD_USE = 1]
        ▼
AP_TECS   _TAS_state = EAS × AP_Baro::_get_EAS2TAS()   ← §1 governs this factor
```

### 2.6 Failsafe / limitations (stated, not silent)

- **Hold-last-valid**; non-finite values are rejected in the callback (same convention as
  `FalconV2Wind`'s command interface). If the pitot stopped publishing, the last value would be
  held indefinitely — there is **no staleness timeout**. `V1 LIMITATION`.
- Before the first message on either topic the packet is upstream-identical, so ArduPilot uses
  its ground-velocity fallback for the first ≲ 0.1 s after model load.
- Pitot publishes at 10 Hz (`update_rate_hz`), zero-order-held into a ~1 kHz FDM loop.
  `AP_Airspeed::update()` runs at 10 Hz, so this is well matched.
- With the `AIRSPEED` bit set, `SIM_JSON.cpp` does **not** recompute `velocity_air_ef` /
  `velocity_air_bf`; outside the unused SITL physics backends the only consumer is
  `SITL_State.cpp:193` `fdm.vcas` (FlightGear visualisation). No functional effect.
- The pitot is a scalar `|V_rel|` model: no AoA/position-error correction, no reverse-flow sign
  (`plugins/sensors/PitotModel.hh` header). With the `AIRSPEED` bit set, SITL's own cosine-AoA
  pitot degradation (`SIM_Aircraft.cpp:1419-1430`) is bypassed, so this limitation is now the
  only one in play — consistent, not additive.

### 2.7 Live evidence (ground, aircraft stationary, `groundspeed ≡ 0`)

The decisive test: with the aircraft stationary, groundspeed is 0, so any non-zero airspeed
**can only** have come through the pitot path.

`SIM_ARSPD_RND = 0` (set at test time over MAVLink only — **not** written to
`falcon_v2_sitl.parm`, whose validated baseline keeps the firmware default 2.0):

| commanded wind (world X, m/s) | Gazebo pitot topic (m/s) | ArduPlane airspeed (m/s) | ArduPlane groundspeed |
|---|---|---|---|
| 0 | 0.0 | 0.024 | 0.0 |
| +5 | 5.0 | 4.71 (window includes the step transient) | 0.0 |
| −5 | 5.0 | **5.000** (step of equal magnitude → no transient) | 0.0 |
| 0 | 0.0 | 0.29 (decaying) | 0.0 |
| +12 | 12.0 | 11.26 (window includes the step transient) | 0.0 |

**Before this change all of these read ≈ 0.** The `−5` row — Gazebo pitot 5.0 → ArduPlane
5.000 with groundspeed 0.0 — is an exact match.

With the firmware-default `SIM_ARSPD_RND = 2.0` the same ground test reads 0.58 / 4.34 / 4.83
at 0 / +5 / −5: the SITL airspeed sensor computes
`sqrt(|ratio·(q + noise·rand)|)` (`sitl_airspeed.cpp:37-39`), whose square-root rectifies
zero-mean pressure noise into a positive airspeed bias near zero. That is ArduPilot's own
sensor model, unrelated to this transport.

The on-ground readings are additionally pulled toward zero by ArduPilot's airspeed
failure/consistency logic: `ARSPD_OPTIONS = 11` sets
`ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE | ..._RECOVERY_DO_REENABLE | USE_EKF_CONSISTENCY`
(`AP_Airspeed.h:180-184`). Parked with 12 m/s of airspeed and 0 groundspeed the sensor is
judged inconsistent and its use is disabled, so `GCS_MAVLink_Plane.cpp:256-266` falls back from
`plane.airspeed.get_airspeed()` to `AHRS::airspeed_EAS()`. That is a **parked-aircraft
artefact**, not a transport error. In flight it does not occur (and `AHRS_WIND_MAX = 0`, so the
groundspeed constraint at `AP_AHRS.cpp:810-819` is inactive). Use dataflash `ARSP.Airspeed` for
acceptance, not on-ground `VFR_HUD`.

### 2.8 Live evidence (in flight) — full TECS/FBWB cruise-hold campaign re-run

Same test, same world, same `.parm` except the `SIM_OPOS_*` block; zero wind both times.
Verdict `TECS_CRUISE_SPEED_HOLD_PASS`, `failed_checks=[]`, all 16 `param_preconditions` true.

| Quantity | BEFORE (`-O 0,0,0,0`) | AFTER | Source |
|---|---|---|---|
| SITL banner altitude | `alt=584.000000m` | `alt=0.000000m` | `arduplane.log` |
| `JSON received:` key list | no `airspeed`, no `velocity_wind` | **`velocity_wind`, `airspeed` present** | `arduplane.log` (printed by `SIM_JSON::parse_sensors()`) |
| `ORGN.Alt` | 584.00 | **0.00** | dataflash |
| `POS.Alt` mean | 664.09 m | **80.16 m** | dataflash |
| `BARO.Press` mean | 93 598.7 Pa | **100 366.2 Pa** | dataflash |
| `EAS2TAS` = `TECS.sp / ARSP.Airspeed`, n = 1297 | **1.03343** | **1.00459** | dataflash |
| implied local ρ = 1.225/EAS2TAS² | 1.1470 kg/m³ (−6.4 % vs plugin) | **1.2138 kg/m³ (−0.91 % vs plugin)** | derived |
| `ARSP.Airspeed` (EAS) in-flight mean | 17.9186 | 17.9127 | dataflash |
| `TECS.sp` (TAS) in-flight mean | **18.5082** | **17.9919** | dataflash |
| A_hold airspeed mean / σ | 17.926 / 0.187 | 17.926 / 0.205 | result JSON |
| A_hold throttle mean | 0.4911 | 0.4910 | result JSON |
| A_hold physical pitch | +2.663° | +2.663° | result JSON |
| A_hold vertical speed | +0.00103 m/s | −0.0001 m/s | result JSON |

TECS's internal true airspeed was **+3.2 %** above the Gazebo physical truth (≈17.93 m/s) and is
now **+0.3 %**. Every flight metric is unchanged to within run-to-run noise, confirming the
change is non-perturbing under zero wind — as it must be, since with zero wind the pitot's
`|V_rel|` equals the ground-speed magnitude that ArduPilot was already using.

**AHRS note (pre-existing, now relevant):** `AHRS_EKF_TYPE = 10` in these runs. It is not set in
`falcon_v2_sitl.parm`; `SIM_JSON.cpp:411-416` sets it as the *default* whenever the FDM reports
`no_time_sync`. Consequences, both of which make the pitot integration matter more, not less:
`AP_AHRS_SIM::airspeed_EAS()` (`AP_AHRS_SIM.cpp:22-28`) returns `_sitl->state.airspeed` — now the
pitot value — and `AP_AHRS_SIM.cpp:168` returns `_sitl->state.wind_ef` as the AHRS wind estimate,
which without the `velocity_wind` key would have been **identically zero** in any wind test.

---

## 3. Open items

| # | Item | Owner | Class |
|---|---|---|---|
| 1 | Aerodynamics plugin uses ρ = 1.225 at all altitudes; ArduPlane uses ISA ρ(z). Residual +0.43 % on TAS at 89 m (§1.7). | `aerodynamics` | `ASSUMPTION`, quantified |
| 2 | No real Falcon V2 flight-site georeference; horizontal datum is ArduPilot's CMAC default and does not match the worlds' declared lat/lon 0,0 (physically inert — §1.6). | project owner | `DATA_REQUIRED` |
| 3 | No staleness timeout on the airspeed/wind bridge (§2.6). | `controls-integration` | `V1 LIMITATION` |
| 4 | Pitot has no AoA / position-error correction and no reverse-flow sign. | `controls-integration` | `V1 LIMITATION` |
| 5 | `ardupilot_gazebo` is an out-of-repo dependency; the patch is tracked here but the working copy at `/home/emirhan/gazebo_sim/ardupilot_gazebo` is not under this repo's version control. | project owner | `INFRASTRUCTURE` |

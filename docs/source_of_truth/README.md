# FALCON V2 — Source of Truth

This directory holds the **authoritative engineering input data** for the FALCON V2 Gazebo Sim Harmonic simulation: geometry, mass properties, aerodynamics, propulsion, and control surfaces.

## Rules

- Every value here must be traceable to CAD, manufacturer data, XFOIL, XFLR5, measured test data, a derived calculation, or an explicitly documented assumption.
- Implementation code (SDF, plugins, controllers) may read this data. It must never silently change it.
- Missing data is marked `DATA_REQUIRED` — never filled in with a guess.
- Values marked `ASSUMPTION` or `TEMPORARY` must state why, and by whom/when authorized.

## Status

As of 2026-08-21, this repository was set up from an empty directory. The values below were provided directly by the project owner in the setup conversation. No CAD files, XFOIL/XFLR5 project files, STL meshes, or manufacturer datasheets have been added to this repository yet — those source files themselves are `DATA_REQUIRED` until placed in the relevant subdirectory below.

---

## geometry/

- Wingspan: **2.093 m** (manufacturer/CAD reference)
- Wing area: **0.4514 m²** (manufacturer/CAD reference)
- CAD/STL mesh files: `DATA_REQUIRED`
- Full SDF-ready geometry breakdown (per-link dimensions, hinge locations, control-surface geometry): `DATA_REQUIRED`

## mass_properties/

- Aircraft mass: **6.000 kg**
- Gazebo/CAD reference CG: **(0.168309, 0, 0.100000) m** — origin/reference-frame definition: `DATA_REQUIRED`
- XFLR5 reference CG: **(0.0637, 0, -0.0210) m** — origin/reference-frame definition: `DATA_REQUIRED`

**These two CG values use different reference frames/origins and are not interchangeable.** A documented derivation converting between them is `DATA_REQUIRED` before either is used to set an SDF `<inertial><pose>`.

- Inertia tensor (Ixx, Iyy, Izz, Ixy, Ixz, Iyz): `DATA_REQUIRED`

## aerodynamics/

Source analyses referenced as existing/expected for this project: XFOIL, XFLR5, full-aircraft stability analysis, wing analysis, horizontal tail analysis, vertical tail analysis, elevator analysis, rudder analysis, aileron analysis. Raw analysis files/exports: `DATA_REQUIRED`.

Current full-aircraft / neutral-vertical-fin reference point (provided 2026-08-21):

| Quantity | Value | Notes |
|---|---|---|
| mass | 6.000 kg | |
| trim velocity | 21.244 m/s | |
| trim alpha | 0.364 deg | |
| CL | 0.47167 | |
| XNP (neutral point) | 0.132 m | reference frame/origin: `DATA_REQUIRED` |
| XCP (center of pressure) | 0.064 m | reference frame/origin: `DATA_REQUIRED` |
| CYb | -0.13216 | |
| Clb | -0.00717 | |
| Cnb | +0.03554 | |
| CYp | -0.04567 | |
| Clp | -0.54187 | |
| Cnp | -0.05878 | |
| CYr | +0.08776 | |
| Clr | +0.10586 | |
| Cnr | -0.02227 | |

This is a single reference operating point, **not** a complete aerodynamic model.

`DATA_REQUIRED`: full CL/CD/Cm vs. alpha curves, control-surface effectiveness derivatives (CLde, Cmde, CYdr, Cndr, Clda, Cnda, etc.), Reynolds-number-dependent airfoil polars from XFOIL, per-surface (wing / horizontal tail / vertical tail / elevator / rudder / aileron) breakdowns.

## propulsion/

- 2 × SunnySky X2820 860KV motors
- 2 × APC 13x6.5E propellers
- 4S electrical system

`DATA_REQUIRED`: motor Kv/Rm/I0 datasheet parameters, ESC response characteristics, APC 13x6.5E thrust/torque/RPM/advance-ratio polar data (static and airspeed-dependent), motor/propeller mount locations and orientation on the airframe, battery discharge characteristics.

## controls/

`DATA_REQUIRED`: control-surface geometry and hinge lines (aileron, elevator, rudder), deflection limits, servo specifications, control mapping/gains, ArduPilot SITL parameter file.

---

## How this maps to agents

- `geometry-structure` reads/writes `geometry/` and `mass_properties/`.
- `aerodynamics` reads/writes `aerodynamics/`.
- `propulsion` reads/writes `propulsion/`.
- `controls-integration` reads/writes `controls/`.
- `validation` reads all of the above to cross-check implementation; it does not write here.

No implementation work (SDF, plugins, controllers) has started. This README will be updated as real source files are added to each subdirectory.

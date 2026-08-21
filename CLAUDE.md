# FALCON V2 — Gazebo Sim Harmonic Simulation Project

## Repository status

Greenfield as of 2026-08-21. When this infrastructure was set up, this directory contained **no files at all** — no SDF, meshes, plugins, aerodynamic/propulsion/control code, and no prior git history. Every numeric value in this file was provided directly by the project owner during setup and is treated as authoritative reference data. The underlying source files (CAD, XFOIL/XFLR5 project files, STL meshes, manufacturer datasheets) have not yet been added to the repository — see `docs/source_of_truth/README.md` for what is `DATA_REQUIRED`.

## Project

- **Aircraft:** FALCON V2
- **Simulator:** Gazebo Sim Harmonic

## Primary goal

Build a physically defensible Gazebo simulation of FALCON V2 using real aircraft geometry and collected aerodynamic/propulsion data.

The goal is **not** to artificially tune the aircraft until it flies. The simulation must reproduce, as accurately as possible, the aircraft represented by the engineering data we have.

## Coordinate system

The Gazebo aircraft body frame is **FLU**:
- +X = forward
- +Y = left
- +Z = up

Never use another convention without an explicit, documented conversion. Do not conflate:
- FLU
- FRD
- NED
- ENU
- XFLR5 reference coordinates

Every coordinate transformation must be documented (derivation, author, location) — see `docs/source_of_truth/geometry/` and `docs/architecture/`.

## Aircraft mass properties

- **Aircraft mass:** 6.000 kg
- **Current Gazebo/CAD CG:** (0.168309, 0, 0.100000) m
- **Current XFLR5 reference CG:** (0.0637, 0, -0.0210) m

These two CG values use **different reference definitions**. Never substitute one for the other directly. Any conversion between them must be explicitly derived and documented before it is used anywhere in the simulation.

Mass or CG must never be changed without explicit authorization.

## Manufacturer geometry reference

- **Wingspan:** 2.093 m
- **Wing area:** 0.4514 m²

Existing CAD/STL geometry is authoritative for model placement unless a documented discrepancy is found. STL/mesh geometry must never be changed without explicit authorization.

## Aerodynamic source data

Completed (or expected) aerodynamic analyses for this project:
- XFOIL
- XFLR5
- full-aircraft stability analysis
- wing analysis
- horizontal tail analysis
- vertical tail analysis
- elevator analysis
- rudder analysis
- aileron analysis

These are **source data**. Never recreate them with arbitrary assumptions while real data exists or can be obtained.

### Current full-aircraft / neutral vertical-fin reference point

| Quantity | Value |
|---|---|
| mass | 6.000 kg |
| trim velocity | 21.244 m/s |
| trim alpha | 0.364 deg |
| CL | 0.47167 |
| XNP | 0.132 m |
| XCP | 0.064 m |
| CYb | -0.13216 |
| Clb | -0.00717 |
| Cnb | +0.03554 |
| CYp | -0.04567 |
| Clp | -0.54187 |
| Cnp | -0.05878 |
| CYr | +0.08776 |
| Clr | +0.10586 |
| Cnr | -0.02227 |

This is **not** the complete aerodynamic model. Do not guess missing coefficients. Report missing data as `DATA_REQUIRED`. Never silently add an estimated value.

## Propulsion reference

- 2 × SunnySky X2820 860KV motors
- 2 × APC 13x6.5E propellers
- 4S electrical system

Target propulsion model chain:

```
throttle → electrical/motor response → motor RPM → propeller aerodynamic loading → thrust, torque
```

Represent airspeed effects to the extent the available data/modeling supports. Do not collapse this into `throttle × maximum_thrust`. That simplification may only be used, and only with explicit authorization, as a temporary diagnostic — and must be labeled `TEMPORARY_TEST_MODEL` everywhere it appears.

## Engineering rules

- Use SI units internally.
- Every numeric constant used in the simulation must have documented provenance, traceable to one of:
  - CAD
  - manufacturer data
  - XFOIL
  - XFLR5
  - measured test data
  - derived calculation
  - explicitly documented assumption
- No unexplained magic numbers.
- A value that must temporarily be estimated: mark `ASSUMPTION`.
- A placeholder value: mark `TEMPORARY`.
- Missing required information: mark `DATA_REQUIRED`.
- Never hide missing information.

## Source-of-truth policy

Raw engineering data and simulation implementation are kept separate.

Large aerodynamic and propulsion datasets belong in configuration files or structured tables — not embedded directly in C++/Python source code.

Authoritative engineering data lives in `docs/source_of_truth/`. Implementation code may read this data but must never silently modify it.

## Simulation tuning policy

Never change aerodynamic coefficients to make the aircraft "stable."

If a simulation test fails, do **not** change any of the following as a shortcut fix:
- CG
- mass
- inertia
- aerodynamic derivatives
- control authority
- motor thrust

Before any tuning, verify, in order:

1. units
2. reference frames
3. sign conventions
4. geometry
5. force application points
6. CG
7. inertia reference
8. aerodynamic equations
9. control directions
10. duplicated forces
11. integration timestep
12. numerical stability

## Agent workflow

For non-trivial changes, the main Claude Code session delegates investigation and implementation to the appropriate specialist agent. Agents work only within their own ownership boundaries.

After a significant implementation change:
1. `gazebo-testing` runs the relevant simulation tests.
2. `validation` independently reviews the implementation and test results.

Testing and validation are separate responsibilities and are never merged into one agent.

Full workflow diagram and ownership boundaries: `docs/architecture/AGENT_WORKFLOW.md`.

## Project agents

| Agent | Owns |
|---|---|
| `geometry-structure` | SDF structure, links/joints, meshes, collision/visual geometry, hinge geometry, mass distribution, inertia tensors, CG, force application locations, coordinate frames |
| `aerodynamics` | Aerodynamic architecture, XFOIL/XFLR5 integration, lift/drag/moment coefficients, stability derivatives, control-surface aero effects, interpolation |
| `propulsion` | Motor/prop/battery modeling, throttle→RPM→thrust/torque chain, airspeed-dependent loading, force application points |
| `controls-integration` | Control surface actuation, servo behavior, joint limits/direction, sign conventions, Gazebo control interfaces, ArduPilot SITL/MAVLink integration |
| `gazebo-testing` | Simulation execution, test infrastructure, test worlds/scripts, regression tests, result capture — never tunes physics to pass a test |
| `validation` | Read-only engineering review of all of the above; classifies findings CRITICAL/MAJOR/MINOR/INFO; never edits parameters itself |

Agent definitions: `.claude/agents/`.

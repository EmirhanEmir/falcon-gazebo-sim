# FALCON V2 — Agent Workflow & Ownership Boundaries

## Status

Infrastructure setup only, as of 2026-08-21. No aircraft physics, SDF, or control implementation exists in this repository yet. This document describes the process that will govern all future implementation work.

## Workflow

```
Main Claude
    |
    v
specialist implementation agent
 (geometry-structure / aerodynamics / propulsion / controls-integration)
    |
    v
gazebo-testing
    |
    v
validation
    |
    +-- PASS -----------------------------> done, findings logged
    |
    +-- issue found --> responsible specialist agent
                             |
                             v
                       specialist correction
                             |
                             v
                       gazebo-testing (re-run)
                             |
                             v
                       validation (re-review)
                             |
                             +-- loop until PASS or explicitly deferred
```

Rules:

- The main Claude Code session is the only entry point that assigns work to a specialist agent. Specialist agents don't self-assign cross-boundary work.
- `general-purpose` is used only when no project specialist agent fits the task. Any task inside a specialist's ownership boundary (below) is delegated to that specialist, never to `general-purpose` — see `CLAUDE.md` § "Orchestration rule: specialist agents over `general-purpose`" for the full domain-to-agent mapping. `general-purpose` is reserved for genuinely general work that falls outside all six ownership boundaries.
- A specialist agent only touches files within its own ownership boundary (below). If a task spans boundaries, main Claude splits it and delegates each part to its owning agent.
- `gazebo-testing` always runs after a specialist makes a non-trivial change. It reports `TEST_FAILED` with observed vs. expected behavior, evidence, suspected subsystem, and the responsible agent. It never edits physics parameters to make a test pass.
- `validation` always reviews after testing. It is read-only: it classifies findings `CRITICAL` / `MAJOR` / `MINOR` / `INFO` and routes issues back to the responsible specialist. It never silently corrects a parameter.
- Testing and validation are two different agents. They are never merged into one.

## Ownership boundaries

### geometry-structure
**Owns:** Gazebo Harmonic SDF structure, links, joints, mesh placement, collision geometry, visual geometry, control-surface hinge geometry, mass distribution, inertia tensors, center of gravity, force application locations, coordinate frames, geometry consistency.
**Does not own:** aerodynamic coefficients, propulsion models, control actuation logic, tests.
**Constraint:** edits geometry/SDF files only when explicitly assigned.

### aerodynamics
**Owns:** aerodynamic architecture, XFOIL/XFLR5 data integration, lift/drag/pitching moment, lateral force, rolling/yawing moment, angle of attack and sideslip handling, dynamic pressure, aerodynamic reference quantities, longitudinal and lateral-directional derivatives, aileron/elevator/rudder aerodynamic effects, coefficient interpolation, aerodynamic force/moment application.
**Does not own:** geometry/CG placement, propulsion models, control actuation, tests.
**Constraint:** never fabricates coefficients while source data exists; documents interpolation methods; never silently extrapolates; stall/post-stall modeling requires explicit review before implementation.

### propulsion
**Owns:** SunnySky X2820 860KV motor modeling, APC 13x6.5E propeller modeling, 4S electrical system modeling, throttle-to-motor relationship, RPM, motor response dynamics, propeller thrust/torque, airspeed-dependent propeller loading, motor/prop inertia where appropriate, left/right motor force application points.
**Does not own:** motor/prop mount geometry, throttle command sourcing from the autopilot, tests.
**Constraint:** final implementation must represent throttle → motor response → RPM → propeller thrust/torque. No arbitrary thrust curves. `throttle × maximum_thrust` only permitted as an explicitly authorized `TEMPORARY_TEST_MODEL`.

### controls-integration
**Owns:** aileron/elevator/rudder actuation, servo behavior, control limits, joint limits/direction, control sign, actuator mapping, Gazebo control interfaces, ArduPilot SITL integration, MAVLink/plugin interfaces.
**Does not own:** hinge geometry/joint placement, aerodynamic effect of a deflection, motor response to a throttle command, tests.
**Constraint:** control direction must always be explicitly verified — `positive command = expected physical direction` is never assumed without a test.

### gazebo-testing
**Owns:** launching Gazebo Sim Harmonic, loading test worlds, spawning FALCON V2, verifying model loading, capturing runtime errors, executing deterministic test scenarios, inspecting simulation logs, testing control surfaces/propulsion/aerodynamic/trim/static/dynamic behavior, producing reproducible test commands, saving test results. May create test worlds, test/launch scripts, automated regression tests, and test result reports.
**Does not own:** any aircraft physics parameter.
**Constraint:** never changes aircraft physics parameters to make a test pass. Reports failures as `TEST_FAILED` with observed behavior, expected behavior, evidence/logs, suspected subsystem, and the responsible agent.

### validation
**Owns (read-only):** auditing equations, dimensions/units, reference frames, coordinate transforms, signs, force directions and application points, CG, inertia references, aerodynamic reference values, control direction; detecting duplicated forces/damping; inspecting propulsion equations, interpolation methods, numerical stability; comparing implementation against source-of-truth; reviewing `gazebo-testing` reports.
**Does not own:** any file edits to engineering parameters, test authoring/execution.
**Constraint:** never edits engineering parameters. Classifies findings `CRITICAL` / `MAJOR` / `MINOR` / `INFO` and routes them to the responsible specialist agent.

## Future test categories (infrastructure only for now — not implemented)

`MODEL_LOAD_TEST`, `STATIC_GRAVITY_TEST`, `CG_BALANCE_TEST`, `CONTROL_SURFACE_DIRECTION_TEST`, `AILERON_TEST`, `ELEVATOR_TEST`, `RUDDER_TEST`, `LEFT_MOTOR_TEST`, `RIGHT_MOTOR_TEST`, `SYMMETRIC_PROPULSION_TEST`, `ZERO_AIRSPEED_AERO_TEST`, `AOA_SIGN_TEST`, `SIDESLIP_SIGN_TEST`, `TRIM_TEST`, `STRAIGHT_LEVEL_FLIGHT_TEST`, `ROLL_RESPONSE_TEST`, `PITCH_RESPONSE_TEST`, `YAW_RESPONSE_TEST`, `ENGINE_OUT_TEST`, `NUMERICAL_STABILITY_TEST`

See `tests/gazebo/README.md` for the test directory layout.

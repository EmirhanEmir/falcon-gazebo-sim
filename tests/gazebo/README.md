# FALCON V2 — Gazebo Test Infrastructure

Status: infrastructure only. No aircraft physics tests are implemented yet — this is scaffolding for the `gazebo-testing` agent to build out once geometry, aerodynamics, propulsion, and controls implementation exists.

## Directory layout

```
tests/
  gazebo/       # Gazebo Sim Harmonic world / model-load / runtime tests (this directory)
  physics/      # lower-level physics checks (gravity, inertia, numerical stability)
  regression/   # regression suite run after any implementation change
```

Test results are saved to `docs/test_results/`.

## Planned test categories

Owned by `gazebo-testing`, independently reviewed by `validation`. None are implemented yet.

- `MODEL_LOAD_TEST`
- `STATIC_GRAVITY_TEST`
- `CG_BALANCE_TEST`
- `CONTROL_SURFACE_DIRECTION_TEST`
- `AILERON_TEST`
- `ELEVATOR_TEST`
- `RUDDER_TEST`
- `LEFT_MOTOR_TEST`
- `RIGHT_MOTOR_TEST`
- `SYMMETRIC_PROPULSION_TEST`
- `ZERO_AIRSPEED_AERO_TEST`
- `AOA_SIGN_TEST`
- `SIDESLIP_SIGN_TEST`
- `TRIM_TEST`
- `STRAIGHT_LEVEL_FLIGHT_TEST`
- `ROLL_RESPONSE_TEST`
- `PITCH_RESPONSE_TEST`
- `YAW_RESPONSE_TEST`
- `ENGINE_OUT_TEST`
- `NUMERICAL_STABILITY_TEST`

## Rules

- `gazebo-testing` may create test worlds, test scripts, launch scripts, automated regression tests, and result reports here.
- `gazebo-testing` must never change aircraft physics parameters (mass, CG, inertia, aerodynamic coefficients, control authority, motor thrust) to make a test pass.
- A failing test is reported as `TEST_FAILED` with: observed behavior, expected behavior, evidence/logs, suspected subsystem, and the responsible specialist agent (`geometry-structure`, `aerodynamics`, `propulsion`, or `controls-integration`).
- `validation` reviews test results independently of `gazebo-testing` — the two roles are never merged.

## Current phase

No test scenarios are implemented yet, since no aircraft physics implementation exists in this repository. See `docs/architecture/AGENT_WORKFLOW.md` for the full agent workflow this test infrastructure supports.

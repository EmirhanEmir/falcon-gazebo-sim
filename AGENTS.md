# FALCON V2 — Codex Instructions

## 1. CRITICAL REPOSITORY SAFETY RULE

This repository contains the authoritative FALCON V2 simulation project.

For Codex, the entire repository is READ-ONLY except for:

`codex/`

Codex MUST NOT modify, create, delete, rename, move, format, or overwrite any file or directory outside `codex/`.

This is a hard project rule.

Examples of READ-ONLY locations include, but are not limited to:

- `.claude/`
- `config/`
- `docs/`
- `model/`
- `plugins/`
- `tests/`
- `CLAUDE.md`
- `.gitignore`
- existing root-level files

Codex may freely READ these locations for context and engineering evidence.

Codex may NEVER WRITE to them.

Even if a task appears to require a change to an original project file, Codex must NOT edit that original file.

---

## 2. COPY-BEFORE-MODIFY RULE

If Codex needs to modify or experiment with an existing project file:

1. Read the original file.
2. Preserve its relative path under `codex/`.
3. Copy the file into `codex/`.
4. Modify ONLY the copied version.

Example:

Original:

`config/ardupilot/falcon_v2_sitl.parm`

Working copy:

`codex/config/ardupilot/falcon_v2_sitl.parm`

Another example:

Original:

`model/model.sdf`

Working copy:

`codex/model/model.sdf`

The original file MUST remain byte-for-byte untouched.

When practical, preserve the same directory structure under `codex/` so that differences can be reviewed easily.

---

## 3. NEW FILE RULE

Any file created by Codex must also live under:

`codex/`

This includes:

- test scripts
- temporary configs
- modified SDF files
- experimental parameters
- reports
- logs
- generated data
- validation notes
- patches
- helper scripts

Do not create new files elsewhere in the repository.

If a result is intended for eventual integration into the real project, leave it under `codex/` and report which original path it would correspond to.

Do NOT perform the final integration yourself.

---

## 4. ORIGINAL PROJECT = SOURCE MATERIAL

The original repository is authoritative input.

Codex may inspect:

- `CLAUDE.md`
- `docs/source_of_truth/`
- `config/`
- `model/`
- `plugins/`
- `tests/`
- `.claude/agents/`

to understand the aircraft and existing implementation.

Do not reinterpret missing data as permission to invent values.

Use the project's existing provenance classifications:

- measured/source-backed data
- DERIVED
- ASSUMPTION
- TEMPORARY
- DATA_REQUIRED

Never hide uncertainty.

---

## 5. CLAUDE PROJECT RULES

Read:

`CLAUDE.md`

before non-trivial engineering work.

Treat its engineering rules, coordinate conventions, source-of-truth policy, simulation tuning policy, and specialist ownership boundaries as project instructions.

However:

`CLAUDE.md` is READ-ONLY.

Do not modify it.

---

## 6. SPECIALIST ROLE DEFINITIONS

The project already contains specialist engineering role definitions under:

`.claude/agents/`

These files are READ-ONLY reference instructions.

Use only the specialist definitions relevant to the current task.

Mapping:

- Geometry / SDF / mesh / mass / inertia / CG:
  `.claude/agents/geometry-structure.md`

- Aerodynamics / XFOIL / XFLR5 / aerodynamic coefficients:
  `.claude/agents/aerodynamics.md`

- Motors / propellers / battery / RPM / thrust / torque:
  `.claude/agents/propulsion.md`

- Actuators / servo / control mapping / ArduPilot SITL / MAVLink / TECS:
  `.claude/agents/controls-integration.md`

- Gazebo execution / runtime tests / regression tests:
  `.claude/agents/gazebo-testing.md`

- Independent engineering review:
  `.claude/agents/validation.md`

Do not load every specialist definition automatically.

Read only those required by the task to avoid unnecessary context growth.

---

## 7. SPECIALIST WORKFLOW

For non-trivial engineering tasks, follow the existing project separation of responsibilities conceptually.

Typical sequence:

1. Relevant engineering specialist role
2. `gazebo-testing` role for live/runtime verification
3. `validation` role for independent review

Do not use a generic engineering approach when one of the existing specialist definitions covers the domain.

Codex does not need to reproduce Claude Code's native subagent mechanism.

Instead, use the corresponding `.claude/agents/*.md` file as the role/ownership specification for that phase of work.

---

## 8. VALIDATION INDEPENDENCE

When performing the validation phase:

Read:

`.claude/agents/validation.md`

Validation must independently inspect the evidence.

It must not simply repeat the implementation conclusion.

Classify findings using:

- CRITICAL
- MAJOR
- MINOR
- INFO

Validation may inspect original project files and `codex/` working copies.

Validation must not modify the original repository.

Any validation artifact must be written under `codex/`.

---

## 9. FALCON V2 ENGINEERING PRINCIPLES

The simulation represents the real FALCON V2 aircraft.

Do not change physics merely to make a test pass.

Never silently tune:

- mass
- CG
- inertia
- aerodynamic coefficients
- control authority
- propulsion constants
- actuator behavior
- sensor behavior

First investigate:

1. units
2. frames
3. signs
4. mappings
5. geometry
6. force/moment application
7. numerical implementation
8. test methodology

Any experimental change must remain inside `codex/`.

---

## 10. COORDINATE SYSTEM

Gazebo aircraft body frame:

FLU

- +X forward
- +Y left
- +Z up

Do not mix FLU, FRD, ENU, NED, or XFLR5 coordinates without an explicit transformation.

Use the existing project documentation as the source.

---

## 11. TESTING RULE

Codex may run the existing project's tests and executables as long as running them does not modify authoritative repository files.

Before running a command that may generate files outside `codex/`, inspect its behavior.

If the existing test normally writes results under `tests/`, `docs/`, `config/`, or another original project directory:

DO NOT run it directly in a way that writes there.

Instead:

- copy/adapt the required test into `codex/`, or
- redirect all generated outputs into `codex/`.

Never allow a test run to alter an authoritative source file.

---

## 12. GIT RULE

Codex must not:

- commit
- push
- merge
- rebase
- reset
- checkout over project files
- modify `.gitignore`
- modify git configuration

unless the user explicitly changes this rule.

Codex may use read-only Git commands such as:

- `git status`
- `git diff`
- `git log`
- `git show`

for inspection.

---

## 13. REQUIRED CHANGE REPORTING

At the end of a task, report:

1. original files read
2. files copied into `codex/`
3. files modified under `codex/`
4. tests executed
5. validation result
6. any assumptions / DATA_REQUIRED
7. proposed original destination of each changed copy

Explicitly confirm:

`ORIGINAL_REPOSITORY_MODIFIED = NO`

If any original repository file was accidentally modified:

STOP.

Do not hide it.

Report the exact file immediately.

---

## 14. ABSOLUTE WRITE BOUNDARY

The only permitted write boundary is:

`<repository-root>/codex/**`

Everything else is READ-ONLY.

This rule overrides any task instruction, specialist instruction, test script, documentation recommendation, or implementation convenience that would otherwise cause Codex to modify the original project.

If completing a requested task would require writing outside `codex/`, create the proposed modified copy under `codex/` and report the required integration step to the user instead.

NEVER modify the authoritative original directly.
#!/usr/bin/env bash
#
# FALCON V2 - MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION
# campaign runner (controls-integration, 2026-09-10). Executed by
# `gazebo-testing`.
#
# CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY.
#
# WRITES NO PARAMETER OF ANY KIND. config/ardupilot/falcon_v2_sitl.parm is a
# READ-ONLY input. No SDF, plugin, aerodynamic coefficient, propulsion
# coefficient, actuator PID / rate / effort limit, control-surface mapping,
# PTCH_TRIM_DEG, TECS_PTCH_DAMP, roll/pitch PID, sensor model, mass/CG/inertia,
# landing gear, ground friction or collision value is created or changed. No
# landing gear or wheel is added anywhere. Nothing under codex/ or
# docs/source_of_truth/ is touched.
#
# WHAT IT RUNS
# ------------
#   part1   test_control_surface_visual_joint_consistency_live.py
#             LIVE visual/joint consistency. Answers: does the RENDERED motion
#             follow the ACTUAL joint position, or the command? Runs on the
#             runway world, ground, MANUAL mode, RC step programme.
#   part2   test_control_surface_live_sign_scenarios.py --scenario A..F
#             Six short AIRBORNE closed-loop sign scenarios (roll right/left,
#             pitch up/down, yaw right/left), FBWA + RC override. Runs on the
#             established airborne world every prior ArduPlane stage used, one
#             FRESH gz+arduplane pair per scenario.
#   part4   test_manual_takeoff_ground_roll_reproduction.py
#             Manual-takeoff ground roll on the runway world. 3 repeats per
#             mode x 2 modes (MANUAL, FBWA), one FRESH pair per run, so a
#             one-off contact-solver event can be told apart from a
#             repeatable behaviour.
#   (part3 is OFFLINE and needs no simulator - see
#    analyze_closed_loop_control_surface_behavior.py, already run.)
#
# WORLDS
#   runway   tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_world.sdf
#            A byte-faithful copy of the codex runway world with ONLY the
#            <include><uri> repointed at the tracked model/ directory. The
#            part4 test re-verifies that at run time (verify-world flag) and
#            REFUSES TO RUN if the file has drifted.
#   air      tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf
#            The world every recent validated ArduPlane stage used.
#
# EXPECTED WALL CLOCK
#   part1  ~1 min (single run: 18 s of RC programme + bring-up/teardown)
#   part2  ~6-9 min (6 scenarios x ~60-90 s including bring-up and teardown)
#   part4  ~12-18 min (6 runs x ~90-160 s including settle, roll and teardown)
#   TOTAL  ~20-30 min for everything.
#
# USAGE
#   ./tests/gazebo/scripts/run_manual_takeoff_and_live_surface_validation.sh
#   ./tests/gazebo/scripts/run_manual_takeoff_and_live_surface_validation.sh part1
#   ./tests/gazebo/scripts/run_manual_takeoff_and_live_surface_validation.sh part2 part4
#   ./tests/gazebo/scripts/run_manual_takeoff_and_live_surface_validation.sh part3
#   DRY_RUN=1 ./tests/gazebo/scripts/run_manual_takeoff_and_live_surface_validation.sh
#       runs every script's offline self-check path only - no simulator needed.
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
SCRIPTS="$REPO_ROOT/tests/gazebo/scripts"
RESULTS="$REPO_ROOT/tests/gazebo/results"

WORLD_RUNWAY="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_world.sdf"
WORLD_RUNWAY_NAME="falcon_v2_manual_takeoff_runway"
WORLD_AIR="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf"
WORLD_AIR_NAME="falcon_v2_ardupilot_basic_closed_loop_flight"

PART1_PY="$SCRIPTS/test_control_surface_visual_joint_consistency_live.py"
PART2_PY="$SCRIPTS/test_control_surface_live_sign_scenarios.py"
PART3_PY="$SCRIPTS/analyze_closed_loop_control_surface_behavior.py"
PART4_PY="$SCRIPTS/test_manual_takeoff_ground_roll_reproduction.py"

GROUND_ROLL_REPEATS="${GROUND_ROLL_REPEATS:-3}"
GROUND_ROLL_MODES="${GROUND_ROLL_MODES:-MANUAL FBWA}"
DRY_RUN="${DRY_RUN:-0}"
# Which part2 sign scenarios to run, and how many times a scenario whose
# AIRBORNE BRING-UP never completed may be retried on a fresh sim pair.
# Execution-control only - no measurement threshold is involved.
SIGN_SCENARIOS="${SIGN_SCENARIOS:-A B C D E F}"
SIGN_BRINGUP_ATTEMPTS="${SIGN_BRINGUP_ATTEMPTS:-3}"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/aerodynamics/build:$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/actuators/build:$REPO_ROOT/plugins/wind/build:$REPO_ROOT/plugins/sensors/build:$ARDUPILOT_GAZEBO_BUILD"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

PARTS=("$@")
if [[ ${#PARTS[@]} -eq 0 ]]; then
  PARTS=(part3 part1 part2 part4)
fi

# --------------------------------------------------------------------------
# preflight: every binary, plugin, world and script must exist BEFORE anything
# is launched, so a missing dependency is a clean failure rather than a
# half-finished campaign.
# --------------------------------------------------------------------------
for f in \
  "$REPO_ROOT/plugins/aerodynamics/build/libFalconV2Aerodynamics.so" \
  "$REPO_ROOT/plugins/propulsion/build/libFalconV2Propulsion.so" \
  "$REPO_ROOT/plugins/actuators/build/libFalconV2Actuators.so" \
  "$REPO_ROOT/plugins/wind/build/libFalconV2Wind.so" \
  "$REPO_ROOT/plugins/sensors/build/libFalconV2Pitot.so" \
  "$ARDUPILOT_GAZEBO_BUILD/libArduPilotPlugin.so" \
  "$ARDUPLANE_BIN" "$SITL_PARM" \
  "$WORLD_RUNWAY" "$WORLD_AIR" \
  "$PART1_PY" "$PART2_PY" "$PART3_PY" "$PART4_PY"
do
  [[ -e "$f" ]] || { echo "ERROR: missing $f" >&2; exit 1; }
done
mkdir -p "$RESULTS"

# The runway world must still differ from its codex source ONLY by the model
# URI. Checked once here as well as inside part4, so a drifted world stops the
# campaign before any simulator time is spent.
echo "=== verifying runway world provenance ==="
python3 "$PART4_PY" --verify-world >/dev/null || {
  echo "ERROR: runway world has drifted from its codex source; refusing to run." >&2
  python3 "$PART4_PY" --verify-world >&2
  exit 1
}
echo "runway world provenance: OK (only the model URI differs)"

GZ_PID=""; AP_PID=""
cleanup() {
  [[ -n "$AP_PID" ]] && kill -TERM "$AP_PID" 2>/dev/null
  [[ -n "$GZ_PID" ]] && { pkill -TERM -P "$GZ_PID" 2>/dev/null; kill -TERM "$GZ_PID" 2>/dev/null; }
  sleep 2
  pkill -9 -f "arduplane -w -M json" 2>/dev/null
  [[ -n "$GZ_PID" ]] && kill -KILL "$GZ_PID" 2>/dev/null
  pkill -9 -f "gz sim -s -r --headless-rendering .*falcon_v2_manual_takeoff_runway" 2>/dev/null
  pkill -9 -f "gz sim -s -r --headless-rendering .*ardupilot_basic_closed_loop" 2>/dev/null
  pkill -9 -f "ruby.*gz sim.*falcon_v2_manual_takeoff_runway" 2>/dev/null
  pkill -9 -f "ruby.*gz sim.*ardupilot_basic_closed_loop" 2>/dev/null
  GZ_PID=""; AP_PID=""
}
on_exit() { trap - INT TERM EXIT; cleanup; echo "cleanup done"; }
trap on_exit INT TERM EXIT

RC_TOTAL=0

# start_pair <world_sdf> <scratch_dir>
start_pair() {
  local world="$1" scratch="$2"
  gz sim -s -r --headless-rendering "$world" > "$scratch/gz.log" 2>&1 &
  GZ_PID=$!
  sleep 5
  cat > "$scratch/gdbcmds.txt" <<'GDBEOF'
set pagination off
handle SIGPIPE nostop noprint pass
run
bt
quit
GDBEOF
  # Do NOT add `-O lat,lng,alt,hdg`: SITL_cmdline.cpp silently substitutes the
  # CMAC origin AND a 584 m elevation for lat/lng 0,0, corrupting EAS2TAS. The
  # origin comes from SIM_OPOS_* in the .parm instead. See
  # docs/source_of_truth/autopilot/SITL_ATMOSPHERE_AND_AIRSPEED.md.
  ( cd "$scratch" && gdb -q -x gdbcmds.txt --args \
      "$ARDUPLANE_BIN" -w -M json \
      --defaults "$SITL_PARM" -I 0 --speedup 1 > "$scratch/arduplane.log" 2>&1 ) &
  AP_PID=$!
  sleep 8
}

# harvest <scratch_dir> <artifact_prefix>
harvest() {
  local scratch="$1" pfx="$2"
  cp "$scratch/gz.log"        "$RESULTS/${pfx}_gz_log.txt" 2>/dev/null || true
  cp "$scratch/arduplane.log" "$RESULTS/${pfx}_arduplane_log.txt" 2>/dev/null || true
  if [[ -d "$scratch/logs" ]]; then
    mkdir -p "$RESULTS/${pfx}_dataflash"
    cp "$scratch"/logs/*.BIN "$RESULTS/${pfx}_dataflash/" 2>/dev/null || true
  fi
}

run_part3() {
  echo "======================================================================"
  echo "=== part3 : OFFLINE closed-loop control-surface behaviour analysis"
  echo "===         (no simulator, reads existing navigation timeseries)"
  echo "======================================================================"
  cd "$REPO_ROOT"
  python3 "$PART3_PY" 2>&1 | tee "$RESULTS/closed_loop_control_surface_behavior_analysis_log.txt"
  [[ ${PIPESTATUS[0]} -ne 0 ]] && RC_TOTAL=1
  return 0
}

run_part1() {
  echo "======================================================================"
  echo "=== part1 : LIVE control-surface visual/joint consistency"
  echo "======================================================================"
  if [[ "$DRY_RUN" == "1" ]]; then
    cd "$REPO_ROOT" && python3 "$PART1_PY" --dry-run || RC_TOTAL=1
    return 0
  fi
  local scratch; scratch="$(mktemp -d /tmp/falcon_vjc_XXXXXX)"
  start_pair "$WORLD_RUNWAY" "$scratch"
  cd "$REPO_ROOT"
  python3 "$PART1_PY" --world "$WORLD_RUNWAY_NAME" 2>&1 \
    | tee "$RESULTS/control_surface_visual_joint_consistency_live_console.txt"
  [[ ${PIPESTATUS[0]} -ne 0 ]] && RC_TOTAL=1
  harvest "$scratch" "control_surface_visual_joint_consistency_live"
  cleanup; sleep 3
  return 0
}

run_part2() {
  echo "======================================================================"
  echo "=== part2 : six live control-surface SIGN scenarios (A..F)"
  echo "======================================================================"
  if [[ "$DRY_RUN" == "1" ]]; then
    cd "$REPO_ROOT" && python3 "$PART2_PY" --dry-run || RC_TOTAL=1
    return 0
  fi
  for S in $SIGN_SCENARIOS; do
    # BRING-UP RETRY (gazebo-testing, 2026-09-10, execution defect only).
    # The airborne bring-up is the pre-existing, already-validated
    # test_ardupilot_basic_closed_loop_flight.phase2_teleport_and_verify()
    # sequence. It is INTERMITTENT: its 0.1 s post-teleport velocity sample
    # is racy, and its own internal 4 attempts are not always enough (see the
    # historical v1_mag records in tests/gazebo/results/*_result.json, where
    # several validated stages needed 3 or 4 attempts to get one clean one).
    # A scenario that exhausts all four aborts with ABORT_BRINGUP_FAILED and
    # produces NO measurement at all. This loop simply retries the whole
    # scenario on a FRESH gz+arduplane pair. It changes no threshold, no
    # criterion, no physics parameter, and it never re-runs a scenario that
    # actually produced a measurement - only one that never got airborne.
    local ok=0
    for ATTEMPT in $(seq 1 "$SIGN_BRINGUP_ATTEMPTS"); do
      local scratch; scratch="$(mktemp -d "/tmp/falcon_sign_${S}_XXXXXX")"
      echo "--- scenario $S attempt $ATTEMPT/$SIGN_BRINGUP_ATTEMPTS : fresh gz sim + arduplane (scratch $scratch)"
      start_pair "$WORLD_AIR" "$scratch"
      cd "$REPO_ROOT"
      python3 "$PART2_PY" --scenario "$S" --world "$WORLD_AIR_NAME" 2>&1 \
        | tee "$RESULTS/control_surface_live_sign_scenario_${S}_console.txt"
      local rc=${PIPESTATUS[0]}
      harvest "$scratch" "control_surface_live_sign_scenario_${S}"
      cleanup; sleep 3
      if grep -q "BRING-UP FAILED" \
           "$RESULTS/control_surface_live_sign_scenario_${S}_console.txt"; then
        echo "--- scenario $S: bring-up did not get airborne; retrying"
        continue
      fi
      [[ $rc -ne 0 ]] && RC_TOTAL=1
      ok=1
      break
    done
    if [[ $ok -eq 0 ]]; then
      echo "--- scenario $S: NO MEASUREMENT after $SIGN_BRINGUP_ATTEMPTS bring-up attempts" >&2
      RC_TOTAL=1
    fi
  done
  cd "$REPO_ROOT"
  python3 "$PART2_PY" --summarize 2>&1 \
    | tee "$RESULTS/control_surface_live_sign_summary_log.txt"
  return 0
}

run_part4() {
  echo "======================================================================"
  echo "=== part4 : manual-takeoff ground-roll reproduction"
  echo "===         modes: $GROUND_ROLL_MODES   repeats each: $GROUND_ROLL_REPEATS"
  echo "======================================================================"
  if [[ "$DRY_RUN" == "1" ]]; then
    cd "$REPO_ROOT" && python3 "$PART4_PY" --dry-run || RC_TOTAL=1
    return 0
  fi
  for MODE in $GROUND_ROLL_MODES; do
    for i in $(seq 1 "$GROUND_ROLL_REPEATS"); do
      local scratch; scratch="$(mktemp -d "/tmp/falcon_roll_${MODE}_${i}_XXXXXX")"
      echo "--- ground roll: mode=$MODE run=$i (scratch $scratch)"
      start_pair "$WORLD_RUNWAY" "$scratch"
      cd "$REPO_ROOT"
      python3 "$PART4_PY" --world "$WORLD_RUNWAY_NAME" --mode "$MODE" \
        --run-index "$i" --repeats "$GROUND_ROLL_REPEATS" 2>&1 \
        | tee "$RESULTS/manual_takeoff_ground_roll_run${i}_${MODE}_console.txt"
      [[ ${PIPESTATUS[0]} -ne 0 ]] && RC_TOTAL=1
      harvest "$scratch" "manual_takeoff_ground_roll_run${i}_${MODE}"
      cleanup; sleep 3
    done
  done
  cd "$REPO_ROOT"
  # Summarise. When a NON-DEFAULT mode set was run (e.g. GROUND_ROLL_MODES=
  # TAKEOFF, the mode the user's own runway logs were actually flown in), the
  # summary is written under its own name so a previous campaign's
  # manual_takeoff_ground_roll_summary.json is never overwritten.
  if [[ "$GROUND_ROLL_MODES" == "MANUAL FBWA" ]]; then
    python3 "$PART4_PY" --summarize 2>&1 \
      | tee "$RESULTS/manual_takeoff_ground_roll_summary_log.txt"
  else
    local tag; tag="$(echo "$GROUND_ROLL_MODES" | tr ' ' '_')"
    for MODE in $GROUND_ROLL_MODES; do
      python3 "$PART4_PY" --summarize --summarize-mode "$MODE" \
        --summary-out "manual_takeoff_ground_roll_summary_${MODE}.json" 2>&1 \
        | tee "$RESULTS/manual_takeoff_ground_roll_summary_${MODE}_log.txt"
    done
  fi
  return 0
}

for P in "${PARTS[@]}"; do
  case "$P" in
    part1) run_part1 ;;
    part2) run_part2 ;;
    part3) run_part3 ;;
    part4) run_part4 ;;
    *) echo "ERROR: unknown part '$P' (expected part1|part2|part3|part4)" >&2
       exit 2 ;;
  esac
done

echo "======================================================================"
echo "=== campaign finished (rc=$RC_TOTAL)"
echo "=== artifacts in $RESULTS"
echo "======================================================================"
exit "$RC_TOTAL"

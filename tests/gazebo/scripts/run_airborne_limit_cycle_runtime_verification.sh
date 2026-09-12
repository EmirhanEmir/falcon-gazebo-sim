#!/usr/bin/env bash
#
# FALCON V2 - AIRBORNE_ACTUATOR_LIMIT_CYCLE_ROOT_CAUSE runtime verification
# campaign runner. Owner: gazebo-testing, 2026-09-11.
#
# CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY.
#
# WRITES NO PHYSICS PARAMETER OF ANY KIND. It does not touch model/model.sdf,
# plugins/, docs/source_of_truth/, config/ardupilot/falcon_v2_sitl.parm,
# codex/, any pre-existing world, any pre-existing test script, or any
# pre-existing result file. Every artifact it produces has a NEW name.
#
# WHAT IT RUNS
#   The EXISTING, unmodified live harness
#   tests/gazebo/scripts/test_manual_takeoff_ground_roll_reproduction.py
#   (part4 of run_manual_takeoff_and_live_surface_validation.sh), plus a
#   CONCURRENT pure-subscriber high-rate recorder
#   tests/gazebo/scripts/record_actuator_highrate.py.
#
#   baseline : world  falcon_v2_manual_takeoff_runway_world.sdf
#              model  model/model.sdf        (UNCHANGED, 20 Hz diagnostics)
#   highrate : world  falcon_v2_manual_takeoff_runway_highrate_world.sdf
#              model  tests/gazebo/models/falcon_v2_highrate_diag/model.sdf
#              (identical to model/model.sdf except two diagnostics PUBLISH
#               RATES, 20.0 -> 1000.0; see that file's header)
#
#   Both configurations are flown in MANUAL and in FBWA, which are the two
#   modes in which the 2026-09-10 offline analysis reports the aileron
#   command to be bit-constant.
#
# USAGE
#   ./run_airborne_limit_cycle_runtime_verification.sh                # all
#   ./run_airborne_limit_cycle_runtime_verification.sh baseline
#   ./run_airborne_limit_cycle_runtime_verification.sh highrate
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
SCRIPTS="$REPO_ROOT/tests/gazebo/scripts"
RESULTS="$REPO_ROOT/tests/gazebo/results"
RAW_DIR="${RAW_DIR:-/tmp/claude-1000/-home-emirhan-Desktop-FalconV2/76fa60fb-30e2-4feb-bf64-2f73f0e53cde/scratchpad/highrate_raw}"

WORLD_BASE="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_world.sdf"
WORLD_HIGH="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_highrate_world.sdf"
WORLD_NAME="falcon_v2_manual_takeoff_runway"

PART4_PY="$SCRIPTS/test_manual_takeoff_ground_roll_reproduction.py"
REC_PY="$SCRIPTS/record_actuator_highrate.py"
REC_DURATION="${REC_DURATION:-240}"

MODES="${MODES:-MANUAL FBWA}"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/aerodynamics/build:$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/actuators/build:$REPO_ROOT/plugins/wind/build:$REPO_ROOT/plugins/sensors/build:$ARDUPILOT_GAZEBO_BUILD"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

CONFIGS=("$@")
if [[ ${#CONFIGS[@]} -eq 0 ]]; then CONFIGS=(baseline highrate); fi

for f in "$ARDUPLANE_BIN" "$SITL_PARM" "$WORLD_BASE" "$WORLD_HIGH" \
         "$PART4_PY" "$REC_PY" \
         "$REPO_ROOT/tests/gazebo/models/falcon_v2_highrate_diag/model.sdf" \
         "$ARDUPILOT_GAZEBO_BUILD/libArduPilotPlugin.so"; do
  [[ -e "$f" ]] || { echo "ERROR: missing $f" >&2; exit 1; }
done
mkdir -p "$RESULTS" "$RAW_DIR"

GZ_PID=""; AP_PID=""; REC_PID=""
cleanup() {
  [[ -n "$REC_PID" ]] && kill -INT "$REC_PID" 2>/dev/null
  [[ -n "$REC_PID" ]] && wait "$REC_PID" 2>/dev/null
  REC_PID=""
  [[ -n "$AP_PID" ]] && kill -TERM "$AP_PID" 2>/dev/null
  [[ -n "$GZ_PID" ]] && { pkill -TERM -P "$GZ_PID" 2>/dev/null; kill -TERM "$GZ_PID" 2>/dev/null; }
  sleep 2
  pkill -9 -f "arduplane -w -M json" 2>/dev/null
  [[ -n "$GZ_PID" ]] && kill -KILL "$GZ_PID" 2>/dev/null
  pkill -9 -f "gz sim -s -r --headless-rendering .*falcon_v2_manual_takeoff_runway" 2>/dev/null
  pkill -9 -f "ruby.*gz sim.*falcon_v2_manual_takeoff_runway" 2>/dev/null
  GZ_PID=""; AP_PID=""
}
trap 'trap - INT TERM EXIT; cleanup; echo cleanup done' INT TERM EXIT

RC_TOTAL=0

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
  ( cd "$scratch" && gdb -q -x gdbcmds.txt --args \
      "$ARDUPLANE_BIN" -w -M json \
      --defaults "$SITL_PARM" -I 0 --speedup 1 > "$scratch/arduplane.log" 2>&1 ) &
  AP_PID=$!
  sleep 8
}

for CFG in "${CONFIGS[@]}"; do
  case "$CFG" in
    baseline) WORLD="$WORLD_BASE"; IDX=11 ;;
    highrate) WORLD="$WORLD_HIGH"; IDX=21 ;;
    *) echo "ERROR: unknown config '$CFG'" >&2; exit 2 ;;
  esac
  for MODE in $MODES; do
    TAG="${CFG}_${MODE}"
    scratch="$(mktemp -d "/tmp/falcon_lc_${TAG}_XXXXXX")"
    echo "======================================================================"
    echo "=== runtime verification: config=$CFG mode=$MODE (scratch $scratch)"
    echo "===   world: $WORLD"
    echo "======================================================================"
    start_pair "$WORLD" "$scratch"

    # high-rate pure-subscriber recorder, started BEFORE the harness so the
    # whole run including ground roll is covered
    python3 "$REC_PY" --world "$WORLD_NAME" \
      --out "$RAW_DIR/airborne_limit_cycle_${TAG}_highrate.json" \
      --duration "$REC_DURATION" 2> "$scratch/recorder.log" &
    REC_PID=$!
    sleep 1

    cd "$REPO_ROOT"
    python3 "$PART4_PY" --world "$WORLD_NAME" --mode "$MODE" \
      --run-index "$IDX" --repeats 1 2>&1 \
      | tee "$RESULTS/airborne_limit_cycle_runtime_${TAG}_console.txt"
    [[ ${PIPESTATUS[0]} -ne 0 ]] && RC_TOTAL=1

    # stop the recorder and let it flush
    [[ -n "$REC_PID" ]] && kill -INT "$REC_PID" 2>/dev/null
    [[ -n "$REC_PID" ]] && wait "$REC_PID" 2>/dev/null
    REC_PID=""
    cat "$scratch/recorder.log" || true

    # rename the harness's own artifacts into this stage's namespace so no
    # pre-existing manual_takeoff_ground_roll_* record is touched and the
    # ground-roll summary namespace is not polluted
    for suf in timeseries.json result.json log.txt; do
      s="$RESULTS/manual_takeoff_ground_roll_run${IDX}_${MODE}_${suf}"
      [[ -e "$s" ]] && mv "$s" "$RESULTS/airborne_limit_cycle_runtime_${TAG}_${suf}"
    done
    cp "$scratch/gz.log"        "$RESULTS/airborne_limit_cycle_runtime_${TAG}_gz_log.txt" 2>/dev/null || true
    cp "$scratch/arduplane.log" "$RESULTS/airborne_limit_cycle_runtime_${TAG}_arduplane_log.txt" 2>/dev/null || true
    cp "$scratch/recorder.log"  "$RESULTS/airborne_limit_cycle_runtime_${TAG}_recorder_log.txt" 2>/dev/null || true
    if [[ -d "$scratch/logs" ]]; then
      mkdir -p "$RESULTS/airborne_limit_cycle_runtime_${TAG}_dataflash"
      cp "$scratch"/logs/*.BIN "$RESULTS/airborne_limit_cycle_runtime_${TAG}_dataflash/" 2>/dev/null || true
    fi
    cleanup; sleep 3
  done
done

echo "======================================================================"
echo "=== runtime verification campaign finished (rc=$RC_TOTAL)"
echo "=== harness artifacts : $RESULTS/airborne_limit_cycle_runtime_*"
echo "=== high-rate raw     : $RAW_DIR"
echo "======================================================================"
exit "$RC_TOTAL"

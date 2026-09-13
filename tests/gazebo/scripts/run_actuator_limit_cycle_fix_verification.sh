#!/usr/bin/env bash
#
# FALCON V2 - ACTUATOR_CONTROL_LIMIT_CYCLE_FIX_AND_VALIDATION campaign runner
# (steps 4 and 5). Owner: gazebo-testing, 2026-09-12.
#
# CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY.
# WRITES NO PHYSICS PARAMETER. Touches no pre-existing world, model, plugin,
# config, source-of-truth file or pre-existing result file. Every artifact it
# produces is prefixed `actuator_fix_`.
#
# USAGE
#   ./run_actuator_limit_cycle_fix_verification.sh binaryproof  # provenance
#   ./run_actuator_limit_cycle_fix_verification.sh standalone   # STEP 4
#   ./run_actuator_limit_cycle_fix_verification.sh airborne     # STEP 5 (all 3)
#   ./run_actuator_limit_cycle_fix_verification.sh airborne_MANUAL
#   ./run_actuator_limit_cycle_fix_verification.sh airborne_FBWA
#   ./run_actuator_limit_cycle_fix_verification.sh airborne_YAW
#
# STEP 5 runs use the SAME world, model, harness, recorder and plugin set the
# 2026-09-11 pre-fix campaign used, so the two are directly comparable:
#   world  tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_highrate_world.sdf
#   model  tests/gazebo/models/falcon_v2_highrate_diag/model.sdf  (1 kHz diag)
#   flight tests/gazebo/scripts/test_manual_takeoff_ground_roll_reproduction.py
#          (UNMODIFIED - the same part4 harness, same --repeats 1)
#   1 kHz  tests/gazebo/scripts/record_actuator_highrate.py (UNMODIFIED, pure
#          subscriber)
# The ONLY new flight script is test_actuator_fix_yaw_response.py, used for
# the YAW_RESPONSE_TEST, which needs a stick input the existing harness never
# makes.
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SCRIPTS="$REPO_ROOT/tests/gazebo/scripts"
RESULTS="$REPO_ROOT/tests/gazebo/results"

# Full plugin path, matching what the flight harness
# run_airborne_limit_cycle_runtime_verification.sh exports, so the standalone
# bench loads EXACTLY the same set of plugin binaries the flight runs do.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/aerodynamics/build:$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/actuators/build:$REPO_ROOT/plugins/wind/build:$REPO_ROOT/plugins/sensors/build:$ARDUPILOT_GAZEBO_BUILD"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

mkdir -p "$RESULTS"

binaryproof() {
  # Records, into a result artifact, HOW the loaded actuator binary was shown
  # to be the rebuilt one rather than a stale copy. See the test report.
  local out="$RESULTS/actuator_fix_binary_provenance.txt"
  {
    echo "=== date ==="; date -Is
    echo "=== every libFalconV2Actuators.so on this machine ==="
    find / -name 'libFalconV2Actuators.so' -not -path '*/proc/*' 2>/dev/null -printf '%T+  %10s  %p\n'
    echo "=== GZ_SIM_SYSTEM_PLUGIN_PATH ==="; echo "$GZ_SIM_SYSTEM_PLUGIN_PATH"
    echo "=== md5 of the one on the path ==="
    md5sum "$REPO_ROOT/plugins/actuators/build/libFalconV2Actuators.so"
    echo "=== source vs object vs library mtimes ==="
    ls -la --time-style=full-iso \
      "$REPO_ROOT/plugins/actuators/ActuatorModel.hh" \
      "$REPO_ROOT/plugins/actuators/ActuatorSystem.cc" \
      "$REPO_ROOT/plugins/actuators/build/CMakeFiles/FalconV2Actuators.dir/ActuatorSystem.cc.o" \
      "$REPO_ROOT/plugins/actuators/build/libFalconV2Actuators.so"
    echo "=== rebuild must be a no-op and must not change the md5 ==="
    cmake --build "$REPO_ROOT/plugins/actuators/build" 2>&1 | tail -5
    md5sum "$REPO_ROOT/plugins/actuators/build/libFalconV2Actuators.so"
    echo "=== F3 constant kRateSaturationDetectFraction = 0.999 present in .rodata? ==="
    python3 - <<'PY'
import struct, glob
pat = struct.pack('<d', 0.999)
for p in ["/home/emirhan/Desktop/FalconV2/plugins/actuators/build/libFalconV2Actuators.so"]:
    d = open(p, 'rb').read()
    print(f"{p}: occurrences of IEEE754(0.999)={d.count(pat)}")
PY
    echo "=== config_path each test model hands the plugin ==="
    grep -n -A1 'plugin filename="FalconV2Actuators"' \
      "$REPO_ROOT/model/model.sdf" \
      "$REPO_ROOT/tests/gazebo/models/falcon_v2_highrate_diag/model.sdf"
  } > "$out" 2>&1
  echo "wrote $out"
}

standalone() {
  cd "$SCRIPTS" || exit 1
  python3 "$SCRIPTS/test_actuator_standalone_step_response.py" \
    2>&1 | tee "$RESULTS/actuator_fix_standalone_console.txt"
  return "${PIPESTATUS[0]}"
}

# ---------------------------------------------------------------------------
# STEP 5 - short airborne regression
# ---------------------------------------------------------------------------
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
WORLD_HIGH="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_highrate_world.sdf"
WORLD_NAME="falcon_v2_manual_takeoff_runway"
PART4_PY="$SCRIPTS/test_manual_takeoff_ground_roll_reproduction.py"
YAW_PY="$SCRIPTS/test_actuator_fix_yaw_response.py"
REC_PY="$SCRIPTS/record_actuator_highrate.py"
RAW_DIR="${RAW_DIR:-/tmp/claude-1000/-home-emirhan-Desktop-FalconV2/d791f088-f001-45d8-802a-39118ca168e8/scratchpad/actuator_fix_highrate_raw}"
REC_DURATION="${REC_DURATION:-160}"

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
  ( cd "$scratch" && gdb -q -x gdbcmds.txt --args       "$ARDUPLANE_BIN" -w -M json       --defaults "$SITL_PARM" -I 0 --speedup 1 > "$scratch/arduplane.log" 2>&1 ) &
  AP_PID=$!
  sleep 8
}

airborne_run() {
  local MODE="$1"        # MANUAL | FBWA | YAW
  local TAG="$MODE"
  local scratch
  scratch="$(mktemp -d "/tmp/falcon_actfix_${TAG}_XXXXXX")"
  echo "======================================================================"
  echo "=== STEP 5 airborne regression: $TAG (scratch $scratch)"
  echo "===   world: $WORLD_HIGH"
  echo "======================================================================"
  trap 'trap - INT TERM EXIT; cleanup; echo cleanup done' INT TERM EXIT
  start_pair "$WORLD_HIGH" "$scratch"

  python3 "$REC_PY" --world "$WORLD_NAME"     --out "$RAW_DIR/actuator_fix_airborne_${TAG}_highrate.json"     --duration "$REC_DURATION" 2> "$scratch/recorder.log" &
  REC_PID=$!
  sleep 1

  cd "$REPO_ROOT" || exit 1
  if [[ "$MODE" == "YAW" ]]; then
    python3 "$YAW_PY" --mode FBWA --tag FBWA 2>&1       | tee "$RESULTS/actuator_fix_airborne_${TAG}_console.txt"
    local rc=${PIPESTATUS[0]}
  else
    python3 "$PART4_PY" --world "$WORLD_NAME" --mode "$MODE"       --run-index 31 --repeats 1 2>&1       | tee "$RESULTS/actuator_fix_airborne_${TAG}_console.txt"
    local rc=${PIPESTATUS[0]}
  fi

  [[ -n "$REC_PID" ]] && kill -INT "$REC_PID" 2>/dev/null
  [[ -n "$REC_PID" ]] && wait "$REC_PID" 2>/dev/null
  REC_PID=""
  cat "$scratch/recorder.log" || true

  # Rename the part4 harness's own artifacts into this stage's namespace so no
  # pre-existing manual_takeoff_ground_roll_* record is touched.
  for suf in timeseries.json result.json log.txt; do
    s="$RESULTS/manual_takeoff_ground_roll_run31_${MODE}_${suf}"
    [[ -e "$s" ]] && mv "$s" "$RESULTS/actuator_fix_airborne_${TAG}_${suf}"
  done
  cp "$scratch/gz.log"        "$RESULTS/actuator_fix_airborne_${TAG}_gz_log.txt" 2>/dev/null || true
  cp "$scratch/arduplane.log" "$RESULTS/actuator_fix_airborne_${TAG}_arduplane_log.txt" 2>/dev/null || true
  cp "$scratch/recorder.log"  "$RESULTS/actuator_fix_airborne_${TAG}_recorder_log.txt" 2>/dev/null || true
  if [[ -d "$scratch/logs" ]]; then
    mkdir -p "$RESULTS/actuator_fix_airborne_${TAG}_dataflash"
    cp "$scratch"/logs/*.BIN "$RESULTS/actuator_fix_airborne_${TAG}_dataflash/" 2>/dev/null || true
  fi
  cleanup; sleep 3
  trap - INT TERM EXIT
  return "$rc"
}

RC=0
for what in "$@"; do
  case "$what" in
    binaryproof) binaryproof ;;
    standalone)  standalone || RC=1 ;;
    airborne)        for m in MANUAL FBWA YAW; do airborne_run "$m" || RC=1; done ;;
    airborne_MANUAL) airborne_run MANUAL || RC=1 ;;
    airborne_FBWA)   airborne_run FBWA   || RC=1 ;;
    airborne_YAW)    airborne_run YAW    || RC=1 ;;
    *) echo "unknown target '$what'" >&2; exit 2 ;;
  esac
done
exit "$RC"

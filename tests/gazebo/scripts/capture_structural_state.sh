#!/usr/bin/env bash
# FALCON V2 - structural V1 test pass, gazebo-testing, 2026-08-21
#
# Loads model/model.sdf (via tests/gazebo/worlds/falcon_v2_zero_g_world.sdf)
# into a real running Gazebo Sim Harmonic server (not just gz sdf --check)
# and captures, via the independent `gz model` CLI client (a separate
# transport-based query path from the SDF text itself), the model/link/joint
# state as actually loaded by the physics engine. This is raw evidence for:
#   - MODEL_LOAD_TEST
#   - CONTROL_JOINT_EXISTENCE_TEST (type, parent, child)
#   - PROP_JOINT_EXISTENCE_TEST / PROP_JOINT_AXIS_TEST
#   - MASS_CG_INERTIA_TEST (per-link mass / inertial pose / inertia matrix)
#
# Usage:
#   tests/gazebo/scripts/capture_structural_state.sh [output_dir]
#
# Never modifies model/model.sdf. Exits non-zero if the server fails to
# start or the model fails to appear.

set -u
REPO_ROOT="/home/emirhan/Desktop/FalconV2"
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_zero_g_world.sdf"
OUT_DIR="${1:-$REPO_ROOT/tests/gazebo/results}"
mkdir -p "$OUT_DIR"
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
LOG="$OUT_DIR/structural_state_${STAMP}.txt"
SERVER_LOG="$OUT_DIR/server_log_${STAMP}.txt"

echo "=== FALCON V2 structural state capture: $STAMP ===" | tee "$LOG"
echo "World: $WORLD" | tee -a "$LOG"

# 1) Static SDF validity check (fast, always run first).
echo "" | tee -a "$LOG"
echo "--- gz sdf --check (model/model.sdf) ---" | tee -a "$LOG"
gz sdf --check "$REPO_ROOT/model/model.sdf" 2>&1 | tee -a "$LOG"
SDF_CHECK_STATUS=${PIPESTATUS[0]}

echo "" | tee -a "$LOG"
echo "--- gz sdf --check (test world) ---" | tee -a "$LOG"
gz sdf --check "$WORLD" 2>&1 | tee -a "$LOG"
WORLD_CHECK_STATUS=${PIPESTATUS[0]}

# 2) Actually launch a real server (independent of the static checker) and
#    query it over transport.
# stdbuf forces line-buffered stdout/stderr so the log is not lost if the
# process later has to be killed with SIGKILL (observed necessary in this
# environment - see stop_server below).
stdbuf -oL -eL gz sim -s -r -v 3 "$WORLD" > "$SERVER_LOG" 2>&1 &
SERVER_PID=$!
# Note: this environment's `gz sim` server has been observed not to exit on
# SIGTERM within a reasonable window when launched this way; the cleanup
# helper below escalates to SIGKILL rather than hanging indefinitely.
stop_server() {
  kill -TERM "$SERVER_PID" 2>/dev/null
  for i in $(seq 1 6); do
    kill -0 "$SERVER_PID" 2>/dev/null || return 0
    sleep 0.5
  done
  kill -KILL "$SERVER_PID" 2>/dev/null
}
trap 'stop_server' EXIT

# Wait for the server to come up and the model/service to be queryable.
READY=0
for i in $(seq 1 20); do
  sleep 0.5
  if gz model --list 2>/dev/null | grep -q "falcon_v2"; then
    READY=1
    break
  fi
done

echo "" | tee -a "$LOG"
echo "--- gz model --list ---" | tee -a "$LOG"
gz model --list 2>&1 | tee -a "$LOG"

if [ "$READY" -ne 1 ]; then
  echo "FAIL: falcon_v2 model did not appear on the running server within 10s" | tee -a "$LOG"
  kill "$SERVER_PID" 2>/dev/null
  exit 1
fi

echo "" | tee -a "$LOG"
echo "--- gz model -m falcon_v2 -p (model pose) ---" | tee -a "$LOG"
gz model -m falcon_v2 -p 2>&1 | tee -a "$LOG"

echo "" | tee -a "$LOG"
echo "--- gz model -m falcon_v2 -l (all links: mass, inertial pose, inertia matrix, link pose) ---" | tee -a "$LOG"
gz model -m falcon_v2 -l 2>&1 | tee -a "$LOG"

echo "" | tee -a "$LOG"
echo "--- gz model -m falcon_v2 -j (all joints: type, parent/child link, axis) ---" | tee -a "$LOG"
gz model -m falcon_v2 -j 2>&1 | tee -a "$LOG"

stop_server
trap - EXIT

echo "" | tee -a "$LOG"
echo "--- server stderr/stdout tail (last 40 lines, checked for Err/Wrn) ---" | tee -a "$LOG"
tail -n 40 "$SERVER_LOG" | tee -a "$LOG"
echo "" | tee -a "$LOG"
echo "--- grep for Err/error in server log ---" | tee -a "$LOG"
grep -i "\[Err\]\|error\|exception\|segfault" "$SERVER_LOG" | tee -a "$LOG"
ERR_COUNT=$(grep -ic "\[Err\]\|exception\|segfault" "$SERVER_LOG")

echo "" | tee -a "$LOG"
echo "sdf_check_status=$SDF_CHECK_STATUS world_check_status=$WORLD_CHECK_STATUS model_ready=$READY err_count_in_server_log=$ERR_COUNT" | tee -a "$LOG"
echo "Log saved to: $LOG"
echo "Server log saved to: $SERVER_LOG"

if [ "$SDF_CHECK_STATUS" -ne 0 ] || [ "$WORLD_CHECK_STATUS" -ne 0 ] || [ "$READY" -ne 1 ] || [ "$ERR_COUNT" -ne 0 ]; then
  exit 1
fi
exit 0

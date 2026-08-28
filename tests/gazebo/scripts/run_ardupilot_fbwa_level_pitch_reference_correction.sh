#!/usr/bin/env bash
#
# FALCON V2 - ARDUPLANE_FBWA_LEVEL_PITCH_REFERENCE_CORRECTION - Step 3 launch
# helper (gazebo-testing, 2026-08-29).
#
# Launches ONE fresh `gz sim` + gdb-wrapped `arduplane` pair (the proven
# sequence from docs/test_results/2026-08-28_ardupilot_trim_reference_
# correction_validation.md sec 3 and run_ardupilot_longitudinal_equilibrium_
# c3.sh), runs the ONE neutral-stick FBWA level-pitch-reference segment, then
# tears both processes down.
#
# Creates/modifies NO aircraft physics parameter, NO SDF, NO .parm file - it
# only sets env vars, launches existing binaries, and runs the existing python
# test. `arduplane -w` wipes its own scratch eeprom; the checked-in
# config/ardupilot/falcon_v2_sitl.parm (now containing PTCH_TRIM_DEG 2.49,
# added by controls-integration) is read-only input.
#
# Usage:
#   ./tests/gazebo/scripts/run_ardupilot_fbwa_level_pitch_reference_correction.sh
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf"
TEST_PY="$REPO_ROOT/tests/gazebo/scripts/test_ardupilot_fbwa_level_pitch_reference_correction.py"
LOG_OUT="$REPO_ROOT/tests/gazebo/results/ardupilot_fbwa_level_pitch_reference_correction_log.txt"
SCRATCH="$(mktemp -d /tmp/falcon_fbwa_pitchref_XXXXXX)"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/aerodynamics/build:$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/actuators/build:$REPO_ROOT/plugins/wind/build:$REPO_ROOT/plugins/sensors/build:$ARDUPILOT_GAZEBO_BUILD"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

GZ_PID=""; AP_PID=""
cleanup() {
  trap - INT TERM EXIT
  [[ -n "$AP_PID" ]] && kill -TERM "$AP_PID" 2>/dev/null
  [[ -n "$GZ_PID" ]] && { pkill -TERM -P "$GZ_PID" 2>/dev/null; kill -TERM "$GZ_PID" 2>/dev/null; }
  sleep 2
  pkill -9 -f "arduplane -w -M json" 2>/dev/null
  [[ -n "$GZ_PID" ]] && kill -KILL "$GZ_PID" 2>/dev/null
  pkill -9 -f "gz sim -s -r --headless-rendering .*ardupilot_basic_closed_loop" 2>/dev/null
  pkill -9 -f "ruby.*gz sim.*ardupilot_basic_closed_loop" 2>/dev/null
  echo "cleanup done"
}
trap cleanup INT TERM EXIT

echo "=== FBWA level-pitch-ref : launching fresh gz sim + arduplane pair (scratch $SCRATCH) ==="
gz sim -s -r --headless-rendering "$WORLD" > "$SCRATCH/gz.log" 2>&1 &
GZ_PID=$!
sleep 5

cat > "$SCRATCH/gdbcmds.txt" <<'EOF'
set pagination off
handle SIGPIPE nostop noprint pass
run
bt
quit
EOF

( cd "$SCRATCH" && gdb -q -x gdbcmds.txt --args \
    "$ARDUPLANE_BIN" -w -M json -O 0,0,0,0 \
    --defaults "$SITL_PARM" -I 0 --speedup 1 > "$SCRATCH/arduplane.log" 2>&1 ) &
AP_PID=$!
sleep 8

echo "=== FBWA level-pitch-ref : running test ==="
cd "$REPO_ROOT"
python3 "$TEST_PY" 2>&1 | tee "$LOG_OUT"
RC=${PIPESTATUS[0]}

echo "=== FBWA level-pitch-ref : done (rc=$RC), logs in $SCRATCH ==="
cp "$SCRATCH/gz.log" "$REPO_ROOT/tests/gazebo/results/ardupilot_fbwa_level_pitch_reference_correction_gz_log.txt" 2>/dev/null || true
cp "$SCRATCH/arduplane.log" "$REPO_ROOT/tests/gazebo/results/ardupilot_fbwa_level_pitch_reference_correction_arduplane_log.txt" 2>/dev/null || true
exit $RC

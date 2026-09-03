#!/usr/bin/env bash
#
# FALCON V2 - SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION
# Airspeed/wind acceptance matrix + short TECS regression - launch helper
# (gazebo-testing, 2026-09-02).
#
# Launches ONE fresh `gz sim` + gdb-wrapped `arduplane` pair per wind case
# (same proven sequence as run_ardupilot_tecs_cruise_speed_hold.sh - see
# docs/test_results/2026-08-28_ardupilot_trim_reference_correction_validation.md
# sec 3 for why arduplane runs under gdb), runs one wind case, tears both
# processes down, and copies out the dataflash .BIN.
#
# A FRESH PROCESS PAIR PER CASE IS DELIBERATE: each case must start from an
# identical, uncontaminated ArduPlane state (wiped EEPROM, fresh EKF, fresh
# TECS integrators) so the three cases are directly comparable. Re-using one
# process across cases would leave the previous case's integrator/EKF wind
# state in the loop.
#
# Creates/modifies NO aircraft physics parameter, NO SDF, NO plugin, NO world,
# NO .parm file. `arduplane -w` wipes only its own scratch EEPROM; the
# checked-in config/ardupilot/falcon_v2_sitl.parm is READ-ONLY input.
# NO `-O` flag is passed (see run_ardupilot_tecs_cruise_speed_hold.sh:93 and
# docs/source_of_truth/autopilot/SITL_ATMOSPHERE_AND_AIRSPEED.md sec 1).
#
# WIND: the test world is NOT modified. FalconV2Wind is a MODEL-level plugin
# in model/model.sdf, present in every world, defaulting to <steady_wind_mps>
# 0 0 0</steady_wind_mps>, and commandable live on
# /model/falcon_v2/wind/steady_cmd. The Python test does the commanding.
#
# Expected wall-clock: ~4 min per case (~5 min for the `zero` case, which flies
# one extra matched-groundspeed segment). Outputs per case:
#   tests/gazebo/results/airspeed_wind_acceptance_<case>_result.json
#   tests/gazebo/results/airspeed_wind_acceptance_<case>_timeseries.json
#   tests/gazebo/results/airspeed_wind_acceptance_<case>_log.txt
#   tests/gazebo/results/airspeed_wind_acceptance_<case>_gz_log.txt
#   tests/gazebo/results/airspeed_wind_acceptance_<case>_arduplane_log.txt
#   tests/gazebo/results/airspeed_wind_acceptance_<case>_dataflash/*.BIN
#       (ArduPlane's own logs - the ONLY source of ARSP.* and TECS.*, neither
#        of which is exposed over MAVLink. Parsed afterwards by
#        analyze_dataflash_airspeed.py.)
#
# Usage:
#   ./tests/gazebo/scripts/run_ardupilot_airspeed_wind_acceptance.sh zero
#   ./tests/gazebo/scripts/run_ardupilot_airspeed_wind_acceptance.sh headwind
#   ./tests/gazebo/scripts/run_ardupilot_airspeed_wind_acceptance.sh tailwind
#   ./tests/gazebo/scripts/run_ardupilot_airspeed_wind_acceptance.sh all
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf"
TEST_PY="$REPO_ROOT/tests/gazebo/scripts/test_ardupilot_airspeed_wind_acceptance.py"
RESULTS="$REPO_ROOT/tests/gazebo/results"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/aerodynamics/build:$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/actuators/build:$REPO_ROOT/plugins/wind/build:$REPO_ROOT/plugins/sensors/build:$ARDUPILOT_GAZEBO_BUILD"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

run_case() {
  local CASE="$1"
  local SCRATCH
  SCRATCH="$(mktemp -d "/tmp/falcon_aswind_${CASE}_XXXXXX")"
  local LOG_OUT="$RESULTS/airspeed_wind_acceptance_${CASE}_log.txt"
  local GZ_PID="" AP_PID=""

  cleanup() {
    trap - INT TERM EXIT
    [[ -n "$AP_PID" ]] && kill -TERM "$AP_PID" 2>/dev/null
    [[ -n "$GZ_PID" ]] && { pkill -TERM -P "$GZ_PID" 2>/dev/null; kill -TERM "$GZ_PID" 2>/dev/null; }
    sleep 2
    pkill -9 -f "arduplane -w -M json" 2>/dev/null
    [[ -n "$GZ_PID" ]] && kill -KILL "$GZ_PID" 2>/dev/null
    pkill -9 -f "gz sim -s -r --headless-rendering .*ardupilot_basic_closed_loop" 2>/dev/null
    pkill -9 -f "ruby.*gz sim.*ardupilot_basic_closed_loop" 2>/dev/null
    echo "cleanup done ($CASE)"
  }
  trap cleanup INT TERM EXIT

  echo "=== airspeed/wind acceptance [$CASE] : launching gz sim + arduplane (scratch $SCRATCH) ==="
  gz sim -s -r --headless-rendering "$WORLD" > "$SCRATCH/gz.log" 2>&1 &
  GZ_PID=$!
  sleep 5

  cat > "$SCRATCH/gdbcmds.txt" <<'GDBEOF'
set pagination off
handle SIGPIPE nostop noprint pass
run
bt
quit
GDBEOF

  ( cd "$SCRATCH" && gdb -q -x gdbcmds.txt --args \
      "$ARDUPLANE_BIN" -w -M json \
      --defaults "$SITL_PARM" -I 0 --speedup 1 > "$SCRATCH/arduplane.log" 2>&1 ) &
  AP_PID=$!
  sleep 8

  echo "=== airspeed/wind acceptance [$CASE] : running test ==="
  cd "$REPO_ROOT"
  python3 "$TEST_PY" "$CASE" 2>&1 | tee "$LOG_OUT"
  local RC=${PIPESTATUS[0]}

  echo "=== airspeed/wind acceptance [$CASE] : done (rc=$RC) ==="
  mkdir -p "$RESULTS"
  cp "$SCRATCH/gz.log"        "$RESULTS/airspeed_wind_acceptance_${CASE}_gz_log.txt" 2>/dev/null || true
  cp "$SCRATCH/arduplane.log" "$RESULTS/airspeed_wind_acceptance_${CASE}_arduplane_log.txt" 2>/dev/null || true
  if [[ -d "$SCRATCH/logs" ]]; then
    rm -rf "$RESULTS/airspeed_wind_acceptance_${CASE}_dataflash"
    mkdir -p "$RESULTS/airspeed_wind_acceptance_${CASE}_dataflash"
    cp "$SCRATCH"/logs/*.BIN "$RESULTS/airspeed_wind_acceptance_${CASE}_dataflash/" 2>/dev/null || true
  fi
  cleanup
  return $RC
}

CASES=("${1:-all}")
if [[ "${CASES[0]}" == "all" ]]; then
  CASES=(zero headwind tailwind)
fi

OVERALL=0
for c in "${CASES[@]}"; do
  run_case "$c"
  rc=$?
  echo "CASE $c -> rc=$rc"
  [[ $rc -ne 0 ]] && OVERALL=$rc
  sleep 3
done
exit $OVERALL

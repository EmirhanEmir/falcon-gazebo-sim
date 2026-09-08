#!/usr/bin/env bash
#
# FALCON V2 - ARDUPLANE_NAVIGATION_AND_AUTOTUNE_VALIDATION - campaign runner
# (controls-integration, 2026-09-07). Executed by `gazebo-testing`.
#
# Runs the four navigation scenarios, EACH IN ITS OWN FRESH gz sim + gdb-wrapped
# arduplane pair (so no scenario can inherit state from another), then writes the
# combined summary.
#
#   A  GUIDED_SINGLE_TARGET          one GUIDED target waypoint
#   B  LOITER_ORBIT                  orbit about the latched loiter centre
#   C  RTL_RETURN_TO_HOME            GUIDED outbound leg, then RTL
#   D  GUIDED_SEQUENTIAL_WAYPOINTS   3-waypoint route (NOT an AUTO mission),
#                                    both turn directions, one +15 m step
#
# WRITES NO PARAMETER. config/ardupilot/falcon_v2_sitl.parm is READ-ONLY input.
# No SDF, plugin, aero, propulsion, actuator, sensor, mass/CG/inertia change.
# DOES NOT RUN AUTOTUNE (the procedure is a dormant stub inside the test
# module: AUTOTUNE_PROCEDURE / autotune_procedure_stub()).
#
# WORLD: falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf - the SAME world
# every recent validated ArduPlane stage used (TECS cruise, TECS climb/descent,
# TECS PTCH_DAMP regression, phugoid damping). ZERO WIND: the world declares no
# wind system and model/model.sdf's FalconV2Wind plugin defaults to
# <steady_wind_mps>0 0 0; SIM_WIND_SPD is additionally live-read and asserted 0.
#
# PRE-FLIGHT: run ./run_ardupilot_navigation_preflight.sh FIRST. This campaign
# assumes the live parameter state that pre-flight verifies.
#
# Expected wall-clock: ~20-25 min for all four scenarios.
# Outputs (tests/gazebo/results/):
#   ardupilot_navigation_scenario_<S>_result.json      summary + PASS/FAIL
#   ardupilot_navigation_scenario_<S>_timeseries.json  full 20 Hz raw record
#   ardupilot_navigation_scenario_<S>_log.txt
#   ardupilot_navigation_scenario_<S>_gz_log.txt
#   ardupilot_navigation_scenario_<S>_arduplane_log.txt
#   ardupilot_navigation_scenario_<S>_dataflash/*.BIN  (ArduPlane's own logs -
#       carry NTUN/TECS/PIDR/PIDP messages that are NOT exposed over MAVLink;
#       copied out for validation, not parsed by the test)
#   ardupilot_navigation_validation_summary.json       combined campaign verdict
#
# Usage:
#   ./tests/gazebo/scripts/run_ardupilot_navigation_validation.sh            # A B C D + summary
#   ./tests/gazebo/scripts/run_ardupilot_navigation_validation.sh B D        # subset
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf"
TEST_PY="$REPO_ROOT/tests/gazebo/scripts/test_ardupilot_navigation_validation.py"
RESULTS="$REPO_ROOT/tests/gazebo/results"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/aerodynamics/build:$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/actuators/build:$REPO_ROOT/plugins/wind/build:$REPO_ROOT/plugins/sensors/build:$ARDUPILOT_GAZEBO_BUILD"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

SCENARIOS=("$@")
if [[ ${#SCENARIOS[@]} -eq 0 ]]; then
  SCENARIOS=(A B C D)
fi

for f in \
  "$REPO_ROOT/plugins/aerodynamics/build/libFalconV2Aerodynamics.so" \
  "$REPO_ROOT/plugins/propulsion/build/libFalconV2Propulsion.so" \
  "$REPO_ROOT/plugins/actuators/build/libFalconV2Actuators.so" \
  "$REPO_ROOT/plugins/wind/build/libFalconV2Wind.so" \
  "$REPO_ROOT/plugins/sensors/build/libFalconV2Pitot.so" \
  "$ARDUPILOT_GAZEBO_BUILD/libArduPilotPlugin.so" \
  "$ARDUPLANE_BIN" "$SITL_PARM" "$WORLD" "$TEST_PY"
do
  [[ -e "$f" ]] || { echo "ERROR: missing $f" >&2; exit 1; }
done

GZ_PID=""; AP_PID=""; SCRATCH=""
cleanup() {
  [[ -n "$AP_PID" ]] && kill -TERM "$AP_PID" 2>/dev/null
  [[ -n "$GZ_PID" ]] && { pkill -TERM -P "$GZ_PID" 2>/dev/null; kill -TERM "$GZ_PID" 2>/dev/null; }
  sleep 2
  pkill -9 -f "arduplane -w -M json" 2>/dev/null
  [[ -n "$GZ_PID" ]] && kill -KILL "$GZ_PID" 2>/dev/null
  pkill -9 -f "gz sim -s -r --headless-rendering .*ardupilot_basic_closed_loop" 2>/dev/null
  pkill -9 -f "ruby.*gz sim.*ardupilot_basic_closed_loop" 2>/dev/null
  GZ_PID=""; AP_PID=""
}
on_exit() { trap - INT TERM EXIT; cleanup; echo "cleanup done"; }
trap on_exit INT TERM EXIT

RC_TOTAL=0
for S in "${SCENARIOS[@]}"; do
  SCRATCH="$(mktemp -d "/tmp/falcon_nav_${S}_XXXXXX")"
  LOG_OUT="$RESULTS/ardupilot_navigation_scenario_${S}_log.txt"
  echo "======================================================================"
  echo "=== navigation scenario $S : fresh gz sim + arduplane (scratch $SCRATCH)"
  echo "======================================================================"
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

  # Do NOT re-add `-O lat,lng,alt,hdg`: SITL_cmdline.cpp:761-766 silently
  # substitutes the CMAC origin AND a 584 m elevation for lat/lng 0,0, which
  # corrupts EAS2TAS. The origin comes from SIM_OPOS_* in the .parm instead.
  # See docs/source_of_truth/autopilot/SITL_ATMOSPHERE_AND_AIRSPEED.md.
  ( cd "$SCRATCH" && gdb -q -x gdbcmds.txt --args \
      "$ARDUPLANE_BIN" -w -M json \
      --defaults "$SITL_PARM" -I 0 --speedup 1 > "$SCRATCH/arduplane.log" 2>&1 ) &
  AP_PID=$!
  sleep 8

  cd "$REPO_ROOT"
  python3 "$TEST_PY" --scenario "$S" 2>&1 | tee "$LOG_OUT"
  RC=${PIPESTATUS[0]}
  [[ $RC -ne 0 ]] && RC_TOTAL=1
  echo "=== navigation scenario $S : done (rc=$RC) ==="

  mkdir -p "$RESULTS"
  cp "$SCRATCH/gz.log"        "$RESULTS/ardupilot_navigation_scenario_${S}_gz_log.txt" 2>/dev/null || true
  cp "$SCRATCH/arduplane.log" "$RESULTS/ardupilot_navigation_scenario_${S}_arduplane_log.txt" 2>/dev/null || true
  if [[ -d "$SCRATCH/logs" ]]; then
    mkdir -p "$RESULTS/ardupilot_navigation_scenario_${S}_dataflash"
    cp "$SCRATCH"/logs/*.BIN "$RESULTS/ardupilot_navigation_scenario_${S}_dataflash/" 2>/dev/null || true
  fi
  cleanup
  sleep 3
done

echo "======================================================================"
echo "=== navigation campaign : combined summary"
echo "======================================================================"
cd "$REPO_ROOT"
python3 "$TEST_PY" --summarize 2>&1 | tee "$RESULTS/ardupilot_navigation_validation_summary_log.txt"
SUM_RC=${PIPESTATUS[0]}
exit $(( RC_TOTAL || SUM_RC ))

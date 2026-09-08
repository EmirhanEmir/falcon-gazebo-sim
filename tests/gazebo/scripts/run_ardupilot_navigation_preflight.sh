#!/usr/bin/env bash
#
# FALCON V2 - ARDUPLANE_NAVIGATION_AND_AUTOTUNE_VALIDATION - PRE-FLIGHT runner
# (controls-integration, 2026-09-07).
#
# Brings up ONE fresh `gz sim` + gdb-wrapped `arduplane` pair (byte-identical
# launch sequence to run_ardupilot_tecs_cruise_speed_hold.sh - see
# docs/test_results/2026-08-28_ardupilot_trim_reference_correction_validation.md
# sec 3 for why arduplane runs under gdb), runs the SHORT pre-flight readback +
# sign-sanity check, then tears both processes down.
#
# WRITES NO PARAMETER. config/ardupilot/falcon_v2_sitl.parm is read-only input.
# No SDF, plugin, aero/propulsion/actuator/sensor value is touched.
#
# Expected wall-clock: ~3 min.
# Outputs:
#   tests/gazebo/results/ardupilot_navigation_preflight_result.json
#   tests/gazebo/results/ardupilot_navigation_preflight_log.txt
#   tests/gazebo/results/ardupilot_navigation_preflight_gz_log.txt
#   tests/gazebo/results/ardupilot_navigation_preflight_arduplane_log.txt
#
# Usage:
#   ./tests/gazebo/scripts/run_ardupilot_navigation_preflight.sh
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
# SAME world as every recent TECS stage (run_ardupilot_tecs_cruise_speed_hold.sh,
# run_ardupilot_tecs_climb_descent_energy.sh, run_ardupilot_longitudinal_
# phugoid_damping.sh). ZERO WIND: the world declares no wind system, and
# model/model.sdf's FalconV2Wind plugin defaults to <steady_wind_mps>0 0 0.
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf"
TEST_PY="$REPO_ROOT/tests/gazebo/scripts/test_ardupilot_navigation_preflight.py"
RESULTS="$REPO_ROOT/tests/gazebo/results"
LOG_OUT="$RESULTS/ardupilot_navigation_preflight_log.txt"
SCRATCH="$(mktemp -d /tmp/falcon_nav_preflight_XXXXXX)"

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

echo "=== nav pre-flight : launching fresh gz sim + arduplane pair (scratch $SCRATCH) ==="
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

echo "=== nav pre-flight : running check ==="
cd "$REPO_ROOT"
python3 "$TEST_PY" 2>&1 | tee "$LOG_OUT"
RC=${PIPESTATUS[0]}

echo "=== nav pre-flight : done (rc=$RC), logs in $SCRATCH ==="
mkdir -p "$RESULTS"
cp "$SCRATCH/gz.log"        "$RESULTS/ardupilot_navigation_preflight_gz_log.txt" 2>/dev/null || true
cp "$SCRATCH/arduplane.log" "$RESULTS/ardupilot_navigation_preflight_arduplane_log.txt" 2>/dev/null || true
exit $RC

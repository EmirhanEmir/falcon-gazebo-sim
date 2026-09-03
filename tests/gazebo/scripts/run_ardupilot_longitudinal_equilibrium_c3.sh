#!/usr/bin/env bash
#
# FALCON V2 - ARDUPLANE_LONGITUDINAL_EQUILIBRIUM_AND_SINK_ROOT_CAUSE_VALIDATION
# Part C.3 launch helper (gazebo-testing, 2026-08-28).
#
# Launches ONE fresh `gz sim` + gdb-wrapped `arduplane` pair (the proven
# sequence from docs/test_results/2026-08-28_ardupilot_trim_reference_
# correction_validation.md sec 3), runs ONE Part C.3 cross-check flight, then
# tears both processes down. A FRESH pair per flight, per the task instruction.
#
# Creates/modifies NO aircraft physics parameter, NO SDF, NO .parm file - it
# only sets env vars, launches existing binaries, and runs the existing python
# test. `arduplane -w` wipes its own scratch eeprom; the checked-in
# config/ardupilot/falcon_v2_sitl.parm is read-only input.
#
# Usage:
#   ./tests/gazebo/scripts/run_ardupilot_longitudinal_equilibrium_c3.sh {fbwa_throttlestar|fbwa_original|manual}
#
set -uo pipefail

RUN="${1:-}"
case "$RUN" in
  fbwa_throttlestar|fbwa_original|manual) ;;
  *) echo "usage: $0 {fbwa_throttlestar|fbwa_original|manual}" >&2; exit 2 ;;
esac

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf"
SCRATCH="$(mktemp -d /tmp/falcon_c3_${RUN}_XXXXXX)"

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
  echo "cleanup done"
}
trap cleanup INT TERM EXIT

echo "=== C.3 $RUN : launching fresh gz sim + arduplane pair (scratch $SCRATCH) ==="
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

# NOTE (gazebo-testing, 2026-09-02, stage SITL_ATMOSPHERE_AND_PITOT_
# INTEGRATION_VALIDATION, Task C housekeeping): `-O 0,0,0,0` was REMOVED here.
# It never worked as intended: AP_HAL_SITL/SITL_cmdline.cpp:761-766
# (parse_home) replaces a lat/lng of exactly 0,0 with the CMAC default AND
# overwrites the requested altitude with 584 m, so this runner was silently
# booting ArduPlane into a 584 m atmosphere datum (EAS2TAS 1.033). The origin
# is now set exactly via SIM_OPOS_LAT/LNG/ALT/HDG in
# config/ardupilot/falcon_v2_sitl.parm, through SIM_Aircraft.cpp:694-707
# update_home() - a path with no CMAC substitution. Do NOT re-add -O.
# See docs/source_of_truth/autopilot/SITL_ATMOSPHERE_AND_AIRSPEED.md.
#
# CONSEQUENCE FOR REPRODUCTION - READ BEFORE COMPARING TO OLD RESULTS: this
# runner is FIXED, not retired, because the results it produced are still cited
# elsewhere. But it now runs at a DIFFERENT (correct) atmosphere datum than the
# archived result files in tests/gazebo/results/ that were produced with the
# 584 m datum. A re-run is expected to differ from those archived numbers in
# every TAS-derived quantity by roughly the EAS2TAS ratio change
# (1.0334 -> ~1.0046, i.e. ~-2.8 %). That is a CORRECTION, not a regression.
# Archived pre-fix results must not be diffed against post-fix re-runs without
# accounting for it.
( cd "$SCRATCH" && gdb -q -x gdbcmds.txt --args \
    "$ARDUPLANE_BIN" -w -M json \
    --defaults "$SITL_PARM" -I 0 --speedup 1 > "$SCRATCH/arduplane.log" 2>&1 ) &
AP_PID=$!
sleep 8

echo "=== C.3 $RUN : running test ==="
cd "$REPO_ROOT"
python3 tests/gazebo/scripts/test_ardupilot_longitudinal_equilibrium_fbwa_crosscheck.py "$RUN"
RC=$?

echo "=== C.3 $RUN : done (rc=$RC), logs in $SCRATCH ==="
exit $RC

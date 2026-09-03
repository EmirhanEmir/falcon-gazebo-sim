#!/usr/bin/env bash
#
# FALCON V2 - ARDUPLANE_TECS_AND_CRUISE_SPEED_HOLD_VALIDATION - launch helper
# (controls-integration, 2026-09-02).
#
# Launches ONE fresh `gz sim` + gdb-wrapped `arduplane` pair (the SAME proven
# sequence as run_ardupilot_fbwa_level_pitch_reference_correction.sh - see
# docs/test_results/2026-08-28_ardupilot_trim_reference_correction_validation.md
# sec 3 for why arduplane is launched under gdb), runs the FBWB/TECS cruise
# speed + altitude hold campaign, then tears both processes down.
#
# Creates/modifies NO aircraft physics parameter, NO SDF, NO plugin, NO .parm
# file. In particular it sets NO TECS_* parameter: this stage is a BASELINE run
# on ArduPlane's own compiled firmware defaults. `arduplane -w` wipes its own
# scratch EEPROM; the checked-in config/ardupilot/falcon_v2_sitl.parm is
# READ-ONLY input and is NOT edited by this stage.
#
# Expected wall-clock: ~4-5 min (preconditions + <=150 s of flight + teardown).
# Outputs:
#   tests/gazebo/results/ardupilot_tecs_cruise_speed_hold_result.json      (summary + PASS/FAIL)
#   tests/gazebo/results/ardupilot_tecs_cruise_speed_hold_timeseries.json  (full 20 Hz raw record, ~8 MB)
#   tests/gazebo/results/ardupilot_tecs_cruise_speed_hold_log.txt
#   tests/gazebo/results/ardupilot_tecs_cruise_speed_hold_gz_log.txt
#   tests/gazebo/results/ardupilot_tecs_cruise_speed_hold_arduplane_log.txt
#   tests/gazebo/results/ardupilot_tecs_cruise_speed_hold_dataflash/*.BIN
#       (ArduPlane's own logs - contain the TECS log message with the internal
#        TECS state, which is NOT exposed over MAVLink. Copied out for
#        validation; not parsed by the test itself.)
#
# Usage:
#   ./tests/gazebo/scripts/run_ardupilot_tecs_cruise_speed_hold.sh
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf"
TEST_PY="$REPO_ROOT/tests/gazebo/scripts/test_ardupilot_tecs_cruise_speed_hold.py"
RESULTS="$REPO_ROOT/tests/gazebo/results"
LOG_OUT="$RESULTS/ardupilot_tecs_cruise_speed_hold_log.txt"
SCRATCH="$(mktemp -d /tmp/falcon_tecs_cruise_XXXXXX)"

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

# WIND (UPDATED 2026-09-02, stage SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_
# VALIDATION - the paragraph previously here is now OBSOLETE):
#   * The test world contains no world-level wind system, but model/model.sdf
#     carries the FalconV2Wind plugin, so /model/falcon_v2/wind exists in EVERY
#     world and defaults to the zero vector (<steady_wind_mps>0 0 0</...>).
#     This run is therefore still a ZERO-WIND run - by default, not by absence.
#     Wind can be commanded live on /model/falcon_v2/wind/steady_cmd; no world
#     change is needed for a wind campaign.
#   * ArduPlane is no longer wind-blind. model/model.sdf now wires
#     <airspeed_topic>/<wind_topic> into ArduPilotPlugin, so the FDM packet
#     carries the official SIM_JSON "airspeed" and "velocity_wind" keys, the
#     AIRSPEED bit is set, and the SIM_JSON.cpp:445 `wind_ef.zero()` branch is
#     NOT taken. See docs/source_of_truth/autopilot/
#     SITL_ATMOSPHERE_AND_AIRSPEED.md.
#   * SITL's own SIM_WIND_* still cannot affect this run (the JSON backend
#     never calls Aircraft::update_wind()); wind comes from the Gazebo side
#     only. The test still reads SIM_WIND_SPD and records it.
echo "=== TECS cruise hold : launching fresh gz sim + arduplane pair (scratch $SCRATCH) ==="
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

# NOTE (controls-integration, 2026-09-02, stage
# SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION): `-O 0,0,0,0` was REMOVED
# here. It never worked: SITL_cmdline.cpp:761-766 (parse_home) silently
# replaces a lat/lng of exactly 0,0 with the CMAC default AND overwrites the
# requested altitude with 584 m, which propagated into ArduPlane's atmosphere
# model (EAS2TAS 1.033). The origin is now set exactly, via SIM_OPOS_LAT/LNG/
# ALT/HDG in config/ardupilot/falcon_v2_sitl.parm, through
# SIM_Aircraft.cpp:694-707 update_home() - a path with no CMAC substitution.
# Do NOT re-add -O. See docs/source_of_truth/autopilot/
# SITL_ATMOSPHERE_AND_AIRSPEED.md.
( cd "$SCRATCH" && gdb -q -x gdbcmds.txt --args \
    "$ARDUPLANE_BIN" -w -M json \
    --defaults "$SITL_PARM" -I 0 --speedup 1 > "$SCRATCH/arduplane.log" 2>&1 ) &
AP_PID=$!
sleep 8

echo "=== TECS cruise hold : running test ==="
cd "$REPO_ROOT"
python3 "$TEST_PY" 2>&1 | tee "$LOG_OUT"
RC=${PIPESTATUS[0]}

echo "=== TECS cruise hold : done (rc=$RC), logs in $SCRATCH ==="
mkdir -p "$RESULTS"
cp "$SCRATCH/gz.log"        "$RESULTS/ardupilot_tecs_cruise_speed_hold_gz_log.txt" 2>/dev/null || true
cp "$SCRATCH/arduplane.log" "$RESULTS/ardupilot_tecs_cruise_speed_hold_arduplane_log.txt" 2>/dev/null || true
if [[ -d "$SCRATCH/logs" ]]; then
  mkdir -p "$RESULTS/ardupilot_tecs_cruise_speed_hold_dataflash"
  cp "$SCRATCH"/logs/*.BIN "$RESULTS/ardupilot_tecs_cruise_speed_hold_dataflash/" 2>/dev/null || true
fi
exit $RC

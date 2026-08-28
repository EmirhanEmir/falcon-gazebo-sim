#!/usr/bin/env bash
#
# FALCON V2 - ArduPlane SITL + Gazebo launch helper.
#
# Author: controls-integration, task
# ARDUPLANE_SITL_TRANSPORT_AND_ACTUATOR_MAPPING_VALIDATION (2026-08-27).
#
# Starts Gazebo Sim Harmonic (with the official ArduPilotPlugin block now
# present in model/model.sdf - see docs/source_of_truth/autopilot/
# SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md for the full design record) and
# ArduPlane SITL (JSON backend) against each other, on this machine's
# already-built binaries. Creates/modifies NO aircraft physics parameter,
# NO SDF, NO .parm file - it only sets environment variables and launches
# existing binaries.
#
# This is controls-integration's OWN minimal launch artifact (to confirm
# the transport connects at all - see the design doc sec 6). gazebo-testing
# owns the actual rigorous test scenarios/harness built on top of this same
# launch pattern - coordinate before extending this script into a full test
# runner; it is deliberately a thin, reusable "get both processes up and
# talking" helper, not a test itself.
#
# Usage:
#   cd ~/Desktop/FalconV2
#   ./tests/gazebo/scripts/launch_ardupilot_sitl.sh [--gui] [world.sdf]
#
#   --gui        Launch Gazebo with the GUI (default: headless, matching
#                this project's other automated-test convention).
#   world.sdf    Optional world file override (default:
#                tests/gazebo/worlds/falcon_v2_sensors_selftest_world.sdf -
#                the same world the prior SENSOR_MODEL_AND_ARDUPLANE_SITL_
#                PREPARATION stage already uses, since it already loads the
#                native gz-sim IMU/NavSat/AirPressure system plugins this
#                model's <sensor> elements require).
#
# Result: two long-running foreground-attached background processes
# (Gazebo server + arduplane). Press Ctrl+C to stop both cleanly. No flight
# mode is armed/changed by this script - it only starts the two processes;
# whatever connects to the resulting mavlink port (default tcp:127.0.0.1:5760,
# ArduPlane's own compiled-default SERIAL0) is responsible for anything
# beyond that (arming, RC input, mode changes), per this project's own
# testing/validation separation - this script does not do that itself.
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPILOT_ROOT="/home/emirhan/gazebo_sim/ardupilot"
ARDUPLANE_BIN="$ARDUPILOT_ROOT/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"

GUI_FLAG=""
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_sensors_selftest_world.sdf"

for arg in "$@"; do
  case "$arg" in
    --gui)
      GUI_FLAG="1"
      ;;
    *.sdf)
      WORLD="$arg"
      ;;
    *)
      echo "WARNING: ignoring unrecognized argument '$arg'" >&2
      ;;
  esac
done

# ---- Sanity checks (fail loud, not silently) ----
for f in \
  "$REPO_ROOT/plugins/aerodynamics/build/libFalconV2Aerodynamics.so" \
  "$REPO_ROOT/plugins/propulsion/build/libFalconV2Propulsion.so" \
  "$REPO_ROOT/plugins/actuators/build/libFalconV2Actuators.so" \
  "$REPO_ROOT/plugins/wind/build/libFalconV2Wind.so" \
  "$REPO_ROOT/plugins/sensors/build/libFalconV2Pitot.so" \
  "$REPO_ROOT/plugins/sensors/build/libFalconV2Magnetometer.so"
do
  if [[ ! -f "$f" ]]; then
    echo "ERROR: $f not found - build the project plugins first." >&2
    exit 1
  fi
done
if [[ ! -f "$ARDUPILOT_GAZEBO_BUILD/libArduPilotPlugin.so" ]]; then
  echo "ERROR: $ARDUPILOT_GAZEBO_BUILD/libArduPilotPlugin.so not found." >&2
  exit 1
fi
if [[ ! -x "$ARDUPLANE_BIN" ]]; then
  echo "ERROR: $ARDUPLANE_BIN not found/executable." >&2
  exit 1
fi
if [[ ! -f "$SITL_PARM" ]]; then
  echo "ERROR: $SITL_PARM not found." >&2
  exit 1
fi
if [[ ! -f "$WORLD" ]]; then
  echo "ERROR: world file not found: $WORLD" >&2
  exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/aerodynamics/build:$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/actuators/build:$REPO_ROOT/plugins/wind/build:$REPO_ROOT/plugins/sensors/build:$ARDUPILOT_GAZEBO_BUILD${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

echo "======================================================================"
echo "FALCON V2 - ArduPlane SITL + Gazebo launch"
echo "======================================================================"
echo "World:                  $WORLD"
echo "ArduPlane binary:       $ARDUPLANE_BIN"
echo "SITL param file:        $SITL_PARM"
echo "GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo
echo "ArduPlane FDM JSON target: 127.0.0.1:9002 (matches this project's"
echo "  ArduPilotPlugin <fdm_addr>/<fdm_port_in> AND ArduPlane's own JSON"
echo "  backend compiled default - no address/port args needed, same host)."
echo "MAVLink: ArduPlane's default SERIAL0, TCP port 5760"
echo "  (connect e.g. 'mavproxy.py --master=tcp:127.0.0.1:5760' or"
echo "  pymavlink 'tcp:127.0.0.1:5760')."
echo
echo "This script does NOT arm, change flight mode, or send any RC/servo"
echo "command - it only starts Gazebo and ArduPlane SITL and connects them."
echo
echo "Press Ctrl+C to stop both processes."
echo "======================================================================"
echo

GZ_PID=""
AP_PID=""
CLEANED_UP=0

cleanup() {
  if [[ "$CLEANED_UP" -eq 1 ]]; then
    return
  fi
  CLEANED_UP=1
  trap - INT TERM EXIT
  echo
  echo "Shutting down..."

  if [[ -n "$AP_PID" ]] && kill -0 "$AP_PID" 2>/dev/null; then
    kill -TERM "$AP_PID" 2>/dev/null
    wait "$AP_PID" 2>/dev/null
  fi

  if [[ -n "$GZ_PID" ]]; then
    pkill -TERM -P "$GZ_PID" 2>/dev/null
    kill -TERM "$GZ_PID" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$GZ_PID" 2>/dev/null || break
      sleep 0.3
    done
    kill -0 "$GZ_PID" 2>/dev/null && kill -KILL "$GZ_PID" 2>/dev/null
    wait "$GZ_PID" 2>/dev/null
  fi

  echo "Done."
}
trap cleanup INT TERM EXIT

if [[ -n "$GUI_FLAG" ]]; then
  gz sim -r "$WORLD" &
else
  gz sim -s -r --headless-rendering "$WORLD" &
fi
GZ_PID=$!

# Give Gazebo a moment to load the model/plugins and start listening on
# the FDM UDP port before ArduPlane tries to connect.
sleep 5

# -w wipes any stale eeprom.bin/parameter state from a prior run in this
# working directory (this script's own working directory only - never the
# checked-in falcon_v2_sitl.parm, which is read-only input here).
# -O 0,0,0,0 gives a fixed, arbitrary local home (no real georeferencing
# exists for Falcon V2 yet - DATA_REQUIRED, unrelated to this task).
"$ARDUPLANE_BIN" \
  -w -M json -O 0,0,0,0 \
  --defaults "$SITL_PARM" \
  -I 0 --speedup 1 &
AP_PID=$!

wait "$GZ_PID"

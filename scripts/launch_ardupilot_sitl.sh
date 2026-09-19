#!/usr/bin/env bash
#
# FALCON V2 - ArduPlane SITL + Gazebo Sim Harmonic launcher.
#
# Starts Gazebo (loading the FALCON V2 model and its five system plugins)
# and ArduPlane SITL with the JSON FDM backend, connected to each other.
# It only sets environment variables and launches existing binaries: no
# aircraft parameter, SDF or .parm file is modified.
#
# Usage:
#   ./scripts/launch_ardupilot_sitl.sh [--gui] [world.sdf]
#
#   --gui        Launch Gazebo with the GUI (default: headless).
#   world.sdf    World file override (default: worlds/falcon_v2_runway.sdf).
#
# External paths may be overridden via the environment:
#   ARDUPILOT_ROOT           ArduPilot source tree (built for SITL)
#   ARDUPILOT_GAZEBO_BUILD   ardupilot_gazebo build directory
#
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/gazebo_sim/ardupilot}"
ARDUPLANE_BIN="$ARDUPILOT_ROOT/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="${ARDUPILOT_GAZEBO_BUILD:-$HOME/gazebo_sim/ardupilot_gazebo/build}"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"

GUI_FLAG=""
WORLD="$REPO_ROOT/worlds/falcon_v2_runway.sdf"

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
    echo "ERROR: $f not found - build the project plugins first" \
         "(./scripts/build_plugins.sh)." >&2
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
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT:$REPO_ROOT/model${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# The <config_path> entries in model/model.sdf are repository-relative, so
# Gazebo must run with the repository root as its working directory.
cd "$REPO_ROOT" || exit 1

echo "======================================================================"
echo "FALCON V2 - ArduPlane SITL + Gazebo"
echo "======================================================================"
echo "World:                  $WORLD"
echo "ArduPlane binary:       $ARDUPLANE_BIN"
echo "SITL param file:        $SITL_PARM"
echo
echo "ArduPlane FDM JSON target: 127.0.0.1:9002 (matches the ArduPilotPlugin"
echo "  <fdm_addr>/<fdm_port_in> in model/model.sdf and ArduPlane's own JSON"
echo "  backend default - no address/port arguments needed on the same host)."
echo "MAVLink: ArduPlane's default SERIAL0, TCP port 5760"
echo "  (connect e.g. 'mavproxy.py --master=tcp:127.0.0.1:5760')."
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

# -w wipes stale eeprom.bin/parameter state from a prior run in this working
# directory only; the checked-in falcon_v2_sitl.parm is read-only input.
#
# Do NOT add `-O 0,0,0,0`: ArduPilot's parse_home() silently replaces a
# lat/lng of exactly 0,0 with the CMAC default and overrides the requested
# altitude with 584 m, which corrupts the atmosphere model (EAS2TAS 1.033).
# The simulation origin is set exactly via SIM_OPOS_LAT/LNG/ALT/HDG in
# config/ardupilot/falcon_v2_sitl.parm instead.
"$ARDUPLANE_BIN" \
  -w -M json \
  --defaults "$SITL_PARM" \
  -I 0 --speedup 1 &
AP_PID=$!

wait "$GZ_PID"

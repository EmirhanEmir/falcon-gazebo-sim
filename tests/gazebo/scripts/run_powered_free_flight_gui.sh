#!/usr/bin/env bash
#
# FALCON V2 - one-command GUI reproduction of the already-verified
# POWERED_FREE_FLIGHT_SMOKE_TEST (gazebo-testing, 2026-08-23).
#
# Launches Gazebo Sim Harmonic WITH the GUI (never headless), loads
# tests/gazebo/worlds/falcon_v2_powered_free_flight_gui_world.sdf (spawns
# the unmodified model/model.sdf at 85m altitude with its exact initial
# body velocity already baked in via that world file's <state> block - see
# its header comment for why no runtime teleport/convergence step is
# needed here, unlike the original headless TestFixture smoke test), then
# starts tests/gazebo/scripts/powered_free_flight_gui_setup.py, which
# continuously commands throttle=0.50/0.50 and the 5 control-surface holds
# (elevators +8deg, ailerons/rudder neutral) for as long as this script
# runs.
#
# This script creates/modifies NO aircraft physics parameter. It only sets
# environment variables, launches existing binaries, and runs the two new
# files above.
#
# Usage:
#   cd ~/Desktop/FalconV2
#   ./tests/gazebo/scripts/run_powered_free_flight_gui.sh
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_powered_free_flight_gui_world.sdf"
SETUP_PY="$REPO_ROOT/tests/gazebo/scripts/powered_free_flight_gui_setup.py"
AERO_PLUGIN="$REPO_ROOT/plugins/aerodynamics/build/libFalconV2Aerodynamics.so"
PROP_PLUGIN="$REPO_ROOT/plugins/propulsion/build/libFalconV2Propulsion.so"

if [[ ! -f "$AERO_PLUGIN" ]]; then
  echo "ERROR: $AERO_PLUGIN not found." >&2
  echo "Build it first: see plugins/aerodynamics/README.md ('Build' section)." >&2
  exit 1
fi
if [[ ! -f "$PROP_PLUGIN" ]]; then
  echo "ERROR: $PROP_PLUGIN not found." >&2
  echo "Build it first: see plugins/propulsion/README.md ('Build' section)." >&2
  exit 1
fi
if [[ ! -f "$WORLD" ]]; then
  echo "ERROR: world file not found: $WORLD" >&2
  exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/aerodynamics/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT:$REPO_ROOT/model${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

echo "======================================================================"
echo "FALCON V2 - POWERED_FREE_FLIGHT_SMOKE_TEST GUI reproduction"
echo "======================================================================"
echo "World: $WORLD"
echo "GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo
echo "Camera note: the world file sets a fixed starting camera view near the"
echo "spawn point. If the aircraft flies out of frame, select 'falcon_v2' in"
echo "the GUI's entity tree (or click it in the 3D view) and use the"
echo "right-click 'Follow' option (Camera Tracking plugin) - or the F key in"
echo "gz-sim versions that bind it - to keep it in view."
echo
echo "Close the Gazebo window, or press Ctrl+C here, to stop."
echo "======================================================================"
echo

GZ_PID=""
HELPER_PID=""
CLEANED_UP=0

cleanup() {
  if [[ "$CLEANED_UP" -eq 1 ]]; then
    return
  fi
  CLEANED_UP=1
  trap - INT TERM EXIT
  echo
  echo "Shutting down Falcon V2 GUI simulation..."

  if [[ -n "$HELPER_PID" ]] && kill -0 "$HELPER_PID" 2>/dev/null; then
    kill -TERM "$HELPER_PID" 2>/dev/null
    wait "$HELPER_PID" 2>/dev/null
  fi

  if [[ -n "$GZ_PID" ]]; then
    # gz sim's top-level process forks separate "gz sim server" / "gz sim
    # gui" children - kill those explicitly too, then the parent, so no
    # child is orphaned regardless of how gz sim forwards signals.
    pkill -TERM -P "$GZ_PID" 2>/dev/null
    kill -TERM "$GZ_PID" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$GZ_PID" 2>/dev/null || break
      sleep 0.3
    done
    kill -0 "$GZ_PID" 2>/dev/null && kill -KILL "$GZ_PID" 2>/dev/null
    wait "$GZ_PID" 2>/dev/null
  fi

  # Safety net: catch any leftover process tied to this exact world file
  # (e.g. if the parent above already exited and reparented its children).
  pkill -f "gz sim -r $WORLD" 2>/dev/null

  echo "Done."
}
trap cleanup INT TERM EXIT

gz sim -r "$WORLD" &
GZ_PID=$!

# Give the server a moment to come up and advertise its topics/services
# before the control-surface/throttle commander starts publishing (same
# discovery-timing consideration documented in propulsion_lib.py -
# publishing continuously, not once, is what actually makes this robust;
# this sleep just avoids a few seconds of visibly-neutral control surfaces
# at the very start).
sleep 3

python3 "$SETUP_PY" &
HELPER_PID=$!

# Wait on the GUI/server process - if the user closes the Gazebo window,
# this returns and the EXIT trap above cleans up the helper too. If the
# user presses Ctrl+C, the INT trap runs cleanup directly.
wait "$GZ_PID"

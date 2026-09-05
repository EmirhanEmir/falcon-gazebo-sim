#!/usr/bin/env bash
#
# FALCON V2 - ARDUPLANE_TECS_CLIMB_DESCENT_AND_ENERGY_VALIDATION - launch helper
# (controls-integration, 2026-09-03).
#
# Launches ONE fresh `gz sim` + gdb-wrapped `arduplane` pair (the SAME proven
# sequence as run_ardupilot_tecs_cruise_speed_hold.sh - see
# docs/test_results/2026-08-28_ardupilot_trim_reference_correction_validation.md
# sec 3 for why arduplane is launched under gdb), runs the five-phase
# FBWB/TECS climb-descent-and-energy campaign, then tears both processes down.
#
# CAMPAIGN (one run, five sequential phases, ~165 s of flight max):
#   P1 CRUISE    45 s  level cruise ~18 m/s + altitude hold
#   P2 CLIMB    <=20 s +10 m via the FBWB pitch-stick ramp
#   P3 SETTLE    40 s  level off at the new altitude, settle
#   P4 DESCENT  <=20 s -10 m, targeting the ORIGINAL P1 altitude
#   P5 RESETTLE  40 s  hold near the original altitude, settle
#
# CREATES/MODIFIES NO aircraft physics parameter, NO SDF, NO plugin, NO .parm
# file. In particular it sets NO TECS_* parameter and NO PID: this stage runs
# on ArduPlane's own compiled firmware defaults. `arduplane -w` wipes its own
# scratch EEPROM; the checked-in config/ardupilot/falcon_v2_sitl.parm is
# READ-ONLY input and is NOT edited by this stage.
#
# AMENDED 2026-09-05 (stage ARDUPLANE_TECS_PTCH_DAMP_ADOPTION_INTEGRATION):
# this script still WRITES no TECS_* parameter, but the .parm it loads is no
# longer TECS-free. config/ardupilot/falcon_v2_sitl.parm now SETS exactly one
# TECS value, TECS_PTCH_DAMP 0.6 (section
# FALCON_V2_SIM_VALIDATED_TECS_PITCH_DAMPING, superseding the AP_TECS.cpp:107
# default 0.3). Every OTHER TECS_* value is still an ArduPlane compiled
# firmware default. A run of this script is therefore now on the PROJECT
# BASELINE - "firmware defaults EXCEPT TECS_PTCH_DAMP 0.6" - not on pure
# compiled firmware defaults. The wording above was true when this stage was
# flown and is SUPERSEDED for any NEW run. NOTE: this script's test module
# still gates on `tecs_at_firmware_defaults`; re-running it unchanged is
# expected to trip that gate. Flagged to gazebo-testing, not silently
# "fixed" here - the gate is deliberately left alone.
#
# Expected wall-clock: ~5-6 min (preconditions + <=165 s of flight + teardown).
# Outputs:
#   tests/gazebo/results/ardupilot_tecs_climb_descent_energy_result.json      (summary + PASS/FAIL)
#   tests/gazebo/results/ardupilot_tecs_climb_descent_energy_timeseries.json  (full 20 Hz raw record, ~10 MB)
#   tests/gazebo/results/ardupilot_tecs_climb_descent_energy_per_sample.json  (the stage's per-sample
#       energy/telemetry trace: target+actual airspeed, target+actual altitude, vertical speed,
#       physical pitch AND raw nav_pitch AND the PTCH_TRIM_DEG-corrected demand, throttle,
#       elevator deg, L/R motor RPM + thrust, advance ratio J, SPE/SKE/STE/SEB, saturation flags)
#   tests/gazebo/results/ardupilot_tecs_climb_descent_energy_log.txt
#   tests/gazebo/results/ardupilot_tecs_climb_descent_energy_gz_log.txt
#   tests/gazebo/results/ardupilot_tecs_climb_descent_energy_arduplane_log.txt
#   tests/gazebo/results/ardupilot_tecs_climb_descent_energy_dataflash/*.BIN
#       (ArduPlane's own logs - contain the TECS log message with the INTERNAL
#        TECS state, which is NOT exposed over MAVLink. Copied out so
#        `validation` can cross-check the energy analysis against TECS's own
#        SPE/SKE/SPEdot/SKEdot/th/pmax/pmin. Not parsed by the test itself.)
#
# Usage:
#   ./tests/gazebo/scripts/run_ardupilot_tecs_climb_descent_energy.sh
#
# Offline re-analysis after a TEST-LOGIC (never physics) fix:
#   python3 tests/gazebo/scripts/test_ardupilot_tecs_climb_descent_energy.py \
#       --reanalyze tests/gazebo/results/ardupilot_tecs_climb_descent_energy_timeseries.json
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf"
TEST_PY="$REPO_ROOT/tests/gazebo/scripts/test_ardupilot_tecs_climb_descent_energy.py"
RESULTS="$REPO_ROOT/tests/gazebo/results"
PREFIX="ardupilot_tecs_climb_descent_energy"
LOG_OUT="$RESULTS/${PREFIX}_log.txt"
SCRATCH="$(mktemp -d /tmp/falcon_tecs_climb_descent_XXXXXX)"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/aerodynamics/build:$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/actuators/build:$REPO_ROOT/plugins/wind/build:$REPO_ROOT/plugins/sensors/build:$ARDUPILOT_GAZEBO_BUILD"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

for f in "$ARDUPLANE_BIN" "$SITL_PARM" "$WORLD" "$TEST_PY"; do
  [[ -e "$f" ]] || { echo "ERROR: missing $f" >&2; exit 2; }
done

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

# WIND - ZERO WIND IS THE ACCEPTANCE CONDITION FOR THIS STAGE.
#   * The test world contains no world-level wind system, but model/model.sdf
#     carries the FalconV2Wind plugin, so /model/falcon_v2/wind exists in EVERY
#     world and defaults to the zero vector (<steady_wind_mps>0 0 0</...>).
#     This run is therefore a ZERO-WIND run BY DEFAULT, not by ArduPlane being
#     wind-blind.
#   * The airspeed path is the OFFICIAL SIM_JSON one: model/model.sdf wires
#     <airspeed_topic> (FalconV2Pitot, EAS) and <wind_topic> (FalconV2Wind,
#     world-ENU airmass velocity) into ArduPilotPlugin, so the FDM packet
#     carries the SIM_JSON "airspeed" and "velocity_wind" keys,
#     DataKey::AIRSPEED is set, the SIM_JSON.cpp:445 wind_ef.zero() branch is
#     NOT taken, and ARSPD_TYPE=100 / ARSPD_USE=1 feed that EAS to TECS. NO
#     physics bypass. See docs/source_of_truth/autopilot/
#     SITL_ATMOSPHERE_AND_AIRSPEED.md.
#   * SITL's own SIM_WIND_* cannot affect this run (the JSON backend never
#     calls Aircraft::update_wind()); wind comes from the Gazebo side only. The
#     test still reads SIM_WIND_SPD/DIR/TURB and records + gates them.
echo "=== TECS climb/descent energy : launching fresh gz sim + arduplane pair (scratch $SCRATCH) ==="
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
# SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION): `-O 0,0,0,0` must NOT be
# used here. SITL_cmdline.cpp:761-766 (parse_home) silently replaces a lat/lng
# of exactly 0,0 with the CMAC default AND overwrites the requested altitude
# with 584 m, which propagates into ArduPlane's atmosphere model (EAS2TAS
# 1.033) and therefore into every TAS-derived TECS quantity - including the
# energies this stage measures. The origin is set exactly instead, via
# SIM_OPOS_LAT/LNG/ALT/HDG in config/ardupilot/falcon_v2_sitl.parm, through
# SIM_Aircraft.cpp:694-707 update_home() - a path with no CMAC substitution.
# The test GATES SIM_OPOS_ALT == 0 so this can never regress silently.
# See docs/source_of_truth/autopilot/SITL_ATMOSPHERE_AND_AIRSPEED.md.
( cd "$SCRATCH" && gdb -q -x gdbcmds.txt --args \
    "$ARDUPLANE_BIN" -w -M json \
    --defaults "$SITL_PARM" -I 0 --speedup 1 > "$SCRATCH/arduplane.log" 2>&1 ) &
AP_PID=$!
sleep 8

echo "=== TECS climb/descent energy : running test ==="
cd "$REPO_ROOT"
python3 "$TEST_PY" 2>&1 | tee "$LOG_OUT"
RC=${PIPESTATUS[0]}

echo "=== TECS climb/descent energy : done (rc=$RC), logs in $SCRATCH ==="
mkdir -p "$RESULTS"
cp "$SCRATCH/gz.log"        "$RESULTS/${PREFIX}_gz_log.txt" 2>/dev/null || true
cp "$SCRATCH/arduplane.log" "$RESULTS/${PREFIX}_arduplane_log.txt" 2>/dev/null || true
if [[ -d "$SCRATCH/logs" ]]; then
  mkdir -p "$RESULTS/${PREFIX}_dataflash"
  cp "$SCRATCH"/logs/*.BIN "$RESULTS/${PREFIX}_dataflash/" 2>/dev/null || true
fi
exit $RC

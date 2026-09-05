#!/usr/bin/env bash
#
# FALCON V2 - ARDUPLANE_LONGITUDINAL_PHUGOID_DAMPING_VALIDATION (part 1)
# launch helper - controls-integration, 2026-09-04.
#
# Launches ONE fresh `gz sim` + gdb-wrapped `arduplane` pair using EXACTLY the
# sequence proven by run_ardupilot_tecs_climb_descent_energy.sh (same world,
# same .parm, same env, same cleanup/trap, same SIM_OPOS origin gating - the
# `-O 0,0,0,0` / CMAC 584 m trap stays avoided and stays gated), runs the
# single-transient longitudinal free-decay campaign, then tears both processes
# down. See docs/test_results/2026-08-28_ardupilot_trim_reference_correction_
# validation.md sec 3 for why arduplane is launched under gdb.
#
# CAMPAIGN (one run, three sequential phases, 99 s of flight):
#   P1 TRIM       30 s  level cruise ~18 m/s, neutral pitch stick
#   P2 EXCITE      4 s  ONE full-up FBWB pitch-stick pulse (no cadence)
#   P3 RINGDOWN   65 s  stick released to RC2_TRIM; free decay with the height
#                       and airspeed demands LOCKED
#                       (ArduPlane/navigation.cpp:418-424
#                        set_target_altitude_current())
#
# PARAMETER POLICY
#   By DEFAULT this run writes NO runtime parameter of any kind: no TECS_*, no
#   PID, no PTCH_TRIM_DEG, no control-surface mapping, no aero/propulsion/
#   actuator/sensor/mass/CG/inertia value. `arduplane -w` wipes its own scratch
#   EEPROM. The checked-in config/ardupilot/falcon_v2_sitl.parm is READ-ONLY
#   input and is NOT edited by this script.
#
#   UPDATED 2026-09-05 (stage ARDUPLANE_TECS_PTCH_DAMP_ADOPTION_INTEGRATION):
#   falcon_v2_sitl.parm now SETS exactly one TECS value, TECS_PTCH_DAMP 0.6
#   (section FALCON_V2_SIM_VALIDATED_TECS_PITCH_DAMPING, superseding the
#   AP_TECS.cpp:107 default 0.3). Every OTHER TECS_* value is still the
#   compiled firmware default. A no-argument run is therefore the PROJECT
#   BASELINE - "firmware defaults EXCEPT TECS_PTCH_DAMP 0.6" - and is NOT the
#   firmware-defaults baseline. The previous wording here ("every TECS_* value
#   is the compiled firmware default") was true before that stage and is
#   SUPERSEDED. Still no runtime parameter is written by the harness.
#
#   THE STAGE BASELINE MUST BE RUN WITH NO ARGUMENTS.
#
#   Any argument given to this script is forwarded verbatim to the test. The
#   only argument the test accepts that changes the vehicle is
#   `--set-param NAME=VALUE`, which does a RUNTIME MAVLink PARAM_SET restricted
#   to the TECS energy-loop whitelist (SETTABLE_PARAMS in the test module) and
#   to each parameter's own ArduPilot @Range; every other name is REFUSED with
#   a non-zero exit. That path exists for part 2 of this stage, not for the
#   baseline.
#
# Expected wall-clock: ~4-5 min (preconditions + 99 s of flight + teardown).
#
# Outputs:
#   tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_result.json
#       summary + acceptance checks + the measured period / envelope tau /
#       decay ratio / damping ratio / free-airframe Lanchester reference /
#       closed-loop-over-free tau ratio (the 1.24x number) and period ratio
#   tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_timeseries.json
#       full 20 Hz raw record (large), enough to re-derive every quantity
#   tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_per_sample.json
#       per-sample trace: target+actual airspeed, target+actual altitude,
#       vertical speed, physical pitch, RAW nav_pitch AND the PTCH_TRIM_DEG-
#       corrected pitch demand, throttle, elevator deg, L/R motor RPM+thrust,
#       advance ratio J, SPE/SKE/STE/SEB, saturation flags, rc2/rc3
#   tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_log.txt
#   tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_gz_log.txt
#   tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_arduplane_log.txt
#   tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_dataflash/*.BIN
#       ArduPlane's own logs. These carry the TECS/TEC2/TEC3/TEC4 messages with
#       the INTERNAL TECS state (SPE/SKE/SPEdot/SKEdot/EBD/EBE/EBDD/EBDE/EBDDT/
#       I/KI/th/pmin/pmax), which is NOT exposed over MAVLink. Copied out so
#       `validation` can cross-check the ring-down analysis and the
#       source-derived tecs_energy_loop_gains diagnostic against TECS's own
#       numbers. Not parsed by the test itself.
#
# Usage (BASELINE - this is what gazebo-testing runs for part 1):
#   ./tests/gazebo/scripts/run_ardupilot_longitudinal_phugoid_damping.sh
#
# Offline re-analysis after a TEST-LOGIC (never physics) fix:
#   python3 tests/gazebo/scripts/test_ardupilot_longitudinal_phugoid_damping.py \
#       --reanalyze tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_timeseries.json
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf"
TEST_PY="$REPO_ROOT/tests/gazebo/scripts/test_ardupilot_longitudinal_phugoid_damping.py"
RESULTS="$REPO_ROOT/tests/gazebo/results"
PREFIX="ardupilot_longitudinal_phugoid_damping"
LOG_OUT="$RESULTS/${PREFIX}_log.txt"
SCRATCH="$(mktemp -d /tmp/falcon_phugoid_damping_XXXXXX)"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/aerodynamics/build:$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/actuators/build:$REPO_ROOT/plugins/wind/build:$REPO_ROOT/plugins/sensors/build:$ARDUPILOT_GAZEBO_BUILD"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

for f in "$ARDUPLANE_BIN" "$SITL_PARM" "$WORLD" "$TEST_PY"; do
  [[ -e "$f" ]] || { echo "ERROR: missing $f" >&2; exit 2; }
done

if [[ $# -gt 0 ]]; then
  echo "NOTE: extra arguments forwarded to the test: $*"
  echo "NOTE: the STAGE BASELINE must be run with NO arguments."
fi

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
#     wind-blind. A free-decay damping measurement is only meaningful in still
#     air, so the test also GATES it (param precondition sim_wind_zero).
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
echo "=== phugoid damping baseline : launching fresh gz sim + arduplane pair (scratch $SCRATCH) ==="
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
# energies and the Lanchester L/D reference this stage measures. The origin is
# set exactly instead, via SIM_OPOS_LAT/LNG/ALT/HDG in
# config/ardupilot/falcon_v2_sitl.parm, through SIM_Aircraft.cpp:694-707
# update_home() - a path with no CMAC substitution. The test GATES
# SIM_OPOS_ALT == 0 so this can never regress silently.
# See docs/source_of_truth/autopilot/SITL_ATMOSPHERE_AND_AIRSPEED.md.
( cd "$SCRATCH" && gdb -q -x gdbcmds.txt --args \
    "$ARDUPLANE_BIN" -w -M json \
    --defaults "$SITL_PARM" -I 0 --speedup 1 > "$SCRATCH/arduplane.log" 2>&1 ) &
AP_PID=$!
sleep 8

echo "=== phugoid damping baseline : running test ==="
cd "$REPO_ROOT"
python3 "$TEST_PY" "$@" 2>&1 | tee "$LOG_OUT"
RC=${PIPESTATUS[0]}

echo "=== phugoid damping baseline : done (rc=$RC), logs in $SCRATCH ==="
mkdir -p "$RESULTS"
cp "$SCRATCH/gz.log"        "$RESULTS/${PREFIX}_gz_log.txt" 2>/dev/null || true
cp "$SCRATCH/arduplane.log" "$RESULTS/${PREFIX}_arduplane_log.txt" 2>/dev/null || true
if [[ -d "$SCRATCH/logs" ]]; then
  mkdir -p "$RESULTS/${PREFIX}_dataflash"
  cp "$SCRATCH"/logs/*.BIN "$RESULTS/${PREFIX}_dataflash/" 2>/dev/null || true
fi
exit $RC

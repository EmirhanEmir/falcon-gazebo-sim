#!/usr/bin/env bash
#
# FALCON V2 - ARDUPLANE_LONGITUDINAL_PHUGOID_DAMPING_VALIDATION (part 3a)
# SHORT TECS_PTCH_DAMP PERFORMANCE-REGRESSION launch helper
# controls-integration, 2026-09-04.
#
# Launches ONE fresh `gz sim` + gdb-wrapped `arduplane` pair using EXACTLY the
# sequence proven by run_ardupilot_tecs_climb_descent_energy.sh and reused by
# run_ardupilot_longitudinal_phugoid_damping.sh (same world, same READ-ONLY
# .parm, same env, same cleanup/trap, same SIM_OPOS origin handling - the
# `-O 0,0,0,0` / CMAC 584 m trap stays avoided AND stays gated), runs the short
# regression campaign, then tears both processes down. See
# docs/test_results/2026-08-28_ardupilot_trim_reference_correction_validation.md
# sec 3 for why arduplane is launched under gdb.
#
# CAMPAIGN (one run, five sequential phases; 134 s worst case, ~121 s expected,
# against the 165 s campaign it replaces):
#   R1_cruise    36 s        level cruise ~18 m/s + altitude hold
#                            (12 s inherited FBWB-entry transient + 24 s analysed)
#   R2_climb     cap 15 s    ONE +10 m climb, closed-loop stop at z >= z_ref+10
#   R3_settle    34 s        settle at the new altitude (10 s + 24 s analysed)
#   R4_descent   cap 15 s    ONE -10 m descent, stop at z <= z_ref
#   R5_resettle  34 s        settle back at the original altitude
# Every duration is derived in the test module's docstring from the prior
# stage's MEASURED ramp behaviour (ramp_vz 1.301 m/s, peak 1.951 m/s, measured
# ramp 8.67 s) and from the closed-loop mode period measured in part 1 of this
# stage (5.6474 s at the firmware default). The 24 s analysed window is 4 full
# cycles of that mode - the shortest window that still resolves the growth test
# and the hold statistics.
#
# WHAT IT MEASURES
#   NOT-WORSE-THAN regression of cruise / altitude hold / climb / descent
#   against TWO recorded reference realisations of the same configuration:
#     REF_A  2026-09-02 cruise campaign (energy.PRIOR)
#     REF_B  2026-09-03/04 climb/descent/energy campaign (its own published
#            result JSON; reproduced exactly by this harness's own analysis)
#   It never gates on BEATING a reference.
#
# PARAMETER POLICY
#   With NO --set-param this run writes NO runtime parameter of any kind:
#   `arduplane -w` wipes its own scratch EEPROM, so every TECS_* value the
#   vehicle flies with comes either from the ArduPlane compiled firmware
#   defaults or from the checked-in config/ardupilot/falcon_v2_sitl.parm.
#   UPDATED 2026-09-05 (stage ARDUPLANE_TECS_PTCH_DAMP_ADOPTION_INTEGRATION):
#   falcon_v2_sitl.parm now SETS exactly one TECS value, TECS_PTCH_DAMP 0.6
#   (section FALCON_V2_SIM_VALIDATED_TECS_PITCH_DAMPING, superseding the
#   AP_TECS.cpp:107 default 0.3). Every OTHER TECS_* parameter is still an
#   ArduPlane compiled firmware default. A no-flag run is therefore the
#   PROJECT BASELINE of this same harness - "firmware defaults EXCEPT
#   TECS_PTCH_DAMP 0.6" - and is NOT the firmware-defaults baseline. The
#   previous wording here ("the .parm sets no TECS_* value", "a no-flag run is
#   the DEFAULTS BASELINE") was true before that stage and is SUPERSEDED.
#   The harness itself still writes NO parameter on a no-flag run.
#   `--set-param NAME=VALUE` does a RUNTIME MAVLink PARAM_SET restricted to the
#   TECS energy-loop whitelist (SETTABLE_PARAMS in the test module) and to each
#   parameter's own ArduPilot @Range; every other name - any PID, PTCH_TRIM_DEG,
#   any SERVOn_*, ARSPD_*, SIM_*, aero/propulsion/actuator/sensor/mass value -
#   is REFUSED with a non-zero exit. This harness writes NO change of any kind
#   to any checked-in file; config/ardupilot/falcon_v2_sitl.parm is READ-ONLY
#   input to it (the 0.6 in that file was put there by the adoption stage, not
#   by this harness).
#
# ARGUMENTS - all forwarded verbatim to the test:
#   --set-param TECS_PTCH_DAMP=0.6    runtime candidate value (part 3a)
#   --tag NAME                        output-filename suffix; also used by THIS
#                                     script for the gz/arduplane/dataflash
#                                     copies, so a candidate run can never
#                                     overwrite a baseline run's artifacts.
#
# THE PART-3a CANDIDATE RUN:
#   ./tests/gazebo/scripts/run_ardupilot_tecs_ptch_damp_regression.sh \
#       --set-param TECS_PTCH_DAMP=0.6 --tag ptchdamp06
#
# OPTIONAL same-harness PROJECT-BASELINE run (only if requested). As of
# 2026-09-05 a no-flag run is the PROJECT baseline (TECS_PTCH_DAMP 0.6 from the
# .parm), NOT the firmware-defaults baseline; there is no flag that reproduces
# the firmware default 0.3 other than an explicit runtime
# `--set-param TECS_PTCH_DAMP=0.3`:
#   ./tests/gazebo/scripts/run_ardupilot_tecs_ptch_damp_regression.sh \
#       --tag baseline
#
# Expected wall-clock: ~4-5 min (preconditions + ~121 s of flight + teardown).
#
# Outputs (SFX = "_<tag>" when --tag is given, empty otherwise):
#   tests/gazebo/results/ardupilot_tecs_ptch_damp_regression_result${SFX}.json
#       summary + acceptance checks + every regression gate with its measured
#       value, BOTH reference values, the measured run-to-run spread, the limit
#       and the derivation string
#   tests/gazebo/results/ardupilot_tecs_ptch_damp_regression_timeseries${SFX}.json
#       full 20 Hz raw record (large), enough to re-derive every quantity
#   tests/gazebo/results/ardupilot_tecs_ptch_damp_regression_per_sample${SFX}.json
#       per-sample trace: target+actual airspeed, target+actual altitude,
#       vertical speed, physical pitch, RAW nav_pitch AND the PTCH_TRIM_DEG-
#       corrected pitch demand, throttle, elevator deg, L/R motor RPM+thrust,
#       advance ratio J, interp-clamp flags, SPE/SKE/STE/SEB, saturation flags,
#       alpha, rc2/rc3
#   tests/gazebo/results/ardupilot_tecs_ptch_damp_regression_log${SFX}.txt
#   tests/gazebo/results/ardupilot_tecs_ptch_damp_regression_gz_log${SFX}.txt
#   tests/gazebo/results/ardupilot_tecs_ptch_damp_regression_arduplane_log${SFX}.txt
#   tests/gazebo/results/ardupilot_tecs_ptch_damp_regression_dataflash${SFX}/*.BIN
#       ArduPlane's own logs, carrying the TECS/TEC2/TEC3/TEC4 messages with the
#       INTERNAL TECS state (SPE/SKE/SPEdot/SKEdot/EBD/EBE/EBDD/EBDE/EBDDT/I/KI/
#       th/pmin/pmax), which is NOT exposed over MAVLink. Copied out so
#       `validation` can cross-check the regression against TECS's own numbers.
#       Not parsed by the test itself.
#
# Offline re-analysis after a TEST-LOGIC (never physics) fix:
#   python3 tests/gazebo/scripts/test_ardupilot_tecs_ptch_damp_regression.py \
#       --reanalyze tests/gazebo/results/ardupilot_tecs_ptch_damp_regression_timeseries_ptchdamp06.json \
#       --tag ptchdamp06
#
set -uo pipefail

REPO_ROOT="/home/emirhan/Desktop/FalconV2"
ARDUPLANE_BIN="/home/emirhan/gazebo_sim/ardupilot/build/sitl/bin/arduplane"
ARDUPILOT_GAZEBO_BUILD="/home/emirhan/gazebo_sim/ardupilot_gazebo/build"
SITL_PARM="$REPO_ROOT/config/ardupilot/falcon_v2_sitl.parm"
WORLD="$REPO_ROOT/tests/gazebo/worlds/falcon_v2_ardupilot_basic_closed_loop_flight_world.sdf"
TEST_PY="$REPO_ROOT/tests/gazebo/scripts/test_ardupilot_tecs_ptch_damp_regression.py"
RESULTS="$REPO_ROOT/tests/gazebo/results"
PREFIX="ardupilot_tecs_ptch_damp_regression"

# --tag NAME -> filename suffix for THIS script's copies (the test applies the
# same suffix to its own artifacts). Arguments are forwarded unchanged.
TAG=""
for ((i=1; i<=$#; i++)); do
  if [[ "${!i}" == "--tag" ]]; then
    j=$((i+1))
    if (( j <= $# )); then TAG="${!j}"; fi
  fi
done
SFX=""
[[ -n "$TAG" ]] && SFX="_$TAG"

LOG_OUT="$RESULTS/${PREFIX}_log${SFX}.txt"
SCRATCH="$(mktemp -d /tmp/falcon_ptch_damp_regression_XXXXXX)"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/plugins/aerodynamics/build:$REPO_ROOT/plugins/propulsion/build:$REPO_ROOT/plugins/actuators/build:$REPO_ROOT/plugins/wind/build:$REPO_ROOT/plugins/sensors/build:$ARDUPILOT_GAZEBO_BUILD"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

for f in "$ARDUPLANE_BIN" "$SITL_PARM" "$WORLD" "$TEST_PY"; do
  [[ -e "$f" ]] || { echo "ERROR: missing $f" >&2; exit 2; }
done

if [[ "$*" != *"--set-param"* ]]; then
  echo "NOTE: no --set-param given -> this is the PROJECT BASELINE of this"
  echo "NOTE: harness (TECS on compiled firmware defaults EXCEPT TECS_PTCH_DAMP"
  echo "NOTE: = 0.6, which comes from config/ardupilot/falcon_v2_sitl.parm;"
  echo "NOTE: parameter writes: NONE (PROJECT baseline))."
fi
echo "NOTE: arguments forwarded to the test: ${*:-<none>}"
echo "NOTE: output suffix: '${SFX:-<none>}'"

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

# WIND - ZERO WIND IS A PRECONDITION OF THIS REGRESSION.
#   * The test world contains no world-level wind system, but model/model.sdf
#     carries the FalconV2Wind plugin, so /model/falcon_v2/wind exists in EVERY
#     world and defaults to the zero vector (<steady_wind_mps>0 0 0</...>).
#     This run is therefore a ZERO-WIND run BY DEFAULT, not by ArduPlane being
#     wind-blind. The regression compares against zero-wind references, so the
#     test also GATES it (param precondition sim_wind_zero).
#   * The airspeed path is the OFFICIAL SIM_JSON one: model/model.sdf wires
#     <airspeed_topic> (FalconV2Pitot, EAS) and <wind_topic> (FalconV2Wind,
#     world-ENU airmass velocity) into ArduPilotPlugin, so the FDM packet
#     carries the SIM_JSON "airspeed" and "velocity_wind" keys,
#     DataKey::AIRSPEED is set, the SIM_JSON.cpp:445 wind_ef.zero() branch is
#     NOT taken, and ARSPD_TYPE=100 / ARSPD_USE=1 feed that EAS to TECS. NO
#     physics bypass. See docs/source_of_truth/autopilot/
#     SITL_ATMOSPHERE_AND_AIRSPEED.md.
#   * SITL's own SIM_WIND_* cannot affect this run (the JSON backend never calls
#     Aircraft::update_wind()); wind comes from the Gazebo side only. The test
#     still reads SIM_WIND_SPD/DIR/TURB and records + gates them.
echo "=== ptch_damp regression : launching fresh gz sim + arduplane pair (scratch $SCRATCH) ==="
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
# energies this regression compares. The origin is set exactly instead, via
# SIM_OPOS_LAT/LNG/ALT/HDG in config/ardupilot/falcon_v2_sitl.parm, through
# SIM_Aircraft.cpp:694-707 update_home() - a path with no CMAC substitution.
# The test GATES SIM_OPOS_ALT == 0 so this can never regress silently.
# See docs/source_of_truth/autopilot/SITL_ATMOSPHERE_AND_AIRSPEED.md.
( cd "$SCRATCH" && gdb -q -x gdbcmds.txt --args \
    "$ARDUPLANE_BIN" -w -M json \
    --defaults "$SITL_PARM" -I 0 --speedup 1 > "$SCRATCH/arduplane.log" 2>&1 ) &
AP_PID=$!
sleep 8

echo "=== ptch_damp regression : running test ==="
cd "$REPO_ROOT"
mkdir -p "$RESULTS"
python3 "$TEST_PY" "$@" 2>&1 | tee "$LOG_OUT"
RC=${PIPESTATUS[0]}

echo "=== ptch_damp regression : done (rc=$RC), logs in $SCRATCH ==="
cp "$SCRATCH/gz.log"        "$RESULTS/${PREFIX}_gz_log${SFX}.txt" 2>/dev/null || true
cp "$SCRATCH/arduplane.log" "$RESULTS/${PREFIX}_arduplane_log${SFX}.txt" 2>/dev/null || true
if [[ -d "$SCRATCH/logs" ]]; then
  mkdir -p "$RESULTS/${PREFIX}_dataflash${SFX}"
  cp "$SCRATCH"/logs/*.BIN "$RESULTS/${PREFIX}_dataflash${SFX}/" 2>/dev/null || true
fi
exit $RC

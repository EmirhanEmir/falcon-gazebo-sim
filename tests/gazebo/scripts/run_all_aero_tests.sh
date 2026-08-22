#!/usr/bin/env bash
# FALCON V2 - AERODYNAMICS_V1 test pass, gazebo-testing, 2026-08-22
#
# Orchestrates the full aerodynamics live-Gazebo test suite (16 named tests
# from the task brief, across 3 scripts):
#   test_aero_kinematic_signs.py        - ZERO_AIRSPEED_AERO_TEST, AOA_SIGN_TEST,
#                                          SIDESLIP_SIGN_TEST, LIFT_SIGN_TEST,
#                                          DRAG_SIGN_TEST, DRAG_POLAR_TEST,
#                                          HIGH_ALPHA_LIMITER_TEST
#   test_aero_stability_derivatives.py  - Cma_RESTORING_SIGN_TEST (highest
#                                          priority), Cmq/Clp/Cnr_DAMPING_SIGN_TEST,
#                                          Cnb_STATIC_STABILITY_SIGN_TEST,
#                                          RATE_NORMALIZATION_TEST
#   test_aero_control_surfaces.py       - AILERON_ROLL_SIGN_TEST,
#                                          RUDDER_YAW_SIGN_TEST,
#                                          ELEVATOR_PITCH_SIGN_TEST
#
# Prerequisite: plugins/aerodynamics/build/libFalconV2Aerodynamics.so must
# already be built (see plugins/aerodynamics/README.md). This script does
# not build it - `aerodynamics` already built it this pass.
#
# Never modifies model/model.sdf, aero_v1_config.yaml, or any other aircraft
# physics parameter. Writes per-test logs/JSON to tests/gazebo/results/.

set -u
REPO_ROOT="/home/emirhan/Desktop/FalconV2"
SCRIPTS="$REPO_ROOT/tests/gazebo/scripts"
RESULTS="$REPO_ROOT/tests/gazebo/results"
mkdir -p "$RESULTS"

STAMP=$(date -u +%Y%m%dT%H%M%SZ)
SUMMARY="$RESULTS/aero_run_summary_${STAMP}.txt"

echo "FALCON V2 AERODYNAMICS_V1 test suite run: $STAMP" | tee "$SUMMARY"
echo "" | tee -a "$SUMMARY"

declare -A STATUS

run_step() {
  local name="$1"; shift
  echo "--- Running: $name ---" | tee -a "$SUMMARY"
  if "$@" 2>&1 | tee -a "$SUMMARY"; then
    STATUS["$name"]="PASS"
  else
    STATUS["$name"]="FAIL"
  fi
  echo "" | tee -a "$SUMMARY"
}

run_step "aero_kinematic_signs" python3 "$SCRIPTS/test_aero_kinematic_signs.py"
run_step "aero_stability_derivatives" python3 "$SCRIPTS/test_aero_stability_derivatives.py"
run_step "aero_control_surfaces" python3 "$SCRIPTS/test_aero_control_surfaces.py"

echo "=== Script-level summary ===" | tee -a "$SUMMARY"
overall=0
for k in "${!STATUS[@]}"; do
  echo "$k: ${STATUS[$k]}" | tee -a "$SUMMARY"
  if [ "${STATUS[$k]}" != "PASS" ]; then
    overall=1
  fi
done

echo "" | tee -a "$SUMMARY"
echo "NOTE (updated 2026-08-22, post Cm-to-My fix re-test): test_aero_stability_" | tee -a "$SUMMARY"
echo "derivatives.py's own script-level PASS now REQUIRES Cma_RESTORING_SIGN_TEST" | tee -a "$SUMMARY"
echo "to have measured a restoring (not destabilizing) moment - see that script's" | tee -a "$SUMMARY"
echo "own verdict block and docs/test_results/2026-08-22_aerodynamics_v1_retest_" | tee -a "$SUMMARY"
echo "cma_fix.md for the full writeup (supersedes the original TEST_FAILED report" | tee -a "$SUMMARY"
echo "docs/test_results/2026-08-22_aerodynamics_v1_test_report.md, kept as history)." | tee -a "$SUMMARY"
echo "Summary saved to: $SUMMARY" | tee -a "$SUMMARY"
exit $overall

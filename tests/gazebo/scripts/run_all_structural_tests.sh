#!/usr/bin/env bash
# FALCON V2 - structural V1 test pass, gazebo-testing, 2026-08-21
#
# Orchestrates the full structural-scope test run (the 11 tests listed in
# this pass's task: MODEL_LOAD_TEST, STATIC_GRAVITY_TEST,
# MASS_CG_INERTIA_TEST, CONTROL_JOINT_EXISTENCE_TEST,
# CONTROL_JOINT_LIMIT_TEST, CONTROL_JOINT_FREE_MOTION_TEST,
# PROP_JOINT_EXISTENCE_TEST, PROP_JOINT_AXIS_TEST,
# MESH_PLACEMENT_VISUAL_TEST, COLLISION_SANITY_TEST,
# NUMERICAL_STABILITY_IDLE_TEST). Deliberately does NOT run any
# aerodynamic/propulsion/trim/dynamic-mode test - none of that is
# implemented yet.
#
# Usage: tests/gazebo/scripts/run_all_structural_tests.sh
#
# Never modifies model/model.sdf or any other aircraft physics parameter.
# Writes per-test logs/JSON to tests/gazebo/results/.

set -u
REPO_ROOT="/home/emirhan/Desktop/FalconV2"
SCRIPTS="$REPO_ROOT/tests/gazebo/scripts"
RESULTS="$REPO_ROOT/tests/gazebo/results"
mkdir -p "$RESULTS"

STAMP=$(date -u +%Y%m%dT%H%M%SZ)
SUMMARY="$RESULTS/run_summary_${STAMP}.txt"

echo "FALCON V2 structural test suite run: $STAMP" | tee "$SUMMARY"
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

run_step "structural_state_capture" bash "$SCRIPTS/capture_structural_state.sh" "$RESULTS"
run_step "model_load" python3 "$SCRIPTS/test_model_load.py"
run_step "freefall_dynamics_suite" python3 "$SCRIPTS/test_freefall_dynamics.py"
run_step "control_joint_limits_and_motion" python3 "$SCRIPTS/test_control_joint_limits_and_motion.py"
run_step "prop_joints" python3 "$SCRIPTS/test_prop_joints.py"
run_step "mesh_placement" python3 "$SCRIPTS/test_mesh_placement.py"

echo "=== Script-level summary ===" | tee -a "$SUMMARY"
overall=0
for k in "${!STATUS[@]}"; do
  echo "$k: ${STATUS[$k]}" | tee -a "$SUMMARY"
  if [ "${STATUS[$k]}" != "PASS" ]; then
    overall=1
  fi
done

echo "" | tee -a "$SUMMARY"
echo "Summary saved to: $SUMMARY"
exit $overall

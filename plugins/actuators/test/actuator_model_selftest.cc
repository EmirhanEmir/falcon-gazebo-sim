// =============================================================================
// FALCON V2 - Actuator/Servo V1 standalone, Gazebo-independent self-test
// =============================================================================
// Owner: controls-integration specialist agent. Task: ACTUATOR_SERVO_MODEL_V1
// (2026-08-23).
//
// Exercises the PURE-MATH core (ActuatorModel.hh) directly - no Gazebo
// instance, no ECM, no plugin loading. Mirrors
// plugins/aerodynamics/test/aero_model_selftest.cc and
// plugins/propulsion/test/propulsion_model_selftest.cc's structure/style.
//
// This is NOT a substitute for gazebo-testing's live-Gazebo suite. Per this
// task's own instruction, the following 4 tests need live Gazebo with BOTH
// the actuator and aerodynamics plugins running together and are
// gazebo-testing's job next, NOT exercised here:
//   ACTUAL_JOINT_STATE_FEEDS_AERO_TEST, LEFT_RIGHT_ELEVATOR_MAPPING_REGRESSION,
//   LEFT_RIGHT_AILERON_MAPPING_REGRESSION, RUDDER_MAPPING_REGRESSION
//
// This self-test covers the other 8, all of which are properties of
// ActuatorModel.hh's Stage 1 (ClampCommand)/Stage 2 (RateLimitSetpoint) -
// the deterministic, provably rate-/travel-bounded "commanded trajectory"
// kinematic chain (StepSetpoint()) - which is exactly what ActuatorSystem.cc
// uses to drive the real Gazebo joint (via the effort-bounded PID torque in
// Stage 3, PidEffort(), not exercised as a REQUIRED test here since its
// closed-loop behavior against the real joint's physics-engine-integrated
// dynamics is a live-Gazebo concern - see the bonus,
// NOT-one-of-the-8-required INFO/anti-windup/disturbance-rejection sections
// at the bottom of this file for additional sanity checks of
// PidEffort()+IntegrateJointStep() against a documented, self-test-only
// reference inertia).
//
// Updated 2026-08-24 (ACTUATOR_SERVO_MODEL_V1_GRAVITY_DROOP_FIX): PdEffort()
// was renamed PidEffort() and gained an integral term + anti-windup (see
// ActuatorModel.hh). This is a Stage-3 change only - it does not touch
// Stage 1/2 (ClampCommand/RateLimitSetpoint/StepSetpoint), so all 8 required
// tests below are UNCHANGED and still pass unmodified. Two new, NOT-one-of-
// the-8-required checks were added at the bottom of this file
// (ACTUATOR_INTEGRAL_ANTI_WINDUP_TEST, ACTUATOR_INTEGRAL_DISTURBANCE_
// REJECTION_INFO) to directly exercise the new behavior.
//
// Updated again 2026-08-24, same day, round 2 (validation found a MAJOR
// transient-overshoot regression in the round-1 fix above - see
// ActuatorModel.hh's file header and actuator_v1_config.yaml's
// sp_weight_derivation_note for the full finding): PidEffort() gained
// setpoint weighting on the P-term (cfg.spWeightB). Still a Stage-3-only
// change - the 8 required tests remain unmodified/unaffected. The
// ACTUATOR_INTEGRAL_ANTI_WINDUP_TEST Case C analytic fixed-point formula
// below was updated to account for spWeightB (the P-term it reasons about
// is now weighted); Cases A/B are unaffected (both saturate trivially
// regardless of spWeightB). A new bonus test,
// ACTUATOR_SETPOINT_STEP_OVERSHOOT_TEST, was added to directly reproduce
// the pre-charged-integral-plus-setpoint-step overshoot mechanism at
// reduced scale and confirm this round's fix meaningfully reduces it while
// the round-1 droop fix (ACTUATOR_INTEGRAL_DISTURBANCE_REJECTION_INFO,
// unmodified below) still holds.
//
// Build: see ../CMakeLists.txt (target actuator_model_selftest, C++ standard
// library only - no gz-math/gz-sim/Gazebo runtime required).
// =============================================================================
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "../ActuatorModel.hh"

using namespace falcon_v2_actuators;

namespace
{
int gPass = 0;
int gFail = 0;
int gInfo = 0;

void Check(const std::string &name, bool ok, const std::string &detail)
{
  if (ok)
  {
    ++gPass;
    std::printf("[PASS] %-40s %s\n", name.c_str(), detail.c_str());
  }
  else
  {
    ++gFail;
    std::printf("[FAIL] %-40s %s\n", name.c_str(), detail.c_str());
  }
}

void Info(const std::string &name, const std::string &detail)
{
  ++gInfo;
  std::printf("[INFO] %-40s %s\n", name.c_str(), detail.c_str());
}

/// \brief V1 config, mirroring
/// docs/source_of_truth/controls/actuator_v1_config.yaml's numeric values
/// exactly (same duplication pattern as aero_model_selftest.cc's
/// MakeFalconV2Config() / propulsion_model_selftest.cc's MakeMotorConfig() -
/// the authoritative single source of truth remains the YAML file, which
/// the real plugin actually loads at runtime).
ActuatorConfig MakeV1Config()
{
  ActuatorConfig c;
  c.minAngleRad = -0.7853981634;  // -45 deg, USER_CONFIRMED_MECHANICAL_CAPABILITY
  c.maxAngleRad = 0.7853981634;   // +45 deg, USER_CONFIRMED_MECHANICAL_CAPABILITY
  c.maxRateRadS = 5.2360;         // V1_PROVISIONAL, ~300 deg/s
  c.maxEffortNm = 0.20;           // V1_PROVISIONAL
  c.kp = 0.0034788;               // V1_PROVISIONAL, DERIVED against the placeholder joint inertia - see actuator_v1_config.yaml control_law block
  c.kd = 6.9576e-05;              // V1_PROVISIONAL, DERIVED (same basis)
  c.ki = 6.9576e-03;              // V1_PROVISIONAL, DERIVED - added 2026-08-24 gravity-droop fix, see actuator_v1_config.yaml integral_derivation_note (Ti=0.5s)
  c.spWeightB = 0.7;              // V1_PROVISIONAL, DERIVED - added 2026-08-24 round-2 (OVERSHOOT_AFTER_STEP) fix, see actuator_v1_config.yaml sp_weight_derivation_note
  return c;
}

}  // namespace

int main()
{
  const ActuatorConfig cfg = MakeV1Config();
  const double dt = 0.001;  // 1 kHz, representative of a real-time-factor-1 gz-sim physics step

  std::printf("=================================================================\n");
  std::printf("FALCON V2 actuator/servo standalone self-test (ActuatorModel.hh only)\n");
  std::printf("=================================================================\n\n");

  // -----------------------------------------------------------------------
  // 1. ACTUATOR_NEUTRAL_START_TEST
  // -----------------------------------------------------------------------
  {
    // No command ever received: rawCmd defaults to 0.0, setpoint state
    // starts at 0.0 (ActuatorSystem.hh in-class initializers). One step at
    // rawCmd=0.0 from setpoint=0.0 must produce exactly 0.0, no kick.
    const double prevSetpoint = 0.0;
    const SetpointStepResult step = StepSetpoint(0.0, prevSetpoint, cfg, dt);
    bool ok = step.setpointRad == 0.0 && !step.targetWasClamped &&
              !step.rateLimitActive && std::isfinite(step.setpointRad);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "setpoint=%.9f rad (expected exactly 0.0), no clamp/rate-limit flags set",
        step.setpointRad);
    Check("ACTUATOR_NEUTRAL_START_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 2. ACTUATOR_POSITIVE_STEP_TEST
  // -----------------------------------------------------------------------
  {
    // Command +20 deg (well within +/-45 deg limit and reachable within a
    // reasonable number of steps at max_rate). Run the step-response
    // simulation to completion and confirm the setpoint monotonically
    // increases and settles exactly at the commanded target.
    const double targetDeg = 20.0;
    const double targetRadReal = targetDeg * M_PI / 180.0;
    double setpoint = 0.0;
    bool monotonic = true;
    double prev = 0.0;
    int steps = 0;
    const int maxSteps = 100000;
    for (; steps < maxSteps; ++steps)
    {
      const SetpointStepResult step = StepSetpoint(targetRadReal, setpoint, cfg, dt);
      if (step.setpointRad < prev - 1e-15) monotonic = false;
      prev = setpoint;
      setpoint = step.setpointRad;
      if (setpoint >= targetRadReal) break;
    }
    bool reachedExactly = std::abs(setpoint - targetRadReal) < 1e-12;
    bool ok = monotonic && reachedExactly && !std::isnan(setpoint);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "target=%.4f deg reached in %d steps (%.4f s), final=%.6f rad, monotonic=%d",
        targetDeg, steps, steps * dt, setpoint, monotonic ? 1 : 0);
    Check("ACTUATOR_POSITIVE_STEP_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 3. ACTUATOR_NEGATIVE_STEP_TEST
  // -----------------------------------------------------------------------
  {
    const double targetDeg = -20.0;
    const double targetRad = targetDeg * M_PI / 180.0;
    double setpoint = 0.0;
    bool monotonic = true;  // monotonically DEcreasing here
    double prev = 0.0;
    int steps = 0;
    const int maxSteps = 100000;
    for (; steps < maxSteps; ++steps)
    {
      const SetpointStepResult step = StepSetpoint(targetRad, setpoint, cfg, dt);
      if (step.setpointRad > prev + 1e-15) monotonic = false;
      prev = setpoint;
      setpoint = step.setpointRad;
      if (setpoint <= targetRad) break;
    }
    bool reachedExactly = std::abs(setpoint - targetRad) < 1e-12;
    bool ok = monotonic && reachedExactly && !std::isnan(setpoint);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "target=%.4f deg reached in %d steps (%.4f s), final=%.6f rad, monotonic=%d",
        targetDeg, steps, steps * dt, setpoint, monotonic ? 1 : 0);
    Check("ACTUATOR_NEGATIVE_STEP_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 4. ACTUATOR_RATE_LIMIT_TEST
  // -----------------------------------------------------------------------
  {
    // Command a huge instantaneous target (100 deg, beyond the mechanical
    // limit so the CLAMPED target is +45 deg) and confirm every single step
    // increment is bounded by max_rate*dt (checked at EVERY step, not just
    // the aggregate).
    const double rawCmd = 100.0 * M_PI / 180.0;
    double setpoint = 0.0;
    const double maxStep = cfg.maxRateRadS * dt;
    bool everyStepBounded = true;
    double worstOvershoot = 0.0;
    for (int i = 0; i < 20; ++i)  // well within the ramp - never reaches target in 20 steps
    {
      const SetpointStepResult step = StepSetpoint(rawCmd, setpoint, cfg, dt);
      const double actualDelta = step.setpointRad - setpoint;
      const double overshoot = std::abs(actualDelta) - maxStep;
      if (overshoot > 1e-12)
      {
        everyStepBounded = false;
        worstOvershoot = std::max(worstOvershoot, overshoot);
      }
      setpoint = step.setpointRad;
    }
    bool ok = everyStepBounded;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "max_step_allowed=%.6e rad (max_rate*dt), 20 steps checked individually, worst_overshoot=%.3e rad",
        maxStep, worstOvershoot);
    Check("ACTUATOR_RATE_LIMIT_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 5. ACTUATOR_POSITIVE_45_LIMIT_TEST
  // -----------------------------------------------------------------------
  {
    const double rawCmd = 60.0 * M_PI / 180.0;  // beyond +45 deg
    const double targetClamped = ClampCommand(rawCmd, cfg);
    double setpoint = 0.0;
    for (int i = 0; i < 100000; ++i)
    {
      const SetpointStepResult step = StepSetpoint(rawCmd, setpoint, cfg, dt);
      setpoint = step.setpointRad;
      if (setpoint >= targetClamped) break;
    }
    bool ok = std::abs(targetClamped - cfg.maxAngleRad) < 1e-12 &&
              std::abs(setpoint - cfg.maxAngleRad) < 1e-9 &&
              setpoint <= cfg.maxAngleRad + 1e-9;  // never exceeds
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "cmd=60deg -> target_clamped=%.6f rad (=max_angle_rad=%.6f), final_setpoint=%.6f, never exceeded",
        targetClamped, cfg.maxAngleRad, setpoint);
    Check("ACTUATOR_POSITIVE_45_LIMIT_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 6. ACTUATOR_NEGATIVE_45_LIMIT_TEST
  // -----------------------------------------------------------------------
  {
    const double rawCmd = -60.0 * M_PI / 180.0;
    const double targetClamped = ClampCommand(rawCmd, cfg);
    double setpoint = 0.0;
    for (int i = 0; i < 100000; ++i)
    {
      const SetpointStepResult step = StepSetpoint(rawCmd, setpoint, cfg, dt);
      setpoint = step.setpointRad;
      if (setpoint <= targetClamped) break;
    }
    bool ok = std::abs(targetClamped - cfg.minAngleRad) < 1e-12 &&
              std::abs(setpoint - cfg.minAngleRad) < 1e-9 &&
              setpoint >= cfg.minAngleRad - 1e-9;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "cmd=-60deg -> target_clamped=%.6f rad (=min_angle_rad=%.6f), final_setpoint=%.6f, never exceeded",
        targetClamped, cfg.minAngleRad, setpoint);
    Check("ACTUATOR_NEGATIVE_45_LIMIT_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 7. ACTUATOR_OVERCOMMAND_CLAMP_TEST
  // -----------------------------------------------------------------------
  {
    // An extreme overcommand (1000 deg) must clamp the TARGET to exactly
    // +45 deg - never let it leak through unclamped.
    const double rawCmd = 1000.0 * M_PI / 180.0;
    const SetpointStepResult step = StepSetpoint(rawCmd, 0.0, cfg, dt);
    bool ok = std::abs(step.targetClampedRad - cfg.maxAngleRad) < 1e-15 &&
              step.targetWasClamped;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "cmd=1000deg -> target_clamped=%.9f rad (expected exactly max_angle_rad=%.9f), targetWasClamped=%d",
        step.targetClampedRad, cfg.maxAngleRad, step.targetWasClamped ? 1 : 0);
    Check("ACTUATOR_OVERCOMMAND_CLAMP_TEST", ok, buf);

    // Symmetric negative overcommand, same check.
    const double rawCmdNeg = -1000.0 * M_PI / 180.0;
    const SetpointStepResult stepNeg = StepSetpoint(rawCmdNeg, 0.0, cfg, dt);
    bool okNeg = std::abs(stepNeg.targetClampedRad - cfg.minAngleRad) < 1e-15 &&
                 stepNeg.targetWasClamped;
    char bufNeg[256];
    std::snprintf(bufNeg, sizeof(bufNeg),
        "cmd=-1000deg -> target_clamped=%.9f rad (expected exactly min_angle_rad=%.9f), targetWasClamped=%d",
        stepNeg.targetClampedRad, cfg.minAngleRad, stepNeg.targetWasClamped ? 1 : 0);
    Check("ACTUATOR_OVERCOMMAND_CLAMP_TEST (negative)", okNeg, bufNeg);
  }

  // -----------------------------------------------------------------------
  // 8. ACTUATOR_NO_TELEPORT_TEST
  // -----------------------------------------------------------------------
  {
    // Full step-response simulation: 0 -> +45 deg (a large, worst-case
    // overcommand target is used further above; here we exercise a full,
    // in-range step commanded at t=0 with no ramp-up) and confirm, at
    // EVERY SINGLE STEP of the entire trajectory (not just the final
    // result), that the position change is bounded by max_rate*dt. Also
    // covers a mid-trajectory large overcommand (a step FROM +45 to -45,
    // the largest possible single-tick target jump) to confirm the
    // guarantee holds even for the most adversarial input, not just a
    // modest step.
    const double maxStep = cfg.maxRateRadS * dt;
    bool allStepsBounded = true;
    double worstStep = 0.0;
    std::size_t stepCount = 0;

    // Phase A: startup step to +45 deg (max in-range target).
    double setpoint = 0.0;
    const double targetA = cfg.maxAngleRad;
    for (int i = 0; i < 1000000 && setpoint < targetA; ++i)
    {
      const SetpointStepResult step = StepSetpoint(targetA, setpoint, cfg, dt);
      const double delta = std::abs(step.setpointRad - setpoint);
      worstStep = std::max(worstStep, delta);
      if (delta > maxStep + 1e-12) allStepsBounded = false;
      setpoint = step.setpointRad;
      ++stepCount;
    }
    // Phase B: the largest possible single-tick target jump, +45 -> -45 deg,
    // commanded instantaneously at one tick (worst-case adversarial input).
    const double targetB = cfg.minAngleRad;
    for (int i = 0; i < 1000000 && setpoint > targetB; ++i)
    {
      const SetpointStepResult step = StepSetpoint(targetB, setpoint, cfg, dt);
      const double delta = std::abs(step.setpointRad - setpoint);
      worstStep = std::max(worstStep, delta);
      if (delta > maxStep + 1e-12) allStepsBounded = false;
      setpoint = step.setpointRad;
      ++stepCount;
    }

    bool ok = allStepsBounded && std::isfinite(setpoint);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "%zu steps checked (0->+45deg, then +45->-45deg), max_step_allowed=%.6e rad, worst_observed_step=%.6e rad, all_bounded=%d",
        stepCount, maxStep, worstStep, allStepsBounded ? 1 : 0);
    Check("ACTUATOR_NO_TELEPORT_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // BONUS (not one of the 8 required tests) - PD_EFFORT_CLAMP_TEST: confirm
  // PidEffort()'s own software torque clamp is correct in isolation.
  // -----------------------------------------------------------------------
  {
    // 100 rad tracking error is deliberately unrealistic (a real joint
    // error can never exceed ~pi given the +/-45 deg travel limits) - it
    // exists ONLY to force PidEffort()'s clamp branch regardless of the
    // specific (small, placeholder-inertia-derived) kp value, confirming
    // the software clamp itself is correct in isolation. prevIntegral=0,
    // targetClampActive=false (this test is about the EFFORT clamp, not
    // the target clamp).
    const EffortResult big = PidEffort(
        /*setpoint=*/100.0, /*actual=*/0.0, /*rate=*/0.0,
        /*prevIntegralRadS=*/0.0, dt, /*targetClampActive=*/false, cfg);
    bool ok = std::abs(big.torqueNm) <= cfg.maxEffortNm + 1e-15 && big.effortClamped;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "large 100 rad tracking error -> torque=%.4f N*m (clamped to +/-%.4f), effortClamped=%d",
        big.torqueNm, cfg.maxEffortNm, big.effortClamped ? 1 : 0);
    Check("PD_EFFORT_CLAMP_TEST (bonus, not one of the 8 required)", ok, buf);
  }

  // -----------------------------------------------------------------------
  // BONUS (not one of the 8 required tests) - ACTUATOR_INTEGRAL_ANTI_WINDUP_
  // TEST: confirm the integral accumulator does NOT grow without bound when
  // (a) the target is clamped (Stage 1 saturation - an unreachable setpoint
  // must not be integrated against) and (b) the effort output is pinned at
  // +max_effort_nm by a large, sustained tracking error (added 2026-08-24,
  // gravity-droop fix's anti-windup requirement).
  // -----------------------------------------------------------------------
  {
    // Case (a): target clamp active. Repeatedly call PidEffort() with
    // targetClampActive=true and a nonzero error; the integral must stay
    // exactly at its previous value every single call (frozen, not merely
    // "growing slower").
    double integral = 0.0;
    bool frozenEveryStepA = true;
    for (int i = 0; i < 1000; ++i)
    {
      const EffortResult r = PidEffort(
          /*setpoint=*/cfg.maxAngleRad, /*actual=*/0.0, /*rate=*/0.0,
          integral, dt, /*targetClampActive=*/true, cfg);
      if (r.integralRadS != integral || !r.integralFrozen) frozenEveryStepA = false;
      integral = r.integralRadS;
    }
    bool okA = frozenEveryStepA && integral == 0.0;

    // Case (b): sustained large SAME-SIGN error with no target clamp - a
    // setpoint of 1000 rad (deliberately unrealistic, same style as
    // PD_EFFORT_CLAMP_TEST above) makes kp*error ALONE (kp*1000 = 3.48 N*m)
    // exceed max_effort_nm (0.20 N*m) on the very first tick, before the
    // integral term has contributed anything - so anti-windup must freeze
    // starting from tick 1, and the integral must never move off 0.0.
    double integralB = 0.0;
    double maxAbsTorque = 0.0;
    bool torqueBounded = true;
    bool frozenEveryStepB = true;
    for (int i = 0; i < 5000; ++i)
    {
      const EffortResult r = PidEffort(
          /*setpoint=*/1000.0, /*actual=*/0.0, /*rate=*/0.0,
          integralB, dt, /*targetClampActive=*/false, cfg);
      if (r.integralRadS != integralB || !r.integralFrozen) frozenEveryStepB = false;
      integralB = r.integralRadS;
      maxAbsTorque = std::max(maxAbsTorque, std::abs(r.torqueNm));
      if (std::abs(r.torqueNm) > cfg.maxEffortNm + 1e-12) torqueBounded = false;
    }
    bool okB = torqueBounded && frozenEveryStepB && integralB == 0.0;

    // Case (c): a smaller, realistic-scale (10 rad, still deliberately
    // beyond +/-45 deg/pi so the closed loop never actually closes the
    // error - actual angle is held fixed at 0.0 throughout, unlike a real
    // closed loop) sustained error where kp*errorP ALONE (errorP =
    // cfg.spWeightB*10 since actual=0.0 throughout; kp*0.7*10=0.02435 N*m
    // at this V1 cfg.spWeightB=0.7 - see the round-2 setpoint-weighting
    // note in ActuatorModel.hh/actuator_v1_config.yaml) does NOT yet
    // saturate - the integral is free to accumulate at first,
    // then anti-windup engages only once the running total pushes the
    // output to +max_effort_nm. Confirms accumulation is bounded (freezes
    // at the theoretical fixed point once reached) rather than growing
    // without limit for the REST of a long run, and that torque never
    // exceeds max_effort_nm throughout.
    double integralC = 0.0;
    bool torqueBoundedC = true;
    for (int i = 0; i < 5000; ++i)
    {
      const EffortResult r = PidEffort(
          /*setpoint=*/10.0, /*actual=*/0.0, /*rate=*/0.0,
          integralC, dt, /*targetClampActive=*/false, cfg);
      integralC = r.integralRadS;
      if (std::abs(r.torqueNm) > cfg.maxEffortNm + 1e-12) torqueBoundedC = false;
    }
    // Analytical fixed point: freeze engages once kp*errorP + ki*integral >=
    // max_effort_nm (errorP = cfg.spWeightB*10, actual held at 0.0), i.e.
    // integral >= (max_effort_nm - kp*cfg.spWeightB*10)/ki =
    // (0.20 - 0.0034788*0.7*10)/6.9576e-3 ~= 25.245 rad*s (was ~23.746
    // rad*s pre-round-2, when the P-term used the unweighted error - this
    // bound moves because the anti-windup freeze condition is evaluated
    // against the SAME weighted P-term PidEffort() actually commands, by
    // design - see PidEffort()'s doc comment). Confirm the integral settles
    // near that computed bound (not the ~50 rad*s = 5000*dt*10 it would
    // reach if NEVER frozen, and not grossly overshooting the bound either -
    // the freeze is not lagged).
    const double expectedFixedPoint =
        (cfg.maxEffortNm - cfg.kp * cfg.spWeightB * 10.0) / cfg.ki;
    bool okC = torqueBoundedC &&
        std::abs(integralC - expectedFixedPoint) < 0.05 * expectedFixedPoint;

    bool ok = okA && okB && okC;
    char buf[384];
    std::snprintf(buf, sizeof(buf),
        "targetClamp-frozen final_integral=%.6e (expected exactly 0), "
        "effortClamp-frozen(immediate) final_integral=%.6e rad*s (expected "
        "exactly 0), effortClamp-frozen(gradual) final_integral=%.6e rad*s "
        "(expected ~%.4f rad*s fixed point, not the ~50 rad*s unfrozen "
        "reference), max_|torque|_seen=%.4f N*m (<=max_effort_nm=%.4f)",
        integral, integralB, integralC, expectedFixedPoint, maxAbsTorque,
        cfg.maxEffortNm);
    Check("ACTUATOR_INTEGRAL_ANTI_WINDUP_TEST (bonus, not one of the 8 required)", ok, buf);
  }

  // -----------------------------------------------------------------------
  // BONUS (not one of the 8 required tests) - ACTUATOR_INTEGRAL_DISTURBANCE_
  // REJECTION_INFO: reproduces, in closed loop against the SAME self-test-
  // only reference inertia used by CLOSED_LOOP_TRACKING_INFO below, the
  // exact failure mode gazebo-testing found live (a constant disturbance
  // torque - standing in for gravity acting on the placeholder-mass control
  // surface about its hinge - causes a pure-P(D) law to settle short of
  // target) and confirms PidEffort()'s integral term eliminates it. This is
  // the pure-math analogue of the live-Gazebo gravity-droop finding, not a
  // substitute for gazebo-testing's own live re-measurement.
  // -----------------------------------------------------------------------
  {
    const double refInertia = 3.4788e-07;
    // Disturbance magnitude is order-of-magnitude matched to the live
    // finding: observed droop (2.6-4.2 deg) * kp implies T_gravity
    // ~1.6-2.5e-4 N*m; 2.0e-4 N*m is a representative mid-range value, NOT
    // a re-derivation of the real gravity torque (that remains
    // gazebo-testing's live measurement to make).
    const double disturbanceNm = 2.0e-4;
    const int subSteps = 1000;
    const double subDt = dt / subSteps;
    const double targetRad = 10.0 * M_PI / 180.0;

    auto RunClosedLoop = [&](double kiUsed, double simSeconds) -> double
    {
      ActuatorConfig localCfg = cfg;
      localCfg.ki = kiUsed;
      // Pinned to 1.0 (round-1 behavior, unweighted P-term) regardless of
      // cfg.spWeightB (0.7 as of the round-2 fix below) - this specific
      // test isolates the ki (integral) effect ONLY, exactly matching the
      // "droop = T_gravity/kp, independent of any setpoint weighting"
      // derivation this test's own ki=0 branch is meant to reproduce as a
      // faithful round-1 baseline. Round-2's setpoint-weighting effect on
      // this same disturbance-rejection scenario (which is unaffected by
      // spWeightB in the FULL PID case - see ACTUATOR_SETPOINT_STEP_
      // OVERSHOOT_TEST below for the transient/overshoot-focused check) is
      // deliberately kept out of this test to avoid conflating two
      // different rounds' fixes in one comparison.
      localCfg.spWeightB = 1.0;
      double angle = 0.0, rate = 0.0, setpoint = 0.0, integral = 0.0;
      const int outerSteps = static_cast<int>(simSeconds / dt);
      for (int i = 0; i < outerSteps; ++i)
      {
        const SetpointStepResult step = StepSetpoint(targetRad, setpoint, localCfg, dt);
        setpoint = step.setpointRad;
        const EffortResult eff = PidEffort(
            setpoint, angle, rate, integral, dt, step.targetWasClamped, localCfg);
        integral = eff.integralRadS;
        for (int s = 0; s < subSteps; ++s)
        {
          const JointStepResult js = IntegrateJointStep(
              angle, rate, eff.torqueNm + disturbanceNm, refInertia, subDt);
          angle = js.angleRad;
          rate = js.rateRadS;
        }
      }
      return (setpoint - angle) * 180.0 / M_PI;  // residual, deg
    };

    const double residualPdOnlyDeg = RunClosedLoop(/*kiUsed=*/0.0, /*simSeconds=*/8.0);
    const double residualPidDeg = RunClosedLoop(/*kiUsed=*/cfg.ki, /*simSeconds=*/8.0);

    // Reproduces the bug (pure P(D), ki=0): residual should be a clearly
    // nonzero, degree-scale droop (matching the live 2.6-4.2 deg finding's
    // order of magnitude, not an exact reproduction - different reference
    // inertia/disturbance value).
    bool reproducesBug = std::abs(residualPdOnlyDeg) > 1.0;
    // Confirms the fix (PID, ki=cfg.ki): residual should have collapsed to
    // a small fraction of a degree over the same 8 s window.
    bool fixesBug = std::abs(residualPidDeg) < 0.05;

    bool ok = reproducesBug && fixesBug;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "constant disturbance=%.2e N*m, target=10deg, t=8s: ki=0 (pre-fix) "
        "residual=%.4f deg (bug reproduced), ki=%.4e (post-fix) "
        "residual=%.4f deg (bug fixed)",
        disturbanceNm, residualPdOnlyDeg, cfg.ki, residualPidDeg);
    Check("ACTUATOR_INTEGRAL_DISTURBANCE_REJECTION_INFO (bonus, not one of the 8 required)", ok, buf);
  }

  // -----------------------------------------------------------------------
  // BONUS (not one of the 8 required tests) - ACTUATOR_SETPOINT_STEP_
  // OVERSHOOT_TEST: reproduces, at reduced scale against the SAME self-
  // test-only reference inertia/disturbance magnitude used by
  // ACTUATOR_INTEGRAL_DISTURBANCE_REJECTION_INFO above, the SAME-DAY
  // round-2 finding (validation, 2026-08-24 - see ActuatorModel.hh's file
  // header and actuator_v1_config.yaml's sp_weight_derivation_note for the
  // full narrative): a transient overshoot on a setpoint step taken in the
  // SAME direction the integral term was pre-charged to fight a constant
  // disturbance in (matching the live gravity-loaded test's own neutral-
  // hold warm-up procedure before each step command), and confirms this
  // round's setpoint-weighting fix (cfg.spWeightB) meaningfully reduces it
  // WITHOUT reintroducing the round-1 steady-state droop fix. This is a
  // reduced-order pure-math analogue of the live-Gazebo finding (this
  // model's constant, target-independent disturbance is a simplification -
  // see sp_weight_derivation_note for why the live magnitude/directionality
  // is expected to exceed what this model reproduces), not a substitute
  // for gazebo-testing's own live re-measurement.
  // -----------------------------------------------------------------------
  {
    const double refInertia = 3.4788e-07;
    // SAME disturbance magnitude/basis as ACTUATOR_INTEGRAL_DISTURBANCE_
    // REJECTION_INFO above (2.0e-4 N*m, order-of-magnitude matched to the
    // live droop finding, not a re-derivation of the real gravity torque).
    // Sign chosen so the disturbance pulls the surface toward NEGATIVE
    // angle - "the disturbance's own direction" for this test is therefore
    // a step to a NEGATIVE target.
    const double disturbanceNm = -2.0e-4;
    const int subSteps = 1000;
    const double subDt = dt / subSteps;
    const double warmupSeconds = 5.0;  // >> Ti=0.5s: integral fully pre-charged before the step, matching the live test's neutral-hold warm-up
    const double simSeconds = 5.0;     // long enough to observe the peak (live: 50-150ms) and reach a clean settle
    const double targetDeg = -20.0;    // step in the disturbance's own (pulling) direction

    // Returns {peakOvershootDeg, finalResidualDeg}.
    auto RunWarmupThenStep = [&](double spWeightBUsed) -> std::pair<double, double>
    {
      ActuatorConfig localCfg = cfg;
      localCfg.spWeightB = spWeightBUsed;
      double angle = 0.0, rate = 0.0, setpoint = 0.0, integral = 0.0;

      // Phase 1: neutral-hold warm-up under the constant disturbance -
      // charges the integral to whatever cancels the disturbance at
      // setpoint=0, exactly mirroring the live test's own pre-command hold.
      const int warmSteps = static_cast<int>(warmupSeconds / dt);
      for (int i = 0; i < warmSteps; ++i)
      {
        const SetpointStepResult step = StepSetpoint(0.0, setpoint, localCfg, dt);
        setpoint = step.setpointRad;
        const EffortResult eff = PidEffort(
            setpoint, angle, rate, integral, dt, step.targetWasClamped, localCfg);
        integral = eff.integralRadS;
        for (int s = 0; s < subSteps; ++s)
        {
          const JointStepResult js = IntegrateJointStep(
              angle, rate, eff.torqueNm + disturbanceNm, refInertia, subDt);
          angle = js.angleRad;
          rate = js.rateRadS;
        }
      }

      // Phase 2: step the setpoint to targetDeg (disturbance's own
      // direction) and track the peak excursion PAST the target.
      const double targetRad = targetDeg * M_PI / 180.0;
      const int simSteps = static_cast<int>(simSeconds / dt);
      double peakOvershootDeg = 0.0;
      for (int i = 0; i < simSteps; ++i)
      {
        const SetpointStepResult step = StepSetpoint(targetRad, setpoint, localCfg, dt);
        setpoint = step.setpointRad;
        const EffortResult eff = PidEffort(
            setpoint, angle, rate, integral, dt, step.targetWasClamped, localCfg);
        integral = eff.integralRadS;
        for (int s = 0; s < subSteps; ++s)
        {
          const JointStepResult js = IntegrateJointStep(
              angle, rate, eff.torqueNm + disturbanceNm, refInertia, subDt);
          angle = js.angleRad;
          rate = js.rateRadS;
        }
        const double angleDeg = angle * 180.0 / M_PI;
        // targetDeg is negative here - "overshoot" means going MORE
        // negative than targetDeg.
        const double excessDeg = targetDeg - angleDeg;
        if (excessDeg > peakOvershootDeg) peakOvershootDeg = excessDeg;
      }
      const double finalResidualDeg = targetDeg - (angle * 180.0 / M_PI);
      return {peakOvershootDeg, finalResidualDeg};
    };

    const auto baseline = RunWarmupThenStep(/*spWeightBUsed=*/1.0);
    const auto fixed = RunWarmupThenStep(/*spWeightBUsed=*/cfg.spWeightB);
    const double peakBaselineDeg = baseline.first;
    const double residualBaselineDeg = baseline.second;
    const double peakFixedDeg = fixed.first;
    const double residualFixedDeg = fixed.second;
    (void)residualBaselineDeg;

    // Baseline (spWeightB=1.0, i.e. the exact round-1 P-term with no
    // setpoint weighting) must show a clearly nonzero overshoot here -
    // confirms this reduced-order model actually reproduces the mechanism
    // validation described, not a vacuous check.
    bool reproducesOvershoot = peakBaselineDeg > 0.1;
    // Fixed (spWeightB=cfg.spWeightB=0.7) must show a substantial,
    // well-justified reduction - target: comfortably under the live test's
    // own 1 deg acceptance bar. This reduced-order model's baseline
    // overshoot is itself well under 1 deg already (see
    // sp_weight_derivation_note for why the live magnitude is expected to
    // be larger), so the bar here is intentionally tightened well beyond
    // "<1 deg": overshoot must collapse to near-numerical-noise scale AND
    // drop by at least 90% relative to the baseline.
    bool overshootMeaningfullyReduced =
        peakFixedDeg < 0.05 && peakFixedDeg < 0.1 * peakBaselineDeg;
    // Round-1 droop fix must still hold: final residual after the step
    // stays sub-0.05 deg, matching ACTUATOR_INTEGRAL_DISTURBANCE_REJECTION_
    // INFO's own bar - confirms this round's P-term-only change did not
    // reintroduce the steady-state droop the I-term fixes.
    bool droopFixStillHolds = std::abs(residualFixedDeg) < 0.05;

    bool ok = reproducesOvershoot && overshootMeaningfullyReduced && droopFixStillHolds;
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "neutral-hold warm-up (%.1fs) under constant disturbance=%.2e N*m, "
        "then step to %.1fdeg (disturbance's own direction): "
        "spWeightB=1.0 (pre-fix) peak_overshoot=%.4fdeg (mechanism "
        "reproduced), spWeightB=%.2f (post-fix) peak_overshoot=%.4fdeg "
        "(%.1f%% reduction), post-fix final_residual=%.5fdeg (droop fix "
        "still holds, <0.05deg)",
        warmupSeconds, disturbanceNm, targetDeg, peakBaselineDeg,
        cfg.spWeightB, peakFixedDeg,
        100.0 * (1.0 - peakFixedDeg / peakBaselineDeg), residualFixedDeg);
    Check("ACTUATOR_SETPOINT_STEP_OVERSHOOT_TEST (bonus, not one of the 8 required)", ok, buf);
  }

  // -----------------------------------------------------------------------
  // INFO (not a required test) - closed-loop tracking sanity check, using a
  // SELF-TEST-ONLY reference inertia (NOT the real Gazebo joint inertia -
  // see ActuatorModel.hh / IntegrateJointStep()'s own doc comment). Purely
  // informational: confirms the PID gains chosen do not blow up/oscillate
  // unstably against SOME plausible small-inertia single-DOF load, nothing
  // more.
  //
  // GENUINE FINDING, documented rather than tuned away: model/model.sdf's
  // 5 control-surface links carry a TEMPORARY_NUMERICAL_MASS = 0.001 kg
  // placeholder (near-massless, NOT physical - geometry-structure's own
  // documented choice), giving a tiny reference inertia here
  // (~3.48e-7 kg*m^2) and therefore a very high natural frequency
  // (omega_n = sqrt(kp/I) ~ 2566 rad/s here) for this V1_PROVISIONAL PD
  // controller. A NAIVE explicit-Euler integrator (IntegrateJointStep(),
  // self-test-only) run at the SAME 1 kHz outer tick used for the
  // rate-limited setpoint is numerically UNSTABLE at that combination
  // (dt*kd/I >> 2, well outside explicit Euler's own stability region for a
  // system this stiff) - this was observed directly during this task's own
  // development (undamped-looking oscillation, several radians of swing).
  // This is a property of the SIMPLE EXPLICIT-EULER SELF-TEST INTEGRATOR
  // combined with the placeholder inertia, at THIS coarse dt - it is not
  // evidence that the real live-Gazebo joint will behave the same way,
  // because gz-sim's actual physics engine (a) uses a more numerically
  // robust implicit/semi-implicit constraint solver that internally
  // substeps far more finely than the plugin's own 1 kHz PreUpdate() tick
  // (mirrored below), and (b) is additionally hard-bounded by
  // gz::sim::Joint::SetVelocityLimits()/SetEffortLimits() regardless of any
  // transient PD overshoot - see ActuatorModel.hh's header comment and
  // ActuatorSystem.cc's Configure(). This substepped version confirms the
  // control law itself is NOT inherently divergent, only sensitive to a
  // too-coarse fixed-step explicit integration of this specific tiny
  // inertia - flagged here for gazebo-testing/validation as a concrete
  // area to watch for chatter/oscillation in the live joint response, not
  // silently hidden by re-tuning kp/kd against an inertia value that is
  // itself only a placeholder.
  // -----------------------------------------------------------------------
  {
    // Reference inertia: order of magnitude of the aileron's own SDF
    // placeholder Iyy (about its hinge-ish axis), model/model.sdf
    // left_aileron <inertial><inertia><iyy> = 3.4788e-07 kg*m^2 - cited only
    // as a representative small value for this INFORMATIONAL closed-loop
    // check, explicitly NOT a claim this equals what gz-sim will actually
    // integrate against (gz-sim uses the full SDF-defined rigid-body
    // inertia via its own physics engine, not this simplified single-DOF
    // integrator).
    const double refInertia = 3.4788e-07;
    // Torque is recomputed once per OUTER 1 kHz tick (matching one real
    // PreUpdate()/SetForce() call), but the reference single-DOF inertia is
    // integrated in many finer SUBSTEPS within that same tick (mirroring
    // how a real physics-engine constraint solver operates at a much finer
    // internal timescale than the plugin's own tick) - substepDt chosen
    // small enough to be comfortably inside explicit Euler's own stability
    // region for this specific (kp,kd,refInertia) combination.
    const int subSteps = 1000;
    const double subDt = dt / subSteps;
    double angle = 0.0, rate = 0.0, setpoint = 0.0, integral = 0.0;
    const double targetRad = 10.0 * M_PI / 180.0;
    double maxAngleSeen = 0.0;
    bool stable = true;
    for (int i = 0; i < 20000 && stable; ++i)
    {
      const SetpointStepResult step = StepSetpoint(targetRad, setpoint, cfg, dt);
      setpoint = step.setpointRad;
      const EffortResult eff = PidEffort(
          setpoint, angle, rate, integral, dt, step.targetWasClamped, cfg);
      integral = eff.integralRadS;
      for (int s = 0; s < subSteps; ++s)
      {
        const JointStepResult js =
            IntegrateJointStep(angle, rate, eff.torqueNm, refInertia, subDt);
        angle = js.angleRad;
        rate = js.rateRadS;
        maxAngleSeen = std::max(maxAngleSeen, std::abs(angle));
        if (!std::isfinite(angle) || !std::isfinite(rate) || std::abs(angle) > 10.0)
        {
          stable = false;
          break;
        }
      }
    }
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "ref_inertia=%.4e kg*m^2 (SDF-placeholder-order-of-magnitude, NOT the real gz-sim value), "
        "%d substeps/tick, final_angle=%.6f rad (target=%.6f), max_|angle|_seen=%.6f rad, stable=%d",
        refInertia, subSteps, angle, targetRad, maxAngleSeen, stable ? 1 : 0);
    Info("CLOSED_LOOP_TRACKING_INFO (not a required test)", buf);
    if (!stable)
    {
      Check("CLOSED_LOOP_TRACKING_INFO stability", false,
            "control law diverged even with substepping - see the finding "
            "documented above this block; does NOT block V1 by itself "
            "(gz-sim uses the real physics engine + SetVelocityLimits/"
            "SetEffortLimits, not this simplified integrator), but should "
            "be treated as a real risk flagged for gazebo-testing/"
            "validation, not silently ignored");
    }
  }

  std::printf("\n=================================================================\n");
  std::printf("RESULTS: %d PASS, %d FAIL, %d INFO\n", gPass, gFail, gInfo);
  std::printf("=================================================================\n");

  return (gFail == 0) ? 0 : 1;
}

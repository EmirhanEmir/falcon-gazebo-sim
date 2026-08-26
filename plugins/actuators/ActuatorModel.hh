// =============================================================================
// FALCON V2 - Actuator/Servo V1 core math model
// =============================================================================
// Owner: controls-integration specialist agent. Task: ACTUATOR_SERVO_MODEL_V1
// (2026-08-23). Updated 2026-08-24, task ACTUATOR_SERVO_MODEL_V1_GRAVITY_DROOP_FIX:
// see the "STEADY-STATE DISTURBANCE FIX" note below Stage 3 for what changed
// and why - this is a scoped control-law fix, not a redesign; Stages 1/2 and
// every clamp/limit value are UNCHANGED. Updated again 2026-08-24, SAME-DAY
// round 2 (validation found a new MAJOR: 1.96-3.46 deg transient OVERSHOOT,
// directional - only on setpoint steps in the same direction the integral
// was pre-charged to fight gravity in, e.g. after the neutral-hold warm-up
// used by the live gravity-loaded test): PidEffort() gained setpoint
// weighting (cfg.spWeightB) on the P-term only - see PidEffort()'s own doc
// comment and actuator_v1_config.yaml's sp_weight_derivation_note for the
// full rationale. kp/kd/ki, the anti-windup logic, and Stages 1/2 are
// UNCHANGED by this round; the I-term still uses the true (unweighted)
// error, so the round-1 steady-state droop fix is untouched by this round.
//
// This header contains ONLY pure math: no gz-sim (ECM/Entity/System)
// dependency AND no gz-math dependency either (every quantity here is a
// scalar - single-DOF revolute joints - so there is no need for
// gz::math::Vector3d the way plugins/aerodynamics/AeroModel.hh needs it).
// This goes one step further than AeroModel.hh and matches
// plugins/propulsion/PropulsionModel.hh's precedent of a fully
// Gazebo-independent header, linkable into both the real Gazebo System
// plugin (ActuatorSystem.cc) and a standalone self-test executable
// (test/actuator_model_selftest.cc) that runs with no Gazebo instance at
// all.
//
// ARCHITECTURE (see plugins/actuators/README.md and
// docs/source_of_truth/controls/actuator_v1_config.yaml for the full
// rationale/provenance - summarized here only as code-adjacent context):
//
//   raw command (rad) --ClampCommand()--> travel-clamped target
//                     --RateLimitSetpoint()--> rate-limited internal setpoint
//                     --PidEffort()--> bounded joint torque (N*m)
//
// The first two stages (StepSetpoint(), which composes ClampCommand() +
// RateLimitSetpoint()) are the actuator's PURE KINEMATIC "commanded
// trajectory" stage: a deterministic, provably rate- and travel-bounded
// function of time, independent of any physics-engine/inertia uncertainty.
// This is what plugins/actuators/test/actuator_model_selftest.cc's 8
// required pure-math tests exercise directly (ACTUATOR_NEUTRAL_START_TEST
// through ACTUATOR_NO_TELEPORT_TEST) - the "no instantaneous jump" guarantee
// for this stage is a direct, provable consequence of RateLimitSetpoint()'s
// own clamp, not an emergent property that depends on tuning.
//
// The third stage (PidEffort()) is what ActuatorSystem.cc uses to physically
// DRIVE the real Gazebo joint toward that rate-limited setpoint via
// gz::sim::Joint::SetForce() (a genuine joint-space generalized-force/torque
// command, integrated by gz-sim's own physics engine through the joint's
// real - currently TEMPORARY_NUMERICAL_MASS-placeholder, see model.sdf -
// inertia; never an instantaneous position write). Because the current
// control-surface link masses are an intentionally near-massless
// placeholder (geometry-structure's TEMPORARY_NUMERICAL_MASS = 0.001 kg,
// NOT a physical claim), ANY plausible real-servo torque value would be
// wildly overpowered relative to that placeholder inertia; therefore
// ActuatorSystem.cc does NOT rely on PidEffort()'s own torque clamp alone to
// keep the REAL joint's velocity bounded - it ALSO applies
// gz::sim::Joint::SetVelocityLimits()/SetEffortLimits() (genuine
// physics-engine-level hard constraints) using the SAME max_rate/max_effort
// values, so the real joint's motion is bounded by max_rate independently
// of the placeholder-inertia/torque mismatch. See ActuatorSystem.cc for
// that wiring - this header only supplies the underlying scalar formulas.
//
// STEADY-STATE DISTURBANCE FIX (2026-08-24, ACTUATOR_SERVO_MODEL_V1_GRAVITY_
// DROOP_FIX): gazebo-testing found, at the powered trim condition (gravity
// ON), that commanded elevator/aileron deflections settled 2.6-4.2 deg SHORT
// of target in a stable, non-oscillating, non-effort-saturated equilibrium
// (rudder unaffected, <=0.012 deg residual at all tested angles). Root
// cause, confirmed by a 3-way zero-g/base-free/base-held diagnostic plus a
// first-principles prediction (droop = T_gravity/kp, matching observed droop
// to within ~5-9% for all three surfaces, predicting exactly 0 deg for
// rudder since its Z-dominant hinge axis sees zero moment from a vertical
// gravity force by construction while the Y-dominant elevator/aileron axes
// do not): the ORIGINAL Stage-3 law (renamed PdEffort() -> PidEffort() by
// this fix) was pure P(D). A pure-P(D) law cannot drive steady-state error
// to zero under ANY nonzero constant disturbance torque (here: gravity
// acting on the control surface's own TEMPORARY_NUMERICAL_MASS about its
// hinge), regardless of kp - a larger kp only shrinks the droop
// proportionally, it never eliminates it, and kp cannot simply be cranked
// higher given the placeholder-inertia dt-compatibility bandwidth
// constraint already documented above (control_law derivation_note in
// actuator_v1_config.yaml). Fix chosen: integral action (PID), NOT gravity
// feedforward - see actuator_v1_config.yaml's control_law block for why
// integral was chosen over feedforward and the full ki derivation/stability
// analysis. Integral action rejects ANY constant disturbance (not gravity
// specifically) without needing to model the disturbance, and does not
// require this Gazebo-independent header (or ActuatorSystem.cc) to start
// reading link mass/CoM/orientation data it never needed before - a smaller
// blast radius for a scoped fix. Anti-windup: integral accumulation is
// frozen whenever Stage 1 has already clamped the target (targetClampActive
// argument to PidEffort(), below) OR whenever integrating this tick's error
// would keep the effort output saturated in the SAME direction it is
// already saturated in ("conditional integration" - a same-tick,
// non-lagged form of "freeze whenever effort_clamp_active is true").
//
// IntegrateJointStep() at the bottom of this file is a SELF-TEST-ONLY
// reference single-DOF rotational-inertia integrator (explicit Euler,
// mirrors PropulsionModel.hh's IntegrateRotorStep() pattern). It is NEVER
// called by ActuatorSystem.cc - the real joint's physical motion is
// integrated by gz-sim's own physics engine, not by this function. It
// exists solely so the standalone self-test can exercise PidEffort() in a
// closed loop against SOME finite, documented reference inertia and confirm
// the control law is stable/bounded - this is explicitly NOT a claim about
// the real Gazebo joint's dynamics (see the self-test's own comments).
//
// Every numeric constant a caller supplies here (min/max angle, max_rate,
// max_effort, kp, kd, ki) is loaded from
// docs/source_of_truth/controls/actuator_v1_config.yaml by
// ActuatorSystem.cc's Configure() step (mirrors AeroConfig/MotorConfig's
// pattern) - no coefficient is hardcoded in this header or in
// ActuatorSystem.cc.
// =============================================================================
#ifndef FALCON_V2_ACTUATOR_MODEL_HH_
#define FALCON_V2_ACTUATOR_MODEL_HH_

#include <algorithm>
#include <cmath>

namespace falcon_v2_actuators
{

// ===========================================================================
// Config - populated by the caller (ActuatorSystem.cc's Configure(), or a
// standalone self-test) from actuator_v1_config.yaml. No default value below
// is meant to be trusted for a real run - defaults exist only so a missing
// config key fails loudly/inert (e.g. maxRateRadS=0 freezes the setpoint,
// an obviously-wrong but finite, non-crashing behavior) rather than reading
// uninitialized memory.
// ===========================================================================
struct ActuatorConfig
{
  // Mechanical travel limits, rad. V1 = +/-45 deg = +/-0.7853981634 rad,
  // USER_CONFIRMED_MECHANICAL_CAPABILITY (see actuator_v1_config.yaml and
  // model/model.sdf's MECHANICAL_ACTUATOR_LIMIT_V1 joint-limit comments).
  double minAngleRad = 0.0;
  double maxAngleRad = 0.0;

  // Servo slew-rate limit, rad/s. V1_PROVISIONAL - no manufacturer datasheet
  // ingested for the named Emax ES08MAII servo (CONTROLS.md sec 5); see
  // actuator_v1_config.yaml for the documented order-of-magnitude reasoning.
  double maxRateRadS = 0.0;

  // Servo/actuator maximum effort (joint torque), N*m. V1_PROVISIONAL, same
  // no-datasheet caveat as maxRateRadS - see actuator_v1_config.yaml.
  double maxEffortNm = 0.0;

  // PID control-law gains used ONLY to drive the real Gazebo joint toward
  // the rate-limited setpoint (PidEffort() below) - V1_PROVISIONAL, see
  // actuator_v1_config.yaml "control_law" block for the documented
  // derivation (a dt=0.001s-compatible closed-loop bandwidth
  // (omega_n=100 rad/s, zeta=1.0) evaluated against the current SDF
  // placeholder joint inertia - NOT simply sized from maxEffortNm/a
  // reference tracking-error angle in isolation; an earlier draft that did
  // exactly that was found, by this project's own self-test, to be
  // numerically unstable at a 1 kHz per-tick torque-update rate given how
  // small the current placeholder joint inertia is - see the YAML
  // "derivation_note" for the full finding).
  double kp = 0.0;  // N*m / rad
  double kd = 0.0;  // N*m / (rad/s)

  // Integral gain, added 2026-08-24 (ACTUATOR_SERVO_MODEL_V1_GRAVITY_DROOP_
  // FIX) to eliminate the steady-state droop a pure-P(D) law cannot reject
  // under a constant disturbance torque (gravity acting on the control
  // surface's own placeholder mass about its hinge - see the file header
  // note above Stage 3 and actuator_v1_config.yaml's "control_law" block
  // for the full derivation: ki = kp / Ti with Ti = 0.5 s, chosen
  // conservatively slow relative to the kp/kd pair's own omega_n=100 rad/s
  // design, confirmed stable by Routh-Hurwitz with a large margin). Defaults
  // to 0.0 so a missing config key degrades to the old pure-P(D) behavior
  // (inert, not silently wrong) rather than reading uninitialized memory.
  double ki = 0.0;  // N*m / (rad*s), multiplies the integral-of-error state

  // Setpoint weight on the PROPORTIONAL term only, added 2026-08-24
  // (ACTUATOR_SERVO_MODEL_V1_GRAVITY_DROOP_FIX round 2 -
  // "OVERSHOOT_AFTER_STEP" fix). Standard PID "setpoint weighting" /
  // two-degree-of-freedom technique (see PidEffort() doc comment and
  // actuator_v1_config.yaml's sp_weight_derivation_note for the full
  // justification): the P-term uses (spWeightB*setpointRad - actualAngleRad)
  // instead of the full (setpointRad - actualAngleRad); the I-term (and the
  // anti-windup sign logic) ALWAYS use the true, unweighted error - this is
  // what keeps disturbance rejection (steady-state droop-rejection) fully
  // intact while only softening the P-term's own instantaneous reaction to a
  // SETPOINT step. Range is conventionally [0,1]; 1.0 = original behavior
  // (P uses the full, unweighted error - mathematically identical to every
  // config predating this round-2 fix). Defaults to 1.0 so a missing config
  // key degrades to the exact pre-existing behavior (inert, not silently
  // wrong) rather than reading uninitialized memory.
  double spWeightB = 1.0;  // dimensionless, conventionally in [0, 1]
};

// ===========================================================================
// Stage 1: travel clamp
// ===========================================================================

/// \brief Clamp a raw commanded angle (rad) to the mechanical travel limits.
/// Pure saturation - no rate/time dependence. This is what makes
/// ACTUATOR_POSITIVE_45_LIMIT_TEST / ACTUATOR_NEGATIVE_45_LIMIT_TEST /
/// ACTUATOR_OVERCOMMAND_CLAMP_TEST provable by construction.
inline double ClampCommand(double rawCmdRad, const ActuatorConfig &cfg)
{
  double lo = cfg.minAngleRad;
  double hi = cfg.maxAngleRad;
  if (lo > hi) std::swap(lo, hi);  // defensive only - a valid config never hits this
  return std::clamp(rawCmdRad, lo, hi);
}

// ===========================================================================
// Stage 2: rate-limited setpoint
// ===========================================================================

/// \brief Advance a persistent internal setpoint state by AT MOST
/// maxRateRadS * dt toward targetClampedRad, for one timestep. This single
/// clamp is the entire "finite rate, no instantaneous jump" guarantee: the
/// returned value can never differ from prevSetpointRad by more than
/// maxRateRadS * dt, for ANY prevSetpointRad/targetClampedRad pair,
/// including a target that is arbitrarily far away (a large overcommand) or
/// a startup jump from an uninitialized state - this is exactly the
/// ACTUATOR_RATE_LIMIT_TEST / ACTUATOR_NO_TELEPORT_TEST property.
inline double RateLimitSetpoint(
    double prevSetpointRad, double targetClampedRad, double maxRateRadS,
    double dt)
{
  if (dt <= 0.0 || maxRateRadS <= 0.0)
  {
    // Defensive only: a non-positive dt or rate means "do not move this
    // step" rather than divide/step incorrectly - never a jump.
    return prevSetpointRad;
  }
  const double maxStep = maxRateRadS * dt;
  const double delta =
      std::clamp(targetClampedRad - prevSetpointRad, -maxStep, maxStep);
  return prevSetpointRad + delta;
}

/// \brief Result of one full kinematic command-chain step (Stage 1 + Stage
/// 2 combined). ActuatorSystem.cc and the standalone self-test both call
/// StepSetpoint() (not the two stages separately) so the two can never
/// diverge in how they combine clamp + rate-limit.
struct SetpointStepResult
{
  double targetClampedRad = 0.0;  // rawCmd after Stage 1 (mechanical clamp)
  bool targetWasClamped = false;  // true if rawCmd was outside [min,max]
  double setpointRad = 0.0;       // rate-limited internal setpoint after this step
  bool rateLimitActive = false;   // true if this step's motion was capped by maxRate (setpoint has not yet reached targetClampedRad)
};

inline SetpointStepResult StepSetpoint(
    double rawCmdRad, double prevSetpointRad, const ActuatorConfig &cfg,
    double dt)
{
  SetpointStepResult out;
  out.targetClampedRad = ClampCommand(rawCmdRad, cfg);
  out.targetWasClamped = (out.targetClampedRad != rawCmdRad);
  out.setpointRad =
      RateLimitSetpoint(prevSetpointRad, out.targetClampedRad, cfg.maxRateRadS, dt);
  out.rateLimitActive = (out.setpointRad != out.targetClampedRad);
  return out;
}

// ===========================================================================
// Stage 3: effort-bounded PID control law (drives the REAL Gazebo joint;
// physical integration happens in gz-sim's own physics engine, never here)
// ===========================================================================

struct EffortResult
{
  double torqueNm = 0.0;
  bool effortClamped = false;

  // Updated integral-accumulator state (integral of error over time, rad*s)
  // - the caller (ActuatorSystem.cc, or the self-test) must persist this
  // and pass it back in as prevIntegralRadS on the NEXT call for the same
  // surface, exactly like setpointRad is already persisted across
  // StepSetpoint() calls. Added 2026-08-24 (gravity-droop fix).
  double integralRadS = 0.0;

  // True if anti-windup froze integral accumulation THIS call (either the
  // Stage-1 target was clamped, or integrating would have kept the effort
  // saturated in the same direction it already was) - informational, not
  // currently published to the diagnostics topic (unchanged 35-field
  // format, gazebo-testing's harness depends on that exact layout).
  bool integralFrozen = false;
};

/// \brief PID control law tracking the rate-limited setpoint, "derivative on
/// measurement" form (kd multiplies the actual joint's own angular rate,
/// not a derivative of the setpoint - avoids derivative-kick on a setpoint
/// step, standard practice). Output is hard-clamped to
/// +/-cfg.maxEffortNm - this is a software-level clamp mirrored by a
/// genuine physics-engine-level gz::sim::Joint::SetEffortLimits() call in
/// ActuatorSystem.cc (defense in depth, never relying on a single
/// mechanism, matching this project's established style).
///
/// SETPOINT WEIGHTING (added 2026-08-24, ACTUATOR_SERVO_MODEL_V1_GRAVITY_
/// DROOP_FIX round 2, "OVERSHOOT_AFTER_STEP" fix): the P-term uses a
/// WEIGHTED error, errorP = cfg.spWeightB*setpointRad - actualAngleRad,
/// instead of the plain error. The I-term (integral accumulation below) and
/// the anti-windup sign logic both continue to use the TRUE, unweighted
/// error (error, not errorP) - this is what a standard "two-degree-of-
/// freedom" PID structure means: only the proportional path's REACTION TO A
/// SETPOINT STEP is softened (cfg.spWeightB<1 shrinks the P-term's initial
/// kick when setpointRad jumps), while disturbance rejection - which is
/// driven entirely by the I-term integrating the TRUE error to zero - is
/// completely unaffected. With cfg.spWeightB=1.0 (the default, matching
/// every config predating this fix), errorP==error exactly and this
/// function is byte-for-byte the same computation as before. See
/// actuator_v1_config.yaml's sp_weight_derivation_note for why this
/// specific technique was chosen (it provably leaves the closed-loop
/// characteristic polynomial - and therefore the kp/kd/ki pole-placement/
/// Routh-Hurwitz analysis already on file - completely unchanged; setpoint
/// weighting only moves a ZERO of the reference-tracking transfer function,
/// never a POLE) and the root cause it addresses (a pre-charged integral
/// state, correct for the disturbance it was compensating, combined with a
/// full-strength P-term kick on a setpoint step, producing a transient
/// overshoot that was never present in the disturbance-rejection dynamics
/// themselves).
///
/// \param[in] prevIntegralRadS Integral-of-error state carried over from the
///   previous call for this SAME surface (0.0 on the first call / after a
///   reset). See EffortResult::integralRadS above.
/// \param[in] dt Timestep, s. Used only to advance the integral accumulator
///   (Stages 1/2's rate limiting is unaffected - unchanged from before this
///   fix).
/// \param[in] targetClampActive Stage 1's ClampCommand() result for this
///   tick (SetpointStepResult::targetWasClamped) - anti-windup input: the
///   rate-limited setpoint can never reach a clamped-away raw command, so
///   integrating against it would wind up for no reachable purpose.
///
/// Anti-windup ("conditional integration"): before deciding whether to
/// accumulate THIS tick's error into the integral, the would-be output is
/// evaluated using the PREVIOUS integral state (and the WEIGHTED P-term, so
/// the freeze decision matches what is actually being commanded). If that
/// would-be output is already saturated at +/-maxEffortNm AND the current
/// TRUE error has the SAME sign as that output (i.e. integrating further
/// would only push deeper into the same saturation), accumulation is frozen
/// for this tick - a same-tick equivalent of "freeze whenever
/// effort_clamp_active is true" that does not suffer a one-tick detection
/// lag. Accumulation is also frozen whenever targetClampActive is true.
/// This keeps the integral state bounded and prevents overshoot-on-recovery
/// after a saturating/overrange command, without needing a separate clamp
/// on integralRadS itself.
inline EffortResult PidEffort(
    double setpointRad, double actualAngleRad, double actualRateRadS,
    double prevIntegralRadS, double dt, bool targetClampActive,
    const ActuatorConfig &cfg)
{
  EffortResult out;
  const double error = setpointRad - actualAngleRad;
  const double errorP = cfg.spWeightB * setpointRad - actualAngleRad;

  const double rawBeforeIntegration =
      cfg.kp * errorP + cfg.ki * prevIntegralRadS - cfg.kd * actualRateRadS;
  const bool wouldSaturate = cfg.maxEffortNm > 0.0 &&
      std::abs(rawBeforeIntegration) >= cfg.maxEffortNm;
  const bool sameSignAsError =
      (rawBeforeIntegration >= 0.0 && error >= 0.0) ||
      (rawBeforeIntegration <= 0.0 && error <= 0.0);
  const bool freeze =
      targetClampActive || (wouldSaturate && sameSignAsError);

  double integral = prevIntegralRadS;
  if (!freeze && dt > 0.0)
  {
    integral = prevIntegralRadS + error * dt;
  }

  const double raw =
      cfg.kp * errorP + cfg.ki * integral - cfg.kd * actualRateRadS;
  double clamped = raw;
  if (cfg.maxEffortNm > 0.0)
  {
    clamped = std::clamp(raw, -cfg.maxEffortNm, cfg.maxEffortNm);
  }
  out.torqueNm = clamped;
  out.effortClamped = (clamped != raw);
  out.integralRadS = integral;
  out.integralFrozen = freeze;
  return out;
}

// ===========================================================================
// Self-test-ONLY reference single-DOF rotational-inertia integrator. NEVER
// called by ActuatorSystem.cc. See file header comment.
// ===========================================================================

struct JointStepResult
{
  double angleRad = 0.0;
  double rateRadS = 0.0;
};

/// \brief One explicit-Euler integration step of
///   I * domega/dt = torqueNm
/// for a SELF-TEST-ONLY reference single-DOF joint. inertiaKgM2 here is an
/// arbitrary, explicitly-documented self-test reference value (see
/// actuator_model_selftest.cc), NOT the real SDF joint inertia - the real
/// joint's motion is always integrated by gz-sim's own physics engine in
/// the live plugin, never by this function.
inline JointStepResult IntegrateJointStep(
    double angleRad, double rateRadS, double torqueNm, double inertiaKgM2,
    double dt)
{
  JointStepResult out;
  double newRate = rateRadS;
  if (inertiaKgM2 > 0.0)
  {
    newRate = rateRadS + dt * torqueNm / inertiaKgM2;
  }
  out.rateRadS = newRate;
  out.angleRad = angleRad + newRate * dt;
  return out;
}

}  // namespace falcon_v2_actuators

#endif  // FALCON_V2_ACTUATOR_MODEL_HH_

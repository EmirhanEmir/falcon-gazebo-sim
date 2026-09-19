// =============================================================================
// FALCON V2 - Actuator/Servo V1 core math model
// =============================================================================
// Pure math only: no gz-sim and no gz-math dependency. Every quantity here is
// a scalar, because all five control surfaces are single-DOF revolute joints.
//
// ARCHITECTURE (see plugins/actuators/README.md and
// config/actuator_v1_config.yaml for the full rationale and provenance):
//
//   raw command (rad) --ClampCommand()--> travel-clamped target
//                     --RateLimitSetpoint()--> rate-limited internal setpoint
//                     --PidEffort()--> bounded joint torque (N*m)
//
// Stages 1 and 2 (composed by StepSetpoint()) form the actuator's pure
// kinematic "commanded trajectory": a deterministic, provably rate- and
// travel-bounded function of time, independent of physics-engine or inertia
// uncertainty. The "no instantaneous jump" guarantee is a direct consequence
// of RateLimitSetpoint()'s own clamp, not an emergent property of tuning.
//
// Stage 3 (PidEffort()) is what ActuatorSystem.cc uses to drive the real
// Gazebo joint toward the rate-limited setpoint, via
// gz::sim::Joint::SetForce() - a genuine joint-space generalized torque
// integrated by the physics engine, never an instantaneous position write.
//
// The control-surface link masses in model.sdf are an intentional near-
// massless numerical placeholder (TEMPORARY_NUMERICAL_MASS = 0.001 kg), not
// a physical claim, so any plausible real-servo torque is wildly overpowered
// relative to that placeholder inertia. ActuatorSystem.cc therefore does NOT
// rely on PidEffort()'s torque clamp alone to bound joint velocity: it also
// applies gz::sim::Joint::SetVelocityLimits()/SetEffortLimits() (hard
// physics-engine constraints) using the same max_rate/max_effort values, so
// real joint motion stays bounded regardless of the inertia mismatch.
//
// WHY PID AND NOT PURE PD: a pure P(D) law cannot drive steady-state error
// to zero under a constant disturbance torque - here, gravity acting on each
// surface's own mass about its hinge axis. A larger kp only shrinks the
// resulting droop proportionally, and kp is bounded above by the dt-
// compatibility bandwidth constraint. Integral action rejects any constant
// disturbance without modelling it. Anti-windup: integral accumulation is
// frozen whenever Stage 1 has already clamped the target, whenever
// integrating this tick's error would keep the effort output saturated in
// the direction it is already saturated in (conditional integration), and
// whenever the joint is at its velocity limit (rate saturation).
//
// The PID gains are derived against each surface's inertia ABOUT ITS OWN
// HINGE AXIS (parallel-axis corrected, per surface), not the link's
// centre-of-mass inertia.
//
// Every numeric constant a caller supplies here (min/max angle, max_rate,
// max_effort, kp, kd, ki) is loaded from config/actuator_v1_config.yaml by
// ActuatorSystem.cc's Configure() step; no coefficient is hardcoded in this
// header or in ActuatorSystem.cc.
// =============================================================================
#ifndef FALCON_V2_ACTUATOR_MODEL_HH_
#define FALCON_V2_ACTUATOR_MODEL_HH_

#include <algorithm>
#include <cmath>

namespace falcon_v2_actuators
{

// ===========================================================================
// Detection threshold (NOT a physical coefficient, NOT a gain) - see
// actuator_v1_config.yaml's rate_saturation_antiwindup_note for provenance.
// A joint is treated as "at its velocity limit" when
// |actual_rate| >= kRateSaturationDetectFraction * cfg.maxRateRadS. The
// 0.999 value is the project's standard definition of "rate-pinned", so
// every consumer of that notion refers to the same event. It is a strict
// inequality margin
// against exact floating-point equality with the physics engine's own
// constraint value, nothing more.
// ===========================================================================
inline constexpr double kRateSaturationDetectFraction = 0.999;

// ===========================================================================
// Config - populated by the caller (ActuatorSystem.cc's Configure()) from
// config/actuator_v1_config.yaml. No default value below
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
  // ingested for the named Emax ES08MAII servo; see
  // actuator_v1_config.yaml for the documented order-of-magnitude reasoning.
  double maxRateRadS = 0.0;

  // Servo/actuator maximum effort (joint torque), N*m. V1_PROVISIONAL, same
  // no-datasheet caveat as maxRateRadS - see actuator_v1_config.yaml.
  double maxEffortNm = 0.0;

  // PID control-law gains used ONLY to drive the real Gazebo joint toward
  // the rate-limited setpoint (PidEffort() below) - V1_PROVISIONAL,
  // PER-SURFACE as of 2026-09-12 (F1), see actuator_v1_config.yaml's
  // "surfaces" map for the values and its "control_law" block for the
  // documented derivation: a dt=0.001s-compatible closed-loop bandwidth
  // (omega_n = 200 rad/s as of 2026-09-12 F2, was 100; zeta = 1.0)
  // evaluated against each surface's inertia ABOUT ITS HINGE AXIS,
  // I_hinge = I_com,nn + m*d_perp^2 (parallel-axis corrected as of
  // 2026-09-12 F1 - the pre-2026-09-12 derivation used the SDF
  // <inertial><inertia> value directly, which is about the link's CENTRE
  // OF MASS, not about the hinge, understating the real plant inertia by
  // 3.03-3.48x and leaving the loop under-damped at zeta = 0.57-0.77
  // instead of the intended 1.0). NOT simply sized from maxEffortNm/a
  // reference tracking-error angle in isolation; an earlier draft that did
  // exactly that was found to be numerically unstable at a 1 kHz per-tick
  // torque-update rate - see the
  // YAML "hinge_inertia_derivation_note" and "derivation_note" for the
  // full finding and its correction.
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
/// 2 combined). Callers use StepSetpoint() rather than the two stages
/// separately, so they can never diverge in how clamp and rate-limit
/// combine.
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
  // - the caller (ActuatorSystem.cc) must persist this
  // and pass it back in as prevIntegralRadS on the NEXT call for the same
  // surface, exactly like setpointRad is already persisted across
  // StepSetpoint() calls. Added 2026-08-24 (gravity-droop fix).
  double integralRadS = 0.0;

  // True if anti-windup froze integral accumulation THIS call (the Stage-1
  // target was clamped, OR integrating would have kept the effort saturated
  // in the same direction it already was, or the joint is at its
  // physics-engine VELOCITY limit and integrating would only deepen an
  // already-unreachable demand). Informational only: not published to the
  // diagnostics topic, whose 35-field layout is fixed.
  bool integralFrozen = false;

  // True if the freeze above was caused specifically by the rate-saturation
  // clause, as opposed to the other two clauses, so its effect is separately
  // identifiable without re-deriving it. Like integralFrozen, this is NOT
  // published to the 35-field diagnostics topic.
  bool rateSatFreezeActive = false;

  // True if the joint was observed at its velocity limit this call,
  // regardless of whether that caused a freeze (the freeze is
  // direction-conditional). Informational; not published.
  bool rateSaturated = false;
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
/// RATE-SATURATION ANTI-WINDUP (added 2026-09-12, ACTUATOR_CONTROL_LIMIT_
/// CYCLE_FIX change F3): the freeze condition below gained a third clause,
/// covering the physics-engine VELOCITY limit. Motivation, measured rather
/// than assumed (see actuator_v1_config.yaml's
/// rate_saturation_antiwindup_note): across both 1 kHz airborne records,
/// all 5 surfaces, both flight modes, the effort-clamp duty and therefore
/// the integral-freeze duty were BOTH 0.0 % - the pre-existing anti-windup
/// never engaged even once - while the joint sat at its SetVelocityLimits()
/// bound 44-85 % of the time with the integrator accumulating against that
/// unreachable demand. The new clause is direction-conditional in exactly
/// the same way as the effort clause, and is provably inert whenever
/// |actualRateRadS| < kRateSaturationDetectFraction * cfg.maxRateRadS, so
/// nominal unsaturated tracking is bit-for-bit unchanged.
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

  // ---- 2026-09-12 (F3): rate-saturation conditional integration ----
  // Same "conditional integration" idea as the effort branch above, applied
  // to the saturation that actually occurs in flight. The joint's velocity
  // is hard-bounded by gz::sim::Joint::SetVelocityLimits() (applied once in
  // ActuatorSystem.cc::Configure() using this same cfg.maxRateRadS), a
  // constraint OUTSIDE this control law that this law had no visibility of.
  // While the joint is pinned there it physically cannot reduce the error
  // any faster, so continuing to integrate only accumulates demand that
  // cannot be met. Direction-conditional exactly like the effort branch:
  // frozen only when the joint is already moving in the direction the error
  // asks for (sign(actualRate) == sign(error)), so the integrator is never
  // prevented from UNWINDING. Provably inert when the joint is not at its
  // velocity limit, so unsaturated tracking is bit-for-bit identical to the
  // pre-F3 computation.
  const bool rateSaturated = cfg.maxRateRadS > 0.0 &&
      std::abs(actualRateRadS) >=
          kRateSaturationDetectFraction * cfg.maxRateRadS;
  const bool rateSameSignAsError =
      (actualRateRadS >= 0.0 && error >= 0.0) ||
      (actualRateRadS <= 0.0 && error <= 0.0);
  const bool rateSatFreeze = rateSaturated && rateSameSignAsError;

  const bool freeze =
      targetClampActive || (wouldSaturate && sameSignAsError) || rateSatFreeze;

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
  out.rateSatFreezeActive = rateSatFreeze;
  out.rateSaturated = rateSaturated;
  return out;
}

}  // namespace falcon_v2_actuators

#endif  // FALCON_V2_ACTUATOR_MODEL_HH_

// =============================================================================
// FALCON V2 - Aerodynamics V1 core math model
// =============================================================================
// Owner: aerodynamics specialist agent. Task: AERODYNAMICS_V1_IMPLEMENTATION
// (2026-08-22).
//
// This header contains ONLY pure math: no gz-sim (ECM/Entity/System)
// dependency, so it can be (a) linked into the real Gazebo System plugin
// (AerodynamicsSystem.cc) and (b) linked into a small standalone,
// Gazebo-independent self-test executable (test/aero_model_selftest.cc) that
// can be compiled and run without launching a full Gazebo Sim instance. The
// only external dependency is gz-math7 (Vector3d), already a project
// dependency via gz-sim8.
//
// Every coefficient/formula below traces to:
//   - CLAUDE.md (task brief "Full derivative set", "Coefficient architecture",
//     "Force/moment computation" blocks)
//   - docs/source_of_truth/aerodynamics/AERODYNAMICS.md (full provenance,
//     CL0/Cm0 derivation, high-alpha limiter derivation - this pass)
//   - docs/source_of_truth/aerodynamics/aero_v1_config.yaml (the structured
//     numeric dataset this header's caller loads and passes in as AeroConfig)
//
// FRAME CONVENTION: Gazebo/CAD FLU body frame throughout (+X forward, +Y
// left, +Z up), per CLAUDE.md. All vectors in this header are BODY-FRAME
// unless explicitly named "*_world". alpha/beta sign derivations are
// documented in detail immediately above their functions below - both were
// re-derived from first principles for FLU (not copied from the standard FRD
// textbook forms), verified by explicit rotation-matrix construction
// (see AERODYNAMICS.md for the full derivation and numerical verification).
// =============================================================================
#ifndef FALCON_V2_AERO_MODEL_HH_
#define FALCON_V2_AERO_MODEL_HH_

#include <algorithm>
#include <cmath>

#include <gz/math/Vector3.hh>

namespace falcon_v2_aero
{

/// \brief All numeric parameters this model needs, loaded from
/// docs/source_of_truth/aerodynamics/aero_v1_config.yaml by the plugin's
/// Configure() step (see AerodynamicsSystem.cc). No default value here is
/// meant to be trusted for a real run - the loader must always overwrite
/// every field from the YAML file; defaults exist only so a missing YAML key
/// fails loudly (e.g. S=0 makes qbar*S=0, an obviously-wrong but non-NaN,
/// non-crashing zero-force output) rather than reading uninitialized memory.
struct AeroConfig
{
  // Reference geometry (AERODYNAMICS.md sec 2.2, CLAUDE.md)
  double S = 0.0;        // wing area, m^2
  double b = 0.0;        // wingspan, m
  double c_ref = 0.0;    // aerodynamic reference chord, m (NOT manufacturer avg chord)

  // Environment
  double rho = 1.225;         // air density, kg/m^3 (ISA sea level, config parameter)
  double vSafeFloor = 1.0e-3; // m/s, TEMPORARY numerical-safety-only V floor for *_hat denominators

  // Longitudinal (AERODYNAMICS.md sec 6.2, 7.1; CL0/Cm0 DERIVED this pass)
  double CLa = 0.0, Cma = 0.0, CLq = 0.0, Cmq = 0.0, Cmde = 0.0;
  double CL0 = 0.0, Cm0 = 0.0;

  // Lateral-directional (AERODYNAMICS.md sec 6.2, 7.2, 7.3)
  double CYb = 0.0, CYp = 0.0, CYr = 0.0, CYda = 0.0, CYdr = 0.0;
  double Clb = 0.0, Clp = 0.0, Clr = 0.0, Clda = 0.0, Cldr = 0.0;
  double Cnb = 0.0, Cnp = 0.0, Cnr = 0.0, Cnda = 0.0, Cndr = 0.0;

  // Drag polar (AERODYNAMICS.md sec 6.5, V1_CALIBRATED)
  double CD0 = 0.0, dragK = 0.0;

  // High-alpha smooth-saturation limiter (V1_SMOOTH_SATURATION, this pass)
  double CLmax = 1.42;           // manufacturer performance-calc input, MD sec 3
  double alphaTransition = 0.0;  // rad, DERIVED = midpoint of 9-9.5 deg XFLR5 reliability band

  // Control-joint-to-deflection-sign mapping. Signs VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST,
  // task CONTROL_SURFACE_SIGN_MAPPING, 2026-08-22 (see AERODYNAMICS.md 19.13).
  // Literal defaults below are pre-load placeholders only, overwritten from
  // aero_v1_config.yaml's control_mapping block (elevator_sign = -1.0,
  // aileron_sign/rudder_sign = +1.0) at config-load time.
  double elevatorSign = 1.0, aileronSign = 1.0, rudderSign = 1.0;
  double controlDeflectionClamp = 0.0; // rad, V1_CONSERVATIVE_CLAMP (~10 deg)

  // ---- Derived-once saturation constants (computed by Prepare(), not read
  // from YAML directly - keeping a single source of truth for the formula in
  // this header rather than duplicating the arithmetic in the config file).
  double satHeadroomPos = 0.0, satKPos = 0.0;
  double satAneg = 0.0, satKNeg = 0.0;
  bool prepared = false;

  /// \brief Must be called once after every field above (except the
  /// satXxx/prepared bookkeeping fields) is populated from YAML. Computes
  /// the C1-continuous high-alpha saturation constants documented in
  /// AERODYNAMICS.md. Safe to call multiple times (idempotent).
  void Prepare()
  {
    const double clLinAtT = CL0 + CLa * alphaTransition;
    satHeadroomPos = CLmax - clLinAtT;
    satKPos = (satHeadroomPos > 1e-9) ? (CLa / satHeadroomPos) : 0.0;

    const double clLinAtNegT = CL0 + CLa * (-alphaTransition);
    satAneg = clLinAtNegT + CLmax;
    satKNeg = (satAneg > 1e-9) ? (CLa / satAneg) : 0.0;

    prepared = true;
  }
};

/// \brief Per-timestep aerodynamic state input, already resolved into body
/// frame and already run through the control-sign-mapping/clamp layer
/// (AerodynamicsSystem.cc does the joint-position-to-delta_x conversion
/// before calling ComputeAero(); this header does not know about joints).
struct AeroState
{
  double u = 0.0, v = 0.0, w = 0.0;       // body-frame relative wind velocity, m/s (Vrel = Vbody - Vwind)
  double p = 0.0, q = 0.0, r = 0.0;       // body-frame angular velocity, rad/s (roll/pitch/yaw rate about X/Y/Z)
  double deltaA = 0.0, deltaE = 0.0, deltaR = 0.0; // rad, sign-mapped and clamped
};

/// \brief Full per-timestep diagnostic + force/moment output.
struct AeroOutput
{
  double V = 0.0, alpha = 0.0, beta = 0.0, qbar = 0.0;
  double CL = 0.0, CD = 0.0, CY = 0.0, Cl = 0.0, Cm = 0.0, Cn = 0.0;
  gz::math::Vector3d forceBody = gz::math::Vector3d::Zero;
  gz::math::Vector3d momentBody = gz::math::Vector3d::Zero;
};

// -----------------------------------------------------------------------
// Axis-rotation physical meanings used throughout this file (re-derived
// this pass by explicit rotation-matrix construction, NOT copied from FRD
// textbook shorthand - every claim below was checked with an actual 90 deg
// / 5 deg numeric rotation, not verbal intuition; see AERODYNAMICS.md for
// the full derivation log). Using the standard right-handed active-rotation
// matrices Rx/Ry/Rz (identical algebraic form in any frame; only the
// PHYSICAL labels of the axes - forward/left/up vs forward/right/down -
// differ between FLU and FRD):
//   +X (roll)  rotation -> LEFT wingtip moves UP. This is the SAME physical
//     roll sense as FRD's textbook "positive roll = right wing down" (both
//     Y and Z flip sign of meaning between FRD/FLU, an EVEN number of
//     flips, so the roll-axis handedness is unchanged between the two
//     frames - only the wing used to describe it changes).
//   +Y (pitch) rotation -> NOSE DOWN. This is the OPPOSITE of the
//     traditional aerospace "q>0 / positive Cm = nose up" shorthand, which
//     IS consistent with strict right-hand-rule rotation in FRD (Z=down)
//     but flips in FLU (Z=up) because only Z (not X) changed meaning for
//     this axis pair - an ODD number of flips.
//   +Z (yaw)   rotation -> NOSE LEFT. Also flipped vs FRD's "r>0=nose
//     right" (only Y changed meaning for this axis pair - also an odd
//     number of flips).
// These are used below to interpret every Mx/My/Mz sign claim in this file
// and in AERODYNAMICS.md's sign-behavior table.
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// Angle of attack (alpha) - FLU derivation
// -----------------------------------------------------------------------
// The textbook FRD form is alpha = atan2(w, u) (X-forward, Z-DOWN). Since
// pitch (+Y) rotation-sense flips between FRD and FLU (see above), a naive
// copy risks the wrong sign. Re-derived directly: a nose-up disturbance by
// physical angle theta corresponds (per the "+Y rotation -> nose down"
// finding above) to a rotation of -theta about +Y. With the flight path
// unchanged in world coordinates (level, Vrel_world=(V,0,0)), the resulting
// body-frame relative wind is u=V*cos(theta), w=-V*sin(theta) for
// nose-up theta>0 (verified numerically for both signs of theta). Since
// theta IS the angle of attack by definition of this scenario, this gives
// alpha = atan2(-w, u) - note the explicit MINUS sign on w, the FLU-vs-FRD
// flip the task brief calls out, re-derived (not copied) and confirmed by
// direct rotation-matrix computation, not verbal analogy.
// Physical meaning: alpha > 0 <=> nose pitched up relative to the relative
// wind (standard "positive alpha increases effective wing incidence" sense)
// - this part of the derivation has NO free convention choice (unlike beta
// below): "nose up" has one unambiguous physical meaning.
// SIGN NOT YET CONFIRMED AGAINST XFLR5's OWN INTERNAL CONVENTION - this is
// exactly what AOA_SIGN_TEST (gazebo-testing) must verify before this sign
// is treated as final; see AERODYNAMICS.md sec 10/13.
inline double AngleOfAttack(double u, double /*v*/, double w)
{
  return std::atan2(-w, u);
}

// -----------------------------------------------------------------------
// Sideslip (beta) - FLU derivation
// -----------------------------------------------------------------------
// Unlike alpha, beta's sign is a genuine CONVENTION CHOICE (both "positive
// = wind from the right" and "positive = wind from the left" are used in
// different texts; there is no unique physical necessity the way there is
// for "nose up"). Two candidate formulas were derived and checked against
// the GIVEN (unmodified) CYb/Cnb signs and their AERODYNAMICS.md sec 6.2
// stated physical meanings, using a nose-yawed-left-by-psi disturbance
// (Vrel unchanged in world coords -> body-frame v = -V*sin(psi) for psi>0,
// which is exactly the "relative wind from the right" case):
//   Candidate A: beta = atan2(-v, hypot(u,w))  [preserves the FRD textbook
//     "positive beta = wind from right" literal physical meaning]
//   Candidate B: beta = atan2(+v, hypot(u,w))  [same formula structure as
//     the naive/un-adjusted FRD copy; physical meaning becomes "positive
//     beta = wind from the LEFT" because the yaw axis (+Z) flips sense
//     between FRD/FLU - see the axis-rotation note above]
// Checked against the GIVEN Cnb=+0.03554 (stated in AERODYNAMICS.md sec 6.2
// as "directionally statically stable") and CYb=-0.13216, using the
// confirmed "+Z rotation -> nose LEFT" finding above: for the nose-slipped-
// left/wind-from-right disturbance, Candidate A produces Cnb*beta > 0 ->
// +Mz -> NOSE LEFT (DESTABILIZING, amplifies the disturbance); Candidate B
// produces Cnb*beta < 0 -> -Mz -> NOSE RIGHT (RESTORING, matches the stated
// physical meaning). CYb under Candidate B also gives a sideforce pushing
// the aircraft AWAY from a right-side crosswind (physically correct, basic
// pressure-force direction). Candidate B is therefore ADOPTED:
//   beta = atan2(v, hypot(u,w))    [physical meaning: positive beta = wind
//                                    from the aircraft's LEFT side]
// This is a documented INFERENCE from matching two independent, reliable
// physical checks (directional weathercock stability + crosswind push
// direction) against the fixed/unmodified Cnb and CYb signs - it is NOT a
// coefficient adjustment (Cnb/CYb are never touched) and NOT a claim of
// final confirmation. A third possible check (Clb "dihedral effect") was
// attempted but NOT used as evidence either way - the physical direction of
// the dihedral-sideslip roll coupling was not re-derivable here with
// confidence from first principles (roll-axis handedness does not flip
// between FRD/FLU per the note above, but the fluid-dynamic dihedral
// mechanism itself is subtle enough that this document does not assert a
// verified direction for it). SIDESLIP_SIGN_TEST and
// Cnb_STATIC_STABILITY_SIGN_TEST (gazebo-testing, live Gazebo) are still
// REQUIRED before this convention is treated as final - see AERODYNAMICS.md
// for the full derivation and self-test log.
inline double Sideslip(double u, double v, double w)
{
  return std::atan2(v, std::hypot(u, w));
}

// -----------------------------------------------------------------------
// Wind-axis -> body-axis force rotation - FLU derivation (not the naive
// Fx=-D, Fz=L shortcut)
// -----------------------------------------------------------------------
// Built from the SAME rotation R(alpha,beta) = Ry(alpha) * Rz(beta) implied
// by the alpha/beta formulas above (guarantees the force rotation is
// exactly self-consistent with the angle formulas actually used - not a
// separately-copied textbook formula that could silently disagree in sign
// convention; verified by reconstructing R(alpha,beta) @ (1,0,0) = Vrel/V to
// ~1e-16 over thousands of random trials, and confirming R(0,0)=identity,
// i.e. wind axes = body axes at zero alpha/beta).
// Force in wind axes: (-D, Y, L) along (Xw, Yw, Zw); Xw is defined along the
// relative-wind unit vector (drag opposes the relative wind exactly, by
// definition, independent of any rotation-matrix convention - a useful
// cross-check), Zw reduces to body +Z (up) and Yw to body +Y (left) at
// alpha=beta=0.
// Closed form (expand F_body = R(alpha,beta) @ (-D, Y, L)^T with
// R = Ry(alpha)*Rz(beta), beta per the Candidate-B convention above):
//   Fx = -D*cos(a)*cos(b) - Y*cos(a)*sin(b) + L*sin(a)
//   Fy = -D*sin(b) + Y*cos(b)
//   Fz =  D*sin(a)*cos(b) + Y*sin(a)*sin(b) + L*cos(a)
// Verified: at alpha=beta=0, reduces to (Fx,Fy,Fz)=(-D,Y,L) exactly, matching
// the naive case as the correct zero-angle special case, not a coincidence -
// the general form correctly rotates as alpha/beta move away from zero
// (checked for positive AND negative alpha; see AERODYNAMICS.md).
inline gz::math::Vector3d WindToBodyForce(
    double lift, double drag, double sideForce, double alpha, double beta)
{
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  const double cb = std::cos(beta), sb = std::sin(beta);

  const double fx = -drag * ca * cb - sideForce * ca * sb + lift * sa;
  const double fy = -drag * sb + sideForce * cb;
  const double fz = drag * sa * cb + sideForce * sa * sb + lift * ca;

  return gz::math::Vector3d(fx, fy, fz);
}

// -----------------------------------------------------------------------
// RESOLVED FINDING (was flagged as an unpatched risk in an earlier pass;
// fixed this pass after independent confirmation by gazebo-testing's live
// measurement and validation's root-cause review) - Cma/My pitch-axis sign
// bug, PRECISELY SCOPED fix
// -----------------------------------------------------------------------
// Unlike the rate/damping terms (Cmq*q_hat, Clp*p_hat, Cnr*r_hat), which are
// mathematically SELF-referential (a negative coefficient times a rate
// always produces a moment opposing that SAME rate, about the SAME axis -
// true in any frame/handedness, no convention risk - proved algebraically:
// Mx/My/Mz = [qbar*S*(b or c_ref)*Cxq/(2*vSafe)] * (p, q, or r), i.e. always
// a NEGATIVE-constant multiple of the SAME rate when Cxq<0, hence always
// opposing it, regardless of any axis-handedness convention), the STATIC
// terms (Cm0, Cma*alpha, Cmde*deltaE) are NOT self-referential: each
// relates an independently-defined ANGLE (alpha, or a control deflection -
// not a rate) to a moment about +Y. Given the confirmed "+Y rotation ->
// NOSE DOWN" finding above (opposite of the traditional "positive Cm/My =
// nose up" aerospace shorthand, which is only consistent with strict
// right-hand-rule rotation in FRD, not FLU), a LITERAL, unmodified
// application of My = qbar*S*c_ref*Cm to the STATIC portion of Cm produces,
// for a nose-up disturbance (alpha>0, Cma<0): a moment that is
// DESTABILIZING (nose-up-reinforcing), the OPPOSITE of the textbook-
// expected restoring (nose-down) behavior.
//
// This was originally reported (not silently patched) pending live
// confirmation. gazebo-testing then independently measured this exact
// behavior live (My=-2.83 N*m at alpha=+8deg, reinforcing not restoring),
// and validation performed an independent root-cause re-derivation
// (rotation-matrix construction on both +X and +Z axes, confirming: (a)
// roll/+X handedness does NOT flip between FRD/FLU, so Cl/Mx needs no
// correction; (b) yaw/+Z handedness DOES flip, exactly like pitch/+Y, but
// is already absorbed by the Sideslip() convention CHOICE above - beta had
// a genuine free sign convention to select, alpha did not, so yaw got a
// "free" fix where pitch could not; (c) the RATE term Cmq*qHat is affected
// by the SAME FLU mirroring on BOTH q (read raw from the FLU ECM) and My's
// output - a double-cancellation, not a coincidence, matching the already-
// passing Cmq_DAMPING_SIGN_TEST exactly). validation's conclusion, applied
// here: negate ONLY the static group (Cm0 + Cma*alpha + Cmde*deltaE) when
// mapping onto the FLU +Y torque axis; the rate group (Cmq*qHat) is
// already correct and must NOT be flipped. Cmde is included in the static
// group deliberately - it is a geometric angle input (control deflection),
// not a rate, so it has the same axis-handedness exposure as Cma*alpha,
// independently of the SEPARATE, still-open ELEVATOR_SIGN_TEST question of
// whether a positive Gazebo elevator joint command physically means
// trailing-edge-down. (Consequence, flagged for re-review: this means
// ELEVATOR_PITCH_SIGN_TEST's measured My sign will FLIP relative to any
// earlier, pre-fix measurement - expected, not a regression.) Cm0 and Cma
// are flipped together as a single group (never independently) because
// Cm0 was specifically derived so that Cm0+Cma*alpha_trim=0 at the neutral
// trim point (AERODYNAMICS.md sec 19.3) - flipping only one would break
// that zero-moment trim condition. out.Cm itself is still reported in
// XFLR5's own (unflipped) convention (for diagnostics/comparability against
// the source-of-truth sweep tables) - only the MOMENT computation (my,
// below) applies the correction. See AERODYNAMICS.md sec 19.7/19.12 for
// the full record of this finding and its resolution, and the self-test
// executable for the updated Cma_RESTORING_SIGN_TEST result.
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// High-alpha smooth saturation (V1_SMOOTH_SATURATION)
// -----------------------------------------------------------------------
// Below |alpha| <= alphaTransition (DERIVED = midpoint of the XFLR5
// attached-flow reliability band, 9-9.5 deg): CL is exactly the linear
// model (CL0 + CLa*alpha), reproducing the validated region exactly.
// Beyond that, CL asymptotically (never exceeding) approaches +/-CLmax via
// a C1-continuous (matching value AND slope at the transition point)
// exponential, so there is no kink and no unexplained free parameter - the
// decay rate k is itself computed from CLa/CL0/CLmax/alphaTransition, not a
// separately-chosen constant. See AeroConfig::Prepare() for the constants
// and AERODYNAMICS.md for the full derivation/numeric table.
// Positive side traces to real data (CLmax=1.42 manufacturer value, 9-9.5
// deg XFLR5 reliability limit). Negative side is ASSUMPTION-tagged (see
// aero_v1_config.yaml high_alpha_limiter.negative_side_note) - no
// full-aircraft negative-alpha stall data exists in the source of truth;
// the symmetric bound exists purely to prevent unbounded negative
// lift/drag growth (the stated V1 requirement), not as validated physics.
inline double SaturatedCL(const AeroConfig &cfg, double alpha)
{
  const double clLinear = cfg.CL0 + cfg.CLa * alpha;
  if (alpha > cfg.alphaTransition)
  {
    return cfg.CLmax -
           cfg.satHeadroomPos * std::exp(-cfg.satKPos * (alpha - cfg.alphaTransition));
  }
  else if (alpha < -cfg.alphaTransition)
  {
    return -cfg.CLmax +
           cfg.satAneg * std::exp(cfg.satKNeg * (alpha + cfg.alphaTransition));
  }
  return clLinear;
}

/// \brief Full V1 aerodynamic model evaluation for one timestep. Pure
/// function: no side effects, no Gazebo dependency. See file header for the
/// full architecture/provenance notes.
inline AeroOutput ComputeAero(const AeroConfig &cfg, const AeroState &st)
{
  AeroOutput out;

  const double u = st.u, v = st.v, w = st.w;
  const double vSq = u * u + v * v + w * w;
  out.V = std::sqrt(vSq);
  out.qbar = 0.5 * cfg.rho * vSq; // exactly zero at true V=0: no protection needed here

  out.alpha = AngleOfAttack(u, v, w);
  out.beta = Sideslip(u, v, w);

  // V floor ONLY for the *_hat rate-normalization denominators (division by
  // 2V) - qbar above already uses the true (unclamped) V, so qbar*(*_hat)
  // still -> 0 correctly as V -> 0 even though *_hat itself is a large-but-
  // finite number at that instant (never Inf/NaN). See ZERO_AIRSPEED_AERO_TEST.
  const double vSafe = std::max(out.V, cfg.vSafeFloor);
  const double pHat = st.p * cfg.b / (2.0 * vSafe);
  const double qHat = st.q * cfg.c_ref / (2.0 * vSafe);
  const double rHat = st.r * cfg.b / (2.0 * vSafe);

  // Control deflections: clamp to the aero-validated +/-10 deg band
  // (V1_CONSERVATIVE_CLAMP) before evaluating the linear derivatives -
  // "no silent extrapolation" (CLAUDE.md). Sign mapping (elevatorSign etc.)
  // and joint-position combination happen upstream in AerodynamicsSystem.cc;
  // st.deltaA/E/R here are already sign-mapped, only the magnitude clamp
  // happens here.
  auto clampDelta = [&cfg](double d)
  {
    return std::clamp(d, -cfg.controlDeflectionClamp, cfg.controlDeflectionClamp);
  };
  const double deltaA = clampDelta(st.deltaA);
  const double deltaE = clampDelta(st.deltaE);
  const double deltaR = clampDelta(st.deltaR);

  // Coefficient build-up - EXACT form per CLAUDE.md, no added/removed terms.
  out.CY = cfg.CYb * out.beta + cfg.CYp * pHat + cfg.CYr * rHat +
           cfg.CYda * deltaA + cfg.CYdr * deltaR;
  out.Cl = cfg.Clb * out.beta + cfg.Clp * pHat + cfg.Clr * rHat +
           cfg.Clda * deltaA + cfg.Cldr * deltaR;
  out.Cn = cfg.Cnb * out.beta + cfg.Cnp * pHat + cfg.Cnr * rHat +
           cfg.Cnda * deltaA + cfg.Cndr * deltaR;

  // Cm split into a STATIC/angle-derived group (Cm0, Cma*alpha, Cmde*deltaE
  // - each relates an independently-defined ANGLE to a moment, exactly like
  // the Cma*alpha term analyzed in the "RESOLVED FINDING" comment above)
  // and a RATE group (Cmq*qHat - self-referential: always opposes its own
  // rate, about the same axis, regardless of frame handedness - proved
  // algebraically in the "RESOLVED FINDING" comment above and confirmed by
  // the passing Cmq_DAMPING_SIGN_TEST). Only the STATIC group needs the
  // FLU pitch-axis sign correction when mapped onto My below; the RATE
  // group must NOT be flipped (independently confirmed by `validation`'s
  // review - flipping it would break the already-correct damping result).
  // out.Cm itself is reported in XFLR5's OWN (unflipped) convention, for
  // diagnostics/comparability against the source-of-truth sweep tables -
  // only the MOMENT computation (my, below) applies the correction.
  const double cmStatic = cfg.Cm0 + cfg.Cma * out.alpha + cfg.Cmde * deltaE;
  const double cmRate = cfg.Cmq * qHat;
  out.Cm = cmStatic + cmRate;

  // CL: high-alpha smooth saturation applied to the alpha-driven STATIC
  // term only (the source data's 9-9.5deg/CLmax reliability boundary
  // characterizes alpha, not q_hat - AERODYNAMICS.md sec 19.5). CLq*qHat
  // (RATE term) is added on top, unsaturated - mirroring the same
  // static/rate split adopted for Cm above. Previously CLq was loaded into
  // AeroConfig but never referenced here (MAJOR finding, fixed this pass -
  // see AERODYNAMICS.md sec 19.4/19.12).
  out.CL = SaturatedCL(cfg, out.alpha) + cfg.CLq * qHat;

  // CD: V1_CALIBRATED full-aircraft polar, fed the full (static+rate) CL
  // so induced drag reflects the complete lift coefficient, still bounded
  // at extreme alpha since the static/dominant term remains saturated.
  out.CD = cfg.CD0 + cfg.dragK * out.CL * out.CL;

  const double lift = out.qbar * cfg.S * out.CL;
  const double drag = out.qbar * cfg.S * out.CD;
  const double sideForce = out.qbar * cfg.S * out.CY;

  out.forceBody = WindToBodyForce(lift, drag, sideForce, out.alpha, out.beta);

  // Moments: Mx (roll) and Mz (yaw) use the given Cl/Cn directly, no axis-
  // handedness correction - roll-axis handedness does not flip between
  // FRD/FLU (see the axis-rotation table earlier in this file), and yaw-
  // axis handedness, while it DOES flip, is already absorbed by the
  // Sideslip() convention choice (Candidate B was selected specifically
  // because it makes the given Cnb/CYb signs behave correctly through the
  // UNMODIFIED Mz=qbar*S*b*Cn formula - see the Sideslip() derivation
  // comment and AERODYNAMICS.md sec 19.6/19.7). This was independently
  // re-derived and confirmed by `validation`'s review (rotation-matrix
  // construction on both +X and +Z axes) - no change needed here.
  // My (pitch) applies the resolved sign correction from above: only
  // cmStatic is negated relative to XFLR5's own convention; cmRate is not.
  const double mx = out.qbar * cfg.S * cfg.b * out.Cl;
  const double my = out.qbar * cfg.S * cfg.c_ref * (-cmStatic + cmRate);
  const double mz = out.qbar * cfg.S * cfg.b * out.Cn;
  out.momentBody = gz::math::Vector3d(mx, my, mz);

  return out;
}

}  // namespace falcon_v2_aero

#endif  // FALCON_V2_AERO_MODEL_HH_

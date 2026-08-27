// =============================================================================
// FALCON V2 - Aerodynamics V1 core math model
// =============================================================================
// Owner: aerodynamics specialist agent. Task: AERODYNAMICS_V1_IMPLEMENTATION
// (2026-08-22).
//
// UPDATED (2026-08-26, task HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION):
// control-surface aerodynamic effect (elevator/aileron/rudder) is now a
// bounded piecewise-linear WIDE-DEFLECTION LOOKUP TABLE (source: XFLR5
// Type-1 fixed-condition wide-deflection sweep, docs/source_of_truth/
// aerodynamics/control_surface_analysis/
// FALCON_V2_CONTROL_SURFACE_WIDE_DEFLECTION_RESULTS.txt), REPLACING the old
// linear-coefficient + generic +/-10 deg clamp model entirely. See the
// "Piecewise-linear control-surface lookup" section below and
// docs/source_of_truth/aerodynamics/AERODYNAMICS.md (dated section, this
// pass) for the full architecture/provenance/1A-1B-resolution record. Not
// touched this pass: CL0/Cm0/CD0/dragK, the high-alpha limiter, the
// actuator/servo model, and alpha/beta/force-rotation/Cm-axis-sign logic
// below (all pre-existing, still valid).
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
//     CL0/Cm0 derivation, high-alpha limiter derivation, wide-deflection
//     lookup architecture - this pass)
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
#include <array>
#include <cmath>

#include <gz/math/Vector3.hh>

namespace falcon_v2_aero
{

/// \brief Number of breakpoints in the shared wide-deflection control-
/// surface lookup-table domain (HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION,
/// 2026-08-26). Fixed by the XFLR5 wide-deflection source data's own sweep
/// grid (FALCON_V2_CONTROL_SURFACE_WIDE_DEFLECTION_RESULTS.txt) - not a
/// tunable parameter. The same 15-point grid (deg) is used for all three
/// control surfaces:
///   [-45,-35,-25,-15,-10,-5,-2,0,+2,+5,+10,+15,+25,+35,+45]
constexpr int kNumCtrlBreakpoints = 15;

/// \brief Index of the delta=0 breakpoint within the 15-point grid above.
/// Used in AeroConfig::Prepare() to baseline-difference the aileron/rudder
/// FULL-VALUE CD/CL/Cm rows (those rows are raw fixed-alpha sweep values,
/// not already baseline-differenced in the source file, unlike the elevator
/// table - see the field comments below).
constexpr int kCtrlZeroIndex = 7;

/// \brief Fixed-size array type for one wide-deflection lookup curve.
using CtrlLookupArray = std::array<double, kNumCtrlBreakpoints>;

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

  // Longitudinal (AERODYNAMICS.md sec 6.2, 7.1; CL0/Cm0 DERIVED this pass).
  // NOTE (HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION, 2026-08-26): Cmde is
  // now SUPERSEDED_BY_LOOKUP - the elevator pitching-moment control effect
  // is applied EXCLUSIVELY via ctrlElevDCm below (full-lookup-replaces-
  // static-term architecture, see ComputeAero()). Cmde is retained only as
  // the documented small-signal reference constant (updated this pass from
  // -0.73 to -1.000/rad per the new fixed-condition XFLR5 data), verified
  // by the self-test to be closely reproduced by a finite difference of the
  // lookup table near delta_e=0 - it is NOT read by ComputeAero() itself.
  // CLde is a NEW field this pass, same status (reference-only; the
  // CL_delta_e effect is applied exclusively via ctrlElevDCL, never as
  // CLde*deltaE - that would double-count).
  double CLa = 0.0, Cma = 0.0, CLq = 0.0, Cmq = 0.0, Cmde = 0.0, CLde = 0.0;
  double CL0 = 0.0, Cm0 = 0.0;

  // Lateral-directional (AERODYNAMICS.md sec 6.2, 7.2, 7.3).
  // NOTE (HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION, 2026-08-26): CYda/
  // CYdr/Clda/Cnda/Cndr are now SUPERSEDED_BY_LOOKUP - the aileron/rudder
  // CY/Cl/Cn control effect is applied EXCLUSIVELY via the ctrlAile*/
  // ctrlRudd* wide-deflection tables below (full-lookup-replaces-static-
  // term architecture). These scalars are retained only as documented
  // small-signal reference constants (updated this pass per the new
  // fixed-condition XFLR5 data - see aero_v1_config.yaml for the full
  // per-value provenance and the 1A/CY_delta_a resolution record), verified
  // by the self-test's derivative-recovery check, NOT read by
  // ComputeAero() itself.
  // Cldr is the ONE exception: task 1B (Cl_delta_r sign conflict between
  // the two XFLR5 sessions) was resolved UNRESOLVED_KEEP_CURRENT (no
  // decisive geometric/methodological reason found to prefer either sign -
  // see AERODYNAMICS.md and aero_v1_config.yaml for the full record), so
  // Cldr KEEPS its old value (+0.0007/rad) and IS still functionally used:
  // AeroConfig::Prepare() builds ctrlRuddCl as a bounded LINEAR EXTENSION
  // of this exact constant (ctrlRuddCl[i] = Cldr * ctrlBreakpointsRad[i]),
  // NOT from the new (disputed-sign) wide-deflection Cl(delta_r) table.
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
  // aileron_sign/rudder_sign = +1.0) at config-load time. NOTE
  // (2026-08-26): the old generic `controlDeflectionClamp` (+/-10 deg,
  // V1_CONSERVATIVE_CLAMP) field that used to live here has been RETIRED
  // for control-surface use - see the wide-deflection lookup block below,
  // whose own domain bound (+/-45 deg, InterpLinear()) is the new "no
  // silent extrapolation" boundary. This is NOT the same thing as, and does
  // not touch, the separate high-alpha limiter above (an angle-of-attack
  // concept, unrelated to control-surface deflection).
  double elevatorSign = 1.0, aileronSign = 1.0, rudderSign = 1.0;

  // ---------------------------------------------------------------------
  // Wide-deflection control-surface lookup tables
  // (HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION, 2026-08-26). Source:
  // docs/source_of_truth/aerodynamics/control_surface_analysis/
  // FALCON_V2_CONTROL_SURFACE_WIDE_DEFLECTION_RESULTS.txt (XFLR5 Type 1
  // fixed-speed, V=18.162 m/s, alpha=2.472 deg, beta=0, viscous OFF; only
  // the tested surface deflected, others held at 0). Loaded from
  // aero_v1_config.yaml's `control_surface_lookup` block.
  //
  // Breakpoints are shared by all 3 surfaces, stored here in RADIANS
  // (converted from the YAML's breakpoints_deg at load time):
  //   [-45,-35,-25,-15,-10,-5,-2,0,+2,+5,+10,+15,+25,+35,+45] deg
  //
  // Domain is BOUNDED at +/-45 deg (the actuator's own mechanical range,
  // docs/source_of_truth/controls/actuator_v1_config.yaml min/max_angle_rad
  // = +/-0.7853981634 rad) - InterpLinear() clamps any input outside
  // [breakpoints.front(), breakpoints.back()] to the nearest edge value
  // ("no silent extrapolation", CLAUDE.md) rather than extrapolating past
  // validated/reference data.
  //
  // Confidence labeling (DOCS-ONLY metadata - does not change any runtime
  // computation below; recorded in aero_v1_config.yaml/AERODYNAMICS.md):
  //   |delta| <= 10 deg  -> HIGH_CONFIDENCE_SMALL_SIGNAL
  //   10 < |delta| <= 25 -> MEDIUM_CONFIDENCE_NONLINEAR_REFERENCE
  //   |delta| > 25       -> LOW_CONFIDENCE_HIGH_DEFLECTION_REFERENCE
  // (the +/-35/+/-45 deg XFLR5/VLM points are explicitly NOT
  // "REAL_FLIGHT_VALIDATED".)
  // ---------------------------------------------------------------------
  CtrlLookupArray ctrlBreakpointsRad{};

  // Elevator: values are ALREADY baseline-differenced in the source file
  // itself (the delta_e=0 row is exactly 0 by construction) - "full lookup
  // contribution" architecture:
  //   CL = SaturatedCL(alpha) + CLq*qHat + ctrlElevDCL(delta_e)
  //   cmStatic = Cm0 + Cma*alpha + ctrlElevDCm(delta_e)
  // (ctrlElevDCm REPLACES the old `+ Cmde*deltaE` term exactly - never add
  // both, that would double-count). ctrlElevDCD feeds the Part-4 drag
  // build-up below.
  CtrlLookupArray ctrlElevDCL{}, ctrlElevDCD{}, ctrlElevDCm{};

  // Aileron: Cl/Cn/CY are RAW values from the source sweep (the baseline at
  // delta_a=0 is already ~0 in the source data - CY0=-0.00002, Cn0=+0.00001,
  // Cl0=0.00000 - a tiny numerical-solver residual, NOT zeroed out here;
  // used exactly as given per the project's no-fabrication rule). These
  // REPLACE the old Clda*deltaA/Cnda*deltaA/CYda*deltaA static terms
  // exactly (full-lookup-replaces-static-term, same pattern as elevator).
  // ctrlAileCDFull/CLFull/CmFull are FULL VALUES at fixed-alpha (a delta_a
  // sweep, not already baseline-differenced in the source file) -
  // AeroConfig::Prepare() differences them against the delta_a=0 row
  // (kCtrlZeroIndex) to produce ctrlAileDCD/DCL/DCm, the even/symmetric
  // secondary corrections (CD feeds Part-4 drag; CL/Cm are small optional
  // corrections included per the task brief, since the data gives them
  // cleanly at no extra cost).
  CtrlLookupArray ctrlAileCl{}, ctrlAileCn{}, ctrlAileCY{};
  CtrlLookupArray ctrlAileCDFull{}, ctrlAileCLFull{}, ctrlAileCmFull{};
  CtrlLookupArray ctrlAileDCD{}, ctrlAileDCL{}, ctrlAileDCm{};  // DERIVED in Prepare()

  // Rudder: CY/Cn are RAW values (baseline at delta_r=0 already ~0, same
  // residual-noise note as aileron above) - REPLACE the old CYdr*deltaR/
  // Cndr*deltaR static terms exactly.
  //
  // Cl_delta_r is the ONE surface/coefficient where task 1B's
  // UNRESOLVED_KEEP_CURRENT resolution applies (see the Cldr field comment
  // above and AERODYNAMICS.md/aero_v1_config.yaml for the full record): the
  // new wide-deflection Cl(delta_r) table is NOT loaded into this struct at
  // all (its raw values are recorded in aero_v1_config.yaml purely for
  // provenance/traceability, explicitly marked NOT_LOADED). Instead,
  // ctrlRuddCl is DERIVED in Prepare() as a bounded LINEAR EXTENSION of the
  // OLD Cldr small-signal constant - see Prepare() below.
  //
  // ctrlRuddCDFull is a FULL VALUE table, differenced into ctrlRuddDCD in
  // Prepare() exactly like the aileron CD table above.
  CtrlLookupArray ctrlRuddCY{}, ctrlRuddCn{};
  CtrlLookupArray ctrlRuddCl{};      // DERIVED in Prepare() from Cldr - see note above, NOT loaded from YAML
  CtrlLookupArray ctrlRuddCDFull{};
  CtrlLookupArray ctrlRuddDCD{};     // DERIVED in Prepare()

  // ---- Derived-once saturation constants (computed by Prepare(), not read
  // from YAML directly - keeping a single source of truth for the formula in
  // this header rather than duplicating the arithmetic in the config file).
  double satHeadroomPos = 0.0, satKPos = 0.0;
  double satAneg = 0.0, satKNeg = 0.0;
  bool prepared = false;

  /// \brief Must be called once after every field above (except the
  /// satXxx/ctrlAileD*/ctrlRuddD*/ctrlRuddCl/prepared bookkeeping/derived
  /// fields) is populated from YAML. Computes the C1-continuous high-alpha
  /// saturation constants documented in AERODYNAMICS.md, AND (this pass)
  /// the baseline-differenced aileron/rudder CD/CL/Cm wide-deflection
  /// tables and the Cldr-derived rudder-roll lookup. Safe to call multiple
  /// times (idempotent).
  void Prepare()
  {
    const double clLinAtT = CL0 + CLa * alphaTransition;
    satHeadroomPos = CLmax - clLinAtT;
    satKPos = (satHeadroomPos > 1e-9) ? (CLa / satHeadroomPos) : 0.0;

    const double clLinAtNegT = CL0 + CLa * (-alphaTransition);
    satAneg = clLinAtNegT + CLmax;
    satKNeg = (satAneg > 1e-9) ? (CLa / satAneg) : 0.0;

    // ---- Wide-deflection lookup derived tables (HIGH_DEFLECTION_CONTROL_
    // AERO_IMPLEMENTATION, 2026-08-26) ----
    for (int i = 0; i < kNumCtrlBreakpoints; ++i)
    {
      // Aileron/rudder FULL-VALUE rows -> baseline-differenced secondary
      // corrections (delta_a=0 / delta_r=0 row is kCtrlZeroIndex).
      ctrlAileDCD[i] = ctrlAileCDFull[i] - ctrlAileCDFull[kCtrlZeroIndex];
      ctrlAileDCL[i] = ctrlAileCLFull[i] - ctrlAileCLFull[kCtrlZeroIndex];
      ctrlAileDCm[i] = ctrlAileCmFull[i] - ctrlAileCmFull[kCtrlZeroIndex];
      ctrlRuddDCD[i] = ctrlRuddCDFull[i] - ctrlRuddCDFull[kCtrlZeroIndex];

      // 1B UNRESOLVED_KEEP_CURRENT: ctrlRuddCl is a bounded LINEAR
      // EXTENSION of the OLD Cldr small-signal constant across the full
      // +/-45 deg breakpoint grid - NOT the new wide-deflection table's
      // disputed-sign nonlinear shape. See the Cldr/ctrlRuddCl field
      // comments above for the full resolution record.
      ctrlRuddCl[i] = Cldr * ctrlBreakpointsRad[i];
    }

    prepared = true;
  }
};

/// \brief Per-timestep aerodynamic state input, already resolved into body
/// frame and already run through the control-sign-mapping layer
/// (AerodynamicsSystem.cc does the joint-position-to-delta_x conversion
/// before calling ComputeAero(); this header does not know about joints).
/// deltaA/deltaE/deltaR are sign-mapped but NOT pre-clamped here - the
/// wide-deflection lookup's own +/-45 deg domain bound (InterpLinear())
/// is now the single place bounding these inputs (HIGH_DEFLECTION_CONTROL_
/// AERO_IMPLEMENTATION, 2026-08-26).
struct AeroState
{
  double u = 0.0, v = 0.0, w = 0.0;       // body-frame relative wind velocity, m/s (Vrel = Vbody - Vwind)
  double p = 0.0, q = 0.0, r = 0.0;       // body-frame angular velocity, rad/s (roll/pitch/yaw rate about X/Y/Z)
  double deltaA = 0.0, deltaE = 0.0, deltaR = 0.0; // rad, sign-mapped (see struct comment re: no pre-clamp)
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
// terms (Cm0, Cma*alpha, and now the elevator/aileron wide-deflection
// pitching-moment lookup corrections - formerly Cmde*deltaE) are NOT
// self-referential: each relates an independently-defined ANGLE (alpha, or
// a control deflection - not a rate) to a moment about +Y. Given the
// confirmed "+Y rotation -> NOSE DOWN" finding above (opposite of the
// traditional "positive Cm/My = nose up" aerospace shorthand, which is only
// consistent with strict right-hand-rule rotation in FRD, not FLU), a
// LITERAL, unmodified application of My = qbar*S*c_ref*Cm to the STATIC
// portion of Cm produces, for a nose-up disturbance (alpha>0, Cma<0): a
// moment that is DESTABILIZING (nose-up-reinforcing), the OPPOSITE of the
// textbook-expected restoring (nose-down) behavior.
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
// here: negate ONLY the static group when mapping onto the FLU +Y torque
// axis; the rate group (Cmq*qHat) is already correct and must NOT be
// flipped. The elevator/aileron wide-deflection pitching-moment lookup
// corrections (ctrlElevDCm/ctrlAileDCm, this pass, replacing Cmde*deltaE)
// are included in the static group deliberately - each is a geometric
// angle input (control deflection), not a rate, so it has the same
// axis-handedness exposure as Cma*alpha, independently of the SEPARATE
// question of whether a positive Gazebo joint command physically means
// trailing-edge-down (already resolved for all 3 surfaces, task
// CONTROL_SURFACE_SIGN_MAPPING). Cm0 and Cma are flipped together as a
// single group (never independently) because Cm0 was specifically derived
// so that Cm0+Cma*alpha_trim=0 at the neutral trim point (AERODYNAMICS.md
// sec 19.3) - flipping only one would break that zero-moment trim
// condition. out.Cm itself is still reported in XFLR5's own (unflipped)
// convention (for diagnostics/comparability against the source-of-truth
// sweep tables) - only the MOMENT computation (my, below) applies the
// correction. See AERODYNAMICS.md sec 19.7/19.12 for the full record of
// this finding and its resolution, and the self-test executable for the
// updated Cma_RESTORING_SIGN_TEST result.
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
// NOT TOUCHED by the HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION pass - this
// is an angle-of-attack concept, unrelated to control-surface deflection.
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

// -----------------------------------------------------------------------
// Piecewise-linear control-surface wide-deflection lookup
// (HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION, 2026-08-26)
// -----------------------------------------------------------------------
// Standard piecewise-linear interpolation over a fixed, sorted (strictly
// increasing) set of breakpoints. DOMAIN-BOUNDED: any input at or beyond
// the first/last breakpoint returns the first/last VALUE exactly (clamped),
// never extrapolated - this is the "no silent extrapolation" boundary
// (CLAUDE.md) for control-surface deflections, now at +/-45 deg (the
// actuator's mechanical range) instead of the old generic +/-10 deg input
// clamp (RETIRED, see AeroConfig::elevatorSign comment above). Method:
// linear interpolation between the two bracketing breakpoints (documented
// here per CLAUDE.md's "document every interpolation method" rule; valid
// range is exactly [breakpoints.front(), breakpoints.back()] = +/-45 deg
// for every curve using this function in this file).
inline double InterpLinear(const CtrlLookupArray &breakpoints,
                            const CtrlLookupArray &values, double x)
{
  const double lo = breakpoints.front();
  const double hi = breakpoints.back();
  if (x <= lo)
    return values.front();
  if (x >= hi)
    return values.back();

  // Breakpoints are sorted strictly increasing (verified by the self-test,
  // BREAKPOINTS_SORTED_TEST) - std::upper_bound finds the first breakpoint
  // strictly greater than x, giving the upper end of the bracketing segment.
  const auto it = std::upper_bound(breakpoints.begin(), breakpoints.end(), x);
  const std::size_t i1 = static_cast<std::size_t>(it - breakpoints.begin());
  const std::size_t i0 = i1 - 1;

  const double t = (x - breakpoints[i0]) / (breakpoints[i1] - breakpoints[i0]);
  return values[i0] + t * (values[i1] - values[i0]);
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

  // -----------------------------------------------------------------------
  // Control-surface aerodynamic effect: WIDE-DEFLECTION PIECEWISE-LINEAR
  // LOOKUP (HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION, 2026-08-26).
  // REPLACES the old linear-coefficient + generic +/-10 deg clamp model
  // entirely, for all three surfaces. st.deltaA/E/R are already sign-mapped
  // (AerodynamicsSystem.cc) and are used DIRECTLY here (no pre-clamp - the
  // old V1_CONSERVATIVE_CLAMP `controlDeflectionClamp` field is RETIRED);
  // InterpLinear()'s own domain bound (+/-45 deg) is now the single "no
  // silent extrapolation" boundary, matching the actuator's mechanical
  // range instead of the old conservative +/-10 deg aero-input clamp.
  //
  // ARCHITECTURE: "full lookup replaces the static control term only" - each
  // lookup result below is used EXACTLY where the corresponding old
  // Cxda/Cxdr/Cmde*deltaX linear term used to be, never in addition to it
  // (no double-count). The old scalar constants (Clda/Cnda/CYda/CYdr/Cndr/
  // Cmde/CLde) remain in AeroConfig purely as small-signal reference/
  // self-test values now - not read here.
  // -----------------------------------------------------------------------
  const double dCYa = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileCY, st.deltaA);
  const double dCla = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileCl, st.deltaA);
  const double dCna = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileCn, st.deltaA);
  const double dCLaCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileDCL, st.deltaA);
  const double dCmaCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileDCm, st.deltaA);
  const double dCDaCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileDCD, st.deltaA);

  const double dCYr = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlRuddCY, st.deltaR);
  const double dClr = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlRuddCl, st.deltaR);
  const double dCnr = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlRuddCn, st.deltaR);
  const double dCDrCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlRuddDCD, st.deltaR);

  const double dCLeCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlElevDCL, st.deltaE);
  const double dCmeCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlElevDCm, st.deltaE);
  const double dCDeCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlElevDCD, st.deltaE);

  // Coefficient build-up. Cross-axis (untouched) stability-derivative terms
  // (beta/p_hat/q_hat/r_hat) are exactly as before; only the control-
  // deflection contribution changed (linear-with-clamp -> lookup).
  out.CY = cfg.CYb * out.beta + cfg.CYp * pHat + cfg.CYr * rHat + dCYa + dCYr;
  out.Cl = cfg.Clb * out.beta + cfg.Clp * pHat + cfg.Clr * rHat + dCla + dClr;
  out.Cn = cfg.Cnb * out.beta + cfg.Cnp * pHat + cfg.Cnr * rHat + dCna + dCnr;

  // Cm split into a STATIC/angle-derived group (Cm0, Cma*alpha, and now the
  // elevator + aileron wide-deflection pitching-moment lookup corrections -
  // each relates an independently-defined ANGLE to a moment, exactly like
  // the Cma*alpha term analyzed in the "RESOLVED FINDING" comment above)
  // and a RATE group (Cmq*qHat - self-referential, unaffected by this
  // pass). Only the STATIC group needs the FLU pitch-axis sign correction
  // when mapped onto My below; the RATE group must NOT be flipped.
  // out.Cm itself is reported in XFLR5's OWN (unflipped) convention, for
  // diagnostics/comparability against the source-of-truth sweep tables -
  // only the MOMENT computation (my, below) applies the correction.
  const double cmStatic = cfg.Cm0 + cfg.Cma * out.alpha + dCmeCtrl + dCmaCtrl;
  const double cmRate = cfg.Cmq * qHat;
  out.Cm = cmStatic + cmRate;

  // CL: high-alpha smooth saturation applied to the alpha-driven STATIC
  // term only (unchanged from the prior pass); CLq*qHat (RATE term) added
  // on top, unsaturated. NEW this pass: the elevator wide-deflection lift
  // lookup (dCLeCtrl, previously entirely absent - CLde was deliberately
  // omitted, see AERODYNAMICS.md sec 7.1/aero_v1_config.yaml) and the
  // aileron secondary lift correction (dCLaCtrl, also new) are added on
  // top, exactly mirroring the "full lookup replaces static term" pattern.
  out.CL = SaturatedCL(cfg, out.alpha) + cfg.CLq * qHat + dCLeCtrl + dCLaCtrl;

  // CD: Part-4 drag integration (HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION,
  // 2026-08-26). V1 full-aircraft calibrated polar (CD0 + k*CL^2, unchanged)
  // PLUS the three per-surface wide-deflection drag corrections, summed
  // ADDITIVELY. This is an explicit, documented V1 SIMPLIFICATION for
  // simultaneous multi-surface deflection: the three dCD_x corrections were
  // each measured with ONLY that one surface deflected (single-surface
  // isolated sweeps) - simply adding them when multiple surfaces are
  // deflected together has NOT been validated against a true combined-
  // deflection XFLR5 sweep (no such multi-surface sweep exists in the
  // source of truth). Flagged here and in AERODYNAMICS.md as
  // V1_ADDITIVE_MULTI_SURFACE_DRAG_APPROXIMATION - not fabricated
  // interaction physics, just a linear superposition assumption.
  //
  // Floored at CD0 (never below the zero-control-deflection parasite+
  // induced baseline): several dCD_x values are slightly NEGATIVE near
  // small deflections (e.g. elevator dCD=-0.00034 at delta_e=-5 deg,
  // aileron/rudder tables are drag-increasing at all nonzero deflections so
  // only elevator can go negative) - CD0 is chosen as the floor (rather
  // than 0) because CD0 is itself the aircraft's own documented, real
  // parasite-drag constant (AERODYNAMICS.md sec 6.5) that always physically
  // exists regardless of control-surface deflection; flooring at exactly 0
  // would imply a physically implausible zero-drag airframe and would be
  // less defensible than flooring at the aircraft's own baseline.
  const double cdRaw = cfg.CD0 + cfg.dragK * out.CL * out.CL +
                        dCDeCtrl + dCDaCtrl + dCDrCtrl;
  out.CD = std::max(cdRaw, cfg.CD0);

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

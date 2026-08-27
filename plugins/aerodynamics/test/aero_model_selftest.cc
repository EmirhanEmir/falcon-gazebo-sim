// =============================================================================
// FALCON V2 - Aerodynamics V1 standalone, Gazebo-independent self-test
// =============================================================================
// Owner: aerodynamics specialist agent. Task: AERODYNAMICS_V1_IMPLEMENTATION
// (2026-08-22).
//
// UPDATED (2026-08-26, task HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION):
// added the LOOKUP_EXACT_BREAKPOINT_TEST, LOOKUP_SMALL_SIGNAL_DERIVATIVE_
// RECOVERY_TEST, and LOOKUP_NO_EXTRAPOLATION_TEST blocks for the new
// wide-deflection control-surface lookup architecture; updated
// MakeFalconV2Config() to mirror the updated aero_v1_config.yaml (new
// lookup tables, updated small-signal reference constants, removed the
// retired controlDeflectionClamp field).
//
// Exercises the PURE-MATH core (AeroModel.hh) directly - no Gazebo instance,
// no ECM, no plugin loading. This is NOT a substitute for gazebo-testing's
// live-Gazebo test suite (ZERO_AIRSPEED_AERO_TEST, AOA_SIGN_TEST, etc. must
// still be run against the real AerodynamicsSystem plugin in a live gz-sim
// instance to confirm the ECM/joint/quaternion plumbing is also correct) -
// this only confirms the underlying formulas in AeroModel.hh are internally
// consistent, numerically sane, and behave as analyzed in AeroModel.hh's own
// derivation comments. Every result below is printed, not silently asserted
// away - failures are reported honestly, including the known Cma finding
// (see AeroModel.hh "IMPORTANT FINDING" comment).
//
// Build: see ../CMakeLists.txt (target aero_model_selftest, only depends on
// gz-math7, no full gz-sim/Gazebo runtime required).
// =============================================================================
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "../AeroModel.hh"

using namespace falcon_v2_aero;

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

bool IsFinite(double v) { return std::isfinite(v); }
bool IsFinite(const gz::math::Vector3d &v)
{
  return IsFinite(v.X()) && IsFinite(v.Y()) && IsFinite(v.Z());
}

/// \brief Populate the FALCON V2 V1 coefficient set exactly as given in
/// CLAUDE.md / AERODYNAMICS.md / aero_v1_config.yaml (same numeric values a
/// live plugin run would load from the YAML file - duplicated here as
/// literal constants ONLY for this standalone test's convenience; the
/// authoritative, single source of truth remains aero_v1_config.yaml, which
/// the real plugin actually loads at runtime).
AeroConfig MakeFalconV2Config()
{
  AeroConfig c;
  c.S = 0.4514;
  c.b = 2.093;
  c.c_ref = 0.224;
  c.rho = 1.225;
  c.vSafeFloor = 1.0e-3;

  c.CLa = 5.44594;
  c.Cma = -1.65805;
  c.CLq = 9.48457;
  c.Cmq = -10.22875;
  // Cmde/CLde: SUPERSEDED_BY_LOOKUP (2026-08-26) - reference-only values,
  // updated per the new fixed-condition XFLR5 data (was Cmde=-0.73, no
  // CLde at all). Not read by ComputeAero() itself anymore.
  c.Cmde = -1.000;
  c.CLde = 0.414;
  c.CL0 = 0.437035;   // DERIVED, see AERODYNAMICS.md
  c.Cm0 = 0.010550;   // DERIVED, see AERODYNAMICS.md

  // Lateral-directional small-signal reference constants. All
  // SUPERSEDED_BY_LOOKUP except Cldr (1B UNRESOLVED_KEEP_CURRENT - still
  // functionally used by Prepare() to derive ctrlRuddCl).
  c.CYb = -0.13216; c.CYp = -0.04567; c.CYr = 0.08776;
  c.CYda = 0.0045;   // 1A RESOLVED_NEW_VALUE_VALID (was 0.0254)
  c.CYdr = 0.0916;   // UPDATED (was 0.085)
  c.Clb = -0.00717; c.Clp = -0.54187; c.Clr = 0.10586;
  c.Clda = 0.414;    // UPDATED (was 0.308)
  c.Cldr = 0.0007;   // 1B UNRESOLVED_KEEP_CURRENT (unchanged) - FUNCTIONALLY USED, see Prepare()
  c.Cnb = 0.03554; c.Cnp = -0.05878; c.Cnr = -0.02227;
  c.Cnda = 0.0017;   // UPDATED (was 0.00144)
  c.Cndr = -0.0272;  // UPDATED (was -0.025)

  c.CD0 = 0.0351;
  c.dragK = 0.0528;

  c.CLmax = 1.42;
  c.alphaTransition = 9.25 * M_PI / 180.0;

  // elevator_sign=-1.0 per task CONTROL_SURFACE_SIGN_MAPPING (2026-08-22):
  // VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST, was ASSUMPTION=+1.0 (backward) -
  // see aero_v1_config.yaml control_mapping. NOTE: these sign fields are not
  // exercised by ComputeAero() itself (the joint-to-delta_e sign mapping
  // happens upstream in AerodynamicsSystem.cc, not in this pure-math core) -
  // kept here only so this struct stays a faithful mirror of the real YAML.
  c.elevatorSign = -1.0; c.aileronSign = 1.0; c.rudderSign = 1.0;

  // ---------------------------------------------------------------------
  // Wide-deflection control-surface lookup tables
  // (HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION, 2026-08-26). Verified
  // 1:1 against aero_v1_config.yaml's control_surface_lookup block, which
  // was itself verified 1:1 against
  // FALCON_V2_CONTROL_SURFACE_WIDE_DEFLECTION_RESULTS.txt.
  // ---------------------------------------------------------------------
  const double kBreakpointsDeg[kNumCtrlBreakpoints] =
      {-45, -35, -25, -15, -10, -5, -2, 0, 2, 5, 10, 15, 25, 35, 45};
  for (int i = 0; i < kNumCtrlBreakpoints; ++i)
    c.ctrlBreakpointsRad[i] = kBreakpointsDeg[i] * M_PI / 180.0;

  c.ctrlElevDCL = {-0.32181, -0.25256, -0.18095, -0.10846, -0.07223, -0.03608, -0.01443, 0, 0.01442, 0.03605, 0.07211, 0.10821, 0.18053, 0.25257, 0.32309};
  c.ctrlElevDCD = {0.03392, 0.01880, 0.00826, 0.00194, 0.00031, -0.00034, -0.00026, 0, 0.00041, 0.00133, 0.00366, 0.00701, 0.01673, 0.03060, 0.04846};
  c.ctrlElevDCm = {0.77221, 0.60714, 0.43568, 0.26150, 0.17425, 0.08710, 0.03483, 0, -0.03485, -0.08714, -0.17440, -0.26184, -0.43732, -0.61249, -0.78429};

  c.ctrlAileCl = {-0.31890, -0.25052, -0.18001, -0.10836, -0.07230, -0.03617, -0.01447, 0, 0.01447, 0.03617, 0.07231, 0.10836, 0.18001, 0.25052, 0.31890};
  c.ctrlAileCn = {-0.00123, -0.00099, -0.00072, -0.00044, -0.00029, -0.00014, -0.00005, 0.00001, 0.00007, 0.00016, 0.00030, 0.00045, 0.00073, 0.00101, 0.00124};
  c.ctrlAileCY = {-0.00486, -0.00354, -0.00228, -0.00126, -0.00082, -0.00041, -0.00018, -0.00002, 0.00014, 0.00037, 0.00078, 0.00122, 0.00224, 0.00350, 0.00482};
  c.ctrlAileCDFull = {0.18174, 0.11664, 0.06757, 0.03407, 0.02355, 0.01723, 0.01546, 0.01513, 0.01546, 0.01723, 0.02355, 0.03407, 0.06757, 0.11664, 0.18174};
  c.ctrlAileCLFull = {0.62116, 0.64137, 0.65493, 0.66353, 0.66613, 0.66766, 0.66809, 0.66817, 0.66809, 0.66766, 0.66613, 0.66353, 0.65492, 0.64136, 0.62115};
  c.ctrlAileCmFull = {-0.07619, -0.07310, -0.06746, -0.06297, -0.06142, -0.06046, -0.06019, -0.06013, -0.06019, -0.06046, -0.06142, -0.06296, -0.06745, -0.07308, -0.07617};

  c.ctrlRuddCY = {-0.07094, -0.05601, -0.04023, -0.02411, -0.01604, -0.00801, -0.00321, -0.00002, 0.00317, 0.00797, 0.01600, 0.02407, 0.04020, 0.05598, 0.07093};
  c.ctrlRuddCn = {0.02115, 0.01666, 0.01195, 0.00715, 0.00475, 0.00238, 0.00095, 0.00001, -0.00094, -0.00236, -0.00474, -0.00713, -0.01194, -0.01665, -0.02114};
  c.ctrlRuddCDFull = {0.02974, 0.02390, 0.01932, 0.01682, 0.01584, 0.01528, 0.01516, 0.01513, 0.01516, 0.01528, 0.01583, 0.01682, 0.01932, 0.02390, 0.02974};
  // ctrlRuddCl is intentionally left default-zero here - Prepare() derives
  // it from c.Cldr (1B UNRESOLVED_KEEP_CURRENT), exactly mirroring
  // AerodynamicsSystem.cc's loader, which also never populates it from YAML.

  c.Prepare();
  return c;
}
}  // namespace

int main()
{
  const AeroConfig cfg = MakeFalconV2Config();

  std::printf("=================================================================\n");
  std::printf("FALCON V2 aerodynamics standalone self-test (AeroModel.hh only)\n");
  std::printf("=================================================================\n\n");

  // -----------------------------------------------------------------------
  // ZERO_AIRSPEED_AERO_TEST
  // -----------------------------------------------------------------------
  {
    AeroState st;  // all zero
    AeroOutput out = ComputeAero(cfg, st);
    bool ok = IsFinite(out.V) && IsFinite(out.alpha) && IsFinite(out.beta) &&
              IsFinite(out.qbar) && IsFinite(out.CL) && IsFinite(out.CD) &&
              IsFinite(out.forceBody) && IsFinite(out.momentBody) &&
              out.qbar == 0.0 && out.forceBody.Length() == 0.0 &&
              out.momentBody.Length() == 0.0;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "V=%.6f qbar=%.6f F=(%.3f,%.3f,%.3f) M=(%.3f,%.3f,%.3f) all-finite",
        out.V, out.qbar, out.forceBody.X(), out.forceBody.Y(), out.forceBody.Z(),
        out.momentBody.X(), out.momentBody.Y(), out.momentBody.Z());
    Check("ZERO_AIRSPEED_AERO_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // AOA_SIGN_TEST (math level: nose-up scenario -> alpha>0)
  // -----------------------------------------------------------------------
  {
    // theta_physical = +5 deg nose-up (per AeroModel.hh derivation):
    // u=V*cos(theta), w=-V*sin(theta)
    const double V = 20.0, thetaDeg = 5.0, theta = thetaDeg * M_PI / 180.0;
    const double u = V * std::cos(theta), w = -V * std::sin(theta);
    const double alpha = AngleOfAttack(u, 0.0, w);
    bool ok = std::abs(alpha - theta) < 1e-9;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "nose-up %.1f deg scenario -> alpha=%.4f deg (expected %.4f)",
        thetaDeg, alpha * 180.0 / M_PI, thetaDeg);
    Check("AOA_SIGN_TEST (math-level)", ok, buf);

    // also confirm negative alpha for nose-down
    const double u2 = V * std::cos(-theta), w2 = -V * std::sin(-theta);
    const double alpha2 = AngleOfAttack(u2, 0.0, w2);
    Check("AOA_SIGN_TEST negative-alpha case", std::abs(alpha2 + theta) < 1e-9,
          "nose-down scenario -> alpha negative as expected");
  }

  // -----------------------------------------------------------------------
  // SIDESLIP_SIGN_TEST (math level: documents the chosen Candidate-B
  // convention and shows it is self-consistent; PHYSICAL confirmation still
  // requires live Gazebo per AeroModel.hh's Sideslip() derivation comment)
  // -----------------------------------------------------------------------
  {
    const double V = 20.0, psiDeg = 5.0, psi = psiDeg * M_PI / 180.0;
    // nose yawed LEFT by psi (= wind from the right): v = -V*sin(psi)
    const double u = V * std::cos(psi), v = -V * std::sin(psi);
    const double beta = Sideslip(u, v, 0.0);
    bool ok = std::abs(beta + psi) < 1e-9;  // Candidate B => beta = -psi here
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "nose-left/wind-from-right scenario -> beta=%.4f deg (Candidate-B "
        "convention: positive beta = wind from LEFT, so this case is negative "
        "by construction; NOT independently physically confirmed - see "
        "AeroModel.hh)",
        beta * 180.0 / M_PI);
    Check("SIDESLIP_SIGN_TEST (formula self-consistency)", ok, buf);
  }

  // -----------------------------------------------------------------------
  // LIFT_SIGN_TEST / DRAG_SIGN_TEST (alpha=0, beta=0, positive CL/CD => Fz>0, Fx<0)
  // -----------------------------------------------------------------------
  {
    gz::math::Vector3d f = WindToBodyForce(/*lift*/10.0, /*drag*/2.0, /*side*/0.0,
                                            /*alpha*/0.0, /*beta*/0.0);
    Check("LIFT_SIGN_TEST (alpha=beta=0)", f.Z() > 0.0 && std::abs(f.Z() - 10.0) < 1e-9,
          "Fz=" + std::to_string(f.Z()) + " (expect +10, lift acts +Z/up)");
    Check("DRAG_SIGN_TEST (alpha=beta=0)", f.X() < 0.0 && std::abs(f.X() + 2.0) < 1e-9,
          "Fx=" + std::to_string(f.X()) + " (expect -2, drag acts -X/aft)");
  }
  {
    // Positive alpha: check the force vector rotates sanely, not naive Fx=-D,Fz=L
    gz::math::Vector3d f0 = WindToBodyForce(10.0, 2.0, 0.0, 0.0, 0.0);
    gz::math::Vector3d fPos = WindToBodyForce(10.0, 2.0, 0.0, 10.0 * M_PI / 180.0, 0.0);
    gz::math::Vector3d fNeg = WindToBodyForce(10.0, 2.0, 0.0, -10.0 * M_PI / 180.0, 0.0);
    bool ok = IsFinite(fPos) && IsFinite(fNeg) &&
              std::abs(fPos.X() - f0.X()) > 1e-6 &&  // rotation actually changes Fx (not the naive constant -D)
              std::abs(fNeg.X() - f0.X()) > 1e-6;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "alpha=0: F=(%.3f,%.3f) | alpha=+10deg: F=(%.3f,%.3f) | alpha=-10deg: F=(%.3f,%.3f) "
        "(Fx/Fz both change with alpha -> proper rotation, not naive Fx=-D/Fz=L)",
        f0.X(), f0.Z(), fPos.X(), fPos.Z(), fNeg.X(), fNeg.Z());
    Check("Wind-to-body rotation sanity (+/-alpha)", ok, buf);
  }

  // -----------------------------------------------------------------------
  // Cma_RESTORING_SIGN_TEST -- RESOLVED (was an honest, reported FAIL;
  // gazebo-testing confirmed it live, validation root-caused it, the scoped
  // static/rate Cm-to-My fix is applied in AeroModel.hh - see the "RESOLVED
  // FINDING" comment there). Uses deltaE=0 throughout, so the 2026-08-26
  // lookup-table change does not affect this test at all.
  // -----------------------------------------------------------------------
  {
    const double alphaTrim = 0.36455 * M_PI / 180.0;
    AeroState stTrim; stTrim.u = 21.244 * std::cos(alphaTrim);
    stTrim.w = -21.244 * std::sin(alphaTrim);
    AeroOutput outTrim = ComputeAero(cfg, stTrim);

    AeroState stDist = stTrim;
    const double extra = 2.0 * M_PI / 180.0;  // +2 deg nose-up disturbance above trim
    const double aDist = alphaTrim + extra;
    stDist.u = 21.244 * std::cos(aDist);
    stDist.w = -21.244 * std::sin(aDist);
    AeroOutput outDist = ComputeAero(cfg, stDist);

    // My>0 = NOSE DOWN (derived in AeroModel.hh). Restoring for a nose-up
    // disturbance means My should be MORE POSITIVE (more nose-down) than at
    // trim.
    bool restoring = outDist.momentBody.Y() > outTrim.momentBody.Y();
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "My(trim)=%.6f My(+2deg nose-up)=%.6f -> %s (My>0=nose-down; scoped "
        "static/rate Cm-to-My fix applied, see AeroModel.hh 'RESOLVED "
        "FINDING')",
        outTrim.momentBody.Y(), outDist.momentBody.Y(),
        restoring ? "RESTORING" : "DESTABILIZING");
    Check("Cma_RESTORING_SIGN_TEST", restoring, buf);
  }

  // -----------------------------------------------------------------------
  // Cmq_DAMPING_SIGN_TEST / Clp_DAMPING_SIGN_TEST / Cnr_DAMPING_SIGN_TEST
  // (self-referential: verified frame-independent in AeroModel.hh comments)
  // -----------------------------------------------------------------------
  {
    AeroState st; st.u = 21.244; st.q = 1.0;  // positive pitch rate only
    AeroOutput out = ComputeAero(cfg, st);
    // damping requires My opposite sign to q (opposes the existing rotation)
    Check("Cmq_DAMPING_SIGN_TEST", out.momentBody.Y() < 0.0,
          "q=+1 rad/s -> My=" + std::to_string(out.momentBody.Y()) + " (expect <0, opposing q)");
  }
  {
    AeroState st; st.u = 21.244; st.p = 1.0;
    AeroOutput out = ComputeAero(cfg, st);
    Check("Clp_DAMPING_SIGN_TEST", out.momentBody.X() < 0.0,
          "p=+1 rad/s -> Mx=" + std::to_string(out.momentBody.X()) + " (expect <0, opposing p)");
  }
  {
    AeroState st; st.u = 21.244; st.r = 1.0;
    AeroOutput out = ComputeAero(cfg, st);
    Check("Cnr_DAMPING_SIGN_TEST", out.momentBody.Z() < 0.0,
          "r=+1 rad/s -> Mz=" + std::to_string(out.momentBody.Z()) + " (expect <0, opposing r)");
  }

  // -----------------------------------------------------------------------
  // Cnb_STATIC_STABILITY_SIGN_TEST
  // -----------------------------------------------------------------------
  {
    const double V = 21.244, psi = 5.0 * M_PI / 180.0;  // nose slipped left = wind from right
    AeroState st; st.u = V * std::cos(psi); st.v = -V * std::sin(psi);
    AeroOutput out = ComputeAero(cfg, st);
    // Mz>0 = nose LEFT (derived). Restoring for a nose-left disturbance needs Mz<0 (nose right).
    Check("Cnb_STATIC_STABILITY_SIGN_TEST", out.momentBody.Z() < 0.0,
          "nose-slipped-left scenario -> Mz=" + std::to_string(out.momentBody.Z()) +
          " (expect <0 = nose pushed right = restoring)");
  }

  // -----------------------------------------------------------------------
  // AILERON_ROLL_SIGN_TEST / RUDDER_YAW_SIGN_TEST / ELEVATOR_PITCH_SIGN_TEST
  // -----------------------------------------------------------------------
  // This self-test can only verify the ALGEBRAIC level here (does a positive
  // delta_x move the corresponding moment in the direction the given
  // lookup table implies, using an already-abstract delta_x - not a real
  // joint angle) - still reported as INFO, not PASS/FAIL, since this
  // executable never re-derives the full joint->delta_x->moment chain
  // itself. The full physical chain (joint sign -> delta_x -> real applied
  // moment) HAS been independently confirmed for all three surfaces via
  // live Gazebo kinematic + aero-moment measurements (task
  // CONTROL_SURFACE_SIGN_MAPPING, 2026-08-22, 9/9 CONFIRMS-HYPOTHESIS) -
  // see aero_v1_config.yaml control_mapping and AERODYNAMICS.md sec 19.13.
  // As of 2026-08-26 the underlying Cl/Cn/CY/Cm/CL contribution comes from
  // the wide-deflection lookup tables instead of the old linear
  // coefficients, but the sign-mapping chain itself (elevator_sign=-1.0,
  // aileron_sign/rudder_sign=+1.0) is UNCHANGED by this pass.
  {
    AeroState st; st.u = 21.244; st.deltaA = 5.0 * M_PI / 180.0;
    AeroOutput out = ComputeAero(cfg, st);
    Info("AILERON_ROLL_SIGN (algebraic only)",
         "delta_a=+5deg -> Mx=" + std::to_string(out.momentBody.X()) +
         " (lookup ctrlAileCl(+5deg)=" +
         std::to_string(InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileCl, 5.0 * M_PI / 180.0)) +
         "; physical joint->moment chain CONFIRMED live, task "
         "CONTROL_SURFACE_SIGN_MAPPING, aileron_sign=+1.0 correct)");
  }
  {
    AeroState st; st.u = 21.244; st.deltaR = 5.0 * M_PI / 180.0;
    AeroOutput out = ComputeAero(cfg, st);
    Info("RUDDER_YAW_SIGN (algebraic only)",
         "delta_r=+5deg -> Mz=" + std::to_string(out.momentBody.Z()) +
         " (lookup ctrlRuddCn(+5deg)=" +
         std::to_string(InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlRuddCn, 5.0 * M_PI / 180.0)) +
         "; physical joint->moment chain CONFIRMED live, task "
         "CONTROL_SURFACE_SIGN_MAPPING, rudder_sign=+1.0 correct)");
  }
  {
    AeroState st; st.u = 21.244; st.deltaE = 5.0 * M_PI / 180.0;
    AeroOutput out = ComputeAero(cfg, st);
    Info("ELEVATOR_PITCH_SIGN (algebraic only)",
         "delta_e=+5deg -> My=" + std::to_string(out.momentBody.Y()) +
         " (lookup ctrlElevDCm(+5deg)=" +
         std::to_string(InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlElevDCm, 5.0 * M_PI / 180.0)) +
         "; physical joint->moment chain CONFIRMED live, task "
         "CONTROL_SURFACE_SIGN_MAPPING, elevator_sign=-1.0 correct. Cm's "
         "elevator lookup contribution is part of the negated 'static' "
         "group in the Cm-to-My fix, same as the old Cmde*deltaE term it "
         "replaced)");
  }

  // -----------------------------------------------------------------------
  // RATE_NORMALIZATION_TEST
  // -----------------------------------------------------------------------
  {
    // At V=21.244, p=q=r=1 rad/s, confirm p_hat/q_hat/r_hat magnitudes match
    // the documented formulas exactly (cross-check via moment/derivative
    // back-solve, since AeroOutput doesn't expose *_hat directly).
    AeroState st; st.u = 21.244; st.p = 1.0; st.q = 0.0; st.r = 0.0;
    AeroOutput out = ComputeAero(cfg, st);
    const double V = 21.244;
    const double pHatExpected = 1.0 * cfg.b / (2.0 * V);
    const double clExpected = cfg.Clp * pHatExpected;  // deltaA/R=0 -> lookup contribution is 0 (elev/aile/rudd tables are 0 at breakpoint index 7)
    bool ok = std::abs(out.Cl - clExpected) < 1e-12;
    Check("RATE_NORMALIZATION_TEST (p_hat = p*b/2V)", ok,
          "Cl=" + std::to_string(out.Cl) + " expected=" + std::to_string(clExpected));

    // CLq*qHat regression check (previously cfg.CLq was loaded but never
    // referenced in the CL build-up - fixed in an earlier pass).
    AeroState stQ; stQ.u = V; stQ.q = 1.0;
    AeroOutput outQ = ComputeAero(cfg, stQ);
    AeroState stQ0; stQ0.u = V;
    AeroOutput outQ0 = ComputeAero(cfg, stQ0);
    const double qHatExpected = 1.0 * cfg.c_ref / (2.0 * V);
    const double clqContribExpected = cfg.CLq * qHatExpected;
    const double clqContribActual = outQ.CL - outQ0.CL;
    bool okClq = std::abs(clqContribActual - clqContribExpected) < 1e-12;
    Check("RATE_NORMALIZATION_TEST (CLq*q_hat still included in CL)", okClq,
          "CL(q=1)-CL(q=0)=" + std::to_string(clqContribActual) +
          " expected=" + std::to_string(clqContribExpected));

    // Also confirm the V-floor prevents div-by-zero at V=0 with nonzero rates
    AeroState st0; st0.p = 1.0; st0.q = 1.0; st0.r = 1.0;
    AeroOutput out0 = ComputeAero(cfg, st0);
    Check("RATE_NORMALIZATION_TEST (V=0, rates nonzero -> no NaN/Inf)",
          IsFinite(out0.Cl) && IsFinite(out0.Cm) && IsFinite(out0.Cn) &&
          IsFinite(out0.forceBody) && IsFinite(out0.momentBody) &&
          out0.forceBody.Length() == 0.0 && out0.momentBody.Length() == 0.0,
          "qbar=0 forces the physical force/moment to exactly zero even though "
          "*_hat itself is a large-but-finite number at V=0");
  }

  // -----------------------------------------------------------------------
  // DRAG_POLAR_TEST + TRIM_BENCHMARK
  // -----------------------------------------------------------------------
  {
    const double alphaTrim = 0.36455 * M_PI / 180.0;
    AeroState st; st.u = 21.244 * std::cos(alphaTrim); st.w = -21.244 * std::sin(alphaTrim);
    AeroOutput out = ComputeAero(cfg, st);
    const double cdExpected = cfg.CD0 + cfg.dragK * out.CL * out.CL;  // deltaE=A=R=0 -> all lookup dCD contributions are exactly 0
    bool ok = std::abs(out.CD - cdExpected) < 1e-12 && std::abs(out.CL - 0.471685) < 2e-3;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "trim: CL=%.6f (expect ~0.4717) CD=%.6f (CD0+k*CL^2=%.6f)",
                  out.CL, out.CD, cdExpected);
    Check("DRAG_POLAR_TEST + TRIM_BENCHMARK (V=21.244, neutral)", ok, buf);
  }

  // -----------------------------------------------------------------------
  // HIGH_ALPHA_LIMITER_TEST (unchanged this pass - not touched)
  // -----------------------------------------------------------------------
  {
    bool ok = true;
    char buf[512]; buf[0] = 0;
    double prevCL = -1e9;
    char tmp[128];
    for (double aDeg = -90.0; aDeg <= 90.0; aDeg += 5.0)
    {
      double a = aDeg * M_PI / 180.0;
      double cl = SaturatedCL(cfg, a);
      if (cl > cfg.CLmax + 1e-6 || cl < -cfg.CLmax - 1e-6) ok = false;
      if (cl < prevCL - 1e-9) ok = false;  // must be monotonically non-decreasing
      prevCL = cl;
    }
    double clLinearWithinBand = cfg.CL0 + cfg.CLa * (5.0 * M_PI / 180.0);
    double clSatWithinBand = SaturatedCL(cfg, 5.0 * M_PI / 180.0);
    ok = ok && std::abs(clLinearWithinBand - clSatWithinBand) < 1e-9;
    double clAt90 = SaturatedCL(cfg, 90.0 * M_PI / 180.0);
    double clAtNeg90 = SaturatedCL(cfg, -90.0 * M_PI / 180.0);
    std::snprintf(tmp, sizeof(tmp), "CL(90deg)=%.4f CL(-90deg)=%.4f (|.|<=CLmax=%.2f); "
                  "exact-linear match within +/-9.25deg band; monotonic",
                  clAt90, clAtNeg90, cfg.CLmax);
    Check("HIGH_ALPHA_LIMITER_TEST", ok, tmp);
  }

  // =========================================================================
  // NEW THIS PASS (2026-08-26, HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION):
  // wide-deflection control-surface lookup table tests.
  // =========================================================================

  // -----------------------------------------------------------------------
  // BREAKPOINTS_SORTED_TEST - precondition InterpLinear() relies on.
  // -----------------------------------------------------------------------
  {
    bool sorted = true;
    for (int i = 1; i < kNumCtrlBreakpoints; ++i)
      if (!(cfg.ctrlBreakpointsRad[i] > cfg.ctrlBreakpointsRad[i - 1]))
        sorted = false;
    bool zeroAtIndex7 = std::abs(cfg.ctrlBreakpointsRad[kCtrlZeroIndex]) < 1e-15;
    Check("BREAKPOINTS_SORTED_TEST", sorted && zeroAtIndex7,
          sorted ? "strictly increasing, breakpoints[7]==0 exactly"
                 : "NOT sorted - InterpLinear() precondition violated");
  }

  // -----------------------------------------------------------------------
  // LOOKUP_EXACT_BREAKPOINT_TEST - InterpLinear() must return the exact
  // table value at every one of the 15 breakpoints, for every curve used by
  // any of the 3 control surfaces.
  // -----------------------------------------------------------------------
  {
    struct Curve { const char *name; const CtrlLookupArray *arr; };
    const Curve curves[] = {
        {"elevator.dCL", &cfg.ctrlElevDCL},
        {"elevator.dCD", &cfg.ctrlElevDCD},
        {"elevator.dCm", &cfg.ctrlElevDCm},
        {"aileron.Cl", &cfg.ctrlAileCl},
        {"aileron.Cn", &cfg.ctrlAileCn},
        {"aileron.CY", &cfg.ctrlAileCY},
        {"aileron.dCD (derived)", &cfg.ctrlAileDCD},
        {"aileron.dCL (derived)", &cfg.ctrlAileDCL},
        {"aileron.dCm (derived)", &cfg.ctrlAileDCm},
        {"rudder.CY", &cfg.ctrlRuddCY},
        {"rudder.Cn", &cfg.ctrlRuddCn},
        {"rudder.Cl (derived from Cldr)", &cfg.ctrlRuddCl},
        {"rudder.dCD (derived)", &cfg.ctrlRuddDCD},
    };
    bool allOk = true;
    int worstCurveIdx = -1;
    double worstErr = 0.0;
    for (std::size_t c = 0; c < sizeof(curves) / sizeof(curves[0]); ++c)
    {
      for (int i = 0; i < kNumCtrlBreakpoints; ++i)
      {
        const double x = cfg.ctrlBreakpointsRad[i];
        const double got = InterpLinear(cfg.ctrlBreakpointsRad, *curves[c].arr, x);
        const double expected = (*curves[c].arr)[i];
        const double err = std::abs(got - expected);
        if (err > 1e-12)
        {
          allOk = false;
          if (err > worstErr) { worstErr = err; worstCurveIdx = static_cast<int>(c); }
        }
      }
    }
    char buf[256];
    if (allOk)
      std::snprintf(buf, sizeof(buf),
          "all 13 curves x 15 breakpoints (195 checks) return the exact "
          "table value (max err < 1e-12)");
    else
      std::snprintf(buf, sizeof(buf), "worst mismatch in curve '%s', err=%.3e",
                    curves[worstCurveIdx].name, worstErr);
    Check("LOOKUP_EXACT_BREAKPOINT_TEST (all 3 surfaces)", allOk, buf);
  }

  // -----------------------------------------------------------------------
  // AILERON/RUDDER_BASELINE_DIFFERENCE_TEST - confirm Prepare()'s
  // baseline-differencing of the FULL-VALUE CD/CL/Cm tables is correct
  // (delta=0 row of the derived table must be exactly 0), and confirm
  // ctrlRuddCl was correctly derived from Cldr (bounded linear extension,
  // 1B UNRESOLVED_KEEP_CURRENT).
  // -----------------------------------------------------------------------
  {
    bool ok = std::abs(cfg.ctrlAileDCD[kCtrlZeroIndex]) < 1e-15 &&
              std::abs(cfg.ctrlAileDCL[kCtrlZeroIndex]) < 1e-15 &&
              std::abs(cfg.ctrlAileDCm[kCtrlZeroIndex]) < 1e-15 &&
              std::abs(cfg.ctrlRuddDCD[kCtrlZeroIndex]) < 1e-15;
    Check("BASELINE_DIFFERENCE_ZERO_AT_ORIGIN_TEST", ok,
          "aileron/rudder derived dCD/dCL/dCm tables are exactly 0 at delta=0 (index 7)");
  }
  {
    // ctrlRuddCl[i] should equal Cldr * breakpoint[i] exactly (linear
    // extension of the OLD small-signal constant, per 1B resolution).
    bool ok = true;
    double worst = 0.0;
    for (int i = 0; i < kNumCtrlBreakpoints; ++i)
    {
      const double expected = cfg.Cldr * cfg.ctrlBreakpointsRad[i];
      const double err = std::abs(cfg.ctrlRuddCl[i] - expected);
      if (err > 1e-15) { ok = false; worst = std::max(worst, err); }
    }
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "ctrlRuddCl[i] == Cldr(%.6f/rad)*breakpoint[i] exactly at all 15 "
        "points (1B UNRESOLVED_KEEP_CURRENT bounded linear extension); "
        "max err=%.3e", cfg.Cldr, worst);
    Check("RUDDER_CL_LINEAR_EXTENSION_TEST (1B resolution)", ok, buf);
  }

  // -----------------------------------------------------------------------
  // LOOKUP_SMALL_SIGNAL_DERIVATIVE_RECOVERY_TEST - a central-difference
  // slope of each lookup curve near delta=0, at the +/-2/+/-5/+/-10 deg
  // windows, should closely match the corresponding Part-2 small-signal
  // reference constant (cfg.CLde/Cmde/Clda/Cnda/CYda/CYdr/Cndr/Cldr) -
  // "closely" here means within a few percent, since the lookup is a
  // piecewise-LINEAR fit through the *exact* source data points (not a
  // perfect single straight line - the source data itself is not perfectly
  // linear across a whole +/-10 deg window, see AERODYNAMICS.md sec 20).
  // -----------------------------------------------------------------------
  {
    auto centralDiffDeg = [&cfg](const CtrlLookupArray &arr, double halfWindowDeg)
    {
      const double h = halfWindowDeg * M_PI / 180.0;
      const double plus = InterpLinear(cfg.ctrlBreakpointsRad, arr, h);
      const double minus = InterpLinear(cfg.ctrlBreakpointsRad, arr, -h);
      return (plus - minus) / (2.0 * h);
    };

    struct Case { const char *name; const CtrlLookupArray *arr; double refVal; double tolFrac; };
    const Case cases[] = {
        {"CL_delta_e (vs CLde=0.414)", &cfg.ctrlElevDCL, cfg.CLde, 0.02},
        {"Cm_delta_e (vs Cmde=-1.000)", &cfg.ctrlElevDCm, cfg.Cmde, 0.02},
        {"Cl_delta_a (vs Clda=0.414)", &cfg.ctrlAileCl, cfg.Clda, 0.02},
        {"Cn_delta_a (vs Cnda=0.0017)", &cfg.ctrlAileCn, cfg.Cnda, 0.10},
        {"CY_delta_a (vs CYda=0.0045, 1A resolved)", &cfg.ctrlAileCY, cfg.CYda, 0.10},
        {"CY_delta_r (vs CYdr=0.0916)", &cfg.ctrlRuddCY, cfg.CYdr, 0.02},
        {"Cn_delta_r (vs Cndr=-0.0272)", &cfg.ctrlRuddCn, cfg.Cndr, 0.02},
        {"Cl_delta_r (vs Cldr=0.0007, 1B kept)", &cfg.ctrlRuddCl, cfg.Cldr, 0.001},
    };

    for (const auto &tc : cases)
    {
      bool allWindowsOk = true;
      char detail[400]; detail[0] = 0;
      for (double halfDeg : {2.0, 5.0, 10.0})
      {
        const double slope = centralDiffDeg(*tc.arr, halfDeg);
        const double tol = std::abs(tc.refVal) * tc.tolFrac + 1e-6;
        const bool ok = std::abs(slope - tc.refVal) <= tol;
        if (!ok) allWindowsOk = false;
        char part[100];
        std::snprintf(part, sizeof(part), "w%.0f=%.6f ", halfDeg, slope);
        std::strncat(detail, part, sizeof(detail) - std::strlen(detail) - 1);
      }
      char buf[512];
      std::snprintf(buf, sizeof(buf), "%s ref=%.6f", detail, tc.refVal);
      Check((std::string("SMALL_SIGNAL_RECOVERY: ") + tc.name).c_str(), allWindowsOk, buf);
    }
  }

  // -----------------------------------------------------------------------
  // LOOKUP_NO_EXTRAPOLATION_TEST - inputs beyond +/-45 deg must clamp to
  // the edge value exactly (never extrapolate), and must never produce
  // NaN/Inf, arbitrarily far outside the domain.
  // -----------------------------------------------------------------------
  {
    struct Curve { const char *name; const CtrlLookupArray *arr; };
    const Curve curves[] = {
        {"elevator.dCL", &cfg.ctrlElevDCL}, {"elevator.dCD", &cfg.ctrlElevDCD}, {"elevator.dCm", &cfg.ctrlElevDCm},
        {"aileron.Cl", &cfg.ctrlAileCl}, {"aileron.Cn", &cfg.ctrlAileCn}, {"aileron.CY", &cfg.ctrlAileCY},
        {"rudder.CY", &cfg.ctrlRuddCY}, {"rudder.Cn", &cfg.ctrlRuddCn}, {"rudder.Cl", &cfg.ctrlRuddCl},
    };
    bool allOk = true;
    const double testDegs[] = {45.0, 46.0, 60.0, 90.0, 1000.0, -45.0, -46.0, -60.0, -90.0, -1000.0};
    for (const auto &curve : curves)
    {
      for (double d : testDegs)
      {
        const double x = d * M_PI / 180.0;
        const double got = InterpLinear(cfg.ctrlBreakpointsRad, *curve.arr, x);
        if (!IsFinite(got)) { allOk = false; continue; }
        const double edgeExpected = (d >= 45.0) ? curve.arr->back() : curve.arr->front();
        if (std::abs(got - edgeExpected) > 1e-12) allOk = false;
      }
    }
    // Also check the exact +/-45 deg boundary matches the edge value.
    Check("LOOKUP_NO_EXTRAPOLATION_TEST (clamped beyond +/-45deg, no NaN/Inf)",
          allOk,
          "checked 9 curves x 10 out-of-domain inputs (up to +/-1000 deg): "
          "all clamp exactly to the nearest edge value, all finite");
  }

  // -----------------------------------------------------------------------
  // DRAG_FLOOR_TEST - CD must never fall below CD0, even with a
  // negative-dCD elevator deflection driving cdRaw below CD0 at low CL.
  // -----------------------------------------------------------------------
  {
    // delta_e = -5 deg alone gives dCD_e = -0.00034 (see ctrlElevDCD[5]).
    // At alpha=0 (CL=CL0, near-minimum-CL condition), cdRaw could dip
    // slightly below CD0 without the floor.
    AeroState st; st.u = 21.244; st.deltaE = -5.0 * M_PI / 180.0;
    AeroOutput out = ComputeAero(cfg, st);
    bool ok = out.CD >= cfg.CD0 - 1e-15 && IsFinite(out.CD);
    Check("DRAG_FLOOR_TEST (CD >= CD0 always)", ok,
          "CD=" + std::to_string(out.CD) + " CD0=" + std::to_string(cfg.CD0) +
          " (elevator dCD=-0.00034 at delta_e=-5deg alone)");
  }

  std::printf("\n=================================================================\n");
  std::printf("SUMMARY: %d PASS, %d FAIL, %d INFO (INFO items require live-Gazebo\n"
              "confirmation, not pass/fail at the pure-math level)\n", gPass, gFail, gInfo);
  std::printf("=================================================================\n");

  return gFail == 0 ? 0 : 1;
}

// =============================================================================
// FALCON V2 - Aerodynamics V1 standalone, Gazebo-independent self-test
// =============================================================================
// Owner: aerodynamics specialist agent. Task: AERODYNAMICS_V1_IMPLEMENTATION
// (2026-08-22).
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
  c.Cmde = -0.73;
  c.CL0 = 0.437035;   // DERIVED, see AERODYNAMICS.md
  c.Cm0 = 0.010550;   // DERIVED, see AERODYNAMICS.md

  c.CYb = -0.13216; c.CYp = -0.04567; c.CYr = 0.08776;
  c.CYda = 0.0254; c.CYdr = 0.085;
  c.Clb = -0.00717; c.Clp = -0.54187; c.Clr = 0.10586;
  c.Clda = 0.308; c.Cldr = 0.0007;
  c.Cnb = 0.03554; c.Cnp = -0.05878; c.Cnr = -0.02227;
  c.Cnda = 0.00144; c.Cndr = -0.025;

  c.CD0 = 0.0351;
  c.dragK = 0.0528;

  c.CLmax = 1.42;
  c.alphaTransition = 9.25 * M_PI / 180.0;

  c.elevatorSign = 1.0; c.aileronSign = 1.0; c.rudderSign = 1.0;
  c.controlDeflectionClamp = 10.0 * M_PI / 180.0;

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
  // Cma_RESTORING_SIGN_TEST -- RESOLVED this pass (was an honest, reported
  // FAIL; gazebo-testing confirmed it live, validation root-caused it, the
  // scoped static/rate Cm-to-My fix is now applied in AeroModel.hh - see
  // the "RESOLVED FINDING" comment there).
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
        "static/rate Cm-to-My fix applied this pass, see AeroModel.hh "
        "'RESOLVED FINDING')",
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
  // These can only be verified at the ALGEBRAIC level here (does a positive
  // delta_x move the corresponding moment in the direction the given
  // coefficient sign implies?) - the PHYSICAL joint-to-deflection mapping is
  // explicitly ASSUMPTION-tagged (aero_v1_config.yaml control_mapping) and
  // requires controls-integration's live AILERON_TEST/ELEVATOR_TEST/
  // RUDDER_TEST before a physical direction can be asserted. Reported as
  // INFO, not PASS/FAIL, to avoid implying a physical confirmation that
  // hasn't happened.
  {
    AeroState st; st.u = 21.244; st.deltaA = 5.0 * M_PI / 180.0;
    AeroOutput out = ComputeAero(cfg, st);
    Info("AILERON_ROLL_SIGN (algebraic only)",
         "delta_a=+5deg -> Mx=" + std::to_string(out.momentBody.X()) +
         " (Clda=" + std::to_string(cfg.Clda) + "; PHYSICAL direction needs live AILERON_TEST)");
  }
  {
    AeroState st; st.u = 21.244; st.deltaR = 5.0 * M_PI / 180.0;
    AeroOutput out = ComputeAero(cfg, st);
    Info("RUDDER_YAW_SIGN (algebraic only)",
         "delta_r=+5deg -> Mz=" + std::to_string(out.momentBody.Z()) +
         " (Cndr=" + std::to_string(cfg.Cndr) + "; PHYSICAL direction needs live RUDDER_TEST)");
  }
  {
    AeroState st; st.u = 21.244; st.deltaE = 5.0 * M_PI / 180.0;
    AeroOutput out = ComputeAero(cfg, st);
    Info("ELEVATOR_PITCH_SIGN (algebraic only)",
         "delta_e=+5deg -> My=" + std::to_string(out.momentBody.Y()) +
         " (Cmde=" + std::to_string(cfg.Cmde) + "; Cmde is now part of the "
         "negated 'static' group in the Cm-to-My fix this pass, so this "
         "measured sign is FLIPPED relative to any pre-fix measurement - "
         "expected, not a regression. PHYSICAL direction still needs live "
         "ELEVATOR_TEST, independent of this axis-handedness fix)");
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
    const double clExpected = cfg.Clp * pHatExpected;
    bool ok = std::abs(out.Cl - clExpected) < 1e-12;
    Check("RATE_NORMALIZATION_TEST (p_hat = p*b/2V)", ok,
          "Cl=" + std::to_string(out.Cl) + " expected=" + std::to_string(clExpected));

    // CLq*qHat regression check (MAJOR finding, fixed this pass - previously
    // cfg.CLq was loaded but never referenced in the CL build-up).
    AeroState stQ; stQ.u = V; stQ.q = 1.0;
    AeroOutput outQ = ComputeAero(cfg, stQ);
    AeroState stQ0; stQ0.u = V;
    AeroOutput outQ0 = ComputeAero(cfg, stQ0);
    const double qHatExpected = 1.0 * cfg.c_ref / (2.0 * V);
    const double clqContribExpected = cfg.CLq * qHatExpected;
    const double clqContribActual = outQ.CL - outQ0.CL;
    bool okClq = std::abs(clqContribActual - clqContribExpected) < 1e-12;
    Check("RATE_NORMALIZATION_TEST (CLq*q_hat now included in CL)", okClq,
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
  // DRAG_POLAR_TEST
  // -----------------------------------------------------------------------
  {
    const double alphaTrim = 0.36455 * M_PI / 180.0;
    AeroState st; st.u = 21.244 * std::cos(alphaTrim); st.w = -21.244 * std::sin(alphaTrim);
    AeroOutput out = ComputeAero(cfg, st);
    const double cdExpected = cfg.CD0 + cfg.dragK * out.CL * out.CL;
    bool ok = std::abs(out.CD - cdExpected) < 1e-12 && std::abs(out.CL - 0.471685) < 2e-3;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "trim: CL=%.6f (expect ~0.4717) CD=%.6f (CD0+k*CL^2=%.6f)",
                  out.CL, out.CD, cdExpected);
    Check("DRAG_POLAR_TEST + TRIM_BENCHMARK (V=21.244, neutral)", ok, buf);
  }

  // -----------------------------------------------------------------------
  // HIGH_ALPHA_LIMITER_TEST
  // -----------------------------------------------------------------------
  {
    bool ok = true;
    char buf[512]; buf[0] = 0;
    auto append = [&](const char *s) { std::snprintf(buf + std::strlen(buf), sizeof(buf) - std::strlen(buf), "%s", s); };
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

  std::printf("\n=================================================================\n");
  std::printf("SUMMARY: %d PASS, %d FAIL, %d INFO (INFO items require live-Gazebo\n"
              "confirmation, not pass/fail at the pure-math level)\n", gPass, gFail, gInfo);
  std::printf("=================================================================\n");

  return gFail == 0 ? 0 : 1;
}

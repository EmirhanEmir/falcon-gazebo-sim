// =============================================================================
// FALCON V2 - Propulsion V1 standalone, Gazebo-independent self-test
// =============================================================================
// Owner: propulsion specialist agent. Task: PROPULSION_V1_IMPLEMENTATION
// (2026-08-23).
//
// Exercises the PURE-MATH core (PropulsionModel.hh) directly - no Gazebo
// instance, no ECM, no plugin loading. Mirrors
// plugins/aerodynamics/test/aero_model_selftest.cc's structure/style. This
// is NOT a substitute for gazebo-testing's live-Gazebo suite
// (PROP_SPINUP_TEST, LEFT_MOTOR_TEST/RIGHT_MOTOR_TEST,
// COUNTER_ROTATION_CANCELLATION_TEST, etc. - tests/gazebo/README.md sec 3)
// which must still confirm the ECM/joint/AddWorldForce plumbing is correct -
// this only confirms the underlying formulas and the parsed APC data table
// are internally consistent and numerically sane.
//
// Build: see ../CMakeLists.txt (target propulsion_model_selftest, only
// depends on the C++ standard library - no gz-sim/gz-math/Gazebo runtime).
// =============================================================================
#include <cmath>
#include <cstdio>
#include <string>

#include "../PropulsionModel.hh"

using namespace falcon_v2_propulsion;

namespace
{
int gPass = 0;
int gFail = 0;

void Check(const std::string &name, bool ok, const std::string &detail)
{
  if (ok) { ++gPass; std::printf("[PASS] %-40s %s\n", name.c_str(), detail.c_str()); }
  else    { ++gFail; std::printf("[FAIL] %-40s %s\n", name.c_str(), detail.c_str()); }
}

bool IsFinite(double v) { return std::isfinite(v); }

// APC data path, relative to this repository's fixed layout (matches the
// absolute-path convention already used by model.sdf's aero <config_path>
// and tests/gazebo/scripts/*.py's WORLD_SDF constants - a documented, known
// V1 portability limitation, not a silent one).
const std::string kApcCsvPath =
    "/home/emirhan/Desktop/FalconV2/docs/source_of_truth/propulsion/data/"
    "apc_13x65e_parsed.csv";

// V1 propulsion config, mirroring propulsion_v1_config.yaml's numeric
// values exactly (same duplication pattern as aero_model_selftest.cc's
// MakeFalconV2Config() - the authoritative single source of truth remains
// the YAML file, which the real plugin actually loads at runtime).
MotorConfig MakeMotorConfig()
{
  MotorConfig m;
  m.kvRpmPerVolt = 860.0;             // CONFIRMED, PROPULSION.md sec 1.1
  m.resistanceOhm = 0.0258;           // CONFIRMED
  m.noLoadCurrentA = 1.3;             // CONFIRMED (10V reference)
  m.noLoadCurrentRefVoltage = 10.0;
  m.currentLimitA = 65.0;             // CONFIRMED rating, used as V1 hard clamp
  m.powerLimitW = 960.0;
  return m;
}

const double kD = 0.3302;   // CONFIRMED, APC 13x6.5E nominal - NEVER the STL mesh figure
const double kRho = 1.225;  // CONFIRMED, ISA sea level
const double kNSafeFloor = 1.0e-3;
const double kRpmCap = 11500.0;
const double kIRotor = 2.7349e-4;  // V1_PROVISIONAL_ROTATIONAL_INERTIA, prop-only

}  // namespace

int main()
{
  std::printf("=================================================================\n");
  std::printf("FALCON V2 propulsion standalone self-test (PropulsionModel.hh only)\n");
  std::printf("=================================================================\n\n");

  // -----------------------------------------------------------------------
  // 1. KV_TO_KT_KE_TEST
  // -----------------------------------------------------------------------
  {
    const double kv = 860.0;
    const double kt = MotorKt(kv);
    const double ktExpected = 60.0 / (2.0 * M_PI * kv);  // = 0.0111022... N*m/A
    bool ok = std::abs(kt - ktExpected) < 1e-12 && kt > 0.0 && kt < 1.0;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "KV=%.1f rpm/V -> Kt=Ke=%.8f N*m/A (=V/(rad/s)) [expected %.8f]",
        kv, kt, ktExpected);
    Check("KV_TO_KT_KE_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 2. APC_DATA_PARSE_TEST
  // -----------------------------------------------------------------------
  ApcTable table;
  {
    std::string err;
    bool loaded = LoadApcTableCsv(kApcCsvPath, table, &err);
    std::size_t totalRows = 0;
    for (const auto &s : table.slices) totalRows += s.J.size();
    bool ok = loaded && table.slices.size() == 18 &&
              table.MinRpm() == 1000.0 && table.MaxRpm() == 18000.0 &&
              totalRows == 537;  // 540 nominal rows - 3 truncated trailing rows dropped (see parse_apc_dat.py)
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "loaded=%d slices=%zu totalRows=%zu minRpm=%.0f maxRpm=%.0f err='%s'",
        loaded ? 1 : 0, table.slices.size(), totalRows, table.MinRpm(),
        table.MaxRpm(), err.c_str());
    Check("APC_DATA_PARSE_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 3-6. APC_STATIC_{5000,6000,9000,10000}_TEST
  // -----------------------------------------------------------------------
  // At J=0 (V=0 mph, static bench condition), these are EXACT tabulated APC
  // rows (independently verified by the coordinator against the raw
  // PER3_13x65E.dat file before this task began). LookupCtCp() at an exact
  // tabulated RPM/J point must reproduce Ct/Cp to near-machine precision (no
  // interpolation error at an exact grid point).
  //
  // Thrust reconstruction (T = Ct*rho*n^2*D^4, with the project-mandated
  // D=0.3302 m and rho=1.225 kg/m^3) is checked SEPARATELY, with a wider,
  // explicitly-documented tolerance - see the FINDING printed below. All
  // four static points show the SAME small systematic offset (T_calc/T_ref
  // ~1.015, P_calc/P_ref ~1.019), and the two ratios are self-consistent
  // with a ~0.4% smaller effective diameter (T scales D^4, P scales D^5:
  // 1.015^(5/4)=1.0185, matching the observed P ratio). This is NOT a bug in
  // this project's interpolation/formula code - it indicates APC's own
  // internal performance-sim tool for THIS file used a slightly different
  // effective diameter and/or air density than this project's mandated
  // nominal D=0.3302 m / rho=1.225 kg/m^3 when it originally derived Ct/Cp
  // from its own raw T/P. Per CLAUDE.md/PROPULSION.md sec 0 ("D=0.3302 m
  // ALWAYS, no exceptions") this project does NOT adjust D to close this
  // gap - the offset is reported here, not silently absorbed. See this
  // task's final report for the full finding.
  struct StaticCase { double rpm, ctRef, cpRef, thrustRef; };
  const StaticCase staticCases[] = {
      {5000.0, 0.0901, 0.0309, 8.975},
      {6000.0, 0.0904, 0.0306, 12.975},
      {9000.0, 0.0919, 0.0303, 29.664},
      {10000.0, 0.0925, 0.0304, 36.877},
  };
  for (const auto &sc : staticCases)
  {
    const CtCpResult r = LookupCtCp(table, sc.rpm, 0.0);
    const double n = sc.rpm / 60.0;
    const double thrustCalc = PropThrust(r.Ct, kRho, n, kD);
    const double ratio = thrustCalc / sc.thrustRef;

    const bool ctCpExact = std::abs(r.Ct - sc.ctRef) < 1e-9 &&
                            std::abs(r.Cp - sc.cpRef) < 1e-9 && !r.interpClamped;
    // Documented systematic-offset tolerance (see comment block above) - not
    // a numerical-precision tolerance.
    const bool thrustInBand = ratio > 0.97 && ratio < 1.03;

    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "Ct=%.4f(ref %.4f) Cp=%.4f(ref %.4f) T_calc=%.4fN T_ref=%.4fN ratio=%.4f",
        r.Ct, sc.ctRef, r.Cp, sc.cpRef, thrustCalc, sc.thrustRef, ratio);
    Check("APC_STATIC_" + std::to_string(static_cast<int>(sc.rpm)) + "_TEST",
          ctCpExact && thrustInBand, buf);
  }

  // -----------------------------------------------------------------------
  // 7. APC_FORWARD_FLIGHT_INTERPOLATION_TEST
  // -----------------------------------------------------------------------
  // Uses REAL raw APC rows at RPM=6000 (read directly from
  // apc_13x65e_parsed.csv, not the task prompt's rounded example numbers):
  //   J=0.0667 -> Ct=0.0857, Cp=0.0315
  //   J=0.0889 -> Ct=0.0839, Cp=0.0317
  // Query at the exact midpoint J=0.0778 (pure within-slice interpolation,
  // no RPM-axis blending needed since the query RPM equals a real slice).
  {
    const double jLo = 0.0667, ctLo = 0.0857, cpLo = 0.0315;
    const double jHi = 0.0889, ctHi = 0.0839, cpHi = 0.0317;
    const double jQuery = 0.5 * (jLo + jHi);
    const double ctExpected = 0.5 * (ctLo + ctHi);
    const double cpExpected = 0.5 * (cpLo + cpHi);

    const CtCpResult r = LookupCtCp(table, 6000.0, jQuery);
    bool ok = std::abs(r.Ct - ctExpected) < 1e-9 &&
              std::abs(r.Cp - cpExpected) < 1e-9 && !r.interpClamped;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "RPM=6000 J=%.4f -> Ct=%.6f(expected %.6f) Cp=%.6f(expected %.6f)",
        jQuery, r.Ct, ctExpected, r.Cp, cpExpected);
    Check("APC_FORWARD_FLIGHT_INTERPOLATION_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 8. J_CALCULATION_TEST
  // -----------------------------------------------------------------------
  {
    const double vAxial = 15.0, n = 100.0;  // rev/s (well above the safety floor)
    const double j = AdvanceRatio(vAxial, n, kD, kNSafeFloor);
    const double jExpected = vAxial / (n * kD);
    bool ok = std::abs(j - jExpected) < 1e-12;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "V=%.1f n=%.1f D=%.4f -> J=%.6f (expected %.6f)",
                  vAxial, n, kD, j, jExpected);
    Check("J_CALCULATION_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 9. PROP_POWER_TORQUE_TEST (formula self-consistency: P=Cp*rho*n^3*D^5,
  // Q=P/omega must satisfy Q*omega=P exactly - deliberately NOT compared
  // against the file's own W/N-m columns here, since that comparison
  // inherits the same documented systematic diameter/rho offset already
  // isolated and reported in the APC_STATIC_* tests above; this test
  // isolates pure formula correctness instead)
  // -----------------------------------------------------------------------
  {
    const CtCpResult r = LookupCtCp(table, 9000.0, 0.0);
    const double n = 9000.0 / 60.0;
    const double omega = 2.0 * M_PI * n;
    const double pProp = PropPower(r.Cp, kRho, n, kD);
    const double qProp = PropTorqueFromPower(pProp, omega, 2.0 * M_PI * kNSafeFloor);
    const double pBack = qProp * omega;
    bool ok = IsFinite(pProp) && IsFinite(qProp) && pProp > 0.0 && qProp > 0.0 &&
              std::abs(pBack - pProp) < 1e-9 * std::max(1.0, pProp);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "RPM=9000 Cp=%.4f -> P_prop=%.4fW Q_prop=%.6fN*m Q*omega=%.4fW(=P?)",
        r.Cp, pProp, qProp, pBack);
    Check("PROP_POWER_TORQUE_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 10. ZERO_RPM_NUMERICAL_TEST
  // -----------------------------------------------------------------------
  {
    // omega=0, vAxial=0 (motor at rest, no airflow) - the baseline zero case.
    const PropLoadResult r0 = PropAeroLoad(table, 0.0, 0.0, kD, kRho, kNSafeFloor);
    bool ok0 = IsFinite(r0.J) && IsFinite(r0.Ct) && IsFinite(r0.Cp) &&
               IsFinite(r0.thrust_N) && IsFinite(r0.pProp_W) &&
               IsFinite(r0.qPropSigned_Nm) && r0.thrust_N == 0.0 && r0.pProp_W == 0.0 &&
               r0.qPropSigned_Nm == 0.0;

    // omega=0 but vAxial>0 (stopped prop with axial airflow - e.g. an
    // engine-out prop at the instant it has fully stopped while the
    // aircraft still has forward speed): J denominator would be an exact
    // 0/0 without the nSafeFloor - must stay finite, and T/P must still
    // collapse to (numerically) zero via the n^2/n^3 factors regardless of
    // the resulting large-but-finite J (PropulsionModel.hh AdvanceRatio doc).
    const PropLoadResult r1 = PropAeroLoad(table, 0.0, 20.0, kD, kRho, kNSafeFloor);
    bool ok1 = IsFinite(r1.J) && IsFinite(r1.Ct) && IsFinite(r1.Cp) &&
               IsFinite(r1.thrust_N) && IsFinite(r1.pProp_W) &&
               IsFinite(r1.qPropSigned_Nm) && r1.thrust_N == 0.0 && r1.pProp_W == 0.0;

    // Motor electrical at throttle=0, omega=0: no divide-by-zero, finite.
    const MotorElectricalResult e = MotorElectrical(MakeMotorConfig(), 0.0, 14.8, 0.0);
    bool ok2 = IsFinite(e.current_A) && IsFinite(e.torque_Nm);

    // Rotor ODE step at rest with zero drive: stays finite, no blow-up.
    const RotorStepResult s = IntegrateRotorStep(0.0, e.torque_Nm, 0.0, kIRotor, 0.01, kRpmCap);
    bool ok3 = IsFinite(s.omega) && IsFinite(s.rpm);

    bool ok = ok0 && ok1 && ok2 && ok3;
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "omega=0,V=0: J=%.3f T=%.3f P=%.3f | omega=0,V=20: J=%.3f(large-finite ok) T=%.6f P=%.6f | "
        "throttle=0 idle: I=%.4fA Q_motor=%.6fN*m -> omega_step=%.6f rad/s (all finite, no NaN/Inf)",
        r0.J, r0.thrust_N, r0.pProp_W, r1.J, r1.thrust_N, r1.pProp_W,
        e.current_A, e.torque_Nm, s.omega);
    Check("ZERO_RPM_NUMERICAL_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // 11. RPM_LIMIT_MATH_TEST
  // -----------------------------------------------------------------------
  {
    const double omegaCap = kRpmCap * 2.0 * M_PI / 60.0;

    const RpmCapResult belowCap = ApplyRpmCap(omegaCap * 0.5, kRpmCap);
    const RpmCapResult atCapExactly = ApplyRpmCap(omegaCap, kRpmCap);
    const RpmCapResult aboveCap = ApplyRpmCap(omegaCap * 1.5, kRpmCap);
    const RpmCapResult belowNegCap = ApplyRpmCap(-omegaCap * 1.5, kRpmCap);

    bool ok = !belowCap.capActive && std::abs(belowCap.omega - omegaCap * 0.5) < 1e-9 &&
              !atCapExactly.capActive &&
              aboveCap.capActive && std::abs(aboveCap.omega - omegaCap) < 1e-9 &&
              belowNegCap.capActive && std::abs(belowNegCap.omega + omegaCap) < 1e-9;

    // Also confirm IntegrateRotorStep saturates: a huge Q_motor over a
    // normal dt should not push |RPM| past the cap.
    const RotorStepResult step = IntegrateRotorStep(
        omegaCap * 0.99, /*qMotor*/ 1000.0, /*qPropSigned*/ 0.0, kIRotor, 0.01, kRpmCap);
    bool okStep = step.rpmCapActive && std::abs(step.omega - omegaCap) < 1e-9 &&
                  std::abs(step.rpm - kRpmCap) < 1e-6;

    ok = ok && okStep;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "cap=%.0f RPM (omegaCap=%.4f rad/s): below=%s at=%s above(capped,omega=%.4f)=%s "
        "step-saturation: rpm=%.2f capActive=%d",
        kRpmCap, omegaCap, belowCap.capActive ? "CAP" : "ok",
        atCapExactly.capActive ? "CAP" : "ok", aboveCap.omega,
        aboveCap.capActive ? "CAP" : "ok", step.rpm, step.rpmCapActive ? 1 : 0);
    Check("RPM_LIMIT_MATH_TEST", ok, buf);
  }

  std::printf("\n=================================================================\n");
  std::printf("SUMMARY: %d PASS, %d FAIL\n", gPass, gFail);
  std::printf("=================================================================\n");

  return gFail == 0 ? 0 : 1;
}

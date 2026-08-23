// =============================================================================
// FALCON V2 - Propulsion V1 core math model
// =============================================================================
// Owner: propulsion specialist agent. Task: PROPULSION_V1_IMPLEMENTATION
// (2026-08-23).
//
// This header contains ONLY pure math (plus trivial CSV table loading via
// <fstream>, which is standard-library-only and carries no gz-sim/ECM/Entity
// dependency - the same "linkable into both the real plugin and a standalone
// self-test" design used by plugins/aerodynamics/AeroModel.hh, mirrored here
// intentionally). No gz-sim dependency anywhere in this file.
//
// Required chain implemented here (never bypassed, never collapsed into
// throttle*maximum_thrust - see CLAUDE.md / PROPULSION.md):
//   throttle -> ESC/motor electrical -> motor torque -> rotor angular
//   dynamics (explicit-Euler state, integrated by the CALLER every tick,
//   never instantaneous) -> RPM -> advance ratio J -> APC Ct(J,RPM)/Cp(J,RPM)
//   -> thrust + propeller aerodynamic torque -> reaction torque
//
// Every constant this header's CALLER must supply (Kv, R, I0, D, rho,
// I_rotor, RPM cap, ...) is loaded from
// docs/source_of_truth/propulsion/propulsion_v1_config.yaml by
// PropulsionSystem.cc's Configure() step (mirrors AeroModel.hh's AeroConfig
// pattern) - no coefficient is hardcoded in this header or in
// PropulsionSystem.cc.
//
// APC 13x6.5E Ct(J)/Cp(J) DATA: loaded at runtime from
// docs/source_of_truth/propulsion/data/apc_13x65e_parsed.csv, itself
// mechanically generated (never hand-edited) from the official APC
// performance-data file docs/source_of_truth/propulsion/data/
// PER3_13x65E.dat by docs/source_of_truth/propulsion/data/parse_apc_dat.py.
// See that script's header comment and
// docs/source_of_truth/propulsion/data/PROVENANCE.md for the full raw ->
// parsed data lineage. No Ct/Cp value in this project is invented - every
// value returned by LookupCtCp() below is either a real tabulated APC value
// or a linear interpolation between two real tabulated APC values, exactly
// as specified by PROPULSION.md.
//
// PROPELLER DIAMETER: D = 0.3302 m ALWAYS (APC 13x6.5E manufacturer nominal),
// never the ~0.2734 m STL mesh figure - PROPULSION.md sec 0. This header
// never reads mesh geometry; D is a config value supplied by the caller.
//
// ROTATION-SIGN CONVENTION used throughout this header (see
// propulsion_v1_config.yaml "rotation" block for the full provenance):
// omega (rad/s) passed into every function below is expressed in EACH
// MOTOR'S OWN "positive = normal spin-up direction" frame - i.e. omega>0
// always means "this motor spinning up under positive throttle", for BOTH
// the left and right motor, regardless of which physical world/body +X
// sense that corresponds to. This deliberately keeps the electrical/ODE/
// aero-loading math IDENTICAL in form for both motors (Kt/Ke/Ct/Cp do not
// change sign based on mounting side - only the mechanical coupling to the
// shared body +X axis does). The mapping from this per-motor "own frame" to
// the single shared body +X axis (for driving the SDF joint's angular
// velocity and for the sign of the reaction torque applied to base_link) is
// an explicit, separate, config-driven sign (rotation.left_sign /
// rotation.right_sign) applied ONLY in PropulsionSystem.cc - never inside
// this header. See propulsion_v1_config.yaml for why this mapping is
// ASSUMPTION-tagged (the master dataset's "Left=CCW,Right=CW" statement
// does not specify a viewing frame).
// =============================================================================
#ifndef FALCON_V2_PROPULSION_MODEL_HH_
#define FALCON_V2_PROPULSION_MODEL_HH_

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace falcon_v2_propulsion
{

// ===========================================================================
// Config structs - populated by the caller (PropulsionSystem.cc's
// Configure(), or a standalone self-test) from
// propulsion_v1_config.yaml / the parsed APC CSV. No default value below is
// meant to be trusted for a real run, mirroring AeroConfig's own comment -
// defaults exist only so a missing config key fails loudly/zero rather than
// reading uninitialized memory.
// ===========================================================================

/// \brief Motor + ESC electrical parameters (PROPULSION.md sec 1.1/1.2).
struct MotorConfig
{
  double kvRpmPerVolt = 0.0;     // CONFIRMED, manufacturer nameplate
  double resistanceOhm = 0.0;    // CONFIRMED, R (motor internal resistance)
  double noLoadCurrentA = 0.0;   // CONFIRMED, I0 (reference point, see below)
  double noLoadCurrentRefVoltage = 10.0;  // CONFIRMED context - I0 was measured at this voltage
  double currentLimitA = 0.0;    // CONFIRMED rating (~65A/30s) - used as V1 hard clamp, ASSUMPTION-tagged (see propulsion_v1_config.yaml)
  double powerLimitW = 0.0;      // CONFIRMED rating (~960W) - diagnostic only in V1, not a hard clamp
};

/// \brief Battery (PROPULSION.md sec 1.3). V1 fixed-voltage model - no sag.
struct BatteryConfig
{
  double vNominal = 0.0;  // CONFIRMED, V1 operating voltage (fixed, no SOC/sag model)
};

/// \brief Propeller physical parameters (PROPULSION.md sec 0/1.4).
struct PropellerConfig
{
  double diameterM = 0.0;  // CONFIRMED, D=0.3302 m always - NEVER the STL mesh figure
};

/// \brief Physics / numerical parameters.
struct PhysicsConfig
{
  double rho = 1.225;              // kg/m^3, CONFIRMED ISA sea-level, V1 fixed config parameter
  double rotorInertiaKgM2 = 0.0;   // I_rotor, V1_PROVISIONAL_ROTATIONAL_INERTIA (see propulsion_v1_config.yaml)
  double nSafeFloorRevS = 1.0e-3;  // TEMPORARY numerical-safety floor for the J denominator only (mirrors AeroModel.hh vSafeFloor)
  double rpmCap = 0.0;             // V1 hard operating cap on |RPM| (saturates the omega integrator)
};

/// \brief One RPM slice of the parsed APC performance table: for a fixed
/// RPM, Ct/Cp as a function of J, both arrays STRICTLY increasing in J
/// (verified at load time, see LoadApcTableCsv()).
struct ApcRpmSlice
{
  double rpm = 0.0;
  std::vector<double> J;
  std::vector<double> Ct;
  std::vector<double> Cp;
};

/// \brief Full parsed APC 13x6.5E performance table, slices sorted
/// ascending by rpm. Loaded from apc_13x65e_parsed.csv - see file header.
struct ApcTable
{
  std::vector<ApcRpmSlice> slices;

  double MinRpm() const { return slices.empty() ? 0.0 : slices.front().rpm; }
  double MaxRpm() const { return slices.empty() ? 0.0 : slices.back().rpm; }
};

/// \brief Load the mechanically-parsed APC CSV (rpm,V_mph,J,Ct,Cp,power_W,
/// torque_Nm,thrust_N) into an ApcTable. Groups rows by rpm (preserving
/// file order within a slice, which the parser already guarantees is
/// strictly increasing in J), sorts slices ascending by rpm. Returns false
/// (with a message in errorOut, if given) on any I/O or parse failure, or if
/// a slice's J array is found not to be strictly increasing (a load-time
/// sanity check protecting the interpolation logic below, which assumes
/// monotonic J per slice - PROPULSION.md's own specified interpolation
/// method).
inline bool LoadApcTableCsv(const std::string &path, ApcTable &table,
                             std::string *errorOut = nullptr)
{
  table.slices.clear();
  std::ifstream f(path);
  if (!f.is_open())
  {
    if (errorOut) *errorOut = "could not open '" + path + "'";
    return false;
  }

  std::string line;
  if (!std::getline(f, line))
  {
    if (errorOut) *errorOut = "empty file '" + path + "'";
    return false;
  }
  // line 0 is the header row (rpm,V_mph,J,Ct,Cp,power_W,torque_Nm,thrust_N) - skip.

  // Preserve insertion order of first-seen RPM values, group rows.
  std::vector<double> sliceOrder;
  struct RawSlice { std::vector<double> J, Ct, Cp; };
  std::vector<RawSlice> rawSlices;
  auto findOrAdd = [&](double rpm) -> RawSlice &
  {
    for (std::size_t i = 0; i < sliceOrder.size(); ++i)
    {
      if (sliceOrder[i] == rpm) return rawSlices[i];
    }
    sliceOrder.push_back(rpm);
    rawSlices.emplace_back();
    return rawSlices.back();
  };

  std::size_t lineNo = 1;
  while (std::getline(f, line))
  {
    ++lineNo;
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ','))
    {
      try { vals.push_back(std::stod(tok)); }
      catch (const std::exception &)
      {
        if (errorOut) *errorOut = "parse error at CSV line " + std::to_string(lineNo);
        return false;
      }
    }
    if (vals.size() != 8)
    {
      if (errorOut) *errorOut = "malformed CSV row (expected 8 fields) at line " + std::to_string(lineNo);
      return false;
    }
    const double rpm = vals[0];
    // vals[1]=V_mph (unused here), vals[2]=J, vals[3]=Ct, vals[4]=Cp,
    // vals[5..7]=power_W/torque_Nm/thrust_N (unused for the interpolation
    // table itself - retained in the CSV for cross-check/self-test use, not
    // re-parsed into ApcTable, which only needs Ct(J)/Cp(J) per PROPULSION.md).
    RawSlice &rs = findOrAdd(rpm);
    rs.J.push_back(vals[2]);
    rs.Ct.push_back(vals[3]);
    rs.Cp.push_back(vals[4]);
  }

  for (std::size_t i = 0; i < sliceOrder.size(); ++i)
  {
    const RawSlice &rs = rawSlices[i];
    for (std::size_t j = 1; j < rs.J.size(); ++j)
    {
      if (!(rs.J[j] > rs.J[j - 1]))
      {
        if (errorOut)
        {
          *errorOut = "non-monotonic J within RPM slice " +
                      std::to_string(sliceOrder[i]) + " at index " + std::to_string(j);
        }
        return false;
      }
    }
    ApcRpmSlice slice;
    slice.rpm = sliceOrder[i];
    slice.J = rs.J;
    slice.Ct = rs.Ct;
    slice.Cp = rs.Cp;
    table.slices.push_back(slice);
  }

  std::sort(table.slices.begin(), table.slices.end(),
            [](const ApcRpmSlice &a, const ApcRpmSlice &b) { return a.rpm < b.rpm; });

  if (table.slices.empty())
  {
    if (errorOut) *errorOut = "no data rows parsed from '" + path + "'";
    return false;
  }
  return true;
}

// ===========================================================================
// 1. ESC/motor electrical (PROPULSION.md sec 2, sec 52 equation block)
// ===========================================================================

/// \brief Motor torque constant Kt = 60/(2*pi*KV), KV in rpm/V, Kt in N*m/A.
/// Ke (back-EMF constant, V/(rad/s)) is numerically identical to Kt in SI
/// units (PROPULSION.md sec 2) - callers use this same value for both.
inline double MotorKt(double kvRpmPerVolt)
{
  if (kvRpmPerVolt <= 0.0) return 0.0;  // defensive only - a valid KV is always > 0
  return 60.0 / (2.0 * M_PI * kvRpmPerVolt);
}

/// \brief Result of one motor-electrical evaluation at a given throttle/
/// battery-voltage/instantaneous-omega operating point.
struct MotorElectricalResult
{
  double current_A = 0.0;      // I_used, after the two clamps below
  double torque_Nm = 0.0;      // Q_motor = Kt*(I_used - I0_effective)
  bool negativeCurrentClamped = false;  // true if the raw back-EMF-driven current was < 0 (regen not modeled - see header note)
  bool currentLimited = false;          // true if I_used was clamped at MotorConfig::currentLimitA
};

/// \brief ESC + motor electrical model, PROPULSION.md sec 2/52:
///   V_ESC = throttle * V_battery
///   I     = (V_ESC - Ke*omega) / R
///   Q_motor = Kt * (I - I0_effective)
///
/// Two explicit, documented V1 policies (propulsion_v1_config.yaml
/// "electrical_policy" block, PROPULSION.md sec 1.2/8.2 item 4 - both
/// explicitly deferred/undocumented beyond this V1 simplification):
///  (a) NEGATIVE-CURRENT CLAMP: if the raw back-EMF-driven current would be
///      negative (back-EMF Ke*omega exceeds V_ESC - e.g. a windmilling prop
///      under an engine-out condition, or throttle=0 with omega>0), it is
///      clamped to 0 rather than modeled as regenerative/reverse current
///      flowing back into the battery. ASSUMPTION: simple non-regenerative
///      ESC (no battery-regen model exists anywhere in the source of truth -
///      fabricating one would violate CLAUDE.md's no-invented-physics rule).
///  (b) I0_EFFECTIVE: I0 is only characterized at ONE reference voltage
///      (10 V, PROPULSION.md sec 1.1). No voltage-scaling law for I0 exists
///      in the source of truth, so V1 uses I0_effective = I0 (constant),
///      ASSUMPTION-tagged. A true voltage-dependent I0 curve is
///      DATA_REQUIRED.
/// Current-limit clamp: I_used is additionally clamped at
/// MotorConfig::currentLimitA (the ~65A/30s manufacturer rating,
/// PROPULSION.md sec 1.1) - ASSUMPTION: this V1 model applies the rating
/// directly as an instantaneous hard clamp, not a true duration-limited
/// (I^2*t) thermal model, since no such model exists in the source of truth.
/// throttle01 is clamped to [0,1] here (defensive - the transport layer
/// should already enforce this, PropulsionSystem.cc).
inline MotorElectricalResult MotorElectrical(
    const MotorConfig &cfg, double throttle01, double vBattery, double omega)
{
  MotorElectricalResult out;
  const double throttle = std::clamp(throttle01, 0.0, 1.0);
  const double kt = MotorKt(cfg.kvRpmPerVolt);
  const double ke = kt;  // SI numeric equivalence, PROPULSION.md sec 2

  const double vEsc = throttle * vBattery;
  double iRaw = 0.0;
  if (cfg.resistanceOhm > 0.0)
  {
    iRaw = (vEsc - ke * omega) / cfg.resistanceOhm;
  }

  double iUsed = iRaw;
  if (iUsed < 0.0)
  {
    iUsed = 0.0;
    out.negativeCurrentClamped = true;
  }
  if (cfg.currentLimitA > 0.0 && iUsed > cfg.currentLimitA)
  {
    iUsed = cfg.currentLimitA;
    out.currentLimited = true;
  }

  out.current_A = iUsed;
  out.torque_Nm = kt * (iUsed - cfg.noLoadCurrentA);
  return out;
}

// ===========================================================================
// 2. RPM safety cap (PROPULSION.md sec 1.7)
// ===========================================================================

struct RpmCapResult
{
  double omega = 0.0;
  bool capActive = false;
};

/// \brief Saturates omega (rad/s, signed - see header rotation-sign note) so
/// that |RPM| never exceeds rpmCap. This is the V1 hard numerical cap
/// (PROPULSION.md sec 1.7: distinct from, and independent of, both the
/// theoretical no-load-RPM figure and the APC table's own [1000,18000]
/// lookup-clamp range below).
inline RpmCapResult ApplyRpmCap(double omega, double rpmCap)
{
  RpmCapResult out;
  out.omega = omega;
  if (rpmCap <= 0.0) return out;  // defensive: cap disabled if non-positive
  const double omegaCap = rpmCap * 2.0 * M_PI / 60.0;
  if (out.omega > omegaCap)
  {
    out.omega = omegaCap;
    out.capActive = true;
  }
  else if (out.omega < -omegaCap)
  {
    out.omega = -omegaCap;
    out.capActive = true;
  }
  return out;
}

// ===========================================================================
// 3. Rotor angular dynamics ODE step (PROPULSION.md sec 2/4/53)
// ===========================================================================

struct RotorStepResult
{
  double omega = 0.0;   // rad/s, new state after this step
  double rpm = 0.0;      // derived diagnostic
  bool rpmCapActive = false;
};

/// \brief One explicit-Euler integration step of
///   I_rotor * domega/dt = Q_motor - Q_propSigned
/// qPropSigned must already be signed to OPPOSE omegaPrev (see PropAeroLoad
/// below, which produces exactly this signed quantity) - this function does
/// not itself infer a sign. RPM is never set instantaneously: this is the
/// state-integration step every PreUpdate tick must call, using the real
/// per-tick dt from the ECM's UpdateInfo (PropulsionSystem.cc).
inline RotorStepResult IntegrateRotorStep(
    double omegaPrev, double qMotor, double qPropSigned, double iRotor,
    double dt, double rpmCap)
{
  RotorStepResult out;
  double omegaNew = omegaPrev;
  if (iRotor > 0.0)
  {
    omegaNew = omegaPrev + dt * (qMotor - qPropSigned) / iRotor;
  }
  const RpmCapResult capped = ApplyRpmCap(omegaNew, rpmCap);
  out.omega = capped.omega;
  out.rpmCapActive = capped.capActive;
  out.rpm = out.omega * 60.0 / (2.0 * M_PI);
  return out;
}

// ===========================================================================
// 4. Advance ratio (PROPULSION.md sec 2/6)
// ===========================================================================

/// \brief J = V_axial / (n*D), n in rev/s (ALWAYS >= 0 by construction - see
/// PropAeroLoad, which passes n=|omega|/(2*pi)). nSafeFloor (rev/s) prevents
/// an exact division by zero at n=0 (ZERO_RPM_NUMERICAL_TEST) - purely a
/// numerical-safety measure: T/P_prop naturally -> 0 as n->0 regardless of
/// the resulting (large-but-finite) J, since both formulas carry n^2/n^3
/// factors (PROPULSION.md sec 6, this task's item 6).
inline double AdvanceRatio(double vAxial, double n, double diameterM, double nSafeFloor)
{
  const double nSafe = std::max(n, nSafeFloor);
  if (diameterM <= 0.0) return 0.0;
  return vAxial / (nSafe * diameterM);
}

// ===========================================================================
// 5. 2D Ct/Cp interpolation over the parsed APC table (PROPULSION.md sec 5,
//    this task's item 6 - the exact 2-stage method specified there)
// ===========================================================================

struct CtCpResult
{
  double Ct = 0.0;
  double Cp = 0.0;
  bool interpClamped = false;  // true if RPM and/or J had to be clamped to the table's own range
};

namespace detail
{
/// \brief Linear interpolation of Ct/Cp within ONE RPM slice's own J array.
/// Clamps J to that slice's own [Jmin,Jmax] first (slices have different J
/// ranges - never assume a shared grid, per this task's explicit
/// instruction). Slice's J array is required strictly increasing (enforced
/// at load time by LoadApcTableCsv()).
inline void InterpWithinSlice(const ApcRpmSlice &slice, double J,
                               double &ctOut, double &cpOut, bool &jClamped)
{
  jClamped = false;
  if (slice.J.empty())
  {
    ctOut = 0.0;
    cpOut = 0.0;
    return;
  }
  if (slice.J.size() == 1)
  {
    ctOut = slice.Ct[0];
    cpOut = slice.Cp[0];
    return;
  }

  double jq = J;
  if (jq < slice.J.front())
  {
    jq = slice.J.front();
    jClamped = true;
  }
  else if (jq > slice.J.back())
  {
    jq = slice.J.back();
    jClamped = true;
  }

  // Find the bracketing pair [i-1, i] with J[i-1] <= jq <= J[i].
  std::size_t hi = 0;
  while (hi < slice.J.size() && slice.J[hi] < jq) ++hi;
  if (hi == 0)
  {
    ctOut = slice.Ct[0];
    cpOut = slice.Cp[0];
    return;
  }
  if (hi >= slice.J.size())
  {
    ctOut = slice.Ct.back();
    cpOut = slice.Cp.back();
    return;
  }
  const std::size_t lo = hi - 1;
  const double jLo = slice.J[lo], jHi = slice.J[hi];
  const double span = jHi - jLo;
  const double f = (span > 0.0) ? (jq - jLo) / span : 0.0;
  ctOut = slice.Ct[lo] + f * (slice.Ct[hi] - slice.Ct[lo]);
  cpOut = slice.Cp[lo] + f * (slice.Cp[hi] - slice.Cp[lo]);
}
}  // namespace detail

/// \brief Full 2-stage 2D interpolation exactly as specified by
/// PROPULSION.md / this task: (1) find the two bracketing RPM slices,
/// interpolate Ct/Cp within EACH using that slice's own J array/range; (2)
/// linearly blend the two per-slice results by RPM position. RPM is clamped
/// to the table's own [minRpm,maxRpm] range for lookup purposes if outside
/// (interpClamped exposed as a diagnostic) - a defensive edge case (e.g.
/// omega transiently overshooting during integration before the RPM cap
/// logic engages), distinct from and NOT required to coincide with the
/// operational RPM cap (PROPULSION.md sec 1.7).
inline CtCpResult LookupCtCp(const ApcTable &table, double rpmQuery, double J)
{
  CtCpResult out;
  if (table.slices.empty()) return out;

  double rpm = rpmQuery;
  if (rpm < table.MinRpm())
  {
    rpm = table.MinRpm();
    out.interpClamped = true;
  }
  else if (rpm > table.MaxRpm())
  {
    rpm = table.MaxRpm();
    out.interpClamped = true;
  }

  // Find bracketing slice indices [lo, hi] with slices[lo].rpm <= rpm <= slices[hi].rpm.
  std::size_t hi = 0;
  while (hi < table.slices.size() && table.slices[hi].rpm < rpm) ++hi;
  std::size_t lo;
  if (hi == 0) { lo = 0; hi = 0; }
  else if (hi >= table.slices.size()) { hi = table.slices.size() - 1; lo = hi; }
  else { lo = hi - 1; }

  double ctLo, cpLo, ctHi, cpHi;
  bool jClampedLo = false, jClampedHi = false;
  detail::InterpWithinSlice(table.slices[lo], J, ctLo, cpLo, jClampedLo);
  detail::InterpWithinSlice(table.slices[hi], J, ctHi, cpHi, jClampedHi);

  const double rpmLo = table.slices[lo].rpm, rpmHi = table.slices[hi].rpm;
  const double span = rpmHi - rpmLo;
  const double f = (span > 0.0) ? (rpm - rpmLo) / span : 0.0;

  out.Ct = ctLo + f * (ctHi - ctLo);
  out.Cp = cpLo + f * (cpHi - cpLo);
  out.interpClamped = out.interpClamped || jClampedLo || jClampedHi;
  return out;
}

// ===========================================================================
// 6. Thrust / power / torque formulas (PROPULSION.md sec 2/6, sec 54-56)
// ===========================================================================

/// \brief T = Ct * rho * n^2 * D^4. n in rev/s, ALWAYS >= 0 by construction
/// (magnitude - see PropAeroLoad). Thrust acts along the fixed thrust axis
/// (+X, geometry-structure's CONFIRMED value) regardless of rotation sign.
inline double PropThrust(double ct, double rho, double n, double diameterM)
{
  const double d2 = diameterM * diameterM;
  return ct * rho * n * n * (d2 * d2);
}

/// \brief P_prop = Cp * rho * n^3 * D^5.
inline double PropPower(double cp, double rho, double n, double diameterM)
{
  const double d2 = diameterM * diameterM;
  return cp * rho * n * n * n * (d2 * d2 * diameterM);
}

/// \brief Q_prop (torque MAGNITUDE) = P_prop / omega, omega safe-floored
/// (rad/s) to avoid division by exactly zero at omega=0 - P_prop itself
/// already -> 0 as n->0 (via the n^3 factor above) regardless of this floor,
/// so the floor is purely a numerical-safety measure here too, matching the
/// same reasoning as AdvanceRatio's nSafeFloor.
inline double PropTorqueFromPower(double pProp, double omegaAbs, double omegaSafeFloor)
{
  const double omegaSafe = std::max(omegaAbs, omegaSafeFloor);
  return pProp / omegaSafe;
}

/// \brief Full per-motor propeller aerodynamic-loading evaluation for one
/// timestep, combining AdvanceRatio + LookupCtCp + the three formulas above.
/// omega is this motor's own-frame signed state (see header rotation-sign
/// note) - n/J/Ct/Cp/thrust/pProp are all computed from |omega| (magnitude),
/// since the APC table only characterizes forward (positive-J) operation
/// and the thrust axis direction does not depend on rotation sign.
/// qPropSigned is the torque this header returns for direct use in
/// IntegrateRotorStep's ODE - signed to always OPPOSE omega (0 at omega=0).
struct PropLoadResult
{
  double n = 0.0;              // rev/s, |omega|/(2*pi)
  double rpm = 0.0;             // diagnostic, signed (same sign as omega)
  double J = 0.0;
  double Ct = 0.0;
  double Cp = 0.0;
  double thrust_N = 0.0;        // magnitude, along the fixed +X thrust axis
  double pProp_W = 0.0;
  double qPropMagnitude_Nm = 0.0;
  double qPropSigned_Nm = 0.0;   // opposes omega - the qProp term IntegrateRotorStep's ODE expects
  bool interpClamped = false;
};

inline PropLoadResult PropAeroLoad(
    const ApcTable &table, double omega, double vAxial, double diameterM,
    double rho, double nSafeFloorRevS)
{
  PropLoadResult out;
  const double omegaAbs = std::abs(omega);
  out.n = omegaAbs / (2.0 * M_PI);
  out.rpm = omega * 60.0 / (2.0 * M_PI);

  out.J = AdvanceRatio(vAxial, out.n, diameterM, nSafeFloorRevS);

  const double rpmQuery = out.n * 60.0;  // always >= 0; table-clamped inside LookupCtCp
  const CtCpResult ctcp = LookupCtCp(table, rpmQuery, out.J);
  out.Ct = ctcp.Ct;
  out.Cp = ctcp.Cp;
  out.interpClamped = ctcp.interpClamped;

  out.thrust_N = PropThrust(out.Ct, rho, out.n, diameterM);
  out.pProp_W = PropPower(out.Cp, rho, out.n, diameterM);

  const double omegaSafeFloor = 2.0 * M_PI * nSafeFloorRevS;
  out.qPropMagnitude_Nm = PropTorqueFromPower(out.pProp_W, omegaAbs, omegaSafeFloor);
  const double sign = (omega >= 0.0) ? 1.0 : -1.0;
  out.qPropSigned_Nm = sign * out.qPropMagnitude_Nm;

  return out;
}

}  // namespace falcon_v2_propulsion

#endif  // FALCON_V2_PROPULSION_MODEL_HH_

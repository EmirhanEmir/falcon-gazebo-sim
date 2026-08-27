// =============================================================================
// FALCON V2 - Wind/gust generator V1 standalone, Gazebo-independent self-test
// =============================================================================
// Owner: aerodynamics specialist agent. Task:
// WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION (2026-08-27).
//
// Exercises the PURE-MATH core (WindModel.hh) directly - no Gazebo instance,
// no ECM, no plugin loading. Mirrors plugins/aerodynamics/test/
// aero_model_selftest.cc's / plugins/propulsion/test/
// propulsion_model_selftest.cc's structure/style. This is NOT a substitute
// for gazebo-testing's live-Gazebo suite (which must confirm the real
// topic-advertise/subscribe/PreUpdate plumbing and the downstream
// AerodynamicsSystem/PropulsionSystem consumption behave as expected) -
// this only confirms the 1-cosine gust formula and steady+gust composition
// are internally consistent, numerically sane, and match the formula
// documented in WindModel.hh and docs/source_of_truth/environment/WIND.md.
//
// Build: see ../CMakeLists.txt (target wind_model_selftest, only depends on
// gz-math7, no full gz-sim/Gazebo runtime required).
// =============================================================================
#include <cmath>
#include <cstdio>
#include <string>

#include "../WindModel.hh"

using namespace falcon_v2_wind;

namespace
{
int gPass = 0;
int gFail = 0;

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

bool IsFinite(double v) { return std::isfinite(v); }
bool IsFinite(const gz::math::Vector3d &v)
{
  return IsFinite(v.X()) && IsFinite(v.Y()) && IsFinite(v.Z());
}

bool NearlyEqual(double a, double b, double tol) { return std::abs(a - b) <= tol; }
bool NearlyEqual(const gz::math::Vector3d &a, const gz::math::Vector3d &b, double tol)
{
  return NearlyEqual(a.X(), b.X(), tol) && NearlyEqual(a.Y(), b.Y(), tol) &&
         NearlyEqual(a.Z(), b.Z(), tol);
}

/// \brief Central-difference numerical derivative of GustEnvelope() w.r.t.
/// its time argument, used only to cross-check the analytically-known
/// derivative (pi/duration)*sin(2*pi*t/duration) - NOT used by the plugin
/// itself, self-test-only.
double NumericalEnvelopeDerivative(double t, double duration, double h)
{
  return (GustEnvelope(t + h, duration) - GustEnvelope(t - h, duration)) / (2.0 * h);
}
}  // namespace

int main()
{
  std::printf("=================================================================\n");
  std::printf("FALCON V2 Wind/Gust Model V1 - Standalone Self-Test\n");
  std::printf("=================================================================\n\n");

  const double kDuration = 4.0;    // s, arbitrary representative test gust
  const double kAmplitude = 6.0;   // m/s

  // -----------------------------------------------------------------------
  // GUST_ENVELOPE_BOUNDARY_TEST: envelope(0)=0 and envelope(duration)=0
  // exactly (analytic zero, not just "small").
  // -----------------------------------------------------------------------
  {
    const double e0 = GustEnvelope(0.0, kDuration);
    const double eEnd = GustEnvelope(kDuration, kDuration);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "envelope(0)=%.3e envelope(duration)=%.3e", e0, eEnd);
    Check("GUST_ENVELOPE_BOUNDARY_TEST",
          NearlyEqual(e0, 0.0, 1e-12) && NearlyEqual(eEnd, 0.0, 1e-12), buf);
  }

  // -----------------------------------------------------------------------
  // GUST_ENVELOPE_MIDPOINT_TEST: envelope(duration/2) = 1.0 exactly (peak
  // of the profile) - amplitude is fully reached at the midpoint.
  // -----------------------------------------------------------------------
  {
    const double eMid = GustEnvelope(kDuration / 2.0, kDuration);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "envelope(duration/2)=%.12f", eMid);
    Check("GUST_ENVELOPE_MIDPOINT_TEST", NearlyEqual(eMid, 1.0, 1e-12), buf);
  }

  // -----------------------------------------------------------------------
  // GUST_ENVELOPE_C1_CONTINUITY_TEST: derivative is exactly zero (per the
  // closed-form (pi/duration)*sin(2*pi*t/duration)) at BOTH boundaries, and
  // a numerical central-difference cross-check agrees to high precision -
  // this is the "no jerk discontinuity" requirement.
  // -----------------------------------------------------------------------
  {
    const double dAnalyticStart = (M_PI / kDuration) * std::sin(0.0);
    const double dAnalyticEnd = (M_PI / kDuration) * std::sin(2.0 * M_PI);
    const double h = 1e-6;
    const double dNumStart = NumericalEnvelopeDerivative(0.0, kDuration, h);
    const double dNumEnd = NumericalEnvelopeDerivative(kDuration, kDuration, h);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "analytic: start=%.3e end=%.3e | numerical(h=1e-6): start=%.3e end=%.3e",
        dAnalyticStart, dAnalyticEnd, dNumStart, dNumEnd);
    // Note: the numerical central difference straddles the domain boundary
    // (t=-h reads the "outside window" branch = 0, t=+h reads the "inside"
    // branch) - this is intentional: it directly tests that splicing the
    // active window onto the surrounding zero background introduces no
    // slope discontinuity, not merely that the interior formula's own
    // derivative is smooth.
    Check("GUST_ENVELOPE_C1_CONTINUITY_TEST",
          NearlyEqual(dAnalyticStart, 0.0, 1e-12) &&
              NearlyEqual(dAnalyticEnd, 0.0, 1e-12) &&
              NearlyEqual(dNumStart, 0.0, 1e-4) && NearlyEqual(dNumEnd, 0.0, 1e-4),
          buf);
  }

  // -----------------------------------------------------------------------
  // GUST_ENVELOPE_OUTSIDE_WINDOW_TEST: zero strictly before start and
  // strictly after end (well outside, not just at the boundary), and zero
  // for a non-positive duration (degenerate/invalid gust spec).
  // -----------------------------------------------------------------------
  {
    const double before = GustEnvelope(-1.0, kDuration);
    const double after = GustEnvelope(kDuration + 1.0, kDuration);
    const double zeroDur = GustEnvelope(1.0, 0.0);
    const double negDur = GustEnvelope(1.0, -2.0);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "before=%.3e after=%.3e zeroDur=%.3e negDur=%.3e",
        before, after, zeroDur, negDur);
    Check("GUST_ENVELOPE_OUTSIDE_WINDOW_TEST",
          before == 0.0 && after == 0.0 && zeroDur == 0.0 && negDur == 0.0, buf);
  }

  // -----------------------------------------------------------------------
  // GUST_VECTOR_MIDPOINT_TEST: EvaluateGust() at the midpoint reproduces
  // direction_unit * amplitude exactly (the envelope is exactly 1 there).
  // -----------------------------------------------------------------------
  {
    GustState g;
    g.directionUnit = gz::math::Vector3d(1.0, 0.0, 0.0);
    g.amplitudeMps = kAmplitude;
    g.startTimeSec = 10.0;  // arbitrary non-zero absolute start time
    g.durationSec = kDuration;
    g.scheduled = true;

    const gz::math::Vector3d vMid =
        EvaluateGust(g, g.startTimeSec + kDuration / 2.0);
    const gz::math::Vector3d expected(kAmplitude, 0.0, 0.0);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "vMid=(%.6f,%.6f,%.6f) expected=(%.6f,0,0)",
        vMid.X(), vMid.Y(), vMid.Z(), kAmplitude);
    Check("GUST_VECTOR_MIDPOINT_TEST", NearlyEqual(vMid, expected, 1e-9) && IsFinite(vMid), buf);
  }

  // -----------------------------------------------------------------------
  // GUST_VECTOR_BOUNDARY_ZERO_TEST: EvaluateGust() is exactly Zero at and
  // outside the gust's own window (absolute-time version of the two tests
  // above, exercised through the full GustState/EvaluateGust() path).
  // -----------------------------------------------------------------------
  {
    GustState g;
    g.directionUnit = gz::math::Vector3d(0.0, 1.0, 0.0);
    g.amplitudeMps = kAmplitude;
    g.startTimeSec = 5.0;
    g.durationSec = kDuration;
    g.scheduled = true;

    const gz::math::Vector3d atStart = EvaluateGust(g, g.startTimeSec);
    const gz::math::Vector3d atEnd = EvaluateGust(g, g.startTimeSec + kDuration);
    const gz::math::Vector3d before = EvaluateGust(g, g.startTimeSec - 100.0);
    const gz::math::Vector3d after = EvaluateGust(g, g.startTimeSec + kDuration + 100.0);
    const bool ok = atStart == gz::math::Vector3d::Zero &&
                    atEnd == gz::math::Vector3d::Zero &&
                    before == gz::math::Vector3d::Zero &&
                    after == gz::math::Vector3d::Zero;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "atStart=(%.3f,%.3f,%.3f) atEnd=(%.3f,%.3f,%.3f) before=(%.3f,%.3f,%.3f) after=(%.3f,%.3f,%.3f)",
        atStart.X(), atStart.Y(), atStart.Z(), atEnd.X(), atEnd.Y(), atEnd.Z(),
        before.X(), before.Y(), before.Z(), after.X(), after.Y(), after.Z());
    Check("GUST_VECTOR_BOUNDARY_ZERO_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // GUST_NOT_SCHEDULED_TEST: a default-constructed (never-commanded)
  // GustState always evaluates to exactly Zero, for any time, including
  // times that would otherwise fall inside [0,duration] if scheduled=true -
  // confirms `scheduled` is a true hard gate, not merely a documentation
  // convention.
  // -----------------------------------------------------------------------
  {
    GustState g;  // scheduled=false by default; durationSec=0 by default too
    g.directionUnit = gz::math::Vector3d(1.0, 0.0, 0.0);
    g.amplitudeMps = 999.0;  // deliberately large - must still be ignored
    g.startTimeSec = 0.0;
    g.durationSec = 100.0;  // would be "inside window" at t=50 if scheduled
    g.scheduled = false;

    const gz::math::Vector3d v = EvaluateGust(g, 50.0);
    Check("GUST_NOT_SCHEDULED_TEST", v == gz::math::Vector3d::Zero,
        "unscheduled GustState ignored regardless of amplitude/duration/t");
  }

  // -----------------------------------------------------------------------
  // ZERO_WIND_REGRESSION_TEST: this is the pure-math analog of the task's
  // Part 4 requirement. With steady=Zero and no gust ever scheduled (the
  // plugin's default-constructed state at Configure() time, before any
  // command topic message has ever been received), ComposeWind() must
  // return EXACTLY gz::math::Vector3d::Zero (bit-identical, not merely
  // "small") at any sim time - matching AerodynamicsSystem.hh's/
  // PropulsionSystem.hh's own default `windWorld{gz::math::Vector3d::Zero}`
  // member-initializer value (confirmed by direct read of both headers this
  // task) exactly, so introducing this plugin cannot alter any already-
  // validated result when the new steady/gust commands are never used.
  // -----------------------------------------------------------------------
  {
    const gz::math::Vector3d steady = gz::math::Vector3d::Zero;
    GustState g{};  // every field at its default (scheduled=false)
    bool allExactZero = true;
    for (double t : {-10.0, 0.0, 0.001, 1.0, 1000.0, 1e9})
    {
      const gz::math::Vector3d gustVec = EvaluateGust(g, t);
      const gz::math::Vector3d total = ComposeWind(steady, gustVec);
      if (total.X() != 0.0 || total.Y() != 0.0 || total.Z() != 0.0)
        allExactZero = false;
    }
    Check("ZERO_WIND_REGRESSION_TEST", allExactZero,
        "default (never-commanded) state composes to bit-identical Zero at all sampled times");
  }

  // -----------------------------------------------------------------------
  // STEADY_GUST_COMPOSITION_NO_CROSS_CONTAMINATION_TEST: ComposeWind() is a
  // plain vector sum - steady-only, gust-only, and combined cases are all
  // exactly consistent with simple addition, with neither term perturbing
  // the other's own contribution.
  // -----------------------------------------------------------------------
  {
    const gz::math::Vector3d steady(2.0, -1.0, 0.5);
    const gz::math::Vector3d gustOnly(0.0, 0.0, 0.0);
    const gz::math::Vector3d steadyOnlyResult = ComposeWind(steady, gustOnly);

    const gz::math::Vector3d zeroSteady = gz::math::Vector3d::Zero;
    const gz::math::Vector3d gustVal(3.0, 4.0, -2.0);
    const gz::math::Vector3d gustOnlyResult = ComposeWind(zeroSteady, gustVal);

    const gz::math::Vector3d combined = ComposeWind(steady, gustVal);
    const gz::math::Vector3d expectedCombined = steady + gustVal;

    const bool ok = steadyOnlyResult == steady && gustOnlyResult == gustVal &&
                     NearlyEqual(combined, expectedCombined, 1e-12);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "steady-only=(%.2f,%.2f,%.2f) gust-only=(%.2f,%.2f,%.2f) combined=(%.2f,%.2f,%.2f)",
        steadyOnlyResult.X(), steadyOnlyResult.Y(), steadyOnlyResult.Z(),
        gustOnlyResult.X(), gustOnlyResult.Y(), gustOnlyResult.Z(),
        combined.X(), combined.Y(), combined.Z());
    Check("STEADY_GUST_COMPOSITION_NO_CROSS_CONTAMINATION_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // NORMALIZE_DIRECTION_TEST: a well-formed (non-unit) vector normalizes to
  // unit length exactly along the same ray; a degenerate (near-zero) vector
  // is correctly rejected (validOut=false, Zero returned) rather than
  // producing a NaN/huge unit vector.
  // -----------------------------------------------------------------------
  {
    bool valid1 = false;
    const gz::math::Vector3d unit1 =
        NormalizeDirection(gz::math::Vector3d(3.0, 4.0, 0.0), valid1);
    const bool lengthOk = NearlyEqual(unit1.Length(), 1.0, 1e-12);
    const bool directionOk = NearlyEqual(unit1, gz::math::Vector3d(0.6, 0.8, 0.0), 1e-12);

    bool valid2 = false;
    const gz::math::Vector3d unit2 =
        NormalizeDirection(gz::math::Vector3d(0.0, 0.0, 0.0), valid2);

    bool valid3 = false;
    const gz::math::Vector3d unit3 = NormalizeDirection(
        gz::math::Vector3d(1e-12, 0.0, 0.0), valid3);

    const bool ok = valid1 && lengthOk && directionOk && !valid2 &&
                     unit2 == gz::math::Vector3d::Zero && !valid3 &&
                     unit3 == gz::math::Vector3d::Zero && IsFinite(unit1);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "unit1=(%.4f,%.4f,%.4f) len=%.6f valid1=%d valid2(zero-vec)=%d valid3(tiny-vec)=%d",
        unit1.X(), unit1.Y(), unit1.Z(), unit1.Length(), valid1, valid2, valid3);
    Check("NORMALIZE_DIRECTION_TEST", ok, buf);
  }

  // -----------------------------------------------------------------------
  // GUST_AMPLITUDE_SIGN_TEST: a negative amplitude correctly flips the
  // resulting vector's sense relative to the unit direction (documented
  // "amplitude may be negative" behavior in WindSystem.hh's OnGustCmd()
  // field doc comment).
  // -----------------------------------------------------------------------
  {
    GustState g;
    g.directionUnit = gz::math::Vector3d(1.0, 0.0, 0.0);
    g.amplitudeMps = -5.0;
    g.startTimeSec = 0.0;
    g.durationSec = kDuration;
    g.scheduled = true;

    const gz::math::Vector3d vMid = EvaluateGust(g, kDuration / 2.0);
    const gz::math::Vector3d expected(-5.0, 0.0, 0.0);
    Check("GUST_AMPLITUDE_SIGN_TEST", NearlyEqual(vMid, expected, 1e-9),
        "negative amplitude flips the vector sense relative to direction_unit");
  }

  // -----------------------------------------------------------------------
  // ALL_FINITE_SWEEP_TEST: sweep a range of times (well before, inside, and
  // well after the gust window) and confirm no NaN/Inf ever appears from
  // either EvaluateGust() or ComposeWind().
  // -----------------------------------------------------------------------
  {
    GustState g;
    g.directionUnit = gz::math::Vector3d(0.3, -0.5, 0.8106).Normalize();
    g.amplitudeMps = 12.0;
    g.startTimeSec = 100.0;
    g.durationSec = 3.5;
    g.scheduled = true;
    const gz::math::Vector3d steady(1.0, 1.0, -1.0);

    bool allFinite = true;
    for (double t = 0.0; t <= 200.0; t += 0.37)
    {
      const gz::math::Vector3d gustVec = EvaluateGust(g, t);
      const gz::math::Vector3d total = ComposeWind(steady, gustVec);
      if (!IsFinite(gustVec) || !IsFinite(total))
        allFinite = false;
    }
    Check("ALL_FINITE_SWEEP_TEST", allFinite,
        "no NaN/Inf across a dense time sweep spanning before/inside/after the gust window");
  }

  std::printf("\n=================================================================\n");
  std::printf("SUMMARY: %d PASS, %d FAIL\n", gPass, gFail);
  std::printf("=================================================================\n");

  return gFail == 0 ? 0 : 1;
}

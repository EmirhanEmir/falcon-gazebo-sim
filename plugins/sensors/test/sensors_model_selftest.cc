// =============================================================================
// FALCON V2 - Sensor-suite V1 standalone, Gazebo-independent self-test
// =============================================================================
// Owner: controls-integration. Task:
// SENSOR_MODEL_AND_ARDUPLANE_SITL_PREPARATION (2026-08-27).
//
// Exercises the PURE-MATH core (Frames.hh, PitotModel.hh) directly - no
// Gazebo instance, no ECM, no plugin loading. Mirrors
// plugins/wind/test/wind_model_selftest.cc's structure/style. This is NOT a
// substitute for gazebo-testing's live-Gazebo suite (which must confirm the
// real native IMU/GPS/baro/magnetometer sensors and the PitotSystem plugin's
// topic-advertise/subscribe/PreUpdate plumbing actually produce live,
// finite output) - this only confirms the frame-transform math and the
// pitot arithmetic are internally consistent, numerically sane, and match
// what is documented in Frames.hh / PitotModel.hh /
// docs/source_of_truth/sensors/SENSORS.md.
//
// Build: see ../CMakeLists.txt (target sensors_model_selftest, only depends
// on gz-math7, no full gz-sim/Gazebo runtime required).
// =============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../Frames.hh"
#include "../PitotModel.hh"

using namespace falcon_v2_sensors;

namespace
{
int gPass = 0;
int gFail = 0;

void Check(const std::string &name, bool ok, const std::string &detail)
{
  if (ok)
  {
    ++gPass;
    std::printf("[PASS] %-46s %s\n", name.c_str(), detail.c_str());
  }
  else
  {
    ++gFail;
    std::printf("[FAIL] %-46s %s\n", name.c_str(), detail.c_str());
  }
}

bool NearlyEqual(double a, double b, double tol) { return std::abs(a - b) <= tol; }
bool NearlyEqual(const gz::math::Vector3d &a, const gz::math::Vector3d &b, double tol)
{
  return NearlyEqual(a.X(), b.X(), tol) && NearlyEqual(a.Y(), b.Y(), tol) &&
         NearlyEqual(a.Z(), b.Z(), tol);
}

/// \brief Apply a 3x3-rotation-as-function to each of the 3 standard basis
/// vectors to recover its matrix columns, then check orthogonality
/// (R^T R = I) and properness (det = +1) purely numerically - no hand
/// re-derivation of the matrix, just a black-box property check of the
/// function under test.
struct Mat3
{
  double m[3][3];
};

Mat3 BuildMatrixFromFunction(gz::math::Vector3d (*f)(const gz::math::Vector3d &))
{
  const gz::math::Vector3d c0 = f(gz::math::Vector3d(1, 0, 0));
  const gz::math::Vector3d c1 = f(gz::math::Vector3d(0, 1, 0));
  const gz::math::Vector3d c2 = f(gz::math::Vector3d(0, 0, 1));
  Mat3 m;
  m.m[0][0] = c0.X(); m.m[0][1] = c1.X(); m.m[0][2] = c2.X();
  m.m[1][0] = c0.Y(); m.m[1][1] = c1.Y(); m.m[1][2] = c2.Y();
  m.m[2][0] = c0.Z(); m.m[2][1] = c1.Z(); m.m[2][2] = c2.Z();
  return m;
}

double Det3(const Mat3 &m)
{
  return m.m[0][0] * (m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1]) -
         m.m[0][1] * (m.m[1][0] * m.m[2][2] - m.m[1][2] * m.m[2][0]) +
         m.m[0][2] * (m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0]);
}

bool IsOrthogonal(const Mat3 &m, double tol)
{
  // (M^T M)_ij = sum_k M_ki * M_kj ; must equal identity.
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) sum += m.m[k][i] * m.m[k][j];
      const double expected = (i == j) ? 1.0 : 0.0;
      if (std::abs(sum - expected) > tol) return false;
    }
  }
  return true;
}
}  // namespace

int main()
{
  std::printf("=================================================================\n");
  std::printf("FALCON V2 Sensor Suite V1 - Standalone Self-Test\n");
  std::printf("=================================================================\n\n");

  // -----------------------------------------------------------------------
  // FLU_FRD_ROTATION_PROPERTIES_TEST
  // -----------------------------------------------------------------------
  {
    const Mat3 R = BuildMatrixFromFunction(&FluFrdSwap);
    const bool orthogonal = IsOrthogonal(R, 1e-12);
    const double det = Det3(R);
    const gz::math::Vector3d v(1.0, 2.0, 3.0);
    const gz::math::Vector3d roundTrip = FluFrdSwap(FluFrdSwap(v));
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "orthogonal=%d det=%.6f roundtrip_err=%.3e (expect (1,-2,-3) once)",
        orthogonal, det, (roundTrip - v).Length());
    const gz::math::Vector3d once = FluFrdSwap(v);
    Check("FLU_FRD_ROTATION_PROPERTIES_TEST",
          orthogonal && NearlyEqual(det, 1.0, 1e-12) &&
              NearlyEqual(roundTrip, v, 1e-12) &&
              NearlyEqual(once, gz::math::Vector3d(1.0, -2.0, -3.0), 1e-12),
          buf);
  }

  // -----------------------------------------------------------------------
  // ENU_NED_ROTATION_PROPERTIES_TEST
  // -----------------------------------------------------------------------
  {
    const Mat3 R = BuildMatrixFromFunction(&EnuNedSwap);
    const bool orthogonal = IsOrthogonal(R, 1e-12);
    const double det = Det3(R);
    const gz::math::Vector3d v(1.0, 2.0, 3.0);  // (East=1, North=2, Up=3)
    const gz::math::Vector3d roundTrip = EnuNedSwap(EnuNedSwap(v));
    const gz::math::Vector3d once = EnuNedSwap(v);  // expect (North=2,East=1,Down=-3)
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "orthogonal=%d det=%.6f roundtrip_err=%.3e once=(%.1f,%.1f,%.1f) expect (2,1,-3)",
        orthogonal, det, (roundTrip - v).Length(), once.X(), once.Y(), once.Z());
    Check("ENU_NED_ROTATION_PROPERTIES_TEST",
          orthogonal && NearlyEqual(det, 1.0, 1e-12) &&
              NearlyEqual(roundTrip, v, 1e-12) &&
              NearlyEqual(once, gz::math::Vector3d(2.0, 1.0, -3.0), 1e-12),
          buf);
  }

  // -----------------------------------------------------------------------
  // IMU_SPECIFIC_FORCE_SIGN_TEST: documented EXPECTED convention - an IMU
  // at rest on a table reads +g along "up" (reaction to gravity pulling
  // down, standard accelerometer specific-force convention). In body FLU,
  // "up" is +Z; in body FRD, "up" is -Z. This test checks
  // FluFrdSwap((0,0,+9.81)) == (0,0,-9.81) - i.e. the SAME transform used
  // for every other body vector, applied to a physically concrete case.
  // gazebo-testing's live IMU reading is the authoritative confirmation of
  // the native sensor's actual sign; this is a documented-assumption check.
  // -----------------------------------------------------------------------
  {
    const gz::math::Vector3d specificForceFluAtRest(0.0, 0.0, 9.81);
    const gz::math::Vector3d specificForceFrd = FluFrdSwap(specificForceFluAtRest);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "FRD specific force at rest = (%.3f,%.3f,%.3f)",
        specificForceFrd.X(), specificForceFrd.Y(), specificForceFrd.Z());
    Check("IMU_SPECIFIC_FORCE_SIGN_TEST",
          NearlyEqual(specificForceFrd, gz::math::Vector3d(0.0, 0.0, -9.81), 1e-12),
          buf);
  }

  // -----------------------------------------------------------------------
  // GPS_VELOCITY_ENU_TO_NED_TEST: gz.msgs.NavSat gives
  // (velocity_east, velocity_north, velocity_up) explicitly (confirmed,
  // navsat.proto). velocity_east=3, velocity_north=5, velocity_up=-1 m/s ->
  // NED (north=5, east=3, down=1).
  // -----------------------------------------------------------------------
  {
    const gz::math::Vector3d vEnu(3.0, 5.0, -1.0);  // (East,North,Up)
    const gz::math::Vector3d vNed = EnuNedSwap(vEnu);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "NED=(%.3f,%.3f,%.3f) expect (5,3,1)",
        vNed.X(), vNed.Y(), vNed.Z());
    Check("GPS_VELOCITY_ENU_TO_NED_TEST",
          NearlyEqual(vNed, gz::math::Vector3d(5.0, 3.0, 1.0), 1e-12), buf);
  }

  // -----------------------------------------------------------------------
  // BARO_ALTITUDE_DOWN_SIGN_TEST
  // -----------------------------------------------------------------------
  {
    const double alt = 120.0;
    const double down = AltitudeToNedDown(alt);
    const double roundTrip = NedDownToAltitude(down);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "altitude=%.1f -> down=%.1f -> altitude=%.1f",
        alt, down, roundTrip);
    Check("BARO_ALTITUDE_DOWN_SIGN_TEST",
          NearlyEqual(down, -120.0, 1e-12) && NearlyEqual(roundTrip, alt, 1e-12), buf);
  }

  // -----------------------------------------------------------------------
  // ATTITUDE_QUATERNION_TRANSFORM_TEST: self-consistency identity -
  // result.RotateVector(v_frd) == EnuNedSwap(q.RotateVector(FluFrdSwap(v_frd)))
  // for several representative attitudes and several test vectors. Does NOT
  // depend on hand-picking an "expected" Euler angle - it is an algebraic
  // consistency check of AttitudeFluEnuToFrdNed()'s own derivation.
  // -----------------------------------------------------------------------
  {
    const std::vector<gz::math::Quaterniond> testAttitudes = {
        gz::math::Quaterniond(1, 0, 0, 0),                                  // identity
        gz::math::Quaterniond(gz::math::Vector3d(0, 0, 1), GZ_PI / 2.0),    // 90 deg yaw
        gz::math::Quaterniond(gz::math::Vector3d(1, 0, 0), 0.3),            // roll 0.3 rad
        gz::math::Quaterniond(0.2, 0.4, -0.1, 0.3)};  // arbitrary, will be normalized
    const std::vector<gz::math::Vector3d> testVecs = {
        gz::math::Vector3d(1, 0, 0), gz::math::Vector3d(0, 1, 0),
        gz::math::Vector3d(0, 0, 1), gz::math::Vector3d(0.5, -1.2, 3.4)};

    bool allOk = true;
    double maxErr = 0.0;
    for (const auto &qRaw : testAttitudes)
    {
      const gz::math::Quaterniond q = qRaw.Normalized();
      const gz::math::Quaterniond qFrdNed = AttitudeFluEnuToFrdNed(q);
      for (const auto &vFrd : testVecs)
      {
        const gz::math::Vector3d lhs = qFrdNed.RotateVector(vFrd);
        const gz::math::Vector3d rhs = EnuNedSwap(q.RotateVector(FluFrdSwap(vFrd)));
        const double err = (lhs - rhs).Length();
        maxErr = std::max(maxErr, err);
        if (err > 1e-9) allOk = false;
      }
    }
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "%zu attitudes x %zu vectors, max identity error=%.3e",
        testAttitudes.size(), testVecs.size(), maxErr);
    Check("ATTITUDE_QUATERNION_TRANSFORM_TEST", allOk, buf);
  }

  // -----------------------------------------------------------------------
  // HEADING_CARDINAL_TEST: north-only -> 0 rad, east-only -> +pi/2 rad,
  // south-only -> +/-pi rad, west-only -> -pi/2 rad (clockwise-from-north
  // aviation/compass convention).
  // -----------------------------------------------------------------------
  {
    const double hNorth = HeadingFromNedHorizontalRad(1.0, 0.0);
    const double hEast = HeadingFromNedHorizontalRad(0.0, 1.0);
    const double hSouth = HeadingFromNedHorizontalRad(-1.0, 0.0);
    const double hWest = HeadingFromNedHorizontalRad(0.0, -1.0);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "N=%.4f E=%.4f S=%.4f W=%.4f (rad)", hNorth, hEast, hSouth, hWest);
    Check("HEADING_CARDINAL_TEST",
          NearlyEqual(hNorth, 0.0, 1e-12) && NearlyEqual(hEast, GZ_PI / 2.0, 1e-12) &&
              NearlyEqual(std::abs(hSouth), GZ_PI, 1e-12) &&
              NearlyEqual(hWest, -GZ_PI / 2.0, 1e-12),
          buf);
  }

  // -----------------------------------------------------------------------
  // PITOT_ZERO_WIND_TEST / PITOT_HEADWIND_TEST / PITOT_TAILWIND_TEST /
  // PITOT_CROSSWIND_TEST: aircraft flying +20 m/s East (world ENU X), wind
  // as documented (Vrel = Vbody - Vwind, air-mass-velocity convention).
  // -----------------------------------------------------------------------
  {
    const gz::math::Vector3d vBody(20.0, 0.0, 0.0);

    const gz::math::Vector3d vZeroWind(0.0, 0.0, 0.0);
    const double asZero = AirspeedMps(AirRelativeVelocityWorld(vBody, vZeroWind));
    Check("PITOT_ZERO_WIND_TEST", NearlyEqual(asZero, 20.0, 1e-9),
          "airspeed=" + std::to_string(asZero) + " expect 20.0");

    // Headwind: air mass moving in -X (toward the nose) -> relative airflow
    // speed EXCEEDS ground speed.
    const gz::math::Vector3d vHeadwind(-5.0, 0.0, 0.0);
    const double asHead = AirspeedMps(AirRelativeVelocityWorld(vBody, vHeadwind));
    Check("PITOT_HEADWIND_TEST", NearlyEqual(asHead, 25.0, 1e-9),
          "airspeed=" + std::to_string(asHead) + " expect 25.0 (> groundspeed)");

    // Tailwind: air mass moving in +X (same direction as flight) ->
    // relative airflow speed is LESS than ground speed.
    const gz::math::Vector3d vTailwind(5.0, 0.0, 0.0);
    const double asTail = AirspeedMps(AirRelativeVelocityWorld(vBody, vTailwind));
    Check("PITOT_TAILWIND_TEST", NearlyEqual(asTail, 15.0, 1e-9),
          "airspeed=" + std::to_string(asTail) + " expect 15.0 (< groundspeed)");

    // Crosswind: air mass moving in +Y (North) -> airspeed slightly EXCEEDS
    // groundspeed (sqrt(20^2+5^2) = sqrt(425) ~= 20.6155).
    const gz::math::Vector3d vCrosswind(0.0, 5.0, 0.0);
    const double asCross = AirspeedMps(AirRelativeVelocityWorld(vBody, vCrosswind));
    Check("PITOT_CROSSWIND_TEST", NearlyEqual(asCross, std::sqrt(425.0), 1e-9),
          "airspeed=" + std::to_string(asCross) + " expect " +
              std::to_string(std::sqrt(425.0)));
  }

  // -----------------------------------------------------------------------
  // PITOT_MAGNITUDE_FRAME_INVARIANCE_TEST: |V_rel| computed in world frame
  // must equal |V_rel| after applying either of the two documented
  // rotations (both are proper, orthogonal - length-preserving by
  // construction) - confirms AirspeedMps() is genuinely frame-invariant,
  // not accidentally correct only in the specific frame it happens to be
  // called with in PitotSystem.cc.
  // -----------------------------------------------------------------------
  {
    const gz::math::Vector3d vRel(12.3, -4.5, 2.1);
    const double magOriginal = AirspeedMps(vRel);
    const double magAfterFluFrd = AirspeedMps(FluFrdSwap(vRel));
    const double magAfterEnuNed = AirspeedMps(EnuNedSwap(vRel));
    char buf[256];
    std::snprintf(buf, sizeof(buf), "orig=%.6f afterFluFrd=%.6f afterEnuNed=%.6f",
        magOriginal, magAfterFluFrd, magAfterEnuNed);
    Check("PITOT_MAGNITUDE_FRAME_INVARIANCE_TEST",
          NearlyEqual(magOriginal, magAfterFluFrd, 1e-9) &&
              NearlyEqual(magOriginal, magAfterEnuNed, 1e-9),
          buf);
  }

  // -----------------------------------------------------------------------
  // PITOT_DIFFERENTIAL_PRESSURE_TEST: q = 0.5*rho*V^2, rho=1.225 (CITED,
  // ISA sea-level, matches aero_v1_config.yaml/propulsion_v1_config.yaml),
  // V=20 m/s -> q = 0.5*1.225*400 = 245.0 Pa.
  // -----------------------------------------------------------------------
  {
    const double q = DifferentialPressurePa(20.0, 1.225);
    Check("PITOT_DIFFERENTIAL_PRESSURE_TEST", NearlyEqual(q, 245.0, 1e-9),
          "q=" + std::to_string(q) + " Pa, expect 245.0 Pa");
  }

  std::printf("\n=================================================================\n");
  std::printf("RESULT: %d passed, %d failed\n", gPass, gFail);
  std::printf("=================================================================\n");
  return (gFail == 0) ? 0 : 1;
}

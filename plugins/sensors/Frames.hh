// =============================================================================
// FALCON V2 - Sensor-suite reference-frame transform library (pure math)
// =============================================================================
// Pure math only: gz-math7 dependency only, no gz-sim/ECM dependency.
//
// SCOPE: every frame/sign conversion the sensor suite needs, derived once
// here so no other file re-derives (and potentially mis-derives) the same
// transform. Nothing in this header reads live Gazebo state; callers supply
// already-fetched vectors and quaternions.
//
// ----------------------------------------------------------------------------
// GOVERNING FRAME FACTS
//   - This project's Gazebo body frame is FLU (+X forward, +Y left, +Z up),
//     which is exactly the body frame the official ardupilot_gazebo
//     ArduPilotPlugin assumes by default (its gazeboXYZToNED transform).
//   - ArduPilot's own body frame is FRD (+X forward, +Y right, +Z down).
//   - Gazebo's world frame in this project is ENU (+X east, +Y north,
//     +Z up), matching sdformat's spherical_coordinates.sdf default
//     (world_frame_orientation = "ENU").
//   - ArduPilot's world frame is NED (+X north, +Y east, +Z down).
// ----------------------------------------------------------------------------
#ifndef FALCON_V2_SENSORS_FRAMES_HH_
#define FALCON_V2_SENSORS_FRAMES_HH_

#include <cmath>

#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace falcon_v2_sensors
{

// =============================================================================
// FLU <-> FRD (body-frame free vectors: angular velocity, specific
// force/linear acceleration, magnetometer body-frame field, and any
// body-relative position/offset vector)
// =============================================================================
//
// DERIVATION: the ardupilot_gazebo plugin's own default
// gazeboXYZToNED = Pose3d(0,0,0, GZ_PI,0,0) is a 180 deg rotation about the
// body's own X axis. As a rotation MATRIX this is R = diag(1,-1,-1)
// (standard rotation-about-X-by-pi matrix: R = [[1,0,0],[0,cos(pi),-sin(pi)],
// [0,sin(pi),cos(pi)]] = [[1,0,0],[0,-1,0],[0,0,-1]]). Applied to an FLU
// vector (x,y,z): R*(x,y,z)^T = (x,-y,-z) - i.e. X (forward) is UNCHANGED,
// Y and Z both negate. This matches FLU->FRD directly: forward stays
// forward; left(+Y_flu) becomes -Y_frd i.e. right is now +Y_frd; up(+Z_flu)
// becomes -Z_frd i.e. down is now +Z_frd. R is orthogonal (R^T R = I), proper
// (det(R) = +1, a genuine rotation not a reflection), and idempotent under
// two applications (R^2 = I, i.e. a 180 deg rotation is its own inverse -
// FLU->FRD and FRD->FLU are THE SAME formula).
//
// Any ordinary body-frame 3-vector rigidly attached to the aircraft
// (angular velocity, specific force/proper acceleration, a magnetometer's
// body-frame field reading, a body-relative offset such as a sensor's
// mount point relative to the body origin) transforms via this SAME
// formula, because all of these are "free vectors" that rotate exactly like
// any other vector under a fixed rotation between two frames rigidly
// attached to the same body - there is no separate formula needed per
// physical quantity.
inline gz::math::Vector3d FluFrdSwap(const gz::math::Vector3d &_v)
{
  return gz::math::Vector3d(_v.X(), -_v.Y(), -_v.Z());
}

// =============================================================================
// ENU <-> NED (world-frame free vectors: GPS ENU velocity -> NED velocity,
// world-frame Earth magnetic field, or any world-relative position/
// displacement vector, e.g. an EKF-origin-relative NED position built from a
// Gazebo ENU displacement)
// =============================================================================
//
// DERIVATION: world ENU (East,North,Up) -> NED (North,East,Down) swaps the
// first two components and negates the third: f(x,y,z) = (y,x,-z). As a
// matrix, M = [[0,1,0],[1,0,0],[0,0,-1]]. Confirmed algebraically (not just
// restated) by ENU_NED_ROTATION_PROPERTIES_TEST: M is orthogonal
// (M^T M = I), proper (det(M) = +1 - a genuine rotation, specifically a
// 180 deg rotation about the horizontal North-East bisector axis
// (1,1,0)/sqrt(2), since trace(M) = 0+0-1 = -1 = 1+2*cos(theta) => theta =
// 180 deg), and idempotent under two applications (M^2 = I - ENU->NED and
// NED->ENU are THE SAME formula, exactly like the FLU/FRD case above,
// though the two rotations are about DIFFERENT axes and must never be
// confused/interchanged - see the class-level warning below).
inline gz::math::Vector3d EnuNedSwap(const gz::math::Vector3d &_v)
{
  return gz::math::Vector3d(_v.Y(), _v.X(), -_v.Z());
}

// =============================================================================
// EXPLICIT WARNING: FluFrdSwap() and EnuNedSwap() are algebraically
// DIFFERENT rotations (about different physical axes - body X vs. the
// horizontal North-East bisector) that happen to share the "both are a 180
// deg involution" property. Applying FluFrdSwap() to a WORLD-frame vector,
// or EnuNedSwap() to a BODY-frame vector, is a frame-category error and
// will silently produce a wrong-but-finite result - every call site must be
// able to name which category (body vs. world) the vector it is converting
// belongs to.
// =============================================================================

// =============================================================================
// Altitude (world "up", meters, positive above this project's world origin)
// <-> NED "Down" coordinate. Trivial by definition (Down = -Up), but stated
// explicitly so it is not conflated with EnuNedSwap() above, which acts on a
// 3-vector rather than a scalar altitude.
// =============================================================================
inline double AltitudeToNedDown(double _altitudeM) { return -_altitudeM; }
inline double NedDownToAltitude(double _downM) { return -_downM; }

// =============================================================================
// Attitude quaternion convention change.
// =============================================================================
// Input: qFluRelEnu - the orientation of the body FLU frame relative to the
// world ENU frame, in the standard "body-to-world" active-rotation sense
// used by gz-sim's IMU sensor (localization=ENU) and by gz::math::Quaternion
// itself: for any body-frame vector v_body, q.RotateVector(v_body) returns
// that same physical vector expressed in world coordinates
// (v_world = q * v_body * q^-1).
//
// Output: the equivalent orientation of the body FRD frame relative to the
// world NED frame, in the same active-rotation sense, i.e. for any
// FRD-frame vector v_frd,
//   result.RotateVector(v_frd) == EnuNedSwap(qFluRelEnu.RotateVector(FluFrdSwap(v_frd)))
// for ALL v_frd - this identity is exactly what
// ATTITUDE_QUATERNION_TRANSFORM_TEST checks numerically (a self-consistency
// check that does not depend on hand-picking a "known" Euler angle).
//
// DERIVATION: let R = the FLU<->FRD rotation matrix (self-inverse) and
// R' = the ENU<->NED rotation matrix (self-inverse), and let Q_fe be the
// rotation matrix corresponding to qFluRelEnu. Substituting
// v_body_flu = R * v_body_frd (R self-inverse) into
// v_world_enu = Q_fe * v_body_flu and then v_world_ned = R' * v_world_enu
// gives v_world_ned = (R' * Q_fe * R) * v_body_frd - i.e. the desired
// FRD-to-NED rotation matrix is R' * Q_fe * R. Expressed as a quaternion
// product (Hamilton product composition matches matrix product composition
// for gz::math::Quaternion, confirmed via the RotateVector identity test
// above): result = qEnuToNedRot * qFluRelEnu * qFluToFrdRot, where
// qFluToFrdRot is the quaternion for a 180 deg rotation about body X, and
// qEnuToNedRot is the quaternion for a 180 deg rotation about the
// world (1,1,0)/sqrt(2) axis - exactly the two rotations derived above for
// FluFrdSwap()/EnuNedSwap(), now applied as quaternions instead of 3x3
// matrices acting on plain vectors.
inline gz::math::Quaterniond AttitudeFluEnuToFrdNed(
    const gz::math::Quaterniond &_qFluRelEnu)
{
  static const gz::math::Quaterniond kQFluToFrdRot(
      gz::math::Vector3d(1.0, 0.0, 0.0), GZ_PI);
  static const gz::math::Quaterniond kQEnuToNedRot(
      gz::math::Vector3d(1.0, 1.0, 0.0).Normalized(), GZ_PI);
  return kQEnuToNedRot * _qFluRelEnu * kQFluToFrdRot;
}

// =============================================================================
// Heading (radians, standard aviation/compass convention: 0 = north,
// positive CLOCKWISE toward east when viewed from above) from the
// horizontal (north, east) components of an NED-frame vector (magnetic
// field or ground-track velocity). atan2(east, north) - NOT atan2(north,
// east) - is what produces the clockwise-from-north convention (verified
// numerically by HEADING_CARDINAL_TEST: north-only input -> 0 rad,
// east-only input -> +pi/2 rad).
// =============================================================================
inline double HeadingFromNedHorizontalRad(double _north, double _east)
{
  return std::atan2(_east, _north);
}

}  // namespace falcon_v2_sensors

#endif  // FALCON_V2_SENSORS_FRAMES_HH_

// =============================================================================
// FALCON V2 - Pitot (simulated airspeed) sensor V1 core math model
// =============================================================================
// Pure math only: gz-math7 dependency only, no gz-sim/ECM dependency.
//
// WHY A CUSTOM PLUGIN INSTEAD OF THE NATIVE gz-sim air_speed SENSOR: the
// installed sdformat air_speed.sdf sensor spec has no wind-related
// configurable field at all (only a "pressure" noise block), so it cannot
// consume this project's /model/falcon_v2/wind gz-transport topic. The only
// native Gazebo wind mechanism (gz-sim-wind-effects-system plus the
// world-level <wind><linear_velocity> element) is a separate, FORCE-applying
// mechanism unrelated to this project's velocity-publishing WindSystem; using
// it would either silently ignore the commanded wind or require a second,
// redundant wind-consumption path. This plugin instead subscribes to the
// existing /model/falcon_v2/wind topic directly, exactly as
// AerodynamicsSystem.cc and PropulsionSystem.cc do, and computes airspeed
// from first principles below.
//
// PHYSICAL MODEL (V1, incompressible flow - Mach ~0.09 at sea level within
// this aircraft's flight envelope, deep in the incompressible regime):
//   V_rel_world  = V_point_world - V_wind_world   (the project-wide Vrel
//                  convention)
//   airspeed_mps = |V_rel_world|                  (frame-invariant magnitude:
//                  rotations preserve vector length)
//   q_pa         = 0.5 * rho * airspeed_mps^2     (standard incompressible
//                  dynamic pressure, which is what a real pitot-static
//                  system's differential port measures at these speeds)
//
// rho (air density, kg/m^3) is not invented here: the caller must supply it
// from config/aero_v1_config.yaml environment.air_density_rho_kg_m3 (ISA
// sea-level, 1.225), the project's single source of truth for this constant,
// shared with config/propulsion_v1_config.yaml environment.rho_kg_m3.
//
// V1 LIMITATION: this is a scalar dynamic-pressure pitot model only. It does
// not model reverse/negative-AoA flow onto the pitot port, compressibility,
// or any position-error (installation) correction. q_pa is always >= 0.
// =============================================================================
#ifndef FALCON_V2_SENSORS_PITOT_MODEL_HH_
#define FALCON_V2_SENSORS_PITOT_MODEL_HH_

#include <gz/math/Vector3.hh>

namespace falcon_v2_sensors
{

/// \brief V_rel_world = V_point_world - V_wind_world. Both inputs MUST be
/// in the SAME frame (world frame, in every real call site in this
/// project) - this function does not itself enforce that, callers are
/// responsible (mirrors AerodynamicsSystem.cc's/PropulsionSystem.cc's own
/// existing Vrel computation, not a new convention).
inline gz::math::Vector3d AirRelativeVelocityWorld(
    const gz::math::Vector3d &_vPointWorld,
    const gz::math::Vector3d &_vWindWorld)
{
  return _vPointWorld - _vWindWorld;
}

/// \brief Scalar airspeed (m/s), frame-invariant magnitude of the relative
/// air velocity. Always >= 0.
inline double AirspeedMps(const gz::math::Vector3d &_vRelWorld)
{
  return _vRelWorld.Length();
}

/// \brief Standard incompressible dynamic/differential pressure (Pa) for a
/// given scalar airspeed and air density. Always >= 0 (V1 limitation - see
/// header comment; a real pitot's SIGNED differential pressure under
/// reverse flow is not modeled).
inline double DifferentialPressurePa(double _airspeedMps, double _rhoKgM3)
{
  return 0.5 * _rhoKgM3 * _airspeedMps * _airspeedMps;
}

}  // namespace falcon_v2_sensors

#endif  // FALCON_V2_SENSORS_PITOT_MODEL_HH_

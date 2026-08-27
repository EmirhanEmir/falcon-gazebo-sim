// =============================================================================
// FALCON V2 - Pitot (simulated airspeed) sensor V1 core math model
// =============================================================================
// Owner: controls-integration. Task:
// SENSOR_MODEL_AND_ARDUPLANE_SITL_PREPARATION (2026-08-27).
//
// Pure math only (gz-math7 dependency ONLY, no gz-sim/ECM dependency) - same
// precedent as plugins/wind/WindModel.hh. Linked into both the real
// PitotSystem.cc plugin and the standalone self-test.
//
// WHY A CUSTOM PLUGIN INSTEAD OF THE NATIVE gz-sim air_speed SENSOR:
// see docs/source_of_truth/sensors/SENSORS.md sec 3 for the full decision
// record. Short version: the installed sdformat air_speed.sdf sensor spec
// (/usr/share/sdformat14/1.11/air_speed.sdf) has NO wind-related
// configurable field at all (only a "pressure" noise block) - it is not
// wired to consume any wind source through its own SDF interface,
// certainly not this project's custom /model/falcon_v2/wind gz-transport
// topic (which plugins/wind/WindSystem.cc publishes and only
// AerodynamicsSystem.cc/PropulsionSystem.cc subscribe to). The ONLY native
// Gazebo wind mechanism found on this system (gz-sim-wind-effects-system
// plus the world-level <wind><linear_velocity> SDF element, confirmed via
// /usr/share/gz/gz-sim8/worlds/wind.sdf) is an entirely separate,
// FORCE-applying mechanism unrelated to this project's velocity-publishing
// WindSystem - using it would either silently ignore this project's
// commanded wind (failing the mandatory headwind/tailwind live test) or
// require standing up a second, redundant wind-consumption path, which
// docs/source_of_truth/environment/WIND.md sec "scope" explicitly forbids
// ("This is the ONLY wind source in the project - no second
// wind-consumption path is created"). This plugin instead subscribes to
// the EXISTING /model/falcon_v2/wind topic directly, exactly like
// AerodynamicsSystem.cc/PropulsionSystem.cc already do, and computes
// airspeed from first principles below.
//
// PHYSICAL MODEL (V1, incompressible flow - valid well within this
// aircraft's flight envelope, max ~30 m/s per
// docs/test_results/2026-08-27_flight_envelope_validation.md, Mach ~0.09 at
// sea level, deep in the incompressible regime):
//   V_rel_world = V_point_world - V_wind_world      (same Vrel convention
//                 already used project-wide, confirmed by direct read of
//                 AerodynamicsSystem.cc/PropulsionSystem.cc/WindModel.hh)
//   airspeed_mps = |V_rel_world|                     (frame-invariant
//                 magnitude - true regardless of which frame V_rel is
//                 expressed in, since rotations preserve vector length;
//                 confirmed by PITOT_MAGNITUDE_FRAME_INVARIANCE_TEST, which
//                 applies plugins/sensors/Frames.hh's rotations to V_rel
//                 before taking the magnitude and checks the result is
//                 unchanged)
//   q_pa = 0.5 * rho * airspeed_mps^2                 (standard
//                 incompressible dynamic/differential pressure - this is
//                 exactly what a real pitot-static system's differential
//                 pressure port measures at these speeds)
// rho (air density, kg/m^3) is NOT invented here - the caller (PitotSystem)
// must supply it from docs/source_of_truth/aerodynamics/aero_v1_config.yaml
// environment.air_density_rho_kg_m3 (CONFIRMED, ISA sea-level, 1.225,
// already the project's single source of truth for this constant, shared
// with docs/source_of_truth/propulsion/propulsion_v1_config.yaml
// environment.rho_kg_m3 - the SAME cited value, not a new/independent
// number for this task).
//
// V1 LIMITATION (stated, not silent): this is a scalar dynamic-pressure
// pitot model only - it does not model reverse/negative-AoA flow direction
// onto the pitot port, compressibility, or any position-error
// (installation) correction. q_pa as defined here is always >= 0.
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

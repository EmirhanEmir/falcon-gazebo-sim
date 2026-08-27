// =============================================================================
// FALCON V2 - Wind/gust generator V1 core math model
// =============================================================================
// Owner: aerodynamics specialist agent. Task:
// WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION (2026-08-27).
//
// This header contains ONLY pure math: no gz-sim (ECM/Entity/System)
// dependency, so it can be (a) linked into the real Gazebo System plugin
// (WindSystem.cc) and (b) linked into a small standalone, Gazebo-independent
// self-test executable (test/wind_model_selftest.cc) that can be compiled
// and run without launching a full Gazebo Sim instance. The only external
// dependency is gz-math7 (Vector3d), the same precedent already used by
// plugins/aerodynamics/AeroModel.hh and plugins/propulsion/PropulsionModel.hh
// (both already gz-sim-free pure-math headers).
//
// SCOPE (explicitly limited, see docs/source_of_truth/environment/WIND.md
// for the full writeup of what is and is NOT modeled):
//   - A single, deterministic 1-cosine gust superposed on a constant
//     ("steady") wind component.
//   - World-frame wind VELOCITY (m/s) - this is the velocity of the AIR MASS
//     in world coordinates, i.e. exactly the "Vwind" already consumed by
//     plugins/aerodynamics/AerodynamicsSystem.cc and
//     plugins/propulsion/PropulsionSystem.cc as
//     Vrel = Vbody - Vwind (CONFIRMED by direct read of both files this
//     task, both already implement this exact relative-airflow convention -
//     it was NOT authored or modified by this task, only a publisher for
//     the topic they already subscribe to was added).
// NOT modeled here (or anywhere in this plugin): spatial wind gradients
// (shear/gust fronts that vary with aircraft position), terrain effects,
// rotor-wake/building-wake effects, Dryden/Von Karman stochastic turbulence,
// or any weather-scale atmospheric model. This is a minimal, deterministic
// V1 disturbance generator only.
// =============================================================================
#ifndef FALCON_V2_WIND_MODEL_HH_
#define FALCON_V2_WIND_MODEL_HH_

#include <cmath>

#include <gz/math/Vector3.hh>

namespace falcon_v2_wind
{

/// \brief Minimum vector norm treated as "non-degenerate" when normalizing a
/// commanded gust direction vector. Below this, the direction is considered
/// degenerate (an all-zero or near-zero vector was commanded) and is NOT
/// normalized (the caller must reject the command instead of dividing by a
/// near-zero norm, which would otherwise produce an arbitrarily large or
/// NaN unit vector). TEMPORARY-style numerical-safety-only constant, not a
/// physical/atmospheric parameter - same role as AeroModel.hh's
/// vSafeFloor/PropulsionModel.hh's nSafeFloorRevS division-by-zero guards.
constexpr double kMinDirectionNorm = 1e-9;

/// \brief One scheduled 1-cosine gust's parameters, already sign/unit
/// normalized by the caller (WindSystem.cc's OnGustCmd() is responsible for
/// calling NormalizeDirection() BEFORE constructing this struct) - this
/// header never normalizes on its own, keeping EvaluateGust() a pure,
/// side-effect-free function of already-validated inputs.
///
/// V1 LIMITATION (stated explicitly, not silently): only ONE GustState can
/// be represented/active at a time by construction (a single struct
/// instance, not a list/queue) - a new commanded gust is meant to fully
/// REPLACE this struct's contents (see WindSystem::OnGustCmd()), never to
/// be superposed with a still-in-progress one.
struct GustState
{
  /// Unit vector, world frame. Meaningless if `scheduled` is false.
  gz::math::Vector3d directionUnit = gz::math::Vector3d::Zero;
  /// Peak gust speed at the profile midpoint, m/s.
  double amplitudeMps = 0.0;
  /// Absolute sim time (seconds) at which this gust's [0,duration] window
  /// begins.
  double startTimeSec = 0.0;
  /// Seconds; must be > 0 for the gust to ever produce a non-zero output.
  double durationSec = 0.0;
  /// False = no gust has ever been commanded (or a command was rejected as
  /// invalid) - EvaluateGust() always returns Zero in that case, regardless
  /// of the other (don't-care) fields' values.
  bool scheduled = false;
};

/// \brief Normalize a raw (not-necessarily-unit) direction vector for a gust
/// command. \param[in] _raw the raw commanded direction (any nonzero
/// vector; magnitude is discarded - only direction matters, amplitude is a
/// separate, explicit field). \param[out] _validOut set true if `_raw`'s
/// norm is >= kMinDirectionNorm (usable), false otherwise. \return the unit
/// vector along `_raw`, or gz::math::Vector3d::Zero if `_validOut` is false
/// - callers MUST check `_validOut` and reject the command rather than
/// scheduling a gust with a zero/degenerate direction.
inline gz::math::Vector3d NormalizeDirection(
    const gz::math::Vector3d &_raw, bool &_validOut)
{
  const double norm = _raw.Length();
  if (norm < kMinDirectionNorm)
  {
    _validOut = false;
    return gz::math::Vector3d::Zero;
  }
  _validOut = true;
  return _raw / norm;
}

/// \brief The 1-cosine gust envelope (dimensionless, in [0,1]) at time
/// offset `_t` (seconds, measured from the gust's OWN start, i.e. `_t = 0`
/// at the start of the window) within a gust of total duration `_duration`
/// (seconds). Exactly the documented V1 formula:
///
///   envelope(t) = 0.5 * (1 - cos(2*pi*t/duration))   for t in [0, duration]
///   envelope(t) = 0                                   otherwise
///
/// C1-CONTINUITY (value AND derivative exactly zero at both boundaries,
/// confirmed analytically and by the self-test below - "no jerk
/// discontinuity" requirement):
///   envelope(0)        = 0.5*(1-cos(0))      = 0.5*(1-1) = 0
///   envelope(duration) = 0.5*(1-cos(2*pi))   = 0.5*(1-1) = 0
///   d(envelope)/dt     = (pi/duration)*sin(2*pi*t/duration)
///     at t=0:        (pi/duration)*sin(0)      = 0
///     at t=duration: (pi/duration)*sin(2*pi)   = 0
/// Since the function is defined to be identically 0 outside [0,duration]
/// as well, splicing this profile onto a constant background produces a
/// globally C1 (value- and slope-continuous) time series - no instantaneous
/// jump in either wind value or its time-derivative at either edge.
/// Domain-bounded ("no silent extrapolation", CLAUDE.md): any `_t` outside
/// [0, _duration], or a non-positive `_duration`, returns exactly 0.0.
inline double GustEnvelope(double _t, double _duration)
{
  if (_duration <= 0.0 || _t < 0.0 || _t > _duration)
    return 0.0;
  return 0.5 * (1.0 - std::cos(2.0 * M_PI * _t / _duration));
}

/// \brief Evaluate the full gust VELOCITY vector (m/s, world frame) at
/// absolute sim time `_simTimeSec`, given an already-normalized `_gust`.
/// Returns exactly gz::math::Vector3d::Zero if `_gust.scheduled` is false,
/// or if `_simTimeSec` falls outside this gust's own
/// [startTimeSec, startTimeSec+durationSec] window (see GustEnvelope()'s
/// domain-bound behavior above, which this function delegates to directly).
inline gz::math::Vector3d EvaluateGust(
    const GustState &_gust, double _simTimeSec)
{
  if (!_gust.scheduled)
    return gz::math::Vector3d::Zero;
  const double t = _simTimeSec - _gust.startTimeSec;
  const double envelope = GustEnvelope(t, _gust.durationSec);
  return _gust.directionUnit * (_gust.amplitudeMps * envelope);
}

/// \brief Total published wind = steady + gust, a plain vector sum with NO
/// cross-term (documented explicitly per this task's "steady+gust
/// composition is a simple sum with no cross-contamination" self-test
/// requirement): the steady component is completely unaffected by whether
/// or where a gust is active, and the gust component is computed
/// independently of the steady value - this function only ever adds the
/// two already-independently-computed vectors.
inline gz::math::Vector3d ComposeWind(
    const gz::math::Vector3d &_steadyWorld, const gz::math::Vector3d &_gustWorld)
{
  return _steadyWorld + _gustWorld;
}

}  // namespace falcon_v2_wind

#endif  // FALCON_V2_WIND_MODEL_HH_

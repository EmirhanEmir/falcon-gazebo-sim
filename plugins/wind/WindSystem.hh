// =============================================================================
// FALCON V2 - Gazebo Sim Harmonic wind/gust generator System plugin
// =============================================================================
// Owner: aerodynamics specialist agent. Task:
// WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION (2026-08-27).
//
// Scope: STRICTLY a wind-VELOCITY publisher. This plugin generates
// V_wind_world = V_steady + V_gust(t) (world frame, m/s - velocity of the
// AIR MASS, not meteorological "wind coming from" phrasing) and publishes
// it every PreUpdate() tick on the EXISTING /model/falcon_v2/wind topic
// (gz.msgs.Vector3d) - the SAME topic
// plugins/aerodynamics/AerodynamicsSystem.cc and
// plugins/propulsion/PropulsionSystem.cc ALREADY subscribe to and ALREADY
// correctly consume as Vrel = Vbody - Vwind (CONFIRMED by direct read of
// both files for this task - neither file is modified by this task; this
// plugin is purely additive). This is the ONLY wind source in the project -
// no second wind-consumption path is created.
//
// HARD REQUIREMENT (task section 20, validation-checked): this plugin NEVER
// applies a direct force/wrench to any link - no AddWorldForce()/
// AddWorldWrench() call exists anywhere in this class. Its only physical
// output is the published wind velocity vector; all aerodynamic/propulsive
// EFFECT of that wind flows exclusively through the existing relative-
// airflow consumption already implemented in AerodynamicsSystem.cc/
// PropulsionSystem.cc.
//
// V1 LIMITATION (stated explicitly, not silently - see WindModel.hh's
// GustState comment and OnGustCmd() below): only ONE gust may be
// scheduled/in-progress at a time. A new gust command REPLACES any
// in-progress or still-scheduled gust outright - no queueing, no
// superposition of multiple simultaneous gusts.
//
// This class is intentionally thin: all math lives in WindModel.hh (a pure,
// Gazebo-independent header also used by the standalone self-test) -
// mirrors plugins/aerodynamics/AerodynamicsSystem.hh /
// plugins/propulsion/PropulsionSystem.hh's architecture split.
// =============================================================================
#ifndef FALCON_V2_WIND_SYSTEM_HH_
#define FALCON_V2_WIND_SYSTEM_HH_

#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/double_v.pb.h>

#include "WindModel.hh"

namespace falcon_v2_wind
{

class WindSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: WindSystem() = default;
  public: ~WindSystem() override = default;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Live-updatable steady-wind command (gz.msgs.Vector3d, world
  /// frame, m/s). Overwrites this->steadyWindWorld outright - hold-last-
  /// valid-command pattern, same failsafe policy already used by
  /// plugins/actuators/ActuatorSystem::OnCommand() and
  /// plugins/propulsion/PropulsionSystem::OnThrottleLeft/Right() - no
  /// timeout/return-to-default logic. Non-finite components are rejected
  /// (previous valid value held), never propagated as NaN/Inf.
  private: void OnSteadyCmd(const gz::msgs::Vector3d &_msg);

  /// \brief Schedule (or replace) ONE deterministic 1-cosine gust.
  /// gz.msgs.Double_V, FIXED field order, exactly 6 fields:
  ///   [0] dir_x, [1] dir_y, [2] dir_z  - raw direction vector, world
  ///       frame. NOT required to be pre-normalized by the caller - this
  ///       callback normalizes internally via WindModel.hh's
  ///       NormalizeDirection(). A degenerate (near-zero-norm) direction is
  ///       REJECTED (logged; gust NOT scheduled; any previously
  ///       scheduled/in-progress gust is left completely untouched)
  ///       rather than silently producing a NaN or arbitrarily large unit
  ///       vector.
  ///   [3] amplitude_mps                - peak gust speed, m/s (may be
  ///       negative, meaning the gust blows opposite to dir_x/y/z - this is
  ///       a valid, documented way to command a gust "into" the given
  ///       direction rather than negating the direction vector itself).
  ///   [4] start_delay_s                - seconds from THIS COMMAND'S
  ///       RECEIPT (NOT absolute sim time) - see the .cc file's OnGustCmd()
  ///       implementation comment for the exact reference-time mechanism
  ///       and docs/source_of_truth/environment/WIND.md for the rationale
  ///       (lets a caller schedule "N seconds from now" without needing to
  ///       first query the current absolute sim time out-of-band).
  ///   [5] duration_s                   - seconds; MUST be > 0. A
  ///       non-positive value is REJECTED (logged; any previously
  ///       scheduled/in-progress gust is left untouched).
  /// A message with any other field count, or any non-finite field, is
  /// likewise REJECTED (logged) rather than partially applied.
  ///
  /// V1 LIMITATION (see class-level comment above): a new, VALID gust
  /// command unconditionally REPLACES this->gustState in full - it is a
  /// full overwrite, never a merge/queue/superposition with a still-
  /// in-progress gust.
  private: void OnGustCmd(const gz::msgs::Double_V &_msg);

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher windPub;

  /// \brief Persistent steady-wind state (world frame, m/s). Initialized
  /// from the <steady_wind_mps> SDF parameter at Configure() time (default
  /// gz::math::Vector3d::Zero if absent), then live-overwritable via the
  /// steady_cmd topic (OnSteadyCmd()).
  private: gz::math::Vector3d steadyWindWorld{gz::math::Vector3d::Zero};

  /// \brief Persistent single-gust state (see GustState comment,
  /// WindModel.hh). scheduled=false initially, so EvaluateGust() returns
  /// exactly Zero until a valid gust command is received (OnGustCmd()).
  private: GustState gustState{};

  /// \brief Sim time (seconds) as of the most recently completed
  /// PreUpdate() tick - used as OnGustCmd()'s "command receipt" time
  /// reference (see OnGustCmd() doc comment above). Updated once per tick
  /// in PreUpdate(), so it is accurate to within one physics timestep of a
  /// command's true transport-layer arrival time - the same tick-
  /// granularity precedent already accepted by this project's other
  /// asynchronous command topics (e.g. ActuatorSystem's hold-last-valid-
  /// command reads, PropulsionSystem's throttle commands), and, like those,
  /// this class does not add a mutex around the plain double/state writes
  /// below (OnSteadyCmd()/OnGustCmd() write from the transport thread,
  /// PreUpdate() reads/writes from the physics thread) - this mirrors the
  /// exact same already-accepted pattern used by
  /// AerodynamicsSystem::OnWind()/PropulsionSystem::OnWind() and every
  /// other *::On*Cmd() callback in this project, not a new or additional
  /// gap introduced by this plugin.
  private: double lastSimTimeSec{0.0};
};

}  // namespace falcon_v2_wind

#endif  // FALCON_V2_WIND_SYSTEM_HH_

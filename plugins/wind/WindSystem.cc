// =============================================================================
// FALCON V2 - Gazebo Sim Harmonic wind/gust generator System plugin (impl)
// =============================================================================
// See WindSystem.hh for the architecture summary. All formulas live in
// WindModel.hh (pure math, no Gazebo dependency) - this file is only the
// gz-sim Configure()/PreUpdate() plumbing (topic advertise/subscribe,
// SDF parameter reads, publishing every tick). No aerodynamic coefficient,
// control lookup table, actuator parameter, or propulsion parameter is
// read, written, or referenced anywhere in this file.
// =============================================================================
#include "WindSystem.hh"

#include <chrono>
#include <cmath>

#include <gz/plugin/Register.hh>

#include <gz/common/Console.hh>
#include <gz/sim/Model.hh>

using namespace falcon_v2_wind;

//////////////////////////////////////////////////
void WindSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "[FalconV2Wind] Configure() called on an invalid model entity - "
          << "plugin must be attached at the <model> level in model.sdf.\n";
    return;
  }

  // ---- Topic names (all optionally overridable via SDF, mirroring the
  // existing aerodynamics/propulsion/actuator plugins' override pattern -
  // defaults match the EXISTING wind topic those two plugins already
  // subscribe to). ----
  const std::string windTopic =
      _sdf->Get<std::string>("wind_topic", "/model/falcon_v2/wind").first;
  const std::string steadyCmdTopic = _sdf->Get<std::string>(
      "steady_cmd_topic", "/model/falcon_v2/wind/steady_cmd").first;
  const std::string gustCmdTopic = _sdf->Get<std::string>(
      "gust_cmd_topic", "/model/falcon_v2/wind/gust_cmd").first;

  // ---- Steady wind: SDF-configured initial value (world frame, m/s),
  // default zero vector if absent - live-overwritable afterward via
  // OnSteadyCmd(). gz::math::Vector3d is a natively-supported sdf::Param
  // type (same mechanism SDF itself uses for <xyz>/<pose> elements), so no
  // custom parsing is needed here. ----
  this->steadyWindWorld =
      _sdf->Get<gz::math::Vector3d>(
          "steady_wind_mps", gz::math::Vector3d::Zero).first;

  this->windPub = this->node.Advertise<gz::msgs::Vector3d>(windTopic);
  if (!this->windPub)
  {
    gzerr << "[FalconV2Wind] Failed to advertise wind topic '" << windTopic
          << "'.\n";
  }

  if (!this->node.Subscribe(steadyCmdTopic, &WindSystem::OnSteadyCmd, this))
  {
    gzerr << "[FalconV2Wind] Failed to subscribe to steady-wind command "
          << "topic '" << steadyCmdTopic << "'.\n";
  }
  if (!this->node.Subscribe(gustCmdTopic, &WindSystem::OnGustCmd, this))
  {
    gzerr << "[FalconV2Wind] Failed to subscribe to gust command topic '"
          << gustCmdTopic << "'.\n";
  }

  gzmsg << "[FalconV2Wind] Configured. wind topic=" << windTopic
        << " (gz.msgs.Vector3d, world frame, m/s, velocity-of-the-air-mass "
        << "convention, published every PreUpdate() tick - never a direct "
        << "force/wrench). initial steady_wind_mps=" << this->steadyWindWorld
        << " steady_cmd_topic=" << steadyCmdTopic << " (gz.msgs.Vector3d)"
        << " gust_cmd_topic=" << gustCmdTopic
        << " (gz.msgs.Double_V, fixed field order: "
        << "[dir_x,dir_y,dir_z,amplitude_mps,start_delay_s,duration_s])."
        << " V1 limitation: only one gust may be scheduled/in-progress at a "
        << "time - a new gust command replaces any existing one.\n";
}

//////////////////////////////////////////////////
void WindSystem::OnSteadyCmd(const gz::msgs::Vector3d &_msg)
{
  const double x = _msg.x();
  const double y = _msg.y();
  const double z = _msg.z();
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
  {
    // Hold-last-valid-command failsafe (same policy as
    // ActuatorSystem::OnCommand()): never let a NaN/Inf command reach the
    // published wind - the previous valid steady value is left unchanged.
    gzerr << "[FalconV2Wind] Ignored non-finite steady-wind command "
          << "(previous value held).\n";
    return;
  }
  this->steadyWindWorld.Set(x, y, z);
}

//////////////////////////////////////////////////
void WindSystem::OnGustCmd(const gz::msgs::Double_V &_msg)
{
  if (_msg.data_size() != 6)
  {
    gzerr << "[FalconV2Wind] Ignored gust command with " << _msg.data_size()
          << " field(s) - expected exactly 6 "
          << "(dir_x,dir_y,dir_z,amplitude_mps,start_delay_s,duration_s). "
          << "Any previously scheduled/in-progress gust is left unchanged.\n";
    return;
  }

  const gz::math::Vector3d rawDir(_msg.data(0), _msg.data(1), _msg.data(2));
  const double amplitude = _msg.data(3);
  const double startDelay = _msg.data(4);
  const double duration = _msg.data(5);

  if (!std::isfinite(rawDir.X()) || !std::isfinite(rawDir.Y()) ||
      !std::isfinite(rawDir.Z()) || !std::isfinite(amplitude) ||
      !std::isfinite(startDelay) || !std::isfinite(duration))
  {
    gzerr << "[FalconV2Wind] Ignored gust command containing a non-finite "
          << "field. Any previously scheduled/in-progress gust is left "
          << "unchanged.\n";
    return;
  }
  if (duration <= 0.0)
  {
    gzerr << "[FalconV2Wind] Ignored gust command with non-positive "
          << "duration_s=" << duration << " (must be > 0). Any previously "
          << "scheduled/in-progress gust is left unchanged.\n";
    return;
  }

  bool dirValid = false;
  const gz::math::Vector3d dirUnit = NormalizeDirection(rawDir, dirValid);
  if (!dirValid)
  {
    gzerr << "[FalconV2Wind] Ignored gust command with a degenerate "
          << "(near-zero-norm) direction vector (" << rawDir << "). Any "
          << "previously scheduled/in-progress gust is left unchanged.\n";
    return;
  }

  // V1 LIMITATION (WindSystem.hh class comment / OnGustCmd() doc comment):
  // a new, VALID gust command REPLACES this->gustState in full - no
  // queueing, no superposition with a still-in-progress gust.
  //
  // start_delay_s is documented as relative to THIS COMMAND'S RECEIPT, not
  // absolute sim time. The exact reference used is this->lastSimTimeSec -
  // the sim time as of the most recently COMPLETED PreUpdate() tick, which
  // gz-transport updates asynchronously relative to the physics thread (see
  // WindSystem.hh's lastSimTimeSec field comment for the accepted, already-
  // precedented tick-granularity/no-mutex rationale). This choice (vs.
  // requiring the caller to supply an absolute sim time) was made so a
  // human/test-script caller can schedule "N seconds from now" without
  // first querying the current absolute sim time out-of-band.
  GustState newGust;
  newGust.directionUnit = dirUnit;
  newGust.amplitudeMps = amplitude;
  newGust.startTimeSec = this->lastSimTimeSec + startDelay;
  newGust.durationSec = duration;
  newGust.scheduled = true;
  this->gustState = newGust;

  gzmsg << "[FalconV2Wind] Gust scheduled (replacing any previous gust): "
        << "direction_unit=" << dirUnit << " amplitude_mps=" << amplitude
        << " start_sim_time_s=" << newGust.startTimeSec
        << " (receipt_sim_time_s=" << this->lastSimTimeSec
        << " + start_delay_s=" << startDelay << ") duration_s=" << duration
        << ".\n";
}

//////////////////////////////////////////////////
void WindSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager & /*_ecm*/)
{
  if (_info.paused)
    return;

  this->lastSimTimeSec = std::chrono::duration<double>(_info.simTime).count();

  // ---- Pure-math evaluation (WindModel.hh) ----
  const gz::math::Vector3d gustVec =
      EvaluateGust(this->gustState, this->lastSimTimeSec);
  const gz::math::Vector3d totalWind =
      ComposeWind(this->steadyWindWorld, gustVec);

  // ---- Publish every tick (never throttled - AerodynamicsSystem.cc/
  // PropulsionSystem.cc each hold only the LAST received wind message, so a
  // smoothly time-varying gust requires a fresh publish every physics step
  // to actually be seen as smoothly time-varying by those consumers). This
  // is ONLY ever a velocity-vector publish - no AddWorldForce()/
  // AddWorldWrench() call exists anywhere in this file (task section 20
  // hard requirement). ----
  gz::msgs::Vector3d msg;
  msg.set_x(totalWind.X());
  msg.set_y(totalWind.Y());
  msg.set_z(totalWind.Z());
  this->windPub.Publish(msg);
}

GZ_ADD_PLUGIN(
    WindSystem,
    gz::sim::System,
    WindSystem::ISystemConfigure,
    WindSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(WindSystem, "falcon_v2_wind::WindSystem")
GZ_ADD_PLUGIN_ALIAS(WindSystem, "FalconV2Wind")

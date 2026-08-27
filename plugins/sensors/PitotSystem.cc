// =============================================================================
// FALCON V2 - Simulated pitot (airspeed) sensor plugin (implementation)
// =============================================================================
// See PitotSystem.hh for the architecture summary; PitotModel.hh for the
// physical model and the native-sensor-vs-custom-plugin decision record.
// =============================================================================
#include "PitotSystem.hh"

#include <chrono>
#include <cmath>

#include <gz/plugin/Register.hh>

#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>

#include "Frames.hh"
#include "PitotModel.hh"

using namespace falcon_v2_sensors;

namespace
{
constexpr const char *kBaseLinkName = "base_link";
}  // namespace

//////////////////////////////////////////////////
void PitotSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "[FalconV2Pitot] Configure() called on an invalid model entity "
          << "- plugin must be attached at the <model> level in model.sdf.\n";
    return;
  }

  this->baseLinkEntity = model.LinkByName(_ecm, kBaseLinkName);
  if (this->baseLinkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[FalconV2Pitot] Could not find link '" << kBaseLinkName
          << "' - pitot will NOT publish.\n";
    return;
  }

  gz::sim::Link baseLink(this->baseLinkEntity);
  baseLink.EnableVelocityChecks(_ecm, true);

  this->mountOffsetBody = _sdf->Get<gz::math::Vector3d>(
      "mount_offset_body", gz::math::Vector3d(0.168309, 0.0, 0.100000)).first;
  this->rhoKgM3 = _sdf->Get<double>("air_density_rho_kg_m3", this->rhoKgM3).first;
  this->updateRateHz = _sdf->Get<double>("update_rate_hz", this->updateRateHz).first;
  this->noiseStddevMps =
      _sdf->Get<double>("noise_stddev_mps", this->noiseStddevMps).first;

  const std::string windTopic =
      _sdf->Get<std::string>("wind_topic", "/model/falcon_v2/wind").first;
  const std::string airspeedTopic = _sdf->Get<std::string>(
      "airspeed_topic", "/model/falcon_v2/sensors/pitot/airspeed_mps").first;
  const std::string pressureTopic = _sdf->Get<std::string>(
      "pressure_topic",
      "/model/falcon_v2/sensors/pitot/differential_pressure_pa").first;

  if (!this->node.Subscribe(windTopic, &PitotSystem::OnWind, this))
  {
    gzerr << "[FalconV2Pitot] Failed to subscribe to wind topic '"
          << windTopic << "'.\n";
  }

  this->airspeedPub = this->node.Advertise<gz::msgs::Double>(airspeedTopic);
  this->pressurePub = this->node.Advertise<gz::msgs::Double>(pressureTopic);

  gzmsg << "[FalconV2Pitot] Configured. mount_offset_body="
        << this->mountOffsetBody << " (m, FLU, relative to base_link origin)"
        << " air_density_rho_kg_m3=" << this->rhoKgM3
        << " (CITED from aero_v1_config.yaml/propulsion_v1_config.yaml,"
        << " NOT re-derived here) update_rate_hz=" << this->updateRateHz
        << " noise_stddev_mps=" << this->noiseStddevMps
        << " (0.0 = deterministic, OFF) wind_topic=" << windTopic
        << " airspeed_topic=" << airspeedTopic
        << " (gz.msgs.Double, m/s) pressure_topic=" << pressureTopic
        << " (gz.msgs.Double, Pa).\n";
}

//////////////////////////////////////////////////
void PitotSystem::OnWind(const gz::msgs::Vector3d &_msg)
{
  const double x = _msg.x();
  const double y = _msg.y();
  const double z = _msg.z();
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
  {
    gzerr << "[FalconV2Pitot] Ignored non-finite wind message (previous "
          << "value held).\n";
    return;
  }
  this->windWorld.Set(x, y, z);
}

//////////////////////////////////////////////////
void PitotSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || this->baseLinkEntity == gz::sim::kNullEntity)
    return;

  const double dt = std::chrono::duration<double>(_info.dt).count();
  this->timeSinceLastPublishSec += dt;
  const double publishPeriodSec =
      (this->updateRateHz > 0.0) ? (1.0 / this->updateRateHz) : 0.0;
  if (this->timeSinceLastPublishSec < publishPeriodSec)
    return;
  this->timeSinceLastPublishSec = 0.0;

  gz::sim::Link baseLink(this->baseLinkEntity);
  const auto vPointWorldOpt =
      baseLink.WorldLinearVelocity(_ecm, this->mountOffsetBody);
  if (!vPointWorldOpt.has_value())
    return;  // velocity checks not yet enabled this tick - skip, no stale publish

  const gz::math::Vector3d vRelWorld =
      AirRelativeVelocityWorld(*vPointWorldOpt, this->windWorld);
  double airspeedMps = AirspeedMps(vRelWorld);

  if (this->noiseStddevMps > 0.0)
  {
    airspeedMps += this->noiseStddevMps * this->noiseDist(this->noiseRng);
    if (airspeedMps < 0.0)
      airspeedMps = 0.0;  // a real pitot cannot report negative airspeed
  }

  const double pressurePa = DifferentialPressurePa(airspeedMps, this->rhoKgM3);

  gz::msgs::Double airspeedMsg;
  airspeedMsg.set_data(airspeedMps);
  this->airspeedPub.Publish(airspeedMsg);

  gz::msgs::Double pressureMsg;
  pressureMsg.set_data(pressurePa);
  this->pressurePub.Publish(pressureMsg);
}

GZ_ADD_PLUGIN(
    PitotSystem,
    gz::sim::System,
    PitotSystem::ISystemConfigure,
    PitotSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(PitotSystem, "falcon_v2_sensors::PitotSystem")
GZ_ADD_PLUGIN_ALIAS(PitotSystem, "FalconV2Pitot")

// =============================================================================
// FALCON V2 - Simulated magnetometer sensor plugin (implementation)
// =============================================================================
// See MagnetometerSystem.hh for the architecture summary and the full
// root-cause record for why this replaces the native gz-sim-magnetometer-
// system for this project.
// =============================================================================
#include "MagnetometerSystem.hh"

#include <chrono>
#include <cmath>

#include <gz/plugin/Register.hh>

#include <gz/common/Console.hh>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>

using namespace falcon_v2_sensors;

namespace
{
constexpr const char *kBaseLinkName = "base_link";
}  // namespace

//////////////////////////////////////////////////
void MagnetometerSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "[FalconV2Magnetometer] Configure() called on an invalid model "
          << "entity - plugin must be attached at the <model> level in "
          << "model.sdf.\n";
    return;
  }

  this->baseLinkEntity = model.LinkByName(_ecm, kBaseLinkName);
  if (this->baseLinkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[FalconV2Magnetometer] Could not find link '" << kBaseLinkName
          << "' - magnetometer will NOT publish.\n";
    return;
  }

  this->worldMagneticFieldTesla = _sdf->Get<gz::math::Vector3d>(
      "world_magnetic_field_tesla", this->worldMagneticFieldTesla).first;
  this->updateRateHz = _sdf->Get<double>("update_rate_hz", this->updateRateHz).first;

  const std::string magTopic =
      _sdf->Get<std::string>("topic", "/model/falcon_v2/sensors/mag").first;

  this->magPub = this->node.Advertise<gz::msgs::Magnetometer>(magTopic);

  gzmsg << "[FalconV2Magnetometer] Configured. world_magnetic_field_tesla="
        << this->worldMagneticFieldTesla << " (T, world ENU frame,"
        << " SIMULATION_ASSUMPTION - see SENSORS.md sec 4.4)"
        << " update_rate_hz=" << this->updateRateHz
        << " topic=" << magTopic << " (gz.msgs.Magnetometer, field_tesla,"
        << " body FLU frame). Replaces the native gz-sim-magnetometer-"
        << "system, confirmed non-functional on this installation - see"
        << " MagnetometerSystem.hh header comment.\n";
}

//////////////////////////////////////////////////
void MagnetometerSystem::PreUpdate(
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
  const auto worldPoseOpt = baseLink.WorldPose(_ecm);
  if (!worldPoseOpt.has_value())
    return;  // no stale publish

  // Sensor mount pose has zero relative rotation to base_link (see
  // model.sdf/SENSORS.md sec 4.4) - base_link's own orientation IS the
  // sensor's body-frame orientation. World vector -> body-frame vector via
  // the inverse rotation (SAME convention already used for this project's
  // IMU specific-force reading): for v_world fixed, v_body =
  // q.RotateVectorReverse(v_world), where q rotates body vectors into world
  // (q.RotateVector(v_body) == v_world).
  const gz::math::Quaterniond qFluRelEnu = worldPoseOpt->Rot();
  const gz::math::Vector3d fieldBodyFlu =
      qFluRelEnu.RotateVectorReverse(this->worldMagneticFieldTesla);

  gz::msgs::Magnetometer msg;
  gz::msgs::Vector3d *field = msg.mutable_field_tesla();
  field->set_x(fieldBodyFlu.X());
  field->set_y(fieldBodyFlu.Y());
  field->set_z(fieldBodyFlu.Z());

  this->magPub.Publish(msg);
}

GZ_ADD_PLUGIN(
    MagnetometerSystem,
    gz::sim::System,
    MagnetometerSystem::ISystemConfigure,
    MagnetometerSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(MagnetometerSystem, "falcon_v2_sensors::MagnetometerSystem")
GZ_ADD_PLUGIN_ALIAS(MagnetometerSystem, "FalconV2Magnetometer")

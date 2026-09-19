// =============================================================================
// FALCON V2 - Simulated magnetometer sensor Gazebo Sim Harmonic plugin
// =============================================================================
// WHY A CUSTOM PLUGIN, NOT THE NATIVE gz-sim-magnetometer-system: on the
// Gazebo Sim Harmonic installation this project targets, the native
// Magnetometer system's published field_tesla output is a constant
// ~0.32-magnitude vector that does not respond to the world's declared
// <magnetic_field> value at all. The world's field was confirmed to reach
// MagnetometerSensor::SetWorldMagneticField() correctly, so the SDF-to-ECM
// parsing path is not the defect; the sensor's Update() path never runs
// despite it publishing at its configured rate. This is a binary-level
// defect in the installed libraries, not in this project's SDF or config.
// This plugin follows the same architectural precedent already used for the
// pitot sensor (PitotSystem.cc / PitotModel.hh).
//
// SCOPE: reads base_link's world orientation from the ECM (ground truth,
// read-only, components::WorldPose), rotates a fixed, SDF-configurable
// world-frame (ENU) Earth magnetic field vector into the body FLU frame via
// gz::math::Quaterniond::RotateVectorReverse() - the standard "world vector
// expressed in body coordinates" inverse rotation, the same convention used
// for this project's IMU specific-force reading - and publishes
// gz.msgs.Magnetometer (field_tesla), matching the native sensor's message
// type and topic so no downstream consumer needs to change. This plugin
// creates no new command path into aerodynamics, propulsion or actuators:
// it is purely a read-only measurement layer.
//
// The field computation is orientation-only. A magnetometer measures a
// locally-uniform field, not a velocity-dependent quantity, so unlike the
// pitot sensor's mount-offset lever arm this plugin's mount point does not
// enter the computation.
// =============================================================================
#ifndef FALCON_V2_SENSORS_MAGNETOMETER_SYSTEM_HH_
#define FALCON_V2_SENSORS_MAGNETOMETER_SYSTEM_HH_

#include <string>

#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace falcon_v2_sensors
{

class MagnetometerSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: MagnetometerSystem() = default;
  public: ~MagnetometerSystem() override = default;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity baseLinkEntity{gz::sim::kNullEntity};

  /// \brief Fixed Earth magnetic field vector, WORLD frame (this project's
  /// world frame is ENU - see Frames.hh header), Tesla.
  /// SIMULATION_ASSUMPTION: default (5.5645e-6, 22.8758e-6, -42.3884e-6) T
  /// matches this project's already-documented world <magnetic_field> value
  /// (sdformat's own documented world-element default,
  /// /usr/share/sdformat14/1.11/world.sdf) so the
  /// numeric reference does not silently change with this bug fix - only
  /// the mechanism that actually consumes it changes (native system ->
  /// this plugin, which genuinely reads and uses the value below). The real
  /// Falcon V2 flight-location-specific magnetic field remains
  /// DATA_REQUIRED.
  private: gz::math::Vector3d worldMagneticFieldTesla{
      5.5645e-6, 22.8758e-6, -42.3884e-6};

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher magPub;

  /// \brief Update rate (Hz). V1_PROVISIONAL, matches the value already
  /// documented/used for the (now-replaced) native sensor - see
  ///
  private: double updateRateHz{50.0};
  private: double timeSinceLastPublishSec{0.0};
};

}  // namespace falcon_v2_sensors

#endif  // FALCON_V2_SENSORS_MAGNETOMETER_SYSTEM_HH_

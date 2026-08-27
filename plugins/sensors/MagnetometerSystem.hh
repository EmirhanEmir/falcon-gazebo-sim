// =============================================================================
// FALCON V2 - Simulated magnetometer sensor Gazebo Sim Harmonic plugin
// =============================================================================
// Owner: controls-integration. Task:
// SENSOR_MODEL_AND_ARDUPLANE_SITL_PREPARATION bug-fix follow-up (2026-08-27).
//
// WHY A CUSTOM PLUGIN, NOT THE NATIVE gz-sim-magnetometer-system: see
// docs/source_of_truth/sensors/SENSORS.md sec 4.4 for the full decision
// record. Short version: gazebo-testing live-validated
// (docs/test_results/2026-08-27_sensor_model_sitl_preparation.md sec 5) that
// the installed gz-sim8/gz-sensors8 native Magnetometer system's PUBLISHED
// field_tesla output is a constant ~0.32-magnitude vector, completely
// unresponsive to a 100x change in the world's declared <magnetic_field>
// value (ratio exactly 1.0000). This agent independently reproduced that
// with a live gdb instrumentation pass (breakpoint on
// gz::sensors::v8::MagnetometerSensor::SetWorldMagneticField in the
// installed libgz-sensors8-magnetometer.so.8.2.2): the world's correctly-
// declared field DOES reach that setter (confirmed exact match, e.g.
// x=5.5645e-06 y=2.28758e-05 z=-4.23884e-05 T for this world's declared
// value) - i.e. the SDF-to-ECM parsing path is NOT the defect. A follow-up
// breakpoint placed directly on the exact, uniquely-mangled
// MagnetometerSensor::Update(...) symbol (the ONLY copy of that symbol
// loaded in the process, confirmed via /proc/<pid>/maps - no duplicate
// library instance) never fired despite the sensor visibly publishing live
// output at its configured rate for 10+ seconds of wall time. With no
// source .cc available on this machine for either
// libgz-sim8-magnetometer-system.so or libgz-sensors8-magnetometer.so (only
// headers/specs are installed), this is a genuine, non-debuggable-from-
// source binary-level defect in this Gazebo Sim Harmonic installation, not
// a defect in this project's own SDF/config - a genuine dead end for the
// native path per this task's own stated fallback criterion. This plugin
// follows the exact same architectural precedent already used for the
// pitot sensor (PitotSystem.cc/PitotModel.hh) for the same reason (native
// path confirmed unusable for this project's need).
//
// SCOPE: reads base_link's own world orientation from the ECM (ground
// truth, read-only, components::WorldPose - populated unconditionally by
// gz-sim, no explicit "EnableXChecks()" opt-in required, unlike velocity),
// rotates a fixed, SDF-configurable world-frame (ENU) Earth magnetic field
// vector into the body FLU frame via
// gz::math::Quaterniond::RotateVectorReverse() (the standard "world vector
// expressed in body coordinates" inverse-rotation, the SAME convention
// already used for this project's IMU specific-force reading), and
// publishes gz.msgs.Magnetometer (field_tesla, matching the native sensor's
// own message type/topic so no downstream consumer needs to change). This
// plugin creates NO new command path into aerodynamics/propulsion/
// actuators - it is purely a read-only measurement layer, mirroring
// PitotSystem.cc.
//
// The field computation is orientation-only (no lever-arm/velocity
// dependence - a magnetometer measures a locally-uniform field, not a
// velocity-dependent quantity), so unlike the pitot sensor's mount-offset
// lever arm, this plugin's "mount point" is documentation-only (kept for
// SENSORS.md consistency with the other 4 co-located sensors) and does not
// enter the field computation.
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
  /// /usr/share/sdformat14/1.11/world.sdf - see SENSORS.md sec 4.4) so the
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
  /// SENSORS.md sec 6.
  private: double updateRateHz{50.0};
  private: double timeSinceLastPublishSec{0.0};
};

}  // namespace falcon_v2_sensors

#endif  // FALCON_V2_SENSORS_MAGNETOMETER_SYSTEM_HH_

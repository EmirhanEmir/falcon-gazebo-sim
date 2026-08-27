// =============================================================================
// FALCON V2 - Simulated pitot (airspeed) sensor Gazebo Sim Harmonic plugin
// =============================================================================
// Owner: controls-integration. Task:
// SENSOR_MODEL_AND_ARDUPLANE_SITL_PREPARATION (2026-08-27).
//
// Scope: a custom sensor-layer System plugin (NOT the native gz-sim
// air_speed sensor - see PitotModel.hh header comment / SENSORS.md sec 3
// for why). Reads base_link's own world-frame velocity from the ECM (ground
// truth, read-only) and this project's existing
// /model/falcon_v2/wind topic, computes airspeed/differential pressure via
// PitotModel.hh, and publishes the result. This plugin creates NO new
// command path into aerodynamics/propulsion/actuators - it is purely a
// read-only measurement layer, mirroring this project's existing
// sensor/diagnostics-only plugin outputs.
// =============================================================================
#ifndef FALCON_V2_SENSORS_PITOT_SYSTEM_HH_
#define FALCON_V2_SENSORS_PITOT_SYSTEM_HH_

#include <random>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace falcon_v2_sensors
{

class PitotSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: PitotSystem() = default;
  public: ~PitotSystem() override = default;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Hold-last-valid-command wind update (same failsafe policy as
  /// every other *::On*Cmd() in this project - non-finite rejected,
  /// previous value held).
  private: void OnWind(const gz::msgs::Vector3d &_msg);

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity baseLinkEntity{gz::sim::kNullEntity};

  /// \brief Sensor mount point, expressed as an offset from base_link's own
  /// origin, in the body-fixed (FLU) frame - see model.sdf/SENSORS.md for
  /// the documented choice (co-located with the Gazebo/CAD CG,
  /// CLAUDE.md-authoritative point (0.168309, 0, 0.100000) m).
  private: gz::math::Vector3d mountOffsetBody{gz::math::Vector3d::Zero};

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher airspeedPub;
  private: gz::transport::Node::Publisher pressurePub;

  private: gz::math::Vector3d windWorld{gz::math::Vector3d::Zero};

  /// \brief Air density (kg/m^3), CITED (not invented) from
  /// docs/source_of_truth/aerodynamics/aero_v1_config.yaml
  /// environment.air_density_rho_kg_m3 via the <air_density_rho_kg_m3> SDF
  /// override (default matches that file's CONFIRMED 1.225 value - see
  /// model.sdf plugin block comment).
  private: double rhoKgM3{1.225};

  /// \brief Update rate (Hz). V1_PROVISIONAL - see SENSORS.md sec 6.
  private: double updateRateHz{10.0};
  private: double timeSinceLastPublishSec{0.0};

  /// \brief V1_PROVISIONAL_SENSOR_NOISE: optional zero-mean Gaussian noise
  /// on the published airspeed (m/s stddev). Default 0.0 = OFF
  /// (deterministic mode, the default/validated mode per task instruction).
  /// NOT a Cube Orange or any real pitot sensor's characterized noise
  /// figure - an explicitly-flagged placeholder only, off by default.
  private: double noiseStddevMps{0.0};
  private: std::mt19937 noiseRng{12345u};  // fixed seed - deterministic even when enabled
  private: std::normal_distribution<double> noiseDist{0.0, 1.0};
};

}  // namespace falcon_v2_sensors

#endif  // FALCON_V2_SENSORS_PITOT_SYSTEM_HH_

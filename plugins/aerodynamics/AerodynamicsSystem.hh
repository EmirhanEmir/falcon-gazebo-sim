// =============================================================================
// FALCON V2 - Gazebo Sim Harmonic aerodynamics System plugin
// =============================================================================
// Scope: strictly aerodynamic force/moment physics. No propulsion, no
// control actuation (this system READS joint positions, it does not write
// them), no ArduPilot/SITL. See plugins/aerodynamics/README.md for build
// instructions.
//
// The class is intentionally thin: all math lives in AeroModel.hh, a pure,
// Gazebo-independent header. This class's only job is to read Gazebo state
// (link velocity/pose, joint positions, wind topic), build an AeroState, call
// falcon_v2_aero::ComputeAero(), apply the resulting force/moment to
// base_link, and optionally publish diagnostics.
// =============================================================================
#ifndef FALCON_V2_AERODYNAMICS_SYSTEM_HH_
#define FALCON_V2_AERODYNAMICS_SYSTEM_HH_

#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>

#include "AeroModel.hh"

namespace falcon_v2_aero
{

class AerodynamicsSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: AerodynamicsSystem() = default;
  public: ~AerodynamicsSystem() override = default;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Wind topic subscriber callback. Wind velocity is expressed in
  /// the WORLD frame, m/s. Defaults to zero wind if no message has ever
  /// been received.
  private: void OnWind(const gz::msgs::Vector3d &_msg);

  /// \brief Load config/aero_v1_config.yaml
  /// (path given by the <config_path> SDF parameter) into this->config.
  /// \return true on success.
  private: bool LoadConfig(const std::string &_path);

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity baseLinkEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity leftAileronJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rightAileronJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity leftElevatorJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rightElevatorJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rudderJoint{gz::sim::kNullEntity};

  private: AeroConfig config;
  private: bool validConfig{false};

  private: gz::transport::Node node;
  private: gz::math::Vector3d windWorld{gz::math::Vector3d::Zero};

  /// \brief Diagnostics publisher, see PublishDiagnostics(). Topic/rate
  /// documented in and printed once at Configure time.
  private: gz::transport::Node::Publisher diagPub;
  private: double diagRateHz{20.0};
  private: double diagAccumulator{0.0};
};

}  // namespace falcon_v2_aero

#endif  // FALCON_V2_AERODYNAMICS_SYSTEM_HH_

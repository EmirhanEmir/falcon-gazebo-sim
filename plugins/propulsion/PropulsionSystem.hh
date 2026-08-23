// =============================================================================
// FALCON V2 - Gazebo Sim Harmonic propulsion System plugin
// =============================================================================
// Owner: propulsion specialist agent. Task: PROPULSION_V1_IMPLEMENTATION
// (2026-08-23).
//
// Scope: STRICTLY the motor/ESC/propeller physics chain (throttle ->
// electrical -> motor torque -> rotor angular dynamics -> RPM -> advance
// ratio -> APC Ct/Cp -> thrust/torque -> reaction torque -> force/moment at
// the real hub). No aerodynamic-coefficient physics (plugins/aerodynamics/
// is untouched), no control-surface actuation, no ArduPilot/SITL.
//
// This class is intentionally thin: all math lives in PropulsionModel.hh (a
// pure, Gazebo-independent header also used by the standalone self-test).
// This class's only job is: read Gazebo state (link velocity/pose, wind
// topic, throttle topics) -> maintain the two explicit omega_left/
// omega_right ODE states -> call PropulsionModel.hh -> apply the resulting
// force/wrench to base_link at the real hub -> drive the visual prop
// joints' angular velocity from the SAME state -> publish diagnostics.
// Mirrors plugins/aerodynamics/AerodynamicsSystem.hh's structure.
// =============================================================================
#ifndef FALCON_V2_PROPULSION_SYSTEM_HH_
#define FALCON_V2_PROPULSION_SYSTEM_HH_

#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include "PropulsionModel.hh"

namespace falcon_v2_propulsion
{

class PropulsionSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: PropulsionSystem() = default;
  public: ~PropulsionSystem() override = default;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  private: void OnThrottleLeft(const gz::msgs::Double &_msg);
  private: void OnThrottleRight(const gz::msgs::Double &_msg);

  /// \brief Wind topic subscriber, same topic/frame convention as
  /// AerodynamicsSystem.cc's OnWind() (gz.msgs.Vector3d, world frame, m/s) -
  /// read-only here, used for relative-airflow V_axial. Zero until a
  /// publisher exists.
  private: void OnWind(const gz::msgs::Vector3d &_msg);

  /// \brief Load docs/source_of_truth/propulsion/propulsion_v1_config.yaml
  /// (path given by the <config_path> SDF parameter) and the parsed APC
  /// table CSV it points to.
  /// \return true on success.
  private: bool LoadConfig(const std::string &_path);

  /// \brief Runs the full per-motor chain for one PreUpdate tick: electrical
  /// -> ODE step -> aero loading -> force/wrench application -> joint
  /// position drive (cosmetic) -> diagnostics accumulation. Called once per
  /// motor.
  private: void StepMotor(
      gz::sim::EntityComponentManager &_ecm,
      double dt,
      const gz::math::Quaterniond &_worldRot,
      const gz::math::Vector3d &_hubOffsetBody,
      double _rotationSign,
      double _throttleCmd,
      double &_omegaOwnState,
      double &_thetaOwnState,
      gz::sim::Entity _propJoint,
      std::vector<double> &_diagOut);

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity baseLinkEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity leftPropJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rightPropJoint{gz::sim::kNullEntity};

  private: MotorConfig motorCfg;
  private: BatteryConfig batteryCfg;
  private: PropellerConfig propCfg;
  private: PhysicsConfig physicsCfg;
  private: ApcTable apcTable;

  private: gz::math::Vector3d leftHubBody{gz::math::Vector3d::Zero};
  private: gz::math::Vector3d rightHubBody{gz::math::Vector3d::Zero};
  private: gz::math::Vector3d thrustAxisBody{1.0, 0.0, 0.0};
  private: double rotationSignLeft{1.0};
  private: double rotationSignRight{-1.0};

  /// \brief Explicit, plugin-owned rotor angular-velocity states (rad/s,
  /// each motor's OWN "positive = normal spin-up" frame - see
  /// PropulsionModel.hh header note and propulsion_v1_config.yaml's
  /// "rotation" block). Integrated every PreUpdate tick via
  /// IntegrateRotorStep() - NEVER set instantaneously, NEVER driven by
  /// Gazebo's native joint-torque integration.
  private: double omegaLeft{0.0};
  private: double omegaRight{0.0};

  /// \brief Cosmetic-only visual prop-joint angular position (rad, same
  /// own-frame/rotation-sign convention as the joint velocity it is
  /// integrated from). NOT a physics state: never read by, or fed back
  /// into, PropulsionModel.hh/IntegrateRotorStep(); exists solely so
  /// left_prop/right_prop's mesh visually rotates in sync with omegaLeft/
  /// omegaRight. Integrated once per PreUpdate tick (theta += omega*dt) and
  /// written to the joint via Joint::ResetPosition() - a direct kinematic
  /// state write - specifically to AVOID Joint::SetVelocity(), which per
  /// gz/sim/Joint.hh's own documented contract has the physics engine
  /// "compute the necessary joint torque for the commanded velocity and
  /// apply it in the next simulation step", i.e. it is a force-based servo
  /// that would inject an additional, undocumented torque onto base_link
  /// through the joint - duplicating the explicit AddWorldWrench()
  /// reaction-torque call below during any transient (dOmega/dt != 0).
  /// ResetPosition()/ResetVelocity() are documented as direct internal
  /// joint-state writes ("does not modify... instead the physics engine
  /// computes...") - no constraint-solver force is applied to achieve them.
  /// Same technique already used by tests/gazebo/scripts/aero_lib.py's
  /// pin_child_joints() (reset_position()/reset_velocity()) for exactly
  /// this "drive a joint kinematically, not dynamically" need.
  private: double thetaLeft{0.0};
  private: double thetaRight{0.0};

  private: bool validConfig{false};

  private: gz::transport::Node node;
  private: gz::math::Vector3d windWorld{gz::math::Vector3d::Zero};
  private: double throttleLeftCmd{0.0};
  private: double throttleRightCmd{0.0};

  private: gz::transport::Node::Publisher diagPub;
  private: double diagRateHz{20.0};
  private: double diagAccumulator{0.0};
};

}  // namespace falcon_v2_propulsion

#endif  // FALCON_V2_PROPULSION_SYSTEM_HH_

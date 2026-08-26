// =============================================================================
// FALCON V2 - Gazebo Sim Harmonic actuator/servo System plugin
// =============================================================================
// Owner: controls-integration specialist agent. Task: ACTUATOR_SERVO_MODEL_V1
// (2026-08-23).
//
// Scope: STRICTLY the physical actuator/servo command chain (actuator
// command -> rate-limited/clamped setpoint -> effort-bounded torque -> real
// Gazebo joint motion) for the 5 control-surface joints (left_aileron_joint,
// right_aileron_joint, left_elevator_joint, right_elevator_joint,
// rudder_joint). This REPLACES the previous test-script-only direct
// position injection (e.g. tests/gazebo/scripts/aero_lib.py's manual
// ResetPosition() pokes) as the actual actuation mechanism; it does not
// change how plugins/aerodynamics/AerodynamicsSystem.cc reads those same 5
// joint positions (unmodified, still gz::sim::Joint(e).Position(_ecm) each
// tick) and does not implement ArduPlane/MAVLink/SITL/TECS - this plugin
// ends at the physical actuator interface.
//
// This class is intentionally thin: all math lives in ActuatorModel.hh (a
// pure, Gazebo-independent header also used by the standalone self-test).
// This class's only job is: read 5 per-surface command topics -> maintain 5
// persistent rate-limited setpoint states -> read each joint's actual
// position/velocity from the ECM -> compute a bounded PD torque via
// ActuatorModel.hh -> apply it via gz::sim::Joint::SetForce() (genuine
// joint-space torque, integrated by gz-sim's own physics engine - never an
// instantaneous ResetPosition() teleport for these 5 real control-surface
// joints) -> also apply gz::sim::Joint::SetVelocityLimits()/
// SetEffortLimits() once at Configure() (physics-engine-level hard
// constraints, see ActuatorModel.hh header comment for why this is
// necessary given the current TEMPORARY_NUMERICAL_MASS placeholder link
// inertia) -> publish diagnostics. Mirrors
// plugins/aerodynamics/AerodynamicsSystem.hh / plugins/propulsion/
// PropulsionSystem.hh's Configure()/PreUpdate() structure.
// =============================================================================
#ifndef FALCON_V2_ACTUATOR_SYSTEM_HH_
#define FALCON_V2_ACTUATOR_SYSTEM_HH_

#include <array>
#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>

#include "ActuatorModel.hh"

namespace falcon_v2_actuators
{

/// \brief The 5 control-surface actuators, in a fixed order used
/// consistently for entity arrays, config arrays, and diagnostics field
/// order (see Configure()'s logged field-order gzmsg line).
enum SurfaceIndex
{
  kLeftAileron = 0,
  kRightAileron = 1,
  kLeftElevator = 2,
  kRightElevator = 3,
  kRudder = 4,
  kNumSurfaces = 5
};

class ActuatorSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: ActuatorSystem() = default;
  public: ~ActuatorSystem() override = default;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Per-surface command topic callback. Physical-angle-radians
  /// interface (see docs/source_of_truth/controls/actuator_v1_config.yaml
  /// "command_interface" block). Failsafe policy: HOLD-LAST-VALID-COMMAND -
  /// this callback simply overwrites the persistent commandRad[idx] state;
  /// with no new message, the state is left unchanged (no timeout/return-
  /// to-neutral logic), which is the documented V1 policy.
  private: void OnCommand(std::size_t idx, const gz::msgs::Double &_msg);

  /// \brief Load docs/source_of_truth/controls/actuator_v1_config.yaml
  /// (path given by the <config_path> SDF parameter) into this->config[].
  /// \return true on success.
  private: bool LoadConfig(const std::string &_path);

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: std::array<gz::sim::Entity, kNumSurfaces> jointEntity{};
  private: std::array<std::string, kNumSurfaces> jointName{};

  private: std::array<ActuatorConfig, kNumSurfaces> config{};
  private: bool validConfig{false};

  /// \brief Persistent per-surface state. commandRad = last received raw
  /// command (rad, hold-last-valid, defaults to 0.0 = neutral until
  /// commanded). setpointRad = rate-limited internal target state (also
  /// starts at 0.0 = neutral - "no startup kick" requirement). integralRadS
  /// = PidEffort()'s integral-of-error accumulator (rad*s), added
  /// 2026-08-24 for the gravity-droop fix (see ActuatorModel.hh's Stage 3
  /// header note) - also starts at 0.0, consistent with the same
  /// "no startup kick" requirement (zero accumulated error at t=0).
  private: std::array<double, kNumSurfaces> commandRad{};
  private: std::array<double, kNumSurfaces> setpointRad{};
  private: std::array<double, kNumSurfaces> integralRadS{};

  private: gz::transport::Node node;

  private: gz::transport::Node::Publisher diagPub;
  private: double diagRateHz{20.0};
  private: double diagAccumulator{0.0};
};

}  // namespace falcon_v2_actuators

#endif  // FALCON_V2_ACTUATOR_SYSTEM_HH_

// =============================================================================
// FALCON V2 - Gazebo Sim Harmonic actuator/servo System plugin (implementation)
// =============================================================================
// See ActuatorSystem.hh for the architecture summary. Every numeric constant
// used here comes from docs/source_of_truth/controls/actuator_v1_config.yaml
// (loaded at Configure() time) - no coefficient is hardcoded in this file.
// Joint NAMES below are structural identifiers (not physics magic numbers)
// matching model/model.sdf exactly, as verified by direct grep of that file
// for this task.
// =============================================================================
#include "ActuatorSystem.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <vector>

#include <gz/plugin/Register.hh>

#include <gz/common/Console.hh>
#include <gz/math/Vector2.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <gz/msgs/double_v.pb.h>

#include <yaml-cpp/yaml.h>

using namespace falcon_v2_actuators;

namespace
{
// Structural identifiers (CONFIRMED from model/model.sdf, cross-checked by
// direct grep for this task - not aerodynamic/servo coefficients, so these
// are not subject to the "no magic numbers" coefficient rule; they are
// names). Order MUST match the SurfaceIndex enum in ActuatorSystem.hh and
// the actuator_v1_config.yaml "surfaces" map keys.
constexpr std::array<const char *, kNumSurfaces> kJointNames = {
    "left_aileron_joint", "right_aileron_joint", "left_elevator_joint",
    "right_elevator_joint", "rudder_joint"};
constexpr std::array<const char *, kNumSurfaces> kYamlSurfaceKeys = {
    "left_aileron", "right_aileron", "left_elevator", "right_elevator",
    "rudder"};
constexpr std::array<const char *, kNumSurfaces> kDefaultCmdTopics = {
    "/model/falcon_v2/actuators/left_aileron/cmd_rad",
    "/model/falcon_v2/actuators/right_aileron/cmd_rad",
    "/model/falcon_v2/actuators/left_elevator/cmd_rad",
    "/model/falcon_v2/actuators/right_elevator/cmd_rad",
    "/model/falcon_v2/actuators/rudder/cmd_rad"};
// SDF <plugin> parameter names for overriding each default command topic
// above (optional - see model.sdf's actuator <plugin> block; each falls
// back to the corresponding kDefaultCmdTopics entry if absent, same pattern
// as PropulsionSystem.cc's left_throttle_topic/right_throttle_topic).
constexpr std::array<const char *, kNumSurfaces> kCmdTopicSdfParams = {
    "left_aileron_cmd_topic", "right_aileron_cmd_topic",
    "left_elevator_cmd_topic", "right_elevator_cmd_topic",
    "rudder_cmd_topic"};
}  // namespace

//////////////////////////////////////////////////
void ActuatorSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "[FalconV2Actuators] Configure() called on an invalid model "
          << "entity - plugin must be attached at the <model> level in "
          << "model.sdf.\n";
    return;
  }

  // ---- Load the structured servo/actuator dataset (source of truth) ----
  const std::string configPath = _sdf->Get<std::string>("config_path", "").first;
  if (configPath.empty())
  {
    gzerr << "[FalconV2Actuators] No <config_path> SDF parameter given - "
          << "cannot load docs/source_of_truth/controls/"
          << "actuator_v1_config.yaml. Control surfaces will NOT move.\n";
    return;
  }
  this->validConfig = this->LoadConfig(configPath);
  if (!this->validConfig)
  {
    gzerr << "[FalconV2Actuators] Failed to load config from '" << configPath
          << "' - control surfaces will NOT move.\n";
    return;
  }

  this->diagRateHz = _sdf->Get<double>("diagnostics_rate_hz", this->diagRateHz).first;
  const std::string diagTopic = _sdf->Get<std::string>(
      "diagnostics_topic", "/model/falcon_v2/actuators/diagnostics").first;

  // ---- Resolve the 5 joint entities ----
  std::array<std::string, kNumSurfaces> cmdTopics{};
  for (std::size_t i = 0; i < kNumSurfaces; ++i)
  {
    this->jointName[i] = kJointNames[i];
    this->jointEntity[i] = model.JointByName(_ecm, this->jointName[i]);
    if (this->jointEntity[i] == gz::sim::kNullEntity)
    {
      gzerr << "[FalconV2Actuators] Could not find joint '"
            << this->jointName[i] << "' - this surface will NOT be "
            << "actuated.\n";
      continue;
    }

    gz::sim::Joint joint(this->jointEntity[i]);
    joint.EnablePositionCheck(_ecm, true);
    joint.EnableVelocityCheck(_ecm, true);

    // ---- Physics-engine-level hard constraints (defense in depth) ----
    // See ActuatorModel.hh's header comment for why this is NOT optional
    // given the current TEMPORARY_NUMERICAL_MASS placeholder link inertia
    // (model/model.sdf): the PD torque below is software-clamped to
    // +/-max_effort already, but a real-servo-scale torque acting on a
    // near-massless placeholder link could otherwise integrate to an
    // extremely large velocity within a single physics step. Setting the
    // joint's own velocity/effort limits gives the physics engine itself a
    // hard constraint on the REAL joint, independent of that mismatch.
    joint.SetVelocityLimits(
        _ecm, {gz::math::Vector2d(-this->config[i].maxRateRadS,
                                   this->config[i].maxRateRadS)});
    joint.SetEffortLimits(
        _ecm, {gz::math::Vector2d(-this->config[i].maxEffortNm,
                                   this->config[i].maxEffortNm)});

    // ---- Command topic subscription (hold-last-valid-command failsafe -
    // see ActuatorSystem.hh's OnCommand() doc comment). Topic optionally
    // overridable via SDF (mirrors PropulsionSystem.cc's throttle-topic
    // override pattern), defaulting to kDefaultCmdTopics[i]. ----
    const std::string cmdTopic = _sdf->Get<std::string>(
        kCmdTopicSdfParams[i], kDefaultCmdTopics[i]).first;
    cmdTopics[i] = cmdTopic;
    std::function<void(const gz::msgs::Double &)> cb =
        [this, i](const gz::msgs::Double &_msg) { this->OnCommand(i, _msg); };
    if (!this->node.Subscribe(cmdTopic, cb))
    {
      gzerr << "[FalconV2Actuators] Failed to subscribe to command topic '"
            << cmdTopic << "' for surface '" << this->jointName[i] << "'.\n";
    }

    // Neutral start - no command ever received yet (CommandRad[i] already
    // zero-initialized by ActuatorSystem.hh's in-class member initializer;
    // restated here defensively, not a behavior change).
    this->commandRad[i] = 0.0;
    this->setpointRad[i] = 0.0;
    this->integralRadS[i] = 0.0;
  }

  this->diagPub = this->node.Advertise<gz::msgs::Double_V>(diagTopic);

  gzmsg << "[FalconV2Actuators] Configured. config=" << configPath
        << " diagnostics topic=" << diagTopic
        << " (order per surface, left_aileron/right_aileron/left_elevator/"
        << "right_elevator/rudder: cmd_rad, target_clamped_rad, setpoint_rad,"
        << " actual_angle_rad, actual_rate_rad_s, target_clamp_active,"
        << " effort_clamp_active - 7 fields x 5 surfaces = 35 total)"
        << " @ " << this->diagRateHz << " Hz. Command topics: "
        << cmdTopics[0] << ", " << cmdTopics[1] << ", "
        << cmdTopics[2] << ", " << cmdTopics[3] << ", "
        << cmdTopics[4] << ".\n";
}

//////////////////////////////////////////////////
bool ActuatorSystem::LoadConfig(const std::string &_path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(_path);
  }
  catch (const std::exception &e)
  {
    gzerr << "[FalconV2Actuators] YAML load error: " << e.what() << "\n";
    return false;
  }

  try
  {
    auto lawNode = root["control_law"];
    const double kp = lawNode["kp_nm_per_rad"].as<double>();
    const double kd = lawNode["kd_nm_per_rad_s"].as<double>();
    // ki_nm_per_rad_s added 2026-08-24 (gravity-droop fix). Read with a
    // 0.0 default (not lawNode["..."].as<double>() unconditionally) so an
    // older config file missing this key degrades to the pre-fix pure-P(D)
    // behavior (inert, loud-by-absence-in-review) rather than failing to
    // load entirely - matches ActuatorConfig::ki's own documented default.
    const double ki =
        lawNode["ki_nm_per_rad_s"] ? lawNode["ki_nm_per_rad_s"].as<double>()
                                    : 0.0;
    // sp_weight_b added 2026-08-24 (round-2 OVERSHOOT_AFTER_STEP fix). Same
    // graceful-degradation pattern as ki above: an older config file missing
    // this key reads as 1.0 (ActuatorConfig::spWeightB's own documented
    // default), which is mathematically identical to the pre-round-2 P-term
    // - not a silent behavior change for a config that predates this fix.
    const double spWeightB =
        lawNode["sp_weight_b"] ? lawNode["sp_weight_b"].as<double>() : 1.0;

    auto surfacesNode = root["surfaces"];
    for (std::size_t i = 0; i < kNumSurfaces; ++i)
    {
      auto s = surfacesNode[kYamlSurfaceKeys[i]];
      ActuatorConfig cfg;
      cfg.minAngleRad = s["min_angle_rad"].as<double>();
      cfg.maxAngleRad = s["max_angle_rad"].as<double>();
      cfg.maxRateRadS = s["max_rate_rad_s"].as<double>();
      cfg.maxEffortNm = s["max_effort_nm"].as<double>();
      cfg.kp = kp;
      cfg.kd = kd;
      cfg.ki = ki;
      cfg.spWeightB = spWeightB;
      this->config[i] = cfg;
    }
  }
  catch (const std::exception &e)
  {
    gzerr << "[FalconV2Actuators] YAML parse error (missing/invalid key): "
          << e.what() << "\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void ActuatorSystem::OnCommand(std::size_t idx, const gz::msgs::Double &_msg)
{
  if (idx >= kNumSurfaces) return;  // defensive only
  const double v = _msg.data();
  if (!std::isfinite(v))
  {
    // Never let a NaN/Inf command reach the setpoint chain - hold the
    // previous valid command instead (consistent with the hold-last-valid
    // failsafe policy; an invalid message is treated the same as no
    // message).
    gzerr << "[FalconV2Actuators] Ignored non-finite command on surface "
          << idx << ".\n";
    return;
  }
  this->commandRad[idx] = v;
}

//////////////////////////////////////////////////
void ActuatorSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || !this->validConfig)
    return;

  const double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0.0)
    return;

  std::vector<double> diag;
  diag.reserve(kNumSurfaces * 7);

  for (std::size_t i = 0; i < kNumSurfaces; ++i)
  {
    if (this->jointEntity[i] == gz::sim::kNullEntity)
    {
      // Joint missing (already reported at Configure()) - emit zeros so the
      // diagnostics vector stays a fixed, documented length/order.
      for (int f = 0; f < 7; ++f) diag.push_back(0.0);
      continue;
    }

    gz::sim::Joint joint(this->jointEntity[i]);
    const auto posOpt = joint.Position(_ecm);
    const auto velOpt = joint.Velocity(_ecm);
    if (!posOpt.has_value() || posOpt->empty() ||
        !velOpt.has_value() || velOpt->empty())
    {
      // Position/velocity checks not yet enabled for this step (first
      // tick(s) after Configure()) - skip this surface this step rather
      // than use stale/garbage data or apply an uninformed torque. The
      // setpoint state itself is left unchanged (still zero/last value) -
      // this cannot itself be a jump, it is simply "no motion commanded
      // yet".
      for (int f = 0; f < 7; ++f) diag.push_back(0.0);
      continue;
    }
    const double actualAngle = (*posOpt)[0];
    const double actualRate = (*velOpt)[0];

    // ---- Stage 1+2: travel-clamped, rate-limited setpoint ----
    const SetpointStepResult step = StepSetpoint(
        this->commandRad[i], this->setpointRad[i], this->config[i], dt);
    this->setpointRad[i] = step.setpointRad;

    // ---- Stage 3: effort-bounded PID torque, applied via SetForce() (real
    // joint-space torque, integrated by gz-sim's own physics engine - never
    // ResetPosition()/a teleport for these 5 real actuated joints). Integral
    // term added 2026-08-24 (gravity-droop fix, see ActuatorModel.hh) -
    // integralRadS[i] is persistent per-surface state, exactly like
    // setpointRad[i] above; targetClampActive feeds PidEffort()'s
    // anti-windup freeze condition directly from Stage 1's own result. ----
    const EffortResult eff = PidEffort(
        step.setpointRad, actualAngle, actualRate, this->integralRadS[i], dt,
        step.targetWasClamped, this->config[i]);
    this->integralRadS[i] = eff.integralRadS;
    joint.SetForce(_ecm, {eff.torqueNm});

    diag.push_back(this->commandRad[i]);
    diag.push_back(step.targetClampedRad);
    diag.push_back(step.setpointRad);
    diag.push_back(actualAngle);
    diag.push_back(actualRate);
    diag.push_back(step.targetWasClamped ? 1.0 : 0.0);
    diag.push_back(eff.effortClamped ? 1.0 : 0.0);
  }

  this->diagAccumulator += dt;
  const double period = (this->diagRateHz > 0.0) ? (1.0 / this->diagRateHz) : -1.0;
  if (period > 0.0 && this->diagAccumulator >= period)
  {
    this->diagAccumulator = 0.0;
    gz::msgs::Double_V msg;
    for (double v : diag) msg.add_data(v);
    this->diagPub.Publish(msg);
  }
}

GZ_ADD_PLUGIN(
    ActuatorSystem,
    gz::sim::System,
    ActuatorSystem::ISystemConfigure,
    ActuatorSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ActuatorSystem, "falcon_v2_actuators::ActuatorSystem")
GZ_ADD_PLUGIN_ALIAS(ActuatorSystem, "FalconV2Actuators")

// =============================================================================
// FALCON V2 - Gazebo Sim Harmonic propulsion System plugin (implementation)
// =============================================================================
// See PropulsionSystem.hh for the architecture summary. Every numeric
// constant used here comes from docs/source_of_truth/propulsion/
// propulsion_v1_config.yaml (loaded at Configure() time) - no coefficient is
// hardcoded in this file. Joint/link NAMES below are structural identifiers
// (not physics magic numbers) matching model/model.sdf exactly.
// =============================================================================
#include "PropulsionSystem.hh"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <gz/plugin/Register.hh>

#include <gz/common/Console.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <gz/msgs/double_v.pb.h>

#include <yaml-cpp/yaml.h>

using namespace falcon_v2_propulsion;

namespace
{
constexpr const char *kBaseLinkName = "base_link";
constexpr const char *kLeftPropJoint = "left_prop_joint";
constexpr const char *kRightPropJoint = "right_prop_joint";
}  // namespace

//////////////////////////////////////////////////
void PropulsionSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "[FalconV2Propulsion] Configure() called on an invalid model "
          << "entity - plugin must be attached at the <model> level in "
          << "model.sdf.\n";
    return;
  }

  const auto configPathPair = _sdf->Get<std::string>("config_path", "");
  const std::string configPath = configPathPair.first;
  if (configPath.empty())
  {
    gzerr << "[FalconV2Propulsion] No <config_path> SDF parameter given - "
          << "cannot load propulsion_v1_config.yaml. Propulsion forces will "
          << "NOT be applied.\n";
    return;
  }
  this->validConfig = this->LoadConfig(configPath);
  if (!this->validConfig)
  {
    gzerr << "[FalconV2Propulsion] Failed to load config from '" << configPath
          << "' - propulsion forces will NOT be applied.\n";
    return;
  }

  // Optional overrides (mirrors AerodynamicsSystem.cc's pattern).
  this->physicsCfg.rho = _sdf->Get<double>("air_density", this->physicsCfg.rho).first;
  this->diagRateHz = _sdf->Get<double>("diagnostics_rate_hz", this->diagRateHz).first;
  const std::string windTopic =
      _sdf->Get<std::string>("wind_topic", "/model/falcon_v2/wind").first;
  const std::string leftThrottleTopic = _sdf->Get<std::string>(
      "left_throttle_topic", "/model/falcon_v2/propulsion/left/throttle_cmd").first;
  const std::string rightThrottleTopic = _sdf->Get<std::string>(
      "right_throttle_topic", "/model/falcon_v2/propulsion/right/throttle_cmd").first;
  const std::string diagTopic = _sdf->Get<std::string>(
      "diagnostics_topic", "/model/falcon_v2/propulsion/diagnostics").first;

  // ---- Resolve link/joint entities ----
  this->baseLinkEntity = model.LinkByName(_ecm, kBaseLinkName);
  this->leftPropJoint = model.JointByName(_ecm, kLeftPropJoint);
  this->rightPropJoint = model.JointByName(_ecm, kRightPropJoint);

  if (this->baseLinkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[FalconV2Propulsion] Could not find link '" << kBaseLinkName
          << "' - propulsion forces will NOT be applied.\n";
    this->validConfig = false;
    return;
  }
  if (this->leftPropJoint == gz::sim::kNullEntity)
  {
    gzerr << "[FalconV2Propulsion] Could not find joint '" << kLeftPropJoint
          << "' - visual spin will NOT be driven for the left propeller.\n";
  }
  if (this->rightPropJoint == gz::sim::kNullEntity)
  {
    gzerr << "[FalconV2Propulsion] Could not find joint '" << kRightPropJoint
          << "' - visual spin will NOT be driven for the right propeller.\n";
  }

  gz::sim::Link baseLink(this->baseLinkEntity);
  baseLink.EnableVelocityChecks(_ecm, true);

  // ---- Topic subscriptions ----
  if (!this->node.Subscribe(windTopic, &PropulsionSystem::OnWind, this))
  {
    gzerr << "[FalconV2Propulsion] Failed to subscribe to wind topic '"
          << windTopic << "'.\n";
  }
  if (!this->node.Subscribe(leftThrottleTopic, &PropulsionSystem::OnThrottleLeft, this))
  {
    gzerr << "[FalconV2Propulsion] Failed to subscribe to left throttle topic '"
          << leftThrottleTopic << "'.\n";
  }
  if (!this->node.Subscribe(rightThrottleTopic, &PropulsionSystem::OnThrottleRight, this))
  {
    gzerr << "[FalconV2Propulsion] Failed to subscribe to right throttle topic '"
          << rightThrottleTopic << "'.\n";
  }

  this->diagPub = this->node.Advertise<gz::msgs::Double_V>(diagTopic);

  gzmsg << "[FalconV2Propulsion] Configured. config=" << configPath
        << " apc_table_slices=" << this->apcTable.slices.size()
        << " rho=" << this->physicsCfg.rho
        << " I_rotor=" << this->physicsCfg.rotorInertiaKgM2
        << " rpm_cap=" << this->physicsCfg.rpmCap
        << " left_throttle_topic=" << leftThrottleTopic
        << " right_throttle_topic=" << rightThrottleTopic
        << " wind_topic=" << windTopic
        << " diagnostics topic=" << diagTopic
        << " (order: L_throttle,L_current_A,L_Qmotor,L_omega_ownframe,L_rpm,"
        << "L_J,L_Ct,L_Cp,L_thrust_N,L_Qprop,L_interpClamped,L_rpmCapActive,"
        << "L_negCurClamped,L_curLimited, then the same 14 fields for R)"
        << " @ " << this->diagRateHz << " Hz.\n";
}

//////////////////////////////////////////////////
bool PropulsionSystem::LoadConfig(const std::string &_path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(_path);
  }
  catch (const std::exception &e)
  {
    gzerr << "[FalconV2Propulsion] YAML load error: " << e.what() << "\n";
    return false;
  }

  std::string apcCsvPath;
  try
  {
    auto motor = root["motor"];
    this->motorCfg.kvRpmPerVolt = motor["kv_rpm_per_v"].as<double>();
    this->motorCfg.resistanceOhm = motor["resistance_ohm"].as<double>();
    this->motorCfg.noLoadCurrentA = motor["no_load_current_A"].as<double>();
    this->motorCfg.noLoadCurrentRefVoltage =
        motor["no_load_current_ref_voltage_V"].as<double>();
    this->motorCfg.currentLimitA = motor["current_limit_A"].as<double>();
    this->motorCfg.powerLimitW = motor["power_limit_W"].as<double>();

    auto battery = root["battery"];
    this->batteryCfg.vNominal = battery["v1_operating_voltage_V"].as<double>();

    auto prop = root["propeller"];
    this->propCfg.diameterM = prop["diameter_m"].as<double>();
    apcCsvPath = prop["apc_parsed_csv_path"].as<std::string>();
    this->physicsCfg.rpmCap = prop["rpm_cap_v1"].as<double>();

    auto physics = root["physics"];
    this->physicsCfg.rho = physics["rho_kg_m3"].as<double>();
    this->physicsCfg.nSafeFloorRevS = physics["n_safe_floor_rev_s"].as<double>();
    this->physicsCfg.rotorInertiaKgM2 = physics["i_rotor_kg_m2"].as<double>();

    auto geom = root["geometry"];
    auto leftHub = geom["left_hub_m"];
    auto rightHub = geom["right_hub_m"];
    auto axis = geom["thrust_axis_body"];
    this->leftHubBody.Set(leftHub[0].as<double>(), leftHub[1].as<double>(),
                           leftHub[2].as<double>());
    this->rightHubBody.Set(rightHub[0].as<double>(), rightHub[1].as<double>(),
                            rightHub[2].as<double>());
    this->thrustAxisBody.Set(axis[0].as<double>(), axis[1].as<double>(),
                              axis[2].as<double>());

    auto rotation = root["rotation"];
    this->rotationSignLeft = rotation["left_sign"].as<double>();
    this->rotationSignRight = rotation["right_sign"].as<double>();
  }
  catch (const std::exception &e)
  {
    gzerr << "[FalconV2Propulsion] YAML parse error (missing/invalid key): "
          << e.what() << "\n";
    return false;
  }

  if (apcCsvPath.empty())
  {
    gzerr << "[FalconV2Propulsion] propeller.apc_parsed_csv_path is empty.\n";
    return false;
  }
  // apc_parsed_csv_path in the YAML is a repository-relative path (matching
  // the YAML's own documented convention); resolve it against the
  // repository root inferred from config_path (.../docs/source_of_truth/
  // propulsion/propulsion_v1_config.yaml) if it is not already absolute.
  std::string resolvedCsvPath = apcCsvPath;
  if (!apcCsvPath.empty() && apcCsvPath[0] != '/')
  {
    const std::string marker = "docs/source_of_truth/propulsion/propulsion_v1_config.yaml";
    const std::string cfgPath = _path;
    const auto pos = cfgPath.find(marker);
    if (pos != std::string::npos)
    {
      resolvedCsvPath = cfgPath.substr(0, pos) + apcCsvPath;
    }
  }

  std::string err;
  if (!LoadApcTableCsv(resolvedCsvPath, this->apcTable, &err))
  {
    gzerr << "[FalconV2Propulsion] Failed to load APC table from '"
          << resolvedCsvPath << "': " << err << "\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void PropulsionSystem::OnThrottleLeft(const gz::msgs::Double &_msg)
{
  this->throttleLeftCmd = std::clamp(_msg.data(), 0.0, 1.0);
}

//////////////////////////////////////////////////
void PropulsionSystem::OnThrottleRight(const gz::msgs::Double &_msg)
{
  this->throttleRightCmd = std::clamp(_msg.data(), 0.0, 1.0);
}

//////////////////////////////////////////////////
void PropulsionSystem::OnWind(const gz::msgs::Vector3d &_msg)
{
  this->windWorld.Set(_msg.x(), _msg.y(), _msg.z());
}

//////////////////////////////////////////////////
void PropulsionSystem::StepMotor(
    gz::sim::EntityComponentManager &_ecm,
    double dt,
    const gz::math::Quaterniond &_worldRot,
    const gz::math::Vector3d &_hubOffsetBody,
    double _rotationSign,
    double _throttleCmd,
    double &_omegaOwnState,
    double &_thetaOwnState,
    gz::sim::Entity _propJoint,
    std::vector<double> &_diagOut)
{
  gz::sim::Link baseLink(this->baseLinkEntity);

  // ---- 5. Local axial airspeed at this motor's own hub ----
  // WorldLinearVelocity(ecm, offset) returns the world-frame velocity of the
  // point [offset] away from the link's own origin (offset expressed in the
  // link-fixed/body frame) - i.e. exactly V_point = V_origin + omega x
  // offset, algebraically identical to the CG-relative formulation
  // (V_cg + omega x r_from_cg) since both are valid reference points on the
  // same rigid body; using this gz-sim primitive directly (rather than
  // re-deriving the cross product by hand) avoids a second, potentially
  // inconsistent implementation of the same rigid-body kinematics already
  // used elsewhere in this project.
  const auto hubVelWorldOpt = baseLink.WorldLinearVelocity(_ecm, _hubOffsetBody);
  double vAxial = 0.0;
  if (hubVelWorldOpt.has_value())
  {
    // Relative airflow: Vrel = Vhub - Vwind (same convention as
    // AerodynamicsSystem.cc), projected onto this motor's thrust axis.
    const gz::math::Vector3d hubVelRel = *hubVelWorldOpt - this->windWorld;
    const gz::math::Vector3d thrustAxisWorld = _worldRot.RotateVector(this->thrustAxisBody);
    vAxial = hubVelRel.Dot(thrustAxisWorld);
  }
  // else: velocity checks not yet enabled for this tick (first tick(s) after
  // Configure()) - fall back to vAxial=0 this step rather than use stale
  // data; the omega ODE still advances correctly (thrust simply computed at
  // zero local airspeed for this one tick).

  // ---- 2. ESC/motor electrical (own-frame omega) ----
  const MotorElectricalResult elec = MotorElectrical(
      this->motorCfg, _throttleCmd, this->batteryCfg.vNominal, _omegaOwnState);

  // ---- 5/6. Propeller aerodynamic loading at the CURRENT (pre-step) omega ----
  const PropLoadResult load = PropAeroLoad(
      this->apcTable, _omegaOwnState, vAxial, this->propCfg.diameterM,
      this->physicsCfg.rho, this->physicsCfg.nSafeFloorRevS);

  // ---- 3/4. Rotor ODE step (explicit Euler, RPM cap saturation) ----
  const RotorStepResult step = IntegrateRotorStep(
      _omegaOwnState, elec.torque_Nm, load.qPropSigned_Nm,
      this->physicsCfg.rotorInertiaKgM2, dt, this->physicsCfg.rpmCap);
  _omegaOwnState = step.omega;

  // ---- 9. Force/moment application at the REAL hub, base_link ----
  // Thrust acts along the fixed +X thrust axis (rotation-sign-independent -
  // both motors intake/accelerate air along the same physical +X sense
  // regardless of which way they spin). Reaction torque
  // (Q_reaction = -Q_prop, PROPULSION.md sec 7) is mapped from this motor's
  // own-frame sign onto the shared body +X axis via the SAME rotation_sign
  // multiplier used for the joint-velocity drive below - see
  // propulsion_v1_config.yaml "rotation" block.
  const gz::math::Vector3d thrustForceBody = this->thrustAxisBody * load.thrust_N;
  const gz::math::Vector3d thrustForceWorld = _worldRot.RotateVector(thrustForceBody);

  const double reactionTorqueOwn = -load.qPropSigned_Nm;
  const double reactionTorqueBodyX = _rotationSign * reactionTorqueOwn;
  const gz::math::Vector3d reactionTorqueBody = this->thrustAxisBody * reactionTorqueBodyX;
  const gz::math::Vector3d reactionTorqueWorld = _worldRot.RotateVector(reactionTorqueBody);

  // Single combined call: the force's own r x F moment (from the hub
  // offset) plus the explicit pure reaction-torque couple - no separate,
  // hand-coded r x F shortcut anywhere in this file (PROPULSION.md sec 7.1).
  baseLink.AddWorldWrench(_ecm, thrustForceWorld, reactionTorqueWorld, _hubOffsetBody);

  // ---- Drive the visual prop joint from the SAME omega state (KINEMATIC
  // position write, NOT SetVelocity) ----
  // RESOLVED (2026-08-23, PROPULSION_V1_IMPLEMENTATION fix-loop): this
  // block previously used Joint::SetVelocity(). gz/sim/Joint.hh documents
  // SetVelocity() as "the physics engine is expected to compute the
  // necessary joint torque for the commanded velocity and apply it in the
  // next simulation step" - i.e. a genuine force-based servo, confirmed
  // directly from the header (not inferred). Because this joint's SDF
  // rotational inertia (I=2.7349e-4 kg*m^2, model.sdf left_prop/right_prop)
  // equals this plugin's own I_rotor, any such servo torque would equal
  // I_rotor*dOmega/dt by construction and would apply to base_link through
  // the joint IN ADDITION to the explicit AddWorldWrench() reaction torque
  // above - a duplicated-force risk during any transient (dOmega/dt != 0;
  // identically zero only at settled equilibrium, which is why the prior
  // steady-state PRIORITY_CHECK test could not detect it).
  //
  // Fix: integrate a NEW, purely-cosmetic angular-position state (theta,
  // own-frame/rotation-sign convention, same as the old velocity command)
  // from the SAME authoritative omega state already used for thrust/
  // reaction-torque above, then write it via Joint::ResetPosition(), which
  // gz/sim/Joint.hh documents as directly modifying the joint's internal
  // state (contrasted explicitly against SetVelocity's "does not modify the
  // internal state" / physics-engine-torque behavior) - a kinematic write,
  // not something the constraint solver applies force to achieve. Same
  // primitive (reset_position()/reset_velocity()) already used by
  // tests/gazebo/scripts/aero_lib.py's pin_child_joints() for exactly this
  // "drive a joint without letting it participate in real dynamics" need.
  // theta itself feeds NOTHING physical - it is wrapped into (-pi, pi] each
  // tick purely to avoid unbounded double growth over a long sim; only
  // omega (unchanged above) drives thrust/torque/RPM.
  if (_propJoint != gz::sim::kNullEntity)
  {
    const double jointOmegaBodyX = _rotationSign * _omegaOwnState;
    _thetaOwnState = std::remainder(
        _thetaOwnState + jointOmegaBodyX * dt, 2.0 * M_PI);
    gz::sim::Joint(_propJoint).ResetPosition(_ecm, {_thetaOwnState});
  }

  // ---- Diagnostics (see Configure()'s logged field-order comment) ----
  _diagOut.push_back(_throttleCmd);
  _diagOut.push_back(elec.current_A);
  _diagOut.push_back(elec.torque_Nm);
  _diagOut.push_back(_omegaOwnState);
  _diagOut.push_back(step.rpm);
  _diagOut.push_back(load.J);
  _diagOut.push_back(load.Ct);
  _diagOut.push_back(load.Cp);
  _diagOut.push_back(load.thrust_N);
  _diagOut.push_back(load.qPropSigned_Nm);
  _diagOut.push_back(load.interpClamped ? 1.0 : 0.0);
  _diagOut.push_back(step.rpmCapActive ? 1.0 : 0.0);
  _diagOut.push_back(elec.negativeCurrentClamped ? 1.0 : 0.0);
  _diagOut.push_back(elec.currentLimited ? 1.0 : 0.0);
}

//////////////////////////////////////////////////
void PropulsionSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || !this->validConfig)
    return;

  const double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0.0)
    return;

  gz::sim::Link baseLink(this->baseLinkEntity);
  const auto worldPoseOpt = baseLink.WorldPose(_ecm);
  if (!worldPoseOpt.has_value())
  {
    // Pose/velocity checks not yet enabled for this step - skip, apply no
    // force this step rather than use stale/garbage data (mirrors
    // AerodynamicsSystem.cc's PreUpdate() guard).
    return;
  }
  const gz::math::Quaterniond &worldRot = worldPoseOpt->Rot();

  std::vector<double> diag;
  diag.reserve(26);

  this->StepMotor(_ecm, dt, worldRot, this->leftHubBody, this->rotationSignLeft,
                   this->throttleLeftCmd, this->omegaLeft, this->thetaLeft,
                   this->leftPropJoint, diag);
  this->StepMotor(_ecm, dt, worldRot, this->rightHubBody, this->rotationSignRight,
                   this->throttleRightCmd, this->omegaRight, this->thetaRight,
                   this->rightPropJoint, diag);

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
    PropulsionSystem,
    gz::sim::System,
    PropulsionSystem::ISystemConfigure,
    PropulsionSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(PropulsionSystem, "falcon_v2_propulsion::PropulsionSystem")
GZ_ADD_PLUGIN_ALIAS(PropulsionSystem, "FalconV2Propulsion")

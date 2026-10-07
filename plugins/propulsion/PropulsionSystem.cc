// FALCON V2 — itki sistemi eklentisinin gerçeklemesi.
// Mimari özet için PropulsionSystem.hh'ye bakın. Buradaki hiçbir katsayı
// gömülü değildir: tümü Configure() anında config/propulsion_v1_config.yaml
// dosyasından okunur. Aşağıdaki eklem/bağlantı adları model.sdf ile eşleşen
// yapısal tanımlayıcılardır, fiziksel sabit değildir.
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

  // SDF üzerinden isteğe bağlı geçersiz kılmalar.
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

  // ---- Bağlantı/eklem varlıklarının çözümlenmesi ----
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

  // ---- Konu abonelikleri ----
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
  // YAML'deki apc_parsed_csv_path, YAML dosyasının bulunduğu dizine görelidir;
  // yüklemeden önce mutlak hâle getirilir. Zaten mutlak olan yol aynen kullanılır.
  std::string resolvedCsvPath = apcCsvPath;
  if (apcCsvPath[0] != '/')
  {
    const std::string cfgPath = _path;
    const auto slash = cfgPath.find_last_of('/');
    if (slash != std::string::npos)
    {
      resolvedCsvPath = cfgPath.substr(0, slash + 1) + apcCsvPath;
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

  // ---- Bu motorun kendi göbeğindeki eksenel hava hızı ----
  // WorldLinearVelocity(ecm, offset), bağlantının orijininden [offset] kadar
  // uzaktaki noktanın dünya çerçevesindeki hızını verir (offset gövde
  // çerçevesinde), yani V_nokta = V_orijin + omega x offset. Aynı rijit cisim
  // üzerindeki iki geçerli referans noktası olduğundan bu, CG'ye göre yazılan
  // (V_cg + omega x r) ifadesiyle cebirsel olarak özdeştir.
  const auto hubVelWorldOpt = baseLink.WorldLinearVelocity(_ecm, _hubOffsetBody);
  double vAxial = 0.0;
  if (hubVelWorldOpt.has_value())
  {
    // Bağıl hava akışı: Vrel = Vgöbek - Vrüzgâr (dünya çerçevesinde), itki
    // ekseni üzerine izdüşürülür; J'yi besleyen eksenel bileşen budur.
    const gz::math::Vector3d hubVelRel = *hubVelWorldOpt - this->windWorld;
    const gz::math::Vector3d thrustAxisWorld = _worldRot.RotateVector(this->thrustAxisBody);
    vAxial = hubVelRel.Dot(thrustAxisWorld);
  }
  // Aksi hâlde (Configure() sonrası ilk tiklerde hız kontrolleri henüz etkin
  // değilse) bayat veri yerine bu adımda vAxial=0 kullanılır; omega ODE'si
  // yine doğru ilerler.

  // ---- ESC/motor elektriği (motorun kendi çerçevesindeki omega) ----
  const MotorElectricalResult elec = MotorElectrical(
      this->motorCfg, _throttleCmd, this->batteryCfg.vNominal, _omegaOwnState);

  // ---- MEVCUT (adım öncesi) omega'da pervane aerodinamik yüklenmesi ----
  const PropLoadResult load = PropAeroLoad(
      this->apcTable, _omegaOwnState, vAxial, this->propCfg.diameterM,
      this->physicsCfg.rho, this->physicsCfg.nSafeFloorRevS);

  // ---- Rotor ODE adımı (açık Euler, RPM sınırında doyum) ----
  const RotorStepResult step = IntegrateRotorStep(
      _omegaOwnState, elec.torque_Nm, load.qPropSigned_Nm,
      this->physicsCfg.rotorInertiaKgM2, dt, this->physicsCfg.rpmCap);
  _omegaOwnState = step.omega;

  // ---- Kuvvet/momentin GERÇEK göbekte, base_link üzerine uygulanması ----
  // İtki, dönüş yönünden bağımsız olarak sabit +X itki ekseni boyuncadır: her
  // iki motor da hangi yöne dönerse dönsün havayı aynı fiziksel +X yönünde
  // hızlandırır. Reaksiyon torku (Q_reaction = -Q_prop) ise motorun kendi
  // çerçevesindeki işaretinden paylaşılan gövde +X eksenine, aşağıdaki görsel
  // eklemle AYNI rotation_sign çarpanıyla eşlenir; yani işareti motorun dönüş
  // yönüne bağlıdır (bkz. propulsion_v1_config.yaml "rotation" bloğu).
  const gz::math::Vector3d thrustForceBody = this->thrustAxisBody * load.thrust_N;
  const gz::math::Vector3d thrustForceWorld = _worldRot.RotateVector(thrustForceBody);

  const double reactionTorqueOwn = -load.qPropSigned_Nm;
  const double reactionTorqueBodyX = _rotationSign * reactionTorqueOwn;
  const gz::math::Vector3d reactionTorqueBody = this->thrustAxisBody * reactionTorqueBodyX;
  const gz::math::Vector3d reactionTorqueWorld = _worldRot.RotateVector(reactionTorqueBody);

  // Tek bir birleşik çağrı: kuvvetin göbek konumundan doğan r x F momenti ile
  // açık saf reaksiyon-tork çifti birlikte uygulanır. Uygulama noktası CG
  // değil, gerçek göbek konumudur (_hubOffsetBody).
  baseLink.AddWorldWrench(_ecm, thrustForceWorld, reactionTorqueWorld, _hubOffsetBody);

  // ---- Görsel pervane eklemi, AYNI omega durumundan sürülür (KİNEMATİK
  // konum yazımı; SetVelocity DEĞİL) ----
  // Joint::SetVelocity() kuvvet tabanlı bir servodur: fizik motoru istenen hız
  // için gereken eklem torkunu hesaplayıp uygular. Bu eklemin SDF atalet
  // değeri eklentinin I_rotor'una eşit olduğundan, böyle bir servo torku her
  // geçici rejimde (dOmega/dt != 0) base_link'e yukarıdaki açık
  // AddWorldWrench() reaksiyon torkuna EK olarak binerdi. Bu nedenle konum,
  // doğrudan iç eklem durumunu yazan Joint::ResetPosition() ile sürülür.
  // theta hiçbir fiziksel büyüklüğü beslemez; uzun koşularda sınırsız büyümeyi
  // önlemek için her tikte (-pi, pi] aralığına sarılır.
  if (_propJoint != gz::sim::kNullEntity)
  {
    const double jointOmegaBodyX = _rotationSign * _omegaOwnState;
    _thetaOwnState = std::remainder(
        _thetaOwnState + jointOmegaBodyX * dt, 2.0 * M_PI);
    gz::sim::Joint(_propJoint).ResetPosition(_ecm, {_thetaOwnState});
  }

  // ---- Teşhis (alan sırası Configure() içinde kayda yazılır) ----
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
    // Bu adımda poz/hız kontrolleri henüz etkin değil: bayat veri kullanmak
    // yerine bu adımda hiç kuvvet uygulamadan çıkılır.
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

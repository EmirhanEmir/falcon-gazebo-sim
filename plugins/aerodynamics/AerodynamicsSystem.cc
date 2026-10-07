// FALCON V2 — aerodinamik System eklentisinin Gazebo bağlantı katmanı.
// Mimari özet için AerodynamicsSystem.hh'ye bakınız. Burada kullanılan her
// sayısal sabit Configure() anında config/aero_v1_config.yaml'den yüklenir;
// bu dosyada gömülü katsayı yoktur. Eklem/bağ adları model.sdf ile eşleşen
// yapısal tanımlayıcılardır, fizik sabiti değildir.
#include "AerodynamicsSystem.hh"

#include <algorithm>
#include <cmath>

#include <gz/plugin/Register.hh>

#include <gz/common/Console.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <gz/msgs/double_v.pb.h>

#include <yaml-cpp/yaml.h>

using namespace falcon_v2_aero;

namespace
{
// model.sdf ile birebir eşleşen yapısal adlar (aerodinamik katsayı değil).
constexpr const char *kBaseLinkName = "base_link";
constexpr const char *kLeftAileronJoint = "left_aileron_joint";
constexpr const char *kRightAileronJoint = "right_aileron_joint";
constexpr const char *kLeftElevatorJoint = "left_elevator_joint";
constexpr const char *kRightElevatorJoint = "right_elevator_joint";
constexpr const char *kRudderJoint = "rudder_joint";
}  // namespace

//////////////////////////////////////////////////
void AerodynamicsSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "[FalconV2Aerodynamics] Configure() called on an invalid model "
          << "entity - plugin must be attached at the <model> level in "
          << "model.sdf.\n";
    return;
  }

  // ---- Katsayı veri kümesini yükle ----
  const auto configPathPair = _sdf->Get<std::string>("config_path", "");
  const std::string configPath = configPathPair.first;
  if (configPath.empty())
  {
    gzerr << "[FalconV2Aerodynamics] No <config_path> SDF parameter given - "
          << "cannot load config/aero_v1_config.yaml. Aerodynamic forces will NOT be applied.\n";
    return;
  }
  this->validConfig = this->LoadConfig(configPath);
  if (!this->validConfig)
  {
    gzerr << "[FalconV2Aerodynamics] Failed to load config from '"
          << configPath << "' - aerodynamic forces will NOT be applied.\n";
    return;
  }

  // İsteğe bağlı SDF geçersiz kılmaları. Her birinin YAML'den yüklenen değere
  // geri dönüşü vardır; eksik bir SDF parametresi bir katsayıyı asla sessizce
  // sıfırlamaz.
  this->config.rho = _sdf->Get<double>("air_density", this->config.rho).first;
  this->diagRateHz = _sdf->Get<double>("diagnostics_rate_hz", this->diagRateHz).first;
  const std::string windTopic =
      _sdf->Get<std::string>("wind_topic", "/model/falcon_v2/wind").first;

  this->config.Prepare();

  // ---- Bağ/eklem varlıklarını çöz ----
  this->baseLinkEntity = model.LinkByName(_ecm, kBaseLinkName);
  this->leftAileronJoint = model.JointByName(_ecm, kLeftAileronJoint);
  this->rightAileronJoint = model.JointByName(_ecm, kRightAileronJoint);
  this->leftElevatorJoint = model.JointByName(_ecm, kLeftElevatorJoint);
  this->rightElevatorJoint = model.JointByName(_ecm, kRightElevatorJoint);
  this->rudderJoint = model.JointByName(_ecm, kRudderJoint);

  if (this->baseLinkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[FalconV2Aerodynamics] Could not find link '" << kBaseLinkName
          << "' - aerodynamic forces will NOT be applied.\n";
    this->validConfig = false;
    return;
  }
  for (auto [name, e] : {
           std::pair{kLeftAileronJoint, this->leftAileronJoint},
           std::pair{kRightAileronJoint, this->rightAileronJoint},
           std::pair{kLeftElevatorJoint, this->leftElevatorJoint},
           std::pair{kRightElevatorJoint, this->rightElevatorJoint},
           std::pair{kRudderJoint, this->rudderJoint}})
  {
    if (e == gz::sim::kNullEntity)
    {
      gzerr << "[FalconV2Aerodynamics] Could not find joint '" << name
            << "' - its deflection will read as zero.\n";
    }
  }

  gz::sim::Link baseLink(this->baseLinkEntity);
  baseLink.EnableVelocityChecks(_ecm, true);
  for (gz::sim::Entity je : {this->leftAileronJoint, this->rightAileronJoint,
                             this->leftElevatorJoint, this->rightElevatorJoint,
                             this->rudderJoint})
  {
    if (je != gz::sim::kNullEntity)
    {
      gz::sim::Joint(je).EnablePositionCheck(_ecm, true);
    }
  }

  // ---- Rüzgâr konusu aboneliği (yayıncı yoksa Vwind = 0) ----
  if (!this->node.Subscribe(windTopic, &AerodynamicsSystem::OnWind, this))
  {
    gzerr << "[FalconV2Aerodynamics] Failed to subscribe to wind topic '"
          << windTopic << "'.\n";
  }

  // ---- Tanı yayıncısı (PreUpdate içinde hız sınırlanır) ----
  const std::string diagTopic = "/model/falcon_v2/aerodynamics/diagnostics";
  this->diagPub = this->node.Advertise<gz::msgs::Double_V>(diagTopic);

  gzmsg << "[FalconV2Aerodynamics] Configured. config=" << configPath
        << " rho=" << this->config.rho << " S=" << this->config.S
        << " b=" << this->config.b << " c_ref=" << this->config.c_ref
        << " diagnostics topic=" << diagTopic
        << " (order: V,alpha_rad,beta_rad,qbar,CL,CD,CY,Cl,Cm,Cn)"
        << " @ " << this->diagRateHz << " Hz. wind topic=" << windTopic
        << " (gz.msgs.Vector3d, world frame, m/s; zero until a publisher "
        << "exists).\n";
}

//////////////////////////////////////////////////
bool AerodynamicsSystem::LoadConfig(const std::string &_path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(_path);
  }
  catch (const std::exception &e)
  {
    gzerr << "[FalconV2Aerodynamics] YAML load error: " << e.what() << "\n";
    return false;
  }

  try
  {
    auto refGeom = root["reference_geometry"];
    this->config.S = refGeom["wing_area_S_m2"].as<double>();
    this->config.b = refGeom["wingspan_b_m"].as<double>();
    this->config.c_ref = refGeom["reference_chord_c_ref_m"].as<double>();

    auto env = root["environment"];
    this->config.rho = env["air_density_rho_kg_m3"].as<double>();
    this->config.vSafeFloor = env["v_safe_floor_m_s"].as<double>();

    auto lon = root["longitudinal"];
    this->config.CLa = lon["CLa_per_rad"].as<double>();
    this->config.Cma = lon["Cma_per_rad"].as<double>();
    this->config.CLq = lon["CLq"].as<double>();
    this->config.Cmq = lon["Cmq"].as<double>();
    // Cmde/CLde: SUPERSEDED_BY_LOOKUP. Yalnızca çapraz kontrol için yüklenir,
    // ComputeAero() tarafından okunmaz; yerini aşağıda yüklenen
    // control_surface_lookup.elevator tabloları alır.
    this->config.Cmde = lon["Cmde_per_rad"].as<double>();
    this->config.CLde = lon["CLde_per_rad"].as<double>();
    this->config.CL0 = lon["CL0"].as<double>();
    this->config.Cm0 = lon["Cm0"].as<double>();

    // CYda/CYdr/Clda/Cnda/Cndr: SUPERSEDED_BY_LOOKUP. Yalnızca çapraz kontrol
    // için yüklenir, ComputeAero() tarafından okunmaz. Cldr tek istisnadır ve
    // işlevsel olarak kullanılır: dümen yuvarlanma araması geniş sapma
    // tablosundan değil doğrudan bu skalerden türetilir (AeroConfig::Prepare).
    auto lat = root["lateral_directional"];
    this->config.CYb = lat["CYb"].as<double>();
    this->config.CYp = lat["CYp"].as<double>();
    this->config.CYr = lat["CYr"].as<double>();
    this->config.CYda = lat["CYda_per_rad"].as<double>();
    this->config.CYdr = lat["CYdr_per_rad"].as<double>();
    this->config.Clb = lat["Clb"].as<double>();
    this->config.Clp = lat["Clp"].as<double>();
    this->config.Clr = lat["Clr"].as<double>();
    this->config.Clda = lat["Clda_per_rad"].as<double>();
    this->config.Cldr = lat["Cldr_per_rad"].as<double>();
    this->config.Cnb = lat["Cnb"].as<double>();
    this->config.Cnp = lat["Cnp"].as<double>();
    this->config.Cnr = lat["Cnr"].as<double>();
    this->config.Cnda = lat["Cnda_per_rad"].as<double>();
    this->config.Cndr = lat["Cndr_per_rad"].as<double>();

    auto drag = root["drag_polar"];
    this->config.CD0 = drag["CD0"].as<double>();
    this->config.dragK = drag["k"].as<double>();

    auto lim = root["high_alpha_limiter"];
    this->config.CLmax = lim["CLmax_manufacturer"].as<double>();
    this->config.alphaTransition =
        lim["alpha_transition_deg"].as<double>() * M_PI / 180.0;

    auto ctrl = root["control_mapping"];
    this->config.elevatorSign = ctrl["elevator_sign"].as<double>();
    this->config.aileronSign = ctrl["aileron_sign"].as<double>();
    this->config.rudderSign = ctrl["rudder_sign"].as<double>();
    // Genel kumanda sapma kırpması artık YAML'de yoktur ve kasten okunmaz:
    // ekstrapolasyon sınırı artık geniş sapma arama tablolarının kendi
    // +/-45 derecelik tanım aralığıdır (AeroModel.hh içindeki InterpLinear).

    // ---- Geniş sapma kumanda yüzeyi arama tabloları ----
    auto loadArr = [](const YAML::Node &node, falcon_v2_aero::CtrlLookupArray &arr)
    {
      for (std::size_t i = 0; i < arr.size(); ++i)
        arr[i] = node[i].as<double>();
    };

    auto csl = root["control_surface_lookup"];
    loadArr(csl["breakpoints_deg"], this->config.ctrlBreakpointsRad);
    for (auto &x : this->config.ctrlBreakpointsRad)
      x *= M_PI / 180.0;  // derece -> radyan, yükleme anında bir kez

    auto elev = csl["elevator"];
    loadArr(elev["dCL"], this->config.ctrlElevDCL);
    loadArr(elev["dCD"], this->config.ctrlElevDCD);
    loadArr(elev["dCm"], this->config.ctrlElevDCm);

    auto aile = csl["aileron"];
    loadArr(aile["Cl"], this->config.ctrlAileCl);
    loadArr(aile["Cn"], this->config.ctrlAileCn);
    loadArr(aile["CY"], this->config.ctrlAileCY);
    loadArr(aile["CD_full"], this->config.ctrlAileCDFull);
    loadArr(aile["CL_full"], this->config.ctrlAileCLFull);
    loadArr(aile["Cm_full"], this->config.ctrlAileCmFull);

    // Dümen Cl dizisi, işareti tartışmalı olduğu için YAML'den KASTEN
    // yüklenmez; AeroConfig::Prepare() onu Cldr skalerinden türetir.
    auto rudd = csl["rudder"];
    loadArr(rudd["CY"], this->config.ctrlRuddCY);
    loadArr(rudd["Cn"], this->config.ctrlRuddCn);
    loadArr(rudd["CD_full"], this->config.ctrlRuddCDFull);
  }
  catch (const std::exception &e)
  {
    gzerr << "[FalconV2Aerodynamics] YAML parse error (missing/invalid key): "
          << e.what() << "\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void AerodynamicsSystem::OnWind(const gz::msgs::Vector3d &_msg)
{
  this->windWorld.Set(_msg.x(), _msg.y(), _msg.z());
}

//////////////////////////////////////////////////
void AerodynamicsSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || !this->validConfig)
    return;

  gz::sim::Link baseLink(this->baseLinkEntity);
  const auto worldPoseOpt = baseLink.WorldPose(_ecm);
  const auto worldLinVelOpt = baseLink.WorldLinearVelocity(_ecm);
  const auto worldAngVelOpt = baseLink.WorldAngularVelocity(_ecm);
  if (!worldPoseOpt.has_value() || !worldLinVelOpt.has_value() ||
      !worldAngVelOpt.has_value())
  {
    // Configure() sonrasındaki ilk tiklerde hız/konum kontrolleri henüz
    // etkin olmayabilir; eski veya anlamsız veri kullanmak yerine bu adımda
    // hiç kuvvet uygulamadan çıkılır. Veri dolduğunda kendiliğinden sürer.
    return;
  }

  const gz::math::Quaterniond &worldRot = worldPoseOpt->Rot();

  // Gövde çerçevesinde bağıl rüzgâr: Vrel = Vbody - Vwind. Dünya çerçevesinde
  // çıkarılır, sonra ters döndürmeyle gövde eksenlerine taşınır.
  const gz::math::Vector3d velWorld = *worldLinVelOpt - this->windWorld;
  const gz::math::Vector3d velBody = worldRot.RotateVectorReverse(velWorld);
  const gz::math::Vector3d angVelBody =
      worldRot.RotateVectorReverse(*worldAngVelOpt);

  // ---- Kumanda eklem açıları -> sapma eşlemesi ----
  // İşaret gerekçesinin tamamı aero_v1_config.yaml içindeki control_mapping
  // bloğundadır.
  auto jointPos = [&_ecm](gz::sim::Entity e) -> double
  {
    if (e == gz::sim::kNullEntity)
      return 0.0;
    auto p = gz::sim::Joint(e).Position(_ecm);
    if (!p.has_value() || p->empty())
      return 0.0;
    return (*p)[0];
  };

  const double thetaLA = jointPos(this->leftAileronJoint);
  const double thetaRA = jointPos(this->rightAileronJoint);
  const double thetaLE = jointPos(this->leftElevatorJoint);
  const double thetaRE = jointPos(this->rightElevatorJoint);
  const double thetaR = jointPos(this->rudderJoint);

  AeroState state;
  state.u = velBody.X();
  state.v = velBody.Y();
  state.w = velBody.Z();
  state.p = angVelBody.X();
  state.q = angVelBody.Y();
  state.r = angVelBody.Z();
  state.deltaA =
      0.5 * this->config.aileronSign * (thetaRA - thetaLA);
  state.deltaE =
      0.5 * this->config.elevatorSign * (thetaLE + thetaRE);
  state.deltaR = this->config.rudderSign * thetaR;

  const AeroOutput out = ComputeAero(this->config, state);

  const gz::math::Vector3d forceWorld = worldRot.RotateVector(out.forceBody);
  const gz::math::Vector3d momentWorld = worldRot.RotateVector(out.momentBody);

  // Kuvvet, bağın kütle merkezinde uygulanır: iki argümanlı AddWorldForce()
  // gz-sim8 API'sinde tanımı gereği kütle merkezine etki eder, dolayısıyla
  // fazladan bir moment katkısı doğmaz. Momentler ise Cl/Cm/Cn'den türetilmiş
  // tam Mx/My/Mz olarak SAF tork biçiminde (çağrıda kuvvet sıfır) uygulanır;
  // böylece ikinci bir r x F katkısı oluşmaz ve çift sayım olmaz.
  baseLink.AddWorldForce(_ecm, forceWorld);
  baseLink.AddWorldWrench(_ecm, gz::math::Vector3d::Zero, momentWorld);

  // ---- Hız sınırlanmış tanı yayını ----
  const double dtSec =
      std::chrono::duration<double>(_info.dt).count();
  this->diagAccumulator += dtSec;
  const double period = (this->diagRateHz > 0.0) ? (1.0 / this->diagRateHz) : -1.0;
  if (period > 0.0 && this->diagAccumulator >= period)
  {
    this->diagAccumulator = 0.0;
    gz::msgs::Double_V msg;
    msg.add_data(out.V);
    msg.add_data(out.alpha);
    msg.add_data(out.beta);
    msg.add_data(out.qbar);
    msg.add_data(out.CL);
    msg.add_data(out.CD);
    msg.add_data(out.CY);
    msg.add_data(out.Cl);
    msg.add_data(out.Cm);
    msg.add_data(out.Cn);
    this->diagPub.Publish(msg);
  }
}

GZ_ADD_PLUGIN(
    AerodynamicsSystem,
    gz::sim::System,
    AerodynamicsSystem::ISystemConfigure,
    AerodynamicsSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(AerodynamicsSystem, "falcon_v2_aero::AerodynamicsSystem")
GZ_ADD_PLUGIN_ALIAS(AerodynamicsSystem, "FalconV2Aerodynamics")

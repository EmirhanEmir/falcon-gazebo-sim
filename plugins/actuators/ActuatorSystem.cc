// FALCON V2 - Aktüatör/servo System plugin'i (uygulama).
// Mimari özeti için ActuatorSystem.hh. Burada kullanılan her sayısal değer
// Configure() anında config/actuator_v1_config.yaml'den okunur; bu dosyada
// hiçbir katsayı sabitlenmemiştir. Aşağıdaki joint adları model/model.sdf ile
// eşleşen yapısal tanımlayıcılardır, fizik sabiti değildir.
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
// Sıra, ActuatorSystem.hh'deki SurfaceIndex enum'ı ve YAML "surfaces"
// anahtarlarıyla birebir aynı olmak ZORUNDADIR.
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

  const std::string configPath = _sdf->Get<std::string>("config_path", "").first;
  if (configPath.empty())
  {
    gzerr << "[FalconV2Actuators] No <config_path> SDF parameter given - "
          << "cannot load config/actuator_v1_config.yaml. Control surfaces will NOT move.\n";
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

    // Fizik motoru seviyesinde sert kısıtlar - PID'in yazılım kırpmasına ek
    // olarak ve ondan bağımsız. Gerekçesi: kontrol yüzeyi link'leri
    // model/model.sdf'de kasıtlı olarak neredeyse kütlesiz sayısal yer
    // tutucular taşır, dolayısıyla gerçek servo mertebesindeki bir tork bu
    // yer tutucu atalete etki ederken tek bir fizik adımında çok büyük bir
    // hıza entegre olabilir. Joint'in kendi hız/effort limitlerini kurmak,
    // bu atalet uyuşmazlığından bağımsız olarak GERÇEK joint'e sert bir
    // kısıt getirir.
    joint.SetVelocityLimits(
        _ecm, {gz::math::Vector2d(-this->config[i].maxRateRadS,
                                   this->config[i].maxRateRadS)});
    joint.SetEffortLimits(
        _ecm, {gz::math::Vector2d(-this->config[i].maxEffortNm,
                                   this->config[i].maxEffortNm)});

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
    // control_law kp/kd/ki belgelenmiş YEDEK değerlerdir; canlı kazançlar
    // aşağıdaki yüzey döngüsünde okunan yüzey başına değerlerdir. Blok yine de
    // zorunlu tutulur, böylece eksikse sessizce sıfır kazançla çalışmak yerine
    // yüklenme yüksek sesle başarısız olur.
    auto lawNode = root["control_law"];
    const double kp = lawNode["kp_nm_per_rad"].as<double>();
    const double kd = lawNode["kd_nm_per_rad_s"].as<double>();
    // ki ve sp_weight_b varsayılanlı okunur: bu anahtarları içermeyen eski bir
    // config, yüklenememek yerine belgelenmiş eski davranışa düşer (ki=0 saf
    // P(D), sp_weight_b=1.0 ağırlıksız P terimi).
    const double ki =
        lawNode["ki_nm_per_rad_s"] ? lawNode["ki_nm_per_rad_s"].as<double>()
                                    : 0.0;
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
      // Kazançlar yüzey başınadır: her yüzeyin kendi menteşe ekseni etrafındaki
      // ataletinden türetilir ve bu atalet değerleri gerçekten farklıdır, tek
      // paylaşılan bir sayı beş yüzey için doğru olamaz. Eksikse paylaşılan
      // control_law değerine düşer.
      cfg.kp = s["kp_nm_per_rad"] ? s["kp_nm_per_rad"].as<double>() : kp;
      cfg.kd = s["kd_nm_per_rad_s"] ? s["kd_nm_per_rad_s"].as<double>() : kd;
      cfg.ki = s["ki_nm_per_rad_s"] ? s["ki_nm_per_rad_s"].as<double>() : ki;
      // sp_weight_b kasten yüzey başına DEĞİLDİR: atalete bağlı bir kazanç
      // değil, boyutsuz bir yapısal seçimdir.
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
  if (idx >= kNumSurfaces) return;
  const double v = _msg.data();
  if (!std::isfinite(v))
  {
    // NaN/Inf bir komut setpoint zincirine asla girmez; son geçerli komut
    // tutulur (geçersiz mesaj, mesaj yokluğuyla aynı şekilde ele alınır).
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
      // Joint yok (Configure() bunu zaten bildirdi) - diagnostik vektörünün
      // uzunluğu/sırası sabit kalsın diye sıfır basılır.
      for (int f = 0; f < 7; ++f) diag.push_back(0.0);
      continue;
    }

    gz::sim::Joint joint(this->jointEntity[i]);
    const auto posOpt = joint.Position(_ecm);
    const auto velOpt = joint.Velocity(_ecm);
    if (!posOpt.has_value() || posOpt->empty() ||
        !velOpt.has_value() || velOpt->empty())
    {
      // Pozisyon/hız okuması bu adımda henüz hazır değil (Configure()
      // sonrasındaki ilk tick'ler): bayat veriyle bilinçsiz tork uygulamak
      // yerine bu yüzey atlanır. Setpoint durumu değişmez.
      for (int f = 0; f < 7; ++f) diag.push_back(0.0);
      continue;
    }
    const double actualAngle = (*posOpt)[0];
    const double actualRate = (*velOpt)[0];

    const SetpointStepResult step = StepSetpoint(
        this->commandRad[i], this->setpointRad[i], this->config[i], dt);
    this->setpointRad[i] = step.setpointRad;

    // Effort sınırlı PID torku SetForce() ile uygulanır: fizik motorunun
    // entegre ettiği gerçek joint-space torku, ResetPosition()/ışınlama değil.
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

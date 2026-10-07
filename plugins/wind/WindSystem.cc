// FALCON V2 — rüzgâr/hamle üreticisi System eklentisinin Gazebo bağlantı katmanı.
// Mimari özet için WindSystem.hh'ye bakınız. Tüm formüller WindModel.hh'dedir;
// bu dosya yalnızca Configure()/PreUpdate() tesisatıdır (konu ilanı/aboneliği,
// SDF parametre okuma, her tikte yayın). Burada hiçbir aerodinamik, eyleyici
// veya itki parametresi okunmaz.
#include "WindSystem.hh"

#include <chrono>
#include <cmath>

#include <gz/plugin/Register.hh>

#include <gz/common/Console.hh>
#include <gz/sim/Model.hh>

using namespace falcon_v2_wind;

//////////////////////////////////////////////////
void WindSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "[FalconV2Wind] Configure() called on an invalid model entity - "
          << "plugin must be attached at the <model> level in model.sdf.\n";
    return;
  }

  // ---- Konu adları: hepsi SDF'ten geçersiz kılınabilir; varsayılanlar
  // tüketici eklentilerin zaten abone olduğu rüzgâr konusuyla eşleşir. ----
  const std::string windTopic =
      _sdf->Get<std::string>("wind_topic", "/model/falcon_v2/wind").first;
  const std::string steadyCmdTopic = _sdf->Get<std::string>(
      "steady_cmd_topic", "/model/falcon_v2/wind/steady_cmd").first;
  const std::string gustCmdTopic = _sdf->Get<std::string>(
      "gust_cmd_topic", "/model/falcon_v2/wind/gust_cmd").first;

  // ---- Sabit rüzgâr: SDF'ten gelen başlangıç değeri (dünya çerçevesi, m/s),
  // yoksa sıfır vektör; sonrasında OnSteadyCmd() ile canlı değiştirilebilir.
  // gz::math::Vector3d doğal olarak desteklenen bir sdf::Param tipidir, bu
  // yüzden özel bir ayrıştırmaya gerek yoktur. ----
  this->steadyWindWorld =
      _sdf->Get<gz::math::Vector3d>(
          "steady_wind_mps", gz::math::Vector3d::Zero).first;

  this->windPub = this->node.Advertise<gz::msgs::Vector3d>(windTopic);
  if (!this->windPub)
  {
    gzerr << "[FalconV2Wind] Failed to advertise wind topic '" << windTopic
          << "'.\n";
  }

  if (!this->node.Subscribe(steadyCmdTopic, &WindSystem::OnSteadyCmd, this))
  {
    gzerr << "[FalconV2Wind] Failed to subscribe to steady-wind command "
          << "topic '" << steadyCmdTopic << "'.\n";
  }
  if (!this->node.Subscribe(gustCmdTopic, &WindSystem::OnGustCmd, this))
  {
    gzerr << "[FalconV2Wind] Failed to subscribe to gust command topic '"
          << gustCmdTopic << "'.\n";
  }

  gzmsg << "[FalconV2Wind] Configured. wind topic=" << windTopic
        << " (gz.msgs.Vector3d, world frame, m/s, velocity-of-the-air-mass "
        << "convention, published every PreUpdate() tick - never a direct "
        << "force/wrench). initial steady_wind_mps=" << this->steadyWindWorld
        << " steady_cmd_topic=" << steadyCmdTopic << " (gz.msgs.Vector3d)"
        << " gust_cmd_topic=" << gustCmdTopic
        << " (gz.msgs.Double_V, fixed field order: "
        << "[dir_x,dir_y,dir_z,amplitude_mps,start_delay_s,duration_s])."
        << " V1 limitation: only one gust may be scheduled/in-progress at a "
        << "time - a new gust command replaces any existing one.\n";
}

//////////////////////////////////////////////////
void WindSystem::OnSteadyCmd(const gz::msgs::Vector3d &_msg)
{
  const double x = _msg.x();
  const double y = _msg.y();
  const double z = _msg.z();
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
  {
    // Son geçerli komutu koruma emniyeti: NaN/Inf bir komutun yayınlanan
    // rüzgâra ulaşmasına asla izin verilmez.
    gzerr << "[FalconV2Wind] Ignored non-finite steady-wind command "
          << "(previous value held).\n";
    return;
  }
  this->steadyWindWorld.Set(x, y, z);
}

//////////////////////////////////////////////////
void WindSystem::OnGustCmd(const gz::msgs::Double_V &_msg)
{
  if (_msg.data_size() != 6)
  {
    gzerr << "[FalconV2Wind] Ignored gust command with " << _msg.data_size()
          << " field(s) - expected exactly 6 "
          << "(dir_x,dir_y,dir_z,amplitude_mps,start_delay_s,duration_s). "
          << "Any previously scheduled/in-progress gust is left unchanged.\n";
    return;
  }

  const gz::math::Vector3d rawDir(_msg.data(0), _msg.data(1), _msg.data(2));
  const double amplitude = _msg.data(3);
  const double startDelay = _msg.data(4);
  const double duration = _msg.data(5);

  if (!std::isfinite(rawDir.X()) || !std::isfinite(rawDir.Y()) ||
      !std::isfinite(rawDir.Z()) || !std::isfinite(amplitude) ||
      !std::isfinite(startDelay) || !std::isfinite(duration))
  {
    gzerr << "[FalconV2Wind] Ignored gust command containing a non-finite "
          << "field. Any previously scheduled/in-progress gust is left "
          << "unchanged.\n";
    return;
  }
  if (duration <= 0.0)
  {
    gzerr << "[FalconV2Wind] Ignored gust command with non-positive "
          << "duration_s=" << duration << " (must be > 0). Any previously "
          << "scheduled/in-progress gust is left unchanged.\n";
    return;
  }

  bool dirValid = false;
  const gz::math::Vector3d dirUnit = NormalizeDirection(rawDir, dirValid);
  if (!dirValid)
  {
    gzerr << "[FalconV2Wind] Ignored gust command with a degenerate "
          << "(near-zero-norm) direction vector (" << rawDir << "). Any "
          << "previously scheduled/in-progress gust is left unchanged.\n";
    return;
  }

  // V1 SINIRI: geçerli yeni bir komut gustState'i tümüyle değiştirir; kuyruk
  // yok, sürmekte olan hamleyle üst üste bindirme yok.
  //
  // start_delay_s, mutlak benzetim zamanına değil BU KOMUTUN ALINMASINA
  // görelidir. Referans olarak this->lastSimTimeSec, yani en son tamamlanan
  // PreUpdate() tikinin benzetim zamanı kullanılır (tik hassasiyeti gerekçesi
  // için WindSystem.hh'deki alan yorumuna bakınız). Böylece çağıran taraf
  // mevcut benzetim zamanını ayrıca sorgulamak zorunda kalmaz.
  GustState newGust;
  newGust.directionUnit = dirUnit;
  newGust.amplitudeMps = amplitude;
  newGust.startTimeSec = this->lastSimTimeSec + startDelay;
  newGust.durationSec = duration;
  newGust.scheduled = true;
  this->gustState = newGust;

  gzmsg << "[FalconV2Wind] Gust scheduled (replacing any previous gust): "
        << "direction_unit=" << dirUnit << " amplitude_mps=" << amplitude
        << " start_sim_time_s=" << newGust.startTimeSec
        << " (receipt_sim_time_s=" << this->lastSimTimeSec
        << " + start_delay_s=" << startDelay << ") duration_s=" << duration
        << ".\n";
}

//////////////////////////////////////////////////
void WindSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager & /*_ecm*/)
{
  if (_info.paused)
    return;

  this->lastSimTimeSec = std::chrono::duration<double>(_info.simTime).count();

  // ---- Saf matematik değerlendirmesi (WindModel.hh) ----
  const gz::math::Vector3d gustVec =
      EvaluateGust(this->gustState, this->lastSimTimeSec);
  const gz::math::Vector3d totalWind =
      ComposeWind(this->steadyWindWorld, gustVec);

  // ---- Her tikte yayın (hız sınırlaması yok): tüketici eklentiler yalnızca
  // SON gelen rüzgâr mesajını saklar, bu yüzden zamanla düzgün değişen bir
  // hamlenin düzgün görünmesi için her fizik adımında yeni yayın gerekir.
  // Bu yalnızca bir hız vektörü yayınıdır; bu dosyada hiçbir
  // AddWorldForce()/AddWorldWrench() çağrısı yoktur. ----
  gz::msgs::Vector3d msg;
  msg.set_x(totalWind.X());
  msg.set_y(totalWind.Y());
  msg.set_z(totalWind.Z());
  this->windPub.Publish(msg);
}

GZ_ADD_PLUGIN(
    WindSystem,
    gz::sim::System,
    WindSystem::ISystemConfigure,
    WindSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(WindSystem, "falcon_v2_wind::WindSystem")
GZ_ADD_PLUGIN_ALIAS(WindSystem, "FalconV2Wind")

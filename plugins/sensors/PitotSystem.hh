// FALCON V2 - Simüle pitot (airspeed) sensörü Gazebo Sim Harmonic plugin'i.
// base_link'in dünya çerçevesindeki hızını ECM'den ve rüzgârı
// /model/falcon_v2/wind topic'inden okur, PitotModel.hh ile airspeed ve
// diferansiyel basıncı hesaplar, sonucu yayınlar. Yerleşik gz-sim air_speed
// sensörü yerine neden özel bir plugin olduğu: PitotModel.hh başlığı.
// Sınır: salt okunur bir ölçüm katmanıdır; aerodinamik, itki veya aktüatöre
// yeni bir komut yolu açmaz.
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

  /// \brief Rüzgâr güncellemesi; projedeki diğer komut geri çağrılarıyla aynı
  /// failsafe politikası: sonlu olmayan değer reddedilir, önceki değer tutulur.
  private: void OnWind(const gz::msgs::Vector3d &_msg);

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity baseLinkEntity{gz::sim::kNullEntity};

  /// \brief Sensör montaj noktası: base_link orijinine göre ofset, gövdeye
  /// sabit (FLU) çerçevede. Gazebo/CAD ağırlık merkeziyle aynı konumda,
  /// (0.168309, 0, 0.100000) m; bkz. model.sdf.
  private: gz::math::Vector3d mountOffsetBody{gz::math::Vector3d::Zero};

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher airspeedPub;
  private: gz::transport::Node::Publisher pressurePub;

  private: gz::math::Vector3d windWorld{gz::math::Vector3d::Zero};

  /// \brief Hava yoğunluğu (kg/m^3); burada türetilmez,
  /// config/aero_v1_config.yaml içindeki environment.air_density_rho_kg_m3
  /// değerinden <air_density_rho_kg_m3> SDF parametresiyle gelir. Varsayılan,
  /// o dosyanın CONFIRMED 1.225 değeriyle aynıdır.
  private: double rhoKgM3{1.225};

  /// \brief Yayın hızı (Hz). V1_PROVISIONAL.
  private: double updateRateHz{10.0};
  private: double timeSinceLastPublishSec{0.0};

  /// \brief V1_PROVISIONAL_SENSOR_NOISE: yayınlanan airspeed üzerine opsiyonel,
  /// sıfır ortalamalı Gauss gürültüsü (m/s standart sapma). Varsayılan 0.0 =
  /// KAPALI (deterministik mod). Gerçek bir pitot sensörünün karakterize
  /// edilmiş gürültü değeri DEĞİLDİR; açıkça işaretlenmiş bir yer tutucudur.
  private: double noiseStddevMps{0.0};
  private: std::mt19937 noiseRng{12345u};  // sabit seed - açıkken de deterministik
  private: std::normal_distribution<double> noiseDist{0.0, 1.0};
};

}  // namespace falcon_v2_sensors

#endif  // FALCON_V2_SENSORS_PITOT_SYSTEM_HH_

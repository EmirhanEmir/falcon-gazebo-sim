// FALCON V2 - Simüle magnetometre sensörü Gazebo Sim Harmonic plugin'i.
// base_link'in dünya yönelimini ECM'den okur, SDF ile verilen sabit dünya
// (ENU) manyetik alan vektörünü gövde FLU çerçevesine döndürür ve
// gz.msgs.Magnetometer olarak yayınlar.
// Sınır: salt okunur bir ölçüm katmanıdır; aerodinamik, itki veya aktüatöre
// yeni bir komut yolu açmaz.
//
// NEDEN YERLEŞİK gz-sim-magnetometer-system YERİNE ÖZEL PLUGIN: bu projenin
// hedeflediği Gazebo Sim Harmonic kurulumunda yerleşik Magnetometer
// sisteminin yayınladığı field_tesla çıktısı, world'ün beyan edilen
// <magnetic_field> değerine hiç tepki vermeyen, büyüklüğü ~0.32 olan sabit bir
// vektördür. World'ün alan değerinin MagnetometerSensor::SetWorldMagneticField()
// çağrısına doğru ulaştığı teyit edildi, yani SDF->ECM ayrıştırma yolu kusurlu
// değil; sensör yapılandırdığı hızda yayın yapmasına rağmen Update() yolu hiç
// çalışmıyor. Bu, kurulu kütüphanelerdeki ikili seviyede bir kusurdur; bu
// projenin SDF veya config dosyalarında değil. Plugin, pitot sensörü için
// kurulan mimari emsali izler (PitotSystem.cc / PitotModel.hh).
//
// Alan hesabı yalnızca yönelime bağlıdır. Magnetometre yerel olarak düzgün bir
// alanı ölçer, hıza bağlı bir büyüklüğü değil; bu yüzden pitot sensörünün
// montaj ofseti kaldıraç kolunun aksine buradaki montaj noktası hesaba girmez.
#ifndef FALCON_V2_SENSORS_MAGNETOMETER_SYSTEM_HH_
#define FALCON_V2_SENSORS_MAGNETOMETER_SYSTEM_HH_

#include <string>

#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace falcon_v2_sensors
{

class MagnetometerSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: MagnetometerSystem() = default;
  public: ~MagnetometerSystem() override = default;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity baseLinkEntity{gz::sim::kNullEntity};

  /// \brief Sabit yer manyetik alan vektörü, DÜNYA çerçevesinde (bu projenin
  /// dünya çerçevesi ENU'dur - bkz. Frames.hh), Tesla.
  /// SIMULATION_ASSUMPTION: varsayılan (5.5645e-6, 22.8758e-6, -42.3884e-6) T,
  /// world'ün zaten belgelenmiş <magnetic_field> değeriyle (sdformat'ın kendi
  /// belgelenmiş world varsayılanı) aynıdır; böylece bu düzeltmeyle sayısal
  /// referans sessizce değişmez, yalnızca değeri tüketen mekanizma değişir
  /// (yerleşik sistem -> değeri gerçekten okuyup kullanan bu plugin). Gerçek
  /// Falcon V2 uçuş konumuna özgü manyetik alan DATA_REQUIRED'dır.
  private: gz::math::Vector3d worldMagneticFieldTesla{
      5.5645e-6, 22.8758e-6, -42.3884e-6};

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher magPub;

  /// \brief Yayın hızı (Hz). V1_PROVISIONAL; yerini aldığı yerleşik sensör için
  /// belgelenmiş/kullanılan değerle aynıdır.
  private: double updateRateHz{50.0};
  private: double timeSinceLastPublishSec{0.0};
};

}  // namespace falcon_v2_sensors

#endif  // FALCON_V2_SENSORS_MAGNETOMETER_SYSTEM_HH_

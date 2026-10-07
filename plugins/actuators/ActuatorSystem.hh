// FALCON V2 - Gazebo Sim Harmonic aktüatör/servo System plugin'i.
// Beş kontrol yüzeyi joint'i için fiziksel aktüatör komut zinciri: komut ->
// kırpılmış/hız limitli setpoint -> effort sınırlı tork -> gerçek joint
// hareketi. Sınıf kasten incedir; matematik ActuatorModel.hh'dedir.
// Sınır: burada ArduPlane/MAVLink/SITL/TECS mantığı yoktur - bu plugin
// fiziksel aktüatör arayüzünde biter.
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

/// \brief Beş kontrol yüzeyi aktüatörü; bu sabit sıra entity dizileri, config
/// dizileri ve diagnostik alan sırası boyunca aynen kullanılır.
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

  /// \brief Yüzey başına komut topic geri çağrısı; arayüz fiziksel açı
  /// radyandır. Failsafe politikası SON GEÇERLİ KOMUTU TUT: bu geri çağrı
  /// kalıcı commandRad[idx] durumunu yalnızca yeni mesaj gelince yazar, yeni
  /// mesaj yokken durum olduğu gibi kalır (timeout/nötre dönüş yoktur).
  private: void OnCommand(std::size_t idx, const gz::msgs::Double &_msg);

  /// \brief config/actuator_v1_config.yaml dosyasını (<config_path> SDF
  /// parametresinden gelen yol) this->config[] içine yükler.
  /// \return Başarılıysa true.
  private: bool LoadConfig(const std::string &_path);

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: std::array<gz::sim::Entity, kNumSurfaces> jointEntity{};
  private: std::array<std::string, kNumSurfaces> jointName{};

  private: std::array<ActuatorConfig, kNumSurfaces> config{};
  private: bool validConfig{false};

  /// \brief Yüzey başına kalıcı durum. commandRad = alınan son ham komut
  /// (rad, son geçerli komut tutulur). setpointRad = hız limitli dahili hedef.
  /// integralRadS = PidEffort()'un hata integrali birikimi (rad*s). Üçü de
  /// 0.0'dan (nötr, sıfır birikmiş hata) başlar: "açılışta tekme yok".
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

// FALCON V2 — itki sistemi Gazebo Sim Harmonic System eklentisi (arayüz).
// Kapsam yalnızca motor/ESC/pervane fiziğidir; aerodinamik katsayı, kumanda
// yüzeyi tahriki veya otopilot mantığı içermez. Tüm matematik, Gazebo'dan
// bağımsız olan PropulsionModel.hh içindedir; bu sınıf yalnızca Gazebo
// durumunu okur, omega durumlarını tutar ve kuvveti/torku göbeğe uygular.
#ifndef FALCON_V2_PROPULSION_SYSTEM_HH_
#define FALCON_V2_PROPULSION_SYSTEM_HH_

#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include "PropulsionModel.hh"

namespace falcon_v2_propulsion
{

class PropulsionSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: PropulsionSystem() = default;
  public: ~PropulsionSystem() override = default;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  private: void OnThrottleLeft(const gz::msgs::Double &_msg);
  private: void OnThrottleRight(const gz::msgs::Double &_msg);

  /// \brief Rüzgâr konusu aboneliği (gz.msgs.Vector3d, dünya çerçevesi, m/s);
  /// burada salt okunur, bağıl hava akışı V_axial için kullanılır.
  private: void OnWind(const gz::msgs::Vector3d &_msg);

  /// \brief config/propulsion_v1_config.yaml dosyasını (<config_path> SDF
  /// parametresindeki yol) ve işaret ettiği ayrıştırılmış APC CSV'sini yükler.
  /// \return başarılıysa true.
  private: bool LoadConfig(const std::string &_path);

  /// \brief Bir PreUpdate tikinde tek bir motor için tüm zinciri işletir:
  /// elektrik -> ODE adımı -> aerodinamik yüklenme -> kuvvet/tork uygulaması ->
  /// görsel eklem konumu -> teşhis birikimi. Motor başına bir kez çağrılır.
  private: void StepMotor(
      gz::sim::EntityComponentManager &_ecm,
      double dt,
      const gz::math::Quaterniond &_worldRot,
      const gz::math::Vector3d &_hubOffsetBody,
      double _rotationSign,
      double _throttleCmd,
      double &_omegaOwnState,
      double &_thetaOwnState,
      gz::sim::Entity _propJoint,
      std::vector<double> &_diagOut);

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity baseLinkEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity leftPropJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rightPropJoint{gz::sim::kNullEntity};

  private: MotorConfig motorCfg;
  private: BatteryConfig batteryCfg;
  private: PropellerConfig propCfg;
  private: PhysicsConfig physicsCfg;
  private: ApcTable apcTable;

  private: gz::math::Vector3d leftHubBody{gz::math::Vector3d::Zero};
  private: gz::math::Vector3d rightHubBody{gz::math::Vector3d::Zero};
  private: gz::math::Vector3d thrustAxisBody{1.0, 0.0, 0.0};
  private: double rotationSignLeft{1.0};
  private: double rotationSignRight{-1.0};

  /// \brief Eklentinin kendi tuttuğu rotor açısal hız durumları (rad/s; her
  /// motorun kendi "pozitif = normal hızlanma yönü" çerçevesinde — bkz.
  /// propulsion_v1_config.yaml "rotation" bloğu). Her PreUpdate tikinde
  /// IntegrateRotorStep() ile entegre edilir; asla anlık atanmaz ve asla
  /// Gazebo'nun kendi eklem-torku entegrasyonuyla sürülmez.
  private: double omegaLeft{0.0};
  private: double omegaRight{0.0};

  /// \brief Yalnızca görsel amaçlı pervane eklemi açısal konumu (rad, omega ile
  /// aynı işaret düzeninde). Fizik durumu DEĞİLDİR: PropulsionModel.hh'ye geri
  /// beslenmez, sadece pervane meshi omega ile eşzamanlı dönsün diye vardır.
  /// Her tikte theta += omega*dt olarak entegre edilip Joint::ResetPosition()
  /// ile yazılır; Joint::SetVelocity() bilinçli olarak kullanılmaz, çünkü o
  /// kuvvet tabanlı bir servodur ve eklem üzerinden base_link'e, aşağıdaki
  /// açık AddWorldWrench() reaksiyon torkunu çiftleyen ek bir tork bindirir.
  private: double thetaLeft{0.0};
  private: double thetaRight{0.0};

  private: bool validConfig{false};

  private: gz::transport::Node node;
  private: gz::math::Vector3d windWorld{gz::math::Vector3d::Zero};
  private: double throttleLeftCmd{0.0};
  private: double throttleRightCmd{0.0};

  private: gz::transport::Node::Publisher diagPub;
  private: double diagRateHz{20.0};
  private: double diagAccumulator{0.0};
};

}  // namespace falcon_v2_propulsion

#endif  // FALCON_V2_PROPULSION_SYSTEM_HH_

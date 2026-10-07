// FALCON V2 — Gazebo Sim Harmonic aerodinamik System eklentisi.
// Kapsam yalnızca aerodinamik kuvvet/moment fiziğidir: itki, kumanda eyletimi
// ve otopilot bu sınıfın dışındadır (kumanda eklem açıları yalnızca OKUNUR).
// Sınıf kasten incedir; tüm matematik Gazebo'dan bağımsız AeroModel.hh'de durur.
#ifndef FALCON_V2_AERODYNAMICS_SYSTEM_HH_
#define FALCON_V2_AERODYNAMICS_SYSTEM_HH_

#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>

#include "AeroModel.hh"

namespace falcon_v2_aero
{

class AerodynamicsSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: AerodynamicsSystem() = default;
  public: ~AerodynamicsSystem() override = default;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Rüzgâr konusu geri çağrımı. Rüzgâr hızı DÜNYA çerçevesinde, m/s;
  /// hiç mesaj gelmediyse sıfır rüzgâr varsayılır.
  private: void OnWind(const gz::msgs::Vector3d &_msg);

  /// \brief config/aero_v1_config.yaml dosyasını (<config_path> SDF
  /// parametresindeki yoldan) this->config içine yükler.
  private: bool LoadConfig(const std::string &_path);

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity baseLinkEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity leftAileronJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rightAileronJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity leftElevatorJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rightElevatorJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rudderJoint{gz::sim::kNullEntity};

  private: AeroConfig config;
  private: bool validConfig{false};

  private: gz::transport::Node node;
  private: gz::math::Vector3d windWorld{gz::math::Vector3d::Zero};

  /// \brief Tanı yayıncısı; konu adı ve alan sırası Configure() sırasında bir
  /// kez günlüğe yazılır.
  private: gz::transport::Node::Publisher diagPub;
  private: double diagRateHz{20.0};
  private: double diagAccumulator{0.0};
};

}  // namespace falcon_v2_aero

#endif  // FALCON_V2_AERODYNAMICS_SYSTEM_HH_

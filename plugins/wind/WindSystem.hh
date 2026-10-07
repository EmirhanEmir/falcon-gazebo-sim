// FALCON V2 — Gazebo Sim Harmonic rüzgâr/hamle üreticisi System eklentisi.
// Kapsam yalnızca rüzgâr HIZI yayınlamaktır: V_wind_world = V_steady + V_gust(t)
// her PreUpdate() tikinde /model/falcon_v2/wind konusunda yayınlanır. Bu eklenti
// hiçbir bağa doğrudan kuvvet/tork UYGULAMAZ; rüzgârın tüm etkisi tüketicilerin
// Vrel = Vbody - Vwind hesabından geçer. Tüm matematik WindModel.hh'dedir.
#ifndef FALCON_V2_WIND_SYSTEM_HH_
#define FALCON_V2_WIND_SYSTEM_HH_

#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/double_v.pb.h>

#include "WindModel.hh"

namespace falcon_v2_wind
{

class WindSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: WindSystem() = default;
  public: ~WindSystem() override = default;

  public: void Configure(
              const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
              const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Canlı güncellenebilir sabit rüzgâr komutu (gz.msgs.Vector3d, dünya
  /// çerçevesi, m/s). Değeri tümüyle üzerine yazar; sonlu olmayan bileşenler
  /// reddedilir ve son geçerli değer korunur (NaN/Inf asla yayılmaz).
  private: void OnSteadyCmd(const gz::msgs::Vector3d &_msg);

  /// \brief Tek bir deterministik 1-kosinüs hamlesini zamanlar (veya
  /// mevcut olanı değiştirir). gz.msgs.Double_V, SABİT alan sırası, tam 6 alan:
  ///   [0] dir_x, [1] dir_y, [2] dir_z  - ham yön vektörü, dünya çerçevesi.
  ///       Önceden normalize edilmiş olması gerekmez; bu geri çağrım
  ///       NormalizeDirection() ile kendisi normalize eder. Sıfıra yakın
  ///       normlu bozuk yön REDDEDİLİR (günlüğe yazılır, hamle zamanlanmaz,
  ///       önceki hamleye dokunulmaz).
  ///   [3] amplitude_mps                - tepe hamle hızı, m/s. Negatif
  ///       olabilir; bu, hamlenin verilen yönün tersine esmesi anlamına gelir.
  ///   [4] start_delay_s                - BU KOMUTUN ALINDIĞI andan itibaren
  ///       saniye (mutlak benzetim zamanı değil), böylece çağıran taraf
  ///       mevcut benzetim zamanını ayrıca sorgulamadan "N saniye sonra"
  ///       diyebilir.
  ///   [5] duration_s                   - saniye; sıfırdan büyük OLMALIDIR.
  ///       Pozitif olmayan değer reddedilir.
  /// Alan sayısı farklı ya da herhangi bir alanı sonlu olmayan mesaj da
  /// kısmen uygulanmak yerine tümüyle REDDEDİLİR.
  /// V1 SINIRI: geçerli yeni bir komut gustState'i koşulsuz olarak tümüyle
  /// değiştirir; kuyruk veya üst üste bindirme yoktur.
  private: void OnGustCmd(const gz::msgs::Double_V &_msg);

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher windPub;

  /// \brief Kalıcı sabit rüzgâr durumu (dünya çerçevesi, m/s). Configure()
  /// anında <steady_wind_mps> SDF parametresinden kurulur (yoksa sıfır
  /// vektör), sonra steady_cmd konusundan canlı olarak değiştirilebilir.
  private: gz::math::Vector3d steadyWindWorld{gz::math::Vector3d::Zero};

  /// \brief Kalıcı tek hamle durumu. Başlangıçta scheduled=false olduğundan,
  /// geçerli bir hamle komutu gelene kadar EvaluateGust() tam olarak Zero döner.
  private: GustState gustState{};

  /// \brief En son tamamlanan PreUpdate() tikindeki benzetim zamanı (saniye);
  /// OnGustCmd() bunu "komutun alınma zamanı" referansı olarak kullanır.
  /// Tik başına bir kez güncellendiği için komutun gerçek varış zamanına bir
  /// fizik adımı hassasiyetinde yakındır. Bu düz değer yazmalarının etrafında
  /// kilit yoktur (geri çağrımlar taşıma iş parçacığından, PreUpdate() fizik
  /// iş parçacığından erişir); bu, projenin diğer asenkron komut konularıyla
  /// aynı kabul edilmiş desendir.
  private: double lastSimTimeSec{0.0};
};

}  // namespace falcon_v2_wind

#endif  // FALCON_V2_WIND_SYSTEM_HH_

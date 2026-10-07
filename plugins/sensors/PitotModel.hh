// FALCON V2 - Pitot (simüle airspeed) sensörünün çekirdek matematiği.
// Saf matematik: yalnızca gz-math7, gz-sim/ECM bağımlılığı yok.
// Sınır: burada yalnızca Vrel -> airspeed -> dinamik basınç hesabı vardır;
// canlı durum okuma ve yayınlama PitotSystem.cc'nin işidir.
//
// NEDEN YERLEŞİK gz-sim air_speed SENSÖRÜ YERİNE ÖZEL PLUGIN: kurulu sdformat
// air_speed sensör şemasında rüzgârla ilgili hiçbir yapılandırılabilir alan
// yoktur (yalnızca bir "pressure" gürültü bloğu), dolayısıyla bu projenin
// /model/falcon_v2/wind gz-transport topic'ini tüketemez. Gazebo'nun tek
// yerleşik rüzgâr mekanizması (gz-sim-wind-effects-system ile world
// seviyesindeki <wind><linear_velocity>) ise KUVVET uygulayan, bu projenin hız
// yayınlayan WindSystem'iyle ilgisiz ayrı bir mekanizmadır; onu kullanmak ya
// komut edilen rüzgârı sessizce yok sayar ya da ikinci ve gereksiz bir rüzgâr
// tüketim yolu gerektirirdi. Bu plugin bunun yerine mevcut
// /model/falcon_v2/wind topic'ine doğrudan abone olur - aerodinamik ve itki
// plugin'lerinin yaptığının aynısı - ve airspeed'i aşağıdaki ilk ilkelerden
// hesaplar.
//
// FİZİKSEL MODEL (V1, sıkıştırılamaz akış - bu uçağın uçuş zarfında deniz
// seviyesinde Mach ~0.09, yani sıkıştırılamaz rejimin derinliğinde):
//   V_rel_world  = V_point_world - V_wind_world   (proje genelindeki Vrel
//                  konvansiyonu)
//   airspeed_mps = |V_rel_world|                  (çerçeveden bağımsız
//                  büyüklük: dönüşler vektör boyunu korur)
//   q_pa         = 0.5 * rho * airspeed_mps^2     (standart sıkıştırılamaz
//                  dinamik basınç; bu hızlarda gerçek bir pitot-statik
//                  sistemin diferansiyel portunun ölçtüğü büyüklük)
//
// rho (hava yoğunluğu, kg/m^3) burada uydurulmaz: çağıran bunu
// config/aero_v1_config.yaml içindeki environment.air_density_rho_kg_m3
// değerinden (ISA deniz seviyesi, 1.225) vermek zorundadır. O değer bu sabitin
// tek kaynağıdır ve config/propulsion_v1_config.yaml'deki
// environment.rho_kg_m3 ile paylaşılır.
//
// V1 SINIRI: bu yalnızca skaler dinamik basınç modelidir. Pitot portuna ters
// akış/negatif AoA, sıkıştırılabilirlik veya montaj (position error) düzeltmesi
// modellenmez. q_pa her zaman >= 0'dır.
#ifndef FALCON_V2_SENSORS_PITOT_MODEL_HH_
#define FALCON_V2_SENSORS_PITOT_MODEL_HH_

#include <gz/math/Vector3.hh>

namespace falcon_v2_sensors
{

/// \brief V_rel_world = V_point_world - V_wind_world. İki girdi de AYNI
/// çerçevede (projedeki her çağrı yerinde dünya çerçevesinde) olmak
/// ZORUNDADIR; bu fonksiyon bunu kendisi denetlemez, sorumluluk çağıranındır.
inline gz::math::Vector3d AirRelativeVelocityWorld(
    const gz::math::Vector3d &_vPointWorld,
    const gz::math::Vector3d &_vWindWorld)
{
  return _vPointWorld - _vWindWorld;
}

/// \brief Skaler airspeed (m/s): bağıl hava hızının çerçeveden bağımsız
/// büyüklüğü. Her zaman >= 0.
inline double AirspeedMps(const gz::math::Vector3d &_vRelWorld)
{
  return _vRelWorld.Length();
}

/// \brief Verilen skaler airspeed ve hava yoğunluğu için standart
/// sıkıştırılamaz dinamik/diferansiyel basınç (Pa). Her zaman >= 0 (V1 sınırı:
/// ters akış altında gerçek bir pitotun İŞARETLİ diferansiyel basıncı
/// modellenmez).
inline double DifferentialPressurePa(double _airspeedMps, double _rhoKgM3)
{
  return 0.5 * _rhoKgM3 * _airspeedMps * _airspeedMps;
}

}  // namespace falcon_v2_sensors

#endif  // FALCON_V2_SENSORS_PITOT_MODEL_HH_

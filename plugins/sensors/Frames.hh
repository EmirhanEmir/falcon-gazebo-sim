// FALCON V2 - Sensör paketinin referans çerçevesi dönüşüm kütüphanesi.
// Saf matematik: yalnızca gz-math7, gz-sim/ECM bağımlılığı yok. Çağıranlar
// hazır vektör/kuaterniyonları verir, burada canlı Gazebo durumu okunmaz.
// Sınır: projedeki her çerçeve/işaret dönüşümü BİR KEZ burada türetilir, başka
// hiçbir dosya aynı dönüşümü yeniden türetmez.
//
// -----------------------------------------------------------------------------
// GEÇERLİ ÇERÇEVE GERÇEKLERİ
//   - Bu projenin Gazebo gövde çerçevesi FLU'dur (+X ileri, +Y sol, +Z yukarı).
//     Resmî ardupilot_gazebo ArduPilotPlugin'in varsayılan olarak kabul ettiği
//     gövde çerçevesi de tam olarak budur (gazeboXYZToNED dönüşümü).
//   - ArduPilot'un kendi gövde çerçevesi FRD'dir (+X ileri, +Y sağ, +Z aşağı).
//   - Bu projede Gazebo dünya çerçevesi ENU'dur (+X doğu, +Y kuzey, +Z yukarı);
//     sdformat'ın spherical_coordinates varsayılanı ile aynı
//     (world_frame_orientation = "ENU").
//   - ArduPilot'un dünya çerçevesi NED'dir (+X kuzey, +Y doğu, +Z aşağı).
// -----------------------------------------------------------------------------
#ifndef FALCON_V2_SENSORS_FRAMES_HH_
#define FALCON_V2_SENSORS_FRAMES_HH_

#include <cmath>

#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace falcon_v2_sensors
{

// =============================================================================
// FLU <-> FRD (gövde çerçevesindeki serbest vektörler: açısal hız, özgül
// kuvvet/doğrusal ivme, magnetometrenin gövde çerçevesi alanı ve gövdeye göre
// her konum/ofset vektörü)
// =============================================================================
//
// TÜRETME: ardupilot_gazebo plugin'inin kendi varsayılanı
// gazeboXYZToNED = Pose3d(0,0,0, GZ_PI,0,0), yani gövdenin kendi X ekseni
// etrafında 180 derecelik bir dönüştür. MATRİS olarak R = diag(1,-1,-1)
// (standart X etrafı pi dönüşü: [[1,0,0],[0,cos(pi),-sin(pi)],
// [0,sin(pi),cos(pi)]] = [[1,0,0],[0,-1,0],[0,0,-1]]). Bir FLU vektörüne
// uygulandığında R*(x,y,z)^T = (x,-y,-z): X (ileri) DEĞİŞMEZ, Y ve Z işaret
// değiştirir. Bu doğrudan FLU->FRD'dir - ileri ileri kalır; sol (+Y_flu) -Y_frd
// olur, yani sağ artık +Y_frd'dir; yukarı (+Z_flu) -Z_frd olur, yani aşağı
// artık +Z_frd'dir. R ortogonaldir (R^T R = I), properdir (det(R) = +1, yansıma
// değil gerçek bir dönüş) ve iki uygulamada birim olur (R^2 = I: 180 derecelik
// bir dönüş kendi tersidir, yani FLU->FRD ile FRD->FLU AYNI formüldür).
//
// Uçağa rijit bağlı herhangi bir sıradan gövde vektörü (açısal hız, özgül
// kuvvet, magnetometrenin gövde çerçevesi okuması, bir sensörün gövde
// orijinine göre montaj ofseti) AYNI formülle dönüşür: bunların hepsi "serbest
// vektör"dür ve aynı cisme rijit bağlı iki çerçeve arasındaki sabit bir dönüş
// altında diğer her vektör gibi döner. Fiziksel büyüklük başına ayrı bir
// formül gerekmez.
inline gz::math::Vector3d FluFrdSwap(const gz::math::Vector3d &_v)
{
  return gz::math::Vector3d(_v.X(), -_v.Y(), -_v.Z());
}

// =============================================================================
// ENU <-> NED (dünya çerçevesindeki serbest vektörler: GPS ENU hızından NED
// hızına, dünya çerçevesi yer manyetik alanı veya dünyaya göre her
// konum/yerdeğiştirme vektörü)
// =============================================================================
//
// TÜRETME: ENU (Doğu,Kuzey,Yukarı) -> NED (Kuzey,Doğu,Aşağı) ilk iki bileşeni
// yer değiştirir ve üçüncüyü işaret değiştirir: f(x,y,z) = (y,x,-z). Matris
// olarak M = [[0,1,0],[1,0,0],[0,0,-1]]. M ortogonaldir (M^T M = I) ve
// properdir (det(M) = +1): yatay Kuzey-Doğu açıortay ekseni (1,1,0)/sqrt(2)
// etrafında 180 derecelik gerçek bir dönüş - trace(M) = 0+0-1 = -1 =
// 1+2*cos(theta) olduğundan theta = 180 derece. İki uygulamada birim olur
// (M^2 = I): ENU->NED ile NED->ENU, yukarıdaki FLU/FRD durumunda olduğu gibi,
// AYNI formüldür - ancak iki dönüş FARKLI eksenler etrafındadır ve asla
// birbirinin yerine kullanılmamalıdır (aşağıdaki uyarı).
inline gz::math::Vector3d EnuNedSwap(const gz::math::Vector3d &_v)
{
  return gz::math::Vector3d(_v.Y(), _v.X(), -_v.Z());
}

// =============================================================================
// AÇIK UYARI: FluFrdSwap() ile EnuNedSwap() cebirsel olarak FARKLI dönüşlerdir
// (farklı fiziksel eksenler etrafında - gövde X'i ile yatay Kuzey-Doğu
// açıortayı) ve yalnızca "ikisi de 180 derecelik bir involüsyon" özelliğini
// paylaşırlar. FluFrdSwap()'ı bir DÜNYA vektörüne, ya da EnuNedSwap()'ı bir
// GÖVDE vektörüne uygulamak çerçeve kategorisi hatasıdır: sessizce yanlış ama
// sonlu bir sonuç üretir. Her çağrı yeri, dönüştürdüğü vektörün hangi
// kategoriye (gövde mi dünya mı) ait olduğunu adlandırabilmek zorundadır.
// =============================================================================

// =============================================================================
// İrtifa (dünya "yukarı" yönü, metre, bu projenin dünya orijini üzerinde
// pozitif) <-> NED "Down" bileşeni. Tanım gereği aşikâr (Down = -Up), ama
// skaler irtifa ile 3-vektör üzerinde çalışan EnuNedSwap() karıştırılmasın
// diye ayrıca yazılmıştır.
// =============================================================================
inline double AltitudeToNedDown(double _altitudeM) { return -_altitudeM; }
inline double NedDownToAltitude(double _downM) { return -_downM; }

// =============================================================================
// Attitude kuaterniyonu konvansiyon değişimi.
// =============================================================================
// Girdi: qFluRelEnu - gövde FLU çerçevesinin dünya ENU çerçevesine göre
// yönelimi; gz-sim'in IMU sensörünün (localization=ENU) ve gz::math::Quaternion
// sınıfının kendi kullandığı standart "gövdeden dünyaya" aktif dönüş anlamında:
// herhangi bir gövde vektörü v_body için q.RotateVector(v_body) aynı fiziksel
// vektörü dünya koordinatlarında verir (v_world = q * v_body * q^-1).
//
// Çıktı: gövde FRD çerçevesinin dünya NED çerçevesine göre eşdeğer yönelimi,
// aynı aktif dönüş anlamında; yani her v_frd için
//   result.RotateVector(v_frd) == EnuNedSwap(qFluRelEnu.RotateVector(FluFrdSwap(v_frd)))
// Bu kimlik, elle seçilmiş "bilinen" bir Euler açısına dayanmayan bir
// öz-tutarlılık kontrolüdür ve sayısal olarak doğrulanabilir.
//
// TÜRETME: R = FLU<->FRD dönüş matrisi (kendi tersi), R' = ENU<->NED dönüş
// matrisi (kendi tersi), Q_fe = qFluRelEnu'nun dönüş matrisi olsun.
// v_body_flu = R * v_body_frd (R kendi tersi) ifadesini
// v_world_enu = Q_fe * v_body_flu içine koyup ardından
// v_world_ned = R' * v_world_enu uygulanırsa
// v_world_ned = (R' * Q_fe * R) * v_body_frd elde edilir; yani aranan
// FRD->NED dönüş matrisi R' * Q_fe * R'dir. Kuaterniyon çarpımı olarak
// (gz::math::Quaternion için Hamilton çarpımı bileşkesi matris çarpımı
// bileşkesiyle örtüşür): result = qEnuToNedRot * qFluRelEnu * qFluToFrdRot.
// Burada qFluToFrdRot gövde X'i etrafındaki 180 derecelik dönüşün,
// qEnuToNedRot ise dünya (1,1,0)/sqrt(2) ekseni etrafındaki 180 derecelik
// dönüşün kuaterniyonudur - yukarıda FluFrdSwap()/EnuNedSwap() için türetilen
// iki dönüşün, 3x3 matris yerine kuaterniyon olarak uygulanmış hâli.
inline gz::math::Quaterniond AttitudeFluEnuToFrdNed(
    const gz::math::Quaterniond &_qFluRelEnu)
{
  static const gz::math::Quaterniond kQFluToFrdRot(
      gz::math::Vector3d(1.0, 0.0, 0.0), GZ_PI);
  static const gz::math::Quaterniond kQEnuToNedRot(
      gz::math::Vector3d(1.0, 1.0, 0.0).Normalized(), GZ_PI);
  return kQEnuToNedRot * _qFluRelEnu * kQFluToFrdRot;
}

// =============================================================================
// Heading (radyan, standart havacılık/pusula konvansiyonu: 0 = kuzey, yukarıdan
// bakıldığında doğuya doğru SAAT YÖNÜ pozitif), bir NED vektörünün (manyetik
// alan ya da yer izi hızı) yatay (kuzey, doğu) bileşenlerinden. Saat yönü
// konvansiyonunu üreten şey atan2(east, north)'tur - atan2(north, east) DEĞİL:
// yalnızca kuzey girdisi 0 rad, yalnızca doğu girdisi +pi/2 rad verir.
// =============================================================================
inline double HeadingFromNedHorizontalRad(double _north, double _east)
{
  return std::atan2(_east, _north);
}

}  // namespace falcon_v2_sensors

#endif  // FALCON_V2_SENSORS_FRAMES_HH_

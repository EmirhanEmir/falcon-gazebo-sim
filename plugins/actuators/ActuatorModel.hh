// FALCON V2 - Aktüatör/servo çekirdek matematiği: travel clamp, rate limit ve
// effort sınırlı PID. Saf matematik - gz-sim/gz-math bağımlılığı yoktur, tüm
// büyüklükler skalerdir (beş yüzey de tek serbestlik dereceli hinge).
// Sınır: fizik entegrasyonu burada değil, fizik motorunda yapılır.
#ifndef FALCON_V2_ACTUATOR_MODEL_HH_
#define FALCON_V2_ACTUATOR_MODEL_HH_

#include <algorithm>
#include <cmath>

namespace falcon_v2_actuators
{

/// \brief "Hız limitine dayanmış" sayılma eşiği: |actual_rate| >= bu oran *
/// cfg.maxRateRadS. Fiziksel katsayı ya da kazanç değil, tespit eşiğidir;
/// provenance: actuator_v1_config.yaml / rate_saturation_antiwindup_note.
inline constexpr double kRateSaturationDetectFraction = 0.999;

/// \brief Tek bir yüzeyin aktüatör parametreleri. Tamamı
/// config/actuator_v1_config.yaml'den ActuatorSystem.cc::Configure()
/// tarafından yüklenir; bu başlıkta hiçbir katsayı sabitlenmemiştir.
/// Varsayılanlar yalnızca eksik bir config anahtarının tanımsız bellek yerine
/// sonlu ve hareketsiz (inert) bir davranışa düşmesi içindir.
struct ActuatorConfig
{
  /// \brief Mekanik hareket limitleri, rad (V1: +/-45 deg).
  double minAngleRad = 0.0;
  double maxAngleRad = 0.0;

  /// \brief Servo slew-rate limiti, rad/s.
  double maxRateRadS = 0.0;

  /// \brief Aktüatörün azami efforu (joint torku), N*m.
  double maxEffortNm = 0.0;

  /// \brief PID kazançları, yüzey başına. Her yüzeyin KENDİ MENTEŞE EKSENİ
  /// etrafındaki ataletine göre türetilir (parallel-axis düzeltmeli), link'in
  /// kütle merkezi ataletine göre değil.
  double kp = 0.0;  // N*m / rad
  double kd = 0.0;  // N*m / (rad/s)

  /// \brief İntegral kazancı. Sabit bir bozucu torku (yüzeyin kendi
  /// ağırlığının menteşe etrafındaki momenti) altında saf P(D) yasası kalıcı
  /// hatayı sıfırlayamaz; integral etki bu bozucuyu modellemeye gerek
  /// kalmadan bastırır. 0.0 varsayılanı eski saf P(D) davranışıdır.
  double ki = 0.0;  // N*m / (rad*s), hata integrali durumunu çarpar

  /// \brief Yalnızca P terimine uygulanan setpoint ağırlığı (standart iki
  /// serbestlik dereceli PID). I terimi ve anti-windup işaret mantığı her
  /// zaman gerçek/ağırlıksız hatayı kullanır: böylece bozucu bastırma aynen
  /// korunurken sadece P teriminin setpoint basamağına ilk tepkisi yumuşar.
  /// 1.0 = ağırlıksız, yani bu tekniğin eklenmesinden önceki davranış.
  double spWeightB = 1.0;  // boyutsuz, [0, 1] aralığında
};

/// \brief Ham komut açısını (rad) mekanik hareket limitlerine kırpar. Saf
/// doygunluk - hız/zaman bağımlılığı yoktur.
inline double ClampCommand(double rawCmdRad, const ActuatorConfig &cfg)
{
  double lo = cfg.minAngleRad;
  double hi = cfg.maxAngleRad;
  if (lo > hi) std::swap(lo, hi);
  return std::clamp(rawCmdRad, lo, hi);
}

/// \brief Kalıcı dahili setpoint durumunu bir zaman adımında
/// targetClampedRad'a doğru EN FAZLA maxRateRadS * dt kadar ilerletir.
/// "Sonlu hız, anlık atlama yok" garantisi tamamen bu tek kırpmadan gelir:
/// hedef ne kadar uzak olursa olsun dönen değer prevSetpointRad'dan
/// maxRateRadS * dt'den fazla sapamaz.
inline double RateLimitSetpoint(
    double prevSetpointRad, double targetClampedRad, double maxRateRadS,
    double dt)
{
  if (dt <= 0.0 || maxRateRadS <= 0.0)
  {
    // Pozitif olmayan dt/hız "bu adımda hareket etme" demektir, asla atlama.
    return prevSetpointRad;
  }
  const double maxStep = maxRateRadS * dt;
  const double delta =
      std::clamp(targetClampedRad - prevSetpointRad, -maxStep, maxStep);
  return prevSetpointRad + delta;
}

/// \brief Kinematik komut zincirinin (clamp + rate limit) tek adımlık sonucu.
struct SetpointStepResult
{
  double targetClampedRad = 0.0;  ///< mekanik kırpma sonrası hedef
  bool targetWasClamped = false;  ///< ham komut [min,max] dışındaysa true
  double setpointRad = 0.0;       ///< bu adım sonundaki hız limitli setpoint
  bool rateLimitActive = false;   ///< hareket hız limitiyle kısıtlandıysa true
};

/// \brief Clamp ve rate limit aşamalarını birleştirir; çağıranlar ikisini ayrı
/// ayrı değil bunun üzerinden kullanır, böylece birleştirme biçimi tek yerde
/// tanımlı kalır.
inline SetpointStepResult StepSetpoint(
    double rawCmdRad, double prevSetpointRad, const ActuatorConfig &cfg,
    double dt)
{
  SetpointStepResult out;
  out.targetClampedRad = ClampCommand(rawCmdRad, cfg);
  out.targetWasClamped = (out.targetClampedRad != rawCmdRad);
  out.setpointRad =
      RateLimitSetpoint(prevSetpointRad, out.targetClampedRad, cfg.maxRateRadS, dt);
  out.rateLimitActive = (out.setpointRad != out.targetClampedRad);
  return out;
}

/// \brief PID kontrol yasasının tek adımlık sonucu.
struct EffortResult
{
  double torqueNm = 0.0;
  bool effortClamped = false;

  /// \brief Güncellenmiş integral durumu (hata integrali, rad*s). Çağıran bunu
  /// saklamak ve aynı yüzeyin SONRAKİ çağrısında prevIntegralRadS olarak geri
  /// vermek zorundadır - setpointRad'ın saklandığı gibi.
  double integralRadS = 0.0;

  /// \brief Bu çağrıda anti-windup integral birikimini dondurduysa true.
  bool integralFrozen = false;

  /// \brief Dondurmayı özellikle hız doygunluğu maddesi tetiklediyse true.
  bool rateSatFreezeActive = false;

  /// \brief Joint bu çağrıda hız limitinde gözlendiyse true (dondurma yönsel
  /// olduğu için bu tek başına dondurma anlamına gelmez).
  bool rateSaturated = false;
};

/// \brief Hız limitli setpoint'i izleyen PID kontrol yasası, "türev ölçüm
/// üzerinden" formunda: kd gerçek joint'in kendi açısal hızını çarpar,
/// setpoint türevini değil - setpoint basamağında türev tekmesi oluşmaz.
/// Çıkış +/-cfg.maxEffortNm'ye kırpılır; aynı sınır ActuatorSystem.cc'de
/// gz::sim::Joint::SetEffortLimits() ile fizik motoru seviyesinde de
/// uygulanır (tek mekanizmaya güvenmemek için).
///
/// Çıkış bir POZİSYON yazımı değil, gerçek bir joint-space torkudur
/// (ActuatorSystem.cc, gz::sim::Joint::SetForce): pozisyon yazmak joint'i
/// fizik motorunu atlayarak ışınlar ve hız/effort limitlerini uygulanamaz
/// hâle getirir.
///
/// SETPOINT AĞIRLIĞI: P terimi errorP = cfg.spWeightB*setpointRad -
/// actualAngleRad kullanır. I terimi ve anti-windup işaret mantığı GERÇEK
/// (ağırlıksız) hatayı kullanmayı sürdürür; bu yüzden teknik kapalı çevrim
/// karakteristik polinomunu - dolayısıyla kp/kd/ki kutup yerleşimini -
/// değiştirmez, yalnızca referans izleme transfer fonksiyonunun bir SIFIRINI
/// kaydırır. cfg.spWeightB=1.0 iken errorP==error ve hesap bu teknik
/// eklenmeden önceki hâliyle birebir aynıdır.
///
/// ANTI-WINDUP ("koşullu integral"): integral birikimi üç durumda dondurulur.
///   1) Aşama 1 hedefi kırptıysa - hız limitli setpoint kırpılmış bir ham
///      komuta hiçbir zaman ulaşamaz, ona karşı biriktirmek boşunadır.
///   2) ÖNCEKİ integral durumuyla (ve ağırlıklı P terimiyle) hesaplanan
///      çıkış zaten +/-maxEffortNm'de doygun ve gerçek hata aynı işaretliyse
///      - biriktirmek yalnızca aynı doymanın derinine iter. Aynı tick içinde
///      değerlendirilir, bir tick gecikmeli bayrak okumaz.
///   3) Joint fizik motorunun hız limitinde (bkz.
///      kRateSaturationDetectFraction) ve hareket yönü hatanın istediği yönle
///      aynıysa - joint hatayı fiziksel olarak daha hızlı kapatamaz, birikim
///      yalnızca karşılanamayan talep üretir. Bu kısıt
///      SetVelocityLimits() ile bu kontrol yasasının DIŞINDA konur, yani yasa
///      onu ancak joint hızından görebilir.
/// Üç madde de yönseldir, bu yüzden integralin GERİ SARMASI hiç engellenmez;
/// 3. madde joint hız limitinde değilken kanıtlanabilir biçimde etkisizdir,
/// dolayısıyla doymamış normal izleme birebir aynı kalır.
///
/// \param[in] prevIntegralRadS Aynı yüzeyin önceki çağrısından taşınan hata
///   integrali (ilk çağrıda / sıfırlamadan sonra 0.0).
/// \param[in] dt Zaman adımı, s. Yalnızca integral biriktiriminde kullanılır.
/// \param[in] targetClampActive Aşama 1'in kırpma sonucu
///   (SetpointStepResult::targetWasClamped); anti-windup girdisi.
inline EffortResult PidEffort(
    double setpointRad, double actualAngleRad, double actualRateRadS,
    double prevIntegralRadS, double dt, bool targetClampActive,
    const ActuatorConfig &cfg)
{
  EffortResult out;
  const double error = setpointRad - actualAngleRad;
  const double errorP = cfg.spWeightB * setpointRad - actualAngleRad;

  const double rawBeforeIntegration =
      cfg.kp * errorP + cfg.ki * prevIntegralRadS - cfg.kd * actualRateRadS;
  const bool wouldSaturate = cfg.maxEffortNm > 0.0 &&
      std::abs(rawBeforeIntegration) >= cfg.maxEffortNm;
  const bool sameSignAsError =
      (rawBeforeIntegration >= 0.0 && error >= 0.0) ||
      (rawBeforeIntegration <= 0.0 && error <= 0.0);

  const bool rateSaturated = cfg.maxRateRadS > 0.0 &&
      std::abs(actualRateRadS) >=
          kRateSaturationDetectFraction * cfg.maxRateRadS;
  const bool rateSameSignAsError =
      (actualRateRadS >= 0.0 && error >= 0.0) ||
      (actualRateRadS <= 0.0 && error <= 0.0);
  const bool rateSatFreeze = rateSaturated && rateSameSignAsError;

  const bool freeze =
      targetClampActive || (wouldSaturate && sameSignAsError) || rateSatFreeze;

  double integral = prevIntegralRadS;
  if (!freeze && dt > 0.0)
  {
    integral = prevIntegralRadS + error * dt;
  }

  const double raw =
      cfg.kp * errorP + cfg.ki * integral - cfg.kd * actualRateRadS;
  double clamped = raw;
  if (cfg.maxEffortNm > 0.0)
  {
    clamped = std::clamp(raw, -cfg.maxEffortNm, cfg.maxEffortNm);
  }
  out.torqueNm = clamped;
  out.effortClamped = (clamped != raw);
  out.integralRadS = integral;
  out.integralFrozen = freeze;
  out.rateSatFreezeActive = rateSatFreeze;
  out.rateSaturated = rateSaturated;
  return out;
}

}  // namespace falcon_v2_actuators

#endif  // FALCON_V2_ACTUATOR_MODEL_HH_

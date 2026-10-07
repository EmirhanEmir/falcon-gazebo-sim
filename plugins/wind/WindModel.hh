// FALCON V2 — rüzgâr/hamle üreticisinin matematik çekirdeği.
// Saf matematik; gz-sim bağımlılığı yoktur, tek bağımlılık gz-math7'dir.
// Kapsam kasten sınırlıdır: sabit bir rüzgâr bileşeni üzerine bindirilmiş tek
// bir deterministik 1-kosinüs hamlesi; çıktı DÜNYA çerçevesinde hava kütlesi
// hızıdır (m/s). Uzamsal gradyan, arazi/iz etkileri ve stokastik türbülans
// (Dryden, Von Karman) bu modelde yoktur.
#ifndef FALCON_V2_WIND_MODEL_HH_
#define FALCON_V2_WIND_MODEL_HH_

#include <cmath>

#include <gz/math/Vector3.hh>

namespace falcon_v2_wind
{

/// \brief Hamle yön vektörü normalize edilirken "bozuk olmayan" sayılan en
/// küçük norm. Bunun altında yön bozuk kabul edilir ve normalize EDİLMEZ;
/// aksi hâlde sıfıra yakın bir norma bölmek keyfi büyüklükte veya NaN bir
/// birim vektör üretirdi. Fiziksel bir parametre değil, yalnızca sayısal
/// güvenlik sabitidir.
constexpr double kMinDirectionNorm = 1e-9;

/// \brief Zamanlanmış tek bir 1-kosinüs hamlesinin parametreleri; çağıran
/// taraf (WindSystem.cc OnGustCmd()) bu yapıyı kurmadan ÖNCE
/// NormalizeDirection() çağırmak zorundadır. Bu başlık kendi başına hiçbir
/// normalizasyon yapmaz, böylece EvaluateGust() doğrulanmış girdiler üzerinde
/// saf bir fonksiyon olarak kalır.
/// Yapısal V1 SINIRI: tek bir yapı örneği olduğu için aynı anda yalnızca bir
/// hamle temsil edilebilir; yeni bir komut bu yapının içeriğini tümüyle
/// DEĞİŞTİRİR, sürmekte olan hamleyle üst üste bindirilmez.
struct GustState
{
  /// Birim vektör, dünya çerçevesi. `scheduled` false ise anlamsızdır.
  gz::math::Vector3d directionUnit = gz::math::Vector3d::Zero;
  /// Profilin orta noktasındaki tepe hamle hızı, m/s.
  double amplitudeMps = 0.0;
  /// Bu hamlenin [0, duration] penceresinin başladığı mutlak benzetim
  /// zamanı, saniye.
  double startTimeSec = 0.0;
  /// Saniye; sıfırdan büyük olmadıkça hamle hiç çıktı üretmez.
  double durationSec = 0.0;
  /// False = hiç geçerli hamle komutu alınmadı; bu durumda EvaluateGust()
  /// diğer alanların değerine bakmaksızın her zaman Zero döner.
  bool scheduled = false;
};

/// \brief Bir hamle komutunun ham yön vektörünü normalize eder. Büyüklük
/// yoksayılır (genlik ayrı bir alandır). `_validOut`, normun
/// kMinDirectionNorm'dan büyük olup olmadığını bildirir; çağıran taraf bunu
/// kontrol edip bozuk yönlü komutu reddetmek ZORUNDADIR.
inline gz::math::Vector3d NormalizeDirection(
    const gz::math::Vector3d &_raw, bool &_validOut)
{
  const double norm = _raw.Length();
  if (norm < kMinDirectionNorm)
  {
    _validOut = false;
    return gz::math::Vector3d::Zero;
  }
  _validOut = true;
  return _raw / norm;
}

/// \brief 1-kosinüs hamle zarfı (boyutsuz, [0,1] aralığında). `_t` hamlenin
/// KENDİ başlangıcından ölçülür:
///   zarf(t) = 0.5 * (1 - cos(2*pi*t/duration)),  t icinde [0, duration]
///   zarf(t) = 0                                  diger durumlarda
/// Her iki uçta hem değer hem türev tam sıfırdır (C1 süreklilik), dolayısıyla
/// sabit bir arka plana eklendiğinde ne değerde ne de türevinde ani atlama
/// olur. Tanım aralığı sınırlıdır: [0, _duration] dışındaki `_t` ya da
/// pozitif olmayan `_duration` tam 0.0 döndürür.
inline double GustEnvelope(double _t, double _duration)
{
  if (_duration <= 0.0 || _t < 0.0 || _t > _duration)
    return 0.0;
  return 0.5 * (1.0 - std::cos(2.0 * M_PI * _t / _duration));
}

/// \brief Mutlak benzetim zamanı `_simTimeSec` için tam hamle HIZ vektörünü
/// (m/s, dünya çerçevesi) hesaplar. `_gust.scheduled` false ise ya da zaman
/// hamlenin kendi penceresi dışındaysa tam olarak Zero döner.
inline gz::math::Vector3d EvaluateGust(
    const GustState &_gust, double _simTimeSec)
{
  if (!_gust.scheduled)
    return gz::math::Vector3d::Zero;
  const double t = _simTimeSec - _gust.startTimeSec;
  const double envelope = GustEnvelope(t, _gust.durationSec);
  return _gust.directionUnit * (_gust.amplitudeMps * envelope);
}

/// \brief Yayınlanan toplam rüzgâr = sabit bileşen + hamle; çapraz terimi
/// olmayan düz bir vektör toplamıdır (iki bileşen birbirinden bağımsız
/// hesaplanır).
inline gz::math::Vector3d ComposeWind(
    const gz::math::Vector3d &_steadyWorld, const gz::math::Vector3d &_gustWorld)
{
  return _steadyWorld + _gustWorld;
}

}  // namespace falcon_v2_wind

#endif  // FALCON_V2_WIND_MODEL_HH_

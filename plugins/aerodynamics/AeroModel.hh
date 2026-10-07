// FALCON V2 — aerodinamik kuvvet/moment modelinin matematik çekirdeği.
// Saf matematik; gz-sim (ECM/Entity/System) bağımlılığı yoktur, tek bağımlılık
// gz-math7'dir. Tüm katsayılar config/aero_v1_config.yaml'den gelir; burada
// hiçbir katsayı gömülü değildir. Tüm vektörler Gazebo FLU gövde çerçevesindedir.
#ifndef FALCON_V2_AERO_MODEL_HH_
#define FALCON_V2_AERO_MODEL_HH_

#include <algorithm>
#include <array>
#include <cmath>

#include <gz/math/Vector3.hh>

namespace falcon_v2_aero
{

/// \brief Kumanda yüzeyi arama tablolarının paylaşılan kırılma noktası sayısı.
/// XFLR5 geniş sapma taramasının kendi ızgarası tarafından sabitlenmiştir
/// (ayarlanabilir bir parametre değildir); üç yüzey için de aynı 15 noktalı
/// ızgara (derece): [-45,-35,-25,-15,-10,-5,-2,0,+2,+5,+10,+15,+25,+35,+45]
constexpr int kNumCtrlBreakpoints = 15;

/// \brief Yukarıdaki ızgarada delta=0 kırılma noktasının indeksi. Prepare()
/// içinde, kaynak dosyada ham (referansa göre farklandırılmamış) verilen
/// kanatçık/dümen CD/CL/Cm satırlarını taban çizgisine göre farklandırmak için
/// kullanılır.
constexpr int kCtrlZeroIndex = 7;

/// \brief Tek bir geniş sapma arama eğrisi için sabit boyutlu dizi tipi.
using CtrlLookupArray = std::array<double, kNumCtrlBreakpoints>;

/// \brief Modelin ihtiyaç duyduğu tüm sayısal parametreler; eklenti
/// Configure() aşamasında config/aero_v1_config.yaml'den yükler.
/// Aşağıdaki varsayılanlar gerçek bir koşu için güvenilir değildir: yükleyici
/// her alanı YAML'den yazmak zorundadır. Varsayılanlar yalnızca eksik bir
/// YAML anahtarının sessizce değil, görünür biçimde (ör. S=0 ile qbar*S=0,
/// yani NaN olmayan ama bariz yanlış sıfır kuvvet) hata vermesi içindir.
struct AeroConfig
{
  double S = 0.0;        // kanat alanı, m^2
  double b = 0.0;        // kanat açıklığı, m
  double c_ref = 0.0;    // aerodinamik referans veter, m (üreticinin ortalama veteri DEĞİL)

  double rho = 1.225;         // hava yoğunluğu, kg/m^3
  double vSafeFloor = 1.0e-3; // m/s, yalnızca *_hat paydaları için sayısal güvenlik tabanı

  // Cmde ve CLde SUPERSEDED_BY_LOOKUP'tır: asansör etkisi yalnızca aşağıdaki
  // ctrlElevDCm/ctrlElevDCL tablolarıyla uygulanır. Bu skalerler sadece
  // belgelenmiş küçük sinyal referansıdır, ComputeAero() tarafından okunmaz
  // (ikisini birlikte eklemek etkiyi iki kez sayardı).
  double CLa = 0.0, Cma = 0.0, CLq = 0.0, Cmq = 0.0, Cmde = 0.0, CLde = 0.0;
  double CL0 = 0.0, Cm0 = 0.0;

  // CYda/CYdr/Clda/Cnda/Cndr de SUPERSEDED_BY_LOOKUP'tır: kanatçık/dümen
  // CY/Cl/Cn etkisi yalnızca ctrlAile*/ctrlRudd* tablolarıyla uygulanır,
  // bu skalerler referans amaçlıdır ve ComputeAero() tarafından okunmaz.
  // TEK istisna Cldr'dir ve işlevsel olarak KULLANILIR: Prepare(), ctrlRuddCl
  // dizisini bu sabitin sınırlı doğrusal uzantısı olarak kurar
  // (ctrlRuddCl[i] = Cldr * ctrlBreakpointsRad[i]); geniş sapma
  // Cl(delta_r) tablosu işaret uyuşmazlığı nedeniyle kullanılmaz.
  double CYb = 0.0, CYp = 0.0, CYr = 0.0, CYda = 0.0, CYdr = 0.0;
  double Clb = 0.0, Clp = 0.0, Clr = 0.0, Clda = 0.0, Cldr = 0.0;
  double Cnb = 0.0, Cnp = 0.0, Cnr = 0.0, Cnda = 0.0, Cndr = 0.0;

  // Sürükleme poları (V1_CALIBRATED)
  double CD0 = 0.0, dragK = 0.0;

  // Yüksek hücum açısı yumuşak doyum sınırlayıcısı.
  double CLmax = 1.42;           // üretici performans hesabı girdisi
  double alphaTransition = 0.0;  // rad, DERIVED: XFLR5 güvenilir akım bandının orta noktası

  // Kumanda eklem açısı -> aerodinamik sapma işaret eşlemesi. Değerler
  // config/aero_v1_config.yaml'deki control_mapping bloğundan yüklenir;
  // buradaki sabitler yalnızca yükleme öncesi yer tutucudur.
  double elevatorSign = 1.0, aileronSign = 1.0, rudderSign = 1.0;

  // Geniş sapma kumanda yüzeyi arama tabloları. Kırılma noktaları üç yüzey
  // için paylaşılır ve burada RADYAN olarak tutulur (YAML'deki
  // breakpoints_deg değerleri yükleme anında dönüştürülür).
  // Tanım aralığı +/-45 derecede SINIRLIDIR (eyleyicinin mekanik aralığı):
  // InterpLinear() aralık dışı girdiyi en yakın uç değere kırpar, referans
  // verinin ötesine sessizce ekstrapolasyon YAPMAZ.
  CtrlLookupArray ctrlBreakpointsRad{};

  // Asansör: değerler kaynak dosyada zaten taban çizgisine göre
  // farklandırılmıştır (delta_e=0 satırı tanımı gereği tam sıfırdır).
  // "Tam arama katkısı" mimarisi:
  //   CL = SaturatedCL(alpha) + CLq*qHat + ctrlElevDCL(delta_e)
  //   cmStatic = Cm0 + Cma*alpha + ctrlElevDCm(delta_e)
  // ctrlElevDCm, eski `+ Cmde*deltaE` terimini birebir DEĞİŞTİRİR; ikisi
  // asla birlikte eklenmez. ctrlElevDCD sürükleme toplamına beslenir.
  CtrlLookupArray ctrlElevDCL{}, ctrlElevDCD{}, ctrlElevDCm{};

  // Kanatçık: Cl/Cn/CY kaynak taramasından gelen HAM değerlerdir (delta_a=0
  // taban çizgisi kaynak veride zaten ~0'dır; kalan çok küçük sayısal çözücü
  // artığı, kaynak veriye müdahale etmeme kuralı gereği sıfırlanmamıştır).
  // ctrlAileCDFull/CLFull/CmFull ise sabit alfadaki TAM değerlerdir; Prepare()
  // bunları delta_a=0 satırına (kCtrlZeroIndex) göre farklandırarak simetrik
  // ikincil düzeltmeleri (ctrlAileDCD/DCL/DCm) üretir.
  CtrlLookupArray ctrlAileCl{}, ctrlAileCn{}, ctrlAileCY{};
  CtrlLookupArray ctrlAileCDFull{}, ctrlAileCLFull{}, ctrlAileCmFull{};
  CtrlLookupArray ctrlAileDCD{}, ctrlAileDCL{}, ctrlAileDCm{};  // Prepare() içinde DERIVED

  // Dümen: CY/Cn HAM değerlerdir (delta_r=0 taban çizgisi ~0, kanatçıkla aynı
  // artık gürültü notu). ctrlRuddCl YAML'den yüklenmez; Prepare() içinde
  // Cldr'den türetilir (yukarıdaki Cldr notuna bakınız). ctrlRuddCDFull TAM
  // değer tablosudur ve Prepare() içinde ctrlRuddDCD'ye farklandırılır.
  CtrlLookupArray ctrlRuddCY{}, ctrlRuddCn{};
  CtrlLookupArray ctrlRuddCl{};      // Prepare() içinde Cldr'den DERIVED, YAML'den yüklenmez
  CtrlLookupArray ctrlRuddCDFull{};
  CtrlLookupArray ctrlRuddDCD{};     // Prepare() içinde DERIVED

  // Prepare() tarafından bir kez hesaplanan doyum sabitleri: formülün tek
  // kaynağı bu başlıktır, aritmetik config dosyasında tekrarlanmaz.
  double satHeadroomPos = 0.0, satKPos = 0.0;
  double satAneg = 0.0, satKNeg = 0.0;
  bool prepared = false;

  /// \brief YAML'den okunan tüm alanlar doldurulduktan sonra bir kez
  /// çağrılmalıdır: C1 sürekli yüksek hücum açısı doyum sabitlerini,
  /// farklandırılmış kanatçık/dümen tablolarını ve Cldr'den türetilen dümen
  /// yuvarlanma aramasını hesaplar. Birden fazla çağrı güvenlidir (idempotent).
  void Prepare()
  {
    const double clLinAtT = CL0 + CLa * alphaTransition;
    satHeadroomPos = CLmax - clLinAtT;
    satKPos = (satHeadroomPos > 1e-9) ? (CLa / satHeadroomPos) : 0.0;

    const double clLinAtNegT = CL0 + CLa * (-alphaTransition);
    satAneg = clLinAtNegT + CLmax;
    satKNeg = (satAneg > 1e-9) ? (CLa / satAneg) : 0.0;

    for (int i = 0; i < kNumCtrlBreakpoints; ++i)
    {
      // Kanatçık/dümen TAM değer satırları -> taban çizgisine göre
      // farklandırılmış ikincil düzeltmeler.
      ctrlAileDCD[i] = ctrlAileCDFull[i] - ctrlAileCDFull[kCtrlZeroIndex];
      ctrlAileDCL[i] = ctrlAileCLFull[i] - ctrlAileCLFull[kCtrlZeroIndex];
      ctrlAileDCm[i] = ctrlAileCmFull[i] - ctrlAileCmFull[kCtrlZeroIndex];
      ctrlRuddDCD[i] = ctrlRuddCDFull[i] - ctrlRuddCDFull[kCtrlZeroIndex];

      // Dümen yuvarlanma araması, geniş sapma tablosunun işareti tartışmalı
      // olduğu için Cldr sabitinin sınırlı DOĞRUSAL UZANTISIDIR.
      ctrlRuddCl[i] = Cldr * ctrlBreakpointsRad[i];
    }

    prepared = true;
  }
};

/// \brief Bir zaman adımına ait aerodinamik durum girdisi; gövde çerçevesine
/// çözülmüş ve kumanda işaret eşlemesinden geçirilmiş olarak gelir (eklem
/// açısı -> delta dönüşümünü AerodynamicsSystem.cc yapar, bu başlık eklemleri
/// bilmez). deltaA/deltaE/deltaR önceden kırpılmaz; tek sınır arama
/// tablosunun +/-45 derecelik tanım aralığıdır.
struct AeroState
{
  double u = 0.0, v = 0.0, w = 0.0;       // gövde çerçevesinde bağıl rüzgâr hızı, m/s (Vrel = Vbody - Vwind)
  double p = 0.0, q = 0.0, r = 0.0;       // gövde çerçevesinde açısal hız, rad/s (X/Y/Z eksenleri)
  double deltaA = 0.0, deltaE = 0.0, deltaR = 0.0; // rad, işareti eşlenmiş
};

/// \brief Bir zaman adımına ait tam tanı çıktısı ile kuvvet/moment sonucu.
struct AeroOutput
{
  double V = 0.0, alpha = 0.0, beta = 0.0, qbar = 0.0;
  double CL = 0.0, CD = 0.0, CY = 0.0, Cl = 0.0, Cm = 0.0, Cn = 0.0;
  gz::math::Vector3d forceBody = gz::math::Vector3d::Zero;
  gz::math::Vector3d momentBody = gz::math::Vector3d::Zero;
};

// -----------------------------------------------------------------------
// FLU'da eksen dönüşlerinin fiziksel anlamı (dönüş matrisi kurularak
// doğrulanmıştır; FRD ders kitabı kısayolundan kopyalanmamıştır). Rx/Ry/Rz
// matrislerinin cebirsel biçimi her çerçevede aynıdır; yalnızca eksenlerin
// FİZİKSEL anlamı (ileri/sol/yukarı ile ileri/sağ/aşağı) değişir:
//   +X (yuvarlanma) dönüşü -> SOL kanat ucu YUKARI kalkar. Yuvarlanma
//     ekseninin elleri FRD ile FLU arasında değişmez (Y ve Z birlikte, yani
//     çift sayıda anlam değiştirir).
//   +Y (yunuslama) dönüşü -> BURUN AŞAĞI. Bu, geleneksel "q>0 / pozitif Cm =
//     burun yukarı" kısayolunun TERSİDİR; o kısayol yalnızca FRD'de (Z aşağı)
//     geçerlidir, FLU'da tek bir eksen anlam değiştirdiği için işaret döner.
//   +Z (sapma) dönüşü -> BURUN SOLA. FRD'deki "r>0 = burun sağa" ile yine
//     terstir (bu eksen çifti için yalnızca Y anlam değiştirir).
// Dosyadaki her Mx/My/Mz işaret iddiası bu tabloya göre yorumlanır.
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// Hücum açısı (alpha) — FLU türetimi
// -----------------------------------------------------------------------
// FRD ders kitabı biçimi alpha = atan2(w, u) olup Z aşağı varsayar; yunuslama
// dönüş yönü FRD ile FLU arasında döndüğü için doğrudan kopyalanamaz. Yeniden
// türetim: burun yukarı theta açısı, +Y ekseni etrafında -theta dönüşüne
// karşılık gelir; uçuş yolu dünya çerçevesinde değişmezken gövde çerçevesindeki
// bağıl rüzgâr u=V*cos(theta), w=-V*sin(theta) olur. Buradan alpha = atan2(-w, u)
// çıkar; w üzerindeki EKSİ işareti FLU'ya özgü farktır.
// Fiziksel anlam: alpha > 0 <=> burun bağıl rüzgâra göre yukarı. Bu kısımda
// serbest işaret seçimi yoktur ("burun yukarı" tek bir fiziksel anlama gelir).
// Bu işaret XFLR5'in kendi iç konvansiyonuna karşı doğrulanmamıştır.
inline double AngleOfAttack(double u, double /*v*/, double w)
{
  return std::atan2(-w, u);
}

// -----------------------------------------------------------------------
// Yan kayma (beta) — FLU türetimi
// -----------------------------------------------------------------------
// Alpha'nın aksine beta'nın işareti gerçek bir KONVANSİYON SEÇİMİDİR (hem
// "pozitif = rüzgâr sağdan" hem "soldan" kullanımı literatürde vardır). İki
// aday, VERİLEN (değiştirilmemiş) CYb/Cnb işaretleri ve bunların belirtilen
// fiziksel anlamına karşı sınandı; burun sola psi kadar kaydığında gövde
// çerçevesinde v = -V*sin(psi) olur (yani rüzgâr sağdan):
//   Aday A: beta = atan2(-v, hypot(u,w))  -> Cnb*beta > 0, +Mz, burun SOLA:
//     bozulmayı büyütür, KARARSIZLAŞTIRICI.
//   Aday B: beta = atan2(+v, hypot(u,w))  -> Cnb*beta < 0, -Mz, burun SAĞA:
//     DÜZELTİCİ; Cnb=+0.03554'ün "yönsel statik kararlı" anlamıyla uyumlu.
// Ayrıca Aday B'de CYb, uçağı sağdan gelen yan rüzgârdan UZAĞA iten bir yan
// kuvvet verir (fiziksel olarak doğru). Bu nedenle Aday B BENİMSENMİŞTİR:
//   beta = atan2(v, hypot(u,w))   [fiziksel anlam: pozitif beta = rüzgâr SOLDAN]
// Bu, sabit tutulan Cnb/CYb işaretlerine iki bağımsız fiziksel kontrolün
// uydurulmasından gelen belgelenmiş bir ÇIKARIMDIR; katsayı ayarı değildir
// (Cnb/CYb'ye hiç dokunulmaz) ve nihai doğrulama iddiası da değildir. Clb
// (dihedral etkisi) üçüncü bir kontrol olarak denendi ancak kanıt olarak
// KULLANILMADI: dihedral-yan kayma yuvarlanma eşleşmesinin yönü burada ilk
// ilkelerden güvenle türetilemedi.
inline double Sideslip(double u, double v, double w)
{
  return std::atan2(v, std::hypot(u, w));
}

// -----------------------------------------------------------------------
// Rüzgâr ekseni -> gövde ekseni kuvvet dönüşümü — FLU türetimi
// (naif Fx=-D, Fz=L kısayolu değil)
// -----------------------------------------------------------------------
// Yukarıdaki alpha/beta formüllerinin ima ettiği AYNI dönüşten kurulmuştur:
// R(alpha,beta) = Ry(alpha) * Rz(beta). Böylece kuvvet dönüşümü kullanılan
// açı formülleriyle birebir tutarlıdır. Doğrulama: R(alpha,beta) @ (1,0,0)
// = Vrel/V ve R(0,0) = birim matris (sıfır alfa/beta'da rüzgâr eksenleri
// gövde eksenlerine eşittir).
// Rüzgâr eksenlerinde kuvvet (Xw, Yw, Zw) boyunca (-D, Y, L)'dir; Xw bağıl
// rüzgâr birim vektörü yönündedir (sürükleme tanımı gereği bağıl rüzgâra tam
// ters), Zw gövde +Z'ye (yukarı) ve Yw gövde +Y'ye (sol) indirgenir.
// Kapalı biçim:
//   Fx = -D*cos(a)*cos(b) - Y*cos(a)*sin(b) + L*sin(a)
//   Fy = -D*sin(b) + Y*cos(b)
//   Fz =  D*sin(a)*cos(b) + Y*sin(a)*sin(b) + L*cos(a)
inline gz::math::Vector3d WindToBodyForce(
    double lift, double drag, double sideForce, double alpha, double beta)
{
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  const double cb = std::cos(beta), sb = std::sin(beta);

  const double fx = -drag * ca * cb - sideForce * ca * sb + lift * sa;
  const double fy = -drag * sb + sideForce * cb;
  const double fz = drag * sa * cb + sideForce * sa * sb + lift * ca;

  return gz::math::Vector3d(fx, fy, fz);
}

// -----------------------------------------------------------------------
// Cma / My yunuslama ekseni işaret konvansiyonu
// -----------------------------------------------------------------------
// Hız/sönüm terimleri (Cmq*qHat, Clp*pHat, Cnr*rHat) matematiksel olarak
// KENDİNE GÖNDERMELİDİR: negatif bir katsayı ile bir hızın çarpımı, aynı eksen
// etrafında daima o hıza karşı koyan bir moment üretir; bu her çerçevede
// doğrudur, konvansiyon riski yoktur.
// STATİK terimler (Cm0, Cma*alpha ve kumanda sapması arama düzeltmeleri) ise
// kendine göndermeli değildir: her biri bağımsız tanımlı bir AÇIYI +Y
// etrafındaki bir momente bağlar. "+Y dönüşü -> BURUN AŞAĞI" bulgusu
// nedeniyle My = qbar*S*c_ref*Cm formülünün statik kısma birebir uygulanması,
// burun yukarı bozulmada (alpha>0, Cma<0) KARARSIZLAŞTIRICI bir moment verir;
// bu, beklenen düzeltici davranışın tersidir.
// Bu nedenle FLU +Y tork eksenine eşlemede YALNIZCA statik grup negatiflenir;
// hız grubu (Cmq*qHat) zaten doğrudur ve çevrilmemelidir (q hem FLU'dan ham
// okunduğu hem de My çıktısında aynı aynalamanın bulunduğu için etki iki kez
// sadeleşir). Kumanda sapması moment düzeltmeleri statik gruba dahildir:
// her biri bir hız değil geometrik açı girdisidir, dolayısıyla Cma*alpha ile
// aynı eksen elleri duyarlılığına sahiptir. Cm0 ile Cma daima tek bir grup
// olarak çevrilir: Cm0, denge noktasında Cm0+Cma*alpha_trim=0 olacak şekilde
// türetildiği için yalnız birini çevirmek sıfır moment denge koşulunu bozar.
// out.Cm kaynak tarama tablolarıyla karşılaştırılabilsin diye XFLR5'in kendi
// (çevrilmemiş) konvansiyonunda raporlanır; düzeltme yalnızca aşağıdaki my
// hesabına uygulanır.
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// Yüksek hücum açısında yumuşak doyum
// -----------------------------------------------------------------------
// |alpha| <= alphaTransition bölgesinde CL tam olarak doğrusal modeldir
// (CL0 + CLa*alpha), yani doğrulanmış bölge birebir yeniden üretilir. Ötesinde
// CL, C1 sürekli (geçiş noktasında hem değer hem eğim eşleşir) bir üstel ile
// +/-CLmax'a asimptotik yaklaşır ve onu asla aşmaz; sönüm oranı k'nın kendisi
// CLa/CL0/CLmax/alphaTransition'dan hesaplanır, ayrıca seçilmiş serbest bir
// sabit değildir.
// Pozitif taraf gerçek veriye dayanır (CLmax=1.42 üretici değeri ve XFLR5
// güvenilir akım sınırı). Negatif taraf ASSUMPTION etiketlidir: tam uçak için
// negatif alfa stall verisi yoktur; simetrik sınır yalnızca sınırsız negatif
// taşıma/sürükleme büyümesini engellemek için vardır, doğrulanmış fizik
// değildir.
inline double SaturatedCL(const AeroConfig &cfg, double alpha)
{
  const double clLinear = cfg.CL0 + cfg.CLa * alpha;
  if (alpha > cfg.alphaTransition)
  {
    return cfg.CLmax -
           cfg.satHeadroomPos * std::exp(-cfg.satKPos * (alpha - cfg.alphaTransition));
  }
  else if (alpha < -cfg.alphaTransition)
  {
    return -cfg.CLmax +
           cfg.satAneg * std::exp(cfg.satKNeg * (alpha + cfg.alphaTransition));
  }
  return clLinear;
}

/// \brief Sabit ve kesin artan kırılma noktaları üzerinde parçalı DOĞRUSAL
/// interpolasyon. TANIM ARALIĞI SINIRLIDIR: ilk/son kırılma noktasında veya
/// ötesinde ilk/son DEĞER aynen döner (kırpma), asla ekstrapolasyon yapılmaz.
/// Geçerli aralık tam olarak [breakpoints.front(), breakpoints.back()], yani
/// bu dosyadaki her eğri için +/-45 derecedir.
inline double InterpLinear(const CtrlLookupArray &breakpoints,
                            const CtrlLookupArray &values, double x)
{
  const double lo = breakpoints.front();
  const double hi = breakpoints.back();
  if (x <= lo)
    return values.front();
  if (x >= hi)
    return values.back();

  // Kırılma noktaları kesin artan olduğundan std::upper_bound, x'ten kesin
  // büyük ilk noktayı, yani kuşatan aralığın üst ucunu verir.
  const auto it = std::upper_bound(breakpoints.begin(), breakpoints.end(), x);
  const std::size_t i1 = static_cast<std::size_t>(it - breakpoints.begin());
  const std::size_t i0 = i1 - 1;

  const double t = (x - breakpoints[i0]) / (breakpoints[i1] - breakpoints[i0]);
  return values[i0] + t * (values[i1] - values[i0]);
}

/// \brief Bir zaman adımı için tam aerodinamik model değerlendirmesi. Saf
/// fonksiyon: yan etkisi ve Gazebo bağımlılığı yoktur.
inline AeroOutput ComputeAero(const AeroConfig &cfg, const AeroState &st)
{
  AeroOutput out;

  const double u = st.u, v = st.v, w = st.w;
  const double vSq = u * u + v * v + w * w;
  out.V = std::sqrt(vSq);
  out.qbar = 0.5 * cfg.rho * vSq; // gerçek V=0'da tam sıfır: burada korumaya gerek yok

  out.alpha = AngleOfAttack(u, v, w);
  out.beta = Sideslip(u, v, w);

  // V tabanı YALNIZCA *_hat hız normalizasyon paydaları (2V) içindir. qbar
  // yukarıda gerçek (kırpılmamış) V'yi kullandığı için qbar*(*_hat) çarpımı
  // V -> 0 giderken doğru biçimde sıfıra gider; *_hat o anda büyük ama sonlu
  // kalır, asla Inf/NaN olmaz.
  const double vSafe = std::max(out.V, cfg.vSafeFloor);
  const double pHat = st.p * cfg.b / (2.0 * vSafe);
  const double qHat = st.q * cfg.c_ref / (2.0 * vSafe);
  const double rHat = st.r * cfg.b / (2.0 * vSafe);

  // Kumanda yüzeyi etkisi: geniş sapma parçalı doğrusal arama. st.deltaA/E/R
  // zaten işaret eşlenmiştir ve doğrudan kullanılır; tek sınır InterpLinear()
  // tanım aralığıdır (+/-45 derece, eyleyicinin mekanik aralığı).
  // Her arama sonucu, karşılık gelen eski doğrusal terimin TAM YERİNE geçer,
  // onun üstüne eklenmez (çift sayım olmaz).
  const double dCYa = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileCY, st.deltaA);
  const double dCla = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileCl, st.deltaA);
  const double dCna = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileCn, st.deltaA);
  const double dCLaCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileDCL, st.deltaA);
  const double dCmaCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileDCm, st.deltaA);
  const double dCDaCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlAileDCD, st.deltaA);

  const double dCYr = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlRuddCY, st.deltaR);
  const double dClr = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlRuddCl, st.deltaR);
  const double dCnr = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlRuddCn, st.deltaR);
  const double dCDrCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlRuddDCD, st.deltaR);

  const double dCLeCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlElevDCL, st.deltaE);
  const double dCmeCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlElevDCm, st.deltaE);
  const double dCDeCtrl = InterpLinear(cfg.ctrlBreakpointsRad, cfg.ctrlElevDCD, st.deltaE);

  out.CY = cfg.CYb * out.beta + cfg.CYp * pHat + cfg.CYr * rHat + dCYa + dCYr;
  out.Cl = cfg.Clb * out.beta + cfg.Clp * pHat + cfg.Clr * rHat + dCla + dClr;
  out.Cn = cfg.Cnb * out.beta + cfg.Cnp * pHat + cfg.Cnr * rHat + dCna + dCnr;

  // Cm, STATİK/açı kaynaklı gruba (Cm0, Cma*alpha ve asansör + kanatçık
  // yunuslama momenti düzeltmeleri) ve HIZ grubuna (Cmq*qHat) ayrılır.
  // Aşağıda My'ye eşlenirken yalnızca STATİK grup FLU işaret düzeltmesi alır;
  // hız grubu çevrilmez. out.Cm ise tanı amacıyla XFLR5'in kendi
  // (çevrilmemiş) konvansiyonunda raporlanır.
  const double cmStatic = cfg.Cm0 + cfg.Cma * out.alpha + dCmeCtrl + dCmaCtrl;
  const double cmRate = cfg.Cmq * qHat;
  out.Cm = cmStatic + cmRate;

  // CL: yumuşak doyum yalnızca alfa kaynaklı STATİK terime uygulanır; hız
  // terimi CLq*qHat ile kumanda arama katkıları doyumsuz olarak eklenir.
  out.CL = SaturatedCL(cfg, out.alpha) + cfg.CLq * qHat + dCLeCtrl + dCLaCtrl;

  // CD: V1 kalibre edilmiş tam uçak poları (CD0 + k*CL^2) artı üç yüzeye ait
  // geniş sapma sürükleme düzeltmesi, TOPLAMSAL olarak. Bu, eşzamanlı çok
  // yüzeyli sapma için belgelenmiş bir V1 BASİTLEŞTİRMESİDİR: her dCD_x
  // yalnızca o yüzey saptırılmışken ölçülmüştür ve birlikte toplanmaları
  // gerçek bir birleşik sapma taramasına karşı doğrulanmamıştır
  // (V1_ADDITIVE_MULTI_SURFACE_DRAG_APPROXIMATION). Uydurulmuş etkileşim
  // fiziği değil, doğrusal süperpozisyon varsayımıdır.
  //
  // Taban CD0'dır: bazı dCD_x değerleri küçük sapmalarda hafifçe NEGATİFTİR.
  // Sıfır yerine CD0'da sınırlamanın nedeni, CD0'ın uçağın kumanda sapmasından
  // bağımsız olarak fiziksel olarak her zaman var olan parazit sürükleme
  // sabiti olmasıdır; tam sıfırda sınırlamak sürüklemesi olmayan bir gövde
  // anlamına gelirdi.
  const double cdRaw = cfg.CD0 + cfg.dragK * out.CL * out.CL +
                        dCDeCtrl + dCDaCtrl + dCDrCtrl;
  out.CD = std::max(cdRaw, cfg.CD0);

  const double lift = out.qbar * cfg.S * out.CL;
  const double drag = out.qbar * cfg.S * out.CD;
  const double sideForce = out.qbar * cfg.S * out.CY;

  out.forceBody = WindToBodyForce(lift, drag, sideForce, out.alpha, out.beta);

  // Momentler: Mx (yuvarlanma) ve Mz (sapma) verilen Cl/Cn'yi doğrudan
  // kullanır, eksen elleri düzeltmesi gerekmez. Yuvarlanma ekseninin elleri
  // FRD ile FLU arasında değişmez; sapma ekseninin elleri değişir ama bu
  // zaten Sideslip() konvansiyon seçimiyle soğurulmuştur (Aday B tam olarak
  // verilen Cnb/CYb işaretlerinin DEĞİŞTİRİLMEMİŞ Mz=qbar*S*b*Cn formülünden
  // doğru davranması için seçilmiştir). My ise yukarıda açıklanan düzeltmeyi
  // uygular: XFLR5 konvansiyonuna göre yalnızca cmStatic negatiflenir,
  // cmRate negatiflenmez.
  const double mx = out.qbar * cfg.S * cfg.b * out.Cl;
  const double my = out.qbar * cfg.S * cfg.c_ref * (-cmStatic + cmRate);
  const double mz = out.qbar * cfg.S * cfg.b * out.Cn;
  out.momentBody = gz::math::Vector3d(mx, my, mz);

  return out;
}

}  // namespace falcon_v2_aero

#endif  // FALCON_V2_AERO_MODEL_HH_

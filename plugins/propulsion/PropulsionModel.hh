// FALCON V2 — motor/ESC/pervane çekirdek matematiği.
// Saf matematik; Gazebo (ECM/Entity/System) bağımlılığı yoktur.
// Zincir: gaz -> elektrik -> motor torku -> rotor dinamiği -> RPM -> J ->
// APC Ct/Cp -> itki + tork. Katsayılar config/propulsion_v1_config.yaml'den gelir.
#ifndef FALCON_V2_PROPULSION_MODEL_HH_
#define FALCON_V2_PROPULSION_MODEL_HH_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace falcon_v2_propulsion
{

// Yapılandırma yapıları: PropulsionSystem.cc, Configure() adımında YAML'den
// ve ayrıştırılmış APC CSV'sinden doldurur. Aşağıdaki varsayılanlar gerçek bir
// koşu için geçerli değildir; eksik bir anahtar sessizce makul bir değer
// üretmesin diye sıfırdır.

/// \brief Motor + ESC elektrik parametreleri.
struct MotorConfig
{
  double kvRpmPerVolt = 0.0;     // KV, rpm/V
  double resistanceOhm = 0.0;    // R, motor iç direnci (ohm)
  double noLoadCurrentA = 0.0;   // I0, boşta akım (A)
  double noLoadCurrentRefVoltage = 10.0;  // I0'ın karakterize edildiği gerilim (V)
  double currentLimitA = 0.0;    // V1 anlık akım kırpma sınırı (A)
  double powerLimitW = 0.0;      // Üretici güç sınırı (W); V1'de kırpma uygulanmaz
};

/// \brief Batarya. V1'de sabit gerilim; SOC/gerilim düşümü modellenmez.
struct BatteryConfig
{
  double vNominal = 0.0;  // İşletme gerilimi (V)
};

/// \brief Pervanenin fiziksel parametreleri.
struct PropellerConfig
{
  double diameterM = 0.0;  // D, pervane çapı (m) - üretici nominal değeri, STL ölçüsü değil
};

/// \brief Fizik / sayısal parametreler.
struct PhysicsConfig
{
  double rho = 1.225;              // Hava yoğunluğu (kg/m^3)
  double rotorInertiaKgM2 = 0.0;   // I_rotor (kg*m^2), V1_PROVISIONAL
  double nSafeFloorRevS = 1.0e-3;  // TEMPORARY: yalnızca J paydası için sayısal taban (devir/s)
  double rpmCap = 0.0;             // |RPM| üst işletme sınırı; omega entegratörünü doyurur
};

/// \brief Ayrıştırılmış APC tablosunun bir RPM dilimi: sabit RPM'de J'ye bağlı
/// Ct/Cp. J dizisi kesin artandır (yükleme anında LoadApcTableCsv() doğrular).
struct ApcRpmSlice
{
  double rpm = 0.0;
  std::vector<double> J;
  std::vector<double> Ct;
  std::vector<double> Cp;
};

/// \brief Ayrıştırılmış APC 13x6.5E performans tablosunun tamamı; dilimler
/// rpm'e göre artan sırada. Kaynak: apc_13x65e_parsed.csv.
struct ApcTable
{
  std::vector<ApcRpmSlice> slices;

  double MinRpm() const { return slices.empty() ? 0.0 : slices.front().rpm; }
  double MaxRpm() const { return slices.empty() ? 0.0 : slices.back().rpm; }
};

/// \brief Ayrıştırılmış APC CSV'sini (rpm,V_mph,J,Ct,Cp,power_W,torque_Nm,
/// thrust_N) bir ApcTable'a yükler: satırları rpm'e göre gruplar, dilim içinde
/// dosya sırasını korur, dilimleri rpm'e göre artan sıralar. Bir dilimin J
/// dizisi kesin artan değilse veya G/Ç-ayrıştırma hatası olursa false döner
/// (errorOut verilmişse mesaj yazar); aşağıdaki interpolasyon dilim başına
/// monotonik J varsayar.
inline bool LoadApcTableCsv(const std::string &path, ApcTable &table,
                             std::string *errorOut = nullptr)
{
  table.slices.clear();
  std::ifstream f(path);
  if (!f.is_open())
  {
    if (errorOut) *errorOut = "could not open '" + path + "'";
    return false;
  }

  std::string line;
  if (!std::getline(f, line))
  {
    if (errorOut) *errorOut = "empty file '" + path + "'";
    return false;
  }
  // 0. satır kolon başlığıdır, atlanır.

  std::vector<double> sliceOrder;
  struct RawSlice { std::vector<double> J, Ct, Cp; };
  std::vector<RawSlice> rawSlices;
  auto findOrAdd = [&](double rpm) -> RawSlice &
  {
    for (std::size_t i = 0; i < sliceOrder.size(); ++i)
    {
      if (sliceOrder[i] == rpm) return rawSlices[i];
    }
    sliceOrder.push_back(rpm);
    rawSlices.emplace_back();
    return rawSlices.back();
  };

  std::size_t lineNo = 1;
  while (std::getline(f, line))
  {
    ++lineNo;
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ','))
    {
      try { vals.push_back(std::stod(tok)); }
      catch (const std::exception &)
      {
        if (errorOut) *errorOut = "parse error at CSV line " + std::to_string(lineNo);
        return false;
      }
    }
    if (vals.size() != 8)
    {
      if (errorOut) *errorOut = "malformed CSV row (expected 8 fields) at line " + std::to_string(lineNo);
      return false;
    }
    const double rpm = vals[0];
    // Kolon sırası: vals[2]=J, vals[3]=Ct, vals[4]=Cp. Tabloya yalnızca
    // Ct(J)/Cp(J) alınır; kalan kolonlar CSV'de çapraz kontrol için durur.
    RawSlice &rs = findOrAdd(rpm);
    rs.J.push_back(vals[2]);
    rs.Ct.push_back(vals[3]);
    rs.Cp.push_back(vals[4]);
  }

  for (std::size_t i = 0; i < sliceOrder.size(); ++i)
  {
    const RawSlice &rs = rawSlices[i];
    for (std::size_t j = 1; j < rs.J.size(); ++j)
    {
      if (!(rs.J[j] > rs.J[j - 1]))
      {
        if (errorOut)
        {
          *errorOut = "non-monotonic J within RPM slice " +
                      std::to_string(sliceOrder[i]) + " at index " + std::to_string(j);
        }
        return false;
      }
    }
    ApcRpmSlice slice;
    slice.rpm = sliceOrder[i];
    slice.J = rs.J;
    slice.Ct = rs.Ct;
    slice.Cp = rs.Cp;
    table.slices.push_back(slice);
  }

  std::sort(table.slices.begin(), table.slices.end(),
            [](const ApcRpmSlice &a, const ApcRpmSlice &b) { return a.rpm < b.rpm; });

  if (table.slices.empty())
  {
    if (errorOut) *errorOut = "no data rows parsed from '" + path + "'";
    return false;
  }
  return true;
}

// --- ESC/motor elektrik modeli ---

/// \brief Motor tork sabiti Kt = 60/(2*pi*KV); KV rpm/V, Kt N*m/A. SI
/// biriminde ters-EMK sabiti Ke, V/(rad/s) olarak Kt ile sayısal olarak aynıdır.
inline double MotorKt(double kvRpmPerVolt)
{
  if (kvRpmPerVolt <= 0.0) return 0.0;
  return 60.0 / (2.0 * M_PI * kvRpmPerVolt);
}

/// \brief Verilen gaz / batarya gerilimi / anlık omega çalışma noktasında tek
/// bir motor-elektrik değerlendirmesinin sonucu.
struct MotorElectricalResult
{
  double current_A = 0.0;      // I_used, kırpmalardan sonra (A)
  double torque_Nm = 0.0;      // Q_motor = Kt*(I_used - I0) (N*m)
  bool negativeCurrentClamped = false;  // ham akım < 0 idi ve 0'a kırpıldı
  bool currentLimited = false;          // I_used, current_limit_A değerinde sınırlandı
};

/// \brief ESC + motor elektrik modeli:
///   V_ESC   = gaz * V_batarya
///   I       = (V_ESC - Ke*omega) / R
///   Q_motor = Kt * (I - I0)
/// V1 politikaları (gerekçeleri propulsion_v1_config.yaml "electrical_policy"
/// bloğunda): ham akım negatif çıkarsa 0'a kırpılır (rejeneratif ESC
/// modellenmez), I0 sabit alınır (tek referans gerilimde karakterize edilmiş;
/// gerilime bağlı I0 eğrisi DATA_REQUIRED) ve akım current_limit_A değerinde
/// anlık olarak sınırlanır (süreye bağlı ısıl model yoktur). Gaz girişi
/// burada da [0,1] aralığına kırpılır.
inline MotorElectricalResult MotorElectrical(
    const MotorConfig &cfg, double throttle01, double vBattery, double omega)
{
  MotorElectricalResult out;
  const double throttle = std::clamp(throttle01, 0.0, 1.0);
  const double kt = MotorKt(cfg.kvRpmPerVolt);
  const double ke = kt;  // SI'de Ke ile Kt sayısal olarak aynıdır

  const double vEsc = throttle * vBattery;
  double iRaw = 0.0;
  if (cfg.resistanceOhm > 0.0)
  {
    iRaw = (vEsc - ke * omega) / cfg.resistanceOhm;
  }

  double iUsed = iRaw;
  if (iUsed < 0.0)
  {
    iUsed = 0.0;
    out.negativeCurrentClamped = true;
  }
  if (cfg.currentLimitA > 0.0 && iUsed > cfg.currentLimitA)
  {
    iUsed = cfg.currentLimitA;
    out.currentLimited = true;
  }

  out.current_A = iUsed;
  out.torque_Nm = kt * (iUsed - cfg.noLoadCurrentA);
  return out;
}

// --- RPM güvenlik sınırı ---

struct RpmCapResult
{
  double omega = 0.0;
  bool capActive = false;
};

/// \brief omega'yı (rad/s, işaretli) |RPM| <= rpmCap olacak şekilde doyurur.
/// Bu V1 işletme sınırıdır; teorik boşta-RPM değerinden ve APC tablosunun
/// kendi arama-kırpma aralığından bağımsızdır.
inline RpmCapResult ApplyRpmCap(double omega, double rpmCap)
{
  RpmCapResult out;
  out.omega = omega;
  if (rpmCap <= 0.0) return out;
  const double omegaCap = rpmCap * 2.0 * M_PI / 60.0;
  if (out.omega > omegaCap)
  {
    out.omega = omegaCap;
    out.capActive = true;
  }
  else if (out.omega < -omegaCap)
  {
    out.omega = -omegaCap;
    out.capActive = true;
  }
  return out;
}

// --- Rotor açısal dinamiği (ODE adımı) ---

struct RotorStepResult
{
  double omega = 0.0;   // rad/s, bu adımdan sonraki yeni durum
  double rpm = 0.0;      // türetilmiş teşhis değeri
  bool rpmCapActive = false;
};

/// \brief Şu denklemin tek bir açık Euler entegrasyon adımı:
///   I_rotor * domega/dt = Q_motor - Q_propSigned
/// qPropSigned, omegaPrev'e KARŞI koyacak şekilde zaten işaretli gelmelidir
/// (bkz. PropAeroLoad); bu fonksiyon işaret çıkarımı yapmaz. RPM hiçbir zaman
/// anlık atanmaz: bu, her PreUpdate tikinde gerçek dt ile çağrılması gereken
/// durum entegrasyonudur.
inline RotorStepResult IntegrateRotorStep(
    double omegaPrev, double qMotor, double qPropSigned, double iRotor,
    double dt, double rpmCap)
{
  RotorStepResult out;
  double omegaNew = omegaPrev;
  if (iRotor > 0.0)
  {
    omegaNew = omegaPrev + dt * (qMotor - qPropSigned) / iRotor;
  }
  const RpmCapResult capped = ApplyRpmCap(omegaNew, rpmCap);
  out.omega = capped.omega;
  out.rpmCapActive = capped.capActive;
  out.rpm = out.omega * 60.0 / (2.0 * M_PI);
  return out;
}

// --- İlerleme oranı (advance ratio) ---

/// \brief J = V_axial / (n*D). V_axial, gövde (FLU) çerçevesindeki itki
/// ekseni (+X) üzerine izdüşürülmüş BAĞIL hava hızıdır (rüzgâr çıkarılmış
/// hâli); n = |omega|/(2*pi) devir/s olup yapısı gereği daima >= 0. nSafeFloor
/// yalnızca n=0'da tam sıfıra bölmeyi engeller; T ve P_prop kendi n^2/n^3
/// çarpanları sayesinde n->0 giderken zaten 0'a gider.
inline double AdvanceRatio(double vAxial, double n, double diameterM, double nSafeFloor)
{
  const double nSafe = std::max(n, nSafeFloor);
  if (diameterM <= 0.0) return 0.0;
  return vAxial / (nSafe * diameterM);
}

// --- Ayrıştırılmış APC tablosu üzerinde 2 boyutlu Ct/Cp interpolasyonu ---

struct CtCpResult
{
  double Ct = 0.0;
  double Cp = 0.0;
  bool interpClamped = false;  // RPM ve/veya J tablonun aralığına kırpıldıysa true
};

namespace detail
{
/// \brief Ct/Cp'yi TEK bir RPM diliminin kendi J dizisi içinde doğrusal
/// interpolasyonla bulur. J önce o dilimin kendi [Jmin,Jmax] aralığına
/// kırpılır: dilimlerin J aralıkları farklıdır, ortak ızgara varsayılmaz.
inline void InterpWithinSlice(const ApcRpmSlice &slice, double J,
                               double &ctOut, double &cpOut, bool &jClamped)
{
  jClamped = false;
  if (slice.J.empty())
  {
    ctOut = 0.0;
    cpOut = 0.0;
    return;
  }
  if (slice.J.size() == 1)
  {
    ctOut = slice.Ct[0];
    cpOut = slice.Cp[0];
    return;
  }

  double jq = J;
  if (jq < slice.J.front())
  {
    jq = slice.J.front();
    jClamped = true;
  }
  else if (jq > slice.J.back())
  {
    jq = slice.J.back();
    jClamped = true;
  }

  // jq'yu kuşatan [lo,hi] çiftini bul.
  std::size_t hi = 0;
  while (hi < slice.J.size() && slice.J[hi] < jq) ++hi;
  if (hi == 0)
  {
    ctOut = slice.Ct[0];
    cpOut = slice.Cp[0];
    return;
  }
  if (hi >= slice.J.size())
  {
    ctOut = slice.Ct.back();
    cpOut = slice.Cp.back();
    return;
  }
  const std::size_t lo = hi - 1;
  const double jLo = slice.J[lo], jHi = slice.J[hi];
  const double span = jHi - jLo;
  const double f = (span > 0.0) ? (jq - jLo) / span : 0.0;
  ctOut = slice.Ct[lo] + f * (slice.Ct[hi] - slice.Ct[lo]);
  cpOut = slice.Cp[lo] + f * (slice.Cp[hi] - slice.Cp[lo]);
}
}  // namespace detail

/// \brief İki aşamalı 2B interpolasyon: (1) rpm'i kuşatan iki dilim bulunur ve
/// her birinde Ct/Cp o dilimin kendi J dizisiyle hesaplanır; (2) iki sonuç
/// rpm konumuna göre doğrusal harmanlanır. Aralık dışı rpm, arama amacıyla
/// tablonun kendi [minRpm,maxRpm] aralığına kırpılır (interpClamped teşhis
/// olarak dışa verilir); bu, işletme RPM sınırından ayrı bir güvenlik
/// durumudur ve onunla çakışmak zorunda değildir.
inline CtCpResult LookupCtCp(const ApcTable &table, double rpmQuery, double J)
{
  CtCpResult out;
  if (table.slices.empty()) return out;

  double rpm = rpmQuery;
  if (rpm < table.MinRpm())
  {
    rpm = table.MinRpm();
    out.interpClamped = true;
  }
  else if (rpm > table.MaxRpm())
  {
    rpm = table.MaxRpm();
    out.interpClamped = true;
  }

  // rpm'i kuşatan dilim indekslerini [lo, hi] bul.
  std::size_t hi = 0;
  while (hi < table.slices.size() && table.slices[hi].rpm < rpm) ++hi;
  std::size_t lo;
  if (hi == 0) { lo = 0; hi = 0; }
  else if (hi >= table.slices.size()) { hi = table.slices.size() - 1; lo = hi; }
  else { lo = hi - 1; }

  double ctLo, cpLo, ctHi, cpHi;
  bool jClampedLo = false, jClampedHi = false;
  detail::InterpWithinSlice(table.slices[lo], J, ctLo, cpLo, jClampedLo);
  detail::InterpWithinSlice(table.slices[hi], J, ctHi, cpHi, jClampedHi);

  const double rpmLo = table.slices[lo].rpm, rpmHi = table.slices[hi].rpm;
  const double span = rpmHi - rpmLo;
  const double f = (span > 0.0) ? (rpm - rpmLo) / span : 0.0;

  out.Ct = ctLo + f * (ctHi - ctLo);
  out.Cp = cpLo + f * (cpHi - cpLo);
  out.interpClamped = out.interpClamped || jClampedLo || jClampedHi;
  return out;
}

// --- İtki / güç / tork formülleri ---

/// \brief T = Ct * rho * n^2 * D^4. n devir/s cinsinden ve yapısı gereği daima
/// >= 0'dır. İtki, dönüş yönünden bağımsız olarak sabit itki ekseni (+X)
/// boyunca etki eder.
inline double PropThrust(double ct, double rho, double n, double diameterM)
{
  const double d2 = diameterM * diameterM;
  return ct * rho * n * n * (d2 * d2);
}

/// \brief P_prop = Cp * rho * n^3 * D^5.
inline double PropPower(double cp, double rho, double n, double diameterM)
{
  const double d2 = diameterM * diameterM;
  return cp * rho * n * n * n * (d2 * d2 * diameterM);
}

/// \brief Q_prop (tork BÜYÜKLÜĞÜ) = P_prop / omega. omega=0'da tam sıfıra
/// bölmemek için omega bir sayısal tabanla korunur; P_prop n^3 çarpanı
/// nedeniyle n->0 giderken zaten 0'a gider, yani taban yalnızca sayısal
/// güvenlik amaçlıdır.
inline double PropTorqueFromPower(double pProp, double omegaAbs, double omegaSafeFloor)
{
  const double omegaSafe = std::max(omegaAbs, omegaSafeFloor);
  return pProp / omegaSafe;
}

/// \brief Bir zaman adımı için pervane aerodinamik yüklenmesinin sonucu.
struct PropLoadResult
{
  double n = 0.0;              // devir/s, |omega|/(2*pi)
  double rpm = 0.0;             // teşhis; işaretli (omega ile aynı işaret)
  double J = 0.0;
  double Ct = 0.0;
  double Cp = 0.0;
  double thrust_N = 0.0;        // büyüklük; sabit +X itki ekseni boyunca
  double pProp_W = 0.0;
  double qPropMagnitude_Nm = 0.0;
  double qPropSigned_Nm = 0.0;   // omega'ya karşı koyar; ODE'nin beklediği terim
  bool interpClamped = false;
};

/// \brief Tek bir motor için pervane aerodinamik yüklenmesinin tam
/// değerlendirmesi: AdvanceRatio + LookupCtCp + yukarıdaki üç formül. omega, o
/// motorun kendi çerçevesindeki işaretli durumudur; n/J/Ct/Cp/itki/pProp
/// |omega| ile hesaplanır, çünkü APC tablosu yalnızca ileri (pozitif J)
/// çalışmayı kapsar ve itki ekseninin yönü dönüş işaretine bağlı değildir.
/// qPropSigned daima omega'ya karşı koyar (omega=0'da 0) ve doğrudan
/// IntegrateRotorStep'in ODE'sinde kullanılmak üzere döndürülür.
inline PropLoadResult PropAeroLoad(
    const ApcTable &table, double omega, double vAxial, double diameterM,
    double rho, double nSafeFloorRevS)
{
  PropLoadResult out;
  const double omegaAbs = std::abs(omega);
  out.n = omegaAbs / (2.0 * M_PI);
  out.rpm = omega * 60.0 / (2.0 * M_PI);

  out.J = AdvanceRatio(vAxial, out.n, diameterM, nSafeFloorRevS);

  const double rpmQuery = out.n * 60.0;
  const CtCpResult ctcp = LookupCtCp(table, rpmQuery, out.J);
  out.Ct = ctcp.Ct;
  out.Cp = ctcp.Cp;
  out.interpClamped = ctcp.interpClamped;

  out.thrust_N = PropThrust(out.Ct, rho, out.n, diameterM);
  out.pProp_W = PropPower(out.Cp, rho, out.n, diameterM);

  const double omegaSafeFloor = 2.0 * M_PI * nSafeFloorRevS;
  out.qPropMagnitude_Nm = PropTorqueFromPower(out.pProp_W, omegaAbs, omegaSafeFloor);
  const double sign = (omega >= 0.0) ? 1.0 : -1.0;
  out.qPropSigned_Nm = sign * out.qPropMagnitude_Nm;

  return out;
}

}  // namespace falcon_v2_propulsion

#endif  // FALCON_V2_PROPULSION_MODEL_HH_

# FALCON V2: flaperon + ±45° aileron veri paketi (v2)

Bu paket, `falcon-gazebo-sim` simülasyonunu yeni kumanda düzenine geçirmek için gereken
bütün verileri içerir. Paketi `falconv2-flap` analiz projesi hazırladı (2026-10-28). Uygulayıcı,
sim reposunu yöneten Claude'dur. Bu belge ona yazılmıştır.

**Sayı uydurulmadı.** Her değer ya STL ölçümü, ya XFoil/AVL hesabı ya da kaynaklı bir veridir.
Tahminler ve düşük güvenli değerler aşağıda ayrıca işaretlidir. Doğrulanmamış her şey
`DOĞRULANACAK` diye geçer.

---

## 1. Kararlar (kullanıcı onaylı)

| Konu | Değer |
|---|---|
| Kanat başına hareketli yüzey | 2: iç **flaperon** (y 0.3104–0.5499 m) + dış **aileron** (y 0.5503–0.7897 m) |
| Dış aileron | Yalnız roll, **±45°** (STL'de 60°'ye kadar mekanik engel yok) |
| İç flaperon | Flap (simetrik sarkma) + roll (antisimetrik), **±26.5°** (çentiğe değme açısı, kullanıcı değeri) |
| CG | Burun ucundan **370 mm** geride → sim CAD çerçevesinde **x = 0.156880 m** (burun ucu `nose_analog.stl` x 526.880 mm − 370; eski 0.168309), z 0.100 değişmedi |
| Kütle | 6.0 kg (değişmedi) |
| Elevator / rudder | ±45°, geometri değişmedi |
| Crow | Kullanılmayacak |

## 2. Paket içeriği

| Dosya | Ne |
|---|---|
| `config/aero_v2_config.yaml` | Yeni aero yapılandırması (tam dosya; v2 şeması, yüzey başına ızgara, flaperon yan tablosu `flaperon_side`) |
| `config/actuator_v2_surfaces.yaml` | Dört kanat yüzeyinin servo bölümü (aileron ±45°, flaperon ±26.5°) |
| `sdf_snippets.xml` | Dört yüzeyin `<link>` + `<joint>` blokları (sayılarla dolu) |
| `meshes/{left,right}_{flap,aileron}.stl` | Yeni parçalar, **sim CAD çerçevesinde, mm** (SDF'de ölçek 0.001) |
| `geometry_links.json` | Menteşe noktası/ekseni, hacim, COM, kg başına atalet, çarpışma kutusu, servo kazançları |
| `expected_values.json` | Test beklenen değerleri (trim, roll; v2 ve v1 karşılaştırması) |
| `tests/sim_aero_mirror.py` | Eklenti matematiğinin Python aynası + v2 flaperon uzantısı + trim/roll çözücü |
| `data/gazebo_coeffs.json` | Ham sonuçlar: δ_eff tabloları, AVL farkları, şerit alanları, temel türevler |
| `data/modes.json`, `fuselage_effect.json`, `stall.json` | Mod analizi, gövde etkisi, stall tahmini |

Üretici betikler `falconv2-flap` reposunda: `analiz/scripts/gazebo_coeffs.py`,
`gazebo_meshes.py`, `gazebo_package.py`, `tail_xfoil.py`, `stl_tail_model.py`,
`stl_fuselage_model.py`, `fuselage_effect.py`.

## 3. Çerçeveler ve işaretler

- **Sim CAD çerçevesi** = analizdeki "old" çerçeve: mm, +x ileri, +y sol, +z yukarı. Kanıt:
  simdeki CG (0.168309, 0, 0.100) m, eski XFLR5 CG'sinin (0.0637 m) bu çerçevedeki
  karşılığıdır (231.96 − 63.7 = 168.26 mm; 0.05 mm yuvarlama farkı). Yeni sol flap + aileron mesh'lerinin birleşik sınırları, simdeki eski birleşik
  `left_aileron.stl` ile **birebir aynıdır** ([-23.6, 310.4, 107.7]–[37.5, 789.7, 128.6] mm).
- **Menteşe** (STL'deki 4 mm karbon çubuk ekseni): x = 30.33 mm, z = 112.136 + 0.01339·|y| mm.
  Simdeki eski aileron ekseni bundan ≈ 6 mm aşağıda ve 2.6 mm öndeydi.
- **Joint ekseni**: sol (0, 0.99991, +0.013389), sağ (0, 0.99991, −0.013389). Pozitif açı
  firar kenarını yukarı kaldırmalıdır (simdeki aileron/elevator kuralı). **DOĞRULANACAK**
  (her yeni jointte kinematik işaret testi).
- **Aero girdileri** (eklentideki tanımlar korunur):
  - `deltaA = 0.5·aileron_sign·(θR − θL)` (dış aileronlar). Pozitif = sağa yatış.
  - Flaperon (YENİ, yan tablosu): `deltaFL = sL·θLF`, `deltaFR = sR·θRF`: her flapın kendi
    fiziksel sapması, firar kenarı aşağı pozitif. Sol katkı `flaperon_side(deltaFL)`; sağ katkı
    aynı tablodan `flaperon_side(deltaFR)`, CY/Cl/Cn işareti ters (ayna).
  - `sL = flap_left_down_sign`, `sR = flap_right_down_sign`, ikisi de −1 varsayımı
    (pozitif eklem = firar kenarı yukarı). **DOĞRULANACAK.**

## 4. Uygulanacak değişiklikler

Sim kodunun haritası ve satır numaraları için Codex'in incelemesi kullanıldı (repo 566ec63,
2026-10-07). Satır numaraları o sürüme göredir.

### 4.1 `model/model.sdf`
1. Eski birleşik `left_aileron`/`right_aileron` link+joint'lerini (≈518–642) `sdf_snippets.xml`'deki
   dört blokla değiştir: `left_flap`, `right_flap`, `left_aileron`, `right_aileron`.
   Mesh'leri `meshes/` altına kopyala.
2. **base_link CG**: `model.sdf:49`'daki inertial pose x 0.168309 → **0.156880**. Aero momentleri
   base_link kütle merkezine uygulanıyor; katsayılar bu CG'ye göre hesaplandı.
3. Kontrol yüzeyi kütleleri **yer tutucu 0.001 kg** (v1 kuralı korundu; gerçek kütle ölçülmedi,
   `DATA_REQUIRED`). Ataletler STL geometrisinden, bu yer tutucu kütleyle.
4. Eyleyici topic'leri (≈1094–1113) ve ArduPilotPlugin kanalları (≈1412–1560): iki flaperon
   için yeni COMMAND kanalı ekle (bkz. 4.4).
5. Link/joint sayısı 8/7 → 10/9; yorumlardaki sayıları güncelle.

### 4.2 Eyleyici (`config/actuator_v1_config.yaml` + C++)
- `actuator_v2_surfaces.yaml` bloklarını kullan. Tork/hız: **EMAX ES3054 standart, üretici
  verisi** ([EMAX ürün sayfası, "Specifications"](https://shop.emaxmodel.com/products/emax-es3054-17g-3-5kg-0-13sec-23t-metal-gear-digital-servo-for-rc-airplane-es3154-upgrade)), 4.8 V: 3.0 kgf·cm = 0.2942 N·m, 0.15 s/60°; 6.0 V: 0.3432 N·m, 0.13 s/60°.
  HV sürümü yalnız bayi verisi; takılı olanın standart mı HV mi olduğu **DOĞRULANACAK**.
- `ActuatorSystem.hh:25–33`: `kLeftFlap=5, kRightFlap=6, kNumSurfaces=7`.
- `ActuatorSystem.cc:32–47`: joint adları, YAML anahtarları, topic'ler ve SDF topic
  parametreleri dizilerine iki flap ekle. `138–145` günlük metni.

### 4.3 Aero (`config/aero_v2_config.yaml` + C++)
Yeni şema: her tablo kendi `breakpoints_deg` + `domain_bound_deg`'ini taşır; `flaperon_side`
13 noktalı (±26.5°), diğerleri 15 noktalı (±45°). Sıfır indeksi sabit 7 değil, ızgarada 0'ın yeri.

**Stall sınırlayıcı:** `CLmax_manufacturer` 1.318 (STL kritik kesit) ve `alpha_transition_deg`
**7.23°**. v1'deki 9.25° yeni CL0/CLα ile geçişte doğrusal CL'yi 1.419'a çıkarıp
`Hpos<0` yapıyordu (doyum bozuluyordu). Yeni geçiş v1 kuralıyla türetildi: geçişteki doğrusal
CL / CLmax oranı v1'deki gibi 0.927. Yükleyiciye `Hpos>0` ve `Aneg>0` kontrolü ekleyin.
Alternatif: kesitlerin 2B polarları düşük Re'de α ≈ 4.5°'de doğrusallıktan ayrılıyor; bu 3B'de
≈ 3°'lik geçiş verirdi ama AVL'nin doğrusal türevleriyle çelişir. Seçilmedi; uçuş stall verisiyle
ayarlanmalı.

| C++ yeri | Değişiklik |
|---|---|
| `AerodynamicsSystem.hh:48–52`, `.cc:29–33, 82–113` | İki flap joint'i: ad, entity, yokluk/position kontrolü |
| `AerodynamicsSystem.cc:309–326` | `deltaFL`, `deltaFR` hesapla (bölüm 3) |
| `AeroModel.hh:156–160` | `AeroState`'e `deltaFL`, `deltaFR` |
| `AeroModel.hh:21–30, 83–145` | Yüzey başına ızgara ve dizi boyutu; `kCtrlZeroIndex` yerine ızgaradaki 0 |
| `AerodynamicsSystem.cc:219–248` | v2 yükleyici; doğrulama: artan, sonlu, eşit uzunluk, 0 içerir, uçlar = ±domain_bound |
| `AeroModel.hh:369–417` | Toplamsal katkı: `L = side(deltaFL)`, `R = side(deltaFR)`; `CL += L.dCL + R.dCL`, `cmStatic += L.dCm + R.dCm` (My bu güncel cmStatic ile), `CD += L.dCD + R.dCD`, `CY/Cl/Cn += L.* − R.*` |

`tests/sim_aero_mirror.py` bu formülleri birebir uygular (v1 için kendi testinden geçer:
sıfır durumda CL 0.437035, Cm 0.010550, CD 0.04518477841668). C++ değişikliğini bu aynayla
karşılaştırın.

**Neden yan tablosu:** simetrik + antisimetrik iki tablonun toplamı, verim doğrusal olmadığı için
karışık sapmada hatalıdır. Örnek: sol flap 26.5°, sağ 0° → toplam yöntemi Cl'yi 0.0190, yan
tablosu 0.0125 verir (%52 fazla). Yan tablosunda her flap kendi açısının verimiyle hesaplanır.
Kalan yaklaşım: iki taraf arasındaki indüklenmiş sürükleme çapraz terimi ihmal edildi (küçük).
Saf durumlarda iki yöntem aynıdır (tam antisimetrik ±26.5°: Cl 0.0314).

### 4.4 ArduPilot SITL (kaynak: ArduPlane 4.5.7 / 4.6.3 kodu ve resmî belgeler)
| Çıkış | `SERVOn_FUNCTION` |
|---|---|
| Dış aileronlar | **4** (Aileron); ayrı çıkış önerilir (ayrı trim/ters çevirme) |
| Sol iç flaperon | **24** (FlaperonLeft) |
| Sağ iç flaperon | **25** (FlaperonRight) |

- Flap girişi: `RCx_OPTION = 208`. Kademeler: `FLAP_1_PERCNT/SPEED`, `FLAP_2_PERCNT/SPEED`;
  otomatik: `TKOFF_FLAP_PCNT`, `LAND_FLAP_PERCNT`; hız sınırı `FLAP_SLEWRATE` (%/s).
- Karışım (4.5.7/4.6.3 `servos.cpp flaperon_update`): `sol = clamp(A + F)`, `sağ = clamp(A − F)`,
  ±4500 iç ölçek. Öncelik yok, her çıkış ayrı kırpılır. Flaperon roll katkısı için ayrı kazanç
  yok; flaperonlar dış aileronla aynı roll komutunu alır.
- ardupilot_gazebo eşlemesi: `cmd = multiplier·(r + offset)`, `r = (PWM−servo_min)/(servo_max−servo_min)`.
  `offset = −0.5` ile `multiplier = 2·θmax`: aileron **π/2 = 1.570796**, flaperon
  **53π/180 = 0.925025**. Bu çarpan PWM aralığından bağımsızdır, **ancak** SDF'deki
  `servo_min/servo_max` ile ArduPilot'taki `SERVOn_MIN/MAX` birebir aynı olmalı. Mevcut sim
  800/1500/2200 kullanıyor; korunabilir. Biri değişip diğeri değişmezse tam yüzey hareketi bozulur.
  İşaret (`multiplier` ve `SERVOn_REVERSED`) yön testinden belirlenir.

## 5. Katsayılar: kaynak ve güven

### 5.1 Yöntem
- **Geometri:** STL'ye sadık AVL modeli. Kanat 13 kesit, STL'den CST ile; burulma 3.95° → −0.8°;
  yukarı kıvrık uç. Kuyruk STL'den: yatay ters NACA 2410, geliş **−1.94°**; fin NACA 0010,
  z 0.009–0.209.
- **Viskoz düzeltme:** AVL doğrusal ve viskozsuz. Her yüzey için
  `δ_eff(δ) = ΔCl_XFoil(δ) / viskozsuz eğim` kullanıldı. Kanatta istasyon ortalaması, Re 180k–250k,
  ±45°'ye kadar. Kuyrukta kendi profili ve menteşesiyle (viskozsuz eğim 0.0733/° yatay,
  0.0778/° fin). Yukarı ve aşağı sapmanın verimi farklı: antisimetrik sapma CL/Cm yan etkisi de üretir.
- **Referans nokta:** V 18 m/s, α 1.054° (AVL trimi), CG 0.0751 m (XFLR5/AVL çerçevesi).
- **Sürükleme artığı:** `ΔCD = ΔCDi_AVL + ΔCDp_XFoil − k[(CL+ΔCL)² − CL²]`. Sim `k·CL²`'yi ayrıca
  ekliyor; çifte sayım yok.
- **Gövde:** Cmα += Munk–Multhopp (+0.1752, kuyrukta downwash 0.363 AVL'den); Cnβ += Raymer
  (−0.0122, konveks hacim). Diğer türevler gövdesiz.
- **İşaret eşlemesi** AVL → sim kanıtlandı. Eski XFLR5 modelinin AVL kopyası sim referans
  noktasında koşuldu; CYp dışında bütün türevlerin işareti simdeki değerlerle aynı.
  Kontrol tabloları fiziksel kurallarla denetlendi: aileron + → sağa yatış + ters yalpa;
  flap ↓ → CL ↑, burun ↓; elevator + → burun ↓; rudder + → burun sağa.

### 5.2 v1 → v2 karşılaştırması (önemli değişiklikler)
| Katsayı | v1 (sim) | v2 | Neden |
|---|---|---|---|
| CLα /rad | 5.446 | 5.596 | STL kanadı |
| Cmα /rad | −1.658 (CG 0.0637) | −0.951 | CG 11 mm geride + gövde |
| CL0 | 0.437 | 0.516 | STL burulma/kamburluk |
| Cm0 | 0.0106 | **0.1108** | Kuyruk geliş −1.94° (v1'de 0°) + CG |
| Clβ | −0.0072 | **−0.0399** | Yukarı kıvrık uçlar + fin STL'de 51 mm yukarıda |
| Cnβ | 0.0355 | 0.0178 | Gövde (Raymer) |
| Clr | 0.106 | 0.161 | AVL STL |
| Aileron Cl @10° | 0.0723 | **0.0207** | Yalnız dış yüzey + viskoz verim; flaperon ayrıca +0.0151 |
| Elevator dCm @10° | −0.1744 | **−0.0880** | STL elevator kordu daha kısa + viskoz verim |
| Rudder Cn @10° | −0.0047 | −0.0027 | AVL + viskoz verim |
| CLmax | 1.42 (üretici) | **1.318** | STL kritik kesit yöntemi (kötümser); seçim 5.3'te |

### 5.3 Güven ve açık noktalar
| Konu | Durum |
|---|---|
| Yuvarlama | YAML 5 ondalık; ham değerler `data/gazebo_coeffs.json`. Rudder Cl eklentide doğrusal `Cldr`'den üretilir (±45°'de ham AVL'den farkı ≈ 0.001) |
| \|δ\| ≤ 15–20° tabloları | Orta–yüksek güven (XFoil yapışık akış) |
| \|δ\| > 25° | **Düşük güven**: XFoil ayrılmış akış. Elevator +25…+45° doymuş görünüyor (+35° yakınsamadı, aradeğer) |
| Yanal türevler (CYβ, Clβ, Cnβ, Cnp) | Aynı geometride AVL ile XFLR5 arasında %13–37 fark; yöntem belirsizliği. CYp'nin işareti iki programda farklı (küçük terim) |
| Cm0 / trim elevatoru | Model 18 m/s düz uçuşta **≈ +10.7° (firar kenarı aşağı)** elevator öngörüyor (v1: −3.7°). Sebep STL'deki kuyruk geliş açısı. **Gerçek uçuş logundaki elevator trimiyle doğrulanmalı**; tutmazsa kuyruk geliş açısı veya Cm0 yeniden ele alınmalı |
| CLmax | 1.318 kötümser yöntem; iyimser sınır ≈ 1.39–1.40. Üretici 1.42. Kullanıcı kararı bekleniyor; paket 1.318 ile hazırlandı |
| Kontrol yüzeyi kütleleri | Yer tutucu 0.001 kg, gerçek değer ölçülmedi |
| ES3054 standart / HV | DOĞRULANACAK (pakette standart 4.8 V) |
| Pervane akımı | Modelde yok (flaperonun iç kısmı akımda: y 0.135–0.465 m) |
| Gövde Cm0, CL0, CYβ katkıları | Hesaplanmadı (yalnız Cmα ve Cnβ düzeltildi) |
| Atalet (Ixx, Iyy, Izz) | Simdeki değerler korunuyor; ölçülmedi |

## 6. Testler ve beklenen değerler

`expected_values.json` dosyası `tests/sim_aero_mirror.py` ile **v2 yapılandırmasından** üretildi
(JSON'da trim 4, roll 2, sıfır durum 6 ondalığa yuvarlı; karşılaştırma toleransı buna göre).
C++ eklentisi değiştirildikten sonra aynı girdilerle aynı sonucu vermeli. Ayna:
`load_config("config/aero_v2_config.yaml")` (yolu açıkça verin; DEFAULT_PATH falconv2-flap'e göredir).

### 6.1 Birim testleri (Gazebo'suz)
1. `sim_aero_mirror.load_config("config/aero_v2_config.yaml")`: yükleme hatasız olmalı.
   Sıfır durum: CL 0.51592, CD 0.049154, CY 0, Cl 0, Cm 0.11075, Cn 0.
2. C++ `ComputeAero` ile çalışma anında ayna karşılaştırması (rastgele α, β, p, q, r, delta; aynı
   YAML): fark < 1e-9.
3. Izgara doğrulaması: her tablo artan, sonlu, 0 içeriyor, uçlar ±domain_bound; flaperon
   tablolarında sıfırdaki değer tam 0.
4. v1 regresyonu: v1 yapılandırmasıyla eski davranış aynen korunmalı (flap tabloları yok).

### 6.2 Trim (aero trim referansı: 6 kg, β = p = q = r = 0, L = mg, itki = D rüzgâr ekseninde)
Tam Gazebo trimi değildir: gövde doğrultusundaki itkinin α≠0'daki düşey bileşeni ihmal edilir.
Simde düz uçuşta küçük fark beklenir.

| V m/s | flap | α° | δe° (+ = firar kenarı aşağı) | CL | L/D | D (= itki) N |
|---|---|---|---|---|---|---|
| 13 | 0° | 7.86 | -3.06 | 1.259 | 10.5 | 5.60 |
| 15 | 0° | 4.23 | +4.68 | 0.946 | 11.6 | 5.05 |
| 18 | 0° | 1.04 | +10.70 | 0.657 | 11.7 | 5.03 |
| 22 | 0° | -1.36 | +15.83 | 0.440 | 10.1 | 5.85 |
| 13 | 26.5° | 6.25 | -8.44 | 1.259 | 9.8 | 6.03 |
| 14 | 26.5° | 4.34 | -2.19 | 1.086 | 10.3 | 5.72 |
| 15 | 26.5° | 2.80 | +1.30 | 0.946 | 10.6 | 5.56 |

v1 (karşılaştırma, flap 0): 13 m/s α 10.23° δe -16.38°; 15 m/s α 6.07° δe -9.48°; 18 m/s α 2.59° δe -3.70°; 22 m/s α -0.02° δe +0.64°

### 6.3 Roll (sabit yuvarlanma, β = r = 0; ArduPlane karışımı sol = clamp(F+A), sağ = clamp(F−A))
| V | Durum | dış aileron | sol / sağ flaperon (aşağı +) | p °/s | yalnız dış aileron °/s |
|---|---|---|---|---|---|
| 14 | tam aileron, flap 0 | 45° | +26.50° / -26.50° | 119.0 | 77.0 |
| 14 | yarım aileron, flap 0 | 22.5° | +13.25° / -13.25° | 77.0 | 51.6 |
| 14 | tam aileron, tam flap | 45° | +26.50° / +0.00° | 93.7 | 77.0 |
| 14 | tam aileron, yarım flap | 45° | +26.50° / -13.25° | 108.2 | 77.0 |
| 18 | tam aileron, flap 0 | 45° | +26.50° / -26.50° | 153.1 | 99.0 |
| 18 | yarım aileron, flap 0 | 22.5° | +13.25° / -13.25° | 99.0 | 66.3 |
| 18 | tam aileron, tam flap | 45° | +26.50° / +0.00° | 120.5 | 99.0 |
| 18 | tam aileron, yarım flap | 45° | +26.50° / -13.25° | 139.2 | 99.0 |

Not: tam flapta tam roll komutu karışımda kırpılır: aşağı inen taraf 26.5°'de kalır, öbür taraf
0°'ye iner.

### 6.3b Tek yüzey takılması (14 m/s, roll momenti)
| Durum (14 m/s) | Cl |
|---|---|
| sol flaperon 26.5° aşağı takılı | +0.0125 |
| sol dış aileron 45° aşağı takılı (deltaA 22.5° yaklaşımı) | +0.0385 |
| karşı: dış aileron tam (deltaA −45°) | -0.0575 |
| karşı: flaperon tam roll (sol −26.5°, sağ +26.5°) | -0.0314 |

### 6.4 Gazebo/SITL testleri
1. **Kinematik işaret:** her yeni joint'e +10° ver, firar kenarı yukarı gitmeli. `deltaFL`
   ve `deltaFR` tanı çıktısı beklenen işaretleri vermeli (firar kenarı aşağı +).
2. **Aero işaret:** flap sarkması → CL artar, burun aşağı; sol flaperon aşağı (deltaFL +) ve `deltaA` + → sağa
   yatış; ikisi birlikteyken yalpa ters yönde (Cn > 0, burun sola).
3. **Sınır:** aileron ±45°'ye, flaperon ±26.5°'ye ulaşıyor; tam flap + tam roll'da karışım
   kırpması doğru.
4. **Trim:** MANUAL/FBWA'da 15–22 m/s düz uçuş; elevator ve α değerleri 6.2 tablosuyla
   karşılaştırılır. Fark varsa aero mi, eyleyici mi, itki mi olduğu ayrıştırılır.
5. **Roll adımı:** 14 ve 18 m/s'de tam çubuk; sabit roll hızı 6.3'e yakın olmalı. Simde β ve r
   serbest olduğu için birebir aynı çıkmaz.
6. **Flap açma:** 15 m/s'de 0 → 26.5°: burun aşağı trim değişimi; elevator farkı 6.2'deki 15 m/s
   satırlarından.
7. **Arıza:** 6.3b tablosu. Bir flaperonun takılması dış aileronla rahat karşılanır; bir dış
   aileronun 45°'de takılması flaperonlarla tek başına karşılanamaz, sağlam aileron da gerekir.
8. **Limit cycle:** yeni yüzey ataletleri/kazançlarıyla eski limit-cycle testlerini tekrarla
   (`docs/validation/2026-09-1x_*`).
9. **Gerçek uçuş verisiyle doğrulama:** düz uçuştaki gerçek elevator trimi, roll hızı ve stall
   hızı (flapsız ≈ 12.7 m/s, flap 26.5° ≈ 13.0 m/s; kritik kesit, kötümser).

## 7. Ek bilgiler (analiz sonuçları)
- **Menteşe momenti**, 25 m/s: aileron +45° 0.080 N·m, −45° 0.060; flaperon +25° 0.069,
  −25° 0.041. ES3054 standart 4.8 V (0.294 N·m) için pay 3.7–7.2 kat (servo kolu ile yüzey
  kolu eşit varsayımı; gerçek bağlantı geometrisi bilinmiyor).
- **Statik marj**: ≈ %17 (gövdeli; aralık %16.7–18.6).
- **Mod analizi** (AVL; gövdesiz/gövdeli aralığı): kısa periyot ζ 0.43–0.51; fugoid T 6–11 s,
  ζ 0.06–0.09; yuvarlanma τ 0.10–0.18 s; Dutch roll T 2.0–3.7 s, ζ 0.07–0.14; spiral nötr ile
  hafif kararsız arası (gövdesiz T2 5.5–10 s, gövdeli > 100 s).
- **Stall**: kök bölgesinden başlıyor (y ≈ 0.16 m); flap stall hızını düşürmüyor.

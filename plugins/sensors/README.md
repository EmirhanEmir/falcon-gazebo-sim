# FALCON V2 — Sensör Eklentileri

İki Gazebo Sim Harmonic System eklentisi ve bunların paylaştığı referans
çerçevesi kütüphanesi: **FalconV2Pitot** (pitot airspeed) ve
**FalconV2Magnetometer** (manyetometre).

Kapsam: sensör ölçümü üretmek ve yayınlamak. Bu eklentiler hiçbir bağa kuvvet
veya tork uygulamaz, hiçbir joint'i sürmez, aerodinamik/itki hesabı yapmaz ve
otopilot mantığı içermez.

IMU, GPS ve barometre ayrıca modellenmemiştir; onlar `model/model.sdf` içinde
Gazebo'nun yerleşik `<sensor>` elemanlarıyla tanımlıdır.

## Neden özel bir manyetometre

Hedeflenen Gazebo sürümündeki yerleşik `gz-sim-magnetometer-system`, world
dosyasının `<magnetic_field>` değerini yok sayar ve kendi derlenmiş
varsayılanını kullanır. `FalconV2Magnetometer` bunun yerine **aynı konuda, aynı
mesaj tipiyle** yayın yapar — `/model/falcon_v2/sensors/mag`,
`gz.msgs.Magnetometer`, `field_tesla`, gövde FLU çerçevesinde — böylece hiçbir
tüketicinin değişmesi gerekmez, ama bildirilen Dünya alanı değeri gerçekten
kullanılır.

## Dosyalar

| Dosya | Amaç |
| ---- | ------- |
| `Frames.hh` | **Projedeki her çerçeve/işaret dönüşümünün tek tanımlandığı yer.** FLU↔FRD ve ENU↔NED eksen takasları, irtifa↔NED-down, FLU/ENU attitude → FRD/NED kuaterniyonu, NED yataydan heading. Saf matematik; yalnızca `gz-math7`'ye bağımlıdır, canlı Gazebo durumu okumaz. |
| `PitotModel.hh` | Saf matematik: hava-bağıl hız vektörü, airspeed büyüklüğü, diferansiyel basınç. Yalnızca `gz-math7`. |
| `PitotSystem.hh` / `.cc` | `gz-sim8` System eklentisi: rüzgâr aboneliği, montaj noktasındaki hızın okunması, airspeed ve basınç yayını. |
| `MagnetometerSystem.hh` / `.cc` | `gz-sim8` System eklentisi: world alan vektörünün gövde çerçevesine döndürülüp yayınlanması. |
| `CMakeLists.txt` | `libFalconV2Pitot.so` ve `libFalconV2Magnetometer.so` derleme tanımı. |

`Frames.hh` bu paketin en önemli dosyasıdır. Projenin kuralı, bir çerçeve
dönüşümünün **bir kez** burada türetilmesi ve başka hiçbir dosyada yeniden
türetilmemesidir — sessizce geçilen bir çerçeve dönüşümü bu projedeki en pahalı
hata sınıfı olduğu için.

## Bağımlılıklar

- `gz-sim8`, `gz-plugin2` (`register` bileşeni), `gz-transport13`,
  `gz-msgs10`, `gz-math7`
- CMake ≥ 3.16, C++17 derleyici

`yaml-cpp` gerekmez: bu iki eklenti katsayılarını YAML'den değil, doğrudan
`model/model.sdf` içindeki `<plugin>` parametrelerinden okur.

## Derleme

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

`build/libFalconV2Pitot.so` ve `build/libFalconV2Magnetometer.so` üretir.
`build/` git tarafından yok sayılır. `scripts/build_plugins.sh` bunları diğer
üç eklentiyle birlikte derler.

## Çalışma zamanı arayüzü

### FalconV2Pitot

`model/model.sdf` içindeki `<plugin filename="FalconV2Pitot"
name="falcon_v2_sensors::PitotSystem">` bloğunun parametreleri:

| Parametre | Anlamı |
| --------- | ------ |
| `<mount_offset_body>` | Pitot'un `base_link` orijinine göre gövde çerçevesi konumu (m) |
| `<update_rate_hz>` | Yayın hızı |
| `<noise_stddev_mps>` | Gürültü standart sapması (m/s); 0 ise gürültüsüz |
| `<wind_topic>` | Abone olunacak rüzgâr konusu |
| `<airspeed_topic>` | Airspeed yayın konusu |
| `<pressure_topic>` | Diferansiyel basınç yayın konusu |

| Konu | Tip | Yön |
| ---- | --- | --- |
| `/model/falcon_v2/wind` | `gz.msgs.Vector3d` | abone (dünya çerçevesinde hava kütlesi hızı, m/s) |
| `/model/falcon_v2/sensors/pitot/airspeed_mps` | `gz.msgs.Double` | yayın |
| `/model/falcon_v2/sensors/pitot/differential_pressure_pa` | `gz.msgs.Double` | yayın |

Airspeed, montaj noktasındaki hava-bağıl hızdan türetilir; yani hem gövde
hızını hem açısal hızdan gelen montaj noktası katkısını hem de rüzgârı içerir.

### FalconV2Magnetometer

| Parametre | Anlamı |
| --------- | ------ |
| `<world_magnetic_field_tesla>` | Dünya çerçevesinde manyetik alan vektörü (T) |
| `<update_rate_hz>` | Yayın hızı |
| `<topic>` | Yayın konusu |

| Konu | Tip | Yön |
| ---- | --- | --- |
| `/model/falcon_v2/sensors/mag` | `gz.msgs.Magnetometer` | yayın (gövde FLU çerçevesinde `field_tesla`) |

### Canlı gözlem

```bash
gz topic -e -t /model/falcon_v2/sensors/pitot/airspeed_mps
gz topic -e -t /model/falcon_v2/sensors/mag
```

## Bilinen sınırlamalar

- **Simülasyona özgü pitot.** Gerçek uçak airspeed sensörü olmadan uçtu. Model;
  ters akış, sıkıştırılabilirlik ve montaj hatası düzeltmesi içermez.
- **Sabit hava yoğunluğu.** Diferansiyel basınç sabit ρ ile hesaplanır; EAS ile
  TAS yapısal olarak özdeştir.

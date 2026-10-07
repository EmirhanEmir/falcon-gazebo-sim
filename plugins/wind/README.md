# FALCON V2 — Rüzgâr/Hamle Üreticisi Eklentisi

Asgari ve deterministik bir atmosferik rüzgâr ve hamle **yayıncısı** sağlayan
Gazebo Sim Harmonic System eklentisi.

`V_wind_world = V_steady + V_gust(t)` hesaplar — dünya çerçevesi, m/s, hava
kütlesinin hızı; meteorolojideki "rüzgârın geldiği yön" ifadesi değil — ve bunu
her `PreUpdate()` tikinde `/model/falcon_v2/wind` konusunda yayınlar. Bu konu,
aerodinamik, itki ve pitot eklentilerinin halihazırda `Vrel = Vbody − Vwind`
olarak tükettiği konudur. Projedeki tek rüzgâr kaynağı budur; ikinci bir
tüketim yolu yoktur.

Bu eklenti hiçbir bağa **asla** doğrudan kuvvet veya tork uygulamaz — içinde
hiçbir `AddWorldForce`/`AddWorldWrench` çağrısı yoktur. Tek fiziksel çıktısı
yayınlanan hız vektörüdür.

**Modellenmeyenler:** uzamsal rüzgâr gradyanları (uçağın konumuna göre değişen
kayma veya hamle cepheleri), arazi etkileri, iz etkileri ve Dryden/Von Karman
stokastik türbülansı.

## Dosyalar

| Dosya | Amaç |
| ---- | ------- |
| `WindModel.hh` | Saf matematik: 1-kosinüs hamle zarfı, hamle vektörü hesabı, sabit+hamle bileşimi, yön normalizasyonu. Yalnızca `gz-math7`'ye bağımlıdır. |
| `WindSystem.hh` / `.cc` | `gz-sim8` System eklentisi: SDF parametre okuma, komut abonelikleri, tik başına yayın. |
| `CMakeLists.txt` | `libFalconV2Wind.so` derleme tanımı. |

## Bağımlılıklar

- `gz-sim8`, `gz-plugin2` (`register` bileşeni), `gz-transport13`,
  `gz-msgs10`, `gz-math7`
- CMake ≥ 3.16, C++17 derleyicisi

`yaml-cpp` gerekmez: bu eklentinin yüklenecek bir katsayı veri kümesi yoktur —
tek girdileri SDF parametreleri ve canlı komut konularıdır.

## Derleme

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Çıktı: `build/libFalconV2Wind.so`. `build/` dizini git tarafından yok sayılır.

## Çalışma zamanı arayüzü

Sabit rüzgâr ayarlamak için:

```bash
gz topic -t /model/falcon_v2/wind/steady_cmd -m gz.msgs.Vector3d -p "x: 5.0, y: 0.0, z: 0.0"
```

1-kosinüs hamlesi zamanlamak için. Alan sırası sabittir:
`[dir_x, dir_y, dir_z, amplitude_mps, start_delay_s, duration_s]`; buradaki
`start_delay_s`, mutlak benzetim zamanına değil komutun kendi alınma zamanına
görelidir:

```bash
gz topic -t /model/falcon_v2/wind/gust_cmd -m gz.msgs.Double_V \
  -p "data: [1.0, 0.0, 0.0, 8.0, 2.0, 3.0]"
```

Bu komut, dünya +X yönüne esen, tepe değeri 8 m/s olan, komut alındıktan 2 s
sonra başlayıp 3 s süren bir hamledir.

**Sınır:** aynı anda yalnızca bir hamle zamanlanabilir veya sürebilir — yeni
bir hamle komutu mevcut olanı değiştirir.

Yayınlanan rüzgârı izlemek için:

```bash
gz topic -e -t /model/falcon_v2/wind
```

# FALCON V2 — Aktüatör/Servo Plugin'i

Beş kontrol yüzeyi joint'i — `left_aileron_joint`, `right_aileron_joint`,
`left_elevator_joint`, `right_elevator_joint`, `rudder_joint` — için fiziksel
aktüatör zincirini uygulayan Gazebo Sim Harmonic System plugin'i.

Kapsam: aktüatör komutu → sonlu hızlı, sonlu torklu servo dinamiği → gerçek
Gazebo joint hareketi. Hiçbir aerodinamik katsayıyı veya kontrol işareti
eşlemesini değiştirmez, ArduPlane/MAVLink/TECS mantığı içermez — bu plugin
fiziksel aktüatör arayüzünde biter.

## Mimari

```
ham komut (rad, gz.msgs.Double, son geçerli komut tutulur)
  → ClampCommand()        [mekanik hareket limiti, ±45°]
  → RateLimitSetpoint()   [servo slew-rate limiti]
  → PidEffort()           [effort sınırlı PID torku]
  → gz::sim::Joint::SetForce()   [gerçek joint-space tork komutu]
  → fizik motoru gerçek joint hareketini entegre eder
```

**Neden tork tabanlı, neden kinematik bir pozisyon yazımı değil:** pozisyon
yazmak joint'i fizik motorunu tamamen atlayarak ışınlar ve hız ile effort
limitlerini uygulanamaz hâle getirir. `SetForce()` gerçek joint dinamiği
üretir.

PID kazançları her yüzeyin kendi menteşe ekseni etrafındaki ataletine göre
türetilmiştir (yüzey başına, parallel-axis düzeltmeli). Yerçekimi ileri
beslemesi yerine integral etki kullanılır; böylece herhangi bir sabit bozucu,
onu modellemeye gerek kalmadan bastırılır. Anti-windup, hedef kırpıldığında,
aynı yöndeki effort doygunluğunda ve hız doygunluğunda birikimi dondurur.

`model.sdf` içindeki kontrol yüzeyi link kütleleri bilinçli olarak neredeyse
kütlesiz sayısal yer tutucular olduğundan, plugin ayrıca `Configure()` anında
aynı azami hız ve azami effort değerleriyle `SetVelocityLimits()` /
`SetEffortLimits()` çağırarak fizik motoru seviyesinde sert kısıtlar uygular.

## Dosyalar

| Dosya | Amaç |
| ---- | ------- |
| `ActuatorModel.hh` | Saf matematik çekirdeği: clamp, rate limit, anti-windup'lı PID. Hiç Gazebo bağımlılığı yok. |
| `ActuatorSystem.hh` / `.cc` | `gz-sim8` System plugin'i: beş komut aboneliği, beş kalıcı setpoint durumu, tork uygulama, diagnostik. |
| `CMakeLists.txt` | `libFalconV2Actuators.so` derleme tanımı. |

Her sayısal değer `config/actuator_v1_config.yaml` dosyasından gelir.

## Bağımlılıklar

- `gz-sim8`, `gz-plugin2` (`register` bileşeni), `gz-transport13`,
  `gz-msgs10`, `gz-math7`
- `yaml-cpp` (`libyaml-cpp-dev` üzerinden)
- CMake ≥ 3.16, C++17 derleyici

## Derleme

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

`build/libFalconV2Actuators.so` üretir. `build/` git tarafından yok sayılır.

## Runtime Arayüzü

Her yüzey fiziksel açı radyan cinsinden bağımsız olarak komut edilir:

```bash
gz topic -t /model/falcon_v2/actuators/left_aileron/cmd_rad  -m gz.msgs.Double -p "data:  0.1745"
gz topic -t /model/falcon_v2/actuators/right_aileron/cmd_rad -m gz.msgs.Double -p "data: -0.1745"
```

Komut topic'leri: `/model/falcon_v2/actuators/{left_aileron, right_aileron,
left_elevator, right_elevator, rudder}/cmd_rad`. Hiç komut almamış bir yüzey
0 rad'da (nötr) kalır, aksi hâlde son geçerli komutunu tutar.

`/model/falcon_v2/actuators/diagnostics` üzerindeki diagnostik 35 alandır —
left_aileron, right_aileron, left_elevator, right_elevator, rudder sırasıyla
yüzey başına 7 alan: `cmd_rad, target_clamped_rad, setpoint_rad,
actual_angle_rad, actual_rate_rad_s, target_clamp_active,
effort_clamp_active`.

```bash
gz topic -e -t /model/falcon_v2/actuators/diagnostics
```

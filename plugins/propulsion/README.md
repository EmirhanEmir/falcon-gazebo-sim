# FALCON V2 — İtki Plugin'i

Çift motor/ESC/pervane zincirini uygulayan Gazebo Sim Harmonic system plugin'i:
4S pack üzerinde 2 × SunnySky X2820 860KV motor, 2 × APC 13x6.5E pervane.

Kapsam yalnızca itki fiziğidir. Aerodinamik katsayı, kontrol yüzeyi tahriki veya
otopilot mantığı içermez.

## Model zinciri

```
throttle → ESC/motor elektriksel → motor torku
         → rotor açısal dinamiği (explicit-Euler durum, her tick entegre edilir)
         → RPM → advance ratio J → APC Ct(J,RPM) / Cp(J,RPM)
         → thrust + pervane aerodinamik torku → reaction torque
         → gerçek hub konumunda base_link'e uygulanan kuvvet/moment
```

Zincir hiçbir zaman `throttle × maximum_thrust` hâline indirgenmez. Pervane
eklemlerinin açısal hızını da aynı omega durumu sürer; böylece görsel dönüş,
kuvveti üreten RPM'den hiçbir zaman ayrışamaz.

## Dosyalar

| Dosya | Amaç |
| ---- | ------- |
| `PropulsionModel.hh` | Saf matematik çekirdeği: Kt/Ke, elektriksel model, rotor ODE adımı, advance ratio, APC tablosunun okunması ve 2-B Ct/Cp interpolasyonu, thrust/güç/tork. Gazebo bağımlılığı yoktur. |
| `PropulsionSystem.hh` / `.cc` | `gz-sim8` system plugin'i: iki omega durumunu tutar, her hub'da wrench uygular, pervane eklemlerini sürer, teşhis yayınlar. |
| `CMakeLists.txt` | `libFalconV2Propulsion.so` için derleme tanımı. |

Parametreler `config/propulsion_v1_config.yaml` dosyasından gelir. Ct/Cp verisi
ise `config/propulsion_data/apc_13x65e_parsed.csv` dosyasından gelir; bu CSV,
resmî APC performans dosyası `PER3_13x65E.dat` üzerinden mekanik olarak
üretilmiştir — bkz. `config/propulsion_data/PROVENANCE.md`. Hiçbir Ct/Cp değeri
uydurulmamıştır: dönen her değer ya tablodaki gerçek bir APC değeridir ya da iki
gerçek değer arasında doğrusal interpolasyondur.

## Bağımlılıklar

- `gz-sim8`, `gz-plugin2` (`register` bileşeni), `gz-transport13`,
  `gz-msgs10`, `gz-math7`
- `yaml-cpp` (`libyaml-cpp-dev` paketi ile)
- CMake ≥ 3.16, C++17 derleyicisi

## Derleme

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Çıktı: `build/libFalconV2Propulsion.so`. `build/` dizini git tarafından
yoksayılır.

## Runtime arayüzü

| Topic | Tip | Yön |
| ----- | ---- | --------- |
| `/model/falcon_v2/propulsion/left/throttle_cmd` | `gz.msgs.Double` | abone, [0,1] aralığına kırpılır |
| `/model/falcon_v2/propulsion/right/throttle_cmd` | `gz.msgs.Double` | abone, [0,1] aralığına kırpılır |
| `/model/falcon_v2/wind` | `gz.msgs.Vector3d` | abone |
| `/model/falcon_v2/propulsion/diagnostics` | `gz.msgs.Double_V` | yayın |

Hiç komut gelmezse throttle 0'dır. ArduPlane SITL altında bu topic'ler
`model/model.sdf` içindeki ArduPilotPlugin kontrol kanalları tarafından sürülür.

Teşhis mesajı 28 alandır — motor başına 14, önce sol sonra sağ:
`throttle, current_A, Qmotor, omega_ownframe, rpm, J, Ct, Cp, thrust_N,
Qprop, interpClamped, rpmCapActive, negCurClamped, curLimited`.

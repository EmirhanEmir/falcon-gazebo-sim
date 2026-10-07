# FALCON V2 — Aerodinamik Eklentisi

Uçağın aerodinamik kuvvet ve moment modelini uygulayan Gazebo Sim Harmonic
System eklentisi.

Kapsam yalnızca aerodinamik fiziktir. Beş kumanda yüzeyi ekleminin açısını
okur; onları komutlamaz ve içinde itki ya da otopilot mantığı bulunmaz.

## Dosyalar

| Dosya | Amaç |
| ---- | ------- |
| `AeroModel.hh` | Saf matematik çekirdeği: alfa/beta, dinamik basınç, katsayı toplamı, rüzgâr→gövde kuvvet dönüşümü, kumanda yüzeyi araması. Yalnızca `gz-math7`'ye bağımlıdır. |
| `AerodynamicsSystem.hh` / `.cc` | `gz-sim8` System eklentisi: Gazebo durumunu okur, `ComputeAero()` çağırır, kuvvet/momenti `base_link`'e uygular, tanı yayınlar. |
| `CMakeLists.txt` | `libFalconV2Aerodynamics.so` derleme tanımı. |

Her katsayı `Configure()` anında `config/aero_v1_config.yaml` dosyasından
yüklenir. C++ içinde gömülü katsayı yoktur.

## Bağımlılıklar

- `gz-sim8`, `gz-plugin2` (`register` bileşeni), `gz-transport13`,
  `gz-msgs10`, `gz-math7`
- `yaml-cpp` (`libyaml-cpp-dev` paketiyle)
- CMake ≥ 3.16, C++17 derleyicisi

## Derleme

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Çıktı: `build/libFalconV2Aerodynamics.so`. `build/` dizini git tarafından
yok sayılır. `scripts/build_plugins.sh` bunu ve diğer dört eklentiyi birlikte
derler.

## Çalışma zamanı arayüzü

`model/model.sdf` içinde
`<plugin filename="FalconV2Aerodynamics" name="falcon_v2_aero::AerodynamicsSystem">`
bloğu, `<config_path>`, `<air_density>`, `<wind_topic>` ve
`<diagnostics_rate_hz>` parametreleriyle birlikte bulunur.

| Konu | Tip | Yön |
| ----- | ---- | --------- |
| `/model/falcon_v2/wind` | `gz.msgs.Vector3d` | abone (dünya çerçevesinde hava kütlesi hızı, m/s) |
| `/model/falcon_v2/aerodynamics/diagnostics` | `gz.msgs.Double_V` | yayın |

Tanı alanlarının sırası: `V, alpha_rad, beta_rad, qbar, CL, CD, CY, Cl, Cm, Cn`.
Canlı izlemek için:

```bash
gz topic -e -t /model/falcon_v2/aerodynamics/diagnostics
```

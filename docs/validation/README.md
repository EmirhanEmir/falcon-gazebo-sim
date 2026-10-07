# Doğrulama Kayıtları

Bu klasör, simülasyonun geliştirme sürecinde tutulan iç doğrulama kaydından
seçilmiş beş rapordur. Hepsi çalışan bir Gazebo örneğinden alınan gerçek ölçüm
verisine dayanır; hiçbiri analitik tahmine veya "böyle olması gerekir"
akıl yürütmesine dayanmaz.

Süreçte uygulanan temel kural: **bir sonucu üreten aşama o sonucu
onaylamaz.** Test koşumu, alan analizi ve mühendislik incelemesi ayrı
geçişlerdir, ve her sayısal iddia kabul edilmeden önce ham veriden bağımsız
olarak yeniden hesaplanır. Raporlardaki `PASS` etiketleri bu yüzden ikinci bir
geçişin ürünüdür.

İkinci kural: **beklenti testten önce yazılır.** Kuvvet ve moment işaretleri,
qbar ölçeklemesi ve yanıt yönleri simülasyon çalıştırılmadan önce geometriden
türetilip rapora yazılır — sonuç görüldükten sonra yorum uydurulmaz.

## Raporlar

| Rapor | Ne gösteriyor |
| ----- | ------------- |
| [`flight_envelope_validation`](2026-08-27_flight_envelope_validation.md) | 8 hızda, 115 trim adayı üzerinden uçuş zarfı taraması. En düşük trim hızı 14 m/s; 12.5 m/s'de geçerli trim yok. Zarf üç kademeye ayrılıp hangi hız aralığının gerçekten doğrulanmış olduğu açıkça etiketlenmiş. |
| [`engine_out_asymmetric_thrust_validation`](2026-08-27_engine_out_asymmetric_thrust_validation.md) | Tek motor arızası ve asimetrik itki. Üç işaret beklentisi (yaw yönü, gereken rudder yönü, reaksiyon torku rolü) test *öncesinde* hub geometrisinden türetilmiş, **3/3 doğrulanmış**. Sol/sağ ayna simetrisi <1e-6 N·m. Rudder-only telafiyle uçak spiral divergence'a değil, sabit ~21.5° banklı dönüşe oturuyor. |
| [`wind_gust_disturbance_validation`](2026-08-27_wind_gust_disturbance_validation.md) | Steady rüzgâr ve 1-cosine gust. 14 ön-kayıtlı beklentinin **14'ü** tutuyor; `qbar` ölçeklemesi makine hassasiyetinde eşleşiyor. Gust fonksiyonu her iki sınırda C1-sürekli. Rüzgâr plugin'inin hiçbir yerde kuvvet uygulamadığı üç bağımsız kaynak taramasıyla teyit edilmiş. |
| [`airborne_actuator_limit_cycle_root_cause`](2026-09-10_airborne_actuator_limit_cycle_root_cause.md) | **CRITICAL hata tespiti.** Havadaki kontrol yüzeyleri slew-rate limitine sürekli dayanıyordu: FBWA'da havadaki örneklerin %43.6–86.0'ı rate-pinned. Önceki aşama bu bulguyu yalnızca üç TAKEOFF koşusunda ölçüp %0.5–3.0 olarak raporlamıştı — rapor kapsamın yanlış kurulduğunu gösteriyor ve yeni uçuş yapmadan, eldeki kayıttan yeniden üretiyor. |
| [`actuator_limit_cycle_fix_implementation`](2026-09-12_actuator_limit_cycle_fix_implementation.md) | Aynı hatanın çözümü. Kök neden PID kazançlarının menteşe ekseni yerine CoM ataleti üzerinden türetilmesiydi; düzeltme üç parçalı: **F1** parallel-axis atalet düzeltmesi, **F2** bant genişliği `ω_n` 100→200 rad/s, **F3** rate-saturation anti-windup. Davranışsal kod değişimi tek bir dosyada; travel clamp ve rate limiter'ın bayt düzeyinde değişmediği aritmetik olarak kanıtlanmış. |

## Dağıtılmayan referanslar

Raporlar `tests/gazebo/` test koşum altyapısına ve `docs/source_of_truth/`
mühendislik veri ağacına atıf yapar. Bunların ikisi de bu repository ile
dağıtılmaz; atıflar tarihsel kayıt olarak olduğu gibi bırakılmıştır.

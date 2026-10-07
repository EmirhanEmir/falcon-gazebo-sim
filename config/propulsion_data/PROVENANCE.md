# APC 13x6.5E performans verisi - köken kaydı

- **KAYNAK:** APC Propellers resmî performans verisi
- **MODEL:** 13x6.5E
- **DOSYA (ham, değiştirilemez):** `PER3_13x65E.dat`
- **URL:** https://www.apcprop.com/files/PER3_13x65E.dat
- **Dosya sürümü:** v2022-0915, simülasyon 2022-09-22
- **Alınma:** 2026-08-23. Bağımsız bir statik thrust tablosuna karşı
  doğrulandı: RPM = 5000/6000/9000/10000 statik (J=0, V=0 mph) satırları
  birebir uyuşuyor.

## Ham -> parse edilmiş ilişkisi

`PER3_13x65E.dat` hiçbir zaman elle düzenlenmez. `apc_13x65e_parsed.csv`,
aynı dizindeki `parse_apc_dat.py` ile bu dosyadan mekanik olarak üretilir;
yapılan iş salt biçim dönüştürmedir: CSV'deki her `(rpm, J, Ct, Cp, power_W,
torque_Nm, thrust_N)` satırı, ham dosyadaki karşılık gelen satırdan birebir
(float olarak) kopyalanır. Parser hiçbir katsayıyı hesaplamaz, yumuşatmaz,
ekstrapole etmez veya uydurmaz. Projenin SI/köken kurallarına uygun olarak
parser, ham dosyanın kendi SI kolonlarını (`W`, `N-m`, `N`) kullanır —
`Hp`/`In-Lbf`/`Lbf` İngiliz birim kolonlarını hiç okumaz — bu nedenle bu
alanlar için birim dönüşümü gerekmez; `Ct`/`Cp`/`J` ise kaynak dosyada zaten
boyutsuz/SI-uyumludur.

Parse edilmiş CSV'yi ham dosyadan yeniden üretmek için:

```bash
cd config/propulsion_data
python3 parse_apc_dat.py
```

## Parse özeti

- 18 RPM dilimi (1000'den 18000 RPM'e, 1000 adımla - dosyanın tüm aralığı).
- 537 veri satırı parse edildi (nominal 540 = 18 x 30 içinden); 3 son satır
  (3000, 9000 ve 18000 RPM dilimlerinde birer tane) thrust sıfıra yaklaştığında
  APC'nin kendi üreteci tarafından kesilmişti ve uydurulmak yerine doğru biçimde
  atlandı (gerekli 11 sayısal alandan, yani SI Thrust(N) kolonundan yoksun
  oldukları için).
- J değerlerinin 18 dilimin her birinde kesin artan olduğu doğrulandı (dilim
  başına doğrusal interpolasyon yöntemi bunu gerektirir) - bkz.
  `parse_apc_dat.py` içindeki monotonluk kontrolünün çıktısı.
- Referans statik (J=0) tabloya karşı noktasal kontrol: RPM=5000 ->
  Ct=0.0901, Cp=0.0309, Thrust=8.975 N; RPM=6000 -> Ct=0.0904, Cp=0.0306,
  Thrust=12.975 N; RPM=9000 -> Ct=0.0919, Cp=0.0303, Thrust=29.664 N;
  RPM=10000 -> Ct=0.0925, Cp=0.0304, Thrust=36.877 N - dördü de
  `apc_13x65e_parsed.csv` içinde birebir yeniden üretiliyor.

## Bilinen bulgu - parse hatası DEĞİL, sessizce yutulmayıp raporlanmıştır

Parse edilmiş Ct/Cp değerlerinden, projenin zorunlu kıldığı formüllerle
(`T = Ct*rho*n^2*D^4`, `P = Cp*rho*n^3*D^5`, `D = 0.3302 m`,
`rho = 1.225 kg/m^3`) boyutlu thrust/güç geri hesaplandığında, ham dosyanın
kendi `Thrust(N)`/`PWR(W)` kolonları **birebir** elde edilmiyor: küçük ve
sistematik bir sapma var (`T_calc/T_ref` ~1.015, `P_calc/P_ref` ~1.019; dört
statik referans noktasının tamamında tutarlı). İki oran, nominal 0.3302 m'den
~%0.4 küçük bir efektif çap ile kendi içinde tutarlıdır
(`1.015^(5/4) = 1.0185`, gözlenen güç oranıyla neredeyse tam örtüşür). Bu,
APC'nin bu dosyayı üreten kendi iç performans aracının, Ct/Cp'yi kendi ham
thrust/güç değerlerinden türetirken bu projenin zorunlu nominal
değerlerinden biraz farklı bir efektif çap ve/veya hava yoğunluğu kullandığını
düşündürüyor. Projenin kuralı gereği (`D = 0.3302 m` HER ZAMAN, istisnasız) bu
fark kapatılmak için D (veya rho) ayarlanmaz; projenin fiilen tükettiği
boyutsuz katsayılar olan `Ct(J)`/`Cp(J)` tabloda yazıldığı gibi, değiştirilmeden
kullanılır. Statik doğrulama kontrolleri bu nedenle birebir eşitlik değil,
belgelenmiş bir tolerans ile yapılır.

#!/usr/bin/env python3
# =============================================================================
# FALCON V2 — APC 13x6.5E performans verisi ayrıştırıcısı (ham .dat -> CSV)
# =============================================================================
# GİRDİ (değiştirilemez, elle düzenlenmez):
#   config/propulsion_data/PER3_13x65E.dat
#   KAYNAK: APC Propellers resmi performans verisi
#   MODEL:  13x6.5E
#   URL:    https://www.apcprop.com/files/PER3_13x65E.dat
#   Dosya sürümü: v2022-0915, simülasyon 2022-09-22. Alınma: 2026-08-23.
#
# ÇIKTI (üretilmiş, makine okunur; bu betik yeniden çalıştırılarak girdiden
# yeniden oluşturulabilir — o da elle düzenlenmez):
#   config/propulsion_data/apc_13x65e_parsed.csv
#
# BETİĞİN YAPTIĞI İŞ: salt mekanik biçim dönüştürme. Hiçbir yumuşatma
# (smoothing), hiçbir dışdeğerleme (extrapolation), hiçbir eğri uydurma (curve
# fitting) ve hiçbir katsayı hesabı yapılmaz. Çıktıdaki her
# (rpm, J, Ct, Cp, power_W, torque_Nm, thrust_N) satırı, ham dosyadaki karşılık
# gelen satırdan birebir (float olarak) kopyalanır. Projenin SI/köken
# kurallarına uygun olarak dosyanın kendi SI kolonları (W, N-m, N) kullanılır;
# Hp/In-Lbf/Lbf İngiliz birim kolonları hiç okunmaz, dolayısıyla bu alanlar için
# birim dönüşümü ne yapılır ne de gerekir.
#
# DOSYA BİÇİMİ (ham dosyadan çözümlenmiştir; APC makine okunur bir şema
# yayınlamadığı için burada belgelenir):
#   - CRLF satır sonları.
#   - İlk "PROP RPM =" işaretine kadar serbest metin başlık bloğu - atlanır.
#   - Dosya 18 RPM dilimine ayrılmıştır (1000..18000, 1000 adımla); her dilim
#     "PROP RPM =     <rpm>" satırıyla başlar.
#   - Her dilimde 2 satırlık kolon başlığı ("V  J  Pe  Ct  Cp  PWR  Torque
#     Thrust  PWR  Torque  Thrust  THR/PWR  Mach  Reyn  FOM" ve ardından birim
#     satırı) bulunur; sonra simüle edilen her hava hızı için bir veri satırı
#     gelir (önce V=0, yani statik/J=0 durumu, sonra artan V/J).
#   - Her veri satırı boşlukla ayrılmış 15 sayısal alan içerir:
#       0 V(mph)  1 J  2 Pe  3 Ct  4 Cp  5 PWR(Hp)  6 Torque(In-Lbf)
#       7 Thrust(Lbf)  8 PWR(W)  9 Torque(N-m)  10 Thrust(N)  11 THR/PWR(g/W)
#       12 Mach  13 Reyn  14 FOM
#   - Bir dilimin SON satırı, itki sıfıra/negatife yaklaştığında APC'nin kendi
#     üretecinin kestiği hâlde gelebilir (örn. 18000 RPM diliminin son
#     satırında yalnızca V ve J vardır). Bu betik 11'den az sayısal belirteç
#     içeren (yani SI Thrust(N) kolonu, indeks 10, eksik olan) satırları eksik
#     alanları uydurmak yerine atlar; atlanan satır sayısı izlenebilirlik için
#     yazdırılır.
#   - J değerleri bir dilim içinde kesin artandır (kaynak tabloda V monotonik
#     artar ve dilim içinde n sabit olduğundan J=V/(nD)). Tüketici tarafın dilim
#     başına doğrusal interpolasyonu buna dayanır; betik bunu doğrular (aşağıdaki
#     monotonluk kontrolü).
# =============================================================================
import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RAW_PATH = HERE / "PER3_13x65E.dat"
OUT_PATH = HERE / "apc_13x65e_parsed.csv"

MIN_FIELDS = 11  # en az 10. kolona (Thrust, N) kadar alan gerekir


def parse(raw_path: Path):
    rows = []
    current_rpm = None
    skipped_incomplete = 0
    slice_counts = {}

    with open(raw_path, "r", encoding="ascii", errors="replace", newline=None) as f:
        for line in f:
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("PROP RPM"):
                # Biçim: "PROP RPM =       1000"
                current_rpm = float(stripped.split("=")[1].strip())
                slice_counts.setdefault(current_rpm, 0)
                continue

            tokens = stripped.split()
            if len(tokens) < MIN_FIELDS:
                # Başlık/birim satırları da buraya düşer ve float()
                # ayrıştırması tarafından doğru biçimde reddedilir; bu uzunluk
                # kontrolü, bazı dilimlerdeki kesilmiş son satırları tek başına
                # zaten ayıklar.
                if current_rpm is not None and tokens and _looks_like_row(tokens):
                    skipped_incomplete += 1
                continue
            try:
                vals = [float(t) for t in tokens[:15]]
            except ValueError:
                continue  # başlık/birim satırı, veri satırı değil
            if current_rpm is None:
                continue

            row = {
                "rpm": current_rpm,
                "V_mph": vals[0],
                "J": vals[1],
                "Ct": vals[3],
                "Cp": vals[4],
                "power_W": vals[8],
                "torque_Nm": vals[9],
                "thrust_N": vals[10],
            }
            rows.append(row)
            slice_counts[current_rpm] = slice_counts.get(current_rpm, 0) + 1

    return rows, skipped_incomplete, slice_counts


def _looks_like_row(tokens):
    """İlk belirteç float olarak ayrıştırılıyorsa True döner: yani bu satır
    metin/başlık satırı değil, (kesilmiş olsa bile) gerçek bir veri satırıdır."""
    try:
        float(tokens[0])
        return True
    except ValueError:
        return False


def check_monotonic_j(rows):
    by_rpm = {}
    for r in rows:
        by_rpm.setdefault(r["rpm"], []).append(r["J"])
    bad = []
    for rpm, js in by_rpm.items():
        for a, b in zip(js, js[1:]):
            if not (b > a):
                bad.append((rpm, a, b))
    return bad


def main():
    if not RAW_PATH.exists():
        print(f"ERROR: raw source file not found: {RAW_PATH}", file=sys.stderr)
        return 1

    rows, skipped, slice_counts = parse(RAW_PATH)

    bad_monotonic = check_monotonic_j(rows)

    with open(OUT_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["rpm", "V_mph", "J", "Ct", "Cp", "power_W", "torque_Nm", "thrust_N"]
        )
        for r in rows:
            writer.writerow(
                [
                    r["rpm"],
                    r["V_mph"],
                    r["J"],
                    r["Ct"],
                    r["Cp"],
                    r["power_W"],
                    r["torque_Nm"],
                    r["thrust_N"],
                ]
            )

    print(f"Parsed {len(rows)} data rows across {len(slice_counts)} RPM slices "
          f"from {RAW_PATH.name}")
    print(f"Skipped {skipped} incomplete/truncated trailing row(s)")
    for rpm in sorted(slice_counts):
        print(f"  RPM={rpm:>6.0f}  rows={slice_counts[rpm]}")
    if bad_monotonic:
        print(f"WARNING: {len(bad_monotonic)} non-monotonic J step(s) found: "
              f"{bad_monotonic[:5]}{' ...' if len(bad_monotonic) > 5 else ''}")
    else:
        print("J monotonicity check: OK (strictly increasing within every slice)")
    print(f"Wrote {OUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
# =============================================================================
# FALCON V2 - APC 13x6.5E performance-data parser (raw .dat -> structured CSV)
# =============================================================================
# Owner: propulsion specialist agent. Task: PROPULSION_V1_IMPLEMENTATION
# (2026-08-23).
#
# INPUT (immutable, never hand-edited):
#   docs/source_of_truth/propulsion/data/PER3_13x65E.dat
#   SOURCE: APC Propellers official performance data
#   MODEL:  13x6.5E
#   URL:    https://www.apcprop.com/files/PER3_13x65E.dat
#   File version: v2022-0915, simulated 2022-09-22. Retrieved 2026-08-23.
#
# OUTPUT (generated, machine-readable, re-creatable from the input by
# re-running this script - never hand-edited either):
#   docs/source_of_truth/propulsion/data/apc_13x65e_parsed.csv
#
# WHAT THIS SCRIPT DOES: pure mechanical reformatting. It does NOT compute,
# smooth, extrapolate, or invent any coefficient. Every (rpm, J, Ct, Cp,
# power_W, torque_Nm, thrust_N) row in the output is copied verbatim (as
# floats) from a corresponding row in the raw file. Per the project's SI/
# provenance rules, this parser deliberately uses the file's own SI columns
# (W, N-m, N) for thrust/torque/power - never the Hp/In-Lbf/Lbf imperial
# columns - so no unit conversion is performed or required for those fields.
#
# FILE FORMAT (reverse-engineered from the raw file, documented here since
# APC does not ship a machine-readable schema):
#   - CRLF line endings.
#   - A free-text header block (title/version/date/definitions) before the
#     first "PROP RPM =" marker - ignored.
#   - The file is divided into 18 RPM slices (1000..18000 step 1000), each
#     introduced by a line "PROP RPM =     <rpm>".
#   - Each slice has a 2-line column header ("V  J  Pe  Ct  Cp  PWR  Torque
#     Thrust  PWR  Torque  Thrust  THR/PWR  Mach  Reyn  FOM" then a units
#     line) followed by data rows, one per simulated airspeed point (V=0
#     first, i.e. the static/J=0 condition, then increasing V/J).
#   - Each data row has 15 whitespace-separated numeric fields:
#       0 V(mph)  1 J  2 Pe  3 Ct  4 Cp  5 PWR(Hp)  6 Torque(In-Lbf)
#       7 Thrust(Lbf)  8 PWR(W)  9 Torque(N-m)  10 Thrust(N)  11 THR/PWR(g/W)
#       12 Mach  13 Reyn  14 FOM
#   - A slice's LAST row is sometimes truncated by APC's own generator once
#     thrust approaches zero/negative (e.g. the 18000 RPM slice's final row
#     has only V and J, no further columns). This parser detects and skips
#     any row with fewer than 11 numeric tokens (i.e. missing the SI
#     Thrust(N) column at index 10) rather than fabricating the missing
#     fields - a dropped-row count is printed for traceability.
#   - J values are strictly increasing within a slice (V increases
#     monotonically in the source table, and J=V/(nD) with n fixed within a
#     slice) - relied upon by the consumer's per-slice linear interpolation,
#     verified by this script (see the monotonicity check below).
# =============================================================================
import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RAW_PATH = HERE / "PER3_13x65E.dat"
OUT_PATH = HERE / "apc_13x65e_parsed.csv"

MIN_FIELDS = 11  # need at least through column 10 (Thrust, N)


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
                # Format: "PROP RPM =       1000"
                current_rpm = float(stripped.split("=")[1].strip())
                slice_counts.setdefault(current_rpm, 0)
                continue

            tokens = stripped.split()
            if len(tokens) < MIN_FIELDS:
                # Header/units lines (letters, parens, dashes) also land here
                # and are correctly rejected below by the float() parse -
                # this length check alone already screens out the truncated
                # trailing rows some slices have.
                if current_rpm is not None and tokens and _looks_like_row(tokens):
                    skipped_incomplete += 1
                continue
            try:
                vals = [float(t) for t in tokens[:15]]
            except ValueError:
                continue  # header/units line, not a data row
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
    """True if the first token parses as a float (i.e. this was a real,
    if truncated, data row rather than a text/header line)."""
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

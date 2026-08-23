# APC 13x6.5E performance data - provenance

- **SOURCE:** APC Propellers official performance data
- **MODEL:** 13x6.5E
- **FILE (raw, immutable):** `PER3_13x65E.dat`
- **URL:** https://www.apcprop.com/files/PER3_13x65E.dat
- **File version:** v2022-0915, simulated 2022-09-22
- **Retrieved:** 2026-08-23, by the FALCON V2 project coordinator (verified
  independently against this task's reference static-thrust table before
  the `propulsion` agent began work: RPM=5000/6000/9000/10000 static (J=0,
  V=0 mph) rows match exactly - see `parse_apc_dat.py` output / the
  `propulsion` agent's task report for the cross-check).

## Raw -> parsed relationship

`PER3_13x65E.dat` is never hand-edited. `apc_13x65e_parsed.csv` is
mechanically generated from it by `parse_apc_dat.py` (same directory) - a
pure reformatting pass: every `(rpm, J, Ct, Cp, power_W, torque_Nm,
thrust_N)` row in the CSV is copied verbatim (as floats) from a
corresponding row in the raw file. No coefficient is computed, smoothed,
extrapolated, or invented by the parser. Per this project's SI/provenance
rules, the parser uses the raw file's own SI columns (`W`, `N-m`, `N`) for
thrust/torque/power - never the `Hp`/`In-Lbf`/`Lbf` imperial columns - so no
unit conversion is required for those fields; `Ct`/`Cp`/`J` are already
dimensionless/SI-consistent in the source file.

To regenerate the parsed CSV from the raw file:

```bash
cd docs/source_of_truth/propulsion/data
python3 parse_apc_dat.py
```

## Parse summary (this pass, 2026-08-23)

- 18 RPM slices (1000 to 18000 RPM, step 1000 - the file's full range).
- 537 data rows parsed (out of a nominal 540 = 18 x 30); 3 trailing rows
  (one each in the 3000, 9000, and 18000 RPM slices) were truncated by APC's
  own generator once thrust approached zero and were correctly dropped
  (fewer than the required 11 numeric fields, including the SI Thrust(N)
  column) rather than fabricated.
- J values confirmed strictly increasing within every one of the 18 slices
  (required for the project's specified per-slice linear interpolation
  method) - see `parse_apc_dat.py`'s own monotonicity check output.
- Spot-check against this task's reference static (J=0) table: RPM=5000 ->
  Ct=0.0901, Cp=0.0309, Thrust=8.975 N; RPM=6000 -> Ct=0.0904, Cp=0.0306,
  Thrust=12.975 N; RPM=9000 -> Ct=0.0919, Cp=0.0303, Thrust=29.664 N;
  RPM=10000 -> Ct=0.0925, Cp=0.0304, Thrust=36.877 N - all four reproduced
  exactly in `apc_13x65e_parsed.csv`.

## Known finding - NOT a parsing bug, reported not silently absorbed

Reconstructing dimensional thrust/power from the parsed Ct/Cp via this
project's mandated formulas (`T = Ct*rho*n^2*D^4`, `P = Cp*rho*n^3*D^5`,
`D = 0.3302 m`, `rho = 1.225 kg/m^3` - both CONFIRMED/mandated,
`PROPULSION.md` sec 0/2) does **not** exactly reproduce the raw file's own
tabulated `Thrust(N)`/`PWR(W)` columns - there is a small, systematic offset
(`T_calc/T_ref` ~1.015, `P_calc/P_ref` ~1.019, consistent across all four
static reference points). The two ratios are self-consistent with a ~0.4%
smaller effective diameter than the nominal 0.3302 m
(`1.015^(5/4) = 1.0185`, matching the observed power ratio almost exactly),
suggesting APC's own internal performance-sim tool for this specific file
used a slightly different effective diameter and/or air density than this
project's mandated nominal values when it originally derived Ct/Cp from its
own raw thrust/power. Per `CLAUDE.md`/`PROPULSION.md` sec 0
("`D = 0.3302 m` ALWAYS ... no exceptions") this project does **not** adjust
D (or rho) to close this gap - `Ct(J)`/`Cp(J)` themselves (the dimensionless
coefficients this project actually consumes) are used exactly as tabulated,
unmodified. See `plugins/propulsion/test/propulsion_model_selftest.cc`'s
`APC_STATIC_*_TEST` cases for the documented tolerance this finding
requires, and this task's final report for the full write-up.

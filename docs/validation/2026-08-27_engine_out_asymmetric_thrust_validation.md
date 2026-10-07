# ENGINE_OUT_AND_ASYMMETRIC_THRUST_VALIDATION — 2026-08-27

> **Not.** Bu dosya projenin iç doğrulama kaydından alınmıştır ve teknik
> içeriği değiştirilmeden korunmuştur. Metinde atıf yapılan `tests/gazebo/`
> test koşum altyapısı ile `docs/source_of_truth/` mühendislik veri ağacı bu
> repository ile dağıtılmaz; o yollar tarihsel kayıt olarak bırakılmıştır.

## 1. Method
Test execution, domain analysis and engineering review ran as three separate
passes: the stage that produced a result never signed off on it, and every
numeric claim below was independently recomputed from the raw captured data
before being accepted. Files written by this stage are listed in §2.

## 2. Files created/modified
- `tests/gazebo/scripts/test_engine_out_asymmetric_thrust.py` (new; later parameterized with a `--followup` mode)
- `tests/gazebo/results/engine_out_asymmetric_thrust_{result.json,log.txt}` (new; updated in place with the extended-flight follow-up data and a decimated RPM series)
- `tests/gazebo/README.md` (two dated entries: initial run + follow-up)
- `docs/source_of_truth/aerodynamics/AERODYNAMICS.md` (new §23, subsections 23.1–23.4)
- This report.
- **Confirmed untouched**: propulsion constants (motor KV/resistance, APC Ct/Cp, rotor inertia, hub geometry, reaction-torque model), aero coefficients, control lookup tables, rudder derivatives, actuator parameters, CG, inertia, mass.

## 3. Analytic asymmetric-thrust sign expectation (pre-registered BEFORE any test)
Derived from actual hub geometry (`left_hub_m=[0.2951,+0.30,0.1271]`, `right_hub_m=[0.2951,-0.30,0.1271]`, `thrust_axis_body=[1,0,0]`) and rotation-sign config (`rotationSignLeft=+1`, `rotationSignRight=-1`): (1) LEFT-engine-out → positive Mz (nose yaws toward the dead/left engine — standard multi-engine aviation fact); RIGHT-engine-out → mirror. (2) LEFT-engine-out needs positive rudder deflection (toward the operating engine — "rudder toward the good engine"); RIGHT-engine-out → mirror. (3) LEFT-engine-out → net positive reaction-torque roll (Mx); RIGHT-engine-out → mirror. **All 3/3 confirmed** — live measurement: LEFT_OUT Mz_prop=+0.8346 N·m, Mx_reaction=+0.1306 N·m, exact mirror for RIGHT_OUT. the independent review independently redid the r×F calculation from live thrust (0.30×2.7818N=0.834616) and matched.

## 4. Static differential-thrust results
7 cases tested (symmetric baseline; mild/strong differential both directions; both engine-out cases), rudder neutral. Live-plugin diagnostics used throughout (not a script-only mirror) — cross-checked against an independent Python Ct/Cp mirror, agreement <0.05%.

## 5. Left/right engine-out symmetry
Mirror-symmetric to <1e-6 N·m across all differential/engine-out pairs — no unexplained asymmetry.

## 6. Engine-out transient response
Real motor spool-down (no instant RPM zeroing): 6087→428 RPM over ~5s, max single-tick drop 229 RPM (independently re-verified at 228.87 RPM from a persisted 250-point decimated series after the independent review flagged the original artifact as unauditable). Yaw/roll onset direction matched the pre-registered signs, mirror-antisymmetric.

## 7. Rudder compensation required per side
Nominal 18.166 m/s: LEFT_OUT +17.10°, RIGHT_OUT −17.00° (residual Mz ≈0.0006–0.0013 N·m) — symmetric to 0.1°. 16 m/s ≈21.5°, 24 m/s ≈13.1° — decreasing with speed as expected (qbar scaling), mirror-symmetric.

## 8. Surviving-engine throttle/RPM/thrust requirement
Previous nominal throttle (0.5010) insufficient (T/D=0.54). Increased to 0.5614: confirmed sufficient (T/D≈0.9995), RPM at 58% of the 11500 cap, current at 29% of the 65A limit — comfortable margin, no `PROPULSION_LIMITED` or `YAW_AUTHORITY_LIMITED`.

## 9. Single-engine trim result
Lift/Weight=0.992, T/D≈0.96, My≈−0.18 N·m, Mz≈0.001 N·m, Mx≈±0.234 N·m residual (aileron deliberately left neutral per the rudder-only-first test design — not a control-authority shortfall; independently confirmed by the independent review that only ≈0.17° of aileron would null it, using `Clda_per_rad=0.414/rad`).

## 10. Single-engine free-flight result
Initial ~13s window: `PASS_WITH_DRIFT`, no NaN — but roll was still at its maximum at the cutoff, an inconclusive snapshot. A follow-up extended run to ~29.1s resolved this: roll genuinely converges to a steady ~21.4–21.8° bank angle with roll rate decaying toward zero (`NEGLIGIBLE_RATE_SETTLED`), and yaw rate levels off at a steady, non-accelerating ~9–10°/s (`RATE_LEVELED_OFF_STEADY_NONZERO`) — the aircraft settles into a steady, mildly-banked turn under rudder-only compensation, **not a spiral divergence**. Mirror-symmetric both sides. Independently re-verified by the independent review directly from the raw JSON data (not just the summary claim).

## 11. Low/nominal/high-speed engine-out comparison
Only the rudder-compensation requirement was measured at 16/24 m/s (by design, minimal): 21.5° / 13.1° vs. nominal's 17.0–17.1°, tracking qbar scaling correctly. No full engine-out trim or free-flight run exists at these two speeds — an evidentiary gap, not a demonstrated concern.

## 12. Rudder authority margin
At nominal speed: 10°/15°/25° of rudder supply ≈58%/88%/147% of the ≈1.543 N·m engine-out propulsive yawing moment; full compensation at ≈17.0–17.1°, leaving ~28° of reserve to the ±45° mechanical limit. No `LOW_RUDDER_MARGIN` (>25° needed) or `RUDDER_SATURATION_RISK` at any tested speed (max requirement 21.5° at 16 m/s). This is a Gazebo-model-only figure, not a certification-grade margin.

## 13. Reaction-torque contribution
Isolated via a pinned (β=0, p=q=r=0) static hold in Part 1: LEFT_OUT Mx_reaction=+0.1306 N·m, exact mirror for RIGHT_OUT — cleanly distinguished from the separate aerodynamic roll coupling (Clb·beta, Clr·rHat) that develops once the aircraft is released into free flight. The reaction-torque term remains the dominant Mx source through trim and free-flight.

## 14. Actuator/propulsion limiting factors
None found. Zero clamp flags, minimal tracking error throughout all rudder tests, no RPM/current cap engaged even at the increased surviving-engine throttle (58%/29% of caps at the highest tested load).

## 15. Cl_delta_r status
Unchanged, `STILL_INCONCLUSIVE`, +0.0007/rad. Not used to resolve the sign question this stage (engine-out roll response is dominated by reaction torque + Clb·beta + Clr·rHat, not an isolated Cldr measurement) — no new decisive evidence claimed, per policy.

## 16. ENGINE_OUT_VALIDATED_REGION
18.166 m/s (nominal) — the full chain (rudder search, throttle adequacy, trim, and now a genuinely-demonstrated ~29s bounded steady-turn free-flight) converges cleanly and symmetrically, with no NaN and a well-explained, non-divergent roll/yaw residual.

## 17. ENGINE_OUT_PROVISIONAL_REGION
16 m/s and 24 m/s — clean rudder-authority math with comfortable margin, but no engine-out-specific trim or free-flight run exists at either speed.

## 18. ENGINE_OUT_NOT_MAINTAINABLE
None — no tested speed reached this classification.

## 19. Validation findings
One MAJOR finding raised and resolved within this stage: the nominal-speed classification initially rested on a non-converged 13s free-flight snapshot (roll still at its maximum at cutoff) — the aerodynamic analysis' "bounded, not diverging" framing outran the evidence. Resolved with a genuine ~29.1s extended run showing real convergence (roll rate → 0, yaw rate leveling off), independently re-verified by the independent review directly from the JSON, plus an `AERODYNAMICS.md` §23.4 addendum citing the upgraded evidence (additive, original §23.1–23.3 left unrewritten per this project's established addendum convention). Two MINOR items carried forward, non-blocking: raw per-tick Part 7 rate series still not persisted (routed to the simulation-test stage); §23.4 doesn't quantify the cumulative heading change during the steady turn (routed to the aerodynamic analysis). All other 9 verification items (hub-geometry sign math, live-not-mirror thrust, dynamic motor decay, left/right symmetry, rudder-sign convention, actual joint position used, genuine 6-DOF free-flight, no coefficient retuning, reaction-torque/aero-coupling separation) **PASS**. No CRITICAL findings. **Final verdict: READY.**

## 20. Remaining limitations
- `Cl_delta_r` sign remains unresolved (unchanged, +0.0007/rad) — carried forward, not blocking.
- 16 m/s and 24 m/s lack direct engine-out trim/free-flight validation (rudder-requirement data only).
- Nominal engine-out settles into a steady ~21° banked turn under rudder-only compensation, not level flight — a small aileron trim (~0.17°, computed and independently cross-checked) would plausibly null the residual roll if ever added; not implemented here by design (rudder-only-first test).
- EMAX ES3054 servo rail voltage remains `DATA_REQUIRED`; actuator parameters remain `V1_PROVISIONAL`.
- Beta-derivative (CYb/Clb/Cnb) saturation gap remains open from an earlier stage (relevant here since the settled steady turn involves nonzero beta).
- Two MINOR persistence/documentation gaps noted in §19.

## 21. Next recommended stage
`WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION` (not started).

---

**ENGINE_OUT_AND_ASYMMETRIC_THRUST_VALIDATION_READY**

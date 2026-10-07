# FLIGHT_ENVELOPE_VALIDATION — 2026-08-27

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
- `tests/gazebo/scripts/test_flight_envelope.py` (new)
- `tests/gazebo/results/flight_envelope_{result.json,log.txt}` (new)
- `tests/gazebo/README.md` (dated entry appended, later corrected for a pitch-number prose inconsistency)
- `docs/source_of_truth/aerodynamics/AERODYNAMICS.md` (§11.1 new — 12.5 m/s architecture note; §22 new — persisted envelope classification)
- This report.
- **Confirmed untouched**: every aero coefficient, lookup table, high-alpha limiter constant, actuator parameter, propulsion parameter, inertia, CG, mass (git diff/mtime-checked by the independent review).

## 3. Compact trim-vs-speed table (115 trim candidates tested, aggregate)

| V(m/s) | alpha(°) | throttle | elev_aero(°) | CL | CD | L/W | Thrust(N) | RPM | My(N·m) | Tier |
|---|---|---|---|---|---|---|---|---|---|---|
| 12.5 | 11.40 | 0.390 | −9.12 | 1.340 | 0.130 | 0.986 | 5.62 | 4724 | +1.550 | **OUTSIDE_VALIDATED_ENVELOPE** |
| 14.0 | 7.82 | 0.415 | −14.97 | 1.072 | 0.098 | 0.988 | 5.29 | 5032 | −0.549 | PROVISIONAL_EDGE_ENVELOPE |
| 16.0 | 4.74 | 0.448 | −9.49 | 0.819 | 0.071 | 0.986 | 4.88 | 5450 | −0.611 | PROVISIONAL_EDGE_ENVELOPE |
| 18.166 | 2.47 | 0.501 | −4.45 | 0.640 | 0.056 | 0.992 | 5.56 | 6087 | −0.337 | **VALIDATED_CORE_ENVELOPE** |
| 21.0 | 0.53 | 0.567 | −1.14 | 0.479 | 0.047 | 0.992 | 6.18 | 6876 | −0.411 | PROVISIONAL_EDGE_ENVELOPE |
| 24.0 | −0.84 | 0.632 | +2.16 | 0.373 | 0.043 | 1.009 | 6.56 | 7670 | +0.095 | VALIDATED_CORE_ENVELOPE* |
| 28.0 | −2.02 | 0.734 | +4.43 | 0.277 | 0.040 | 1.019 | 8.40 | 8879 | +0.392 | PROVISIONAL_EDGE_ENVELOPE |
| 30.0 | −2.45 | 0.787 | +5.34 | 0.243 | 0.040 | 9.48 | 9491 | +0.645 | PROVISIONAL_EDGE_ENVELOPE |

*24.0 m/s: the independent review flags this tier placement as slightly generous — the underlying raw search classifier only reached `MARGINAL_TRIM` (not `TRIM_FOUND`) for this point; the upgrade to top-tier rests on real 14s free-flight corroboration (see §6), which is solid evidence but the tier label vs. the raw reconciliation is a carried-forward MINOR/INFO inconsistency, not a blocking defect.

## 4. Lowest successful trim speed: **14 m/s** (PROVISIONAL_EDGE_ENVELOPE)
12.5 m/s is `NO_VALID_TRIM`/`OUTSIDE_VALIDATED_ENVELOPE` — see §9.

## 5. Highest tested successful trim speed: **30 m/s** (PROVISIONAL_EDGE_ENVELOPE)
No propulsion or actuator limit encountered even at this point (RPM 9491/11500=82% of cap, current 23.4/65A=36% of limit).

## 6. Free-flight summaries (3 runs, ~14s each, fully free 6-DOF, no autopilot)

| Label | V | Altitude drift | Pitch: start→end | Pitch peak (max|pitch|) | Max|alpha| | Classification |
|---|---|---|---|---|---|---|
| LOW (14 m/s) | 13.6–14.0 | 98.5→90.9m | +0.18°→−5.65° | **11.95°** | 9.23° (at `alpha_transition=9.25°`) | PASS |
| NOMINAL (18.166 m/s) | 17.8–18.2 | 99.3→102.5m | +0.26°→−4.24° | 6.06° | 2.75° | PASS |
| HIGH (24 m/s) | 24.0–24.7 | 100.2→88.8m | +0.42°→+2.94° | 4.39° | 1.54° | PASS |

All: `any_nan=False`, roll≤0.09°, no divergence. (Note: the LOW run's peak pitch swing, 11.95°, is materially larger than its start/end values suggest — a mid-flight phugoid-like transient, still well within the ≤15° PASS threshold, but worth flagging since 14 m/s's max alpha sits essentially at the high-alpha transition boundary.)

## 7. Control authority vs speed (representative ±5° command, 3 surfaces × 3 speeds)
Dimensional moments grow with dynamic pressure: aileron ΔMx = 2.80 → 4.80 → 9.50 N·m (14 → 18.166 → 24 m/s), tracking qbar∝V² once actual achieved deflection is accounted for (independently confirmed by the independent review: qbar ratio 14→24 m/s = 2.94×, matches V² exactly; the raw ΔMx ratio of 3.39× is explained by the achieved aileron deflection also growing 4.90°→7.20° plus mild lookup nonlinearity — a qbar·ΔCl reconstruction lands within 4% of the measured ratio). Coefficient-level response (ΔCl/ΔCm/ΔCn) stays roughly flat across speed (e.g. aileron ΔCl: 0.0239/0.0247/0.0286, <20% variation) — confirms the growth is expected dynamic-pressure physics, not a derivative change.

## 8. Actuator/propulsion limiting regions: **none found**
Zero `ACTUATOR_LIMITED_REGION` flags at any of the 8 trim points or the 9 high-alpha static points (tracking error ≤0.048° everywhere, improving with speed). Zero `PROPULSION_LIMITED` flags — no RPM safety cap (max 82% of cap at 30 m/s) or current clamp (max 36% of limit) engaged anywhere.

## 9. Model-predicted low-speed/high-alpha boundary
The model's trim search is moment-**infeasible** at 12.5 m/s (`AERODYNAMIC_NO_TRIM_MOMENT`: My=+1.55 N·m residual, ~2.4× the next-largest; mean|q|=7.4°/s during release vs ≤2.7°/s everywhere else) despite CL=1.340 being individually reachable on the no-post-stall saturation curve — confirmed as an expected, self-consistent architecture consequence (the elevator's own coupled dCm/dCL lookup terms leave no self-consistent (alpha, delta_e) solution once alpha is 2.15° past `alpha_transition=9.25°`), not a new bug (`AERODYNAMICS.md` §11.1). The **model-predicted practical low-speed limit is ≈14 m/s**. This is explicitly distinct from the real-aircraft reference already in the source of truth (`Vstall≈12.24 m/s`, `Vmin_safe≈14.69 m/s` with 20% margin, both manufacturer-CLmax-based, cited not re-derived) — their numerical closeness to the model's own ~14 m/s limit is coincidental agreement between two unrelated derivations, not cross-validation of either, and must not be read as validating real-aircraft stall behavior (`AERODYNAMICS.md` §22.2). A 9-point static high-alpha check (alpha=7°→20°, V=15 m/s) confirmed the limiter itself remains C1-continuous, finite, and NaN-free well beyond any alpha reached in this stage's trims.

## 10. VALIDATED_CORE_ENVELOPE
18.166 m/s (reference trim, independently re-validated). 24.0 m/s (strong free-flight corroboration, but see the tier-placement caveat in §3/§13).

## 11. PROVISIONAL_EDGE_ENVELOPE
14, 16, 21, 28, 30 m/s — all aerodynamically/propulsion/actuator self-consistent with no real limit hit, but each either lacks direct free-flight corroboration this stage (16/21/28/30) or sits at the edge of the high-alpha reliability boundary (14 m/s, max alpha≈9.23° vs. transition at 9.25°).

## 12. OUTSIDE_VALIDATED_ENVELOPE
12.5 m/s — moment-infeasible trim, at/below the real-aircraft stall-speed references, and only mathematically "liftable" due to the model's known lack of post-stall physics.

## 13. Validation findings
No CRITICAL defect. One MAJOR **process** finding, resolved within this stage: the three-tier classification was initially only reported in chat, not persisted as a reviewable artifact — fixed by the aerodynamic analysis adding `AERODYNAMICS.md` §22, then independently re-confirmed by the independent review (content match, doc-only diff). Two MINOR/INFO items carried forward, non-blocking: (a) the 24.0 m/s `VALIDATED_CORE_ENVELOPE` tier placement is arguably a notch more generous than the raw search classifier's own `MARGINAL_TRIM` result, resting mainly on real free-flight corroboration — flagged for awareness, not disputed as wrong; (b) this and the prior several stages remain uncommitted in one working tree, which weakens git-diff-only auditing going forward — a process/hygiene note for the user, not a data defect. One README prose inconsistency (pitch range description vs. the JSON's actual peak value) was found and fixed. **Final verdict: READY.**

## 14. Remaining limitations / DATA_REQUIRED
- `Cl_delta_r` sign remains unresolved (carried forward unchanged, +0.0007/rad) — envelope testing produced no new decisive evidence, as expected/accepted per this stage's own policy.
- No stall/post-stall aerodynamic model exists (V1 architecture limitation, `AERODYNAMICS.md` §11/§11.1) — the ~14 m/s model-predicted limit is a simulation artifact, not validated real-aircraft performance.
- Beta-derivative (CYb/Clb/Cnb) saturation gap remains open from the prior stage (MINOR, unaffected by this stage's small-beta-anchored results).
- EMAX ES3054 servo rail voltage remains `DATA_REQUIRED`; actuator parameters remain `V1_PROVISIONAL`.
- 16/21/28/30 m/s lack direct free-flight corroboration this stage (self-consistent trim only) — a natural target for deeper testing if the envelope needs tightening later.
- The repository has several stages of uncommitted work accumulated in one working tree — recommend committing in logical chunks before further stages compound the diff.

## 15. Next recommended stage
`ENGINE_OUT_AND_ASYMMETRIC_THRUST_VALIDATION` (not started).

---

**FLIGHT_ENVELOPE_VALIDATION_READY**

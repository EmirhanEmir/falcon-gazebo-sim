# WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION — 2026-08-27

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
- `plugins/wind/{WindModel.hh,WindSystem.hh,WindSystem.cc,CMakeLists.txt,README.md,test/wind_model_selftest.cc}` (new plugin)
- `docs/source_of_truth/environment/WIND.md` (new docs)
- `model/model.sdf` (one additive `<plugin>` block for `FalconV2Wind`, nothing else touched)
- `tests/gazebo/scripts/test_wind_gust_disturbance.py` (new)
- `tests/gazebo/results/wind_gust_disturbance_{result.json,log.txt}` (new)
- `tests/gazebo/README.md` (dated entry appended, later corrected for two wording issues)
- This report.
- **Confirmed untouched**: `plugins/aerodynamics/`, `plugins/propulsion/`, `plugins/actuators/`, every `*_config.yaml`.

## 3. Wind convention
`V_wind_world` = velocity of the air mass itself in world frame (m/s) — never meteorological "wind coming from" phrasing. Relative airflow (already pre-existing, unchanged, in both `AerodynamicsSystem.cc` and `PropulsionSystem.cc`): `V_rel = V_something_world − V_wind_world`, then rotated into body frame. The new plugin only ever publishes a velocity vector on the existing `/model/falcon_v2/wind` topic (`gz.msgs.Vector3d`) — it never calls `AddWorldForce`/`AddWorldWrench` (confirmed by three independent greps: the aerodynamic analysis, the simulation-test stage, and the independent review).

## 4. Gust profile
Deterministic 1-cosine bump, one gust scheduled at a time (V1 simplification, documented): for `t ∈ [t_start, t_start+duration]`, `gust_vec(t) = dir_unit · amplitude · 0.5(1−cos(2π(t−t_start)/duration))`, zero outside that window. C1-continuous at both boundaries (value and derivative exactly zero) — no jerk discontinuity, confirmed analytically and numerically in the plugin's own self-test (12/12 pass) and independently re-confirmed by the independent review. Commanded via `/model/falcon_v2/wind/gust_cmd` (`gz.msgs.Double_V`, fixed order `[dir_x,dir_y,dir_z,amplitude_mps,start_delay_s,duration_s]`); steady component via `/model/falcon_v2/wind/steady_cmd` (`gz.msgs.Vector3d`).

## 5. Zero-wind regression: PASS
With the new plugin loaded but wind=[0,0,0] (default), an 8s free-flight snippet stayed exactly zero-wind throughout, and static-hold CL/CD/V/alpha at nominal trim matched the pre-existing 2026-08-26 reference to a few parts in 10⁴ (CL 0.63979 vs 0.63955, CD 0.05639 vs 0.05637, V 18.1656 vs 18.166 m/s, alpha 2.4728° vs 2.473°) — independently re-confirmed by the independent review directly from the JSON.

## 6. Static relative-air sign tests
7 cases (zero/headwind/tailwind/±Y crosswind/±Z vertical), pre-registered analytically before any test from the existing, already-validated `AeroModel.hh` formulas. **14/14 real (non-"?") pre-registered predictions agreed** (corrected from an initial miscount of 12 — the underlying data was always correct, only the reported count was off): headwind increases relative airspeed/qbar, tailwind decreases them; crosswind toward +Y gives beta<0/CY>0/Cn<0 (mirror for −Y); updraft (+Z) gives alpha>0/CL>0 (mirror for −Z). `qbar=0.5·rho·V_rel²` matched to machine precision (the independent review spot-checked 7/7 cases, relative diff 2.9e-14 to 1.2e-7). Cl/roll sign under crosswind was reported but deliberately not pre-registered with confidence, per an already-documented dihedral-coupling derivation caveat in `AeroModel.hh` predating this stage.

## 7. Headwind free-flight result
~13s, fully free 6-DOF, no controller. Ground speed 9.8–18.2 m/s vs. relative airspeed 14.8–23.2 m/s (reported separately, as required) — a clean, consistent +5 m/s offset. Mild climb tendency from the higher qbar at unchanged release attitude (expected lift/weight imbalance, not a bug). Pitch rate q decaying.

## 8. Tailwind free-flight result
Ground speed 18.2–26.0 m/s vs. relative airspeed 13.2–21.0 m/s — clean −5 m/s offset. Mild descent tendency (mirror of headwind's effect, expected). Pitch rate q decaying.

## 9. Crosswind free-flight result
Ground speed 17.3–18.6 vs. relative airspeed 17.3–19.0 m/s (crosswind barely changes airspeed magnitude, as expected — it's a Y-axis effect). Max roll 4.45°, max pitch 7.2°, p/q/r all decaying.

## 10. Longitudinal/lateral/vertical gust results
4 cases (longitudinal ∓5, lateral +5, vertical +3, plus a bonus longitudinal +8 m/s), 2s 1-cosine duration, ~13s observation window. All bounded on the **first attempt** — no extension needed (the boundedness-check safeguard carried forward from the engine-out stage's methodology lesson never had to trigger, since every case was already trending decaying/negligible/plateaued at cutoff). Longitudinal/vertical gusts primarily excited alpha (max 3.0–4.0°) with negligible p/r and decaying q. The lateral gust excited beta to 15.95° with p/q/r all decaying — the independent review independently extracted the local |r| peak envelope (48.07→30.02→20.20→...→1.81°/s, monotonically decaying with no uptick near cutoff) and confirmed the "decaying" label is genuinely supported by the data, not an engine-out-style premature call.

## 11. Boundedness evidence
See §10 — every case decaying or negligible by the end of its window, independently re-verified by the independent review at the raw-series level (not just the summary label) for the largest-excursion case.

## 12. Actuator behavior
Zero NaN/Inf, zero `target_clamp_active`/`effort_clamp_active` events anywhere across static, free-flight, and gust runs. Max elevator tracking error ~1.0–2.2°, but the independent review found this actually occurs during the initial servo slew-to-setpoint at sim start (t=0–0.05s), before release and before any wind/gust is active — not caused by wind/gust loading as originally (incorrectly) worded. Corrected in the README; tracking during the actual wind/gust-affected window remained bounded and non-clamping throughout, always recovering.

## 13. Propulsion inflow/wind coupling status
No gap. Confirmed directly in source (`plugins/propulsion/PropulsionSystem.cc:285`, `hubVelRel = *hubVelWorldOpt - this->windWorld`) that propulsion's per-motor advance-ratio calculation already used wind-corrected relative airspeed before this stage began — this was a pre-existing, correct design, not something this stage needed to add or fix. No propulsion-specific analysis was needed.

## 14. Validation findings
No CRITICAL or MAJOR findings. All 9 required verification items (headwind/tailwind V_rel sign, crosswind beta sign, vertical alpha sign, qbar scaling, zero-wind regression, no artificial force, genuine 6-DOF free-flight, no unrelated modifications) independently recomputed from raw data and confirmed PASS — not just accepted from the reports. The dual-independent-subscriber pattern (`AerodynamicsSystem` and `PropulsionSystem` each maintaining their own `windWorld` copy from the same published topic) was assessed and confirmed benign (standard pub/sub broadcast, no shared mutable state, no race). Two MINOR reporting-only items were found and corrected within this stage (a mislabeled prediction count, 12 vs. the actual 14; a tracking-error causal attribution that didn't match the raw timing) — both fixed in `tests/gazebo/README.md`, no code or data change required. **Final verdict: READY.**

## 15. Remaining atmospheric-model limitations
This V1 wind model validates: steady uniform wind and a deterministic finite (1-cosine) gust, one at a time. **Not yet modeled** (explicitly, per `WIND.md`): spatial wind gradients, terrain effects, rotor wake/building effects, Dryden/Von Kármán stochastic turbulence, any weather-scale atmospheric model. `Cl_delta_r` remains unresolved (unchanged, unrelated to this stage). Multiple simultaneous/overlapping gusts are not supported in V1 (a new gust command replaces any in-progress one).

## 16. Next recommended stage
`SENSOR_MODEL_AND_ARDUPLANE_SITL_PREPARATION` (not started).

---

**WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION_READY**

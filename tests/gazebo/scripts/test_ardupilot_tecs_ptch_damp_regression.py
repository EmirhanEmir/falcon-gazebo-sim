#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_LONGITUDINAL_PHUGOID_DAMPING_VALIDATION, part 3a:
SHORT TECS_PTCH_DAMP PERFORMANCE-REGRESSION HARNESS
(controls-integration, 2026-09-04)

WHY THIS HARNESS EXISTS
-----------------------
Part 2 of this stage showed that TECS_PTCH_DAMP = 0.6 damps the closed-loop
longitudinal energy mode materially harder than the firmware default 0.3
(last-15 s detrended altitude residual RMS 0.0916 -> 0.0121 m; altitude
second/first-half residual ratio 0.305 -> 0.105; pitch 0.308 -> 0.068) and
that it MOVES the mode frequency (T 5.6474 -> 4.7317 s, -16.2 %, confirmed
independently by DFT peak and by zero crossings).

Damping a mode harder is not automatically free. This harness answers ONE
question and nothing else:

    does TECS_PTCH_DAMP = 0.6 DEGRADE cruise / altitude hold / climb /
    descent performance relative to the measurements the 2026-09-03/04
    ARDUPLANE_TECS_CLIMB_DESCENT_AND_ENERGY_VALIDATION stage recorded?

It is a NOT-WORSE-THAN test against those recorded numbers. It is NOT a
re-run of that stage's 165 s campaign, it does not re-derive that stage's
verdict, and it never gates on BEATING the reference.

WHAT THIS HARNESS DOES **NOT** DO
---------------------------------
  * It does not re-audit FBWB mode selection, the RC->command mappings, the
    PTCH_TRIM_DEG telemetry convention, the atmosphere/pitot datum, the
    +/-45 deg surface scaling, the actuator/aero/propulsion models, or
    mass/CG/inertia. Those are closed in earlier stages.
  * It does not touch the inner pitch-rate loop (PTCH_RATE_*/PTCH2SRV_*).
  * It writes NO parameter by default, and can only ever write a TECS
    energy-loop parameter at RUNTIME (see PARAMETER POLICY).
  * It does not modify, and does not import in a way that could modify,
    tests/gazebo/scripts/test_ardupilot_tecs_climb_descent_energy.py. That
    module is READ-ONLY input: this file imports its analysis functions and
    its recorded reference measurements, and changes neither.

LAUNCH SEQUENCE - INHERITED VERBATIM
------------------------------------
run_ardupilot_tecs_ptch_damp_regression.sh uses EXACTLY the launch sequence
proven by run_ardupilot_tecs_climb_descent_energy.sh and reused by
run_ardupilot_longitudinal_phugoid_damping.sh: same world, same READ-ONLY
config/ardupilot/falcon_v2_sitl.parm, same environment, same gdb-wrapped
arduplane, same cleanup/trap, same SIM_OPOS origin handling (the
`-O 0,0,0,0` / CMAC 584 m trap stays AVOIDED and stays GATED through the
sim_opos_alt_zero_atmosphere_datum precondition), same zero-wind gating, and
the same LIVE re-proof of TECS throttle authority (never inherited on trust).

=============================================================================
PHASE PROFILE AND DURATION DERIVATION
=============================================================================
The stage spec asks for a SHORT regression:

    level cruise ~18 m/s + altitude hold
      -> ONE +10 m climb -> settle -> ONE -10 m descent -> settle

Every duration below is derived from MEASURED behaviour, not chosen for
convenience. Two measured quantities drive the whole plan:

  (a) the FBWB ramp rate actually achieved in the prior stage:
          ramp_vz      = 1.301 m/s  (mean over the climb ramp)
          ramp_peak_vz = 1.951 m/s
      so a commanded +10 m step needs 10/1.301 = 7.69 s of nominal ramp and
      TOOK 8.67 s in the prior run;
  (b) the closed-loop longitudinal energy-mode period measured in part 1 of
      THIS stage: T = 5.6474 s at the firmware default, 4.7317 s at
      TECS_PTCH_DAMP = 0.6. The SLOWER (baseline) period is used everywhere
      below, because it is the conservative one for window sizing.

ANALYSED-WINDOW LENGTH  W_HOLD_ANALYSED_S = 24.0 s
  Binding requirement: the "no growing oscillation" test
  (fbwa.detrended_growth, inherited) splits the window in half and compares
  residual spread, so each half must contain at least 2 full cycles of the
  mode -> 4 cycles minimum -> 4 * 5.6474 = 22.59 s. 24.0 s = 4.25 baseline
  cycles (5.07 candidate cycles, 2.96 free-airframe Lanchester phugoid
  periods, 4.8 x TECS_TIME_CONST). This is the SHORTEST window that resolves
  what the hold phases measure; anything shorter degrades both the growth
  test and the mean/std statistics.

R1_CRUISE = 12.0 + 24.0 = 36.0 s
  12.0 s = P1_TRANSIENT_S, INHERITED UNCHANGED from the 2026-09-02 cruise
  stage: FBWB entry, throttle unsuppression and TECS filter initialisation.
  Followed by the 24.0 s analysed window above. The prior stage used 45 s
  (33 s analysed); the extra 9 s bought additional cycles, not additional
  resolution of any gated quantity.

R2_CLIMB  cap 15.0 s, closed-loop stop at z >= z_ref + 10 m
  Nominal ramp 7.69 s; prior measured 8.67 s. Cap = nominal + 2 x
  TECS_HDEM_TCONST (3.0 s, AP_TECS.cpp:292) = 13.7 s, rounded up to 15.0 s
  = 1.73 x the prior measured duration. The cap only bounds a FAILURE to
  achieve the demand; hitting it is itself a reportable regression
  (climb_ramp_stopped_early is recorded). The prior stage's 20 s cap is
  shortened here because the cap is not a measurement window.

R3_SETTLE = 10.0 + 24.0 = 34.0 s
  10.0 s = HOLD_TRANSIENT_S, INHERITED UNCHANGED (2 x TECS_TIME_CONST), the
  same post-transient cutoff every previous stage used, so the hold-window
  statistics are directly comparable with the recorded reference. Then the
  24.0 s analysed window. The prior stage used 40 s because it had to
  contain a SETTLING-TIME criterion; this harness does not measure settling
  time (part 1/2 of this stage did that on a purpose-built ring-down), so
  that extra length is not needed here.

R4_DESCENT cap 15.0 s, closed-loop stop at z <= z_ref  (mirror of R2)

R5_RESETTLE = 10.0 + 24.0 = 34.0 s  (mirror of R3; carries the
  return-to-origin / round-trip residual measurement)

TOTAL FLIGHT TIME
  worst case (both ramps run to their caps): 36 + 15 + 34 + 15 + 34 = 134.0 s
  expected  (ramps ~8.7 s, from the prior measurement):        ~121.4 s
  the campaign this replaces:                                   165.0 s
  -> 18.8 % shorter worst case, ~26.4 % shorter in expectation. The residual
  length is set by the 4-cycle window requirement above; trimming further
  would trade measurement validity for wall-clock time, which this project's
  rules do not allow.

=============================================================================
ACCEPTANCE CRITERIA - NOT-WORSE-THAN, AGAINST **TWO** RECORDED REFERENCE
REALISATIONS OF THE SAME CONFIGURATION
=============================================================================
The same nominal manoeuvre (FBWB, 18 m/s, +/-10 m step, TECS at the compiled
firmware defaults) has been flown and recorded TWICE:

  REF_A  2026-09-02  ARDUPLANE_TECS_AND_CRUISE_SPEED_HOLD_VALIDATION campaign
         - the measurement set carried forward as energy.PRIOR and quoted in
           this stage's task brief (cruise_airspeed_mean 17.926 m/s,
           achieved_climb +10.239 m, ...).
  REF_B  2026-09-03/04 ARDUPLANE_TECS_CLIMB_DESCENT_AND_ENERGY_VALIDATION
         campaign - the numbers that stage itself recorded and published
         (tests/gazebo/results/ardupilot_tecs_climb_descent_energy_result.json,
          verdict TECS_CLIMB_DESCENT_ENERGY_PASS).

OFFLINE CROSS-CHECK, 2026-09-04, before this harness was ever flown: the
analysis in THIS module was run over REF_B's own recorded timeseries and
reproduced its published numbers exactly - achieved_climb 10.521232 m,
achieved_descent -11.153656 m, roundtrip -0.632424 m, hold altitude p2p
2.492022 m, hold |vz| 0.008534 m/s, cruise airspeed 17.925597 +/- 0.239048
m/s, throttle 0.490856 / 0.540432 / 0.420059, pitch 2.684121 / 6.718590 /
-1.332015 deg, STEdot -0.006060 / +11.903230 / -11.471466 W/kg, whole-flight
airspeed 16.650883 .. 19.497429 m/s, elevator 6.333634 deg. This harness's
analysis is therefore numerically identical to the stage it regresses
against, and the reference values below are not re-derived quantities.

WHY BOTH REFERENCES ARE USED. Gating against REF_A alone would FAIL REF_B on
two metrics (achieved_descent -11.154 m vs a REF_A-only tolerance of 0.85 m,
and hold altitude p2p 2.492 m) - i.e. a REF_A-only criterion would reject a
known-good, already-validated baseline run. The two runs are the ONLY
measured run-to-run scatter that exists for this configuration, and using it
is what stops the tolerances from being either invented or tuned. The
per-metric spread |REF_A - REF_B| is recorded in the result artifact.

GATE FORM (uniform, no double counting):
    band-type metric :  min(A,B) - TOL  <=  measured  <=  max(A,B) + TOL
    max-type metric  :  measured <= max(A,B) + TOL
    min-type metric  :  measured >= min(A,B) - TOL
where TOL is that metric's INDEPENDENTLY DERIVED estimator / command-path
uncertainty (below). min/max over the two realisations already carries the
run-to-run scatter, so TOL is added once and only once. No gate requires
BEATING either reference. Where an INHERITED absolute threshold exists it is
applied as a SEPARATE, additional gate and is never relaxed.

TOL derivations - each from a recorded scatter source, none invented:
  S1  within-window airspeed scatter (REF_A)              std = 0.187 m/s
  S2  disturbance-realisation spread within REF_B: the SAME nominal +/-10 m
      command produced A0_alt 3.354222 m (P3) and 2.922292 m (P5)
      -> |dA0|/mean(A0) = 0.1376
  S3  estimator uncertainty caused by the residual oscillation itself. For a
      component A*cos(w*t + phi) of unknown phase over a window W the exact
      worst-case values of the two linear functionals are
          mean bias      <= 2A/(w*W)
          LSQ slope bias <= 12A/(w*W^2)
      with A = REF_A hold_alt_p2p_max/2 = 1.0895 m and w = 2*pi/5.6474 =
      1.11259 rad/s (the closed-loop mode measured in part 1 of THIS stage at
      the firmware default - the slower, conservative choice).

  cruise_airspeed_mean   band, TOL = 3*S1/sqrt(n_cycles) = 0.272 m/s
                         (n_cycles = 24.0/5.6474 = 4.25 independent cycles in
                         the analysed window). PLUS the inherited absolute
                         gate |mean - TECS target| <= TH_SPEED_MEAN_TOL_MS
                         (0.5 m/s).
  cruise_airspeed_std    max-type, TOL factor (1 + 3/sqrt(2*(n_cycles-1))) =
                         2.18x applied to max(A,B): the sampling uncertainty
                         of a std estimated from 4.25 effective samples.
                         RESOLUTION LIMITATION, declared: with A = 0.187 and
                         B = 0.239 m/s this gate cannot resolve small changes
                         in speed-hold scatter; the inherited absolute gate
                         TH_SPEED_STD_MAX_MS (0.5 m/s) is the binding one.
                         More repeats are DATA_REQUIRED.
  hold_vz_max_abs        max-type, TOL = 12*A/(w*W^2) = 0.0204 m/s - the LSQ
                         slope bias the residual oscillation alone produces
                         over a 24 s window, i.e. gating tighter than this
                         would be gating on estimator noise. PLUS inherited
                         TH_ALT_SLOPE_MAX_MS (0.10 m/s).
  hold_alt_p2p_max       max-type, TOL = S2 * max(A,B) (the disturbance-
                         realisation spread applied to the larger reference).
                         PLUS inherited TH_ALT_P2P_MAX_M (5.0 m).
                         DECLARED BIAS: p2p is an extreme-value statistic and
                         this harness analyses 24 s against the references'
                         30-33 s, which can only bias p2p DOWN. The gate is
                         NECESSARY, not SUFFICIENT; it is not loosened to
                         compensate and the bias direction is recorded.
  achieved_climb /       band, TOL = 0.30 + 0.39 + 0.162 = 0.852 m
  achieved_descent       0.30 m = FBWB target-altitude integrator granularity
                           (FBWB_CLIMB_RATE 2.0 m/s x the 0.15 s max update
                           interval), derivation inherited from the energy
                           stage;
                         0.39 m = travel between the ramp stop trigger and
                           the stick-release lock, peak_vz 1.951 m/s x
                           (0.1 s RC refresh + 0.1 s FBWB check period);
                         0.162 m = mean bias 2A/(w*W) of the settle-window
                           altitude mean, with A = A0_alt(P3) decayed to the
                           window start, 3.354222*exp(-10/22.91) = 2.166 m.
                         PLUS inherited TH_TARGET_STEP_TOL_M (3.0 m).
  roundtrip_alt_residual max-type on |residual|, TOL = 0.39 + 2*0.162 =
                         0.715 m. PLUS inherited TH_RESETTLE_TOL_M (2.0 m).
  ramp vz (mean, peak;   min-type on the MAGNITUDE, TOL = 2*A_vz/(w*W_ramp) =
  climb and descent)     0.251 m/s with A_vz = A*w = 1.212 m/s and W_ramp =
                         8.67 s: the mean bias the residual oscillation can
                         produce over a ramp that short. PLUS the inherited
                         direction gate TH_RAMP_DIRECTION_MIN_MS (0.2 m/s).
  ramp duration          max-type, TOL factor (1 + 0.251/1.1534) = 1.218x on
                         max(A,B) - the same vz uncertainty propagated
                         through duration = step / vz.
  throttle (3 phases)    band, TOL = TH_THROTTLE_TOL (0.05), INHERITED. The
                         oscillation-induced mean bias on throttle over these
                         windows is ~0.004, so the inherited tolerance
                         dominates. PLUS the inherited ORDERING checks:
                         climb > level > descent, each by >=
                         TH_COORD_THROTTLE_DELTA (0.01).
  pitch (3 phases)       band, TOL = TH_PITCH_ALPHA_GAMMA_RESID_DEG (1.5 deg),
                         INHERITED: this infrastructure's own demonstrated
                         pitch consistency bound (the pitch = alpha + gamma
                         residual), i.e. the finest pitch statement the
                         harness can support. PLUS the inherited ORDERING
                         checks: climb > level > descent, each by >=
                         TH_COORD_PITCH_DELTA_DEG (0.5 deg).
  level STEdot           |meas| <= TH_LEVEL_STEDOT_MAX_W_PER_KG (1.0),
                         INHERITED. References: +0.05 and -0.006 W/kg.
  climb/descent STEdot   min-/max-type, TOL = g * 0.251 = 2.466 W/kg (the
                         ramp vz mean-bias expressed as specific power,
                         g = 9.81 world gravity). PLUS the inherited sign
                         gate TH_RAMP_STEDOT_MIN_W_PER_KG (2.0 W/kg).
  whole_flight airspeed  min-type / max-type, TOL = 3*S1 = 0.561 m/s (3-sigma
                         bound on ONE extreme sample using the recorded
                         within-window scatter). PLUS the ABSOLUTE envelope
                         airspeed >= TH_SPEED_MIN_MS (16.0 m/s = AIRSPEED_MIN).
                         The shorter campaign biases extremes INWARD;
                         declared, not compensated.
  whole_flight elevator  GATED on the ABSOLUTE envelope only: <= 10 deg
                         (TH_SURF_HOLD_MAX_DEG, the stage spec's "normally
                         <= 10 deg") and never within 5 deg of the +/-45 deg
                         mechanical travel (TH_SURF_MAX_ABS_DEG = 40 deg).
                         The not-worse-than-reference elevator comparison is
                         REPORTED, NON-GATING, on purpose: raising
                         TECS_PTCH_DAMP raises the loop's ONLY derivative
                         term, so MORE transient surface activity is the
                         intended, physically correct consequence of the
                         change - not a performance regression, as long as
                         the deflection stays inside the linear, unsaturated
                         envelope. Gating it would gate against the intended
                         effect.

ABSOLUTE ENVELOPE (from the stage spec, all GATING)
  airspeed >= 16 m/s; no growing oscillation in any hold window; no sustained
  throttle saturation (>= TH_SAT_RUN_MAX_S = 2.0 s at THR_MIN/THR_MAX) and no
  actuator clamping; elevator normally <= 10 deg; no NaN/Inf anywhere in the
  analysed quantities.

PARAMETER POLICY - READ THIS BEFORE ADDING A FLAG
-------------------------------------------------
  * DEFAULT: this test writes NO parameter of any kind. TECS then runs on the
    ArduPlane compiled firmware defaults (config/ardupilot/falcon_v2_sitl.parm
    sets no TECS_* value and arduplane is launched with -w, a wiped scratch
    EEPROM). Running with no flags therefore produces a DEFAULTS baseline of
    this same harness.
  * config/ardupilot/falcon_v2_sitl.parm is READ-ONLY input and is never
    edited by this file or its runner. NO TECS default change is written to
    any checked-in file: the 0.6 value exists only as a runtime PARAM_SET.
  * `--set-param NAME=VALUE` (repeatable) performs a RUNTIME MAVLink
    PARAM_SET, in the SITL scratch EEPROM only, and ONLY for names in
    SETTABLE_PARAMS below (the TECS energy loop), and only inside each
    parameter's own ArduPilot-documented @Range. Every other name is REFUSED
    with a non-zero exit: no PID, no PTCH_TRIM_DEG, no SERVOn_*, no ARSPD_*,
    no SIM_*, no aero/propulsion/actuator/sensor/mass value can be written
    through this path. Each write is read back and confirmed, and the before
    and after values are recorded.
    This is the IDENTICAL mechanism and the IDENTICAL whitelist used by
    test_ardupilot_longitudinal_phugoid_damping.py. It is duplicated here
    rather than imported so that this harness has no import-time dependency
    on a stage module that is under concurrent edit; the whitelist ranges are
    cited to the same AP_TECS.cpp @Range lines in both files.

OPEN LIMITATION CARRIED (NON-GATING)
------------------------------------
  PROPULSION_HIGH_J_WINDMILLING (owner: propulsion, DATA_REQUIRED). The APC
  13x6.5E Ct/Cp table ends at the zero-thrust advance ratio, so thrust is
  floored at 0 N where a real fixed-pitch prop would windmill and produce
  NEGATIVE thrust. The simulated descent is therefore less draggy than
  reality. Interp-clamped and zero-thrust motor-sample counts are recorded
  for every window and for the whole flight (high_j blocks). The descent
  RESULT of this harness must not be presented as absolute high-fidelity
  descent performance; it is a comparison between two TECS_PTCH_DAMP settings
  under the SAME propulsion limitation.

USAGE (a FRESH gz sim + gdb-wrapped arduplane pair MUST already be running -
see tests/gazebo/scripts/run_ardupilot_tecs_ptch_damp_regression.sh):
    python3 test_ardupilot_tecs_ptch_damp_regression.py \
        --set-param TECS_PTCH_DAMP=0.6 --tag ptchdamp06
    python3 test_ardupilot_tecs_ptch_damp_regression.py --tag defaults
    python3 test_ardupilot_tecs_ptch_damp_regression.py \
        --reanalyze <timeseries.json> --tag ptchdamp06
"""
import json
import math
import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import gz.transport13 as tp  # noqa: E402
from gz.msgs10 import entity_wrench_pb2, entity_pb2  # noqa: E402

import test_ardupilot_basic_closed_loop_flight as base  # noqa: E402
import test_ardupilot_basic_closed_loop_flight_campaign as campaign  # noqa: E402
import test_ardupilot_fbwa_level_pitch_reference_correction as fbwa  # noqa: E402
import test_ardupilot_tecs_cruise_speed_hold as cruise  # noqa: E402
# The 2026-09-03/04 climb/descent/energy harness, IMPORTED READ-ONLY (never
# edited by this stage). Every window/energy/high-J analysis function and every
# inherited threshold below comes from it unmodified.
import test_ardupilot_tecs_climb_descent_energy as energy  # noqa: E402
import aero_lib  # noqa: E402
import propulsion_lib  # noqa: E402
import actuator_lib  # noqa: E402

linreg = fbwa.linreg
mean = fbwa.mean
stdev = fbwa.stdev
minmaxmean = fbwa.minmaxmean
series_report = fbwa.series_report
detrended_growth = fbwa.detrended_growth
read_param = fbwa.read_param

collect = energy.collect
longest_run_seconds = energy.longest_run_seconds
s_alt = energy.s_alt
s_tas = energy.s_tas
s_pitch_phys = energy.s_pitch_phys
s_pitch_demand_phys = energy.s_pitch_demand_phys
s_throttle_actual = energy.s_throttle_actual
s_surface_deg = energy.s_surface_deg
s_elev_deg = energy.s_elev_deg
s_spe, s_ske, s_ste, s_seb = energy.s_spe, energy.s_ske, energy.s_ste, energy.s_seb
seb_weights = energy.seb_weights
high_j_block = energy.high_j_block

STAGE = "ARDUPLANE_LONGITUDINAL_PHUGOID_DAMPING_VALIDATION"
PART = "part3a_tecs_ptch_damp_performance_regression"
FILE_PREFIX = "ardupilot_tecs_ptch_damp_regression"

ARDUPLANE_FBWB_CUSTOM_MODE = cruise.ARDUPLANE_FBWB_CUSTOM_MODE   # 6

# ---- reference constants (all inherited, none invented here) ---------------
MASS_KG = energy.MASS_KG                        # 6.000    CLAUDE.md
S_REF_M2 = energy.S_REF_M2                      # 0.4514   CLAUDE.md
G_WORLD = energy.G_WORLD                        # 9.81     world <gravity>
G_TECS = energy.G_TECS                          # 9.80665  AP_Math/definitions.h:45
V_TRIM_REF = energy.V_TRIM_REF                  # 18.162 m/s measured Gazebo trim
TRIM_THROTTLE_REF = energy.TRIM_THROTTLE_REF    # 0.4957
ELEV_TRIM_DEG_REF = energy.ELEV_TRIM_DEG_REF    # +4.092 deg
PTCH_TRIM_DEG_EXPECTED = energy.PTCH_TRIM_DEG_EXPECTED   # 2.49
V_TARGET_MS = energy.V_TARGET_MS                # 18.0 = AIRSPEED_CRUISE
ALT_STEP_M = energy.ALT_STEP_M                  # 10.0
SURFACE_TRAVEL_LIMIT_DEG = energy.SURFACE_TRAVEL_LIMIT_DEG   # 45.0

# =============================================================================
# THE REFERENCES. TWO recorded realisations of the SAME nominal configuration
# (FBWB, 18 m/s, +/-10 m step, TECS at the compiled firmware defaults):
#
#   REF_A  2026-09-02 ARDUPLANE_TECS_AND_CRUISE_SPEED_HOLD_VALIDATION campaign,
#          imported from energy.PRIOR so the two files cannot silently diverge.
#          This is the measurement set quoted in this stage's task brief.
#   REF_B  2026-09-03/04 ARDUPLANE_TECS_CLIMB_DESCENT_AND_ENERGY_VALIDATION
#          campaign, as that stage itself recorded and published
#          (tests/gazebo/results/ardupilot_tecs_climb_descent_energy_result.json,
#          verdict TECS_CLIMB_DESCENT_ENERGY_PASS). Every value below was
#          reproduced EXACTLY by running THIS module's analysis over that
#          stage's own timeseries (offline cross-check, 2026-09-04 - see the
#          module docstring).
#
# These are regression REFERENCES only: no value here feeds a control path, and
# no gate requires reproducing any of them exactly or beating them.
# =============================================================================
REF_A = dict(energy.PRIOR)
REF_B = dict(
    cruise_airspeed_mean_ms=17.925597, cruise_airspeed_std_ms=0.239048,
    hold_vz_max_abs_ms=0.008534, hold_alt_p2p_max_m=2.492022,
    achieved_climb_m=10.521232, achieved_descent_m=-11.153656,
    roundtrip_alt_residual_m=-0.632424,
    ramp_vz_ms=1.153409, ramp_peak_vz_ms=2.131080,
    descent_ramp_vz_ms=-1.146794, descent_ramp_peak_vz_ms=-1.936985,
    climb_ramp_duration_s=8.719136, descent_ramp_duration_s=9.260584,
    level_throttle=0.490856, climb_throttle=0.540432, descent_throttle=0.420059,
    level_pitch_deg=2.684121, climb_pitch_deg=6.718590, descent_pitch_deg=-1.332015,
    level_specific_energy_rate_W_per_kg=-0.006060,
    climb_specific_energy_rate_W_per_kg=11.903230,
    descent_specific_energy_rate_W_per_kg=-11.471466,
    whole_flight_airspeed_min_ms=16.650883,
    whole_flight_airspeed_max_ms=19.497429,
    whole_flight_elevator_max_abs_deg=6.333634,
)
# Kept for the constants that only exist in one of the two records.
REF = REF_A
# REF_B's own within-run disturbance-realisation scatter (peak altitude
# excursion produced by the SAME nominal +/-10 m command, P3 vs P5).
REF_A0_ALT_P3_M = 3.354222
REF_A0_ALT_P5_M = 2.922292
# Ramp length used to size the ramp-window estimator bias: REF_A's measured
# climb ramp (REF_B measured 8.719 s, so the two agree to 0.6 %).
REF_CLIMB_RAMP_DURATION_S = 8.67

# Closed-loop longitudinal energy-mode measurements from part 1/2 of THIS
# stage (purpose-built free ring-down, 35/35 gates PASS).
# Source: tests/gazebo/results/ardupilot_longitudinal_phugoid_damping_result_
# baseline.json and ..._result_ptchdamp06.json
MODE_PERIOD_BASELINE_S = 5.6474       # TECS_PTCH_DAMP = 0.3 (firmware default)
MODE_PERIOD_PTCHDAMP06_S = 4.7317     # TECS_PTCH_DAMP = 0.6 (candidate)
MODE_TAU_BASELINE_S = 22.91           # log-decrement envelope time constant
MODE_OMEGA_BASELINE_RAD_S = 2.0 * math.pi / MODE_PERIOD_BASELINE_S   # 1.11259

# =============================================================================
# PHASE PLAN (full derivation: module docstring, PHASE PROFILE section)
# =============================================================================
R1_TRANSIENT_S = cruise.SEG_A_TRANSIENT_S       # 12.0  INHERITED UNCHANGED
HOLD_TRANSIENT_S = energy.HOLD_TRANSIENT_S      # 10.0  INHERITED UNCHANGED
# Shortest window that still resolves what the hold phases measure: 4 full
# cycles of the SLOWER (baseline) closed-loop mode, so each half of the
# detrended-growth test contains >= 2 cycles.
W_HOLD_ANALYSED_S = 24.0
N_CYCLES_HOLD = W_HOLD_ANALYSED_S / MODE_PERIOD_BASELINE_S            # 4.250
R1_CRUISE_S = R1_TRANSIENT_S + W_HOLD_ANALYSED_S                      # 36.0
R3_SETTLE_S = HOLD_TRANSIENT_S + W_HOLD_ANALYSED_S                    # 34.0
R5_RESETTLE_S = HOLD_TRANSIENT_S + W_HOLD_ANALYSED_S                  # 34.0
# Ramp cap = nominal ramp time + 2 x TECS_HDEM_TCONST (3.0 s, AP_TECS.cpp:292),
# rounded up: 10/1.301 + 6 = 13.69 -> 15.0 s = 1.73 x the prior measured 8.67 s.
RAMP_NOMINAL_S = ALT_STEP_M / REF["ramp_vz_ms"]                       # 7.686
R2_CLIMB_MAX_S = 15.0
R4_DESCENT_MAX_S = 15.0
RAMP_STOP_CONSECUTIVE = energy.RAMP_STOP_CONSECUTIVE                  # 3

PHASES = ["R1_cruise", "R2_climb", "R3_settle", "R4_descent", "R5_resettle"]
HOLD_WINDOWS = ["R1_cruise_hold", "R3_settle_hold", "R5_resettle_hold"]
RAMP_PHASES = ["R2_climb", "R4_descent"]
TOTAL_FLIGHT_MAX_S = (R1_CRUISE_S + R2_CLIMB_MAX_S + R3_SETTLE_S
                      + R4_DESCENT_MAX_S + R5_RESETTLE_S)             # 134.0
TOTAL_FLIGHT_EXPECTED_S = (R1_CRUISE_S + REF_CLIMB_RAMP_DURATION_S + R3_SETTLE_S
                           + REF_CLIMB_RAMP_DURATION_S + R5_RESETTLE_S)  # 121.34
PRIOR_CAMPAIGN_TOTAL_S = 165.0

# =============================================================================
# INHERITED ACCEPTANCE THRESHOLDS (cited to the imported symbol so they cannot
# silently diverge). NO inherited threshold is changed by this harness.
# =============================================================================
TH_SPEED_MIN_MS = energy.TH_SPEED_MIN_MS                  # 16.0 = AIRSPEED_MIN
TH_SPEED_HARD_FLOOR_MS = energy.TH_SPEED_HARD_FLOOR_MS    # 14.4
TH_SPEED_MEAN_TOL_MS = energy.TH_SPEED_MEAN_TOL_MS        # 0.5
TH_SPEED_STD_MAX_MS = energy.TH_SPEED_STD_MAX_MS          # 0.5
TH_TECS_TARGET_TOL_MS = energy.TH_TECS_TARGET_TOL_MS      # 0.4
TH_ALT_SLOPE_MAX_MS = energy.TH_ALT_SLOPE_MAX_MS          # 0.10
TH_ALT_P2P_MAX_M = energy.TH_ALT_P2P_MAX_M                # 5.0
TH_THROTTLE_TOL = energy.TH_THROTTLE_TOL                  # 0.05
TH_SAT_RUN_MAX_S = energy.TH_SAT_RUN_MAX_S                # 2.0
TH_SAT_MARGIN = energy.TH_SAT_MARGIN                      # 0.01
TH_TECS_AUTHORITY_MIN_DELTA = energy.TH_TECS_AUTHORITY_MIN_DELTA   # 0.10
TH_THROTTLE_MODULATION_MIN = energy.TH_THROTTLE_MODULATION_MIN     # 0.05
TH_SURF_HOLD_MAX_DEG = energy.TH_SURF_HOLD_MAX_DEG        # 10.0
TH_SURF_FLIGHT_MAX_DEG = energy.TH_SURF_FLIGHT_MAX_DEG    # 15.0
TH_LATERAL_SURF_MAX_DEG = energy.TH_LATERAL_SURF_MAX_DEG  # 10.0
TH_SURF_MAX_ABS_DEG = energy.TH_SURF_MAX_ABS_DEG          # 40.0 = 45 - 5
TH_COORD_THROTTLE_DELTA = energy.TH_COORD_THROTTLE_DELTA  # 0.01
TH_COORD_PITCH_DELTA_DEG = energy.TH_COORD_PITCH_DELTA_DEG            # 0.5
TH_PITCH_ALPHA_GAMMA_RESID_DEG = energy.TH_PITCH_ALPHA_GAMMA_RESID_DEG  # 1.5
TH_RAMP_DIRECTION_MIN_MS = energy.TH_RAMP_DIRECTION_MIN_MS            # 0.2
TH_TARGET_STEP_TOL_M = energy.TH_TARGET_STEP_TOL_M        # 3.0
TH_RESETTLE_TOL_M = energy.TH_RESETTLE_TOL_M              # 2.0
TH_LEVEL_STEDOT_MAX_W_PER_KG = energy.TH_LEVEL_STEDOT_MAX_W_PER_KG    # 1.0
TH_RAMP_STEDOT_MIN_W_PER_KG = energy.TH_RAMP_STEDOT_MIN_W_PER_KG      # 2.0
TH_DESCENT_SPEED_OVERSHOOT_MAX_MS = energy.TH_DESCENT_SPEED_OVERSHOOT_MAX_MS  # 2.0
TH_PITCH_DEMAND_MARGIN_DEG = 1.0    # same 1 deg clearance part 1 of this stage
                                    # used against the TECS pitch-demand clip
                                    # (AP_TECS.cpp:1488-1500 + Attitude.cpp:638)

# =============================================================================
# REGRESSION TOLERANCES - DERIVED HERE, EACH FROM A RECORDED SCATTER SOURCE
# (full narrative: module docstring, ACCEPTANCE CRITERIA section)
# =============================================================================
# S1: within-window airspeed scatter recorded by the prior stage.
S1_AIRSPEED_STD_MS = REF["cruise_airspeed_std_ms"]                     # 0.187
# S2: disturbance-realisation scatter - the SAME nominal +/-10 m command
#     produced two different peak excursions in the prior run.
S2_DISTURBANCE_REL_SPREAD = (abs(REF_A0_ALT_P3_M - REF_A0_ALT_P5_M)
                             / (0.5 * (REF_A0_ALT_P3_M + REF_A0_ALT_P5_M)))   # 0.1376
# S3: estimator uncertainty produced by the residual oscillation itself.
A_HOLD_M = REF["hold_alt_p2p_max_m"] / 2.0                             # 1.0895 m
A_VZ_HOLD_MS = A_HOLD_M * MODE_OMEGA_BASELINE_RAD_S                    # 1.2122 m/s
# worst-case mean bias of a cosine of unknown phase over a window: 2A/(w*W)
MEAN_BIAS_HOLD_M = 2.0 * (REF_A0_ALT_P3_M * math.exp(-HOLD_TRANSIENT_S
                                                     / MODE_TAU_BASELINE_S)) \
    / (MODE_OMEGA_BASELINE_RAD_S * W_HOLD_ANALYSED_S)                  # 0.162 m
MEAN_BIAS_RAMP_VZ_MS = 2.0 * A_VZ_HOLD_MS / (MODE_OMEGA_BASELINE_RAD_S
                                             * REF_CLIMB_RAMP_DURATION_S)  # 0.251
# worst-case LSQ slope bias of a cosine of unknown phase: 12A/(w*W^2)
SLOPE_BIAS_HOLD_MS = 12.0 * A_HOLD_M / (MODE_OMEGA_BASELINE_RAD_S
                                        * W_HOLD_ANALYSED_S ** 2)      # 0.0204 m/s
# FBWB command-path quantisation (derivations inherited from the prior stage)
FBWB_INTEGRATOR_GRANULARITY_M = 2.0 * 0.15      # FBWB_CLIMB_RATE x max update dt
RAMP_STOP_LAG_M = REF["ramp_peak_vz_ms"] * (campaign.RC_REFRESH_PERIOD + 0.1)

# ---------------------------------------------------------------------------
# TWO-REFERENCE GATE CONSTRUCTION. min/max over the two recorded realisations
# already carries the run-to-run scatter, so the derived TOL above is added
# once and only once (see the module docstring, GATE FORM).
# ---------------------------------------------------------------------------
def _pair(key_a, key_b=None):
    """(REF_A value, REF_B value) for one metric; either may be None."""
    return (REF_A.get(key_a), REF_B.get(key_b or key_a))


def _lo(pair, tol):
    vals = [v for v in pair if v is not None]
    return (min(vals) - tol) if vals else None


def _hi(pair, tol):
    vals = [v for v in pair if v is not None]
    return (max(vals) + tol) if vals else None


def _band(pair, tol):
    return (_lo(pair, tol), _hi(pair, tol))


def _spread(pair):
    vals = [v for v in pair if v is not None]
    return (max(vals) - min(vals)) if len(vals) == 2 else None


# ---- per-metric TOL values (each derived above) ----------------------------
TOL_CRUISE_V_MEAN_MS = 3.0 * S1_AIRSPEED_STD_MS / math.sqrt(N_CYCLES_HOLD)  # 0.272
TH_REG_CRUISE_V_STD_FACTOR = 1.0 + 3.0 / math.sqrt(2.0 * (N_CYCLES_HOLD - 1.0))
TOL_HOLD_VZ_MS = SLOPE_BIAS_HOLD_MS                                         # 0.0204
TOL_STEP_M = (FBWB_INTEGRATOR_GRANULARITY_M + RAMP_STOP_LAG_M
              + MEAN_BIAS_HOLD_M)                                           # 0.853
TOL_ROUNDTRIP_M = RAMP_STOP_LAG_M + 2.0 * MEAN_BIAS_HOLD_M                  # 0.715
TOL_RAMP_VZ_MS = MEAN_BIAS_RAMP_VZ_MS                                       # 0.251
TOL_RAMP_DURATION_FACTOR = 1.0 + MEAN_BIAS_RAMP_VZ_MS / REF_B["ramp_vz_ms"]  # 1.218
TOL_STEDOT_W_PER_KG = G_WORLD * MEAN_BIAS_RAMP_VZ_MS                        # 2.466
TOL_AIRSPEED_EXTREME_MS = 3.0 * S1_AIRSPEED_STD_MS                          # 0.561

# ---- resulting gate limits -------------------------------------------------
PAIR_V_MEAN = _pair("cruise_airspeed_mean_ms")
PAIR_V_STD = _pair("cruise_airspeed_std_ms")
PAIR_HOLD_VZ = _pair("hold_vz_max_abs_ms")
PAIR_ALT_P2P = _pair("hold_alt_p2p_max_m")
PAIR_CLIMB_M = _pair("achieved_climb_m")
PAIR_DESCENT_MAG_M = (abs(REF_A["achieved_descent_m"]), abs(REF_B["achieved_descent_m"]))
PAIR_ROUNDTRIP_ABS_M = (abs(REF_A["roundtrip_alt_residual_m"]),
                        abs(REF_B["roundtrip_alt_residual_m"]))
PAIR_CLIMB_VZ = _pair("ramp_vz_ms")
PAIR_CLIMB_PEAK_VZ = _pair("ramp_peak_vz_ms")
# ASSUMPTION RAMP_VZ_REFERENCE_IS_CLIMB: REF_A records only one ramp figure and
# it is treated as the CLIMB ramp; REF_B measured the descent ramp separately.
PAIR_DESCENT_VZ_MAG = (REF_A["ramp_vz_ms"], abs(REF_B["descent_ramp_vz_ms"]))
PAIR_DESCENT_PEAK_VZ_MAG = (REF_A["ramp_peak_vz_ms"], abs(REF_B["descent_ramp_peak_vz_ms"]))
PAIR_LEVEL_THR = _pair("level_throttle")
PAIR_CLIMB_THR = _pair("climb_throttle")
PAIR_DESCENT_THR = _pair("descent_throttle")
PAIR_LEVEL_PITCH = _pair("level_pitch_deg")
PAIR_CLIMB_PITCH = _pair("climb_pitch_deg")
PAIR_DESCENT_PITCH = _pair("descent_pitch_deg")
PAIR_LEVEL_STEDOT = _pair("level_specific_energy_rate_W_per_kg")
PAIR_CLIMB_STEDOT = _pair("climb_specific_energy_rate_W_per_kg")
PAIR_DESCENT_STEDOT = _pair("descent_specific_energy_rate_W_per_kg")
PAIR_WF_V_MIN = _pair("whole_flight_airspeed_min_ms")
PAIR_WF_V_MAX = _pair("whole_flight_airspeed_max_ms")
PAIR_ELEVATOR = _pair("whole_flight_elevator_max_abs_deg")

TH_REG_V_MEAN_BAND_MS = _band(PAIR_V_MEAN, TOL_CRUISE_V_MEAN_MS)
TH_REG_V_STD_MAX_MS = max(PAIR_V_STD) * TH_REG_CRUISE_V_STD_FACTOR
TH_REG_HOLD_VZ_MAX_MS = _hi(PAIR_HOLD_VZ, TOL_HOLD_VZ_MS)
TH_REG_ALT_P2P_MAX_M = _hi(PAIR_ALT_P2P, S2_DISTURBANCE_REL_SPREAD * max(PAIR_ALT_P2P))
TH_REG_CLIMB_BAND_M = _band(PAIR_CLIMB_M, TOL_STEP_M)
TH_REG_DESCENT_MAG_BAND_M = _band(PAIR_DESCENT_MAG_M, TOL_STEP_M)
TH_REG_ROUNDTRIP_MAX_M = _hi(PAIR_ROUNDTRIP_ABS_M, TOL_ROUNDTRIP_M)
TH_REG_CLIMB_VZ_MIN_MS = _lo(PAIR_CLIMB_VZ, TOL_RAMP_VZ_MS)
TH_REG_CLIMB_PEAK_VZ_MIN_MS = _lo(PAIR_CLIMB_PEAK_VZ, TOL_RAMP_VZ_MS)
TH_REG_DESCENT_VZ_MAG_MIN_MS = _lo(PAIR_DESCENT_VZ_MAG, TOL_RAMP_VZ_MS)
TH_REG_DESCENT_PEAK_VZ_MAG_MIN_MS = _lo(PAIR_DESCENT_PEAK_VZ_MAG, TOL_RAMP_VZ_MS)
TH_REG_CLIMB_RAMP_DURATION_MAX_S = REF_B["climb_ramp_duration_s"] * TOL_RAMP_DURATION_FACTOR
TH_REG_DESCENT_RAMP_DURATION_MAX_S = (REF_B["descent_ramp_duration_s"]
                                      * TOL_RAMP_DURATION_FACTOR)
TH_REG_LEVEL_THR_BAND = _band(PAIR_LEVEL_THR, TH_THROTTLE_TOL)
TH_REG_CLIMB_THR_BAND = _band(PAIR_CLIMB_THR, TH_THROTTLE_TOL)
TH_REG_DESCENT_THR_BAND = _band(PAIR_DESCENT_THR, TH_THROTTLE_TOL)
TH_REG_LEVEL_PITCH_BAND_DEG = _band(PAIR_LEVEL_PITCH, TH_PITCH_ALPHA_GAMMA_RESID_DEG)
TH_REG_CLIMB_PITCH_BAND_DEG = _band(PAIR_CLIMB_PITCH, TH_PITCH_ALPHA_GAMMA_RESID_DEG)
TH_REG_DESCENT_PITCH_BAND_DEG = _band(PAIR_DESCENT_PITCH, TH_PITCH_ALPHA_GAMMA_RESID_DEG)
TH_REG_CLIMB_STEDOT_MIN_W_PER_KG = _lo(PAIR_CLIMB_STEDOT, TOL_STEDOT_W_PER_KG)
TH_REG_DESCENT_STEDOT_MAX_W_PER_KG = _hi(PAIR_DESCENT_STEDOT, TOL_STEDOT_W_PER_KG)
TH_REG_AIRSPEED_MIN_MS = _lo(PAIR_WF_V_MIN, TOL_AIRSPEED_EXTREME_MS)
TH_REG_AIRSPEED_MAX_MS = _hi(PAIR_WF_V_MAX, TOL_AIRSPEED_EXTREME_MS)
# REPORT-ONLY (see docstring: more surface activity is the INTENDED effect of
# raising the loop's only derivative term, not a performance regression).
REPORT_ELEVATOR_NOT_WORSE_DEG = max(PAIR_ELEVATOR) * (1.0 + S2_DISTURBANCE_REL_SPREAD)

# Measured run-to-run scatter between the two reference realisations. Recorded
# for the record; it is what makes the tolerances above measured rather than
# invented.
REFERENCE_RUN_TO_RUN_SPREAD = {
    "cruise_airspeed_mean_ms": _spread(PAIR_V_MEAN),
    "cruise_airspeed_std_ms": _spread(PAIR_V_STD),
    "hold_vz_max_abs_ms": _spread(PAIR_HOLD_VZ),
    "hold_alt_p2p_max_m": _spread(PAIR_ALT_P2P),
    "achieved_climb_m": _spread(PAIR_CLIMB_M),
    "achieved_descent_magnitude_m": _spread(PAIR_DESCENT_MAG_M),
    "roundtrip_alt_residual_abs_m": _spread(PAIR_ROUNDTRIP_ABS_M),
    "climb_ramp_vz_ms": _spread(PAIR_CLIMB_VZ),
    "climb_ramp_peak_vz_ms": _spread(PAIR_CLIMB_PEAK_VZ),
    "level_throttle": _spread(PAIR_LEVEL_THR),
    "climb_throttle": _spread(PAIR_CLIMB_THR),
    "descent_throttle": _spread(PAIR_DESCENT_THR),
    "level_pitch_deg": _spread(PAIR_LEVEL_PITCH),
    "climb_pitch_deg": _spread(PAIR_CLIMB_PITCH),
    "descent_pitch_deg": _spread(PAIR_DESCENT_PITCH),
    "level_STEdot_W_per_kg": _spread(PAIR_LEVEL_STEDOT),
    "climb_STEdot_W_per_kg": _spread(PAIR_CLIMB_STEDOT),
    "descent_STEdot_W_per_kg": _spread(PAIR_DESCENT_STEDOT),
    "whole_flight_airspeed_min_ms": _spread(PAIR_WF_V_MIN),
    "whole_flight_airspeed_max_ms": _spread(PAIR_WF_V_MAX),
    "whole_flight_elevator_max_abs_deg": _spread(PAIR_ELEVATOR),
}

# =============================================================================
# RUNTIME PARAMETER WRITES (OPT-IN ONLY - see PARAMETER POLICY)
# Identical mechanism and identical whitelist to
# test_ardupilot_longitudinal_phugoid_damping.py; duplicated deliberately so
# this harness carries no import-time dependency on that module. Ranges are
# ArduPilot's OWN documented @Range values, cited per line.
# =============================================================================
SETTABLE_PARAMS = {
    "TECS_PTCH_DAMP": (0.1, 1.0),      # AP_TECS.cpp:101-107  @Range 0.1 1.0
    "TECS_THR_DAMP": (0.1, 1.0),       # AP_TECS.cpp:45-51    @Range 0.1 1.0
    "TECS_INTEG_GAIN": (0.0, 0.5),     # AP_TECS.cpp:53-59    @Range 0.0 0.5
    "TECS_TIME_CONST": (3.0, 10.0),    # AP_TECS.cpp:37-43    @Range 3.0 10.0
    "TECS_SPDWEIGHT": (0.0, 2.0),      # AP_TECS.cpp:93-99    @Range 0.0 2.0
    "TECS_HGT_OMEGA": (1.0, 5.0),      # AP_TECS.cpp:69-75    @Range 1.0 5.0
    "TECS_SPD_OMEGA": (0.5, 2.0),      # AP_TECS.cpp:77-83    @Range 0.5 2.0
    "TECS_HDEM_TCONST": (1.0, 5.0),    # AP_TECS.cpp:285-292  @Range 1.0 5.0
    "TECS_VERT_ACC": (1.0, 10.0),      # AP_TECS.cpp:61-67    @Range 1.0 10.0
}

# Firmware defaults, from AP_TECS.cpp AP_GROUPINFO, cross-checked against the
# LIVE 1367-parameter dumps of the 2026-09-03 and 2026-09-04 runs.
TECS_FIRMWARE_DEFAULTS = {
    "TECS_TIME_CONST": 5.0, "TECS_THR_DAMP": 0.5, "TECS_PTCH_DAMP": 0.3,
    "TECS_INTEG_GAIN": 0.3, "TECS_SPDWEIGHT": 1.0, "TECS_HGT_OMEGA": 3.0,
    "TECS_SPD_OMEGA": 2.0, "TECS_VERT_ACC": 7.0, "TECS_HDEM_TCONST": 3.0,
    "TECS_PTCH_FF_K": 0.0, "TECS_CLMB_MAX": 5.0, "TECS_SINK_MIN": 2.0,
    "TECS_SINK_MAX": 5.0, "TECS_PITCH_MAX": 15.0, "TECS_PITCH_MIN": 0.0,
}

EXTRA_PARAMS = ["TECS_PITCH_MIN", "TECS_PITCH_MAX", "PTCH_LIM_MIN_DEG",
                "PTCH_LIM_MAX_DEG", "FBWB_CLIMB_RATE", "TECS_HDEM_TCONST",
                "TECS_TIME_CONST", "TECS_PTCH_DAMP", "TECS_INTEG_GAIN",
                "TECS_THR_DAMP", "TECS_SPDWEIGHT", "TECS_HGT_OMEGA",
                "TECS_SPD_OMEGA", "TECS_VERT_ACC", "TECS_CLMB_MAX",
                "TECS_SINK_MIN", "TECS_SINK_MAX", "AHRS_EKF_TYPE"]
PARAMS_OF_INTEREST = list(dict.fromkeys(energy.PARAMS_OF_INTEREST + EXTRA_PARAMS))
# energy.dump_params() reads the ENERGY module's own list, so extend that list
# in place with anything this harness additionally needs (a no-op as of
# 2026-09-04 for every name already present there). This mutates a runtime list
# only; it does not modify the energy module file.
for _n in PARAMS_OF_INTEREST:
    if _n not in energy.PARAMS_OF_INTEREST:
        energy.PARAMS_OF_INTEREST.append(_n)


def parse_set_param_args(argv):
    """Parse repeated `--set-param NAME=VALUE`. Returns (list_of_(name,value),
    error_string_or_None). REFUSES any name outside SETTABLE_PARAMS and any
    value outside that parameter's own ArduPilot-documented @Range."""
    out = []
    i = 0
    while i < len(argv):
        if argv[i] != "--set-param":
            i += 1
            continue
        if i + 1 >= len(argv):
            return out, "--set-param given with no NAME=VALUE argument"
        spec = argv[i + 1]
        i += 2
        if "=" not in spec:
            return out, f"--set-param expects NAME=VALUE, got {spec!r}"
        name, _, sval = spec.partition("=")
        name = name.strip().upper()
        if name not in SETTABLE_PARAMS:
            return out, (f"REFUSED: {name} is not in SETTABLE_PARAMS. This harness "
                         f"may only write TECS energy-loop parameters; it can never "
                         f"write a PID, PTCH_TRIM_DEG, a servo mapping, an ARSPD_*, "
                         f"a SIM_* or any aero/propulsion/actuator value. Allowed: "
                         f"{sorted(SETTABLE_PARAMS)}")
        try:
            val = float(sval)
        except ValueError:
            return out, f"REFUSED: {name}={sval!r} is not a number"
        lo, hi = SETTABLE_PARAMS[name]
        if not (lo <= val <= hi):
            return out, (f"REFUSED: {name}={val} is outside ArduPilot's own "
                         f"documented @Range [{lo}, {hi}] for that parameter")
        out.append((name, val))
    return out, None


def parse_tag(argv):
    """`--tag NAME` -> output-filename suffix. Keeps this harness's artifacts
    from ever overwriting the part-1/part-2 phugoid artifacts (which use the
    _baseline / _ptchdamp06 suffixes on a DIFFERENT file prefix) or each
    other's."""
    for i, a in enumerate(argv):
        if a == "--tag" and i + 1 < len(argv):
            t = argv[i + 1].strip().strip("_")
            safe = "".join(ch for ch in t if ch.isalnum() or ch in "-_.")
            return safe
    return ""


def out_paths(tag):
    sfx = f"_{tag}" if tag else ""
    d = base.RESULTS_DIR
    return (os.path.join(d, f"{FILE_PREFIX}_result{sfx}.json"),
            os.path.join(d, f"{FILE_PREFIX}_timeseries{sfx}.json"),
            os.path.join(d, f"{FILE_PREFIX}_per_sample{sfx}.json"))


OUT_JSON, OUT_TS, OUT_TRACE = out_paths("")     # rebound in main()/reanalyze()


def param_set_confirmed(mav, name, value, timeout=6.0):
    """MAVLink PARAM_SET + read-back confirmation. Writes ONLY to the SITL
    scratch EEPROM (arduplane is launched with -w). Never touches
    config/ardupilot/falcon_v2_sitl.parm or any other checked-in file."""
    from pymavlink import mavutil
    before = read_param(mav, name)
    mav.m.mav.param_set_send(mav.m.target_system, mav.m.target_component,
                             name.encode("ascii"), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    deadline = time.time() + timeout
    after = None
    while time.time() < deadline:
        r, _, _ = select.select([mav.m.port], [], [], 0.3)
        if not r:
            continue
        msg = mav.m.recv_match(type="PARAM_VALUE", blocking=False)
        if msg is None:
            continue
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode("ascii", "ignore")
        if pid.rstrip("\x00") == name:
            after = float(msg.param_value)
            break
    if after is None:
        after = read_param(mav, name)
    ok = after is not None and abs(after - float(value)) <= 1e-4
    return dict(name=name, requested=float(value), before=before, after=after,
                confirmed=bool(ok))


def tecs_delta_from_firmware_defaults(p):
    """Which TECS_* values actually differ from the compiled firmware default,
    read from the LIVE parameter dump. Used to prove that a --set-param run
    changed EXACTLY the requested names and nothing else."""
    diff = {}
    for name, dflt in TECS_FIRMWARE_DEFAULTS.items():
        live = p.get(name)
        if live is None:
            diff[name] = dict(live=None, default=dflt, unreadable=True)
        elif abs(live - dflt) > 1e-6:
            diff[name] = dict(live=live, default=dflt)
    return diff


# =============================================================================
# envelope gating over a window (absolute envelope from the stage spec)
# =============================================================================
def envelope_block(samples, p, label, surf_limit_deg):
    """Did the run stay inside the linear, unsaturated acceptance envelope?"""
    out = {"label": label, "surface_limit_used_deg": surf_limit_deg,
           "n_samples": len(samples)}
    if not samples:
        out["insufficient_samples"] = True
        return out
    thr_min_p = (p.get("THR_MIN") or 0.0) / 100.0
    thr_max_p = (p.get("THR_MAX") or 100.0) / 100.0
    asp = [s["mav"]["airspeed"] for s in samples if s["mav"]["airspeed"] is not None]
    _, thr = collect(samples, s_throttle_actual)
    elev = [abs(v) for v in (s_surface_deg(s, n) for s in samples
                             for n in ("left_elevator", "right_elevator"))
            if v is not None]
    lat = [abs(v) for v in (s_surface_deg(s, n) for s in samples
                            for n in ("left_aileron", "right_aileron", "rudder"))
           if v is not None]
    ptd = p.get("PTCH_TRIM_DEG", PTCH_TRIM_DEG_EXPECTED)
    _, pdem = collect(samples, lambda s: s_pitch_demand_phys(s, ptd))
    _, navp = collect(samples, lambda s: s["mav"]["nav_pitch_deg"])

    out["duration_s"] = (samples[-1]["t"] - samples[0]["t"]) if len(samples) >= 2 else None
    out["airspeed_min_ms"] = min(asp) if asp else None
    out["airspeed_max_ms"] = max(asp) if asp else None
    out["airspeed_min_required_ms"] = TH_SPEED_MIN_MS
    out["airspeed_ok"] = bool(asp and min(asp) >= TH_SPEED_MIN_MS)
    out["airspeed_above_hard_floor"] = bool(asp and min(asp) >= TH_SPEED_HARD_FLOOR_MS)
    out["throttle_min"] = min(thr) if thr else None
    out["throttle_max"] = max(thr) if thr else None
    out["throttle_range"] = (max(thr) - min(thr)) if thr else None
    out["thr_min_param"] = thr_min_p
    out["thr_max_param"] = thr_max_p
    out["throttle_sat_high_longest_run_s"] = longest_run_seconds(
        samples, lambda s: (s_throttle_actual(s) is not None
                            and s_throttle_actual(s) >= thr_max_p - TH_SAT_MARGIN))
    out["throttle_sat_low_longest_run_s"] = longest_run_seconds(
        samples, lambda s: (s_throttle_actual(s) is not None
                            and s_throttle_actual(s) <= thr_min_p + TH_SAT_MARGIN))
    out["throttle_sat_run_limit_s"] = TH_SAT_RUN_MAX_S
    out["throttle_no_sustained_saturation"] = (
        max(out["throttle_sat_high_longest_run_s"] or 0.0,
            out["throttle_sat_low_longest_run_s"] or 0.0) <= TH_SAT_RUN_MAX_S)
    out["elevator_max_abs_deg"] = max(elev) if elev else None
    out["elevator_limit_deg"] = surf_limit_deg
    out["elevator_ok"] = bool(elev and max(elev) <= surf_limit_deg)
    out["elevator_below_travel_margin"] = bool(elev and max(elev) <= TH_SURF_MAX_ABS_DEG)
    out["surface_travel_limit_deg"] = SURFACE_TRAVEL_LIMIT_DEG
    out["lateral_surface_max_abs_deg"] = max(lat) if lat else None
    out["lateral_limit_deg"] = TH_LATERAL_SURF_MAX_DEG
    out["lateral_ok"] = bool(lat and max(lat) <= TH_LATERAL_SURF_MAX_DEG)

    # TECS pitch-demand clipping. _PITCHmaxf = TECS_PITCH_MAX if non-zero else
    # PTCH_LIM_MAX_DEG; _PITCHminf = TECS_PITCH_MIN if non-zero else
    # PTCH_LIM_MIN_DEG (AP_TECS.cpp:1488-1500). nav_pitch_cd is clipped AGAIN to
    # [PTCH_LIM_MIN_DEG, PTCH_LIM_MAX_DEG] at ArduPlane/Attitude.cpp:638. The
    # clip acts on the RAW demand; PTCH_TRIM_DEG is added AFTER it
    # (Attitude.cpp:244), so the margin is checked against RAW nav_pitch.
    tpmax, tpmin = p.get("TECS_PITCH_MAX"), p.get("TECS_PITCH_MIN")
    lmax, lmin = p.get("PTCH_LIM_MAX_DEG"), p.get("PTCH_LIM_MIN_DEG")
    eff_max = (tpmax if (tpmax is not None and abs(tpmax) > 1e-9) else lmax)
    eff_min = (tpmin if (tpmin is not None and abs(tpmin) > 1e-9) else lmin)
    if eff_max is not None and lmax is not None:
        eff_max = min(eff_max, lmax)
    if eff_min is not None and lmin is not None:
        eff_min = max(eff_min, lmin)
    out["tecs_pitch_limits_effective_deg"] = dict(
        min=eff_min, max=eff_max,
        source="AP_TECS.cpp:1488-1500 + ArduPlane/Attitude.cpp:638",
        TECS_PITCH_MAX=tpmax, TECS_PITCH_MIN=tpmin,
        PTCH_LIM_MAX_DEG=lmax, PTCH_LIM_MIN_DEG=lmin)
    out["nav_pitch_raw_deg"] = minmaxmean(navp)
    out["pitch_demand_physical_deg"] = minmaxmean(pdem)
    out["pitch_demand_margin_required_deg"] = TH_PITCH_DEMAND_MARGIN_DEG
    if navp and eff_max is not None and eff_min is not None:
        out["nav_pitch_margin_to_max_deg"] = eff_max - max(navp)
        out["nav_pitch_margin_to_min_deg"] = min(navp) - eff_min
        out["pitch_demand_not_clipped"] = bool(
            out["nav_pitch_margin_to_max_deg"] >= TH_PITCH_DEMAND_MARGIN_DEG
            and out["nav_pitch_margin_to_min_deg"] >= TH_PITCH_DEMAND_MARGIN_DEG)
    else:
        out["pitch_demand_not_clipped"] = False
        out["pitch_demand_clip_check_unavailable"] = True

    tgt = eff = 0
    for s in samples:
        if not s["actuators"]:
            continue
        for _, d in s["actuators"].items():
            tgt += 1 if d["target_clamp_active"] else 0
            eff += 1 if d["effort_clamp_active"] else 0
    out["actuator_clamp"] = dict(target_clamp_active_samples=tgt,
                                 effort_clamp_active_samples=eff)
    out["no_actuator_clamping"] = (tgt == 0 and eff == 0)

    bad = 0
    for s in samples:
        for v in (s_alt(s), s_tas(s), s["mav"]["airspeed"], s["mav"]["climb"],
                  s_pitch_phys(s), s_throttle_actual(s), s_elev_deg(s)):
            if v is not None and not math.isfinite(v):
                bad += 1
    out["nonfinite_values"] = bad
    out["all_values_finite"] = (bad == 0)
    out["high_j"] = high_j_block(samples)
    return out


# =============================================================================
# regression gate helper
# =============================================================================
def gate(name, measured, refs, kind, limit, derivation, unit):
    """One NOT-WORSE-THAN comparison against the TWO recorded reference
    realisations (refs = (REF_A value, REF_B value); either may be None).

       kind 'band'    : limit = (lo, hi), lo <= measured <= hi
       kind 'max'     :  measured <= limit
       kind 'min'     :  measured >= limit
       kind 'abs_max' : |measured| <= limit

    A missing measurement is a FAILURE, never a silent pass."""
    ra = refs[0] if isinstance(refs, (tuple, list)) else refs
    rb = refs[1] if isinstance(refs, (tuple, list)) and len(refs) > 1 else None
    vals = [v for v in (ra, rb) if isinstance(v, (int, float))]
    d = dict(metric=name, unit=unit, measured=measured,
             reference_A_2026_09_02=ra, reference_B_2026_09_03=rb,
             reference_run_to_run_spread=((max(vals) - min(vals))
                                          if len(vals) == 2 else None),
             kind=kind, limit=limit, derivation=derivation)
    if measured is None or not isinstance(measured, (int, float)) \
            or not math.isfinite(measured):
        d["ok"] = False
        d["unmeasured"] = True
        return d
    if kind == "band":
        lo, hi = limit
        d["ok"] = (lo is not None and hi is not None and lo <= measured <= hi)
    elif kind == "abs_max":
        d["ok"] = limit is not None and abs(measured) <= limit
    elif kind == "max":
        d["ok"] = limit is not None and measured <= limit
    elif kind == "min":
        d["ok"] = limit is not None and measured >= limit
    else:
        raise ValueError(f"unknown gate kind {kind}")
    if vals:
        d["delta_from_nearest_reference"] = min(measured - v for v in vals) \
            if measured >= max(vals) else (max(measured - v for v in vals)
                                           if measured <= min(vals) else 0.0)
    return d


# =============================================================================
# analysis
# =============================================================================
def analyze(R, segs, p, ptch_trim_deg):
    ptd = ptch_trim_deg
    an = {"phase_plan": dict(
        R1_cruise_s=R1_CRUISE_S, R1_transient_s=R1_TRANSIENT_S,
        R2_climb_max_s=R2_CLIMB_MAX_S, R3_settle_s=R3_SETTLE_S,
        R4_descent_max_s=R4_DESCENT_MAX_S, R5_resettle_s=R5_RESETTLE_S,
        hold_transient_s=HOLD_TRANSIENT_S,
        hold_analysed_window_s=W_HOLD_ANALYSED_S,
        hold_analysed_window_cycles_baseline_mode=N_CYCLES_HOLD,
        ramp_nominal_s=RAMP_NOMINAL_S,
        total_flight_max_s=TOTAL_FLIGHT_MAX_S,
        total_flight_expected_s=TOTAL_FLIGHT_EXPECTED_S,
        prior_campaign_total_s=PRIOR_CAMPAIGN_TOTAL_S,
        alt_step_m=ALT_STEP_M, v_target_ms=V_TARGET_MS,
        rationale="see the module docstring, PHASE PROFILE AND DURATION "
                  "DERIVATION. Every duration is derived from the prior "
                  "stage's measured ramp behaviour and from the closed-loop "
                  "mode period measured in part 1 of this stage.")}
    R["analysis"] = an

    # ---- per-phase and hold windows ---------------------------------------
    for ph in PHASES:
        seg = segs.get(ph)
        an[ph + "_full"] = (energy.analyze_window(seg["samples"], ph + "_full", p, ptd)
                            if seg else None)
    for ph, tr in (("R1_cruise", R1_TRANSIENT_S),
                   ("R3_settle", HOLD_TRANSIENT_S),
                   ("R5_resettle", HOLD_TRANSIENT_S)):
        seg = segs.get(ph)
        if not seg:
            an[ph + "_hold"] = None
            continue
        sub = [s for s in seg["samples"] if s["t_seg"] >= tr]
        an[ph + "_hold"] = energy.analyze_window(sub, ph + "_hold", p, ptd)

    # ---- TECS authority, RE-PROVED LIVE (never inherited) ------------------
    r1 = segs.get("R1_cruise")
    if r1:
        rc3 = r1["rc3"]
        manual_equiv = cruise.control_in_range_no_dz(
            rc3, p["RC3_MIN"], p["RC3_MAX"], bool(p["RC3_REVERSED"])) / 100.0
        a_thr = (an["R1_cruise_hold"].get("throttle_actual")
                 if an.get("R1_cruise_hold") else None)
        an["tecs_authority"] = dict(
            rc3_pwm_us=rc3,
            manual_passthrough_equivalent_throttle=manual_equiv,
            measured_throttle_mean_R1_hold=(a_thr["mean"] if a_thr else None),
            abs_delta=(abs(a_thr["mean"] - manual_equiv) if a_thr else None),
            note="In FBWB the throttle stick sets target AIRSPEED "
                 "(ArduPlane/navigation.cpp:187-189); throttle itself is TECS "
                 "output (Attitude.cpp:510). A large delta proves TECS - not "
                 "the stick - is the throttle authority. RE-PROVED LIVE here.")

    # ---- whole flight ------------------------------------------------------
    allsamp = []
    for ph in PHASES:
        if segs.get(ph):
            allsamp.extend(segs[ph]["samples"])
    allsamp.sort(key=lambda s: s["t"])
    an["whole_flight"] = envelope_block(allsamp, p, "whole_flight",
                                        TH_SURF_FLIGHT_MAX_DEG)
    an["envelope_hold_windows"] = {}
    for ph, tr in (("R1_cruise", R1_TRANSIENT_S),
                   ("R3_settle", HOLD_TRANSIENT_S),
                   ("R5_resettle", HOLD_TRANSIENT_S)):
        seg = segs.get(ph)
        if not seg:
            continue
        sub = [s for s in seg["samples"] if s["t_seg"] >= tr]
        an["envelope_hold_windows"][ph + "_hold"] = envelope_block(
            sub, p, ph + "_hold", TH_SURF_HOLD_MAX_DEG)
    for ph in RAMP_PHASES:
        seg = segs.get(ph)
        if seg:
            an["envelope_" + ph] = envelope_block(seg["samples"], p, ph,
                                                  TH_SURF_FLIGHT_MAX_DEG)

    # ---- helpers -----------------------------------------------------------
    def wm(win, key, sub="mean"):
        d = an.get(win)
        if not d or d.get(key) is None:
            return None
        v = d[key]
        return v.get(sub) if isinstance(v, dict) else v

    def eb(win, key):
        w = an.get(win)
        if not w or not w.get("energy"):
            return None
        return w["energy"].get(key)

    def dd(a, b):
        return (a - b) if (a is not None and b is not None) else None

    # ---- altitude step / round trip ---------------------------------------
    alt1 = wm("R1_cruise_hold", "altitude_gz_m")
    alt3 = wm("R3_settle_hold", "altitude_gz_m")
    alt5 = wm("R5_resettle_hold", "altitude_gz_m")
    tgt1 = wm("R1_cruise_hold", "ap_target_alt_rel_m")
    tgt3 = wm("R3_settle_hold", "ap_target_alt_rel_m")
    tgt5 = wm("R5_resettle_hold", "ap_target_alt_rel_m")
    an["altitude_step"] = dict(
        commanded_step_m=ALT_STEP_M,
        reference_altitude_m=R.get("reference_altitude_m"),
        alt_R1_hold_mean_m=alt1, alt_R3_hold_mean_m=alt3, alt_R5_hold_mean_m=alt5,
        achieved_climb_m=dd(alt3, alt1),
        achieved_descent_m=dd(alt5, alt3),
        roundtrip_residual_m=dd(alt5, alt1),
        ap_target_R1_hold_m=tgt1, ap_target_R3_hold_m=tgt3, ap_target_R5_hold_m=tgt5,
        ap_target_climb_step_m=dd(tgt3, tgt1),
        ap_target_descent_step_m=dd(tgt5, tgt3),
        climb_ramp_duration_s=(segs.get("R2_climb") or {}).get("actual_duration_s"),
        descent_ramp_duration_s=(segs.get("R4_descent") or {}).get("actual_duration_s"),
        climb_ramp_stopped_early=(segs.get("R2_climb") or {}).get("stopped_early"),
        descent_ramp_stopped_early=(segs.get("R4_descent") or {}).get("stopped_early"),
        climb_ramp_stop_reason=(segs.get("R2_climb") or {}).get("stop_reason"),
        descent_ramp_stop_reason=(segs.get("R4_descent") or {}).get("stop_reason"),
        note="ap_target_* is ArduPlane's OWN height demand reconstructed from "
             "GLOBAL_POSITION_INT.relative_alt + NAV_CONTROLLER_OUTPUT.alt_error "
             "(derivation: the 2026-09-03 energy stage module docstring).")

    # ---- coordination (throttle / pitch division of labour) ----------------
    an["coordination"] = dict(
        level_throttle=wm("R1_cruise_hold", "throttle_actual"),
        climb_throttle=wm("R2_climb_full", "throttle_actual"),
        settle_throttle=wm("R3_settle_hold", "throttle_actual"),
        descent_throttle=wm("R4_descent_full", "throttle_actual"),
        resettle_throttle=wm("R5_resettle_hold", "throttle_actual"),
        level_pitch_deg=wm("R1_cruise_hold", "pitch_physical_noseup_deg"),
        climb_pitch_deg=wm("R2_climb_full", "pitch_physical_noseup_deg"),
        settle_pitch_deg=wm("R3_settle_hold", "pitch_physical_noseup_deg"),
        descent_pitch_deg=wm("R4_descent_full", "pitch_physical_noseup_deg"),
        resettle_pitch_deg=wm("R5_resettle_hold", "pitch_physical_noseup_deg"),
        level_nav_pitch_raw_deg=wm("R1_cruise_hold", "nav_pitch_raw_tecs_demand_deg"),
        climb_nav_pitch_raw_deg=wm("R2_climb_full", "nav_pitch_raw_tecs_demand_deg"),
        descent_nav_pitch_raw_deg=wm("R4_descent_full", "nav_pitch_raw_tecs_demand_deg"),
        level_pitch_demand_phys_deg=wm("R1_cruise_hold", "pitch_demand_physical_deg"),
        climb_pitch_demand_phys_deg=wm("R2_climb_full", "pitch_demand_physical_deg"),
        descent_pitch_demand_phys_deg=wm("R4_descent_full", "pitch_demand_physical_deg"),
        level_elevator_deg=wm("R1_cruise_hold", "elevator_deg"),
        climb_elevator_deg=wm("R2_climb_full", "elevator_deg"),
        descent_elevator_deg=wm("R4_descent_full", "elevator_deg"),
        level_vz_ms=wm("R1_cruise_hold", "vfr_hud_climb_ms"),
        climb_vz_mean_ms=wm("R2_climb_full", "vfr_hud_climb_ms", "mean"),
        climb_vz_peak_ms=wm("R2_climb_full", "vfr_hud_climb_ms", "max"),
        descent_vz_mean_ms=wm("R4_descent_full", "vfr_hud_climb_ms", "mean"),
        descent_vz_peak_ms=wm("R4_descent_full", "vfr_hud_climb_ms", "min"),
        note="TECS division of labour: the THROTTLE loop tracks total specific "
             "energy rate STEdot (AP_TECS.cpp:739-772) and the PITCH loop "
             "tracks the specific energy BALANCE SEBdot (AP_TECS.cpp:1031-1096). "
             "Climb must show MORE throttle and MORE nose-up than level; "
             "descent the opposite. Raw nav_pitch and the PTCH_TRIM_DEG-"
             "corrected physical demand are both reported so the convention "
             "cannot be double counted.")

    # ---- energy management -------------------------------------------------
    an["energy_management"] = dict(
        level_STEdot_W_per_kg=eb("R1_cruise_hold", "STEdot_W_per_kg"),
        settle_STEdot_W_per_kg=eb("R3_settle_hold", "STEdot_W_per_kg"),
        resettle_STEdot_W_per_kg=eb("R5_resettle_hold", "STEdot_W_per_kg"),
        climb_STEdot_W_per_kg=eb("R2_climb_full", "STEdot_W_per_kg"),
        descent_STEdot_W_per_kg=eb("R4_descent_full", "STEdot_W_per_kg"),
        climb_SPEdot_W_per_kg=eb("R2_climb_full", "SPEdot_W_per_kg"),
        climb_SKEdot_W_per_kg=eb("R2_climb_full", "SKEdot_W_per_kg"),
        descent_SPEdot_W_per_kg=eb("R4_descent_full", "SPEdot_W_per_kg"),
        descent_SKEdot_W_per_kg=eb("R4_descent_full", "SKEdot_W_per_kg"),
        climb_SEBdot_W_per_kg=eb("R2_climb_full", "SEBdot_W_per_kg"),
        descent_SEBdot_W_per_kg=eb("R4_descent_full", "SEBdot_W_per_kg"),
        definitions="AP_TECS.cpp:678-697 (energies/rates), :1024-1036 (balance). "
                    "Rates are RAW kinematic rates over the window, not TECS's "
                    "high-passed internal state.")

    # ---- descent airspeed excursion ---------------------------------------
    dsamp = list((segs.get("R4_descent") or {}).get("samples", []))
    dsamp += [s for s in (segs.get("R5_resettle") or {}).get("samples", [])
              if s["t_seg"] <= HOLD_TRANSIENT_S]
    dv = [s["mav"]["airspeed"] for s in dsamp if s["mav"]["airspeed"] is not None]
    dtg = [v for v in (energy.s_tecs_target_airspeed(s) for s in dsamp) if v is not None]
    an["descent_speed_excursion"] = dict(
        n_samples=len(dv),
        airspeed_max_ms=max(dv) if dv else None,
        airspeed_min_ms=min(dv) if dv else None,
        tecs_target_mean_ms=mean(dtg) if dtg else None,
        overshoot_above_target_ms=((max(dv) - mean(dtg)) if dv and dtg else None),
        window="R4_descent + the first 10 s of R5_resettle (the recovery transient)",
        note="Guards against an energy-management failure in which the descent "
             "is flown by trading altitude into SPEED instead of reducing "
             "throttle.")

    # ---- oscillation growth in every hold window --------------------------
    grow = {}
    for w in HOLD_WINDOWS:
        d = an.get(w)
        og = (d or {}).get("oscillation_growth")
        if not og:
            grow[w] = None
            continue
        flags = {k: (v.get("growing") if isinstance(v, dict) else None)
                 for k, v in og.items()}
        grow[w] = dict(per_channel=flags,
                       any_growing=any(bool(v) for v in flags.values()))
    an["oscillation_growth_hold_windows"] = grow
    an["no_growing_oscillation"] = bool(grow) and all(
        (v is not None and not v["any_growing"]) for v in grow.values())

    # =========================================================================
    # THE REGRESSION COMPARISON
    # =========================================================================
    m = dict(
        cruise_airspeed_mean_ms=wm("R1_cruise_hold", "airspeed_vfr_hud_ms", "mean"),
        cruise_airspeed_std_ms=wm("R1_cruise_hold", "airspeed_vfr_hud_ms", "std"),
        cruise_tecs_target_ms=wm("R1_cruise_hold", "tecs_target_airspeed_ms", "mean"),
        achieved_climb_m=an["altitude_step"]["achieved_climb_m"],
        achieved_descent_m=an["altitude_step"]["achieved_descent_m"],
        roundtrip_alt_residual_m=an["altitude_step"]["roundtrip_residual_m"],
        climb_ramp_vz_mean_ms=an["coordination"]["climb_vz_mean_ms"],
        climb_ramp_vz_peak_ms=an["coordination"]["climb_vz_peak_ms"],
        descent_ramp_vz_mean_ms=an["coordination"]["descent_vz_mean_ms"],
        descent_ramp_vz_peak_ms=an["coordination"]["descent_vz_peak_ms"],
        level_throttle=an["coordination"]["level_throttle"],
        climb_throttle=an["coordination"]["climb_throttle"],
        descent_throttle=an["coordination"]["descent_throttle"],
        level_pitch_deg=an["coordination"]["level_pitch_deg"],
        climb_pitch_deg=an["coordination"]["climb_pitch_deg"],
        descent_pitch_deg=an["coordination"]["descent_pitch_deg"],
        level_STEdot_W_per_kg=an["energy_management"]["level_STEdot_W_per_kg"],
        climb_STEdot_W_per_kg=an["energy_management"]["climb_STEdot_W_per_kg"],
        descent_STEdot_W_per_kg=an["energy_management"]["descent_STEdot_W_per_kg"],
        whole_flight_airspeed_min_ms=an["whole_flight"].get("airspeed_min_ms"),
        whole_flight_airspeed_max_ms=an["whole_flight"].get("airspeed_max_ms"),
        whole_flight_elevator_max_abs_deg=an["whole_flight"].get("elevator_max_abs_deg"),
    )
    # hold-window statistics: worst (max |.|) across the three hold windows
    vzs, p2ps = [], []
    for w in HOLD_WINDOWS:
        d = an.get(w) or {}
        if d.get("vertical_speed_regression_ms") is not None:
            vzs.append(abs(d["vertical_speed_regression_ms"]))
        if d.get("altitude_p2p_m") is not None:
            p2ps.append(d["altitude_p2p_m"])
    m["hold_vz_max_abs_ms"] = max(vzs) if vzs else None
    m["hold_alt_p2p_max_m"] = max(p2ps) if p2ps else None
    an["regression_measured"] = m

    G = []
    G.append(gate("cruise_airspeed_mean_ms", m["cruise_airspeed_mean_ms"],
                  PAIR_V_MEAN, "band", TH_REG_V_MEAN_BAND_MS,
                  "band over both references +/- 3*S1/sqrt(n_cycles) = "
                  f"{TOL_CRUISE_V_MEAN_MS:.4f} m/s; S1 = REF_A within-window "
                  "airspeed std 0.187 m/s, n_cycles = 24.0/5.6474 = 4.25 "
                  "independent cycles of the measured closed-loop mode", "m/s"))
    G.append(gate("cruise_airspeed_mean_vs_tecs_target_ms",
                  (None if (m["cruise_airspeed_mean_ms"] is None
                            or m["cruise_tecs_target_ms"] is None)
                   else m["cruise_airspeed_mean_ms"] - m["cruise_tecs_target_ms"]),
                  (0.0, None), "abs_max", TH_SPEED_MEAN_TOL_MS,
                  "INHERITED TH_SPEED_MEAN_TOL_MS (cruise stage)", "m/s"))
    G.append(gate("cruise_airspeed_std_ms", m["cruise_airspeed_std_ms"],
                  PAIR_V_STD, "max", TH_REG_V_STD_MAX_MS,
                  f"max(A,B) x (1 + 3/sqrt(2*(n_cycles-1))) = "
                  f"{TH_REG_CRUISE_V_STD_FACTOR:.3f}x: the sampling uncertainty "
                  "of a std estimated from 4.25 effective samples. RESOLUTION "
                  "LIMITATION - with A = 0.187 and B = 0.239 m/s this cannot "
                  "resolve small changes; the inherited absolute gate is the "
                  "binding one. More repeats are DATA_REQUIRED", "m/s"))
    G.append(gate("cruise_airspeed_std_absolute_ms", m["cruise_airspeed_std_ms"],
                  PAIR_V_STD, "max", TH_SPEED_STD_MAX_MS,
                  "INHERITED TH_SPEED_STD_MAX_MS (cruise stage)", "m/s"))
    G.append(gate("hold_vz_max_abs_ms", m["hold_vz_max_abs_ms"], PAIR_HOLD_VZ,
                  "max", TH_REG_HOLD_VZ_MAX_MS,
                  "max(A,B) + 12*A_hold/(w*W^2) = max + 0.0204 m/s: the LSQ "
                  "slope bias the residual oscillation alone can produce over a "
                  "24 s window (A_hold = REF_A hold_alt_p2p_max/2, w = 2pi/T at "
                  "the measured baseline mode period)", "m/s"))
    G.append(gate("hold_vz_absolute_ms", m["hold_vz_max_abs_ms"], PAIR_HOLD_VZ,
                  "max", TH_ALT_SLOPE_MAX_MS,
                  "INHERITED TH_ALT_SLOPE_MAX_MS (cruise stage)", "m/s"))
    G.append(gate("hold_alt_p2p_max_m", m["hold_alt_p2p_max_m"], PAIR_ALT_P2P,
                  "max", TH_REG_ALT_P2P_MAX_M,
                  f"max(A,B) x (1 + S2), S2 = {S2_DISTURBANCE_REL_SPREAD:.4f} = "
                  "REF_B's own two-realisation spread of the peak excursion the "
                  "SAME +/-10 m command produced (3.354222 vs 2.922292 m). "
                  "DECLARED BIAS: p2p is an extreme-value statistic and this "
                  "harness analyses 24 s against the references' 30-33 s, which "
                  "can only bias p2p DOWN - the gate is NECESSARY, not "
                  "SUFFICIENT", "m"))
    G.append(gate("hold_alt_p2p_absolute_m", m["hold_alt_p2p_max_m"], PAIR_ALT_P2P,
                  "max", TH_ALT_P2P_MAX_M,
                  "INHERITED TH_ALT_P2P_MAX_M (cruise stage)", "m"))
    step_derivation = ("band over both references +/- (FBWB integrator "
                       "granularity 2.0*0.15 = 0.30 m + ramp-stop lag peak_vz "
                       "1.951*(0.1 RC refresh + 0.1 FBWB check) = 0.39 m + "
                       "settle-window mean bias 2A/(w*W) = 0.162 m) = 0.853 m")
    G.append(gate("achieved_climb_m", m["achieved_climb_m"], PAIR_CLIMB_M,
                  "band", TH_REG_CLIMB_BAND_M, step_derivation, "m"))
    G.append(gate("achieved_descent_magnitude_m",
                  (abs(m["achieved_descent_m"]) if m["achieved_descent_m"] is not None
                   else None), PAIR_DESCENT_MAG_M, "band",
                  TH_REG_DESCENT_MAG_BAND_M, step_derivation, "m"))
    G.append(gate("achieved_climb_absolute_m",
                  (None if m["achieved_climb_m"] is None
                   else m["achieved_climb_m"] - ALT_STEP_M),
                  (0.0, None), "abs_max", TH_TARGET_STEP_TOL_M,
                  "INHERITED TH_TARGET_STEP_TOL_M (energy stage): the achieved "
                  "step must be within 3.0 m of the 10.0 m command", "m"))
    G.append(gate("achieved_descent_absolute_m",
                  (None if m["achieved_descent_m"] is None
                   else abs(m["achieved_descent_m"]) - ALT_STEP_M),
                  (0.0, None), "abs_max", TH_TARGET_STEP_TOL_M,
                  "INHERITED TH_TARGET_STEP_TOL_M (energy stage)", "m"))
    G.append(gate("roundtrip_alt_residual_abs_m",
                  (abs(m["roundtrip_alt_residual_m"])
                   if m["roundtrip_alt_residual_m"] is not None else None),
                  PAIR_ROUNDTRIP_ABS_M, "max", TH_REG_ROUNDTRIP_MAX_M,
                  "max(|A|,|B|) + (ramp-stop lag 0.39 m + 2 x settle-window "
                  "mean bias 0.162 m) = max + 0.715 m", "m"))
    G.append(gate("roundtrip_alt_residual_absolute_m",
                  m["roundtrip_alt_residual_m"], PAIR_ROUNDTRIP_ABS_M, "abs_max",
                  TH_RESETTLE_TOL_M,
                  "INHERITED TH_RESETTLE_TOL_M (energy stage)", "m"))
    ramp_derivation = ("min(A,B) - 2*A_vz/(w*W_ramp) = min - 0.251 m/s: the mean "
                       "bias the residual oscillation can produce over an 8.67 s "
                       "ramp (A_vz = A_hold*w)")
    G.append(gate("climb_ramp_vz_mean_ms", m["climb_ramp_vz_mean_ms"],
                  PAIR_CLIMB_VZ, "min", TH_REG_CLIMB_VZ_MIN_MS,
                  ramp_derivation, "m/s"))
    G.append(gate("climb_ramp_vz_peak_ms", m["climb_ramp_vz_peak_ms"],
                  PAIR_CLIMB_PEAK_VZ, "min", TH_REG_CLIMB_PEAK_VZ_MIN_MS,
                  ramp_derivation, "m/s"))
    G.append(gate("descent_ramp_vz_mean_magnitude_ms",
                  (-m["descent_ramp_vz_mean_ms"]
                   if m["descent_ramp_vz_mean_ms"] is not None else None),
                  PAIR_DESCENT_VZ_MAG, "min", TH_REG_DESCENT_VZ_MAG_MIN_MS,
                  ramp_derivation + ". ASSUMPTION RAMP_VZ_REFERENCE_IS_CLIMB: "
                  "REF_A records only one ramp figure and it is taken as the "
                  "CLIMB ramp; REF_B measured the descent ramp separately "
                  "(-1.146794 m/s). PROPULSION_HIGH_J_WINDMILLING affects "
                  "descent DRAG (non-gating limitation) - see the high_j blocks",
                  "m/s"))
    G.append(gate("descent_ramp_vz_peak_magnitude_ms",
                  (-m["descent_ramp_vz_peak_ms"]
                   if m["descent_ramp_vz_peak_ms"] is not None else None),
                  PAIR_DESCENT_PEAK_VZ_MAG, "min",
                  TH_REG_DESCENT_PEAK_VZ_MAG_MIN_MS, ramp_derivation, "m/s"))
    G.append(gate("descent_ramp_direction_ms", m["descent_ramp_vz_mean_ms"],
                  (None, REF_B["descent_ramp_vz_ms"]), "max",
                  -TH_RAMP_DIRECTION_MIN_MS,
                  "INHERITED TH_RAMP_DIRECTION_MIN_MS (sign of the ramp)", "m/s"))
    duration_derivation = ("REF_B measured duration x (1 + 0.251/1.1534) = "
                           "x1.218 - the ramp vz uncertainty propagated through "
                           "duration = step / vz. Also proves the ramp did not "
                           "run into its phase cap")
    G.append(gate("climb_ramp_duration_s",
                  an["altitude_step"].get("climb_ramp_duration_s"),
                  (None, REF_B["climb_ramp_duration_s"]), "max",
                  TH_REG_CLIMB_RAMP_DURATION_MAX_S, duration_derivation, "s"))
    G.append(gate("descent_ramp_duration_s",
                  an["altitude_step"].get("descent_ramp_duration_s"),
                  (None, REF_B["descent_ramp_duration_s"]), "max",
                  TH_REG_DESCENT_RAMP_DURATION_MAX_S, duration_derivation, "s"))
    thr_derivation = ("band over both references +/- TH_THROTTLE_TOL (0.05), "
                      "INHERITED from the cruise stage. The oscillation-induced "
                      "mean bias on throttle over these windows is ~0.004, so "
                      "the inherited tolerance dominates")
    for nm, meas, pair, band in (
            ("level_throttle", m["level_throttle"], PAIR_LEVEL_THR, TH_REG_LEVEL_THR_BAND),
            ("climb_throttle", m["climb_throttle"], PAIR_CLIMB_THR, TH_REG_CLIMB_THR_BAND),
            ("descent_throttle", m["descent_throttle"], PAIR_DESCENT_THR,
             TH_REG_DESCENT_THR_BAND)):
        G.append(gate(nm, meas, pair, "band", band, thr_derivation, "-"))
    pitch_derivation = ("band over both references +/- "
                        "TH_PITCH_ALPHA_GAMMA_RESID_DEG (1.5 deg), INHERITED: "
                        "this infrastructure's own demonstrated pitch "
                        "consistency bound (the pitch = alpha + gamma residual), "
                        "i.e. the finest pitch statement the harness can "
                        "support. The ORDERING checks (0.5 deg) are the sharper "
                        "test")
    for nm, meas, pair, band in (
            ("level_pitch_deg", m["level_pitch_deg"], PAIR_LEVEL_PITCH,
             TH_REG_LEVEL_PITCH_BAND_DEG),
            ("climb_pitch_deg", m["climb_pitch_deg"], PAIR_CLIMB_PITCH,
             TH_REG_CLIMB_PITCH_BAND_DEG),
            ("descent_pitch_deg", m["descent_pitch_deg"], PAIR_DESCENT_PITCH,
             TH_REG_DESCENT_PITCH_BAND_DEG)):
        G.append(gate(nm, meas, pair, "band", band, pitch_derivation, "deg"))
    G.append(gate("level_STEdot_W_per_kg", m["level_STEdot_W_per_kg"],
                  PAIR_LEVEL_STEDOT, "abs_max", TH_LEVEL_STEDOT_MAX_W_PER_KG,
                  "INHERITED TH_LEVEL_STEDOT_MAX_W_PER_KG (energy stage)", "W/kg"))
    G.append(gate("climb_STEdot_W_per_kg", m["climb_STEdot_W_per_kg"],
                  PAIR_CLIMB_STEDOT, "min", TH_REG_CLIMB_STEDOT_MIN_W_PER_KG,
                  "min(A,B) - g*0.251 W/kg (the ramp vz mean-bias expressed as "
                  "specific power, g = 9.81 world gravity)", "W/kg"))
    G.append(gate("climb_STEdot_sign_W_per_kg", m["climb_STEdot_W_per_kg"],
                  PAIR_CLIMB_STEDOT, "min", TH_RAMP_STEDOT_MIN_W_PER_KG,
                  "INHERITED TH_RAMP_STEDOT_MIN_W_PER_KG (energy stage)", "W/kg"))
    G.append(gate("descent_STEdot_W_per_kg", m["descent_STEdot_W_per_kg"],
                  PAIR_DESCENT_STEDOT, "max", TH_REG_DESCENT_STEDOT_MAX_W_PER_KG,
                  "max(A,B) + g*0.251 W/kg (same term, opposite sign)", "W/kg"))
    G.append(gate("descent_STEdot_sign_W_per_kg", m["descent_STEdot_W_per_kg"],
                  PAIR_DESCENT_STEDOT, "max", -TH_RAMP_STEDOT_MIN_W_PER_KG,
                  "INHERITED TH_RAMP_STEDOT_MIN_W_PER_KG (energy stage)", "W/kg"))
    G.append(gate("whole_flight_airspeed_min_ms", m["whole_flight_airspeed_min_ms"],
                  PAIR_WF_V_MIN, "min", TH_REG_AIRSPEED_MIN_MS,
                  "min(A,B) - 3*S1 (3-sigma bound on one extreme sample using "
                  "the recorded within-window airspeed scatter). The shorter "
                  "campaign biases extremes INWARD; declared", "m/s"))
    G.append(gate("whole_flight_airspeed_min_absolute_ms",
                  m["whole_flight_airspeed_min_ms"], PAIR_WF_V_MIN, "min",
                  TH_SPEED_MIN_MS,
                  "ABSOLUTE ENVELOPE, stage spec: airspeed >= AIRSPEED_MIN 16 m/s",
                  "m/s"))
    G.append(gate("whole_flight_airspeed_max_ms", m["whole_flight_airspeed_max_ms"],
                  PAIR_WF_V_MAX, "max", TH_REG_AIRSPEED_MAX_MS,
                  "max(A,B) + 3*S1 (same derivation)", "m/s"))
    G.append(gate("whole_flight_elevator_max_abs_deg",
                  m["whole_flight_elevator_max_abs_deg"], PAIR_ELEVATOR, "max",
                  TH_SURF_HOLD_MAX_DEG,
                  "ABSOLUTE ENVELOPE, stage spec: elevator normally <= 10 deg. "
                  "The not-worse-than-reference elevator comparison is REPORTED "
                  "NON-GATING (see regression_report_only): raising "
                  "TECS_PTCH_DAMP raises the loop's only derivative term, so "
                  "more transient surface activity is the INTENDED effect, not a "
                  "performance regression", "deg"))
    an["regression_gates"] = G
    an["regression_gates_failed"] = [g["metric"] for g in G if not g["ok"]]

    an["regression_report_only"] = dict(
        elevator_not_worse_than_deg=REPORT_ELEVATOR_NOT_WORSE_DEG,
        elevator_measured_deg=m["whole_flight_elevator_max_abs_deg"],
        elevator_within_reference_scaled=(
            m["whole_flight_elevator_max_abs_deg"] is not None
            and m["whole_flight_elevator_max_abs_deg"] <= REPORT_ELEVATOR_NOT_WORSE_DEG),
        note="NON-GATING by design - see the gate derivation above.")
    return an


# =============================================================================
# verdict
# =============================================================================
def verdict(R):
    an = R.get("analysis")
    if not an or not an.get("regression_gates"):
        return "TECS_PTCH_DAMP_REGRESSION_FAILED", ["no analysable flight"]
    pre = R.get("param_preconditions", {})
    wf = an.get("whole_flight") or {}
    co = an.get("coordination") or {}
    c = {}

    def num(x):
        return isinstance(x, (int, float)) and math.isfinite(x)

    # --- 1. configuration integrity ----------------------------------------
    modes_ok = True
    for ph in PHASES:
        w = an.get(ph + "_full")
        if not w or w.get("insufficient_samples") or not w.get("all_fbwb"):
            modes_ok = False
    c["mode_is_fbwb_throughout"] = modes_ok
    c["pids_unchanged"] = bool(pre.get("pids_unchanged"))
    c["ptch_trim_deg_unchanged"] = bool(pre.get("ptch_trim_deg_2p49"))
    c["zero_wind_confirmed"] = bool(pre.get("sim_wind_zero"))
    c["atmosphere_datum_ok"] = bool(pre.get("sim_opos_alt_zero_atmosphere_datum"))
    c["fbwb_climb_rate_default"] = bool(pre.get("fbwb_climb_rate_2ms"))
    c["airspeed_params_unchanged"] = all(
        bool(pre.get(k)) for k in ("airspeed_min_16", "airspeed_cruise_18",
                                   "airspeed_max_28"))
    c["surface_travel_scaling_unchanged"] = bool(
        wf.get("surface_travel_limit_deg") == SURFACE_TRAVEL_LIMIT_DEG)

    # --- 2. parameter policy ------------------------------------------------
    writes = R.get("parameter_writes") or []
    requested = sorted({w["name"] for w in writes})
    c["no_parameter_written_unless_explicitly_requested"] = (
        (len(writes) == 0) if not R.get("set_param_requested") else True)
    c["all_requested_parameter_writes_confirmed"] = (
        all(w.get("confirmed") for w in writes) if writes else True)
    diff = R.get("tecs_delta_from_firmware_defaults") or {}
    c["only_requested_tecs_params_differ_from_firmware_defaults"] = (
        sorted(diff.keys()) == requested)
    R["is_firmware_default_baseline"] = (len(writes) == 0 and not diff)

    # --- 3. TECS is genuinely the authority (re-proved live) ----------------
    ta = an.get("tecs_authority") or {}
    c["tecs_is_driving_throttle_not_the_stick"] = (
        num(ta.get("abs_delta")) and ta["abs_delta"] > TH_TECS_AUTHORITY_MIN_DELTA)
    c["throttle_is_actively_modulated"] = (
        num(wf.get("throttle_range")) and wf["throttle_range"] > TH_THROTTLE_MODULATION_MIN)
    tt = (an.get("R1_cruise_hold") or {}).get("tecs_target_airspeed_ms")
    c["tecs_target_airspeed_matches_command"] = (
        isinstance(tt, dict) and num(tt.get("mean"))
        and abs(tt["mean"] - V_TARGET_MS) <= TH_TECS_TARGET_TOL_MS)

    # --- 4. the manoeuvres actually happened --------------------------------
    st = an.get("altitude_step") or {}
    c["climb_ramp_completed_before_cap"] = bool(st.get("climb_ramp_stopped_early"))
    c["descent_ramp_completed_before_cap"] = bool(st.get("descent_ramp_stopped_early"))

    # --- 5. TECS division of labour (inherited ordering checks) -------------
    def ordered(a, b, delta):
        return num(a) and num(b) and (a - b) >= delta
    c["climb_uses_more_throttle_than_level"] = ordered(
        co.get("climb_throttle"), co.get("level_throttle"), TH_COORD_THROTTLE_DELTA)
    c["descent_uses_less_throttle_than_level"] = ordered(
        co.get("level_throttle"), co.get("descent_throttle"), TH_COORD_THROTTLE_DELTA)
    c["climb_is_more_nose_up_than_level"] = ordered(
        co.get("climb_pitch_deg"), co.get("level_pitch_deg"), TH_COORD_PITCH_DELTA_DEG)
    c["descent_is_less_nose_up_than_level"] = ordered(
        co.get("level_pitch_deg"), co.get("descent_pitch_deg"), TH_COORD_PITCH_DELTA_DEG)

    # --- 6. absolute envelope (stage spec) ----------------------------------
    c["whole_flight_airspeed_above_min"] = bool(wf.get("airspeed_ok"))
    c["whole_flight_no_sustained_throttle_saturation"] = bool(
        wf.get("throttle_no_sustained_saturation"))
    c["whole_flight_no_actuator_clamping"] = bool(wf.get("no_actuator_clamping"))
    c["whole_flight_surfaces_below_travel_margin"] = bool(
        wf.get("elevator_below_travel_margin"))
    c["whole_flight_lateral_surfaces_quiet"] = bool(wf.get("lateral_ok"))
    c["whole_flight_pitch_demand_not_clipped"] = bool(wf.get("pitch_demand_not_clipped"))
    c["all_values_finite"] = bool(wf.get("all_values_finite"))
    hold_env = an.get("envelope_hold_windows") or {}
    c["hold_windows_elevator_within_10deg"] = bool(hold_env) and all(
        bool(v.get("elevator_ok")) for v in hold_env.values())
    c["no_growing_oscillation_in_any_hold_window"] = bool(an.get("no_growing_oscillation"))
    dse = an.get("descent_speed_excursion") or {}
    c["no_descent_airspeed_runaway"] = (
        num(dse.get("overshoot_above_target_ms"))
        and dse["overshoot_above_target_ms"] <= TH_DESCENT_SPEED_OVERSHOOT_MAX_MS)

    # --- 7. THE REGRESSION GATES -------------------------------------------
    for g in an["regression_gates"]:
        c["regression_" + g["metric"]] = bool(g["ok"])

    # ---- DECLARED, NON-GATING: carried open limitation ---------------------
    hj = wf.get("high_j") or {}
    R["open_limitations_declared"] = [dict(
        id="PROPULSION_HIGH_J_WINDMILLING", status="OPEN_LIMITATION",
        owner="propulsion", gating=False,
        motor_samples=hj.get("motor_samples"),
        interp_clamped_samples=hj.get("interp_clamped_samples"),
        interp_clamped_fraction=hj.get("interp_clamped_fraction"),
        zero_thrust_samples=hj.get("zero_thrust_samples"),
        advance_ratio_J=hj.get("advance_ratio_J"),
        note="The APC 13x6.5E Ct/Cp table ends at the zero-thrust advance ratio, "
             "so thrust is floored at 0 N where a real fixed-pitch prop would "
             "windmill with NEGATIVE thrust. The DESCENT result of this harness "
             "is a like-for-like comparison between two TECS_PTCH_DAMP settings "
             "under the SAME limitation; it must NOT be presented as absolute "
             "high-fidelity descent performance. DATA_REQUIRED: measured or "
             "extrapolated APC 13x6.5E Ct/Cp beyond the zero-thrust J.")]

    R["acceptance_checks"] = c
    fails = sorted(k for k, v in c.items() if not v)
    vd = ("TECS_PTCH_DAMP_NO_PERFORMANCE_REGRESSION" if not fails
          else "TECS_PTCH_DAMP_REGRESSION_FAILED")
    return vd, fails


# =============================================================================
# per-sample trace (identical column set to the phugoid harness)
# =============================================================================
TRACE_COLUMNS = list(energy.TRACE_COLUMNS) + ["rc2_us", "rc3_us"]


def build_trace(segs, p, ptd):
    w_spe, w_ske = seb_weights(p)
    thr_min_p = (p.get("THR_MIN") or 0.0) / 100.0
    thr_max_p = (p.get("THR_MAX") or 100.0) / 100.0
    rows = []
    for ph in PHASES:
        seg = segs.get(ph)
        if not seg:
            continue
        for s in seg["samples"]:
            pr = s["propulsion"] or {}
            L = pr.get("left", {})
            Rr = pr.get("right", {})
            thr = s_throttle_actual(s)
            clamped = 0
            if s["actuators"]:
                for _, d_ in s["actuators"].items():
                    if d_["target_clamp_active"] or d_["effort_clamp_active"]:
                        clamped = 1
            rows.append([
                ph, s["t"], s.get("t_seg"),
                energy.s_tecs_target_airspeed(s), s["mav"]["airspeed"], s_tas(s),
                energy.s_ap_target_alt_rel_m(s), energy.s_ap_alt_rel_m(s), s_alt(s),
                s["mav"]["climb"],
                s_pitch_phys(s), s["mav"]["nav_pitch_deg"], s_pitch_demand_phys(s, ptd),
                thr, s_elev_deg(s),
                L.get("rpm"), Rr.get("rpm"), L.get("thrust_N"), Rr.get("thrust_N"),
                L.get("J"), Rr.get("J"), L.get("interpClamped"), Rr.get("interpClamped"),
                s_spe(s), s_ske(s), s_ste(s), s_seb(s, w_spe, w_ske),
                (1 if (thr is not None and (thr <= thr_min_p + TH_SAT_MARGIN
                                            or thr >= thr_max_p - TH_SAT_MARGIN)) else 0),
                clamped,
                (math.degrees(s["aero"]["alpha"]) if s["aero"] else None),
                seg["rc2"], seg["rc3"],
            ])
    return rows


# =============================================================================
# flight
# =============================================================================
def flight_sequence(mav, sub, osub, adiag, pdiag, aerodiag, p, R):
    rc3_cruise, achieved_target, err = cruise.rc3_pwm_for_target_airspeed(V_TARGET_MS, p)
    R["command_derivation"] = dict(
        v_target_ms=V_TARGET_MS, rc3_pwm_us=rc3_cruise,
        rc3_pwm_us_rounded=(int(round(rc3_cruise)) if rc3_cruise else None),
        predicted_target_airspeed_ms=achieved_target, error=err,
        rc2_up_us=p.get("RC2_MAX"), rc2_down_us=p.get("RC2_MIN"),
        rc2_neutral_us=p.get("RC2_TRIM"),
        formula="ArduPlane/navigation.cpp:187-189 inverted through "
                "RC_Channel.cpp:388-402 (imported from the 2026-09-02 cruise "
                "stage, unchanged).")
    print("command derivation:", json.dumps(R["command_derivation"], default=str))
    if rc3_cruise is None:
        R["flight_result"] = dict(aborted=True, reason="rc3_derivation_failed")
        return False, {}
    rc3_cruise = int(round(rc3_cruise))
    rc2_neutral = int(round(p["RC2_TRIM"]))
    rc2_up = int(round(p["RC2_MAX"]))
    rc2_down = int(round(p["RC2_MIN"]))

    if not cruise.enter_fbwb(mav, rc3_cruise, R):
        R["flight_result"] = dict(aborted=True, reason="fbwb_not_confirmed")
        return False, {}

    for msg_id in campaign.MSG_IDS_20HZ:
        campaign.request_rate(mav, msg_id, 20.0)
    time.sleep(0.3)

    latest = {}
    t0 = time.time()
    segs = {}
    ptd = p.get("PTCH_TRIM_DEG", PTCH_TRIM_DEG_EXPECTED)

    def run(label, dur, rc2, stop_fn=None):
        return cruise.run_seg(mav, sub, osub, adiag, pdiag, aerodiag, label, dur,
                              1500, rc2, rc3_cruise, t0, latest, stop_fn=stop_fn)

    def fail(ph, seg):
        R["flight_result"] = dict(aborted=True, reason=f"{ph}_aborted",
                                  detail=seg["abort_reason"])

    # ---------- R1: level cruise + altitude hold ---------------------------
    print(f"R1 CRUISE: {R1_CRUISE_S}s level cruise (rc2={rc2_neutral}, "
          f"rc3={rc3_cruise} -> TECS target {achieved_target:.3f} m/s)")
    segs["R1_cruise"] = run("R1_cruise", R1_CRUISE_S, rc2_neutral)
    if segs["R1_cruise"]["aborted"]:
        fail("R1_cruise", segs["R1_cruise"])
        return False, segs

    # REFERENCE ALTITUDE = mean over the SETTLED part of R1 (not the last
    # sample), so the mode phase at the instant R1 ends cannot bias the
    # commanded step or the return-to-origin criterion. Same rule as the
    # prior stage.
    r1_hold = [s for s in segs["R1_cruise"]["samples"] if s["t_seg"] >= R1_TRANSIENT_S]
    _, z_hold = collect(r1_hold, s_alt)
    if len(z_hold) < 4:
        R["flight_result"] = dict(aborted=True, reason="no_R1_reference_altitude")
        return False, segs
    z_ref = mean(z_hold)
    R["reference_altitude_m"] = z_ref
    R["reference_altitude_note"] = (
        f"mean Gazebo z over the settled part of R1 (t_seg >= {R1_TRANSIENT_S} s, "
        f"n={len(z_hold)}). Used as BOTH the climb target (+10 m) and the "
        f"descent target (return to origin).")
    print(f"reference altitude (R1 settled mean): {z_ref:.3f} m")

    # ---------- R2: +10 m climb via the FBWB pitch-stick ramp --------------
    z_climb_target = z_ref + ALT_STEP_M
    st1 = {"n": 0}

    def stop_climb(s, _):
        z = s_alt(s)
        st1["n"] = (st1["n"] + 1) if (z is not None and z >= z_climb_target) else 0
        if st1["n"] >= RAMP_STOP_CONSECUTIVE:
            return True, (f"altitude >= {z_climb_target:.2f} m for "
                          f"{RAMP_STOP_CONSECUTIVE} consecutive samples")
        return False, None

    print(f"R2 CLIMB: up-stick rc2={rc2_up}, cap {R2_CLIMB_MAX_S}s, "
          f"stop at z >= {z_climb_target:.2f} m")
    segs["R2_climb"] = run("R2_climb", R2_CLIMB_MAX_S, rc2_up, stop_fn=stop_climb)
    if segs["R2_climb"]["aborted"]:
        fail("R2_climb", segs["R2_climb"])
        return False, segs

    # ---------- R3: settle at the new altitude -----------------------------
    print(f"R3 SETTLE: {R3_SETTLE_S}s at the new altitude (rc2={rc2_neutral} -> "
          f"set_target_altitude_current(), ArduPlane/navigation.cpp:418-424)")
    segs["R3_settle"] = run("R3_settle", R3_SETTLE_S, rc2_neutral)
    if segs["R3_settle"]["aborted"]:
        fail("R3_settle", segs["R3_settle"])
        return False, segs

    # ---------- R4: -10 m descent, back to the ORIGINAL altitude -----------
    st2 = {"n": 0}

    def stop_descend(s, _):
        z = s_alt(s)
        st2["n"] = (st2["n"] + 1) if (z is not None and z <= z_ref) else 0
        if st2["n"] >= RAMP_STOP_CONSECUTIVE:
            return True, (f"altitude <= original reference {z_ref:.2f} m for "
                          f"{RAMP_STOP_CONSECUTIVE} consecutive samples")
        return False, None

    print(f"R4 DESCENT: down-stick rc2={rc2_down}, cap {R4_DESCENT_MAX_S}s, "
          f"stop at z <= {z_ref:.2f} m (the ORIGINAL altitude)")
    segs["R4_descent"] = run("R4_descent", R4_DESCENT_MAX_S, rc2_down,
                             stop_fn=stop_descend)
    if segs["R4_descent"]["aborted"]:
        fail("R4_descent", segs["R4_descent"])
        return False, segs

    # ---------- R5: re-settle near the ORIGINAL altitude -------------------
    print(f"R5 RESETTLE: {R5_RESETTLE_S}s near the original altitude "
          f"(rc2={rc2_neutral})")
    segs["R5_resettle"] = run("R5_resettle", R5_RESETTLE_S, rc2_neutral)

    aborted = any(v["aborted"] for v in segs.values())
    R["flight_result"] = dict(
        aborted=aborted,
        reference_altitude_m=z_ref,
        climb_target_altitude_m=z_climb_target,
        descent_target_altitude_m=z_ref,
        total_flight_s=sum(v.get("actual_duration_s") or 0.0 for v in segs.values()),
        segment_summary=[(k, v["n_samples"], v["aborted"], v["stopped_early"],
                          v["stop_reason"]) for k, v in segs.items()])
    if not aborted:
        analyze(R, segs, p, ptd)
    return not aborted, segs


# =============================================================================
# I/O
# =============================================================================
def write_outputs(R, segs, p=None):
    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    ts_doc = {"stage": STAGE, "part": PART, "timestamp": R.get("timestamp"),
              "tecs_baseline_params_live": R.get("tecs_baseline_params_live"),
              "command_derivation": R.get("command_derivation"),
              "reference_altitude_m": R.get("reference_altitude_m"),
              "parameter_writes": R.get("parameter_writes"),
              "set_param_requested": R.get("set_param_requested"),
              "segments": {k: {kk: vv for kk, vv in v.items()} for k, v in segs.items()}}
    with open(OUT_TS, "w") as f:
        json.dump(ts_doc, f, default=str, separators=(",", ":"))
    if segs and (p or R.get("tecs_baseline_params_live")):
        pp = p or R["tecs_baseline_params_live"]
        try:
            trace = build_trace(segs, pp, pp.get("PTCH_TRIM_DEG", PTCH_TRIM_DEG_EXPECTED))
            with open(OUT_TRACE, "w") as f:
                json.dump({"stage": STAGE, "part": PART, "columns": TRACE_COLUMNS,
                           "units_note": "SI throughout; angles in deg where the "
                                         "column name says deg; energies J/kg. "
                                         "altitude_gz_m is Gazebo world z (FLU +Z "
                                         "up); ap_alt_rel_m / ap_target_alt_rel_m "
                                         "are ArduPlane altitudes above HOME. "
                                         "nav_pitch_raw_deg is TECS's raw demand; "
                                         "pitch_demand_physical_deg adds "
                                         "PTCH_TRIM_DEG (Attitude.cpp:244).",
                           "rows": trace}, f, default=str, separators=(",", ":"))
        except Exception as exc:      # noqa: BLE001 - reported, never silent
            R["per_sample_trace_error"] = str(exc)
    slim = dict(R)
    slim["segments_summary"] = {
        k: {kk: vv for kk, vv in v.items() if kk != "samples"} for k, v in segs.items()}
    slim["timeseries_file"] = OUT_TS
    slim["per_sample_trace_file"] = OUT_TRACE
    with open(OUT_JSON, "w") as f:
        json.dump(slim, f, indent=2, default=str)


def finish_fail(R, phase, mav, segs=None):
    R["overall_result"] = "TEST_FAILED"
    R["verdict"] = "TECS_PTCH_DAMP_REGRESSION_FAILED"
    R["blocking_phase"] = phase
    write_outputs(R, segs or {})
    print(f"FAILED at {phase} - see", OUT_JSON)
    if mav is not None:
        mav.close()
    return 1


def provenance_block():
    """STATIC declarations that MUST appear whether the artifact came from a
    flight or from --reanalyze (validation finding V-13 of the energy stage).
    Contains no flight measurement."""
    return {
        "part": PART,
        "purpose": ("SHORT performance regression for a candidate TECS_PTCH_DAMP "
                    "value. NOT-WORSE-THAN against the RECORDED measurements of "
                    "ARDUPLANE_TECS_CLIMB_DESCENT_AND_ENERGY_VALIDATION "
                    "(2026-09-03/04). Never gates on beating the reference."),
        "mode": dict(name="FBWB", custom_mode=ARDUPLANE_FBWB_CUSTOM_MODE,
                     evidence="docs/source_of_truth/controls/"
                              "ardupilot_fbwb_tecs_baseline.yaml. TECS throttle "
                              "authority is RE-PROVED LIVE here "
                              "(tecs_is_driving_throttle_not_the_stick), never "
                              "inherited on trust."),
        "launch_sequence": ("inherited verbatim from "
                            "run_ardupilot_tecs_climb_descent_energy.sh: same "
                            "world, same READ-ONLY config/ardupilot/"
                            "falcon_v2_sitl.parm, same env, same gdb-wrapped "
                            "arduplane, same cleanup/trap, same SIM_OPOS origin "
                            "handling (-O 0,0,0,0 / CMAC 584 m trap avoided AND "
                            "gated), same zero-wind gating."),
        "parameter_policy": (
            "DEFAULT: writes NO parameter of any kind - TECS runs on ArduPlane "
            "compiled firmware defaults, so a no-flag run of this same harness "
            "IS the defaults baseline. config/ardupilot/falcon_v2_sitl.parm is "
            "READ-ONLY input and is never edited; no TECS default change is "
            "written to any checked-in file. `--set-param NAME=VALUE` performs a "
            "RUNTIME MAVLink PARAM_SET in the SITL scratch EEPROM only, "
            "restricted to SETTABLE_PARAMS (TECS energy-loop parameters) and to "
            "each parameter's own ArduPilot @Range; every other name is REFUSED. "
            "No PID, no PTCH_TRIM_DEG, no +/-45 deg surface scaling, no aero/"
            "propulsion/actuator/sensor/mass/CG/inertia value can be written "
            "through this harness. AUTOTUNE is never run; LOITER/AUTO/RTL are "
            "never entered."),
        "read_only_inputs": [
            "tests/gazebo/scripts/test_ardupilot_tecs_climb_descent_energy.py "
            "(imported for its analysis functions and its recorded reference "
            "measurements; NOT modified)",
            "tests/gazebo/scripts/test_ardupilot_tecs_cruise_speed_hold.py",
            "config/ardupilot/falcon_v2_sitl.parm",
        ],
        "open_limitations": ["PROPULSION_HIGH_J_WINDMILLING"],
        "open_limitations_note": (
            "PROPULSION_HIGH_J_WINDMILLING (owner: propulsion, DATA_REQUIRED) is "
            "carried unchanged and is NON-GATING. Interp-clamped and zero-thrust "
            "motor-sample counts are recorded per window and for the whole "
            "flight. The descent result is a like-for-like comparison between "
            "two TECS_PTCH_DAMP settings under the same limitation and is NOT "
            "absolute high-fidelity descent performance."),
        "assumptions": [
            "RAMP_VZ_REFERENCE_IS_CLIMB - the recorded ramp_vz 1.301 / "
            "ramp_peak_vz 1.951 m/s are treated as the CLIMB ramp; the same "
            "magnitude floor is applied to the descent ramp because it is "
            "commanded by the mirrored stick through the same FBWB_CLIMB_RATE.",
            "TWO_REFERENCE_REALISATIONS - the gate limits use the min/max over "
            "the TWO recorded runs of this configuration (REF_A 2026-09-02, "
            "REF_B 2026-09-03/04) widened by an independently derived estimator "
            "tolerance. n = 2 is the entire run-to-run evidence that exists; a "
            "proper reproducibility distribution is DATA_REQUIRED (repeat runs). "
            "Gating against REF_A alone would REJECT REF_B, a known-good "
            "validated baseline, on achieved_descent and hold altitude p2p.",
            "SHORTER_WINDOW_BIASES_EXTREMES_INWARD - this harness analyses 24 s "
            "hold windows and ~121 s of flight against references measured over "
            "30-33 s windows and 143-165 s of flight, so extreme-value "
            "statistics (altitude p2p, whole-flight airspeed min/max, max "
            "elevator) are biased toward the candidate. Declared, recorded, and "
            "NOT compensated by loosening any gate.",
        ],
        "reference_constants": dict(
            MASS_KG=MASS_KG, S_REF_M2=S_REF_M2, G_WORLD=G_WORLD, G_TECS=G_TECS,
            V_TRIM_REF=V_TRIM_REF, TRIM_THROTTLE_REF=TRIM_THROTTLE_REF,
            ELEV_TRIM_DEG_REF=ELEV_TRIM_DEG_REF,
            PTCH_TRIM_DEG_EXPECTED=PTCH_TRIM_DEG_EXPECTED,
            SURFACE_TRAVEL_LIMIT_DEG=SURFACE_TRAVEL_LIMIT_DEG,
            V_TARGET_MS=V_TARGET_MS, ALT_STEP_M=ALT_STEP_M,
            reference_A_2026_09_02_cruise_campaign=REF_A,
            reference_B_2026_09_03_energy_campaign=REF_B,
            reference_run_to_run_spread=REFERENCE_RUN_TO_RUN_SPREAD,
            prior_stage_extra=dict(A0_alt_P3_m=REF_A0_ALT_P3_M,
                                   A0_alt_P5_m=REF_A0_ALT_P5_M,
                                   climb_ramp_duration_s=REF_CLIMB_RAMP_DURATION_S),
            closed_loop_mode_measured_this_stage=dict(
                period_baseline_s=MODE_PERIOD_BASELINE_S,
                period_ptchdamp06_s=MODE_PERIOD_PTCHDAMP06_S,
                tau_baseline_s=MODE_TAU_BASELINE_S,
                omega_baseline_rad_s=MODE_OMEGA_BASELINE_RAD_S)),
        "tecs_firmware_defaults_expected": TECS_FIRMWARE_DEFAULTS,
        "settable_params_whitelist": {k: dict(range=list(v))
                                      for k, v in SETTABLE_PARAMS.items()},
    }


def threshold_block():
    return dict(
        INHERITED=dict(
            TH_SPEED_MIN_MS=TH_SPEED_MIN_MS,
            TH_SPEED_HARD_FLOOR_MS=TH_SPEED_HARD_FLOOR_MS,
            TH_SPEED_MEAN_TOL_MS=TH_SPEED_MEAN_TOL_MS,
            TH_SPEED_STD_MAX_MS=TH_SPEED_STD_MAX_MS,
            TH_TECS_TARGET_TOL_MS=TH_TECS_TARGET_TOL_MS,
            TH_ALT_SLOPE_MAX_MS=TH_ALT_SLOPE_MAX_MS,
            TH_ALT_P2P_MAX_M=TH_ALT_P2P_MAX_M,
            TH_THROTTLE_TOL=TH_THROTTLE_TOL,
            TH_SAT_RUN_MAX_S=TH_SAT_RUN_MAX_S, TH_SAT_MARGIN=TH_SAT_MARGIN,
            TH_TECS_AUTHORITY_MIN_DELTA=TH_TECS_AUTHORITY_MIN_DELTA,
            TH_THROTTLE_MODULATION_MIN=TH_THROTTLE_MODULATION_MIN,
            TH_SURF_HOLD_MAX_DEG=TH_SURF_HOLD_MAX_DEG,
            TH_SURF_FLIGHT_MAX_DEG=TH_SURF_FLIGHT_MAX_DEG,
            TH_LATERAL_SURF_MAX_DEG=TH_LATERAL_SURF_MAX_DEG,
            TH_SURF_MAX_ABS_DEG=TH_SURF_MAX_ABS_DEG,
            TH_COORD_THROTTLE_DELTA=TH_COORD_THROTTLE_DELTA,
            TH_COORD_PITCH_DELTA_DEG=TH_COORD_PITCH_DELTA_DEG,
            TH_PITCH_ALPHA_GAMMA_RESID_DEG=TH_PITCH_ALPHA_GAMMA_RESID_DEG,
            TH_RAMP_DIRECTION_MIN_MS=TH_RAMP_DIRECTION_MIN_MS,
            TH_TARGET_STEP_TOL_M=TH_TARGET_STEP_TOL_M,
            TH_RESETTLE_TOL_M=TH_RESETTLE_TOL_M,
            TH_LEVEL_STEDOT_MAX_W_PER_KG=TH_LEVEL_STEDOT_MAX_W_PER_KG,
            TH_RAMP_STEDOT_MIN_W_PER_KG=TH_RAMP_STEDOT_MIN_W_PER_KG,
            TH_DESCENT_SPEED_OVERSHOOT_MAX_MS=TH_DESCENT_SPEED_OVERSHOOT_MAX_MS,
            HOLD_TRANSIENT_S=HOLD_TRANSIENT_S, R1_TRANSIENT_S=R1_TRANSIENT_S,
            source="tests/gazebo/scripts/test_ardupilot_tecs_climb_descent_energy.py "
                   "(2026-09-03, READ-ONLY) and test_ardupilot_tecs_cruise_speed_"
                   "hold.py (2026-09-02). NO inherited threshold is changed here."),
        SCATTER_SOURCES=dict(
            S1_airspeed_std_ms=S1_AIRSPEED_STD_MS,
            S2_disturbance_relative_spread=S2_DISTURBANCE_REL_SPREAD,
            A_hold_m=A_HOLD_M, A_vz_hold_ms=A_VZ_HOLD_MS,
            mode_omega_rad_s=MODE_OMEGA_BASELINE_RAD_S,
            mean_bias_hold_m=MEAN_BIAS_HOLD_M,
            mean_bias_ramp_vz_ms=MEAN_BIAS_RAMP_VZ_MS,
            slope_bias_hold_ms=SLOPE_BIAS_HOLD_MS,
            fbwb_integrator_granularity_m=FBWB_INTEGRATOR_GRANULARITY_M,
            ramp_stop_lag_m=RAMP_STOP_LAG_M,
            n_cycles_hold=N_CYCLES_HOLD,
            note="mean bias <= 2A/(w*W) and LSQ slope bias <= 12A/(w*W^2) are the "
                 "exact worst-case values of those linear functionals for a "
                 "cosine of unknown phase over a window of length W."),
        DERIVED_REGRESSION_TOLERANCES=dict(
            TOL_CRUISE_V_MEAN_MS=TOL_CRUISE_V_MEAN_MS,
            TH_REG_CRUISE_V_STD_FACTOR=TH_REG_CRUISE_V_STD_FACTOR,
            TOL_HOLD_VZ_MS=TOL_HOLD_VZ_MS,
            TOL_STEP_M=TOL_STEP_M,
            TOL_ROUNDTRIP_M=TOL_ROUNDTRIP_M,
            TOL_RAMP_VZ_MS=TOL_RAMP_VZ_MS,
            TOL_RAMP_DURATION_FACTOR=TOL_RAMP_DURATION_FACTOR,
            TOL_STEDOT_W_PER_KG=TOL_STEDOT_W_PER_KG,
            TOL_AIRSPEED_EXTREME_MS=TOL_AIRSPEED_EXTREME_MS,
            gate_form="band: min(A,B)-TOL <= x <= max(A,B)+TOL | max: x <= "
                      "max(A,B)+TOL | min: x >= min(A,B)-TOL. min/max over the "
                      "two recorded realisations already carries the run-to-run "
                      "scatter, so TOL is added once and only once.",
            limits=dict(
                TH_REG_V_MEAN_BAND_MS=list(TH_REG_V_MEAN_BAND_MS),
                TH_REG_V_STD_MAX_MS=TH_REG_V_STD_MAX_MS,
                TH_REG_HOLD_VZ_MAX_MS=TH_REG_HOLD_VZ_MAX_MS,
                TH_REG_ALT_P2P_MAX_M=TH_REG_ALT_P2P_MAX_M,
                TH_REG_CLIMB_BAND_M=list(TH_REG_CLIMB_BAND_M),
                TH_REG_DESCENT_MAG_BAND_M=list(TH_REG_DESCENT_MAG_BAND_M),
                TH_REG_ROUNDTRIP_MAX_M=TH_REG_ROUNDTRIP_MAX_M,
                TH_REG_CLIMB_VZ_MIN_MS=TH_REG_CLIMB_VZ_MIN_MS,
                TH_REG_CLIMB_PEAK_VZ_MIN_MS=TH_REG_CLIMB_PEAK_VZ_MIN_MS,
                TH_REG_DESCENT_VZ_MAG_MIN_MS=TH_REG_DESCENT_VZ_MAG_MIN_MS,
                TH_REG_DESCENT_PEAK_VZ_MAG_MIN_MS=TH_REG_DESCENT_PEAK_VZ_MAG_MIN_MS,
                TH_REG_CLIMB_RAMP_DURATION_MAX_S=TH_REG_CLIMB_RAMP_DURATION_MAX_S,
                TH_REG_DESCENT_RAMP_DURATION_MAX_S=TH_REG_DESCENT_RAMP_DURATION_MAX_S,
                TH_REG_LEVEL_THR_BAND=list(TH_REG_LEVEL_THR_BAND),
                TH_REG_CLIMB_THR_BAND=list(TH_REG_CLIMB_THR_BAND),
                TH_REG_DESCENT_THR_BAND=list(TH_REG_DESCENT_THR_BAND),
                TH_REG_LEVEL_PITCH_BAND_DEG=list(TH_REG_LEVEL_PITCH_BAND_DEG),
                TH_REG_CLIMB_PITCH_BAND_DEG=list(TH_REG_CLIMB_PITCH_BAND_DEG),
                TH_REG_DESCENT_PITCH_BAND_DEG=list(TH_REG_DESCENT_PITCH_BAND_DEG),
                TH_REG_CLIMB_STEDOT_MIN_W_PER_KG=TH_REG_CLIMB_STEDOT_MIN_W_PER_KG,
                TH_REG_DESCENT_STEDOT_MAX_W_PER_KG=TH_REG_DESCENT_STEDOT_MAX_W_PER_KG,
                TH_REG_AIRSPEED_MIN_MS=TH_REG_AIRSPEED_MIN_MS,
                TH_REG_AIRSPEED_MAX_MS=TH_REG_AIRSPEED_MAX_MS,
                REPORT_ELEVATOR_NOT_WORSE_DEG=REPORT_ELEVATOR_NOT_WORSE_DEG),
            provenance="each derived in code from a SCATTER_SOURCE above or from "
                       "an INHERITED threshold - see the module docstring, "
                       "ACCEPTANCE CRITERIA, for the per-metric narrative"),
        REFERENCE_A_2026_09_02=REF_A,
        REFERENCE_B_2026_09_03=REF_B,
        REFERENCE_RUN_TO_RUN_SPREAD=REFERENCE_RUN_TO_RUN_SPREAD,
        PHASE_PLAN=dict(
            R1_CRUISE_S=R1_CRUISE_S, R1_TRANSIENT_S=R1_TRANSIENT_S,
            R2_CLIMB_MAX_S=R2_CLIMB_MAX_S, R3_SETTLE_S=R3_SETTLE_S,
            R4_DESCENT_MAX_S=R4_DESCENT_MAX_S, R5_RESETTLE_S=R5_RESETTLE_S,
            W_HOLD_ANALYSED_S=W_HOLD_ANALYSED_S, N_CYCLES_HOLD=N_CYCLES_HOLD,
            RAMP_NOMINAL_S=RAMP_NOMINAL_S,
            TOTAL_FLIGHT_MAX_S=TOTAL_FLIGHT_MAX_S,
            TOTAL_FLIGHT_EXPECTED_S=TOTAL_FLIGHT_EXPECTED_S,
            PRIOR_CAMPAIGN_TOTAL_S=PRIOR_CAMPAIGN_TOTAL_S))


def param_precondition_checks(p, R):
    """The inherited energy-stage preconditions, plus the TECS-delta accounting
    this harness needs so a --set-param run can be proved to have changed
    EXACTLY the requested names."""
    chk = dict(energy.param_precondition_checks(p, R))
    diff = tecs_delta_from_firmware_defaults(p)
    R["tecs_delta_from_firmware_defaults"] = diff
    writes = R.get("parameter_writes") or []
    if writes:
        # The inherited "tecs_at_firmware_defaults" precondition will correctly
        # be False for a candidate run. It is NOT used as a gate here; the gate
        # is only_requested_tecs_params_differ_from_firmware_defaults.
        R["param_precondition_override_note"] = (
            "tecs_at_firmware_defaults is expected False for this run because "
            f"--set-param deliberately wrote {[w['name'] for w in writes]}. This "
            "run is NOT a firmware-default baseline; the acceptance check "
            "only_requested_tecs_params_differ_from_firmware_defaults proves "
            "that nothing ELSE changed, and the before/after values are in "
            "parameter_writes.")
    R["param_preconditions"] = chk
    print("param preconditions:", json.dumps(chk, default=str))
    print("TECS delta from firmware defaults:", json.dumps(diff, default=str))
    return chk


def reanalyze(path):
    with open(path) as f:
        doc = json.load(f)
    segs = doc["segments"]
    p = doc["tecs_baseline_params_live"]
    R = {"stage": STAGE, "timestamp": doc.get("timestamp"),
         **provenance_block(),
         "tecs_baseline_params_live": p,
         "command_derivation": doc.get("command_derivation"),
         "reference_altitude_m": doc.get("reference_altitude_m"),
         "parameter_writes": doc.get("parameter_writes"),
         "set_param_requested": doc.get("set_param_requested"),
         "thresholds": threshold_block(), "reanalyzed_from": path,
         "provenance_blocks_source": (
             "purpose / mode / launch_sequence / parameter_policy / "
             "open_limitations / assumptions / reference_constants are STATIC "
             "declarations regenerated from this module by provenance_block(); "
             "they are not flight measurements and are not read from the "
             "timeseries file.")}
    param_precondition_checks(p, R)
    analyze(R, segs, p, p.get("PTCH_TRIM_DEG", PTCH_TRIM_DEG_EXPECTED))
    vd, fails = verdict(R)
    R["verdict"] = vd
    R["failed_checks"] = fails
    R["overall_result"] = "REANALYZED"
    write_outputs(R, segs, p)
    print_summary(R)
    print("verdict:", vd)
    print("failed_checks:", fails)
    return 0


def print_summary(R):
    an = R.get("analysis") or {}
    try:
        print("-" * 78)
        pw = R.get("parameter_writes") or []
        print(f"parameter writes: {pw if pw else 'NONE (firmware-defaults baseline)'}")
        print(f"TECS delta vs firmware defaults: "
              f"{json.dumps(R.get('tecs_delta_from_firmware_defaults'), default=str)}")
        st = an.get("altitude_step") or {}
        print(f"altitude step   : climb {st.get('achieved_climb_m')} m in "
              f"{st.get('climb_ramp_duration_s')} s | descent "
              f"{st.get('achieved_descent_m')} m in "
              f"{st.get('descent_ramp_duration_s')} s | roundtrip "
              f"{st.get('roundtrip_residual_m')} m")
        print("-" * 78)
        print(f"{'METRIC':40s} {'MEAS':>10s} {'REF_A':>9s} {'REF_B':>9s} "
              f"{'LIMIT':>21s}  OK")
        for g in an.get("regression_gates", []):
            def f(x, w=10):
                return (f"{x:{w}.4f}" if isinstance(x, (int, float))
                        else f"{'-':>{w}s}")
            lim = g["limit"]
            ls = (f"[{f(lim[0], 9)},{f(lim[1], 9)}]"
                  if isinstance(lim, (tuple, list)) else f"{f(lim, 21)}")
            print(f"{g['metric']:40s} {f(g['measured'])} "
                  f"{f(g['reference_A_2026_09_02'], 9)} "
                  f"{f(g['reference_B_2026_09_03'], 9)} {ls}  "
                  f"{'PASS' if g['ok'] else 'FAIL'}")
        print("-" * 78)
        wf = an.get("whole_flight") or {}
        print(f"whole flight    : {wf.get('duration_s')} s | V "
              f"{wf.get('airspeed_min_ms')}..{wf.get('airspeed_max_ms')} m/s | thr "
              f"{wf.get('throttle_min')}..{wf.get('throttle_max')} | elev_max "
              f"{wf.get('elevator_max_abs_deg')} deg | finite "
              f"{wf.get('all_values_finite')}")
        hj = wf.get("high_j") or {}
        print(f"HIGH-J (OPEN_LIMITATION, NON-GATING): interpClamped "
              f"{hj.get('interp_clamped_samples')}/{hj.get('motor_samples')} "
              f"motor-samples, zero-thrust {hj.get('zero_thrust_samples')}, "
              f"J {hj.get('advance_ratio_J')}")
        print("-" * 78)
    except Exception as exc:      # summary print only - JSON is authoritative
        print("summary print failed:", exc)


def main():
    global OUT_JSON, OUT_TS, OUT_TRACE
    argv = sys.argv[1:]
    tag = parse_tag(argv)
    OUT_JSON, OUT_TS, OUT_TRACE = out_paths(tag)

    if "--reanalyze" in argv:
        i = argv.index("--reanalyze")
        if i + 1 >= len(argv):
            print("ERROR: --reanalyze needs a timeseries json path", file=sys.stderr)
            return 2
        return reanalyze(argv[i + 1])

    set_params, perr = parse_set_param_args(argv)
    if perr:
        print("ERROR:", perr, file=sys.stderr)
        return 2

    R = {"stage": STAGE,
         "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         **provenance_block(),
         "output_tag": tag,
         "set_param_requested": [f"{n}={v}" for n, v in set_params],
         "thresholds": threshold_block()}
    print(f"outputs: {OUT_JSON}")
    if set_params:
        print("NOTE: --set-param requested (runtime PARAM_SET, scratch EEPROM "
              "only):", R["set_param_requested"])
    else:
        print("NOTE: no --set-param; this run is the FIRMWARE-DEFAULTS baseline "
              "of this harness.")

    node = tp.Node()
    sub = base.PoseSub(base.WORLD)
    osub = base.OdomSub()
    time.sleep(0.5)
    pub_oneshot = node.advertise(f"/world/{base.WORLD}/wrench", entity_wrench_pb2.EntityWrench)
    pub_clear = node.advertise(f"/world/{base.WORLD}/wrench/clear", entity_pb2.Entity)
    time.sleep(0.3)

    adiag = actuator_lib.DiagSubscriber()
    pdiag = propulsion_lib.DiagSubscriber()
    aerodiag = aero_lib.DiagSubscriber()
    time.sleep(0.5)

    mav, armed = base.phase1_mavlink_arm(R)
    print("PHASE 1 (mavlink arm):",
          json.dumps(R.get("phase1_mavlink_arm", {}), default=str)[:300])
    if not armed:
        return finish_fail(R, "phase1_mavlink_arm", mav)

    # OPT-IN parameter writes happen BEFORE the param dump, so the dump records
    # what actually flew.
    if set_params:
        writes = []
        for name, val in set_params:
            w = param_set_confirmed(mav, name, val)
            print("PARAM_SET:", json.dumps(w, default=str))
            writes.append(w)
        R["parameter_writes"] = writes
        if not all(w["confirmed"] for w in writes):
            return finish_fail(R, "param_set_not_confirmed", mav)
    else:
        R["parameter_writes"] = []

    p = energy.dump_params(mav, R)
    param_precondition_checks(p, R)
    if not base.is_armed(mav):
        base.arm(mav)
        R["rearmed_after_param_dump"] = base.is_armed(mav)
        print("re-armed after param dump:", R["rearmed_after_param_dump"])
        if not R["rearmed_after_param_dump"]:
            return finish_fail(R, "rearm_after_param_dump", mav)
    required = ["AIRSPEED_MIN", "AIRSPEED_MAX", "RC3_MIN", "RC3_MAX", "RC3_DZ",
                "RC3_REVERSED", "RC2_MIN", "RC2_MAX", "RC2_TRIM", "THR_MIN",
                "THR_MAX", "TECS_SPDWEIGHT", "TECS_TIME_CONST", "TECS_PTCH_DAMP",
                "TECS_INTEG_GAIN", "PTCH_LIM_MIN_DEG", "TECS_PITCH_MAX",
                "FBWB_CLIMB_RATE", "ALT_OFFSET"]
    if any(p.get(k) is None for k in required):
        R["missing_required_params"] = [k for k in required if p.get(k) is None]
        return finish_fail(R, "param_dump_incomplete", mav)

    settled, elapsed = base.wait_ground_settle(osub)
    R["ground_settle"] = dict(settled=settled, elapsed_s=elapsed)
    print("ground settle:", R["ground_settle"])
    if not settled:
        base.disarm(mav)
        return finish_fail(R, "ground_settle", mav)

    _, ok_v = base.phase2_teleport_and_verify(
        node, sub, osub, pub_oneshot, (0.0, 0.0, 90.0, 0.0, 0.0, 0.0), R)
    print("PHASE 2 (teleport+verify):", R["phase2_teleport_verify"]["ok"])
    if not ok_v:
        base.disarm(mav)
        return finish_fail(R, "phase2_teleport_verify", mav)

    base.clear_wrench(pub_clear)

    hold_ok = base.phase3_hold_to_trim(pub_oneshot, pub_clear, sub, osub, mav, R)
    print("PHASE 3 (hold-to-trim): aborted =", R["phase3_hold_to_trim"]["aborted"],
          "reason =", R["phase3_hold_to_trim"]["abort_reason"])
    if not hold_ok:
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        base.disarm(mav)
        return finish_fail(R, "phase3_hold_to_trim", mav)

    ok, segs = flight_sequence(mav, sub, osub, adiag, pdiag, aerodiag, p, R)

    mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    base.disarm(mav)

    if ok and "analysis" in R:
        vd, fails = verdict(R)
        R["verdict"] = vd
        R["failed_checks"] = fails
        R["overall_result"] = "FLIGHT_COMPLETED_NO_ABORT"
        print_summary(R)
        print(f"VERDICT: {vd}  failed_checks={fails}")
    else:
        R["overall_result"] = "FLIGHT_ABORTED"
        R["verdict"] = "TECS_PTCH_DAMP_REGRESSION_FAILED"
        print("FLIGHT ABORTED:", R.get("flight_result"))

    write_outputs(R, segs, p)
    print("RESULT:", R["overall_result"], R.get("verdict"), "->", OUT_JSON)
    print("TIMESERIES:", OUT_TS)
    print("PER-SAMPLE TRACE:", OUT_TRACE)
    mav.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

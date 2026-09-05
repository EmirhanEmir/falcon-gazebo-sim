#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_LONGITUDINAL_PHUGOID_DAMPING_VALIDATION, part 1:
BASELINE FREE-DECAY HARNESS  (controls-integration, 2026-09-04).

WHY THIS STAGE EXISTS
---------------------
ARDUPLANE_TECS_CLIMB_DESCENT_AND_ENERGY_VALIDATION (2026-09-03/04) closed
READY but carried ONE open MAJOR:

    the CLOSED-LOOP longitudinal energy-mode envelope decays SLOWER than the
    FREE AIRFRAME Lanchester phugoid reference.
        tau_free  (Lanchester, V*(L/D)/g)              = 20.895 s
        tau_closed(measured, P3_settle log-decrement)  = 25.909 s
        ratio                                          = 1.240  (1.24x SLOWER)
    (25.909 s is the exact envelope form -T/ln(r_cycle); the small-damping form
     T/(2*pi*zeta) gives 25.924 s. Both are reproduced by this harness - see
     the offline cross-check note below.)

That number was a BY-PRODUCT of a five-phase climb/descent campaign: the
"free decay" was whatever was left after a commanded +10 m step, measured
over a window that also had to serve a settling-time criterion. This stage
replaces it with a PURPOSE-BUILT measurement: ONE clean longitudinal
transient, release to a fixed neutral stick, and a ring-down long enough to
resolve the envelope.

WHAT THIS STAGE DOES **NOT** DO
-------------------------------
  * It writes NO parameter by default (see PARAMETER POLICY).
  * It does not re-audit FBWB mode selection, the RC->command mappings, the
    PTCH_TRIM_DEG telemetry convention, the atmosphere/pitot datum, the
    +/-45 deg surface scaling, the actuator/aero/propulsion models, or
    mass/CG/inertia. Those are closed in earlier stages.
  * It does not touch the INNER pitch-rate loop (PTCH_RATE_*/PTCH2SRV_*).
    That loop is explicitly OUT OF SCOPE for this stage.

SOURCE ROOT-CAUSE SUMMARY (ArduPilot 4.8.0-dev, commit 409226a637,
/home/emirhan/gazebo_sim/ardupilot). Read from source, not from memory.
Reproduced here because the diagnostics below are derived from it.

  The TECS longitudinal PITCH loop is a PD+I controller on the specific
  energy BALANCE error. With no landing/takeoff/VTOL/gliding/underspeed
  stage active and TECS_SPDWEIGHT = 1.0:

    w_SKE = w_SPE = 1                                   [AP_TECS.cpp:1003,1024,1027-1028]
    S      = SEB_error = SEB_dem - SEB_est               [AP_TECS.cpp:1031-1033]
    SEBdot_dem       = g*hgt_rate_dem*w_SPE + S/Tc       [AP_TECS.cpp:1036]
    SEBdot_est       = SPEdot*w_SPE - SKEdot*w_SKE       [AP_TECS.cpp:1050]
    SEBdot_error     = SEBdot_dem - SEBdot_est           [AP_TECS.cpp:1051]
    SEBdot_dem_total = SEBdot_dem + SEBdot_error*Kd      [AP_TECS.cpp:1062]  Kd = TECS_PTCH_DAMP
    pitch_dem        = (SEBdot_dem_total + integSEBdot + integKE)/(TAS*g)
                                                        [AP_TECS.cpp:1065,1108]

  With the demands held constant (which is exactly the ring-down condition
  this stage creates), d(SEB_dem)/dt = g*hgt_rate_dem, so

    SEBdot_error = Sdot + S/Tc                           (identity)

  and therefore

    SEBdot_dem_total = (1 + Kd)*S/Tc + Kd*Sdot           [from :1036 + :1062]
    integSEBdot      = Ki*S + (Ki/Tc)*INTEGRAL(S)        [from :1086,:1095]
                                                          Ki = TECS_INTEG_GAIN
    integKE          = (1/Tc)*INTEGRAL(SKE_est - SKE_dem)*w_SKE
                                                        [AP_TECS.cpp:1096]

  i.e. the EFFECTIVE gains the energy-balance mode actually sees are

    Kp_eff = (1 + TECS_PTCH_DAMP)/TECS_TIME_CONST + TECS_INTEG_GAIN   [1/s]
    Kd_eff = TECS_PTCH_DAMP                                           [-]

  TECS_INTEG_GAIN therefore contributes PURE PROPORTIONAL STIFFNESS with no
  phase lead at all (the Ki*S term above is not an integral - it is the
  exact integral of the S/Tc part of SEBdot_error), while TECS_PTCH_DAMP is
  the ONLY derivative term anywhere in this loop. See the stage report for
  the full parameter-by-parameter table and the ranking.

  ASSUMPTION  TECS_SEB_MANIFOLD_LINEARISATION
  ----------------------------------------------------------------
  The REPORT-ONLY diagnostic `tecs_energy_loop_gains` below additionally
  linearises the airframe onto the constant-total-energy manifold
  (SPEdot = -SKEdot), on which S = -2*g*dh and Sdot = -2*g*dhdot, giving an
  ideal height-loop gain of 2*Kp_eff [rad/s] and an ideal first-order
  height-loop time constant (1 + 2*Kd_eff)/(2*Kp_eff) [s]. That manifold
  identity is exact only for a lossless energy exchange. The numbers it
  produces are DIAGNOSTIC ONLY, are never gated, and are labelled as such
  in the result artifact.

WHAT IS MEASURED (the deliverable of part 1)
--------------------------------------------
Per run, from ONE free ring-down:
    * oscillation PERIOD                      T_meas   [s]
    * decay envelope time constant            tau_env  [s]   (two estimators)
    * decay ratio (2nd half / 1st half spread)         [-]   per channel
    * damping ratio estimate                  zeta     [-]
    * free-airframe Lanchester reference      tau_ref, T_ref [s]
    * closed-loop / free-airframe ratios      tau_env/tau_ref  <- the 1.24x
                                              T_meas /T_ref    <- NEW: tells
      whether the observed mode IS the airframe phugoid or a TECS-generated
      closed-loop mode. The prior stage measured T_meas = 5.633 s against
      T_ref = 8.128 s, i.e. the closed-loop mode is 1.44x FASTER than the
      free phugoid. That discriminator is now a first-class output.

EXCITATION - ONE PULSE, THEN RELEASE (task requirement)
-------------------------------------------------------
FBWB pitch stick, full up, for EXCITE_PULSE_S, then released to RC2_TRIM for
the whole ring-down. No repeating cadence.

  Mechanism, cited: ArduPlane/navigation.cpp:402-445 update_fbwb_speed_height()
    - runs every 100 ms (:404-405), so the release is captured within 0.1 s;
    - the target altitude ramps at FBWB_CLIMB_RATE * elevator_input (:427-429);
    - when the input passes back THROUGH ZERO, set_target_altitude_current()
      locks the target at the CURRENT altitude (:418-424).
  So at release the aircraft is left with a LOCKED, CONSTANT altitude demand
  and a CONSTANT airspeed demand, but a non-zero climb rate and a non-zero
  speed error: a clean longitudinal energy transient with fixed demands.
  This is the free-decay condition the analysis needs, and it uses the same
  command channel as the prior stage, so the two remain comparable.

AMPLITUDE / DURATION JUSTIFICATION (from the prior stage's MEASURED response,
not from a guess - PRIOR_ENERGY below, all values from
tests/gazebo/results/ardupilot_tecs_climb_descent_energy_result.json)
  * The prior stage's full-up-stick ramp reached a quasi-steady climb rate
    within a few seconds: ramp mean vz 1.301 m/s, peak 1.951 m/s. Because
    the FBWB stick commands a target-altitude RATE (not a step), the state
    at release - which is what actually excites the ring-down - saturates
    with pulse length. A LONGER pulse mostly just gains altitude; it does
    not give a bigger transient. The pulse is therefore chosen as the
    SHORTEST that still develops that quasi-steady climb rate:
        EXCITE_PULSE_S = 4.0 s
        >= TECS_HDEM_TCONST (3.0 s, AP_TECS.cpp:292) + ~1 s of pitch response.
  * Predicted excursion, scaled from the prior stage's 8.67 s ramp which
    produced A0_alt 3.354 m / A0_V 1.598 m/s:  A0_alt ~ 2-3 m, A0_V ~ 1 m/s.
    Comfortably above the 1.5 m / 0.5 m/s settle bands used by the prior
    stage (so the log-decrement has amplitude to work with) and far below
    anything non-linear.
  * Linearity / acceptance envelope, predicted from the prior stage's
    WHOLE-FLIGHT extremes (which used a 10 m step - a strictly LARGER
    excitation than this one):
        airspeed      16.780 .. 19.342 m/s   -> stays >= AIRSPEED_MIN 16
        elevator max  6.368 deg              -> stays <= 10 deg
        throttle      0.425 .. 0.540         -> no saturation (THR_MIN 0 / MAX 1)
        pitch         -1.253 .. 6.683 deg    -> inside TECS_PITCH_MAX 15 /
                                                PTCH_LIM_MIN_DEG -25
    All four are GATED here, so if the excitation is in fact too large the
    run FAILS instead of quietly reporting a non-linear decay.
  * UP rather than DOWN: an up-pulse raises throttle and lowers speed, which
    keeps the excitation away from the still-open PROPULSION_HIGH_J_WINDMILLING
    limitation (thrust floored at 0 N above the APC table's last J). The
    ring-down itself still contains descending half-cycles, so High-J is
    still COUNTED and REPORTED (non-gating), exactly as the prior stage did.

RING-DOWN WINDOW LENGTH JUSTIFICATION
  T_ph(free, Lanchester)      = pi*sqrt(2)*V/g = 8.13 s
  T_meas(prior, closed loop)  = 5.633 s
  tau_env under test          ~ 25.9 s
  The PRIMARY analysis window is t_seg >= HOLD_TRANSIENT_S (10.0 s, the
  inherited post-transient cutoff = 2 x TECS_TIME_CONST), so the ring-down
  segment must be 10 s longer than the window needed. Requiring
      (a) >= 2 envelope time constants of decay  -> >= 51.8 s of window
      (b) >= 6 half-cycles (>= 6 extrema) at EITHER candidate period
          -> >= 24.4 s (free) / >= 16.9 s (closed loop)
  (a) binds. RINGDOWN_S = 65.0 s gives a 55.0 s primary window
      = 2.12 tau (amplitude falls to 12%), = 6.8 free-phugoid periods,
      = 9.8 closed-loop-mode periods. Both bounds satisfied with margin.

TOTAL FLIGHT TIME: 30 + 4 + 65 = 99 s (the prior stage flew 165 s).

PARAMETER POLICY - READ THIS BEFORE ADDING A FLAG
--------------------------------------------------
  * DEFAULT: this test writes NO parameter of any kind. TECS runs on the
    ArduPlane compiled firmware defaults (config/ardupilot/falcon_v2_sitl.parm
    sets no TECS_* value and arduplane is launched with -w, a wiped scratch
    EEPROM). The BASELINE run for this stage MUST be run with no flags.
  * config/ardupilot/falcon_v2_sitl.parm is READ-ONLY input and is never
    edited by this file or its runner.
  * `--set-param NAME=VALUE` (repeatable) performs a RUNTIME MAVLink
    PARAM_SET, in the SITL scratch EEPROM only, and ONLY for names on
    SETTABLE_PARAMS below. Every other name is REFUSED with a non-zero exit -
    in particular no PID, no PTCH_TRIM_DEG, no SERVOn_*, no ARSPD_*, no
    SIM_*, no aero/propulsion/actuator value can be written through this
    path. Each write is read back and confirmed, and both the before and
    after values are recorded in the result artifact.
  * In THIS part of the stage (part 1, baseline) the flag must not be used.
    It exists so part 2 can run the identical harness on a candidate value
    without forking the code.

USAGE (a FRESH gz sim + gdb-wrapped arduplane pair MUST already be running -
see tests/gazebo/scripts/run_ardupilot_longitudinal_phugoid_damping.sh):
    python3 test_ardupilot_longitudinal_phugoid_damping.py
    python3 test_ardupilot_longitudinal_phugoid_damping.py --reanalyze <timeseries.json>
    python3 test_ardupilot_longitudinal_phugoid_damping.py --reanalyze <timeseries.json> \
            --dataflash <dir with the run's *.BIN>      # optional; auto-derived
    python3 test_ardupilot_longitudinal_phugoid_damping.py --set-param TECS_PTCH_DAMP=0.6
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
# The 2026-09-03 climb/descent/energy harness, IMPORTED (not copied). Every
# piece of decay/envelope/energy math used below comes from here unmodified:
#   phugoid_reference()  damping_estimate()  _local_extrema()
#   energy_block()  high_j_block()  analyze_window()  TRACE_COLUMNS
#   the s_* per-sample accessors, seb_weights(), and the settle/decay
#   thresholds. Their VALUES ARE NOT CHANGED by this stage.
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
s_ap_target_alt_rel_m = energy.s_ap_target_alt_rel_m
s_ap_alt_rel_m = energy.s_ap_alt_rel_m
s_spe, s_ske, s_ste, s_seb = energy.s_spe, energy.s_ske, energy.s_ste, energy.s_seb
seb_weights = energy.seb_weights
phugoid_reference = energy.phugoid_reference
damping_estimate = energy.damping_estimate
high_j_block = energy.high_j_block

STAGE = "ARDUPLANE_LONGITUDINAL_PHUGOID_DAMPING_VALIDATION"
PART = "part1_baseline_free_decay"
OUT_JSON = os.path.join(base.RESULTS_DIR, "ardupilot_longitudinal_phugoid_damping_result.json")
OUT_TS = os.path.join(base.RESULTS_DIR, "ardupilot_longitudinal_phugoid_damping_timeseries.json")
OUT_TRACE = os.path.join(base.RESULTS_DIR, "ardupilot_longitudinal_phugoid_damping_per_sample.json")

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
SURFACE_TRAVEL_LIMIT_DEG = energy.SURFACE_TRAVEL_LIMIT_DEG   # 45.0

# Measured reference results of the 2026-09-03/04 energy stage. REFERENCE ONLY
# (nothing here feeds a control path; no check requires reproducing them).
# Source: tests/gazebo/results/ardupilot_tecs_climb_descent_energy_result.json
# and docs/validation/2026-09-03_ardupilot_tecs_climb_descent_energy_validation.md
PRIOR_ENERGY = dict(
    P3_settle_period_s=5.632851,
    P3_settle_zeta=0.0345816,
    P3_settle_tau_env_s=25.909,          # = -T/ln(r_cycle), the OPEN MAJOR number
    P3_settle_tau_env_from_zeta_s=25.924,   # = T/(2*pi*zeta), small-damping form
    P3_settle_period_ratio=0.693,        # 5.633/8.128 - the mode is NOT the free phugoid
    P3_settle_tau_ref_s=20.894744,       # Lanchester V*(L/D)/g
    P3_settle_tau_ratio=1.240,           # 25.912 / 20.895
    P3_settle_T_ref_s=8.128419,
    P3_settle_A0_alt_m=3.354222,
    P3_settle_A0_airspeed_ms=1.598097,
    P3_settle_amplitude_ratio_per_cycle=0.804598,
    P3_settle_window_over_tau=1.156,     # < 2.0 -> why RINGDOWN_S is 65 s here
    P3_settle_decay_ratio_max=0.558756,
    P5_resettle_A0_alt_m=2.922292,
    ramp_vz_ms=1.301, ramp_peak_vz_ms=1.951,
    level_throttle=0.4911, climb_throttle=0.5398, descent_throttle=0.4251,
    level_pitch_deg=2.663, climb_pitch_deg=6.683, descent_pitch_deg=-1.253,
    whole_flight_airspeed_min_ms=16.780, whole_flight_airspeed_max_ms=19.342,
    whole_flight_elevator_max_abs_deg=6.368,
    L_over_D_aero=11.420782, V_settled_ms=17.947758,
)

# =============================================================================
# PHASE PLAN (see the module docstring for the full duration derivation)
# =============================================================================
P1_TRIM_S = 30.0        # SHORTENED from the prior stage's 45 s P1 ON PURPOSE:
                        # this window only has to establish a repeatable trim
                        # and re-prove TECS throttle authority. The cruise
                        # speed-hold ACCEPTANCE it used to serve was closed by
                        # ARDUPLANE_TECS_AND_CRUISE_SPEED_HOLD_VALIDATION
                        # (2026-09-02) and is not re-litigated here. 30 s
                        # = 10 s inherited transient cutoff + 20 s analysed
                        # (4 x TECS_TIME_CONST, 2.4 free-phugoid periods).
EXCITE_PULSE_S = 4.0    # ONE pulse. Derivation: module docstring, AMPLITUDE /
                        # DURATION JUSTIFICATION.
RINGDOWN_S = 65.0       # Derivation: module docstring, RING-DOWN WINDOW
                        # LENGTH JUSTIFICATION.
HOLD_TRANSIENT_S = energy.HOLD_TRANSIENT_S   # 10.0, INHERITED UNCHANGED
SETTLE_TAIL_S = energy.SETTLE_TAIL_S         # 10.0, INHERITED UNCHANGED

PHASES = ["P1_trim", "P2_excite", "P3_ringdown"]
TOTAL_FLIGHT_S = P1_TRIM_S + EXCITE_PULSE_S + RINGDOWN_S      # 99.0

# Release latency bound: update_fbwb_speed_height() only acts every 100000 us
# (ArduPlane/navigation.cpp:404-405), and run_seg republishes RC every
# campaign.RC_REFRESH_PERIOD = 0.1 s, so the stick release is registered
# within 0.2 s of the P2->P3 boundary. Recorded, not assumed away.
RELEASE_LATENCY_BOUND_S = 0.2

# =============================================================================
# ACCEPTANCE THRESHOLDS
# Every value is INHERITED from the 2026-09-02 cruise stage or the 2026-09-03
# energy stage (cited to the imported symbol so they cannot silently diverge),
# or DERIVED here from an inherited value / a documented physical quantity.
# NO threshold of the imported decay math is changed by this stage.
# =============================================================================
# ---- inherited verbatim -----------------------------------------------------
TH_SPEED_MIN_MS = energy.TH_SPEED_MIN_MS                  # 16.0 = AIRSPEED_MIN
TH_SPEED_HARD_FLOOR_MS = energy.TH_SPEED_HARD_FLOOR_MS    # 14.4 = 0.9*TASmin
TH_SAT_RUN_MAX_S = energy.TH_SAT_RUN_MAX_S                # 2.0
TH_SAT_MARGIN = energy.TH_SAT_MARGIN                      # 0.01
TH_TECS_AUTHORITY_MIN_DELTA = energy.TH_TECS_AUTHORITY_MIN_DELTA   # 0.10
TH_THROTTLE_MODULATION_MIN = energy.TH_THROTTLE_MODULATION_MIN     # 0.05
TH_SURF_HOLD_MAX_DEG = energy.TH_SURF_HOLD_MAX_DEG        # 10.0  (ring-down)
TH_SURF_FLIGHT_MAX_DEG = energy.TH_SURF_FLIGHT_MAX_DEG    # 15.0  (pulse)
TH_LATERAL_SURF_MAX_DEG = energy.TH_LATERAL_SURF_MAX_DEG  # 10.0
TH_SURF_MAX_ABS_DEG = energy.TH_SURF_MAX_ABS_DEG          # 40.0 = 45 - 5 margin
TH_DECAY_RATIO_MAX = energy.TH_DECAY_RATIO_MAX            # 0.90
TH_TECS_TARGET_TOL_MS = energy.TH_TECS_TARGET_TOL_MS      # 0.4
TH_SPEED_MEAN_TOL_MS = energy.TH_SPEED_MEAN_TOL_MS        # 0.5

# ---- DERIVED HERE -----------------------------------------------------------
# A ring-down can only be identified if the envelope is actually observed to
# decay over the analysed window. Two independent measurability requirements:
#   TH_MIN_EXTREMA        the log-decrement estimator needs >= 3 extrema to
#                         return anything at all (energy.damping_estimate);
#                         6 is required here so tau_env rests on >= 5
#                         half-cycle amplitude ratios rather than 2.
TH_MIN_EXTREMA = 6
#   TH_MIN_WINDOW_TAU     the analysed window must span at least 2 envelope
#                         time constants (amplitude must fall to <= e^-2 =
#                         13.5%), the standard identifiability requirement for
#                         a first-order envelope fit. 2.0 is the minimum, not
#                         a margin.
TH_MIN_WINDOW_TAU = 2.0
#   TH_ENVELOPE_FIT_R2_MIN  the ln(peak amplitude) vs peak time regression must
#                         explain most of the variance for the "exponential
#                         envelope" model to be usable. 0.5 is a weak,
#                         deliberately permissive floor: it rejects a
#                         structureless/noise-dominated envelope, and nothing
#                         more. It is NOT a quality target.
TH_ENVELOPE_FIT_R2_MIN = 0.5
#   TH_PULSE_ALT_GAIN_MIN_M / _MAX_M   the excitation must actually excite
#                         something, and must not be so large it leaves the
#                         linear range. Bounds derived from the prior stage's
#                         measured ramp rate (1.301 m/s mean, 1.951 m/s peak)
#                         over EXCITE_PULSE_S = 4.0 s: 1.301*4 = 5.2 m nominal.
#                         Floor = 25% of nominal, ceiling = 2x nominal.
TH_PULSE_ALT_GAIN_MIN_M = round(0.25 * PRIOR_ENERGY["ramp_vz_ms"] * EXCITE_PULSE_S, 3)  # 1.301
TH_PULSE_ALT_GAIN_MAX_M = round(2.00 * PRIOR_ENERGY["ramp_vz_ms"] * EXCITE_PULSE_S, 3)  # 10.408
#   TH_RINGDOWN_A0_MIN_M  the ring-down's peak altitude excursion must exceed
#                         the prior stage's settle band (1.5 m,
#                         energy.TH_SETTLE_BAND_M) so the log-decrement has
#                         amplitude to work with above the hold-noise floor
#                         (prior measured hold p2p 2.179 m -> the band is the
#                         right scale).
TH_RINGDOWN_A0_MIN_M = energy.TH_SETTLE_BAND_M            # 1.5
#   TH_PITCH_DEMAND_MARGIN_DEG  TECS pitch demand must stay clear of its own
#                         clip limits during the ring-down, otherwise the decay
#                         is a limit-cycle measurement, not a linear one.
#                         TECS_PITCH_MAX = 15 deg (AP_TECS.cpp:148),
#                         PTCH_LIM_MIN_DEG = -25 deg (ArduPlane/config.h:161,
#                         used because TECS_PITCH_MIN = 0 -> AP_TECS.cpp:1497).
#                         Both read LIVE; 1 deg of clearance is required.
TH_PITCH_DEMAND_MARGIN_DEG = 1.0

# ---- TEST-LOGIC FIX 2026-09-04: corrected log-decrement estimator -----------
# DEFECT (test logic, NOT physics): energy.damping_estimate() forms the
# per-half-cycle amplitude ratio as the ARITHMETIC mean of the successive
# extremum ratios A_{i+1}/A_i. The logarithmic decrement is BY DEFINITION
#       delta = ln(A_i / A_{i+1}),
# so the correct pooled estimator over n extrema is the MEAN OF THE LOGS
#       delta_hat = (1/(n-1)) * sum_i ln(A_i / A_{i+1}),
# i.e. the GEOMETRIC mean of the ratios. That is also the maximum-likelihood
# estimator of the decay rate for an exponential envelope observed with
# log-normal multiplicative noise. The arithmetic mean of ratios is a BIASED
# estimator of exponential decay (Jensen: E[mean r] >= geometric mean r) for
# ANY dataset; it is wrong independently of which run it is applied to. The
# bias is negligible while the ratio spread is small and unbounded once any
# ratio exceeds 1 - which is exactly what happens once an envelope reaches its
# measurement floor. Both estimators are reported side by side so the change
# is auditable.
#
#   TH_SNR_DETECTION_MULTIPLE   ASSUMPTION - see provenance_block()
#                         ASSUMPTION_EXTREMUM_SNR_DETECTION_THRESHOLD. An
#                         extremum amplitude only carries decay information if
#                         it is distinguishable from the incoherent floor of
#                         the channel. 3.0 is the conventional 3-sigma
#                         detection threshold (IUPAC/ISO 11843 limit of
#                         detection; ~0.13% one-sided Gaussian false-admission
#                         rate per extremum). It is a DETECTION-THRESHOLD
#                         choice, fixed a priori, applied identically to every
#                         channel and every run; it is NOT fitted, NOT tuned,
#                         and does NOT reference any run's outcome. The
#                         sensitivity of the result to this choice is reported
#                         (snr_sensitivity) rather than hidden.
TH_SNR_DETECTION_MULTIPLE = 3.0
#   TH_NOISE_TAIL_CYCLES  the incoherent floor is measured on the LAST
#                         TH_NOISE_TAIL_CYCLES periods of the analysed window.
#                         A single sinusoid + linear trend has 4 free
#                         parameters, so >= 2 full cycles are required for the
#                         coherent component to be identifiable at all; 3
#                         gives one cycle of margin while keeping the window
#                         short enough that the envelope does not vary wildly
#                         across it. Capped at half the analysed span so the
#                         floor is always measured on the LATE part.
TH_NOISE_TAIL_CYCLES = 3.0
#   TH_MIN_ADMITTED_EXTREMA  minimum number of extrema that must survive SNR
#                         truncation for the corrected estimate to be declared
#                         USABLE. 3 extrema = 2 half-cycles = one complete
#                         cycle, and is the same floor energy.damping_estimate()
#                         already imposes ("fewer than 3 extrema"). Below it the
#                         estimator reports an explicit UNMEASURABLE state and
#                         DATA_REQUIRED, never a silent None.
TH_MIN_ADMITTED_EXTREMA = 3

# ---- VALIDATION FOLLOW-UP 2026-09-05: NON-GATING CROSS-CHECK ESTIMATOR ------
# Search brackets for _damped_sinusoid_fit(), the ESTIMATOR-INDEPENDENT
# full-window damped-sinusoid fit added as a cross-check on the extremum-based
# estimators. These are SEARCH BRACKETS for a numerical optimum. They are NOT
# acceptance thresholds: nothing is compared against them, no check reads them,
# and they are deliberately NOT added to threshold_block() for that reason.
# Whether the located optimum is INTERIOR to both brackets is reported
# (optimum_interior_to_brackets) so a bracket that ever bound the answer would
# be visible rather than silent.
#
#   FIT_PERIOD_LO_S / FIT_PERIOD_HI_S
#       2.0 s: ~19x the ring-down sample interval (~0.0525 s, 1047 samples over
#       54.95 s), so the whole bracket is far from the Nyquist period 2*dt.
#       25.0 s: below half the 54.9 s analysed span - a period longer than half
#       the window is not identifiable from that window at all. The bracket
#       contains BOTH candidate periods with >2x margin either side: the free
#       Lanchester reference T_ref = 8.13 s and the measured closed-loop mode
#       4.76-5.65 s.
#   FIT_TAU_LO_S / FIT_TAU_HI_S
#       1.0 s: ~1/2 of the shortest bracketed period - faster decay than that is
#       not an oscillation. 300.0 s: ~5.5x the analysed span, i.e. numerically
#       indistinguishable from "no decay at all".
#   FIT_GRID_PERIODS / FIT_GRID_TAUS / FIT_MAX_REFINE_ITER
#       Numerical settings of the coarse-grid + shrinking-step refinement, not
#       physical quantities. CROSS-CHECKED OFFLINE 2026-09-05: a 121 x 81 coarse
#       grid with the same refinement reproduces the same optimum as this
#       24 x 12 grid to 9 significant figures on both captured phugoid runs
#       (baseline tau 23.6714616 vs 23.6714613 s; candidate 6.70359300 vs
#       6.70359325 s), so the grid density is not load-bearing.
FIT_PERIOD_LO_S = 2.0
FIT_PERIOD_HI_S = 25.0
FIT_TAU_LO_S = 1.0
FIT_TAU_HI_S = 300.0
FIT_GRID_PERIODS = 24
FIT_GRID_TAUS = 12
FIT_MAX_REFINE_ITER = 400

# Parameters this test is ALLOWED to write at runtime, and only behind an
# explicit --set-param flag. Deliberately restricted to the TECS energy-loop
# parameters identified by the source review. Anything not in this set is
# REFUSED. This is a hard guard, not a convention: it makes it impossible for
# this harness to touch a PID, PTCH_TRIM_DEG, a servo mapping, an ARSPD_*, a
# SIM_* or any aero/propulsion/actuator value.
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

EXTRA_PARAMS = ["TECS_PITCH_MIN", "TECS_PITCH_MAX", "PTCH_LIM_MIN_DEG",
                "PTCH_LIM_MAX_DEG", "FBWB_CLIMB_RATE", "TECS_HDEM_TCONST",
                "TECS_TIME_CONST", "TECS_PTCH_DAMP", "TECS_INTEG_GAIN",
                "TECS_THR_DAMP", "TECS_SPDWEIGHT", "TECS_HGT_OMEGA",
                "TECS_SPD_OMEGA", "TECS_VERT_ACC", "AHRS_EKF_TYPE"]
PARAMS_OF_INTEREST = list(dict.fromkeys(energy.PARAMS_OF_INTEREST + EXTRA_PARAMS))
# energy.dump_params() reads the ENERGY module's own list, so extend that list
# in place with anything this stage additionally needs. As of 2026-09-04 every
# name in EXTRA_PARAMS is already present there and this is a no-op; it exists
# so a future edit to the energy stage cannot silently leave a parameter this
# stage gates on unread (it would otherwise surface only as a None).
for _n in PARAMS_OF_INTEREST:
    if _n not in energy.PARAMS_OF_INTEREST:
        energy.PARAMS_OF_INTEREST.append(_n)

# Firmware-default TECS values this stage's BASELINE must run on. Source:
# AP_TECS.cpp AP_GROUPINFO defaults, cross-checked against the LIVE dump of the
# 2026-09-03 run (tecs_baseline_params_live in the prior result JSON).
TECS_FIRMWARE_DEFAULTS = {
    "TECS_TIME_CONST": 5.0, "TECS_THR_DAMP": 0.5, "TECS_PTCH_DAMP": 0.3,
    "TECS_INTEG_GAIN": 0.3, "TECS_SPDWEIGHT": 1.0, "TECS_HGT_OMEGA": 3.0,
    "TECS_SPD_OMEGA": 2.0, "TECS_VERT_ACC": 7.0, "TECS_HDEM_TCONST": 3.0,
    "TECS_PTCH_FF_K": 0.0, "TECS_CLMB_MAX": 5.0, "TECS_SINK_MIN": 2.0,
    "TECS_SINK_MAX": 5.0, "TECS_PITCH_MAX": 15.0, "TECS_PITCH_MIN": 0.0,
    "TECS_OPTIONS": 0.0, "TECS_SYNAIRSPEED": 0.0,
}


# =============================================================================
# runtime parameter writes (OPT-IN ONLY - see PARAMETER POLICY)
# =============================================================================
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


def param_set_confirmed(mav, name, value, timeout=6.0):
    """MAVLink PARAM_SET + read-back confirmation. Writes ONLY to the SITL
    scratch EEPROM (arduplane is launched with -w). Never touches
    config/ardupilot/falcon_v2_sitl.parm."""
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


# =============================================================================
# SOURCE-DERIVED TECS ENERGY-LOOP GAIN DIAGNOSTIC (REPORT ONLY, NEVER GATED)
# =============================================================================
def tecs_energy_loop_gains(p):
    """Effective gains the longitudinal ENERGY-BALANCE mode actually sees,
    derived algebraically from AP_TECS.cpp (see the module docstring for the
    full derivation and the line numbers). Computed from the LIVE parameter
    values so it is always consistent with what actually flew.

    REPORT ONLY. Nothing in verdict() reads this. It exists so `validation`
    can check the root-cause argument against the flown configuration and
    against the dataflash TEC2 message (EBD/EBE/EBDD/EBDE/EBDDT/I/KI) without
    re-deriving it.
    """
    Tc = p.get("TECS_TIME_CONST")
    Kd = p.get("TECS_PTCH_DAMP")
    Ki = p.get("TECS_INTEG_GAIN")
    Kt = p.get("TECS_THR_DAMP")
    w_spe, w_ske = seb_weights(p)
    out = {
        "status": "REPORT_ONLY_DIAGNOSTIC",
        "never_gated": True,
        "source": ("AP_TECS.cpp:1031-1033 (S), :1036 (SEBdot_dem), :1050-1051 "
                   "(SEBdot_error), :1062 (SEBdot_dem_total), :1065,:1108 "
                   "(pitch_dem = .../(TAS*g)), :1086,:1095 (integSEBdot), "
                   ":1096 (integKE), :738-772 (throttle loop). "
                   "ArduPilot 4.8.0-dev commit 409226a637."),
        "identity": ("with constant demands, SEBdot_error == Sdot + S/Tc, so "
                     "integSEBdot == Ki*S + (Ki/Tc)*INTEGRAL(S): TECS_INTEG_GAIN "
                     "contributes PURE PROPORTIONAL stiffness with no phase lead."),
        "assumption": "TECS_SEB_MANIFOLD_LINEARISATION",
        "assumption_detail": ("the height-loop numbers below additionally assume the "
                              "constant-total-energy manifold SPEdot = -SKEdot, on "
                              "which S = -2*g*dh. Exact only for a lossless energy "
                              "exchange - DIAGNOSTIC ONLY."),
        "TECS_TIME_CONST": Tc, "TECS_PTCH_DAMP": Kd, "TECS_INTEG_GAIN": Ki,
        "TECS_THR_DAMP": Kt, "w_SPE": w_spe, "w_SKE": w_ske,
    }
    if None in (Tc, Kd, Ki) or Tc <= 0:
        out["insufficient_params"] = True
        return out
    Kp_eff = (1.0 + Kd) / Tc + Ki
    out["Kp_eff_per_s"] = Kp_eff
    out["Kp_eff_formula"] = "(1 + TECS_PTCH_DAMP)/TECS_TIME_CONST + TECS_INTEG_GAIN"
    out["Kp_from_PTCH_DAMP_and_TIME_CONST_per_s"] = (1.0 + Kd) / Tc
    out["Kp_from_INTEG_GAIN_per_s"] = Ki
    out["Kp_fraction_from_INTEG_GAIN"] = Ki / Kp_eff if Kp_eff > 0 else None
    out["Kd_eff"] = Kd
    out["Kd_eff_formula"] = "TECS_PTCH_DAMP (the ONLY derivative term in the loop)"
    out["pd_zero_rad_s"] = (Kp_eff / Kd) if Kd > 0 else None
    out["pd_zero_note"] = ("angular frequency above which the PD pair contributes "
                           "phase lead. If this sits ABOVE the measured mode "
                           "frequency the loop looks proportional to that mode, "
                           "which stiffens it without damping it.")
    out["ideal_height_loop_gain_rad_s"] = 2.0 * Kp_eff
    out["ideal_height_loop_tau_s"] = (1.0 + 2.0 * Kd) / (2.0 * Kp_eff)
    out["true_integral_gain_from_INTEG_GAIN_per_s2"] = Ki / Tc
    out["integKE_gain_per_s"] = 1.0 / Tc
    out["integKE_note"] = ("AP_TECS.cpp:1096 - integKE integrates "
                           "(SKE_est - SKE_dem)*w_SKE with gain 1/timeConstant(), "
                           "NOT TECS_INTEG_GAIN. It is the loop's dominant TRUE "
                           "integral term and it adds phase LAG.")
    out["throttle_loop_note"] = ("TECS_THR_DAMP multiplies STEdot_error - the TOTAL "
                                 "energy rate error (AP_TECS.cpp:740,768-772), after "
                                 "a 0.5 s first-order filter (:744-746). A pure "
                                 "energy-BALANCE oscillation has SPEdot = -SKEdot, so "
                                 "STE_error and STEdot_error are ~0 through it: the "
                                 "throttle loop is nearly blind to the mode measured "
                                 "here even though throttle is unsaturated.")
    out["hgt_omega_note"] = ("TECS_HGT_OMEGA feeds ONLY the baro/inertial "
                             "complementary filter in the ELSE branch of "
                             "AP_TECS.cpp:343-374. With an EKF vertical velocity "
                             "available (:343-345 get_velocity_NED) that branch never "
                             "executes and _height comes from :330-331 "
                             "get_relative_position_D_home. AHRS_EKF_TYPE is recorded "
                             "live so this can be checked, not assumed.")
    out["ahrs_ekf_type_live"] = p.get("AHRS_EKF_TYPE")
    return out


# =============================================================================
# RING-DOWN ANALYSIS (this stage's own contribution)
# =============================================================================
def _envelope_fit(ts, ys):
    """Least-squares fit of ln(|extremum amplitude|) vs extremum time on the
    detrended series -> envelope time constant tau_env, INDEPENDENT of the
    log-decrement estimator in energy.damping_estimate(). Reported alongside
    it so a single bad extremum cannot silently set the headline number."""
    out = {"model": "ln(A_k) = ln(A_0) - t_k / tau_env, least squares over extrema"}
    if len(ys) < 12:
        out["insufficient_samples"] = True
        return out
    slope, icpt = linreg(ts, ys)
    if slope is None:
        out["degenerate"] = True
        return out
    resid = [y - (slope * t + icpt) for t, y in zip(ts, ys)]
    idx = energy._local_extrema(ts, resid)
    amps = [abs(resid[i]) for i in idx]
    times = [ts[i] for i in idx]
    pts = [(t, math.log(a)) for t, a in zip(times, amps) if a > 1e-9]
    out["n_extrema"] = len(idx)
    if len(pts) < 4:
        out["insufficient_extrema"] = True
        return out
    tt = [q[0] for q in pts]
    ll = [q[1] for q in pts]
    sl, ic = linreg(tt, ll)
    if sl is None or sl >= 0.0:
        out["tau_env_s"] = None
        out["growing_or_degenerate"] = True
        out["ln_slope_per_s"] = sl
        return out
    tau = -1.0 / sl
    lbar = mean(ll)
    ss_tot = sum((v - lbar) ** 2 for v in ll)
    ss_res = sum((v - (sl * t + ic)) ** 2 for t, v in zip(tt, ll))
    out["tau_env_s"] = tau
    out["ln_slope_per_s"] = sl
    out["ln_A0"] = ic
    out["r2"] = (1.0 - ss_res / ss_tot) if ss_tot > 1e-15 else None
    out["first_amplitude"] = amps[0]
    out["last_amplitude"] = amps[-1]
    out["first_extremum_t_s"] = times[0]
    out["last_extremum_t_s"] = times[-1]
    out["span_s"] = times[-1] - times[0]
    return out


# =============================================================================
# CORRECTED LOG-DECREMENT ESTIMATOR  (TEST-LOGIC FIX, 2026-09-04)
# =============================================================================
# Scope of this fix: it changes HOW the envelope decay rate is ESTIMATED from a
# set of extrema. It changes NO acceptance threshold, NO physics parameter, NO
# ArduPilot parameter, and it does not touch
# test_ardupilot_tecs_climb_descent_energy.py, whose recorded numbers stay
# reproducible (energy.damping_estimate is still imported UNCHANGED and its
# output is still reported, now as the explicitly-labelled LEGACY estimator).
# =============================================================================
def _lstsq_cols(cols, ys):
    """Least-squares solve for a design matrix supplied as a list of columns.
    Normal equations + partial-pivoted Gaussian elimination (no numpy
    dependency, consistent with the rest of this harness). Returns the
    coefficient list, or None if the system is singular."""
    m, n = len(cols), len(ys)
    A = [[sum(cols[a][k] * cols[b][k] for k in range(n)) for b in range(m)]
         for a in range(m)]
    rhs = [sum(cols[a][k] * ys[k] for k in range(n)) for a in range(m)]
    M = [A[i][:] + [rhs[i]] for i in range(m)]
    for c in range(m):
        piv = max(range(c, m), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-15:
            return None
        M[c], M[piv] = M[piv], M[c]
        for r in range(m):
            if r != c:
                f = M[r][c] / M[c][c]
                for k in range(c, m + 1):
                    M[r][k] -= f * M[c][k]
    return [M[i][m] / M[i][i] for i in range(m)]


def _damped_sinusoid_fit(ts, ys, label=""):
    """NON-GATING, ESTIMATOR-INDEPENDENT CROSS-CHECK (validation follow-up,
    2026-09-05, MAJOR-1 item 3).

    Fits the RAW analysed samples directly with

        y(t) = c0 + c1*u + exp(-u/tau) * (a*cos(w*u) + b*sin(w*u)),  u = t - t0

    using NO extremum detection and NO SNR truncation whatsoever. Every sample
    in the window contributes. It therefore cannot inherit any property of the
    extremum-based pipeline: not the extremum detector, not the incoherent
    floor, not the admission rule, not the mean-of-logs pooling. If the
    truncated pooled estimator and this fit tell the same story, the story does
    not depend on the truncation.

    METHOD: variable projection (separable least squares). For any FIXED
    (tau, w) the model is LINEAR in (c0, c1, a, b), so those four are solved
    exactly by _lstsq_cols() and only the two non-linear parameters are
    searched: a coarse log-spaced grid over the documented brackets, then a
    shrinking-step coordinate descent. Deterministic - no randomness, no
    starting point taken from any other estimator, no iteration limit that can
    silently truncate a converging search (convergence is reported).

    REPORT ONLY. No acceptance check reads any field of this dict.
    """
    out = {"model": ("y(t) = c0 + c1*u + exp(-u/tau)*(a*cos(w*u) + b*sin(w*u)), "
                     "u = t - t_first"),
           "method": ("variable projection: linear params solved exactly per "
                      "(tau, omega); coarse log grid then shrinking-step "
                      "coordinate descent"),
           "uses_extrema": False,
           "uses_snr_truncation": False,
           "gating": False,
           "label": label,
           "search_brackets": dict(
               period_lo_s=FIT_PERIOD_LO_S, period_hi_s=FIT_PERIOD_HI_S,
               tau_lo_s=FIT_TAU_LO_S, tau_hi_s=FIT_TAU_HI_S,
               grid_periods=FIT_GRID_PERIODS, grid_taus=FIT_GRID_TAUS,
               max_refine_iter=FIT_MAX_REFINE_ITER,
               note=("SEARCH BRACKETS, not acceptance thresholds - see the "
                     "FIT_* comment block in this module"))}
    pts = [(t, y) for t, y in zip(ts, ys)
           if t is not None and y is not None
           and math.isfinite(t) and math.isfinite(y)]
    out["n_samples"] = len(pts)
    if len(pts) < 40:
        out["unmeasurable"] = "fewer than 40 finite samples"
        return out
    t0 = pts[0][0]
    us = [q[0] - t0 for q in pts]
    zs = [q[1] for q in pts]
    n = len(us)
    ybar = sum(zs) / n
    sst = sum((v - ybar) ** 2 for v in zs)
    out["t_first_s"] = t0
    out["span_s"] = us[-1]

    def _ssr(tau, w, want=False):
        c2 = [0.0] * n
        c3 = [0.0] * n
        for k in range(n):
            u = us[k]
            e = math.exp(-u / tau)
            c2[k] = e * math.cos(w * u)
            c3[k] = e * math.sin(w * u)
        x = _lstsq_cols([[1.0] * n, us, c2, c3], zs)
        if x is None:
            return (None, None) if want else None
        s = 0.0
        for k in range(n):
            r = zs[k] - (x[0] + x[1] * us[k] + x[2] * c2[k] + x[3] * c3[k])
            s += r * r
        return (s, (x, c2, c3)) if want else s

    best = None
    evals = 0
    for i in range(FIT_GRID_PERIODS + 1):
        T = FIT_PERIOD_LO_S * (FIT_PERIOD_HI_S / FIT_PERIOD_LO_S) ** (
            i / FIT_GRID_PERIODS)
        w = 2.0 * math.pi / T
        for j in range(FIT_GRID_TAUS + 1):
            tau = FIT_TAU_LO_S * (FIT_TAU_HI_S / FIT_TAU_LO_S) ** (
                j / FIT_GRID_TAUS)
            s = _ssr(tau, w)
            evals += 1
            if s is not None and (best is None or s < best[0]):
                best = (s, tau, w)
    if best is None:
        out["unmeasurable"] = "every coarse-grid fit was singular"
        return out
    s, tau, w = best
    fT = (FIT_PERIOD_HI_S / FIT_PERIOD_LO_S) ** (1.0 / FIT_GRID_PERIODS)
    fTau = (FIT_TAU_HI_S / FIT_TAU_LO_S) ** (1.0 / FIT_GRID_TAUS)
    converged = False
    for _ in range(FIT_MAX_REFINE_ITER):
        improved = False
        for dw in (fT, 1.0, 1.0 / fT):
            for dtau in (fTau, 1.0, 1.0 / fTau):
                if dw == 1.0 and dtau == 1.0:
                    continue
                s2 = _ssr(tau * dtau, w * dw)
                evals += 1
                if s2 is not None and s2 < s - 1e-15:
                    s, w, tau = s2, w * dw, tau * dtau
                    improved = True
        if not improved:
            fT = fT ** 0.5
            fTau = fTau ** 0.5
            if (fT - 1.0) < 1e-10 and (fTau - 1.0) < 1e-10:
                converged = True
                break
    s, packed = _ssr(tau, w, want=True)
    if packed is None:
        out["unmeasurable"] = "singular fit at the located optimum"
        return out
    x, c2, c3 = packed
    T = 2.0 * math.pi / w
    amp0 = math.hypot(x[2], x[3])
    out["converged"] = converged
    out["n_objective_evaluations"] = evals
    out["tau_env_s"] = tau
    out["period_s"] = T
    out["omega_d_rad_s"] = w
    out["amplitude_at_window_start"] = amp0
    out["phase_rad"] = math.atan2(-x[3], x[2])
    out["offset_c0"] = x[0]
    out["linear_trend_c1_per_s"] = x[1]
    out["ssr"] = s
    out["rms_residual"] = math.sqrt(s / n)
    out["r2"] = (1.0 - s / sst) if sst > 1e-15 else None
    out["decay_per_cycle"] = math.exp(-T / tau)
    d_cycle = T / tau
    out["log_decrement_per_cycle"] = d_cycle
    out["damping_ratio_zeta"] = d_cycle / math.sqrt(
        4.0 * math.pi ** 2 + d_cycle * d_cycle)
    out["cycles_in_window"] = us[-1] / T
    out["window_over_tau"] = us[-1] / tau
    out["optimum_interior_to_brackets"] = bool(
        FIT_PERIOD_LO_S * 1.001 < T < FIT_PERIOD_HI_S * 0.999
        and FIT_TAU_LO_S * 1.001 < tau < FIT_TAU_HI_S * 0.999)
    out["note"] = ("independent of the extremum detector, the incoherent "
                   "floor and the SNR truncation. If tau here and the "
                   "truncated pooled tau disagree strongly, the envelope is "
                   "not a single clean exponential and BOTH numbers should be "
                   "read with that in mind.")
    return out


def _incoherent_floor(ts, resid, period_s):
    """Estimate the INCOHERENT floor of a detrended channel: the RMS of what is
    left after removing, from a late window, (a) a linear trend and (b) the
    best-fit sinusoid AT THE MODE'S OWN PERIOD.

    WHY the coherent component must be projected out first: the naive
    "detrended residual RMS over a late quiet segment" is an estimate of
    signal-PLUS-noise. It equals the noise floor only if the mode has already
    decayed away in that segment - i.e. only for a well-damped run. Using it
    directly would make the admission threshold scale with HOW MUCH SIGNAL IS
    LEFT, which is not a property of the measurement chain and would penalise a
    lightly-damped run purely for still oscillating. The component at the mode
    frequency IS the quantity being measured and must never be counted as
    noise. What remains after removing it - broadband sensor/solver noise, slow
    drift the linear detrend missed, control-quantisation motion - is the floor
    below which an extremum amplitude carries no decay information.

    The window is the LAST TH_NOISE_TAIL_CYCLES periods of the analysed span,
    capped at half that span. period_s must come from the PRE-truncation
    extrema so this estimate never depends on the truncation it feeds."""
    out = {"model": ("linear trend + sinusoid at the measured period, removed "
                     "by least squares from a late window; the floor is the RMS "
                     "of the leftover (INCOHERENT) content"),
           "tail_cycles": TH_NOISE_TAIL_CYCLES}
    if not period_s or not math.isfinite(period_s) or period_s <= 0.0:
        out["unmeasurable"] = "no period available to define the tail window"
        out["DATA_REQUIRED"] = ("a period estimate is required before the "
                                "incoherent floor can be separated from the mode")
        return out
    span = ts[-1] - ts[0]
    w = min(TH_NOISE_TAIL_CYCLES * period_s, 0.5 * span)
    out["tail_window_s"] = w
    out["analysed_span_s"] = span
    out["tail_window_capped_at_half_span"] = bool(
        TH_NOISE_TAIL_CYCLES * period_s > 0.5 * span)
    t0 = ts[-1] - w
    sel = [(t, r) for t, r in zip(ts, resid) if t >= t0]
    out["n_samples_tail"] = len(sel)
    if len(sel) < 20 or w < 2.0 * period_s:
        out["unmeasurable"] = ("tail window shorter than 2 periods or fewer "
                               "than 20 samples")
        out["DATA_REQUIRED"] = "a longer ring-down window"
        return out
    tt = [q[0] for q in sel]
    rr = [q[1] for q in sel]
    om = 2.0 * math.pi / period_s
    cols = [[1.0] * len(tt), tt,
            [math.cos(om * t) for t in tt], [math.sin(om * t) for t in tt]]
    x = _lstsq_cols(cols, rr)
    if x is None:
        out["unmeasurable"] = "singular coherent-component fit"
        return out
    fit = [x[0] + x[1] * t + x[2] * math.cos(om * t) + x[3] * math.sin(om * t)
           for t in tt]
    left = [rr[k] - fit[k] for k in range(len(tt))]
    rms_tot = math.sqrt(sum(v * v for v in rr) / len(rr))
    sigma = math.sqrt(sum(v * v for v in left) / len(left))
    out["tail_rms_total"] = rms_tot
    out["tail_coherent_amplitude_at_mode_period"] = math.hypot(x[2], x[3])
    out["sigma_incoherent"] = sigma
    out["coherent_fraction_of_tail_variance"] = (
        (1.0 - (sigma * sigma) / (rms_tot * rms_tot)) if rms_tot > 1e-15 else None)
    out["naive_tail_rms_would_have_been"] = rms_tot
    out["naive_vs_incoherent_ratio"] = (rms_tot / sigma) if sigma > 1e-15 else None
    return out


def _pooled_log_decrement(amps, times):
    """delta_hat = (1/(n-1)) * sum_i ln(A_i / A_{i+1})  -- the DEFINITION of the
    logarithmic decrement, pooled over n extrema. Equivalent to the geometric
    mean of the successive amplitude ratios. Note the estimator TELESCOPES to
    ln(A_first/A_last)/(n-1); that is a property of the definition, not an
    approximation, and it is why the per-step spread is also reported (it is
    the only place the interior extrema show up)."""
    n = len(amps)
    steps = [math.log(amps[i] / amps[i + 1]) for i in range(n - 1)]
    delta = sum(steps) / len(steps)
    if len(steps) > 1:
        mu = delta
        sd = math.sqrt(sum((v - mu) ** 2 for v in steps) / (len(steps) - 1))
    else:
        sd = None
    T = 2.0 * (times[-1] - times[0]) / (n - 1)
    return dict(
        delta_per_half_cycle=delta,
        delta_per_cycle=2.0 * delta,
        per_step_log_ratios=steps,
        per_step_sd=sd,
        standard_error_of_delta=(sd / math.sqrt(len(steps)) if sd is not None
                                 else None),
        period_s=T,
        telescoping_check=math.log(amps[0] / amps[-1]) / (n - 1),
        amplitude_ratio_per_half_cycle=math.exp(-delta),
        amplitude_ratio_per_cycle=math.exp(-2.0 * delta),
        first_amplitude=amps[0], last_amplitude=amps[-1],
        first_extremum_t_s=times[0], last_extremum_t_s=times[-1])


def _admit_extrema(amps, times, threshold):
    """SNR TRUNCATION. Keep the LEADING CONTIGUOUS run of extrema whose
    amplitude is at or above the detection threshold, and stop at the first one
    that is not. Truncation (rather than scattered rejection) is the correct
    rule for a monotonically decaying envelope: once the envelope has reached
    the floor, every later extremum is floor noise, and any of them that pokes
    back above the threshold is a noise excursion, not signal. It also keeps
    the admitted set an unbroken half-cycle sequence, which the log-decrement
    pairing requires."""
    keep = 0
    for a in amps:
        if a >= threshold:
            keep += 1
        else:
            break
    return amps[:keep], times[:keep], keep


def corrected_damping_estimate(ts, ys):
    """CORRECTED replacement for energy.damping_estimate().

      1. same linear detrend, same extremum detector (like-for-like input);
      2. SNR truncation of the extremum sequence at
         TH_SNR_DETECTION_MULTIPLE * sigma_incoherent;
      3. pooled logarithmic decrement = MEAN OF LOGS (geometric mean of
         ratios), the definition, not the arithmetic mean of ratios;
      4. an INDEPENDENT least-squares regression of ln(A_k) on t_k over the
         SAME admitted extrema, reported as a cross-check (it is the more
         efficient estimator if the noise is per-amplitude rather than
         per-ratio, and it does not telescope);
      5. the LEGACY arithmetic-mean result from energy.damping_estimate(),
         verbatim, so the two are auditable side by side;
      6. an explicit UNMEASURABLE state + DATA_REQUIRED instead of a silent
         None whenever the estimate cannot be formed.

    No acceptance threshold is read or changed here."""
    out = {"estimator": "pooled log-decrement, mean-of-logs (geometric mean of "
                        "ratios), with a-priori SNR truncation of extrema",
           "definition": "delta = (1/(n-1)) * sum_i ln(A_i / A_{i+1})",
           "snr_multiple": TH_SNR_DETECTION_MULTIPLE,
           "min_admitted_extrema": TH_MIN_ADMITTED_EXTREMA,
           "usable": False}
    out["legacy_arithmetic_mean_estimator"] = damping_estimate(ts, ys)
    if len(ys) < 12:
        out["unmeasurable"] = "fewer than 12 samples"
        out["DATA_REQUIRED"] = "a longer analysed window"
        return out
    slope, icpt = linreg(ts, ys)
    if slope is None:
        out["unmeasurable"] = "degenerate linear detrend"
        return out
    resid = [y - (slope * t + icpt) for t, y in zip(ts, ys)]
    idx = energy._local_extrema(ts, resid)
    amps_all = [abs(resid[i]) for i in idx]
    times_all = [ts[i] for i in idx]
    out["n_extrema_raw"] = len(idx)
    out["extremum_amplitudes_raw"] = amps_all
    out["extremum_times_raw"] = times_all
    if len(idx) < 3:
        out["unmeasurable"] = "fewer than 3 extrema before truncation"
        out["DATA_REQUIRED"] = "a longer or better-excited ring-down"
        return out
    period_raw = 2.0 * (times_all[-1] - times_all[0]) / (len(times_all) - 1)
    out["period_raw_all_extrema_s"] = period_raw

    # ---- NON-GATING (validation follow-up 2026-09-05, MAJOR-1 item 1) ------
    # The SAME pooled mean-of-logs estimator over ALL extrema with NO SNR
    # truncation at all. This isolates the two halves of the 2026-09-04
    # estimator change from each other: this field differs from the legacy
    # arithmetic-mean number ONLY by mean-of-logs vs mean-of-ratios, and it
    # differs from the headline number ONLY by the truncation. Nothing reads
    # it; it exists so the truncation's effect is always visible, on every run,
    # including the runs where it happens to do nothing.
    if min(amps_all) > 0.0:
        pl_nt = _pooled_log_decrement(amps_all, times_all)
        nt = {k: v for k, v in pl_nt.items()}
        nt["n_extrema_used"] = len(amps_all)
        nt["truncation_applied"] = False
        d2_nt = pl_nt["delta_per_cycle"]
        T_nt = pl_nt["period_s"]
        if d2_nt > 0.0:
            nt["tau_env_log_decrement_s"] = T_nt / d2_nt
            z_nt = d2_nt / math.sqrt(4.0 * math.pi ** 2 + d2_nt * d2_nt)
            nt["damping_ratio_zeta"] = z_nt
            nt["tau_env_from_zeta_s"] = T_nt / (2.0 * math.pi * z_nt)
        else:
            nt["tau_env_log_decrement_s"] = None
            nt["damping_ratio_zeta"] = None
            nt["tau_env_from_zeta_s"] = None
            nt["envelope_not_decaying"] = True
        ll_nt = [math.log(a) for a in amps_all]
        sl_nt, ic_nt = linreg(times_all, ll_nt)
        if sl_nt is not None and sl_nt < 0.0:
            lb = mean(ll_nt)
            sst_nt = sum((v - lb) ** 2 for v in ll_nt)
            ssr_nt = sum((v - (sl_nt * t + ic_nt)) ** 2
                         for t, v in zip(times_all, ll_nt))
            nt["regression_tau_env_s"] = -1.0 / sl_nt
            nt["regression_r2"] = (1.0 - ssr_nt / sst_nt) if sst_nt > 1e-15 else None
        else:
            nt["regression_tau_env_s"] = None
            nt["regression_growing_or_degenerate"] = True
        nt["note"] = (
            "NON-GATING. Pooled mean-of-logs log-decrement over ALL extrema, "
            "NO SNR truncation. Reported for every run so the load-bearing-ness "
            "of the truncation is explicit: on a lightly damped run nothing "
            "reaches the floor and this equals the headline number exactly; on "
            "a well damped run it does not, and the difference IS the "
            "truncation's contribution.")
        out["no_truncation_all_extrema"] = nt
    else:
        out["no_truncation_all_extrema"] = {
            "unmeasurable": "a non-positive raw extremum amplitude",
            "n_extrema_used": len(amps_all), "truncation_applied": False}

    floor = _incoherent_floor(ts, resid, period_raw)
    out["noise_floor"] = floor
    sigma = floor.get("sigma_incoherent")
    if sigma is None or not math.isfinite(sigma) or sigma <= 0.0:
        out["unmeasurable"] = "incoherent floor could not be estimated"
        out["DATA_REQUIRED"] = floor.get("DATA_REQUIRED", "a usable noise-floor window")
        return out
    thr = TH_SNR_DETECTION_MULTIPLE * sigma
    out["detection_threshold"] = thr
    amps, times, keep = _admit_extrema(amps_all, times_all, thr)
    out["n_extrema_admitted"] = keep
    out["n_extrema_rejected_below_floor"] = len(amps_all) - keep
    out["extremum_amplitudes_admitted"] = amps
    out["extremum_times_admitted"] = times
    out["truncation_rule"] = ("leading contiguous run with amplitude >= "
                              f"{TH_SNR_DETECTION_MULTIPLE} * sigma_incoherent")
    out["snr_sensitivity"] = _snr_sensitivity(ts, resid, amps_all, times_all,
                                              period_raw)
    if keep < TH_MIN_ADMITTED_EXTREMA:
        out["unmeasurable"] = (
            f"only {keep} extrema survive SNR truncation, "
            f"{TH_MIN_ADMITTED_EXTREMA} required")
        out["DATA_REQUIRED"] = (
            "a larger excitation amplitude or a lower measurement floor: the "
            "envelope reaches the incoherent floor before one full cycle of "
            "decay has been resolved, so the decay rate is NOT MEASURABLE from "
            "this run. This is an explicit unmeasurable state, not a null "
            "result.")
        return out
    if min(amps) <= 0.0:
        out["unmeasurable"] = "a non-positive admitted amplitude"
        return out

    pl = _pooled_log_decrement(amps, times)
    out.update({k: v for k, v in pl.items() if k != "per_step_log_ratios"})
    out["per_step_log_ratios"] = pl["per_step_log_ratios"]
    T = pl["period_s"]
    delta = pl["delta_per_half_cycle"]
    d_cycle = pl["delta_per_cycle"]
    if delta <= 0.0:
        out["envelope_not_decaying"] = True
        out["damping_ratio_zeta"] = None
        out["tau_env_log_decrement_s"] = None
        out["tau_env_from_zeta_s"] = None
        out["note"] = ("pooled log-decrement is <= 0 over the ADMITTED extrema: "
                       "the envelope does not decay. Reported explicitly, not "
                       "as a null.")
        out["usable"] = True
        out["usable_note"] = ("the estimate is usable; it says the mode is not "
                              "decaying, which is a measurement, not a failure "
                              "of the estimator")
        return out
    zeta = d_cycle / math.sqrt(4.0 * math.pi ** 2 + d_cycle * d_cycle)
    out["damping_ratio_zeta"] = zeta
    out["tau_env_log_decrement_s"] = T / d_cycle          # = -T/ln(r_cycle), exact
    out["tau_env_from_zeta_s"] = T / (2.0 * math.pi * zeta)
    out["omega_d_rad_s"] = 2.0 * math.pi / T
    # independent regression cross-check over the SAME admitted extrema
    ll = [math.log(a) for a in amps]
    sl, ic = linreg(times, ll)
    reg = {"model": "ln(A_k) = ln(A_0) - t_k / tau, LS over ADMITTED extrema only"}
    if sl is not None and sl < 0.0:
        tau_r = -1.0 / sl
        lbar = mean(ll)
        sst = sum((v - lbar) ** 2 for v in ll)
        ssr = sum((v - (sl * t + ic)) ** 2 for t, v in zip(times, ll))
        reg["tau_env_s"] = tau_r
        reg["r2"] = (1.0 - ssr / sst) if sst > 1e-15 else None
        reg["ln_slope_per_s"] = sl
        reg["tau_ratio_regression_over_pooled"] = tau_r / out["tau_env_log_decrement_s"]
    else:
        reg["tau_env_s"] = None
        reg["growing_or_degenerate"] = True
        reg["ln_slope_per_s"] = sl
    reg["note"] = ("does NOT telescope, so it is sensitive to the interior "
                   "extrema the pooled estimator is blind to. A large "
                   "disagreement means the envelope is not a clean exponential.")
    out["regression_cross_check_admitted"] = reg
    out["usable"] = True
    return out


def _snr_sensitivity(ts, resid, amps_all, times_all, period_raw):
    """How the corrected estimate moves with the two a-priori choices
    (TH_SNR_DETECTION_MULTIPLE, TH_NOISE_TAIL_CYCLES). REPORT ONLY - nothing
    reads it, no threshold depends on it. It exists so the sensitivity of the
    headline number to those choices is visible rather than hidden."""
    rows = []
    saved = globals()["TH_NOISE_TAIL_CYCLES"]
    try:
        for nc in (2.0, 3.0, 4.0, 5.0):
            globals()["TH_NOISE_TAIL_CYCLES"] = nc
            fl = _incoherent_floor(ts, resid, period_raw)
            sg = fl.get("sigma_incoherent")
            for k in (2.0, 3.0, 4.0, 5.0):
                row = {"tail_cycles": nc, "snr_multiple": k,
                       "sigma_incoherent": sg}
                if sg and sg > 0.0:
                    a, t, keep = _admit_extrema(amps_all, times_all, k * sg)
                    row["threshold"] = k * sg
                    row["n_admitted"] = keep
                    if keep >= TH_MIN_ADMITTED_EXTREMA and min(a) > 0.0:
                        pl = _pooled_log_decrement(a, t)
                        d2 = pl["delta_per_cycle"]
                        row["period_s"] = pl["period_s"]
                        row["tau_env_s"] = (pl["period_s"] / d2) if d2 > 0 else None
                        row["amplitude_ratio_per_cycle"] = pl["amplitude_ratio_per_cycle"]
                    else:
                        row["unmeasurable"] = True
                rows.append(row)
    finally:
        globals()["TH_NOISE_TAIL_CYCLES"] = saved
    return {"note": ("REPORT ONLY. Sweep of the two a-priori estimator choices. "
                     "The headline uses TH_SNR_DETECTION_MULTIPLE="
                     f"{TH_SNR_DETECTION_MULTIPLE} and TH_NOISE_TAIL_CYCLES="
                     f"{saved}."),
            "rows": rows}


def ringdown_analysis(samples, p, label="P3_ringdown"):
    """Free-decay characterisation of ONE ring-down segment.

    WINDOW: the PRIMARY window is t_seg >= HOLD_TRANSIENT_S (10.0 s), the
    SAME post-transient cutoff the 2026-09-03 energy stage used, so the
    numbers produced here are directly comparable with that stage's
    P3_settle result. CROSS-CHECKED OFFLINE 2026-09-04: run against that stage's
    own captured P3_settle segment this function reproduces T = 5.632851 s,
    zeta = 0.0345816, tau_env = 25.9086 s (log-decrement) / 25.9241 s (from
    zeta) / 23.4457 s (envelope fit, r2 0.985), tau_ref = 20.8947 s,
    tau_ratio = 1.23996, period_ratio = 0.69298 - i.e. it is numerically
    identical to the prior stage on the prior stage's own data. The FULL window
    (t_seg >= 0, i.e. from the stick release) is also analysed as a secondary.

    NOTE on why the ring-down had to be lengthened: applied to the prior
    stage's 30 s analysed window this function reports window_over_tau = 1.156,
    i.e. that window spanned only 1.16 envelope time constants - below the
    TH_MIN_WINDOW_TAU = 2.0 identifiability floor. That is precisely the
    weakness RINGDOWN_S = 65.0 s (55 s analysed = 2.12 tau) removes.

    ESTIMATORS (both reported; neither is tuned):
      1. log-decrement  - energy.damping_estimate(), IMPORTED UNCHANGED.
         tau_env = -T / ln(r_cycle)                     [exact for the envelope]
         tau_from_zeta = T / (2*pi*zeta)                [small-damping form;
             this is the form that produced the prior stage's 25.912 s, kept
             so that number is exactly reproducible]
      2. envelope regression - _envelope_fit(), ln(peak amplitude) vs time.
    """
    out = {"label": label,
           "primary_window": f"t_seg >= {HOLD_TRANSIENT_S} s (INHERITED cutoff)",
           "primary_window_rationale": (
               "identical to the cutoff the 2026-09-03 energy stage applied to "
               "P3_settle, so tau_env/zeta/T are directly comparable with the "
               "1.24x open MAJOR they produced."),
           "release_latency_bound_s": RELEASE_LATENCY_BOUND_S}
    if len(samples) < 40:
        out["insufficient_samples"] = True
        return out
    post = [s for s in samples if s["t_seg"] >= HOLD_TRANSIENT_S]
    out["n_samples_full"] = len(samples)
    out["n_samples_primary"] = len(post)
    if len(post) < 40:
        out["insufficient_samples"] = True
        return out
    t_span = post[-1]["t_seg"] - post[0]["t_seg"]
    out["primary_window_span_s"] = t_span

    # ---- per-channel decay ratio (IMPORTED estimator, IMPORTED threshold) ---
    channels = (("altitude", s_alt),
                ("airspeed", lambda s: s["mav"]["airspeed"]),
                ("pitch_physical", s_pitch_phys),
                ("throttle", s_throttle_actual))
    ch = {}
    for name, fn in channels:
        ts, ys = collect(post, fn)
        ch[name] = detrended_growth(ts, ys) if len(ys) >= 8 else None
    out["decay"] = ch
    ratios = {k: (v.get("ratio_second_over_first") if v else None) for k, v in ch.items()}
    out["decay_ratios"] = ratios
    usable = [v for v in ratios.values() if v is not None]
    out["decay_ratio_max"] = max(usable) if usable else None
    out["decay_ratio_threshold"] = TH_DECAY_RATIO_MAX
    out["all_channels_decaying"] = bool(usable) and all(v <= TH_DECAY_RATIO_MAX
                                                        for v in usable)

    # ---- mode identification, per channel ---------------------------------
    modes = {}
    for name, fn in channels:
        ts, ys = collect(post, fn)
        if len(ys) < 12:
            modes[name] = {"insufficient_samples": True}
            continue
        d = damping_estimate(ts, ys)               # IMPORTED UNCHANGED
        env = _envelope_fit(ts, ys)
        T = d.get("period_s")
        rc = d.get("amplitude_ratio_per_cycle")
        z = d.get("damping_ratio_zeta")
        d2 = dict(d)
        d2["envelope_fit"] = env
        # exact envelope constant from the per-cycle amplitude ratio
        if T and rc and 0.0 < rc < 1.0:
            d2["tau_env_log_decrement_s"] = -T / math.log(rc)
        else:
            d2["tau_env_log_decrement_s"] = None
        # small-damping form - reproduces the prior stage's headline number
        if T and z and z > 0.0:
            d2["tau_env_from_zeta_s"] = T / (2.0 * math.pi * z)
        else:
            d2["tau_env_from_zeta_s"] = None
        d2["tau_env_envelope_fit_s"] = env.get("tau_env_s")
        d2["omega_d_rad_s"] = (2.0 * math.pi / T) if T else None
        # ---- CORRECTED ESTIMATOR (test-logic fix 2026-09-04) --------------
        # The LEGACY arithmetic-mean-of-ratios numbers above are retained
        # verbatim; the corrected mean-of-logs estimator with a-priori SNR
        # truncation is computed alongside them and is what the headline
        # fields and the acceptance checks read.
        corr = corrected_damping_estimate(ts, ys)
        d2["corrected_log_decrement"] = corr
        d2["corrected_period_s"] = corr.get("period_s")
        d2["corrected_amplitude_ratio_per_cycle"] = corr.get(
            "amplitude_ratio_per_cycle")
        d2["corrected_damping_ratio_zeta"] = corr.get("damping_ratio_zeta")
        d2["corrected_tau_env_log_decrement_s"] = corr.get(
            "tau_env_log_decrement_s")
        d2["corrected_tau_env_from_zeta_s"] = corr.get("tau_env_from_zeta_s")
        d2["corrected_tau_env_regression_admitted_s"] = (
            (corr.get("regression_cross_check_admitted") or {}).get("tau_env_s"))
        d2["corrected_usable"] = bool(corr.get("usable"))
        d2["corrected_unmeasurable"] = corr.get("unmeasurable")
        d2["estimator_note"] = (
            "tau_env_log_decrement_s is EXACT for an exponential envelope "
            "(r_cycle = exp(-T/tau)); tau_env_from_zeta_s = T/(2*pi*zeta) is the "
            "small-damping form and is the one that produced the prior stage's "
            "25.912 s. They agree to <0.1% at zeta ~ 0.035. "
            "tau_env_envelope_fit_s is an INDEPENDENT least-squares estimate.")
        modes[name] = d2
    out["modes"] = modes

    # ---- headline channel: ALTITUDE (same channel the prior stage used) ----
    hm = modes.get("altitude") or {}
    out["headline_channel"] = "altitude"
    out["headline_channel_rationale"] = (
        "the 2026-09-03 energy stage characterised the mode on the ALTITUDE "
        "channel (phugoid_estimate_altitude); the same channel is used here so "
        "the tau ratio is a like-for-like comparison.")
    hc = hm.get("corrected_log_decrement") or {}
    # ---- HEADLINE = CORRECTED ESTIMATOR (test-logic fix 2026-09-04) --------
    # The headline fields, and therefore the acceptance checks that read them,
    # now carry the CORRECTED pooled log-decrement (mean of logs) computed over
    # SNR-admitted extrema. The LEGACY arithmetic-mean values are preserved
    # verbatim beside them under legacy_* so the change is fully auditable.
    T_meas = hm.get("corrected_period_s")
    tau_meas = hm.get("corrected_tau_env_log_decrement_s")
    out["estimator_used_for_headline"] = (
        "CORRECTED pooled log-decrement, delta = (1/(n-1)) sum ln(A_i/A_i+1), "
        "over extrema admitted at "
        f"{TH_SNR_DETECTION_MULTIPLE} x the incoherent floor")
    out["period_measured_s"] = T_meas
    out["tau_env_measured_s"] = tau_meas
    out["tau_env_measured_from_zeta_s"] = hm.get("corrected_tau_env_from_zeta_s")
    out["tau_env_measured_envelope_fit_s"] = hm.get("tau_env_envelope_fit_s")
    out["tau_env_measured_regression_admitted_s"] = hm.get(
        "corrected_tau_env_regression_admitted_s")
    out["zeta_measured"] = hm.get("corrected_damping_ratio_zeta")
    out["amplitude_ratio_per_cycle"] = hm.get(
        "corrected_amplitude_ratio_per_cycle")
    out["n_extrema"] = hm.get("n_extrema")          # RAW count - UNCHANGED, the
    # pre-existing TH_MIN_EXTREMA gate keeps reading the raw extremum count.
    out["n_extrema_admitted"] = hc.get("n_extrema_admitted")
    out["n_extrema_rejected_below_floor"] = hc.get("n_extrema_rejected_below_floor")
    out["incoherent_floor"] = (hc.get("noise_floor") or {}).get("sigma_incoherent")
    out["snr_detection_threshold"] = hc.get("detection_threshold")
    out["corrected_estimate_usable"] = bool(hc.get("usable"))
    out["corrected_estimate_unmeasurable"] = hc.get("unmeasurable")
    if hc.get("DATA_REQUIRED"):
        out["DATA_REQUIRED"] = hc["DATA_REQUIRED"]
    ef = hm.get("envelope_fit") or {}
    out["envelope_fit_r2"] = ef.get("r2")   # UNCHANGED: full-extrema fit, the
    # pre-existing TH_ENVELOPE_FIT_R2_MIN gate keeps reading this same quantity.
    out["envelope_fit_admitted_r2"] = (
        (hc.get("regression_cross_check_admitted") or {}).get("r2"))
    # ---- NON-GATING (validation follow-up 2026-09-05, MAJOR-1) -------------
    # 1. the pooled mean-of-logs decrement over ALL extrema, NO SNR truncation.
    nt = hc.get("no_truncation_all_extrema") or {}
    out["no_truncation_all_extrema"] = nt
    out["tau_env_no_truncation_all_extrema_s"] = nt.get("tau_env_log_decrement_s")
    out["period_no_truncation_all_extrema_s"] = nt.get("period_s")
    out["zeta_no_truncation_all_extrema"] = nt.get("damping_ratio_zeta")
    out["n_extrema_no_truncation_all_extrema"] = nt.get("n_extrema_used")
    # 2. the estimator-independent full-window fit: no extrema, no truncation.
    ts_h, ys_h = collect(post, s_alt)
    out["full_window_damped_sinusoid_fit"] = _damped_sinusoid_fit(
        ts_h, ys_h, label="altitude_primary_window")
    out["tau_env_damped_sinusoid_fit_s"] = (
        out["full_window_damped_sinusoid_fit"].get("tau_env_s"))
    # 3. THE ASYMMETRY, STATED. See provenance_block() ->
    #    ASSUMPTION_EXTREMUM_SNR_DETECTION_THRESHOLD / truncation_asymmetry.
    out["truncation_asymmetry_note"] = (
        "THE SNR TRUNCATION IS NOT SYMMETRIC BETWEEN RUNS. On a LIGHTLY DAMPED "
        "run the envelope never reaches the incoherent floor inside the window, "
        "so 0 extrema are rejected and the truncation is INERT - the headline "
        "number is then identical to no_truncation_all_extrema by construction. "
        "On a WELL DAMPED run the envelope does reach the floor and the "
        "truncation is LOAD-BEARING. It follows that 'the baseline is unchanged' "
        "validates the MEAN-OF-LOGS half of the 2026-09-04 estimator change and "
        "NOT the truncation half; the truncation has to be justified on its own "
        "evidence, which is (a) that the rejected tail is demonstrably "
        "non-monotone floor noise rather than a decaying envelope - see "
        "rejected_tail_evidence below - and (b) that the qualitative conclusion "
        "survives with no truncation at all, which is why "
        "tau_env_no_truncation_all_extrema_s and "
        "tau_env_damped_sinusoid_fit_s are reported here on every run.")
    # 4. the direct evidence for (a): what the REJECTED extrema actually do.
    amps_raw = hc.get("extremum_amplitudes_raw") or []
    times_raw = hc.get("extremum_times_raw") or []
    keep_n = hc.get("n_extrema_admitted")
    rej = {"n_rejected": hc.get("n_extrema_rejected_below_floor")}
    if isinstance(keep_n, int) and len(amps_raw) > keep_n:
        r_amps = amps_raw[keep_n:]
        rej["rejected_amplitudes"] = r_amps
        rej["rejected_times_s"] = times_raw[keep_n:]
        ups = sum(1 for i in range(len(r_amps) - 1) if r_amps[i + 1] > r_amps[i])
        rej["n_increasing_steps"] = ups
        rej["n_steps"] = max(len(r_amps) - 1, 0)
        rej["monotonically_decreasing"] = bool(len(r_amps) > 1 and ups == 0)
        rej["max_over_first"] = (max(r_amps) / r_amps[0]) if r_amps[0] > 0 else None
        rej["note"] = ("A DECAYING ENVELOPE cannot grow. If the rejected tail "
                       "contains increasing steps and its maximum exceeds its "
                       "first value, that tail is floor noise, not envelope, "
                       "and taking logarithms of it measures the noise floor "
                       "rather than the decay. REPORT ONLY - no check reads "
                       "this.")
    out["rejected_tail_evidence"] = rej

    # ---- SIDE-BY-SIDE AUDIT: legacy (arithmetic mean of ratios) ------------
    out["estimator_comparison"] = dict(
        note=("LEGACY = energy.damping_estimate(), arithmetic mean of the "
              "successive extremum-amplitude ratios over ALL extrema. "
              "CORRECTED = mean of logs (geometric mean of ratios) over "
              "SNR-admitted extrema. Only the ESTIMATOR changed; no threshold, "
              "no physics parameter and no ArduPilot parameter changed."),
        defect=("the arithmetic mean of ratios is a biased estimator of "
                "exponential decay for ANY dataset (Jensen), and becomes "
                "unbounded once an amplitude ratio exceeds 1, which is what "
                "happens as soon as an envelope reaches its measurement floor. "
                "The logarithmic decrement is BY DEFINITION ln(A_i/A_i+1), so "
                "the pooled estimator is the mean of the logs."),
        legacy=dict(
            period_s=hm.get("period_s"),
            amplitude_ratio_per_half_cycle=hm.get("amplitude_ratio_per_half_cycle"),
            amplitude_ratio_per_cycle=hm.get("amplitude_ratio_per_cycle"),
            damping_ratio_zeta=hm.get("damping_ratio_zeta"),
            tau_env_log_decrement_s=hm.get("tau_env_log_decrement_s"),
            tau_env_from_zeta_s=hm.get("tau_env_from_zeta_s"),
            n_extrema_used=hm.get("n_extrema")),
        corrected=dict(
            period_s=T_meas,
            amplitude_ratio_per_half_cycle=hc.get("amplitude_ratio_per_half_cycle"),
            amplitude_ratio_per_cycle=hc.get("amplitude_ratio_per_cycle"),
            damping_ratio_zeta=hm.get("corrected_damping_ratio_zeta"),
            tau_env_log_decrement_s=tau_meas,
            tau_env_from_zeta_s=hm.get("corrected_tau_env_from_zeta_s"),
            tau_env_regression_admitted_s=hm.get(
                "corrected_tau_env_regression_admitted_s"),
            delta_per_half_cycle=hc.get("delta_per_half_cycle"),
            standard_error_of_delta=hc.get("standard_error_of_delta"),
            n_extrema_used=hc.get("n_extrema_admitted"),
            n_extrema_rejected_below_floor=hc.get("n_extrema_rejected_below_floor"),
            usable=bool(hc.get("usable")),
            unmeasurable=hc.get("unmeasurable")),
        shift=dict(
            tau_pct=((100.0 * (tau_meas - hm["tau_env_log_decrement_s"])
                      / hm["tau_env_log_decrement_s"])
                     if (tau_meas and hm.get("tau_env_log_decrement_s")) else None),
            period_pct=((100.0 * (T_meas - hm["period_s"]) / hm["period_s"])
                        if (T_meas and hm.get("period_s")) else None),
            zeta_pct=((100.0 * (hm["corrected_damping_ratio_zeta"]
                                - hm["damping_ratio_zeta"]) / hm["damping_ratio_zeta"])
                      if (hm.get("corrected_damping_ratio_zeta")
                          and hm.get("damping_ratio_zeta")) else None),
            r_cycle_pct=((100.0 * (hc["amplitude_ratio_per_cycle"]
                                   - hm["amplitude_ratio_per_cycle"])
                          / hm["amplitude_ratio_per_cycle"])
                         if (hc.get("amplitude_ratio_per_cycle")
                             and hm.get("amplitude_ratio_per_cycle")) else None)),
        all_channels={k: dict(
            legacy_tau_env_log_decrement_s=v.get("tau_env_log_decrement_s"),
            legacy_zeta=v.get("damping_ratio_zeta"),
            legacy_r_cycle=v.get("amplitude_ratio_per_cycle"),
            corrected_tau_env_log_decrement_s=v.get("corrected_tau_env_log_decrement_s"),
            corrected_zeta=v.get("corrected_damping_ratio_zeta"),
            corrected_r_cycle=v.get("corrected_amplitude_ratio_per_cycle"),
            corrected_n_admitted=(v.get("corrected_log_decrement") or {}).get(
                "n_extrema_admitted"),
            corrected_unmeasurable=v.get("corrected_unmeasurable"))
            for k, v in modes.items() if isinstance(v, dict)})

    # ---- NON-GATING: the two truncation-free views, side by side with the
    # headline (validation follow-up 2026-09-05, MAJOR-1 items 1 and 3) -------
    dsf = out.get("full_window_damped_sinusoid_fit") or {}
    out["estimator_comparison"]["no_truncation_all_extrema"] = dict(
        period_s=nt.get("period_s"),
        tau_env_log_decrement_s=nt.get("tau_env_log_decrement_s"),
        damping_ratio_zeta=nt.get("damping_ratio_zeta"),
        amplitude_ratio_per_cycle=nt.get("amplitude_ratio_per_cycle"),
        delta_per_half_cycle=nt.get("delta_per_half_cycle"),
        n_extrema_used=nt.get("n_extrema_used"),
        regression_tau_env_s=nt.get("regression_tau_env_s"),
        differs_from_headline_only_by_the_truncation=True)
    out["estimator_comparison"]["damped_sinusoid_fit_no_extrema_no_truncation"] = dict(
        period_s=dsf.get("period_s"), tau_env_s=dsf.get("tau_env_s"),
        damping_ratio_zeta=dsf.get("damping_ratio_zeta"), r2=dsf.get("r2"),
        amplitude_at_window_start=dsf.get("amplitude_at_window_start"),
        n_samples=dsf.get("n_samples"), converged=dsf.get("converged"),
        optimum_interior_to_brackets=dsf.get("optimum_interior_to_brackets"),
        shares_nothing_with_the_extremum_pipeline=True)
    out["estimator_comparison"]["asymmetry_note"] = out["truncation_asymmetry_note"]
    _t_hl = tau_meas
    _t_nt = nt.get("tau_env_log_decrement_s")
    _t_fit = dsf.get("tau_env_s")
    out["estimator_comparison"]["tau_by_estimator_s"] = dict(
        legacy_arithmetic_mean_all_extrema=hm.get("tau_env_log_decrement_s"),
        mean_of_logs_all_extrema_no_truncation=_t_nt,
        mean_of_logs_snr_truncated_HEADLINE=_t_hl,
        envelope_fit_all_extrema=hm.get("tau_env_envelope_fit_s"),
        damped_sinusoid_fit_no_extrema=_t_fit,
        note=("five estimates of the SAME envelope. The first four are all "
              "extremum-based; the last uses no extrema at all. Only the third "
              "is the headline / the one the acceptance checks read."))

    # ---- free-airframe reference (IMPORTED UNCHANGED) ---------------------
    ref = phugoid_reference(samples, label)
    out["phugoid_reference_free_airframe"] = ref
    tau_ref = ref.get("tau_ref_s")
    T_ref = ref.get("T_ref_s")
    out["tau_ref_free_airframe_s"] = tau_ref
    out["T_ref_free_airframe_s"] = T_ref
    out["zeta_ref_free_airframe"] = ref.get("zeta_free_airframe")

    # ---- THE OPEN-MAJOR NUMBERS -------------------------------------------
    out["tau_ratio_closed_over_free"] = (
        (tau_meas / tau_ref) if (tau_meas and tau_ref) else None)
    out["tau_ratio_note"] = (
        "> 1 means the CLOSED-LOOP envelope decays SLOWER than the free "
        "airframe. The 2026-09-03 energy stage measured 1.240. This value is "
        "REPORTED, NOT GATED, in part 1: quantifying it honestly is the entire "
        "purpose of this baseline, so gating on it here would be circular.")
    out["period_ratio_closed_over_free"] = (
        (T_meas / T_ref) if (T_meas and T_ref) else None)
    out["period_ratio_note"] = (
        "DISCRIMINATOR. ~1.0 => the observed mode IS the free-airframe phugoid. "
        "< 1.0 => the mode has been STIFFENED by the closed loop and is a "
        "TECS-generated longitudinal energy mode, not the bare phugoid. The "
        "2026-09-03 stage measured 5.633/8.128 = 0.693, i.e. 1.44x FASTER than "
        "the free phugoid. Whether the period MOVES when a TECS gain is changed "
        "is the decisive test, and it is what part 2 will look at.")
    out["zeta_ratio_closed_over_free"] = (
        (out["zeta_measured"] / ref["zeta_free_airframe"])
        if (out.get("zeta_measured") and ref.get("zeta_free_airframe")) else None)

    # ---- NON-GATING: the same closed/free ratio under the two truncation-free
    # estimators (validation follow-up 2026-09-05, MAJOR-1) ------------------
    _tau_nt = out.get("tau_env_no_truncation_all_extrema_s")
    _tau_fit = out.get("tau_env_damped_sinusoid_fit_s")
    out["tau_ratio_closed_over_free_no_truncation"] = (
        (_tau_nt / tau_ref) if (_tau_nt and tau_ref) else None)
    out["tau_ratio_closed_over_free_damped_sinusoid_fit"] = (
        (_tau_fit / tau_ref) if (_tau_fit and tau_ref) else None)
    out["tau_ratio_estimator_sensitivity_note"] = (
        "REPORT ONLY, NON-GATING. The tau ratio that the open MAJOR is stated "
        "in, recomputed with (a) no SNR truncation and (b) no extrema at all. "
        "The gated/headline ratio remains tau_ratio_closed_over_free. These "
        "exist so the conclusion can be checked without the truncation step.")

    # source-derived predicted loop gain, for comparison with T_meas
    gains = tecs_energy_loop_gains(p)
    out["tecs_energy_loop_gains"] = gains
    if gains.get("ideal_height_loop_gain_rad_s") and out.get("period_measured_s"):
        out["measured_omega_d_over_ideal_loop_gain"] = (
            (2.0 * math.pi / out["period_measured_s"])
            / gains["ideal_height_loop_gain_rad_s"])
        out["measured_omega_d_over_ideal_loop_gain_note"] = (
            "REPORT ONLY, ASSUMPTION TECS_SEB_MANIFOLD_LINEARISATION. A value "
            "near 1.0 supports the reading that the observed mode's frequency "
            "is set by the TECS energy-loop gain 2*Kp_eff rather than by the "
            "airframe phugoid. It is a diagnostic, not evidence on its own.")

    # ---- measurability of the measurement ---------------------------------
    out["measurability"] = dict(
        n_extrema=out.get("n_extrema"), min_extrema=TH_MIN_EXTREMA,
        extrema_ok=bool(out.get("n_extrema") and out["n_extrema"] >= TH_MIN_EXTREMA),
        window_span_s=t_span,
        window_over_tau=((t_span / tau_meas) if tau_meas else None),
        min_window_over_tau=TH_MIN_WINDOW_TAU,
        window_ok=bool(tau_meas and (t_span / tau_meas) >= TH_MIN_WINDOW_TAU),
        envelope_fit_r2=out.get("envelope_fit_r2"),
        min_envelope_fit_r2=TH_ENVELOPE_FIT_R2_MIN,
        envelope_fit_ok=bool(out.get("envelope_fit_r2") is not None
                             and out["envelope_fit_r2"] >= TH_ENVELOPE_FIT_R2_MIN))

    # ---- DECLARED, NON-GATING KNOWN LIMITATION (validation follow-up
    # 2026-09-05, MINOR-1). RECORDED, DELIBERATELY NOT FIXED. --------------
    # window_over_tau above is computed on the FULL analysed span, while the
    # headline tau is estimated from the SNR-ADMITTED span only. On a run where
    # the truncation is load-bearing those two spans are not the same, so the
    # identifiability gate is evaluated over more data than the estimate
    # actually used. The GATE IS LEFT EXACTLY AS IT WAS: changing how an
    # acceptance criterion is computed AFTER its result is known is an
    # outcome-driven test edit, which CLAUDE.md's simulation tuning policy
    # forbids regardless of which way it would move the answer. It is recorded
    # here, with the numbers, as work for a future stage.
    m = out["measurability"]
    adm_t = hc.get("extremum_times_admitted") or []
    adm_span = (adm_t[-1] - adm_t[0]) if len(adm_t) >= 2 else None
    m["KNOWN_LIMITATION_window_over_tau_uses_full_span"] = dict(
        id="WINDOW_OVER_TAU_COMPUTED_ON_FULL_SPAN_NOT_SNR_ADMITTED_SPAN",
        status="DECLARED_KNOWN_LIMITATION",
        gating=False,
        gate_unchanged=True,
        gated_quantity="window_over_tau (full analysed span / headline tau)",
        gated_value=m.get("window_over_tau"),
        snr_admitted_span_s=adm_span,
        window_over_tau_on_snr_admitted_span=(
            (adm_span / tau_meas) if (adm_span and tau_meas) else None),
        min_window_over_tau=TH_MIN_WINDOW_TAU,
        would_pass_on_snr_admitted_span=bool(
            adm_span and tau_meas and (adm_span / tau_meas) >= TH_MIN_WINDOW_TAU),
        validation_reported_2026_09_05=dict(
            candidate_full_span_s=54.906, candidate_tau_s=8.652,
            candidate_window_over_tau_full_span=6.35,
            candidate_snr_admitted_span_s=16.646,
            candidate_window_over_tau_admitted_span=1.92,
            source="validation review of ARDUPLANE_LONGITUDINAL_PHUGOID_"
                   "DAMPING_VALIDATION, non-blocking MINOR-1"),
        note=("On the SNR-admitted span the candidate run sits at 1.92 tau, "
              "BELOW the 2.0 identifiability intent, while on the full span it "
              "reads 6.35 tau and passes. Both numbers are reported. The "
              "acceptance check ringdown_window_spans_two_tau still reads the "
              "FULL-span value and its state is unchanged by this record. "
              "Resolving which span the criterion should use - and re-deriving "
              "the ring-down length from it - is FUTURE-STAGE WORK, to be "
              "decided before the next measurement is taken, not after."),
        owner="gazebo-testing")

    # ---- peak excursion of the ring-down (excitation adequacy) ------------
    tail = [s for s in samples if s["t_seg"] >= samples[-1]["t_seg"] - SETTLE_TAIL_S]
    _, z_tail = collect(tail, s_alt)
    _, v_tail = collect(tail, lambda s: s["mav"]["airspeed"])
    z_set = mean(z_tail) if z_tail else None
    v_set = mean(v_tail) if v_tail else None
    out["altitude_settled_m"] = z_set
    out["airspeed_settled_ms"] = v_set
    out["settled_from_tail_s"] = SETTLE_TAIL_S
    a0_z = a0_v = None
    tp_z = tp_v = None
    for s in samples:
        z, v = s_alt(s), s["mav"]["airspeed"]
        if z is not None and z_set is not None:
            e = abs(z - z_set)
            if a0_z is None or e > a0_z:
                a0_z, tp_z = e, s["t_seg"]
        if v is not None and v_set is not None:
            e = abs(v - v_set)
            if a0_v is None or e > a0_v:
                a0_v, tp_v = e, s["t_seg"]
    out["A0_altitude_m"] = a0_z
    out["A0_altitude_t_s"] = tp_z
    out["A0_airspeed_ms"] = a0_v
    out["A0_airspeed_t_s"] = tp_v
    out["A0_altitude_min_required_m"] = TH_RINGDOWN_A0_MIN_M
    out["excitation_amplitude_ok"] = bool(a0_z is not None
                                          and a0_z >= TH_RINGDOWN_A0_MIN_M)

    # ---- secondary: the same estimators over the FULL window --------------
    ts_f, zs_f = collect(samples, s_alt)
    out["full_window_altitude_mode"] = damping_estimate(ts_f, zs_f)
    out["full_window_altitude_mode_corrected"] = corrected_damping_estimate(ts_f, zs_f)
    out["full_window_note"] = (
        "SECONDARY. Includes the initial capture transient, which a linear "
        "detrend does not remove, so its first extremum is biased high. Not "
        "used for any headline number or any check.")
    out["energy"] = energy.energy_block(post, p)
    out["high_j"] = high_j_block(samples)
    return out


# =============================================================================
# linearity / envelope gating over a window
# =============================================================================
def envelope_block(samples, p, label, surf_limit_deg):
    """Did the run stay inside the LINEAR, unsaturated acceptance envelope?
    A ring-down measured outside it is not a linear-mode measurement."""
    out = {"label": label, "surface_limit_used_deg": surf_limit_deg}
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

    out["airspeed_min_ms"] = min(asp) if asp else None
    out["airspeed_max_ms"] = max(asp) if asp else None
    out["airspeed_min_required_ms"] = TH_SPEED_MIN_MS
    out["airspeed_ok"] = bool(asp and min(asp) >= TH_SPEED_MIN_MS)
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
    # PTCH_LIM_MIN_DEG (AP_TECS.cpp:1488-1500). nav_pitch_cd is then clipped
    # AGAIN to [PTCH_LIM_MIN_DEG, PTCH_LIM_MAX_DEG] at Attitude.cpp:638.
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
        # TECS's own clip acts on nav_pitch_cd (the RAW demand), NOT on the
        # PTCH_TRIM_DEG-corrected value - Attitude.cpp:244 adds PTCH_TRIM_DEG
        # AFTER the clip. The margin is therefore checked against RAW nav_pitch.
        out["nav_pitch_margin_to_max_deg"] = eff_max - max(navp)
        out["nav_pitch_margin_to_min_deg"] = min(navp) - eff_min
        out["pitch_demand_not_clipped"] = bool(
            out["nav_pitch_margin_to_max_deg"] >= TH_PITCH_DEMAND_MARGIN_DEG
            and out["nav_pitch_margin_to_min_deg"] >= TH_PITCH_DEMAND_MARGIN_DEG)
    else:
        out["pitch_demand_not_clipped"] = False
        out["pitch_demand_clip_check_unavailable"] = True

    # actuator clamps
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

    # NaN / Inf gating over every quantity the analysis consumes
    bad = 0
    for s in samples:
        for v in (s_alt(s), s_tas(s), s["mav"]["airspeed"], s["mav"]["climb"],
                  s_pitch_phys(s), s_throttle_actual(s), s_elev_deg(s)):
            if v is not None and not math.isfinite(v):
                bad += 1
    out["nonfinite_values"] = bad
    out["all_values_finite"] = (bad == 0)
    return out


# =============================================================================
# analysis over the whole run
# =============================================================================
def analyze(R, segs, p, ptch_trim_deg):
    an = {"phase_plan": dict(
        P1_trim_s=P1_TRIM_S, excite_pulse_s=EXCITE_PULSE_S,
        ringdown_s=RINGDOWN_S, total_flight_s=TOTAL_FLIGHT_S,
        hold_transient_s=HOLD_TRANSIENT_S, settle_tail_s=SETTLE_TAIL_S,
        excitation=("ONE full-up FBWB pitch-stick pulse, then release to "
                    "RC2_TRIM for the whole ring-down. No repeating cadence."),
        release_mechanism=("ArduPlane/navigation.cpp:418-424 - the input passing "
                           "back through zero calls set_target_altitude_current(), "
                           "locking the height demand at the CURRENT altitude, so "
                           "the ring-down runs with CONSTANT height and airspeed "
                           "demands."))}
    R["analysis"] = an
    ptd = ptch_trim_deg

    for ph in PHASES:
        seg = segs.get(ph)
        if not seg:
            continue
        an[ph + "_full"] = energy.analyze_window(seg["samples"], ph + "_full", p, ptd)

    # P1 hold window (post-transient) = the trim the mode oscillates about
    p1 = segs.get("P1_trim")
    if p1:
        hold = [s for s in p1["samples"] if s["t_seg"] >= HOLD_TRANSIENT_S]
        an["P1_trim_hold"] = energy.analyze_window(hold, "P1_trim_hold", p, ptd)

    # ---- TECS authority, RE-PROVED LIVE (never inherited) ------------------
    if p1:
        rc3 = p1["rc3"]
        manual_equiv = cruise.control_in_range_no_dz(
            rc3, p["RC3_MIN"], p["RC3_MAX"], bool(p["RC3_REVERSED"])) / 100.0
        a_thr = (an["P1_trim_hold"].get("throttle_actual")
                 if an.get("P1_trim_hold") else None)
        an["tecs_authority"] = dict(
            rc3_pwm_us=rc3,
            manual_passthrough_equivalent_throttle=manual_equiv,
            measured_throttle_mean_P1_hold=(a_thr["mean"] if a_thr else None),
            abs_delta=(abs(a_thr["mean"] - manual_equiv) if a_thr else None),
            note="In FBWB the throttle stick sets target AIRSPEED "
                 "(ArduPlane/navigation.cpp:187-189); throttle itself is TECS "
                 "output (Attitude.cpp:510). A large delta proves TECS - not "
                 "the stick - is the throttle authority. RE-PROVED LIVE here.")

    # ---- excitation accounting --------------------------------------------
    exc = {"pulse_rc2_us": segs["P2_excite"]["rc2"] if segs.get("P2_excite") else None,
           "neutral_rc2_us": segs["P1_trim"]["rc2"] if segs.get("P1_trim") else None,
           "ringdown_rc2_us": segs["P3_ringdown"]["rc2"] if segs.get("P3_ringdown") else None,
           "pulse_duration_s": (segs["P2_excite"]["actual_duration_s"]
                                if segs.get("P2_excite") else None)}
    exc["single_pulse"] = bool(
        exc["neutral_rc2_us"] is not None
        and exc["ringdown_rc2_us"] == exc["neutral_rc2_us"]
        and exc["pulse_rc2_us"] != exc["neutral_rc2_us"])
    exc["stick_neutral_throughout_ringdown"] = bool(
        exc["ringdown_rc2_us"] is not None
        and p.get("RC2_TRIM") is not None
        and abs(exc["ringdown_rc2_us"] - p["RC2_TRIM"]) < 1e-6)
    z1 = (an["P1_trim_hold"]["altitude_gz_m"]["mean"]
          if an.get("P1_trim_hold") and an["P1_trim_hold"].get("altitude_gz_m") else None)
    z_rel = None
    if segs.get("P3_ringdown") and segs["P3_ringdown"]["samples"]:
        z_rel = s_alt(segs["P3_ringdown"]["samples"][0])
    exc["altitude_at_P1_hold_mean_m"] = z1
    exc["altitude_at_release_m"] = z_rel
    exc["pulse_altitude_gain_m"] = ((z_rel - z1) if (z1 is not None and z_rel is not None)
                                    else None)
    exc["pulse_altitude_gain_min_m"] = TH_PULSE_ALT_GAIN_MIN_M
    exc["pulse_altitude_gain_max_m"] = TH_PULSE_ALT_GAIN_MAX_M
    exc["pulse_gain_within_bounds"] = bool(
        exc["pulse_altitude_gain_m"] is not None
        and TH_PULSE_ALT_GAIN_MIN_M <= exc["pulse_altitude_gain_m"] <= TH_PULSE_ALT_GAIN_MAX_M)
    if segs.get("P3_ringdown") and segs["P3_ringdown"]["samples"]:
        exc["climb_rate_at_release_ms"] = segs["P3_ringdown"]["samples"][0]["mav"]["climb"]
        exc["airspeed_at_release_ms"] = segs["P3_ringdown"]["samples"][0]["mav"]["airspeed"]
    an["excitation"] = exc

    # ---- envelope gating ---------------------------------------------------
    if segs.get("P2_excite"):
        an["envelope_P2_excite"] = envelope_block(
            segs["P2_excite"]["samples"], p, "P2_excite", TH_SURF_FLIGHT_MAX_DEG)
    if segs.get("P3_ringdown"):
        an["envelope_P3_ringdown"] = envelope_block(
            segs["P3_ringdown"]["samples"], p, "P3_ringdown", TH_SURF_HOLD_MAX_DEG)

    # ---- THE MEASUREMENT ---------------------------------------------------
    if segs.get("P3_ringdown"):
        an["ringdown"] = ringdown_analysis(segs["P3_ringdown"]["samples"], p)

    # ---- whole flight ------------------------------------------------------
    allsamp = []
    for ph in PHASES:
        if segs.get(ph):
            allsamp.extend(segs[ph]["samples"])
    allsamp.sort(key=lambda s: s["t"])
    if allsamp:
        an["whole_flight"] = envelope_block(allsamp, p, "whole_flight",
                                            TH_SURF_FLIGHT_MAX_DEG)
        an["whole_flight"]["n_samples"] = len(allsamp)
        an["whole_flight"]["duration_s"] = (allsamp[-1]["t"] - allsamp[0]["t"]
                                            if len(allsamp) >= 2 else None)
        an["whole_flight"]["high_j"] = high_j_block(allsamp)
    an["tecs_energy_loop_gains"] = tecs_energy_loop_gains(p)
    return an


# =============================================================================
# verdict
# =============================================================================
def verdict(R):
    an = R.get("analysis")
    if not an or not an.get("ringdown") or an["ringdown"].get("insufficient_samples"):
        return "PHUGOID_DAMPING_BASELINE_FAILED", ["no analysable ring-down"]
    p = R.get("tecs_baseline_params_live", {})
    pre = R.get("param_preconditions", {})
    rd = an["ringdown"]
    env_r = an.get("envelope_P3_ringdown") or {}
    env_p = an.get("envelope_P2_excite") or {}
    wf = an.get("whole_flight") or {}
    exc = an.get("excitation") or {}
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
    c["param_preconditions_all_ok"] = all(pre.values()) if pre else False
    c["pids_unchanged"] = bool(pre.get("pids_unchanged"))
    c["ptch_trim_deg_unchanged"] = bool(pre.get("ptch_trim_deg_2p49"))
    c["zero_wind_confirmed"] = bool(pre.get("sim_wind_zero"))
    c["atmosphere_datum_ok"] = bool(pre.get("sim_opos_alt_zero_atmosphere_datum"))

    # --- 2. parameter policy ------------------------------------------------
    writes = R.get("parameter_writes") or []
    c["no_parameter_written_unless_explicitly_requested"] = (
        (len(writes) == 0) if not R.get("set_param_requested") else True)
    c["all_requested_parameter_writes_confirmed"] = all(
        w.get("confirmed") for w in writes) if writes else True
    if writes:
        # a run WITH writes is not the stage baseline; declare it loudly
        c["is_firmware_default_baseline"] = False
    else:
        c["is_firmware_default_baseline"] = bool(pre.get("tecs_at_firmware_defaults"))

    # --- 3. TECS is genuinely the authority (re-proved live) ----------------
    ta = an.get("tecs_authority") or {}
    c["tecs_is_driving_throttle_not_the_stick"] = (
        num(ta.get("abs_delta")) and ta["abs_delta"] > TH_TECS_AUTHORITY_MIN_DELTA)
    c["throttle_is_actively_modulated"] = (
        num(wf.get("throttle_range")) and wf["throttle_range"] > TH_THROTTLE_MODULATION_MIN)
    tt = (an.get("P1_trim_hold") or {}).get("tecs_target_airspeed_ms")
    c["tecs_target_airspeed_matches_command"] = (
        isinstance(tt, dict) and num(tt.get("mean"))
        and abs(tt["mean"] - V_TARGET_MS) <= TH_TECS_TARGET_TOL_MS)

    # --- 4. the excitation was what this stage says it was ------------------
    c["excitation_was_a_single_pulse"] = bool(exc.get("single_pulse"))
    c["stick_neutral_throughout_ringdown"] = bool(
        exc.get("stick_neutral_throughout_ringdown"))
    c["pulse_altitude_gain_within_bounds"] = bool(exc.get("pulse_gain_within_bounds"))
    c["ringdown_excitation_amplitude_sufficient"] = bool(
        rd.get("excitation_amplitude_ok"))

    # --- 5. the ring-down stayed linear and unsaturated ---------------------
    c["ringdown_airspeed_above_min"] = bool(env_r.get("airspeed_ok"))
    c["ringdown_elevator_within_limit"] = bool(env_r.get("elevator_ok"))
    c["ringdown_no_sustained_throttle_saturation"] = bool(
        env_r.get("throttle_no_sustained_saturation"))
    c["ringdown_pitch_demand_not_clipped"] = bool(env_r.get("pitch_demand_not_clipped"))
    c["ringdown_no_actuator_clamping"] = bool(env_r.get("no_actuator_clamping"))
    c["ringdown_lateral_surfaces_quiet"] = bool(env_r.get("lateral_ok"))
    c["pulse_elevator_within_flight_limit"] = bool(env_p.get("elevator_ok"))
    c["pulse_airspeed_above_min"] = bool(env_p.get("airspeed_ok"))
    c["whole_flight_surfaces_below_travel_margin"] = bool(
        wf.get("elevator_below_travel_margin"))
    c["all_values_finite"] = bool(wf.get("all_values_finite"))

    # --- 6. the measurement is actually resolvable --------------------------
    m = rd.get("measurability") or {}
    c["ringdown_has_enough_extrema"] = bool(m.get("extrema_ok"))
    c["ringdown_window_spans_two_tau"] = bool(m.get("window_ok"))
    c["ringdown_envelope_fit_usable"] = bool(m.get("envelope_fit_ok"))
    c["period_measured"] = num(rd.get("period_measured_s"))
    c["tau_env_measured"] = num(rd.get("tau_env_measured_s"))
    c["free_airframe_reference_available"] = num(rd.get("tau_ref_free_airframe_s"))
    c["tau_ratio_reported"] = num(rd.get("tau_ratio_closed_over_free"))

    # --- 7. the mode decays at all ------------------------------------------
    c["all_channels_decaying"] = bool(rd.get("all_channels_decaying"))
    arc = rd.get("amplitude_ratio_per_cycle")
    c["mode_is_not_growing"] = num(arc) and arc < 1.0

    # ---- DECLARED, NON-GATING: the open MAJOR indicator --------------------
    ratio = rd.get("tau_ratio_closed_over_free")
    R["open_major_indicator"] = dict(
        id="CLOSED_LOOP_LONGITUDINAL_DAMPING_WEAKER_THAN_FREE_AIRFRAME",
        status="OPEN_MAJOR",
        gating=False,
        gating_rationale=("part 1 exists to MEASURE this ratio on a purpose-built "
                          "free decay. Gating the baseline on it would be circular "
                          "and would invite tuning to pass, which CLAUDE.md's "
                          "simulation tuning policy forbids."),
        tau_ratio_closed_over_free=ratio,
        tau_ratio_prior_stage=PRIOR_ENERGY["P3_settle_tau_ratio"],
        satisfied_if_le=1.0,
        satisfied=(bool(ratio is not None and ratio <= 1.0)),
        period_ratio_closed_over_free=rd.get("period_ratio_closed_over_free"),
        period_ratio_prior_stage=(PRIOR_ENERGY["P3_settle_period_s"]
                                  / PRIOR_ENERGY["P3_settle_T_ref_s"]))

    R["acceptance_checks"] = c
    fails = sorted(k for k, v in c.items() if not v)
    vd = "PHUGOID_DAMPING_BASELINE_MEASURED" if not fails else "PHUGOID_DAMPING_BASELINE_FAILED"
    return vd, fails


# =============================================================================
# per-sample trace
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
                s_ap_target_alt_rel_m(s), s_ap_alt_rel_m(s), s_alt(s),
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
        rc2_up_us=p.get("RC2_MAX"), rc2_neutral_us=p.get("RC2_TRIM"),
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

    def run(label, dur, rc2):
        return cruise.run_seg(mav, sub, osub, adiag, pdiag, aerodiag, label, dur,
                              1500, rc2, rc3_cruise, t0, latest)

    def fail(ph, seg):
        R["flight_result"] = dict(aborted=True, reason=f"{ph}_aborted",
                                  detail=seg["abort_reason"])

    print(f"P1 TRIM: {P1_TRIM_S}s level cruise (rc2={rc2_neutral}, rc3={rc3_cruise} "
          f"-> TECS target {achieved_target:.3f} m/s)")
    segs["P1_trim"] = run("P1_trim", P1_TRIM_S, rc2_neutral)
    if segs["P1_trim"]["aborted"]:
        fail("P1_trim", segs["P1_trim"])
        return False, segs

    print(f"P2 EXCITE: ONE {EXCITE_PULSE_S}s full-up pitch-stick pulse (rc2={rc2_up})")
    segs["P2_excite"] = run("P2_excite", EXCITE_PULSE_S, rc2_up)
    if segs["P2_excite"]["aborted"]:
        fail("P2_excite", segs["P2_excite"])
        return False, segs

    print(f"P3 RINGDOWN: {RINGDOWN_S}s free decay, stick released to neutral "
          f"(rc2={rc2_neutral}) -> set_target_altitude_current() locks the "
          f"height demand (navigation.cpp:418-424)")
    segs["P3_ringdown"] = run("P3_ringdown", RINGDOWN_S, rc2_neutral)

    aborted = any(v["aborted"] for v in segs.values())
    R["flight_result"] = dict(
        aborted=aborted,
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
              "parameter_writes": R.get("parameter_writes"),
              "set_param_requested": R.get("set_param_requested"),
              "segments": {k: {kk: vv for kk, vv in v.items()} for k, v in segs.items()}}
    if R.get("live_bulk_param_dump") is not None:
        # so --reanalyze can persist the full parameter set without the .BIN
        ts_doc["live_bulk_param_dump"] = R["live_bulk_param_dump"]
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
    R["verdict"] = "PHUGOID_DAMPING_BASELINE_FAILED"
    R["blocking_phase"] = phase
    write_outputs(R, segs or {})
    print(f"FAILED at {phase} - see", OUT_JSON)
    if mav is not None:
        mav.close()
    return 1


# =============================================================================
# BULK PARAMETER AUDIT  (validation follow-up 2026-09-05, MINOR-2)
# =============================================================================
# WHY: the live MAVLink parameter dump taken at run time contains 1367 names,
# but energy.dump_params() keeps only PARAMS_OF_INTEREST and persists just
# `param_bulk_count: 1367`, discarding the values. A claim of the form "only
# one parameter differed between the two runs" was therefore NOT auditable from
# the result artifacts at all - it could only be checked by re-parsing the
# dataflash logs by hand. This block persists the FULL parameter set, with a
# hash, into the result artifact, and computes the run-to-run diff itself.
#
# NON-GATING. Nothing in acceptance_checks reads any field produced here.
#
# CLASSIFICATION. Not every differing parameter is evidence of a different
# configuration. ArduPilot writes two kinds of parameter by itself:
#   AUTO_CALIBRATION - sensor zero-points re-measured on every boot
#                      (ARSPD_OFFSET, BARO*_GND_PRESS). These CANNOT be held
#                      equal across two boots; requiring them to match would be
#                      requiring the sensors not to be calibrated.
#   FLIGHT_STATISTIC - odometry-style counters (STAT_*). They are written BY
#                      the flight and are outputs, never inputs to any loop.
# Anything else is FUNCTIONAL: it can change what the controller does. The
# claim this harness emits is therefore about FUNCTIONAL parameters only, and
# says so in those words. The non-functional deltas are still reported in full,
# with their physical size, so "negligible" is a number and not an adjective.
NON_FUNCTIONAL_PARAM_EXACT = {
    "ARSPD_OFFSET": ("AUTO_CALIBRATION",
                     "airspeed zero-offset, re-measured at every boot by the "
                     "pitot auto-zero (AP_Airspeed::calibrate); it is a "
                     "measured sensor zero, not a tuning input"),
}
NON_FUNCTIONAL_PARAM_PREFIX = {
    "STAT_": ("FLIGHT_STATISTIC",
              "cumulative flight statistics written BY the flight "
              "(AP_Stats); an output of the run, never an input to any loop"),
}


def classify_param(name):
    """FUNCTIONAL vs an explicitly recognised NON-FUNCTIONAL class. The rules
    are name-based, fixed a priori, and applied identically to every parameter
    and every run."""
    if name in NON_FUNCTIONAL_PARAM_EXACT:
        return NON_FUNCTIONAL_PARAM_EXACT[name]
    for pref, cls in NON_FUNCTIONAL_PARAM_PREFIX.items():
        if name.startswith(pref):
            return cls
    if name.startswith("BARO") and name.endswith("_GND_PRESS"):
        return ("AUTO_CALIBRATION",
                "barometric ground-pressure datum, captured at every boot by "
                "AP_Baro::update_calibration(); a measured datum, not a "
                "tuning input")
    return ("FUNCTIONAL", "")


# Unit conversions used ONLY to state the physical size of a non-functional
# delta. REPORT ONLY - nothing is gated on them.
#   ASSUMPTION_ISA_SEA_LEVEL_DENSITY_FOR_DELTA_SIZING: rho = 1.225 kg/m^3
#   (ISA sea level). The test flies at ~92 m, where rho is ~1% lower; a 1%
#   error on a quantity that is already ~4 orders of magnitude below the
#   measured effect changes nothing about the conclusion, and using the ISA
#   datum keeps the number checkable by hand.
RHO_ISA_SEA_LEVEL = 1.225


def _param_delta_physical_size(name, va, vb):
    """Translate a raw parameter delta into the physical quantity it perturbs,
    for the two auto-calibration parameters that can differ between boots."""
    d = abs((va or 0.0) - (vb or 0.0))
    if name == "ARSPD_OFFSET":
        dv = d / (RHO_ISA_SEA_LEVEL * V_TARGET_MS)
        return dict(delta_raw_Pa=d, equivalent_airspeed_error_ms=dv,
                    formula="dV = dP / (rho * V), rho = 1.225, V = V_TARGET_MS",
                    compare_to="the MEASURED airspeed standard deviation of the "
                               "two runs")
    if name.startswith("BARO") and name.endswith("_GND_PRESS"):
        dh = d / (RHO_ISA_SEA_LEVEL * G_WORLD)
        return dict(delta_raw_Pa=d, equivalent_altitude_datum_shift_m=dh,
                    formula="dh = dP / (rho * g), rho = 1.225, g = G_WORLD",
                    compare_to="the MEASURED hold altitudes of the two runs")
    return dict(delta_raw=d)


def _live_bulk_param_dump(mav):
    """Full LIVE MAVLink parameter set, captured and KEPT. energy.dump_params()
    performs the same fetch but retains only PARAMS_OF_INTEREST and persists
    just the count, which is why a run-to-run configuration claim could not be
    audited from the result artifact. READ-ONLY: PARAM_REQUEST_LIST only, no
    PARAM_SET of any kind. NON-GATING - a failure here is recorded and the run
    continues."""
    out = {"source": "MAVLink PARAM_REQUEST_LIST, live, read-only",
           "gating": False}
    try:
        import hashlib
        bulk = mav.fetch_all_params(timeout=45, idle_cutoff=3.0)
        out["params"] = {k: bulk[k] for k in sorted(bulk)}
        out["n_params"] = len(bulk)
        out["params_sha256"] = hashlib.sha256(
            json.dumps(out["params"], sort_keys=True,
                       separators=(",", ":")).encode()).hexdigest()
    except Exception as exc:          # noqa: BLE001 - reported, never silent
        out["error"] = f"{type(exc).__name__}: {exc}"
    return out


def _dataflash_param_dump(bin_path):
    """Full PARM set from one ArduPlane dataflash log. Returns the FINAL value
    of every parameter, plus every parameter whose value CHANGED during the log
    (which is how a runtime PARAM_SET shows up), plus a hash of the whole set.
    Read-only; the .BIN is never modified."""
    out = {"bin_file": bin_path}
    try:
        import hashlib
        h = hashlib.sha256()
        with open(bin_path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        out["bin_sha256"] = h.hexdigest()
        out["bin_bytes"] = os.path.getsize(bin_path)
        from pymavlink import mavutil
        m = mavutil.mavlink_connection(bin_path)
        first, last, changed = {}, {}, {}
        while True:
            msg = m.recv_match(type="PARM")
            if msg is None:
                break
            nm, val = msg.Name, msg.Value
            if nm not in first:
                first[nm] = val
            elif last.get(nm) != val:
                changed.setdefault(nm, {"first": first[nm]})["last"] = val
            last[nm] = val
        out["params"] = {k: last[k] for k in sorted(last)}
        out["n_params"] = len(last)
        out["changed_during_log"] = changed
        out["params_sha256"] = hashlib.sha256(
            json.dumps(out["params"], sort_keys=True,
                       separators=(",", ":")).encode()).hexdigest()
    except Exception as exc:          # noqa: BLE001 - reported, never silent
        out["error"] = f"{type(exc).__name__}: {exc}"
    return out


def _find_dataflash_bins(directory):
    try:
        names = sorted(n for n in os.listdir(directory) if n.upper().endswith(".BIN"))
    except OSError:
        return []
    return [os.path.join(directory, n) for n in names]


def _dataflash_dir_for(ts_path):
    """Convention used by run_ardupilot_longitudinal_phugoid_damping.sh:
    <PREFIX>_timeseries<SUFFIX>.json  <->  <PREFIX>_dataflash<SUFFIX>/ ."""
    if not ts_path:
        return None
    d = os.path.dirname(os.path.abspath(ts_path))
    b = os.path.basename(ts_path)
    if "_timeseries" not in b or not b.endswith(".json"):
        return None
    return os.path.join(d, b[:-len(".json")].replace("_timeseries", "_dataflash"))


def bulk_parameter_audit(R, ts_path=None, explicit_dir=None):
    """Persist the FULL parameter set of this run into the result artifact and
    diff it against every other captured run of this stage.

    NON-GATING, REPORT ONLY. Failure to locate a dataflash log is reported as
    an explicit `unavailable` state with a reason - never silently omitted, and
    never a test failure."""
    aud = {"purpose": ("make the run-to-run parameter-configuration claim "
                       "AUDITABLE FROM THE RESULT ARTIFACT ALONE. Previously "
                       "only param_bulk_count was persisted and the 1367 values "
                       "were discarded."),
           "gating": False,
           "source": "ArduPlane dataflash PARM messages (the log the run wrote)",
           "classification_rules": dict(
               exact={k: list(v) for k, v in NON_FUNCTIONAL_PARAM_EXACT.items()},
               prefix={k: list(v) for k, v in NON_FUNCTIONAL_PARAM_PREFIX.items()},
               pattern={"BARO*_GND_PRESS": [
                   "AUTO_CALIBRATION",
                   "barometric ground-pressure datum captured at every boot"]},
               default="FUNCTIONAL"),
           "wording_note": (
               "Any claim derived from this block must say 'exactly N "
               "FUNCTIONAL parameters differ', NOT 'exactly N parameters "
               "differ'. Auto-calibration and flight-statistic parameters "
               "CANNOT be identical across two boots and their presence in a "
               "diff is expected, not a confound. Their physical size is "
               "reported so the negligibility is a number.")}
    live = R.get("live_bulk_param_dump")
    if live:
        aud["live_run_dump"] = live
    d = explicit_dir or _dataflash_dir_for(ts_path)
    aud["dataflash_dir"] = d
    bins = _find_dataflash_bins(d) if d else []
    if not bins:
        if live and live.get("params"):
            aud["this_run"] = dict(live)
            aud["this_run"]["source"] = (
                "LIVE MAVLink dump - no dataflash log was reachable from this "
                "artifact when it was written")
            aud["dataflash_unavailable"] = (
                f"no .BIN found under {d!r}" if d else
                "no dataflash directory could be resolved")
            aud["counterpart_diffs"] = []
            aud["counterpart_diffs_note"] = (
                "no counterpart diff is possible without the dataflash logs; "
                "the FULL parameter set of this run is persisted above, so the "
                "diff can be taken against any other run's artifact directly.")
            R["bulk_parameter_dump"] = aud
            return aud
        aud["unavailable"] = (
            f"no .BIN found under {d!r}" if d else
            "no dataflash directory could be resolved from the timeseries path")
        aud["DATA_REQUIRED"] = (
            "the run's ArduPlane dataflash log, or a live bulk PARAM dump, is "
            "required for the full parameter set to be persisted here")
        R["bulk_parameter_dump"] = aud
        return aud
    aud["bin_files"] = bins
    this = _dataflash_param_dump(bins[-1])
    aud["this_run"] = this
    if this.get("error"):
        aud["unavailable"] = this["error"]
        R["bulk_parameter_dump"] = aud
        return aud
    pr = this["params"]
    # TECS values, against the compiled firmware defaults this stage expects
    tecs = {}
    for k, v in sorted(TECS_FIRMWARE_DEFAULTS.items()):
        got = pr.get(k)
        tecs[k] = dict(value=got, firmware_default=v,
                       differs=(got is None or abs(got - v) > 1e-6))
    aud["tecs_vs_firmware_defaults"] = tecs
    aud["tecs_differing_from_firmware_defaults"] = sorted(
        k for k, v in tecs.items() if v["differs"])

    # diff against every OTHER captured run of this stage
    root = os.path.dirname(os.path.abspath(d))
    stem = os.path.basename(d).split("_dataflash")[0] + "_dataflash"
    others = []
    try:
        for nm in sorted(os.listdir(root)):
            full = os.path.join(root, nm)
            if full != os.path.abspath(d) and nm.startswith(stem) and os.path.isdir(full):
                others.append(full)
    except OSError:
        pass
    diffs = []
    for od in others:
        ob = _find_dataflash_bins(od)
        if not ob:
            continue
        other = _dataflash_param_dump(ob[-1])
        if other.get("error"):
            diffs.append({"counterpart_dir": od, "error": other["error"]})
            continue
        op = other["params"]
        names = sorted(set(pr) | set(op))
        func, nonfunc = {}, {}
        for nm in names:
            a, b = pr.get(nm), op.get(nm)
            if a is None or b is None or abs(a - b) > 1e-9:
                cls, why = classify_param(nm)
                rec = {"this_run": a, "counterpart": b, "class": cls,
                       "class_reason": why}
                if cls == "FUNCTIONAL":
                    func[nm] = rec
                else:
                    rec["physical_size"] = _param_delta_physical_size(nm, a, b)
                    nonfunc[nm] = rec
        diffs.append(dict(
            counterpart_dir=od, counterpart_bin=ob[-1],
            counterpart_params_sha256=other.get("params_sha256"),
            n_params_this=len(pr), n_params_counterpart=len(op),
            n_differing_total=len(func) + len(nonfunc),
            n_differing_functional=len(func),
            n_differing_non_functional=len(nonfunc),
            functional_differences=func,
            non_functional_differences=nonfunc,
            claim=(f"exactly {len(func)} FUNCTIONAL parameter"
                   f"{'' if len(func) == 1 else 's'} differ"
                   f"{'s' if len(func) == 1 else ''} between these two runs "
                   f"({sorted(func) if func else 'none'}); "
                   f"{len(nonfunc)} further differences are auto-calibration or "
                   f"flight-statistic values, which are re-measured or written "
                   f"by every boot and cannot be held equal across runs"),
            negligibility_argument=(
                "each non-functional delta is converted above into the physical "
                "quantity it perturbs (physical_size). Compare each against the "
                "MEASURED difference between the two runs in that same quantity "
                "before treating it as a confound.")))
    aud["counterpart_diffs"] = diffs
    R["bulk_parameter_dump"] = aud
    return aud


def provenance_block():
    """STATIC declarations that MUST appear whether the artifact came from a
    flight or from --reanalyze (the same requirement validation finding V-13
    imposed on the previous stage). Contains no flight measurement."""
    return {
        "part": PART,
        "mode": dict(name="FBWB", custom_mode=ARDUPLANE_FBWB_CUSTOM_MODE,
                     evidence="docs/source_of_truth/controls/ardupilot_fbwb_tecs_"
                              "baseline.yaml. TECS throttle authority is RE-PROVED "
                              "LIVE here (tecs_is_driving_throttle_not_the_stick), "
                              "never inherited on trust."),
        "parameter_policy": (
            "DEFAULT: writes NO parameter of any kind - TECS runs on ArduPlane "
            "compiled firmware defaults. config/ardupilot/falcon_v2_sitl.parm is "
            "READ-ONLY input and is never edited. `--set-param NAME=VALUE` is an "
            "explicit opt-in that performs a RUNTIME MAVLink PARAM_SET in the "
            "SITL scratch EEPROM only, restricted to SETTABLE_PARAMS (TECS "
            "energy-loop parameters) and to each parameter's own ArduPilot "
            "@Range; every other name is REFUSED. No PID, no PTCH_TRIM_DEG, no "
            "+/-45 deg surface scaling, no aero/propulsion/actuator/sensor/"
            "mass/CG/inertia value can be written through this harness."),
        "scope_exclusions": [
            "INNER_PITCH_RATE_LOOP_OUT_OF_SCOPE: PTCH_RATE_*/PTCH2SRV_* are not "
            "read as candidates, not written, and are never blamed by this stage.",
            "AUTOTUNE is never run. LOITER/AUTO/RTL are never entered.",
        ],
        "open_limitations": [
            "PROPULSION_HIGH_J_WINDMILLING",
            "CLOSED_LOOP_LONGITUDINAL_DAMPING_WEAKER_THAN_FREE_AIRFRAME",
        ],
        "open_limitations_note": (
            "PROPULSION_HIGH_J_WINDMILLING (owner: propulsion, DATA_REQUIRED) is "
            "carried unchanged from the 2026-09-03 energy stage: the APC 13x6.5E "
            "Ct/Cp table ends at the zero-thrust advance ratio, so thrust is "
            "floored at 0 N where a real fixed-pitch prop would windmill. It is "
            "COUNTED and REPORTED here (high_j block) and never gates. The "
            "excitation is deliberately an UP pulse so the transient itself does "
            "not depend on that region. "
            "CLOSED_LOOP_LONGITUDINAL_DAMPING_WEAKER_THAN_FREE_AIRFRAME is the "
            "open MAJOR this stage exists to quantify; see open_major_indicator "
            "in the result artifact. It is declared and NON-GATING in part 1."),
        "declared_known_limitations": [
            dict(id="WINDOW_OVER_TAU_COMPUTED_ON_FULL_SPAN_NOT_SNR_ADMITTED_SPAN",
                 status="DECLARED_KNOWN_LIMITATION",
                 gating=False, gate_changed=False, owner="gazebo-testing",
                 raised_by="validation, 2026-09-05, non-blocking MINOR-1",
                 statement=(
                     "the identifiability check ringdown_window_spans_two_tau "
                     "computes window_over_tau on the FULL analysed span, while "
                     "the headline tau is estimated from the SNR-ADMITTED span. "
                     "Where the truncation is load-bearing those spans differ."),
                 numbers=dict(
                     candidate_full_span_s=54.906, candidate_tau_s=8.652,
                     window_over_tau_full_span=6.35, verdict_full_span="PASS",
                     candidate_snr_admitted_span_s=16.646,
                     window_over_tau_admitted_span=1.92,
                     min_window_over_tau=TH_MIN_WINDOW_TAU,
                     verdict_admitted_span="would be BELOW the 2.0 intent"),
                 deliberately_not_fixed_because=(
                     "the results are already known. Re-deriving how an "
                     "acceptance criterion is computed AFTER seeing the outcome "
                     "is an outcome-driven test edit and is forbidden by "
                     "CLAUDE.md's simulation tuning policy - in either "
                     "direction. The gate is left exactly as it was and this "
                     "limitation is carried openly instead."),
                 future_stage_work=(
                     "decide, BEFORE the next measurement is taken, which span "
                     "the identifiability criterion should be evaluated on, and "
                     "re-derive RINGDOWN_S from that decision."),
                 reported_per_run_at=("analysis.ringdown.measurability."
                                      "KNOWN_LIMITATION_window_over_tau_uses_"
                                      "full_span")),
        ],
        "snr_truncation_asymmetry": (
            "The SNR truncation added on 2026-09-04 is not symmetric between a "
            "lightly damped and a well damped run: it is INERT on the former "
            "(nothing reaches the floor, 0 extrema rejected, result identical "
            "to no truncation) and LOAD-BEARING on the latter. 'The baseline is "
            "essentially unchanged' therefore validates the mean-of-logs change "
            "and NOT the truncation. Both truncation-free views are reported on "
            "every run - see analysis.ringdown.no_truncation_all_extrema and "
            "analysis.ringdown.full_window_damped_sinusoid_fit - together with "
            "the direct evidence that the rejected tail is non-monotone floor "
            "noise (analysis.ringdown.rejected_tail_evidence)."),
        "non_functional_parameter_class": (
            "A run-to-run parameter diff is classified before any claim is made "
            "about it: AUTO_CALIBRATION (ARSPD_OFFSET, BARO*_GND_PRESS - "
            "re-measured at every boot) and FLIGHT_STATISTIC (STAT_* - written "
            "BY the flight) are NON-FUNCTIONAL and cannot be held equal across "
            "two boots; everything else is FUNCTIONAL. Claims must therefore be "
            "worded 'exactly N FUNCTIONAL parameters differ'. The full "
            "parameter set, its hash and the classified diff are persisted in "
            "bulk_parameter_dump so the claim is auditable from the result "
            "artifact alone."),
        "assumptions": [
            "PHUGOID_REFERENCE_IS_LANCHESTER - inherited unchanged from "
            "energy.phugoid_reference(). tau_ref = V*(L/D)/g, T_ref = "
            "pi*sqrt(2)*V/g. A MEASURED Falcon V2 stick-fixed phugoid (period + "
            "damping) is DATA_REQUIRED and would replace it directly.",
            "TECS_SEB_MANIFOLD_LINEARISATION - only used by the REPORT-ONLY "
            "tecs_energy_loop_gains diagnostic; never gated.",
            "ASSUMPTION_EXTREMUM_SNR_DETECTION_THRESHOLD - an extremum of the "
            "detrended ring-down is admitted into the logarithmic-decrement "
            "estimate only if its amplitude is at least "
            f"TH_SNR_DETECTION_MULTIPLE = {TH_SNR_DETECTION_MULTIPLE} times the "
            "INCOHERENT floor of that channel, where the floor is measured from "
            "the run's own data as the RMS left after removing a linear trend "
            "and the best-fit sinusoid at the mode period from the last "
            f"TH_NOISE_TAIL_CYCLES = {TH_NOISE_TAIL_CYCLES} periods of the "
            "analysed window. RATIONALE: 3 sigma is the conventional limit of "
            "detection (ISO 11843 / IUPAC), ~0.13% one-sided Gaussian "
            "false-admission rate per extremum; below it ln(A) is dominated by "
            "the floor rather than by the decay, and the logarithm of a "
            "floor-level amplitude is not a measurement of anything. The "
            "multiple is FIXED A PRIORI, is identical for every channel and "
            "every run, references no run's outcome, and its influence is "
            "reported in full (corrected_log_decrement.snr_sensitivity) rather "
            "than hidden. The coherent component at the mode period is "
            "deliberately EXCLUDED from the floor because it is the signal "
            "being measured; counting it as noise would make the threshold "
            "scale with how much signal remains, penalising a lightly damped "
            "run for still oscillating. DATA_REQUIRED to remove this "
            "assumption entirely: an independently characterised altitude "
            "measurement-noise spectrum for the Gazebo/SITL sensor chain. "
            "ASYMMETRY, STATED EXPLICITLY (validation follow-up 2026-09-05): "
            "this truncation is INERT on a lightly damped run - the envelope "
            "never reaches the floor inside the window, 0 extrema are "
            "rejected, and the estimate is bit-identical to the untruncated "
            "one - and LOAD-BEARING on a well damped run, where the envelope "
            "does reach the floor. Therefore the observation that the BASELINE "
            "is unchanged validates the MEAN-OF-LOGS half of the 2026-09-04 "
            "estimator change ONLY; it says nothing about the truncation half, "
            "which by construction could not have moved the baseline. The "
            "truncation is justified instead by two things reported on every "
            "run: (a) the rejected tail is demonstrably NON-MONOTONE floor "
            "noise rather than a decaying envelope (ringdown."
            "rejected_tail_evidence: an envelope that GROWS is not an "
            "envelope), and (b) the qualitative conclusion survives with NO "
            "truncation at all (ringdown.tau_env_no_truncation_all_extrema_s "
            "and ringdown.tau_env_damped_sinusoid_fit_s, the latter using no "
            "extrema whatsoever).",
        ],
        "reference_constants": dict(
            MASS_KG=MASS_KG, S_REF_M2=S_REF_M2, G_WORLD=G_WORLD, G_TECS=G_TECS,
            V_TRIM_REF=V_TRIM_REF, TRIM_THROTTLE_REF=TRIM_THROTTLE_REF,
            ELEV_TRIM_DEG_REF=ELEV_TRIM_DEG_REF,
            PTCH_TRIM_DEG_EXPECTED=PTCH_TRIM_DEG_EXPECTED,
            SURFACE_TRAVEL_LIMIT_DEG=SURFACE_TRAVEL_LIMIT_DEG,
            V_TARGET_MS=V_TARGET_MS,
            prior_stage_measurements=PRIOR_ENERGY),
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
            TH_TECS_TARGET_TOL_MS=TH_TECS_TARGET_TOL_MS,
            TH_SAT_RUN_MAX_S=TH_SAT_RUN_MAX_S, TH_SAT_MARGIN=TH_SAT_MARGIN,
            TH_TECS_AUTHORITY_MIN_DELTA=TH_TECS_AUTHORITY_MIN_DELTA,
            TH_THROTTLE_MODULATION_MIN=TH_THROTTLE_MODULATION_MIN,
            TH_SURF_HOLD_MAX_DEG=TH_SURF_HOLD_MAX_DEG,
            TH_SURF_FLIGHT_MAX_DEG=TH_SURF_FLIGHT_MAX_DEG,
            TH_LATERAL_SURF_MAX_DEG=TH_LATERAL_SURF_MAX_DEG,
            TH_SURF_MAX_ABS_DEG=TH_SURF_MAX_ABS_DEG,
            TH_DECAY_RATIO_MAX=TH_DECAY_RATIO_MAX,
            HOLD_TRANSIENT_S=HOLD_TRANSIENT_S, SETTLE_TAIL_S=SETTLE_TAIL_S,
            source="tests/gazebo/scripts/test_ardupilot_tecs_climb_descent_energy.py "
                   "(2026-09-03) and test_ardupilot_tecs_cruise_speed_hold.py "
                   "(2026-09-02). NO inherited threshold is changed by this stage."),
        DERIVED_THIS_STAGE=dict(
            TH_MIN_EXTREMA=TH_MIN_EXTREMA,
            TH_MIN_WINDOW_TAU=TH_MIN_WINDOW_TAU,
            TH_ENVELOPE_FIT_R2_MIN=TH_ENVELOPE_FIT_R2_MIN,
            TH_PULSE_ALT_GAIN_MIN_M=TH_PULSE_ALT_GAIN_MIN_M,
            TH_PULSE_ALT_GAIN_MAX_M=TH_PULSE_ALT_GAIN_MAX_M,
            TH_RINGDOWN_A0_MIN_M=TH_RINGDOWN_A0_MIN_M,
            TH_PITCH_DEMAND_MARGIN_DEG=TH_PITCH_DEMAND_MARGIN_DEG,
            provenance="each derived from an inherited threshold or a documented "
                       "physical quantity - see the comment block above each "
                       "constant in this module"),
        ESTIMATOR_SETTINGS_NOT_ACCEPTANCE_THRESHOLDS=dict(
            TH_SNR_DETECTION_MULTIPLE=TH_SNR_DETECTION_MULTIPLE,
            TH_NOISE_TAIL_CYCLES=TH_NOISE_TAIL_CYCLES,
            TH_MIN_ADMITTED_EXTREMA=TH_MIN_ADMITTED_EXTREMA,
            note=("added by the 2026-09-04 TEST-LOGIC FIX. These configure HOW "
                  "the envelope decay rate is ESTIMATED. They are NOT acceptance "
                  "criteria: no check compares a measurement against them. NO "
                  "acceptance threshold value was changed by that fix - every "
                  "INHERITED and DERIVED_THIS_STAGE value above is exactly what "
                  "it was before it."),
            assumption_tag="ASSUMPTION_EXTREMUM_SNR_DETECTION_THRESHOLD"),
        PHASE_PLAN=dict(P1_TRIM_S=P1_TRIM_S, EXCITE_PULSE_S=EXCITE_PULSE_S,
                        RINGDOWN_S=RINGDOWN_S, TOTAL_FLIGHT_S=TOTAL_FLIGHT_S,
                        RELEASE_LATENCY_BOUND_S=RELEASE_LATENCY_BOUND_S))


def param_precondition_checks(p, R):
    chk = dict(energy.param_precondition_checks(p, R))

    def eq(name, val):
        return p.get(name) is not None and abs(p[name] - val) < 1e-6

    chk["tecs_pitch_max_15"] = eq("TECS_PITCH_MAX", 15.0)
    chk["ptch_lim_min_deg_minus25"] = eq("PTCH_LIM_MIN_DEG", -25.0)
    # If a --set-param write was requested, the "firmware defaults" precondition
    # inherited from the energy stage will (correctly) be False. Record which
    # TECS values were deliberately changed so that is never ambiguous.
    writes = R.get("parameter_writes") or []
    if writes:
        chk["tecs_at_firmware_defaults"] = True   # neutralised - see note
        R["param_precondition_override_note"] = (
            "tecs_at_firmware_defaults was neutralised for this run because "
            f"--set-param deliberately wrote {[w['name'] for w in writes]}. The "
            "run is NOT the stage baseline; is_firmware_default_baseline is "
            "False in acceptance_checks and the before/after values are in "
            "parameter_writes.")
    R["param_preconditions"] = chk
    print("param preconditions:", json.dumps(chk, default=str))
    return chk


def reanalyze(path, dataflash_dir=None):
    with open(path) as f:
        doc = json.load(f)
    segs = doc["segments"]
    p = doc["tecs_baseline_params_live"]
    R = {"stage": STAGE, "timestamp": doc.get("timestamp"),
         **provenance_block(),
         "tecs_baseline_params_live": p,
         "command_derivation": doc.get("command_derivation"),
         "parameter_writes": doc.get("parameter_writes"),
         "set_param_requested": doc.get("set_param_requested"),
         "thresholds": threshold_block(), "reanalyzed_from": path,
         "provenance_blocks_source": (
             "mode / parameter_policy / scope_exclusions / open_limitations / "
             "assumptions / reference_constants are STATIC declarations "
             "regenerated from this module by provenance_block(); they are not "
             "flight measurements and are not read from the timeseries file.")}
    if doc.get("live_bulk_param_dump") is not None:
        R["live_bulk_param_dump"] = doc["live_bulk_param_dump"]
    param_precondition_checks(p, R)
    # NON-GATING (validation follow-up 2026-09-05, MINOR-2): persist the FULL
    # parameter set + the classified run-to-run diff into the result artifact.
    bulk_parameter_audit(R, ts_path=path, explicit_dir=dataflash_dir)
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
    rd = an.get("ringdown") or {}
    try:
        print("-" * 78)
        exc = an.get("excitation") or {}
        print(f"excitation      : single pulse {exc.get('pulse_duration_s')} s "
              f"rc2 {exc.get('pulse_rc2_us')} -> neutral {exc.get('neutral_rc2_us')} "
              f"| alt gain {exc.get('pulse_altitude_gain_m')} m "
              f"| vz@release {exc.get('climb_rate_at_release_ms')} m/s "
              f"| V@release {exc.get('airspeed_at_release_ms')} m/s")
        print(f"ring-down A0    : alt {rd.get('A0_altitude_m')} m @ "
              f"{rd.get('A0_altitude_t_s')} s | V {rd.get('A0_airspeed_ms')} m/s @ "
              f"{rd.get('A0_airspeed_t_s')} s")
        print(f"MEASURED  mode  : T {rd.get('period_measured_s')} s | "
              f"zeta {rd.get('zeta_measured')} | r_cycle "
              f"{rd.get('amplitude_ratio_per_cycle')} | n_extrema "
              f"{rd.get('n_extrema')}")
        ec = rd.get('estimator_comparison') or {}
        lg, cr, sh = ec.get('legacy') or {}, ec.get('corrected') or {}, ec.get('shift') or {}
        print(f"ESTIMATOR       : headline = CORRECTED pooled log-decrement "
              f"(mean of logs), SNR x{TH_SNR_DETECTION_MULTIPLE}")
        print(f"  LEGACY  (arith mean of ratios, all extrema): T "
              f"{lg.get('period_s')} | zeta {lg.get('damping_ratio_zeta')} | "
              f"r_cycle {lg.get('amplitude_ratio_per_cycle')} | tau "
              f"{lg.get('tau_env_log_decrement_s')} | n {lg.get('n_extrema_used')}")
        print(f"  CORRECTED (mean of logs, SNR-admitted)     : T "
              f"{cr.get('period_s')} | zeta {cr.get('damping_ratio_zeta')} | "
              f"r_cycle {cr.get('amplitude_ratio_per_cycle')} | tau "
              f"{cr.get('tau_env_log_decrement_s')} | n {cr.get('n_extrema_used')} "
              f"(rejected {cr.get('n_extrema_rejected_below_floor')}) | "
              f"tau_reg {cr.get('tau_env_regression_admitted_s')} | usable "
              f"{cr.get('usable')} {cr.get('unmeasurable') or ''}")
        print(f"  SHIFT corrected-vs-legacy: tau {sh.get('tau_pct')} % | T "
              f"{sh.get('period_pct')} % | zeta {sh.get('zeta_pct')} % | r_cycle "
              f"{sh.get('r_cycle_pct')} %")
        nt = rd.get('no_truncation_all_extrema') or {}
        dsf = rd.get('full_window_damped_sinusoid_fit') or {}
        print(f"  NO-TRUNCATION (mean of logs, ALL extrema, NON-GATING)      : T "
              f"{nt.get('period_s')} | zeta {nt.get('damping_ratio_zeta')} | tau "
              f"{nt.get('tau_env_log_decrement_s')} | n {nt.get('n_extrema_used')}")
        print(f"  DAMPED-SINUSOID FIT (no extrema, no truncation, NON-GATING): T "
              f"{dsf.get('period_s')} | zeta {dsf.get('damping_ratio_zeta')} | tau "
              f"{dsf.get('tau_env_s')} | r2 {dsf.get('r2')} | n "
              f"{dsf.get('n_samples')} | converged {dsf.get('converged')} | "
              f"interior {dsf.get('optimum_interior_to_brackets')}")
        rj = rd.get('rejected_tail_evidence') or {}
        print(f"  rejected tail   : n {rj.get('n_rejected')} | increasing steps "
              f"{rj.get('n_increasing_steps')}/{rj.get('n_steps')} | monotone "
              f"{rj.get('monotonically_decreasing')} | max/first "
              f"{rj.get('max_over_first')} | amps {rj.get('rejected_amplitudes')}")
        print(f"  tau ratio closed/free by estimator: headline "
              f"{rd.get('tau_ratio_closed_over_free')} | no-truncation "
              f"{rd.get('tau_ratio_closed_over_free_no_truncation')} | "
              f"damped-sinusoid fit "
              f"{rd.get('tau_ratio_closed_over_free_damped_sinusoid_fit')}")
        kl = ((rd.get('measurability') or {}).get(
            'KNOWN_LIMITATION_window_over_tau_uses_full_span') or {})
        print(f"  KNOWN LIMITATION (declared, gate UNCHANGED): window_over_tau "
              f"full span {kl.get('gated_value')} vs SNR-admitted span "
              f"{kl.get('window_over_tau_on_snr_admitted_span')} "
              f"(min {kl.get('min_window_over_tau')})")
        bp = R.get('bulk_parameter_dump') or {}
        tr = bp.get('this_run') or {}
        print(f"BULK PARAMS     : {tr.get('n_params')} params sha256 "
              f"{tr.get('params_sha256')} {bp.get('unavailable') or ''}")
        for dd in (bp.get('counterpart_diffs') or []):
            print(f"  vs {os.path.basename(dd.get('counterpart_dir') or '')}: "
                  f"{dd.get('claim') or dd.get('error')}")
        print(f"  incoherent floor {rd.get('incoherent_floor')} -> detection "
              f"threshold {rd.get('snr_detection_threshold')} "
              f"(admitted {rd.get('n_extrema_admitted')} of {rd.get('n_extrema')})")
        print(f"MEASURED  tau   : log-dec {rd.get('tau_env_measured_s')} s | "
              f"from-zeta {rd.get('tau_env_measured_from_zeta_s')} s | "
              f"envelope-fit {rd.get('tau_env_measured_envelope_fit_s')} s "
              f"(r2 {rd.get('envelope_fit_r2')})")
        print(f"FREE AIRFRAME   : tau_ref {rd.get('tau_ref_free_airframe_s')} s | "
              f"T_ref {rd.get('T_ref_free_airframe_s')} s | zeta_ref "
              f"{rd.get('zeta_ref_free_airframe')}")
        print(f"RATIOS          : tau closed/free {rd.get('tau_ratio_closed_over_free')} "
              f"(prior stage 1.240) | period closed/free "
              f"{rd.get('period_ratio_closed_over_free')} (prior stage 0.693)")
        print(f"decay ratios    : {rd.get('decay_ratios')} "
              f"(limit {rd.get('decay_ratio_threshold')})")
        g = an.get("tecs_energy_loop_gains") or {}
        print(f"TECS gains (REPORT ONLY): Kp_eff {g.get('Kp_eff_per_s')} 1/s "
              f"(INTEG_GAIN share {g.get('Kp_fraction_from_INTEG_GAIN')}) | "
              f"Kd_eff {g.get('Kd_eff')} | PD zero {g.get('pd_zero_rad_s')} rad/s | "
              f"ideal loop gain {g.get('ideal_height_loop_gain_rad_s')} rad/s")
        print(f"omega_d / ideal loop gain: "
              f"{rd.get('measured_omega_d_over_ideal_loop_gain')}")
        omi = R.get("open_major_indicator") or {}
        print(f"OPEN MAJOR      : {omi.get('id')} satisfied={omi.get('satisfied')} "
              f"(NON-GATING in part 1)")
        env_r = an.get("envelope_P3_ringdown") or {}
        print(f"ring-down env   : V_min {env_r.get('airspeed_min_ms')} m/s | "
              f"elev_max {env_r.get('elevator_max_abs_deg')} deg | thr "
              f"{env_r.get('throttle_min')}..{env_r.get('throttle_max')} | "
              f"nav_pitch margins {env_r.get('nav_pitch_margin_to_min_deg')} / "
              f"{env_r.get('nav_pitch_margin_to_max_deg')} deg")
        wf = an.get("whole_flight") or {}
        hj = wf.get("high_j") or {}
        print(f"HIGH-J (OPEN_LIMITATION): interpClamped "
              f"{hj.get('interp_clamped_samples')}/{hj.get('motor_samples')} "
              f"motor-samples, zero-thrust {hj.get('zero_thrust_samples')}")
        pw = R.get("parameter_writes") or []
        print(f"parameter writes: {pw if pw else 'NONE (firmware defaults baseline)'}")
        print("-" * 78)
    except Exception as exc:      # summary print only - JSON is authoritative
        print("summary print failed:", exc)


def main():
    argv = sys.argv[1:]
    df = None
    if "--dataflash" in argv:
        i = argv.index("--dataflash")
        if i + 1 >= len(argv):
            print("ERROR: --dataflash given with no directory", file=sys.stderr)
            return 2
        df = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    if len(argv) >= 2 and argv[0] == "--reanalyze":
        return reanalyze(argv[1], dataflash_dir=df)

    set_params, perr = parse_set_param_args(argv)
    if perr:
        print("ERROR:", perr, file=sys.stderr)
        return 2

    R = {"stage": STAGE,
         "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         **provenance_block(),
         "set_param_requested": [f"{n}={v}" for n, v in set_params],
         "thresholds": threshold_block()}
    if set_params:
        print("WARNING: --set-param requested; this run is NOT the stage baseline:",
              R["set_param_requested"])

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
    print("PHASE 1 (mavlink arm):", json.dumps(R.get("phase1_mavlink_arm", {}), default=str)[:300])
    if not armed:
        return finish_fail(R, "phase1_mavlink_arm", mav)

    # OPT-IN parameter writes happen BEFORE the param dump so the dump records
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

    # NON-GATING (validation follow-up 2026-09-05, MINOR-2): keep the FULL
    # live parameter set, which energy.dump_params() fetches but discards.
    R["live_bulk_param_dump"] = _live_bulk_param_dump(mav)
    print("live bulk param dump:", R["live_bulk_param_dump"].get("n_params"),
          "params, sha256", R["live_bulk_param_dump"].get("params_sha256"),
          R["live_bulk_param_dump"].get("error") or "")
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
                "TECS_INTEG_GAIN", "PTCH_LIM_MIN_DEG", "TECS_PITCH_MAX"]
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
        R["verdict"] = "PHUGOID_DAMPING_BASELINE_FAILED"
        print("FLIGHT ABORTED:", R.get("flight_result"))

    bulk_parameter_audit(R, ts_path=OUT_TS, explicit_dir=df)
    write_outputs(R, segs, p)
    print("RESULT:", R["overall_result"], R.get("verdict"), "->", OUT_JSON)
    print("TIMESERIES:", OUT_TS)
    print("PER-SAMPLE TRACE:", OUT_TRACE)
    mav.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

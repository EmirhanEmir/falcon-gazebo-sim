#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_TECS_CLIMB_DESCENT_AND_ENERGY_VALIDATION
(controls-integration, 2026-09-03).

GOAL
----
Verify that ArduPlane TECS performs correct throttle + pitch ENERGY MANAGEMENT
on Falcon V2 across a five-phase campaign:

    P1 CRUISE     level cruise at ~18 m/s + altitude hold
    P2 CLIMB      +10 m, commanded through the FBWB pitch-stick ramp
    P3 SETTLE     level off at the new altitude and demonstrate it SETTLES
    P4 DESCENT    -10 m, commanded back down toward the ORIGINAL altitude
    P5 RESETTLE   hold near the ORIGINAL altitude and demonstrate it settles

RELATIONSHIP TO THE PREVIOUS STAGE (do not duplicate, do not re-litigate)
------------------------------------------------------------------------
ARDUPLANE_TECS_AND_CRUISE_SPEED_HOLD_VALIDATION (2026-09-02,
docs/test_results/2026-09-02_ardupilot_tecs_and_cruise_speed_hold_validation.md)
already established, and this stage does NOT re-audit:
  * FBWB is the correct mode and TECS genuinely drives BOTH throttle and pitch,
  * the RC3 -> target-airspeed and RC2 -> target-altitude command mappings,
  * the PTCH_TRIM_DEG telemetry convention,
  * that the baseline is stable at cruise (43/43 checks PASS, verdict
    TECS_CRUISE_SPEED_HOLD_PASS).
That stage's test module is IMPORTED here (run_seg, enter_fbwb, the command
inversions, the window analysis, every threshold that still applies) rather
than copied, so the two stages stay numerically comparable.

WHAT THIS STAGE ADDS - the four things the cruise stage did NOT do:
  1. An explicit SETTLE phase between the climb and the descent, with a
     quantitative settling-time + decay measurement (P3), not just a hold.
  2. A RESETTLE phase that returns to and holds near the ORIGINAL starting
     altitude. The descent is commanded to stop at the P1 reference altitude
     (the cruise stage stopped at "last altitude - 10 m" and finished parked
     at the descended altitude with no return criterion).
  3. Phase-resolved ENERGY MANAGEMENT as a first-class output: specific
     potential / kinetic / total energy and their rates, the TECS specific
     energy BALANCE, and the throttle-vs-pitch division of labour - gated,
     not narrated.
  4. Settling / decay analysis after EACH transient (P3 after the climb, P5
     after the descent), i.e. does the oscillation actually decay - a strictly
     stronger requirement than the previous stage's "is not growing".

  5. (Also new.) The commanded altitude step is verified against ARDUPLANE'S
     OWN TARGET ALTITUDE, reconstructed live over MAVLink, instead of assuming
     the FBWB stick ramp integrated correctly. See TARGET ALTITUDE READBACK.

MODE: FBWB (MAVLink custom_mode 6). All mode-selection evidence, the proof
that TECS - not the stick - is the throttle authority, the command mapping and
the pitch-telemetry caveat are in the imported module's docstring and in
docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml. The
TECS_IS_DRIVING_THROTTLE check is re-run here as a GATING criterion; it is not
inherited on trust.

TARGET ALTITUDE READBACK (new derivation, cited to source)
----------------------------------------------------------
  NAV_CONTROLLER_OUTPUT.alt_error = Plane::calc_altitude_error_cm() * 0.01
      [ArduPlane/GCS_MAVLink_Plane.cpp:240]
  calc_altitude_error_cm() = target_altitude.amsl_cm - adjusted_altitude_cm()
      [ArduPlane/altitude.cpp:389-398], the terrain branch being excluded
      because TERRAIN_FOLLOW = 0 (checked as a precondition).
  adjusted_altitude_cm() = current_loc.alt - ALT_OFFSET*100
      [ArduPlane/altitude.cpp], ALT_OFFSET = 0 (checked as a precondition).
  GLOBAL_POSITION_INT.relative_alt = current AMSL - home AMSL.
  =>  ap_target_alt_rel_m = GLOBAL_POSITION_INT.relative_alt/1000
                            + NAV_CONTROLLER_OUTPUT.alt_error
      is ArduPlane's OWN height demand, expressed above home.

  UNIT CORRECTION vs the 2026-09-02 documentation: alt_error is a MAVLink
  `float` in metres (common.xml NAV_CONTROLLER_OUTPUT; confirmed live via
  pymavlink fieldtypes -> ['float','float','int16_t','int16_t','uint16_t',
  'float','float','float']). It is NOT an int16 with 1 m resolution - only
  nav_bearing/target_bearing are int16. The
  docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml line that
  said "int16 -> 1 m resolution" was wrong and is corrected by this stage.
  This matters: the full-precision field is what makes the target-altitude
  verification below possible at all.

ENERGY DEFINITIONS - TECS's own, cited to source, not invented here
-------------------------------------------------------------------
  SPE  = h * g                    [AP_TECS.cpp:689  _SPE_est = _height*GRAVITY_MSS]
  SKE  = 0.5 * TAS^2              [AP_TECS.cpp:690]
  STE  = SPE + SKE
  SPEdot = climb_rate * g         [AP_TECS.cpp:694]
  SKEdot = TAS * dTAS/dt          [AP_TECS.cpp:695, see FILTER NOTE below]
  STEdot = SPEdot + SKEdot        -> what the THROTTLE loop controls
                                     [AP_TECS.cpp:739-772]
  SEB  = SPE*w_SPE - SKE*w_SKE    [AP_TECS.cpp:1031]
  SEBdot = SPEdot*w_SPE - SKEdot*w_SKE
                                  -> what the PITCH loop controls
                                     [AP_TECS.cpp:1036-1096]
  w_SKE = min(constrain(TECS_SPDWEIGHT,0,2), 1)   [AP_TECS.cpp:1003,1028]
  w_SPE = min(2 - w_SKE, 1)                       [AP_TECS.cpp:1024,1027]
  (read LIVE from TECS_SPDWEIGHT; at the baseline 1.0 both weights are 1.0.)

  FILTER NOTE - a deliberate, declared difference: AP_TECS computes
  _SKEdot = _TAS_state * (_vel_dot - _vel_dot_lpf), i.e. a HIGH-PASSED
  acceleration, to reject complementary-filter bias. This test computes the
  RAW kinematic rate dTAS/dt by linear regression over the analysis window.
  Over a window that is long compared with the TECS filters these agree in the
  mean but not sample-by-sample. The numbers reported here are therefore the
  PHYSICAL energy rates of the airframe, not a bit-exact replica of the TECS
  internal state (which is only available in the dataflash TECS log message,
  copied out by the runner for validation).

  GRAVITY: energies from Gazebo ground truth use the WORLD value
  g = 9.81 m/s^2 (tests/gazebo/worlds/..._world.sdf:56, and CLAUDE.md).
  ArduPlane internally uses GRAVITY_MSS = 9.80665 (AP_Math/definitions.h:45).
  The 0.034 % difference is recorded, not corrected away.

  AIRSPEED: TECS works in TAS. The primary V here is the aerodynamics plugin's
  own true airspeed (Gazebo ground truth; this is a ZERO-WIND run so TAS is
  unambiguous). VFR_HUD.airspeed (EAS, via the official SIM_JSON pitot path
  with ARSPD_USE=1) is recorded alongside for every window.

SIGN CONVENTIONS - TEST-VERIFIED, NEVER ASSUMED (controls-integration rule)
  * FBWB pitch stick: RC2 > RC2_TRIM is EXPECTED to command climb
    (FBWB_ELEV_REV read live, expected 0). `fbwb_up_stick_climbs` and
    `fbwb_down_stick_descends` are GATING checks on the MEASURED altitude rate.
  * Physical pitch is nose-up-positive from Gazebo ground truth; gz Euler pitch
    is nose-DOWN-positive in this FLU world, so pitch_phys = -(gz euler pitch).
  * ArduPlane's physically demanded pitch = NAV_CONTROLLER_OUTPUT.nav_pitch
    + PTCH_TRIM_DEG [Attitude.cpp:244]. Raw nav_pitch and the trim-corrected
    demand are BOTH recorded, per phase and per sample, so the convention can
    never be double counted silently.

SCOPE / HARD CONSTRAINTS OBSERVED
  * NO TECS_* parameter is set, anywhere, by this stage. TECS runs on the
    ArduPlane compiled firmware defaults; the live effective values are dumped
    into the result JSON and gated against the documented baseline.
  * NO PID, PTCH_TRIM_DEG, control-surface +/-45 deg mapping, aero, actuator,
    propulsion, sensor, mass/CG/inertia or SDF change. falcon_v2_sitl.parm is
    READ-ONLY input.
  * Nothing is tuned to make this pass. The test is written to FAIL honestly.

OPEN LIMITATION CARRIED INTO THIS STAGE (not a blocker, reported every run)
  PROPULSION_HIGH_J_WINDMILLING: the APC 13x6.5E Ct/Cp table stops at the
  zero-thrust advance ratio (~J 0.64). At low throttle in a descent the model
  clamps Ct to the table end (thrust floored at 0 N) where a real fixed-pitch
  prop would windmill and produce NEGATIVE thrust. The simulated descent is
  therefore slightly LESS draggy than reality. The test COUNTS and REPORTS the
  affected samples (propulsion diagnostic `interpClamped`, plus J statistics)
  per phase. Consequence: the DIRECTION, controllability, settling and energy
  bookkeeping of the descent are valid; its ABSOLUTE drag/sink performance is
  NOT high-fidelity real-aircraft truth. Owner: `propulsion`. DATA_REQUIRED.

USAGE (a FRESH gz sim + gdb-wrapped arduplane pair MUST already be running -
see tests/gazebo/scripts/run_ardupilot_tecs_climb_descent_energy.sh):
    python3 test_ardupilot_tecs_climb_descent_energy.py
    python3 test_ardupilot_tecs_climb_descent_energy.py --reanalyze <timeseries.json>
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import gz.transport13 as tp  # noqa: E402
from gz.msgs10 import entity_wrench_pb2, entity_pb2  # noqa: E402

import test_ardupilot_basic_closed_loop_flight as base  # noqa: E402
import test_ardupilot_basic_closed_loop_flight_campaign as campaign  # noqa: E402
import test_ardupilot_fbwa_level_pitch_reference_correction as fbwa  # noqa: E402
# The 2026-09-02 FBWB/TECS harness, imported (NOT copied): mode entry, the
# live-param-derived command inversions, the segment runner, the per-sample
# accessors and the window analysis all come from here.
import test_ardupilot_tecs_cruise_speed_hold as cruise  # noqa: E402
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

STAGE = "ARDUPLANE_TECS_CLIMB_DESCENT_AND_ENERGY_VALIDATION"
OUT_JSON = os.path.join(base.RESULTS_DIR, "ardupilot_tecs_climb_descent_energy_result.json")
OUT_TS = os.path.join(base.RESULTS_DIR, "ardupilot_tecs_climb_descent_energy_timeseries.json")
OUT_TRACE = os.path.join(base.RESULTS_DIR, "ardupilot_tecs_climb_descent_energy_per_sample.json")

ARDUPLANE_FBWB_CUSTOM_MODE = cruise.ARDUPLANE_FBWB_CUSTOM_MODE   # 6

# =============================================================================
# READ-ONLY CITATIONS (never modified, never fed back into any physics path)
# =============================================================================
MASS_KG = cruise.MASS_KG            # 6.000  CLAUDE.md
S_REF_M2 = cruise.S_REF_M2          # 0.4514 CLAUDE.md
G_WORLD = 9.81                      # world <gravity>, CLAUDE.md
G_TECS = 9.80665                    # AP_Math/definitions.h:45 GRAVITY_MSS
V_TRIM_REF = cruise.V_TRIM_REF              # 18.162 m/s  measured Gazebo trim
TRIM_THROTTLE_REF = cruise.TRIM_THROTTLE_REF  # 0.4957
ELEV_TRIM_DEG_REF = cruise.ELEV_TRIM_DEG_REF  # +4.092 deg
PTCH_TRIM_DEG_EXPECTED = cruise.PTCH_TRIM_DEG_EXPECTED  # 2.49
V_TARGET_MS = cruise.V_TARGET_MS    # 18.0 = AIRSPEED_CRUISE
ALT_STEP_M = cruise.ALT_STEP_M      # 10.0
SURFACE_TRAVEL_LIMIT_DEG = 45.0     # ARDUPLANE_CONTROL_SURFACE_TRAVEL_SCALING
                                    # stage; docs/source_of_truth/controls/CONTROLS.md

# Measured reference results of the 2026-09-02 cruise stage, cited so this
# stage's thresholds are anchored to real Falcon V2 data instead of guesses.
# Source: docs/test_results/2026-09-02_ardupilot_tecs_and_cruise_speed_hold_
# validation.md sections 3-9. These are REFERENCE VALUES ONLY - nothing here is
# fed into a control path, and no check requires reproducing them exactly.
PRIOR = dict(
    cruise_airspeed_mean_ms=17.926, cruise_airspeed_std_ms=0.187,
    hold_vz_max_abs_ms=0.00829, hold_alt_p2p_max_m=2.179,
    achieved_climb_m=10.239, achieved_descent_m=-10.314,
    ramp_vz_ms=1.301, ramp_peak_vz_ms=1.951,
    roundtrip_alt_residual_m=-0.074,
    level_throttle=0.4911, climb_throttle=0.5398, descent_throttle=0.4251,
    level_pitch_deg=2.663, climb_pitch_deg=6.683, descent_pitch_deg=-1.253,
    climb_specific_energy_rate_W_per_kg=10.37,
    descent_specific_energy_rate_W_per_kg=-9.60,
    level_specific_energy_rate_W_per_kg=0.05,
    whole_flight_airspeed_min_ms=16.780, whole_flight_airspeed_max_ms=19.342,
    whole_flight_elevator_max_abs_deg=6.368,
    decay_ratio_range=[0.405, 0.653],
)

# =============================================================================
# PHASE PLAN + DURATION RATIONALE
# -----------------------------------------------------------------------------
# Longitudinal time constants of THIS airframe / THIS controller (identical
# derivation to the 2026-09-02 stage, so the two are directly comparable):
#   phugoid period    T_ph ~ pi*sqrt(2)*V/g = 4.443*18.162/9.81 = 8.2 s
#                     (FBWA stage independently observed ~9 s, zeta ~ 0.2)
#   TECS_TIME_CONST   5.0 s   (firmware default, AP_TECS.cpp:43)
#   TECS_HDEM_TCONST  3.0 s   (firmware default, AP_TECS.cpp:292)
#   THR_SLEWRATE      100 %/s -> <= 1.0 s for a full-range throttle move
#
# P1_CRUISE 45 s / 12 s transient  -> INHERITED UNCHANGED from the cruise stage
#     (SEG_A_DURATION_S / SEG_A_TRANSIENT_S). 33 s analysed = 4 phugoid
#     periods = 6.6 x TECS_TIME_CONST. Also covers FBWB entry and the
#     throttle-unsuppression / TECS-filter initialisation transient.
# P2_CLIMB   cap 20 s  -> INHERITED (SEG_B_RAMP_MAX_S). At FBWB_CLIMB_RATE
#     2.0 m/s the +10 m ramp needs ~5 s of demand and took 8.67 s in the prior
#     run; the cap only bounds a failure to achieve the demand.
# P3_SETTLE  40 s / 10 s transient -> LENGTHENED from the prior 35 s. 30 s
#     analysed = 6 x TECS_TIME_CONST = 3.7 phugoid periods. The extra 5 s is
#     required because this stage measures a SETTLING TIME (which must be
#     contained inside the segment; the acceptance limit is DERIVED PER PHASE
#     from the mode - see the TH_SETTLE_TAU_MARGIN_K block - and the analysed
#     30 s is the observable window against which that derived limit is
#     checked for measurability) AND still needs a steady tail to define the
#     settled value from. The segment DURATIONS are unchanged by the V-1
#     correction; only the acceptance criterion changed.
#     The 10 s transient cutoff (2 x TECS_TIME_CONST) is inherited
#     (HOLD_TRANSIENT_S).
# P4_DESCENT cap 20 s  -> mirror of P2 (SEG_C_RAMP_MAX_S).
# P5_RESETTLE 40 s / 10 s transient -> LENGTHENED from the prior 30 s, for the
#     same reason as P3. Unlike the prior stage this window carries a GATING
#     "returned to the original altitude" criterion, so it must be long enough
#     to distinguish a settled recapture from a slow drift.
# Total flight time <= 45+20+40+20+40 = 165 s.
# =============================================================================
P1_CRUISE_S = cruise.SEG_A_DURATION_S          # 45.0
P1_TRANSIENT_S = cruise.SEG_A_TRANSIENT_S      # 12.0
P2_CLIMB_MAX_S = cruise.SEG_B_RAMP_MAX_S       # 20.0
P3_SETTLE_S = 40.0
P4_DESCENT_MAX_S = cruise.SEG_C_RAMP_MAX_S     # 20.0
P5_RESETTLE_S = 40.0
HOLD_TRANSIENT_S = cruise.HOLD_TRANSIENT_S     # 10.0
SETTLE_TAIL_S = 10.0        # tail used to DEFINE the settled altitude/airspeed
                            # (2 x TECS_TIME_CONST, > 1 phugoid period)

PHASES = ["P1_cruise", "P2_climb", "P3_settle", "P4_descent", "P5_resettle"]
HOLD_PHASES = ["P1_cruise", "P3_settle", "P5_resettle"]
RAMP_PHASES = ["P2_climb", "P4_descent"]

# Ramp stop debounce: 3 consecutive samples (0.15 s at 20 Hz) past the
# threshold, so a single-sample spike cannot terminate a ramp early. The ramps
# run at ~1.3 m/s so a genuine crossing lasts far longer than 0.15 s.
RAMP_STOP_CONSECUTIVE = 3

# =============================================================================
# ACCEPTANCE THRESHOLDS
# Every value is either INHERITED from the 2026-09-02 stage (cited to the
# imported symbol, so the two stages cannot silently diverge) or DERIVED here
# from an inherited value / a documented physical quantity. Nothing is invented
# to make a result pass.
# =============================================================================
# ---- inherited verbatim from the cruise stage -------------------------------
TH_SPEED_MEAN_TOL_MS = cruise.TH_SPEED_MEAN_TOL_MS            # 0.5
TH_SPEED_STD_MAX_MS = cruise.TH_SPEED_STD_MAX_MS              # 0.5
TH_SPEED_MIN_MS = cruise.TH_SPEED_MIN_MS                      # 16.0 = AIRSPEED_MIN
TH_SPEED_HARD_FLOOR_MS = cruise.TH_SPEED_HARD_FLOOR_MS        # 14.4 = 0.9*TASmin
TH_SPEED_SLOPE_MAX_MS_PER_S = cruise.TH_SPEED_SLOPE_MAX_MS_PER_S  # 0.02
TH_TECS_TARGET_TOL_MS = cruise.TH_TECS_TARGET_TOL_MS          # 0.4
TH_ALT_SLOPE_MAX_MS = cruise.TH_ALT_SLOPE_MAX_MS              # 0.10
TH_ALT_SLOPE_TIGHT_MS = cruise.TH_ALT_SLOPE_TIGHT_MS          # 0.02 (preferred)
TH_ALT_P2P_MAX_M = cruise.TH_ALT_P2P_MAX_M                    # 5.0
TH_SINK_CLOSED_MS = cruise.TH_SINK_CLOSED_MS                  # 0.078 (FBWA residual)
TH_THROTTLE_TOL = cruise.TH_THROTTLE_TOL                      # 0.05
TH_SAT_RUN_MAX_S = cruise.TH_SAT_RUN_MAX_S                    # 2.0
TH_SAT_MARGIN = cruise.TH_SAT_MARGIN                          # 0.01
TH_TECS_AUTHORITY_MIN_DELTA = cruise.TH_TECS_AUTHORITY_MIN_DELTA  # 0.10
TH_THROTTLE_MODULATION_MIN = cruise.TH_THROTTLE_MODULATION_MIN     # 0.05
TH_SURF_HOLD_MAX_DEG = cruise.TH_SURF_HOLD_MAX_DEG            # 10.0
TH_SURF_FLIGHT_MAX_DEG = cruise.TH_SURF_FLIGHT_MAX_DEG        # 15.0
TH_LATERAL_SURF_MAX_DEG = cruise.TH_LATERAL_SURF_MAX_DEG      # 10.0
TH_COORD_THROTTLE_DELTA = cruise.TH_COORD_THROTTLE_DELTA      # 0.01
TH_COORD_PITCH_DELTA_DEG = cruise.TH_COORD_PITCH_DELTA_DEG    # 0.5
TH_PITCH_ALPHA_GAMMA_RESID_DEG = cruise.TH_PITCH_ALPHA_GAMMA_RESID_DEG  # 1.5
TH_GS_VS_AS_MAX_MS = cruise.TH_GS_VS_AS_MAX_MS                # 1.0
TH_RAMP_DIRECTION_MIN_MS = cruise.TH_RAMP_DIRECTION_MIN_MS    # 0.2
TH_ALT_STEP_ACHIEVED_FRAC = cruise.TH_ALT_STEP_ACHIEVED_FRAC  # 0.7

# ---- DERIVED HERE (each from an inherited value or a physical quantity) -----
# Target-altitude step verification. The two-sided form of the inherited
# 70 %-of-step criterion: |achieved - 10| <= (1-0.7)*10 = 3.0 m. Physical
# error sources are far smaller: the FBWB integrator granularity is
# FBWB_CLIMB_RATE * dt_max = 2.0*0.15 = 0.30 m per update, and the altitude
# travelled between the stop trigger and the stick-release lock is at most
# peak_vz * (RC refresh 0.1 s + FBWB check period 0.1 s) = 1.95*0.2 = 0.39 m.
TH_TARGET_STEP_TOL_M = round((1.0 - TH_ALT_STEP_ACHIEVED_FRAC) * ALT_STEP_M, 6)  # 3.0
# ArduPlane's own demand and the achieved altitude must agree: a 2 m gap is
# 20 % of the step and 4 x the settled hold-window half-amplitude
# (prior p2p 2.179 m). Larger means the aircraft is not tracking its demand.
TH_TARGET_VS_ACTUAL_TOL_M = 2.0
# Return-to-origin. Gating 2.0 m (same reasoning as above; the prior stage
# measured a -0.074 m round-trip residual, so this is ~27x the demonstrated
# performance and only fails on a genuine failure to recapture).
TH_RESETTLE_TOL_M = 2.0
TH_RESETTLE_TIGHT_M = 0.5        # preferred, non-gating
# Level-flight total-energy rate. Derived: g x the inherited 0.10 m/s
# altitude-hold slope limit = 0.981 W/kg -> 1.0 W/kg. (Prior measured
# 0.03-0.07 W/kg in the three hold windows.)
TH_LEVEL_STEDOT_MAX_W_PER_KG = 1.0
# Climb/descent total-energy rate must be unambiguously signed. 2.0 W/kg
# corresponds to g*0.204 m/s, i.e. 2 x the altitude-hold slope limit, so it
# cannot be confused with hold-window noise. (Prior: +10.37 / -9.60 W/kg.)
TH_RAMP_STEDOT_MIN_W_PER_KG = 2.0
# "The energy went where TECS says it should": on a speed-holding climb or
# descent, essentially all of the total-energy rate must appear as POTENTIAL
# energy rate, not kinetic. Bound: holding airspeed to the inherited 0.5 m/s
# band over the ~8.7 s ramp gives |SKEdot| <= 18*0.5/8.7 = 1.03 W/kg against
# |SPEdot| ~ g*1.3 = 12.8 W/kg, a ratio of 0.08. 0.25 is 3x that, so it is a
# genuine bound on speed bleed/runaway rather than a repeat of the speed test.
TH_KINETIC_FRACTION_MAX = 0.25
# Round-trip energy closure: the 2.0 m re-settle tolerance expressed as
# specific potential energy, g*2.0 = 19.6 -> 20 J/kg.
TH_STE_ROUNDTRIP_MAX_J_PER_KG = 20.0
# No airspeed runaway in the descent. 2.0 m/s above the TECS demand equals the
# whole AIRSPEED_MIN->AIRSPEED_CRUISE span (16->18) and is 11 % of cruise;
# the prior run peaked at +1.42 m/s over demand, so this bounds the observed
# behaviour without being tuned to it.
TH_DESCENT_SPEED_OVERSHOOT_MAX_MS = 2.0
# Settling. Band 1.5 m: above the prior settled hold-window half-amplitude
# (p2p 2.179 m -> 1.09 m) and 15 % of the commanded step, so "settled" means
# the transient is gone, not that the phugoid has vanished entirely.
TH_SETTLE_BAND_M = 1.5
TH_SETTLE_SPEED_BAND_MS = TH_SPEED_MEAN_TOL_MS      # 0.5, inherited
# ---------------------------------------------------------------------------
# SETTLING TIME. There is NO fixed settling-time threshold any more.
#
# CORRECTION 2026-09-03, cause: validation finding V-1
#   (docs/validation/2026-09-03_ardupilot_tecs_climb_descent_energy_validation.md).
# The previous bound was TH_SETTLE_TIME_MAX_S = 5 x TECS_TIME_CONST = 25.0 s.
# That is a category error: TECS_TIME_CONST parameterises the FIRST-order
# demand-tracking loops (AP_TECS.cpp:735, :760) and cannot bound the decay
# envelope of the lightly damped SECOND-order longitudinal energy mode that
# actually settles here. The correct settling time of that mode,
#   t = t_peak + ln(A0/B)/(zeta*omega_n),
# contains no TECS_TIME_CONST term and DOES contain the initial excursion A0 -
# which is set by the FBWB command lag at stick release, not by the aircraft.
# A fixed time limit therefore gates the size of the disturbance as much as
# the controller.
#
# The corrected criterion is AMPLITUDE-NORMALISED and derived per phase from
# that phase's own measured trim state (see settling_analysis() and
# phugoid_reference()):
#
#   TAU_REF = V * (L/D) / g          free-airframe phugoid envelope time
#                                    constant (Lanchester: omega_n=sqrt2*g/V,
#                                    zeta = CD/(sqrt2*CL) = 1/(sqrt2*L/D),
#                                    so 1/(zeta*omega_n) = V*(L/D)/g)
#   t_limit_c = t_peak_c + T/2 + K * TAU_REF * ln(A0_c / B_c)
#
# per channel c (altitude, airspeed), where A0_c and t_peak_c are the measured
# peak excursion and its time, T is the measured damped period of the mode
# (the "last exit from the band" statistic can only land on a peak, so it is
# quantised on a T/2 grid) and B_c is the unchanged settling band.
# Equivalently the gate is  tau_implied_c <= K * TAU_REF  with
# tau_implied_c = (t_settle_c - t_peak_c - T/2)/ln(A0_c/B_c), i.e. the
# amplitude-normalised envelope time constant.
#
# ENGINEERING MEANING: with TECS engaged, a longitudinal energy transient must
# decay at least as fast as the SAME airframe's own uncontrolled phugoid
# envelope. A closed-loop energy controller that decays energy transients more
# slowly than no controller at all is not managing energy acceptably.
#
# K = 1.0 - NO margin, fixed before the corrected metric was ever evaluated.
# Any K > 1 would need a Falcon V2 handling-qualities basis (an acceptable
# damping ratio or settling time). None exists in docs/source_of_truth/ ->
# DATA_REQUIRED. Inventing a K would reintroduce exactly the magic number this
# correction removes.
# ---------------------------------------------------------------------------
TH_SETTLE_TAU_MARGIN_K = 1.0
# Decay. The prior stage only required "not growing" (ratio < 1.3). This stage
# requires ACTUAL decay. Prior measured second/first-half detrended-residual
# ratios of 0.405-0.653 across all channels and windows; 0.90 sits above that
# whole measured range yet still fails a sustained limit cycle (ratio ~1.0).
TH_DECAY_RATIO_MAX = 0.90
# Control-surface travel margin: never within 5 deg of the +/-45 deg
# mechanical limit (ARDUPLANE_CONTROL_SURFACE_TRAVEL_SCALING stage). 5 deg is
# ~11 % of travel and ~1 deg less than the entire deflection the prior flight
# ever used (6.368 deg), so this can only trip on a real authority problem.
TH_SURF_LIMIT_MARGIN_DEG = 5.0
TH_SURF_MAX_ABS_DEG = SURFACE_TRAVEL_LIMIT_DEG - TH_SURF_LIMIT_MARGIN_DEG   # 40.0
# Throttle must never be pinned. THR_MIN/THR_MAX are read live; TH_SAT_MARGIN
# (0.01, inherited) defines "at the limit".
# Preferred, NON-GATING: propulsive excess power (T-D)*V/m should match STEdot.
# Non-gating because D is a plugin diagnostic (qbar*S*CD) and because the
# descent is affected by PROPULSION_HIGH_J_WINDMILLING (thrust floored at 0 N),
# which biases exactly this comparison. 3.0 W/kg = 30 % of the observed ramp
# energy rate.
TH_PROPULSIVE_POWER_RESID_W_PER_KG = 3.0

# Extra parameter this stage needs on top of the inherited dump: ALT_OFFSET
# must be 0 for the ArduPlane target-altitude readback derived in the module
# docstring to be exact (adjusted_altitude_cm() = current_loc.alt -
# ALT_OFFSET*100). Firmware default 0, ArduPlane/Parameters.cpp:236.
EXTRA_PARAMS = ["ALT_OFFSET"]
PARAMS_OF_INTEREST = list(dict.fromkeys(cruise.PARAMS_OF_INTEREST + EXTRA_PARAMS))


# =============================================================================
# live parameter acquisition (same pattern as the cruise stage; the only
# difference is the extended PARAMS_OF_INTEREST list)
# =============================================================================
def dump_params(mav, R):
    got = {}
    try:
        bulk = mav.fetch_all_params(timeout=45, idle_cutoff=3.0)
    except Exception as exc:          # noqa: BLE001 - reported, not swallowed
        bulk = {}
        R["param_bulk_error"] = str(exc)
    for name in PARAMS_OF_INTEREST:
        v = bulk.get(name)
        if v is None:
            v = read_param(mav, name)
        got[name] = v
    missing = sorted(k for k, v in got.items() if v is None)
    R["tecs_baseline_params_live"] = got
    R["tecs_baseline_params_missing"] = missing
    R["param_bulk_count"] = len(bulk)
    R["tecs_baseline_params_provenance"] = (
        "config/ardupilot/falcon_v2_sitl.parm sets NO TECS_* value and arduplane "
        "is launched with -w (wiped EEPROM), so every TECS_* value above is the "
        "ArduPlane 4.8.0-dev COMPILED FIRMWARE DEFAULT. Expected values + source "
        "line numbers: docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml. "
        "THIS STAGE WROTE NO PARAMETER.")
    if missing:
        print("WARNING: params not read:", missing)
    return got


def param_precondition_checks(p, R):
    """The inherited cruise-stage preconditions, PLUS the three this stage's
    own derivations depend on. Any failure is surfaced, never worked around."""
    chk = dict(cruise.param_precondition_checks(p, R))

    def eq(name, val):
        return p.get(name) is not None and abs(p[name] - val) < 1e-6

    # ALT_OFFSET must be 0 for ap_target_alt_rel_m to be exact
    # (adjusted_altitude_cm() = current_loc.alt - ALT_OFFSET*100).
    chk["alt_offset_zero_for_target_readback"] = eq("ALT_OFFSET", 0)
    # TERRAIN_FOLLOW must be 0 so calc_altitude_error_cm() takes the AMSL
    # branch (ArduPlane/altitude.cpp:389-398).
    chk["terrain_follow_zero"] = eq("TERRAIN_FOLLOW", 0)
    # The target-altitude step verification is only meaningful if the ramp rate
    # is the documented firmware default (ArduPlane/Parameters.cpp:345).
    chk["fbwb_climb_rate_2ms"] = eq("FBWB_CLIMB_RATE", 2.0)
    # SEB weighting derivation assumes TECS_SPDWEIGHT is readable.
    chk["tecs_spdweight_readable"] = p.get("TECS_SPDWEIGHT") is not None
    R["param_preconditions"] = chk
    print("param preconditions (extended):", json.dumps(chk, default=str))
    return chk


# =============================================================================
# per-sample derived quantities (energy + target altitude)
# =============================================================================
s_alt = cruise.s_alt
s_pitch_phys = cruise.s_pitch_phys
s_pitch_demand_phys = cruise.s_pitch_demand_phys
s_tecs_target_airspeed = cruise.s_tecs_target_airspeed
s_throttle_actual = cruise.s_throttle_actual
s_surface_deg = cruise.s_surface_deg
collect = cruise.collect
longest_run_seconds = cruise.longest_run_seconds


def s_ap_target_alt_rel_m(s):
    """ArduPlane's OWN height demand, above home, reconstructed live:
        GLOBAL_POSITION_INT.relative_alt/1000 + NAV_CONTROLLER_OUTPUT.alt_error
    Derivation + source lines: see module docstring 'TARGET ALTITUDE READBACK'.
    Valid only while ALT_OFFSET == 0 and TERRAIN_FOLLOW == 0 (both gated as
    preconditions)."""
    ra = s["mav"]["relative_alt_m"]
    ae = s["mav"]["nav_alt_error"]
    if ra is None or ae is None:
        return None
    return ra + ae


def s_ap_alt_rel_m(s):
    return s["mav"]["relative_alt_m"]


def s_tas(s):
    """True airspeed, Gazebo ground truth from the aerodynamics plugin
    diagnostics. This is a ZERO-WIND run, so TAS is unambiguous."""
    return s["aero"]["V"] if s["aero"] else None


def s_spe(s):
    """specific potential energy, J/kg. AP_TECS.cpp:689 (_SPE_est = h*g)."""
    z = s_alt(s)
    return (G_WORLD * z) if z is not None else None


def s_ske(s):
    """specific kinetic energy, J/kg. AP_TECS.cpp:690 (0.5*TAS^2)."""
    v = s_tas(s)
    return (0.5 * v * v) if v is not None else None


def s_ste(s):
    a, b = s_spe(s), s_ske(s)
    return (a + b) if (a is not None and b is not None) else None


def seb_weights(p):
    """AP_TECS.cpp:1003 _SKE_weighting = constrain(TECS_SPDWEIGHT, 0, 2);
    :1024 SPE_weighting = 2 - _SKE_weighting; :1027-1028 both MIN(...,1).
    Note: this is the NORMAL-flight branch. The landing/approach branches
    (:1005-1020) cannot apply here - no landing/approach stage is ever entered
    in FBWB."""
    w = p.get("TECS_SPDWEIGHT")
    if w is None:
        return None, None
    w_ske = min(max(float(w), 0.0), 2.0)
    w_spe = 2.0 - w_ske
    return min(w_spe, 1.0), min(w_ske, 1.0)


def s_seb(s, w_spe, w_ske):
    """specific energy BALANCE, J/kg. AP_TECS.cpp:1031-1032."""
    a, b = s_spe(s), s_ske(s)
    if a is None or b is None or w_spe is None:
        return None
    return a * w_spe - b * w_ske


def s_elev_deg(s):
    """mean of the two elevator surfaces (they track to <0.001 deg)."""
    vals = [s_surface_deg(s, n) for n in ("left_elevator", "right_elevator")]
    vals = [v for v in vals if v is not None]
    return mean(vals) if vals else None


# =============================================================================
# energy block for one analysis window
# =============================================================================
def energy_block(samples, p):
    """Phase-resolved energy management. All rates are computed from the RAW
    kinematics over the window (see FILTER NOTE in the module docstring); they
    are the airframe's physical energy rates, not a replica of the TECS
    internal filtered state."""
    out = {}
    w_spe, w_ske = seb_weights(p)
    out["seb_weights"] = dict(w_SPE=w_spe, w_SKE=w_ske,
                              tecs_spdweight_live=p.get("TECS_SPDWEIGHT"),
                              source="AP_TECS.cpp:1003,1024,1027-1028")
    out["gravity_used_m_s2"] = G_WORLD
    out["gravity_ardupilot_internal_m_s2"] = G_TECS

    ts_z, zs = collect(samples, s_alt)
    ts_v, vs = collect(samples, s_tas)
    _, spe = collect(samples, s_spe)
    _, ske = collect(samples, s_ske)
    ts_e, ste = collect(samples, s_ste)
    _, seb = collect(samples, lambda s: s_seb(s, w_spe, w_ske))
    _, climb = collect(samples, lambda s: s["mav"]["climb"])

    out["SPE_J_per_kg"] = minmaxmean(spe)
    out["SKE_J_per_kg"] = minmaxmean(ske)
    out["STE_J_per_kg"] = minmaxmean(ste)
    out["SEB_J_per_kg"] = minmaxmean(seb)
    out["TAS_ms"] = minmaxmean(vs)

    if len(zs) < 2 or len(vs) < 2 or len(ste) < 2:
        out["insufficient_samples"] = True
        return out

    vz_reg, _ = linreg(ts_z, zs)              # gz ground-truth vertical speed
    dvdt, _ = linreg(ts_v, vs)                # dTAS/dt over the window
    v_mean = mean(vs)
    spedot = G_WORLD * vz_reg
    skedot = v_mean * dvdt
    stedot = spedot + skedot
    sebdot = spedot * w_spe - skedot * w_ske

    out["vz_regression_ms"] = vz_reg
    out["vz_vfr_hud_mean_ms"] = mean(climb) if climb else None
    out["dTAS_dt_ms2"] = dvdt
    out["SPEdot_W_per_kg"] = spedot
    out["SKEdot_W_per_kg"] = skedot
    out["STEdot_W_per_kg"] = stedot
    out["SEBdot_W_per_kg"] = sebdot
    # Endpoint cross-check of STEdot: independent of the regression fit.
    dur = ts_e[-1] - ts_e[0]
    out["STEdot_endpoint_W_per_kg"] = ((ste[-1] - ste[0]) / dur) if dur > 0 else None
    out["STE_change_over_window_J_per_kg"] = ste[-1] - ste[0]

    # Division of labour: what fraction of the total energy rate went into
    # ALTITUDE rather than SPEED.
    if abs(stedot) > 1e-6:
        out["potential_fraction_of_STEdot"] = spedot / stedot
        out["kinetic_fraction_of_STEdot"] = skedot / stedot
    else:
        out["potential_fraction_of_STEdot"] = None
        out["kinetic_fraction_of_STEdot"] = None
    out["kinetic_over_potential_abs"] = (abs(skedot) / abs(spedot)
                                         if abs(spedot) > 1e-9 else None)

    # Propulsive excess power, REPORT-ONLY cross-check: (T - D)*V/m.
    _, thrust = collect(samples, lambda s: (s["propulsion"]["left"]["thrust_N"]
                                            + s["propulsion"]["right"]["thrust_N"])
                        if s["propulsion"] else None)
    _, drag = collect(samples, lambda s: (s["aero"]["qbar"] * S_REF_M2 * s["aero"]["CD"])
                      if s["aero"] else None)
    if thrust and drag:
        out["thrust_total_N_mean"] = mean(thrust)
        out["drag_N_mean"] = mean(drag)
        out["propulsive_excess_power_W_per_kg"] = (
            (mean(thrust) - mean(drag)) * v_mean / MASS_KG)
        out["propulsive_vs_STEdot_residual_W_per_kg"] = (
            out["propulsive_excess_power_W_per_kg"] - stedot)
        out["propulsive_note"] = (
            "REPORT-ONLY. D is the aero plugin's qbar*S*CD diagnostic and the "
            "descent is affected by PROPULSION_HIGH_J_WINDMILLING (thrust "
            "floored at 0 N above the APC table's last J), which biases exactly "
            "this comparison. Not a gating criterion.")
    return out


# =============================================================================
# PROPULSION_HIGH_J_WINDMILLING accounting (OPEN_LIMITATION, reported not gated)
# =============================================================================
def high_j_block(samples):
    out = {"limitation_id": "PROPULSION_HIGH_J_WINDMILLING",
           "status": "OPEN_LIMITATION",
           "owner": "propulsion",
           "detail": ("APC 13x6.5E Ct/Cp table has no windmilling data above the "
                      "zero-thrust advance ratio. Ct is clamped to the table end, "
                      "so thrust is floored at 0 N where a real fixed-pitch prop "
                      "would produce NEGATIVE thrust. The simulated descent is "
                      "therefore less draggy than reality. DATA_REQUIRED: measured "
                      "or extrapolated APC 13x6.5E Ct/Cp for J beyond zero-thrust."),
           "effect_on_this_stage": ("Descent DIRECTION, controllability, settling and "
                                    "energy bookkeeping remain valid. The ABSOLUTE "
                                    "descent drag/sink performance is NOT high-fidelity "
                                    "real-aircraft truth. NOT a blocker for this stage.")}
    n_motor = 0
    n_clamped = 0
    n_zero_thrust = 0
    js = []
    for s in samples:
        pr = s["propulsion"]
        if not pr:
            continue
        for side in ("left", "right"):
            d = pr[side]
            n_motor += 1
            if d.get("interpClamped"):
                n_clamped += 1
            if d.get("thrust_N") is not None and abs(d["thrust_N"]) < 1e-9:
                n_zero_thrust += 1
            if d.get("J") is not None and math.isfinite(d["J"]):
                js.append(d["J"])
    out["motor_samples"] = n_motor
    out["interp_clamped_samples"] = n_clamped
    out["interp_clamped_fraction"] = (n_clamped / n_motor) if n_motor else None
    out["zero_thrust_samples"] = n_zero_thrust
    out["advance_ratio_J"] = minmaxmean(js)
    return out


# =============================================================================
# settling / decay analysis (new this stage)
# =============================================================================
def _local_extrema(ts, ys):
    """indices of interior local maxima/minima."""
    out = []
    for i in range(1, len(ys) - 1):
        if (ys[i] - ys[i - 1]) * (ys[i + 1] - ys[i]) < 0:
            out.append(i)
    return out


def damping_estimate(ts, ys):
    """Log-decrement damping estimate on a detrended series. REPORT-ONLY:
    it characterises the phugoid, it is not an acceptance criterion."""
    if len(ys) < 12:
        return dict(n=len(ys), note="too few samples")
    slope, icpt = linreg(ts, ys)
    if slope is None:
        return dict(n=len(ys), note="degenerate")
    resid = [y - (slope * t + icpt) for t, y in zip(ts, ys)]
    idx = _local_extrema(ts, resid)
    if len(idx) < 3:
        return dict(n=len(ys), n_extrema=len(idx), note="fewer than 3 extrema")
    amps = [abs(resid[i]) for i in idx]
    times = [ts[i] for i in idx]
    # half-period = spacing between successive extrema
    half_periods = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    period = 2.0 * mean(half_periods) if half_periods else None
    ratios = [amps[i + 1] / amps[i] for i in range(len(amps) - 1)
              if amps[i] > 1e-9]
    if not ratios:
        return dict(n=len(ys), n_extrema=len(idx), period_s=period,
                    note="amplitudes at noise floor")
    r_half = mean(ratios)
    r_cycle = r_half * r_half
    zeta = None
    if 0.0 < r_cycle < 1.0:
        d = -math.log(r_cycle)          # logarithmic decrement, per cycle
        zeta = d / math.sqrt(4.0 * math.pi ** 2 + d * d)
    return dict(n=len(ys), n_extrema=len(idx), period_s=period,
                amplitude_ratio_per_half_cycle=r_half,
                amplitude_ratio_per_cycle=r_cycle,
                damping_ratio_zeta=zeta,
                first_amplitude=amps[0], last_amplitude=amps[-1],
                note="REPORT_ONLY log-decrement estimate; not an acceptance criterion")


def phugoid_reference(samples, label):
    """FREE-AIRFRAME (stick-fixed, throttle-fixed) longitudinal energy-mode
    reference for one phase, computed from THAT PHASE'S OWN measured trim
    state. This is the reference the settling criterion is gated against
    (validation finding V-1); it deliberately contains NO controller
    parameter and NO initial amplitude.

    Lanchester phugoid approximation:
        omega_n = sqrt(2) * g / V
        zeta    = CD / (sqrt(2) * CL) = 1 / (sqrt(2) * (L/D))
        =>  zeta*omega_n = g / (V * (L/D))
        =>  TAU_REF = 1/(zeta*omega_n) = V * (L/D) / g       [envelope, s]
            T_REF   = 2*pi/omega_n     = pi*sqrt(2) * V / g  [period, s]

    ASSUMPTION  PHUGOID_REFERENCE_IS_LANCHESTER: the Lanchester form neglects
    the thrust/airspeed dependence and short-period coupling, so it is an
    ESTIMATE of the free-airframe phugoid, not a measurement of it. It is used
    because it is the only free-airframe longitudinal energy-mode reference
    derivable from data this project actually holds. A measured Falcon V2
    stick-fixed phugoid (period + damping) is DATA_REQUIRED and would replace
    it directly. See docs/source_of_truth/controls/
    ardupilot_tecs_energy_management.yaml (settling_criterion_correction).

    V, CL and CD are the aero-plugin diagnostics averaged over the phase's
    POST-TRANSIENT window (t_seg >= HOLD_TRANSIENT_S), i.e. the settled trim
    the mode oscillates about. L/D is also cross-checked against the
    thrust-based level-flight identity L/D = m*g/T_total (REPORT ONLY: thrust
    is affected by PROPULSION_HIGH_J_WINDMILLING, so it never gates).
    """
    post = [s for s in samples if s["t_seg"] >= HOLD_TRANSIENT_S]
    vs, cls, cds, ths = [], [], [], []
    for s in post:
        a = s.get("aero")
        if a and all(a.get(k) is not None for k in ("V", "CL", "CD")):
            vs.append(a["V"])
            cls.append(a["CL"])
            cds.append(a["CD"])
        pr = s.get("propulsion")
        if pr:
            ths.append(pr["left"]["thrust_N"] + pr["right"]["thrust_N"])
    out = {"label": label, "window": f"t_seg >= {HOLD_TRANSIENT_S} s",
           "n_samples": len(vs),
           "model": "Lanchester phugoid: tau = V*(L/D)/g, T = pi*sqrt(2)*V/g",
           "assumption": "PHUGOID_REFERENCE_IS_LANCHESTER",
           "gravity_m_s2": G_WORLD}
    if len(vs) < 8:
        out["insufficient_samples"] = True
        return out
    V = mean(vs)
    CL = mean(cls)
    CD = mean(cds)
    if V <= 0.0 or CD <= 0.0 or CL <= 0.0:
        out["degenerate"] = True
        return out
    LD = CL / CD
    out["V_ms"] = V
    out["CL"] = CL
    out["CD"] = CD
    out["L_over_D_aero"] = LD
    out["zeta_free_airframe"] = 1.0 / (math.sqrt(2.0) * LD)
    out["omega_n_rad_s"] = math.sqrt(2.0) * G_WORLD / V
    out["tau_ref_s"] = V * LD / G_WORLD
    out["T_ref_s"] = math.pi * math.sqrt(2.0) * V / G_WORLD
    if ths:
        T_tot = mean(ths)
        out["thrust_N_mean"] = T_tot
        out["L_over_D_thrust_crosscheck"] = (
            (MASS_KG * G_WORLD / T_tot) if T_tot > 1e-6 else None)
        out["crosscheck_note"] = (
            "REPORT ONLY - level-flight identity L/D = m*g/T. Thrust is "
            "affected by PROPULSION_HIGH_J_WINDMILLING and never gates.")
    return out


def settling_analysis(samples, label):
    """Measured settling of ONE post-transient phase (the whole segment,
    starting at the stick release that ended the ramp).

    settled value  = mean over the last SETTLE_TAIL_S of the segment
    settling time  = the classic "last exit from the settling band": the time
                     of the LAST sample violating the band, so every later
                     sample is inside it. Reported THREE ways so a failure is
                     immediately attributable:
                       altitude-only  |altitude - altitude_settled| <= TH_SETTLE_BAND_M
                       airspeed-only  |airspeed - airspeed_settled| <= TH_SETTLE_SPEED_BAND_MS
                       combined       both simultaneously
                     TECS settles an ENERGY state, of which altitude and
                     airspeed are the two halves, so BOTH channels must settle.

    ACCEPTANCE (corrected 2026-09-03 for validation finding V-1; the previous
    fixed 25 s limit is removed - see the TH_SETTLE_TAU_MARGIN_K block):
    each channel is gated against its OWN amplitude-normalised limit

        t_limit_c = t_peak_c + T/2 + K * TAU_REF * ln(A0_c / B_c)

    with A0_c / t_peak_c the measured peak excursion and its time, T the
    measured damped period of this phase's mode (T/2 = the quantisation of the
    last-exit statistic, which can only land on a peak), TAU_REF = V*(L/D)/g
    the free-airframe phugoid envelope time constant from phugoid_reference()
    and K = TH_SETTLE_TAU_MARGIN_K = 1.0. Equivalently the gate is
    tau_implied_c <= K*TAU_REF, the amplitude-normalised form. The SAME
    criterion is applied to P3_settle and P5_resettle with no special-casing.

    decay          = second-half / first-half detrended residual spread over
                     the POST-TRANSIENT part of the segment, per channel.
    """
    out = {"label": label, "band_m": TH_SETTLE_BAND_M,
           "speed_band_ms": TH_SETTLE_SPEED_BAND_MS,
           "tail_s": SETTLE_TAIL_S,
           "settle_limit_basis": (
               "AMPLITUDE-NORMALISED, derived per phase: t_limit = t_peak + T/2 "
               "+ K*TAU_REF*ln(A0/B), TAU_REF = V*(L/D)/g (free-airframe "
               "Lanchester phugoid envelope), K = TH_SETTLE_TAU_MARGIN_K. "
               "Replaces the withdrawn fixed 25 s limit (validation V-1)."),
           "settle_tau_margin_k": TH_SETTLE_TAU_MARGIN_K}
    if len(samples) < 20:
        out["insufficient_samples"] = True
        return out
    t0 = samples[0]["t_seg"]
    rows = []
    for s in samples:
        z = s_alt(s)
        v = s["mav"]["airspeed"]
        if z is None or v is None:
            continue
        rows.append((s["t_seg"] - t0, z, v))
    if len(rows) < 20:
        out["insufficient_samples"] = True
        return out
    t_end = rows[-1][0]
    tail = [r for r in rows if r[0] >= t_end - SETTLE_TAIL_S]
    if len(tail) < 5:
        out["insufficient_tail"] = True
        return out
    z_set = mean([r[1] for r in tail])
    v_set = mean([r[2] for r in tail])
    out["altitude_settled_m"] = z_set
    out["airspeed_settled_ms"] = v_set

    # last exit from the band (three variants)
    t_alt = t_spd = t_both = 0.0
    for t, z, v in rows:
        bad_z = abs(z - z_set) > TH_SETTLE_BAND_M
        bad_v = abs(v - v_set) > TH_SETTLE_SPEED_BAND_MS
        if bad_z:
            t_alt = t
        if bad_v:
            t_spd = t
        if bad_z or bad_v:
            t_both = t
    # t_* is the time of the LAST out-of-band sample; settling occurs at the
    # following sample. If the LAST sample itself is out of band the phase
    # never settled.
    never = (rows[-1][0] - t_both) < 1e-9
    never_alt = (rows[-1][0] - t_alt) < 1e-9
    never_spd = (rows[-1][0] - t_spd) < 1e-9
    out["settling_time_altitude_s"] = None if never_alt else t_alt
    out["settling_time_airspeed_s"] = None if never_spd else t_spd
    out["settling_time_s"] = None if never else t_both   # combined, reported
    out["settling_binding_channel"] = (
        None if never else ("airspeed" if t_spd >= t_alt else "altitude"))
    out["settled_within_segment"] = not never

    # peak excursion AND ITS TIME, per channel (the envelope decay clock starts
    # at the peak, not at the segment start: before the peak the aircraft is
    # still capturing the new datum and the mode is not yet decaying)
    a0_z = a0_v = -1.0
    tp_z = tp_v = 0.0
    for t, z, v in rows:
        if abs(z - z_set) > a0_z:
            a0_z, tp_z = abs(z - z_set), t
        if abs(v - v_set) > a0_v:
            a0_v, tp_v = abs(v - v_set), t
    out["initial_altitude_excursion_m"] = a0_z
    out["initial_airspeed_excursion_ms"] = a0_v
    out["initial_altitude_excursion_time_s"] = tp_z
    out["initial_airspeed_excursion_time_s"] = tp_v

    # decay, measured over the post-transient part only (the transient itself
    # is a commanded step response, not an oscillation amplitude)
    post = [s for s in samples if s["t_seg"] >= HOLD_TRANSIENT_S]
    ch = {}
    for name, fn in (("altitude", s_alt),
                     ("airspeed", lambda s: s["mav"]["airspeed"]),
                     ("pitch_physical", s_pitch_phys),
                     ("throttle", s_throttle_actual)):
        ts, ys = collect(post, fn)
        ch[name] = detrended_growth(ts, ys) if len(ys) >= 8 else None
    out["decay"] = ch
    ratios = {k: (v.get("ratio_second_over_first") if v else None) for k, v in ch.items()}
    out["decay_ratios"] = ratios
    usable = [v for v in ratios.values() if v is not None]
    out["decay_ratio_max"] = max(usable) if usable else None
    out["all_channels_decaying"] = bool(usable) and all(
        v <= TH_DECAY_RATIO_MAX for v in usable)
    out["decay_ratio_threshold"] = TH_DECAY_RATIO_MAX
    # measured mode characterisation (period + log-decrement damping)
    ts_z, zs = collect(post, s_alt)
    ph_meas = damping_estimate(ts_z, zs)
    out["phugoid_estimate_altitude"] = ph_meas

    # ---- amplitude-normalised settling acceptance (validation V-1) ---------
    ref = phugoid_reference(samples, label)
    out["phugoid_reference_free_airframe"] = ref
    tau_ref = ref.get("tau_ref_s")
    T_meas = ph_meas.get("period_s") if isinstance(ph_meas, dict) else None
    if T_meas is not None and T_meas > 0.0:
        T_used, T_src = T_meas, "measured closed-loop period (log-decrement extrema spacing)"
    else:
        T_used, T_src = ref.get("T_ref_s"), "fallback: Lanchester T_ref = pi*sqrt(2)*V/g"
    out["settle_period_used_s"] = T_used
    out["settle_period_source"] = T_src
    out["tau_ref_s"] = tau_ref

    lim = {}
    all_ok = True
    for cname, a0, tp, band, t_set, never_c in (
            ("altitude", a0_z, tp_z, TH_SETTLE_BAND_M, t_alt, never_alt),
            ("airspeed", a0_v, tp_v, TH_SETTLE_SPEED_BAND_MS, t_spd, never_spd)):
        d = {"A0": a0, "t_peak_s": tp, "band": band,
             "settling_time_s": (None if never_c else t_set)}
        if tau_ref is None or T_used is None:
            d["limit_s"] = None
            d["ok"] = False
            d["note"] = "no phugoid reference available - cannot evaluate"
            all_ok = False
            lim[cname] = d
            continue
        if a0 <= band:
            # never left the band at all
            d["limit_s"] = 0.0
            d["tau_implied_s"] = None
            d["ok"] = (not never_c) and t_set <= 1e-9
            d["note"] = "peak excursion never exceeded the band"
        else:
            ln_ratio = math.log(a0 / band)
            d["ln_A0_over_B"] = ln_ratio
            d["limit_s"] = tp + 0.5 * T_used + TH_SETTLE_TAU_MARGIN_K * tau_ref * ln_ratio
            d["tau_implied_s"] = (
                None if never_c else (t_set - tp - 0.5 * T_used) / ln_ratio)
            d["tau_limit_s"] = TH_SETTLE_TAU_MARGIN_K * tau_ref
            d["ok"] = (not never_c) and (t_set <= d["limit_s"])
            d["margin_s"] = None if never_c else (d["limit_s"] - t_set)
            if d["tau_implied_s"] is not None and d["tau_implied_s"] < 0.0:
                d["tau_implied_note"] = (
                    "NEGATIVE = the band was re-entered within half a period of "
                    "the peak, i.e. faster than the quantisation allowance of the "
                    "last-exit statistic. The normalised envelope constant is not "
                    "resolvable here; the channel trivially satisfies the limit.")
        # NON-GATING QUALITY-OF-REFERENCE flag (wording corrected 2026-09-04
        # for validation finding V-10). It does NOT mean the limit is
        # unfalsifiable: `never_c` is set only when the LAST sample of the
        # segment is out of band, so a last exit later than `obs` is still
        # measured and still compared against the limit. What it means is that
        # a settling time beyond `obs` would overlap the SETTLE_TAIL_S window
        # from which the settled reference is defined, so that reference - and
        # hence A0, the derived limit and the last-exit time - would be taken
        # from a not-yet-settled state. Contamination is in the PERMISSIVE
        # direction (inflated A0 -> wider limit). See
        # docs/source_of_truth/controls/ardupilot_tecs_energy_management.yaml
        # settling_criterion_correction.limitation_falsifiability.
        obs = P3_SETTLE_S - SETTLE_TAIL_S
        d["limit_exceeds_observable_window"] = (
            d["limit_s"] is not None and d["limit_s"] > obs)
        d["observable_window_s"] = obs
        all_ok = all_ok and bool(d["ok"])
        lim[cname] = d
    out["settle_limits"] = lim
    out["settle_time_limit_s"] = max(
        (d["limit_s"] for d in lim.values() if d.get("limit_s") is not None),
        default=None)
    out["settled_within_limit"] = bool(all_ok) and (not never)
    return out


# =============================================================================
# window analysis = inherited cruise analysis + this stage's additions
# =============================================================================
def analyze_window(samples, label, p, ptch_trim_deg):
    out = cruise.analyze_window(samples, label, p, ptch_trim_deg)
    if out.get("insufficient_samples"):
        return out
    out["energy"] = energy_block(samples, p)
    out["high_j"] = high_j_block(samples)

    # ArduPlane's own altitude demand (see module docstring derivation)
    ts_t, tgt = collect(samples, s_ap_target_alt_rel_m)
    out["ap_target_alt_rel_m"] = series_report(ts_t, tgt, "ap_target_alt_rel") if tgt else None
    _, apalt = collect(samples, s_ap_alt_rel_m)
    out["ap_alt_rel_m"] = minmaxmean(apalt)
    resid = []
    for s in samples:
        t_, a_ = s_ap_target_alt_rel_m(s), s_ap_alt_rel_m(s)
        if t_ is not None and a_ is not None:
            resid.append(t_ - a_)
    out["ap_height_error_m"] = minmaxmean(resid)     # == NAV_CONTROLLER_OUTPUT.alt_error
    # datum offset between ArduPlane's home-relative altitude and Gazebo z,
    # recorded so the two frames can never be silently conflated
    off = []
    for s in samples:
        z, a_ = s_alt(s), s_ap_alt_rel_m(s)
        if z is not None and a_ is not None:
            off.append(z - a_)
    out["gz_z_minus_ap_relative_alt_m"] = minmaxmean(off)

    # elevator, signed (the inherited block reports max|.| only)
    ts_e, el = collect(samples, s_elev_deg)
    out["elevator_deg"] = series_report(ts_e, el, "elevator_deg") if el else None
    return out


# =============================================================================
# analysis over the whole campaign
# =============================================================================
def analyze(R, segs, p, ptch_trim_deg):
    ptd = ptch_trim_deg
    an = {"phase_plan": dict(
        p1_cruise_s=P1_CRUISE_S, p1_transient_s=P1_TRANSIENT_S,
        p2_climb_max_s=P2_CLIMB_MAX_S, p3_settle_s=P3_SETTLE_S,
        p4_descent_max_s=P4_DESCENT_MAX_S, p5_resettle_s=P5_RESETTLE_S,
        hold_transient_s=HOLD_TRANSIENT_S, settle_tail_s=SETTLE_TAIL_S,
        alt_step_m=ALT_STEP_M, v_target_ms=V_TARGET_MS)}

    # ---- per-phase windows -------------------------------------------------
    for ph in PHASES:
        seg = segs.get(ph)
        if seg is None:
            an[ph + "_full"] = None
            continue
        an[ph + "_full"] = analyze_window(seg["samples"], ph + "_full", p, ptd)
    for ph, tr in (("P1_cruise", P1_TRANSIENT_S),
                   ("P3_settle", HOLD_TRANSIENT_S),
                   ("P5_resettle", HOLD_TRANSIENT_S)):
        seg = segs.get(ph)
        if seg is None:
            an[ph + "_hold"] = None
            continue
        sub = [s for s in seg["samples"] if s["t_seg"] >= tr]
        an[ph + "_hold"] = analyze_window(sub, ph + "_hold", p, ptd)

    # ---- settling after EACH transient ------------------------------------
    an["settling"] = {}
    for ph in ("P3_settle", "P5_resettle"):
        seg = segs.get(ph)
        an["settling"][ph] = (settling_analysis(seg["samples"], ph)
                              if seg else None)

    # ---- whole-flight ------------------------------------------------------
    allsamp = []
    for ph in PHASES:
        if ph in segs:
            allsamp.extend(segs[ph]["samples"])
    allsamp.sort(key=lambda s: s["t"])
    _, thr_all = collect(allsamp, s_throttle_actual)
    thr_min_p = (p.get("THR_MIN") or 0.0) / 100.0
    thr_max_p = (p.get("THR_MAX") or 100.0) / 100.0
    elev_all = [abs(v) for v in
                (s_surface_deg(s, n) for s in allsamp
                 for n in ("left_elevator", "right_elevator")) if v is not None]
    lat_all = [abs(v) for v in
               (s_surface_deg(s, n) for s in allsamp
                for n in ("left_aileron", "right_aileron", "rudder")) if v is not None]
    asp_all = [s["mav"]["airspeed"] for s in allsamp if s["mav"]["airspeed"] is not None]
    an["whole_flight"] = dict(
        n_samples=len(allsamp),
        duration_s=(allsamp[-1]["t"] - allsamp[0]["t"]) if len(allsamp) >= 2 else None,
        throttle_range=(max(thr_all) - min(thr_all)) if thr_all else None,
        throttle_min=min(thr_all) if thr_all else None,
        throttle_max=max(thr_all) if thr_all else None,
        thr_min_param=thr_min_p, thr_max_param=thr_max_p,
        throttle_sat_high_longest_run_s=longest_run_seconds(
            allsamp, lambda s: (s_throttle_actual(s) is not None
                                and s_throttle_actual(s) >= thr_max_p - TH_SAT_MARGIN)),
        throttle_sat_low_longest_run_s=longest_run_seconds(
            allsamp, lambda s: (s_throttle_actual(s) is not None
                                and s_throttle_actual(s) <= thr_min_p + TH_SAT_MARGIN)),
        elevator_max_abs_deg=max(elev_all) if elev_all else None,
        surface_travel_limit_deg=SURFACE_TRAVEL_LIMIT_DEG,
        surface_limit_margin_deg=TH_SURF_LIMIT_MARGIN_DEG,
        lateral_surface_max_abs_deg=max(lat_all) if lat_all else None,
        airspeed_min_ms=min(asp_all) if asp_all else None,
        airspeed_max_ms=max(asp_all) if asp_all else None,
        high_j=high_j_block(allsamp),
    )

    # ---- TECS authority (re-proved, not inherited on trust) ----------------
    rc3 = segs["P1_cruise"]["rc3"]
    manual_equiv = cruise.control_in_range_no_dz(
        rc3, p["RC3_MIN"], p["RC3_MAX"], bool(p["RC3_REVERSED"])) / 100.0
    a_thr = an["P1_cruise_hold"].get("throttle_actual") if an.get("P1_cruise_hold") else None
    an["tecs_authority"] = dict(
        rc3_pwm_us=rc3,
        manual_passthrough_equivalent_throttle=manual_equiv,
        measured_throttle_mean_P1_hold=(a_thr["mean"] if a_thr else None),
        abs_delta=(abs(a_thr["mean"] - manual_equiv) if a_thr else None),
        note="In FBWB the throttle stick sets target AIRSPEED "
             "(ArduPlane/navigation.cpp:187-189); throttle itself is TECS output "
             "(Attitude.cpp:510). A large delta proves TECS - not the stick - is "
             "the throttle authority.")

    # ---- altitude step: ACTUAL and ARDUPLANE'S OWN TARGET ------------------
    def wm(win, key, sub=None):
        d = an.get(win)
        if not d or d.get(key) is None:
            return None
        v = d[key]
        if isinstance(v, dict):
            return v.get(sub or "mean")
        return v

    alt_p1 = wm("P1_cruise_hold", "altitude_gz_m")
    alt_p3 = wm("P3_settle_hold", "altitude_gz_m")
    alt_p5 = wm("P5_resettle_hold", "altitude_gz_m")
    tgt_p1 = wm("P1_cruise_hold", "ap_target_alt_rel_m")
    tgt_p3 = wm("P3_settle_hold", "ap_target_alt_rel_m")
    tgt_p5 = wm("P5_resettle_hold", "ap_target_alt_rel_m")

    def d(a, b):
        return (a - b) if (a is not None and b is not None) else None

    an["altitude_step"] = dict(
        commanded_step_m=ALT_STEP_M,
        reference_altitude_m=R.get("reference_altitude_m"),
        alt_P1_hold_mean_m=alt_p1, alt_P3_hold_mean_m=alt_p3, alt_P5_hold_mean_m=alt_p5,
        achieved_climb_m=d(alt_p3, alt_p1),
        achieved_descent_m=d(alt_p5, alt_p3),
        roundtrip_residual_m=d(alt_p5, alt_p1),
        ap_target_P1_hold_m=tgt_p1, ap_target_P3_hold_m=tgt_p3, ap_target_P5_hold_m=tgt_p5,
        ap_target_climb_step_m=d(tgt_p3, tgt_p1),
        ap_target_descent_step_m=d(tgt_p5, tgt_p3),
        ap_target_roundtrip_residual_m=d(tgt_p5, tgt_p1),
        target_vs_actual_climb_gap_m=(
            d(d(tgt_p3, tgt_p1), d(alt_p3, alt_p1))),
        target_vs_actual_descent_gap_m=(
            d(d(tgt_p5, tgt_p3), d(alt_p5, alt_p3))),
        climb_ramp_duration_s=segs.get("P2_climb", {}).get("actual_duration_s"),
        descent_ramp_duration_s=segs.get("P4_descent", {}).get("actual_duration_s"),
        climb_ramp_stopped_early=segs.get("P2_climb", {}).get("stopped_early"),
        descent_ramp_stopped_early=segs.get("P4_descent", {}).get("stopped_early"),
        climb_ramp_stop_reason=segs.get("P2_climb", {}).get("stop_reason"),
        descent_ramp_stop_reason=segs.get("P4_descent", {}).get("stop_reason"),
        note="ap_target_* is ArduPlane's OWN height demand reconstructed from "
             "GLOBAL_POSITION_INT.relative_alt + NAV_CONTROLLER_OUTPUT.alt_error "
             "(see module docstring). It verifies that the FBWB stick ramp really "
             "moved the demand by ~+/-10 m rather than assuming it integrated.")

    # ---- energy management summary (first-class deliverable) ---------------
    def eb(win, key):
        w = an.get(win)
        if not w or not w.get("energy"):
            return None
        return w["energy"].get(key)

    an["energy_management"] = dict(
        per_phase={ph: (an[ph + "_full"]["energy"] if an.get(ph + "_full")
                        and an[ph + "_full"].get("energy") else None)
                   for ph in PHASES},
        hold_windows={w: (an[w]["energy"] if an.get(w) and an[w].get("energy") else None)
                      for w in ("P1_cruise_hold", "P3_settle_hold", "P5_resettle_hold")},
        summary=dict(
            level_STEdot_W_per_kg=eb("P1_cruise_hold", "STEdot_W_per_kg"),
            settle_STEdot_W_per_kg=eb("P3_settle_hold", "STEdot_W_per_kg"),
            resettle_STEdot_W_per_kg=eb("P5_resettle_hold", "STEdot_W_per_kg"),
            climb_STEdot_W_per_kg=eb("P2_climb_full", "STEdot_W_per_kg"),
            descent_STEdot_W_per_kg=eb("P4_descent_full", "STEdot_W_per_kg"),
            climb_SPEdot_W_per_kg=eb("P2_climb_full", "SPEdot_W_per_kg"),
            climb_SKEdot_W_per_kg=eb("P2_climb_full", "SKEdot_W_per_kg"),
            descent_SPEdot_W_per_kg=eb("P4_descent_full", "SPEdot_W_per_kg"),
            descent_SKEdot_W_per_kg=eb("P4_descent_full", "SKEdot_W_per_kg"),
            climb_SEBdot_W_per_kg=eb("P2_climb_full", "SEBdot_W_per_kg"),
            descent_SEBdot_W_per_kg=eb("P4_descent_full", "SEBdot_W_per_kg"),
            climb_kinetic_over_potential=eb("P2_climb_full", "kinetic_over_potential_abs"),
            descent_kinetic_over_potential=eb("P4_descent_full", "kinetic_over_potential_abs"),
            STE_level_J_per_kg=(an["P1_cruise_hold"]["energy"]["STE_J_per_kg"]["mean"]
                                if eb("P1_cruise_hold", "STE_J_per_kg") else None),
            STE_resettle_J_per_kg=(an["P5_resettle_hold"]["energy"]["STE_J_per_kg"]["mean"]
                                   if eb("P5_resettle_hold", "STE_J_per_kg") else None),
        ),
        definitions="AP_TECS.cpp:678-697 (energies/rates), :1024-1036 (balance). "
                    "Rates here are RAW kinematic rates over the window, not TECS's "
                    "high-passed internal state - see the module docstring FILTER NOTE.",
        reference_prior_stage=dict(
            climb_W_per_kg=PRIOR["climb_specific_energy_rate_W_per_kg"],
            descent_W_per_kg=PRIOR["descent_specific_energy_rate_W_per_kg"],
            level_W_per_kg=PRIOR["level_specific_energy_rate_W_per_kg"],
            source="docs/test_results/2026-09-02_ardupilot_tecs_and_cruise_speed_"
                   "hold_validation.md sec 7"),
    )
    ste_l = an["energy_management"]["summary"]["STE_level_J_per_kg"]
    ste_r = an["energy_management"]["summary"]["STE_resettle_J_per_kg"]
    an["energy_management"]["summary"]["STE_roundtrip_residual_J_per_kg"] = (
        (ste_r - ste_l) if (ste_l is not None and ste_r is not None) else None)

    # ---- throttle / pitch division of labour -------------------------------
    an["coordination"] = dict(
        level_throttle=wm("P1_cruise_hold", "throttle_actual"),
        climb_throttle=wm("P2_climb_full", "throttle_actual"),
        settle_throttle=wm("P3_settle_hold", "throttle_actual"),
        descent_throttle=wm("P4_descent_full", "throttle_actual"),
        resettle_throttle=wm("P5_resettle_hold", "throttle_actual"),
        level_pitch_deg=wm("P1_cruise_hold", "pitch_physical_noseup_deg"),
        climb_pitch_deg=wm("P2_climb_full", "pitch_physical_noseup_deg"),
        settle_pitch_deg=wm("P3_settle_hold", "pitch_physical_noseup_deg"),
        descent_pitch_deg=wm("P4_descent_full", "pitch_physical_noseup_deg"),
        resettle_pitch_deg=wm("P5_resettle_hold", "pitch_physical_noseup_deg"),
        level_nav_pitch_raw_deg=wm("P1_cruise_hold", "nav_pitch_raw_tecs_demand_deg"),
        climb_nav_pitch_raw_deg=wm("P2_climb_full", "nav_pitch_raw_tecs_demand_deg"),
        descent_nav_pitch_raw_deg=wm("P4_descent_full", "nav_pitch_raw_tecs_demand_deg"),
        level_pitch_demand_phys_deg=wm("P1_cruise_hold", "pitch_demand_physical_deg"),
        climb_pitch_demand_phys_deg=wm("P2_climb_full", "pitch_demand_physical_deg"),
        descent_pitch_demand_phys_deg=wm("P4_descent_full", "pitch_demand_physical_deg"),
        level_elevator_deg=wm("P1_cruise_hold", "elevator_deg"),
        climb_elevator_deg=wm("P2_climb_full", "elevator_deg"),
        descent_elevator_deg=wm("P4_descent_full", "elevator_deg"),
        level_vz_ms=wm("P1_cruise_hold", "vfr_hud_climb_ms"),
        climb_vz_ms=wm("P2_climb_full", "vfr_hud_climb_ms"),
        descent_vz_ms=wm("P4_descent_full", "vfr_hud_climb_ms"),
        note="TECS division of labour: the THROTTLE loop tracks total specific "
             "energy rate STEdot (AP_TECS.cpp:739-772) and the PITCH loop tracks "
             "the specific energy BALANCE SEBdot (AP_TECS.cpp:1031-1096). Climb "
             "must therefore show MORE throttle and MORE nose-up than level; "
             "descent the opposite. The checks test the SIGN of the split, not "
             "its magnitude. Raw nav_pitch and the PTCH_TRIM_DEG-corrected "
             "physical demand are both reported so the convention cannot be "
             "double counted.")

    # ---- descent airspeed excursion ---------------------------------------
    dsamp = list(segs.get("P4_descent", {}).get("samples", []))
    dsamp += [s for s in segs.get("P5_resettle", {}).get("samples", [])
              if s["t_seg"] <= HOLD_TRANSIENT_S]
    dv = [s["mav"]["airspeed"] for s in dsamp if s["mav"]["airspeed"] is not None]
    dtg = [s_tecs_target_airspeed(s) for s in dsamp]
    dtg = [v for v in dtg if v is not None]
    an["descent_speed_excursion"] = dict(
        n_samples=len(dv),
        airspeed_max_ms=max(dv) if dv else None,
        airspeed_min_ms=min(dv) if dv else None,
        tecs_target_mean_ms=mean(dtg) if dtg else None,
        overshoot_above_target_ms=((max(dv) - mean(dtg)) if dv and dtg else None),
        window="P4_descent + the first 10 s of P5_resettle (the recovery transient)",
        note="Guards against an energy-management failure in which the descent is "
             "flown by trading altitude into SPEED instead of reducing throttle.")

    R["analysis"] = an
    return an


# =============================================================================
# acceptance
# =============================================================================
def verdict(R):
    an = R.get("analysis")
    if not an or not an.get("P1_cruise_hold") or an["P1_cruise_hold"].get("insufficient_samples"):
        return "TECS_CLIMB_DESCENT_ENERGY_FAILED", ["no analysable P1 cruise hold window"]
    p = R.get("tecs_baseline_params_live", {})
    pre = R.get("param_preconditions", {})
    wf = an["whole_flight"]
    st = an["altitude_step"]
    es = an["energy_management"]["summary"]
    co = an["coordination"]
    c = {}

    def num(x):
        return isinstance(x, (int, float)) and math.isfinite(x)

    def abs_le(x, lim):
        return num(x) and abs(x) <= lim

    def sub_abs_le(dd, key, lim):
        return isinstance(dd, dict) and abs_le(dd.get(key), lim)

    def win(name):
        w = an.get(name)
        return w if (w and not w.get("insufficient_samples")) else None

    # --- 1. mode / configuration integrity ----------------------------------
    modes_ok = True
    for ph in PHASES:
        w = win(ph + "_full")
        if w is None or not w.get("all_fbwb"):
            modes_ok = False
    c["mode_is_fbwb_throughout"] = modes_ok
    c["param_preconditions_all_ok"] = all(pre.values()) if pre else False
    c["tecs_at_firmware_defaults_baseline"] = bool(pre.get("tecs_at_firmware_defaults"))
    c["pids_unchanged"] = bool(pre.get("pids_unchanged"))
    c["ptch_trim_deg_unchanged"] = bool(pre.get("ptch_trim_deg_2p49"))
    c["target_altitude_readback_preconditions_ok"] = bool(
        pre.get("alt_offset_zero_for_target_readback")
        and pre.get("terrain_follow_zero"))

    # --- 2. TECS is genuinely the speed/height controller -------------------
    ta = an["tecs_authority"]
    c["tecs_is_driving_throttle_not_the_stick"] = (
        num(ta["abs_delta"]) and ta["abs_delta"] > TH_TECS_AUTHORITY_MIN_DELTA)
    c["throttle_is_actively_modulated"] = (
        num(wf.get("throttle_range")) and wf["throttle_range"] > TH_THROTTLE_MODULATION_MIN)
    tt = an["P1_cruise_hold"].get("tecs_target_airspeed_ms")
    c["tecs_target_airspeed_matches_command"] = (
        isinstance(tt, dict) and num(tt.get("mean"))
        and abs(tt["mean"] - V_TARGET_MS) <= TH_TECS_TARGET_TOL_MS)

    # --- 3. speed --------------------------------------------------------------
    for wname, tag in (("P1_cruise_hold", "P1"), ("P3_settle_hold", "P3"),
                       ("P5_resettle_hold", "P5")):
        w = win(wname)
        asp = w.get("airspeed_vfr_hud_ms") if w else None
        c[f"{tag}_hold_window_present"] = w is not None
        c[f"speed_mean_within_tol_{tag}"] = (
            isinstance(asp, dict) and num(asp.get("mean"))
            and abs(asp["mean"] - V_TARGET_MS) <= TH_SPEED_MEAN_TOL_MS)
        c[f"speed_std_bounded_{tag}"] = sub_abs_le(asp, "std", TH_SPEED_STD_MAX_MS)
        c[f"speed_no_divergence_{tag}"] = sub_abs_le(asp, "slope_per_s",
                                                     TH_SPEED_SLOPE_MAX_MS_PER_S)
    c["speed_never_below_airspeed_min"] = (
        num(wf.get("airspeed_min_ms")) and wf["airspeed_min_ms"] >= TH_SPEED_MIN_MS)
    c["speed_never_below_underspeed_trigger"] = (
        num(wf.get("airspeed_min_ms")) and wf["airspeed_min_ms"] >= TH_SPEED_HARD_FLOOR_MS)
    c["speed_never_above_airspeed_max"] = (
        num(wf.get("airspeed_max_ms")) and p.get("AIRSPEED_MAX") is not None
        and wf["airspeed_max_ms"] <= p["AIRSPEED_MAX"])
    dse = an["descent_speed_excursion"]
    c["no_airspeed_runaway_in_descent"] = abs_le(
        dse.get("overshoot_above_target_ms"), TH_DESCENT_SPEED_OVERSHOOT_MAX_MS)
    gsd = an["P1_cruise_hold"].get("groundspeed_minus_airspeed_max_abs_ms")
    c["zero_wind_confirmed_gs_vs_as"] = abs_le(gsd, TH_GS_VS_AS_MAX_MS)

    # --- 4. altitude hold in all three level phases -------------------------
    for wname, tag in (("P1_cruise_hold", "P1"), ("P3_settle_hold", "P3"),
                       ("P5_resettle_hold", "P5")):
        w = win(wname)
        c[f"alt_hold_slope_bounded_{tag}"] = (
            w is not None and sub_abs_le(w, "vertical_speed_regression_ms",
                                         TH_ALT_SLOPE_MAX_MS))
        c[f"alt_hold_p2p_bounded_{tag}"] = (
            w is not None and sub_abs_le(w, "altitude_p2p_m", TH_ALT_P2P_MAX_M))
        c[f"alt_hold_no_unidirectional_drift_{tag}"] = (
            w is not None and sub_abs_le(w, "vertical_speed_endpoint_ms",
                                         TH_ALT_SLOPE_MAX_MS))
    c["fbwa_residual_sink_closed_P1"] = sub_abs_le(
        an["P1_cruise_hold"], "vertical_speed_regression_ms", TH_SINK_CLOSED_MS)
    c["alt_hold_tight_P1_preferred"] = sub_abs_le(
        an["P1_cruise_hold"], "vertical_speed_regression_ms", TH_ALT_SLOPE_TIGHT_MS)

    # --- 5. the steps happened, in the right direction, by the right amount --
    c["altitude_step_climb_achieved"] = (
        num(st["achieved_climb_m"])
        and st["achieved_climb_m"] >= TH_ALT_STEP_ACHIEVED_FRAC * ALT_STEP_M)
    c["altitude_step_descent_achieved"] = (
        num(st["achieved_descent_m"])
        and st["achieved_descent_m"] <= -TH_ALT_STEP_ACHIEVED_FRAC * ALT_STEP_M)
    # ArduPlane's OWN demand really moved by ~+/-10 m (not assumed)
    c["ap_target_alt_climb_step_verified"] = (
        num(st["ap_target_climb_step_m"])
        and abs(st["ap_target_climb_step_m"] - ALT_STEP_M) <= TH_TARGET_STEP_TOL_M)
    c["ap_target_alt_descent_step_verified"] = (
        num(st["ap_target_descent_step_m"])
        and abs(st["ap_target_descent_step_m"] + ALT_STEP_M) <= TH_TARGET_STEP_TOL_M)
    c["ap_target_tracks_actual_climb"] = abs_le(
        st["target_vs_actual_climb_gap_m"], TH_TARGET_VS_ACTUAL_TOL_M)
    c["ap_target_tracks_actual_descent"] = abs_le(
        st["target_vs_actual_descent_gap_m"], TH_TARGET_VS_ACTUAL_TOL_M)
    # CONTROL DIRECTION - explicitly test-verified, never assumed
    br = win("P2_climb_full")
    dr = win("P4_descent_full")
    c["fbwb_up_stick_climbs"] = (
        br is not None and num(br.get("vertical_speed_regression_ms"))
        and br["vertical_speed_regression_ms"] > TH_RAMP_DIRECTION_MIN_MS)
    c["fbwb_down_stick_descends"] = (
        dr is not None and num(dr.get("vertical_speed_regression_ms"))
        and dr["vertical_speed_regression_ms"] < -TH_RAMP_DIRECTION_MIN_MS)
    # returned to the ORIGINAL altitude
    c["resettled_near_original_altitude"] = abs_le(
        st["roundtrip_residual_m"], TH_RESETTLE_TOL_M)
    c["resettled_near_original_altitude_tight_preferred"] = abs_le(
        st["roundtrip_residual_m"], TH_RESETTLE_TIGHT_M)
    c["ap_target_roundtrip_closed"] = abs_le(
        st["ap_target_roundtrip_residual_m"], TH_RESETTLE_TOL_M)

    # --- 6. throttle equilibrium + saturation -------------------------------
    thr = an["P1_cruise_hold"].get("throttle_actual")
    c["throttle_plausible_vs_measured_trim"] = (
        isinstance(thr, dict) and num(thr.get("mean"))
        and abs(thr["mean"] - TRIM_THROTTLE_REF) <= TH_THROTTLE_TOL)
    c["no_sustained_throttle_high_saturation"] = abs_le(
        wf.get("throttle_sat_high_longest_run_s"), TH_SAT_RUN_MAX_S)
    c["no_sustained_throttle_low_saturation"] = abs_le(
        wf.get("throttle_sat_low_longest_run_s"), TH_SAT_RUN_MAX_S)
    c["throttle_never_pinned_at_a_limit"] = (
        num(wf.get("throttle_min")) and num(wf.get("throttle_max"))
        and wf["throttle_min"] > wf["thr_min_param"] + TH_SAT_MARGIN
        and wf["throttle_max"] < wf["thr_max_param"] - TH_SAT_MARGIN)

    # --- 7. ENERGY MANAGEMENT ----------------------------------------------
    c["climb_total_energy_rate_positive"] = (
        num(es["climb_STEdot_W_per_kg"])
        and es["climb_STEdot_W_per_kg"] >= TH_RAMP_STEDOT_MIN_W_PER_KG)
    c["descent_total_energy_rate_negative"] = (
        num(es["descent_STEdot_W_per_kg"])
        and es["descent_STEdot_W_per_kg"] <= -TH_RAMP_STEDOT_MIN_W_PER_KG)
    c["level_total_energy_rate_near_zero_P1"] = abs_le(
        es["level_STEdot_W_per_kg"], TH_LEVEL_STEDOT_MAX_W_PER_KG)
    c["level_total_energy_rate_near_zero_P3"] = abs_le(
        es["settle_STEdot_W_per_kg"], TH_LEVEL_STEDOT_MAX_W_PER_KG)
    c["level_total_energy_rate_near_zero_P5"] = abs_le(
        es["resettle_STEdot_W_per_kg"], TH_LEVEL_STEDOT_MAX_W_PER_KG)
    c["climb_energy_goes_to_altitude_not_speed"] = abs_le(
        es["climb_kinetic_over_potential"], TH_KINETIC_FRACTION_MAX)
    c["descent_energy_comes_from_altitude_not_speed"] = abs_le(
        es["descent_kinetic_over_potential"], TH_KINETIC_FRACTION_MAX)
    c["energy_roundtrip_closed"] = abs_le(
        es["STE_roundtrip_residual_J_per_kg"], TH_STE_ROUNDTRIP_MAX_J_PER_KG)

    # throttle loop <-> total energy;  pitch loop <-> energy balance
    def gt(a, b, dd):
        return num(a) and num(b) and (a - b) > dd

    def lt(a, b, dd):
        return num(a) and num(b) and (b - a) > dd

    c["climb_uses_more_throttle_than_level"] = gt(
        co["climb_throttle"], co["level_throttle"], TH_COORD_THROTTLE_DELTA)
    c["descent_uses_less_throttle_than_level"] = lt(
        co["descent_throttle"], co["level_throttle"], TH_COORD_THROTTLE_DELTA)
    c["climb_is_more_nose_up_than_level"] = gt(
        co["climb_pitch_deg"], co["level_pitch_deg"], TH_COORD_PITCH_DELTA_DEG)
    c["descent_is_more_nose_down_than_level"] = lt(
        co["descent_pitch_deg"], co["level_pitch_deg"], TH_COORD_PITCH_DELTA_DEG)
    c["climb_energy_balance_rate_positive"] = (
        num(es["climb_SEBdot_W_per_kg"]) and es["climb_SEBdot_W_per_kg"] > 0.0)
    c["descent_energy_balance_rate_negative"] = (
        num(es["descent_SEBdot_W_per_kg"]) and es["descent_SEBdot_W_per_kg"] < 0.0)
    # non-gating physical cross-check
    pres = []
    for wname in ("P1_cruise_hold", "P2_climb_full", "P4_descent_full"):
        w = win(wname)
        if w and w.get("energy"):
            pres.append(w["energy"].get("propulsive_vs_STEdot_residual_W_per_kg"))
    c["propulsive_power_matches_energy_rate_preferred"] = bool(pres) and all(
        abs_le(x, TH_PROPULSIVE_POWER_RESID_W_PER_KG) for x in pres)

    # --- 8. SETTLING / DECAY after each transient ---------------------------
    for ph, tag in (("P3_settle", "after_climb"), ("P5_resettle", "after_descent")):
        sa = an["settling"].get(ph)
        ok = bool(sa) and not sa.get("insufficient_samples") and not sa.get("insufficient_tail")
        c[f"settling_window_present_{tag}"] = ok
        c[f"settles_{tag}"] = ok and bool(sa.get("settled_within_limit"))
        c[f"oscillation_decays_{tag}"] = ok and bool(sa.get("all_channels_decaying"))

    # Scope, made explicit for validation finding V-5: this check evaluates the
    # three HOLD windows only. detrended_growth() compares second-half to
    # first-half residual spread about a STRAIGHT-LINE fit, so on a commanded
    # ramp window it is a curvature statistic, not an oscillation statistic -
    # gating it there would be meaningless. The ramp-window ratios are still
    # RECORDED below as declared INFO so nothing is silently discarded, and the
    # "does the response keep growing after the excitation stops" question is
    # answered by oscillation_decays_after_climb / _after_descent (P3/P5).
    GATED_GROWTH_WINDOWS = ("P1_cruise_hold", "P3_settle_hold", "P5_resettle_hold")
    REPORTED_ONLY_GROWTH_WINDOWS = ("P2_climb_full", "P4_descent_full")
    growing = False
    for wname in GATED_GROWTH_WINDOWS:
        w = win(wname)
        if not w or not w.get("oscillation_growth"):
            continue
        for _, g in w["oscillation_growth"].items():
            if g and g.get("growing"):
                growing = True
    c["no_growing_oscillation_in_hold_windows"] = not growing
    info_growth = {}
    for wname in REPORTED_ONLY_GROWTH_WINDOWS:
        w = win(wname)
        if not w or not w.get("oscillation_growth"):
            continue
        info_growth[wname] = {k: (g.get("ratio_second_over_first") if g else None)
                              for k, g in w["oscillation_growth"].items()}
        info_growth[wname]["growing_flag_raw"] = {
            k: (g.get("growing") if g else None)
            for k, g in w["oscillation_growth"].items()}
    an["oscillation_growth_scope"] = dict(
        gated_windows=list(GATED_GROWTH_WINDOWS),
        reported_only_windows=list(REPORTED_ONLY_GROWTH_WINDOWS),
        reported_only_ratios=info_growth,
        status="INFO - NOT GATED",
        reason="detrended_growth() on a commanded ramp is a curvature statistic, "
               "not an oscillation statistic (validation V-5 / sec 6).")

    # --- 9. control surfaces ------------------------------------------------
    hold_elev_ok = True
    for wname in ("P1_cruise_hold", "P3_settle_hold", "P5_resettle_hold"):
        w = win(wname)
        if w is None or not abs_le(w.get("elevator_max_abs_deg"), TH_SURF_HOLD_MAX_DEG):
            hold_elev_ok = False
    c["elevator_within_10deg_in_all_hold_windows"] = hold_elev_ok
    c["elevator_within_15deg_whole_flight"] = abs_le(
        wf.get("elevator_max_abs_deg"), TH_SURF_FLIGHT_MAX_DEG)
    c["elevator_never_near_travel_limit"] = abs_le(
        wf.get("elevator_max_abs_deg"), TH_SURF_MAX_ABS_DEG)
    c["lateral_surfaces_bounded"] = abs_le(
        wf.get("lateral_surface_max_abs_deg"), TH_LATERAL_SURF_MAX_DEG)
    clamp_total = 0
    for ph in PHASES:
        w = win(ph + "_full")
        if w and w.get("actuator_clamp"):
            clamp_total += (w["actuator_clamp"]["target_clamp_active_samples"]
                            + w["actuator_clamp"]["effort_clamp_active_samples"])
    c["zero_actuator_clamp"] = (clamp_total == 0)

    # --- 10. kinematics / numerics -----------------------------------------
    par = an["P1_cruise_hold"].get("pitch_minus_alpha_minus_gamma_deg")
    c["longitudinal_kinematics_consistent"] = sub_abs_le(
        par, "mean", TH_PITCH_ALPHA_GAMMA_RESID_DEG)
    nan_total = 0
    for ph in PHASES:
        w = an.get(ph + "_full")
        if w and "nan_inf_count" in w:
            nan_total += w["nan_inf_count"]
    c["no_nan_inf"] = (nan_total == 0)

    R["acceptance_checks"] = c
    fails = [k for k, ok in c.items() if not ok and not k.endswith("_preferred")]

    # CORE = the criteria that decide the stage. A criterion is core if failing
    # it would mean the energy management claim is not supported.
    core = [
        "mode_is_fbwb_throughout", "param_preconditions_all_ok",
        "tecs_at_firmware_defaults_baseline", "pids_unchanged",
        "ptch_trim_deg_unchanged", "target_altitude_readback_preconditions_ok",
        "tecs_is_driving_throttle_not_the_stick", "throttle_is_actively_modulated",
        "tecs_target_airspeed_matches_command",
        "speed_mean_within_tol_P1", "speed_mean_within_tol_P3", "speed_mean_within_tol_P5",
        "speed_std_bounded_P1", "speed_std_bounded_P3", "speed_std_bounded_P5",
        "speed_never_below_airspeed_min", "speed_never_below_underspeed_trigger",
        "speed_never_above_airspeed_max", "no_airspeed_runaway_in_descent",
        "zero_wind_confirmed_gs_vs_as",
        "alt_hold_slope_bounded_P1", "alt_hold_slope_bounded_P3", "alt_hold_slope_bounded_P5",
        "alt_hold_p2p_bounded_P1", "alt_hold_p2p_bounded_P3", "alt_hold_p2p_bounded_P5",
        "fbwa_residual_sink_closed_P1",
        "altitude_step_climb_achieved", "altitude_step_descent_achieved",
        "ap_target_alt_climb_step_verified", "ap_target_alt_descent_step_verified",
        "ap_target_tracks_actual_climb", "ap_target_tracks_actual_descent",
        "fbwb_up_stick_climbs", "fbwb_down_stick_descends",
        "resettled_near_original_altitude", "ap_target_roundtrip_closed",
        "throttle_plausible_vs_measured_trim",
        "no_sustained_throttle_high_saturation", "no_sustained_throttle_low_saturation",
        "throttle_never_pinned_at_a_limit",
        "climb_total_energy_rate_positive", "descent_total_energy_rate_negative",
        "level_total_energy_rate_near_zero_P1", "level_total_energy_rate_near_zero_P3",
        "level_total_energy_rate_near_zero_P5",
        "climb_energy_goes_to_altitude_not_speed",
        "descent_energy_comes_from_altitude_not_speed",
        "energy_roundtrip_closed",
        "climb_uses_more_throttle_than_level", "descent_uses_less_throttle_than_level",
        "climb_is_more_nose_up_than_level", "descent_is_more_nose_down_than_level",
        "climb_energy_balance_rate_positive", "descent_energy_balance_rate_negative",
        "settles_after_climb", "settles_after_descent",
        "oscillation_decays_after_climb", "oscillation_decays_after_descent",
        "no_growing_oscillation_in_hold_windows",
        "elevator_within_10deg_in_all_hold_windows",
        "elevator_within_15deg_whole_flight", "elevator_never_near_travel_limit",
        "lateral_surfaces_bounded", "zero_actuator_clamp",
        "longitudinal_kinematics_consistent", "no_nan_inf",
    ]
    R["core_checks"] = core
    core_ok = all(c.get(k, False) for k in core)
    # PARTIAL = TECS demonstrably manages energy correctly and the flight is
    # safe/stable/correctly-signed, but a secondary quantitative criterion
    # missed. NEVER used to hide a direction/sign, saturation, clamp, NaN,
    # divergence or settling failure.
    partial_ok = all(c.get(k, False) for k in [
        "mode_is_fbwb_throughout", "tecs_is_driving_throttle_not_the_stick",
        "fbwb_up_stick_climbs", "fbwb_down_stick_descends",
        "climb_total_energy_rate_positive", "descent_total_energy_rate_negative",
        "climb_uses_more_throttle_than_level", "descent_uses_less_throttle_than_level",
        "climb_is_more_nose_up_than_level", "descent_is_more_nose_down_than_level",
        "settles_after_climb", "settles_after_descent",
        "oscillation_decays_after_climb", "oscillation_decays_after_descent",
        "no_growing_oscillation_in_hold_windows", "no_nan_inf",
        "zero_actuator_clamp",
        "speed_never_below_airspeed_min", "no_airspeed_runaway_in_descent",
        "no_sustained_throttle_high_saturation", "no_sustained_throttle_low_saturation",
        "elevator_never_near_travel_limit"])
    if core_ok:
        return "TECS_CLIMB_DESCENT_ENERGY_PASS", fails
    if partial_ok:
        return "TECS_CLIMB_DESCENT_ENERGY_PARTIAL", fails
    return "TECS_CLIMB_DESCENT_ENERGY_FAILED", fails


# =============================================================================
# per-sample trace (the stage's "record per sample" deliverable)
# =============================================================================
TRACE_COLUMNS = [
    "phase", "t_s", "t_seg_s",
    "target_airspeed_tecs_ms", "airspeed_eas_ms", "airspeed_tas_ms",
    "ap_target_alt_rel_m", "ap_alt_rel_m", "altitude_gz_m",
    "vertical_speed_ms",
    "pitch_physical_deg", "nav_pitch_raw_deg", "pitch_demand_physical_deg",
    "throttle", "elevator_deg",
    "rpm_left", "rpm_right", "thrust_left_N", "thrust_right_N",
    "J_left", "J_right", "interp_clamped_left", "interp_clamped_right",
    "SPE_J_per_kg", "SKE_J_per_kg", "STE_J_per_kg", "SEB_J_per_kg",
    "throttle_saturated", "surface_clamped", "alpha_deg",
]


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
                s_tecs_target_airspeed(s), s["mav"]["airspeed"], s_tas(s),
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
        demand_quantisation_grid_ms=(
            (int(p["AIRSPEED_MAX"]) - int(p["AIRSPEED_MIN"])) / 100.0
            if p.get("AIRSPEED_MAX") is not None and p.get("AIRSPEED_MIN") is not None
            else None),
        demand_quantisation_error_ms=(
            (achieved_target - V_TARGET_MS) if achieved_target is not None else None),
        rc2_up_us=p.get("RC2_MAX"), rc2_down_us=p.get("RC2_MIN"),
        rc2_neutral_us=p.get("RC2_TRIM"),
        formula="ArduPlane/navigation.cpp:187-189 inverted through "
                "RC_Channel.cpp:388-402, including the int16_t control_in "
                "truncation (RC_Channel.h:99/542). Derivation and the resulting "
                "0.08 m/s command quantisation: docs/test_results/"
                "2026-09-02_ardupilot_tecs_and_cruise_speed_hold_validation.md sec 3.")
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

    # ---------- P1: level cruise + altitude hold ---------------------------
    print(f"P1 CRUISE: {P1_CRUISE_S}s level cruise (rc2={rc2_neutral}, "
          f"rc3={rc3_cruise} -> TECS target {achieved_target:.3f} m/s)")
    segs["P1_cruise"] = run("P1_cruise", P1_CRUISE_S, rc2_neutral)
    if segs["P1_cruise"]["aborted"]:
        fail("P1_cruise", segs["P1_cruise"])
        return False, segs

    # REFERENCE ALTITUDE = mean of the SETTLED part of P1 (not the last
    # sample), so the phugoid phase at the instant P1 ends cannot bias the
    # commanded step or the return-to-origin criterion.
    p1_hold = [s for s in segs["P1_cruise"]["samples"] if s["t_seg"] >= P1_TRANSIENT_S]
    _, z_hold = collect(p1_hold, s_alt)
    if len(z_hold) < 4:
        R["flight_result"] = dict(aborted=True, reason="no_P1_reference_altitude")
        return False, segs
    z_ref = mean(z_hold)
    R["reference_altitude_m"] = z_ref
    R["reference_altitude_note"] = (
        "mean Gazebo z over the settled part of P1 (t_seg >= "
        f"{P1_TRANSIENT_S} s, n={len(z_hold)}). Used as BOTH the climb target "
        "(+10 m) and the descent target (return to origin).")
    print(f"reference altitude (P1 settled mean): {z_ref:.3f} m")

    # ---------- P2: +10 m climb via the FBWB pitch-stick ramp ---------------
    z_climb_target = z_ref + ALT_STEP_M
    state = {"n": 0}

    def stop_climb(s, _):
        z = s_alt(s)
        if z is not None and z >= z_climb_target:
            state["n"] += 1
        else:
            state["n"] = 0
        if state["n"] >= RAMP_STOP_CONSECUTIVE:
            return True, (f"altitude >= {z_climb_target:.2f} m for "
                          f"{RAMP_STOP_CONSECUTIVE} consecutive samples")
        return False, None

    print(f"P2 CLIMB: up-stick rc2={rc2_up}, cap {P2_CLIMB_MAX_S}s, "
          f"stop at z >= {z_climb_target:.2f} m")
    segs["P2_climb"] = run("P2_climb", P2_CLIMB_MAX_S, rc2_up, stop_fn=stop_climb)
    if segs["P2_climb"]["aborted"]:
        fail("P2_climb", segs["P2_climb"])
        return False, segs

    # ---------- P3: settle / level off at the new altitude ------------------
    print(f"P3 SETTLE: {P3_SETTLE_S}s at the new altitude (rc2={rc2_neutral} "
          f"-> set_target_altitude_current(), navigation.cpp:421-426)")
    segs["P3_settle"] = run("P3_settle", P3_SETTLE_S, rc2_neutral)
    if segs["P3_settle"]["aborted"]:
        fail("P3_settle", segs["P3_settle"])
        return False, segs

    # ---------- P4: -10 m descent, targeting the ORIGINAL altitude ----------
    state2 = {"n": 0}

    def stop_descend(s, _):
        z = s_alt(s)
        if z is not None and z <= z_ref:
            state2["n"] += 1
        else:
            state2["n"] = 0
        if state2["n"] >= RAMP_STOP_CONSECUTIVE:
            return True, (f"altitude <= original reference {z_ref:.2f} m for "
                          f"{RAMP_STOP_CONSECUTIVE} consecutive samples")
        return False, None

    print(f"P4 DESCENT: down-stick rc2={rc2_down}, cap {P4_DESCENT_MAX_S}s, "
          f"stop at z <= {z_ref:.2f} m (the ORIGINAL altitude)")
    segs["P4_descent"] = run("P4_descent", P4_DESCENT_MAX_S, rc2_down,
                             stop_fn=stop_descend)
    if segs["P4_descent"]["aborted"]:
        fail("P4_descent", segs["P4_descent"])
        return False, segs

    # ---------- P5: re-settle near the ORIGINAL altitude --------------------
    print(f"P5 RESETTLE: {P5_RESETTLE_S}s near the original altitude "
          f"(rc2={rc2_neutral})")
    segs["P5_resettle"] = run("P5_resettle", P5_RESETTLE_S, rc2_neutral)

    aborted = any(v["aborted"] for v in segs.values())
    R["flight_result"] = dict(
        aborted=aborted,
        reference_altitude_m=z_ref,
        climb_target_altitude_m=z_climb_target,
        descent_target_altitude_m=z_ref,
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
    ts_doc = {"stage": STAGE, "timestamp": R.get("timestamp"),
              "tecs_baseline_params_live": R.get("tecs_baseline_params_live"),
              "command_derivation": R.get("command_derivation"),
              "reference_altitude_m": R.get("reference_altitude_m"),
              "segments": {k: {kk: vv for kk, vv in v.items()} for k, v in segs.items()}}
    with open(OUT_TS, "w") as f:
        # compact separators: the raw record is kept COMPLETE (nothing dropped)
        # so validation can independently re-derive any quantity.
        json.dump(ts_doc, f, default=str, separators=(",", ":"))
    if segs and (p or R.get("tecs_baseline_params_live")):
        pp = p or R["tecs_baseline_params_live"]
        try:
            trace = build_trace(segs, pp, pp.get("PTCH_TRIM_DEG", PTCH_TRIM_DEG_EXPECTED))
            with open(OUT_TRACE, "w") as f:
                json.dump({"stage": STAGE, "columns": TRACE_COLUMNS,
                           "units_note": "SI throughout; angles in deg where the "
                                         "column name says deg; energies J/kg; "
                                         "energy rates W/kg. altitude_gz_m is "
                                         "Gazebo world z (FLU +Z up); "
                                         "ap_alt_rel_m / ap_target_alt_rel_m are "
                                         "ArduPlane altitudes above HOME.",
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
    R["verdict"] = "TECS_CLIMB_DESCENT_ENERGY_FAILED"
    R["blocking_phase"] = phase
    write_outputs(R, segs or {})
    print(f"FAILED at {phase} - see", OUT_JSON)
    if mav is not None:
        mav.close()
    return 1


def reanalyze(path):
    """Re-run analyze()+verdict() offline against a captured timeseries file.
    The flight is NOT re-flown; this only regenerates the analysis after a
    TEST-LOGIC (never physics) fix."""
    with open(path) as f:
        doc = json.load(f)
    segs = doc["segments"]
    p = doc["tecs_baseline_params_live"]
    R = {"stage": STAGE, "timestamp": doc.get("timestamp"),
         **provenance_block(),
         "tecs_baseline_params_live": p,
         "command_derivation": doc.get("command_derivation"),
         "reference_altitude_m": doc.get("reference_altitude_m"),
         "thresholds": threshold_block(), "reanalyzed_from": path,
         "provenance_blocks_source": (
             "mode / parameter_policy / open_limitations / reference_constants "
             "are STATIC declarations regenerated from this module by "
             "provenance_block(); they are not flight measurements and are not "
             "read from the timeseries file (which does not carry them). "
             "Restored 2026-09-04 for validation finding V-13 - before that "
             "fix a re-analysed artifact silently dropped them.")}
    param_precondition_checks(p, R)
    analyze(R, segs, p, p.get("PTCH_TRIM_DEG", PTCH_TRIM_DEG_EXPECTED))
    vd, fails = verdict(R)
    R["verdict"] = vd
    R["failed_checks"] = fails
    R["overall_result"] = "REANALYZED"
    write_outputs(R, segs, p)
    print("verdict:", vd)
    print("failed_checks:", fails)
    return 0


def provenance_block():
    """Provenance blocks that MUST appear in the result artifact whether it was
    produced by a flight or by --reanalyze (validation finding V-13: the
    re-analysed artifact used to drop `mode`, `parameter_policy`,
    `open_limitations` and `reference_constants`, leaving the authoritative
    machine-readable artifact weaker in provenance than the one it replaced).

    Everything here is a STATIC declaration derived from this module's
    read-only citations - it contains no flight measurement - so regenerating
    it during a re-analysis reproduces exactly what the flight run recorded.
    """
    return {
        "mode": dict(name="FBWB", custom_mode=ARDUPLANE_FBWB_CUSTOM_MODE,
                     evidence="docs/source_of_truth/controls/ardupilot_fbwb_tecs_"
                              "baseline.yaml + the imported cruise-stage module "
                              "docstring. TECS authority is RE-PROVED live here "
                              "(tecs_is_driving_throttle_not_the_stick), not "
                              "inherited on trust."),
        "parameter_policy": ("NO TECS_* parameter, no PID, no PTCH_TRIM_DEG, no "
                             "control-surface mapping, no aero/propulsion/actuator/"
                             "sensor/mass/CG/inertia value is written by this stage. "
                             "config/ardupilot/falcon_v2_sitl.parm is read-only input."),
        "open_limitations": ["PROPULSION_HIGH_J_WINDMILLING",
                             "SETTLE_LIMIT_NOT_FULLY_FALSIFIABLE_P3_AIRSPEED"],
        "open_limitations_note": (
            "The 2026-09-03 flight artifact listed only "
            "PROPULSION_HIGH_J_WINDMILLING. "
            "SETTLE_LIMIT_NOT_FULLY_FALSIFIABLE_P3_AIRSPEED was added "
            "2026-09-04 so the machine-readable artifact surfaces the "
            "limitation that until then existed only as the non-gating "
            "settle_limits.<channel>.limit_exceeds_observable_window flag and "
            "as a yaml entry. Both are declared, NON-GATING: neither is read "
            "by verdict(). Full entries: docs/source_of_truth/controls/"
            "ardupilot_tecs_energy_management.yaml open_limitations."),
        "reference_constants": dict(
            MASS_KG=MASS_KG, S_REF_M2=S_REF_M2, G_WORLD=G_WORLD, G_TECS=G_TECS,
            V_TRIM_REF=V_TRIM_REF, TRIM_THROTTLE_REF=TRIM_THROTTLE_REF,
            ELEV_TRIM_DEG_REF=ELEV_TRIM_DEG_REF,
            PTCH_TRIM_DEG_EXPECTED=PTCH_TRIM_DEG_EXPECTED,
            SURFACE_TRAVEL_LIMIT_DEG=SURFACE_TRAVEL_LIMIT_DEG,
            prior_stage_measurements=PRIOR),
    }


def threshold_block():
    """Every acceptance threshold, recorded in the result JSON. Values marked
    INHERITED are imported from the 2026-09-02 cruise stage module so the two
    stages cannot silently diverge."""
    return dict(
        INHERITED=dict(
            TH_SPEED_MEAN_TOL_MS=TH_SPEED_MEAN_TOL_MS,
            TH_SPEED_STD_MAX_MS=TH_SPEED_STD_MAX_MS,
            TH_SPEED_MIN_MS=TH_SPEED_MIN_MS,
            TH_SPEED_HARD_FLOOR_MS=TH_SPEED_HARD_FLOOR_MS,
            TH_SPEED_SLOPE_MAX_MS_PER_S=TH_SPEED_SLOPE_MAX_MS_PER_S,
            TH_TECS_TARGET_TOL_MS=TH_TECS_TARGET_TOL_MS,
            TH_ALT_SLOPE_MAX_MS=TH_ALT_SLOPE_MAX_MS,
            TH_ALT_SLOPE_TIGHT_MS=TH_ALT_SLOPE_TIGHT_MS,
            TH_ALT_P2P_MAX_M=TH_ALT_P2P_MAX_M,
            TH_SINK_CLOSED_MS=TH_SINK_CLOSED_MS,
            TH_THROTTLE_TOL=TH_THROTTLE_TOL,
            TH_SAT_RUN_MAX_S=TH_SAT_RUN_MAX_S, TH_SAT_MARGIN=TH_SAT_MARGIN,
            TH_TECS_AUTHORITY_MIN_DELTA=TH_TECS_AUTHORITY_MIN_DELTA,
            TH_THROTTLE_MODULATION_MIN=TH_THROTTLE_MODULATION_MIN,
            TH_SURF_HOLD_MAX_DEG=TH_SURF_HOLD_MAX_DEG,
            TH_SURF_FLIGHT_MAX_DEG=TH_SURF_FLIGHT_MAX_DEG,
            TH_LATERAL_SURF_MAX_DEG=TH_LATERAL_SURF_MAX_DEG,
            TH_COORD_THROTTLE_DELTA=TH_COORD_THROTTLE_DELTA,
            TH_COORD_PITCH_DELTA_DEG=TH_COORD_PITCH_DELTA_DEG,
            TH_PITCH_ALPHA_GAMMA_RESID_DEG=TH_PITCH_ALPHA_GAMMA_RESID_DEG,
            TH_GS_VS_AS_MAX_MS=TH_GS_VS_AS_MAX_MS,
            TH_RAMP_DIRECTION_MIN_MS=TH_RAMP_DIRECTION_MIN_MS,
            TH_ALT_STEP_ACHIEVED_FRAC=TH_ALT_STEP_ACHIEVED_FRAC,
            source="tests/gazebo/scripts/test_ardupilot_tecs_cruise_speed_hold.py "
                   "(ARDUPLANE_TECS_AND_CRUISE_SPEED_HOLD_VALIDATION, 2026-09-02)"),
        DERIVED_THIS_STAGE=dict(
            TH_TARGET_STEP_TOL_M=TH_TARGET_STEP_TOL_M,
            TH_TARGET_VS_ACTUAL_TOL_M=TH_TARGET_VS_ACTUAL_TOL_M,
            TH_RESETTLE_TOL_M=TH_RESETTLE_TOL_M,
            TH_RESETTLE_TIGHT_M=TH_RESETTLE_TIGHT_M,
            TH_LEVEL_STEDOT_MAX_W_PER_KG=TH_LEVEL_STEDOT_MAX_W_PER_KG,
            TH_RAMP_STEDOT_MIN_W_PER_KG=TH_RAMP_STEDOT_MIN_W_PER_KG,
            TH_KINETIC_FRACTION_MAX=TH_KINETIC_FRACTION_MAX,
            TH_STE_ROUNDTRIP_MAX_J_PER_KG=TH_STE_ROUNDTRIP_MAX_J_PER_KG,
            TH_DESCENT_SPEED_OVERSHOOT_MAX_MS=TH_DESCENT_SPEED_OVERSHOOT_MAX_MS,
            TH_SETTLE_BAND_M=TH_SETTLE_BAND_M,
            TH_SETTLE_SPEED_BAND_MS=TH_SETTLE_SPEED_BAND_MS,
            TH_SETTLE_TIME_MAX_S=dict(
                value=None, status='WITHDRAWN_2026-09-03',
                was=25.0,
                cause='validation finding V-1 - 5 x TECS_TIME_CONST bounds a '
                      'FIRST-order loop, not the decay envelope of the measured '
                      'SECOND-order longitudinal energy mode, and it ignored the '
                      'uncontrolled initial excursion A0',
                replaced_by='TH_SETTLE_TAU_MARGIN_K + per-phase derived limit'),
            TH_SETTLE_TAU_MARGIN_K=dict(
                value=TH_SETTLE_TAU_MARGIN_K, units='-',
                criterion='t_limit_c = t_peak_c + T/2 + K*TAU_REF*ln(A0_c/B_c) '
                          'per channel; equivalently tau_implied_c <= K*TAU_REF',
                TAU_REF='V*(L/D)/g, free-airframe Lanchester phugoid envelope '
                        'time constant from the phase own post-transient trim '
                        'state (ASSUMPTION PHUGOID_REFERENCE_IS_LANCHESTER)',
                K_justification='no margin. Any K>1 needs a Falcon V2 '
                                'handling-qualities basis, which is DATA_REQUIRED'),
            TH_DECAY_RATIO_MAX=TH_DECAY_RATIO_MAX,
            TH_SURF_LIMIT_MARGIN_DEG=TH_SURF_LIMIT_MARGIN_DEG,
            TH_SURF_MAX_ABS_DEG=TH_SURF_MAX_ABS_DEG,
            TH_PROPULSIVE_POWER_RESID_W_PER_KG=TH_PROPULSIVE_POWER_RESID_W_PER_KG,
            provenance="Each derived from an inherited threshold or a documented "
                       "physical quantity - see the comment block above each "
                       "constant in this file and "
                       "docs/source_of_truth/controls/ardupilot_tecs_energy_management.yaml"),
    )


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--reanalyze":
        return reanalyze(sys.argv[2])

    R = {"stage": STAGE,
         "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         **provenance_block(),
         "thresholds": threshold_block()}

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

    p = dump_params(mav, R)
    param_precondition_checks(p, R)
    # The full PARAM_REQUEST_LIST dump takes tens of seconds during which no
    # RC_CHANNELS_OVERRIDE is published; re-confirm (and if necessary restore)
    # the armed state rather than silently proceeding disarmed.
    if not base.is_armed(mav):
        base.arm(mav)
        R["rearmed_after_param_dump"] = base.is_armed(mav)
        print("re-armed after param dump:", R["rearmed_after_param_dump"])
        if not R["rearmed_after_param_dump"]:
            return finish_fail(R, "rearm_after_param_dump", mav)
    required = ["AIRSPEED_MIN", "AIRSPEED_MAX", "RC3_MIN", "RC3_MAX", "RC3_DZ",
                "RC3_REVERSED", "RC2_MIN", "RC2_MAX", "RC2_TRIM", "THR_MIN",
                "THR_MAX", "TECS_SPDWEIGHT"]
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
        an = R["analysis"]
        try:
            print("-" * 78)
            for w in ("P1_cruise_hold", "P3_settle_hold", "P5_resettle_hold"):
                d_ = an[w]
                print(f"{w:18s} V {d_['airspeed_vfr_hud_ms']['mean']:.3f}+-"
                      f"{d_['airspeed_vfr_hud_ms']['std']:.3f} m/s | "
                      f"alt {d_['altitude_gz_m']['mean']:.3f} m | "
                      f"vz {d_['vertical_speed_regression_ms']:+.4f} m/s | "
                      f"thr {d_['throttle_actual']['mean']:.4f} | "
                      f"STEdot {d_['energy']['STEdot_W_per_kg']:+.3f} W/kg")
            st = an["altitude_step"]
            print(f"achieved step (actual)   : climb {st['achieved_climb_m']:+.3f} m, "
                  f"descent {st['achieved_descent_m']:+.3f} m, "
                  f"roundtrip {st['roundtrip_residual_m']:+.3f} m")
            print(f"achieved step (AP target): climb {st['ap_target_climb_step_m']:+.3f} m, "
                  f"descent {st['ap_target_descent_step_m']:+.3f} m, "
                  f"roundtrip {st['ap_target_roundtrip_residual_m']:+.3f} m")
            es = an["energy_management"]["summary"]
            print(f"energy climb   : STEdot {es['climb_STEdot_W_per_kg']:+.3f} "
                  f"= SPEdot {es['climb_SPEdot_W_per_kg']:+.3f} + SKEdot "
                  f"{es['climb_SKEdot_W_per_kg']:+.3f} W/kg | SEBdot "
                  f"{es['climb_SEBdot_W_per_kg']:+.3f}")
            print(f"energy descent : STEdot {es['descent_STEdot_W_per_kg']:+.3f} "
                  f"= SPEdot {es['descent_SPEdot_W_per_kg']:+.3f} + SKEdot "
                  f"{es['descent_SKEdot_W_per_kg']:+.3f} W/kg | SEBdot "
                  f"{es['descent_SEBdot_W_per_kg']:+.3f}")
            co = an["coordination"]
            print(f"throttle L/C/D : {co['level_throttle']} / {co['climb_throttle']} / "
                  f"{co['descent_throttle']}")
            print(f"pitch    L/C/D : {co['level_pitch_deg']} / {co['climb_pitch_deg']} / "
                  f"{co['descent_pitch_deg']} deg (physical, nose-up +)")
            for ph in ("P3_settle", "P5_resettle"):
                sa = an["settling"][ph]
                lim = sa.get("settle_limits") or {}
                print(f"{ph:12s} settling {sa.get('settling_time_s')} s "
                      f"(binding {sa.get('settling_binding_channel')}; alt "
                      f"{sa.get('settling_time_altitude_s')} s, spd "
                      f"{sa.get('settling_time_airspeed_s')} s) | decay ratios "
                      f"{sa.get('decay_ratios')}")
                print(f"{'':12s} TAU_REF {sa.get('tau_ref_s')} s "
                      f"(V*(L/D)/g), T_used {sa.get('settle_period_used_s')} s")
                for cn in ("altitude", "airspeed"):
                    d = lim.get(cn) or {}
                    print(f"{'':12s}   {cn:8s} A0 {d.get('A0')} "
                          f"t_peak {d.get('t_peak_s')} s -> limit {d.get('limit_s')} s "
                          f"| tau_implied {d.get('tau_implied_s')} s vs "
                          f"{d.get('tau_limit_s')} s -> {d.get('ok')}")
            hj = an["whole_flight"]["high_j"]
            print(f"HIGH-J (OPEN_LIMITATION): interpClamped "
                  f"{hj['interp_clamped_samples']}/{hj['motor_samples']} motor-samples "
                  f"({hj['interp_clamped_fraction']}), zero-thrust "
                  f"{hj['zero_thrust_samples']}")
            print(f"elevator max |deg| (flight): {an['whole_flight']['elevator_max_abs_deg']} "
                  f"(travel limit {SURFACE_TRAVEL_LIMIT_DEG})")
        except Exception as exc:      # summary print only - JSON is authoritative
            print("summary print failed:", exc)
        print(f"VERDICT: {vd}  failed_checks={fails}")
        print("-" * 78)
    else:
        R["overall_result"] = "FLIGHT_ABORTED"
        R["verdict"] = "TECS_CLIMB_DESCENT_ENERGY_FAILED"
        print("FLIGHT ABORTED:", R.get("flight_result"))

    write_outputs(R, segs, p)
    print("RESULT:", R["overall_result"], R.get("verdict"), "->", OUT_JSON)
    print("TIMESERIES:", OUT_TS)
    print("PER-SAMPLE TRACE:", OUT_TRACE)
    mav.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

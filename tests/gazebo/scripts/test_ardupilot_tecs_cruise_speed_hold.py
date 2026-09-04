#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_TECS_AND_CRUISE_SPEED_HOLD_VALIDATION
(controls-integration, 2026-09-02).

GOAL
----
Verify that ArduPlane TECS can hold Falcon V2 at ~18 m/s cruise airspeed AND
hold altitude, in a mode where TECS is GENUINELY the active speed/height
controller (not a pass-through, not a navigation controller).

MODE: FBWB (MAVLink custom_mode 6, ArduPlane/mode.h:45).
  * TECS drives BOTH throttle and pitch:
      ArduPlane/Plane.cpp:635   should_run_tecs = control_mode->does_auto_throttle()
      ArduPlane/mode.h:654      ModeFBWB::does_auto_throttle() -> true
      ArduPlane/Plane.cpp:669   TECS_controller.update_pitch_throttle(...)
                                (the ONLY call site in the vehicle)
      ArduPlane/navigation.cpp:450-451  update_fbwb_speed_height() ends with
                                calc_throttle(); calc_nav_pitch();
      ArduPlane/Attitude.cpp:510-511    throttle  <- TECS get_throttle_demand()
      ArduPlane/Attitude.cpp:637-638    nav_pitch <- TECS get_pitch_demand()
  * NO navigation controller: ModeFBWB does not override does_auto_navigation()
    (base returns false, ArduPlane/mode.h:134), declares no navigate(), and sets
    nav_roll_cd directly from the roll stick (ArduPlane/mode_fbwb.cpp:19).
  * NO stick mixing contamination: ArduPlane/Attitude.cpp:293-301 returns early
    for mode_fbwb, so STICK_MIXING (default 1) cannot inject stick into demand.
  * CRUISE was REJECTED: ArduPlane/mode_cruise.cpp:75-88 auto-locks heading after
    0.5 s of zero roll/rudder input and then calls
    nav_controller->update_waypoint() - i.e. it becomes an L1-navigating mode,
    which this stage must exclude.

Full mode-selection evidence, command mapping and the TECS baseline parameter
table with per-value provenance:
    docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml

COMMAND MAPPING USED HERE (ArduPlane's own formulas, re-derived at RUNTIME from
live-read parameters - nothing is hardcoded):
  * Target AIRSPEED comes from the THROTTLE stick in FBWB:
        target_airspeed_cm = (AIRSPEED_MAX-AIRSPEED_MIN)*get_throttle_input()
                             + AIRSPEED_MIN*100          [navigation.cpp:187-189]
        get_throttle_input() = RC_Channel::get_control_in() WITH dead zone
        control_in = 100*(pwm-(RC3_MIN+RC3_DZ))/(RC3_MAX-(RC3_MIN+RC3_DZ))
                                                 [RC_Channel.cpp:388-402]
    -> the test INVERTS this for V_target = AIRSPEED_CRUISE = 18 m/s, then
       CROSS-CHECKS the achieved demand against ArduPlane's own reported TECS
       target airspeed = VFR_HUD.airspeed + NAV_CONTROLLER_OUTPUT.aspd_error/100
       [GCS_MAVLink_Plane.cpp:241 + navigation.cpp:297].
    -> the throttle stick does NOT set throttle in FBWB; that is what makes the
       TECS_IS_DRIVING_THROTTLE check below decisive.
  * Target ALTITUDE is ramped by the PITCH stick at
        climb_rate = FBWB_CLIMB_RATE * elevator_input   [navigation.cpp:427]
    and LOCKS to the current altitude when the stick returns through zero
    (set_target_altitude_current(), navigation.cpp:421-426).

SIGN CONVENTION - NOT ASSUMED, TEST-VERIFIED (controls-integration rule):
  RC2 > RC2_TRIM is EXPECTED to command climb (FBWB_ELEV_REV read live, expected
  0). This test does NOT treat that as known-good: `fbwb_pitch_stick_direction_
  correct` is a GATING acceptance check requiring measured d(altitude)/dt > 0
  during the up-stick ramp and < 0 during the down-stick ramp. A failure is
  reported as a control-direction finding, never "adapted" away.

PITCH TELEMETRY CAVEAT (same as the FBWA stage - do NOT misread):
  * GCS ATTITUDE.pitch = (true_pitch - PTCH_TRIM_DEG)   [GCS_MAVLink_Plane.cpp:139]
  * NAV_CONTROLLER_OUTPUT.nav_pitch = RAW TECS pitch demand, WITHOUT
    PTCH_TRIM_DEG. The physically demanded attitude is
        pitch_demand_phys = nav_pitch + PTCH_TRIM_DEG   [Attitude.cpp:244]
    TECS does NOT add PTCH_TRIM_DEG itself (AP_TECS.h:66-68 returns raw
    _pitch_dem; the pitch_trim argument at Plane.cpp:677 is only used by
    AP_TECS.cpp:919-940 _update_throttle_without_airspeed(), a path NOT taken
    because ARSPD_USE=1). This is a feed-forward, not a double count.
  * TRUE physical pitch is therefore taken from Gazebo ground truth. gz Euler
    pitch is nose-DOWN-positive in this FLU world, so physical nose-up pitch =
    -(gz euler pitch), exactly as in the FBWA stage.

AIRSPEED PROVENANCE (important for validation) - UPDATED 2026-09-02, stage
SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION. THE PARAGRAPH THAT WAS HERE
IS NOW OBSOLETE; it described the pre-fix behaviour and must not be quoted:
  * WAS: ArduPilotPlugin sent no `airspeed` key, so SITL took the else-branch at
    SIM_JSON.cpp:445-455, ZEROED wind_ef, and derived airspeed from the Gazebo
    GROUND velocity. FalconV2Pitot was not in the loop.
  * IS NOW: model/model.sdf wires <airspeed_topic> (FalconV2Pitot, EAS) and
    <wind_topic> (FalconV2Wind, world-ENU airmass velocity) into
    ArduPilotPlugin, so the FDM packet carries the OFFICIAL SIM_JSON `airspeed`
    and `velocity_wind` keys, DataKey::AIRSPEED is set, the wind_ef.zero()
    branch is NOT taken, and SIM_JSON.cpp:456-458 assigns the real wind.
    ARSPD_TYPE=100 then turns that EAS into a differential pressure
    (AP_HAL_SITL/sitl_airspeed.cpp:30-66) which AP_Airspeed_SITL reads back.
  * This run is still a ZERO-WIND run, but now BY DEFAULT (FalconV2Wind's
    <steady_wind_mps>0 0 0</steady_wind_mps>) rather than by ArduPlane being
    wind-blind. Non-zero-wind acceptance is covered by
    test_ardupilot_airspeed_wind_acceptance.py.
  Full record: docs/source_of_truth/autopilot/SITL_ATMOSPHERE_AND_AIRSPEED.md.

SCOPE / HARD CONSTRAINTS OBSERVED
  * BASELINE ONLY. No TECS_* parameter is set anywhere - falcon_v2_sitl.parm is
    read-only input and is NOT modified by this stage. TECS therefore runs on
    ArduPlane 4.8.0-dev compiled firmware defaults; the test DUMPS the live
    effective values into its result JSON.
  * No physics / aero / propulsion / actuator / sensor / SDF / plugin change.
  * No PID change, no AUTOTUNE, no LOITER/AUTO/RTL/waypoint navigation.
  * Nothing is tuned to make this pass. The test is written to FAIL honestly.

USAGE (a FRESH gz sim + gdb-wrapped arduplane pair MUST already be running -
see tests/gazebo/scripts/run_ardupilot_tecs_cruise_speed_hold.sh):
    python3 test_ardupilot_tecs_cruise_speed_hold.py
    python3 test_ardupilot_tecs_cruise_speed_hold.py --reanalyze <timeseries.json>
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
from pymavlink import mavutil  # noqa: E402

import test_ardupilot_basic_closed_loop_flight as base  # noqa: E402
import test_ardupilot_basic_closed_loop_flight_campaign as campaign  # noqa: E402
# Proven statistics / param-read helpers, imported VERBATIM (not copy-pasted)
# from the FBWA level-pitch-reference stage so the analysis maths is identical
# and directly comparable between the two stages.
import test_ardupilot_fbwa_level_pitch_reference_correction as fbwa  # noqa: E402
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

STAGE = "ARDUPLANE_TECS_AND_CRUISE_SPEED_HOLD_VALIDATION"
OUT_JSON = os.path.join(base.RESULTS_DIR, "ardupilot_tecs_cruise_speed_hold_result.json")
OUT_TS = os.path.join(base.RESULTS_DIR, "ardupilot_tecs_cruise_speed_hold_timeseries.json")

ARDUPLANE_FBWB_CUSTOM_MODE = 6      # ArduPlane/mode.h:45  FLY_BY_WIRE_B

# =============================================================================
# READ-ONLY CITATIONS (never modified, never fed back into any physics path)
# =============================================================================
MASS_KG = 6.000                     # CLAUDE.md
S_REF_M2 = 0.4514                   # CLAUDE.md wing area
G = 9.81
# Measured pure-Gazebo straight-and-level equilibrium C.2, cited from
# docs/test_results/2026-08-28_ardupilot_longitudinal_equilibrium_and_sink_
# root_cause_validation.md sec 6.2 (via test_ardupilot_basic_closed_loop_flight):
V_TRIM_REF = base.V_TRIM            # 18.162 m/s
TRIM_THROTTLE_REF = base.TRIM_THROTTLE  # 0.4957
ELEV_TRIM_DEG_REF = math.degrees(base.ELEVATOR_THETA_RAD)  # +4.092 deg
PTCH_TRIM_DEG_EXPECTED = 2.49       # config/ardupilot/falcon_v2_sitl.parm
# Residual sink left by the FBWA level-pitch-reference stage (attitude hold
# only, no height loop): docs/test_results/2026-08-29 FBWA stage, 0.078 m/s.
FBWA_RESIDUAL_SINK_MS = 0.078

# =============================================================================
# SEGMENT PLAN + DURATION RATIONALE
# -----------------------------------------------------------------------------
# Durations are taken from THIS airframe's own longitudinal time constants and
# from TECS's own configured time constants - not picked round-number-style.
#
#   phugoid period      T_ph ~ pi*sqrt(2)*V/g = 4.443*18.162/9.81 = 8.2 s
#                       (classical approximation; the FBWA stage independently
#                       observed ~9 s with zeta ~ 0.2). With zeta = 0.2 the
#                       envelope decay constant is T_ph/(2*pi*zeta) = 6.5 s.
#   TECS_TIME_CONST     5.0 s  (firmware default, AP_TECS.cpp:43)
#   TECS_HDEM_TCONST    3.0 s  (firmware default, AP_TECS.cpp:292)
#   THR_SLEWRATE        100 %/s -> <= 1.0 s for a full-range throttle move
#
# SEG_A_TRANSIENT_S = 12 s   ~= 1.5 phugoid periods, 2.4 x TECS_TIME_CONST,
#                              4 x TECS_HDEM_TCONST, >> throttle slew time.
#                              Covers mode entry + throttle unsuppression
#                              (servos.cpp:135-139 clears throttle_suppressed
#                              once |relative_altitude| >= 10 m) + TECS filter
#                              initialisation.
# SEG_A_DURATION_S  = 45 s   -> 33 s of analysed steady data = 4 phugoid
#                              periods = 6.6 x TECS_TIME_CONST = 5 envelope
#                              decay constants. Enough to separate "bounded"
#                              from "slowly growing" without over-extending.
# SEG_B_RAMP_MAX_S  = 20 s   hard cap. At FBWB_CLIMB_RATE=2.0 m/s the +10 m
#                              ramp needs ~5 s; the cap only bounds the case
#                              where the aircraft cannot achieve the demand
#                              (which is then reported honestly, not retried).
# SEG_B_HOLD_S      = 35 s   -> 25 s analysed = 3 phugoid periods /
#                              5 x TECS_TIME_CONST after the step: enough to
#                              show settling AND absence of growth.
# SEG_C_RAMP_MAX_S  = 20 s   mirror of B.
# SEG_C_HOLD_S      = 30 s   -> 20 s analysed. Shorter than B on purpose: C
#                              only has to close the picture (descent-direction
#                              sign + re-capture of the original altitude), the
#                              stability question is already answered by A/B.
# Total flight time <= 45+20+35+20+30 = 150 s.
# =============================================================================
SEG_A_DURATION_S = 45.0
SEG_A_TRANSIENT_S = 12.0
SEG_B_RAMP_MAX_S = 20.0
SEG_B_HOLD_S = 35.0
SEG_C_RAMP_MAX_S = 20.0
SEG_C_HOLD_S = 30.0
HOLD_TRANSIENT_S = 10.0     # post-step transient cutoff for B/C hold windows
                            # (2 x TECS_TIME_CONST, ~1.2 phugoid periods)

ALT_STEP_M = 10.0           # task-specified small altitude step
V_TARGET_MS = 18.0          # = AIRSPEED_CRUISE in config/ardupilot/falcon_v2_sitl.parm

# Flight-safety envelope for this test (abort, preserving all samples so far).
ALT_FLOOR_M = 40.0          # start alt is 90 m; 40 m still leaves ample margin
ALT_CEILING_M = 220.0       # start 90 m + 10 m step; 220 m is a runaway-climb trip
ATT_ABORT_DEG = 60.0

# =============================================================================
# ACCEPTANCE THRESHOLDS - every value justified here, in the file that uses it.
# =============================================================================
# Speed hold. 0.5 m/s = 2.8 % of 18 m/s; identical band to the FBWA stage's own
# airspeed criterion, so the two stages stay directly comparable. ASSUMPTION:
# no manufacturer cruise-speed-hold specification exists (DATA_REQUIRED).
TH_SPEED_MEAN_TOL_MS = 0.5
TH_SPEED_STD_MAX_MS = 0.5        # bounded, non-oscillatory
TH_SPEED_MIN_MS = 16.0           # = AIRSPEED_MIN (falcon_v2_sitl.parm)
# ArduPlane's OWN underspeed trigger: _TAS_state < 0.9*_TASmin (AP_TECS.cpp:660)
TH_SPEED_HARD_FLOOR_MS = 0.9 * TH_SPEED_MIN_MS      # 14.4 m/s
# TECS demand read-back must match the RC3->airspeed mapping we derived.
TH_TECS_TARGET_TOL_MS = 0.4      # 1 PWM us ~= 0.0156 m/s of demand; 0.4 m/s is
                                 # far above quantisation and far below the
                                 # 2 m/s AIRSPEED_MIN->CRUISE span.

# Altitude hold. FBWA stage accepted |vz| <= 0.10 m/s (preferred <= 0.05); reuse
# that so the stages compare directly. 0.10 m/s over the 33 s analysed window is
# 3.3 m of drift.
TH_ALT_SLOPE_MAX_MS = 0.10
TH_ALT_P2P_MAX_M = 5.0           # ASSUMPTION: no altitude-hold spec exists
                                 # (DATA_REQUIRED). 5 m ~= 2.4 wingspans and is
                                 # half the commanded 10 m step, so a failure is
                                 # unambiguous rather than marginal.
# "residual FBWA sink closed": TECS carries an explicit height-error integrator
# (TECS_INTEG_GAIN 0.3), so it must be at least as good as the attitude-only
# FBWA result. Hence the bar is exactly the FBWA residual.
TH_SINK_CLOSED_MS = FBWA_RESIDUAL_SINK_MS   # 0.078 m/s
TH_ALT_SLOPE_TIGHT_MS = 0.02     # preferred (non-gating)

# --- MINOR-3 fix (gazebo-testing, 2026-09-02): three literals previously
# inlined in verdict() are named here and recorded in the result JSON's
# `thresholds` block, per CLAUDE.md's no-magic-numbers rule. VALUES ARE
# UNCHANGED from the validated baseline run - this is a traceability fix
# only, and re-running verdict() on the baseline timeseries reproduces it
# check-for-check.
# Speed must not DIVERGE upward over the hold window. 0.02 m/s per second over
# the 33 s window is 0.66 m of speed drift, i.e. well inside the 0.5 m/s mean
# band that the same window must already satisfy - so this is a trend test, not
# a second amplitude test.
TH_SPEED_SLOPE_MAX_MS_PER_S = 0.02
# An altitude step counts as "achieved" at 70 % of the commanded step. The
# ramp is stick-driven and terminated by a stop condition, so the exact final
# offset is not commanded; 0.7 separates "the step happened" from "it did not"
# without asserting a tracking accuracy this stage does not test.
TH_ALT_STEP_ACHIEVED_FRAC = 0.7
# Control-DIRECTION bar for the ramps. 0.2 m/s of mean vertical speed is ~2x
# the 0.10 m/s altitude-hold slope limit, so a ramp that clears it is
# unambiguously commanding a climb/descent rather than drifting.
TH_RAMP_DIRECTION_MIN_MS = 0.2

# Throttle. Physical-plausibility band around the MEASURED trim throttle*
# 0.4957 (at V* 18.162 m/s). TECS holds 18.0 m/s, slightly slower, so an exact
# match is not expected and is explicitly NOT forced. 0.05 (5 percentage points)
# is a plausibility window, not a tuning target. ASSUMPTION.
TH_THROTTLE_TOL = 0.05
# Sustained-saturation limit. THR_SLEWRATE=100 %/s means a legitimate full-range
# transient lasts <= 1.0 s; 2.0 s is also 0.4 x TECS_TIME_CONST. Anything longer
# is a genuine authority/controller problem, not a transient.
TH_SAT_RUN_MAX_S = 2.0
TH_SAT_MARGIN = 0.01             # within 1 % of THR_MIN / THR_MAX counts as saturated
# TECS must demonstrably be the throttle authority: in FBWB the RC3 stick sets
# AIRSPEED, not throttle, so the actual motor throttle must differ clearly from
# the manual-passthrough value the same PWM would give in FBWA/MANUAL.
TH_TECS_AUTHORITY_MIN_DELTA = 0.10
TH_THROTTLE_MODULATION_MIN = 0.05  # throttle range over the whole flight

# Control surfaces. Trim elevator is +4.092 deg, mechanical travel is +/-45 deg
# (ARDUPLANE_CONTROL_SURFACE_TRAVEL_SCALING stage). +/-10 deg in the hold
# windows leaves ~6 deg of manoeuvring authority around trim; +/-15 deg over the
# whole flight (incl. the commanded 2 m/s ramps) is 33 % of available travel.
TH_SURF_HOLD_MAX_DEG = 10.0
TH_SURF_FLIGHT_MAX_DEG = 15.0
TH_LATERAL_SURF_MAX_DEG = 10.0   # no lateral command is given; report-and-bound

# Pitch/throttle coordination. These test the SIGN of the energy split, not its
# magnitude, so the thresholds are deliberately small/conservative. Physical
# reference: a 2 m/s climb at 6.000 kg needs m*g*vz = 117.7 W of extra
# mechanical power, and a steady 2 m/s climb at 18 m/s implies a flight-path
# angle asin(2/18) = 6.4 deg - both far above these bars.
TH_COORD_THROTTLE_DELTA = 0.01
TH_COORD_PITCH_DELTA_DEG = 0.5
# Longitudinal kinematic consistency in the hold windows:
#   pitch_physical ~= alpha + gamma,  gamma = asin(vz/V)
TH_PITCH_ALPHA_GAMMA_RESID_DEG = 1.5

# Zero-wind confirmation (the world contains no wind system, and SIM_JSON zeroes
# wind_ef anyway). In a hold window groundspeed and airspeed should agree to
# within EAS/TAS scaling (~0.1 m/s at 90-100 m) plus climb-angle projection.
TH_GS_VS_AS_MAX_MS = 1.0

# =============================================================================
# Parameters dumped live (the "TECS baseline params" deliverable). Expected
# values + per-value provenance:
#   docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml
# =============================================================================
PARAMS_OF_INTEREST = [
    # --- TECS proper ---
    "TECS_CLMB_MAX", "TECS_SINK_MIN", "TECS_SINK_MAX", "TECS_TIME_CONST",
    "TECS_THR_DAMP", "TECS_PTCH_DAMP", "TECS_INTEG_GAIN", "TECS_VERT_ACC",
    "TECS_SPDWEIGHT", "TECS_HGT_OMEGA", "TECS_SPD_OMEGA", "TECS_RLL2THR",
    "TECS_PITCH_MAX", "TECS_PITCH_MIN", "TECS_HDEM_TCONST", "TECS_OPTIONS",
    "TECS_SYNAIRSPEED", "TECS_PTCH_FF_V0", "TECS_PTCH_FF_K", "TECS_THR_ERATE",
    "TECS_LAND_ARSPD", "TECS_LAND_THR", "TECS_LAND_SPDWGT",
    # --- throttle / speed envelope TECS depends on ---
    "THR_MIN", "THR_MAX", "TRIM_THROTTLE", "THR_SLEWRATE", "THROTTLE_NUDGE",
    "AIRSPEED_MIN", "AIRSPEED_CRUISE", "AIRSPEED_MAX", "AIRSPEED_STALL",
    "ARSPD_TYPE", "ARSPD_USE", "MIN_GROUNDSPEED",
    # --- FBWB mode + attitude limits ---
    "FBWB_CLIMB_RATE", "FBWB_ELEV_REV", "CRUISE_ALT_FLOOR",
    "PTCH_LIM_MAX_DEG", "PTCH_LIM_MIN_DEG", "ROLL_LIMIT_DEG",
    "PTCH_TRIM_DEG", "KFF_THR2PTCH", "STAB_PITCH_DOWN",
    "STICK_MIXING", "FLIGHT_OPTIONS", "TERRAIN_FOLLOW", "FENCE_ENABLE",
    # --- RC calibration needed to invert the command mapping ---
    "RC2_MIN", "RC2_MAX", "RC2_TRIM", "RC2_DZ", "RC2_REVERSED",
    "RC3_MIN", "RC3_MAX", "RC3_TRIM", "RC3_DZ", "RC3_REVERSED",
    # --- PIDs: recorded ONLY to prove they were not touched this stage ---
    "RLL_RATE_P", "RLL_RATE_I", "RLL_RATE_D", "RLL_RATE_FF",
    "PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D", "PTCH_RATE_FF",
    # --- environment guard ---
    "SIM_WIND_SPD", "SIM_WIND_DIR", "SIM_WIND_TURB",
    # --- atmosphere datum guard (see param_precondition_checks) ---
    "SIM_OPOS_LAT", "SIM_OPOS_LNG", "SIM_OPOS_ALT", "SIM_OPOS_HDG",
    "SIM_ARSPD_RND", "SIM_ARSPD_RATIO", "ARSPD_OPTIONS", "AHRS_EKF_TYPE",
    "AHRS_WIND_MAX",
]


# =============================================================================
# ArduPlane command-mapping inversions (formulas cited above; live inputs)
# =============================================================================
def control_in_range_dz(pwm, rc_min, rc_max, dz, reversed_flag):
    """RC_Channel::pwm_to_range_dz(), high_in = 100 (radio.cpp:29
    channel_throttle->set_range(100)). RC_Channel.cpp:388-402."""
    r_in = min(max(pwm, rc_min), rc_max)
    if reversed_flag:
        r_in = rc_max - (r_in - rc_min)
    trim_low = rc_min + dz
    if r_in > trim_low:
        return 100.0 * (r_in - trim_low) / (rc_max - trim_low)
    return 0.0


def control_in_range_no_dz(pwm, rc_min, rc_max, reversed_flag):
    """get_control_in_zero_dz() - the MANUAL/FBWA throttle passthrough path
    (ArduPlane/mode.cpp:318 -> get_adjusted_throttle_input(no_deadzone=True) ->
    get_throttle_input(True), reverse_thrust.cpp:132-149)."""
    return control_in_range_dz(pwm, rc_min, rc_max, 0, reversed_flag)


def achievable_target_airspeed(pwm, p):
    """Forward model of what ArduPlane ACTUALLY demands for a given RC3 PWM.

    FIX (gazebo-testing, 2026-09-02, closes `validation` MINOR-2 of
    docs/validation/2026-09-02_ardupilot_tecs_and_cruise_speed_hold_validation.md):
    the earlier version used the raw FLOAT `control_in`, which was optimistic.
    `RC_Channel::control_in` is an `int16_t` (RC_Channel.h:99/542) and
    RC_Channel.cpp:316 assigns the float returned by pwm_to_range_dz()
    (:388-402) straight into it, so the value is TRUNCATED TOWARD ZERO before
    navigation.cpp:187-189 ever sees it. That quantises the reachable demand to
    a grid of (AIRSPEED_MAX-AIRSPEED_MIN)/100 m/s = 0.12 m/s here, 7.7x coarser
    than the 0.0156 m/s per-microsecond sensitivity.

    Returns (control_in_int, demand_ms).
    """
    a_min, a_max = p["AIRSPEED_MIN"], p["AIRSPEED_MAX"]
    rc_min, rc_max, dz = p["RC3_MIN"], p["RC3_MAX"], p["RC3_DZ"]
    ci_f = control_in_range_dz(pwm, rc_min, rc_max, dz, bool(p["RC3_REVERSED"]))
    ci = int(ci_f)                       # int16_t assignment: truncate toward 0
    # navigation.cpp:187-189 uses integer (airspeed_max - airspeed_min)
    return ci, (int(a_max) - int(a_min)) * ci / 100.0 + a_min


def rc3_pwm_for_target_airspeed(v_target, p):
    """Invert ArduPlane/navigation.cpp:187-189 + RC_Channel.cpp:388-402.
    Returns (pwm_float, achieved_target_airspeed_ms, notes).

    NOTE: `achieved` is now the demand ArduPlane will REALLY hold for the
    PWM this function returns, including the int16_t truncation (see
    achievable_target_airspeed()). The returned PWM itself is DELIBERATELY
    UNCHANGED from the validated baseline run: nudging it to land on the
    next grid point up would change the commanded flight condition and
    therefore the recorded baseline result, which is out of scope for a
    test-reporting fix. The residual command quantisation is reported
    explicitly in R["command_derivation"] instead of being hidden."""
    a_min, a_max = p["AIRSPEED_MIN"], p["AIRSPEED_MAX"]
    rc_min, rc_max, dz = p["RC3_MIN"], p["RC3_MAX"], p["RC3_DZ"]
    span = a_max - a_min
    if span <= 0:
        return None, None, "AIRSPEED_MAX <= AIRSPEED_MIN"
    frac = (v_target - a_min) / span
    frac = min(max(frac, 0.0), 1.0)
    trim_low = rc_min + dz
    pwm = trim_low + frac * (rc_max - trim_low)
    # Evaluate on the PWM that is actually transmitted (an integer microsecond
    # count in RC_CHANNELS_OVERRIDE), not on the unrounded float.
    _ci, achieved = achievable_target_airspeed(int(round(pwm)), p)
    return pwm, achieved, None


# =============================================================================
# live parameter acquisition
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
        "line numbers: docs/source_of_truth/controls/ardupilot_fbwb_tecs_baseline.yaml")
    if missing:
        print("WARNING: params not read:", missing)
    return got


def param_precondition_checks(p, R):
    """Every precondition this test's derivations depend on. Recorded, and any
    failure is surfaced - never silently worked around."""
    def eq(name, val):
        return p.get(name) is not None and abs(p[name] - val) < 1e-6
    chk = {
        "arspd_use_1_tecs_uses_airspeed_path": eq("ARSPD_USE", 1),
        "arspd_type_100_sitl_backend": eq("ARSPD_TYPE", 100),
        "airspeed_min_16": eq("AIRSPEED_MIN", 16),
        "airspeed_cruise_18": eq("AIRSPEED_CRUISE", 18),
        "airspeed_max_28": eq("AIRSPEED_MAX", 28),
        "ptch_trim_deg_2p49": (p.get("PTCH_TRIM_DEG") is not None
                               and abs(p["PTCH_TRIM_DEG"] - PTCH_TRIM_DEG_EXPECTED) < 1e-3),
        "kff_thr2ptch_zero": eq("KFF_THR2PTCH", 0),
        # FLIGHT_OPTIONS must be 0: bits CRUISE_TRIM_AIRSPEED /
        # CRUISE_TRIM_THROTTLE would REPLACE the RC3->airspeed mapping this test
        # inverts (navigation.cpp:162-186), and bit 8 would change pitch
        # reporting (GCS_MAVLink_Plane.cpp:139).
        "flight_options_zero": eq("FLIGHT_OPTIONS", 0),
        "fbwb_elev_rev_zero": eq("FBWB_ELEV_REV", 0),
        "cruise_alt_floor_zero": eq("CRUISE_ALT_FLOOR", 0),
        "min_groundspeed_zero": eq("MIN_GROUNDSPEED", 0),
        "rc3_not_reversed": eq("RC3_REVERSED", 0),
        "rc2_not_reversed": eq("RC2_REVERSED", 0),
        "sim_wind_zero": (p.get("SIM_WIND_SPD") is None or abs(p["SIM_WIND_SPD"]) < 1e-6),
        # ATMOSPHERE DATUM GUARD (gazebo-testing, 2026-09-02, stage
        # SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION Task C). The declared
        # datum is "ArduPlane altitude AMSL == Gazebo world z", both referenced
        # to the world's <elevation>0.0</elevation>
        # (docs/source_of_truth/autopilot/SITL_ATMOSPHERE_AND_AIRSPEED.md sec 1.5).
        # If SIM_OPOS_ALT is ever non-zero - or if a `-O lat,lng,alt,hdg` with
        # lat/lng exactly 0,0 is reintroduced anywhere, which silently forces the
        # 584 m CMAC elevation (SITL_cmdline.cpp:761-766) - then EAS2TAS is wrong
        # and every TAS-derived TECS quantity is wrong with it. This check exists
        # so that regression can never happen silently again. eq() FAILS on a
        # missing parameter, so an unreadable SIM_OPOS_ALT also fails.
        "sim_opos_alt_zero_atmosphere_datum": eq("SIM_OPOS_ALT", 0),
        # PIDs unchanged from the manufacturer initial values
        "pids_unchanged": all([eq("RLL_RATE_P", 0.25), eq("RLL_RATE_I", 0.125),
                               eq("RLL_RATE_D", 0.002), eq("RLL_RATE_FF", 0.125),
                               eq("PTCH_RATE_P", 0.25), eq("PTCH_RATE_I", 0.125),
                               eq("PTCH_RATE_D", 0.002), eq("PTCH_RATE_FF", 0.125)]),
        # No TECS_* value may have been written by anything
        "tecs_at_firmware_defaults": all([
            eq("TECS_CLMB_MAX", 5.0), eq("TECS_SINK_MIN", 2.0), eq("TECS_SINK_MAX", 5.0),
            eq("TECS_TIME_CONST", 5.0), eq("TECS_THR_DAMP", 0.5), eq("TECS_PTCH_DAMP", 0.3),
            eq("TECS_INTEG_GAIN", 0.3), eq("TECS_VERT_ACC", 7.0), eq("TECS_SPDWEIGHT", 1.0),
            eq("TECS_HDEM_TCONST", 3.0), eq("TECS_PITCH_MAX", 15), eq("TECS_PITCH_MIN", 0)]),
    }
    R["param_preconditions"] = chk
    print("param preconditions:", json.dumps(chk, default=str))
    return chk


# =============================================================================
# segment runner (campaign.run_segment + early-stop + tighter altitude envelope)
# =============================================================================
def run_seg(mav, sub, osub, adiag, pdiag, aerodiag, label, duration_s,
            rc1, rc2, rc3, t_flight0, latest_mav, stop_fn=None):
    """Same structure/rates as campaign.run_segment (20 Hz combined telemetry,
    RC re-published every 0.1 s vs RC_OVERRIDE_TIME=3.0 s), with:
      * an optional stop_fn(sample, samples) -> (bool, reason) early exit, used
        for the closed-loop altitude ramps, and
      * an altitude floor/ceiling envelope suited to this 90 m test instead of
        campaign's 5 m floor."""
    samples = []
    aborted = False
    abort_reason = None
    stopped_early = False
    stop_reason = None
    t0 = time.time()
    last_rc = -1.0
    last_sample = -1.0
    while True:
        tnow = time.time() - t0
        if tnow > duration_s:
            break
        campaign.drain_mavlink(mav, latest_mav)
        if tnow - last_rc >= campaign.RC_REFRESH_PERIOD:
            mav.send_rc_override(rc1=int(round(rc1)), rc2=int(round(rc2)),
                                 rc3=int(round(rc3)), rc4=1500, rc5=1000)
            last_rc = tnow
        if tnow - last_sample >= campaign.SAMPLE_PERIOD:
            s = campaign.build_sample(time.time() - t_flight0, latest_mav, sub, osub,
                                      adiag, pdiag, aerodiag)
            s["t_seg"] = tnow
            samples.append(s)
            last_sample = tnow
            att = s["gz"]["att_deg"]
            pos = s["gz"]["pos"]
            bad = None
            if att is not None:
                if not (math.isfinite(att[0]) and math.isfinite(att[1])):
                    bad = "nonfinite_attitude"
                elif abs(att[0]) > ATT_ABORT_DEG or abs(att[1]) > ATT_ABORT_DEG:
                    bad = "attitude_envelope"
            if pos is not None and bad is None:
                if not math.isfinite(pos[2]):
                    bad = "nonfinite_altitude"
                elif pos[2] < ALT_FLOOR_M:
                    bad = "altitude_floor"
                elif pos[2] > ALT_CEILING_M:
                    bad = "altitude_ceiling"
            if bad is not None:
                aborted = True
                abort_reason = dict(reason=bad, sample=s)
                break
            if stop_fn is not None:
                ok, why = stop_fn(s, samples)
                if ok:
                    stopped_early = True
                    stop_reason = why
                    break
        time.sleep(0.005)
    return dict(label=label, duration_s=duration_s, actual_duration_s=time.time() - t0,
                rc1=rc1, rc2=rc2, rc3=rc3, n_samples=len(samples), samples=samples,
                aborted=aborted, abort_reason=abort_reason,
                stopped_early=stopped_early, stop_reason=stop_reason)


def enter_fbwb(mav, rc3_cruise, R):
    """Same mode-switch/confirm pattern as campaign.enter_fbwa (wait for the
    LAST heartbeat carrying the target custom_mode, not the first), targeting
    FBWB=6 and publishing the cruise-airspeed RC3 from the first instant."""
    import select
    mav.m.mav.set_mode_send(mav.m.target_system, base.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            ARDUPLANE_FBWB_CUSTOM_MODE)
    mav.send_rc_override(rc1=1500, rc2=1500, rc3=int(round(rc3_cruise)), rc4=1500, rc5=1000)
    hb = None
    t0 = time.time()
    while time.time() - t0 < 5.0:
        r, _, _ = select.select([mav.m.port], [], [], 0.3)
        if not r:
            continue
        msg = mav.m.recv_match(type="HEARTBEAT", blocking=False)
        if msg is None:
            continue
        hb = msg
        if hb.custom_mode == ARDUPLANE_FBWB_CUSTOM_MODE:
            break
    confirmed = bool(hb and hb.custom_mode == ARDUPLANE_FBWB_CUSTOM_MODE)
    R["fbwb_handoff"] = dict(
        confirmed=confirmed, custom_mode=(hb.custom_mode if hb else None),
        armed=(bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None),
        rc3_cruise_us=int(round(rc3_cruise)))
    return confirmed


# =============================================================================
# per-sample derived quantities
# =============================================================================
def s_alt(s):
    return s["gz"]["pos"][2] if s["gz"]["pos"] is not None else None


def s_pitch_phys(s):
    """physical nose-up-positive pitch from Gazebo ground truth. gz Euler pitch
    is nose-DOWN-positive in this FLU world (same convention as the FBWA
    stage)."""
    a = s["gz"]["att_deg"]
    return (-a[1]) if a is not None else None


def s_pitch_demand_phys(s, ptch_trim_deg):
    """Attitude.cpp:244 - the physically demanded attitude is the raw TECS
    demand PLUS PTCH_TRIM_DEG."""
    np_ = s["mav"]["nav_pitch_deg"]
    return (np_ + ptch_trim_deg) if np_ is not None else None


def s_tecs_target_airspeed(s):
    """VFR_HUD.airspeed + NAV_CONTROLLER_OUTPUT.aspd_error/100.
    aspd_error is airspeed_error*100 (cm/s despite the field name -
    GCS_MAVLink_Plane.cpp:241) and airspeed_error =
    TECS_controller.get_target_airspeed() - airspeed_measured (navigation.cpp:297)."""
    a = s["mav"]["airspeed"]
    e = s["mav"]["nav_aspd_error"]
    if a is None or e is None:
        return None
    return a + e / 100.0


def s_throttle_actual(s):
    """mean of the two live motor throttles from the propulsion diagnostics
    (Gazebo side ground truth, not the MAVLink report)."""
    pr = s["propulsion"]
    if not pr:
        return None
    return 0.5 * (pr["left"]["throttle"] + pr["right"]["throttle"])


def s_surface_deg(s, surf):
    a = s["actuators"]
    if not a:
        return None
    return math.degrees(a[surf]["actual_angle_rad"])


def collect(samples, fn):
    ts, ys = [], []
    for s in samples:
        v = fn(s)
        if v is None or (isinstance(v, float) and not math.isfinite(v)):
            continue
        ts.append(s["t"])
        ys.append(v)
    return ts, ys


def longest_run_seconds(samples, pred):
    """longest contiguous wall-clock run of samples satisfying pred()."""
    best = 0.0
    run_start = None
    prev_t = None
    for s in samples:
        ok = pred(s)
        if ok:
            if run_start is None:
                run_start = s["t"]
            prev_t = s["t"]
        else:
            if run_start is not None:
                best = max(best, prev_t - run_start)
            run_start = None
    if run_start is not None and prev_t is not None:
        best = max(best, prev_t - run_start)
    return best


# =============================================================================
# window analysis
# =============================================================================
def analyze_window(samples, label, p, ptch_trim_deg):
    out = {"label": label, "n": len(samples)}
    if len(samples) < 4:
        out["insufficient_samples"] = True
        return out
    out["t_start"] = samples[0]["t"]
    out["t_end"] = samples[-1]["t"]
    out["duration_s"] = samples[-1]["t"] - samples[0]["t"]

    # ---- airspeed (3 independent sources) ----
    ts, v_vfr = collect(samples, lambda s: s["mav"]["airspeed"])
    out["airspeed_vfr_hud_ms"] = series_report(ts, v_vfr, "airspeed_vfr_hud") if v_vfr else None
    _, v_aero = collect(samples, lambda s: s["aero"]["V"] if s["aero"] else None)
    out["airspeed_aero_diag_V_ms"] = minmaxmean(v_aero)
    _, gs = collect(samples, lambda s: s["mav"]["groundspeed"])
    out["groundspeed_ms"] = minmaxmean(gs)
    _, vtgt = collect(samples, s_tecs_target_airspeed)
    out["tecs_target_airspeed_ms"] = minmaxmean(vtgt)
    if v_vfr and gs and len(v_vfr) == len(gs):
        out["groundspeed_minus_airspeed_max_abs_ms"] = max(abs(a - b) for a, b in zip(gs, v_vfr))
    else:
        out["groundspeed_minus_airspeed_max_abs_ms"] = None

    # ---- altitude / vertical speed (gz ground truth + 2 cross-checks) ----
    ts_z, zs = collect(samples, s_alt)
    if len(zs) >= 2:
        slope, _ = linreg(ts_z, zs)
        out["altitude_gz_m"] = series_report(ts_z, zs, "altitude_gz")
        out["vertical_speed_regression_ms"] = slope
        out["vertical_speed_endpoint_ms"] = (zs[-1] - zs[0]) / (ts_z[-1] - ts_z[0])
        out["altitude_p2p_m"] = max(zs) - min(zs)
    _, climb = collect(samples, lambda s: s["mav"]["climb"])
    out["vfr_hud_climb_ms"] = minmaxmean(climb)
    _, ralt = collect(samples, lambda s: s["mav"]["relative_alt_m"])
    out["mav_relative_alt_m"] = minmaxmean(ralt)
    _, aerr = collect(samples, lambda s: s["mav"]["nav_alt_error"])
    # COMMENT-ONLY CORRECTION (controls-integration, 2026-09-03, stage
    # ARDUPLANE_TECS_CLIMB_DESCENT_AND_ENERGY_VALIDATION). The comment that was
    # here said "1 m resolution (int16), coarse". That was WRONG: in
    # NAV_CONTROLLER_OUTPUT only nav_bearing/target_bearing are int16;
    # alt_error is a MAVLink float in metres carrying the underlying int32
    # centimetre value (~0.01 m resolution). No code or threshold is changed by
    # this edit and no recorded result is affected. See
    # docs/source_of_truth/controls/ardupilot_tecs_energy_management.yaml
    # section target_altitude_readback.
    out["nav_alt_error_m"] = minmaxmean(aerr)   # float, metres, ~0.01 m resolution

    # ---- pitch: physical (gz truth), demand, MAVLink-reported, rate ----
    ts_p, pitch = collect(samples, s_pitch_phys)
    out["pitch_physical_noseup_deg"] = series_report(ts_p, pitch, "pitch_physical") if pitch else None
    _, pdem = collect(samples, lambda s: s_pitch_demand_phys(s, ptch_trim_deg))
    out["pitch_demand_physical_deg"] = minmaxmean(pdem)
    _, nav_p = collect(samples, lambda s: s["mav"]["nav_pitch_deg"])
    out["nav_pitch_raw_tecs_demand_deg"] = minmaxmean(nav_p)
    _, pmav = collect(samples, lambda s: s["mav"]["att_pitch_deg"])
    out["pitch_mav_reported_deg"] = minmaxmean(pmav)
    _, q = collect(samples, lambda s: s["gz"]["av_body_deg"][1] if s["gz"]["av_body_deg"] else None)
    out["pitch_rate_q_gz_deg_s"] = minmaxmean(q)

    # ---- alpha / flight path consistency:  pitch ~= alpha + gamma ----
    _, alpha = collect(samples, lambda s: math.degrees(s["aero"]["alpha"]) if s["aero"] else None)
    out["alpha_aero_diag_deg"] = minmaxmean(alpha)
    resid = []
    for s in samples:
        pp = s_pitch_phys(s)
        if pp is None or not s["aero"]:
            continue
        v = s["aero"]["V"]
        vz = s["mav"]["climb"]
        if v is None or vz is None or v <= 1e-3:
            continue
        gamma = math.degrees(math.asin(max(-1.0, min(1.0, vz / v))))
        resid.append(pp - (math.degrees(s["aero"]["alpha"]) + gamma))
    out["pitch_minus_alpha_minus_gamma_deg"] = minmaxmean(resid)

    # ---- throttle (Gazebo-side ground truth + MAVLink report) ----
    ts_t, thr = collect(samples, s_throttle_actual)
    out["throttle_actual"] = series_report(ts_t, thr, "throttle_actual") if thr else None
    _, thrL = collect(samples, lambda s: s["propulsion"]["left"]["throttle"] if s["propulsion"] else None)
    _, thrR = collect(samples, lambda s: s["propulsion"]["right"]["throttle"] if s["propulsion"] else None)
    out["throttle_L"] = minmaxmean(thrL)
    out["throttle_R"] = minmaxmean(thrR)
    out["throttle_LR_asymmetry_max"] = (max(abs(a - b) for a, b in zip(thrL, thrR))
                                        if thrL and thrR and len(thrL) == len(thrR) else None)
    _, thr_vfr = collect(samples, lambda s: s["mav"]["throttle_pct"])
    out["throttle_vfr_hud_pct"] = minmaxmean(thr_vfr)

    # ---- motors: RPM + thrust ----
    for side in ("left", "right"):
        _, rpm = collect(samples, lambda s, k=side: s["propulsion"][k]["rpm"] if s["propulsion"] else None)
        _, thn = collect(samples, lambda s, k=side: s["propulsion"][k]["thrust_N"] if s["propulsion"] else None)
        out[f"motor_{side}_rpm"] = minmaxmean(rpm)
        out[f"motor_{side}_thrust_N"] = minmaxmean(thn)
    _, ttot = collect(samples, lambda s: (s["propulsion"]["left"]["thrust_N"]
                                          + s["propulsion"]["right"]["thrust_N"]) if s["propulsion"] else None)
    out["thrust_total_N"] = minmaxmean(ttot)
    _, drag = collect(samples, lambda s: (s["aero"]["qbar"] * S_REF_M2 * s["aero"]["CD"]) if s["aero"] else None)
    out["drag_qbar_S_CD_N"] = minmaxmean(drag)
    if ttot and drag:
        out["thrust_minus_drag_N"] = mean(ttot) - mean(drag)
    _, lift = collect(samples, lambda s: (s["aero"]["qbar"] * S_REF_M2 * s["aero"]["CL"]) if s["aero"] else None)
    out["lift_over_weight"] = (mean(lift) / (MASS_KG * G)) if lift else None

    # ---- specific total energy rate (energy split diagnostic) ----
    # d/dt(E/m) = g*vz + V*dV/dt
    if v_vfr and len(v_vfr) >= 2 and climb:
        dv, _ = linreg(ts, v_vfr)
        out["specific_energy_rate_W_per_kg"] = G * mean(climb) + mean(v_vfr) * dv
        out["dV_dt_ms2"] = dv

    # ---- control surfaces ----
    surf = {}
    for name in actuator_lib.SURFACES:
        _, cmd = collect(samples, lambda s, n=name: math.degrees(s["actuators"][n]["cmd_rad"]) if s["actuators"] else None)
        _, act = collect(samples, lambda s, n=name: s_surface_deg(s, n))
        surf[name] = dict(cmd_deg=minmaxmean(cmd), actual_deg=minmaxmean(act),
                          max_abs_actual_deg=(max(abs(x) for x in act) if act else None))
    out["surfaces"] = surf
    out["elevator_max_abs_deg"] = max(
        [surf[n]["max_abs_actual_deg"] for n in ("left_elevator", "right_elevator")
         if surf[n]["max_abs_actual_deg"] is not None] or [None])
    lat = [surf[n]["max_abs_actual_deg"] for n in ("left_aileron", "right_aileron", "rudder")
           if surf[n]["max_abs_actual_deg"] is not None]
    out["lateral_surface_max_abs_deg"] = max(lat) if lat else None
    tgt = eff = 0
    for s in samples:
        if not s["actuators"]:
            continue
        for _, d in s["actuators"].items():
            tgt += 1 if d["target_clamp_active"] else 0
            eff += 1 if d["effort_clamp_active"] else 0
    out["actuator_clamp"] = dict(target_clamp_active_samples=tgt, effort_clamp_active_samples=eff)

    # ---- lateral sanity (no lateral command is given) ----
    _, roll = collect(samples, lambda s: s["gz"]["att_deg"][0] if s["gz"]["att_deg"] else None)
    _, nav_r = collect(samples, lambda s: s["mav"]["nav_roll_deg"])
    out["roll_gz_deg"] = minmaxmean(roll)
    out["nav_roll_demand_deg"] = minmaxmean(nav_r)

    # ---- mode integrity ----
    modes = sorted(set(s["mav"]["custom_mode"] for s in samples if s["mav"]["custom_mode"] is not None))
    out["custom_modes_seen"] = modes
    out["all_fbwb"] = (modes == [ARDUPLANE_FBWB_CUSTOM_MODE])

    # ---- oscillation growth (same detrended first-half/second-half test the
    #      FBWA stage used, imported verbatim) ----
    out["oscillation_growth"] = dict(
        altitude=detrended_growth(ts_z, zs) if len(zs) >= 8 else None,
        airspeed=detrended_growth(ts, v_vfr) if len(v_vfr) >= 8 else None,
        pitch_physical=detrended_growth(ts_p, pitch) if len(pitch) >= 8 else None,
        throttle=detrended_growth(ts_t, thr) if len(thr) >= 8 else None,
    )

    # ---- NaN / Inf guard ----
    bad = []
    for s in samples:
        for grp in ("gz", "mav"):
            for k, v in s[grp].items():
                for x in (v if isinstance(v, list) else [v]):
                    if isinstance(x, float) and not math.isfinite(x):
                        bad.append([s["t"], grp, k])
        for grp in ("aero", "propulsion", "actuators"):
            d = s[grp]
            if not d:
                continue
            stack = [(d, grp)]
            while stack:
                dd, path = stack.pop()
                for kk, vv in dd.items():
                    if isinstance(vv, dict):
                        stack.append((vv, path + "." + kk))
                    elif isinstance(vv, float) and not math.isfinite(vv):
                        bad.append([s["t"], path + "." + kk])
    out["nan_inf_count"] = len(bad)
    out["nan_inf_samples"] = bad[:50]
    return out


def analyze(R, segs, p, ptch_trim_deg):
    ptd = ptch_trim_deg
    A = segs["A_baseline_level_cruise"]["samples"]
    A_hold = [s for s in A if s["t_seg"] >= SEG_A_TRANSIENT_S]
    an = {"segment_plan": dict(
        seg_a_duration_s=SEG_A_DURATION_S, seg_a_transient_s=SEG_A_TRANSIENT_S,
        seg_b_ramp_max_s=SEG_B_RAMP_MAX_S, seg_b_hold_s=SEG_B_HOLD_S,
        seg_c_ramp_max_s=SEG_C_RAMP_MAX_S, seg_c_hold_s=SEG_C_HOLD_S,
        hold_transient_s=HOLD_TRANSIENT_S, alt_step_m=ALT_STEP_M)}
    an["A_full"] = analyze_window(A, "A_full", p, ptd)
    an["A_hold"] = analyze_window(A_hold, "A_hold_settled", p, ptd)

    for key, seg_key, tr in (("B_ramp", "B_climb_ramp", 0.0),
                             ("B_hold", "B_hold_new_altitude", HOLD_TRANSIENT_S),
                             ("C_ramp", "C_descent_ramp", 0.0),
                             ("C_hold", "C_hold_return_altitude", HOLD_TRANSIENT_S)):
        seg = segs.get(seg_key)
        if seg is None:
            an[key] = None
            continue
        sub = [s for s in seg["samples"] if s["t_seg"] >= tr]
        an[key] = analyze_window(sub, key, p, ptd)

    # ---- whole-flight quantities ----
    allsamp = []
    for k in ("A_baseline_level_cruise", "B_climb_ramp", "B_hold_new_altitude",
              "C_descent_ramp", "C_hold_return_altitude"):
        if k in segs:
            allsamp.extend(segs[k]["samples"])
    allsamp.sort(key=lambda s: s["t"])
    _, thr_all = collect(allsamp, s_throttle_actual)
    thr_min_p = (p.get("THR_MIN") or 0.0) / 100.0
    thr_max_p = (p.get("THR_MAX") or 100.0) / 100.0
    an["whole_flight"] = dict(
        n_samples=len(allsamp),
        duration_s=(allsamp[-1]["t"] - allsamp[0]["t"]) if len(allsamp) >= 2 else None,
        throttle_range=(max(thr_all) - min(thr_all)) if thr_all else None,
        throttle_min=min(thr_all) if thr_all else None,
        throttle_max=max(thr_all) if thr_all else None,
        throttle_sat_high_longest_run_s=longest_run_seconds(
            allsamp, lambda s: (s_throttle_actual(s) is not None
                                and s_throttle_actual(s) >= thr_max_p - TH_SAT_MARGIN)),
        throttle_sat_low_longest_run_s=longest_run_seconds(
            allsamp, lambda s: (s_throttle_actual(s) is not None
                                and s_throttle_actual(s) <= thr_min_p + TH_SAT_MARGIN)),
        elevator_max_abs_deg=max(
            [abs(s_surface_deg(s, n)) for s in allsamp
             for n in ("left_elevator", "right_elevator")
             if s_surface_deg(s, n) is not None] or [None]),
        lateral_surface_max_abs_deg=max(
            [abs(s_surface_deg(s, n)) for s in allsamp
             for n in ("left_aileron", "right_aileron", "rudder")
             if s_surface_deg(s, n) is not None] or [None]),
        airspeed_min_ms=min([s["mav"]["airspeed"] for s in allsamp
                             if s["mav"]["airspeed"] is not None] or [None]),
        airspeed_max_ms=max([s["mav"]["airspeed"] for s in allsamp
                             if s["mav"]["airspeed"] is not None] or [None]),
    )

    # ---- TECS authority: FBWB RC3 sets AIRSPEED, not throttle. Compare the
    #      measured motor throttle against the MANUAL-passthrough throttle the
    #      SAME RC3 PWM would produce (mode.cpp:318 path). ----
    rc3 = segs["A_baseline_level_cruise"]["rc3"]
    manual_equiv = control_in_range_no_dz(rc3, p["RC3_MIN"], p["RC3_MAX"],
                                          bool(p["RC3_REVERSED"])) / 100.0
    a_thr = an["A_hold"].get("throttle_actual")
    an["tecs_authority"] = dict(
        rc3_pwm_us=rc3,
        manual_passthrough_equivalent_throttle=manual_equiv,
        measured_throttle_mean_A_hold=(a_thr["mean"] if a_thr else None),
        abs_delta=(abs(a_thr["mean"] - manual_equiv) if a_thr else None),
        note="In FBWB the throttle stick sets target AIRSPEED (navigation.cpp:187-189); "
             "throttle itself is TECS output (Attitude.cpp:510). A large delta proves "
             "TECS - not the stick - is the throttle authority.")

    # ---- pitch/throttle coordination across the climb and descent ----
    def wmean(w, key, sub=None):
        d = an.get(w)
        if not d or d.get(key) is None:
            return None
        v = d[key]
        return v.get("mean") if isinstance(v, dict) else v
    an["coordination"] = dict(
        level_throttle=wmean("A_hold", "throttle_actual"),
        climb_throttle=wmean("B_ramp", "throttle_actual"),
        descent_throttle=wmean("C_ramp", "throttle_actual"),
        level_pitch_deg=wmean("A_hold", "pitch_physical_noseup_deg"),
        climb_pitch_deg=wmean("B_ramp", "pitch_physical_noseup_deg"),
        descent_pitch_deg=wmean("C_ramp", "pitch_physical_noseup_deg"),
        level_vz_ms=wmean("A_hold", "vfr_hud_climb_ms"),
        climb_vz_ms=wmean("B_ramp", "vfr_hud_climb_ms"),
        descent_vz_ms=wmean("C_ramp", "vfr_hud_climb_ms"),
        expected_extra_power_for_2ms_climb_W=MASS_KG * G * 2.0,
        expected_gamma_for_2ms_climb_deg=math.degrees(math.asin(2.0 / V_TARGET_MS)),
        note="climb must show MORE throttle and MORE nose-up than level; descent the "
             "opposite. Thresholds test the SIGN of the energy split, not its magnitude.")

    # ---- altitude step achieved ----
    def alt_mean(w):
        d = an.get(w)
        if not d or d.get("altitude_gz_m") is None:
            return None
        return d["altitude_gz_m"]["mean"]
    an["altitude_step"] = dict(
        commanded_step_m=ALT_STEP_M,
        alt_A_hold_mean_m=alt_mean("A_hold"),
        alt_B_hold_mean_m=alt_mean("B_hold"),
        alt_C_hold_mean_m=alt_mean("C_hold"),
        achieved_climb_m=((alt_mean("B_hold") - alt_mean("A_hold"))
                          if alt_mean("B_hold") is not None and alt_mean("A_hold") is not None else None),
        achieved_descent_m=((alt_mean("C_hold") - alt_mean("B_hold"))
                            if alt_mean("C_hold") is not None and alt_mean("B_hold") is not None else None),
        climb_ramp_duration_s=segs.get("B_climb_ramp", {}).get("actual_duration_s"),
        descent_ramp_duration_s=segs.get("C_descent_ramp", {}).get("actual_duration_s"),
        climb_ramp_stopped_early=segs.get("B_climb_ramp", {}).get("stopped_early"),
        descent_ramp_stopped_early=segs.get("C_descent_ramp", {}).get("stopped_early"))

    R["analysis"] = an
    return an


# =============================================================================
# acceptance
# =============================================================================
def verdict(R):
    an = R.get("analysis")
    if not an or not an.get("A_hold") or an["A_hold"].get("insufficient_samples"):
        return "TECS_CRUISE_SPEED_HOLD_FAILED", ["no analysable Segment A hold window"]
    p = R.get("tecs_baseline_params_live", {})
    A = an["A_hold"]
    wf = an["whole_flight"]
    c = {}

    # Safe accessors: a window with too few usable samples yields a partial
    # dict. A missing quantity must FAIL its check, never raise and never be
    # silently treated as passing.
    def abs_le(x, lim):
        return x is not None and isinstance(x, (int, float)) and math.isfinite(x) and abs(x) <= lim

    def sub_abs_le(d, key, lim):
        return isinstance(d, dict) and abs_le(d.get(key), lim)

    # --- 1. mode / configuration integrity -----------------------------------
    pre = R.get("param_preconditions", {})
    c["mode_is_fbwb_throughout"] = bool(A.get("all_fbwb"))
    c["param_preconditions_all_ok"] = all(pre.values()) if pre else False
    c["tecs_at_firmware_defaults_baseline"] = bool(pre.get("tecs_at_firmware_defaults"))
    c["pids_unchanged"] = bool(pre.get("pids_unchanged"))

    # --- 2. TECS is genuinely the active speed/height controller --------------
    ta = an["tecs_authority"]
    c["tecs_is_driving_throttle_not_the_stick"] = (
        ta["abs_delta"] is not None and ta["abs_delta"] > TH_TECS_AUTHORITY_MIN_DELTA)
    c["throttle_is_actively_modulated"] = (
        wf.get("throttle_range") is not None and wf["throttle_range"] > TH_THROTTLE_MODULATION_MIN)
    tt = A.get("tecs_target_airspeed_ms")
    c["tecs_target_airspeed_matches_command"] = (
        isinstance(tt, dict) and tt.get("mean") is not None
        and abs(tt["mean"] - V_TARGET_MS) <= TH_TECS_TARGET_TOL_MS)

    # --- 3. speed hold --------------------------------------------------------
    asp = A["airspeed_vfr_hud_ms"]
    c["speed_mean_within_tol_of_18"] = (isinstance(asp, dict) and asp.get("mean") is not None
                                        and abs(asp["mean"] - V_TARGET_MS) <= TH_SPEED_MEAN_TOL_MS)
    c["speed_std_bounded"] = sub_abs_le(asp, "std", TH_SPEED_STD_MAX_MS)
    c["speed_no_upward_divergence"] = sub_abs_le(asp, "slope_per_s",
                                                 TH_SPEED_SLOPE_MAX_MS_PER_S)
    c["speed_never_below_airspeed_min"] = (
        wf.get("airspeed_min_ms") is not None and wf["airspeed_min_ms"] >= TH_SPEED_MIN_MS)
    c["speed_never_below_underspeed_trigger"] = (
        wf.get("airspeed_min_ms") is not None and wf["airspeed_min_ms"] >= TH_SPEED_HARD_FLOOR_MS)
    c["speed_never_above_airspeed_max"] = (
        wf.get("airspeed_max_ms") is not None and p.get("AIRSPEED_MAX") is not None
        and wf["airspeed_max_ms"] <= p["AIRSPEED_MAX"])
    gsd = A.get("groundspeed_minus_airspeed_max_abs_ms")
    c["zero_wind_confirmed_gs_vs_as"] = abs_le(gsd, TH_GS_VS_AS_MAX_MS)

    # --- 4. altitude hold -----------------------------------------------------
    c["alt_hold_slope_bounded_A"] = sub_abs_le(A, "vertical_speed_regression_ms", TH_ALT_SLOPE_MAX_MS)
    c["alt_hold_p2p_bounded_A"] = sub_abs_le(A, "altitude_p2p_m", TH_ALT_P2P_MAX_M)
    c["alt_hold_no_unidirectional_drift_A"] = sub_abs_le(A, "vertical_speed_endpoint_ms",
                                                         TH_ALT_SLOPE_MAX_MS)
    c["fbwa_residual_sink_closed"] = sub_abs_le(A, "vertical_speed_regression_ms", TH_SINK_CLOSED_MS)
    c["alt_hold_tight_preferred"] = sub_abs_le(A, "vertical_speed_regression_ms",
                                               TH_ALT_SLOPE_TIGHT_MS)
    # MINOR-4 fix (gazebo-testing, 2026-09-02): these checks were previously
    # only CREATED when the window existed, so a missing/short B or C hold
    # window silently vanished from the check set instead of failing it. The
    # segments are unconditionally flown by flight_sequence(), so their absence
    # is a real defect and is now reported as one.
    for key, name in (("B_hold", "B"), ("C_hold", "C")):
        w = an.get(key)
        usable = bool(w) and not w.get("insufficient_samples")
        wasp = w.get("airspeed_vfr_hud_ms") if usable else None
        c[f"{name}_hold_window_present"] = usable
        c[f"alt_hold_slope_bounded_{name}"] = usable and sub_abs_le(
            w, "vertical_speed_regression_ms", TH_ALT_SLOPE_MAX_MS)
        c[f"alt_hold_p2p_bounded_{name}"] = usable and sub_abs_le(
            w, "altitude_p2p_m", TH_ALT_P2P_MAX_M)
        c[f"speed_mean_within_tol_of_18_{name}"] = (
            usable and isinstance(wasp, dict) and wasp.get("mean") is not None
            and abs(wasp["mean"] - V_TARGET_MS) <= TH_SPEED_MEAN_TOL_MS)

    # --- 5. altitude step actually executed, in the right DIRECTION -----------
    st = an["altitude_step"]
    c["altitude_step_climb_achieved"] = (
        st["achieved_climb_m"] is not None
        and st["achieved_climb_m"] >= TH_ALT_STEP_ACHIEVED_FRAC * ALT_STEP_M)
    # MINOR-4 fix (gazebo-testing, 2026-09-02): `achieved_descent_m is None`
    # previously PASSED this check. Missing data is now a FAILURE, per this
    # file's own stated policy. Same for the down-stick direction check below.
    c["altitude_step_descent_achieved"] = (
        st["achieved_descent_m"] is not None
        and st["achieved_descent_m"] <= -TH_ALT_STEP_ACHIEVED_FRAC * ALT_STEP_M)
    # CONTROL DIRECTION - explicitly test-verified, never assumed.
    br = an.get("B_ramp")
    cr = an.get("C_ramp")
    c["fbwb_up_stick_climbs"] = (isinstance(br, dict)
                                 and br.get("vertical_speed_regression_ms") is not None
                                 and br["vertical_speed_regression_ms"] > TH_RAMP_DIRECTION_MIN_MS)
    c["fbwb_down_stick_descends"] = (isinstance(cr, dict)
                                     and cr.get("vertical_speed_regression_ms") is not None
                                     and cr["vertical_speed_regression_ms"] < -TH_RAMP_DIRECTION_MIN_MS)

    # --- 6. throttle equilibrium + saturation --------------------------------
    thr = A["throttle_actual"]
    c["throttle_plausible_vs_measured_trim"] = (
        isinstance(thr, dict) and thr.get("mean") is not None
        and abs(thr["mean"] - TRIM_THROTTLE_REF) <= TH_THROTTLE_TOL)
    c["no_sustained_throttle_high_saturation"] = abs_le(
        wf.get("throttle_sat_high_longest_run_s"), TH_SAT_RUN_MAX_S)
    c["no_sustained_throttle_low_saturation"] = abs_le(
        wf.get("throttle_sat_low_longest_run_s"), TH_SAT_RUN_MAX_S)

    # --- 7. pitch / throttle coordination (energy split sign) ----------------
    co = an["coordination"]
    def _gt(a, b, d):
        return a is not None and b is not None and (a - b) > d
    def _lt(a, b, d):
        return a is not None and b is not None and (b - a) > d
    c["climb_uses_more_throttle_than_level"] = _gt(co["climb_throttle"], co["level_throttle"],
                                                   TH_COORD_THROTTLE_DELTA)
    c["climb_is_more_nose_up_than_level"] = _gt(co["climb_pitch_deg"], co["level_pitch_deg"],
                                                TH_COORD_PITCH_DELTA_DEG)
    if co["descent_throttle"] is not None:
        c["descent_uses_less_throttle_than_level"] = _lt(co["descent_throttle"], co["level_throttle"],
                                                         TH_COORD_THROTTLE_DELTA)
        c["descent_is_more_nose_down_than_level"] = _lt(co["descent_pitch_deg"], co["level_pitch_deg"],
                                                        TH_COORD_PITCH_DELTA_DEG)
    par = A.get("pitch_minus_alpha_minus_gamma_deg")
    c["longitudinal_kinematics_consistent"] = sub_abs_le(par, "mean",
                                                         TH_PITCH_ALPHA_GAMMA_RESID_DEG)

    # --- 8. control surfaces --------------------------------------------------
    c["elevator_within_10deg_in_hold"] = abs_le(A.get("elevator_max_abs_deg"),
                                                TH_SURF_HOLD_MAX_DEG)
    c["elevator_within_15deg_whole_flight"] = abs_le(wf.get("elevator_max_abs_deg"),
                                                     TH_SURF_FLIGHT_MAX_DEG)
    c["lateral_surfaces_bounded"] = abs_le(wf.get("lateral_surface_max_abs_deg"),
                                           TH_LATERAL_SURF_MAX_DEG)
    clamp_total = 0
    for k in ("A_full", "B_ramp", "B_hold", "C_ramp", "C_hold"):
        w = an.get(k)
        if w and w.get("actuator_clamp"):
            clamp_total += (w["actuator_clamp"]["target_clamp_active_samples"]
                            + w["actuator_clamp"]["effort_clamp_active_samples"])
    c["zero_actuator_clamp"] = (clamp_total == 0)

    # --- 9. stability / numerics ---------------------------------------------
    growing = False
    for k in ("A_hold", "B_hold", "C_hold"):
        w = an.get(k)
        if not w or not w.get("oscillation_growth"):
            continue
        for _, g in w["oscillation_growth"].items():
            if g and g.get("growing"):
                growing = True
    c["no_growing_oscillation"] = not growing
    nan_total = sum((an[k]["nan_inf_count"] if an.get(k) and "nan_inf_count" in an[k] else 0)
                    for k in ("A_full", "B_ramp", "B_hold", "C_ramp", "C_hold"))
    c["no_nan_inf"] = (nan_total == 0)

    R["acceptance_checks"] = c
    fails = [k for k, ok in c.items() if not ok and not k.endswith("_preferred")]

    core = [
        "mode_is_fbwb_throughout", "param_preconditions_all_ok",
        "tecs_at_firmware_defaults_baseline", "pids_unchanged",
        "tecs_is_driving_throttle_not_the_stick", "tecs_target_airspeed_matches_command",
        "speed_mean_within_tol_of_18", "speed_std_bounded", "speed_no_upward_divergence",
        "speed_never_below_airspeed_min", "zero_wind_confirmed_gs_vs_as",
        "alt_hold_slope_bounded_A", "alt_hold_p2p_bounded_A",
        "alt_hold_no_unidirectional_drift_A", "fbwa_residual_sink_closed",
        "altitude_step_climb_achieved", "fbwb_up_stick_climbs", "fbwb_down_stick_descends",
        "no_sustained_throttle_high_saturation", "no_sustained_throttle_low_saturation",
        "climb_uses_more_throttle_than_level", "climb_is_more_nose_up_than_level",
        "elevator_within_10deg_in_hold", "elevator_within_15deg_whole_flight",
        "zero_actuator_clamp", "no_growing_oscillation", "no_nan_inf",
        "throttle_plausible_vs_measured_trim", "longitudinal_kinematics_consistent",
    ]
    core_ok = all(c.get(k, False) for k in core)
    # PARTIAL = TECS demonstrably works and is stable/safe, but a secondary,
    # quantitative criterion missed. Never used to hide a direction/sign,
    # saturation, NaN or divergence failure.
    partial_ok = all(c.get(k, False) for k in [
        "mode_is_fbwb_throughout", "tecs_is_driving_throttle_not_the_stick",
        "fbwb_up_stick_climbs", "no_growing_oscillation", "no_nan_inf",
        "zero_actuator_clamp", "speed_never_below_airspeed_min",
        "no_sustained_throttle_high_saturation", "no_sustained_throttle_low_saturation"])
    if core_ok:
        return "TECS_CRUISE_SPEED_HOLD_PASS", fails
    if partial_ok:
        return "TECS_CRUISE_SPEED_HOLD_PARTIAL", fails
    return "TECS_CRUISE_SPEED_HOLD_FAILED", fails


# =============================================================================
# flight
# =============================================================================
def flight_sequence(mav, sub, osub, adiag, pdiag, aerodiag, p, R):
    rc3_cruise, achieved_target, err = rc3_pwm_for_target_airspeed(V_TARGET_MS, p)
    R["command_derivation"] = dict(
        v_target_ms=V_TARGET_MS, rc3_pwm_us=rc3_cruise,
        rc3_pwm_us_rounded=(int(round(rc3_cruise)) if rc3_cruise else None),
        predicted_target_airspeed_ms=achieved_target, error=err,
        # MINOR-2 fix: what ArduPlane can actually demand, and by how much the
        # commanded PWM misses the nominal target because of the int16_t
        # control_in truncation (RC_Channel.cpp:316 / RC_Channel.h:99).
        demand_quantisation_grid_ms=(
            (int(p["AIRSPEED_MAX"]) - int(p["AIRSPEED_MIN"])) / 100.0
            if p.get("AIRSPEED_MAX") is not None and p.get("AIRSPEED_MIN") is not None
            else None),
        demand_quantisation_error_ms=(
            (achieved_target - V_TARGET_MS) if achieved_target is not None else None),
        rc2_up_us=p.get("RC2_MAX"), rc2_down_us=p.get("RC2_MIN"),
        rc2_neutral_us=p.get("RC2_TRIM"),
        formula="navigation.cpp:187-189 inverted through RC_Channel.cpp:388-402")
    print("command derivation:", json.dumps(R["command_derivation"], default=str))
    if rc3_cruise is None:
        R["flight_result"] = dict(aborted=True, reason="rc3_derivation_failed")
        return False, {}
    rc3_cruise = int(round(rc3_cruise))
    rc2_neutral = int(round(p["RC2_TRIM"]))
    rc2_up = int(round(p["RC2_MAX"]))
    rc2_down = int(round(p["RC2_MIN"]))

    if not enter_fbwb(mav, rc3_cruise, R):
        R["flight_result"] = dict(aborted=True, reason="fbwb_not_confirmed")
        return False, {}

    for msg_id in campaign.MSG_IDS_20HZ:
        campaign.request_rate(mav, msg_id, 20.0)
    time.sleep(0.3)

    latest = {}
    t0 = time.time()
    segs = {}
    ptd = p.get("PTCH_TRIM_DEG", PTCH_TRIM_DEG_EXPECTED)

    # --- Segment A: baseline level cruise ---
    print(f"SEGMENT A: baseline level cruise, {SEG_A_DURATION_S}s "
          f"(rc2={rc2_neutral}, rc3={rc3_cruise} -> target {achieved_target:.3f} m/s)")
    segs["A_baseline_level_cruise"] = run_seg(
        mav, sub, osub, adiag, pdiag, aerodiag, "A_baseline_level_cruise",
        SEG_A_DURATION_S, 1500, rc2_neutral, rc3_cruise, t0, latest)
    if segs["A_baseline_level_cruise"]["aborted"]:
        R["flight_result"] = dict(aborted=True, reason="segment_A_aborted",
                                  detail=segs["A_baseline_level_cruise"]["abort_reason"])
        return False, segs

    a_end_alt = None
    for s in reversed(segs["A_baseline_level_cruise"]["samples"]):
        if s_alt(s) is not None:
            a_end_alt = s_alt(s)
            break

    # --- Segment B: +10 m step (FBWB ramps the target at FBWB_CLIMB_RATE;
    #     releasing the stick locks the target at the current altitude) ---
    target_alt = a_end_alt + ALT_STEP_M

    def stop_climb(s, _):
        z = s_alt(s)
        if z is not None and z >= target_alt:
            return True, f"reached target altitude {target_alt:.2f} m"
        return False, None

    print(f"SEGMENT B: +{ALT_STEP_M} m step, up-stick rc2={rc2_up}, "
          f"cap {SEG_B_RAMP_MAX_S}s, target alt {target_alt:.2f} m")
    segs["B_climb_ramp"] = run_seg(
        mav, sub, osub, adiag, pdiag, aerodiag, "B_climb_ramp", SEG_B_RAMP_MAX_S,
        1500, rc2_up, rc3_cruise, t0, latest, stop_fn=stop_climb)
    if segs["B_climb_ramp"]["aborted"]:
        R["flight_result"] = dict(aborted=True, reason="segment_B_ramp_aborted",
                                  detail=segs["B_climb_ramp"]["abort_reason"])
        return False, segs

    print(f"SEGMENT B: hold new altitude, {SEG_B_HOLD_S}s")
    segs["B_hold_new_altitude"] = run_seg(
        mav, sub, osub, adiag, pdiag, aerodiag, "B_hold_new_altitude", SEG_B_HOLD_S,
        1500, rc2_neutral, rc3_cruise, t0, latest)
    if segs["B_hold_new_altitude"]["aborted"]:
        R["flight_result"] = dict(aborted=True, reason="segment_B_hold_aborted",
                                  detail=segs["B_hold_new_altitude"]["abort_reason"])
        return False, segs

    # --- Segment C: descend back down and re-hold ---
    b_end_alt = None
    for s in reversed(segs["B_hold_new_altitude"]["samples"]):
        if s_alt(s) is not None:
            b_end_alt = s_alt(s)
            break
    target_alt_c = b_end_alt - ALT_STEP_M

    def stop_descend(s, _):
        z = s_alt(s)
        if z is not None and z <= target_alt_c:
            return True, f"reached target altitude {target_alt_c:.2f} m"
        return False, None

    print(f"SEGMENT C: -{ALT_STEP_M} m, down-stick rc2={rc2_down}, cap {SEG_C_RAMP_MAX_S}s")
    segs["C_descent_ramp"] = run_seg(
        mav, sub, osub, adiag, pdiag, aerodiag, "C_descent_ramp", SEG_C_RAMP_MAX_S,
        1500, rc2_down, rc3_cruise, t0, latest, stop_fn=stop_descend)
    if not segs["C_descent_ramp"]["aborted"]:
        print(f"SEGMENT C: hold return altitude, {SEG_C_HOLD_S}s")
        segs["C_hold_return_altitude"] = run_seg(
            mav, sub, osub, adiag, pdiag, aerodiag, "C_hold_return_altitude",
            SEG_C_HOLD_S, 1500, rc2_neutral, rc3_cruise, t0, latest)

    aborted = any(v["aborted"] for v in segs.values())
    R["flight_result"] = dict(aborted=aborted,
                              segment_summary=[(k, v["n_samples"], v["aborted"],
                                                v["stopped_early"], v["stop_reason"])
                                               for k, v in segs.items()])
    if not aborted:
        analyze(R, segs, p, ptd)
    return not aborted, segs


# =============================================================================
# I/O
# =============================================================================
def write_outputs(R, segs):
    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    ts_doc = {"stage": STAGE, "timestamp": R.get("timestamp"),
              "tecs_baseline_params_live": R.get("tecs_baseline_params_live"),
              "command_derivation": R.get("command_derivation"),
              "segments": {k: {kk: vv for kk, vv in v.items()} for k, v in segs.items()}}
    with open(OUT_TS, "w") as f:
        # compact separators: the raw record is ~3000 x 20 Hz samples and is
        # kept COMPLETE (nothing is dropped) so validation can re-derive any
        # quantity independently.
        json.dump(ts_doc, f, default=str, separators=(",", ":"))
    slim = dict(R)
    slim["segments_summary"] = {
        k: {kk: vv for kk, vv in v.items() if kk not in ("samples",)}
        for k, v in segs.items()}
    slim["timeseries_file"] = OUT_TS
    with open(OUT_JSON, "w") as f:
        json.dump(slim, f, indent=2, default=str)


def finish_fail(R, phase, mav, segs=None):
    R["overall_result"] = "TEST_FAILED"
    R["verdict"] = "TECS_CRUISE_SPEED_HOLD_FAILED"
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
         "tecs_baseline_params_live": p, "command_derivation": doc.get("command_derivation"),
         "reanalyzed_from": path}
    R["param_preconditions"] = param_precondition_checks(p, R)
    analyze(R, segs, p, p.get("PTCH_TRIM_DEG", PTCH_TRIM_DEG_EXPECTED))
    vd, fails = verdict(R)
    R["verdict"] = vd
    R["failed_checks"] = fails
    R["overall_result"] = "REANALYZED"
    write_outputs(R, segs)
    print("verdict:", vd)
    print("failed_checks:", fails)
    return 0


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--reanalyze":
        return reanalyze(sys.argv[2])

    R = {"stage": STAGE,
         "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         "mode": dict(name="FBWB", custom_mode=ARDUPLANE_FBWB_CUSTOM_MODE,
                      evidence="see module docstring + docs/source_of_truth/controls/"
                               "ardupilot_fbwb_tecs_baseline.yaml"),
         "reference_constants": dict(
             V_TRIM_REF=V_TRIM_REF, TRIM_THROTTLE_REF=TRIM_THROTTLE_REF,
             ELEV_TRIM_DEG_REF=ELEV_TRIM_DEG_REF,
             PTCH_TRIM_DEG_EXPECTED=PTCH_TRIM_DEG_EXPECTED,
             FBWA_RESIDUAL_SINK_MS=FBWA_RESIDUAL_SINK_MS),
         "thresholds": dict(
             TH_SPEED_MEAN_TOL_MS=TH_SPEED_MEAN_TOL_MS, TH_SPEED_STD_MAX_MS=TH_SPEED_STD_MAX_MS,
             TH_SPEED_MIN_MS=TH_SPEED_MIN_MS, TH_SPEED_HARD_FLOOR_MS=TH_SPEED_HARD_FLOOR_MS,
             TH_TECS_TARGET_TOL_MS=TH_TECS_TARGET_TOL_MS,
             TH_ALT_SLOPE_MAX_MS=TH_ALT_SLOPE_MAX_MS, TH_ALT_P2P_MAX_M=TH_ALT_P2P_MAX_M,
             TH_SINK_CLOSED_MS=TH_SINK_CLOSED_MS, TH_ALT_SLOPE_TIGHT_MS=TH_ALT_SLOPE_TIGHT_MS,
             TH_THROTTLE_TOL=TH_THROTTLE_TOL, TH_SAT_RUN_MAX_S=TH_SAT_RUN_MAX_S,
             TH_TECS_AUTHORITY_MIN_DELTA=TH_TECS_AUTHORITY_MIN_DELTA,
             TH_THROTTLE_MODULATION_MIN=TH_THROTTLE_MODULATION_MIN,
             TH_SURF_HOLD_MAX_DEG=TH_SURF_HOLD_MAX_DEG,
             TH_SURF_FLIGHT_MAX_DEG=TH_SURF_FLIGHT_MAX_DEG,
             TH_LATERAL_SURF_MAX_DEG=TH_LATERAL_SURF_MAX_DEG,
             TH_COORD_THROTTLE_DELTA=TH_COORD_THROTTLE_DELTA,
             TH_COORD_PITCH_DELTA_DEG=TH_COORD_PITCH_DELTA_DEG,
             TH_PITCH_ALPHA_GAMMA_RESID_DEG=TH_PITCH_ALPHA_GAMMA_RESID_DEG,
             TH_GS_VS_AS_MAX_MS=TH_GS_VS_AS_MAX_MS,
             TH_SPEED_SLOPE_MAX_MS_PER_S=TH_SPEED_SLOPE_MAX_MS_PER_S,
             TH_ALT_STEP_ACHIEVED_FRAC=TH_ALT_STEP_ACHIEVED_FRAC,
             TH_RAMP_DIRECTION_MIN_MS=TH_RAMP_DIRECTION_MIN_MS)}

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
                "RC3_REVERSED", "RC2_MIN", "RC2_MAX", "RC2_TRIM", "THR_MIN", "THR_MAX"]
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
        A = R["analysis"]["A_hold"]
        co = R["analysis"]["coordination"]
        ta = R["analysis"]["tecs_authority"]
        try:
            print("-" * 74)
            print(f"A_hold airspeed mean/std      : {A['airspeed_vfr_hud_ms']['mean']:.3f} / "
                  f"{A['airspeed_vfr_hud_ms']['std']:.3f} m/s (target {V_TARGET_MS})")
            print(f"A_hold TECS target airspeed   : {A['tecs_target_airspeed_ms']['mean']:.3f} m/s")
            print(f"A_hold vertical speed (reg)   : {A['vertical_speed_regression_ms']:+.4f} m/s "
                  f"(FBWA residual was {FBWA_RESIDUAL_SINK_MS})")
            print(f"A_hold altitude p2p           : {A['altitude_p2p_m']:.3f} m")
            print(f"A_hold throttle mean          : {A['throttle_actual']['mean']:.4f} "
                  f"(measured trim* {TRIM_THROTTLE_REF})")
            print(f"TECS authority |delta|        : {ta['abs_delta']} "
                  f"(manual passthrough would be {ta['manual_passthrough_equivalent_throttle']:.4f})")
            print(f"A_hold physical pitch         : {A['pitch_physical_noseup_deg']['mean']:+.3f} deg")
            print(f"level/climb/descent throttle  : {co['level_throttle']} / {co['climb_throttle']} / "
                  f"{co['descent_throttle']}")
            print(f"level/climb/descent pitch deg : {co['level_pitch_deg']} / {co['climb_pitch_deg']} / "
                  f"{co['descent_pitch_deg']}")
            print(f"altitude step                 : {R['analysis']['altitude_step']}")
            print(f"elevator max |deg| (flight)   : {R['analysis']['whole_flight']['elevator_max_abs_deg']}")
        except Exception as exc:  # summary print only - JSON is authoritative
            print("summary print failed:", exc)
        print(f"VERDICT: {vd}  failed_checks={fails}")
        print("-" * 74)
    else:
        R["overall_result"] = "FLIGHT_ABORTED"
        R["verdict"] = "TECS_CRUISE_SPEED_HOLD_FAILED"
        print("FLIGHT ABORTED:", R.get("flight_result"))

    write_outputs(R, segs)
    print("RESULT:", R["overall_result"], R.get("verdict"), "->", OUT_JSON)
    print("TIMESERIES:", OUT_TS)
    mav.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

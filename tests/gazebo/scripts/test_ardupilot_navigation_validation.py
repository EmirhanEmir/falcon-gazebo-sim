#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_NAVIGATION_AND_AUTOTUNE_VALIDATION
(controls-integration, 2026-09-07).

GOAL
----
Fly Falcon V2 under GENUINE ArduPlane navigation control - GUIDED, LOITER,
RTL, and a sequential-GUIDED waypoint route - in free 6-DOF, zero wind, and
record enough evidence to answer, from measurement rather than impression:

  * does the aircraft navigate in the RIGHT DIRECTION (no wrong-sign response)?
  * are altitude and airspeed bounded, and does airspeed stay >= AIRSPEED_MIN?
  * is control-surface usage inside the aerodynamically validated range, and
    is the +/-45 deg mechanical limit ever continuously pinned (a BLOCKER)?
  * is there any GROWING oscillation (measured envelope metric, not eyeballed)?
  * and - as DECISION SUPPORT ONLY - do roll/pitch tracking error, saturation
    duty cycle and oscillation growth constitute evidence that the current
    PIDs are inadequate?

THIS TEST DOES NOT RUN AUTOTUNE, and does not write any parameter of any kind.
See section "AUTOTUNE" at the bottom of this file: the procedure is present as
a DORMANT, DELIBERATELY UNEXECUTED stub. Per the stage instruction, AUTOTUNE is
run only if a later independent evidence review proves the current PIDs are
genuinely inadequate during navigation.

=============================================================================
1. MODES USED - SOURCE-CITED (ArduPlane 4.8.0-dev, commit 409226a637,
   /home/emirhan/gazebo_sim/ardupilot)
=============================================================================
  GUIDED  custom_mode 15   ArduPlane/mode.h:53
  RTL     custom_mode 11   ArduPlane/mode.h:49
  LOITER  custom_mode 12   ArduPlane/mode.h:50
  FBWA    custom_mode 5    ArduPlane/mode.h:44   (bring-up / settle only)

All three navigation modes are full auto-navigation + auto-throttle modes:
ModeGuided::navigate()->plane.update_loiter() (mode_guided.cpp:104-107),
ModeLoiter::navigate() (mode_loiter.cpp:137), ModeRTL::navigate()
(mode_rtl.cpp:77). Roll comes from plane.calc_nav_roll() (the L1 controller),
pitch/throttle from TECS via calc_nav_pitch()/calc_throttle().

  * NO full AUTO mission is uploaded anywhere in this file (stage instruction).
    Scenario D is a route flown as SEQUENTIAL GUIDED TARGETS - the minimal
    equivalent - using MAV_CMD_DO_REPOSITION.
  * GUIDED targets are set with MAV_CMD_DO_REPOSITION (COMMAND_INT, 192),
    handled at GCS_MAVLink_Plane.cpp:560-609 -> plane.set_guided_WP().
    SET_POSITION_TARGET_GLOBAL_INT is NOT used: in ArduPlane it only handles
    the ALTITUDE field (GCS_MAVLink_Plane.cpp:1101-1136 calls
    handle_change_alt_request() and nothing else) - it cannot set a position
    target. Using it would have silently produced a no-op waypoint.

=============================================================================
2. TELEMETRY SEMANTICS - VERIFIED IN SOURCE, NOT ASSUMED
=============================================================================
NAV_CONTROLLER_OUTPUT.xtrack_error IS A RADIAL ERROR IN ALL FOUR SCENARIOS,
not a path cross-track error. Derivation:
  * set_guided_WP() (commands.cpp:97) and do_RTL() (commands_logic.cpp:340)
    BOTH set auto_state.crosstrack = false.
  * update_loiter_update_nav() (navigation.cpp:343-359) only takes the
    nav_controller->update_waypoint() branch when auto_state.crosstrack is
    TRUE; otherwise it calls nav_controller->update_loiter().
  * AP_L1_Control::update_loiter() sets
        _crosstrack_error = xtrackErrCirc = A_air.length() - radius
    (AP_L1_Control.cpp:424-426), i.e. the RADIAL distance outside/inside the
    loiter circle around the active target. Positive = outside the circle.
  * LOITER mode never had crosstrack set either.
  Consequence used by the analysis below: in every scenario the identity
        xtrack_error ~= wp_dist - loiter_radius
  must hold; it is CHECKED as an independent measurement cross-check
  (`xtrack_identity_consistent`). If it ever fails, the semantics assumed
  here are wrong and every derived navigation metric is suspect - which is
  exactly what this project's measurement-discipline rule demands be
  detectable rather than assumed away.
  (For completeness, in the update_waypoint() branch the sign convention is
  _crosstrack_error = A_air % AB with NE components, i.e. POSITIVE = aircraft
  LEFT of track - AP_L1_Control.cpp:277. That branch is not taken here.)

NAV_CONTROLLER_OUTPUT.nav_roll  = the L1/navigation ROLL DEMAND in deg.
NAV_CONTROLLER_OUTPUT.nav_pitch = the RAW TECS pitch demand in deg, WITHOUT
    PTCH_TRIM_DEG. The physically demanded attitude is
        pitch_demand_phys = nav_pitch + PTCH_TRIM_DEG      (Attitude.cpp:244)
GCS ATTITUDE.pitch = (true_pitch - PTCH_TRIM_DEG)   (GCS_MAVLink_Plane.cpp:139)
GCS ATTITUDE.roll is NOT offset by anything - roll needs no correction.
Gazebo Euler pitch is nose-DOWN-positive in this FLU world, so physical
nose-up pitch = -(gz euler pitch). Roll and yaw need no such flip.

=============================================================================
3. FRAMES AND SIGNS - STATED, THEN VERIFIED IN-RUN
=============================================================================
World: ENU (world file <world_frame_orientation>ENU</world_frame_orientation>,
       <heading_deg>0.0</heading_deg>) => gz +X = EAST, +Y = NORTH, +Z = UP.
Body:  FLU (CLAUDE.md) => +X fwd, +Y left, +Z up.

POSITIVE ROLL = RIGHT BANK. Right-hand rule about body +X carries +Y (left
wing) toward +Z (up), i.e. right wing down. ArduPilot's own FRD convention
independently gives the same sign, so gz ground-truth roll and MAVLink
ATTITUDE.roll must agree in sign - CHECKED (`roll_sign_gz_mav_agree`).

COMPASS HEADING from gz yaw: gz yaw is CCW from +X (East) in ENU, so
    heading_compass_deg = (90 - yaw_deg) mod 360
GROUND COURSE from the gz world velocity (vE, vN):
    course_compass_deg = atan2(vE, vN) in deg, mod 360
Both are CHECKED against ArduPlane's own reported heading
(`heading_frame_consistent`) rather than assumed. A failure here is a
frame regression, not a tuning problem.

NAVIGATION SIGN UNDER TEST (the "no wrong-sign response" acceptance item):
    err = wrap180(target_bearing - heading)   ( + = target is to the RIGHT )
    A correct navigator commands nav_roll with the SAME sign as err
    (turn right for a right-hand target) and drives |err| down.
This is measured, per scenario, over the capture window; it is never assumed.

=============================================================================
4. WHAT IS NOT DONE HERE
=============================================================================
  * No parameter is written. config/ardupilot/falcon_v2_sitl.parm is read-only
    input. Every value this test depends on is LIVE-READ over MAVLink.
  * No SDF / plugin / aero / propulsion / actuator / sensor / mass / CG change.
  * No AUTOTUNE. No AUTO mission. No physics bypass, no pose or velocity
    forcing during any measured window (see section 5).
  * The high-advance-ratio / windmilling propeller regime remains OPEN /
    DATA_REQUIRED / NON-GATING and is NOT touched, resolved or worked around
    by this stage. It is carried forward verbatim as a limitation in the
    result JSON (`known_open_limitations`).

=============================================================================
5. AIRBORNE INITIAL CONDITION (not a physics bypass)
=============================================================================
Each scenario starts from this project's already-validated bring-up, identical
to every prior ArduPlane stage here (test_ardupilot_basic_closed_loop_flight):
    arm on the ground -> wait for ground settle -> teleport to (0,0,90) m ->
    short "hold to trim" wrench window -> FULL wrench release (clear_wrench)
From the release instant onward there is ZERO force, torque, pose or velocity
intervention: every measured window is genuinely free 6-DOF flight under
ArduPlane's own closed-loop control. The bring-up window itself is excluded
from all analysis.

USAGE (a FRESH gz sim + gdb-wrapped arduplane pair MUST already be running -
see tests/gazebo/scripts/run_ardupilot_navigation_validation.sh):
    python3 test_ardupilot_navigation_validation.py --scenario A|B|C|D
    python3 test_ardupilot_navigation_validation.py --summarize
    python3 test_ardupilot_navigation_validation.py --reanalyze <timeseries.json>
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
from pymavlink import mavutil  # noqa: E402

import test_ardupilot_basic_closed_loop_flight as base  # noqa: E402
import test_ardupilot_basic_closed_loop_flight_campaign as campaign  # noqa: E402
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

STAGE = "ARDUPLANE_NAVIGATION_AND_AUTOTUNE_VALIDATION"
RES = base.RESULTS_DIR
PREFIX = "ardupilot_navigation"

# ---- ArduPlane mode numbers (ArduPlane/mode.h, cited in the docstring) ----
MODE_FBWA = 5
MODE_RTL = 11
MODE_LOITER = 12
MODE_GUIDED = 15
MODE_NAME = {5: "FBWA", 11: "RTL", 12: "LOITER", 15: "GUIDED"}

MAV_CMD_DO_REPOSITION = 192
MAV_CMD_REQUEST_MESSAGE = 512
MAVLINK_MSG_ID_HOME_POSITION = 242
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

# -----------------------------------------------------------------------------
# Geodesy. ArduPilot's OWN scaling constant, so distances computed here are
# directly comparable with NAV_CONTROLLER_OUTPUT.wp_dist.
#   AP_Common/Location.h:217  LOCATION_SCALING_FACTOR = LATLON_TO_M
#   AP_Math/definitions.h     LATLON_TO_M = RADIUS_OF_EARTH * DEG_TO_RAD * 1e-7
#                             RADIUS_OF_EARTH = 6378100 m
# => 1 degree of latitude = 6378100 * pi/180 = 111318.845 m
# -----------------------------------------------------------------------------
RADIUS_OF_EARTH_M = 6378100.0                                  # AP_Math/definitions.h
DEG_TO_M = RADIUS_OF_EARTH_M * math.pi / 180.0                 # 111318.845 m/deg

# =============================================================================
# ACCEPTANCE THRESHOLDS - every one carries its own provenance. They are also
# written verbatim into the result JSON so a reviewer never has to read this
# file to know what a verdict meant.
# =============================================================================
TH = {}
TH_PROV = {}


def _th(name, value, provenance, gating, cls):
    """Register an acceptance threshold.

    `cls` is the machine-readable provenance CLASS and is emitted into every
    result JSON alongside the value, the prose provenance and the gating flag.
    Closes validation MINOR-3: docs/source_of_truth/controls/
    ardupilot_navigation_modes.yaml declares the result JSON to be the
    authoritative machine-readable copy, so the JSON must carry every field the
    YAML carries - previously `class` existed only in the YAML and could drift
    from the code silently."""
    TH[name] = value
    TH_PROV[name] = dict(value=value, provenance=provenance, gating=gating,
                         **{"class": cls})
    return value


def threshold_class_census():
    """Machine-computed census of threshold provenance classes.

    Closes validation MINOR-4. The ASSUMPTION count is NEVER stated by hand
    again anywhere - it is derived from TH_PROV at runtime and emitted into the
    result JSON, so it cannot drift from the code as thresholds are added or
    reclassified. `assumption_total` counts every class containing the token
    ASSUMPTION, i.e. pure ASSUMPTION plus the dual-tagged
    ASSUMPTION_DECISION_SUPPORT_ONLY entries."""
    by_class = {}
    for name, meta in TH_PROV.items():
        by_class.setdefault(meta["class"], []).append(name)
    pure = sorted(by_class.get("ASSUMPTION", []))
    dual = sorted(n for c, names in by_class.items() if c != "ASSUMPTION"
                  and "ASSUMPTION" in c for n in names)
    return {
        "n_thresholds": len(TH_PROV),
        "n_gating": sum(1 for m in TH_PROV.values() if m["gating"]),
        "n_non_gating": sum(1 for m in TH_PROV.values() if not m["gating"]),
        "by_class": {k: sorted(v) for k, v in sorted(by_class.items())},
        "assumption_pure": pure,
        "assumption_dual_tagged": dual,
        "assumption_total": len(pure) + len(dual),
        # Surfaces any threshold whose PROSE calls itself an assumption while
        # its machine-readable class says otherwise. Such an entry is not
        # necessarily wrong (e.g. a derived rule that notes an assumption about
        # one input), but it is exactly where a hand-counted "number of
        # ASSUMPTION thresholds" silently diverges from the code - which is how
        # the earlier count discrepancy arose. Listed, never resolved silently.
        "prose_mentions_assumption_but_class_does_not": sorted(
            n for n, m in TH_PROV.items()
            if "ASSUMPTION" in m["provenance"].upper()
            and "ASSUMPTION" not in m["class"]),
        "note": ("Counts are computed from TH_PROV at runtime. Do not restate "
                 "them by hand anywhere; quote this block."),
    }


# --- control-surface usage -----------------------------------------------
# 10 deg is NOT a comfort number: it is the outer edge of the XFLR5-validated
# LINEAR control-derivative range for every surface on this aircraft
# (docs/source_of_truth/controls/CONTROLS.md sec 1 "Aero-derivative validated
# linear range" = ~+/-10 deg; master dataset sec 65, and the Type 7 sweeps
# actually run: sec 22 elevator, sec 32 rudder, sec 36 aileron, all -10..+10).
# Beyond it the simulation's own aerodynamic model is EXTRAPOLATING, so
# "predominantly <= 10 deg" is a model-validity statement.
TH_SURF_LINEAR_DEG = _th(
    "TH_SURF_LINEAR_DEG", 10.0,
    "XFLR5 validated linear control-derivative range +/-10 deg "
    "(docs/source_of_truth/controls/CONTROLS.md sec 1; master dataset sec 65, "
    "sweeps sec 22/32/36 run -10..+10)", gating=True, cls="XFLR5_VALIDATED_RANGE")
# "predominantly" is quantified as the 95th percentile of |deflection| over the
# steady windows. p95 tolerates brief transients while still failing a control
# that genuinely lives outside the validated range.
TH_SURF_PERCENTILE = _th(
    "TH_SURF_PERCENTILE", 95.0,
    "ASSUMPTION - quantifies the stage instruction's word 'predominantly'. "
    "p95 admits short transients (<=5 % of samples) but fails sustained "
    "out-of-linear-range operation. Non-derived; declared, not discovered.",
    gating=True, cls="ASSUMPTION")

# --- hard mechanical limit (BLOCKER) --------------------------------------
# +/-45 deg = +/-0.7853981634 rad, the joint <limit> in model/model.sdf (all 5
# control-surface joints) and the actuator clamp in
# docs/source_of_truth/controls/actuator_v1_config.yaml min/max_angle_rad
# (USER_CONFIRMED_MECHANICAL_CAPABILITY). The actuator diagnostics publish
# `target_clamp_active` when that clamp actually engages - that flag, not an
# inferred angle comparison, is the primary detector.
TH_SURF_HARD_LIMIT_DEG = _th(
    "TH_SURF_HARD_LIMIT_DEG", 45.0,
    "model/model.sdf control-surface joint <limit> +/-0.7853981634 rad and "
    "actuator_v1_config.yaml min/max_angle_rad (USER_CONFIRMED_MECHANICAL_"
    "CAPABILITY)", gating=True, cls="USER_CONFIRMED_MECHANICAL_CAPABILITY")
TH_SURF_PIN_MARGIN_DEG = _th(
    "TH_SURF_PIN_MARGIN_DEG", 0.5,
    "ASSUMPTION - 'at the limit' band. 0.5 deg is ~1 % of the 45 deg limit and "
    "far above actuator numerical jitter.", gating=True, cls="ASSUMPTION")
TH_SURF_PIN_RUN_MAX_S = _th(
    "TH_SURF_PIN_RUN_MAX_S", 0.5,
    "ASSUMPTION - quantifies 'CONTINUOUSLY pinning'. At the 20 Hz sample rate "
    "0.5 s is 10 consecutive samples: long enough to reject a single-sample "
    "touch, short enough to catch genuine sustained saturation.", gating=True, cls="ASSUMPTION")
TH_SURF_PIN_DUTY_MAX = _th(
    "TH_SURF_PIN_DUTY_MAX", 0.01,
    "ASSUMPTION - total fraction of samples allowed at the hard limit (1 %).",
    gating=True, cls="ASSUMPTION")

# --- airspeed --------------------------------------------------------------
# AIRSPEED_MIN = 16 m/s, live-read (config/ardupilot/falcon_v2_sitl.parm,
# SIMULATION_DERIVED_FROM_GAZEBO_ENVELOPE, 2026-08-27 flight-envelope
# validation). The gate is applied AFTER the mode-entry transient because a
# mode change legitimately perturbs speed briefly.
TH_AIRSPEED_MIN_MS = _th(
    "TH_AIRSPEED_MIN_MS", 16.0,
    "AIRSPEED_MIN, live-read; falcon_v2_sitl.parm SIMULATION_DERIVED_FROM_"
    "GAZEBO_ENVELOPE (docs/test_results/2026-08-27_flight_envelope_validation.md)",
    gating=True, cls="LIVE_READBACK")
TH_AIRSPEED_HARD_FLOOR_FRAC = _th(
    "TH_AIRSPEED_HARD_FLOOR_FRAC", 0.9,
    "Same convention as the TECS cruise stage (TH_SPEED_HARD_FLOOR_MS = 0.9 * "
    "AIRSPEED_MIN): a 10 % excursion below AIRSPEED_MIN is a hard failure "
    "anywhere in the record, transient or not.", gating=True, cls="CONVENTION_REUSED")
TH_MODE_TRANSIENT_S = _th(
    "TH_MODE_TRANSIENT_S", 10.0,
    "ASSUMPTION - post-mode-change transient excluded from the steady-window "
    "gates. 10 s is ~2 TECS_TIME_CONST (live-read 5.0 s).", gating=True, cls="ASSUMPTION")

# --- altitude envelope (run abort guards, not quality metrics) -------------
TH_ALT_FLOOR_M = _th("TH_ALT_FLOOR_M", 40.0,
                     "Run-abort guard. Start altitude is 90 m; 40 m still "
                     "leaves ample margin. Same value as the TECS stages.",
                     gating=True, cls="RUN_ABORT_GUARD")
TH_ALT_CEILING_M = _th("TH_ALT_CEILING_M", 260.0,
                       "Run-abort guard. RTL_ALTITUDE is live-read (100 m); "
                       "260 m is a runaway-climb trip well above any commanded "
                       "altitude in this campaign.", gating=True, cls="RUN_ABORT_GUARD")
TH_ATT_ABORT_DEG = _th("TH_ATT_ABORT_DEG", 60.0,
                       "Run-abort guard on |roll| and |pitch|. ROLL_LIMIT_DEG "
                       "is live-read (45 deg); 60 deg means the attitude "
                       "controller has lost the aircraft.", gating=True, cls="RUN_ABORT_GUARD")
# Bounded-altitude quality gate over the steady window.
TH_ALT_P2P_MAX_M = _th(
    "TH_ALT_P2P_MAX_M", 25.0,
    "ASSUMPTION - 'altitude stays bounded' during navigation. Navigation "
    "modes bank hard, and TECS trades height for speed in turns, so this is "
    "deliberately looser than the TECS altitude-hold stage's own straight-"
    "flight number. It is a BOUNDEDNESS gate, not a hold-quality gate.",
    gating=True, cls="ASSUMPTION")
TH_AIRSPEED_P2P_MAX_MS = _th(
    "TH_AIRSPEED_P2P_MAX_MS", 6.0,
    "ASSUMPTION - 'airspeed stays bounded'. AIRSPEED_MAX-AIRSPEED_MIN is 12 "
    "m/s (live-read); half of the full commandable envelope is a generous "
    "boundedness gate that still fails a divergence.", gating=True, cls="ASSUMPTION")

# --- oscillation -----------------------------------------------------------
# Reused VERBATIM from the phugoid/TECS stages (fbwa.detrended_growth): a fit
# line is removed, then second-half residual std is compared with first-half.
# growing = (s2 >= 1.3 * s1). Not re-invented here, so results are directly
# comparable with the already-validated longitudinal stages.
TH_OSC_GROWTH_RATIO = _th(
    "TH_OSC_GROWTH_RATIO", 1.3,
    "fbwa.detrended_growth()'s own criterion, reused verbatim from the "
    "validated phugoid-damping and TECS stages. REPORTED FOR COMPARABILITY, "
    "NOT GATED - see TH_OSC_ENVELOPE_RATIO and the ESTIMATOR NOTE below.",
    gating=False, cls="REPORTED_FOR_COMPARABILITY")
# ---------------------------------------------------------------------------
# ESTIMATOR NOTE (controls-integration, 2026-09-07, decided during HARNESS
# CONSTRUCTION on a SMOKE run, BEFORE any campaign data existed - it is not a
# reaction to a campaign result):
#   fbwa.detrended_growth() removes a straight line and compares first-half vs
#   second-half residual spread. In the STRAIGHT, single-condition flight of the
#   phugoid and TECS stages that is a valid divergence detector. In NAVIGATION
#   flight it is NOT: the aircraft deliberately changes flight condition inside
#   the window (roll into a turn, capture, circle), so a larger second-half
#   residual can mean "manoeuvring harder", not "oscillation growing". Measured
#   live on the harness smoke run: altitude residual p2p went 1.91 m -> 2.41 m
#   and airspeed 1.27 -> 1.81 m/s across the window - both plainly BOUNDED and
#   small, yet both flagged "growing". That is a measurement category error of
#   exactly the kind this project has repeatedly mis-read as physics.
#   The GATE is therefore a PEAK-ENVELOPE metric (envelope_growth() below): the
#   signal is detrended, split at its zero crossings into half-cycles, and the
#   half-cycle peak magnitudes are fitted with ln|peak| = sigma*t + c. sigma is
#   an envelope growth rate in 1/s; a monotonic manoeuvring trend produces no
#   half-cycle peaks and is therefore correctly reported as "not oscillatory"
#   instead of "growing". The ACCEPTANCE STRENGTH is unchanged: the same 1.3x
#   envelope growth across the window that detrended_growth() uses.
#   Both metrics are recorded for every channel; only the envelope one gates.
# ---------------------------------------------------------------------------
TH_OSC_ENVELOPE_RATIO = _th(
    "TH_OSC_ENVELOPE_RATIO", 1.3,
    "GATING. Peak-envelope growth exp(sigma*T) across the analysis window. The "
    "1.3x figure is taken deliberately unchanged from fbwa.detrended_growth() "
    "so the acceptance STRENGTH matches the validated stages while the "
    "ESTIMATOR is the well-posed one - see the ESTIMATOR NOTE above.",
    gating=True, cls="CONVENTION_REUSED")
TH_OSC_PEAK_PROMINENCE_MEDIAN_FRAC = _th(
    "TH_OSC_PEAK_PROMINENCE_MEDIAN_FRAC", 0.25,
    "GATING. A half-cycle peak below this fraction of the MEDIAN half-cycle "
    "peak is a zero-crossing artefact, not a cycle of the mode being measured. "
    "REPLACED the previous rule '5 % of the detrended peak-to-peak' on "
    "2026-09-08. ROOT-CAUSE FIX, see the ROBUSTNESS NOTE below: peak-to-peak is "
    "an EXTREME-VALUE statistic, so a single large transient inflated the "
    "threshold and filtered out the ordinary cycles of the very oscillation "
    "being measured - which collapsed both the peak span and the spacing "
    "statistics. The median half-cycle peak is a robust amplitude scale and "
    "does not have that failure mode. The exact fraction is an ASSUMPTION; the "
    "use of a robust scale rather than an extreme one is not.",
    gating=True, cls="ASSUMPTION")
TH_OSC_MIN_PEAKS = _th(
    "TH_OSC_MIN_PEAKS", 6,
    "IDENTIFIABILITY REQUIREMENT (raised from 4 to 6 on 2026-09-07 during "
    "harness construction - see the IDENTIFIABILITY NOTE below). 6 half-cycle "
    "peaks = 3 complete cycles, the minimum over which an exponential envelope "
    "fit is identifiable at all. With fewer, the channel is reported "
    "UNDERDETERMINED_TOO_FEW_CYCLES and is neither passed nor failed on this "
    "metric; raw-state divergence over such a short window is instead covered "
    "by the boundedness and run-abort gates.", gating=True, cls="IDENTIFIABILITY_REQUIREMENT")
# ---------------------------------------------------------------------------
# IDENTIFIABILITY NOTE (controls-integration, 2026-09-07, decided on SMOKE data
# during harness construction, BEFORE any campaign data existed):
#   With the original TH_OSC_MIN_PEAKS = 4, the scenario C smoke run reported
#   roll_err_deg as "growing" with sigma = +0.0217 /s and an envelope ratio of
#   1.53. The underlying numbers show that verdict was not identifiable:
#     n_half_cycle_peaks = 4, period 13.1 s, window 19.7 s  -> 1.5 CYCLES
#     first_peak = 7.09 deg, last_peak = 7.34 deg           -> +3 %, not +53 %
#     peak_max   = 31.4 deg                                 -> ONE large
#         intermediate transient (the RTL turn-in step response) dominating a
#         4-point log-linear fit.
#   i.e. the envelope was flat end to end and the "growth" was an artefact of
#   fitting an exponential through a step response.
#   FOUR requirements now guard a growth verdict, all statistical rather than
#   physical (see the COHERENCE REQUIREMENTS block below for (2)-(4), added
#   2026-09-08 after validation found that (1) was carrying the whole load):
#     (1) TH_OSC_MIN_PEAKS       >= 3 complete cycles
#     (2) TH_OSC_MIN_PEAK_SPAN_FRAC  peaks span most of the reported window
#     (3) TH_OSC_MAX_GAP_FRAC        no single gap swallows half the peak span
#     (4) a ROBUST slope estimator (Theil-Sen) so no single peak can dominate
#         the fit - this replaced the bolt-on endpoint guard on 2026-09-08
#   None weakens detection of a genuine divergence, which by definition produces
#   many, regularly spaced cycles spanning the window with a monotonically
#   rising envelope: the injected exp(t/60) negative control is still DETECTED
#   with all four in place.
#   This is flagged explicitly for `validation` review: it CHANGED a smoke-run
#   outcome from FAIL to PASS, and must be judged on the identifiability
#   argument above, not on the outcome. (Validation SUSTAINED it on 2026-09-07;
#   requirements (2)-(4) are its follow-on MINOR-1/MINOR-2 tightenings.)
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# COHERENCE REQUIREMENTS (added 2026-09-08, closing validation MINOR-1 and
# MINOR-2 on the identifiability argument).
#
# MINOR-1: the previous endpoint guard was `last_peak >= first_peak`, which does
#   not bite. Scenario C's roll_err_deg satisfied it by +0.03 % (last 6.759 vs
#   first 6.757) while the fit claimed +53 % envelope growth, so TH_OSC_MIN_PEAKS
#   was in fact the SOLE discriminator and the documented "second guard" was
#   decorative. It is replaced by a MAGNITUDE requirement below.
#
# MINOR-2: peak COUNT is only a proxy for coherence. Scenario C's 4 peaks spanned
#   just 25 % of the analysis window with half-cycle spacings of 21.7 / 3.1 /
#   5.7 s (a 7:1 spread) - six equally clustered peaks would have been fitted
#   just as happily. Two explicit criteria are added: the peaks must SPAN most of
#   the window being reported on, and their spacing must be CONSISTENT enough to
#   be one oscillation rather than a handful of unrelated transients.
#
# All three are TIGHTENINGS: each one can only turn a "growing" verdict into
# UNDERDETERMINED or not-growing where the evidence does not support the claim,
# and none can turn a non-growing signal into growing.
# ---------------------------------------------------------------------------
TH_OSC_ENDPOINT_RATIO_MIN = _th(
    "TH_OSC_ENDPOINT_RATIO_MIN", 1.15,
    "RETIRED AS A GATE 2026-09-08, RETAINED AS A REPORTED DIAGNOSTIC. Its "
    "purpose - stopping one large intermediate peak from dominating the fit - "
    "now lives in the estimator (Theil-Sen slope + robust prominence scale). "
    "Measured on the real campaign channels it was load-bearing in ZERO cases, "
    "while on injected genuine divergences it produced false negatives. "
    "Historical definition: last_half_cycle_peak / first_half_cycle_peak "
    "this before a 'growing' verdict is allowed. DERIVED, not chosen: it is "
    "half of the fitted-envelope growth already required "
    "(1 + (TH_OSC_ENVELOPE_RATIO - 1) / 2 = 1 + 0.3/2 = 1.15), i.e. a genuine "
    "1.3x envelope must show at least half of that growth end to end even "
    "allowing for peak-to-peak scatter. Replaces the previous non-biting "
    "`last >= first` rule, which scenario C satisfied by +0.03 %.",
    gating=False, cls="RETIRED_REPORTED_ONLY")
TH_OSC_MIN_PEAK_SPAN_FRAC = _th(
    "TH_OSC_MIN_PEAK_SPAN_FRAC", 0.60,
    "GATING. The half-cycle peaks must span at least this fraction of the "
    "analysis window. ASSUMPTION as to the exact fraction, but the principle is "
    "not: a growth rate fitted over a quarter of the window and then reported "
    "as the window's envelope ratio is an EXTRAPOLATION, not a measurement. "
    "Scenario C's peaks spanned 25 %.",
    gating=True, cls="ASSUMPTION")
# ---------------------------------------------------------------------------
# ROBUSTNESS NOTE (controls-integration, 2026-09-08, closing a TEST-COVERAGE
# REGRESSION that gazebo-testing found in this agent's own 2026-09-08 MINOR-2
# tightening; measured on the REAL campaign channels, not on synthetics).
#
# WHAT WENT WRONG. The first MINOR-2 implementation made the oscillation gate
# VACUOUS: gated channels reported UNDERDETERMINED went from 6/19 to 19/19
# across the campaign, so `no_growing_oscillation` returned True everywhere
# because nothing was being evaluated - not because nothing was growing. That
# is strictly worse than the gap it was meant to narrow. Two independent
# defects, one shared root cause:
#
#   (a) SPACING as max/min. An unbounded extreme-value ratio. On real flight
#       data the detrended residual makes brief zero-crossing excursions, one
#       half-cycle collapses toward the sample interval, min(spacing) goes tiny
#       and the ratio explodes regardless of oscillation strength. Measured
#       3.10-46.71 against a limit of 3.0, and it rejected scenario-D per-leg
#       cross-track trains of 163, 72 and 226 half-cycle peaks - at which point
#       an envelope fit is unambiguously identifiable. A criterion that
#       tightens as evidence accumulates is measuring the wrong thing.
#
#   (b) PROMINENCE keyed to peak-to-peak - the SHARED ROOT CAUSE. p2p is itself
#       an extreme-value statistic. One large transient inflates it, so the
#       5 %-of-p2p floor filtered out the ordinary cycles of the oscillation
#       being measured, leaving only the few peaks clustered around the
#       transient. That simultaneously collapsed the peak SPAN (measured
#       0.13-0.59 on channels that are plainly oscillatory) and manufactured
#       the tiny spacings that made (a) explode. Fixing the spacing statistic
#       alone would have left 16 of 19 channels ungated by the span criterion.
#
# WHY THE ORIGINAL SYNTHETIC VERIFICATION MISSED IT: those fixtures contained
# nothing but the injected oscillation, so p2p and the median peak coincide and
# spacings are exactly uniform. The same synthetic scores a spacing ratio of
# 1.79 at every amplitude. Verification of a robustness property must be done
# on signals that carry the real underlying content - this is now done by
# injection into the actual captured campaign channels.
#
# THE FIX: a robust amplitude scale for prominence (median half-cycle peak) and
# a robust, identifiability-motivated coherence statistic (largest gap as a
# fraction of the peak span). TH_OSC_MIN_PEAKS = 6 and the span criterion are
# UNCHANGED - both were sustained by validation and neither is at fault.
# ---------------------------------------------------------------------------
TH_OSC_MAX_GAP_FRAC = _th(
    "TH_OSC_MAX_GAP_FRAC", 0.50,
    "GATING. max(half-cycle spacing) / (time spanned by the peaks). REPLACED "
    "TH_OSC_SPACING_RATIO_MAX = max/min spacing on 2026-09-08 - see the "
    "ROBUSTNESS NOTE below; max/min is an unbounded extreme-value ratio whose "
    "denominator collapses toward the sample interval on any real signal, so it "
    "rejected peak trains of 163, 72 and 226 cycles as 'incoherent'. "
    "DERIVED, not chosen: if a SINGLE gap covers more than half the interval "
    "the peaks span, the train is two clusters separated by a void and the "
    "envelope across that void is INTERPOLATED rather than measured - a real "
    "identifiability defect. Structurally immune to the failure mode above "
    "because it does not depend on min(spacing) at all. For a uniform train of "
    "n peaks it equals 1/(n-1), so it relaxes automatically as evidence "
    "accumulates - which is the correct behaviour for an identifiability "
    "screen. The scenario-C train that motivated the original criterion scores "
    "0.711 and is still rejected.",
    gating=True, cls="DERIVED_IDENTIFIABILITY_CRITERION")

# --- navigation sign -------------------------------------------------------
TH_NAV_SIGN_AGREE_FRAC = _th(
    "TH_NAV_SIGN_AGREE_FRAC", 0.80,
    "ASSUMPTION - fraction of capture-window samples in which sign(nav_roll) "
    "must equal sign(heading error to target). Not 1.0: the L1 law legitimately "
    "reverses briefly at overshoot and near |err|~0, where the sign is noise.",
    gating=True, cls="ASSUMPTION")
TH_NAV_SIGN_DEADBAND_DEG = _th(
    "TH_NAV_SIGN_DEADBAND_DEG", 5.0,
    "ASSUMPTION - heading errors below this are excluded from the sign vote; "
    "their sign carries no navigation information.", gating=True, cls="ASSUMPTION")
TH_HEADING_FRAME_TOL_DEG = _th(
    "TH_HEADING_FRAME_TOL_DEG", 10.0,
    "ASSUMPTION - agreement required between ArduPlane's reported heading and "
    "the heading derived from Gazebo ground-truth yaw via the documented ENU-> "
    "compass conversion. A frame regression would blow far past 10 deg.",
    gating=True, cls="ASSUMPTION")
TH_XTRACK_IDENTITY_TOL_M = _th(
    "TH_XTRACK_IDENTITY_TOL_M", 3.0,
    "ASSUMPTION - tolerance on the source-derived identity xtrack_error == "
    "wp_dist - loiter_radius (see docstring sec 2). wp_dist is a uint16 in "
    "WHOLE METRES, so >=1 m of quantisation is unavoidable; 3 m allows for "
    "that plus the ~0.05 s sampling skew between the two fields.", gating=True, cls="ASSUMPTION")

# --- scenario-specific -----------------------------------------------------
TH_LOITER_RADIUS_TOL_M = _th(
    "TH_LOITER_RADIUS_TOL_M", 20.0,
    "ASSUMPTION - allowed |mean orbit radius - WP_LOITER_RAD| (live-read, "
    "60 m). 20 m is 33 % of the commanded radius: loose enough not to be a "
    "disguised L1-tuning gate, tight enough to fail a genuine spiral.",
    gating=True, cls="ASSUMPTION")
TH_LOITER_RADIUS_STD_MAX_M = _th(
    "TH_LOITER_RADIUS_STD_MAX_M", 15.0,
    "ASSUMPTION - orbit-radius std over the steady window; a growing spiral "
    "fails this and the oscillation-growth metric together.", gating=True, cls="ASSUMPTION")
TH_RTL_ALT_TOL_M = _th(
    "TH_RTL_ALT_TOL_M", 10.0,
    "GATING (scenario C only). |mean relative altitude in the post-arrival home "
    "loiter - RTL_ALTITUDE| (live-read, 100 m). ASSUMPTION: tight enough to "
    "prove RTL actually CAPTURED its commanded altitude rather than merely "
    "staying airborne (10 % of RTL_ALTITUDE), deliberately not tighter, because "
    "TECS altitude-hold QUALITY is a separate, already-validated stage and this "
    "check must not silently become a re-run of it.",
    gating=True, cls="ASSUMPTION")
TH_APPROACH_CLOSURE_FRAC = _th(
    "TH_APPROACH_CLOSURE_FRAC", 0.5,
    "ASSUMPTION - a GUIDED/RTL capture must reduce the distance to the target "
    "by at least this fraction of the initial distance within the window, "
    "otherwise it is not navigating toward it at all.", gating=True, cls="ASSUMPTION")

# --- AUTOTUNE decision support (NON-GATING, never used to pass/fail) -------
TH_AUTOTUNE_ROLL_RMS_DEG = _th(
    "TH_AUTOTUNE_ROLL_RMS_DEG", 5.0,
    "ASSUMPTION - DECISION SUPPORT ONLY, NON-GATING. RMS(nav_roll - roll) "
    "above which the roll rate loop is FLAGGED for an evidence review. No "
    "measured Falcon V2 tracking-error reference exists yet: DATA_REQUIRED.",
    gating=False, cls="ASSUMPTION_DECISION_SUPPORT_ONLY")
TH_AUTOTUNE_PITCH_RMS_DEG = _th(
    "TH_AUTOTUNE_PITCH_RMS_DEG", 5.0,
    "ASSUMPTION - DECISION SUPPORT ONLY, NON-GATING. Same basis as the roll "
    "figure. DATA_REQUIRED.", gating=False, cls="ASSUMPTION_DECISION_SUPPORT_ONLY")
TH_AUTOTUNE_SAT_DUTY = _th(
    "TH_AUTOTUNE_SAT_DUTY", 0.05,
    "ASSUMPTION - DECISION SUPPORT ONLY, NON-GATING. Fraction of steady-window "
    "samples with a surface at the hard limit above which the control loop is "
    "flagged as authority-limited.", gating=False, cls="ASSUMPTION_DECISION_SUPPORT_ONLY")

# --- sampling / windows ----------------------------------------------------
# HEARTBEAT is requested at 4 Hz below, so a mode change can take up to ~0.25 s
# to appear in telemetry; 2 s is a generous allowance that still fails a mode
# that silently reverted.
MODE_ENTRY_LATENCY_S = 2.0
# A route leg may never terminate before this; guards against a stale
# NAV_CONTROLLER_OUTPUT capturing a leg instantly. A 550 m leg needs ~31 s at
# AIRSPEED_CRUISE, so 5 s cannot mask a genuine early capture.
LEG_MIN_S = 5.0
SETTLE_FBWA_S = 12.0     # neutral-stick FBWA settle before any nav mode
AIRBORNE_ALT_M = 90.0

SURFACES = ("left_aileron", "right_aileron", "left_elevator", "right_elevator", "rudder")

PARAMS_OF_INTEREST = [
    # navigation
    "WP_LOITER_RAD", "WP_RADIUS", "NAVL1_PERIOD", "NAVL1_DAMPING", "NAVL1_XTRACK_I",
    "RTL_ALTITUDE", "RTL_AUTOLAND", "RTL_CLIMB_MIN", "RTL_RADIUS",
    "ROLL_LIMIT_DEG", "PTCH_LIM_MAX_DEG", "PTCH_LIM_MIN_DEG", "PTCH_TRIM_DEG",
    "STICK_MIXING", "FLIGHT_OPTIONS", "FENCE_ENABLE", "FENCE_ACTION",
    "TERRAIN_FOLLOW", "GUIDED_TIMEOUT",
    # speed / energy
    "AIRSPEED_MIN", "AIRSPEED_CRUISE", "AIRSPEED_MAX", "AIRSPEED_STALL",
    "ARSPD_USE", "ARSPD_TYPE", "MIN_GROUNDSPEED", "THROTTLE_NUDGE",
    "THR_MIN", "THR_MAX", "TRIM_THROTTLE", "THR_SLEWRATE", "KFF_THR2PTCH",
    "TECS_PTCH_DAMP", "TECS_TIME_CONST", "TECS_CLMB_MAX", "TECS_SINK_MIN",
    "TECS_SINK_MAX", "TECS_SPDWEIGHT", "TECS_RLL2THR", "TECS_PITCH_MAX",
    "TECS_PITCH_MIN", "TECS_THR_DAMP", "TECS_INTEG_GAIN", "TECS_HDEM_TCONST",
    # attitude PIDs (recorded to prove they were NOT touched, and as the
    # baseline the AUTOTUNE decision would be made against)
    "RLL_RATE_P", "RLL_RATE_I", "RLL_RATE_D", "RLL_RATE_FF",
    "PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D", "PTCH_RATE_FF",
    "RLL2SRV_TCONST", "PTCH2SRV_TCONST", "AUTOTUNE_LEVEL",
    # RC calibration (needed to prove no stick contamination)
    "RC1_MIN", "RC1_MAX", "RC1_TRIM", "RC1_DZ", "RC1_REVERSED",
    "RC2_MIN", "RC2_MAX", "RC2_TRIM", "RC2_DZ", "RC2_REVERSED",
    "RC3_MIN", "RC3_MAX", "RC3_TRIM", "RC3_DZ", "RC3_REVERSED",
    "RC4_MIN", "RC4_MAX", "RC4_TRIM", "RC4_DZ", "RC4_REVERSED",
    # environment guards
    "SIM_WIND_SPD", "SIM_WIND_DIR", "SIM_WIND_TURB",
    "SIM_OPOS_LAT", "SIM_OPOS_LNG", "SIM_OPOS_ALT", "SIM_OPOS_HDG",
    "AHRS_EKF_TYPE",
]

KNOWN_OPEN_LIMITATIONS = [
    {"item": "HIGH_ADVANCE_RATIO_WINDMILLING_PROPELLER_REGIME",
     "status": "OPEN / DATA_REQUIRED / NON_GATING",
     "statement": (
         "The propulsion model's behaviour in the high-J / windmilling regime "
         "is not validated and the required data does not exist in this "
         "repository. This stage does NOT resolve, work around, or tune "
         "anything related to it. It is carried forward unchanged. It is "
         "explicitly NON-GATING for this stage's verdict; where a descent or "
         "an idle-throttle segment enters that regime, any propulsion number "
         "recorded there is reported as-is and must not be read as validated."),
     "owner": "propulsion"},
]


# =============================================================================
# small numeric helpers
# =============================================================================
def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def percentile(xs, q):
    if not xs:
        return None
    s = sorted(xs)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * q / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return s[lo]
    return s[lo] * (hi - k) + s[hi] * (k - lo)


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else None


def finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    """Flat-earth offset using ArduPilot's OWN scaling constant, so distances
    are directly comparable with NAV_CONTROLLER_OUTPUT.wp_dist. See the
    DEG_TO_M derivation at the top of this file."""
    dlat = north_m / DEG_TO_M
    dlon = east_m / (DEG_TO_M * math.cos(math.radians(lat_deg)))
    return lat_deg + dlat, lon_deg + dlon


def latlon_to_ne(lat0, lon0, lat, lon):
    """(north_m, east_m) of (lat,lon) relative to (lat0,lon0)."""
    dn = (lat - lat0) * DEG_TO_M
    de = (lon - lon0) * DEG_TO_M * math.cos(math.radians(0.5 * (lat + lat0)))
    return dn, de


def bearing_deg(dn, de):
    """compass bearing (deg from North, clockwise) of the NE vector."""
    return math.degrees(math.atan2(de, dn)) % 360.0


def circle_fit(xs, ys):
    """Algebraic (Kasa) least-squares circle fit. Returns (cx, cy, r) or None.
    Used as an ArduPlane-INDEPENDENT measurement of the loiter orbit, so the
    orbit verdict does not rest solely on the autopilot's own reported numbers."""
    n = len(xs)
    if n < 8:
        return None
    sx = sum(xs); sy = sum(ys)
    sxx = sum(x * x for x in xs); syy = sum(y * y for y in ys)
    sxy = sum(x * y for x, y in zip(xs, ys))
    sxxx = sum(x ** 3 for x in xs); syyy = sum(y ** 3 for y in ys)
    sxyy = sum(x * y * y for x, y in zip(xs, ys))
    sxxy = sum(x * x * y for x, y in zip(xs, ys))
    a11 = 2 * (sx * sx / n - sxx)
    a12 = 2 * (sx * sy / n - sxy)
    a22 = 2 * (sy * sy / n - syy)
    b1 = sx * (sxx + syy) / n - (sxxx + sxyy)
    b2 = sy * (sxx + syy) / n - (sxxy + syyy)
    det = a11 * a22 - a12 * a12
    if abs(det) < 1e-12:
        return None
    cx = (b1 * a22 - b2 * a12) / det
    cy = (a11 * b2 - a12 * b1) / det
    r = mean([math.hypot(x - cx, y - cy) for x, y in zip(xs, ys)])
    return cx, cy, r


def longest_true_run_s(ts, flags):
    best, start, prev = 0.0, None, None
    for t, f in zip(ts, flags):
        if f:
            if start is None:
                start = t
            prev = t
        else:
            if start is not None:
                best = max(best, prev - start)
            start = None
    if start is not None and prev is not None:
        best = max(best, prev - start)
    return best


# =============================================================================
# MAVLink helpers built on SafeMav (SafeMav itself is NOT modified)
# =============================================================================
def command_int(mav, command, frame, p1=0, p2=0, p3=0, p4=0, x=0, y=0, z=0,
                timeout=4.0):
    mav.m.mav.command_int_send(mav.m.target_system, mav.m.target_component,
                               frame, command, 0, 0,
                               float(p1), float(p2), float(p3), float(p4),
                               int(x), int(y), float(z))
    t0 = time.time()
    while time.time() - t0 < timeout:
        r, _, _ = select.select([mav.m.port], [], [], 0.3)
        if not r:
            continue
        msg = mav.m.recv_match(type="COMMAND_ACK", blocking=False)
        if msg and msg.command == command:
            return msg.result
    return None


def set_mode_confirm(mav, custom_mode, timeout=6.0):
    """Wait for the LAST heartbeat carrying the target custom_mode, not the
    first - identical pattern to campaign.enter_fbwa()/enter_fbwb()."""
    mav.m.mav.set_mode_send(mav.m.target_system,
                            base.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode)
    hb = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        r, _, _ = select.select([mav.m.port], [], [], 0.3)
        if not r:
            continue
        msg = mav.m.recv_match(type="HEARTBEAT", blocking=False)
        if msg is None:
            continue
        hb = msg
        if hb.custom_mode == custom_mode:
            return True, hb.custom_mode
    return False, (hb.custom_mode if hb else None)


def request_home(mav, timeout=5.0):
    mav.m.mav.command_long_send(mav.m.target_system, mav.m.target_component,
                                MAV_CMD_REQUEST_MESSAGE, 0,
                                MAVLINK_MSG_ID_HOME_POSITION, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        r, _, _ = select.select([mav.m.port], [], [], 0.3)
        if not r:
            continue
        msg = mav.m.recv_match(type="HOME_POSITION", blocking=False)
        if msg:
            return dict(lat=msg.latitude / 1e7, lon=msg.longitude / 1e7,
                        alt_m=msg.altitude / 1000.0)
    return None


def wait_global_position(mav, timeout=8.0):
    t0 = time.time()
    got = None
    while time.time() - t0 < timeout:
        r, _, _ = select.select([mav.m.port], [], [], 0.3)
        if not r:
            continue
        msg = mav.m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            got = dict(lat=msg.lat / 1e7, lon=msg.lon / 1e7,
                       rel_alt_m=msg.relative_alt / 1000.0,
                       amsl_alt_m=msg.alt / 1000.0, hdg_deg=msg.hdg / 100.0)
            if got["hdg_deg"] < 3600:
                return got
    return got


def invalidate_nav_telemetry(latest):
    """Drop the cached NAV_CONTROLLER_OUTPUT after a target or mode change.

    BUG FOUND LIVE 2026-09-07 on a harness smoke run, fixed here: `latest` is a
    last-value cache, so immediately after a new DO_REPOSITION the cached
    wp_dist/xtrack_error still describe the PREVIOUS target. Two concrete
    failures were observed: (1) scenario D leg 1 terminated after 0.0 s because
    the leg-capture stop condition fired on leg 0's final wp_dist of 90 m, so
    that leg was never flown at all; (2) the first sample of a leg reported the
    previous leg's wp_dist, which corrupted the closure metric
    (wp_dist_start read 169 m for a target that was 550 m away). Clearing the
    cache forces the first navigation sample of a new leg to come from the new
    target."""
    latest.pop("NAV_CONTROLLER_OUTPUT", None)


def send_guided_target(mav, lat, lon, rel_alt_m, R, tag):
    """MAV_CMD_DO_REPOSITION as a COMMAND_INT with
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT. param2 bit0 = MAV_DO_REPOSITION_FLAGS_
    CHANGE_MODE (GCS_MAVLink_Plane.cpp:588); we are already in GUIDED so the
    other branch of that same condition applies, but the flag is set anyway so
    the command cannot silently no-op if the mode switch raced."""
    res = command_int(mav, MAV_CMD_DO_REPOSITION, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                      p1=-1, p2=1, p3=0, p4=float("nan"),
                      x=int(round(lat * 1e7)), y=int(round(lon * 1e7)), z=rel_alt_m)
    R.setdefault("guided_targets", []).append(
        dict(tag=tag, lat=lat, lon=lon, rel_alt_m=rel_alt_m,
             command_ack_result=res,
             accepted=(res == mavutil.mavlink.MAV_RESULT_ACCEPTED)))
    return res == mavutil.mavlink.MAV_RESULT_ACCEPTED


# =============================================================================
# parameters + preconditions
# =============================================================================
def dump_params(mav, R):
    bulk = {}
    try:
        bulk = mav.fetch_all_params(timeout=45, idle_cutoff=3.0)
    except Exception as exc:                       # noqa: BLE001 - reported
        R["param_bulk_error"] = str(exc)
    got = {}
    for name in PARAMS_OF_INTEREST:
        v = bulk.get(name)
        if v is None:
            v = read_param(mav, name)
        got[name] = v
    R["params_live"] = got
    R["params_unreadable"] = sorted(k for k, v in got.items() if v is None)
    R["param_bulk_count"] = len(bulk)
    R["param_provenance"] = ("Read LIVE over MAVLink from the running vehicle. "
                             "config/ardupilot/falcon_v2_sitl.parm is read-only "
                             "input; THIS STAGE WROTE NO PARAMETER.")
    return got


def throttle_control_in(pwm, p):
    """RC_Channel::pwm_to_range_dz(), high_in=100 (radio.cpp set_range(100)),
    RC_Channel.cpp:388-402 - the same inversion the TECS stage uses."""
    rc_min, rc_max, dz = p["RC3_MIN"], p["RC3_MAX"], p["RC3_DZ"]
    v = min(max(pwm, rc_min), rc_max)
    if p.get("RC3_REVERSED"):
        v = rc_max - (v - rc_min)
    lo = rc_min + dz
    return 100.0 * (v - lo) / (rc_max - lo) if v > lo else 0.0


def preconditions(p, rc3_pwm, R):
    def eq(name, val, tol=1e-6):
        return p.get(name) is not None and abs(p[name] - val) < tol

    ci = throttle_control_in(rc3_pwm, p) if p.get("RC3_MIN") is not None else None
    chk = {
        "arspd_use_1": eq("ARSPD_USE", 1),
        "arspd_type_100_sitl": eq("ARSPD_TYPE", 100),
        "airspeed_min_16": eq("AIRSPEED_MIN", 16),
        "airspeed_cruise_18": eq("AIRSPEED_CRUISE", 18),
        "airspeed_max_28": eq("AIRSPEED_MAX", 28),
        "ptch_trim_deg_2p49": eq("PTCH_TRIM_DEG", 2.49, 1e-3),
        "tecs_ptch_damp_0p6": eq("TECS_PTCH_DAMP", 0.6, 1e-3),
        "pids_unchanged": all([eq("RLL_RATE_P", 0.25), eq("RLL_RATE_I", 0.125),
                               eq("RLL_RATE_D", 0.002), eq("RLL_RATE_FF", 0.125),
                               eq("PTCH_RATE_P", 0.25), eq("PTCH_RATE_I", 0.125),
                               eq("PTCH_RATE_D", 0.002), eq("PTCH_RATE_FF", 0.125)]),
        "kff_thr2ptch_zero": eq("KFF_THR2PTCH", 0),
        "flight_options_zero": eq("FLIGHT_OPTIONS", 0),
        "fence_disabled": eq("FENCE_ENABLE", 0),
        "terrain_follow_off": eq("TERRAIN_FOLLOW", 0),
        "min_groundspeed_zero": eq("MIN_GROUNDSPEED", 0),
        "sim_wind_zero": (p.get("SIM_WIND_SPD") is None or abs(p["SIM_WIND_SPD"]) < 1e-6),
        "sim_opos_alt_zero_atmosphere_datum": eq("SIM_OPOS_ALT", 0),
        "rc_not_reversed": all(eq(f"RC{i}_REVERSED", 0) for i in (1, 2, 3, 4)),
        # STICK_MIXING is 1 (ArduPlane default). That is HARMLESS here only
        # because RC1/RC2/RC4 are held exactly at their TRIM: get_control_in()
        # is then 0 and the mix contributes nothing. Assert the premise rather
        # than assert the conclusion.
        "sticks_at_trim_so_stick_mixing_inert": all(eq(f"RC{i}_TRIM", 1500) for i in (1, 2, 4)),
        # THROTTLE_NUDGE (default 1) raises the TECS target airspeed whenever
        # channel_throttle->get_control_in() > 50 (radio.cpp:143-152). The RC3
        # PWM this test holds must therefore stay at or below control_in 50, or
        # the commanded cruise speed would be silently nudged upward.
        "throttle_nudge_inactive": (ci is not None and ci <= 50.0),
    }
    R["preconditions"] = chk
    R["throttle_stick_control_in"] = ci
    R["throttle_nudge_note"] = (
        "THROTTLE_NUDGE live value %s. Nudge is active only for control_in > 50 "
        "(ArduPlane/radio.cpp:143-152); this run holds RC3=%s us -> control_in "
        "%.2f." % (p.get("THROTTLE_NUDGE"), rc3_pwm, ci if ci is not None else float("nan")))
    return chk


# =============================================================================
# sampling
# =============================================================================
MSG_IDS_20HZ = list(campaign.MSG_IDS_20HZ)


def build_nav_sample(t_rel, latest, sub, osub, adiag, pdiag, aerodiag, ptch_trim_deg):
    """campaign.build_sample() + everything navigation-specific. Nothing from
    campaign.build_sample() is removed or altered."""
    s = campaign.build_sample(t_rel, latest, sub, osub, adiag, pdiag, aerodiag)
    nav = latest.get("NAV_CONTROLLER_OUTPUT")
    vfr = latest.get("VFR_HUD")
    gpi = latest.get("GLOBAL_POSITION_INT")
    s["mav"]["nav_bearing_deg"] = (nav.nav_bearing if nav else None)
    s["mav"]["target_bearing_deg"] = (nav.target_bearing if nav else None)
    s["mav"]["wp_dist_m"] = (nav.wp_dist if nav else None)
    s["mav"]["xtrack_error_m"] = (nav.xtrack_error if nav else None)
    s["mav"]["nav_alt_error_m"] = (nav.alt_error if nav else None)
    s["mav"]["vfr_heading_deg"] = (vfr.heading if vfr else None)
    s["mav"]["gpi_hdg_deg"] = ((gpi.hdg / 100.0) if gpi and gpi.hdg < 36000 else None)
    s["mav"]["lat"] = ((gpi.lat / 1e7) if gpi else None)
    s["mav"]["lon"] = ((gpi.lon / 1e7) if gpi else None)

    # ---- Gazebo ground-truth derived quantities (frames per docstring sec 3)
    d = {}
    att = s["gz"]["att_deg"]
    if att is not None:
        d["roll_deg"] = att[0]
        d["pitch_phys_deg"] = -att[1]          # gz euler pitch is nose-DOWN +
        d["yaw_deg"] = att[2]
        d["heading_compass_deg"] = (90.0 - att[2]) % 360.0
    pose = sub.latest()
    od = osub.latest()
    if pose is not None and od is not None:
        q = (pose[4], pose[5], pose[6], pose[7])
        vw = base.rotate_body_to_world(q, (od.twist.linear.x, od.twist.linear.y,
                                           od.twist.linear.z))
        d["v_world_enu"] = list(vw)
        d["groundspeed_gz_ms"] = math.hypot(vw[0], vw[1])
        if d["groundspeed_gz_ms"] > 1.0:
            d["course_compass_deg"] = math.degrees(math.atan2(vw[0], vw[1])) % 360.0
        d["climb_rate_gz_ms"] = vw[2]
    # demanded attitude, physical
    if s["mav"]["nav_pitch_deg"] is not None:
        d["pitch_demand_phys_deg"] = s["mav"]["nav_pitch_deg"] + ptch_trim_deg
    d["roll_demand_deg"] = s["mav"]["nav_roll_deg"]
    # tracking errors (autopilot demand minus achieved physical attitude)
    if d.get("roll_demand_deg") is not None and d.get("roll_deg") is not None:
        d["roll_err_deg"] = d["roll_demand_deg"] - d["roll_deg"]
    if d.get("pitch_demand_phys_deg") is not None and d.get("pitch_phys_deg") is not None:
        d["pitch_err_deg"] = d["pitch_demand_phys_deg"] - d["pitch_phys_deg"]

    # ---- control surfaces, degrees, plus the REAL clamp flags -------------
    a = s["actuators"]
    if a:
        surf = {}
        clamped_any = False
        for name in SURFACES:
            e = a[name]
            deg = math.degrees(e["actual_angle_rad"])
            cl = bool(e["target_clamp_active"]) or bool(e["effort_clamp_active"])
            at_limit = abs(deg) >= (TH_SURF_HARD_LIMIT_DEG - TH_SURF_PIN_MARGIN_DEG)
            surf[name] = dict(deg=deg, target_clamp=bool(e["target_clamp_active"]),
                              effort_clamp=bool(e["effort_clamp_active"]),
                              at_hard_limit=bool(cl or at_limit))
            clamped_any = clamped_any or surf[name]["at_hard_limit"]
        d["surfaces_deg"] = {k: v["deg"] for k, v in surf.items()}
        d["surface_at_hard_limit"] = {k: v["at_hard_limit"] for k, v in surf.items()}
        d["any_surface_at_hard_limit"] = clamped_any
        # roll/pitch/yaw command channels as the pilot would read them
        d["aileron_deg"] = max(abs(surf["left_aileron"]["deg"]),
                               abs(surf["right_aileron"]["deg"]))
        d["elevator_deg"] = max(abs(surf["left_elevator"]["deg"]),
                                abs(surf["right_elevator"]["deg"]))
        d["rudder_deg"] = abs(surf["rudder"]["deg"])
    # ---- propulsion ground truth -----------------------------------------
    pr = s["propulsion"]
    if pr:
        d["prop_rpm"] = [pr["left"]["rpm"], pr["right"]["rpm"]]
        d["prop_thrust_N"] = [pr["left"]["thrust_N"], pr["right"]["thrust_N"]]
        d["prop_J"] = [pr["left"]["J"], pr["right"]["J"]]
        d["throttle_actual"] = 0.5 * (pr["left"]["throttle"] + pr["right"]["throttle"])
    if s["aero"]:
        d["airspeed_aero_ms"] = s["aero"].get("V")
    s["derived"] = d
    return s


def run_nav_segment(mav, sub, osub, adiag, pdiag, aerodiag, label, duration_s,
                    rc1, rc2, rc3, t0_flight, latest, ptch_trim_deg, stop_fn=None):
    samples = []
    aborted, abort_reason = False, None
    stopped_early, stop_reason = False, None
    t0 = time.time()
    last_rc = last_s = -1.0
    while time.time() - t0 < duration_s:
        tnow = time.time() - t0
        campaign.drain_mavlink(mav, latest)
        if tnow - last_rc >= campaign.RC_REFRESH_PERIOD:
            # RC is republished at neutral throughout. In GUIDED/LOITER/RTL
            # this sets NO attitude and NO throttle (see preconditions
            # `sticks_at_trim_so_stick_mixing_inert` and
            # `throttle_nudge_inactive`); it exists only to keep the RC link
            # alive (RC_OVERRIDE_TIME = 3.0 s).
            mav.send_rc_override(rc1=int(round(rc1)), rc2=int(round(rc2)),
                                 rc3=int(round(rc3)), rc4=1500, rc5=1000)
            last_rc = tnow
        if tnow - last_s >= campaign.SAMPLE_PERIOD:
            s = build_nav_sample(time.time() - t0_flight, latest, sub, osub,
                                 adiag, pdiag, aerodiag, ptch_trim_deg)
            s["t_seg"] = tnow
            samples.append(s)
            last_s = tnow
            att, pos = s["gz"]["att_deg"], s["gz"]["pos"]
            bad = None
            if att is not None:
                if not (math.isfinite(att[0]) and math.isfinite(att[1])):
                    bad = "nonfinite_attitude"
                elif abs(att[0]) > TH_ATT_ABORT_DEG or abs(att[1]) > TH_ATT_ABORT_DEG:
                    bad = "attitude_envelope"
            if pos is not None and bad is None:
                if not math.isfinite(pos[2]):
                    bad = "nonfinite_altitude"
                elif pos[2] < TH_ALT_FLOOR_M:
                    bad = "altitude_floor"
                elif pos[2] > TH_ALT_CEILING_M:
                    bad = "altitude_ceiling"
            if bad is not None:
                aborted = True
                abort_reason = dict(reason=bad, t=s["t"], att_deg=att, pos=pos)
                break
            if stop_fn is not None:
                ok, why = stop_fn(s, samples)
                if ok:
                    stopped_early, stop_reason = True, why
                    break
        time.sleep(0.005)
    return dict(label=label, duration_s=duration_s,
                actual_duration_s=time.time() - t0, rc1=rc1, rc2=rc2, rc3=rc3,
                n_samples=len(samples), samples=samples,
                aborted=aborted, abort_reason=abort_reason,
                stopped_early=stopped_early, stop_reason=stop_reason)


# =============================================================================
# ANALYSIS
# =============================================================================
def col(samples, fn):
    ts, ys = [], []
    for s in samples:
        try:
            v = fn(s)
        except (KeyError, TypeError, IndexError):
            v = None
        if v is None or not finite(v):
            continue
        ts.append(s["t"])
        ys.append(v)
    return ts, ys


def steady_slice(samples, transient_s):
    """Drop the first `transient_s` seconds of a window. Used for every gate
    that is about sustained behaviour rather than the command transient."""
    if not samples:
        return []
    t0 = samples[0]["t"]
    return [s for s in samples if s["t"] - t0 >= transient_s]


def surface_report(samples, label):
    """Per-surface deflection statistics + hard-limit pinning detection.
    Pinning uses the actuator plugin's OWN clamp flags (target_clamp_active /
    effort_clamp_active), not an inferred angle comparison, plus an
    angle-within-margin test as a redundant detector."""
    out = {"label": label, "n": len(samples)}
    if not samples:
        return out
    for chan, key in (("aileron", "aileron_deg"), ("elevator", "elevator_deg"),
                      ("rudder", "rudder_deg")):
        _, ys = col(samples, lambda s, k=key: s["derived"].get(k))
        if not ys:
            out[chan] = None
            continue
        out[chan] = dict(
            n=len(ys), mean_abs=mean(ys), p50=percentile(ys, 50),
            percentile_used=TH_SURF_PERCENTILE,
            p95=percentile(ys, TH_SURF_PERCENTILE), p99=percentile(ys, 99),
            max_abs=max(ys),
            frac_over_linear=sum(1 for y in ys if y > TH_SURF_LINEAR_DEG) / len(ys))
    per_surf = {}
    for name in SURFACES:
        ts, flags = col(samples, lambda s, n=name: (
            1.0 if s["derived"].get("surface_at_hard_limit", {}).get(n) else 0.0))
        if not flags:
            per_surf[name] = None
            continue
        per_surf[name] = dict(
            n=len(flags),
            hard_limit_duty=sum(flags) / len(flags),
            hard_limit_longest_run_s=longest_true_run_s(ts, [f > 0.5 for f in flags]))
    out["hard_limit_per_surface"] = per_surf
    duties = [v["hard_limit_duty"] for v in per_surf.values() if v]
    runs = [v["hard_limit_longest_run_s"] for v in per_surf.values() if v]
    out["hard_limit_duty_max"] = max(duties) if duties else None
    out["hard_limit_longest_run_s"] = max(runs) if runs else None
    out["hard_limit_deg"] = TH_SURF_HARD_LIMIT_DEG
    return out


def theil_sen_slope(xs, ys):
    """Median of all pairwise slopes - a robust linear slope with a ~29 %
    breakdown point.

    Used instead of least squares for the log-peak envelope fit. This is where
    validation MINOR-1's intent now lives: least squares has a 0 % breakdown
    point, so ONE large intermediate peak could dominate the fit - which was the
    whole reason a bolt-on endpoint guard was introduced. Fixing it in the
    ESTIMATOR removes the need for that guard and, unlike the guard, does not
    manufacture false negatives on a genuine divergence that happens to sit on
    top of a large early transient."""
    n = len(xs)
    if n < 2:
        return None
    sl = []
    for i in range(n - 1):
        xi, yi = xs[i], ys[i]
        for j in range(i + 1, n):
            dx = xs[j] - xi
            if dx != 0:
                sl.append((ys[j] - yi) / dx)
    if not sl:
        return None
    sl.sort()
    m = len(sl) // 2
    return sl[m] if len(sl) % 2 else 0.5 * (sl[m - 1] + sl[m])


def _provisional(out, peaks, ts, ys):
    """Attach a NON-GATING provisional growth estimate to a channel whose
    envelope fit was judged not identifiable.

    An ungated channel must never be a SILENT hole. Gating it off says "this
    window cannot support a growth verdict"; it must not also mean "nobody can
    see what the number would have been". These fields let a reviewer confirm
    at a glance that an ungated channel is nowhere near growing - or notice
    immediately if it is. They are advisory and are never used in any verdict.
    The project's own validated detrended_growth() is included as an
    independent second opinion."""
    try:
        pt = [t for t, _ in peaks]
        pa = [math.log(a) for _, a in peaks if a > 0]
        if len(pa) == len(pt) and len(pt) >= 2:
            sig = theil_sen_slope(pt, pa)
            dur = pt[-1] - pt[0]
            out["provisional_sigma_per_s_NON_GATING"] = sig
            out["provisional_envelope_ratio_NON_GATING"] = (
                math.exp(sig * dur) if sig is not None else None)
        out["provisional_detrended_growth_NON_GATING"] = detrended_growth(ts, ys)
        out["provisional_note"] = (
            "ADVISORY ONLY - this channel is NOT gated on the oscillation "
            "metric because its envelope fit is not identifiable from this "
            "window. These numbers exist so the exclusion is transparent "
            "rather than a silent gap. They never enter a verdict.")
    except Exception as exc:                      # noqa: BLE001 - advisory only
        out["provisional_error"] = str(exc)


def robust_detrend(ts, ys, maxpts=240):
    """Robust linear trend (Theil-Sen slope on an evenly spaced subsample,
    median-residual intercept). Returns (slope, intercept).

    WHY NOT LEAST SQUARES. The trend is removed before half-cycle peaks are
    found, so a tilted trend line adds a ramp to the residual and inflates the
    later half-cycle peaks - which reads as envelope growth that is not there.
    Least squares has a 0 % breakdown point, so a single large transient tilts
    it. Measured on a fixture of a CONSTANT-amplitude 2 deg oscillation plus one
    6 s 25 deg burst: the least-squares trend was -0.00964 /s and the channel
    was reported growing (envelope ratio 1.917); the Theil-Sen trend is
    -0.00247 /s and it is correctly reported NOT growing (1.107).
    Verified on the real campaign channels: this changes ZERO verdicts there,
    and it IMPROVES injection sensitivity (see the ROBUSTNESS NOTE).
    Subsampled to bound the O(n^2) pair count; the subsample is evenly spaced,
    so it is unbiased with respect to time."""
    n = len(ts)
    if n < 2:
        return None, None
    step = max(1, n // maxpts)
    sl = theil_sen_slope(ts[::step], ys[::step])
    if sl is None:
        return linreg(ts, ys)
    res = sorted(y - sl * t for t, y in zip(ts, ys))
    m = len(res) // 2
    ic = res[m] if len(res) % 2 else 0.5 * (res[m - 1] + res[m])
    return sl, ic


def envelope_growth(ts, ys, label):
    """Peak-envelope growth rate of an oscillation. See the ESTIMATOR NOTE.

    Method: remove the ROBUST linear trend (robust_detrend); walk the residual
    and, between consecutive
    zero crossings, record the largest |residual| as one half-cycle peak; drop
    peaks below TH_OSC_PEAK_PROMINENCE_MEDIAN_FRAC of the MEDIAN half-cycle
    peak (a ROBUST amplitude scale - see the ROBUSTNESS NOTE); fit
    ln|peak| = sigma*t + c. sigma > 0 means a growing envelope. The window
    growth factor exp(sigma*T) is compared with TH_OSC_ENVELOPE_RATIO.

    A monotonic manoeuvring trend yields < TH_OSC_MIN_PEAKS half-cycle peaks and
    is reported NOT_OSCILLATORY - which is the correct answer, not a pass."""
    out = {"label": label, "n": len(ys)}
    if len(ys) < 20:
        out["status"] = "INSUFFICIENT_SAMPLES"
        out["growing"] = False
        return out
    slope, icpt = robust_detrend(ts, ys)
    if slope is None:
        out["status"] = "DEGENERATE"
        out["growing"] = False
        return out
    out["detrend_estimator"] = "Theil-Sen slope, median-residual intercept (robust)"
    out["detrend_slope_per_s"] = slope
    out["detrend_slope_least_squares_REPORTED_ONLY"] = linreg(ts, ys)[0]
    r = [y - (slope * t + icpt) for t, y in zip(ts, ys)]
    out["detrended_p2p"] = max(r) - min(r)
    # Pass 1: EVERY half-cycle peak, unfiltered, so the amplitude scale is
    # derived from the peak population itself rather than from its extremes.
    raw = []
    seg_max, seg_t, sign = 0.0, None, None
    for t, v in zip(ts, r):
        sg = 1 if v >= 0 else -1
        if sign is None:
            sign = sg
        if sg != sign:
            if seg_t is not None:
                raw.append((seg_t, seg_max))
            seg_max, seg_t, sign = 0.0, None, sg
        if abs(v) > seg_max:
            seg_max, seg_t = abs(v), t
    if seg_t is not None:
        raw.append((seg_t, seg_max))
    out["n_raw_half_cycle_peaks"] = len(raw)
    if len(raw) < 2:
        out["status"] = "NOT_OSCILLATORY"
        out["n_half_cycle_peaks"] = len(raw)
        out["growing"] = False
        return out
    # Pass 2: filter against a ROBUST amplitude scale (median peak), never
    # against the peak-to-peak extreme. See the ROBUSTNESS NOTE.
    amps = sorted(a for _, a in raw)
    med_peak = (amps[len(amps) // 2] if len(amps) % 2
                else 0.5 * (amps[len(amps) // 2 - 1] + amps[len(amps) // 2]))
    prom = TH_OSC_PEAK_PROMINENCE_MEDIAN_FRAC * med_peak
    out["median_half_cycle_peak"] = med_peak
    peaks = [(t, a) for t, a in raw if a >= prom]
    out["n_half_cycle_peaks"] = len(peaks)
    out["peak_prominence_threshold"] = prom
    out["min_peaks_required"] = TH_OSC_MIN_PEAKS
    if len(peaks) < TH_OSC_MIN_PEAKS:
        # <2 peaks: there is no oscillation at all (a monotonic manoeuvring
        # trend). 2..MIN-1 peaks: something oscillatory exists but fewer than
        # 3 complete cycles were observed, so an exponential envelope fit is
        # NOT IDENTIFIABLE - report that honestly instead of guessing.
        out["status"] = ("NOT_OSCILLATORY" if len(peaks) < 2
                         else "UNDERDETERMINED_TOO_FEW_CYCLES")
        out["growing"] = False
        if len(peaks) >= 2:
            out["cycles_observed"] = len(peaks) / 2.0
            out["first_peak"] = peaks[0][1]
            out["last_peak"] = peaks[-1][1]
            out["peak_max"] = max(a for _, a in peaks)
            _provisional(out, peaks, ts, ys)
        return out
    pt = [t for t, _ in peaks]
    pa = [a for _, a in peaks]
    dur = pt[-1] - pt[0]
    analysis_span = ts[-1] - ts[0]
    dts = [pt[i] - pt[i - 1] for i in range(1, len(pt))]
    span_frac = (dur / analysis_span) if analysis_span > 0 else 0.0
    # ROBUST coherence statistic: the largest single gap as a fraction of the
    # interval the peaks span. Does not depend on min(spacing), so a brief
    # zero-crossing excursion cannot dominate it. For a uniform train of n
    # peaks it equals 1/(n-1) and therefore relaxes as evidence accumulates.
    max_gap_frac = (max(dts) / dur) if (dts and dur > 0) else None
    out["analysis_span_s"] = analysis_span
    out["peak_span_s"] = dur
    out["peak_span_frac"] = span_frac
    out["half_cycle_spacings_s"] = dts
    out["max_gap_frac"] = max_gap_frac
    out["uniform_train_reference_gap_frac"] = (1.0 / (len(peaks) - 1)
                                               if len(peaks) > 1 else None)
    # RETIRED as a gate, kept as a diagnostic so the regression that motivated
    # its removal stays visible in the record. DO NOT gate on this.
    out["spacing_ratio_max_over_min_REPORTED_ONLY"] = (
        (max(dts) / min(dts)) if dts and min(dts) > 0 else None)
    out["min_peak_span_frac_required"] = TH_OSC_MIN_PEAK_SPAN_FRAC
    out["max_gap_frac_allowed"] = TH_OSC_MAX_GAP_FRAC
    out["cycles_observed"] = len(peaks) / 2.0
    out["first_peak"] = pa[0]
    out["last_peak"] = pa[-1]
    out["peak_max"] = max(pa)
    # ---- COHERENCE GATE (validation MINOR-2) -----------------------------
    # Enough peaks is not enough. They must also cover most of the window being
    # reported on, and be spaced like one oscillation rather than like a
    # handful of unrelated transients. Failing either means the envelope growth
    # rate is not identifiable from this window - report that, do not guess.
    incoherent = []
    if span_frac < TH_OSC_MIN_PEAK_SPAN_FRAC:
        incoherent.append("peak_span_frac %.3f < %.2f"
                          % (span_frac, TH_OSC_MIN_PEAK_SPAN_FRAC))
    if max_gap_frac is None or max_gap_frac > TH_OSC_MAX_GAP_FRAC:
        incoherent.append("max_gap_frac %s > %.2f"
                          % (("None" if max_gap_frac is None
                              else "%.3f" % max_gap_frac), TH_OSC_MAX_GAP_FRAC))
    if incoherent:
        out["status"] = "UNDERDETERMINED_INCOHERENT"
        out["incoherence_reasons"] = incoherent
        out["growing"] = False
        _provisional(out, peaks, ts, ys)
        return out
    logpa = [math.log(a) for a in pa]
    sigma = theil_sen_slope(pt, logpa)
    out["sigma_estimator"] = "Theil-Sen median pairwise slope (robust)"
    out["sigma_least_squares_REPORTED_ONLY"] = linreg(pt, logpa)[0]
    out["status"] = "OSCILLATORY"
    out["sigma_per_s"] = sigma
    out["window_s"] = dur
    out["envelope_ratio"] = math.exp(sigma * dur) if sigma is not None else None
    # mean half-cycle spacing -> period -> an approximate damping ratio, so the
    # result is comparable with the phugoid stage's own zeta numbers.
    half_T = mean(dts) if dts else None
    if half_T and half_T > 0:
        period = 2.0 * half_T
        omega = 2.0 * math.pi / period
        out["period_s"] = period
        out["omega_rad_s"] = omega
        if sigma is not None:
            out["zeta_estimate"] = -sigma / math.sqrt(sigma * sigma + omega * omega)
            out["tau_s"] = (-1.0 / sigma) if sigma < 0 else None
    # ---- ENDPOINT RATIO: RETIRED AS A GATE 2026-09-08, kept as a diagnostic.
    # It was introduced (validation MINOR-1) because a least-squares fit could
    # be dominated by one large intermediate peak. That concern is now handled
    # in the ESTIMATOR (theil_sen_slope + a robust prominence scale), which is
    # the right place for it. Measured on the real campaign channels, the
    # endpoint rule was load-bearing in ZERO cases - it never changed a verdict
    # on real data - while on injected genuine divergences it produced FALSE
    # NEGATIVES: a growing mode riding on top of a large early transient (e.g.
    # the RTL turn-in in scenario C) has last_peak/first_peak ~ 1 even when the
    # tail is plainly diverging, so the rule suppressed detection of exactly the
    # case a divergence gate exists to catch. MINOR-1's motivating shape,
    # re-injected on top of a real channel, is now rejected by the fit alone
    # (envelope ratio 0.373, far below 1.30). Reported, not gated.
    out["endpoint_ratio"] = (pa[-1] / pa[0]) if pa[0] > 0 else None
    out["endpoint_ratio_reference_REPORTED_ONLY"] = TH_OSC_ENDPOINT_RATIO_MIN
    out["endpoint_ratio_is_gating"] = False
    out["growing"] = bool(out["envelope_ratio"] is not None
                          and out["envelope_ratio"] >= TH_OSC_ENVELOPE_RATIO)
    out["growth_criterion"] = (
        ">= %d half-cycle peaks AND peak_span_frac >= %.2f AND max_gap_frac "
        "<= %.2f AND exp(sigma_TheilSen * T) >= %.1f"
        % (TH_OSC_MIN_PEAKS, TH_OSC_MIN_PEAK_SPAN_FRAC,
           TH_OSC_MAX_GAP_FRAC, TH_OSC_ENVELOPE_RATIO))
    return out


def oscillation_report(samples, gating_exclude=()):
    """Growth metrics on every axis that could diverge.

    `gating_exclude` demotes a channel to report-only where the CONCATENATED
    series is not a physically continuous signal. The only current use is
    xtrack_error_m in the multi-target route (scenario D): wp_dist - and hence
    xtrack_error - RESETS at every waypoint change, so the concatenated series
    is a sawtooth whose "peaks" are leg boundaries, not oscillation. Measured on
    the 2026-09-07 harness smoke run: detrended p2p 492 m over a signal that
    ranged 50-547 m across three separate 550 m legs. Detection capability is
    NOT lost - route_extra() runs the SAME envelope metric PER LEG, where the
    target is constant and the signal is continuous. Uses
    fbwa.detrended_growth() VERBATIM (same criterion as the validated phugoid
    and TECS stages) so the numbers are directly comparable across stages."""
    out = {}
    # ---- CHANNEL SELECTION, and why the GATED channels are ERROR signals ----
    # An oscillation gate is only well posed on a signal whose COMMANDED value
    # is constant over the window. In navigation flight the raw states are not:
    # the navigator commands roll, TECS commands pitch, and scenario D commands
    # a deliberate +15 m altitude step. A linear detrend cannot remove a
    # commanded step or a manoeuvre, so gating raw altitude flagged a clean,
    # correctly-tracked +15 m climb as a "growing oscillation" on the
    # 2026-09-07 harness smoke run (envelope ratio 3.52 on a monotonic step).
    # The commanded value of an ERROR signal, by contrast, is identically zero
    # in every window and every flight condition - so the error channels below
    # are well posed unconditionally, and they are exactly what "is the control
    # loop oscillating or diverging?" actually asks.
    # Divergence of the raw STATES is not lost: it is covered by the separate
    # boundedness gates (altitude_bounded, airspeed_bounded, airspeed floor,
    # attitude/altitude run-abort envelope). The raw states are still analysed
    # with the same envelope metric and reported.
    chans = {
        # --- gated: error signals, commanded value identically zero ---
        "roll_err_deg": lambda s: s["derived"].get("roll_err_deg"),
        "pitch_err_deg": lambda s: s["derived"].get("pitch_err_deg"),
        "alt_error_m": lambda s: s["mav"].get("nav_alt_error_m"),
        "aspd_error_ms": lambda s: (None if s["mav"].get("nav_aspd_error") is None
                                    else s["mav"]["nav_aspd_error"] / 100.0),
        "xtrack_error_m": lambda s: s["mav"].get("xtrack_error_m"),
        # --- reported only: raw states (commanded value is NOT constant) ---
        "roll_deg": lambda s: s["derived"].get("roll_deg"),
        "pitch_phys_deg": lambda s: s["derived"].get("pitch_phys_deg"),
        "altitude_m": lambda s: (s["gz"]["pos"][2] if s["gz"]["pos"] else None),
        "airspeed_ms": lambda s: s["mav"]["airspeed"],
    }
    gating_channels = tuple(c for c in ("roll_err_deg", "pitch_err_deg",
                                       "alt_error_m", "aspd_error_ms",
                                       "xtrack_error_m")
                            if c not in gating_exclude)
    growing, growing_reported_only = [], []
    for name, fn in chans.items():
        ts, ys = col(samples, fn)
        if len(ys) < 8:
            out[name] = dict(n=len(ys), note="too few samples")
            continue
        env = envelope_growth(ts, ys, name)
        lin = detrended_growth(ts, ys)
        env["linear_detrend_growth_reported_only"] = lin
        env["series"] = series_report(ts, ys, name)
        env["gating"] = name in gating_channels
        out[name] = env
        if env.get("growing"):
            (growing if name in gating_channels else growing_reported_only).append(name)
        if str(env.get("status", "")).startswith("UNDERDETERMINED"):
            out.setdefault("underdetermined_channels", []).append(name)
        if lin.get("growing"):
            out.setdefault("linear_detrend_flagged_channels", []).append(name)
    out["growing_channels"] = growing
    out["growing_channels_non_gating"] = growing_reported_only
    out["any_growing"] = bool(growing)
    out["gating_channels"] = list(gating_channels)
    out["gating_excluded_channels"] = list(gating_exclude)
    out.setdefault("underdetermined_channels", [])
    # Every ungated channel carries an advisory, NON-GATING growth estimate so
    # that gating it off is never a silent hole in the record.
    out["underdetermined_provisional_summary"] = {
        ch: {"envelope_ratio": out[ch].get("provisional_envelope_ratio_NON_GATING"),
             "sigma_per_s": out[ch].get("provisional_sigma_per_s_NON_GATING"),
             "detrended_growth_ratio": (
                 (out[ch].get("provisional_detrended_growth_NON_GATING") or {})
                 .get("ratio_second_over_first")),
             "reasons": out[ch].get("incoherence_reasons") or out[ch].get("status")}
        for ch in out.get("underdetermined_channels", []) if ch in out}
    out["underdetermined_note"] = (
        "Channels whose envelope growth rate is NOT IDENTIFIABLE from this "
        "window: either fewer than %d half-cycle peaks (< 3 complete cycles) -> "
        "UNDERDETERMINED_TOO_FEW_CYCLES, or peaks covering less than %.0f %% of "
        "the window, or a single gap covering more than %.0f %% of the peak "
        "span -> UNDERDETERMINED_INCOHERENT. They are neither passed nor failed on the "
        "oscillation metric, and are listed here so the gap is visible rather "
        "than silent; raw-state divergence over such a window is covered by the "
        "boundedness and run-abort gates."
        % (TH_OSC_MIN_PEAKS, 100 * TH_OSC_MIN_PEAK_SPAN_FRAC,
           100 * TH_OSC_MAX_GAP_FRAC))
    out["gated_channels_are_error_signals_because"] = (
        "the commanded value of an error signal is identically zero in every "
        "window, so the oscillation-envelope metric is well posed regardless of "
        "manoeuvring or of a commanded altitude step. Raw-state divergence is "
        "covered by the separate boundedness gates.")
    out["criterion"] = (
        "GATE: envelope_growth() - detrend, split at zero crossings, fit "
        "ln|half-cycle peak| = sigma*t + c; growing if exp(sigma*T) >= %.1f. "
        "REPORTED ONLY: fbwa.detrended_growth() half-vs-half residual std ratio "
        ">= %.1f, which is not well posed for manoeuvring flight - see the "
        "ESTIMATOR NOTE in this module."
        % (TH_OSC_ENVELOPE_RATIO, TH_OSC_GROWTH_RATIO))
    return out


def tracking_report(samples):
    """Roll / pitch attitude-tracking error statistics. This is the primary
    AUTOTUNE DECISION SUPPORT product - it is NOT a gating quantity."""
    out = {}
    for name, key in (("roll", "roll_err_deg"), ("pitch", "pitch_err_deg")):
        _, ys = col(samples, lambda s, k=key: s["derived"].get(k))
        if not ys:
            out[name] = None
            continue
        ab = [abs(y) for y in ys]
        out[name] = dict(n=len(ys), mean=mean(ys), std=stdev(ys), rms=rms(ys),
                         mean_abs=mean(ab), p95_abs=percentile(ab, 95),
                         max_abs=max(ab))
    for name, dkey, akey in (("roll", "roll_demand_deg", "roll_deg"),
                             ("pitch", "pitch_demand_phys_deg", "pitch_phys_deg")):
        _, d = col(samples, lambda s, k=dkey: s["derived"].get(k))
        _, a = col(samples, lambda s, k=akey: s["derived"].get(k))
        out[f"{name}_demand"] = minmaxmean(d)
        out[f"{name}_actual"] = minmaxmean(a)
    return out


def frame_report(samples, p):
    """Independent frame/sign cross-checks. These are REGRESSION TRIPWIRES:
    a failure means a coordinate-convention problem, not a tuning problem."""
    out = {}
    # gz-derived compass heading vs ArduPlane's own reported heading
    diffs = []
    for s in samples:
        h_gz = s["derived"].get("heading_compass_deg")
        h_ap = s["mav"].get("gpi_hdg_deg")
        if h_gz is None or h_ap is None:
            continue
        diffs.append(abs(wrap180(h_gz - h_ap)))
    out["heading_gz_vs_ardupilot_absdiff_deg"] = minmaxmean(diffs)
    out["heading_frame_consistent"] = bool(
        diffs and percentile(diffs, 95) <= TH_HEADING_FRAME_TOL_DEG)
    out["heading_conversion"] = ("heading_compass = (90 - gz_yaw_deg) mod 360; "
                                 "world is ENU with heading_deg 0 so gz +X = East, "
                                 "+Y = North")
    # roll sign agreement between Gazebo ground truth and MAVLink
    pairs = [(s["derived"].get("roll_deg"), s["mav"].get("att_roll_deg"))
             for s in samples]
    pairs = [(a, b) for a, b in pairs if a is not None and b is not None
             and abs(a) > 3.0]
    agree = sum(1 for a, b in pairs if a * b > 0)
    out["roll_sign_gz_mav_agree_frac"] = (agree / len(pairs)) if pairs else None
    out["roll_sign_gz_mav_agree"] = bool(pairs and agree / len(pairs) > 0.98)
    out["roll_sign_n_compared"] = len(pairs)
    # source-derived identity: xtrack_error == wp_dist - loiter_radius
    rad = p.get("WP_LOITER_RAD")
    ids = []
    if rad is not None:
        for s in samples:
            wd, xt = s["mav"].get("wp_dist_m"), s["mav"].get("xtrack_error_m")
            if wd is None or xt is None:
                continue
            ids.append(abs(xt - (wd - rad)))
    out["xtrack_identity_residual_m"] = minmaxmean(ids)
    out["xtrack_identity_consistent"] = bool(
        ids and percentile(ids, 95) <= TH_XTRACK_IDENTITY_TOL_M)
    out["xtrack_identity_note"] = (
        "AP_L1_Control.cpp:424-426 in the update_loiter() branch (the branch "
        "taken in GUIDED/LOITER/RTL because auto_state.crosstrack is false - "
        "commands.cpp:97, commands_logic.cpp:340) sets _crosstrack_error = "
        "A_air.length() - radius. wp_dist is a uint16 in whole metres, hence "
        "the %.1f m tolerance." % TH_XTRACK_IDENTITY_TOL_M)
    return out


def envelope_report(samples, p, transient_s):
    """Airspeed / altitude boundedness over the steady window, plus the
    whole-window hard floor."""
    out = {}
    st = steady_slice(samples, transient_s)
    a_min = p.get("AIRSPEED_MIN", TH_AIRSPEED_MIN_MS)
    hard = TH_AIRSPEED_HARD_FLOOR_FRAC * a_min
    ts, v_all = col(samples, lambda s: s["mav"]["airspeed"])
    ts_s, v_st = col(st, lambda s: s["mav"]["airspeed"])
    out["airspeed_whole_window"] = series_report(ts, v_all, "airspeed") if v_all else None
    out["airspeed_steady"] = series_report(ts_s, v_st, "airspeed_steady") if v_st else None
    out["airspeed_min_threshold_ms"] = a_min
    out["airspeed_hard_floor_ms"] = hard
    out["airspeed_min_steady_ms"] = (min(v_st) if v_st else None)
    out["airspeed_min_whole_ms"] = (min(v_all) if v_all else None)
    out["n_samples_below_min_steady"] = sum(1 for v in v_st if v < a_min)
    out["n_samples_below_hard_floor_whole"] = sum(1 for v in v_all if v < hard)
    out["airspeed_above_min_steady"] = bool(v_st and min(v_st) >= a_min)
    out["airspeed_above_hard_floor"] = bool(v_all and min(v_all) >= hard)
    out["airspeed_bounded"] = bool(v_st and (max(v_st) - min(v_st)) <= TH_AIRSPEED_P2P_MAX_MS)
    out["airspeed_p2p_steady_ms"] = ((max(v_st) - min(v_st)) if v_st else None)

    tz, z_st = col(st, lambda s: (s["gz"]["pos"][2] if s["gz"]["pos"] else None))
    out["altitude_steady"] = series_report(tz, z_st, "altitude_steady") if z_st else None
    out["altitude_p2p_steady_m"] = ((max(z_st) - min(z_st)) if z_st else None)
    out["altitude_bounded"] = bool(z_st and (max(z_st) - min(z_st)) <= TH_ALT_P2P_MAX_M)

    # groundspeed and throttle, recorded (report-only)
    _, gs = col(samples, lambda s: s["mav"]["groundspeed"])
    out["groundspeed_ms"] = minmaxmean(gs)
    _, thr = col(samples, lambda s: s["derived"].get("throttle_actual"))
    out["throttle_actual"] = minmaxmean(thr)
    _, rpm_l = col(samples, lambda s: (s["derived"]["prop_rpm"][0]
                                       if s["derived"].get("prop_rpm") else None))
    _, rpm_r = col(samples, lambda s: (s["derived"]["prop_rpm"][1]
                                       if s["derived"].get("prop_rpm") else None))
    out["prop_rpm_left"] = minmaxmean(rpm_l)
    out["prop_rpm_right"] = minmaxmean(rpm_r)
    _, thr_l = col(samples, lambda s: (s["derived"]["prop_thrust_N"][0]
                                       if s["derived"].get("prop_thrust_N") else None))
    _, thr_r = col(samples, lambda s: (s["derived"]["prop_thrust_N"][1]
                                       if s["derived"].get("prop_thrust_N") else None))
    out["prop_thrust_left_N"] = minmaxmean(thr_l)
    out["prop_thrust_right_N"] = minmaxmean(thr_r)
    _, jj = col(samples, lambda s: (max(s["derived"]["prop_J"])
                                    if s["derived"].get("prop_J") else None))
    out["prop_advance_ratio_J_max"] = minmaxmean(jj)
    out["prop_note"] = ("Propulsion numbers are RECORDED, not validated by this "
                        "stage. The high-advance-ratio/windmilling regime remains "
                        "OPEN / DATA_REQUIRED / NON-GATING (see "
                        "known_open_limitations).")
    return out


def nav_sign_report(samples, label):
    """Does the navigator turn the RIGHT WAY? Measured, never assumed.

    err = wrap180(target_bearing - heading). Positive err = the target lies to
    the RIGHT. A correct navigator commands nav_roll with the SAME sign (right
    bank = positive roll, docstring sec 3) and drives |err| toward zero.
    Samples with |err| below TH_NAV_SIGN_DEADBAND_DEG are excluded: their sign
    carries no navigation information."""
    out = {"label": label}
    votes, errs, ts = [], [], []
    for s in samples:
        tb = s["mav"].get("target_bearing_deg")
        hd = s["mav"].get("gpi_hdg_deg")
        nr = s["mav"].get("nav_roll_deg")
        if tb is None or hd is None or nr is None:
            continue
        e = wrap180(tb - hd)
        ts.append(s["t"])
        errs.append(e)
        if abs(e) < TH_NAV_SIGN_DEADBAND_DEG:
            continue
        votes.append(1.0 if (e * nr > 0) else 0.0)
    out["n_error_samples"] = len(errs)
    out["n_sign_votes"] = len(votes)
    out["sign_agreement_frac"] = (mean(votes) if votes else None)
    out["heading_error_abs"] = minmaxmean([abs(e) for e in errs]) if errs else None
    if errs:
        n = max(4, len(errs) // 5)
        out["heading_error_abs_first_fifth_mean"] = mean([abs(e) for e in errs[:n]])
        out["heading_error_abs_last_fifth_mean"] = mean([abs(e) for e in errs[-n:]])
        out["heading_error_converged"] = bool(
            out["heading_error_abs_last_fifth_mean"] <=
            out["heading_error_abs_first_fifth_mean"])
    out["nav_sign_correct"] = bool(
        out["sign_agreement_frac"] is not None
        and out["sign_agreement_frac"] >= TH_NAV_SIGN_AGREE_FRAC)
    out["convention"] = ("err = wrap180(target_bearing - heading); positive err "
                         "= target to the RIGHT; correct response is positive "
                         "nav_roll (right bank). Deadband %.1f deg."
                         % TH_NAV_SIGN_DEADBAND_DEG)
    # inner loop: does the achieved roll follow the commanded roll in sign?
    pairs = [(s["mav"].get("nav_roll_deg"), s["derived"].get("roll_deg"))
             for s in samples]
    pairs = [(a, b) for a, b in pairs if a is not None and b is not None and abs(a) > 5.0]
    out["inner_roll_sign_agree_frac"] = (
        (sum(1 for a, b in pairs if a * b > 0) / len(pairs)) if pairs else None)
    out["inner_roll_sign_correct"] = bool(
        out["inner_roll_sign_agree_frac"] is not None
        and out["inner_roll_sign_agree_frac"] >= TH_NAV_SIGN_AGREE_FRAC)
    return out


def approach_report(samples):
    """Distance-to-target closure. Uses ArduPlane's own wp_dist."""
    ts, d = col(samples, lambda s: s["mav"].get("wp_dist_m"))
    out = dict(n=len(d))
    if len(d) < 8:
        out["note"] = "too few samples"
        return out
    out["wp_dist_start_m"] = d[0]
    out["wp_dist_min_m"] = min(d)
    out["wp_dist_end_m"] = d[-1]
    out["closure_frac"] = ((d[0] - min(d)) / d[0]) if d[0] > 0 else None
    out["closure_ok"] = bool(out["closure_frac"] is not None
                             and out["closure_frac"] >= TH_APPROACH_CLOSURE_FRAC)
    out["wp_dist_series"] = series_report(ts, d, "wp_dist")
    return out


def analyze_scenario(scen, samples, p, extra=None, nav_samples=None,
                     approach_samples=None):
    """Common analysis applied identically to every scenario.

    nav_samples / approach_samples let a scenario restrict the navigation-sign
    and target-closure metrics to the window in which they are DEFINED. This
    is a measurement-validity restriction, not a way to hide data: the full
    record is analysed for every other metric and the restriction used is
    written into the result JSON (`nav_window_rule`)."""
    A = {"scenario": scen, "n_samples": len(samples)}
    if len(samples) < 20:
        A["insufficient_samples"] = True
        return A
    st = steady_slice(samples, TH_MODE_TRANSIENT_S)
    A["t_start"] = samples[0]["t"]
    A["t_end"] = samples[-1]["t"]
    A["duration_s"] = samples[-1]["t"] - samples[0]["t"]
    A["steady_window_start_offset_s"] = TH_MODE_TRANSIENT_S
    A["n_steady_samples"] = len(st)
    A["mode_seen"] = sorted({s["mav"]["custom_mode"] for s in samples
                             if s["mav"]["custom_mode"] is not None})
    A["mode_names_seen"] = [MODE_NAME.get(m, str(m)) for m in A["mode_seen"]]
    want = SCENARIOS[scen]["mode"]
    A["target_mode"] = want
    A["mode_entry_latency_allowance_s"] = MODE_ENTRY_LATENCY_S
    t_ok = samples[0]["t"] + MODE_ENTRY_LATENCY_S
    late = [s for s in samples if s["t"] >= t_ok and s["mav"]["custom_mode"] is not None]
    A["mode_stable_after_entry"] = bool(
        late and all(s["mav"]["custom_mode"] == want for s in late))
    A["mode_frac_on_target"] = (
        (sum(1 for s in samples if s["mav"]["custom_mode"] == want) / len(samples))
        if samples else None)
    A["armed_throughout"] = all(s["mav"]["armed"] for s in samples
                                if s["mav"]["armed"] is not None)
    A["surfaces_steady"] = surface_report(st, "steady")
    A["surfaces_whole"] = surface_report(samples, "whole")
    # Scenario D chains three separate GUIDED targets, so wp_dist/xtrack_error
    # reset at every leg boundary and the concatenated series is not a
    # continuous signal. It is still analysed and reported, and it IS gated
    # per leg inside route_extra().
    osc_exclude = ("xtrack_error_m",) if scen == "D" else ()
    A["oscillation_gating_exclusions_reason"] = (
        "xtrack_error_m demoted to report-only at the concatenated level: the "
        "target changes at each of the three waypoints, so the series resets. "
        "It is gated PER LEG in analysis.route.legs[*].xtrack_envelope_growth "
        "and by the check route_xtrack_not_growing_per_leg."
        if scen == "D" else "none")
    A["oscillation_steady"] = oscillation_report(st, gating_exclude=osc_exclude)
    A["tracking_steady"] = tracking_report(st)
    A["frames"] = frame_report(samples, p)
    A["envelope"] = envelope_report(samples, p, TH_MODE_TRANSIENT_S)
    nav_s = samples if nav_samples is None else nav_samples
    app_s = samples if approach_samples is None else approach_samples
    A["nav_sign"] = nav_sign_report(nav_s, scen)
    A["nav_sign_window_n"] = len(nav_s)
    A["approach"] = approach_report(app_s)
    if extra:
        A.update(extra)

    # ---- AUTOTUNE decision support (NON-GATING) ----
    tr = A["tracking_steady"]
    sat = A["surfaces_steady"].get("hard_limit_duty_max")
    ind = {}
    ind["roll_rms_deg"] = (tr.get("roll") or {}).get("rms")
    ind["pitch_rms_deg"] = (tr.get("pitch") or {}).get("rms")
    ind["saturation_duty_max"] = sat
    ind["oscillation_growing_channels"] = A["oscillation_steady"]["growing_channels"]
    ind["roll_rms_exceeds_threshold"] = bool(
        ind["roll_rms_deg"] is not None and ind["roll_rms_deg"] > TH_AUTOTUNE_ROLL_RMS_DEG)
    ind["pitch_rms_exceeds_threshold"] = bool(
        ind["pitch_rms_deg"] is not None and ind["pitch_rms_deg"] > TH_AUTOTUNE_PITCH_RMS_DEG)
    ind["saturation_exceeds_threshold"] = bool(
        sat is not None and sat > TH_AUTOTUNE_SAT_DUTY)
    ind["autotune_indicated_by_this_scenario"] = bool(
        ind["roll_rms_exceeds_threshold"] or ind["pitch_rms_exceeds_threshold"]
        or ind["saturation_exceeds_threshold"]
        or A["oscillation_steady"]["any_growing"])
    ind["policy"] = (
        "NON-GATING DECISION SUPPORT ONLY. A true value here does NOT authorise "
        "running AUTOTUNE and does NOT fail this stage. AUTOTUNE is run only if "
        "an independent evidence review concludes the current PIDs are genuinely "
        "inadequate during navigation. See autotune_procedure_stub() - dormant, "
        "never executed by this file.")
    A["autotune_decision_support"] = ind
    return A


def verdict_scenario(A, scen):
    """Gating checks. BLOCKER items are listed separately from ordinary fails."""
    if A.get("insufficient_samples"):
        return "INSUFFICIENT_DATA", ["insufficient_samples"], ["insufficient_samples"]
    surf = A["surfaces_steady"]
    env = A["envelope"]
    osc = A["oscillation_steady"]
    fr = A["frames"]
    ns = A["nav_sign"]

    checks = {}
    # --- BLOCKER: continuous hard-limit pinning ---
    run_s = surf.get("hard_limit_longest_run_s")
    duty = surf.get("hard_limit_duty_max")
    checks["BLOCKER_no_continuous_hard_limit_pinning"] = bool(
        run_s is not None and duty is not None
        and run_s <= TH_SURF_PIN_RUN_MAX_S and duty <= TH_SURF_PIN_DUTY_MAX)
    # --- BLOCKER: airspeed hard floor ---
    checks["BLOCKER_airspeed_above_hard_floor"] = bool(env["airspeed_above_hard_floor"])

    # --- normal gates ---
    for chan in ("aileron", "elevator", "rudder"):
        c = surf.get(chan)
        checks[f"surface_{chan}_p{int(TH_SURF_PERCENTILE)}_within_linear_range"] = bool(
            c and c["p95"] is not None and c["p95"] <= TH_SURF_LINEAR_DEG)
    checks["airspeed_never_below_min_in_steady_window"] = bool(env["airspeed_above_min_steady"])
    checks["airspeed_bounded"] = bool(env["airspeed_bounded"])
    checks["altitude_bounded"] = bool(env["altitude_bounded"])
    checks["no_growing_oscillation"] = not osc["any_growing"]
    checks["inner_loop_roll_sign_correct"] = bool(ns["inner_roll_sign_correct"])
    if scen != "B":
        # In a steady LOITER orbit the aircraft flies TANGENT to the circle, so
        # the target bearing (which points at the loiter CENTRE) is ~90 deg off
        # the heading by construction and never converges. Applying the
        # heading-convergence / bearing-sign test there would be a measurement
        # category error, not a physics finding. LOITER gets its own,
        # well-posed sign test instead: orbit_direction_correct.
        checks["no_wrong_sign_navigation_response"] = bool(ns["nav_sign_correct"])
        checks["heading_error_converged"] = bool(ns.get("heading_error_converged"))
    checks["heading_frame_consistent"] = bool(fr["heading_frame_consistent"])
    checks["roll_sign_gz_mav_agree"] = bool(fr["roll_sign_gz_mav_agree"])
    checks["xtrack_identity_consistent"] = bool(fr["xtrack_identity_consistent"])
    checks["armed_throughout"] = bool(A["armed_throughout"])
    # HEARTBEAT carries the mode; a stale heartbeat can survive a fraction of a
    # second past the mode switch, so the gate is "the target mode is the ONLY
    # mode seen once the entry latency has passed", not "seen in every sample".
    checks["target_mode_stable"] = bool(A.get("mode_stable_after_entry"))

    # --- scenario-specific ---
    if scen in ("A", "C", "D"):
        checks["navigated_toward_target"] = bool(A["approach"].get("closure_ok"))
    if scen == "B":
        lo = A.get("loiter", {})
        checks["orbit_direction_correct"] = bool(lo.get("direction_correct"))
        checks["orbit_radius_matches_command"] = bool(lo.get("radius_matches_command"))
        checks["orbit_radius_bounded"] = bool(lo.get("radius_bounded"))
        checks["orbit_not_spiralling"] = bool(lo.get("not_spiralling"))
    if scen == "C":
        rt = A.get("rtl", {})
        # RETIRED 2026-09-08: rtl_turned_toward_home. It duplicated
        # no_wrong_sign_navigation_response / heading_error_converged, which are
        # already evaluated over nav_window(), and its own unwindowed form was
        # invalid in both directions (see rtl_extra()'s window discipline).
        checks["rtl_reduced_home_distance"] = bool(rt.get("reduced_home_distance"))
        # ADDED 2026-09-08: a POSITIVE, well-posed RTL-specific acceptance check
        # measured on the post-arrival phase only.
        checks["rtl_loiters_at_home"] = bool(rt.get("loiters_at_home"))
    if scen == "D":
        wp = A.get("route", {})
        checks["all_legs_flown"] = bool(wp.get("all_legs_flown"))
        checks["altitude_change_leg_achieved"] = bool(wp.get("alt_change_achieved"))
        checks["route_xtrack_not_growing_per_leg"] = bool(wp.get("xtrack_not_growing_per_leg"))

    fails = sorted(k for k, v in checks.items() if not v)
    blockers = [k for k in fails if k.startswith("BLOCKER_")]
    if blockers:
        vd = "NAVIGATION_BLOCKER"
    elif fails:
        vd = "NAVIGATION_FAILED"
    else:
        vd = "NAVIGATION_PASS"
    A["checks"] = checks
    return vd, fails, blockers


# =============================================================================
# SCENARIO DEFINITIONS
# =============================================================================
# Every geometric quantity below is expressed RELATIVE TO THE AIRCRAFT'S OWN
# live-measured position and heading at the instant the navigation mode is
# entered. Nothing is hardcoded to a compass direction, so the campaign does
# not silently depend on the ENU<->NED mapping being what we think it is (that
# mapping is separately CHECKED - see frame_report()).
SCENARIOS = {
    "A": dict(
        mode=MODE_GUIDED, name="GUIDED_SINGLE_TARGET",
        purpose=("One simple GUIDED target waypoint. Verifies that heading "
                 "converges toward the target bearing with the correct sign, "
                 "that the aircraft closes on the target, and that altitude "
                 "and airspeed stay bounded."),
        # A 60 deg turn is large enough that the turn direction is unambiguous
        # and the roll demand is well clear of noise, and small enough to stay
        # far from ROLL_LIMIT_DEG (live-read 45 deg).
        turn_deg=60.0, distance_m=800.0, alt_change_m=0.0, duration_s=110.0),
    "B": dict(
        mode=MODE_LOITER, name="LOITER_ORBIT",
        purpose=("Orbit about the fixed centre ArduPlane latches at LOITER "
                 "entry (ModeLoiter::_enter -> do_loiter_at_location, "
                 "mode_loiter.cpp:4-6). Verifies bounded orbit radius, correct "
                 "orbit direction, bounded altitude/airspeed and no growing "
                 "spiral or oscillation."),
        # WP_LOITER_RAD is live-read (60 m). At AIRSPEED_CRUISE 18 m/s one
        # orbit takes 2*pi*60/18 = 20.9 s, so 150 s is ~7 orbits: enough for a
        # growth metric to have something to measure.
        duration_s=150.0),
    "C": dict(
        mode=MODE_RTL, name="RTL_RETURN_TO_HOME",
        purpose=("Fly away from home under GUIDED, then RTL. Verifies the "
                 "turn toward home, correct navigation sign, distance closure, "
                 "and stable altitude/airspeed with no divergence."),
        outbound_turn_deg=0.0, outbound_distance_m=1400.0,
        outbound_min_home_dist_m=500.0, outbound_max_s=75.0,
        duration_s=130.0),
    "D": dict(
        mode=MODE_GUIDED, name="GUIDED_SEQUENTIAL_WAYPOINTS",
        purpose=("A 3-waypoint mission-like route flown as SEQUENTIAL GUIDED "
                 "targets (no AUTO mission is uploaded, per the stage "
                 "instruction). Includes one small altitude change. Exercises "
                 "both turn directions."),
        # (turn relative to the INITIAL heading, leg length, altitude delta)
        legs=[dict(turn_deg=+45.0, distance_m=550.0, alt_change_m=0.0),
              dict(turn_deg=-55.0, distance_m=550.0, alt_change_m=+15.0),
              dict(turn_deg=+15.0, distance_m=550.0, alt_change_m=0.0)],
        leg_max_s=70.0, duration_s=200.0),
}


def nav_window(samples, p):
    """Samples in which the target-bearing navigation-sign test is well posed:
    farther than 3*loiter_radius from the target. Inside that distance
    update_loiter_update_nav() has handed over to the circle-capture law
    (navigation.cpp:343-359) and the aircraft is deliberately no longer
    pointing at the target."""
    rad = p.get("WP_LOITER_RAD") or 60.0
    lim = 3.0 * rad
    return [s for s in samples
            if s["mav"].get("wp_dist_m") is not None and s["mav"]["wp_dist_m"] > lim]


# =============================================================================
# BRING-UP (identical to every prior ArduPlane stage in this repository)
# =============================================================================
def bring_up(R):
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

    ctx = dict(node=node, sub=sub, osub=osub, adiag=adiag, pdiag=pdiag,
               aerodiag=aerodiag, pub_oneshot=pub_oneshot, pub_clear=pub_clear)

    mav, armed = base.phase1_mavlink_arm(R)
    ctx["mav"] = mav
    if not armed:
        return ctx, None, "phase1_mavlink_arm"

    p = dump_params(mav, R)
    rc3 = int(round(base.RC3_TRIM_TARGET_US))
    ctx["rc3"] = rc3
    preconditions(p, rc3, R)
    if not base.is_armed(mav):
        base.arm(mav)
        R["rearmed_after_param_dump"] = base.is_armed(mav)
        if not R["rearmed_after_param_dump"]:
            return ctx, p, "rearm_after_param_dump"

    R["home_position"] = request_home(mav)

    settled, elapsed = base.wait_ground_settle(osub)
    R["ground_settle"] = dict(settled=settled, elapsed_s=elapsed)
    if not settled:
        base.disarm(mav)
        return ctx, p, "ground_settle"

    _, ok_v = base.phase2_teleport_and_verify(
        node, sub, osub, pub_oneshot, (0.0, 0.0, AIRBORNE_ALT_M, 0.0, 0.0, 0.0), R)
    base.clear_wrench(pub_clear)
    if not ok_v:
        base.disarm(mav)
        return ctx, p, "phase2_teleport_verify"

    if not base.phase3_hold_to_trim(pub_oneshot, pub_clear, sub, osub, mav, R):
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        base.disarm(mav)
        return ctx, p, "phase3_hold_to_trim"
    # From here on: wrench cleared, free 6-DOF, autopilot in full control.
    R["free_flight_from"] = ("phase3_hold_to_trim release (clear_wrench). No "
                             "force, torque, pose or velocity is applied to the "
                             "model after this point.")

    if not campaign.enter_fbwa(mav, R):
        base.disarm(mav)
        return ctx, p, "fbwa_handoff"
    for mid in MSG_IDS_20HZ:
        campaign.request_rate(mav, mid, 20.0)
    # HEARTBEAT defaults to 1 Hz, which would make every mode transition
    # ambiguous for up to a second of samples. 4 Hz is enough to resolve it and
    # is far below any bandwidth concern on a loopback TCP link.
    campaign.request_rate(mav, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 4.0)
    time.sleep(0.3)
    return ctx, p, None


def shutdown(ctx, R):
    mav = ctx.get("mav")
    if mav is None:
        return
    try:
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        base.disarm(mav)
    finally:
        mav.close()


# =============================================================================
# SCENARIOS
# =============================================================================
def _seg(ctx, p, label, dur, t0f, latest, stop_fn=None):
    return run_nav_segment(ctx["mav"], ctx["sub"], ctx["osub"], ctx["adiag"],
                           ctx["pdiag"], ctx["aerodiag"], label, dur,
                           1500, 1500, ctx["rc3"], t0f, latest,
                           p.get("PTCH_TRIM_DEG", 2.49), stop_fn=stop_fn)


def _start_state(ctx, R, tag):
    """Live position/heading at the instant a navigation leg is commanded."""
    g = wait_global_position(ctx["mav"])
    R.setdefault("start_states", {})[tag] = g
    return g


def scenario_A(ctx, p, R):
    cfg = SCENARIOS["A"]
    t0f = time.time()
    latest = {}
    segs = {}
    segs["settle_fbwa"] = _seg(ctx, p, "settle_fbwa", SETTLE_FBWA_S, t0f, latest)
    if segs["settle_fbwa"]["aborted"]:
        return segs, None, "settle_fbwa_aborted"

    g = _start_state(ctx, R, "A")
    if g is None:
        return segs, None, "no_global_position"
    brg = (g["hdg_deg"] + cfg["turn_deg"]) % 360.0
    dn = cfg["distance_m"] * math.cos(math.radians(brg))
    de = cfg["distance_m"] * math.sin(math.radians(brg))
    lat, lon = offset_latlon(g["lat"], g["lon"], dn, de)
    alt = g["rel_alt_m"] + cfg["alt_change_m"]
    R["scenario_geometry"] = dict(
        start=g, commanded_turn_deg=cfg["turn_deg"], target_bearing_deg=brg,
        distance_m=cfg["distance_m"], target_lat=lat, target_lon=lon,
        target_rel_alt_m=alt,
        note="target placed relative to the live heading; positive turn_deg = "
             "target to the RIGHT, which must produce a RIGHT (positive) bank")

    ok, cm = set_mode_confirm(ctx["mav"], MODE_GUIDED)
    R["mode_entry"] = dict(target=MODE_GUIDED, confirmed=ok, seen=cm)
    if not ok:
        return segs, None, "guided_mode_not_confirmed"
    if not send_guided_target(ctx["mav"], lat, lon, alt, R, "A_target"):
        return segs, None, "guided_target_rejected"
    invalidate_nav_telemetry(latest)

    segs["guided"] = _seg(ctx, p, "guided", cfg["duration_s"], t0f, latest)
    if segs["guided"]["aborted"]:
        return segs, None, "guided_aborted"
    return segs, segs["guided"]["samples"], None


def scenario_B(ctx, p, R):
    cfg = SCENARIOS["B"]
    t0f = time.time()
    latest = {}
    segs = {}
    segs["settle_fbwa"] = _seg(ctx, p, "settle_fbwa", SETTLE_FBWA_S, t0f, latest)
    if segs["settle_fbwa"]["aborted"]:
        return segs, None, "settle_fbwa_aborted"

    g = _start_state(ctx, R, "B")
    ok, cm = set_mode_confirm(ctx["mav"], MODE_LOITER)
    R["mode_entry"] = dict(target=MODE_LOITER, confirmed=ok, seen=cm)
    R["scenario_geometry"] = dict(
        start=g, commanded_radius_m=p.get("WP_LOITER_RAD"),
        expected_direction="CLOCKWISE",
        direction_evidence=(
            "Plane::update_loiter() (navigation.cpp:362-372) sets "
            "loiter.direction = -1 only when aparm.loiter_radius < 0 or the WP "
            "carries loiter_ccw; WP_LOITER_RAD is live-read POSITIVE and LOITER "
            "mode sets no ccw flag, so direction = +1 = clockwise. A clockwise "
            "orbit means increasing compass heading and a sustained RIGHT "
            "(positive) bank."))
    if not ok:
        return segs, None, "loiter_mode_not_confirmed"
    invalidate_nav_telemetry(latest)

    segs["loiter"] = _seg(ctx, p, "loiter", cfg["duration_s"], t0f, latest)
    if segs["loiter"]["aborted"]:
        return segs, None, "loiter_aborted"
    return segs, segs["loiter"]["samples"], None


def scenario_C(ctx, p, R):
    cfg = SCENARIOS["C"]
    t0f = time.time()
    latest = {}
    segs = {}
    segs["settle_fbwa"] = _seg(ctx, p, "settle_fbwa", SETTLE_FBWA_S, t0f, latest)
    if segs["settle_fbwa"]["aborted"]:
        return segs, None, "settle_fbwa_aborted"

    home = R.get("home_position")
    g = _start_state(ctx, R, "C_outbound")
    if g is None or home is None:
        return segs, None, "no_home_or_position"
    # --- outbound leg (GUIDED) so that RTL has a real distance to close ---
    brg = (g["hdg_deg"] + cfg["outbound_turn_deg"]) % 360.0
    dn = cfg["outbound_distance_m"] * math.cos(math.radians(brg))
    de = cfg["outbound_distance_m"] * math.sin(math.radians(brg))
    lat, lon = offset_latlon(g["lat"], g["lon"], dn, de)
    ok, cm = set_mode_confirm(ctx["mav"], MODE_GUIDED)
    R["outbound_mode_entry"] = dict(confirmed=ok, seen=cm)
    if not ok or not send_guided_target(ctx["mav"], lat, lon, g["rel_alt_m"], R,
                                        "C_outbound"):
        return segs, None, "outbound_setup_failed"
    invalidate_nav_telemetry(latest)

    need = cfg["outbound_min_home_dist_m"]

    def far_enough(s, _):
        if s["t_seg"] < LEG_MIN_S:
            return False, None
        la, lo = s["mav"].get("lat"), s["mav"].get("lon")
        if la is None or lo is None:
            return False, None
        dnn, dee = latlon_to_ne(home["lat"], home["lon"], la, lo)
        d = math.hypot(dnn, dee)
        if d >= need:
            return True, f"home distance {d:.0f} m >= {need} m"
        return False, None

    segs["outbound_guided"] = _seg(ctx, p, "outbound_guided",
                                   cfg["outbound_max_s"], t0f, latest,
                                   stop_fn=far_enough)
    if segs["outbound_guided"]["aborted"]:
        return segs, None, "outbound_aborted"

    g2 = _start_state(ctx, R, "C_rtl")
    d0 = None
    brg_home = None
    if g2:
        dnn, dee = latlon_to_ne(home["lat"], home["lon"], g2["lat"], g2["lon"])
        d0 = math.hypot(dnn, dee)
        brg_home = bearing_deg(-dnn, -dee)   # from the aircraft back to home
    R["scenario_geometry"] = dict(
        home=home, rtl_entry=g2, home_distance_at_rtl_entry_m=d0,
        home_bearing_at_rtl_entry_deg=brg_home,
        rtl_altitude_param_m=p.get("RTL_ALTITUDE"),
        note="RTL climbs to RTL_ALTITUDE (relative to home) and returns; "
             "RTL_AUTOLAND is live-read and expected 0, so it loiters at home.")

    ok, cm = set_mode_confirm(ctx["mav"], MODE_RTL)
    R["mode_entry"] = dict(target=MODE_RTL, confirmed=ok, seen=cm)
    if not ok:
        return segs, None, "rtl_mode_not_confirmed"
    invalidate_nav_telemetry(latest)
    segs["rtl"] = _seg(ctx, p, "rtl", cfg["duration_s"], t0f, latest)
    if segs["rtl"]["aborted"]:
        return segs, None, "rtl_aborted"
    return segs, segs["rtl"]["samples"], None


def scenario_D(ctx, p, R):
    cfg = SCENARIOS["D"]
    t0f = time.time()
    latest = {}
    segs = {}
    segs["settle_fbwa"] = _seg(ctx, p, "settle_fbwa", SETTLE_FBWA_S, t0f, latest)
    if segs["settle_fbwa"]["aborted"]:
        return segs, None, "settle_fbwa_aborted"

    g0 = _start_state(ctx, R, "D")
    if g0 is None:
        return segs, None, "no_global_position"
    ok, cm = set_mode_confirm(ctx["mav"], MODE_GUIDED)
    R["mode_entry"] = dict(target=MODE_GUIDED, confirmed=ok, seen=cm)
    if not ok:
        return segs, None, "guided_mode_not_confirmed"

    wp_radius = p.get("WP_RADIUS") or 90.0
    h0 = g0["hdg_deg"]
    lat, lon = g0["lat"], g0["lon"]
    alt = g0["rel_alt_m"]
    route = []
    all_samples = []
    for i, leg in enumerate(cfg["legs"]):
        brg = (h0 + leg["turn_deg"]) % 360.0
        dn = leg["distance_m"] * math.cos(math.radians(brg))
        de = leg["distance_m"] * math.sin(math.radians(brg))
        lat, lon = offset_latlon(lat, lon, dn, de)
        alt = alt + leg["alt_change_m"]
        route.append(dict(index=i, bearing_deg=brg, distance_m=leg["distance_m"],
                          lat=lat, lon=lon, rel_alt_m=alt,
                          alt_change_m=leg["alt_change_m"],
                          turn_deg_from_initial_heading=leg["turn_deg"]))
        if not send_guided_target(ctx["mav"], lat, lon, alt, R, f"D_wp{i}"):
            R["route_setup_failure"] = i
            return segs, None, f"guided_target_{i}_rejected"

        invalidate_nav_telemetry(latest)

        def captured(s, _, rr=wp_radius):
            # Second, independent guard against a stale capture (see
            # invalidate_nav_telemetry): a leg is never allowed to end inside
            # LEG_MIN_S, which is far shorter than the ~31 s a 550 m leg needs.
            if s["t_seg"] < LEG_MIN_S:
                return False, None
            d = s["mav"].get("wp_dist_m")
            if d is not None and d <= rr:
                return True, f"wp_dist {d} m <= WP_RADIUS {rr} m"
            return False, None

        seg = _seg(ctx, p, f"leg{i}", cfg["leg_max_s"], t0f, latest,
                   stop_fn=captured)
        segs[f"leg{i}"] = seg
        all_samples.extend(seg["samples"])
        route[-1]["captured"] = seg["stopped_early"]
        route[-1]["stop_reason"] = seg["stop_reason"]
        route[-1]["n_samples"] = seg["n_samples"]
        if seg["aborted"]:
            R["route"] = route
            return segs, None, f"leg{i}_aborted"
    R["route_plan"] = route
    R["route_geometry_note"] = (
        "Waypoints are chained: each leg's bearing is measured from the INITIAL "
        "heading and its distance from the PREVIOUS waypoint. Leg 0 turns right "
        "(+45 deg), leg 1 turns left (-55 deg) so both turn directions are "
        "exercised, and leg 1 carries the small +15 m altitude change.")
    return segs, all_samples, None


SCENARIO_FN = {"A": scenario_A, "B": scenario_B, "C": scenario_C, "D": scenario_D}


# =============================================================================
# SCENARIO-SPECIFIC ANALYSIS
# =============================================================================
def loiter_extra(samples, p):
    """Orbit geometry measured TWO independent ways:
      (1) a least-squares circle fit of the Gazebo ground-truth XY track
          (world ENU: x = East, y = North) - completely independent of
          ArduPlane's own reporting;
      (2) ArduPlane's own NAV_CONTROLLER_OUTPUT.wp_dist (distance to the
          latched loiter centre).
    Agreement between the two is itself reported. Disagreement would mean the
    orbit metric is a measurement artefact, which is exactly the failure mode
    this project keeps finding."""
    out = {}
    st = steady_slice(samples, TH_MODE_TRANSIENT_S)
    xs = [s["gz"]["pos"][0] for s in st if s["gz"]["pos"]]
    ys = [s["gz"]["pos"][1] for s in st if s["gz"]["pos"]]
    ts = [s["t"] for s in st if s["gz"]["pos"]]
    cmd_r = p.get("WP_LOITER_RAD") or 60.0
    out["commanded_radius_m"] = cmd_r
    fit = circle_fit(xs, ys)
    if fit:
        cx, cy, r = fit
        radii = [math.hypot(x - cx, y - cy) for x, y in zip(xs, ys)]
        out["circle_fit"] = dict(centre_east_m=cx, centre_north_m=cy,
                                 radius_m=r, radius_std_m=stdev(radii),
                                 radius_min_m=min(radii), radius_max_m=max(radii),
                                 n=len(radii))
        out["radius_envelope_growth"] = envelope_growth(ts, radii, "loiter_radius")
        out["radius_growth_linear_detrend_reported_only"] = detrended_growth(ts, radii)
        slope = linreg(ts, radii)[0]
        dur = (ts[-1] - ts[0]) if len(ts) > 1 else 0.0
        out["radius_slope_m_per_s"] = slope
        out["radius_drift_over_window_m"] = (slope * dur) if slope is not None else None
        out["radius_matches_command"] = bool(abs(r - cmd_r) <= TH_LOITER_RADIUS_TOL_M)
        out["radius_bounded"] = bool(stdev(radii) <= TH_LOITER_RADIUS_STD_MAX_M)
        # A SPIRAL is a monotonic radius trend, which an oscillation-envelope
        # metric correctly reports as NOT_OSCILLATORY - so the spiral test must
        # be the radius TREND, not the envelope. No new threshold is invented:
        # the drift is required to stay inside the SAME radius tolerance the
        # orbit itself is held to, over the analysed window.
        out["not_spiralling"] = bool(
            out["radius_drift_over_window_m"] is not None
            and abs(out["radius_drift_over_window_m"]) <= TH_LOITER_RADIUS_TOL_M
            and not out["radius_envelope_growth"].get("growing"))
        out["not_spiralling_criterion"] = (
            "|radius slope * window| <= TH_LOITER_RADIUS_TOL_M (%.1f m) AND the "
            "radius oscillation envelope is not growing"
            % TH_LOITER_RADIUS_TOL_M)
    else:
        out["circle_fit"] = None
        out["radius_matches_command"] = False
        out["radius_bounded"] = False
        out["not_spiralling"] = False
    tw, wd = col(st, lambda s: s["mav"].get("wp_dist_m"))
    out["ardupilot_wp_dist_to_centre_m"] = minmaxmean(wd)
    if fit and wd:
        out["radius_gz_vs_ardupilot_delta_m"] = out["circle_fit"]["radius_m"] - mean(wd)

    # --- orbit DIRECTION (a well-posed sign test for LOITER) ---
    _, rolls = col(st, lambda s: s["derived"].get("roll_deg"))
    out["mean_roll_deg"] = mean(rolls) if rolls else None
    hs = [(s["t"], s["derived"].get("heading_compass_deg")) for s in st]
    hs = [(t, h) for t, h in hs if h is not None]
    unwrapped, total = [], 0.0
    for i in range(1, len(hs)):
        total += wrap180(hs[i][1] - hs[i - 1][1])
        unwrapped.append((hs[i][0], total))
    out["total_heading_change_deg"] = total
    out["n_orbits"] = (total / 360.0) if total else None
    out["heading_rate_deg_s"] = (linreg([t for t, _ in unwrapped],
                                        [v for _, v in unwrapped])[0]
                                 if len(unwrapped) > 8 else None)
    # Positive WP_LOITER_RAD, no ccw flag => clockwise => heading increasing and
    # a sustained RIGHT (positive) bank. Both must agree.
    out["expected_direction"] = "CLOCKWISE (heading increasing, positive roll)"
    out["direction_correct"] = bool(
        out["mean_roll_deg"] is not None and out["heading_rate_deg_s"] is not None
        and out["mean_roll_deg"] > 0 and out["heading_rate_deg_s"] > 0)
    return {"loiter": out}


def home_loiter_window(samples, p):
    """The POST-ARRIVAL phase of an RTL: the contiguous tail of the record from
    the first moment the aircraft is within 3 * WP_LOITER_RAD of home, with the
    first TH_MODE_TRANSIENT_S of that tail dropped so the circle can establish.

    This is the exact complement of nav_window(): inside 3 * loiter_radius,
    update_loiter_update_nav() (navigation.cpp:343-359) has handed over to the
    circle law and the aircraft is deliberately no longer pointing at the
    target. A contiguous TAIL is taken rather than a filtered subset so a
    transient dip in wp_dist on the way in cannot open the window early."""
    rad = p.get("WP_LOITER_RAD") or 60.0
    lim = 3.0 * rad
    idx = next((i for i, s in enumerate(samples)
                if s["mav"].get("wp_dist_m") is not None
                and s["mav"]["wp_dist_m"] <= lim), None)
    if idx is None:
        return []
    tail = samples[idx:]
    t0 = tail[0]["t"]
    return [s for s in tail if s["t"] - t0 >= TH_MODE_TRANSIENT_S]


def rtl_extra(samples, nav_samples, R, p):
    """RTL analysis, split into THREE windows because the three questions are
    not answerable over the same samples.

    WINDOW DISCIPLINE (rewritten 2026-09-08, closing validation MAJOR):
      * CLOSURE  -> the FULL sample set. "Did it come home?" needs the whole
        approach (measured 501 -> 53.9 m); windowing it would destroy the very
        quantity being measured.
      * BEARING / SIGN / CONVERGENCE -> nav_window() ONLY. Previously these ran
        over the full set, which INCLUDED the ~97 s post-arrival home loiter.
        In that phase ArduPlane is running update_loiter() and the aircraft
        flies TANGENT to the circle, so |bearing_to_home - heading| is ~88.8 deg
        BY CONSTRUCTION and never converges. The check was broken in BOTH
        directions: the failing half was meaningless (a tangent aircraft is not
        mis-navigating), and so was the passing half - sign agreement of 0.9929
        arose because a +88.8 deg bearing error times a +28.3 deg right bank
        always agree in a clockwise orbit. The verdict tracked window
        composition, not the aircraft. verdict_scenario() already exempted
        LOITER mode from exactly these checks; the post-arrival RTL phase is
        also a loiter and was simply never classified as one.
        Note the thin-margin trap this also removes: over the full set the
        pre-arrival home-bearing sign agreement was 0.8058 against a threshold
        of 0.80 - a 0.7 % margin - and all 40 disagreements fell in the final
        2.0 s at loiter capture. Windowing gives 1.000. BOTH halves are windowed
        here, not just the convergence half.
      * LOITER-AT-HOME -> home_loiter_window() ONLY, and it is now a POSITIVE
        acceptance check rather than a phase that corrupted another check.

    RETIRED: `turned_toward_home`. It is redundant - the generic
    no_wrong_sign_navigation_response and heading_error_converged checks already
    answer the same question correctly, because analyze_scenario() evaluates
    them over nav_window(). Retiring it removes a broken check; the new
    loiters_at_home check replaces it with a stronger one.
    """
    out = {}
    home = R.get("home_position")
    if home is None:
        return {"rtl": {"note": "no HOME_POSITION received",
                        "reduced_home_distance": False,
                        "loiters_at_home": False}}

    def home_ne(s):
        la, lo = s["mav"].get("lat"), s["mav"].get("lon")
        if la is None or lo is None:
            return None
        return latlon_to_ne(home["lat"], home["lon"], la, lo)

    # ---------------- 1. CLOSURE - FULL sample set (never windowed) --------
    ts, d = [], []
    for s in samples:
        ne = home_ne(s)
        if ne is None:
            continue
        ts.append(s["t"])
        d.append(math.hypot(*ne))
    if not d:
        return {"rtl": {"note": "no global position samples",
                        "reduced_home_distance": False,
                        "loiters_at_home": False}}
    out["closure_window"] = "FULL sample set - deliberately NOT windowed"
    out["n_closure_samples"] = len(d)
    out["home_distance_start_m"] = d[0]
    out["home_distance_min_m"] = min(d)
    out["home_distance_end_m"] = d[-1]
    out["home_distance_series"] = series_report(ts, d, "home_distance")
    out["closure_frac"] = (d[0] - min(d)) / d[0] if d[0] > 0 else None
    out["reduced_home_distance"] = bool(
        out["closure_frac"] is not None
        and out["closure_frac"] >= TH_APPROACH_CLOSURE_FRAC)

    # ---------------- 2. BEARING / SIGN / CONVERGENCE - nav_window ONLY ----
    nav_s = nav_samples if nav_samples is not None else []
    sign_votes, errs = [], []
    for s in nav_s:
        ne = home_ne(s)
        hd, nr = s["mav"].get("gpi_hdg_deg"), s["mav"].get("nav_roll_deg")
        if ne is None or hd is None or nr is None:
            continue
        dn, de = ne
        e = wrap180(bearing_deg(-dn, -de) - hd)   # aircraft -> home, vs heading
        errs.append((s["t"], e))
        if abs(e) >= TH_NAV_SIGN_DEADBAND_DEG:
            sign_votes.append(1.0 if e * nr > 0 else 0.0)
    out["bearing_window"] = ("nav_window() only - wp_dist > 3 * WP_LOITER_RAD, "
                             "i.e. the pre-arrival phase where a bearing to "
                             "home defines a desired heading at all")
    out["n_bearing_samples"] = len(errs)
    out["home_bearing_sign_agreement_frac"] = mean(sign_votes) if sign_votes else None
    out["n_home_sign_votes"] = len(sign_votes)
    if errs:
        n = max(4, len(errs) // 5)
        out["home_heading_error_first_fifth_deg"] = mean([abs(e) for _, e in errs[:n]])
        out["home_heading_error_last_fifth_deg"] = mean([abs(e) for _, e in errs[-n:]])
    out["turned_toward_home_RETIRED"] = (
        "This check has been RETIRED, not merely relaxed. It duplicated "
        "no_wrong_sign_navigation_response and heading_error_converged, which "
        "analyze_scenario() already evaluates over nav_window(), and its own "
        "unwindowed form was invalid in both directions. The numbers above are "
        "kept as a windowed cross-check of those two generic checks; they gate "
        "nothing.")

    # ---------------- 3. LOITER AT HOME - post-arrival window only ---------
    lw = home_loiter_window(samples, p)
    cmd_r = p.get("WP_LOITER_RAD") or 60.0
    rtl_alt = p.get("RTL_ALTITUDE")
    L = {"window": ("home_loiter_window() - contiguous tail from wp_dist <= "
                    "3 * WP_LOITER_RAD, first %.0f s dropped"
                    % TH_MODE_TRANSIENT_S),
         "n_samples": len(lw), "commanded_radius_m": cmd_r,
         "rtl_altitude_param_m": rtl_alt}
    east, north = [], []
    for s in lw:
        ne = home_ne(s)
        if ne is None:
            continue
        north.append(ne[0])
        east.append(ne[1])
    if len(east) < 20:
        L["status"] = "INSUFFICIENT_POST_ARRIVAL_SAMPLES"
        L["note"] = ("Fewer than 20 post-arrival samples: the aircraft did not "
                     "establish a home loiter within the window. Reported as a "
                     "FAILED check, never as a pass on absence of data.")
        out["loiter_at_home"] = L
        out["loiters_at_home"] = False
        return {"rtl": out}
    # Circle fit in the HOME-RELATIVE NE frame, so the fitted centre IS the
    # offset from home - no assumption that the Gazebo origin coincides with
    # ArduPlane's home.
    fit = circle_fit(east, north)
    if fit:
        ce, cn, r = fit
        radii = [math.hypot(e - ce, n - cn) for e, n in zip(east, north)]
        L["centre_east_of_home_m"] = ce
        L["centre_north_of_home_m"] = cn
        L["centre_offset_from_home_m"] = math.hypot(ce, cn)
        L["radius_m"] = r
        L["radius_std_m"] = stdev(radii)
        L["radius_min_m"] = min(radii)
        L["radius_max_m"] = max(radii)
        L["radius_error_m"] = r - cmd_r
        L["radius_matches_command"] = bool(abs(r - cmd_r) <= TH_LOITER_RADIUS_TOL_M)
        L["radius_bounded"] = bool(stdev(radii) <= TH_LOITER_RADIUS_STD_MAX_M)
        # "AT HOME" is held to the SAME tolerance as the orbit radius itself -
        # no new number is introduced for it.
        L["centred_on_home"] = bool(
            L["centre_offset_from_home_m"] <= TH_LOITER_RADIUS_TOL_M)
    else:
        L["radius_matches_command"] = False
        L["radius_bounded"] = False
        L["centred_on_home"] = False
    # altitude capture
    _, rel = col(lw, lambda s: s["mav"].get("relative_alt_m"))
    L["relative_altitude_m"] = minmaxmean(rel)
    if rel and rtl_alt is not None:
        L["altitude_error_m"] = mean(rel) - rtl_alt
        L["altitude_matches_rtl_altitude"] = bool(
            abs(mean(rel) - rtl_alt) <= TH_RTL_ALT_TOL_M)
    else:
        L["altitude_matches_rtl_altitude"] = False
    # orbit direction: same source-derived expectation as LOITER mode
    _, rolls = col(lw, lambda s: s["derived"].get("roll_deg"))
    L["mean_roll_deg"] = mean(rolls) if rolls else None
    hs = [(s["t"], s["derived"].get("heading_compass_deg")) for s in lw]
    hs = [(t, h) for t, h in hs if h is not None]
    unw, total = [], 0.0
    for i in range(1, len(hs)):
        total += wrap180(hs[i][1] - hs[i - 1][1])
        unw.append((hs[i][0], total))
    L["total_heading_change_deg"] = total
    L["n_orbits"] = (total / 360.0) if total else None
    L["heading_rate_deg_s"] = (linreg([t for t, _ in unw], [v for _, v in unw])[0]
                               if len(unw) > 8 else None)
    L["expected_direction"] = "CLOCKWISE (heading increasing, positive roll)"
    L["direction_evidence"] = (
        "Plane::do_RTL() (commands_logic.cpp:349-353) sets loiter.direction = -1 "
        "only when aparm.loiter_radius < 0; WP_LOITER_RAD is live-read POSITIVE, "
        "so direction = +1 = clockwise - the same derivation as LOITER mode.")
    L["direction_correct"] = bool(
        L["mean_roll_deg"] is not None and L["heading_rate_deg_s"] is not None
        and L["mean_roll_deg"] > 0 and L["heading_rate_deg_s"] > 0)
    # Tangency, recorded so this phase is never again mistaken for a navigation
    # error and the retired check never re-added.
    tang = []
    for s in lw:
        ne = home_ne(s)
        hd = s["mav"].get("gpi_hdg_deg")
        if ne is None or hd is None:
            continue
        tang.append(abs(wrap180(bearing_deg(-ne[0], -ne[1]) - hd)))
    L["bearing_to_home_minus_heading_abs_deg"] = minmaxmean(tang)
    L["tangency_note"] = (
        "~90 deg here is CORRECT, not an error: in a circular loiter the "
        "aircraft flies tangent to the circle, so the bearing to the centre is "
        "orthogonal to the heading by construction. This is why the bearing / "
        "sign / convergence metrics above are restricted to nav_window().")
    L["status"] = "MEASURED"
    out["loiter_at_home"] = L
    out["loiters_at_home"] = bool(
        L.get("radius_matches_command") and L.get("radius_bounded")
        and L.get("centred_on_home") and L.get("altitude_matches_rtl_altitude")
        and L.get("direction_correct"))
    out["loiters_at_home_criterion"] = (
        "orbit radius within %.0f m of WP_LOITER_RAD AND radius std <= %.0f m "
        "AND fitted centre within %.0f m of HOME AND |mean relative altitude - "
        "RTL_ALTITUDE| <= %.0f m AND clockwise orbit direction"
        % (TH_LOITER_RADIUS_TOL_M, TH_LOITER_RADIUS_STD_MAX_M,
           TH_LOITER_RADIUS_TOL_M, TH_RTL_ALT_TOL_M))
    return {"rtl": out}


def route_extra(segs, R, p):
    out = {"legs": []}
    plan = R.get("route_plan") or []
    all_captured = True
    for i, leg in enumerate(plan):
        seg = segs.get(f"leg{i}")
        if seg is None:
            all_captured = False
            continue
        ap = approach_report(seg["samples"])
        nsr = nav_sign_report(nav_window(seg["samples"], p), f"D_leg{i}")
        _, rolls = col(steady_slice(seg["samples"], 3.0),
                       lambda s: s["derived"].get("roll_deg"))
        _, alts = col(seg["samples"], lambda s: (s["gz"]["pos"][2] if s["gz"]["pos"] else None))
        # Per-leg oscillation: within ONE leg the target is constant, so
        # xtrack_error IS a continuous signal and the envelope metric is valid.
        lst = steady_slice(seg["samples"], 3.0)
        txt, xt = col(lst, lambda s: s["mav"].get("xtrack_error_m"))
        tro, ro = col(lst, lambda s: s["derived"].get("roll_err_deg"))
        xt_growth = envelope_growth(txt, xt, f"leg{i}_xtrack") if xt else None
        roll_growth = envelope_growth(tro, ro, f"leg{i}_roll_err") if ro else None
        entry = dict(index=i, planned=leg, captured=bool(seg["stopped_early"]),
                     xtrack_envelope_growth=xt_growth,
                     roll_envelope_growth=roll_growth,
                     stop_reason=seg["stop_reason"], n_samples=seg["n_samples"],
                     duration_s=seg["actual_duration_s"], approach=ap,
                     nav_sign=nsr,
                     commanded_turn_deg=leg["turn_deg_from_initial_heading"],
                     roll_mean_deg=(mean(rolls) if rolls else None),
                     altitude_start_m=(alts[0] if alts else None),
                     altitude_end_m=(alts[-1] if alts else None),
                     altitude_change_m=((alts[-1] - alts[0]) if alts else None))
        out["legs"].append(entry)
        all_captured = all_captured and entry["captured"]
    out["all_legs_flown"] = bool(plan and all_captured)
    grow = [e["index"] for e in out["legs"]
            if (e.get("xtrack_envelope_growth") or {}).get("growing")
            or (e.get("roll_envelope_growth") or {}).get("growing")]
    out["legs_with_growing_oscillation"] = grow
    out["xtrack_not_growing_per_leg"] = bool(out["legs"] and not grow)
    out["per_leg_oscillation_note"] = (
        "Envelope growth evaluated per leg, where the GUIDED target is constant "
        "and xtrack_error is a continuous signal. This is the gating form of "
        "the cross-track oscillation test for scenario D.")
    # the leg that carries the altitude change
    alt_leg = next((i for i, l in enumerate(plan) if abs(l["alt_change_m"]) > 1e-6), None)
    out["altitude_change_leg_index"] = alt_leg
    if alt_leg is not None and alt_leg < len(out["legs"]):
        e = out["legs"][alt_leg]
        want = plan[alt_leg]["alt_change_m"]
        got = e.get("altitude_change_m")
        out["altitude_change_commanded_m"] = want
        out["altitude_change_measured_m"] = got
        # 0.7 is the same "step achieved" fraction the validated TECS
        # climb/descent stage uses (TH_ALT_STEP_ACHIEVED_FRAC = 0.7).
        out["alt_change_achieved"] = bool(
            got is not None and want != 0 and (got / want) >= 0.7)
        out["altitude_change_criterion"] = (
            "measured/commanded >= 0.7, the same achieved-fraction criterion as "
            "test_ardupilot_tecs_climb_descent_energy.py")
    else:
        out["alt_change_achieved"] = False
    return {"route": out}


# =============================================================================
# AUTOTUNE - DORMANT. NOT EXECUTED BY THIS FILE OR ITS RUNNER.
# =============================================================================
AUTOTUNE_PROCEDURE = {
    "status": "DORMANT_NOT_EXECUTED",
    "authorisation": (
        "AUTOTUNE is NOT authorised by this stage and is NOT run by this file. "
        "It may only be run after an INDEPENDENT evidence review concludes, "
        "from the navigation campaign data, that the current PIDs are genuinely "
        "inadequate during navigation. The decision inputs are recorded per "
        "scenario in analysis.autotune_decision_support (roll/pitch tracking "
        "error statistics, saturation duty cycle, oscillation growth). Those "
        "numbers are DECISION SUPPORT ONLY and never gate this stage."),
    "level": {
        "AUTOTUNE_LEVEL": 8,
        "provenance": ("config/ardupilot/falcon_v2_sitl.parm, section "
                       "MANUFACTURER_RECOMMENDED_AUTOTUNE_LEVEL (Titan Dynamics "
                       "Falcon V2 Build & User Manual Rev 1.0). Already present "
                       "in the live parameter set; setting it is NOT part of "
                       "running autotune."),
        "note": "ArduPlane compiled default is 6 (ArduPlane/Parameters.cpp:29)."},
    "mode": {"AUTOTUNE": 8, "source": "ArduPlane/mode.h AUTOTUNE = 8"},
    "before_after_parameter_capture": [
        "RLL_RATE_P", "RLL_RATE_I", "RLL_RATE_D", "RLL_RATE_FF",
        "RLL_RATE_IMAX", "RLL_RATE_SMAX", "RLL2SRV_TCONST", "RLL2SRV_RMAX",
        "PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D", "PTCH_RATE_FF",
        "PTCH_RATE_IMAX", "PTCH_RATE_SMAX", "PTCH2SRV_TCONST",
        "PTCH2SRV_RMAX_UP", "PTCH2SRV_RMAX_DN", "AUTOTUNE_LEVEL",
    ],
    "procedure": [
        "0. PRECONDITION: an independent evidence review has concluded the "
        "   current PIDs are inadequate, and has said so in writing. Without "
        "   that, stop here.",
        "1. Capture the FULL live parameter set BEFORE (fetch_all_params) and "
        "   archive it verbatim as the 'before' record.",
        "2. Bring the aircraft to the same airborne initial condition this file "
        "   uses (bring_up()), stabilise in FBWA at AIRSPEED_CRUISE.",
        "3. ROLL AXIS, EVALUATED SEPARATELY: enter AUTOTUNE (custom_mode 8) and "
        "   command ROLL ONLY - repeated full-authority roll stick reversals "
        "   with pitch held neutral - for a fixed, recorded number of "
        "   reversals. Do not touch pitch during this block.",
        "4. Read back and archive the roll gains produced. Return to FBWA. Fly "
        "   the SAME scenario A/B/C/D harness in this file and compare the "
        "   roll tracking-error / saturation / oscillation metrics before vs "
        "   after. Roll is accepted or rejected on that comparison ALONE.",
        "5. PITCH AXIS, EVALUATED SEPARATELY: repeat steps 3-4 commanding PITCH "
        "   ONLY, with roll held neutral, and compare the pitch metrics.",
        "6. Capture the FULL live parameter set AFTER and diff it against the "
        "   'before' record. Every changed parameter must be listed explicitly.",
        "7. Nothing is adopted implicitly. Any gain change is a separate, "
        "   explicitly authorised source-of-truth change to "
        "   config/ardupilot/falcon_v2_sitl.parm with its own provenance entry, "
        "   reviewed by `validation` - exactly as TECS_PTCH_DAMP 0.6 was.",
    ],
    "caveats": [
        "ArduPlane's AUTOTUNE mode tunes whichever axis is being EXERCISED; it "
        "does not have separate roll-only/pitch-only sub-modes. Separating the "
        "axes is therefore done by the STICK INPUT and by evaluating the two "
        "axes' metrics independently, not by a mode setting.",
        "AUTOTUNE writes parameters to the vehicle. Any run must use a scratch "
        "EEPROM (arduplane -w) and must never write "
        "config/ardupilot/falcon_v2_sitl.parm.",
        "A gain produced by AUTOTUNE against the GAZEBO model is SIM-SPECIFIC "
        "and is not transferable to the real aircraft without separate flight "
        "validation - the same rule already recorded for TECS_PTCH_DAMP.",
    ],
}


def autotune_procedure_stub(*_a, **_kw):
    """DELIBERATELY UNIMPLEMENTED. Present so the procedure travels with the
    harness, dormant, and cannot be run by accident."""
    raise NotImplementedError(
        "AUTOTUNE is not authorised by this stage and is not implemented here. "
        "See AUTOTUNE_PROCEDURE for the full procedure and its preconditions.")


# =============================================================================
# I/O
# =============================================================================
# SMOKE MODE: a short mechanical shake-out of the harness itself (mode entry,
# DO_REPOSITION acceptance, sampling, analysis code paths). Its windows are far
# too short for any acceptance check to be meaningful, so it writes to SEPARATE
# `*_smoke_*` files that summarize() never reads, and every record it produces
# is stamped HARNESS_SMOKE_NOT_A_CAMPAIGN_RESULT.
SMOKE = False
SMOKE_OVERRIDES = {
    "settle_fbwa_s": 8.0,
    "A": dict(duration_s=40.0, distance_m=500.0),
    "B": dict(duration_s=45.0),
    "C": dict(outbound_max_s=30.0, outbound_min_home_dist_m=300.0, duration_s=45.0),
    # 45 s: a 550 m leg needs ~31 s of closure at AIRSPEED_CRUISE plus the turn,
    # so this is the shortest smoke that still exercises the leg-CAPTURE and
    # advance-to-next-target code path (the campaign uses 70 s).
    "D": dict(leg_max_s=45.0, duration_s=140.0),
}


def paths(scen):
    sfx = "_smoke" if SMOKE else ""
    return (os.path.join(RES, f"{PREFIX}_scenario_{scen}{sfx}_result.json"),
            os.path.join(RES, f"{PREFIX}_scenario_{scen}{sfx}_timeseries.json"))


def write_outputs(R, segs, scen):
    out_json, out_ts = paths(scen)
    os.makedirs(RES, exist_ok=True)
    ts_doc = {"stage": STAGE, "scenario": scen, "timestamp": R.get("timestamp"),
              "params_live": R.get("params_live"),
              "scenario_geometry": R.get("scenario_geometry"),
              "route_plan": R.get("route_plan"),
              "home_position": R.get("home_position"),
              "guided_targets": R.get("guided_targets"),
              "segments": segs}
    with open(out_ts, "w") as f:
        json.dump(ts_doc, f, default=str, separators=(",", ":"))
    slim = dict(R)
    slim["segments_summary"] = {
        k: {kk: vv for kk, vv in v.items() if kk != "samples"}
        for k, v in (segs or {}).items()}
    slim["timeseries_file"] = out_ts
    for key in ("phase2_teleport_verify", "phase3_hold_to_trim"):
        blk = slim.get(key)
        if isinstance(blk, dict):
            slim[key] = {k: v for k, v in blk.items() if k not in ("samples", "attempts")}
            slim[key]["samples_omitted"] = True
    with open(out_json, "w") as f:
        json.dump(slim, f, indent=2, default=str)
    return out_json, out_ts


def base_record(scen):
    cfg = SCENARIOS[scen]
    return {
        "stage": STAGE,
        "scenario": scen,
        "scenario_name": cfg["name"],
        "scenario_purpose": cfg["purpose"],
        "scenario_config": {k: v for k, v in cfg.items() if k != "purpose"},
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "mode": dict(number=cfg["mode"], name=MODE_NAME[cfg["mode"]],
                     evidence="ArduPlane/mode.h - see module docstring sec 1"),
        "writes_parameters": False,
        "wind": ("ZERO. The world declares no wind system and model/model.sdf's "
                 "FalconV2Wind plugin defaults to <steady_wind_mps>0 0 0. "
                 "SIM_WIND_SPD is additionally live-read and asserted zero."),
        "physics_bypass": ("NONE during any measured window. The airborne "
                           "initial condition is established by the project's "
                           "already-validated teleport + hold-to-trim bring-up, "
                           "which fully releases (clear_wrench) before the "
                           "autopilot takes over. See module docstring sec 5."),
        "thresholds": TH,
        "threshold_provenance": TH_PROV,
        "threshold_class_census": threshold_class_census(),
        "known_open_limitations": KNOWN_OPEN_LIMITATIONS,
        "nav_window_rule": (
            "The target-bearing navigation-sign and heading-convergence metrics "
            "are evaluated only where wp_dist > 3 * WP_LOITER_RAD, i.e. where "
            "ArduPlane is still running the waypoint-capture law "
            "(navigation.cpp:343-359). Inside that distance the aircraft is "
            "deliberately flying a circle and the target bearing no longer "
            "defines a desired heading. Scenario C applies the SAME rule to its "
            "RTL-specific metrics: closure over the full record, bearing/sign/"
            "convergence over nav_window() only, and loiter-at-home over the "
            "post-arrival window only - see rtl_extra()."),
        "autotune": AUTOTUNE_PROCEDURE,
    }


def run_scenario(scen):
    global SETTLE_FBWA_S
    if SMOKE:
        SETTLE_FBWA_S = SMOKE_OVERRIDES["settle_fbwa_s"]
        SCENARIOS[scen].update(SMOKE_OVERRIDES[scen])
    R = base_record(scen)
    if SMOKE:
        R["smoke"] = True
        R["smoke_warning"] = (
            "HARNESS_SMOKE_NOT_A_CAMPAIGN_RESULT. Windows are deliberately too "
            "short for any acceptance threshold to be meaningful. This record "
            "proves only that the harness mechanically works end to end: mode "
            "entry, DO_REPOSITION acceptance, telemetry capture, and the "
            "analysis code paths. Do NOT read its verdict as a result.")
    ctx, p, fail = bring_up(R)
    if fail:
        R["overall_result"] = "SCENARIO_ABORTED"
        R["verdict"] = "NAVIGATION_NOT_FLOWN"
        R["blocking_phase"] = fail
        write_outputs(R, {}, scen)
        shutdown(ctx, R)
        print("ABORTED during bring-up:", fail)
        return 1

    segs, samples, err = SCENARIO_FN[scen](ctx, p, R)
    shutdown(ctx, R)

    if err or not samples:
        R["overall_result"] = "SCENARIO_ABORTED"
        R["verdict"] = "NAVIGATION_NOT_FLOWN"
        R["blocking_phase"] = err or "no_samples"
        write_outputs(R, segs, scen)
        print("ABORTED:", err)
        return 1

    # nav_s is computed FIRST: rtl_extra() needs it. Its bearing/sign/
    # convergence metrics are only defined outside the home loiter, while its
    # closure metric needs the full set - see rtl_extra()'s window discipline.
    nav_s = None if scen == "B" else nav_window(samples, p)
    extra = {}
    if scen == "B":
        extra.update(loiter_extra(samples, p))
    elif scen == "C":
        extra.update(rtl_extra(samples, nav_s, R, p))
    elif scen == "D":
        extra.update(route_extra(segs, R, p))
    A = analyze_scenario(scen, samples, p, extra=extra, nav_samples=nav_s)
    vd, fails, blockers = verdict_scenario(A, scen)
    R["analysis"] = A
    R["verdict"] = vd
    R["failed_checks"] = fails
    R["blocking_checks"] = blockers
    R["overall_result"] = "FLIGHT_COMPLETED_NO_ABORT"
    out_json, out_ts = write_outputs(R, segs, scen)

    print("-" * 74)
    print(f"scenario {scen} ({SCENARIOS[scen]['name']})  n={A['n_samples']} "
          f"steady={A['n_steady_samples']}  modes={A['mode_names_seen']}")
    su = A["surfaces_steady"]
    for ch in ("aileron", "elevator", "rudder"):
        c = su.get(ch)
        if c:
            print(f"  {ch:9s} p{int(TH_SURF_PERCENTILE)}={c['p95']:.2f} deg  "
                  f"max={c['max_abs']:.2f} deg  frac>{TH_SURF_LINEAR_DEG:.0f}deg="
                  f"{c['frac_over_linear']:.3f}")
    print(f"  hard-limit duty={su.get('hard_limit_duty_max')} "
          f"longest run={su.get('hard_limit_longest_run_s')} s")
    env = A["envelope"]
    print(f"  airspeed min steady={env['airspeed_min_steady_ms']} "
          f"(AIRSPEED_MIN={env['airspeed_min_threshold_ms']}) "
          f"p2p={env['airspeed_p2p_steady_ms']}")
    print(f"  altitude p2p steady={env['altitude_p2p_steady_m']} m")
    print(f"  oscillation growing channels={A['oscillation_steady']['growing_channels']}")
    print(f"  nav sign agreement={A['nav_sign']['sign_agreement_frac']} "
          f"inner roll={A['nav_sign']['inner_roll_sign_agree_frac']}")
    print(f"  autotune decision support={A['autotune_decision_support']}")
    for k in sorted(A["checks"]):
        print(f"   {'PASS' if A['checks'][k] else 'FAIL'}  {k}")
    print(f"VERDICT: {vd}  failed={fails}  blockers={blockers}")
    print("RESULT:", out_json)
    print("TIMESERIES:", out_ts)
    return 0 if vd == "NAVIGATION_PASS" else 1


def summarize():
    out = {"stage": STAGE,
           "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "thresholds": TH, "threshold_provenance": TH_PROV,
           "threshold_class_census": threshold_class_census(),
           "known_open_limitations": KNOWN_OPEN_LIMITATIONS,
           "autotune": AUTOTUNE_PROCEDURE,
           "scenarios": {}}
    verdicts, all_fails, all_blockers = {}, {}, {}
    at_flags = {}
    for scen in ("A", "B", "C", "D"):
        pj, _ = paths(scen)
        if not os.path.exists(pj):
            out["scenarios"][scen] = {"present": False}
            verdicts[scen] = "NOT_RUN"
            continue
        with open(pj) as f:
            r = json.load(f)
        A = r.get("analysis", {})
        verdicts[scen] = r.get("verdict")
        all_fails[scen] = r.get("failed_checks", [])
        all_blockers[scen] = r.get("blocking_checks", [])
        at_flags[scen] = (A.get("autotune_decision_support") or {})
        out["scenarios"][scen] = {
            "present": True, "name": r.get("scenario_name"),
            "verdict": r.get("verdict"), "overall_result": r.get("overall_result"),
            "failed_checks": r.get("failed_checks"),
            "blocking_checks": r.get("blocking_checks"),
            "n_samples": A.get("n_samples"),
            "modes_seen": A.get("mode_names_seen"),
            "surfaces_steady": A.get("surfaces_steady"),
            "envelope": A.get("envelope"),
            "oscillation_growing": (A.get("oscillation_steady") or {}).get("growing_channels"),
            "tracking_steady": A.get("tracking_steady"),
            "nav_sign": A.get("nav_sign"),
            "approach": A.get("approach"),
            "frames": A.get("frames"),
            "loiter": A.get("loiter"), "rtl": A.get("rtl"), "route": A.get("route"),
            "autotune_decision_support": A.get("autotune_decision_support"),
            "preconditions": r.get("preconditions"),
            "result_file": pj,
        }
    out["verdicts"] = verdicts
    out["failed_checks_by_scenario"] = all_fails
    out["blocking_checks_by_scenario"] = all_blockers
    ran = [s for s in verdicts if verdicts[s] not in (None, "NOT_RUN")]
    out["scenarios_run"] = ran
    out["all_scenarios_run"] = (len(ran) == 4)
    out["any_blocker"] = any(all_blockers.get(s) for s in ran)
    out["campaign_verdict"] = (
        "NAVIGATION_BLOCKER" if out["any_blocker"] else
        "NAVIGATION_PASS" if (out["all_scenarios_run"]
                              and all(verdicts[s] == "NAVIGATION_PASS" for s in ran))
        else "NAVIGATION_FAILED" if ran else "NOT_RUN")
    out["autotune_decision"] = {
        "policy": AUTOTUNE_PROCEDURE["authorisation"],
        "indicated_by_scenario": {s: at_flags.get(s, {}).get(
            "autotune_indicated_by_this_scenario") for s in ran},
        "any_scenario_indicates_review": any(
            at_flags.get(s, {}).get("autotune_indicated_by_this_scenario") for s in ran),
        "action": ("If any_scenario_indicates_review is true, the ONLY action "
                   "authorised by this stage is to hand the evidence to an "
                   "independent review. AUTOTUNE is still NOT run by this "
                   "campaign under any circumstance."),
    }
    path = os.path.join(RES, f"{PREFIX}_validation_summary.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=2, default=str)
    print("-" * 74)
    for s in ("A", "B", "C", "D"):
        print(f"  {s} {SCENARIOS[s]['name']:32s} {verdicts.get(s)}  "
              f"fails={all_fails.get(s)}")
    print("CAMPAIGN VERDICT:", out["campaign_verdict"])
    print("AUTOTUNE review indicated:", out["autotune_decision"]["any_scenario_indicates_review"])
    print("SUMMARY:", path)
    return 0 if out["campaign_verdict"] == "NAVIGATION_PASS" else 1


def reanalyze(path):
    """Re-run the analysis offline against a captured timeseries. The flight is
    NOT re-flown. Used only after a TEST-LOGIC fix - never to make a physics
    result more convenient."""
    with open(path) as f:
        doc = json.load(f)
    scen = doc["scenario"]
    p = doc["params_live"]
    segs = doc["segments"]
    R = base_record(scen)
    R["timestamp"] = doc.get("timestamp")
    R["params_live"] = p
    R["scenario_geometry"] = doc.get("scenario_geometry")
    R["route_plan"] = doc.get("route_plan")
    R["home_position"] = doc.get("home_position")
    R["guided_targets"] = doc.get("guided_targets")
    R["reanalyzed_from"] = path
    if scen == "A":
        samples = segs["guided"]["samples"]
    elif scen == "B":
        samples = segs["loiter"]["samples"]
    elif scen == "C":
        samples = segs["rtl"]["samples"]
    else:
        samples = [s for i in range(len(SCENARIOS["D"]["legs"]))
                   if f"leg{i}" in segs for s in segs[f"leg{i}"]["samples"]]
    # nav_s is computed FIRST: rtl_extra() needs it. Its bearing/sign/
    # convergence metrics are only defined outside the home loiter, while its
    # closure metric needs the full set - see rtl_extra()'s window discipline.
    nav_s = None if scen == "B" else nav_window(samples, p)
    extra = {}
    if scen == "B":
        extra.update(loiter_extra(samples, p))
    elif scen == "C":
        extra.update(rtl_extra(samples, nav_s, R, p))
    elif scen == "D":
        extra.update(route_extra(segs, R, p))
    A = analyze_scenario(scen, samples, p, extra=extra, nav_samples=nav_s)
    vd, fails, blockers = verdict_scenario(A, scen)
    R["analysis"] = A
    R["verdict"] = vd
    R["failed_checks"] = fails
    R["blocking_checks"] = blockers
    R["overall_result"] = "REANALYZED"
    write_outputs(R, segs, scen)
    print("verdict:", vd, "fails:", fails, "blockers:", blockers)
    return 0


def main():
    global SMOKE
    args = [a for a in sys.argv[1:]]
    if "--smoke" in args:
        SMOKE = True
        args = [a for a in args if a != "--smoke"]
    if args and args[0] == "--summarize":
        return summarize()
    if len(args) > 1 and args[0] == "--reanalyze":
        return reanalyze(args[1])
    if len(args) > 1 and args[0] == "--scenario":
        scen = args[1].upper()
        if scen not in SCENARIOS:
            print("unknown scenario:", scen)
            return 2
        return run_scenario(scen)
    print(__doc__.strip().splitlines()[0])
    print("usage: --scenario A|B|C|D | --summarize | --reanalyze <timeseries.json>")
    return 2


if __name__ == "__main__":
    sys.exit(main())

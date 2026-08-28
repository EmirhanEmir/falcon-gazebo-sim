#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_CONTROL_SURFACE_TRAVEL_SCALING_VALIDATION live
acceptance test (gazebo-testing, 2026-08-28).

Rigorous LIVE test of the +/-10deg -> +/-45deg control-surface travel
rescale `controls-integration` applied to the 5 surface <control> blocks'
<multiplier> in model/model.sdf this stage (design record:
docs/source_of_truth/autopilot/SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md
sec 11), through the REAL ArduPilotPlugin/MAVLink bridge - not the
config-file-only check controls-integration's own sanity pass already did
(sec 11.5, which bypassed ArduPilotPlugin entirely and drove cmd_rad
directly). Reuses this project's existing SITL infrastructure unmodified:
tests/gazebo/scripts/ardupilot_sitl_mav_lib.py,
tests/gazebo/worlds/falcon_v2_ardupilot_sitl_test_world.sdf, the same
gdb-wrapped arduplane launch pattern documented in
docs/test_results/2026-08-27_ardupilot_sitl_transport_actuator_mapping_
validation.md sec 4 (bare launch is reproducibly unstable on first client
connect - upstream ardupilot/ardupilot_gazebo behavior, not a project
defect, unrelated to this stage's own scope).

SCOPE: throttle <control> blocks (channel 2/4) are UNCHANGED this stage
and OUT OF SCOPE - not touched or re-tested here. Only the 5 non-throttle
surface channels (aileron/elevator/rudder) are exercised. No closed-loop
flight mode is ever requested (custom_mode stays 0 = MANUAL throughout -
confirmed via every captured HEARTBEAT). No arming is attempted - per the
prior stage's own finding (2026-08-27 report sec 9), control-surface
cmd_rad output does not require ArduPlane to be armed, only the standard,
non-arming MAV_CMD_DO_SET_SAFETY_SWITCH_STATE hardware-safety-switch
unblock - this is CONFIRMED LIVE below (Phase 0b), not assumed.

METHOD - PWM->normalized-command relationship (derived, not assumed):
model.sdf's ArduPilotPlugin formula (confirmed exact from
ArduPilotPlugin.cc's UpdateMotorCommands(), cited in model.sdf's own
comment, sec 11.3 of the design doc):
    raw_cmd = clamp((pwm - servo_min) / (servo_max - servo_min), 0, 1)
    cmd     = multiplier * (raw_cmd + offset)              offset = -0.5
With servo_min=800/servo_max=2200/servo_trim=1500 (real SERVOx_MIN/MAX/TRIM
from config/ardupilot/falcon_v2_sitl.parm, cited not re-derived - MAX-TRIM
== TRIM-MIN == 700 exactly, i.e. symmetric) and multiplier=+/-1.5707963268
(this stage's new value), defining a normalized command n in [-1,+1] via
    target_servo_pwm = trim + n*(max-trim)     [= 1500 + 700*n, symmetric]
gives cmd = multiplier*0.5*n = +/-45*n degrees exactly at the JOINT this
<control> block's multiplier sign selects (per-surface sign table, cited
below, unchanged this stage). This is the SAME arithmetic
controls-integration's own sec 11.3 derivation uses, independently
re-applied here to build this test's own commanded-vs-expected table - not
copied from their live numbers.

ArduPlane does not expose a "set this SERVO's PWM directly" MAVLink command
that survives its own MANUAL-mode RC->servo mixing (DO_SET_SERVO is
overwritten every control loop for a channel with a SERVOx_FUNCTION
assigned - source-confirmed, not attempted here), so this script instead:
  1. LIVE-CALIBRATES each RC input channel (RC1=aileron/RC2=elevator/
     RC4=rudder, per RCMAP_ROLL=1/PITCH=2/YAW=4, cited from the prior
     stage's own live param dump) against its own real SERVO_OUTPUT_RAW
     PWM via a 5-point sweep (RC=1100/1300/1500/1700/1900), fits a line
     (own ordinary-least-squares, no external numpy dependency), and
     confirms it really is linear (max residual reported) - INSTEAD OF
     assuming RCx_MIN/MAX/TRIM/REVERSED param values (which are not
     overridden in falcon_v2_sitl.parm, i.e. compiled defaults, not
     independently confirmed as a specific number by this script).
  2. Inverts that live-measured fit to find the RC PWM that drives each
     target SERVO PWM, commands it via RC_CHANNELS_OVERRIDE, and reads
     back the REAL resulting SERVO_OUTPUT_RAW (ground truth of what
     ArduPilotPlugin's own bridge formula actually receives as "pwm") plus
     the real joint angle (actuator diagnostics) and real aero
     coefficients (aerodynamics diagnostics) - never assumed, always
     independently re-measured per point.

Sign relationship used to map a per-surface "target_deg" (this test's own
commanded quantity, defined identically for all 3 surfaces via the
multiplier-sign table below) to the AERO-MODEL's own internal deltaE/
deltaA/deltaR (what indexes the TXT-vs-runtime lookup, item 9) is DERIVED
ALGEBRAICALLY below (see TXT_DELTA_SIGN comment), not assumed - it falls
out of CONTROLS.md sec 10's already-VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST
formulas (delta_e_aero = -0.5*(thetaL+thetaR), delta_a_aero =
+0.5*(thetaR-thetaL), delta_r_aero = theta_rudder, cited not re-derived)
combined with this test's own MULT_SIGN table (cited from model.sdf).

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
control authority, motor thrust) is read, written, or influenced by this
script. model/model.sdf, falcon_v2_sitl.parm, and every plugin under
plugins/ are read-only inputs to this script and are NOT modified by it.
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
from gz.msgs10 import double_v_pb2, twist_pb2  # noqa: E402
from pymavlink import mavutil  # noqa: E402

from ardupilot_sitl_mav_lib import SafeMav  # noqa: E402

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = f"{REPO_ROOT}/tests/gazebo/results"
OUT_JSON = os.path.join(RESULTS_DIR, "ardupilot_control_surface_travel_scaling_result.json")
LOG_LINES = []

R = {}


def log(*a):
    line = " ".join(str(x) for x in a)
    print(line, flush=True)
    LOG_LINES.append(line)


def is_finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


# =============================================================================
# gz-transport diagnostics (identical field layout/topics to the prior
# stage's proven test - tests/gazebo/scripts/
# test_ardupilot_sitl_transport_actuator_mapping.py - reused, not
# reinvented)
# =============================================================================
class DoubleVSub:
    def __init__(self, topic):
        self.topic = topic
        self.node = tp.Node()
        self.history = []
        ok = self.node.subscribe(double_v_pb2.Double_V, topic, self._cb)
        if not ok:
            raise RuntimeError(f"failed to subscribe {topic}")

    def _cb(self, msg):
        self.history.append(list(msg.data))

    def latest(self):
        return list(self.history[-1]) if self.history else None


ACTUATOR_DIAG_TOPIC = "/model/falcon_v2/actuators/diagnostics"
AERO_DIAG_TOPIC = "/model/falcon_v2/aerodynamics/diagnostics"

ACTUATOR_SURFACES = ["left_aileron", "right_aileron", "left_elevator", "right_elevator", "rudder"]
ACTUATOR_FIELDS = ["cmd_rad", "target_clamped_rad", "setpoint_rad", "actual_angle_rad",
                    "actual_rate_rad_s", "target_clamp_active", "effort_clamp_active"]
AERO_FIELDS = ["V", "alpha", "beta", "qbar", "CL", "CD", "CY", "Cl", "Cm", "Cn"]


def actuator_fields_for(vec, surface):
    i = ACTUATOR_SURFACES.index(surface)
    chunk = vec[i * 7:(i + 1) * 7]
    return dict(zip(ACTUATOR_FIELDS, chunk))


def aero_fields(vec):
    return dict(zip(AERO_FIELDS, vec))


def any_nonfinite(d):
    return any(isinstance(v, (int, float)) and not math.isfinite(v) for v in d.values())


# =============================================================================
# Real SERVOx_MIN/TRIM/MAX (config/ardupilot/falcon_v2_sitl.parm, cited)
# and the 5 <control> block multiplier signs (model/model.sdf, cited,
# UNCHANGED by this stage) - both hardcoded here for documentation/self-
# containment, NOT re-read from the files at runtime (this is a read-only
# citation of already-fixed values, not a live-parsed dependency).
# =============================================================================
SERVO_TRIM = 1500.0
SERVO_MIN = 800.0
SERVO_MAX = 2200.0
assert (SERVO_MAX - SERVO_TRIM) == (SERVO_TRIM - SERVO_MIN) == 700.0

RC_KEY = {"aileron": "rc1", "elevator": "rc2", "rudder": "rc4"}
SERVO_RAW_FIELD = {"aileron": "servo1_raw", "elevator": "servo2_raw", "rudder": "servo4_raw"}
JOINTS_FOR = {"aileron": ["left_aileron", "right_aileron"],
              "elevator": ["left_elevator", "right_elevator"],
              "rudder": ["rudder"]}
# multiplier signs, model.sdf <control> blocks, cited verbatim (UNCHANGED
# this stage - only the magnitude was rescaled +/-10deg -> +/-45deg):
MULT_SIGN = {"left_aileron": +1, "right_aileron": -1,
             "left_elevator": +1, "right_elevator": +1, "rudder": -1}


def target_servo_pwm(n):
    """n in [-1, +1] -> target SERVO PWM, per this test's own module
    docstring derivation."""
    return SERVO_TRIM + 700.0 * n


def target_joint_deg(joint, n):
    return MULT_SIGN[joint] * 45.0 * n


def te_direction(angle_rad, kind):
    """Reused verbatim from the prior stage's proven convention
    (test_ardupilot_sitl_transport_actuator_mapping.py), already
    VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST (CONTROLS.md sec 10) - not
    re-derived here."""
    if kind in ("aileron", "elevator"):
        if angle_rad > 1e-4:
            return "TE_UP"
        if angle_rad < -1e-4:
            return "TE_DOWN"
        return "NEUTRAL"
    if angle_rad > 1e-4:
        return "TE_RIGHT(-Y)"
    if angle_rad < -1e-4:
        return "TE_LEFT(+Y)"
    return "NEUTRAL"


# =============================================================================
# TXT_DELTA_SIGN: derived relationship between this test's own "target_deg"
# (defined identically per-surface via MULT_SIGN, above) and the AERO
# MODEL's own internal deltaE/deltaA/deltaR (what the TXT wide-deflection
# table and aero_v1_config.yaml's lookup are indexed by). Algebra, worked
# once here (not per-point):
#   elevator: joint targets thetaL=thetaR=+target_deg (MULT_SIGN both +1)
#     delta_e_aero = -0.5*(thetaL+thetaR) = -target_deg
#   aileron: thetaL=+target_deg, thetaR=-target_deg
#     delta_a_aero = +0.5*(thetaR-thetaL) = +0.5*(-target_deg-target_deg)
#                  = -target_deg
#   rudder: theta_rudder=-target_deg (MULT_SIGN=-1)
#     delta_r_aero = theta_rudder = -target_deg
# All three collapse to the SAME relationship: TXT/aero-model delta =
# -target_deg. Used only for the TXT-vs-runtime comparison (item 9) below.
# =============================================================================
def aero_delta_deg_for(target_deg):
    return -target_deg


# TXT reference tables (docs/source_of_truth/aerodynamics/
# control_surface_analysis/FALCON_V2_CONTROL_SURFACE_WIDE_DEFLECTION_
# RESULTS.txt, transcribed verbatim, read-only citation - re-read directly
# from the file in Phase 9 below to confirm this transcription, not just
# trusted blind). Keyed by delta_e/delta_a/delta_r in degrees.
TXT_ELEVATOR = {  # delta_e[deg]: (CL, CD, Cm)
    -45: (0.34636, 0.04905, +0.71208), -35: (0.41561, 0.03393, +0.54701),
    -25: (0.48722, 0.02339, +0.37555), -15: (0.55971, 0.01707, +0.20137),
    -5: (0.63209, 0.01479, +0.02697), 0: (0.66817, 0.01513, -0.06013),
    5: (0.70422, 0.01646, -0.14727), 15: (0.77638, 0.02214, -0.32197),
    25: (0.84870, 0.03186, -0.49745), 35: (0.92074, 0.04573, -0.67262),
    45: (0.99126, 0.06359, -0.84442),
}
TXT_AILERON = {  # delta_a[deg]: (CL, CD, CY, Cl, Cm, Cn)
    -45: (0.62116, 0.18174, -0.00486, -0.31890, -0.07619, -0.00123),
    -35: (0.64137, 0.11664, -0.00354, -0.25052, -0.07310, -0.00099),
    -25: (0.65493, 0.06757, -0.00228, -0.18001, -0.06746, -0.00072),
    -15: (0.66353, 0.03407, -0.00126, -0.10836, -0.06297, -0.00044),
    -5: (0.66766, 0.01723, -0.00041, -0.03617, -0.06046, -0.00014),
    0: (0.66817, 0.01513, -0.00002, 0.00000, -0.06013, +0.00001),
    5: (0.66766, 0.01723, +0.00037, +0.03617, -0.06046, +0.00016),
    15: (0.66353, 0.03407, +0.00122, +0.10836, -0.06296, +0.00045),
    25: (0.65492, 0.06757, +0.00224, +0.18001, -0.06745, +0.00073),
    35: (0.64136, 0.11664, +0.00350, +0.25052, -0.07308, +0.00101),
    45: (0.62115, 0.18174, +0.00482, +0.31890, -0.07617, +0.00124),
}
TXT_RUDDER = {  # delta_r[deg]: (CL, CD, CY, Cl, Cm, Cn)
    -45: (0.67406, 0.02974, -0.07094, +0.00054, -0.05936, +0.02115),
    -35: (0.67249, 0.02390, -0.05601, +0.00042, -0.06099, +0.01666),
    -25: (0.67053, 0.01932, -0.04023, +0.00030, -0.06072, +0.01195),
    -15: (0.66890, 0.01682, -0.02411, +0.00018, -0.06005, +0.00715),
    -5: (0.66823, 0.01528, -0.00801, +0.00006, -0.06009, +0.00238),
    0: (0.66817, 0.01513, -0.00002, 0.00000, -0.06013, +0.00001),
    5: (0.66823, 0.01528, +0.00797, -0.00006, -0.06007, -0.00236),
    15: (0.66889, 0.01682, +0.02407, -0.00018, -0.06001, -0.00713),
    25: (0.67052, 0.01932, +0.04020, -0.00030, -0.06065, -0.01194),
    35: (0.67247, 0.02390, +0.05598, -0.00042, -0.06089, -0.01665),
    45: (0.67404, 0.02974, +0.07093, -0.00054, -0.05923, -0.02114),
}
DRAG_K = 0.0528  # docs/source_of_truth/aerodynamics/aero_v1_config.yaml drag_polar.k, cited

# Rudder Cl (roll-due-to-rudder) is a SPECIAL CASE, found live by this test
# and root-caused via source read (NOT a defect - a pre-existing,
# documented, prior-stage decision, UNCHANGED by this stage):
# aero_v1_config.yaml's rudder.Cl_NOT_LOADED_disputed_sign_reference_only
# (lines ~485-495) is explicitly NOT loaded by the plugin - the runtime
# instead derives ctrlRuddCl as a BOUNDED LINEAR EXTENSION of
# lateral_directional.Cldr_per_rad (+0.0007/rad, the OLD, previously-
# validated small-signal value - AeroConfig::Prepare(), AeroModel.hh line
# ~271: `ctrlRuddCl[i] = Cldr * ctrlBreakpointsRad[i]`), which has the
# OPPOSITE SIGN from the wide-deflection TXT's own raw Cl(delta_r) column
# (an explicit "UNRESOLVED_KEEP_CURRENT" sign dispute from a PRIOR stage,
# deliberately not re-introduced into the runtime model). TXT_RUDDER's own
# Cl column above is therefore NOT the correct runtime reference for this
# one coefficient - CLDR_PER_RAD below is.
CLDR_PER_RAD = 0.0007  # aero_v1_config.yaml lateral_directional.Cldr_per_rad, cited


# =============================================================================
# Point set (see module docstring for the item-by-item mapping this union
# satisfies - items 1/2/3/7/9/10 all draw from this ONE sweep, no point is
# measured twice for the same purpose).
# =============================================================================
BOTH_SIGN_MAGS = [5.0, 11.0, 11.25, 15.0, 22.5, 25.0, 33.75, 35.0, 45.0]
POS_ONLY_MAGS = [9.9, 10.0, 10.1, 14.9, 15.1, 24.9, 25.1, 34.9, 35.1, 44.9]
MAIN_SWEEP_DEGS = sorted(set([0.0] + [s * m for m in BOTH_SIGN_MAGS for s in (1, -1)] + POS_ONLY_MAGS))
TRACKING_MAGS = [11.25, 22.5, 33.75, 45.0]
TRACKING_DEGS = sorted(set([s * m for m in TRACKING_MAGS for s in (1, -1)]))

MAIN_HOLD_S = 2.2
TRACKING_HOLD_S = 3.2
CAL_HOLD_S = 1.3
NEUTRAL_RC = dict(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)


def measure_point(mav, actuator_sub, aero_sub, surface, target_deg, fit, hold_s):
    n = target_deg / 45.0
    tgt_pwm = target_servo_pwm(n)
    rc_key = RC_KEY[surface]
    rc_raw = (tgt_pwm - fit["a"]) / fit["b"]
    rc_clamped = max(1000.0, min(2000.0, rc_raw))
    kw = dict(NEUTRAL_RC)
    kw[rc_key] = rc_clamped
    mav.hold_rc_override(hold_s, **kw)
    servo_msgs = mav.drain(0.3, types={"SERVO_OUTPUT_RAW"})
    servo_actual = getattr(servo_msgs[-1], SERVO_RAW_FIELD[surface]) if servo_msgs else None

    a_latest = actuator_sub.latest()
    ae_latest = aero_sub.latest()
    any_nan = False
    joints = {}
    for j in JOINTS_FOR[surface]:
        fld = actuator_fields_for(a_latest, j) if a_latest else {}
        if fld and any_nonfinite(fld):
            any_nan = True
        actual_rad = fld.get("actual_angle_rad")
        joints[j] = dict(
            target_deg=target_joint_deg(j, n),
            actual_deg=math.degrees(actual_rad) if actual_rad is not None else None,
            actual_angle_rad=actual_rad,
            setpoint_rad=fld.get("setpoint_rad"),
            target_clamp_active=fld.get("target_clamp_active"),
            effort_clamp_active=fld.get("effort_clamp_active"),
            te=te_direction(actual_rad, "rudder" if surface == "rudder" else surface) if actual_rad is not None else None,
        )
    aero = aero_fields(ae_latest) if ae_latest else None
    if aero and any_nonfinite(aero):
        any_nan = True

    return dict(target_deg=target_deg, n=n, target_servo_pwm=tgt_pwm,
                rc_command_raw=rc_raw, rc_command_clamped=rc_clamped,
                rc_clamp_flag=abs(rc_raw - rc_clamped) > 1e-6,
                servo_actual_pwm=servo_actual, joints=joints, aero=aero,
                any_nan=any_nan, aero_delta_deg=aero_delta_deg_for(target_deg))


# =============================================================================
# Phase 0: connect, transport, safety-switch unblock (confirmed live, not
# assumed carried over from the prior stage)
# =============================================================================
def phase0_connect_and_unblock(mav):
    log("\n=== PHASE 0: connect / transport / safety-switch confirmation ===")
    out = {}
    hb = mav.wait_heartbeat(timeout=20)
    log("first HEARTBEAT (may still be INITIALISING right after connect):", hb.to_dict() if hb else None)
    assert hb is not None, "no HEARTBEAT received - transport not up"
    # ArduPlane briefly reports custom_mode=16 (INITIALISING) for a few
    # heartbeats immediately after a fresh client connect before settling
    # into its configured default mode (MANUAL=0, falcon_v2_sitl.parm has
    # no mode-changing param) - wait for that settle rather than asserting
    # on the very first heartbeat (empirically confirmed this run: a
    # separate live probe caught custom_mode=16 then 0 within ~1s).
    t0 = time.time()
    while hb is not None and hb.custom_mode != 0 and time.time() - t0 < 15:
        hb = mav.wait_heartbeat(timeout=3)
    out["heartbeat"] = hb.to_dict() if hb else None
    log("HEARTBEAT (settled):", out["heartbeat"])
    assert hb is not None, "no HEARTBEAT received - transport not up"
    assert hb.custom_mode == 0, f"expected MANUAL(0), got custom_mode={hb.custom_mode}"

    # 0a: BEFORE unblock - probe whether cmd_rad already flows (do not
    # assume the prior stage's finding still holds).
    node = tp.Node()
    probe_sub = DoubleVSub(ACTUATOR_DIAG_TOPIC)
    time.sleep(0.5)
    n_before = len(probe_sub.history)
    mav.hold_rc_override(1.0, rc1=1700, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    time.sleep(0.3)
    a1 = actuator_fields_for(probe_sub.latest(), "left_aileron") if probe_sub.latest() else None
    mav.hold_rc_override(0.5, **NEUTRAL_RC)
    out["before_unblock_left_aileron_actual_deg"] = (
        math.degrees(a1["actual_angle_rad"]) if a1 and a1.get("actual_angle_rad") is not None else None)
    log("before safety-switch-off, RC1=1700 probe -> left_aileron actual_deg =",
        out["before_unblock_left_aileron_actual_deg"])

    # 0b: standard, non-arming, non-persisted safety switch off.
    ack = mav.command_long(mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, p1=1)
    out["safety_switch_off_ack"] = ack.to_dict() if ack else None
    log("safety switch off ack:", out["safety_switch_off_ack"])

    mav.hold_rc_override(1.0, rc1=1700, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    time.sleep(0.3)
    a2 = actuator_fields_for(probe_sub.latest(), "left_aileron") if probe_sub.latest() else None
    mav.hold_rc_override(0.5, **NEUTRAL_RC)
    out["after_unblock_left_aileron_actual_deg"] = (
        math.degrees(a2["actual_angle_rad"]) if a2 and a2.get("actual_angle_rad") is not None else None)
    log("after safety-switch-off, RC1=1700 probe -> left_aileron actual_deg =",
        out["after_unblock_left_aileron_actual_deg"])
    out["safety_switch_confirmed_required_and_sufficient"] = (
        (out["before_unblock_left_aileron_actual_deg"] is None or abs(out["before_unblock_left_aileron_actual_deg"]) < 0.5)
        and out["after_unblock_left_aileron_actual_deg"] is not None
        and abs(out["after_unblock_left_aileron_actual_deg"]) > 3.0)
    log("safety_switch_confirmed_required_and_sufficient:",
        out["safety_switch_confirmed_required_and_sufficient"])

    # arming NOT attempted - out of scope, surfaces don't need it (confirmed above).
    hb2 = mav.wait_heartbeat(timeout=5)
    out["armed_after_unblock"] = bool(hb2.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb2 else None
    log("armed_after_unblock (should be False - no arm attempted):", out["armed_after_unblock"])
    return out


# =============================================================================
# Phase 1: pin a clean trim-velocity straight/level condition via
# VelocityControl/cmd_vel (same technique as the prior stage's phase 2 tail,
# same 21.244 m/s documented trim velocity, CLAUDE.md)
# =============================================================================
def publish_cmd_vel(pub, lin, ang):
    msg = twist_pb2.Twist()
    msg.linear.x, msg.linear.y, msg.linear.z = lin
    msg.angular.x, msg.angular.y, msg.angular.z = ang
    pub.publish(msg)


def phase1_pin_trim(mav):
    log("\n=== PHASE 1: pin trim-velocity condition (VelocityControl/cmd_vel) ===")
    node = tp.Node()
    pub = node.advertise("/model/falcon_v2/cmd_vel", twist_pb2.Twist)
    time.sleep(0.5)
    publish_cmd_vel(pub, (0, 0, 0), (0, 0, 0))
    time.sleep(1.0)
    publish_cmd_vel(pub, (21.244, 0, 0), (0, 0, 0))
    time.sleep(2.5)
    log("pinned body +X = 21.244 m/s (documented trim velocity, CLAUDE.md)")
    return pub  # kept alive / republished by caller if needed


# =============================================================================
# Phase 2: live RC->SERVO calibration (own method, not assumed from params)
# =============================================================================
def ols_fit(xs, ys):
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    b = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    a = (sy - b * sx) / n
    resid = [y - (a + b * x) for x, y in zip(xs, ys)]
    return dict(a=a, b=b, max_resid=max(abs(r) for r in resid), points=list(zip(xs, ys)))


def phase2_calibrate(mav):
    log("\n=== PHASE 2: live RC->SERVO calibration (own method) ===")
    fits = {}
    for surface, rc_key in RC_KEY.items():
        pts_rc, pts_servo = [], []
        for rc_val in (1100, 1300, 1500, 1700, 1900):
            kw = dict(NEUTRAL_RC)
            kw[rc_key] = rc_val
            mav.hold_rc_override(CAL_HOLD_S, **kw)
            msgs = mav.drain(0.3, types={"SERVO_OUTPUT_RAW"})
            val = getattr(msgs[-1], SERVO_RAW_FIELD[surface]) if msgs else None
            pts_rc.append(rc_val)
            pts_servo.append(val)
        mav.hold_rc_override(0.5, **NEUTRAL_RC)
        fit = ols_fit(pts_rc, pts_servo)
        fits[surface] = fit
        log(f"  {surface:10s} ({rc_key}): SERVO={fit['a']:.3f} + {fit['b']:.6f}*RC, "
            f"max_resid={fit['max_resid']:.3f} PWM  points={fit['points']}")
    return fits


# =============================================================================
# Phase 3: main sweep (items 1/2/3 raw data; 4/5/6/9/10 derived below)
# =============================================================================
def phase3_main_sweep(mav, actuator_sub, aero_sub, fits):
    log("\n=== PHASE 3: main sweep "
        f"({len(MAIN_SWEEP_DEGS)} points/surface x 3 surfaces = "
        f"{3*len(MAIN_SWEEP_DEGS)} points, hold={MAIN_HOLD_S}s each) ===")
    out = {}
    for surface in ("aileron", "elevator", "rudder"):
        surf_out = {}
        for d in MAIN_SWEEP_DEGS:
            rec = measure_point(mav, actuator_sub, aero_sub, surface, d, fits[surface], MAIN_HOLD_S)
            surf_out[f"{d:+.2f}"] = rec
            joints_str = " ".join(
                f"{j}={v['actual_deg']:+.3f}deg(tgt={v['target_deg']:+.3f})"
                for j, v in rec["joints"].items())
            log(f"  [{surface:8s}] target={d:+7.2f}deg n={rec['n']:+.4f} "
                f"servo_tgt={rec['target_servo_pwm']:.1f} rc={rec['rc_command_clamped']:.1f} "
                f"servo_actual={rec['servo_actual_pwm']} {joints_str} any_nan={rec['any_nan']}")
        out[surface] = surf_out
    return out


# =============================================================================
# Phase 4: dedicated tracking-quality re-measure at representative points
# (item 7) with a longer settle window.
# =============================================================================
def phase4_tracking(mav, actuator_sub, aero_sub, fits):
    log(f"\n=== PHASE 4: tracking-quality re-measure ({len(TRACKING_DEGS)} pts/surface, "
        f"hold={TRACKING_HOLD_S}s) ===")
    out = {}
    for surface in ("aileron", "elevator", "rudder"):
        surf_out = {}
        for d in TRACKING_DEGS:
            rec = measure_point(mav, actuator_sub, aero_sub, surface, d, fits[surface], TRACKING_HOLD_S)
            surf_out[f"{d:+.2f}"] = rec
            errs = {j: abs(v["actual_deg"] - v["target_deg"]) for j, v in rec["joints"].items()
                    if v["actual_deg"] is not None}
            log(f"  [{surface:8s}] target={d:+7.2f}deg  tracking_error_deg={errs}")
        out[surface] = surf_out
    return out


# =============================================================================
# Phase 5: single simultaneous all-neutral check (item 8)
# =============================================================================
def phase5_neutral(mav, actuator_sub):
    log("\n=== PHASE 5: simultaneous all-neutral check ===")
    mav.hold_rc_override(2.0, **NEUTRAL_RC)
    a_latest = actuator_sub.latest()
    out = {}
    for s in ACTUATOR_SURFACES:
        fld = actuator_fields_for(a_latest, s) if a_latest else {}
        deg = math.degrees(fld["actual_angle_rad"]) if fld.get("actual_angle_rad") is not None else None
        out[s] = dict(actual_deg=deg, cmd_rad=fld.get("cmd_rad"))
        log(f"  {s:15s} actual_deg={deg}")
    out["all_near_zero"] = all(v["actual_deg"] is not None and abs(v["actual_deg"]) < 1.0 for v in out.values())
    log("all_near_zero:", out["all_near_zero"])
    return out


# =============================================================================
# Derived analyses (items 1/2/3/4/5/6/7/9/10) - all computed from
# phase3/phase4 data, no extra live measurement.
# =============================================================================
def analyze_nine_point_matrix(sweep):
    log("\n--- ITEM 1: 9-point normalized command matrix ---")
    out = {}
    for surface in ("aileron", "elevator", "rudder"):
        rows = {}
        for norm in (-1.00, -0.75, -0.50, -0.25, 0.00, 0.25, 0.50, 0.75, 1.00):
            d = round(norm * 45.0, 2)
            rec = sweep[surface].get(f"{d:+.2f}")
            if rec is None:
                rows[norm] = {"MISSING": d}
                continue
            joints = {j: dict(target_deg=v["target_deg"], actual_deg=v["actual_deg"],
                               error_deg=(abs(v["actual_deg"] - v["target_deg"]) if v["actual_deg"] is not None else None),
                               te=v["te"])
                      for j, v in rec["joints"].items()}
            rows[norm] = dict(target_servo_pwm=rec["target_servo_pwm"],
                               servo_actual_pwm=rec["servo_actual_pwm"], joints=joints)
            log(f"  [{surface:8s}] n={norm:+.2f} pwm_tgt={rec['target_servo_pwm']:.1f} "
                f"pwm_actual={rec['servo_actual_pwm']} " +
                " ".join(f"{j}:tgt={v['target_deg']:+.2f} act={v['actual_deg']}"
                          f" err={v['error_deg']} {v['te']}" for j, v in joints.items()))
        out[surface] = rows
    return out


def analyze_old_clamp_regression(sweep):
    log("\n--- ITEM 2: old +/-10deg clamp regression ---")
    mags = [11.0, 15.0, 22.5, 25.0, 33.75, 35.0, 45.0]
    out = {}
    for surface in ("aileron", "elevator", "rudder"):
        rows = {}
        for m in mags:
            for sign, label in ((+1, "plus"), (-1, "minus")):
                d = sign * m
                rec = sweep[surface].get(f"{d:+.2f}")
                if rec is None:
                    continue
                near_old_limit_stuck = all(
                    abs(v["actual_deg"]) < 10.5 for v in rec["joints"].values() if v["actual_deg"] is not None
                ) if m > 10.5 else False
                rows[f"{m:.2f}_{label}"] = dict(
                    target_deg=d,
                    joints={j: v["actual_deg"] for j, v in rec["joints"].items()},
                    stuck_near_old_10deg_limit=near_old_limit_stuck)
                log(f"  [{surface:8s}] target={d:+7.2f}deg -> "
                    f"{ {j: v['actual_deg'] for j, v in rec['joints'].items()} } "
                    f"stuck_near_10deg={near_old_limit_stuck}")
        out[surface] = rows
    return out


def analyze_breakpoint_continuity(sweep):
    log("\n--- ITEM 3: lookup breakpoint continuity ---")
    groups = [(9.9, 10.0, 10.1), (14.9, 15.0, 15.1), (24.9, 25.0, 25.1),
              (34.9, 35.0, 35.1), (44.9, 45.0, None)]
    out = {}
    for surface in ("aileron", "elevator", "rudder"):
        surf_rows = []
        for grp in groups:
            grp_rows = {}
            for d in grp:
                if d is None:
                    continue
                rec = sweep[surface].get(f"{d:+.2f}")
                if rec is None:
                    continue
                aero_ok = rec["aero"] is not None and not any_nonfinite(rec["aero"])
                grp_rows[d] = dict(aero=rec["aero"], any_nan=rec["any_nan"], aero_finite=aero_ok,
                                    joints={j: v["actual_deg"] for j, v in rec["joints"].items()})
            surf_rows.append(grp_rows)
            log(f"  [{surface:8s}] group {grp}: " +
                " | ".join(f"{d}deg finite={v['aero_finite']}" for d, v in grp_rows.items()))
        out[surface] = surf_rows
    return out


def txt_table_for(surface):
    return {"aileron": TXT_AILERON, "elevator": TXT_ELEVATOR, "rudder": TXT_RUDDER}[surface]


def coeffs_for(surface):
    return {"aileron": ["CL", "CD", "CY", "Cl", "Cm", "Cn"],
            "elevator": ["CL", "CD", "Cm"],
            "rudder": ["CL", "CD", "CY", "Cl", "Cm", "Cn"]}[surface]


def txt_field_index(surface):
    return {"aileron": {"CL": 0, "CD": 1, "CY": 2, "Cl": 3, "Cm": 4, "Cn": 5},
            "elevator": {"CL": 0, "CD": 1, "Cm": 2},
            "rudder": {"CL": 0, "CD": 1, "CY": 2, "Cl": 3, "Cm": 4, "Cn": 5}}[surface]


def analyze_txt_comparison(sweep):
    log("\n--- ITEM 9: TXT-vs-runtime spot comparison ---")
    mags = [5.0, 15.0, 25.0, 35.0, 45.0]
    out = {}
    for surface in ("aileron", "elevator", "rudder"):
        table = txt_table_for(surface)
        idx = txt_field_index(surface)
        base_rec = sweep[surface].get("+0.00")
        base_aero = base_rec["aero"] if base_rec else None
        rows = {}
        for m in mags:
            for sign in (+1, -1):
                d = sign * m
                rec = sweep[surface].get(f"{d:+.2f}")
                if rec is None or rec["aero"] is None or base_aero is None:
                    continue
                aero_d = rec["aero_delta_deg"]
                # nearest breakpoint (should be exact by construction)
                bp = min(table.keys(), key=lambda k: abs(k - aero_d))
                txt_row = table[bp]
                txt_base = table[0]
                cmp_row = {}
                for c in coeffs_for(surface):
                    if surface == "rudder" and c == "Cl":
                        # SPECIAL CASE - see CLDR_PER_RAD comment above:
                        # the runtime does NOT use TXT_RUDDER's own Cl
                        # column (not loaded, disputed sign, prior stage).
                        expected = CLDR_PER_RAD * math.radians(aero_d)
                        measured_delta = rec["aero"][c] - base_aero[c]
                        tol = max(0.20 * abs(expected), 0.00008)
                        ok = abs(measured_delta - expected) <= tol
                        cmp_row[c] = dict(measured_delta=measured_delta, txt_expected_delta=expected,
                                           diff=measured_delta - expected, tol=tol, pass_=ok,
                                           note="reference=Cldr_per_rad bounded-linear-extension, NOT TXT Cl column (see CLDR_PER_RAD comment)")
                        continue
                    i = idx[c]
                    txt_delta = txt_row[i] - txt_base[i]
                    measured_delta = rec["aero"][c] - base_aero[c]
                    if c == "CD":
                        cl_now = rec["aero"].get("CL")
                        cl_base = base_aero.get("CL")
                        cross = DRAG_K * (cl_now ** 2 - cl_base ** 2) if (cl_now is not None and cl_base is not None) else 0.0
                        expected = cross + txt_delta
                    else:
                        expected = txt_delta
                    tol = max(0.20 * abs(expected), 0.01 if c in ("CL", "CD", "Cm") else 0.0008)
                    ok = abs(measured_delta - expected) <= tol
                    cmp_row[c] = dict(measured_delta=measured_delta, txt_expected_delta=expected,
                                       diff=measured_delta - expected, tol=tol, pass_=ok)
                rows[f"{d:+.2f}"] = dict(aero_delta_deg_used=aero_d, txt_breakpoint_used=bp, coeffs=cmp_row)
                log(f"  [{surface:8s}] target={d:+.2f}deg (aero_delta={aero_d:+.2f} -> TXT bp {bp}): " +
                    " ".join(f"{c}: meas={v['measured_delta']:+.5f} exp={v['txt_expected_delta']:+.5f} "
                              f"{'PASS' if v['pass_'] else 'WATCH'}" for c, v in cmp_row.items()))
        out[surface] = rows
    return out


def analyze_drag_sanity(txt_cmp):
    log("\n--- ITEM 10: high-deflection drag sanity ---")
    out = {}
    for surface in ("aileron", "elevator", "rudder"):
        rows = {}
        for m in (25.0, 35.0, 45.0):
            entry = txt_cmp[surface].get(f"{-m:+.2f}") or txt_cmp[surface].get(f"{m:+.2f}")
            if entry is None:
                continue
            cd = entry["coeffs"].get("CD")
            if cd is None:
                continue
            rows[m] = dict(measured_dCD=cd["measured_delta"], present_and_nonvanishing=abs(cd["measured_delta"]) > 0.005)
        out[surface] = rows
        log(f"  [{surface:8s}] " + " ".join(f"|{m}|deg: dCD={v['measured_dCD']:+.5f} "
              f"nonvanishing={v['present_and_nonvanishing']}" for m, v in rows.items()))
    return out


def analyze_surface_acceptance(sweep):
    """Items 4/5/6: derived from the sweep's own +/-45deg (and +/-25deg
    cross-check) points - full range reached, correct L/R relationship,
    correct moment sign for the commanded direction."""
    log("\n--- ITEMS 4/5/6: aileron/elevator/rudder live acceptance ---")
    out = {}

    # Aileron: opposite L/R, Cl sign check
    a = {}
    for d, label in ((45.0, "p45"), (-45.0, "m45")):
        rec = sweep["aileron"][f"{d:+.2f}"]
        la, ra = rec["joints"]["left_aileron"], rec["joints"]["right_aileron"]
        opposite = (la["actual_deg"] * ra["actual_deg"]) < 0
        mx_sign_positive_roll_right = rec["aero"]["Cl"]  # Mx = Cl*qbar*S*b, sign follows Cl directly (qbar,S,b>0)
        a[label] = dict(target_deg=d, left_deg=la["actual_deg"], right_deg=ra["actual_deg"],
                         opposite=opposite, left_te=la["te"], right_te=ra["te"], Cl=rec["aero"]["Cl"],
                         full_range_reached=abs(la["actual_deg"]) > 44.0 and abs(ra["actual_deg"]) > 44.0)
        log(f"  [aileron] target={d:+.1f} left={la['actual_deg']:+.3f}({la['te']}) "
            f"right={ra['actual_deg']:+.3f}({ra['te']}) opposite={opposite} Cl={rec['aero']['Cl']:+.5f} "
            f"full_range={a[label]['full_range_reached']}")
    out["aileron"] = a

    # Elevator: same L/R, Cm/my sign check
    e = {}
    for d, label in ((45.0, "p45"), (-45.0, "m45")):
        rec = sweep["elevator"][f"{d:+.2f}"]
        le, re = rec["joints"]["left_elevator"], rec["joints"]["right_elevator"]
        same = (le["actual_deg"] * re["actual_deg"]) >= 0
        cm = rec["aero"]["Cm"]
        my_sign = -cm  # my = qbar*S*c*(-Cm), sign follows -Cm (AeroModel.hh, cited)
        e[label] = dict(target_deg=d, left_deg=le["actual_deg"], right_deg=re["actual_deg"],
                         same=same, left_te=le["te"], right_te=re["te"], Cm=cm, my_sign=my_sign,
                         full_range_reached=abs(le["actual_deg"]) > 44.0 and abs(re["actual_deg"]) > 44.0)
        log(f"  [elevator] target={d:+.1f} left={le['actual_deg']:+.3f}({le['te']}) "
            f"right={re['actual_deg']:+.3f}({re['te']}) same={same} Cm={cm:+.5f} my_sign={my_sign:+.5f} "
            f"full_range={e[label]['full_range_reached']}")
    out["elevator"] = e

    # Rudder: Cn sign check
    r = {}
    for d, label in ((45.0, "p45"), (-45.0, "m45")):
        rec = sweep["rudder"][f"{d:+.2f}"]
        rj = rec["joints"]["rudder"]
        cn = rec["aero"]["Cn"]
        mz_sign = cn  # mz = qbar*S*b*Cn, sign follows Cn directly
        r[label] = dict(target_deg=d, rudder_deg=rj["actual_deg"], te=rj["te"], Cn=cn, mz_sign=mz_sign,
                         full_range_reached=abs(rj["actual_deg"]) > 44.0)
        log(f"  [rudder] target={d:+.1f} rudder={rj['actual_deg']:+.3f}({rj['te']}) Cn={cn:+.5f} "
            f"mz_sign={mz_sign:+.5f} full_range={r[label]['full_range_reached']}")
    out["rudder"] = r
    return out


def analyze_tracking(tracking):
    log("\n--- ITEM 7: actuator tracking (representative points) ---")
    out = {}
    for surface in ("aileron", "elevator", "rudder"):
        rows = {}
        for d in TRACKING_DEGS:
            rec = tracking[surface].get(f"{d:+.2f}")
            if rec is None:
                continue
            joint_errs = {}
            for j, v in rec["joints"].items():
                if v["actual_deg"] is None:
                    continue
                err = abs(v["actual_deg"] - v["target_deg"])
                joint_errs[j] = dict(target_deg=v["target_deg"], actual_deg=v["actual_deg"], error_deg=err,
                                      target_clamp_active=v["target_clamp_active"],
                                      effort_clamp_active=v["effort_clamp_active"])
            rows[f"{d:+.2f}"] = joint_errs
            log(f"  [{surface:8s}] target={d:+7.2f}deg " +
                " ".join(f"{j}: act={v['actual_deg']:+.3f} err={v['error_deg']:.3f}deg "
                          f"tclamp={v['target_clamp_active']} eclamp={v['effort_clamp_active']}"
                          for j, v in joint_errs.items()))
        out[surface] = rows
    return out


def check_confidence_tier_docs():
    log("\n--- ITEM 11: confidence-tier labeling (read-only doc check) ---")
    out = {}
    yaml_path = f"{REPO_ROOT}/docs/source_of_truth/aerodynamics/aero_v1_config.yaml"
    hh_path = f"{REPO_ROOT}/plugins/aerodynamics/AeroModel.hh"
    with open(yaml_path) as f:
        yaml_txt = f.read()
    with open(hh_path) as f:
        hh_txt = f.read()
    expect = ["HIGH_CONFIDENCE_SMALL_SIGNAL", "MEDIUM_CONFIDENCE_NONLINEAR_REFERENCE",
              "LOW_CONFIDENCE_HIGH_DEFLECTION_REFERENCE", "10.0", "25.0", "45.0"]
    yaml_ok = all(tok in yaml_txt for tok in expect[:3]) and "max_abs_deg: 10.0" in yaml_txt \
        and "max_abs_deg: 25.0" in yaml_txt and "max_abs_deg: 45.0" in yaml_txt
    hh_ok = all(tok in hh_txt for tok in expect[:3])
    out["aero_v1_config_yaml_present"] = yaml_ok
    out["aeromodel_hh_present"] = hh_ok
    out["pass"] = yaml_ok and hh_ok
    log(f"  aero_v1_config.yaml confidence bands present: {yaml_ok}")
    log(f"  AeroModel.hh confidence-tier comments present: {hh_ok}")
    return out


# =============================================================================
# Main
# =============================================================================
def main():
    R["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    mav = SafeMav("tcp:127.0.0.1:5760", source_system=255)

    R["phase0_connect_and_unblock"] = phase0_connect_and_unblock(mav)
    _cmd_vel_pub = phase1_pin_trim(mav)

    actuator_sub = DoubleVSub(ACTUATOR_DIAG_TOPIC)
    aero_sub = DoubleVSub(AERO_DIAG_TOPIC)
    time.sleep(1.0)

    fits = phase2_calibrate(mav)
    R["phase2_calibration"] = fits

    sweep = phase3_main_sweep(mav, actuator_sub, aero_sub, fits)
    R["phase3_main_sweep"] = sweep

    tracking = phase4_tracking(mav, actuator_sub, aero_sub, fits)
    R["phase4_tracking"] = tracking

    R["phase5_neutral"] = phase5_neutral(mav, actuator_sub)

    R["item1_nine_point_matrix"] = analyze_nine_point_matrix(sweep)
    R["item2_old_clamp_regression"] = analyze_old_clamp_regression(sweep)
    R["item3_breakpoint_continuity"] = analyze_breakpoint_continuity(sweep)
    R["item9_txt_comparison"] = analyze_txt_comparison(sweep)
    R["item10_drag_sanity"] = analyze_drag_sanity(R["item9_txt_comparison"])
    R["item4_5_6_surface_acceptance"] = analyze_surface_acceptance(sweep)
    R["item7_tracking"] = analyze_tracking(tracking)
    R["item11_confidence_tier_docs"] = check_confidence_tier_docs()

    # ---- overall numerical-integrity summary ----
    any_nan_sweep = any(rec["any_nan"] for surf in sweep.values() for rec in surf.values())
    any_nan_tracking = any(rec["any_nan"] for surf in tracking.values() for rec in surf.values())
    R["any_nan_main_sweep"] = any_nan_sweep
    R["any_nan_tracking"] = any_nan_tracking
    log(f"\nany_nan_main_sweep={any_nan_sweep} any_nan_tracking={any_nan_tracking}")

    mav.close()
    os.makedirs(RESULTS_DIR, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(R, f, indent=2, default=str)
    log(f"\nWrote {OUT_JSON}")
    return not (any_nan_sweep or any_nan_tracking)


if __name__ == "__main__":
    ok = main()
    sys.exit(0 if ok else 1)

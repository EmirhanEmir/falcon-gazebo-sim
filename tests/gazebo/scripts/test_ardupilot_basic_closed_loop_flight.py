#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_BASIC_CLOSED_LOOP_FLIGHT_VALIDATION (gazebo-testing,
2026-08-28). First-ever closed-loop ArduPlane FBWA-controlled airborne test
of this aircraft.

=============================================================================
UPDATE (2026-08-28, same day, methodology-correction pass): the ORIGINAL
force-RAMP approach below (module history preserved for the record) was
TEST_FAILED. Per an explicit follow-up instruction, this pass replaced the
ramp with aero_lib.hold_step()'s own PROVEN mechanism (fixed trim target,
reached fast via strong proportional feedback - NOT a slow ramp through
low-airspeed states) - see PHASE 3 (`phase3_hold_to_trim`) below. Doing so
surfaced and root-caused TWO real, independent GAZEBO-TESTING-OWNED
TEST-METHODOLOGY BUGS (neither is an aircraft-physics parameter, and
neither was "tuned" to make a test pass - see docs/test_results/2026-08-28_
ardupilot_basic_closed_loop_flight_validation.md sec 10+ for the full
live-evidence trail):

  BUG 1 - `/world/<world>/wrench/persistent` ACCUMULATES per publish, it
  does NOT replace the previously-published persistent wrench for the same
  entity (confirmed by a controlled A/B transport experiment: re-publishing
  a DIFFERENT force to the SAME entity's persistent topic, without an
  intervening `/wrench/clear`, left the FIRST force still fully active
  alongside the second). The ORIGINAL PHASE 3 (`phase3_force_ramp`, kept
  below unmodified for the historical record) republished to this exact
  topic in a tight loop for up to 15s WITHOUT ever clearing between
  iterations - meaning its true applied force grew catastrophically with
  every iteration, fully sufficient by itself to explain that version's
  observed "extreme pitch excursion" WITHOUT invoking any aerodynamic
  model behavior at all. This casts real doubt on the ORIGINAL run's
  "aerodynamic low-airspeed/high-alpha edge case" conclusion - it was very
  likely, at least in significant part, this accumulation bug instead.
  FIX: the new PHASE 3 uses the PLAIN, non-persistent `/world/<world>/wrench`
  topic (confirmed by the same kind of controlled experiment to genuinely
  auto-expire, not accumulate), republished every control-loop tick with a
  freshly computed value - the correct transport primitive for a
  continuously-updating force controller.

  BUG 2 (found AFTER fixing bug 1, via a second, still-divergent run) -
  aero_lib.hold_step()'s own gains (`kp_lin=150`, `kp_ang=400`) are tuned
  for, and only proven at, its ORIGINAL 1ms-per-tick, zero-latency,
  in-process `gz.sim8.TestFixture` control loop. Reused VERBATIM over this
  necessarily-different EXTERNAL gz-transport control loop (this stage's
  own measured mean loop period ~2.5ms, i.e. still fast, but ~2.5x slower
  than the proven script's own 1ms), `kp_ang=400` produces a growing,
  OSCILLATORY pitch-rate divergence (q ringing up to +/-1000s deg/s within
  <1s) - a textbook discrete-time proportional-control stability failure
  (loop-gain x sample-period, `kp_ang*dt`, exceeds the <1 margin needed for
  non-oscillatory convergence at this achievable external sample rate),
  NOT an aerodynamic-model artifact. A controlled diagnostic (identical
  fixed-target/topic/duration, ONLY `kp_ang` lowered to 100) converged
  CLEANLY to the exact trim condition with small, bounded attitude/rate
  excursions - confirming the mechanism and confirming there is no
  currently-visible aerodynamic obstruction to a clean hold-to-trim release
  once both methodology bugs are addressed. See the report for the full,
  single, theory-driven confirmatory run (not an open-ended gain search).

  BUG 3 (found live during PHASE 2's own rewrite, applying bugs 1/2's own
  lesson to a THIRD spot) - a raw OdometrySub body-velocity sample read
  immediately after a `set_pose` teleport can occasionally be a wild,
  non-physical glitch (a more extreme instance of the already-documented
  finding-3 "occasional bogus post-teleport velocity reading" - observed
  live this pass: a single 8999.9 m/s sample, one tick after teleport,
  sane again the very next tick). Feeding that raw sample into a
  proportional force controller (force = kp*mass*error) computed and
  genuinely APPLIED an ~8-million-newton force, crashing the physics
  engine outright (ODE collision-broadphase `aabbBound` assertion abort).
  FIX: `VEL_SANITY_MAX_MS` - any sample whose magnitude exceeds this bound
  is treated as stale and SKIPPED (no force computed/published that tick).
  `phase3_hold_to_trim` was already incidentally safe from this (its own
  `abort_v=40` safety envelope check runs BEFORE any force is computed),
  but PHASE 2's rewritten hold loop had no such check before this fix.

Net effect: `phase3_hold_to_trim` below uses `kp_lin=150` (proven value,
stable at this sample rate) and `kp_ang=100` (DEVIATES from hold_step()'s
own `kp_ang=400` - explicitly, documented, for the reason above - not a
silent change). No aircraft-physics parameter (mass, CG, inertia,
aerodynamic coefficient, control authority, motor thrust, PID gain) was
read for any purpose other than citing already-published/live-confirmed
values, and none is modified anywhere in this file, this pass included -
both `kp_lin`/`kp_ang` are this SCRIPT's own external controller gains
(test-harness-owned), never an aircraft/model parameter.
=============================================================================

ORIGINAL TOP-LINE RESULT (superseded by the update above - kept verbatim
for the historical record; see the report for the full before/after
picture): the prescribed pre-flight sequence's "airborne teleport + real
force-based velocity ramp" step (sec 4/5 of the task) is TEST_FAILED. The
aircraft naturally diverges into an extreme pitch/attitude excursion
within ~0.3-0.5s of the ramp starting, even under a PURE, deterministic,
zero-feedback, gravity-compensated + gentle-constant-acceleration force
schedule (i.e. NOT a controller-tuning artifact - see PHASE 5 below and
the report). This blocks the flight campaign before ArduPlane is ever
switched to FBWA; no fabricated flight data is produced by this script.
=============================================================================

Reused, unmodified: tests/gazebo/scripts/ardupilot_sitl_mav_lib.py
(SafeMav), the gdb-wrapped arduplane launch pattern, and the grounded-
world arm-readiness-gate pattern from the prior ARDUPLANE_SITL_TRANSPORT_
AND_ACTUATOR_MAPPING_VALIDATION stage (real, non-forceful MAV_CMD_
COMPONENT_ARM_DISARM only - no ARMING_SKIPCHK, no force-arm, no safety-
check suppression). New this stage: tests/gazebo/worlds/falcon_v2_
ardupilot_basic_closed_loop_flight_world.sdf (additive derivative of the
grounded test world + a world-scoped, stock upstream gz-sim-apply-link-
wrench-system, for the real force-based velocity ramp - see that file's
own header) and this script's force-ramp mechanism (PHASE 4/5 below).

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
control authority, motor thrust, PID gain) is read for any purpose other
than citing already-published/live-confirmed values, and NONE is modified
by this script. model/model.sdf, falcon_v2_sitl.parm, the real .param, and
every plugin under plugins/ are unmodified.

=============================================================================
KEY FINDINGS THIS STAGE (full evidence in the report; summarized here for
anyone re-running/extending this script)
=============================================================================

1. CG-OFFSET BUG (test-methodology, FIXED here): the stock gz-sim
   `ApplyLinkWrench` system applies `wrench.force` at the LINK ORIGIN by
   default (confirmed empirically), NOT the center of mass - unlike the
   in-process `Link::AddWorldForce()` API `aero_lib.hold_step()` uses
   elsewhere in this suite. Since base_link's `<inertial><pose>` (CG) is
   offset from the link origin by exactly (0.168309, 0, 0.100000) m
   (model/model.sdf, matches CLAUDE.md's documented CG), a pure
   translational hold force with zero commanded torque was observed to
   induce a large, real, r x F spurious moment (~-9.9 N*m about world Y
   for a ~59 N vertical hold force). Fix: every wrench publish in this
   script sets `wrench.force_offset` = that CG vector (link frame), so
   the force is correctly applied through the COM.

2. `World::SetPose` (`/world/<world>/set_pose`) DOES NOT reset the body's
   linear/angular velocity - confirmed decisively via direct Odometry
   ground-truth readback (teleporting a genuinely-at-rest aircraft over
   ANY distance, even 0.2 m, leaves velocity untouched; the service only
   ever touches position/orientation). Any gap between a successful
   teleport and applying real force leaves the aircraft in genuine,
   uncompensated free-fall for that gap's whole duration. Fix: this
   script publishes the gravity-feedforward hold wrench in the same
   breath as a successful teleport, before any verification polling.

3. An external, tight (sub-50ms-sleep), active Python polling loop
   against this project's `OdomSub`/`PoseSub` helper classes was found to
   CORRELATE with intermittent large bogus post-teleport velocity
   readings (10-20 m/s) that a passive `time.sleep()`-based polling
   pattern never reproduced in isolated diagnostics - suspected GIL/
   transport-callback-thread contention, not proven at the C++ level.
   Mitigation (not a fix at the root): this script uses passive,
   sleep-then-single-read polling throughout, never a tight loop.

4. DECISIVE FINDING (the actual blocker, NOT a test-harness artifact):
   with findings 1-3 all addressed/eliminated, and using a PURE,
   deterministic feedforward force schedule (constant acceleration in X,
   exact gravity compensation in Z, ZERO reactive P-feedback of any kind
   on any axis, ZERO commanded torque) - the gentlest, most conservative,
   least-latency-sensitive control scheme possible - the aircraft still
   naturally diverges into an extreme pitch/attitude excursion (pitch
   magnitude >45-60 deg, body pitch rate into the hundreds to 1000+ deg/s)
   within ~0.3-0.5 s of the ramp starting, driven by a genuine, growing,
   real aerodynamic force/moment (confirmed via Odometry ground-truth
   velocity, not a measurement artifact) as forward speed u climbs only
   to a few m/s while a naturally-induced vertical velocity w also
   develops. This is consistent with (and a live confirmation of) the
   already-flagged, general concern that this V1 aerodynamic model's
   linear/lookup-based coefficients are validated only in a bounded
   small-signal/moderate-deflection envelope and may not be well-behaved
   at extreme alpha - but this is the first time a LOW-AIRSPEED,
   FROM-REST TRANSIENT (not a control-surface deflection) has been shown
   to reach that regime. Every attempted mitigation (P-feedback at gains
   4 through 150, EMA smoothing at multiple time constants, a raised-
   cosine vs. linear vs. two-stage "kick" ramp profile, an emergency
   alpha-triggered correction) either failed to arrest the divergence or
   made it WORSE (bigger control authority -> bigger overshoot, given
   this external control loop's inherent tens-of-ms latency). Routed to
   `aerodynamics`/`validation` - not something gazebo-testing tunes or
   fixes.

=============================================================================
SCOPE OF WHAT THIS SCRIPT ACTUALLY DOES
=============================================================================
PHASE 1: connect MAVLink, wait for GPS/EKF3/DCM readiness (reused
         readiness-gate pattern), arm (real, non-forceful, with retry).
PHASE 2: teleport to the airborne test pose (one-shot, position+
         orientation only, via /world/<world>/set_pose) with the
         zero-gap gravity-feedforward fix (finding 2/3 above) and a
         sustained-velocity verification/retry gate.
PHASE 3 (UPDATED - see the top-of-file update note): `phase3_hold_to_trim`
         - reuses aero_lib.hold_step()'s own fixed-target mechanism (NOT a
         ramp): a real, external proportional force/torque controller
         (`kp_lin=150`, `kp_ang=100` - see update note for why `kp_ang`
         deviates from hold_step()'s own 400) drives body-frame linear/
         angular velocity directly to the trim condition
         (V_TRIM=18.162 m/s, alpha=2.472deg, zero rates - measured
         equilibrium, see PHASE 3 constants block below) from t=0, for a
         fixed HOLD_DURATION_S=0.8s (matching the proven script's own
         HOLD_STEPS=800 @ 1ms), using the plain (non-persistent, bug-1-fix)
         `/world/<world>/wrench` topic. Elevator is held at its own trim
         deflection (+4.092deg physical, both L/R) via a REAL RC2 override
         through the ArduPilotPlugin bridge (not a raw cmd_rad topic write,
         which would race with ArduPilotPlugin's own continuous republish
         onto that identical topic) - see ELEV_RC2_TARGET_US derivation.
         The same hard safety envelope (|v_body|>40 m/s or |roll|/|pitch|
         >60 deg) aborts immediately and preserves full evidence.
PHASE 4: if the hold completed cleanly, clear all wrench force (full
         release - zero further intervention on base_link, exactly like
         the proven script's own release window), drop the elevator RC2
         override back to neutral, and switch to FBWA (custom_mode=5) -
         t=0 of the actual flight evaluation. If the hold aborted, this
         script disarms safely, does NOT switch to FBWA, and reports
         TEST_FAILED with full evidence - it does not fabricate or
         continue a flight past this point.

`phase3_force_ramp` (the ORIGINAL, ramp-based mechanism) is kept below,
unmodified, for the historical record - `main()` no longer calls it.
"""
import json
import math
import os
import select
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import gz.transport13 as tp  # noqa: E402
from gz.msgs10 import (pose_v_pb2, entity_wrench_pb2, entity_pb2, pose_pb2,  # noqa: E402
                        boolean_pb2, odometry_pb2)
from pymavlink import mavutil  # noqa: E402

from ardupilot_sitl_mav_lib import SafeMav  # noqa: E402

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = f"{REPO_ROOT}/tests/gazebo/results"
OUT_JSON = os.path.join(RESULTS_DIR, "ardupilot_basic_closed_loop_ramp_result.json")

WORLD = "falcon_v2_ardupilot_basic_closed_loop_flight"
MASS = 6.000  # kg, CLAUDE.md - read-only citation, never modified
G = 9.81

MAV_CMD_COMPONENT_ARM_DISARM = 400
MAVLINK_MSG_ID_AHRS2 = 178
MAVLINK_MSG_ID_ATTITUDE = 30
MAV_CMD_SET_MESSAGE_INTERVAL = 511

# base_link's own <inertial><pose> offset from the link origin (model.sdf,
# matches CLAUDE.md's documented CG) - see module docstring finding 1.
CG_LINK_FRAME = (0.168309, 0.0, 0.100000)

# =============================================================================
# PHASE 3 (hold-to-trim) constants - see top-of-file update note.
# =============================================================================
# UPDATED 2026-08-28 by ARDUPLANE_FBWA_LEVEL_PITCH_REFERENCE_CORRECTION
# (controls-integration). The trim reference is now cited VERBATIM from the
# MEASURED pure-Gazebo straight-and-level equilibrium (C.2) in
# docs/test_results/2026-08-28_ardupilot_longitudinal_equilibrium_and_sink_
# root_cause_validation.md sec 6.2:
#     V*        = 18.162 m/s
#     alpha*    = 2.472 deg
#     throttle* = 0.4957      (Thrust - Drag = +0.02 N)
#     elevator* = +4.092 deg physical, both L/R  (aero delta_e = -4.092 deg)
#     pitch*    = +2.487 deg nose-up  -> now set as ArduPlane PTCH_TRIM_DEG
#                 = +2.49 in config/ardupilot/falcon_v2_sitl.parm
# This equilibrium (world_vz +0.005 m/s, L/W 0.996, 14/14 acceptance) was
# confirmed two independent ways: pure Gazebo with no autopilot, and
# ArduPlane MANUAL (flew level at pitch_mav +2.52 deg, V 18.11 m/s).
# It SUPERSEDES the prior reference (throttle=0.5010, elevator=+4.50deg,
# V_TRIM=18.166, alpha=2.473deg) from ARDUPLANE_TRIM_REFERENCE_CORRECTION_
# VALIDATION, which the equilibrium stage measured to carry ~0.5% excess
# throttle and ~0.4deg excess elevator (a mild +0.235 m/s climb in the free
# airframe, sub-threshold - see that report sec 12 hypotheses 2/3). The
# earlier trims (test_free_flight_dynamic_response.py 0.4915/+5.50deg;
# 2026-08-26_updated_powered_trim_high_deflection_validation.md 0.5010/
# +4.50deg) are both retired here.
# MASS_CTRL, I_DIAG remain cited VERBATIM from aero_lib.py / model/model.sdf
# (runtime-queried base_link controller-gain-only inputs, never fed back into
# any physics computation), reused for consistency with the proven script's
# own gains.
# =============================================================================
V_TRIM = 18.162
ALPHA_TRIM_DEG = 2.472
ALPHA_TRIM_RAD = math.radians(ALPHA_TRIM_DEG)
U_HOLD = V_TRIM * math.cos(ALPHA_TRIM_RAD)
W_HOLD = -V_TRIM * math.sin(ALPHA_TRIM_RAD)
MASS_CTRL = 5.9348  # kg, runtime-queried base_link mass (aero_lib.py) - controller-gain-only
I_DIAG = (0.7284, 0.2507, 0.9523)  # kg*m^2, base_link Ixx/Iyy/Izz (model/model.sdf) - controller-gain-only
KP_LIN = 150.0  # matches hold_step()'s own value - confirmed stable at this external sample rate
KP_ANG = 100.0  # DEVIATES from hold_step()'s own kp_ang=400 - see top-of-file update note (bug 2)
HOLD_DURATION_S = 0.8  # matches the proven script's HOLD_STEPS=800 @ 1ms

# BUG 3 (found live this pass, AFTER bugs 1/2): the very first Odometry
# sample read immediately after a `/world/<world>/set_pose` teleport can
# occasionally be a wild, non-physical finite-difference glitch (observed
# live: 8999.9 m/s on a single sample, one tick after teleport, settling to
# sane values by the very next sample) - a more extreme instance of the
# SAME already-documented finding-3 "occasional bogus post-teleport
# velocity reading" artifact. Feeding a raw, unguarded velocity sample like
# that into a proportional force controller (force = kp*mass*error) is
# catastrophic: it computed and genuinely APPLIED an ~8-million-newton
# force from that single bad sample, launching the aircraft to an
# unrecoverable state and crashing the physics engine's collision
# broadphase (ODE `aabbBound` assertion). FIX: any body-frame velocity
# sample whose magnitude exceeds this sanity bound is treated as stale/
# invalid and SKIPPED (no force computed or published that tick) rather
# than used - a test-harness input-validation fix, not a physics change.
VEL_SANITY_MAX_MS = 100.0  # far above V_TRIM=18.162 m/s and any real transient

# Elevator trim target (+4.092deg physical, both L/R - the MEASURED
# equilibrium elevator* from docs/test_results/2026-08-28_ardupilot_
# longitudinal_equilibrium_and_sink_root_cause_validation.md sec 6.2,
# superseding the prior +4.50deg reference; delta_e_aero = -0.5*
# (theta_left+theta_right) = -4.092deg, per CONTROLS.md's aero-convention
# formula), converted to an RC2 PWM target via the REAL, live-
# calibrated ArduPilotPlugin bridge formulas from the immediately-prior
# ARDUPLANE_CONTROL_SURFACE_TRAVEL_SCALING_VALIDATION stage (both cited,
# not re-derived/assumed):
#   cmd_rad = ELEV_MULT * (raw_cmd + ELEV_OFFSET)   [model/model.sdf SERVO2 control blocks]
#   servo_pwm = ELEV_SERVO_MIN + raw_cmd * (ELEV_SERVO_MAX - ELEV_SERVO_MIN)
#   servo_pwm = ELEV_RC_A + ELEV_RC_B * rc2         [live RC2->SERVO_OUTPUT_RAW calibration,
#                                                     max_resid=0.000 PWM, ardupilot_control_
#                                                     surface_travel_scaling_result.json]
# This RC2 target is used ONLY by phase3_hold_to_trim (pre-FBWA elevator pin);
# the FBWA evaluation segment itself commands RC2=1500 (neutral) and lets the
# autopilot pick the elevator. Recomputes to ELEV_RC2_TARGET_US ~= 1536.4 us.
ELEVATOR_THETA_RAD = math.radians(4.092)
ELEV_MULT = 1.5707963268
ELEV_OFFSET = -0.5
ELEV_SERVO_MIN = 800.0
ELEV_SERVO_MAX = 2200.0
ELEV_RC_A = -1125.0
ELEV_RC_B = 1.75
_elev_raw_cmd = ELEVATOR_THETA_RAD / ELEV_MULT - ELEV_OFFSET
_elev_servo_pwm = ELEV_SERVO_MIN + _elev_raw_cmd * (ELEV_SERVO_MAX - ELEV_SERVO_MIN)
ELEV_RC2_TARGET_US = (_elev_servo_pwm - ELEV_RC_A) / ELEV_RC_B

# Trim throttle: the MEASURED equilibrium throttle* = 0.4957 (Thrust - Drag
# = +0.02 N) from docs/test_results/2026-08-28_ardupilot_longitudinal_
# equilibrium_and_sink_root_cause_validation.md sec 6.2, superseding the
# prior 0.5010 reference (measured to carry ~0.5% excess -> mild climb).
# -> RC3 PWM via ArduPlane's standard linear RC3 (throttle) convention
# (1000us=0%, 2000us=100%, no reversal/trim concept for the throttle
# channel) - ASSUMPTION: not independently live-calibrated (the
# travel-scaling stage explicitly left throttle out of scope), flagged
# here rather than silently assumed. Recomputes to RC3_TRIM_TARGET_US
# = 1495.7 us (round -> 1496).
TRIM_THROTTLE = 0.4957
RC3_TRIM_TARGET_US = 1000.0 + TRIM_THROTTLE * 1000.0  # ASSUMPTION


# =============================================================================
# gz-transport helpers
# =============================================================================
def quat_to_rpy(qw, qx, qy, qz):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    s = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def cross3(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def rotate_body_to_world(q, v):
    """Standard active vector rotation by quaternion q=(qw,qx,qy,qz), where q
    is the body's orientation relative to world (as read from PoseSub) - i.e.
    rotates a BODY-frame vector v into WORLD frame, mirroring gz.math7's own
    Quaterniond::RotateVector() semantics (used by aero_lib.hold_step() via
    `rot.rotate_vector()`). Pure math, no gz.math7 dependency needed here."""
    qw, qx, qy, qz = q
    qv = (qx, qy, qz)
    t = tuple(2.0 * c for c in cross3(qv, v))
    ct = cross3(qv, t)
    return (v[0] + qw * t[0] + ct[0], v[1] + qw * t[1] + ct[1], v[2] + qw * t[2] + ct[2])


class PoseSub:
    def __init__(self, world, model_name="falcon_v2"):
        self.model_name = model_name
        self.node = tp.Node()
        self.hist = []
        self.lock = threading.Lock()
        ok = self.node.subscribe(pose_v_pb2.Pose_V, f"/world/{world}/pose/info", self._cb)
        if not ok:
            raise RuntimeError("subscribe failed")

    def _cb(self, msg):
        now = time.time()
        for p in msg.pose:
            if p.name == self.model_name:
                with self.lock:
                    self.hist.append((now, p.position.x, p.position.y, p.position.z,
                                       p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z))
                    if len(self.hist) > 60:
                        self.hist.pop(0)
                break

    def clear(self):
        with self.lock:
            self.hist = []

    def latest(self):
        with self.lock:
            return self.hist[-1] if self.hist else None


class OdomSub:
    """Real physics-engine-sourced twist (linear+angular velocity) via the
    stock gz-sim OdometryPublisher system - found this stage to be far
    more trustworthy than a pose-based finite-difference reconstruction
    over external gz-transport (see module docstring finding 3)."""

    def __init__(self):
        self.node = tp.Node()
        self.lock = threading.Lock()
        self.last = None
        ok = self.node.subscribe(odometry_pb2.Odometry, "/model/falcon_v2/odometry", self._cb)
        if not ok:
            raise RuntimeError("subscribe failed")

    def _cb(self, msg):
        with self.lock:
            self.last = msg

    def latest(self):
        with self.lock:
            return self.last


def set_pose(node, name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    req = pose_pb2.Pose()
    req.name = name
    req.position.x, req.position.y, req.position.z = x, y, z
    req.orientation.w, req.orientation.x, req.orientation.y, req.orientation.z = qw, qx, qy, qz
    result, rep = node.request(f"/world/{WORLD}/set_pose", req, pose_pb2.Pose, boolean_pb2.Boolean, 2000)
    return result, (rep.data if result else None)


def pub_wrench_persistent(pub, fx, fy, fz, tx, ty, tz, name="falcon_v2"):
    m = entity_wrench_pb2.EntityWrench()
    m.entity.name = name
    m.entity.type = entity_pb2.Entity.MODEL
    m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = fx, fy, fz
    m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = tx, ty, tz
    m.wrench.force_offset.x, m.wrench.force_offset.y, m.wrench.force_offset.z = CG_LINK_FRAME
    pub.publish(m)


def clear_wrench(pub_clear, name="falcon_v2"):
    e = entity_pb2.Entity()
    e.name = name
    e.type = entity_pb2.Entity.MODEL
    pub_clear.publish(e)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# =============================================================================
# PHASE 1: MAVLink connect / readiness / arm (reused pattern from the prior
# ARDUPLANE_SITL_TRANSPORT_AND_ACTUATOR_MAPPING_VALIDATION stage's grounded-
# world arming work - real, non-forceful, retry loop, no bypass of any kind)
# =============================================================================
def wait_dcm_ekf_ready(mav, timeout=30.0):
    """poll AHRS2 (DCM)/ATTITUDE (EKF3) divergence <10deg as the readiness
    gate, matching the already-validated pattern from the prior stage."""
    mav.m.mav.command_long_send(mav.m.target_system, mav.m.target_component,
                                 MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_AHRS2, 250000, 0, 0, 0, 0, 0)
    mav.m.mav.command_long_send(mav.m.target_system, mav.m.target_component,
                                 MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE, 250000, 0, 0, 0, 0, 0)
    t0 = time.time()
    last_ahrs2 = last_att = None
    while time.time() - t0 < timeout:
        r, _, _ = select.select([mav.m.port], [], [], 0.3)
        if not r:
            continue
        msg = mav.m.recv_match(type=["AHRS2", "ATTITUDE"], blocking=False)
        if msg is None:
            continue
        if msg.get_type() == "AHRS2":
            last_ahrs2 = msg
        else:
            last_att = msg
        if last_ahrs2 is not None and last_att is not None:
            droll = abs(math.degrees(last_ahrs2.roll - last_att.roll))
            dpitch = abs(math.degrees(last_ahrs2.pitch - last_att.pitch))
            if droll < 10.0 and dpitch < 10.0:
                return True, droll, dpitch
    return False, None, None


def arm(mav, timeout=4.0):
    mav.m.mav.command_long_send(mav.m.target_system, mav.m.target_component,
                                 MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    ack, sts = None, []
    while time.time() - t0 < timeout:
        r, _, _ = select.select([mav.m.port], [], [], 0.2)
        if not r:
            continue
        msg = mav.m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=False)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK" and msg.command == MAV_CMD_COMPONENT_ARM_DISARM:
            ack = msg.to_dict()
        elif msg.get_type() == "STATUSTEXT":
            sts.append(msg.text)
    return ack, sts


def disarm(mav, timeout=4.0):
    for _ in range(4):
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        time.sleep(0.15)
    mav.m.mav.command_long_send(mav.m.target_system, mav.m.target_component,
                                 MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        r, _, _ = select.select([mav.m.port], [], [], 0.2)
        if not r:
            continue
        msg = mav.m.recv_match(type="COMMAND_ACK", blocking=False)
        if msg and msg.command == MAV_CMD_COMPONENT_ARM_DISARM:
            return msg.to_dict()
    return None


def is_armed(mav, timeout=3.0):
    hb = mav.m.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
    if hb is None:
        return None
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def phase1_mavlink_arm(R):
    log = {}
    mav = SafeMav("tcp:127.0.0.1:5760", source_system=255)
    hb = mav.wait_heartbeat(20)
    log["heartbeat"] = hb.to_dict() if hb else None
    if hb is None:
        R["phase1_mavlink_arm"] = {"ok": False, "reason": "no heartbeat", **log}
        return None, False

    ready, droll, dpitch = wait_dcm_ekf_ready(mav, timeout=30.0)
    log["dcm_ekf_ready"] = ready
    log["dcm_ekf_droll_deg"] = droll
    log["dcm_ekf_dpitch_deg"] = dpitch
    if not ready:
        R["phase1_mavlink_arm"] = {"ok": False, "reason": "DCM/EKF3 never converged", **log}
        return mav, False

    armed = False
    for attempt in range(5):
        ack, sts = arm(mav)
        log[f"arm_attempt_{attempt}"] = {"ack": ack, "statustexts": sts}
        armed = is_armed(mav)
        if armed:
            break
        time.sleep(1.0)
    log["armed_confirmed"] = armed
    R["phase1_mavlink_arm"] = {"ok": bool(armed), **log}
    return mav, bool(armed)


def wait_ground_settle(osub, timeout=15.0, vmag_thresh=0.05):
    """Passive (see finding 3), sustained (not instantaneous) near-zero-
    velocity confirmation that the aircraft has genuinely finished its
    initial drop-onto-ground-plane settle (world file's own 0.5m spawn
    height) BEFORE ever teleporting it away. Required because World::
    SetPose does not reset velocity (finding 2) - teleporting while still
    mid-settle-transient carries real, non-negligible residual velocity
    into the airborne pose, contaminating everything downstream."""
    t0 = time.time()
    for _ in range(int(timeout / 0.5) + 1):
        time.sleep(0.5)
        od1 = osub.latest()
        if od1 is not None:
            m1 = max(abs(od1.twist.linear.x), abs(od1.twist.linear.y), abs(od1.twist.linear.z))
            if m1 < vmag_thresh:
                time.sleep(0.5)
                od2 = osub.latest()
                m2 = max(abs(od2.twist.linear.x), abs(od2.twist.linear.y), abs(od2.twist.linear.z)) if od2 else 999.0
                if m2 < vmag_thresh:
                    return True, time.time() - t0
        if time.time() - t0 > timeout:
            break
    return False, time.time() - t0


# =============================================================================
# PHASE 2: teleport + zero-gap gravity-feedforward + sustained-velocity
# verification/retry (findings 2/3 above)
# =============================================================================
def phase2_teleport_and_verify(node, sub, osub, pub_oneshot, target_pose, R, hold_window_s=0.35):
    """UPDATED (see top-of-file update note, bug 1): the original single
    `/wrench/persistent` gravity-feedforward publish was found, live this
    pass, to be unsafe across RETRY attempts (each retry compounds another
    persistent entry - a runaway, never-settling climb). A subsequent fix
    (clear-then-publish, zero gap) was ALSO found live to be unsafe for a
    DIFFERENT reason: `/wrench/clear` and `/wrench/persistent` are
    different topics with no cross-topic delivery-order guarantee over
    gz-transport - a controlled probe confirmed the clear can arrive AFTER
    the fresh publish it was meant to precede, wiping out the gravity hold
    entirely and free-falling (confirmed: growing velocity across repeated
    clear-then-publish bursts). FIX: PHASE 2 now uses the SAME validated,
    non-accumulating, per-tick mechanism as PHASE 3
    (`phase3_hold_to_trim`) - a real proportional force controller toward
    a ZERO body-velocity target, republished continuously on the plain,
    non-persistent `/world/<world>/wrench` topic for the ENTIRE
    `hold_window_s` settle+verify window (not a single feedforward
    publish) - this never touches `/wrench/persistent` or `/wrench/clear`
    at all, so neither bug applies here."""
    log = {"target_pose": target_pose, "attempts": []}
    x, y, z, roll, pitch, yaw = target_pose
    ok_v = False
    t_teleport = None
    for teleport_attempt in range(4):
        attempt_log = {"attempt": teleport_attempt + 1}
        r, ok = False, None
        for _ in range(5):
            r, ok = set_pose(node, "falcon_v2", x, y, z, roll, pitch, yaw)
            if r and ok:
                break
            time.sleep(0.3)
        attempt_log["set_pose_result"] = [r, ok]
        if not (r and ok):
            attempt_log["outcome"] = "set_pose_failed"
            log["attempts"].append(attempt_log)
            continue
        t_teleport = time.time()
        sub.clear()

        # Continuous zero-body-velocity/zero-body-rate proportional hold
        # (zero-gap, started in the exact same breath as the successful
        # teleport - finding 2) over the validated per-tick, non-
        # accumulating transport primitive - see docstring above. Angular
        # (rate) damping toward zero was ADDED after a live regression:
        # a translation-only version of this hold left attitude completely
        # uncontrolled, and real aerodynamic moments at ~zero airspeed
        # (an already-flagged low-airspeed regime) let the aircraft begin
        # tumbling freely, which then showed up as apparent, sometimes-
        # growing BODY-frame linear velocity even under active
        # translational damping (a rotating target for a body-frame-only
        # controller). Mirrors `phase3_hold_to_trim`'s own angular term.
        m1 = m2 = None
        thold0 = time.time()
        while True:
            th = time.time() - thold0
            if th > hold_window_s:
                break
            latest = sub.latest()
            od = osub.latest()
            if latest is None or od is None:
                time.sleep(0.001)
                continue
            lv_b = (od.twist.linear.x, od.twist.linear.y, od.twist.linear.z)
            av_b = (od.twist.angular.x, od.twist.angular.y, od.twist.angular.z)
            if max(abs(x) for x in lv_b) > VEL_SANITY_MAX_MS:
                # stale/glitched sample (bug 3) - skip this tick entirely,
                # do NOT compute or publish a force/torque from it
                time.sleep(0.001)
                continue
            qw, qx, qy, qz = latest[4:8]
            q = (qw, qx, qy, qz)
            f_body = tuple(-v * KP_LIN * MASS_CTRL for v in lv_b)  # target = (0,0,0)
            f_world = rotate_body_to_world(q, f_body)
            t_body = tuple(-w * i * KP_ANG for w, i in zip(av_b, I_DIAG))  # target = (0,0,0)
            t_world = rotate_body_to_world(q, t_body)
            pub_wrench_persistent(pub_oneshot, f_world[0], f_world[1], f_world[2],
                                   t_world[0], t_world[1], t_world[2])
            if m1 is None and th >= 0.1:
                m1 = max(abs(x) for x in lv_b)
            if th >= 0.3:
                m2 = max(abs(x) for x in lv_b)
            time.sleep(0.001)

        if m1 is not None and m2 is not None and m1 < 0.5 and m2 < 0.5:
            ok_v = True
        attempt_log["v1_mag"] = m1
        attempt_log["v2_mag"] = m2
        attempt_log["outcome"] = "clean" if ok_v else "not_sustained_clean"
        log["attempts"].append(attempt_log)
        if ok_v:
            t_teleport = time.time()
            break

    log["ok"] = ok_v
    R["phase2_teleport_verify"] = log
    return t_teleport, ok_v


# =============================================================================
# PHASE 3: force-based velocity ramp - finding 4's pure-feedforward
# configuration (the most conservative one found this stage; see module
# docstring - this is the configuration used to produce the decisive
# divergence evidence, kept as-is rather than "tuned to pass").
# =============================================================================
def phase3_force_ramp(pub, pub_clear, sub, osub, t_teleport, R,
                       v_target_final=18.16, t_ramp=15.0,
                       kp_x=4.0, max_trim_fx=35.0,
                       abort_v=40.0, abort_att_deg=60.0):
    log = {"config": dict(v_target_final=v_target_final, t_ramp=t_ramp, kp_x=kp_x,
                           max_trim_fx=max_trim_fx, abort_v=abort_v, abort_att_deg=abort_att_deg,
                           note="pure feedforward; ZERO reactive Y/Z feedback, ZERO torque - see finding 4")}
    samples = []
    aborted = False
    abort_reason = None
    n_iter = 0
    while True:
        t = time.time() - t_teleport
        if t > t_ramp:
            break
        accel_ff = v_target_final / t_ramp
        v_target = min(v_target_final, accel_ff * t)
        fx_ff = MASS * accel_ff if t < t_ramp else 0.0

        latest = sub.latest()
        od = osub.latest()
        lv = None
        if od is not None and t > 0.08:
            ox, oy, oz = od.twist.linear.x, od.twist.linear.y, od.twist.linear.z
            if max(abs(ox), abs(oy), abs(oz)) <= 60.0:
                lv = (ox, oy, oz)
        vx, vy, vz = lv if lv else (0.0, 0.0, 0.0)
        if latest is not None:
            qw, qx, qy, qz = latest[4:8]
            roll, pitch, yaw = quat_to_rpy(qw, qx, qy, qz)
        else:
            roll = pitch = yaw = 0.0

        if lv is not None and (abs(vx) > abort_v or abs(vy) > abort_v or abs(vz) > abort_v or
                                abs(math.degrees(roll)) > abort_att_deg or abs(math.degrees(pitch)) > abort_att_deg):
            aborted = True
            abort_reason = dict(t=t, v=[vx, vy, vz], roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch))
            break

        ex = v_target - vx
        fx = fx_ff + clamp(kp_x * MASS * ex, -max_trim_fx, max_trim_fx)
        fy = 0.0
        fz = MASS * G
        pub_wrench_persistent(pub, fx, fy, fz, 0.0, 0.0, 0.0)
        n_iter += 1

        if n_iter < 100 or n_iter % 10 == 0:
            samples.append(dict(t=t, v_target=v_target, v=[vx, vy, vz],
                                 att_deg=[math.degrees(roll), math.degrees(pitch), math.degrees(yaw)],
                                 F=[fx, fy, fz]))
        time.sleep(0.005)

    clear_wrench(pub_clear)
    log["aborted"] = aborted
    log["abort_reason"] = abort_reason
    log["n_iter"] = n_iter
    log["samples"] = samples
    R["phase3_force_ramp"] = log
    return not aborted


# =============================================================================
# PHASE 3 (UPDATED, current) - hold-to-trim, reusing aero_lib.hold_step()'s
# own fixed-target mechanism over the corrected transport primitive
# (plain, non-persistent /world/<world>/wrench - see top-of-file update
# note, bug 1) with a sample-rate-matched angular gain (bug 2). `pub_oneshot`
# MUST be advertised on `/world/<world>/wrench` (NOT `/wrench/persistent`) -
# see `main()`.
# =============================================================================
def phase3_hold_to_trim(pub_oneshot, pub_clear, sub, osub, mav, R,
                         hold_duration_s=HOLD_DURATION_S,
                         abort_v=40.0, abort_att_deg=60.0):
    log = {"config": dict(kp_lin=KP_LIN, kp_ang=KP_ANG, hold_duration_s=hold_duration_s,
                           lin_target_body=[U_HOLD, 0.0, W_HOLD], ang_target_body=[0.0, 0.0, 0.0],
                           elev_rc2_target_us=ELEV_RC2_TARGET_US, abort_v=abort_v,
                           abort_att_deg=abort_att_deg,
                           note="fixed hold_step()-style target reached fast (NOT a ramp) - see "
                                "top-of-file update note for kp_ang's deviation from hold_step()'s own value")}
    lin_target = (U_HOLD, 0.0, W_HOLD)
    ang_target = (0.0, 0.0, 0.0)
    samples = []
    dts = []
    aborted = False
    abort_reason = None
    n_iter = 0
    t0 = time.time()
    t_prev = t0
    last_rc_refresh = -1.0
    while True:
        t = time.time() - t0
        if t > hold_duration_s:
            break
        latest = sub.latest()
        od = osub.latest()
        if latest is None or od is None:
            time.sleep(0.001)
            continue
        now = time.time()
        dts.append(now - t_prev)
        t_prev = now

        qw, qx, qy, qz = latest[4:8]
        q = (qw, qx, qy, qz)
        lv_b = (od.twist.linear.x, od.twist.linear.y, od.twist.linear.z)
        av_b = (od.twist.angular.x, od.twist.angular.y, od.twist.angular.z)
        roll, pitch, yaw = quat_to_rpy(qw, qx, qy, qz)

        if (max(abs(lv_b[0]), abs(lv_b[1]), abs(lv_b[2])) > abort_v or
                abs(math.degrees(roll)) > abort_att_deg or abs(math.degrees(pitch)) > abort_att_deg):
            aborted = True
            abort_reason = dict(t=t, v_body=list(lv_b), av_body_deg=[math.degrees(x) for x in av_b],
                                 roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch))
            break

        e_lin = tuple(lt - lv for lt, lv in zip(lin_target, lv_b))
        f_body = tuple(e * KP_LIN * MASS_CTRL for e in e_lin)
        f_world = rotate_body_to_world(q, f_body)

        e_ang = tuple(at - av for at, av in zip(ang_target, av_b))
        t_body = tuple(e * i * KP_ANG for e, i in zip(e_ang, I_DIAG))
        t_world = rotate_body_to_world(q, t_body)

        pub_wrench_persistent(pub_oneshot, f_world[0], f_world[1], f_world[2],
                               t_world[0], t_world[1], t_world[2])

        # Elevator trim hold via a REAL RC2 override through the ArduPilotPlugin
        # bridge, refreshed ~every 0.1s (well under RC_OVERRIDE_TIME=3.0s) -
        # see module docstring "elevator note" / ELEV_RC2_TARGET_US derivation.
        if mav is not None and (t - last_rc_refresh) >= 0.1:
            mav.send_rc_override(rc1=1500, rc2=int(round(ELEV_RC2_TARGET_US)), rc3=1000, rc4=1500, rc5=1000)
            last_rc_refresh = t

        n_iter += 1
        if n_iter < 400 or n_iter % 4 == 0:
            samples.append(dict(t=t, v_body=list(lv_b), av_body_deg=[math.degrees(x) for x in av_b],
                                 att_deg=[math.degrees(roll), math.degrees(pitch), math.degrees(yaw)],
                                 F_world=list(f_world), T_world=list(t_world)))
        time.sleep(0.001)

    # Full release - zero further force/torque intervention, matching the
    # proven script's own release-window semantics.
    clear_wrench(pub_clear)
    if dts:
        log["loop_dt_ms"] = dict(mean=sum(dts) / len(dts) * 1000.0, min=min(dts) * 1000.0, max=max(dts) * 1000.0, n=len(dts))
    log["aborted"] = aborted
    log["abort_reason"] = abort_reason
    log["n_iter"] = n_iter
    log["samples"] = samples
    if samples:
        log["end_state"] = samples[-1]
    R["phase3_hold_to_trim"] = log
    return not aborted


# =============================================================================
# PHASE 4 - FBWA handoff + short post-release stabilization observation.
# Switches ArduPlane to FBWA (custom_mode=5) at the exact instant PHASE 3's
# hold releases, drops the elevator RC2 override back to neutral RC (the
# autopilot, not this script, is responsible for elevator from this point
# on), sets RC3 to an ASSUMPTION-labeled trim-throttle PWM (RC3_TRIM_TARGET_US
# - see that constant's own derivation comment), and observes real,
# ground-truth (Odometry) state for `obs_duration_s` with ZERO further
# force/torque/pose/velocity intervention of any kind - a genuinely free,
# closed-loop ArduPlane-controlled flight window.
# =============================================================================
MAV_CMD_DO_SET_MODE = 176
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
ARDUPLANE_FBWA_CUSTOM_MODE = 5


def phase4_fbwa_handoff_and_observe(mav, osub, sub, R, obs_duration_s=10.0):
    log = {}
    mav.m.mav.set_mode_send(mav.m.target_system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ARDUPLANE_FBWA_CUSTOM_MODE)
    t_fbwa = time.time()
    # neutral RC (autopilot now owns attitude/elevator); RC3 per the
    # ASSUMPTION-labeled trim-throttle mapping.
    mav.send_rc_override(rc1=1500, rc2=1500, rc3=int(round(RC3_TRIM_TARGET_US)), rc4=1500, rc5=1000)

    # Confirmation: keep draining HEARTBEATs for the full window and track
    # the LAST one seen, breaking early only once custom_mode==FBWA is
    # actually observed - NOT the first heartbeat received (found live,
    # this pass: the very first heartbeat after a mode-switch request can
    # be one already in flight/queued before ArduPilot processed the mode
    # change, still reporting the OLD mode - confirmed via a standalone
    # follow-up probe using the identical set_mode_send() call, which
    # showed the mode change itself is reliable; only this confirmation
    # loop's "accept the first message" logic was the bug).
    hb = None
    t_hb0 = time.time()
    while time.time() - t_hb0 < 5.0:
        r, _, _ = select.select([mav.m.port], [], [], 0.3)
        if not r:
            continue
        msg = mav.m.recv_match(type="HEARTBEAT", blocking=False)
        if msg is None:
            continue
        hb = msg
        if hb.custom_mode == ARDUPLANE_FBWA_CUSTOM_MODE:
            break
    log["mode_after_switch"] = dict(custom_mode=hb.custom_mode, base_mode=hb.base_mode,
                                     armed=bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)) if hb else None
    log["fbwa_confirmed"] = bool(hb and hb.custom_mode == ARDUPLANE_FBWA_CUSTOM_MODE)

    samples = []
    aborted = False
    abort_reason = None
    last_rc_refresh = 0.0
    while time.time() - t_fbwa < obs_duration_s:
        tnow = time.time() - t_fbwa
        if tnow - last_rc_refresh >= 0.1:
            mav.send_rc_override(rc1=1500, rc2=1500, rc3=int(round(RC3_TRIM_TARGET_US)), rc4=1500, rc5=1000)
            last_rc_refresh = tnow
        od = osub.latest()
        latest = sub.latest()
        if od is not None and latest is not None:
            qw, qx, qy, qz = latest[4:8]
            roll, pitch, yaw = quat_to_rpy(qw, qx, qy, qz)
            v_body = (od.twist.linear.x, od.twist.linear.y, od.twist.linear.z)
            av_body = (od.twist.angular.x, od.twist.angular.y, od.twist.angular.z)
            samples.append(dict(t=tnow, pos_z=latest[3], v_body=list(v_body),
                                 av_body_deg=[math.degrees(x) for x in av_body],
                                 att_deg=[math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]))
            if abs(math.degrees(roll)) > 80.0 or abs(math.degrees(pitch)) > 80.0 or latest[3] < 5.0:
                aborted = True
                abort_reason = samples[-1]
                break
        time.sleep(0.02)

    log["aborted"] = aborted
    log["abort_reason"] = abort_reason
    log["n_samples"] = len(samples)
    log["samples"] = samples
    if samples:
        log["start_state"] = samples[0]
        log["end_state"] = samples[-1]
    R["phase4_fbwa_handoff_and_observe"] = log
    return not aborted


# =============================================================================
# Main
# =============================================================================
def main():
    R = {"timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}

    node = tp.Node()
    sub = PoseSub(WORLD)
    osub = OdomSub()
    time.sleep(0.5)
    # `pub_oneshot` (plain, non-persistent topic) is the ONLY wrench
    # publisher PHASE 2/PHASE 3 use now - see top-of-file update note, bug 1
    # (`/wrench/persistent` accumulates per publish) and PHASE 2's own
    # docstring (bug 1 fix #2: clear-then-publish on `/wrench/persistent`
    # is ALSO unsafe, confirmed live, due to no cross-topic delivery-order
    # guarantee). `pub_clear` is kept only as a defensive no-op / for the
    # historical `phase3_force_ramp` reference path.
    pub_oneshot = node.advertise(f"/world/{WORLD}/wrench", entity_wrench_pb2.EntityWrench)
    pub_clear = node.advertise(f"/world/{WORLD}/wrench/clear", entity_pb2.Entity)
    time.sleep(0.3)

    mav, armed = phase1_mavlink_arm(R)
    print("PHASE 1 (mavlink connect/readiness/arm):", json.dumps(R.get("phase1_mavlink_arm", {}), default=str)[:500])
    if not armed:
        print("PHASE 1 FAILED - not proceeding.")
        os.makedirs(RESULTS_DIR, exist_ok=True)
        with open(OUT_JSON, "w") as f:
            json.dump(R, f, indent=2, default=str)
        return 1

    settled, settle_elapsed = wait_ground_settle(osub)
    R["ground_settle"] = {"settled": settled, "elapsed_s": settle_elapsed}
    print("ground settle (pre-teleport, sustained near-zero velocity):", R["ground_settle"])
    if not settled:
        print("GROUND SETTLE FAILED - not proceeding to teleport.")
        disarm(mav)
        os.makedirs(RESULTS_DIR, exist_ok=True)
        with open(OUT_JSON, "w") as f:
            json.dump(R, f, indent=2, default=str)
        return 1

    # PHASE 2: teleport to ~90m AGL, level, forward-pointing (world +X)
    t_teleport, ok_v = phase2_teleport_and_verify(
        node, sub, osub, pub_oneshot, (0.0, 0.0, 90.0, 0.0, 0.0, 0.0), R)
    print("PHASE 2 (teleport+verify):", R["phase2_teleport_verify"]["ok"],
          [a["outcome"] for a in R["phase2_teleport_verify"]["attempts"]])
    if not ok_v:
        print("PHASE 2 FAILED - could not achieve a clean post-teleport state. Disarming, not proceeding.")
        disarm(mav)
        os.makedirs(RESULTS_DIR, exist_ok=True)
        with open(OUT_JSON, "w") as f:
            json.dump(R, f, indent=2, default=str)
        return 1

    # PHASE 2 never touches /wrench/persistent now (see its own docstring) -
    # this clear is a harmless defensive no-op before PHASE 3 begins.
    clear_wrench(pub_clear)

    # PHASE 3 (UPDATED): hold-to-trim, fixed target reached fast
    hold_ok = phase3_hold_to_trim(pub_oneshot, pub_clear, sub, osub, mav, R)
    print("PHASE 3 (hold-to-trim): aborted =", R["phase3_hold_to_trim"]["aborted"],
          "reason =", R["phase3_hold_to_trim"]["abort_reason"],
          "loop_dt_ms =", R["phase3_hold_to_trim"].get("loop_dt_ms"))

    if not hold_ok:
        print("PHASE 3 FAILED (TEST_FAILED) - hold-to-trim diverged before reaching trim condition. "
              "NOT switching to FBWA. Disarming and stopping - no flight data will be fabricated.")
        # drop any lingering elevator RC override before disarming
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        disarm(mav)
        R["overall_result"] = "TEST_FAILED"
        R["blocking_phase"] = "phase3_hold_to_trim"
        os.makedirs(RESULTS_DIR, exist_ok=True)
        with open(OUT_JSON, "w") as f:
            json.dump(R, f, indent=2, default=str)
        mav.close()
        return 1

    # PHASE 4: FBWA handoff (t=0 of the flight evaluation), immediately
    # following PHASE 3's release with zero intervening gap - neutral RC,
    # short free-flight stabilization observation window.
    stable = phase4_fbwa_handoff_and_observe(mav, osub, sub, R, obs_duration_s=10.0)
    p4 = R["phase4_fbwa_handoff_and_observe"]
    print("PHASE 4 (FBWA handoff+observe): fbwa_confirmed =", p4["fbwa_confirmed"],
          "aborted =", p4["aborted"], "reason =", p4["abort_reason"],
          "start =", p4.get("start_state"), "end =", p4.get("end_state"))

    disarm(mav)
    if not p4["fbwa_confirmed"]:
        R["overall_result"] = "TEST_FAILED"
        R["blocking_phase"] = "phase4_fbwa_mode_switch"
        print("PHASE 4 FAILED - FBWA mode switch not confirmed.")
    elif not stable:
        R["overall_result"] = "TEST_FAILED"
        R["blocking_phase"] = "phase4_fbwa_handoff_and_observe"
        print("PHASE 4 FAILED - aircraft diverged/crashed during the post-release FBWA observation window.")
    else:
        R["overall_result"] = "PHASE3_4_PASSED_PRECONDITION_MET"
        print("PRECONDITION MET: hold-to-trim released cleanly into a stable, bounded FBWA window. "
              "See report for whether the full 3-flight campaign (roll/pitch commands) was also run this pass.")

    os.makedirs(RESULTS_DIR, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(R, f, indent=2, default=str)
    mav.close()
    return 0 if R["overall_result"] == "PHASE3_4_PASSED_PRECONDITION_MET" else 1


if __name__ == "__main__":
    sys.exit(main())

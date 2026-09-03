#!/usr/bin/env python3
"""
FALCON V2 - SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION
Airspeed / wind acceptance matrix + short TECS regression
(gazebo-testing, 2026-09-02).

WHAT THIS TESTS
---------------
`controls-integration` landed two fixes (record:
docs/source_of_truth/autopilot/SITL_ATMOSPHERE_AND_AIRSPEED.md):

  M-1  SITL atmosphere datum: `-O 0,0,0,0` silently forced the 584 m CMAC
       elevation (SITL_cmdline.cpp:761-766). Now removed; origin comes from
       SIM_OPOS_* (SIM_Aircraft.cpp:694-707 update_home(), no CMAC path).
  M-2  ArduPilotPlugin now emits the OFFICIAL SIM_JSON `airspeed` (EAS) and
       `velocity_wind` (NED) keys, fed from FalconV2Pitot and FalconV2Wind, so
       SIM_JSON.cpp:437-453 no longer takes the `wind_ef.zero()` else-branch.

This test is the ACCEPTANCE MATRIX for M-2 (and re-checks M-1's datum every
run through a param precondition). It answers ONE question with numbers:

    does the air-relative airspeed that Falcon V2's own pitot measures in
    Gazebo actually reach ArduPlane, and then actually reach TECS, with the
    right magnitude and the right SIGN, in wind?

THE MEASUREMENT PROBLEM, AND HOW IT IS HANDLED
----------------------------------------------
The project owner's criterion is: *compared at approximately the same aircraft
GROUNDSPEED*, airspeed must go ~+5 m/s in headwind and ~-5 m/s in tailwind.

But in a CLOSED-LOOP TECS cruise TECS holds AIRSPEED, so airspeed does NOT
move - the GROUNDSPEED does. Both statements are the same physics
(V_air = V_ground - V_wind along the flight path); which one you see depends on
what is being held constant. This test therefore measures BOTH, separately and
labelled, and never converts one into the other:

  PROBE 1 - "matched groundspeed" (steady state, the owner's literal criterion)
    Fly a segment whose TECS airspeed demand is chosen so that the resulting
    GROUNDSPEED matches a zero-wind reference segment. Then read off the
    airspeed difference directly.
        MG_LOW  (groundspeed ~16 m/s):  zero-wind demand 16.00  vs
                                        headwind demand 20.92  -> +~5 airspeed
        MG_HIGH (groundspeed ~21 m/s):  zero-wind demand 20.92  vs
                                        tailwind demand 16.00   -> -~5 airspeed
    Why these numbers and not "18 +/- 5": at matched groundspeed G the headwind
    case must fly at G+5 and the tailwind case at G-5. With AIRSPEED_MIN = 16
    (below which TECS clamps the demand) and NO_VALID_TRIM at >= 24 m/s
    (tests/gazebo/results/flight_envelope_result.json), a SINGLE G that serves
    both is impossible: it needs G <= ~16.5 and G >= ~21 simultaneously. So the
    matched-groundspeed comparison is done PAIRWISE at two different
    groundspeeds. This is a real, recorded limitation of the aircraft's
    trimmable envelope, NOT a fudge - and 24 m/s NO_VALID_TRIM is the known
    high-J/windmilling prop-table gap that is explicitly out of scope here.
    The two demands actually used are 16.00 and 20.92 m/s, i.e. 4.92 m/s apart,
    not exactly 5: 18.00 and 21.00 are NOT reachable through RC3 because
    RC_Channel::control_in is an int16_t (RC_Channel.cpp:316), quantising the
    demand grid to (AIRSPEED_MAX-AIRSPEED_MIN)/100 = 0.12 m/s. The test uses
    the ACHIEVED demand, never the nominal one, and reports the residual
    groundspeed mismatch explicitly.

  PROBE 1b - "matched groundspeed, exactly" (the identity, mismatch-corrected)
    The residual groundspeed mismatch is removed algebraically. Along the
    flight path (+X world, the aircraft is spawned nose-along-+X):
        V_air = V_groundX - W_X
      =>  dV_air - dV_groundX  ==  -dW_X   EXACTLY, for any two segments.
    So (airspeed_case - airspeed_ref) - (groundspeed_case - groundspeed_ref)
    must equal -W_X = +5 (headwind, W_X = -5) / -5 (tailwind, W_X = +5),
    independent of how well the groundspeeds happened to match. This is the
    tightest form of the criterion and is checked with a tight tolerance.

  PROBE 2 - "closed-loop cruise" (TECS holds airspeed, groundspeed moves)
    A 18 m/s FBWB cruise segment flown in all three wind cases. Here airspeed
    must STAY at the demand in all three, while groundspeed must shift by
    +W_X: 18 / 13 / 23 for zero / headwind / tailwind. This is the same physics
    seen from the other side, and it is also the TASK B short regression.

  PROBE 0 - "pre-step zero-wind reference"
    Every run flies an IDENTICAL zero-wind 18 m/s cruise segment BEFORE the
    wind is switched on, so the three runs are known to start from the same
    flight condition and any cross-run drift is visible rather than assumed.

WIND CONVENTION (docs/source_of_truth/environment/WIND.md sec 2)
    /model/falcon_v2/wind carries the VELOCITY OF THE AIR MASS in the Gazebo
    world frame (ENU), m/s. NOT meteorological "wind from" phrasing. The
    aircraft is teleported to yaw = 0, i.e. nose along world +X. Therefore:
        HEADWIND  = air mass moving toward -X = (-5, 0, 0)
        TAILWIND  = air mass moving toward +X = (+5, 0, 0)
    Commanded live on /model/falcon_v2/wind/steady_cmd (gz.msgs.Vector3d).
    FalconV2Wind is a MODEL-level plugin present in every world and defaulting
    to zero, so NO test-world change is needed and none is made.

THE THREE QUANTITIES COMPARED, AND WHERE EACH COMES FROM
    1. GAZEBO PITOT   /model/falcon_v2/sensors/pitot/airspeed_mps (gz.msgs.Double)
                      = |V_rel| from FalconV2Pitot. Sampled live in this script.
    2. ARDUPLANE      dataflash ARSP.Airspeed (EAS), post-processed from the
                      .BIN by analyze_dataflash_airspeed.py. NOT VFR_HUD:
                      ARSPD_OPTIONS=11 (AP_Airspeed.h:180-184) can disable the
                      sensor on an airspeed/groundspeed inconsistency and
                      GCS_MAVLink_Plane.cpp:256-266 then substitutes a
                      synthetic value. VFR_HUD.airspeed is ALSO recorded, and
                      the two are compared, precisely so that substitution
                      would be visible rather than silent.
    3. TECS           dataflash TECS.sp (_TAS_state, TRUE airspeed) and
                      TECS.spdem (_TAS_dem). TECS.sp / ARSP.Airspeed is the
                      live EAS2TAS and is reported per case - that is the M-1
                      atmosphere-datum readout.
    GROUNDSPEED is recorded ALONGSIDE all of them, from Gazebo ground truth
    (world-frame position derivative and body velocity) AND from VFR_HUD, so
    airspeed and groundspeed can never be confused with one another.

PER-TEST PARAMETER CHANGES (NOT written to falcon_v2_sitl.parm)
    SIM_ARSPD_RND = 0, set over MAVLink at test time ONLY. The firmware default
    2.0 is rectified by sqrt(|ratio*(q + noise)|) (sitl_airspeed.cpp:37-39)
    into a positive airspeed bias near zero airspeed (~0.58 m/s at true 0),
    which would contaminate a quantitative comparison. It is DELIBERATELY not
    put in the .parm: that would perturb the validated TECS baseline.
    The pre-change value is read back and recorded, and restored at the end.

HARD CONSTRAINTS OBSERVED
    No aero coefficient, propulsion value, actuator parameter, PID, TECS_*
    parameter, PTCH_TRIM_DEG, control-surface scaling, mass, CG or inertia is
    read-modified anywhere. No SDF, no plugin, no .parm file is written. No
    physics parameter is touched to make anything pass. Nothing is retried with
    altered physics.

USAGE (a fresh gz sim + gdb-wrapped arduplane pair must already be running -
see run_ardupilot_airspeed_wind_acceptance.sh):
    python3 test_ardupilot_airspeed_wind_acceptance.py <zero|headwind|tailwind>
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
from gz.msgs10 import entity_wrench_pb2, entity_pb2, vector3d_pb2, double_pb2  # noqa: E402

import test_ardupilot_basic_closed_loop_flight as base  # noqa: E402
import test_ardupilot_basic_closed_loop_flight_campaign as campaign  # noqa: E402
import test_ardupilot_fbwa_level_pitch_reference_correction as fbwa  # noqa: E402
import test_ardupilot_tecs_cruise_speed_hold as tecs  # noqa: E402
import aero_lib  # noqa: E402
import propulsion_lib  # noqa: E402
import actuator_lib  # noqa: E402

mean = fbwa.mean
stdev = fbwa.stdev
linreg = fbwa.linreg
read_param = fbwa.read_param

STAGE = "SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION__AIRSPEED_WIND_ACCEPTANCE"

PITOT_TOPIC = "/model/falcon_v2/sensors/pitot/airspeed_mps"
WIND_OUT_TOPIC = "/model/falcon_v2/wind"
WIND_CMD_TOPIC = "/model/falcon_v2/wind/steady_cmd"

# -----------------------------------------------------------------------------
# Wind cases. World-frame ENU airmass velocity, m/s. Aircraft nose is +X.
# -----------------------------------------------------------------------------
WIND_CASES = {
    "zero":     (0.0, 0.0, 0.0),
    "headwind": (-5.0, 0.0, 0.0),
    "tailwind": (+5.0, 0.0, 0.0),
    # BRIEF SANITY CHECK ONLY (not part of the +/-5 acceptance matrix).
    # The three cases above are all along world X, so on their own they cannot
    # distinguish a correct X mapping from an X/Y swap anywhere in the
    # Gazebo-ENU -> ArduPilot-NED wind rotation. This case moves the air mass
    # along world +Y (ENU North) instead, and asserts that ArduPlane reports
    # the wind as coming FROM the South (bearing 180 deg) at 5 m/s, and that
    # the pitot still equals |V_ground_world - V_wind_world|. NO matched-
    # groundspeed segment is flown: with a lateral airmass and no heading hold
    # the aircraft weathervanes, so a "matched groundspeed" comparison would
    # not be a clean steady state. That is stated rather than papered over.
    "crosswind": (0.0, +5.0, 0.0),
}
WIND_MAG_MS = 5.0

# -----------------------------------------------------------------------------
# Segment plan. Nominal demands; the ACHIEVED (int16-quantised) demand is
# derived at runtime from live parameters and is what every comparison uses.
# -----------------------------------------------------------------------------
V_CRUISE_MS = 18.0        # = AIRSPEED_CRUISE, the Probe-2 closed-loop cruise
V_MG_LOW_MS = 16.0        # = AIRSPEED_MIN  -> demand lands exactly on 16.00
V_MG_HIGH_MS = 21.0       # -> demand lands on 20.92 (0.12 m/s int16 grid)

SEG_REF_S = 40.0          # Probe 0: zero wind, 18 m/s, IDENTICAL in all runs
SEG_CRUISE_S = 55.0       # Probe 2: wind on, 18 m/s
SEG_MG_S = 55.0           # Probe 1: wind on, matched-groundspeed demand
SETTLE_S = 25.0           # discarded head of every analysis window

# Which matched-groundspeed segments each case flies.
MG_PLAN = {
    "zero":     [("MG_LOW", V_MG_LOW_MS), ("MG_HIGH", V_MG_HIGH_MS)],
    "headwind": [("MG_HIGH", V_MG_HIGH_MS)],   # gs ~= 20.92 - 5 = 15.92 ~ MG_LOW
    "tailwind": [("MG_LOW", V_MG_LOW_MS)],     # gs ~= 16.00 + 5 = 21.00 ~ MG_HIGH
    "crosswind": [],                            # sanity case only - see WIND_CASES
}

# =============================================================================
# ACCEPTANCE THRESHOLDS - every value justified here, in the file that uses it.
# =============================================================================
# (1) ArduPlane vs Gazebo pitot agreement. These are the SAME scalar travelling
#     over UDP through a 10 Hz zero-order hold and ArduPilot's own first-order
#     pressure filter (AP_Airspeed::read(), 0.7/0.3 blend at 10 Hz => ~0.28 s
#     time constant) and back out through sqrt(q). In a settled hold window
#     that chain is lossless to well under 0.1 m/s. 0.25 m/s is ~1.4 % of an
#     18 m/s cruise: tight enough that a broken transport (which would show a
#     WHOLE-WIND-VECTOR, i.e. 5 m/s, error) cannot hide, loose enough not to
#     trip on filter lag against a slightly unsteady airspeed.
TH_AP_VS_PITOT_MS = 0.25
# (2) The owner's criterion, raw form: airspeed delta at approximately matched
#     groundspeed. Expected |delta| is the achieved-demand difference (4.92 m/s
#     for the segments actually flown), and the band must absorb the residual
#     groundspeed mismatch plus hold noise. 0.6 m/s = 12 % of the 5 m/s signal;
#     a broken transport gives 0 m/s here, which is 8 sigma outside.
TH_MG_DELTA_TOL_MS = 0.6
# (3) The identity form (Probe 1b), which removes the groundspeed mismatch
#     algebraically and must therefore be much tighter. Residual is only sensor
#     filtering and hold noise.
TH_MG_IDENTITY_TOL_MS = 0.35
# (4) Residual groundspeed mismatch allowed before "matched groundspeed" stops
#     being an honest description. 1.0 m/s = 5-6 % of the ~16/~21 m/s
#     groundspeeds compared. Reported numerically either way.
TH_MG_GS_MISMATCH_MS = 1.0
# (5) Probe 2: TECS must hold the airspeed demand in wind as well as it does in
#     still air. Reuses the validated TECS stage's own band verbatim so the two
#     stages stay directly comparable (test_ardupilot_tecs_cruise_speed_hold.py
#     TH_SPEED_MEAN_TOL_MS / TH_SPEED_STD_MAX_MS).
TH_HOLD_MEAN_TOL_MS = tecs.TH_SPEED_MEAN_TOL_MS   # 0.5
TH_HOLD_STD_MAX_MS = tecs.TH_SPEED_STD_MAX_MS     # 0.5
# (6) Probe 2: groundspeed must move by +W_X. Same 0.6 m/s band as (2), same
#     reasoning - this is the identical physical quantity seen from the other
#     side.
TH_GS_SHIFT_TOL_MS = 0.6
# (7) EAS2TAS sanity (M-1 readout). The declared datum makes ArduPlane AMSL ==
#     Gazebo z; at the ~90 m test altitude ISA gives EAS2TAS = 1.0043, and the
#     pre-fix 584 m datum gave 1.0331. 1.015 sits between them, so this check
#     cannot pass if the 584 m datum ever returns, and does not fail on the
#     genuine ISA density gradient the aerodynamics plugin does not model
#     (SITL_ATMOSPHERE_AND_AIRSPEED.md sec 1.7, open item, NOT this test's to
#     close).
TH_EAS2TAS_MAX = 1.015
# (8) Throttle saturation. Same bar as the TECS stage.
TH_SAT_RUN_MAX_S = tecs.TH_SAT_RUN_MAX_S
# (9) Airspeed sensor must remain USED and HEALTHY in flight (ARSP.U / ARSP.H).
#     If ARSPD_OPTIONS=11's consistency logic disabled the sensor in flight,
#     ArduPlane's "airspeed" would silently become a synthetic AHRS value and
#     the whole comparison would be meaningless. Fraction of in-window samples
#     required.
TH_ARSP_USED_FRAC_MIN = 0.99
# (10) ArduPlane's own AHRS wind-estimate magnitude vs the commanded 5 m/s.
#      With AHRS_EKF_TYPE=10 this is a straight pass-through of the SIM_JSON
#      `velocity_wind` key (AP_AHRS_SIM.cpp:168), so it should be near-exact;
#      0.3 m/s covers telemetry rounding and the brief step transient that the
#      SETTLE_S window head already removes. It was IDENTICALLY ZERO before the
#      M-2 fix, so this check has ~17 sigma of separation from the failure mode
#      it is guarding.
TH_AP_WIND_TOL_MS = 0.3

# Flight-safety envelope (abort, preserving every sample collected so far).
ALT_FLOOR_M = tecs.ALT_FLOOR_M
ALT_CEILING_M = tecs.ALT_CEILING_M


# =============================================================================
# Gazebo pitot / wind plumbing
# =============================================================================
class PitotSub:
    """Live subscriber to FalconV2Pitot's scalar airspeed output. Same raw
    subscribe pattern as test_sensor_model_live_validation.py's RawSub."""

    def __init__(self):
        self.node = tp.Node()
        self._v = None
        self._n = 0
        ok = self.node.subscribe_raw(PITOT_TOPIC, self._cb, "gz.msgs.Double",
                                     tp.SubscribeOptions())
        if not ok:
            raise RuntimeError(f"failed to subscribe to {PITOT_TOPIC}")

    def _cb(self, data, info):
        m = double_pb2.Double()
        m.ParseFromString(data)
        self._v = m.data
        self._n += 1

    def latest(self):
        return self._v

    def count(self):
        return self._n


class WindOutSub:
    """Ground truth of what FalconV2Wind actually PUBLISHED - independent of
    what this script asked for. Guards against a commanded-but-not-applied
    wind silently turning the whole matrix into a zero-wind run."""

    def __init__(self):
        self.node = tp.Node()
        self._v = None
        self._n = 0
        ok = self.node.subscribe_raw(WIND_OUT_TOPIC, self._cb, "gz.msgs.Vector3d",
                                     tp.SubscribeOptions())
        if not ok:
            raise RuntimeError(f"failed to subscribe to {WIND_OUT_TOPIC}")

    def _cb(self, data, info):
        m = vector3d_pb2.Vector3d()
        m.ParseFromString(data)
        self._v = (m.x, m.y, m.z)
        self._n += 1

    def latest(self):
        return self._v

    def count(self):
        return self._n


class WindCommander:
    """Publishes /steady_cmd every tick. OnSteadyCmd() is a plain overwrite
    (WindSystem.cc), so republishing is idempotent and safe."""

    def __init__(self):
        self.node = tp.Node()
        self.pub = self.node.advertise(WIND_CMD_TOPIC, vector3d_pb2.Vector3d)
        self.steady = (0.0, 0.0, 0.0)

    def set_steady(self, x, y, z):
        self.steady = (float(x), float(y), float(z))

    def tick(self):
        m = vector3d_pb2.Vector3d()
        m.x, m.y, m.z = self.steady
        self.pub.publish(m)


def set_param(mav, name, value, timeout=6.0):
    """Set a SITL parameter over MAVLink and confirm the read-back.
    Used ONLY for SIM_ARSPD_RND (see module docstring). Never writes a file."""
    from pymavlink import mavutil as _mu
    deadline = time.time() + timeout
    while time.time() < deadline:
        mav.m.mav.param_set_send(mav.m.target_system, mav.m.target_component,
                                 name.encode("ascii"), float(value),
                                 _mu.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < 1.5:
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
                return float(msg.param_value)
    return None


# =============================================================================
# per-sample derived quantities
# =============================================================================
def s_alt(s):
    return s["gz"]["pos"][2] if s["gz"]["pos"] is not None else None


def s_gz_groundspeed(s):
    """Groundspeed magnitude from Gazebo ground truth body velocity. The
    ArduPilotPlugin/odometry twist is BODY frame; its magnitude is the
    inertial ground speed magnitude, frame-independent."""
    v = s["gz"]["v_body"]
    if v is None or not all(math.isfinite(x) for x in v):
        return None
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def s_gz_groundspeed_x(s):
    """Signed world +X (== nominal flight-path) component of groundspeed.
    Needed for the Probe-1b identity, which is one-dimensional along +X.
    Reconstructed from the body velocity and the Gazebo attitude."""
    v = s["gz"]["v_body"]
    a = s["gz"]["att_deg"]
    if v is None or a is None:
        return None
    if not (all(math.isfinite(x) for x in v) and all(math.isfinite(x) for x in a)):
        return None
    r, p, y = (math.radians(a[0]), math.radians(a[1]), math.radians(a[2]))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    # world_x row of R_wb (ZYX intrinsic yaw-pitch-roll, the convention
    # base.quat_to_rpy() inverts)
    return (v[0] * (cy * cp)
            + v[1] * (cy * sp * sr - sy * cr)
            + v[2] * (cy * sp * cr + sy * sr))


def s_throttle(s):
    t = s["mav"]["throttle_pct"]
    return (t / 100.0) if t is not None else None


def finite(xs):
    return [x for x in xs if x is not None and isinstance(x, (int, float)) and math.isfinite(x)]


def stat(xs):
    v = finite(xs)
    if not v:
        return dict(n=0, mean=None, std=None, min=None, max=None)
    return dict(n=len(v), mean=mean(v), std=(stdev(v) if len(v) > 1 else 0.0),
                min=min(v), max=max(v))


# =============================================================================
# segment runner (tecs.run_seg + wind ticking + pitot sampling)
# =============================================================================
def run_seg(mav, sub, osub, adiag, pdiag, aerodiag, pitot, windout, windcmd,
            label, duration_s, rc1, rc2, rc3, t_flight0, latest_mav):
    samples = []
    aborted = False
    abort_reason = None
    t0 = time.time()
    last_rc = -1.0
    last_sample = -1.0
    last_wind = -1.0
    while True:
        tnow = time.time() - t0
        if tnow > duration_s:
            break
        campaign.drain_mavlink(mav, latest_mav)
        # Wind is republished at the RC rate. It is an idempotent overwrite;
        # republishing guards against a single dropped gz-transport message
        # silently reverting the case to zero wind.
        if tnow - last_wind >= campaign.RC_REFRESH_PERIOD:
            windcmd.tick()
            last_wind = tnow
        if tnow - last_rc >= campaign.RC_REFRESH_PERIOD:
            mav.send_rc_override(rc1=int(round(rc1)), rc2=int(round(rc2)),
                                 rc3=int(round(rc3)), rc4=1500, rc5=1000)
            last_rc = tnow
        if tnow - last_sample >= campaign.SAMPLE_PERIOD:
            s = campaign.build_sample(time.time() - t_flight0, latest_mav, sub, osub,
                                      adiag, pdiag, aerodiag)
            s["t_seg"] = tnow
            s["t_wall"] = time.time()
            s["pitot_airspeed_mps"] = pitot.latest()
            s["wind_world_mps"] = windout.latest()
            wmsg = latest_mav.get("WIND")
            s["ap_wind"] = (dict(direction_deg=wmsg.direction, speed_ms=wmsg.speed,
                                 speed_z_ms=wmsg.speed_z) if wmsg else None)
            samples.append(s)
            last_sample = tnow
            att = s["gz"]["att_deg"]
            pos = s["gz"]["pos"]
            bad = None
            if att is not None:
                if not (math.isfinite(att[0]) and math.isfinite(att[1])):
                    bad = "nonfinite_attitude"
                elif abs(att[0]) > tecs.ATT_ABORT_DEG or abs(att[1]) > tecs.ATT_ABORT_DEG:
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
        time.sleep(0.005)
    return dict(label=label, duration_s=duration_s, actual_duration_s=time.time() - t0,
                rc1=rc1, rc2=rc2, rc3=rc3, n_samples=len(samples), samples=samples,
                aborted=aborted, abort_reason=abort_reason)


# =============================================================================
# window analysis
# =============================================================================
def analyze_window(seg, settle_s, wind_expected):
    """Everything is computed on the SETTLED tail of the segment only."""
    ss = [s for s in seg["samples"] if s.get("t_seg", 0.0) >= settle_s]
    out = dict(label=seg["label"], n_total=seg["n_samples"], n_window=len(ss),
               settle_s=settle_s)
    if len(ss) < 40:
        out["insufficient_samples"] = True
        return out
    out["insufficient_samples"] = False
    t = [s["t_seg"] for s in ss]

    out["pitot_airspeed_mps"] = stat([s.get("pitot_airspeed_mps") for s in ss])
    out["vfr_airspeed_mps"] = stat([s["mav"]["airspeed"] for s in ss])
    out["vfr_groundspeed_mps"] = stat([s["mav"]["groundspeed"] for s in ss])
    out["gz_groundspeed_mps"] = stat([s_gz_groundspeed(s) for s in ss])
    out["gz_groundspeed_x_mps"] = stat([s_gz_groundspeed_x(s) for s in ss])
    out["altitude_m"] = stat([s_alt(s) for s in ss])
    out["throttle"] = stat([s_throttle(s) for s in ss])
    out["pitch_phys_deg"] = stat([tecs.s_pitch_phys(s) for s in ss])
    out["nav_aspd_error_cms"] = stat([s["mav"]["nav_aspd_error"] for s in ss])

    alts = [s_alt(s) for s in ss]
    pair = [(tt, a) for tt, a in zip(t, alts) if a is not None and math.isfinite(a)]
    if len(pair) > 5:
        sl, _ = linreg([q[0] for q in pair], [q[1] for q in pair])
        out["vertical_speed_regression_ms"] = sl
        out["altitude_p2p_m"] = max(q[1] for q in pair) - min(q[1] for q in pair)
    asp = finite([s.get("pitot_airspeed_mps") for s in ss])
    if len(asp) > 5:
        out["pitot_slope_per_s"] = linreg(
            [tt for tt, s in zip(t, ss) if s.get("pitot_airspeed_mps") is not None
             and math.isfinite(s["pitot_airspeed_mps"])], asp)[0]

    # actual wind applied, as PUBLISHED by FalconV2Wind (ground truth)
    wx = [s["wind_world_mps"][0] for s in ss if s.get("wind_world_mps") is not None]
    wy = [s["wind_world_mps"][1] for s in ss if s.get("wind_world_mps") is not None]
    wz = [s["wind_world_mps"][2] for s in ss if s.get("wind_world_mps") is not None]
    out["wind_world_applied"] = dict(x=stat(wx), y=stat(wy), z=stat(wz),
                                     expected=list(wind_expected))
    out["wind_matches_command"] = bool(
        wx and abs(mean(wx) - wind_expected[0]) < 1e-6
        and wy and abs(mean(wy) - wind_expected[1]) < 1e-6
        and wz and abs(mean(wz) - wind_expected[2]) < 1e-6)

    # ArduPlane vs Gazebo pitot, sample-by-sample (the transport check)
    d = [s["mav"]["airspeed"] - s["pitot_airspeed_mps"] for s in ss
         if s["mav"]["airspeed"] is not None and s.get("pitot_airspeed_mps") is not None]
    out["vfr_minus_pitot_ms"] = stat(d)

    # ArduPlane's OWN wind estimate (MAVLink WIND <- AP_AHRS_SIM state.wind_ef
    # <- the SIM_JSON `velocity_wind` key). Direction is the meteorological
    # "wind is coming FROM" bearing in degrees; speed is the horizontal
    # magnitude. Zero before the M-2 fix, in every case.
    out["ap_wind_speed_ms"] = stat([s["ap_wind"]["speed_ms"] for s in ss
                                    if s.get("ap_wind")])
    out["ap_wind_direction_deg"] = stat([s["ap_wind"]["direction_deg"] for s in ss
                                         if s.get("ap_wind")])

    # airspeed MINUS groundspeed - the quantity that is identically zero when
    # the transport is broken (because ArduPlane's "airspeed" is then the
    # ground velocity), and equals -W_X when it works.
    dgs = [s["mav"]["airspeed"] - g for s, g in
           ((s, s_gz_groundspeed_x(s)) for s in ss)
           if s["mav"]["airspeed"] is not None and g is not None]
    out["ap_airspeed_minus_gz_groundspeed_x_ms"] = stat(dgs)

    # NaN / Inf sweep over every numeric telemetry field in the window
    nan_count = 0
    for s in ss:
        for grp in ("mav", "gz"):
            for _, v in (s[grp] or {}).items():
                if isinstance(v, float) and not math.isfinite(v):
                    nan_count += 1
                elif isinstance(v, (list, tuple)):
                    nan_count += sum(1 for q in v
                                     if isinstance(q, float) and not math.isfinite(q))
        pv = s.get("pitot_airspeed_mps")
        if isinstance(pv, float) and not math.isfinite(pv):
            nan_count += 1
    out["nan_inf_count"] = nan_count

    # throttle saturation runs (VFR_HUD throttle is an integer percent)
    thr = [s_throttle(s) for s in ss]
    def longest_run(pred):
        best = cur = 0
        prev_t = None
        for tt, x in zip(t, thr):
            if x is not None and pred(x):
                cur += 0.0 if prev_t is None else (tt - prev_t)
                best = max(best, cur)
            else:
                cur = 0.0
            prev_t = tt
        return best
    out["throttle_sat_high_longest_run_s"] = longest_run(lambda x: x >= 0.995)
    out["throttle_sat_low_longest_run_s"] = longest_run(lambda x: x <= 0.005)

    # actuator clamp (target/effort limiting inside the actuator model)
    clamp_t = clamp_e = 0
    for s in ss:
        a = s.get("actuators")
        if not a:
            continue
        for _, d in a.items():
            clamp_t += 1 if d["target_clamp_active"] else 0
            clamp_e += 1 if d["effort_clamp_active"] else 0
    out["actuator_clamp"] = dict(target_clamp_active_samples=clamp_t,
                                 effort_clamp_active_samples=clamp_e)
    return out


# =============================================================================
# flight
# =============================================================================
def flight_sequence(case, mav, sub, osub, adiag, pdiag, aerodiag,
                    pitot, windout, windcmd, p, R):
    wind = WIND_CASES[case]

    # --- derive every RC3 command from LIVE parameters (nothing hardcoded) ---
    cmds = {}
    for name, v_nom in [("CRUISE", V_CRUISE_MS)] + MG_PLAN[case]:
        pwm, achieved, err = tecs.rc3_pwm_for_target_airspeed(v_nom, p)
        if pwm is None:
            R["flight_result"] = dict(aborted=True, reason=f"rc3_derivation_failed:{name}:{err}")
            return False, {}
        pwm_i = int(round(pwm))
        ci, demand = tecs.achievable_target_airspeed(pwm_i, p)
        cmds[name] = dict(nominal_ms=v_nom, rc3_pwm_us=pwm_i, control_in_int16=ci,
                          achieved_demand_ms=demand,
                          quantisation_error_ms=demand - v_nom)
    R["command_derivation"] = dict(
        commands=cmds,
        demand_grid_ms=(int(p["AIRSPEED_MAX"]) - int(p["AIRSPEED_MIN"])) / 100.0,
        formula="navigation.cpp:187-189 inverted through RC_Channel.cpp:388-402, "
                "with the RC_Channel.cpp:316 int16_t control_in truncation modelled")
    print("command derivation:", json.dumps(cmds, default=str))

    rc2_neutral = int(round(p["RC2_TRIM"]))
    if not tecs.enter_fbwb(mav, cmds["CRUISE"]["rc3_pwm_us"], R):
        R["flight_result"] = dict(aborted=True, reason="fbwb_handoff_not_confirmed")
        return False, {}

    t0 = time.time()
    latest = {}
    for msg_id in campaign.MSG_IDS_20HZ:
        campaign.request_rate(mav, msg_id, 20.0)
    # MAVLINK_MSG_ID_WIND = 168. ArduPlane fills it from AP_AHRS::wind_estimate()
    # (GCS_Common.cpp send_wind()). With AHRS_EKF_TYPE = 10 in force,
    # AP_AHRS_SIM.cpp:168 returns _sitl->state.wind_ef - i.e. exactly the
    # `velocity_wind` key the patched ArduPilotPlugin now sends. This is a
    # THIRD, independent readout of the M-2 fix: before the fix wind_ef was
    # unconditionally zeroed by SIM_JSON.cpp:445, so this message would read
    # 0.00 m/s in every wind case.
    campaign.request_rate(mav, 168, 5.0)
    time.sleep(0.3)

    segs = {}

    def go(label, dur, rc3):
        print(f"SEGMENT {label}: {dur:.0f}s  rc3={rc3}  wind={windcmd.steady}")
        seg = run_seg(mav, sub, osub, adiag, pdiag, aerodiag, pitot, windout, windcmd,
                      label, dur, 1500, rc2_neutral, rc3, t0, latest)
        segs[label] = seg
        if seg["aborted"]:
            print(f"  ABORTED: {seg['abort_reason']['reason']}")
        return not seg["aborted"]

    # --- Probe 0: zero-wind reference cruise. Identical in all three runs. ---
    windcmd.set_steady(0.0, 0.0, 0.0)
    if not go("P0_zero_wind_ref_cruise", SEG_REF_S, cmds["CRUISE"]["rc3_pwm_us"]):
        R["flight_result"] = dict(aborted=True, reason="P0_aborted")
        return False, segs

    # --- Probe 2: wind ON, same 18 m/s demand. TECS holds airspeed. ---------
    windcmd.set_steady(*wind)
    if not go("P2_wind_on_cruise", SEG_CRUISE_S, cmds["CRUISE"]["rc3_pwm_us"]):
        R["flight_result"] = dict(aborted=True, reason="P2_aborted")
        return False, segs

    # --- Probe 1: matched-groundspeed segment(s), wind still ON ------------
    for name, _ in MG_PLAN[case]:
        if not go(f"P1_{name}", SEG_MG_S, cmds[name]["rc3_pwm_us"]):
            R["flight_result"] = dict(aborted=True, reason=f"P1_{name}_aborted")
            return False, segs

    windcmd.set_steady(0.0, 0.0, 0.0)
    for _ in range(20):
        windcmd.tick()
        time.sleep(0.05)

    R["flight_result"] = dict(
        aborted=False,
        segment_summary=[(k, v["n_samples"], v["aborted"]) for k, v in segs.items()])
    return True, segs


# =============================================================================
# analysis + verdict (per-run; the cross-case matrix is assembled by
# build_airspeed_wind_matrix.py once all three runs exist)
# =============================================================================
def analyze(R, segs, case):
    wind = WIND_CASES[case]
    an = {}
    an["P0_zero_wind_ref_cruise"] = analyze_window(
        segs["P0_zero_wind_ref_cruise"], SETTLE_S, (0.0, 0.0, 0.0))
    an["P2_wind_on_cruise"] = analyze_window(
        segs["P2_wind_on_cruise"], SETTLE_S, wind)
    for name, _ in MG_PLAN[case]:
        k = f"P1_{name}"
        if k in segs:
            an[k] = analyze_window(segs[k], SETTLE_S, wind)
    R["analysis"] = an
    return an


def verdict(R, case):
    """PER-RUN checks only: the things that must be true of THIS run regardless
    of the other two. The cross-case +/-5 m/s comparison is deliberately NOT
    done here - it needs all three runs and is done by
    build_airspeed_wind_matrix.py, which is where its tolerances live."""
    an = R.get("analysis") or {}
    wind = WIND_CASES[case]
    c = {}
    pre = R.get("param_preconditions", {})
    c["param_preconditions_all_ok"] = all(pre.values()) if pre else False

    wins = [k for k in an]
    c["all_windows_present"] = bool(wins) and all(
        not an[k].get("insufficient_samples") for k in wins)
    if not c["all_windows_present"]:
        R["acceptance_checks"] = c
        return "AIRSPEED_WIND_ACCEPTANCE_FAILED", [k for k, v in c.items() if not v]

    c["flight_not_aborted"] = not R.get("flight_result", {}).get("aborted", True)
    c["no_nan_inf"] = all(an[k].get("nan_inf_count") == 0 for k in wins)
    c["no_sustained_throttle_high_saturation"] = all(
        an[k].get("throttle_sat_high_longest_run_s", 0.0) <= TH_SAT_RUN_MAX_S for k in wins)
    c["no_sustained_throttle_low_saturation"] = all(
        an[k].get("throttle_sat_low_longest_run_s", 0.0) <= TH_SAT_RUN_MAX_S for k in wins)
    c["zero_actuator_clamp"] = all(
        (an[k]["actuator_clamp"]["target_clamp_active_samples"]
         + an[k]["actuator_clamp"]["effort_clamp_active_samples"]) == 0 for k in wins)

    # wind actually applied, in every wind-on window
    wind_on = [k for k in wins if k != "P0_zero_wind_ref_cruise"]
    c["commanded_wind_actually_applied"] = all(an[k]["wind_matches_command"] for k in wind_on)
    c["reference_window_is_zero_wind"] = an["P0_zero_wind_ref_cruise"]["wind_matches_command"]

    # ArduPlane airspeed agrees with the Gazebo pitot, in EVERY window
    worst = None
    for k in wins:
        m = an[k]["vfr_minus_pitot_ms"]["mean"]
        if m is not None and (worst is None or abs(m) > abs(worst)):
            worst = m
    c["arduplane_agrees_with_gazebo_pitot"] = worst is not None and abs(worst) <= TH_AP_VS_PITOT_MS
    R["worst_vfr_minus_pitot_ms"] = worst

    # TECS holds its demand in the 18 m/s cruise, wind on
    dem = R["command_derivation"]["commands"]["CRUISE"]["achieved_demand_ms"]
    a2 = an["P2_wind_on_cruise"]["pitot_airspeed_mps"]
    c["tecs_holds_airspeed_demand_with_wind"] = (
        a2["mean"] is not None and abs(a2["mean"] - dem) <= TH_HOLD_MEAN_TOL_MS)
    c["airspeed_hold_bounded_with_wind"] = (
        a2["std"] is not None and a2["std"] <= TH_HOLD_STD_MAX_MS)

    # airspeed is NOT groundspeed: (AP airspeed - gz groundspeed_x) must equal
    # -W_X. With a broken transport this is identically 0 in every case.
    obs = an["P2_wind_on_cruise"]["ap_airspeed_minus_gz_groundspeed_x_ms"]["mean"]
    R["p2_ap_airspeed_minus_groundspeed_x_ms"] = obs
    R["p2_expected_minus_wind_x_ms"] = -wind[0]
    if abs(wind[1]) < 1e-9 and abs(wind[2]) < 1e-9:
        # The one-dimensional identity airspeed = V_groundX - W_X is only exact
        # for a purely along-X airmass. It is therefore NOT evaluated for the
        # crosswind sanity case; that case is covered by the
        # heading-independent pitot-vs-ground-truth check in the matrix builder
        # instead. Skipping it is recorded, not silent.
        c["airspeed_is_not_groundspeed"] = (
            obs is not None and abs(obs - (-wind[0])) <= TH_MG_IDENTITY_TOL_MS)
    else:
        R["airspeed_is_not_groundspeed_skipped"] = (
            "1-D along-X identity not applicable to a lateral airmass; see "
            "pitot_minus_gz_predicted_airspeed_ms in the matrix builder")

    # ArduPlane's own AHRS wind estimate (independent of the airspeed path)
    wmag = math.sqrt(wind[0] ** 2 + wind[1] ** 2)
    apw = {k: an[k]["ap_wind_speed_ms"]["mean"] for k in wins}
    R["ap_wind_speed_by_window_ms"] = apw
    c["arduplane_sees_the_wind"] = all(
        apw[k] is not None
        and abs(apw[k] - (0.0 if k == "P0_zero_wind_ref_cruise" else wmag))
        <= TH_AP_WIND_TOL_MS for k in wins)

    R["acceptance_checks"] = c
    fails = [k for k, ok in c.items() if not ok]
    if not fails:
        return "AIRSPEED_WIND_ACCEPTANCE_PASS", fails
    return "AIRSPEED_WIND_ACCEPTANCE_FAILED", fails


# =============================================================================
# I/O
# =============================================================================
def out_paths(case):
    d = base.RESULTS_DIR
    return (os.path.join(d, f"airspeed_wind_acceptance_{case}_result.json"),
            os.path.join(d, f"airspeed_wind_acceptance_{case}_timeseries.json"))


def write_outputs(R, segs, case):
    oj, ot = out_paths(case)
    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    with open(ot, "w") as f:
        json.dump({"stage": STAGE, "case": case, "timestamp": R.get("timestamp"),
                   "params_live": R.get("params_live"),
                   "command_derivation": R.get("command_derivation"),
                   "segments": segs}, f, default=str, separators=(",", ":"))
    slim = dict(R)
    slim["segments_summary"] = {
        k: {kk: vv for kk, vv in v.items() if kk != "samples"} for k, v in segs.items()}
    slim["timeseries_file"] = ot
    with open(oj, "w") as f:
        json.dump(slim, f, indent=2, default=str)
    return oj, ot


def finish_fail(R, phase, mav, case, segs=None):
    R["overall_result"] = "TEST_FAILED"
    R["verdict"] = "AIRSPEED_WIND_ACCEPTANCE_FAILED"
    R["blocking_phase"] = phase
    oj, _ = write_outputs(R, segs or {}, case)
    print(f"FAILED at {phase} - see {oj}")
    if mav is not None:
        mav.close()
    return 1


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in WIND_CASES:
        print(f"usage: {sys.argv[0]} <{'|'.join(WIND_CASES)}>")
        return 2
    case = sys.argv[1]
    wind = WIND_CASES[case]

    R = {"stage": STAGE, "case": case,
         "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         "wind_world_enu_mps": list(wind),
         "wind_convention": "velocity of the AIR MASS, Gazebo world ENU frame, m/s "
                            "(docs/source_of_truth/environment/WIND.md sec 2). "
                            "Aircraft is teleported to yaw=0 => nose along world +X, "
                            "so -X is a HEADWIND and +X is a TAILWIND.",
         "thresholds": dict(
             TH_AP_VS_PITOT_MS=TH_AP_VS_PITOT_MS,
             TH_MG_DELTA_TOL_MS=TH_MG_DELTA_TOL_MS,
             TH_MG_IDENTITY_TOL_MS=TH_MG_IDENTITY_TOL_MS,
             TH_MG_GS_MISMATCH_MS=TH_MG_GS_MISMATCH_MS,
             TH_HOLD_MEAN_TOL_MS=TH_HOLD_MEAN_TOL_MS,
             TH_HOLD_STD_MAX_MS=TH_HOLD_STD_MAX_MS,
             TH_GS_SHIFT_TOL_MS=TH_GS_SHIFT_TOL_MS,
             TH_EAS2TAS_MAX=TH_EAS2TAS_MAX,
             TH_SAT_RUN_MAX_S=TH_SAT_RUN_MAX_S,
             TH_ARSP_USED_FRAC_MIN=TH_ARSP_USED_FRAC_MIN,
             TH_AP_WIND_TOL_MS=TH_AP_WIND_TOL_MS),
         "segment_plan": dict(SEG_REF_S=SEG_REF_S, SEG_CRUISE_S=SEG_CRUISE_S,
                              SEG_MG_S=SEG_MG_S, SETTLE_S=SETTLE_S,
                              mg_plan=MG_PLAN[case])}

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
    pitot = PitotSub()
    windout = WindOutSub()
    windcmd = WindCommander()
    time.sleep(1.0)
    R["pitot_topic_alive_at_start"] = pitot.count() > 0
    R["wind_topic_alive_at_start"] = windout.count() > 0
    print("pitot msgs:", pitot.count(), " wind msgs:", windout.count())

    mav, armed = base.phase1_mavlink_arm(R)
    if not armed:
        return finish_fail(R, "phase1_mavlink_arm", mav, case)

    p = tecs.dump_params(mav, R)
    R["params_live"] = p
    R["param_preconditions"] = tecs.param_precondition_checks(p, R)

    # --- per-test-only parameter change, recorded and later restored --------
    rnd_before = p.get("SIM_ARSPD_RND")
    rnd_after = set_param(mav, "SIM_ARSPD_RND", 0.0)
    R["sim_arspd_rnd"] = dict(
        before=rnd_before, set_to=0.0, readback=rnd_after,
        rationale="firmware default 2.0 is rectified by sqrt(|ratio*(q+noise)|) "
                  "(sitl_airspeed.cpp:37-39) into a positive airspeed bias; set to 0 "
                  "at TEST TIME ONLY over MAVLink. config/ardupilot/falcon_v2_sitl.parm "
                  "is NOT modified - that would perturb the validated TECS baseline.",
        applied=(rnd_after is not None and abs(rnd_after) < 1e-9))
    print("SIM_ARSPD_RND:", R["sim_arspd_rnd"])
    if not R["sim_arspd_rnd"]["applied"]:
        return finish_fail(R, "sim_arspd_rnd_not_applied", mav, case)

    if not base.is_armed(mav):
        base.arm(mav)
        if not base.is_armed(mav):
            return finish_fail(R, "rearm_after_param_dump", mav, case)

    required = ["AIRSPEED_MIN", "AIRSPEED_MAX", "RC3_MIN", "RC3_MAX", "RC3_DZ",
                "RC3_REVERSED", "RC2_TRIM"]
    if any(p.get(k) is None for k in required):
        R["missing_required_params"] = [k for k in required if p.get(k) is None]
        return finish_fail(R, "param_dump_incomplete", mav, case)

    settled, elapsed = base.wait_ground_settle(osub)
    R["ground_settle"] = dict(settled=settled, elapsed_s=elapsed)
    if not settled:
        base.disarm(mav)
        return finish_fail(R, "ground_settle", mav, case)

    # Launch is performed with ZERO wind in every case, so all three runs start
    # from an identical flight condition; the wind is switched on later.
    windcmd.set_steady(0.0, 0.0, 0.0)
    windcmd.tick()

    _, ok_v = base.phase2_teleport_and_verify(
        node, sub, osub, pub_oneshot, (0.0, 0.0, 90.0, 0.0, 0.0, 0.0), R)
    if not ok_v:
        base.disarm(mav)
        return finish_fail(R, "phase2_teleport_verify", mav, case)
    base.clear_wrench(pub_clear)

    if not base.phase3_hold_to_trim(pub_oneshot, pub_clear, sub, osub, mav, R):
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        base.disarm(mav)
        return finish_fail(R, "phase3_hold_to_trim", mav, case)

    ok, segs = flight_sequence(case, mav, sub, osub, adiag, pdiag, aerodiag,
                               pitot, windout, windcmd, p, R)

    windcmd.set_steady(0.0, 0.0, 0.0)
    windcmd.tick()
    mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    # restore the firmware default we temporarily overrode
    if rnd_before is not None:
        R["sim_arspd_rnd"]["restored_to"] = set_param(mav, "SIM_ARSPD_RND", rnd_before)
    base.disarm(mav)

    if ok:
        analyze(R, segs, case)
        vd, fails = verdict(R, case)
        R["verdict"] = vd
        R["failed_checks"] = fails
        R["overall_result"] = "FLIGHT_COMPLETED_NO_ABORT"
        an = R["analysis"]
        print("-" * 78)
        print(f"CASE {case}   wind(world ENU) = {wind}")
        for k in an:
            w = an[k]
            print(f"{k:26s} pitot={w['pitot_airspeed_mps']['mean']:.3f}  "
                  f"vfr_as={w['vfr_airspeed_mps']['mean']:.3f}  "
                  f"gs(gz)={w['gz_groundspeed_mps']['mean']:.3f}  "
                  f"gsX(gz)={w['gz_groundspeed_x_mps']['mean']:.3f}  "
                  f"vfr_gs={w['vfr_groundspeed_mps']['mean']:.3f}  "
                  f"thr={w['throttle']['mean']:.4f}  alt={w['altitude_m']['mean']:.2f}")
        print(f"VERDICT: {vd}  failed_checks={fails}")
        print("-" * 78)
    else:
        R["overall_result"] = "FLIGHT_ABORTED"
        R["verdict"] = "AIRSPEED_WIND_ACCEPTANCE_FAILED"
        print("FLIGHT ABORTED:", R.get("flight_result"))

    oj, ot = write_outputs(R, segs, case)
    print("RESULT:", R["overall_result"], R.get("verdict"), "->", oj)
    mav.close()
    return 0 if ok and not R.get("failed_checks") else (0 if ok else 1)


if __name__ == "__main__":
    sys.exit(main())

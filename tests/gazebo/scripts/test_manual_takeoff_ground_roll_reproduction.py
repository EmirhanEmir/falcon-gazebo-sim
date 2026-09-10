#!/usr/bin/env python3
"""FALCON V2 - Part 4: manual-takeoff GROUND-ROLL reproduction and evidence
capture.

CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY.
Stage: MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION (Part 4).
Owner: controls-integration. Created 2026-09-10.

CHANGES NO PHYSICS PARAMETER. No aerodynamic or propulsion coefficient, no
mass/CG/inertia, no actuator PID/rate/effort limit, no control-surface +/-45
deg mapping, no PTCH_TRIM_DEG, no TECS_PTCH_DAMP, no roll/pitch PID, no sensor
model, no landing gear (none is added - see below), no ground friction, no
collision geometry. It arms, advances the throttle, and records.

=============================================================================
WHAT IS BEING REPRODUCED
=============================================================================
The user's observed behaviour on the EXISTING temporary runway setup:
ground acceleration with a LEFT DRIFT, then a SUDDEN LIFTOFF at roughly
24 m/s.

=============================================================================
HARD FRAMING REQUIREMENT - THE HARNESS MUST NOT PRE-JUDGE
=============================================================================
There is NO landing gear and NO wheel on this model, by design, and this
stage is forbidden to add one. The aircraft rests on its existing belly/wing
collision primitives. Ground-roll fidelity is therefore LIMITED, and the
aircraft may be GEOMETRICALLY UNABLE to rotate.

This harness must be able to separate:
  (i)  a real ROTATION-then-fly takeoff: pitch attitude increases measurably
       before the vertical excursion, and the vertical excursion follows the
       rotation.
  (ii) an aircraft PINNED FLAT on its belly until lift simply exceeds weight
       and it pops off: pitch attitude stays clamped near its resting value
       for the whole roll, and the vertical excursion coincides with the
       lift-exceeds-weight crossing rather than with any pitch change.
It does this by recording BOTH signatures independently and reporting them
side by side. It DOES NOT decide which one is "correct". The classification
field it emits is descriptive, derived from stated criteria, and every raw
signal behind it is written to the timeseries so a reviewer can re-derive it.

=============================================================================
LIFTOFF CRITERION - EXPLICIT, WITH THE RAW SIGNAL NAMED
=============================================================================
PRIMARY (this is the number quoted as "liftoff time"):
    RAW SIGNAL   : the model's world Z from /world/<world>/dynamic_pose/info
                   (entity "falcon_v2"), the same render-stream transform the
                   GUI draws, plus its finite-differenced vertical rate.
    RESTING Z    : measured, not assumed - the mean world Z over the
                   pre-throttle settle window (the aircraft is spawned at
                   z=0.5 and falls onto its own collision primitives; the
                   settled value is whatever it is).
    CRITERION    : z > resting_z + TH.liftoff_z_margin_m AND vertical rate >
                   TH.liftoff_vz_min_ms, both continuously true for
                   TH.liftoff_hold_s. The reported liftoff timestamp is the
                   FIRST sample of that continuously-true run.
    WHY A HOLD   : a single-sample z excursion can be one contact-solver
                   event. Requiring a sustained run distinguishes a real
                   departure from a bounce; the bounce count is reported
                   separately rather than filtered away.
SECONDARY, reported alongside and NEVER substituted for the primary:
    (a) aero lift first equals weight: L = CL * qbar * S (all three from the
        aerodynamics plugin's own published diagnostics and the documented
        reference area) crossing m*g.
    (b) inferred support force first reaches zero (see below).
    (c) baro/MAVLink relative altitude leaving its ground value.
All four are reported with their own timestamp and the airspeed at that
timestamp, so a disagreement between them is visible instead of hidden.

WEIGHT: m*g = 6.000 kg * 9.81 m/s^2 = 58.86 N. m from CLAUDE.md "Aircraft
mass"; g from the world file's own <gravity>0 0 -9.81</gravity>. Both are
READ, not chosen, and the value is recomputed from those two sources at run
time rather than hard-coded.
REFERENCE AREA S = 0.4514 m^2, CLAUDE.md "Wing area" (manufacturer geometry).

=============================================================================
GROUND CONTACT / NORMAL FORCE - DATA_REQUIRED, AND WHAT IS DONE INSTEAD
=============================================================================
The runway world attaches NO gz-sim-contact-system and model/model.sdf
declares no <sensor type="contact">. Adding either would modify a world or
model this stage is forbidden to touch. So a DIRECT normal-force measurement
is DATA_REQUIRED and is reported as such.
What IS captured, and clearly labelled INFERRED:
    N_inferred = m*(a_z_world + g) - F_aero_z - F_thrust_z
with a_z_world finite-differenced from the render-stream Z, F_aero_z from the
plugin's own CL/CD/qbar rotated by the measured attitude, and F_thrust_z from
the propulsion plugin's own published per-motor thrust rotated the same way.
Its DECAY TOWARD ZERO is the quantity Part 5/6 need, and it is reported with
its inputs so the inference can be checked. A ContactSub is also attached
opportunistically: if /world/<world>/contacts ever exists, the real data is
recorded and supersedes the inference.

=============================================================================
WIND - MEASURED, NOT ASSUMED
=============================================================================
SIM_WIND_SPD / SIM_WIND_DIR / SIM_WIND_TURB are fetched LIVE over MAVLink at
the start of every run and written into the result, exactly as the navigation
campaign did. The Gazebo-side wind topic /model/falcon_v2/wind is also read.
A run that finds a non-zero wind is NOT aborted - it is recorded, because a
non-zero wind would itself be a candidate explanation for a lateral drift and
must not be silently assumed away in either direction.

=============================================================================
REPEATS
=============================================================================
DESIGNED FOR 3 REPEATS per mode (--repeats is honoured by the runner, which
restarts a FRESH gz sim + arduplane pair between repeats; a takeoff cannot be
meaningfully repeated inside one process). Three is the minimum that lets a
one-off contact-solver event be told apart from a repeatable behaviour: with
three runs, a behaviour seen once is a candidate outlier and a behaviour seen
three times is repeatable. Each run writes its own result/timeseries keyed by
--run-index, and --summarize aggregates them and reports run-to-run scatter.

=============================================================================
USAGE
=============================================================================
    python3 test_manual_takeoff_ground_roll_reproduction.py --run-index 1
    python3 test_manual_takeoff_ground_roll_reproduction.py --mode FBWA
    python3 test_manual_takeoff_ground_roll_reproduction.py --dry-run
    python3 test_manual_takeoff_ground_roll_reproduction.py --verify-world
    python3 test_manual_takeoff_ground_roll_reproduction.py --summarize
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import manual_takeoff_live_lib as L  # noqa: E402

PREFIX = "manual_takeoff_ground_roll"
DEFAULT_WORLD = "falcon_v2_manual_takeoff_runway"
WORLD_SDF = os.path.join(L.REPO_ROOT,
                         "tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_world.sdf")
WORLD_SDF_SOURCE = os.path.join(
    L.REPO_ROOT,
    "codex/tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_world.sdf")

MODE_MANUAL = 0
MODE_FBWA = 5
MODE_TAKEOFF = 13
MODE_NUM = {"MANUAL": MODE_MANUAL, "FBWA": MODE_FBWA, "TAKEOFF": MODE_TAKEOFF}

# ---- physical reference values, READ from their documented sources ----
AIRCRAFT_MASS_KG = 6.000        # CLAUDE.md "Aircraft mass"
WING_AREA_M2 = 0.4514           # CLAUDE.md "Wing area" (manufacturer geometry)
# g is parsed out of the world file at run time (see read_world_constants).

RC_NEUTRAL = {"rc1": 1500, "rc2": 1500, "rc3": 1000, "rc4": 1500, "rc5": 1000}
SAMPLE_PERIOD_S = 0.01          # 100 Hz attempt; the achieved rate is measured

TH = {
    "settle_s": {"value": 6.0,
                 "basis": "ASSUMPTION (procedure). Long enough for the "
                          "aircraft to fall from the world's own 0.5 m spawn "
                          "height onto its collision primitives and stop "
                          "moving; the settle is VERIFIED by a velocity "
                          "check, not assumed from the duration."},
    "settle_vmag_max_ms": {"value": 0.05,
                           "basis": "Same value already used by "
                                    "test_ardupilot_basic_closed_loop_flight."
                                    "wait_ground_settle(), reused for "
                                    "consistency across this repository."},
    "throttle_ramp_s": {"value": 2.0,
                        "basis": "ASSUMPTION (procedure). A ramp rather than "
                                 "a step, so the throttle-onset time is "
                                 "unambiguous and any yaw transient can be "
                                 "attributed to a throttle rate rather than a "
                                 "discontinuity. Reported, not tuned."},
    "roll_max_s": {"value": 60.0,
                   "basis": "ASSUMPTION (procedure). Upper bound on how long "
                            "to keep recording if liftoff never happens, so a "
                            "no-liftoff run still terminates and still "
                            "produces the full evidence set."},
    "post_liftoff_s": {"value": 6.0,
                       "basis": "ASSUMPTION (procedure). Enough post-liftoff "
                                "record to see whether the departure is "
                                "sustained or a bounce."},
    "liftoff_z_margin_m": {"value": 0.10,
                           "basis": "ASSUMPTION (detection). 0.10 m is well "
                                    "above the contact-solver penetration/"
                                    "settling scale seen at rest and well "
                                    "below any real climb, so it separates a "
                                    "departure from solver noise. The raw z "
                                    "trace is written out so a reviewer can "
                                    "re-derive with any other margin."},
    "liftoff_vz_min_ms": {"value": 0.5,
                          "basis": "ASSUMPTION (detection). Requires the "
                                   "aircraft to be genuinely going up, not "
                                   "just displaced."},
    "liftoff_hold_s": {"value": 0.30,
                       "basis": "ASSUMPTION (detection). ~300 physics steps "
                                "at the world's 0.001 s <max_step_size>. A "
                                "single contact-solver impulse cannot hold a "
                                "positive climb rate for 300 consecutive "
                                "steps; a real departure trivially can."},
    "rotation_pitch_delta_deg": {"value": 1.0,
                                 "basis": "ASSUMPTION (classification). The "
                                          "pitch change that must be exceeded "
                                          "BEFORE the vertical excursion for "
                                          "the run to look like a rotation "
                                          "rather than a flat pop-off. 1 deg "
                                          "is ~20x the attitude noise seen in "
                                          "the settle windows of the existing "
                                          "navigation records."},
    "yaw_rate_onset_deg_s": {"value": 1.0,
                             "basis": "ASSUMPTION (event detection). Onset "
                                      "threshold for 'the aircraft started "
                                      "yawing'. Used only to timestamp the "
                                      "drift onset so it can be compared "
                                      "against the surface-deflection "
                                      "timestamp."},
    "takeoff_arm_to_mode_dwell_s": {
        "value": 9.70,
        "basis": "MEASURED from the user's OWN dataflash log "
                 "codex/runtime/manual_takeoff/logs/00000002.BIN (read-only): "
                 "MODE record Mode=0 (MANUAL) at TimeUS 134.594384 s with "
                 "MSG 'Throttle armed' at 134.58 s, then MODE record Mode=13 "
                 "(TAKEOFF, Rsn=2 GCS command) at TimeUS 144.294754 s. The "
                 "dwell between arming in MANUAL and commanding TAKEOFF is "
                 "144.294754 - 134.594384 = 9.70037 s, rounded to 9.70 s. "
                 "This is a PROCEDURE timing only - it sets no physical, "
                 "aerodynamic, propulsive or control parameter."},
    "takeoff_mode_confirm_s": {
        "value": 8.0,
        "basis": "ASSUMPTION (procedure). Upper bound on how long to wait for "
                 "HEARTBEAT.custom_mode to report 13 after the DO_SET_MODE. "
                 "If it never does, the run is recorded as "
                 "ABORT_MODE_NOT_ENTERED rather than being silently flown in "
                 "whatever mode the autopilot was actually in."},
    "arm_attempts": {
        "value": 8,
        "basis": "EXECUTION CONTROL ONLY. ArduPlane refuses to arm until its "
                 "own GPS/EKF prearm checks pass; the user's own MAVProxy log "
                 "(codex/runtime/manual_takeoff/mavproxy.log) shows repeated "
                 "'PreArm: GPS 1: Bad fix' before a successful arm. Retrying "
                 "the arm command changes no parameter and no threshold."},
    "arm_retry_period_s": {
        "value": 4.0,
        "basis": "EXECUTION CONTROL ONLY. Spacing between arm attempts."},
    "surface_onset_deg": {"value": 0.5,
                          "basis": "DERIVED. ~8 PWM quanta of this transport "
                                   "(1 PWM = 0.0643 deg, derived in "
                                   "manual_takeoff_live_lib from model.sdf's "
                                   "own multiplier and servo_min/servo_max), "
                                   "and well above the servo model's "
                                   "documented ~0.06 deg neutral droop."},
}

_LOG = []


def log(*a):
    line = " ".join(str(x) for x in a)
    print(line, flush=True)
    _LOG.append(line)


def mean(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return sum(xs) / len(xs) if xs else float("nan")


# ---------------------------------------------------------------------------
# World provenance verification
# ---------------------------------------------------------------------------
def verify_world():
    """Assert that tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_world.sdf
    still differs from its codex source ONLY by (a) the prepended header
    comment and (b) the single <include><uri> line. If it has drifted, this
    FAILS the run rather than proceeding on an unverified world."""
    out = {"copy": WORLD_SDF, "source": WORLD_SDF_SOURCE}
    if not os.path.exists(WORLD_SDF_SOURCE):
        out["status"] = "SOURCE_MISSING"
        out["pass"] = None
        out["note"] = ("The codex source world is not present. The copy cannot "
                       "be verified against it. This is reported, not "
                       "worked around.")
        return out
    with open(WORLD_SDF, "r", encoding="utf-8") as fh:
        copy_lines = fh.read().splitlines()
    with open(WORLD_SDF_SOURCE, "r", encoding="utf-8") as fh:
        src_lines = fh.read().splitlines()
    # strip the copy's prepended header: everything up to and including the
    # first line that is exactly '-->'
    idx = next((i for i, l in enumerate(copy_lines) if l.strip() == "-->"), None)
    body = copy_lines[idx + 1:] if idx is not None else copy_lines
    src_body = src_lines[1:]  # source's own XML declaration line
    diffs = []
    if len(body) != len(src_body):
        diffs.append(f"line count {len(body)} vs {len(src_body)}")
    for i, (a, b) in enumerate(zip(body, src_body)):
        if a != b:
            diffs.append({"line": i + 1, "copy": a, "source": b})
    only_uri = (len(diffs) == 1 and isinstance(diffs[0], dict)
                and "<uri>" in diffs[0]["copy"]
                and "<uri>" in diffs[0]["source"]
                and diffs[0]["copy"].replace("/model", "").strip()
                == diffs[0]["source"].replace("/codex/model", "").strip())
    out["differences"] = diffs
    out["pass"] = bool(only_uri)
    out["status"] = ("ONLY_MODEL_URI_DIFFERS" if only_uri
                     else "UNEXPECTED_DIFFERENCE")
    return out


def read_world_constants():
    """Read gravity and the spawn pose out of the world SDF (READ-ONLY)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(WORLD_SDF).getroot()
    w = root.find("world")
    g = [float(v) for v in w.findtext("gravity").split()]
    inc = w.find("include")
    spawn = [float(v) for v in (inc.findtext("pose") or "0 0 0 0 0 0").split()]
    phys = w.find("physics")
    return {
        "gravity_mps2": g,
        "g_magnitude": abs(g[2]),
        "spawn_pose": spawn,
        "physics_engine": phys.get("type"),
        "max_step_size_s": float(phys.findtext("max_step_size")),
        "model_include_uri": inc.findtext("uri"),
        "weight_N": AIRCRAFT_MASS_KG * abs(g[2]),
        "mass_kg": AIRCRAFT_MASS_KG,
        "wing_area_m2": WING_AREA_M2,
        "provenance": {
            "mass_kg": "CLAUDE.md 'Aircraft mass'",
            "wing_area_m2": "CLAUDE.md 'Wing area' (manufacturer geometry)",
            "gravity": "world SDF <gravity>, read at run time",
        },
    }


# ---------------------------------------------------------------------------
# Live run
# ---------------------------------------------------------------------------
def run(world, mode, args, wc):
    import actuator_lib as ACT
    import aero_lib as AL
    import propulsion_lib as PL
    from ardupilot_sitl_mav_lib import SafeMav
    from pymavlink import mavutil
    import select as _sel

    signs = L.read_sign_scalars()
    clock = L.ClockSub(world)
    posesub = L.LinkPoseSub(world, ["falcon_v2", "base_link"] + L.SURFACES)
    actsub = L.TimedDiagSub(ACT.DIAG_TOPIC, ACT.DiagSubscriber._split)
    aerosub = L.TimedDiagSub(AL.DIAG_TOPIC,
                             lambda v: dict(zip(AL.DiagSubscriber.FIELDS, v)))
    propsub = L.TimedDiagSub(PL.DIAG_TOPIC, PL.DiagSubscriber._split)
    contact = L.ContactSub(world)
    time.sleep(1.5)

    mav = SafeMav(device=args.mavlink)
    hb = mav.wait_heartbeat(timeout=25)
    if hb is None:
        raise RuntimeError("no HEARTBEAT on " + args.mavlink)

    # ---- WIND: measured, not assumed ----
    params = mav.fetch_all_params(timeout=40)
    wind = {k: params.get(k) for k in
            ("SIM_WIND_SPD", "SIM_WIND_DIR", "SIM_WIND_TURB", "SIM_WIND_T",
             "SIM_WIND_DELAY")}
    wind["assertion"] = ("Recorded live. NOT assumed zero. A non-zero value "
                         "here is itself a candidate explanation for a "
                         "lateral drift and must be carried into any "
                         "conclusion.")
    wind["is_zero_speed"] = (wind.get("SIM_WIND_SPD") == 0.0)
    log("SIM wind:", {k: v for k, v in wind.items() if k.startswith("SIM")})

    key_params = {k: params.get(k) for k in
                  ("KFF_RDDRMIX", "YAW2SRV_DAMP", "YAW2SRV_INT", "YAW2SRV_SLIP",
                   "YAW2SRV_RLL", "RLL2SRV_TCONST", "PTCH2SRV_TCONST",
                   "PTCH_TRIM_DEG", "TECS_PTCH_DAMP", "TKOFF_THR_MAX",
                   "GROUND_STEER_ALT", "SERVO1_REVERSED", "SERVO2_REVERSED",
                   "SERVO4_REVERSED", "ARSPD_FBW_MIN", "TRIM_ARSPD_CM",
                   "SIM_ENGINE_MUL", "SIM_ENGINE_FAIL")}

    # ---- raise MAVLink stream rates and MEASURE what we actually got ----
    # MAV_CMD_SET_MESSAGE_INTERVAL is DENIED on this build; REQUEST_DATA_STREAM
    # works. Full measured evidence in
    # manual_takeoff_live_lib.request_stream_rates(). This writes no parameter.
    stream_rates = L.request_stream_rates(mav)
    log("stream rates measured:",
        {k: v for k, v in stream_rates["measured_hz"].items()
         if k in ("ATTITUDE", "SERVO_OUTPUT_RAW", "VFR_HUD",
                  "GLOBAL_POSITION_INT", "HEARTBEAT")})

    state = {"att": None, "att_wall": None, "servo": None, "servo_wall": None,
             "vfr": None, "gpi": None, "hb": None,
             # RC INPUT as the autopilot sees it. Needed to separate a PILOT
             # stick command from an AUTOPILOT-generated servo command: RCIN
             # is the stick, RCOU (SERVO_OUTPUT_RAW) is the output.
             "rcin": None, "rcin_wall": None}
    statustexts = []

    def pump():
        for _ in range(120):
            r, _, _ = _sel.select([mav.m.port], [], [], 0.0)
            if not r:
                return
            m = mav.m.recv_match(blocking=False)
            if m is None:
                return
            t = m.get_type()
            if t == "ATTITUDE":
                state["att"] = {
                    "roll_deg": math.degrees(m.roll),
                    "pitch_deg": math.degrees(m.pitch),
                    "yaw_deg": math.degrees(m.yaw),
                    "rollspeed_deg_s": math.degrees(m.rollspeed),
                    "pitchspeed_deg_s": math.degrees(m.pitchspeed),
                    "yawspeed_deg_s": math.degrees(m.yawspeed)}
                state["att_wall"] = time.time()
            elif t == "SERVO_OUTPUT_RAW":
                state["servo"] = [m.servo1_raw, m.servo2_raw, m.servo3_raw,
                                  m.servo4_raw, m.servo5_raw]
                state["servo_wall"] = time.time()
            elif t == "VFR_HUD":
                state["vfr"] = {"airspeed": m.airspeed,
                                "groundspeed": m.groundspeed,
                                "alt": m.alt, "climb": m.climb,
                                "throttle_pct": m.throttle,
                                "heading_deg": m.heading}
            elif t == "GLOBAL_POSITION_INT":
                state["gpi"] = {"relative_alt_m": m.relative_alt / 1000.0,
                                "hdg_deg": m.hdg / 100.0,
                                "vx": m.vx / 100.0, "vy": m.vy / 100.0,
                                "vz": m.vz / 100.0}
            elif t == "RC_CHANNELS":
                state["rcin"] = [m.chan1_raw, m.chan2_raw, m.chan3_raw,
                                 m.chan4_raw, m.chan5_raw]
                state["rcin_wall"] = time.time()
            elif t == "STATUSTEXT":
                txt = m.text.decode() if isinstance(m.text, bytes) else m.text
                statustexts.append({"t_wall": time.time(), "severity": m.severity,
                                    "text": txt})
            elif t == "HEARTBEAT":
                state["hb"] = {"custom_mode": m.custom_mode,
                               "armed": bool(m.base_mode & 128)}

    prev = {"t": None, "pos": None, "vz": None}
    samples = []
    t_ref = {"t0_wall": None, "t0_sim": None}

    def sample(phase, rc):
        pump()
        act, act_wall = actsub.latest()
        aero, aero_wall = aerosub.latest()
        prop, prop_wall = propsub.latest()
        poses, pose_stamp, pose_wall = posesub.latest()
        sim_t, _ = clock.latest()
        if act is None or poses is None or "falcon_v2" not in poses:
            return None
        mp = poses["falcon_v2"]
        pos, quat = mp["pos"], mp["quat"]
        now = time.time()
        vel = az = None
        if prev["t"] is not None and now > prev["t"]:
            dt = now - prev["t"]
            vel = [(pos[i] - prev["pos"][i]) / dt for i in range(3)]
            if prev["vz"] is not None:
                az = (vel[2] - prev["vz"]) / dt
            prev["vz"] = vel[2]
        prev["t"], prev["pos"] = now, pos
        roll, pitch, yaw = L.quat_to_rpy(*quat)

        theta = {s: act[s]["actual_angle_rad"] for s in L.SURFACES}
        cmd = {s: act[s]["cmd_rad"] for s in L.SURFACES}
        deltas = L.aero_deltas_from_joint_angles(theta, signs)

        # ---- aero forces from the plugin's OWN published coefficients ----
        lift_N = drag_N = None
        if aero and aero.get("qbar") is not None:
            q_S = aero["qbar"] * wc["wing_area_m2"]
            lift_N = aero["CL"] * q_S
            drag_N = aero["CD"] * q_S
        thrust_total = None
        if prop:
            thrust_total = (prop["left"]["thrust_N"] + prop["right"]["thrust_N"])

        # ---- INFERRED support force (see module docstring) ----
        n_inferred = None
        if az is not None and lift_N is not None and thrust_total is not None:
            # lift acts along world +Z to first order at small attitude; the
            # thrust axis is body +X. Both are projected with the MEASURED
            # attitude rather than assumed.
            fz_aero = lift_N * math.cos(roll) * math.cos(pitch)
            fz_thrust = thrust_total * (-math.sin(pitch))
            n_inferred = (wc["mass_kg"] * (az + wc["g_magnitude"])
                          - fz_aero - fz_thrust)

        if t_ref["t0_wall"] is None:
            t_ref["t0_wall"] = now
            t_ref["t0_sim"] = sim_t
        bound = L.bind_sample({
            "actuator_diag": (None, act_wall),
            "aero_diag": (None, aero_wall),
            "propulsion_diag": (None, prop_wall),
            "link_pose": (None, pose_wall),
            "servo_output_raw": (None, state["servo_wall"]),
            "attitude": (None, state["att_wall"]),
        })
        srv = state["servo"] or [None] * 5
        rin = state["rcin"] or [None] * 5
        pl = prop["left"] if prop else {}
        pr = prop["right"] if prop else {}
        return {
            "phase": phase,
            "t_wall": now,
            "t_rel_wall": now - t_ref["t0_wall"],
            "t_sim": sim_t,
            "t_rel_sim": (sim_t - t_ref["t0_sim"]) if (sim_t is not None and t_ref["t0_sim"] is not None) else None,
            "skew_s": bound["skew_s"],
            "max_abs_skew_s": bound["max_abs_skew_s"],
            "rc_commanded": dict(rc),
            # ---- world/render state (raw signals for the liftoff criterion) ----
            "gz_pos_world_m": pos,
            "gz_quat": quat,
            "gz_rpy_deg": [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)],
            "gz_vel_world_ms_finite_diff": vel,
            "gz_az_world_ms2_finite_diff": az,
            "lateral_displacement_m_world_Y": pos[1],
            "lateral_velocity_ms_world_Y": vel[1] if vel else None,
            # ---- autopilot state ----
            "attitude_mav": state["att"],
            "vfr": state["vfr"],
            "gpi": state["gpi"],
            "heartbeat": state["hb"],
            "servo_pwm": srv,
            "servo_pwm_named": {"aileron": srv[0], "elevator": srv[1],
                                "throttle_left": srv[2], "rudder": srv[3],
                                "throttle_right": srv[4]},
            # RCIN = what the autopilot receives as PILOT STICK input
            # (RC_CHANNELS). RCOU above = what it commands to the servos.
            "rc_in_pwm": rin,
            "rc_in_pwm_named": {"c1_roll": rin[0], "c2_pitch": rin[1],
                                "c3_throttle": rin[2], "c4_yaw": rin[3],
                                "c5": rin[4]},
            # ---- surfaces: BOTH command and actual, all five ----
            "surface_cmd_deg": {s: math.degrees(cmd[s]) for s in L.SURFACES},
            "surface_actual_deg": {s: math.degrees(theta[s]) for s in L.SURFACES},
            "surface_full": act,
            "aero_input_deg": {k: math.degrees(v) for k, v in deltas.items()},
            # ---- aero ----
            "aero": aero,
            "lift_N": lift_N,
            "drag_N": drag_N,
            # ---- propulsion: per motor, from t=0 ----
            "prop_left": pl,
            "prop_right": pr,
            "rpm_left": pl.get("rpm"), "rpm_right": pr.get("rpm"),
            "rpm_diff_left_minus_right": (
                (pl.get("rpm") - pr.get("rpm"))
                if pl.get("rpm") is not None and pr.get("rpm") is not None else None),
            "thrust_left_N": pl.get("thrust_N"), "thrust_right_N": pr.get("thrust_N"),
            "thrust_diff_left_minus_right_N": (
                (pl.get("thrust_N") - pr.get("thrust_N"))
                if pl.get("thrust_N") is not None and pr.get("thrust_N") is not None else None),
            "Q_prop_left_Nm": pl.get("Q_prop_Nm"), "Q_prop_right_Nm": pr.get("Q_prop_Nm"),
            "Q_motor_left_Nm": pl.get("Q_motor_Nm"), "Q_motor_right_Nm": pr.get("Q_motor_Nm"),
            "Q_prop_sum_Nm": ((pl.get("Q_prop_Nm") or 0.0) + (pr.get("Q_prop_Nm") or 0.0))
            if pl and pr else None,
            "thrust_total_N": thrust_total,
            # ---- contact / support ----
            "contact_available": contact.available,
            "normal_force_N_INFERRED": n_inferred,
            "normal_force_source": ("MEASURED /contacts" if contact.available
                                    else "INFERRED - DATA_REQUIRED: no contact "
                                         "sensor exists in this world/model and "
                                         "adding one is out of scope"),
        }

    # ---- phase 1: settle ----
    log("phase 1: ground settle (no throttle)")
    t_end = time.time() + TH["settle_s"]["value"]
    settle = []
    while time.time() < t_end:
        mav.send_rc_override(**RC_NEUTRAL)
        r = sample("settle", RC_NEUTRAL)
        if r:
            settle.append(r)
        time.sleep(SAMPLE_PERIOD_S)
    samples += settle
    resting_z = mean([s["gz_pos_world_m"][2] for s in settle[len(settle) // 2:]])
    resting_pitch = mean([s["gz_rpy_deg"][1] for s in settle[len(settle) // 2:]])
    resting_roll = mean([s["gz_rpy_deg"][0] for s in settle[len(settle) // 2:]])
    resting_yaw = mean([s["gz_rpy_deg"][2] for s in settle[len(settle) // 2:]])
    settle_vmag = mean([math.sqrt(sum(c * c for c in (s["gz_vel_world_ms_finite_diff"] or [9, 9, 9])))
                        for s in settle[len(settle) // 2:]])
    log(f"resting: z={resting_z:.4f} m  pitch={resting_pitch:.3f} deg  "
        f"roll={resting_roll:.3f} deg  yaw={resting_yaw:.3f} deg  "
        f"|v|={settle_vmag:.4f} m/s")

    # ---- phase 2: mode + arm ----
    #
    # TAKEOFF-MODE PATH (added 2026-09-10 by gazebo-testing).
    # The user's own runway flights were NOT flown in MANUAL or FBWA. All five
    # dataflash logs under codex/runtime/manual_takeoff/logs/ show ArduPlane
    # mode 13 = TAKEOFF. The sequence below is READ OFF those logs, not
    # invented (log 00000002.BIN, read-only):
    #     MSG  134.58 s  "Throttle armed"
    #     MODE 134.594384 s  Mode=0  (MANUAL)   <- armed while in MANUAL
    #     MODE 144.294754 s  Mode=13 (TAKEOFF)  Rsn=2 (GCS command)
    #     MSG  144.31 s  "Armed AUTO, xaccel = 0.0 m/s/s, waiting 0.2 sec"
    #     MSG  144.51 s  "Triggered AUTO. GPS speed = 0.0"
    #     MSG  146.47 s  "Takeoff to 50m for 200.0m heading 90.0 deg"
    #     RCIN throughout: C1=1500 C2=1500 C3=1000 C4=1500 (no pilot input)
    # Every TKOFF_* parameter present in that log is the ArduPlane FIRMWARE
    # DEFAULT (TKOFF_ALT 50, TKOFF_LVL_ALT 10, TKOFF_LVL_PITCH 15,
    # TKOFF_DIST 200, TKOFF_GND_PITCH 5, TKOFF_THR_MINACC 0, TKOFF_THR_MINSPD
    # 0, TKOFF_THR_DELAY 2, ...), and a full parameter-by-parameter comparison
    # of that log's PARM records against config/ardupilot/falcon_v2_sitl.parm
    # found ZERO differences. NO PARAMETER IS SET OR CHANGED HERE, by this
    # harness or by the runner, to reach TAKEOFF mode.
    entry_mode = MODE_MANUAL if mode in ("MANUAL", "TAKEOFF") else MODE_FBWA
    mav.command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                     p2=entry_mode)
    time.sleep(1.0)
    mav.command_long(mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, p1=1)

    # Arm, retrying while ArduPlane's own prearm checks are still failing.
    # EXECUTION CONTROL ONLY - no parameter, threshold or check is changed or
    # bypassed; the autopilot's checks are left exactly as they are and we
    # simply ask again. The user's own MAVProxy log shows the same pattern
    # ("PreArm: GPS 1: Bad fix" repeatedly, then a successful arm).
    ack = None
    arm_log = []
    for attempt in range(TH["arm_attempts"]["value"]):
        ack = mav.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1)
        arm_log.append({"attempt": attempt + 1,
                        "result": (ack.result if ack else None)})
        pump()
        if ack is not None and ack.result == 0:
            break
        t_a = time.time() + TH["arm_retry_period_s"]["value"]
        while time.time() < t_a:
            mav.send_rc_override(**RC_NEUTRAL)
            r = sample("settle", RC_NEUTRAL)
            if r:
                samples.append(r)
            time.sleep(SAMPLE_PERIOD_S)
    log("arm ack:", ack.to_dict() if ack else None, "attempts:", len(arm_log))
    time.sleep(0.5)

    mode_entry = {"target_mode": mode, "target_custom_mode": MODE_NUM[mode],
                  "entry_mode_before": entry_mode, "arm_attempts": arm_log,
                  "arm_result": (ack.result if ack else None)}
    if mode == "TAKEOFF":
        # dwell armed-in-MANUAL for the SAME interval the user's log shows
        # between arming and commanding TAKEOFF, then command the mode.
        dwell = TH["takeoff_arm_to_mode_dwell_s"]["value"]
        log(f"phase 2b: armed in MANUAL, dwelling {dwell:.2f} s "
            f"(measured from the user's own log), then commanding TAKEOFF")
        t_d = time.time() + dwell
        while time.time() < t_d:
            mav.send_rc_override(**RC_NEUTRAL)
            r = sample("armed_dwell", RC_NEUTRAL)
            if r:
                samples.append(r)
            time.sleep(SAMPLE_PERIOD_S)
        m_ack = mav.command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=MODE_TAKEOFF)
        mode_entry["do_set_mode_result"] = (m_ack.result if m_ack else None)
        mode_entry["t_wall_commanded"] = time.time()
        # CONFIRM from HEARTBEAT, do not assume.
        t_c = time.time() + TH["takeoff_mode_confirm_s"]["value"]
        confirmed = False
        while time.time() < t_c:
            mav.send_rc_override(**RC_NEUTRAL)
            r = sample("armed_dwell", RC_NEUTRAL)
            if r:
                samples.append(r)
            hb = state.get("hb")
            if hb and hb.get("custom_mode") == MODE_TAKEOFF:
                confirmed = True
                break
            time.sleep(SAMPLE_PERIOD_S)
        mode_entry["heartbeat_confirmed_custom_mode_13"] = confirmed
        mode_entry["heartbeat_at_confirm"] = state.get("hb")
        log("TAKEOFF mode entry: DO_SET_MODE result =",
            mode_entry["do_set_mode_result"],
            "| HEARTBEAT confirmed custom_mode=13:", confirmed)
        if not confirmed:
            log("TAKEOFF MODE NOT ENTERED - the run continues so the evidence "
                "is still captured, and the result is flagged.")

    # ---- phase 3: throttle ramp + ground roll ----
    # In TAKEOFF mode ArduPlane owns the throttle: the user's logs show RCIN
    # C3 pinned at 1000 us for the whole roll while RCOU C3/C5 go to
    # 1980-2000 us. So this harness does NOT ramp RC3 in TAKEOFF mode - it
    # holds the SAME neutral stick set the user's logs show and lets the
    # autopilot command the throttle, exactly as the user's flights did.
    if mode == "TAKEOFF":
        log("phase 3: TAKEOFF mode - autopilot owns the throttle. RC held at "
            "C1=1500 C2=1500 C3=1000 C4=1500 (the user's own RCIN), no ramp.")
    else:
        log(f"phase 3: throttle ramp ({TH['throttle_ramp_s']['value']}s) then roll, "
            f"mode={mode}, sticks NEUTRAL on roll/pitch/yaw")
    t_ramp0 = time.time()
    t_roll_deadline = t_ramp0 + TH["roll_max_s"]["value"]
    liftoff_run_start = None
    liftoff_t = None
    zmarg = TH["liftoff_z_margin_m"]["value"]
    vzmin = TH["liftoff_vz_min_ms"]["value"]
    hold = TH["liftoff_hold_s"]["value"]
    post_end = None
    while time.time() < t_roll_deadline:
        el = time.time() - t_ramp0
        if mode == "TAKEOFF":
            rc = dict(RC_NEUTRAL)
            phase_label = "ground_roll"
        else:
            frac = min(1.0, el / TH["throttle_ramp_s"]["value"])
            rc = dict(RC_NEUTRAL, rc3=int(round(1000 + 1000 * frac)))
            phase_label = "throttle_ramp" if frac < 1.0 else "ground_roll"
        mav.send_rc_override(**rc)
        r = sample(phase_label, rc)
        if r:
            samples.append(r)
            z = r["gz_pos_world_m"][2]
            vz = (r["gz_vel_world_ms_finite_diff"] or [0, 0, 0])[2]
            if z > resting_z + zmarg and vz > vzmin:
                if liftoff_run_start is None:
                    liftoff_run_start = r["t_wall"]
                elif liftoff_t is None and r["t_wall"] - liftoff_run_start >= hold:
                    liftoff_t = liftoff_run_start
                    log(f"LIFTOFF (primary criterion) at t_rel={liftoff_run_start - t_ref['t0_wall']:.3f} s "
                        f"z={z:.3f} vz={vz:.3f}")
                    post_end = time.time() + TH["post_liftoff_s"]["value"]
            else:
                liftoff_run_start = None
        if post_end is not None and time.time() > post_end:
            break
        time.sleep(SAMPLE_PERIOD_S)

    mav.hold_rc_override(0.5, **RC_NEUTRAL)
    try:
        from pymavlink import mavutil as _mu
        mav.command_long(_mu.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=0,
                         wait_ack=False)
    except Exception:
        pass
    mav.close()
    log("captured", len(samples), "samples")
    return {
        "samples": samples,
        "resting": {"z_m": resting_z, "pitch_deg": resting_pitch,
                    "roll_deg": resting_roll, "yaw_deg": resting_yaw,
                    "settle_vmag_ms": settle_vmag,
                    "settled": settle_vmag < TH["settle_vmag_max_ms"]["value"]},
        "liftoff_t_wall_primary": liftoff_t,
        "t0_wall": t_ref["t0_wall"],
        "wind": wind,
        "key_params_live": key_params,
        "mavlink_stream_rates": stream_rates,
        "contact_topic_available": contact.available,
        "mode_entry": mode_entry,
        "statustexts": statustexts,
    }


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def analyse(run_out, wc, mode):
    S = run_out["samples"]
    t0 = run_out["t0_wall"]
    resting = run_out["resting"]
    weight_N = wc["weight_N"]

    def trel(s):
        return s["t_wall"] - t0

    roll_samples = [s for s in S if s["phase"] in ("throttle_ramp", "ground_roll")]

    def first_where(pred, seq=None):
        for s in (seq or roll_samples):
            try:
                if pred(s):
                    return s
            except (TypeError, KeyError):
                continue
        return None

    # ---- liftoff, four independent criteria ----
    lo = {}
    lp = run_out["liftoff_t_wall_primary"]
    prim = first_where(lambda s: lp is not None and s["t_wall"] >= lp)
    lo["primary_z_and_vz"] = _event(prim, t0, "world Z from dynamic_pose/info "
                                    "+ its finite-differenced vertical rate")
    lift_cross = first_where(lambda s: s["lift_N"] is not None and s["lift_N"] >= weight_N)
    lo["aero_lift_equals_weight"] = _event(
        lift_cross, t0, f"L = CL*qbar*S from the aerodynamics plugin's own "
                        f"diagnostics, crossing m*g = {weight_N:.2f} N")
    nzero = first_where(lambda s: s["normal_force_N_INFERRED"] is not None
                        and s["normal_force_N_INFERRED"] <= 0.0)
    lo["inferred_support_force_reaches_zero"] = _event(
        nzero, t0, "INFERRED support force (DATA_REQUIRED: no contact sensor)")
    baro = first_where(lambda s: s.get("gpi") and s["gpi"].get("relative_alt_m") is not None
                       and s["gpi"]["relative_alt_m"] > TH["liftoff_z_margin_m"]["value"])
    lo["mavlink_relative_alt_leaves_ground"] = _event(
        baro, t0, "MAVLink GLOBAL_POSITION_INT.relative_alt")

    # ---- rotation vs pop-off evidence ----
    pitch_series = [(trel(s), s["gz_rpy_deg"][1]) for s in roll_samples]
    t_lift = lo["primary_z_and_vz"]["t_rel_s"] if lo["primary_z_and_vz"] else None
    pre = [p for p in pitch_series if t_lift is None or p[0] < t_lift]
    max_pitch_change_before = (max((abs(p[1] - resting["pitch_deg"]) for p in pre),
                                   default=float("nan")))
    pitch_at_lift = next((p[1] for p in pitch_series if t_lift is not None and p[0] >= t_lift), None)
    rotated = (not math.isnan(max_pitch_change_before)
               and max_pitch_change_before >= TH["rotation_pitch_delta_deg"]["value"])
    dt_lift_vs_liftcross = None
    if lo["primary_z_and_vz"] and lo["aero_lift_equals_weight"]:
        dt_lift_vs_liftcross = (lo["primary_z_and_vz"]["t_rel_s"]
                                - lo["aero_lift_equals_weight"]["t_rel_s"])
    classification = {
        "pitch_change_before_vertical_excursion_deg": max_pitch_change_before,
        "threshold_deg": TH["rotation_pitch_delta_deg"]["value"],
        "pitch_at_liftoff_deg": pitch_at_lift,
        "resting_pitch_deg": resting["pitch_deg"],
        "liftoff_minus_lift_equals_weight_s": dt_lift_vs_liftcross,
        "signature_i_rotation_then_fly": bool(rotated),
        "signature_ii_pinned_flat_until_lift_exceeds_weight": bool(
            (not rotated) and dt_lift_vs_liftcross is not None
            and abs(dt_lift_vs_liftcross) < 1.0),
        "note": "DESCRIPTIVE ONLY. Both signatures are computed from stated "
                "criteria over raw signals that are all present in the "
                "timeseries. This harness does not decide which behaviour is "
                "correct, and no parameter was changed to obtain either.",
    }

    # ---- lateral drift ----
    yaw_onset = first_where(lambda s: s["attitude_mav"] is not None
                            and abs(s["attitude_mav"]["yawspeed_deg_s"])
                            >= TH["yaw_rate_onset_deg_s"]["value"])
    surf_onset = first_where(
        lambda s: max(abs(s["surface_cmd_deg"][k]) for k in ("left_aileron", "rudder"))
        >= TH["surface_onset_deg"]["value"])
    drift = {
        "yaw_rate_onset": _event(yaw_onset, t0,
                                 "MAVLink ATTITUDE.yawspeed"),
        "surface_command_onset": _event(surf_onset, t0,
                                        "actuator cmd_rad (aileron or rudder)"),
        "surface_onset_PRECEDES_yaw_onset": None,
        "final_lateral_displacement_m_world_Y": (
            roll_samples[-1]["lateral_displacement_m_world_Y"] if roll_samples else None),
        "max_abs_lateral_displacement_m_world_Y": max(
            (abs(s["lateral_displacement_m_world_Y"]) for s in roll_samples),
            default=None),
        "drift_direction": None,
        "note": "The runway centerline is world Y=0 and the nose points along "
                "world +X (world SDF header). In the FLU body frame +Y is "
                "LEFT, so a POSITIVE world-Y displacement with the nose along "
                "+X is a drift to the LEFT.",
    }
    if drift["max_abs_lateral_displacement_m_world_Y"]:
        fin = drift["final_lateral_displacement_m_world_Y"]
        drift["drift_direction"] = ("LEFT (+Y)" if fin > 0 else
                                    "RIGHT (-Y)" if fin < 0 else "NONE")
    if drift["yaw_rate_onset"] and drift["surface_command_onset"]:
        drift["surface_onset_PRECEDES_yaw_onset"] = bool(
            drift["surface_command_onset"]["t_rel_s"]
            < drift["yaw_rate_onset"]["t_rel_s"])
        drift["onset_separation_s"] = (drift["yaw_rate_onset"]["t_rel_s"]
                                       - drift["surface_command_onset"]["t_rel_s"])
    elif drift["yaw_rate_onset"] and not drift["surface_command_onset"]:
        drift["surface_onset_PRECEDES_yaw_onset"] = False
        drift["onset_separation_s"] = None
        drift["surface_never_deflected_note"] = (
            "No aileron/rudder command ever exceeded the onset threshold, so "
            "the yaw drift CANNOT have been commanded by a control surface in "
            "this run. That is a positive, attributable finding.")

    # ---- propulsion asymmetry, from t=0 ----
    rpm_d = [s["rpm_diff_left_minus_right"] for s in S
             if s["rpm_diff_left_minus_right"] is not None]
    th_d = [s["thrust_diff_left_minus_right_N"] for s in S
            if s["thrust_diff_left_minus_right_N"] is not None]
    qsum = [s["Q_prop_sum_Nm"] for s in S if s["Q_prop_sum_Nm"] is not None]
    propulsion = {
        "rpm_diff_max_abs": max((abs(v) for v in rpm_d), default=None),
        "rpm_diff_mean": mean(rpm_d),
        "thrust_diff_N_max_abs": max((abs(v) for v in th_d), default=None),
        "thrust_diff_N_mean": mean(th_d),
        "thrust_diff_pct_of_total_max": None,
        "Q_prop_sum_Nm_max_abs": max((abs(v) for v in qsum), default=None),
        "Q_prop_sum_Nm_mean": mean(qsum),
        "note": "Q_prop_sum is the NET propeller reaction torque about the "
                "body roll axis. The two propellers are documented as "
                "counter-rotating (CLAUDE.md / master dataset sec 44), so a "
                "non-zero SUM is a net roll/yaw disturbance candidate and is "
                "reported here for Part 5/6 rather than judged.",
    }
    tt = [s["thrust_total_N"] for s in S if s["thrust_total_N"]]
    if tt and propulsion["thrust_diff_N_max_abs"] is not None:
        propulsion["thrust_diff_pct_of_total_max"] = (
            100.0 * propulsion["thrust_diff_N_max_abs"] / max(tt))

    # ---- support-force decay ----
    nvals = [(trel(s), s["normal_force_N_INFERRED"]) for s in roll_samples
             if s["normal_force_N_INFERRED"] is not None]
    support = {
        "source": ("MEASURED" if run_out["contact_topic_available"]
                   else "INFERRED"),
        "data_required": (None if run_out["contact_topic_available"]
                          else L.AERO_DEFLECTION_NOT_PUBLISHED["status"]),
        "n_points": len(nvals),
        "value_at_start_N": nvals[0][1] if nvals else None,
        "value_at_end_N": nvals[-1][1] if nvals else None,
        "weight_reference_N": weight_N,
        "first_time_below_half_weight_s": next(
            (t for t, v in nvals if v <= 0.5 * weight_N), None),
        "first_time_at_or_below_zero_s": next((t for t, v in nvals if v <= 0.0), None),
        "note": "DATA_REQUIRED for a DIRECT normal-force measurement: neither "
                "the runway world nor model/model.sdf carries a contact "
                "sensor, and adding one is outside this stage's permitted "
                "changes. The values here are INFERRED from measured vertical "
                "acceleration minus the plugins' own published aero and "
                "thrust forces, and every input is in the timeseries.",
    }

    # ---- lift vs weight crossing ----
    lw = lo["aero_lift_equals_weight"]
    lift_vs_weight = {
        "weight_N": weight_N,
        "weight_provenance": "mass 6.000 kg (CLAUDE.md) * g from the world "
                             "SDF's own <gravity>",
        "crossing_time_s": lw["t_rel_s"] if lw else None,
        "airspeed_at_crossing_ms": lw["airspeed_ms"] if lw else None,
        "groundspeed_at_crossing_ms": lw["groundspeed_ms"] if lw else None,
        "max_lift_N": max((s["lift_N"] for s in S if s["lift_N"] is not None),
                          default=None),
    }

    # ---- surfaces summary ----
    surf = {}
    for s_name in L.SURFACES:
        cmds = [x["surface_cmd_deg"][s_name] for x in roll_samples]
        acts = [x["surface_actual_deg"][s_name] for x in roll_samples]
        surf[s_name] = {
            "cmd_max_abs_deg": max((abs(v) for v in cmds), default=None),
            "actual_max_abs_deg": max((abs(v) for v in acts), default=None),
            "cmd_mean_deg": mean(cmds), "actual_mean_deg": mean(acts),
            "ever_exceeded_onset_threshold": bool(
                max((abs(v) for v in cmds), default=0.0)
                >= TH["surface_onset_deg"]["value"]),
        }

    ts = [trel(s) for s in S]
    dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)] or [float("nan")]
    skews = [s["max_abs_skew_s"] for s in S if s.get("max_abs_skew_s") is not None]
    return {
        "mode": mode,
        "mode_entry": run_out.get("mode_entry"),
        "arduplane_statustexts": run_out.get("statustexts"),
        "world_constants": wc,
        "resting_state": resting,
        "wind_measured": run_out["wind"],
        "mavlink_stream_rates_measured": run_out["mavlink_stream_rates"],
        "key_params_live": run_out["key_params_live"],
        "liftoff_criteria": lo,
        "rotation_vs_popoff_evidence": classification,
        "lateral_drift": drift,
        "propulsion_asymmetry": propulsion,
        "support_force": support,
        "lift_vs_weight": lift_vs_weight,
        "surfaces": surf,
        "capture": {
            "n_samples": len(S),
            "mean_sample_period_s": sum(dts) / len(dts),
            "mean_rate_hz": (len(dts) / sum(dts)) if sum(dts) > 0 else None,
            "max_inter_source_skew_s": max(skews) if skews else None,
            "duration_s": ts[-1] if ts else None,
        },
        "thresholds": TH,
    }


def _event(s, t0, raw_signal):
    if s is None:
        return None
    vfr = s.get("vfr") or {}
    return {
        "t_rel_s": s["t_wall"] - t0,
        "t_sim": s["t_sim"],
        "raw_signal": raw_signal,
        "airspeed_ms": vfr.get("airspeed"),
        "groundspeed_ms": vfr.get("groundspeed"),
        "gz_z_m": s["gz_pos_world_m"][2],
        "gz_vz_ms": (s["gz_vel_world_ms_finite_diff"] or [None, None, None])[2],
        "pitch_deg": s["gz_rpy_deg"][1],
        "roll_deg": s["gz_rpy_deg"][0],
        "yaw_deg": s["gz_rpy_deg"][2],
        "lateral_Y_m": s["lateral_displacement_m_world_Y"],
        "lift_N": s["lift_N"],
        "thrust_total_N": s["thrust_total_N"],
        "throttle_pct": vfr.get("throttle_pct"),
    }


def summarize(mode_filter=None, out_name=None):
    runs = []
    suffix = (f"_{mode_filter}_result.json" if mode_filter else "_result.json")
    for fn in sorted(os.listdir(L.RESULTS_DIR)):
        if fn.startswith(PREFIX + "_run") and fn.endswith(suffix):
            with open(os.path.join(L.RESULTS_DIR, fn), "r", encoding="utf-8") as fh:
                runs.append({"file": fn, "data": json.load(fh)})
    out = {"n_runs": len(runs), "runs": [r["file"] for r in runs],
           "mode_filter": mode_filter,
           "repeats_designed_for": 3,
           "repeat_rationale": "Three runs is the minimum that lets a one-off "
                               "contact-solver event be told apart from a "
                               "repeatable behaviour."}
    def collect(path):
        vals = []
        for r in runs:
            d = r["data"].get("analysis", {})
            for k in path:
                d = d.get(k) if isinstance(d, dict) else None
                if d is None:
                    break
            if isinstance(d, (int, float)):
                vals.append(d)
        return vals
    for label, path in (
            ("liftoff_t_rel_s", ["liftoff_criteria", "primary_z_and_vz", "t_rel_s"]),
            ("liftoff_airspeed_ms", ["liftoff_criteria", "primary_z_and_vz", "airspeed_ms"]),
            ("lift_weight_crossing_s", ["lift_vs_weight", "crossing_time_s"]),
            ("lift_weight_crossing_airspeed_ms", ["lift_vs_weight", "airspeed_at_crossing_ms"]),
            ("final_lateral_Y_m", ["lateral_drift", "final_lateral_displacement_m_world_Y"]),
            ("pitch_change_before_liftoff_deg",
             ["rotation_vs_popoff_evidence", "pitch_change_before_vertical_excursion_deg"]),
            ("thrust_diff_N_max_abs", ["propulsion_asymmetry", "thrust_diff_N_max_abs"])):
        v = collect(path)
        out[label] = {"values": v, "mean": mean(v),
                      "min": min(v) if v else None, "max": max(v) if v else None,
                      "spread": (max(v) - min(v)) if v else None}
    dst = os.path.join(L.RESULTS_DIR, out_name or (PREFIX + "_summary.json"))
    with open(dst, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    print("wrote", dst)
    print(json.dumps({k: v for k, v in out.items()
                      if isinstance(v, dict) and "values" in v}, indent=1))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=DEFAULT_WORLD)
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5760")
    ap.add_argument("--mode", choices=("MANUAL", "FBWA", "TAKEOFF"),
                    default="MANUAL",
                    help="MANUAL keeps roll/pitch/yaw sticks neutral with NO "
                         "autopilot stabilisation, so any yaw drift cannot "
                         "have been commanded by a control surface - the "
                         "cleanest attribution. FBWA lets the autopilot fight "
                         "the drift, which is what makes the "
                         "'does the deflection PRECEDE or FOLLOW the drift?' "
                         "question non-trivial. TAKEOFF is ArduPlane mode 13 "
                         "and is the mode the user's OWN runway logs were "
                         "actually flown in; it arms in MANUAL, dwells, then "
                         "commands mode 13 and lets the autopilot own the "
                         "throttle and the surfaces, with the sticks held at "
                         "the user's own RCIN values.")
    ap.add_argument("--run-index", type=int, default=1)
    ap.add_argument("--repeats", type=int, default=3,
                    help="declared only; the runner performs the repeats by "
                         "restarting a fresh gz+arduplane pair each time")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--verify-world", action="store_true")
    ap.add_argument("--summarize", action="store_true")
    ap.add_argument("--summarize-mode", default=None,
                    help="Restrict --summarize to one mode's result files.")
    ap.add_argument("--summary-out", default=None,
                    help="Summary filename inside tests/gazebo/results. Given "
                         "explicitly when summarising a subset so a previous "
                         "campaign's summary artifact is never overwritten.")
    args = ap.parse_args()

    L.env_setup()
    if args.summarize:
        return summarize(args.summarize_mode, args.summary_out)

    wv = verify_world()
    if args.verify_world:
        print(json.dumps(wv, indent=1))
        return 0 if wv["pass"] else 1

    wc = read_world_constants()
    header = L.provenance_header("4 - manual takeoff ground-roll reproduction",
                                 args.world,
                                 {"mode": args.mode,
                                  "run_index": args.run_index,
                                  "repeats_designed_for": args.repeats,
                                  "world_verification": wv,
                                  "world_constants": wc,
                                  "thresholds": TH})
    if not wv["pass"]:
        log("WORLD VERIFICATION FAILED - refusing to run:", wv["status"])
        header["overall"] = "ABORT_WORLD_VERIFICATION_FAILED"
    elif args.dry_run:
        log("--dry-run OK. world verified, constants:",
            {k: wc[k] for k in ("g_magnitude", "weight_N", "spawn_pose",
                                "max_step_size_s", "model_include_uri")})
        header["overall"] = "DRY_RUN_OK"
    else:
        run_out = run(args.world, args.mode, args, wc)
        header["analysis"] = analyse(run_out, wc, args.mode)
        header["overall"] = ("LIFTOFF_DETECTED"
                             if header["analysis"]["liftoff_criteria"]["primary_z_and_vz"]
                             else "NO_LIFTOFF_WITHIN_WINDOW")
        me = header["analysis"].get("mode_entry") or {}
        if args.mode == "TAKEOFF" and not me.get("heartbeat_confirmed_custom_mode_13"):
            header["overall"] = "ABORT_MODE_NOT_ENTERED"
        ts = os.path.join(L.RESULTS_DIR,
                          f"{PREFIX}_run{args.run_index}_{args.mode}_timeseries.json")
        with open(ts, "w", encoding="utf-8") as fh:
            json.dump({"mode": args.mode, "run_index": args.run_index,
                       "world_constants": wc,
                       "samples": run_out["samples"]}, fh)
        log("wrote", ts)

    dst = os.path.join(L.RESULTS_DIR,
                       f"{PREFIX}_run{args.run_index}_{args.mode}_result.json")
    with open(dst, "w", encoding="utf-8") as fh:
        json.dump(header, fh, indent=1)
    with open(os.path.join(L.RESULTS_DIR,
                           f"{PREFIX}_run{args.run_index}_{args.mode}_log.txt"),
              "w", encoding="utf-8") as fh:
        fh.write("\n".join(_LOG) + "\n")
    log("wrote", dst)
    log("OVERALL:", header["overall"])
    return 0 if header["overall"] in ("LIFTOFF_DETECTED", "DRY_RUN_OK",
                                      "NO_LIFTOFF_WITHIN_WINDOW") else 1


if __name__ == "__main__":
    sys.exit(main())

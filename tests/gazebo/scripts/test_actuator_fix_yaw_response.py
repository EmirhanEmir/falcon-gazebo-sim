#!/usr/bin/env python3
"""
FALCON V2 - YAW_RESPONSE_TEST for stage
ACTUATOR_CONTROL_LIMIT_CYCLE_FIX_AND_VALIDATION step 5.
Owner: gazebo-testing, 2026-09-12.

CLASSIFICATION: LIVE SIMULATION / MEASUREMENT-ONLY.
CHANGES NO PHYSICS PARAMETER. It sets no ArduPilot parameter, writes no
config, and touches no model, plugin or source-of-truth file. It arms, flies,
moves the rudder stick, and records.

=============================================================================
WHY THIS TEST EXISTS - AND WHY A CLEAN RUDDER UNDER PITCH EXCITATION PROVES
NOTHING
=============================================================================
Every airborne record in this repository so far (2026-09-10, 2026-09-11) was
flown with the rudder stick at exactly 1500 us, in flight conditions whose
disturbance was almost entirely LONGITUDINAL. In those records the rudder was
quiet (0.002 deg uncommanded p2p, 0 of 107 359 samples rate-pinned) while the
four +Y-axis surfaces swung 34-43 deg.

`controls-integration` established that this immunity is EXCITATION-DEPENDENT,
not structural: the rudder link's CoM sits 20.1229 mm perpendicular to its own
(near-vertical, +Z) hinge axis, so a LATERAL specific force f_y at that link
produces a real hinge torque m*f_y*d_perp on it - exactly the same mechanism
that drives the +Y surfaces under f_z. The rudder has simply never been
excited in yaw. A quiet rudder in a pitch-dominated record is therefore not
evidence of anything about the rudder.

THIS TEST EXCITES THE YAW/LATERAL AXIS ON PURPOSE and measures the rudder.

=============================================================================
WHAT IS COMMANDED
=============================================================================
After a normal ramp-and-climb, with the aircraft airborne and stabilised, the
rudder stick (RC4) is driven through a doublet ladder of increasing amplitude
and then a sustained hold:

    +/-100 us doublet, +/-200 us doublet, +/-400 us doublet, then a
    sustained +400 us hold and a sustained -400 us hold.

Rationale for the ladder (stated, not tuned to an outcome): the quantity under
test, the hinge torque m*f_y*d_perp, scales with the lateral specific force
the manoeuvre produces, so the test must sweep that quantity rather than pick
one value. The ladder is reported with the f_y each rung actually produced, so
the result is a RESPONSE CURVE, not a single pass/fail point. 400 us is the
largest step taken because it is 80 % of the stick's own half-travel and
already saturates the commanded rudder deflection well inside the +/-45 deg
mechanical limit.

The rudder is being commanded here, so "is the rudder oscillating?" must be
asked about the TRACKING ERROR (actual joint angle minus the plugin's own
rate-limited setpoint), not about the raw joint angle - the raw angle is
SUPPOSED to move. Both are recorded; the analysis script uses the tracking
error.

=============================================================================
WHAT IS RECORDED, AND WHERE THE 1 kHz DATA COMES FROM
=============================================================================
This script records, over MAVLink at the achieved stream rate: ATTITUDE
(roll/pitch/yaw + p/q/r), VFR_HUD (airspeed, altitude), SERVO_OUTPUT_RAW
(the PWM ArduPilot actually commanded), plus a PHASE MARKER LIST carrying the
WALL time of every stick transition.

It does NOT itself sample the actuator or aero topics. The 1 kHz record of
cmd_rad / setpoint_rad / actual joint angle / actual joint rate / clamp flags
/ aero coefficients / IMU comes from the pre-existing, unmodified pure
subscriber tests/gazebo/scripts/record_actuator_highrate.py, run CONCURRENTLY
by the campaign runner. The phase markers written here are in the same wall
clock that recorder stamps its messages with, and the recorder also logs the
Gazebo /clock stream, so the markers can be mapped onto sim time by the same
monotone piecewise-linear map the 2026-09-11 analysis established.

TIMEBASE CAVEAT, restated (validation 2026-09-11, finding I15): gz.msgs.
Double_V carries no header, so actuator/aero sample times in that record are
RECONSTRUCTED from wall arrival time through the /clock map. Only the IMU
carries a true publisher stamp. The per-topic transport latency DIFFERENCE
between /clock and the actuator topic is NOT measured and remains an
uncontrolled term.
"""
import argparse
import json
import math
import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import manual_takeoff_live_lib as L          # noqa: E402
from ardupilot_sitl_mav_lib import SafeMav   # noqa: E402
from pymavlink import mavutil                # noqa: E402

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
PREFIX = "actuator_fix_yaw_response"

MODE_MANUAL = 0
MODE_FBWA = 5

RC_NEUTRAL = {"rc1": 1500, "rc2": 1500, "rc3": 1000, "rc4": 1500, "rc5": 1000}

# ---- procedure constants. Every one is ASSUMPTION (procedure) and is copied
# from the existing takeoff harness where an equivalent exists, so the two
# runs are comparable. None is a physics parameter. ----
SETTLE_S = 6.0            # = TH.settle_s in test_manual_takeoff_ground_roll_reproduction.py
THROTTLE_RAMP_S = 2.0     # = TH.throttle_ramp_s, same file
CLIMB_MAX_S = 60.0        # = TH.roll_max_s, same file (upper bound, not a target)
AIRBORNE_ALT_M = 30.0     # ASSUMPTION: relative altitude at which the climb phase ends.
STABILISE_S = 6.0         # ASSUMPTION: neutral-stick dwell after reaching altitude,
                          # so the excitation starts from a settled condition and
                          # a BASELINE (unexcited) window exists in the same flight.
TAIL_S = 5.0              # ASSUMPTION: neutral-stick dwell after the ladder.
ARM_ATTEMPTS = 20
ARM_RETRY_S = 1.0
SAMPLE_PERIOD_S = 0.02

# The doublet ladder: (label, rc4 offset us, hold s).
LADDER = [
    ("baseline_neutral", 0, 4.0),
    ("doublet_100_pos", +100, 1.0), ("doublet_100_neg", -100, 1.0),
    ("rest_a", 0, 2.0),
    ("doublet_200_pos", +200, 1.0), ("doublet_200_neg", -200, 1.0),
    ("rest_b", 0, 2.0),
    ("doublet_400_pos", +400, 1.0), ("doublet_400_neg", -400, 1.0),
    ("rest_c", 0, 2.0),
    ("sustained_400_pos", +400, 3.0),
    ("rest_d", 0, 2.0),
    ("sustained_400_neg", -400, 3.0),
    ("rest_e", 0, 2.0),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5760")
    ap.add_argument("--mode", choices=("MANUAL", "FBWA"), default="FBWA",
                    help="FBWA keeps the autopilot stabilising roll/pitch so "
                         "the yaw excitation is not confounded by an "
                         "uncontrolled longitudinal departure. MANUAL is "
                         "offered for a stick-only variant.")
    ap.add_argument("--tag", default="FBWA")
    args = ap.parse_args()

    L.env_setup()
    out = {
        "test": "YAW_RESPONSE_TEST",
        "stage": "ACTUATOR_CONTROL_LIMIT_CYCLE_FIX_AND_VALIDATION step 5",
        "date": "2026-09-12", "owner": "gazebo-testing", "mode": args.mode,
        "classification": "LIVE SIMULATION / MEASUREMENT-ONLY. No physics "
                          "parameter and no ArduPilot parameter written.",
        "ladder": [{"label": a, "rc4_offset_us": b, "hold_s": c} for a, b, c in LADDER],
        "procedure_constants": {
            "settle_s": SETTLE_S, "throttle_ramp_s": THROTTLE_RAMP_S,
            "climb_max_s": CLIMB_MAX_S, "airborne_alt_m": AIRBORNE_ALT_M,
            "stabilise_s": STABILISE_S, "tail_s": TAIL_S},
        "phase_markers_wall": [], "samples": [], "events": [],
    }
    log_lines = []

    def log(*a):
        s = " ".join(str(x) for x in a)
        print(s, flush=True)
        log_lines.append(s)

    def mark(label, extra=None):
        m = {"label": label, "t_wall": time.time()}
        if extra:
            m.update(extra)
        out["phase_markers_wall"].append(m)
        log(f"[marker] {label} t_wall={m['t_wall']:.6f}")

    mav = SafeMav(args.mavlink)
    hb = mav.wait_heartbeat(timeout=30)
    if hb is None:
        log("no heartbeat - aborting")
        out["overall"] = "ABORT_NO_HEARTBEAT"
        return finish(out, log_lines, args.tag, 1)
    log("heartbeat ok", hb.to_dict())

    state = {"att": None, "hud": None, "gpi": None, "srv": None, "hb": None}

    def pump(dur=0.0):
        t0 = time.time()
        while True:
            r, _, _ = select.select([mav.m.port], [], [], 0.0 if dur == 0 else 0.01)
            if r:
                while True:
                    msg = mav.m.recv_match(blocking=False)
                    if msg is None:
                        break
                    t = msg.get_type()
                    if t == "ATTITUDE":
                        state["att"] = msg.to_dict()
                    elif t == "VFR_HUD":
                        state["hud"] = msg.to_dict()
                    elif t == "GLOBAL_POSITION_INT":
                        state["gpi"] = msg.to_dict()
                    elif t == "SERVO_OUTPUT_RAW":
                        state["srv"] = msg.to_dict()
                    elif t == "HEARTBEAT":
                        state["hb"] = msg.to_dict()
                    elif t == "STATUSTEXT":
                        out["events"].append({"t_wall": time.time(),
                                              "text": msg.text})
            if time.time() - t0 >= dur:
                return

    def sample(phase, rc):
        pump()
        att = state["att"] or {}
        hud = state["hud"] or {}
        gpi = state["gpi"] or {}
        srv = state["srv"] or {}
        out["samples"].append({
            "t_wall": time.time(), "phase": phase, "rc": dict(rc),
            "roll_deg": math.degrees(att.get("roll", float("nan"))),
            "pitch_deg": math.degrees(att.get("pitch", float("nan"))),
            "yaw_deg": math.degrees(att.get("yaw", float("nan"))),
            "p_deg_s": math.degrees(att.get("rollspeed", float("nan"))),
            "q_deg_s": math.degrees(att.get("pitchspeed", float("nan"))),
            "r_deg_s": math.degrees(att.get("yawspeed", float("nan"))),
            "airspeed_ms": hud.get("airspeed"), "groundspeed_ms": hud.get("groundspeed"),
            "alt_msl_m": hud.get("alt"), "climb_ms": hud.get("climb"),
            "rel_alt_m": (gpi.get("relative_alt", 0) / 1000.0) if gpi else None,
            "servo_us": [srv.get(f"servo{i}_raw") for i in range(1, 6)],
        })

    def hold(phase, rc, dur):
        t_end = time.time() + dur
        while time.time() < t_end:
            mav.send_rc_override(**rc)
            sample(phase, rc)
            time.sleep(SAMPLE_PERIOD_S)

    # ---- phase 1: ground settle ----
    mark("settle_start")
    log(f"phase 1: ground settle {SETTLE_S}s")
    hold("settle", RC_NEUTRAL, SETTLE_S)

    # ---- phase 2: mode + arm ----
    mark("arm_start")
    mav.command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                     p2=(MODE_MANUAL if args.mode == "MANUAL" else MODE_FBWA))
    time.sleep(1.0)
    mav.command_long(mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, p1=1)
    ack = None
    for attempt in range(ARM_ATTEMPTS):
        ack = mav.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1)
        pump()
        if ack is not None and ack.result == 0:
            break
        hold("arm_retry", RC_NEUTRAL, ARM_RETRY_S)
    out["arm_result"] = (ack.result if ack else None)
    log("arm result:", out["arm_result"])
    if ack is None or ack.result != 0:
        out["overall"] = "ABORT_ARM_FAILED"
        mav.close()
        return finish(out, log_lines, args.tag, 1)

    # ---- phase 3: throttle ramp + climb to AIRBORNE_ALT_M ----
    mark("throttle_ramp_start")
    log(f"phase 3: throttle ramp {THROTTLE_RAMP_S}s then climb, mode={args.mode}")
    t0 = time.time()
    deadline = t0 + CLIMB_MAX_S
    reached = False
    while time.time() < deadline:
        el = time.time() - t0
        frac = min(1.0, el / THROTTLE_RAMP_S)
        rc = dict(RC_NEUTRAL, rc3=int(round(1000 + 1000 * frac)))
        mav.send_rc_override(**rc)
        sample("ramp_climb", rc)
        ra = out["samples"][-1]["rel_alt_m"]
        if ra is not None and ra >= AIRBORNE_ALT_M:
            reached = True
            break
        time.sleep(SAMPLE_PERIOD_S)
    out["reached_airborne_alt"] = reached
    mark("airborne_reached", {"reached": reached,
                              "rel_alt_m": out["samples"][-1]["rel_alt_m"]})
    log("airborne alt reached:", reached,
        "rel_alt_m:", out["samples"][-1]["rel_alt_m"])

    rc_fly = dict(RC_NEUTRAL, rc3=2000)

    # ---- phase 4: stabilise at neutral stick (the in-flight BASELINE) ----
    mark("stabilise_start")
    hold("stabilise", rc_fly, STABILISE_S)

    # ---- phase 5: the yaw doublet ladder ----
    mark("ladder_start")
    for label, off, dur in LADDER:
        rc = dict(rc_fly, rc4=1500 + off)
        mark(f"ladder_{label}", {"rc4_us": rc["rc4"], "hold_s": dur})
        hold(f"ladder_{label}", rc, dur)
    mark("ladder_end")

    # ---- phase 6: tail ----
    hold("tail", rc_fly, TAIL_S)
    mark("run_end")

    # Deliberately NOT disarming: the aircraft is airborne. The campaign
    # runner tears the whole gz+arduplane pair down. Disarming an airborne
    # aircraft (which the ground-roll harness does at the end of its own run,
    # by design, because it is a ground-roll test) would stop both motors and
    # contaminate the tail of the record.
    mav.close()
    out["overall"] = "COMPLETED" if reached else "COMPLETED_WITHOUT_TARGET_ALT"
    log("captured", len(out["samples"]), "MAVLink samples")
    return finish(out, log_lines, args.tag, 0)


def finish(out, log_lines, tag, rc):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    p = os.path.join(RESULTS_DIR, f"{PREFIX}_{tag}_result.json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    with open(os.path.join(RESULTS_DIR, f"{PREFIX}_{tag}_log.txt"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(log_lines) + "\n")
    print("wrote", p, flush=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())

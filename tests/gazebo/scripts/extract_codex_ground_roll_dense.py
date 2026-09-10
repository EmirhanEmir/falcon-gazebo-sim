#!/usr/bin/env python3
"""FALCON V2 - dense, READ-ONLY trace of one codex ground roll, at the native
SIM2 rate, to timestamp the LATERAL DEPARTURE against the CONTROL-SURFACE
COMMAND departure.

CLASSIFICATION: MEASUREMENT-ONLY, READ-ONLY.
Owner: gazebo-testing. Created 2026-09-10.
Reads codex/runtime/manual_takeoff/logs/*.BIN. Writes nothing under codex/.

The coarse extraction (extract_codex_manual_takeoff_dataflash.py) showed the
user's own runs departing ~37 m to the LEFT during the ground roll. The
decisive datum for the next task is the ORDER of two events:
    (a) the first departure of yaw / yaw rate from zero, and
    (b) the first departure of the aileron and rudder SERVO OUTPUT from trim.
This script reports both, per log, with explicit thresholds and the raw signal
named for each.

THRESHOLDS (stated, not tuned)
  yaw angle      : |SIM.Yaw - Yaw_at_roll_start| >= 0.5 deg
  yaw rate       : |IMU.GyrZ| >= 1.0 deg/s  (same number the Part 4 harness
                   uses for its yaw-rate onset)
  lateral        : |SIM2.PN| >= 0.05 m
  servo output   : |RCOU.Cx - 1500| >= 5 us   (SERVOx_TRIM is 1500 in
                   config/ardupilot/falcon_v2_sitl.parm; 5 us is ~0.32 deg of
                   surface at this transport's derived 0.0643 deg/us, i.e.
                   comfortably above quantisation and below any real command)

USAGE
    python3 extract_codex_ground_roll_dense.py [00000004.BIN ...]
"""
import json
import math
import os
import sys

from pymavlink import mavutil

REPO = "/home/emirhan/Desktop/FalconV2"
LOGDIR = os.path.join(REPO, "codex/runtime/manual_takeoff/logs")
RES = os.path.join(REPO, "tests/gazebo/results")

YAW_DEG_THR = 0.5
YAW_RATE_DEG_S_THR = 1.0
LAT_M_THR = 0.05
SERVO_US_THR = 5.0
SERVO_TRIM = 1500.0
MODE_NAMES = {0: "MANUAL", 5: "FBWA", 11: "RTL", 13: "TAKEOFF", 15: "GUIDED",
              16: "INITIALISING", 26: "THERMAL"}


def load(path, types):
    m = mavutil.mavlink_connection(path)
    rows = {k: [] for k in types}
    while True:
        msg = m.recv_match()
        if msg is None:
            break
        t = msg.get_type()
        if t in rows:
            d = msg.to_dict()
            d.pop("mavpackettype", None)
            rows[t].append(d)
    return rows


def nearest(series, t_us, key):
    if not series:
        return None
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid]["TimeUS"] < t_us:
            lo = mid + 1
        else:
            hi = mid
    best = series[lo]
    if lo > 0 and abs(series[lo - 1]["TimeUS"] - t_us) < abs(best["TimeUS"] - t_us):
        best = series[lo - 1]
    return best.get(key)


def analyse(path):
    rows = load(path, {"SIM", "SIM2", "ARSP", "RCOU", "RCIN", "IMU", "MODE",
                       "CTUN", "ATT"})
    sim2 = rows["SIM2"]
    if not sim2:
        return {"file": path, "status": "NO_SIM2"}
    imu0 = [r for r in rows["IMU"] if r.get("I") == 0]

    # ground roll = from the last slow-and-on-the-ground sample before the
    # peak groundspeed, up to either liftoff or the end of forward motion
    gs = [math.hypot(s["VN"], s["VE"]) for s in sim2]
    i_peak = max(range(len(gs)), key=lambda i: gs[i])
    i0 = i_peak
    while i0 > 0 and gs[i0] >= 0.5:
        i0 -= 1
    i1 = i_peak
    while i1 < len(sim2) - 1 and (-sim2[i1]["PD"]) < 5.0:
        i1 += 1
    seg = sim2[i0:i1 + 1]
    t0 = seg[0]["TimeUS"]
    yaw0 = nearest(rows["SIM"], t0, "Yaw")

    ev = {}

    def stamp(i, what, raw):
        s = seg[i]
        t = s["TimeUS"]
        ev[what] = {
            "t_rel_s": (t - t0) / 1e6,
            "raw_signal": raw,
            "groundspeed_ms": math.hypot(s["VN"], s["VE"]),
            "airspeed_ms": nearest(rows["ARSP"], t, "Airspeed"),
            "lateral_N_m_LEFT_POSITIVE": s["PN"],
            "alt_m": -s["PD"],
            "yaw_deg": nearest(rows["SIM"], t, "Yaw"),
            "yaw_rate_deg_s_body_z": (
                math.degrees(nearest(imu0, t, "GyrZ") or 0.0)),
            "rcou": {k: nearest(rows["RCOU"], t, c) for k, c in
                     (("aileron", "C1"), ("elevator", "C2"),
                      ("throttle_left", "C3"), ("rudder", "C4"),
                      ("throttle_right", "C5"))},
            "mode": MODE_NAMES.get(nearest(rows["MODE"], t, "ModeNum")),
        }

    for i, s in enumerate(seg):
        t = s["TimeUS"]
        if "yaw_angle_departure" not in ev:
            y = nearest(rows["SIM"], t, "Yaw")
            if y is not None and yaw0 is not None and abs(y - yaw0) >= YAW_DEG_THR:
                stamp(i, "yaw_angle_departure",
                      f"SIM.Yaw (simulator ground truth) moved >= {YAW_DEG_THR} deg")
        if "yaw_rate_departure" not in ev:
            g = nearest(imu0, t, "GyrZ")
            if g is not None and abs(math.degrees(g)) >= YAW_RATE_DEG_S_THR:
                stamp(i, "yaw_rate_departure",
                      f"IMU[0].GyrZ >= {YAW_RATE_DEG_S_THR} deg/s")
        if "lateral_departure" not in ev and abs(s["PN"]) >= LAT_M_THR:
            stamp(i, "lateral_departure",
                  f"SIM2.PN (ground truth North = LEFT) >= {LAT_M_THR} m")
        for name, ch in (("aileron", "C1"), ("rudder", "C4"), ("elevator", "C2")):
            key = f"servo_{name}_departure"
            if key not in ev:
                v = nearest(rows["RCOU"], t, ch)
                if v is not None and abs(v - SERVO_TRIM) >= SERVO_US_THR:
                    stamp(i, key,
                          f"RCOU.{ch} ({name}) moved >= {SERVO_US_THR} us from "
                          f"the 1500 us SERVO trim")

    step = max(1, len(seg) // 400)
    trace = []
    for s in seg[::step]:
        t = s["TimeUS"]
        trace.append({
            "t_rel_s": (t - t0) / 1e6,
            "gs_ms": math.hypot(s["VN"], s["VE"]),
            "airspeed_ms": nearest(rows["ARSP"], t, "Airspeed"),
            "lateral_N_m_LEFT_POSITIVE": s["PN"],
            "along_runway_E_m": s["PE"],
            "alt_m": -s["PD"],
            "vz_ms": -s["VD"],
            "yaw_deg": nearest(rows["SIM"], t, "Yaw"),
            "yaw_rate_deg_s": math.degrees(nearest(imu0, t, "GyrZ") or 0.0),
            "roll_deg": nearest(rows["SIM"], t, "Roll"),
            "pitch_deg": nearest(rows["SIM"], t, "Pitch"),
            "rcou_ail": nearest(rows["RCOU"], t, "C1"),
            "rcou_ele": nearest(rows["RCOU"], t, "C2"),
            "rcou_thr_l": nearest(rows["RCOU"], t, "C3"),
            "rcou_rud": nearest(rows["RCOU"], t, "C4"),
            "rcou_thr_r": nearest(rows["RCOU"], t, "C5"),
            "rcin_ail": nearest(rows["RCIN"], t, "C1"),
            "rcin_ele": nearest(rows["RCIN"], t, "C2"),
            "rcin_thr": nearest(rows["RCIN"], t, "C3"),
            "rcin_rud": nearest(rows["RCIN"], t, "C4"),
            "mode": MODE_NAMES.get(nearest(rows["MODE"], t, "ModeNum")),
        })
    return {
        "file": path,
        "status": "OK",
        "yaw_deg_at_roll_start": yaw0,
        "peak_groundspeed_ms": gs[i_peak],
        "peak_groundspeed_t_rel_s": (sim2[i_peak]["TimeUS"] - t0) / 1e6,
        "segment_duration_s": (seg[-1]["TimeUS"] - t0) / 1e6,
        "events": ev,
        "trace": trace,
    }


def main():
    names = sys.argv[1:] or sorted(p for p in os.listdir(LOGDIR)
                                   if p.endswith(".BIN"))
    docs = []
    for n in names:
        full = n if os.path.isabs(n) else os.path.join(LOGDIR, n)
        print("reading", full, flush=True)
        d = analyse(full)
        docs.append(d)
        if d["status"] != "OK":
            print("  ", d["status"])
            continue
        print("   peak gs=%.2f m/s at t=%.2f s; segment %.2f s"
              % (d["peak_groundspeed_ms"], d["peak_groundspeed_t_rel_s"],
                 d["segment_duration_s"]))
        for k in sorted(d["events"]):
            e = d["events"][k]
            print("   %-26s t=%7.3f gs=%6.2f latN=%+8.3f yaw=%7.2f "
                  "yawrate=%7.2f ail=%s rud=%s ele=%s mode=%s"
                  % (k, e["t_rel_s"], e["groundspeed_ms"],
                     e["lateral_N_m_LEFT_POSITIVE"], e["yaw_deg"],
                     e["yaw_rate_deg_s_body_z"], e["rcou"]["aileron"],
                     e["rcou"]["rudder"], e["rcou"]["elevator"], e["mode"]))
    dst = os.path.join(RES, "codex_manual_takeoff_ground_roll_dense.json")
    with open(dst, "w", encoding="utf-8") as fh:
        json.dump({"owner": "gazebo-testing",
                   "read_only": "nothing under codex/ was modified",
                   "thresholds": {"yaw_deg": YAW_DEG_THR,
                                  "yaw_rate_deg_s": YAW_RATE_DEG_S_THR,
                                  "lateral_m": LAT_M_THR,
                                  "servo_us_from_1500": SERVO_US_THR},
                   "logs": docs}, fh, indent=1)
    print("wrote", dst)
    return 0


if __name__ == "__main__":
    sys.exit(main())

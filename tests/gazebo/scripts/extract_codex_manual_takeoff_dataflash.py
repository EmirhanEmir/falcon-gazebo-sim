#!/usr/bin/env python3
"""FALCON V2 - READ-ONLY extraction of the ground-roll / takeoff segments from
the PRE-EXISTING codex manual-takeoff dataflash logs.

CLASSIFICATION: MEASUREMENT-ONLY, READ-ONLY.
Stage: MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION.
Owner: gazebo-testing. Created 2026-09-10.

Reads codex/runtime/manual_takeoff/logs/*.BIN. NOTHING under codex/ is written
or modified. No physics parameter of any kind is read for modification.

WHY
---
The user watched a manual takeoff on the EXISTING temporary runway setup and
reported: a LEFT drift while accelerating, ~24 m/s, then a sudden liftoff.
Those .BIN files are dated 2026-09-08 and are the only recording of that
session, so if one of them contains that run it is the single most valuable
artifact in this stage. This script finds every takeoff-shaped segment in each
file and reports its timeseries, so the user's own run can be compared
directly against the fresh Part 4 runs.

FRAME / CONVENTION (stated explicitly, never mixed)
---------------------------------------------------
SIM2 carries the SIMULATOR's own ground truth in NED: PN (North), PE (East),
PD (Down). At the start of every one of these logs PE = -150.000 m, PN = 0,
and SIM.Yaw = 90 deg, which matches the runway world's spawn pose
(-150, 0, 0.5) with the nose along Gazebo world +X. Therefore, for these logs:
    Gazebo world +X  ==  NED East   (the runway centreline, nose direction)
    Gazebo world +Y  ==  NED North  (the aircraft's LEFT)
    Gazebo world +Z  ==  NED -Down
So LATERAL DISPLACEMENT = SIM2.PN, and POSITIVE PN = LEFT, matching the
Part 4 convention that positive world Y is left. Altitude = -PD.

LIFTOFF CRITERION (explicit, raw signal named)
----------------------------------------------
    RAW SIGNAL: SIM2.PD (simulator ground-truth Down position) and its
                finite difference.
    CRITERION : (-PD) > 0.10 m above the segment's own resting altitude AND
                (-VD) > 0.5 m/s, both continuously true for 0.30 s. This is
                the SAME criterion and the SAME numbers the Part 4 harness
                uses, applied to the dataflash equivalent of the same
                quantity, so the two are directly comparable.

USAGE
    python3 extract_codex_manual_takeoff_dataflash.py
"""
import json
import math
import os
import sys

from pymavlink import mavutil

REPO = "/home/emirhan/Desktop/FalconV2"
LOGDIR = os.path.join(REPO, "codex/runtime/manual_takeoff/logs")
RES = os.path.join(REPO, "tests/gazebo/results")

LIFTOFF_Z_MARGIN_M = 0.10
LIFTOFF_VZ_MIN_MS = 0.5
LIFTOFF_HOLD_S = 0.30

# ArduPlane flight-mode numbers seen in these logs.
MODE_NAMES = {0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING",
              4: "ACRO", 5: "FBWA", 6: "FBWB", 7: "CRUISE", 8: "AUTOTUNE",
              10: "AUTO", 11: "RTL", 12: "LOITER", 13: "TAKEOFF",
              15: "GUIDED", 16: "INITIALISING", 26: "THERMAL"}

WANT = {"SIM", "SIM2", "ATT", "GPS", "ARSP", "RCOU", "RCIN", "CTUN", "MODE",
        "POS", "IMU", "XKF1", "EV", "MSG"}


def load(path):
    m = mavutil.mavlink_connection(path)
    rows = {k: [] for k in WANT}
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


def sample_at(series, t_us, key):
    """Nearest-in-time value of `key` from `series` (assumed time-ordered)."""
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


def find_takeoffs(sim2):
    """Segments that look like: on the ground, then a sustained departure."""
    if len(sim2) < 10:
        return []
    segs = []
    i = 0
    n = len(sim2)
    while i < n:
        # find the next time we are on the ground and slow
        while i < n and not (abs(sim2[i]["PD"]) < 0.5
                             and math.hypot(sim2[i]["VN"], sim2[i]["VE"]) < 1.0):
            i += 1
        if i >= n:
            break
        start = i
        resting_alt = -sim2[i]["PD"]
        run_start = None
        lift = None
        j = i
        while j < n:
            alt = -sim2[j]["PD"]
            vz = -sim2[j]["VD"]
            if alt > resting_alt + LIFTOFF_Z_MARGIN_M and vz > LIFTOFF_VZ_MIN_MS:
                if run_start is None:
                    run_start = j
                elif (sim2[j]["TimeUS"] - sim2[run_start]["TimeUS"]) / 1e6 >= LIFTOFF_HOLD_S:
                    lift = run_start
                    break
            else:
                run_start = None
            j += 1
        if lift is None:
            break
        segs.append({"start_idx": start, "liftoff_idx": lift,
                     "resting_alt_m": resting_alt})
        # skip forward past this departure until back on the ground
        k = lift
        while k < n and -sim2[k]["PD"] > resting_alt + 1.0:
            k += 1
        i = max(k, lift + 1)
    return segs


def describe(path, rows):
    sim2 = rows["SIM2"]
    out = {"file": path, "n_SIM2": len(sim2), "takeoff_segments": []}
    if not sim2:
        out["status"] = "NO_SIM2_GROUND_TRUTH"
        return out
    t_first = sim2[0]["TimeUS"]
    out["duration_s"] = (sim2[-1]["TimeUS"] - t_first) / 1e6
    out["modes_seen"] = sorted({(m["ModeNum"], MODE_NAMES.get(m["ModeNum"], "?"))
                                for m in rows["MODE"]})
    out["start_position_NED"] = {"PN": sim2[0]["PN"], "PE": sim2[0]["PE"],
                                 "PD": sim2[0]["PD"]}
    out["max_groundspeed_ms"] = max(math.hypot(s["VN"], s["VE"]) for s in sim2)
    out["max_altitude_m"] = max(-s["PD"] for s in sim2)

    for seg in find_takeoffs(sim2):
        li = seg["liftoff_idx"]
        t_lift = sim2[li]["TimeUS"]
        # ground-roll start: last time before liftoff that groundspeed < 0.5
        gs0 = li
        while gs0 > 0 and math.hypot(sim2[gs0]["VN"], sim2[gs0]["VE"]) >= 0.5:
            gs0 -= 1
        roll = sim2[gs0:li + 1]
        pn = [s["PN"] for s in roll]
        lat_at_lift = sim2[li]["PN"]
        step = max(1, len(roll) // 150)
        trace = []
        for s in roll[::step]:
            t = s["TimeUS"]
            trace.append({
                "t_rel_s": (t - sim2[gs0]["TimeUS"]) / 1e6,
                "gs_ms": math.hypot(s["VN"], s["VE"]),
                "airspeed_ms": sample_at(rows["ARSP"], t, "Airspeed"),
                "lateral_N_m_LEFT_POSITIVE": s["PN"],
                "along_runway_E_m": s["PE"],
                "alt_m": -s["PD"],
                "vz_ms": -s["VD"],
                "roll_deg": sample_at(rows["SIM"], t, "Roll"),
                "pitch_deg": sample_at(rows["SIM"], t, "Pitch"),
                "yaw_deg": sample_at(rows["SIM"], t, "Yaw"),
                "mode": MODE_NAMES.get(sample_at(rows["MODE"], t, "ModeNum")),
                "rcou_ail": sample_at(rows["RCOU"], t, "C1"),
                "rcou_ele": sample_at(rows["RCOU"], t, "C2"),
                "rcou_thr_l": sample_at(rows["RCOU"], t, "C3"),
                "rcou_rud": sample_at(rows["RCOU"], t, "C4"),
                "rcou_thr_r": sample_at(rows["RCOU"], t, "C5"),
                "throttle_out_pct": sample_at(rows["CTUN"], t, "ThO"),
            })
        yaw0 = sample_at(rows["SIM"], sim2[gs0]["TimeUS"], "Yaw")
        yaw_lift = sample_at(rows["SIM"], t_lift, "Yaw")
        out["takeoff_segments"].append({
            "ground_roll_start_t_s": (sim2[gs0]["TimeUS"] - t_first) / 1e6,
            "liftoff_t_s": (t_lift - t_first) / 1e6,
            "ground_roll_duration_s": (t_lift - sim2[gs0]["TimeUS"]) / 1e6,
            "mode_at_liftoff": MODE_NAMES.get(sample_at(rows["MODE"], t_lift, "ModeNum")),
            "liftoff_groundspeed_ms": math.hypot(sim2[li]["VN"], sim2[li]["VE"]),
            "liftoff_airspeed_ms": sample_at(rows["ARSP"], t_lift, "Airspeed"),
            "liftoff_alt_m": -sim2[li]["PD"],
            "liftoff_criterion": "SIM2 ground-truth: (-PD) > resting+0.10 m AND "
                                 "(-VD) > 0.5 m/s held 0.30 s (same numbers as "
                                 "the Part 4 harness)",
            "lateral_N_at_liftoff_m": lat_at_lift,
            "lateral_direction_at_liftoff": ("LEFT (+N)" if lat_at_lift > 0 else
                                             "RIGHT (-N)" if lat_at_lift < 0 else "NONE"),
            "max_abs_lateral_N_during_roll_m": max(abs(v) for v in pn),
            "final_lateral_N_of_roll_m": pn[-1],
            "yaw_deg_at_roll_start": yaw0,
            "yaw_deg_at_liftoff": yaw_lift,
            "yaw_change_during_roll_deg": (None if yaw0 is None or yaw_lift is None
                                           else yaw_lift - yaw0),
            "pitch_deg_at_liftoff": sample_at(rows["SIM"], t_lift, "Pitch"),
            "roll_deg_at_liftoff": sample_at(rows["SIM"], t_lift, "Roll"),
            "rcou_at_liftoff": {k: sample_at(rows["RCOU"], t_lift, c) for k, c in
                                (("aileron", "C1"), ("elevator", "C2"),
                                 ("throttle_left", "C3"), ("rudder", "C4"),
                                 ("throttle_right", "C5"))},
            "n_trace_points": len(trace),
            "trace": trace,
        })
    if not out["takeoff_segments"]:
        out["status"] = ("NO_TAKEOFF_SEGMENT_FOUND - this log contains no "
                         "ground roll that ends in a sustained departure by "
                         "the stated criterion.")
    else:
        out["status"] = "OK"
    return out


def main():
    paths = sorted(p for p in os.listdir(LOGDIR) if p.endswith(".BIN"))
    docs = []
    for name in paths:
        full = os.path.join(LOGDIR, name)
        print("reading", full, flush=True)
        rows = load(full)
        d = describe(full, rows)
        docs.append(d)
        print("   status:", d["status"], " segments:", len(d["takeoff_segments"]),
              " max_gs=%.2f max_alt=%.1f" % (d.get("max_groundspeed_ms", 0),
                                             d.get("max_altitude_m", 0)),
              flush=True)
        for s in d["takeoff_segments"]:
            print("     liftoff t=%.1f s mode=%s AS=%.2f GS=%.2f latN=%+.3f m (%s) "
                  "yawchg=%s pitch=%.2f" % (
                      s["liftoff_t_s"], s["mode_at_liftoff"],
                      s["liftoff_airspeed_ms"] or float("nan"),
                      s["liftoff_groundspeed_ms"],
                      s["lateral_N_at_liftoff_m"],
                      s["lateral_direction_at_liftoff"],
                      None if s["yaw_change_during_roll_deg"] is None
                      else round(s["yaw_change_during_roll_deg"], 3),
                      s["pitch_deg_at_liftoff"] or float("nan")), flush=True)
    out = {
        "stage": "MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION",
        "part": "pre-existing codex dataflash extraction",
        "owner": "gazebo-testing",
        "source": LOGDIR,
        "read_only": "Nothing under codex/ was written or modified.",
        "convention": "lateral displacement = SIM2.PN (NED North); with the "
                      "nose along East, POSITIVE PN = LEFT.",
        "logs": docs,
    }
    dst = os.path.join(RES, "codex_manual_takeoff_dataflash_extraction.json")
    with open(dst, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    print("wrote", dst)
    return 0


if __name__ == "__main__":
    sys.exit(main())

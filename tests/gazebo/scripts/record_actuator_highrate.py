#!/usr/bin/env python3
"""FALCON V2 - HIGH-RATE actuator/aero/IMU raw recorder.

CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY / OBSERVABILITY-ONLY.
Owner: gazebo-testing. Created 2026-09-11 for stage
AIRBORNE_ACTUATOR_LIMIT_CYCLE_ROOT_CAUSE (runtime verification pass).

WHY THIS EXISTS
---------------
The 2026-09-10 controls-integration offline analysis
(docs/test_results/2026-09-10_airborne_actuator_limit_cycle_root_cause.md)
states that every rate-pinned count it reports is a LOWER BOUND, because a
limit enforced by the physics engine at the 1 kHz physics tick was observed
through a 20 Hz diagnostics topic, and that the oscillation frequency was
read off a 50 Hz channel. Its own recommendation #6 assigns the harness-side
observation rate to `gazebo-testing`.

The existing live harness
(tests/gazebo/scripts/test_manual_takeoff_ground_roll_reproduction.py) polls
`latest()` in a ~100 Hz python loop, so it can never resolve more than the
topic publish rate and it re-reads duplicates. This recorder instead logs
EVERY message that arrives on each topic, with its wall arrival time, and
does no per-sample work beyond appending the raw bytes. It runs as a
SEPARATE process alongside the unmodified harness and touches nothing the
harness owns.

WHAT IT DOES NOT DO
-------------------
It commands nothing, writes no parameter, loads no config for modification,
and publishes on no topic. It is a pure subscriber. It cannot change the
simulation.

USAGE
  record_actuator_highrate.py --world <name> --out <path> --duration <s>
"""
import argparse
import json
import os
import sys
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ACT_TOPIC = "/model/falcon_v2/actuators/diagnostics"
AERO_TOPIC = "/model/falcon_v2/aerodynamics/diagnostics"
IMU_TOPIC = "/model/falcon_v2/sensors/imu"

# Field order restated from the owning libraries, never redefined here.
import actuator_lib as ACT          # noqa: E402
import aero_lib as AL               # noqa: E402

ACT_SURFACES = ACT.SURFACES  # order per actuator_lib.py, never redefined here
ACT_FIELDS = ACT.DIAG_FIELDS
AERO_FIELDS = AL.DiagSubscriber.FIELDS


class RawLogger:
    """Appends (wall_time, payload_bytes) for every message. No parsing in
    the callback - parsing is done once at dump time so the callback cost
    cannot become the rate limit."""

    def __init__(self, node, topic, msgtype):
        import gz.transport13 as tp
        self.topic = topic
        self.buf = []
        self.lock = threading.Lock()
        self.ok = node.subscribe_raw(topic, self._cb, msgtype,
                                     tp.SubscribeOptions())

    def _cb(self, data, info):
        t = time.time()
        with self.lock:
            self.buf.append((t, data))

    def snapshot(self):
        with self.lock:
            return list(self.buf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=200.0)
    args = ap.parse_args()

    import gz.transport13 as tp
    node = tp.Node()

    act = RawLogger(node, ACT_TOPIC, "gz.msgs.Double_V")
    aero = RawLogger(node, AERO_TOPIC, "gz.msgs.Double_V")
    imu = RawLogger(node, IMU_TOPIC, "gz.msgs.IMU")

    # Clock: (t_sim, t_wall) pairs, used to regress wall -> sim for all the
    # other channels. The residual of that regression is REPORTED, not
    # assumed negligible.
    clk = {"pairs": []}
    clk_lock = threading.Lock()

    def clk_cb(data, info):
        from gz.msgs10 import clock_pb2
        t = time.time()
        m = clock_pb2.Clock()
        m.ParseFromString(data)
        s = m.sim.sec + m.sim.nsec * 1e-9
        with clk_lock:
            clk["pairs"].append((t, s))

    clk_ok = node.subscribe_raw(f"/world/{args.world}/clock", clk_cb,
                                "gz.msgs.Clock", tp.SubscribeOptions())

    sys.stderr.write(f"[highrate] subscribed act={act.ok} aero={aero.ok} "
                     f"imu={imu.ok} clock={clk_ok}\n")
    sys.stderr.flush()

    t_end = time.time() + args.duration
    try:
        while time.time() < t_end:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    from gz.msgs10 import double_v_pb2, imu_pb2

    def parse_dv(entries, splitter):
        out = []
        m = double_v_pb2.Double_V()
        for tw, data in entries:
            m.Clear()
            m.ParseFromString(data)
            out.append((tw, splitter(list(m.data))))
        return out

    act_rows = parse_dv(act.snapshot(), lambda v: v)
    aero_rows = parse_dv(aero.snapshot(), lambda v: v)

    imu_rows = []
    mi = imu_pb2.IMU()
    for tw, data in imu.snapshot():
        mi.Clear()
        mi.ParseFromString(data)
        st = mi.header.stamp
        imu_rows.append({
            "t_wall": tw,
            "t_stamp_s": st.sec + st.nsec * 1e-9,
            "gyro_rad_s": [mi.angular_velocity.x, mi.angular_velocity.y,
                            mi.angular_velocity.z],
            "accel_ms2": [mi.linear_acceleration.x, mi.linear_acceleration.y,
                           mi.linear_acceleration.z],
            "orientation_wxyz": [mi.orientation.w, mi.orientation.x,
                                  mi.orientation.y, mi.orientation.z],
        })

    with clk_lock:
        pairs = list(clk["pairs"])

    payload = {
        "classification": "TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY / "
                          "OBSERVABILITY-ONLY. Pure subscriber. Wrote no "
                          "parameter, published on no topic.",
        "world": args.world,
        "topics": {"actuator": ACT_TOPIC, "aero": AERO_TOPIC,
                    "imu": IMU_TOPIC,
                    "clock": f"/world/{args.world}/clock"},
        "subscribe_ok": {"actuator": bool(act.ok), "aero": bool(aero.ok),
                          "imu": bool(imu.ok), "clock": bool(clk_ok)},
        "actuator_field_order": {"surfaces": ACT_SURFACES,
                                  "fields_per_surface": ACT_FIELDS},
        "aero_field_order": AERO_FIELDS,
        "duration_requested_s": args.duration,
        "clock_pairs_wall_sim": pairs,
        "actuator": [{"t_wall": t, "v": v} for t, v in act_rows],
        "aero": [{"t_wall": t, "v": v} for t, v in aero_rows],
        "imu": imu_rows,
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(payload, fh)
    sys.stderr.write(
        f"[highrate] wrote {args.out}: act={len(act_rows)} aero={len(aero_rows)} "
        f"imu={len(imu_rows)} clock={len(pairs)}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

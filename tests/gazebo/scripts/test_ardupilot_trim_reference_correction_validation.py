#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_TRIM_REFERENCE_CORRECTION_VALIDATION (gazebo-testing,
2026-08-28). Short, bounded stage - ONE ~20-30s straight-and-level/neutral
closed-loop FBWA flight using the CORRECTED trim reference constants that
`controls-integration` just fixed at the source, in
test_ardupilot_basic_closed_loop_flight.py:

    TRIM_THROTTLE      0.4915  -> 0.5010
    ELEVATOR_THETA_RAD 5.50deg -> 4.50deg
    V_TRIM             18.165  -> 18.166
    ALPHA_TRIM_DEG     2.461   -> 2.473

Purpose: check whether the previously-observed ~0.67 m/s steady sink
(docs/test_results/2026-08-28_ardupilot_pitch_pid_phugoid_baseline.md sec 6,
measured under the STALE trim constants above) is meaningfully reduced now
that the trim reference itself has been corrected.

NOT a tuning stage. No PID changes, no TECS, no actuator/propulsion/
aero-coefficient change, no long campaign - one flight, one comparison.
Reuses the already-proven PHASE1-4 precondition mechanism VERBATIM from
`base` (test_ardupilot_basic_closed_loop_flight.py: phase1_mavlink_arm,
wait_ground_settle, phase2_teleport_and_verify, phase3_hold_to_trim - all
of which automatically pick up the corrected trim constants above, since
they are read from `base`'s own module-level constants, not re-derived
here) and `campaign` (test_ardupilot_basic_closed_loop_flight_campaign.py:
enter_fbwa, run_segment, build_sample combined MAVLink+gz-transport
telemetry) - exactly the same reuse pattern as
test_ardupilot_pitch_pid_phugoid_baseline.py. The only new code in this
file is the single neutral-hold segment definition and the sink-rate
comparison analysis.

No aircraft-physics parameter (mass, CG, inertia, aerodynamic coefficient,
control authority, motor thrust, PID gain) is read for any purpose other
than citing already-published/live-confirmed values, and none is modified
anywhere in this file. model/model.sdf, falcon_v2_sitl.parm, the real
.param, every PID value, and every plugin under plugins/ are unmodified -
all explicitly out of scope for this stage.

Usage (Gazebo + arduplane already running, a FRESH pair, per the proven
launch sequence in the prior stages' reports):
    python3 test_ardupilot_trim_reference_correction_validation.py
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import gz.transport13 as tp  # noqa: E402
from gz.msgs10 import entity_wrench_pb2, entity_pb2  # noqa: E402

import test_ardupilot_basic_closed_loop_flight as base  # noqa: E402
import test_ardupilot_basic_closed_loop_flight_campaign as campaign  # noqa: E402
import aero_lib  # noqa: E402
import propulsion_lib  # noqa: E402
import actuator_lib  # noqa: E402

OUT_JSON = os.path.join(base.RESULTS_DIR, "ardupilot_trim_reference_correction_validation_result.json")

SEGMENT_LABEL = "neutral_stabilization_corrected_trim"
SEGMENT_DURATION_S = 25.0
TRANSIENT_CUTOFF_S = 5.0  # excludes the PHASE3->FBWA handoff transient, per task instruction

# Cited, not re-derived: docs/test_results/2026-08-28_ardupilot_pitch_pid_
# phugoid_baseline.md sec 6 - overall average sink measured under the STALE
# trim (throttle=0.4915, elevator=5.50deg, V_TRIM=18.165, alpha=2.461deg)
# over its full 46.99s flight (altitude 89.70m -> 58.11m). Also cited: the
# earlier basic_closed_loop_flight campaign's own per-flight range
# (0.5-0.8 m/s across its Flights 1-3/4, e.g. Flight 1 0.785 m/s).
PRIOR_STALE_TRIM_AVG_SINK_MS = 0.672
PRIOR_STALE_TRIM_RANGE_MS = (0.5, 0.8)


def linreg_slope(ts, ys):
    n = len(ts)
    if n < 2:
        return None
    tbar = sum(ts) / n
    ybar = sum(ys) / n
    den = sum((t - tbar) ** 2 for t in ts)
    if den == 0:
        return None
    num = sum((t - tbar) * (y - ybar) for t, y in zip(ts, ys))
    return num / den


def sink_stats(subset, label):
    full_pts = [(s["t"], s["gz"]["pos"][2]) for s in subset if s["gz"]["pos"] is not None]
    if len(full_pts) < 2:
        return dict(window=label, n_samples=len(full_pts))
    ts = [p[0] for p in full_pts]
    alts = [p[1] for p in full_pts]
    t0, t1 = ts[0], ts[-1]
    z0, z1 = alts[0], alts[-1]
    endpoint_sink = -(z1 - z0) / (t1 - t0) if t1 > t0 else None
    slope = linreg_slope(ts, alts)
    regression_sink = -slope if slope is not None else None
    return dict(window=label, t_start=t0, t_end=t1, duration_s=t1 - t0,
                alt_start=z0, alt_end=z1,
                endpoint_avg_sink_ms=endpoint_sink,
                linear_regression_avg_sink_ms=regression_sink,
                n_samples=len(full_pts))


def stat3(vals):
    return dict(min=min(vals), max=max(vals), mean=sum(vals) / len(vals)) if vals else None


def analyze(seg):
    samples = seg["samples"]
    full = samples
    stab = [s for s in samples if s["t"] >= TRANSIENT_CUTOFF_S]

    out = dict(
        full_window=sink_stats(full, "full_0_to_end"),
        stabilized_window=sink_stats(stab, f"stabilized_{TRANSIENT_CUTOFF_S}s_to_end"),
    )

    if stab:
        pitches_gz = [s["gz"]["att_deg"][1] for s in stab if s["gz"]["att_deg"] is not None]
        pitches_mav = [s["mav"]["att_pitch_deg"] for s in stab if s["mav"]["att_pitch_deg"] is not None]
        airspeeds = [s["mav"]["airspeed"] for s in stab if s["mav"]["airspeed"] is not None]
        throttle_pct = [s["mav"]["throttle_pct"] for s in stab if s["mav"]["throttle_pct"] is not None]
        thr_L = [s["propulsion"]["left"]["throttle"] for s in stab if s["propulsion"]]
        thr_R = [s["propulsion"]["right"]["throttle"] for s in stab if s["propulsion"]]
        elev_cmd_L = [s["actuators"]["left_elevator"]["cmd_rad"] for s in stab if s["actuators"]]
        elev_cmd_R = [s["actuators"]["right_elevator"]["cmd_rad"] for s in stab if s["actuators"]]
        elev_act_L = [s["actuators"]["left_elevator"]["actual_angle_rad"] for s in stab if s["actuators"]]
        elev_act_R = [s["actuators"]["right_elevator"]["actual_angle_rad"] for s in stab if s["actuators"]]
        clamp_samples = [s for s in stab if s["actuators"] and any(
            s["actuators"][surf]["target_clamp_active"] or s["actuators"][surf]["effort_clamp_active"]
            for surf in s["actuators"])]
        out["stabilized_diagnostics"] = dict(
            n=len(stab),
            pitch_gz_deg=stat3(pitches_gz),
            pitch_mav_deg=stat3(pitches_mav),
            airspeed_ms=stat3(airspeeds),
            throttle_pct_mav=stat3(throttle_pct),
            throttle_actual_L=stat3(thr_L),
            throttle_actual_R=stat3(thr_R),
            elevator_cmd_rad_L=stat3(elev_cmd_L),
            elevator_cmd_rad_R=stat3(elev_cmd_R),
            elevator_actual_rad_L=stat3(elev_act_L),
            elevator_actual_rad_R=stat3(elev_act_R),
            clamp_event_samples=len(clamp_samples),
        )
    return out


def flight_sequence(mav, sub, osub, adiag, pdiag, aerodiag, R):
    confirmed = campaign.enter_fbwa(mav, R)
    if not confirmed:
        R["flight_result"] = dict(aborted=True, reason="fbwa_not_confirmed")
        return False
    latest_mav = {}
    t_flight0 = time.time()
    seg = campaign.run_segment(mav, sub, osub, adiag, pdiag, aerodiag,
                                SEGMENT_LABEL, SEGMENT_DURATION_S,
                                1500, 1500, base.RC3_TRIM_TARGET_US,
                                t_flight0, latest_mav)
    print(f"  segment {SEGMENT_LABEL}: dur={SEGMENT_DURATION_S}s rc3={base.RC3_TRIM_TARGET_US:.1f} "
          f"n_samples={seg['n_samples']} aborted={seg['aborted']}")
    R["segments"] = [seg]
    R["flight_result"] = dict(aborted=seg["aborted"], reason=seg["abort_reason"])
    if not seg["aborted"]:
        R["analysis"] = analyze(seg)
    return not seg["aborted"]


def finish_fail(R, phase, mav):
    R["overall_result"] = "TEST_FAILED"
    R["blocking_phase"] = phase
    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(R, f, indent=2, default=str)
    print(f"FAILED at {phase} - see", OUT_JSON)
    if mav is not None:
        mav.close()
    return 1


def main():
    R = {"stage": "ARDUPLANE_TRIM_REFERENCE_CORRECTION_VALIDATION",
         "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         "corrected_trim_constants": dict(
             TRIM_THROTTLE=base.TRIM_THROTTLE, ELEVATOR_THETA_RAD_deg=4.50,
             V_TRIM=base.V_TRIM, ALPHA_TRIM_DEG=base.ALPHA_TRIM_DEG,
             RC3_TRIM_TARGET_US=base.RC3_TRIM_TARGET_US,
             ELEV_RC2_TARGET_US=base.ELEV_RC2_TARGET_US),
         "prior_stale_trim_reference": dict(
             avg_sink_ms=PRIOR_STALE_TRIM_AVG_SINK_MS, range_ms=list(PRIOR_STALE_TRIM_RANGE_MS),
             source="docs/test_results/2026-08-28_ardupilot_pitch_pid_phugoid_baseline.md sec 6"),
         "segment_config": dict(label=SEGMENT_LABEL, duration_s=SEGMENT_DURATION_S,
                                 transient_cutoff_s=TRANSIENT_CUTOFF_S)}

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
    print("PHASE 1 (mavlink arm):", json.dumps(R.get("phase1_mavlink_arm", {}), default=str)[:400])
    if not armed:
        return finish_fail(R, "phase1_mavlink_arm", mav)

    settled, elapsed = base.wait_ground_settle(osub)
    R["ground_settle"] = dict(settled=settled, elapsed_s=elapsed)
    print("ground settle:", R["ground_settle"])
    if not settled:
        base.disarm(mav)
        return finish_fail(R, "ground_settle", mav)

    t_teleport, ok_v = base.phase2_teleport_and_verify(
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

    for msg_id in campaign.MSG_IDS_20HZ:
        campaign.request_rate(mav, msg_id, 20.0)
    time.sleep(0.3)

    print(f"Starting trim_reference_correction_validation ({SEGMENT_DURATION_S}s neutral, FBWA)...")
    ok = flight_sequence(mav, sub, osub, adiag, pdiag, aerodiag, R)

    mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    base.disarm(mav)

    R["overall_result"] = "FLIGHT_COMPLETED_NO_ABORT" if ok else "FLIGHT_ABORTED"
    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(R, f, indent=2, default=str)

    if ok and "analysis" in R:
        stab = R["analysis"]["stabilized_window"]
        print("STABILIZED WINDOW sink (regression):", stab.get("linear_regression_avg_sink_ms"),
              "m/s, (endpoint):", stab.get("endpoint_avg_sink_ms"), "m/s, n=", stab.get("n_samples"))
        print("PRIOR (stale trim) avg sink:", PRIOR_STALE_TRIM_AVG_SINK_MS, "m/s")

    print("RESULT:", R["overall_result"], "->", OUT_JSON)
    mav.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

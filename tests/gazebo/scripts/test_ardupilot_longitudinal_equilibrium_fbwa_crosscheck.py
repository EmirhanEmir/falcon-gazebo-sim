#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_LONGITUDINAL_EQUILIBRIUM_AND_SINK_ROOT_CAUSE_VALIDATION
Step 5 - Part C.3 FBWA (and MANUAL tie-breaker) cross-check (gazebo-testing,
2026-08-28).

MEASURE-AND-PROVE stage. NO change to any ArduPlane PID,
config/ardupilot/falcon_v2_sitl.parm, the real .param, actuator/sign mapping,
joint limits, any plugin source, or any SDF. This script only:
  - reads the pure-Gazebo sweep's converged (V*, throttle*, elevator*, alpha*)
    from tests/gazebo/results/ardupilot_longitudinal_equilibrium_sweep_result.json
    (produced by test_ardupilot_longitudinal_equilibrium_root_cause.py),
  - RUNTIME-retargets `base`'s own module-level PHASE-3 hold constants (U_HOLD /
    W_HOLD / ELEV_RC2_TARGET_US / RC3_TRIM_TARGET_US) to that point - this is
    test-script runtime configuration, NOT an edit to the imported module,
  - flies ONE 25-30 s neutral (FBWA) or fixed-elevator (MANUAL) segment,
  - measures the steady sink and compares it side-by-side against the prior
    0.55 m/s FBWA sink (ARDUPLANE_TRIM_REFERENCE_CORRECTION_VALIDATION sec 5).

REUSED, UNMODIFIED, AS LIBRARIES:
  - test_ardupilot_basic_closed_loop_flight (base)        : PHASE1-4 precondition
  - test_ardupilot_basic_closed_loop_flight_campaign (campaign) : enter_fbwa, run_segment, build_sample
  - test_ardupilot_trim_reference_correction_validation (trimref) : analyze / sink_stats / stat3

Usage (a FRESH gz sim + gdb-wrapped arduplane pair MUST already be running, per
the launch block in docs/test_results/2026-08-28_ardupilot_trim_reference_
correction_validation.md sec 3):

    python3 test_ardupilot_longitudinal_equilibrium_fbwa_crosscheck.py fbwa_throttlestar
    python3 test_ardupilot_longitudinal_equilibrium_fbwa_crosscheck.py fbwa_original
    python3 test_ardupilot_longitudinal_equilibrium_fbwa_crosscheck.py manual
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
import test_ardupilot_trim_reference_correction_validation as trimref  # noqa: E402
import aero_lib  # noqa: E402
import propulsion_lib  # noqa: E402
import actuator_lib  # noqa: E402

RESULTS_DIR = base.RESULTS_DIR
SWEEP_JSON = os.path.join(RESULTS_DIR, "ardupilot_longitudinal_equilibrium_sweep_result.json")
C1_JSON = os.path.join(RESULTS_DIR, "ardupilot_longitudinal_equilibrium_c1_reference_result.json")

SEGMENT_DURATION_S = 28.0
TRANSIENT_CUTOFF_S = 5.0

# Prior FBWA sink, cited not re-derived: ARDUPLANE_TRIM_REFERENCE_CORRECTION_
# VALIDATION sec 5 (25 s neutral FBWA at corrected trim throttle 0.5010 / RC3=1501).
PRIOR_FBWA_SINK_MS = 0.55
PRIOR_FBWA_SINK_SOURCE = "docs/test_results/2026-08-28_ardupilot_trim_reference_correction_validation.md sec 5"

# ArduPlane custom_mode values
FBWA_MODE = base.ARDUPLANE_FBWA_CUSTOM_MODE   # 5
MANUAL_MODE = 0

# Reference point (fallback if the sweep did not converge)
REF_V, REF_THROTTLE, REF_ELEV_PHYS_DEG, REF_ALPHA_DEG = 18.166, 0.5010, 4.50, 2.473

ELEV_MULT = base.ELEV_MULT          # 1.5707963268
ELEV_OFFSET = base.ELEV_OFFSET      # -0.5
ELEV_SERVO_MIN = base.ELEV_SERVO_MIN
ELEV_SERVO_MAX = base.ELEV_SERVO_MAX
ELEV_RC_A = base.ELEV_RC_A          # -1125.0
ELEV_RC_B = base.ELEV_RC_B          # 1.75


def rc2_for_elevator_phys_deg(theta_deg):
    """Generalised RC2 for a swept physical elevator angle - the Part A.1 formula,
    identical construction to base.ELEV_RC2_TARGET_US, just with theta as input."""
    theta_rad = math.radians(theta_deg)
    raw = theta_rad / ELEV_MULT - ELEV_OFFSET
    servo_pwm = ELEV_SERVO_MIN + raw * (ELEV_SERVO_MAX - ELEV_SERVO_MIN)
    return (servo_pwm - ELEV_RC_A) / ELEV_RC_B


def rc3_for_throttle(throttle):
    return round(1000.0 + throttle * 1000.0)


def load_star_point():
    """(V*, throttle*, elevator_phys*, alpha*, pitch_phys*, source)."""
    if os.path.exists(SWEEP_JSON):
        with open(SWEEP_JSON) as f:
            sw = json.load(f)
        node = sw.get("final_full_window") or None
        if node is None:
            # fall back to the last converged confirm run in the table
            for row in reversed(sw.get("run_table", [])):
                if row["label"].endswith("confirm") and row.get("accept_all"):
                    pass
        if node is not None:
            cmd = node["command"]
            ss = node["steady_state"]
            return (cmd["V_target"], cmd["throttle"], cmd["elevator_phys_deg"],
                    ss["alpha_tail_deg"]["mean"], ss["pitch_tail_deg"]["mean"],
                    "sweep final_full_window (converged, 60 s)")
        if sw.get("converged") and sw.get("thr_star") is not None:
            # reconstruct from elev_star/thr_star + the matching confirm run
            elev = sw["elev_star_deg"]
            thr = sw["thr_star"]
            V = sw["tried_V"][-1]
            for r in reversed(sw["runs"]):
                if r["label"].endswith("confirm"):
                    ss = r["steady_state"]
                    return (V, thr, elev, ss["alpha_tail_deg"]["mean"],
                            ss["pitch_tail_deg"]["mean"], "sweep converged confirm run (30 s)")
    print("WARNING: no converged sweep point found - falling back to the current reference point")
    return (REF_V, REF_THROTTLE, REF_ELEV_PHYS_DEG, REF_ALPHA_DEG, REF_ALPHA_DEG,
            "reference fallback (sweep did not converge)")


def retarget_base(V_star, alpha_star_deg, elevator_phys_deg, throttle_for_phase3):
    a = math.radians(alpha_star_deg)
    base.V_TRIM = V_star
    base.ALPHA_TRIM_DEG = alpha_star_deg
    base.ALPHA_TRIM_RAD = a
    base.U_HOLD = V_star * math.cos(a)
    base.W_HOLD = -V_star * math.sin(a)
    base.ELEV_RC2_TARGET_US = rc2_for_elevator_phys_deg(elevator_phys_deg)
    base.RC3_TRIM_TARGET_US = float(rc3_for_throttle(throttle_for_phase3))
    return dict(V_TRIM=base.V_TRIM, ALPHA_TRIM_DEG=base.ALPHA_TRIM_DEG,
               U_HOLD=base.U_HOLD, W_HOLD=base.W_HOLD,
               ELEV_RC2_TARGET_US=base.ELEV_RC2_TARGET_US,
               RC3_TRIM_TARGET_US=base.RC3_TRIM_TARGET_US)


def enter_mode(mav, mode, R, rc2, rc3):
    mav.m.mav.set_mode_send(mav.m.target_system, base.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
    mav.send_rc_override(rc1=1500, rc2=int(round(rc2)), rc3=int(round(rc3)), rc4=1500, rc5=1000)
    hb = None
    t0 = time.time()
    while time.time() - t0 < 5.0:
        r, _, _ = select.select([mav.m.port], [], [], 0.3)
        if not r:
            continue
        msg = mav.m.recv_match(type="HEARTBEAT", blocking=False)
        if msg is None:
            continue
        hb = msg
        if hb.custom_mode == mode:
            break
    confirmed = bool(hb and hb.custom_mode == mode)
    R["mode_handoff"] = dict(requested_mode=mode, confirmed=confirmed,
                             custom_mode=(hb.custom_mode if hb else None))
    return confirmed


def flight_sequence(run, mav, sub, osub, adiag, pdiag, aerodiag, R,
                    seg_rc1, seg_rc2, seg_rc3, mode):
    if mode == FBWA_MODE:
        confirmed = campaign.enter_fbwa(mav, R)   # reused verbatim
        R["mode_handoff"] = dict(requested_mode=mode, confirmed=confirmed)
    else:
        confirmed = enter_mode(mav, mode, R, seg_rc2, seg_rc3)
    if not confirmed:
        R["flight_result"] = dict(aborted=True, reason="mode_not_confirmed")
        return False
    latest_mav = {}
    t_flight0 = time.time()
    seg = campaign.run_segment(mav, sub, osub, adiag, pdiag, aerodiag,
                               f"{run}_segment", SEGMENT_DURATION_S,
                               seg_rc1, seg_rc2, seg_rc3, t_flight0, latest_mav)
    print(f"  segment {run}: dur={SEGMENT_DURATION_S}s rc1={seg_rc1} rc2={seg_rc2:.1f} rc3={seg_rc3} "
          f"n_samples={seg['n_samples']} aborted={seg['aborted']}")
    R["segments"] = [seg]
    R["flight_result"] = dict(aborted=seg["aborted"], reason=seg["abort_reason"])
    if not seg["aborted"]:
        R["analysis"] = trimref.analyze(seg)
    return not seg["aborted"]


def finish_fail(R, phase, mav, out_json):
    R["overall_result"] = "TEST_FAILED"
    R["blocking_phase"] = phase
    os.makedirs(RESULTS_DIR, exist_ok=True)
    with open(out_json, "w") as f:
        json.dump(R, f, indent=2, default=str)
    print(f"FAILED at {phase} - see {out_json}")
    if mav is not None:
        mav.close()
    return 1


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("fbwa_throttlestar", "fbwa_original", "manual"):
        print(f"usage: {sys.argv[0]} {{fbwa_throttlestar|fbwa_original|manual}}")
        return 2
    run = sys.argv[1]
    out_json = os.path.join(RESULTS_DIR, f"ardupilot_longitudinal_equilibrium_c3_{run}_result.json")

    V_star, thr_star, elev_star, alpha_star, pitch_star, src = load_star_point()

    if run == "fbwa_throttlestar":
        mode = FBWA_MODE
        seg_rc1, seg_rc2 = 1500, 1500
        seg_rc3 = rc3_for_throttle(thr_star)
        throttle_used = thr_star
    elif run == "fbwa_original":
        mode = FBWA_MODE
        seg_rc1, seg_rc2 = 1500, 1500
        seg_rc3 = rc3_for_throttle(REF_THROTTLE)   # RC3 = 1501, the original 0.5010
        throttle_used = REF_THROTTLE
    else:  # manual
        mode = MANUAL_MODE
        seg_rc1 = 1500
        seg_rc2 = rc2_for_elevator_phys_deg(elev_star)
        seg_rc3 = rc3_for_throttle(thr_star)
        throttle_used = thr_star

    R = {"stage": "ARDUPLANE_LONGITUDINAL_EQUILIBRIUM_AND_SINK_ROOT_CAUSE_VALIDATION - Part C.3",
         "run": run, "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         "star_point": dict(source=src, V_star=V_star, throttle_star=thr_star,
                            elevator_phys_star_deg=elev_star, alpha_star_deg=alpha_star,
                            pitch_phys_star_deg=pitch_star),
         "commanded": dict(mode=mode, seg_rc1=seg_rc1, seg_rc2=seg_rc2, seg_rc3=seg_rc3,
                           throttle_used=throttle_used,
                           rc2_maps_to_elevator_phys_deg=(elev_star if run == "manual" else 0.0)),
         "prior_fbwa_sink_ms": PRIOR_FBWA_SINK_MS, "prior_fbwa_sink_source": PRIOR_FBWA_SINK_SOURCE,
         "segment_config": dict(duration_s=SEGMENT_DURATION_S, transient_cutoff_s=TRANSIENT_CUTOFF_S)}

    # PHASE-3 hold is always retargeted to the star point (V*, alpha*, elevator*);
    # throttle during the hold itself is off (phase3 sends rc3=1000).
    R["phase3_retarget"] = retarget_base(V_star, alpha_star, elev_star, throttle_used)
    print("Star point:", json.dumps(R["star_point"], default=str))
    print("Commanded:", json.dumps(R["commanded"], default=str))
    print("PHASE-3 retarget:", json.dumps(R["phase3_retarget"], default=str))

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
    print("PHASE 1 (mavlink arm):", json.dumps(R.get("phase1_mavlink_arm", {}), default=str)[:300])
    if not armed:
        return finish_fail(R, "phase1_mavlink_arm", mav, out_json)

    settled, elapsed = base.wait_ground_settle(osub)
    R["ground_settle"] = dict(settled=settled, elapsed_s=elapsed)
    print("ground settle:", R["ground_settle"])
    if not settled:
        base.disarm(mav)
        return finish_fail(R, "ground_settle", mav, out_json)

    t_teleport, ok_v = base.phase2_teleport_and_verify(
        node, sub, osub, pub_oneshot, (0.0, 0.0, 90.0, 0.0, 0.0, 0.0), R)
    print("PHASE 2 (teleport+verify):", R["phase2_teleport_verify"]["ok"])
    if not ok_v:
        base.disarm(mav)
        return finish_fail(R, "phase2_teleport_verify", mav, out_json)

    base.clear_wrench(pub_clear)

    hold_ok = base.phase3_hold_to_trim(pub_oneshot, pub_clear, sub, osub, mav, R)
    print("PHASE 3 (hold-to-trim): aborted =", R["phase3_hold_to_trim"]["aborted"],
          "reason =", R["phase3_hold_to_trim"]["abort_reason"])
    if not hold_ok:
        mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
        base.disarm(mav)
        return finish_fail(R, "phase3_hold_to_trim", mav, out_json)

    for msg_id in campaign.MSG_IDS_20HZ:
        campaign.request_rate(mav, msg_id, 20.0)
    time.sleep(0.3)

    print(f"Starting Part C.3 {run} ({SEGMENT_DURATION_S}s, mode={mode})...")
    ok = flight_sequence(run, mav, sub, osub, adiag, pdiag, aerodiag, R,
                         seg_rc1, seg_rc2, seg_rc3, mode)

    mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    base.disarm(mav)

    R["overall_result"] = "FLIGHT_COMPLETED_NO_ABORT" if ok else "FLIGHT_ABORTED"
    os.makedirs(RESULTS_DIR, exist_ok=True)
    with open(out_json, "w") as f:
        json.dump(R, f, indent=2, default=str)

    if ok and "analysis" in R:
        stab = R["analysis"]["stabilized_window"]
        print(f"STABILIZED WINDOW (t>={TRANSIENT_CUTOFF_S}s) sink: "
              f"regression={stab.get('linear_regression_avg_sink_ms')} m/s  "
              f"endpoint={stab.get('endpoint_avg_sink_ms')} m/s  n={stab.get('n_samples')}")
        print(f"PRIOR FBWA sink (corrected-trim reference): {PRIOR_FBWA_SINK_MS} m/s")
    print("RESULT:", R["overall_result"], "->", out_json)
    mav.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_PITCH_PID_AND_PHUGOID_TUNING_VALIDATION (gazebo-
testing, 2026-08-28). Short, bounded follow-up to
ARDUPLANE_BASIC_CLOSED_LOOP_FLIGHT_VALIDATION (READY). ONE baseline
characterization flight with a SLOWER command cadence than the prior
stage's Flight 3, specifically designed to separate genuine phugoid
behavior from cadence-forced resonance. The prior stage's own analysis
(docs/test_results/2026-08-28_ardupilot_basic_closed_loop_flight_
validation.md sec 11.11/11.12) found Flight 3's 3-4s neutral-hold cadence
was comparable to or shorter than the longitudinal mode's own apparent
~7-8s period, so each new RC2 step was injected before the previous
transient had settled - this run's whole point is to let it actually
settle before commanding again.

No AUTOTUNE, no TECS tuning, no LOITER/AUTO/RTL, no PID changes (that is a
possible follow-up decision for `controls-integration`, using this run's
data - not something this script does).

=============================================================================
REUSED, UNMODIFIED, AS LIBRARIES (no re-derivation)
=============================================================================
  - test_ardupilot_basic_closed_loop_flight.py (`base`): the already-proven
    PHASE1-4 precondition mechanism - phase1_mavlink_arm (MAVLink connect/
    readiness/arm), wait_ground_settle, phase2_teleport_and_verify
    (teleport to the airborne test pose with the zero-gap gravity-
    feedforward fix), phase3_hold_to_trim (real, external force/torque
    controller driving body velocity to the trim condition
    V_TRIM=18.166 m/s / ALPHA_TRIM_DEG=2.473 (docs/test_results/2026-08-26_
    updated_powered_trim_high_deflection_validation.md re-derived trim,
    picked up automatically via base.py's own corrected constants -
    ARDUPLANE_TRIM_REFERENCE_CORRECTION_VALIDATION), elevator held at its
    own trim deflection via a real RC2 override, released with zero further
    intervention), PoseSub/OdomSub, quat_to_rpy, set_pose, disarm,
    clear_wrench, and every already-documented constant/gain
    (KP_LIN=150/KP_ANG=100, VEL_SANITY_MAX_MS, ELEV_RC2_TARGET_US,
    RC3_TRIM_TARGET_US - the ASSUMPTION-labeled linear RC3 throttle
    mapping used for the trim-throttle command, see base.py's own
    derivation comment).
  - test_ardupilot_basic_closed_loop_flight_campaign.py (`campaign`): the
    already-proven FBWA RC1/RC2 command formulas (rc2_for_pitch_deg -
    live-verified ROLL_LIMIT_DEG=45.0/PITCH_LIM_MAX_DEG=20.0/
    PITCH_LIM_MIN_DEG_MAG=25.0, RC2_pwm = 1500 + 400*norm_input),
    enter_fbwa (mode-switch/confirm, "wait for the LAST heartbeat with
    custom_mode==FBWA, not the first"), run_segment (fixed-RC-for-a-fixed-
    duration execution with 20 Hz combined MAVLink+gz-transport telemetry
    capture and the same hard safety envelope as base.py:
    |roll_gz|/|pitch_gz| > 80deg, altitude < 5m, or non-finite attitude ->
    immediate abort with all samples gathered so far preserved),
    build_sample (full per-sample telemetry: MAVLink mode/ATTITUDE/
    NAV_CONTROLLER_OUTPUT/SERVO_OUTPUT_RAW/VFR_HUD/GLOBAL_POSITION_INT
    cross-referenced against gz-transport ground-truth pose/odometry, all
    5 actuator-surface joint diagnostics via actuator_lib.DiagSubscriber,
    both-motor propulsion diagnostics via propulsion_lib.DiagSubscriber,
    aero diagnostics via aero_lib.DiagSubscriber), MSG_IDS_20HZ,
    request_rate.

NEW in this file (the whole point of this stage): only the command
SEQUENCE/TIMING below (`SEQUENCE`) - no new transport/telemetry plumbing,
no bounded_ok()-style online escalation (this stage is a single fixed
sequence at one magnitude, not an escalating campaign), no PID changes.

Body-frame sign-convention note (already documented and live-confirmed in
the prior stage, cited not re-derived): ArduPilotPlugin's compiled-default
180deg-about-X body-frame override means roll_mav ~= +roll_gz,
pitch_mav ~= -pitch_gz. Both MAVLink and Gazebo ground-truth attitude are
captured per-sample by build_sample() for direct cross-check in the report.

No aircraft-physics parameter (mass, CG, inertia, aerodynamic coefficient,
control authority, motor thrust, PID gain) is read for any purpose other
than citing already-published/live-confirmed values, and none is modified
anywhere in this file. model/model.sdf, falcon_v2_sitl.parm, the real
.param, and every plugin under plugins/ are unmodified.

=============================================================================
SEQUENCE (per task instruction - deliberately slower cadence than the
prior stage's flight3, ~47s total)
=============================================================================
  1. PHASE1-4 precondition (arm -> teleport -> hold-to-trim -> FBWA
     release), reused as-is from base.py.
  2. neutral_stabilization  10.0s  RC1/RC2=1500/1500 (nominal throttle)
  3. pitch_up_5              5.0s  RC2 = campaign.rc2_for_pitch_deg(+5.0)
  4. neutral_settle_1       12.0s  RC1/RC2=1500/1500  <- the critical
     difference from the prior stage's 3-4s cadence: let the mode actually
     settle before the next command.
  5. pitch_down_5            5.0s  RC2 = campaign.rc2_for_pitch_deg(-5.0)
  6. neutral_settle_2       15.0s  RC1/RC2=1500/1500 (final settle window)

Usage (Gazebo + arduplane already running, a FRESH pair, per the proven
launch sequence in the prior stage's report sec 11.2):
    python3 test_ardupilot_pitch_pid_phugoid_baseline.py
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

OUT_JSON = os.path.join(base.RESULTS_DIR, "ardupilot_pitch_pid_phugoid_baseline_result.json")

# (label, duration_s, desired_pitch_deg) - desired_pitch_deg=0.0 maps through
# campaign.rc2_for_pitch_deg() to exactly RC2=1500 (neutral), so the same
# formula is used uniformly for every segment, commanded or neutral.
SEQUENCE = [
    ("neutral_stabilization", 10.0, 0.0),
    ("pitch_up_5", 5.0, 5.0),
    ("neutral_settle_1", 12.0, 0.0),
    ("pitch_down_5", 5.0, -5.0),
    ("neutral_settle_2", 15.0, 0.0),
]


def flight_sequence(mav, sub, osub, adiag, pdiag, aerodiag, R):
    confirmed = campaign.enter_fbwa(mav, R)
    if not confirmed:
        R["flight_result"] = dict(aborted=True, reason="fbwa_not_confirmed")
        return False
    latest_mav = {}
    t_flight0 = time.time()
    segments = []
    for label, dur, pitch in SEQUENCE:
        rc2 = campaign.rc2_for_pitch_deg(pitch)
        seg = campaign.run_segment(mav, sub, osub, adiag, pdiag, aerodiag, label, dur,
                                    1500, rc2, base.RC3_TRIM_TARGET_US, t_flight0, latest_mav)
        seg["commanded_pitch_deg"] = pitch
        segments.append(seg)
        print(f"  segment {label}: dur={dur}s commanded_pitch={pitch}deg rc2={rc2:.1f} "
              f"n_samples={seg['n_samples']} aborted={seg['aborted']}")
        if seg["aborted"]:
            break
    R["segments"] = segments
    aborted_any = any(s["aborted"] for s in segments)
    R["flight_result"] = dict(aborted=aborted_any)
    return not aborted_any


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
    R = {"stage": "ARDUPLANE_PITCH_PID_AND_PHUGOID_TUNING_VALIDATION",
         "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         "sequence_config": SEQUENCE}

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

    print("Starting pitch_pid_phugoid_baseline command sequence (FBWA)...")
    ok = flight_sequence(mav, sub, osub, adiag, pdiag, aerodiag, R)

    mav.send_rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500, rc5=1000)
    base.disarm(mav)

    R["overall_result"] = "FLIGHT_COMPLETED_NO_ABORT" if ok else "FLIGHT_ABORTED"
    os.makedirs(base.RESULTS_DIR, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(R, f, indent=2, default=str)
    seg_summary = [(s["label"], s["aborted"], s["n_samples"]) for s in R.get("segments", [])]
    print("RESULT:", R["overall_result"], "segments:", seg_summary, "->", OUT_JSON)
    mav.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

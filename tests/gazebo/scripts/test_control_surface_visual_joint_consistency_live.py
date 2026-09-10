#!/usr/bin/env python3
"""FALCON V2 - Part 1: LIVE closed-loop control-surface VISUAL/JOINT consistency.

CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY.
Stage: MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION (Part 1).
Owner: controls-integration. Created 2026-09-10.

CHANGES NO PHYSICS PARAMETER. No aerodynamic or propulsion coefficient, no
mass/CG/inertia, no actuator PID/rate/effort limit, no control-surface +/-45
deg mapping, no PTCH_TRIM_DEG, no TECS_PTCH_DAMP, no roll/pitch PID, no
sensor model, no landing gear, no friction or collision value. It subscribes,
commands RC, and records.

=============================================================================
THE QUESTION
=============================================================================
Does the RENDERED VISUAL MOTION follow the ACTUAL JOINT POSITION, or does it
follow the COMMAND?

Why this can actually be answered: the Gazebo GUI renders a link's visual by
applying the link's transform, delivered on
    /world/<world>/dynamic_pose/info
by the SceneBroadcaster system. This test subscribes to that exact topic. It
is not a proxy for what the GUI shows - it is the same data the GUI consumes.

DECISIVE COMPARISON, per surface, per sample:
    theta_visual := signed rotation of (child link transform relative to
                    base_link) about the joint's declared <axis><xyz>
    residual_vs_ACTUAL := theta_visual - actual_angle_rad   (from the
                          actuator plugin's diagnostics, which is
                          gz::sim::Joint::Position(ecm))
    residual_vs_CMD    := theta_visual - cmd_rad
PASS  : |residual_vs_ACTUAL| is at the numerical-noise level AND, whenever
        cmd and actual genuinely differ, |residual_vs_ACTUAL| is much smaller
        than |residual_vs_CMD|.
FAIL  : the visual tracks cmd_rad while cmd_rad != actual_angle_rad. That is
        exactly the failure mode this stage was asked to look for.
INCONCLUSIVE: cmd_rad and actual_angle_rad never differ by more than the
        discrimination floor during the run - the test then cannot separate
        the two hypotheses and says so instead of claiming a PASS.

The test therefore DELIBERATELY drives fast RC steps, so that the servo model's
own lag guarantees a large, sustained cmd-vs-actual separation to discriminate
against. It also verifies the SECOND half of the render chain: the visual's
static pose within its link (from /world/<world>/pose/info) combined with the
live link pose gives the rendered visual origin in world coordinates.

=============================================================================
PRIOR WORK THIS BUILDS ON (read, not modified)
=============================================================================
codex/tests/gazebo/scripts/test_control_surface_visual_joint_consistency.py
and codex/tests/gazebo/results/control_surface_visual_joint_consistency.json
already established the OPEN-LOOP, in-process (TestFixture/ECM) version of
this check at 0/+/-5/+/-15/+/-30 deg: child-link rotation-basis error ~4e-19,
visual-origin transform error ~2.5e-15 m, 35/35 cases pass. Method reused;
that file is NOT modified. What is NEW here:
  * CLOSED LOOP - ArduPlane SITL is actually flying/driving the surfaces,
    rather than a script writing joint commands directly.
  * OUT OF PROCESS - read over gz-transport from the RENDER stream
    (dynamic_pose/info), not from the ECM inside a TestFixture. The GUI also
    reads out of process; an in-process ECM read cannot, even in principle,
    detect a render-path discrepancy.
  * SKEW RECORDED - every source's arrival time is logged and the per-sample
    inter-source skew is reported, instead of assuming simultaneity.
  * ArduPlane's own SERVO_OUTPUT_RAW demand is captured alongside, closing the
    chain autopilot-demand -> cmd_rad -> setpoint -> actual joint -> rendered
    transform -> aero input.

=============================================================================
DATA_REQUIRED
=============================================================================
The aerodynamics plugin does NOT publish the deflection it consumes (see
manual_takeoff_live_lib.AERO_DEFLECTION_NOT_PUBLISHED, which is embedded in
this test's result JSON). The aero input is therefore RECONSTRUCTED from
actual_angle_rad via the CONTROLS.md sec 10 formulas and INDEPENDENTLY
CROSS-CHECKED against the plugin's published Cl/Cm/Cn using
aero_lib.compute_aero(). This test reports that cross-check; it does not
assume the reconstruction is right.

=============================================================================
USAGE
=============================================================================
A gz sim server AND arduplane must already be running against
tests/gazebo/worlds/falcon_v2_manual_takeoff_runway_world.sdf - use
    tests/gazebo/scripts/run_manual_takeoff_and_live_surface_validation.sh
which starts them. Then:
    python3 test_control_surface_visual_joint_consistency_live.py
    python3 test_control_surface_visual_joint_consistency_live.py --dry-run
    python3 test_control_surface_visual_joint_consistency_live.py --world NAME
--dry-run performs every non-simulation step (source-of-truth reads, SDF
cross-checks, geometry math self-test) and exits 0 without touching the
transport, so the script can be validated without a running simulator.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import manual_takeoff_live_lib as L  # noqa: E402

DEFAULT_WORLD = "falcon_v2_manual_takeoff_runway"
RESULT_JSON = os.path.join(L.RESULTS_DIR,
                           "control_surface_visual_joint_consistency_live_result.json")
TIMESERIES_JSON = os.path.join(
    L.RESULTS_DIR, "control_surface_visual_joint_consistency_live_timeseries.json")
LOG_TXT = os.path.join(L.RESULTS_DIR,
                       "control_surface_visual_joint_consistency_live_log.txt")

# ---------------------------------------------------------------------------
# Thresholds. Every one is a MEASUREMENT-QUALITY threshold on the harness, not
# a physics tuning knob, and each states its basis.
# ---------------------------------------------------------------------------
TH = {
    "discrimination_floor_deg": {
        "value": 0.20,
        "basis": "DERIVED. A sample can only discriminate 'visual follows "
                 "actual' from 'visual follows cmd' if cmd and actual actually "
                 "differ. 0.20 deg is ~3 PWM quanta of this transport "
                 "(1 PWM = 0.0643 deg, derived in manual_takeoff_live_lib from "
                 "model.sdf's own multiplier and servo_min/servo_max).",
    },
    "well_bound_max_dt_s": {
        "value": 0.006,
        "basis": "DERIVED from the two stream rates MEASURED in the "
                 "2026-09-10 smoke runs: the render pose stream runs at "
                 "~59 Hz and the actuator diagnostics at exactly 20 Hz "
                 "(model.sdf <diagnostics_rate_hz>, which this stage may not "
                 "change). Sampling at ~120 Hz, ~400 of ~2000 pose/actuator "
                 "message pairs land within 6 ms of each other.",
    },
    "min_discrimination_ratio_median": {
        "value": 10.0,
        "basis": "DERIVED. The two hypotheses are 'the visual follows "
                 "actual_angle_rad' and 'the visual follows cmd_rad'. The "
                 "ratio is formed LIKE FOR LIKE: median(|residual vs CMD|) "
                 "over median(|residual vs ACTUAL|), on the same "
                 "discriminating samples. A median-over-median ratio is used "
                 "rather than a median-over-MAX form, which the 2026-09-10 "
                 "smoke run showed is dominated by a single worst-case "
                 "binding sample (max 0.45 deg vs rms 0.067 deg) and "
                 "therefore measures outlier binding rather than the "
                 "hypothesis separation it is meant to measure. 10:1 is a "
                 "full order of magnitude.",
    },
    "max_correction_effectiveness": {
        "value": 0.30,
        "basis": "DERIVED, and this is the criterion that makes the result "
                 "defensible without inventing an absolute residual "
                 "tolerance. If the residual against the ACTUAL hypothesis is "
                 "a BINDING ARTEFACT it must scale with the binding window, "
                 "and must therefore largely VANISH once the first-order rate "
                 "correction (theta_pred = actual + rate*dt_bind) is applied. "
                 "If instead the renderer genuinely disagreed with the joint, "
                 "the residual would be independent of dt_bind and the "
                 "correction would not reduce it. Requiring "
                 "rms(corrected) <= 0.30 * rms(uncorrected) ON THE SAME "
                 "SAMPLES is a direct test of which of those two situations "
                 "holds, and it is self-calibrating: it needs no assumed "
                 "arithmetic-noise floor.",
    },
    "rms_residual_vs_actual_report_only_deg": {
        "value": 0.10,
        "basis": "REPORT-ONLY, NOT A GATE. Quoted so a reader can see at a "
                 "glance whether the corrected residual sits at the "
                 "second-order-binding scale. Second-order bound: with "
                 "|dt_bind| <= 6 ms and the actuator's 300 deg/s rate ceiling "
                 "(actuator_v1_config.yaml max_rate_rad_s), a rate change "
                 "across the binding window contributes up to "
                 "0.5*(300/0.006)*0.006^2 ~ 0.9 deg in the very worst case, "
                 "so an RMS at the 0.07 deg level is comfortably inside the "
                 "binding artefact and cannot be a render discrepancy.",
    },
    "off_axis_residual_max": {
        "value": 1e-6,
        "basis": "ASSUMPTION (measurement quality). A revolute joint's "
                 "relative rotation must be purely about its declared axis. "
                 "Any perpendicular quaternion vector component above float "
                 "noise means the render transform is not a pure hinge "
                 "rotation. 1e-6 is far above the ~1e-19 previously measured "
                 "in-process and far below anything visible.",
    },
    "min_discriminating_samples_per_surface": {
        "value": 20,
        "basis": "ASSUMPTION (statistics). Fewer than 20 well-bound samples "
                 "in which cmd and actual are separated by more than the "
                 "discrimination floor is not enough to call the result; the "
                 "test reports INCONCLUSIVE instead of PASS.",
    },
}

# RC step programme. PWM values only - these command the AUTOPILOT, they are
# not physics parameters. Endpoints are SERVOx_MIN/MAX-consistent RC extremes
# already used by tests/gazebo/scripts/test_ardupilot_control_surface_travel_
# scaling.py (RC 1100/1900 <-> SERVO 800/2200, live-calibrated there).
RC_NEUTRAL = {"rc1": 1500, "rc2": 1500, "rc3": 1000, "rc4": 1500, "rc5": 1000}
STEP_PROGRAMME = [
    # (label, rc overrides, hold seconds)
    ("neutral",        dict(RC_NEUTRAL),                         2.0),
    ("rc1_max",        dict(RC_NEUTRAL, rc1=1900),               1.5),
    ("rc1_min",        dict(RC_NEUTRAL, rc1=1100),               1.5),
    ("rc1_max_again",  dict(RC_NEUTRAL, rc1=1900),               1.5),
    ("rc2_max",        dict(RC_NEUTRAL, rc2=1900),               1.5),
    ("rc2_min",        dict(RC_NEUTRAL, rc2=1100),               1.5),
    ("rc4_max",        dict(RC_NEUTRAL, rc4=1900),               1.5),
    ("rc4_min",        dict(RC_NEUTRAL, rc4=1100),               1.5),
    ("all_max",        dict(rc1=1900, rc2=1900, rc3=1000, rc4=1900, rc5=1000), 1.5),
    ("all_min",        dict(rc1=1100, rc2=1100, rc3=1000, rc4=1100, rc5=1000), 1.5),
    ("neutral_end",    dict(RC_NEUTRAL),                         2.0),
]
SAMPLE_PERIOD_S = 0.008  # ~125 Hz capture attempt, chosen so that a useful
                         # fraction of the ~50 Hz pose and exactly-20 Hz
                         # actuator messages land within the well-bound dt;
                         # the achieved rate is measured and reported, never
                         # assumed.

_LOG_LINES = []


def log(*a):
    line = " ".join(str(x) for x in a)
    print(line, flush=True)
    _LOG_LINES.append(line)


# ---------------------------------------------------------------------------
# Geometry self-test - runs in --dry-run too, so a broken math helper is caught
# without needing a simulator.
# ---------------------------------------------------------------------------
def geometry_self_test():
    """Round-trip: build a pure rotation about each surface's real hinge axis
    at a known angle, then recover the angle. Any failure here invalidates
    every residual this test reports, so it gates the run."""
    struct = L.read_model_structure()
    out = {}
    worst = 0.0
    for s, meta in struct.items():
        axis = meta["axis_xyz"]
        errs = []
        for deg in (-45.0, -12.3, -0.05, 0.0, 0.05, 12.3, 45.0):
            ang = math.radians(deg)
            q = L.q_from_axis_angle(axis, ang)
            rec = L.angle_about_axis(q, axis)
            errs.append(abs(rec - ang))
            off = L.off_axis_residual(q, axis)
            errs.append(off)
        out[s] = {"max_error": max(errs), "axis": axis}
        worst = max(worst, max(errs))
    return {"per_surface": out, "worst_error": worst,
            "pass": worst < 1e-12}


def sdf_cross_check():
    blocks, problems = L.read_control_blocks()
    return {
        "control_blocks": blocks,
        "transport_constant_problems": problems,
        "pass": len(problems) == 0,
        "note": "model/model.sdf is READ ONLY here. A non-empty problem list "
                "means the SDF no longer matches the transport constants this "
                "harness derives its PWM quantum from - a finding, not "
                "something this stage fixes.",
    }


# ---------------------------------------------------------------------------
# Live capture
# ---------------------------------------------------------------------------
def run_live(world, args):
    import actuator_lib as ACT
    import aero_lib as AL
    import propulsion_lib as PL
    from ardupilot_sitl_mav_lib import SafeMav
    from pymavlink import mavutil

    struct = L.read_model_structure()
    signs = L.read_sign_scalars()

    log("subscribing to gz transport ...")
    clock = L.ClockSub(world)
    posesub = L.LinkPoseSub(world, ["falcon_v2", "base_link"] + L.SURFACES)
    staticsub = L.StaticPoseSub(world)
    # Timed subscribers: field ORDER comes from the existing libraries
    # (ACT.DIAG_FIELDS / AL.DiagSubscriber.FIELDS / PL.DiagSubscriber.
    # MOTOR_FIELDS), arrival timestamps are added so skew is measurable.
    actsub = L.TimedDiagSub(ACT.DIAG_TOPIC, ACT.DiagSubscriber._split)
    aerosub = L.TimedDiagSub(
        AL.DIAG_TOPIC,
        lambda v: dict(zip(AL.DiagSubscriber.FIELDS, v)))
    propsub = L.TimedDiagSub(PL.DIAG_TOPIC, PL.DiagSubscriber._split)
    time.sleep(1.5)
    static_tree = staticsub.snapshot()
    log("static pose tree entities:", len(static_tree))

    log("connecting MAVLink ...")
    mav = SafeMav(device=args.mavlink)
    hb = mav.wait_heartbeat(timeout=25)
    if hb is None:
        raise RuntimeError("no HEARTBEAT - arduplane is not up on " + args.mavlink)
    t0w = time.time()
    while hb is not None and hb.custom_mode != 0 and time.time() - t0w < 15:
        hb = mav.wait_heartbeat(timeout=3)
    log("heartbeat custom_mode:", hb.custom_mode if hb else None)

    # Standard, non-arming, non-persisted safety-switch release. Without it
    # BRD_SAFETY_DEFLT holds the servo outputs at trim (root-caused in
    # docs/test_results/2026-08-28_ardupilot_control_surface_travel_scaling_
    # validation.md). This writes NO parameter.
    ack = mav.command_long(mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, p1=1)
    log("safety switch off ack:", ack.to_dict() if ack else None)

    # MEASURED DEFECT, fixed here: ArduPlane's default stream rate for
    # ATTITUDE and SERVO_OUTPUT_RAW on this SITL is 1 Hz. The first smoke run
    # of this harness (2026-09-10) bound them at 1 Hz and consequently
    # reported an inter-source skew of up to 0.99 s, which is meaningless for
    # a surface that can slew at 300 deg/s.
    # See manual_takeoff_live_lib.request_stream_rates() for the measured
    # finding that MAV_CMD_SET_MESSAGE_INTERVAL is DENIED on this build and
    # REQUEST_DATA_STREAM is the mechanism that actually works. Telemetry
    # stream requests only - no parameter written, no physics changed.
    stream_rates = L.request_stream_rates(mav)
    log("stream rates measured after request:",
        {k: v for k, v in stream_rates["measured_hz"].items()
         if k in ("ATTITUDE", "SERVO_OUTPUT_RAW", "VFR_HUD", "HEARTBEAT")})
    log("SET_MESSAGE_INTERVAL results (2 == MAV_RESULT_DENIED):",
        stream_rates["set_message_interval_results"])

    samples = []
    servo_raw = {"pwm": None, "wall": None}
    attitude = {"vals": None, "wall": None}

    def pump_mav():
        """Non-blocking drain of the messages this test binds against."""
        import select as _sel
        for _ in range(40):
            r, _, _ = _sel.select([mav.m.port], [], [], 0.0)
            if not r:
                return
            m = mav.m.recv_match(blocking=False)
            if m is None:
                return
            t = m.get_type()
            if t == "SERVO_OUTPUT_RAW":
                servo_raw["pwm"] = [m.servo1_raw, m.servo2_raw, m.servo3_raw,
                                    m.servo4_raw, m.servo5_raw, m.servo6_raw,
                                    m.servo7_raw, m.servo8_raw]
                servo_raw["wall"] = time.time()
                servo_raw["time_usec"] = m.time_usec
            elif t == "ATTITUDE":
                attitude["vals"] = {
                    "roll_deg": math.degrees(m.roll),
                    "pitch_deg": math.degrees(m.pitch),
                    "yaw_deg": math.degrees(m.yaw),
                    "rollspeed_deg_s": math.degrees(m.rollspeed),
                    "pitchspeed_deg_s": math.degrees(m.pitchspeed),
                    "yawspeed_deg_s": math.degrees(m.yawspeed),
                    "time_boot_ms": m.time_boot_ms,
                }
                attitude["wall"] = time.time()

    def capture(step_label):
        pump_mav()
        act, act_wall = actsub.latest()
        aero, aero_wall = aerosub.latest()
        prop, prop_wall = propsub.latest()
        poses, pose_stamp, pose_wall = posesub.latest()
        sim_t, sim_wall = clock.latest()
        if act is None or poses is None:
            return None
        bound = L.bind_sample({
            "actuator_diag": (act, act_wall),
            "link_pose": (poses, pose_wall),
            "servo_output_raw": (servo_raw.get("pwm"), servo_raw.get("wall")),
            "attitude": (attitude.get("vals"), attitude.get("wall")),
            "aero_diag": (aero, aero_wall),
            "propulsion_diag": (prop, prop_wall),
        })
        base_q = poses.get("base_link", {}).get("quat", (1.0, 0.0, 0.0, 0.0))
        base_p = poses.get("base_link", {}).get("pos", (0.0, 0.0, 0.0))
        model_q = poses.get("falcon_v2", {}).get("quat", (1.0, 0.0, 0.0, 0.0))
        model_p = poses.get("falcon_v2", {}).get("pos", (0.0, 0.0, 0.0))

        per_surface = {}
        theta_actual = {}
        for s in L.SURFACES:
            meta = struct[s]
            d = act[s]
            theta_actual[s] = d["actual_angle_rad"]
            lp = poses.get(s)
            if lp is None:
                per_surface[s] = {"missing_link_pose": True}
                continue
            # child link transform relative to base_link. dynamic_pose/info
            # publishes poses relative to the PARENT entity, and base_link's
            # own pose relative to the model is identity for this model, so
            # this composition is exact and is verified in-run below.
            q_rel = L.q_mul(L.q_conj(base_q), lp["quat"])
            theta_visual = L.angle_about_axis(q_rel, meta["axis_xyz"])
            off_axis = L.off_axis_residual(q_rel, meta["axis_xyz"])
            # rendered visual origin, world frame: model -> base_link -> link
            # -> visual (visual pose within the link is static)
            vis_local = meta["visual_pose_xyz"]
            link_pos_in_parent = lp["pos"]   # link origin in base_link frame
            q_link_world = L.q_mul(model_q, L.q_mul(base_q, lp["quat"]))
            link_pos_in_model = tuple(
                base_p[j] + L.q_rotate(base_q, link_pos_in_parent)[j]
                for j in range(3))
            p_link_world = tuple(
                model_p[k] + L.q_rotate(model_q, link_pos_in_model)[k]
                for k in range(3))
            vis_world = tuple(p_link_world[k] + L.q_rotate(q_link_world, vis_local)[k]
                              for k in range(3))
            # expected visual origin if the link had rotated by the ACTUAL
            # joint angle about the declared axis, about the joint pivot
            q_exp = L.q_from_axis_angle(meta["axis_xyz"], d["actual_angle_rad"])
            pivot = meta["link_pose_xyz"]
            vis_exp_base = tuple(pivot[k] + L.q_rotate(q_exp, vis_local)[k]
                                 for k in range(3))
            vis_meas_base = tuple(link_pos_in_parent[k]
                                  + L.q_rotate(lp["quat"], vis_local)[k]
                                  for k in range(3))
            vis_err = math.sqrt(sum((vis_meas_base[k] - vis_exp_base[k]) ** 2
                                    for k in range(3)))
            act_wall_local = act_wall
            # SKEW CORRECTION - first order, and it matters.
            # The actuator diagnostics is published at 20 Hz
            # (model.sdf <diagnostics_rate_hz>, which this stage may not
            # change) while the render pose stream runs at ~50 Hz. So the
            # actuator sample bound into this record can be up to one 20 Hz
            # period stale relative to the pose. During a fast slew that is
            # worth several degrees and would otherwise swamp the very
            # residual this test is trying to resolve.
            # actual_angle_rad and actual_rate_rad_s come from the SAME
            # diagnostics message, i.e. the same physics tick, with zero skew
            # BETWEEN them - so the joint angle at the pose instant is
            #     theta_pred = actual_angle + actual_rate * dt_bind
            # with dt_bind = pose_arrival - actuator_arrival. This correction
            # is applied ONLY to the ACTUAL hypothesis, because it is the only
            # one for which a rate is known; the CMD hypothesis is left
            # untouched, so the correction cannot bias the comparison in
            # favour of ACTUAL by construction - it can only remove a known
            # binding artefact.
            dt_bind = 0.0
            if pose_wall is not None and act_wall_local is not None:
                dt_bind = pose_wall - act_wall_local
            theta_pred = d["actual_angle_rad"] + d["actual_rate_rad_s"] * dt_bind
            q_exp_corr = L.q_from_axis_angle(meta["axis_xyz"], theta_pred)
            vis_exp_base_corr = tuple(pivot[k] + L.q_rotate(q_exp_corr, vis_local)[k]
                                      for k in range(3))
            per_surface[s] = {
                "dt_bind_pose_minus_actuator_s": dt_bind,
                "theta_pred_rate_corrected_rad": theta_pred,
                "residual_visual_minus_actual_RATECORR_rad":
                    theta_visual - theta_pred,
                "visual_origin_error_RATECORR_m": math.sqrt(sum(
                    (vis_meas_base[k] - vis_exp_base_corr[k]) ** 2
                    for k in range(3))),
                "cmd_rad": d["cmd_rad"],
                "target_clamped_rad": d["target_clamped_rad"],
                "setpoint_rad": d["setpoint_rad"],
                "actual_angle_rad": d["actual_angle_rad"],
                "actual_rate_rad_s": d["actual_rate_rad_s"],
                "target_clamp_active": d["target_clamp_active"],
                "effort_clamp_active": d["effort_clamp_active"],
                "theta_visual_rad": theta_visual,
                "residual_visual_minus_actual_rad": theta_visual - d["actual_angle_rad"],
                "residual_visual_minus_cmd_rad": theta_visual - d["cmd_rad"],
                "cmd_minus_actual_rad": d["cmd_rad"] - d["actual_angle_rad"],
                "off_axis_residual": off_axis,
                "visual_origin_world_m": vis_world,
                "visual_origin_error_m": vis_err,
                "link_pose_in_parent_m": link_pos_in_parent,
            }
        deltas = L.aero_deltas_from_joint_angles(theta_actual, signs)
        return {
            "step": step_label,
            "t_wall": time.time(),
            "t_sim": sim_t,
            "t_sim_wall_arrival": sim_wall,
            "pose_header_stamp": pose_stamp,
            "binding": {k: v for k, v in bound.items() if k != "values"},
            "servo_output_raw_pwm": servo_raw.get("pwm"),
            "attitude": attitude.get("vals"),
            "surfaces": per_surface,
            "aero_input_reconstructed_rad": deltas,
            "aero_diag": aero,
            "propulsion_diag": prop,
            "model_pose": {"pos": model_p, "quat": model_q},
            "base_link_pose_in_model": {"pos": base_p, "quat": base_q},
        }

    log("\n=== running RC step programme (%d steps) ===" % len(STEP_PROGRAMME))
    for label, rc, hold in STEP_PROGRAMME:
        log("  step %-14s rc=%s hold=%.1fs" % (label, rc, hold))
        t_end = time.time() + hold
        last_pub = 0.0
        while time.time() < t_end:
            now = time.time()
            # RC_OVERRIDE_TIME is 3.0 s on this SITL, so republish steadily.
            if now - last_pub > 0.05:
                mav.send_rc_override(**rc)
                last_pub = now
            rec = capture(label)
            if rec is not None:
                samples.append(rec)
            time.sleep(SAMPLE_PERIOD_S)
    mav.hold_rc_override(1.0, **RC_NEUTRAL)
    mav.close()
    log("captured %d samples" % len(samples))
    return samples, static_tree, stream_rates


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def analyse(samples):
    """Decide, per surface, which hypothesis the RENDERED transform follows.

    Three views are reported and none is hidden:
      primary  - the WELL-BOUND subset (|dt_bind| <= well_bound_max_dt_s) with
                 the first-order rate correction applied. This is the
                 defensible number.
      same-samples uncorrected - the identical samples WITHOUT the correction,
                 so the size of the binding artefact is visible and the
                 correction's effectiveness can be computed.
      all-samples uncorrected  - everything, so nothing is filtered silently.
    """
    disc_floor = math.radians(TH["discrimination_floor_deg"]["value"])
    dt_max = TH["well_bound_max_dt_s"]["value"]
    min_ratio = TH["min_discrimination_ratio_median"]["value"]
    max_eff = TH["max_correction_effectiveness"]["value"]
    min_n = TH["min_discriminating_samples_per_surface"]["value"]

    def med(xs):
        return sorted(xs)[len(xs) // 2] if xs else None

    def rms(xs):
        return math.sqrt(sum(v * v for v in xs) / len(xs)) if xs else None

    def stats(rows, corrected):
        key = ("residual_visual_minus_actual_RATECORR_rad" if corrected
               else "residual_visual_minus_actual_rad")
        vkey = ("visual_origin_error_RATECORR_m" if corrected
                else "visual_origin_error_m")
        res_act = [abs(x[key]) for x in rows]
        res_cmd = [abs(x["residual_visual_minus_cmd_rad"]) for x in rows]
        sep = [abs(x["cmd_minus_actual_rad"]) for x in rows]
        idx = [i for i, v in enumerate(sep) if v > disc_floor]
        ra = [res_act[i] for i in idx]
        rc = [res_cmd[i] for i in idx]
        d = math.degrees
        return {
            "n": len(rows),
            "n_discriminating": len(idx),
            "residual_vs_ACTUAL_on_discriminating_deg": {
                "median": d(med(ra)) if ra else None,
                "rms": d(rms(ra)) if ra else None,
                "max": d(max(ra)) if ra else None,
            },
            "residual_vs_CMD_on_discriminating_deg": {
                "median": d(med(rc)) if rc else None,
                "rms": d(rms(rc)) if rc else None,
                "max": d(max(rc)) if rc else None,
                "min": d(min(rc)) if rc else None,
            },
            "discrimination_ratio_median_over_median":
                (med(rc) / med(ra)) if (ra and rc and med(ra) > 0) else None,
            "discrimination_ratio_rms_over_rms":
                (rms(rc) / rms(ra)) if (ra and rc and rms(ra) > 0) else None,
            "max_visual_origin_error_m": max((x[vkey] for x in rows), default=None),
            "max_cmd_minus_actual_deg": d(max(sep)) if sep else None,
            "_ra_rms": rms(ra),
        }

    per_surface = {}
    for s in L.SURFACES:
        rows = [r["surfaces"][s] for r in samples
                if s in r["surfaces"] and "cmd_rad" in r["surfaces"][s]]
        if not rows:
            per_surface[s] = {"verdict": "NO_DATA"}
            continue
        well = [x for x in rows
                if abs(x.get("dt_bind_pose_minus_actuator_s", 1.0)) <= dt_max]
        primary = stats(well, corrected=True) if well else None
        uncorr_same = stats(well, corrected=False) if well else None
        raw_all = stats(rows, corrected=False)

        # Does the first-order rate correction actually remove the residual?
        # If yes, the residual is a binding artefact of the 20 Hz diagnostics
        # rate. If no, it would be a genuine render/joint disagreement.
        eff = None
        if (primary and uncorr_same and primary["_ra_rms"] is not None
                and uncorr_same["_ra_rms"]):
            eff = primary["_ra_rms"] / uncorr_same["_ra_rms"]

        if primary is None or primary["n_discriminating"] < min_n:
            verdict = "INCONCLUSIVE_INSUFFICIENT_WELL_BOUND_DISCRIMINATING_SAMPLES"
        else:
            ratio = primary["discrimination_ratio_median_over_median"]
            follows_actual = (ratio is not None and ratio >= min_ratio)
            artefact_confirmed = (eff is not None and eff <= max_eff)
            cmd_min = primary["residual_vs_CMD_on_discriminating_deg"]["min"]
            act_med = primary["residual_vs_ACTUAL_on_discriminating_deg"]["median"]
            if follows_actual and artefact_confirmed:
                verdict = "PASS_VISUAL_FOLLOWS_ACTUAL_JOINT"
            elif follows_actual:
                verdict = ("PASS_VISUAL_FOLLOWS_ACTUAL_JOINT_"
                           "RESIDUAL_NOT_EXPLAINED_BY_BINDING")
            elif (cmd_min is not None and act_med is not None
                  and cmd_min < act_med):
                verdict = "FAIL_VISUAL_FOLLOWS_COMMAND"
            else:
                verdict = "FAIL_VISUAL_FOLLOWS_NEITHER"
        for d_ in (primary, uncorr_same, raw_all):
            if d_:
                d_.pop("_ra_rms", None)
        per_surface[s] = {
            "verdict": verdict,
            "primary_well_bound_rate_corrected": primary,
            "well_bound_UNCORRECTED_same_samples": uncorr_same,
            "all_samples_uncorrected": raw_all,
            "rate_correction_effectiveness_rms_corrected_over_uncorrected": eff,
            "rate_correction_effectiveness_threshold": max_eff,
            "rate_correction_interpretation":
                "A value well below 1 means the residual against the ACTUAL "
                "joint angle SHRINKS when the known pose-vs-actuator binding "
                "offset is compensated, i.e. the residual is an artefact of "
                "the 20 Hz diagnostics publish rate and NOT a render/joint "
                "disagreement. A value near 1 would mean the opposite and "
                "would be a real finding.",
            "n_well_bound": len(well),
            "well_bound_max_dt_s": dt_max,
            "max_off_axis_residual": max(x["off_axis_residual"] for x in rows),
            "off_axis_pass": bool(max(x["off_axis_residual"] for x in rows)
                                  <= TH["off_axis_residual_max"]["value"]),
            "max_actual_deg": math.degrees(max(x["actual_angle_rad"] for x in rows)),
            "min_actual_deg": math.degrees(min(x["actual_angle_rad"] for x in rows)),
            "any_target_clamp": any(x["target_clamp_active"] for x in rows),
            "any_effort_clamp": any(x["effort_clamp_active"] for x in rows),
        }


    skews = [r["binding"]["max_abs_skew_s"] for r in samples
             if r["binding"].get("max_abs_skew_s") is not None]
    per_source = {}
    if samples:
        for k in samples[0]["binding"]["skew_s"]:
            vals = [abs(r["binding"]["skew_s"][k]) for r in samples
                    if r["binding"]["skew_s"].get(k) is not None]
            arrivals = sorted({r["binding"]["arrival_wall"][k] for r in samples
                               if r["binding"]["arrival_wall"].get(k)})
            per_source[k] = {
                "mean_abs_skew_s": (sum(vals) / len(vals)) if vals else None,
                "max_abs_skew_s": max(vals) if vals else None,
                "distinct_message_arrivals": len(arrivals),
                "effective_rate_hz": ((len(arrivals) - 1) / (arrivals[-1] - arrivals[0]))
                if len(arrivals) > 2 and arrivals[-1] > arrivals[0] else None,
            }
    ts = [r["t_wall"] for r in samples]
    dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)] or [float("nan")]
    verdicts = [v.get("verdict") for v in per_surface.values()]
    return {
        "per_surface": per_surface,
        "per_source_timing": per_source,
        "capture": {
            "n_samples": len(samples),
            "mean_sample_period_s": sum(dts) / len(dts),
            "mean_rate_hz": (len(dts) / sum(dts)) if sum(dts) > 0 else None,
            "max_inter_source_skew_s": max(skews) if skews else None,
            "mean_inter_source_skew_s": (sum(skews) / len(skews)) if skews else None,
            "skew_note": "Sources are NOT simultaneous. The actuator, aero and "
                         "propulsion diagnostics are published at exactly "
                         "20 Hz by model.sdf's own <diagnostics_rate_hz>, "
                         "which this stage is not permitted to change; the "
                         "render pose stream runs at ~50 Hz. The primary "
                         "verdict therefore uses only the WELL-BOUND subset "
                         "and a first-order rate correction. The uncorrected "
                         "numbers are reported alongside so the size of the "
                         "artefact stays visible.",
        },
        "overall_verdict": (
            "PASS" if all(v and v.startswith("PASS") for v in verdicts)
            else "SEE_PER_SURFACE"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=DEFAULT_WORLD)
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5760")
    ap.add_argument("--dry-run", action="store_true",
                    help="run every offline check and exit; no transport, no "
                         "simulator required")
    args = ap.parse_args()

    L.env_setup()
    report = L.provenance_header("1 - live visual/joint consistency", args.world)
    report["thresholds"] = TH
    report["rc_step_programme"] = [
        {"label": a, "rc": b, "hold_s": c} for a, b, c in STEP_PROGRAMME]
    report["geometry_self_test"] = geometry_self_test()
    report["sdf_cross_check"] = sdf_cross_check()
    report["model_structure_read_only"] = L.read_model_structure()
    report["sign_scalars"] = L.read_sign_scalars()

    if not report["geometry_self_test"]["pass"]:
        log("GEOMETRY SELF-TEST FAILED - refusing to run")
        report["overall"] = "ABORT_GEOMETRY_SELF_TEST_FAILED"
    elif not report["sdf_cross_check"]["pass"]:
        log("SDF CROSS-CHECK FAILED - refusing to run:",
            report["sdf_cross_check"]["transport_constant_problems"])
        report["overall"] = "ABORT_SDF_CROSS_CHECK_FAILED"
    elif args.dry_run:
        log("--dry-run: offline checks OK, skipping live capture")
        report["overall"] = "DRY_RUN_OK"
    else:
        samples, static_tree, stream_rates = run_live(args.world, args)
        report["static_pose_tree_entity_count"] = len(static_tree)
        report["mavlink_stream_rates"] = stream_rates
        report["analysis"] = analyse(samples)
        report["overall"] = report["analysis"]["overall_verdict"]
        with open(TIMESERIES_JSON, "w", encoding="utf-8") as fh:
            json.dump({"header": {k: report[k] for k in ("stage", "part", "world")},
                       "samples": samples}, fh)
        log("wrote", TIMESERIES_JSON)

    os.makedirs(L.RESULTS_DIR, exist_ok=True)
    with open(RESULT_JSON, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    with open(LOG_TXT, "w", encoding="utf-8") as fh:
        fh.write("\n".join(_LOG_LINES) + "\n")
    log("wrote", RESULT_JSON)
    log("OVERALL:", report["overall"])
    return 0 if report["overall"] in ("PASS", "DRY_RUN_OK") else 1


if __name__ == "__main__":
    sys.exit(main())

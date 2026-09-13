#!/usr/bin/env python3
"""
FALCON V2 - ACTUATOR_CONTROL_LIMIT_CYCLE_FIX_AND_VALIDATION, STEP 4:
STANDALONE (NO-FLIGHT) per-surface actuator step-response bench.
Owner: gazebo-testing, 2026-09-12.

CLASSIFICATION: MEASUREMENT-ONLY. This script writes NO physics parameter of
any kind. It does not touch model/model.sdf, plugins/, config/,
docs/source_of_truth/, or any pre-existing world/script/result file. Every
artifact it produces carries the prefix `actuator_fix_standalone_`.

WHAT IT MEASURES, per surface, per commanded angle
  rise time, settling time, overshoot, steady-state error, oscillation
  (amplitude + sustained/not), actual joint rate, applied effort, rate-clamp
  duty, effort-clamp duty. Plus the +/-45 deg travel limit, a +/-60 deg
  overcommand clamp, and the 2026-08-24 gravity-droop re-verification.

BENCH SETUP (why this is "standalone")
  World tests/gazebo/worlds/falcon_v2_actuator_standalone_bench_world.sdf:
  gravity ON, no ground plane, model spawned at 100 m. The test holds
  base_link's body-frame linear AND angular velocity at zero every step
  (aero_lib.hold_step), so:
    * true airspeed is identically zero -> dynamic pressure zero -> the
      aerodynamics plugin contributes no force/moment at all;
    * no throttle is ever published -> the propulsion plugin contributes
      nothing;
    * the base never rotates -> no kinematic drag on the hinge.
  The ONLY torques on the surface under test are its own actuator torque and
  its own weight about its own hinge. That is the bench.

  Exactly one control-surface joint is left free per run; the other 4 control
  surfaces and both prop joints are pinned
  (actuator_lib.pin_other_child_joints), the same isolation convention used
  by the 2026-08-24 ACTUATOR_SERVO_MODEL_V1 pass this stage re-verifies
  against.

DIAGNOSTICS RATE
  The bench world spawns tests/gazebo/models/falcon_v2_highrate_diag (1 kHz
  diagnostics) rather than model/ (20 Hz). Rise times here are 5-20 ms and
  rate-clamp episodes are ~10-100 ms; 20 Hz cannot resolve either.
  Per validation 2026-09-11 finding m13, BOTH changed diagnostics rates
  belong to plugins that apply their forces every physics step and throttle
  only Publish(), so the publish rate is observability-only.

HOW "APPLIED EFFORT" IS OBTAINED  (it is NOT a published channel)
  The 35-field actuator diagnostics vector does NOT contain the applied joint
  torque. ActuatorSystem.cc::PreUpdate() computes `eff.torqueNm`, applies it
  via Joint::SetForce(), and publishes only `eff.effortClamped` (a bool). So
  effort here is obtained two independent ways, both reported:

    (E1) EXACT RECONSTRUCTION of the control law. Every input PidEffort()
         consumes - setpoint_rad, actual_angle_rad, actual_rate_rad_s,
         target_clamp_active - is present in the SAME 1 kHz diagnostics
         record, which is one coherent snapshot of one plugin tick. The only
         other input is the persistent integral, which is integrated forward
         in Python from its documented initial value 0.0 using the exact
         freeze logic of ActuatorModel.hh (all three clauses incl. F3). This
         is exact IF AND ONLY IF no diagnostics message was dropped; message
         accounting is performed and reported per run.
         Cross-check with power: the reconstruction also predicts
         `effortClamped`, which IS published - so whenever clamping occurs
         the reconstruction is falsifiable against ground truth. If clamping
         never occurs the agreement is only consistency, not proof, and is
         reported as such.

    (E2) INDEPENDENT DYNAMIC INVERSION, from Gazebo's own state, using no
         controller quantity at all:
             tau_applied = I_hinge * theta_ddot  -  tau_gravity(theta)
         with theta_ddot from the ECM-read joint rate series and
         tau_gravity from the surface's own SDF mass/CoM offset and the
         world gravity vector. Valid only on steps where the joint is NOT at
         its velocity limit (dartsim's velocity constraint injects an
         unmeasured constraint torque) and not at a position limit; those
         steps are excluded and counted.

  E1 and E2 are compared. Neither is a published measurement of the applied
  torque; that remains DATA_REQUIRED as a first-class channel.

TIMEBASE
  Unlike the flight records (validation finding I15: gz.msgs.Double_V carries
  no header, so actuator timestamps there are RECONSTRUCTED), this bench
  reads joint position/velocity directly from the ECM inside the fixture's
  own post-update callback, once per physics step. Those samples carry an
  exact, non-reconstructed step index. The diagnostics stream is aligned to
  that index by sequence, and the alignment is verified by direct value
  comparison (not assumed) - see `diag_alignment` in the result JSON.

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
control-sign mapping, motor/prop constant, actuator max_rate/max_effort/
kp/kd/ki, hinge geometry) is modified anywhere in this script.
"""
import json
import math
import os
import sys
import time

import actuator_lib as ACT
import aero_lib as AL

ACT.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

REPO = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = os.path.join(REPO, "tests/gazebo/results")
WORLD_SDF = os.path.join(
    REPO, "tests/gazebo/worlds/falcon_v2_actuator_standalone_bench_world.sdf")
PREFIX = "actuator_fix_standalone"

DT = ACT.STEP  # 0.001 s, = the bench world's <max_step_size>

# hold_step gains: reused VERBATIM from the already-established
# test_actuator_flight_load.py bench (KP_LIN/KP_ANG=150.0), which the
# 2026-08-24 report's own kp_ang sweep (50/150/400) showed to be a small
# secondary effect on the droop measurement, not the dominant term.
KP_LIN = 150.0
KP_ANG = 150.0

# ---------------------------------------------------------------------------
# Command schedule (radians of COMMANDED joint angle).
#   Phase 0  : 5.0 s at cmd = 0  -> the PRIMARY GATE (no sustained limit cycle
#              at zero command) AND the 2026-08-24 gravity-droop
#              re-verification (transient peak + steady state).
#   Phase 1..: for each angle a in ANGLES_DEG: 5.0 s at a, then 3.0 s back at
#              0. Each "to a" segment is a clean step FROM 0, so rise /
#              settling / overshoot are all measured against a known step.
#              5.0 s is chosen from a measured pilot run, not guessed: the
#              closed loop's slow integral pole is -2.041 rad/s (tau = 0.49 s,
#              docs/test_results/2026-09-12_actuator_limit_cycle_fix_
#              implementation.md section 3.1), and the sp_weight_b = 0.7
#              setpoint weighting (UNCHANGED by this fix) means the I-term
#              must supply kp*(1-b)*setpoint at steady state, so a large step
#              approaches its final value on that slow pole. A pilot run on
#              left_elevator measured residual 1.25 deg at t+2.5 s, 0.058 deg
#              at t+4.0 s and 0.0074 deg at t+5.0 s after a +30 deg step. A
#              2.5 s window would have reported a 1.25 deg "steady-state
#              error" that is simply an unsettled transient.
# ---------------------------------------------------------------------------
ANGLES_DEG = [5.0, -5.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0]
ZERO_HOLD_S = 5.0
STEP_HOLD_S = 5.0
RETURN_HOLD_S = 3.0


def build_schedule():
    """Returns (segments, total_steps). Each segment:
       dict(name, start_step, end_step, target_cmd_rad, prev_cmd_rad, kind)."""
    segs = []
    n = 0
    z = int(round(ZERO_HOLD_S / DT))
    segs.append(dict(name="CMD_ZERO_HOLD", start_step=0, end_step=z,
                     target_cmd_rad=0.0, prev_cmd_rad=0.0, kind="zero"))
    n = z
    for a in ANGLES_DEG:
        ns = int(round(STEP_HOLD_S / DT))
        nr = int(round(RETURN_HOLD_S / DT))
        segs.append(dict(name=f"STEP_TO_{a:+.0f}DEG", start_step=n, end_step=n + ns,
                         target_cmd_rad=math.radians(a), prev_cmd_rad=0.0, kind="step"))
        n += ns
        segs.append(dict(name=f"RETURN_FROM_{a:+.0f}DEG", start_step=n, end_step=n + nr,
                         target_cmd_rad=0.0, prev_cmd_rad=math.radians(a), kind="return"))
        n += nr
    return segs, n


def cmd_at_step(segs, k):
    for s in segs:
        if s["start_step"] <= k < s["end_step"]:
            return s["target_cmd_rad"]
    return 0.0


# ---------------------------------------------------------------------------
# Per-surface config, read (never written) from the authoritative YAML.
# Per-surface gains have existed since 2026-09-12 (F1); actuator_lib's older
# load_actuator_config() reads only left_aileron + the shared control_law
# block and is therefore no longer sufficient. Left untouched; a per-surface
# reader is added here instead.
# ---------------------------------------------------------------------------
def load_per_surface_config():
    import yaml
    with open(ACT.ACTUATOR_CONFIG_YAML_PATH) as f:
        root = yaml.safe_load(f)
    law = root["control_law"]
    out = {}
    for s in ACT.SURFACES:
        n = root["surfaces"][s]
        out[s] = dict(
            min_angle_rad=n["min_angle_rad"], max_angle_rad=n["max_angle_rad"],
            max_rate_rad_s=n["max_rate_rad_s"], max_effort_nm=n["max_effort_nm"],
            kp=n.get("kp_nm_per_rad", law["kp_nm_per_rad"]),
            kd=n.get("kd_nm_per_rad_s", law["kd_nm_per_rad_s"]),
            ki=n.get("ki_nm_per_rad_s", law.get("ki_nm_per_rad_s", 0.0)),
            sp_weight_b=law.get("sp_weight_b", 1.0),
            hinge_inertia_kg_m2=n.get("hinge_inertia_kg_m2"),
        )
    return out


# Surface link mass / CoM local offset / hinge axis, read VERBATIM from
# model/model.sdf (the same source model/ and falcon_v2_highrate_diag/ share;
# the latter's meshes/ is a symlink to model/meshes and its link/inertial
# blocks are byte-identical - see that model's header). Used ONLY to compute
# the gravity torque about the hinge for the E2 dynamic inversion. Read-only.
def load_sdf_surface_inertials():
    import xml.etree.ElementTree as ET
    path = os.path.join(REPO, "model/model.sdf")
    root = ET.parse(path).getroot()
    model = root.find("model")
    out = {}
    for s in ACT.SURFACES:
        link = None
        for lk in model.findall("link"):
            if lk.get("name") == s:
                link = lk
                break
        if link is None:
            continue
        inert = link.find("inertial")
        mass = float(inert.find("mass").text)
        pose = inert.find("pose")
        d = [float(x) for x in pose.text.split()][:3] if pose is not None else [0, 0, 0]
        lpose = link.find("pose")
        lp = [float(x) for x in lpose.text.split()] if lpose is not None else [0]*6
        ii = inert.find("inertia")
        tens = {k: float(ii.find(k).text) for k in
                ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
        out[s] = dict(mass=mass, com_local=d, link_pose=lp, inertia=tens)
    for j in model.findall("joint"):
        nm = j.get("name")
        for s in ACT.SURFACES:
            if nm == ACT.JOINT_NAMES[s]:
                ax = [float(x) for x in j.find("axis").find("xyz").text.split()]
                lim = j.find("axis").find("limit")
                out[s]["axis"] = ax
                out[s]["joint_lower"] = float(lim.find("lower").text)
                out[s]["joint_upper"] = float(lim.find("upper").text)
    return out


# ---------------------------------------------------------------------------
# Exact Python mirrors of ActuatorModel.hh (read from the committed header,
# line-for-line). Used for the E1 effort reconstruction ONLY - never to
# replace a live measurement.
# ---------------------------------------------------------------------------
RATE_SAT_DETECT_FRACTION = 0.999  # kRateSaturationDetectFraction, ActuatorModel.hh


def pid_effort(setpoint, angle, rate, prev_integral, dt, target_clamp_active, c):
    error = setpoint - angle
    error_p = c["sp_weight_b"] * setpoint - angle
    raw_before = c["kp"] * error_p + c["ki"] * prev_integral - c["kd"] * rate
    would_sat = c["max_effort_nm"] > 0.0 and abs(raw_before) >= c["max_effort_nm"]
    same_sign = (raw_before >= 0.0 and error >= 0.0) or (raw_before <= 0.0 and error <= 0.0)
    rate_sat = c["max_rate_rad_s"] > 0.0 and abs(rate) >= RATE_SAT_DETECT_FRACTION * c["max_rate_rad_s"]
    rate_same_sign = (rate >= 0.0 and error >= 0.0) or (rate <= 0.0 and error <= 0.0)
    rate_sat_freeze = rate_sat and rate_same_sign
    freeze = bool(target_clamp_active) or (would_sat and same_sign) or rate_sat_freeze
    integral = prev_integral + error * dt if (not freeze and dt > 0.0) else prev_integral
    raw = c["kp"] * error_p + c["ki"] * integral - c["kd"] * rate
    clamped = max(-c["max_effort_nm"], min(c["max_effort_nm"], raw)) if c["max_effort_nm"] > 0 else raw
    return dict(torque=clamped, effort_clamped=(clamped != raw), integral=integral,
                frozen=freeze, rate_sat_freeze=rate_sat_freeze, rate_saturated=rate_sat)


# ---------------------------------------------------------------------------
def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    return sim.Model(world.model_by_name(ecm, "falcon_v2"))


def run_surface(surface, segs, total_steps, log):
    jn = ACT.JOINT_NAMES[surface]
    state = dict(n=0, cmd=None, diag=None, inertia=None,
                 theta=[], rate=[], base_av=[], base_lv=[], nan=False,
                 diag_seen0=None)

    def on_pre(info, ecm):
        k = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if state["cmd"] is None:
            state["cmd"] = ACT.ActuatorCommander()
            state["inertia"] = AL.read_base_link_inertia(model, ecm, sim)
        state["cmd"].set(**{surface: cmd_at_step(segs, k)})
        state["cmd"].tick()
        ACT.pin_other_child_joints(model, ecm, sim, leave_free_joints=jn)
        AL.hold_step(base, ecm, state["inertia"]["mass"],
                     (state["inertia"]["ixx"], state["inertia"]["iyy"],
                      state["inertia"]["izz"]),
                     gm.Vector3d(0, 0, 0), gm.Vector3d(0, 0, 0),
                     kp_lin=KP_LIN, kp_ang=KP_ANG)

    def on_post(info, ecm):
        if state["diag"] is None:
            try:
                state["diag"] = ACT.DiagSubscriber()
            except Exception:
                pass
        model = get_model(ecm)
        th, rt = ACT.read_joint_state(model, ecm, sim, jn)
        state["theta"].append(th if th is not None else float("nan"))
        state["rate"].append(rt if rt is not None else float("nan"))
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        av = base.world_angular_velocity(ecm)
        lv = base.world_linear_velocity(ecm)
        if av is not None and lv is not None:
            state["base_av"].append((av.x(), av.y(), av.z()))
            state["base_lv"].append((lv.x(), lv.y(), lv.z()))
            if any(not math.isfinite(v) for v in
                   (av.x(), av.y(), av.z(), lv.x(), lv.y(), lv.z())):
                state["nan"] = True
        if th is None or rt is None or not math.isfinite(th) or not math.isfinite(rt):
            state["nan"] = True
        state["n"] += 1

    t0 = time.time()
    fixture = sim.TestFixture(WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)
    wall = time.time() - t0
    diag_hist = state["diag"].all_split() if state["diag"] else []
    log(f"  [{surface}] {state['n']} steps in {wall:.1f}s wall, "
        f"{len(diag_hist)} diagnostics messages received")
    return state, diag_hist


# ---------------------------------------------------------------------------
# Segment analysis
# ---------------------------------------------------------------------------
def analyse_segment(seg, theta, rate, eff_series, clamp_series, cfg, steady_win_s=0.5):
    a, b = seg["start_step"], seg["end_step"]
    th = theta[a:b]
    rt = rate[a:b]
    n = len(th)
    tgt_cmd = seg["target_cmd_rad"]
    tgt = max(cfg["min_angle_rad"], min(cfg["max_angle_rad"], tgt_cmd))  # travel clamp
    overcmd = (abs(tgt_cmd - tgt) > 1e-12)
    th0 = theta[a - 1] if a > 0 else th[0]
    delta = tgt - th0
    nw = int(round(steady_win_s / DT))
    tail = th[-nw:]
    prev = th[-2 * nw:-nw] if n >= 2 * nw else tail
    tail_rt = rt[-nw:]

    ss_mean = sum(tail) / len(tail)
    ss_err = ss_mean - tgt
    tail_p2p = max(tail) - min(tail)
    prev_p2p = max(prev) - min(prev)
    sustained = (tail_p2p > math.radians(0.05)) and (tail_p2p > 0.5 * prev_p2p)

    rise = None
    if abs(delta) > math.radians(0.5):
        lo, hi = th0 + 0.1 * delta, th0 + 0.9 * delta
        t_lo = t_hi = None
        for i, v in enumerate(th):
            if t_lo is None and ((delta > 0 and v >= lo) or (delta < 0 and v <= lo)):
                t_lo = i
            if t_lo is not None and ((delta > 0 and v >= hi) or (delta < 0 and v <= hi)):
                t_hi = i
                break
        if t_lo is not None and t_hi is not None:
            rise = (t_hi - t_lo) * DT

    over = 0.0
    if abs(delta) > 1e-12:
        if delta > 0:
            over = max(0.0, max(th) - tgt)
        else:
            over = max(0.0, tgt - min(th))
    over_pct = (over / abs(delta) * 100.0) if abs(delta) > 1e-12 else None

    band = max(0.02 * abs(delta), math.radians(0.05))
    settle = None
    for i in range(n - 1, -1, -1):
        if abs(th[i] - tgt) > band:
            if i + 1 < n:
                settle = (i + 1) * DT
            break
    else:
        settle = 0.0

    maxr = max(abs(v) for v in rt)
    pin = cfg["max_rate_rad_s"] * RATE_SAT_DETECT_FRACTION
    rate_duty = sum(1 for v in rt if abs(v) >= pin) / n

    eff = eff_series[a:b] if eff_series else []
    clamp = clamp_series[a:b] if clamp_series else []
    return dict(
        name=seg["name"], kind=seg["kind"],
        cmd_deg=math.degrees(tgt_cmd), target_deg_after_travel_clamp=math.degrees(tgt),
        overcommand=overcmd,
        start_deg=math.degrees(th0), step_size_deg=math.degrees(delta),
        rise_time_10_90_s=rise,
        settling_time_s=settle,
        settling_band_deg=math.degrees(band),
        overshoot_deg=math.degrees(over), overshoot_pct=over_pct,
        steady_state_mean_deg=math.degrees(ss_mean),
        steady_state_error_deg=math.degrees(ss_err),
        tail_p2p_deg=math.degrees(tail_p2p),
        prev_window_p2p_deg=math.degrees(prev_p2p),
        oscillation_sustained=bool(sustained),
        tail_rate_absmax_deg_s=math.degrees(max(abs(v) for v in tail_rt)),
        max_abs_rate_deg_s=math.degrees(maxr),
        max_abs_rate_over_limit_deg_s=math.degrees(maxr - cfg["max_rate_rad_s"]),
        rate_clamp_duty_frac=rate_duty,
        effort_clamp_duty_frac=(sum(1 for v in clamp if v > 0.5) / len(clamp)) if clamp else None,
        effort_recon_max_abs_nm=(max(abs(v) for v in eff) if eff else None),
        effort_recon_mean_abs_nm=((sum(abs(v) for v in eff) / len(eff)) if eff else None),
        effort_recon_tail_mean_nm=((sum(eff[-nw:]) / len(eff[-nw:])) if eff else None),
        n_samples=n,
    )


def main():
    lines = []

    def log(msg):
        print(msg, flush=True)
        lines.append(msg)

    cfgs = load_per_surface_config()
    sdfi = load_sdf_surface_inertials()
    segs, total_steps = build_schedule()
    log("=" * 78)
    log("ACTUATOR STANDALONE STEP-RESPONSE BENCH (STEP 4) - gazebo-testing 2026-09-12")
    log(f"world  : {WORLD_SDF}")
    log(f"steps  : {total_steps} per surface ({total_steps * DT:.1f} s sim), dt={DT}")
    log(f"angles : 0, {', '.join(f'{a:+.0f}' for a in ANGLES_DEG)} deg")
    log("=" * 78)

    out = dict(
        test="ACTUATOR_FIX_STANDALONE_STEP_RESPONSE",
        stage="ACTUATOR_CONTROL_LIMIT_CYCLE_FIX_AND_VALIDATION step 4",
        date="2026-09-12", owner="gazebo-testing",
        world=WORLD_SDF, dt_s=DT, total_steps=total_steps,
        schedule=[{k: v for k, v in s.items()} for s in segs],
        config_per_surface=cfgs, sdf_surface_inertials=sdfi,
        surfaces={},
    )

    for surface in ACT.SURFACES:
        log(f"--- surface {surface} ---")
        cfg = cfgs[surface]
        state, diag_hist = run_surface(surface, segs, total_steps, log)
        theta, rate = state["theta"], state["rate"]

        # ---- diagnostics alignment + E1 effort reconstruction ----
        d = [r[surface] for r in diag_hist]
        n_diag = len(d)
        align = dict(n_diag_messages=n_diag, n_steps=state["n"],
                     dropped_or_late=state["n"] - n_diag)
        eff_series = None
        clamp_series = None
        recon = None
        if n_diag >= state["n"] - 5:
            # The plugin publishes once per physics step at 1 kHz; diag record
            # k holds the joint state the plugin READ at step k, which is the
            # ECM post-update state of step k-1. Verify that by direct value
            # comparison rather than assuming it (offsets 0..2 scored).
            best = None
            for off in (0, 1, 2, 3):
                m = min(n_diag - off, len(theta) - 1)
                if m <= 10:
                    continue
                err = max(abs(d[i + off]["actual_angle_rad"] - theta[i - 1 if i > 0 else 0])
                          for i in range(1, m))
                if best is None or err < best[1]:
                    best = (off, err)
            align["best_offset"] = best[0] if best else None
            align["max_abs_angle_mismatch_rad"] = best[1] if best else None
            integ = 0.0
            eff_by_diag = []
            clamp_by_diag = []
            recon_flag_match = 0
            recon_flag_total = 0
            n_clamp_true = 0
            for rec in d:
                r = pid_effort(rec["setpoint_rad"], rec["actual_angle_rad"],
                               rec["actual_rate_rad_s"], integ, DT,
                               rec["target_clamp_active"] > 0.5, cfg)
                integ = r["integral"]
                eff_by_diag.append(r["torque"])
                clamp_by_diag.append(rec["effort_clamp_active"])
                recon_flag_total += 1
                if bool(r["effort_clamped"]) == (rec["effort_clamp_active"] > 0.5):
                    recon_flag_match += 1
                if rec["effort_clamp_active"] > 0.5:
                    n_clamp_true += 1
            off = best[0] if best else 0
            # map diag index -> step index (diag k <-> step k-1+off)
            eff_series = [float("nan")] * len(theta)
            clamp_series = [0.0] * len(theta)
            for i, v in enumerate(eff_by_diag):
                k = i - off + 1
                if 0 <= k < len(theta):
                    eff_series[k] = v
                    clamp_series[k] = clamp_by_diag[i]
            recon = dict(
                effort_clamp_flag_agreement_frac=recon_flag_match / max(1, recon_flag_total),
                n_published_effort_clamp_true=n_clamp_true,
                falsifiable=bool(n_clamp_true > 0),
                final_integral_rad_s=integ,
                note=("effort_clamp_active was never true in this run, so the "
                      "flag agreement is consistency only, not proof of the "
                      "reconstruction - it has no discriminating power when "
                      "both sides are constant false")
                if n_clamp_true == 0 else
                "effort_clamp_active went true; the flag agreement IS falsifiable here",
            )

        # ---- E2 independent dynamic inversion ----
        inert = sdfi.get(surface, {})
        e2 = None
        if inert and eff_series:
            ax = inert["axis"]
            nrm = math.sqrt(sum(v * v for v in ax))
            axn = [v / nrm for v in ax]
            m = inert["mass"]
            com = inert["com_local"]
            g = [0.0, 0.0, -9.81]
            # gravity torque about the hinge axis, in the link frame at theta=0
            # (base_link held level, so the world/link frames coincide to the
            # residual hold error, reported separately). tau = n . (r x F)
            F = [m * g[0], m * g[1], m * g[2]]
            rxF = [com[1] * F[2] - com[2] * F[1],
                   com[2] * F[0] - com[0] * F[2],
                   com[0] * F[1] - com[1] * F[0]]
            tau_g0 = sum(axn[i] * rxF[i] for i in range(3))
            Ih = cfg["hinge_inertia_kg_m2"]
            diffs = []
            excluded = 0
            for k in range(2, len(theta) - 1):
                if not math.isfinite(eff_series[k]):
                    continue
                if abs(rate[k]) >= RATE_SAT_DETECT_FRACTION * cfg["max_rate_rad_s"] or \
                   abs(rate[k - 1]) >= RATE_SAT_DETECT_FRACTION * cfg["max_rate_rad_s"]:
                    excluded += 1
                    continue
                if abs(theta[k]) >= 0.999 * cfg["max_angle_rad"]:
                    excluded += 1
                    continue
                acc = (rate[k] - rate[k - 1]) / DT
                tau_impl = Ih * acc - tau_g0 * math.cos(theta[k])
                diffs.append((eff_series[k], tau_impl))
            if diffs:
                dd = [b_ - a_ for a_, b_ in diffs]
                e2 = dict(
                    hinge_gravity_torque_at_zero_nm=tau_g0,
                    I_hinge_used_kg_m2=Ih,
                    n_compared=len(diffs), n_excluded=excluded,
                    max_abs_E1_nm=max(abs(a_) for a_, _ in diffs),
                    max_abs_E2_nm=max(abs(b_) for _, b_ in diffs),
                    rms_difference_nm=math.sqrt(sum(x * x for x in dd) / len(dd)),
                    max_abs_difference_nm=max(abs(x) for x in dd),
                    mean_difference_nm=sum(dd) / len(dd),
                )

        seg_results = [analyse_segment(s, theta, rate, eff_series, clamp_series, cfg)
                       for s in segs]

        # ---- gravity droop re-verification, from the CMD_ZERO_HOLD segment ----
        z = segs[0]
        zt = theta[z["start_step"]:z["end_step"]]
        peak_i = max(range(len(zt)), key=lambda i: abs(zt[i]))
        droop = dict(
            transient_peak_deg=math.degrees(zt[peak_i]),
            transient_peak_abs_deg=abs(math.degrees(zt[peak_i])),
            time_of_peak_s=peak_i * DT,
            steady_state_last_0p5s_mean_deg=math.degrees(
                sum(zt[-500:]) / len(zt[-500:])),
            steady_state_last_0p5s_p2p_deg=math.degrees(max(zt[-500:]) - min(zt[-500:])),
        )
        if inert:
            ax = inert["axis"]
            nrm = math.sqrt(sum(v * v for v in ax))
            axn = [v / nrm for v in ax]
            m = inert["mass"]
            com = inert["com_local"]
            F = [0.0, 0.0, -9.81 * m]
            rxF = [com[1] * F[2] - com[2] * F[1],
                   com[2] * F[0] - com[0] * F[2],
                   com[0] * F[1] - com[1] * F[0]]
            tg = sum(axn[i] * rxF[i] for i in range(3))
            droop["gravity_hinge_torque_nm"] = tg
            droop["predicted_P_only_equilibrium_deg"] = math.degrees(tg / cfg["kp"])
            droop["predicted_P_only_equilibrium_deg_prefix_kp"] = math.degrees(tg / 0.0034788)

        base_av = state["base_av"]
        hold_q = dict(
            base_ang_rate_absmax_deg_s=math.degrees(
                max(max(abs(c) for c in v) for v in base_av)) if base_av else None,
            base_ang_rate_rms_deg_s=math.degrees(math.sqrt(
                sum(sum(c * c for c in v) for v in base_av) / max(1, len(base_av)))) if base_av else None,
        )

        out["surfaces"][surface] = dict(
            n_steps=state["n"], any_nan_or_inf=state["nan"],
            diag_alignment=align, effort_reconstruction_E1=recon,
            effort_dynamic_inversion_E2=e2,
            base_hold_quality=hold_q,
            gravity_droop=droop,
            segments=seg_results,
            theta_deg_decimated_every_10=[
                round(math.degrees(theta[i]), 6) for i in range(0, len(theta), 10)],
            rate_deg_s_decimated_every_10=[
                round(math.degrees(rate[i]), 4) for i in range(0, len(rate), 10)],
        )

        # ---- console summary ----
        log(f"  droop: transient peak {droop['transient_peak_deg']:+.4f} deg at "
            f"t={droop['time_of_peak_s']:.3f}s, steady {droop['steady_state_last_0p5s_mean_deg']:+.5f} deg, "
            f"predicted P-only {droop.get('predicted_P_only_equilibrium_deg', float('nan')):+.4f} deg")
        for sr in seg_results:
            if sr["kind"] == "return":
                continue
            log(f"  {sr['name']:<22} rise={_f(sr['rise_time_10_90_s'],'s',4)} "
                f"settle={_f(sr['settling_time_s'],'s',3)} "
                f"over={sr['overshoot_deg']:.4f}deg "
                f"sse={sr['steady_state_error_deg']:+.5f}deg "
                f"p2p={sr['tail_p2p_deg']:.5f}deg sust={int(sr['oscillation_sustained'])} "
                f"maxrate={sr['max_abs_rate_deg_s']:.4f}deg/s "
                f"rdty={sr['rate_clamp_duty_frac']*100:.2f}% "
                f"edty={_pct(sr['effort_clamp_duty_frac'])} "
                f"|tau|max={_f(sr['effort_recon_max_abs_nm'],'Nm',6)}")

    os.makedirs(RESULTS_DIR, exist_ok=True)
    rp = os.path.join(RESULTS_DIR, f"{PREFIX}_step_response_result.json")
    with open(rp, "w") as f:
        json.dump(out, f, indent=2)
    lp = os.path.join(RESULTS_DIR, f"{PREFIX}_step_response_log.txt")
    with open(lp, "w") as f:
        f.write("\n".join(lines) + "\n")
    log(f"wrote {rp}")
    log(f"wrote {lp}")
    return 0


def _f(v, unit, nd):
    return "n/a" if v is None else f"{v:.{nd}f}{unit}"


def _pct(v):
    return "n/a" if v is None else f"{v*100:.2f}%"


if __name__ == "__main__":
    sys.exit(main())

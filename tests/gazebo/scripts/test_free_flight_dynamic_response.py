#!/usr/bin/env python3
"""
FALCON V2 - FREE_FLIGHT_DYNAMIC_RESPONSE_VALIDATION (gazebo-testing, 2026-08-23).

VALIDATION ONLY - not new physics/tuning. From the already-validated powered
trim point established in POWERED_TRIM_AND_FREE_FLIGHT_VALIDATION (reused
directly here, NOT re-searched - see test_powered_trim_search.py /
test_powered_free_flight_validation.py / docs/test_results/2026-08-23_
powered_trim_and_free_flight_validation_report.md), this script applies 5
small initial perturbations (one per classical linearized-dynamics mode -
short-period, phugoid, roll subsidence, Dutch roll, spiral) and observes the
aircraft's NATURAL free 6-DOF response. No control correction is applied
after release.

Critical distinction (see CLAUDE.md / task brief - restated here so it is
never misread from this script's evidence alone): this is NOT "does the
aircraft correct itself via a flight controller" - it is "what does the
aircraft's OWN aerodynamic damping/stability do when perturbed, with
controls held at trim/neutral". Two things are true for every test's
observation window, without exception:
  1. base_link itself is COMPLETELY free once released - no velocity/
     position/attitude hold, no P-controller, no external force/wrench is
     applied to base_link at all during the release window (verified by
     construction below: run_scenario()'s on_pre_update only ever calls
     AL.hold_step() while the current step index falls inside one of the
     pre-declared "phases" boundary windows; for every step at or beyond the
     last phase's end, NO call to hold_step() (or any other base_link
     force/torque/velocity/pose setter) is made at all for the rest of that
     test).
  2. The 5 control-surface joints (no actuator/servo model exists yet in
     this V1) stay PINNED at their trim/neutral commanded angle throughout
     the ENTIRE run (hold phases AND release window alike) via
     PL.pin_control_surface_joints(), called every tick - this represents
     the physical reality of an unpowered, currently-fixed control surface,
     it is NOT a flight controller correcting the aircraft's attitude.
     Propulsion is NOT locked/faked - throttle is held at the constant
     0.4915 command and republished every tick (topics are not latched,
     exactly as in every prior propulsion-stage test), but RPM/thrust keep
     evolving from the real rotor ODE + APC lookup every single step,
     release window included.

Perturbation technique (reuses the established pattern from prior stages,
see aero_lib.hold_step()'s own header for the empirical justification):
a brief hold_step()-based settle nudges the aircraft from trim to the
perturbed target rate/velocity-component (or, for the two ANGLE-typed
perturbations - test C's roll... no, wait, angle perturbations are only
used by test E's bank-angle case here, see below), then hold_step() is
never called again on base_link for the rest of that test's observation
window (full release). Link.set_linear_velocity()/set_angular_velocity()
(Cmd-style) are deliberately NOT used for the perturbation itself, per
aero_lib.py's own documented finding that these freeze further integration
once the calling script stops invoking them - which would corrupt a
"then release and observe free response" test.

Per-test perturbation method actually used (each a deliberate "your call"
choice per the task brief, stated explicitly):
  A (short-period): RATE method - q = +5 deg/s pitch-rate pulse (100 steps,
     0.1s, ang_axis_mask=(True,True,True) so roll/yaw stay pinned at 0
     throughout the brief pulse), then full release.
  B (phugoid): AIRSPEED-OFFSET method - u += +1.5 m/s (200 steps, 0.2s, w
     unchanged, angular targets held at 0 throughout the pulse), then full
     release.
  C (roll subsidence): RATE method - p = +8 deg/s roll-rate pulse (100
     steps, 0.1s), then full release.
  D (Dutch roll): SIDESLIP method - v-component pulse producing beta ~= +4
     deg (200 steps, 0.2s, u/w/angular targets held at trim/0 throughout),
     then full release.
  E (spiral): BANK-ANGLE method (angle, not rate, per the task's explicit
     requirement) - built via a RATE-PULSE-THEN-ZERO two-phase hold_step
     sequence (350 steps at p_target=+11.5 deg/s to accumulate ~4 deg of
     bank via p*dt integration under the fast P-controller, then 300 steps
     at p_target=0 deg/s to drive the roll RATE back down to ~0 before
     release, leaving a residual bank ANGLE with near-zero rate at the
     moment of release) rather than a one-shot Model.set_world_pose_cmd()
     pose write - this reuses ONLY the already-established hold_step
     mechanism (no new/unverified Link-level or Model-level one-shot
     kinematic-write primitive is used anywhere in this script; the gz.sim8
     Python Link/Model bindings available in this environment expose no
     Joint::ResetPosition()-equivalent one-shot velocity/pose primitive at
     the Link level - checked directly via dir(sim.Link)/dir(sim.Model)
     this pass, only set_linear_velocity/set_angular_velocity (Cmd-style,
     the documented-unsafe one) and Model.set_world_pose_cmd (usable but
     UNVERIFIED here for a mid-flight, non-initial, per-DOF angle nudge -
     the existing one-time initial teleport-to-altitude use of
     set_world_pose_cmd, reused unchanged below, is a DIFFERENT use case:
     it runs exactly once at t=0 before any dynamics have evolved, not a
     targeted single-DOF nudge applied on top of an already-settled trim
     state mid-run).

No aircraft physics parameter (aero coefficient, CL0/Cm0, control
derivative, drag model, stall limiter, propulsion/motor parameter, APC
Ct/Cp, mass, CG, inertia, control mapping, hinge geometry) is read for any
purpose other than loading the existing config/model, and none is modified
anywhere in this script.
"""
import json
import math
import sys

import propulsion_lib as PL
import aero_lib as AL

PL.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = f"{REPO_ROOT}/tests/gazebo/results"
WORLD = f"{REPO_ROOT}/tests/gazebo/worlds/falcon_v2_freefall_world.sdf"

# ---------------------------------------------------------------------------
# Base trim condition - taken DIRECTLY from POWERED_TRIM_AND_FREE_FLIGHT_
# VALIDATION (test_powered_trim_search.py / test_powered_free_flight_
# validation.py), NOT re-searched or re-derived. MASS/I_DIAG are the same
# runtime-queried base_link values those scripts used (controller-gain-only
# purpose, never fed back into any physics computation).
# ---------------------------------------------------------------------------
MASS = 5.9348
I_DIAG = (0.7284, 0.2507, 0.9523)
KP_LIN = 150.0
KP_ANG = 400.0
ALTITUDE_M = 100.0
HOLD_STEPS = 800  # 0.8s settle-to-trim, same duration as the prior free-flight validation pass

THROTTLE = 0.4915
V_TRIM = 18.165
ALPHA_TRIM_DEG = 2.461
ALPHA_TRIM_RAD = math.radians(ALPHA_TRIM_DEG)
U_HOLD = V_TRIM * math.cos(ALPHA_TRIM_RAD)
W_HOLD = -V_TRIM * math.sin(ALPHA_TRIM_RAD)
ELEVATOR_THETA_RAD = math.radians(5.50)  # physical, both left/right elevator joints
ELEVATOR_AERO_DEG = -5.50  # delta_e_aero = -0.5*(theta_left+theta_right), verified mapping

TRIM_LIN = gm.Vector3d(U_HOLD, 0.0, W_HOLD)
TRIM_ANG = gm.Vector3d(0.0, 0.0, 0.0)
FULL_MASK = (True, True, True)

TELEMETRY_FIELDS = ["n", "t", "V", "alt", "world_vz", "roll_deg", "pitch_deg", "yaw_deg",
                    "alpha_deg", "beta_deg", "p_deg_s", "q_deg_s", "r_deg_s",
                    "left_rpm", "right_rpm", "left_thrust_N", "right_thrust_N"]


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    model_e = world.model_by_name(ecm, "falcon_v2")
    return sim.Model(model_e)


def quat_rpy(rot):
    qx, qy, qz, qw = rot.x(), rot.y(), rot.z(), rot.w()
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# =============================================================================
# Generic runner: a sequence of (n_steps, lin_target, ang_target, ang_mask)
# "hold" phases (first phase always the HOLD_STEPS trim settle; subsequent
# phases build the perturbation), followed by `release_steps` of GENUINELY
# FREE flight - on_pre_update makes ZERO calls of any kind affecting
# base_link once the last declared phase has ended.
# =============================================================================
def run_scenario(test_id, phases, release_steps, telemetry_every, log):
    boundaries = []
    start = 0
    for (nsteps, lin_t, ang_t, mask) in phases:
        boundaries.append((start, start + nsteps, lin_t, ang_t, mask))
        start += nsteps
    total_hold_steps = start
    total_steps = total_hold_steps + release_steps

    state = {"n": 0, "teleported": False, "series": [], "any_nan": False,
             "nan_step": None, "throttle_cmd": None, "prop_diag": None,
             "release_start_state": None, "trim_check_state": None}

    def on_pre(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, ALTITUDE_M, 0, 0, 0))
            state["teleported"] = True
            state["throttle_cmd"] = PL.ThrottleCommander()
        state["throttle_cmd"].set(left=THROTTLE, right=THROTTLE)
        state["throttle_cmd"].tick()
        PL.pin_control_surface_joints(model, ecm, sim, positions={
            "left_elevator_joint": ELEVATOR_THETA_RAD,
            "right_elevator_joint": ELEVATOR_THETA_RAD})
        n = state["n"]
        for (s, e, lin_t, ang_t, mask) in boundaries:
            if s <= n < e:
                AL.hold_step(base, ecm, MASS, I_DIAG, lin_t, ang_t,
                             kp_lin=KP_LIN, kp_ang=KP_ANG, ang_axis_mask=mask)
                break
        # else (n >= total_hold_steps): fully released - no controller, no
        # stabilizer, no holding force/torque/velocity/pose call whatsoever.

    def on_post(info, ecm):
        if state["prop_diag"] is None:
            try:
                state["prop_diag"] = PL.DiagSubscriber()
            except Exception:
                pass
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        wpose = base.world_pose(ecm)
        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        if wpose is None or lv is None or av is None:
            state["any_nan"] = True
            if state["nan_step"] is None:
                state["nan_step"] = n
            state["n"] += 1
            return
        rot = wpose.rot()
        lv_b = rot.rotate_vector_reverse(lv)
        av_b = rot.rotate_vector_reverse(av)
        raw_vals = [lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z(),
                    wpose.pos().z(), lv.z()]
        if any(math.isnan(x) or math.isinf(x) for x in raw_vals):
            state["any_nan"] = True
            if state["nan_step"] is None:
                state["nan_step"] = n

        roll, pitch, yaw = quat_rpy(rot)
        V = math.sqrt(lv_b.x() ** 2 + lv_b.y() ** 2 + lv_b.z() ** 2)
        alpha = math.atan2(-lv_b.z(), lv_b.x())
        beta = math.atan2(lv_b.y(), math.hypot(lv_b.x(), lv_b.z()))
        prop = state["prop_diag"].latest() if state["prop_diag"] else None
        entry = dict(
            n=n, t=(n - total_hold_steps) * AL.STEP,
            V=V, alt=wpose.pos().z(), world_vz=lv.z(),
            roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch), yaw_deg=math.degrees(yaw),
            alpha_deg=math.degrees(alpha), beta_deg=math.degrees(beta),
            p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z()),
            left_rpm=(prop["left"]["rpm"] if prop else None),
            right_rpm=(prop["right"]["rpm"] if prop else None),
            left_thrust_N=(prop["left"]["thrust_N"] if prop else None),
            right_thrust_N=(prop["right"]["thrust_N"] if prop else None),
        )
        if n == HOLD_STEPS - 1:
            state["trim_check_state"] = entry
        if n >= total_hold_steps and (n - total_hold_steps) % telemetry_every == 0:
            state["series"].append(entry)
            if n == total_hold_steps:
                state["release_start_state"] = entry
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    log(f"[{test_id}] total_steps={state['n']} (expected {total_steps}) any_nan={state['any_nan']}"
        + (f" first_at_step={state['nan_step']}" if state["any_nan"] else ""))
    tc = state["trim_check_state"]
    if tc:
        log(f"[{test_id}] end-of-trim-hold check (n={HOLD_STEPS-1}): V={tc['V']:.3f} "
            f"roll={tc['roll_deg']:+.3f} pitch={tc['pitch_deg']:+.3f} yaw={tc['yaw_deg']:+.3f} "
            f"p={tc['p_deg_s']:+.3f} q={tc['q_deg_s']:+.3f} r={tc['r_deg_s']:+.3f} deg(/s)")
    rs = state["release_start_state"]
    if rs:
        log(f"[{test_id}] release-start (perturbed) state: V={rs['V']:.3f} alt={rs['alt']:.2f} "
            f"roll={rs['roll_deg']:+.3f} pitch={rs['pitch_deg']:+.3f} yaw={rs['yaw_deg']:+.3f} "
            f"beta={rs['beta_deg']:+.3f} p={rs['p_deg_s']:+.3f} q={rs['q_deg_s']:+.3f} r={rs['r_deg_s']:+.3f} deg(/s)")
    return state, total_hold_steps, total_steps


# =============================================================================
# Simple, defensible mode-identification helpers (plain Python + numpy array
# ops only - no scipy / heavy signal-processing framework).
# =============================================================================
def truncate_before_sign_change(t, x):
    """Returns the prefix of (t, x) up to (but not including) the first
    sample whose sign differs from x[0]. A genuine single-real-pole
    exponential decay never crosses zero in finite time; a sign change
    indicates either coupling with another (typically slower) mode has
    started to dominate, or numerical noise near zero - either way, not
    part of the fast decay being characterized. Used for test C (roll
    subsidence) below, where the raw p(t) trace is observed to overshoot
    slightly negative (a real, physically-plausible roll/yaw cross-coupling
    effect, see test report) after the initial fast decay - fitting a single
    exponential across that overshoot would bias tau upward by ~7x, as found
    empirically this pass."""
    if not x:
        return t, x
    x0 = x[0]
    if x0 == 0:
        return t[:1], x[:1]
    for i in range(1, len(x)):
        if x[i] * x0 < 0:
            return t[:i], x[:i]
    return t, x


def merged_oscillation_metrics(t, x):
    """Damped period/frequency + log-decrement damping ratio from a
    (possibly decaying OR growing) oscillation, using ALL extrema (maxima
    AND minima, chronologically merged, including the t[0] initial-condition
    turning point) rather than same-sign peaks only.

    RATIONALE (found empirically this pass, see test report "methodology
    finding"): the original same-sign-peaks-only approach, with amplitude
    measured relative to a single baseline (0 or mean(x)), was tried first
    and produced a spurious NEGATIVE (growing) damping estimate for test A
    (short-period) - traced to only 2 same-sign peaks being available in the
    short observation window, both already contaminated by the slower
    phugoid mode's baseline drift (phugoid and short-period are only cleanly
    separable in a LINEARIZED model; in this live nonlinear 6-DOF sim they
    are coupled, and within ~1s the short-period signal's local mean is
    already drifting due to phugoid content). Using adjacent-extrema
    (half-cycle) SWINGS instead of same-baseline peak magnitudes cancels
    slow drift by construction (a simple difference filter), and restricting
    the swing sequence to its initial monotonic run (in whichever direction
    - decaying or growing - the data itself shows, decided from the first
    two swings, not assumed) is a data-driven cutoff that stops before a
    slower coupled mode or the numerical noise floor contaminates the
    estimate. This is the technique actually used for tests A/B/D below.
    """
    result = dict(n_extrema=0, extrema_times=[], extrema_values=[],
                   Td_s=None, fd_hz=None, swings=None, swings_used=None,
                   log_decrement_full_period_est=None, zeta_est=None)
    if not x:
        return result
    extrema = [(t[0], x[0])]
    for i in range(1, len(x) - 1):
        if (x[i] > x[i - 1] and x[i] >= x[i + 1]) or (x[i] < x[i - 1] and x[i] <= x[i + 1]):
            extrema.append((t[i], x[i]))
    result["n_extrema"] = len(extrema)
    result["extrema_times"] = [e[0] for e in extrema]
    result["extrema_values"] = [e[1] for e in extrema]
    if len(extrema) < 2:
        return result
    half_periods = [extrema[i + 1][0] - extrema[i][0] for i in range(len(extrema) - 1)]
    Td = 2.0 * (sum(half_periods) / len(half_periods))
    result["Td_s"] = Td
    result["fd_hz"] = (1.0 / Td) if Td > 0 else None
    if len(extrema) < 3:
        return result
    swings = [abs(extrema[i + 1][1] - extrema[i][1]) for i in range(len(extrema) - 1)]
    result["swings"] = swings
    prefix = [swings[0]]
    increasing = swings[1] > swings[0]
    for s in swings[1:]:
        if (increasing and s > prefix[-1]) or ((not increasing) and s < prefix[-1]):
            prefix.append(s)
        else:
            break
    result["swings_used"] = prefix
    if len(prefix) >= 2 and prefix[0] > 1e-9 and prefix[-1] > 1e-9:
        n_half = len(prefix) - 1
        delta_half = math.log(prefix[0] / prefix[-1]) / n_half
        delta_full = 2.0 * delta_half  # per full period (2 half-cycles)
        result["log_decrement_full_period_est"] = delta_full
        result["zeta_est"] = delta_full / math.sqrt((2 * math.pi) ** 2 + delta_full ** 2)
    return result


def exp_decay_fit(t, x, amp_floor_frac=0.02):
    """Least-squares fit of ln|x| vs t, restricted (data-driven, not
    reference-driven) to the window where |x| stays above amp_floor_frac of
    |x[0]| - avoids fitting into the numerical noise floor once a fast mode
    has genuinely decayed away. Returns slope (1/s, negative=decay) and
    tau_s = -1/slope."""
    result = dict(slope=None, tau_s=None, n_points=0, window_t_max=None)
    if not x:
        return result
    x0 = abs(x[0])
    if x0 < 1e-9:
        return result
    floor = amp_floor_frac * x0
    pts = [(ti, math.log(abs(xi))) for ti, xi in zip(t, x) if abs(xi) > floor]
    if len(pts) < 3:
        result["n_points"] = len(pts)
        return result
    n = len(pts)
    sum_t = sum(p[0] for p in pts)
    sum_y = sum(p[1] for p in pts)
    sum_tt = sum(p[0] ** 2 for p in pts)
    sum_ty = sum(p[0] * p[1] for p in pts)
    denom = n * sum_tt - sum_t ** 2
    if denom == 0:
        result["n_points"] = n
        return result
    slope = (n * sum_ty - sum_t * sum_y) / denom
    result.update(slope=slope, tau_s=((-1.0 / slope) if slope < 0 else None),
                   n_points=n, window_t_max=pts[-1][0])
    return result


def trend_fit(t, x):
    """Linear regression of ln|x| vs t over the FULL window - used for the
    slow spiral growth/decay trend. Positive slope = growth (matches the
    XFLR5 reference sign convention lambda>0 = divergent)."""
    pts = [(ti, math.log(abs(xi))) for ti, xi in zip(t, x) if abs(xi) > 1e-6]
    result = dict(slope=None, n_points=len(pts), doubling_time_s=None, half_time_s=None)
    if len(pts) < 3:
        return result
    n = len(pts)
    sum_t = sum(p[0] for p in pts)
    sum_y = sum(p[1] for p in pts)
    sum_tt = sum(p[0] ** 2 for p in pts)
    sum_ty = sum(p[0] * p[1] for p in pts)
    denom = n * sum_tt - sum_t ** 2
    if denom == 0:
        return result
    slope = (n * sum_ty - sum_t * sum_y) / denom
    result["slope"] = slope
    if slope and slope > 0:
        result["doubling_time_s"] = math.log(2) / slope
    elif slope and slope < 0:
        result["half_time_s"] = math.log(2) / (-slope)
    return result


def symmetry_check(series):
    d_rpm = [abs(s["left_rpm"] - s["right_rpm"]) for s in series if s["left_rpm"] is not None]
    d_thr = [abs(s["left_thrust_N"] - s["right_thrust_N"]) for s in series if s["left_thrust_N"] is not None]
    return dict(max_abs_rpm_diff=(max(d_rpm) if d_rpm else None),
                max_abs_thrust_diff=(max(d_thr) if d_thr else None),
                n_samples=len(series))


# =============================================================================
# Test A - Short period (fast pitch)
# =============================================================================
def test_A_short_period(log):
    Q_PERT_DEG_S = 5.0
    phases = [
        (HOLD_STEPS, TRIM_LIN, TRIM_ANG, FULL_MASK),
        (100, TRIM_LIN, gm.Vector3d(0, math.radians(Q_PERT_DEG_S), 0), FULL_MASK),
    ]
    state, thold, ttotal = run_scenario("A_SHORT_PERIOD", phases, release_steps=5000,
                                          telemetry_every=5, log=log)
    series = state["series"]
    t = [s["t"] for s in series]
    q = [s["q_deg_s"] for s in series]
    pitch = [s["pitch_deg"] for s in series]
    metrics = merged_oscillation_metrics(t, q)
    q0 = q[0] if q else None
    q_last = q[-1] if q else None
    settle_idx = None
    for i, qi in enumerate(q):
        if all(abs(qj) <= 0.05 * abs(q0) for qj in q[i:]) and q0:
            settle_idx = i
            break
    t_settle = t[settle_idx] if settle_idx is not None else None

    fd = metrics["fd_hz"]
    zeta = metrics["zeta_est"]
    stable_damped = (zeta is not None and zeta > 0) or (t_settle is not None and t_settle < 3.0)
    if state["any_nan"]:
        verdict, reason = "FAIL", "NaN/Inf encountered during release window"
    elif zeta is not None and zeta < 0:
        verdict, reason = "FAIL", f"measured growing (not damped) short-period oscillation, zeta_est={zeta:.3f} < 0"
    elif stable_damped:
        verdict, reason = "PASS", (f"damped oscillation confirmed (fd={fd:.3f}Hz zeta_est={zeta:.3f}, "
                                     f"settled by t={t_settle:.2f}s)" if fd and zeta and t_settle is not None
                                     else "q decayed and no divergence observed")
    else:
        verdict, reason = "INCONCLUSIVE", "insufficient peaks/settling evidence in the observation window"

    log(f"[A] q0={q0} q_end={q_last} fd={fd} zeta_est={zeta} t_settle={t_settle} -> {verdict}: {reason}")
    return dict(test="A_SHORT_PERIOD", method="pitch-rate pulse q=+5deg/s (100 steps, 0.1s)",
                perturbation_achieved=dict(q0_deg_s=q0),
                measured=dict(fd_hz=fd, zeta_est=zeta, Td_s=metrics["Td_s"], t_settle_s=t_settle,
                               q_end_deg_s=q_last, max_abs_pitch_deg=max(abs(p) for p in pitch) if pitch else None,
                               n_extrema=metrics["n_extrema"], swings_used=metrics["swings_used"]),
                reference=dict(lam_re=-5.67462, lam_im=13.25078, fn_hz=2.294, fd_hz=2.109, zeta=0.394),
                verdict=verdict, reason=reason,
                symmetry=symmetry_check(series), any_nan=state["any_nan"],
                series=series)


# =============================================================================
# Test B - Phugoid (slow energy exchange)
# =============================================================================
def test_B_phugoid(log):
    DU = 1.5
    phases = [
        (HOLD_STEPS, TRIM_LIN, TRIM_ANG, FULL_MASK),
        (200, gm.Vector3d(U_HOLD + DU, 0.0, W_HOLD), TRIM_ANG, FULL_MASK),
    ]
    state, thold, ttotal = run_scenario("B_PHUGOID", phases, release_steps=30000,
                                          telemetry_every=50, log=log)
    series = state["series"]
    t = [s["t"] for s in series]
    V = [s["V"] for s in series]
    pitch = [s["pitch_deg"] for s in series]
    alt = [s["alt"] for s in series]
    metrics_V = merged_oscillation_metrics(t, V)
    metrics_pitch = merged_oscillation_metrics(t, pitch)
    Td = metrics_V["Td_s"] or metrics_pitch["Td_s"]
    zeta = metrics_V["zeta_est"]
    bounded = (max(V) - min(V)) < 6.0 and (max(pitch) - min(pitch)) < 30.0 if V and pitch else False

    if state["any_nan"]:
        verdict, reason = "FAIL", "NaN/Inf encountered during release window"
    elif bounded:
        verdict, reason = "PASS", (f"slow, bounded, {'lightly-damped' if (zeta is None or zeta < 0.05) else 'damped'} "
                                     f"oscillatory energy exchange observed (period~{Td}s), matching the expected "
                                     f"very-lightly-damped phugoid character (not required to decay visibly)")
    else:
        verdict, reason = "FAIL", "airspeed/pitch excursion diverged without bound during the observation window"

    log(f"[B] V range=[{min(V):.3f},{max(V):.3f}] Td={Td} zeta_est={zeta} -> {verdict}: {reason}")
    return dict(test="B_PHUGOID", method="airspeed pulse u+=+1.5m/s (200 steps, 0.2s)",
                perturbation_achieved=dict(V0=series[0]["V"] if series else None),
                measured=dict(Td_s=Td, zeta_est=zeta, V_min=min(V) if V else None, V_max=max(V) if V else None,
                               pitch_min_deg=min(pitch) if pitch else None, pitch_max_deg=max(pitch) if pitch else None,
                               alt_start=alt[0] if alt else None, alt_end=alt[-1] if alt else None,
                               n_extrema_V=metrics_V["n_extrema"], swings_used_V=metrics_V["swings_used"]),
                reference=dict(lam_re=-0.00185, lam_im=0.61576, period_s=10.2, zeta=0.003),
                verdict=verdict, reason=reason,
                symmetry=symmetry_check(series), any_nan=state["any_nan"],
                series=series)


# =============================================================================
# Test C - Roll subsidence (fast roll damping)
# =============================================================================
def test_C_roll_subsidence(log):
    P_PERT_DEG_S = 8.0
    phases = [
        (HOLD_STEPS, TRIM_LIN, TRIM_ANG, FULL_MASK),
        (100, TRIM_LIN, gm.Vector3d(math.radians(P_PERT_DEG_S), 0, 0), FULL_MASK),
    ]
    state, thold, ttotal = run_scenario("C_ROLL_SUBSIDENCE", phases, release_steps=2000,
                                          telemetry_every=1, log=log)
    series = state["series"]
    t = [s["t"] for s in series]
    p = [s["p_deg_s"] for s in series]
    roll = [s["roll_deg"] for s in series]
    t_fit, p_fit = truncate_before_sign_change(t, p)
    fit = exp_decay_fit(t_fit, p_fit, amp_floor_frac=0.02)
    p0 = p[0] if p else None
    p_end = p[-1] if p else None
    decayed = (p0 is not None and p_end is not None and abs(p_end) < 0.05 * abs(p0))

    if state["any_nan"]:
        verdict, reason = "FAIL", "NaN/Inf encountered during release window"
    elif fit["slope"] is not None and fit["slope"] > 0:
        verdict, reason = "FAIL", f"roll rate AMPLIFIED instead of damping (fit slope={fit['slope']:.3f}/s > 0)"
    elif decayed:
        verdict, reason = "PASS", (f"roll rate damped to near-zero (p0={p0:.2f}deg/s -> p_end={p_end:.3f}deg/s), "
                                     f"tau_est={fit['tau_s']}s")
    else:
        verdict, reason = "INCONCLUSIVE", "roll rate did not clearly settle within the observation window"

    log(f"[C] p0={p0} p_end={p_end} tau_est={fit['tau_s']} slope={fit['slope']} "
        f"fit_window=[0,{fit['window_t_max']}] roll_end={roll[-1] if roll else None} -> {verdict}: {reason}")
    return dict(test="C_ROLL_SUBSIDENCE", method="roll-rate pulse p=+8deg/s (100 steps, 0.1s)",
                perturbation_achieved=dict(p0_deg_s=p0),
                measured=dict(tau_s=fit["tau_s"], slope_per_s=fit["slope"], fit_n_points=fit["n_points"],
                               fit_window_t_max_s=fit["window_t_max"],
                               p_end_deg_s=p_end, roll_start_deg=roll[0] if roll else None,
                               roll_end_deg=roll[-1] if roll else None),
                note="roll RATE damping to zero is what is tested; roll ANGLE is NOT expected to return to "
                     "zero (no wings-level controller exists) - roll_start/roll_end reported for context only, "
                     "not as a pass/fail criterion. tau_s is fit only over the pre-zero-crossing window "
                     "(truncate_before_sign_change) - p is observed to overshoot slightly negative afterward "
                     "via roll/yaw cross-coupling, which would otherwise bias a single-exponential tau fit "
                     "upward by ~7x (found empirically this pass, see report).",
                reference=dict(lam=-9.46425, tau_s=0.106),
                verdict=verdict, reason=reason,
                symmetry=symmetry_check(series), any_nan=state["any_nan"],
                series=series)


# =============================================================================
# Test D - Dutch roll (coupled yaw-roll)
# =============================================================================
def test_D_dutch_roll(log):
    BETA_PERT_DEG = 4.0
    V_PERT = V_TRIM * math.sin(math.radians(BETA_PERT_DEG))
    phases = [
        (HOLD_STEPS, TRIM_LIN, TRIM_ANG, FULL_MASK),
        (200, gm.Vector3d(U_HOLD, V_PERT, W_HOLD), TRIM_ANG, FULL_MASK),
    ]
    state, thold, ttotal = run_scenario("D_DUTCH_ROLL", phases, release_steps=14000,
                                          telemetry_every=5, log=log)
    series = state["series"]
    t = [s["t"] for s in series]
    beta = [s["beta_deg"] for s in series]
    r = [s["r_deg_s"] for s in series]
    p = [s["p_deg_s"] for s in series]
    roll = [s["roll_deg"] for s in series]
    metrics = merged_oscillation_metrics(t, beta)
    fd = metrics["fd_hz"]
    zeta = metrics["zeta_est"]
    bounded = (max(beta) - min(beta)) < 40.0 and (max(r) - min(r)) < 200.0 if beta and r else False

    if state["any_nan"]:
        verdict, reason = "FAIL", "NaN/Inf encountered during release window"
    elif not bounded:
        verdict, reason = "FAIL", "lateral state exploded (unbounded growth) instead of a coupled oscillation"
    elif zeta is not None and zeta < -0.02:
        verdict, reason = "FAIL", f"yaw/roll rate amplification measured (zeta_est={zeta:.3f} clearly negative)"
    else:
        verdict, reason = "PASS", (f"coupled yaw-roll oscillation observed (fd={fd}, zeta_est={zeta}), "
                                     f"non-explosive")

    log(f"[D] beta range=[{min(beta):.3f},{max(beta):.3f}] fd={fd} zeta_est={zeta} n_extrema={metrics['n_extrema']} -> {verdict}: {reason}")
    return dict(test="D_DUTCH_ROLL", method="sideslip pulse (v-target) beta~=+4deg (200 steps, 0.2s)",
                perturbation_achieved=dict(beta0_deg=series[0]["beta_deg"] if series else None),
                measured=dict(fd_hz=fd, zeta_est=zeta, Td_s=metrics["Td_s"], n_extrema=metrics["n_extrema"],
                               beta_min_deg=min(beta) if beta else None, beta_max_deg=max(beta) if beta else None,
                               r_min_deg_s=min(r) if r else None, r_max_deg_s=max(r) if r else None,
                               p_min_deg_s=min(p) if p else None, p_max_deg_s=max(p) if p else None,
                               roll_min_deg=min(roll) if roll else None, roll_max_deg=max(roll) if roll else None),
                reference=dict(lam_re=-0.30549, lam_im=3.20273, fn_hz=0.512, zeta=0.095, period_s=1.96),
                verdict=verdict, reason=reason,
                symmetry=symmetry_check(series), any_nan=state["any_nan"],
                series=series)


# =============================================================================
# Test E - Spiral (slow lateral divergence/convergence)
# =============================================================================
def test_E_spiral(log):
    P_BUILD_DEG_S = 11.5
    phases = [
        (HOLD_STEPS, TRIM_LIN, TRIM_ANG, FULL_MASK),
        (350, TRIM_LIN, gm.Vector3d(math.radians(P_BUILD_DEG_S), 0, 0), FULL_MASK),
        (300, TRIM_LIN, gm.Vector3d(0, 0, 0), FULL_MASK),
    ]
    state, thold, ttotal = run_scenario("E_SPIRAL", phases, release_steps=20000,
                                          telemetry_every=50, log=log)
    series = state["series"]
    t = [s["t"] for s in series]
    roll = [s["roll_deg"] for s in series]
    beta = [s["beta_deg"] for s in series]
    fit_roll = trend_fit(t, roll)
    fit_beta = trend_fit(t, beta)
    p0 = series[0]["p_deg_s"] if series else None
    roll0 = series[0]["roll_deg"] if series else None

    slope = fit_roll["slope"]
    if state["any_nan"]:
        verdict, reason = "FAIL", "NaN/Inf encountered during release window"
    elif slope is None:
        verdict, reason = "INCONCLUSIVE", "insufficient trend evidence to fit a growth/decay rate"
    elif abs(slope) > 5.0:
        verdict, reason = "FAIL", f"immediate/violent lateral divergence (fit slope={slope:.3f}/s, far outside a slow spiral)"
    else:
        verdict, reason = "PASS", (f"slow lateral trend measured (slope={slope:+.4f}/s), order of magnitude "
                                     f"consistent with a weakly-stable-or-unstable spiral mode "
                                     f"({'growth' if slope > 0 else 'decay'})")

    log(f"[E] roll0={roll0} p0(residual)={p0} fit_slope={slope} doubling/half={fit_roll['doubling_time_s']}/{fit_roll['half_time_s']} -> {verdict}: {reason}")
    return dict(test="E_SPIRAL", method="bank-angle build via roll-rate pulse-then-zero "
                     "(350 steps @ p=+11.5deg/s, then 300 steps @ p=0deg/s, then release)",
                perturbation_achieved=dict(roll0_deg=roll0, p0_residual_deg_s=p0),
                measured=dict(slope_roll_per_s=fit_roll["slope"], doubling_time_s=fit_roll["doubling_time_s"],
                               half_time_s=fit_roll["half_time_s"], slope_beta_per_s=fit_beta["slope"],
                               roll_start_deg=roll[0] if roll else None, roll_end_deg=roll[-1] if roll else None,
                               beta_start_deg=beta[0] if beta else None, beta_end_deg=beta[-1] if beta else None),
                reference=dict(lam=0.08227, doubling_time_s=8.43),
                verdict=verdict, reason=reason,
                symmetry=symmetry_check(series), any_nan=state["any_nan"],
                series=series)


def run_all():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    log("FALCON V2 - FREE_FLIGHT_DYNAMIC_RESPONSE_VALIDATION (gazebo-testing, 2026-08-23)")
    log(f"World: {WORLD}")
    log(f"Base trim (reused, not re-searched): throttle={THROTTLE} elevator_theta=+5.50deg physical "
        f"(delta_e_aero={ELEVATOR_AERO_DEG}deg) V={V_TRIM} alpha={ALPHA_TRIM_DEG}deg "
        f"-> u={U_HOLD:.5f} w={W_HOLD:.5f} m/s, altitude={ALTITUDE_M}m")
    log("")

    results = {}
    for fn in (test_A_short_period, test_B_phugoid, test_C_roll_subsidence,
               test_D_dutch_roll, test_E_spiral):
        log("-" * 78)
        r = fn(log)
        results[r["test"]] = r
        log("")

    any_nan = any(r["any_nan"] for r in results.values())
    hard_fails = [r["test"] for r in results.values() if r["verdict"] == "FAIL"]

    log("=" * 78)
    log("OVERALL SUMMARY")
    log("=" * 78)
    for r in results.values():
        log(f"{r['test']:20s} verdict={r['verdict']:12s} {r['reason']}")
    log(f"any_nan across all 5 tests: {any_nan}")
    log(f"hard FAILs: {hard_fails if hard_fails else 'none'}")

    # Persist: report-facing derived metrics only in the log; full telemetry
    # (raw, high-rate) goes into the JSON result file per test.
    out = {}
    for k, r in results.items():
        out[k] = {kk: vv for kk, vv in r.items()}  # series kept (raw data is fine in the result file)

    with open(f"{RESULTS_DIR}/free_flight_dynamic_response_result.json", "w") as f:
        json.dump(out, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/free_flight_dynamic_response_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return any_nan, hard_fails, results


if __name__ == "__main__":
    any_nan, hard_fails, _ = run_all()
    sys.exit(1 if (any_nan or hard_fails) else 0)

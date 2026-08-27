#!/usr/bin/env python3
"""
FALCON V2 - FLIGHT_ENVELOPE_VALIDATION (gazebo-testing, 2026-08-27).

Validates the CURRENT, already-implemented physics model (aero lookup +
high-alpha smooth-saturation limiter, propulsion, actuators) across a
speed/alpha envelope. Does NOT retune any coefficient/lookup/limiter/
propulsion/actuator/inertia/CG/mass parameter - see CLAUDE.md's hard
constraint and this task's own brief.

Reuses (does not reinvent) the methodology and helpers already established
by tests/gazebo/scripts/test_updated_powered_trim_high_deflection.py (PREV
below): its pure-Python `predict_aero()` mirror of AeroModel.hh::ComputeAero()
(control-surface wide-deflection lookup + high-alpha smooth saturation +
resolved FLU Cm-to-My sign correction), its `REF` aero-config mirror, its
`get_model()`/`quat_rpy()`/`actual_deltas()` helpers, and actuator_lib.py/
aero_lib.py/propulsion_lib.py's ActuatorCommander/ThrottleCommander/
DiagSubscriber/hold_step/pin_control_surface_joints primitives. No new
methodology is invented - only the SAME "hold briefly at a target body-frame
condition then release" / "hold through the real actuator, quasi-static
isolation" / "short free 6-DOF flight" / "short pulse, real actuator"
techniques already validated in prior stages, now applied across 8 target
airspeeds instead of a single one.

=============================================================================
CRITICAL MODEL LIMITATION (per this task's brief, docs/source_of_truth/
aerodynamics/AERODYNAMICS.md sec 11): this model has NO real stall/post-stall
physics. `SaturatedCL()` smoothly asymptotes toward CLmax=1.42 (a
manufacturer performance-calc input, NOT flight-validated) as alpha grows
past alpha_transition=9.25deg, but NEVER drops CL post-stall the way a real
wing would. Any trim this script's search finds at or near/below the master
dataset's own Vstall~=12.24 m/s estimate is explicitly labeled
MODEL_PREDICTED_ONLY_NO_STALL_PHYSICS below and must never be read as
evidence the real aircraft can fly there.
=============================================================================

PART 1 - offline analytical solve (pure Python, no Gazebo; used ONLY to
center/seed each live per-speed search - never as the reported trim itself,
exactly the role the prior stage's own "analytical center" technique played,
here extended to jointly solve alpha/delta_e (lift+moment balance, using the
REAL nonlinear lookup + saturation curve, not a small-signal linearization)
and throttle (thrust=drag, via propulsion_lib's own pure-math primitives -
a bisection root-find on the rotor's steady-state torque balance, then a
bisection root-find on throttle - the same primitives test_propulsion_
operating_points.py already uses for a different purpose). The live Gazebo
search below (a bounded Hooke-Jeeves-style "+"-pattern coarse->refine local
search, capped rounds - NOT a brute-force grid) then confirms/refines this
per the task's explicit instruction to march outward from the already-
validated 18.166 m/s trim in both directions, seeding each new speed from
its nearest already-solved neighbor (not restarting from scratch).

PART 2/3 - once a speed's kinematically-pinned search converges, the
selected (throttle, elevator) is re-validated through the REAL actuator
(quasi-static isolation, KP_ANG_QSTATIC=1500, same technique as PREV's
run_actuator_quasi_static) to obtain the AUTHORITATIVE, live-plugin-measured
CL/CD/Cm/Cl/Cn, propulsion diagnostics, and actuator tracking - this is the
officially reported trim-point data (not the pinned-joint search's own
Python-mirror numbers, which exist only to drive the search efficiently).

PART 4 - a short static alpha sweep (7-20deg) at fixed V, elevator=0,
through the real actuator/live diagnostics, confirming the high-alpha
limiter behaves as documented (finite, monotonic-non-decreasing CL, no
kink/discontinuity in slope at alpha_transition, no NaN, no sign reversal)
at more extreme alphas than previously exercised - NOT a limiter redesign.

PART 6 - 3 short (~13-14s) fully-free 6DOF flights (LOW/NOMINAL/HIGH,
same "hold briefly then fully release, nothing else touches base_link"
technique as PREV Part 4).

PART 7 - a representative +/-5deg quasi-static-command control-authority
check per surface at the same 3 speeds (short hold-then-release-and-sample-
early technique, same pattern as PREV's pulse tests, just a single smaller
deflection instead of the +/-15/+/-25deg high-deflection sweep already
validated last stage).

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
control lookup table, control derivative, high-alpha limiter constant,
propulsion parameter, actuator parameter) is read for any purpose other than
loading the existing config/state, and NONE is modified anywhere in this
script.
"""
import json
import math
import sys

import test_updated_powered_trim_high_deflection as PREV  # noqa: E402 (env/plugin-path setup happens on import)

ACT = PREV.ACT
AL = PREV.AL
PL = PREV.PL
sim = PREV.sim
gm = PREV.gm
REF = PREV.REF
get_model = PREV.get_model
quat_rpy = PREV.quat_rpy
actual_deltas = PREV.actual_deltas
predict_aero = PREV.predict_aero
interp_lin = PREV.interp_lin
saturated_CL = PREV.saturated_CL

REPO_ROOT = PREV.REPO_ROOT
RESULTS_DIR = PREV.RESULTS_DIR
WORLD = PREV.WORLD
MASS = PREV.MASS
I_DIAG = PREV.I_DIAG
KP_LIN = PREV.KP_LIN
KP_ANG_SETTLE = PREV.KP_ANG_SETTLE
KP_ANG_QSTATIC = PREV.KP_ANG_QSTATIC
ALTITUDE_M = PREV.ALTITUDE_M
WEIGHT_N = PREV.WEIGHT_N
DZ_HUB_CG = PREV.DZ_HUB_CG
DIAG_HZ = PREV.DIAG_HZ

RHO = REF["rho"]
S = REF["S"]
ALPHA_TRANSITION_DEG = math.degrees(REF["alphaTransition"])
CLMAX = REF["CLmax"]

# Already-validated reference trim @ V=18.166 m/s (2026-08-27,
# UPDATED_POWERED_TRIM_AND_HIGH_DEFLECTION_FLIGHT_VALIDATION) - reused
# verbatim, NOT re-searched, per this task's own instruction.
U_HOLD_REF, W_HOLD_REF = PREV.U_HOLD, PREV.W_HOLD  # V=18.162 m/s, alpha=2.472deg (established hold baseline)
TRIM_REF = dict(throttle=0.5010, elevator_aero_deg=-4.50, elevator_theta_deg=4.50)
V_REF = 18.166

TARGET_SPEEDS = [12.5, 14.0, 16.0, V_REF, 21.0, 24.0, 28.0, 30.0]

THROTTLE_MIN, THROTTLE_MAX = 0.05, 1.0
ELEV_MIN, ELEV_MAX = -30.0, 15.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# =============================================================================
# PART 1a - offline analytical trim solver (pure Python, no Gazebo).
# =============================================================================
PCFG = PL.load_config()
APC_SLICES = PL.load_apc_table(PCFG.apc_parsed_csv_path)


ALPHA_BISECT_HI = math.radians(30.0)


def solve_alpha_delta_e(CL_required, cmStatic_target, elev_lo_deg=-30.0, elev_hi_deg=15.0, n_grid=121):
    """Solve (alpha_rad, delta_e_aero_deg) s.t. SaturatedCL(alpha)+dCLeCtrl(delta_e)=CL_required
    and Cm0+Cma*alpha+dCmeCtrl(delta_e)=cmStatic_target, using the REAL nonlinear
    lookup/saturation curve (not a small-signal linearization). Robust grid-scan +
    bisection (no monotonicity assumed a priori across the whole elevator range -
    only within the located bracket).

    IMPORTANT (found during this task's own sanity-check, before any live Gazebo
    run - a genuine model-feasibility finding, not a search bug): for a given
    delta_e, `target_sat = CL_required - dCL(delta_e)` can exceed CLmax (the
    saturation curve's own asymptote), in which case NO alpha ever satisfies the
    lift equation - the inner alpha-bisection then just pins at its own upper
    search bound (ALPHA_BISECT_HI) rather than a real equilibrium. Such grid
    points are explicitly marked INFEASIBLE (`feasible=False`) and excluded from
    both the bracket search and the "best effort" fallback, so a CL-infeasible
    point can never be silently reported as if it were a real solution. Returns
    a 3rd value, `status`:
      "OK"                        - a genuine sign-change bracket found within
                                     the CL-feasible sub-range; alpha/delta_e are
                                     the bisected root.
      "AERODYNAMIC_NO_TRIM_MOMENT" - CL is achievable somewhere in the domain,
                                     but no delta_e within the CL-feasible
                                     sub-range also zeros the pitching moment
                                     (elevator's own moment authority is
                                     insufficient without violating CL
                                     feasibility) - returns the CL-feasible
                                     point with the smallest |cm residual| as a
                                     documented best-effort seed only.
      "CL_INFEASIBLE_EVERYWHERE"  - CL_required itself exceeds what SaturatedCL
                                     plus the elevator's own dCL range can ever
                                     produce anywhere in the swept domain.
    """
    def alpha_for_delta_e(delta_e_deg):
        delta_e_rad = math.radians(delta_e_deg)
        dCL = interp_lin(REF["bps"], REF["elev_dCL"], delta_e_rad)
        target_sat = CL_required - dCL
        lo, hi = math.radians(-8.0), ALPHA_BISECT_HI
        for _ in range(60):
            mid = (lo + hi) / 2.0
            if saturated_CL(REF, mid) < target_sat:
                lo = mid
            else:
                hi = mid
        alpha = (lo + hi) / 2.0
        feasible = target_sat <= CLMAX + 1e-6
        return alpha, feasible

    def cm_residual(delta_e_deg):
        alpha, feasible = alpha_for_delta_e(delta_e_deg)
        dCm = interp_lin(REF["bps"], REF["elev_dCm"], math.radians(delta_e_deg))
        cmStatic = REF["Cm0"] + REF["Cma"] * alpha + dCm
        return cmStatic - cmStatic_target, alpha, feasible

    grid = [elev_lo_deg + i * (elev_hi_deg - elev_lo_deg) / (n_grid - 1) for i in range(n_grid)]
    residuals = [cm_residual(d) for d in grid]
    feasible_idx = [i for i in range(n_grid) if residuals[i][2]]

    if not feasible_idx:
        best_i = min(range(n_grid), key=lambda i: abs(residuals[i][0]))
        return residuals[best_i][1], grid[best_i], "CL_INFEASIBLE_EVERYWHERE"

    bracket = None
    for k in range(len(feasible_idx) - 1):
        i, j = feasible_idx[k], feasible_idx[k + 1]
        if residuals[i][0] == 0.0:
            bracket = (i, i)
            break
        if residuals[i][0] * residuals[j][0] < 0.0:
            bracket = (i, j)
            break

    if bracket is None:
        best_i = min(feasible_idx, key=lambda i: abs(residuals[i][0]))
        return residuals[best_i][1], grid[best_i], "AERODYNAMIC_NO_TRIM_MOMENT"

    lo_d, hi_d = grid[bracket[0]], grid[bracket[1]]
    lo_r = residuals[bracket[0]][0]
    for _ in range(60):
        mid_d = (lo_d + hi_d) / 2.0
        mid_r, _, _ = cm_residual(mid_d)
        if lo_r * mid_r <= 0.0:
            hi_d = mid_d
        else:
            lo_d, lo_r = mid_d, mid_r
    delta_e_final = (lo_d + hi_d) / 2.0
    _, alpha_final, _ = cm_residual(delta_e_final)
    return alpha_final, delta_e_final, "OK"


def steady_state_rpm(throttle, v_axial, iters=60):
    omega_lo = 1.0
    omega_hi = (PCFG.rpm_cap_v1 * 1.05) * 2.0 * math.pi / 60.0

    def f(omega):
        me = PL.motor_electrical(PCFG, throttle, PCFG.v1_operating_voltage_V, omega)
        load = PL.prop_aero_load(APC_SLICES, omega, v_axial, PCFG.diameter_m, PCFG.rho, PCFG.n_safe_floor_rev_s)
        return me["torque_Nm"] - load["qPropSigned_Nm"]

    lo, hi = omega_lo, omega_hi
    flo, fhi = f(lo), f(hi)
    if flo * fhi > 0.0:
        omega_ss = lo if abs(flo) < abs(fhi) else hi
    else:
        for _ in range(iters):
            mid = (lo + hi) / 2.0
            fm = f(mid)
            if flo * fm <= 0.0:
                hi, fhi = mid, fm
            else:
                lo, flo = mid, fm
        omega_ss = (lo + hi) / 2.0
    me = PL.motor_electrical(PCFG, throttle, PCFG.v1_operating_voltage_V, omega_ss)
    load = PL.prop_aero_load(APC_SLICES, omega_ss, v_axial, PCFG.diameter_m, PCFG.rho, PCFG.n_safe_floor_rev_s)
    return dict(omega=omega_ss, rpm=omega_ss * 60.0 / (2.0 * math.pi),
                thrust_N=load["thrust_N"], current_A=me["current_A"])


def solve_throttle_for_thrust(drag_target, v_axial, lo=0.05, hi=1.0, iters=40):
    def f(th):
        return 2.0 * steady_state_rpm(th, v_axial)["thrust_N"] - drag_target
    flo, fhi = f(lo), f(hi)
    if flo > 0.0:
        return lo, "THROTTLE_FLOOR"
    if fhi < 0.0:
        return hi, "PROPULSION_LIMITED"
    for _ in range(iters):
        mid = (lo + hi) / 2.0
        fm = f(mid)
        if flo * fm <= 0.0:
            hi, fhi = mid, fm
        else:
            lo, flo = mid, fm
    return (lo + hi) / 2.0, "OK"


def analytical_trim_guess(V, iters=3):
    """Offline solve (Lift=Weight, aero+propulsion pitch moment=0, Thrust=Drag) -
    used ONLY to center the live search below, never as the reported trim."""
    qbar = 0.5 * RHO * V * V
    CL_req = WEIGHT_N / (qbar * S)
    cm_offset_Nm = 0.0
    alpha = delta_e_deg = throttle = drag = thrust_total = u = w = None
    pred = None
    prop_status = "OK"
    aero_status = "OK"
    for _ in range(iters):
        cmStatic_target = cm_offset_Nm / (qbar * S * REF["c_ref"])
        alpha, delta_e_deg, aero_status = solve_alpha_delta_e(CL_req, cmStatic_target)
        u = V * math.cos(alpha)
        w = -V * math.sin(alpha)
        pred = predict_aero(REF, u, 0.0, w, 0.0, 0.0, 0.0, deltaE=math.radians(delta_e_deg))
        drag = pred["Drag"]
        throttle, prop_status = solve_throttle_for_thrust(drag, u)
        ss = steady_state_rpm(throttle, u)
        thrust_total = 2.0 * ss["thrust_N"]
        cm_offset_Nm = DZ_HUB_CG * thrust_total
    status = aero_status if aero_status != "OK" else prop_status
    return dict(V=V, alpha_deg=math.degrees(alpha), delta_e_aero_deg=delta_e_deg,
                u=u, w=w, throttle=throttle, drag_N=drag, thrust_total_N=thrust_total,
                CL=pred["CL"], CD=pred["CD"], status=status, aero_status=aero_status, prop_status=prop_status)


# =============================================================================
# PART 1b - live Gazebo trim candidate (kinematically-pinned elevator, real
# throttle/propulsion) - generalized version of PREV.run_trim_candidate,
# parameterized by (u_hold, w_hold) instead of a single fixed module baseline.
# =============================================================================
HOLD_STEPS = 800


def run_trim_candidate_generic(log, u_hold, w_hold, throttle, elevator_aero_deg, window_steps, verbose=True):
    elevator_theta_rad = math.radians(-elevator_aero_deg)  # elevatorSign=-1.0, symmetric L=R

    state = {"n": 0, "teleported": False, "series": [], "any_nan": False,
             "release_sample": None, "throttle_cmd": None, "prop_diag": None, "aero_diag": None}

    def on_pre(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, ALTITUDE_M, 0, 0, 0))
            state["teleported"] = True
            state["throttle_cmd"] = PL.ThrottleCommander()
        state["throttle_cmd"].set(left=throttle, right=throttle)
        state["throttle_cmd"].tick()
        PL.pin_control_surface_joints(model, ecm, sim, positions={
            "left_elevator_joint": elevator_theta_rad, "right_elevator_joint": elevator_theta_rad})
        n = state["n"]
        if n < HOLD_STEPS:
            AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(u_hold, 0, w_hold),
                         gm.Vector3d(0, 0, 0), kp_lin=KP_LIN, kp_ang=KP_ANG_SETTLE)

    def on_post(info, ecm):
        if state["prop_diag"] is None:
            try:
                state["prop_diag"] = PL.DiagSubscriber()
            except Exception:
                pass
        if state["aero_diag"] is None:
            try:
                state["aero_diag"] = AL.DiagSubscriber()
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
            state["n"] += 1
            return
        rot = wpose.rot()
        lv_b = rot.rotate_vector_reverse(lv)
        av_b = rot.rotate_vector_reverse(av)
        vals = [lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z(), wpose.pos().z(), lv.z()]
        if any(math.isnan(x) or math.isinf(x) for x in vals):
            state["any_nan"] = True
        _, pitch, _ = quat_rpy(rot)
        sample = dict(n=n, t=n * AL.STEP, u=lv_b.x(), v=lv_b.y(), w=lv_b.z(),
                      p=av_b.x(), q=av_b.y(), r=av_b.z(), alt=wpose.pos().z(), world_vz=lv.z(), pitch=pitch)
        if n == HOLD_STEPS:
            sample["prop"] = state["prop_diag"].latest() if state["prop_diag"] else None
            sample["aero"] = state["aero_diag"].latest() if state["aero_diag"] else None
        if n >= HOLD_STEPS:
            state["series"].append(sample)
        if n == HOLD_STEPS:
            state["release_sample"] = sample
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, HOLD_STEPS + window_steps, False)

    rel = state["release_sample"]
    series = state["series"]
    win_end = series[-1]

    pred_rel = predict_aero(REF, rel["u"], rel["v"], rel["w"], rel["p"], rel["q"], rel["r"],
                             deltaE=math.radians(elevator_aero_deg))
    V_rel = pred_rel["V"]
    alpha_rel_deg = math.degrees(pred_rel["alpha"])

    half_s_idx = min(500, len(series) - 1)
    u0, u_half = series[0]["u"], series[half_s_idx]["u"]
    dt_half = series[half_s_idx]["t"] - series[0]["t"]
    long_accel = (u_half - u0) / dt_half if dt_half else 0.0

    V_end = math.sqrt(win_end["u"] ** 2 + win_end["v"] ** 2 + win_end["w"] ** 2)
    airspeed_drift = V_end - V_rel
    mean_world_vz = sum(s["world_vz"] for s in series) / len(series)
    mean_abs_q_deg_s = sum(abs(math.degrees(s["q"])) for s in series) / len(series)
    alt_drift = win_end["alt"] - rel["alt"]

    tail_start_idx = int(len(series) * 0.7)
    tail = series[tail_start_idx:]
    tail_mean_world_vz = sum(s["world_vz"] for s in tail) / len(tail)
    dt_tail = tail[-1]["t"] - tail[0]["t"]
    tail_alt_drift_rate_mps = (tail[-1]["alt"] - tail[0]["alt"]) / dt_tail if dt_tail else 0.0

    prop = rel.get("prop")
    left_rpm = prop["left"]["rpm"] if prop else None
    right_rpm = prop["right"]["rpm"] if prop else None
    left_thrust = prop["left"]["thrust_N"] if prop else None
    right_thrust = prop["right"]["thrust_N"] if prop else None
    thrust_total = (left_thrust + right_thrust) if (left_thrust is not None and right_thrust is not None) else None
    rpm_cap_active = bool(prop["left"]["rpmCapActive"] or prop["right"]["rpmCapActive"]) if prop else False
    current_limited = bool(prop["left"]["currentLimited"] or prop["right"]["currentLimited"]) if prop else False

    result = dict(
        u_hold=u_hold, w_hold=w_hold, throttle=throttle, elevator_aero_deg=elevator_aero_deg,
        elevator_theta_deg=math.degrees(elevator_theta_rad),
        release=dict(V=V_rel, alpha_deg=alpha_rel_deg, world_vz=rel["world_vz"], q_deg_s=math.degrees(rel["q"])),
        aero_rel=dict(Lift_N=pred_rel["Lift"], Drag_N=pred_rel["Drag"], My_Nm=pred_rel["My"],
                      CL=pred_rel["CL"], CD=pred_rel["CD"], Cm_diag=pred_rel["Cm"]),
        prop_rel=dict(left_rpm=left_rpm, right_rpm=right_rpm, left_thrust_N=left_thrust, right_thrust_N=right_thrust,
                      thrust_total_N=thrust_total, rpm_cap_active=rpm_cap_active, current_limited=current_limited),
        long_accel_mps2=long_accel,
        window=dict(V_end=V_end, airspeed_drift_mps=airspeed_drift, mean_world_vz_mps=mean_world_vz,
                    mean_abs_q_deg_s=mean_abs_q_deg_s, alt_drift_m=alt_drift, duration_s=window_steps * AL.STEP,
                    tail_mean_world_vz_mps=tail_mean_world_vz, tail_alt_drift_rate_mps=tail_alt_drift_rate_mps),
        any_nan=state["any_nan"],
    )
    if verbose:
        log(f"    cand th={throttle:.4f} elev_aero={elevator_aero_deg:+.2f}deg -> V={V_rel:.3f} alpha={alpha_rel_deg:.3f}deg "
            f"Lift={pred_rel['Lift']:.2f}N Drag={pred_rel['Drag']:.2f}N My={pred_rel['My']:+.3f}Nm "
            f"ThrustTot={thrust_total:.2f}N tail_vz={tail_mean_world_vz:+.4f} drift={airspeed_drift:+.3f} "
            f"mean|q|={mean_abs_q_deg_s:.2f} any_nan={state['any_nan']}")
    return result


def candidate_score(r):
    vz_bucket = round(abs(r["window"]["mean_world_vz_mps"]) / 0.3)
    return (vz_bucket, abs(r["window"]["airspeed_drift_mps"]), r["window"]["mean_abs_q_deg_s"], abs(r["aero_rel"]["My_Nm"]))


COARSE_WINDOW = 1500
CONFIRM_WINDOW = 5000


def local_trim_search(log, u_hold, w_hold, seed_throttle, seed_elevator, counter,
                       th_steps=(0.02, 0.008), el_steps=(0.5, 0.2), max_rounds=2):
    """Bounded Hooke-Jeeves-style '+'-pattern coarse->refine local search
    (NOT a brute-force grid): evaluates center + 4 neighbors per round at a
    given step size, moves the center to the best result, repeats (capped at
    max_rounds) until no improvement or the cap is hit, then repeats once
    more with the next (smaller) step size. Finishes with one longer-window
    confirm run at the final center."""
    all_candidates = []

    def evaluate(th, el, window_steps=COARSE_WINDOW):
        th = clamp(th, THROTTLE_MIN, THROTTLE_MAX)
        el = clamp(el, ELEV_MIN, ELEV_MAX)
        r = run_trim_candidate_generic(log, u_hold, w_hold, th, el, window_steps=window_steps)
        all_candidates.append(r)
        counter[0] += 1
        return r

    center = evaluate(seed_throttle, seed_elevator)
    for dth, del_ in zip(th_steps, el_steps):
        for _ in range(max_rounds):
            neighbor_keys = [(center["throttle"] + dth, center["elevator_aero_deg"]),
                              (center["throttle"] - dth, center["elevator_aero_deg"]),
                              (center["throttle"], center["elevator_aero_deg"] + del_),
                              (center["throttle"], center["elevator_aero_deg"] - del_)]
            neighbors = [evaluate(t, e) for t, e in neighbor_keys]
            pool = [center] + neighbors
            best = min(pool, key=candidate_score)
            if best is center:
                break
            center = best
    confirm = evaluate(center["throttle"], center["elevator_aero_deg"], window_steps=CONFIRM_WINDOW)
    return dict(center=confirm, all_candidates=all_candidates)


def classify_trim(confirm, alpha_deg):
    if confirm["any_nan"]:
        return "NO_VALID_TRIM", None, {}
    lift = confirm["aero_rel"]["Lift_N"]
    lw_ratio = lift / WEIGHT_N
    drag = confirm["aero_rel"]["Drag_N"]
    thrust = confirm["prop_rel"]["thrust_total_N"] or 0.0
    td_ratio = thrust / drag if drag else float("nan")
    tail_vz = confirm["window"]["tail_mean_world_vz_mps"]
    tail_alt_rate = confirm["window"]["tail_alt_drift_rate_mps"]
    mean_q = confirm["window"]["mean_abs_q_deg_s"]
    my = confirm["aero_rel"]["My_Nm"]

    if abs(lw_ratio - 1.0) <= 0.05 and abs(tail_alt_rate) <= 0.30 and mean_q <= 3.0:
        classification = "TRIM_FOUND"
    elif abs(lw_ratio - 1.0) <= 0.12 and abs(tail_alt_rate) <= 0.8 and mean_q <= 8.0:
        classification = "MARGINAL_TRIM"
    else:
        classification = "NO_VALID_TRIM"

    sub_flag = "MODEL_PREDICTED_ONLY_NO_STALL_PHYSICS" if alpha_deg >= (ALPHA_TRANSITION_DEG - 0.5) else None
    metrics = dict(lw_ratio=lw_ratio, td_ratio=td_ratio, tail_vz=tail_vz, tail_alt_rate=tail_alt_rate,
                   mean_q=mean_q, my=my)
    return classification, sub_flag, metrics


# =============================================================================
# PART 1c - marching orchestration across the 8 target speeds.
# =============================================================================
def run_part1(log):
    counter = [0]
    results_by_speed = {}

    analytical_ref = analytical_trim_guess(V_REF)
    log(f"Analytical cross-check @ V={V_REF} m/s: alpha={analytical_ref['alpha_deg']:.2f}deg "
        f"delta_e_aero={analytical_ref['delta_e_aero_deg']:+.2f}deg throttle={analytical_ref['throttle']:.4f} "
        f"status={analytical_ref['status']} (vs the already-validated LIVE trim throttle={TRIM_REF['throttle']:.4f} "
        f"elevator_aero={TRIM_REF['elevator_aero_deg']:+.2f}deg - NOT re-searched, reused verbatim).")
    confirm_ref = run_trim_candidate_generic(log, U_HOLD_REF, W_HOLD_REF, TRIM_REF["throttle"],
                                              TRIM_REF["elevator_aero_deg"], window_steps=CONFIRM_WINDOW)
    counter[0] += 1
    cls, sub, metrics = classify_trim(confirm_ref, confirm_ref["release"]["alpha_deg"])
    results_by_speed[V_REF] = dict(target_V=V_REF, analytical=analytical_ref, seed=TRIM_REF,
                                    confirm=confirm_ref, all_candidates=[confirm_ref],
                                    classification=cls, sub_flag=sub, metrics=metrics,
                                    u_hold=U_HOLD_REF, w_hold=W_HOLD_REF)
    log(f"--> V={V_REF}: reused reference trim confirmed. classification={cls} Lift/W={metrics['lw_ratio']:.4f} "
        f"tail_vz={metrics['tail_vz']:+.4f}\n")

    def march(speed_list, reference_speed):
        prev_speed = reference_speed
        for V in speed_list:
            analytical = analytical_trim_guess(V)
            prev = results_by_speed[prev_speed]
            prev_analytical = prev["analytical"]
            corr_th = prev["confirm"]["throttle"] - prev_analytical["throttle"]
            corr_el = prev["confirm"]["elevator_aero_deg"] - prev_analytical["delta_e_aero_deg"]
            seed_th = clamp(analytical["throttle"] + corr_th, THROTTLE_MIN, THROTTLE_MAX)
            seed_el = clamp(analytical["delta_e_aero_deg"] + corr_el, ELEV_MIN, ELEV_MAX)
            log("=" * 78)
            log(f"TRIM SEARCH @ V_target={V:.3f} m/s (analytical: alpha={analytical['alpha_deg']:.2f}deg "
                f"delta_e={analytical['delta_e_aero_deg']:+.2f}deg throttle={analytical['throttle']:.4f} "
                f"status={analytical['status']}; march-seed from V={prev_speed:.3f}: "
                f"throttle={seed_th:.4f} elevator={seed_el:+.2f}deg)")
            log("=" * 78)
            u_hold, w_hold = analytical["u"], analytical["w"]
            search = local_trim_search(log, u_hold, w_hold, seed_th, seed_el, counter)
            confirm = search["center"]
            cls, sub, metrics = classify_trim(confirm, confirm["release"]["alpha_deg"])
            log(f"--> V={V}: SELECTED throttle={confirm['throttle']:.4f} "
                f"elevator_aero={confirm['elevator_aero_deg']:+.2f}deg classification={cls}"
                f"{' [' + sub + ']' if sub else ''} Lift/W={metrics['lw_ratio']:.4f} T/D={metrics['td_ratio']:.4f} "
                f"tail_vz={metrics['tail_vz']:+.4f} ({len(search['all_candidates'])} candidates this speed)\n")
            results_by_speed[V] = dict(target_V=V, analytical=analytical,
                                        seed=dict(throttle=seed_th, elevator_aero_deg=seed_el),
                                        confirm=confirm, all_candidates=search["all_candidates"],
                                        classification=cls, sub_flag=sub, metrics=metrics,
                                        u_hold=u_hold, w_hold=w_hold)
            prev_speed = V

    march([16.0, 14.0, 12.5], V_REF)
    march([21.0, 24.0, 28.0, 30.0], V_REF)

    log(f"\nTotal Part-1 trim candidates tested (aggregate, incl. the V={V_REF} confirm): {counter[0]}")
    return results_by_speed, counter[0]


# =============================================================================
# PART 2/3/8/9 - real-actuator quasi-static confirm at each speed's selected
# trim (authoritative live-plugin CL/CD/Cm + propulsion + actuator
# diagnostics). Generalized version of PREV.run_actuator_quasi_static.
# =============================================================================
WARM_STEPS = 300
SETTLE_STEPS = 1800
TAIL_STEPS = 400


def run_actuator_hold_generic(log, label, u_hold, w_hold, throttle, elev_theta_deg,
                               aile_L_deg=0.0, aile_R_deg=0.0, rudder_deg=0.0,
                               warm_steps=WARM_STEPS, settle_steps=SETTLE_STEPS, tail_steps=TAIL_STEPS):
    cmd_rad = dict(left_elevator=math.radians(elev_theta_deg), right_elevator=math.radians(elev_theta_deg),
                   left_aileron=math.radians(aile_L_deg), right_aileron=math.radians(aile_R_deg),
                   rudder=math.radians(rudder_deg))
    lin_target = gm.Vector3d(u_hold, 0.0, w_hold)

    state = {"n": 0, "teleported": False, "cmd": None, "thr": None, "any_nan": False,
             "aero_diag": None, "prop_diag": None, "actuator_diag": None,
             "theta": {s: [] for s in ACT.SURFACES}, "body_state": []}
    total_steps = warm_steps + settle_steps + tail_steps + 5

    def on_pre(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, ALTITUDE_M, 0, 0, 0))
            state["teleported"] = True
            state["cmd"] = ACT.ActuatorCommander()
            state["thr"] = PL.ThrottleCommander()
        state["thr"].set(left=throttle, right=throttle)
        state["thr"].tick()
        state["cmd"].set(**cmd_rad)
        state["cmd"].tick()
        AL.hold_step(base, ecm, MASS, I_DIAG, lin_target, gm.Vector3d(0, 0, 0), kp_lin=KP_LIN, kp_ang=KP_ANG_QSTATIC)

    def on_post(info, ecm):
        if state["aero_diag"] is None:
            try:
                state["aero_diag"] = AL.DiagSubscriber()
            except Exception:
                pass
        if state["prop_diag"] is None:
            try:
                state["prop_diag"] = PL.DiagSubscriber()
            except Exception:
                pass
        if state["actuator_diag"] is None:
            try:
                state["actuator_diag"] = ACT.DiagSubscriber()
            except Exception:
                pass
        model = get_model(ecm)
        for s in ACT.SURFACES:
            th, _ = ACT.read_joint_state(model, ecm, sim, ACT.JOINT_NAMES[s])
            state["theta"][s].append(th if th is not None else float("nan"))
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        wpose = base.world_pose(ecm)
        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        if lv is None or av is None or wpose is None or any(
                math.isnan(v) or math.isinf(v) for v in [lv.x(), lv.y(), lv.z(), av.x(), av.y(), av.z()]):
            state["any_nan"] = True
        else:
            rot = wpose.rot()
            lv_b = rot.rotate_vector_reverse(lv)
            av_b = rot.rotate_vector_reverse(av)
            state["body_state"].append((lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z()))
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    bs_tail = state["body_state"][-tail_steps:]
    u_m = sum(s[0] for s in bs_tail) / len(bs_tail)
    v_m = sum(s[1] for s in bs_tail) / len(bs_tail)
    w_m = sum(s[2] for s in bs_tail) / len(bs_tail)
    p_m = sum(s[3] for s in bs_tail) / len(bs_tail)
    q_m = sum(s[4] for s in bs_tail) / len(bs_tail)
    r_m = sum(s[5] for s in bs_tail) / len(bs_tail)

    tail_mean_rad = {s: sum(state["theta"][s][-tail_steps:]) / len(state["theta"][s][-tail_steps:]) for s in ACT.SURFACES}
    thetaLA, thetaRA = tail_mean_rad["left_aileron"], tail_mean_rad["right_aileron"]
    thetaLE, thetaRE = tail_mean_rad["left_elevator"], tail_mean_rad["right_elevator"]
    thetaRud = tail_mean_rad["rudder"]
    actual_delta_a = 0.5 * REF["aileronSign"] * (thetaRA - thetaLA)
    actual_delta_e = 0.5 * REF["elevatorSign"] * (thetaLE + thetaRE)
    actual_delta_r = REF["rudderSign"] * thetaRud

    aero_hist = state["aero_diag"].history if state["aero_diag"] else []
    tail_msgs = max(1, round(tail_steps * ACT.STEP * DIAG_HZ))
    aero_tail = aero_hist[-tail_msgs:] if aero_hist else []
    aero_avg = ({k: sum(m[k] for m in aero_tail) / len(aero_tail) for k in AL.DiagSubscriber.FIELDS}
                if aero_tail else {k: None for k in AL.DiagSubscriber.FIELDS})

    prop_hist_split = state["prop_diag"].all_split() if state["prop_diag"] else []
    prop_tail = prop_hist_split[-tail_msgs:] if prop_hist_split else []
    left_rpm = sum(p["left"]["rpm"] for p in prop_tail) / len(prop_tail) if prop_tail else None
    right_rpm = sum(p["right"]["rpm"] for p in prop_tail) / len(prop_tail) if prop_tail else None
    left_thrust = sum(p["left"]["thrust_N"] for p in prop_tail) / len(prop_tail) if prop_tail else None
    right_thrust = sum(p["right"]["thrust_N"] for p in prop_tail) / len(prop_tail) if prop_tail else None
    left_current = sum(p["left"]["current_A"] for p in prop_tail) / len(prop_tail) if prop_tail else None
    right_current = sum(p["right"]["current_A"] for p in prop_tail) / len(prop_tail) if prop_tail else None
    any_rpm_cap = any((p["left"]["rpmCapActive"] > 0.5 or p["right"]["rpmCapActive"] > 0.5) for p in prop_tail) if prop_tail else False
    any_current_limited = any((p["left"]["currentLimited"] > 0.5 or p["right"]["currentLimited"] > 0.5) for p in prop_tail) if prop_tail else False

    ad = state["actuator_diag"].latest() if state["actuator_diag"] else None
    any_target_clamp = any(ad[s]["target_clamp_active"] > 0.5 for s in ACT.SURFACES) if ad else False
    any_effort_clamp = any(ad[s]["effort_clamp_active"] > 0.5 for s in ACT.SURFACES) if ad else False
    tracking_err_deg = ({s: math.degrees(abs(ad[s]["setpoint_rad"] - ad[s]["actual_angle_rad"])) for s in ACT.SURFACES}
                         if ad else {})

    pred = predict_aero(REF, u_m, v_m, w_m, p_m, q_m, r_m, deltaA=actual_delta_a, deltaE=actual_delta_e, deltaR=actual_delta_r)

    result = dict(
        label=label, throttle=throttle, elev_theta_cmd_deg=elev_theta_deg,
        aile_L_cmd_deg=aile_L_deg, aile_R_cmd_deg=aile_R_deg, rudder_cmd_deg=rudder_deg,
        actual_delta_e_deg=math.degrees(actual_delta_e), actual_delta_a_deg=math.degrees(actual_delta_a),
        actual_delta_r_deg=math.degrees(actual_delta_r),
        actual_theta_deg={s: math.degrees(tail_mean_rad[s]) for s in ACT.SURFACES},
        aero_tail_avg=aero_avg, aero_tail_n_msgs=len(aero_tail),
        prop_tail=dict(left_rpm=left_rpm, right_rpm=right_rpm, left_thrust_N=left_thrust, right_thrust_N=right_thrust,
                       left_current_A=left_current, right_current_A=right_current,
                       any_rpm_cap=any_rpm_cap, any_current_limited=any_current_limited),
        pred_from_measured_state=pred,
        any_target_clamp=any_target_clamp, any_effort_clamp=any_effort_clamp,
        tracking_err_deg=tracking_err_deg, any_nan=state["any_nan"],
    )
    log(f"  [{label}] elev_cmd={elev_theta_deg:+.3f}deg aile(L/R)={aile_L_deg:+.2f}/{aile_R_deg:+.2f}deg "
        f"rudder={rudder_deg:+.2f}deg -> actual delta_e={result['actual_delta_e_deg']:+.4f} "
        f"delta_a={result['actual_delta_a_deg']:+.4f} delta_r={result['actual_delta_r_deg']:+.4f}deg | "
        f"CL={aero_avg['CL']:.5f} CD={aero_avg['CD']:.5f} Cm={aero_avg['Cm']:.5f} Cl={aero_avg['Cl']:.6f} "
        f"Cn={aero_avg['Cn']:.6f} | My={pred['My']:+.4f}Nm Mx={pred['Mx']:+.4f}Nm Mz={pred['Mz']:+.4f}Nm | "
        f"RPM(L/R)={left_rpm:.1f}/{right_rpm:.1f} Thrust(L/R)={left_thrust:.3f}/{right_thrust:.3f}N | "
        f"target_clamp={any_target_clamp} effort_clamp={any_effort_clamp} rpm_cap={any_rpm_cap} "
        f"cur_lim={any_current_limited} any_nan={state['any_nan']}")
    return result


def run_part2_3(log, results_by_speed):
    log("=" * 78)
    log("PART 2/3: REAL-ACTUATOR TRIM CONFIRM AT ALL 8 SPEEDS (authoritative live-plugin data)")
    log("=" * 78)
    actuator_confirms = {}
    for V in TARGET_SPEEDS:
        r = results_by_speed[V]
        confirm = r["confirm"]
        ah = run_actuator_hold_generic(log, f"V{V:g}", r["u_hold"], r["w_hold"], confirm["throttle"],
                                        confirm["elevator_theta_deg"])
        actuator_confirms[V] = ah
    return actuator_confirms


# =============================================================================
# PART 4 - high-alpha limiter static sweep (only meaningfully triggered by
# the low-speed region pushing alpha near/past alpha_transition=9.25deg).
# =============================================================================
def run_part4_high_alpha_check(log, trigger_V=15.0):
    log("=" * 78)
    log("PART 4: HIGH-ALPHA LIMITER STATIC CHECK (triggered by low-speed region)")
    log("=" * 78)
    alphas_deg = [7.0, 8.0, 9.0, 9.25, 9.5, 10.0, 12.0, 15.0, 20.0]
    points = []
    for a_deg in alphas_deg:
        a_rad = math.radians(a_deg)
        u = trigger_V * math.cos(a_rad)
        w = -trigger_V * math.sin(a_rad)
        r = run_actuator_hold_generic(log, f"ALPHA{a_deg:g}", u, w, throttle=0.0, elev_theta_deg=0.0,
                                       warm_steps=200, settle_steps=1000, tail_steps=200)
        points.append(dict(alpha_cmd_deg=a_deg, **r))

    CLs = [p["aero_tail_avg"]["CL"] for p in points]
    Cms = [p["aero_tail_avg"]["Cm"] for p in points]
    any_nan = any(p["any_nan"] for p in points) or any(c is None or math.isnan(c) for c in CLs)
    monotonic_CL = all(CLs[i + 1] >= CLs[i] - 1e-6 for i in range(len(CLs) - 1))
    max_CL = max(CLs)
    slopes = [(CLs[i + 1] - CLs[i]) / (alphas_deg[i + 1] - alphas_deg[i]) for i in range(len(CLs) - 1)]
    slope_jump_ok = all(abs(slopes[i + 1] - slopes[i]) < 0.05 for i in range(len(slopes) - 1))
    no_sign_reversal = all(c > 0 for c in CLs)

    log(f"CL by alpha: {list(zip(alphas_deg, [round(c, 5) for c in CLs]))}")
    log(f"Cm by alpha: {list(zip(alphas_deg, [round(c, 5) for c in Cms]))}")
    log(f"any_nan={any_nan} monotonic_CL={monotonic_CL} max_CL={max_CL:.5f} (CLmax_manufacturer={CLMAX}) "
        f"slope_jump_ok(C1-continuity)={slope_jump_ok} no_sign_reversal={no_sign_reversal}")
    verdict = "PASS" if (not any_nan and monotonic_CL and max_CL <= CLMAX * 1.001 and slope_jump_ok and no_sign_reversal) else "WATCH"
    log(f"HIGH_ALPHA_LIMITER behavior at extended alphas: {verdict}\n")
    return dict(points=[dict(alpha_cmd_deg=p["alpha_cmd_deg"], CL=p["aero_tail_avg"]["CL"], CD=p["aero_tail_avg"]["CD"],
                             Cm=p["aero_tail_avg"]["Cm"], any_nan=p["any_nan"]) for p in points],
                any_nan=any_nan, monotonic_CL=monotonic_CL, max_CL=max_CL, slope_jump_ok=slope_jump_ok,
                no_sign_reversal=no_sign_reversal, verdict=verdict)


# =============================================================================
# PART 6 - short free 6-DOF flight, generalized version of PREV.run_part4_free_flight.
# =============================================================================
def run_free_flight_generic(log, u_hold, w_hold, throttle, elev_theta_deg, hold_steps=800, release_steps=14000,
                             telemetry_every=100):
    cmd_rad = dict(left_elevator=math.radians(elev_theta_deg), right_elevator=math.radians(elev_theta_deg),
                   left_aileron=0.0, right_aileron=0.0, rudder=0.0)
    state = {"n": 0, "teleported": False, "cmd": None, "thr": None, "any_nan": False, "nan_step": None,
             "series": [], "prop_diag": None}

    def on_pre(info, ecm):
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, ALTITUDE_M, 0, 0, 0))
            state["teleported"] = True
            state["cmd"] = ACT.ActuatorCommander()
            state["thr"] = PL.ThrottleCommander()
        state["thr"].set(left=throttle, right=throttle)
        state["thr"].tick()
        state["cmd"].set(**cmd_rad)
        state["cmd"].tick()
        if n < hold_steps:
            AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(u_hold, 0, w_hold), gm.Vector3d(0, 0, 0),
                         kp_lin=KP_LIN, kp_ang=KP_ANG_SETTLE)

    def on_post(info, ecm):
        n = state["n"]
        if state["prop_diag"] is None:
            try:
                state["prop_diag"] = PL.DiagSubscriber()
            except Exception:
                pass
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
        raw_vals = [lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z(), wpose.pos().z()]
        if any(math.isnan(x) or math.isinf(x) for x in raw_vals):
            state["any_nan"] = True
            if state["nan_step"] is None:
                state["nan_step"] = n
        if n >= hold_steps and (n - hold_steps) % telemetry_every == 0:
            roll, pitch, yaw = quat_rpy(rot)
            V = math.sqrt(lv_b.x() ** 2 + lv_b.y() ** 2 + lv_b.z() ** 2)
            alpha = math.atan2(-lv_b.z(), lv_b.x())
            beta = math.atan2(lv_b.y(), math.hypot(lv_b.x(), lv_b.z()))
            prop = state["prop_diag"].latest() if state["prop_diag"] else None
            th, _ = ACT.read_joint_state(model, ecm, sim, ACT.JOINT_NAMES["left_elevator"])
            state["series"].append(dict(
                t=(n - hold_steps) * AL.STEP, V=V, alt=wpose.pos().z(), world_vz=lv.z(),
                roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch), yaw_deg=math.degrees(yaw),
                alpha_deg=math.degrees(alpha), beta_deg=math.degrees(beta),
                p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z()),
                elevator_actual_deg=math.degrees(th) if th is not None else None,
                left_rpm=(prop["left"]["rpm"] if prop else None), right_rpm=(prop["right"]["rpm"] if prop else None),
                left_thrust_N=(prop["left"]["thrust_N"] if prop else None),
                right_thrust_N=(prop["right"]["thrust_N"] if prop else None)))
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, hold_steps + release_steps, False)

    series = state["series"]
    start, end = series[0], series[-1]
    max_abs_pitch = max(abs(s["pitch_deg"]) for s in series)
    max_abs_roll = max(abs(s["roll_deg"]) for s in series)
    max_abs_yaw_drift = max(abs(s["yaw_deg"] - start["yaw_deg"]) for s in series)
    max_abs_alpha = max(abs(s["alpha_deg"]) for s in series)
    v_drift = end["V"] - start["V"]
    alt_drift = end["alt"] - start["alt"]

    log(f"any_nan={state['any_nan']} (first at step {state['nan_step']})")
    log(f"Airspeed: start={start['V']:.3f} end={end['V']:.3f} drift={v_drift:+.3f} m/s")
    log(f"Altitude: start={start['alt']:.2f} end={end['alt']:.2f} drift={alt_drift:+.2f} m")
    log(f"Pitch: start={start['pitch_deg']:+.2f} end={end['pitch_deg']:+.2f} max|pitch|={max_abs_pitch:.2f}deg")
    log(f"Roll: start={start['roll_deg']:+.2f} end={end['roll_deg']:+.2f} max|roll|={max_abs_roll:.2f}deg")
    log(f"Alpha: start={start['alpha_deg']:+.2f} end={end['alpha_deg']:+.2f} max|alpha|={max_abs_alpha:.2f}deg")

    classification = "FAIL"
    reason = ""
    if state["any_nan"]:
        reason = "NaN/Inf encountered"
    elif max_abs_pitch > 60.0 or max_abs_roll > 60.0 or max_abs_alpha > 25.0:
        reason = "unbounded/runaway attitude or alpha excursion"
    elif abs(v_drift) > 3.0 or abs(alt_drift) > 60.0:
        classification = "PASS_WITH_SMALL_DRIFT" if (abs(v_drift) < 6.0 and abs(alt_drift) < 120.0) else "FAIL"
        reason = "moderate airspeed/altitude drift, bounded"
    elif max_abs_pitch > 15.0 or max_abs_roll > 5.0 or abs(v_drift) > 0.8 or abs(alt_drift) > 15.0:
        classification = "PASS_WITH_SMALL_DRIFT"
        reason = "small natural oscillation/drift, bounded, no divergence"
    else:
        classification = "PASS"
        reason = "bounded, near-trim, no divergence"
    if classification == "FAIL" and not reason:
        reason = "see thresholds above"
    log(f"CLASSIFICATION: {classification} ({reason})\n")

    return dict(any_nan=state["any_nan"], nan_step=state["nan_step"], series=series,
                summary=dict(v_drift=v_drift, alt_drift=alt_drift, max_abs_pitch=max_abs_pitch,
                             max_abs_roll=max_abs_roll, max_abs_yaw_drift=max_abs_yaw_drift, max_abs_alpha=max_abs_alpha),
                classification=classification, reason=reason)


def choose_best(speeds, results_by_speed):
    def score(V):
        m = results_by_speed[V]["metrics"]
        return abs(m["lw_ratio"] - 1.0) + abs(m["tail_alt_rate"])
    return min(speeds, key=score)


def run_part6(log, results_by_speed):
    log("=" * 78)
    log("PART 6: REPRESENTATIVE FREE 6-DOF FLIGHT (LOW/NOMINAL/HIGH)")
    log("=" * 78)
    low_V = choose_best([14.0, 16.0], results_by_speed)
    high_V = choose_best([24.0, 28.0], results_by_speed)
    nominal_V = V_REF
    out = {}
    chosen = {"LOW": low_V, "NOMINAL": nominal_V, "HIGH": high_V}
    for label, V in chosen.items():
        r = results_by_speed[V]
        confirm = r["confirm"]
        log("-" * 78)
        log(f"{label}: V_target={V} m/s (throttle={confirm['throttle']:.4f} "
            f"elevator_theta={confirm['elevator_theta_deg']:+.3f}deg)")
        log("-" * 78)
        ff = run_free_flight_generic(log, r["u_hold"], r["w_hold"], confirm["throttle"], confirm["elevator_theta_deg"],
                                       release_steps=14000)
        out[label] = dict(V_target=V, **ff)
    return out, chosen


# =============================================================================
# PART 7 - representative +/-5deg control-authority check per surface, at the
# 3 speeds chosen for Part 6.
# =============================================================================
PULSE_HOLD_STEPS = 800
PULSE_PRE_STEPS = 200
PULSE_STEPS = 700
PULSE_TAIL_STEPS = 400
PULSE_TELEMETRY_EVERY = 5


def build_pulse_cmd(channel, pulse_deg, trim_theta_e_deg):
    cmd = dict(left_elevator=math.radians(trim_theta_e_deg), right_elevator=math.radians(trim_theta_e_deg),
               left_aileron=0.0, right_aileron=0.0, rudder=0.0)
    if channel == "elevator":
        trim_delta_e_aero = REF["elevatorSign"] * trim_theta_e_deg
        target_delta_e_aero = trim_delta_e_aero + pulse_deg
        theta = REF["elevatorSign"] * target_delta_e_aero
        cmd["left_elevator"] = math.radians(theta)
        cmd["right_elevator"] = math.radians(theta)
    elif channel == "aileron":
        cmd["left_aileron"] = math.radians(-pulse_deg)
        cmd["right_aileron"] = math.radians(pulse_deg)
    elif channel == "rudder":
        cmd["rudder"] = math.radians(pulse_deg)
    else:
        raise ValueError(channel)
    return cmd


def run_control_authority_test(log, channel, pulse_deg, u_hold, w_hold, throttle, trim_theta_e_deg):
    trim_cmd = build_pulse_cmd(channel, 0.0, trim_theta_e_deg)
    pulse_cmd = build_pulse_cmd(channel, pulse_deg, trim_theta_e_deg)
    bounds = (PULSE_HOLD_STEPS, PULSE_HOLD_STEPS + PULSE_PRE_STEPS,
              PULSE_HOLD_STEPS + PULSE_PRE_STEPS + PULSE_STEPS,
              PULSE_HOLD_STEPS + PULSE_PRE_STEPS + PULSE_STEPS + PULSE_TAIL_STEPS)
    total_steps = bounds[-1]
    state = {"n": 0, "teleported": False, "cmd": None, "thr": None, "any_nan": False, "series": []}

    def on_pre(info, ecm):
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, ALTITUDE_M, 0, 0, 0))
            state["teleported"] = True
            state["cmd"] = ACT.ActuatorCommander()
            state["thr"] = PL.ThrottleCommander()
        state["thr"].set(left=throttle, right=throttle)
        state["thr"].tick()
        if n < bounds[2]:
            state["cmd"].set(**(trim_cmd if n < bounds[1] else pulse_cmd))
        else:
            state["cmd"].set(**trim_cmd)
        state["cmd"].tick()
        if n < bounds[0]:
            AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(u_hold, 0, w_hold), gm.Vector3d(0, 0, 0),
                         kp_lin=KP_LIN, kp_ang=KP_ANG_SETTLE)

    def on_post(info, ecm):
        n = state["n"]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        wpose = base.world_pose(ecm)
        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        if wpose is None or lv is None or av is None:
            state["any_nan"] = True
            state["n"] += 1
            return
        rot = wpose.rot()
        lv_b = rot.rotate_vector_reverse(lv)
        av_b = rot.rotate_vector_reverse(av)
        raw_vals = [lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z()]
        if any(math.isnan(x) or math.isinf(x) for x in raw_vals):
            state["any_nan"] = True
        if n >= bounds[0] and (n - bounds[0]) % PULSE_TELEMETRY_EVERY == 0:
            da, de, dr, _ = actual_deltas(model, ecm)
            pred = predict_aero(REF, lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z(),
                                 deltaA=da, deltaE=de, deltaR=dr)
            phase = ("hold" if n < bounds[0] else "pre" if n < bounds[1] else "pulse" if n < bounds[2] else "tail")
            state["series"].append(dict(
                n=n, t=(n - bounds[0]) * AL.STEP, phase=phase,
                p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z()),
                actual_delta_e_deg=math.degrees(de), actual_delta_a_deg=math.degrees(da), actual_delta_r_deg=math.degrees(dr),
                CL=pred["CL"], CD=pred["CD"], CY=pred["CY"], Cl=pred["Cl"], Cm=pred["Cm"], Cn=pred["Cn"],
                Mx=pred["Mx"], My=pred["My"], Mz=pred["Mz"]))
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    series = state["series"]
    pre = [s for s in series if s["phase"] == "pre"]
    pulse = [s for s in series if s["phase"] == "pulse"]
    baseline = pre[-1] if pre else series[0]
    pulse_t0 = pulse[0]["t"] if pulse else 0.0
    early = [s for s in pulse if 0.05 <= (s["t"] - pulse_t0) <= 0.15]

    def avg(lst, key):
        vals = [s[key] for s in lst if s[key] is not None]
        return sum(vals) / len(vals) if vals else None

    keys_coeff = ["CL", "CD", "CY", "Cl", "Cm", "Cn", "Mx", "My", "Mz"]
    early_avg = {k: avg(early, k) for k in keys_coeff + ["actual_delta_e_deg", "actual_delta_a_deg",
                                                          "actual_delta_r_deg", "p_deg_s", "q_deg_s", "r_deg_s"]}
    delta = {k: (early_avg[k] - baseline[k]) for k in keys_coeff if early_avg[k] is not None and baseline.get(k) is not None}
    initial_rate = dict(p_deg_s=early_avg["p_deg_s"], q_deg_s=early_avg["q_deg_s"], r_deg_s=early_avg["r_deg_s"])
    actual_delta_key = {"elevator": "actual_delta_e_deg", "aileron": "actual_delta_a_deg", "rudder": "actual_delta_r_deg"}[channel]

    result = dict(channel=channel, pulse_deg=pulse_deg, any_nan=state["any_nan"], baseline=baseline,
                  early=early_avg, delta=delta, initial_rate=initial_rate)
    log(f"  [{channel} {pulse_deg:+.0f}deg] actual_delta={early_avg.get(actual_delta_key, float('nan')):+.3f}deg "
        f"dMx={delta.get('Mx', 0.0):+.4f} dMy={delta.get('My', 0.0):+.4f} dMz={delta.get('Mz', 0.0):+.4f} "
        f"dCl={delta.get('Cl', 0.0):+.6f} dCm={delta.get('Cm', 0.0):+.6f} dCn={delta.get('Cn', 0.0):+.6f} "
        f"initial(p/q/r)={initial_rate['p_deg_s']:+.2f}/{initial_rate['q_deg_s']:+.2f}/{initial_rate['r_deg_s']:+.2f}deg/s "
        f"any_nan={state['any_nan']}")
    return result


def run_part7(log, results_by_speed, part6_chosen):
    log("=" * 78)
    log("PART 7: CONTROL AUTHORITY VS SPEED (+/-5deg representative command)")
    log("=" * 78)
    out = {}
    for label, V in part6_chosen.items():
        r = results_by_speed[V]
        confirm = r["confirm"]
        log("-" * 78)
        log(f"{label} (V={V})")
        log("-" * 78)
        res = {}
        for channel in ("elevator", "aileron", "rudder"):
            res[channel] = run_control_authority_test(log, channel, 5.0, r["u_hold"], r["w_hold"],
                                                        confirm["throttle"], confirm["elevator_theta_deg"])
        out[label] = dict(V=V, **res)
    return out


# =============================================================================
# Orchestration
# =============================================================================
def strip_series(obj):
    if not isinstance(obj, dict):
        return obj
    out = dict(obj)
    if "series" in out:
        out["series_n"] = len(out.get("series", []))
        out.pop("series", None)
    return out


def main():
    log_lines = []

    def log(msg):
        print(msg, flush=True)
        log_lines.append(msg)

    log("FALCON V2 - FLIGHT_ENVELOPE_VALIDATION (gazebo-testing, 2026-08-27)")
    log(f"World: {WORLD}")
    log(f"Target speeds: {TARGET_SPEEDS}")
    log(f"alpha_transition={ALPHA_TRANSITION_DEG:.2f}deg CLmax_manufacturer={CLMAX} "
        f"(NO real stall/post-stall physics implemented - see module docstring)")
    log("")

    results_by_speed, n_candidates = run_part1(log)
    actuator_confirms = run_part2_3(log, results_by_speed)

    # Part 4 trigger: any speed whose confirmed alpha is within 1deg of, or past, alpha_transition.
    trigger = any((results_by_speed[V]["confirm"]["release"]["alpha_deg"] >= ALPHA_TRANSITION_DEG - 1.0)
                  for V in TARGET_SPEEDS)
    part4 = run_part4_high_alpha_check(log) if trigger else None
    if not trigger:
        log("PART 4 not triggered - no speed's confirmed trim alpha approached alpha_transition.\n")

    part6, part6_chosen = run_part6(log, results_by_speed)
    part7 = run_part7(log, results_by_speed, part6_chosen)

    # ---------------- Summary table ----------------
    log("=" * 100)
    log("SUMMARY TABLE (all 8 speeds)")
    log("=" * 100)
    header = (f"{'V_target':>8} {'V_ach':>7} {'alpha':>7} {'thr':>7} {'elev_phys':>9} {'elev_aero':>9} "
              f"{'CL':>8} {'CD':>8} {'L/W':>6} {'Thrust':>7} {'RPM_L':>7} {'RPM_R':>7} {'My':>8} {'class':>14}")
    log(header)
    for V in TARGET_SPEEDS:
        r = results_by_speed[V]
        ah = actuator_confirms[V]
        conf = r["confirm"]
        cl = ah["aero_tail_avg"]["CL"]
        cd = ah["aero_tail_avg"]["CD"]
        lift = ah["pred_from_measured_state"]["Lift"]
        lw = lift / WEIGHT_N
        my = ah["pred_from_measured_state"]["My"]
        thrust = (ah["prop_tail"]["left_thrust_N"] or 0.0) + (ah["prop_tail"]["right_thrust_N"] or 0.0)
        cls_str = r["classification"] + (f"+{r['sub_flag']}" if r["sub_flag"] else "")
        log(f"{V:8.3f} {conf['release']['V']:7.3f} {conf['release']['alpha_deg']:7.3f} "
            f"{conf['throttle']:7.4f} {ah['elev_theta_cmd_deg']:9.3f} {ah['actual_delta_e_deg']:9.3f} "
            f"{cl:8.5f} {cd:8.5f} {lw:6.3f} {thrust:7.3f} {ah['prop_tail']['left_rpm']:7.1f} "
            f"{ah['prop_tail']['right_rpm']:7.1f} {my:8.4f} {cls_str:>14}")
    log("")

    # ---------------- Save results ----------------
    results_by_speed_out = {str(V): strip_series({k: v for k, v in r.items() if k != "confirm"} |
                                                   {"confirm": strip_series(r["confirm"]),
                                                    "all_candidates": [strip_series(c) for c in r["all_candidates"]]})
                             for V, r in results_by_speed.items()}
    actuator_confirms_out = {str(V): strip_series(a) for V, a in actuator_confirms.items()}
    part6_out = {label: strip_series(d) for label, d in part6.items()}
    part7_out = {label: {k: (strip_series(v) if k != "V" else v) for k, v in d.items()} for label, d in part7.items()}

    summary = dict(
        n_trim_candidates=n_candidates,
        target_speeds=TARGET_SPEEDS,
        part6_chosen_speeds=part6_chosen,
        classifications={str(V): dict(classification=results_by_speed[V]["classification"],
                                        sub_flag=results_by_speed[V]["sub_flag"])
                          for V in TARGET_SPEEDS},
    )

    with open(f"{RESULTS_DIR}/flight_envelope_result.json", "w") as f:
        json.dump(dict(summary=summary, results_by_speed=results_by_speed_out,
                        actuator_confirms=actuator_confirms_out,
                        part4_high_alpha_check=part4, part6_free_flight=part6_out, part7_control_authority=part7_out),
                  f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/flight_envelope_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    overall_ok = (not any(results_by_speed[V]["confirm"]["any_nan"] for V in TARGET_SPEEDS)
                  and not any(actuator_confirms[V]["any_nan"] for V in TARGET_SPEEDS)
                  and not any(part6[label]["any_nan"] for label in part6))
    return overall_ok


if __name__ == "__main__":
    ok = main()
    sys.exit(0 if ok else 1)

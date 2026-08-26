#!/usr/bin/env python3
"""
FALCON V2 - POWERED_TRIM_AND_FREE_FLIGHT_VALIDATION, Phase 1 (gazebo-testing,
2026-08-23).

Coarse -> refine search for a real powered straight-and-level trim point near
V=18 m/s, using the already-verified, already-built aerodynamics+propulsion
plugins (plugins/aerodynamics/, plugins/propulsion/) and the already-verified
control mapping (delta_e_aero = -0.5*(theta_left_elevator+theta_right_elevator),
elevator_sign=-1.0, aero_v1_config.yaml). Search variables: symmetric throttle
x symmetric elevator deflection. Ailerons/rudder stay neutral throughout.

No aircraft physics parameter (mass, CG, inertia, aero coefficient, control
derivative, motor/prop constant, control-sign mapping) is read for any
purpose other than loading the existing config/state, and none is modified
anywhere in this script.

-----------------------------------------------------------------------------
METHODOLOGY NOTE - why the "RPM-settling hold" phase holds BOTH linear AND
angular rate (not "linear only", despite the task brief's "hold velocity
only if needed" wording) - empirically validated this pass before writing
this production script:
-----------------------------------------------------------------------------
A "hold body-frame linear velocity only, leave angular rate completely free"
technique was tried FIRST during this pass's setup and produced a spurious,
non-physical result: at throttle=0.50/elevator=-8deg(aero) (the exact
"known non-trim" reference point already established by
POWERED_FREE_FLIGHT_SMOKE_TEST), holding u/w rigidly for as little as 0.3-0.9s
while leaving p/q/r completely free let the airframe rotate to an
implausible -70deg+ nose-up pitch attitude within under a second, and kept
accelerating rather than showing the mild, bounded (~-10deg peak) phugoid-
like oscillation the reference run already established for this exact
condition. Root cause: alpha is computed purely from BODY-FRAME u/w
(atan2(-w,u)), which is INDEPENDENT of the airframe's world-frame attitude -
so rigidly holding u/w while letting the airframe rotate freely artificially
freezes alpha near its held value regardless of how far the airframe has
actually rotated, which removes the real, physical negative-feedback
coupling (rotation -> changing alpha/flight-path -> changing moment) that
normally bounds pitch rate in genuine free flight. This is a test-harness
artifact of that specific technique, not a real aircraft-dynamics finding -
confirmed by re-running the SAME condition holding BOTH linear AND angular
rate (0,0,0 target) for a SHORT settling phase (800 steps / 0.8s, using
kp_ang=400 - still just a controller gain, not a physics parameter) and then
releasing fully: the resulting free-flight trace matches the established
reference almost exactly (release-point V=18.16 m/s/alpha=2.47deg matching
the reference's stated start; pitch reaching -10.8deg at t=3.5s, matching
the reference's "peaked -10.02deg near t=4s" to within ~1deg/0.5s). This
validated technique (both-DOF short hold, then full release) is what this
script actually uses below.
-----------------------------------------------------------------------------

Per-candidate procedure (quasi-steady evaluation, see task brief):
  1. One-time teleport (Model.set_world_pose_cmd) to altitude=100m, level
     attitude - confirmed clean/non-freezing this pass (unlike
     Link.set_linear_velocity()/set_angular_velocity(), which aero_lib.py's
     own header already documents as freezing further integration).
  2. hold_step() both linear (u,v,w) AND angular (p,q,r) rate targets at a
     COMMON baseline (u=18.14534, w=-0.78335, i.e. V=18.162 m/s, alpha=2.47deg
     - the already-established "known non-trim" reference condition, used as
     a consistent starting point across every candidate so differences in
     post-release behavior are attributable to the candidate's throttle/
     elevator choice, not to a different starting alpha) for HOLD_STEPS=800
     (0.8s) - long enough for RPM to settle (propulsion's own spin-up
     characterization + this pass's own smoke check: thrust flatlines by
     ~0.3-0.4s at throttle=0.4-0.55) plus margin, short enough to keep the
     P-only angular-hold's steady-state residual-rate drift small (see note
     above).
  3. Elevator commanded via pin_control_surface_joints() (left=right=theta,
     ailerons/rudder=0) EVERY tick, throughout - the 5 control-surface joints
     are documented TEMPORARY_NUMERICAL_MASS (1g, no actuator, no damping),
     same technique the whole existing test suite already uses. Throttle
     commanded via propulsion_lib.ThrottleCommander, republished every tick
     (topics are not latched).
  4. RELEASE fully (no hold_step call at all, no other controller, nothing)
     for WINDOW_STEPS=2000 (2s) of genuinely free 6-DOF dynamics. Record
     u,v,w,p,q,r,world-Z,world-Z-velocity every step; record propulsion
     diagnostics (RPM/thrust) periodically.

Candidate metrics (evaluated at the release instant, i.e. the last held
sample - representing the quasi-steady condition each candidate was
commanded to - AND across the free-flight window that follows):
  - V, alpha, vertical speed, pitch rate q, longitudinal acceleration,
    Lift, Drag, My (aero-only, via aero_lib.compute_aero fed the measured
    release-instant state), and per-motor RPM/thrust (from the live
    propulsion diagnostics topic) at the release instant.
  - airspeed drift, mean vertical speed, mean pitch rate, altitude drift
    across the WINDOW_STEPS free-flight window that follows.

Candidate ranking (per task brief, in this priority order): 1) minimize
|vertical speed| at release, 2) minimize |airspeed drift| over the window,
3) minimize |pitch-rate drift| (mean |q| over the window), 4) minimize
|pitch moment| (My) at release - while staying near V~18 m/s.
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

CFG = AL.load_config()
MASS = 5.9348  # kg, base_link mass, model/model.sdf (queried live too, cross-checked)
I_DIAG = (0.7284, 0.2507, 0.9523)  # kg*m^2, base_link diagonal inertia, model/model.sdf - controller gain only
KP_LIN = 150.0
KP_ANG = 400.0
HOLD_STEPS = 800
WINDOW_STEPS = 2000
ALTITUDE_M = 100.0
U_HOLD, W_HOLD = 18.14534, -0.78335  # common baseline, matches the established "known non-trim" reference point
WEIGHT_N = 58.86  # CONFIRMED, CLAUDE.md (6.000 kg * 9.81 m/s^2 = 58.86 N)


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


def run_candidate(log, throttle, elevator_aero_deg, window_steps=WINDOW_STEPS):
    """theta = -delta_e_aero for symmetric commands (verified control mapping,
    task brief) - elevator_theta_rad is the PHYSICAL joint angle commanded to
    BOTH left_elevator_joint/right_elevator_joint. window_steps overridable
    for ROUND 3 below (a longer post-release window used to measure the
    SETTLED/tail-window vertical speed, not just the short-window mean - see
    ROUND 3's own comment for why this distinction matters)."""
    elevator_theta_rad = math.radians(-elevator_aero_deg)

    state = {"n": 0, "teleported": False, "series": [], "any_nan": False,
             "release_sample": None, "throttle_cmd": None, "prop_diag": None}

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
            "left_elevator_joint": elevator_theta_rad,
            "right_elevator_joint": elevator_theta_rad})
        n = state["n"]
        if n < HOLD_STEPS:
            AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(U_HOLD, 0, W_HOLD),
                         gm.Vector3d(0, 0, 0), kp_lin=KP_LIN, kp_ang=KP_ANG)
        # else: fully released - no controller call of any kind.

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
            state["n"] += 1
            return
        rot = wpose.rot()
        lv_b = rot.rotate_vector_reverse(lv)
        av_b = rot.rotate_vector_reverse(av)
        vals = [lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z(),
                wpose.pos().z(), lv.z()]
        if any(math.isnan(x) or math.isinf(x) for x in vals):
            state["any_nan"] = True
        _, pitch, _ = quat_rpy(rot)
        sample = dict(n=n, t=n * AL.STEP, u=lv_b.x(), v=lv_b.y(), w=lv_b.z(),
                      p=av_b.x(), q=av_b.y(), r=av_b.z(),
                      alt=wpose.pos().z(), world_vz=lv.z(), pitch=pitch)
        if n == HOLD_STEPS:
            prop = state["prop_diag"].latest() if state["prop_diag"] else None
            sample["prop"] = prop
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

    aero_rel = AL.compute_aero(CFG, rel["u"], rel["v"], rel["w"], rel["p"], rel["q"], rel["r"],
                                deltaA=0.0, deltaE=0.5 * CFG.elevatorSign * (elevator_theta_rad + elevator_theta_rad),
                                deltaR=0.0)
    V_rel = aero_rel["V"]
    alpha_rel_deg = math.degrees(aero_rel["alpha"])

    # Short-window (first 0.5s post-release) longitudinal-acceleration estimate.
    half_s_idx = min(500, len(series) - 1)
    u0, u_half = series[0]["u"], series[half_s_idx]["u"]
    long_accel = (u_half - u0) / (series[half_s_idx]["t"] - series[0]["t"])

    V_end = math.sqrt(win_end["u"] ** 2 + win_end["v"] ** 2 + win_end["w"] ** 2)
    airspeed_drift = V_end - V_rel
    mean_world_vz = sum(s["world_vz"] for s in series) / len(series)
    mean_abs_q_deg_s = sum(abs(math.degrees(s["q"])) for s in series) / len(series)
    alt_drift = win_end["alt"] - rel["alt"]

    # Tail-window (last 30% of the window) metrics - used by ROUND 3 below to
    # measure the SETTLED vertical-speed/altitude-drift-rate, as distinct
    # from the whole-window mean (which, for the short WINDOW_STEPS=2s used
    # by rounds 1-2, is dominated by the initial post-release transient, not
    # the true settled behavior - see ROUND 3's header comment for the
    # empirical finding that motivated this).
    tail_start_idx = int(len(series) * 0.7)
    tail = series[tail_start_idx:]
    tail_mean_world_vz = sum(s["world_vz"] for s in tail) / len(tail)
    tail_alt_drift_rate_mps = ((tail[-1]["alt"] - tail[0]["alt"]) /
                                (tail[-1]["t"] - tail[0]["t"])) if tail[-1]["t"] != tail[0]["t"] else 0.0

    prop = rel.get("prop")
    left_rpm = prop["left"]["rpm"] if prop else None
    right_rpm = prop["right"]["rpm"] if prop else None
    left_thrust = prop["left"]["thrust_N"] if prop else None
    right_thrust = prop["right"]["thrust_N"] if prop else None
    thrust_total = (left_thrust + right_thrust) if (left_thrust is not None and right_thrust is not None) else None

    result = dict(
        throttle=throttle, elevator_aero_deg=elevator_aero_deg,
        elevator_theta_deg=math.degrees(elevator_theta_rad),
        release=dict(V=V_rel, alpha_deg=alpha_rel_deg, world_vz=rel["world_vz"],
                     q_deg_s=math.degrees(rel["q"])),
        aero_rel=dict(Lift_N=(aero_rel["qbar"] * CFG.S * aero_rel["CL"]),
                       Drag_N=(aero_rel["qbar"] * CFG.S * aero_rel["CD"]),
                       My_Nm=aero_rel["My"], CL=aero_rel["CL"], CD=aero_rel["CD"]),
        prop_rel=dict(left_rpm=left_rpm, right_rpm=right_rpm,
                      left_thrust_N=left_thrust, right_thrust_N=right_thrust,
                      thrust_total_N=thrust_total),
        long_accel_mps2=long_accel,
        window=dict(V_end=V_end, airspeed_drift_mps=airspeed_drift,
                    mean_world_vz_mps=mean_world_vz, mean_abs_q_deg_s=mean_abs_q_deg_s,
                    alt_drift_m=alt_drift, duration_s=window_steps * AL.STEP,
                    tail_mean_world_vz_mps=tail_mean_world_vz,
                    tail_alt_drift_rate_mps=tail_alt_drift_rate_mps),
        any_nan=state["any_nan"],
    )

    log(f"Candidate throttle={throttle:.3f} elevator_aero={elevator_aero_deg:+.1f}deg "
        f"(theta={math.degrees(elevator_theta_rad):+.1f}deg): "
        f"V_rel={V_rel:.3f} alpha_rel={alpha_rel_deg:.3f}deg world_vz_rel={rel['world_vz']:+.4f} "
        f"q_rel={math.degrees(rel['q']):+.4f}deg/s "
        f"Lift={result['aero_rel']['Lift_N']:.3f}N Drag={result['aero_rel']['Drag_N']:.3f}N "
        f"My={result['aero_rel']['My_Nm']:+.4f}Nm "
        f"RPM(L/R)={left_rpm:.1f}/{right_rpm:.1f} Thrust(L/R)={left_thrust:.3f}/{right_thrust:.3f}N "
        f"ThrustTotal={thrust_total:.3f}N "
        f"long_accel(0.5s)={long_accel:+.4f}m/s^2 "
        f"window[V_end={V_end:.3f} drift={airspeed_drift:+.4f} "
        f"mean_vz={mean_world_vz:+.4f} mean|q|={mean_abs_q_deg_s:.3f}deg/s alt_drift={alt_drift:+.3f}m "
        f"tail_mean_vz={tail_mean_world_vz:+.4f} tail_alt_rate={tail_alt_drift_rate_mps:+.4f}m/s] "
        f"any_nan={state['any_nan']}")

    return result


def candidate_score(r):
    """Priority order per task brief: 1) |vertical speed| 2) |airspeed drift|
    3) |pitch-rate drift| 4) |pitch moment|. Returns a tuple for lexicographic
    ranking (smaller is better) with generous bucketing on the top-priority
    metric so that a small vertical-speed difference doesn't dominate over a
    much better airspeed/pitch-rate/moment result (bucket width 0.3 m/s,
    matching the ~+/-0.3 m/s scale of the candidate spread observed this
    pass) - documented explicitly here rather than a silent design choice.

    IMPORTANT: uses window['mean_world_vz_mps'] (the mean vertical speed over
    the free-flight window that follows release), NOT release['world_vz'].
    release['world_vz'] is measured on the very FIRST free-dynamics tick
    (1 ms after the hold_step() controller stops being called) - found this
    pass to be dominated by whatever body-w the hold controller had just
    converged to (it is, by construction, still extremely close to the held
    target w=-0.78335 m/s at that single instant, since only one un-held
    integration step has elapsed) rather than reflecting the candidate's own
    aerodynamic+propulsive vertical-force balance. The window-mean value,
    averaged over the full WINDOW_STEPS=2s of genuinely free dynamics that
    follows, is the metric that actually varies meaningfully across
    candidates and is used for ranking; release['world_vz'] is still recorded
    in every result for transparency/traceability, just not used to rank."""
    vz_bucket = round(abs(r["window"]["mean_world_vz_mps"]) / 0.3)
    return (vz_bucket, abs(r["window"]["airspeed_drift_mps"]),
            r["window"]["mean_abs_q_deg_s"], abs(r["aero_rel"]["My_Nm"]))


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    log("FALCON V2 - POWERED_TRIM_AND_FREE_FLIGHT_VALIDATION - Phase 1 trim search (gazebo-testing, 2026-08-23)")
    log(f"World: {WORLD}")
    log(f"Common hold baseline: u={U_HOLD} w={W_HOLD} (V=18.162 m/s, alpha=2.472deg), "
        f"HOLD_STEPS={HOLD_STEPS} ({HOLD_STEPS*AL.STEP}s), WINDOW_STEPS={WINDOW_STEPS} ({WINDOW_STEPS*AL.STEP}s)")
    log(f"elevator_sign (loaded fresh from aero_v1_config.yaml) = {CFG.elevatorSign}")
    log("")

    # ---------------- Coarse grid ----------------
    throttles = [0.40, 0.475, 0.55]
    elevators = [-4.0, -6.0, -8.0, -10.0]

    log("=" * 78)
    log("COARSE GRID")
    log("=" * 78)
    coarse_results = []
    for th in throttles:
        for el in elevators:
            r = run_candidate(log, th, el)
            coarse_results.append(r)
    log("")

    def show_ranked(label, results, n=8):
        ranked = sorted(results, key=candidate_score)
        log(f"{label} (best first, priority: |mean window vz| > |airspeed drift| > |mean q| > |My|):")
        for r in ranked[:n]:
            log(f"  throttle={r['throttle']:.3f} elevator={r['elevator_aero_deg']:+.1f}deg -> "
                f"mean_vz={r['window']['mean_world_vz_mps']:+.4f} "
                f"drift={r['window']['airspeed_drift_mps']:+.4f} "
                f"mean|q|={r['window']['mean_abs_q_deg_s']:.3f} My={r['aero_rel']['My_Nm']:+.4f} "
                f"ThrustTotal={r['prop_rel']['thrust_total_N']:.3f} Drag={r['aero_rel']['Drag_N']:.3f} "
                f"Lift={r['aero_rel']['Lift_N']:.3f}")
        return ranked

    ranked = show_ranked("Coarse grid ranked", coarse_results, n=5)
    best_coarse = ranked[0]
    log(f"\nBest coarse candidate: throttle={best_coarse['throttle']:.3f} "
        f"elevator={best_coarse['elevator_aero_deg']:+.1f}deg\n")

    # ---------------- Refinement round 1 (+/-0.025 throttle, +/-1deg elevator) ----------------
    log("=" * 78)
    log("REFINEMENT ROUND 1 (around best coarse candidate)")
    log("=" * 78)
    th0, el0 = best_coarse["throttle"], best_coarse["elevator_aero_deg"]
    refine1_candidates = set()
    for dth in (-0.025, 0.0, 0.025):
        for delv in (-1.0, 0.0, 1.0):
            th = round(min(0.55, max(0.40, th0 + dth)), 4)
            el = round(min(-4.0, max(-10.0, el0 + delv)), 4)
            refine1_candidates.add((th, el))
    coarse_keys = {(r["throttle"], r["elevator_aero_deg"]) for r in coarse_results}
    refine1_candidates = sorted(c for c in refine1_candidates if c not in coarse_keys)

    refine1_results = [run_candidate(log, th, el) for th, el in refine1_candidates]
    log("")

    pool = coarse_results + refine1_results
    ranked = show_ranked("Round-1 combined ranking", pool, n=5)
    best1 = ranked[0]
    log(f"\nBest after round 1: throttle={best1['throttle']:.3f} "
        f"elevator={best1['elevator_aero_deg']:+.1f}deg\n")

    # ---------------- Refinement round 2 (finer, +/-0.0125 throttle, +/-0.5deg elevator) ----------------
    log("=" * 78)
    log("REFINEMENT ROUND 2 (finer grid around round-1 best)")
    log("=" * 78)
    th1, el1 = best1["throttle"], best1["elevator_aero_deg"]
    refine2_candidates = set()
    for dth in (-0.0125, 0.0, 0.0125):
        for delv in (-0.5, 0.0, 0.5):
            th = round(min(0.55, max(0.40, th1 + dth)), 4)
            el = round(min(-4.0, max(-10.0, el1 + delv)), 4)
            refine2_candidates.add((th, el))
    seen_keys = {(r["throttle"], r["elevator_aero_deg"]) for r in pool}
    refine2_candidates = sorted(c for c in refine2_candidates if c not in seen_keys)

    refine2_results = [run_candidate(log, th, el) for th, el in refine2_candidates]
    log("")

    all_results = pool + refine2_results
    ranked_all = show_ranked("FULL RANKING (coarse + round 1 + round 2)", all_results, n=10)
    best2 = ranked_all[0]

    # ---------------- Round 3: settled/tail-window check on the round-1/2 shortlist ----------------
    # MOTIVATION (found this pass, empirically, when the round-2 winner was
    # carried into a full Phase-2-style free-flight run): rounds 1-2 rank
    # candidates on the WHOLE-window (WINDOW_STEPS=2s) mean vertical speed.
    # That 2s window sits almost entirely inside the initial post-release
    # phugoid-excitation transient (the same transient documented in
    # test_powered_free_flight_validation.py's own results - pitch/V/vz all
    # swing through a full excursion within the first ~4-8s before damping
    # toward a settled condition), so a candidate can score well on the
    # SHORT window's mean (transient oscillation happens to average near
    # zero over exactly 2s) while its TRUE settled-state vertical speed
    # (after the transient damps out) is meaningfully different. Round 3
    # re-checks a small, deliberately targeted shortlist (round-1/2's best
    # candidate plus a few nearby points chosen from an analytical
    # propulsion-pitch-moment cross-check - see below) using a longer
    # (12s) window and ranks on the TAIL (last 30%, i.e. last ~3.6s) mean
    # vertical speed / altitude-drift-rate instead - a much better proxy for
    # genuine settled-state trim than the short-window mean. This is still
    # a search over CONTROL INPUTS only (throttle, elevator) - no aircraft
    # physics parameter is touched.
    #
    # Analytical cross-check used to pick the shortlist: propulsion's own
    # thrust is applied at the real hub position (geometry/PROPULSION.md,
    # propulsion_v1_config.yaml geometry.left_hub_m/right_hub_m =
    # [0.2951,+/-0.300,0.1271]) relative to base_link's real CG
    # (0.168309,0,0.100000, MASS_PROPERTIES.md/CLAUDE.md) - a forward thrust
    # force applied above the CG (hub Z=0.1271 > CG Z=0.100, dz=0.0271m)
    # produces r x F = a NOSE-DOWN pitching moment of dz*T per motor, i.e.
    # 2*dz*T_per_motor = 0.0542*T_per_motor (Nm) for both motors combined -
    # NOT modeled by aero_lib.compute_aero() (which is aero-only), so it is
    # never included in this script's own "My" column above, but it is
    # real and present in every live-Gazebo run via propulsion's own
    # AddWorldWrench() call. Adding this estimate to each candidate's
    # aero-only My gives a rough NET static-moment estimate that better
    # predicts genuine settled pitch/vz behavior than aero-only My alone -
    # used only to pick which nearby points are worth spending a 12s check
    # on, never as a substitute for the actual live measurement.
    log("")
    log("=" * 78)
    log("REFINEMENT ROUND 3 (settled/tail-window check, longer 12s release window)")
    log("=" * 78)
    DZ_HUB_CG = 0.0271  # m, hub Z (0.1271) - CG Z (0.100000), both CONFIRMED (cited above)
    shortlist = set()
    shortlist.add((best2["throttle"], best2["elevator_aero_deg"]))
    for r in all_results:
        # net propulsion My = dz * (T_left + T_right) = dz * thrust_total_N
        # (each motor contributes dz*T_per_motor; summing both motors and
        # substituting T_per_motor=thrust_total_N/2 gives dz*thrust_total_N).
        net_My_est = r["aero_rel"]["My_Nm"] + DZ_HUB_CG * (r["prop_rel"]["thrust_total_N"] or 0.0)
        r["_net_My_est"] = net_My_est
    near_zero_net = sorted(all_results, key=lambda r: abs(r["_net_My_est"]))[:4]
    for r in near_zero_net:
        shortlist.add((r["throttle"], r["elevator_aero_deg"]))
    shortlist = sorted(shortlist)
    log(f"Shortlist for round 3 (round-1/2 winner + up to 4 candidates with smallest "
        f"|aero My + estimated propulsion pitch moment|): {shortlist}")

    ROUND3_WINDOW_STEPS = 12000  # 12 s - still far cheaper than a full 15-30s Phase-2 run, long enough to reach a settled tail
    round3_results = [run_candidate(log, th, el, window_steps=ROUND3_WINDOW_STEPS) for th, el in shortlist]
    log("")

    ranked3 = sorted(round3_results, key=lambda r: (
        round(abs(r["window"]["tail_mean_world_vz_mps"]) / 0.1),
        abs(r["window"]["tail_alt_drift_rate_mps"]),
        abs(r["window"]["airspeed_drift_mps"])))
    log("Round 3 ranked (best first, priority: |tail mean vz| > |tail alt drift rate| > |airspeed drift|):")
    for r in ranked3:
        log(f"  throttle={r['throttle']:.4f} elevator={r['elevator_aero_deg']:+.2f}deg -> "
            f"tail_mean_vz={r['window']['tail_mean_world_vz_mps']:+.4f} "
            f"tail_alt_rate={r['window']['tail_alt_drift_rate_mps']:+.4f} "
            f"drift={r['window']['airspeed_drift_mps']:+.4f} "
            f"net_My_est={r.get('_net_My_est', float('nan')):+.4f}")

    best3 = ranked3[0]
    log(f"\nBest after round 3: throttle={best3['throttle']:.4f} "
        f"elevator_aero={best3['elevator_aero_deg']:+.2f}deg\n")

    # ---------------- Round 4: directional extrapolation along throttle at the round-3 elevator ----------------
    # Round 3's own data at elevator=-5.5deg shows tail_mean_vz increasing
    # roughly linearly with throttle (0.5125->+0.3875, 0.525->+0.6155,
    # 0.5375->+0.8495 m/s) - i.e. the true zero-crossing (settled level
    # flight, tail_mean_vz=0) lies BELOW the throttle range rounds 1-2's
    # grid ever explored around their own (different) local optimum. This
    # round fits a simple linear trend through round 3's same-elevator
    # points and tests 3 candidates bracketing the extrapolated zero
    # crossing - still a control-input-only search, same technique as
    # rounds 1-3, just following the trend the data itself revealed rather
    # than re-centering on a fixed step size.
    log("=" * 78)
    log("REFINEMENT ROUND 4 (directional extrapolation along throttle, same elevator)")
    log("=" * 78)
    same_elev = [r for r in round3_results if r["elevator_aero_deg"] == best3["elevator_aero_deg"]]
    same_elev.sort(key=lambda r: r["throttle"])
    if len(same_elev) >= 2:
        th_a, vz_a = same_elev[0]["throttle"], same_elev[0]["window"]["tail_mean_world_vz_mps"]
        th_b, vz_b = same_elev[-1]["throttle"], same_elev[-1]["window"]["tail_mean_world_vz_mps"]
        slope = (vz_b - vz_a) / (th_b - th_a) if th_b != th_a else 0.0
        th_zero = th_a - vz_a / slope if slope != 0.0 else th_a
        th_zero = min(0.55, max(0.40, th_zero))
    else:
        th_zero = best3["throttle"]
        slope = 0.0
    log(f"Linear fit through round-3 same-elevator ({best3['elevator_aero_deg']:+.1f}deg) points: "
        f"slope={slope:+.3f} (m/s per throttle unit), extrapolated zero-crossing throttle={th_zero:.4f}")

    round3_keys = {(r["throttle"], r["elevator_aero_deg"]) for r in round3_results}
    round4_candidates = sorted({round(min(0.55, max(0.40, th_zero + d)), 4)
                                 for d in (-0.0125, 0.0, 0.0125)})
    round4_candidates = [(th, best3["elevator_aero_deg"]) for th in round4_candidates
                          if (th, best3["elevator_aero_deg"]) not in round3_keys]
    log(f"Round 4 candidates: {round4_candidates}")
    round4_results = [run_candidate(log, th, el, window_steps=ROUND3_WINDOW_STEPS) for th, el in round4_candidates]
    log("")

    final_pool = round3_results + round4_results
    ranked_final = sorted(final_pool, key=lambda r: (
        round(abs(r["window"]["tail_mean_world_vz_mps"]) / 0.1),
        abs(r["window"]["tail_alt_drift_rate_mps"]),
        abs(r["window"]["airspeed_drift_mps"])))
    log("Round 3+4 combined ranking (best first, same priority as round 3):")
    for r in ranked_final:
        log(f"  throttle={r['throttle']:.4f} elevator={r['elevator_aero_deg']:+.2f}deg -> "
            f"tail_mean_vz={r['window']['tail_mean_world_vz_mps']:+.4f} "
            f"tail_alt_rate={r['window']['tail_alt_drift_rate_mps']:+.4f} "
            f"drift={r['window']['airspeed_drift_mps']:+.4f}")

    best = ranked_final[0]
    log(f"\nSELECTED TRIM CANDIDATE (final, after round 4): throttle={best['throttle']:.4f} "
        f"elevator_aero={best['elevator_aero_deg']:+.2f}deg "
        f"(elevator_theta={best['elevator_theta_deg']:+.2f}deg physical)")

    all_results_full = all_results + round3_results + round4_results
    any_nan_overall = any(r["any_nan"] for r in all_results_full)
    log(f"any_nan across all candidates: {any_nan_overall}")

    with open(f"{RESULTS_DIR}/powered_trim_search_result.json", "w") as f:
        json.dump({
            "coarse_grid": coarse_results,
            "refinement_round_1": refine1_results,
            "refinement_round_2": refine2_results,
            "refinement_round_3_settled_tail_check": round3_results,
            "refinement_round_4_directional_extrapolation": round4_results,
            "selected": best,
            "any_nan_overall": any_nan_overall,
        }, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/powered_trim_search_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return best, any_nan_overall


if __name__ == "__main__":
    best, any_nan = run()
    sys.exit(1 if any_nan else 0)

#!/usr/bin/env python3
"""
FALCON V2 - AERODYNAMICS_V1 live-Gazebo test pass (gazebo-testing, 2026-08-22).

Covers the "kinematic" (steady-condition, diagnostics-driven) subset of the
mandatory aero test list from the task brief:

  - ZERO_AIRSPEED_AERO_TEST
  - AOA_SIGN_TEST
  - SIDESLIP_SIGN_TEST
  - LIFT_SIGN_TEST / DRAG_SIGN_TEST  (real force measured via velocity slope)
  - DRAG_POLAR_TEST
  - HIGH_ALPHA_LIMITER_TEST

(RATE_NORMALIZATION_TEST is covered in test_aero_stability_derivatives.py
instead, alongside Cmq/Clp/Cnr_DAMPING_SIGN_TEST, since it needs the same
per-axis-release direct-measurement technique - see that file and this run's
test report for why: the diagnostics topic was found, empirically this pass,
to intermittently report Cl/Cm/Cn values inconsistent with directly-sampled
ECM state specifically during phases with an actively-controlled nonzero
body rate; not corroborated by direct state sampling, which stayed smooth -
suspected numerical solver noise from the extreme, ~5935:1 mass ratio
between base_link and its 7 lightweight child links under continuous
applied torque. Flagged as an OBSERVATION for `aerodynamics`/`validation`,
not asserted as a plugin defect, and not relied upon for PASS/FAIL below.)

Method: one continuous run against tests/gazebo/worlds/falcon_v2_zero_g_world.sdf
(gravity=0, no ground - isolates pure aerodynamic response), which already
includes model/model.sdf with the FalconV2Aerodynamics plugin attached
(additive <plugin> block, unmodified by this script). The plugin .so is made
discoverable via GZ_SIM_SYSTEM_PLUGIN_PATH (aero_lib.setup_env()), exactly as
documented in plugins/aerodynamics/README.md, applied here to the Python
TestFixture API.

Each scenario is run as a "hold" phase using aero_lib.hold_step() - a
proportional force/torque controller applied via the SAME
Link.add_world_force()/add_world_wrench() primitives the aero plugin itself
uses - to reach and hold an exact, known relative-wind/rate condition, while
the plugin's diagnostics topic (/model/falcon_v2/aerodynamics/diagnostics)
is read live and cross-checked against an independent pure-python
re-implementation of the exact AeroModel.hh formulas (aero_lib.compute_aero).
For LIFT_SIGN_TEST/DRAG_SIGN_TEST, BOTH controllers are then RELEASED for a
short window so the REAL applied force can be measured directly from the
resulting velocity change - not merely inferred from the diagnostics topic.
(An earlier attempt at this suite used gz.sim8 Link.set_linear_velocity() /
set_angular_velocity() Cmd-based override instead; empirically found to
freeze the link's reported state once the override calls stop, making it
unusable for a "release and measure real response" test - see aero_lib.py's
hold_step() docstring and this run's test report for the full evidence.)

No aircraft physics parameter is modified by this script.
"""
import json
import math
import sys

import aero_lib as AL

AL.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"

CFG = AL.load_config()

MASS = 5.9348  # kg, base_link mass (model/model.sdf) - cross-checked at runtime below
# Nominal scalar torque-controller gain scale (kg*m^2-ish units, not a
# physical quantity - just needs enough control authority; empirically this
# specific (I_EFF, KP_LIN, KP_ANG) combination converges cleanly for these
# steady-condition holds - see this run's test report for the tuning
# history). Every PASS/FAIL below is based on live-measured/live-published
# values; the real inertia is independently re-queried at runtime
# (aero_lib.read_base_link_inertia) for the record, never assumed.
I_EFF = 0.5
KP_LIN = 150.0
KP_ANG = 150.0


def deg(x):
    return math.degrees(x)


def rad(x):
    return math.radians(x)


# -----------------------------------------------------------------------
# Phase plan. Each phase holds base_link at a known body-frame condition
# (lin_vel, ang_vel) for `hold` steps via aero_lib.hold_step(), optionally
# followed by `release` steps during which BOTH controllers are turned off
# so the real physics response can be measured directly.
# -----------------------------------------------------------------------
U = 15.0  # m/s, representative cruise-ish speed for all steady-condition phases

PHASES = []


def add_phase(name, lin_vel, ang_vel, hold=400, release=0, release_axes="both"):
    PHASES.append(dict(name=name, lin_vel=lin_vel, ang_vel=ang_vel,
                        hold=hold, release=release, release_axes=release_axes))


# 0: ZERO_AIRSPEED_AERO_TEST - NO control at all, spawned at rest (u=v=w=0,
# p=q=r=0 by default), gravity=0. If the aero model has any V=0 divide-by-zero
# bug, this would show up as NaN/explosive motion.
add_phase("ZERO_AIRSPEED", lin_vel=None, ang_vel=None, hold=200, release=0)

# 1: AOA_SIGN_TEST, positive alpha (nose-up relative to wind)
alpha_p = rad(8.0)
add_phase("AOA_POS_8DEG", gm.Vector3d(U, 0.0, -U * math.tan(alpha_p)),
          gm.Vector3d(0, 0, 0))

# 2: AOA_SIGN_TEST, negative alpha
alpha_n = rad(-8.0)
add_phase("AOA_NEG_8DEG", gm.Vector3d(U, 0.0, -U * math.tan(alpha_n)),
          gm.Vector3d(0, 0, 0))

# 3: SIDESLIP_SIGN_TEST, positive beta
beta_p = rad(8.0)
add_phase("BETA_POS_8DEG", gm.Vector3d(U, U * math.tan(beta_p), 0.0),
          gm.Vector3d(0, 0, 0))

# 4: SIDESLIP_SIGN_TEST, negative beta
beta_n = rad(-8.0)
add_phase("BETA_NEG_8DEG", gm.Vector3d(U, U * math.tan(beta_n), 0.0),
          gm.Vector3d(0, 0, 0))

# 5: LIFT_SIGN_TEST / DRAG_SIGN_TEST - alpha=beta=0, hold then release LINEAR
# ONLY (angular stays actively held at 0 throughout, including the release
# window, so rotation cannot contaminate the translational-force measurement -
# validated empirically this pass: releasing BOTH controllers together let
# residual rotation dynamics contaminate the reading; releasing linear only
# gives a clean, real-physics Fx/Fz measurement matching the independent
# replica to <4%, see this run's test report).
add_phase("LIFT_DRAG", gm.Vector3d(U, 0.0, 0.0), gm.Vector3d(0, 0, 0),
          hold=400, release=15, release_axes="linear")

# 6-8: DRAG_POLAR_TEST - alpha sweep in the validated linear region
for a_deg in (0.0, 3.0, 6.0):
    a = rad(a_deg)
    add_phase(f"POLAR_ALPHA_{a_deg:g}DEG", gm.Vector3d(U, 0.0, -U * math.tan(a)),
              gm.Vector3d(0, 0, 0))

# 9-13: HIGH_ALPHA_LIMITER_TEST - well past the 9.25 deg transition, both signs
for a_deg in (15.0, 30.0, 60.0, -15.0, -30.0):
    a = rad(a_deg)
    add_phase(f"LIMITER_ALPHA_{a_deg:g}DEG", gm.Vector3d(U, 0.0, -U * math.tan(a)),
              gm.Vector3d(0, 0, 0))


# -----------------------------------------------------------------------
# Runner
# -----------------------------------------------------------------------
def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    diag = AL.DiagSubscriber()

    bounds = []
    t = 0
    for ph in PHASES:
        start = t
        hold_end = start + ph["hold"]
        rel_end = hold_end + ph["release"]
        bounds.append((start, hold_end, rel_end))
        t = rel_end
    total_steps = t

    state = {"n": 0, "phase_series": [[] for _ in PHASES],
             "phase_diag_at_hold_end": [None] * len(PHASES),
             "any_nan": False, "inertia": None}

    def get_model(ecm):
        world = sim.World(sim.world_entity(ecm))
        model_e = world.model_by_name(ecm, "falcon_v2")
        return sim.Model(model_e)

    def current_phase_idx(n):
        for i, (s, h, r) in enumerate(bounds):
            if s <= n < r:
                return i, n < h  # (index, in_hold)
        return len(PHASES) - 1, False

    def on_pre(info, ecm):
        n = state["n"]
        idx, in_hold = current_phase_idx(n)
        ph = PHASES[idx]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)

        if ph["lin_vel"] is None and ph["ang_vel"] is None:
            return  # ZERO_AIRSPEED phase: no control at all

        if in_hold:
            lin_target, ang_target = ph["lin_vel"], ph["ang_vel"]
        else:
            ra = ph["release_axes"]
            lin_target = None if ra in ("linear", "both") else ph["lin_vel"]
            ang_target = None if ra in ("angular", "both") else ph["ang_vel"]
        AL.hold_step(base, ecm, MASS, I_EFF, lin_target, ang_target,
                     kp_lin=KP_LIN, kp_ang=KP_ANG)
        AL.pin_child_joints(model, ecm, sim)

    def on_post(info, ecm):
        n = state["n"]
        idx, in_hold = current_phase_idx(n)
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)

        if state["inertia"] is None and n > 5:
            state["inertia"] = AL.read_base_link_inertia(model, ecm, sim)

        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        wpose = base.world_pose(ecm)
        if lv is None or av is None or wpose is None:
            state["any_nan"] = True
            state["n"] += 1
            return
        rot = wpose.rot()
        lv_b = rot.rotate_vector_reverse(lv)
        av_b = rot.rotate_vector_reverse(av)
        vals = [lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z()]
        if any(math.isnan(v) or math.isinf(v) for v in vals):
            state["any_nan"] = True
        state["phase_series"][idx].append(dict(
            n=n, t=n * AL.STEP, in_hold=in_hold,
            u=lv_b.x(), v=lv_b.y(), w=lv_b.z(),
            p=av_b.x(), q=av_b.y(), r=av_b.z()))

        # Keep overwriting with the most recently RECEIVED diagnostics message
        # throughout the hold window (not just once at n==h-1) - the topic
        # publishes at a fixed rate (20 Hz = every 50 steps by default) which
        # can otherwise lag the exact hold-boundary step by up to ~49 steps;
        # phases are held long enough (see HOLD_STEPS) that the state is
        # already steady well before the end, so this converges to a stable
        # reading rather than a stale one.
        if in_hold:
            latest = diag.latest()
            if latest is not None:
                state["phase_diag_at_hold_end"][idx] = latest

        state["n"] += 1

    fixture = sim.TestFixture(AL.WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    log(f"Total steps run: {state['n']} (expected {total_steps})")
    log(f"Any NaN/Inf observed anywhere: {state['any_nan']}")
    log(f"Diagnostics messages received over whole run: {diag.count()}")
    log(f"base_link inertia (queried at runtime): {state['inertia']}")
    log("")

    results = {}
    overall = not state["any_nan"]

    # ================= ZERO_AIRSPEED_AERO_TEST =================
    idx = 0
    series = state["phase_series"][idx]
    log("=== ZERO_AIRSPEED_AERO_TEST ===")
    max_speed = max(math.hypot(r["u"], r["v"], r["w"]) for r in series)
    max_ang = max(math.hypot(r["p"], r["q"], r["r"]) for r in series)
    d = state["phase_diag_at_hold_end"][idx]
    diag_ok = d is not None and abs(d["V"]) < 1e-6 and abs(d["qbar"]) < 1e-9
    no_nan_here = not any(math.isnan(v) for r in series for v in r.values() if isinstance(v, float))
    motion_ok = max_speed < 1e-6 and max_ang < 1e-6
    ok = diag_ok and motion_ok and no_nan_here
    log(f"Spawned at rest, no control applied at all, {len(series)} samples, gravity=0.")
    log(f"Max observed |lin vel| = {max_speed:.3e} m/s, max |ang vel| = {max_ang:.3e} rad/s (both expect ~0) -> {'PASS' if motion_ok else 'FAIL'}")
    log(f"Diagnostics at end of window: {d}")
    log(f"  V=0, qbar=0 as reported -> {'PASS' if diag_ok else 'FAIL'}")
    log(f"No NaN/Inf -> {'PASS' if no_nan_here else 'FAIL'}")
    log(f"ZERO_AIRSPEED_AERO_TEST overall: {'PASS' if ok else 'FAIL'}")
    results["ZERO_AIRSPEED_AERO_TEST"] = ok
    overall = overall and ok
    log("")

    # ================= AOA_SIGN_TEST =================
    log("=== AOA_SIGN_TEST ===")
    for idx, label, expect_sign in [(1, "AOA_POS_8DEG", +1), (2, "AOA_NEG_8DEG", -1)]:
        d = state["phase_diag_at_hold_end"][idx]
        series = state["phase_series"][idx]
        last = series[-1]
        cross = AL.compute_aero(CFG, last["u"], last["v"], last["w"], 0, 0, 0)
        sign_ok = d is not None and (d["alpha"] * expect_sign > 0)
        match_ok = d is not None and abs(d["alpha"] - cross["alpha"]) < 5e-3
        ok = sign_ok and match_ok
        log(f"{label}: imposed body vel (u,v,w)=({last['u']:.4f},{last['v']:.4f},{last['w']:.4f}) "
            f"(intended alpha={'+' if expect_sign>0 else '-'}8deg, nose {'up' if expect_sign>0 else 'down'} relative to wind)")
        log(f"  diagnostics alpha = {deg(d['alpha']):.4f} deg (expected sign: {'+' if expect_sign>0 else '-'}) -> {'PASS' if sign_ok else 'FAIL'}")
        log(f"  cross-check vs independent python AeroModel.hh replica: alpha_replica={deg(cross['alpha']):.4f} deg -> {'PASS' if match_ok else 'FAIL'}")
        results[f"AOA_SIGN_TEST_{label}"] = ok
        overall = overall and ok
    log("")

    # ================= SIDESLIP_SIGN_TEST =================
    log("=== SIDESLIP_SIGN_TEST ===")
    for idx, label, expect_sign in [(3, "BETA_POS_8DEG", +1), (4, "BETA_NEG_8DEG", -1)]:
        d = state["phase_diag_at_hold_end"][idx]
        series = state["phase_series"][idx]
        last = series[-1]
        cross = AL.compute_aero(CFG, last["u"], last["v"], last["w"], 0, 0, 0)
        sign_ok = d is not None and (d["beta"] * expect_sign > 0)
        match_ok = d is not None and abs(d["beta"] - cross["beta"]) < 5e-3
        ok = sign_ok and match_ok
        log(f"{label}: imposed body vel (u,v,w)=({last['u']:.4f},{last['v']:.4f},{last['w']:.4f})")
        log(f"  diagnostics beta = {deg(d['beta']):.4f} deg (expected sign: {'+' if expect_sign>0 else '-'}) -> {'PASS' if sign_ok else 'FAIL'}")
        log(f"  cross-check vs independent python replica: beta_replica={deg(cross['beta']):.4f} deg -> {'PASS' if match_ok else 'FAIL'}")
        results[f"SIDESLIP_SIGN_TEST_{label}"] = ok
        overall = overall and ok
    log("Adopted convention (per AERODYNAMICS.md sec 19.6): positive beta = relative "
        "wind from the aircraft's LEFT side. Confirmed live: v>0 (wind-from-left "
        "component) -> diagnostics beta>0.")
    log("")

    # ================= LIFT_SIGN_TEST / DRAG_SIGN_TEST =================
    log("=== LIFT_SIGN_TEST / DRAG_SIGN_TEST ===")
    idx = 5
    series = state["phase_series"][idx]
    hold_last = [r for r in series if r["in_hold"]][-1]
    free = [r for r in series if not r["in_hold"]]
    d = state["phase_diag_at_hold_end"][idx]
    cross = AL.compute_aero(CFG, hold_last["u"], hold_last["v"], hold_last["w"], 0, 0, 0)
    log(f"Hold condition (converged, real force/torque P-controller): u={hold_last['u']:.4f} "
        f"v={hold_last['v']:.4f} w={hold_last['w']:.4f} p={hold_last['p']:.5f} q={hold_last['q']:.5f} r={hold_last['r']:.5f}")
    log(f"Diagnostics at hold end: {d}")
    log(f"Independent replica: CL={cross['CL']:.6f} CD={cross['CD']:.6f} qbar={cross['qbar']:.4f} "
        f"expected Fz(lift,+up)={cross['Fz']:.4f} N, Fx(drag axis)={cross['Fx']:.4f} N")

    import numpy as np
    ts = np.array([r["t"] for r in free])
    us = np.array([r["u"] for r in free])
    ws = np.array([r["w"] for r in free])
    qs = np.array([r["q"] for r in free])
    ts0 = ts - ts[0]
    ax_meas = float(np.polyfit(ts0, us, 1)[0])
    az_meas = float(np.polyfit(ts0, ws, 1)[0])
    aq_meas = float(np.polyfit(ts0, qs, 1)[0])
    mass = state["inertia"]["mass"]
    Fx_meas = mass * ax_meas
    Fz_meas = mass * az_meas
    log(f"Measured (real physics, LINEAR controller released / ANGULAR still held at 0, {len(free)} free steps, mass={mass:.4f} kg):")
    log(f"  u(t), w(t) samples: {[(round(r['t'],4), round(r['u'],5), round(r['w'],5)) for r in free]}")
    log(f"  ax(slope) = {ax_meas:.5f} m/s^2 -> Fx_measured = {Fx_meas:.5f} N (expect <0, drag opposing +X forward motion)")
    log(f"  az(slope) = {az_meas:.5f} m/s^2 -> Fz_measured = {Fz_meas:.5f} N (expect >0, lift acting +Z up)")
    log(f"  q(t) drift over same window (rotation-contamination check): slope={aq_meas:.4f} rad/s^2, "
        f"q range [{min(r['q'] for r in free):.5f},{max(r['q'] for r in free):.5f}] rad/s (should stay small over this short window)")

    lift_ok = bool(Fz_meas > 0)
    drag_ok = bool(Fx_meas < 0)
    lift_mag_ok = bool(abs(Fz_meas - cross["Fz"]) < 0.25 * max(abs(cross["Fz"]), 0.01))
    drag_mag_ok = bool(abs(Fx_meas - cross["Fx"]) < 0.25 * max(abs(cross["Fx"]), 0.01))
    log(f"LIFT_SIGN_TEST (Fz>0): {'PASS' if lift_ok else 'FAIL'}; magnitude vs replica within 25%: {'PASS' if lift_mag_ok else 'FAIL'}")
    log(f"DRAG_SIGN_TEST (Fx<0, opposes airflow): {'PASS' if drag_ok else 'FAIL'}; magnitude vs replica within 25%: {'PASS' if drag_mag_ok else 'FAIL'}")
    results["LIFT_SIGN_TEST"] = lift_ok and lift_mag_ok
    results["DRAG_SIGN_TEST"] = drag_ok and drag_mag_ok
    overall = overall and results["LIFT_SIGN_TEST"] and results["DRAG_SIGN_TEST"]
    log("")

    # (RATE_NORMALIZATION_TEST: see test_aero_stability_derivatives.py)

    # ================= DRAG_POLAR_TEST =================
    log("=== DRAG_POLAR_TEST ===")
    polar_ok_all = True
    for i, a_deg in enumerate((0.0, 3.0, 6.0)):
        idx = 6 + i
        d = state["phase_diag_at_hold_end"][idx]
        expected_CD = CFG.CD0 + CFG.dragK * d["CL"] ** 2
        ok = abs(d["CD"] - expected_CD) < 1e-6
        log(f"  alpha={a_deg}deg: CL={d['CL']:.6f} CD={d['CD']:.6f} expected(CD0+k*CL^2)={expected_CD:.6f} -> {'PASS' if ok else 'FAIL'}")
        polar_ok_all = polar_ok_all and ok
    log(f"CD0={CFG.CD0} k={CFG.dragK} (aero_v1_config.yaml, V1_CALIBRATED)")
    log(f"DRAG_POLAR_TEST overall: {'PASS' if polar_ok_all else 'FAIL'}")
    results["DRAG_POLAR_TEST"] = polar_ok_all
    overall = overall and polar_ok_all
    log("")

    # ================= HIGH_ALPHA_LIMITER_TEST =================
    log("=== HIGH_ALPHA_LIMITER_TEST ===")
    limiter_ok_all = True
    for i, a_deg in enumerate((15.0, 30.0, 60.0, -15.0, -30.0)):
        idx = 9 + i
        d = state["phase_diag_at_hold_end"][idx]
        series = state["phase_series"][idx]
        last = series[-1]
        # Cross-check using the LIVE-ACHIEVED state (not just the commanded
        # alpha target) via the FULL compute_aero replica (not the bare
        # saturated_CL() static-only helper) - post-fix, out.CL =
        # SaturatedCL(alpha) + CLq*qHat, so a correct cross-check must
        # include whatever small residual q the hold-controller left behind
        # (the alpha-hold controls u,v,w directly, not q, so a small non-zero
        # q residual is expected and legitimately contributes to CL now).
        cross = AL.compute_aero(CFG, last["u"], last["v"], last["w"],
                                 last["p"], last["q"], last["r"])
        expected_CL = cross["CL"]
        expected_CL_static_only = AL.saturated_CL(CFG, d["alpha"])
        match_ok = abs(d["CL"] - expected_CL) < 3e-3
        # |CL| may now legitimately exceed CLmax by a small margin (the CLq*qHat
        # rate term is deliberately added UNSATURATED on top of the saturated
        # static term - AeroModel.hh comment) - bound generously to catch a
        # genuine unbounded-growth bug, not this small, expected rate addition.
        bounded_ok = abs(d["CL"]) <= CFG.CLmax + 0.1
        finite_ok = not (math.isnan(d["CL"]) or math.isinf(d["CL"]))
        ok = match_ok and bounded_ok and finite_ok
        log(f"  alpha_target={a_deg}deg alpha_live={deg(d['alpha']):.3f}deg q_residual={last['q']:.5f}rad/s: "
            f"live CL={d['CL']:.6f} expected(static+CLq*qHat)={expected_CL:.6f} "
            f"(static-only={expected_CL_static_only:.6f}) "
            f"|CL|<=CLmax+0.1({CFG.CLmax+0.1})={bounded_ok} finite={finite_ok} -> {'PASS' if ok else 'FAIL'}")
        limiter_ok_all = limiter_ok_all and ok
    log(f"HIGH_ALPHA_LIMITER_TEST overall: {'PASS' if limiter_ok_all else 'FAIL'}")
    results["HIGH_ALPHA_LIMITER_TEST"] = limiter_ok_all
    overall = overall and limiter_ok_all
    log("")

    log(f"=== test_aero_kinematic_signs.py OVERALL: {'PASS' if overall else 'FAIL'} ===")

    with open(f"{RESULTS_DIR}/aero_kinematic_signs_result.json", "w") as f:
        json.dump({"results": results, "overall": overall,
                    "diag_at_hold_end": state["phase_diag_at_hold_end"],
                    "any_nan": state["any_nan"], "inertia": state["inertia"]},
                   f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/aero_kinematic_signs_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

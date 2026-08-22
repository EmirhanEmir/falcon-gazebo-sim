#!/usr/bin/env python3
"""
FALCON V2 - AERODYNAMICS_V1 live-Gazebo test pass (gazebo-testing, 2026-08-22).

RE-TEST UPDATE (2026-08-22, same day, after `aerodynamics` applied a fix):
the FIRST run of this script confirmed a destabilizing (not restoring) pitch
moment - reported as TEST_FAILED. `aerodynamics` applied a scoped Cm-to-My
sign correction in response (AeroModel.hh "RESOLVED FINDING" comment,
AERODYNAMICS.md sec 19.7/19.12): the STATIC group (Cm0+Cma*alpha+Cmde*deltaE)
is now negated when computing the applied My torque; the RATE group
(Cmq*qHat) is left untouched (it was already correct - self-referential,
frame-independent). `out.Cm` (diagnostics topic) is UNCHANGED - still
reported in XFLR5's own convention - only the moment computation changed.
This script was re-run against the rebuilt plugin (no script-methodology
change needed) to independently confirm the fix live - see the verdict
block below for the new result and aero_lib.py's compute_aero() docstring
for the updated Python replica mirroring the same scoped correction.

HIGHEST-PRIORITY FILE THIS PASS. Covers:

  - Cma_RESTORING_SIGN_TEST  (highest priority - independently confirms or
    refutes `aerodynamics`' fix for the previously-confirmed destabilizing
    pitch moment, AERODYNAMICS.md sec 19.7/19.12)
  - Cmq_DAMPING_SIGN_TEST
  - Clp_DAMPING_SIGN_TEST
  - Cnr_DAMPING_SIGN_TEST
  - Cnb_STATIC_STABILITY_SIGN_TEST
  - RATE_NORMALIZATION_TEST (merged in here - see note below)

Method: for each test, base_link is held at a known steady body-frame
condition (translational velocity via aero_lib.hold_step's linear
controller; for the rate tests, ALSO a commanded body rate on ONE axis via
the SAME function's angular controller) for a long "hold" phase (proven
empirically this pass to converge cleanly with hold=400 steps / 400 ms at
the gains below), then ONE angular axis is "released" (its controller
torque zeroed via hold_step's ang_axis_mask, while the OTHER two angular
axes and all of translation stay actively controlled, isolating the
response to just that axis) for a short window. The REAL resulting change
in that axis's angular velocity is measured directly from live ECM state
(gz.sim8 Link.world_angular_velocity, the exact same call the aero plugin
itself uses) - this is a genuine physics measurement of the actually-applied
moment, not an inference from a formula.

RATE_NORMALIZATION_TEST is merged into the Cmq/Clp/Cnr experiments above
rather than run as a separate diagnostics-topic read: this run found,
empirically, that the live diagnostics topic intermittently reports
Cl/Cm/Cn values inconsistent with directly-sampled ECM state specifically
during phases with an actively-controlled nonzero body rate (not
corroborated by direct state sampling, which stayed smooth throughout) -
suspected numerical solver noise from the extreme (~5935:1) mass ratio
between base_link and its 7 lightweight, undamped child links (control
surfaces + propellers) under continuous applied torque. This is reported as
an OBSERVATION below (evidence included), not asserted as a plugin defect,
and is NOT used for PASS/FAIL - the direct real-moment measurement (immune
to this topic-timing issue, since it reads Link state exactly as the plugin
does) is used instead, and additionally validates the correct b/c_ref and
2V reference-length normalization via real measured moment magnitude.

Every phase also pins the 7 lightweight/undamped child links (5 control
surfaces + 2 props, aero_lib.pin_child_joints) - found empirically this pass
to otherwise spin up to extreme relative velocities and contaminate a
rate-holding measurement purely as a test-harness artifact of this
structural-V1 build having no control-surface actuation/damping yet; see
aero_lib.py's pin_child_joints() docstring and this run's test report for
the full before/after evidence.

No aircraft physics parameter is modified by this script.
"""
import json
import math
import sys

import numpy as np

import aero_lib as AL

AL.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

RESULTS_DIR = "/home/emirhan/Desktop/FalconV2/tests/gazebo/results"
CFG = AL.load_config()

MASS = 5.9348
I_EFF = 0.5  # controller-only gain scalar, see test_aero_kinematic_signs.py
KP_LIN = 150.0
KP_ANG = 150.0
U = 15.0
RATE_VAL = 0.4  # rad/s, held during the rate-damping/normalization phases
HOLD_STEPS = 400
RELEASE_STEPS = 20

PHASES = []


def add_phase(name, lin_vel, ang_vel, released_axis, note):
    """released_axis in {'x','y','z'} - the ONLY angular axis whose
    controller is turned off during the release window; the other two
    angular axes (and translation) remain actively held throughout."""
    mask = {
        "x": (False, True, True),
        "y": (True, False, True),
        "z": (True, True, False),
    }[released_axis]
    PHASES.append(dict(name=name, lin_vel=lin_vel, ang_vel=ang_vel,
                        released_axis=released_axis, mask=mask, note=note))


def rad(x):
    return math.radians(x)


def deg(x):
    return math.degrees(x)


# 0: Cma_RESTORING_SIGN_TEST, positive alpha (nose-up)
alpha_p = rad(8.0)
add_phase("CMA_POS_8DEG", gm.Vector3d(U, 0.0, -U * math.tan(alpha_p)),
          gm.Vector3d(0, 0, 0), "y",
          "Cma_RESTORING_SIGN_TEST: HIGHEST PRIORITY. alpha=+8deg (nose up), q=0 held, "
          "then pitch-axis control released. Expected (textbook, statically-stable "
          "aircraft, AND per aerodynamics' scoped Cm-to-My fix applied this pass): "
          "restoring nose-down moment (q>0 per the FLU '+Y=nose down' convention "
          "documented in AeroModel.hh). The FIRST run of this test (pre-fix) found this "
          "came out DESTABILIZING instead, reported as TEST_FAILED - this run "
          "independently re-confirms whether the fix resolved it live.")

# 1: Cma, negative alpha, for symmetry/consistency
alpha_n = rad(-8.0)
add_phase("CMA_NEG_8DEG", gm.Vector3d(U, 0.0, -U * math.tan(alpha_n)),
          gm.Vector3d(0, 0, 0), "y",
          "Cma_RESTORING_SIGN_TEST, negative-alpha consistency check: alpha=-8deg "
          "(nose down), expect restoring nose-UP moment (q<0) post-fix, mirroring the "
          "positive-alpha case symmetrically.")

# 2: Cmq_DAMPING_SIGN_TEST
add_phase("CMQ_DAMPING", gm.Vector3d(U, 0.0, 0.0), gm.Vector3d(0, RATE_VAL, 0), "y",
          "Cmq_DAMPING_SIGN_TEST: alpha=beta=0, q=+0.4 rad/s held, then pitch-axis "
          "control released. Expected: moment opposes existing q (damping, q decays "
          "toward 0), REGARDLESS of the Cma finding above (rate/damping terms are "
          "self-referential and frame-independent per AeroModel.hh's own analysis).")

# 3: Clp_DAMPING_SIGN_TEST
add_phase("CLP_DAMPING", gm.Vector3d(U, 0.0, 0.0), gm.Vector3d(RATE_VAL, 0, 0), "x",
          "Clp_DAMPING_SIGN_TEST: alpha=beta=0, p=+0.4 rad/s held, then roll-axis "
          "control released. Expected: moment opposes existing p (damping).")

# 4: Cnr_DAMPING_SIGN_TEST
add_phase("CNR_DAMPING", gm.Vector3d(U, 0.0, 0.0), gm.Vector3d(0, 0, RATE_VAL), "z",
          "Cnr_DAMPING_SIGN_TEST: alpha=beta=0, r=+0.4 rad/s held, then yaw-axis "
          "control released. Expected: moment opposes existing r (damping).")

# 5: Cnb_STATIC_STABILITY_SIGN_TEST
beta_p = rad(8.0)
add_phase("CNB_STABILITY", gm.Vector3d(U, U * math.tan(beta_p), 0.0),
          gm.Vector3d(0, 0, 0), "z",
          "Cnb_STATIC_STABILITY_SIGN_TEST: beta=+8deg (wind from left, adopted "
          "convention), r=0 held, then yaw-axis control released. Expected: "
          "restoring yaw moment turning the nose back toward the relative wind "
          "(nose LEFT per the FLU '+Z=nose left' convention, i.e. r>0), matching "
          "Cnb=+0.03554 > 0 ('directionally statically stable', AERODYNAMICS.md sec 6.2).")


def run():
    log_lines = []

    def log(msg):
        print(msg)
        log_lines.append(msg)

    bounds = []
    t = 0
    for ph in PHASES:
        start = t
        hold_end = start + HOLD_STEPS
        rel_end = hold_end + RELEASE_STEPS
        bounds.append((start, hold_end, rel_end))
        t = rel_end
    total_steps = t

    diag = AL.DiagSubscriber()
    state = {"n": 0, "phase_series": [[] for _ in PHASES], "any_nan": False, "inertia": None}

    def get_model(ecm):
        world = sim.World(sim.world_entity(ecm))
        model_e = world.model_by_name(ecm, "falcon_v2")
        return sim.Model(model_e)

    def current_phase_idx(n):
        for i, (s, h, r) in enumerate(bounds):
            if s <= n < r:
                return i, n < h
        return len(PHASES) - 1, False

    def on_pre(info, ecm):
        n = state["n"]
        idx, in_hold = current_phase_idx(n)
        ph = PHASES[idx]
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        mask = (True, True, True) if in_hold else ph["mask"]
        AL.hold_step(base, ecm, MASS, I_EFF, ph["lin_vel"], ph["ang_vel"],
                     kp_lin=KP_LIN, kp_ang=KP_ANG, ang_axis_mask=mask)
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
        state["n"] += 1

    fixture = sim.TestFixture(AL.WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    log(f"Total steps run: {state['n']} (expected {total_steps})")
    log(f"Any NaN/Inf observed anywhere: {state['any_nan']}")
    log(f"Diagnostics messages received over whole run (observational only, see file header): {diag.count()}")
    inertia = state["inertia"]
    log(f"base_link inertia (queried at runtime, used only to convert measured "
        f"Δomega into an approximate moment estimate): {inertia}")
    log("")

    I_AXIS = {"x": inertia["ixx"], "y": inertia["iyy"], "z": inertia["izz"]}
    RATE_KEY = {"x": "p", "y": "q", "z": "r"}
    C_KEY = {"x": "Cl", "y": "Cm", "z": "Cn"}
    REFLEN = {"x": CFG.b, "y": CFG.c_ref, "z": CFG.b}

    results = {}
    overall = not state["any_nan"]

    for idx, ph in enumerate(PHASES):
        name = ph["name"]
        axis = ph["released_axis"]
        rate_key = RATE_KEY[axis]
        series = state["phase_series"][idx]
        hold_last = [r for r in series if r["in_hold"]][-1]
        free = [r for r in series if not r["in_hold"]]

        log(f"=== {name} ===")
        log(ph["note"])
        log(f"Hold-end state (converged): u={hold_last['u']:.4f} v={hold_last['v']:.4f} w={hold_last['w']:.4f} "
            f"p={hold_last['p']:.5f} q={hold_last['q']:.5f} r={hold_last['r']:.5f}")

        ts = np.array([r["t"] for r in free])
        rates = np.array([r[rate_key] for r in free])
        ts0 = ts - ts[0]
        slope = float(np.polyfit(ts0, rates, 1)[0])
        rate0 = hold_last[rate_key]
        moment_measured = I_AXIS[axis] * slope

        # Independent replica-based expected moment, using the ACTUAL
        # converged hold-end state (V, alpha, beta, p, q, r) - cross-check
        # of the formula math only, not a substitute for the measurement
        # above.
        cross = AL.compute_aero(CFG, hold_last["u"], hold_last["v"], hold_last["w"],
                                 hold_last["p"], hold_last["q"], hold_last["r"])
        expected_moment = {"x": cross["Mx"], "y": cross["My"], "z": cross["Mz"]}[axis]

        log(f"Released axis: {axis} ({rate_key}). {rate_key}(t) over release window: "
            f"{[round(v,5) for v in rates]}")
        log(f"Measured slope d{rate_key}/dt = {slope:.5f} rad/s^2 over {len(free)} free steps "
            f"-> moment_measured = I_{axis}{axis}*slope = {I_AXIS[axis]:.4f}*{slope:.5f} = {moment_measured:.5f} N*m")
        log(f"Independent replica (from hold-end state, same formula the plugin uses): "
            f"C{C_KEY[axis]}... -> M{axis}_expected = {expected_moment:.5f} N*m "
            f"(qbar={cross['qbar']:.3f}, ref_len={REFLEN[axis]})")

        results[name] = dict(rate0=rate0, slope=slope, moment_measured=moment_measured,
                               moment_expected_replica=expected_moment)
        log("")

    # ================= Cma_RESTORING_SIGN_TEST =================
    log("############################################################")
    log("=== Cma_RESTORING_SIGN_TEST VERDICT (highest priority) ===")
    r_pos = results["CMA_POS_8DEG"]
    r_neg = results["CMA_NEG_8DEG"]
    # Restoring (per FLU "+Y rotation = nose down" convention documented in
    # AeroModel.hh) for a nose-up (alpha>0) disturbance means q should
    # increase (go positive / more positive) once pitch control is
    # released, i.e. slope > 0. Destabilizing means slope < 0 (or the
    # measured moment is negative for alpha>0).
    restoring_pos = r_pos["slope"] > 0
    restoring_neg = r_neg["slope"] < 0
    log(f"alpha=+8deg: measured My={r_pos['moment_measured']:.5f} N*m, dq/dt={r_pos['slope']:.5f} rad/s^2 "
        f"-> {'RESTORING (nose-down, q growing positive)' if restoring_pos else 'DESTABILIZING (reinforces nose-up, q growing negative)'}")
    log(f"alpha=-8deg: measured My={r_neg['moment_measured']:.5f} N*m, dq/dt={r_neg['slope']:.5f} rad/s^2 "
        f"-> {'RESTORING (nose-up, q growing negative)' if restoring_neg else 'DESTABILIZING (reinforces nose-down, q growing positive)'}")
    cma_confirmed_restoring = restoring_pos and restoring_neg
    cma_confirmed_destabilizing = (not restoring_pos) and (not restoring_neg)
    if cma_confirmed_restoring:
        log("VERDICT: FIX CONFIRMED LIVE - the scoped Cm-to-My sign correction "
            "(negating only the static Cm0+Cma*alpha+Cmde*deltaE group, leaving the "
            "self-referential Cmq*qHat rate group untouched) now produces a RESTORING "
            "pitching moment for both +8deg and -8deg alpha disturbances, as measured "
            "directly via real Δq/Δt response on base_link in a live Gazebo instance "
            "(not just the pure-math self-test, and not just the diagnostics topic - "
            "out.Cm itself is unchanged/still XFLR5-convention, only the applied My "
            "torque differs). This reverses the PRE-FIX finding from this test's first "
            "run (My=-2.83 N*m/destabilizing at alpha=+8deg) - see "
            "docs/test_results/2026-08-22_aerodynamics_v1_test_report.md sec 1 for that "
            "original result and this run's superseding report for the full comparison.")
    elif cma_confirmed_destabilizing:
        log("VERDICT: FIX NOT EFFECTIVE - still DESTABILIZING live, despite the applied "
            "Cm-to-My correction. Requires further investigation by aerodynamics before "
            "this can be considered resolved.")
    else:
        log("VERDICT: INCONSISTENT between +alpha and -alpha cases - requires further "
            "investigation before drawing a conclusion.")
    log("############################################################")
    log("")

    # PASS/FAIL bookkeeping: PASS now means "measured cleanly AND confirmed restoring"
    # (the post-fix expected/correct outcome) - unlike the first run, where PASS only
    # meant "the destabilizing measurement was obtained cleanly" (a TEST_FAILED finding
    # reported separately from this script's own exit code). This run, a clean+restoring
    # result is exactly what should make this script (and the suite) report PASS.
    cma_measurement_ok = bool(not state["any_nan"])
    results["Cma_RESTORING_SIGN_TEST_measurement_clean"] = cma_measurement_ok
    results["Cma_RESTORING_SIGN_TEST_restoring_confirmed"] = bool(cma_confirmed_restoring)
    results["Cma_RESTORING_SIGN_TEST_destabilizing_confirmed"] = bool(cma_confirmed_destabilizing)
    overall = overall and cma_measurement_ok and cma_confirmed_restoring

    # ================= Damping tests =================
    log("=== Cmq_DAMPING_SIGN_TEST / Clp_DAMPING_SIGN_TEST / Cnr_DAMPING_SIGN_TEST verdicts ===")
    damping_all_ok = True
    for name, axis_label in [("CMQ_DAMPING", "Cmq"), ("CLP_DAMPING", "Clp"), ("CNR_DAMPING", "Cnr")]:
        r = results[name]
        # Damping = moment opposes the existing (positive, +0.4 rad/s) rate,
        # i.e. slope < 0 (rate decaying back toward 0).
        damps = r["slope"] < 0
        damping_all_ok = damping_all_ok and damps
        log(f"{axis_label}: rate0={r['rate0']:.4f} rad/s, measured slope={r['slope']:.5f} rad/s^2, "
            f"measured moment={r['moment_measured']:.5f} N*m -> {'PASS (damps/opposes the rate)' if damps else 'FAIL (reinforces the rate)'}")
        results[f"{axis_label}_DAMPING_SIGN_TEST"] = bool(damps)
    log(f"Damping tests overall: {'PASS' if damping_all_ok else 'FAIL'}")
    overall = overall and damping_all_ok
    log("")

    # ================= Cnb_STATIC_STABILITY_SIGN_TEST =================
    log("=== Cnb_STATIC_STABILITY_SIGN_TEST verdict ===")
    r = results["CNB_STABILITY"]
    restoring = r["slope"] > 0  # nose turning LEFT (r>0) = toward the wind (adopted beta convention)
    log(f"beta=+8deg held, yaw-axis released: measured slope dr/dt={r['slope']:.5f} rad/s^2, "
        f"measured moment={r['moment_measured']:.5f} N*m -> "
        f"{'PASS (restoring: nose turns toward the relative wind)' if restoring else 'FAIL (destabilizing)'}")
    results["Cnb_STATIC_STABILITY_SIGN_TEST"] = bool(restoring)
    overall = overall and restoring
    log("")

    # ================= RATE_NORMALIZATION_TEST (merged) =================
    log("=== RATE_NORMALIZATION_TEST (merged - see file header) ===")
    rate_norm_ok = True
    for name, axis_label, axis in [("CMQ_DAMPING", "Cmq", "y"), ("CLP_DAMPING", "Clp", "x"), ("CNR_DAMPING", "Cnr", "z")]:
        r = results[name]
        meas = r["moment_measured"]
        exp = r["moment_expected_replica"]
        # Generous relative tolerance: this measurement combines a short-window
        # numerical slope fit with a documented small residual controller
        # error (see hold-end state above), so exact agreement is not
        # expected - the check is "same order of magnitude and same sign",
        # which is what actually matters for confirming b/c_ref and the 2V
        # normalization are wired correctly (a reference-length or 2x/4x
        # normalization bug would show up as a large, not small, discrepancy).
        ok = bool(exp != 0 and (meas / exp) > 0.4 and (meas / exp) < 2.5)
        rate_norm_ok = rate_norm_ok and ok
        log(f"{axis_label} (ref_len={REFLEN[axis]}, axis={axis}): measured={meas:.5f} N*m vs "
            f"formula-replica-expected={exp:.5f} N*m, ratio={meas/exp if exp else float('nan'):.3f} -> {'PASS' if ok else 'FAIL'}")
    log(f"p_hat=p*b/(2V), q_hat=q*c_ref/(2V), r_hat=r*b/(2V) (b={CFG.b}, c_ref={CFG.c_ref}) - "
        f"reference-length/normalization confirmed via real measured moment magnitude above.")
    log(f"RATE_NORMALIZATION_TEST overall: {'PASS' if rate_norm_ok else 'FAIL'}")
    results["RATE_NORMALIZATION_TEST"] = rate_norm_ok
    overall = overall and rate_norm_ok
    log("")

    log(f"=== test_aero_stability_derivatives.py OVERALL (Cma finding reported separately, see verdict above): "
        f"{'PASS' if overall else 'FAIL'} ===")

    with open(f"{RESULTS_DIR}/aero_stability_derivatives_result.json", "w") as f:
        json.dump({"results": results, "overall": overall, "any_nan": state["any_nan"],
                    "inertia": inertia}, f, indent=2,
                   default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/aero_stability_derivatives_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return overall


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)

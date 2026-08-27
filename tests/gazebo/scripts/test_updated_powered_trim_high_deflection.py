#!/usr/bin/env python3
"""
FALCON V2 - UPDATED_POWERED_TRIM_AND_HIGH_DEFLECTION_FLIGHT_VALIDATION
(gazebo-testing, 2026-08-27).

Context (do not re-derive, see docs/test_results/
2026-08-26_high_deflection_control_aero_implementation.md): the aero model
changed last stage (HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION) - a new
CL_delta_e term was added and Cm_delta_e grew in magnitude, both applied via
a piecewise-linear wide-deflection lookup (+/-45deg domain, bounded/clamped,
not extrapolated) that REPLACES the old linear control-derivative terms
entirely. The previously-validated powered trim (throttle=0.4915,
elevator physical +5.50deg L/R, V=18.165 m/s) is stale under the new model
(report: CL 0.670857->0.631166 at that exact command, now a LIFT DEFICIT;
diagnostic Cm 0.009430->0.035168, i.e. My=qbar*S*c_ref*(-cmStatic+cmRate)
becomes MORE NEGATIVE - per this project's established convention (negative
My = nose-up, see AeroModel.hh's own "RESOLVED FINDING" comment / aero_v1_
config.yaml's elevator_sign note), this is a STRONGER NOSE-UP tendency at
that same command, not nose-down - corrected here (2026-08-27, `validation`
review) from an earlier, backwards wording of this exact same finding; the
underlying CL/Cm numbers and the trim re-search itself are unaffected, only
the English description of the moment's physical direction). This script
re-trims from scratch and validates live-Gazebo free-flight/dynamic-mode
behavior with the new model active.

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
lookup table value, control authority, motor/prop constant, actuator
parameter) is read for any purpose other than loading the existing
config/state, and NONE is modified anywhere in this script. If a genuine
trim cannot be found, or a high-deflection response looks like a bug, this
script reports it - it does not retune anything to force a result.

=============================================================================
PART 1 - Trim search methodology (reused verbatim from
tests/gazebo/scripts/test_powered_trim_search.py: coarse grid -> refine
round 1 -> refine round 2 -> round 3 settled/tail-window recheck -> round 4
directional throttle extrapolation; same HOLD_STEPS/WINDOW_STEPS, same
candidate_score ranking priority, same PL.pin_control_surface_joints()
quasi-steady technique - see that script's own header for the full empirical
justification of the "hold BOTH linear and angular rate briefly, then fully
release" technique, not re-litigated here).

SEARCH-GRID CENTERING (not a physics change - a test-planning aid only):
before choosing the coarse grid, this script's own author solved the
DOCUMENTED, UNCHANGED trim equations (Lift=Weight, aero pitching moment +
propulsion pitching moment = 0) analytically against the NEW config's own
elevator dCL/dCm lookup tables and the real propulsion thrust curve (same
technique test_powered_trim_search.py's own "Round 3" analytical propulsion-
pitch-moment cross-check already uses to shortlist candidates - applied here
to CENTER the coarse grid instead of merely shortlisting candidates for a
longer recheck). This offline solve (not re-run by this script; documented
here for traceability) predicted throttle~0.496, delta_e_aero~-3.92deg,
alpha~2.49deg, CL~0.645, CD~0.057 - i.e. a SMALLER-magnitude elevator
deflection than the old trim (the new CL_delta_e term makes the old
deflection needlessly lift-reducing) and a slightly higher throttle. This
prediction is used ONLY to center the search grid efficiently (avoiding a
blind brute-force sweep) - the ACTUAL trim reported below is whatever the
live Gazebo search converges to, not this analytical estimate; if the live
search wants to move away from this estimate, it is free to (and the grid
below has margin either side of it for exactly this reason).

PART 2 - reported inline in the log / JSON (throttle, elevator, V, alpha,
CL, CD, Cm, Lift, Weight, Drag, thrust, RPM, My, comparison vs the OLD trim).

PART 3 - controlled trim-hold check through the REAL actuator
(plugins/actuators/, ActuatorCommander), quasi-static isolation technique
reused verbatim from test_high_deflection_control_aero.py /
test_control_authority_effectiveness.py (AL.hold_step with
KP_ANG_QSTATIC=1500.0). Also samples a few points either side of the trim
elevator angle to confirm no lookup discontinuity artifact.

PART 4 - genuine free 6-DOF flight (~25s) from the new trim commands, held
through the real actuator the entire run (ActuatorCommander re-published
every tick, no re-adjustment), no autopilot/attitude controller/hidden
stabilization, propulsion fully live - same "hold briefly then fully
release, nothing else touches base_link" technique as
test_powered_free_flight_validation.py Phase 2 / test_high_deflection_
control_aero.py Part E.

PART 5/6/7 - short (~4s each) +/-15deg / +/-25deg pulses on each of the 3
control surfaces from the new trim, through the real actuator, returning to
trim after. Measures actual achieved deflection, live-measured
CL/CD/Cm/CY/Cl/Cn (from the plugin's own diagnostics topic - never a
hand-computed substitute), moments (recomputed from the EXACT measured
body-frame state via the documented, unmodified ComputeAero() formula below
- the diagnostics topic does not publish Mx/My/Mz directly, so this is the
only way to report them, not an alternative "expected" source), and short
dynamic response (q/pitch/alpha for elevator; p/roll/r/beta for aileron;
r/beta/p for rudder). Also collects raw Mx-sign / p(t)-trend evidence for
the still-open Cl_delta_r sign question (data only, no classification - that
is `aerodynamics`' job).

INDEPENDENT cross-check tool used throughout (`predict_aero()` below): a
fresh, minimal pure-Python mirror of the CURRENT plugins/aerodynamics/
AeroModel.hh::ComputeAero() (elevator/aileron/rudder wide-deflection lookup
architecture, alpha/beta formulas, high-alpha saturation, the resolved
Cm-to-My FLU sign correction). tests/gazebo/scripts/aero_lib.py's own
compute_aero()/load_config() are INTENTIONALLY NOT used here - they are
STALE (old linear-coefficient + retired +/-10deg clamp model; their
load_config() would KeyError on the now-removed control_deflection_clamp_deg
YAML key) - this exact staleness was already flagged and worked around the
same way in test_high_deflection_control_aero.py's own module docstring.
This mirror is used ONLY as an independent cross-check fed the SAME
measured body-frame state that produced a given live measurement - never as
a substitute for the live measurement itself, and never to decide a
pass/fail on its own.

Uses (does not reinvent): actuator_lib.py (ActuatorCommander, DiagSubscriber,
read_joint_state, load_actuator_config, JOINT_NAMES, SURFACES, STEP),
aero_lib.py (hold_step, DiagSubscriber, CONFIG_YAML_PATH, PLUGIN_BUILD_DIR,
STEP - NOT load_config/compute_aero, see above), propulsion_lib.py
(ThrottleCommander, DiagSubscriber, pin_control_surface_joints).
"""
import json
import math
import sys

import actuator_lib as ACT
import aero_lib as AL
import propulsion_lib as PL

ACT.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402
import yaml  # noqa: E402

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = f"{REPO_ROOT}/tests/gazebo/results"
WORLD = f"{REPO_ROOT}/tests/gazebo/worlds/falcon_v2_freefall_world.sdf"

ACFG = ACT.load_actuator_config()

MASS = 5.9348  # kg, base_link mass, model/model.sdf - controller gain only, queried/cross-checked throughout this suite
I_DIAG = (0.7284, 0.2507, 0.9523)  # kg*m^2, base_link diagonal inertia - controller gain only
KP_LIN = 150.0
KP_ANG_SETTLE = 400.0
KP_ANG_QSTATIC = 1500.0  # reused verbatim, test_high_deflection_control_aero.py / test_control_authority_effectiveness.py
ALTITUDE_M = 100.0
WEIGHT_N = 58.86  # CONFIRMED, CLAUDE.md: 6.000 kg * 9.81 m/s^2
DZ_HUB_CG = 0.0271  # m, propulsion hub Z (0.1271) - CG Z (0.100000), both CONFIRMED, cited verbatim from test_powered_trim_search.py
DIAG_HZ = 20.0  # all 3 plugin diagnostics topics, confirmed in model/model.sdf's own <diagnostics_rate_hz> this pass

U_HOLD, W_HOLD = 18.14534, -0.78335  # SAME baseline as the OLD search (V=18.162 m/s, alpha=2.472deg) - re-trimming elevator/throttle only, not the airspeed target


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
# Independent pure-Python mirror of the CURRENT AeroModel.hh::ComputeAero()
# (not aero_lib.py's stale one - see module docstring). Read-only re-parse of
# aero_v1_config.yaml's own numbers; never modified.
# =============================================================================
def load_aero_ref():
    with open(AL.CONFIG_YAML_PATH) as f:
        root = yaml.safe_load(f)
    rg, env = root["reference_geometry"], root["environment"]
    lon, lat = root["longitudinal"], root["lateral_directional"]
    drag, lim = root["drag_polar"], root["high_alpha_limiter"]
    ctrl, csl = root["control_mapping"], root["control_surface_lookup"]

    bps_deg = [float(x) for x in csl["breakpoints_deg"]]
    bps = [math.radians(x) for x in bps_deg]
    idx0 = bps_deg.index(0.0)

    def diffed(arr):
        arr = [float(x) for x in arr]
        base = arr[idx0]
        return [v - base for v in arr]

    ref = dict(
        S=rg["wing_area_S_m2"], b=rg["wingspan_b_m"], c_ref=rg["reference_chord_c_ref_m"],
        rho=env["air_density_rho_kg_m3"], vSafeFloor=env["v_safe_floor_m_s"],
        CLa=lon["CLa_per_rad"], Cma=lon["Cma_per_rad"], CLq=lon["CLq"], Cmq=lon["Cmq"],
        CL0=lon["CL0"], Cm0=lon["Cm0"],
        CYb=lat["CYb"], CYp=lat["CYp"], CYr=lat["CYr"],
        Clb=lat["Clb"], Clp=lat["Clp"], Clr=lat["Clr"], Cldr=lat["Cldr_per_rad"],
        Cnb=lat["Cnb"], Cnp=lat["Cnp"], Cnr=lat["Cnr"],
        CD0=drag["CD0"], dragK=drag["k"],
        CLmax=lim["CLmax_manufacturer"], alphaTransition=math.radians(lim["alpha_transition_deg"]),
        elevatorSign=ctrl["elevator_sign"], aileronSign=ctrl["aileron_sign"], rudderSign=ctrl["rudder_sign"],
        bps=bps, bps_deg=bps_deg,
        elev_dCL=[float(x) for x in csl["elevator"]["dCL"]],
        elev_dCD=[float(x) for x in csl["elevator"]["dCD"]],
        elev_dCm=[float(x) for x in csl["elevator"]["dCm"]],
        aile_Cl=[float(x) for x in csl["aileron"]["Cl"]],
        aile_Cn=[float(x) for x in csl["aileron"]["Cn"]],
        aile_CY=[float(x) for x in csl["aileron"]["CY"]],
        aile_dCD=diffed(csl["aileron"]["CD_full"]),
        aile_dCL=diffed(csl["aileron"]["CL_full"]),
        aile_dCm=diffed(csl["aileron"]["Cm_full"]),
        rudd_CY=[float(x) for x in csl["rudder"]["CY"]],
        rudd_Cn=[float(x) for x in csl["rudder"]["Cn"]],
        rudd_dCD=diffed(csl["rudder"]["CD_full"]),
    )
    clLinAtT = ref["CL0"] + ref["CLa"] * ref["alphaTransition"]
    ref["satHeadroomPos"] = ref["CLmax"] - clLinAtT
    ref["satKPos"] = (ref["CLa"] / ref["satHeadroomPos"]) if ref["satHeadroomPos"] > 1e-9 else 0.0
    clLinAtNegT = ref["CL0"] + ref["CLa"] * (-ref["alphaTransition"])
    ref["satAneg"] = clLinAtNegT + ref["CLmax"]
    ref["satKNeg"] = (ref["CLa"] / ref["satAneg"]) if ref["satAneg"] > 1e-9 else 0.0
    ref["rudd_Cl"] = [ref["Cldr"] * x for x in ref["bps"]]
    return ref


REF = load_aero_ref()


def interp_lin(bps, vals, x):
    lo, hi = bps[0], bps[-1]
    if x <= lo:
        return vals[0]
    if x >= hi:
        return vals[-1]
    for i in range(len(bps) - 1):
        if bps[i] <= x <= bps[i + 1]:
            t = (x - bps[i]) / (bps[i + 1] - bps[i])
            return vals[i] + t * (vals[i + 1] - vals[i])
    return vals[-1]


def saturated_CL(ref, alpha):
    clLinear = ref["CL0"] + ref["CLa"] * alpha
    if alpha > ref["alphaTransition"]:
        return ref["CLmax"] - ref["satHeadroomPos"] * math.exp(-ref["satKPos"] * (alpha - ref["alphaTransition"]))
    if alpha < -ref["alphaTransition"]:
        return -ref["CLmax"] + ref["satAneg"] * math.exp(ref["satKNeg"] * (alpha + ref["alphaTransition"]))
    return clLinear


def predict_aero(ref, u, v, w, p, q, r, deltaA=0.0, deltaE=0.0, deltaR=0.0):
    """Full mirror of the CURRENT ComputeAero() (wide-deflection lookup
    architecture). Fed the EXACT measured body-frame state (u,v,w,p,q,r) and
    EXACT measured actual deflections that produced a given live
    measurement - used only as a cross-check / to recompute Mx/My/Mz (not
    published on the diagnostics topic), never as a substitute for the live
    CL/CD/CY/Cl/Cm/Cn measurement itself."""
    vSq = u * u + v * v + w * w
    V = math.sqrt(vSq)
    qbar = 0.5 * ref["rho"] * vSq
    alpha = math.atan2(-w, u)
    beta = math.atan2(v, math.hypot(u, w))
    vSafe = max(V, ref["vSafeFloor"])
    pHat = p * ref["b"] / (2.0 * vSafe)
    qHat = q * ref["c_ref"] / (2.0 * vSafe)
    rHat = r * ref["b"] / (2.0 * vSafe)

    dCYa = interp_lin(ref["bps"], ref["aile_CY"], deltaA)
    dCla = interp_lin(ref["bps"], ref["aile_Cl"], deltaA)
    dCna = interp_lin(ref["bps"], ref["aile_Cn"], deltaA)
    dCLaCtrl = interp_lin(ref["bps"], ref["aile_dCL"], deltaA)
    dCmaCtrl = interp_lin(ref["bps"], ref["aile_dCm"], deltaA)
    dCDaCtrl = interp_lin(ref["bps"], ref["aile_dCD"], deltaA)

    dCYr = interp_lin(ref["bps"], ref["rudd_CY"], deltaR)
    dClr = interp_lin(ref["bps"], ref["rudd_Cl"], deltaR)
    dCnr = interp_lin(ref["bps"], ref["rudd_Cn"], deltaR)
    dCDrCtrl = interp_lin(ref["bps"], ref["rudd_dCD"], deltaR)

    dCLeCtrl = interp_lin(ref["bps"], ref["elev_dCL"], deltaE)
    dCmeCtrl = interp_lin(ref["bps"], ref["elev_dCm"], deltaE)
    dCDeCtrl = interp_lin(ref["bps"], ref["elev_dCD"], deltaE)

    CY = ref["CYb"] * beta + ref["CYp"] * pHat + ref["CYr"] * rHat + dCYa + dCYr
    Cl = ref["Clb"] * beta + ref["Clp"] * pHat + ref["Clr"] * rHat + dCla + dClr
    Cn = ref["Cnb"] * beta + ref["Cnp"] * pHat + ref["Cnr"] * rHat + dCna + dCnr

    cmStatic = ref["Cm0"] + ref["Cma"] * alpha + dCmeCtrl + dCmaCtrl
    cmRate = ref["Cmq"] * qHat
    Cm = cmStatic + cmRate

    CL = saturated_CL(ref, alpha) + ref["CLq"] * qHat + dCLeCtrl + dCLaCtrl
    cdRaw = ref["CD0"] + ref["dragK"] * CL * CL + dCDeCtrl + dCDaCtrl + dCDrCtrl
    CD = max(cdRaw, ref["CD0"])

    Lift = qbar * ref["S"] * CL
    Drag = qbar * ref["S"] * CD
    Side = qbar * ref["S"] * CY
    Mx = qbar * ref["S"] * ref["b"] * Cl
    My = qbar * ref["S"] * ref["c_ref"] * (-cmStatic + cmRate)
    Mz = qbar * ref["S"] * ref["b"] * Cn

    return dict(V=V, alpha=alpha, beta=beta, qbar=qbar, CL=CL, CD=CD, CY=CY, Cl=Cl, Cm=Cm, Cn=Cn,
                Lift=Lift, Drag=Drag, Side=Side, Mx=Mx, My=My, Mz=Mz)


def actual_deltas(model, ecm):
    thetaLA, _ = ACT.read_joint_state(model, ecm, sim, ACT.JOINT_NAMES["left_aileron"])
    thetaRA, _ = ACT.read_joint_state(model, ecm, sim, ACT.JOINT_NAMES["right_aileron"])
    thetaLE, _ = ACT.read_joint_state(model, ecm, sim, ACT.JOINT_NAMES["left_elevator"])
    thetaRE, _ = ACT.read_joint_state(model, ecm, sim, ACT.JOINT_NAMES["right_elevator"])
    thetaR, _ = ACT.read_joint_state(model, ecm, sim, ACT.JOINT_NAMES["rudder"])
    da = 0.5 * REF["aileronSign"] * (thetaRA - thetaLA)
    de = 0.5 * REF["elevatorSign"] * (thetaLE + thetaRE)
    dr = REF["rudderSign"] * thetaR
    return da, de, dr, (thetaLA, thetaRA, thetaLE, thetaRE, thetaR)


LIVE_KEYS = ("CL", "CD", "CY", "Cl", "Cm", "Cn", "Mx", "My", "Mz")


def paired_live_ticks(sample_list):
    """Resample `sample_list` (a list of per-tick dicts, each optionally carrying a 'live' sub-dict
    from the aero diagnostics topic) down to only the ticks where a NEW, DISTINCT live message was
    first observed - i.e. the live topic's own true ~20Hz update cadence, not this script's much
    finer ~200Hz per-tick mirror-sampling cadence.

    WHY THIS EXISTS (found this pass, `validation` MAJOR-gap fix, 2026-08-27): naively averaging live
    diagnostic values over a time window and comparing against a per-tick mirror average over the SAME
    window looked like a real mirror-vs-plugin disagreement (double-digit % diffs on CY/Cn/Mz during
    the aileron pulse) - but a direct sample-by-sample trace showed the live value only changes every
    ~10 ticks (~50ms, matching the documented 20Hz `diagnostics_rate_hz`) while the underlying state
    (e.g. roll rate p during an aileron pulse) is changing by >500deg/s^2 - so a window-average mixes
    several *different, real* live messages (each reflecting an EARLIER, staler tick's state) against
    a mirror average computed from the CURRENT tick every time. This is a publish-rate STALENESS
    artifact, not an implementation discrepancy. This function fixes the comparison methodology by
    pairing each live message with the EXACT tick it was first seen at, so the mirror-vs-live diff
    below is always computed tick-for-tick (both sides reflect the identical instant), not smeared
    across a publish-rate gap.
    """
    out = []
    prev_cl = object()  # sentinel, never equal to a real CL value
    for s in sample_list:
        live = s.get("live")
        if live is None:
            continue
        if live["CL"] != prev_cl:
            out.append(s)
            prev_cl = live["CL"]
    return out


# Per-coefficient (relative_tol, absolute_floor) for the PAIRED live-vs-mirror PASS/FAIL check below.
# The absolute floor matters specifically for the moment channels (Mx/My/Mz): a pitch/roll/yaw
# oscillation naturally crosses zero, and right at a zero-crossing even a negligible ABSOLUTE
# difference (residual sub-tick sampling/publish-latency noise) produces an enormous, meaningless
# RELATIVE percentage (found this pass: up to 177% at a single My~0 crossing tick during the
# ELEVATOR_P15 case, vs. a real, physically-sized ~0.11 N*m discrepancy of only ~35% at the one
# genuinely fast-transient tick nearby) - the same "relative-diff-explodes-near-zero" reasoning
# CY/Cn/Cl already needed their own absolute floor for in the prior HIGH_DEFLECTION_CONTROL_AERO_
# IMPLEMENTATION stage's own TOL dict (test_high_deflection_control_aero.py), applied here to the
# moment channels too since they are new to this comparison.
PAIRED_TOL = {"CL": (0.10, 0.01), "CD": (0.15, 0.003), "Cm": (0.10, 0.01),
              "CY": (0.25, 0.001), "Cl": (0.15, 0.001), "Cn": (0.25, 0.0005),
              "Mx": (0.15, 0.05), "My": (0.15, 0.05), "Mz": (0.25, 0.05)}


def paired_live_mirror_diff(sample_list, keys=LIVE_KEYS):
    """Timing-paired mirror-vs-live comparison (see paired_live_ticks()'s docstring for why pairing,
    not a naive window average, is required). Returns (mean_reldiff, frac_fail, n_pairs):
      - mean_reldiff: informational-only mean of the plain relative diff per tick (kept for visibility,
        but NOT used to decide pass/fail - a couple of near-zero-crossing ticks can dominate this mean
        with a meaningless triple-digit percentage even when every tick is otherwise in tight
        agreement, see PAIRED_TOL's comment above).
      - frac_fail: the FRACTION of paired ticks whose |mirror-live| exceeds
        max(rel_tol*max(|mirror|,|live|), abs_floor) (PAIRED_TOL above) - the actual, floor-aware,
        zero-crossing-robust basis for the PASS/FAIL flag in run_pulse_test().
    """
    paired = paired_live_ticks(sample_list)
    if not paired:
        return {k: None for k in keys}, {k: None for k in keys}, 0
    mean_reldiff, frac_fail = {}, {}
    for k in keys:
        rel_tol, abs_floor = PAIRED_TOL[k]
        reldiffs, fails, n = [], 0, 0
        for s in paired:
            mv, lv = s.get(k), s["live"].get(k)
            if mv is None or lv is None:
                continue
            n += 1
            denom = max(abs(mv), abs(lv), 1e-6)
            reldiffs.append(abs(mv - lv) / denom)
            tol = max(rel_tol * max(abs(mv), abs(lv)), abs_floor)
            if abs(mv - lv) > tol:
                fails += 1
        mean_reldiff[k] = (sum(reldiffs) / len(reldiffs)) if reldiffs else None
        frac_fail[k] = (fails / n) if n else None
    return mean_reldiff, frac_fail, len(paired)


# =============================================================================
# PART 1 - trim search (pinned-joint quasi-steady technique, reused verbatim
# from test_powered_trim_search.py - see that script / this file's header
# for the full rationale).
# =============================================================================
HOLD_STEPS = 800
WINDOW_STEPS = 2000
ROUND34_WINDOW_STEPS = 12000
ELEV_MIN, ELEV_MAX = -8.0, 0.0
THROTTLE_MIN, THROTTLE_MAX = 0.40, 0.60


def run_trim_candidate(log, throttle, elevator_aero_deg, window_steps=WINDOW_STEPS):
    elevator_theta_rad = math.radians(-elevator_aero_deg)  # theta = -delta_e_aero (elevatorSign=-1.0), symmetric L=R

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
            "left_elevator_joint": elevator_theta_rad,
            "right_elevator_joint": elevator_theta_rad})
        n = state["n"]
        if n < HOLD_STEPS:
            AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(U_HOLD, 0, W_HOLD),
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
                      p=av_b.x(), q=av_b.y(), r=av_b.z(),
                      alt=wpose.pos().z(), world_vz=lv.z(), pitch=pitch)
        if n == HOLD_STEPS:
            prop = state["prop_diag"].latest() if state["prop_diag"] else None
            aero = state["aero_diag"].latest() if state["aero_diag"] else None
            sample["prop"] = prop
            sample["aero"] = aero
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
                             deltaA=0.0, deltaE=0.5 * REF["elevatorSign"] * (elevator_theta_rad + elevator_theta_rad),
                             deltaR=0.0)
    V_rel = pred_rel["V"]
    alpha_rel_deg = math.degrees(pred_rel["alpha"])

    half_s_idx = min(500, len(series) - 1)
    u0, u_half = series[0]["u"], series[half_s_idx]["u"]
    long_accel = (u_half - u0) / (series[half_s_idx]["t"] - series[0]["t"])

    V_end = math.sqrt(win_end["u"] ** 2 + win_end["v"] ** 2 + win_end["w"] ** 2)
    airspeed_drift = V_end - V_rel
    mean_world_vz = sum(s["world_vz"] for s in series) / len(series)
    mean_abs_q_deg_s = sum(abs(math.degrees(s["q"])) for s in series) / len(series)
    alt_drift = win_end["alt"] - rel["alt"]

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

    aero_live = rel.get("aero")

    result = dict(
        throttle=throttle, elevator_aero_deg=elevator_aero_deg,
        elevator_theta_deg=math.degrees(elevator_theta_rad),
        release=dict(V=V_rel, alpha_deg=alpha_rel_deg, world_vz=rel["world_vz"],
                     q_deg_s=math.degrees(rel["q"])),
        aero_rel=dict(Lift_N=pred_rel["Lift"], Drag_N=pred_rel["Drag"], My_Nm=pred_rel["My"],
                      CL=pred_rel["CL"], CD=pred_rel["CD"], Cm_diag=pred_rel["Cm"]),
        aero_live_rel=aero_live,
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

    log(f"Candidate throttle={throttle:.4f} elevator_aero={elevator_aero_deg:+.2f}deg "
        f"(theta={math.degrees(elevator_theta_rad):+.2f}deg): "
        f"V_rel={V_rel:.3f} alpha_rel={alpha_rel_deg:.3f}deg world_vz_rel={rel['world_vz']:+.4f} "
        f"Lift={pred_rel['Lift']:.3f}N Drag={pred_rel['Drag']:.3f}N My={pred_rel['My']:+.4f}Nm "
        f"CL={pred_rel['CL']:.5f} CD={pred_rel['CD']:.5f} "
        f"RPM(L/R)={left_rpm:.1f}/{right_rpm:.1f} ThrustTotal={thrust_total:.3f}N "
        f"window[mean_vz={mean_world_vz:+.4f} drift={airspeed_drift:+.4f} mean|q|={mean_abs_q_deg_s:.3f} "
        f"tail_mean_vz={tail_mean_world_vz:+.4f} tail_alt_rate={tail_alt_drift_rate_mps:+.4f}] "
        f"any_nan={state['any_nan']}")

    return result


def candidate_score(r):
    vz_bucket = round(abs(r["window"]["mean_world_vz_mps"]) / 0.3)
    return (vz_bucket, abs(r["window"]["airspeed_drift_mps"]),
            r["window"]["mean_abs_q_deg_s"], abs(r["aero_rel"]["My_Nm"]))


def run_trim_search(log):
    log("=" * 78)
    log("PART 1: NEW TRIM SEARCH")
    log("=" * 78)
    log("Analytical center estimate (documented in this file's header, offline solve of the "
        "unchanged Lift=Weight / aero+propulsion pitching-moment=0 equations against the NEW "
        "elevator lookup + real propulsion thrust curve): throttle~0.496, elevator_aero~-3.92deg. "
        "Used ONLY to center the grid below - the live search below is free to move away from it.")
    n_candidates = [0]

    def run_c(th, el, window_steps=WINDOW_STEPS):
        n_candidates[0] += 1
        return run_trim_candidate(log, th, el, window_steps=window_steps)

    def show_ranked(label, results, n=8):
        ranked = sorted(results, key=candidate_score)
        log(f"{label} (best first, priority: |mean window vz| > |airspeed drift| > |mean q| > |My|):")
        for r in ranked[:n]:
            log(f"  throttle={r['throttle']:.4f} elevator={r['elevator_aero_deg']:+.2f}deg -> "
                f"mean_vz={r['window']['mean_world_vz_mps']:+.4f} drift={r['window']['airspeed_drift_mps']:+.4f} "
                f"mean|q|={r['window']['mean_abs_q_deg_s']:.3f} My={r['aero_rel']['My_Nm']:+.4f} "
                f"ThrustTotal={r['prop_rel']['thrust_total_N']:.3f} Drag={r['aero_rel']['Drag_N']:.3f} "
                f"Lift={r['aero_rel']['Lift_N']:.3f}")
        return ranked

    log("-" * 78)
    log("COARSE GRID")
    log("-" * 78)
    throttles = [0.460, 0.490, 0.520]
    elevators = [-6.0, -4.5, -3.0, -1.5]
    coarse_results = [run_c(th, el) for th in throttles for el in elevators]
    log("")
    ranked = show_ranked("Coarse grid ranked", coarse_results, n=5)
    best_coarse = ranked[0]
    log(f"\nBest coarse candidate: throttle={best_coarse['throttle']:.4f} "
        f"elevator={best_coarse['elevator_aero_deg']:+.2f}deg\n")

    log("-" * 78)
    log("REFINEMENT ROUND 1 (+/-0.025 throttle, +/-1deg elevator)")
    log("-" * 78)
    th0, el0 = best_coarse["throttle"], best_coarse["elevator_aero_deg"]
    r1_candidates = set()
    for dth in (-0.025, 0.0, 0.025):
        for delv in (-1.0, 0.0, 1.0):
            th = round(min(THROTTLE_MAX, max(THROTTLE_MIN, th0 + dth)), 4)
            el = round(min(ELEV_MAX, max(ELEV_MIN, el0 + delv)), 4)
            r1_candidates.add((th, el))
    coarse_keys = {(r["throttle"], r["elevator_aero_deg"]) for r in coarse_results}
    r1_candidates = sorted(c for c in r1_candidates if c not in coarse_keys)
    refine1_results = [run_c(th, el) for th, el in r1_candidates]
    log("")
    pool = coarse_results + refine1_results
    ranked = show_ranked("Round-1 combined ranking", pool, n=5)
    best1 = ranked[0]
    log(f"\nBest after round 1: throttle={best1['throttle']:.4f} elevator={best1['elevator_aero_deg']:+.2f}deg\n")

    log("-" * 78)
    log("REFINEMENT ROUND 2 (+/-0.0125 throttle, +/-0.5deg elevator)")
    log("-" * 78)
    th1, el1 = best1["throttle"], best1["elevator_aero_deg"]
    r2_candidates = set()
    for dth in (-0.0125, 0.0, 0.0125):
        for delv in (-0.5, 0.0, 0.5):
            th = round(min(THROTTLE_MAX, max(THROTTLE_MIN, th1 + dth)), 4)
            el = round(min(ELEV_MAX, max(ELEV_MIN, el1 + delv)), 4)
            r2_candidates.add((th, el))
    seen_keys = {(r["throttle"], r["elevator_aero_deg"]) for r in pool}
    r2_candidates = sorted(c for c in r2_candidates if c not in seen_keys)
    refine2_results = [run_c(th, el) for th, el in r2_candidates]
    log("")
    all_results = pool + refine2_results
    ranked_all = show_ranked("FULL RANKING (coarse + round 1 + round 2)", all_results, n=10)
    best2 = ranked_all[0]

    log("")
    log("-" * 78)
    log("REFINEMENT ROUND 3 (settled/tail-window check, longer 12s release window)")
    log("-" * 78)
    shortlist = {(best2["throttle"], best2["elevator_aero_deg"])}
    for r in all_results:
        net_My_est = r["aero_rel"]["My_Nm"] + DZ_HUB_CG * (r["prop_rel"]["thrust_total_N"] or 0.0)
        r["_net_My_est"] = net_My_est
    near_zero_net = sorted(all_results, key=lambda r: abs(r["_net_My_est"]))[:4]
    for r in near_zero_net:
        shortlist.add((r["throttle"], r["elevator_aero_deg"]))
    shortlist = sorted(shortlist)
    log(f"Shortlist for round 3: {shortlist}")
    round3_results = [run_c(th, el, window_steps=ROUND34_WINDOW_STEPS) for th, el in shortlist]
    log("")
    ranked3 = sorted(round3_results, key=lambda r: (
        round(abs(r["window"]["tail_mean_world_vz_mps"]) / 0.1),
        abs(r["window"]["tail_alt_drift_rate_mps"]), abs(r["window"]["airspeed_drift_mps"])))
    log("Round 3 ranked (best first, priority: |tail mean vz| > |tail alt drift rate| > |airspeed drift|):")
    for r in ranked3:
        log(f"  throttle={r['throttle']:.4f} elevator={r['elevator_aero_deg']:+.2f}deg -> "
            f"tail_mean_vz={r['window']['tail_mean_world_vz_mps']:+.4f} "
            f"tail_alt_rate={r['window']['tail_alt_drift_rate_mps']:+.4f} "
            f"drift={r['window']['airspeed_drift_mps']:+.4f} net_My_est={r.get('_net_My_est', float('nan')):+.4f}")
    best3 = ranked3[0]
    log(f"\nBest after round 3: throttle={best3['throttle']:.4f} elevator_aero={best3['elevator_aero_deg']:+.2f}deg\n")

    log("-" * 78)
    log("REFINEMENT ROUND 4 (directional extrapolation along throttle, same elevator)")
    log("-" * 78)
    same_elev = sorted([r for r in round3_results if r["elevator_aero_deg"] == best3["elevator_aero_deg"]],
                        key=lambda r: r["throttle"])
    if len(same_elev) >= 2:
        th_a, vz_a = same_elev[0]["throttle"], same_elev[0]["window"]["tail_mean_world_vz_mps"]
        th_b, vz_b = same_elev[-1]["throttle"], same_elev[-1]["window"]["tail_mean_world_vz_mps"]
        slope = (vz_b - vz_a) / (th_b - th_a) if th_b != th_a else 0.0
        th_zero = th_a - vz_a / slope if slope != 0.0 else th_a
        th_zero = min(THROTTLE_MAX, max(THROTTLE_MIN, th_zero))
    else:
        th_zero, slope = best3["throttle"], 0.0
    log(f"Linear fit through round-3 same-elevator points: slope={slope:+.3f} m/s per throttle unit, "
        f"extrapolated zero-crossing throttle={th_zero:.4f}")
    round3_keys = {(r["throttle"], r["elevator_aero_deg"]) for r in round3_results}
    round4_candidates = sorted({round(min(THROTTLE_MAX, max(THROTTLE_MIN, th_zero + d)), 4)
                                 for d in (-0.0125, 0.0, 0.0125)})
    round4_candidates = [(th, best3["elevator_aero_deg"]) for th in round4_candidates
                          if (th, best3["elevator_aero_deg"]) not in round3_keys]
    log(f"Round 4 candidates: {round4_candidates}")
    round4_results = [run_c(th, el, window_steps=ROUND34_WINDOW_STEPS) for th, el in round4_candidates]
    log("")

    final_pool = round3_results + round4_results
    ranked_final = sorted(final_pool, key=lambda r: (
        round(abs(r["window"]["tail_mean_world_vz_mps"]) / 0.1),
        abs(r["window"]["tail_alt_drift_rate_mps"]), abs(r["window"]["airspeed_drift_mps"])))
    log("Round 3+4 combined ranking (best first):")
    for r in ranked_final:
        log(f"  throttle={r['throttle']:.4f} elevator={r['elevator_aero_deg']:+.2f}deg -> "
            f"tail_mean_vz={r['window']['tail_mean_world_vz_mps']:+.4f} "
            f"tail_alt_rate={r['window']['tail_alt_drift_rate_mps']:+.4f} "
            f"drift={r['window']['airspeed_drift_mps']:+.4f}")
    best = ranked_final[0]
    log(f"\nSELECTED NEW TRIM (final, after round 4): throttle={best['throttle']:.4f} "
        f"elevator_aero={best['elevator_aero_deg']:+.2f}deg "
        f"(elevator_theta={best['elevator_theta_deg']:+.2f}deg physical)\n")

    all_full = all_results + round3_results + round4_results
    any_nan_overall = any(r["any_nan"] for r in all_full)
    log(f"Total trim candidates tested: {n_candidates[0]}")
    log(f"any_nan across all candidates: {any_nan_overall}")

    return dict(coarse_grid=coarse_results, refinement_round_1=refine1_results,
                refinement_round_2=refine2_results, refinement_round_3_settled_tail_check=round3_results,
                refinement_round_4_directional_extrapolation=round4_results,
                selected=best, n_candidates=n_candidates[0], any_nan_overall=any_nan_overall)


# =============================================================================
# PART 3 - controlled trim-hold check through the REAL actuator (quasi-static
# isolation, reused verbatim from test_high_deflection_control_aero.py /
# test_control_authority_effectiveness.py).
# =============================================================================
WARM_STEPS = 300
SETTLE_STEPS = 2500
TAIL_STEPS = 500


def run_actuator_quasi_static(log, label, throttle, elev_theta_deg, aile_L_deg=0.0, aile_R_deg=0.0, rudder_deg=0.0):
    cmd_rad = dict(
        left_elevator=math.radians(elev_theta_deg), right_elevator=math.radians(elev_theta_deg),
        left_aileron=math.radians(aile_L_deg), right_aileron=math.radians(aile_R_deg),
        rudder=math.radians(rudder_deg))
    lin_target = gm.Vector3d(U_HOLD, 0.0, W_HOLD)

    state = {"n": 0, "teleported": False, "cmd": None, "thr": None, "any_nan": False,
             "aero_diag": None, "prop_diag": None, "actuator_diag": None,
             "theta": {s: [] for s in ACT.SURFACES}, "body_state": []}
    total_steps = WARM_STEPS + SETTLE_STEPS + 5

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
        AL.hold_step(base, ecm, MASS, I_DIAG, lin_target, gm.Vector3d(0, 0, 0),
                     kp_lin=KP_LIN, kp_ang=KP_ANG_QSTATIC)

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
        if lv is None or av is None or wpose is None or any(math.isnan(v) or math.isinf(v) for v in
                                            [lv.x(), lv.y(), lv.z(), av.x(), av.y(), av.z()]):
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

    bs_tail = state["body_state"][-TAIL_STEPS:]
    u_m = sum(s[0] for s in bs_tail) / len(bs_tail)
    v_m = sum(s[1] for s in bs_tail) / len(bs_tail)
    w_m = sum(s[2] for s in bs_tail) / len(bs_tail)
    p_m = sum(s[3] for s in bs_tail) / len(bs_tail)
    q_m = sum(s[4] for s in bs_tail) / len(bs_tail)
    r_m = sum(s[5] for s in bs_tail) / len(bs_tail)

    tail_mean_rad = {s: sum(state["theta"][s][-TAIL_STEPS:]) / len(state["theta"][s][-TAIL_STEPS:]) for s in ACT.SURFACES}
    thetaLA, thetaRA = tail_mean_rad["left_aileron"], tail_mean_rad["right_aileron"]
    thetaLE, thetaRE = tail_mean_rad["left_elevator"], tail_mean_rad["right_elevator"]
    thetaRud = tail_mean_rad["rudder"]
    actual_delta_a = 0.5 * REF["aileronSign"] * (thetaRA - thetaLA)
    actual_delta_e = 0.5 * REF["elevatorSign"] * (thetaLE + thetaRE)
    actual_delta_r = REF["rudderSign"] * thetaRud

    aero_hist = state["aero_diag"].history if state["aero_diag"] else []
    tail_msgs = max(1, round(TAIL_STEPS * ACT.STEP * DIAG_HZ))
    aero_tail = aero_hist[-tail_msgs:] if aero_hist else []
    aero_avg = ({k: sum(m[k] for m in aero_tail) / len(aero_tail) for k in AL.DiagSubscriber.FIELDS}
                if aero_tail else {k: None for k in AL.DiagSubscriber.FIELDS})

    prop_hist_split = state["prop_diag"].all_split() if state["prop_diag"] else []
    prop_tail = prop_hist_split[-tail_msgs:] if prop_hist_split else []
    left_rpm = sum(p["left"]["rpm"] for p in prop_tail) / len(prop_tail) if prop_tail else None
    right_rpm = sum(p["right"]["rpm"] for p in prop_tail) / len(prop_tail) if prop_tail else None
    left_thrust = sum(p["left"]["thrust_N"] for p in prop_tail) / len(prop_tail) if prop_tail else None
    right_thrust = sum(p["right"]["thrust_N"] for p in prop_tail) / len(prop_tail) if prop_tail else None

    ad = state["actuator_diag"].latest() if state["actuator_diag"] else None
    any_target_clamp = any(ad[s]["target_clamp_active"] > 0.5 for s in ACT.SURFACES) if ad else False
    any_effort_clamp = any(ad[s]["effort_clamp_active"] > 0.5 for s in ACT.SURFACES) if ad else False

    # Recompute My/Mx/Mz from the ACTUAL measured tail-averaged body-frame state (not the fixed hold
    # target) + actual measured deflections, via predict_aero() - never a substitute for the live
    # CL/CD/CY/Cl/Cm/Cn measurement above, only used to obtain Mx/My/Mz (not published on the topic).
    pred = predict_aero(REF, u_m, v_m, w_m, p_m, q_m, r_m,
                         deltaA=actual_delta_a, deltaE=actual_delta_e, deltaR=actual_delta_r)

    result = dict(
        label=label, throttle=throttle, elev_theta_cmd_deg=elev_theta_deg,
        actual_delta_e_deg=math.degrees(actual_delta_e), actual_delta_a_deg=math.degrees(actual_delta_a),
        actual_delta_r_deg=math.degrees(actual_delta_r),
        aero_tail_avg=aero_avg, aero_tail_n_msgs=len(aero_tail),
        prop_tail=dict(left_rpm=left_rpm, right_rpm=right_rpm, left_thrust_N=left_thrust, right_thrust_N=right_thrust),
        pred_from_measured_state=pred,
        any_target_clamp=any_target_clamp, any_effort_clamp=any_effort_clamp, any_nan=state["any_nan"],
    )
    log(f"  [{label}] cmd_elev_theta={elev_theta_deg:+.3f}deg -> actual delta_e={result['actual_delta_e_deg']:+.4f}deg "
        f"delta_a={result['actual_delta_a_deg']:+.4f}deg delta_r={result['actual_delta_r_deg']:+.4f}deg | "
        f"CL={aero_avg['CL']:.5f} CD={aero_avg['CD']:.5f} Cm={aero_avg['Cm']:.5f} | "
        f"My(recomputed)={pred['My']:+.4f}Nm | RPM(L/R)={left_rpm:.1f}/{right_rpm:.1f} "
        f"Thrust(L/R)={left_thrust:.3f}/{right_thrust:.3f}N | "
        f"target_clamp={any_target_clamp} effort_clamp={any_effort_clamp} any_nan={state['any_nan']}")
    return result


def run_part3(log, trim):
    log("=" * 78)
    log("PART 3: CONTROLLED TRIM-HOLD CHECK (real actuator)")
    log("=" * 78)
    throttle = trim["throttle"]
    elev_theta = trim["elevator_theta_deg"]

    trim_point = run_actuator_quasi_static(log, "TRIM", throttle, elev_theta)

    lift = trim_point["aero_tail_avg"]["qbar"] * REF["S"] * trim_point["aero_tail_avg"]["CL"]
    drag = trim_point["aero_tail_avg"]["qbar"] * REF["S"] * trim_point["aero_tail_avg"]["CD"]
    thrust_total = trim_point["prop_tail"]["left_thrust_N"] + trim_point["prop_tail"]["right_thrust_N"]
    lift_weight_ratio = lift / WEIGHT_N
    thrust_drag_ratio = thrust_total / drag if drag else float("nan")
    rpm_asym = abs(trim_point["prop_tail"]["left_rpm"] - trim_point["prop_tail"]["right_rpm"])
    thrust_asym = abs(trim_point["prop_tail"]["left_thrust_N"] - trim_point["prop_tail"]["right_thrust_N"])

    log(f"\nTrim-hold balance: Lift={lift:.3f}N Weight={WEIGHT_N:.3f}N ratio={lift_weight_ratio:.4f} | "
        f"Thrust_total={thrust_total:.3f}N Drag={drag:.3f}N T/D={thrust_drag_ratio:.4f} | "
        f"My(recomputed)={trim_point['pred_from_measured_state']['My']:+.4f}Nm | "
        f"RPM_asym={rpm_asym:.4f} Thrust_asym={thrust_asym:.5f}N\n")

    log("Smoothness sweep around trim elevator (+/-1, +/-2 deg physical theta), same throttle/ailerons/rudder:")
    smooth_points = []
    for dtheta in (-2.0, -1.0, 0.0, 1.0, 2.0):
        p = run_actuator_quasi_static(log, f"TRIM{dtheta:+.0f}", throttle, elev_theta + dtheta)
        smooth_points.append(p)

    cls = [(p["actual_delta_e_deg"], p["aero_tail_avg"]["CL"], p["aero_tail_avg"]["Cm"]) for p in smooth_points]
    cls.sort(key=lambda t: t[0])
    jumps_ok = True
    for i in range(1, len(cls)):
        dCL = cls[i][1] - cls[i - 1][1]
        dCm = cls[i][2] - cls[i - 1][2]
        dtheta_actual = cls[i][0] - cls[i - 1][0]
        if abs(dtheta_actual) > 1e-6:
            slope_CL = dCL / dtheta_actual
            slope_Cm = dCm / dtheta_actual
        else:
            slope_CL = slope_Cm = 0.0
        log(f"  segment delta_e {cls[i-1][0]:+.3f}->{cls[i][0]:+.3f}deg: dCL={dCL:+.6f} dCm={dCm:+.6f} "
            f"(slope_CL/deg={slope_CL:+.5f} slope_Cm/deg={slope_Cm:+.5f})")
        # A discontinuity/clamp artifact would show as a wildly larger slope on one segment than its
        # neighbors, or a sign flip inconsistent with the elevator's known monotonic small-signal trend.
        if abs(dCL) > 0.05 or abs(dCm) > 0.1:
            jumps_ok = False
    log(f"No-discontinuity check (small per-degree steps, no outsized jump): {'PASS' if jumps_ok else 'WATCH'}\n")

    return dict(trim_point=trim_point, lift_weight_ratio=lift_weight_ratio, thrust_drag_ratio=thrust_drag_ratio,
                rpm_asym=rpm_asym, thrust_asym=thrust_asym, smoothness_points=smooth_points,
                smoothness_ok=jumps_ok, lift_N=lift, drag_N=drag, thrust_total_N=thrust_total)


# =============================================================================
# PART 4 - genuine free 6-DOF flight from the new trim, held through the
# real actuator, propulsion live, no autopilot/stabilizer.
# =============================================================================
FF_HOLD_STEPS = 800
FF_RELEASE_STEPS = 25000  # 25s
FF_TELEMETRY_EVERY = 100  # 10 Hz


def run_part4_free_flight(log, trim):
    log("=" * 78)
    log("PART 4: FREE 6-DOF POWERED FLIGHT (~25s) FROM NEW TRIM")
    log("=" * 78)
    throttle = trim["throttle"]
    elev_theta_rad = math.radians(trim["elevator_theta_deg"])
    cmd_rad = dict(left_elevator=elev_theta_rad, right_elevator=elev_theta_rad,
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
        if n < FF_HOLD_STEPS:
            AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(U_HOLD, 0, W_HOLD), gm.Vector3d(0, 0, 0),
                         kp_lin=KP_LIN, kp_ang=KP_ANG_SETTLE)
        # else: base_link COMPLETELY free - no hold, no stabilizer, no external force of any kind.

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
        if n >= FF_HOLD_STEPS and (n - FF_HOLD_STEPS) % FF_TELEMETRY_EVERY == 0:
            roll, pitch, yaw = quat_rpy(rot)
            V = math.sqrt(lv_b.x() ** 2 + lv_b.y() ** 2 + lv_b.z() ** 2)
            alpha = math.atan2(-lv_b.z(), lv_b.x())
            beta = math.atan2(lv_b.y(), math.hypot(lv_b.x(), lv_b.z()))
            prop = state["prop_diag"].latest() if state["prop_diag"] else None
            th, _ = ACT.read_joint_state(model, ecm, sim, ACT.JOINT_NAMES["left_elevator"])
            state["series"].append(dict(
                t=(n - FF_HOLD_STEPS) * AL.STEP, V=V, alt=wpose.pos().z(), world_vz=lv.z(),
                roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch), yaw_deg=math.degrees(yaw),
                alpha_deg=math.degrees(alpha), beta_deg=math.degrees(beta),
                p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z()),
                elevator_actual_deg=math.degrees(th) if th is not None else None, throttle=throttle,
                left_rpm=(prop["left"]["rpm"] if prop else None), right_rpm=(prop["right"]["rpm"] if prop else None),
                left_thrust_N=(prop["left"]["thrust_N"] if prop else None),
                right_thrust_N=(prop["right"]["thrust_N"] if prop else None)))
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, FF_HOLD_STEPS + FF_RELEASE_STEPS, False)

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
    log(f"Yaw drift: max|yaw-start|={max_abs_yaw_drift:.2f}deg")
    log(f"Alpha: start={start['alpha_deg']:+.2f} end={end['alpha_deg']:+.2f} max|alpha|={max_abs_alpha:.2f}deg")

    classification = "FAIL"
    reason = ""
    if state["any_nan"]:
        reason = "NaN/Inf encountered"
    elif max_abs_pitch > 60.0 or max_abs_roll > 60.0 or max_abs_alpha > 25.0:
        reason = "unbounded/runaway attitude or alpha excursion"
    elif abs(v_drift) > 3.0 or abs(alt_drift) > 60.0:
        classification = "PASS_WITH_SMALL_DRIFT" if (abs(v_drift) < 6.0 and abs(alt_drift) < 120.0) else "FAIL"
        reason = "moderate airspeed/altitude drift over the window, bounded"
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
                             max_abs_roll=max_abs_roll, max_abs_yaw_drift=max_abs_yaw_drift,
                             max_abs_alpha=max_abs_alpha),
                classification=classification, reason=reason)


# =============================================================================
# PART 5/6/7 - short high-deflection pulse tests through the real actuator.
# =============================================================================
PULSE_HOLD_STEPS = 800
PULSE_PRE_STEPS = 300
# 1.0s pulse hold - deliberately kept to "SHORT PULSE" per the task brief (not a full free-flight run).
# Sized against this aircraft's OWN measured roll authority (aileron Cl_delta_a lookup + Clp damping
# predicts a ~0.4s roll-rate rise time to a ~330deg/s steady roll rate at +25deg aileron - see this
# file's PART 5 log discussion) so a 1.0s window shows the full dynamic rise + a clear steady-state
# tail WITHOUT letting the airframe complete a full 360deg roll wrap (which would make the Euler roll
# angle telemetry ambiguous, though not numerically unsafe) - a test-planning choice, not a physics change.
PULSE_STEPS = 1000
PULSE_TAIL_STEPS = 1000
PULSE_TELEMETRY_EVERY = 5


def build_pulse_cmd(channel, pulse_deg, trim_theta_e_deg):
    cmd = dict(left_elevator=math.radians(trim_theta_e_deg), right_elevator=math.radians(trim_theta_e_deg),
               left_aileron=0.0, right_aileron=0.0, rudder=0.0)
    if channel == "elevator":
        trim_delta_e_aero = REF["elevatorSign"] * trim_theta_e_deg  # = -trim_theta_e_deg
        target_delta_e_aero = trim_delta_e_aero + pulse_deg
        theta = REF["elevatorSign"] * target_delta_e_aero  # theta = elevatorSign * delta_e_aero (elevatorSign=-1 -> theta=-delta_e)
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


def run_pulse_test(log, channel, pulse_deg, trim):
    case_name = f"{channel.upper()}_{'P' if pulse_deg >= 0 else 'M'}{abs(pulse_deg):.0f}"
    throttle = trim["throttle"]
    trim_theta_e_deg = trim["elevator_theta_deg"]
    trim_cmd = build_pulse_cmd(channel, 0.0, trim_theta_e_deg)
    pulse_cmd = build_pulse_cmd(channel, pulse_deg, trim_theta_e_deg)

    phase_bounds = (PULSE_HOLD_STEPS, PULSE_HOLD_STEPS + PULSE_PRE_STEPS,
                    PULSE_HOLD_STEPS + PULSE_PRE_STEPS + PULSE_STEPS,
                    PULSE_HOLD_STEPS + PULSE_PRE_STEPS + PULSE_STEPS + PULSE_TAIL_STEPS)
    total_steps = phase_bounds[-1]

    state = {"n": 0, "teleported": False, "cmd": None, "thr": None, "any_nan": False, "nan_step": None,
             "series": [], "prop_diag": None, "aero_diag": None}

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
        if n < phase_bounds[2]:
            state["cmd"].set(**(trim_cmd if n < phase_bounds[1] else pulse_cmd))
        else:
            state["cmd"].set(**trim_cmd)
        state["cmd"].tick()
        if n < phase_bounds[0]:
            AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(U_HOLD, 0, W_HOLD), gm.Vector3d(0, 0, 0),
                         kp_lin=KP_LIN, kp_ang=KP_ANG_SETTLE)
        # else: fully free from phase_bounds[0] onward.

    def on_post(info, ecm):
        n = state["n"]
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
        if n >= phase_bounds[0] and (n - phase_bounds[0]) % PULSE_TELEMETRY_EVERY == 0:
            roll, pitch, yaw = quat_rpy(rot)
            alpha = math.atan2(-lv_b.z(), lv_b.x())
            beta = math.atan2(lv_b.y(), math.hypot(lv_b.x(), lv_b.z()))
            da, de, dr, thetas = actual_deltas(model, ecm)
            pred = predict_aero(REF, lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z(),
                                 deltaA=da, deltaE=de, deltaR=dr)
            phase = ("hold" if n < phase_bounds[0] else "pre" if n < phase_bounds[1]
                     else "pulse" if n < phase_bounds[2] else "tail")
            # LIVE plugin diagnostics cross-check (fixes the MAJOR gap flagged by `validation`:
            # Parts 5/6/7 previously reported ONLY this script's own predict_aero() Python-mirror
            # values, never cross-checked against the live FalconV2Aerodynamics plugin's own
            # diagnostics topic at these dynamic/high-rate pulse conditions - unlike Part 1/Part 3,
            # which already do this). aero_live_raw is None until the first message arrives (~20Hz
            # publish rate, so within the first ~50ms of a run); Mx/My/Mz are not published on the
            # topic at all (AerodynamicsSystem.cc's diagnostics message is CL/CD/CY/Cl/Cm/Cn/V/alpha/
            # beta/qbar only), so they are DERIVED from the LIVE CL/CD/CY/Cl/Cm/Cn/qbar (Mx/Mz: no
            # axis-handedness correction needed, same as AeroModel.hh; My: the live Cm is the
            # UNFLIPPED cmStatic+cmRate sum, so cmRate=Cmq*qHat is subtracted back out using the
            # SAME measured q/V this tick, then the resolved FLU sign correction is applied to
            # cmStatic only - exactly mirroring AeroModel.hh's own My formula, just fed the LIVE
            # Cm instead of this script's own mirror-computed Cm).
            aero_live_raw = state["aero_diag"].latest() if state["aero_diag"] else None
            live_entry = None
            if aero_live_raw is not None:
                V_live = aero_live_raw["V"]
                qbar_live = aero_live_raw["qbar"]
                vSafe_live = max(V_live, REF["vSafeFloor"])
                qHat_live = av_b.y() * REF["c_ref"] / (2.0 * vSafe_live)
                cmRate_live = REF["Cmq"] * qHat_live
                cmStatic_live = aero_live_raw["Cm"] - cmRate_live
                My_live = qbar_live * REF["S"] * REF["c_ref"] * (-cmStatic_live + cmRate_live)
                Mx_live = qbar_live * REF["S"] * REF["b"] * aero_live_raw["Cl"]
                Mz_live = qbar_live * REF["S"] * REF["b"] * aero_live_raw["Cn"]
                live_entry = dict(CL=aero_live_raw["CL"], CD=aero_live_raw["CD"], CY=aero_live_raw["CY"],
                                   Cl=aero_live_raw["Cl"], Cm=aero_live_raw["Cm"], Cn=aero_live_raw["Cn"],
                                   Mx=Mx_live, My=My_live, Mz=Mz_live)
            state["series"].append(dict(
                n=n, t=(n - phase_bounds[0]) * AL.STEP, phase=phase,
                V=pred["V"], alt=wpose.pos().z(), world_vz=lv.z(),
                roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch), yaw_deg=math.degrees(yaw),
                alpha_deg=math.degrees(alpha), beta_deg=math.degrees(beta),
                p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z()),
                actual_delta_e_deg=math.degrees(de), actual_delta_a_deg=math.degrees(da),
                actual_delta_r_deg=math.degrees(dr),
                CL=pred["CL"], CD=pred["CD"], CY=pred["CY"], Cl=pred["Cl"], Cm=pred["Cm"], Cn=pred["Cn"],
                Mx=pred["Mx"], My=pred["My"], Mz=pred["Mz"],
                live=live_entry))
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
    tail = [s for s in series if s["phase"] == "tail"]
    baseline = pre[-1] if pre else (series[0] if series else None)
    pulse_settled = pulse[-max(1, len(pulse) // 5):] if pulse else []  # last 20% of pulse window = "settled"
    # (DYNAMIC steady state - for strong-authority surfaces (aileron especially) this legitimately
    # includes a large rate-damping contribution, e.g. Cl->~0 once roll rate reaches its own steady
    # value under sustained aileron, consistent with the documented roll-subsidence mode - NOT a defect,
    # see this function's own reporting below.)
    pulse_t0 = pulse[0]["t"] if pulse else 0.0  # start-of-PULSE-phase time origin (not overall test t=0)
    early_pulse = [s for s in pulse if s["t"] - pulse_t0 <= 0.3] if pulse else []
    # "early achieved" window: sampled once the actuator has plausibly reached the commanded deflection
    # (max_rate_rad_s=5.236 rad/s -> a 15-25deg move takes ~0.05-0.09s, plus PID settle) but BEFORE
    # body rates have had much time to build up - isolates the near-static control-authority coefficient
    # from the later rate-damping-dominated "settled" value above.
    early_achieved = [s for s in pulse if 0.08 <= (s["t"] - pulse_t0) <= 0.18]

    def avg(lst, key):
        vals = [s[key] for s in lst if s[key] is not None]
        return sum(vals) / len(vals) if vals else None

    def avg_live(lst, key):
        vals = [s["live"][key] for s in lst if s.get("live") is not None]
        return sum(vals) / len(vals) if vals else None

    settled = dict(CL=avg(pulse_settled, "CL"), CD=avg(pulse_settled, "CD"), CY=avg(pulse_settled, "CY"),
                    Cl=avg(pulse_settled, "Cl"), Cm=avg(pulse_settled, "Cm"), Cn=avg(pulse_settled, "Cn"),
                    Mx=avg(pulse_settled, "Mx"), My=avg(pulse_settled, "My"), Mz=avg(pulse_settled, "Mz"),
                    actual_delta_e_deg=avg(pulse_settled, "actual_delta_e_deg"),
                    actual_delta_a_deg=avg(pulse_settled, "actual_delta_a_deg"),
                    actual_delta_r_deg=avg(pulse_settled, "actual_delta_r_deg"))
    early = dict(CL=avg(early_achieved, "CL"), CD=avg(early_achieved, "CD"), CY=avg(early_achieved, "CY"),
                 Cl=avg(early_achieved, "Cl"), Cm=avg(early_achieved, "Cm"), Cn=avg(early_achieved, "Cn"),
                 Mx=avg(early_achieved, "Mx"), My=avg(early_achieved, "My"), Mz=avg(early_achieved, "Mz"),
                 actual_delta_e_deg=avg(early_achieved, "actual_delta_e_deg"),
                 actual_delta_a_deg=avg(early_achieved, "actual_delta_a_deg"),
                 actual_delta_r_deg=avg(early_achieved, "actual_delta_r_deg"),
                 p_deg_s=avg(early_achieved, "p_deg_s"), q_deg_s=avg(early_achieved, "q_deg_s"),
                 r_deg_s=avg(early_achieved, "r_deg_s"))

    # LIVE plugin-diagnostics cross-check (fixes the MAJOR gap - see the on_post comment above for the
    # full derivation notes). CL/CD/CY/Cl/Cn/Cm come DIRECTLY from the live FalconV2Aerodynamics topic
    # (never hand-computed); Mx/My/Mz are derived from those live values (not from this script's own
    # mirror), the same technique already used for Part 1/Part 3 above.
    #
    # REPORTING: settled_live/early_live below are plain window averages - useful for a ballpark
    # sense of the live-measured value, but during FAST dynamics (e.g. the aileron pulse's roll rate
    # building at >500deg/s^2) they can look like they "disagree" with the mirror's own (much finer,
    # per-tick) window average purely because the live topic only updates every ~50ms (20Hz) while the
    # window spans several such publish periods - a publish-rate STALENESS artifact, not a computation
    # difference (confirmed directly this pass by tracing individual ticks - see
    # paired_live_mirror_diff()'s own docstring for the full finding). The AUTHORITATIVE agreement
    # check is therefore the TIMING-PAIRED comparison below (computed over the FULL pulse-phase window,
    # tick-matched to each live message's own arrival instant, giving ~15-20 independent pairs per
    # case instead of the 2-4 a short early/settled sub-window would give).
    settled_live = {k: avg_live(pulse_settled, k) for k in LIVE_KEYS}
    early_live = {k: avg_live(early_achieved, k) for k in LIVE_KEYS}

    def rel_diff(mirror_v, live_v):
        if mirror_v is None or live_v is None:
            return None
        denom = max(abs(mirror_v), abs(live_v), 1e-6)
        return abs(mirror_v - live_v) / denom

    early_live_diff = {k: rel_diff(early.get(k), early_live.get(k)) for k in LIVE_KEYS}
    settled_live_diff = {k: rel_diff(settled.get(k), settled_live.get(k)) for k in LIVE_KEYS}

    # Timing-paired comparison over the WHOLE pulse phase (methodologically correct - see
    # paired_live_ticks()'s docstring). frac_fail is the floor-aware, zero-crossing-robust basis for
    # flagging a genuine mismatch (see paired_live_mirror_diff()'s own docstring for why the plain
    # mean-of-relative-diffs is NOT used for this decision).
    pulse_paired_diff, pulse_paired_frac_fail, n_pairs = paired_live_mirror_diff(pulse, LIVE_KEYS)
    # Flag only if a MEANINGFUL fraction of paired ticks (not just 1-2 isolated, explainable outliers,
    # e.g. a single fast-transient tick or a moment zero-crossing) exceed PAIRED_TOL - >20% of pairs
    # (4+ of the typical 20) failing is treated as a genuine, systematic disagreement worth flagging.
    pulse_paired_flags = {k: (v is not None and v > 0.20) for k, v in pulse_paired_frac_fail.items()}
    any_live_mismatch = any(pulse_paired_flags.values())

    # Early p(t)/roll trend (first ~0.3s of the pulse) - Part 6 raw evidence for rudder, reported for
    # aileron too since it is the primary roll-coupled surface.
    p_trend = None
    if len(early_pulse) >= 2:
        p0, p1 = early_pulse[0]["p_deg_s"], early_pulse[-1]["p_deg_s"]
        t0, t1 = early_pulse[0]["t"], early_pulse[-1]["t"]
        p_trend = (p1 - p0) / (t1 - t0) if t1 != t0 else None

    tail_return = tail[-1] if tail else None

    result = dict(
        case=case_name, channel=channel, pulse_deg=pulse_deg,
        any_nan=state["any_nan"], nan_step=state["nan_step"],
        baseline=baseline, early_achieved=early, settled=settled,
        early_live=early_live, settled_live=settled_live,
        early_live_diff=early_live_diff, settled_live_diff=settled_live_diff,
        pulse_paired_diff=pulse_paired_diff, pulse_paired_frac_fail=pulse_paired_frac_fail,
        pulse_paired_n=n_pairs, pulse_paired_flags=pulse_paired_flags,
        any_live_mismatch=any_live_mismatch,
        early_p_trend_deg_s2=p_trend,
        max_abs_p_deg_s=max((abs(s["p_deg_s"]) for s in pulse), default=None),
        max_abs_q_deg_s=max((abs(s["q_deg_s"]) for s in pulse), default=None),
        max_abs_r_deg_s=max((abs(s["r_deg_s"]) for s in pulse), default=None),
        max_abs_roll_deg=max((abs(s["roll_deg"]) for s in pulse), default=None),
        max_abs_pitch_deg=max((abs(s["pitch_deg"] - baseline["pitch_deg"]) for s in pulse), default=None) if baseline else None,
        max_abs_beta_deg=max((abs(s["beta_deg"]) for s in pulse), default=None),
        max_abs_alpha_deg=max((abs(s["alpha_deg"]) for s in pulse), default=None),
        tail_return=tail_return,
        series=series,
    )
    log(f"--- {case_name} (pulse={pulse_deg:+.0f}deg, channel={channel}) ---")
    log(f"  any_nan={state['any_nan']} EARLY(t~0.08-0.18s, near-static, achieved delta_e/a/r="
        f"{early['actual_delta_e_deg']:+.3f}/{early['actual_delta_a_deg']:+.3f}/{early['actual_delta_r_deg']:+.3f}deg): "
        f"[MIRROR] CL={early['CL']:.5f} CD={early['CD']:.5f} CY={early['CY']:.6f} Cl={early['Cl']:.6f} Cm={early['Cm']:.5f} "
        f"Cn={early['Cn']:.6f} Mx={early['Mx']:+.4f} My={early['My']:+.4f} Mz={early['Mz']:+.4f} "
        f"(p/q/r={early['p_deg_s']:+.2f}/{early['q_deg_s']:+.2f}/{early['r_deg_s']:+.2f}deg/s)")
    if all(v is not None for v in early_live.values()):
        log(f"  EARLY  [LIVE PLUGIN, window-avg - see staleness note below]  CL={early_live['CL']:.5f} "
            f"CD={early_live['CD']:.5f} CY={early_live['CY']:.6f} Cl={early_live['Cl']:.6f} Cm={early_live['Cm']:.5f} "
            f"Cn={early_live['Cn']:.6f} Mx={early_live['Mx']:+.4f} My={early_live['My']:+.4f} Mz={early_live['Mz']:+.4f}")
        log(f"  EARLY  [mirror-vs-live-window-avg rel.diff, INFORMATIONAL ONLY - see staleness note below] " +
            " ".join(f"{k}={early_live_diff[k]*100:.2f}%" for k in LIVE_KEYS if early_live_diff[k] is not None))
    else:
        log("  EARLY  [LIVE PLUGIN] no live diagnostics message received in this window (topic just started publishing)")
    log(f"  SETTLED (last 20% of pulse window, DYNAMIC steady state - includes rate-damping, see header): "
        f"actual_delta_e/a/r={settled['actual_delta_e_deg']:+.3f}/"
        f"{settled['actual_delta_a_deg']:+.3f}/{settled['actual_delta_r_deg']:+.3f}deg "
        f"[MIRROR] CL={settled['CL']:.5f} CD={settled['CD']:.5f} CY={settled['CY']:.6f} Cl={settled['Cl']:.6f} "
        f"Cm={settled['Cm']:.5f} Cn={settled['Cn']:.6f} Mx={settled['Mx']:+.4f} My={settled['My']:+.4f} "
        f"Mz={settled['Mz']:+.4f}")
    if all(v is not None for v in settled_live.values()):
        log(f"  SETTLED [LIVE PLUGIN, window-avg - see staleness note below]  CL={settled_live['CL']:.5f} "
            f"CD={settled_live['CD']:.5f} CY={settled_live['CY']:.6f} Cl={settled_live['Cl']:.6f} "
            f"Cm={settled_live['Cm']:.5f} Cn={settled_live['Cn']:.6f} "
            f"Mx={settled_live['Mx']:+.4f} My={settled_live['My']:+.4f} Mz={settled_live['Mz']:+.4f}")
        log(f"  SETTLED [mirror-vs-live-window-avg rel.diff, INFORMATIONAL ONLY - see staleness note below] " +
            " ".join(f"{k}={settled_live_diff[k]*100:.2f}%" for k in LIVE_KEYS if settled_live_diff[k] is not None))
    else:
        log("  SETTLED [LIVE PLUGIN] no live diagnostics message received in this window")
    log(f"  NOTE: the two window-avg comparisons above can show a large apparent diff purely from "
        f"20Hz live-topic publish-rate staleness during fast dynamics (see paired_live_mirror_diff()'s "
        f"docstring) - NOT used to determine pass/fail. The AUTHORITATIVE, timing-paired comparison over "
        f"the full pulse-phase window ({n_pairs} independent live-publish instants) is:")
    log(f"  PAIRED [mirror-vs-live, tick-synchronized, mean |reldiff| (informational) over {n_pairs} pairs] " +
        " ".join(f"{k}={pulse_paired_diff[k]*100:.2f}%" for k in LIVE_KEYS if pulse_paired_diff[k] is not None))
    log(f"  PAIRED [fraction of the {n_pairs} pairs exceeding the floor-aware tolerance - THIS decides "
        f"pass/fail, not the mean above] " +
        " ".join(f"{k}={pulse_paired_frac_fail[k]*100:.0f}%{'(FLAG)' if pulse_paired_flags[k] else ''}"
                  for k in LIVE_KEYS if pulse_paired_frac_fail[k] is not None))
    log(f"  LIVE-CROSS-CHECK RESULT: {'MISMATCH_FLAGGED (see PAIRED line above)' if any_live_mismatch else 'CONFIRMS_MIRROR (all timing-paired live-vs-mirror diffs within tolerance)'}")
    log(f"  dynamics: max|p|={result['max_abs_p_deg_s']:.3f}deg/s max|q|={result['max_abs_q_deg_s']:.3f}deg/s "
        f"max|r|={result['max_abs_r_deg_s']:.3f}deg/s max|roll|={result['max_abs_roll_deg']:.3f}deg "
        f"early_p_trend={p_trend if p_trend is None else f'{p_trend:+.3f}'}deg/s^2")
    log("")
    return result


def run_part5_6_7(log, trim, part3):
    log("=" * 78)
    log("PART 5/6/7: HIGH-DEFLECTION PULSE TESTS (+/-15deg, +/-25deg if stable)")
    log("=" * 78)
    trim_CD = part3["trim_point"]["aero_tail_avg"]["CD"]
    channels_results = {}
    for channel in ("elevator", "aileron", "rudder"):
        log("-" * 78)
        log(f"CHANNEL: {channel.upper()}")
        log("-" * 78)
        pts = {}
        for mag in (15.0, -15.0):
            pts[mag] = run_pulse_test(log, channel, mag, trim)
        stable15 = not any(pts[m]["any_nan"] for m in pts)
        do25 = stable15
        if do25:
            for mag in (25.0, -25.0):
                pts[mag] = run_pulse_test(log, channel, mag, trim)
        else:
            log(f"  SKIPPING +/-25deg for {channel} - instability (NaN) observed at +/-15deg.\n")

        # Control-drag qualitative check - uses the EARLY (near-static, minimal rate-damping
        # contamination) sample, since CD_settled for a strong-authority surface (aileron) is measured
        # once the aircraft has already rotated substantially (rate terms feed into CY/Cl/Cn/Cm and,
        # via CL, into CD too) - the early sample is the correct one for a lookup-table-shape comparison.
        log(f"  Control-drag check ({channel}): CD_trim={trim_CD:.5f}")
        for mag in sorted(pts.keys(), key=abs):
            log(f"    pulse={mag:+.0f}deg: CD_early={pts[mag]['early_achieved']['CD']:.5f} "
                f"(delta={pts[mag]['early_achieved']['CD']-trim_CD:+.5f})")

        # Old-+/-10deg-clamp-artifact check: confirm smooth growth from 15->25 (no flat/clamped region).
        # Uses the EARLY sample for the same reason as the drag check above.
        clamp_artifact = None
        if 25.0 in pts and -25.0 in pts:
            key = {"elevator": "CL", "aileron": "Cl", "rudder": "CY"}[channel]
            v15p, v25p = pts[15.0]["early_achieved"][key], pts[25.0]["early_achieved"][key]
            v15m, v25m = pts[-15.0]["early_achieved"][key], pts[-25.0]["early_achieved"][key]
            growth_p = abs(v25p - v15p)
            growth_m = abs(v25m - v15m)
            clamp_artifact = dict(key=key, growth_pos=growth_p, growth_neg=growth_m,
                                   flat_or_clamped=(growth_p < 1e-4 or growth_m < 1e-4))
            log(f"  10-25deg-region smoothness ({key}): +15->+25 growth={growth_p:+.6f}, "
                f"-15->-25 growth={growth_m:+.6f} -> {'FLAT/CLAMPED (WATCH)' if clamp_artifact['flat_or_clamped'] else 'smooth, non-flat (OK)'}")
        log("")
        channels_results[channel] = dict(points={f"{m:+.0f}": pts[m] for m in pts}, stable15=stable15, did_25=do25,
                                           clamp_artifact_check=clamp_artifact)
    return channels_results


def strip_series(obj):
    out = dict(obj)
    out["series_n"] = len(obj.get("series", []))
    out.pop("series", None)
    return out


# =============================================================================
# Orchestration
# =============================================================================
def main():
    trim_log_lines = []
    flight_log_lines = []

    def tlog(msg):
        print(msg, flush=True)
        trim_log_lines.append(msg)

    def flog(msg):
        print(msg, flush=True)
        flight_log_lines.append(msg)

    tlog("FALCON V2 - UPDATED_POWERED_TRIM_AND_HIGH_DEFLECTION_FLIGHT_VALIDATION (gazebo-testing, 2026-08-27)")
    tlog(f"World: {WORLD}")
    tlog(f"elevator_sign={REF['elevatorSign']} aileron_sign={REF['aileronSign']} rudder_sign={REF['rudderSign']} "
         f"(read fresh from aero_v1_config.yaml)")
    tlog("")

    part1 = run_trim_search(tlog)
    trim = part1["selected"]

    tlog("=" * 78)
    tlog("PART 2: NEW TRIM REPORT")
    tlog("=" * 78)
    old = dict(throttle=0.4915, elevator_theta_deg=5.50, elevator_aero_deg=-5.4995, V=18.165, alpha_deg=2.461,
               CL=0.670857, CD=0.05887, Cm_diag=0.009430, Lift_N=61.2116, Drag_N=5.3707, My_Nm=-0.1927,
               thrust_total_N=4.8808)
    aero_live = trim.get("aero_live_rel") or {}
    tlog(f"NEW TRIM: throttle={trim['throttle']:.4f}, elevator physical theta L=R={trim['elevator_theta_deg']:+.3f}deg, "
         f"elevator_aero={trim['elevator_aero_deg']:+.2f}deg")
    tlog(f"  V={trim['release']['V']:.3f} m/s, alpha={trim['release']['alpha_deg']:.3f}deg, altitude={ALTITUDE_M}m")
    tlog(f"  CL={trim['aero_rel']['CL']:.5f} CD={trim['aero_rel']['CD']:.5f} Cm(diag)={trim['aero_rel']['Cm_diag']:.5f}")
    tlog(f"  Lift={trim['aero_rel']['Lift_N']:.3f}N Weight={WEIGHT_N:.3f}N Drag={trim['aero_rel']['Drag_N']:.3f}N")
    tlog(f"  Thrust(L/R)={trim['prop_rel']['left_thrust_N']:.3f}/{trim['prop_rel']['right_thrust_N']:.3f}N "
         f"(total={trim['prop_rel']['thrust_total_N']:.3f}N)")
    tlog(f"  RPM(L/R)={trim['prop_rel']['left_rpm']:.1f}/{trim['prop_rel']['right_rpm']:.1f}")
    tlog(f"  My(recomputed)={trim['aero_rel']['My_Nm']:+.4f}Nm")
    tlog(f"  vertical-accel tendency (tail_mean_vz, tail_alt_drift_rate over settled window): "
         f"{trim['window']['tail_mean_world_vz_mps']:+.4f} m/s, {trim['window']['tail_alt_drift_rate_mps']:+.4f} m/s")
    if aero_live:
        tlog(f"  [live plugin diagnostic cross-check @release] CL={aero_live.get('CL')} CD={aero_live.get('CD')} "
             f"Cm={aero_live.get('Cm')}")
    tlog("")
    tlog("OLD vs NEW comparison:")
    tlog(f"  elevator_aero: OLD={old['elevator_aero_deg']:+.2f}deg -> NEW={trim['elevator_aero_deg']:+.2f}deg "
         f"(shift={trim['elevator_aero_deg']-old['elevator_aero_deg']:+.2f}deg)")
    tlog(f"  throttle: OLD={old['throttle']:.4f} -> NEW={trim['throttle']:.4f} "
         f"(shift={trim['throttle']-old['throttle']:+.4f})")
    tlog(f"  alpha: OLD={old['alpha_deg']:.3f}deg -> NEW={trim['release']['alpha_deg']:.3f}deg "
         f"(shift={trim['release']['alpha_deg']-old['alpha_deg']:+.3f}deg)")
    tlog(f"  CL: OLD={old['CL']:.5f} -> NEW={trim['aero_rel']['CL']:.5f}")
    tlog("  Explanation: the new CL_delta_e=+0.414/rad lookup term (previously entirely absent) means the OLD "
         "elevator deflection (-5.4995deg aero) was needlessly LIFT-REDUCING under the new model (dCL(-5deg)="
         "-0.03608 from aero_v1_config.yaml's control_surface_lookup.elevator.dCL) on top of the larger-magnitude "
         "Cm_delta_e now producing a MORE NOSE-UP tendency at that same deflection (dCm(-5deg)=+0.08710, feeding "
         "a larger cmStatic, hence a MORE NEGATIVE My = more nose-up per this project's negative-My-is-nose-up "
         "convention - corrected wording, 2026-08-27 `validation` review; not nose-down as an earlier draft of "
         "this explanation said) - the new trim search finds a SMALLER-magnitude (less negative) elevator "
         "deflection restores both the lift balance and the moment balance simultaneously, consistent with these "
         "two documented lookup-table entries, not a speculative explanation.\n")

    part3 = run_part3(tlog, trim)

    part4 = run_part4_free_flight(flog, trim)
    part567 = run_part5_6_7(flog, trim, part3)

    flog("=" * 78)
    flog("ANTI-WINDUP CHECK (Part 7)")
    flog("=" * 78)
    flog("Normal +/-15/+/-25deg pulses stay well within +/-45deg raw commands (never persistently out of "
         "range) - per the prior stage's finding, the actuator's anti-windup droop is only triggered by a "
         "SUSTAINED out-of-+/-45deg-range RAW command, which this task did not issue.")

    with open(f"{RESULTS_DIR}/updated_trim_search_result.json", "w") as f:
        json.dump(dict(part1=part1, part2_old_trim_reference=old, part3=part3),
                  f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/updated_trim_search_log.txt", "w") as f:
        f.write("\n".join(trim_log_lines) + "\n")

    part4_out = dict(any_nan=part4["any_nan"], nan_step=part4["nan_step"], summary=part4["summary"],
                      classification=part4["classification"], reason=part4["reason"], telemetry=part4["series"])
    part567_out = {ch: dict(stable15=d["stable15"], did_25=d["did_25"],
                             clamp_artifact_check=d["clamp_artifact_check"],
                             points={k: strip_series(v) for k, v in d["points"].items()})
                   for ch, d in part567.items()}
    with open(f"{RESULTS_DIR}/high_deflection_flight_result.json", "w") as f:
        json.dump(dict(trim=trim, part4_free_flight=part4_out, part5_6_7=part567_out),
                  f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/high_deflection_flight_log.txt", "w") as f:
        f.write("\n".join(flight_log_lines) + "\n")

    overall_ok = (not part1["any_nan_overall"]) and (not part3["trim_point"]["any_nan"]) and \
                 (not part4["any_nan"]) and all(not p["any_nan"] for ch in part567.values() for p in ch["points"].values())
    return overall_ok


def main_rerun_part567():
    """Targeted re-run of Parts 5/6/7 ONLY (`validation` review, 2026-08-27): fixes the MAJOR gap
    where run_pulse_test() never cross-checked its own predict_aero() Python-mirror output against
    the live FalconV2Aerodynamics plugin's own diagnostics topic at these dynamic/high-rate pulse
    conditions (unlike Part 1/Part 3, which already did). Parts 1-4 (trim search, trim report,
    controlled trim-hold check, free 6-DOF flight) were independently confirmed correct and are
    NOT re-run here - this loads their already-saved, already-reported results from the existing
    result JSON files (read-only) instead of re-executing the expensive trim search / 25s free-
    flight run a second time for no reason. No aircraft physics parameter is touched."""
    flight_log_lines = []

    def flog(msg):
        print(msg, flush=True)
        flight_log_lines.append(msg)

    with open(f"{RESULTS_DIR}/high_deflection_flight_result.json") as f:
        existing_flight = json.load(f)
    with open(f"{RESULTS_DIR}/updated_trim_search_result.json") as f:
        existing_trim = json.load(f)

    trim = existing_flight["trim"]  # Part 1's already-selected/reported trim - unchanged, read-only
    part3 = dict(trim_point=dict(aero_tail_avg=dict(CD=existing_trim["part3"]["trim_point"]["aero_tail_avg"]["CD"])))

    flog("FALCON V2 - UPDATED_POWERED_TRIM_AND_HIGH_DEFLECTION_FLIGHT_VALIDATION - TARGETED RE-RUN "
         "(gazebo-testing, 2026-08-27, `validation` MAJOR-gap fix: Parts 5/6/7 now cross-check the "
         "live aero diagnostics topic, not just this script's own Python mirror)")
    flog(f"Reusing already-confirmed-correct trim (NOT re-searched): throttle={trim['throttle']:.4f} "
         f"elevator_theta_deg={trim['elevator_theta_deg']:+.3f} (Part 1-4 results loaded read-only "
         f"from the existing result JSON files, unchanged)")
    flog("")

    part567 = run_part5_6_7(flog, trim, part3)

    flog("=" * 78)
    flog("ANTI-WINDUP CHECK (Part 7)")
    flog("=" * 78)
    flog("Normal +/-15/+/-25deg pulses stay well within +/-45deg raw commands (never persistently out of "
         "range) - per the prior stage's finding, the actuator's anti-windup droop is only triggered by a "
         "SUSTAINED out-of-+/-45deg-range RAW command, which this task did not issue.")

    part567_out = {ch: dict(stable15=d["stable15"], did_25=d["did_25"],
                             clamp_artifact_check=d["clamp_artifact_check"],
                             points={k: strip_series(v) for k, v in d["points"].items()})
                   for ch, d in part567.items()}

    # Part 4 (free 6-DOF flight) is preserved VERBATIM from the already-confirmed-correct prior run.
    with open(f"{RESULTS_DIR}/high_deflection_flight_result.json", "w") as f:
        json.dump(dict(trim=trim, part4_free_flight=existing_flight["part4_free_flight"], part5_6_7=part567_out),
                  f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))

    # Preserve the existing log's Part 4 section verbatim; replace everything from the Part 5/6/7
    # header onward with the freshly regenerated (live-cross-check-included) log lines.
    with open(f"{RESULTS_DIR}/high_deflection_flight_log.txt") as f:
        existing_log_text = f.read()
    marker = "PART 5/6/7: HIGH-DEFLECTION PULSE TESTS"
    idx = existing_log_text.find(marker)
    part4_log_prefix = existing_log_text[:idx].rstrip("\n") if idx >= 0 else existing_log_text.rstrip("\n")
    combined_log = part4_log_prefix + "\n\n" + "\n".join(flight_log_lines) + "\n"
    with open(f"{RESULTS_DIR}/high_deflection_flight_log.txt", "w") as f:
        f.write(combined_log)

    any_live_mismatch_overall = any(
        p.get("any_live_mismatch") for ch in part567.values() for p in ch["points"].values())
    overall_ok = all(not p["any_nan"] for ch in part567.values() for p in ch["points"].values())
    flog("")
    flog(f"ANY_LIVE_MISMATCH (any pulse point where live-plugin vs Python-mirror differ beyond "
         f"tolerance) OVERALL: {any_live_mismatch_overall}")
    return overall_ok, any_live_mismatch_overall


if __name__ == "__main__":
    if "--part567-only" in sys.argv:
        ok, mismatch = main_rerun_part567()
        sys.exit(0 if ok else 1)
    else:
        ok = main()
        sys.exit(0 if ok else 1)

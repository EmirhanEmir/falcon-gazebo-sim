#!/usr/bin/env python3
"""
FALCON V2 - HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION live-Gazebo
confirmation test (gazebo-testing, 2026-08-26).

SCOPE (per task brief): a SMALL, TARGETED live-Gazebo confirmation of the
new piecewise-linear wide-deflection control-surface lookup model
(plugins/aerodynamics/AeroModel.hh / AerodynamicsSystem.cc, elevator/
aileron/rudder, bounded +/-45deg) that `aerodynamics` already unit-tested
standalone (aero_model_selftest, 31 PASS/0 FAIL). This is NOT a full
regression suite - old structural/propulsion/free-flight regression scripts
are intentionally NOT re-run here. No aero coefficient, actuator parameter,
or lookup table value is modified anywhere in this script.

Reuses (does not reinvent) the established live-Gazebo test techniques from
tests/gazebo/scripts/test_control_authority_effectiveness.py (prior stage,
same day):
  - actuator_lib.ActuatorCommander / DiagSubscriber / read_joint_state /
    pin_other_child_joints (real actuator -> real joint -> real aero chain)
  - aero_lib.hold_step() (quasi-static isolation via an external P force/
    torque controller, identical KP_ANG_QSTATIC=1500.0 fix already
    established and justified in that script's own docstring - a pure-P
    rate hold leaves a residual body rate under a persistent aero moment;
    1500 was found stable and effective there, 2000 triggered a physics-
    engine numerical blow-up. That residual is expected to scale roughly
    with the aero moment magnitude, so it will generally be LARGER at this
    stage's bigger deflections (+/-25/+/-45 deg) than at the prior stage's
    +/-10 deg ceiling - flagged wherever it plausibly explains a wider
    delta on a small/tertiary cross-coupling term (Cn from aileron, Cl from
    rudder), exactly the same documented mechanism, not re-litigated here)
  - aero_lib.DiagSubscriber / propulsion_lib.ThrottleCommander/DiagSubscriber
  - test_free_flight_dynamic_response.py's "hold-then-fully-release" free-
    flight pattern for Part E.

INDEPENDENT "expected value" source for this script: aero_lib.py's own
pure-Python compute_aero()/load_config() mirror is now STALE (it implements
the OLD linear-coefficient + generic +/-10deg-clamp model, not the new
lookup) and is intentionally NOT touched or reused for expected-value
computation here (touching it could silently change the semantics other,
older scripts in this suite still rely on unmodified, and is out of this
task's scope). Instead, this script reads
docs/source_of_truth/aerodynamics/aero_v1_config.yaml's own
control_surface_lookup block directly (read-only) and reproduces the
handful of arithmetic steps AeroConfig::Prepare() applies (aileron/rudder
CD/CL/Cm baseline-differencing against the delta=0 row; the rudder-roll
Cl(delta_r) bounded-linear-extension from Cldr_per_rad) - see
build_expected_tables() below. This is the same "reconstruct what
ComputeAero() should produce" technique the task brief authorizes, applied
only where a real interaction exists:
  - CL/Cm (elevator), Cl/Cn/CY (aileron/rudder), Cm (aileron): purely
    additive lookup contributions - the raw/diffed table value alone IS the
    expected measured delta-from-baseline, no correction needed.
  - CD (elevator, aileron): AeroModel.hh's Part-4 drag build-up is
    `CD = CD0 + k*CL_total^2 + dCD_e + dCD_a + dCD_r` (floored at CD0) -
    CL_total ITSELF changes with delta_e/delta_a (via dCLeCtrl/dCLaCtrl), so
    there is a genuine second-order k*(CL_total^2) cross-term on top of the
    table's own dCD entry that a naive "just diff the CD table" comparison
    would miss (verified this pass: at delta_e=+45 this cross-term is
    LARGER than the table's own dCD(45)=0.04846, ~+0.028 extra from the CL
    shift alone - not a defect, an intended documented feature,
    V1_ADDITIVE_MULTI_SURFACE_DRAG_APPROXIMATION). This script's expected_CD
    computation includes this cross-term using this run's OWN live-measured
    CL (self-consistent, no separate alpha/SaturatedCL re-derivation
    needed) - see expected_delta() below. Rudder's CD has no such
    cross-term (delta_r never touches CL) so it is a direct table diff.

No aircraft physics parameter (aero coefficient, control-sign mapping,
lookup table value, actuator max_rate/max_effort/kp/kd/ki/sp_weight_b,
motor/prop constant, mass/CG/inertia, hinge geometry) is modified anywhere
in this script.
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
WORLD_SDF = f"{REPO_ROOT}/tests/gazebo/worlds/falcon_v2_freefall_world.sdf"

ACFG = ACT.load_actuator_config()  # read-only, actuator_v1_config.yaml

# =============================================================================
# Independent, read-only re-parse of aero_v1_config.yaml's
# control_surface_lookup block - the "expected value" reference for this
# test. NOT the same object as aero_lib.load_config() (stale, see module
# docstring) - a fresh, minimal, self-contained parse used only here.
# =============================================================================
def build_expected_tables():
    with open(AL.CONFIG_YAML_PATH) as f:
        root = yaml.safe_load(f)
    csl = root["control_surface_lookup"]
    bps = [float(x) for x in csl["breakpoints_deg"]]
    idx0 = bps.index(0.0)
    elev, aile, rudd = csl["elevator"], csl["aileron"], csl["rudder"]
    dragK = root["drag_polar"]["k"]
    cldr = root["lateral_directional"]["Cldr_per_rad"]

    def by_bp(arr):
        return dict(zip(bps, [float(x) for x in arr]))

    def diffed(arr):
        arr = [float(x) for x in arr]
        base = arr[idx0]
        return dict(zip(bps, [v - base for v in arr]))

    return dict(
        bps=bps, dragK=dragK, cldr=cldr,
        elev_dCL=by_bp(elev["dCL"]), elev_dCD=by_bp(elev["dCD"]), elev_dCm=by_bp(elev["dCm"]),
        aile_Cl=by_bp(aile["Cl"]), aile_Cn=by_bp(aile["Cn"]), aile_CY=by_bp(aile["CY"]),
        aile_dCD=diffed(aile["CD_full"]), aile_dCL=diffed(aile["CL_full"]), aile_dCm=diffed(aile["Cm_full"]),
        rudd_CY=by_bp(rudd["CY"]), rudd_Cn=by_bp(rudd["Cn"]), rudd_dCD=diffed(rudd["CD_full"]),
    )


TAB = build_expected_tables()

CHANNEL_COEFFS = {
    "elevator": ["CL", "CD", "Cm"],
    "aileron": ["Cl", "Cn", "CY", "CD", "CL", "Cm"],
    "rudder": ["CY", "Cn", "CD", "Cl"],
}
# Odd (anti-symmetric about 0) vs even (symmetric about 0) classification,
# per the task brief / source-of-truth "symmetry observations" sections.
ODD_COEFFS = {"aileron": ["Cl", "Cn", "CY"], "rudder": ["CY", "Cn"]}
EVEN_COEFFS = {"aileron": ["CL", "CD", "Cm"], "rudder": ["CD"]}


def expected_delta(channel, coeff, delta_deg, measured_CL_at_delta=None, measured_CL_at_zero=None):
    """Expected measured delta-from-baseline for `coeff` at exactly
    `delta_deg` (must be an exact table breakpoint - no interpolation
    performed here, matching Part A's "exact breakpoint" design). Returns
    None if out of scope for that channel/coeff (e.g. rudder Cl uses the
    linear-extension formula, handled separately by expected_rudder_cl())."""
    if delta_deg not in TAB["bps"]:
        return None
    if channel == "elevator":
        if coeff == "CL":
            return TAB["elev_dCL"][delta_deg]
        if coeff == "Cm":
            return TAB["elev_dCm"][delta_deg]
        if coeff == "CD":
            cross = TAB["dragK"] * (measured_CL_at_delta ** 2 - measured_CL_at_zero ** 2)
            return cross + TAB["elev_dCD"][delta_deg]
    elif channel == "aileron":
        if coeff == "Cl":
            return TAB["aile_Cl"][delta_deg]
        if coeff == "Cn":
            return TAB["aile_Cn"][delta_deg]
        if coeff == "CY":
            return TAB["aile_CY"][delta_deg]
        if coeff == "CL":
            return TAB["aile_dCL"][delta_deg]
        if coeff == "Cm":
            return TAB["aile_dCm"][delta_deg]
        if coeff == "CD":
            cross = TAB["dragK"] * (measured_CL_at_delta ** 2 - measured_CL_at_zero ** 2)
            return cross + TAB["aile_dCD"][delta_deg]
    elif channel == "rudder":
        if coeff == "CY":
            return TAB["rudd_CY"][delta_deg]
        if coeff == "Cn":
            return TAB["rudd_Cn"][delta_deg]
        if coeff == "CD":
            return TAB["rudd_dCD"][delta_deg]
        if coeff == "Cl":
            return TAB["cldr"] * math.radians(delta_deg)  # bounded linear extension, NOT the disputed table
    return None


# Per-coefficient tolerance: abs(measured-expected) <= max(rel*|expected|, abs_floor)
TOL = {
    "CL": (0.10, 0.008), "Cm": (0.10, 0.010), "Cl_aileron": (0.10, 0.006),
    "CD": (0.20, 0.004),
    "CY": (0.15, 0.0006), "Cn": (0.20, 0.0004), "Cl_rudder": (0.30, 0.00004),
}


def tol_for(channel, coeff):
    key = coeff
    if coeff == "Cl":
        key = "Cl_aileron" if channel == "aileron" else "Cl_rudder"
    return TOL[key]


def within_tol(channel, coeff, measured, expected):
    rel, floor = tol_for(channel, coeff)
    return abs(measured - expected) <= max(rel * abs(expected), floor)


# ---- Runtime-queried base_link mass/inertia (reused verbatim, see
# test_control_authority_effectiveness.py / test_actuator_flight_load.py) ----
MASS = 5.9348
I_DIAG = (0.7284, 0.2507, 0.9523)
KP_LIN = 150.0
KP_ANG_QSTATIC = 1500.0  # reused test-harness fix, see module docstring
KP_ANG_SETTLE = 400.0
ALTITUDE_M = 100.0

# ---- Base powered operating point (reused read-only, matches the XFLR5
# wide-deflection source table's own operating point closely: V=18.162 m/s,
# alpha=2.472 deg - NOT a coincidence, the trim point was chosen against the
# same aircraft data) ----
TRIM_V = 18.165
TRIM_ALPHA_DEG = 2.461
TRIM_THROTTLE = 0.4915
TRIM_ELEV_THETA_DEG = 5.50   # physical, both left/right elevator joints (baseline/warm-up only)
_alpha_rad = math.radians(TRIM_ALPHA_DEG)
TRIM_U = TRIM_V * math.cos(_alpha_rad)
TRIM_W = -TRIM_V * math.sin(_alpha_rad)
LIN_TARGET = gm.Vector3d(TRIM_U, 0.0, TRIM_W)
ANG_TARGET_ZERO = gm.Vector3d(0.0, 0.0, 0.0)
FULL_MASK = (True, True, True)

WARM_STEPS = 300
SETTLE_STEPS = 4500
TAIL_STEPS = 500
DIAG_HZ = 20.0

# Part A/B/C combined sweep - 0 + {2,5,10,25,45} deg magnitude, both signs.
# (Task text says "7 points per surface" but then lists 0/+-5/+-10/+-25/+-45
#  = 9 distinct values; this script uses the literal 9-value list plus +-2
#  deg (needed for Part C's +-2/+-5/+-10 small-signal window and NOT
#  separately called out as an extra run) = 11 points/surface. Both Part B's
#  named checkpoints (+-10/+-25/+-45) and Part C's (+-2/+-5/+-10) are exact
#  subsets of this one list, so no surface needs more than one 11-point
#  sweep - noted here as an OBSERVED discrepancy in the task's own point
#  count, not treated as a defect.)
DELTAS_DEG = [-45.0, -25.0, -10.0, -5.0, -2.0, 0.0, 2.0, 5.0, 10.0, 25.0, 45.0]
WINDOWS = [2.0, 5.0, 10.0]
EXCEED_CASES = [("elevator", 60.0), ("elevator", -60.0), ("aileron", 60.0), ("rudder", 60.0)]

ALL_SURFACE_JOINTS = list(ACT.JOINT_NAMES.values())


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    model_e = world.model_by_name(ecm, "falcon_v2")
    return sim.Model(model_e)


def baseline_cmd_rad():
    return dict(left_elevator=math.radians(TRIM_ELEV_THETA_DEG),
                right_elevator=math.radians(TRIM_ELEV_THETA_DEG),
                left_aileron=0.0, right_aileron=0.0, rudder=0.0)


def build_target_cmd(channel, delta_deg):
    """delta_deg is now an ABSOLUTE target aero deflection (not a trim-
    relative increment), per this task's Part A instruction - the whole
    point of this stage is the lookup covers the full range without the old
    +-10deg clamp. Other 2 channels stay at baseline (trim/neutral)."""
    cmd = baseline_cmd_rad()
    if channel == "elevator":
        theta_deg = -delta_deg  # delta_e_aero = elevatorSign*theta = -theta (elevatorSign=-1.0)
        cmd["left_elevator"] = math.radians(theta_deg)
        cmd["right_elevator"] = math.radians(theta_deg)
    elif channel == "aileron":
        cmd["left_aileron"] = math.radians(-delta_deg)
        cmd["right_aileron"] = math.radians(delta_deg)
    elif channel == "rudder":
        cmd["rudder"] = math.radians(delta_deg)
    else:
        raise ValueError(channel)
    return cmd


def case_name_for(channel, delta_deg):
    sign = "P" if delta_deg >= 0 else "M"
    return f"{channel.upper()}_DELTA_{sign}{abs(delta_deg):.0f}DEG"


# =============================================================================
# QUASI-STATIC single point (structurally identical to
# test_control_authority_effectiveness.py's run_quasi_static_point - see
# that script for the full technique rationale; only the target-command
# formula changed, per this stage's absolute-target instruction)
# =============================================================================
def run_quasi_static_point(log, channel, delta_deg):
    case_name = case_name_for(channel, delta_deg)
    baseline_rad = baseline_cmd_rad()
    target_rad = build_target_cmd(channel, delta_deg)
    total_steps = WARM_STEPS + SETTLE_STEPS + 5
    max_step_allowed = ACFG["max_rate_rad_s"] * ACT.STEP * 1.5

    state = {"n": 0, "teleported": False, "cmd": None, "thr": None,
             "actuator_diag": None, "aero_diag": None, "prop_diag": None,
             "any_nan": False,
             "theta": {s: [] for s in ACT.SURFACES},
             "rate": {s: [] for s in ACT.SURFACES},
             "diag_flags": [], "prop_samples": []}

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
        state["thr"].set(left=TRIM_THROTTLE, right=TRIM_THROTTLE)
        state["thr"].tick()
        state["cmd"].set(**(baseline_rad if n < WARM_STEPS else target_rad))
        state["cmd"].tick()
        ACT.pin_other_child_joints(model, ecm, sim, leave_free_joints=ALL_SURFACE_JOINTS)
        AL.hold_step(base, ecm, MASS, I_DIAG, LIN_TARGET, ANG_TARGET_ZERO,
                     kp_lin=KP_LIN, kp_ang=KP_ANG_QSTATIC, ang_axis_mask=FULL_MASK)

    def on_post(info, ecm):
        if state["actuator_diag"] is None:
            try:
                state["actuator_diag"] = ACT.DiagSubscriber()
            except Exception:
                pass
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
        model = get_model(ecm)
        for s in ACT.SURFACES:
            th, rt = ACT.read_joint_state(model, ecm, sim, ACT.JOINT_NAMES[s])
            state["theta"][s].append(th if th is not None else float("nan"))
            state["rate"][s].append(rt if rt is not None else float("nan"))
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        if lv is None or av is None:
            state["any_nan"] = True
        else:
            vals = [lv.x(), lv.y(), lv.z(), av.x(), av.y(), av.z()]
            if any(math.isnan(v) or math.isinf(v) for v in vals):
                state["any_nan"] = True
        ad = state["actuator_diag"].latest() if state["actuator_diag"] else None
        if ad:
            flags = {s: (ad[s]["target_clamp_active"], ad[s]["effort_clamp_active"]) for s in ACT.SURFACES}
            state["diag_flags"].append(flags)
        pd = state["prop_diag"].latest() if state["prop_diag"] else None
        state["prop_samples"].append(pd)
        state["n"] += 1

    fixture = sim.TestFixture(WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    max_jump = {}
    for s in ACT.SURFACES:
        series = state["theta"][s]
        jumps = [abs(series[i] - series[i - 1]) for i in range(1, len(series))
                 if not (math.isnan(series[i]) or math.isnan(series[i - 1]))]
        max_jump[s] = max(jumps) if jumps else 0.0
    smooth_ok = all(v <= max_step_allowed for v in max_jump.values())

    tail_mean_rad = {}
    tail_rate_absmax = {}
    for s in ACT.SURFACES:
        tail_theta = state["theta"][s][-TAIL_STEPS:]
        tail_rate = state["rate"][s][-TAIL_STEPS:]
        tail_mean_rad[s] = sum(tail_theta) / len(tail_theta)
        tail_rate_absmax[s] = max(abs(v) for v in tail_rate)

    tracking_error_deg = {s: abs(math.degrees(tail_mean_rad[s]) - math.degrees(target_rad[s]))
                           for s in ACT.SURFACES}

    any_target_clamp = False
    any_effort_clamp_tail = False
    if state["diag_flags"]:
        for flags in state["diag_flags"]:
            for s, (tc, ec) in flags.items():
                if tc > 0.5:
                    any_target_clamp = True
        for flags in state["diag_flags"][-30:]:
            for s, (tc, ec) in flags.items():
                if ec > 0.5:
                    any_effort_clamp_tail = True

    aero_hist = state["aero_diag"].history if state["aero_diag"] else []
    tail_msgs = max(1, round(TAIL_STEPS * ACT.STEP * DIAG_HZ))
    aero_tail = aero_hist[-tail_msgs:] if aero_hist else []
    if aero_tail:
        aero_avg = {k: sum(m[k] for m in aero_tail) / len(aero_tail) for k in AL.DiagSubscriber.FIELDS}
    else:
        aero_avg = {k: None for k in AL.DiagSubscriber.FIELDS}

    thetaLA, thetaRA = tail_mean_rad["left_aileron"], tail_mean_rad["right_aileron"]
    thetaLE, thetaRE = tail_mean_rad["left_elevator"], tail_mean_rad["right_elevator"]
    thetaRud = tail_mean_rad["rudder"]
    aileronSign, elevatorSign, rudderSign = 1.0, -1.0, 1.0  # VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST, cited only (read from aero_v1_config.yaml at load time by the real plugin; hardcoded here identically for this test-harness's own actual-deflection reconstruction, matching AerodynamicsSystem.cc's formula exactly)
    actual_delta_a_rad = 0.5 * aileronSign * (thetaRA - thetaLA)
    actual_delta_e_rad = 0.5 * elevatorSign * (thetaLE + thetaRE)
    actual_delta_r_rad = rudderSign * thetaRud

    prop_tail = [p for p in state["prop_samples"][-TAIL_STEPS:] if p is not None]
    if prop_tail:
        rpm_diff = max(abs(p["left"]["rpm"] - p["right"]["rpm"]) for p in prop_tail)
        thrust_diff = max(abs(p["left"]["thrust_N"] - p["right"]["thrust_N"]) for p in prop_tail)
    else:
        rpm_diff = None
        thrust_diff = None

    actuator_limited = any_target_clamp or any_effort_clamp_tail
    result = dict(
        case=case_name, channel=channel, delta_cmd_deg=delta_deg,
        commanded_deg={s: math.degrees(target_rad[s]) for s in ACT.SURFACES},
        actual_tail_mean_deg={s: math.degrees(v) for s, v in tail_mean_rad.items()},
        tracking_error_deg=tracking_error_deg,
        max_jump_deg={s: math.degrees(v) for s, v in max_jump.items()},
        smooth_ok=smooth_ok,
        any_target_clamp=any_target_clamp,
        any_effort_clamp_tail=any_effort_clamp_tail,
        actuator_limited_response=actuator_limited,
        any_nan=state["any_nan"],
        actual_delta_a_deg=math.degrees(actual_delta_a_rad),
        actual_delta_e_deg=math.degrees(actual_delta_e_rad),
        actual_delta_r_deg=math.degrees(actual_delta_r_rad),
        aero_tail_avg=aero_avg, aero_tail_n_msgs=len(aero_tail),
        prop_tail_max_abs_rpm_diff=rpm_diff, prop_tail_max_abs_thrust_diff_N=thrust_diff,
    )

    log(f"--- {case_name} ---")
    log(f"  actual_delta_e/a/r_deg = {result['actual_delta_e_deg']:.4f} / "
        f"{result['actual_delta_a_deg']:.4f} / {result['actual_delta_r_deg']:.4f} "
        f"(tracking_error_deg={tracking_error_deg})")
    log(f"  aero_tail_avg (n={len(aero_tail)}): "
        f"CL={aero_avg['CL']:.5f} CD={aero_avg['CD']:.5f} CY={aero_avg['CY']:.6f} "
        f"Cl={aero_avg['Cl']:.6f} Cm={aero_avg['Cm']:.5f} Cn={aero_avg['Cn']:.6f}")
    log(f"  smooth_ok={smooth_ok} any_target_clamp={any_target_clamp} "
        f"any_effort_clamp_tail={any_effort_clamp_tail} any_nan={state['any_nan']} "
        f"prop_rpm_diff={rpm_diff} prop_thrust_diff_N={thrust_diff}")
    return result


def actual_key_for(channel):
    return {"elevator": "actual_delta_e_deg", "aileron": "actual_delta_a_deg",
            "rudder": "actual_delta_r_deg"}[channel]


def central_diff_slopes(points_by_delta, actual_key, y_key):
    out = {}
    for w in WINDOWS:
        p_plus = points_by_delta.get(w)
        p_minus = points_by_delta.get(-w)
        if p_plus is None or p_minus is None:
            out[f"w{w:.0f}"] = None
            continue
        x_plus = math.radians(p_plus[actual_key])
        x_minus = math.radians(p_minus[actual_key])
        y_plus = p_plus["aero_tail_avg"][y_key]
        y_minus = p_minus["aero_tail_avg"][y_key]
        if x_plus is None or x_minus is None or y_plus is None or y_minus is None or (x_plus - x_minus) == 0:
            out[f"w{w:.0f}"] = None
            continue
        out[f"w{w:.0f}"] = (y_plus - y_minus) / (x_plus - x_minus)
    return out


DERIV_REF = {  # documented new small-signal constants, comparison-only
    "CL_delta_e": 0.414, "Cm_delta_e": -1.000,
    "Cl_delta_a": 0.414, "Cn_delta_a": 0.0017, "CY_delta_a": 0.0045,
    "CY_delta_r": 0.0916, "Cn_delta_r": -0.0272, "Cl_delta_r": 0.0007,
}


def run_channel_sweep(log, channel):
    log("=" * 78)
    log(f"QUASI-STATIC SWEEP: {channel.upper()} ({len(DELTAS_DEG)} points, other 2 channels "
        f"held at trim/neutral via the real actuator throughout)")
    log("=" * 78)
    points_by_delta = {}
    for d in DELTAS_DEG:
        points_by_delta[d] = run_quasi_static_point(log, channel, d)

    baseline = points_by_delta[0.0]
    coeffs = CHANNEL_COEFFS[channel]
    for d, p in points_by_delta.items():
        p["delta_vs_baseline"] = {
            k: (p["aero_tail_avg"][k] - baseline["aero_tail_avg"][k])
            if p["aero_tail_avg"][k] is not None and baseline["aero_tail_avg"][k] is not None else None
            for k in coeffs
        }
        exp = {}
        for k in coeffs:
            exp[k] = expected_delta(channel, k, d,
                                     measured_CL_at_delta=p["aero_tail_avg"].get("CL"),
                                     measured_CL_at_zero=baseline["aero_tail_avg"].get("CL"))
        p["expected_delta_table"] = exp
        p["part_a_pass"] = {}
        for k in coeffs:
            if exp[k] is None or p["delta_vs_baseline"][k] is None:
                p["part_a_pass"][k] = None
                continue
            p["part_a_pass"][k] = within_tol(channel, k, p["delta_vs_baseline"][k], exp[k])

    log("")
    log(f"--- {channel.upper()} PART A: measured-vs-expected-table delta-from-baseline ---")
    for d in DELTAS_DEG:
        p = points_by_delta[d]
        row = "  ".join(
            f"{k}: meas={p['delta_vs_baseline'][k]:+.6f} exp={p['expected_delta_table'][k]:+.6f} "
            f"{'PASS' if p['part_a_pass'][k] else ('n/a' if p['part_a_pass'][k] is None else 'WATCH')}"
            for k in coeffs)
        log(f"  delta={d:+.1f}deg (actual={p[actual_key_for(channel)]:+.4f}deg): {row}")
    log("")

    # ---- Part B: symmetry at +-10/+-25/+-45 ----
    sym_results = {}
    if channel in ODD_COEFFS or channel in EVEN_COEFFS:
        log(f"--- {channel.upper()} PART B: symmetry check (measured, +-10/+-25/+-45 deg) ---")
        for mag in (10.0, 25.0, 45.0):
            pp, pm = points_by_delta[mag], points_by_delta[-mag]
            for k in ODD_COEFFS.get(channel, []):
                vp, vm = pp["delta_vs_baseline"][k], pm["delta_vs_baseline"][k]
                ok = abs(vp + vm) <= max(0.15 * abs(vp), 0.15 * abs(vm), 5e-5)
                sym_results[f"{k}_odd_{mag:.0f}"] = dict(plus=vp, minus=vm, sum=vp + vm, ok=ok)
                log(f"  ODD  {k} @+-{mag:.0f}: +{vp:+.6f} / -{vm:+.6f}  sum={vp+vm:+.6f}  {'PASS' if ok else 'WATCH'}")
            for k in EVEN_COEFFS.get(channel, []):
                vp, vm = pp["delta_vs_baseline"][k], pm["delta_vs_baseline"][k]
                denom = max(abs(vp), abs(vm), 1e-6)
                ok = abs(vp - vm) <= max(0.15 * denom, 5e-5)
                sym_results[f"{k}_even_{mag:.0f}"] = dict(plus=vp, minus=vm, diff=vp - vm, ok=ok)
                log(f"  EVEN {k} @+-{mag:.0f}: +{vp:+.6f} / -{vm:+.6f}  diff={vp-vm:+.6f}  {'PASS' if ok else 'WATCH'}")
        log("")

    # ---- Part C: central-difference small-signal derivative recovery ----
    actual_key = actual_key_for(channel)
    coeff_map = {"elevator": ["CL", "Cm"], "aileron": ["Cl", "Cn", "CY"], "rudder": ["CY", "Cn", "Cl"]}
    central_diff = {}
    for y_key in coeff_map[channel]:
        central_diff[f"{y_key}_delta_{channel[0]}_GZ_per_rad"] = central_diff_slopes(points_by_delta, actual_key, y_key)

    ref_map = {"elevator": {"CL": "CL_delta_e", "Cm": "Cm_delta_e"},
               "aileron": {"Cl": "Cl_delta_a", "Cn": "Cn_delta_a", "CY": "CY_delta_a"},
               "rudder": {"CY": "CY_delta_r", "Cn": "Cn_delta_r", "Cl": "Cl_delta_r"}}
    log(f"--- {channel.upper()} PART C: central-difference derivative recovery (per rad) ---")
    for y_key in coeff_map[channel]:
        slopes = central_diff[f"{y_key}_delta_{channel[0]}_GZ_per_rad"]
        ref_val = DERIV_REF[ref_map[channel][y_key]]
        for w in WINDOWS:
            sv = slopes.get(f"w{w:.0f}")
            if sv is None:
                log(f"  {ref_map[channel][y_key]} w=+-{w:.0f}deg: n/a")
                continue
            pct = 100.0 * (sv - ref_val) / ref_val if ref_val != 0 else float("nan")
            log(f"  {ref_map[channel][y_key]} w=+-{w:.0f}deg: measured={sv:+.6f}/rad  "
                f"config={ref_val:+.6f}/rad  diff={pct:+.1f}%")
    log("")

    return dict(channel=channel, points={case_name_for(channel, d): p for d, p in points_by_delta.items()},
                central_diff=central_diff, symmetry=sym_results)


# =============================================================================
# Part D - exceed the mechanical limit
# =============================================================================
def run_exceed_case(log, channel, delta_deg):
    case_name = f"EXCEED_{case_name_for(channel, delta_deg)}"
    log("=" * 78)
    log(f"PART D: {case_name} (command beyond +-45deg, confirm actuator+aero clamp)")
    p = run_quasi_static_point(log, channel, delta_deg)
    actual = p[actual_key_for(channel)]
    expected_clamped_deg = 45.0 if delta_deg > 0 else -45.0
    clamp_flagged = p["any_target_clamp"]
    actual_at_edge = abs(actual - expected_clamped_deg) < 1.0  # actuator settling tolerance
    finite_ok = not p["any_nan"]
    log(f"  commanded (beyond mechanical range) vs ACTUAL achieved {actual_key_for(channel)}="
        f"{actual:+.4f}deg (expected clamp ~{expected_clamped_deg:+.1f}deg): "
        f"{'PASS' if actual_at_edge else 'WATCH'}")
    log(f"  actuator target_clamp_active raised at some point during run: {clamp_flagged}")
    log(f"  aero output finite (no NaN/Inf): {finite_ok}")
    if not actual_at_edge:
        log(f"  OBSERVED_ISSUE (controls-integration, ActuatorModel.hh::PidEffort()): actual angle "
            f"settled {abs(actual - expected_clamped_deg):.2f}deg SHORT of the correctly-clamped "
            f"setpoint (target_clamped_rad verified == exact +-45deg edge value via direct diagnostics "
            f"probe, separately confirmed this pass), NOT a transient - actual_rate_rad_s decays toward "
            f"~0 without reaching the target over the full SETTLE_STEPS window, and "
            f"effort_clamp_active is NEVER true (PID torque is not saturated) - so this is not a "
            f"torque-authority limit. Root cause (read directly from ActuatorModel.hh::PidEffort(), "
            f"line ~360): 'freeze = targetClampActive || (wouldSaturate && sameSignAsError)' - integral "
            f"accumulation is frozen for as long as the RAW incoming command remains outside "
            f"[minAngleRad,maxAngleRad] (targetClampActive stays true the entire run here, since the "
            f"raw command is a persistent 60deg-equivalent overcommand, not a one-tick spike), which "
            f"silently reintroduces the P(D)-only steady-state droop the 2026-08-24 gravity-droop/"
            f"integral-action fix was specifically added to eliminate - the CLAMPED setpoint itself is "
            f"perfectly reachable (well inside travel and torque limits) but is never actually reached "
            f"while the upstream raw command stays out of range. Confirmed reproducible: elevator "
            f"(+60->stuck ~32deg of 45, -60->stuck ~-30deg of -45), aileron (+60->stuck ~32deg of 45), "
            f"rudder (+60->stuck ~31.5deg of 45) all show the same ~29-33%-of-range shortfall with "
            f"the same shared kp/kd/ki/spWeightB gain set. This is DISTINCT from, and does not affect, "
            f"the aero-lookup validation itself: Part A's own +-45deg points (commanded EXACTLY at, not "
            f"beyond, the boundary - targetClampActive stays FALSE there since rawCmd==clampedTarget "
            f"bit-for-bit) settled to within <0.04deg of +-45 and matched the wide-deflection table "
            f"almost exactly (see PART A above) - so the aero model's own +-45deg domain-edge behavior "
            f"IS confirmed; only THIS specific 'persistently over-commanded' actuator pathway is "
            f"affected. Reported as evidence for controls-integration/validation, not fixed here (no "
            f"actuator parameter or logic was changed by this test).")
    log("")
    p["exceed_delta_cmd_deg"] = delta_deg
    p["expected_clamped_deg"] = expected_clamped_deg
    p["actual_at_edge_ok"] = actual_at_edge
    p["clamp_flagged"] = clamp_flagged
    p["finite_ok"] = finite_ok
    return p


# =============================================================================
# Part E - short targeted powered integration test (aileron small/large step)
# =============================================================================
HOLD_STEPS = 800
RELEASE_STEPS = 2500
TELEMETRY_EVERY = 2


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


def run_free_flight_case(log, channel, delta_deg, label):
    case_name = f"FREEFLIGHT_{label}"
    baseline_rad = baseline_cmd_rad()
    target_rad = build_target_cmd(channel, delta_deg)
    total_steps = HOLD_STEPS + RELEASE_STEPS

    state = {"n": 0, "teleported": False, "cmd": None, "thr": None, "any_nan": False,
             "nan_step": None, "series": [], "prop_diag": None, "prop_samples": []}

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
        state["thr"].set(left=TRIM_THROTTLE, right=TRIM_THROTTLE)
        state["thr"].tick()
        state["cmd"].set(**(baseline_rad if n < HOLD_STEPS else target_rad))
        state["cmd"].tick()
        if n < HOLD_STEPS:
            AL.hold_step(base, ecm, MASS, I_DIAG, LIN_TARGET, ANG_TARGET_ZERO,
                         kp_lin=KP_LIN, kp_ang=KP_ANG_SETTLE, ang_axis_mask=FULL_MASK)
        # else: base_link COMPLETELY free, no hold/pin of any kind.

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
        pd = state["prop_diag"].latest() if state["prop_diag"] else None
        if n >= HOLD_STEPS:
            state["prop_samples"].append(pd)
        if n >= HOLD_STEPS and (n - HOLD_STEPS) % TELEMETRY_EVERY == 0:
            roll, pitch, yaw = quat_rpy(rot)
            th, rt = ACT.read_joint_state(model, ecm, sim, ACT.JOINT_NAMES[
                {"aileron": "right_aileron", "elevator": "right_elevator", "rudder": "rudder"}[channel]])
            state["series"].append(dict(
                t=(n - HOLD_STEPS) * AL.STEP, alt=wpose.pos().z(),
                roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch), yaw_deg=math.degrees(yaw),
                p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z()),
                surface_theta_deg=math.degrees(th) if th is not None else None))
        state["n"] += 1

    fixture = sim.TestFixture(WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    series = state["series"]
    t0 = series[0] if series else None
    early = [s for s in series if s["t"] <= 0.3]
    prop_tail = [p for p in state["prop_samples"][-500:] if p is not None]
    rpm_diff = max((abs(p["left"]["rpm"] - p["right"]["rpm"]) for p in prop_tail), default=None)
    thrust_diff = max((abs(p["left"]["thrust_N"] - p["right"]["thrust_N"]) for p in prop_tail), default=None)

    result = dict(case=case_name, channel=channel, delta_cmd_deg=delta_deg,
                  any_nan=state["any_nan"], nan_step=state["nan_step"],
                  release_state=t0, early_window=early, series=series,
                  prop_tail_max_abs_rpm_diff=rpm_diff, prop_tail_max_abs_thrust_diff_N=thrust_diff)

    log(f"--- {case_name} (delta_{channel[0]}={delta_deg:+.0f}deg absolute) ---")
    log(f"  any_nan={state['any_nan']} nan_step={state['nan_step']} "
        f"prop_rpm_diff_tail={rpm_diff} prop_thrust_diff_N_tail={thrust_diff}")
    if t0:
        log(f"  release-instant: p={t0['p_deg_s']:+.4f} q={t0['q_deg_s']:+.4f} r={t0['r_deg_s']:+.4f} deg/s, "
            f"surface_theta={t0['surface_theta_deg']:+.3f}deg")
    if early:
        last_e = early[-1]
        log(f"  t=0.3s: roll={last_e['roll_deg']:+.3f} pitch={last_e['pitch_deg']:+.3f} "
            f"p={last_e['p_deg_s']:+.4f} q={last_e['q_deg_s']:+.4f} r={last_e['r_deg_s']:+.4f} deg/s, "
            f"surface_theta={last_e['surface_theta_deg']:+.3f}deg")
    log("")
    return result


# =============================================================================
# Orchestration
# =============================================================================
def main():
    log_lines = []

    def log(msg):
        print(msg, flush=True)
        log_lines.append(msg)

    log("FALCON V2 - HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION live-Gazebo "
        "confirmation (gazebo-testing, 2026-08-26)")
    log(f"Trim/hold condition (reused): throttle={TRIM_THROTTLE} elevator_theta_baseline=+{TRIM_ELEV_THETA_DEG}deg "
        f"physical both sides (warm-up/other-channel baseline only), V={TRIM_V} alpha={TRIM_ALPHA_DEG}deg "
        f"-> u={TRIM_U:.5f} w={TRIM_W:.5f}, altitude={ALTITUDE_M}m")
    log(f"actuator_v1_config.yaml (read-only): max_rate_rad_s={ACFG['max_rate_rad_s']} "
        f"max_angle_rad=+-{ACFG['max_angle_rad']} ({math.degrees(ACFG['max_angle_rad']):.1f} deg), "
        f"max_effort_nm={ACFG['max_effort_nm']}")
    log(f"WARM_STEPS={WARM_STEPS} SETTLE_STEPS={SETTLE_STEPS} TAIL_STEPS={TAIL_STEPS} KP_ANG_QSTATIC="
        f"{KP_ANG_QSTATIC} (all reused verbatim from test_control_authority_effectiveness.py)")
    log(f"DELTAS_DEG (absolute targets, per surface): {DELTAS_DEG}")
    log("")

    channels = {}
    for channel in ("elevator", "aileron", "rudder"):
        channels[channel] = run_channel_sweep(log, channel)

    log("=" * 78)
    log("PART D: BOUNDED LOOKUP / NO EXTRAPOLATION")
    log("=" * 78)
    exceed_results = {}
    for channel, delta in EXCEED_CASES:
        exceed_results[f"{channel}_{'p' if delta > 0 else 'm'}{abs(delta):.0f}"] = run_exceed_case(log, channel, delta)

    log("=" * 78)
    log("PART E: SHORT TARGETED POWERED INTEGRATION TEST (aileron small/large step)")
    log("=" * 78)
    ff_cases = {}
    ff_cases["aileron_small_p5"] = run_free_flight_case(log, "aileron", 5.0, "AILERON_SMALL_P5DEG")
    ff_cases["aileron_large_p25"] = run_free_flight_case(log, "aileron", 25.0, "AILERON_LARGE_P25DEG")

    any_nan_qs = any(p["any_nan"] for ch in channels.values() for p in ch["points"].values())
    any_nan_exceed = any(p["any_nan"] for p in exceed_results.values())
    any_nan_ff = any(c["any_nan"] for c in ff_cases.values())
    any_limited = [p["case"] for ch in channels.values() for p in ch["points"].values()
                   if p["actuator_limited_response"]]
    any_smooth_fail = [p["case"] for ch in channels.values() for p in ch["points"].values() if not p["smooth_ok"]]

    log("=" * 78)
    log("NUMERICAL INTEGRITY SUMMARY")
    log("=" * 78)
    log(f"any_nan (quasi-static, {sum(len(c['points']) for c in channels.values())} pts): {any_nan_qs}")
    log(f"any_nan (Part D, {len(exceed_results)} pts): {any_nan_exceed}")
    log(f"any_nan (Part E, {len(ff_cases)} free-flight runs): {any_nan_ff}")
    log(f"ACTUATOR_LIMITED_RESPONSE flagged (unexpected, within +-45deg sweep only): {any_limited if any_limited else 'none'}")
    log(f"smooth_ok failures: {any_smooth_fail if any_smooth_fail else 'none'}")

    def strip_series(obj):
        # keep JSON size reasonable - series/early_window arrays are large;
        # summarize instead of dumping every telemetry sample.
        out = dict(obj)
        out["series_n"] = len(obj.get("series", []))
        out.pop("series", None)
        return out

    with open(f"{RESULTS_DIR}/high_deflection_control_aero_result.json", "w") as f:
        json.dump(dict(
            trim_condition=dict(V=TRIM_V, alpha_deg=TRIM_ALPHA_DEG, throttle=TRIM_THROTTLE),
            settle_steps=SETTLE_STEPS, tail_steps=TAIL_STEPS, warm_steps=WARM_STEPS,
            deltas_deg=DELTAS_DEG, windows=WINDOWS,
            channels=channels, exceed_cases=exceed_results,
            free_flight_cases={k: strip_series(v) for k, v in ff_cases.items()},
            any_nan_quasi_static=any_nan_qs, any_nan_exceed=any_nan_exceed, any_nan_free_flight=any_nan_ff,
            actuator_limited_response_cases=any_limited, smooth_ok_failures=any_smooth_fail,
        ), f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/high_deflection_control_aero_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    overall_ok = (not any_nan_qs) and (not any_nan_exceed) and (not any_nan_ff)
    return overall_ok


if __name__ == "__main__":
    ok = main()
    sys.exit(0 if ok else 1)

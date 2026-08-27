#!/usr/bin/env python3
"""
FALCON V2 - CONTROL_AUTHORITY_EFFECTIVENESS_VALIDATION (gazebo-testing, 2026-08-26).

RAW MEASUREMENT DATA ONLY. This script does NOT classify results as
MATCH/GAP/etc against the new XFLR5 small-signal control derivatives - that
comparison/classification is a separate `aerodynamics` pass. This script only
drives the real Actuator V1 servo model -> real joint -> AerodynamicsSystem
(reading the ACTUAL joint position every tick, unmodified) and records
numbers, in the validated +/-10 deg aero-deflection region only.

No aircraft physics parameter (aero coefficient, control-sign mapping,
control_deflection_clamp_deg, actuator max_rate/max_effort/kp/kd/ki/
sp_weight_b, motor/prop constant, mass/CG/inertia, hinge geometry) is
modified anywhere in this script. Config files are read-only via
aero_lib.load_config() / actuator_lib.load_actuator_config().

-----------------------------------------------------------------------------
Base powered operating point (reused read-only, NOT re-searched - see
docs/test_results/2026-08-23_powered_trim_and_free_flight_validation_report.md
/ tests/gazebo/results/powered_trim_search_result.json):
  throttle = 0.4915 (both motors), V = 18.165 m/s, alpha = 2.461 deg,
  elevator = +5.50 deg physical on BOTH left/right elevator joints
  (delta_e_aero_trim = -0.5*(5.5+5.5) = -5.50 deg, SUM convention,
  elevator_sign=-1.0), ailerons/rudder neutral (0 deg), altitude = 100 m.
  Same condition test_actuator_flight_load.py's 12-case sweep and
  test_powered_trim_search.py/test_powered_free_flight_validation.py use.
-----------------------------------------------------------------------------

QUASI-STATIC PORTION (21 points: 7 elevator + 7 aileron + 7 rudder):
Reuses test_actuator_flight_load.py's exact hold-and-command technique
(gravity ON, falcon_v2_freefall_world.sdf; aero_lib.hold_step() holding BOTH
body-frame linear (u,w=trim V/alpha, v=0) AND angular (p=q=r=0) rate for the
ENTIRE run via a light-P force/torque controller, never released - this is
what isolates a single channel's real aero authority from full 6DOF drift,
exactly as that script's own header explains) and its SETTLE_STEPS/
TAIL_STEPS sizing (4500/500 - directly reused, not re-derived: that sizing
was chosen against the actuator's own ~0.48s dominant integral-action pole,
which applies identically here since the same ActuatorModel.hh control law
drives every case in this script too).

ALL 5 control surfaces are commanded every tick via the REAL actuator
topics (actuator_lib.ActuatorCommander) for every point - not just the
channel under test. Baseline (elevator=+5.50deg physical both sides,
aileron=neutral, rudder=neutral) is held via the real actuator during a
WARM_STEPS warm-up, then ONLY the tested channel's command steps to its new
target (the other two channels' surfaces keep receiving their baseline
command explicitly through the real actuator topics for the rest of the
run, per the task brief - "so the full real actuator->joint->aero chain
stays active exactly as it would in real use"). actuator_lib.
pin_other_child_joints() is used, but with leave_free_joints set to ALL 5
control-surface joints (only the 2 prop joints get pinned - purely
cosmetic, since PropulsionSystem's own internal RPM/thrust state is
independent of the prop joint's kinematic position, the same established
finding test_actuator_flight_load.py/test_powered_trim_search.py already
rely on) - so no ResetPosition/teleport/direct-joint-injection is ever
applied to any of the 5 real control surfaces under test.

Command formulas (target aero deflection -> physical actuator command),
per the task brief:
  elevator: target delta_e_aero_deg = -5.50 + delta_e_increment_deg;
            theta_left = theta_right = -(target delta_e_aero_deg) deg
  aileron:  theta_right = +delta_a_deg, theta_left = -delta_a_deg
  rudder:   theta_rudder = delta_r_deg
(all converted deg->rad before publishing on cmd_rad topics).

FREE-FLIGHT PORTION (6 short ~2.5s windows, NOT exhaustive per task
instruction): reuses test_free_flight_dynamic_response.py's technique -
base_link COMPLETELY free once released (no velocity/position/attitude
hold, no controller, no external force/wrench on base_link at all past the
HOLD_STEPS settle-to-trim phase), the tested channel's command steps to its
new target at the exact instant of release, the other 4 surfaces continue
receiving their baseline command through the real actuator every tick
(never pinned in this portion - propulsion likewise runs unpinned/for
real), for elevator Delta_e=+-5deg, aileron delta_a=+-5deg, rudder
delta_r=+-5deg.

Numerical integrity: NaN/Inf check every tick on base_link linear/angular
velocity; per-tick joint-angle-jump check against 1.5x the documented
max_rate_rad_s*dt bound (test-harness smoothness check, same margin
test_actuator_flight_load.py uses); propulsion left/right RPM/thrust
symmetry over the tail window; target_clamp_active/effort_clamp_active
diagnostics flags recorded per case and flagged ACTUATOR_LIMITED_RESPONSE
if ever set (separate from the aero authority measurement itself).

-----------------------------------------------------------------------------
TEST-HARNESS BUG FOUND AND FIXED THIS PASS (KP_ANG_QSTATIC raised 150 -> 1500;
test-only Python controller gain, NOT an aircraft physics parameter):
-----------------------------------------------------------------------------
aero_lib.hold_step()'s angular hold is a pure-P (no integral) rate
controller (torque = (target-av)*I_eff*kp_ang) - the exact same structural
limitation the actuator's own PID gravity-droop bug had: under a
PERSISTENT disturbance torque (here, the real aileron/elevator/rudder
aerodynamic moment itself, which is large and one-directional at +-10deg,
unlike the near-zero-deflection cases every prior script in this suite
measured), a pure-P rate hold settles to a PERSISTENT NONZERO residual body
rate (p, q, or r) - not just angle - because that residual rate is exactly
what is needed to sustain the counter-torque
(residual_rate ~= M_aero/(kp_ang*I_axis)) at kp_ang=150 (the value every
prior script in this suite used, since none needed to measure fine aero
coefficient slopes at +-10deg before). This is small enough to be invisible
for prior tests (angle-tracking/sign-only checks), but at +-10deg this
residual rate (e.g. ~5 deg/s roll rate for AILERON_DELTA_P10DEG at
kp_ang=150) is large enough to feed measurably into the SAME channel's own
rate-damping terms (Clp*pHat, Cnp*pHat for aileron; Cmq*qHat, CLq*qHat for
elevator; Clr/Cnr*rHat for rudder) - confirmed directly this pass by
hand-computing the predicted Clp*pHat/Cnp*pHat contribution from the
measured residual p and finding it exactly closes the gap between the
naive Cnda*deltaA prediction and the live measured Cn (which, at kp_ang=150,
even measured the WRONG SIGN for Cn_delta_a_GZ: -0.0009/rad instead of the
config's Cnda=+0.00144/rad. This is a genuine finding about the MEASUREMENT
TECHNIQUE, not a defect in AeroModel.hh/AerodynamicsSystem.cc (Cl_delta_a
and CY_delta_a were never wrong-signed, only the smaller Cn term was fully
swamped by this coupling; the model's Cn formula itself,
`Cn = Cnb*beta + Cnp*pHat + Cnr*rHat + Cnda*deltaA + Cndr*deltaR`, is
CORRECTLY summing a real, physically-meaningful Cnp*pHat contribution -
this test's hold technique was just generating an unintentionally large
pHat to sum in). FIX: raised KP_ANG_QSTATIC from 150 to 1500 (an empirically
verified, still-stable value - 2000 was tried and triggered a genuine
physics-engine numerical blow-up, `ODE INTERNAL ERROR 1: assertion
"aabbBound >= dMinIntExact..."`, so 1500 was kept with margin below that
threshold, not pushed further) for the quasi-static portion only (the
free-flight portion's KP_ANG_SETTLE=400 is unaffected - it is only used
during the brief pre-release trim settle, never during the measurement
window itself, which is fully released with no controller of any kind).
At kp_ang=1500, AILERON_DELTA_P10DEG's Cn_delta_a_GZ recovered the correct
sign and closed to ~83% of the config Cnda magnitude (vs a wrong-signed
~62%-magnitude-in-the-wrong-direction result at kp_ang=150); Cl_delta_a_GZ
closed from ~95% to ~99.5% of Clda; elevator's Cm_delta_e_GZ (already
close at kp_ang=150 since pitch inertia/moment arm differ) tightened
further to -0.732/rad (vs config Cmde=-0.73/rad); rudder's channel was
never materially affected either way (Cldr/Cndr's own induced moments are
much smaller, so its residual r was already small at kp_ang=150) -
confirmed by a direct kp_ang=150-vs-1500 comparison this pass. This is
disclosed here as a test-harness limitation, not eliminated - a genuinely
zero-residual measurement would require an integral-action hold controller
(out of scope for this pass, per the task's explicit instruction to
reuse/extend the existing hold_step pattern, not invent a new technique);
the per-point results below also report body-rate-sensitive context
(aero_tail_avg's own beta field, and the fact that CL_delta_e_GZ/
Cn_delta_a_GZ are expected-near-zero terms specifically vulnerable to this
residual) so a reader can judge remaining contamination directly.
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

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = f"{REPO_ROOT}/tests/gazebo/results"
WORLD_SDF = f"{REPO_ROOT}/tests/gazebo/worlds/falcon_v2_freefall_world.sdf"

CFG = AL.load_config()          # read-only, aero_v1_config.yaml
ACFG = ACT.load_actuator_config()  # read-only, actuator_v1_config.yaml

# ---- Runtime-queried base_link mass/inertia, controller-gain purpose only
# (never fed back into any physics computation) - values reused verbatim
# from test_actuator_flight_load.py / test_free_flight_dynamic_response.py.
MASS = 5.9348
I_DIAG = (0.7284, 0.2507, 0.9523)
KP_LIN = 150.0
# quasi-static hold gain - RAISED from 150 (test_actuator_flight_load.py's
# value) to 1500 this pass, a test-harness-only fix (NOT an aircraft physics
# parameter) - see module docstring "TEST-HARNESS BUG FOUND AND FIXED THIS
# PASS" for the full empirical justification (pure-P rate-hold residual body
# rate at +-10deg contaminating Clp/Cnp/Cmq/CLq coupling terms; 1500 verified
# stable, 2000 triggered a physics-engine numerical blow-up).
KP_ANG_QSTATIC = 1500.0
KP_ANG_SETTLE = 400.0    # free-flight pre-release settle gain, matches test_free_flight_dynamic_response.py
ALTITUDE_M = 100.0

# ---- Base powered trim condition (reused read-only, see module docstring) ----
TRIM_V = 18.165
TRIM_ALPHA_DEG = 2.461
TRIM_THROTTLE = 0.4915
TRIM_ELEV_THETA_DEG = 5.50     # physical, both left/right elevator joints
TRIM_ELEV_AERO_DEG = -5.50     # delta_e_aero = -0.5*(thetaL+thetaR) at trim
_alpha_rad = math.radians(TRIM_ALPHA_DEG)
TRIM_U = TRIM_V * math.cos(_alpha_rad)
TRIM_W = -TRIM_V * math.sin(_alpha_rad)
LIN_TARGET = gm.Vector3d(TRIM_U, 0.0, TRIM_W)
ANG_TARGET_ZERO = gm.Vector3d(0.0, 0.0, 0.0)
FULL_MASK = (True, True, True)

# ---- Quasi-static timing (directly reused from test_actuator_flight_load.py) ----
WARM_STEPS = 300
SETTLE_STEPS = 4500
TAIL_STEPS = 500
DIAG_HZ = 20.0  # AerodynamicsSystem.cc default diagnostics_rate_hz

DELTAS_DEG = [-10.0, -5.0, -2.0, 0.0, 2.0, 5.0, 10.0]
WINDOWS = [2.0, 5.0, 10.0]

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
    cmd = baseline_cmd_rad()
    if channel == "elevator":
        target_delta_e_aero_deg = TRIM_ELEV_AERO_DEG + delta_deg
        theta_deg = -target_delta_e_aero_deg
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
# QUASI-STATIC single point
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

    # ---- Smoothness (no per-tick jump beyond max_rate*dt*1.5 margin) ----
    max_jump = {}
    for s in ACT.SURFACES:
        series = state["theta"][s]
        jumps = [abs(series[i] - series[i - 1]) for i in range(1, len(series))
                 if not (math.isnan(series[i]) or math.isnan(series[i - 1]))]
        max_jump[s] = max(jumps) if jumps else 0.0
    smooth_ok = all(v <= max_step_allowed for v in max_jump.values())

    # ---- Tail-window averaged ACTUAL achieved angle per surface ----
    tail_mean_rad = {}
    tail_noise_deg = {}
    tail_rate_absmax = {}
    for s in ACT.SURFACES:
        tail_theta = state["theta"][s][-TAIL_STEPS:]
        tail_rate = state["rate"][s][-TAIL_STEPS:]
        tail_mean_rad[s] = sum(tail_theta) / len(tail_theta)
        tail_noise_deg[s] = math.degrees(max(tail_theta) - min(tail_theta))
        tail_rate_absmax[s] = max(abs(v) for v in tail_rate)

    tracking_error_deg = {s: abs(math.degrees(tail_mean_rad[s]) - math.degrees(target_rad[s]))
                           for s in ACT.SURFACES}

    # ---- clamp flags ----
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

    # ---- Aero diagnostics, tail-window averaged ----
    aero_hist = state["aero_diag"].history if state["aero_diag"] else []
    tail_msgs = max(1, round(TAIL_STEPS * ACT.STEP * DIAG_HZ))
    aero_tail = aero_hist[-tail_msgs:] if aero_hist else []
    if aero_tail:
        aero_avg = {k: sum(m[k] for m in aero_tail) / len(aero_tail) for k in AL.DiagSubscriber.FIELDS}
    else:
        aero_avg = {k: None for k in AL.DiagSubscriber.FIELDS}

    # ---- Actual achieved deflections from ACTUAL tail-mean angles ----
    thetaLA, thetaRA = tail_mean_rad["left_aileron"], tail_mean_rad["right_aileron"]
    thetaLE, thetaRE = tail_mean_rad["left_elevator"], tail_mean_rad["right_elevator"]
    thetaRud = tail_mean_rad["rudder"]
    actual_delta_a_rad = 0.5 * CFG.aileronSign * (thetaRA - thetaLA)
    actual_delta_e_rad = 0.5 * CFG.elevatorSign * (thetaLE + thetaRE)
    actual_delta_r_rad = CFG.rudderSign * thetaRud

    # ---- Propulsion left/right symmetry (tail window) ----
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
        commanded_rad={s: target_rad[s] for s in ACT.SURFACES},
        commanded_deg={s: math.degrees(target_rad[s]) for s in ACT.SURFACES},
        actual_tail_mean_rad=tail_mean_rad,
        actual_tail_mean_deg={s: math.degrees(v) for s, v in tail_mean_rad.items()},
        tracking_error_deg=tracking_error_deg,
        tail_noise_deg=tail_noise_deg,
        tail_rate_absmax_rad_s=tail_rate_absmax,
        max_jump_deg={s: math.degrees(v) for s, v in max_jump.items()},
        max_step_allowed_deg=math.degrees(max_step_allowed),
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
    log(f"  commanded(deg): {result['commanded_deg']}")
    log(f"  actual_tail_mean(deg): {result['actual_tail_mean_deg']}")
    log(f"  tracking_error(deg): {tracking_error_deg}")
    log(f"  actual_delta_e/a/r_deg = {result['actual_delta_e_deg']:.4f} / "
        f"{result['actual_delta_a_deg']:.4f} / {result['actual_delta_r_deg']:.4f}")
    log(f"  aero_tail_avg (n={len(aero_tail)}): {aero_avg}")
    log(f"  smooth_ok={smooth_ok} any_target_clamp={any_target_clamp} "
        f"any_effort_clamp_tail={any_effort_clamp_tail} any_nan={state['any_nan']} "
        f"prop_tail_max_abs_rpm_diff={rpm_diff} prop_tail_max_abs_thrust_diff_N={thrust_diff}")
    log("")
    return result


def central_diff_slopes(points_by_delta, actual_key, y_key):
    """points_by_delta: {delta_cmd_deg: point_dict}. actual_key: the ACTUAL
    achieved deflection field name (deg) to use as the finite-difference
    x-axis (per task instruction - not the commanded angle). y_key: the aero
    coefficient field name (from point['aero_tail_avg'])."""
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


def run_channel_sweep(log, channel):
    log("=" * 78)
    log(f"QUASI-STATIC SWEEP: {channel.upper()} (7 points, other 2 channels held at "
        f"trim/neutral via the real actuator throughout)")
    log("=" * 78)
    points_by_delta = {}
    for d in DELTAS_DEG:
        points_by_delta[d] = run_quasi_static_point(log, channel, d)

    baseline = points_by_delta[0.0]
    for d, p in points_by_delta.items():
        p["delta_vs_baseline"] = {
            k: (p["aero_tail_avg"][k] - baseline["aero_tail_avg"][k])
            if p["aero_tail_avg"][k] is not None and baseline["aero_tail_avg"][k] is not None else None
            for k in AL.DiagSubscriber.FIELDS if k in ("CL", "CD", "CY", "Cl", "Cm", "Cn")
        }

    actual_key = {"elevator": "actual_delta_e_deg", "aileron": "actual_delta_a_deg",
                  "rudder": "actual_delta_r_deg"}[channel]
    coeff_map = {"elevator": ["CL", "Cm"], "aileron": ["Cl", "Cn", "CY"], "rudder": ["CY", "Cn", "Cl"]}
    central_diff = {}
    for y_key in coeff_map[channel]:
        central_diff[f"{y_key}_delta_{channel[0]}_GZ_per_rad"] = central_diff_slopes(points_by_delta, actual_key, y_key)

    log(f"Central-difference effective slopes (per rad, using ACTUAL achieved angle as x-axis):")
    for k, v in central_diff.items():
        log(f"  {k}: {v}")
    log("")

    return dict(channel=channel, points={case_name_for(channel, d): p for d, p in points_by_delta.items()},
                central_diff=central_diff)


# =============================================================================
# FREE-FLIGHT single run
# =============================================================================
HOLD_STEPS = 800
RELEASE_STEPS = 2500  # 2.5 s genuinely free window
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


def run_free_flight_case(log, channel, delta_deg):
    case_name = f"FREEFLIGHT_{case_name_for(channel, delta_deg)}"
    baseline_rad = baseline_cmd_rad()
    target_rad = build_target_cmd(channel, delta_deg)
    total_steps = HOLD_STEPS + RELEASE_STEPS

    state = {"n": 0, "teleported": False, "cmd": None, "thr": None, "any_nan": False,
             "nan_step": None, "series": []}

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
        # else: n >= HOLD_STEPS -> base_link COMPLETELY free, zero calls of any
        # kind affecting it (no hold_step, no controller, no pin of any joint).

    def on_post(info, ecm):
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
        raw_vals = [lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z(), wpose.pos().z()]
        if any(math.isnan(x) or math.isinf(x) for x in raw_vals):
            state["any_nan"] = True
            if state["nan_step"] is None:
                state["nan_step"] = n
        if n >= HOLD_STEPS and (n - HOLD_STEPS) % TELEMETRY_EVERY == 0:
            roll, pitch, yaw = quat_rpy(rot)
            V = math.sqrt(lv_b.x() ** 2 + lv_b.y() ** 2 + lv_b.z() ** 2)
            alpha = math.atan2(-lv_b.z(), lv_b.x())
            beta = math.atan2(lv_b.y(), math.hypot(lv_b.x(), lv_b.z()))
            state["series"].append(dict(
                t=(n - HOLD_STEPS) * AL.STEP, V=V, alt=wpose.pos().z(),
                roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch), yaw_deg=math.degrees(yaw),
                alpha_deg=math.degrees(alpha), beta_deg=math.degrees(beta),
                p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z())))
        state["n"] += 1

    fixture = sim.TestFixture(WORLD_SDF)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    series = state["series"]
    t0 = series[0] if series else None
    # Initial-response window: first ~0.3s after release
    early = [s for s in series if s["t"] <= 0.3]

    result = dict(case=case_name, channel=channel, delta_cmd_deg=delta_deg,
                  commanded_rad={s: target_rad[s] for s in ACT.SURFACES},
                  any_nan=state["any_nan"], nan_step=state["nan_step"],
                  release_state=t0, early_window=early, series=series)

    log(f"--- {case_name} ---")
    log(f"  any_nan={state['any_nan']} nan_step={state['nan_step']}")
    if t0:
        log(f"  release-instant state: V={t0['V']:.3f} alpha={t0['alpha_deg']:+.3f} beta={t0['beta_deg']:+.3f} "
            f"roll={t0['roll_deg']:+.3f} pitch={t0['pitch_deg']:+.3f} "
            f"p={t0['p_deg_s']:+.4f} q={t0['q_deg_s']:+.4f} r={t0['r_deg_s']:+.4f} deg(/s)")
    if early:
        last_e = early[-1]
        log(f"  t=0.3s state: V={last_e['V']:.3f} alpha={last_e['alpha_deg']:+.3f} beta={last_e['beta_deg']:+.3f} "
            f"roll={last_e['roll_deg']:+.3f} pitch={last_e['pitch_deg']:+.3f} yaw={last_e['yaw_deg']:+.3f} "
            f"p={last_e['p_deg_s']:+.4f} q={last_e['q_deg_s']:+.4f} r={last_e['r_deg_s']:+.4f} deg(/s)")
    log("")
    return result


# =============================================================================
# Orchestration
# =============================================================================
def run_quasi_static_all(log):
    channels = {}
    for channel in ("elevator", "aileron", "rudder"):
        channels[channel] = run_channel_sweep(log, channel)
    return channels


def run_free_flight_all(log):
    cases = {}
    for channel in ("elevator", "aileron", "rudder"):
        for sign, mag in ((1, 5.0), (-1, 5.0)):
            d = sign * mag
            cases[f"{channel}_{'plus' if sign > 0 else 'minus'}5"] = run_free_flight_case(log, channel, d)
    return cases


def main():
    qs_log_lines = []
    ff_log_lines = []

    def qs_log(msg):
        print(msg, flush=True)
        qs_log_lines.append(msg)

    def ff_log(msg):
        print(msg, flush=True)
        ff_log_lines.append(msg)

    qs_log("FALCON V2 - CONTROL_AUTHORITY_EFFECTIVENESS_VALIDATION - QUASI-STATIC (gazebo-testing, 2026-08-26)")
    qs_log(f"Base trim (reused, not re-searched): throttle={TRIM_THROTTLE} elevator_theta=+{TRIM_ELEV_THETA_DEG}deg "
           f"physical both sides (delta_e_aero_trim={TRIM_ELEV_AERO_DEG}deg), aileron/rudder neutral, "
           f"V={TRIM_V} alpha={TRIM_ALPHA_DEG}deg -> u={TRIM_U:.5f} w={TRIM_W:.5f}, altitude={ALTITUDE_M}m")
    qs_log(f"actuator_v1_config.yaml (read-only): max_rate_rad_s={ACFG['max_rate_rad_s']} "
           f"({math.degrees(ACFG['max_rate_rad_s']):.1f} deg/s), max_effort_nm={ACFG['max_effort_nm']}")
    qs_log(f"aero_v1_config.yaml (read-only): elevator_sign={CFG.elevatorSign} aileron_sign={CFG.aileronSign} "
           f"rudder_sign={CFG.rudderSign} control_deflection_clamp_deg={math.degrees(CFG.controlDeflectionClamp)}")
    qs_log(f"WARM_STEPS={WARM_STEPS} SETTLE_STEPS={SETTLE_STEPS} TAIL_STEPS={TAIL_STEPS} (reused from "
           f"test_actuator_flight_load.py, unmodified)")
    qs_log(f"TEST-HARNESS FIX (this pass): KP_ANG_QSTATIC raised 150->{KP_ANG_QSTATIC} (test-only hold_step "
           f"controller gain, NOT a physics parameter) - a pure-P rate hold leaves a persistent residual body "
           f"rate under the large +-10deg aero moments this script measures, contaminating Clp/Cnp (aileron) "
           f"and Cmq/CLq (elevator) coupling terms; see module docstring for full empirical evidence.")
    qs_log("")

    channels = run_quasi_static_all(qs_log)

    any_nan_qs = any(p["any_nan"] for ch in channels.values() for p in ch["points"].values())
    any_limited = [p["case"] for ch in channels.values() for p in ch["points"].values()
                   if p["actuator_limited_response"]]
    any_smooth_fail = [p["case"] for ch in channels.values() for p in ch["points"].values() if not p["smooth_ok"]]

    qs_log("=" * 78)
    qs_log("QUASI-STATIC NUMERICAL INTEGRITY SUMMARY")
    qs_log("=" * 78)
    qs_log(f"any_nan across all 21 points: {any_nan_qs}")
    qs_log(f"ACTUATOR_LIMITED_RESPONSE flagged cases: {any_limited if any_limited else 'none'}")
    qs_log(f"smooth_ok failures: {any_smooth_fail if any_smooth_fail else 'none'}")

    with open(f"{RESULTS_DIR}/control_authority_quasi_static_result.json", "w") as f:
        json.dump(dict(
            trim_condition=dict(V=TRIM_V, alpha_deg=TRIM_ALPHA_DEG, throttle=TRIM_THROTTLE,
                                 elevator_theta_deg=TRIM_ELEV_THETA_DEG, elevator_aero_deg=TRIM_ELEV_AERO_DEG),
            settle_steps=SETTLE_STEPS, tail_steps=TAIL_STEPS, warm_steps=WARM_STEPS,
            channels=channels, any_nan=any_nan_qs, actuator_limited_response_cases=any_limited,
            smooth_ok_failures=any_smooth_fail,
        ), f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/control_authority_quasi_static_log.txt", "w") as f:
        f.write("\n".join(qs_log_lines) + "\n")

    ff_log("FALCON V2 - CONTROL_AUTHORITY_EFFECTIVENESS_VALIDATION - FREE_FLIGHT (gazebo-testing, 2026-08-26)")
    ff_log(f"Base trim (reused): throttle={TRIM_THROTTLE} elevator_theta=+{TRIM_ELEV_THETA_DEG}deg both sides, "
           f"V={TRIM_V} alpha={TRIM_ALPHA_DEG}deg, altitude={ALTITUDE_M}m. HOLD_STEPS={HOLD_STEPS} "
           f"RELEASE_STEPS={RELEASE_STEPS} ({RELEASE_STEPS*AL.STEP:.1f}s free window)")
    ff_log("")

    ff_cases = run_free_flight_all(ff_log)
    any_nan_ff = any(c["any_nan"] for c in ff_cases.values())

    ff_log("=" * 78)
    ff_log("FREE-FLIGHT NUMERICAL INTEGRITY SUMMARY")
    ff_log("=" * 78)
    ff_log(f"any_nan across all 6 free-flight runs: {any_nan_ff}")

    with open(f"{RESULTS_DIR}/control_authority_free_flight_result.json", "w") as f:
        json.dump(dict(trim_condition=dict(V=TRIM_V, alpha_deg=TRIM_ALPHA_DEG, throttle=TRIM_THROTTLE,
                                            elevator_theta_deg=TRIM_ELEV_THETA_DEG),
                        hold_steps=HOLD_STEPS, release_steps=RELEASE_STEPS,
                        cases=ff_cases, any_nan=any_nan_ff),
                  f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/control_authority_free_flight_log.txt", "w") as f:
        f.write("\n".join(ff_log_lines) + "\n")

    overall_ok = (not any_nan_qs) and (not any_nan_ff)
    return overall_ok


if __name__ == "__main__":
    ok = main()
    sys.exit(0 if ok else 1)

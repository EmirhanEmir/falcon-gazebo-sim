#!/usr/bin/env python3
"""
FALCON V2 - ENGINE_OUT_AND_ASYMMETRIC_THRUST_VALIDATION (gazebo-testing, 2026-08-27).

Validates the CURRENT twin-motor asymmetric-thrust / engine-out behavior of
the already-implemented model (propulsion + aerodynamics + actuators). Does
NOT retune propulsion constants, aero coefficients, control lookup tables,
rudder derivatives, actuator parameters, CG, inertia, or mass to make
engine-out flight "work" - if a genuine model limit blocks single-engine
flight, that is reported as a valid, classified result (CLAUDE.md hard
constraint / this task's own failure policy).

Context (read, not re-derived): docs/test_results/
2026-08-27_flight_envelope_validation.md (envelope/trim reference points),
docs/test_results/2026-08-26_updated_powered_trim_high_deflection_validation.md
(the currently-validated symmetric trim: throttle=0.5010, elevator physical
+4.50deg L/R, V=18.166 m/s, alpha=2.473deg).

=============================================================================
PRE-REGISTERED SIGN PREDICTIONS (derived analytically BEFORE any live test,
from propulsion_v1_config.yaml geometry + PropulsionSystem.cc's own
AddWorldWrench(force, torque, hub_offset) call - verified against live
measurement in PART 0 below, not re-derived here):

Force F = (thrust, 0, 0) body frame (thrust_axis_body=[1,0,0], CONFIRMED) at
hub offset r=(rx,ry,rz) relative to base_link's <inertial><pose> (CG) -
gz-sim's AddWorldWrench resolves the force+offset argument to a moment about
the link's OWN center of mass (confirmed by the pre-existing DZ_HUB_CG=
hub_z-CG_z=0.0271 convention already used by test_updated_powered_trim_high_
deflection.py / test_flight_envelope.py's own analytical trim solver, not
re-derived here) while the explicit `torque` argument is a separate, offset-
independent pure couple (see AddWorldWrench's own gz/sim/Link.hh doc comment):
  r x F = (ry*Fz - rz*Fy, rz*Fx - rx*Fz, rx*Fy - ry*Fx)
  F=(Fx,0,0) => (r x F) = (0, rz_eff*Fx, -ry_eff*Fx)
  => Mx_rF = 0 (thrust along +X never produces a roll moment via r x F)
     My_rF = rz_eff*Fx   (rz_eff = hub_z - CG_z = DZ_HUB_CG = 0.0271 m, same for both hubs)
     Mz_rF = -ry_eff*Fx  (ry_eff = hub_y - CG_y = hub_y exactly, CG_y=0)
  Mz_prop_total = -ry_L*Fx_L - ry_R*Fx_R = 0.30*(Fx_R - Fx_L)
    (matches the ALREADY-VALIDATED DIFFERENTIAL_THRUST_MOMENT_TEST result,
    "Mz ~= 0.300 x (T_right - T_left)", PROPULSION_V1_IMPLEMENTATION test
    report 2026-08-23 - cross-checked, not re-derived, in PART 0 below.)
  LEFT ENGINE OUT (right motor alone, Fx_L=0): Mz_prop = +0.30*Fx_R > 0
    => POSITIVE Mz => nose LEFT (toward the dead/left engine, matches the
    standard multi-engine-aviation fact).
  RIGHT ENGINE OUT (left motor alone, Fx_R=0): Mz_prop = -0.30*Fx_L < 0
    => NEGATIVE Mz => nose RIGHT (toward the dead/right engine). Mirror-
    antisymmetric, as expected.

  Reaction torque (separate, offset-independent couple, body +X axis only,
  PropulsionSystem.cc: reactionTorqueBodyX = rotationSign*(-qPropSigned_Nm),
  qPropSigned_Nm > 0 during normal forward-thrust operation for BOTH motors
  in their own frame):
    Mx_reaction_total = rotSignL*(-Qprop_L) + rotSignR*(-Qprop_R)
  Symmetric (both motors on): rotSignL=+1, rotSignR=-1, Qprop_L=Qprop_R =>
    Mx_reaction = -Qprop + Qprop = 0 (already validated,
    COUNTER_ROTATION_CANCELLATION_TEST, PROPULSION_V1_IMPLEMENTATION).
  LEFT ENGINE OUT (right motor alone, rotSignR=-1): Mx_reaction =
    rotSignR*(-Qprop_R) = (-1)*(-Qprop_R) = +Qprop_R > 0 => POSITIVE Mx =>
    per this project's "+X rotation -> LEFT wingtip UP" FLU convention,
    roll tendency TOWARD THE RIGHT (left wing up).
  RIGHT ENGINE OUT (left motor alone, rotSignL=+1): Mx_reaction =
    rotSignL*(-Qprop_L) = -Qprop_L < 0 => NEGATIVE Mx => roll tendency
    TOWARD THE LEFT (left wing down). Mirror-antisymmetric.

  Rudder compensation direction (already-verified rudder_sign=+1, delta_r_aero
  = theta_rudder; live fact from CONTROL_SURFACE_SIGN_MAPPING:
  theta_rudder=+8deg -> Mz=-0.446 N*m, nose-right): to oppose LEFT-ENGINE-
  OUT's positive/nose-left Mz_prop, need a NEGATIVE aero Mz => POSITIVE
  rudder deflection (matches aviation's "rudder toward the good/operating
  engine" - the operating engine here is the right one, and positive rudder
  yaws nose-right). To oppose RIGHT-ENGINE-OUT's negative/nose-right Mz_prop,
  need a POSITIVE aero Mz => NEGATIVE rudder deflection.
=============================================================================

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
control lookup table, rudder derivative, actuator parameter, motor/prop
constant, hub geometry, rotation-sign convention) is read for any purpose
other than loading the existing config/state, and NONE is modified anywhere
in this script.

Reuses (does not reinvent): test_flight_envelope.py (imported as FE, which
itself chains test_updated_powered_trim_high_deflection.py as PREV) for
ACT/AL/PL/sim/gm/REF/get_model()/quat_rpy()/actual_deltas()/predict_aero(),
PCFG/APC_SLICES/steady_state_rpm() (propulsion config + pure-Python rotor-ODE
mirror + steady-state throttle/thrust solver), and the established quasi-
static-hold / free-6DOF-flight / real-actuator-commanding techniques.
"""
import json
import math
import sys

import test_flight_envelope as FE

ACT = FE.ACT
AL = FE.AL
PL = FE.PL
sim = FE.sim
gm = FE.gm
REF = FE.REF
PREV = FE.PREV
get_model = FE.get_model
quat_rpy = FE.quat_rpy
actual_deltas = FE.actual_deltas
predict_aero = FE.predict_aero

REPO_ROOT = FE.REPO_ROOT
RESULTS_DIR = FE.RESULTS_DIR
WORLD = FE.WORLD
MASS = FE.MASS
I_DIAG = FE.I_DIAG
KP_LIN = FE.KP_LIN
KP_ANG_SETTLE = FE.KP_ANG_SETTLE
KP_ANG_QSTATIC = FE.KP_ANG_QSTATIC
ALTITUDE_M = FE.ALTITUDE_M
WEIGHT_N = FE.WEIGHT_N
DZ_HUB_CG = FE.DZ_HUB_CG
DIAG_HZ = FE.DIAG_HZ

PCFG = FE.PCFG
APC_SLICES = FE.APC_SLICES
steady_state_rpm = FE.steady_state_rpm

# =============================================================================
# Reference trim (2026-08-27 UPDATED_POWERED_TRIM_AND_HIGH_DEFLECTION_FLIGHT_
# VALIDATION - NOT re-searched here, per task instruction).
# =============================================================================
U_HOLD, W_HOLD = PREV.U_HOLD, PREV.W_HOLD  # V=18.162 m/s, alpha=2.472deg hold baseline
TRIM_THROTTLE = 0.5010
TRIM_ELEV_THETA_DEG = 4.50  # physical, L=R (elevator_aero=-4.50deg)
V_NOMINAL = 18.166

# Hub geometry / rotation signs (propulsion_v1_config.yaml, CONFIRMED/DERIVED - read only).
RY_L, RY_R = PCFG.left_hub_m[1], PCFG.right_hub_m[1]
RZ_L, RZ_R = PCFG.left_hub_m[2], PCFG.right_hub_m[2]
ROT_SIGN_L, ROT_SIGN_R = PCFG.rotation_left_sign, PCFG.rotation_right_sign
RPM_CAP = PCFG.rpm_cap_v1
CURRENT_LIMIT = PCFG.current_limit_A

SIDES = ("LEFT_OUT", "RIGHT_OUT")


def analytic_prop_moments(thrust_L, thrust_R, qprop_signed_L, qprop_signed_R):
    """Independent r x F + reaction-torque analytic recomputation, mirroring
    PropulsionSystem.cc::StepMotor()'s single AddWorldWrench(force, torque,
    hub_offset) call EXACTLY (see module docstring for the full derivation).
    Uses DZ_HUB_CG (hub_z - CG_z) for My since AddWorldWrench's offset
    argument resolves to a moment about the link's CENTER OF MASS, not its
    SDF link origin (CG_y=0 so this correction is a no-op for Mz)."""
    mz_rF = -RY_L * thrust_L - RY_R * thrust_R
    my_rF = DZ_HUB_CG * thrust_L + DZ_HUB_CG * thrust_R
    mx_reaction = ROT_SIGN_L * (-qprop_signed_L) + ROT_SIGN_R * (-qprop_signed_R)
    return dict(Mz_prop=mz_rF, My_prop_rF=my_rF, Mx_prop_reaction=mx_reaction)


def mirror_prop_load(rpm_live, v_axial):
    """Fully independent (not the live plugin's own internal Ct/Cp
    interpolation) recomputation of thrust_N/qPropSigned_Nm from a LIVE-
    MEASURED rpm + axial airspeed, via propulsion_lib.py's pure-Python
    mirror of PropulsionModel.hh (same APC table, independently re-parsed -
    see propulsion_lib.py's own module docstring). Used to cross-check the
    live diagnostics thrust/Q_prop values before trusting them, per this
    task's explicit "do not just trust a script's Python mirror, cross-check
    against live diagnostics" instruction (applied here in the OTHER
    direction: cross-check the live value against an independent mirror)."""
    omega = rpm_live * 2.0 * math.pi / 60.0
    load = PL.prop_aero_load(APC_SLICES, omega, v_axial, PCFG.diameter_m,
                              PCFG.rho, PCFG.n_safe_floor_rev_s)
    return load


def side_labels(side):
    """Returns (failed_letter, survivor_letter, rot_sign_survivor, ry_survivor)."""
    if side == "LEFT_OUT":
        return "left", "right", ROT_SIGN_R, RY_R
    return "right", "left", ROT_SIGN_L, RY_L


# =============================================================================
# PART 0 - sign pre-registration cross-check (pure analytic, no Gazebo) using
# the CONFIRMED config values loaded above - just confirms the module
# docstring's own arithmetic is self-consistent before any live run.
# =============================================================================
def run_part0(log):
    log("=" * 78)
    log("PART 0: ANALYTIC SIGN PRE-REGISTRATION CROSS-CHECK (no Gazebo)")
    log("=" * 78)
    log(f"left_hub_m={PCFG.left_hub_m} right_hub_m={PCFG.right_hub_m} "
        f"thrust_axis_body={PCFG.thrust_axis_body}")
    log(f"rotation_left_sign={ROT_SIGN_L} rotation_right_sign={ROT_SIGN_R}")
    log(f"DZ_HUB_CG (hub_z - CG_z) = {DZ_HUB_CG} m (established convention, PREV/FE)")
    # symmetric case cancellation
    ss = steady_state_rpm(TRIM_THROTTLE, U_HOLD)
    mirror = mirror_prop_load(ss["rpm"], U_HOLD)
    sym = analytic_prop_moments(ss["thrust_N"], ss["thrust_N"],
                                 mirror["qPropSigned_Nm"], mirror["qPropSigned_Nm"])
    log(f"SYMMETRIC (both @ throttle={TRIM_THROTTLE}): thrust_each={ss['thrust_N']:.4f}N "
        f"Qprop_each={mirror['qPropSigned_Nm']:.5f}Nm -> Mz_prop={sym['Mz_prop']:+.6f} "
        f"Mx_reaction={sym['Mx_prop_reaction']:+.6f} (expect both ~0)")
    # left-out (right alone)
    left_out = analytic_prop_moments(0.0, ss["thrust_N"], 0.0, mirror["qPropSigned_Nm"])
    log(f"LEFT_OUT (right alone @ throttle={TRIM_THROTTLE}): Mz_prop={left_out['Mz_prop']:+.6f} "
        f"(expect POSITIVE=nose-left) Mx_reaction={left_out['Mx_prop_reaction']:+.6f} "
        f"(expect POSITIVE=roll right/left-wing-up)")
    right_out = analytic_prop_moments(ss["thrust_N"], 0.0, mirror["qPropSigned_Nm"], 0.0)
    log(f"RIGHT_OUT (left alone @ throttle={TRIM_THROTTLE}): Mz_prop={right_out['Mz_prop']:+.6f} "
        f"(expect NEGATIVE=nose-right) Mx_reaction={right_out['Mx_prop_reaction']:+.6f} "
        f"(expect NEGATIVE=roll left/left-wing-down)")
    checks = dict(
        symmetric_mz_zero=abs(sym["Mz_prop"]) < 1e-9,
        symmetric_mx_zero=abs(sym["Mx_prop_reaction"]) < 1e-9,
        left_out_mz_positive=left_out["Mz_prop"] > 0,
        left_out_mx_positive=left_out["Mx_prop_reaction"] > 0,
        right_out_mz_negative=right_out["Mz_prop"] < 0,
        right_out_mx_negative=right_out["Mx_prop_reaction"] < 0,
        mirror_antisymmetric_mz=abs(left_out["Mz_prop"] + right_out["Mz_prop"]) < 1e-9,
        mirror_antisymmetric_mx=abs(left_out["Mx_prop_reaction"] + right_out["Mx_prop_reaction"]) < 1e-9,
    )
    all_ok = all(checks.values())
    log(f"PRE-REGISTRATION SELF-CONSISTENCY: {'ALL PASS' if all_ok else 'MISMATCH -> ' + str(checks)}")
    log("")
    return dict(symmetric=sym, left_out=left_out, right_out=right_out, checks=checks, all_ok=all_ok)


# =============================================================================
# PART 1 - static/controlled differential-thrust test (rudder neutral).
# =============================================================================
WARM_STEPS = 300
SETTLE_STEPS = 1500
TAIL_STEPS = 300


def run_actuator_hold(log, label, throttle_L, throttle_R, rudder_deg=0.0,
                       elev_theta_deg=TRIM_ELEV_THETA_DEG, aile_L_deg=0.0, aile_R_deg=0.0,
                       u_hold=U_HOLD, w_hold=W_HOLD,
                       warm_steps=WARM_STEPS, settle_steps=SETTLE_STEPS, tail_steps=TAIL_STEPS,
                       verbose=True):
    """Quasi-static hold (body linear velocity pinned at (u_hold,0,w_hold),
    angular rate pinned at (0,0,0) via AL.hold_step's real force/torque
    controller - the SAME primitive the aero/propulsion plugins themselves
    use) with INDEPENDENT left/right throttle commanded through the real
    PL.ThrottleCommander and all 5 control surfaces commanded through the
    real ACT.ActuatorCommander. Generalizes test_flight_envelope.py's
    run_actuator_hold_generic() (which only supports symmetric throttle) to
    independent per-side throttle for asymmetric-thrust/engine-out testing -
    no new methodology invented, same technique."""
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
        state["thr"].set(left=throttle_L, right=throttle_R)
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

    def pavg(side, field):
        vals = [p[side][field] for p in prop_tail]
        return sum(vals) / len(vals) if vals else None

    left_rpm, right_rpm = pavg("left", "rpm"), pavg("right", "rpm")
    left_thrust, right_thrust = pavg("left", "thrust_N"), pavg("right", "thrust_N")
    left_current, right_current = pavg("left", "current_A"), pavg("right", "current_A")
    left_qprop, right_qprop = pavg("left", "Q_prop_Nm"), pavg("right", "Q_prop_Nm")
    any_rpm_cap = any((p["left"]["rpmCapActive"] > 0.5 or p["right"]["rpmCapActive"] > 0.5) for p in prop_tail) if prop_tail else False
    any_current_limited = any((p["left"]["currentLimited"] > 0.5 or p["right"]["currentLimited"] > 0.5) for p in prop_tail) if prop_tail else False

    ad = state["actuator_diag"].latest() if state["actuator_diag"] else None
    any_target_clamp = any(ad[s]["target_clamp_active"] > 0.5 for s in ACT.SURFACES) if ad else False
    any_effort_clamp = any(ad[s]["effort_clamp_active"] > 0.5 for s in ACT.SURFACES) if ad else False
    tracking_err_deg = ({s: math.degrees(abs(ad[s]["setpoint_rad"] - ad[s]["actual_angle_rad"])) for s in ACT.SURFACES}
                         if ad else {})

    # ---- propulsion Mz/Mx: LIVE-STATE value (r x F fed the LIVE thrust_N the
    # plugin itself computed/applied) AND an INDEPENDENT mirror recomputation
    # (propulsion_lib.py's own Ct/Cp interpolation fed the LIVE rpm + the
    # measured axial body velocity u_m) - never just trusting one or the other.
    live_prop_moments = analytic_prop_moments(left_thrust or 0.0, right_thrust or 0.0,
                                               left_qprop or 0.0, right_qprop or 0.0)
    mirror_left = mirror_prop_load(left_rpm or 0.0, u_m)
    mirror_right = mirror_prop_load(right_rpm or 0.0, u_m)
    mirror_prop_moments = analytic_prop_moments(mirror_left["thrust_N"], mirror_right["thrust_N"],
                                                 mirror_left["qPropSigned_Nm"], mirror_right["qPropSigned_Nm"])

    pred = predict_aero(REF, u_m, v_m, w_m, p_m, q_m, r_m,
                         deltaA=actual_delta_a, deltaE=actual_delta_e, deltaR=actual_delta_r)
    aero_mz_live = ((aero_avg["qbar"] * REF["S"] * REF["b"] * aero_avg["Cn"]) if aero_avg["Cn"] is not None else None)
    aero_mx_live = ((aero_avg["qbar"] * REF["S"] * REF["b"] * aero_avg["Cl"]) if aero_avg["Cl"] is not None else None)
    aero_my_live = None
    if aero_avg["Cm"] is not None and aero_avg["V"] is not None:
        vSafe = max(aero_avg["V"], REF["vSafeFloor"])
        qHat_live = q_m * REF["c_ref"] / (2.0 * vSafe)
        cmRate_live = REF["Cmq"] * qHat_live
        cmStatic_live = aero_avg["Cm"] - cmRate_live
        aero_my_live = aero_avg["qbar"] * REF["S"] * REF["c_ref"] * (-cmStatic_live + cmRate_live)

    mz_total_live = (live_prop_moments["Mz_prop"] + aero_mz_live) if aero_mz_live is not None else None
    mx_total_live = (live_prop_moments["Mx_prop_reaction"] + aero_mx_live) if aero_mx_live is not None else None
    my_total_live = (live_prop_moments["My_prop_rF"] + aero_my_live) if aero_my_live is not None else None

    result = dict(
        label=label, throttle_L=throttle_L, throttle_R=throttle_R,
        elev_theta_cmd_deg=elev_theta_deg, aile_L_cmd_deg=aile_L_deg, aile_R_cmd_deg=aile_R_deg,
        rudder_cmd_deg=rudder_deg,
        actual_delta_e_deg=math.degrees(actual_delta_e), actual_delta_a_deg=math.degrees(actual_delta_a),
        actual_delta_r_deg=math.degrees(actual_delta_r),
        actual_theta_deg={s: math.degrees(tail_mean_rad[s]) for s in ACT.SURFACES},
        aero_tail_avg=aero_avg, aero_tail_n_msgs=len(aero_tail),
        prop_tail=dict(left_rpm=left_rpm, right_rpm=right_rpm, left_thrust_N=left_thrust, right_thrust_N=right_thrust,
                       left_current_A=left_current, right_current_A=right_current,
                       left_Qprop_Nm=left_qprop, right_Qprop_Nm=right_qprop,
                       any_rpm_cap=any_rpm_cap, any_current_limited=any_current_limited),
        thrust_total_N=((left_thrust or 0.0) + (right_thrust or 0.0)),
        live_prop_moments=live_prop_moments, mirror_prop_moments=mirror_prop_moments,
        mirror_thrust_L=mirror_left["thrust_N"], mirror_thrust_R=mirror_right["thrust_N"],
        aero_mz_live=aero_mz_live, aero_mx_live=aero_mx_live, aero_my_live=aero_my_live,
        mz_total_live=mz_total_live, mx_total_live=mx_total_live, my_total_live=my_total_live,
        pred_from_measured_state=pred,
        any_target_clamp=any_target_clamp, any_effort_clamp=any_effort_clamp,
        tracking_err_deg=tracking_err_deg, any_nan=state["any_nan"],
        body_state_tail=dict(u=u_m, v=v_m, w=w_m, p=p_m, q=q_m, r=r_m),
    )
    if verbose:
        log(f"  [{label}] thr(L/R)={throttle_L:.4f}/{throttle_R:.4f} rudder={rudder_deg:+.2f}deg "
            f"elev={elev_theta_deg:+.2f}deg aile(L/R)={aile_L_deg:+.2f}/{aile_R_deg:+.2f}deg")
        log(f"    RPM(L/R)={left_rpm:.1f}/{right_rpm:.1f} Thrust(L/R)={left_thrust:.4f}/{right_thrust:.4f}N "
            f"(total={result['thrust_total_N']:.4f}N) Current(L/R)={left_current:.2f}/{right_current:.2f}A "
            f"rpm_cap={any_rpm_cap} cur_lim={any_current_limited}")
        log(f"    Mz_prop: LIVE-STATE(r x F, live thrust)={live_prop_moments['Mz_prop']:+.5f}Nm | "
            f"INDEPENDENT-MIRROR(r x F, mirror thrust from live rpm)={mirror_prop_moments['Mz_prop']:+.5f}Nm "
            f"(mirror thrust L/R={mirror_left['thrust_N']:.4f}/{mirror_right['thrust_N']:.4f}N)")
        log(f"    Mx_reaction: LIVE-STATE={live_prop_moments['Mx_prop_reaction']:+.5f}Nm | "
            f"INDEPENDENT-MIRROR={mirror_prop_moments['Mx_prop_reaction']:+.5f}Nm")
        log(f"    aero(live): CY={aero_avg['CY']} Cl={aero_avg['Cl']} Cn={aero_avg['Cn']} qbar={aero_avg['qbar']}")
        log(f"    Mz_total(prop_live+aero_live)={mz_total_live} Mx_total={mx_total_live} My_total={my_total_live}")
        log(f"    tracking_err_deg={tracking_err_deg} any_target_clamp={any_target_clamp} "
            f"any_effort_clamp={any_effort_clamp} any_nan={state['any_nan']}")
    return result


def run_part1(log):
    log("=" * 78)
    log("PART 1: STATIC/CONTROLLED DIFFERENTIAL-THRUST TEST (rudder neutral)")
    log("=" * 78)
    cases = [
        ("A_BASELINE", TRIM_THROTTLE, TRIM_THROTTLE),
        ("B_L40_R50", 0.40, 0.50),
        ("C_L50_R40", 0.50, 0.40),
        ("D_L25_R50", 0.25, 0.50),
        ("E_L50_R25", 0.50, 0.25),
        ("F_LEFT_OUT", 0.0, TRIM_THROTTLE),
        ("G_RIGHT_OUT", TRIM_THROTTLE, 0.0),
    ]
    results = {}
    for name, tl, tr in cases:
        log("-" * 60)
        r = run_actuator_hold(log, name, tl, tr, rudder_deg=0.0)
        results[name] = r

    any_nan_overall = any(r["any_nan"] for r in results.values())

    log("")
    log("Left/right mirror symmetry checks:")

    def sym_check(name_pos, name_neg):
        a, b = results[name_pos], results[name_neg]
        mz_a, mz_b = a["live_prop_moments"]["Mz_prop"], b["live_prop_moments"]["Mz_prop"]
        sum_ = mz_a + mz_b
        rel = abs(sum_) / max(abs(mz_a), abs(mz_b), 1e-9)
        log(f"  {name_pos} Mz={mz_a:+.5f} vs {name_neg} Mz={mz_b:+.5f} : sum={sum_:+.6f} "
            f"(rel to larger magnitude={rel*100:.3f}%) {'OK' if rel < 0.05 else 'WATCH'}")
        return dict(a=name_pos, b=name_neg, mz_a=mz_a, mz_b=mz_b, sum=sum_, rel=rel, ok=rel < 0.05)

    symmetry = dict(
        BC=sym_check("B_L40_R50", "C_L50_R40"),
        DE=sym_check("D_L25_R50", "E_L50_R25"),
        FG=sym_check("F_LEFT_OUT", "G_RIGHT_OUT"),
    )
    log("")
    return dict(cases=results, symmetry=symmetry, any_nan_overall=any_nan_overall)


# =============================================================================
# PART 2 - zero-rudder engine-out TRANSIENT response (real motor spool-down,
# throttle command to 0, NOT an instant RPM zero - the rotor ODE integrates
# the decay naturally through I_rotor*domega/dt = Q_motor - Q_prop with
# Q_motor -> Kt*(0 - I0) once throttle=0, i.e. a real, finite-time decay).
# =============================================================================
TR_HOLD_STEPS = 800
TR_RELEASE_STEPS = 5000  # 5s
TR_TELEMETRY_EVERY = 20  # 50 Hz


def run_transient_engine_out(log, side):
    failed, survivor, _, _ = side_labels(side)
    elev_theta_rad = math.radians(TRIM_ELEV_THETA_DEG)
    cmd_rad = dict(left_elevator=elev_theta_rad, right_elevator=elev_theta_rad,
                   left_aileron=0.0, right_aileron=0.0, rudder=0.0)

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
        # Throttle: BOTH nominal until release, then the FAILED side commands
        # 0 while the survivor RETAINS its operating throttle - a real
        # command-level step (not an omega/RPM override) so the rotor ODE
        # supplies the actual finite spool-down.
        if n < TR_HOLD_STEPS:
            state["thr"].set(left=TRIM_THROTTLE, right=TRIM_THROTTLE)
        else:
            if failed == "left":
                state["thr"].set(left=0.0, right=TRIM_THROTTLE)
            else:
                state["thr"].set(left=TRIM_THROTTLE, right=0.0)
        state["thr"].tick()
        state["cmd"].set(**cmd_rad)
        state["cmd"].tick()
        if n < TR_HOLD_STEPS:
            AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(U_HOLD, 0, W_HOLD), gm.Vector3d(0, 0, 0),
                         kp_lin=KP_LIN, kp_ang=KP_ANG_SETTLE)
        # else: base_link COMPLETELY free from the moment of engine failure onward.

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
        if n >= TR_HOLD_STEPS and (n - TR_HOLD_STEPS) % TR_TELEMETRY_EVERY == 0:
            roll, pitch, yaw = quat_rpy(rot)
            alpha = math.atan2(-lv_b.z(), lv_b.x())
            beta = math.atan2(lv_b.y(), math.hypot(lv_b.x(), lv_b.z()))
            prop = state["prop_diag"].latest() if state["prop_diag"] else None
            aero = state["aero_diag"].latest() if state["aero_diag"] else None
            V = math.sqrt(lv_b.x() ** 2 + lv_b.y() ** 2 + lv_b.z() ** 2)
            left_thrust = prop["left"]["thrust_N"] if prop else None
            right_thrust = prop["right"]["thrust_N"] if prop else None
            left_qprop = prop["left"]["Q_prop_Nm"] if prop else None
            right_qprop = prop["right"]["Q_prop_Nm"] if prop else None
            mzp = mxr = None
            if None not in (left_thrust, right_thrust, left_qprop, right_qprop):
                mm = analytic_prop_moments(left_thrust, right_thrust, left_qprop, right_qprop)
                mzp, mxr = mm["Mz_prop"], mm["Mx_prop_reaction"]
            aero_mz = (aero["qbar"] * REF["S"] * REF["b"] * aero["Cn"]) if aero else None
            state["series"].append(dict(
                t=(n - TR_HOLD_STEPS) * AL.STEP, V=V, alt=wpose.pos().z(),
                roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch), yaw_deg=math.degrees(yaw),
                alpha_deg=math.degrees(alpha), beta_deg=math.degrees(beta),
                p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z()),
                left_rpm=(prop["left"]["rpm"] if prop else None), right_rpm=(prop["right"]["rpm"] if prop else None),
                left_thrust_N=left_thrust, right_thrust_N=right_thrust,
                left_current_A=(prop["left"]["current_A"] if prop else None),
                right_current_A=(prop["right"]["current_A"] if prop else None),
                Mz_prop=mzp, Mx_reaction=mxr, aero_Mz=aero_mz))
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, TR_HOLD_STEPS + TR_RELEASE_STEPS, False)

    series = state["series"]
    start, end = series[0], series[-1]
    max_abs_r = max(abs(s["r_deg_s"]) for s in series)
    max_abs_p = max(abs(s["p_deg_s"]) for s in series)
    max_abs_roll = max(abs(s["roll_deg"] - start["roll_deg"]) for s in series)
    max_abs_yaw = max(abs(s["yaw_deg"] - start["yaw_deg"]) for s in series)

    # Smooth-decay check: no instantaneous RPM jump (bounded per-sample delta).
    failed_rpm_key = "left_rpm" if failed == "left" else "right_rpm"
    rpm_series = [s[failed_rpm_key] for s in series if s[failed_rpm_key] is not None]
    max_step_drop = max((rpm_series[i] - rpm_series[i + 1] for i in range(len(rpm_series) - 1)), default=0.0)
    smooth_decay = max_step_drop < (rpm_series[0] * 0.5 if rpm_series else 1.0)  # no single-sample >50% collapse

    log(f"--- {side} TRANSIENT (failed={failed}, survivor={survivor} retains throttle={TRIM_THROTTLE}) ---")
    log(f"any_nan={state['any_nan']} (first at step {state['nan_step']})")
    log(f"Failed-motor RPM: t=0 {rpm_series[0] if rpm_series else None:.1f} -> "
        f"t={series[-1]['t']:.2f}s {rpm_series[-1] if rpm_series else None:.1f} "
        f"(max single-sample drop={max_step_drop:.2f} RPM, smooth_decay={smooth_decay})")
    log(f"Yaw: start={start['yaw_deg']:+.3f} end={end['yaw_deg']:+.3f} (drift toward "
        f"{'LEFT(+)' if (end['yaw_deg']-start['yaw_deg'])>0 else 'RIGHT(-)'}), max|r|={max_abs_r:.3f}deg/s")
    log(f"Roll: start={start['roll_deg']:+.3f} end={end['roll_deg']:+.3f} max|roll_drift|={max_abs_roll:.3f}deg, "
        f"max|p|={max_abs_p:.3f}deg/s")
    log(f"Beta: start={start['beta_deg']:+.3f} end={end['beta_deg']:+.3f}")
    log(f"Early Mz_prop (t=0+)={series[0]['Mz_prop']} Mx_reaction(t=0+)={series[0]['Mx_reaction']}")
    log("")
    # PERSISTED decimated raw RPM time-series (2026-08-27 validation follow-up
    # fix, `gazebo-testing`): the full per-tick series is already sampled at
    # TR_TELEMETRY_EVERY=20 ticks (50 Hz, well within the requested 10-50
    # tick decimation band) - kept here as a TOP-LEVEL key (not nested inside
    # "series") so it survives strip_series()'s pop("series") when the caller
    # persists this dict to the result JSON, addressing `validation`'s finding
    # that the original artifact only kept summary stats (start/end/max-drop),
    # not a re-verifiable raw trace, for the "smooth decay, not instantaneous"
    # claim.
    rpm_decimated = [dict(t=s["t"], left_rpm=s["left_rpm"], right_rpm=s["right_rpm"],
                           left_current_A=s["left_current_A"], right_current_A=s["right_current_A"])
                      for s in series]
    return dict(side=side, any_nan=state["any_nan"], nan_step=state["nan_step"], series=series,
                rpm_decimated=rpm_decimated, rpm_decimated_sample_period_s=TR_TELEMETRY_EVERY * AL.STEP,
                summary=dict(max_abs_r_deg_s=max_abs_r, max_abs_p_deg_s=max_abs_p,
                             max_abs_roll_drift_deg=max_abs_roll, max_abs_yaw_drift_deg=max_abs_yaw,
                             yaw_drift_deg=end["yaw_deg"] - start["yaw_deg"],
                             roll_drift_deg=end["roll_deg"] - start["roll_deg"],
                             smooth_decay=smooth_decay, max_single_sample_rpm_drop=max_step_drop))


# =============================================================================
# PART 3/4 - rudder compensation search per engine-out side.
# =============================================================================
def rudder_candidates(side):
    # Pre-registered direction: LEFT_OUT needs POSITIVE rudder, RIGHT_OUT
    # needs NEGATIVE rudder (see module docstring).
    mags = [5.0, 10.0, 15.0, 25.0]
    sign = 1.0 if side == "LEFT_OUT" else -1.0
    return [sign * m for m in mags]


def run_rudder_search(log, side, throttle_survivor, u_hold=U_HOLD, w_hold=W_HOLD,
                       elev_theta_deg=TRIM_ELEV_THETA_DEG, label_prefix="",
                       settle_steps=SETTLE_STEPS, tail_steps=TAIL_STEPS):
    failed, survivor, _, _ = side_labels(side)
    tl = 0.0 if failed == "left" else throttle_survivor
    tr = throttle_survivor if failed == "left" else 0.0

    candidates = rudder_candidates(side)
    points = []
    for rud in candidates:
        r = run_actuator_hold(log, f"{label_prefix}{side}_RUD{rud:+.0f}", tl, tr, rudder_deg=rud,
                               elev_theta_deg=elev_theta_deg, u_hold=u_hold, w_hold=w_hold,
                               settle_steps=settle_steps, tail_steps=tail_steps, verbose=False)
        mz_total = r["mz_total_live"]
        log(f"  rudder={rud:+.1f}deg -> actual_delta_r={r['actual_delta_r_deg']:+.3f}deg "
            f"Mz_prop={r['live_prop_moments']['Mz_prop']:+.5f} aero_Mz={r['aero_mz_live']:+.5f} "
            f"Mz_total={mz_total:+.5f}")
        points.append(dict(rudder_deg=rud, result=r, mz_total=mz_total))

    # Refine: linear-interpolate between the bracketing pair with opposite-sign
    # residual closest to zero, then confirm with 1-2 live points near there.
    points.sort(key=lambda p: p["rudder_deg"])
    bracket = None
    for i in range(len(points) - 1):
        m0, m1 = points[i]["mz_total"], points[i + 1]["mz_total"]
        if m0 == 0 or (m0 < 0) != (m1 < 0):
            bracket = (points[i], points[i + 1])
            break
    refine_points = []
    if bracket is not None:
        (p0, p1) = bracket
        r0, r1 = p0["rudder_deg"], p1["rudder_deg"]
        m0, m1 = p0["mz_total"], p1["mz_total"]
        r_interp = r0 - m0 * (r1 - r0) / (m1 - m0) if (m1 - m0) != 0 else 0.5 * (r0 + r1)
        r_test = round(r_interp, 1)
        rr = run_actuator_hold(log, f"{label_prefix}{side}_RUD{r_test:+.1f}_REFINE", tl, tr, rudder_deg=r_test,
                               elev_theta_deg=elev_theta_deg, u_hold=u_hold, w_hold=w_hold,
                               settle_steps=settle_steps, tail_steps=tail_steps, verbose=False)
        log(f"  REFINE rudder={r_test:+.2f}deg (interpolated between {r0:+.0f}/{r1:+.0f}) -> "
            f"Mz_total={rr['mz_total_live']:+.5f}")
        refine_points.append(dict(rudder_deg=r_test, result=rr, mz_total=rr["mz_total_live"]))
        all_points = points + refine_points
    else:
        all_points = points
        log("  No sign-crossing bracket found among the 4 candidates - reporting the smallest-|residual| candidate.")

    best = min(all_points, key=lambda p: abs(p["mz_total"]))
    log(f"  BEST for {side}: rudder={best['rudder_deg']:+.2f}deg Mz_total_residual={best['mz_total']:+.5f}Nm")
    log("")
    return dict(side=side, throttle_survivor=throttle_survivor, candidates=points, refine=refine_points,
                best=best)


# =============================================================================
# PART 5 - surviving-engine throttle adequacy.
# =============================================================================
def run_throttle_adequacy(log, side, u_hold=U_HOLD, w_hold=W_HOLD, V_nominal=V_NOMINAL,
                           elev_theta_deg=TRIM_ELEV_THETA_DEG, rudder_deg=0.0):
    failed, survivor, _, _ = side_labels(side)
    log(f"--- {side}: surviving-engine ({survivor}) throttle adequacy ---")

    # Step 1: baseline @ TRIM_THROTTLE (already measured in Part 1 F/G, but
    # recomputed here standalone so this Part is self-contained/reusable per
    # speed in Part 8).
    tl = 0.0 if failed == "left" else TRIM_THROTTLE
    tr = TRIM_THROTTLE if failed == "left" else 0.0
    base = run_actuator_hold(log, f"{side}_THR_BASELINE", tl, tr, rudder_deg=rudder_deg,
                             elev_theta_deg=elev_theta_deg, u_hold=u_hold, w_hold=w_hold, verbose=False)
    drag_N = base["aero_tail_avg"]["qbar"] * REF["S"] * base["aero_tail_avg"]["CD"]
    thrust_baseline = base["thrust_total_N"]
    log(f"  baseline throttle={TRIM_THROTTLE}: thrust_total={thrust_baseline:.4f}N vs Drag={drag_N:.4f}N "
        f"(T/D={thrust_baseline/drag_N:.4f})")

    classification = None
    required_throttle = TRIM_THROTTLE
    if thrust_baseline >= drag_N:
        log("  Baseline survivor throttle ALREADY sufficient to balance drag - no increase needed.")
        required_throttle = TRIM_THROTTLE
        confirm = base
        classification = "SUFFICIENT_AT_BASELINE"
    else:
        # Analytic (pure-Python, no Gazebo) bisection for the throttle that
        # makes the SURVIVOR ALONE match the drag target - fast offline
        # search, mirrors test_flight_envelope.py's own
        # solve_throttle_for_thrust() pattern (that one assumes 2 motors;
        # here only 1 motor is thrusting).
        def f(th):
            return steady_state_rpm(th, u_hold)["thrust_N"] - drag_N
        lo, hi = 0.05, 1.0
        flo, fhi = f(lo), f(hi)
        if fhi < 0.0:
            required_throttle = hi
            classification = "PROPULSION_LIMITED"
        else:
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                fm = f(mid)
                if flo * fm <= 0.0:
                    hi = mid
                else:
                    lo, flo = mid, fm
            required_throttle = 0.5 * (lo + hi)
            classification = "OK"
        ss = steady_state_rpm(required_throttle, u_hold)
        log(f"  Analytic (offline, pure-Python bisection) required throttle={required_throttle:.4f} "
            f"-> predicted thrust={ss['thrust_N']:.4f}N RPM={ss['rpm']:.1f} current={ss['current_A']:.2f}A "
            f"(rpm_cap={RPM_CAP}, current_limit={CURRENT_LIMIT})")
        tl2 = 0.0 if failed == "left" else required_throttle
        tr2 = required_throttle if failed == "left" else 0.0
        confirm = run_actuator_hold(log, f"{side}_THR_CONFIRM_{required_throttle:.3f}", tl2, tr2,
                                    rudder_deg=rudder_deg, elev_theta_deg=elev_theta_deg,
                                    u_hold=u_hold, w_hold=w_hold, verbose=False)
        thrust_confirm = confirm["thrust_total_N"]
        log(f"  LIVE confirm @ throttle={required_throttle:.4f}: thrust_total={thrust_confirm:.4f}N "
            f"(target Drag={drag_N:.4f}N, T/D={thrust_confirm/drag_N:.4f}) "
            f"RPM={confirm['prop_tail']['left_rpm'] if failed=='right' else confirm['prop_tail']['right_rpm']:.1f} "
            f"current={confirm['prop_tail']['left_current_A'] if failed=='right' else confirm['prop_tail']['right_current_A']:.2f}A "
            f"rpm_cap={confirm['prop_tail']['any_rpm_cap']} cur_lim={confirm['prop_tail']['any_current_limited']}")
        if confirm["prop_tail"]["any_rpm_cap"] or confirm["prop_tail"]["any_current_limited"]:
            classification = "PROPULSION_LIMITED"
    log("")
    return dict(side=side, drag_N=drag_N, thrust_baseline_N=thrust_baseline, required_throttle=required_throttle,
                classification=classification, baseline=base, confirm=confirm)


# =============================================================================
# PART 6 - single-engine trim attempt (quasi-static hold at V_nominal).
# =============================================================================
def run_single_engine_trim(log, side, throttle_survivor, rudder_deg, elev_theta_deg=TRIM_ELEV_THETA_DEG,
                            u_hold=U_HOLD, w_hold=W_HOLD, aile_L_deg=0.0, aile_R_deg=0.0):
    failed, survivor, _, _ = side_labels(side)
    tl = 0.0 if failed == "left" else throttle_survivor
    tr = throttle_survivor if failed == "left" else 0.0
    log(f"--- {side}: single-engine trim attempt (throttle={throttle_survivor:.4f}, "
        f"rudder={rudder_deg:+.2f}deg, elevator={elev_theta_deg:+.2f}deg, aileron L/R={aile_L_deg:+.2f}/{aile_R_deg:+.2f}deg) ---")
    r = run_actuator_hold(log, f"{side}_TRIM_ATTEMPT", tl, tr, rudder_deg=rudder_deg,
                          elev_theta_deg=elev_theta_deg, aile_L_deg=aile_L_deg, aile_R_deg=aile_R_deg,
                          u_hold=u_hold, w_hold=w_hold, verbose=False)
    lift = r["aero_tail_avg"]["qbar"] * REF["S"] * r["aero_tail_avg"]["CL"]
    drag = r["aero_tail_avg"]["qbar"] * REF["S"] * r["aero_tail_avg"]["CD"]
    lw_ratio = lift / WEIGHT_N
    td_ratio = r["thrust_total_N"] / drag if drag else float("nan")
    log(f"  Lift={lift:.3f}N Weight={WEIGHT_N:.3f}N ratio={lw_ratio:.4f} | Thrust={r['thrust_total_N']:.3f}N "
        f"Drag={drag:.3f}N T/D={td_ratio:.4f}")
    log(f"  My_total={r['my_total_live']:+.4f}Nm Mz_total={r['mz_total_live']:+.4f}Nm "
        f"Mx_total={r['mx_total_live']:+.4f}Nm")
    log("")
    return dict(side=side, hold=r, lift_N=lift, drag_N=drag, lift_weight_ratio=lw_ratio, thrust_drag_ratio=td_ratio)


# =============================================================================
# PART 7 - single-engine free 6-DOF flight (~10-15s).
# =============================================================================
SE_FF_HOLD_STEPS = 800
SE_FF_RELEASE_STEPS = 13000  # 13s
SE_FF_TELEMETRY_EVERY = 100


def run_single_engine_free_flight(log, side, throttle_survivor, rudder_deg, elev_theta_deg=TRIM_ELEV_THETA_DEG,
                                   aile_L_deg=0.0, aile_R_deg=0.0, u_hold=U_HOLD, w_hold=W_HOLD,
                                   hold_steps=SE_FF_HOLD_STEPS, release_steps=SE_FF_RELEASE_STEPS,
                                   telemetry_every=SE_FF_TELEMETRY_EVERY):
    """hold_steps/release_steps/telemetry_every are overridable (2026-08-27
    validation follow-up fix, `gazebo-testing`) so the SAME technique can be
    re-run at an extended duration (e.g. ~30s, matching FLIGHT_ENVELOPE_
    VALIDATION's free-flight precedent) without duplicating this function -
    the original ~13s Part 7 call sites are unaffected (defaults unchanged)."""
    failed, survivor, _, _ = side_labels(side)
    cmd_rad = dict(left_elevator=math.radians(elev_theta_deg), right_elevator=math.radians(elev_theta_deg),
                   left_aileron=math.radians(aile_L_deg), right_aileron=math.radians(aile_R_deg),
                   rudder=math.radians(rudder_deg))
    tl = 0.0 if failed == "left" else throttle_survivor
    tr = throttle_survivor if failed == "left" else 0.0

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
        state["thr"].set(left=tl, right=tr)
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
            da, de, dr, _ = actual_deltas(model, ecm)
            state["series"].append(dict(
                t=(n - hold_steps) * AL.STEP, V=V, alt=wpose.pos().z(), world_vz=lv.z(),
                roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch), yaw_deg=math.degrees(yaw),
                alpha_deg=math.degrees(alpha), beta_deg=math.degrees(beta),
                p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z()),
                actual_delta_e_deg=math.degrees(de), actual_delta_a_deg=math.degrees(da), actual_delta_r_deg=math.degrees(dr),
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
    max_abs_roll = max(abs(s["roll_deg"]) for s in series)
    max_abs_pitch = max(abs(s["pitch_deg"]) for s in series)
    max_abs_yaw_drift = max(abs(s["yaw_deg"] - start["yaw_deg"]) for s in series)
    max_abs_beta = max(abs(s["beta_deg"]) for s in series)
    v_drift = end["V"] - start["V"]
    alt_drift = end["alt"] - start["alt"]

    log(f"--- {side} SINGLE-ENGINE FREE FLIGHT (throttle_survivor={throttle_survivor:.4f}, "
        f"rudder={rudder_deg:+.2f}deg) ---")
    log(f"any_nan={state['any_nan']} (first at step {state['nan_step']})")
    log(f"Airspeed: start={start['V']:.3f} end={end['V']:.3f} drift={v_drift:+.3f} m/s")
    log(f"Altitude: start={start['alt']:.2f} end={end['alt']:.2f} drift={alt_drift:+.2f} m")
    log(f"Roll: start={start['roll_deg']:+.2f} end={end['roll_deg']:+.2f} max|roll|={max_abs_roll:.2f}deg")
    log(f"Pitch: start={start['pitch_deg']:+.2f} end={end['pitch_deg']:+.2f} max|pitch|={max_abs_pitch:.2f}deg")
    log(f"Yaw drift: max|yaw-start|={max_abs_yaw_drift:.2f}deg")
    log(f"Beta: start={start['beta_deg']:+.2f} end={end['beta_deg']:+.2f} max|beta|={max_abs_beta:.2f}deg")

    classification = "UNABLE_TO_MAINTAIN_ENGINE_OUT_FLIGHT"
    if state["any_nan"] or max_abs_roll > 60.0 or max_abs_pitch > 45.0 or abs(v_drift) > 6.0 or abs(alt_drift) > 60.0:
        classification = "UNABLE_TO_MAINTAIN_ENGINE_OUT_FLIGHT"
    elif max_abs_roll > 20.0 or max_abs_yaw_drift > 20.0 or abs(v_drift) > 1.5 or abs(alt_drift) > 20.0:
        classification = "PASS_WITH_DRIFT"
    else:
        classification = "PASS"
    log(f"CLASSIFICATION: {classification}\n")

    # PLATEAU-vs-CONTINUED-GROWTH trend analysis (2026-08-27 validation
    # follow-up fix, `gazebo-testing`): `validation` correctly found that the
    # original ~13s window showed roll/yaw STILL AT their window-max at the
    # very end, with no persisted evidence of leveling off - "bounded through
    # 13s" is not the same claim as "bounded".
    #
    # CORRECTED METHODOLOGY (self-caught bug during this same follow-up
    # pass): a first version of this analysis finite-differenced the Euler
    # `yaw_deg` ANGLE directly across the mid/last thirds. That is WRONG for
    # yaw specifically: (1) `yaw_deg` wraps at +/-180 deg (atan2-based), so a
    # window straddling a wrap (observed directly in this run's own raw
    # series, e.g. -179.57deg -> +162.96deg between consecutive telemetry
    # samples) produces a spurious, arbitrarily large finite-difference "rate"
    # with no physical meaning; (2) even ignoring wrap, ANY sustained nonzero
    # yaw rate (e.g. a steady, bounded, banked turn) makes the yaw ANGLE grow
    # without bound forever BY DEFINITION - that is normal, expected turning
    # flight, not evidence of divergence, so "is the yaw angle still
    # growing" is the wrong question to ask at all. The physically correct
    # question is "is the yaw RATE (r, the body angular rate - never wraps)
    # still growing in magnitude (a real, developing spiral) or has it leveled
    # off at a roughly steady value (a bounded, steady turn/bank)?" - this is
    # answered directly from the already-recorded, unwrapped `p_deg_s`/
    # `r_deg_s` body rate fields, by comparing their MEAN over the last third
    # of the window vs the middle third.
    def thirds_mean_rate(rate_key):
        n = len(series)
        i_mid_start, i_last_start = n // 3, (2 * n) // 3
        mid = series[i_mid_start:i_last_start]
        last = series[i_last_start:]
        if len(mid) < 2 or len(last) < 2:
            return None, None, None
        m_mid = sum(s[rate_key] for s in mid) / len(mid)
        m_last = sum(s[rate_key] for s in last) / len(last)
        ratio = (abs(m_last) / abs(m_mid)) if abs(m_mid) > 1e-9 else (float("inf") if abs(m_last) > 1e-9 else 0.0)
        return m_mid, m_last, ratio

    roll_rate_mid, roll_rate_last, roll_rate_ratio = thirds_mean_rate("p_deg_s")
    yaw_rate_mid, yaw_rate_last, yaw_rate_ratio = thirds_mean_rate("r_deg_s")
    # Three-way bucket (a ratio near 1.0 - the rate has leveled off at a
    # roughly constant NONZERO value - is a genuinely different, non-
    # divergent outcome from a ratio >>1 - the rate ITSELF is still growing
    # in magnitude, a real ongoing/developing divergence):
    #   ratio < 0.6         -> rate decaying toward zero (angle itself settling to a fixed value)
    #   0.6 <= ratio < 1.3  -> rate has leveled off at a steady nonzero value (bounded steady-state, e.g. a steady turn/bank - NOT divergence)
    #   ratio >= 1.3        -> rate magnitude still growing (a real, still-developing divergence)
    # FLOOR-AWARE (self-caught during this same follow-up pass, same
    # near-zero-crossing-produces-a-meaningless-ratio pattern this suite has
    # already had to handle elsewhere, e.g. test_updated_powered_trim_high_
    # deflection.py's PAIRED_TOL abs-floor / test_high_deflection_control_
    # aero.py's TOL dict): when BOTH mid/last means are already tiny in
    # absolute terms (well below any operationally meaningful rate for this
    # airframe), a ratio computed from them is dominated by noise/sign-flip-
    # near-zero, not a real trend - reported as its own bucket instead of a
    # possibly-misleading ratio-driven label.
    RATE_FLOOR_DEG_S = 1.0

    def trend_label(m_mid, m_last, ratio):
        if ratio is None:
            return "INSUFFICIENT_DATA"
        if abs(m_mid) < RATE_FLOOR_DEG_S and abs(m_last) < RATE_FLOOR_DEG_S:
            return "NEGLIGIBLE_RATE_SETTLED"
        if ratio < 0.6:
            return "RATE_DECAYING_TOWARD_ZERO"
        if ratio >= 1.3:
            return "RATE_MAGNITUDE_STILL_GROWING"
        return "RATE_LEVELED_OFF_STEADY_NONZERO"

    trend = dict(
        roll_rate_mid_third_deg_s=roll_rate_mid, roll_rate_last_third_deg_s=roll_rate_last,
        roll_rate_ratio_last_over_mid=roll_rate_ratio,
        roll_trend=trend_label(roll_rate_mid, roll_rate_last, roll_rate_ratio),
        yaw_rate_mid_third_deg_s=yaw_rate_mid, yaw_rate_last_third_deg_s=yaw_rate_last,
        yaw_rate_ratio_last_over_mid=yaw_rate_ratio,
        yaw_trend=trend_label(yaw_rate_mid, yaw_rate_last, yaw_rate_ratio),
        roll_at_window_end_is_window_max=(abs(end["roll_deg"]) >= 0.999 * max_abs_roll),
        roll_mean_mid_third_deg=(sum(s["roll_deg"] for s in series[len(series)//3:(2*len(series))//3]) /
                                  max(1, len(series[len(series)//3:(2*len(series))//3]))),
        roll_mean_last_third_deg=(sum(s["roll_deg"] for s in series[(2*len(series))//3:]) /
                                   max(1, len(series[(2*len(series))//3:]))),
    )
    log(f"TREND (mean BODY RATE p/r over last-third vs mid-third of the window - NOT the wrap-prone Euler "
        f"yaw ANGLE, see corrected-methodology comment above; window={series[-1]['t']:.1f}s): "
        f"roll_rate(p) mid={roll_rate_mid:+.3f}deg/s last={roll_rate_last:+.3f}deg/s "
        f"ratio={roll_rate_ratio:.3f} -> {trend['roll_trend']} | "
        f"yaw_rate(r) mid={yaw_rate_mid:+.3f}deg/s last={yaw_rate_last:+.3f}deg/s "
        f"ratio={yaw_rate_ratio:.3f} -> {trend['yaw_trend']}")
    log(f"Roll ANGLE mean: mid-third={trend['roll_mean_mid_third_deg']:+.2f}deg "
        f"last-third={trend['roll_mean_last_third_deg']:+.2f}deg (bank angle settling check) | "
        f"window-end roll ({end['roll_deg']:+.2f}deg) still the window max ({max_abs_roll:.2f}deg): "
        f"{trend['roll_at_window_end_is_window_max']}\n")

    return dict(side=side, any_nan=state["any_nan"], nan_step=state["nan_step"], series=series,
                summary=dict(v_drift=v_drift, alt_drift=alt_drift, max_abs_roll=max_abs_roll,
                             max_abs_pitch=max_abs_pitch, max_abs_yaw_drift=max_abs_yaw_drift,
                             max_abs_beta=max_abs_beta),
                classification=classification, trend=trend,
                window_s=series[-1]["t"], end_roll_deg=end["roll_deg"], end_yaw_deg=end["yaw_deg"])


def strip_series(obj):
    out = dict(obj)
    out["series_n"] = len(obj.get("series", []))
    out.pop("series", None)
    return out


# =============================================================================
# Orchestration
# =============================================================================
def main():
    log_lines = []

    def log(msg):
        print(msg, flush=True)
        log_lines.append(msg)

    log("FALCON V2 - ENGINE_OUT_AND_ASYMMETRIC_THRUST_VALIDATION (gazebo-testing, 2026-08-27)")
    log(f"World: {WORLD}")
    log(f"Reference trim: throttle=L=R={TRIM_THROTTLE}, elevator physical +{TRIM_ELEV_THETA_DEG}deg L/R, "
        f"V={V_NOMINAL} m/s (reused, not re-searched)")
    log("")

    part0 = run_part0(log)
    part1 = run_part1(log)
    part2 = {side: run_transient_engine_out(log, side) for side in SIDES}

    log("=" * 78)
    log("PART 5: SURVIVING-ENGINE THROTTLE ADEQUACY (rudder neutral, needed to pick a")
    log("        realistic survivor throttle for Parts 3/4/6/7 - task order: run before")
    log("        the rudder search is FINALIZED so the search reflects a sustainable throttle)")
    log("=" * 78)
    part5 = {side: run_throttle_adequacy(log, side) for side in SIDES}

    log("=" * 78)
    log("PART 3/4: RUDDER COMPENSATION SEARCH (at each side's Part-5 survivor throttle)")
    log("=" * 78)
    part34 = {}
    for side in SIDES:
        log(f"-- {side} --")
        part34[side] = run_rudder_search(log, side, part5[side]["required_throttle"])

    log("=" * 78)
    log("PART 4 REPORT TABLE")
    log("=" * 78)
    for side in SIDES:
        best = part34[side]["best"]
        r = best["result"]
        log(f"{side}: survivor_throttle={part5[side]['required_throttle']:.4f} "
            f"survivor_thrust={r['thrust_total_N']:.4f}N Mz_prop={r['live_prop_moments']['Mz_prop']:+.5f}Nm "
            f"required_rudder_physical={best['rudder_deg']:+.2f}deg actual_rudder_aero={r['actual_delta_r_deg']:+.3f}deg "
            f"aero_Mz={r['aero_mz_live']:+.5f}Nm residual_Mz_total={best['mz_total']:+.5f}Nm")
    fg = part34["LEFT_OUT"]["best"], part34["RIGHT_OUT"]["best"]
    mirror_rudder_sum = fg[0]["rudder_deg"] + fg[1]["rudder_deg"]
    mirror_mz_prop_sum = fg[0]["result"]["live_prop_moments"]["Mz_prop"] + fg[1]["result"]["live_prop_moments"]["Mz_prop"]
    log(f"Mirror symmetry: rudder sum (should be ~0, opposite sign)={mirror_rudder_sum:+.3f}deg, "
        f"Mz_prop sum (should be ~0)={mirror_mz_prop_sum:+.5f}Nm")
    log("")

    log("=" * 78)
    log("PART 6: SINGLE-ENGINE TRIM ATTEMPT")
    log("=" * 78)
    part6 = {}
    for side in SIDES:
        best_rudder = part34[side]["best"]["rudder_deg"]
        r = run_single_engine_trim(log, side, part5[side]["required_throttle"], best_rudder)
        mx_mag = abs(r["hold"]["mx_total_live"])
        log(f"  Roll-moment check (aileron-neutral attempt): |Mx_total|={mx_mag:.4f}Nm - for reference, "
            f"~5deg aileron authority is on the order of several N*m at this qbar (CONTROL_AUTHORITY_"
            f"EFFECTIVENESS_VALIDATION, 2026-08-26), so this residual is judged SMALL relative to aileron "
            f"authority; aileron kept neutral (rudder-only attempt sufficient, per task instruction to only "
            f"add aileron if genuinely needed).")
        part6[side] = r
    log("")

    log("=" * 78)
    log("PART 7: SINGLE-ENGINE FREE 6-DOF FLIGHT")
    log("=" * 78)
    part7 = {}
    for side in SIDES:
        best_rudder = part34[side]["best"]["rudder_deg"]
        part7[side] = run_single_engine_free_flight(log, side, part5[side]["required_throttle"], best_rudder)

    log("=" * 78)
    log("PART 8: SPEED DEPENDENCE (16 and 24 m/s, rudder-compensation-requirement only)")
    log("=" * 78)
    part8 = {}
    try:
        with open(f"{RESULTS_DIR}/flight_envelope_result.json") as f:
            fe_result = json.load(f)
        ac = fe_result["actuator_confirms"]
        for V_key, V_label in (("16.0", 16.0), ("24.0", 24.0)):
            c = ac[V_key]
            rb = fe_result["results_by_speed"][V_key]
            u_h, w_h = rb["u_hold"], rb["w_hold"]
            elev_deg = c["elev_theta_cmd_deg"]
            drag_N = c["aero_tail_avg"]["qbar"] * REF["S"] * c["aero_tail_avg"]["CD"]
            log(f"-- V={V_label} m/s (from flight_envelope_result.json, read-only, not re-searched): "
                f"two-engine throttle={c['throttle']:.4f} elevator={elev_deg:+.3f}deg Drag(two-engine "
                f"reference)={drag_N:.4f}N --")
            part8_speed = {}
            for side in SIDES:
                def f_single(th):
                    return steady_state_rpm(th, u_h)["thrust_N"] - drag_N
                lo, hi = 0.05, 1.0
                flo, fhi = f_single(lo), f_single(hi)
                if fhi < 0.0:
                    req_th = hi
                else:
                    for _ in range(40):
                        mid = 0.5 * (lo + hi)
                        fm = f_single(mid)
                        if flo * fm <= 0.0:
                            hi = mid
                        else:
                            lo, flo = mid, fm
                    req_th = 0.5 * (lo + hi)
                ss = steady_state_rpm(req_th, u_h)
                log(f"   {side}: analytic survivor throttle={req_th:.4f} -> thrust={ss['thrust_N']:.4f}N "
                    f"RPM={ss['rpm']:.1f} current={ss['current_A']:.2f}A")
                search = run_rudder_search(log, side, req_th, u_hold=u_h, w_hold=w_h, elev_theta_deg=elev_deg,
                                           label_prefix=f"V{V_label}_", settle_steps=1200, tail_steps=250)
                part8_speed[side] = dict(required_throttle=req_th, search=search)
            part8[V_label] = part8_speed
    except FileNotFoundError:
        log("  flight_envelope_result.json not found - Part 8 skipped.")
    log("")

    log("=" * 78)
    log("PART 9: RUDDER YAW AUTHORITY MARGIN")
    log("=" * 78)
    part9 = {}
    for side in SIDES:
        mz_prop = part34[side]["best"]["result"]["live_prop_moments"]["Mz_prop"]
        margins = {}
        for p in part34[side]["candidates"]:
            deg = abs(p["rudder_deg"])
            aero_mz = abs(p["result"]["aero_mz_live"])
            margins[f"{deg:.0f}deg"] = aero_mz / abs(mz_prop) if mz_prop else None
        low_flag = abs(part34[side]["best"]["rudder_deg"]) > 25.0
        sat_flag = abs(part34[side]["best"]["rudder_deg"]) >= 45.0
        log(f"{side}: Mz_prop(engine-out)={mz_prop:+.5f}Nm, required_rudder={part34[side]['best']['rudder_deg']:+.2f}deg, "
            f"margins(|aero_Mz(delta)|/|Mz_prop|)={margins}, LOW_RUDDER_MARGIN={low_flag}, "
            f"RUDDER_SATURATION_RISK={sat_flag}")
        part9[side] = dict(mz_prop=mz_prop, margins=margins, low_rudder_margin=low_flag,
                            rudder_saturation_risk=sat_flag)
    if part8:
        for V_label, speed_data in part8.items():
            for side, d in speed_data.items():
                search = d["search"]
                mz_prop = search["best"]["result"]["live_prop_moments"]["Mz_prop"]
                margins = {f"{abs(p['rudder_deg']):.0f}deg": abs(p["result"]["aero_mz_live"]) / abs(mz_prop) if mz_prop else None
                           for p in search["candidates"]}
                low_flag = abs(search["best"]["rudder_deg"]) > 25.0
                sat_flag = abs(search["best"]["rudder_deg"]) >= 45.0
                key = f"{side}@{V_label}"
                log(f"{key}: Mz_prop={mz_prop:+.5f}Nm required_rudder={search['best']['rudder_deg']:+.2f}deg "
                    f"margins={margins} LOW_RUDDER_MARGIN={low_flag} RUDDER_SATURATION_RISK={sat_flag}")
                part9[key] = dict(mz_prop=mz_prop, margins=margins, low_rudder_margin=low_flag,
                                   rudder_saturation_risk=sat_flag)
    log("")

    log("=" * 78)
    log("PART 10: ACTUATOR/NUMERICAL SAFETY SUMMARY")
    log("=" * 78)
    any_nan_overall = (
        part1["any_nan_overall"]
        or any(part2[s]["any_nan"] for s in SIDES)
        or any(part7[s]["any_nan"] for s in SIDES)
        or any(p["result"]["any_nan"] for s in SIDES for p in part34[s]["candidates"])
    )
    any_clamp_overall = any(
        r["any_target_clamp"] or r["any_effort_clamp"]
        for name, r in part1["cases"].items()
    )
    smooth_decay_overall = all(part2[s]["summary"]["smooth_decay"] for s in SIDES)
    log(f"any_nan_overall={any_nan_overall}")
    log(f"any_target_clamp/effort_clamp seen in Part 1 static cases: {any_clamp_overall}")
    log(f"smooth (non-instantaneous) RPM decay confirmed both sides: {smooth_decay_overall}")
    part10 = dict(any_nan_overall=any_nan_overall, any_clamp_overall=any_clamp_overall,
                  smooth_decay_overall=smooth_decay_overall)
    log("")

    # ---------------- Save results ----------------
    part1_out = dict(cases=part1["cases"], symmetry=part1["symmetry"], any_nan_overall=part1["any_nan_overall"])
    part2_out = {s: strip_series(part2[s]) for s in SIDES}
    part34_out = {s: dict(throttle_survivor=part34[s]["throttle_survivor"],
                           candidates=part34[s]["candidates"], refine=part34[s]["refine"],
                           best=part34[s]["best"]) for s in SIDES}
    part7_out = {s: strip_series(part7[s]) for s in SIDES}
    part8_out = {str(V): {s: dict(required_throttle=d["required_throttle"],
                                    search=dict(throttle_survivor=d["search"]["throttle_survivor"],
                                                candidates=d["search"]["candidates"],
                                                refine=d["search"]["refine"], best=d["search"]["best"]))
                           for s, d in sd.items()} for V, sd in part8.items()}

    with open(f"{RESULTS_DIR}/engine_out_asymmetric_thrust_result.json", "w") as f:
        json.dump(dict(part0=part0, part1=part1_out, part2_transient=part2_out, part5_throttle_adequacy=part5,
                       part3_4_rudder_search=part34_out, part6_trim=part6, part7_free_flight=part7_out,
                       part8_speed_dependence=part8_out, part9_rudder_margin=part9, part10_safety=part10),
                  f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/engine_out_asymmetric_thrust_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    overall_ok = not any_nan_overall
    return overall_ok


def main_followup():
    """Targeted follow-up ONLY (`validation` review, 2026-08-27) - does NOT
    re-run Parts 1/3/4/5/6/8/9/10 or the original ~13s Part 7 (loaded
    read-only from the existing result JSON instead). Addresses two findings:

    1. MAJOR: Part 7's original ~13s single-engine free-flight window ended
       with roll/yaw STILL AT their window-max, with no persisted evidence of
       leveling off - "bounded through 13s" was not the same claim as
       "bounded". Re-runs BOTH sides at the SAME nominal-speed condition
       (same survivor throttle, same rudder-compensation value, aileron
       neutral, no controller) for ~30s (matching FLIGHT_ENVELOPE_
       VALIDATION's free-flight duration precedent) and reports the
       plateau-vs-continued-growth trend analysis added to
       run_single_engine_free_flight() above.
    2. MINOR/process: persists a decimated raw RPM time-series for the Part 2
       engine-out transient (both sides) so the "smooth decay, not
       instantaneous" claim can be independently re-verified from the result
       JSON alone, not just from a printed summary statistic. Part 2 is cheap
       (~5s sim time per side) so this is a fresh, quick re-run of ONLY that
       part, not a re-derivation of anything already concluded.

    No aircraft physics parameter is modified. Existing part1/part3_4/part5/
    part6/part8/part9/part10 entries in the result JSON are read back
    verbatim (unchanged) and re-written as-is alongside the new data below -
    nothing already concluded is silently dropped from the persisted file.
    """
    log_lines = []

    def log(msg):
        print(msg, flush=True)
        log_lines.append(msg)

    result_path = f"{RESULTS_DIR}/engine_out_asymmetric_thrust_result.json"
    with open(result_path) as f:
        prior = json.load(f)

    log("FALCON V2 - ENGINE_OUT_AND_ASYMMETRIC_THRUST_VALIDATION - VALIDATION FOLLOW-UP (gazebo-testing, 2026-08-27)")
    log("Targeted re-run: (1) Part 2 decimated-RPM persistence (both sides, fresh quick re-run), "
        "(2) Part 7 extended ~30s single-engine free-flight (both sides, same conditions as the original run).")
    log("Parts 1/3/4/5/6/8/9/10 are NOT re-run - loaded read-only from the existing result JSON.")
    log("")

    # ---- (1) Part 2 decimated RPM persistence - fresh, quick re-run ----
    log("=" * 78)
    log("PART 2 RE-RUN (decimated RPM persistence only)")
    log("=" * 78)
    part2_fresh = {side: run_transient_engine_out(log, side) for side in SIDES}
    part2_out = {s: strip_series(part2_fresh[s]) for s in SIDES}

    # ---- (2) Part 7 extended ~30s free flight, same conditions as before ----
    log("=" * 78)
    log("PART 7 EXTENDED (~30s) SINGLE-ENGINE FREE 6-DOF FLIGHT")
    log("=" * 78)
    EXT_HOLD_STEPS = 800
    EXT_RELEASE_STEPS = 29200  # total 30.0s
    part7_extended = {}
    for side in SIDES:
        throttle_survivor = prior["part5_throttle_adequacy"][side]["required_throttle"]
        rudder_deg = prior["part3_4_rudder_search"][side]["best"]["rudder_deg"]
        log(f"-- {side}: throttle_survivor={throttle_survivor:.4f} (from existing Part 5 result, read-only), "
            f"rudder={rudder_deg:+.2f}deg (from existing Part 4 result, read-only) --")
        r = run_single_engine_free_flight(log, side, throttle_survivor, rudder_deg,
                                          hold_steps=EXT_HOLD_STEPS, release_steps=EXT_RELEASE_STEPS,
                                          telemetry_every=SE_FF_TELEMETRY_EVERY)
        part7_extended[side] = r

    log("=" * 78)
    log("SUMMARY: EXTENDED (~30s) vs ORIGINAL (~13s) Part 7 outcome")
    log("=" * 78)
    for side in SIDES:
        ext = part7_extended[side]
        orig = prior["part7_free_flight"][side]
        log(f"{side}: ORIGINAL(~{orig['summary']['max_abs_yaw_drift']:.0f}deg-yaw-window) end_roll="
            f"{orig.get('end_roll_deg', 'N/A')} end_yaw={orig.get('end_yaw_deg', 'N/A')} "
            f"classification={orig['classification']}")
        log(f"{side}: EXTENDED(~{ext['window_s']:.1f}s) end_roll={ext['end_roll_deg']:+.2f}deg "
            f"end_yaw={ext['end_yaw_deg']:+.2f}deg max|roll|={ext['summary']['max_abs_roll']:.2f}deg "
            f"max|yaw_drift|={ext['summary']['max_abs_yaw_drift']:.2f}deg classification={ext['classification']}")
        log(f"{side}: TREND VERDICT: roll(p) rate={ext['trend']['roll_trend']} "
            f"(mean-rate ratio last/mid={ext['trend']['roll_rate_ratio_last_over_mid']:.3f}, "
            f"bank angle mid-3rd={ext['trend']['roll_mean_mid_third_deg']:+.2f}deg -> "
            f"last-3rd={ext['trend']['roll_mean_last_third_deg']:+.2f}deg), "
            f"yaw(r) rate={ext['trend']['yaw_trend']} "
            f"(mean-rate ratio last/mid={ext['trend']['yaw_rate_ratio_last_over_mid']:.3f})")
        # Interpretation (see the corrected-methodology comment in
        # run_single_engine_free_flight() for the full reasoning): roll is
        # only "bounded" in the traditional sense if its RATE decays toward
        # zero (the bank ANGLE converges to a fixed value) - a steady nonzero
        # roll rate would mean a continuous, never-stopping roll, which is a
        # real problem. Yaw is different: ANY sustained nonzero turn rate
        # makes the yaw ANGLE grow forever by definition (ordinary turning
        # flight, not divergence) - so for yaw, EITHER the rate decaying to
        # zero (it stops turning) OR the rate leveling off at a steady
        # nonzero value (a steady, bounded turn/bank) both count as "not a
        # growing divergence"; only the rate's own MAGNITUDE still growing
        # is the real problem case for yaw too.
        roll_bounded = ext["trend"]["roll_trend"] in ("RATE_DECAYING_TOWARD_ZERO", "NEGLIGIBLE_RATE_SETTLED")
        roll_divergent = ext["trend"]["roll_trend"] in ("RATE_MAGNITUDE_STILL_GROWING", "RATE_LEVELED_OFF_STEADY_NONZERO")
        yaw_bounded = ext["trend"]["yaw_trend"] in ("RATE_DECAYING_TOWARD_ZERO", "RATE_LEVELED_OFF_STEADY_NONZERO",
                                                     "NEGLIGIBLE_RATE_SETTLED")
        yaw_divergent = ext["trend"]["yaw_trend"] == "RATE_MAGNITUDE_STILL_GROWING"
        if roll_bounded and yaw_bounded:
            overall_side_verdict = ("(a) BOUNDED: roll settles to a steady bank angle (roll rate decaying "
                                     "toward zero) and yaw settles into a steady, non-growing turn rate "
                                     "(or stops) - a genuinely bounded engine-out flight condition (a steady, "
                                     "mildly-banked turn under rudder-only compensation), not a divergence.")
        elif roll_divergent or yaw_divergent:
            overall_side_verdict = ("(b) CONTINUED GROWTH / NOT YET SETTLED: at least one axis's rate "
                                     "magnitude is still growing at 30s - a real, uncorrected slow "
                                     "roll/spiral divergence under rudder-only compensation.")
        else:
            overall_side_verdict = "AMBIGUOUS (not cleanly bucketed at 30s - see raw ratios above)"
        log(f"{side}: OVERALL 30s VERDICT: {overall_side_verdict}\n")

    part7_extended_out = {s: strip_series(part7_extended[s]) for s in SIDES}

    merged = dict(prior)
    merged["part2_transient"] = part2_out  # superseded in place: same methodology, now with decimated RPM persisted
    merged["part7_free_flight_extended_30s"] = part7_extended_out
    merged["followup_note"] = (
        "2026-08-27 validation follow-up: part2_transient re-run fresh (same technique, decimated RPM now "
        "persisted); part7_free_flight (original ~13s) left untouched; part7_free_flight_extended_30s added "
        "(same conditions, ~30s duration) to directly address the MAJOR finding that the original 13s window "
        "ended at its roll/yaw maximum with no evidence of leveling off.")

    with open(result_path, "w") as f:
        json.dump(merged, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/engine_out_asymmetric_thrust_log.txt", "a") as f:
        f.write("\n\n" + "=" * 78 + "\n")
        f.write("VALIDATION FOLLOW-UP RUN (appended, 2026-08-27)\n")
        f.write("=" * 78 + "\n")
        f.write("\n".join(log_lines) + "\n")

    any_nan = any(part2_fresh[s]["any_nan"] for s in SIDES) or any(part7_extended[s]["any_nan"] for s in SIDES)
    return not any_nan


if __name__ == "__main__":
    if "--followup" in sys.argv:
        ok = main_followup()
        sys.exit(0 if ok else 1)
    ok = main()
    sys.exit(0 if ok else 1)

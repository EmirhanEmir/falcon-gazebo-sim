#!/usr/bin/env python3
"""
FALCON V2 - ARDUPLANE_LONGITUDINAL_EQUILIBRIUM_AND_SINK_ROOT_CAUSE_VALIDATION
Steps 2/3 - pure-Gazebo controller-independent longitudinal-equilibrium harness
(gazebo-testing, 2026-08-28).

MEASURE-AND-PROVE stage. This script does NOT change any aero coefficient/table,
propulsion parameter, PID, config/ardupilot/falcon_v2_sitl.parm, actuator/sign
mapping, joint limit, plugin source, or any SDF. It measures the *physical*
longitudinal equilibrium of airframe + aerodynamics + propulsion with the
autopilot pitch/TECS loop entirely removed (PRIMARY = pure Gazebo, no ArduPlane -
Part A.0 / Option 1 of the controls-integration Step-2 spec), then runs a bounded
single-variable sweep (Part C.2) for the true steady-level point. Any discrepancy
is reported and routed, never tuned away (CLAUDE.md simulation-tuning policy).

Implements Part A.2 (pure-Gazebo harness), Part B.1 (2 s IC hold -> long free
window + damped-sinusoid fit), Part C.1 (reproduce the current reference point
verbatim), Part C.2 (iterative elevator->throttle->V sweep), Part D (required
logged quantities + steady-state acceptance table). Part C.3 (FBWA / MANUAL
cross-check) lives in the companion script
test_ardupilot_longitudinal_equilibrium_fbwa_crosscheck.py.

REUSED, UNMODIFIED, AS LIBRARIES (never edited):
  - aero_lib.py           : hold_step, DiagSubscriber, STEP, CONFIG_YAML_PATH
  - propulsion_lib.py     : ThrottleCommander, DiagSubscriber, pin_control_surface_joints, setup_env
  - test_updated_powered_trim_high_deflection.py : predict_aero, load_aero_ref / REF,
                            interp_lin, saturated_CL, quat_rpy  (the CURRENT AeroModel.hh
                            pure-Python mirror - NOT aero_lib.compute_aero, which is stale)

ENVIRONMENT (Part A.2): GZ_SIM_SYSTEM_PLUGIN_PATH is hard-set to
plugins/propulsion/build:plugins/aerodynamics/build ONLY, AFTER importing the
mirror module (whose own import side-effect would otherwise add the actuators
build dir). ardupilot_gazebo/build is deliberately NOT on the path -> the
ArduPilotPlugin fails to load (harmless), and there is genuinely no bridge
publishing to the command topics. wind/pitot/magnetometer/actuators also fail to
load - harmless, none affect airframe/aero/propulsion; the 5 control-surface
joints are held rigid by a direct-ECM kinematic pin (Part A.2 (i) primary).

Usage:
    python3 test_ardupilot_longitudinal_equilibrium_root_cause.py c1      # Part C.1 only
    python3 test_ardupilot_longitudinal_equilibrium_root_cause.py sweep   # Part C.2 only
    python3 test_ardupilot_longitudinal_equilibrium_root_cause.py all     # C.1 then C.2
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import aero_lib as AL          # noqa: E402
import propulsion_lib as PL    # noqa: E402

PL.setup_env()  # prop + aero on the path first

# Importing the mirror module runs actuator_lib.setup_env() as a side effect
# (prepends the actuators build dir). Import it, then HARD-RESET the plugin path
# back to prop+aero only, per Part A.2. This is runtime env configuration from
# this test script, never an edit to the imported module.
import test_updated_powered_trim_high_deflection as TR  # noqa: E402

os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{PL.PROP_PLUGIN_BUILD_DIR}:{PL.AERO_PLUGIN_BUILD_DIR}"
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import numpy as np             # noqa: E402
import gz.math7 as gm          # noqa: E402
import gz.sim8 as sim          # noqa: E402

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = f"{REPO_ROOT}/tests/gazebo/results"
WORLD = f"{REPO_ROOT}/tests/gazebo/worlds/falcon_v2_freefall_world.sdf"

REF = TR.REF                     # current AeroModel.hh config mirror (read-only)
predict_aero = TR.predict_aero
interp_lin = TR.interp_lin
quat_rpy = TR.quat_rpy

# ---- controller-gain-only constants (never fed into a physics computation) ----
MASS_CTRL = 5.9348              # kg, base_link mass (aero_lib.py) - controller gain only
I_DIAG = (0.7284, 0.2507, 0.9523)  # kg m^2, base_link Ixx/Iyy/Izz - controller gain only
KP_LIN = 150.0
KP_ANG = 400.0                  # safe here: 1 ms in-process loop, the value hold_step is proven at

# ---- reference/physical constants (documented provenance) ----
S_REF = REF["S"]               # 0.4514 m^2, aero_v1_config.yaml reference_geometry
RHO = REF["rho"]               # air density, aero_v1_config.yaml environment
G_WORLD = 9.81                 # m/s^2, falcon_v2_freefall_world.sdf <gravity>0 0 -9.81</gravity>
DZ_HUB_CG = 0.0271             # m, propulsion hub Z (0.1271) - CG Z (0.100000); fwd thrust above CG => nose-down dz*T
ELEV_SIGN = REF["elevatorSign"]   # -1.0 (aero_v1_config.yaml control_mapping)

LINK_NAMES = ["base_link", "left_aileron", "right_aileron", "left_elevator",
              "right_elevator", "rudder", "left_prop", "right_prop"]

# ---- Part C.1 reference point (controls-integration spec, verbatim) ----
C1_V = 18.166
C1_THROTTLE = 0.5010
C1_ELEV_PHYS_DEG = 4.50        # physical joint angle both halves; delta_e_aero = -4.50 deg

STEP = AL.STEP                 # 0.001 s
HOLD_STEPS = 2000              # 2.0 s IC hold (Part B)
C1_WINDOW_STEPS = 60000        # 60 s free window (Part C.1 / B.1)
SWEEP_WINDOW_STEPS = 30000     # 30 s free window for the bounded sweep runs (Part B: ">= 2 phugoid periods; use 30 s")
RECORD_EVERY = 10             # store body state at 100 Hz (NaN-guard is per-tick); diag/mirror sampled at DIAG cadence
DIAG_EVERY = 50              # 20 Hz mirror/diag capture (live aero/prop diag publish at 20 Hz)


def get_model(ecm):
    world = sim.World(sim.world_entity(ecm))
    return sim.Model(world.model_by_name(ecm, "falcon_v2"))


def sum_model_weight(model, ecm):
    """Query the ACTUAL summed model weight at run start: all links' runtime
    mass * g. Do NOT assume 6.000 kg."""
    total_mass = 0.0
    per_link = {}
    for ln in LINK_NAMES:
        le = model.link_by_name(ecm, ln)
        if le is None:
            per_link[ln] = None
            continue
        try:
            mm = sim.Link(le).world_inertial(ecm).mass_matrix()
            per_link[ln] = mm.mass()
            total_mass += mm.mass()
        except Exception:
            per_link[ln] = None
    return total_mass, total_mass * G_WORLD, per_link


def alpha_guess_deg_for(V, delta_e_aero_rad, weight_N=58.86):
    """IC AoA estimate: invert CL0 + CLa*alpha + dCL_e(delta_e) = W/(qbar S).
    Used ONLY to set the initial teleport pitch + hold target - the actual
    equilibrium is whatever the free window converges to."""
    qbar = 0.5 * RHO * V * V
    cl_req = weight_N / (qbar * S_REF)
    dcle = interp_lin(REF["bps"], REF["elev_dCL"], delta_e_aero_rad)
    alpha = (cl_req - REF["CL0"] - dcle) / REF["CLa"]
    return math.degrees(alpha)


# =============================================================================
# damped-sinusoid + linear-trend fit (Part B.1):
#   y(t) = A + B*t + exp(-sigma*t) * (Cc*cos(w*t) + Cs*sin(w*t))
# grid over (w, sigma), linear least squares for [A, B, Cc, Cs] at each node.
# Returns asymptotic offset A, non-oscillatory drift B, phugoid period 2*pi/w,
# damping ratio zeta = sigma / sqrt(sigma^2 + w^2).
# =============================================================================
def damped_sinusoid_fit(t, y):
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    if len(t) < 20:
        return None
    t = t - t[0]
    periods = np.linspace(4.0, 16.0, 121)          # phugoid period search 4..16 s
    ws = 2.0 * math.pi / periods
    sigmas = np.linspace(0.0, 0.6, 61)             # decay-rate search 0..0.6 1/s
    best = None
    for w in ws:
        cw = np.cos(w * t)
        sw = np.sin(w * t)
        for sig in sigmas:
            env = np.exp(-sig * t)
            X = np.column_stack([np.ones_like(t), t, env * cw, env * sw])
            coef, res, rank, _ = np.linalg.lstsq(X, y, rcond=None)
            pred = X @ coef
            sse = float(np.sum((y - pred) ** 2))
            if best is None or sse < best[0]:
                zeta = sig / math.sqrt(sig * sig + w * w) if (sig * sig + w * w) > 0 else 0.0
                best = (sse, dict(A=float(coef[0]), B=float(coef[1]),
                                  Cc=float(coef[2]), Cs=float(coef[3]),
                                  amp0=float(math.hypot(coef[2], coef[3])),
                                  omega=float(w), period_s=float(2.0 * math.pi / w),
                                  sigma=float(sig), zeta=float(zeta),
                                  sse=sse, rmse=float(math.sqrt(sse / len(y)))))
    return best[1]


def linreg_slope(t, y):
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    if len(t) < 2:
        return None
    A = np.column_stack([t, np.ones_like(t)])
    m, _ = np.linalg.lstsq(A, y, rcond=None)[0]
    return float(m)


def stat(vals):
    a = np.asarray([v for v in vals if v is not None and math.isfinite(v)], float)
    if a.size == 0:
        return None
    return dict(mean=float(a.mean()), std=float(a.std()), min=float(a.min()),
               max=float(a.max()), n=int(a.size))


# =============================================================================
# One pure-Gazebo equilibrium run (Part A.2 + B.1).
# =============================================================================
def run_point(label, V_target, throttle, elevator_phys_deg, window_steps,
              hold_steps=HOLD_STEPS, alpha_guess_deg=None, raw_decimate=1):
    E = math.radians(elevator_phys_deg)                 # physical joint angle, both halves
    delta_e_aero = 0.5 * ELEV_SIGN * (E + E)            # = -E for elevator_sign=-1.0
    if alpha_guess_deg is None:
        alpha_guess_deg = alpha_guess_deg_for(V_target, delta_e_aero)
    a_g = math.radians(alpha_guess_deg)
    U0 = V_target * math.cos(a_g)
    W0 = -V_target * math.sin(a_g)                      # alpha = atan2(-W0,U0) = +a_g (air from below, nose-up AoA)
    # SIGN, probed this task (scratchpad/pitchprobe.py): gz.math7 Pose3d Euler
    # pitch AND aero_lib/TR.quat_rpy pitch are NOSE-DOWN-POSITIVE for this FLU
    # freefall world (a Pose3d pitch arg of +8.59 deg puts the body +X/nose
    # world-Z at -0.149 = pointing DOWN). Physical nose-up angle = -quat_rpy_pitch.
    # Per Part A.3 ("if the readback is -alpha_guess, negate PITCH0 and document
    # it"): to spawn genuinely NOSE-UP (so initial world_vz ~ 0 for a level IC),
    # PITCH0 must be -a_g in the gz Euler convention.
    PITCH0 = -a_g

    st = {"n": 0, "teleported": False, "throttle_cmd": None,
          "prop_diag": None, "aero_diag": None,
          "series": [], "diag_series": [],
          "any_nan": False, "nan_first_tick": None,
          "weight": None, "first_free_pitch": None, "release_sample": None}

    def on_pre(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not st["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0.0, 0.0, 150.0, 0.0, PITCH0, 0.0))
            st["teleported"] = True
            st["throttle_cmd"] = PL.ThrottleCommander()
        st["throttle_cmd"].set(left=throttle, right=throttle)
        st["throttle_cmd"].tick()
        PL.pin_control_surface_joints(model, ecm, sim, positions={
            "left_elevator_joint": E, "right_elevator_joint": E})
        n = st["n"]
        if n < hold_steps:
            AL.hold_step(base, ecm, MASS_CTRL, I_DIAG,
                         gm.Vector3d(U0, 0.0, W0), gm.Vector3d(0.0, 0.0, 0.0),
                         kp_lin=KP_LIN, kp_ang=KP_ANG)
        # else: fully released - nothing touches base_link (Part B.1 primary, no SETTLE_ASSIST)

    def on_post(info, ecm):
        n = st["n"]
        if st["prop_diag"] is None:
            try:
                st["prop_diag"] = PL.DiagSubscriber()
            except Exception:
                pass
        if st["aero_diag"] is None:
            try:
                st["aero_diag"] = AL.DiagSubscriber()
            except Exception:
                pass
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        wpose = base.world_pose(ecm)
        lv = base.world_linear_velocity(ecm)
        av = base.world_angular_velocity(ecm)
        if wpose is None or lv is None or av is None:
            st["any_nan"] = True
            st["nan_first_tick"] = st["nan_first_tick"] if st["nan_first_tick"] is not None else n
            st["n"] += 1
            return
        rot = wpose.rot()
        lv_b = rot.rotate_vector_reverse(lv)
        av_b = rot.rotate_vector_reverse(av)
        u, v, w = lv_b.x(), lv_b.y(), lv_b.z()
        p, q, r = av_b.x(), av_b.y(), av_b.z()
        world_vz = lv.z()
        alt = wpose.pos().z()
        roll, pitch, yaw = quat_rpy(rot)
        V = math.sqrt(u * u + v * v + w * w)
        alpha = math.atan2(-w, u)
        beta = math.atan2(v, math.hypot(u, w))
        guard = [u, v, w, p, q, r, world_vz, alt, roll, pitch, yaw, V, alpha, beta]
        if any((x is None) or math.isnan(x) or math.isinf(x) for x in guard):
            st["any_nan"] = True
            st["nan_first_tick"] = st["nan_first_tick"] if st["nan_first_tick"] is not None else n

        if n == hold_steps:
            tm, wN, per_link = sum_model_weight(model, ecm)
            st["weight"] = dict(total_mass_kg=tm, weight_N=wN, per_link_mass_kg=per_link,
                                g_used=G_WORLD)
            st["first_free_pitch"] = pitch

        if n >= hold_steps:
            k = n - hold_steps
            if k % RECORD_EVERY == 0:
                st["series"].append(dict(
                    t=k * STEP, u=u, v=v, w=w, p=p, q=q, r=r,
                    world_vz=world_vz, alt=alt, V=V,
                    roll=roll, pitch=pitch, yaw=yaw, alpha=alpha, beta=beta))
            if k % DIAG_EVERY == 0:
                aero_live = st["aero_diag"].latest() if st["aero_diag"] else None
                prop = st["prop_diag"].latest() if st["prop_diag"] else None
                # actual pinned elevator deflection (ground-truth ECM readback)
                thLE = _joint_pos(model, ecm, "left_elevator_joint")
                thRE = _joint_pos(model, ecm, "right_elevator_joint")
                de_act = 0.5 * ELEV_SIGN * ((thLE or 0.0) + (thRE or 0.0))
                mir = predict_aero(REF, u, v, w, p, q, r, deltaA=0.0, deltaE=de_act, deltaR=0.0)
                st["diag_series"].append(dict(
                    t=k * STEP, aero_live=aero_live, prop=prop,
                    theta_LE=thLE, theta_RE=thRE, delta_e_aero_actual=de_act,
                    mirror=dict(CL=mir["CL"], CD=mir["CD"], CY=mir["CY"], Cl=mir["Cl"],
                                Cm=mir["Cm"], Cn=mir["Cn"], Mx=mir["Mx"], My=mir["My"],
                                Mz=mir["Mz"], qbar=mir["qbar"], V=mir["V"],
                                alpha=mir["alpha"], beta=mir["beta"])))
            if n == hold_steps:
                st["release_sample"] = dict(V=V, alpha=alpha, pitch=pitch, world_vz=world_vz, q=q)
        st["n"] += 1

    def _joint_pos(model, ecm, jn):
        je = model.joint_by_name(ecm, jn)
        if je is None:
            return None
        j = sim.Joint(je)
        j.enable_position_check(ecm, True)
        pos = j.position(ecm)
        return pos[0] if pos else None

    t_wall0 = time.time()
    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, hold_steps + window_steps, False)
    wall_s = time.time() - t_wall0

    return _reduce(label, V_target, throttle, elevator_phys_deg, delta_e_aero,
                   alpha_guess_deg, PITCH0, window_steps, st, wall_s, raw_decimate)


def _reduce(label, V_target, throttle, elevator_phys_deg, delta_e_aero,
            alpha_guess_deg, pitch0, window_steps, st, wall_s, raw_decimate=1):
    ser = st["series"]
    dser = st["diag_series"]
    out = dict(label=label, command=dict(V_target=V_target, throttle=throttle,
               elevator_phys_deg=elevator_phys_deg,
               delta_e_aero_deg=math.degrees(delta_e_aero),
               alpha_guess_deg=alpha_guess_deg, pitch0_rad=pitch0),
               window_s=window_steps * STEP, hold_s=HOLD_STEPS * STEP,
               wall_clock_s=wall_s, any_nan=st["any_nan"], nan_first_tick=st["nan_first_tick"],
               weight=st["weight"],
               first_free_quat_pitch_deg=(math.degrees(st["first_free_pitch"])
                                          if st["first_free_pitch"] is not None else None),
               first_free_phys_pitch_deg=(-math.degrees(st["first_free_pitch"])
                                          if st["first_free_pitch"] is not None else None),
               pitch_convention="reported pitch is PHYSICAL nose-up-positive = -(gz/quat_rpy Euler pitch); "
                                "raw_series 'pitch' field is the raw gz/quat_rpy value (nose-down-positive)",
               release=st["release_sample"], n_series=len(ser), n_diag=len(dser))
    if not ser:
        out["error"] = "no samples recorded"
        return out

    t = [s["t"] for s in ser]
    n = len(ser)
    half = n // 2
    tail = ser[half:]
    tt = [s["t"] for s in tail]

    def col(rows, k):
        return [rw[k] for rw in rows]

    # ---- damped-sinusoid + trend fits over the WHOLE window ----
    # pitch reported PHYSICAL nose-up-positive = -(raw gz/quat_rpy pitch); see probe note above.
    fit_vz = damped_sinusoid_fit(t, col(ser, "world_vz"))
    fit_V = damped_sinusoid_fit(t, col(ser, "V"))
    fit_pitch = damped_sinusoid_fit(t, [-math.degrees(x) for x in col(ser, "pitch")])

    # ---- Part D reductions ----
    vz_tail = col(tail, "world_vz")
    alt_tail = col(tail, "alt")
    V_all = col(ser, "V")
    V_tail = col(tail, "V")
    u_all = col(ser, "u")
    q_tail_degs = [math.degrees(x) for x in col(tail, "q")]  # raw gz body pitch rate (nose-down-positive)
    pitch_all_deg = [-math.degrees(x) for x in col(ser, "pitch")]   # PHYSICAL nose-up-positive
    pitch_tail_deg = [-math.degrees(x) for x in col(tail, "pitch")]
    alpha_tail_deg = [math.degrees(x) for x in col(tail, "alpha")]
    roll_tail_deg = [math.degrees(x) for x in col(tail, "roll")]
    p_tail_degs = [math.degrees(x) for x in col(tail, "p")]
    r_tail_degs = [math.degrees(x) for x in col(tail, "r")]
    beta_tail_deg = [math.degrees(x) for x in col(tail, "beta")]

    # longitudinal accel = d(u)/dt (finite difference over the record cadence)
    dt_rec = RECORD_EVERY * STEP
    du = np.diff(np.asarray(u_all, float)) / dt_rec
    long_accel = dict(mean=float(np.mean(du)), max_abs=float(np.max(np.abs(du))),
                      tail_mean=float(np.mean(du[half:])) if du.size > half else None)

    V_slope_full = linreg_slope(t, V_all)
    V_slope_tail = linreg_slope(tt, V_tail)
    alt_slope_full = linreg_slope(t, col(ser, "alt"))
    alt_slope_tail = linreg_slope(tt, alt_tail)
    pitch_drift_full = pitch_all_deg[-1] - pitch_all_deg[0]
    pitch_drift_tail = pitch_tail_deg[-1] - pitch_tail_deg[0]
    V_drift_full = V_all[-1] - V_all[0]

    steady = dict(
        world_vz_tail=stat(vz_tail),
        world_vz_full=stat(col(ser, "world_vz")),
        world_vz_fit_offset=(fit_vz["A"] if fit_vz else None),
        world_vz_fit_drift_Bt_over_window=((fit_vz["B"] * (t[-1] - t[0])) if fit_vz else None),
        altitude_slope_tail_mps=alt_slope_tail,
        altitude_slope_full_mps=alt_slope_full,
        V_mean_tail=stat(V_tail),
        V_slope_full_mps2=V_slope_full,
        V_slope_tail_mps2=V_slope_tail,
        V_drift_full_mps=V_drift_full,
        V_fit_offset=(fit_V["A"] if fit_V else None),
        long_accel_mps2=long_accel,
        q_tail_degs=stat(q_tail_degs),
        pitch_tail_deg=stat(pitch_tail_deg),
        pitch_drift_full_deg=pitch_drift_full,
        pitch_drift_tail_deg=pitch_drift_tail,
        pitch_fit_offset_deg=(fit_pitch["A"] if fit_pitch else None),
        alpha_tail_deg=stat(alpha_tail_deg),
        roll_tail_deg=stat(roll_tail_deg),
        p_tail_degs=stat(p_tail_degs),
        r_tail_degs=stat(r_tail_degs),
        beta_tail_deg=stat(beta_tail_deg),
    )
    out["phugoid"] = dict(from_world_vz=fit_vz, from_V=fit_V, from_pitch=fit_pitch)
    out["steady_state"] = steady

    # ---- aero + propulsion tail-window means (Part D) ----
    dtail = [d for d in dser if d["t"] >= tt[0]] if tt else []
    if not dtail:
        dtail = dser[len(dser) // 2:]

    def dcol_live(k):
        return [d["aero_live"][k] for d in dtail if d.get("aero_live") is not None]

    def dcol_mir(k):
        return [d["mirror"][k] for d in dtail if d.get("mirror") is not None]

    aero_live_mean = {k: stat(dcol_live(k)) for k in ("CL", "CD", "CY", "Cl", "Cm", "Cn", "qbar", "V", "alpha", "beta")}
    aero_mir_mean = {k: stat(dcol_mir(k)) for k in ("CL", "CD", "CY", "Cl", "Cm", "Cn", "Mx", "My", "Mz", "qbar")}

    # tick-paired mirror-vs-live diffs: keep only ticks where a NEW, DISTINCT live
    # message was first seen (the live topic's true ~20 Hz cadence), per the
    # paired_live_ticks() methodology in the mirror module.
    paired = []
    _prev_cl = object()
    for d in dtail:
        lv_ = d.get("aero_live")
        if lv_ is None:
            continue
        if lv_["CL"] != _prev_cl:
            paired.append((d["mirror"], lv_))
            _prev_cl = lv_["CL"]
    paired_diff = {}
    for k in ("CL", "CD", "CY", "Cl", "Cm", "Cn"):
        ds = [abs(m[k] - l[k]) for m, l in paired if (m.get(k) is not None and l.get(k) is not None)]
        rel = [abs(m[k] - l[k]) / max(abs(m[k]), abs(l[k]), 1e-6) for m, l in paired
               if (m.get(k) is not None and l.get(k) is not None)]
        paired_diff[k] = dict(mean_abs=(float(np.mean(ds)) if ds else None),
                              max_abs=(float(np.max(ds)) if ds else None),
                              mean_rel=(float(np.mean(rel)) if rel else None),
                              n=len(ds))
    out["aero_live_tail_mean"] = aero_live_mean
    out["aero_mirror_tail_mean"] = aero_mir_mean
    out["aero_mirror_vs_live_paired_diff"] = paired_diff

    # Lift/Drag from LIVE qbar & CL/CD; L/W; T_total - D
    qbar_m = aero_live_mean["qbar"]["mean"] if aero_live_mean["qbar"] else None
    CL_m = aero_live_mean["CL"]["mean"] if aero_live_mean["CL"] else None
    CD_m = aero_live_mean["CD"]["mean"] if aero_live_mean["CD"] else None
    lift = (qbar_m * S_REF * CL_m) if (qbar_m is not None and CL_m is not None) else None
    drag = (qbar_m * S_REF * CD_m) if (qbar_m is not None and CD_m is not None) else None

    lthr = [d["prop"]["left"]["throttle"] for d in dtail if d.get("prop")]
    rthr = [d["prop"]["right"]["throttle"] for d in dtail if d.get("prop")]
    lrpm = [d["prop"]["left"]["rpm"] for d in dtail if d.get("prop")]
    rrpm = [d["prop"]["right"]["rpm"] for d in dtail if d.get("prop")]
    lT = [d["prop"]["left"]["thrust_N"] for d in dtail if d.get("prop")]
    rT = [d["prop"]["right"]["thrust_N"] for d in dtail if d.get("prop")]
    lJ = [d["prop"]["left"]["J"] for d in dtail if d.get("prop")]
    lCt = [d["prop"]["left"]["Ct"] for d in dtail if d.get("prop")]
    lQp = [d["prop"]["left"]["Q_prop_Nm"] for d in dtail if d.get("prop")]
    thrust_total = ((float(np.mean(lT)) + float(np.mean(rT))) if (lT and rT) else None)
    weight_N = st["weight"]["weight_N"] if st["weight"] else 58.86

    my_aero = aero_mir_mean["My"]["mean"] if aero_mir_mean["My"] else None
    net_my = (my_aero + DZ_HUB_CG * thrust_total) if (my_aero is not None and thrust_total is not None) else None

    out["force_balance"] = dict(
        weight_N=weight_N,
        lift_N=lift, drag_N=drag,
        lift_over_weight=(lift / weight_N) if lift is not None else None,
        thrust_total_N=thrust_total,
        thrust_minus_drag_N=((thrust_total - drag) if (thrust_total is not None and drag is not None) else None),
        throttle_actual_L=stat(lthr), throttle_actual_R=stat(rthr),
        throttle_asym_max=(float(np.max(np.abs(np.asarray(lthr) - np.asarray(rthr)))) if (lthr and rthr) else None),
        rpm_L=stat(lrpm), rpm_R=stat(rrpm),
        thrust_L=stat(lT), thrust_R=stat(rT),
        prop_J_L=stat(lJ), prop_Ct_L=stat(lCt), prop_Qprop_L=stat(lQp),
        My_aero_Nm=my_aero,
        net_My_Nm=net_my,
        net_My_note="net_My = My_aero(mirror) + dz*T_total, dz=0.0271 m; fwd thrust above CG => nose-down dz*T; ~0 at pitch trim",
    )

    # ---- flight-path / equilibrium cross-check (Part D context) ----
    V_eq = steady["V_mean_tail"]["mean"] if steady["V_mean_tail"] else None
    a_eq = steady["alpha_tail_deg"]["mean"] if steady["alpha_tail_deg"] else None
    th_eq = steady["pitch_tail_deg"]["mean"] if steady["pitch_tail_deg"] else None
    vz_eq = steady["world_vz_tail"]["mean"] if steady["world_vz_tail"] else None
    gamma_from_vz = (math.degrees(math.asin(max(-1.0, min(1.0, vz_eq / V_eq))))
                     if (V_eq and vz_eq is not None) else None)
    gamma_from_att = ((th_eq - a_eq) if (th_eq is not None and a_eq is not None) else None)
    # along flight path (steady): T - D - W*sin(gamma) should be ~0
    tmd_expected_for_climb = ((weight_N * math.sin(math.radians(gamma_from_vz)))
                              if gamma_from_vz is not None else None)
    out["flight_path"] = dict(
        V_eq=V_eq, alpha_eq_deg=a_eq, pitch_phys_eq_deg=th_eq, world_vz_eq=vz_eq,
        gamma_from_world_vz_deg=gamma_from_vz, gamma_from_pitch_minus_alpha_deg=gamma_from_att,
        gamma_consistency_deg=((gamma_from_vz - gamma_from_att)
                               if (gamma_from_vz is not None and gamma_from_att is not None) else None),
        thrust_minus_drag_N=out["force_balance"]["thrust_minus_drag_N"],
        thrust_minus_drag_expected_for_that_climb_N=tmd_expected_for_climb,
        note="steady climb angle gamma = asin(world_vz/V) should equal (pitch_phys - alpha); "
             "and T-D should equal W*sin(gamma). Both are independent self-consistency checks on the equilibrium.")

    # ---- Part D steady-state acceptance table ----
    def le(x, lim):
        return (x is not None) and (abs(x) <= lim)

    vz_for_accept = steady["world_vz_fit_offset"]
    if vz_for_accept is None:
        vz_for_accept = steady["world_vz_tail"]["mean"] if steady["world_vz_tail"] else None
    acc = dict(
        world_vz_abs_le_0p10=le(vz_for_accept, 0.10),
        world_vz_abs_le_0p05=le(vz_for_accept, 0.05),
        alt_slope_abs_le_0p10=le(alt_slope_tail, 0.10),
        q_mean_abs_le_0p3_degs=le(steady["q_tail_degs"]["mean"] if steady["q_tail_degs"] else None, 0.3),
        long_accel_abs_le_0p03=le(long_accel["tail_mean"], 0.03),
        V_drift_abs_le_0p4=le(V_drift_full, 0.4),
        pitch_drift_abs_le_0p5_deg=le(pitch_drift_full, 0.5),
        roll_mean_abs_le_0p5_deg=le(steady["roll_tail_deg"]["mean"] if steady["roll_tail_deg"] else None, 0.5),
        p_mean_abs_le_0p5_degs=le(steady["p_tail_degs"]["mean"] if steady["p_tail_degs"] else None, 0.5),
        r_mean_abs_le_0p5_degs=le(steady["r_tail_degs"]["mean"] if steady["r_tail_degs"] else None, 0.5),
        beta_abs_le_0p5_deg=le(steady["beta_tail_deg"]["mean"] if steady["beta_tail_deg"] else None, 0.5),
        lift_over_weight_1p00_pm0p01=(out["force_balance"]["lift_over_weight"] is not None
                                      and abs(out["force_balance"]["lift_over_weight"] - 1.0) <= 0.01),
        thrust_minus_drag_abs_le_0p3_N=le(out["force_balance"]["thrust_minus_drag_N"], 0.3),
        any_nan_false=(not st["any_nan"]),
    )
    acc["ALL_PASS"] = all(bool(x) for x in acc.values())
    out["acceptance"] = dict(vz_used_for_accept=vz_for_accept, checks=acc)

    # keep ALL raw timeseries
    # keep ALL raw timeseries (raw_decimate=1); intermediate sweep runs pass
    # raw_decimate>1 purely to bound the aggregate result-file size in the repo -
    # the per-tick NaN guard and every reduction/fit above already used the full
    # 100 Hz series.
    out["raw_series_100hz"] = ser[::raw_decimate] if raw_decimate > 1 else ser
    out["raw_diag_series"] = dser[::raw_decimate] if raw_decimate > 1 else dser
    out["raw_decimate"] = raw_decimate
    return out


def _short(r):
    s = r["steady_state"]
    fb = r["force_balance"]
    vz = r["acceptance"]["vz_used_for_accept"]
    return (f"[{r['label']}] V_cmd={r['command']['V_target']:.3f} thr={r['command']['throttle']:.4f} "
            f"elev={r['command']['elevator_phys_deg']:+.2f}deg | "
            f"world_vz(fit/tail)={vz:+.4f}/"
            f"{(s['world_vz_tail']['mean'] if s['world_vz_tail'] else float('nan')):+.4f} m/s "
            f"q_tail={(s['q_tail_degs']['mean'] if s['q_tail_degs'] else float('nan')):+.4f} deg/s "
            f"Vdot_tail={(s['long_accel_mps2']['tail_mean'] if s['long_accel_mps2']['tail_mean'] is not None else float('nan')):+.5f} "
            f"alpha_tail={(s['alpha_tail_deg']['mean'] if s['alpha_tail_deg'] else float('nan')):+.3f} "
            f"theta_tail={(s['pitch_tail_deg']['mean'] if s['pitch_tail_deg'] else float('nan')):+.3f} | "
            f"L/W={(fb['lift_over_weight'] if fb['lift_over_weight'] else float('nan')):.4f} "
            f"T-D={(fb['thrust_minus_drag_N'] if fb['thrust_minus_drag_N'] is not None else float('nan')):+.3f}N "
            f"net_My={(fb['net_My_Nm'] if fb['net_My_Nm'] is not None else float('nan')):+.4f} "
            f"nan={r['any_nan']} accept={r['acceptance']['checks']['ALL_PASS']} "
            f"phug_T={(r['phugoid']['from_world_vz']['period_s'] if r['phugoid']['from_world_vz'] else float('nan')):.2f}s "
            f"zeta={(r['phugoid']['from_world_vz']['zeta'] if r['phugoid']['from_world_vz'] else float('nan')):.3f} "
            f"wall={r['wall_clock_s']:.0f}s")


# =============================================================================
# Part C.1
# =============================================================================
def run_c1(log):
    log("=" * 90)
    log("PART C.1 - reproduce the current reference point verbatim (pure Gazebo, 60 s free window)")
    log(f"  V_target={C1_V}  throttle={C1_THROTTLE}  elevator_physical=+{C1_ELEV_PHYS_DEG:.2f} deg "
        f"(delta_e_aero=-{C1_ELEV_PHYS_DEG:.2f} deg), ailerons/rudder 0")
    log("=" * 90)
    r = run_point("C1_reference", C1_V, C1_THROTTLE, C1_ELEV_PHYS_DEG, C1_WINDOW_STEPS)
    log(_short(r))
    log("")
    log(f"  measured weight at run start: total_mass={r['weight']['total_mass_kg']:.4f} kg "
        f"-> weight={r['weight']['weight_N']:.3f} N (g={G_WORLD}); per-link: "
        + ", ".join(f"{k}={v:.4f}" for k, v in r['weight']['per_link_mass_kg'].items() if v is not None))
    log(f"  sign check (see scratchpad/pitchprobe.py): gz Euler / quat_rpy pitch is NOSE-DOWN-POSITIVE "
        f"for this FLU freefall world; PHYSICAL pitch = -(gz pitch).")
    log(f"    teleport Pose3d pitch arg = {math.degrees(r['command']['pitch0_rad']):+.3f} deg "
        f"(= -alpha_guess, i.e. genuine NOSE-UP), first free-tick: gz pitch = "
        f"{r['first_free_quat_pitch_deg']:+.3f} deg -> physical pitch = {r['first_free_phys_pitch_deg']:+.3f} deg "
        f"(expect ~ +alpha_guess {r['command']['alpha_guess_deg']:+.3f} for a level nose-up IC)")
    s = r["steady_state"]
    fb = r["force_balance"]
    log("")
    log("  Part D residual table (C.1):")
    log(f"    world_vz:  fit_offset A = {s['world_vz_fit_offset']:+.5f} m/s   "
        f"fit drift B*T = {s['world_vz_fit_drift_Bt_over_window']:+.5f} m/s   "
        f"tail mean = {s['world_vz_tail']['mean']:+.5f} +/- {s['world_vz_tail']['std']:.5f}   "
        f"full mean = {s['world_vz_full']['mean']:+.5f}")
    log(f"    altitude slope:  tail = {s['altitude_slope_tail_mps']:+.5f} m/s   full = {s['altitude_slope_full_mps']:+.5f} m/s")
    log(f"    airspeed V:  tail mean = {s['V_mean_tail']['mean']:.4f}   slope_full = {s['V_slope_full_mps2']:+.6f} m/s^2   "
        f"slope_tail = {s['V_slope_tail_mps2']:+.6f}   drift_full = {s['V_drift_full_mps']:+.4f} m/s   fit_offset = {s['V_fit_offset']:.4f}")
    log(f"    longitudinal accel du/dt:  mean = {s['long_accel_mps2']['mean']:+.6f}   "
        f"tail mean = {s['long_accel_mps2']['tail_mean']:+.6f}   max|.| = {s['long_accel_mps2']['max_abs']:.5f} m/s^2")
    log(f"    pitch rate q:  tail mean = {s['q_tail_degs']['mean']:+.5f} +/- {s['q_tail_degs']['std']:.5f} deg/s")
    log(f"    pitch theta:  tail mean = {s['pitch_tail_deg']['mean']:+.4f} deg   drift_full = {s['pitch_drift_full_deg']:+.4f}   "
        f"drift_tail = {s['pitch_drift_tail_deg']:+.4f}   fit_offset = {s['pitch_fit_offset_deg']:+.4f}")
    log(f"    alpha:  tail mean = {s['alpha_tail_deg']['mean']:+.4f} deg")
    log(f"    lateral:  roll = {s['roll_tail_deg']['mean']:+.4f} deg   p = {s['p_tail_degs']['mean']:+.4f} deg/s   "
        f"r = {s['r_tail_degs']['mean']:+.4f} deg/s   beta = {s['beta_tail_deg']['mean']:+.4f} deg")
    al = r["aero_live_tail_mean"]
    log(f"    aero (live diag tail mean):  CL={al['CL']['mean']:.5f} CD={al['CD']['mean']:.5f} CY={al['CY']['mean']:+.6f} "
        f"Cl={al['Cl']['mean']:+.6f} Cm={al['Cm']['mean']:+.6f} Cn={al['Cn']['mean']:+.6f}  qbar={al['qbar']['mean']:.4f}")
    log(f"    Lift = {fb['lift_N']:.4f} N   Drag = {fb['drag_N']:.4f} N   Weight = {fb['weight_N']:.3f} N   "
        f"Lift/Weight = {fb['lift_over_weight']:.5f}")
    log(f"    Thrust L/R = {fb['thrust_L']['mean']:.4f}/{fb['thrust_R']['mean']:.4f} N   total = {fb['thrust_total_N']:.4f} N   "
        f"Thrust_total - Drag = {fb['thrust_minus_drag_N']:+.4f} N")
    log(f"    RPM L/R = {fb['rpm_L']['mean']:.1f}/{fb['rpm_R']['mean']:.1f}   throttle_actual L/R = "
        f"{fb['throttle_actual_L']['mean']:.5f}/{fb['throttle_actual_R']['mean']:.5f}")
    log(f"    My_aero (mirror) = {fb['My_aero_Nm']:+.5f} N m   net_My (aero + dz*T) = {fb['net_My_Nm']:+.5f} N m")
    fp = r["flight_path"]
    log(f"    flight-path cross-check:  gamma(from world_vz) = {fp['gamma_from_world_vz_deg']:+.4f} deg   "
        f"gamma(pitch_phys - alpha) = {fp['gamma_from_pitch_minus_alpha_deg']:+.4f} deg   "
        f"(consistency {fp['gamma_consistency_deg']:+.4f} deg)")
    log(f"    along-path balance:  T-D = {fp['thrust_minus_drag_N']:+.4f} N   vs  W*sin(gamma) expected "
        f"{fp['thrust_minus_drag_expected_for_that_climb_N']:+.4f} N")
    pv = r["phugoid"]["from_world_vz"]
    pp = r["phugoid"]["from_pitch"]
    log(f"    phugoid (world_vz fit):  period = {pv['period_s']:.3f} s   zeta = {pv['zeta']:.4f}   "
        f"amp0 = {pv['amp0']:.4f} m/s   rmse = {pv['rmse']:.4f}")
    log(f"    phugoid (pitch fit):     period = {pp['period_s']:.3f} s   zeta = {pp['zeta']:.4f}   amp0 = {pp['amp0']:.4f} deg")
    pd = r["aero_mirror_vs_live_paired_diff"]
    log(f"    mirror vs live (tick-paired tail): "
        + ", ".join(f"{k} mean|d|={pd[k]['mean_abs']:.5f} rel={pd[k]['mean_rel']*100:.2f}% (n={pd[k]['n']})"
                    for k in ("CL", "CD", "Cm")))
    log("")
    acc = r["acceptance"]["checks"]
    log(f"  ACCEPTANCE (Part D table): ALL_PASS = {acc['ALL_PASS']}")
    for k, v in acc.items():
        if k != "ALL_PASS":
            log(f"    {'PASS' if v else 'FAIL'}  {k}")
    verdict = "DOES NOT SINK / meets acceptance" if acc["ALL_PASS"] else \
              ("SINKS" if (r["acceptance"]["vz_used_for_accept"] or 0) < -0.10 else
               "CLIMBS" if (r["acceptance"]["vz_used_for_accept"] or 0) > 0.10 else "residual - fails other criteria")
    log(f"  C.1 VERDICT: {verdict}")
    return r


# =============================================================================
# Part C.2 - bounded iterative single-variable sweep
# =============================================================================
def interp_for_zero(pairs):
    """pairs: [(x, y), ...]. Return x where linear interp of y crosses 0.
    Prefer a bracketing pair; else extrapolate from the two points with the
    smallest |y|."""
    pairs = sorted(pairs, key=lambda p: p[0])
    for (x0, y0), (x1, y1) in zip(pairs, pairs[1:]):
        if y0 == 0:
            return x0
        if (y0 < 0) != (y1 < 0):
            return x0 + (x1 - x0) * (-y0) / (y1 - y0)
    ps = sorted(pairs, key=lambda p: abs(p[1]))[:2]
    (x0, y0), (x1, y1) = sorted(ps, key=lambda p: p[0])
    if y1 == y0:
        return 0.5 * (x0 + x1)
    return x0 + (x1 - x0) * (-y0) / (y1 - y0)


def _q_metric(r):
    return r["steady_state"]["q_tail_degs"]["mean"]


def _vdot_metric(r):
    return r["steady_state"]["long_accel_mps2"]["tail_mean"]


def _vz_metric(r):
    return r["acceptance"]["vz_used_for_accept"]


def run_sweep(log):
    log("=" * 90)
    log("PART C.2 - bounded iterative single-variable sweep (elevator -> throttle -> V, <= 3 V values)")
    log(f"  sweep window = {SWEEP_WINDOW_STEPS * STEP:.0f} s free (Part B: '>= 2 phugoid periods; use 30 s'); "
        f"C.1 + final converged point use the full 60 s")
    log("=" * 90)
    runs = []
    run_table = []

    def do(label, V, thr, elev):
        r = run_point(label, V, thr, elev, SWEEP_WINDOW_STEPS, raw_decimate=10)
        runs.append(r)
        s = r["steady_state"]
        fb = r["force_balance"]
        row = dict(label=label, V_cmd=V, elevator_phys_deg=elev,
                   delta_e_aero_deg=r["command"]["delta_e_aero_deg"], throttle_cmd=thr,
                   throttle_actual_L=(fb["throttle_actual_L"]["mean"] if fb["throttle_actual_L"] else None),
                   world_vz=_vz_metric(r),
                   world_vz_tail_mean=(s["world_vz_tail"]["mean"] if s["world_vz_tail"] else None),
                   q_tail_degs=_q_metric(r), Vdot_tail=_vdot_metric(r),
                   V_tail_mean=(s["V_mean_tail"]["mean"] if s["V_mean_tail"] else None),
                   alpha_tail_deg=(s["alpha_tail_deg"]["mean"] if s["alpha_tail_deg"] else None),
                   theta_tail_deg=(s["pitch_tail_deg"]["mean"] if s["pitch_tail_deg"] else None),
                   pitch_drift_full_deg=s["pitch_drift_full_deg"],
                   lift_over_weight=fb["lift_over_weight"],
                   thrust_minus_drag_N=fb["thrust_minus_drag_N"],
                   net_My_Nm=fb["net_My_Nm"], any_nan=r["any_nan"],
                   accept_all=r["acceptance"]["checks"]["ALL_PASS"])
        run_table.append(row)
        log(_short(r))
        return r

    V_candidates = [C1_V]
    tried_V = []
    converged = None
    elev_star = thr_star = None
    for it in range(3):
        V = round(V_candidates[it], 3)
        tried_V.append(V)
        log("-" * 90)
        log(f"ITERATION {it + 1}: V = {V:.3f} m/s")
        log("-" * 90)

        # step 1: elevator sweep at nominal throttle -> q_tail ~ 0 AND non-drifting pitch
        nominal_thr = 0.50 if thr_star is None else round(thr_star, 4)
        elev_pts = [3.5, 4.5, 5.5]
        e_runs = [do(f"it{it+1}_elev{e:+.1f}", V, nominal_thr, e) for e in elev_pts]
        elev_star = interp_for_zero([(e, _q_metric(r)) for e, r in zip(elev_pts, e_runs)])
        elev_star = max(2.5, min(6.0, elev_star))
        # analytical cross-check: net_My_est vs elevator
        log(f"  step 1: q_tail(deg/s) vs elevator_phys = "
            + ", ".join(f"{e:+.1f}:{_q_metric(r):+.4f}" for e, r in zip(elev_pts, e_runs))
            + f"  -> elevator* = {elev_star:+.3f} deg (physical)")
        log(f"          net_My_est vs elevator = "
            + ", ".join(f"{e:+.1f}:{r['force_balance']['net_My_Nm']:+.4f}" for e, r in zip(elev_pts, e_runs)))

        # step 2: throttle sweep at elevator* -> Vdot_tail ~ 0
        thr_pts = [0.47, 0.50, 0.53]
        t_runs = [do(f"it{it+1}_thr{th:.2f}", V, th, elev_star) for th in thr_pts]
        thr_star = interp_for_zero([(th, _vdot_metric(r)) for th, r in zip(thr_pts, t_runs)])
        thr_star = max(0.46, min(0.54, thr_star))
        log(f"  step 2: Vdot_tail(m/s^2) vs throttle = "
            + ", ".join(f"{th:.2f}:{_vdot_metric(r):+.5f}" for th, r in zip(thr_pts, t_runs))
            + f"  -> throttle* = {thr_star:.4f}")

        # step 3: confirm run at (V, throttle*, elevator*)
        rc = do(f"it{it+1}_confirm", V, round(thr_star, 4), elev_star)
        vz = _vz_metric(rc)
        q = _q_metric(rc)
        vd = _vdot_metric(rc)
        ok = (abs(vz) <= 0.10) and (abs(q) <= 0.3) and (abs(vd) <= 0.03)
        log(f"  step 3: confirm (V={V:.3f}, thr*={thr_star:.4f}, elev*={elev_star:+.3f}): "
            f"world_vz={vz:+.4f} q_tail={q:+.4f} Vdot_tail={vd:+.5f}  -> {'CONVERGED' if ok else 'not yet'}")
        if ok:
            converged = rc
            break
        # adjust V: climb -> V down, sink -> V up
        if vz > 0:
            V_candidates.append(V - 0.5)
            log(f"  -> climb (world_vz>0): step V down to {V - 0.5:.3f}")
        else:
            V_candidates.append(V + 0.5)
            log(f"  -> sink (world_vz<0): step V up to {V + 0.5:.3f}")

    log("")
    log("-" * 90)
    log("FULL SWEEP RUN TABLE")
    log("-" * 90)
    hdr = ("label", "V_cmd", "elev_phys", "de_aero", "thr_cmd", "thr_act", "world_vz",
           "vz_tail", "q_tail", "Vdot_tail", "V_tail", "alpha", "theta", "th_drift", "L/W", "T-D", "net_My", "accept")
    log("  " + " | ".join(f"{h:>9s}" for h in hdr))
    for row in run_table:
        log("  " + " | ".join([
            f"{row['label']:>9s}", f"{row['V_cmd']:>9.3f}", f"{row['elevator_phys_deg']:>+9.2f}",
            f"{row['delta_e_aero_deg']:>+9.2f}", f"{row['throttle_cmd']:>9.4f}",
            f"{(row['throttle_actual_L'] or float('nan')):>9.4f}", f"{(row['world_vz'] or float('nan')):>+9.4f}",
            f"{(row['world_vz_tail_mean'] or float('nan')):>+9.4f}", f"{(row['q_tail_degs'] or float('nan')):>+9.4f}",
            f"{(row['Vdot_tail'] if row['Vdot_tail'] is not None else float('nan')):>+9.5f}",
            f"{(row['V_tail_mean'] or float('nan')):>9.3f}", f"{(row['alpha_tail_deg'] or float('nan')):>+9.3f}",
            f"{(row['theta_tail_deg'] or float('nan')):>+9.3f}",
            f"{(row['pitch_drift_full_deg'] if row['pitch_drift_full_deg'] is not None else float('nan')):>+9.3f}",
            f"{(row['lift_over_weight'] or float('nan')):>9.4f}",
            f"{(row['thrust_minus_drag_N'] if row['thrust_minus_drag_N'] is not None else float('nan')):>+9.3f}",
            f"{(row['net_My_Nm'] if row['net_My_Nm'] is not None else float('nan')):>+9.4f}",
            f"{str(row['accept_all']):>9s}"]))

    final = None
    if converged is not None:
        cV = converged["command"]["V_target"]
        cthr = converged["command"]["throttle"]
        cel = converged["command"]["elevator_phys_deg"]
        log("")
        log(f"CONVERGED sweep point: V={cV:.3f}  throttle={cthr:.4f}  elevator_phys={cel:+.3f} deg. "
            f"Re-running at the FULL 60 s window for the definitive (V*, throttle*, elevator*, pitch*).")
        final = run_point("final_60s", cV, cthr, cel, C1_WINDOW_STEPS, raw_decimate=2)
        # NOT appended to `runs` (avoids duplicating its ~3 MB raw arrays) - it is
        # returned separately as `final_full_window`.
        log(_short(final))
        s = final["steady_state"]
        log(f"  (V*, throttle*, elevator*, pitch*, alpha*) = ({s['V_mean_tail']['mean']:.3f}, {cthr:.4f}, "
            f"{cel:+.3f} deg, {s['pitch_tail_deg']['mean']:+.3f} deg, {s['alpha_tail_deg']['mean']:+.3f} deg)")
        log(f"  world_vz there (fit / tail) = {s['world_vz_fit_offset']:+.5f} / {s['world_vz_tail']['mean']:+.5f} m/s")
        pv = final["phugoid"]["from_world_vz"]
        log(f"  phugoid (world_vz fit): period = {pv['period_s']:.3f} s   zeta = {pv['zeta']:.4f}")
        log(f"  acceptance ALL_PASS = {final['acceptance']['checks']['ALL_PASS']}")
    else:
        log("")
        log("NO sweep point converged within 3 V iterations (Part E branch A candidate - see findings).")

    return dict(runs=runs, run_table=run_table, tried_V=tried_V,
                converged=(converged is not None),
                elev_star_deg=elev_star, thr_star=thr_star, final_full_window=final)


# =============================================================================
def _dump(name, obj, log_lines):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    jp = os.path.join(RESULTS_DIR, f"ardupilot_longitudinal_equilibrium_{name}_result.json")
    lp = os.path.join(RESULTS_DIR, f"ardupilot_longitudinal_equilibrium_{name}_log.txt")
    with open(jp, "w") as f:
        json.dump(obj, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(lp, "w") as f:
        f.write("\n".join(log_lines) + "\n")
    print(f"  wrote {jp}")
    print(f"  wrote {lp}")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    if mode in ("c1", "all"):
        lines = []
        def log(m):
            print(m, flush=True)
            lines.append(m)
        log(f"FALCON V2 - ARDUPLANE_LONGITUDINAL_EQUILIBRIUM_AND_SINK_ROOT_CAUSE_VALIDATION")
        log(f"Part C.1 pure-Gazebo reference-point reproduction. {stamp}")
        log(f"World: {WORLD}")
        log(f"GZ_SIM_SYSTEM_PLUGIN_PATH = {os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']}")
        log("")
        r = run_c1(log)
        _dump("c1_reference", dict(stage="C1", timestamp=stamp, result=r), lines)

    if mode in ("sweep", "all"):
        lines = []
        def log(m):
            print(m, flush=True)
            lines.append(m)
        log(f"FALCON V2 - ARDUPLANE_LONGITUDINAL_EQUILIBRIUM_AND_SINK_ROOT_CAUSE_VALIDATION")
        log(f"Part C.2 bounded iterative sweep. {stamp}")
        log(f"World: {WORLD}")
        log("")
        sw = run_sweep(log)
        _dump("sweep", dict(stage="C2", timestamp=stamp, **sw), lines)

    return 0


if __name__ == "__main__":
    sys.exit(main())

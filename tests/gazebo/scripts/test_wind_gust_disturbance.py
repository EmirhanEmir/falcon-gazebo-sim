#!/usr/bin/env python3
"""
FALCON V2 - WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION (gazebo-testing, 2026-08-27).

Live-Gazebo validation of the new `plugins/wind/` FalconV2Wind System plugin
(`aerodynamics` specialist agent), which publishes a composed steady + 1-cosine
gust wind VELOCITY (world frame, m/s, "velocity of the air mass" convention -
see docs/source_of_truth/environment/WIND.md) on the EXISTING
`/model/falcon_v2/wind` topic that `AerodynamicsSystem`/`PropulsionSystem`
already correctly consume as `Vrel = Vbody - Vwind` (pre-existing,
CONFIRMED unmodified by this task via direct grep of both files).

No aircraft physics parameter (mass, CG, inertia, aerodynamic coefficient,
lookup table, control authority, motor/prop constant, actuator parameter) is
modified anywhere in this script - the wind plugin is a pure additive
disturbance-VELOCITY generator, never a direct force/wrench source (Part 6
below independently re-confirms zero AddWorldForce/AddWorldWrench calls in
plugins/wind/).

Nominal trim reused verbatim (NOT re-searched, per task brief) from
docs/test_results/2026-08-26_updated_powered_trim_high_deflection_validation.md:
throttle L=R=0.5010, elevator physical +4.50deg L=R, V~18.166 m/s,
alpha~2.473deg, ailerons/rudder neutral. U_HOLD/W_HOLD reused verbatim from
test_updated_powered_trim_high_deflection.py's own already-validated hold
target for this exact trim family.

Methodology reused verbatim, not reinvented:
  - hold_step() body-velocity/attitude proportional force/torque controller
    (aero_lib.py) - same technique as run_actuator_quasi_static() in
    test_updated_powered_trim_high_deflection.py for Part 2's static/held
    sign tests.
  - ActuatorCommander / ThrottleCommander / DiagSubscriber (actuator_lib.py,
    aero_lib.py, propulsion_lib.py) - real actuator-driven control surfaces,
    real propulsion, live diagnostics topics, never a hand-computed
    substitute for a live measurement.
  - Free 6-DOF release technique (hold briefly, then fully release, nothing
    else touches base_link) - same as test_updated_powered_trim_high_
    deflection.py Part 4 / test_engine_out_asymmetric_thrust.py Part 7.
  - Amplitude-trend boundedness classification (mid-third vs last-third
    windowed max|.|) - directly modeled on test_engine_out_asymmetric_
    thrust.py's thirds_mean_rate()/trend_label() "plateau vs continued
    growth" methodology (2026-08-27 validation-driven fix from that stage),
    adapted here to an AMPLITUDE (max|.|) basis rather than a signed MEAN
    basis, because this airframe's own documented baseline behavior at this
    exact trim is a lightly-damped, not fully-decaying-within-25s phugoid
    oscillation (2026-08-26 report Part 6) - a raw signed-mean comparison
    over an oscillatory signal can spuriously read as "near zero" in both
    windows regardless of whether the oscillation's amplitude is still
    growing, decaying, or steady, so the ENVELOPE (max|.|) is the physically
    meaningful basis for this specific baseline dynamic character.

New, wind-specific pieces added by this script only:
  - WindCommander: publishes /steady_cmd (gz.msgs.Vector3d, ticked every
    step - safe/idempotent, mirrors ActuatorCommander/ThrottleCommander's
    documented discovery-timing rationale) and /gust_cmd (gz.msgs.Double_V,
    fixed 6-field order, sent only during a short BURST WINDOW then never
    again - repeatedly re-sending a gust command after that window would
    keep re-arming/replacing the schedule per WIND.md sec 5/7, which is the
    opposite of what a single scheduled gust test needs). start_delay_s is
    recomputed fresh on every burst-window tick from the CURRENT tick's own
    time, so the resulting absolute target start time stays anchored
    regardless of exactly which burst tick is actually received/processed
    first by the plugin (see module comment in WindCommander.send_gust()).
  - WindTopicSubscriber: subscribes directly to the actual output topic
    `/model/falcon_v2/wind` - ground truth of what the plugin published,
    independent of what this script merely intended to command (catches any
    command-delivery/timing bug, not just a plugin-math bug).

No aircraft physics parameter is read for any purpose other than loading
existing, already-validated config/state (S, b, rho for the moment/qbar
cross-check only), and none is modified anywhere in this script.
"""
import json
import math
import os
import sys
import threading

import actuator_lib as ACT
import aero_lib as AL
import propulsion_lib as PL

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
WIND_PLUGIN_BUILD_DIR = os.path.join(REPO_ROOT, "plugins/wind/build")


def setup_env():
    """Must be called BEFORE `import gz.sim8` / creating any TestFixture.
    Extends actuator_lib.setup_env() (which already covers actuators +
    propulsion + aerodynamics build dirs) with the new wind plugin build
    dir."""
    ACT.setup_env()
    existing = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
        WIND_PLUGIN_BUILD_DIR + (":" + existing if existing else ""))


setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402
import yaml  # noqa: E402

RESULTS_DIR = f"{REPO_ROOT}/tests/gazebo/results"
WORLD = f"{REPO_ROOT}/tests/gazebo/worlds/falcon_v2_freefall_world.sdf"

WIND_TOPIC = "/model/falcon_v2/wind"
STEADY_CMD_TOPIC = "/model/falcon_v2/wind/steady_cmd"
GUST_CMD_TOPIC = "/model/falcon_v2/wind/gust_cmd"

# =============================================================================
# CURRENT VALIDATED NOMINAL TRIM - reused verbatim, NOT re-searched (task brief
# / docs/test_results/2026-08-26_updated_powered_trim_high_deflection_validation.md).
# =============================================================================
THROTTLE = 0.5010
ELEV_THETA_DEG = 4.50  # physical joint angle, L=R (delta_e_aero = elevator_sign*theta = -4.50deg)
# body-frame GROUND-relative velocity hold target - reused verbatim from
# test_updated_powered_trim_high_deflection.py (same trim family, V~18.16m/s,
# alpha~2.47deg; hold_step() controls GROUND velocity, never wind-relative -
# this is intentional and required for Part 2's "same ground speed, vary wind"
# design, which is exactly how the pre-registered predictions are phrased).
U_HOLD, W_HOLD = 18.14534, -0.78335

MASS = 5.9348  # kg, base_link mass, model/model.sdf - controller gain only
I_DIAG = (0.7284, 0.2507, 0.9523)  # kg*m^2, base_link diagonal inertia - controller gain only
KP_LIN = 150.0
KP_ANG_SETTLE = 400.0
KP_ANG_QSTATIC = 1500.0  # reused verbatim from test_updated_powered_trim_high_deflection.py / test_high_deflection_control_aero.py
ALTITUDE_M = 100.0
DIAG_HZ = 20.0  # all diagnostics topics, confirmed in model/model.sdf's own <diagnostics_rate_hz>

# Read-only reference geometry/environment (S, b, rho) for the Mx/Mz/qbar
# cross-checks in Part 2 - NEVER modified, loaded from the SAME source-of-truth
# YAML the live plugin itself loads.
with open(AL.CONFIG_YAML_PATH) as _f:
    _root = yaml.safe_load(_f)
REF_S = _root["reference_geometry"]["wing_area_S_m2"]
REF_B = _root["reference_geometry"]["wingspan_b_m"]
REF_RHO = _root["environment"]["air_density_rho_kg_m3"]


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
# Wind command/observe helpers
# =============================================================================
class WindCommander:
    """Publishes /steady_cmd (gz.msgs.Vector3d) every tick (safe/idempotent -
    OnSteadyCmd() is a plain overwrite, no time-based scheduling) and
    /gust_cmd (gz.msgs.Double_V, fixed 6-field order) ONLY when explicitly
    asked via send_gust() - the caller is responsible for calling this only
    during a short burst window and then never again (see module docstring)."""

    def __init__(self):
        import gz.transport13 as tp
        from gz.msgs10 import vector3d_pb2, double_v_pb2
        self._v3 = vector3d_pb2
        self._dv = double_v_pb2
        self.node = tp.Node()
        self.pub_steady = self.node.advertise(STEADY_CMD_TOPIC, vector3d_pb2.Vector3d)
        self.pub_gust = self.node.advertise(GUST_CMD_TOPIC, double_v_pb2.Double_V)
        self.steady = (0.0, 0.0, 0.0)

    def set_steady(self, x, y, z):
        self.steady = (x, y, z)

    def tick_steady(self):
        m = self._v3.Vector3d()
        m.x, m.y, m.z = self.steady
        self.pub_steady.publish(m)

    def send_gust(self, dirx, diry, dirz, amplitude, start_delay_s, duration_s):
        """Fixed field order per WIND.md sec 6.3 / WindSystem.cc OnGustCmd():
        [dir_x, dir_y, dir_z, amplitude_mps, start_delay_s, duration_s]."""
        m = self._dv.Double_V()
        for v in (dirx, diry, dirz, amplitude, start_delay_s, duration_s):
            m.data.append(v)
        self.pub_gust.publish(m)


class WindTopicSubscriber:
    """Subscribes directly to the ACTUAL output topic /model/falcon_v2/wind -
    ground truth of what the plugin published, independent of what this
    script merely intended to command."""

    def __init__(self):
        import gz.transport13 as tp
        self.node = tp.Node()
        self.lock = threading.Lock()
        self.history = []
        ok = self.node.subscribe_raw(
            WIND_TOPIC, self._cb, "gz.msgs.Vector3d", tp.SubscribeOptions())
        if not ok:
            raise RuntimeError(f"Failed to subscribe to {WIND_TOPIC}")

    def _cb(self, data, info):
        from gz.msgs10 import vector3d_pb2
        m = vector3d_pb2.Vector3d()
        m.ParseFromString(data)
        with self.lock:
            self.history.append((m.x, m.y, m.z))

    def latest(self):
        with self.lock:
            return self.history[-1] if self.history else None

    def count(self):
        with self.lock:
            return len(self.history)


# =============================================================================
# PART 6 (quick, independent) - grep confirmation that the wind plugin never
# applies a direct force/wrench (source-code fact-check, not a deep audit -
# that is `validation`'s job).
# =============================================================================
def part6_no_artificial_force_check(log):
    log("=" * 78)
    log("PART 6: NO-ARTIFICIAL-FORCE QUICK CONFIRMATION (independent grep)")
    log("=" * 78)
    import subprocess
    out = subprocess.run(
        ["grep", "-rn", "AddWorldForce\\|AddWorldWrench", f"{REPO_ROOT}/plugins/wind/",
         "--include=*.hh", "--include=*.cc"],
        capture_output=True, text=True)
    lines = [l for l in out.stdout.splitlines() if l.strip()]
    # Every hit found (if any) must be inside a comment (starts with // after
    # stripping whitespace, or is part of a /* */ block - this codebase only
    # ever uses // line comments for this kind of note, confirmed by direct
    # read of WindSystem.hh/.cc above) - i.e. a DOCUMENTATION reference to the
    # absence of the call, never an actual call site.
    code_hits = []
    for l in lines:
        # l format: "path:lineno:content"
        content = l.split(":", 2)[-1].strip()
        if not content.startswith("//"):
            code_hits.append(l)
    log(f"grep hits for AddWorldForce/AddWorldWrench in plugins/wind/*.hh,*.cc: {len(lines)} total, "
        f"{len(code_hits)} outside a // comment line (i.e. actual call sites).")
    for l in lines:
        log(f"  {l}")
    ok = (len(code_hits) == 0)
    log(f"CONFIRMED: zero actual AddWorldForce/AddWorldWrench call sites in plugins/wind/: {ok}\n")
    return dict(total_hits=len(lines), code_hits=len(code_hits), ok=ok, raw_lines=lines)


# =============================================================================
# PART 2 - static/held sign tests (also reused for Part 1's zero-wind case A).
# =============================================================================
STATIC_WARM_STEPS = 300
STATIC_SETTLE_STEPS = 2500
STATIC_TAIL_STEPS = 500


def run_static_case(log, label, steady_wind_world):
    cmd_rad = dict(left_elevator=math.radians(ELEV_THETA_DEG), right_elevator=math.radians(ELEV_THETA_DEG),
                   left_aileron=0.0, right_aileron=0.0, rudder=0.0)
    lin_target = gm.Vector3d(U_HOLD, 0.0, W_HOLD)
    total_steps = STATIC_WARM_STEPS + STATIC_SETTLE_STEPS + 5

    state = {"n": 0, "teleported": False, "cmd": None, "thr": None, "wind_cmd": None,
             "any_nan": False, "aero_diag": None, "wind_sub": None,
             "body_state": [], "wind_actual": []}

    def on_pre(info, ecm):
        model = get_model(ecm)
        base = sim.Link(model.link_by_name(ecm, "base_link"))
        base.enable_velocity_checks(ecm, True)
        if not state["teleported"]:
            model.set_world_pose_cmd(ecm, gm.Pose3d(0, 0, ALTITUDE_M, 0, 0, 0))
            state["teleported"] = True
            state["cmd"] = ACT.ActuatorCommander()
            state["thr"] = PL.ThrottleCommander()
            state["wind_cmd"] = WindCommander()
        state["thr"].set(left=THROTTLE, right=THROTTLE)
        state["thr"].tick()
        state["cmd"].set(**cmd_rad)
        state["cmd"].tick()
        state["wind_cmd"].set_steady(*steady_wind_world)
        state["wind_cmd"].tick_steady()
        AL.hold_step(base, ecm, MASS, I_DIAG, lin_target, gm.Vector3d(0, 0, 0),
                     kp_lin=KP_LIN, kp_ang=KP_ANG_QSTATIC)

    def on_post(info, ecm):
        if state["aero_diag"] is None:
            try:
                state["aero_diag"] = AL.DiagSubscriber()
            except Exception:
                pass
        if state["wind_sub"] is None:
            try:
                state["wind_sub"] = WindTopicSubscriber()
            except Exception:
                pass
        model = get_model(ecm)
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
            state["body_state"].append((lv_b.x(), lv_b.y(), lv_b.z(), lv.x(), lv.y(), lv.z(), rot))
        w = state["wind_sub"].latest() if state["wind_sub"] else None
        if w is not None:
            state["wind_actual"].append(w)
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    bs_tail = state["body_state"][-STATIC_TAIL_STEPS:]
    n_tail = len(bs_tail)
    u_g = sum(s[0] for s in bs_tail) / n_tail
    v_g = sum(s[1] for s in bs_tail) / n_tail
    w_g = sum(s[2] for s in bs_tail) / n_tail
    lvx = sum(s[3] for s in bs_tail) / n_tail
    lvy = sum(s[4] for s in bs_tail) / n_tail
    lvz = sum(s[5] for s in bs_tail) / n_tail
    rot_last = bs_tail[-1][6]

    # Independent mirror of Vrel = Vbody - Vwind (world frame subtraction,
    # THEN rotate to body - exactly AerodynamicsSystem.cc's own order) fed the
    # EXACT commanded wind vector - never a substitute for the live
    # measurement, only a cross-check.
    lv_world_avg = gm.Vector3d(lvx, lvy, lvz)
    wind_vec = gm.Vector3d(*steady_wind_world)
    vel_world_rel = lv_world_avg - wind_vec
    vel_body_rel = rot_last.rotate_vector_reverse(vel_world_rel)
    u_r, v_r, w_r = vel_body_rel.x(), vel_body_rel.y(), vel_body_rel.z()
    V_mirror = math.sqrt(u_r * u_r + v_r * v_r + w_r * w_r)
    alpha_mirror = AL.angle_of_attack(u_r, v_r, w_r)
    beta_mirror = AL.sideslip(u_r, v_r, w_r)

    aero_hist = state["aero_diag"].history if state["aero_diag"] else []
    tail_msgs = max(1, round(STATIC_TAIL_STEPS * AL.STEP * DIAG_HZ))
    aero_tail = aero_hist[-tail_msgs:] if aero_hist else []
    aero_avg = ({k: sum(m[k] for m in aero_tail) / len(aero_tail) for k in AL.DiagSubscriber.FIELDS}
                if aero_tail else {k: None for k in AL.DiagSubscriber.FIELDS})

    wind_actual_tail = state["wind_actual"][-STATIC_TAIL_STEPS:]
    wind_actual_avg = (tuple(sum(w[i] for w in wind_actual_tail) / len(wind_actual_tail) for i in range(3))
                       if wind_actual_tail else None)

    Mx = aero_avg["qbar"] * REF_S * REF_B * aero_avg["Cl"] if aero_avg["qbar"] is not None else None
    Mz = aero_avg["qbar"] * REF_S * REF_B * aero_avg["Cn"] if aero_avg["qbar"] is not None else None
    qbar_expected = 0.5 * REF_RHO * aero_avg["V"] ** 2 if aero_avg["V"] is not None else None
    qbar_reldiff = (abs(aero_avg["qbar"] - qbar_expected) / max(abs(qbar_expected), 1e-9)
                    if (aero_avg["qbar"] is not None and qbar_expected is not None) else None)

    result = dict(
        label=label, commanded_wind=list(steady_wind_world),
        wind_actual_published_avg=wind_actual_avg,
        ground_vel_body=(u_g, v_g, w_g),
        mirror_Vrel=V_mirror, mirror_alpha_deg=math.degrees(alpha_mirror), mirror_beta_deg=math.degrees(beta_mirror),
        live=aero_avg, Mx_Nm=Mx, Mz_Nm=Mz, qbar_expected=qbar_expected, qbar_reldiff=qbar_reldiff,
        any_nan=state["any_nan"])
    log(f"[{label}] wind_cmd={steady_wind_world} wind_actual_avg={wind_actual_avg} | "
        f"ground_vel_body=({u_g:.4f},{v_g:.4f},{w_g:.4f}) | "
        f"mirror: V={V_mirror:.4f} alpha={math.degrees(alpha_mirror):+.4f}deg beta={math.degrees(beta_mirror):+.4f}deg | "
        f"LIVE: V={aero_avg['V']:.4f} alpha={math.degrees(aero_avg['alpha']):+.4f}deg beta={math.degrees(aero_avg['beta']):+.4f}deg "
        f"qbar={aero_avg['qbar']:.4f}(expected {qbar_expected:.4f}, reldiff={qbar_reldiff:.5f}) "
        f"CL={aero_avg['CL']:.5f} CD={aero_avg['CD']:.5f} CY={aero_avg['CY']:.5f} "
        f"Cl={aero_avg['Cl']:.5f} Cm={aero_avg['Cm']:.5f} Cn={aero_avg['Cn']:.5f} | "
        f"Mx={Mx:+.4f}Nm Mz={Mz:+.4f}Nm any_nan={state['any_nan']}")
    return result


def sign_of(x, tol=1e-9):
    if x > tol:
        return "+"
    if x < -tol:
        return "-"
    return "0"


def run_part1_and_part2(log):
    log("=" * 78)
    log("PART 1 (partial) + PART 2: STATIC/HELD WIND SIGN TESTS")
    log("=" * 78)
    cases = [
        ("A_ZERO", (0.0, 0.0, 0.0)),
        ("B_HEADWIND", (-5.0, 0.0, 0.0)),
        ("C_TAILWIND", (5.0, 0.0, 0.0)),
        ("D_CROSSWIND_PLUS_Y", (0.0, 5.0, 0.0)),
        ("E_CROSSWIND_MINUS_Y", (0.0, -5.0, 0.0)),
        ("F_VERTICAL_PLUS_Z", (0.0, 0.0, 3.0)),
        ("G_VERTICAL_MINUS_Z", (0.0, 0.0, -3.0)),
    ]
    results = {}
    for label, vec in cases:
        results[label] = run_static_case(log, label, vec)
    log("")

    base = results["A_ZERO"]["live"]
    log("-" * 78)
    log("PRE-REGISTERED SIGN-CHECK TABLE (delta relative to A_ZERO baseline)")
    log("-" * 78)
    checks = {}

    def rec(case, quantity, delta, expect_sign_str, note=""):
        obs_sign = sign_of(delta)
        agree = (obs_sign == expect_sign_str) if expect_sign_str != "?" else None
        checks.setdefault(case, []).append(dict(quantity=quantity, delta=delta, observed_sign=obs_sign,
                                                  expected_sign=expect_sign_str, agree=agree, note=note))
        log(f"  [{case}] {quantity}: delta={delta:+.6f} observed_sign={obs_sign} expected_sign={expect_sign_str} "
            f"-> {'AGREE' if agree else ('N/A' if agree is None else 'DISAGREE')} {note}")

    # 1/2: headwind/tailwind -> V_rel (live 'V') and qbar increase/decrease
    b = results["B_HEADWIND"]["live"]
    rec("B_HEADWIND", "V_rel", b["V"] - base["V"], "+")
    rec("B_HEADWIND", "qbar", b["qbar"] - base["qbar"], "+")
    c = results["C_TAILWIND"]["live"]
    rec("C_TAILWIND", "V_rel", c["V"] - base["V"], "-")
    rec("C_TAILWIND", "qbar", c["qbar"] - base["qbar"], "-")

    # 3: crosswind +Y -> beta negative, CY positive, Cn negative, Cl unconstrained
    d = results["D_CROSSWIND_PLUS_Y"]["live"]
    rec("D_CROSSWIND_PLUS_Y", "beta", d["beta"] - base["beta"], "-")
    rec("D_CROSSWIND_PLUS_Y", "CY", d["CY"] - base["CY"], "+")
    rec("D_CROSSWIND_PLUS_Y", "Cn", d["Cn"] - base["Cn"], "-")
    rec("D_CROSSWIND_PLUS_Y", "Cl", d["Cl"] - base["Cl"], "?", note="(NOT pre-registered - report only)")
    e = results["E_CROSSWIND_MINUS_Y"]["live"]
    rec("E_CROSSWIND_MINUS_Y", "beta", e["beta"] - base["beta"], "+")
    rec("E_CROSSWIND_MINUS_Y", "CY", e["CY"] - base["CY"], "-")
    rec("E_CROSSWIND_MINUS_Y", "Cn", e["Cn"] - base["Cn"], "+")
    rec("E_CROSSWIND_MINUS_Y", "Cl", e["Cl"] - base["Cl"], "?", note="(NOT pre-registered - report only)")

    # 4: vertical +Z -> alpha increases, CL increases; -Z mirror
    f = results["F_VERTICAL_PLUS_Z"]["live"]
    rec("F_VERTICAL_PLUS_Z", "alpha", f["alpha"] - base["alpha"], "+")
    rec("F_VERTICAL_PLUS_Z", "CL", f["CL"] - base["CL"], "+")
    g = results["G_VERTICAL_MINUS_Z"]["live"]
    rec("G_VERTICAL_MINUS_Z", "alpha", g["alpha"] - base["alpha"], "-")
    rec("G_VERTICAL_MINUS_Z", "CL", g["CL"] - base["CL"], "-")

    # 5: qbar = 0.5*rho*Vrel^2 for every case (already computed per-case as qbar_reldiff)
    log("")
    log("qbar = 0.5*rho*|V_rel|^2 scaling check (all cases):")
    max_qbar_reldiff = 0.0
    for label, _ in cases:
        rd = results[label]["qbar_reldiff"]
        max_qbar_reldiff = max(max_qbar_reldiff, rd)
        log(f"  [{label}] qbar_reldiff={rd:.6f}")
    log(f"max qbar_reldiff across all 7 cases: {max_qbar_reldiff:.6f}\n")

    any_disagree = any(c["agree"] is False for lst in checks.values() for c in lst)
    return dict(cases=results, sign_checks=checks, max_qbar_reldiff=max_qbar_reldiff, any_disagree=any_disagree)


# =============================================================================
# PART 1 (remainder) - short zero-wind free-flight regression snippet.
# =============================================================================
FF_HOLD_STEPS = 800
FF_TELEMETRY_EVERY = 50  # 0.05s, aligned to the 20Hz diagnostics publish rate


def run_free_flight(log, label, release_steps, steady_wind_world=(0.0, 0.0, 0.0),
                     gust=None, gust_burst_start_step=100, gust_burst_len=300):
    """gust = dict(dir=(x,y,z), amplitude=A, target_start_s=<absolute sim
    seconds from t=0 of THIS run>, duration=T) or None. See module docstring
    for the burst-window-then-stop gust-commanding rationale."""
    cmd_rad = dict(left_elevator=math.radians(ELEV_THETA_DEG), right_elevator=math.radians(ELEV_THETA_DEG),
                   left_aileron=0.0, right_aileron=0.0, rudder=0.0)
    total_steps = FF_HOLD_STEPS + release_steps

    state = {"n": 0, "teleported": False, "cmd": None, "thr": None, "wind_cmd": None, "wind_sub": None,
             "any_nan": False, "nan_step": None, "series": [], "prop_diag": None, "aero_diag": None,
             "act_diag": None, "gust_sent": False, "gust_send_log": []}

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
            state["wind_cmd"] = WindCommander()
        state["thr"].set(left=THROTTLE, right=THROTTLE)
        state["thr"].tick()
        state["cmd"].set(**cmd_rad)
        state["cmd"].tick()
        state["wind_cmd"].set_steady(*steady_wind_world)
        state["wind_cmd"].tick_steady()
        if gust is not None and not state["gust_sent"] and gust_burst_start_step <= n < gust_burst_start_step + gust_burst_len:
            command_time_s = n * AL.STEP
            start_delay_s = gust["target_start_s"] - command_time_s
            state["wind_cmd"].send_gust(gust["dir"][0], gust["dir"][1], gust["dir"][2], gust["amplitude"],
                                         start_delay_s, gust["duration"])
            state["gust_send_log"].append((n, command_time_s, start_delay_s))
            if n == gust_burst_start_step + gust_burst_len - 1:
                state["gust_sent"] = True
        if n < FF_HOLD_STEPS:
            AL.hold_step(base, ecm, MASS, I_DIAG, gm.Vector3d(U_HOLD, 0, W_HOLD), gm.Vector3d(0, 0, 0),
                         kp_lin=KP_LIN, kp_ang=KP_ANG_SETTLE)
        # else: base_link COMPLETELY free - no hold, no stabilizer, controls held at trim only.

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
        if state["act_diag"] is None:
            try:
                state["act_diag"] = ACT.DiagSubscriber()
            except Exception:
                pass
        if state["wind_sub"] is None:
            try:
                state["wind_sub"] = WindTopicSubscriber()
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
        raw = [lv_b.x(), lv_b.y(), lv_b.z(), av_b.x(), av_b.y(), av_b.z(), wpose.pos().z()]
        if any(math.isnan(x) or math.isinf(x) for x in raw):
            state["any_nan"] = True
            if state["nan_step"] is None:
                state["nan_step"] = n
        if n >= FF_HOLD_STEPS and (n - FF_HOLD_STEPS) % FF_TELEMETRY_EVERY == 0:
            roll, pitch, yaw = quat_rpy(rot)
            V_ground = math.sqrt(lv_b.x() ** 2 + lv_b.y() ** 2 + lv_b.z() ** 2)
            aero = state["aero_diag"].latest() if state["aero_diag"] else None
            prop = state["prop_diag"].latest() if state["prop_diag"] else None
            act = state["act_diag"].latest() if state["act_diag"] else None
            wind_now = state["wind_sub"].latest() if state["wind_sub"] else None
            state["series"].append(dict(
                t=(n - FF_HOLD_STEPS) * AL.STEP, alt=wpose.pos().z(), world_vz=lv.z(),
                roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch), yaw_deg=math.degrees(yaw),
                p_deg_s=math.degrees(av_b.x()), q_deg_s=math.degrees(av_b.y()), r_deg_s=math.degrees(av_b.z()),
                V_ground=V_ground,
                V_rel=(aero["V"] if aero else None),
                alpha_rel_deg=(math.degrees(aero["alpha"]) if aero else None),
                beta_rel_deg=(math.degrees(aero["beta"]) if aero else None),
                qbar=(aero["qbar"] if aero else None), CL=(aero["CL"] if aero else None),
                CD=(aero["CD"] if aero else None), CY=(aero["CY"] if aero else None),
                Cl=(aero["Cl"] if aero else None), Cm=(aero["Cm"] if aero else None),
                Cn=(aero["Cn"] if aero else None),
                wind_x=(wind_now[0] if wind_now else None), wind_y=(wind_now[1] if wind_now else None),
                wind_z=(wind_now[2] if wind_now else None),
                left_rpm=(prop["left"]["rpm"] if prop else None), right_rpm=(prop["right"]["rpm"] if prop else None),
                left_thrust_N=(prop["left"]["thrust_N"] if prop else None),
                right_thrust_N=(prop["right"]["thrust_N"] if prop else None),
                elev_L_actual_deg=(math.degrees(act["left_elevator"]["actual_angle_rad"]) if act else None),
                elev_L_setpoint_deg=(math.degrees(act["left_elevator"]["setpoint_rad"]) if act else None),
                elev_R_actual_deg=(math.degrees(act["right_elevator"]["actual_angle_rad"]) if act else None),
                elev_R_setpoint_deg=(math.degrees(act["right_elevator"]["setpoint_rad"]) if act else None),
                aile_L_actual_deg=(math.degrees(act["left_aileron"]["actual_angle_rad"]) if act else None),
                aile_R_actual_deg=(math.degrees(act["right_aileron"]["actual_angle_rad"]) if act else None),
                rudder_actual_deg=(math.degrees(act["rudder"]["actual_angle_rad"]) if act else None),
                any_target_clamp=(any(act[s]["target_clamp_active"] > 0.5 for s in ACT.SURFACES) if act else None),
                any_effort_clamp=(any(act[s]["effort_clamp_active"] > 0.5 for s in ACT.SURFACES) if act else None),
            ))
        state["n"] += 1

    fixture = sim.TestFixture(WORLD)
    fixture.on_pre_update(on_pre)
    fixture.on_post_update(on_post)
    fixture.finalize()
    server = fixture.server()
    server.run(True, total_steps, False)

    series = state["series"]

    def amplitude_trend(key, floor=1.0):
        vals = [s[key] for s in series if s[key] is not None]
        n = len(vals)
        if n < 6:
            return dict(mid=None, last=None, ratio=None, label="INSUFFICIENT_DATA")
        i_mid, i_last = n // 3, (2 * n) // 3
        mid, last = vals[i_mid:i_last], vals[i_last:]
        a_mid = max(abs(x) for x in mid)
        a_last = max(abs(x) for x in last)
        if a_mid < floor and a_last < floor:
            return dict(mid=a_mid, last=a_last, ratio=None, label="NEGLIGIBLE_AMPLITUDE_SETTLED")
        ratio = (a_last / a_mid) if a_mid > 1e-9 else (float("inf") if a_last > 1e-9 else 0.0)
        if ratio < 0.7:
            lbl = "AMPLITUDE_DECAYING"
        elif ratio > 1.4:
            lbl = "AMPLITUDE_STILL_GROWING"
        else:
            lbl = "AMPLITUDE_STEADY_OR_PLATEAU"
        return dict(mid=a_mid, last=a_last, ratio=ratio, label=lbl)

    trend = {k: amplitude_trend(k) for k in ("p_deg_s", "q_deg_s", "r_deg_s")}

    # Actuator tracking-error / clamp / finiteness summary (Part 5).
    def track_err(actual_key, setpoint_key):
        vals = [abs(s[actual_key] - s[setpoint_key]) for s in series
                if s[actual_key] is not None and s[setpoint_key] is not None]
        return max(vals) if vals else None

    elev_L_max_err = track_err("elev_L_actual_deg", "elev_L_setpoint_deg")
    elev_R_max_err = track_err("elev_R_actual_deg", "elev_R_setpoint_deg")
    any_clamp_ever = any(s["any_target_clamp"] or s["any_effort_clamp"] for s in series
                         if s["any_target_clamp"] is not None)
    all_actuator_vals = [s[k] for s in series for k in
                         ("elev_L_actual_deg", "elev_R_actual_deg", "aile_L_actual_deg",
                          "aile_R_actual_deg", "rudder_actual_deg") if s[k] is not None]
    actuator_all_finite = all(math.isfinite(v) for v in all_actuator_vals) if all_actuator_vals else None

    start, end = (series[0], series[-1]) if series else (None, None)
    summary = dict(
        any_nan=state["any_nan"], nan_step=state["nan_step"],
        start=start, end=end,
        max_abs_roll=max((abs(s["roll_deg"]) for s in series), default=None),
        max_abs_pitch=max((abs(s["pitch_deg"]) for s in series), default=None),
        max_abs_alpha_rel=max((abs(s["alpha_rel_deg"]) for s in series if s["alpha_rel_deg"] is not None), default=None),
        max_abs_beta_rel=max((abs(s["beta_rel_deg"]) for s in series if s["beta_rel_deg"] is not None), default=None),
        v_ground_range=(min((s["V_ground"] for s in series), default=None), max((s["V_ground"] for s in series), default=None)),
        v_rel_range=(min((s["V_rel"] for s in series if s["V_rel"] is not None), default=None),
                     max((s["V_rel"] for s in series if s["V_rel"] is not None), default=None)),
        alt_drift=(end["alt"] - start["alt"]) if (start and end) else None,
        trend=trend,
        elev_L_max_tracking_err_deg=elev_L_max_err, elev_R_max_tracking_err_deg=elev_R_max_err,
        any_actuator_clamp_ever=any_clamp_ever, actuator_all_finite=actuator_all_finite,
        gust_send_log=state["gust_send_log"],
    )

    any_still_growing = any(trend[k]["label"] == "AMPLITUDE_STILL_GROWING" for k in trend)
    classification = "FAIL" if state["any_nan"] else (
        "INCONCLUSIVE_NEEDS_EXTENSION" if any_still_growing else "BOUNDED")

    log(f"[{label}] any_nan={state['any_nan']} (first@{state['nan_step']}) | "
        f"V_ground=[{summary['v_ground_range'][0]:.3f},{summary['v_ground_range'][1]:.3f}] "
        f"V_rel=[{summary['v_rel_range'][0]:.3f},{summary['v_rel_range'][1]:.3f}] | "
        f"max|roll|={summary['max_abs_roll']:.3f}deg max|pitch|={summary['max_abs_pitch']:.3f}deg "
        f"max|alpha_rel|={summary['max_abs_alpha_rel']:.3f}deg max|beta_rel|={summary['max_abs_beta_rel']:.3f}deg "
        f"alt_drift={summary['alt_drift']:+.3f}m")
    log(f"[{label}] TREND: p={trend['p_deg_s']['label']}(ratio={trend['p_deg_s']['ratio']}) "
        f"q={trend['q_deg_s']['label']}(ratio={trend['q_deg_s']['ratio']}) "
        f"r={trend['r_deg_s']['label']}(ratio={trend['r_deg_s']['ratio']})")
    log(f"[{label}] ACTUATOR: elev_L_max_track_err={elev_L_max_err:.5f}deg elev_R_max_track_err={elev_R_max_err:.5f}deg "
        f"any_clamp_ever={any_clamp_ever} all_finite={actuator_all_finite}")
    log(f"[{label}] CLASSIFICATION: {classification}\n")

    return dict(label=label, series=series, summary=summary, classification=classification,
                release_steps=release_steps, steady_wind_world=list(steady_wind_world), gust=gust)


def run_with_extension(log, label, release_steps, steady_wind_world=(0.0, 0.0, 0.0),
                        gust=None, gust_burst_start_step=100, gust_burst_len=300, max_extensions=1,
                        extension_steps=10000):
    """Runs run_free_flight(); if the result is INCONCLUSIVE_NEEDS_EXTENSION
    (a rate amplitude still growing at cutoff - see module docstring's
    methodology-lesson note), reruns FRESH with a longer window (matching
    test_engine_out_asymmetric_thrust.py's own fresh-rerun-not-resume
    precedent), up to max_extensions times."""
    attempt = 0
    steps = release_steps
    result = run_free_flight(log, f"{label}_attempt{attempt}", steps, steady_wind_world,
                              gust, gust_burst_start_step, gust_burst_len)
    while result["classification"] == "INCONCLUSIVE_NEEDS_EXTENSION" and attempt < max_extensions:
        attempt += 1
        steps += extension_steps
        log(f"[{label}] EXTENDING: prior attempt inconclusive (rate still growing at cutoff) - "
            f"re-running fresh with release_steps={steps} ({steps * AL.STEP:.1f}s)")
        result = run_free_flight(log, f"{label}_attempt{attempt}", steps, steady_wind_world,
                                  gust, gust_burst_start_step, gust_burst_len)
    result["extension_attempts"] = attempt
    return result


def run_part1_free_flight_regression(log):
    log("=" * 78)
    log("PART 1 (remainder): SHORT ZERO-WIND FREE-FLIGHT REGRESSION SNIPPET")
    log("=" * 78)
    log("Wind plugin LOADED but never commanded (default state) - published wind must be exactly "
        "(0,0,0) at every tick (bit-identical to the prior always-zero default, WIND.md sec 8).")
    r = run_free_flight(log, "ZERO_WIND_REGRESSION", 8000, steady_wind_world=(0.0, 0.0, 0.0))
    wind_all_zero = all(
        (s["wind_x"] == 0.0 and s["wind_y"] == 0.0 and s["wind_z"] == 0.0)
        for s in r["series"] if s["wind_x"] is not None)
    s = r["summary"]
    # Broad envelope check vs the historical 25s baseline (2026-08-26 report,
    # Part 6): V 17.4-18.7, pitch damping through a -6..+0.3 range, alpha
    # bounded 2.47-2.75, roll<=0.10, yaw drift<=0.7 OVER THE FULL 25s window -
    # this 8s snippet is a SUBSET of that window, so a modest margin is used
    # (not tightened, not loosened beyond what the shorter/earlier-only
    # portion of that same trace could plausibly show).
    envelope_ok = (
        17.2 <= s["v_rel_range"][0] and s["v_rel_range"][1] <= 18.9 and
        s["max_abs_roll"] <= 0.20 and
        1.8 <= (s["end"]["alpha_rel_deg"] if s["end"] else 999) <= 4.0 and
        not s["any_nan"])
    log(f"wind_actual_all_zero_throughout={wind_all_zero}")
    log(f"REGRESSION envelope check (broad margin around 2026-08-26 25s baseline): {envelope_ok}\n")
    return dict(free_flight=r, wind_all_zero=wind_all_zero, envelope_ok=envelope_ok)


# =============================================================================
# PART 3 - steady-wind free-flight tests.
# =============================================================================
def run_part3(log):
    log("=" * 78)
    log("PART 3: STEADY-WIND FREE-FLIGHT TESTS (~12-13s, from nominal trim, fully free 6-DOF)")
    log("=" * 78)
    cases = {
        "A_HEADWIND_5MPS": (-5.0, 0.0, 0.0),
        "B_TAILWIND_5MPS": (5.0, 0.0, 0.0),
        "C_CROSSWIND_5MPS": (0.0, 5.0, 0.0),
    }
    out = {}
    for label, vec in cases.items():
        log(f"-- {label}: steady wind (world) = {vec} --")
        out[label] = run_with_extension(log, label, 13000, steady_wind_world=vec)
    return out


# =============================================================================
# PART 4 - gust tests.
# =============================================================================
def run_part4(log):
    log("=" * 78)
    log("PART 4: GUST TESTS (1-cosine profile via /gust_cmd, ~12-13s window, from nominal trim)")
    log("=" * 78)
    # target_start_s measured from t=0 of the RUN (i.e. includes the FF_HOLD_STEPS=0.8s hold phase)
    # -> gust actually begins at (target_start_s - 0.8)s AFTER release, giving ~1.2s of clean settled
    # free flight before the disturbance, per task Part 4 window guidance.
    cases = {
        "LONGITUDINAL_M5": dict(dir=(1.0, 0.0, 0.0), amplitude=-5.0, target_start_s=2.0, duration=2.0),
        "LATERAL_P5": dict(dir=(0.0, 1.0, 0.0), amplitude=5.0, target_start_s=2.0, duration=2.0),
        "VERTICAL_P3": dict(dir=(0.0, 0.0, 1.0), amplitude=3.0, target_start_s=2.0, duration=2.0),
        "LONGITUDINAL_P8_SANITY": dict(dir=(1.0, 0.0, 0.0), amplitude=8.0, target_start_s=2.0, duration=2.0),
    }
    out = {}
    for label, g in cases.items():
        log(f"-- {label}: gust dir={g['dir']} amplitude={g['amplitude']:+.1f}m/s "
            f"target_start_s={g['target_start_s']:.2f} duration={g['duration']:.1f}s --")
        out[label] = run_with_extension(log, label, 13000, steady_wind_world=(0.0, 0.0, 0.0), gust=g)
    return out


# =============================================================================
# Main
# =============================================================================
def strip_series(d, keep_series=True):
    if keep_series:
        return d
    d = dict(d)
    d.pop("series", None)
    return d


def main():
    log_lines = []

    def log(msg):
        print(msg, flush=True)
        log_lines.append(msg)

    log("FALCON V2 - WIND_GUST_DISTURBANCE_MODEL_AND_VALIDATION (gazebo-testing, 2026-08-27)")
    log(f"Nominal trim (reused, NOT re-searched): throttle L=R={THROTTLE} elevator_theta_deg={ELEV_THETA_DEG:+.2f} "
        f"U_HOLD={U_HOLD} W_HOLD={W_HOLD}")
    log("")

    part6 = part6_no_artificial_force_check(log)

    part1_2 = run_part1_and_part2(log)
    part1_ff = run_part1_free_flight_regression(log)
    part3 = run_part3(log)
    part4 = run_part4(log)

    any_nan_overall = (
        any(part1_2["cases"][c]["any_nan"] for c in part1_2["cases"]) or
        part1_ff["free_flight"]["summary"]["any_nan"] or
        any(part3[c]["summary"]["any_nan"] for c in part3) or
        any(part4[c]["summary"]["any_nan"] for c in part4))

    log("=" * 78)
    log("OVERALL")
    log("=" * 78)
    log(f"any_nan_overall={any_nan_overall}")
    log(f"part2_any_disagree_vs_preregistered_signs={part1_2['any_disagree']}")
    log(f"part6_no_artificial_force_ok={part6['ok']}")

    result = dict(
        part6_no_artificial_force=part6,
        part1_and_part2_static=dict(
            cases=part1_2["cases"], sign_checks=part1_2["sign_checks"],
            max_qbar_reldiff=part1_2["max_qbar_reldiff"], any_disagree=part1_2["any_disagree"]),
        part1_free_flight_regression=part1_ff,
        part3_steady_free_flight=part3,
        part4_gust=part4,
        any_nan_overall=any_nan_overall,
    )

    os.makedirs(RESULTS_DIR, exist_ok=True)
    with open(f"{RESULTS_DIR}/wind_gust_disturbance_result.json", "w") as f:
        json.dump(result, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(f"{RESULTS_DIR}/wind_gust_disturbance_log.txt", "w") as f:
        f.write("\n".join(log_lines) + "\n")

    return not any_nan_overall


if __name__ == "__main__":
    ok = main()
    sys.exit(0 if ok else 1)

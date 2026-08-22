#!/usr/bin/env python3
"""
FALCON V2 - shared helper library for the AERODYNAMICS_V1 live-Gazebo test
suite (gazebo-testing, 2026-08-22).

This module is imported by tests/gazebo/scripts/test_aero_*.py. It provides:

  1. Environment/plugin-path setup for loading the built
     plugins/aerodynamics/build/libFalconV2Aerodynamics.so into a
     gz.sim8.TestFixture-driven server (same mechanism documented in
     plugins/aerodynamics/README.md, applied here instead to the Python
     TestFixture API rather than a standalone `gz sim` process).

  2. A PURE-PYTHON re-implementation of the exact math in
     plugins/aerodynamics/AeroModel.hh (AngleOfAttack, Sideslip,
     WindToBodyForce, SaturatedCL, ComputeAero), loading the SAME
     docs/source_of_truth/aerodynamics/aero_v1_config.yaml the plugin loads.
     This is used ONLY as an independent cross-check of the live plugin's
     published diagnostics (never to "fix" or replace a live measurement,
     and never as a substitute for actually reading real ECM/link state) -
     see CLAUDE.md / this suite's test reports for how each test combines
     this with a live measurement.

  3. A raw gz-transport subscriber for the plugin's diagnostics topic
     (/model/falcon_v2/aerodynamics/diagnostics, gz.msgs.Double_V), using
     the double_v_pb2 module directly (subscribe_raw + manual parse) since
     this environment's gz.msgs10 Python package does not re-export
     compiled message classes through its normal import path.

  4. A small helper for driving a gz.sim8.TestFixture through a two-phase
     "hold steady condition, then release one degree of freedom and
     measure the real physics response" scenario - the core technique used
     by this suite's dynamic sign tests (Cma_RESTORING_SIGN_TEST,
     Cmq/Clp/Cnr_DAMPING_SIGN_TEST, Cnb_STATIC_STABILITY_SIGN_TEST,
     AILERON/RUDDER/ELEVATOR sign tests, LIFT/DRAG_SIGN_TEST). Rationale
     for this technique (verified empirically this pass, see test report):
     gz-sim's Link.set_linear_velocity()/set_angular_velocity() suppress
     the physics engine's use of any externally-queued wrench (i.e. the
     plugin's AddWorldForce/AddWorldWrench calls) for whichever DOF
     (translational and/or rotational) is overridden that step - so
     holding one DOF fixed while leaving another DOF un-overridden lets
     that other DOF evolve under the REAL applied aerodynamic force/moment,
     which can then be measured directly from the resulting change in
     linear/angular velocity - not merely inferred from the diagnostics
     topic's published coefficients.

No aircraft physics parameter is modified anywhere in this file.
"""
import math
import os
import threading

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
PLUGIN_BUILD_DIR = os.path.join(REPO_ROOT, "plugins/aerodynamics/build")
CONFIG_YAML_PATH = os.path.join(
    REPO_ROOT, "docs/source_of_truth/aerodynamics/aero_v1_config.yaml")
WORLD_SDF = os.path.join(
    REPO_ROOT, "tests/gazebo/worlds/falcon_v2_zero_g_world.sdf")
DIAG_TOPIC = "/model/falcon_v2/aerodynamics/diagnostics"

STEP = 0.001  # s, matches falcon_v2_zero_g_world.sdf's <max_step_size>


def setup_env():
    """Must be called BEFORE `import gz.sim8` / creating any TestFixture in
    the calling script, so the plugin .so is discoverable and so the gz
    Python protobuf bindings (compiled against an older protoc) load
    correctly against this environment's newer `protobuf` pip package."""
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    existing = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
        PLUGIN_BUILD_DIR + (":" + existing if existing else ""))


# =============================================================================
# Independent pure-python re-implementation of plugins/aerodynamics/AeroModel.hh
# Mirrors the C++ header line-for-line (see that file for full derivation
# comments) - used ONLY as a cross-check tool, never as ground truth over a
# live measurement.
# =============================================================================
class AeroConfig:
    pass


def load_config():
    import yaml
    with open(CONFIG_YAML_PATH) as f:
        root = yaml.safe_load(f)

    cfg = AeroConfig()
    rg = root["reference_geometry"]
    cfg.S = rg["wing_area_S_m2"]
    cfg.b = rg["wingspan_b_m"]
    cfg.c_ref = rg["reference_chord_c_ref_m"]

    env = root["environment"]
    cfg.rho = env["air_density_rho_kg_m3"]
    cfg.vSafeFloor = env["v_safe_floor_m_s"]

    lon = root["longitudinal"]
    cfg.CLa = lon["CLa_per_rad"]
    cfg.Cma = lon["Cma_per_rad"]
    cfg.CLq = lon["CLq"]
    cfg.Cmq = lon["Cmq"]
    cfg.Cmde = lon["Cmde_per_rad"]
    cfg.CL0 = lon["CL0"]
    cfg.Cm0 = lon["Cm0"]

    lat = root["lateral_directional"]
    cfg.CYb = lat["CYb"]
    cfg.CYp = lat["CYp"]
    cfg.CYr = lat["CYr"]
    cfg.CYda = lat["CYda_per_rad"]
    cfg.CYdr = lat["CYdr_per_rad"]
    cfg.Clb = lat["Clb"]
    cfg.Clp = lat["Clp"]
    cfg.Clr = lat["Clr"]
    cfg.Clda = lat["Clda_per_rad"]
    cfg.Cldr = lat["Cldr_per_rad"]
    cfg.Cnb = lat["Cnb"]
    cfg.Cnp = lat["Cnp"]
    cfg.Cnr = lat["Cnr"]
    cfg.Cnda = lat["Cnda_per_rad"]
    cfg.Cndr = lat["Cndr_per_rad"]

    drag = root["drag_polar"]
    cfg.CD0 = drag["CD0"]
    cfg.dragK = drag["k"]

    lim = root["high_alpha_limiter"]
    cfg.CLmax = lim["CLmax_manufacturer"]
    cfg.alphaTransition = math.radians(lim["alpha_transition_deg"])

    ctrl = root["control_mapping"]
    cfg.elevatorSign = ctrl["elevator_sign"]
    cfg.aileronSign = ctrl["aileron_sign"]
    cfg.rudderSign = ctrl["rudder_sign"]
    cfg.controlDeflectionClamp = math.radians(ctrl["control_deflection_clamp_deg"])

    # AeroConfig::Prepare() mirror
    clLinAtT = cfg.CL0 + cfg.CLa * cfg.alphaTransition
    cfg.satHeadroomPos = cfg.CLmax - clLinAtT
    cfg.satKPos = (cfg.CLa / cfg.satHeadroomPos) if cfg.satHeadroomPos > 1e-9 else 0.0

    clLinAtNegT = cfg.CL0 + cfg.CLa * (-cfg.alphaTransition)
    cfg.satAneg = clLinAtNegT + cfg.CLmax
    cfg.satKNeg = (cfg.CLa / cfg.satAneg) if cfg.satAneg > 1e-9 else 0.0

    return cfg


def angle_of_attack(u, v, w):
    return math.atan2(-w, u)


def sideslip(u, v, w):
    return math.atan2(v, math.hypot(u, w))


def saturated_CL(cfg, alpha):
    clLinear = cfg.CL0 + cfg.CLa * alpha
    if alpha > cfg.alphaTransition:
        return cfg.CLmax - cfg.satHeadroomPos * math.exp(-cfg.satKPos * (alpha - cfg.alphaTransition))
    elif alpha < -cfg.alphaTransition:
        return -cfg.CLmax + cfg.satAneg * math.exp(cfg.satKNeg * (alpha + cfg.alphaTransition))
    return clLinear


def wind_to_body_force(lift, drag, sideForce, alpha, beta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    fx = -drag * ca * cb - sideForce * ca * sb + lift * sa
    fy = -drag * sb + sideForce * cb
    fz = drag * sa * cb + sideForce * sa * sb + lift * ca
    return fx, fy, fz


def compute_aero(cfg, u, v, w, p, q, r, deltaA=0.0, deltaE=0.0, deltaR=0.0):
    """Pure-python mirror of AeroModel.hh::ComputeAero(). Returns a dict with
    the same fields as the plugin's AeroOutput plus forceBody/momentBody.

    UPDATED 2026-08-22 (post-Cma-fix re-test pass) to mirror the scoped
    Cm-to-My sign correction and the CLq*qHat addition applied to
    AeroModel.hh this pass (see that file's "RESOLVED FINDING" comment):
      - out['Cm'] (diagnostics-comparable) stays in XFLR5's own UNFLIPPED
        convention: Cm = cmStatic + cmRate, cmStatic = Cm0+Cma*alpha+Cmde*deltaE,
        cmRate = Cmq*qHat - exactly as before, unchanged.
      - out['My'] (the actual applied torque) now uses
        qbar*S*c_ref*(-cmStatic + cmRate) - only the static/angle-derived
        group is negated for the FLU +Y axis mapping; the self-referential
        rate group is left alone.
      - out['CL'] now includes the previously-missing CLq*qHat rate term,
        added UNSATURATED on top of the saturated alpha-driven static term.
    """
    vSq = u * u + v * v + w * w
    V = math.sqrt(vSq)
    qbar = 0.5 * cfg.rho * vSq

    alpha = angle_of_attack(u, v, w)
    beta = sideslip(u, v, w)

    vSafe = max(V, cfg.vSafeFloor)
    pHat = p * cfg.b / (2.0 * vSafe)
    qHat = q * cfg.c_ref / (2.0 * vSafe)
    rHat = r * cfg.b / (2.0 * vSafe)

    def clamp(d):
        return max(-cfg.controlDeflectionClamp, min(cfg.controlDeflectionClamp, d))

    deltaA = clamp(deltaA)
    deltaE = clamp(deltaE)
    deltaR = clamp(deltaR)

    CY = cfg.CYb * beta + cfg.CYp * pHat + cfg.CYr * rHat + cfg.CYda * deltaA + cfg.CYdr * deltaR
    Cl = cfg.Clb * beta + cfg.Clp * pHat + cfg.Clr * rHat + cfg.Clda * deltaA + cfg.Cldr * deltaR
    Cn = cfg.Cnb * beta + cfg.Cnp * pHat + cfg.Cnr * rHat + cfg.Cnda * deltaA + cfg.Cndr * deltaR

    cmStatic = cfg.Cm0 + cfg.Cma * alpha + cfg.Cmde * deltaE
    cmRate = cfg.Cmq * qHat
    Cm = cmStatic + cmRate  # XFLR5's own (unflipped) convention - diagnostics-comparable

    CL = saturated_CL(cfg, alpha) + cfg.CLq * qHat
    CD = cfg.CD0 + cfg.dragK * CL * CL

    lift = qbar * cfg.S * CL
    drag = qbar * cfg.S * CD
    sideForce = qbar * cfg.S * CY

    fx, fy, fz = wind_to_body_force(lift, drag, sideForce, alpha, beta)

    mx = qbar * cfg.S * cfg.b * Cl
    my = qbar * cfg.S * cfg.c_ref * (-cmStatic + cmRate)  # scoped sign correction
    mz = qbar * cfg.S * cfg.b * Cn

    return dict(V=V, alpha=alpha, beta=beta, qbar=qbar, CL=CL, CD=CD, CY=CY,
                Cl=Cl, Cm=Cm, Cn=Cn, cmStatic=cmStatic, cmRate=cmRate,
                pHat=pHat, qHat=qHat, rHat=rHat,
                Fx=fx, Fy=fy, Fz=fz, Mx=mx, My=my, Mz=mz)


# =============================================================================
# Diagnostics topic subscriber
# =============================================================================
class DiagSubscriber:
    """Subscribes to /model/falcon_v2/aerodynamics/diagnostics and stores
    every received message (field order per AerodynamicsSystem.cc:
    V, alpha_rad, beta_rad, qbar, CL, CD, CY, Cl, Cm, Cn)."""

    FIELDS = ["V", "alpha", "beta", "qbar", "CL", "CD", "CY", "Cl", "Cm", "Cn"]

    def __init__(self):
        import gz.transport13 as tp
        self._tp = tp
        self.node = tp.Node()
        self.lock = threading.Lock()
        self.history = []
        ok = self.node.subscribe_raw(
            DIAG_TOPIC, self._cb, "gz.msgs.Double_V", tp.SubscribeOptions())
        if not ok:
            raise RuntimeError(f"Failed to subscribe to {DIAG_TOPIC}")

    def _cb(self, data, info):
        from gz.msgs10 import double_v_pb2
        m = double_v_pb2.Double_V()
        m.ParseFromString(data)
        vals = list(m.data)
        with self.lock:
            self.history.append(dict(zip(self.FIELDS, vals)))

    def latest(self):
        with self.lock:
            return dict(self.history[-1]) if self.history else None

    def count(self):
        with self.lock:
            return len(self.history)


# =============================================================================
# Real-physics "hold at a target body-frame condition" controller.
#
# IMPORTANT, found empirically this pass (see test report): gz.sim8's
# Link.set_linear_velocity()/set_angular_velocity() (Cmd-based velocity
# override) were tried FIRST and abandoned - confirmed, via a dedicated
# smoke test, to put the link into a state where, once the calling script
# stops invoking the setter, the link's reported position/velocity FREEZES
# bit-exactly (no further physics integration at all, for any DOF, even
# ones never Cmd-driven) rather than resuming normal dynamics. This makes
# Cmd-based velocity override unusable for "hold a condition, then release
# and observe the REAL applied force/moment" testing.
#
# This function instead holds a target body-frame linear/angular velocity
# using a plain proportional-force/torque CONTROLLER applied via
# Link.add_world_force()/add_world_wrench() - the SAME primitives the
# aerodynamics plugin itself uses (AeroModel.hh's output is applied via
# these exact calls in AerodynamicsSystem.cc), so this technique never
# creates any special/frozen link state: it is just another ordinary
# external force/torque, summed by the physics engine with whatever the
# aero plugin also applies that step, exactly like two real force sources
# would combine in reality. Setting a target to None skips controlling
# that DOF for this call (used to "release" one DOF while continuing to
# hold the other, isolating the real physics response being measured).
# =============================================================================
def hold_step(base, ecm, mass, i_eff, body_lin_target, body_ang_target,
               kp_lin=150.0, kp_ang=150.0, ang_axis_mask=(True, True, True)):
    """`i_eff` may be a single scalar (uniform gain on all 3 rotational axes)
    or a (Ixx, Iyy, Izz) tuple - using the REAL, runtime-queried diagonal
    inertia components (see read_base_link_inertia()) gives much more even,
    faster convergence across roll/pitch/yaw than one uniform scalar, since
    base_link's real inertia is markedly anisotropic (Ixx=0.7284, Iyy=0.2507,
    Izz=0.9523 kg*m^2, model/model.sdf) - found empirically this pass (a
    uniform scalar under-drives whichever axis has the largest real inertia,
    e.g. roll, leaving a much larger steady-state residual on that axis).

    `ang_axis_mask` = (control_x, control_y, control_z): per-axis switch for
    the angular controller, used to "release" only ONE rotational DOF (e.g.
    pitch for a Cmq test) while continuing to actively hold the other two -
    isolating the real physics response on just the released axis instead of
    letting all 3 evolve freely (which would let e.g. roll/yaw cross-coupling
    or the (documented, separately-investigated) Cma static-instability
    contaminate a pure-rate-damping measurement)."""
    import gz.math7 as gm
    wpose = base.world_pose(ecm)
    lv = base.world_linear_velocity(ecm)
    av = base.world_angular_velocity(ecm)
    if wpose is None or lv is None or av is None:
        return None, None, None
    rot = wpose.rot()
    lv_b = rot.rotate_vector_reverse(lv)
    av_b = rot.rotate_vector_reverse(av)
    if body_lin_target is not None:
        e_lin = body_lin_target - lv_b
        f_world = rot.rotate_vector(e_lin * (kp_lin * mass))
        base.add_world_force(ecm, f_world)
    if body_ang_target is not None:
        e_ang = body_ang_target - av_b
        if isinstance(i_eff, (tuple, list)):
            ixx, iyy, izz = i_eff
        else:
            ixx = iyy = izz = i_eff
        mx, my, mz = ang_axis_mask
        t_body = gm.Vector3d(
            e_ang.x() * ixx * (1.0 if mx else 0.0),
            e_ang.y() * iyy * (1.0 if my else 0.0),
            e_ang.z() * izz * (1.0 if mz else 0.0)) * kp_ang
        t_world = rot.rotate_vector(t_body)
        base.add_world_wrench(ecm, gm.Vector3d(0, 0, 0), t_world)
    return lv_b, av_b, rot


# =============================================================================
# Pin the 7 lightweight/undamped child joints (5 control surfaces + 2 props).
#
# IMPORTANT, found empirically this pass (see test report): under a
# SUSTAINED, actively-held nonzero base_link body rate (p, q, or r), the 5
# control-surface joints - documented elsewhere as TEMPORARY_NUMERICAL_MASS
# = 1 g links with NO actuator and NO joint damping in this structural-V1
# build - were observed to spin up to extreme relative joint velocities
# (tens of rad/s, repeatedly slamming into their +/-30deg mechanical limits)
# purely as a kinematic consequence of being dragged around by the rotating
# base through joint axes not aligned with the rotation axis. The resulting
# joint-limit-impact reaction forces feed back onto base_link and were
# confirmed (by a dedicated before/after comparison, see report) to
# contaminate a rate-damping measurement by up to ~14x in magnitude and even
# flip its sign - a test-harness artifact of holding a sustained rate on an
# otherwise-free-floating structural-V1 airframe with no control-surface
# actuation yet, NOT an aerodynamics-plugin defect (confirmed: pinning these
# joints removes the artifact and recovers the expected magnitude/sign, see
# report). This helper pins all 7 child joints' position AND velocity to a
# fixed value (default 0) every step, isolating base_link's own dynamics for
# these specific measurements. Never used to alter model.sdf or any aircraft
# physics parameter - purely a test-harness technique, exactly analogous to
# this suite's pre-existing use of a zero-gravity test world for isolation.
# =============================================================================
CHILD_JOINTS = ["left_aileron_joint", "right_aileron_joint", "left_elevator_joint",
                "right_elevator_joint", "rudder_joint", "left_prop_joint", "right_prop_joint"]


def pin_child_joints(model, ecm, sim, positions=None):
    positions = positions or {}
    for jn in CHILD_JOINTS:
        je = model.joint_by_name(ecm, jn)
        if je is None:
            continue
        j = sim.Joint(je)
        j.reset_position(ecm, [positions.get(jn, 0.0)])
        j.reset_velocity(ecm, [0.0])


# =============================================================================
# Body-frame inertia tensor readback (queried dynamically at test start, never
# hardcoded - matches the documented model.sdf base_link <inertia> block, used
# only to convert a measured angular-velocity change into an approximate
# applied-moment ESTIMATE for reporting; the primary PASS/FAIL evidence is
# always the raw sign/direction of the measured velocity change itself).
# =============================================================================
def read_base_link_inertia(model, ecm, sim):
    base = sim.Link(model.link_by_name(ecm, "base_link"))
    inertial = base.world_inertial(ecm)
    mm = inertial.mass_matrix()
    return dict(mass=mm.mass(), ixx=mm.ixx(), iyy=mm.iyy(), izz=mm.izz(),
                ixy=mm.ixy(), ixz=mm.ixz(), iyz=mm.iyz())

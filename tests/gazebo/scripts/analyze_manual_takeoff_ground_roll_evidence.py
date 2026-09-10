#!/usr/bin/env python3
"""FALCON V2 - independent, OFFLINE re-analysis of the Part 4 manual-takeoff
ground-roll timeseries.

CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY.
Stage: MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION.
Owner: gazebo-testing. Created 2026-09-10.

CHANGES NO PHYSICS PARAMETER. It reads
tests/gazebo/results/manual_takeoff_ground_roll_run*_*_timeseries.json and
nothing else. No SDF, no plugin, no config, no ArduPilot parameter, no
aerodynamic/propulsion coefficient, no mass/CG/inertia, no actuator limit is
read for modification or written anywhere.

WHY THIS EXISTS (three measured harness defects, all in the ANALYSIS layer -
the CAPTURED DATA itself is fine and is reused verbatim)
=============================================================================
D1  LIFTOFF DETECTOR ALIASES TO ZERO.
    test_manual_takeoff_ground_roll_reproduction.py computes the vertical
    rate by differencing consecutive SAMPLES (~95 Hz), but the pose it
    differences arrives on /world/<w>/dynamic_pose/info at ~50-60 Hz. Two
    consecutive samples therefore frequently carry the IDENTICAL pose, and
    the finite difference is EXACTLY 0.0 for that sample. The primary liftoff
    criterion requires `vz > 0.5` to hold CONTINUOUSLY for 0.30 s, so a
    single aliased-to-zero sample resets the run and the criterion never
    fires - every one of the six runs was reported
    NO_LIFTOFF_WITHIN_WINDOW even though the aircraft demonstrably reached
    >100 m altitude. This module differences the pose stream on its OWN
    distinct-sample grid instead, which removes the alias. The criterion
    itself (0.10 m margin, 0.5 m/s, 0.30 s hold) is UNCHANGED - only the
    signal it is evaluated on is de-aliased.
D2  ROTATION-VS-POPOFF CLASSIFICATION IS DOWNSTREAM OF D1.
    With no liftoff timestamp, "pitch change BEFORE the vertical excursion"
    was computed over the ENTIRE record including the post-departure
    excursion, yielding ~90 deg and declaring "rotation then fly" in every
    run. Recomputed here against the de-aliased liftoff time.
D3  INFERRED SUPPORT FORCE WAS LABELLED "MEASURED".
    manual_takeoff_live_lib.ContactSub sets `available = node.subscribe(...)`,
    and gz-transport's subscribe() returns True when the SUBSCRIPTION is
    registered - it does NOT require a publisher to exist. The runway world
    attaches no gz-sim-contact-system and model/model.sdf declares no contact
    sensor, so no message can ever arrive, yet `contact_topic_available` came
    back True and the report labelled the support force "MEASURED". The
    NUMBERS were always the inferred ones; only the label was wrong. This
    module verifies that no contact message was ever recorded and labels the
    quantity INFERRED / DATA_REQUIRED everywhere.

ALSO ADDED HERE, because it was documented as done but is not implemented
anywhere in the Part 1/2/4 harness:
    THE AERO-DEFLECTION CROSS-CHECK. manual_takeoff_live_lib's
    AERO_DEFLECTION_NOT_PUBLISHED block states that the reconstructed
    delta_a/delta_e/delta_r are "INDEPENDENTLY CROSS-CHECKED against the
    published Cl/Cm/Cn via aero_lib.compute_aero()". No script calls
    compute_aero(). It is done here: the plugin's own published V, alpha,
    beta and qbar plus the reconstructed deflections and the measured body
    rates are pushed through aero_lib's pure-python mirror of the plugin's
    ComputeAero(), and the residual against the published Cl/Cm/Cn is
    reported.

NET PROPELLER REACTION TORQUE - SIGN NOTE
=========================================
PropulsionSystem.cc applies, per motor,
    reactionTorqueBodyX_i = rotation_sign_i * (-Q_prop_signed_i)
with rotation_sign left=+1.0, right=-1.0
(docs/source_of_truth/propulsion/propulsion_v1_config.yaml "rotation" block).
The net body-X (roll-axis) reaction torque is therefore
    Mx_reaction = -Q_prop_left + Q_prop_right = Q_prop_right - Q_prop_left
The Part 4 harness reports `Q_prop_sum_Nm = Q_prop_left + Q_prop_right` and
labels it "the NET propeller reaction torque about the body roll axis". Under
the counter-rotating sign convention above that label is wrong: the SUM is
not the net body-X torque, the DIFFERENCE is. Both are reported here, each
under its own name, so the reviewer can see which is which.
The net body-Z (yaw-axis) moment from the thrust pair is the r x F moment of
the two hub offsets (left hub y=+0.3000 m, right hub y=-0.3000 m, model.sdf /
GEOMETRY.md sec 7), with thrust along body +X:
    Mz_thrust = y_left*(-T_left) + y_right*(-T_right)   [ (r x F)_z = -y*Fx ]
              = 0.3000 * (T_right - T_left)
In FLU a POSITIVE Mz is a NOSE-LEFT moment.

FRAME CONVENTIONS USED IN THIS REPORT (stated, never mixed)
===========================================================
  Gazebo world (this runway world): +X along the runway centreline in the
  nose direction at spawn, +Y to the aircraft's LEFT, +Z up (ENU-like).
  The centreline is Y = 0 and the spawn is (-150, 0, 0.5).
  => LATERAL DISPLACEMENT = world Y, and POSITIVE Y = LEFT.
  Gazebo body frame: FLU.
  MAVLink ATTITUDE rates: FRD. p_FLU = +p_FRD, q_FLU = -q_FRD, r_FLU = -r_FRD.
  A nose-right yaw is a NEGATIVE rotation about FLU +Z and a POSITIVE
  yawspeed in MAVLink FRD.

USAGE
    python3 analyze_manual_takeoff_ground_roll_evidence.py
"""
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import manual_takeoff_live_lib as L  # noqa: E402

RES = L.RESULTS_DIR
PREFIX = "manual_takeoff_ground_roll"

# Detection constants - IDENTICAL to the Part 4 harness's own TH block, quoted
# here so the de-aliased recomputation uses the SAME criterion and only the
# signal differs.
LIFTOFF_Z_MARGIN_M = 0.10
LIFTOFF_VZ_MIN_MS = 0.5
LIFTOFF_HOLD_S = 0.30
YAW_RATE_ONSET_DEG_S = 1.0
SURFACE_ONSET_DEG = 0.5
ROTATION_PITCH_DELTA_DEG = 1.0

# Geometry, READ from their documented sources - not chosen here.
HUB_Y_LEFT_M = 0.3000       # model.sdf left_prop <pose>, GEOMETRY.md sec 7
HUB_Y_RIGHT_M = -0.3000     # model.sdf right_prop <pose>
MASS_KG = 6.000             # CLAUDE.md "Aircraft mass"
WING_AREA_M2 = 0.4514       # CLAUDE.md "Wing area"
G = 9.81                    # runway world <gravity>
WEIGHT_N = MASS_KG * G

# Smallest surface command this transport can express, DERIVED in
# manual_takeoff_live_lib from model.sdf's own multiplier and servo range.
PWM_QUANTUM_DEG = L.PWM_QUANTUM_DEG

# Servo rate limit, READ from docs/source_of_truth/controls/
# actuator_v1_config.yaml servo.max_rate_rad_s (V1_PROVISIONAL, = 300 deg/s).
# Used only to count how often the reported actual_rate sits ON that limit.
MAX_RATE_RAD_S = 5.2360


def mean(xs):
    xs = [x for x in xs if isinstance(x, (int, float)) and not math.isnan(x)]
    return sum(xs) / len(xs) if xs else float("nan")


def std(xs):
    xs = [x for x in xs if isinstance(x, (int, float)) and not math.isnan(x)]
    if len(xs) < 2:
        return float("nan")
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def spread(xs):
    xs = [x for x in xs if isinstance(x, (int, float)) and not math.isnan(x)]
    if not xs:
        return None
    return {"values": xs, "mean": mean(xs), "min": min(xs), "max": max(xs),
            "spread": max(xs) - min(xs), "std": std(xs)}


# ---------------------------------------------------------------------------
# D1: de-aliased vertical rate
# ---------------------------------------------------------------------------
def dealiased_vertical_rate(samples):
    """Return {index -> vz} computed on the DISTINCT pose grid.

    The pose stream repeats the same transform across consecutive samples
    (publisher ~50-60 Hz, sampler ~95 Hz). Differencing consecutive SAMPLES
    yields exactly 0.0 whenever the pose repeated. Here the difference is
    taken between successive DISTINCT poses and held until the next distinct
    pose arrives - the same quantity, without the alias."""
    vz = {}
    last_i = None
    last_t = None
    last_z = None
    cur = None
    for i, s in enumerate(samples):
        z = s["gz_pos_world_m"][2]
        t = s["t_wall"]
        if last_z is None:
            last_i, last_t, last_z = i, t, z
            vz[i] = None
            continue
        if z != last_z:
            dt = t - last_t
            cur = (z - last_z) / dt if dt > 0 else None
            last_i, last_t, last_z = i, t, z
        vz[i] = cur
    return vz


def find_liftoff(samples, resting_z, vz_map):
    """The Part 4 primary criterion, unchanged, on the de-aliased rate."""
    run_start = None
    for i, s in enumerate(samples):
        if s["phase"] in ("settle", "armed_dwell"):
            continue
        z = s["gz_pos_world_m"][2]
        v = vz_map.get(i)
        if v is not None and z > resting_z + LIFTOFF_Z_MARGIN_M and v > LIFTOFF_VZ_MIN_MS:
            if run_start is None:
                run_start = i
            elif s["t_wall"] - samples[run_start]["t_wall"] >= LIFTOFF_HOLD_S:
                return run_start
        else:
            run_start = None
    return None


def first_index(samples, pred, start=0):
    for i in range(start, len(samples)):
        try:
            if pred(samples[i]):
                return i
        except (TypeError, KeyError, IndexError):
            continue
    return None


def event(samples, i, t0, raw_signal, vz_map=None):
    if i is None:
        return None
    s = samples[i]
    vfr = s.get("vfr") or {}
    att = s.get("attitude_mav") or {}
    return {
        "index": i,
        "t_rel_s": s["t_wall"] - t0,
        "t_sim": s["t_sim"],
        "raw_signal": raw_signal,
        "airspeed_ms": vfr.get("airspeed"),
        "groundspeed_ms": vfr.get("groundspeed"),
        "throttle_pct": vfr.get("throttle_pct"),
        "gz_z_m": s["gz_pos_world_m"][2],
        "gz_vz_ms_dealiased": (vz_map or {}).get(i),
        "lateral_Y_m": s["lateral_displacement_m_world_Y"],
        "roll_deg": s["gz_rpy_deg"][0],
        "pitch_deg": s["gz_rpy_deg"][1],
        "yaw_deg": s["gz_rpy_deg"][2],
        "yawspeed_deg_s_MAVLINK_FRD": att.get("yawspeed_deg_s"),
        "lift_N": s.get("lift_N"),
        "thrust_total_N": s.get("thrust_total_N"),
        "normal_force_N_INFERRED": s.get("normal_force_N_INFERRED"),
    }


# ---------------------------------------------------------------------------
# Aero deflection cross-check (documented in the harness, never implemented)
# ---------------------------------------------------------------------------
def aero_cross_check(samples, lo_i):
    import aero_lib as AL
    try:
        cfg = AL.load_config()
    except Exception as e:  # config API drift must be reported, not hidden
        return {
            "status": "BLOCKED",
            "reason": repr(e),
            "interpretation":
                "aero_lib.load_config() cannot parse the CURRENT "
                "docs/source_of_truth/aerodynamics/aero_v1_config.yaml. The "
                "key control_deflection_clamp_deg was RETIRED from that "
                "config on 2026-08-26 (task "
                "HIGH_DEFLECTION_CONTROL_AERO_IMPLEMENTATION) and "
                "AerodynamicsSystem.cc stopped reading it, but "
                "tests/gazebo/scripts/aero_lib.py line 138 still requires "
                "it. aero_lib's pure-python ComputeAero mirror therefore "
                "cannot be loaded at all today, which is why NO script in "
                "this stage actually performs the cross-check its docstrings "
                "describe.",
            "status_label": "DATA_REQUIRED",
            "owner_for_fix": "aerodynamics",
            "not_fixed_here_because":
                "Re-synchronising aero_lib.py with the current aerodynamic "
                "model is an aerodynamics-domain change (the model FORM also "
                "moved on: CLde_per_rad and breakpoint-table interpolation "
                "were added in the same pass), not a mechanical test-harness "
                "repair. gazebo-testing does not make that change.",
        }
    resid_cl, resid_cm, resid_cn = [], [], []
    n = 0
    end = lo_i if lo_i is not None else len(samples)
    for s in samples[:end]:
        a = s.get("aero")
        att = s.get("attitude_mav")
        if not a or a.get("V") is None or att is None:
            continue
        V = a["V"]
        if V < 1.0:
            continue  # below the plugin's own low-speed floor: not meaningful
        alpha, beta, qbar = a["alpha"], a["beta"], a["qbar"]
        # reconstruct u,v,w consistent with aero_lib's own alpha/beta defs
        u = V * math.cos(alpha) * math.cos(beta)
        v = V * math.sin(beta)
        w = -V * math.sin(alpha) * math.cos(beta)
        # MAVLink ATTITUDE rates are FRD; the aero model is FLU
        p = math.radians(att["rollspeed_deg_s"])
        q = -math.radians(att["pitchspeed_deg_s"])
        r = -math.radians(att["yawspeed_deg_s"])
        d = s["aero_input_deg"]
        got = AL.compute_aero(cfg, u, v, w, p, q, r,
                              deltaA=math.radians(d["delta_a"]),
                              deltaE=math.radians(d["delta_e"]),
                              deltaR=math.radians(d["delta_r"]))
        resid_cl.append(got["Cl"] - a["Cl"])
        resid_cm.append(got["Cm"] - a["Cm"])
        resid_cn.append(got["Cn"] - a["Cn"])
        n += 1
    if n == 0:
        return {"status": "NOT_RUN", "reason": "no sample above the 1 m/s floor"}

    def stat(xs):
        return {"max_abs": max(abs(x) for x in xs), "mean": mean(xs),
                "rms": math.sqrt(sum(x * x for x in xs) / len(xs))}
    return {
        "status": "RUN",
        "n_samples": n,
        "window": "throttle onset .. liftoff (ground roll only)",
        "residual_Cl": stat(resid_cl),
        "residual_Cm": stat(resid_cm),
        "residual_Cn": stat(resid_cn),
        "method": "aero_lib.compute_aero() fed with the aerodynamics "
                  "plugin's OWN published V/alpha/beta and the deflections "
                  "reconstructed from the actuator plugin's actual joint "
                  "angles via CONTROLS.md sec 10, compared against the same "
                  "message's published Cl/Cm/Cn. Body rates come from "
                  "MAVLink ATTITUDE (FRD) converted to FLU, which is an "
                  "EKF-filtered estimate rather than the plugin's own "
                  "instantaneous rate, so a small residual from the rate "
                  "terms is expected and is NOT evidence of a mapping error.",
    }


# ---------------------------------------------------------------------------
def analyse_run(path):
    with open(path, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    S = doc["samples"]
    mode = doc["mode"]
    idx = doc["run_index"]
    t0 = S[0]["t_wall"]

    settle = [s for s in S if s["phase"] == "settle"]
    # PRE_ROLL_PHASES: phases in which the aircraft is stationary with the
    # throttle not yet commanded. "armed_dwell" only exists on the TAKEOFF
    # path (armed in MANUAL, waiting out the user's measured 9.70 s dwell
    # before mode 13 is commanded); excluding it keeps the ground-roll window
    # meaning the same across all three modes. MANUAL/FBWA timeseries contain
    # no armed_dwell sample, so their analysis is bit-identical to before.
    PRE_ROLL_PHASES = ("settle", "armed_dwell")
    roll_i0 = next((i for i, s in enumerate(S)
                    if s["phase"] not in PRE_ROLL_PHASES), 0)
    resting_z = mean([s["gz_pos_world_m"][2] for s in settle[len(settle) // 2:]])
    resting_pitch = mean([s["gz_rpy_deg"][1] for s in settle[len(settle) // 2:]])

    vz = dealiased_vertical_rate(S)
    lo_i = find_liftoff(S, resting_z, vz)
    t_lift = (S[lo_i]["t_wall"] - t0) if lo_i is not None else None

    # ---- the four liftoff criteria, all recomputed on the same grid ----
    lift_cross_i = first_index(S, lambda s: s.get("lift_N") is not None
                               and s["lift_N"] >= WEIGHT_N, roll_i0)
    nzero_i = first_index(S, lambda s: s.get("normal_force_N_INFERRED") is not None
                          and s["normal_force_N_INFERRED"] <= 0.0, roll_i0)
    baro_i = first_index(S, lambda s: (s.get("gpi") or {}).get("relative_alt_m") is not None
                         and s["gpi"]["relative_alt_m"] > LIFTOFF_Z_MARGIN_M, roll_i0)
    criteria = {
        "primary_z_and_vz_DEALIASED": event(S, lo_i, t0,
            "world Z from dynamic_pose/info + vertical rate differenced on "
            "the DISTINCT pose grid (see D1)", vz),
        "aero_lift_equals_weight": event(S, lift_cross_i, t0,
            f"L = CL*qbar*S from the aerodynamics plugin's own diagnostics "
            f"crossing m*g = {WEIGHT_N:.2f} N", vz),
        "inferred_support_force_reaches_zero": event(S, nzero_i, t0,
            "INFERRED support force (DATA_REQUIRED: no contact sensor)", vz),
        "mavlink_relative_alt_leaves_ground": event(S, baro_i, t0,
            "MAVLink GLOBAL_POSITION_INT.relative_alt", vz),
    }

    # ---- ground-roll window ----
    thr_i = first_index(S, lambda s: (s.get("vfr") or {}).get("throttle_pct", 0) > 0,
                        roll_i0)
    gr_end = lo_i if lo_i is not None else len(S)
    ground = S[roll_i0:gr_end]

    # ---- pitch while in contact ----
    pitches = [s["gz_rpy_deg"][1] for s in ground]
    pitch_stats = {
        "window": "throttle onset .. liftoff (in contact)",
        "n": len(pitches),
        "resting_pitch_deg": resting_pitch,
        "min_deg": min(pitches) if pitches else None,
        "max_deg": max(pitches) if pitches else None,
        "mean_deg": mean(pitches),
        "std_deg": std(pitches),
        "max_abs_change_from_resting_deg": max((abs(p - resting_pitch) for p in pitches),
                                               default=None),
        "rotation_threshold_deg": ROTATION_PITCH_DELTA_DEG,
    }
    pitch_stats["exceeded_rotation_threshold_before_liftoff"] = bool(
        pitch_stats["max_abs_change_from_resting_deg"] is not None
        and pitch_stats["max_abs_change_from_resting_deg"] >= ROTATION_PITCH_DELTA_DEG)

    # ---- lateral drift, and the decisive onset ordering ----
    yaw_mav_i = first_index(S, lambda s: s.get("attitude_mav") is not None
                            and abs(s["attitude_mav"]["yawspeed_deg_s"]) >= YAW_RATE_ONSET_DEG_S,
                            roll_i0)
    # gz-side yaw departure, independent of the EKF
    yaw_gz_i = first_index(S, lambda s: abs(s["gz_rpy_deg"][2]) >= 0.1, roll_i0)
    lat_i = {}
    for thr in (0.01, 0.1, 1.0):
        lat_i[thr] = first_index(
            S, lambda s, t=thr: abs(s["lateral_displacement_m_world_Y"]) >= t, roll_i0)
    surf_any_i = first_index(
        S, lambda s: max(abs(s["surface_cmd_deg"][k]) for k in L.SURFACES) > 0.0,
        roll_i0)
    surf_thr_i = first_index(
        S, lambda s: max(abs(s["surface_cmd_deg"][k])
                         for k in ("left_aileron", "right_aileron", "rudder"))
        >= SURFACE_ONSET_DEG, roll_i0)
    surf_quantum_i = first_index(
        S, lambda s: max(abs(s["surface_cmd_deg"][k])
                         for k in ("left_aileron", "right_aileron", "rudder"))
        >= PWM_QUANTUM_DEG, roll_i0)
    y_at_lift = S[lo_i]["lateral_displacement_m_world_Y"] if lo_i is not None else None
    ground_y = [s["lateral_displacement_m_world_Y"] for s in ground]
    drift = {
        "convention": "lateral displacement is Gazebo world Y; the runway "
                      "centreline is Y=0 and the nose points along world +X "
                      "at spawn, so POSITIVE Y = LEFT.",
        "y_at_liftoff_m": y_at_lift,
        "y_at_liftoff_direction": (None if y_at_lift is None else
                                   "LEFT (+Y)" if y_at_lift > 0 else
                                   "RIGHT (-Y)" if y_at_lift < 0 else "NONE"),
        "max_abs_y_during_ground_roll_m": max((abs(v) for v in ground_y), default=None),
        "y_at_end_of_ground_roll_m": ground_y[-1] if ground_y else None,
        "final_y_of_whole_record_m": S[-1]["lateral_displacement_m_world_Y"],
        "max_abs_y_of_whole_record_m": max(abs(s["lateral_displacement_m_world_Y"])
                                           for s in S),
        "yaw_rate_onset_MAVLINK": event(S, yaw_mav_i, t0,
            f"MAVLink ATTITUDE.yawspeed >= {YAW_RATE_ONSET_DEG_S} deg/s", vz),
        "yaw_angle_departure_GZ_0p1deg": event(S, yaw_gz_i, t0,
            "Gazebo world yaw from dynamic_pose/info >= 0.1 deg", vz),
        "surface_command_onset_ANY_NONZERO": event(S, surf_any_i, t0,
            "any of the five actuator cmd_rad != 0", vz),
        "surface_command_onset_1_PWM_QUANTUM": event(S, surf_quantum_i, t0,
            f"aileron/rudder cmd >= one PWM quantum ({PWM_QUANTUM_DEG:.4f} deg)", vz),
        "surface_command_onset_0p5deg": event(S, surf_thr_i, t0,
            f"aileron/rudder cmd >= {SURFACE_ONSET_DEG} deg", vz),
        "lateral_displacement_onset": {str(k): event(S, v, t0,
            f"|world Y| >= {k} m", vz) for k, v in lat_i.items()},
    }
    yaw_t = drift["yaw_rate_onset_MAVLINK"]
    srf_t = drift["surface_command_onset_1_PWM_QUANTUM"]
    if yaw_t and srf_t:
        drift["which_came_first"] = ("SURFACE_COMMAND"
                                     if srf_t["t_rel_s"] < yaw_t["t_rel_s"]
                                     else "YAW_RATE")
        drift["onset_separation_s"] = yaw_t["t_rel_s"] - srf_t["t_rel_s"]
    elif yaw_t and not srf_t:
        drift["which_came_first"] = "YAW_RATE (no surface command ever occurred)"
        drift["onset_separation_s"] = None
    elif srf_t and not yaw_t:
        drift["which_came_first"] = "SURFACE_COMMAND (yaw rate never reached the threshold)"
        drift["onset_separation_s"] = None
    else:
        drift["which_came_first"] = "NEITHER OCCURRED"
        drift["onset_separation_s"] = None

    # ---- propulsion asymmetry, over the ground roll ----
    def series(key, seq):
        return [s[key] for s in seq if s.get(key) is not None]
    rpm_d = series("rpm_diff_left_minus_right", ground)
    th_d = series("thrust_diff_left_minus_right_N", ground)
    ql = series("Q_prop_left_Nm", ground)
    qr = series("Q_prop_right_Nm", ground)
    mx_react = [b - a for a, b in zip(ql, qr)]           # = Qr - Ql, see docstring
    q_sum = [a + b for a, b in zip(ql, qr)]
    mz_thrust = [HUB_Y_LEFT_M * (-tl) + HUB_Y_RIGHT_M * (-tr)
                 for tl, tr in zip(series("thrust_left_N", ground),
                                   series("thrust_right_N", ground))]
    tt = series("thrust_total_N", ground)
    propulsion = {
        "window": "throttle onset .. liftoff",
        "rpm_diff_left_minus_right": spread_stats(rpm_d),
        "thrust_diff_left_minus_right_N": spread_stats(th_d),
        "thrust_diff_max_abs_pct_of_total": (
            100.0 * max((abs(v) for v in th_d), default=0.0) / max(tt)) if tt else None,
        "Q_prop_left_Nm": spread_stats(ql),
        "Q_prop_right_Nm": spread_stats(qr),
        "net_reaction_torque_about_body_X_Nm": spread_stats(mx_react),
        "net_reaction_torque_about_body_X_formula":
            "Qr - Ql  (rotation_sign left=+1, right=-1; "
            "reaction = rotation_sign * (-Q_prop))",
        "Q_prop_SUM_Nm_as_reported_by_the_harness": spread_stats(q_sum),
        "Q_prop_SUM_label_note":
            "The Part 4 harness calls this SUM 'the NET propeller reaction "
            "torque about the body roll axis'. Under the counter-rotating "
            "sign convention it is not; the DIFFERENCE above is. Both are "
            "given so the mislabel cannot propagate silently.",
        "net_thrust_yaw_moment_about_body_Z_Nm": spread_stats(mz_thrust),
        "net_thrust_yaw_moment_formula":
            "0.3000 m * (T_right - T_left); FLU, POSITIVE = NOSE LEFT",
        "thrust_total_N": spread_stats(tt),
    }

    # ---- surfaces: commanded vs actual over the ground roll ----
    surfaces = {}
    for s_name in L.SURFACES:
        cmds = [x["surface_cmd_deg"][s_name] for x in ground]
        acts = [x["surface_actual_deg"][s_name] for x in ground]
        surfaces[s_name] = {
            "cmd_max_abs_deg": max((abs(v) for v in cmds), default=None),
            "cmd_mean_deg": mean(cmds),
            "actual_max_abs_deg": max((abs(v) for v in acts), default=None),
            "actual_mean_deg": mean(acts),
            "actual_std_deg": std(acts),
            "max_abs_cmd_minus_actual_deg": max(
                (abs(c - a) for c, a in zip(cmds, acts)), default=None),
        }

    # ---- plugin diagnostic clamps ----
    clamps = {"interpClamped": 0, "rpmCapActive": 0, "currentLimited": 0,
              "negativeCurrentClamped": 0, "target_clamp_active": 0,
              "effort_clamp_active": 0}
    for s in S:
        for side in ("prop_left", "prop_right"):
            d = s.get(side) or {}
            for k in ("interpClamped", "rpmCapActive", "currentLimited",
                      "negativeCurrentClamped"):
                if d.get(k):
                    clamps[k] += 1
        sf = s.get("surface_full") or {}
        for name in L.SURFACES:
            d = sf.get(name) or {}
            for k in ("target_clamp_active", "effort_clamp_active"):
                if d.get(k):
                    clamps[k] += 1
    clamps["total_samples"] = len(S)

    # ---- inferred support force decay ----
    nvals = [(s["t_wall"] - t0, s["normal_force_N_INFERRED"]) for s in ground
             if s.get("normal_force_N_INFERRED") is not None]
    support = {
        "source": "INFERRED",
        "data_required": "DIRECT normal-force measurement. Neither the runway "
                         "world nor model/model.sdf carries a contact sensor. "
                         "The Part 4 result file labels this 'MEASURED' - "
                         "that label is wrong (see D3); the numbers were "
                         "always the inferred ones.",
        "contact_messages_ever_received": 0,
        "n_points": len(nvals),
        "value_at_throttle_onset_N": nvals[0][1] if nvals else None,
        "weight_reference_N": WEIGHT_N,
        "first_time_below_half_weight_s": next(
            (t for t, v in nvals if v <= 0.5 * WEIGHT_N), None),
        "first_time_at_or_below_zero_s": next((t for t, v in nvals if v <= 0.0), None),
    }
    if t_lift is not None:
        for k in ("first_time_below_half_weight_s", "first_time_at_or_below_zero_s"):
            v = support[k]
            support[k + "_minus_liftoff_s"] = (v - t_lift) if v is not None else None

    # ---- lift vs weight ----
    lw = criteria["aero_lift_equals_weight"]
    lift_vs_weight = {
        "weight_N": WEIGHT_N,
        "crossing_t_rel_s": lw["t_rel_s"] if lw else None,
        "airspeed_at_crossing_ms": lw["airspeed_ms"] if lw else None,
        "groundspeed_at_crossing_ms": lw["groundspeed_ms"] if lw else None,
        "liftoff_minus_lift_crossing_s": (
            (t_lift - lw["t_rel_s"]) if (t_lift is not None and lw) else None),
        "max_lift_N_whole_record": max((s["lift_N"] for s in S
                                        if s.get("lift_N") is not None), default=None),
    }

    # ---- airspeed / groundspeed at liftoff ----
    prim = criteria["primary_z_and_vz_DEALIASED"]
    liftoff_speeds = {
        "t_rel_s": t_lift,
        "airspeed_ms": prim["airspeed_ms"] if prim else None,
        "groundspeed_ms": prim["groundspeed_ms"] if prim else None,
        "note": "VFR_HUD airspeed vs groundspeed, both from ArduPlane, at the "
                "de-aliased primary liftoff sample.",
    }

    # ---- downsampled ground-roll timeseries for the next task ----
    step = max(1, len(ground) // 200)
    trace = []
    for s in ground[::step]:
        att = s.get("attitude_mav") or {}
        vfr = s.get("vfr") or {}
        trace.append({
            "t_rel_s": s["t_wall"] - t0,
            "gs": vfr.get("groundspeed"), "as": vfr.get("airspeed"),
            "y_m": s["lateral_displacement_m_world_Y"],
            "z_m": s["gz_pos_world_m"][2],
            "roll_deg": s["gz_rpy_deg"][0], "pitch_deg": s["gz_rpy_deg"][1],
            "yaw_deg": s["gz_rpy_deg"][2],
            "yawrate_frd": att.get("yawspeed_deg_s"),
            "rpm_l": s.get("rpm_left"), "rpm_r": s.get("rpm_right"),
            "T_l": s.get("thrust_left_N"), "T_r": s.get("thrust_right_N"),
            "Q_l": s.get("Q_prop_left_Nm"), "Q_r": s.get("Q_prop_right_Nm"),
            "lift_N": s.get("lift_N"),
            "N_inferred": s.get("normal_force_N_INFERRED"),
            "thr_pct": vfr.get("throttle_pct"),
        })

    # ======================================================================
    # ADDITIONAL MEASUREMENTS REQUIRED FOR THE TAKEOFF-MODE SCOPE EXTENSION
    # (gazebo-testing, 2026-09-10). MEASUREMENT ONLY - every one of these is
    # a read of already-captured raw signals. Nothing is tuned or fitted.
    # ======================================================================
    # ---- RCIN (pilot stick) vs RCOU (autopilot servo output), ch1 and ch4 --
    def _rc(sq, key):
        return [ (x.get("rc_in_pwm_named") or {}).get(key) for x in sq ]

    def _srv(sq, key):
        return [ (x.get("servo_pwm_named") or {}).get(key) for x in sq ]
    rcin_c1 = [v for v in _rc(ground, "c1_roll") if v is not None]
    rcin_c4 = [v for v in _rc(ground, "c4_yaw") if v is not None]
    rcin_c3 = [v for v in _rc(ground, "c3_throttle") if v is not None]
    rcin_c2 = [v for v in _rc(ground, "c2_pitch") if v is not None]
    rcou_c1 = [v for v in _srv(ground, "aileron") if v is not None]
    rcou_c2 = [v for v in _srv(ground, "elevator") if v is not None]
    rcou_c4 = [v for v in _srv(ground, "rudder") if v is not None]
    rcou_c3 = [v for v in _srv(ground, "throttle_left") if v is not None]
    rcou_c5 = [v for v in _srv(ground, "throttle_right") if v is not None]

    def _pwm_stats(xs, trim=1500):
        if not xs:
            return {"available": False,
                    "data_required": "RC_CHANNELS / SERVO_OUTPUT_RAW not "
                                     "present in this timeseries"}
        return {"available": True, "n": len(xs), "min": min(xs), "max": max(xs),
                "first": xs[0], "last": xs[-1], "mean": mean(xs),
                "max_abs_departure_from_%d_us" % trim: max(abs(x - trim) for x in xs),
                "constant": (min(xs) == max(xs))}
    rc_evidence = {
        "window": "ground roll (throttle onset .. liftoff)",
        "RCIN_is_pilot_stick_RCOU_is_autopilot_output": True,
        "RCIN_c1_roll": _pwm_stats(rcin_c1),
        "RCIN_c2_pitch": _pwm_stats(rcin_c2),
        "RCIN_c3_throttle": _pwm_stats(rcin_c3, 1000),
        "RCIN_c4_yaw": _pwm_stats(rcin_c4),
        "RCOU_c1_aileron": _pwm_stats(rcou_c1),
        "RCOU_c2_elevator": _pwm_stats(rcou_c2),
        "RCOU_c3_throttle_left": _pwm_stats(rcou_c3, 1000),
        "RCOU_c4_rudder": _pwm_stats(rcou_c4),
        "RCOU_c5_throttle_right": _pwm_stats(rcou_c5, 1000),
    }
    rc_evidence["stick_input_present_on_c1_or_c4"] = bool(
        (rc_evidence["RCIN_c1_roll"].get("max_abs_departure_from_1500_us") or 0) > 0
        or (rc_evidence["RCIN_c4_yaw"].get("max_abs_departure_from_1500_us") or 0) > 0)
    rc_evidence["interpretation_rule"] = (
        "If RCIN c1/c4 stay at 1500 us while RCOU c1/c4 depart from 1500 us, "
        "the surface commands are AUTOPILOT GENERATED, not pilot stick input. "
        "This is a statement about the source of the command only.")

    # ---- speed profile over the ground roll ------------------------------
    gs = [(s["t_wall"] - t0, (s.get("vfr") or {}).get("groundspeed"))
          for s in ground if (s.get("vfr") or {}).get("groundspeed") is not None]
    asp = [(s["t_wall"] - t0, (s.get("vfr") or {}).get("airspeed"))
           for s in ground if (s.get("vfr") or {}).get("airspeed") is not None]
    peak_gs = max(gs, key=lambda x: x[1]) if gs else None
    speed_profile = {
        "peak_groundspeed_ms": peak_gs[1] if peak_gs else None,
        "peak_groundspeed_t_rel_s": peak_gs[0] if peak_gs else None,
        "groundspeed_at_end_of_ground_roll_ms": gs[-1][1] if gs else None,
        "decel_after_peak_ms": ((peak_gs[1] - gs[-1][1]) if (gs and peak_gs) else None),
        "decelerated_after_peak": bool(gs and peak_gs and gs[-1][1] < peak_gs[1]),
        "peak_airspeed_ms": max((v for _, v in asp), default=None),
        "note": "VFR_HUD groundspeed/airspeed, ArduPlane's own estimate.",
    }

    # ---- rudder deflection sign vs yaw excursion sign --------------------
    # FLU: a POSITIVE Gazebo world-yaw excursion is NOSE LEFT. The rudder
    # ACTUAL joint angle sign convention is model.sdf's own; it is reported
    # raw and correlated, never reinterpreted here.
    rud_vs_yaw = []
    for s in ground:
        rud_vs_yaw.append((s["gz_rpy_deg"][2], s["surface_actual_deg"]["rudder"],
                           s["surface_cmd_deg"]["rudder"],
                           (s.get("attitude_mav") or {}).get("yawspeed_deg_s")))
    # SIGN CHAIN, stated so the comparison is physical and not numeric:
    #   POSITIVE rudder joint angle = TE toward -Y = RIGHT  -> NOSE RIGHT
    #     (manual_takeoff_live_lib.PRE_REGISTERED_EXPECTATIONS "yaw_right":
    #      "positive rudder joint = TE toward -Y = RIGHT", and the live
    #      scenario-E measurement confirms it: rudder joint delta +20.83 deg
    #      produced a GZ FLU body yaw rate of -70.69 deg/s)
    #   NOSE RIGHT is a NEGATIVE rotation about FLU +Z.
    # Therefore the yaw direction the rudder is COMMANDING, expressed in the
    # SAME FLU convention as the measured yaw, is -sign(rudder joint angle).
    # Comparing the raw numeric signs of yaw and rudder angle would invert
    # the physical answer, so it is not done.
    sig = [(y, -r) for y, r, c, w in rud_vs_yaw if abs(y) >= 0.1]
    same_dir = sum(1 for y, rc in sig if y * rc > 0)
    opp_dir = sum(1 for y, rc in sig if y * rc < 0)
    rudder_vs_yaw = {
        "sign_chain": "POSITIVE rudder joint = TE toward -Y = RIGHT = NOSE "
                      "RIGHT = NEGATIVE rotation about FLU +Z. So the FLU yaw "
                      "direction COMMANDED by the rudder is -sign(rudder "
                      "joint angle). Verified live in Part 2 scenario E "
                      "(rudder +20.83 deg -> GZ FLU yaw rate -70.69 deg/s).",
        "yaw_convention": "gz world yaw, FLU, POSITIVE = NOSE LEFT",
        "n_samples_with_abs_yaw_ge_0p1deg": len(sig),
        "n_rudder_commanding_the_SAME_direction_as_the_yaw_excursion": same_dir,
        "n_rudder_commanding_the_OPPOSITE_direction_to_the_yaw_excursion": opp_dir,
        "max_abs_rudder_actual_deg_during_roll": max(
            (abs(r) for _, r, _, _ in rud_vs_yaw), default=None),
        "max_abs_rudder_cmd_deg_during_roll": max(
            (abs(c) for _, _, c, _ in rud_vs_yaw), default=None),
        "note": "COUNTS ONLY. Whether a same-direction rudder is 'creating' "
                "the excursion or an opposite-direction rudder is "
                "'correcting' it is an attribution this module does not make. "
                "The onset ORDERING in lateral_drift is the relevant "
                "additional evidence.",
    }

    # ---- pre-throttle yaw alignment --------------------------------------
    pre = S[:thr_i] if thr_i else settle
    pre_yaw = [s["gz_rpy_deg"][2] for s in pre] or [None]
    initial_state = {
        "spawn_pose_from_world_sdf": doc.get("world_constants", {}).get("spawn_pose"),
        "gz_yaw_deg_first_sample": S[0]["gz_rpy_deg"][2],
        "gz_yaw_deg_max_abs_before_throttle_onset": max(
            (abs(v) for v in pre_yaw if v is not None), default=None),
        "gz_roll_deg_max_abs_before_throttle_onset": max(
            (abs(s["gz_rpy_deg"][0]) for s in pre), default=None),
        "gz_lateral_Y_m_max_abs_before_throttle_onset": max(
            (abs(s["lateral_displacement_m_world_Y"]) for s in pre), default=None),
    }

    # ---- AIRBORNE actuator limit cycle (reported, never fixed here) -------
    air = S[gr_end:] if lo_i is not None else []
    lc = {}
    for s_name in L.SURFACES:
        rates = []
        acts = []
        pinned = 0
        # The actuator diagnostics message carries actual_rate_rad_s but NOT
        # the limit, so the limit is READ from its own source of truth:
        # docs/source_of_truth/controls/actuator_v1_config.yaml
        # servo.max_rate_rad_s = 5.2360 (V1_PROVISIONAL, 300 deg/s). Read
        # only; nothing is written back.
        maxrate = MAX_RATE_RAD_S
        for x in air:
            d = (x.get("surface_full") or {}).get(s_name) or {}
            ar = d.get("actual_rate_rad_s")
            if ar is not None:
                rates.append(ar)
                if abs(abs(ar) - maxrate) <= 1e-4 * maxrate:
                    pinned += 1
            acts.append(x["surface_actual_deg"][s_name])
        lc[s_name] = {
            "n_airborne_samples": len(air),
            "max_rate_rad_s_reported_by_plugin": maxrate,
            "n_samples_actual_rate_pinned_at_max": pinned,
            "pct_samples_pinned": (100.0 * pinned / len(rates)) if rates else None,
            "actual_deg_min": min(acts) if acts else None,
            "actual_deg_max": max(acts) if acts else None,
            "cmd_deg_max_abs": max((abs(x["surface_cmd_deg"][s_name]) for x in air),
                                   default=None),
        }
    airborne_limit_cycle = {
        "window": "after the de-aliased liftoff sample to end of record",
        "per_surface": lc,
        "owner_if_real": "controls-integration",
        "not_fixed_here": "gazebo-testing does not change actuator rate, "
                          "effort or PID parameters.",
    }

    return {
        "run_index": idx,
        "mode": mode,
        "source_timeseries": path,
        "rc_input_vs_output": rc_evidence,
        "speed_profile": speed_profile,
        "rudder_sign_vs_yaw_excursion": rudder_vs_yaw,
        "initial_state_before_throttle": initial_state,
        "airborne_actuator_limit_cycle": airborne_limit_cycle,
        "n_samples": len(S),
        "capture_rate_hz": (len(S) - 1) / (S[-1]["t_wall"] - t0),
        "max_inter_source_skew_s": max((s.get("max_abs_skew_s") or 0.0) for s in S),
        "resting_z_m": resting_z,
        "throttle_onset": event(S, thr_i, t0, "VFR_HUD.throttle > 0", vz),
        "liftoff_criteria": criteria,
        "liftoff_speeds": liftoff_speeds,
        "pitch_while_in_contact": pitch_stats,
        "lateral_drift": drift,
        "propulsion_asymmetry": propulsion,
        "surfaces_during_ground_roll": surfaces,
        "plugin_diagnostic_clamps": clamps,
        "support_force": support,
        "lift_vs_weight": lift_vs_weight,
        "aero_deflection_cross_check": aero_cross_check(S[:gr_end], None),
        "ground_roll_trace_downsampled": trace,
    }


def spread_stats(xs):
    xs = [x for x in xs if isinstance(x, (int, float)) and not math.isnan(x)]
    if not xs:
        return None
    return {"n": len(xs), "min": min(xs), "max": max(xs), "mean": mean(xs),
            "max_abs": max(abs(x) for x in xs), "std": std(xs)}


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default=None,
                    help="Only analyse runs of this mode (MANUAL|FBWA|"
                         "TAKEOFF). Default: every timeseries present.")
    ap.add_argument("--out", default=None,
                    help="Output filename (inside tests/gazebo/results). "
                         "Default: manual_takeoff_ground_roll_"
                         "independent_analysis.json. Given explicitly when "
                         "analysing a subset so a previous campaign's "
                         "artifact is never overwritten.")
    args = ap.parse_args()
    pat = PREFIX + ("_run*_%s_timeseries.json" % args.mode if args.mode
                    else "_run*_timeseries.json")
    paths = sorted(glob.glob(os.path.join(RES, pat)))
    if not paths:
        print("no Part 4 timeseries found in", RES, "matching", pat)
        return 1
    runs = []
    for p in paths:
        print("analysing", os.path.basename(p), flush=True)
        runs.append(analyse_run(p))

    def col(fn):
        return [fn(r) for r in runs]
    campaign = {
        "n_runs": len(runs),
        "liftoff_t_rel_s": spread(col(lambda r: r["liftoff_speeds"]["t_rel_s"])),
        "liftoff_airspeed_ms": spread(col(lambda r: r["liftoff_speeds"]["airspeed_ms"])),
        "liftoff_groundspeed_ms": spread(col(lambda r: r["liftoff_speeds"]["groundspeed_ms"])),
        "y_at_liftoff_m": spread(col(lambda r: r["lateral_drift"]["y_at_liftoff_m"])),
        "max_abs_y_ground_roll_m": spread(
            col(lambda r: r["lateral_drift"]["max_abs_y_during_ground_roll_m"])),
        "lift_weight_crossing_s": spread(
            col(lambda r: r["lift_vs_weight"]["crossing_t_rel_s"])),
        "lift_weight_crossing_airspeed_ms": spread(
            col(lambda r: r["lift_vs_weight"]["airspeed_at_crossing_ms"])),
        "pitch_std_in_contact_deg": spread(
            col(lambda r: r["pitch_while_in_contact"]["std_deg"])),
        "pitch_max_change_in_contact_deg": spread(
            col(lambda r: r["pitch_while_in_contact"]["max_abs_change_from_resting_deg"])),
        "thrust_diff_max_abs_N": spread(
            col(lambda r: (r["propulsion_asymmetry"]["thrust_diff_left_minus_right_N"] or {}).get("max_abs"))),
        "net_reaction_torque_body_X_max_abs_Nm": spread(
            col(lambda r: (r["propulsion_asymmetry"]["net_reaction_torque_about_body_X_Nm"] or {}).get("max_abs"))),
        "net_thrust_yaw_moment_body_Z_max_abs_Nm": spread(
            col(lambda r: (r["propulsion_asymmetry"]["net_thrust_yaw_moment_about_body_Z_Nm"] or {}).get("max_abs"))),
    }
    out = {
        "stage": "MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION",
        "part": "4 - independent offline re-analysis",
        "mode_filter": args.mode,
        "source_glob": pat,
        "owner": "gazebo-testing",
        "physics_parameters_changed": "NONE",
        "frames": {
            "lateral": "Gazebo world Y, centreline Y=0, POSITIVE Y = LEFT",
            "body": "FLU",
            "mavlink_rates": "FRD (nose-right yaw is POSITIVE yawspeed)",
        },
        "campaign": campaign,
        "runs": runs,
    }
    dst = os.path.join(RES, args.out or (PREFIX + "_independent_analysis.json"))
    with open(dst, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    print("wrote", dst)
    print(json.dumps(campaign, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

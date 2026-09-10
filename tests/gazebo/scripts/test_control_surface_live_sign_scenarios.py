#!/usr/bin/env python3
"""FALCON V2 - Part 2: six SHORT airborne closed-loop control-surface SIGN
scenarios.

CLASSIFICATION: TEMPORARY_TEST_SETUP / MEASUREMENT-ONLY.
Stage: MANUAL_TAKEOFF_AND_LIVE_CONTROL_SURFACE_BEHAVIOR_VALIDATION (Part 2).
Owner: controls-integration. Created 2026-09-10.

CHANGES NO PHYSICS PARAMETER. No aerodynamic or propulsion coefficient, no
mass/CG/inertia, no actuator PID/rate/effort limit, no control-surface +/-45
deg mapping, no PTCH_TRIM_DEG, no TECS_PTCH_DAMP, no roll/pitch PID, no sensor
model, no landing gear, no friction or collision value. It injects RC
overrides and records.

=============================================================================
SCENARIOS
=============================================================================
  A) roll right      B) roll left
  C) pitch up        D) pitch down
  E) yaw right       F) yaw left
Each: settle window -> input window -> settle window. Short by design
(default 6 s / 8 s / 6 s = 20 s per scenario) so an input is attributable to
one axis and does not turn into a long uncontrolled excursion.

MODE: FBWA (custom_mode 5) with RC_CHANNELS_OVERRIDE. This is the established
pattern in this repository (tests/gazebo/scripts/
test_ardupilot_fbwa_level_pitch_reference_correction.py and
test_ardupilot_navigation_validation.py's settle_fbwa segment). FBWA gives a
clean, repeatable, attributable stick-to-attitude-demand relationship while
ArduPlane's own inner loops remain in charge - which is what makes the
resulting surface deflection a genuine CLOSED-LOOP observation rather than a
direct joint write.

=============================================================================
PRE-REGISTERED EXPECTATIONS - STATED BEFORE THE RUN, NOT FITTED AFTER
=============================================================================
The full table lives in manual_takeoff_live_lib.PRE_REGISTERED_EXPECTATIONS
and is embedded verbatim in this test's result JSON. It is derived ONLY from
already-verified documented facts:
  * docs/source_of_truth/controls/CONTROLS.md sec 10
      (VERIFIED_BY_GAZEBO_GEOMETRY_SIGN_TEST, 2026-08-22)
  * docs/test_results/2026-08-22_control_surface_sign_mapping_test_report.md
      (+joint angle -> TE up on all four wing/tail surfaces; +rudder angle ->
       TE toward -Y = right; delta_a>0 -> Mx>0 -> roll right;
       delta_e<0 -> My<0 -> nose up; delta_r>0 -> Mz<0 -> nose right)
  * docs/source_of_truth/autopilot/SITL_TRANSPORT_AND_ACTUATOR_MAPPING.md
      sec 7.1/7.2/7.3 (SERVO1_REVERSED=1, SERVO2_REVERSED=0,
      SERVO4_REVERSED=1 -> which PWM direction each demand maps to)
Summary of what each scenario MUST show if the chain is correct:
  roll right : aileron PWM < 1500; left_aileron_joint NEGATIVE (TE down) and
               right_aileron_joint POSITIVE (TE up) - OPPOSITE signs;
               delta_a_aero > 0; Cl > 0; roll rate > 0 (right wing down).
  roll left  : mirror of the above.
  pitch up   : elevator PWM > 1500; BOTH elevator joints POSITIVE (TE up) -
               SAME sign; delta_e_aero < 0; Cm > 0; pitch rate > 0 (nose up).
  pitch down : mirror.
  yaw right  : rudder PWM < 1500; rudder_joint POSITIVE (TE right);
               delta_r_aero > 0; Cn < 0; MAVLink (FRD) yaw rate > 0.
  yaw left   : mirror.
NOTE ON THE YAW-RATE SIGN, because it is the one that is easy to get wrong:
"nose right" is a NEGATIVE rotation about the FLU body +Z (up) axis, hence
Mz < 0, but it is a POSITIVE yaw rate in MAVLink's FRD convention. Both are
recorded separately and each is checked against its own convention. They are
never compared to each other without the flip.

=============================================================================
AIRBORNE INITIAL CONDITION
=============================================================================
Reuses this repository's already-validated bring-up unchanged
(test_ardupilot_basic_closed_loop_flight): arm -> ground settle -> teleport to
altitude -> brief hold-to-trim wrench -> FULL wrench release. From release
onward there is ZERO force/torque/pose/velocity intervention; every measured
window is free 6-DOF flight under ArduPlane's own control. The bring-up window
is excluded from all analysis. This is the same non-bypass justification
already accepted in docs/validation/2026-09-08_ardupilot_navigation_and_
autotune_validation.md.

=============================================================================
USAGE
=============================================================================
    python3 test_control_surface_live_sign_scenarios.py --scenario A|B|C|D|E|F
    python3 test_control_surface_live_sign_scenarios.py --all
    python3 test_control_surface_live_sign_scenarios.py --dry-run
    python3 test_control_surface_live_sign_scenarios.py --summarize
A FRESH gz sim + arduplane pair must be running per scenario - see
tests/gazebo/scripts/run_manual_takeoff_and_live_surface_validation.sh.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import manual_takeoff_live_lib as L  # noqa: E402

PREFIX = "control_surface_live_sign"
MODE_FBWA = 5

# RC endpoints. PWM only - autopilot inputs, not physics parameters. 1100/1900
# are the live-calibrated full-scale RC endpoints of this SITL
# (docs/test_results/2026-08-28_ardupilot_control_surface_travel_scaling_
# validation.md sec: "endpoints landing exactly on RC=1100/1900 <-> SERVO=
# 800/2200"). A partial-scale value is used for the airborne scenarios so the
# aircraft is disturbed clearly but does not depart controlled flight.
RC_NEUTRAL = {"rc1": 1500, "rc2": 1500, "rc3": 1500, "rc4": 1500, "rc5": 1000}
RC_PARTIAL_HIGH = 1700   # ASSUMPTION (test input, not physics): +50% stick.
RC_PARTIAL_LOW = 1300    # ASSUMPTION: -50% stick.

SCENARIOS = {
    "A": {"name": "roll_right",  "expect_key": "roll_right",
          "rc": {"rc1": RC_PARTIAL_HIGH}, "axis": "roll"},
    "B": {"name": "roll_left",   "expect_key": "roll_left",
          "rc": {"rc1": RC_PARTIAL_LOW},  "axis": "roll"},
    "C": {"name": "pitch_up",    "expect_key": "pitch_up",
          "rc": {"rc2": RC_PARTIAL_HIGH}, "axis": "pitch"},
    "D": {"name": "pitch_down",  "expect_key": "pitch_down",
          "rc": {"rc2": RC_PARTIAL_LOW},  "axis": "pitch"},
    "E": {"name": "yaw_right",   "expect_key": "yaw_right",
          "rc": {"rc4": RC_PARTIAL_HIGH}, "axis": "yaw"},
    "F": {"name": "yaw_left",    "expect_key": "yaw_left",
          "rc": {"rc4": RC_PARTIAL_LOW},  "axis": "yaw"},
}

# NOTE ON SCENARIO C/D RC2 POLARITY: ArduPlane's RC2 pitch stick sense depends
# on RC2_REVERSED / the pilot-input convention, and this project has NOT
# separately verified which RC2 endpoint ArduPlane treats as "pitch up".
# THIS TEST DOES NOT ASSUME IT. It records the resulting nav_pitch_deg demand
# and classifies the scenario by the SIGN OF THE DEMAND ArduPlane actually
# produced, then checks the elevator/Cm/pitch-rate chain against THAT demand.
# The RC endpoint is only the stimulus; the pre-registered expectation is
# expressed in terms of the DEMAND, which is unambiguous.
RC2_POLARITY_STATUS = (
    "DATA_REQUIRED (non-blocking): the mapping RC2 endpoint -> ArduPlane "
    "pitch-up vs pitch-down demand is not separately documented in this "
    "repository. This test therefore classifies scenarios C/D by the measured "
    "sign of nav_pitch_deg (the demand ArduPlane actually formed), not by the "
    "RC endpoint. The elevator sign chain is validated against the demand, so "
    "the conclusion does not depend on resolving this.")

SETTLE_BEFORE_S = 6.0
INPUT_S = 8.0
SETTLE_AFTER_S = 6.0
SAMPLE_PERIOD_S = 0.02

TH = {
    "min_surface_deflection_deg": {
        "value": 0.5,
        "basis": "DERIVED. ~8 PWM quanta of this transport (1 PWM = 0.0643 "
                 "deg, derived in manual_takeoff_live_lib from model.sdf's "
                 "own multiplier/servo_min/servo_max). Below this the "
                 "deflection is not clearly distinguishable from the servo "
                 "model's documented neutral-hold droop (~0.06 deg, "
                 "actuator_v1_config.yaml integral_derivation_note), so a "
                 "sign claim would not be defensible.",
    },
    "min_rate_response_deg_s": {
        "value": 2.0,
        "basis": "ASSUMPTION (reporting only). The settle windows in the "
                 "existing navigation records show wings-level body-rate "
                 "noise well under 1 deg/s; 2 deg/s is a clear, "
                 "attributable response above that.",
    },
    "min_coefficient_magnitude": {
        "value": 1e-4,
        "basis": "ASSUMPTION (reporting only). Aero coefficients in the "
                 "existing navigation records sit at ~1e-5 during "
                 "wings-level cruise, so 1e-4 is an order of magnitude above "
                 "the quiescent level and its SIGN is meaningful.",
    },
    "opposite_sign_aileron_required": {
        "value": True,
        "basis": "CONTROLS.md sec 10 + the 2026-08-22 report's finding that "
                 "the two aileron joints are NOT sign-mirrored, so "
                 "differential roll requires opposite-sign joint commands. "
                 "This is a documented structural fact being re-checked live, "
                 "not a tuning target.",
    },
    "same_sign_elevator_required": {
        "value": True,
        "basis": "Same sources: the two elevator joints share the +Y-dominant "
                 "axis sense, so pure pitch requires SAME-sign commands.",
    },
}

_LOG = []


def log(*a):
    line = " ".join(str(x) for x in a)
    print(line, flush=True)
    _LOG.append(line)


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def sgn_str(x, floor=0.0):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "?"
    if abs(x) <= floor:
        return "0"
    return "+" if x > 0 else "-"


def capture_window(ctx, label, duration, rc):
    """Sample everything for `duration` seconds while holding `rc`."""
    out = []
    t_end = time.time() + duration
    last_pub = 0.0
    while time.time() < t_end:
        now = time.time()
        if now - last_pub > 0.05:
            ctx["mav"].send_rc_override(**rc)
            last_pub = now
        rec = ctx["sample"](label)
        if rec is not None:
            out.append(rec)
        time.sleep(SAMPLE_PERIOD_S)
    return out


def analyse_scenario(key, pre, before, during, after):
    """Compare the pre-registered expectation against what was measured.

    Everything below is a comparison of SIGNS and MAGNITUDES that were
    recorded; nothing is adjusted, fitted or tuned."""
    defl_floor = TH["min_surface_deflection_deg"]["value"]
    rate_floor = TH["min_rate_response_deg_s"]["value"]
    coef_floor = TH["min_coefficient_magnitude"]["value"]

    def base_of(recs, path, default=float("nan")):
        vals = []
        for r in recs:
            v = r
            for k in path:
                if v is None:
                    break
                v = v.get(k) if isinstance(v, dict) else None
            if isinstance(v, (int, float)):
                vals.append(v)
        return mean(vals) if vals else default

    # baseline from the settle window immediately before the input
    baseline = {
        "left_aileron_deg": math.degrees(base_of(before, ["actual_rad", "left_aileron"]) or 0.0),
        "right_aileron_deg": math.degrees(base_of(before, ["actual_rad", "right_aileron"]) or 0.0),
        "left_elevator_deg": math.degrees(base_of(before, ["actual_rad", "left_elevator"]) or 0.0),
        "right_elevator_deg": math.degrees(base_of(before, ["actual_rad", "right_elevator"]) or 0.0),
        "rudder_deg": math.degrees(base_of(before, ["actual_rad", "rudder"]) or 0.0),
        "delta_a_deg": math.degrees(base_of(before, ["aero_input_rad", "delta_a"]) or 0.0),
        "delta_e_deg": math.degrees(base_of(before, ["aero_input_rad", "delta_e"]) or 0.0),
        "delta_r_deg": math.degrees(base_of(before, ["aero_input_rad", "delta_r"]) or 0.0),
        "Cl": base_of(before, ["aero", "Cl"]),
        "Cm": base_of(before, ["aero", "Cm"]),
        "Cn": base_of(before, ["aero", "Cn"]),
        "roll_deg": base_of(before, ["attitude", "roll_deg"]),
        "pitch_deg": base_of(before, ["attitude", "pitch_deg"]),
        "yaw_deg": base_of(before, ["attitude", "yaw_deg"]),
        "servo_ail_pwm": base_of(before, ["servo_pwm_named", "aileron"]),
        "servo_ele_pwm": base_of(before, ["servo_pwm_named", "elevator"]),
        "servo_rud_pwm": base_of(before, ["servo_pwm_named", "rudder"]),
    }
    # steady portion of the input window: drop the first 25% as transient
    n = len(during)
    steady = during[max(1, n // 4):] if n > 4 else during
    measured = {
        "left_aileron_deg": math.degrees(base_of(steady, ["actual_rad", "left_aileron"]) or 0.0),
        "right_aileron_deg": math.degrees(base_of(steady, ["actual_rad", "right_aileron"]) or 0.0),
        "left_elevator_deg": math.degrees(base_of(steady, ["actual_rad", "left_elevator"]) or 0.0),
        "right_elevator_deg": math.degrees(base_of(steady, ["actual_rad", "right_elevator"]) or 0.0),
        "rudder_deg": math.degrees(base_of(steady, ["actual_rad", "rudder"]) or 0.0),
        "delta_a_deg": math.degrees(base_of(steady, ["aero_input_rad", "delta_a"]) or 0.0),
        "delta_e_deg": math.degrees(base_of(steady, ["aero_input_rad", "delta_e"]) or 0.0),
        "delta_r_deg": math.degrees(base_of(steady, ["aero_input_rad", "delta_r"]) or 0.0),
        "Cl": base_of(steady, ["aero", "Cl"]),
        "Cm": base_of(steady, ["aero", "Cm"]),
        "Cn": base_of(steady, ["aero", "Cn"]),
        "servo_ail_pwm": base_of(steady, ["servo_pwm_named", "aileron"]),
        "servo_ele_pwm": base_of(steady, ["servo_pwm_named", "elevator"]),
        "servo_rud_pwm": base_of(steady, ["servo_pwm_named", "rudder"]),
        "nav_roll_deg": base_of(steady, ["nav", "nav_roll_deg"]),
        "nav_pitch_deg": base_of(steady, ["nav", "nav_pitch_deg"]),
    }
    peak = {
        "rollspeed_deg_s": max((r["attitude"]["rollspeed_deg_s"] for r in during
                                if r.get("attitude")), key=abs, default=float("nan")),
        "pitchspeed_deg_s": max((r["attitude"]["pitchspeed_deg_s"] for r in during
                                 if r.get("attitude")), key=abs, default=float("nan")),
        "yawspeed_deg_s_MAVLINK_FRD": max(
            (r["attitude"]["yawspeed_deg_s"] for r in during
             if r.get("attitude")), key=abs, default=float("nan")),
        "yawrate_deg_s_GZ_FLU_BODY": max(
            (r["gz_body_rate_deg_s"][2] for r in during
             if r.get("gz_body_rate_deg_s")), key=abs, default=float("nan")),
        "d_roll_deg": (base_of(during[-max(1, len(during)//8):], ["attitude", "roll_deg"])
                       - baseline["roll_deg"]),
        "d_pitch_deg": (base_of(during[-max(1, len(during)//8):], ["attitude", "pitch_deg"])
                        - baseline["pitch_deg"]),
    }
    delta = {k: measured[k] - baseline.get(k, 0.0)
             for k in measured if k in baseline and
             isinstance(measured[k], float) and isinstance(baseline.get(k), float)}

    checks = []

    def check(name, got, expected_sign, floor, note=""):
        s = sgn_str(got, floor)
        ok = (s == expected_sign) if expected_sign in ("+", "-") else None
        checks.append({"check": name, "measured": got, "measured_sign": s,
                       "expected_sign": expected_sign, "floor": floor,
                       "pass": ok, "note": note})
        return ok

    ex = pre
    axis = SCENARIOS[key]["axis"]
    if axis == "roll":
        # [0] for the same reason as the Cm/Cn checks below: take the sign
        # character, never a sign-plus-prose string. These two entries happen
        # to be plain "+"/"-" today, so this is defensive and changes nothing
        # about how they are judged.
        exp_da = ex["expected_delta_a_aero_sign"][0]
        exp_cl = ex["expected_Cl_sign"][0]
        exp_p = ex["expected_roll_rate_sign"][0]
        exp_la = "-" if "negative" in ex["expected_left_aileron_joint"] else "+"
        exp_ra = "-" if "negative" in ex["expected_right_aileron_joint"] else "+"
        check("left_aileron_joint_delta_deg", delta.get("left_aileron_deg"),
              exp_la, defl_floor, "TE down = negative joint angle")
        check("right_aileron_joint_delta_deg", delta.get("right_aileron_deg"),
              exp_ra, defl_floor, "TE up = positive joint angle")
        checks.append({
            "check": "ailerons_deflect_in_OPPOSITE_directions",
            "measured": [delta.get("left_aileron_deg"), delta.get("right_aileron_deg")],
            "pass": bool(delta.get("left_aileron_deg", 0.0)
                         * delta.get("right_aileron_deg", 0.0) < 0.0),
            "requirement_source": "CONTROLS.md sec 10 / 2026-08-22 report "
                                  "(joints are NOT sign-mirrored, so "
                                  "differential roll needs opposite signs)",
        })
        check("delta_a_aero_deg", delta.get("delta_a_deg"), exp_da, defl_floor)
        check("Cl", delta.get("Cl"), exp_cl, coef_floor,
              "aero rolling-moment coefficient from the plugin's own diagnostics")
        check("peak_roll_rate_deg_s", peak["rollspeed_deg_s"], exp_p, rate_floor)
    elif axis == "pitch":
        # classify by the DEMAND ArduPlane actually formed (see
        # RC2_POLARITY_STATUS above), not by the RC endpoint
        demand = measured.get("nav_pitch_deg")
        demand_sign = sgn_str(demand, 0.5)
        exp = ex if demand_sign != "?" else ex
        check("delta_e_aero_deg", delta.get("delta_e_deg"),
              ex["expected_delta_e_aero_sign"], defl_floor,
              "delta_e < 0 == nose-up per CONTROLS.md sec 10")
        checks.append({
            "check": "elevators_deflect_in_the_SAME_direction",
            "measured": [delta.get("left_elevator_deg"), delta.get("right_elevator_deg")],
            "pass": bool(delta.get("left_elevator_deg", 0.0)
                         * delta.get("right_elevator_deg", 0.0) > 0.0),
            "requirement_source": "CONTROLS.md sec 10 / 2026-08-22 report "
                                  "(same-sign pair; common mode = pure pitch)",
        })
        # HARNESS FIX (gazebo-testing, 2026-09-10, evaluation defect only):
        # PRE_REGISTERED_EXPECTATIONS stores some expected signs as a sign
        # character FOLLOWED BY EXPLANATORY PROSE (e.g. "+ (Cm>0 <=> My<0 <=>
        # nose up in this plugin)"). check() only judges a value when the
        # expected sign is exactly "+" or "-", so passing the whole string
        # SILENTLY left this check UNJUDGED (pass=None) for pitch_up while the
        # plain-"-" pitch_down entry WAS judged - an asymmetry that hid a real
        # comparison. [0] takes the sign character, which is the idiom this
        # same function already uses for the rate signs
        # (ex["expected_pitch_rate_sign"][0]). This makes the test STRICTER,
        # judging a check that was previously skipped; it changes no
        # expectation, no threshold and no physics value.
        check("Cm", delta.get("Cm"), ex["expected_Cm_sign"][0], coef_floor,
              "Cm>0 <=> My<0 <=> nose up in this plugin's FLU convention")
        check("peak_pitch_rate_deg_s", peak["pitchspeed_deg_s"],
              ex["expected_pitch_rate_sign"][0], rate_floor)
        checks.append({"check": "arduplane_pitch_demand_sign_observed",
                       "measured": demand, "measured_sign": demand_sign,
                       "note": RC2_POLARITY_STATUS, "pass": None})
    else:  # yaw
        check("rudder_joint_delta_deg", delta.get("rudder_deg"),
              "+" if "positive" in ex["expected_rudder_joint"] else "-",
              defl_floor, "positive rudder joint = TE toward -Y = RIGHT")
        check("delta_r_aero_deg", delta.get("delta_r_deg"),
              ex["expected_delta_r_aero_sign"], defl_floor)
        # HARNESS FIX (gazebo-testing, 2026-09-10) - same defect as the Cm
        # check above: yaw_right's expected_Cn_sign is "- (Mz<0 = nose right)"
        # and was therefore never judged, while yaw_left's plain "+" WAS.
        # Taking the sign character judges both. STRICTER, not looser.
        check("Cn", delta.get("Cn"), ex["expected_Cn_sign"][0], coef_floor,
              "Cn<0 <=> Mz<0 <=> nose right (FLU)")
        check("peak_yaw_rate_MAVLINK_FRD_deg_s",
              peak["yawspeed_deg_s_MAVLINK_FRD"],
              "+" if "yaw_right" == ex.get("_key", SCENARIOS[key]["name"]) else "-",
              rate_floor,
              "MAVLink FRD: nose-right is POSITIVE yaw rate")
        check("peak_yaw_rate_GZ_FLU_BODY_deg_s",
              peak["yawrate_deg_s_GZ_FLU_BODY"],
              "-" if SCENARIOS[key]["name"] == "yaw_right" else "+",
              rate_floor,
              "FLU body frame: nose-right is a NEGATIVE rotation about +Z-up. "
              "This is the SAME physical motion as the MAVLink check above, "
              "expressed in the other convention - the opposite expected sign "
              "here is the frame conversion, not a contradiction.")
        # aileron cross-check: rudder input should not be commanding roll
        checks.append({
            "check": "aileron_differential_stayed_small_during_yaw_input",
            "measured_delta_a_deg": delta.get("delta_a_deg"),
            "note": "KFF_RDDRMIX is a roll->rudder feedforward, NOT "
                    "rudder->roll, so a rudder input should not directly "
                    "drive the ailerons; any aileron motion here is FBWA "
                    "holding wings level against the yaw-roll coupling, which "
                    "is expected and is reported rather than judged.",
            "pass": None,
        })

    hard = [c for c in checks if c.get("pass") is not None]
    return {
        "scenario": key,
        "name": SCENARIOS[key]["name"],
        "rc_injected": SCENARIOS[key]["rc"],
        "rc_full_override_sent": dict(RC_NEUTRAL, **SCENARIOS[key]["rc"]),
        "pre_registered_expectation": pre,
        "baseline_settle_before": baseline,
        "measured_steady_during_input": measured,
        "delta_vs_baseline": delta,
        "peak_response": peak,
        "checks": checks,
        "n_checks_pass": sum(1 for c in hard if c["pass"]),
        "n_checks_total": len(hard),
        "verdict": "PASS" if hard and all(c["pass"] for c in hard) else "SEE_CHECKS",
        "window_sample_counts": {"before": len(before), "during": len(during),
                                 "after": len(after)},
    }


def build_context(world, mav):
    """Create every subscriber and the per-sample capture closure around an
    ALREADY-OPEN MAVLink connection.

    The connection is passed in, never created here: ArduPlane SITL's SERIAL0
    TCP endpoint serves a single client, so opening a second connection while
    the bring-up helper still holds one would silently starve one of them."""
    import actuator_lib as ACT
    import aero_lib as AL
    import propulsion_lib as PL
    import select as _sel

    signs = L.read_sign_scalars()
    clock = L.ClockSub(world)
    posesub = L.LinkPoseSub(world, ["falcon_v2", "base_link"] + L.SURFACES)
    actsub = L.TimedDiagSub(ACT.DIAG_TOPIC, ACT.DiagSubscriber._split)
    aerosub = L.TimedDiagSub(AL.DIAG_TOPIC,
                             lambda v: dict(zip(AL.DiagSubscriber.FIELDS, v)))
    propsub = L.TimedDiagSub(PL.DIAG_TOPIC, PL.DiagSubscriber._split)

    state = {"servo": None, "servo_wall": None, "att": None, "att_wall": None,
             "nav": None, "nav_wall": None, "vfr": None, "gpi": None,
             "hb": None}

    def pump():
        for _ in range(60):
            r, _, _ = _sel.select([mav.m.port], [], [], 0.0)
            if not r:
                return
            m = mav.m.recv_match(blocking=False)
            if m is None:
                return
            t = m.get_type()
            if t == "SERVO_OUTPUT_RAW":
                state["servo"] = [m.servo1_raw, m.servo2_raw, m.servo3_raw,
                                  m.servo4_raw, m.servo5_raw]
                state["servo_wall"] = time.time()
            elif t == "ATTITUDE":
                state["att"] = {
                    "roll_deg": math.degrees(m.roll),
                    "pitch_deg": math.degrees(m.pitch),
                    "yaw_deg": math.degrees(m.yaw),
                    "rollspeed_deg_s": math.degrees(m.rollspeed),
                    "pitchspeed_deg_s": math.degrees(m.pitchspeed),
                    "yawspeed_deg_s": math.degrees(m.yawspeed),
                }
                state["att_wall"] = time.time()
            elif t == "NAV_CONTROLLER_OUTPUT":
                state["nav"] = {"nav_roll_deg": m.nav_roll,
                                "nav_pitch_deg": m.nav_pitch,
                                "nav_bearing_deg": m.nav_bearing,
                                "target_bearing_deg": m.target_bearing,
                                "xtrack_error_m": m.xtrack_error}
                state["nav_wall"] = time.time()
            elif t == "VFR_HUD":
                state["vfr"] = {"airspeed": m.airspeed, "groundspeed": m.groundspeed,
                                "alt": m.alt, "climb": m.climb,
                                "throttle_pct": m.throttle, "heading_deg": m.heading}
            elif t == "HEARTBEAT":
                state["hb"] = {"custom_mode": m.custom_mode,
                               "armed": bool(m.base_mode & 128)}

    prev = {"pos": None, "quat": None, "t": None}

    def sample(label):
        pump()
        act, act_wall = actsub.latest()
        aero, aero_wall = aerosub.latest()
        prop, prop_wall = propsub.latest()
        poses, pose_stamp, pose_wall = posesub.latest()
        sim_t, _ = clock.latest()
        if act is None:
            return None
        theta = {s: act[s]["actual_angle_rad"] for s in L.SURFACES}
        cmd = {s: act[s]["cmd_rad"] for s in L.SURFACES}
        deltas = L.aero_deltas_from_joint_angles(theta, signs)
        # gz body angular rate, finite-differenced from the RENDER pose stream.
        # Labelled explicitly as a finite difference so it is never confused
        # with a physics-engine twist reading.
        gz_rate = None
        if poses and "falcon_v2" in poses:
            q = poses["falcon_v2"]["quat"]
            p = poses["falcon_v2"]["pos"]
            now = time.time()
            if prev["quat"] is not None and prev["t"] is not None and now > prev["t"]:
                dq = L.q_mul(L.q_conj(prev["quat"]), q)
                dt = now - prev["t"]
                gz_rate = [math.degrees(2.0 * dq[i + 1] / dt) for i in range(3)]
            prev["quat"], prev["pos"], prev["t"] = q, p, now
        bound = L.bind_sample({
            "actuator_diag": (None, act_wall),
            "aero_diag": (None, aero_wall),
            "propulsion_diag": (None, prop_wall),
            "link_pose": (None, pose_wall),
            "servo_output_raw": (None, state["servo_wall"]),
            "attitude": (None, state["att_wall"]),
            "nav_controller_output": (None, state["nav_wall"]),
        })
        srv = state["servo"] or [None] * 5
        return {
            "window": label,
            "t_wall": time.time(),
            "t_sim": sim_t,
            "skew_s": bound["skew_s"],
            "max_abs_skew_s": bound["max_abs_skew_s"],
            "cmd_rad": cmd,
            "actual_rad": theta,
            "actuator_full": act,
            "aero_input_rad": deltas,
            "aero": aero,
            "propulsion": prop,
            "servo_pwm": srv,
            "servo_pwm_named": {"aileron": srv[0], "elevator": srv[1],
                                "throttle_left": srv[2], "rudder": srv[3],
                                "throttle_right": srv[4]},
            "attitude": state["att"],
            "nav": state["nav"],
            "vfr": state["vfr"],
            "heartbeat": state["hb"],
            "gz_body_rate_deg_s": gz_rate,
            "model_pose": poses.get("falcon_v2") if poses else None,
        }

    return {"mav": mav, "sample": sample, "signs": signs, "clock": clock,
            "posesub": posesub}


def run_scenario(key, world, args):
    """Airborne bring-up (reused verbatim) then one short scenario.

    The bring-up is NOT reimplemented here. It calls
    test_ardupilot_navigation_validation.bring_up(), which is the same
    already-validated sequence every prior ArduPlane stage in this repository
    used: arm -> ground settle -> teleport to altitude -> brief hold-to-trim
    wrench -> FULL wrench release (clear_wrench) -> FBWA handoff. From the
    release instant on there is ZERO force/torque/pose/velocity intervention.
    """
    import test_ardupilot_basic_closed_loop_flight as base
    # World-name override: base.WORLD is a module global used by set_pose()
    # and PoseSub(). It is set here so the bring-up helper acts on whichever
    # world this scenario is actually running in. This mutates only an
    # in-memory test-module variable - no file, no SDF, no parameter.
    base.WORLD = world
    import test_ardupilot_navigation_validation as navval

    cfg = SCENARIOS[key]
    log(f"\n=== SCENARIO {key}: {cfg['name']} (world={world}) ===")

    R = {}
    log("bring-up: arm -> settle -> teleport -> hold-to-trim -> FULL release "
        "-> FBWA")
    bctx, params, fail = navval.bring_up(R)
    if fail:
        log("BRING-UP FAILED at:", fail)
        return {"scenario": key, "name": cfg["name"],
                "verdict": "ABORT_BRINGUP_FAILED", "bringup_failed_at": fail,
                "bringup_record": R}
    mav = bctx["mav"]
    log("bring-up OK; free flight from:", R.get("free_flight_from"))

    # Raise and MEASURE the telemetry stream rates. The bring-up helper uses
    # MAV_CMD_SET_MESSAGE_INTERVAL, which this stage measured to be DENIED on
    # this ArduPlane build (see manual_takeoff_live_lib.request_stream_rates
    # for the evidence), so the rates are re-requested here via the mechanism
    # that works and the achieved rates are recorded. No parameter is written.
    stream_rates = L.request_stream_rates(
        mav, requests=L.DEFAULT_STREAM_REQUESTS + (("EXTENDED_STATUS", 10, []),))
    log("stream rates measured:",
        {k: v for k, v in stream_rates["measured_hz"].items()
         if k in ("ATTITUDE", "SERVO_OUTPUT_RAW", "VFR_HUD",
                  "NAV_CONTROLLER_OUTPUT", "HEARTBEAT")})

    ctx = build_context(world, mav)
    ctx["bringup_record"] = R

    before = capture_window(ctx, "settle_before", SETTLE_BEFORE_S, dict(RC_NEUTRAL))
    rc_in = dict(RC_NEUTRAL, **cfg["rc"])
    during = capture_window(ctx, "input", INPUT_S, rc_in)
    after = capture_window(ctx, "settle_after", SETTLE_AFTER_S, dict(RC_NEUTRAL))
    mav.hold_rc_override(1.0, **RC_NEUTRAL)
    try:
        base.disarm(mav)
    except Exception as e:  # disarm failure must not lose the captured data
        log("disarm raised (data already captured):", e)
    mav.close()

    pre = dict(L.PRE_REGISTERED_EXPECTATIONS[cfg["expect_key"]])
    result = analyse_scenario(key, pre, before, during, after)
    result["bringup_record"] = R
    result["mavlink_stream_rates_measured"] = stream_rates
    result["free_flight_from"] = R.get("free_flight_from")
    ts_path = os.path.join(L.RESULTS_DIR, f"{PREFIX}_scenario_{key}_timeseries.json")
    with open(ts_path, "w", encoding="utf-8") as fh:
        json.dump({"scenario": key, "name": cfg["name"],
                   "windows": {"before": before, "during": during, "after": after}}, fh)
    log("wrote", ts_path)
    return result


def summarize():
    out = {"scenarios": {}}
    for k in SCENARIOS:
        p = os.path.join(L.RESULTS_DIR, f"{PREFIX}_scenario_{k}_result.json")
        if os.path.exists(p):
            with open(p, "r", encoding="utf-8") as fh:
                out["scenarios"][k] = json.load(fh).get("result")
    dst = os.path.join(L.RESULTS_DIR, f"{PREFIX}_summary.json")
    with open(dst, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    print("wrote", dst)
    for k, v in out["scenarios"].items():
        if v:
            # HARNESS FIX (gazebo-testing, 2026-09-10, execution defect only):
            # an ABORT_BRINGUP_FAILED result carries no n_checks_* keys, and
            # the unguarded lookup raised KeyError and killed the whole
            # summary AFTER every scenario had already been run and written.
            # .get() with an explicit "n/a" keeps the aborted scenario VISIBLE
            # in the summary instead of losing the whole table. No threshold,
            # no criterion and no physics value is touched.
            npass = v.get("n_checks_pass")
            ntot = v.get("n_checks_total")
            counts = "n/a" if npass is None or ntot is None else f"{npass}/{ntot}"
            print(f"  {k} {v['name']:12s} {v['verdict']:24s} {counts}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=sorted(SCENARIOS))
    ap.add_argument("--all", action="store_true")
    # DEFAULT WORLD: the established airborne world every prior ArduPlane
    # stage in this repository used. These six scenarios are AIRBORNE, so the
    # runway world is not needed and is deliberately not the default - Part 4
    # (ground roll) is the test that uses the runway world. Overridable.
    ap.add_argument("--world", default="falcon_v2_ardupilot_basic_closed_loop_flight")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--summarize", action="store_true")
    args = ap.parse_args()

    L.env_setup()
    if args.summarize:
        return summarize()

    header = L.provenance_header("2 - live control-surface sign scenarios",
                                 args.world,
                                 {"thresholds": TH,
                                  "rc2_polarity_status": RC2_POLARITY_STATUS,
                                  "scenarios": SCENARIOS,
                                  "mode": "FBWA (custom_mode 5) + "
                                          "RC_CHANNELS_OVERRIDE",
                                  "window_seconds": {
                                      "settle_before": SETTLE_BEFORE_S,
                                      "input": INPUT_S,
                                      "settle_after": SETTLE_AFTER_S}})
    if args.dry_run:
        # Exercise every offline path: source-of-truth reads, expectation
        # table completeness, and the analyser against a synthetic record set
        # so a crash in analyse_scenario() is caught without a simulator.
        problems = []
        for k, cfg in SCENARIOS.items():
            if cfg["expect_key"] not in L.PRE_REGISTERED_EXPECTATIONS:
                problems.append(f"scenario {k}: missing expectation")
        blocks, sdf_problems = L.read_control_blocks()
        problems += sdf_problems
        synth = _synthetic_windows()
        for k in SCENARIOS:
            r = analyse_scenario(k, dict(L.PRE_REGISTERED_EXPECTATIONS[
                SCENARIOS[k]["expect_key"]]), *synth)
            if "checks" not in r:
                problems.append(f"scenario {k}: analyser produced no checks")
        header["dry_run"] = {"problems": problems, "pass": not problems,
                             "synthetic_analyser_exercised": True}
        header["overall"] = "DRY_RUN_OK" if not problems else "DRY_RUN_PROBLEMS"
        dst = os.path.join(L.RESULTS_DIR, f"{PREFIX}_dry_run.json")
        with open(dst, "w", encoding="utf-8") as fh:
            json.dump(header, fh, indent=1)
        print("wrote", dst)
        print("OVERALL:", header["overall"], problems)
        return 0 if not problems else 1

    keys = sorted(SCENARIOS) if args.all else [args.scenario]
    if keys == [None]:
        ap.error("give --scenario X, --all, --dry-run or --summarize")
    rc = 0
    for k in keys:
        res = run_scenario(k, args.world, args)
        doc = dict(header)
        doc["result"] = res
        dst = os.path.join(L.RESULTS_DIR, f"{PREFIX}_scenario_{k}_result.json")
        with open(dst, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, indent=1)
        log("wrote", dst, "verdict", res["verdict"])
        if res["verdict"] != "PASS":
            rc = 1
    with open(os.path.join(L.RESULTS_DIR, f"{PREFIX}_log.txt"), "w",
              encoding="utf-8") as fh:
        fh.write("\n".join(_LOG) + "\n")
    return rc


def _synthetic_windows():
    """Deterministic fake records used ONLY by --dry-run to exercise the
    analyser end to end. Contains no physics and is never used in a real run."""
    def rec(scale):
        return {
            "actual_rad": {"left_aileron": -0.05 * scale, "right_aileron": 0.05 * scale,
                           "left_elevator": 0.04 * scale, "right_elevator": 0.04 * scale,
                           "rudder": 0.03 * scale},
            "aero_input_rad": {"delta_a": 0.05 * scale, "delta_e": -0.04 * scale,
                               "delta_r": 0.03 * scale},
            "aero": {"Cl": 0.01 * scale, "Cm": 0.02 * scale, "Cn": -0.01 * scale},
            "attitude": {"roll_deg": 3.0 * scale, "pitch_deg": 2.0 * scale,
                         "yaw_deg": 1.0 * scale, "rollspeed_deg_s": 10.0 * scale,
                         "pitchspeed_deg_s": 6.0 * scale, "yawspeed_deg_s": 4.0 * scale},
            "nav": {"nav_roll_deg": 5.0 * scale, "nav_pitch_deg": 3.0 * scale},
            "servo_pwm_named": {"aileron": 1500 - 60 * scale, "elevator": 1500 + 60 * scale,
                                "rudder": 1500 - 30 * scale},
            "gz_body_rate_deg_s": [10.0 * scale, 6.0 * scale, -4.0 * scale],
        }
    before = [rec(0.0) for _ in range(30)]
    during = [rec(1.0) for _ in range(40)]
    after = [rec(0.0) for _ in range(30)]
    return before, during, after


if __name__ == "__main__":
    sys.exit(main())

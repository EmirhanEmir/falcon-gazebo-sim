#!/usr/bin/env python3
"""
FALCON V2 - ACTUATOR_CONTROL_LIMIT_CYCLE_FIX_AND_VALIDATION, STEP 4
supplement: SMALL-SIGNAL actuator time-constant bench.
Owner: gazebo-testing, 2026-09-12.

WHY THIS EXISTS SEPARATELY. The main standalone bench
(test_actuator_standalone_step_response.py) stores theta decimated to 10 ms,
which is enough for rise/settling/overshoot/duty but NOT enough to
distinguish a 10 ms time constant from a 20 ms one. Prediction P12 of
docs/test_results/2026-09-12_actuator_limit_cycle_fix_implementation.md is
exactly such a claim ("elevator small-signal actuator tau 0.058 s -> approx
0.010-0.020 s"), so it needs full-rate data to be judged at all. This bench
records theta and theta_dot at the FULL 1 kHz physics rate for 1.0 s after
each step edge and saves the raw series, so the numbers below can be
re-derived by anyone from the saved artifact rather than trusted.

Same bench, same world, same isolation, same no-flight conditions as
test_actuator_standalone_step_response.py - see that file's header. Commands
are small (+/-1 deg and +/-5 deg) specifically so the setpoint rate limiter
(300.0007 deg/s) is a negligible part of the response: a 1 deg step's
setpoint is complete in 3.3 ms and a 5 deg step's in 16.7 ms.

MEASUREMENT-ONLY. No physics parameter is written anywhere.
"""
import json
import math
import os
import sys
import time

import actuator_lib as ACT
import aero_lib as AL

ACT.setup_env()

import gz.math7 as gm  # noqa: E402
import gz.sim8 as sim  # noqa: E402

import test_actuator_standalone_step_response as BENCH  # shared bench code

REPO = "/home/emirhan/Desktop/FalconV2"
RESULTS_DIR = os.path.join(REPO, "tests/gazebo/results")
PREFIX = "actuator_fix_standalone"
DT = ACT.STEP

STEPS_DEG = [1.0, -1.0, 5.0, -5.0]
SETTLE_S = 1.0     # at 0 before each step
HOLD_S = 1.0       # recorded window after each step edge


def build_schedule():
    segs = []
    n = 0
    ns = int(round(SETTLE_S / DT))
    nh = int(round(HOLD_S / DT))
    segs.append(dict(name="PRE_ZERO", start_step=0, end_step=ns,
                     target_cmd_rad=0.0, prev_cmd_rad=0.0, kind="zero"))
    n = ns
    for a in STEPS_DEG:
        segs.append(dict(name=f"SMALL_STEP_{a:+.0f}DEG", start_step=n, end_step=n + nh,
                         target_cmd_rad=math.radians(a), prev_cmd_rad=0.0, kind="step"))
        n += nh
        segs.append(dict(name=f"SMALL_RETURN_{a:+.0f}DEG", start_step=n, end_step=n + ns,
                         target_cmd_rad=0.0, prev_cmd_rad=math.radians(a), kind="return"))
        n += ns
    return segs, n


def descriptive(th, th0, delta, b):
    """Model-free descriptors of one step edge, all in SI/deg. `b` is
    sp_weight_b - the P-term alone can only drive the surface to b*setpoint,
    so b*delta is the natural asymptote of the FAST mode and 1.0*delta the
    asymptote of the SLOW (integral) mode. Both are reported; neither is
    asserted to be 'the' time constant."""
    out = {}
    for label, frac, asy in (("t_to_63p2pct_of_full_step", 0.632, 1.0),
                             ("t_to_63p2pct_of_b_times_step", 0.632, b),
                             ("t_to_90pct_of_b_times_step", 0.90, b),
                             ("t_to_95pct_of_b_times_step", 0.95, b)):
        tgt = th0 + frac * asy * delta
        t = None
        for i, v in enumerate(th):
            if (delta > 0 and v >= tgt) or (delta < 0 and v <= tgt):
                t = i * DT
                break
        out[label] = t
    out["theta_at_50ms_frac_of_step"] = (th[50] - th0) / delta if len(th) > 50 else None
    out["theta_at_100ms_frac_of_step"] = (th[100] - th0) / delta if len(th) > 100 else None
    out["theta_at_1000ms_frac_of_step"] = (th[-1] - th0) / delta
    out["peak_frac_of_step"] = (max(th) - th0) / delta if delta > 0 else (min(th) - th0) / delta
    return out


def main():
    lines = []

    def log(m):
        print(m, flush=True)
        lines.append(m)

    cfgs = BENCH.load_per_surface_config()
    segs, total = build_schedule()
    log("=" * 78)
    log("ACTUATOR SMALL-SIGNAL TAU BENCH (STEP 4 supplement) - gazebo-testing 2026-09-12")
    log(f"world  : {BENCH.WORLD_SDF}")
    log(f"steps  : {total} per surface ({total*DT:.1f} s sim) at dt={DT}")
    log(f"steps commanded: {STEPS_DEG} deg")
    log("=" * 78)

    out = dict(test="ACTUATOR_FIX_SMALL_SIGNAL_TAU", date="2026-09-12",
               owner="gazebo-testing", world=BENCH.WORLD_SDF, dt_s=DT,
               commanded_steps_deg=STEPS_DEG, surfaces={})

    for surface in ACT.SURFACES:
        cfg = cfgs[surface]
        state, diag_hist = BENCH.run_surface(surface, segs, total, log)
        theta = state["theta"]
        rate = state["rate"]
        res = {}
        for s in segs:
            if s["kind"] != "step":
                continue
            a, b_ = s["start_step"], s["end_step"]
            th = theta[a:b_]
            th0 = theta[a - 1]
            delta = s["target_cmd_rad"] - th0
            d = descriptive(th, th0, delta, cfg["sp_weight_b"])
            d["step_size_deg"] = math.degrees(delta)
            d["theta_deg_full_rate_first_300ms"] = [round(math.degrees(v), 7) for v in th[:300]]
            d["rate_deg_s_full_rate_first_300ms"] = [round(math.degrees(v), 5) for v in rate[a:a + 300]]
            res[s["name"]] = d
            log(f"  {surface:<15} {s['name']:<20} "
                f"t63(full)={_f(d['t_to_63p2pct_of_full_step'])} "
                f"t63(b*step)={_f(d['t_to_63p2pct_of_b_times_step'])} "
                f"t90(b*step)={_f(d['t_to_90pct_of_b_times_step'])} "
                f"@50ms={d['theta_at_50ms_frac_of_step']:.4f} "
                f"@100ms={d['theta_at_100ms_frac_of_step']:.4f} "
                f"@1000ms={d['theta_at_1000ms_frac_of_step']:.4f}")
        out["surfaces"][surface] = dict(sp_weight_b=cfg["sp_weight_b"],
                                        any_nan_or_inf=state["nan"], steps=res)

    os.makedirs(RESULTS_DIR, exist_ok=True)
    rp = os.path.join(RESULTS_DIR, f"{PREFIX}_smallsignal_tau_result.json")
    with open(rp, "w") as f:
        json.dump(out, f, indent=2)
    with open(os.path.join(RESULTS_DIR, f"{PREFIX}_smallsignal_tau_log.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    log(f"wrote {rp}")
    return 0


def _f(v):
    return "n/a" if v is None else f"{v*1000:.0f}ms"


if __name__ == "__main__":
    sys.exit(main())

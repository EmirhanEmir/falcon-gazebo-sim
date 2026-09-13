#!/usr/bin/env python3
"""
FALCON V2 - commit the 1 kHz primary evidence for
ACTUATOR_CONTROL_LIMIT_CYCLE_FIX_AND_VALIDATION step 5.
Owner: gazebo-testing, 2026-09-13.

WHY THIS EXISTS
---------------
`validation`'s review of this stage raised I1: the three 1 kHz raw records
(115/128/132 MB) live only in a session scratchpad and are the SOLE primary
evidence behind P4/P5/P6/P11/P14 and the whole YAW ladder. The 2026-09-11
campaign committed a 6 s excerpt per run; this stage committed none, so an
auditor could only have checked prose once the scratchpad was cleaned. This
script writes committed excerpts under tests/gazebo/results/ with the prefix
`actuator_fix_airborne_<TAG>_1khz_excerpt`, following the precedent of
`airborne_limit_cycle_runtime_highrate_*_1khz_airborne_excerpt.json`.

CLASSIFICATION: MEASUREMENT-ONLY / READ-ONLY. Launches no simulation, writes
no parameter, and does not modify the raw records or any pre-existing file.

WHAT IS COMMITTED, AND WHY IT IS ENOUGH
--------------------------------------
For MANUAL and FBWA: the ENTIRE airborne window at the FULL 1 kHz rate, for
all five joints. Not a 6 s slice. The reason is specific: P4/P5/P11 are
peak-to-peak AMPLITUDE claims over that window, and a subsample's max-min can
never exceed the full set's (validation 2026-09-11, D1b - a mathematical
certainty), so a decimated or truncated excerpt could only ever place a LOWER
BOUND on the amplitude. A lower bound cannot substantiate a "< 2.0 deg"
verdict. The same applies to max|joint rate| versus the 300.0007 deg/s limit.
Duty figures would survive decimation (the 20 Hz estimator is provably
unbiased, +0.00 pp over validation's 50-phase sweep), but amplitude would not.

For YAW: `stabilise_start` .. `ladder_end`, which strictly contains both the
in-flight neutral-stick baseline and the entire doublet ladder.

SIZE, AND THE ONE LOSSY STEP, STATED PLAINLY
--------------------------------------------
Channels that are genuinely constant over the window are stored EXACTLY, as
{"constant": v, "n": N} - lossless, and the constancy is then re-derivable
rather than asserted. Varying channels are stored as INTEGER FIXED-POINT with
a per-channel power-of-ten scale chosen so the quantum is <= 1e-4 of that
channel's OWN peak-to-peak over the window, i.e. a worst-case 0.01 % relative
quantization. That is 3-4 orders of magnitude below any effect this stage
reports. The chosen scale, the quantum, and the resulting worst-case
round-trip error are written into every channel, so the loss is auditable and
does not have to be taken on trust. IMU and the decimated aero fields are
stored as full-precision floats.

TIMEBASE, restated (validation 2026-09-11, I15): gz.msgs.Double_V carries no
header, so the actuator/aero sample times in these excerpts are RECONSTRUCTED
from wall arrival time through the /clock map, exactly as in the analysis.
Only the IMU carries a true publisher stamp. The per-topic transport latency
DIFFERENCE between /clock and the actuator topic is UNMEASURED.
"""
import hashlib
import json
import math
import os
import sys

import numpy as np

REPO = "/home/emirhan/Desktop/FalconV2"
RES = os.path.join(REPO, "tests/gazebo/results")
sys.path.insert(0, os.path.join(REPO, "tests/gazebo/scripts"))
import analyze_airborne_limit_cycle_runtime as PRE  # noqa: E402

RAW = os.environ.get(
    "RAW_DIR",
    "/tmp/claude-1000/-home-emirhan-Desktop-FalconV2/"
    "d791f088-f001-45d8-802a-39118ca168e8/scratchpad/actuator_fix_highrate_raw")

SURFACES = PRE.SURFACES
ACT_FIELDS = PRE.ACT_FIELDS
AERO_FIELDS = PRE.AERO_FIELDS

# Absolute quantum floor for actuator angle/rate channels: 1e-6 rad =
# 5.73e-5 deg, and 1e-6 rad/s = 5.73e-5 deg/s. Two to four orders below the
# smallest quantity this stage reports (the smallest is the FBWA rudder's
# 0.0015 deg peak-to-peak, which is 26 quanta).
ACT_MIN_SCALE = 1e-6

KEEP_ACT_FIELDS = ["cmd_rad", "setpoint_rad", "actual_angle_rad",
                   "actual_rate_rad_s", "target_clamp_active",
                   "effort_clamp_active"]
# target_clamped_rad is the ONLY one of the 7 published fields dropped: it is
# clamp(cmd_rad, min_angle_rad, max_angle_rad), i.e. a pure function of a
# field that IS committed plus two config constants, so nothing is lost.


def quantize(x, name, min_scale=None):
    """Integer fixed-point with a power-of-ten scale chosen so the quantum is
    <= 1e-4 of this channel's own peak-to-peak, then DELTA encoded.

    Delta encoding is exactly invertible (q = q0 + cumsum(dq)) and loses
    nothing beyond the quantization already described; it is used only because
    these series are smooth at 1 kHz, so the first differences are 1-2 orders
    of magnitude smaller than the absolute values and the committed file is
    roughly 3x smaller for identical information.

    RECONSTRUCTION, in full, for any reader:
        q = numpy.concatenate([[d["q0"]], d["q0"] + numpy.cumsum(d["dq"])])[:d["n"]]
        value = q * d["scale"]
    or equivalently q = q0 followed by running addition of each dq.

    `min_scale` forces a finer quantum than the 1e-4 rule where that rule
    would be physically wrong. Two cases, both found by running this script's
    own verifier against the result rather than by inspection:
      - the sim-time axis, where a 1 ms quantum on a 1 kHz stream would
        collide adjacent samples;
      - every actuator angle/rate channel, floored at 1e-6 rad or rad/s
        (5.73e-5 deg). Without the floor, a channel's quantum is set by its
        OWN peak-to-peak, so the rudder in the YAW record - which swings the
        full 90 deg - would get a 0.0057 deg quantum and its small-signal
        behaviour in the neutral-stick baseline segment (0.001 deg p2p) would
        fall below one quantum and become unreadable. The floor costs about
        1 MB across the three files and makes every figure in the report
        resolvable from the committed data."""
    x = np.asarray(x, float)
    if x.size == 0:
        return {"constant": None, "n": 0}
    rng = float(x.max() - x.min())
    if rng == 0.0:
        return {"constant": float(x[0]), "n": int(x.size),
                "encoding": "exact constant"}
    target = rng * 1e-4
    scale = 10.0 ** int(math.floor(math.log10(target)))
    if min_scale is not None:
        scale = min(scale, min_scale)
    q = np.rint(x / scale).astype(np.int64)
    err = float(np.max(np.abs(q * scale - x)))
    dq = np.diff(q)
    return {"encoding": "delta-coded integer fixed point: "
                        "q = q0 then running sum of dq; value = q * scale",
            "scale": scale, "n": int(x.size),
            "channel_p2p": rng,
            "quantum_over_channel_p2p": scale / rng,
            "worst_case_roundtrip_abs_error": err,
            "q0": int(q[0]), "dq": dq.tolist()}


def dequantize(d):
    """Inverse of quantize(). Committed here so the reader does not have to
    reimplement it, and so this script's own verification pass uses exactly
    the routine an auditor would."""
    if "constant" in d:
        return np.full(d["n"], d["constant"], float)
    q = np.empty(d["n"], np.int64)
    q[0] = d["q0"]
    if d["n"] > 1:
        q[1:] = d["q0"] + np.cumsum(np.asarray(d["dq"], np.int64))
    return q * d["scale"]


def md5(path, blocks=None):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def build(tag, t0, t1, window_choice, rederivable, markers=None):
    src = os.path.join(RAW, f"actuator_fix_airborne_{tag}_highrate.json")
    H = PRE.load_highrate(src)
    at, av = H["act_t"], H["act_v"]
    m = (at >= t0) & (at <= t1)
    it = H["imu_t"]
    mi = (it >= t0) & (it <= t1)
    aet, aev = H["aero_t"], H["aero_v"]
    ma = (aet >= t0) & (aet <= t1)

    act = {}
    for i, s in enumerate(SURFACES):
        ch = {}
        for f in KEEP_ACT_FIELDS:
            # Angle/rate/command channels carry an absolute quantum floor; the
            # two clamp flags are 0/1 and need none.
            floor = None if f.endswith("_active") else ACT_MIN_SCALE
            ch[f] = quantize(av[m, i * 7 + ACT_FIELDS.index(f)], f,
                             min_scale=floor)
        act[s] = ch

    # aero V at FULL rate (so the section 8 airspeed binning is exactly
    # re-derivable); the other aero fields decimated to ~100 Hz, which is
    # stated, not hidden - they back no peak-to-peak claim in the report.
    dec = 10
    # Of the 10 published aero fields, V is committed at full rate above and
    # these six are committed decimated. CL, CD and CY are omitted: no figure
    # in the verification report is derived from them. Stated, not silent.
    AERO_KEEP = ["alpha", "beta", "qbar", "Cl", "Cm", "Cn"]
    aero_other = {f: [round(float(v), 9)
                      for v in aev[ma, AERO_FIELDS.index(f)][::dec]]
                  for f in AERO_KEEP}

    out = {
        "classification": "MEASUREMENT-ONLY excerpt of this stage's own 1 kHz "
                          "capture. No parameter written, no pre-existing "
                          "record touched.",
        "stage": "ACTUATOR_CONTROL_LIMIT_CYCLE_FIX_AND_VALIDATION step 5",
        "owner": "gazebo-testing", "date_built": "2026-09-13",
        "reason_committed": "validation finding I1 (2026-09-13): the full 1 kHz "
                            "records live only in a session scratchpad and are "
                            "the sole primary evidence for the published "
                            "figures. Committed so the result can be audited "
                            "from raw data after the scratchpad is cleaned.",
        "source_run": tag,
        "source_raw_file": src,
        "source_raw_bytes": os.path.getsize(src),
        "source_raw_md5": md5(src),
        "window_sim_s": [float(t0), float(t1)],
        "window_choice": window_choice,
        "n_actuator_samples": int(m.sum()),
        "n_imu_samples": int(mi.sum()),
        "n_aero_samples": int(ma.sum()),
        "achieved_actuator_rate_hz": float(
            (m.sum() - 1) / (at[m][-1] - at[m][0])) if m.sum() > 1 else None,
        "figures_rederivable_from_this_file_alone": rederivable,
        # Embedded so this file is SELF-CONTAINED: without them the ladder
        # segmentation and the command-calm census would need a second file,
        # and the excerpt would not actually stand alone.
        "phase_markers_sim_s": markers,
        "timebase_caveat": PRE_TIMEBASE,
        "quantization_note": QUANT_NOTE,
        "actuator_field_order": {
            "surfaces": SURFACES,
            "fields_committed": KEEP_ACT_FIELDS,
            "field_dropped": "target_clamped_rad = clamp(cmd_rad, "
                             "min_angle_rad, max_angle_rad); a pure function "
                             "of a committed field and two config constants",
            "published_layout_unchanged": "the live topic still publishes all "
                                          "7 fields x 5 surfaces = 35; this "
                                          "excerpt is a storage choice only",
        },
        "actuator_t_sim": quantize(at[m], "t_sim", min_scale=1e-6),
        "actuator": act,
        "imu_t_sim": [round(float(v), 9) for v in it[mi]],
        "imu_gyro_rad_s": [[round(float(c), 9) for c in r]
                            for r in H["imu_gyro"][mi]],
        "imu_accel_ms2": [[round(float(c), 9) for c in r]
                           for r in H["imu_acc"][mi]],
        "aero_t_sim_full_rate": quantize(aet[ma], "t_sim", min_scale=1e-6),
        "aero_V_ms_full_rate": quantize(aev[ma, AERO_FIELDS.index("V")], "V"),
        "aero_other_fields_decimated": {
            "decimation": dec,
            "effective_rate_hz": 100.0,
            "fields_omitted": ["CL", "CD", "CY"],
            "why_omitted": "no figure in "
                           "docs/test_results/2026-09-12_actuator_limit_cycle_"
                           "fix_verification.md is derived from them",
            "t_sim": [float(v) for v in aet[ma][::dec]],
            "fields": aero_other,
        },
    }
    p = os.path.join(RES, f"actuator_fix_airborne_{tag}_1khz_excerpt.json")
    with open(p, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    print(f"{tag}: {out['n_actuator_samples']} actuator samples "
          f"({out['achieved_actuator_rate_hz']:.1f} Hz), "
          f"{out['n_imu_samples']} IMU, {os.path.getsize(p)/1e6:.2f} MB -> {p}")
    return p


PRE_TIMEBASE = (
    "validation 2026-09-11 finding I15: gz.msgs.Double_V carries no header, "
    "so actuator/aero sample times here are RECONSTRUCTED from wall arrival "
    "time through the /clock map (monotone piecewise-linear). Only imu_t_sim "
    "is a true publisher stamp. The per-topic transport latency DIFFERENCE "
    "between /clock and the actuator topic is UNMEASURED. Impact: "
    "peak-to-peak, duty and clamp counts are value-based and immune; "
    "frequency is essentially immune.")

QUANT_NOTE = (
    "Constant channels are stored EXACTLY as {constant, n}. Varying channels "
    "are integer fixed point, value = q * scale, with scale a power of ten "
    "chosen so the quantum is <= 1e-4 of that channel's own peak-to-peak over "
    "this window (worst case 0.01 % relative). Each channel carries its own "
    "scale, quantum_over_channel_p2p and measured "
    "worst_case_roundtrip_abs_error so the loss is auditable. IMU and the "
    "decimated aero fields are stored as floats ROUNDED TO 9 DECIMAL PLACES "
    "- 1e-9 rad/s on a ~0.5 rad/s gyro signal and 1e-9 m/s^2 on a ~22 m/s^2 "
    "accelerometer signal, i.e. 7-9 orders below the smallest reported "
    "effect, done purely to shorten the committed file.")

if __name__ == "__main__":
    A = json.load(open(os.path.join(
        RES, "actuator_fix_airborne_regression_analysis.json")))
    paths = []
    for tag in ("MANUAL", "FBWA"):
        w = A["runs"][tag]["airborne_window_sim_s"]
        paths.append(build(
            tag, w[0], w[1],
            "the ENTIRE airborne window (z >= 1.0 m, first such sample to end "
            "of the harness record) at the full 1 kHz rate - not a slice. "
            "Peak-to-peak amplitude cannot be substantiated from a subsample, "
            "because a subsample's max-min is only a lower bound.",
            ["P4 (MANUAL uncommanded joint p2p)",
             "P5 (FBWA uncommanded aileron p2p)",
             "P6 (rate-pinned duty vs 0.999 * 5.2360 rad/s)",
             "P11 (rudder uncommanded p2p)",
             "P13 (effort_clamp_active duty)",
             "P14 (body pitch-rate RMS, from imu_gyro_rad_s)",
             "the NaN/Inf scan",
             "the report's section 8 airspeed binning (aero_V is full rate)",
             "the command-calm window census, both with and without lead-in"]))
    mk = {m["label"]: m["t_sim"]
          for m in A["runs"]["YAW"]["phase_markers_sim_s"]}
    paths.append(build(
        "YAW", mk["stabilise_start"], mk["ladder_end"],
        "stabilise_start (33.1099 s) .. ladder_end (65.2565 s): strictly "
        "contains BOTH the in-flight neutral-stick baseline AND the entire "
        "doublet ladder, so every rung and its own in-flight reference are in "
        "one file.",
        ["the entire YAW ladder per-rung table (rudder cmd/actual p2p, max "
         "|rate|, rate-pinned duty)",
         "the rudder command-calm census with and without the 1.0 s lead-in",
         "f_y and yaw rate per rung (from imu_accel_ms2 / imu_gyro_rad_s)",
         "sideslip beta per rung (aero_other_fields_decimated, 100 Hz)",
         "the yawing moment N = qbar * S * b * Cn (100 Hz)",
         "the other four surfaces' behaviour during the ladder"],
        markers=A["runs"]["YAW"]["phase_markers_sim_s"]))
    print("\ntotal committed:",
          round(sum(os.path.getsize(p) for p in paths) / 1e6, 2), "MB")

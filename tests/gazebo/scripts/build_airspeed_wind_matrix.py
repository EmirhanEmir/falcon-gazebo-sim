#!/usr/bin/env python3
"""
FALCON V2 - SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION
Cross-case airspeed/wind acceptance matrix builder (gazebo-testing, 2026-09-02).

Consumes the three per-case artifacts produced by
run_ardupilot_airspeed_wind_acceptance.sh {zero,headwind,tailwind}:

    tests/gazebo/results/airspeed_wind_acceptance_<case>_result.json
    tests/gazebo/results/airspeed_wind_acceptance_<case>_timeseries.json
    tests/gazebo/results/airspeed_wind_acceptance_<case>_dataflash/*.BIN

and emits the full comparison matrix, the pass/fail decisions that need more
than one case to evaluate, and a plain-text table.

NOTHING IS FLOWN HERE. This is pure post-processing; it re-reads captured data
and never touches a physics parameter, a .parm, an SDF or a plugin.

WHY DATAFLASH AT ALL
--------------------
ARSP.* (ArduPlane's airspeed sensor output) and TECS.* (TECS's internal
_TAS_state / _TAS_dem) are NOT exposed over MAVLink. They only exist in
ArduPlane's own .BIN log. The live test captures the Gazebo pitot topic and
VFR_HUD; this script supplies the other two of the three required quantities.

TIME ALIGNMENT (and why it is not circular)
-------------------------------------------
Dataflash time is ArduPlane's boot clock (TimeUS); the live test's time is
seconds since its own flight t0. The offset between them is aligned by
minimising the RMS difference between dataflash `GPS.Spd` and the live
Gazebo-ground-truth GROUNDSPEED - i.e. on a quantity that is NOT part of the
airspeed comparison under test, so the alignment cannot manufacture agreement
between ARSP.Airspeed and the pitot. The mode-change event (MODE -> FBWB) is
computed independently and reported alongside as a consistency cross-check.
Every dataflash window is additionally shrunk by a guard band at both ends so
that a residual alignment error of up to that guard cannot pull a transient
into a steady-state window.

ACCEPTANCE CRITERIA EVALUATED HERE (tolerances defined in
test_ardupilot_airspeed_wind_acceptance.py and re-read from the result JSONs,
never redefined):

  MG-1  matched-groundspeed airspeed delta, headwind:  +~5 m/s
  MG-2  matched-groundspeed airspeed delta, tailwind:  -~5 m/s
  MG-3/4 the same two, in exact identity form:
            (dV_air - dV_ground) == -dW  (mismatch-corrected)
  GS-1/2 closed-loop cruise groundspeed shift: -5 (headwind) / +5 (tailwind)
         while TECS holds the SAME airspeed in all three cases
  AP-1  ArduPlane (dataflash ARSP.Airspeed) agrees with the Gazebo pitot
  TE-1  TECS sees the real airspeed change (TECS.sp tracks it)
  E2T-1 EAS2TAS is the post-M-1-fix ~1.004, not the pre-fix ~1.033

Usage:
    python3 build_airspeed_wind_matrix.py
"""
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

REPO_ROOT = "/home/emirhan/Desktop/FalconV2"
RESULTS = f"{REPO_ROOT}/tests/gazebo/results"
# The +/-5 m/s acceptance matrix proper. All three are along world X.
CASES = ["zero", "headwind", "tailwind"]
# Brief frame sanity check only: a LATERAL (world +Y) airmass, which the three
# along-X cases above cannot distinguish from an X/Y swap in the ENU->NED wind
# rotation. It is deliberately NOT part of the matched-groundspeed or
# groundspeed-shift criteria (with no heading hold the aircraft weathervanes,
# so there is no clean matched-groundspeed steady state to compare).
SANITY_CASES = ["crosswind"]
ALL_CASES = CASES + SANITY_CASES
# Expected ArduPlane WIND bearing (deg, "wind is coming FROM") per case, given
# the Gazebo world-ENU airmass velocity that was commanded. ENU +X = East and
# +Y = North, so an airmass moving toward -X comes FROM the East (90 deg), an
# airmass moving toward +X comes FROM the West (-90/270 deg), and an airmass
# moving toward +Y comes FROM the South (180 deg).
EXPECTED_AP_WIND_BEARING_DEG = {"headwind": 90.0, "tailwind": -90.0, "crosswind": 180.0}
TH_AP_WIND_BEARING_DEG = 2.0

# Guard band trimmed off BOTH ends of every dataflash window, to absorb any
# residual time-alignment error. 3 s is >> the observed alignment residual and
# still leaves >= 9 s of the shortest (15 s) window.
GUARD_S = 3.0
# Alignment search, in seconds of dataflash-minus-flight time. The search is
# CENTRED ON the dataflash MODE->FBWB event, because the live test sets its
# flight t0 immediately after confirming that mode change; the true offset is
# therefore t_mode plus a small positive setup delay. The window is
# deliberately wide relative to that delay, and a result that lands on either
# boundary is treated as a FAILED alignment rather than used (an earlier
# version of this script used a fixed [-5, +25] s window, which railed at +25 s
# for every case because t_mode is 43-60 s into the ArduPlane boot clock; the
# resulting garbage is exactly the failure mode this guard now catches).
ALIGN_HALF_WINDOW_S = 20.0
ALIGN_STEP = 0.02
# An alignment is only trusted if the residual RMS between dataflash GPS.Spd
# and the live Gazebo groundspeed is below this. GPS.Spd is a 5 Hz synthesised
# groundspeed of the same physical quantity, so a correct alignment gives a
# residual of a few cm/s; anything above 0.5 m/s means the alignment is wrong.
ALIGN_RMS_MAX_MS = 0.5


def mean(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    return (sum(xs) / len(xs)) if xs else None


def stdev(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    if len(xs) < 2:
        return 0.0 if xs else None
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def stat(xs):
    v = [x for x in xs if x is not None and isinstance(x, (int, float)) and math.isfinite(x)]
    if not v:
        return dict(n=0, mean=None, std=None, min=None, max=None)
    return dict(n=len(v), mean=mean(v), std=stdev(v), min=min(v), max=max(v))


# =============================================================================
# dataflash
# =============================================================================
def load_dataflash(case):
    from pymavlink import DFReader
    files = sorted(glob.glob(f"{RESULTS}/airspeed_wind_acceptance_{case}_dataflash/*.BIN"))
    if not files:
        return None
    r = DFReader.DFReader_binary(files[-1])
    out = {k: [] for k in ("ARSP", "TECS", "GPS", "MODE", "ORGN", "BARO", "POS")}
    while True:
        m = r.recv_msg()
        if m is None:
            break
        t = m.get_type()
        if t in out:
            out[t].append(m.to_dict())
    out["_file"] = files[-1]
    return out


def series(rows, field):
    o = []
    for d in rows:
        v = d.get(field)
        if v is None:
            continue
        try:
            v = float(v)
        except (TypeError, ValueError):
            continue
        o.append((float(d["TimeUS"]) / 1e6, v))
    return o


def resample(sig, t_query):
    """Nearest-neighbour lookup with a hard 0.5 s validity limit (all these
    logs are >= 10 Hz, so a >0.5 s gap means the message really was absent)."""
    if not sig:
        return [None] * len(t_query)
    ts = [s[0] for s in sig]
    out = []
    j = 0
    for t in t_query:
        while j + 1 < len(ts) and abs(ts[j + 1] - t) <= abs(ts[j] - t):
            j += 1
        out.append(sig[j][1] if abs(ts[j] - t) <= 0.5 else None)
    return out


def align_offset(df, live_t, live_gs, centre):
    """Return (best_offset_s, rms, n, railed) minimising
    RMS(GPS.Spd(t+off) - live_gs(t)) over off in centre +/- ALIGN_HALF_WINDOW_S.
    Aligned on GROUNDSPEED, deliberately not on airspeed - see module docstring."""
    gps = series(df["GPS"], "Spd")
    if not gps or centre is None:
        return None, None, 0, None
    lo, hi = centre - ALIGN_HALF_WINDOW_S, centre + ALIGN_HALF_WINDOW_S
    best = (None, None, 0)
    off = lo
    while off <= hi:
        q = resample(gps, [t + off for t in live_t])
        pairs = [(a, b) for a, b in zip(q, live_gs) if a is not None and b is not None]
        if len(pairs) >= 50:
            rms = math.sqrt(sum((a - b) ** 2 for a, b in pairs) / len(pairs))
            if best[1] is None or rms < best[1]:
                best = (off, rms, len(pairs))
        off += ALIGN_STEP
    railed = (best[0] is not None
              and (abs(best[0] - lo) < 2 * ALIGN_STEP or abs(best[0] - hi) < 2 * ALIGN_STEP))
    return best[0], best[1], best[2], railed


def fbwb_entry_time(df):
    """Dataflash time of the last transition into FBWB (ModeNum 6)."""
    t = None
    for d in df["MODE"]:
        mn = d.get("ModeNum", d.get("Mode"))
        try:
            mn = int(mn)
        except (TypeError, ValueError):
            continue
        if mn == 6:
            t = float(d["TimeUS"]) / 1e6
    return t


# =============================================================================
# per-case assembly
# =============================================================================
def load_case(case):
    with open(f"{RESULTS}/airspeed_wind_acceptance_{case}_result.json") as f:
        res = json.load(f)
    with open(f"{RESULTS}/airspeed_wind_acceptance_{case}_timeseries.json") as f:
        ts = json.load(f)
    return res, ts


def window_bounds(seg, settle_s):
    ss = [s for s in seg["samples"] if s["t_seg"] >= settle_s]
    if not ss:
        return None
    return (ss[0]["t"], ss[-1]["t"])


def case_report(case):
    res, ts = load_case(case)
    settle = res["segment_plan"]["SETTLE_S"]
    segs = ts["segments"]
    rep = dict(case=case, wind_world_enu_mps=res["wind_world_enu_mps"],
               verdict=res.get("verdict"), failed_checks=res.get("failed_checks"),
               command_derivation=res.get("command_derivation"),
               sim_arspd_rnd=res.get("sim_arspd_rnd"),
               param_preconditions_all_ok=res["acceptance_checks"].get(
                   "param_preconditions_all_ok"),
               live=res["analysis"], windows={})

    # ---- live-side extras that need the raw samples --------------------------
    for k, seg in segs.items():
        ss = [s for s in seg["samples"] if s["t_seg"] >= settle]
        if not ss:
            continue
        # oscillation growth: std of the 2nd half vs the 1st half of the window.
        # >1 means the oscillation is growing over the window.
        half = len(ss) // 2
        a = [s["pitot_airspeed_mps"] for s in ss[:half]]
        b = [s["pitot_airspeed_mps"] for s in ss[half:]]
        sa, sb = stdev(a), stdev(b)
        rep["windows"].setdefault(k, {})["airspeed_osc_growth_ratio"] = (
            (sb / sa) if (sa and sa > 1e-9) else None)
        rep["windows"][k]["airspeed_std_first_half"] = sa
        rep["windows"][k]["airspeed_std_second_half"] = sb
        # exact predicted airspeed from Gazebo ground truth:
        #   |V_ground_world - V_wind_world|
        # This is heading- and attitude-independent and is the single most
        # direct check that the Gazebo pitot itself is right.
        pred, obs = [], []
        for s in ss:
            v = s["gz"]["v_body"]
            a_deg = s["gz"]["att_deg"]
            w = s.get("wind_world_mps")
            pv = s.get("pitot_airspeed_mps")
            if not (v and a_deg and w and pv is not None):
                continue
            r, p, y = (math.radians(a_deg[0]), math.radians(a_deg[1]),
                       math.radians(a_deg[2]))
            cr, sr = math.cos(r), math.sin(r)
            cp, sp = math.cos(p), math.sin(p)
            cy, sy = math.cos(y), math.sin(y)
            R = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                 (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                 (-sp,     cp * sr,                cp * cr))
            vw = [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]
            rel = [vw[i] - w[i] for i in range(3)]
            pred.append(math.sqrt(sum(q * q for q in rel)))
            obs.append(pv)
        rep["windows"][k]["pitot_minus_gz_predicted_airspeed_ms"] = stat(
            [o - q for o, q in zip(obs, pred)])
        rep["windows"][k]["gz_predicted_airspeed_ms"] = stat(pred)

    # ---- dataflash ----------------------------------------------------------
    df = load_dataflash(case)
    if df is None:
        rep["dataflash"] = dict(error="no .BIN found")
        return rep, res, ts

    live_t, live_gs = [], []
    for seg in segs.values():
        for s in seg["samples"]:
            v = s["gz"]["v_body"]
            if v and all(math.isfinite(x) for x in v):
                live_t.append(s["t"])
                live_gs.append(math.sqrt(sum(x * x for x in v)))
    t_mode = fbwb_entry_time(df)
    off, rms, n, railed = align_offset(df, live_t, live_gs, t_mode)
    rep["dataflash"] = dict(
        file=df["_file"], align_offset_s=off, align_rms_ms=rms, align_n=n,
        align_railed=railed,
        align_ok=(off is not None and not railed and rms is not None
                  and rms <= ALIGN_RMS_MAX_MS),
        align_rms_max_ms=ALIGN_RMS_MAX_MS,
        fbwb_entry_dataflash_s=t_mode,
        flight_t0_minus_fbwb_entry_s=((off - t_mode) if (off is not None and t_mode)
                                      else None),
        align_method="minimise RMS(GPS.Spd vs Gazebo ground-truth groundspeed); "
                     "NOT aligned on airspeed, so it cannot manufacture "
                     "ARSP-vs-pitot agreement",
        guard_band_s=GUARD_S,
        orgn_alt_m=stat([float(d["Alt"]) for d in df["ORGN"]])["mean"] if df["ORGN"] else None,
        baro_press_pa=stat([float(d["Press"]) for d in df["BARO"]])["mean"] if df["BARO"] else None,
        pos_alt_m=stat([float(d["Alt"]) for d in df["POS"]])["mean"] if df["POS"] else None)

    if off is None or not rep["dataflash"]["align_ok"]:
        rep["dataflash"]["error"] = ("dataflash time alignment failed - ARSP/TECS "
                                     "columns deliberately NOT produced rather than "
                                     "produced wrong")
        return rep, res, ts

    arsp_v = series(df["ARSP"], "Airspeed")
    arsp_u = series(df["ARSP"], "U")
    arsp_h = series(df["ARSP"], "H")
    tecs_sp = series(df["TECS"], "sp")
    tecs_spd = series(df["TECS"], "spdem")
    tecs_th = series(df["TECS"], "th")
    tecs_ph = series(df["TECS"], "ph")
    gps_spd = series(df["GPS"], "Spd")

    for k, seg in segs.items():
        wb = window_bounds(seg, settle)
        if wb is None:
            continue
        t_lo, t_hi = wb[0] + GUARD_S, wb[1] - GUARD_S
        if t_hi <= t_lo:
            continue
        tq = [t_lo + i * 0.05 for i in range(int((t_hi - t_lo) / 0.05) + 1)]
        tdf = [t + off for t in tq]
        w = rep["windows"].setdefault(k, {})
        w["df_window_flight_s"] = [t_lo, t_hi]
        av = resample(arsp_v, tdf)
        w["ARSP_Airspeed_ms"] = stat(av)
        uu = [x for x in resample(arsp_u, tdf) if x is not None]
        hh = [x for x in resample(arsp_h, tdf) if x is not None]
        w["ARSP_used_frac"] = (sum(1 for x in uu if x >= 0.5) / len(uu)) if uu else None
        w["ARSP_healthy_frac"] = (sum(1 for x in hh if x >= 0.5) / len(hh)) if hh else None
        sp = resample(tecs_sp, tdf)
        w["TECS_sp_TAS_ms"] = stat(sp)
        w["TECS_spdem_TAS_ms"] = stat(resample(tecs_spd, tdf))
        w["TECS_th_demand"] = stat(resample(tecs_th, tdf))
        w["TECS_ph_demand_deg"] = stat(resample(tecs_ph, tdf))
        w["GPS_Spd_ms"] = stat(resample(gps_spd, tdf))
        # EAS2TAS. PRIMARY = ratio of window MEANS. TECS.sp and ARSP.Airspeed
        # are logged by different scheduler slots, so a pointwise ratio picks up
        # the inter-message time skew; on an OSCILLATING segment that skew is
        # amplified into several percent of spurious spread (and can even
        # produce a physically impossible EAS2TAS < 1). Taking the ratio of the
        # window means cancels the skew to first order. The pointwise mean is
        # kept alongside for comparison with the prior stage, which used it on a
        # steady window where the two agree.
        mv, ma = stat(sp)["mean"], stat(av)["mean"]
        w["EAS2TAS_TECSsp_over_ARSP"] = (mv / ma) if (mv is not None and ma) else None
        w["EAS2TAS_pointwise_mean"] = stat(
            [s / a for s, a in zip(sp, av) if s is not None and a not in (None, 0)])["mean"]
        # the dataflash-side transport check
        pit = resample([(s["t"], s["pitot_airspeed_mps"])
                        for seg2 in segs.values() for s in seg2["samples"]
                        if s.get("pitot_airspeed_mps") is not None], tq)
        w["ARSP_minus_pitot_ms"] = stat(
            [a - p for a, p in zip(av, pit) if a is not None and p is not None])
    return rep, res, ts


# =============================================================================
# cross-case matrix
# =============================================================================
def build():
    reps, ress = {}, {}
    for c in ALL_CASES:
        try:
            rep, res, _ = case_report(c)
        except FileNotFoundError as exc:
            print(f"MISSING artifacts for case {c}: {exc}")
            continue
        reps[c], ress[c] = rep, res

    TH = ress[CASES[0]]["thresholds"] if CASES[0] in ress else {}
    out = dict(
        stage="SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION__AIRSPEED_WIND_MATRIX",
        cases=reps, thresholds=TH,
        note="All numbers are means over the SETTLED window of each segment. "
             "Live columns (Gazebo pitot, VFR_HUD, Gazebo groundspeed) are from "
             "the 20 Hz live capture; ARSP/TECS columns are from ArduPlane's own "
             "dataflash, time-aligned on groundspeed and guard-banded.")

    def W(case, seg, key, sub="mean"):
        try:
            v = reps[case]["live"][seg][key]
            return v[sub] if isinstance(v, dict) else v
        except (KeyError, TypeError):
            return None

    def D(case, seg, key, sub="mean"):
        try:
            v = reps[case]["windows"][seg][key]
            return v[sub] if isinstance(v, dict) else v
        except (KeyError, TypeError):
            return None

    # ---- the 3 x 3 table (+ groundspeed columns) ----------------------------
    rows = []
    for case in ALL_CASES:
        if case not in reps:
            continue
        for seg in reps[case]["live"]:
            rows.append(dict(
                case=case, segment=seg,
                wind_x_ms=reps[case]["wind_world_enu_mps"][0],
                wind_y_ms=reps[case]["wind_world_enu_mps"][1],
                demand_ms=None,
                gazebo_pitot_ms=W(case, seg, "pitot_airspeed_mps"),
                gazebo_pitot_std=W(case, seg, "pitot_airspeed_mps", "std"),
                arduplane_ARSP_ms=D(case, seg, "ARSP_Airspeed_ms"),
                arduplane_VFR_ms=W(case, seg, "vfr_airspeed_mps"),
                tecs_sp_TAS_ms=D(case, seg, "TECS_sp_TAS_ms"),
                tecs_spdem_TAS_ms=D(case, seg, "TECS_spdem_TAS_ms"),
                eas2tas=D(case, seg, "EAS2TAS_TECSsp_over_ARSP"),
                groundspeed_gz_ms=W(case, seg, "gz_groundspeed_mps"),
                groundspeed_gz_x_ms=W(case, seg, "gz_groundspeed_x_mps"),
                groundspeed_vfr_ms=W(case, seg, "vfr_groundspeed_mps"),
                groundspeed_gps_ms=D(case, seg, "GPS_Spd_ms"),
                throttle=W(case, seg, "throttle"),
                ap_wind_speed_ms=W(case, seg, "ap_wind_speed_ms"),
                arsp_used_frac=D(case, seg, "ARSP_used_frac"),
                ARSP_minus_pitot_ms=D(case, seg, "ARSP_minus_pitot_ms"),
                osc_growth_ratio=D(case, seg, "airspeed_osc_growth_ratio"),
                pitot_minus_gz_pred_ms=D(case, seg, "pitot_minus_gz_predicted_airspeed_ms"),
            ))
        for r in rows:
            if r["case"] == case:
                cd = reps[case]["command_derivation"]["commands"]
                key = "CRUISE" if "cruise" in r["segment"] else r["segment"].replace("P1_", "")
                if key in cd:
                    r["demand_ms"] = cd[key]["achieved_demand_ms"]
    out["table"] = rows

    # ---- matched-groundspeed comparisons -----------------------------------
    checks = {}
    comps = []

    def comp(name, ref_case, ref_seg, cas_case, cas_seg):
        if ref_case not in reps or cas_case not in reps:
            return None
        a_ref = W(ref_case, ref_seg, "pitot_airspeed_mps")
        a_cas = W(cas_case, cas_seg, "pitot_airspeed_mps")
        g_ref = W(ref_case, ref_seg, "gz_groundspeed_mps")
        g_cas = W(cas_case, cas_seg, "gz_groundspeed_mps")
        ap_ref = D(ref_case, ref_seg, "ARSP_Airspeed_ms")
        ap_cas = D(cas_case, cas_seg, "ARSP_Airspeed_ms")
        wx_ref = reps[ref_case]["wind_world_enu_mps"][0]
        wx_cas = reps[cas_case]["wind_world_enu_mps"][0]
        if None in (a_ref, a_cas, g_ref, g_cas):
            return None
        d = dict(name=name,
                 reference=f"{ref_case}/{ref_seg}", case=f"{cas_case}/{cas_seg}",
                 airspeed_ref_ms=a_ref, airspeed_case_ms=a_cas,
                 groundspeed_ref_ms=g_ref, groundspeed_case_ms=g_cas,
                 arduplane_airspeed_ref_ms=ap_ref, arduplane_airspeed_case_ms=ap_cas,
                 groundspeed_mismatch_ms=g_cas - g_ref,
                 delta_airspeed_ms=a_cas - a_ref,
                 delta_airspeed_arduplane_ms=((ap_cas - ap_ref)
                                              if None not in (ap_ref, ap_cas) else None),
                 expected_delta_airspeed_ms=-(wx_cas - wx_ref),
                 identity_delta_air_minus_delta_ground_ms=(a_cas - a_ref) - (g_cas - g_ref),
                 identity_expected_ms=-(wx_cas - wx_ref))
        d["delta_airspeed_error_ms"] = d["delta_airspeed_ms"] - d["expected_delta_airspeed_ms"]
        d["identity_error_ms"] = (d["identity_delta_air_minus_delta_ground_ms"]
                                  - d["identity_expected_ms"])
        comps.append(d)
        return d

    # MG-1 / MG-3 : headwind, matched at groundspeed ~16 m/s
    mg1 = comp("MG_HEADWIND_at_gs16", "zero", "P1_MG_LOW", "headwind", "P1_MG_HIGH")
    # MG-2 / MG-4 : tailwind, matched at groundspeed ~21 m/s
    mg2 = comp("MG_TAILWIND_at_gs21", "zero", "P1_MG_HIGH", "tailwind", "P1_MG_LOW")
    out["matched_groundspeed_comparisons"] = comps

    tol = TH.get("TH_MG_DELTA_TOL_MS", 0.6)
    itol = TH.get("TH_MG_IDENTITY_TOL_MS", 0.35)
    gtol = TH.get("TH_MG_GS_MISMATCH_MS", 1.0)
    for tag, d in (("MG1_headwind", mg1), ("MG2_tailwind", mg2)):
        checks[f"{tag}_groundspeeds_actually_matched"] = (
            d is not None and abs(d["groundspeed_mismatch_ms"]) <= gtol)
        checks[f"{tag}_airspeed_delta_correct"] = (
            d is not None and abs(d["delta_airspeed_error_ms"]) <= tol)
        checks[f"{tag}_identity_exact"] = (
            d is not None and abs(d["identity_error_ms"]) <= itol)
        checks[f"{tag}_arduplane_delta_matches_gazebo"] = (
            d is not None and d["delta_airspeed_arduplane_ms"] is not None
            and abs(d["delta_airspeed_arduplane_ms"] - d["delta_airspeed_ms"])
            <= TH.get("TH_AP_VS_PITOT_MS", 0.25))

    # ---- closed-loop cruise: TECS holds airspeed, groundspeed shifts -------
    gs_tol = TH.get("TH_GS_SHIFT_TOL_MS", 0.6)
    hold_tol = TH.get("TH_HOLD_MEAN_TOL_MS", 0.5)
    cl = {}
    for case in CASES:
        if case not in reps:
            continue
        wx = reps[case]["wind_world_enu_mps"][0]
        cl[case] = dict(
            wind_x_ms=wx,
            airspeed_ms=W(case, "P2_wind_on_cruise", "pitot_airspeed_mps"),
            airspeed_std=W(case, "P2_wind_on_cruise", "pitot_airspeed_mps", "std"),
            arduplane_ARSP_ms=D(case, "P2_wind_on_cruise", "ARSP_Airspeed_ms"),
            tecs_sp_TAS_ms=D(case, "P2_wind_on_cruise", "TECS_sp_TAS_ms"),
            tecs_spdem_TAS_ms=D(case, "P2_wind_on_cruise", "TECS_spdem_TAS_ms"),
            groundspeed_ms=W(case, "P2_wind_on_cruise", "gz_groundspeed_mps"),
            groundspeed_ref_zero_wind_ms=W(case, "P0_zero_wind_ref_cruise",
                                           "gz_groundspeed_mps"),
            throttle=W(case, "P2_wind_on_cruise", "throttle"),
            eas2tas=D(case, "P2_wind_on_cruise", "EAS2TAS_TECSsp_over_ARSP"))
        c = cl[case]
        if None not in (c["groundspeed_ms"], c["groundspeed_ref_zero_wind_ms"]):
            c["groundspeed_shift_ms"] = c["groundspeed_ms"] - c["groundspeed_ref_zero_wind_ms"]
            c["groundspeed_shift_expected_ms"] = wx
            c["groundspeed_shift_error_ms"] = c["groundspeed_shift_ms"] - wx
        demand = reps[case]["command_derivation"]["commands"]["CRUISE"]["achieved_demand_ms"]
        c["demand_ms"] = demand
        if c["airspeed_ms"] is not None:
            c["airspeed_error_vs_demand_ms"] = c["airspeed_ms"] - demand
    out["closed_loop_cruise"] = cl

    for case in CASES:
        if case not in cl:
            continue
        checks[f"CL_{case}_tecs_holds_airspeed"] = (
            cl[case].get("airspeed_error_vs_demand_ms") is not None
            and abs(cl[case]["airspeed_error_vs_demand_ms"]) <= hold_tol)
        checks[f"CL_{case}_groundspeed_shifts_by_wind"] = (
            cl[case].get("groundspeed_shift_error_ms") is not None
            and abs(cl[case]["groundspeed_shift_error_ms"]) <= gs_tol)

    # airspeed held EQUAL across the three cases (the "TECS holds airspeed" claim
    # stated across cases rather than against the demand)
    asps = [cl[c]["airspeed_ms"] for c in cl
            if c in CASES and cl[c]["airspeed_ms"] is not None]
    out["closed_loop_airspeed_spread_ms"] = (max(asps) - min(asps)) if len(asps) > 1 else None
    checks["CL_airspeed_identical_across_all_three_cases"] = (
        out["closed_loop_airspeed_spread_ms"] is not None
        and out["closed_loop_airspeed_spread_ms"] <= hold_tol)

    # ---- brief lateral-airmass (crosswind) frame sanity ---------------------
    sanity = {}
    for case in ALL_CASES:
        if case not in reps:
            continue
        exp = EXPECTED_AP_WIND_BEARING_DEG.get(case)
        obs = W(case, "P2_wind_on_cruise", "ap_wind_direction_deg")
        spd = W(case, "P2_wind_on_cruise", "ap_wind_speed_ms")
        wv = reps[case]["wind_world_enu_mps"]
        wmag = math.sqrt(wv[0] ** 2 + wv[1] ** 2)
        sanity[case] = dict(commanded_wind_world_enu=wv,
                            expected_ap_bearing_deg=exp,
                            observed_ap_bearing_deg=obs,
                            expected_ap_speed_ms=wmag,
                            observed_ap_speed_ms=spd)
        if exp is not None:
            err = (obs - exp + 180.0) % 360.0 - 180.0 if obs is not None else None
            sanity[case]["bearing_error_deg"] = err
            checks[f"FRAME_{case}_ap_wind_bearing_correct"] = (
                err is not None and abs(err) <= TH_AP_WIND_BEARING_DEG
                and spd is not None and abs(spd - wmag) <= TH.get("TH_AP_WIND_TOL_MS", 0.3))
    out["ap_wind_frame_sanity"] = sanity
    for case in SANITY_CASES:
        if case not in reps:
            continue
        dem = reps[case]["command_derivation"]["commands"]["CRUISE"]["achieved_demand_ms"]
        a = W(case, "P2_wind_on_cruise", "pitot_airspeed_mps")
        checks[f"SANITY_{case}_tecs_holds_airspeed"] = (
            a is not None and abs(a - dem) <= hold_tol)

    # ---- ArduPlane / TECS / EAS2TAS -----------------------------------------
    ap_err, e2t, used = [], [], []
    for case in reps:
        for seg in reps[case]["windows"]:
            v = D(case, seg, "ARSP_minus_pitot_ms")
            if v is not None:
                ap_err.append(abs(v))
            e = D(case, seg, "EAS2TAS_TECSsp_over_ARSP")
            if e is not None:
                e2t.append(e)
            u = D(case, seg, "ARSP_used_frac")
            if u is not None:
                used.append(u)
    out["worst_ARSP_minus_pitot_ms"] = max(ap_err) if ap_err else None
    out["dataflash_alignment_ok_all_cases"] = all(
        reps[c]["dataflash"].get("align_ok") for c in reps)
    out["eas2tas_range"] = [min(e2t), max(e2t)] if e2t else None
    out["min_arsp_used_frac"] = min(used) if used else None
    checks["AP1_arduplane_dataflash_agrees_with_gazebo_pitot"] = (
        out["worst_ARSP_minus_pitot_ms"] is not None
        and out["worst_ARSP_minus_pitot_ms"] <= TH.get("TH_AP_VS_PITOT_MS", 0.25))
    checks["E2T1_eas2tas_is_post_fix_datum"] = (
        out["eas2tas_range"] is not None
        and out["eas2tas_range"][1] <= TH.get("TH_EAS2TAS_MAX", 1.015))
    checks["dataflash_time_alignment_ok"] = bool(out["dataflash_alignment_ok_all_cases"])
    checks["ARSP_sensor_used_in_flight"] = (
        out["min_arsp_used_frac"] is not None
        and out["min_arsp_used_frac"] >= TH.get("TH_ARSP_USED_FRAC_MIN", 0.99))

    # TECS demonstrably SEES the airspeed change: its own _TAS_state must track
    # the matched-groundspeed airspeed delta.
    for tag, d in (("MG1_headwind", mg1), ("MG2_tailwind", mg2)):
        if d is None:
            checks[f"TE1_{tag}_tecs_sees_the_change"] = False
            continue
        rc, rs = d["reference"].split("/")
        cc, cs = d["case"].split("/")
        t_ref, t_cas = D(rc, rs, "TECS_sp_TAS_ms"), D(cc, cs, "TECS_sp_TAS_ms")
        if None in (t_ref, t_cas):
            checks[f"TE1_{tag}_tecs_sees_the_change"] = False
            continue
        d["delta_TECS_sp_TAS_ms"] = t_cas - t_ref
        # TECS.sp is TAS, i.e. EAS * EAS2TAS (~1.004), so the expected TAS delta
        # is the EAS delta scaled by that factor - NOT the raw EAS delta.
        e2 = D(cc, cs, "EAS2TAS_TECSsp_over_ARSP") or 1.0
        d["delta_TECS_sp_expected_ms"] = d["expected_delta_airspeed_ms"] * e2
        d["delta_TECS_sp_error_ms"] = d["delta_TECS_sp_TAS_ms"] - d["delta_TECS_sp_expected_ms"]
        checks[f"TE1_{tag}_tecs_sees_the_change"] = abs(d["delta_TECS_sp_error_ms"]) <= tol

    # ---- per-case verdicts carried through ---------------------------------
    for case in reps:
        checks[f"per_case_{case}_passed"] = (
            reps[case]["verdict"] == "AIRSPEED_WIND_ACCEPTANCE_PASS")

    out["checks"] = checks
    out["failed_checks"] = [k for k, v in checks.items() if not v]
    out["verdict"] = ("AIRSPEED_WIND_ACCEPTANCE_MATRIX_PASS" if not out["failed_checks"]
                      else "AIRSPEED_WIND_ACCEPTANCE_MATRIX_FAILED")
    return out


def fmt(v, w=9, p=3):
    if v is None:
        return " " * (w - 3) + "n/a"
    return f"{v:{w}.{p}f}"


def text_table(out):
    L = []
    L.append("FALCON V2 - AIRSPEED / WIND ACCEPTANCE MATRIX")
    L.append("stage: SITL_ATMOSPHERE_AND_PITOT_INTEGRATION_VALIDATION")
    L.append("")
    L.append("Wind = velocity of the AIR MASS, world ENU (WIND.md sec 2). Nose is +X,")
    L.append("so wind_x = -5 is a HEADWIND and wind_x = +5 is a TAILWIND. wind_y is the")
    L.append("lateral (ENU North) component, non-zero only in the crosswind sanity case.")
    L.append("NOTE: the P0 segment of EVERY case is flown at ZERO wind by design (it is the")
    L.append("common reference); the case's wind is applied from P2 onward. The wndX/wndY")
    L.append("columns show the CASE's wind, not the segment's - read P0 rows as zero wind.")
    L.append("All values are means over the settled window. AIRSPEED columns are EAS")
    L.append("except TECS.sp/spdem which are TAS.")
    L.append("")
    hdr = (f"{'case':9s} {'segment':22s} {'wndX':>5s} {'wndY':>5s} {'dem':>6s} "
           f"{'gzPitot':>9s} {'ARSP':>9s} {'VFR_as':>9s} {'TECS.sp':>9s} "
           f"{'TECSdem':>9s} {'EAS2TAS':>8s} {'GS(gz)':>9s} {'GS(gps)':>9s} "
           f"{'GS(vfr)':>9s} {'thr':>6s} {'APwind':>7s}")
    L.append(hdr)
    L.append("-" * len(hdr))
    for r in out["table"]:
        L.append(f"{r['case']:9s} {r['segment']:22s} {r['wind_x_ms']:5.1f} "
                 f"{r['wind_y_ms']:5.1f} "
                 f"{fmt(r['demand_ms'],6,2)} {fmt(r['gazebo_pitot_ms'])} "
                 f"{fmt(r['arduplane_ARSP_ms'])} {fmt(r['arduplane_VFR_ms'])} "
                 f"{fmt(r['tecs_sp_TAS_ms'])} {fmt(r['tecs_spdem_TAS_ms'])} "
                 f"{fmt(r['eas2tas'],8,5)} {fmt(r['groundspeed_gz_ms'])} "
                 f"{fmt(r['groundspeed_gps_ms'])} {fmt(r['groundspeed_vfr_ms'])} "
                 f"{fmt(r['throttle'],6,4)} {fmt(r['ap_wind_speed_ms'],7,3)}")
    L.append("")
    L.append("MATCHED-GROUNDSPEED COMPARISONS (the owner's literal criterion)")
    L.append("-" * 78)
    for d in out["matched_groundspeed_comparisons"]:
        L.append(f"{d['name']}:  {d['reference']}  ->  {d['case']}")
        L.append(f"    groundspeed  {d['groundspeed_ref_ms']:.3f} -> {d['groundspeed_case_ms']:.3f}"
                 f"   (mismatch {d['groundspeed_mismatch_ms']:+.3f} m/s)")
        L.append(f"    airspeed     {d['airspeed_ref_ms']:.3f} -> {d['airspeed_case_ms']:.3f}"
                 f"   delta {d['delta_airspeed_ms']:+.3f} m/s"
                 f"   (expected {d['expected_delta_airspeed_ms']:+.3f},"
                 f" err {d['delta_airspeed_error_ms']:+.3f})")
        if d.get("delta_airspeed_arduplane_ms") is not None:
            L.append(f"    ArduPlane    {d['arduplane_airspeed_ref_ms']:.3f} -> "
                     f"{d['arduplane_airspeed_case_ms']:.3f}"
                     f"   delta {d['delta_airspeed_arduplane_ms']:+.3f} m/s")
        L.append(f"    identity  (dV_air - dV_ground) = "
                 f"{d['identity_delta_air_minus_delta_ground_ms']:+.4f}"
                 f"   expected {d['identity_expected_ms']:+.3f}"
                 f"   err {d['identity_error_ms']:+.4f}")
        if d.get("delta_TECS_sp_TAS_ms") is not None:
            L.append(f"    TECS.sp   delta {d['delta_TECS_sp_TAS_ms']:+.3f} TAS m/s"
                     f"   expected {d['delta_TECS_sp_expected_ms']:+.3f}"
                     f"   err {d['delta_TECS_sp_error_ms']:+.3f}")
        L.append("")
    L.append("CLOSED-LOOP CRUISE (TECS holds AIRSPEED; GROUNDSPEED moves)")
    L.append("-" * 78)
    L.append(f"{'case':10s} {'wndX':>5s} {'demand':>7s} {'airspeed':>9s} {'err':>7s} "
             f"{'ARSP':>8s} {'TECS.sp':>8s} {'GS':>8s} {'GSshift':>8s} {'err':>7s} {'thr':>7s}")
    for case, c in out["closed_loop_cruise"].items():
        L.append(f"{case:10s} {c['wind_x_ms']:5.1f} {fmt(c['demand_ms'],7,2)} "
                 f"{fmt(c['airspeed_ms'])} {fmt(c.get('airspeed_error_vs_demand_ms'),7,3)} "
                 f"{fmt(c['arduplane_ARSP_ms'],8,3)} {fmt(c['tecs_sp_TAS_ms'],8,3)} "
                 f"{fmt(c['groundspeed_ms'],8,3)} {fmt(c.get('groundspeed_shift_ms'),8,3)} "
                 f"{fmt(c.get('groundspeed_shift_error_ms'),7,3)} {fmt(c['throttle'],7,4)}")
    L.append("")
    L.append(f"closed-loop airspeed spread across all 3 cases: "
             f"{fmt(out['closed_loop_airspeed_spread_ms'],6,4)} m/s")
    L.append(f"worst |ARSP - gazebo pitot|: {fmt(out['worst_ARSP_minus_pitot_ms'],6,4)} m/s")
    L.append(f"EAS2TAS range: {out['eas2tas_range']}")
    L.append(f"min ARSP used fraction: {out['min_arsp_used_frac']}")
    L.append("")
    L.append("ArduPlane WIND-ESTIMATE FRAME SANITY (bearing = wind is coming FROM)")
    L.append("-" * 78)
    for case, sn in out.get("ap_wind_frame_sanity", {}).items():
        L.append(f"  {case:10s} commanded ENU {sn['commanded_wind_world_enu']}  ->  "
                 f"ArduPlane {fmt(sn['observed_ap_speed_ms'],6,3)} m/s @ "
                 f"{fmt(sn['observed_ap_bearing_deg'],7,2)} deg"
                 + (f"   (expected {sn['expected_ap_bearing_deg']:.1f} deg, err "
                    f"{sn['bearing_error_deg']:+.2f})"
                    if sn.get("bearing_error_deg") is not None else "   (zero wind)"))
    L.append("")
    L.append("CHECKS")
    L.append("-" * 78)
    for k, v in out["checks"].items():
        L.append(f"  {'PASS' if v else 'FAIL'}  {k}")
    L.append("")
    L.append(f"VERDICT: {out['verdict']}   failed={out['failed_checks']}")
    return "\n".join(L)


def main():
    out = build()
    oj = f"{RESULTS}/airspeed_wind_acceptance_matrix.json"
    ot = f"{RESULTS}/airspeed_wind_acceptance_matrix.txt"
    with open(oj, "w") as f:
        json.dump(out, f, indent=2, default=str)
    txt = text_table(out)
    with open(ot, "w") as f:
        f.write(txt + "\n")
    print(txt)
    print()
    print("wrote", oj)
    print("wrote", ot)
    return 0 if not out["failed_checks"] else 1


if __name__ == "__main__":
    sys.exit(main())

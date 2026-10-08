"""Fidelity metrics of the replay vs the field data (same metric set and thresholds as the
ZuriTwinBench paper / calibration/geh_evaluator.py).

Counts are rising edges (0->1 loop transitions) per period (default 1 h).  Per detector over the
periods of a day:  daily + per-period GEH (DfT WebTAG M3.1, < 5), nRMSE (FHWA), nMAE (FHWA), MAPE
(FHWA), R2 (NCHRP 765, > 0.90), PBIAS (ASCE), Willmott d, Theil U2.  Aggregates: DfT pass rate
(share of detectors with daily GEH < 5, needs >= 85 %) and R2 of simulated vs observed daily
counts across detectors (the paper's scatter-plot R2).
"""
import numpy as np, pandas as pd

# metric: (excellent, pass, higher_is_better, source)
THRESHOLDS = {
    "geh":   (2.0, 5.0, False, "DfT WebTAG M3.1 / FHWA TAT"),
    "nrmse": (5.0, 15.0, False, "FHWA Traffic Analysis Toolbox v3"),
    "nmae":  (5.0, 10.0, False, "FHWA Traffic Analysis Toolbox v3"),
    "mape":  (5.0, 15.0, False, "FHWA Calibration Guidelines"),
    "r2":    (0.95, 0.90, True, "NCHRP Report 765"),
    "pbias": (5.0, 10.0, False, "ASCE Model Calibration Protocol"),
    "d":     (0.95, 0.90, True, "Willmott (1981)"),
    "u2":    (0.50, 1.00, False, "Theil (1966)"),
}


def geh(sim, obs):
    sim, obs = np.asarray(sim, float), np.asarray(obs, float)
    den = sim + obs
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(den > 0, np.sqrt(2.0 * (sim - obs) ** 2 / den), 0.0)


def _r2(s, o):
    ss_t = np.sum((o - o.mean()) ** 2)
    return float(1 - np.sum((o - s) ** 2) / ss_t) if ss_t > 0 else float("nan")


def _d(s, o):
    den = np.sum((np.abs(s - o.mean()) + np.abs(o - o.mean())) ** 2)
    return float(1 - np.sum((s - o) ** 2) / den) if den > 0 else float("nan")


def _u2(s, o):
    if len(s) < 2:
        return float("nan")
    num, den = np.sqrt(np.mean((s[1:] - o[:-1]) ** 2)), np.sqrt(np.mean((o[1:] - o[:-1]) ** 2))
    return float(num / den) if den > 0 else float("nan")


def metrics(sim, obs):
    s, o = np.asarray(sim, float), np.asarray(obs, float)
    mo, m = o.mean(), o > 0
    gts = geh(s, o)
    nz = lambda v: float(v / mo * 100) if mo > 0 else float("nan")
    return {
        "obs_total": o.sum(), "sim_total": s.sum(), "geh_total": float(geh(s.sum(), o.sum())),
        "geh_p50": float(np.median(gts)), "geh_p90": float(np.percentile(gts, 90)),
        "geh_pct_pass": float(100 * (gts < 5).mean()),
        "nrmse": nz(np.sqrt(np.mean((s - o) ** 2))), "nmae": nz(np.mean(np.abs(s - o))),
        "mape": float(100 * np.mean(np.abs((s[m] - o[m]) / o[m]))) if m.any() else float("nan"),
        "r2": _r2(s, o), "pbias": float(100 * (s.sum() - o.sum()) / o.sum()) if o.sum() > 0 else float("nan"),
        "d": _d(s, o), "u2": _u2(s, o),
    }


def verdict(name, v):
    if pd.isna(v):
        return "n/a"
    exc, ok, higher, _ = THRESHOLDS[name]
    v = abs(v) if name == "pbias" else v
    return "excellent" if (v >= exc if higher else v <= exc) else ("pass" if (v >= ok if higher else v <= ok) else "FAIL")


def period_counts(df, dets, period_s):
    """Rising edges per period; df is indexed by consecutive seconds (row i = second r0+i)."""
    sod = np.asarray(df.index, int)
    bins = sod // period_s
    out = {}
    for d in dets:
        x = np.asarray(df[d].to_numpy()) > 0
        rise = np.concatenate([[False], x[1:] & ~x[:-1]])
        out[d] = pd.Series(rise).groupby(bins).sum()
    return pd.DataFrame(out)


def evaluate(obs_counts, sim_counts, dets, exclude=("d1",)):
    """obs_counts/sim_counts: DataFrames (periods x detectors).  Returns (per_detector, aggregate)."""
    rows = {}
    for d in dets:
        r = metrics(sim_counts[d], obs_counts[d])
        r["in_criterion"] = d not in exclude
        r["geh_total_verdict"] = verdict("geh", r["geh_total"])
        for k in ("nrmse", "nmae", "mape", "r2", "pbias", "d", "u2"):
            r[k + "_verdict"] = verdict(k, r[k])
        rows[d] = r
    pdet = pd.DataFrame(rows).T
    crit = pdet[pdet["in_criterion"].astype(bool)]
    o_tot, s_tot = crit["obs_total"].astype(float).to_numpy(), crit["sim_total"].astype(float).to_numpy()
    agg = {
        "detectors_in_criterion": len(crit),
        "geh_pass_rate_pct": float(100 * (crit["geh_total"].astype(float) < 5).mean()),
        "dft_webtag_m3_1": "PASS" if (crit["geh_total"].astype(float) < 5).mean() >= 0.85 else "FAIL",
        "r2_daily_counts_across_detectors": _r2(s_tot, o_tot),
        "nchrp765_r2": "PASS" if _r2(s_tot, o_tot) > 0.90 else "FAIL",
    }
    hob = np.concatenate([obs_counts[d].to_numpy() for d in crit.index])
    hsi = np.concatenate([sim_counts[d].to_numpy() for d in crit.index])
    agg["r2_all_detector_periods"] = _r2(hsi.astype(float), hob.astype(float))
    agg["geh_period_pass_rate_pct"] = float(100 * (geh(hsi, hob) < 5).mean())
    return pdet, agg


def format_fidelity(pdet, agg):
    cols = ["obs_total", "sim_total", "geh_total", "geh_p50", "geh_p90", "geh_pct_pass", "nrmse", "nmae",
            "mape", "r2", "pbias", "d", "u2"]
    out = pdet[cols].astype(float).round(3).to_string()
    out += "\n" + "\n".join(f"{k}: {round(v, 4) if isinstance(v, float) else v}" for k, v in agg.items())
    return out

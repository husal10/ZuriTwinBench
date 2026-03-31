#!/usr/bin/env python3
"""
geh_evaluator.py  —  Comprehensive Calibration Validator  (v3)
===============================================================
Zurich Intersection Digital Twin  |  Genser et al. (2023), Data in Brief 48, 109117

Corrections applied in this version
-------------------------------------
  D1  : Informational only. Tram detector 220 m upstream; ~50 % of N→S trams
        originate from a stop BETWEEN D1 and the junction (VBZ Cobra schedule).
        trams.rou.xml now uses per-tram departPos so the simulation correctly
        reproduces the 139 obs activations.  D1 is included in the report but
        not counted toward the DfT 85 % acceptance criterion.

  D4  : Raw CSV column d4 detects BOTH motor vehicles and cyclists (sg3 is the
        dedicated north-approach cycle signal, shared with sg2 motor phase).
        Corrected observed count = downstream balance:
          D4_motor = D8 + max(0, D10 − D7_motor × 0.10)
        Simulation detector aggregate = det_d4.xml + det_d4b.xml (both lanes).

  D6/D7: Raw CSV includes cyclists (sg3 serves south cyclists too). Corrected
         observed = rising edges only while sg6 = 1 (motor-vehicle green).
         D6 and D7 are reported SEPARATELY — no averaging.

  D9/D10: Always reported separately — no averaging or aggregation.

Metrics computed (all vs published standard thresholds)
--------------------------------------------------------
  GEH     < 5.0      DfT WebTAG M3.1 / FHWA TAT
  nRMSE   < 15 %     FHWA Traffic Analysis Toolbox v3
  nMAE    < 10 %     FHWA Traffic Analysis Toolbox v3
  MAPE    < 15 %     FHWA Calibration Guidelines
  R²      > 0.90     NCHRP Report 765 / TMC
  PBIAS   |x| < 10 % ASCE Model Calibration Protocol
  d       > 0.90     Willmott (1981)
  U2      < 1.0      Theil (1966)

Figures  →  calibration/figures/
  fig_01_geh_bar.png        GEH per detector + threshold bands
  fig_02_obs_vs_sim.png     Observed vs Simulated scatter
  fig_03_metrics_radar.png  Radar: normalised scores
  fig_04_timeseries.png     Hourly time-series overlay
  fig_05_residuals.png      Signed residual heat-map
  fig_06_metric_table.png   Full metric table with colour coding

Usage
-----
  python calibration/geh_evaluator.py
  python calibration/geh_evaluator.py --period 900   # 15-min
  python calibration/geh_evaluator.py --no-figures
"""

from __future__ import annotations
import argparse, sys, warnings, xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
import pandas as pd

warnings.filterwarnings("ignore", category=RuntimeWarning)

# ── Paths ─────────────────────────────────────────────────────────────────────
ROOT_DIR    = Path(__file__).parent.parent
CSV_DEFAULT = ROOT_DIR / "data"       / "feb04_only.csv"
DET_DEFAULT = ROOT_DIR / "simulation" / "output"
FIG_DIR     = Path(__file__).parent   / "figures"

# ── Palette ───────────────────────────────────────────────────────────────────
C_EXCELLENT = "#1a6632"
C_GOOD      = "#2878b5"
C_WARN      = "#e07b00"
C_FAIL      = "#b82020"
C_NAVY      = "#1e345a"
C_LGREY     = "#ebeef2"
C_CREAM     = "#fdfaf4"
C_MGREY     = "#8a96a5"
C_INFO      = "#7b5ea7"   # purple for informational-only detectors

# ── Detector map ──────────────────────────────────────────────────────────────
# informational=True  → included in report but NOT counted in DfT criterion
DETECTOR_MAP = {
    "d1":  {"csv_col": "d1",  "sim_ids": ["d1"],
            "label": "D1  Tram 220m↑",     "group": "Tram",       "informational": True},
    "d2":  {"csv_col": "d2",  "sim_ids": ["d2"],
            "label": "D2  Tram  50m↑",     "group": "Tram",       "informational": False},
    "d3":  {"csv_col": "d3",  "sim_ids": ["d3"],
            "label": "D3  Tram   1m↑",     "group": "Tram",       "informational": False},
    "d4":  {"csv_col": "d4",  "sim_ids": ["d4", "d4b"],           # both lanes
            "label": "D4  N 18m↑ (L0+L1)", "group": "Approach",   "informational": False,
            "obs_correction": "d4_motor"},                          # use motor-only obs
    "d5":  {"csv_col": "d5",  "sim_ids": ["d5a", "d5b"],
            "label": "D5  E 43m↑ (L0+L1)", "group": "Approach",   "informational": False},
    "d6":  {"csv_col": "d6",  "sim_ids": ["d6a", "d6b"],
            "label": "D6  S  2m↑ (L0+L1)", "group": "Approach",   "informational": False,
            "obs_correction": "d6_motor"},
    "d7":  {"csv_col": "d7",  "sim_ids": ["d7a", "d7b"],
            "label": "D7  S 15m↑ (L0+L1)", "group": "Approach",   "informational": False,
            "obs_correction": "d7_motor"},
    "d8":  {"csv_col": "d8",  "sim_ids": ["d8"],
            "label": "D8  S_out 50m↓",     "group": "Downstream", "informational": False},
    "d9":  {"csv_col": "d9",  "sim_ids": ["d9"],
            "label": "D9  W_out L0 10m↓",  "group": "Downstream", "informational": False},
    "d10": {"csv_col": "d10", "sim_ids": ["d10"],
            "label": "D10 W_out L1 10m↓",  "group": "Downstream", "informational": False},
}

# ── Thresholds ────────────────────────────────────────────────────────────────
THRESHOLDS = {
    "geh":   (2.0,  5.0,   "—",    "DfT WebTAG M3.1 / FHWA TAT",       False),
    "nrmse": (5.0,  15.0,  "% mn", "FHWA Traffic Analysis Toolbox v3",  False),
    "nmae":  (5.0,  10.0,  "% mn", "FHWA Traffic Analysis Toolbox v3",  False),
    "mape":  (5.0,  15.0,  "%",    "FHWA Calibration Guidelines",       False),
    "r2":    (0.95, 0.90,  "—",    "NCHRP Report 765 / TMC",            True),
    "pbias": (5.0,  10.0,  "%",    "ASCE Model Calibration Protocol",   False),
    "d":     (0.95, 0.90,  "—",    "Willmott (1981)",                   True),
    "u2":    (0.50, 1.00,  "—",    "Theil (1966) — U2<1 beats naïve",  False),
}

# ═════════════════════════════════════════════════════════════════════════════
# METRIC FUNCTIONS
# ═════════════════════════════════════════════════════════════════════════════

def _geh_scalar(q_sim, q_obs):
    if q_sim + q_obs == 0: return 0.0
    return float(np.sqrt(2.0 * (q_sim - q_obs) ** 2 / (q_sim + q_obs)))

def _geh_arr(sim, obs):
    d = sim + obs
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(d > 0, np.sqrt(2.0 * (sim - obs) ** 2 / d), 0.0)

def _rmse(s, o):  return float(np.sqrt(np.mean((s - o) ** 2)))
def _mae(s, o):   return float(np.mean(np.abs(s - o)))

def _mape(s, o):
    m = o > 0
    return float(100.0 * np.mean(np.abs((s[m] - o[m]) / o[m]))) if m.any() else float("nan")

def _r2(s, o):
    ss_r = np.sum((o - s) ** 2); ss_t = np.sum((o - o.mean()) ** 2)
    return float(1.0 - ss_r / ss_t) if ss_t > 0 else float("nan")

def _pbias(s, o):
    t = o.sum()
    return float(100.0 * (s.sum() - t) / t) if t > 0 else float("nan")

def _willmott_d(s, o):
    om = o.mean()
    num = np.sum((s - o) ** 2)
    den = np.sum((np.abs(s - om) + np.abs(o - om)) ** 2)
    return float(1.0 - num / den) if den > 0 else float("nan")

def _theil_u2(s, o):
    if len(s) < 2: return float("nan")
    sc = s[1:] - o[:-1]; oc = o[1:] - o[:-1]
    num = np.sqrt(np.mean(sc ** 2)); den = np.sqrt(np.mean(oc ** 2))
    return float(num / den) if den > 0 else float("nan")

def _norm(v, mean_o):
    return float(v / mean_o * 100.0) if mean_o > 0 else float("nan")

def metric_badge(mkey, value):
    if pd.isna(value): return "N/A", C_MGREY
    exc, pas, _, _, higher = THRESHOLDS[mkey]
    v = abs(value) if mkey == "pbias" else value
    if higher:
        if v >= exc: return "EXCELLENT ✓✓", C_EXCELLENT
        if v >= pas: return "GOOD ✓",       C_GOOD
        return "FAIL ✗", C_FAIL
    else:
        if v <= exc: return "EXCELLENT ✓✓", C_EXCELLENT
        if v <= pas: return "GOOD ✓",       C_GOOD
        return "FAIL ✗", C_FAIL

# ═════════════════════════════════════════════════════════════════════════════
# DATA LOADERS
# ═════════════════════════════════════════════════════════════════════════════

def load_sim(det_dir, det_id, period_s):
    xf = det_dir / f"det_{det_id}.xml"
    if not xf.exists(): return pd.Series(dtype=float, name=det_id)
    recs = []
    try:
        for iv in ET.parse(xf).getroot().findall("interval"):
            b = float(iv.get("begin", 0))
            n = int(iv.get("nVehContrib", iv.get("nVehEntered", 0)))
            recs.append({"b": b, "n": n})
    except ET.ParseError as e:
        print(f"  [WARN] {xf.name}: {e}")
        return pd.Series(dtype=float, name=det_id)
    if not recs: return pd.Series(dtype=float, name=det_id)
    dframe = pd.DataFrame(recs)
    dframe["bin"] = (dframe["b"] // period_s).astype(int) * period_s
    return dframe.groupby("bin")["n"].sum().rename(det_id)


def load_csv(csv_path, period_s):
    """Load CSV and compute per-period observed counts with motor-vehicle corrections.

    Motor-vehicle corrections (v3):
      d4  → downstream balance: D8 + max(0, D10 − max(D6_sg6,D7_sg6) × 0.10)
            Raw d4 includes cyclists activated by sg3 (bike phase).
      d6  → rising edges during sg6=1 only (motor-vehicle green on S_in).
      d7  → rising edges during sg6=1 only (motor-vehicle green on S_in).
      d1,d2,d3,d5,d8,d9,d10 → raw rising edges (no cyclists in loop).
    """
    df = pd.read_csv(csv_path, parse_dates=["time"])
    df = df.sort_values("time").reset_index(drop=True)
    t0 = df["time"].iloc[0].normalize()
    df["el"] = (df["time"] - t0).dt.total_seconds()
    for col in [c for c in df.columns if c != "time"]:
        df[col] = pd.to_numeric(df[col], errors="coerce").fillna(0)

    bins = np.arange(0, 86400 + period_s, period_s)
    df["bin"] = (pd.cut(df["el"], bins=bins, labels=bins[:-1].astype(int), right=False)
                 .astype(float).astype(int))

    rows = []
    for b, g in df.groupby("bin"):
        row = {"t": int(b)}
        # Raw rising edges for all detectors
        for d in ["d1","d2","d3","d4","d5","d6","d7","d8","d9","d10"]:
            row[d] = max(0, int((g[d].astype(float).diff() == 1).sum()))

        # ── Motor-vehicle corrections ──────────────────────────────────────
        # D6/D7 motor = rising edges only while sg6=1
        sg6_on = g["sg6"] == 1
        d6m = max(0, int((g.loc[sg6_on, "d6"].astype(float).diff() == 1).sum()))
        d7m = max(0, int((g.loc[sg6_on, "d7"].astype(float).diff() == 1).sum()))
        s_motor = max(d6m, d7m)
        # D4 motor = downstream balance
        s_to_w = round(s_motor * 0.10)
        d4m = row["d8"] + max(0, row["d10"] - s_to_w)

        row["d4_motor"] = d4m
        row["d6_motor"] = d6m
        row["d7_motor"] = d7m
        rows.append(row)

    return pd.DataFrame(rows).set_index("t")


# ═════════════════════════════════════════════════════════════════════════════
# METRICS BUNDLE
# ═════════════════════════════════════════════════════════════════════════════

def compute_metrics(obs_s, sim_s):
    obs = obs_s.values.astype(float)
    sim = sim_s.values.astype(float)
    o_day = obs.sum(); s_day = sim.sum(); mo = obs.mean()
    geh_ts = _geh_arr(sim, obs)
    rmse_v = _rmse(sim, obs); mae_v = _mae(sim, obs)
    return {
        "obs_daily":    o_day,
        "sim_daily":    s_day,
        "geh_daily":    _geh_scalar(s_day, o_day),
        "geh_p50":      float(np.median(geh_ts)),
        "geh_p90":      float(np.percentile(geh_ts, 90)),
        "geh_pct_pass": float(100.0 * (geh_ts < 5.0).mean()),
        "rmse":   rmse_v, "nrmse":  _norm(rmse_v, mo),
        "mae":    mae_v,  "nmae":   _norm(mae_v, mo),
        "mape":   _mape(sim, obs),
        "r2":     _r2(sim, obs),
        "pbias":  _pbias(sim, obs),
        "d":      _willmott_d(sim, obs),
        "u2":     _theil_u2(sim, obs),
        "_obs": obs, "_sim": sim, "_geh_ts": geh_ts,
    }


# ═════════════════════════════════════════════════════════════════════════════
# EVALUATE
# ═════════════════════════════════════════════════════════════════════════════

def evaluate(csv_path, det_dir, period_s, save_csv=True, make_figures=True):
    SEP = "═" * 80
    print(f"\n{SEP}")
    print("  Zurich Intersection Digital Twin — Comprehensive Calibration Validator  v3")
    print(f"  Period: {period_s}s ({period_s//3600}h {(period_s%3600)//60:02d}min)  |  "
          f"Genser et al. (2023), Data in Brief 48, 109117")
    print(SEP)

    if not csv_path.exists():
        print(f"[ERROR] CSV not found: {csv_path}"); sys.exit(1)

    print(f"\n[INFO] Loading {csv_path.name}")
    obs_df = load_csv(csv_path, period_s)
    t_bins = obs_df.index.values
    print(f"       {len(obs_df)} bins  |  motor corrections applied to d4/d6/d7")

    records, ts_data = [], {}

    for dk, cfg in DETECTOR_MAP.items():
        info = cfg.get("informational", False)
        corr_col = cfg.get("obs_correction")       # e.g. "d4_motor"

        # ── Observed series ───────────────────────────────────────────────
        if corr_col and corr_col in obs_df.columns:
            obs_s = obs_df[corr_col]
        else:
            obs_s = obs_df.get(cfg["csv_col"], pd.Series(0, index=obs_df.index))

        # ── Simulated series (aggregate multi-lane) ───────────────────────
        sim_c = pd.Series(0.0, index=obs_df.index, name=dk)
        any_sim = False
        for sid in cfg["sim_ids"]:
            s = load_sim(det_dir, sid, period_s)
            if len(s): any_sim = True
            sim_c = sim_c.add(s.reindex(obs_df.index, fill_value=0), fill_value=0)

        if not any_sim:
            records.append({"detector": dk, "label": cfg["label"],
                            "group": cfg["group"], "informational": info,
                            **{k: float("nan") for k in
                               ["obs_daily","sim_daily","geh_daily","geh_p50",
                                "geh_p90","geh_pct_pass","rmse","nrmse","mae",
                                "nmae","mape","r2","pbias","d","u2"]}})
            continue

        m = compute_metrics(obs_s, sim_c)
        ts_data[dk] = {"label": cfg["label"], "group": cfg["group"],
                       "informational": info,
                       "obs": m.pop("_obs"), "sim": m.pop("_sim"),
                       "geh_ts": m.pop("_geh_ts"), "t_bins": t_bins}
        records.append({"detector": dk, "label": cfg["label"],
                        "group": cfg["group"], "informational": info, **m})

    res_df = pd.DataFrame(records)
    _print_report(res_df)

    if save_csv:
        out = Path(__file__).parent / "validation_report.csv"
        res_df[[c for c in res_df.columns if not c.startswith("_")]].to_csv(out, index=False)
        print(f"\n  Report saved → {out}")

    if make_figures:
        FIG_DIR.mkdir(parents=True, exist_ok=True)
        print(f"\n  Generating figures → {FIG_DIR}/")
        _fig_geh_bar(res_df)
        _fig_obs_vs_sim(res_df)
        _fig_radar(res_df)
        _fig_timeseries(ts_data, period_s)
        _fig_residuals(ts_data)
        _fig_metric_table(res_df)
        print("  All 6 figures saved.")

    print(f"\n{SEP}\n")
    return res_df


# ═════════════════════════════════════════════════════════════════════════════
# CONSOLE REPORT
# ═════════════════════════════════════════════════════════════════════════════

def _print_report(res_df):
    MET = [("geh_daily","GEH<5 PASS"),("nrmse","nRMSE%<15"),("nmae","nMAE%<10"),
           ("mape","MAPE%<15"),("r2","R²>0.90"),("pbias","|PBias|<10"),
           ("d","d>0.90"),("u2","U2<1.0")]
    print(f"\n  {'─'*80}")
    hdr = f"  {'Detector':<16} {'Obs':>7} {'Sim':>7}"
    for _, lbl in MET: hdr += f"  {lbl:>12}"
    print(hdr)
    print("  " + "─"*16 + " " + "─"*7 + " " + "─"*7 + "  " + "  ".join(["─"*12]*len(MET)))

    for _, r in res_df.iterrows():
        if pd.isna(r.get("obs_daily", float("nan"))): continue
        tag = " [info]" if r.get("informational") else ""
        line = f"  {r['detector']+tag:<16} {r['obs_daily']:.0f}  {r.get('sim_daily',float('nan')):.0f}"
        for mk, _ in MET:
            v = r.get(mk, float("nan"))
            if pd.isna(v): line += f"  {'—':>12}"
            elif mk in ("r2","d","u2"): line += f"  {v:>12.4f}"
            else:
                vd = abs(v) if mk == "pbias" else v
                line += f"  {vd:>11.2f} "
        print(line)

    print(f"\n  {'─'*80}")
    # DfT criterion: exclude informational detectors
    scorable = res_df[~res_df.get("informational", pd.Series(False, index=res_df.index))
                      .fillna(False)].dropna(subset=["geh_daily"])
    if len(scorable):
        n = len(scorable)
        ne = (scorable["geh_daily"] < 2.0).sum()
        ng = (scorable["geh_daily"] < 4.0).sum()
        np_ = (scorable["geh_daily"] < 5.0).sum()
        pct = 100 * np_ / n
        crit = "✓ PASSED" if pct >= 85 else "✗ FAILED"
        print(f"\n  DfT WebTAG M3.1 — ≥85 % daily GEH < 5  (D1 excluded: informational):")
        print(f"    Excellent (GEH<2): {ne}/{n}  Good (GEH<4): {ng}/{n}  "
              f"Pass (GEH<5): {np_}/{n} ({pct:.0f}%)  →  {crit}")
        for mk, mn in [("r2","R²"),("mape","MAPE"),("d","Willmott d"),("u2","Theil U2")]:
            vals = scorable[mk].dropna()
            if len(vals):
                bdg, _ = metric_badge(mk, vals.mean())
                print(f"    Mean {mn}: {vals.mean():.4f}  →  {bdg}")


# ═════════════════════════════════════════════════════════════════════════════
# FIG 1 — GEH BAR
# ═════════════════════════════════════════════════════════════════════════════

def _fig_geh_bar(res_df):
    df = res_df.dropna(subset=["geh_daily"]).copy()
    labels = df["label"].tolist()
    vals   = df["geh_daily"].values
    p50    = df["geh_p50"].values
    p90    = df["geh_p90"].values
    is_info = df.get("informational", pd.Series(False, index=df.index)).fillna(False).values

    def barcol(v, info):
        if info: return C_INFO
        if v < 2: return C_EXCELLENT
        if v < 4: return C_GOOD
        if v < 5: return C_WARN
        return C_FAIL

    grp_col = {"Tram": "#5b8fc9", "Approach": C_NAVY, "Downstream": C_WARN}
    groups  = df["group"].tolist()

    fig, ax = plt.subplots(figsize=(14, 5.5))
    fig.patch.set_facecolor(C_CREAM); ax.set_facecolor(C_CREAM)
    x = np.arange(len(labels))
    bars = ax.bar(x, vals, width=0.5,
                  color=[barcol(v, info) for v, info in zip(vals, is_info)],
                  edgecolor=C_NAVY, linewidth=0.7, zorder=3, label="GEH (daily)")
    ax.scatter(x, p50, marker="D", s=52, color=C_NAVY, zorder=6,
               label="Per-period GEH median")
    ax.scatter(x, p90, marker="^", s=58, color="#6b2c8e", zorder=6,
               label="Per-period GEH p90")

    for y0, y1, c, a in [(0,2,"#1a6632",0.07),(2,4,"#2878b5",0.06),
                          (4,5,"#e07b00",0.06),(5,14,"#b82020",0.05)]:
        ax.axhspan(y0, y1, color=c, alpha=a, zorder=1)
    ax.axhline(5.0, color=C_FAIL, lw=2.0, ls="--", zorder=4,
               label="PASS/FAIL threshold (GEH=5)")
    ax.axhline(4.0, color=C_GOOD, lw=1.2, ls=":", zorder=4)
    ax.axhline(2.0, color=C_EXCELLENT, lw=1.2, ls=":", zorder=4)

    for bar, v, info in zip(bars, vals, is_info):
        tag = "*" if info else ""
        ax.text(bar.get_x() + bar.get_width()/2, v + 0.06, f"{v:.2f}{tag}",
                ha="center", va="bottom", fontsize=8.5, fontweight="bold",
                color=C_INFO if info else C_NAVY)

    grp_pos: dict = {}
    for i, g in enumerate(groups):
        grp_pos.setdefault(g, []).append(i)
    for g, idxs in grp_pos.items():
        ax.annotate(g, xy=(np.mean(idxs), -0.85), ha="center", fontsize=9,
                    color=grp_col.get(g, C_NAVY), fontweight="bold",
                    annotation_clip=False)

    ax.set_xlim(-0.6, len(labels) - 0.4)
    ax.set_ylim(0, max(vals.max() * 1.30, 6.5))
    ax.set_xticks(x); ax.set_xticklabels(labels, rotation=32, ha="right", fontsize=8.5)
    ax.set_ylabel("GEH Statistic", fontsize=11)
    ax.set_title("Fig. 1 — Daily GEH per Detector\n"
                 "GEH < 5 = PASS  (DfT WebTAG M3.1)  |  * = informational, excluded from criterion",
                 fontsize=11, fontweight="bold", color=C_NAVY, pad=12)
    ax.grid(axis="y", ls="--", lw=0.5, color=C_MGREY, zorder=2)
    ax.legend(fontsize=8, loc="upper right", framealpha=0.92, ncol=2)
    _sa(ax); fig.tight_layout()
    fig.savefig(FIG_DIR / "fig_01_geh_bar.png", dpi=200, bbox_inches="tight")
    plt.close(fig); print("    fig_01_geh_bar.png")


# ═════════════════════════════════════════════════════════════════════════════
# FIG 2 — OBS vs SIM SCATTER  (D9/D10 always separate)
# ═════════════════════════════════════════════════════════════════════════════

def _fig_obs_vs_sim(res_df):
    df = res_df.dropna(subset=["obs_daily","sim_daily"]).copy()
    grp_mk  = {"Tram": "^", "Approach": "o", "Downstream": "s"}
    grp_col = {"Tram": "#5b8fc9", "Approach": C_NAVY, "Downstream": C_WARN}

    fig, ax = plt.subplots(figsize=(7, 6.5))
    fig.patch.set_facecolor(C_CREAM); ax.set_facecolor(C_CREAM)
    mv = max(df["obs_daily"].max(), df["sim_daily"].max()) * 1.12
    ax.plot([0,mv],[0,mv], color=C_NAVY, lw=1.5, label="1:1 perfect", zorder=2)
    ax.fill_between([0,mv],[0,mv*0.90],[0,mv*1.10], color=C_GOOD, alpha=0.08, label="±10 %")
    ax.fill_between([0,mv],[0,mv*0.85],[0,mv*1.15], color=C_WARN, alpha=0.07, label="±15 %")

    for _, r in df.iterrows():
        g = r["group"]
        info = r.get("informational", False)
        ec   = C_INFO if info else C_NAVY
        ax.scatter(r["obs_daily"], r["sim_daily"],
                   marker=grp_mk.get(g,"o"), s=90,
                   color=C_INFO if info else grp_col.get(g, C_NAVY),
                   edgecolors=ec, linewidths=0.7, zorder=5)
        det_label = r["detector"] + ("*" if info else "")
        ax.annotate(det_label, (r["obs_daily"], r["sim_daily"]),
                    textcoords="offset points", xytext=(6,4),
                    fontsize=8, color=ec)

    obs_v = df["obs_daily"].values; sim_v = df["sim_daily"].values
    r2_all = float(np.corrcoef(obs_v, sim_v)[0,1] ** 2)
    ax.text(0.05, 0.93, f"R² = {r2_all:.4f}", transform=ax.transAxes,
            fontsize=12, color=C_EXCELLENT, fontweight="bold")
    ax.text(0.05, 0.87, "Threshold: R² > 0.90  (NCHRP 765)",
            transform=ax.transAxes, fontsize=8, color=C_MGREY)
    ax.text(0.05, 0.82, "* = informational (D1)",
            transform=ax.transAxes, fontsize=7.5, color=C_INFO)

    for g, mk in grp_mk.items():
        ax.scatter([], [], marker=mk, color=grp_col[g], s=70, label=g,
                   edgecolors=C_NAVY, linewidths=0.7)
    ax.set_xlim(0, mv); ax.set_ylim(0, mv)
    ax.set_xlabel("Observed (veh/day)", fontsize=11)
    ax.set_ylabel("Simulated (veh/day)", fontsize=11)
    ax.set_title("Fig. 2 — Observed vs Simulated Daily Counts\n"
                 "R² > 0.90 = PASS (NCHRP 765)  |  D9 and D10 shown separately",
                 fontsize=10.5, fontweight="bold", color=C_NAVY, pad=10)
    ax.legend(fontsize=8.5, loc="upper left", framealpha=0.9)
    ax.grid(ls="--", lw=0.5, color=C_MGREY, zorder=1)
    _sa(ax); fig.tight_layout()
    fig.savefig(FIG_DIR / "fig_02_obs_vs_sim.png", dpi=200, bbox_inches="tight")
    plt.close(fig); print("    fig_02_obs_vs_sim.png")


# ═════════════════════════════════════════════════════════════════════════════
# FIG 3 — RADAR
# ═════════════════════════════════════════════════════════════════════════════

def _fig_radar(res_df):
    axes_def = [
        ("geh_daily","GEH\n(daily)",  2.0,  5.0, False),
        ("mape",     "MAPE %",        5.0, 15.0, False),
        ("r2",       "R²",           0.95, 0.90,  True),
        ("pbias",    "|PBIAS| %",     5.0, 10.0, False),
        ("d",        "Willmott d",   0.95, 0.90,  True),
        ("u2",       "Theil U2",      0.5,  1.0, False),
    ]
    NA = len(axes_def)
    angles = np.linspace(0, 2*np.pi, NA, endpoint=False).tolist(); angles += angles[:1]

    def score(val, exc, pas, higher):
        if pd.isna(val): return 0.0
        v = abs(val)
        if higher:
            if v >= exc: return 1.0
            if v >= pas: return 0.5 + 0.5*(v-pas)/(exc-pas)
            return max(0.0, 0.5*v/pas)
        else:
            if v <= exc: return 1.0
            if v <= pas: return 0.5 + 0.5*(pas-v)/(pas-exc)
            return max(0.0, 0.5*(1-(v-pas)/pas))

    df = res_df.dropna(subset=["geh_daily"]).copy()
    n = len(df); nc = min(5,n); nr = (n+nc-1)//nc
    fig, axs = plt.subplots(nr, nc, subplot_kw={"projection":"polar"},
                            figsize=(3.2*nc, 3.2*nr + 0.8))
    fig.patch.set_facecolor(C_CREAM)
    axs_flat = np.array(axs).reshape(-1)

    for idx, (_, row) in enumerate(df.iterrows()):
        ax = axs_flat[idx]; ax.set_facecolor(C_CREAM)
        info = row.get("informational", False)
        sc = [score(row.get(a[0],float("nan")),a[2],a[3],a[4]) for a in axes_def]
        sc += sc[:1]
        for lvl, lc, la in [(1.0,C_EXCELLENT,0.12),(0.5,C_WARN,0.12)]:
            ax.fill(angles,[lvl]*len(angles),color=lc,alpha=la)
        col = C_INFO if info else C_NAVY
        ax.plot(angles, sc, color=col, lw=1.8, zorder=5)
        ax.fill(angles, sc, color=col, alpha=0.22, zorder=4)
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels([a[1] for a in axes_def], size=7.5, color=C_NAVY)
        ax.set_yticks([0.0,0.5,1.0])
        ax.set_yticklabels(["Fail","Pass","Exc."], size=6, color=C_MGREY)
        ax.set_ylim(0, 1.05)
        tag = " [info]" if info else ""
        ax.set_title(row["label"]+tag, fontsize=8.5, fontweight="bold",
                     color=col, pad=10)
        ax.grid(color=C_MGREY, lw=0.5)

    for i in range(n, len(axs_flat)): axs_flat[i].set_visible(False)
    fig.suptitle("Fig. 3 — Normalised Metric Score Radar  (outer = Excellent, mid = Pass)",
                 fontsize=11, fontweight="bold", color=C_NAVY, y=1.01)
    fig.tight_layout()
    fig.savefig(FIG_DIR/"fig_03_metrics_radar.png", dpi=200, bbox_inches="tight")
    plt.close(fig); print("    fig_03_metrics_radar.png")


# ═════════════════════════════════════════════════════════════════════════════
# FIG 4 — TIME-SERIES
# ═════════════════════════════════════════════════════════════════════════════

def _fig_timeseries(ts_data, period_s):
    PRIO = ["d4","d5","d8","d9","d10","d2"]
    keys = [k for k in PRIO if k in ts_data] or list(ts_data.keys())[:6]
    nr = (len(keys)+1)//2
    fig, axs = plt.subplots(nr, 2, figsize=(13, 3.2*nr), sharex=False)
    fig.patch.set_facecolor(C_CREAM)
    axs = np.array(axs).reshape(-1)

    for i, dk in enumerate(keys):
        ax = axs[i]; ax.set_facecolor(C_CREAM)
        td = ts_data[dk]
        th = td["t_bins"] / 3600.0
        obs = td["obs"]; sim = td["sim"]; geh = td["geh_ts"]
        ax.fill_between(th, obs, alpha=0.15, color=C_NAVY)
        ax.plot(th, obs, color=C_NAVY, lw=1.8, label="Observed", zorder=4)
        ax.plot(th, sim, color=C_WARN, lw=1.8, ls="--", label="Simulated", zorder=4)
        ax2 = ax.twinx()
        ax2.bar(th, geh, width=period_s/3600.0*0.9, align="edge", alpha=0.35,
                color=[C_EXCELLENT if g<2 else C_GOOD if g<4
                       else C_WARN if g<5 else C_FAIL for g in geh], zorder=2)
        ax2.axhline(5.0, color=C_FAIL, lw=0.8, ls=":", zorder=3)
        ax2.set_ylim(0, max(geh.max()*1.5, 6.5))
        ax2.set_ylabel("GEH", fontsize=7.5, color=C_MGREY)
        ax2.tick_params(labelsize=6.5, colors=C_MGREY)
        ax.axvspan(1.0, 5.0, color="#aaaaaa", alpha=0.10)
        pct = 100*(geh < 5.0).mean()
        ax.set_title(f"{td['label']}  |  GEH pass rate: {pct:.0f} %",
                     fontsize=9, fontweight="bold", color=C_NAVY)
        ax.set_xlabel("Hour of day", fontsize=8)
        ax.set_ylabel("Count / period", fontsize=8)
        ax.set_xlim(0, 24); ax.set_xticks(range(0,25,4))
        ax.grid(ls="--", lw=0.5, color=C_MGREY, zorder=1)
        ax.legend(fontsize=7.5, loc="upper left"); _sa(ax)

    for i in range(len(keys), len(axs)): axs[i].set_visible(False)
    fig.suptitle("Fig. 4 — Time-Series: Observed vs Simulated\n"
                 "(bars = per-period GEH, shaded = night mode 01:00–05:00)",
                 fontsize=11, fontweight="bold", color=C_NAVY)
    fig.tight_layout(rect=[0,0,1,0.95])
    fig.savefig(FIG_DIR/"fig_04_timeseries.png", dpi=200, bbox_inches="tight")
    plt.close(fig); print("    fig_04_timeseries.png")


# ═════════════════════════════════════════════════════════════════════════════
# FIG 5 — RESIDUAL HEAT-MAP
# ═════════════════════════════════════════════════════════════════════════════

def _fig_residuals(ts_data):
    keys = list(ts_data.keys())
    labs = [ts_data[k]["label"] + (" [*]" if ts_data[k].get("informational") else "")
            for k in keys]
    tb   = ts_data[keys[0]]["t_bins"]
    mat  = np.zeros((len(keys), len(tb)))
    for i, dk in enumerate(keys):
        obs = ts_data[dk]["obs"]; sim = ts_data[dk]["sim"]
        with np.errstate(invalid="ignore", divide="ignore"):
            mat[i] = np.where(obs > 0, (sim - obs) / obs * 100.0, 0.0)

    cmap = LinearSegmentedColormap.from_list("r", [C_FAIL,"#f8f8f8",C_EXCELLENT], N=256)
    fig, ax = plt.subplots(figsize=(14, 4.5))
    fig.patch.set_facecolor(C_CREAM); ax.set_facecolor(C_CREAM)
    vmax = min(60, float(np.nanpercentile(np.abs(mat), 95)))
    im = ax.imshow(mat, aspect="auto", cmap=cmap, vmin=-vmax, vmax=vmax,
                   extent=[tb[0]/3600, tb[-1]/3600, len(keys)-0.5, -0.5])
    cb = fig.colorbar(im, ax=ax, shrink=0.85, pad=0.01)
    cb.set_label("(sim − obs) / obs × 100 %", fontsize=9)
    cb.ax.tick_params(labelsize=8)
    ax.set_yticks(range(len(keys))); ax.set_yticklabels(labs, fontsize=8.5)
    ax.set_xlabel("Hour of day", fontsize=10); ax.set_xticks(range(0,25,4))
    ax.axvline(1.0, color="#888", lw=0.8, ls=":")
    ax.axvline(5.0, color="#888", lw=0.8, ls=":")
    ax.text(3.0/24, -0.04, "Night mode", ha="center", fontsize=7.5,
            color=C_MGREY, transform=ax.get_xaxis_transform())
    ax.set_title("Fig. 5 — Signed Residual Heat-map  |  "
                 "Green = over-prediction  |  Red = under-prediction",
                 fontsize=10.5, fontweight="bold", color=C_NAVY, pad=10)
    _sa(ax); fig.tight_layout()
    fig.savefig(FIG_DIR/"fig_05_residuals.png", dpi=200, bbox_inches="tight")
    plt.close(fig); print("    fig_05_residuals.png")


# ═════════════════════════════════════════════════════════════════════════════
# FIG 6 — METRIC TABLE
# ═════════════════════════════════════════════════════════════════════════════

def _fig_metric_table(res_df):
    df = res_df.dropna(subset=["geh_daily"]).copy()
    if df.empty: return

    COL_DEF = [
        ("label",    "Detector",        "{}",      None,    3.0),
        ("obs_daily","Obs\nveh/day",    "{:.0f}",  None,    1.3),
        ("sim_daily","Sim\nveh/day",    "{:.0f}",  None,    1.3),
        ("geh_daily","GEH\n<5=PASS",   "{:.2f}",  "geh",   1.3),
        ("nrmse",    "nRMSE\n<15%",    "{:.1f}",  "nrmse", 1.2),
        ("nmae",     "nMAE\n<10%",     "{:.1f}",  "nmae",  1.2),
        ("mape",     "MAPE\n<15%",     "{:.1f}",  "mape",  1.2),
        ("r2",       "R²\n>0.90",      "{:.4f}",  "r2",    1.1),
        ("pbias",    "PBIAS\n|<10|",   "{:+.1f}", "pbias", 1.2),
        ("d",        "Willmott\nd>0.90","{:.4f}", "d",     1.3),
        ("u2",       "Theil\nU2<1.0",  "{:.3f}",  "u2",    1.2),
    ]
    col_widths = [c[4] for c in COL_DEF]
    fig_w = sum(col_widths) + 0.5; fig_h = 0.58*len(df) + 1.8
    rh = 0.58; top_y = fig_h - 0.85

    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    ax.set_visible(False); fig.patch.set_facecolor(C_CREAM)
    xs = [0.25]
    for w in col_widths[:-1]: xs.append(xs[-1]+w)

    BADGE_BG = {C_EXCELLENT:"#d4edda", C_GOOD:"#d0e4f5",
                C_WARN:"#fff3cd",      C_FAIL:"#f8d7da", C_MGREY:C_LGREY}

    def cell_bg(mkey, v):
        if mkey is None or pd.isna(v): return C_LGREY
        _, col = metric_badge(mkey, v)
        return BADGE_BG.get(col, C_LGREY)

    # Header row
    for ci, (_, lbl, _, _, w) in enumerate(COL_DEF):
        x = xs[ci]
        p = mpatches.FancyBboxPatch((x/fig_w,(top_y-rh)/fig_h), w/fig_w, rh/fig_h,
                boxstyle="square,pad=0", lw=0.5, ec=C_NAVY, fc=C_NAVY,
                transform=fig.transFigure, clip_on=False)
        fig.add_artist(p)
        fig.text((x+w/2)/fig_w, (top_y-rh/2)/fig_h, lbl, ha="center", va="center",
                 fontsize=7.5, fontweight="bold", color="white",
                 transform=fig.transFigure)

    # Data rows
    for ri, (_, row) in enumerate(df.iterrows()):
        y = top_y - (ri+1)*rh
        info = row.get("informational", False)
        base_bg = "#f5f0ff" if info else ("#ffffff" if ri%2==0 else C_LGREY)
        for ci, (ck, _, fmt, mkey, w) in enumerate(COL_DEF):
            x = xs[ci]; v = row.get(ck, float("nan"))
            bg = base_bg if (mkey is None or info) else cell_bg(mkey, v)
            p = mpatches.FancyBboxPatch((x/fig_w,y/fig_h), w/fig_w, rh/fig_h,
                    boxstyle="square,pad=0", lw=0.4, ec="#cccccc", fc=bg,
                    transform=fig.transFigure, clip_on=False)
            fig.add_artist(p)
            if ck == "label":
                txt = str(v) + (" *" if info else "")
            elif pd.isna(v):
                txt = "—"
            else:
                try:   txt = fmt.format(v)
                except: txt = str(v)
            tc = C_INFO if info else C_NAVY
            fig.text((x+w/2)/fig_w, (y+rh/2)/fig_h, txt, ha="center", va="center",
                     fontsize=7.8, color=tc, transform=fig.transFigure)

    fn = ("Thresholds:  GEH<5 PASS (DfT WebTAG M3.1)  |  nRMSE<15%, nMAE<10% (FHWA TAT v3)  |  "
          "MAPE<15% (FHWA)  |  R²>0.90 (NCHRP 765)  |  |PBIAS|<10% (ASCE)  |  "
          "d>0.90 (Willmott 1981)  |  U2<1 (Theil 1966)  |  * informational (D1 excluded from DfT criterion)")
    fig.text(0.5, 0.02, fn, ha="center", va="bottom", fontsize=6.5, color=C_MGREY,
             style="italic", transform=fig.transFigure)
    fig.suptitle("Fig. 6 — Full Validation Metric Table\n"
                 "Green=Excellent  Blue=Good/PASS  Amber=Warn  Red=Fail  Purple=Informational",
                 fontsize=10, fontweight="bold", color=C_NAVY, y=0.99)
    fig.savefig(FIG_DIR/"fig_06_metric_table.png", dpi=200, bbox_inches="tight")
    plt.close(fig); print("    fig_06_metric_table.png")


# ── Axis styling ──────────────────────────────────────────────────────────────
def _sa(ax):
    for sp in ["top","right"]: ax.spines[sp].set_visible(False)
    ax.spines["left"].set_color(C_MGREY); ax.spines["bottom"].set_color(C_MGREY)
    ax.tick_params(colors=C_NAVY, labelsize=8.5)


# ═════════════════════════════════════════════════════════════════════════════
# CLI
# ═════════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(
        description="Comprehensive Calibration Validator v3 — Zurich Intersection Digital Twin",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--csv",   type=Path, default=CSV_DEFAULT)
    p.add_argument("--det",   type=Path, default=DET_DEFAULT)
    p.add_argument("--period",type=int,  default=3600)
    p.add_argument("--no-save",    action="store_true")
    p.add_argument("--no-figures", action="store_true")
    args = p.parse_args()
    evaluate(args.csv, args.det, args.period,
             save_csv=not args.no_save, make_figures=not args.no_figures)

if __name__ == "__main__":
    main()

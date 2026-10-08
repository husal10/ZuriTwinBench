#!/usr/bin/env python3
"""Fidelity figures of the replay / actuated digital twin (matplotlib, one theme, print-style palette).

  python plots.py --csv ../data/feb04_only.csv --runs exact=out_exact hybrid=out_hybrid actuated=out_actuated \
                  --date 2019-02-04 --holdout-from "2019-02-04 12:00:00" --out results/figures

Palette (validated with the dataviz skill's validate_palette.js on the paper surface, light mode: lightness band, chroma
floor, adjacent CVD separation and normal-vision floor all PASS; ochre is below 3:1 on paper, so every series is also
direct-labelled): brick #b03a2a, ochre #d9a22c, fern #2f8a48, mulberry #9c4f93; observed data in ink.
"""
import argparse, sys
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import data as D, fidelity as F, signal_fidelity as SF

PAPER, INK, MUTED, GRID = "#f7f2e8", "#2a2723", "#6f675b", "#ddd4c3"
BRICK, OCHRE, FERN, MULB = "#b03a2a", "#d9a22c", "#2f8a48", "#9c4f93"
MODE_COL = {"actuated": BRICK, "hybrid": OCHRE, "exact": FERN}
MODE_LAB = {"actuated": "closed-loop actuated", "hybrid": "hybrid (signals replayed)", "exact": "exact replay"}
DETS = [f"d{i}" for i in range(1, 11)]
ROLE = {"d1": "in", "d2": "in", "d4": "in", "d5": "in", "d7": "in", "d3": "out", "d6": "out", "d8": "out", "d9": "out", "d10": "out"}
DNAME = {"d1": "PT 220 m", "d2": "PT 50 m", "d3": "PT 1 m", "d4": "N 18 m", "d5": "E 43 m", "d6": "S 2 m", "d7": "S 15 m",
         "d8": "S exit 50 m", "d9": "W exit L0", "d10": "W exit L1"}

plt.rcParams.update({
    "figure.facecolor": PAPER, "axes.facecolor": PAPER, "savefig.facecolor": PAPER,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK, "text.color": INK, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.7,
    "axes.axisbelow": True, "font.size": 9.5, "axes.titlesize": 10.5, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "legend.frameon": False, "lines.solid_capstyle": "round", "font.family": "DejaVu Sans",
})


def load(args):
    df = D.load([args.csv])
    date, g, _ = next(D.days(df))
    g = g.reset_index(drop=True)
    sims = {}
    for m, d in args.runs.items():
        p = Path(d) / date.strftime("%Y-%m-%d") / "sim_detectors.csv"
        s = pd.read_csv(p)
        sims[m] = s.drop(columns=["time"]).reset_index(drop=True)
    hrow = int((pd.Timestamp(args.holdout_from) - date).total_seconds()) if args.holdout_from else None
    return g, sims, hrow, date


def hourly(df):
    return F.period_counts(df, DETS, 3600)


def savefig(fig, out, name):
    fig.savefig(Path(out) / name, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("wrote", Path(out) / name)


# ---------------------------------------------------------------------------------------------------------------------
def fig_geh(g, sims, hrow, out):
    """Daily GEH per detector (DfT WebTAG M3.1 < 5) for the three modes; inputs are marked, only outputs are validation."""
    fig, ax = plt.subplots(figsize=(9.2, 4.6))
    modes = [m for m in ("exact", "hybrid", "actuated") if m in sims]
    obs = hourly(g)
    w = 0.26
    for k, m in enumerate(modes):
        sim = hourly(sims[m])
        gd = [float(F.geh(sim[d].sum(), obs[d].sum())) for d in DETS]
        xs = np.arange(len(DETS)) + (k - (len(modes) - 1) / 2) * w
        ax.bar(xs, gd, width=w - 0.04, color=MODE_COL[m], label=MODE_LAB[m], zorder=3,
               hatch=None)
        for x, v in zip(xs, gd):
            if v > 0.05:
                ax.text(x, v + 0.08, f"{v:.1f}", ha="center", va="bottom", fontsize=7, color=INK)
    ax.axhline(5, color=INK, lw=1.2, ls=(0, (4, 3)), zorder=4)
    ax.text(len(DETS) - 0.45, 5.08, "GEH = 5  (DfT WebTAG M3.1)", ha="right", va="bottom", fontsize=8.5)
    ax.set_xticks(range(len(DETS)))
    ax.set_xticklabels([f"{d}\n{DNAME[d]}\n" + ("input" if ROLE[d] == "in" else "output") for d in DETS], fontsize=8)
    for lab, d in zip(ax.get_xticklabels(), DETS):
        lab.set_color(MUTED if ROLE[d] == "in" else INK)
        lab.set_fontweight("normal" if ROLE[d] == "in" else "bold")
    ax.set_ylim(0, 6); ax.set_ylabel("daily GEH (vehicles per day)")
    ax.text(0.01, 0.72, "exact replay: GEH = 0 at all ten detectors (green bars have zero height)", transform=ax.transAxes, fontsize=8.5, color=FERN, fontweight="bold")
    ax.grid(axis="x", visible=False)
    ax.set_title("Detector fidelity: daily GEH per loop, 2019-02-04 (inputs are injected, outputs are validation)")
    ax.legend(loc="upper left", ncol=3, bbox_to_anchor=(0, 1.0), fontsize=8.5)
    savefig(fig, out, "fig1_detector_geh.png")


def fig_scatter(g, sims, hrow, out):
    """Hourly simulated vs observed counts, closed-loop run. Hollow = calibration half-day, filled = held-out half-day."""
    m = "actuated"
    obs, sim = hourly(g), hourly(sims[m])
    fig, axes = plt.subplots(2, 5, figsize=(12.2, 5.6))
    for ax, d in zip(axes.ravel(), DETS):
        o, s = obs[d].to_numpy(float), sim[d].to_numpy(float)
        lim = max(o.max(), s.max()) * 1.08 + 1
        ax.fill_between([0, lim], [0, lim * 0.85], [0, lim * 1.15], color=GRID, alpha=0.7, lw=0, zorder=1)
        ax.plot([0, lim], [0, lim], color=INK, lw=1, zorder=2)
        hh = np.arange(len(o)) >= (hrow // 3600 if hrow else 10 ** 9)
        ax.scatter(o[~hh], s[~hh], s=26, facecolors="none", edgecolors=BRICK, lw=1.3, zorder=3)
        ax.scatter(o[hh], s[hh], s=26, color=BRICK, zorder=3, edgecolors=PAPER, lw=0.6)
        ax.set_xlim(0, lim); ax.set_ylim(0, lim); ax.set_aspect("equal")
        gh = F.geh(s, o)
        ax.set_title(f"{d} {DNAME[d]}" + ("  (input)" if ROLE[d] == "in" else ""), fontsize=9)
        ax.text(0.97, 0.04, f"R² {F._r2(s, o):.2f}\nGEH<5: {100 * (gh < 5).mean():.0f}% of hours", transform=ax.transAxes,
                va="bottom", ha="right", fontsize=7.6, color=INK)
        ax.tick_params(labelsize=7)
    fig.text(0.5, -0.01, "observed vehicles per hour", ha="center")
    fig.text(-0.012, 0.5, "simulated vehicles per hour", va="center", rotation=90)
    h = [Line2D([], [], marker="o", ls="", mfc="none", mec=BRICK, mew=1.3, label="calibration hours (00-12)"),
         Line2D([], [], marker="o", ls="", color=BRICK, label="held-out hours (12-24)"),
         Line2D([], [], color=INK, lw=1, label="1:1"), plt.Rectangle((0, 0), 1, 1, color=GRID, label="±15 %")]
    fig.legend(handles=h, loc="upper center", ncol=4, bbox_to_anchor=(0.5, 1.02), fontsize=8.5)
    fig.suptitle("Hourly agreement, closed-loop actuated twin", x=0.01, y=1.08, ha="left", fontsize=11, fontweight="bold")
    fig.tight_layout()
    savefig(fig, out, "fig2_hourly_scatter.png")


def fig_timeseries(g, sims, hrow, out):
    """Hourly counts at the validation (output) detectors: observed vs closed-loop vs hybrid; night mode shaded."""
    outs = ["d3", "d6", "d8", "d9", "d10"]
    obs = hourly(g)
    fig, axes = plt.subplots(len(outs), 1, figsize=(9.6, 9.0), sharex=True)
    night = (g["sg1"].to_numpy() == 8)
    n0, n1 = np.where(night)[0][[0, -1]] / 3600 if night.any() else (None, None)
    for ax, d in zip(axes, outs):
        if n0 is not None:
            ax.axvspan(n0, n1, color=GRID, alpha=0.8, lw=0, zorder=0)
        h = np.arange(len(obs)) + 0.5
        ax.plot(h, obs[d], color=INK, lw=2.0, zorder=5)
        for m in ("hybrid", "actuated"):
            if m in sims:
                ax.plot(h, hourly(sims[m])[d], color=MODE_COL[m], lw=1.6, ls="-" if m == "actuated" else (0, (5, 2)), zorder=4)
        if hrow:
            ax.axvline(hrow / 3600, color=MULB, lw=1.2, ls=":", zorder=6)
        ymax = ax.get_ylim()[1]
        ax.set_ylim(0, ymax)
        ax.text(0.005, 0.93, f"{d}  {DNAME[d]}", transform=ax.transAxes, va="top", fontweight="bold", fontsize=9.5)
        ax.set_ylabel("veh / h", fontsize=8)
    axd = axes[3]                                   # direct labels in the d9 panel, right of the series ends
    axd.text(24.2, obs["d9"].iloc[-1], "observed", color=INK, va="center", ha="left", fontsize=8.5, fontweight="bold", clip_on=False)
    axd.text(24.2, hourly(sims["actuated"])["d9"].iloc[-1] - 6, "twin", color=BRICK, va="center", ha="left", fontsize=8.5, clip_on=False)
    axes[-1].set_xlim(0, 24); axes[-1].set_xticks(range(0, 25, 3)); axes[-1].set_xlabel("hour of day")
    if n0 is not None:
        axes[-1].text((n0 + n1) / 2, axes[-1].get_ylim()[1] * 0.55, "night mode\n(flashing amber)", ha="center", va="center", fontsize=7.8, color=MUTED)
    if hrow:
        axes[-1].text(hrow / 3600 + 0.15, axes[-1].get_ylim()[1] * 0.95, "held-out →", color=MULB, fontsize=8, va="top")
    h = [Line2D([], [], color=INK, lw=2.0, label="observed (loop data)"),
         Line2D([], [], color=BRICK, lw=1.6, label=MODE_LAB["actuated"]),
         Line2D([], [], color=OCHRE, lw=1.6, ls=(0, (5, 2)), label=MODE_LAB["hybrid"])]
    fig.legend(handles=h, loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.0), fontsize=8.8)
    fig.suptitle("Validation detectors over the day (emergent outputs)", x=0.01, y=1.025, ha="left", fontsize=11, fontweight="bold")
    fig.tight_layout()
    savefig(fig, out, "fig3_validation_timeseries.png")


def fig_matrix(args, out):
    """Eight-metric table per detector, closed-loop run on the held-out half-day (values + verdict mark, never colour alone)."""
    d = Path(args.runs["actuated"]) / args.date
    f = pd.read_csv(d / "fidelity_holdout.csv", index_col=0)
    mets = [("geh_total", "GEH\n(daily)", "geh"), ("geh_pct_pass", "GEH<5\nhours %", None), ("nrmse", "nRMSE\n%", "nrmse"),
            ("nmae", "nMAE\n%", "nmae"), ("mape", "MAPE\n%", "mape"), ("r2", "R²", "r2"), ("pbias", "PBIAS\n%", "pbias"),
            ("d", "Willmott\nd", "d"), ("u2", "Theil\nU2", "u2")]
    fig, ax = plt.subplots(figsize=(10.2, 5.0))
    ax.set_xlim(0, len(mets) + 1.4); ax.set_ylim(len(DETS), -1.2); ax.axis("off"); ax.grid(False)
    tint = {"excellent": "#cfe3d1", "pass": "#efe1b4", "FAIL": "#e8c3bc", "n/a": PAPER}
    mark = {"excellent": "✓✓", "pass": "✓", "FAIL": "✗", "n/a": "–"}
    ax.text(0.0, -0.8, "detector", fontweight="bold", va="center")
    for j, (_, lab, _) in enumerate(mets):
        ax.text(1.8 + j, -0.8, lab, ha="center", va="center", fontsize=8.2, fontweight="bold")
    for i, dname in enumerate(DETS):
        ax.text(0.0, i, f"{dname}  {DNAME[dname]}" + ("*" if ROLE[dname] == "in" else ""), va="center", fontsize=8.8,
                color=MUTED if ROLE[dname] == "in" else INK, fontweight="normal" if ROLE[dname] == "in" else "bold")
        for j, (col, _, key) in enumerate(mets):
            v = float(f.loc[dname, col])
            if key is None:
                ver = "excellent" if v >= 100 else ("pass" if v >= 85 else "FAIL")
            else:
                ver = F.verdict(key, v)
            ax.add_patch(plt.Rectangle((1.3 + j, i - 0.42), 0.92, 0.84, color=tint[ver], lw=0))
            txt = "–" if np.isnan(v) else (f"{v:.2f}" if abs(v) < 10 else f"{v:.0f}")
            ax.text(1.76 + j, i - 0.07, txt, ha="center", va="center", fontsize=8.2)
            ax.text(1.76 + j, i + 0.27, mark[ver], ha="center", va="center", fontsize=6.8, color=MUTED)
    ax.text(0.0, len(DETS) + 0.15, "* injected input (not a validation target)    ✓✓ excellent   ✓ pass   ✗ fail   – undefined (zero variance / no events)\n"
            "thresholds: GEH<5 DfT; nRMSE<15, nMAE<10, MAPE<15 FHWA; R²>0.90 NCHRP 765; |PBIAS|<10 ASCE; d>0.90 Willmott; U2<1 Theil",
            fontsize=7.6, color=MUTED, va="top")
    ax.set_title("Eight-metric fidelity, closed-loop twin, held-out hours 12-24 (hourly bins)", loc="left")
    savefig(fig, out, "fig4_metric_matrix.png")


def ecdf(ax, a, color, lw, label=None, ls="-"):
    a = np.sort(np.asarray(a, float))
    ax.step(a, np.arange(1, len(a) + 1) / len(a), where="post", color=color, lw=lw, ls=ls, label=label)


def fig_signals(g, sims, hrow, out):
    """Signal-timing fidelity of the closed-loop controller: distributions of stage greens, cycle and tram greens (held-out)."""
    s = sims["actuated"]
    sl = slice(hrow or 0, None)
    gr, sr = g.iloc[sl].reset_index(drop=True), s.iloc[sl].reset_index(drop=True)
    m = gr["sg1"].to_numpy() != 8
    gr, sr = gr[m].reset_index(drop=True), sr[m].reset_index(drop=True)

    def stage(df, cols):
        x = (df[cols].sum(axis=1) > 0).to_numpy()
        a, b = SF.runs(x)
        return a, b
    rows = {}
    for nm, cols in (("EW vehicle green (sg1,4,5)", ["sg1", "sg4", "sg5"]), ("NS vehicle green (sg2,3,6)", ["sg2", "sg3", "sg6"])):
        ra, rb = stage(gr, cols); sa, sb = stage(sr, cols)
        rows[nm] = (rb - ra, sb - sa)
    ra, _ = stage(gr, ["sg1", "sg4", "sg5"]); sa, _ = stage(sr, ["sg1", "sg4", "sg5"])
    rows["cycle length (EW start to EW start)"] = (np.diff(ra), np.diff(sa))
    for nm, c in (("PT N→S green (sg12)", "sg12"), ("PT S→N green (sg11)", "sg11")):
        a, b = SF.runs(gr[c].to_numpy()); a2, b2 = SF.runs(sr[c].to_numpy())
        rows[nm] = (b - a, b2 - a2)
    for nm, c in (("pedestrians EW (sg10)", "sg10"),):
        a, b = SF.runs(gr[c].to_numpy()); a2, b2 = SF.runs(sr[c].to_numpy())
        rows[nm] = (b - a, b2 - a2)
    fig, axes = plt.subplots(2, 3, figsize=(11.4, 6.0))
    for ax, (nm, (r, sdur)) in zip(axes.ravel(), rows.items()):
        ecdf(ax, r, INK, 2.2); ecdf(ax, sdur, BRICK, 1.8)
        ks, w1 = SF.ks_w1(r, sdur)
        hi = np.percentile(np.concatenate([r, sdur]), 99.5)
        ax.set_xlim(0, hi * 1.05); ax.set_ylim(0, 1.02)
        ax.set_title(nm, fontsize=9.5)
        ax.text(0.03, 0.96, f"median {np.median(r):.0f} s obs / {np.median(sdur):.0f} s twin\nKS {ks:.2f}   W₁ {w1:.1f} s", transform=ax.transAxes,
                ha="left", va="top", fontsize=7.8)
        ax.set_xlabel("seconds", fontsize=8)
    axes[0, 0].set_ylabel("cumulative share"); axes[1, 0].set_ylabel("cumulative share")
    h = [Line2D([], [], color=INK, lw=2.2, label="observed signals"), Line2D([], [], color=BRICK, lw=1.8, label="closed-loop controller")]
    fig.legend(handles=h, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.02), fontsize=9)
    fig.suptitle("Signal-timing fidelity on the held-out half-day (controller parameters fitted on hours 00-12 only)", x=0.01, y=1.07,
                 ha="left", fontsize=11, fontweight="bold")
    fig.tight_layout()
    savefig(fig, out, "fig5_signal_timing.png")


def fig_tsp(args, g, out):
    """Public-transport priority and pedestrians: tram waiting at the stop line (data vs twin) and pedestrian waiting."""
    d = Path(args.runs["actuated"]) / args.date
    waits = pd.read_csv(d / "tram_waits_sim.csv")
    waits["kind"] = waits["kind"].str.replace("->", "→")
    stats = pd.read_csv(d / "controller_stats.csv")
    fig, axes = plt.subplots(1, 3, figsize=(11.4, 3.9))
    for ax, kind, sg, det in ((axes[0], "N→S (sg12)", "sg12", "d3"), (axes[1], "S→N (sg11)", "sg11", "d6")):
        sgv = g[sg].to_numpy()
        arr = SF.runs(g[det].to_numpy())[0]
        real = [next((k for k in range(0, 120) if a + k < len(sgv) and sgv[a + k] == 1), 120) for a in arr]
        sim = waits[waits["kind"] == kind]["wait_s"].to_numpy(float)
        ecdf(ax, real, INK, 2.2); ecdf(ax, sim, BRICK, 1.8)
        ax.set_xlim(0, 60); ax.set_ylim(0, 1.02)
        ax.set_title(f"tram {kind}: wait at the stop line", fontsize=9.5)
        ax.text(0.97, 0.06, f"no wait: {100 * np.mean(np.array(real) == 0):.0f}% obs / {100 * np.mean(sim == 0):.0f}% twin\n"
                            f"p90: {np.percentile(real, 90):.0f} s obs / {np.percentile(sim, 90):.0f} s twin", transform=ax.transAxes,
                ha="right", va="bottom", fontsize=7.8)
        ax.set_xlabel("seconds")
    axes[0].set_ylabel("cumulative share")
    pw = stats["ped_wait"].dropna().to_numpy()
    ax = axes[2]
    ax.hist(pw, bins=np.arange(0, 70, 3), color=MULB, edgecolor=PAPER, lw=0.8, zorder=3)
    ax.set_title("pedestrian wait (assumed press times)", fontsize=9.5)
    ax.set_xlabel("seconds from press to walk"); ax.set_ylabel("pedestrian calls")
    ax.text(0.97, 0.93, f"median {np.median(pw):.0f} s, p90 {np.percentile(pw, 90):.0f} s\n≥ 1 press per pedestrian green\n(no pedestrian data: model assumption)",
            transform=ax.transAxes, ha="right", va="top", fontsize=7.8)
    h = [Line2D([], [], color=INK, lw=2.2, label="observed"), Line2D([], [], color=BRICK, lw=1.8, label="twin")]
    fig.legend(handles=h, loc="upper center", ncol=2, bbox_to_anchor=(0.36, 1.03), fontsize=9)
    fig.suptitle("Transit signal priority and pedestrians", x=0.01, y=1.1, ha="left", fontsize=11, fontweight="bold")
    fig.tight_layout()
    savefig(fig, out, "fig6_tsp_pedestrians.png")


def fig_green_hours(g, sims, hrow, out):
    """Hourly green seconds per signal group, observed vs closed-loop controller (all 12 groups x 24 h)."""
    s = sims["actuated"]
    hb = np.arange(len(g)) // 3600
    fig, axes = plt.subplots(3, 4, figsize=(11.4, 7.0))
    for ax, k in zip(axes.ravel(), range(1, 13)):
        c = f"sg{k}"
        o = (g[c] == 1).groupby(hb).sum().to_numpy(float); sm = (s[c] == 1).groupby(hb).sum().to_numpy(float)
        day_h = ((g["sg1"] == 8).groupby(hb).mean().to_numpy() < 0.5)          # night-mode hours carry no green time
        hh = (np.arange(len(o)) >= (hrow // 3600 if hrow else 10 ** 9))[day_h]
        o, sm = o[day_h], sm[day_h]
        lo, hi = min(o.min(), sm.min()), max(o.max(), sm.max())
        pad = max((hi - lo) * 0.15, 5)
        lo, hi = lo - pad, hi + pad
        ax.plot([lo, hi], [lo, hi], color=INK, lw=1)
        ax.scatter(o[~hh], sm[~hh], s=20, facecolors="none", edgecolors=BRICK, lw=1.2, zorder=3)
        ax.scatter(o[hh], sm[hh], s=20, color=BRICK, zorder=3, edgecolors=PAPER, lw=0.5)
        ax.set_xlim(lo, hi); ax.set_ylim(lo, hi); ax.set_aspect("equal")
        ax.set_title(c, fontsize=9)
        pb = 100 * (sm.sum() - o.sum()) / max(o.sum(), 1)
        nm = 100 * np.mean(np.abs(sm - o)) / max(o.mean(), 1)
        ax.text(0.97, 0.04, f"bias {pb:+.1f}%\nnMAE {nm:.1f}%", transform=ax.transAxes, va="bottom", ha="right", fontsize=7.8)
        ax.tick_params(labelsize=7)
    fig.text(0.5, 0.0, "observed green seconds per hour", ha="center"); fig.text(0.0, 0.5, "controller green seconds per hour", va="center", rotation=90)
    h = [Line2D([], [], marker="o", ls="", mfc="none", mec=BRICK, mew=1.3, label="calibration hours"),
         Line2D([], [], marker="o", ls="", color=BRICK, label="held-out hours"), Line2D([], [], color=INK, lw=1, label="1:1")]
    fig.legend(handles=h, loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.02), fontsize=8.8)
    fig.suptitle("Hourly green time per signal group (night-mode hours excluded; daytime green time is nearly constant, so bias and nMAE are shown instead of R²)", x=0.01, y=1.06, ha="left", fontsize=11, fontweight="bold")
    fig.tight_layout()
    savefig(fig, out, "fig7_green_time_hourly.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--runs", nargs="+", required=True, help="mode=dir, e.g. actuated=out_actuated")
    ap.add_argument("--date", default="2019-02-04")
    ap.add_argument("--holdout-from", default=None)
    ap.add_argument("--out", default=str(HERE / "results" / "figures"))
    a = ap.parse_args()
    a.runs = dict(r.split("=", 1) for r in a.runs)
    Path(a.out).mkdir(parents=True, exist_ok=True)
    g, sims, hrow, date = load(a)
    fig_geh(g, sims, hrow, a.out); fig_scatter(g, sims, hrow, a.out); fig_timeseries(g, sims, hrow, a.out)
    fig_matrix(a, a.out)
    if "actuated" in sims:
        fig_signals(g, sims, hrow, a.out); fig_tsp(a, g, a.out); fig_green_hours(g, sims, hrow, a.out)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Manuscript figures for the multi-day study (IEEEtran column widths, vector PDF + PNG, white background).

  python plots_multiday.py --runs OUT --days DAYDIR --tables results/multiday --out ../paper/figures

Palette (validated with the dataviz skill's validate_palette.js; ochre is <3:1 on white so series are also direct-labelled):
brick #b03a2a, ochre #d9a22c, fern #2f8a48, mulberry #9c4f93; observed data in ink.
"""
import argparse, sys, json
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import data as D, fidelity as F, signal_fidelity as SF

INK, MUTED, GRID, WHITE = "#2a2723", "#6f675b", "#e3ddd0", "#ffffff"
BRICK, OCHRE, FERN, MULB = "#b03a2a", "#d9a22c", "#2f8a48", "#9c4f93"
DETS = D.DET
OUTS = ["d3", "d6", "d8", "d9", "d10"]
DNAME = {"d1": "PT 220 m", "d2": "PT 50 m", "d3": "PT 1 m", "d4": "N 18 m", "d5": "E 43 m", "d6": "S 2 m", "d7": "S 15 m",
         "d8": "S exit", "d9": "W exit L0", "d10": "W exit L1"}
W1, W2 = 3.5, 7.16

plt.rcParams.update({
    "figure.facecolor": WHITE, "axes.facecolor": WHITE, "savefig.facecolor": WHITE, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
    "text.color": INK, "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.5, "axes.axisbelow": True, "font.size": 7.5, "axes.titlesize": 8,
    "axes.titleweight": "bold", "axes.titlelocation": "left", "legend.frameon": False, "legend.fontsize": 7, "font.family": "DejaVu Sans",
    "pdf.fonttype": 42, "ps.fonttype": 42,
})


def save(fig, out, name):
    fig.savefig(Path(out) / f"{name}.pdf", bbox_inches="tight")
    fig.savefig(Path(out) / f"{name}.png", dpi=220, bbox_inches="tight")
    plt.close(fig)
    print("wrote", name)


def is_weekend(d):
    return pd.Timestamp(d).dayofweek >= 5


# ------------------------------------------------------------------------------------------------------------------
def fig_arch(out):
    fig, ax = plt.subplots(figsize=(W2, 3.1)); ax.axis("off"); ax.set_xlim(0, 100); ax.set_ylim(0, 49); ax.grid(False)

    def box(x, y, w, h, txt, fc, ec=INK, fs=6.6):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.25,rounding_size=1.0", fc=fc, ec=ec, lw=0.8))
        ax.text(x + w / 2, y + h / 2, txt, ha="center", va="center", fontsize=fs, color=INK)

    def arr(x0, y0, x1, y1, c=INK, ls="-"):
        ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>", mutation_scale=7, lw=0.9, color=c, linestyle=ls))
    ax.text(0.5, 48.5, "Problem 1: a replayable and actuated multimodal twin", fontsize=8, fontweight="bold", va="top")
    ax.text(0.5, 45.0, "Problem 2 (future work): a learning controller replaces the stage logic", fontsize=6.8, color=MUTED, style="italic", va="top")
    box(0.5, 17, 17, 10, "59-day 1 Hz record\nd1-d10, sg1-sg12\n(Genser et al.)", "#f3efe6")
    box(22, 31, 24, 7, "EXACT REPLAY\nrecorded signals + virtual\nvehicles on every loop", "#e4f0e6", FERN)
    box(22, 17, 24, 10, "INJECTED ARRIVALS\nd1, d2, d4, d5, d7\n(real vehicles, exogenous)", "#f6ecd2", OCHRE)
    box(22, 1, 24, 10, "CALIBRATION\ninputs + controller\n30 January days", "#efe4f0", MULB)
    box(51, 17, 22, 10, "SUMO NETWORK\n12 link indices, loops,\ntrams, crossings", "#f3efe6")
    box(51, 1, 22, 10, "ACTUATED CONTROLLER\n+ TSP (sg11, sg12)\nstep(t, obs) -> greens", "#f5dcd6", BRICK)
    box(77, 17, 22, 10, "OUTPUTS\nd3, d6, d8, d9, d10\n+ signal timing", "#f3efe6")
    box(77, 31, 22, 7, "VALIDATION\n28 held-out February days", "#f3efe6")
    arr(17.5, 24.5, 22, 33.5, FERN); arr(17.5, 22, 22, 22, OCHRE); arr(46, 34.5, 51.5, 26, FERN); arr(46, 22, 51, 22, OCHRE)
    arr(34, 11, 34, 17, MULB, "--"); arr(46, 6, 51, 6, MULB, "--")
    arr(62, 11, 62, 17, BRICK); arr(73, 22, 77, 22); arr(88, 27, 88, 31)
    save(fig, out, "fig1_architecture")


def fig_exact(tables, out):
    d = pd.read_csv(Path(tables) / "exact_report_by_day.csv")
    piv = d.pivot(index="date", columns="detector", values="jaccard")[DETS]
    dates = list(piv.index)
    fig, axs = plt.subplots(1, 2, figsize=(W2, 3.2), gridspec_kw={"width_ratios": [3.2, 1.6]})
    ax = axs[0]
    im = ax.imshow(piv.to_numpy().T, aspect="auto", cmap=matplotlib.colors.LinearSegmentedColormap.from_list("j", ["#b03a2a", "#f4e3d5", "#2f8a48"]),
                   vmin=0.97, vmax=1.0)
    ax.set_yticks(range(len(DETS))); ax.set_yticklabels([f"{x} {DNAME[x]}" for x in DETS], fontsize=6.5)
    tick = [i for i, x in enumerate(dates) if x.endswith("-01") or x.endswith("-15")]
    ax.set_xticks(tick); ax.set_xticklabels([dates[i][5:] for i in tick], fontsize=6.5); ax.grid(False)
    for i, x in enumerate(dates):
        if is_weekend(x):
            ax.plot([i - .5, i + .5], [len(DETS) - .35] * 2, color=MULB, lw=2.2, clip_on=False)
    cb = fig.colorbar(im, ax=ax, fraction=0.025, pad=0.01); cb.set_label("per-second Jaccard", fontsize=7); cb.ax.tick_params(labelsize=6.5)
    ax.set_title("(a) Exact replay: per-second Jaccard, 59 days x 10 detectors (purple ticks = weekends)")
    ax = axs[1]
    m = d.groupby("detector")["second_accuracy"].min().reindex(DETS) * 100
    ax.barh(range(len(DETS)), m, color=FERN, height=0.6)
    ax.set_yticks(range(len(DETS))); ax.set_yticklabels([]); ax.invert_yaxis(); ax.set_xlim(99.0, 100.01)
    for i, v in enumerate(m):
        ax.text(v - 0.01, i, f"{v:.3f}", ha="right", va="center", fontsize=6, color=WHITE)
    ax.set_xlabel("worst-day second accuracy (%)"); ax.set_title("(b) worst day"); ax.grid(axis="y", visible=False)
    save(fig, out, "fig2_exact_replay")


def fig_daily(tables, out):
    d = pd.read_csv(Path(tables) / "detector_by_day.csv")
    d = d[(d.label == "actuated_tsp") & d.detector.isin(OUTS)].copy()
    d["date"] = pd.to_datetime(d["date"])
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.7))
    cols = {"d3": INK, "d6": MULB, "d8": OCHRE, "d9": BRICK, "d10": FERN}
    for ax, col, ttl, ylab in ((axs[0], "geh_total", "(a) Daily GEH of the emergent detectors", "daily GEH"),
                               (axs[1], "geh_pct_pass", "(b) Hours with GEH < 5", "% of hours")):
        for det in OUTS:
            s = d[d.detector == det].sort_values("date")
            ax.plot(s["date"], s[col].astype(float), color=cols[det], lw=1.0, marker="o", ms=2.2, label=f"{det} {DNAME[det]}")
        for dt in sorted(d["date"].unique()):
            if pd.Timestamp(dt).dayofweek >= 5:
                ax.axvspan(dt - pd.Timedelta(hours=12), dt + pd.Timedelta(hours=12), color=GRID, alpha=0.7, lw=0)
        ax.set_title(ttl); ax.set_ylabel(ylab)
        ax.xaxis.set_major_formatter(matplotlib.dates.DateFormatter("%d %b")); ax.tick_params(axis="x", labelsize=6.5)
    axs[0].axhline(5, color=INK, ls=(0, (4, 3)), lw=0.9); axs[0].text(d["date"].min(), 5.15, "GEH = 5", fontsize=6.5)
    axs[1].axhline(85, color=INK, ls=(0, (4, 3)), lw=0.9); axs[1].text(d["date"].min(), 83, "85 %", fontsize=6.5, va="top")
    axs[1].set_ylim(40, 101)
    axs[0].legend(ncol=2, loc="upper right", fontsize=6.3)
    fig.text(0.5, -0.03, "shaded = weekends; closed loop, controller calibrated on January only", ha="center", fontsize=6.5, color=MUTED)
    save(fig, out, "fig3_daily_closed_loop")


def fig_matrix(tables, out):
    p = pd.read_csv(Path(tables) / "detector_pooled.csv")
    pdd = pd.read_csv(Path(tables) / "detector_by_day.csv")
    pdd = pdd[pdd.label == "actuated_tsp"].copy(); pdd["geh_total"] = pdd["geh_total"].astype(float)
    for sub in ("weekday", "weekend"):                       # per-day GEH (mean over days), not a pooled total
        m = pdd[pdd.daytype == sub].groupby("detector")["geh_total"].mean()
        idx = (p.label == "actuated_tsp") & (p.subset == sub)
        p.loc[idx, "geh_total"] = p.loc[idx, "detector"].map(m)
    mets = [("geh_total", "mean daily\nGEH", "geh"), ("geh_pct_pass", "GEH<5\nhours %", None), ("nrmse", "nRMSE\n%", "nrmse"), ("nmae", "nMAE\n%", "nmae"),
            ("mape", "MAPE\n%", "mape"), ("r2", "R²", "r2"), ("pbias", "PBIAS\n%", "pbias"), ("d", "Willmott\nd", "d"), ("u2", "Theil\nU2", "u2")]
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.9))
    tint = {"excellent": "#cfe3d1", "pass": "#f1e3b8", "FAIL": "#ebc6bf", "n/a": WHITE}
    mark = {"excellent": "✓✓", "pass": "✓", "FAIL": "✗", "n/a": "–"}
    for ax, sub, ttl in ((axs[0], "weekday", "(a) Weekdays (20 test days)"), (axs[1], "weekend", "(b) Weekends (8 test days)")):
        f = p[(p.label == "actuated_tsp") & (p.subset == sub)].set_index("detector")
        ax.set_xlim(0, len(mets) + 1.6); ax.set_ylim(len(OUTS), -1.4); ax.axis("off"); ax.grid(False)
        for j, (_, lab, _) in enumerate(mets):
            ax.text(1.9 + j, -0.85, lab, ha="center", va="center", fontsize=5.8, fontweight="bold")
        for i, det in enumerate(OUTS):
            ax.text(0, i, f"{det} {DNAME[det]}", va="center", fontsize=6.5, fontweight="bold")
            for j, (col, _, key) in enumerate(mets):
                v = float(f.loc[det, col])
                ver = ("excellent" if v >= 100 else "pass" if v >= 85 else "FAIL") if key is None else F.verdict(key, v)
                ax.add_patch(plt.Rectangle((1.4 + j, i - 0.42), 0.92, 0.84, color=tint[ver], lw=0))
                ax.text(1.86 + j, i - 0.08, "–" if np.isnan(v) else (f"{v:.2f}" if abs(v) < 10 else f"{v:.0f}"), ha="center", va="center", fontsize=6.2)
                ax.text(1.86 + j, i + 0.28, mark[ver], ha="center", va="center", fontsize=5.2, color=MUTED)
        ax.set_title(ttl)
    fig.text(0.5, -0.02, "pooled hourly bins over the held-out days; ✓✓ excellent, ✓ pass, ✗ fail (thresholds: GEH<5, nRMSE<15, nMAE<10, MAPE<15, R²>0.90, |PBIAS|<10, d>0.90, U2<1)",
             ha="center", fontsize=5.8, color=MUTED)
    save(fig, out, "fig4_metric_matrix")


def durations(df, cols):
    x = (df[cols].sum(axis=1) > 0).to_numpy()
    a, b = SF.runs(x.astype(int))
    return a, b


def fig_signals(args, out):
    root, ddir = Path(args.runs), Path(args.days)
    labs = ("actuated_tsp",)
    acc = {k: ([], []) for k in ("EW green (sg1,4,5)", "NS green (sg2,3,6)", "cycle length", "PT N→S green (sg12)", "PT S→N green (sg11)", "ped. EW green (sg10)")}
    dts = sorted(p.name for p in (root / labs[0]).iterdir() if (p / p.name / "sim_detectors.csv").exists())
    for dt in dts:
        _, g, _ = next(D.days(D.load([str(ddir / f"{dt}.csv")])))
        g = g.reset_index(drop=True)
        s = pd.read_csv(root / labs[0] / dt / dt / "sim_detectors.csv")
        m = (g["sg1"] != 8).to_numpy()[: len(s)]
        gr, sr = g.iloc[: len(s)][m].reset_index(drop=True), s[m].reset_index(drop=True)
        for key, cols in (("EW green (sg1,4,5)", ["sg1", "sg4", "sg5"]), ("NS green (sg2,3,6)", ["sg2", "sg3", "sg6"])):
            ra, rb = durations(gr, cols); sa, sb = durations(sr, cols)
            acc[key][0].extend(rb - ra); acc[key][1].extend(sb - sa)
        ra, _ = durations(gr, ["sg1", "sg4", "sg5"]); sa, _ = durations(sr, ["sg1", "sg4", "sg5"])
        acc["cycle length"][0].extend(np.diff(ra)); acc["cycle length"][1].extend(np.diff(sa))
        for key, c in (("PT N→S green (sg12)", ["sg12"]), ("PT S→N green (sg11)", ["sg11"]), ("ped. EW green (sg10)", ["sg10"])):
            ra, rb = durations(gr, c); sa, sb = durations(sr, c)
            acc[key][0].extend(rb - ra); acc[key][1].extend(sb - sa)
    fig, axs = plt.subplots(2, 3, figsize=(W2, 3.9))
    stats = []
    for ax, (k, (r, sm)) in zip(axs.ravel(), acc.items()):
        r, sm = np.sort(np.array(r, float)), np.sort(np.array(sm, float))
        ax.step(r, np.arange(1, len(r) + 1) / len(r), where="post", color=INK, lw=1.6, label="observed")
        ax.step(sm, np.arange(1, len(sm) + 1) / len(sm), where="post", color=BRICK, lw=1.2, label="twin")
        hi = np.percentile(np.concatenate([r, sm]), 99.3); ax.set_xlim(0, hi * 1.05); ax.set_ylim(0, 1.02)
        ks, w1 = SF.ks_w1(r, sm); stats.append((k, np.median(r), np.median(sm), ks, w1, len(r), len(sm)))
        ax.set_title(k, fontsize=7.5)
        ax.text(0.97, 0.05, f"median {np.median(r):.0f} / {np.median(sm):.0f} s\nKS {ks:.2f}, W₁ {w1:.1f} s", transform=ax.transAxes, ha="right", va="bottom", fontsize=6.3)
        ax.set_xlabel("seconds", fontsize=6.8)
    axs[0, 0].set_ylabel("cumulative share"); axs[1, 0].set_ylabel("cumulative share"); axs[0, 0].legend(loc="upper left", fontsize=6.5)
    pd.DataFrame(stats, columns=["quantity", "median_obs", "median_twin", "ks", "w1_s", "n_obs", "n_twin"]).to_csv(Path(args.tables) / "signal_timing_pooled.csv", index=False)
    save(fig, out, "fig5_signal_timing")


def fig_tram(args, out):
    root, ddir = Path(args.runs), Path(args.days)
    lab = "actuated_tsp"
    dts = sorted(p.name for p in (root / lab).iterdir() if (p / p.name / "tram_waits_sim.csv").exists())
    real = {"N→S (sg12)": [], "S→N (sg11)": []}; sim = {"N→S (sg12)": [], "S→N (sg11)": []}
    for dt in dts:
        _, g, _ = next(D.days(D.load([str(ddir / f"{dt}.csv")])))
        g = g.reset_index(drop=True)
        for k, sg, det in (("N→S (sg12)", "sg12", "d3"), ("S→N (sg11)", "sg11", "d6")):
            sgv = g[sg].to_numpy()
            for a in SF.runs(g[det].to_numpy())[0]:
                real[k].append(next((j for j in range(0, 120) if a + j < len(sgv) and sgv[a + j] == 1), 120))
        tw = pd.read_csv(root / lab / dt / dt / "tram_waits_sim.csv"); tw["kind"] = tw["kind"].str.replace("->", "→")
        for k in sim:
            sim[k].extend(tw[tw.kind == k]["wait_s"].tolist())
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.3))
    rows = []
    for ax, k in zip(axs, real):
        r, s = np.sort(real[k]), np.sort(sim[k])
        ax.step(r, np.arange(1, len(r) + 1) / len(r), where="post", color=INK, lw=1.6, label="observed")
        ax.step(s, np.arange(1, len(s) + 1) / len(s), where="post", color=BRICK, lw=1.2, label="twin")
        ax.set_xlim(0, 60); ax.set_ylim(0.4, 1.01); ax.set_xlabel("wait at the stop line (s)"); ax.set_title(f"tram {k}")
        ax.text(0.97, 0.06, f"no wait {100 * np.mean(r == 0):.0f} % obs / {100 * np.mean(s == 0):.0f} % twin\np90 {np.percentile(r, 90):.0f} / {np.percentile(s, 90):.0f} s",
                transform=ax.transAxes, ha="right", va="bottom", fontsize=6.5)
        rows.append((k, len(r), len(s), 100 * np.mean(r == 0), 100 * np.mean(s == 0), np.percentile(r, 90), np.percentile(s, 90)))
    axs[0].set_ylabel("cumulative share"); axs[0].legend(loc="center right", fontsize=6.5)
    pd.DataFrame(rows, columns=["tram", "n_obs", "n_twin", "nowait_obs_pct", "nowait_twin_pct", "p90_obs_s", "p90_twin_s"]).to_csv(Path(args.tables) / "tram_wait_pooled.csv", index=False)
    save(fig, out, "fig6_tram_priority")


def fig_hourly(args, out):
    root, ddir = Path(args.runs), Path(args.days)
    lab = "actuated_tsp"
    dts = sorted(p.name for p in (root / lab).iterdir() if (p / p.name / "sim_detectors.csv").exists())
    obs = {(t, w): [] for t in OUTS for w in (False, True)}; sim = {(t, w): [] for t in OUTS for w in (False, True)}
    for dt in dts:
        _, g, _ = next(D.days(D.load([str(ddir / f"{dt}.csv")])))
        s = pd.read_csv(root / lab / dt / dt / "sim_detectors.csv"); s.index = g.index[: len(s)]
        if len(s) < 86000:
            continue
        o, sm = F.period_counts(g, DETS, 3600), F.period_counts(s, DETS, 3600)
        for t in OUTS:
            obs[(t, is_weekend(dt))].append(o[t].reindex(range(24), fill_value=0).to_numpy()); sim[(t, is_weekend(dt))].append(sm[t].reindex(range(24), fill_value=0).to_numpy())
    fig, axs = plt.subplots(2, 5, figsize=(W2, 3.4), sharex=True)
    for r, w in enumerate((False, True)):
        for c, t in enumerate(OUTS):
            ax = axs[r, c]; O, S = np.array(obs[(t, w)]), np.array(sim[(t, w)])
            h = np.arange(24) + 0.5
            ax.fill_between(h, np.percentile(O, 25, axis=0), np.percentile(O, 75, axis=0), color=GRID, lw=0)
            ax.plot(h, O.mean(axis=0), color=INK, lw=1.5); ax.plot(h, S.mean(axis=0), color=BRICK, lw=1.1)
            ax.set_title(f"{t} {DNAME[t]}" if r == 0 else "", fontsize=7)
            if c == 0:
                ax.set_ylabel(("weekdays" if not w else "weekends") + "\nveh / h", fontsize=7)
            ax.set_xlim(0, 24); ax.set_xticks([0, 6, 12, 18, 24]); ax.set_ylim(bottom=0)
    for ax in axs[1]:
        ax.set_xlabel("hour of day", fontsize=6.8)
    h = [Line2D([], [], color=INK, lw=1.5, label="observed mean (band = IQR over days)"), Line2D([], [], color=BRICK, lw=1.1, label="closed-loop twin mean")]
    fig.legend(handles=h, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.04), fontsize=7)
    save(fig, out, "fig7_hourly_profiles")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", required=True); ap.add_argument("--days", required=True); ap.add_argument("--tables", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--only", nargs="*", default=None)
    a = ap.parse_args()
    Path(a.out).mkdir(parents=True, exist_ok=True)
    todo = {"arch": lambda: fig_arch(a.out), "exact": lambda: fig_exact(a.tables, a.out), "daily": lambda: fig_daily(a.tables, a.out),
            "matrix": lambda: fig_matrix(a.tables, a.out), "signals": lambda: fig_signals(a, a.out), "tram": lambda: fig_tram(a, a.out),
            "hourly": lambda: fig_hourly(a, a.out)}
    for k, f in todo.items():
        if a.only is None or k in a.only:
            f()


if __name__ == "__main__":
    main()

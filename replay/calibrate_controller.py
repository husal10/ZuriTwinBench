#!/usr/bin/env python3
"""Calibrate the actuated controller from the field data (train window) and test it offline (held-out window).

Offline test = the controller is driven by the *recorded* detector pulses (d4, d5, d3, d6) and by tram calls placed a
sampled advance time before the recorded tram arrivals; its signal output is compared with the recorded signals.
Parameters that are statistics of the data (offsets, clearances, tram margins) are medians on the train window;
the two gap-out times are fitted by grid search on the train window (Wasserstein distance of the stage durations).

  python calibrate_controller.py --csv ../data/feb04_only.csv --train-until "2019-02-04 12:00:00" --out controller_params.json
"""
import argparse, itertools, json, sys
from pathlib import Path
import numpy as np, pandas as pd

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import data as D, controller as K, signal_fidelity as SF

SG = D.SG


def union_runs(df, groups):
    x = (df[groups].sum(axis=1) > 0).to_numpy()
    return SF.runs(x)


def derive(df):
    """Data-derived parameters (medians) from a day-mode frame (rows consecutive)."""
    P = K.Params()
    st = {}
    for ax in ("EW", "NS"):
        s, e = union_runs(df, K.VEH[ax]); st[ax] = (s, e)
        dur = e - s
        pct = lambda q: int(np.percentile(dur, q))
        so = {g: [] for g in K.VEH[ax]}; ee = {g: [] for g in K.VEH[ax]}
        for g in K.VEH[ax]:
            gs, ge = SF.runs(df[g].to_numpy())
            for a, b in zip(s, e):
                k = np.where((gs >= a - 1) & (gs <= b))[0]
                if len(k):
                    so[g].append(gs[k[0]] - a); ee[g].append(b - ge[k[0]])
        for g in K.VEH[ax]:
            P.start_off[g], P.end_early[g] = int(np.median(so[g])), int(np.median(ee[g]))
        Dm = max(P.end_early[g] for g in K.VEH[ax])          # the stage decision precedes the union end by this much
        P.min_green_pri[ax], P.min_green[ax], P.max_green[ax] = max(1, pct(5) - Dm), max(1, pct(25) - Dm), max(2, pct(97) - Dm)
        lead, tail = [], []
        for g in K.PED[ax]:
            gs, ge = SF.runs(df[g].to_numpy())
            for a, b in zip(s, e):
                k = np.where((gs <= a) & (ge >= a))[0]
                if len(k):
                    lead.append(a - gs[k[0]]); tail.append(ge[k[0]] - b)
        P.ped_lead[ax], P.ped_tail[ax] = int(np.median(lead)), int(np.median(tail))
    for a, b in (("EW", "NS"), ("NS", "EW")):
        sa, ea = st[a]; sb, eb = st[b]
        P.gap_to_next[a] = int(np.median([sb[sb >= x][0] - x for x in ea if (sb >= x).any()]))
    vend = np.sort(np.concatenate([st["EW"][1], st["NS"][1]])); vstart = np.sort(np.concatenate([st["EW"][0], st["NS"][0]]))
    s12, e12 = SF.runs(df["sg12"].to_numpy()); s11, e11 = SF.runs(df["sg11"].to_numpy())
    d3s, d3e = SF.runs(df["d3"].to_numpy()); d6s, d6e = SF.runs(df["d6"].to_numpy())
    near = lambda a, arr: arr[np.abs(arr - a).argmin()]
    P.t12_delay = int(np.median([a - vend[vend <= a][-1] for a in s12 if (vend <= a).any()]))
    P.t12_to_next = int(np.median([vstart[vstart >= b][0] - b for b in e12 if (vstart >= b).any()]))
    P.t12_margin = int(np.median([b - near(b, d3e) for b in e12]))
    P.t12_min, P.t12_max = int(np.percentile(e12 - s12, 10)), int(np.percentile(e12 - s12, 97))
    ns_s = st["NS"][0]
    P.t11_offset = int(np.median([a - near(a, ns_s) for a in s11]))
    P.t11_margin = int(np.median([b - near(b, d6e) for b in e11]))
    P.t11_min, P.t11_max = int(np.percentile(e11 - s11, 10)), int(np.percentile(e11 - s11, 97))
    leads = np.array([max(0, a - s12[np.abs(s12 - a).argmin()]) for a in d3s])      # d3 rise - sg12 start (advance notice)
    leads_s = np.array([max(0, a - s11[np.abs(s11 - a).argmin()]) for a in d6s])  # d6 rise - sg11 start
    return P, np.clip(leads, 0, 40), np.clip(leads_s, 0, 40)


def offline(df, P, leads, leads_s=None, seed=1, night=None, ped_rate=1 / 50.0):
    leads_s = leads if leads_s is None else leads_s
    """Drive the controller with recorded detector pulses; returns a DataFrame of green codes."""
    rng = np.random.default_rng(seed)
    n = len(df)
    ob = {k: (df[k].to_numpy() > 0) for k in ("d3", "d4", "d5", "d6")}
    d3r = SF.runs(df["d3"].to_numpy())[0]; d6r = SF.runs(df["d6"].to_numpy())[0]
    calls_n = {int(max(0, a - rng.choice(leads) - P.tram_call_extra)) for a in d3r}
    calls_s = {int(max(0, a - rng.choice(leads_s) - P.tram_s_call_extra)) for a in d6r}
    night = night if night is not None else (df["sg1"].to_numpy() == 8)
    c = K.ActuatedController(P, "NS", 0)
    out = np.zeros((n, 12), dtype=np.int8)
    was_night = False
    for t in range(n):
        if night[t]:
            out[t, :] = 8; was_night = True
            continue
        if was_night:
            c.reset(t, "EW"); was_night = False
        if t in calls_n: c.call_tram_n(t)
        if t in calls_s: c.call_tram_s(t)
        for ax in ("EW", "NS"):
            if rng.random() < ped_rate: c.ped_press(ax, t)
        g = c.step(t, {k: bool(v[t]) for k, v in ob.items()})
        out[t, :] = [g[f"sg{i}"] for i in range(1, 13)]
    return pd.DataFrame(out, columns=SG, index=df.index), c


def score(real, sim):
    out = 0.0
    for ax in ("EW", "NS"):
        rs, re = union_runs(real, K.VEH[ax]); ss, se = union_runs(sim, K.VEH[ax])
        out += SF.ks_w1(re - rs, se - ss)[1]
    return out


def main_single():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", nargs="+", required=True)
    ap.add_argument("--train-until", default=None, help="end of the training window (default: first half of the first day)")
    ap.add_argument("--out", default=str(HERE / "controller_params.json"))
    ap.add_argument("--report", default=str(HERE / "results" / "controller_offline"))
    a = ap.parse_args()
    df = D.load(a.csv)
    date, g, _ = next(D.days(df)); g = g.reset_index(drop=True)
    cut = int((pd.Timestamp(a.train_until) - date).total_seconds()) if a.train_until else len(g) // 2
    day = g["sg1"] != 8
    train, test = g.iloc[:cut], g.iloc[cut:]
    P, leads, leads_s = derive(train[train["sg1"] != 8])
    json.dump({'n': leads.tolist(), 's': leads_s.tolist()}, open(str(HERE / 'tram_leads.json'), 'w'))
    m_tr = (train["sg1"] != 8).to_numpy()
    real_tr = train[m_tr].reset_index(drop=True)

    def stage_score(ax):
        sim, _ = offline(train, P, leads, leads_s)
        rs, re = union_runs(real_tr, K.VEH[ax]); ss, se = union_runs(sim[m_tr].reset_index(drop=True), K.VEH[ax])
        return SF.ks_w1(re - rs, se - ss)[1]
    for ax, gaps, maxes in (("EW", [2, 3, 4, 6, 8, 12], [10, 11, 12, 13, 14, 16]), ("NS", [1, 2, 3, 4, 6, 8, 12], [11, 12, 14, 16, 20])):
        bb = None
        for gp, mx in itertools.product(gaps, maxes):
            P.gap[ax], P.max_green[ax] = float(gp), mx
            sc = stage_score(ax)
            if bb is None or sc < bb[0]:
                bb = (sc, gp, mx)
        P.gap[ax], P.max_green[ax] = float(bb[1]), bb[2]
        print(f"fitted {ax}: gap {bb[1]} s, max green {bb[2]} s, train W1 {bb[0]:.2f} s")
    best = (0, P.gap["EW"], P.gap["NS"])
    m = (train["sg1"] != 8).to_numpy()
    d3r_tr = SF.runs(train["d3"].to_numpy())[0]
    real_on = SF.tram_metrics(train[SG], train[SG], d3r_tr, SF.runs(train["d6"].to_numpy())[0]).loc["sg12_green_on_arrival_pct", "real"]
    bx = None
    for extra in range(0, 41, 2):
        P.tram_call_extra = extra
        sim, _ = offline(train, P, leads, leads_s)
        got = SF.tram_metrics(train[SG], sim, d3r_tr, SF.runs(train["d6"].to_numpy())[0]).loc["sg12_green_on_arrival_pct", "sim"]
        if bx is None or abs(got - real_on) < bx[0]:
            bx = (abs(got - real_on), extra)
    P.tram_call_extra = bx[1]
    print("fitted tram_call_extra", P.tram_call_extra, "s (green on arrival target", round(real_on, 1), "%)")
    d6r_tr = SF.runs(train["d6"].to_numpy())[0]
    real_on6 = SF.tram_metrics(train[SG], train[SG], d3r_tr, d6r_tr).loc["sg11_green_on_arrival_pct", "real"]
    bx = None
    for extra in range(0, 41, 2):
        P.tram_s_call_extra = extra
        sim, _ = offline(train, P, leads, leads_s)
        got = SF.tram_metrics(train[SG], sim, d3r_tr, d6r_tr).loc["sg11_green_on_arrival_pct", "sim"]
        if bx is None or abs(got - real_on6) < bx[0]:
            bx = (abs(got - real_on6), extra)
    P.tram_s_call_extra = bx[1]
    print("fitted tram_s_call_extra", P.tram_s_call_extra, "s (target", round(real_on6, 1), "%)")
    P.save(a.out)
    print("fitted gaps", P.gap, "train score", round(best[0], 3)); print(json.dumps(json.load(open(a.out)), indent=None))
    Path(a.report).mkdir(parents=True, exist_ok=True)
    for name, w in (("train", train), ("test", test)):
        w = w.reset_index(drop=True)
        sim, c = offline(w, P, leads, leads_s)
        m = (w["sg1"] != 8).to_numpy()
        real_d, sim_d = w[SG][m].reset_index(drop=True), sim[m].reset_index(drop=True)
        pg = SF.per_group(real_d, sim_d)
        cyc, ks = SF.cycle_metrics(real_d, sim_d)
        d3r, d6r = SF.runs(w["d3"].to_numpy())[0], SF.runs(w["d6"].to_numpy())[0]
        tm = SF.tram_metrics(w[SG], sim, d3r, d6r)
        print(f"\n=== offline controller vs data, {name} window ({len(w)//3600} h) ===")
        print(pg[["starts_real", "starts_sim", "dur_med_real", "dur_med_sim", "dur_ks", "dur_w1_s", "r2_green_s_hourly"]].round(3).to_string())
        print(cyc.round(2).to_string()); print("KS/W1 (cycle, ew green, ns green):", {k: tuple(round(x, 3) for x in v) for k, v in ks.items()})
        print(tm.round(2).to_string())
        pg.to_csv(f"{a.report}/{name}_per_group.csv"); cyc.to_csv(f"{a.report}/{name}_cycle.csv"); tm.to_csv(f"{a.report}/{name}_tram.csv")
        print("ped wait (s) median/p90:", np.median(c.stats["ped_wait"]).round(1), np.percentile(c.stats["ped_wait"], 90).round(1))



# ---------------------------------------------------------------------------------------------------------------------
def load_day(p):
    date, g, _ = next(D.days(D.load([p])))
    return date, g.reset_index(drop=True)


def main_multi(a):
    """Train on whole days (a.train_days), fit gaps/max greens/tram call advances on a sample of them, test on other days."""
    import glob
    tr_files = sorted(f for pat in a.train_days for f in glob.glob(pat))
    te_files = sorted(f for pat in (a.test_days or []) for f in glob.glob(pat))
    tr = [load_day(f) for f in tr_files]
    P, leads, leads_s = derive(pd.concat([g[g["sg1"] != 8] for _, g in tr], ignore_index=True))
    json.dump({"n": leads.tolist(), "s": leads_s.tolist()}, open(str(HERE / "tram_leads.json"), "w"))
    sel = [tr[i][1] for i in np.linspace(0, len(tr) - 1, min(a.fit_days, len(tr))).astype(int)]
    masks = [(g["sg1"] != 8).to_numpy() for g in sel]
    reals = [g[m].reset_index(drop=True) for g, m in zip(sel, masks)]

    def stage_score(ax):
        s = 0.0
        for g, m, r in zip(sel, masks, reals):
            sim, _ = offline(g, P, leads, leads_s)
            rs, re = union_runs(r, K.VEH[ax]); ss, se = union_runs(sim[m].reset_index(drop=True), K.VEH[ax])
            s += SF.ks_w1(re - rs, se - ss)[1]
        return s / len(sel)
    for ax, gaps, maxes in (("EW", [2, 3, 4, 6, 8, 12], [10, 11, 12, 13, 14, 16]), ("NS", [1, 2, 3, 4, 6, 8, 12], [11, 12, 14, 16, 20])):
        bb = None
        for gp, mx in itertools.product(gaps, maxes):
            P.gap[ax], P.max_green[ax] = float(gp), mx
            sc = stage_score(ax)
            if bb is None or sc < bb[0]:
                bb = (sc, gp, mx)
        P.gap[ax], P.max_green[ax] = float(bb[1]), bb[2]
        print(f"fitted {ax}: gap {bb[1]} s, max green {bb[2]} s, train W1 {bb[0]:.2f} s", flush=True)

    def green_on_arrival(which, extra):
        out_r, out_s = [], []
        for g in sel:
            if which == "n":
                P.tram_call_extra = extra
            else:
                P.tram_s_call_extra = extra
            sim, _ = offline(g, P, leads, leads_s)
            tm = SF.tram_metrics(g[SG], sim, SF.runs(g["d3"].to_numpy())[0], SF.runs(g["d6"].to_numpy())[0])
            k = "sg12_green_on_arrival_pct" if which == "n" else "sg11_green_on_arrival_pct"
            out_r.append(tm.loc[k, "real"]); out_s.append(tm.loc[k, "sim"])
        return np.mean(out_r), np.mean(out_s)
    for which, name in (("n", "tram_call_extra"), ("s", "tram_s_call_extra")):
        bx = None
        for extra in range(0, 41, 2):
            r, s = green_on_arrival(which, extra)
            if bx is None or abs(s - r) < bx[0]:
                bx = (abs(s - r), extra)
        setattr(P, name, bx[1])
        print("fitted", name, bx[1], "s", flush=True)
    P.save(a.out)
    print(json.dumps(json.load(open(a.out))))
    rows = []
    for split, files in (("train", tr_files), ("test", te_files)):
        for f in files:
            date, g = load_day(f)
            sim, c = offline(g, P, leads, leads_s)
            m = (g["sg1"] != 8).to_numpy()
            rs_, ss_ = g[SG][m].reset_index(drop=True), sim[m].reset_index(drop=True)
            cyc, ks = SF.cycle_metrics(rs_, ss_)
            tm = SF.tram_metrics(g[SG], sim, SF.runs(g["d3"].to_numpy())[0], SF.runs(g["d6"].to_numpy())[0])
            rows.append(dict(split=split, date=str(date.date()), weekday=date.day_name()[:3],
                             cycle_real=cyc.loc["cycle_med", "real"], cycle_sim=cyc.loc["cycle_med", "sim"],
                             ew_real=cyc.loc["ew_green_med", "real"], ew_sim=cyc.loc["ew_green_med", "sim"],
                             ns_real=cyc.loc["ns_green_med", "real"], ns_sim=cyc.loc["ns_green_med", "sim"],
                             ks_cycle=ks["cyc"][0], w1_cycle=ks["cyc"][1], ks_ew=ks["g_ew"][0], w1_ew=ks["g_ew"][1],
                             ks_ns=ks["g_ns"][0], w1_ns=ks["g_ns"][1],
                             sg12_on_arrival_real=tm.loc["sg12_green_on_arrival_pct", "real"], sg12_on_arrival_sim=tm.loc["sg12_green_on_arrival_pct", "sim"],
                             sg11_on_arrival_real=tm.loc["sg11_green_on_arrival_pct", "real"], sg11_on_arrival_sim=tm.loc["sg11_green_on_arrival_pct", "sim"],
                             ped_wait_med=float(np.median(c.stats["ped_wait"])) if c.stats["ped_wait"] else np.nan))
    r = pd.DataFrame(rows)
    Path(a.report).mkdir(parents=True, exist_ok=True)
    r.to_csv(f"{a.report}/offline_by_day.csv", index=False)
    pd.set_option("display.width", 250)
    print(r.groupby("split")[["cycle_real", "cycle_sim", "ew_real", "ew_sim", "ns_real", "ns_sim", "w1_cycle", "w1_ew", "w1_ns",
                              "sg12_on_arrival_real", "sg12_on_arrival_sim", "sg11_on_arrival_real", "sg11_on_arrival_sim"]].mean().round(2).T.to_string())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", nargs="*", default=None)
    ap.add_argument("--train-until", default=None)
    ap.add_argument("--train-days", nargs="*", default=None, help="per-day CSV files/globs used for calibration (multi-day mode)")
    ap.add_argument("--test-days", nargs="*", default=None, help="per-day CSV files/globs evaluated offline (hold-out)")
    ap.add_argument("--fit-days", type=int, default=6, help="training days sampled for the grid search")
    ap.add_argument("--out", default=str(HERE / "controller_params.json"))
    ap.add_argument("--report", default=str(HERE / "results" / "controller_offline"))
    a = ap.parse_args()
    if a.train_days:
        main_multi(a)
    else:
        sys.argv = [sys.argv[0]] + sys.argv[1:]
        main_single()


if __name__ == "__main__":
    main()

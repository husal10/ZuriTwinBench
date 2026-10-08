"""Signal-timing fidelity: simulated/controller greens vs the field signal data (per signal group).

Inputs are two DataFrames with columns sg1..sg12 (1 = green) indexed by consecutive seconds.
Metrics per group: green starts, total green seconds, median green duration, two-sample KS statistic and
Wasserstein-1 distance of the green-duration distributions, and hourly agreement (GEH of green starts and of
green seconds per hour, temporal R2 of green seconds).  Cycle metrics: EW/NS vehicle-stage cycle length, the
clearance gaps, and public-transport priority (green on tram arrival, wait).
"""
import numpy as np, pandas as pd
from fidelity import geh, _r2

SG = [f"sg{i}" for i in range(1, 13)]


def runs(x):
    a = np.concatenate([[0], (np.asarray(x) == 1).astype(int), [0]])
    d = np.diff(a)
    s, e = np.where(d == 1)[0], np.where(d == -1)[0]
    return s, e


def ks_w1(a, b):
    a, b = np.sort(np.asarray(a, float)), np.sort(np.asarray(b, float))
    if len(a) == 0 or len(b) == 0:
        return float("nan"), float("nan")
    grid = np.union1d(a, b)
    ks = float(np.max(np.abs(np.searchsorted(a, grid, side="right") / len(a) - np.searchsorted(b, grid, side="right") / len(b))))
    qs = np.linspace(0, 1, 201)
    w1 = float(np.mean(np.abs(np.quantile(a, qs) - np.quantile(b, qs))))
    return ks, w1


def per_group(real, sim, period_s=3600):
    rows = {}
    n = len(real)
    hb = np.arange(n) // period_s
    for g in SG:
        rs, re = runs(real[g].to_numpy())
        ss, se = runs(sim[g].to_numpy())
        ks, w1 = ks_w1(re - rs, se - ss)
        r_start = pd.Series(np.bincount(hb[rs], minlength=hb.max() + 1))
        s_start = pd.Series(np.bincount(hb[ss], minlength=hb.max() + 1))
        r_sec = real[g].groupby(hb).sum().to_numpy()
        s_sec = sim[g].groupby(hb).sum().to_numpy()
        rows[g] = {
            "starts_real": len(rs), "starts_sim": len(ss),
            "green_s_real": int(real[g].sum()), "green_s_sim": int(sim[g].sum()),
            "dur_med_real": float(np.median(re - rs)) if len(rs) else np.nan,
            "dur_med_sim": float(np.median(se - ss)) if len(ss) else np.nan,
            "dur_ks": ks, "dur_w1_s": w1,
            "geh_starts_hourly_pass_pct": float(100 * (geh(s_start, r_start) < 5).mean()),
            "geh_green_s_hourly_pass_pct": float(100 * (geh(s_sec, r_sec) < 5).mean()),
            "r2_green_s_hourly": _r2(s_sec.astype(float), r_sec.astype(float)),
        }
    return pd.DataFrame(rows).T


def cycle_metrics(real, sim):
    out = {}
    for name, df in (("real", real), ("sim", sim)):
        ew = (df["sg1"] | df["sg4"] | df["sg5"]).to_numpy() if df["sg1"].dtype != float else ((df["sg1"] + df["sg4"] + df["sg5"]) > 0).to_numpy()
        ns = ((df["sg2"] + df["sg3"] + df["sg6"]) > 0).to_numpy()
        es, ee = runs(ew); ns_, ne = runs(ns)
        cyc = np.diff(es)
        g_ew, g_ns = ee - es, ne - ns_
        # gap EW end -> next NS start and NS end -> next EW start
        gen = [ns_[ns_ >= b][0] - b for b in ee if (ns_ >= b).any()]
        gne = [es[es >= b][0] - b for b in ne if (es >= b).any()]
        out[name] = dict(cycle_med=np.median(cyc), cycle_p90=np.percentile(cyc, 90), ew_green_med=np.median(g_ew), ns_green_med=np.median(g_ns),
                         gap_ew_ns_med=np.median(gen), gap_ns_ew_med=np.median(gne), cycles=len(cyc))
        out[name + "_arrays"] = dict(cyc=cyc, g_ew=g_ew, g_ns=g_ns)
    res = pd.DataFrame({k: v for k, v in out.items() if not k.endswith("arrays")})
    ks = {k: ks_w1(out["real_arrays"][k], out["sim_arrays"][k]) for k in ("cyc", "g_ew", "g_ns")}
    return res, ks


def tram_metrics(real, sim, d3_rise, d6_rise, sim_waits=None):
    """Green on tram arrival (sg12 at d3 rise, sg11 at d6 rise) and waiting time for the same arrival events."""
    out = {}
    for name, df in (("real", real), ("sim", sim)):
        s12, s11 = df["sg12"].to_numpy(), df["sg11"].to_numpy()
        w12 = [next((k for k in range(0, 120) if a + k < len(s12) and s12[a + k] == 1), 120) for a in d3_rise if a < len(s12)]
        w11 = [next((k for k in range(0, 120) if a + k < len(s11) and s11[a + k] == 1), 120) for a in d6_rise if a < len(s11)]
        if name == "sim" and sim_waits is not None:       # closed loop: waiting of the simulated trams at the stop line
            w12, w11 = list(sim_waits["n"]) or [0], list(sim_waits["s"]) or [0]
        out[name] = {"n_arrivals_d3": len(w12), "sg12_green_on_arrival_pct": 100 * np.mean(np.array(w12) == 0),
                     "sg12_wait_med_s": float(np.median(w12)), "sg12_wait_p90_s": float(np.percentile(w12, 90)),
                     "n_arrivals_d6": len(w11), "sg11_green_on_arrival_pct": 100 * np.mean(np.array(w11) == 0),
                     "sg11_wait_med_s": float(np.median(w11)), "sg11_wait_p90_s": float(np.percentile(w11, 90))}
    return pd.DataFrame(out)

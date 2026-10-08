#!/usr/bin/env python3
"""Aggregate per-day replay runs into multi-day fidelity tables (detector metrics pooled over days, by day type; signal timing).

  python aggregate_multiday.py --runs OUT --days DAYDIR --labels exact actuated_tsp --out results/multiday
Reads OUT/<label>/<date>/<date>/{fidelity.csv,sim_detectors.csv,signal_*.csv}; real data from DAYDIR/<date>.csv.
"""
import argparse, sys
from pathlib import Path
import numpy as np, pandas as pd

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import data as D, fidelity as F, signal_fidelity as SF
import json

DETS = D.DET
INPUTS = {d for d, c in json.load(open(HERE / "detectors.json")).items() if not d.startswith("_") and c.get("role") == "input"}


def rundir(root, label, date):
    return Path(root) / label / date / date


def dates_of(root, label):
    return sorted(p.name for p in (Path(root) / label).iterdir() if p.is_dir() and (p / p.name / "fidelity.csv").exists())


def daytype(date):
    return "weekend" if pd.Timestamp(date).dayofweek >= 5 else "weekday"


def counts(days_dir, root, label, date):
    real = D.load([str(Path(days_dir) / f"{date}.csv")])
    d0, g, _ = next(D.days(real))
    sim = pd.read_csv(rundir(root, label, date) / "sim_detectors.csv")
    sim.index = g.index[: len(sim)]
    return F.period_counts(g, DETS, 3600), F.period_counts(sim, DETS, 3600)


def pooled(args, label, dates, excl, tag):
    O, S = [], []
    for d in dates:
        o, s = counts(args.days, args.runs, label, d)
        O.append(o); S.append(s)
    O, S = pd.concat(O, ignore_index=True), pd.concat(S, ignore_index=True)
    fid, agg = F.evaluate(O, S, DETS, exclude=tuple(excl))
    fid.insert(0, "label", label); fid.insert(1, "subset", tag); fid.insert(2, "n_days", len(dates))
    return fid, pd.Series(agg, name=f"{label}|{tag}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", required=True); ap.add_argument("--days", required=True)
    ap.add_argument("--labels", nargs="+", required=True); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    per_day, pooled_all, aggs, sig, rep_rows, rb_rows = [], [], [], [], [], []
    for label in a.labels:
        dates = dates_of(a.runs, label)
        excl = {"d1"} if label == "exact" else (INPUTS | {"d1"})
        for d in dates:
            f = pd.read_csv(rundir(a.runs, label, d) / "fidelity.csv", index_col=0)
            ag = pd.read_csv(rundir(a.runs, label, d) / "fidelity_aggregate.csv", index_col=0)["value"]
            f = f.assign(label=label, date=d, daytype=daytype(d)).reset_index().rename(columns={"index": "detector"})
            per_day.append(f)
            rp = pd.read_csv(rundir(a.runs, label, d) / 'report.csv', index_col=0)
            rep_rows.append(rp.assign(label=label, date=d, daytype=daytype(d)).reset_index().rename(columns={'index': 'detector'}))
            sd = pd.read_csv(rundir(a.runs, label, d) / 'sim_detectors.csv', usecols=['sg_applied_ok'])
            rb_rows.append(dict(label=label, date=d, rows=len(sd), readback_match_pct=100 * sd['sg_applied_ok'].mean()))
            sp = rundir(a.runs, label, d) / "signal_cycle.csv"
            if sp.exists():
                c = pd.read_csv(sp, index_col=0)
                t = pd.read_csv(rundir(a.runs, label, d) / "signal_tram.csv", index_col=0)
                ks = pd.read_csv(rundir(a.runs, label, d) / "signal_cycle_ks.csv", index_col=0)
                sig.append(dict(label=label, date=d, daytype=daytype(d), cycle_real=c.loc["cycle_med", "real"], cycle_sim=c.loc["cycle_med", "sim"],
                                ew_real=c.loc["ew_green_med", "real"], ew_sim=c.loc["ew_green_med", "sim"], ns_real=c.loc["ns_green_med", "real"],
                                ns_sim=c.loc["ns_green_med", "sim"], w1_cycle=ks.loc["w1_s", "cyc"], w1_ew=ks.loc["w1_s", "g_ew"], w1_ns=ks.loc["w1_s", "g_ns"],
                                sg12_nowait_real=t.loc["sg12_green_on_arrival_pct", "real"], sg12_nowait_sim=t.loc["sg12_green_on_arrival_pct", "sim"],
                                sg11_nowait_real=t.loc["sg11_green_on_arrival_pct", "real"], sg11_nowait_sim=t.loc["sg11_green_on_arrival_pct", "sim"],
                                sg12_p90_real=t.loc["sg12_wait_p90_s", "real"], sg12_p90_sim=t.loc["sg12_wait_p90_s", "sim"],
                                sg11_p90_real=t.loc["sg11_wait_p90_s", "real"], sg11_p90_sim=t.loc["sg11_wait_p90_s", "sim"]))
        subsets = {"all": dates, "weekday": [d for d in dates if daytype(d) == "weekday"], "weekend": [d for d in dates if daytype(d) == "weekend"]}
        for tag, ds in subsets.items():
            if len(ds) == 0:
                continue
            fid, ag = pooled(a, label, ds, excl, tag)
            pooled_all.append(fid.reset_index().rename(columns={"index": "detector"})); aggs.append(ag)
            print(f"{label:16s} {tag:8s} days={len(ds):2d}  GEH-pass {ag['geh_pass_rate_pct']:.0f}%  R2(daily counts) {float(ag['r2_daily_counts_across_detectors']):.3f}  "
                  f"R2(all hours) {float(ag['r2_all_detector_periods']):.3f}  hourly GEH<5 {float(ag['geh_period_pass_rate_pct']):.1f}%", flush=True)
    pd.concat(per_day).to_csv(out / "detector_by_day.csv", index=False)
    pd.concat(pooled_all).to_csv(out / "detector_pooled.csv", index=False)
    pd.DataFrame(aggs).to_csv(out / "aggregate_pooled.csv")
    allrep = pd.concat(rep_rows)
    for lab in a.labels:
        allrep[allrep.label == lab].to_csv(out / f'{lab}_report_by_day.csv', index=False)
    pd.DataFrame(rb_rows).to_csv(out / 'readback_by_day.csv', index=False)
    if sig:
        pd.DataFrame(sig).to_csv(out / "signal_by_day.csv", index=False)


if __name__ == "__main__":
    main()

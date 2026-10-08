#!/usr/bin/env python3
"""Run replay.py on many one-day CSVs in parallel (one SUMO process per day).

  python run_batch.py --days DAYDIR --dates 2019-02-01:2019-02-28 --mode actuated --label actuated_tsp --out OUT -j 4
  python run_batch.py ... --mode actuated --label actuated_notsp --extra --no-tram-priority      # counterfactual: no TSP
Outputs: OUT/<label>/<date>/<date>/{fidelity.csv, signal_*.csv, kpi_queues.csv.gz, ...} + OUT/<label>/<date>/tripinfo.xml
"""
import argparse, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import pandas as pd

HERE = Path(__file__).parent


def run_one(a, date):
    out = Path(a.out) / a.label / date
    if (out / date / "fidelity.csv").exists() and not a.force:
        return date, 0, 0.0
    out.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, str(HERE / "replay.py"), "--csv", str(Path(a.days) / f"{date}.csv"), "--mode", a.mode, "--out", str(out)] + a.extra
    t = time.time()
    with open(out / "run.log", "w") as lf:
        rc = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT).returncode
    return date, rc, time.time() - t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--days", required=True)
    ap.add_argument("--dates", required=True, help="START:END (inclusive) or comma list")
    ap.add_argument("--mode", default="actuated")
    ap.add_argument("--label", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("-j", type=int, default=4)
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--extra", nargs=argparse.REMAINDER, default=[])
    a = ap.parse_args()
    a.label = a.label or a.mode
    if ":" in a.dates:
        s, e = a.dates.split(":"); dates = [str(d.date()) for d in pd.date_range(s, e)]
    else:
        dates = a.dates.split(",")
    dates = [d for d in dates if (Path(a.days) / f"{d}.csv").exists()]
    t0 = time.time()
    with ThreadPoolExecutor(a.j) as ex:
        for date, rc, dt in ex.map(lambda d: run_one(a, d), dates):
            print(f"{a.label} {date} rc={rc} {dt:.0f}s  (elapsed {time.time() - t0:.0f}s)", flush=True)


if __name__ == "__main__":
    main()

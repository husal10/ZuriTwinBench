#!/usr/bin/env python3
"""Split the multi-day Genser et al. CSVs into one CSV per calendar day (fast parallel runs, low memory).
   python split_days.py OUT_DIR file1.csv file2.csv ..."""
import sys
from pathlib import Path
import pandas as pd

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
for f in sys.argv[2:]:
    df = pd.read_csv(f)
    df.columns = [c.strip().lower() for c in df.columns]
    df["time"] = pd.to_datetime(df["time"])
    for date, g in df.groupby(df["time"].dt.normalize()):
        p = out / f"{date.date()}.csv"
        g.to_csv(p, index=False, mode="a", header=not p.exists())   # a day may straddle two files
    print("split", f, flush=True)
print(len(list(out.glob("*.csv"))), "day files in", out)

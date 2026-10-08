"""Multi-day loader for the Genser et al. 1-s loop-detector / signal CSVs.

Accepts any mix of files / directories / globs.  Every file needs the columns
time, d1..d10, sg1..sg12 (the layout of feb04_only.csv).  Rows are merged,
de-duplicated and split into calendar days; each day is returned as a dense
86400-row (or shorter, if the file is truncated) numpy frame so that row i is
exactly second i of the day.  Missing seconds are reported, detectors are set to
0 and signals hold their last state (flag in `missing`).
"""
import glob, os
import numpy as np, pandas as pd

DET = [f"d{i}" for i in range(1, 11)]
SG = [f"sg{i}" for i in range(1, 13)]


def expand(paths):
    out = []
    for p in paths:
        if os.path.isdir(p):
            out += sorted(glob.glob(os.path.join(p, "*.csv")))
        else:
            out += sorted(glob.glob(p)) or [p]
    if not out:
        raise FileNotFoundError(f"no CSV found for {paths}")
    return out


def load(paths):
    frames = []
    for f in expand(paths):
        df = pd.read_csv(f)
        df.columns = [c.strip().lower() for c in df.columns]
        miss = [c for c in ["time"] + DET + SG if c not in df.columns]
        if miss:
            raise ValueError(f"{f}: missing columns {miss}")
        df["time"] = pd.to_datetime(df["time"])
        frames.append(df[["time"] + DET + SG])
    df = pd.concat(frames).drop_duplicates("time", keep="last").sort_values("time")
    return df.set_index("time")


def days(df):
    """Yield (date, DataFrame indexed by second-of-day 0..N, missing_mask)."""
    for date, g in df.groupby(df.index.normalize()):
        sod = ((g.index - date).total_seconds()).astype(int)
        last = int(sod.max())
        full = pd.RangeIndex(int(sod.min()), last + 1)
        g = g.set_axis(sod).reindex(full)
        missing = g["sg1"].isna().to_numpy()
        g[SG] = g[SG].ffill().bfill()
        g[DET] = g[DET].fillna(0)
        yield date, g.astype(float), missing

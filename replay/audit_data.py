#!/usr/bin/env python3
"""Per-day coverage / quality audit of the Genser et al. CSV files (run before any multi-day replay)."""
import sys, glob
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0, str(Path(__file__).parent))
import data as D

def main(paths, out):
    df = D.load(paths)
    rows = []
    for date, g in df.groupby(df.index.normalize()):
        t = (g.index - date).total_seconds().astype(int).to_numpy()
        gap = np.diff(t)
        d3 = g["d3"].to_numpy() > 0
        rows.append(dict(date=date.date(), weekday=date.day_name()[:3], n_rows=len(g), first=str(g.index[0].time()), last=str(g.index[-1].time()),
                         coverage_pct=100 * len(g) / 86400, gaps_gt1s=int((gap > 1).sum()), max_gap_s=int(gap.max()) if len(gap) else 0,
                         night_rows=int((g["sg1"] == 8).sum()), trams_d3=int((d3[1:] & ~d3[:-1]).sum()),
                         d4=int(((g["d4"].to_numpy()[1:] > 0) & (g["d4"].to_numpy()[:-1] == 0)).sum()),
                         d5=int(((g["d5"].to_numpy()[1:] > 0) & (g["d5"].to_numpy()[:-1] == 0)).sum()),
                         bad_codes=int((~g[D.SG].isin([0, 1, 8]).all(axis=1)).sum() + (~g[D.DET].isin([0, 1]).all(axis=1)).sum())))
    r = pd.DataFrame(rows)
    r.to_csv(out, index=False)
    pd.set_option("display.width", 250)
    print(r.to_string())
    return r

if __name__ == "__main__":
    main(sys.argv[2:] or ["."], sys.argv[1])

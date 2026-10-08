#!/usr/bin/env python3
"""Calibrate the injected-arrival splits in detectors.json from TRAINING day files (no test data touched).

Quantities (all ratios of daily totals pooled over the training days):
  bike share b   = non-tram d8 events (not 20-50 s after a d3 tram detection) / d4 pulses   (bikes N->S, sg3)
  N->W lane split p10 = d10 / ((1-b) d4)       share of N->W cars on the outer W_out lane (d10); the rest use the d9 lane
  E->W share         = (d9 - (1-b)(1-p10) d4) / d5   of the d5 arrivals, the rest turn E->N (sg5)
These come from flow conservation, they are not measurements.
  python calibrate_inputs.py day1.csv day2.csv ...
"""
import json, sys
from pathlib import Path
import numpy as np, pandas as pd
HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import data as D, signal_fidelity as SF


def stats(paths):
    tot = dict(d4=0, d5=0, d9=0, d10=0, d8=0, d8_nontram=0, days=0)
    for p in paths:
        g = D.load([p]).reset_index(drop=True)
        for d in ("d4", "d5", "d9", "d10"):
            tot[d] += len(SF.runs(g[d].to_numpy())[0])
        s8 = SF.runs(g["d8"].to_numpy())[0]; s3 = SF.runs(g["d3"].to_numpy())[0]
        tot["d8"] += len(s8)
        tot["d8_nontram"] += sum(1 for a in s8 if not ((a - s3 >= 20) & (a - s3 <= 50)).any())
        tot["days"] += 1
    return tot


def splits(t):
    b = t["d8_nontram"] / t["d4"]
    p10 = t["d10"] / ((1 - b) * t["d4"])
    nw9 = (1 - b) * (1 - p10) * t["d4"]
    ew = (t["d9"] - nw9) / t["d5"]
    return b, p10, ew


def loops(b, p10, ew):
    r = lambda x: round(float(x), 4)
    d4 = [{"edge": "N_in", "lane": 0, "w": r((1 - b) * p10), "vtype": "car", "route": "rt_car_NW"},
          {"edge": "N_in", "lane": 1, "w": r((1 - b) * (1 - p10)), "vtype": "car", "route": "rt_car_NW"},
          {"edge": "N_in", "lane": 1, "w": r(b), "vtype": "bike", "route": "rt_bike_NS", "shadow": True}]
    d5 = [{"edge": "E_in", "lane": 0, "w": r(1 - ew), "vtype": "car", "route": "rt_car_EN"},
          {"edge": "E_in", "lane": 1, "w": r(ew), "vtype": "car", "route": "rt_car_EW"}]
    return d4, d5


def main():
    paths = sys.argv[1:]
    # day-type specific splits (weekday / weekend), estimated on the training days only
    wk = [p for p in paths if pd.Timestamp(Path(p).stem).dayofweek < 5]
    we = [p for p in paths if pd.Timestamp(Path(p).stem).dayofweek >= 5]
    j = json.load(open(HERE / "detectors.json"))
    res = {}
    for name, ps in (("weekday", wk), ("weekend", we)):
        t = stats(ps); b, p10, ew = splits(t)
        d4, d5 = loops(b, p10, ew)
        res[name] = dict(totals=t, bike_share=b, outer_lane_share=p10, ew_share=ew)
        print(name, t, f"\n  bike {b:.4f}  outer-lane {p10:.4f}  E->W {ew:.4f}")
        if name == "weekday":
            j["d4"]["loops"], j["d5"]["loops"] = d4, d5
        else:
            j["d4"]["loops_weekend"], j["d5"]["loops_weekend"] = d4, d5
    j["d4"]["_inferred"] = "Calibrated on the January training days, separately for weekdays and weekends (loops / loops_weekend): bike share = non-tram d8 events / d4 pulses; outer W_out lane (d10) share of N->W cars = d10 / ((1-b) d4). Flow conservation, not measurements."
    j["d5"]["_inferred"] = "Calibrated on the January training days per day type: E->W share = (d9 - N->W cars on the d9 lane)/d5; the rest turn E->N (sg5). Flow conservation."
    json.dump(j, open(HERE / "detectors.json", "w"), indent=1)
    json.dump(res, open(HERE / "results" / "input_calibration.json", "w"), indent=1)
    return
    t = stats(paths)
    b = t["d8_nontram"] / t["d4"]
    p10 = t["d10"] / ((1 - b) * t["d4"])
    nw9 = (1 - b) * (1 - p10) * t["d4"]
    ew = (t["d9"] - nw9) / t["d5"]
    print(t, f"\nbike share {b:.4f}  outer-lane share {p10:.4f}  E->W share {ew:.4f}")
    j = json.load(open(HERE / "detectors.json"))
    r = lambda x: round(float(x), 4)
    j["d4"]["loops"] = [{"edge": "N_in", "lane": 0, "w": r((1 - b) * p10), "vtype": "car", "route": "rt_car_NW"},
                        {"edge": "N_in", "lane": 1, "w": r((1 - b) * (1 - p10)), "vtype": "car", "route": "rt_car_NW"},
                        {"edge": "N_in", "lane": 1, "w": r(b), "vtype": "bike", "route": "rt_bike_NS", "shadow": True}]
    j["d4"]["_inferred"] = (f"Calibrated on {t['days']} training days: bike share {b:.4f} (non-tram d8 events / d4 pulses); outer W_out lane (d10) share of N->W cars "
                            f"{p10:.4f} (d10 / N->W cars). Flow conservation, not measurements.")
    j["d5"]["loops"] = [{"edge": "E_in", "lane": 0, "w": r(1 - ew), "vtype": "car", "route": "rt_car_EN"},
                        {"edge": "E_in", "lane": 1, "w": r(ew), "vtype": "car", "route": "rt_car_EW"}]
    j["d5"]["_inferred"] = f"Calibrated on {t['days']} training days: E->W share {ew:.4f} = (d9 - N->W cars on the d9 lane)/d5; the rest turn E->N (sg5). Flow conservation."
    json.dump(j, open(HERE / "detectors.json", "w"), indent=1)
    json.dump(dict(totals=t, bike_share=b, outer_lane_share=p10, ew_share=ew), open(HERE / "results" / "input_calibration.json", "w"), indent=1)


if __name__ == "__main__":
    main()

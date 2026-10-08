# ZuriTwinBench — Zürich Intersection Digital Twin

[![Validate Calibration](https://github.com/husal10/ZuriTwinBench/actions/workflows/validate.yml/badge.svg)](https://github.com/husal10/ZuriTwinBench/actions/workflows/validate.yml)
[![License: CC BY 4.0](https://img.shields.io/badge/Data-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/)
[![SUMO 1.18+](https://img.shields.io/badge/SUMO-1.18+-green.svg)](https://sumo.dlr.de)

A SUMO digital twin of the Langstrasse intersection in Zürich, Switzerland, built from the open one-second loop-detector and signal-state dataset of [Genser et al. (2023)](https://doi.org/10.1016/j.dib.2023.109117) (CC BY 4.0; data from the [ETH Research Collection](https://doi.org/10.3929/ethz-b-000556642)).

## What to use: `replay/`

The maintained twin is in [`replay/`](replay/README.md). It provides, for the full 59-day record (Jan–Feb 2019):

* **exact replay**: the recorded signals sg1–sg12 are imposed every second (the 1 s red-yellow and 3 s amber that the data fold into red and the night flashing mode are re-created) and every detector event d1–d10 is reproduced; the traffic-light state is read back from SUMO and checked each second;
* **hybrid replay**: recorded signals with injected real vehicles;
* **closed-loop actuated twin with transit signal priority** for the two tram groups sg11/sg12: a data-derived controller calibrated on January days and tested on all 28 February days, with injected arrivals (d1, d2, d4, d5, d7) separated from validated outputs (d3, d6, d8, d9, d10);
* fidelity measures: hourly GEH, R², nRMSE, nMAE, MAPE, PBIAS, Willmott's d, Theil's U2, plus signal-timing and tram-waiting statistics.

The SUMO traffic light has exactly 12 link indices, one per signal group. Signal-group movements follow Table 1 of the dataset paper, except sg4 = E→N and sg5 = E→W (Fig. 3 of the paper and the data agree with this; Table 1 lists them the other way round).

```bash
pip install -r requirements.txt
python replay/build_network.py                       # only if you change the geometry
python replay/replay.py --csv data/feb04_only.csv --mode exact --out out
```
See [`replay/README.md`](replay/README.md) for multi-day batches (`run_batch.py`, `aggregate_multiday.py`, `plots_multiday.py`), calibration and the controller.

Note: only one day (4 Feb 2019) is included in `data/`. For multi-day runs download the full dataset from the ETH Research Collection (link above).

## Legacy first version

The top-level folders `network/`, `simulation/`, `calibration/`, `demand/`, `benchmark/` and `build_network.py` are the first, single-day version of ZuriTwinBench. It is kept for reference. Its detector roles and validation design contain errors that the `replay/` twin corrects (calibrators enforcing counts at detectors that were then reported as validated, d6/d7/d8 treated as car detectors although they mostly see trams, a daily-count R² mislabelled as hourly), so its numbers should not be used as validation results.

---

## Continuous Integration

A GitHub Actions workflow runs on every push and pull request. It executes the full calibration evaluation pipeline and enforces the DfT WebTAG M3.1 acceptance criterion (≥ 85% of detectors with GEH < 5.0) as a merge gate. Any change that degrades calibration below this threshold is automatically blocked.

---

## Citation

If you use this work, please cite the underlying dataset:

```bibtex
@article{Genser2023Data,
  title    = {A traffic signal and loop detector dataset of an urban intersection regulated by a fully actuated signal control system},
  journal  = {Data in Brief},
  volume   = {48},
  pages    = {109117},
  year     = {2023},
  issn     = {2352-3409},
  doi      = {10.1016/j.dib.2023.109117},
  url      = {https://www.sciencedirect.com/science/article/pii/S2352340923002366},
  author   = {Alexander Genser and Michail A. Makridis and Kaidi Yang and Lukas Abm{\"u}hl and Monica Menendez and Anastasios Kouvelas},
  keywords = {Intelligent transportation systems, Traffic signals, Loop detectors, Signal control systems, Fully actuated systems}
}
```

---

## Licence

The code in this repository is released under the [MIT licence](LICENSE). The field data (`data/feb04_only.csv`) is published by Genser et al. under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) and must be cited as above.

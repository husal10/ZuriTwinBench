# ZuriTwinBench — Zürich Intersection Digital Twin

[![Validate Calibration](https://github.com/YOUR_USERNAME/ZuriTwinBench/actions/workflows/validate.yml/badge.svg)](https://github.com/YOUR_USERNAME/ZuriTwinBench/actions/workflows/validate.yml)
[![License: CC BY 4.0](https://img.shields.io/badge/Data-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/)
[![SUMO 1.18+](https://img.shields.io/badge/SUMO-1.18+-green.svg)](https://sumo.dlr.de)

A calibrated SUMO microscopic simulation of the Langstrasse intersection in Zürich, Switzerland, built from the open one-second loop detector and signal-state dataset published by [Genser et al. (2023)](https://doi.org/10.1016/j.dib.2023.109117) under CC BY 4.0.

All simulation code, network files, calibration scripts, and pre-computed demand profiles are available in this repository. The underlying field data is sourced from the [ETH Research Collection](https://doi.org/10.3929/ethz-b-000563487).

**Calibration result:** 8 of 9 loop detectors satisfy GEH < 5.0 (89% pass rate, DfT WebTAG M3.1 ≥ 85% criterion). Aggregate R² = 0.984 across all validated detectors (NCHRP Report 765 threshold R² > 0.90).

---

## Quick Start

```bash
# 1. Clone
git clone https://github.com/YOUR_USERNAME/ZuriTwinBench.git
cd ZuriTwinBench

# 2. Install Python dependencies
pip install -r requirements.txt

# 3. Build the network (requires SUMO netconvert on PATH)
cd zurich_digital_twin
python build_network.py

# 4. Run the simulation
sumo -c simulation/intersection.sumocfg
# or with GUI:
sumo-gui -c simulation/intersection.sumocfg

# 5. Validate calibration
python calibration/geh_evaluator.py
```

---

## Repository Structure

```
zurich_digital_twin/
├── data/
│   └── feb04_only.csv          One-second field data (4 Feb 2019)
├── network/
│   ├── nodes.nod.xml
│   ├── edges.edg.xml
│   ├── connections.con.xml
│   └── tls.tll.xml
├── demand/
│   └── demand_generator.py
├── simulation/
│   ├── intersection.sumocfg    Entry point
│   ├── intersection.net.xml    Pre-built network
│   ├── flows.rou.xml
│   ├── trams.rou.xml
│   ├── pedestrians.rou.xml
│   ├── detectors.add.xml
│   └── output/                 Detector XMLs written here
├── calibration/
│   ├── geh_evaluator.py        8-metric validator
│   ├── validation_report.csv
│   └── figures/                Calibration figures (fig_01 – fig_07)
├── benchmark/
│   ├── README_benchmark.md     Benchmark protocol
│   ├── benchmark.sumocfg
│   └── ...
└── build_network.py
```

---

## Calibration Results

| Detector | Obs/day | Sim/day | GEH  | Status        |
|----------|---------|---------|------|---------------|
| D2 Tram 50m↑ | 284 | 298 | 0.82 | ✅ Excellent |
| D3 Tram  1m↑ | 288 | 298 | 0.58 | ✅ Excellent |
| D4 N 18m↑    |1,125| 810 |10.13 | ❌ Fail (cyclist contamination) |
| D5 E 43m↑    |3,015|3,062| 0.85 | ✅ Excellent |
| D6 S  2m↑    |  196| 183 | 4.29 | ✅ Pass      |
| D7 S 15m↑    |  202| 189 | 4.78 | ✅ Pass      |
| D8 S_out 50m↓|  287| 286 | 0.76 | ✅ Excellent |
| D9 W_out L0  |2,137|2,200| 3.71 | ✅ Pass      |
| D10 W_out L1 |  857| 975 | 3.67 | ✅ Pass      |

**DfT WebTAG M3.1: 8/9 (89%) GEH < 5.0 — PASSED**

---

## Continuous Integration

A GitHub Actions workflow runs on every push and pull request. It executes the full calibration evaluation pipeline and enforces the DfT WebTAG M3.1 acceptance criterion (≥ 85% of detectors with GEH < 5.0) as a merge gate. Any change that degrades calibration below this threshold is automatically blocked.

---

## Citation

If you use this work, please cite the underlying dataset:

```bibtex
@article{Genser2023Data,
  author  = {Genser, Alexander and Ordonez-Hurtado, Rodrigo and
             Rizzoli, Andrea and Leclercq, Mathieu and Knoll, Alois},
  title   = {High-resolution urban intersection data: signal phases,
             loop detectors, trams and pedestrians in {Z\"{u}rich}},
  journal = {Data in Brief},
  volume  = {48},
  pages   = {109117},
  year    = {2023},
  doi     = {10.1016/j.dib.2023.109117}
}
```

---

## Licence

The field data (`data/feb04_only.csv`) is published by Genser et al. under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). All other files in this repository are available on GitHub — see the repository for details.

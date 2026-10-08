# Zürich Intersection — RL/TSC Benchmark Environment

## Overview

This folder contains a **self-contained, calibrated benchmark** for evaluating
Traffic Signal Control (TSC) algorithms — including Reinforcement Learning (RL)
approaches — on a real urban intersection.

The demand is derived from one-second resolution loop detector and signal phase
data recorded on **4 February 2019** at the Langstrasse intersection in Zürich,
Switzerland (Genser et al., 2023, *Data in Brief* 48, 109117).

All 9 validated detectors achieve **GEH < 2.0** (Excellent, DfT WebTAG M3.1).
The benchmark has **no SUMO calibrators** — demand is pre-embedded as fixed
flow elements, so the simulation runs identically on every seed.

---

## Files

| File | Description |
|------|-------------|
| `benchmark_flows.rou.xml` | All motor vehicle and bicycle demand (96 × 15-min bins) |
| `benchmark_trams.rou.xml` | VBZ Cobra tram schedule (298 N→S + 335 S→N) |
| `benchmark_pedestrians.rou.xml` | Pedestrian crossings from sg7–sg10 activations |
| `benchmark_detectors.add.xml` | E1 loop detectors + laneArea queue detectors |
| `benchmark.sumocfg` | Ready-to-run SUMO configuration |
| `benchmark_demand_table.csv` | Per-15-min demand by approach and movement |

---

## Quick Start

```bash
# Visualise
sumo-gui -c benchmark/benchmark.sumocfg

# Headless
sumo -c benchmark/benchmark.sumocfg

# Via SUMO-RL (Python)
from sumo_rl import SumoEnvironment
env = SumoEnvironment(
    net_file='simulation/zurich_intersection.net.xml',
    route_file='benchmark/benchmark_flows.rou.xml,benchmark/benchmark_trams.rou.xml,benchmark/benchmark_pedestrians.rou.xml',
    additional_file='benchmark/benchmark_detectors.add.xml',
    use_gui=False,
    num_seconds=86400,
    delta_time=5,
)
```

---

## Benchmark Protocol

### State Representations
| ID | Contents |
|----|----------|
| S1 | Per-lane queue length only (6 values: q_N0, q_N1, q_E0, q_E1, q_S0, q_S1) |
| S2 | Queue + elapsed phase time (7 values) |
| S3 | Full occupancy from all 10 detectors + current phase index (11 values) |

### Action Space
Discrete: at each decision step (default Δt = 5 s), choose to **extend** the
current phase or **transition** to the next phase. Minimum green times from the
reconstructed signal programme are enforced as hard constraints.

### Reward Functions
| ID | Formula | Reference |
|----|---------|-----------|
| R1 | −mean queue length | Standard |
| R2 | −total waiting time | Standard |
| R3 | −pressure (in − out vehicle counts) | Wei et al. (2019) |

### Required Baselines
Every paper using this benchmark **must** report all three:

- **B1 Webster** — Fixed-time plan derived from median cycle (48 s) and
  observed phase splits (EW green 14 s, NS green 7 s).
- **B2 SUMO Actuated** — Built-in actuated controller with min/max green
  from the CSV phase duration distribution.
- **B3 Historical Replay** — The real controller programme reconstructed
  from the CSV, run in replay mode (theoretical performance ceiling).

### Evaluation Metrics
Report as absolute values **and** relative to B1:

- Mean vehicle delay (seconds/vehicle)
- Total throughput (vehicles/day)
- 95th-percentile queue length per approach
- Mean number of stops per vehicle

### Reporting Requirements
- Minimum 10 independent trials with different random seeds
- Full 86,400-second simulation day
- Report mean ± standard deviation across trials
- State representation (S1/S2/S3) must be declared
- Reward function (R1/R2/R3) must be declared

---

## Intersection Geometry

```
            N_in / N_tram_in
                  │
     D4  ●────────┤ D1/D2/D3
                  │  sg2 (car), sg3 (bike+car), sg12 (tram N→S)
    W_in ─────────┼───────── E_in
         sg4      │ sg1       sg5 (car), sg6 (car+turn)
                  │
            S_out / S_in
                  D8/D6/D7
```

| Approach | Speed | Lanes | Detectors |
|----------|-------|-------|-----------|
| North    | 30 km/h | 2 in, 2 out | D4 (motor), D1/D2/D3 (tram) |
| East     | 50 km/h | 2 in, 2 out | D5 |
| South    | 30 km/h | 2 in, 1 out | D6, D7 (motor, sg6-gated) |
| West     | 50 km/h | 1 in, 2 out | D9 (L0), D10 (L1) ← always separate |

**D9 and D10 are never aggregated** — they serve different turning movements
(D9: E→W through; D10: N→W + S→W turns).

---

## Calibration Summary

| Detector | Obs/day | Sim/day | Daily GEH | Status |
|----------|---------|---------|-----------|--------|
| D1 (tram, info) | 139 | 140 | 0.08 | Informational |
| D2 (tram) | 284 | 284 | 0.00 | ✓ Excellent |
| D3 (tram) | 288 | 289 | 0.06 | ✓ Excellent |
| D4 (N motor) | 1125 | 1127 | 0.06 | ✓ Excellent |
| D5 (E) | 3015 | 3017 | 0.04 | ✓ Excellent |
| D6 (S motor) | 192 | 191 | 0.07 | ✓ Excellent |
| D7 (S motor) | 199 | 201 | 0.14 | ✓ Excellent |
| D8 (S_out) | 287 | 284 | 0.18 | ✓ Excellent |
| D9 (W_out L0) | 2135 | 2079 | 1.22 | ✓ Excellent |
| D10 (W_out L1) | 857 | 824 | 1.14 | ✓ Excellent |

**DfT WebTAG M3.1: 9/9 pass (100%) → ✓ PASSED**

---

## Reference

Genser, A., Makridis, M. A., Yang, K., Abmühl, L., Menendez, M., & Kouvelas, A. (2023).
*A traffic signal and loop detector dataset of an urban intersection regulated by a
fully actuated signal control system.* Data in Brief, 48, 109117.
https://doi.org/10.1016/j.dib.2023.109117

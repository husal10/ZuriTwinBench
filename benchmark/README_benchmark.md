# Zürich Intersection — RL/TSC Benchmark Environment

> **Status: legacy, single-day benchmark.** This benchmark belongs to the first version of ZuriTwinBench
> (the WETICE 2026 paper, one day of data, 4 February 2019). It is kept because it is a ready-to-run
> fixed-demand environment, but its demand and detector roles were built on assumptions that the later,
> multi-day work in [`replay/`](../replay/README.md) found to be wrong (see "Known limitations" below).
> For validated, multi-day replay and the actuated twin with transit signal priority, use `replay/`.
>
> | Part of the repository | Data | Signals | Use it for |
> |---|---|---|---|
> | Original twin (`network/`, `simulation/`, `calibration/`) | 1 day | SUMO actuated, calibrators on N/E/S | reproducing the WETICE 2026 paper |
> | **This benchmark** (`benchmark/`) | 1 day (fixed demand) | SUMO actuated (default) | a quick fixed-demand environment for signal-control experiments, with the limitations below |
> | Replay twin (`replay/`) | 59 days, 1 s | recorded signals (exact/hybrid) or data-derived actuated controller with TSP | validated replay, closed-loop control, hold-out validation |

## Overview

This folder contains a **self-contained, calibrated benchmark** for evaluating
Traffic Signal Control (TSC) algorithms — including Reinforcement Learning (RL)
approaches — on a real urban intersection.

The demand is derived from one-second resolution loop detector and signal phase
data recorded on **4 February 2019** at the Langstrasse intersection in Zürich,
Switzerland (Genser et al., 2023, *Data in Brief* 48, 109117).

The fixed demand reproduces the daily counts of the 9 scored detectors of that single day with **GEH < 2.0**. This is an in-sample fit to the same day the demand was built from, not an independent validation (see "Known limitations").
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
    net_file='benchmark/benchmark.net.xml',
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
| North    | 30 km/h | 2 in, 2 out | D4 (MPT and bikes), D1/D2/D3 (trams only) |
| East     | 50 km/h | 2 in, 2 out | D5 |
| South    | 30 km/h | 2 in, 1 out | D6, D7 (mostly trams, sg11; few MPT/bikes with sg6) |
| West     | 50 km/h | 1 in, 2 out | D9 (L0), D10 (L1) ← always separate |

**D9 and D10 are never aggregated** — they are different west-exit lanes. In the multi-day analysis
(`replay/`) D9 is the inner lane (E→W traffic plus part of the N→W turn) and D10 the outer lane (N→W).

---

## Fit of the fixed demand to the 4 Feb 2019 daily totals (in-sample)

| Detector | Obs/day | Sim/day | Daily GEH | Status |
|----------|---------|---------|-----------|--------|
| D1 (tram, info) | 139 | 140 | 0.08 | Informational |
| D2 (tram) | 284 | 284 | 0.00 | ✓ Excellent |
| D3 (tram) | 288 | 289 | 0.06 | ✓ Excellent |
| D4 (N, MPT + bikes) | 1125 | 1127 | 0.06 | ✓ Excellent |
| D5 (E) | 3015 | 3017 | 0.04 | ✓ Excellent |
| D6 (S, mostly trams) | 192 | 191 | 0.07 | ✓ Excellent |
| D7 (S, mostly trams) | 199 | 201 | 0.14 | ✓ Excellent |
| D8 (S_out) | 287 | 284 | 0.18 | ✓ Excellent |
| D9 (W_out L0) | 2135 | 2079 | 1.22 | ✓ Excellent |
| D10 (W_out L1) | 857 | 824 | 1.14 | ✓ Excellent |

**9/9 scored detectors have daily GEH < 5 (and < 2) for this one day.** This says that the fixed demand
reproduces the daily totals it was built from. It is not a validation: the demand was constructed from these same
counts, and daily GEH hides hour-to-hour errors (the multi-day work uses hourly GEH on held-out days).

---

## Known limitations (found by the multi-day work in `replay/`)

1. **Detector roles.** D6, D7 and D8 mostly observe trams (D7→D6 pairs 99.7 %, 98 % of D6 occupancies end
   with the tram signal sg11 green). This benchmark's demand treats D6/D7 as ~190 cars/day on the south approach
   and adds car flows to match D8, which over-represents south-approach cars. sg3 is bicycle-only, so there is no N→S car flow.
2. **Bicycles at D4.** D4 counts motorized traffic and bicycles together; the bicycle share is a small part of it
   (≈2 %, estimated in `replay/`), not the ≈97/day assumed in the first version.
3. **Traffic-light link indices.** The network of this folder carries more TLS link indices than there are signal
   groups. The corrected network in `replay/net/` has exactly 12 link indices, one per signal group (sg1–sg12).
4. **Signal-group movements.** Table 1 of Genser et al. lists sg4 = E→W and sg5 = E→N, but Fig. 3 of the same paper and the d9
   timing show sg4 = E→N and sg5 = E→W; `replay/` uses the latter. This benchmark's network uses the table's assignment.
5. **Pedestrians.** The 5,781 pedestrians are assumed (one per pedestrian green); there are no pedestrian data, so pedestrian
   delay cannot be validated.
6. **One day only.** The demand is a single Monday (4 February 2019); other weekdays and weekends are not covered.

If you need demand and signals validated on many days, use the `replay/` twin.


---

## Reference

Genser, A., Makridis, M. A., Yang, K., Abmühl, L., Menendez, M., & Kouvelas, A. (2023).
*A traffic signal and loop detector dataset of an urban intersection regulated by a
fully actuated signal control system.* Data in Brief, 48, 109117.
https://doi.org/10.1016/j.dib.2023.109117

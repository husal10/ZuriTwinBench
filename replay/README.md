# Zürich intersection — SUMO detector + signal replay

Replays the Genser et al. (2023) 1-s dataset in SUMO: signals sg1–sg12 forced from the CSV
every simulated second, detectors d1–d10 as induction loops at the paper's positions, multi-day.

> Genser, Makridis, Yang, Abmühl, Menendez, Kouvelas (2023). *A traffic signal and loop detector dataset of an
> urban intersection regulated by a fully actuated signal control system.* Data in Brief 48, 109117.
> Data: ETH Research Collection, doi:10.3929/ethz-b-000556642 (CC BY 4.0).
> (The top-level README of this repo cites the wrong title/authors.)

## Run
```bash
pip install eclipse-sumo traci pandas numpy         # or a system SUMO >= 1.18
python replay/build_network.py                      # only needed if you edit the geometry
python replay/replay.py --csv data/feb04_only.csv --mode exact --out out
# multi-day: any mix of files / directories / globs; days are split automatically
python replay/replay.py --csv /path/to/intersection_data_set_*.csv --mode exact --out out
python replay/replay.py --csv data/ --from "2019-02-04 06:00" --to "2019-02-04 10:00" --gui   # watch it
```
Full day ≈ 5 min headless (≈ 86 400 TraCI steps). Outputs per day in `out/<date>/`:
`sim_detectors.csv` (same layout as the input; detector columns = what the SUMO loops saw, sg columns = field
signals, `sg_applied_ok` = TLS state read back from SUMO equals the commanded state) and `report.csv`
(events, occupancy, per-second accuracy, Jaccard, ±`--tol` s event precision/recall). `out/report_<mode>.csv` aggregates days.

## Clock
Sim time = seconds since 00:00 of the first day in the data; day *n* starts at *n*·86 400 s, so the
SUMO clock is the time of day (`--human-readable-time`). Missing days are skipped by fast-forwarding
(network emptied); missing seconds inside a day are reported (`missing_seconds`): detectors 0, signals held.

## Signals (paper, Table 1 / Fig. 1)
CSV: 1 = green, 0 = red; the 1 s red-yellow before and 3 s yellow after each green are folded into 0, so the
replay re-creates them for the vehicle/bike groups sg1–sg6 (`--red-yellow 1`, `--amber 3`; green timing is untouched).
Code 8 (present in `feb04_only.csv`, 01:00:54–05:00:00, all groups) = night mode, replayed as
unregulated/blinking (`o` for N–S, `O` main road, trams, crossings).
Groups → links are resolved from the net (`sg_map.py`), not by index. 100 % of seconds in the test day read back identical.

| sg | movement | | sg | movement |
|---|---|---|---|---|
| 1 | W→E | | 7, 9 | pedestrians N/S |
| 2 | N→W | | 8, 10 | pedestrians W/E |
| 3 | N→S, bikes only | | 11 | PT S→N |
| 4 | E→W | | 12 | PT N→S |
| 5 | E→N | | 6 | S→W / N / E |

## Detectors (paper, Table 2) — `detectors.json`
D1/D2/D3 PT 220/50/1 m upstream (sg12); D4 18 m (sg2,3), D5 43 m (sg4,5), D6 2 m and D7 15 m (sg6,11) upstream;
D8 50 m, D9/D10 10 m downstream. Distances are from the stop line / junction exit.

## Two replay modes
* **`exact`** (default): every field detection is reproduced by a virtual vehicle on that loop for exactly the
  observed occupancy. Result on the full day 2019-02-04 (86 400 s; `results/2019-02-04_exact_report.csv`): all ten detectors identical to the
  data at 1 s resolution (event precision/recall 1.0, per-second Jaccard 1.0), signal read-back 100 %. Vehicles do not travel through the junction.
* **`hybrid`**: d2, d4, d5, d7 inject *real* vehicles (speed from the occupancy time) that drive through the junction
  under the replayed signals; d1 stays a ghost; d3, d6, d8, d9, d10 are **emergent** and are the validation targets.
  This is the starting point for research with real vehicle dynamics; fidelity of the emergent detectors is
  moderate (see the report) and not tuned.

## What differs from the original package, and why
1. **Not a replay.** The package runs hourly calibrator flow rates; here every event is time-aligned.
2. **d6, d7, d8 are not car detectors.** In the data they fire ≈285×/day, pair with the tram chain
   (d7→d6 26 s; d2→d3 6 s→d8 34 s; 285/288/287 of ≈ 288 trams) and precede sg11/sg12.
3. **Pedestrian crossings are real SUMO crossings** controlled by the TLS (the package's were disconnected edges).
4. **Movements follow Table 1**: sg3 is bikes only, sg5 is E→N, sg6 serves W/N/E.
5. Signal timing comes from the data; nothing is re-actuated.

## Assumptions to verify (not stated in the paper)
* Arm each of sg7–sg10 crosses (net: sg7 W, sg8 N, sg9 E, sg10 S).
* Lane layout / lane counts and the lane of D9 vs D10: inferred D9 = left (E→W) lane, D10 = right (N→W) lane
  from counts (d9 ≈ 2137 ≈ E→W; d10 ≈ 857 ≈ N→W) and Fig. 3.
* Injection splits in hybrid mode (`_inferred` fields in `detectors.json`): d5 71 % E→W / 29 % E→N; d4 80 % cars N→W / 20 % bikes N→S.
* Tram speeds/geometry (30 km/h, separate rail edges); no pedestrian or bicycle demand beyond d4's bike share.

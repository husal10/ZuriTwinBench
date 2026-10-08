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
D8 50 m, D9/D10 10 m downstream. Distances are from the stop line / junction exit. What the CSV shows about each:

* **d1** (PT, 220 m): 139 events/day, *every one* followed by a d2 event within 150 s; occupancy 26–46 s (median 33 s);
  only 07–22 h. So d1 sees trams **dwelling at a stop** (one line, about half of the ~285 trams; 60 % of d2 events are preceded by a d1 event).
  Hybrid mode models this as a real tram that stops on d1 for the observed time and then continues.
* **d2/d3/d8** form the N→S tram chain (d2→d3 6 s, d3→d8 34 s). d8 also sees non-tram traffic: 26 of 287 events are not
  tram-linked (short, 2–5 s), i.e. bikes/MPT on the S arm. d8 therefore keeps loops on the tram *and* the road lanes and is validated on all of them.
* **d6/d7** (shared by MPT with sg6 and PT with sg11): sg6 = MPT only, sg11 = tram only. An event is PT when sg11 is green at the
  end of the occupancy (98 % of d6 events; sg6: 18 %) and MPT otherwise; d7 inherits the class of the d6 event that follows it (99.7 % of
  d7 events are followed by a d6 event within 60 s). d7 occupancy is about 28 s regardless of the signal: trams dwelling at a stop on the S arm.
  Classes per day are printed by the replay.
* **d4/d5** arrive independently of the signals; d4: about 97.5 % cars N→W (sg2), about 2.5 % bikes N→S (sg3, upper bound from d8). d5: 71 % E→W (sg4), 29 % E→N (sg5).
* **d9/d10**: W_out inner lane (E→W) / outer lane (N→W).
* **West arm**: there is no detector on the W approach or on the E exit, so the W→E movement (sg1) exists only as a signal; no vehicles are generated for it.
* **sg7–sg10**: no pedestrian data; the crosswalks are simply open whenever the group is green (no pedestrians are generated).

## Two replay modes
* **`exact`** (default): every field detection is reproduced by a virtual vehicle on that loop for exactly the
  observed occupancy. Result on the full day 2019-02-04 (86 400 s; `results/2019-02-04_exact_*.csv`): all ten detectors identical to the
  data at 1 s resolution (event precision/recall 1.0, per-second Jaccard 1.0), signal read-back 100 %. Vehicles do not travel through the junction.
* **`hybrid`**: d2, d4, d5, d7 inject *real* vehicles (speed from the occupancy time) that drive through the junction
  under the replayed signals; d1 stays a ghost; d3, d6, d8, d9, d10 are **emergent** and are the validation targets.
  This is the starting point for research with real vehicle dynamics; fidelity of the emergent detectors is
  moderate (see the report) and not tuned.

## Fidelity metrics (same set as the ZuriTwinBench paper / `calibration/geh_evaluator.py`)
After every day the replay prints and saves (`out/<date>/fidelity.csv`, `fidelity_aggregate.csv`; pooled over all days in
`out/fidelity_all_days*.csv`) the paper's metrics on rising-edge counts per `--period` (default 3600 s), per detector:
daily GEH + per-period GEH p50/p90/pass-% (DfT WebTAG M3.1, < 5), nRMSE and nMAE (FHWA), MAPE, R² (NCHRP 765, > 0.90),
PBIAS (ASCE), Willmott *d*, Theil U2. Aggregates: DfT pass rate over the detectors in the criterion (≥ 85 % needed; d1 is
excluded as in the paper, `--exclude`), and R² of simulated vs observed daily counts across detectors. The second-level
report (`report.csv`) adds event precision/recall (±`--tol` s), occupancy and per-second Jaccard.
The paper's motor-vehicle corrections (sg6 gating, D4 balance) are **not** applied: the replay reproduces the raw loop data.

| 2019-02-04, full day | exact | hybrid (untuned) |
|---|---|---|
| GEH < 5 detectors (9 in criterion) | 100 % → DfT PASS | 77.8 % → FAIL (d9 GEH 18.0, d10 5.8) |
| R² daily counts across detectors | 1.000 | 0.920 |
| per-period GEH pass rate (all detector-hours) | 100 % | 98.1 % |
| per-second Jaccard of the loops | 1.0 all ten | 0.00 – 0.78 |

Hybrid weakness is in the emergent detectors (d9: 1382 vs 2137, d10: 1036 vs 857, d8: 313 vs 287, d2/d3 timing): the
real vehicles' lane choice / queueing and the N→W vs E→W split are not calibrated. That is the research starting point.

## What differs from the original package, and why
1. **Not a replay.** The package runs hourly calibrator flow rates; here every event is time-aligned.
2. **d6, d7, d8 are mostly tram detectors.** In the data they fire ≈285×/day, pair with the tram chain
   (d7→d6 26 s; d2→d3 6 s→d8 34 s) and are released by sg11/sg12; only a handful of events are MPT/bikes (see above).
3. **Pedestrian crossings are real SUMO crossings** controlled by the TLS (the package's were disconnected edges).
4. **Movements follow Table 1**: sg3 is bikes only, sg5 is E→N, sg6 serves W/N/E.
5. Signal timing comes from the data; nothing is re-actuated.

## Remaining assumptions
* Lane layout / lane counts of the arms; tram speed/geometry (30 km/h, separate rail edges).
* Hybrid-mode injection splits and dwell-time model (see `detectors.json`, `_inferred` fields). Hybrid fidelity of the emergent
  detectors (d2, d3, d8, d9, d10) is low and untuned; `exact` is the faithful replay.

## Note on the paper's sg6 gating of D6/D7
Counting only rising edges while sg6 = 1 does not isolate MPT: of the 96 d6 rises with sg6 green, 84 also have sg11 (tram) green, and
98 % of d6 occupancies end with sg11 green. With sg6 = MPT and sg11 = tram only, the classification above gives ≈ 4 (d6) / 2 (d7)
MPT events on 2019-02-04, not 196 / 202. Worth re-checking the D6/D7 corrections and the D4 balance that uses them.

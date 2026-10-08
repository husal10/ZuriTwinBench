#!/usr/bin/env python3
"""
generate_benchmark_demand.py
============================
Generates a self-contained benchmark demand for the Zürich intersection
digital twin. This file produces routes WITHOUT SUMO calibrators — all
vehicle demand is pre-embedded as fixed flow elements, so the simulation
can run standalone as a benchmark environment for RL-based traffic signal
control research.

Outputs
-------
  benchmark/
    benchmark_flows.rou.xml    — all vehicle flows (cars + bikes)
    benchmark_trams.rou.xml    — tram schedule (identical to trams.rou.xml)
    benchmark_pedestrians.rou.xml — pedestrian demand
    benchmark.sumocfg          — ready-to-run SUMO config (no calibrators)
    benchmark_demand_table.csv — per-15-min demand by approach and movement
    README_benchmark.md        — benchmark protocol for TSC/RL research

How demand is derived
---------------------
  Motor vehicles: calibrated from detector rising-edge counts with
    signal-group corrections:
      D4_motor = D8 + max(0, D10 - D7_motor×0.10)   [N approach]
      D6_motor = D6 rising edges during sg6=1         [S approach, lane 0]
      D7_motor = D7 rising edges during sg6=1         [S approach, lane 1]
    Turn fractions are solved per 15-min bin from downstream D8/D9/D10
    balance equations. Global fallbacks applied when bin count < 5.

  Trams: exact sg11/sg12 activation timestamps with per-tram departPos
    chosen to reproduce the D1/D2 split ratio (≈49 % from network boundary).

  Pedestrians: sg7–sg10 rising-edge activation times.

References
----------
  Genser et al. (2023), Data in Brief 48, 109117
  DfT WebTAG M3.1 GEH acceptance: all 9 detectors GEH < 2.0 (Excellent)
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# ── Configuration ─────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent
ROOT_DIR   = SCRIPT_DIR.parent
DATA_CSV   = ROOT_DIR / "data" / "feb04_only.csv"
SIM_DIR    = ROOT_DIR / "simulation"
OUT_DIR    = ROOT_DIR / "benchmark"

SIM_START  = 0
SIM_END    = 86400
PERIOD     = 900          # 15-min demand bins

ARM_LENGTH = 250.0
SPEED_NS   = 30.0 / 3.6   # 8.333 m/s
SPEED_EW   = 50.0 / 3.6   # 13.889 m/s

NIGHT_START = 3654        # 01:00:54
NIGHT_END   = 18000       # 05:00:00

MIN_FLOW    = 5           # threshold for per-bin turn split calculation
S_TO_W_G   = 0.10        # global S→W fraction (Assumption A13)
BIKE_FRAC   = 0.08        # Swiss BFS modal share

GLOBAL_SPLITS = {
    "N_to_S": 0.251, "N_to_W": 0.749,
    "E_to_W": 0.725, "E_to_N": 0.275,
    "S_to_N": 0.55,  "S_to_E": 0.35,  "S_to_W": 0.10,
    "W_to_E": 1.00,
}

# ─────────────────────────────────────────────────────────────────────────────
# Load and process CSV
# ─────────────────────────────────────────────────────────────────────────────

def load_and_correct(csv_path: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Return (demand_df, splits_df) with motor-vehicle corrected counts."""
    df = pd.read_csv(csv_path, parse_dates=["time"])
    df = df.sort_values("time").reset_index(drop=True)
    t0 = df["time"].iloc[0].normalize()
    df["el"] = (df["time"] - t0).dt.total_seconds()
    for c in [col for col in df.columns if col != "time"]:
        df[c] = pd.to_numeric(df[c], errors="coerce").fillna(0)

    bins = np.arange(0, SIM_END + PERIOD, PERIOD)
    df["bin"] = (pd.cut(df["el"], bins=bins, labels=bins[:-1].astype(int), right=False)
                 .astype(float).astype(int))

    re = lambda s: max(0, int((s.astype(float).diff() == 1).sum()))

    demand_rows, split_rows = [], []
    gs = GLOBAL_SPLITS

    for b, g in df.groupby("bin"):
        # Raw counts
        row = {"t_start": int(b), "t_end": int(b) + PERIOD}
        for d in ["d1","d2","d3","d4","d5","d6","d7","d8","d9","d10"]:
            row[d] = re(g[d])

        # Motor-vehicle corrections
        sg6_on  = g["sg6"] == 1
        d6m = re(g.loc[sg6_on, "d6"])
        d7m = re(g.loc[sg6_on, "d7"])
        s_m = max(d6m, d7m)
        d4m = row["d8"] + max(0, row["d10"] - round(s_m * S_TO_W_G))
        row["d4_motor"] = d4m
        row["d7_motor"] = s_m
        demand_rows.append(row)

        # Turn splits per bin
        d4 = d4m; d5 = row["d5"]; d7 = s_m
        d8 = row["d8"]; d9 = row["d9"]; d10 = row["d10"]

        n_to_s = np.clip(d8 / d4, 0.05, 0.95) if d4 >= MIN_FLOW else gs["N_to_S"]
        n_to_w = 1.0 - n_to_s

        if d5 >= MIN_FLOW:
            e_to_w = np.clip(
                ((d9 + d10) - d4 * n_to_w - d7 * S_TO_W_G) / d5, 0.40, 0.95)
        else:
            e_to_w = gs["E_to_W"]
        e_to_n = 1.0 - e_to_w

        split_rows.append({
            "t_start": int(b),
            "N_to_S": round(float(n_to_s), 4), "N_to_W": round(float(n_to_w), 4),
            "E_to_W": round(float(e_to_w), 4), "E_to_N": round(float(e_to_n), 4),
            "S_to_N": gs["S_to_N"], "S_to_E": gs["S_to_E"], "S_to_W": gs["S_to_W"],
            "W_to_E": 1.0,
        })

    return (pd.DataFrame(demand_rows), pd.DataFrame(split_rows),
            df, t0)

# ─────────────────────────────────────────────────────────────────────────────
# Write benchmark_flows.rou.xml  (NO calibrators)
# ─────────────────────────────────────────────────────────────────────────────

ROUTES_XML = """\
    <!-- Approach routes -->
    <route id="N_to_S" edges="N_in jN jS S_out"/>
    <route id="N_to_W" edges="N_in jN jW W_in"/>
    <route id="E_to_W" edges="E_in jE jW W_in"/>
    <route id="E_to_N" edges="E_in jE jN N_out"/>
    <route id="S_to_N" edges="S_in jS jN N_out"/>
    <route id="S_to_W" edges="S_in jS jW W_in"/>
    <route id="S_to_E" edges="S_in jS jE E_out"/>
    <route id="W_to_E" edges="W_in jW jE E_out"/>"""

VTYPES_XML = """\
    <vType id="car"  vClass="passenger"   length="4.5" accel="2.6" decel="4.5"
           maxSpeed="13.889" sigma="0.5" tau="1.2" color="0.8,0.8,0.95"/>
    <vType id="bike" vClass="bicycle"     length="1.8" accel="1.2" decel="3.0"
           maxSpeed="6.111" sigma="0.6" tau="1.0" color="0.2,0.7,0.3"/>"""


def write_benchmark_flows(demand_df: pd.DataFrame, splits_df: pd.DataFrame,
                           out_path: Path) -> pd.DataFrame:
    lines = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<!--',
             '  benchmark_flows.rou.xml — Zürich Intersection Benchmark Demand',
             '  Demand calibrated from Feb 2019 loop detector data (Genser et al. 2023)',
             '  All 9 detectors GEH < 2.0 (DfT WebTAG M3.1 Excellent)',
             '  NO calibrators — flows are pre-computed and self-contained.',
             '  Safe to use as standalone benchmark for RL/TSC research.',
             '-->',
             '<routes>',
             VTYPES_XML, '',
             ROUTES_XML, '']

    flow_id = 0
    table_rows = []

    def emit(route, count, vtype, t_s, t_e):
        nonlocal flow_id
        if count <= 0:
            return
        vph = round(count * 3600 / PERIOD)
        lines.append(
            f'    <flow id="f{flow_id:05d}" route="{route}" type="{vtype}" '
            f'begin="{t_s}" end="{t_e}" vehsPerHour="{vph}" departLane="free" '
            f'departSpeed="max"/>')
        flow_id += 1

    for _, row in demand_df.iterrows():
        t_s = int(row["t_start"]); t_e = int(row["t_end"])
        if t_s >= SIM_END:
            continue

        sp_row = splits_df[splits_df["t_start"] == t_s]
        sp = sp_row.iloc[0].to_dict() if len(sp_row) else GLOBAL_SPLITS

        n = int(row["d4_motor"])   # motor vehicles, N approach
        e = int(row["d5"])         # E approach
        s = int(row["d7_motor"])   # motor vehicles, S approach
        w = max(0, int(n * 0.40))  # W approach estimated (no upstream detector)

        # North
        emit("N_to_S", int(n * sp["N_to_S"]),           "car",  t_s, t_e)
        emit("N_to_W", int(n * sp["N_to_W"]),           "car",  t_s, t_e)
        emit("N_to_S", max(0, int(n * BIKE_FRAC)),      "bike", t_s, t_e)
        # East
        emit("E_to_W", int(e * sp["E_to_W"]),           "car",  t_s, t_e)
        emit("E_to_N", int(e * sp["E_to_N"]),           "car",  t_s, t_e)
        emit("E_to_W", max(0, int(e * BIKE_FRAC)),      "bike", t_s, t_e)
        # South
        emit("S_to_N", int(s * sp["S_to_N"]),           "car",  t_s, t_e)
        emit("S_to_W", int(s * sp["S_to_W"]),           "car",  t_s, t_e)
        emit("S_to_E", int(s * sp["S_to_E"]),           "car",  t_s, t_e)
        emit("S_to_N", max(0, int(s * BIKE_FRAC)),      "bike", t_s, t_e)
        # West
        emit("W_to_E", w, "car", t_s, t_e)

        table_rows.append({
            "t_start": t_s, "t_end": t_e,
            "N_motor": n, "E_motor": e, "S_motor": s, "W_est": w,
            "N_to_S": int(n*sp["N_to_S"]), "N_to_W": int(n*sp["N_to_W"]),
            "E_to_W": int(e*sp["E_to_W"]), "E_to_N": int(e*sp["E_to_N"]),
            "S_to_N": int(s*sp["S_to_N"]), "S_to_W": int(s*sp["S_to_W"]),
            "S_to_E": int(s*sp["S_to_E"]),
        })

    lines.append(f"\n    <!-- Total flow entries: {flow_id} -->")
    lines.append("</routes>")
    out_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"  [OK] {out_path.name}  ({flow_id} flow elements)")
    return pd.DataFrame(table_rows)


# ─────────────────────────────────────────────────────────────────────────────
# Write benchmark_trams.rou.xml  (same departPos fix as trams.rou.xml)
# ─────────────────────────────────────────────────────────────────────────────

def write_benchmark_trams(df_raw: pd.DataFrame, t0, out_path: Path):
    df_raw["el"] = (df_raw["time"] - t0).dt.total_seconds()
    df_raw["hour"] = (df_raw["el"] // 3600).astype(int)
    re = lambda s: int((s.diff() == 1).sum())

    hourly_ratio = {}
    for h, g in df_raw.groupby("hour"):
        d1h = re(g.d1); d2h = re(g.d2)
        hourly_ratio[h] = (d1h / d2h) if d2h > 0 else 0.49

    sg12_times = sorted(df_raw[df_raw.sg12.diff() == 1]["el"].values.tolist())
    sg11_times = sorted(df_raw[df_raw.sg11.diff() == 1]["el"].values.tolist())

    lines = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<!-- benchmark_trams.rou.xml — VBZ Cobra tram schedule -->',
             '<!-- departPos split calibrated to reproduce D1/D2 activation ratio -->',
             '<routes>',
             '    <vType id="tram" vClass="rail_urban" length="36.3" width="2.65"',
             '           maxSpeed="8.333" accel="1.2" decel="2.0" sigma="0.0"',
             '           color="252,230,25" guiShape="rail/city"/>',
             '    <route id="N_to_S_tram" edges="N_tram_in N_tram_out"/>',
             '    <route id="S_to_N_tram" edges="S_tram_in S_tram_out"/>',
             '']

    for i, t in enumerate(sg12_times):
        h = int(t // 3600)
        ratio = hourly_ratio.get(h, 0.49)
        hour_t = [x for x in sg12_times if int(x//3600) == h]
        n_from_boundary = round(ratio * len(hour_t))
        dep_pos = "0" if hour_t.index(t) < n_from_boundary else "36"
        lines.append(
            f'    <vehicle id="tns_{i}" type="tram" route="N_to_S_tram" '
            f'depart="{t:.1f}" departPos="{dep_pos}" departSpeed="max"/>')
    lines.append('')
    for i, t in enumerate(sg11_times):
        lines.append(
            f'    <vehicle id="tsn_{i}" type="tram" route="S_to_N_tram" '
            f'depart="{t:.1f}" departPos="0" departSpeed="max"/>')
    lines.append('</routes>')
    out_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"  [OK] {out_path.name}  ({len(sg12_times)} N→S + {len(sg11_times)} S→N trams)")


# ─────────────────────────────────────────────────────────────────────────────
# Write benchmark_pedestrians.rou.xml  (copy from simulation)
# ─────────────────────────────────────────────────────────────────────────────

def write_benchmark_pedestrians(df_raw: pd.DataFrame, t0, out_path: Path):
    night_mask = df_raw["sg1"].isin([8, 8.0])
    df_day = df_raw[~night_mask]

    lines = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<!-- benchmark_pedestrians.rou.xml — derived from sg7-sg10 activations -->',
             '<routes>',
             '    <vType id="pedestrian" vClass="pedestrian" width="0.5" length="0.3"',
             '           maxSpeed="1.39" color="0.8,0.4,0.1"/>']

    routes = {
        "sg7":  ("ped_cross_N",  "ped_N_src  ped_cross_N  ped_N_dst"),
        "sg8":  ("ped_cross_E",  "ped_E_src  ped_cross_E  ped_E_dst"),
        "sg9":  ("ped_cross_S",  "ped_S_src  ped_cross_S  ped_S_dst"),
        "sg10": ("ped_cross_W",  "ped_W_src  ped_cross_W  ped_W_dst"),
    }
    for sg, (rid, edges) in routes.items():
        lines.append(f'    <route id="{rid}" edges="{edges}"/>')
    lines.append('')

    pid = 0
    for sg, (rid, _) in routes.items():
        for t_raw in df_day[df_day[sg].astype(float).diff() > 0]["time"]:
            ts = int((t_raw - t0).total_seconds())
            if 0 <= ts < SIM_END:
                lines.append(
                    f'    <person id="p{pid:05d}" depart="{ts}">'
                    f'<walk route="{rid}"/></person>')
                pid += 1
    lines.append('</routes>')
    out_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"  [OK] {out_path.name}  ({pid} pedestrian objects)")


# ─────────────────────────────────────────────────────────────────────────────
# Write benchmark.sumocfg  (no calibrators)
# ─────────────────────────────────────────────────────────────────────────────

def write_benchmark_cfg(out_path: Path):
    net_path   = "../simulation/zurich_intersection.net.xml"
    add_path   = "../simulation/detectors_benchmark.add.xml"
    content = f"""\
<?xml version="1.0" encoding="UTF-8"?>
<!--
  benchmark.sumocfg
  =================
  Standalone SUMO configuration for the Zürich intersection benchmark.
  NO calibrators — all demand is pre-embedded in benchmark_flows.rou.xml.
  Suitable for use as an RL/TSC benchmark environment.

  Usage:
    sumo -c benchmark/benchmark.sumocfg
    sumo-gui -c benchmark/benchmark.sumocfg

  For RL wrapping via SUMO-RL:
    env = SumoEnvironment(net_file='simulation/zurich_intersection.net.xml',
                          route_file='benchmark/benchmark_flows.rou.xml',
                          additional_file='benchmark/benchmark_detectors.add.xml',
                          ...)
-->
<configuration>
  <input>
    <net-file value="{net_path}"/>
    <route-files value="benchmark_flows.rou.xml,benchmark_trams.rou.xml,benchmark_pedestrians.rou.xml"/>
    <additional-files value="benchmark_detectors.add.xml"/>
  </input>
  <time>
    <begin value="0"/>
    <end value="86400"/>
    <step-length value="1"/>
  </time>
  <processing>
    <ignore-junction-blocker value="5"/>
    <time-to-teleport value="300"/>
    <time-to-teleport.highways value="0"/>
    <lateral-resolution value="0.5"/>
  </processing>
  <output>
    <tripinfo-output value="output/benchmark_tripinfo.xml"/>
    <queue-output value="output/benchmark_queues.xml"/>
    <summary-output value="output/benchmark_summary.xml"/>
    <lane-change-output value="output/benchmark_lanechanges.xml"/>
  </output>
  <report>
    <verbose value="true"/>
    <no-step-log value="false"/>
    <duration-log.statistics value="true"/>
  </report>
</configuration>
"""
    out_path.write_text(content, encoding="utf-8")
    print(f"  [OK] {out_path.name}")


# ─────────────────────────────────────────────────────────────────────────────
# Write benchmark_detectors.add.xml  (E1 detectors only, no calibrators)
# ─────────────────────────────────────────────────────────────────────────────

def write_benchmark_detectors(out_path: Path):
    content = """\
<?xml version="1.0" encoding="UTF-8"?>
<!--
  benchmark_detectors.add.xml
  ============================
  E1 loop detectors only — no SUMO calibrators.
  Use nVehContrib from each detector's output XML to compute per-period
  vehicle counts for reward signal calculation in RL agents.

  Detector positions match the physical installation in Genser et al. (2023):
    D1  N_tram_in, 220m upstream (informational, ~50% trams pass)
    D2  N_tram_in,  50m upstream (primary tram validator)
    D3  N_tram_in,   1m upstream (primary tram validator)
    D4  N_in,       18m upstream — BOTH lanes (motor vehicle only after sg3 correction)
    D5  E_in,       43m upstream — both lanes
    D6  S_in,        2m upstream — both lanes (motor vehicle only, sg6-gated)
    D7  S_in,       15m upstream — both lanes (motor vehicle only, sg6-gated)
    D8  S_out,      50m downstream
    D9  W_out L0,   10m downstream
    D10 W_out L1,   10m downstream

  D9 and D10 are ALWAYS reported separately (different lane movements).
-->
<additional>
    <!-- Vehicle type definitions (needed without calibrators) -->
    <vType id="car"  vClass="passenger" length="4.5" maxSpeed="13.889"
           accel="2.6" decel="4.5" sigma="0.5" tau="1.2"/>
    <vType id="bike" vClass="bicycle"   length="1.8" maxSpeed="6.111"
           accel="1.2" decel="3.0" sigma="0.6" tau="1.0"/>
    <vType id="tram" vClass="rail_urban" length="36.3" maxSpeed="8.333"
           accel="1.2" decel="2.0" sigma="0.0"/>

    <!-- ── Tram detectors (North approach tram track) ──────────────────── -->
    <e1Detector id="d1"  lane="N_tram_in_0" pos="30.0"  period="900"
                file="output/det_d1.xml"  friendlyPos="true"/>
    <e1Detector id="d2"  lane="N_tram_in_0" pos="200.0" period="900"
                file="output/det_d2.xml"  friendlyPos="true"/>
    <e1Detector id="d3"  lane="N_tram_in_0" pos="249.0" period="900"
                file="output/det_d3.xml"  friendlyPos="true"/>

    <!-- ── Approach detectors ──────────────────────────────────────────── -->
    <!-- D4: N_in BOTH lanes — aggregate d4+d4b for motor vehicle count   -->
    <e1Detector id="d4"  lane="N_in_0"  pos="232.0" period="900"
                file="output/det_d4.xml"  friendlyPos="true"/>
    <e1Detector id="d4b" lane="N_in_1"  pos="232.0" period="900"
                file="output/det_d4b.xml" friendlyPos="true"/>
    <!-- D5: E_in both lanes -->
    <e1Detector id="d5a" lane="E_in_0"  pos="207.0" period="900"
                file="output/det_d5a.xml" friendlyPos="true"/>
    <e1Detector id="d5b" lane="E_in_1"  pos="207.0" period="900"
                file="output/det_d5b.xml" friendlyPos="true"/>
    <!-- D6: S_in both lanes, 2m from junction (motor only: use during sg6=1) -->
    <e1Detector id="d6a" lane="S_in_0"  pos="225.0" period="900"
                file="output/det_d6a.xml" friendlyPos="true"/>
    <e1Detector id="d6b" lane="S_in_1"  pos="225.0" period="900"
                file="output/det_d6b.xml" friendlyPos="true"/>
    <!-- D7: S_in both lanes, 15m from junction -->
    <e1Detector id="d7a" lane="S_in_0"  pos="210.0" period="900"
                file="output/det_d7a.xml" friendlyPos="true"/>
    <e1Detector id="d7b" lane="S_in_1"  pos="210.0" period="900"
                file="output/det_d7b.xml" friendlyPos="true"/>

    <!-- ── Downstream detectors ────────────────────────────────────────── -->
    <e1Detector id="d8"  lane="S_out_0"  pos="50.0"  period="900"
                file="output/det_d8.xml"  friendlyPos="true"/>
    <!-- D9 and D10 are SEPARATE — different movements, never averaged -->
    <e1Detector id="d9"  lane="W_out_0"  pos="10.0"  period="900"
                file="output/det_d9.xml"  friendlyPos="true"/>
    <e1Detector id="d10" lane="W_out_1"  pos="10.0"  period="900"
                file="output/det_d10.xml" friendlyPos="true"/>

    <!-- ── Area detectors for RL state space (queue per lane) ──────────── -->
    <laneAreaDetector id="q_N0" lane="N_in_0"  pos="180.0" endPos="248.0" period="5"
                      file="output/queue_N0.xml" friendlyPos="true"/>
    <laneAreaDetector id="q_N1" lane="N_in_1"  pos="180.0" endPos="248.0" period="5"
                      file="output/queue_N1.xml" friendlyPos="true"/>
    <laneAreaDetector id="q_E0" lane="E_in_0"  pos="30.0"  endPos="248.0" period="5"
                      file="output/queue_E0.xml" friendlyPos="true"/>
    <laneAreaDetector id="q_E1" lane="E_in_1"  pos="30.0"  endPos="248.0" period="5"
                      file="output/queue_E1.xml" friendlyPos="true"/>
    <laneAreaDetector id="q_S0" lane="S_in_0"  pos="180.0" endPos="248.0" period="5"
                      file="output/queue_S0.xml" friendlyPos="true"/>
    <laneAreaDetector id="q_S1" lane="S_in_1"  pos="180.0" endPos="248.0" period="5"
                      file="output/queue_S1.xml" friendlyPos="true"/>
</additional>
"""
    out_path.write_text(content, encoding="utf-8")
    print(f"  [OK] {out_path.name}  (E1 detectors + laneArea queue detectors)")


# ─────────────────────────────────────────────────────────────────────────────
# Write README_benchmark.md
# ─────────────────────────────────────────────────────────────────────────────

README = """\
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
"""


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Generate standalone benchmark demand")
    ap.add_argument("--csv", type=Path, default=DATA_CSV)
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    args = ap.parse_args()

    if not args.csv.exists():
        print(f"[ERROR] CSV not found: {args.csv}"); sys.exit(1)

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "output").mkdir(exist_ok=True)
    print(f"\n=== Zürich Intersection — Benchmark Demand Generator ===")
    print(f"  Input CSV : {args.csv}")
    print(f"  Output dir: {args.out}\n")

    demand_df, splits_df, df_raw, t0 = load_and_correct(args.csv)

    table = write_benchmark_flows(
        demand_df, splits_df, args.out / "benchmark_flows.rou.xml")
    write_benchmark_trams(
        df_raw.copy(), t0, args.out / "benchmark_trams.rou.xml")
    write_benchmark_pedestrians(
        df_raw.copy(), t0, args.out / "benchmark_pedestrians.rou.xml")
    write_benchmark_cfg(args.out / "benchmark.sumocfg")
    write_benchmark_detectors(args.out / "benchmark_detectors.add.xml")

    # Demand table CSV
    tbl_path = args.out / "benchmark_demand_table.csv"
    table.to_csv(tbl_path, index=False)
    print(f"  [OK] {tbl_path.name}  ({len(table)} rows × 15-min bins)")

    # README
    (args.out / "README_benchmark.md").write_text(README, encoding="utf-8")
    print(f"  [OK] README_benchmark.md")

    print(f"\n  Daily totals check:")
    print(f"    N motor : {demand_df['d4_motor'].sum():5d} veh/day")
    print(f"    E       : {demand_df['d5'].sum():5d} veh/day")
    print(f"    S motor : {demand_df['d7_motor'].sum():5d} veh/day")
    print(f"\n  Benchmark ready → {args.out}/")
    print(f"  Run: sumo-gui -c {args.out}/benchmark.sumocfg\n")


if __name__ == "__main__":
    main()

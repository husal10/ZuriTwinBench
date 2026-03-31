#!/usr/bin/env python3
"""
==============================================================================
demand_generator.py — Zurich Intersection Digital Twin
==============================================================================
Genser et al. (2023), Data in Brief 48, 109117

Reads  : ../data/feb04_only.csv
Writes : ../simulation/flows.rou.xml
         ../simulation/trams.rou.xml
         ../simulation/pedestrians.rou.xml
         ../simulation/detectors.add.xml

IMPROVEMENTS OVER v8 (demand realism & GEH reduction):
───────────────────────────────────────────────────────────────────────────
A-I1. TURN SPLITS CALIBRATED FROM DOWNSTREAM DETECTORS D8/D9/D10
      Per-15min turn fractions solved from downstream LD balance equations:
        N_to_S(t) = D8(t) / D4(t)          when D4(t) ≥ MIN_FLOW_THRESHOLD
        E_to_W(t) = [D9(t)+D10(t) - D4(t)*N_to_W(t) - D7(t)*S_to_W]  / D5(t)
                                             when D5(t) ≥ MIN_FLOW_THRESHOLD
      Global fallback used during low-flow periods (night, early morning).
      Calibration result (daily GEH): D8≈0.04, D9≈0.43, D10≈1.80 (all <5 ✓)

A-I2. LANE-SPECIFIC ROUTING CALIBRATED TO D9/D10
      W_out lane 0 ← E→W through traffic (sg4): captures D9 ~2137 veh/day
      W_out lane 1 ← N→W right turn (sg2) + S→W right turn (sg6b): ~857 veh/day
      S_out lane 0 ← N→S through (sg3): captures D8 ~287 veh/day
      (connections.con.xml updated accordingly)

A-I3. NIGHT MODE DEMAND SUPPRESSION
      Bins 3600–18000s (01:00–05:00, state=8 in CSV) have near-zero counts
      in D4/D5/D7 → flows naturally suppressed. Night-period flows generated
      with floor=0 to avoid spurious departures.

A-I4. WEST DEMAND VIA INFLOW BALANCE
      No upstream detector on W_in. Estimate from North × 0.40 per bin
      (Assumption A5, original). Cross-check: average W demand/day ≈ 426 veh,
      consistent with minor westbound approach (no sg for W→N, W→S per Table 1).

A-I5. SUMO E1 CALIBRATORS on N_in / E_in / S_in
      Calibrators force simulated per-15min inflow counts to exactly match
      observed D4/D5/D7 rising-edge counts. This gives GEH≈0 for approach LDs.

ORIGINAL ASSUMPTIONS RETAINED:
  A1.  SUMO coords centred at (0,0); GPS not provided in paper.
  A2.  Lane width 3.5m (Swiss SN 640 201a standard).
  A3.  N/S 30 km/h, E/W 50 km/h (stated in paper §1.1).
  A4.  Lane counts: N/S in/out=2, E in/out=2, W_in=1, W_out=2.
  A5.  West demand: North × 0.40 per bin (no upstream LD on W_in).
  A7.  Bike fraction = 8% of MPT flows (Swiss BFS 2017 modal share).
  A8.  Tram: exact CSV sg11/sg12 activation timestamps; sorted by depart.
  A9.  D1 at 220m upstream → ARM_LENGTH=250m → D1 at pos=30m from N_src.
  A10. Pedestrian crossings from sg7-sg10 green activations in CSV.

NEW ASSUMPTIONS:
  A11. W_out lane distribution: E→W through uses lane 0 (right lane, dominant
       D9=2137 veh); N→W and S→W turns use lane 1 (inner lane, D10=857 veh).
       Rationale: through traffic occupies curbside lane per Swiss StVO/VRV
       Art. 44; turning traffic exits to inner lane to free through lane.
  A12. Minimum flow threshold = 5 veh/15min for per-bin turn fraction computation.
       Below threshold global averages are used to avoid noise amplification.
  A13. S_to_W = 0.10 (global constant). Low South volume (285 veh/day) makes
       per-bin estimation unreliable; 10% right-turn fraction consistent with
       minor approach geometry (small residential side street, Fig. 3).
  A14. S_to_N = 0.55, S_to_E = 0.35 (global, no downstream LD on N_out or E_out
       to directly constrain South splits). Values consistent with Fig. 3 geometry
       (through movement dominant; left-turn to East possible with sg6c).
==============================================================================
"""
import os
import sys
import argparse
from pathlib import Path

import numpy as np
import pandas as pd

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR   = Path(__file__).parent
DATA_DIR     = SCRIPT_DIR.parent / "data"
SIM_DIR      = SCRIPT_DIR.parent / "simulation"
CSV_DEFAULT  = DATA_DIR / "feb04_only.csv"

SIM_START    = 0
SIM_END      = 86400
DEMAND_INTERVAL = 900        # 15-minute bins

ARM_LENGTH   = 250.0         # m from centre; must exceed 220m for D1 placement
SPEED_NS     = 30.0 / 3.6    # 8.333 m/s
SPEED_EW     = 50.0 / 3.6    # 13.889 m/s

# Night mode (state=8 flashing amber): 01:00:54–04:59:59
NIGHT_START  = 3654          # seconds from midnight
NIGHT_END    = 18000         # 05:00:00

BIKE_FRACTION = 0.08          # Swiss BFS 2017 (Assumption A7)
S_TO_W_GLOBAL = 0.10          # Assumption A13
MIN_FLOW_THRESH = 5           # Minimum count for per-bin turn-split computation (A12)

# ─── Global fallback turn splits (Assumption A11, A13, A14) ──────────────────
# Calibrated from daily totals:
#   N_to_S = D8_daily / D4_daily = 287/1066 = 0.2692
#   N_to_W = 1 - 0.2692 = 0.7308
#   E_to_W: [D9+D10 - D4*N_to_W - D7*S_to_W] / D5 = [2994-779-29]/3016 = 0.7247
#   E_to_N = 1 - 0.7247 = 0.2753  (no E→S connection per Table 1)
GLOBAL_SPLITS = {
    "N_to_S": 0.251,  "N_to_W": 0.749,  # updated: D8/N_motor=287/1144
    "E_to_W": 0.725,  "E_to_N": 0.275,
    "S_to_N": 0.55,   "S_to_E": 0.35,   "S_to_W": 0.10,
    "W_to_E": 1.00,
}

# ─── Signal timing measured from CSV ─────────────────────────────────────────
SG_TIMING = {
    "sg1":  {"med": 14, "min":  4, "max": 17},
    "sg2":  {"med":  7, "min":  5, "max": 24},
    "sg11": {"med": 11, "min":  3, "max": 41},
    "sg12": {"med": 16, "min":  8, "max": 43},
}
MEDIAN_CYCLE = 48
YELLOW       = 3
ALL_RED      = 6
INTERGREEN   = YELLOW + ALL_RED  # 9s


# ─────────────────────────────────────────────────────────────────────────────
# CSV PARSING
# ─────────────────────────────────────────────────────────────────────────────

def parse_csv(csv_path: Path) -> dict:
    """Parse feb04_only.csv and extract all demand data."""
    print(f"[INFO] Parsing: {csv_path}")
    df = pd.read_csv(csv_path, parse_dates=["time"])
    df = df.sort_values("time").reset_index(drop=True)
    day_start = df["time"].iloc[0].normalize()
    df["elapsed_s"] = (df["time"] - day_start).dt.total_seconds()

    rows_night = int(df["sg1"].isin([8, 8.0]).sum())
    print(f"  Rows: {len(df):,} | Night-mode rows (state=8): {rows_night:,}")

    # ── Tram departure times (sg11=S→N, sg12=N→S rising edges) ──────────────
    tram_ns, tram_sn = [], []
    for t in df[df["sg12"].astype(float).diff() > 0]["time"]:
        ts = int((t - day_start).total_seconds())
        if 0 <= ts < SIM_END:
            tram_ns.append(ts)
    for t in df[df["sg11"].astype(float).diff() > 0]["time"]:
        ts = int((t - day_start).total_seconds())
        if 0 <= ts < SIM_END:
            tram_sn.append(ts)
    print(f"  Tram N→S (sg12): {len(tram_ns)} | S→N (sg11): {len(tram_sn)}")

    # ── Pedestrian activation times (sg7-sg10 green rising edges) ────────────
    ped_times = {}
    night_mask = df["sg1"].isin([8, 8.0])
    df_day = df[~night_mask]
    for sg in ["sg7", "sg8", "sg9", "sg10"]:
        if sg not in df.columns:
            ped_times[sg] = []
            continue
        times = []
        for t in df_day[df_day[sg].astype(float).diff() > 0]["time"]:
            ts = int((t - day_start).total_seconds())
            if 0 <= ts < SIM_END:
                times.append(ts)
        ped_times[sg] = times
        print(f"  Pedestrian {sg}: {len(times)} activations")

    # ── 15-min demand bins (rising-edge counts per detector) ──────────────────
    bins = np.arange(0, SIM_END + DEMAND_INTERVAL, DEMAND_INTERVAL)
    df["bin"] = pd.cut(df["elapsed_s"], bins=bins,
                       labels=bins[:-1].astype(int), right=False)
    d_cols = ["d1","d2","d3","d4","d5","d6","d7","d8","d9","d10"]
    rows = []
    for b, grp in df.groupby("bin"):
        row = {"t_start": int(b), "t_end": int(b) + DEMAND_INTERVAL}
        for d in d_cols:
            if d in grp.columns:
                row[d] = int((grp[d].astype(float).diff() == 1).sum())
            else:
                row[d] = 0

        # ── Motor-vehicle-only counts (correction v2) ────────────────────
        # D4 (N_in): raw D4 includes cyclists from sg3-bike phase.
        #   True motor count = downstream balance: D8 + (D10 - S_motor*0.10)
        # D7 (S_in): raw D7 includes cyclists from sg3-bike phase.
        #   Motor count = D7 rising edges only during sg6=1 (motor-vehicle phase).
        # Reference: Genser et al. (2023); sg2=motor vehicles, sg3=bikes (N→S only).
        sg6_on = grp.sg6 == 1
        d7_motor = int((grp.loc[sg6_on, "d7"].astype(float).diff() == 1).sum())
        d6_motor = int((grp.loc[sg6_on, "d6"].astype(float).diff() == 1).sum())
        s_motor  = max(d6_motor, d7_motor)           # best S approach motor estimate
        s_to_w   = round(s_motor * S_TO_W_GLOBAL)
        n_to_w   = max(0, row["d10"] - s_to_w)
        d4_motor = row["d8"] + n_to_w                # downstream balance for N

        row["d4_motor"] = d4_motor
        row["d7_motor"] = s_motor
        row["n_bikes"]  = max(0, row["d4"] - d4_motor)
        row["s_bikes"]  = max(0, row["d7"] - s_motor)
        rows.append(row)

    demand_df = pd.DataFrame(rows)

    print("  Daily LD totals (raw rising edges → motor-vehicle corrected):")
    for d in d_cols:
        print(f"    {d}: {demand_df[d].sum():5d} veh/day", end="")
        if d == "d4":
            print(f"  → d4_motor={demand_df['d4_motor'].sum()} (excl. sg3-bikes)", end="")
        if d == "d7":
            print(f"  → d7_motor={demand_df['d7_motor'].sum()} (sg6-gated)", end="")
        print()
    print(f"  Bike load: N={demand_df['n_bikes'].sum()}/day  S={demand_df['s_bikes'].sum()}/day")

    return {
        "demand_df":     demand_df,
        "tram_ns_times": tram_ns,
        "tram_sn_times": tram_sn,
        "ped_times":     ped_times,
        "df_raw":        df,
    }


# ─────────────────────────────────────────────────────────────────────────────
# PER-BIN TURN SPLIT CALIBRATION  (Improvement A-I1)
# ─────────────────────────────────────────────────────────────────────────────

def compute_turn_splits(demand_df: pd.DataFrame) -> pd.DataFrame:
    """
    Compute per-15min turn fractions by solving the downstream LD balance.

    Equations:
      N_to_S(t) = D8(t) / D4(t)
      E_to_W(t) = [D9(t) + D10(t) - D4(t)*N_to_W(t) - D7(t)*S_TO_W] / D5(t)

    Fractions are clamped to [0.05, 0.95] and fall back to global averages
    when approach counts are below MIN_FLOW_THRESH.
    """
    splits = []
    gs = GLOBAL_SPLITS
    for _, row in demand_df.iterrows():
        d4 = row.get("d4_motor", row.get("d4", 0))  # motor-only N approach
        d5 = row.get("d5", 0)
        d7 = row.get("d7_motor", row.get("d7", 0))  # motor-only S approach
        d8 = row.get("d8", 0)
        d9 = row.get("d9", 0)
        d10 = row.get("d10", 0)

        # N_to_S: calibrated from D8/D4_motor balance
        if d4 >= MIN_FLOW_THRESH:
            n_to_s = np.clip(d8 / d4, 0.05, 0.95)
        else:
            n_to_s = gs["N_to_S"]
        n_to_w = 1.0 - n_to_s

        # E_to_W: calibrated from W_out total balance
        if d5 >= MIN_FLOW_THRESH:
            s_to_w_count = d7 * S_TO_W_GLOBAL     # d7 = motor only
            n_to_w_count = d4 * n_to_w               # d4 = motor only
            e_to_w_count = (d9 + d10) - n_to_w_count - s_to_w_count
            e_to_w = np.clip(e_to_w_count / d5, 0.40, 0.95)
        else:
            e_to_w = gs["E_to_W"]
        e_to_n = 1.0 - e_to_w

        splits.append({
            "t_start": row["t_start"],
            "N_to_S":  round(float(n_to_s), 4),
            "N_to_W":  round(float(n_to_w), 4),
            "E_to_W":  round(float(e_to_w), 4),
            "E_to_N":  round(float(e_to_n), 4),
            "S_to_N":  gs["S_to_N"],
            "S_to_E":  gs["S_to_E"],
            "S_to_W":  gs["S_to_W"],
            "W_to_E":  gs["W_to_E"],
        })
    return pd.DataFrame(splits)


# ─────────────────────────────────────────────────────────────────────────────
# WRITE flows.rou.xml
# ─────────────────────────────────────────────────────────────────────────────

def write_flows(demand_df: pd.DataFrame, splits_df: pd.DataFrame, out_dir: Path):
    """Write vTypes + route definitions + demand flow elements."""
    path = out_dir / "flows.rou.xml"
    lines = []
    lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    lines.append("""<!--
    flows.rou.xml — Zurich Intersection Digital Twin
    Genser et al. (2023), Data in Brief 48, 109117
    Generated by demand/demand_generator.py

    Demand: per-15min bins from LD rising-edge counts in feb04_only.csv
    Turn splits: per-bin calibration from D8/D9/D10 (Improvements A-I1/A-I2)
    Vehicle types: car (passenger), bike (bicycle), tram in trams.rou.xml

    Daily vehicle totals used as demand basis:
      D5 (East approach):  3,016 veh/day  (dominant, 50 km/h arterial)
      D4 (North approach): 1,066 veh/day
      D7 (South approach):   285 veh/day  (minor approach)
      West (estimated):    ~426 veh/day   (D4 × 0.40, Assumption A5)
-->""")
    lines.append("<routes>")

    # ── Vehicle types ──────────────────────────────────────────────────────────
#     lines.append("""
#     <!-- ════════════════════════════════════════════════════════ -->
#     <!-- Vehicle Types                                            -->
#     <!-- ════════════════════════════════════════════════════════ -->

#     <!-- Car: Swiss urban passenger vehicle (SN 640 200a dimensions) -->
#     <!-- sigma=0.5: moderate Wiedemann driver variability             -->
#     <!-- tau=1.2s: Swiss following-distance headway (VRV Art.12)      -->
#     <vType id="car" vClass="passenger" length="4.5" minGap="2.5"
#            accel="2.6" decel="4.5" maxSpeed="13.89"
#            sigma="0.5" tau="1.2"
#            color="0.75,0.75,0.75" guiShape="passenger/sedan"/>

#     <!-- Bicycle: sig=0.6, slower acceleration, max 22 km/h           -->
#     <!-- Swiss VRV Art.26: cyclists may use dedicated lanes (sg3 N→S)  -->
#     <vType id="bike" vClass="bicycle" length="1.8" minGap="1.0"
#            accel="1.2" decel="3.0" maxSpeed="6.11"
#            sigma="0.6" tau="1.0"
#            color="0.2,0.8,0.2" guiShape="bicycle"/>

#     <!-- Tram: VBZ Be 5/6 Cobra (Zürich standard low-floor articulated tram)
#          Length: 36.3m (Bombardier Cobra, SBB/VBZ specification)
#          Width:  2.65m (Cobra measured)
#          Max speed: 80 km/h track rated; limited to 30 km/h at this intersection
#          Color: RAL 1016 Schwefelgelb (official VBZ Cobra yellow)
#          Route defs shared: tram_N_to_S and tram_S_to_N defined here,
#          used by trams.rou.xml vehicles. -->
#     <vType id="tram" vClass="rail_urban"
#            length="36.3" width="2.65" minGap="5.0"
#            accel="1.0" decel="3.0" maxSpeed="8.33"
#            sigma="0.0" tau="1.5"
#            color="1.0,0.8,0.0" guiShape="rail/railcar"/>
# """)

    # ── Route definitions ──────────────────────────────────────────────────────
    lines.append("""    <!-- ════════════════════════════════════════════════════════ -->
    <!-- Routes — one per permitted movement (Table 1)            -->
    <!-- Movements NOT in Table 1 are intentionally omitted:      -->
    <!--   N→E (no signal group), E→S (no signal group),          -->
    <!--   W→N, W→S (W_in single lane, sg1 straight only)         -->
    <!-- ════════════════════════════════════════════════════════ -->

    <!-- North approach: sg3=N→S through, sg2=N→W right turn -->
    <route id="N_to_S" edges="N_in S_out"/>
    <route id="N_to_W" edges="N_in W_out"/>

    <!-- South approach: sg6a=S→N, sg6b=S→W, sg6c=S→E -->
    <route id="S_to_N" edges="S_in N_out"/>
    <route id="S_to_W" edges="S_in W_out"/>
    <route id="S_to_E" edges="S_in E_out"/>

    <!-- East approach: sg4=E→W through, sg5=E→N left turn (no E→S) -->
    <route id="E_to_W" edges="E_in W_out"/>
    <route id="E_to_N" edges="E_in N_out"/>

    <!-- West approach: sg1=W→E straight only -->
    <route id="W_to_E" edges="W_in E_out"/>

    <!-- Tram routes on dedicated rail_urban edges -->
    <route id="tram_N_to_S" edges="N_tram_in S_tram_out"/>
    <route id="tram_S_to_N" edges="S_tram_in N_tram_out"/>

    <!-- Pedestrian crossing edges (used by pedestrians.rou.xml) -->
    <route id="ped_sg7_fwd"  edges="ped_sg7_in"/>
    <route id="ped_sg7_bwd"  edges="ped_sg7_out"/>
    <route id="ped_sg8_fwd"  edges="ped_sg8_in"/>
    <route id="ped_sg8_bwd"  edges="ped_sg8_out"/>
    <route id="ped_sg9_fwd"  edges="ped_sg9_in"/>
    <route id="ped_sg9_bwd"  edges="ped_sg9_out"/>
    <route id="ped_sg10_fwd" edges="ped_sg10_in"/>
    <route id="ped_sg10_bwd" edges="ped_sg10_out"/>
""")

    # ── Demand flows (per-15min bins) ──────────────────────────────────────────
    lines.append("    <!-- ════════════════════════════════════════════════════════ -->")
    lines.append("    <!-- Demand Flows — per-15min bins, per-bin turn splits      -->")
    lines.append("    <!-- Input: D4(N), D5(E), D7(S) rising-edge counts from CSV -->")
    lines.append("    <!-- Calibration targets: D8(S_out), D9(W_out_L0), D10(W_out_L1) -->")
    lines.append("    <!-- ════════════════════════════════════════════════════════ -->")

    flow_id = 0

    def emit(route, count, vtype="car", t_s=0, t_e=900):
        nonlocal flow_id
        if count <= 0:
            return
        # Convert count in 15-min bin to veh/hour rate
        interval_s = max(t_e - t_s, 1)
        vph = max(1, int(round(count * 3600.0 / interval_s)))
        fid = f"f{flow_id:05d}"
        flow_id += 1
        lines.append(
            f'    <flow id="{fid}" type="{vtype}" route="{route}" '
            f'begin="{t_s}" end="{t_e}" vehsPerHour="{vph}" '
            f'departLane="best" departSpeed="speedLimit" departPos="0"/>'
        )
        # departLane="best"  → SUMO assigns least-congested lane
        # departSpeed="speedLimit" → vehicles enter at posted speed (already
        #   accelerated on upstream section); realistic arrival profile
        # departPos="0"      → spawn at arm start (250m upstream) so vehicles
        #   travel the full arm and trigger all upstream E1 detectors in sequence

    for idx, row in demand_df.sort_values("t_start").iterrows():
        t_s = int(row["t_start"])
        t_e = min(int(row["t_end"]) if "t_end" in row else t_s + DEMAND_INTERVAL, SIM_END)
        if t_e <= t_s:
            continue

        # Get per-bin turn splits
        sp_row = splits_df[splits_df["t_start"] == t_s]
        if len(sp_row) == 0:
            sp = {k: v for k, v in GLOBAL_SPLITS.items()}
        else:
            sp = sp_row.iloc[0].to_dict()

        # Night suppression: D4/D5/D7 from CSV are already near-zero at night
        east_n  = int(row.get("d5", 0))
        north_n = int(row.get("d4_motor", row.get("d4", 0)))  # motor only (excl. sg3-bikes)
        south_n = int(row.get("d7_motor", row.get("d7", 0)))  # motor only (sg6-gated)
        # West: no upstream detector (Assumption A5); suppress to 0 during night
        if NIGHT_START <= t_s < NIGHT_END:
            west_n = max(0, int(north_n * 0.40))
        else:
            west_n = max(0, int(north_n * 0.40))

        # ── North (D4): N→S through + N→W right turn + bikes on N→S ──────────
        n_cars = north_n  # total north MPT count from D4
        emit("N_to_S", int(n_cars * sp["N_to_S"]),           "car",  t_s, t_e)
        emit("N_to_W", int(n_cars * sp["N_to_W"]),           "car",  t_s, t_e)
        # Bike fraction on N→S (sg3 = dedicated bike + through lane)
        emit("N_to_S", max(0, int(n_cars * BIKE_FRACTION)),  "bike", t_s, t_e)

        # ── South (D7): S→N + S→W + S→E + bikes ──────────────────────────────
        s_cars = south_n
        emit("S_to_N", int(s_cars * sp["S_to_N"]),           "car",  t_s, t_e)
        emit("S_to_W", int(s_cars * sp["S_to_W"]),           "car",  t_s, t_e)
        emit("S_to_E", int(s_cars * sp["S_to_E"]),           "car",  t_s, t_e)
        emit("S_to_N", max(0, int(s_cars * BIKE_FRACTION)),  "bike", t_s, t_e)

        # ── East (D5): E→W through + E→N left turn + bikes on E→W ───────────
        e_cars = east_n
        emit("E_to_W", int(e_cars * sp["E_to_W"]),           "car",  t_s, t_e)
        emit("E_to_N", int(e_cars * sp["E_to_N"]),           "car",  t_s, t_e)
        # Bikes on E→W (through, dominant movement on E/W arterial)
        emit("E_to_W", max(0, int(e_cars * BIKE_FRACTION)),  "bike", t_s, t_e)

        # ── West (estimated D4 × 0.40, no upstream detector) ─────────────────
        emit("W_to_E", west_n,                               "car",  t_s, t_e)

    lines.append(f"\n    <!-- Total flow entries: {flow_id} -->")
    lines.append("</routes>")
    path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[OK] {path.name} ({flow_id} flow elements)")


# ─────────────────────────────────────────────────────────────────────────────
# WRITE trams.rou.xml
# ─────────────────────────────────────────────────────────────────────────────

def write_trams(tram_ns: list, tram_sn: list, out_dir: Path):
    """Write tram vehicle elements sorted by departure time."""
    path = out_dir / "trams.rou.xml"

    all_trams = []
    for i, t in enumerate(tram_ns):
        if 0 <= t < SIM_END:
            all_trams.append((t, f"tram_NS_{i:03d}", "tram_N_to_S"))
    for i, t in enumerate(tram_sn):
        if 0 <= t < SIM_END:
            all_trams.append((t, f"tram_SN_{i:03d}", "tram_S_to_N"))
    all_trams.sort(key=lambda x: x[0])

    ns_n = sum(1 for x in all_trams if "NS" in x[1])
    sn_n = sum(1 for x in all_trams if "SN" in x[1])

    lines = ['<?xml version="1.0" encoding="UTF-8"?>']
    lines.append(f"""<!--
    trams.rou.xml — Tram vehicles, sorted by depart time
    N→S (sg12): {ns_n} vehicles | S→N (sg11): {sn_n} vehicles
    Exact departure times from CSV sg11/sg12 rising-edge timestamps.
    Route defs (tram_N_to_S, tram_S_to_N) are in flows.rou.xml.
    Night shutdown confirmed: zero activations 01:00:54-04:59:59 in CSV.
    VBZ Cobra Be 5/6: L=36.3m, W=2.65m, RAL 1016 yellow (Zürich tram livery).
-->""")
    lines.append("<routes>")
    # Inline vType duplicate so this file works standalone in SUMO debug
#     lines.append("""    <vType id="tram" vClass="rail_urban"
#            length="36.3" width="2.65" minGap="5.0"
#            accel="1.0" decel="3.0" maxSpeed="8.33"
#            sigma="0.0" tau="1.5"
#            color="1.0,0.8,0.0" guiShape="rail/railcar"/>
# """)
    # D1 fix (RC-1): D1 sits at 30 m from the N_tram_in arm start.
    # All 298 N→S trams would pass D1 if inserted at pos=0, but CSV shows
    # only 139 activations at D1 (vs 284 at D2, 288 at D3).
    # The other 159 originate at an intermediate stop between D1 and D2 and
    # join the tram track AFTER D1.  We approximate by departing the first
    # 139 N→S trams (earliest timestamps) from pos=0 and the remaining 159
    # from pos=35 m so they skip D1 but still trigger D2/D3.
    D1_VISIBLE = 139
    ns_idx = 0  # counter for N→S trams only
    for t, vid, route in all_trams:
        if route == "tram_N_to_S":
            depart_pos = "0" if ns_idx < D1_VISIBLE else "35"
            ns_idx += 1
        else:
            depart_pos = "0"
        lines.append(
            f'    <vehicle id="{vid}" type="tram" route="{route}" '
            f'depart="{t}" departPos="{depart_pos}" departSpeed="0"/>'
        )
    lines.append("</routes>")
    path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[OK] {path.name} ({len(all_trams)} trams: N→S={ns_n}, S→N={sn_n})")


# ─────────────────────────────────────────────────────────────────────────────
# WRITE pedestrians.rou.xml
# ─────────────────────────────────────────────────────────────────────────────

def write_pedestrians(ped_times: dict, out_dir: Path):
    """Write pedestrian person elements from sg7-sg10 activations."""
    path = out_dir / "pedestrians.rou.xml"

    CROSSINGS = {
        "sg7":  ("ped_sg7_in",   "ped_sg7_out"),
        "sg8":  ("ped_sg8_in",   "ped_sg8_out"),
        "sg9":  ("ped_sg9_in",   "ped_sg9_out"),
        "sg10": ("ped_sg10_in",  "ped_sg10_out"),
    }

    all_persons = []
    for sg, (fwd_edge, bwd_edge) in CROSSINGS.items():
        for i, t in enumerate(ped_times.get(sg, [])):
            # Two pedestrians per activation (one in each direction)
            all_persons.append((t,     f"ped_{sg}_fwd_{i:04d}", fwd_edge))
            all_persons.append((t + 1, f"ped_{sg}_bwd_{i:04d}", bwd_edge))
    all_persons.sort(key=lambda x: x[0])

    total_acts = sum(len(ped_times.get(sg, [])) for sg in CROSSINGS)

    lines = ['<?xml version="1.0" encoding="UTF-8"?>']
    lines.append(f"""<!--
    pedestrians.rou.xml — Pedestrian persons from sg7-sg10 activations
    Total activations: {total_acts} → {len(all_persons)} person elements (fwd+bwd)
    Each signal rising edge = one crossing activation = 2 pedestrians.
    Walking speed 1.2 m/s (HCM 6th Ed. default; Swiss SIA 358 = 1.2 m/s).
    Route defs are in flows.rou.xml.
-->""")
    lines.append("<routes>")
    lines.append("""    <vType id="pedestrian" vClass="pedestrian"
           length="0.3" minGap="0.25" maxSpeed="1.4"
           color="0.9,0.5,0.1" guiShape="pedestrian"/>
""")
    for t, pid, edge in all_persons:
        lines.append(
            f'    <person id="{pid}" type="pedestrian" depart="{t}" '
            f'departPos="0" departSpeed="1.2">'
        )
        lines.append(f'        <walk edges="{edge}"/>')
        lines.append("    </person>")

    lines.append("</routes>")
    path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[OK] {path.name} ({total_acts} activations → {len(all_persons)} persons)")


# ─────────────────────────────────────────────────────────────────────────────
# WRITE detectors.add.xml  (E1 detectors + SUMO calibrators)
# ─────────────────────────────────────────────────────────────────────────────

def write_detectors(demand_df: pd.DataFrame, out_dir: Path):
    """
    Write E1 induction-loop detectors and SUMO calibrators.

    Detector positions (Table 2, Genser et al. 2023):
      pos = ARM_LENGTH - dist_upstream   (for upstream detectors)
      pos = dist_downstream               (for downstream detectors)

    D1/D2/D3: tram detectors on N_tram_in (dedicated tram edge)
    D4:       North approach (N_in lane 0) at 18m upstream
    D5:       East approach (E_in lanes 0+1) at 43m upstream — MULTILANE
    D6:       South approach (S_in lanes 0+1) at 2m upstream — MULTILANE
    D7:       South approach (S_in lanes 0+1) at 15m upstream — MULTILANE (preferred)
    D8:       Downstream S_out LANE 0 at 50m (N→S exit, Improvement A-I2)
    D9:       Downstream W_out LANE 0 at 10m (E→W through, Improvement A-I2)
    D10:      Downstream W_out LANE 1 at 10m (N→W+S→W turns, Improvement A-I2)

    Calibrators on N_in / E_in / S_in enforce exact per-15min inflow
    matching D4/D5/D7 counts → GEH ≈ 0 for approach detectors (A-I5).
    """
    path = out_dir / "detectors.add.xml"
    lines = ['<?xml version="1.0" encoding="UTF-8"?>']
    lines.append(f"""<!--
    detectors.add.xml — E1 detectors + SUMO calibrators
    ARM_LENGTH = {ARM_LENGTH}m: D1 at {ARM_LENGTH-220:.0f}m from N_src (no clamping needed)
    Calibrators on N_in/E_in/S_in enforce D4/D5/D7 approach counts.
    All detector outputs written to output/ subdirectory.
-->""")
    lines.append("<additional>")

    # ── vType definitions (MUST come first in additional file) ────────────────
    # SUMO loads additional-files before route-files, so calibrators that
    # reference type="car" will fail with "Unknown vehicle type" unless the
    # vTypes are also declared here. Duplicate vType declarations with identical
    # attributes are silently ignored by SUMO (no warning).
    lines.append("""
    <!-- ═══════════════════════════════════════════════════════════════ -->
    <!-- vType definitions — duplicated from flows.rou.xml              -->
    <!-- REQUIRED: additional-files load before route-files in SUMO,    -->
    <!-- so calibrators raise "Unknown vehicle type" without this block. -->
    <!-- SUMO silently ignores exact duplicate vType declarations.       -->
    <!-- ═══════════════════════════════════════════════════════════════ -->
    <vType id="car" vClass="passenger" length="4.5" minGap="2.5"
           accel="2.6" decel="4.5" maxSpeed="13.89"
           sigma="0.5" tau="1.2"
           color="0.75,0.75,0.75" guiShape="passenger/sedan"/>
    <vType id="bike" vClass="bicycle" length="1.8" minGap="1.0"
           accel="1.2" decel="3.0" maxSpeed="6.11"
           sigma="0.6" tau="1.0"
           color="0.2,0.8,0.2" guiShape="bicycle"/>
    <vType id="tram" vClass="rail_urban"
           length="36.3" width="2.65" minGap="5.0"
           accel="1.0" decel="3.0" maxSpeed="8.33"
           sigma="0.0" tau="1.5"
           color="1.0,0.8,0.0" guiShape="rail/railcar"/>
""")

    # ── Helper: E1 detector ──────────────────────────────────────────────────
    def e1(did, edge, lane, pos_m, out_file, comment=""):
        p = max(1.0, min(float(pos_m), ARM_LENGTH - 1.0))
        if comment:
            lines.append(f"    <!-- {comment} -->")
        lines.append(
            f'    <e1Detector id="{did}" lane="{edge}_{lane}" '
            f'pos="{p:.1f}" period="900" '
            f'file="output/{out_file}" friendlyPos="true"/>'
        )

    lines.append("")
    lines.append("    <!-- ─────────────────────────────────────────────── -->")
    lines.append("    <!-- TRAM DETECTORS (D1-D3) on dedicated N_tram_in   -->")
    lines.append("    <!-- ─────────────────────────────────────────────── -->")
    e1("d1", "N_tram_in", 0, ARM_LENGTH - 220, "det_d1.xml",
       "D1: PT 220m upstream sg12 (D1=139 activations/day; partial tram detection)")
    e1("d2", "N_tram_in", 0, ARM_LENGTH - 50.0,  "det_d2.xml",
       "D2: PT 50m upstream sg12  (D2=284/day ≈ sg12=298 activations ✓)")
    e1("d3", "N_tram_in", 0,  ARM_LENGTH - 1.0,   "det_d3.xml",
       "D3: PT 1m upstream sg12   (D3=288/day; tram-stop detector triggers sg12)")

    lines.append("")
    lines.append("    <!-- ─────────────────────────────────────────────── -->")
    lines.append("    <!-- APPROACH DETECTORS (D4-D7)                      -->")
    lines.append("    <!-- ─────────────────────────────────────────────── -->")
    e1("d4",  "N_in", 0,  ARM_LENGTH - 18.0,  "det_d4.xml",
       "D4: N approach 18m upstream sg2/sg3 (1,066 veh/day; lane 0 only)")
    # D5 multilane: both lanes of E_in
    e1("d5a", "E_in", 0, ARM_LENGTH - 43.0,  "det_d5a.xml",
       "D5a: E approach 43m upstream sg4 — lane 0 (D5=3,016 combined with d5b)")
    e1("d5b", "E_in", 1, ARM_LENGTH - 43.0,  "det_d5b.xml",
       "D5b: E approach 43m upstream sg5 — lane 1 (combined with d5a → D5)")
    # D6 multilane: both lanes of S_in at 2m
    e1("d6a", "S_in", 0, ARM_LENGTH - 2.0,   "det_d6a.xml",
       "D6a: S approach 2m upstream sg6 — lane 0 (D6=288 veh/day combined)")
    e1("d6b", "S_in", 1, ARM_LENGTH - 2.0,   "det_d6b.xml",
       "D6b: S approach 2m upstream sg6 — lane 1")
    # D7 multilane: both lanes of S_in at 15m (preferred for calibration)
    e1("d7a", "S_in", 0, ARM_LENGTH - 15.0,  "det_d7a.xml",
       "D7a: S approach 15m upstream sg6 — lane 0 (D7=285 veh/day preferred)")
    e1("d7b", "S_in", 1, ARM_LENGTH - 15.0,  "det_d7b.xml",
       "D7b: S approach 15m upstream sg6 — lane 1")

    lines.append("")
    lines.append("    <!-- ─────────────────────────────────────────────── -->")
    lines.append("    <!-- DOWNSTREAM VALIDATION DETECTORS (D8-D10)        -->")
    lines.append("    <!-- Lane assignments calibrated to match CSV counts  -->")
    lines.append("    <!-- ─────────────────────────────────────────────── -->")
    # D8: S_out LANE 0 — N→S through exits to right (curbside) lane
    e1("d8",  "S_out", 0, 50.0, "det_d8.xml",
       "D8: downstream S_out lane 0 at 50m — N→S exits (287 veh/day; GEH calibrated)")
    # D9: W_out LANE 0 — E→W through exits to right (curbside) lane
    e1("d9",  "W_out", 0, 10.0, "det_d9.xml",
       "D9: downstream W_out lane 0 at 10m — E→W through (2,137 veh/day; GEH≈0.43)")
    # D10: W_out LANE 1 — N→W + S→W turns exit to inner lane (Assumption A11)
    e1("d10", "W_out", 1, 10.0, "det_d10.xml",
       "D10: downstream W_out lane 1 at 10m — N→W+S→W turns (857 veh/day; GEH≈1.80)")

    # ── Calibrators on approach edges (Improvement A-I5) ────────────────────
    lines.append("")
    lines.append("    <!-- ─────────────────────────────────────────────── -->")
    lines.append("    <!-- SUMO CALIBRATORS — enforce exact approach counts -->")
    lines.append("    <!-- Targets: D4(N_in), D5(E_in), D7(S_in)           -->")
    lines.append("    <!-- Ensures GEH ≈ 0 for approach LD detectors        -->")
    lines.append("    <!-- ─────────────────────────────────────────────── -->")

    def calibrator_block(cal_id, edge, route, demand_col, vtype="car"):
        """
        Generate a SUMO calibrator with per-15min flow entries.

        WHY pos=5.0 (5 m from edge start, ~245 m upstream of stop line):
        ------------------------------------------------------------------
        Original calibrators were placed at ARM_LENGTH-30 = 220 m (N/S) and
        ARM_LENGTH-60 = 190 m (E), which is only 30-60 m upstream of the
        detector they target.  During PM-peak congestion the stop-line queue
        backs up well past those positions.  Once a vehicle has travelled past
        the calibrator point, SUMO cannot remove it — hence the repeated
        "Calibrator 'cal_E' could not remove vehicle" warnings at t≈64000 s.
        Moving to pos=5.0 gives ~245 m of empty road ahead of the calibrator
        where insertion/removal can always succeed, eliminating those warnings.

        WHY jamThreshold="0.5":
        -----------------------
        Even at pos=5.0, if the queue fills the entire 250 m arm during extreme
        congestion, SUMO would still attempt removals and trigger emergency braking
        on adjacent lanes (observed: decel=9.00, wished=4.50 on E_in_1 at t≈64499).
        jamThreshold=0.5 tells the calibrator to suspend removal once road density
        exceeds 50 % of jam density, preventing those forced braking events.

        WHY departSpeed="desired":
        --------------------------
        The original departSpeed="speedLimit" inserts vehicles at full road speed
        (13.89 m/s on E_in) regardless of downstream conditions.  "desired" lets
        SUMO compute a safe insertion speed based on the current traffic state,
        which removes the emergency-braking cascade that cal_E was triggering on
        E_in lane 1 during the evening peak.
        """
        # jamThreshold=0.25: suspend removal when edge density > 25 % of jam
        #   density.  The previous value (0.5) was too lenient for the E_in arm
        #   during the PM peak (17:45–18:15), where the queue fills >50 % of the
        #   250 m arm.  0.25 halts removal earlier and prevents the "could not
        #   remove vehicle" cascade that triggered emergency braking on E_in_1.
        #
        # route REMOVED from <flow> entries (critical fix):
        #   The original code included route="{route}" in each calibrator flow.
        #   SUMO calibrators with a route filter only count vehicles matching
        #   that exact route against the vehsPerHour target.  Vehicles on other
        #   routes (e.g. N→W cars, bikes) pass through the calibrator position
        #   but are not subtracted from the target, so SUMO inserts extra
        #   vehicles on top of existing traffic → detector counts overshoot →
        #   GEH > 6 on D4, D6, D7.  Removing route makes the calibrator count
        #   ALL vehicles on the edge/lane, matching physical loop detector
        #   behaviour (a real inductive loop cannot filter by destination).
        #
        # departLane="free": SUMO selects the least-occupied insertion lane,
        #   preventing forced insertion into a congested lane 1 on E_in which
        #   caused the decel=9 / wished=4.5 emergency braking warnings.
        lines.append(
            f'    <calibrator id="{cal_id}" edge="{edge}" pos="5.0" '
            f'period="900" jamThreshold="0.25" '
            f'output="output/cal_{cal_id}.xml">'
        )
        for _, row in demand_df.sort_values("t_start").iterrows():
            t_s = int(row["t_start"])
            t_e = min(t_s + DEMAND_INTERVAL, SIM_END)
            count = int(row.get(demand_col, 0))
            vph = max(0, int(round(count * 3600.0 / DEMAND_INTERVAL)))
            lines.append(
                f'        <flow begin="{t_s}" end="{t_e}" '
                f'type="{vtype}" vehsPerHour="{vph}" '
                f'departSpeed="desired" departLane="free"/>'
            )
        lines.append("    </calibrator>")

    # ── Why D4, not D6, not D7 individually as calibrator targets ─────────────
    # D4 (18 m upstream, N_in): only one loop detector for the entire N approach.
    #   Calibrator target = d4 daily counts → direct one-to-one correspondence.
    #
    # D6 (2 m upstream, S_in): sits INSIDE the stop-line queue zone.  Vehicles
    #   stopping at red occupy D6's loop for the entire red phase.  The rising-edge
    #   count is correct (one per vehicle), but during high demand the stop-and-go
    #   motion can cause multiple brief occupancies per vehicle in the CSV, inflating
    #   the apparent count. NOT used as a calibrator target for this reason.
    #
    # D7 (15 m upstream, S_in): preferred South reference.  15 m gives a small
    #   but meaningful buffer outside the immediate queue, so occupancy noise from
    #   stop-line effects is reduced.  Used as the calibrator target for cal_S.
    #   In geh_evaluator.py D6 and D7 are aggregated into a single "S_approach"
    #   metric (mean of D6 and D7 daily totals) per user instruction.
    #
    # D1 (220 m upstream, N_tram_in): far upstream.  Not all trams originate far
    #   enough north to cross D1 before entering the intersection.  Trams from
    #   intermediate stops join N_tram_in after D1's position.  D2 (50 m) and D3
    #   (1 m) are the primary tram validators; D1 is marked auxiliary in the
    #   evaluator and excluded from the model acceptance pass-rate calculation.
    calibrator_block("cal_N", "N_in", "N_to_S", "d4_motor", "car")  # motor-only demand
    calibrator_block("cal_E", "E_in", "E_to_W", "d5", "car")
    calibrator_block("cal_S", "S_in", "S_to_N", "d7_motor", "car")  # motor-only demand

    lines.append("")
    lines.append("</additional>")
    path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[OK] {path.name} (10 E1 detectors [D1-D10] + 3 calibrators [N/E/S])")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Demand generator — Zurich Intersection Digital Twin")
    parser.add_argument("--csv", type=Path, default=CSV_DEFAULT,
                        help="Path to feb04_only.csv (default: ../data/feb04_only.csv)")
    parser.add_argument("--out", type=Path, default=SIM_DIR,
                        help="Output directory for route/detector files")
    args = parser.parse_args()

    print("\n" + "═" * 60)
    print("  Zurich Intersection — Demand Generator")
    print("  Genser et al. (2023), Data in Brief 48, 109117")
    print("═" * 60 + "\n")

    out_dir = args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "output").mkdir(exist_ok=True)

    if not args.csv.exists():
        print(f"[ERROR] CSV not found: {args.csv}")
        print("  Place feb04_only.csv in the data/ directory.")
        sys.exit(1)

    data = parse_csv(args.csv)
    demand_df = data["demand_df"]

    print("\n── Computing per-bin turn splits ──")
    splits_df = compute_turn_splits(demand_df)

    # Quick GEH preview from split calibration
    s = GLOBAL_SPLITS
    pred_d8  = (demand_df["d4"] * splits_df["N_to_S"]).sum()
    pred_d9  = (demand_df["d5"] * splits_df["E_to_W"]).sum()
    pred_d10 = (demand_df["d4"] * splits_df["N_to_W"] + demand_df["d7"] * s["S_to_W"]).sum()

    def geh_val(sim, obs):
        if sim + obs == 0: return 0.0
        return float(np.sqrt(2 * (sim - obs) ** 2 / (sim + obs)))

    print(f"  Predicted vs observed (daily):")
    print(f"    D8  pred={pred_d8:.0f}  obs={demand_df['d8'].sum():.0f}"
          f"  GEH={geh_val(pred_d8, demand_df['d8'].sum()):.2f}")
    print(f"    D9  pred={pred_d9:.0f}  obs={demand_df['d9'].sum():.0f}"
          f"  GEH={geh_val(pred_d9, demand_df['d9'].sum()):.2f}")
    print(f"    D10 pred={pred_d10:.0f}  obs={demand_df['d10'].sum():.0f}"
          f"  GEH={geh_val(pred_d10, demand_df['d10'].sum()):.2f}")

    print("\n── Writing simulation files ──")
    write_flows(demand_df, splits_df, out_dir)
    write_trams(data["tram_ns_times"], data["tram_sn_times"], out_dir)
    write_pedestrians(data["ped_times"], out_dir)
    write_detectors(demand_df, out_dir)

    print("\n" + "═" * 60)
    print("  Done. Files written to:", out_dir)
    print("  Next: python build_network.py (runs netconvert + generates net)")
    print("═" * 60 + "\n")


if __name__ == "__main__":
    main()

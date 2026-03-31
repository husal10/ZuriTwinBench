#!/usr/bin/env python3
"""
run_sumo.py — TraCI Runner for Zurich Intersection Digital Twin
==============================================================
Genser et al. (2023), Data in Brief 48, 109117

Handles:
  1. Night-mode TL program switching via TraCI
       t=3654s (01:00:54): TL_C switches day → night (Swiss Gelbblinker SVG §68)
       t=18000s (05:00:00): TL_C switches night → day
  2. Full-day simulation (0–86400s, 1s steps)
  3. Detector count collection and summary

Usage (from simulation/ directory):
  python run_sumo.py                   # runs sumo-gui
  python run_sumo.py --nogui           # runs sumo (headless, faster)
  python run_sumo.py --nogui --fast    # sumo with --no-step-log for speed

Requirements:
  - SUMO installed with SUMO_HOME set
  - intersection.net.xml built (run: python ../build_network.py)
  - flows.rou.xml, trams.rou.xml, pedestrians.rou.xml present
  - detectors.add.xml present
"""
import os
import sys
import argparse
import time
import csv
from pathlib import Path

# ─── Locate SUMO ──────────────────────────────────────────────────────────────
def find_sumo():
    """Find SUMO binary and set up TraCI path."""
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        tools = os.path.join(sumo_home, "tools")
        if tools not in sys.path:
            sys.path.append(tools)
    try:
        import traci
        return traci
    except ImportError:
        print("[ERROR] TraCI not found. Please:")
        print("  1. Install SUMO: https://sumo.dlr.de/docs/Installing/index.html")
        print("  2. Set SUMO_HOME to your SUMO installation directory")
        print("  3. Or run: sumo-gui -c intersection.sumocfg  (without TraCI)")
        sys.exit(1)


# ─── Configuration ────────────────────────────────────────────────────────────
SIM_DIR    = Path(__file__).parent
CFG_FILE   = SIM_DIR / "intersection.sumocfg"
OUT_DIR    = SIM_DIR / "output"

NIGHT_START = 3654    # 01:00:54 in seconds from midnight
NIGHT_END   = 18000   # 05:00:00

TL_ID       = "TL_C"
TL_DAY      = "day"
TL_NIGHT    = "night"

LOG_INTERVAL = 3600   # Print progress every N simulation seconds


def run(use_gui: bool = True, fast: bool = False):
    traci = find_sumo()

    OUT_DIR.mkdir(exist_ok=True)

    # ── Build SUMO command ─────────────────────────────────────────────────────
    sumo_bin = "sumo-gui" if use_gui else "sumo"
    cmd = [sumo_bin,
           "-c", str(CFG_FILE),
           "--start",              # auto-start (needed for TraCI)
           "--quit-on-end",
           ]
    if fast:
        cmd += ["--no-step-log"]

    print("\n" + "═" * 60)
    print("  Zurich Intersection — SUMO Digital Twin")
    print("  Genser et al. (2023), Data in Brief 48, 109117")
    print("═" * 60)
    print(f"  Config : {CFG_FILE}")
    print(f"  Mode   : {'GUI' if use_gui else 'Headless'}")
    print(f"  Night  : {NIGHT_START}s (01:00:54) → {NIGHT_END}s (05:00:00)")
    print()

    traci.start(cmd)

    tl_mode = TL_DAY    # current TL program
    step     = 0
    t_wall_start = time.time()
    last_log     = 0

    try:
        while traci.simulation.getMinExpectedNumber() > 0 or step < 86400:
            traci.simulationStep()
            step = int(traci.simulation.getTime())

            # ── Night-mode TL switching ──────────────────────────────────────
            if tl_mode == TL_DAY and step >= NIGHT_START:
                traci.trafficlight.setProgram(TL_ID, TL_NIGHT)
                tl_mode = TL_NIGHT
                print(f"  [t={step:5d}s] TL_C → NIGHT program (Gelbblinker SVG §68)")

            elif tl_mode == TL_NIGHT and step >= NIGHT_END:
                traci.trafficlight.setProgram(TL_ID, TL_DAY)
                tl_mode = TL_DAY
                print(f"  [t={step:5d}s] TL_C → DAY program")

            # ── Progress logging ─────────────────────────────────────────────
            if step - last_log >= LOG_INTERVAL:
                n_veh  = traci.vehicle.getIDCount()
                n_ped  = traci.person.getIDCount()
                n_halt = traci.vehicle.getIDCount()
                elapsed_wall = time.time() - t_wall_start
                hh = step // 3600
                mm = (step % 3600) // 60
                print(f"  [t={hh:02d}:{mm:02d}] veh={n_veh:4d} ped={n_ped:3d}"
                      f"  wall={elapsed_wall:.0f}s")
                last_log = step

            if step >= 86400:
                break

    except Exception as exc:
        print(f"[ERROR] Simulation error at step {step}: {exc}")
        raise
    finally:
        traci.close()

    wall_total = time.time() - t_wall_start
    print(f"\n  Simulation complete: {step}s simulated in {wall_total:.1f}s wall-clock")
    print(f"  Detector outputs: {OUT_DIR}/det_d*.xml")
    print(f"  Run calibration : cd .. && python calibration/geh_evaluator.py")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="TraCI runner — Zurich Intersection Digital Twin")
    parser.add_argument("--nogui", action="store_true",
                        help="Run sumo (headless) instead of sumo-gui")
    parser.add_argument("--fast", action="store_true",
                        help="Suppress per-step logging for speed (--no-step-log)")
    args = parser.parse_args()
    run(use_gui=not args.nogui, fast=args.fast)


if __name__ == "__main__":
    main()

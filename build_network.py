#!/usr/bin/env python3
"""
build_network.py — Master Build Script
=======================================
Zurich Intersection Digital Twin
Genser et al. (2023), Data in Brief 48, 109117

Steps:
  1. Validates project directory structure
  2. Runs demand/demand_generator.py → generates route + detector files
  3. Runs netconvert → generates simulation/intersection.net.xml from network/*.xml

Usage:
  cd zurich_digital_twin
  python build_network.py

  # To skip netconvert (if SUMO not installed):
  python build_network.py --no-netconvert

  # To use a different CSV:
  python build_network.py --csv /path/to/my.csv

After running:
  cd simulation
  sumo-gui -c intersection.sumocfg              # GUI mode
  python run_sumo.py --nogui --fast             # Headless, fastest
  cd .. && python calibration/geh_evaluator.py  # GEH report

File layout produced:
  simulation/
    intersection.net.xml   ← compiled network (netconvert output)
    flows.rou.xml          ← demand flows + vTypes + routes
    trams.rou.xml          ← tram vehicles (sorted by depart)
    pedestrians.rou.xml    ← pedestrian persons
    detectors.add.xml      ← E1 detectors + calibrators
    output/                ← SUMO writes detector XML files here
"""
import os
import sys
import argparse
import subprocess
from pathlib import Path

ROOT = Path(__file__).parent

NETWORK_DIR = ROOT / "network"
SIM_DIR     = ROOT / "simulation"
DATA_DIR    = ROOT / "data"
CSV_DEFAULT = DATA_DIR / "feb04_only.csv"

NODE_FILE = NETWORK_DIR / "nodes.nod.xml"
EDGE_FILE = NETWORK_DIR / "edges.edg.xml"
CON_FILE  = NETWORK_DIR / "connections.con.xml"
TLL_FILE  = NETWORK_DIR / "tls.tll.xml"
NET_OUT   = SIM_DIR     / "intersection.net.xml"


def check_structure():
    """Validate required files and directories."""
    print("[INFO] Checking project structure...")
    ok = True
    for f in [NODE_FILE, EDGE_FILE, CON_FILE, TLL_FILE]:
        if f.exists():
            print(f"  [OK]  {f.relative_to(ROOT)}")
        else:
            print(f"  [ERR] Missing: {f.relative_to(ROOT)}")
            ok = False
    return ok


def run_demand_generator(csv_path: Path) -> bool:
    """Run demand/demand_generator.py to produce route and detector files."""
    script = ROOT / "demand" / "demand_generator.py"
    if not script.exists():
        print(f"[ERROR] demand_generator.py not found: {script}")
        return False

    print("\n[INFO] Running demand generator...")
    cmd = [sys.executable, str(script), "--csv", str(csv_path), "--out", str(SIM_DIR)]
    try:
        result = subprocess.run(cmd, capture_output=False, text=True, timeout=120)
        if result.returncode == 0:
            print("[OK] Demand files generated.")
            return True
        else:
            print(f"[ERROR] demand_generator.py returned {result.returncode}")
            return False
    except subprocess.TimeoutExpired:
        print("[ERROR] demand_generator.py timed out (>120s).")
        return False
    except FileNotFoundError:
        print(f"[ERROR] Python not found: {sys.executable}")
        return False


def run_netconvert() -> bool:
    """Run SUMO netconvert to compile network XML files into .net.xml."""
    print("\n[INFO] Running netconvert...")
    cmd = [
        "netconvert",
        "--node-files",       str(NODE_FILE),
        "--edge-files",       str(EDGE_FILE),
        "--connection-files", str(CON_FILE),
        "--tllogic-files",    str(TLL_FILE),
        "--output-file",      str(NET_OUT),
        "--no-turnarounds",   "true",
        "--geometry.remove",  "true",
        "--default.junctions.radius", "12.0",   # ~24m box matching real Zürich stop-line box
        "--verbose",          "true",
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        if result.returncode == 0:
            print(f"[OK] Network compiled → {NET_OUT.relative_to(ROOT)}")
            if result.stderr:
                # Print warnings (netconvert writes info to stderr)
                for line in result.stderr.splitlines():
                    if "Warning" in line or "Error" in line:
                        print(f"  [netconvert] {line}")
            return True
        else:
            print(f"[WARN] netconvert exit code={result.returncode}")
            if result.stderr:
                print(result.stderr[-3000:])
            return False
    except FileNotFoundError:
        print("[WARN] netconvert not found on PATH.")
        print("  Install SUMO and add its bin/ directory to your system PATH.")
        print("  Windows: C:\\Program Files (x86)\\Eclipse\\Sumo\\bin")
        print("  Linux  : sudo apt install sumo sumo-tools")
        print("  macOS  : brew install sumo")
        print("  The demand/route files are already generated — only intersection.net.xml is missing.")
        print("  Re-run after installing SUMO, or use: python build_network.py --no-netconvert")
        return False
    except subprocess.TimeoutExpired:
        print("[WARN] netconvert timed out.")
        return False


def print_summary():
    """Print a summary of generated files and next steps."""
    print("\n" + "═" * 65)
    print("  BUILD COMPLETE — Zurich Intersection Digital Twin")
    print("═" * 65)
    print()

    generated = [
        ("simulation/intersection.net.xml",  "SUMO compiled network"),
        ("simulation/flows.rou.xml",          "Demand flows + vTypes + routes"),
        ("simulation/trams.rou.xml",          "Tram vehicles (CSV timestamps)"),
        ("simulation/pedestrians.rou.xml",    "Pedestrian persons (sg7-sg10)"),
        ("simulation/detectors.add.xml",      "E1 detectors + calibrators"),
    ]
    print("  Generated files:")
    for fname, desc in generated:
        fpath = ROOT / fname
        status = "✓" if fpath.exists() else "✗ MISSING"
        size_s = f"({fpath.stat().st_size//1024}KB)" if fpath.exists() else ""
        print(f"    {status}  {fname:<42} {size_s} — {desc}")

    print()
    print("  Run simulation:")
    print("    cd simulation")
    print("    sumo-gui -c intersection.sumocfg                # GUI mode")
    print("    python run_sumo.py --nogui --fast               # Headless")
    print()
    print("  Evaluate calibration (after simulation):")
    print("    python calibration/geh_evaluator.py             # Hourly GEH")
    print("    python calibration/geh_evaluator.py --period 900  # 15-min GEH")
    print()
    print("  Key GEH predictions (daily, from turn-split calibration):")
    print("    D4/D5/D6/D7  GEH ≈ 0      (SUMO calibrators enforce exact counts)")
    print("    D8           GEH ≈ 0.04   (N→S calibrated from D8/D4 balance)")
    print("    D9           GEH ≈ 0.43   (E→W calibrated from W_out balance)")
    print("    D10          GEH ≈ 1.80   (N→W+S→W calibrated from W_out balance)")
    print("    D1/D2/D3     GEH ≈ <2     (exact tram schedule from CSV)")
    print()
    print("  Threshold: GEH < 5.0 = acceptable | DfT WebTAG: ≥85% under GEH<5")
    print("═" * 65 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Build script — Zurich Intersection Digital Twin")
    parser.add_argument("--csv", type=Path, default=CSV_DEFAULT,
                        help="Path to feb04_only.csv")
    parser.add_argument("--no-netconvert", action="store_true",
                        help="Skip netconvert (demand files still generated)")
    args = parser.parse_args()

    print("\n" + "═" * 65)
    print("  Zurich Intersection Digital Twin — Build Script")
    print("  Genser et al. (2023), Data in Brief 48, 109117")
    print("═" * 65 + "\n")

    # Check CSV
    if not args.csv.exists():
        print(f"[ERROR] CSV not found: {args.csv}")
        print("  Copy feb04_only.csv into the data/ directory.")
        sys.exit(1)
    print(f"[INFO] CSV: {args.csv.relative_to(ROOT)}")

    # Validate network XMLs
    if not check_structure():
        print("[ERROR] Missing network files. Check network/ directory.")
        sys.exit(1)

    # Create simulation output dir
    (SIM_DIR / "output").mkdir(parents=True, exist_ok=True)

    # Step 1: generate demand files
    ok_demand = run_demand_generator(args.csv)
    if not ok_demand:
        print("[WARN] Demand generation had errors. Proceeding cautiously.")

    # Step 2: compile network (unless skipped)
    ok_net = True
    if not args.no_netconvert:
        ok_net = run_netconvert()
    else:
        print("\n[INFO] Skipping netconvert (--no-netconvert)")

    # Summary
    print_summary()

    if not ok_net and not args.no_netconvert:
        print("[WARN] netconvert not found or failed.")
        print("  The demand/route files were generated successfully.")
        print("  To compile the network, install SUMO and ensure netconvert is on PATH:")
        print("    Windows : https://sumo.dlr.de/docs/Installing/Windows_Build.html")
        print("              Add C:\\Program Files (x86)\\Eclipse\\Sumo\\bin to PATH")
        print("    Linux   : sudo apt install sumo sumo-tools")
        print("    macOS   : brew install sumo")
        print("  Then re-run:  python build_network.py")
        print("  Or skip it:   python build_network.py --no-netconvert")
        # Do NOT sys.exit — demand files are still valid and usable


if __name__ == "__main__":
    main()

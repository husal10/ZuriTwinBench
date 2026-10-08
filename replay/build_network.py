#!/usr/bin/env python3
"""Build the Zürich intersection SUMO network used by the detector/signal replay.

Geometry follows the ZuriTwinBench package (four arms, 250 m, centre node C
controlled by one TLS) but, unlike it, the pedestrian crossings (sg7-sg10) are
real SUMO crossings attached to the TLS, so every one of sg1..sg12 drives real
links.  Link->signal-group assignment is NOT hard-coded by index: replay.py
derives it from the generated net via sg_map.py.

Usage:  python build_network.py            # writes net/zurich.net.xml
"""
import subprocess, sys
from pathlib import Path

OUT = Path(__file__).parent / "net"
OUT.mkdir(exist_ok=True)
ARM = 250.0
V_NS, V_EW, V_TRAM = 8.3333, 13.8889, 8.3333   # 30 / 50 / 30 km/h

nodes = f"""<nodes>
  <node id="C" x="0" y="0" type="traffic_light" tl="TL_C"/>
  <node id="N_src" x="0" y="{ARM}"/><node id="S_src" x="0" y="-{ARM}"/>
  <node id="E_src" x="{ARM}" y="0"/><node id="W_src" x="-{ARM}" y="0"/>
</nodes>"""

def road(i, a, b, n, v):
    return (f'<edge id="{i}" from="{a}" to="{b}" numLanes="{n}" speed="{v}" '
            f'allow="all" disallow="rail_urban tram" sidewalkWidth="2.0" spreadType="right"/>')
def rail(i, a, b):
    return (f'<edge id="{i}" from="{a}" to="{b}" numLanes="1" speed="{V_TRAM}" '
            f'allow="tram rail_urban" spreadType="center"/>')

edges = "<edges>\n" + "\n".join([
    road("N_in","N_src","C",2,V_NS), road("N_out","C","N_src",2,V_NS),
    road("S_in","S_src","C",2,V_NS), road("S_out","C","S_src",2,V_NS),
    road("E_in","E_src","C",2,V_EW), road("E_out","C","E_src",2,V_EW),
    road("W_in","W_src","C",1,V_EW), road("W_out","C","W_src",2,V_EW),
    rail("N_tram_in","N_src","C"), rail("N_tram_out","C","N_src"),
    rail("S_tram_in","S_src","C"), rail("S_tram_out","C","S_src"),
]) + "\n</edges>"

# vehicle links.  sg ids are documented in sg_map.py
# logical vehicle lane k of a road edge is SUMO lane k+1 (lane 0 is the sidewalk)
def c(f, t, fl, tl): return f'<connection from="{f}" to="{t}" fromLane="{fl+1}" toLane="{tl+1}"/>'
cons = "<connections>\n" + "\n".join([
    # Table 1 of Genser et al. (2023); logical lane 0 = right-most vehicle lane
    c("W_in","E_out",0,0),                         # sg1  W->E
    c("N_in","W_out",0,0),                         # sg2  N->W (right turn), outer W_out lane (d10)
    c("N_in","W_out",1,1),                         # sg2  N->W, inner W_out lane (d9) - lane split from flow conservation
    c("N_in","S_out",1,0),                         # sg3  N->S (bikes only)
    c("E_in","W_out",1,1),                         # sg4  E->W
    c("E_in","N_out",0,0),                         # sg5  E->N (right turn)
    c("S_in","E_out",0,0),                         # sg6  S->E (right turn)
    c("S_in","N_out",1,1),                         # sg6  S->N
    c("S_in","W_out",1,1),                         # sg6  S->W (left turn)
    '<connection from="N_tram_in" to="S_tram_out" fromLane="0" toLane="0"/>',  # sg12 PT N->S
    '<connection from="S_tram_in" to="N_tram_out" fromLane="0" toLane="0"/>',  # sg11 PT S->N
    # crossings: sg7 W arm, sg8 N arm, sg9 E arm, sg10 S arm (arm assignment assumed)
    '<crossing node="C" edges="W_in W_out" priority="1"/>',
    '<crossing node="C" edges="N_in N_out" priority="1"/>',
    '<crossing node="C" edges="E_in E_out" priority="1"/>',
    '<crossing node="C" edges="S_in S_out" priority="1"/>',
]) + "\n</connections>"

for n, s in (("nodes.nod.xml",nodes),("edges.edg.xml",edges),("connections.con.xml",cons)):
    (OUT/n).write_text('<?xml version="1.0" encoding="UTF-8"?>\n'+s+"\n")

cmd = ["netconvert", "-n", OUT/"nodes.nod.xml", "-e", OUT/"edges.edg.xml",
       "-x", OUT/"connections.con.xml", "-o", OUT/"zurich.net.xml",
       "--crossings.guess", "false", "--no-turnarounds", "true",
       "--tls.default-type", "static", "--junctions.corner-detail", "0",
       "--no-warnings", "true"]
subprocess.run([str(x) for x in cmd], check=True)
print("wrote", OUT/"zurich.net.xml")

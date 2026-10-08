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
# logical vehicle lane k of a road edge is SUMO lane k+1 (lane 0 is the sidewalk).
# The traffic light has exactly 12 link indices, one per signal group: linkIndex = sg number - 1, so the TLS state string is
# [sg1 ... sg12].  Connections governed by the same group (sg2: two lanes, sg6: three movements) share its link index; the four
# pedestrian crossings take indices 6-9 (sg7-sg10).
def c(f, t, fl, tl, sg):
    return f'<connection from="{f}" to="{t}" fromLane="{fl+1}" toLane="{tl+1}" tl="TL_C" linkIndex="{sg-1}"/>'
def tram(f, t, sg):
    return f'<connection from="{f}" to="{t}" fromLane="0" toLane="0" tl="TL_C" linkIndex="{sg-1}"/>'
def cross(edges, sg):
    return f'<crossing node="C" edges="{edges}" priority="1" tlLinkIndex="{sg-1}"/>'
cons = "<connections>\n" + "\n".join([
    # Table 1 of Genser et al. (2023); logical lane 0 = right-most vehicle lane
    c("W_in","E_out",0,0,1),                       # sg1  W->E
    c("N_in","W_out",0,0,2),                       # sg2  N->W (right turn), outer W_out lane (d10)
    c("N_in","W_out",1,1,2),                       # sg2  N->W, inner W_out lane (d9) - lane split from flow conservation
    c("N_in","S_out",1,0,3),                       # sg3  N->S (bikes only)
    c("E_in","W_out",1,1,4),                       # sg4  E->W
    c("E_in","N_out",0,0,5),                       # sg5  E->N (right turn)
    c("S_in","E_out",0,0,6),                       # sg6  S->E (right turn)
    c("S_in","N_out",1,1,6),                       # sg6  S->N
    c("S_in","W_out",1,1,6),                       # sg6  S->W (left turn)
    tram("S_tram_in","N_tram_out",11),             # sg11 PT S->N
    tram("N_tram_in","S_tram_out",12),             # sg12 PT N->S
    # crossings: sg7 W arm, sg8 N arm, sg9 E arm, sg10 S arm (arm assignment assumed)
    cross("W_in W_out",7), cross("N_in N_out",8), cross("E_in E_out",9), cross("S_in S_out",10),
]) + "\n</connections>"

# one placeholder program with exactly 12 positions (sg1..sg12); the replay overrides the state every second
tll = '<tlLogics>\n  <tlLogic id="TL_C" type="static" programID="0" offset="0">\n    <phase duration="86400" state="rrrrrrrrrrrr"/>\n  </tlLogic>\n</tlLogics>'
(OUT/"tls.tll.xml").write_text('<?xml version="1.0" encoding="UTF-8"?>\n'+tll+"\n")
for n, s in (("nodes.nod.xml",nodes),("edges.edg.xml",edges),("connections.con.xml",cons)):
    (OUT/n).write_text('<?xml version="1.0" encoding="UTF-8"?>\n'+s+"\n")

cmd = ["netconvert", "-n", OUT/"nodes.nod.xml", "-e", OUT/"edges.edg.xml",
       "-x", OUT/"connections.con.xml", "-i", OUT/"tls.tll.xml", "-o", OUT/"zurich.net.xml",
       "--crossings.guess", "false", "--no-turnarounds", "true",
       "--tls.default-type", "static", "--junctions.corner-detail", "0",
       "--no-warnings", "true"]
subprocess.run([str(x) for x in cmd], check=True)

# ---- post-process: exactly 12 TLS link indices (linkIndex = sg number - 1) ----------------------------------------------
# netconvert gives every connection its own TLS link index.  Re-index them by signal group so that the state string has exactly
# the 12 positions sg1..sg12; connections governed by the same group (sg2, sg6) share one index, as SUMO allows.
import sys, xml.etree.ElementTree as ET
sys.path.insert(0, str(Path(__file__).parent))
import sg_map
net_file = OUT/"zurich.net.xml"
n_old, groups = sg_map.resolve(net_file)
old2sg = {i: int(sg[2:]) for sg, idxs in groups.items() for i in idxs}
tree = ET.parse(net_file); root = tree.getroot()
for con in root.iter("connection"):
    if con.get("tl") == "TL_C":
        con.set("linkIndex", str(old2sg[int(con.get("linkIndex"))] - 1))
        if con.get("linkIndex2") is not None:
            del con.attrib["linkIndex2"]
for tl in root.iter("tlLogic"):
    if tl.get("id") == "TL_C":
        for ph in list(tl):
            ph.set("state", "r" * 12)
tree.write(net_file, encoding="UTF-8", xml_declaration=True)
n_new, groups_new = sg_map.resolve(net_file)
assert n_new == 12 and all(v == [int(k[2:]) - 1] for k, v in groups_new.items()), groups_new
print("wrote", net_file, f"(TLS with {n_new} link indices; {n_old} connections re-indexed by signal group)")

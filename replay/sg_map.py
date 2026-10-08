"""Signal-group (sg1..sg12) -> SUMO TLS link mapping.

Resolved from the generated net by (from-edge, to-edge) / crossing edges so
link indices never depend on netconvert's internal ordering.

Semantics: Table 1 of Genser et al. (2023), Data in Brief 48, 109117
  sg1 W->E | sg2 N->W | sg3 N->S (bikes only) | sg4 E->W | sg5 E->N |
  sg6 S->W/N/E | sg7,sg9 pedestrians N/S | sg8,sg10 pedestrians W/E |
  sg11 PT S->N | sg12 PT N->S
CSV code: 0 = red (incl. 1 s red-yellow before and 3 s amber after green),
1 = green, 8 = night mode (flashing amber, Gelbblinker; seen in the Feb-04 file).
Which physical arm each of sg7..sg10 crosses is NOT stated in the paper (ASSUMED).
"""
import xml.etree.ElementTree as ET

VEH_SG = ["sg1", "sg2", "sg3", "sg4", "sg5", "sg6"]      # may show amber
ALL_SG = [f"sg{i}" for i in range(1, 13)]

# sg -> list of (fromEdge, toEdge) vehicle links, or ("X", crossing-arm-edge)
SG_LINKS = {
    "sg1": [("W_in", "E_out")],
    "sg2": [("N_in", "W_out")],
    "sg3": [("N_in", "S_out")],
    "sg4": [("E_in", "W_out")],
    "sg5": [("E_in", "N_out")],
    "sg6": [("S_in", "N_out"), ("S_in", "W_out"), ("S_in", "E_out")],
    "sg7": [("X", "W_in")],
    "sg8": [("X", "N_in")],
    "sg9": [("X", "E_in")],
    "sg10": [("X", "S_in")],
    "sg11": [("S_tram_in", "N_tram_out")],
    "sg12": [("N_tram_in", "S_tram_out")],
}
# links that must yield to others / to pedestrians when green -> lowercase 'g'
YIELD_G = {"sg2", "sg6"}
# night (Gelbblinker): main E-W road has priority, N-S yields, trams have priority
NIGHT_PRIORITY = {"sg1", "sg4", "sg5", "sg11", "sg12", "sg7", "sg8", "sg9", "sg10"}


def resolve(net_file, tls_id="TL_C"):
    """Return (n_links, {sg: [link_index,...]}) parsed from the net file."""
    root = ET.parse(net_file).getroot()
    cross = {}
    for e in root.iter("edge"):
        if e.get("function") == "crossing":
            cross[e.get("id")] = e.get("crossingEdges").split()
    links = []                                   # (idx, from_edge, to_edge, from_lane)
    for c in root.iter("connection"):
        if c.get("tl") != tls_id:
            continue
        idx = int(c.get("linkIndex"))
        fe, te = c.get("from"), c.get("to")
        fl = int(c.get("fromLane", 0))
        fl -= 0 if fe.endswith("tram_in") or fe.startswith(":") else 1   # sidewalk = lane 0 on road edges
        links.append((idx, fe, te, fl))
    n = max(i for i, *_ in links) + 1
    out = {sg: [] for sg in ALL_SG}
    for sg, sel in SG_LINKS.items():
        for s in sel:
            for idx, fe, te, fl in links:
                if s[0] == "X":
                    if fe.startswith(":"):
                        # walkingarea -> crossing link: te is crossing edge id
                        if te in cross and s[1] in cross[te]:
                            out[sg].append(idx)
                elif fe == s[0] and te == s[1] and (len(s) < 3 or fl == s[2]):
                    out[sg].append(idx)
    missing = [sg for sg, v in out.items() if not v]
    if missing:
        raise RuntimeError(f"signal groups without links in {net_file}: {missing}")
    return n, {sg: sorted(set(v)) for sg, v in out.items()}


if __name__ == "__main__":
    import sys
    print(resolve(sys.argv[1] if len(sys.argv) > 1 else "net/zurich.net.xml"))

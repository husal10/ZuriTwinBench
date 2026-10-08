#!/usr/bin/env python3
"""Replay the Zürich intersection (Genser et al. 2023) in SUMO, second by second.

* Signal groups sg1..sg12 are forced onto the TLS every simulated second from the
  CSV (sim clock == time of day; day n starts at n*86400 s).  Code 8 = night mode
  (Gelbblinker).  The CSV has no amber information: after a vehicle group leaves
  green an amber of --amber seconds is shown inside the CSV's "0" period.
* Detectors d1..d10 are SUMO induction loops at the documented positions
  (detectors.json).  Their per-second state is logged to sim_detectors.csv in the
  same layout as the input, and compared with the field data (compare.py).
* Two replay modes
    exact   every detector event in the CSV is reproduced by a virtual vehicle
            placed on that loop for exactly the observed occupancy (ghost).
            => detector columns match the field data by construction; vehicles
            do not travel through the junction.
    hybrid  the upstream detectors d2 (tram N->S), d4, d5 (cars), d7 (tram S->N)
            inject real vehicles that drive through the junction under the
            replayed signals; d3, d6, d8, d9, d10 are *emergent* and are
            validated against the data (d1 stays a ghost).

Usage
  python replay.py --csv ../data/feb04_only.csv --mode exact --out out
  python replay.py --csv data_dir/ --mode hybrid --from "2019-02-04 06:00" --to "2019-02-04 10:00" --gui
"""
import argparse, json, os, subprocess, sys, time
from pathlib import Path
import numpy as np, pandas as pd

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import data as D, sg_map, compare as C, fidelity as F, controller as K, signal_fidelity as SF
import xml.etree.ElementTree as ET

TLS = "TL_C"
NET = HERE / "net" / "zurich.net.xml"
NIGHT_CODE = 8


def lane_ids(net):
    """edge -> [lane ids that allow vehicles/rail], in index order (sidewalks skipped)."""
    out = {}
    for e in ET.parse(net).getroot().iter("edge"):
        if e.get("function") in ("internal", "crossing", "walkingarea"):
            continue
        ls = [l for l in e.iter("lane") if l.get("allow") != "pedestrian"]
        out[e.get("id")] = [l.get("id") for l in ls]
    return out


def lane_lengths(net):
    return {l.get("id"): float(l.get("length")) for l in ET.parse(net).getroot().iter("lane")}


LANES = lane_ids(NET)


def write_support_files(work, dets, llen, nlinks):
    """Loops, vTypes, routes and the static 'replay' TLS program."""
    work.mkdir(parents=True, exist_ok=True)
    loops, pos, edges = [], {}, set()
    for d, c in dets.items():
        for k, lp in enumerate(c["loops"]):
            if lp.get("shadow"):          # injection-only entry (shares a lane with another loop)
                continue
            lid = LANES[lp["edge"]][lp["lane"]]
            L = llen[lid]
            p = L - c["dist_m"] if c["ref"] == "stop" else c["dist_m"]
            p = max(0.1, min(p, L - 0.1))
            pos[(d, k)] = p
            edges.add(lp["edge"])
            loops.append(f'<inductionLoop id="{d}_{k}" lane="{lid}" pos="{p:.2f}" period="3600" file="NUL"/>')
    groutes = "".join(f'<route id="g_{e}" edges="{e}"/>' for e in sorted(edges))
    add = f"""<additional>
 <vType id="car" vClass="passenger" length="4.5" minGap="2.5" accel="2.6" decel="4.5" maxSpeed="13.89" sigma="0.3" tau="1.9"/>
 <vType id="bike" vClass="bicycle" length="1.8" minGap="1.0" accel="1.2" decel="3.0" maxSpeed="6.11" sigma="0.3" color="0.2,0.8,0.2" guiShape="bicycle"/>
 <vType id="tram" vClass="tram" length="36.3" width="2.65" minGap="5" accel="1.0" decel="3.0" maxSpeed="8.33" sigma="0" tau="1.5" color="1,0.8,0" guiShape="rail/railcar"/>
 <vType id="ghost_car" vClass="passenger" length="2.0" minGap="0" accel="1e-9" decel="9" maxSpeed="100" speedFactor="1" speedDev="0" sigma="0" color="0.6,0.6,0.6"/>
 <vType id="ghost_tram" vClass="tram" length="3.0" width="2.65" minGap="0" accel="1e-9" decel="9" maxSpeed="100" speedFactor="1" speedDev="0" sigma="0" color="0.9,0.6,0" guiShape="rail/railcar"/>
 <route id="rt_car_NW" edges="N_in W_out"/><route id="rt_bike_NS" edges="N_in S_out"/>
 <route id="rt_car_EN" edges="E_in N_out"/><route id="rt_car_EW" edges="E_in W_out"/>
 <route id="rt_car_SN" edges="S_in N_out"/>
 <route id="rt_tram_NS" edges="N_tram_in S_tram_out"/><route id="rt_tram_SN" edges="S_tram_in N_tram_out"/>
 <route id="rt_tram_out_S" edges="S_tram_out"/><route id="rt_car_out_W" edges="W_out"/><route id="rt_bike_out_S" edges="S_out"/>
 {groutes}
 {chr(10).join(loops)}
 <tlLogic id="{TLS}" type="static" programID="replay" offset="0">
  <phase duration="31536000" state="{'r' * nlinks}"/>
 </tlLogic>
</additional>"""
    (work / "replay.add.xml").write_text(add)
    return pos


def events(col):
    """[(start_idx, length)] of consecutive 1-runs."""
    x = np.concatenate([[0], (col > 0).astype(np.int8), [0]])
    dx = np.diff(x)
    s, e = np.where(dx == 1)[0], np.where(dx == -1)[0]
    return list(zip(s.tolist(), (e - s).tolist()))


class Signals:
    """CSV code -> TLS state string.  Paper: 1 s red-yellow before and 3 s amber after
    each green are folded into code 0, so they are re-created here for sg1..sg6."""
    def __init__(self, nlinks, sgl, amber, red_yellow=1):
        self.n, self.sgl, self.amber, self.ry = nlinks, sgl, amber, red_yellow
        self.prev = {s: 0 for s in sg_map.ALL_SG}
        self.left = {s: 0 for s in sg_map.ALL_SG}

    def state(self, row, nxt):
        st = ["r"] * self.n
        for sg in sg_map.ALL_SG:
            v, vn = int(row[sg]), int(nxt[sg])
            if v == NIGHT_CODE:
                ch = "O" if sg in sg_map.NIGHT_PRIORITY else "o"
                self.left[sg] = 0
            elif v == 1:
                ch = "g" if sg in sg_map.YIELD_G else "G"
                self.left[sg] = 0
            else:
                veh = sg in sg_map.VEH_SG
                if veh and self.prev[sg] == 1 and self.amber > 0:
                    self.left[sg] = self.amber
                if self.left[sg] > 0:
                    ch = "y"; self.left[sg] -= 1
                elif veh and vn == 1 and self.ry > 0:
                    ch = "u"
                else:
                    ch = "r"
            self.prev[sg] = v
            for i in self.sgl[sg]:
                st[i] = ch
        return "".join(st)


def build_cmd(args, work, begin):
    tripinfo = [] if args.mode == "exact" else ["--tripinfo-output", str(Path(args.out) / "tripinfo.xml")]
    exe = "sumo-gui" if args.gui else "sumo"
    cmd = [exe, "-n", str(NET), "-a", str(work / "replay.add.xml"), "--begin", str(begin),
           "--step-length", "1", "--no-step-log", "true", "--no-warnings", "true",
           "--collision.action", "none", "--time-to-teleport", "-1", "--insertion-checks", "none",
           "--ignore-junction-blocker", "-1", "--lateral-resolution", "0",
           "--seed", str(args.seed), "--human-readable-time", "true"] + tripinfo
    if args.gui:
        cmd += ["--start", "true", "--delay", str(args.delay), "--quit-on-end", "true"]
    return cmd


def rises_idx(col):
    x = np.asarray(col) > 0
    return np.where(x[1:] & ~x[:-1])[0] + 1


def pick(rng, loops):
    w = np.array([lp["w"] for lp in loops], float)
    return int(rng.choice(len(loops), p=w / w.sum())) if w.sum() > 0 else None


def rises(col):
    x = np.asarray(col) > 0
    return np.where(x[1:] & ~x[:-1])[0] + 1


def classify(g, dets, name, evs, cache):
    """Tram vs MPT/bike label per event of a detector that shares its loop between modes.
    Rules are data-derived (see detectors.json): a PT event is followed by a green start of
    the PT signal (sg11) or follows a d3 tram detection; d6 inherits the label of the d7
    event just before it (d7->d6 pairing is 98.6 %)."""
    if name in cache:
        return cache[name]
    rule, starts = dets[name]["classify"], [a for a, _ in evs]
    out = []
    for a in starts:
        cls = None
        D0 = dict(evs)[a]
        if "tram_if_green_at_end" in rule:
            sg = g[rule["tram_if_green_at_end"]].to_numpy()
            e = a + D0
            cls = "tram" if (sg[max(0, e - 2):e + 2] == 1).any() else "mpt"
        if cls is None and "inherit_forward" in rule:
            fwd = classify(g, dets, rule["inherit_forward"], events(g[rule["inherit_forward"]].to_numpy()), cache)
            nxt = [c0 for s0, c0 in fwd if a <= s0 <= a + rule["within_s"]]
            cls = nxt[0] if nxt else None
        if cls is None and "inherit" in rule:
            base = classify(g, dets, rule["inherit"], events(g[rule["inherit"]].to_numpy()), cache)
            prev = [(s0, c0) for s0, c0 in base if a - rule["within_s"] <= s0 <= a]
            cls = prev[-1][1] if prev else None
        if cls is None and "tram_if_rise" in rule:
            r = rises(g[rule["tram_if_rise"]].to_numpy())
            w0, w1 = rule["window_s"]
            cls = "tram" if ((r >= a + w0) & (r <= a + w1)).any() else "mpt"
        if cls is None and "tram_if_lag_after" in rule:
            s3 = np.array([s0 for s0, _ in events(g[rule["tram_if_lag_after"]].to_numpy())])
            l0, l1 = rule["lag_s"]
            cls = "tram" if len(s3) and ((a - s3 >= l0) & (a - s3 <= l1)).any() else "mpt"
        out.append((a, cls or "tram"))
    cache[name] = out
    return out


def schedule(g, dets, mode, seed, ordinal, pos, llen, nrows, pt_leads=None, pt_extra=0):
    """Return ({row: [action]}, stats).  Actions are dicts consumed by run()."""
    plan, cache, stats = {}, {}, {}
    pt_calls = []                  # advance PT requests (V2I) for the injected N->S trams
    prng = np.random.default_rng(seed + 31 + ordinal)
    # d2 trams that follow a dwelling d1 tram are emergent in hybrid mode (not injected twice)
    emergent_d2 = set()
    if mode != "exact" and dets.get("d1", {}).get("role_hybrid") == "dwell":
        s2 = [a for a, _ in events(g["d2"].to_numpy())]
        for a1, _ in events(g["d1"].to_numpy()):
            nxt = [x for x in s2 if a1 + 10 < x <= a1 + 150]
            if nxt:
                emergent_d2.add(nxt[0])
    for d, c in dets.items():
        role = "ghost" if mode == "exact" else c.get("role_hybrid", "observe")
        if role == "observe":
            continue
        rng = np.random.default_rng(seed + int(d[1:]) + ordinal * 101)
        evs = events(g[d].to_numpy())
        labels = dict(classify(g, dets, d, evs, cache)) if "classify" in c else {}
        if labels:
            stats[d] = {k: sum(1 for v in labels.values() if v == k) for k in ("tram", "mpt")}
        for a, D in evs:
            if role == "drive" and d == "d2" and a in emergent_d2:
                continue
            if labels:
                want = labels[a]
                cand = [i for i, lp in enumerate(c["loops"]) if (lp["vtype"] == "tram") == (want == "tram")]
                k = int(rng.choice(cand)) if cand else 0
            else:
                k = pick(rng, c["loops"])
            if k is None:
                continue
            lp = c["loops"][k]
            tram = lp["vtype"] == "tram"
            lane_id = LANES[lp["edge"]][lp["lane"]]
            p = pos.get((d, k)) or pos[(d, max(i for i in range(k) if (d, i) in pos))]
            act = dict(det=d, k=k, lp=lp, dur=D, a=a, lane=int(lane_id.rsplit("_", 1)[1]), p=p)
            if role == "ghost":
                gt = "ghost_tram" if tram else "ghost_car"
                Lg = 3.0 if tram else 2.0
                if D <= 2:        # fast crossing: on the loop for exactly D one-second steps
                    v = Lg / (D - 0.5)
                    act.update(type=gt, v=v, p0=max(0.05, p - v * 1.25), at=max(0, a - 2), rm=a + D + 1)
                else:             # hold on the loop, then remove (removal takes effect ~2 steps later)
                    act.update(type=gt, v=0.0, p0=p + 0.5, at=a, rm=a + D - 2)
            else:                 # drive / dwell: a real vehicle crossing the loop, then travelling on
                Lv = {"tram": 36.3, "car": 4.5, "bike": 1.8}[lp["vtype"]]
                vmax = 8.0 if tram else (6.0 if lp["vtype"] == "bike" else (13.0 if lp["edge"].startswith("E") else 8.0))
                dwell = role == "dwell" and tram        # MPT events on a 'dwell' detector just drive
                v = 8.0 if dwell else float(np.clip(Lv / (D - 0.5), 1.0, vmax))
                act.update(type=lp["vtype"], v=v, p0=max(0.05, p - v * 1.25), at=max(0, a - 2), rm=None, drive=True)
                if dwell:             # stop with the tram body on the loop for the observed dwell
                    act["stop"] = (lp["edge"], p + 3.0, act["lane"], max(1, D - 9))
            if role == "drive" and d == "d2" and pt_leads:
                pt_calls.append(int(max(0, a + 6 - prng.choice(pt_leads) - pt_extra)))      # d3 arrival ~ d2 + 6 s
            plan.setdefault(act["at"], []).append(act)
    stats["_pt_calls_n"] = pt_calls
    return plan, stats


def run(args):
    import traci
    dets = {k: v for k, v in json.load(open(HERE / "detectors.json")).items() if not k.startswith("_")}
    nlinks, sgl = sg_map.resolve(NET, TLS)
    llen = lane_lengths(NET)
    work = Path(args.out) / "_work"
    pos = write_support_files(work, dets, llen, nlinks)

    df = D.load(args.csv)
    if args.t_from:
        df = df[df.index >= pd.Timestamp(args.t_from)]
    if args.t_to:
        df = df[df.index < pd.Timestamp(args.t_to)]
    if df.empty:
        sys.exit("no rows in the selected window")
    day_list = list(D.days(df))
    dets_base = {k: dict(v) for k, v in dets.items()}
    d0 = day_list[0][0]
    begin = int(day_list[0][1].index[0])
    traci.start(build_cmd(args, work, begin), label="replay")
    traci.trafficlight.setProgram(TLS, "replay")
    print(f"net links={nlinks}  sg->links={sgl}")

    summary, nveh, t_wall, fid_days = [], 0, time.time(), []
    det_names = list(dets)
    loop_ids = {d: [f"{d}_{k}" for k, lp in enumerate(c["loops"]) if not lp.get("shadow")] for d, c in dets.items()}
    for date, g, missing in day_list:
        ordinal = (date - d0).days
        dets = {k: dict(v) for k, v in dets_base.items()}
        if date.dayofweek >= 5:                       # day-type specific injected splits (calibrated on training weekends)
            for k_, v_ in dets.items():
                if "loops_weekend" in v_:
                    v_["loops"] = v_["loops_weekend"]
        out_dir = Path(args.out) / date.strftime("%Y-%m-%d")
        out_dir.mkdir(parents=True, exist_ok=True)
        base, r0 = ordinal * 86400, int(g.index[0])
        if base + r0 > int(round(traci.simulation.getTime())):     # skipped days / late start
            for v in traci.vehicle.getIDList():
                traci.vehicle.remove(v)
            traci.simulationStep(base + r0)
        g = g.reset_index(drop=True)                             # row i == second r0+i of the day
        n = len(g)
        pt_leads, pt_extra = None, 0
        if args.mode == "actuated":
            pt_leads = json.load(open(HERE / "tram_leads.json"))["n"]
            pt_extra = K.Params.load(args.controller).tram_call_extra
        plan, cstats = schedule(g, dets, args.mode, args.seed, ordinal, pos, llen, n, pt_leads, pt_extra)
        pt_calls_n = set(cstats.pop("_pt_calls_n", []))
        if cstats:
            print(f"  event classes {date.date()}: {cstats}")
        rows = g[sg_map.ALL_SG].to_dict("records")
        sig = Signals(nlinks, sgl, args.amber, args.red_yellow)
        ctl, g_cur, g_next, calls_n, calls_s, applied = None, None, None, set(), set(), None
        if args.mode == "actuated":
            P = K.Params.load(args.controller)
            P.tram_priority = not args.no_tram_priority
            lead = json.load(open(HERE / "tram_leads.json"))
            rng_c = np.random.default_rng(args.seed + 7 + ordinal)
            calls_n, calls_s = pt_calls_n, set()
            dwell_of, dwell_t0 = {}, {}
            ctl = K.ActuatedController(P, "NS", 0)
            called, stopped = set(), set()
            wait_n, wait_s = {}, {}          # per simulated tram: seconds stopped at the stop line
            LEN_N, LEN_S = llen[LANES['N_tram_in'][0]], llen[LANES['S_tram_in'][0]]
            night_rows = (g["sg1"].to_numpy() == NIGHT_CODE)
            applied = np.zeros((n, 12), dtype=np.int8)
            was_night, last_obs = False, {k: False for k in ("d3", "d4", "d5", "d6")}

            def ctl_step(t):
                nonlocal was_night
                if night_rows[min(t, n - 1)]:
                    was_night = True
                    return {f"sg{k}": NIGHT_CODE for k in range(1, 13)}
                if was_night:
                    ctl.reset(t, "EW"); was_night = False
                # PT requests from the simulated trams themselves (V2I): N->S tram within pt_call_dist_n of the stop line;
                # S->N tram within pt_call_dist_s once it is moving again after its dwell
                if t in calls_n:
                    ctl.call_tram_n(t)
                for v in traci.edge.getLastStepVehicleIDs("N_tram_in"):
                    sp_, dist_ = traci.vehicle.getSpeed(v), LEN_N - traci.vehicle.getLanePosition(v)
                    wait_n.setdefault(v, 0)
                    if dist_ < 5 and sp_ < 0.3:
                        wait_n[v] += 1
                    if (v not in called and dist_ <= P.pt_call_dist_n and sp_ > 1.0) or (dist_ < 5 and sp_ < 0.3):
                        called.add(v); ctl.call_tram_n(t)          # request on approach, repeated while the tram waits at the line
                for v in traci.edge.getLastStepVehicleIDs("S_tram_in"):
                    sp_, dist_ = traci.vehicle.getSpeed(v), LEN_S - traci.vehicle.getLanePosition(v)
                    wait_s.setdefault(v, 0)
                    if dist_ < 5 and sp_ < 0.3:
                        wait_s[v] += 1
                    if sp_ < 0.3 and dist_ > 5:
                        stopped.add(v)
                        dwell_t0.setdefault(v, t)
                        if v not in called and v in dwell_of and (t - dwell_t0[v]) >= dwell_of[v] - P.pt_s_dwell_lead:
                            called.add(v); ctl.call_tram_s(t)       # announces itself shortly before the dwell ends                              # dwelling at the stop
                    if (v in stopped and v not in called and dist_ <= P.pt_call_dist_s and sp_ > 0.5) or (dist_ < 5 and sp_ < 0.3):
                        called.add(v); ctl.call_tram_s(t)           # request once the dwell is over, repeated at the line
                for ax in ("EW", "NS"):
                    if rng_c.random() < 1 / 50.0:
                        ctl.ped_press(ax, t)           # >= 1 pedestrian presses per cycle (assumption, no ped data)
                return ctl.step(t, last_obs)
            g_next = ctl_step(0)
        sim = np.zeros((n, len(dets)), dtype=np.int8)
        sig_ok = np.zeros(n, dtype=bool)
        removals = {}
        failed = 0
        KEDGES = ["E_in", "N_in", "S_in", "N_tram_in", "S_tram_in"]
        kpi = np.zeros((n, 2 * len(KEDGES)), dtype=np.int16)      # halted + total vehicles per approach edge, per second
        for i in range(n):
            if ctl is not None:
                g_cur, g_next = g_next, ctl_step(i + 1)
                applied[i, :] = [g_cur[f"sg{k}"] for k in range(1, 13)]
                state = sig.state(g_cur, g_next)
            else:
                state = sig.state(rows[i], rows[min(i + 1, n - 1)])
            traci.trafficlight.setRedYellowGreenState(TLS, state)
            for vid in removals.pop(i, ()):
                try:
                    traci.vehicle.remove(vid)
                except traci.TraCIException:
                    pass
            for act in plan.get(i, ()):
                nveh += 1
                vid = f"{act['det']}_{ordinal}_{act['a']}_{nveh}"
                lp = act["lp"]
                try:
                    traci.vehicle.add(vid, lp["route"] if act.get("drive") else "g_" + lp["edge"], typeID=act["type"],
                                      depart="now", departLane=str(act["lane"]),
                                      departPos=f"{act['p0']:.2f}", departSpeed=f"{act['v']:.2f}")
                    if act.get("drive"):
                        traci.vehicle.setLaneChangeMode(vid, 0)   # keep the connection-defined lane (d9/d10 lane identity)
                    if not act.get("drive"):
                        traci.vehicle.setSpeedMode(vid, 0)
                        traci.vehicle.setLaneChangeMode(vid, 0)   # ghosts must stay on their loop's lane
                        removals.setdefault(act["rm"], []).append(vid)
                    elif act.get("stop"):
                        e, sp, ln, du = act["stop"]
                        traci.vehicle.setStop(vid, e, pos=sp, laneIndex=ln, duration=du)
                        if ctl is not None:
                            dwell_of[vid] = du
                except traci.TraCIException as ex:
                    failed += 1
                    if failed <= 3:
                        print("  insertion failed:", vid, str(ex)[:90])
            traci.simulationStep()
            for k, d in enumerate(det_names):
                for lid in loop_ids[d]:
                    if traci.inductionloop.getLastStepOccupancy(lid) > 0 or traci.inductionloop.getLastStepVehicleNumber(lid) > 0:
                        sim[i, k] = 1
                        break
            sig_ok[i] = traci.trafficlight.getRedYellowGreenState(TLS) == state
            if args.mode != "exact":
                for ke, e_ in enumerate(KEDGES):
                    kpi[i, 2 * ke] = traci.edge.getLastStepHaltingNumber(e_)
                    kpi[i, 2 * ke + 1] = traci.edge.getLastStepVehicleNumber(e_)
            if ctl is not None:
                last_obs = {k: bool(sim[i, det_names.index(k)]) for k in ("d3", "d4", "d5", "d6")}
            if args.progress and i % 3600 == 0:
                print(f"  {date.date()} {i // 3600:02d}:00 veh={traci.vehicle.getIDCount():3d} wall={time.time() - t_wall:.0f}s", flush=True)
        for v in traci.vehicle.getIDList():
            traci.vehicle.remove(v)
        real = g.copy()
        simdf = g.copy()
        simdf[det_names] = sim
        if ctl is not None:
            simdf[sg_map.ALL_SG] = applied
        simdf.insert(0, "time", [date + pd.Timedelta(seconds=r0 + int(s)) for s in range(n)])
        simdf["sg_applied_ok"] = sig_ok.astype(int)
        simdf.to_csv(out_dir / "sim_detectors.csv", index=False)
        if args.mode != "exact":
            kd = pd.DataFrame(kpi, columns=[f"{e_}_{m_}" for e_ in KEDGES for m_ in ("halted", "veh")])
            kd.insert(0, "sec", np.arange(n) + r0)
            kd.to_csv(out_dir / "kpi_queues.csv.gz", index=False)
            if ctl is not None:
                pd.DataFrame(ctl.events, columns=["sec", "event"]).assign(sec=lambda d_: d_["sec"] + r0).to_csv(out_dir / "controller_events.csv", index=False)
        rep = C.compare(real, simdf, det_names, tol=args.tol)
        rep.attrs.update(sig_state_match_pct=float(sig_ok.mean() * 100), missing_seconds=int(missing.sum()),
                         insertion_failures=failed)
        rep.to_csv(out_dir / "report.csv")
        excl = set(args.exclude)
        if args.mode != "exact":      # injected arrivals are inputs, only emergent detectors are validation targets
            excl |= {d for d in det_names if dets[d].get("role") == "input"}
        wins = [("all", slice(0, n))]
        if args.holdout_from:
            hrow = int((pd.Timestamp(args.holdout_from) - date).total_seconds()) - r0
            if 0 < hrow < n:
                wins.append(("holdout", slice(hrow, n)))
        print(f"\n== {date.date()} ({args.mode}) ==")
        print(C.format_report(rep))
        for wname, sl in wins:
            sfx = "" if wname == "all" else "_" + wname
            rw, sw = real.iloc[sl], simdf.iloc[sl].set_index(real.index[sl])
            obs_c, sim_c = F.period_counts(rw, det_names, args.period), F.period_counts(sw, det_names, args.period)
            fid, agg = F.evaluate(obs_c, sim_c, det_names, exclude=tuple(excl))
            fid.to_csv(out_dir / f"fidelity{sfx}.csv")
            pd.Series(agg).to_csv(out_dir / f"fidelity_aggregate{sfx}.csv", header=["value"])
            if wname == "all":
                fid_days.append((date.date(), fid, agg, obs_c, sim_c))
            print(f"\n-- detector fidelity [{wname}] ({args.period} s periods; criterion detectors: "
                  f"{[d for d in det_names if d not in excl]}) --")
            print(F.format_fidelity(fid, agg))
            if ctl is not None:       # signal-timing fidelity of the closed-loop controller vs the field signals
                m = (rw["sg1"].to_numpy() != NIGHT_CODE)
                rs_ = rw[sg_map.ALL_SG][m].reset_index(drop=True)
                ss_ = simdf.iloc[sl][sg_map.ALL_SG][m].reset_index(drop=True)
                pg = SF.per_group(rs_, ss_, args.period)
                cyc, ks = SF.cycle_metrics(rs_, ss_)
                tm = SF.tram_metrics(rw[sg_map.ALL_SG], simdf.iloc[sl][sg_map.ALL_SG].set_index(rw.index),
                                     rises_idx(rw["d3"]), rises_idx(rw["d6"]),
                                     sim_waits={"n": list(wait_n.values()), "s": list(wait_s.values())} if wname == "all" else None)
                pg.to_csv(out_dir / f"signal_per_group{sfx}.csv"); cyc.to_csv(out_dir / f"signal_cycle{sfx}.csv")
                tm.to_csv(out_dir / f"signal_tram{sfx}.csv")
                pd.DataFrame(ks, index=["ks", "w1_s"]).to_csv(out_dir / f"signal_cycle_ks{sfx}.csv")
                if wname == "all":
                    pd.DataFrame({k: pd.Series(v) for k, v in ctl.stats.items()}).to_csv(out_dir / "controller_stats.csv", index=False)
                    pd.concat([pd.DataFrame({"kind": "N->S (sg12)", "wait_s": list(wait_n.values())}),
                               pd.DataFrame({"kind": "S->N (sg11)", "wait_s": list(wait_s.values())})]).to_csv(out_dir / "tram_waits_sim.csv", index=False)
                print(f"\n-- signal-timing fidelity [{wname}] (actuated controller vs field signals) --")
                print(pg[["starts_real", "starts_sim", "dur_med_real", "dur_med_sim", "dur_ks", "dur_w1_s"]].round(2).to_string())
                print(cyc.round(2).to_string()); print(tm.round(1).to_string())
        summary.append((date.date(), rep))
        if ctl is not None and ctl.stats["ped_wait"]:
            print("pedestrian wait (assumed press times) median/p90 s:", np.median(ctl.stats["ped_wait"]).round(1), np.percentile(ctl.stats["ped_wait"], 90).round(1))
    traci.close()
    pd.concat({str(d): r for d, r in summary}).to_csv(Path(args.out) / f"report_{args.mode}.csv")
    if len(fid_days) > 1:      # pooled over all days: every detector-day is one observation
        dets_ = list(fid_days[0][1].index)
        obs_all = pd.concat([o for *_, o, _ in fid_days], ignore_index=True)
        sim_all = pd.concat([s_ for *_, s_ in fid_days], ignore_index=True)
        pf, pa = F.evaluate(obs_all, sim_all, dets_, exclude=tuple(excl))
        pf.to_csv(Path(args.out) / "fidelity_all_days.csv")
        pd.Series(pa).to_csv(Path(args.out) / "fidelity_all_days_aggregate.csv", header=["value"])
        print(f"\n== all {len(fid_days)} days pooled (per-period series concatenated) ==")
        print(F.format_fidelity(pf, pa))
    print(f"\nwall time {time.time() - t_wall:.0f}s; outputs in {args.out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", nargs="+", required=True, help="CSV files / directories / globs (multi-day)")
    ap.add_argument("--mode", choices=["exact", "hybrid", "actuated"], default="exact",
                    help="exact: signals+detectors replayed; hybrid: signals replayed, arrivals injected; actuated: closed loop, own controller")
    ap.add_argument("--out", default="out")
    ap.add_argument("--from", dest="t_from", help="start timestamp, e.g. '2019-02-04 06:00:00'")
    ap.add_argument("--to", dest="t_to", help="end timestamp (exclusive)")
    ap.add_argument("--amber", type=int, default=3, help="amber seconds after a vehicle group leaves green (0 = none)")
    ap.add_argument("--red-yellow", type=int, default=1, help="red-yellow seconds before green on sg1..sg6 (paper: 1 s)")
    ap.add_argument("--period", type=int, default=3600, help="aggregation period (s) for the fidelity metrics (paper: 3600)")
    ap.add_argument("--exclude", nargs="*", default=["d1"], help="detectors left out of the acceptance criterion (paper: d1, >50 m from the stop line)")
    ap.add_argument("--controller", default=str(HERE / "controller_params.json"), help="calibrated controller parameters (actuated mode)")
    ap.add_argument("--holdout-from", help="timestamp: also report fidelity on the rows from here on (parameters are calibrated before it)")
    ap.add_argument("--no-tram-priority", action="store_true", help="actuated mode: disable PT priority")
    ap.add_argument("--tol", type=int, default=2, help="event matching tolerance in seconds")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--delay", type=int, default=100, help="sumo-gui delay (ms/step)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--progress", action="store_true")
    run(ap.parse_args())


if __name__ == "__main__":
    main()

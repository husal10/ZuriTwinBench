"""Fully-actuated signal controller of the Zürich intersection (closed-loop, RL-ready interface).

Structure derived from the Genser et al. 1-s data (see calibrate_controller.py):
  * two vehicle stages, EW = {sg1, sg4, sg5} and NS = {sg2, sg3, sg6}, served in a cycle EW -> NS -> EW ... (min recall,
    as in the data: 1447 EW and 1445 NS stages a day); pedestrian groups sg8/sg10 (EW) and sg7/sg9 (NS) lead/trail the
    vehicle greens by calibrated offsets; at least one pedestrian is assumed to call every pedestrian green;
  * each vehicle stage has a minimum green, a maximum green and a gap-out on its approach detector (EW: d5, NS: d4);
  * public transport has priority (TSP, sg11 and sg12): a call for the N->S tram (sg12) terminates the running stage once its minimum green is
    served, sg12 turns green `t12_delay` s after the vehicle stage ends and stays until the tram has cleared d3 (+ margin);
    a call for the S->N tram (sg11) grants sg11 `t11_offset` s into the next NS stage and holds it until d6 has cleared;
  * night mode (code 8) is imposed from outside (it comes from the data / a schedule).
The controller only outputs the green/red code per signal group (0/1/8, the CSV encoding); amber and red-yellow are added
by replay.Signals.  Everything a learned policy needs is exposed: step(t, obs) -> greens, plus the parameters in `Params`.
"""
import json
from dataclasses import dataclass, field, asdict
import numpy as np

VEH = {"EW": ["sg1", "sg4", "sg5"], "NS": ["sg2", "sg3", "sg6"]}
PED = {"EW": ["sg8", "sg10"], "NS": ["sg7", "sg9"]}
OTHER = {"EW": "NS", "NS": "EW"}
DET = {"EW": "d5", "NS": "d4"}


@dataclass
class Params:
    min_green: dict = field(default_factory=lambda: {"EW": 15, "NS": 10})
    min_green_pri: dict = field(default_factory=lambda: {"EW": 11, "NS": 10})   # earliest end of a stage under tram priority
    max_green: dict = field(default_factory=lambda: {"EW": 18, "NS": 19})
    gap: dict = field(default_factory=lambda: {"EW": 3.0, "NS": 3.0})          # gap-out time on the approach detector (s)
    gap_to_next: dict = field(default_factory=lambda: {"EW": 4, "NS": 17})     # veh end -> next veh start (s)
    start_off: dict = field(default_factory=lambda: {"sg1": 0, "sg4": 0, "sg5": 0, "sg2": 0, "sg3": 0, "sg6": 0})
    end_early: dict = field(default_factory=lambda: {"sg1": 0, "sg4": 0, "sg5": 0, "sg2": 0, "sg3": 0, "sg6": 0})
    ped_lead: dict = field(default_factory=lambda: {"EW": 11, "NS": 1})        # ped green starts this long before the veh stage
    ped_tail: dict = field(default_factory=lambda: {"EW": 0, "NS": 8})         # ... and ends this long after it
    t12_delay: int = 3          # veh stage end -> sg12 green
    t12_min: int = 10
    t12_max: int = 32
    t12_margin: int = 3         # sg12 held this long after d3 clears
    t12_to_next: int = 4        # sg12 end -> next vehicle stage start
    t11_offset: int = 6         # sg11 start after the NS stage start
    t11_min: int = 7
    t11_max: int = 23
    t11_margin: int = 4         # sg11 held this long after d6 clears
    tram_priority: bool = True
    tram_call_extra: int = 0     # N->S PT request is sent this many s earlier than the recorded green lead (fitted)
    tram_s_call_extra: int = 0   # same for the S->N tram (request relative to its d6 arrival)
    pt_call_dist_n: float = 130.0   # closed loop: N->S tram sends its PT request when this close (m) to the stop line
    pt_call_dist_s: float = 40.0    # closed loop: S->N tram (after its dwell)
    pt_s_dwell_lead: int = 14       # closed loop: S->N tram announces itself this long before its dwell ends (s)
    recall: bool = True         # serve both stages every cycle (data); False = skip a stage without any call

    @staticmethod
    def load(path):
        raw = json.load(open(path))
        p = Params()
        for k, v in raw.items():
            if hasattr(p, k):
                setattr(p, k, v)
        return p

    def save(self, path):
        json.dump(asdict(self), open(path, "w"), indent=1)


class ActuatedController:
    def __init__(self, params=None, start_stage="NS", t0=0):
        self.p = params or Params()
        self.reset(t0, start_stage)

    # ---- life cycle -------------------------------------------------------------------------------------------
    def reset(self, t, stage="NS"):
        """(Re)start with `stage` having just begun green at time t (used at the start and after night mode)."""
        self.cur, self.state = stage, "VEH"
        self.t0 = t
        self.last_call = t
        self.end_at = {}                    # sg -> time its green ends (set when the stage end is decided)
        self.stage_end = None
        self.next_stage, self.next_start = None, None
        self.ped_on = {g: (t - 1, None) for g in PED[stage]}
        self.pend = {"n": False, "s": False}         # tram calls (N->S = sg12 'n', S->N = sg11 's')
        self.t12 = None                              # [start, end or None]
        self.t11 = None
        self.t11_pending_since = None
        self.stats = {"ped_wait": [], "tram_n_wait": [], "tram_s_wait": []}
        self.events = []                 # (t, kind): stage / TSP actions, for event studies
        self._ped_press = {"EW": [], "NS": []}       # press times of waiting pedestrians per axis
        self._call_t = {"n": None, "s": None}

    def call_tram_n(self, t):
        if self.t12 is not None and (self.t12[1] is None or t < self.t12[1] + 2):
            return                                   # the tram stage is running: the request is already being served
        if not self.pend["n"]:
            self.pend["n"], self._call_t["n"] = True, t

    def call_tram_s(self, t):
        if self.t11 is not None and (self.t11[1] is None or t < self.t11[1] + 2):
            return
        if not self.pend["s"]:
            self.pend["s"], self._call_t["s"] = True, t

    def ped_press(self, axis, t):
        self._ped_press[axis].append(t)

    # ---- core ---------------------------------------------------------------------------------------------------
    def step(self, t, obs):
        """obs: dict of booleans {'d4','d5','d3','d6'} (loop occupied in the last second). Returns {sg: 0/1} for second t."""
        p = self.p
        if self.t11 is not None and self.t11[1] is not None and t >= self.t11[1]:
            self.t11 = None
        if self.t12 is not None and self.t12[1] is not None and t >= self.t12[1]:
            self.t12 = None
        if obs.get("d3"):
            self._d3_last = t
        if obs.get("d6"):
            self._d6_last = t
        if obs.get(DET[self.cur]) and self.state == "VEH":
            self.last_call = t
        # -- vehicle stage running: decide the end
        if self.state == "VEH" and not self.end_at:
            dur = t - self.t0
            gap_out = (t - self.last_call) >= p.gap[self.cur]
            tram = p.tram_priority and (self.pend["n"] or (self.cur == "EW" and self.pend["s"]))   # TSP: sg12 pre-empts, sg11 brings the NS stage forward
            if dur >= p.max_green[self.cur] or (dur >= p.min_green[self.cur] and gap_out) or (tram and dur >= p.min_green_pri[self.cur]):
                self._end_stage(t)
        # -- between stages
        if self.state == "GAP":
            if self.t12 is not None and self.t12[1] is None:
                pass                                   # waiting for the tram to clear
            elif t >= self.next_start:
                self._start_stage(t, self.next_stage)
        # -- N->S tram stage (sg12)
        if self.pend["n"] and p.tram_priority and self.t12 is None and self.state == "GAP" and t >= self.t12_ready:
            self.t12 = [t, None]
            self.events.append((t, "T12"))
            self._d3_last = None
            self.stats["tram_n_wait"].append(t - self._call_t["n"])
            self.pend["n"] = False
        if self.t12 is not None and self.t12[1] is None:
            dur = t - self.t12[0]
            cleared = (getattr(self, "_d3_last", None) is not None) and (not obs.get("d3")) and (t - self._d3_last) >= p.t12_margin
            if dur >= p.t12_max or (dur >= p.t12_min and cleared):
                self.t12[1] = t
                self.next_start = max(self.next_start, t + p.t12_to_next)
        # -- S->N tram (sg11) rides on the NS stage
        if self.pend["s"] and self.t11 is None and self.state == "VEH" and self.cur == "NS" and (t - self.t0) >= p.t11_offset:
            self.t11 = [t, None]
            self.events.append((t, "T11"))
            self._d6_last = None
            self.stats["tram_s_wait"].append(t - self._call_t["s"])
            self.pend["s"] = False
        if self.t11 is not None and self.t11[1] is None:
            dur = t - self.t11[0]
            cleared = (getattr(self, "_d6_last", None) is not None) and (t - self._d6_last) >= p.t11_margin and not obs.get("d6")
            if dur >= p.t11_max or (dur >= p.t11_min and cleared):
                self.t11[1] = t
        return self._greens(t)

    # ---- transitions ----------------------------------------------------------------------------------------------
    def _end_stage(self, t):
        p, cur = self.p, self.cur
        D = max(p.end_early[g] for g in VEH[cur])
        if p.tram_priority and (self.pend["n"] or (cur == "EW" and self.pend["s"])) and (t - self.t0) < p.min_green[cur]:
            self.events.append((t, "early_termination_" + cur))      # stage cut short by TSP
        self.end_at = {g: t + D - p.end_early[g] for g in VEH[cur]}
        self.stage_end = t + D
        for g in PED[cur]:
            self.end_at[g] = self.stage_end + p.ped_tail[cur]
        nxt = OTHER[cur]
        self.state, self.next_stage = "GAP", nxt
        self.next_start = self.stage_end + p.gap_to_next[cur]
        self.t12_ready = self.stage_end + p.t12_delay
        if p.tram_priority and self.pend["n"]:
            self.next_start = max(self.next_start, self.t12_ready + p.t12_min + p.t12_to_next)
        self._old = (cur, dict(self.end_at), self.stage_end)

    def _start_stage(self, t, stage):
        self.cur, self.state = stage, "VEH"
        self.events.append((t, "stage_" + stage))
        self.t0, self.last_call = t, t
        self.end_at, self.stage_end = {}, None
        self.t12 = None if self.t12 is None or self.t12[1] is not None else self.t12
        # pedestrians that were pressed are served now (they started up to `ped_lead` s earlier)
        axis = stage
        waits = [max(0, t - self.p.ped_lead[axis] - pt) for pt in self._ped_press[axis]]
        self.stats["ped_wait"] += waits
        self._ped_press[axis] = []

    # ---- outputs --------------------------------------------------------------------------------------------------
    def _greens(self, t):
        p = self.p
        g = {s: 0 for s in ["sg%d" % i for i in range(1, 13)]}
        g["sg12"] = 1 if (self.t12 is not None and (self.t12[1] is None or t < self.t12[1])) else 0
        # running or just-ended vehicle stage groups
        if self.state == "VEH":
            for s in VEH[self.cur]:
                if t >= self.t0 + p.start_off[s] and t < self.end_at.get(s, 1e18):
                    g[s] = 1
            for s in PED[self.cur]:
                if t < self.end_at.get(s, 1e18):
                    g[s] = 1
        else:
            old_cur, old_end, _ = self._old
            for s, te in old_end.items():
                if t < te:
                    g[s] = 1                                           # ped tail / groups ending later than the decision
            # pedestrian lead of the next stage
            tram_active = (self.t12 is not None and self.t12[1] is None) or (self.pend["n"] and p.tram_priority)
            if t >= self.next_start - p.ped_lead[self.next_stage] and not tram_active and not g["sg12"]:
                for s in PED[self.next_stage]:
                    g[s] = 1
        prev = getattr(self, "_prev", {})
        if self.t12 is not None and (self.t12[1] is None or t < self.t12[1]):
            g["sg12"] = 1
            for s in PED["NS"]:
                g[s] = max(g[s], prev.get(s, 0))                       # NS crossings stay green through the tram stage
        if self.t11 is not None and (self.t11[1] is None or t < self.t11[1]):
            g["sg11"] = 1
            for s in PED["NS"]:
                g[s] = max(g[s], prev.get(s, 0))
        self._prev = dict(g)
        return g

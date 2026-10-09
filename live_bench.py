"""
live_bench.py -- the sim-to-real benchmark, scored while the cell runs.

The digital twin (twin.py) drives the benchmark's reference simulation, S0,
and optionally a CANDIDATE model beside it, with the real controller's own
commanded joints, packet by packet. Each tool-point comparison -- how far
each simulated tool is from the real one, in millimetres -- comes here.

Two things are kept from those numbers:

  * a rolling window (the last few seconds), so the console can show the
    gap as it is right now, whatever the arm is doing;
  * one score per RECORDED run, worked out exactly as the offline harness
    does it (sonair_benchmark.scoring.score_run): the median and the 95th
    percentile of the tool-position error over the run, and for a candidate
    GCR = 1 - err(candidate) / err(S0) on each. Runs are grouped by their
    condition cell and averaged, as the leaderboard averages them.

What it is NOT: the score of record. The record is the offline replay of
each run file, because that is reproducible and does not depend on whether
this PC kept up. The live score runs the same models on the same input and
should agree closely; where it does not, the replay is right. The page says
so wherever it shows a live number.

Simulated-cell rehearsals are scored too, so the whole chain can be tried
without the robot, but they are kept in their own book and labelled.
"""
from __future__ import annotations

import json
import math
import statistics
import threading
import time
from collections import deque
from pathlib import Path

try:
    from sonair_benchmark.metrics import percentile
except Exception:                                   # noqa: BLE001
    def percentile(xs, p):
        xs = sorted(xs)
        if not xs:
            return 0.0
        k = (len(xs) - 1) * p / 100.0
        lo, hi = math.floor(k), math.ceil(k)
        return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)

HERE = Path(__file__).resolve().parent
REFERENCE = "S0"
WINDOW_S = 5.0
MIN_RUN_POINTS = 20         # fewer comparisons than this and a run is not scored


def gcr(pred: float, base: float) -> float | None:
    """The benchmark's gap-closure ratio, with the same guard scoring uses."""
    if base is None or pred is None or base <= 1e-9:
        return None
    return 1.0 - pred / base


class LiveBench:
    def __init__(self, out_dir: Path | None = None, window_s: float = WINDOW_S):
        self.out_dir = Path(out_dir) if out_dir else HERE / "results"
        self.window_s = window_s
        self._lock = threading.Lock()
        self._win: deque = deque()          # (t, {arm: mm})
        self._run = None                    # the run being recorded now
        self.runs: list[dict] = []          # scored runs, this session
        self.sim_runs: list[dict] = []      # simulated-cell rehearsals
        self.started_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self.models: dict[str, str] = {}    # arm name -> what it is

    # -- input ---------------------------------------------------------------
    def add(self, t: float, errs_mm: dict, run: dict | None) -> None:
        """
        One comparison: `errs_mm` maps each simulated arm to its tool-point
        error at time `t`. `run` is the recorder's current run (run_id, cell,
        experiment, simulated) or None when nothing is being recorded.
        """
        finished = None
        with self._lock:
            self._win.append((t, dict(errs_mm)))
            while self._win and t - self._win[0][0] > self.window_s:
                self._win.popleft()
            rid = (run or {}).get("run_id")
            if self._run is not None and self._run["run_id"] != rid:
                finished = self._close_locked()
            if rid and self._run is None:
                self._run = {"run_id": rid, "cell": run.get("cell", ""),
                             "experiment": run.get("experiment", "E2"),
                             "simulated": bool(run.get("simulated")),
                             "t0": t, "errs": {}}
            if self._run is not None:
                for k, v in errs_mm.items():
                    self._run["errs"].setdefault(k, []).append(float(v))
        if finished:
            self._save()

    def run_ended(self) -> None:
        """The recorder stopped: score the run now rather than on the next packet."""
        with self._lock:
            done = self._close_locked() if self._run is not None else None
        if done:
            self._save()

    def reset_session(self) -> None:
        with self._lock:
            self.runs, self.sim_runs, self._run = [], [], None
            self.started_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self._save()

    # -- scoring ---------------------------------------------------------------
    def _close_locked(self):
        r, self._run = self._run, None
        base = r["errs"].get(REFERENCE) or []
        if len(base) < MIN_RUN_POINTS:
            return None
        out = {"run_id": r["run_id"], "cell": r["cell"],
               "experiment": r["experiment"], "simulated": r["simulated"],
               "n": len(base),
               "duration_s": round(max(0.0, self._win[-1][0] - r["t0"])
                                   if self._win else 0.0, 2),
               "models": {}}
        b_med, b_p95 = statistics.median(base), percentile(base, 95)
        for name, e in r["errs"].items():
            if len(e) < MIN_RUN_POINTS:
                continue
            med, p95 = statistics.median(e), percentile(e, 95)
            row = {"median_mm": round(med, 3), "p95_mm": round(p95, 3)}
            if name != REFERENCE:
                row["gcr_median"] = _r(gcr(med, b_med))
                row["gcr_p95"] = _r(gcr(p95, b_p95))
            out["models"][name] = row
        (self.sim_runs if r["simulated"] else self.runs).append(out)
        return out

    # -- output ----------------------------------------------------------------
    def _window_stats(self) -> dict:
        names = sorted({k for _, e in self._win for k in e})
        rows = {}
        for n in names:
            xs = [e[n] for _, e in self._win if n in e]
            if xs:
                rows[n] = {"median_mm": round(statistics.median(xs), 3),
                           "p95_mm": round(percentile(xs, 95), 3),
                           "now_mm": round(xs[-1], 3)}
        base = rows.get(REFERENCE)
        if base:
            for n, row in rows.items():
                if n != REFERENCE:
                    row["gcr_median"] = _r(gcr(row["median_mm"], base["median_mm"]))
                    row["gcr_p95"] = _r(gcr(row["p95_mm"], base["p95_mm"]))
        return rows

    @staticmethod
    def aggregate(runs: list[dict]) -> dict:
        """Mean over runs, and per cell -- as score_submission aggregates."""
        names = sorted({k for r in runs for k in r["models"]})
        overall, cells = {}, {}
        for n in names:
            have = [r for r in runs if n in r["models"]]
            if not have:
                continue
            row = {"runs": len(have),
                   "median_mm": _r(statistics.fmean(r["models"][n]["median_mm"] for r in have)),
                   "p95_mm": _r(statistics.fmean(r["models"][n]["p95_mm"] for r in have))}
            g = [r["models"][n].get("gcr_p95") for r in have]
            g = [x for x in g if x is not None]
            if n != REFERENCE and g:
                row["gcr_p95"] = _r(statistics.fmean(g))
                gm = [r["models"][n].get("gcr_median") for r in have]
                gm = [x for x in gm if x is not None]
                row["gcr_median"] = _r(statistics.fmean(gm)) if gm else None
            overall[n] = row
        for r in runs:
            c = cells.setdefault(r["cell"], {"runs": 0, "models": {}})
            c["runs"] += 1
            for n, m in r["models"].items():
                c["models"].setdefault(n, []).append(m)
        for c in cells.values():
            c["models"] = {
                n: {"p95_mm": _r(statistics.fmean(x["p95_mm"] for x in ms)),
                    "median_mm": _r(statistics.fmean(x["median_mm"] for x in ms)),
                    **({"gcr_p95": _r(statistics.fmean(
                        [x["gcr_p95"] for x in ms if x.get("gcr_p95") is not None]))}
                       if n != REFERENCE and any(x.get("gcr_p95") is not None for x in ms)
                       else {})}
                for n, ms in c["models"].items()}
        return {"overall": overall, "cells": cells}

    def snapshot(self) -> dict:
        with self._lock:
            live = self._window_stats()
            runs, sims = list(self.runs), list(self.sim_runs)
            cur = None
            if self._run is not None:
                cur = {"run_id": self._run["run_id"], "cell": self._run["cell"],
                       "n": len(self._run["errs"].get(REFERENCE, []))}
        return {
            "reference": REFERENCE, "models": dict(self.models),
            "window_s": self.window_s, "live": live, "recording": cur,
            "session": {"started_utc": self.started_utc, "runs": runs[-200:],
                        **self.aggregate(runs)},
            "rehearsal": {"runs": sims[-200:], **self.aggregate(sims)},
            "note": ("Live preview, scored as the offline harness scores. The "
                     "score of record is the offline replay of each run file."),
        }

    def _save(self) -> None:
        try:
            self.out_dir.mkdir(parents=True, exist_ok=True)
            doc = self.snapshot()
            doc["saved_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            tmp = self.out_dir / "live_score.json.tmp"
            tmp.write_text(json.dumps(doc, indent=1), encoding="utf-8")
            tmp.replace(self.out_dir / "live_score.json")
        except Exception:                           # noqa: BLE001
            pass


def _r(v, nd=4):
    return None if v is None else round(float(v), nd)


LIVE = LiveBench()

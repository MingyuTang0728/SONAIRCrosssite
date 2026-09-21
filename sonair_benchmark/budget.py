"""
Phase 2 — the error budget, i.e. the measurement floor of the rig.

Five rows, each a measured number from Phase 0 or Phase 1, never an estimate:

    inertial noise floor        from imu.stationary_stats
    tracker distortion          from the Phase 0 working-volume sweep
    frame fit residual          from the base-to-generator registration
    temporal alignment residual from clock.tap_alignment
    arm repeatability           from the manufacturer figure, verified once

The sum is the floor beneath which no gap claim can be made. Gate B of the
plan is exactly the comparison implemented in `gate_b`: if the calibration
residual is the same order as the gap, the benchmark is measuring itself.

Keeping this in code rather than in a spreadsheet is the point — every metric
in metrics.py is reported against it automatically, so a result can never be
published without its own floor attached.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path


@dataclass
class BudgetRow:
    name: str
    position_mm: float = 0.0      # contribution to position error, mm
    orientation_deg: float = 0.0  # contribution to orientation error, deg
    temporal_ms: float = 0.0      # contribution to timing error, ms
    source: str = ""              # which phase / measurement produced it
    measured: bool = False        # False means it is still a placeholder


@dataclass
class ErrorBudget:
    calib_version: str = "calib-0"
    rows: list[BudgetRow] = field(default_factory=list)
    notes: str = ""

    @classmethod
    def blank(cls, calib_version: str = "calib-0") -> "ErrorBudget":
        """The five mandatory rows, unmeasured. Fill them as Phase 0/1 land."""
        return cls(calib_version=calib_version, rows=[
            BudgetRow("inertial_noise_floor", source="Phase 0 stationary log"),
            BudgetRow("tracker_distortion", source="Phase 0 working-volume sweep"),
            BudgetRow("frame_fit_residual", source="Phase 2 point-set registration"),
            BudgetRow("temporal_alignment", source="Phase 1 tap verification"),
            BudgetRow("arm_repeatability", source="UR5e spec, verified once"),
        ])

    def set(self, name: str, *, position_mm: float = 0.0, orientation_deg: float = 0.0,
            temporal_ms: float = 0.0, source: str = "") -> None:
        for r in self.rows:
            if r.name == name:
                r.position_mm = position_mm
                r.orientation_deg = orientation_deg
                r.temporal_ms = temporal_ms
                r.measured = True
                if source:
                    r.source = source
                return
        self.rows.append(BudgetRow(name, position_mm, orientation_deg,
                                   temporal_ms, source, True))

    # --- the floor -----------------------------------------------------------

    def floor_position_mm(self) -> float:
        """
        Root-sum-square, not arithmetic sum.

        The five rows are independent measurements, so RSS is the honest
        combination; a plain sum would overstate the floor and let a real gap
        be dismissed as noise.
        """
        return math.sqrt(sum(r.position_mm ** 2 for r in self.rows))

    def floor_orientation_deg(self) -> float:
        return math.sqrt(sum(r.orientation_deg ** 2 for r in self.rows))

    def floor_temporal_ms(self) -> float:
        return math.sqrt(sum(r.temporal_ms ** 2 for r in self.rows))

    def unmeasured(self) -> list[str]:
        return [r.name for r in self.rows if not r.measured]

    def is_complete(self) -> bool:
        return not self.unmeasured()

    # --- Gate B --------------------------------------------------------------

    def gate_b(self, measured_gap_position_mm: float,
               measured_gap_orientation_deg: float = 0.0,
               ratio_required: float = 3.0) -> dict:
        """
        Gate B of the deployment plan.

        PASS      the gap is at least `ratio_required` times the floor, so the
                  benchmark is measuring the gap
        MARGINAL  between 1x and the required ratio: publishable only as an
                  upper bound, and stated as such
        FAIL      the gap is at or below the floor: Phase 3 should not start,
                  or the result is "the gap is below the measurement floor of
                  this rig", which is itself an honest publishable finding
        """
        fp = self.floor_position_mm()
        fo = self.floor_orientation_deg()
        ratios = []
        if fp > 0:
            ratios.append(measured_gap_position_mm / fp)
        if fo > 0 and measured_gap_orientation_deg > 0:
            ratios.append(measured_gap_orientation_deg / fo)
        worst = min(ratios) if ratios else float("inf")
        if worst >= ratio_required:
            verdict = "PASS"
        elif worst >= 1.0:
            verdict = "MARGINAL"
        else:
            verdict = "FAIL"
        return {
            "verdict": verdict,
            "worst_ratio": worst,
            "ratio_required": ratio_required,
            "floor_position_mm": fp,
            "floor_orientation_deg": fo,
            "gap_position_mm": measured_gap_position_mm,
            "gap_orientation_deg": measured_gap_orientation_deg,
            "incomplete_rows": self.unmeasured(),
        }

    # --- io ------------------------------------------------------------------

    def save(self, path: str | Path) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

    def to_dict(self) -> dict:
        return {
            "calib_version": self.calib_version,
            "rows": [asdict(r) for r in self.rows],
            "floor": {
                "position_mm": self.floor_position_mm(),
                "orientation_deg": self.floor_orientation_deg(),
                "temporal_ms": self.floor_temporal_ms(),
            },
            "complete": self.is_complete(),
            "unmeasured": self.unmeasured(),
            "notes": self.notes,
        }

    @classmethod
    def load(cls, path: str | Path) -> "ErrorBudget":
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        b = cls(calib_version=d.get("calib_version", "calib-0"),
                notes=d.get("notes", ""))
        b.rows = [BudgetRow(**r) for r in d.get("rows", [])]
        return b


def from_phase0(noise_floor, tap_spread_s: float, tracker_distortion_mm: float,
                frame_fit_residual_mm: float, arm_repeatability_mm: float = 0.03,
                calib_version: str = "calib-0",
                lever_arm_m: float = 0.15) -> ErrorBudget:
    """
    Assemble a budget straight out of the Phase 0/1/2 measurements.

    `lever_arm_m` converts an orientation error into the position error it
    causes at the tool tip. Without that conversion the orientation row and
    the position rows are in different units and cannot be combined, which is
    how orientation quietly drops out of most error budgets.
    """
    b = ErrorBudget.blank(calib_version)
    ori_deg = noise_floor.orientation_noise_deg() if noise_floor else 0.0
    b.set("inertial_noise_floor",
          orientation_deg=ori_deg,
          position_mm=math.radians(ori_deg) * lever_arm_m * 1000.0,
          source="Phase 0 stationary log")
    b.set("tracker_distortion", position_mm=tracker_distortion_mm,
          source="Phase 0 working-volume sweep")
    b.set("frame_fit_residual", position_mm=frame_fit_residual_mm,
          source="Phase 2 point-set registration")
    b.set("temporal_alignment", temporal_ms=tap_spread_s * 1000.0,
          source="Phase 1 tap verification")
    b.set("arm_repeatability", position_mm=arm_repeatability_mm,
          source="UR5e spec, verified once")
    return b

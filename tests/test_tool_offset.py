"""
The tool offset must be composed onto the flange, never substituted for it.

`attachment_site` in the menagerie UR5e is not at its parent body's origin --
it sits 100 mm out along the wrist, because that is where the flange is. A tool
offset read off the pendant is measured FROM THAT FLANGE, so writing it into
`site_pos` discards the flange's own position and moves the measurement point
100 mm.

That is the same class of error as the one the replay's frame check exists to
catch, except pointing the other way: it fired on exactly the people who
followed the check's advice and passed `--tcp-offset`. It surfaced when a dry
run gave its stand-in robot the simulator's own forward kinematics, so the two
tool positions had to agree and instead differed by 94 mm.

Run:  python tests/test_tool_offset.py    (needs mujoco + a menagerie checkout)
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

Q = [0.867, -1.621, 2.034, -1.953, -1.598, 3.203]


def _arm():
    import sim_mujoco
    ok, why = sim_mujoco.available()
    if not ok:
        return None, why
    men = Path(os.environ.get("MENAGERIE", "mujoco_menagerie"))
    if not (men / "universal_robots_ur5e" / "scene.xml").exists():
        return None, (f"no menagerie at {men}; clone it or set MENAGERIE=<path>")
    a = sim_mujoco.Arm(sim_mujoco.ensure_model(men))
    a.reset(Q)
    a.settle(Q, 0.5)
    return a, ""


def test_tool_offset():
    arm, why = _arm()
    if arm is None:
        print("  skipped:", why)
        return
    flange = list(arm.tcp_position()[:3])

    # A zero offset is a no-op. Before the fix this moved the point 100 mm.
    arm.set_tool_offset([0, 0, 0])
    d = math.dist(flange, arm.tcp_position()[:3])
    assert d < 1e-9, f"a zero tool offset moved the tool point {d * 1000:.1f} mm"

    # A 50 mm offset moves the point by exactly 50 mm.
    arm.set_tool_offset([0, 0, 0.05])
    moved = math.dist(flange, arm.tcp_position()[:3])
    assert abs(moved - 0.05) < 1e-6, f"moved {moved * 1000:.2f} mm, wanted 50"

    # Applying it again lands in the same place rather than compounding.
    again = list(arm.tcp_position()[:3])
    arm.set_tool_offset([0, 0, 0.05])
    assert math.dist(again, arm.tcp_position()[:3]) < 1e-12, "offsets compounded"

    # And it is reversible.
    arm.set_tool_offset([0, 0, 0])
    assert math.dist(flange, arm.tcp_position()[:3]) < 1e-9

    # A rotation-only offset pivots about the flange without translating it.
    arm.set_tool_offset([0, 0, 0, 0, 0, math.pi / 4])
    d = math.dist(flange, arm.tcp_position()[:3])
    assert d < 1e-9, f"a pure rotation moved the tool point {d * 1000:.3f} mm"
    print("  pass  test_tool_offset")


if __name__ == "__main__":
    test_tool_offset()

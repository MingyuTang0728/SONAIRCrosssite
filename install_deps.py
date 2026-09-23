"""
install_deps.py — install this project's dependencies into THIS interpreter.

Run it with the same Python you run the agent with:

    python install_deps.py

That is the whole point. Every other way of saying "install into the right
Python" has a way to go wrong:

  `pip install x`            lands wherever PATH points, which on a machine
                             with a system Python, a virtual environment and
                             an IDE interpreter is a coin toss. It succeeds,
                             the import still fails, and the instruction looks
                             wrong.
  `"C:\\path\\python.exe" -m pip install x`
                             is correct and PowerShell refuses it: a command
                             beginning with a quoted string is parsed as a
                             string expression, giving "unexpected token '-m'"
                             unless you remember the `&` call operator.
  `cd C:\\path\\python.exe`    is what people type when a path appears in an
                             instruction, and `cd` to an executable fails in a
                             way that says nothing about the real task.

Running a script sidesteps all three: whatever interpreter starts this file is
by definition the one the packages land in, and there is no path to quote.

It installs only what is missing, and it never touches a package that already
imports — so it is safe to run repeatedly, and safe to run on a machine where
pyrealsense2 was a struggle to get working.
"""
from __future__ import annotations

import importlib
import subprocess
import sys

# import name, pip name, why you want it, is it required
DEPS = [
    ("websockets",   "websockets",    "the agent cannot start without it", True),
    ("numpy",        "numpy",         "3D scanning and calibration", False),
    ("cv2",          "opencv-python", "camera images, hand-eye calibration, "
                                      "the inspection pipeline", False),
    ("zmq",          "pyzmq",         "FusionHub's External Output node", False),
    ("pyrealsense2", "pyrealsense2",  "the RealSense camera", False),
    ("serial",       "pyserial",      "an IMU on a COM port (not needed over "
                                      "the network)", False),
]


def probe(mod: str) -> tuple[bool, str]:
    """
    Can this interpreter actually import it, and if not, why?

    The reason matters. "No module named x" means pip has not put it there.
    Anything else — a missing system library, a wheel built for another
    architecture — means pip DID put it there and it still will not load, and
    that has a completely different fix. Reporting both as "missing" sends
    people to reinstall a package that is already installed.
    """
    importlib.invalidate_caches()
    try:
        importlib.import_module(mod)
        return True, ""
    except ImportError as e:
        return False, str(e)
    except Exception as e:  # noqa: BLE001
        return False, f"{type(e).__name__}: {e}"


def present(mod: str) -> bool:
    return probe(mod)[0]


def _wheel_advice(pip_name: str) -> str:
    if pip_name == "pyrealsense2":
        return ("Intel does not publish a wheel for every Python version. If "
                "this one has none, install Python 3.11 alongside and make a "
                "virtual environment with it:\n"
                "      py -3.11 -m venv .venv\n"
                "      .\\.venv\\Scripts\\Activate.ps1\n"
                "      python install_deps.py")
    return "Check the network, or a proxy, and try again."


def main() -> int:
    print("Installing into:")
    print(f"  {sys.executable}")
    print(f"  Python {sys.version.split()[0]}")
    if sys.prefix != sys.base_prefix:
        print("  (a virtual environment — good, this is the isolated one)")
    print()

    missing = [(m, p, why, req) for m, p, why, req in DEPS if not present(m)]
    for mod, pip_name, why, _ in DEPS:
        mark = "already there" if present(mod) else "MISSING"
        print(f"  {pip_name:<16} {mark:<14} {why}")
    print()

    if not missing:
        print("Nothing to do — everything this project uses is already "
              "installed in this interpreter.")
        return 0

    names = [p for _, p, _, _ in missing]
    print("Installing: " + " ".join(names))
    print()
    rc = subprocess.call([sys.executable, "-m", "pip", "install", *names])

    print()
    still = [(m, p, why, req) for m, p, why, req in missing if not present(m)]
    if not still:
        print("All installed. Restart the agent so it picks them up — pip "
              "cannot load a package into a process that is already running.")
        return 0

    # Report per package rather than as one failure: pyrealsense2 has no wheel
    # on some Python versions, and "install failed" would suggest the whole
    # thing is broken when in fact only the camera is affected.
    print("Still not usable after the attempt:")
    for mod, pip_name, why, req in still:
        _, err = probe(mod)
        not_installed = "no module named" in err.lower()
        print(f"  {pip_name} — you lose: {why}")
        if not_installed:
            print(f"    pip could not install it. {_wheel_advice(pip_name)}")
        else:
            # Installed, and still will not load. Pointing at pip here is the
            # wrong direction entirely.
            print(f"    It IS installed, but it will not load: {err}")
            print("    That is a missing system library or a wheel built for "
                  "a different machine, not a pip problem. Reinstalling it "
                  "will not help.")
            if pip_name == "pyrealsense2":
                print("    On Windows this usually means the Visual C++ "
                      "redistributable is missing — install it from "
                      "Microsoft, then try again.")
    if any(req for _, _, _, req in still):
        print()
        print("One of these is REQUIRED — the agent will not start without it.")
        return 1
    print()
    print("Everything required is present; the rest are optional features.")
    return 0 if rc == 0 else rc


if __name__ == "__main__":
    raise SystemExit(main())

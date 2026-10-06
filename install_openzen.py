"""
install_openzen.py -- put LP-Research's OpenZen, and a Python it runs on, in
%LOCALAPPDATA%\SONAIR\openzen (shared by every copy of the project). Run it once,
with any Python, on the PC the sensor talks to:

    python install_openzen.py

What it fetches, and why each:

  * OpenZen for Windows, built for Python 3.11 -- LP-Research's own SDK for
    LPMS sensors, from their download page on Bitbucket. It is the library
    their own tools are built on; FusionHub is a fusion product layered on
    top, not the sensor's driver.
  * The official embeddable Python 3.11 from python.org: a 10 MB folder, not
    an installation. OpenZen's prebuilt module only loads in the Python it
    was built for, and the console's agent runs on whatever Python this PC
    has, so the sensor is read by this small private Python in a process of
    its own (openzen_bridge.py). Nothing on the system is changed.

Then it starts OpenZen once to prove it loads, and says what to do next.
Re-running it is safe; pass --force to download everything again.

    python install_openzen.py --lpms-control

also fetches LP-Research's LPMS-Control for the LPMS-B2 (it ships inside
their OpenMAT 1.3.5 package) and starts its installer -- the vendor's own
tool, for checking the sensor without this platform in the way.
"""
from __future__ import annotations

import argparse
import io
import os
import shutil
import subprocess
import sys
import urllib.request
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
# Per user, not per project folder: every copy of the project on this PC (and
# the agent, whichever copy it is started from) then finds the same install.
HOME = (Path(os.environ["LOCALAPPDATA"]) / "SONAIR" / "openzen"
        if os.environ.get("LOCALAPPDATA") else HERE / "vendor" / "openzen")
PY_URL = "https://www.python.org/ftp/python/3.11.9/python-3.11.9-embed-amd64.zip"
OZ_URL = ("https://bitbucket.org/lpresearch/openzen/downloads/"
          "OpenZen-Windows-x64-Python-3.11.zip")
# LPMS-Control for the LPMS2 series (B2 included) is part of OpenMAT; this is
# the last build on LP-Research's download page.
OPENMAT_URL = ("https://bitbucket.org/lpresearch/openmat/downloads/"
               "OpenMAT-1.3.5-Setup-Build20180418.exe")


def fetch(url: str, what: str) -> bytes:
    print(f"  downloading {what} ...", flush=True)
    req = urllib.request.Request(url, headers={"User-Agent": "sonair-setup"})
    with urllib.request.urlopen(req, timeout=120) as r:
        data = r.read()
    print(f"    {len(data) / 1e6:.1f} MB", flush=True)
    return data


def install_python(force: bool) -> Path:
    dest = HOME / "python"
    exe = dest / "python.exe"
    if exe.exists() and not force:
        print(f"  Python 3.11 already in {dest}")
        return exe
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    zipfile.ZipFile(io.BytesIO(fetch(PY_URL, "Python 3.11 (embeddable)"))
                    ).extractall(dest)
    return exe


def install_openzen(force: bool) -> Path:
    dest = HOME / "lib"
    if (dest / "openzen.pyd").exists() and not force:
        print(f"  OpenZen already in {dest}")
        return dest
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    zf = zipfile.ZipFile(io.BytesIO(fetch(OZ_URL, "OpenZen for Python 3.11")))
    for name in zf.namelist():
        base = name.rsplit("/", 1)[-1]
        # The module, the USB driver DLL it may load, and the vendor's own
        # example -- kept as the reference for the API.
        if base.lower().endswith((".pyd", ".dll", ".py")):
            (dest / base).write_bytes(zf.read(name))
    if not (dest / "openzen.pyd").exists():
        raise RuntimeError("the OpenZen download did not contain openzen.pyd")
    return dest


def fetch_lpms_control(force: bool) -> Path:
    dest = HOME / "OpenMAT-1.3.5-Setup.exe"
    if dest.exists() and not force:
        print(f"  LPMS-Control installer already in {dest}")
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(fetch(OPENMAT_URL, "LPMS-Control (OpenMAT 1.3.5)"))
    return dest


def verify(exe: Path, lib: Path) -> str:
    code = ("import sys, os; sys.path.insert(0, r'%s'); "
            "os.add_dll_directory(r'%s'); import openzen; "
            "e, c = openzen.make_client(); print('ok', e); c.close()"
            % (lib, lib))
    out = subprocess.run([str(exe), "-c", code], capture_output=True,
                         text=True, timeout=60)
    if out.returncode != 0 or "ok" not in out.stdout:
        msg = "OpenZen did not load:\n" + out.stdout + out.stderr
        if "DLL load failed" in msg:
            # openzen.pyd is C++ and needs MSVCP140.dll, which the embeddable
            # Python does not carry. Nearly every PC already has it.
            msg += ("\nInstall the Microsoft Visual C++ Redistributable (x64) "
                    "from https://aka.ms/vs/17/release/vc_redist.x64.exe and "
                    "run this again.")
        raise RuntimeError(msg)
    return out.stdout.strip()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--force", action="store_true",
                    help="download everything again")
    ap.add_argument("--lpms-control", action="store_true",
                    help="also download LP-Research's LPMS-Control and start "
                         "its installer")
    args = ap.parse_args(argv)
    if os.name != "nt":
        print("OpenZen's prebuilt module here is for Windows. On Linux, build "
              "OpenZen with -DZEN_PYTHON=ON and set SONAIR_OPENZEN_DIR to the "
              "folder holding openzen.so.")
        return 1
    print(f"Installing OpenZen into {HOME}")
    try:
        exe = install_python(args.force)
        lib = install_openzen(args.force)
        print("  checking that OpenZen loads ...", flush=True)
        verify(exe, lib)
    except Exception as e:      # noqa: BLE001
        print(f"\nFAILED: {e}\n\nIf the download was blocked, fetch the two "
              f"files by hand and unzip them:\n  {PY_URL}\n    -> {HOME / 'python'}"
              f"\n  {OZ_URL}\n    -> {HOME / 'lib'}\nthen run this again.")
        return 1
    if args.lpms_control:
        try:
            setup = fetch_lpms_control(args.force)
            print(f"  starting the LPMS-Control installer: {setup}")
            os.startfile(str(setup))        # noqa: S606 -- the vendor's installer
        except Exception as e:      # noqa: BLE001
            print(f"  LPMS-Control could not be fetched ({e}). Download it by "
                  f"hand: {OPENMAT_URL}")
    print("\nDone. OpenZen loads.\n\nNext:\n"
          "  1. Pair the LPMS-B2 in Windows Settings > Bluetooth (once).\n"
          "  2. Close FusionHub and LPMS-Control -- only one program can hold\n"
          "     the sensor at a time.\n"
          "  3. Start the agent, open the Sensors page, choose\n"
          "     'LPMS sensor directly (OpenZen)', press 'Find it for me',\n"
          "     then Connect.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

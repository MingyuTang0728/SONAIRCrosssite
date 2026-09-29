"""
The console must load in a real browser without a single script error.

Syntax checks and the static wiring audit both passed on a console whose
campaign section called a helper defined in another closure. The exception
it threw at load stopped every handler registered after it -- Suggest did
nothing and Save the carrier said "Saving..." forever, with the agent
answering both within milliseconds. Only running the page shows that.

Run:  python tests/test_console_loads.py   (needs playwright + chromium)
"""
import os
import sys
from pathlib import Path

PAGE = Path(__file__).resolve().parent.parent / "SONAIR_Console.html"


def main():
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("  skipped: playwright is not installed")
        return
    exe = os.environ.get("CHROMIUM", "/opt/pw-browsers/chromium")
    errors = []
    with sync_playwright() as pw:
        kw = {"executable_path": exe} if Path(exe).exists() else {}
        b = pw.chromium.launch(**kw)
        pg = b.new_page()
        pg.on("pageerror", lambda e: errors.append(str(e)))
        pg.goto(PAGE.as_uri())
        pg.wait_for_timeout(1500)
        # every page opened once, so page-entry code runs too
        for p in pg.eval_on_selector_all("nav.rail .step[data-page]",
                                         "e => e.map(x => x.dataset.page)"):
            pg.click(f"nav.rail .step[data-page='{p}']")
            pg.wait_for_timeout(200)
        # the hold buttons are bound (the helper exists where it is used)
        bound = pg.evaluate("() => !!(window.SONAIR && window.SONAIR.holdButton)")
        b.close()
    assert not errors, "script errors on load:\n  " + "\n  ".join(errors)
    assert bound, "the hold-button helper was never shared"
    print("  pass  the console loads and every page opens with no script error")


if __name__ == "__main__":
    main()

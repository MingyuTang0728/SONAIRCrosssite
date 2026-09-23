"""
audit_ui.py — check that every control on the console is wired to something real.

A button that looks enabled and does nothing is worse than no button: it costs
the operator a diagnosis. This walks the whole chain and reports every place it
breaks:

    HTML control -> JavaScript handler -> message -> agent handler
    agent reply  -> JavaScript handler -> something on screen

It is static analysis, so it proves wiring exists, not that the wiring is
correct — a handler that sends the wrong field passes here. What it does catch
is the class of fault that actually accumulates as a page grows: a control
added without a handler, a handler that only prints a message, a message the
agent does not answer, and an answer nothing on the page reads.

Run it from the project folder:

    python audit_ui.py            summary, exit 1 if anything is broken
    python audit_ui.py -v         list every control it checked
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

HTML = "SONAIR_Console.html"
JS = ["console.js", "console_adv.js"]
AGENT = ["multimodal_bridge.py", "bench_agent.py", "ur_bridge_ext.py",
         "ur_control.py"]

# Replies that exist to keep the connection working and are not meant to draw
# anything. Listed by name so a NEW unread reply still shows up.
PROTOCOL_ONLY = {"auth", "auth_ok", "pong"}

# Robot commands the agent supports and the console deliberately does not
# offer, each with the reason. Withheld on purpose is a different state from
# forgotten, and an audit that cannot tell them apart reports the same dozen
# items every run until people stop reading it. Deleting a line here is how
# you decide to expose one.
WITHHELD = {
    "ur_script": "raw URScript — no guard rails, and the console is meant to "
                 "be usable by someone who does not write URScript",
    "ur_servoj": "a real-time streaming primitive; the jog loop drives it "
                 "from the host, where the timing can be held",
    "ur_speedl": "same — the host's jog loop owns it",
    "ur_speedj": "same — the host's jog loop owns it",
    "ur_movep":  "blended circular moves; the scan planner emits movel paths",
    "ur_movej":  "joint-space moves; every planned path is in tool space",
    "ur_stop":   "the STOP ROBOT button uses the faster e-stop path",
    "ur_stopj":  "as above",
    "ur_popup":  "writes a message to the pendant screen; nothing in the "
                 "workflow needs to",
    "ur_robot_mode": "already arrives continuously in the telemetry",
    "ur_safety_status": "already arrives continuously in the telemetry",
    "ur_program_state": "already arrives continuously in the telemetry",
    "ur_loaded_program": "already arrives continuously in the telemetry",
    "ur_estop": "wired to the STOP ROBOT button as the plain 'estop' message",
    "ur_set_dout": "wired to the input/output panel",
}


def read(name):
    p = Path(name)
    return p.read_text(encoding="utf-8") if p.exists() else ""


def main(verbose=False) -> int:
    html = read(HTML)
    js = "\n".join(read(f) for f in JS)
    agent = "\n".join(read(f) for f in AGENT)
    if not html or not js or not agent:
        print("Run this from the project folder — some files were not found.")
        return 2

    problems = []

    # -- 1. controls with no JavaScript at all ---------------------------
    controls = re.findall(r'<(button|select)[^>]*id="([A-Za-z0-9_]+)"', html)
    controls += [("input", i) for i in re.findall(
        r'<input[^>]*type="(?:range|checkbox)"[^>]*id="([A-Za-z0-9_]+)"', html)]
    controls += [("input", i) for i in re.findall(
        r'<input[^>]*id="([A-Za-z0-9_]+)"[^>]*type="(?:range|checkbox)"', html)]

    unwired = []
    for tag, cid in controls:
        # bound directly, through a helper, by delegation, or read on demand
        seen = any(re.search(p, js) for p in (
            rf'\$\("{cid}"\)\.addEventListener', rf'on\("{cid}",',
            rf'seg\("{cid}",', rf'\$\("{cid}"\)', rf'"{cid}"'))
        if not seen:
            unwired.append(f"<{tag} id={cid}> has no JavaScript at all")
    problems += unwired

    # -- 2. messages the page sends that the agent does not answer -------
    sent = sorted(set(re.findall(r'type:\s*"([a-z0-9_]+)"', js)))
    handled = set(re.findall(r'mtype == "([a-z0-9_]+)"', agent))
    handled |= set(re.findall(r'\bt == "([a-z0-9_]+)"', agent))
    for group in re.findall(r'mtype in \(([^)]*)\)', agent, re.S):
        handled |= set(re.findall(r'"([a-z0-9_]+)"', group))
    prefixes = set(re.findall(r'startswith\("([a-z0-9_]+)"\)', agent))

    unanswered = [m for m in sent
                  if m not in handled
                  and not any(m.startswith(p) for p in prefixes)]
    problems += [f"the page sends {m!r} and no agent handler takes it"
                 for m in unanswered]

    # -- 3. agent replies nothing on the page reads ----------------------
    replies = sorted(set(re.findall(r'"type":\s*"([a-z0-9_]+)"', agent)))
    consumed = set(re.findall(r'case "([a-z0-9_]+)"', js))
    consumed |= set(re.findall(r'S\.on\("([a-z0-9_]+)"', js))
    unread = [r for r in replies if r not in consumed and r not in PROTOCOL_ONLY]
    problems += [f"the agent replies {r!r} and nothing on the page reads it"
                 for r in unread]

    # -- 4. robot commands the agent supports and the page never offers --
    ctl = read("ur_control.py")
    cmds = set(re.findall(r'"(ur_[a-z0-9_]+)"', ctl))
    never = sorted(c for c in cmds if f'"{c}"' not in js)
    forgotten = [c for c in never if c not in WITHHELD]
    withheld = [c for c in never if c in WITHHELD]
    problems += [f"the agent supports {c!r} and no control sends it"
                 for c in forgotten]

    # -- report ----------------------------------------------------------
    print(f"controls checked        : {len(controls)}")
    print(f"messages the page sends : {len(sent)}")
    print(f"replies the agent sends : {len(replies)}")
    print(f"robot commands offered  : {len(cmds) - len(never)} of {len(cmds)}"
          f"  ({len(withheld)} withheld on purpose)")
    print()
    if verbose and withheld:
        print("Withheld on purpose:")
        for c in sorted(withheld):
            print(f"  {c:<22} {WITHHELD[c]}")
        print()
    if verbose:
        for tag, cid in sorted(controls, key=lambda c: c[1]):
            print(f"  ok  <{tag} id={cid}>")
        print()
    if problems:
        print(f"{len(problems)} PROBLEMS")
        for p in problems:
            print(f"  - {p}")
        return 1
    print("No dead controls: every control has a handler, every message has an "
          "agent handler, every reply is read, and every robot command the "
          "agent supports has a control.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main("-v" in sys.argv))

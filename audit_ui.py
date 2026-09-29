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


# Names the browser provides. Anything called that is neither declared in our
# own files nor on this list is a typo or a function somebody deleted.
_BROWSER = {
    "parseInt", "parseFloat", "isNaN", "isFinite", "String", "Number",
    "Boolean", "Array", "Object", "Math", "JSON", "Date", "RegExp", "Error",
    "Promise", "Map", "Set", "WeakMap", "Symbol", "BigInt", "Proxy",
    "setTimeout", "setInterval", "clearTimeout", "clearInterval",
    "requestAnimationFrame", "cancelAnimationFrame", "queueMicrotask",
    "fetch", "atob", "btoa", "alert", "confirm", "prompt", "encodeURIComponent",
    "decodeURIComponent", "encodeURI", "decodeURI", "escape", "unescape",
    "Blob", "File", "FileReader", "URL", "URLSearchParams", "FormData",
    "Image", "ImageBitmap", "createImageBitmap", "WebSocket", "Worker",
    "Uint8Array", "Uint8ClampedArray", "Int8Array", "Uint16Array",
    "Int16Array", "Uint32Array", "Int32Array", "Float32Array", "Float64Array",
    "ArrayBuffer", "DataView", "TextDecoder", "TextEncoder",
    "CustomEvent", "Event", "MutationObserver", "IntersectionObserver",
    "ResizeObserver", "structuredClone", "reportError", "getComputedStyle",
    "matchMedia", "open", "close", "print", "focus", "blur", "scrollTo",
    "THREE", "performance", "console", "document", "window", "navigator",
    "location", "history", "screen", "localStorage", "sessionStorage",
    "if", "for", "while", "switch", "catch", "return", "typeof", "function",
    "new", "delete", "void", "in", "of", "do", "else", "try", "finally",
    "case", "break", "continue", "throw", "var", "let", "const", "class",
    "extends", "super", "this", "null", "true", "false", "undefined",
}


def _undefined_css_vars(html: str) -> list:
    """Custom properties a stylesheet reads but never sets."""
    css = "\n".join(re.findall(r"<style>(.*?)</style>", html, re.S))
    if not css:
        return []
    defined = set(re.findall(r"(--[a-z0-9-]+)\s*:", css))
    out = []
    for name in sorted(set(re.findall(r"var\((--[a-z0-9-]+)\s*\)", css))):
        if name in defined:
            continue
        n = len(re.findall(re.escape(f"var({name}"), css))
        out.append(f"the stylesheet reads {name} {n} time"
                   + ("s" if n != 1 else "")
                   + " and never defines it, so those declarations are "
                     "dropped and the elements render unstyled")
    return out


def _unreachable_handlers(agent: str) -> list:
    """
    Every `mtype == "x"` branch must sit in a function some route sends "x" to.

    Finds the router's `startswith(...) -> _handle_y(data)` pairs, then walks
    each handler function and checks the message names it answers against the
    prefixes that reach it.
    """
    # Which prefixes route to which handler function.
    routes: dict[str, set] = {}
    pat = re.compile(
        r'startswith\(\s*(\(?[^)]*?\)?)\s*\)\s*:\s*\n\s*reply\s*=\s*'
        r'await\s+asyncio\.to_thread\(\s*([A-Za-z_][A-Za-z0-9_]*)')
    for raw, fn in pat.findall(agent):
        routes.setdefault(fn, set()).update(re.findall(r'"([a-z0-9_]+)"', raw))
    if not routes:
        return []

    # Where each handler function starts and ends.
    bounds = {}
    for fn in routes:
        m = re.search(rf'^(?:async )?def {re.escape(fn)}\(', agent, re.M)
        if not m:
            continue
        # `async def` counts too -- missing it swallowed every function after
        # the one being measured and reported half the agent as unreachable.
        nxt = re.search(r'^(?:async def |def |class |@)', agent[m.end():], re.M)
        bounds[fn] = (m.start(),
                      m.end() + (nxt.start() if nxt else len(agent) - m.end()))

    out = []
    for fn, (a, b) in bounds.items():
        body = agent[a:b]
        for msg in sorted(set(re.findall(r'mtype == "([a-z0-9_]+)"', body))):
            if not any(msg.startswith(p) for p in routes[fn]):
                out.append(
                    f"{fn}() answers {msg!r}, but the router only sends it "
                    + " or ".join(repr(p) + "*" for p in sorted(routes[fn]))
                    + f" — {msg!r} never gets there, so the page waits forever")
    return out


def check_js_calls(js_files) -> list:
    """
    Every function called must exist.

    Neither `node --check` nor the wiring audit above catches a call to a name
    that was never declared: the file parses, every control has a handler by
    name, and the failure only appears when a person presses that particular
    button. One such call -- a helper whose declaration a bad edit had dropped
    -- sat in the preflight path and made the gate report the wrong thing.
    """
    import re as _re

    def strip_code(text: str) -> str:
        """
        Comments and string literals blanked, newlines kept.

        Without this the scan reads prose and CSS out of string literals --
        "rgba(" in a colour, "(see the pendant)" in an operator message -- and
        reports them as calls to undeclared functions. Blanking rather than
        deleting keeps every line number correct.
        """
        out = []
        i, n = 0, len(text)
        while i < n:
            c = text[i]
            if c == "/" and i + 1 < n and text[i + 1] == "/":
                j = text.find("\n", i)
                j = n if j < 0 else j
                out.append(" " * (j - i)); i = j
            elif c == "/" and i + 1 < n and text[i + 1] == "*":
                j = text.find("*/", i + 2)
                j = n if j < 0 else j + 2
                out.append("".join(ch if ch == "\n" else " " for ch in text[i:j]))
                i = j
            elif c in "\"'`":
                q, j = c, i + 1
                while j < n:
                    if text[j] == "\\":
                        j += 2; continue
                    if text[j] == q:
                        j += 1; break
                    j += 1
                out.append("".join(ch if ch == "\n" else " " for ch in text[i:j]))
                i = j
            else:
                out.append(c); i += 1
        return "".join(out)

    src = {}
    for f in js_files:
        src[f] = strip_code(read(f))
    joined = "\n".join(src.values())

    declared = set()
    declared |= set(_re.findall(r"\bfunction\s+([A-Za-z_$][\w$]*)", joined))
    declared |= set(_re.findall(r"\b(?:var|let|const)\s+([A-Za-z_$][\w$]*)", joined))
    # destructured and multi-declarator forms, and function parameters
    declared |= set(_re.findall(r"\bfunction\s*\(([^)]*)\)", joined)
                    and [] or [])
    for params in _re.findall(r"function[^(]*\(([^)]*)\)", joined):
        for nm in _re.findall(r"[A-Za-z_$][\w$]*", params):
            declared.add(nm)
    for params in _re.findall(r"\(([^()]*)\)\s*=>", joined):
        for nm in _re.findall(r"[A-Za-z_$][\w$]*", params):
            declared.add(nm)
    declared |= set(_re.findall(r"\bcatch\s*\(\s*([A-Za-z_$][\w$]*)", joined))
    declared |= set(_re.findall(r"([A-Za-z_$][\w$]*)\s*:\s*function", joined))
    declared |= set(_re.findall(r"([A-Za-z_$][\w$]*)\s*=\s*function", joined))

    problems = []
    for f, text in src.items():
        # a bare `name(` not preceded by a dot, and not a declaration site
        for m in _re.finditer(r"(?<![.\w$])([A-Za-z_$][\w$]*)\s*\(", text):
            name = m.group(1)
            if name in declared or name in _BROWSER:
                continue
            line = text.count("\n", 0, m.start()) + 1
            problems.append(f"{f}:{line} calls {name}() which is never declared")
    return sorted(set(problems))


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
    for group in re.findall(r'startswith\(\(([^)]*)\)\)', agent):
        prefixes |= set(re.findall(r'"([a-z0-9_]+)"', group))

    unanswered = [m for m in sent
                  if m not in handled
                  and not any(m.startswith(p) for p in prefixes)]
    problems += [f"the page sends {m!r} and no agent handler takes it"
                 for m in unanswered]

    # -- 1b. styling that resolves to nothing ----------------------------
    #
    # `background: var(--panel)` with no `--panel` defined is not an error
    # anywhere: the declaration is dropped and the element renders with no
    # background at all. The jog dock and every reading card on the sensors
    # page were doing exactly that -- floating as bare text over whatever was
    # underneath, with borders drawn around transparency. It is invisible in
    # the markup and obvious on screen, which is the worst combination, so it
    # is checked here rather than found by looking.
    problems += _undefined_css_vars(html)

    # -- 2b. handlers the router cannot actually reach -------------------
    #
    # The check above asks "does a handler for this message exist anywhere in
    # the agent", and that is not the same question as "will this message get
    # there". The agent is a PREFIX ROUTER: a message is handed to one handler
    # function chosen by the start of its name, so a branch sitting in the
    # wrong function is dead code no matter how correct it is.
    #
    # It passed this audit and shipped: `carrier_set` was answered inside
    # `_handle_automation`, which only ever sees messages beginning `auto_`.
    # From the operator's side an unreachable handler and a missing one are the
    # same thing -- a button that spins forever -- so it is worth its own check.
    problems += _unreachable_handlers(agent)

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

    # -- 5. functions that are called and do not exist -------------------
    problems += check_js_calls(JS)

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

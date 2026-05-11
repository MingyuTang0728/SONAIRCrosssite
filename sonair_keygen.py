"""
sonair_keygen.py — API Key Issuance & Management for SONAIR
=============================================================

This script generates structured API keys for the SONAIR cross-site teleop
system. Each key carries metadata (institution, role, scope, expiry) so the
relay can enforce fine-grained permissions and the audit log can attribute
every action to a specific institution/person.

Key types produced:
  - AGENT key   — for the UR Host Agent (one per physical robot cell).
                   Highest privilege — anyone holding this can directly
                   command the robot. Treat like a private TLS key.

  - HOST key    — for the on-site operator's browser (e.g. UoN).
                   Has authority arbitration privileges (can grant/revoke).

  - GUEST key   — for remote operators (UCL, ORE Catapult, etc.).
                   Must request authority from a host before any motion.
                   Subject to envelope + watchdog checks.

Usage:
  # Generate a fresh key set on first install
  python sonair_keygen.py issue --institution UoN  --role agent
  python sonair_keygen.py issue --institution UoN  --role host
  python sonair_keygen.py issue --institution UCL  --role guest --expires 90

  # List all currently-issued keys
  python sonair_keygen.py list

  # Revoke a compromised key
  python sonair_keygen.py revoke --id <key-id>

The keys live in `sonair_keys.json` (gitignored). The hashed form is what
the relay actually checks against — the plaintext is shown to the user
ONCE at issuance time and never stored.

Author: Mingyu Tang, University of Nottingham
"""
import argparse
import hashlib
import json
import os
import secrets
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

KEYS_FILE = Path(os.environ.get("SONAIR_KEYS_FILE", "sonair_keys.json"))


def load_db():
    if not KEYS_FILE.exists():
        return {"version": 1, "keys": []}
    with open(KEYS_FILE, "r", encoding="utf-8") as f:
        return json.load(f)


def save_db(db):
    # Atomic write — never leave a half-written keys file
    tmp = KEYS_FILE.with_suffix(".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(db, f, indent=2)
    tmp.replace(KEYS_FILE)
    # Tighten permissions: owner read/write only (Unix). On Windows this is
    # a no-op but ACLs are still honoured.
    try:
        os.chmod(KEYS_FILE, 0o600)
    except Exception:
        pass


def hash_key(plaintext):
    """We never store the plaintext key. Only the SHA-256 digest is kept,
    so a leaked keys file can't be used to authenticate."""
    return hashlib.sha256(plaintext.encode("utf-8")).hexdigest()


def cmd_issue(args):
    if args.role not in ("agent", "host", "guest"):
        print("ERROR: role must be one of: agent, host, guest")
        sys.exit(1)

    # Compose human-readable key id like:  UoN-host-2026-01-15-a3f9
    today = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    suffix = secrets.token_hex(2)
    key_id = f"{args.institution}-{args.role}-{today}-{suffix}"

    # Plaintext part — what the operator actually pastes into the UI.
    # Format: sonair_<role>_<random>  — easy to recognise in logs/leaks
    plaintext = f"sonair_{args.role}_{secrets.token_urlsafe(32)}"

    expires_at = None
    if args.expires:
        expires_at = (datetime.now(timezone.utc)
                      + timedelta(days=args.expires)).isoformat()

    record = {
        "id":           key_id,
        "institution":  args.institution,
        "role":         args.role,
        "label":        args.label or f"{args.institution} {args.role}",
        "hash":         hash_key(plaintext),
        "issued_at":    datetime.now(timezone.utc).isoformat(),
        "expires_at":   expires_at,
        "revoked":      False,
        "rooms":        args.rooms.split(",") if args.rooms else ["uon-cell-1"],
    }

    db = load_db()
    db["keys"].append(record)
    save_db(db)

    print()
    print("=" * 72)
    print(f"  Key issued:  {key_id}")
    print(f"  Institution: {args.institution}")
    print(f"  Role:        {args.role}")
    print(f"  Rooms:       {', '.join(record['rooms'])}")
    print(f"  Expires:     {expires_at or 'never'}")
    print("=" * 72)
    print()
    print("  PLAINTEXT KEY (copy this NOW — it cannot be retrieved later):")
    print()
    print(f"    {plaintext}")
    print()
    print("  Distribute via secure channel only (1Password share, encrypted")
    print("  email, in-person). Never commit to git.")
    print("=" * 72)


def cmd_list(args):
    db = load_db()
    if not db["keys"]:
        print("No keys issued yet. Run: python sonair_keygen.py issue ...")
        return
    print(f"{'ID':<36} {'INST':<6} {'ROLE':<6} {'STATE':<10} {'EXPIRES':<25}")
    print("-" * 90)
    for k in db["keys"]:
        state = "REVOKED" if k["revoked"] else "active"
        if not k["revoked"] and k.get("expires_at"):
            if datetime.fromisoformat(k["expires_at"]) < datetime.now(timezone.utc):
                state = "expired"
        exp = k.get("expires_at") or "never"
        print(f"{k['id']:<36} {k['institution']:<6} {k['role']:<6} "
              f"{state:<10} {exp:<25}")


def cmd_revoke(args):
    db = load_db()
    found = False
    for k in db["keys"]:
        if k["id"] == args.id:
            k["revoked"] = True
            k["revoked_at"] = datetime.now(timezone.utc).isoformat()
            found = True
            print(f"Revoked: {args.id}")
            break
    if not found:
        print(f"No key with id={args.id}")
        sys.exit(1)
    save_db(db)


def cmd_export_for_relay(args):
    """Emit the env-var setup that the relay needs."""
    db = load_db()
    print("# ----- SONAIR Relay environment exports -----")
    print("# Source this file or paste into your shell before starting relay")
    print()
    for k in db["keys"]:
        if k["revoked"]:
            continue
        if k.get("expires_at"):
            if datetime.fromisoformat(k["expires_at"]) < datetime.now(timezone.utc):
                continue
        # The relay verifies hashes, not plaintext. So we export the hash.
        var = f"SONAIR_KEY_{k['role'].upper()}_{k['institution'].upper()}_HASH"
        print(f"export {var}={k['hash']}  # {k['label']}")
    print()
    print("# Per-role single-token shortcuts (for legacy single-tenant mode):")
    print("# Pick the ONE key per role you want active — relay checks any match")


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="SONAIR API key management",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Initial setup — issue one of each
  python sonair_keygen.py issue --institution UoN --role agent  --label "UoN Cell 1 Agent"
  python sonair_keygen.py issue --institution UoN --role host   --label "Mingyu (UoN ops)"
  python sonair_keygen.py issue --institution UCL --role guest  --label "UCL data team" --expires 90

  # Audit
  python sonair_keygen.py list

  # Revoke if compromised
  python sonair_keygen.py revoke --id UCL-guest-2026-01-15-a3f9

  # Get env vars for the relay
  python sonair_keygen.py export
""")
    sp = p.add_subparsers(dest="cmd", required=True)

    p_issue = sp.add_parser("issue", help="Issue a new API key")
    p_issue.add_argument("--institution", required=True,
                         help="e.g. UoN, UCL, ORE, LR")
    p_issue.add_argument("--role", required=True,
                         choices=["agent", "host", "guest"])
    p_issue.add_argument("--label", help="Human-readable description")
    p_issue.add_argument("--expires", type=int,
                         help="Days until expiry (omit for no expiry)")
    p_issue.add_argument("--rooms", default="uon-cell-1",
                         help="Comma-separated rooms this key is valid for")
    p_issue.set_defaults(func=cmd_issue)

    p_list = sp.add_parser("list", help="List all issued keys")
    p_list.set_defaults(func=cmd_list)

    p_revoke = sp.add_parser("revoke", help="Revoke a key by id")
    p_revoke.add_argument("--id", required=True)
    p_revoke.set_defaults(func=cmd_revoke)

    p_export = sp.add_parser("export", help="Print env vars for the relay")
    p_export.set_defaults(func=cmd_export_for_relay)

    args = p.parse_args()
    args.func(args)

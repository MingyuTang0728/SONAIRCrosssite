"""
relay_server.py — SONAIR Cross-Site Cloud Relay
=================================================

Sits in the cloud (or any always-on internet-reachable host) and brokers
WebSocket traffic between the UR Host Agent (at UoN, where the physical
robot lives) and one or more remote operators (UCL, partner labs, etc.).

Why a relay instead of UCL → UoN direct?
  - University firewalls block inbound public traffic. UoN's UR cell can
    open OUTBOUND wss connections (always allowed) but cannot accept
    INBOUND ones without IT involvement. The relay flips the topology
    so both sides initiate outbound — neither needs an open inbound port.
  - Identity & audit centralized. Every authority transition, every
    rejected motion command, every E-STOP gets logged in ONE place.
  - Multi-site future-proof. Adding ORE Catapult, Lloyd's Register, etc.
    later is just another wss client connection — no new infrastructure.

Trust model:
  - The relay is the SOLE authority arbiter. Browsers' claimed authority
    is never trusted.
  - The UR Host Agent is the only entity allowed to forward URScript to
    the physical UR. It has its own short-lived auth token, separate from
    operator tokens.
  - Operators (host or guest) connect with their own bearer tokens and
    are mapped to a "room" (the cell they want to operate). One robot
    cell = one room.

Reference architectures (for your SONAIR proposal):
  - WebRTC TURN servers (same outbound-only NAT-traversal pattern)
  - ROS2 Discovery Server (centralized rendezvous)
  - Foxglove Studio remote bridge
  - VICTOR (Voice and Internet for Coordinated Telerobotics) — NASA JSC

Author: Mingyu Tang, University of Nottingham
"""
import asyncio
import websockets
import json
import os
import time
import secrets
import hashlib
import logging
from datetime import datetime, timezone
from pathlib import Path
from collections import defaultdict


# ============================================================
# Configuration — all overridable via environment variables
# ============================================================
RELAY_HOST   = os.environ.get("RELAY_HOST", "0.0.0.0")
RELAY_PORT   = int(os.environ.get("RELAY_PORT", 8770))

# ----------------------------------------------------------------------
# API key validation — TWO modes are supported:
#
#  Mode A — Structured keys file (preferred for production):
#    Set SONAIR_KEYS_FILE to the path of a JSON file produced by
#    `sonair_keygen.py issue ...`. The relay loads it on startup, hashes
#    every incoming token and looks up its metadata (institution, role,
#    rooms, expiry, revocation). This gives you per-institution audit
#    attribution and key rotation without restarting.
#
#  Mode B — Single-token env vars (backwards-compatible, dev mode):
#    Set RELAY_AGENT_TOKEN / RELAY_HOST_TOKEN / RELAY_GUEST_TOKEN to
#    plain bearer strings. Anyone presenting that string with the
#    matching role passes auth. Simple but lacks per-user audit.
# ----------------------------------------------------------------------
KEYS_FILE = Path(os.environ.get("SONAIR_KEYS_FILE", "sonair_keys.json"))

# Fallback single-token mode
AGENT_TOKEN          = os.environ.get("RELAY_AGENT_TOKEN") or os.environ.get("SONAIR_AGENT_TOKEN") or os.environ.get("AGENT_TOKEN") or "agent-default-replace-me"
OPERATOR_TOKEN_HOST  = os.environ.get("RELAY_HOST_TOKEN") or os.environ.get("SONAIR_HOST_TOKEN") or os.environ.get("HOST_TOKEN") or "host-default-replace-me"
OPERATOR_TOKEN_GUEST = os.environ.get("RELAY_GUEST_TOKEN") or os.environ.get("SONAIR_GUEST_TOKEN") or os.environ.get("GUEST_TOKEN") or "guest-default-replace-me"

# Server-side workspace envelope (defense in depth, even though both
# the host browser and the agent also enforce it).
ENVELOPE = {
    "x_min": -0.6, "x_max": 0.6,
    "y_min": -0.6, "y_max": 0.6,
    "z_min":  0.05, "z_max": 0.7,
}

# Watchdog: if the operator currently holding authority misses heartbeats
# for this long, force E-STOP and revoke. 700ms accommodates ~250ms WAN
# RTT plus a few missed pings.
REMOTE_WATCHDOG_MS = 700

# Audit log root directory (JSONL, one file per day)
AUDIT_DIR = Path(os.environ.get("RELAY_AUDIT_DIR", "./relay_audit"))
AUDIT_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# Logging
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-5s [%(name)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("relay")


# ============================================================
# API Key validation — Mode A (structured keys file)
# ============================================================
_KEYS_DB_CACHE = {"mtime": 0, "by_hash": {}}


def _load_keys_db():
    """Load and index the keys file. Auto-reloads when the file changes,
    so you can issue/revoke keys without restarting the relay."""
    if not KEYS_FILE.exists():
        return None
    try:
        mtime = KEYS_FILE.stat().st_mtime
        if mtime == _KEYS_DB_CACHE["mtime"]:
            return _KEYS_DB_CACHE["by_hash"]
        with open(KEYS_FILE, "r", encoding="utf-8") as f:
            db = json.load(f)
        # Index by hash for O(1) lookup
        by_hash = {k["hash"]: k for k in db.get("keys", [])}
        _KEYS_DB_CACHE["by_hash"] = by_hash
        _KEYS_DB_CACHE["mtime"]   = mtime
        log.info("loaded %d API keys from %s", len(by_hash), KEYS_FILE)
        return by_hash
    except Exception as e:
        log.warning("keys file load failed: %s", e)
        return None



def _validate_env_token(plaintext, expected_role):
    expected_token = {
        "agent": AGENT_TOKEN,
        "host":  OPERATOR_TOKEN_HOST,
        "guest": OPERATOR_TOKEN_GUEST,
    }.get(expected_role)
    if expected_token and plaintext == expected_token and not expected_token.endswith("replace-me"):
        return True, {"id": "env-mode", "institution": "?", "role": expected_role}
    return False, "bad_token"


def validate_key(plaintext, expected_role, room_id):
    """Returns (ok, key_record_or_reason).

    Tries Mode A (structured keys file) first. If that file isn't present,
    falls back to Mode B (single env-var tokens).
    """
    # ---- Mode A: structured keys ----
    db = _load_keys_db()
    if db is not None:
        digest = hashlib.sha256(plaintext.encode("utf-8")).hexdigest()
        rec = db.get(digest)
        if rec is None:
            # If a keys file is present but does not contain this token,
            # still allow explicit environment tokens. This prevents a stale
            # sonair_keys.json from breaking a live demo when env tokens are set.
            ok_env, info_env = _validate_env_token(plaintext, expected_role)
            if ok_env:
                return ok_env, info_env
            return False, "unknown_key"
        if rec.get("revoked"):
            return False, "key_revoked"
        if rec.get("expires_at"):
            if datetime.fromisoformat(rec["expires_at"]) < datetime.now(timezone.utc):
                return False, "key_expired"
        if rec.get("role") != expected_role:
            return False, "role_mismatch"
        if rec.get("rooms") and room_id not in rec["rooms"]:
            return False, "room_not_permitted"
        return True, rec

    # ---- Mode B: env-var fallback ----
    return _validate_env_token(plaintext, expected_role)


# ============================================================
# Latency-adaptive velocity cap — applied to remote-origin commands
# ============================================================
def latency_to_vcap(rtt_ms):
    if rtt_ms <= 50:    return 1.0
    if rtt_ms <= 150:   return 1.0 - 0.5 * (rtt_ms - 50) / 100.0
    if rtt_ms <= 300:   return 0.25
    return 0.0


# ============================================================
# Audit
# ============================================================
def audit(event_kind, session, payload=None):
    rec = {
        "ts":      datetime.now(timezone.utc).isoformat(),
        "kind":    event_kind,
        "role":    session.role if session else None,
        "site":    session.site if session else None,
        "session": session.id if session else None,
        "room":    session.room if session else None,
        "payload": payload or {},
    }
    fname = AUDIT_DIR / f"audit_{datetime.utcnow():%Y%m%d}.jsonl"
    try:
        with open(fname, "a", encoding="utf-8") as fp:
            fp.write(json.dumps(rec, default=str) + "\n")
    except Exception as e:
        log.warning("audit write failed: %s", e)


# ============================================================
# Session — one per WebSocket connection
# ============================================================
class Session:
    __slots__ = ("id", "websocket", "role", "site", "room",
                 "last_seen_ms", "last_rtt_ms", "peer_ip", "peer_port",
                 "user_agent", "client_net")

    def __init__(self, websocket, role, site, room, peer_ip=None, peer_port=None, user_agent=None, client_net=None):
        self.id            = secrets.token_hex(8)
        self.websocket     = websocket
        self.role          = role          # 'agent' | 'host' | 'guest'
        self.site          = site          # 'UoN', 'UCL', etc.
        self.room          = room          # which cell/robot this session is bound to
        self.last_seen_ms  = int(time.time() * 1000)
        self.last_rtt_ms   = 0
        self.peer_ip       = peer_ip
        self.peer_port     = peer_port
        self.user_agent    = user_agent
        self.client_net    = client_net or {}


# ============================================================
# Room — one robot cell, with one agent + N operators
# ============================================================
class Room:
    """
    A room represents one physical UR cell. It has:
      - exactly 0 or 1 agent (the UR Host Agent — the bridge to the robot)
      - exactly 0 or 1 host operator (the on-site UoN browser)
      - 0..N guest operators (UCL, partner labs)

    Authority states (server-authoritative):
      HOST_OPERATOR    — host's browser drives (default)
      REMOTE_OPERATOR  — a specific guest drives, with explicit grant
      LOCKED           — E-STOP is active; nobody drives until host resets
    """
    HOST_OPERATOR    = "host_operator"
    REMOTE_OPERATOR  = "remote_operator"
    LOCKED           = "locked"

    def __init__(self, room_id):
        self.id                 = room_id
        self.agent              = None     # Session of the agent
        self.host_operator      = None     # Session of the host browser
        self.guests             = set()    # Sessions of guest browsers
        self.state              = self.HOST_OPERATOR
        self.remote_session     = None     # the guest holding REMOTE_OPERATOR
        self.pending_request    = None     # guest session awaiting host's grant
        self.lock               = asyncio.Lock()

    def all_sessions(self):
        out = []
        if self.agent:         out.append(self.agent)
        if self.host_operator: out.append(self.host_operator)
        out.extend(self.guests)
        return out

    def operators(self):
        out = []
        if self.host_operator: out.append(self.host_operator)
        out.extend(self.guests)
        return out

    def is_writer(self, session):
        """Server-authoritative check — does this session have permission
        to issue motion commands right now?"""
        if self.state == self.LOCKED:
            return False
        if session.role == "host":
            return self.state == self.HOST_OPERATOR
        if session.role == "guest":
            return (self.state == self.REMOTE_OPERATOR
                    and self.remote_session is session)
        # agents never originate motion — they only execute relayed commands
        return False


ROOMS = {}              # room_id -> Room
ROOMS_LOCK = asyncio.Lock()


async def get_or_create_room(room_id):
    async with ROOMS_LOCK:
        if room_id not in ROOMS:
            ROOMS[room_id] = Room(room_id)
            log.info("room created: %s", room_id)
        return ROOMS[room_id]


# ============================================================
# Envelope check
# ============================================================
def envelope_accepts(pose):
    if not pose or len(pose) < 3:
        return True
    x, y, z = pose[0], pose[1], pose[2]
    return (ENVELOPE["x_min"] <= x <= ENVELOPE["x_max"]
            and ENVELOPE["y_min"] <= y <= ENVELOPE["y_max"]
            and ENVELOPE["z_min"] <= z <= ENVELOPE["z_max"])


# ============================================================
# Sending helpers
# ============================================================
async def safe_send(session, message):
    try:
        await session.websocket.send(json.dumps(message))
        return True
    except Exception:
        return False


async def broadcast_room(room, message, exclude=None):
    for s in room.all_sessions():
        if s is exclude:
            continue
        await safe_send(s, message)



async def session_public_info(session):
    return {
        "type": "peer_info",
        "session": session.id,
        "role": session.role,
        "site": session.site,
        "peer_ip": session.peer_ip,
        "peer_port": session.peer_port,
        "relay_rtt_ms": session.last_rtt_ms,
        "client_net": session.client_net,
    }


async def relay_ping_loop(session):
    """Server-originated RTT measurement using relay timestamps."""
    try:
        while True:
            await asyncio.sleep(0.5)
            await safe_send(session, {
                "type": "ping",
                "ts": int(time.time() * 1000),
                "source": "relay",
            })
    except asyncio.CancelledError:
        pass


# ============================================================
# Watchdog
# ============================================================
async def watchdog_loop():
    """Force E-STOP if the active remote operator goes silent."""
    while True:
        await asyncio.sleep(0.1)
        async with ROOMS_LOCK:
            rooms = list(ROOMS.values())
        for room in rooms:
            rs = room.remote_session
            if rs is None or room.state != Room.REMOTE_OPERATOR:
                continue
            silent_ms = int(time.time() * 1000) - rs.last_seen_ms
            if silent_ms > REMOTE_WATCHDOG_MS:
                log.warning("WATCHDOG[%s]: remote %s silent %dms — E-STOP",
                            room.id, rs.id, silent_ms)
                async with room.lock:
                    room.state = Room.LOCKED
                    room.remote_session = None
                    room.pending_request = None
                # Tell the agent to E-STOP the physical robot
                if room.agent:
                    await safe_send(room.agent, {"type": "estop"})
                audit("watchdog_estop", rs, {"silent_ms": silent_ms})
                await broadcast_room(room, {
                    "type":  "authority_changed",
                    "state": "locked",
                    "by":    "watchdog",
                })


# ============================================================
# Connection handler
# ============================================================
async def handle_connection(websocket):
    """One per WebSocket. The first message MUST be {"type":"auth", ...}."""
    session = None
    room    = None
    ping_task = None

    # ---- AUTH HANDSHAKE ----
    try:
        first = await asyncio.wait_for(websocket.recv(), timeout=5.0)
        data  = json.loads(first)

        if data.get("type") != "auth":
            await websocket.send(json.dumps({
                "type": "error", "reason": "auth_required"}))
            return

        token   = data.get("token", "")
        role    = data.get("role", "")
        site    = data.get("site", "?")
        room_id = data.get("room", "default")
        client_net = data.get("client_net", {}) if isinstance(data.get("client_net", {}), dict) else {}

        if role not in ("agent", "host", "guest"):
            await websocket.send(json.dumps({
                "type": "error", "reason": "unknown_role"}))
            return

        ok, info = validate_key(token, role, room_id)
        if not ok:
            log.warning("auth fail role=%s site=%s room=%s reason=%s",
                        role, site, room_id, info)
            await websocket.send(json.dumps({
                "type": "error", "reason": info}))
            return

        # Use the institution from the key metadata if Mode A was used,
        # otherwise fall back to whatever the client claimed in `site`
        if isinstance(info, dict) and info.get("institution") not in (None, "?"):
            site = info["institution"]
            key_id = info.get("id", "?")
        else:
            key_id = "env-mode"
        log.info("auth OK role=%s key=%s site=%s room=%s",
                 role, key_id, site, room_id)

        peer = getattr(websocket, "remote_address", None)
        peer_ip, peer_port = (peer[0], peer[1]) if isinstance(peer, tuple) and len(peer) >= 2 else (None, None)
        user_agent = None
        try:
            user_agent = websocket.request_headers.get("User-Agent")
        except Exception:
            user_agent = None
        session = Session(websocket, role, site, room_id, peer_ip=peer_ip, peer_port=peer_port,
                          user_agent=user_agent, client_net=client_net)
        room    = await get_or_create_room(room_id)

        # Bind session into the room
        async with room.lock:
            if role == "agent":
                if room.agent is not None:
                    # Don't allow two agents — second connection is a sign of
                    # split-brain and would be unsafe.
                    await websocket.send(json.dumps({
                        "type": "error", "reason": "agent_already_bound"}))
                    return
                room.agent = session
            elif role == "host":
                if room.host_operator is not None:
                    # Replace the existing host with this fresh one
                    try: await room.host_operator.websocket.close()
                    except Exception: pass
                room.host_operator = session
            elif role == "guest":
                room.guests.add(session)

        # Acknowledge auth
        await websocket.send(json.dumps({
            "type":            "auth_ok",
            "session":         session.id,
            "role":            role,
            "site":            site,
            "room":            room_id,
            "envelope":        ENVELOPE,
            "authority_state": room.state,
            "agent_online":    room.agent is not None,
            "peer_ip":         session.peer_ip,
            "peer_port":       session.peer_port,
            "relay_rtt_ms":    session.last_rtt_ms,
        }))
        audit("session_open", session)
        log.info("session OPEN: id=%s role=%s site=%s room=%s peer=%s:%s",
                 session.id, role, site, room_id, session.peer_ip, session.peer_port)
        ping_task = asyncio.create_task(relay_ping_loop(session))
        await safe_send(session, await session_public_info(session))

        # Notify peers in same room
        await broadcast_room(room, {
            "type":    "peer_hello",
            "name":    f"{site}/{role}",
            "session": session.id,
        }, exclude=session)

    except (asyncio.TimeoutError, json.JSONDecodeError) as e:
        log.warning("auth handshake failed: %s", e)
        return
    except Exception as e:
        log.exception("unexpected error during auth: %s", e)
        return

    # ============================================================
    # Main message loop
    # ============================================================
    try:
        async for raw in websocket:
            try:
                data = json.loads(raw)
            except Exception:
                continue
            mtype = data.get("type")
            session.last_seen_ms = int(time.time() * 1000)

            # ---- Liveness ----
            if mtype == "ping":
                await safe_send(session, {"type": "pong", "ts": data.get("ts"), "seq": data.get("seq"), "source": data.get("source", "client"), "relay_rtt_ms": session.last_rtt_ms})
                continue
            if mtype == "pong":
                ts = data.get("ts")
                if ts:
                    session.last_rtt_ms = max(0, int(time.time() * 1000) - int(ts))
                continue

            # ====================================================
            # AGENT messages — coming from the UR Host Agent at UoN
            # The agent forwards UR state and camera frames upstream;
            # we relay them to all operators in the room.
            # ====================================================
            if session.role == "agent":
                if mtype in ("state", "tcp_pose", "camera_frame", "urp_list",
                             "dashboard_res", "agent_status"):
                    for op in room.operators():
                        await safe_send(op, data)
                    continue
                # Agent shouldn't normally send anything else
                continue

            # ====================================================
            # OPERATOR messages — host or guest browser
            # ====================================================

            # ---- Authority handshake ----
            if mtype == "auth_request" and session.role == "guest":
                async with room.lock:
                    if room.state == Room.LOCKED:
                        await safe_send(session, {"type": "auth_denied",
                                                  "reason": "estop_active"})
                        continue
                    if room.state == Room.REMOTE_OPERATOR:
                        await safe_send(session, {"type": "auth_denied",
                                                  "reason": "another_remote_active"})
                        continue
                    room.pending_request = session
                audit("auth_request", session)
                # Notify host
                if room.host_operator:
                    await safe_send(room.host_operator, {
                        "type":    "auth_request_pending",
                        "from":    f"{session.site}/{session.role}",
                        "session": session.id,
                    })
                continue

            if mtype == "auth_grant" and session.role == "host":
                async with room.lock:
                    if room.pending_request is None:
                        await safe_send(session, {"type": "auth_denied",
                                                  "reason": "no_pending"})
                        continue
                    granted = room.pending_request
                    room.state          = Room.REMOTE_OPERATOR
                    room.remote_session = granted
                    room.pending_request = None
                audit("auth_grant", session, {"granted_to": granted.id})
                # Tell the granted guest
                await safe_send(granted, {"type": "auth_grant"})
                # Broadcast new state to whole room
                await broadcast_room(room, {
                    "type":  "authority_changed",
                    "state": room.state,
                    "by":    "host_grant",
                })
                continue

            if mtype == "auth_release":
                async with room.lock:
                    room.state          = Room.HOST_OPERATOR
                    room.remote_session = None
                    room.pending_request = None
                audit("auth_release", session)
                # Safety: also stop the robot when authority changes hands
                if room.agent:
                    await safe_send(room.agent, {
                        "type": "dashboard", "cmd": "stop"})
                await broadcast_room(room, {
                    "type":  "authority_changed",
                    "state": room.state,
                    "by":    f"{session.role}_release",
                })
                continue

            # ---- E-STOP — anyone can fire ----
            if mtype == "estop":
                async with room.lock:
                    room.state = Room.LOCKED
                    room.remote_session = None
                    room.pending_request = None
                audit("estop", session)
                if room.agent:
                    await safe_send(room.agent, {"type": "estop"})
                await broadcast_room(room, {
                    "type":  "authority_changed",
                    "state": "locked",
                    "by":    f"estop_by_{session.role}",
                })
                continue

            if mtype == "estop_reset" and session.role == "host":
                async with room.lock:
                    if room.state == Room.LOCKED:
                        room.state = Room.HOST_OPERATOR
                audit("estop_reset", session)
                await broadcast_room(room, {
                    "type":  "authority_changed",
                    "state": room.state,
                    "by":    "host_reset",
                })
                continue

            # ---- Motion commands (jog/movel/run_script) ----
            # Server-authoritative gate — no claim from the client matters.
            if mtype in ("jog", "movel", "run_script", "speedl", "speedl_stop", "speedj", "speedj_stop", "freedrive_start", "freedrive_stop"):
                if not room.is_writer(session):
                    audit("motion_denied", session,
                          {"mtype": mtype, "reason": "no_authority"})
                    await safe_send(session, {
                        "type":   "cmd_rejected",
                        "seq":    data.get("seq"),
                        "reason": "no_authority",
                    })
                    continue

                # Guests get extra defense-in-depth checks
                if session.role == "guest":
                    if mtype == "movel":
                        pose = data.get("pose")
                        if not envelope_accepts(pose):
                            audit("motion_denied", session, {
                                "mtype": "movel", "reason": "envelope",
                                "pose": pose,
                            })
                            await safe_send(session, {
                                "type":   "cmd_rejected",
                                "seq":    data.get("seq"),
                                "reason": "envelope_violation",
                            })
                            continue
                    vcap = latency_to_vcap(session.last_rtt_ms)
                    if vcap <= 0.01:
                        audit("motion_denied", session, {
                            "mtype": mtype, "reason": "vcap_zero",
                            "rtt_ms": session.last_rtt_ms,
                        })
                        await safe_send(session, {
                            "type":   "cmd_rejected",
                            "seq":    data.get("seq"),
                            "reason": "link_degraded",
                        })
                        continue
                    # Annotate with vcap so the agent applies it
                    data["_vcap"] = vcap
                else:
                    data["_vcap"] = 1.0

                # Forward to agent. The agent is the only entity that talks
                # to the physical robot.
                if not room.agent:
                    await safe_send(session, {
                        "type":   "cmd_rejected",
                        "seq":    data.get("seq"),
                        "reason": "agent_offline",
                    })
                    continue
                # Tag origin for the agent's own audit
                data["_origin"] = {
                    "session": session.id,
                    "role":    session.role,
                    "site":    session.site,
                }
                await safe_send(room.agent, data)
                await safe_send(session, {
                    "type": "cmd_ack", "seq": data.get("seq")})
                continue

            # ---- Non-motion control plane (host-only for safety) ----
            if mtype in ("get_urp_list", "dashboard"):
                if session.role != "host":
                    await safe_send(session, {
                        "type": "cmd_rejected",
                        "reason": "host_only_command",
                    })
                    continue
                if room.agent:
                    await safe_send(room.agent, data)
                continue

            log.debug("unhandled message from %s/%s: %s",
                      session.role, session.site, mtype)

    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        log.exception("connection loop error: %s", e)
    finally:
        if ping_task is not None:
            ping_task.cancel()
        if session is not None and room is not None:
            async with room.lock:
                if session.role == "agent" and room.agent is session:
                    room.agent = None
                elif session.role == "host" and room.host_operator is session:
                    room.host_operator = None
                elif session.role == "guest":
                    room.guests.discard(session)

                # If this session was the active remote, force E-STOP
                if room.remote_session is session:
                    room.state = Room.LOCKED
                    room.remote_session = None
                    log.warning("remote %s disconnected mid-control — E-STOP",
                                session.id)
                    if room.agent:
                        await safe_send(room.agent, {"type": "estop"})
                    audit("remote_disconnect_estop", session)
                    await broadcast_room(room, {
                        "type":  "authority_changed",
                        "state": "locked",
                        "by":    "remote_disconnect",
                    })
                # If the agent dropped, lock the room — nobody can drive
                # without an agent anyway
                if session.role == "agent":
                    room.state = Room.HOST_OPERATOR  # unlocked, but no agent
                    log.warning("agent for room %s went offline", room.id)
                    await broadcast_room(room, {
                        "type":  "agent_status",
                        "online": False,
                    })

            audit("session_close", session)
            log.info("session CLOSE: id=%s role=%s", session.id, session.role)


# ============================================================
# Server entrypoint
# ============================================================
async def main():
    asyncio.create_task(watchdog_loop())

    log.info("=" * 64)
    log.info(" SONAIR Cloud Relay starting")
    log.info(" Listening: ws://%s:%d  (use TLS termination in production)",
             RELAY_HOST, RELAY_PORT)
    log.info(" Audit dir: %s", AUDIT_DIR.resolve())
    if AGENT_TOKEN.endswith("replace-me"):
        log.warning(" !! AGENT_TOKEN is default — set RELAY_AGENT_TOKEN env var !!")
    if OPERATOR_TOKEN_HOST.endswith("replace-me") \
       or OPERATOR_TOKEN_GUEST.endswith("replace-me"):
        log.warning(" !! Operator tokens are default — set RELAY_HOST_TOKEN "
                    "and RELAY_GUEST_TOKEN env vars !!")
    log.info("=" * 64)

    async with websockets.serve(
        handle_connection, RELAY_HOST, RELAY_PORT,
        # Generous limits — we do framing & rate limiting in app layer
        max_size=8 * 1024 * 1024,
        ping_interval=20,
        ping_timeout=20,
    ):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info("relay stopped by user")

"""
Bridges the daemon's ZMQ PUB socket (port 5555) into 
validated Pydantic model instances, and fans them out to every 
connected browser WebSocket.

ARCHITECTURE NOTE: there is exactly ONE upstream publisher — the
daemon publishes a single DaemonStatus JSON blob per tick (every
0.5s), containing rotor state, spectrum frame, logs, etc. all
together.

This module reads from that ZMQ socket exactly ONCE regardless of how
many browser tabs are connected — a single background asyncio task
subscribes to ZMQ, validates each message into a DaemonStatus, and
fans the resulting JSON out to every currently-connected WebSocket.
This avoids N redundant ZMQ subscriptions + N redundant
model_validate() calls for N browser tabs.

Browser -> daemon is `CommandListener` below: the daemon BINDs a PULL on
5556 and recv_string()s a text language, not JSON. Also one instance per
process, not one per tab.
"""

import asyncio
import json
import logging

import zmq
import zmq.asyncio
from pydantic import ValidationError
from starlette.websockets import WebSocket

from ...daemon.telescope_types import DaemonStatus
from ...daemon.command_types import (
    CalibrateEncoders,
    EmergencyStop,
    FindObjectLocation,
    ObservationPlan,
    PointAtAzEl,
    PointAtObject,
    PointAtOffset,
    SpectrumConfig,
    SpectrumStart,
    SpectrumStop,
    Stow,
    TelescopeCommand,
    Wait,
    WaitUntil,
)

_ctx = zmq.asyncio.Context.instance()


class StatusBroadcaster:
    """
    Owns the single ZMQ SUB connection to the daemon's status PUB
    socket, and fans validated DaemonStatus JSON out to every
    currently-connected WebSocket client.

    One instance for the whole FastAPI process (see `status_broadcaster`
    below) — not one per connection.
    """

    def __init__(self, endpoint: str = "tcp://localhost:5555"):
        self.endpoint = endpoint
        self._socket: zmq.asyncio.Socket | None = None
        self._clients: set[WebSocket] = set()
        self._latest: DaemonStatus | None = None
        self._task: asyncio.Task | None = None

    async def start(self) -> None:
        """Call once, e.g. from a FastAPI startup event. Connects to
        the daemon's status PUB socket and starts the background
        fan-out loop."""
        self._socket = _ctx.socket(zmq.SUB)
        self._socket.connect(self.endpoint)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._task = asyncio.create_task(self._listen_and_broadcast())
        logging.info("StatusBroadcaster connected to %s", self.endpoint)

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
        if self._socket is not None:
            self._socket.close()

    async def _listen_and_broadcast(self) -> None:
        assert self._socket is not None
        while True:
            try:
                raw = await self._socket.recv_string()
                parsed = json.loads(raw)
                status = DaemonStatus.model_validate(parsed)
            except json.JSONDecodeError as e:
                logging.warning("StatusBroadcaster: malformed JSON from daemon: %s", e)
                continue
            except ValidationError as e:
                logging.warning("StatusBroadcaster: DaemonStatus validation failed: %s", e)
                continue

            self._latest = status
            await self._broadcast(status.model_dump_json())

    async def _broadcast(self, payload: str) -> None:
        if not self._clients:
            return
        dead: list[WebSocket] = []
        for ws in self._clients:
            try:
                await ws.send_text(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._clients.discard(ws)

    def register(self, websocket: WebSocket) -> None:
        self._clients.add(websocket)

    def unregister(self, websocket: WebSocket) -> None:
        self._clients.discard(websocket)

    @property
    def latest(self) -> DaemonStatus | None:
        """Most recent validated status, if any — used to send an
        immediate snapshot to a client right after it connects,
        rather than making it wait up to 0.5s for the next tick."""
        return self._latest


# One instance for the whole FastAPI process. Created here so the
# WebSocket route module and the app startup/shutdown hooks (see
# main.py) share the same object.
status_broadcaster = StatusBroadcaster()


# ---------------------------------------------------------------------------
# Browser -> daemon
# ---------------------------------------------------------------------------


class CommandError(Exception):
    """Reportable to the browser as `{"ok": false}`, not a 500."""


class DaemonUnreachable(CommandError):
    """No connected peer — not queued, and won't arrive later."""


class CommandRejected(CommandError):
    """Valid Pydantic, but not doable (unknown object, out of limits)."""


def encode_command(cmd: TelescopeCommand) -> str:
    """Pydantic command -> daemon text language (daemon.py `srt_daemon_main`).

    Object ids are case-sensitive and unkeyworded; unset spectrum fields are
    dropped so they don't clobber the driver's current values.
    """
    if isinstance(cmd, PointAtObject):
        return cmd.object_id
    if isinstance(cmd, FindObjectLocation):
        return f"object {cmd.object_id}"
    if isinstance(cmd, PointAtAzEl):
        return f"azel {cmd.azimuth:.6f} {cmd.elevation:.6f}"
    if isinstance(cmd, PointAtOffset):
        return f"offset {cmd.azimuth_offset:.6f} {cmd.elevation_offset:.6f}"
    if isinstance(cmd, SpectrumConfig):
        fields = cmd.model_dump(exclude_none=True, exclude={"command"})
        if not fields:  # daemon would no-op silently
            raise CommandRejected("spectrum_config with no fields set")
        pairs = " ".join(f"{k}={v}" for k, v in sorted(fields.items()))
        return f"spectrum_config {pairs}"
    if isinstance(cmd, Wait):
        return f"wait {cmd.duration_seconds:.6f}"
    if isinstance(cmd, WaitUntil):
        return cmd.time.strftime("%Y:%j:%H:%M:%S")
    if isinstance(cmd, Stow):
        return "stow"
    if isinstance(cmd, CalibrateEncoders):
        return "calibrate_encoders"
    if isinstance(cmd, SpectrumStart):
        return "spectrum_start"
    if isinstance(cmd, SpectrumStop):
        return "spectrum_stop"
    if isinstance(cmd, EmergencyStop):  # guard; `submit` intercepts it first
        raise CommandRejected("EmergencyStop belongs on the e-stop socket, not the queue")

    # Only reachable if someone extends TelescopeCommand and forgets this.
    raise CommandRejected(f"no daemon encoding for command type {type(cmd).__name__}")


class CommandListener:
    """One per process: sockets track daemons, not browser tabs."""

    def __init__(
        self,
        endpoint: str = "tcp://localhost:5556",
        estop_endpoint: str = "tcp://localhost:5567",
    ):
        self.endpoint = endpoint

        # Moore6mController binds a PULL here and calls moore6m.spa()
        # directly, bypassing the daemon and whatever it's blocked inside.
        self.estop_endpoint = estop_endpoint

        self._socket: zmq.asyncio.Socket | None = None
        self._estop_socket: zmq.asyncio.Socket | None = None
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def start(self) -> None:
        """Call once from the lifespan startup, next to status_broadcaster.start()."""
        sock = _ctx.socket(zmq.PUSH)

        # Without IMMEDIATE, PUSH buffers for a reconnecting peer: a command
        # sent while the daemon is down replays on restart, slewing the dish
        # to a target someone asked for 20 minutes ago.
        sock.setsockopt(zmq.IMMEDIATE, 1)
        sock.setsockopt(zmq.LINGER, 0)   # don't stall shutdown
        sock.setsockopt(zmq.SNDHWM, 64)  # fail fast rather than backlog

        sock.connect(self.endpoint)
        self._socket = sock

        estop = _ctx.socket(zmq.PUSH)
        estop.setsockopt(zmq.IMMEDIATE, 1)
        estop.setsockopt(zmq.LINGER, 0)
        estop.setsockopt(zmq.SNDHWM, 8)
        estop.connect(self.estop_endpoint)
        self._estop_socket = estop

        logging.info(
            "CommandListener connected: commands=%s estop=%s",
            self.endpoint, self.estop_endpoint,
        )

    async def stop(self) -> None:
        for attr in ("_socket", "_estop_socket"):
            sock = getattr(self, attr)
            if sock is not None:
                sock.close()
                setattr(self, attr, None)

    # -- client bookkeeping ---------------------------------------------
    # Diagnostics only — acks go to the requester, and other tabs see the
    # effect via the status stream.

    def register(self, websocket: WebSocket) -> None:
        self._clients.add(websocket)

    def unregister(self, websocket: WebSocket) -> None:
        self._clients.discard(websocket)

    @property
    def client_count(self) -> int:
        return len(self._clients)

    # -- submission ---------------------------------------------------------

    async def submit(self, cmd: TelescopeCommand) -> str:
        """Push one command; returns the text sent, which reappears in the
        daemon's `queued_item`. Raises CommandRejected / DaemonUnreachable.
        """
        if isinstance(cmd, EmergencyStop):
            return await self._send_estop()

        self._preflight(cmd)
        line = encode_command(cmd)
        async with self._lock:
            await self._send(line)
        return line

    async def _send_estop(self) -> str:
        """Push to the controller's e-stop socket (payload ignored; any
        message fires it). Skips `self._lock` — an e-stop queued behind a
        40-step plan send isn't an e-stop.

        !! SAFETY GAP, not fixed here: `Moore6mController._estop_loop` calls
        spa() only, while its own GUI button calls spa() AND
        set_safe_mode(True). Only the latter blocks further motion, so this
        halts the dish without latching — the next queued command re-slews.
        """
        if self._estop_socket is None:
            raise DaemonUnreachable("CommandListener.start() was never called")
        try:
            await self._estop_socket.send_string("estop", flags=zmq.NOBLOCK)
        except zmq.Again as e:
            raise DaemonUnreachable(
                f"controller not connected on {self.estop_endpoint}; "
                f"E-STOP WAS NOT DELIVERED — use the physical stop"
            ) from e
        logging.warning("CommandListener -> controller: E-STOP")
        return "estop"

    async def submit_plan(self, plan: ObservationPlan) -> list[str]:
        """Push a plan in order, nothing spliced in. Encoded and pre-flighted
        whole first, so a bad step 4 doesn't execute steps 1-3. If the daemon
        dies mid-loop the rest are already queued — hence the count in the
        error, which the caller should surface.
        """
        for cmd in plan.commands:
            self._preflight(cmd)
        lines = [encode_command(cmd) for cmd in plan.commands]

        sent = 0
        async with self._lock:
            for line in lines:
                try:
                    await self._send(line)
                except DaemonUnreachable as e:
                    raise DaemonUnreachable(
                        f"plan {plan.name or '(unnamed)'} aborted after "
                        f"{sent}/{len(lines)} command(s) were already queued "
                        f"on the daemon: {e}"
                    ) from e
                sent += 1
        return lines

    async def _send(self, line: str) -> None:
        if self._socket is None:
            raise DaemonUnreachable("CommandListener.start() was never called")
        try:
            # NOBLOCK + IMMEDIATE: raises Again when no daemon is attached.
            await self._socket.send_string(line, flags=zmq.NOBLOCK)
        except zmq.Again as e:
            raise DaemonUnreachable(
                f"daemon not connected on {self.endpoint}; command {line!r} was not sent"
            ) from e
        logging.info("CommandListener -> daemon: %s", line)

    # -- pre-flight ---------------------------------------------------------

    def _preflight(self, cmd: TelescopeCommand) -> None:
        """Check against the latest DaemonStatus. Best-effort — skipped on a
        cold start, and the daemon re-checks. Error messages, not safety.
        """
        status = status_broadcaster.latest
        if status is None:
            return

        if isinstance(cmd, (PointAtObject, FindObjectLocation)):
            # The daemon matches a single whitespace token against
            # ephemeris_locations, so a spaced id vanishes as "Command Not
            # Identified". sky_coords.csv is space-free, but unenforced.
            if cmd.object_id != cmd.object_id.strip() or " " in cmd.object_id:
                raise CommandRejected(
                    f"object id {cmd.object_id!r} contains whitespace; the daemon's "
                    f"command language cannot address it (rename it in sky_coords.csv)"
                )
            if status.object_locs and cmd.object_id not in status.object_locs:
                known = ", ".join(sorted(status.object_locs)) or "(none)"
                raise CommandRejected(
                    f"unknown object {cmd.object_id!r}; daemon knows: {known}"
                )

        if isinstance(cmd, PointAtAzEl):
            el_lo, el_hi = status.el_limits
            if not (el_lo <= cmd.elevation <= el_hi):
                raise CommandRejected(
                    f"elevation {cmd.elevation} outside mount limits "
                    f"[{el_lo}, {el_hi}]"
                )
            az_lo, az_hi = status.az_limits
            if not (az_lo <= cmd.azimuth <= az_hi):
                raise CommandRejected(
                    f"azimuth {cmd.azimuth} outside mount limits [{az_lo}, {az_hi}]"
                )

        # PointAtOffset isn't checked: relative to wherever the dish lands.


command_listener = CommandListener()
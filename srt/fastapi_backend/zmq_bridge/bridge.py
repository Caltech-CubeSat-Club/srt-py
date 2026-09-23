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

The reverse direction (browser -> daemon) is `CommandListener` at the
bottom of this file. Two corrections to what the old TODO here assumed:

  * The daemon's command socket is a **PULL** socket *bound* on
    `tcp://*:5556` (daemon.py `update_command_queue`), not a SUB. So this
    side is a PUSH that *connects*. PUSH/PULL is load-balanced, not
    fan-out — which is what we want: exactly one consumer, no dropped
    commands, ordering preserved per connection.
  * The daemon does **not** speak JSON on that socket. It `recv_string()`s
    and parses a legacy whitespace-delimited text language
    (daemon.py `srt_daemon_main`). The Pydantic models are the *browser*
    wire format only; `encode_command()` below translates them.

Like StatusBroadcaster, CommandListener is one instance per process with
one socket — not one per browser tab, and no threads. A PUSH socket is
cheap to share and pyzmq sockets are not safe to use from multiple
threads anyway; concurrency here is asyncio, and an `asyncio.Lock`
serializes sends so a multi-command ObservationPlan can't interleave with
another tab's commands mid-plan.
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
    """Base for command-submission failures that should be reported to the
    browser as a clean `{"ok": false, ...}` rather than a 500/close."""


class DaemonUnreachable(CommandError):
    """The daemon's PULL socket has no completed connection right now, so the
    command was NOT queued anywhere and will NOT be delivered later."""


class CommandRejected(CommandError):
    """The command is structurally valid Pydantic but can't be carried out
    (unknown object, outside the mount's limits, ...). Rejected here so the
    operator gets an immediate reason instead of the daemon silently logging
    'Command Not Identified' into a status field nobody is reading."""


def encode_command(cmd: TelescopeCommand) -> str:
    """Translate one Pydantic command into the daemon's text command language.

    Mapping is taken directly from daemon.py's `srt_daemon_main` dispatch
    chain. Notes on the non-obvious ones:

    * `PointAtObject` encodes as the bare object name — the daemon checks
      `command_parts[0] in self.ephemeris_locations` *before* the keyword
      chain, and that check is case-sensitive, so the id is not lowercased.
    * `EmergencyStop` is NOT handled here — it does not go through the
      command queue at all. See `CommandListener._send_estop`.
    * `WaitUntil` uses the daemon's 5-field `%Y:%j:%H:%M:%S` form (it selects
      that branch purely on `len(token.split(":")) == 5`). `command_types`
      has already normalized the datetime to UTC, which is what the daemon
      compares against. The daemon's other time branch, `LST:`, is
      unreachable dead code (it lowercases the token, then tests for the
      uppercase literal), so don't try to route through it.
    * Floats are formatted fixed-point, never exponential — the daemon splits
      on whitespace and calls `float()` per token, and while `float()` does
      parse `1e-07`, fixed-point keeps the operator-facing log readable.
    * `spectrum_config` becomes `key=value` pairs; the daemon's
      `_parse_key_value_pairs` splits on the first `=` and ignores any token
      without one. Values must therefore contain no spaces (all numeric
      here, so fine). `None` fields are dropped so an unset field doesn't
      overwrite the driver's current value.
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
        if not fields:
            # The daemon no-ops on an empty update anyway; rejecting here
            # makes the pointless round-trip visible to whoever sent it.
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
    if isinstance(cmd, EmergencyStop):
        # Guard, not a mapping. Encoding an e-stop as the queued `spa`
        # keyword would put it behind every blocking command already in the
        # daemon's FIFO — an emergency stop that waits out a 180s slew.
        # `CommandListener.submit` intercepts it before reaching here.
        raise CommandRejected(
            "EmergencyStop must not be encoded into the command queue; "
            "it is routed to the controller's dedicated e-stop socket"
        )

    # Reachable only if someone adds a member to the TelescopeCommand union and forgets this function.
    raise CommandRejected(f"no daemon encoding for command type {type(cmd).__name__}")


class CommandListener:
    """Owns the single ZMQ PUSH connection to the daemon's command PULL
    socket, and accepts commands from any number of browser WebSockets.

    One instance for the whole FastAPI process (see `command_listener`
    below) — mirroring StatusBroadcaster, and for the same reason: the
    number of sockets should track the number of *daemons* (one), not the
    number of browser tabs.
    """

    def __init__(
        self,
        endpoint: str = "tcp://localhost:5556",
        estop_endpoint: str = "tcp://localhost:5567",
    ):
        self.endpoint = endpoint

        # Second, separate channel. `Moore6mController` binds a PULL on 5567
        # (DEFAULT_ESTOP_PORT) whose loop discards the message body and calls
        # `moore6m.spa()` directly on the shared driver object — bypassing the
        # daemon, its command queue, and whatever blocking call the daemon is
        # currently sitting inside. That is the entire point of it, and it is
        # why EmergencyStop must not travel over `self.endpoint`.
        self.estop_endpoint = estop_endpoint

        self._socket: zmq.asyncio.Socket | None = None
        self._estop_socket: zmq.asyncio.Socket | None = None
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def start(self) -> None:
        """Call once from the FastAPI lifespan startup, alongside
        `status_broadcaster.start()`."""
        sock = _ctx.socket(zmq.PUSH)

        # IMMEDIATE=1 is the safety-relevant option here. By default a PUSH
        # socket happily queues messages for a peer that is merely *being*
        # connected, and ZMQ's automatic reconnect means a command sent
        # while the daemon is down would be silently delivered whenever it
        # comes back — i.e. the dish could start slewing to a target someone
        # requested twenty minutes ago and gave up on. With IMMEDIATE=1,
        # messages are only queued to completed connections, so a send with
        # NOBLOCK raises EAGAIN instead and we can tell the operator "not
        # sent" truthfully.
        sock.setsockopt(zmq.IMMEDIATE, 1)

        # Don't let a queued-but-undelivered command hold up process exit.
        sock.setsockopt(zmq.LINGER, 0)

        # Bounded local queue. The daemon drains 5556 into an unbounded
        # Python Queue quickly, so this should never fill in practice; if it
        # does, failing fast beats accumulating a backlog of stale pointing
        # commands.
        sock.setsockopt(zmq.SNDHWM, 64)

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

    # -- client bookkeeping -------------------------------------------------
    # Unlike StatusBroadcaster's client set, this one is not used to fan
    # anything out: command acks go back on the requesting socket only, and
    # *other* tabs learn what happened from the status stream, which already
    # carries `queued_item`, `queue_size` and `error_logs`. This set exists
    # so shutdown/diagnostics can see how many command sockets are live.

    def register(self, websocket: WebSocket) -> None:
        self._clients.add(websocket)

    def unregister(self, websocket: WebSocket) -> None:
        self._clients.discard(websocket)

    @property
    def client_count(self) -> int:
        return len(self._clients)

    # -- submission ---------------------------------------------------------

    async def submit(self, cmd: TelescopeCommand) -> str:
        """Validate, encode, and push one command. Returns the exact text
        handed to the daemon (useful to echo back in the ack, since that
        string is what shows up in the daemon's own logs and `queued_item`).

        Raises CommandRejected or DaemonUnreachable.
        """
        if isinstance(cmd, EmergencyStop):
            return await self._send_estop()

        self._preflight(cmd)
        line = encode_command(cmd)
        async with self._lock:
            await self._send(line)
        return line

    async def _send_estop(self) -> str:
        """Push to the controller's dedicated e-stop socket.

        Deliberately does NOT take `self._lock`: the lock exists to keep one
        client's multi-command plan contiguous on the *command* socket, and
        an e-stop queued behind a 40-step plan send is not an e-stop. It's a
        different socket, so there is nothing to interleave with anyway.

        The payload is ignored by the receiver (`_estop_loop` discards the
        result of `recv_string()` and fires immediately on any message), so
        the string is purely for whoever reads a packet capture.

        !! SAFETY GAP, NOT FIXED HERE — see `Moore6mController._estop_loop`:
        the ZMQ path calls `_send_spa_immediate()` (which is only
        `moore6m.spa()`), while the Tk GUI's own e-stop button does
        `spa()` *and then* `set_safe_mode(True)`. Only `set_safe_mode` makes
        the driver block subsequent motion commands. So an e-stop arriving
        over this socket halts the motors but does not latch: the daemon's
        next queued motion command will slew the dish again. That one-line
        asymmetry is in the controller, on someone else's branch, and
        changing e-stop semantics isn't a call to make silently.
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
        """Push every command in an observation plan, in order, with no other
        client's commands interleaved between them.

        Encoding and pre-flight happen for the *whole* plan before anything
        is sent, so a plan with a bad step 4 is rejected without having
        already executed steps 1-3.

        Note the remaining partial-failure window: if the daemon dies midway
        through the send loop, the earlier commands are already in its queue.
        That's not fixable from this side — the daemon's PULL socket has no
        acknowledgement — so the raised error reports how many made it, and
        the caller should surface that count rather than a bare "failed".
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
            # NOBLOCK + IMMEDIATE: raises Again when no daemon is attached,
            # rather than awaiting forever on a socket nobody is reading.
            await self._socket.send_string(line, flags=zmq.NOBLOCK)
        except zmq.Again as e:
            raise DaemonUnreachable(
                f"daemon not connected on {self.endpoint}; command {line!r} was not sent"
            ) from e
        logging.info("CommandListener -> daemon: %s", line)

    # -- pre-flight ---------------------------------------------------------

    def _preflight(self, cmd: TelescopeCommand) -> None:
        """Cheap checks against the most recent DaemonStatus.

        Pydantic validates *shape*; this validates against the live machine.
        Everything here is best-effort: if no status has arrived yet
        (`latest is None`, cold start), all checks are skipped rather than
        blocking the operator on a stale assumption. The daemon re-checks its
        own bounds regardless — this is for error messages, not for safety.
        """
        status = status_broadcaster.latest
        if status is None:
            return

        if isinstance(cmd, (PointAtObject, FindObjectLocation)):
            # The daemon splits on whitespace and only ever tests a *single*
            # token against ephemeris_locations (`command_parts[0]` for the
            # bare-name form, `command_parts[-1]` after `object`). An id
            # containing a space is therefore inexpressible in the command
            # language — it would be silently swallowed as "Command Not
            # Identified". Every name in sky_coords.csv is currently
            # space-free (CassA, SgrA, GNpole, ...), evidently on purpose,
            # but nothing enforces that at the CSV, so catch it here rather
            # than letting a new catalog entry fail invisibly at 2am.
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

        # PointAtOffset is deliberately not bounds-checked: it's relative to
        # wherever the dish ends up when the command is dequeued, which is
        # not knowable from here if anything is queued ahead of it.


# One instance for the whole FastAPI process, same rationale as
# `status_broadcaster` above.
command_listener = CommandListener()
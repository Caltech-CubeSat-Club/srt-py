"""
Browser -> daemon. Everything a browser sends on /ws/command is one
command_types.ClientRequest, and `CommandListener.submit_request` takes it
through three steps:

    check(request, status)   can it be done?  raises CommandRejected / ValidationError
    encode(request)          the daemon lines it becomes
    send                     push them to the daemon over ZMQ 5556

The daemon's protocol is lines: a verb, then arguments. Simple commands take
plain arguments (`azel 120.500000 45.000000`, `stow`); structured payloads
take a single JSON argument (`update_settings {...}`). It stays typeable by
hand, on purpose.

EmergencyStop skips all of it and goes to the controller's own e-stop
socket (5567), so it can't queue behind whatever the daemon is blocked on.

Daemon -> browser is status.py.
"""

import asyncio
import json
import logging

import zmq
import zmq.asyncio
from starlette.websockets import WebSocket

from ...daemon.command_types import (
    CalibrateEncoders,
    ClientRequest,
    CommandRequest,
    EmergencyStop,
    PointAtAzEl,
    PointAtObject,
    PointAtOffset,
    SettingsPatch,
    SpectrumStart,
    SpectrumStop,
    Stow,
    SubmitPlan,
    TelescopeCommand,
    Wait,
    WaitUntil,
)
from ...daemon.settings import merge_settings
from ...daemon.status import DaemonStatus
from .status import status_broadcaster

_ctx = zmq.asyncio.Context.instance()


class CommandError(Exception):
    """Reportable to the browser as `{"ok": false}`, not a 500."""


class DaemonUnreachable(CommandError):
    """No connected peer — not queued, and won't arrive later."""


class CommandRejected(CommandError):
    """Valid Pydantic, but not doable (unknown object, out of limits)."""


# ---------------------------------------------------------------------------
# check — against the latest DaemonStatus. Best-effort: skipped on a cold
# start, and the daemon re-checks. These exist for prompt error messages,
# not for safety.
# ---------------------------------------------------------------------------


def check(req: ClientRequest, status: DaemonStatus | None) -> None:
    if status is None:
        return
    if isinstance(req, CommandRequest):
        check_command(req.command, status)
    elif isinstance(req, SettingsPatch) and status.settings is not None:
        # Raises ValidationError if the patch wouldn't validate on top of the
        # daemon's current settings.
        merge_settings(status.settings, req.patch)
    elif isinstance(req, SubmitPlan):
        for cmd in req.plan.commands:
            check_command(cmd, status)


def check_command(cmd: TelescopeCommand, status: DaemonStatus) -> None:
    if isinstance(cmd, PointAtObject):
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
            raise CommandRejected(f"unknown object {cmd.object_id!r}; daemon knows: {known}")

    if isinstance(cmd, PointAtAzEl):
        el_lo, el_hi = status.el_limits
        if not (el_lo <= cmd.elevation <= el_hi):
            raise CommandRejected(
                f"elevation {cmd.elevation} outside mount limits [{el_lo}, {el_hi}]"
            )
        az_lo, az_hi = status.az_limits
        if not (az_lo <= cmd.azimuth <= az_hi):
            raise CommandRejected(
                f"azimuth {cmd.azimuth} outside mount limits [{az_lo}, {az_hi}]"
            )

    # PointAtOffset isn't checked: relative to wherever the dish lands.


# ---------------------------------------------------------------------------
# encode — to daemon lines (daemon.py `srt_daemon_main`)
# ---------------------------------------------------------------------------


def encode(req: ClientRequest) -> list[str]:
    if isinstance(req, CommandRequest):
        return [encode_command(req.command)]
    if isinstance(req, SettingsPatch):
        return [encode_settings(req.patch)]
    if isinstance(req, SubmitPlan):
        # Until the daemon can run plans, a plan is its commands queued in
        # order, starting now. Refuse timing that would silently be ignored:
        # a plan for 03:00 would run immediately, with no time cap.
        if req.plan.start_time is not None or req.plan.max_duration_seconds is not None:
            raise CommandRejected(
                "start_time and max_duration_seconds need the daemon's plan "
                "scheduler, which isn't built yet; without them the plan runs "
                "now, in order"
            )
        return [encode_command(cmd) for cmd in req.plan.commands]
    raise CommandRejected(
        f"plan operation {req.op!r} needs the daemon's plan scheduler, which "
        f"isn't built yet (docs/command-execution-design.md)"
    )


def encode_settings(patch: dict) -> str:
    # Compact separators keep the JSON one whitespace-free token after the verb.
    return f"update_settings {json.dumps(patch, separators=(',', ':'))}"


def encode_command(cmd: TelescopeCommand) -> str:
    """Object ids are case-sensitive and unkeyworded."""
    if isinstance(cmd, PointAtObject):
        return cmd.object_id
    if isinstance(cmd, PointAtAzEl):
        return f"azel {cmd.azimuth:.6f} {cmd.elevation:.6f}"
    if isinstance(cmd, PointAtOffset):
        return f"offset {cmd.azimuth_offset:.6f} {cmd.elevation_offset:.6f}"
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
    if isinstance(cmd, EmergencyStop):  # guard; submit_request intercepts it first
        raise CommandRejected("EmergencyStop belongs on the e-stop socket, not the queue")

    # Reached by the observation commands (ObserveObject, GridScan, ...),
    # which the daemon can't run until it has a plan scheduler. Also reached
    # if someone extends TelescopeCommand and forgets this function.
    raise CommandRejected(
        f"{type(cmd).__name__} can't be sent yet; observation commands need the "
        f"daemon's plan scheduler"
    )


# ---------------------------------------------------------------------------
# send
# ---------------------------------------------------------------------------


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

    async def submit_request(self, req: ClientRequest) -> list[str]:
        """Check, encode, send. Returns the lines sent, which reappear in the
        daemon's `queued_item`. Raises CommandRejected, DaemonUnreachable, or
        ValidationError (a settings patch that wouldn't validate).

        Everything is checked and encoded before the first line goes out, so
        a bad step 4 of a plan doesn't execute steps 1-3.
        """
        if isinstance(req, CommandRequest) and isinstance(req.command, EmergencyStop):
            return [await self._send_estop()]

        check(req, status_broadcaster.latest)
        lines = encode(req)

        sent = 0
        async with self._lock:
            for line in lines:
                try:
                    await self._send(line)
                except DaemonUnreachable as e:
                    if sent == 0:
                        raise
                    # The rest are already on the daemon's queue — the caller
                    # should surface the count.
                    raise DaemonUnreachable(
                        f"aborted after {sent}/{len(lines)} line(s) were already "
                        f"queued on the daemon: {e}"
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
                f"daemon not connected on {self.endpoint}; {line!r} was not sent"
            ) from e
        logging.info("CommandListener -> daemon: %s", line)

    async def _send_estop(self) -> str:
        """Push to the controller's e-stop socket (payload ignored; any
        message fires it). Skips `self._lock`. Raises DaemonUnreachable if
        the controller isn't connected.
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


command_listener = CommandListener()

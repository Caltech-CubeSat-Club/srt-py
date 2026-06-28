"""
Bridges the daemon's ZMQ PUB socket (port 5555, confirmed from the real
daemon.py's update_status() and the existing Dash dashboard's
status_fetcher.py) into validated DaemonStatus model instances, and
fans them out to every connected browser WebSocket.

ARCHITECTURE NOTE: there is exactly ONE upstream publisher — the
daemon publishes a single DaemonStatus JSON blob per tick (every
0.5s), containing rotor state, spectrum frame, logs, etc. all
together. This replaces an earlier draft of this file that assumed
two separate publishers (rotor-only, spectrum-only) — that was never
how the real system worked; confirmed by reading status_fetcher.py,
which does exactly one ZMQ SUB on port 5555 and one
DaemonStatus.from_dict() call.

This module reads from that ZMQ socket exactly ONCE regardless of how
many browser tabs are connected — a single background asyncio task
subscribes to ZMQ, validates each message into a DaemonStatus, and
fans the resulting JSON out to every currently-connected WebSocket.
This avoids N redundant ZMQ subscriptions + N redundant
model_validate() calls for N browser tabs, and is the natural place
to track a real connected-client count (see `client_count` below) —
this does NOT belong on DaemonStatus itself, since client count is a
property of the web transport layer, not of telescope state.
"""

import asyncio
import json
import logging

import zmq
import zmq.asyncio
from pydantic import ValidationError
from starlette.websockets import WebSocket

from ...daemon.telescope_types import DaemonStatus

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

    @property
    def client_count(self) -> int:
        """Number of currently-connected WebSocket clients. This is
        the real implementation of what an earlier draft tried to put
        as a `connected_clients` field on DaemonStatus itself — kept
        here instead, since it's a transport-layer concern, not
        telescope state."""
        return len(self._clients)

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
                # The daemon publishes with send_string(json.dumps(...)),
                # NOT zmq's send_json — recv_string() + json.loads() is
                # the correct match (recv_json() would also technically
                # work since it's valid JSON either way, but matching
                # the daemon's actual send_* call makes the pairing
                # explicit rather than coincidentally compatible).
                raw = await self._socket.recv_string()
                parsed = json.loads(raw)
                status = DaemonStatus.model_validate(parsed)
            except json.JSONDecodeError as e:
                logging.warning("StatusBroadcaster: malformed JSON from daemon: %s", e)
                continue
            except ValidationError as e:
                # Same failure mode flagged earlier in this conversion:
                # the daemon sent a shape that doesn't match the model.
                # Drop this tick and keep going — a single bad publish
                # shouldn't take down the broadcast loop for every
                # connected browser.
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
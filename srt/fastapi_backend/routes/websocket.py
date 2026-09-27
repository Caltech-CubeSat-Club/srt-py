"""
WebSocket endpoint streaming telescope status to connected browser
clients.

ARCHITECTURE: there is exactly one daemon-side publisher (ZMQ PUB on
port 5555) sending one DaemonStatus JSON blob per tick — see 
daemon.py's update_status() and the existing Dash
dashboard's status_fetcher.py, which does exactly this with one
ZMQ SUB. 

ZMQ <-> Pydantic conversion happens exactly once per tick, in
zmq_bridge.status.StatusBroadcaster — a single background task shared
by the whole FastAPI process, not one ZMQ subscription per browser
tab. This route just registers/unregisters each WebSocket with that
shared broadcaster; it never touches ZMQ directly.
"""

import json
import logging

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import TypeAdapter, ValidationError

from .auth import get_current_user_ws
from ..zmq_bridge.commands import CommandError, command_listener
from ..zmq_bridge.status import status_broadcaster
from ...daemon.command_types import ClientRequest

router = APIRouter()
_request_adapter = TypeAdapter(ClientRequest)


@router.websocket("/ws/status")
async def status_ws(websocket: WebSocket, token: str | None = None):
    # Token arrives as a query param (?token=...) since browsers can't
    # set Authorization headers on the WS handshake — see auth.py.
    # Validate BEFORE accept() so unauthenticated clients get a clean
    # rejection rather than a connection that's accepted then dropped.
    await get_current_user_ws(token)
    await websocket.accept()

    status_broadcaster.register(websocket)
    try:
        # Send an immediate snapshot on connect rather than making the
        # client wait up to 0.5s for the next daemon tick — only if
        # the broadcaster has actually received at least one tick
        # since it started (None on a cold start before the daemon's
        # first publish).
        if status_broadcaster.latest is not None:
            await websocket.send_text(status_broadcaster.latest.model_dump_json())

        # Nothing else to do from this coroutine's side — the
        # broadcaster's background task pushes new status to this
        # socket directly via _broadcast(). We just need to keep this
        # coroutine alive to detect disconnect, and to handle any
        # client->server messages if the protocol ever needs them
        # (currently it doesn't; commands go over the REST/command
        # endpoints, not this socket).
        while True:
            # recv_text() raises WebSocketDisconnect when the client
            # closes — that's the only thing this loop is actually
            # waiting for. If you never expect the client to send
            # anything, this still correctly detects disconnects.
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        status_broadcaster.unregister(websocket)


@router.websocket("/ws/command")
async def command_ws(websocket: WebSocket, token: str | None = None):
    await get_current_user_ws(token)
    await websocket.accept()

    command_listener.register(websocket)

    try:
        while True:
            raw = await websocket.receive_text()

            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                await websocket.send_text(json.dumps({
                    "ok": False,
                    "error": "Invalid JSON",
                }))
                continue

            # One envelope for everything: command_types.ClientRequest,
            # discriminated by `kind`.
            try:
                lines = await command_listener.submit_request(
                    _request_adapter.validate_python(data)
                )
            except ValidationError as e:
                # Context can hold the raising exception itself (model-level
                # validators), which json.dumps can't serialize.
                await websocket.send_text(json.dumps({
                    "ok": False,
                    "error": e.errors(include_url=False, include_context=False, include_input=False),
                }))
                continue
            except CommandError as e:
                # Normal outcome for operator input, not a server fault —
                # keep the socket open.
                await websocket.send_text(json.dumps({
                    "ok": False,
                    "error": str(e),
                }))
                continue

            # Echo the daemon-language text back — it reappears in the status
            # stream's queued_item/error_logs, so the client can correlate.
            await websocket.send_text(json.dumps({
                "ok": True,
                "sent": lines,
            }))

    except WebSocketDisconnect:
        pass
    finally:
        command_listener.unregister(websocket)
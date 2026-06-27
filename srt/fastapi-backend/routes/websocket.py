"""
WebSocket endpoints streaming telescope status to connected browser
clients. Raw ZMQ -> Pydantic conversion happens once, in
app.zmq_bridge — these routes only ever handle already-validated
model instances and re-serialize them with .model_dump_json(),
never a hand-built dict via json.dumps().
"""

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from app.models.telescope import TelescopeStatus
from app.routes.auth import get_current_user_ws
from app.zmq_bridge.bridge import rotor_bridge, spectrum_bridge

router = APIRouter()


@router.websocket("/ws/telescope")
async def telescope_status_ws(websocket: WebSocket, token: str | None = None):
    # Token arrives as a query param (?token=...) since browsers can't
    # set Authorization headers on the WS handshake — see auth.py.
    # Validate BEFORE accept() so unauthenticated clients get a clean
    # rejection rather than a connection that's accepted then dropped.
    await get_current_user_ws(token)
    await websocket.accept()

    connected_clients = 1  # placeholder — real app would track this globally

    try:
        async for rotor in rotor_bridge.listen():
            status = TelescopeStatus(
                rotor=rotor,
                latest_spectrum=None,  # see spectrum_ws — separate stream
                connected_clients=connected_clients,
            )
            await websocket.send_text(status.model_dump_json())
    except WebSocketDisconnect:
        pass


@router.websocket("/ws/spectrum")
async def spectrum_ws(websocket: WebSocket, token: str | None = None):
    """Same pattern, separate stream — spectrum sweeps are a
    different cadence/volume than rotor status."""
    await get_current_user_ws(token)
    await websocket.accept()

    try:
        async for reading in spectrum_bridge.listen():
            await websocket.send_text(reading.model_dump_json())
    except WebSocketDisconnect:
        pass

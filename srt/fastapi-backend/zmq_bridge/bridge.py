"""
Bridges raw ZMQ traffic from instrument/control processes
(Moore6mController, the PyVISA spectrum daemon) into validated
Pydantic models.

This is the ONE place raw, untyped ZMQ payloads get converted into
real model instances. Anything downstream of this module — the
WebSocket routes, the REST control endpoints — should only ever see
RotorState / SpectrumReading objects, never raw dicts.

Kept deliberately thin: a real implementation would add reconnect
logic, heartbeat/timeout detection, and probably a pub/sub topic
filter rather than subscribing to everything.
"""

import zmq
import zmq.asyncio
from pydantic import ValidationError

from app.models.telescope import RotorState, SpectrumReading

_ctx = zmq.asyncio.Context.instance()


class ZmqBridge:
    """
    Wraps a single ZMQ SUB socket and exposes an async generator of
    validated model instances. One instance per upstream publisher
    (Moore6mController, the PyVISA daemon), not one global bridge —
    keeps failure/reconnect handling isolated per source.
    """

    def __init__(self, endpoint: str, model: type[RotorState | SpectrumReading]):
        self.endpoint = endpoint
        self.model = model
        self._socket: zmq.asyncio.Socket | None = None

    def connect(self) -> None:
        self._socket = _ctx.socket(zmq.SUB)
        self._socket.connect(self.endpoint)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")

    async def listen(self):
        """
        Yields validated model instances forever. Malformed messages
        are logged and skipped rather than crashing the stream — a
        single bad packet from a flaky serial link shouldn't take
        down the whole WebSocket broadcast loop.
        """
        if self._socket is None:
            self.connect()

        while True:
            raw = await self._socket.recv_json()
            try:
                yield self.model.model_validate(raw)
            except ValidationError as e:
                # TODO: real logging, plus a metrics counter — this is
                # exactly the failure mode flagged earlier in chat
                # (upstream sends a shape that doesn't match the model).
                print(f"[zmq_bridge] dropped malformed message from {self.endpoint}: {e}")
                continue

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()


# Concrete bridges for the two known upstream publishers. Ports are
# placeholders — match whatever Moore6mController / the PyVISA daemon
# actually bind to.
rotor_bridge = ZmqBridge("tcp://localhost:5556", RotorState)
spectrum_bridge = ZmqBridge("tcp://localhost:5557", SpectrumReading)

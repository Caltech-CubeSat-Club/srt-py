"""
FastAPI app entry point. This is the only server process — it serves
the compiled static Svelte build AND the WebSocket/API endpoints.
There is no separate Node server in this architecture.
"""

from fastapi import APIRouter, FastAPI
from contextlib import asynccontextmanager
from pathlib import Path

from .routes import auth, websocket
from .zmq_bridge.bridge import status_broadcaster

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Runs once before the app starts accepting requests.
    await status_broadcaster.start()
    yield
    # Runs once on shutdown, after the app stops accepting new requests.
    await status_broadcaster.stop()

app = FastAPI(title="SRT Dashboard", lifespan=lifespan)

auth_router = APIRouter()
auth_router.add_api_route("/auth/token", auth.login_for_token, methods=["GET"])
app.include_router(auth_router)

app.include_router(websocket.router)

app.frontend("/", directory=Path(__file__).parent.parent / "svelte-frontend" / "build", fallback="index.html")

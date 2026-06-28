"""
FastAPI app entry point. This is the only server process — it serves
the compiled static Svelte build AND the WebSocket/API endpoints.
There is no separate Node server in this architecture.

Run with: uvicorn app.main:app --host 0.0.0.0 --port 8000
"""

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from starlette.middleware.sessions import SessionMiddleware

from .routes import auth, websocket
from .zmq_bridge.bridge import status_broadcaster

app = FastAPI(title="SRT Dashboard")

app.add_middleware(SessionMiddleware, secret_key="REPLACE_ME")  # required by authlib's OAuth flow

app.include_router(auth.router)
app.include_router(websocket.router)

@app.on_event("startup")
async def _start_status_broadcaster():
    await status_broadcaster.start()
 
 
@app.on_event("shutdown")
async def _stop_status_broadcaster():
    await status_broadcaster.stop()

# Static Svelte build mounted LAST so it doesn't shadow /ws or /api
# routes. adapter-static outputs to frontend/build/ by default.
# NOTE: this directory won't exist until you actually run
# `npm run build` in frontend/ — FastAPI will raise on startup
# if it's missing. Fine during early backend-only development;
# comment out the mount or build the frontend first.
app.mount(
    "/",
    StaticFiles(directory="../svelte-frontend/build", html=True),
    name="frontend",
)

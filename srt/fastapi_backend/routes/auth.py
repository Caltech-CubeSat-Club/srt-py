"""
Simple username/password auth, replacing the Google OAuth scaffold for
now. Carries over the exact same config fields the existing Dash
dashboard already uses (DASHBOARD_REQUIRE_AUTH / DASHBOARD_USERNAME /
DASHBOARD_PASSWORD — confirmed from dashboard/app.py's generate_app()),
so config.yaml needs zero changes.

Two pieces:
  - HTTPBasic for normal HTTP requests (the browser's native login
    prompt — no custom login modal to build, unlike the old Dash
    version's hand-rolled dbc.Modal).
  - A short-lived JWT, issued after a successful Basic Auth check, for
    the WebSocket — browsers can't send Authorization headers on the
    WS handshake, so the token rides as a ?token=... query param, same
    pattern used in the Google-OAuth version of this file.

SECURITY NOTE: HTTP Basic sends the password on every single request,
base64-encoded (NOT encrypted) — this is fine ONLY over HTTPS. If this
server is reachable from the open internet without TLS in front of it
(e.g. a reverse proxy terminating HTTPS), the password is sent in the
clear on every page load. Don't deploy this past localhost/LAN testing
without HTTPS in front of it.
"""

import os
import secrets
import time

import jwt
from fastapi import Depends, HTTPException, Request, status
from fastapi.security import HTTPBasic, HTTPBasicCredentials

basic_auth = HTTPBasic()

JWT_SECRET = os.environ.get("JWT_SECRET", "dev-only-insecure-secret-change-me")
JWT_ALGORITHM = "HS256"
JWT_EXPIRY_SECONDS = 60 * 60 * 8  # 8 hour session


def _get_dashboard_credentials(request: Request) -> tuple[bool, str, str]:
    """Reads DASHBOARD_REQUIRE_AUTH / DASHBOARD_USERNAME / DASHBOARD_PASSWORD
    from the loaded DaemonConfig. Imported lazily inside the function
    (rather than at module level) to avoid a circular import between
    this module and wherever the global config singleton lives —
    adjust this to match however your app actually exposes the loaded
    config (e.g. a FastAPI dependency, an app.state.config, etc.)."""
    config = request.app.state.config

    return (
        bool(config.DASHBOARD_REQUIRE_AUTH),
        str(config.DASHBOARD_USERNAME),
        str(config.DASHBOARD_PASSWORD),
    )


def _check_credentials(request: Request, username: str, password: str) -> bool:
    require_auth, real_username, real_password = _get_dashboard_credentials(request)
    if not require_auth:
        return True
    # secrets.compare_digest instead of == — avoids a timing
    # side-channel that could let an attacker infer the password
    # character-by-character from response timing. Minor in practice
    # for a small research-instrument deployment, but free to do
    # correctly.
    username_ok = secrets.compare_digest(username, real_username)
    password_ok = secrets.compare_digest(password, real_password)
    return username_ok and password_ok


def require_auth(request: Request, credentials: HTTPBasicCredentials = Depends(basic_auth)) -> str:
    """FastAPI dependency for normal HTTP routes:
    Depends(require_auth). Returns the username on success."""
    if not _check_credentials(request, credentials.username, credentials.password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Basic"},
        )
    return credentials.username


def _issue_jwt(username: str) -> str:
    payload = {"sub": username, "exp": int(time.time()) + JWT_EXPIRY_SECONDS}
    return jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)


def login_for_token(request: Request, credentials: HTTPBasicCredentials = Depends(basic_auth)) -> dict:
    """Mount this as a GET/POST route (e.g. /auth/token) that the
    Svelte frontend calls once after the browser's native Basic Auth
    prompt succeeds, to get a JWT for the WebSocket. See main.py for
    wiring."""
    if not _check_credentials(request, credentials.username, credentials.password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Basic"},
        )
    return {"access_token": _issue_jwt(credentials.username), "token_type": "bearer"}


async def get_current_user_ws(token: str | None) -> str:
    """WebSocket variant — same shape as the Google-OAuth version of
    this function. Token arrives as a query param
    (wss://host/ws/status?token=...) since browsers can't set
    Authorization headers on the WS upgrade request."""
    if token is None:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Missing token")
    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
        return payload["sub"]
    except jwt.PyJWTError:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid or expired token")
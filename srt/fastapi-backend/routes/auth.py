"""
Auth: Google OAuth for identity, restricted to @caltech.edu accounts,
then our own short-lived JWT for everything afterward (HTTP routes
and WebSocket auth alike).

Google's token is only trusted at login time. Once we've verified
identity and domain, we mint our own JWT — the rest of the app never
talks to Google again per-request. This keeps the auth dependency
uniform regardless of login method, and avoids re-validating against
Google on every request.

Kept deliberately thin/illustrative:
- no refresh-token flow
- no persistent user table (stateless JWT only)
- secrets/config hardcoded as placeholders, not loaded from env
A real implementation would add all of the above plus rate limiting
on the login endpoint and proper secret management.
"""

import time

import jwt
from authlib.integrations.starlette_client import OAuth
from fastapi import APIRouter, Depends, HTTPException, Request, status
from fastapi.security import OAuth2PasswordBearer

router = APIRouter()

# --- Config placeholders — load from env/secrets manager in reality ---
GOOGLE_CLIENT_ID = "REPLACE_ME.apps.googleusercontent.com"
GOOGLE_CLIENT_SECRET = "REPLACE_ME"
JWT_SECRET = "REPLACE_ME_WITH_A_REAL_SECRET"
JWT_ALGORITHM = "HS256"
JWT_EXPIRY_SECONDS = 60 * 60 * 8  # 8 hour session
ALLOWED_HOSTED_DOMAIN = "caltech.edu"

oauth = OAuth()
oauth.register(
    name="google",
    client_id=GOOGLE_CLIENT_ID,
    client_secret=GOOGLE_CLIENT_SECRET,
    server_metadata_url="https://accounts.google.com/.well-known/openid-configuration",
    client_kwargs={"scope": "openid email profile"},
)

# Used only so FastAPI's dependency system can extract the Bearer
# token from the Authorization header on normal HTTP requests; the
# WebSocket case is handled separately (see get_current_user_ws below)
# since browsers can't set custom headers on the WS handshake.
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="auth/login", auto_error=False)


def _issue_jwt(email: str) -> str:
    payload = {
        "sub": email,
        "exp": int(time.time()) + JWT_EXPIRY_SECONDS,
    }
    return jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)


def _verify_jwt(token: str) -> str:
    """Returns the email (sub claim) if valid, raises otherwise."""
    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
        return payload["sub"]
    except jwt.PyJWTError:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid or expired token")


@router.get("/auth/login")
async def login(request: Request):
    redirect_uri = request.url_for("auth_callback")
    return await oauth.google.authorize_redirect(request, redirect_uri)


@router.get("/auth/callback", name="auth_callback")
async def auth_callback(request: Request):
    token = await oauth.google.authorize_access_token(request)
    userinfo = token.get("userinfo") or {}

    email = userinfo.get("email", "")
    hosted_domain = userinfo.get("hd")
    email_verified = userinfo.get("email_verified")

    # Belt-and-suspenders per earlier discussion: check both the `hd`
    # claim AND the email's actual domain suffix, since `hd` alone
    # isn't bulletproof.
    domain_ok = (
        hosted_domain == ALLOWED_HOSTED_DOMAIN
        and email
        and email.endswith(f"@{ALLOWED_HOSTED_DOMAIN}")
    )

    if not (email_verified and domain_ok):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail=f"Access restricted to verified @{ALLOWED_HOSTED_DOMAIN} accounts",
        )

    our_jwt = _issue_jwt(email)

    # Real implementation: redirect to the Svelte app with the token
    # (e.g. as a fragment) so client-side code can pick it up and
    # store it. Returning it directly here for scaffold simplicity.
    return {"access_token": our_jwt, "token_type": "bearer"}


# --- Dependencies for protecting routes ---

async def get_current_user(token: str = Depends(oauth2_scheme)) -> str:
    """Use as a dependency on normal HTTP routes: Depends(get_current_user)."""
    if token is None:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Not authenticated")
    return _verify_jwt(token)


async def get_current_user_ws(token: str | None) -> str:
    """
    WebSocket variant. Browsers can't set Authorization headers on the
    WS upgrade request, so the token arrives as a query param instead:
    wss://host/ws/telescope?token=...
    Call this explicitly inside the websocket route before accepting
    the connection / before entering the broadcast loop.
    """
    if token is None:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Missing token")
    return _verify_jwt(token)

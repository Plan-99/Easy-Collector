"""Google OAuth device-flow authentication via the home-next API.

Replaces the legacy serial-key flow (license_validator.py).

Storage layout (under APP_HOME):
    auth.json               { access_token, user: {...}, device: {...}, server_url }
    license.key (legacy)    deleted on first successful sign-in to avoid drift.

Flow:
    1. POST /api/device-auth/start  → { nonce, verificationUrl, expiresAt }
    2. Open verificationUrl in default browser; user signs in with Google + approves
    3. Poll GET /api/device-auth/poll?nonce=…  every 2 s until APPROVED / EXPIRED / REJECTED
    4. Save the access_token to auth.json — Bearer-auth all subsequent calls
"""
from __future__ import annotations

import json
import platform
import socket
import sys
import time
import webbrowser
from pathlib import Path
from typing import Any, Optional
import urllib.error
import urllib.request

from app_context import APP_HOME, load_config, save_config

# License-server URL is shared with the legacy validator key for backward
# compatibility — admins who previously overrode it via config keep their setting.
_CONFIG_KEY = "license_server_url"
_DEFAULT_URL = "https://easy-trainer-home.vercel.app"
# Vercel moved the canonical alias; the old host now 307s and breaks POST clients.
_LEGACY_HOSTS = ("easytrainerhome.vercel.app",)

AUTH_FILE = APP_HOME / "auth.json"
LEGACY_LICENSE_FILE = APP_HOME / "license.key"


def _canonicalize_url(url: str) -> str:
    out = url
    for legacy in _LEGACY_HOSTS:
        out = out.replace(legacy, "easy-trainer-home.vercel.app")
    return out


def _get_api_url() -> str:
    try:
        cfg = load_config()
        raw = (cfg.get(_CONFIG_KEY) or _DEFAULT_URL).rstrip("/")
    except Exception:
        return _DEFAULT_URL
    fixed = _canonicalize_url(raw)
    if fixed != raw:
        try:
            cfg[_CONFIG_KEY] = fixed
            save_config(cfg)
        except Exception:
            pass
    return fixed


# ---------------------------------------------------------------------------
# Machine fingerprint (kept identical to license_validator for migration parity)
# ---------------------------------------------------------------------------

def _read_stable_machine_id() -> str | None:
    import hashlib  # noqa: F401  (kept for parity)
    import subprocess
    for path in ("/etc/machine-id", "/var/lib/dbus/machine-id"):
        try:
            text = Path(path).read_text().strip()
            if text:
                return text
        except Exception:
            continue
    if sys.platform == "darwin":
        try:
            out = subprocess.check_output(
                ["ioreg", "-rd1", "-c", "IOPlatformExpertDevice"],
                text=True, timeout=5,
            )
            for line in out.splitlines():
                if "IOPlatformUUID" in line:
                    return line.split('"')[-2]
        except Exception:
            pass
    if sys.platform == "win32":
        try:
            import winreg
            key = winreg.OpenKey(
                winreg.HKEY_LOCAL_MACHINE,
                r"SOFTWARE\Microsoft\Cryptography",
                0,
                winreg.KEY_READ | winreg.KEY_WOW64_64KEY,
            )
            value, _ = winreg.QueryValueEx(key, "MachineGuid")
            winreg.CloseKey(key)
            return value
        except Exception:
            pass
    return None


def get_machine_fingerprint() -> str:
    import hashlib
    import uuid
    try:
        stable_id = _read_stable_machine_id()
        if stable_id:
            return hashlib.sha256(stable_id.encode()).hexdigest()[:32]
        mac = uuid.getnode()
        node_name = platform.node()
        raw = f"{mac}-{node_name}-{platform.machine()}"
        return hashlib.sha256(raw.encode()).hexdigest()[:32]
    except Exception:
        return "unknown"


def _machine_meta() -> dict:
    try:
        os_label = f"{platform.system()} {platform.release()}".strip()
    except Exception:
        os_label = sys.platform
    try:
        host = socket.gethostname()
    except Exception:
        host = ""
    return {
        "machineId": get_machine_fingerprint(),
        "hostname": host or None,
        "os": os_label or None,
    }


# ---------------------------------------------------------------------------
# Auth state persistence
# ---------------------------------------------------------------------------

def load_auth() -> dict | None:
    if not AUTH_FILE.exists():
        return None
    try:
        return json.loads(AUTH_FILE.read_text(encoding="utf-8"))
    except Exception:
        return None


def save_auth(state: dict) -> None:
    APP_HOME.mkdir(parents=True, exist_ok=True)
    AUTH_FILE.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
    # Restrict to user-only read since the access_token is a long-lived bearer.
    try:
        AUTH_FILE.chmod(0o600)
    except Exception:
        pass
    # Drop the legacy license file if it exists so we never accidentally fall
    # back to the deprecated serial-key path.
    try:
        if LEGACY_LICENSE_FILE.exists():
            LEGACY_LICENSE_FILE.unlink()
    except Exception:
        pass


def clear_auth() -> None:
    try:
        if AUTH_FILE.exists():
            AUTH_FILE.unlink()
    except Exception:
        pass


def get_access_token() -> str | None:
    s = load_auth()
    return s.get("access_token") if s else None


def get_signed_in_user() -> dict | None:
    s = load_auth()
    return s.get("user") if s else None


# ---------------------------------------------------------------------------
# HTTP helpers
# ---------------------------------------------------------------------------

class AuthError(Exception):
    """Raised when the saved token is invalid or revoked."""


def _request(
    method: str,
    path: str,
    *,
    body: dict | None = None,
    auth_required: bool = False,
    timeout: float = 15,
) -> tuple[int, dict]:
    url = f"{_get_api_url()}{path}"
    headers = {"Accept": "application/json"}
    data: bytes | None = None
    if body is not None:
        data = json.dumps(body).encode()
        headers["Content-Type"] = "application/json"
    if auth_required:
        token = get_access_token()
        if not token:
            raise AuthError("no token")
        headers["Authorization"] = f"Bearer {token}"
    req = urllib.request.Request(url, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            payload = resp.read().decode() or "{}"
            return resp.status, json.loads(payload)
    except urllib.error.HTTPError as e:
        try:
            payload = json.loads((e.read() or b"{}").decode())
        except Exception:
            payload = {"error": f"HTTP {e.code}"}
        return e.code, payload


def verify_token() -> Optional[dict]:
    """Hit /api/me with the saved Bearer. Return user dict on success, None on auth failure."""
    if not get_access_token():
        return None
    try:
        status, payload = _request("GET", "/api/me", auth_required=True)
    except AuthError:
        return None
    except Exception:
        # Network failure — assume the token is fine; caller can re-validate later.
        cached = get_signed_in_user()
        return cached
    if status == 200 and payload.get("user"):
        return payload["user"]
    if status in (401, 403):
        # Token was revoked (device unbound from web dashboard, etc.)
        clear_auth()
    return None


# ---------------------------------------------------------------------------
# Device-flow primitives (testable without Qt)
# ---------------------------------------------------------------------------

def start_device_auth() -> dict:
    """POST /api/device-auth/start → {nonce, verificationUrl, expiresAt, pollIntervalSec}."""
    status, payload = _request("POST", "/api/device-auth/start", body=_machine_meta())
    if status != 200:
        raise RuntimeError(f"start failed: HTTP {status}: {payload.get('error', payload)}")
    return payload


def poll_device_auth(nonce: str) -> dict:
    """GET /api/device-auth/poll?nonce=… → {status, …}."""
    from urllib.parse import quote
    status, payload = _request("GET", f"/api/device-auth/poll?nonce={quote(nonce)}")
    if status >= 500:
        raise RuntimeError(f"poll failed: HTTP {status}")
    return payload


# ---------------------------------------------------------------------------
# GUI entry point
# ---------------------------------------------------------------------------

def ensure_signed_in_gui() -> bool:
    """Block until the user is signed in, or the user cancels the dialog.

    On success, auth.json is written and the function returns True. On user
    cancellation (or unrecoverable error), returns False so the launcher can
    exit gracefully.
    """
    # Lazy import so non-GUI tests of this module don't pull in Qt.
    from app_context import (
        QApplication,
        QDialog,
        QLabel,
        QLineEdit,
        QMessageBox,
        QPushButton,
        QHBoxLayout,
        QVBoxLayout,
        QTimer,
        Qt,
    )

    # 1. Fast path — token already saved and still valid.
    user = verify_token()
    if user:
        return True

    # 2. Start a fresh device-auth request.
    try:
        start = start_device_auth()
    except Exception as e:
        QMessageBox.critical(None, "로그인 실패", f"인증 서버 연결에 실패했습니다.\n{e}")
        return False

    nonce = start["nonce"]
    verification_url = start["verificationUrl"]
    poll_every_s = max(1, int(start.get("pollIntervalSec", 2)))

    # Open the browser immediately.
    try:
        webbrowser.open(verification_url, new=2, autoraise=True)
    except Exception:
        pass

    # 3. Modal dialog while we poll.
    dlg = QDialog()
    dlg.setWindowTitle("Easy Trainer 로그인")
    dlg.setModal(True)
    dlg.resize(560, 260)

    layout = QVBoxLayout(dlg)
    title = QLabel("Google 계정으로 로그인을 진행해 주세요.")
    title.setStyleSheet("font-size: 15px; font-weight: 700;")
    layout.addWidget(title)

    desc = QLabel(
        "브라우저가 자동으로 열리지 않으면 아래 주소를 복사해 직접 여세요.\n"
        "로그인 후 \"이 기기 등록\" 버튼을 누르면 자동으로 진행됩니다."
    )
    desc.setStyleSheet("color: #aaaaaa;")
    desc.setWordWrap(True)
    layout.addWidget(desc)

    url_row = QHBoxLayout()
    url_edit = QLineEdit(verification_url)
    url_edit.setReadOnly(True)
    copy_btn = QPushButton("복사")
    copy_btn.setFixedWidth(60)
    url_row.addWidget(url_edit, 1)
    url_row.addWidget(copy_btn)
    layout.addLayout(url_row)

    def on_copy():
        try:
            QApplication.clipboard().setText(verification_url)
            copy_btn.setText("복사됨")
            QTimer.singleShot(1500, lambda: copy_btn.setText("복사"))
        except Exception:
            pass
    copy_btn.clicked.connect(on_copy)

    button_row = QHBoxLayout()
    open_btn = QPushButton("브라우저 다시 열기")
    cancel_btn = QPushButton("취소")
    button_row.addWidget(open_btn)
    button_row.addStretch(1)
    button_row.addWidget(cancel_btn)
    layout.addLayout(button_row)

    status_label = QLabel("브라우저에서 로그인을 기다리는 중…")
    status_label.setStyleSheet("color: #cfd8dc;")
    layout.addWidget(status_label)

    state: dict[str, Any] = {"result": None}

    def on_open():
        try:
            webbrowser.open(verification_url, new=2, autoraise=True)
        except Exception:
            pass

    def on_cancel():
        state["result"] = "cancelled"
        dlg.reject()

    open_btn.clicked.connect(on_open)
    cancel_btn.clicked.connect(on_cancel)

    deadline = time.monotonic() + 10 * 60  # match server-side 10-min TTL

    def tick():
        if time.monotonic() > deadline:
            state["result"] = "expired"
            dlg.reject()
            return
        try:
            payload = poll_device_auth(nonce)
        except Exception as e:
            status_label.setText(f"네트워크 오류 — 재시도 중… ({e})")
            return
        s = payload.get("status")
        if s == "PENDING":
            status_label.setText("브라우저에서 로그인을 기다리는 중…")
            return
        if s == "APPROVED":
            state["result"] = "approved"
            state["payload"] = payload
            dlg.accept()
            return
        if s == "REJECTED":
            err = payload.get("errorCode") or "REJECTED"
            state["result"] = ("rejected", err)
            dlg.reject()
            return
        if s == "EXPIRED":
            state["result"] = "expired"
            dlg.reject()
            return
        # Unknown — treat as transient.
        status_label.setText(f"알 수 없는 상태: {s} — 재시도 중…")

    timer = QTimer(dlg)
    timer.setInterval(poll_every_s * 1000)
    timer.timeout.connect(tick)
    # Run a tick immediately so the user sees status without waiting.
    QTimer.singleShot(200, tick)
    timer.start()

    dlg.exec()
    timer.stop()

    res = state["result"]
    if res == "approved":
        payload = state.get("payload", {})
        save_auth(
            {
                "access_token": payload["accessToken"],
                "user": payload["user"],
                "device": payload["device"],
                "server_url": _get_api_url(),
            }
        )
        return True

    if res == "expired":
        QMessageBox.warning(None, "시간 초과", "10분 안에 로그인을 마치지 못했습니다.\n다시 시도해 주세요.")
    elif isinstance(res, tuple) and res[0] == "rejected":
        code = res[1]
        msg = "로그인이 거절되었습니다."
        if code == "DEVICE_ALREADY_BOUND":
            msg = (
                "이미 사용중인 기기가 있습니다.\n"
                f"{_get_api_url()}/dashboard 에서 해제 후 다시 시도해 주세요."
            )
        elif code == "USER_REJECTED":
            msg = "사용자가 등록을 취소했습니다."
        QMessageBox.warning(None, "로그인 실패", msg)
    # cancelled → silent

    return False


def sign_out() -> None:
    """Forget the local credentials. Server-side device row remains until explicitly unbound."""
    clear_auth()

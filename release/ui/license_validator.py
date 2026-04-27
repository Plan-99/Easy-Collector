"""License validation via EasyTrainer home-next API."""
from __future__ import annotations

import json
import platform
import hashlib
import subprocess
import sys
import uuid
from pathlib import Path
import urllib.request
import urllib.error

from app_context import load_config, save_config

# API base URL — change to production domain when deployed
_CONFIG_KEY = "license_server_url"
_DEFAULT_URL = "https://easytrainerhome.vercel.app"


def _get_api_url() -> str:
    try:
        cfg = load_config()
        return cfg.get(_CONFIG_KEY, _DEFAULT_URL).rstrip("/")
    except Exception:
        return _DEFAULT_URL


def _read_stable_machine_id() -> str | None:
    """Read OS-provided stable machine ID. Survives reboot and network changes."""
    # Linux: /etc/machine-id (systemd) or /var/lib/dbus/machine-id
    for path in ("/etc/machine-id", "/var/lib/dbus/machine-id"):
        try:
            text = Path(path).read_text().strip()
            if text:
                return text
        except Exception:
            continue
    # macOS: IOPlatformUUID
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
    # Windows: Registry MachineGuid
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
    """Generate a stable machine-unique ID.

    Uses OS-provided stable ID (machine-id on Linux, IOPlatformUUID on macOS,
    MachineGuid on Windows). Falls back to MAC+hostname only if unavailable.
    """
    try:
        stable_id = _read_stable_machine_id()
        if stable_id:
            return hashlib.sha256(stable_id.encode()).hexdigest()[:32]
        # Fallback (less reliable — MAC may vary across network interfaces)
        mac = uuid.getnode()
        node_name = platform.node()
        raw = f"{mac}-{node_name}-{platform.machine()}"
        return hashlib.sha256(raw.encode()).hexdigest()[:32]
    except Exception:
        return "unknown"


def verify_license(key_string: str) -> bool:
    """Validate a serial key against the home-next API.

    Sends the key + machineId to the server.
    On first activation, the key is bound to this machine.
    """
    api_url = _get_api_url()
    url = f"{api_url}/api/serial-key/validate"
    machine_id = get_machine_fingerprint()

    payload = json.dumps({"key": key_string, "machineId": machine_id}).encode()
    req = urllib.request.Request(
        url,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read().decode())
            if data.get("valid"):
                plan = data.get("plan", "free")
                print(f"[Success] 인증 성공! (plan: {plan})")
                # Save plan info to config
                try:
                    cfg = load_config()
                    cfg["license_plan"] = plan
                    save_config(cfg)
                except Exception:
                    pass
                return True
            else:
                print(f"[Fail] 인증 실패: {data.get('error', 'unknown')}")
                return False
    except urllib.error.HTTPError as e:
        try:
            body = json.loads(e.read().decode())
            print(f"[Fail] 인증 실패 (HTTP {e.code}): {body.get('error', 'unknown')}")
        except Exception:
            print(f"[Fail] 인증 실패 (HTTP {e.code})")
        return False
    except Exception as e:
        print(f"[Error] 라이선스 서버 연결 실패: {e}")
        return False

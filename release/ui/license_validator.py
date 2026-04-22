"""License validation via EasyTrainer home-next API."""
from __future__ import annotations

import json
import platform
import hashlib
import uuid
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


def get_machine_fingerprint() -> str:
    """Generate a stable machine-unique ID."""
    try:
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

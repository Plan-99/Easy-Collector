"""Stable machine fingerprint derived from the host's machine-id.

The launcher mounts /etc/machine-id from the host into the backend container so
this returns the SAME hash as the launcher's license_validator.get_machine_fingerprint.
"""
from __future__ import annotations

import hashlib
import os
import platform
import uuid


def _read_stable_machine_id() -> str | None:
    for path in ("/etc/machine-id", "/var/lib/dbus/machine-id"):
        try:
            with open(path) as f:
                text = f.read().strip()
            if text:
                return text
        except Exception:
            continue
    return None


def get_machine_fingerprint() -> str:
    """Return a 32-char SHA256-derived machine ID.

    Matches release/ui/license_validator.get_machine_fingerprint when /etc/machine-id
    is bind-mounted from host.
    """
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


_cached: str | None = None


def machine_id() -> str:
    global _cached
    if _cached is None:
        _cached = get_machine_fingerprint()
    return _cached

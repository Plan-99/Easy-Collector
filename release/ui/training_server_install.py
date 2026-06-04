"""Training server helpers for the launcher.

Two modes:
  - **local**: training_server runs INSIDE the backend container as a sub-process,
    sharing torch/CUDA. No separate install. The launcher only shows status + logs.
  - **remote**: backend talks to a training_server on a different host. Nothing
    runs locally; the launcher only shows the configured URL.

The standalone training_server Dockerfile and CI release exist solely so a remote
GPU box can be provisioned by extracting the tar.gz there — that flow is *not*
driven from this launcher.
"""
from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import tarfile
import tempfile
from pathlib import Path

from modules import get_training_server_config, save_training_server_config, _gh_request, _get_repo
from i18n import t

BACKEND_CONTAINER = "easytrainer_backend"
DEFAULT_PORT = 5100

# Host-visible log file (entrypoint.sh writes here when EASYTRAINER_LOCAL_TRAINING=1).
DATA_ROOT = Path(os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer"))
LOG_FILE = DATA_ROOT / "logs" / "training_server.log"


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def get_configured_port() -> int:
    return int(get_training_server_config().get("port", DEFAULT_PORT))


def set_configured_port(port: int) -> None:
    ts = get_training_server_config()
    ts["port"] = int(port)
    save_training_server_config(ts)


def get_local_training_enabled() -> bool:
    """Whether the launcher should ask backend to host training_server in-process."""
    return get_training_server_config().get("local_training", False)


def set_local_training_enabled(enabled: bool) -> None:
    ts = get_training_server_config()
    ts["local_training"] = bool(enabled)
    save_training_server_config(ts)


# ---------------------------------------------------------------------------
# Status
# ---------------------------------------------------------------------------

def _project_root() -> Path:
    """Resolve the project root that contains docker-compose.yml + training_server/."""
    cfg = get_training_server_config()
    p = cfg.get("project_root")
    if p and Path(p, "docker-compose.yml").is_file():
        return Path(p)
    return Path("/opt/easytrainer/project")


def is_installed() -> bool:
    """학습 서버가 설치되어 있는가?

    Source files exist at <project_root>/backend/training_server/app.py and a
    backend docker-compose.yml is present (so the launcher can recreate the
    container).
    """
    pr = _project_root()
    if not (pr / "docker-compose.yml").is_file():
        return False
    return (pr / "backend" / "training_server" / "app.py").is_file()


def is_backend_running() -> bool:
    try:
        out = subprocess.check_output(
            ["docker", "ps", "--format", "{{.Names}}", "--filter", f"name={BACKEND_CONTAINER}"],
            text=True, timeout=10,
        ).strip()
        return out == BACKEND_CONTAINER
    except Exception:
        return False


def is_port_listening(port: int | None = None) -> bool:
    """Check whether something is bound to localhost:<port>."""
    port = port or get_configured_port()
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1):
            return True
    except OSError:
        return False


def is_running() -> bool:
    """학습 서버가 켜져 있는가? (port 5100 responds)"""
    return is_port_listening()


# ---------------------------------------------------------------------------
# Lifecycle (recreates the backend container with the local-training flag)
# ---------------------------------------------------------------------------

def _set_project_root(path: Path) -> None:
    ts = get_training_server_config()
    ts["project_root"] = str(path)
    save_training_server_config(ts)


def apply_local_training(enabled: bool, port: int | None = None,
                          project_root: Path | None = None) -> tuple[bool, str]:
    """Persist the flag and recreate the backend container so the entrypoint picks it up."""
    set_local_training_enabled(enabled)
    if port is not None:
        set_configured_port(port)
    pr = project_root or _project_root()
    if not (pr / "docker-compose.yml").is_file():
        return False, t("tsi.composeNotFound", path=pr)
    _set_project_root(pr)

    env = os.environ.copy()
    env["EASYTRAINER_LOCAL_TRAINING"] = "1" if enabled else "0"
    env["TRAINING_SERVER_PORT"] = str(port or get_configured_port())
    try:
        result = subprocess.run(
            ["docker", "compose", "up", "-d", "--force-recreate", "backend"],
            cwd=str(pr), env=env, timeout=120,
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            return False, t("tsi.composeUpFailed", error=result.stderr.strip() or result.stdout.strip())
    except Exception as e:
        return False, t("tsi.recreateFailed", error=e)
    return True, (t("tsi.localTrainingEnabled") if enabled else t("tsi.localTrainingDisabled"))


# ---------------------------------------------------------------------------
# Log streaming
# ---------------------------------------------------------------------------

_TAG_PREFIX = "training-server-v"


def fetch_latest_release() -> dict | None:
    """Return the most recent training-server release dict or None."""
    import urllib.request
    repo = _get_repo()
    url = f"https://api.github.com/repos/{repo}/releases?per_page=30"
    try:
        with urllib.request.urlopen(_gh_request(url), timeout=15) as resp:
            releases = json.loads(resp.read().decode())
    except Exception:
        return None
    for r in releases:
        if str(r.get("tag_name", "")).startswith(_TAG_PREFIX):
            return r
    return None


def download_from_release(on_log=None, on_progress=None) -> tuple[bool, str]:
    """Download backend/training_server/ source from Easy-Trainer-Modules release.

    Extracts to <project_root>/backend/training_server/. The tar.gz produced by CI
    contains a single top-level entry named `training_server`, which we re-root.
    """
    import urllib.request

    def _log(msg: str):
        if on_log:
            on_log(msg)

    release = fetch_latest_release()
    if not release:
        return False, t("tsi.releaseNotFound")

    asset_url = None
    asset_name = None
    for a in release.get("assets", []):
        name = a.get("name", "")
        if name.startswith("training-server-v") and name.endswith(".tar.gz"):
            asset_url = a.get("browser_download_url")
            asset_name = name
            break
    if not asset_url:
        return False, t("tsi.noAsset")

    pr = _project_root()
    target_dir = pr / "backend" / "training_server"
    target_dir.parent.mkdir(parents=True, exist_ok=True)

    _log(f"Downloading {asset_name}...")
    try:
        with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
            tmp_path = tmp.name
            with urllib.request.urlopen(_gh_request(asset_url, timeout=300), timeout=300) as resp:
                total = int(resp.headers.get("Content-Length", 0))
                downloaded = 0
                while True:
                    chunk = resp.read(8192)
                    if not chunk:
                        break
                    tmp.write(chunk)
                    downloaded += len(chunk)
                    if on_progress:
                        on_progress(downloaded, total)
    except Exception as e:
        return False, t("tsi.downloadFailed", error=e)

    if target_dir.is_dir():
        try:
            shutil.rmtree(target_dir)
        except PermissionError:
            try:
                subprocess.run(["sudo", "rm", "-rf", str(target_dir)], check=True, timeout=30)
            except Exception as e:
                return False, t("tsi.removeDirFailed", error=e)

    _log("Extracting...")
    try:
        with tarfile.open(tmp_path, "r:gz") as tar:
            members = []
            for m in tar.getmembers():
                if m.name == "training_server" or m.name.startswith("training_server/"):
                    rel = m.name[len("training_server"):].lstrip("/")
                    if not rel:
                        continue
                    m.name = rel
                    members.append(m)
            tar.extractall(target_dir, members=members)
    except Exception as e:
        return False, t("tsi.extractFailed", error=e)
    finally:
        try:
            os.unlink(tmp_path)
        except Exception:
            pass

    if not (target_dir / "app.py").is_file():
        return False, t("tsi.appPyMissing")
    return True, t("tsi.installComplete", name=asset_name)


def remove() -> tuple[bool, str]:
    """Stop the server (if running) and delete training_server source files.

    Persistent data (datasets/, checkpoints/) is *always* preserved under
    TRAINING_SERVER_DATA_DIR — only the source code is removed.
    Note: deb upgrades will restore the source files on next package update.
    """
    pr = _project_root()
    src_dir = pr / "backend" / "training_server"

    if is_running():
        ok, msg = apply_local_training(False, project_root=pr)
        if not ok:
            return False, t("tsi.stopFailedAbort", error=msg)

    if src_dir.is_dir():
        try:
            shutil.rmtree(src_dir)
        except PermissionError:
            try:
                subprocess.run(["sudo", "rm", "-rf", str(src_dir)],
                               check=True, timeout=30)
            except Exception as e:
                return False, t("tsi.srcRemoveFailed", error=e)
        except Exception as e:
            return False, t("tsi.srcRemoveFailed", error=e)

    set_local_training_enabled(False)
    return True, t("tsi.srcRemoveComplete")


def open_log_stream(tail: int = 200) -> subprocess.Popen | None:
    """Tail the host-visible training_server log file. Returns None if missing."""
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    if not LOG_FILE.exists():
        try:
            LOG_FILE.touch()
        except Exception:
            return None
    try:
        return subprocess.Popen(
            ["tail", "-F", "-n", str(int(tail)), str(LOG_FILE)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
    except Exception:
        return None

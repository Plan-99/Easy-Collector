"""Training server install / start / stop / log helpers.

Downloads the `training-server-vX.Y.Z` release tar.gz from
Plan-99/Easy-Trainer-Modules and unpacks it into /opt/easytrainer_server/.
The launcher uses these helpers to manage the lifecycle.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import tarfile
import tempfile
from pathlib import Path

from modules import _gh_request, _get_repo, get_training_server_config, save_training_server_config


INSTALL_DIR = Path("/opt/easytrainer_server")
COMPOSE_FILE = INSTALL_DIR / "docker-compose.yml"
ENV_FILE = INSTALL_DIR / ".env"
DATA_DIR = INSTALL_DIR / "data"
VERSION_FILE = INSTALL_DIR / "version.json"
CONTAINER_NAME = "easytrainer_training_server"
DEFAULT_PORT = 5100


# ---------------------------------------------------------------------------
# Filesystem / install state
# ---------------------------------------------------------------------------

def _ensure_install_dir() -> Path:
    """Create /opt/easytrainer_server (escalating to sudo if /opt isn't writable)."""
    if not INSTALL_DIR.exists():
        try:
            INSTALL_DIR.mkdir(parents=True, exist_ok=True)
        except PermissionError:
            subprocess.run(
                ["sudo", "mkdir", "-p", str(INSTALL_DIR)],
                check=True, timeout=10,
            )
    if INSTALL_DIR.exists() and INSTALL_DIR.stat().st_uid == 0 and os.getuid() != 0:
        try:
            subprocess.run(
                ["sudo", "chown", "-R", f"{os.getuid()}:{os.getgid()}", str(INSTALL_DIR)],
                check=False, timeout=10,
            )
        except Exception:
            pass
    return INSTALL_DIR


def is_installed() -> bool:
    return COMPOSE_FILE.is_file()


def get_installed_version() -> str | None:
    if not VERSION_FILE.is_file():
        return None
    try:
        return json.loads(VERSION_FILE.read_text()).get("version")
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Release fetching
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


def get_remote_version(release: dict | None = None) -> str | None:
    release = release or fetch_latest_release()
    if not release:
        return None
    tag = release.get("tag_name", "")
    if tag.startswith(_TAG_PREFIX):
        return tag[len(_TAG_PREFIX):]
    return None


# ---------------------------------------------------------------------------
# Install / uninstall
# ---------------------------------------------------------------------------

def download_and_install(on_progress=None, on_log=None) -> tuple[bool, str]:
    """Download tar.gz, extract into /opt/easytrainer_server/, and `docker compose build`.

    Returns (success, message).
    """
    import urllib.request

    def _log(msg: str):
        if on_log:
            on_log(msg)

    release = fetch_latest_release()
    if not release:
        return False, "No training-server release found in Easy-Trainer-Modules."

    tag = release.get("tag_name", "")
    asset_url = None
    asset_name = None
    for asset in release.get("assets", []):
        name = asset.get("name", "")
        if name.startswith("training-server-v") and name.endswith(".tar.gz"):
            asset_url = asset.get("browser_download_url")
            asset_name = name
            break
    if not asset_url:
        return False, f"Release {tag} has no training-server tar.gz asset."

    _ensure_install_dir()

    # If a previous install exists, preserve data/ and replace the rest.
    preserve_data = DATA_DIR.is_dir()

    _log(f"Downloading {asset_name} from {tag}...")
    try:
        with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
            tmp_path = tmp.name
            req = _gh_request(asset_url, timeout=300)
            with urllib.request.urlopen(req, timeout=300) as resp:
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
        return False, f"Download failed: {e}"

    # Wipe old install (except data/), then extract fresh.
    _log("Extracting...")
    try:
        for entry in INSTALL_DIR.iterdir():
            if preserve_data and entry.name == "data":
                continue
            if entry.is_dir():
                shutil.rmtree(entry, ignore_errors=True)
            else:
                try:
                    entry.unlink()
                except Exception:
                    pass

        with tarfile.open(tmp_path, "r:gz") as tar:
            # Tar root is `training_server/` — strip that prefix.
            members = []
            for m in tar.getmembers():
                if m.name == "training_server" or m.name.startswith("training_server/"):
                    rel = m.name[len("training_server"):].lstrip("/")
                    if not rel:
                        continue
                    m.name = rel
                    members.append(m)
            tar.extractall(INSTALL_DIR, members=members)
    except Exception as e:
        return False, f"Extraction failed: {e}"
    finally:
        try:
            os.unlink(tmp_path)
        except Exception:
            pass

    if not COMPOSE_FILE.is_file():
        return False, "docker-compose.yml not found in extracted archive."

    DATA_DIR.mkdir(parents=True, exist_ok=True)

    # Persist version + installed flag in launcher config.
    ts = get_training_server_config()
    ts["installed"] = True
    ts["version"] = get_remote_version(release)
    ts["install_dir"] = str(INSTALL_DIR)
    save_training_server_config(ts)

    # Build the docker image.
    _log("Building Docker image (first build can take several minutes)...")
    try:
        proc = subprocess.Popen(
            ["docker", "compose", "-f", str(COMPOSE_FILE), "build"],
            cwd=str(INSTALL_DIR),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        if on_log:
            for line in proc.stdout:
                on_log(line.rstrip())
        rc = proc.wait()
        if rc != 0:
            return False, f"docker compose build failed (exit {rc})."
    except Exception as e:
        return False, f"Build failed: {e}"

    return True, f"Installed {asset_name}."


def uninstall(remove_data: bool = False) -> tuple[bool, str]:
    """Stop the container, remove the image, and optionally wipe data."""
    if not is_installed():
        return True, "Already uninstalled."
    try:
        subprocess.run(
            ["docker", "compose", "-f", str(COMPOSE_FILE), "down", "--rmi", "all"],
            cwd=str(INSTALL_DIR), timeout=120,
        )
    except Exception:
        pass

    # Hard-remove the container if it still exists for any reason.
    try:
        subprocess.run(["docker", "rm", "-f", CONTAINER_NAME], timeout=30,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass

    if INSTALL_DIR.exists():
        try:
            for entry in INSTALL_DIR.iterdir():
                if not remove_data and entry.name == "data":
                    continue
                if entry.is_dir():
                    shutil.rmtree(entry, ignore_errors=True)
                else:
                    try:
                        entry.unlink()
                    except Exception:
                        pass
            if remove_data and INSTALL_DIR.exists():
                shutil.rmtree(INSTALL_DIR, ignore_errors=True)
        except Exception as e:
            return False, f"Failed to clean install dir: {e}"

    ts = get_training_server_config()
    ts["installed"] = False
    ts.pop("version", None)
    save_training_server_config(ts)
    return True, "Uninstalled."


# ---------------------------------------------------------------------------
# Start / stop / status
# ---------------------------------------------------------------------------

def get_running_container() -> str | None:
    try:
        out = subprocess.check_output(
            ["docker", "ps", "--format", "{{.Names}}", "--filter", f"name={CONTAINER_NAME}"],
            text=True, timeout=10,
        ).strip()
        return out or None
    except Exception:
        return None


def is_running() -> bool:
    return get_running_container() is not None


def remove_existing_container() -> None:
    """Force-remove any existing trainer container (running or stopped)."""
    try:
        subprocess.run(
            ["docker", "rm", "-f", CONTAINER_NAME],
            timeout=30, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


def get_configured_port() -> int:
    return int(get_training_server_config().get("port", DEFAULT_PORT))


def set_configured_port(port: int) -> None:
    ts = get_training_server_config()
    ts["port"] = int(port)
    save_training_server_config(ts)


def _write_env_file(port: int) -> None:
    ENV_FILE.write_text(f"TRAINING_SERVER_PORT={int(port)}\n")


def start(port: int | None = None, force: bool = True) -> tuple[bool, str]:
    """Start the trainer container on the given port. Force-replaces an existing one."""
    if not is_installed():
        return False, "Training server is not installed."

    port = int(port or get_configured_port())
    set_configured_port(port)
    _write_env_file(port)

    if force:
        remove_existing_container()

    try:
        result = subprocess.run(
            ["docker", "compose", "-f", str(COMPOSE_FILE), "up", "-d"],
            cwd=str(INSTALL_DIR), timeout=120,
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            return False, f"docker compose up failed: {result.stderr.strip() or result.stdout.strip()}"
    except Exception as e:
        return False, f"Start failed: {e}"

    return True, f"Started on port {port}."


def stop() -> tuple[bool, str]:
    if not is_installed():
        return True, "Not installed."
    try:
        result = subprocess.run(
            ["docker", "compose", "-f", str(COMPOSE_FILE), "stop"],
            cwd=str(INSTALL_DIR), timeout=60,
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            return False, f"docker compose stop failed: {result.stderr.strip()}"
    except Exception as e:
        return False, f"Stop failed: {e}"
    return True, "Stopped."


# ---------------------------------------------------------------------------
# Log streaming
# ---------------------------------------------------------------------------

def open_log_stream(tail: int = 200) -> subprocess.Popen | None:
    """Spawn `docker logs -f --tail N` for the trainer container.

    Returns the Popen so the caller can read line-by-line and terminate on close.
    """
    try:
        return subprocess.Popen(
            ["docker", "logs", "-f", "--tail", str(int(tail)), CONTAINER_NAME],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
    except Exception:
        return None

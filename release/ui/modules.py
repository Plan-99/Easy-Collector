"""Module registry and management for EasyTrainer modular architecture."""
from __future__ import annotations

import json
import os
import shutil
import tarfile
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from app_context import load_config, save_config

# ---------------------------------------------------------------------------
# Module metadata
# ---------------------------------------------------------------------------

@dataclass
class ModuleInfo:
    id: str
    name: str
    category: str  # "core" | "feature" | "extension" | "robot"
    description: str
    required: bool = False
    default: bool = False  # checked by default in installer
    asset_name: str = ""   # GitHub Release asset pattern e.g. "module-training-{version}.tar.gz"
    dependencies: list[str] = field(default_factory=list)


MODULE_REGISTRY: list[ModuleInfo] = [
    # ── Core (always installed, hidden in UI) ──
    ModuleInfo(id="core", name="메인 프레임워크", category="core", description="Flask API, Frontend, DB 등 기본 시스템", required=True, default=True),
    ModuleInfo(id="import", name="로봇/센서 Import", category="feature", description="로봇·센서 불러오기", required=True, default=True),
    ModuleInfo(id="controller", name="Easy Controller", category="feature", description="텔레오퍼레이션 제어", required=True, default=True),
    ModuleInfo(id="training", name="학습 (Training)", category="feature", description="모방학습 정책 훈련", required=True, default=True),
    ModuleInfo(id="inference", name="추론 (Inference)", category="feature", description="학습된 정책 실행", required=True, default=True),
    # ── Robot drivers (from GitHub: Plan-99/Easy-Trainer-Modules) ──
    ModuleInfo(id="robot_piper", name="Piper", category="robot", description="Agilex Piper 로봇", asset_name="module-robot_piper-{version}.tar.gz"),
    ModuleInfo(id="robot_dynamixel", name="Dynamixel", category="robot", description="Robotis 서보 모터", asset_name="module-robot_dynamixel-{version}.tar.gz"),
    ModuleInfo(id="robot_unitree", name="Unitree", category="robot", description="Unitree 로봇", asset_name="module-robot_unitree-{version}.tar.gz"),
    ModuleInfo(id="robot_jaka", name="Jaka", category="robot", description="Jaka 협동로봇", asset_name="module-robot_jaka-{version}.tar.gz"),
    ModuleInfo(id="robot_fairino", name="Fairino", category="robot", description="Fairino 협동로봇", asset_name="module-robot_fairino-{version}.tar.gz"),
    ModuleInfo(id="robot_rbpodo", name="RBPodo", category="robot", description="Rainbow Robotics RBPodo", asset_name="module-robot_rbpodo-{version}.tar.gz"),
    ModuleInfo(id="robot_kinova", name="Kinova Kortex", category="robot", description="Kinova Kortex 로봇", asset_name="module-robot_kinova-{version}.tar.gz"),
    ModuleInfo(id="robot_techman", name="Techman TM", category="robot", description="Techman TM 로봇", asset_name="module-robot_techman-{version}.tar.gz"),
    ModuleInfo(id="gripper_generic", name="Generic Gripper", category="robot", description="범용 그리퍼 인터페이스", asset_name="module-gripper_generic-{version}.tar.gz"),
    ModuleInfo(id="gripper_robotiq", name="Robotiq Gripper", category="robot", description="Robotiq 그리퍼", asset_name="module-gripper_robotiq-{version}.tar.gz"),
    ModuleInfo(id="gripper_onrobot", name="OnRobot Gripper", category="robot", description="OnRobot 그리퍼", asset_name="module-gripper_onrobot-{version}.tar.gz"),
    ModuleInfo(id="custom_interfaces", name="Custom Interfaces", category="robot", description="EasyTrainer 커스텀 ROS 메시지", asset_name="module-custom_interfaces-{version}.tar.gz"),
    ModuleInfo(id="serial_comm", name="Serial Communication", category="robot", description="시리얼 통신 유틸리티", asset_name="module-serial_comm-{version}.tar.gz"),
    # ── Sensors ──
    ModuleInfo(id="sensor_webcam", name="Webcam", category="sensor", description="USB 웹캠 (Logitech 등)", asset_name="module-sensor_webcam-{version}.tar.gz"),
    # ── Extensions ──
    ModuleInfo(id="vr_teleop", name="VR 텔레오퍼레이션", category="extension", description="VR 기반 원격 조종", asset_name="module-vr_teleop-{version}.tar.gz"),
    ModuleInfo(id="test_arm", name="Test Arm", category="extension", description="테스트 로봇 암 시뮬레이션", asset_name="module-test_arm-{version}.tar.gz"),
]

# Lookup helpers
_REGISTRY_MAP: dict[str, ModuleInfo] = {m.id: m for m in MODULE_REGISTRY}

CATEGORY_LABELS = {
    "core": "코어 기능 (필수)",
    "feature": "기능 모듈",
    "robot": "로봇",
    "sensor": "센서",
    "extension": "확장 기능",
}
# UI visible categories (core/feature are hidden)
VISIBLE_CATEGORIES = ["robot", "sensor", "extension"]
CATEGORY_ORDER = ["core", "feature", "robot", "sensor", "extension"]


def get_module(module_id: str) -> Optional[ModuleInfo]:
    return _REGISTRY_MAP.get(module_id)


def modules_by_category() -> dict[str, list[ModuleInfo]]:
    result: dict[str, list[ModuleInfo]] = {}
    for m in MODULE_REGISTRY:
        result.setdefault(m.category, []).append(m)
    return result


# ---------------------------------------------------------------------------
# Installed modules persistence (config.json)
# ---------------------------------------------------------------------------

_CONFIG_KEY = "installed_modules"
_MODULES_DIR_NAME = "modules"


def _modules_dir() -> Path:
    data_dir = os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer")
    return Path(data_dir) / _MODULES_DIR_NAME


_VERSIONS_KEY = "installed_module_versions"


def get_installed_modules() -> list[str]:
    """Return list of installed module IDs from config."""
    try:
        cfg = load_config()
        return list(cfg.get(_CONFIG_KEY, []))
    except Exception:
        return []


def get_installed_versions() -> dict[str, str]:
    """Return {module_id: version} for installed modules."""
    try:
        cfg = load_config()
        return dict(cfg.get(_VERSIONS_KEY, {}))
    except Exception:
        return {}


def get_installed_version(module_id: str) -> str | None:
    return get_installed_versions().get(module_id)


def save_installed_modules(module_ids: list[str]) -> None:
    """Persist installed module IDs to config."""
    try:
        cfg = load_config()
    except Exception:
        cfg = {}
    cfg[_CONFIG_KEY] = sorted(set(module_ids))
    save_config(cfg)


def _save_module_version(module_id: str, version: str | None) -> None:
    try:
        cfg = load_config()
    except Exception:
        cfg = {}
    versions = cfg.get(_VERSIONS_KEY, {})
    if version:
        versions[module_id] = version
    else:
        versions.pop(module_id, None)
    cfg[_VERSIONS_KEY] = versions
    save_config(cfg)


def is_module_installed(module_id: str) -> bool:
    return module_id in get_installed_modules()


def set_module_installed(module_id: str, installed: bool = True, version: str | None = None) -> None:
    modules = get_installed_modules()
    if installed and module_id not in modules:
        modules.append(module_id)
    elif not installed and module_id in modules:
        modules.remove(module_id)
    save_installed_modules(modules)
    if installed and version:
        _save_module_version(module_id, version)
    elif not installed:
        _save_module_version(module_id, None)


# ---------------------------------------------------------------------------
# GitHub Releases download
# ---------------------------------------------------------------------------

_DEFAULT_REPO = "Plan-99/Easy-Trainer-Modules"
_CONFIG_REPO_KEY = "module_repo"


def _get_repo() -> str:
    try:
        cfg = load_config()
        return cfg.get(_CONFIG_REPO_KEY, _DEFAULT_REPO)
    except Exception:
        return _DEFAULT_REPO


def fetch_latest_release(repo: str | None = None) -> dict | None:
    """Fetch latest release info from GitHub API. Returns JSON dict or None."""
    import urllib.request
    import urllib.error

    repo = repo or _get_repo()
    url = f"https://api.github.com/repos/{repo}/releases/latest"
    req = urllib.request.Request(url, headers={"Accept": "application/vnd.github+json"})
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return json.loads(resp.read().decode())
    except Exception:
        return None


def get_remote_versions(release: dict | None = None) -> dict[str, str]:
    """Parse asset names from a release to get {module_id: version}.

    Asset format: module-{id}-{version}.tar.gz
    """
    if release is None:
        release = fetch_latest_release()
    if not release:
        return {}
    result: dict[str, str] = {}
    for asset in release.get("assets", []):
        name = asset.get("name", "")
        if name.startswith("module-") and name.endswith(".tar.gz"):
            # module-robot_piper-1.1.0.tar.gz → id=robot_piper, ver=1.1.0
            stripped = name[len("module-"):-len(".tar.gz")]
            # Find last dash followed by version (digit)
            parts = stripped.rsplit("-", 1)
            if len(parts) == 2 and parts[1] and parts[1][0].isdigit():
                result[parts[0]] = parts[1]
    return result


def _get_project_root() -> Path:
    """Return the project root (where docker-compose.yml lives)."""
    data_dir = os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer")
    project = Path(data_dir) / "project"
    if project.exists():
        return project
    return Path.cwd()


def _get_install_dir(mod: ModuleInfo) -> Path:
    """Return the directory where a module should be installed."""
    project = _get_project_root()
    # Read module.json from downloaded archive to get install_path
    # Default: ros2 modules → ros2/ros2_ws/src/, extensions → backend/extensions/
    if mod.category in ("robot", "sensor"):
        return project / "ros2" / "ros2_ws" / "src"
    else:
        return project / "backend" / "modules"


def download_module(
    module_id: str,
    release: dict | None = None,
    version: str = "latest",
    on_progress: callable | None = None,
) -> bool:
    """Download and install a module from GitHub Release assets.

    For robot/sensor modules: extracts to ros2/ros2_ws/src/
    For extension modules: extracts to backend/extensions/
    Then reads module.json and installs dependencies.
    """
    import urllib.request
    import urllib.error
    import subprocess as _sp

    mod = get_module(module_id)
    if not mod:
        return False

    if release is None:
        release = fetch_latest_release()
    if not release:
        return False

    # Find the asset matching this module ID (any version)
    # Asset naming: module-{id}-{version}.tar.gz
    prefix = f"module-{module_id}-"
    asset_url = None
    remote_version = None
    for asset in release.get("assets", []):
        name = asset.get("name", "")
        if name.startswith(prefix) and name.endswith(".tar.gz"):
            asset_url = asset.get("browser_download_url")
            remote_version = name[len(prefix):-len(".tar.gz")]
            break

    if not asset_url:
        return False

    install_dir = _get_install_dir(mod)
    install_dir.mkdir(parents=True, exist_ok=True)

    try:
        # Download
        with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
            tmp_path = tmp.name
            req = urllib.request.Request(asset_url)
            with urllib.request.urlopen(req, timeout=120) as resp:
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

        # Extract to install dir
        with tarfile.open(tmp_path, "r:gz") as tar:
            tar.extractall(path=str(install_dir))

        # Read module.json from extracted folder and install dependencies
        _install_module_deps(install_dir, module_id)

        set_module_installed(module_id, True, version=remote_version)
        return True
    except Exception:
        return False
    finally:
        try:
            os.unlink(tmp_path)
        except Exception:
            pass


def install_modules_batch(
    module_ids: list[str],
    on_module_start: callable | None = None,
    on_module_done: callable | None = None,
    on_progress: callable | None = None,
) -> dict[str, bool]:
    """Download and install multiple modules.

    Reusable from both installer wizard and launcher module manager.

    Args:
        module_ids: List of module IDs to install.
        on_module_start: Callback(module_id, index, total) called before each module.
        on_module_done: Callback(module_id, success, index, total) called after each.
        on_progress: Callback(bytes_downloaded, total_bytes) for download progress.

    Returns:
        Dict of {module_id: success_bool}.
    """
    release = fetch_latest_release()
    results: dict[str, bool] = {}
    total = len(module_ids)

    for i, mid in enumerate(module_ids):
        if on_module_start:
            on_module_start(mid, i, total)
        ok = download_module(mid, release=release, on_progress=on_progress)
        results[mid] = ok
        if on_module_done:
            on_module_done(mid, ok, i, total)

    return results


def _install_module_deps(install_dir: Path, module_id: str) -> None:
    """Install dependencies and run install script from module.json."""
    import subprocess as _sp

    # Find module.json in any subfolder
    for mj in install_dir.rglob("module.json"):
        try:
            meta = json.loads(mj.read_text())
            if meta.get("id") != module_id:
                continue
        except Exception:
            continue

        module_dir = mj.parent
        deps = meta.get("dependencies", {})

        # apt dependencies (best-effort, may need sudo)
        apt_deps = deps.get("apt", [])
        if apt_deps:
            try:
                _sp.run(
                    ["sudo", "apt-get", "install", "-y", "--no-install-recommends"] + apt_deps,
                    timeout=300, check=False,
                )
            except Exception:
                pass

        # pip dependencies
        pip_deps = deps.get("pip", [])
        if pip_deps:
            try:
                _sp.run(
                    ["python3", "-m", "pip", "install", "--quiet"] + pip_deps,
                    timeout=120, check=False,
                )
            except Exception:
                pass

        # Custom install script (for C++ builds, etc.)
        install_script = meta.get("install_script")
        if install_script:
            script_path = module_dir / install_script
            if script_path.exists():
                try:
                    _sp.run(
                        ["bash", str(script_path)],
                        cwd=str(module_dir),
                        timeout=600, check=False,
                    )
                except Exception:
                    pass
        break


def remove_module(module_id: str) -> bool:
    """Remove an installed module."""
    mod = get_module(module_id)
    if not mod or mod.required:
        return False

    # Remove from install dir
    install_dir = _get_install_dir(mod)
    # Find the actual folder name by checking module.json
    for mj in install_dir.rglob("module.json"):
        try:
            meta = json.loads(mj.read_text())
            if meta.get("id") == module_id:
                pkg_dir = mj.parent
                if pkg_dir.exists() and pkg_dir != install_dir:
                    shutil.rmtree(pkg_dir, ignore_errors=True)
                break
        except Exception:
            continue

    # Also clean from legacy modules dir
    legacy_dir = _modules_dir() / module_id
    if legacy_dir.exists():
        shutil.rmtree(legacy_dir, ignore_errors=True)

    set_module_installed(module_id, False)
    return True


# ---------------------------------------------------------------------------
# GPU VRAM detection
# ---------------------------------------------------------------------------

@dataclass
class GpuInfo:
    index: int
    name: str
    vram_total_mb: int
    vram_used_mb: int
    vram_free_mb: int


def detect_gpus() -> list[GpuInfo]:
    """Detect NVIDIA GPUs and their VRAM via nvidia-smi."""
    import subprocess as _sp
    try:
        out = _sp.check_output([
            "nvidia-smi",
            "--query-gpu=index,name,memory.total,memory.used,memory.free",
            "--format=csv,noheader,nounits",
        ], text=True, timeout=10)
        gpus = []
        for line in out.strip().splitlines():
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 5:
                gpus.append(GpuInfo(
                    index=int(parts[0]),
                    name=parts[1],
                    vram_total_mb=int(parts[2]),
                    vram_used_mb=int(parts[3]),
                    vram_free_mb=int(parts[4]),
                ))
        return gpus
    except Exception:
        return []


# ---------------------------------------------------------------------------
# Training server management
# ---------------------------------------------------------------------------

_TRAINING_SERVER_CONFIG_KEY = "training_server"


def get_training_server_config() -> dict:
    """Get training server config from config.json."""
    try:
        cfg = load_config()
        return cfg.get(_TRAINING_SERVER_CONFIG_KEY, {})
    except Exception:
        return {}


def save_training_server_config(ts_cfg: dict) -> None:
    try:
        cfg = load_config()
    except Exception:
        cfg = {}
    cfg[_TRAINING_SERVER_CONFIG_KEY] = ts_cfg
    save_config(cfg)


def is_training_server_installed() -> bool:
    return get_training_server_config().get("installed", False)


def set_training_server_installed(installed: bool) -> None:
    ts = get_training_server_config()
    ts["installed"] = installed
    save_training_server_config(ts)


def get_training_mode() -> str:
    """Return 'local' or 'remote'."""
    return get_training_server_config().get("mode", "remote")


def set_training_mode(mode: str) -> None:
    ts = get_training_server_config()
    ts["mode"] = mode
    save_training_server_config(ts)

"""Module registry and management for EasyTrainer modular architecture."""
from __future__ import annotations

import json
import os
import shutil
import tarfile
import tempfile
import threading
import urllib.error
import urllib.request
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


# Hardcoded fallback used when the launcher can't reach home-next AND no cache
# exists yet. The home-next /api/modules response is the source of truth at
# runtime — see _refresh_registry_from_remote(). Keep this list in sync with
# home-next/prisma/seed-modules.sql so a fresh install works offline.
_FALLBACK_REGISTRY: list[ModuleInfo] = [
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
    ModuleInfo(id="sensor_realsense", name="Intel RealSense", category="sensor", description="Intel RealSense 깊이 카메라 (D435, D405 등)", asset_name="module-sensor_realsense-{version}.tar.gz"),
    # ── Extensions ──
    ModuleInfo(id="vr_teleop", name="VR 텔레오퍼레이션", category="extension", description="VR 기반 원격 조종", asset_name="module-vr_teleop-{version}.tar.gz"),
    ModuleInfo(id="test_arm", name="Test Arm", category="extension", description="테스트 로봇 암 시뮬레이션", asset_name="module-test_arm-{version}.tar.gz"),
    ModuleInfo(id="sim_isaaclab", name="Isaac Lab 시뮬레이션", category="extension", description="NVIDIA Isaac Lab + cuRobo (GPU 필요, NGC 로그인 필요)", asset_name="module-sim_isaaclab-{version}.tar.gz"),
]

# Mutable runtime registry, swapped in-place on remote refresh. Importers
# (`from modules import MODULE_REGISTRY`) keep their reference valid because
# we mutate the list, not rebind it.
MODULE_REGISTRY: list[ModuleInfo] = list(_FALLBACK_REGISTRY)
_REGISTRY_MAP: dict[str, ModuleInfo] = {m.id: m for m in MODULE_REGISTRY}


def _apply_registry(items: list["ModuleInfo"]) -> None:
    """Replace runtime registry contents in place."""
    MODULE_REGISTRY[:] = items
    _REGISTRY_MAP.clear()
    _REGISTRY_MAP.update({m.id: m for m in items})

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
    """Return /opt/easytrainer/project/modules/ — the single source of truth for installed modules."""
    data_dir = os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer")
    return Path(data_dir) / "project" / _MODULES_DIR_NAME


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


_MODULE_DEPS_KEY = "installed_module_deps"


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


def _ensure_modules_dir() -> Path:
    """Ensure project/modules/ dir exists with correct permissions."""
    import subprocess as _sp
    d = _modules_dir()
    if not d.exists():
        try:
            d.mkdir(parents=True, exist_ok=True)
        except PermissionError:
            _sp.run(["sudo", "mkdir", "-p", str(d)], check=True, timeout=5)
    # Fix ownership if owned by root
    if d.exists() and d.stat().st_uid == 0 and os.getuid() != 0:
        try:
            _sp.run(["sudo", "chown", "-R", f"{os.getuid()}:{os.getgid()}", str(d)],
                     check=False, timeout=5)
        except Exception:
            pass
    return d


def _save_module_manifest(module_id: str, meta: dict) -> None:
    """Save module.json as {module_id}.json in project/modules/ dir."""
    d = _ensure_modules_dir()
    (d / f"{module_id}.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))


def _remove_module_manifest(module_id: str) -> None:
    """Remove {module_id}.json from project/modules/ dir."""
    p = _modules_dir() / f"{module_id}.json"
    if p.exists():
        p.unlink()


def _load_module_manifest(module_id: str) -> dict | None:
    """Load {module_id}.json from project/modules/ dir."""
    p = _modules_dir() / f"{module_id}.json"
    if p.is_file():
        try:
            return json.loads(p.read_text())
        except Exception:
            pass
    return None


def get_all_installed_manifests() -> dict[str, dict]:
    """Return {module_id: meta} for all installed modules from project/modules/."""
    d = _modules_dir()
    result = {}
    if not d.is_dir():
        return result
    for f in d.glob("*.json"):
        try:
            meta = json.loads(f.read_text())
            mid = meta.get("id", f.stem)
            result[mid] = meta
        except Exception:
            pass
    return result



def is_module_installed(module_id: str) -> bool:
    """Check if a module is installed by looking for its manifest in project/modules/."""
    # core/feature modules are always installed
    mod = get_module(module_id)
    if mod and mod.required:
        return True

    # Single source of truth: project/modules/{module_id}.json
    return _load_module_manifest(module_id) is not None


def set_module_installed(module_id: str, installed: bool = True, version: str | None = None, deps: dict | None = None) -> None:
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
    # 의존성 정보도 config에 저장 (ros2 컨테이너에서 참조)
    if deps is not None or not installed:
        try:
            cfg = load_config()
        except Exception:
            cfg = {}
        all_deps = cfg.get(_MODULE_DEPS_KEY, {})
        if installed and deps:
            all_deps[module_id] = deps
        elif not installed:
            all_deps.pop(module_id, None)
        cfg[_MODULE_DEPS_KEY] = all_deps
        save_config(cfg)


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


def _gh_token() -> str | None:
    """Get GitHub token from environment, gh CLI, or gh config file."""
    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if token:
        return token
    # Try gh auth token command
    try:
        import subprocess as _sp
        result = _sp.run(["gh", "auth", "token"], capture_output=True, text=True, timeout=5)
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip()
    except Exception:
        pass
    # Fallback: read from gh config file
    gh_hosts = Path.home() / ".config" / "gh" / "hosts.yml"
    if gh_hosts.is_file():
        try:
            for line in gh_hosts.read_text().splitlines():
                line = line.strip()
                if line.startswith("oauth_token:"):
                    return line.split(":", 1)[1].strip()
        except Exception:
            pass
    return None


def _gh_request(url: str, timeout: int = 15):
    """Create a urllib Request with GitHub API headers and optional auth token."""
    import urllib.request
    headers = {"Accept": "application/vnd.github+json"}
    token = _gh_token()
    if token:
        headers["Authorization"] = f"token {token}"
    return urllib.request.Request(url, headers=headers)


def fetch_latest_release(repo: str | None = None) -> dict | None:
    """Fetch latest release info from GitHub API. Returns JSON dict or None."""
    import urllib.request

    repo = repo or _get_repo()
    url = f"https://api.github.com/repos/{repo}/releases/latest"
    req = _gh_request(url)
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return json.loads(resp.read().decode())
    except Exception:
        return None


def fetch_module_release(module_id: str, version: str | None = None, repo: str | None = None) -> dict | None:
    """Fetch a specific module's release by tag. Returns release dict or None.

    Tag format: module-{id}-v{version}
    If version is None, searches recent releases for the latest version of this module.
    """
    import urllib.request

    repo = repo or _get_repo()

    if version:
        tag = f"module-{module_id}-v{version}"
        url = f"https://api.github.com/repos/{repo}/releases/tags/{tag}"
        req = _gh_request(url)
        try:
            with urllib.request.urlopen(req, timeout=15) as resp:
                return json.loads(resp.read().decode())
        except Exception:
            return None

    # version not specified: find the latest release for this module
    prefix = f"module-{module_id}-v"
    url = f"https://api.github.com/repos/{repo}/releases?per_page=50"
    req = _gh_request(url)
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            releases = json.loads(resp.read().decode())
        for r in releases:
            tag = r.get("tag_name", "")
            if tag.startswith(prefix):
                return r
    except Exception:
        pass
    return None


def get_remote_versions(release: dict | None = None) -> dict[str, str]:
    """Scan recent releases to get {module_id: latest_version}.

    Each module has its own release with tag: module-{id}-v{version}
    """
    import urllib.request

    repo = _get_repo()
    url = f"https://api.github.com/repos/{repo}/releases?per_page=100"
    req = _gh_request(url)
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            releases = json.loads(resp.read().decode())
    except Exception:
        return {}

    result: dict[str, str] = {}
    for r in releases:
        tag = r.get("tag_name", "")
        # module-robot_piper-v1.0.1 → id=robot_piper, ver=1.0.1
        if not tag.startswith("module-"):
            continue
        stripped = tag[len("module-"):]
        parts = stripped.rsplit("-v", 1)
        if len(parts) == 2 and parts[1] and parts[1][0].isdigit():
            mid, ver = parts[0], parts[1]
            if mid not in result:  # 최신순이므로 첫 번째만 유지
                result[mid] = ver
    return result


_ROS2_CONTAINER = "easytrainer_ros2"
_BACKEND_CONTAINER = "easytrainer_backend"


def _install_deps_in_containers(meta: dict) -> None:
    """Install module dependencies inside running Docker containers via docker exec."""
    import subprocess as _sp

    deps = meta.get("dependencies", {})
    install = meta.get("install", {})

    # Determine which containers need deps
    containers = []
    category = meta.get("category", "")
    if category in ("robot", "sensor"):
        containers.append(_ROS2_CONTAINER)
    elif category == "extension":
        containers.append(_BACKEND_CONTAINER)
    else:
        containers.extend([_ROS2_CONTAINER, _BACKEND_CONTAINER])

    for container in containers:
        # Check container is running
        try:
            result = _sp.run(
                ["docker", "inspect", "-f", "{{.State.Running}}", container],
                capture_output=True, text=True, timeout=5,
            )
            if "true" not in result.stdout.lower():
                continue
        except Exception:
            continue

        # pip deps
        pip_deps = deps.get("pip", [])
        if pip_deps:
            try:
                _sp.run(
                    ["docker", "exec", container, "python3", "-m", "pip", "install", "--quiet"] + pip_deps,
                    timeout=120, check=False,
                )
            except Exception:
                pass

        # apt deps
        apt_deps = deps.get("apt", [])
        if apt_deps:
            try:
                _sp.run(
                    ["docker", "exec", container, "bash", "-c",
                     "apt-get update -qq && apt-get install -y --no-install-recommends " + " ".join(apt_deps)],
                    timeout=300, check=False,
                )
            except Exception:
                pass

    # SDK install_cmd (e.g. "pip3 install -e .")
    sdk_cfg = install.get("sdk", {})
    install_cmd = sdk_cfg.get("install_cmd", "")
    sdk_target = sdk_cfg.get("target", "")
    if install_cmd and sdk_target:
        # sdk_target is relative to project root, map to container path
        # e.g. "ros2/robot_sdk/piper" → "/root/robot_sdk/piper"
        container_sdk_path = sdk_target.replace("ros2/robot_sdk", "/root/robot_sdk")
        try:
            result = _sp.run(
                ["docker", "inspect", "-f", "{{.State.Running}}", _ROS2_CONTAINER],
                capture_output=True, text=True, timeout=5,
            )
            if "true" in result.stdout.lower():
                _sp.run(
                    ["docker", "exec", "-w", container_sdk_path, _ROS2_CONTAINER,
                     "bash", "-c", install_cmd],
                    timeout=120, check=False,
                )
        except Exception:
            pass


def _get_project_root() -> Path:
    """Return the project root (where docker-compose.yml lives)."""
    data_dir = os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer")
    project = Path(data_dir) / "project"
    if project.exists():
        return project
    return Path.cwd()


def _get_install_dir(mod: ModuleInfo) -> Path:
    """Return the base directory where a module should be installed (extensions only)."""
    project = _get_project_root()
    if mod.category in ("robot", "sensor"):
        # robot/sensor modules are handled by _install_robot_sensor_module
        return project / "ros2"
    else:
        return project / "backend" / "extensions"


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

    # Entitlement gate. The home-next API is also the source of truth for
    # "owned vs. not owned"; checking here is defense-in-depth so a user with
    # a tampered launcher can't sideload a paid module.
    # If the catalog isn't loaded yet (offline first run) we let it through —
    # the payment phase is opt-in for paid modules and we don't want to break
    # the wizard for users who lost network mid-install.
    if remote_state_loaded() and not is_module_entitled(module_id):
        print(f"[MODULE] {module_id}: not entitled — purchase required")
        return False

    # 모듈별 릴리즈를 직접 조회
    if release is None:
        release = fetch_module_release(module_id)
    if not release:
        return False

    # Find the asset matching this module ID
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
            req = _gh_request(asset_url, timeout=120)
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

        # Extract and install
        # module.json에서 메타정보 추출
        _is_pkg_only = False
        _module_deps = {}
        _module_meta = {}
        with tarfile.open(tmp_path, "r:gz") as tar:
            for mj_member in tar.getmembers():
                if mj_member.name.endswith("module.json"):
                    try:
                        _module_meta = json.loads(tar.extractfile(mj_member).read())
                        check_type = _module_meta.get("check", {}).get("type", "path")
                        if check_type in ("apt", "pip"):
                            _is_pkg_only = True
                        _module_deps = _module_meta.get("dependencies", {})
                    except Exception:
                        pass
                    break

        if _is_pkg_only:
            # apt/pip 모듈: 파일 복사 없이 의존성만 설치
            with tempfile.TemporaryDirectory() as tmpdir:
                with tarfile.open(tmp_path, "r:gz") as tar:
                    tar.extractall(path=tmpdir)
                extracted = [d for d in Path(tmpdir).iterdir() if d.is_dir()]
                if extracted:
                    _install_module_deps(extracted[0], module_id)
        elif _module_meta.get("install", {}).get("compose"):
            # Docker compose 모듈 (sim_isaaclab 등): compose 파일 + 소스 디렉토리 별도 설치
            _install_compose_module(tmp_path, module_id, _module_meta)
        elif mod.category in ("robot", "sensor") or _module_meta.get("install", {}).get("ros2"):
            # ros2/ 트리를 가진 모듈은 같은 layout을 사용 (robot/sensor + sim 등)
            _install_robot_sensor_module(tmp_path, module_id)
        else:
            with tarfile.open(tmp_path, "r:gz") as tar:
                tar.extractall(path=str(install_dir))
            _install_module_deps(install_dir, module_id)

        # Save manifest to project/modules/{module_id}.json (single source of truth)
        if _module_meta:
            if remote_version:
                _module_meta["installed_version"] = remote_version
            _save_module_manifest(module_id, _module_meta)

        # Install deps inside running containers via docker exec
        _install_deps_in_containers(_module_meta)

        # 모듈에 post_install 스크립트가 있으면 실행
        _run_module_script(module_id, _module_meta, "post_install")

        set_module_installed(module_id, True, version=remote_version, deps=_module_deps)
        return True
    except Exception as e:
        import traceback
        print(f"[MODULE ERROR] {module_id}: {e}\n{traceback.format_exc()}")
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
    results: dict[str, bool] = {}
    total = len(module_ids)

    for i, mid in enumerate(module_ids):
        if on_module_start:
            on_module_start(mid, i, total)
        ok = download_module(mid, on_progress=on_progress)
        results[mid] = ok
        if on_module_done:
            on_module_done(mid, ok, i, total)

    return results


def _install_robot_sensor_module(tar_path: str, module_id: str) -> None:
    """Extract a robot/sensor module and copy ros2/ and sdk/ to their targets.

    Archive structure: module_name/{ros2/, sdk/, module.json}
    - ros2/ contents → project/ros2/ros2_ws/src/
    - sdk/ contents  → project/ros2/robot_sdk/<module_name>/
    - module.json    → project/ros2/ros2_ws/src/<module_name>/module.json (for dep install)
    """
    import shutil

    project = _get_project_root()

    with tempfile.TemporaryDirectory() as tmpdir:
        with tarfile.open(tar_path, "r:gz") as tar:
            tar.extractall(path=tmpdir)

        # Find the extracted module directory (top-level folder in tar)
        extracted = [d for d in Path(tmpdir).iterdir() if d.is_dir()]
        if not extracted:
            return
        module_dir = extracted[0]
        module_name = module_dir.name

        # Read module.json for install targets
        mj_path = module_dir / "module.json"
        install_cfg = {}
        if mj_path.exists():
            try:
                install_cfg = json.loads(mj_path.read_text()).get("install", {})
            except Exception:
                pass

        # Install ros2/ → ros2/ros2_ws/src/
        ros2_src = module_dir / "ros2"
        if ros2_src.is_dir() and any(ros2_src.iterdir()):
            ros2_target_cfg = install_cfg.get("ros2", {}).get("target", "ros2/ros2_ws/src")
            if not ros2_target_cfg.startswith("ros2/"):
                ros2_target_cfg = "ros2/" + ros2_target_cfg
            ros2_base = project / ros2_target_cfg

            # ros2_ws/src/<module_name>/ 안에 패키지들을 설치
            ros2_target = ros2_base / module_name
            if ros2_target.exists():
                shutil.rmtree(ros2_target)

            # ros2/src/ 가 있으면 그 안의 패키지들을 module_name/ 아래에 복사
            ros2_inner_src = ros2_src / "src"
            if ros2_inner_src.is_dir():
                ros2_target.mkdir(parents=True, exist_ok=True)
                for pkg_dir in ros2_inner_src.iterdir():
                    if pkg_dir.is_dir():
                        shutil.copytree(pkg_dir, ros2_target / pkg_dir.name)
            else:
                # src/ 없으면 ros2/ 전체를 module_name/ 으로 복사
                shutil.copytree(ros2_src, ros2_target)

        # Install sdk/ contents
        sdk_src = module_dir / "sdk"
        if sdk_src.is_dir() and any(f for f in sdk_src.iterdir() if f.name != ".gitkeep"):
            sdk_target_cfg = install_cfg.get("sdk", {}).get("target", f"ros2/robot_sdk/{module_name}")
            if not sdk_target_cfg.startswith("ros2/"):
                sdk_target_cfg = "ros2/" + sdk_target_cfg
            sdk_target = project / sdk_target_cfg
            sdk_target.mkdir(parents=True, exist_ok=True)
            for item in sdk_src.iterdir():
                if item.name == ".gitkeep":
                    continue
                dst = sdk_target / item.name
                if dst.exists():
                    shutil.rmtree(dst) if dst.is_dir() else dst.unlink()
                shutil.copytree(item, dst) if item.is_dir() else shutil.copy2(item, dst)

        # Copy module.json into ros2 target for dependency resolution
        ros2_dep_cfg = install_cfg.get("ros2", {}).get("target", "ros2/ros2_ws/src")
        if not ros2_dep_cfg.startswith("ros2/"):
            ros2_dep_cfg = "ros2/" + ros2_dep_cfg
        ros2_base = project / ros2_dep_cfg
        # ros2/src/ 구조일 때는 module_name과 같은 이름의 패키지에, 없으면 module_name/ 폴더에
        ros2_inner_src = module_dir / "ros2" / "src"
        if ros2_inner_src.is_dir() and (ros2_base / module_name).is_dir():
            target_mj = ros2_base / module_name / "module.json"
        else:
            target_mj = ros2_base / module_name / "module.json"
        if mj_path.exists() and not target_mj.exists():
            shutil.copy2(mj_path, target_mj)

        # Install dependencies
        _install_module_deps(ros2_base / module_name, module_id)


def _install_compose_module(tar_path: str, module_id: str, meta: dict) -> None:
    """Install a Docker compose-based module (e.g. sim_isaaclab).

    Archive structure: <module_name>/{module.json, docker-compose.*.yml, scripts/, <source_dir>/}
    - compose 파일 + 스크립트 → project/modules/<module_id>/
    - source 디렉토리 (e.g. isaaclab/) → project/<source_target>/
    """
    import shutil

    project = _get_project_root()
    install = meta.get("install", {})
    compose_cfg = install.get("compose", {})
    source_cfg = install.get("source", {})

    compose_target_rel = compose_cfg.get("target", f"modules/{module_id}")
    source_target_rel = source_cfg.get("target", "")

    with tempfile.TemporaryDirectory() as tmpdir:
        with tarfile.open(tar_path, "r:gz") as tar:
            tar.extractall(path=tmpdir)
        extracted = [d for d in Path(tmpdir).iterdir() if d.is_dir()]
        if not extracted:
            return
        module_dir = extracted[0]

        # 1) module 폴더 → project/modules/<module_id>/ (compose 파일, 스크립트 등)
        compose_target = project / compose_target_rel
        if compose_target.exists():
            shutil.rmtree(compose_target)
        compose_target.parent.mkdir(parents=True, exist_ok=True)

        # source 디렉토리는 별도 위치로 옮기므로 일단 모듈 폴더에 포함된 채 복사
        # (이후 source 디렉토리만 분리 이동)
        shutil.copytree(module_dir, compose_target)

        # 2) source 디렉토리 분리 (예: isaaclab/ → project/isaaclab/)
        if source_target_rel:
            source_inside = compose_target / source_target_rel.split("/")[-1]
            if source_inside.is_dir():
                source_target = project / source_target_rel
                if source_target.exists():
                    shutil.rmtree(source_target)
                source_target.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(source_inside), str(source_target))

        # 3) 권한 설정 (스크립트 실행 가능)
        scripts_dir = compose_target / "scripts"
        if scripts_dir.is_dir():
            for script in scripts_dir.iterdir():
                if script.suffix == ".sh":
                    try:
                        script.chmod(0o755)
                    except Exception:
                        pass

        print(f"[MODULE] {module_id}: installed compose to {compose_target}, source to {project / source_target_rel if source_target_rel else 'N/A'}")


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


def _uninstall_deps(meta: dict) -> None:
    """Uninstall pip packages listed in module dependencies (best-effort)."""
    import subprocess as _sp
    deps = meta.get("dependencies", {})
    pip_deps = deps.get("pip", [])
    if pip_deps:
        pkg_names = [p.split(">=")[0].split("==")[0].split("[")[0] for p in pip_deps]
        try:
            _sp.run(["python3", "-m", "pip", "uninstall", "-y", "--quiet"] + pkg_names,
                     timeout=60, check=False)
        except Exception:
            pass
    # apt packages: 일반적으로 제거하지 않음 (다른 모듈이 공유할 수 있으므로)


def _run_module_script(module_id: str, meta: dict, script_key: str) -> bool:
    """Run a module's lifecycle script (post_install, start_command, stop_command).

    Returns True if executed (or no script defined), False on error.
    """
    import subprocess as _sp

    script_rel = meta.get(script_key)
    if not script_rel:
        return True

    project = _get_project_root()
    install = meta.get("install", {})
    compose_target_rel = install.get("compose", {}).get("target")
    if compose_target_rel:
        module_dir = project / compose_target_rel
    else:
        # fallback: project/modules/<module_id>
        module_dir = project / "modules" / module_id

    script_path = module_dir / script_rel
    if not script_path.is_file():
        print(f"[MODULE] {module_id}: {script_key} script not found: {script_path}")
        return False

    # 환경변수 — config.json의 ros_domain_id 전달
    env = os.environ.copy()
    try:
        cfg = load_config()
        env["ROS_DOMAIN_ID"] = str(cfg.get("ros_domain_id", 0))
        env["EASYTRAINER_DATA_DIR"] = os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer")
    except Exception:
        pass

    try:
        result = _sp.run(
            ["bash", str(script_path)],
            cwd=str(module_dir),
            env=env,
            timeout=600,
            check=False,
        )
        return result.returncode == 0
    except Exception as e:
        print(f"[MODULE] {module_id}: {script_key} failed: {e}")
        return False


def _load_module_meta(module_id: str) -> dict | None:
    """Load module.json from installed location."""
    project = _get_project_root()
    # compose 모듈
    candidate = project / "modules" / module_id / "module.json"
    if candidate.is_file():
        try:
            return json.loads(candidate.read_text())
        except Exception:
            return None
    # ros2_ws/src 모듈
    ros2_src = project / "ros2" / "ros2_ws" / "src"
    if ros2_src.is_dir():
        for d in ros2_src.iterdir():
            mj = d / "module.json"
            if mj.is_file():
                try:
                    meta = json.loads(mj.read_text())
                    if meta.get("id") == module_id:
                        return meta
                except Exception:
                    continue
    return None


def start_module(module_id: str) -> bool:
    """Start a module's runtime (e.g. docker compose up)."""
    meta = _load_module_meta(module_id)
    if not meta:
        return False
    return _run_module_script(module_id, meta, "start_command")


def stop_module(module_id: str) -> bool:
    """Stop a module's runtime."""
    meta = _load_module_meta(module_id)
    if not meta:
        return False
    return _run_module_script(module_id, meta, "stop_command")


def restart_module(module_id: str) -> bool:
    """Restart a module (used when ROS_DOMAIN_ID changes)."""
    stop_module(module_id)
    return start_module(module_id)


def remove_module(module_id: str) -> bool:
    """Remove an installed module."""
    mod = get_module(module_id)
    if not mod or mod.required:
        return False

    project = _get_project_root()

    # Load manifest for uninstall info
    manifest = _load_module_manifest(module_id)

    # Compose 모듈 (sim_isaaclab 등): stop 후 폴더 + source 디렉토리 삭제
    if manifest and manifest.get("install", {}).get("compose"):
        stop_module(module_id)
        compose_target = manifest["install"]["compose"].get("target", f"modules/{module_id}")
        compose_path = project / compose_target
        if compose_path.is_dir():
            shutil.rmtree(compose_path, ignore_errors=True)
        source_target = manifest.get("install", {}).get("source", {}).get("target", "")
        if source_target:
            source_path = project / source_target
            if source_path.is_dir():
                shutil.rmtree(source_path, ignore_errors=True)
        set_module_installed(module_id, False)
        return True

    if mod.category in ("robot", "sensor"):
        # ros2_ws/src/ 에서 module.json으로 모듈 폴더 찾아 삭제
        ros2_src = project / "ros2" / "ros2_ws" / "src"
        if ros2_src.is_dir():
            for d in ros2_src.iterdir():
                if not d.is_dir():
                    continue
                mj = d / "module.json"
                if mj.is_file():
                    try:
                        meta = json.loads(mj.read_text())
                        if meta.get("id") == module_id:
                            shutil.rmtree(d, ignore_errors=True)
                            break
                    except Exception:
                        continue

        # robot_sdk/ 에서도 삭제 — manifest의 install.sdk.target 경로 사용
        sdk_target = None
        if manifest:
            sdk_target = manifest.get("install", {}).get("sdk", {}).get("target", "")
        _log = f"[MODULE REMOVE] {module_id}: manifest={manifest is not None}, sdk_target={sdk_target!r}, project={project}\n"
        if sdk_target:
            sdk_path = project / sdk_target
            _log += f"[MODULE REMOVE] sdk_path={sdk_path}, exists={sdk_path.is_dir()}\n"
            if sdk_path.is_dir():
                try:
                    shutil.rmtree(sdk_path)
                except PermissionError:
                    # root 소유 파일(egg-info 등)이 있으면 docker exec로 삭제
                    import subprocess as _sp
                    container_path = sdk_target.replace("ros2/robot_sdk", "/root/robot_sdk")
                    _sp.run(["docker", "exec", _ROS2_CONTAINER, "rm", "-rf", container_path],
                            check=False, timeout=10)
                    # 컨테이너가 없을 경우 sudo fallback
                    if sdk_path.is_dir():
                        _sp.run(["sudo", "rm", "-rf", str(sdk_path)], check=False, timeout=10)
                _log += f"[MODULE REMOVE] Deleted {sdk_path}, still_exists={sdk_path.exists()}\n"
        else:
            # fallback: 이름 추측
            sdk_dir = project / "ros2" / "robot_sdk"
            if sdk_dir.is_dir():
                expected_name = module_id.replace("robot_", "").replace("sensor_", "").replace("gripper_", "")
                candidate = sdk_dir / expected_name
                if candidate.is_dir():
                    shutil.rmtree(candidate, ignore_errors=True)
    else:
        # extension: backend/extensions/ 에서 삭제
        install_dir = project / "backend" / "extensions"
        if install_dir.is_dir():
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

    # Uninstall pip dependencies if manifest available
    if manifest:
        _uninstall_deps(manifest)

    # Remove manifest from project/modules/
    _remove_module_manifest(module_id)

    # Legacy modules dir
    legacy_dir = Path(os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer")) / "modules" / module_id
    if legacy_dir.exists():
        shutil.rmtree(legacy_dir, ignore_errors=True)

    set_module_installed(module_id, False)

    # Debug log to file
    try:
        Path("/tmp/easytrainer_module_remove.log").write_text(_log)
    except Exception:
        pass

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
    """학습 서버 소스가 설치되어 있는가?

    training_server lives under backend/training_server/ and is bind-mounted
    into the backend container as part of the regular ./backend mount.
    """
    cfg = get_training_server_config()
    project_root = cfg.get("project_root") or "/opt/easytrainer/project"
    app_py = Path(project_root) / "backend" / "training_server" / "app.py"
    compose = Path(project_root) / "docker-compose.yml"
    return app_py.is_file() and compose.is_file()


def set_training_server_installed(installed: bool) -> None:
    """Legacy no-op — install state is now derived from filesystem (see is_training_server_installed)."""
    pass


def get_training_mode() -> str:
    """Return 'local' or 'remote'."""
    return get_training_server_config().get("mode", "remote")


def set_training_mode(mode: str) -> None:
    ts = get_training_server_config()
    ts["mode"] = mode
    save_training_server_config(ts)


# ---------------------------------------------------------------------------
# Catalog + entitlements (home-next API)
# ---------------------------------------------------------------------------
#
# Two pieces of remote state:
#   - Catalog: { module_id: priceKrw }       — public, /api/modules
#   - Owned:   set of entitled module_ids    — Bearer-auth, /api/entitlements
#
# Both are cached in process for the launcher session. UI dialogs call
# refresh_remote_state() before opening so the catalog is fresh.

_REMOTE_LOCK = threading.Lock()
_REMOTE_CATALOG: dict[str, dict] = {}     # module_id -> {name, category, priceKrw, ...}
_REMOTE_OWNED: set[str] = set()
_REMOTE_LOADED = False


# Vercel moved the canonical alias to easy-trainer-home.vercel.app; the old
# host now returns 307 redirects which break POST/Bearer flows on urllib.
_LEGACY_HOSTS = ("easytrainerhome.vercel.app",)
_DEFAULT_API_URL = "https://easy-trainer-home.vercel.app"


def _canonicalize_api_url(url: str) -> str:
    out = url
    for legacy in _LEGACY_HOSTS:
        out = out.replace(legacy, "easy-trainer-home.vercel.app")
    return out


def _api_base_url() -> str:
    try:
        cfg = load_config()
    except Exception:
        return _DEFAULT_API_URL
    raw = (cfg.get("license_server_url") or _DEFAULT_API_URL).rstrip("/")
    fixed = _canonicalize_api_url(raw)
    if fixed != raw:
        try:
            cfg["license_server_url"] = fixed
            from app_context import save_config
            save_config(cfg)
        except Exception:
            pass
    return fixed


def get_api_base_url() -> str:
    """Public alias of _api_base_url for callers outside this module."""
    return _api_base_url()


def _get_bearer() -> str | None:
    """Read the saved access_token written by device_auth.save_auth()."""
    try:
        from app_context import APP_HOME
        auth_file = APP_HOME / "auth.json"
        if not auth_file.exists():
            return None
        data = json.loads(auth_file.read_text(encoding="utf-8"))
        return data.get("access_token")
    except Exception:
        return None


def _http_get_json(url: str, *, bearer: str | None = None, timeout: float = 10) -> tuple[int, dict]:
    headers = {"Accept": "application/json"}
    if bearer:
        headers["Authorization"] = f"Bearer {bearer}"
    req = urllib.request.Request(url, headers=headers, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads((resp.read() or b"{}").decode())
    except urllib.error.HTTPError as e:
        try:
            body = json.loads((e.read() or b"{}").decode())
        except Exception:
            body = {"error": f"HTTP {e.code}"}
        return e.code, body
    except Exception as e:
        return 0, {"error": str(e)}


def fetch_catalog() -> dict[str, dict]:
    """Fetch /api/modules and return {id: {name, category, priceKrw, description}}."""
    url = f"{_api_base_url()}/api/modules"
    status, payload = _http_get_json(url)
    if status != 200:
        return {}
    out: dict[str, dict] = {}
    for m in payload.get("modules", []):
        mid = m.get("id")
        if mid:
            out[mid] = m
    return out


def fetch_owned_module_ids() -> set[str]:
    """Fetch /api/entitlements (Bearer required) and return the set of owned module IDs."""
    bearer = _get_bearer()
    if not bearer:
        return set()
    url = f"{_api_base_url()}/api/entitlements"
    status, payload = _http_get_json(url, bearer=bearer)
    if status != 200:
        return set()
    return set(payload.get("moduleIds", []))


def _module_info_from_payload(m: dict) -> Optional[ModuleInfo]:
    """Convert one /api/modules entry into a ModuleInfo, defensive about missing keys."""
    try:
        mid = m.get("id")
        if not mid:
            return None
        deps = m.get("dependencies") or []
        return ModuleInfo(
            id=mid,
            name=m.get("name") or mid,
            category=m.get("category") or "extension",
            description=m.get("description") or "",
            required=bool(m.get("required")),
            default=bool(m.get("installByDefault")),
            asset_name=m.get("assetName") or "",
            dependencies=list(deps) if isinstance(deps, list) else [],
        )
    except Exception:
        return None


def _registry_cache_path() -> Path:
    from app_context import APP_HOME
    return APP_HOME / "modules-cache.json"


def _load_cached_registry() -> Optional[list[ModuleInfo]]:
    """Read the last-known-good API response from disk so the launcher works
    offline (after at least one successful online launch)."""
    try:
        path = _registry_cache_path()
        if not path.exists():
            return None
        data = json.loads(path.read_text(encoding="utf-8"))
        items = []
        for raw in data.get("modules", []):
            mi = _module_info_from_payload(raw)
            if mi:
                items.append(mi)
        return items if items else None
    except Exception:
        return None


def _save_cached_registry(payload: dict) -> None:
    try:
        path = _registry_cache_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception:
        pass


def refresh_remote_state() -> None:
    """Pull the latest module catalog + owned-set from home-next.

    On success: refresh MODULE_REGISTRY in place, persist cache, update
    catalog/owned snapshot. On failure (network down, server error): leave
    existing registry contents (loaded from fallback or cache) untouched.

    Catalog and owned-set fetches run in parallel since they're independent
    HTTP requests (cuts wall time roughly in half on Vercel cold-start).
    """
    global _REMOTE_LOADED

    catalog_box: dict[str, dict] = {}
    items_box: list[ModuleInfo] = []
    raw_modules_box: list[dict] = []
    owned_box: set[str] = set()

    def _fetch_catalog():
        url = f"{_api_base_url()}/api/modules"
        status, payload = _http_get_json(url)
        if status == 200 and isinstance(payload, dict):
            for raw in payload.get("modules") or []:
                mi = _module_info_from_payload(raw)
                if mi:
                    items_box.append(mi)
                    catalog_box[mi.id] = raw
                    raw_modules_box.append(raw)

    def _fetch_owned():
        owned_box.update(fetch_owned_module_ids())

    t1 = threading.Thread(target=_fetch_catalog, daemon=True)
    t2 = threading.Thread(target=_fetch_owned, daemon=True)
    t1.start()
    t2.start()
    t1.join(timeout=15)
    t2.join(timeout=15)

    if items_box:
        _apply_registry(items_box)
        _save_cached_registry({"modules": raw_modules_box})

    with _REMOTE_LOCK:
        _REMOTE_CATALOG.clear()
        _REMOTE_CATALOG.update(catalog_box)
        _REMOTE_OWNED.clear()
        _REMOTE_OWNED.update(owned_box)
        _REMOTE_LOADED = True


# On import, prefer the cached registry over the hardcoded fallback so the
# launcher reflects the latest catalog even before refresh_remote_state runs.
_cached = _load_cached_registry()
if _cached:
    _apply_registry(_cached)


def get_module_price_krw(module_id: str) -> int:
    """Return the cached price in KRW. 0 means free or catalog miss."""
    with _REMOTE_LOCK:
        m = _REMOTE_CATALOG.get(module_id)
    if not m:
        return 0
    try:
        return int(m.get("priceKrw") or 0)
    except Exception:
        return 0


def is_module_entitled(module_id: str) -> bool:
    """Whether the signed-in user owns this module.

    Required-by-spec modules (core/feature) are always considered entitled
    locally — they ship with every launcher and are not gated by purchase even
    if the catalog says so.
    """
    mod = get_module(module_id)
    if mod and mod.required:
        return True
    with _REMOTE_LOCK:
        return module_id in _REMOTE_OWNED


def remote_state_loaded() -> bool:
    with _REMOTE_LOCK:
        return _REMOTE_LOADED

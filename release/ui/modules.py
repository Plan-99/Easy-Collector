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
from i18n import t

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
    local: bool = False    # 로컬 위자드로 만든 모듈 — 마켓 카탈로그에 없음


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
    ModuleInfo(id="robot_fairino", name="Fairino", category="robot", description="Fairino 협동로봇 (SDK 직접 제어)", asset_name="module-robot_fairino-{version}.tar.gz"),
    ModuleInfo(id="robot_rbpodo", name="RBPodo", category="robot", description="Rainbow Robotics RBPodo", asset_name="module-robot_rbpodo-{version}.tar.gz"),
    ModuleInfo(id="robot_kinova", name="Kinova Kortex", category="robot", description="Kinova Kortex 로봇", asset_name="module-robot_kinova-{version}.tar.gz"),
    ModuleInfo(id="robot_techman", name="Techman TM", category="robot", description="Techman TM 로봇", asset_name="module-robot_techman-{version}.tar.gz"),
    ModuleInfo(id="robot_omx", name="OMX", category="robot", description="ROBOTIS OMX (5-DOF arm + 1-DOF gripper, Dynamixel)", asset_name="module-robot_omx-{version}.tar.gz"),
    ModuleInfo(id="robot_omy", name="OMY", category="robot", description="ROBOTIS OMY (6-DOF arm, DYNAMIXEL-Y; OMY-3M / OMY-F3M + RH-P12-RN gripper)", asset_name="module-robot_omy-{version}.tar.gz"),
    ModuleInfo(id="robot_ai_worker", name="AI Worker", category="robot", description="ROBOTIS AI Worker (FFW-BG2/SG2 듀얼 7-DOF 암, 온보드 Orin, 원격 SSH)", asset_name="module-robot_ai_worker-{version}.tar.gz"),
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
    ModuleInfo(id="sam3", name="SAM 3 세그멘테이션", category="extension", description="Meta SAM 3 기반 객체 세그멘테이션 (텍스트/박스 프롬프트)", asset_name="module-sam3-{version}.tar.gz"),
    ModuleInfo(id="yoloe", name="YOLOE 비주얼-프롬프트 검출", category="extension", description="박스 exemplar로 타겟 지정 → 뷰가 바뀌어도 재검출 (cross-view one-shot)", asset_name="module-yoloe-{version}.tar.gz"),
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
    m = _REGISTRY_MAP.get(module_id)
    if m is not None:
        return m
    # 로컬 모듈 — manifest 가 있으면 ModuleInfo 형태로 합성. remove_module 등이 그대로 동작.
    meta = _load_module_manifest(module_id)
    if meta is not None:
        return ModuleInfo(
            id=module_id,
            name=meta.get("name") or module_id,
            category=meta.get("category") or "robot",
            description=meta.get("description") or "",
            asset_name="",
            local=True,
        )
    return None


def modules_by_category() -> dict[str, list[ModuleInfo]]:
    result: dict[str, list[ModuleInfo]] = {}
    seen_ids: set[str] = set()
    for m in MODULE_REGISTRY:
        result.setdefault(m.category, []).append(m)
        seen_ids.add(m.id)
    # 로컬 위자드로 만든 모듈을 카탈로그에 합쳐 다이얼로그에서 함께 보이게 한다.
    try:
        for mid, meta in get_all_installed_manifests().items():
            if mid in seen_ids:
                continue
            cat = meta.get("category") or "robot"
            info = ModuleInfo(
                id=mid,
                name=(meta.get("name") or mid) + t("mod.localSuffix"),
                category=cat,
                description=meta.get("description") or "",
                asset_name="",
                local=True,
            )
            result.setdefault(cat, []).append(info)
    except Exception:
        pass
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

    # version not specified: find the latest release for this module.
    # 이 repo 는 모듈×버전마다 릴리즈가 쌓여(현재 70+개) GitHub /releases 목록이
    # 날짜순으로 안정적이지 않다 — 한 페이지(50/100)만 보면 새 모듈이 윈도우 밖으로
    # 밀려 "release not found" 가 난다(예: ai_worker index 51). 전체 페이지를 돌며
    # prefix 매칭 중 **버전이 가장 높은** 릴리즈를 고른다.
    prefix = f"module-{module_id}-v"

    def _vkey(v: str):
        out = []
        for p in v.split("."):
            try:
                out.append(int(p))
            except ValueError:
                out.append(0)
        return tuple(out)

    best = None
    best_key: tuple = ()
    try:
        for page in range(1, 21):  # per_page=100 × 20 = 2000 릴리즈까지 커버
            url = f"https://api.github.com/repos/{repo}/releases?per_page=100&page={page}"
            req = _gh_request(url)
            with urllib.request.urlopen(req, timeout=15) as resp:
                releases = json.loads(resp.read().decode())
            if not releases:
                break
            for r in releases:
                tag = r.get("tag_name", "")
                if tag.startswith(prefix):
                    k = _vkey(tag[len(prefix):])
                    if k >= best_key:
                        best_key = k
                        best = r
            if len(releases) < 100:
                break
    except Exception:
        pass
    return best


def get_remote_versions(release: dict | None = None) -> dict[str, str]:
    """Scan recent releases to get {module_id: latest_version}.

    Each module has its own release with tag: module-{id}-v{version}
    """
    return {mid: info["version"] for mid, info in _scan_remote_module_releases().items()}


def get_remote_sizes(release: dict | None = None) -> dict[str, int]:
    """Scan recent releases to get {module_id: latest_asset_size_bytes}."""
    return {mid: info["size"] for mid, info in _scan_remote_module_releases().items()
            if info.get("size")}


def _scan_remote_module_releases() -> dict[str, dict]:
    """Walk recent module releases and pick the newest version per module id.

    Returns {module_id: {"version": str, "size": int}}. Size comes from the
    matching tar.gz asset; missing assets just leave size unset.
    """
    import urllib.request

    repo = _get_repo()

    def _vkey(v: str):
        out = []
        for p in v.split("."):
            try:
                out.append(int(p))
            except ValueError:
                out.append(0)
        return tuple(out)

    # /releases 목록은 날짜순이 안정적이지 않고 항목이 70+개라, 전체 페이지를 돌며
    # 모듈별로 **버전이 가장 높은** 릴리즈를 고른다(한 페이지·first-hit 는 신규 모듈을
    # 누락하거나 옛 버전을 집을 수 있음).
    result: dict[str, dict] = {}
    best_key: dict[str, tuple] = {}
    try:
        for page in range(1, 21):  # per_page=100 × 20
            url = f"https://api.github.com/repos/{repo}/releases?per_page=100&page={page}"
            req = _gh_request(url)
            with urllib.request.urlopen(req, timeout=15) as resp:
                releases = json.loads(resp.read().decode())
            if not releases:
                break
            for r in releases:
                tag = r.get("tag_name", "")
                if not tag.startswith("module-"):
                    continue
                stripped = tag[len("module-"):]
                parts = stripped.rsplit("-v", 1)
                if len(parts) != 2 or not parts[1] or not parts[1][0].isdigit():
                    continue
                mid, ver = parts[0], parts[1]
                k = _vkey(ver)
                if mid in best_key and k < best_key[mid]:
                    continue
                best_key[mid] = k
                info: dict = {"version": ver}
                prefix = f"module-{mid}-"
                for asset in r.get("assets", []) or []:
                    name = asset.get("name", "")
                    if name.startswith(prefix) and name.endswith(".tar.gz"):
                        try:
                            info["size"] = int(asset.get("size") or 0)
                        except Exception:
                            pass
                        break
                result[mid] = info
            if len(releases) < 100:
                break
    except Exception:
        pass
    return result


_ROS2_CONTAINER = "easytrainer_ros2"
_BACKEND_CONTAINER = "easytrainer_backend"
_CONTAINER_TO_SERVICE = {
    _ROS2_CONTAINER: "ros2",
    _BACKEND_CONTAINER: "backend",
}


def _affected_targets(meta: dict) -> set[str]:
    """Return {"ros2", "backend"} subset whose images need to be rebuilt for this module.

    Bake rules (mirror of scripts/rebuild_images.sh):
      - robot/sensor 카테고리에 dependencies.apt|pip 가 있으면 ros2 베이크
      - extension   카테고리에 dependencies.apt|pip 가 있으면 backend 베이크
      - 카테고리 무관 dependencies.backend_apt|backend_pip 는 backend 베이크
    """
    deps = meta.get("dependencies", {}) or {}
    category = meta.get("category", "")
    has_main = bool(deps.get("apt") or deps.get("pip"))
    has_backend = bool(deps.get("backend_apt") or deps.get("backend_pip"))
    out: set[str] = set()
    if has_main:
        if category in ("robot", "sensor"):
            out.add("ros2")
        elif category == "extension":
            out.add("backend")
        else:
            out.update({"ros2", "backend"})
    if has_backend:
        out.add("backend")
    return out


def _rebuild_and_restart(targets: set[str]) -> bool:
    """Rebuild derived image(s) for the given service names then restart them.

    Returns True if rebuild & restart completed successfully for all targets.
    On failure (e.g. rebuild script missing, build error) returns False — caller
    may fall back to legacy docker-exec install for "good enough" immediate
    availability until the next clean rebuild.
    """
    import subprocess as _sp

    if not targets:
        return True

    project_root = _get_project_root()
    rebuild_script = project_root / "scripts" / "rebuild_images.sh"
    compose_file = None
    for name in ("docker-compose.yml", "docker-compose.cpu.yml"):
        cand = project_root / name
        if cand.is_file():
            compose_file = cand
            break

    if not rebuild_script.is_file():
        print(f"[MODULE] rebuild script missing — skipping image bake: {rebuild_script}")
        return False
    if compose_file is None:
        print(f"[MODULE] docker-compose file not found under {project_root} — skipping restart.")
        return False

    ok = True
    for target in sorted(targets):  # deterministic order: backend, ros2
        print(f"[MODULE] rebuilding image: {target}")
        try:
            r = _sp.run(
                ["bash", str(rebuild_script), target],
                cwd=str(project_root),
                timeout=1800,  # 30분 (첫 base 빌드 여유)
                check=False,
            )
            if r.returncode != 0:
                print(f"[MODULE] rebuild {target} returned {r.returncode}")
                ok = False
        except Exception as e:
            print(f"[MODULE] rebuild {target} failed: {e}")
            ok = False

    if ok:
        try:
            print(f"[MODULE] restarting containers: {sorted(targets)}")
            _sp.run(
                ["docker", "compose", "-f", str(compose_file), "up", "-d", "--no-build"] + sorted(targets),
                cwd=str(project_root),
                timeout=120, check=False,
            )
        except Exception as e:
            print(f"[MODULE] container restart failed: {e}")
            ok = False
    return ok


def _legacy_docker_exec_install(meta: dict) -> None:
    """Fallback: install module deps via docker exec into the running container.

    Used when image rebuild infra is unavailable (older project layout, missing
    scripts, build failure). Packages installed this way live only in the
    container's writable layer — they vanish on `docker compose down && up`.
    The next successful rebuild_and_restart cycle restores persistence.
    """
    import subprocess as _sp

    deps = meta.get("dependencies", {}) or {}
    category = meta.get("category", "")

    containers: list[str] = []
    if category in ("robot", "sensor"):
        containers.append(_ROS2_CONTAINER)
    elif category == "extension":
        containers.append(_BACKEND_CONTAINER)
    else:
        containers.extend([_ROS2_CONTAINER, _BACKEND_CONTAINER])

    def _is_running(container: str) -> bool:
        try:
            r = _sp.run(["docker", "inspect", "-f", "{{.State.Running}}", container],
                        capture_output=True, text=True, timeout=5)
            return "true" in r.stdout.lower()
        except Exception:
            return False

    def _pip_install(container: str, pkgs: list) -> None:
        if not pkgs:
            return
        cmd_base = ["docker", "exec", container, "python3", "-m", "pip", "install", "--quiet"]
        try:
            r = _sp.run(cmd_base + ["--break-system-packages"] + pkgs,
                        capture_output=True, text=True, timeout=120, check=False)
            if r.returncode == 0:
                return
            _sp.run(cmd_base + pkgs, timeout=120, check=False)
        except Exception:
            pass

    def _apt_install(container: str, pkgs: list) -> None:
        if not pkgs:
            return
        try:
            _sp.run(
                ["docker", "exec", container, "bash", "-c",
                 "apt-get update -qq && apt-get install -y --no-install-recommends " + " ".join(pkgs)],
                timeout=300, check=False,
            )
        except Exception:
            pass

    pip_deps = deps.get("pip", []) or []
    apt_deps = deps.get("apt", []) or []
    backend_pip_deps = deps.get("backend_pip", []) or []
    backend_apt_deps = deps.get("backend_apt", []) or []

    for c in containers:
        if not _is_running(c):
            continue
        _pip_install(c, pip_deps)
        _apt_install(c, apt_deps)

    if (backend_pip_deps or backend_apt_deps) and _is_running(_BACKEND_CONTAINER):
        _pip_install(_BACKEND_CONTAINER, backend_pip_deps)
        _apt_install(_BACKEND_CONTAINER, backend_apt_deps)


def _install_deps_in_containers(meta: dict) -> None:
    """Bake module deps into derived images, restart affected containers, then
    run any SDK install_cmd (bind-mount source, idempotent).

    Replaces the old docker-exec install. New behavior is "persistent by
    construction" — deps survive `docker compose down && up`. If the rebuild
    pipeline isn't available (older layout, transient build error) we fall
    back to docker exec install so the module is still usable until the next
    successful rebuild.
    """
    import subprocess as _sp

    targets = _affected_targets(meta)
    if targets:
        rebuilt = _rebuild_and_restart(targets)
        if not rebuilt:
            print("[MODULE] image rebuild failed — falling back to docker exec install.")
            _legacy_docker_exec_install(meta)

    # SDK install_cmd (e.g. "pip3 install -e .") — bind-mount source install,
    # idempotent. start_ros2_services.sh re-runs an equivalent check on every
    # boot so this is best-effort (skip on failure).
    install = meta.get("install", {}) or {}
    sdk_cfg = install.get("sdk", {}) or {}
    install_cmd = sdk_cfg.get("install_cmd", "")
    if install_cmd and meta.get("id"):
        container_sdk_path = f"/root/robot_sdk/{meta['id']}"
        try:
            r = _sp.run(
                ["docker", "inspect", "-f", "{{.State.Running}}", _ROS2_CONTAINER],
                capture_output=True, text=True, timeout=5,
            )
            if "true" in r.stdout.lower():
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
        # Guard against a dropped connection: a short file vs the advertised
        # size means a truncated download — fail with a clear error here rather
        # than a confusing "tar 압축 해제 실패" later (and never install a partial).
        if total and downloaded < total:
            raise RuntimeError(
                t("mod.downloadTruncated", downloaded=downloaded, total=total)
            )

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
            # colcon build 도 설치의 일부 — entrypoint 가 install/ 존재 시 재빌드
            # 안 하므로, 설치 직후 빌드 안 하면 새 패키지가 ros2 launch 에서 'not found'.
            _colcon_build_module(module_id)
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

        # 일부 sensor 모듈(webcam_publisher 등)은 tar 최상위가 곧 ROS 패키지인
        # bare 레이아웃이라 ros2/ 서브폴더가 없다. 이 경우 module_dir 자체를 ROS
        # 패키지로 보고 그대로 ros2_ws/src/<module_id>/ 에 복사한다.
        if not (module_dir / "ros2").is_dir() and (module_dir / "package.xml").is_file():
            ros2_target_cfg = install_cfg.get("ros2", {}).get("target", "ros2/ros2_ws/src")
            if not ros2_target_cfg.startswith("ros2/"):
                ros2_target_cfg = "ros2/" + ros2_target_cfg
            ros2_target = project / ros2_target_cfg / module_id
            if ros2_target.exists():
                try:
                    shutil.rmtree(ros2_target)
                except PermissionError:
                    import subprocess as _sp
                    parts = ros2_target.relative_to(project).parts
                    container_path = '/root/' + '/'.join(parts[1:])
                    _sp.run(["docker", "exec", _ROS2_CONTAINER, "rm", "-rf", container_path],
                            check=False, timeout=15)
                    if ros2_target.exists():
                        _sp.run(["sudo", "rm", "-rf", str(ros2_target)], check=False, timeout=15)
            ros2_target.mkdir(parents=True, exist_ok=True)
            for item in module_dir.iterdir():
                if item.name == "module.json":
                    continue  # module.json 은 별도 step 에서 복사
                dst = ros2_target / item.name
                try:
                    if item.is_dir():
                        shutil.copytree(item, dst, symlinks=True)
                    else:
                        shutil.copy2(item, dst)
                except Exception as e:
                    print(f"[MODULE] {module_id}: skip {item.name}: {e}")

        # Install ros2/ → ros2/ros2_ws/src/
        ros2_src = module_dir / "ros2"
        if ros2_src.is_dir() and any(ros2_src.iterdir()):
            ros2_target_cfg = install_cfg.get("ros2", {}).get("target", "ros2/ros2_ws/src")
            if not ros2_target_cfg.startswith("ros2/"):
                ros2_target_cfg = "ros2/" + ros2_target_cfg
            ros2_base = project / ros2_target_cfg

            # ros2_ws/src/<module_id>/ 안에 패키지들을 설치
            # (옛 버전은 tar 의 top folder name 을 썼으나 vendor 명과 module_id 가 달라
            #  manifest path 와 mismatch. id 로 통일.)
            ros2_target = ros2_base / module_id
            if ros2_target.exists():
                # 이전 빌드의 .pyc / __pycache__ 등이 컨테이너 안에서 root 소유로
                # 만들어져 있을 수 있다. 호스트 user 로 직접 rmtree 가 막히면
                # docker exec / sudo 로 폴백.
                try:
                    shutil.rmtree(ros2_target)
                except PermissionError:
                    # 호스트 <project>/ros2/ros2_ws/src/<id> ↔ 컨테이너 /root/ros2_ws/src/<id>
                    import subprocess as _sp
                    parts = ros2_target.relative_to(project).parts  # ('ros2', 'ros2_ws', ...)
                    container_path = '/root/' + '/'.join(parts[1:])
                    _sp.run(["docker", "exec", _ROS2_CONTAINER, "rm", "-rf", container_path],
                            check=False, timeout=15)
                    if ros2_target.exists():
                        _sp.run(["sudo", "rm", "-rf", str(ros2_target)], check=False, timeout=15)
                    if ros2_target.exists():
                        raise

            # ros2/src/ 가 있으면 그 안의 패키지들을 module_name/ 아래에 복사.
            # symlinks=True 로 두면 깨진 절대경로 심볼릭 링크(예: fairino libfairino.so
            # 체인이 빌드 시점 /root/ros2_ws 경로를 가리키는 경우)를 그대로 보존해
            # follow 시 PermissionError로 설치 자체가 실패하는 문제를 막는다.
            ros2_inner_src = ros2_src / "src"
            if ros2_inner_src.is_dir():
                ros2_target.mkdir(parents=True, exist_ok=True)
                for pkg_dir in ros2_inner_src.iterdir():
                    if pkg_dir.is_dir():
                        shutil.copytree(pkg_dir, ros2_target / pkg_dir.name, symlinks=True)
                # ros2/ 루트의 비-src 콘텐츠(스크립트, README, asserts 등) 도 함께 복사.
                # 예: piper 의 can_activate_main.sh 가 모듈 루트에 있어 패키지 안에는 없음.
                for item in ros2_src.iterdir():
                    if item.name in ("src", "install", "build", "log"):
                        continue
                    dst = ros2_target / item.name
                    if dst.exists():
                        continue
                    try:
                        if item.is_dir():
                            shutil.copytree(item, dst, symlinks=True)
                        else:
                            shutil.copy2(item, dst)
                    except Exception as e:
                        print(f"[MODULE] {module_id}: skip {item.name}: {e}")
            else:
                # src/ 없으면 ros2/ 전체를 module_name/ 으로 복사
                shutil.copytree(ros2_src, ros2_target, symlinks=True)

        # Install sdk/ contents
        sdk_src = module_dir / "sdk"
        if sdk_src.is_dir() and any(f for f in sdk_src.iterdir() if f.name != ".gitkeep"):
            # sdk 설치 폴더도 module_id 로 통일 (manifest 의 install.sdk.target 무시)
            sdk_target_cfg = f"ros2/robot_sdk/{module_id}"
            if not sdk_target_cfg.startswith("ros2/"):
                sdk_target_cfg = "ros2/" + sdk_target_cfg
            sdk_target = project / sdk_target_cfg
            sdk_target.mkdir(parents=True, exist_ok=True)
            for item in sdk_src.iterdir():
                if item.name == ".gitkeep":
                    continue
                dst = sdk_target / item.name
                if dst.exists():
                    # 컨테이너가 root로 생성한 __pycache__ / .egg-info가 남아있으면
                    # 호스트 launcher가 PermissionError로 막힌다 → docker exec rm으로 fallback.
                    try:
                        shutil.rmtree(dst) if dst.is_dir() else dst.unlink()
                    except PermissionError:
                        import subprocess as _sp
                        container_path = f"/root/robot_sdk/{module_id}/{item.name}"
                        _sp.run(["docker", "exec", _ROS2_CONTAINER, "rm", "-rf", container_path],
                                check=False, timeout=10)
                        if dst.exists():
                            _sp.run(["sudo", "rm", "-rf", str(dst)], check=False, timeout=10)
                shutil.copytree(item, dst) if item.is_dir() else shutil.copy2(item, dst)

        # Copy module.json into ros2 target for dependency resolution.
        # 설치 폴더는 module_id 로 통일했으므로(line 765) tar top folder name(module_name)
        # 이 아닌 module_id 를 써야 한다 — 예: tar 의 'webcam_publisher' 가 'sensor_webcam'
        # 으로 설치되기 때문에 module_name 을 쓰면 path 불일치로 FileNotFoundError.
        ros2_dep_cfg = install_cfg.get("ros2", {}).get("target", "ros2/ros2_ws/src")
        if not ros2_dep_cfg.startswith("ros2/"):
            ros2_dep_cfg = "ros2/" + ros2_dep_cfg
        ros2_base = project / ros2_dep_cfg
        target_mj = ros2_base / module_id / "module.json"
        if mj_path.exists() and not target_mj.exists():
            target_mj.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(mj_path, target_mj)

        # Install dependencies
        _install_module_deps(ros2_base / module_id, module_id)


def _colcon_build_module(module_id: str) -> None:
    """모듈이 설치한 ROS2 패키지들을 ros2 컨테이너에서 colcon build.

    이게 없으면 새 모듈 설치 후 컨테이너를 재시작해야만 패키지가 인식됨 — 사용자가
    바로 launch 시 'package not found' 에러를 봤음. 설치 직후 build 를 해 즉시 사용
    가능하게 만든다.

    빌드 대상은 `<install_dir>/<module_id>/` 아래의 모든 package.xml 에서 추출한
    패키지 이름들. tar top folder 이름과 module_id 가 다를 수 있어(예: tar 의
    'webcam_publisher' → install 'sensor_webcam') 정확한 매핑은 manifest 가 아닌
    실제 package.xml 의 <name> 태그를 본다.

    실패 시 install 자체는 계속 — 사용자가 수동 build 로 회복 가능. 컨테이너가
    꺼져있으면 조용히 skip (entrypoint 가 다음 부팅 때 빌드).
    """
    import subprocess as _sp
    import re

    project = _get_project_root()
    pkg_root = project / "ros2" / "ros2_ws" / "src" / module_id
    if not pkg_root.is_dir():
        return

    # package.xml 들에서 <name> 태그를 추출 (간단한 regex — XML 파서까진 과함)
    pkg_names = []
    for pkg_xml in pkg_root.rglob("package.xml"):
        try:
            text = pkg_xml.read_text(encoding="utf-8", errors="ignore")
            m = re.search(r"<name>\s*([^<\s]+)\s*</name>", text)
            if m:
                pkg_names.append(m.group(1))
        except OSError:
            continue
    if not pkg_names:
        return

    try:
        result = _sp.run(
            ["docker", "inspect", "-f", "{{.State.Running}}", _ROS2_CONTAINER],
            capture_output=True, text=True, timeout=5,
        )
        if "true" not in result.stdout.lower():
            return
    except Exception:
        return

    # colcon build 는 `--packages-select` 로 변경분만 빌드하면 빠름. 컨테이너 안에서
    # ros 환경 source 후 ros2_ws 에서 build. 결과 setup.bash 는 entrypoint 의 source
    # 시점이 지났더라도 새 노드를 띄울 때 ros2 launch 가 알아서 install/ 을 본다.
    cmd = (
        "set -e; "
        "source /opt/ros/humble/setup.bash; "
        "cd /root/ros2_ws; "
        "colcon build --symlink-install "
        "--packages-select " + " ".join(pkg_names) + " "
        "--cmake-args -DCMAKE_BUILD_TYPE=Release"
    )
    try:
        _sp.run(
            ["docker", "exec", _ROS2_CONTAINER, "bash", "-c", cmd],
            timeout=600, check=False,
        )
    except Exception as e:
        print(f"[MODULE] {module_id}: colcon build failed (non-fatal): {e}")


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


# ---------------------------------------------------------------------------
# post_install.credentials — generic mechanism for modules that need an
# external secret (HF token, API key, license file, etc.) at runtime.
#
# Schema (in module.json):
#   "post_install": {
#     "credentials": [
#       {
#         "id": "hf_token",
#         "title": "HuggingFace 액세스 토큰",
#         "description": "facebook/sam3 gated repo 접근에 필요합니다.",
#         "instructions": [
#           {"label": "1. 라이선스 동의", "url": "https://huggingface.co/facebook/sam3"},
#           {"label": "2. 토큰 발급",     "url": "https://huggingface.co/settings/tokens"}
#         ],
#         "input": {"type": "secret", "placeholder": "hf_xxxx", "validate": "^hf_.{20,}$"},
#         "save_to": "$EASYTRAINER_DATA_DIR/.hf_token",
#         "save_format": "raw",   // "raw" | "json:<key>" | "env:<NAME>"
#         "skippable": true
#       }
#     ]
#   }
#
# The launcher UI fetches `pending_credentials_for(module_id)` after a
# successful install — anything whose `save_to` file is missing/empty becomes
# a dialog prompt. `save_credential(spec, value)` writes the value with 0600
# perms. Modules without a `post_install.credentials` block are unaffected.
# ---------------------------------------------------------------------------

def _expand_credential_path(raw_path: str) -> Path:
    data_dir = os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer")
    expanded = (raw_path or "").replace("$EASYTRAINER_DATA_DIR", data_dir)
    expanded = os.path.expandvars(expanded)
    expanded = os.path.expanduser(expanded)
    return Path(expanded)


def _credential_already_satisfied(spec: dict) -> bool:
    """True if the file backing this credential exists and has content."""
    save_to = spec.get("save_to")
    if not save_to:
        return False
    p = _expand_credential_path(save_to)
    try:
        return p.is_file() and p.stat().st_size > 0
    except OSError:
        return False


def pending_credentials_for(module_id: str) -> list[dict]:
    """Return credential specs from module.json that aren't yet satisfied.

    Reads the saved manifest at project/modules/<id>.json so this works for
    modules installed in any past session. Returns [] for modules without
    a credentials block. `post_install` is allowed to be a script-path string
    (legacy) or a dict — only the dict form may carry credentials."""
    meta = _load_module_manifest(module_id)
    if not meta:
        return []
    pi = meta.get("post_install")
    if not isinstance(pi, dict):
        return []
    creds = pi.get("credentials") or []
    if not isinstance(creds, list):
        return []
    return [c for c in creds if isinstance(c, dict) and not _credential_already_satisfied(c)]


def save_credential(spec: dict, value: str) -> bool:
    """Persist `value` to spec.save_to using spec.save_format. Returns ok."""
    save_to = spec.get("save_to")
    if not save_to:
        return False
    target = _expand_credential_path(save_to)
    fmt = spec.get("save_format", "raw")

    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        if fmt == "raw":
            target.write_text(str(value).strip() + "\n", encoding="utf-8")
        elif fmt.startswith("json:"):
            key = fmt.split(":", 1)[1] or spec.get("id", "value")
            existing = {}
            if target.is_file():
                try:
                    existing = json.loads(target.read_text(encoding="utf-8")) or {}
                except Exception:
                    existing = {}
            existing[key] = str(value).strip()
            target.write_text(json.dumps(existing, indent=2), encoding="utf-8")
        elif fmt.startswith("env:"):
            name = fmt.split(":", 1)[1] or spec.get("id", "VALUE")
            target.write_text(f"{name}={str(value).strip()}\n", encoding="utf-8")
        else:
            target.write_text(str(value).strip() + "\n", encoding="utf-8")
        try:
            target.chmod(0o600)
        except OSError:
            pass
        return True
    except Exception as e:
        print(f"[MODULE] save_credential failed for {spec.get('id')}: {e}")
        return False


def _run_module_script(module_id: str, meta: dict, script_key: str) -> bool:
    """Run a module's lifecycle script (post_install, start_command, stop_command).

    Returns True if executed (or no script defined), False on error.
    """
    import subprocess as _sp

    raw = meta.get(script_key)
    if not raw:
        return True
    # post_install may be a string ("scripts/post_install.sh") or a dict
    # ({"script": "...", "credentials": [...]}). Pull the script path out
    # of either shape; missing → nothing to run, return True.
    if isinstance(raw, dict):
        script_rel = raw.get("script")
    else:
        script_rel = raw
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

        # robot_sdk/ 에서도 삭제 — module_id 기준 폴더 (install 코드와 동일 정책)
        # 옛 install (vendor 폴더명) 도 함께 정리해야 leftover 안 남음
        sdk_dir = project / "ros2" / "robot_sdk"
        sdk_targets_rel = [f"ros2/robot_sdk/{module_id}"]
        if manifest:
            legacy_target = manifest.get("install", {}).get("sdk", {}).get("target", "")
            if legacy_target and legacy_target not in sdk_targets_rel:
                sdk_targets_rel.append(legacy_target)
        _log = f"[MODULE REMOVE] {module_id}: cleaning sdk targets {sdk_targets_rel}\n"
        for sdk_target in sdk_targets_rel:
            sdk_path = project / sdk_target
            if not sdk_path.is_dir():
                continue
            try:
                shutil.rmtree(sdk_path)
            except PermissionError:
                import subprocess as _sp
                container_path = sdk_target.replace("ros2/robot_sdk", "/root/robot_sdk")
                _sp.run(["docker", "exec", _ROS2_CONTAINER, "rm", "-rf", container_path],
                        check=False, timeout=10)
                if sdk_path.is_dir():
                    _sp.run(["sudo", "rm", "-rf", str(sdk_path)], check=False, timeout=10)
            _log += f"  removed {sdk_path}\n"
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

    # 이미지 rebuild 대상은 manifest 가 사라지기 전에 미리 산출.
    targets_to_rebuild: set[str] = set()
    if manifest:
        targets_to_rebuild = _affected_targets(manifest)

    # Remove manifest from project/modules/ (rebuild_images.sh 가 이 디렉터리를
    # 읽어 새 이미지를 만들기 때문에 반드시 rebuild 전에 제거)
    _remove_module_manifest(module_id)

    # Legacy modules dir
    legacy_dir = Path(os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer")) / "modules" / module_id
    if legacy_dir.exists():
        shutil.rmtree(legacy_dir, ignore_errors=True)

    set_module_installed(module_id, False)

    # 이 모듈의 apt/pip deps 가 이미지에 baked 되어 있던 경우 → 새 이미지(=해당
    # 모듈 deps 제외) 로 리빌드 + 컨테이너 재시작. 실패하면 폴백으로 pip uninstall
    # 만 수행 (apt 는 다른 모듈이 공유할 수 있어 건드리지 않는 게 기본 정책).
    if targets_to_rebuild:
        rebuilt = _rebuild_and_restart(targets_to_rebuild)
        if not rebuilt and manifest:
            print("[MODULE] image rebuild failed during uninstall — running legacy pip uninstall as fallback.")
            _uninstall_deps(manifest)
    elif manifest:
        # deps 가 없으면 굳이 rebuild 할 필요 없음. legacy 정리만.
        _uninstall_deps(manifest)

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

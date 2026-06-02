from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from urllib import request
from urllib.error import HTTPError, URLError

from app_context import (
    APP_HOME,
    APP_VERSION,
    DEFAULT_PROJECT_PATH,
    MARKER_FILE,
    QApplication,
    QProcess,
    QTimer,
    load_config,
    save_config,
)
try:
    from PySide6.QtCore import QObject, Signal, Slot
except Exception:
    from PyQt6.QtCore import QObject, pyqtSignal as Signal, pyqtSlot as Slot

DEFAULT_REPO_URL = "https://github.com/Plan-99/Easy-Collector-Release.git"
DEFAULT_UPDATE_DIR = "/opt/easytrainer/update"
REPO_ENV = "EASYTRAINER_UPDATE_REPO"
UPDATE_DIR_ENV = "EASYTRAINER_UPDATE_DIR"
CONFIG_REPO_KEY = "update_repo_url"
CONFIG_SKIP_KEY = "update_skip_version"
CONFIG_LAST_APPLIED_KEY = "update_last_applied_version"
CONFIG_UPDATE_DIR_KEY = "update_dir"
CONFIG_UPGRADE_KEY = "update_upgrade_pending"
LAUNCHER_BIN_NAME = "EasyLauncher"

_VERSION_RE = re.compile(r"(\d+)")
_GITHUB_API = "https://api.github.com/repos/{slug}/releases/latest"


class _UiDispatcher(QObject):
    invoke = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.invoke.connect(self._on_invoke)

    def call(self, fn):
        self.invoke.emit(fn)

    @Slot(object)
    def _on_invoke(self, fn):
        try:
            fn()
        except Exception:
            pass


@dataclass(frozen=True)
class UpdateInfo:
    version: str
    version_tuple: tuple[int, int, int]
    deb_name: str
    deb_url: str
    is_major_upgrade: bool


def _parse_version(raw: str | None) -> tuple[int, int, int] | None:
    if not raw:
        return None
    raw = raw.strip()
    if raw.startswith(("v", "V")):
        raw = raw[1:]
    parts = [int(p) for p in _VERSION_RE.findall(raw)]
    if not parts:
        return None
    parts = parts[:3]
    while len(parts) < 3:
        parts.append(0)
    return tuple(parts)  # type: ignore[return-value]


def _version_to_str(ver: tuple[int, int, int]) -> str:
    return f"{ver[0]}.{ver[1]}.{ver[2]}"


def _format_version_display(raw: str | None) -> str:
    if not raw:
        return "?"
    value = str(raw).strip()
    if not value:
        return "?"
    if value.lower().startswith("v"):
        return value
    return f"v{value}"


def _read_app_version_file() -> str | None:
    path = APP_HOME / "VERSION"
    try:
        if path.is_file():
            value = path.read_text(encoding="utf-8").strip()
            return value or None
    except Exception:
        return None
    return None


def _repo_slug_from_url(url: str | None) -> str | None:
    if not url:
        return None
    raw = url.strip()
    if raw.endswith(".git"):
        raw = raw[:-4]
    if raw.startswith("git@"):
        try:
            host_part, path_part = raw.split(":", 1)
            if "github.com" not in host_part:
                return None
            path = path_part.strip("/")
            pieces = path.split("/")
            if len(pieces) < 2:
                return None
            return f"{pieces[0]}/{pieces[1]}"
        except Exception:
            return None
    if "github.com" not in raw:
        return None
    if not raw.startswith(("http://", "https://")):
        raw = "https://" + raw
    try:
        from urllib.parse import urlparse

        parsed = urlparse(raw)
        path = (parsed.path or "").strip("/")
        pieces = path.split("/")
        if len(pieces) < 2:
            return None
        return f"{pieces[0]}/{pieces[1]}"
    except Exception:
        return None


def _select_deb_asset(assets: list[dict]) -> dict | None:
    candidates = []
    for asset in assets or []:
        name = str(asset.get("name") or "")
        if name.lower().endswith(".deb"):
            candidates.append(asset)
    if not candidates:
        return None
    for needle in ("amd64", "x86_64"):
        for asset in candidates:
            name = str(asset.get("name") or "").lower()
            if needle in name:
                return asset
    return candidates[0]


def _is_newer(latest: tuple[int, int, int], current: tuple[int, int, int]) -> bool:
    return latest > current


class UpdateManager:
    def __init__(self, window):
        self._window = window
        self._checked = False
        self._in_progress = False
        self._prompt_open = False
        self._continue_handler = None
        self._continue_called = False
        self._from_version: str | None = None
        self._dispatcher = _UiDispatcher(self._window)

    def schedule_check(self, delay_ms: int = 2500):
        if self._checked:
            return
        self._checked = True
        if delay_ms <= 0:
            self.check_for_updates()
            return
        try:
            QTimer.singleShot(delay_ms, self.check_for_updates)
        except Exception:
            pass

    def set_continue_handler(self, handler):
        self._continue_handler = handler

    def _continue(self):
        if self._continue_called:
            return
        self._continue_called = True
        if not self._continue_handler:
            return
        def _go():
            try:
                self._continue_handler()
            except Exception:
                pass
        self._ui_call(_go)

    def check_for_updates(self):
        if self._in_progress:
            return
        try:
            if not self._window.is_installed():
                return
        except Exception:
            return
        thread = threading.Thread(target=self._check_worker, daemon=True)
        thread.start()

    def _check_worker(self):
        repo_url = self._resolve_repo_url()
        slug = _repo_slug_from_url(repo_url)
        if not slug:
            self._log(f"[UPDATE][WARN] GitHub repo URL이 잘못되었습니다: {repo_url}")
            self._continue()
            return
        self._ui_call(lambda: self._show_check_panel(slug))
        self._log(f"[UPDATE] 최신 릴리즈 확인 중... ({slug})")
        release = self._fetch_latest_release(slug)
        if not release:
            self._ui_call(lambda: self._handle_no_update())
            return
        version_tuple, version_str = self._resolve_release_version(release)
        if not version_tuple:
            self._log("[UPDATE][WARN] 릴리즈 버전을 파싱하지 못했습니다.")
            self._ui_call(lambda: self._handle_no_update())
            return
        asset = _select_deb_asset(release.get("assets", []))
        if not asset:
            self._log("[UPDATE][WARN] 릴리즈에 deb 파일이 없습니다.")
            self._ui_call(lambda: self._handle_no_update())
            return
        current_version = self._current_version()
        current_tuple = _parse_version(current_version) or (0, 0, 0)
        if not _is_newer(version_tuple, current_tuple):
            self._log(f"[UPDATE] 최신 버전입니다. (현재 {current_version}, 최신 {version_str})")
            self._ui_call(lambda: self._handle_no_update())
            return
        cfg = load_config()
        skip_version = cfg.get(CONFIG_SKIP_KEY)
        skip_tuple = _parse_version(skip_version)
        if skip_tuple and skip_tuple == version_tuple:
            self._log(f"[UPDATE] 건너뛰기 버전({skip_version})과 동일하여 생략합니다.")
            self._ui_call(lambda: self._handle_no_update())
            return
        info = UpdateInfo(
            version=version_str,
            version_tuple=version_tuple,
            deb_name=str(asset.get("name") or "easytrainer_update.deb"),
            deb_url=str(asset.get("browser_download_url") or ""),
            is_major_upgrade=version_tuple[0] > current_tuple[0],
        )
        if not info.deb_url:
            self._log("[UPDATE][WARN] deb 다운로드 URL을 찾지 못했습니다.")
            self._ui_call(lambda: self._handle_no_update())
            return
        self._log(f"[UPDATE] 새 버전 발견: {info.version}")
        self._ui_call(lambda: self._prompt_update(info))

    def _resolve_repo_url(self) -> str:
        cfg = load_config()
        return (
            os.environ.get(REPO_ENV)
            or cfg.get(CONFIG_REPO_KEY)
            or DEFAULT_REPO_URL
        )

    def _resolve_update_dir(self) -> Path:
        cfg = load_config()
        raw = (
            os.environ.get(UPDATE_DIR_ENV)
            or cfg.get(CONFIG_UPDATE_DIR_KEY)
            or DEFAULT_UPDATE_DIR
        )
        return Path(str(raw)).expanduser()

    def _fetch_latest_release(self, slug: str) -> dict | None:
        url = _GITHUB_API.format(slug=slug)
        req = request.Request(url, headers={"User-Agent": "EasyTrainer"})
        try:
            with request.urlopen(req, timeout=8) as resp:
                payload = resp.read().decode("utf-8")
            return json.loads(payload or "{}")
        except (HTTPError, URLError, TimeoutError) as e:
            self._log(f"[UPDATE][WARN] 릴리즈 확인 실패: {e}")
            return None
        except Exception as e:
            self._log(f"[UPDATE][WARN] 릴리즈 응답 처리 실패: {e}")
            return None

    def _resolve_release_version(self, release: dict) -> tuple[tuple[int, int, int] | None, str]:
        for key in ("tag_name", "name"):
            raw = release.get(key) or ""
            parsed = _parse_version(str(raw))
            if parsed:
                return parsed, _version_to_str(parsed)
        return None, ""

    def _prompt_update(self, info: UpdateInfo):
        if self._in_progress or self._prompt_open:
            return
        self._prompt_open = True
        current_version = self._current_version()
        if info.is_major_upgrade:
            detail = "버전 업그레이드가 예정되어있어서 약 30분의 시간이 소요됩니다."
        else:
            detail = "버그 수정 및 개선을 위한 버전 업데이트가 예정되어있습니다."
        version_summary = f"{_format_version_display(current_version)} -> {_format_version_display(info.version)}"
        detail = f"{version_summary}\n{detail}"

        def _on_skip():
            cfg = load_config()
            cfg[CONFIG_SKIP_KEY] = info.version
            save_config(cfg)
            self._log(f"[UPDATE] {info.version} 버전을 건너뜁니다.")
            self._prompt_open = False
            self._handle_no_update()

        def _on_later():
            self._prompt_open = False
            self._handle_no_update()

        def _on_install():
            self._prompt_open = False
            self._start_update(info)

        try:
            self._window.show_update_prompt_panel(
                current_version,
                info.version,
                detail,
                _on_install,
                _on_skip,
                _on_later,
            )
        except Exception:
            self._prompt_open = False
            self._handle_no_update()

    def _show_check_panel(self, slug: str):
        try:
            self._window.show_update_check_panel(f"{slug} 최신 릴리즈 확인 중...")
        except Exception:
            pass

    def _handle_no_update(self):
        try:
            self._window.hide_update_panel()
        except Exception:
            pass
        self._continue()

    def _start_update(self, info: UpdateInfo):
        if self._in_progress:
            return
        self._in_progress = True
        self._from_version = self._current_version()
        self._show_progress("업데이트를 준비하는 중입니다...")
        thread = threading.Thread(target=self._apply_update_worker, args=(info,), daemon=True)
        thread.start()

    def _apply_update_worker(self, info: UpdateInfo):
        try:
            update_dir = self._resolve_update_dir()
            self._ensure_update_dir(update_dir)
            self._clear_update_dir(update_dir)
            deb_path = update_dir / info.deb_name
            self._set_progress_text("업데이트 파일을 다운로드하는 중입니다...")
            def _download_progress(percent: int | None):
                text = f"다운로드 {percent}%" if isinstance(percent, int) else None
                self._ui_call(lambda: self._window.set_update_progress(percent, text))
            self._download_file(info.deb_url, deb_path, progress_cb=_download_progress)
            extract_root = update_dir / "extract"
            self._set_progress_text("업데이트 파일을 압축 해제하는 중입니다...")
            self._extract_deb(deb_path, extract_root)
            payload_root = extract_root / "usr" / "share" / "easytrainer-project"
            self._set_progress_text("코드를 동기화하는 중입니다...")
            self._sync_payload(payload_root)
            try:
                self._window._apply_compose_variant(self._window.install_variant)
            except Exception:
                pass
            self._set_progress_text("런처를 업데이트하는 중입니다...")
            self._sync_launcher_assets(extract_root)
            self._set_progress_text("마무리하는 중입니다...")
            self._update_app_assets(extract_root)
            cfg = load_config()
            cfg[CONFIG_LAST_APPLIED_KEY] = info.version
            if cfg.get(CONFIG_SKIP_KEY) == info.version:
                cfg.pop(CONFIG_SKIP_KEY, None)
            save_config(cfg)
            self._set_progress_text("버전 정보를 갱신하는 중입니다...")
            self._update_version_file(extract_root, info.version)
            self._ui_call(lambda: self._finish_update(success=True, info=info))
        except Exception as e:
            self._ui_call(lambda: self._finish_update(success=False, info=info, error=str(e)))

    def _ensure_update_dir(self, path: Path):
        try:
            path.mkdir(parents=True, exist_ok=True)
            test_file = path / ".ec_write_test"
            test_file.write_text("ok")
            test_file.unlink()
            return
        except Exception:
            pass
        try:
            fixer = getattr(self._window, "_run_pkexec_chown", None)
            if callable(fixer) and fixer(path):
                path.mkdir(parents=True, exist_ok=True)
                return
        except Exception:
            pass
        raise RuntimeError(f"업데이트 폴더를 생성할 수 없습니다: {path}")

    def _clear_update_dir(self, path: Path):
        resolved = path.resolve()
        if not resolved.is_absolute() or len(resolved.parts) < 3:
            raise RuntimeError(f"업데이트 폴더 경로가 안전하지 않습니다: {resolved}")
        if not resolved.exists():
            resolved.mkdir(parents=True, exist_ok=True)
            return
        for item in resolved.iterdir():
            try:
                if item.is_dir():
                    shutil.rmtree(item)
                else:
                    item.unlink()
            except Exception:
                continue

    def _download_file(self, url: str, dest: Path, progress_cb=None,
                       connect_timeout: int = 30, stall_timeout: int = 60):
        """Download `url` -> `dest` with a stall watchdog so a dead or trickling
        connection can never hang the updater indefinitely. If no new bytes
        arrive for `stall_timeout` seconds, the socket is force-closed and a
        clear error is raised — the caller turns that into a failed update with
        a message instead of an endless spinner. `connect_timeout` is the
        per-socket read timeout (a hard stall trips this first)."""
        import socket
        import threading
        import time as _time
        tmp = dest.with_suffix(dest.suffix + ".part")
        req = request.Request(url, headers={"User-Agent": "EasyTrainer"})
        # The socket timeout is the PRIMARY stall guard: it stays on the socket
        # for every recv(), so a dead/trickle-free connection raises
        # socket.timeout after stall_timeout on its own — no thread needed to
        # interrupt the blocked read. (resp.close() from another thread does NOT
        # unblock an in-progress recv on CPython; only the socket timeout or a
        # shutdown() does.) connect_timeout bounds the initial connect.
        sock_timeout = max(1, min(connect_timeout, stall_timeout))
        resp = request.urlopen(req, timeout=sock_timeout)
        last_data = [_time.monotonic()]
        stalled = [False]
        stop_watch = threading.Event()

        # Backstop watchdog: shutdown() (unlike close()) DOES interrupt a recv
        # blocked in another thread, covering the rare case the socket timeout
        # doesn't fire (e.g. a slow trickle that resets the per-recv timer).
        raw_sock = None
        try:
            raw_sock = resp.fp.raw._sock  # type: ignore[attr-defined]
        except Exception:
            raw_sock = None

        def _watchdog():
            while not stop_watch.wait(1.0):
                if _time.monotonic() - last_data[0] > stall_timeout:
                    stalled[0] = True
                    for fn in (lambda: raw_sock.shutdown(socket.SHUT_RDWR), resp.close):
                        try:
                            fn()
                        except Exception:
                            pass
                    return

        watcher = threading.Thread(target=_watchdog, daemon=True)
        watcher.start()
        try:
            total = None
            try:
                total = int(resp.getheader("Content-Length") or 0)
            except Exception:
                total = None
            if progress_cb:
                try:
                    progress_cb(0 if total else None)
                except Exception:
                    pass
            downloaded = 0
            last_percent = -1
            with tmp.open("wb") as f:
                while True:
                    try:
                        chunk = resp.read(1024 * 1024)
                    except Exception as e:
                        # socket.timeout (primary guard) or the watchdog's
                        # shutdown both surface here → report as a stall.
                        if stalled[0] or isinstance(e, (socket.timeout, TimeoutError)):
                            raise RuntimeError(
                                f"다운로드가 {stall_timeout}초 동안 진행되지 않아 중단했습니다. "
                                f"네트워크 연결을 확인한 뒤 다시 시도하세요."
                            )
                        raise
                    if not chunk:
                        # An empty read can be a genuine EOF OR the watchdog's
                        # shutdown() forcing recv() to return 0 mid-download —
                        # the latter must surface as a stall, not "complete".
                        if stalled[0]:
                            raise RuntimeError(
                                f"다운로드가 {stall_timeout}초 동안 진행되지 않아 중단했습니다. "
                                f"네트워크 연결을 확인한 뒤 다시 시도하세요."
                            )
                        break
                    last_data[0] = _time.monotonic()
                    f.write(chunk)
                    if total and progress_cb:
                        downloaded += len(chunk)
                        percent = int((downloaded / total) * 100)
                        if percent != last_percent:
                            last_percent = percent
                            try:
                                progress_cb(min(percent, 100))
                            except Exception:
                                pass
        finally:
            stop_watch.set()
            try:
                resp.close()
            except Exception:
                pass
        # Guard against silent truncation: a short file vs the advertised size
        # means the connection dropped mid-download — don't install a partial deb.
        if total and downloaded < total:
            raise RuntimeError(
                f"다운로드가 중간에 끊겼습니다 ({downloaded}/{total} 바이트). "
                f"네트워크 연결을 확인한 뒤 다시 시도하세요."
            )
        tmp.replace(dest)
        if progress_cb and total:
            try:
                progress_cb(100)
            except Exception:
                pass

    def _extract_deb(self, deb_path: Path, extract_root: Path):
        if extract_root.exists():
            shutil.rmtree(extract_root)
        extract_root.mkdir(parents=True, exist_ok=True)
        dpkg = shutil.which("dpkg-deb")
        if dpkg:
            # timeout so a wedged extraction can't hang the update forever.
            result = subprocess.run([dpkg, "-x", str(deb_path), str(extract_root)],
                                    capture_output=True, text=True, timeout=600)
            if result.returncode == 0:
                return
            self._log(f"[UPDATE][WARN] dpkg-deb 실패: {result.stderr.strip()}")
        ar_bin = shutil.which("ar")
        tar_bin = shutil.which("tar")
        if not ar_bin or not tar_bin:
            raise RuntimeError("deb 압축 해제 도구(dpkg-deb/ar/tar)를 찾지 못했습니다.")
        tmp_dir = extract_root / ".ar"
        tmp_dir.mkdir(parents=True, exist_ok=True)
        subprocess.run([ar_bin, "x", str(deb_path)], cwd=str(tmp_dir), check=True, timeout=600)
        data_tar = None
        for cand in tmp_dir.glob("data.tar.*"):
            data_tar = cand
            break
        if not data_tar:
            raise RuntimeError("deb payload를 찾을 수 없습니다.")
        subprocess.run([tar_bin, "-xf", str(data_tar), "-C", str(extract_root)], check=True, timeout=600)

    def _sync_payload(self, payload_root: Path):
        if not payload_root.exists():
            raise RuntimeError(f"payload 경로가 없습니다: {payload_root}")
        script = None
        try:
            script = self._window._resolve_quick_apply_script()
        except Exception:
            script = None
        if script and script.is_file():
            # timeout so a stuck rsync/copy can't hang the update indefinitely.
            result = subprocess.run(
                [str(script), str(payload_root), str(self._window.project_root)],
                capture_output=True,
                text=True,
                timeout=900,
            )
            self._log_output(result)
            if result.returncode == 0:
                return
            self._log("[UPDATE][WARN] quick_apply 실패, 내부 동기화로 전환합니다.")
        self._fallback_sync(payload_root, self._window.project_root)

    def _fallback_sync(self, src_root: Path, dst_root: Path):
        self._sync_tree(src_root / "backend", dst_root / "backend", skip_db=True)
        self._sync_tree(src_root / "frontend", dst_root / "frontend", skip_db=False)
        # ros2/ros2_ws/src and backend/modules are managed by module installer — do not sync
        self._copy_core_files(src_root, dst_root)
        self._sync_scripts(src_root, APP_HOME / "scripts")

    def _sync_tree(self, src: Path, dst: Path, skip_db: bool):
        if not src.exists():
            return
        for root, dirs, files in os.walk(src):
            root_path = Path(root)
            rel = root_path.relative_to(src)
            if "__pycache__" in rel.parts or "node_modules" in rel.parts:
                dirs[:] = []
                continue
            target_dir = dst / rel
            target_dir.mkdir(parents=True, exist_ok=True)
            dirs[:] = [d for d in dirs if d not in ("__pycache__", "node_modules")]
            for name in files:
                if name.endswith(".pyc"):
                    continue
                rel_file = rel / name
                if skip_db and rel.parts and rel.parts[0] == "database" and name.endswith(".db"):
                    continue
                src_file = root_path / name
                dst_file = dst / rel_file
                dst_file.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src_file, dst_file)

    def _copy_core_files(self, src_root: Path, dst_root: Path):
        files = [
            "docker-compose.yml",
            "docker-compose.dev.yml",
            "docker-compose.cpu.yml",
            "docker-compose.gpu.yml",
            "start_services.sh",
            "src/kill.sh",
            "Dockerfile",
            ".dockerignore",
            "requirements.txt",
            "requirements.min.txt",
        ]
        for name in files:
            src = src_root / name
            if not src.exists():
                continue
            dst = dst_root / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)

    def _sync_scripts(self, src_root: Path, dst_root: Path):
        src = src_root / "scripts"
        if not src.exists():
            return
        dst_root.mkdir(parents=True, exist_ok=True)
        shutil.copytree(
            src,
            dst_root,
            dirs_exist_ok=True,
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
        )

    def _sync_ui(self, ui_root: Path):
        if not ui_root.exists():
            return
        dst = APP_HOME / "ui"
        dst.mkdir(parents=True, exist_ok=True)
        shutil.copytree(
            ui_root,
            dst,
            dirs_exist_ok=True,
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
        )

    def _sync_launcher_assets(self, extract_root: Path):
        bin_src = extract_root / "opt" / "easytrainer" / LAUNCHER_BIN_NAME
        if bin_src.is_file():
            self._sync_launcher_bin(bin_src)
            self._ensure_legacy_launcher_stub()
            return
        ui_root = extract_root / "opt" / "easytrainer" / "ui"
        if ui_root.exists():
            self._sync_ui(ui_root)
            return
        self._log("[UPDATE][WARN] 런처 업데이트 대상이 없습니다.")

    def _sync_launcher_bin(self, bin_src: Path):
        dst = APP_HOME / LAUNCHER_BIN_NAME
        dst.parent.mkdir(parents=True, exist_ok=True)
        tmp = dst.parent / f"{dst.name}.new"
        shutil.copy2(bin_src, tmp)
        try:
            tmp.chmod(0o755)
        except Exception:
            pass
        tmp.replace(dst)

    def _ensure_legacy_launcher_stub(self):
        ui_root = APP_HOME / "ui"
        ui_root.mkdir(parents=True, exist_ok=True)
        stub_path = ui_root / "main.py"
        stub = (
            "#!/usr/bin/env python3\n"
            "import os\n"
            "import sys\n"
            "from pathlib import Path\n"
            "\n"
            "def main():\n"
            "    app_home = Path(__file__).resolve().parents[1]\n"
            f"    launcher = app_home / \"{LAUNCHER_BIN_NAME}\"\n"
            "    if launcher.is_file():\n"
            "        os.execv(str(launcher), [str(launcher)] + sys.argv[1:])\n"
            "    print(f\"EasyLauncher binary not found: {launcher}\", file=sys.stderr)\n"
            "    return 1\n"
            "\n"
            "if __name__ == \"__main__\":\n"
            "    raise SystemExit(main())\n"
        )
        try:
            stub_path.write_text(stub, encoding="utf-8")
        except Exception:
            pass

    def _update_version_file(self, extract_root: Path, version: str):
        src_candidates = [
            extract_root / "opt" / "easytrainer" / "VERSION",
            extract_root / "usr" / "share" / "easytrainer-project" / "VERSION",
        ]
        dst = APP_HOME / "VERSION"
        try:
            copied = False
            for src in src_candidates:
                if src.exists():
                    dst.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(src, dst)
                    copied = True
                    break
            if copied:
                return
        except Exception:
            pass
        try:
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_text(version, encoding="utf-8")
        except Exception:
            pass

    def _update_app_assets(self, extract_root: Path):
        icon_src = extract_root / "opt" / "easytrainer" / "app_icon.png"
        icon_dst = APP_HOME / "app_icon.png"
        if icon_src.exists():
            try:
                shutil.copy2(icon_src, icon_dst)
            except Exception:
                pass

    def _finish_update(self, success: bool, info: UpdateInfo, error: str | None = None):
        self._in_progress = False
        from_version = self._from_version
        self._from_version = None
        if not success:
            detail = f"업데이트에 실패했습니다:\n{error or '알 수 없는 오류'}"
            def _continue_after_error():
                try:
                    self._window.hide_update_panel()
                except Exception:
                    pass
                self._continue()
            try:
                self._window.show_update_error_panel(detail, _continue_after_error, _continue_after_error)
            except Exception:
                pass
            return
        detail = "업데이트가 완료되었습니다.\n필요 시 프로그램을 다시 실행해 주세요."
        try:
            def _close_panel():
                try:
                    self._window.close()
                except Exception:
                    try:
                        QApplication.instance().quit()
                    except Exception:
                        pass
            self._window.show_update_done_panel(
                detail,
                _close_panel,
                from_version or self._current_version(),
                info.version,
            )
        except Exception:
            pass

    def _current_version(self) -> str:
        version = _read_app_version_file()
        if version:
            return version
        return APP_VERSION or "0.0.0"

    def _restart_application(self, force_upgrade: bool, version: str | None = None):
        if force_upgrade:
            try:
                cfg = load_config()
                cfg[CONFIG_UPGRADE_KEY] = version or cfg.get(CONFIG_LAST_APPLIED_KEY) or APP_VERSION
                save_config(cfg)
            except Exception:
                pass
        try:
            launcher_bin = APP_HOME / LAUNCHER_BIN_NAME
            if launcher_bin.is_file():
                QProcess.startDetached(str(launcher_bin), [])
            elif getattr(sys, "frozen", False):
                QProcess.startDetached(sys.executable, [])
            else:
                main_py = Path(__file__).resolve().parent / "main.py"
                QProcess.startDetached(sys.executable, [str(main_py)])
        except Exception as e:
            QMessageBox.critical(self._window, "재시작 실패", f"프로그램 재시작에 실패했습니다: {e}")
            return
        try:
            setattr(self._window, "_skip_shutdown_on_exit", True)
        except Exception:
            pass
        try:
            QApplication.instance().quit()
        except Exception:
            pass

    def _show_progress(self, text: str):
        try:
            self._window.show_update_progress_panel(text)
        except Exception:
            pass

    def _set_progress_text(self, text: str):
        def _apply():
            try:
                self._window.show_update_progress_panel(text)
            except Exception:
                pass
        self._ui_call(_apply)

    def _hide_progress(self):
        try:
            self._window.hide_update_panel()
        except Exception:
            pass

    def _ui_call(self, fn):
        try:
            self._dispatcher.call(fn)
        except Exception:
            try:
                fn()
            except Exception:
                pass

    def _log_output(self, proc_result):
        try:
            if proc_result.stdout:
                for line in proc_result.stdout.splitlines():
                    self._log(f"[UPDATE][OUT] {line}")
            if proc_result.stderr:
                for line in proc_result.stderr.splitlines():
                    self._log(f"[UPDATE][ERR] {line}")
        except Exception:
            pass

    def _log(self, msg: str):
        try:
            self._window.append_log(msg)
        except Exception:
            pass
        try:
            print(msg, flush=True)
        except Exception:
            pass

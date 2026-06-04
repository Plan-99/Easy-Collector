from __future__ import annotations

import os
import pwd
import grp
import shlex
import shutil
import subprocess
from pathlib import Path

from app_context import (
    APP_HOME,
    APP_VERSION,
    CONFIG_FILE,
    FRONTEND_URL,
    MARKER_FILE,
    REPO_ROOT_CANDIDATE,
    SYSTEM_PAYLOAD_DIR,
    QDialog,
    QMessageBox,
    QPlainTextEdit,
    QProcess,
    QTimer,
    QTextEdit,
    QVBoxLayout,
    QInputDialog,
    QLineEdit,
    load_config,
    save_config,
)

import device_auth

from i18n import t


def ensure_signed_in() -> bool:
    """Ensure the user has a valid Google-OAuth Bearer token saved locally.

    Returns True on success, False if the user cancels or auth fails terminally.
    """
    return device_auth.ensure_signed_in_gui()



def docker_compose_available() -> bool:
    """Detect docker compose capability (prefer v2 plugin)."""
    # Prefer docker CLI with compose plugin (v2)
    if shutil.which("docker"):
        return True
    # Fallback to legacy docker-compose v1 binary
    return shutil.which("docker-compose") is not None


def grant_local_x11_access() -> None:
    """Allow the docker container's root user to talk to the host X server.

    MuJoCo viewer / RViz / Quasar PWA(Webview) 등 호스트 디스플레이를 쓰는 GUI는
    컨테이너 내부 root가 X 서버에 붙을 수 있어야 한다. xhost ACL은 X 세션 동안만
    유지되므로 런처 시작 시마다 한 번씩 호출해 두는 게 안전.

    Idempotent — 이미 추가돼 있어도 무해. headless / DISPLAY 미설정 / xhost 미설치
    환경에서는 조용히 skip하므로 서버 환경에서도 문제 없음.
    """
    if not os.environ.get("DISPLAY"):
        return
    xhost = shutil.which("xhost")
    if not xhost:
        return
    try:
        subprocess.run(
            [xhost, "+SI:localuser:root"],
            check=False, timeout=5,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except Exception:
        # X 서버 미가동 / 권한 부족 등 — 호출자(GUI 시작 단계) 입장에선 비치명적.
        pass


def get_compose_cmd():
    """Return (program, prefix_args) for Compose.
    Prefer `docker compose` (v2 plugin); fallback to legacy `docker-compose`.
    """
    docker_bin = shutil.which("docker")
    if docker_bin:
        return docker_bin, ["compose"]
    dc_bin = shutil.which("docker-compose")
    if dc_bin:
        return dc_bin, []
    return None, None


class ComposeServiceMixin:
    def run_compose_blocking(self, args: list[str]) -> int:
        program, prefix = get_compose_cmd()
        if not program:
            return 1
        try:
            res = subprocess.run([program, *prefix, *args], cwd=str(self.project_root), text=True)
            return res.returncode
        except Exception:
            return 1

    def run_compose(self, args: list[str], on_finish=None):
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            return

        program, prefix = get_compose_cmd()
        if not program:
            QMessageBox.critical(self, t("service.errorTitle"), self._compose_help_text())
            return
        file_args: list[str] = []
        self.process = QProcess(self)
        self.process.setProgram(program)
        self.process.setArguments([*prefix, *file_args, *args])
        self.process.setWorkingDirectory(str(self.project_root))

        self.process.readyReadStandardOutput.connect(
            lambda: self._read_stream(self.process, False)
        )
        self.process.readyReadStandardError.connect(
            lambda: self._read_stream(self.process, True)
        )
        if on_finish:
            self.process.finished.connect(on_finish)
        self.process.finished.connect(lambda *_: self.update_state_label())
        self.process.start()

    def _run_compose_sequence(self, commands: list[list[str]], on_finish=None):
        commands = [cmd for cmd in commands if cmd]
        if not commands:
            if on_finish:
                on_finish(0)
            return

        def _step(index: int):
            if index >= len(commands):
                if on_finish:
                    on_finish(0)
                return
            cmd = commands[index]
            def _after(exit_code: int, *_):
                if exit_code != 0:
                    if on_finish:
                        on_finish(exit_code)
                else:
                    _step(index + 1)
            self.run_compose(cmd, on_finish=_after)

        _step(0)

    def run_compose_to_widget(self, args: list[str], log_widget: QTextEdit, on_finish=None):
        proc = QProcess(self)
        program, prefix = get_compose_cmd()
        if not program:
            log_widget.append(self._compose_help_text())
            return
        file_args: list[str] = []
        proc.setProgram(program)
        proc.setArguments([*prefix, *file_args, *args])
        proc.setWorkingDirectory(str(self.project_root))
        def _append(is_err=False):
            try:
                data = proc.readAllStandardError() if is_err else proc.readAllStandardOutput()
                text = bytes(data).decode(errors="ignore")
                if text:
                    log_widget.append(text.rstrip("\n"))
            except Exception:
                pass
        proc.readyReadStandardOutput.connect(lambda: _append(False))
        proc.readyReadStandardError.connect(lambda: _append(True))
        if on_finish:
            proc.finished.connect(on_finish)
        proc.start()

    def run_compose_to_widget_parent(self, parent, args: list[str], log_widget: QTextEdit, on_finish=None):
        proc = QProcess(parent)
        program, prefix = get_compose_cmd()
        if not program:
            log_widget.append(self._compose_help_text())
            return
        file_args: list[str] = []
        proc.setProgram(program)
        proc.setArguments([*prefix, *file_args, *args])
        proc.setWorkingDirectory(str(self.project_root))
        def _append(is_err=False):
            try:
                data = proc.readAllStandardError() if is_err else proc.readAllStandardOutput()
                text = bytes(data).decode(errors="ignore")
                if text:
                    log_widget.append(text.rstrip("\n"))
            except Exception:
                pass
        proc.readyReadStandardOutput.connect(lambda: _append(False))
        proc.readyReadStandardError.connect(lambda: _append(True))
        if on_finish:
            proc.finished.connect(on_finish)
        proc.start()

    def _compose_help_text(self) -> str:
        return t("service.composeHelp")

    def _read_stream(self, proc: QProcess, is_err: bool):
        try:
            data = proc.readAllStandardError() if is_err else proc.readAllStandardOutput()
            text = bytes(data).decode(errors="ignore")
            if text:
                self.append_log(text.rstrip("\n"))
        except Exception:
            pass


class HealthServiceMixin:
    def _start_inline_logs(self):
        # Inline logs removed; keep method to satisfy callers.
        self._stop_inline_log_retry()
        try:
            proc = self._inline_log_proc
            if proc is not None and proc.state() != QProcess.NotRunning:
                proc.terminate()
                proc.waitForFinished(500)
                proc.kill()
        except Exception:
            pass
        self._inline_log_proc = None
        self._inline_log_initialized = False
        return

    def _stop_inline_logs(self):
        self._inline_log_proc = None
        self._stop_inline_log_retry()

    def _stop_inline_log_retry(self):
        if self._inline_log_retry:
            try:
                self._inline_log_retry.stop()
            except Exception:
                pass
            self._inline_log_retry = None

    def _resolve_ready_timeout_ms(self) -> int:
        # Allow multiple env keys; treat small numbers as seconds for convenience
        keys = [
            "EASYTRAINER_READY_TIMEOUT_MS",
            "EASYCOLLECTOR_READY_TIMEOUT_MS",
            "READY_TIMEOUT_MS",
            "timeout_ms",
        ]
        for k in keys:
            val = os.environ.get(k)
            if not val:
                continue
            try:
                num = float(val.strip())
                if num <= 0:
                    continue
                # if <= 300, assume seconds
                return int(num * 1000) if num <= 300 else int(num)
            except Exception:
                continue
        return 120000  # default 120s for slower cold starts

    def _wait_for_services_ready(self, on_ready=None, auto_open_new: bool | None = None):
        if auto_open_new is None:
            auto_open_new = getattr(self, "_preload_auto_open_new", True)
        if on_ready is None and auto_open_new:
            on_ready = self.load_ui
        self._stop_ready_timer()
        self._stop_ready_timeout()
        self._show_preload_dialog(t("service.preparing"), auto_open_new=auto_open_new)

        def poll():
            backend_ok = self._check_backend_ready()
            frontend_ok = self._is_frontend_ready()
            ready = t("service.statusReady")
            waiting = t("service.statusWaiting")
            detail = t(
                "service.readinessDetail",
                frontend=ready if frontend_ok else waiting,
                backend=ready if backend_ok else waiting,
            )
            self._set_preload_detail(detail)
            if backend_ok and frontend_ok:
                self._stop_ready_timer()
                self._stop_ready_timeout()
                self._hide_preload_dialog(ready=True)
                try:
                    if auto_open_new:
                        self.load_ui(open_mode=self._open_ui_mode)
                    elif on_ready is not None:
                        on_ready()
                except Exception:
                    pass

        self._ready_check_timer = QTimer(self)
        self._ready_check_timer.setInterval(1000)
        self._ready_check_timer.timeout.connect(poll)
        poll()
        if self._ready_check_timer is not None:
            self._ready_check_timer.start()

        # Optional timeout if configured
        timeout_ms = self._resolve_ready_timeout_ms()
        if timeout_ms > 0:
            self._ready_timeout_timer = QTimer(self)
            self._ready_timeout_timer.setSingleShot(True)
            self._ready_timeout_timer.setInterval(timeout_ms)
            self._ready_timeout_timer.timeout.connect(self._on_ready_timeout)
            self._ready_timeout_timer.start()

    def _check_backend_ready(self, timeout: float = 1.0) -> bool:
        try:
            from urllib import request
            req = request.Request("http://127.0.0.1:5000/api/healthz", method="GET")
            with request.urlopen(req, timeout=timeout):
                return True
        except Exception:
            return False

    def _is_frontend_up(self, timeout: float = 0.5) -> bool:
        try:
            from urllib.parse import urlparse
            import socket
            u = urlparse(FRONTEND_URL)
            host = u.hostname or "localhost"
            port = u.port or (443 if (u.scheme or "http").lower() == "https" else 80)
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except Exception:
            return False

    def _is_frontend_ready(self, timeout: float = 0.8) -> bool:
        try:
            from urllib import request
            from urllib.parse import urljoin
            import re
            req = request.Request(
                FRONTEND_URL,
                method="GET",
                headers={"Cache-Control": "no-cache"},
            )
            with request.urlopen(req, timeout=timeout) as resp:
                status = getattr(resp, "status", 200)
                if status >= 400:
                    return False
                body = resp.read(65536)
        except Exception:
            return False
        if not body:
            return False
        text = body.decode("utf-8", errors="ignore")
        lower = text.lower()
        if "<html" not in lower:
            return False
        sources: list[str] = []
        for match in re.finditer(r"<script[^>]+src=[\"']([^\"']+)[\"']", text, re.IGNORECASE):
            src = match.group(1).strip()
            if not src or src.startswith("data:"):
                continue
            if src not in sources:
                sources.append(src)
        if not sources:
            if "q-app" in lower or "quasar" in lower or "@vite" in lower or "type=\"module\"" in lower:
                return True
            return len(lower.strip()) > 200
        for src in sources[:2]:
            url = urljoin(FRONTEND_URL, src)
            try:
                req = request.Request(
                    url,
                    method="GET",
                    headers={"Cache-Control": "no-cache", "Range": "bytes=0-1024"},
                )
                with request.urlopen(req, timeout=timeout) as resp:
                    status = getattr(resp, "status", 200)
                    if status >= 400:
                        return False
                    chunk = resp.read(64)
                    if not chunk:
                        return False
            except Exception:
                return False
        return True

    def _is_ui_ready(self, frontend_timeout: float = 0.8, backend_timeout: float = 0.4) -> bool:
        if not self._is_frontend_ready(timeout=frontend_timeout):
            return False
        return self._check_backend_ready(timeout=backend_timeout)

    def _collect_docker_logs(self, container: str, tail: int = 200) -> str:
        try:
            result = subprocess.run(
                ["docker", "logs", f"--tail={tail}", container],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode == 0:
                return result.stdout.strip() or "[docker logs] 로그가 비어 있습니다."
            return f"[docker logs] 실패 (code={result.returncode}): {result.stderr.strip()}"
        except Exception as e:
            return f"[docker logs] 실행 실패: {e}"

    def _show_docker_logs_dialog(self, title: str, log_text: str):
        dlg = QDialog(self)
        dlg.setWindowTitle(title)
        dlg.resize(700, 500)
        viewer = QPlainTextEdit(dlg)
        viewer.setReadOnly(True)
        self._apply_log_font(viewer)
        viewer.setPlainText(log_text)
        layout = QVBoxLayout(dlg)
        layout.addWidget(viewer)
        dlg.show()
        dlg.raise_()
        self._register_log_window(dlg)

    def _show_docker_logs_follow(self, title: str, container: str, tail: int = 200, initial_text: str = "", on_close=None):
        dlg = QDialog(self)
        dlg.setWindowTitle(title)
        dlg.resize(800, 520)
        viewer = QPlainTextEdit(dlg)
        viewer.setReadOnly(True)
        self._apply_log_font(viewer)
        if initial_text:
            viewer.setPlainText(initial_text.strip() + "\n")
        layout = QVBoxLayout(dlg)
        layout.addWidget(viewer)

        proc = QProcess(dlg)
        proc.setProgram("docker")
        proc.setArguments(["logs", "-f", f"--tail={tail}", container])
        proc.setProcessChannelMode(QProcess.MergedChannels)

        def _append():
            try:
                data = proc.readAll()
                text = bytes(data).decode(errors="ignore")
                if text:
                    viewer.moveCursor(viewer.textCursor().End)  # type: ignore
                    viewer.insertPlainText(text)
                    viewer.moveCursor(viewer.textCursor().End)  # type: ignore
            except Exception:
                pass

        proc.readyRead.connect(_append)

        def _on_finish(*_):
            try:
                viewer.append("\n[docker logs] 종료되었습니다.")
            except Exception:
                pass

        proc.finished.connect(_on_finish)

        on_close_called = {"done": False}

        def _cleanup(*_):
            try:
                proc.terminate()
                proc.waitForFinished(2000)
                proc.kill()
            except Exception:
                pass
            if on_close and not on_close_called["done"]:
                on_close_called["done"] = True
                try:
                    on_close()
                except Exception:
                    pass

        dlg.finished.connect(_cleanup)
        dlg.destroyed.connect(_cleanup)
        proc.start()

        dlg.show()
        dlg.raise_()
        self._register_log_window(dlg)

    def _on_ready_timeout(self):
        self._stop_ready_timer()
        self._stop_ready_timeout()
        self._stop_restart_timer()
        self._stop_inline_logs()
        self._hide_preload_dialog()
        self.append_log("[ERROR] 서비스가 제시간에 준비되지 않아 중지합니다.")
        logs = self._collect_docker_logs("easytrainer_backend", tail=200)
        self._show_docker_logs_follow(
            t("service.startFailedTitle"),
            container="easytrainer_backend",
            tail=200,
            initial_text=logs,
        )
        self._enable_restart_button()
        if docker_compose_available() and self._is_valid_project_root(self.project_root):
            self.append_log("[STOP] docker compose stop ...")
            self.run_compose(["stop"], on_finish=self._on_stop_finished)


class RuntimeServiceMixin:
    def is_installed(self) -> bool:
        # Must have marker AND a usable Docker/compose environment with built image
        if not MARKER_FILE.exists():
            return False
        if not docker_compose_available():
            return False
        if not self._is_valid_project_root(self.project_root):
            return False
        # Check that at least one compose image exists
        try:
            return (self._image_exists("easytrainer-backend:latest")
                    or self._image_exists("easytrainer-frontend:latest")
                    or self._image_exists("easytrainer-ros2:latest"))
        except Exception:
            return False

    def _is_valid_project_root(self, path: Path) -> bool:
        try:
            if not path or not path.is_dir():
                return False
            required = [
                path / "docker-compose.yml",
                path / "backend" / "Dockerfile",
                path / "frontend" / "Dockerfile",
                path / "ros2" / "Dockerfile",
            ]
            return all(p.exists() for p in required)
        except Exception:
            return False

    def _is_home_scoped(self, path: Path | None) -> bool:
        if not path:
            return False
        try:
            resolved = path.expanduser().resolve()
            home = Path.home().resolve()
            return resolved == home or home in resolved.parents
        except Exception:
            return False

    # Container name → compose service name mapping
    _CONTAINER_SERVICE_MAP = {
        "easytrainer_frontend": "frontend",
        "easytrainer_backend": "backend",
        "easytrainer_ros2": "ros2",
        # Legacy single-container support
        "easy_collector_service": "service",
    }

    def _get_running_services(self) -> list[str]:
        """Return a short list of running compose services based on container names."""
        try:
            out = subprocess.check_output([
                "docker", "ps", "--format", "{{.Names}}"
            ], text=True)
            names = [n.strip() for n in out.splitlines() if n.strip()]
            svc = []
            for container, service in self._CONTAINER_SERVICE_MAP.items():
                if container in names:
                    svc.append(service)
            return svc
        except Exception:
            return []

    def _service_exists(self) -> bool:
        """Check whether any service container exists (running or stopped)."""
        try:
            out = subprocess.check_output(
                ["docker", "ps", "-a", "--format", "{{.Names}}"],
                text=True,
            )
            names = [n.strip() for n in out.splitlines() if n.strip()]
            return any(c in names for c in self._CONTAINER_SERVICE_MAP)
        except Exception:
            return False

    def _image_exists(self, image: str) -> bool:
        """Return True if a local Docker image exists, without noisy stderr."""
        try:
            res = subprocess.run(
                ["docker", "image", "ls", "-q", image],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                check=False,
                timeout=30,
            )
            return bool((res.stdout or "").strip())
        except Exception:
            return False

    def _has_nvidia_runtime(self) -> bool:
        """Best-effort detection of NVIDIA Docker runtime availability."""
        try:
            res = subprocess.run(
                ["docker", "info", "--format", "{{json .Runtimes}}"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                check=False,
                timeout=30,
            )
            payload = (res.stdout or "").strip()
            if payload:
                try:
                    import json as _json
                    runtimes = _json.loads(payload)
                    if isinstance(runtimes, dict):
                        if any(k.lower() == "nvidia" for k in runtimes.keys()):
                            return True
                except Exception:
                    if "nvidia" in payload.lower():
                        return True
        except Exception:
            pass
        try:
            subprocess.run(
                ["nvidia-ctk", "--version"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True,
                timeout=15,
            )
            return True
        except Exception:
            return False

    def _resolve_nvidia_script(self) -> Path | None:
        candidates = [
            self.project_root / "scripts" / "install_nvidia_runtime.sh",
            APP_HOME / "scripts" / "install_nvidia_runtime.sh",
            REPO_ROOT_CANDIDATE / "scripts" / "install_nvidia_runtime.sh",
            Path("/opt/easytrainer/scripts/install_nvidia_runtime.sh"),
        ]
        for cand in candidates:
            try:
                if cand.is_file():
                    return cand
            except Exception:
                continue
        return None

    def _nvidia_setup_command(self, script: Path) -> list[str] | None:
        """Return a privileged command (preferring pkexec) to run the GPU setup script."""
        pkexec = shutil.which("pkexec")
        if pkexec:
            return [pkexec, "bash", str(script)]
        return None

    def _current_variant(self) -> str:
        return "cpu" if getattr(self, "install_variant", "gpu") == "cpu" else "gpu"

    def _requires_gpu(self) -> bool:
        return self._current_variant() == "gpu"

    def _set_install_variant(self, variant: str):
        variant = "cpu" if variant == "cpu" else "gpu"
        self.install_variant = variant
        cfg = load_config()
        cfg["install_variant"] = variant
        save_config(cfg)
        self._update_window_title()

    def _update_window_title(self):
        variant = self._current_variant()
        label = "GPU" if variant == "gpu" else "CPU"
        version = APP_VERSION or "0.0.0"
        title = f"Easy Trainer v{version} ({label})"
        try:
            self.setWindowTitle(title)
        except Exception:
            pass

    def _apply_compose_variant(self, variant: str | None = None) -> bool:
        """Apply CPU/GPU variant.

        In the 3-service architecture, docker-compose.yml is always the GPU version.
        For CPU, we copy docker-compose.cpu.yml over docker-compose.yml.
        For GPU, we ensure docker-compose.yml is unchanged (it's already GPU).
        """
        variant = (variant or self._current_variant())
        dst = self.project_root / "docker-compose.yml"

        if variant == "cpu":
            src = self.project_root / "docker-compose.cpu.yml"
            if not src.exists():
                try:
                    self.append_log("[VARIANT] docker-compose.cpu.yml 파일이 없어 기본 compose를 유지합니다.")
                except Exception:
                    pass
                return True  # GPU compose is fine as fallback
            if not self._ensure_project_root_writable():
                try:
                    self.append_log("[VARIANT][ERROR] 프로젝트 경로 권한 확보에 실패했습니다.")
                except Exception:
                    pass
                return False
            try:
                shutil.copy2(src, dst)
                return True
            except Exception as e:
                try:
                    self.append_log(f"[VARIANT][ERROR] compose 템플릿 적용 실패: {e}")
                except Exception:
                    pass
                return False
        else:
            # GPU: docker-compose.yml is already the 3-service GPU version
            return dst.exists()

    def _ensure_project_root_writable(self, force_auth: bool = False, allow_auth: bool = True) -> bool:
        if not force_auth and getattr(self, "_project_root_writable_fixed", False):
            return True
        path = getattr(self, "project_root", None)
        if not path:
            return False
        if force_auth:
            if allow_auth and self._run_pkexec_chown(path):
                self._project_root_writable_fixed = True
                return True
            return False
        needs_escalation = False
        try:
            path.mkdir(parents=True, exist_ok=True)
            test_file = path / ".ec_write_test"
            test_file.write_text("ok")
            try:
                test_file.unlink()
            except Exception:
                pass
        except Exception:
            needs_escalation = True
        compose_file = path / "docker-compose.yml"
        if compose_file.exists() and not os.access(compose_file, os.W_OK):
            needs_escalation = True
        if not needs_escalation:
            self._project_root_writable_fixed = True
            return True
        if allow_auth and self._run_pkexec_chown(path):
            self._project_root_writable_fixed = True
            return True
        return False

    def _ensure_app_home_writable(self, force_auth: bool = False, allow_auth: bool = True) -> bool:
        if not force_auth and getattr(self, "_app_home_writable_fixed", False):
            return True
        path = APP_HOME
        if force_auth:
            if allow_auth and self._run_pkexec_chown(path):
                self._app_home_writable_fixed = True
                return True
            return False
        needs_escalation = False
        try:
            path.mkdir(parents=True, exist_ok=True)
            test_file = path / ".ec_write_test"
            test_file.write_text("ok")
            try:
                test_file.unlink()
            except Exception:
                pass
        except Exception:
            needs_escalation = True
        try:
            cfg_path = CONFIG_FILE
            if cfg_path.exists() and not os.access(cfg_path, os.W_OK):
                needs_escalation = True
        except Exception:
            needs_escalation = True
        if not needs_escalation:
            self._app_home_writable_fixed = True
            return True
        if allow_auth and self._run_pkexec_chown(path):
            self._app_home_writable_fixed = True
            return True
        return False

    def _run_pkexec_chown(self, path: Path) -> bool:
        if not shutil.which("pkexec"):
            return False
        try:
            user = pwd.getpwuid(os.getuid()).pw_name
            group = grp.getgrgid(os.getgid()).gr_name
            cmd = "mkdir -p %s && chown -R %s:%s %s" % (
                shlex.quote(str(path)),
                shlex.quote(user),
                shlex.quote(group),
                shlex.quote(str(path)),
            )
            # timeout so a polkit auth dialog with no agent (or no response)
            # can't hang the launcher/update forever — fail to a clear error instead.
            res = subprocess.run(
                ["pkexec", "/bin/sh", "-c", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=120,
            )
            return res.returncode == 0
        except Exception:
            return False

    def _run_pkexec_chown_paths(self, paths: list[Path | None]) -> bool:
        if not shutil.which("pkexec"):
            return False
        try:
            user = pwd.getpwuid(os.getuid()).pw_name
            group = grp.getgrgid(os.getgid()).gr_name
            uniq: list[Path] = []
            for path in paths:
                if not path:
                    continue
                if path not in uniq:
                    uniq.append(path)
            if not uniq:
                return True
            quoted = " ".join(shlex.quote(str(p)) for p in uniq)
            cmd = "mkdir -p %s && chown -R %s:%s %s" % (
                quoted,
                shlex.quote(user),
                shlex.quote(group),
                quoted,
            )
            res = subprocess.run(
                ["pkexec", "/bin/sh", "-c", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=120,
            )
            return res.returncode == 0
        except Exception:
            return False

    def _run_pkexec_shell_cmd(self, cmd: str, timeout: int = 300) -> bool:
        """Run an arbitrary shell command as root via a single pkexec auth prompt.
        Used by the full-uninstall flow to remove system files / the deb package.
        Returns True only if pkexec ran and the command exited 0; False on a
        missing pkexec, a cancelled/failed auth, a timeout, or a non-zero exit."""
        if not shutil.which("pkexec"):
            return False
        try:
            res = subprocess.run(
                ["pkexec", "/bin/sh", "-c", cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
                timeout=timeout,
            )
            if res.stdout:
                try:
                    self.append_log(f"[UNINSTALL] {res.stdout.strip()}")
                except Exception:
                    pass
            return res.returncode == 0
        except Exception as e:
            try:
                self.append_log(f"[UNINSTALL][ERROR] pkexec 실패: {e}")
            except Exception:
                pass
            return False

    def _ensure_app_and_project_writable(self, force_auth: bool = False) -> bool:
        app_ok = self._ensure_app_home_writable(force_auth=False, allow_auth=False)
        proj_ok = self._ensure_project_root_writable(force_auth=False, allow_auth=False)
        if app_ok and proj_ok:
            return True
        if not force_auth:
            return False
        if self._run_pkexec_chown_paths([APP_HOME, getattr(self, "project_root", None)]):
            self._app_home_writable_fixed = True
            self._project_root_writable_fixed = True
            return True
        return False

    def _has_host_nvidia_driver(self) -> bool:
        smi = shutil.which("nvidia-smi")
        if not smi:
            return False
        try:
            res = subprocess.run(
                [smi],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True,
                timeout=15,
            )
            return res.returncode == 0
        except Exception:
            return False

    def _disk_usage_target(self) -> Path:
        try:
            if self.project_root and self.project_root.exists():
                return self.project_root
        except Exception:
            pass
        try:
            parent = getattr(self.project_root, "parent", None)
            if parent and parent.exists():
                return parent
        except Exception:
            pass
        return Path.home()

    def _unify_compose_service(self):
        """Legacy compat — no longer modifies compose files in 3-service architecture."""
        pass

    def _ensure_project_present(self):
        """Ensure a valid project exists under the shared data root.
        Copy from system payload or repo root, and merge if destination exists.
        """
        try:
            if self._is_valid_project_root(self.project_root):
                return

            # Ensure parent exists; warn on permission error (no HOME fallback)
            try:
                self.project_root.parent.mkdir(parents=True, exist_ok=True)
            except PermissionError:
                self.append_log("[WARN] 프로젝트 경로에 쓸 수 없습니다. EASYTRAINER_DATA_DIR를 쓰기 가능한 위치로 설정하거나 권한을 조정하세요.")
                return
            except Exception as e:
                self.append_log(f"[WARN] 프로젝트 경로를 준비하지 못했습니다: {e}")
                return

            # Pick source: prefer packaged payload; only use repo when explicitly allowed
            src = None
            if SYSTEM_PAYLOAD_DIR.exists():
                src = SYSTEM_PAYLOAD_DIR
            else:
                allow_repo = os.environ.get("EASYCOLLECTOR_USE_REPO", "0") == "1"
                if allow_repo and REPO_ROOT_CANDIDATE.exists():
                    src = REPO_ROOT_CANDIDATE
            if src is None:
                self.append_log("[WARN] No payload sources found.")
                return

            dest = self.project_root
            if not dest.exists():
                shutil.copytree(src, dest)
                self.append_log(f"[INIT] Copied project to {dest}")
            else:
                for item in src.iterdir():
                    s = item
                    d = dest / item.name
                    if s.is_dir():
                        shutil.copytree(s, d, dirs_exist_ok=True)
                    else:
                        shutil.copy2(s, d)
                self.append_log(f"[INIT] Populated existing project at {dest}")
        except Exception as e:
            self.append_log(f"[WARN] Failed to prepare project: {e}")

    def _clear_conflicting_containers(self):
        """Remove stale/orphan containers that can block compose up due to duplicate names.

        This handles the case where a container was created by a different compose project
        (e.g. running from a different directory) and now blocks 'up -d' from this project.
        Only removes containers that are NOT currently running.
        """
        targets = [
            "easytrainer_frontend",
            "easytrainer_backend",
            "easytrainer_ros2",
            # Legacy names
            "easy_collector_frontend",
            "easy_collector_backend",
            "easy_collector_service",
        ]
        try:
            # Get currently running container names
            running = set()
            try:
                out = subprocess.check_output(
                    ["docker", "ps", "--format", "{{.Names}}"], text=True, timeout=30
                )
                running = {n.strip() for n in out.splitlines() if n.strip()}
            except Exception:
                pass
            for name in targets:
                if name in running:
                    continue  # Don't kill running containers
                subprocess.run(
                    ["docker", "rm", "-f", name],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                    timeout=60,
                )
        except Exception:
            # Non-fatal; compose will still emit the conflict if removal fails
            pass

    def _ensure_service_running(self, reason: str, restart_if_running: bool, on_finish=None):
        """Start or restart the main service without destroying the container."""
        running = "service" in self._get_running_services()
        exists = self._service_exists()
        if running and not restart_if_running:
            self.append_log(f"[{reason}] 서비스가 이미 실행 중입니다.")
            if on_finish:
                on_finish(0)
            return
        if running and restart_if_running:
            self.append_log(f"[{reason}] docker compose restart ...")
            cmd = ["restart"]
        else:
            # Always use 'up -d' instead of 'start' — 'start' fails when container
            # was removed (e.g. after 'down --volumes'). 'up -d' handles both cases:
            # creates if missing, starts if stopped.
            self.append_log(f"[{reason}] docker compose up -d ...")
            cmd = ["up", "-d"]
        self.run_compose(cmd, on_finish=on_finish)

    # ------------------------ Actions ------------------------
    def on_start(self):
        if not self.is_installed():
            QMessageBox.warning(self, t("service.warningTitle"), t("service.installFirst"))
            return
        if not self._is_valid_project_root(self.project_root):
            QMessageBox.warning(self, t("service.projectRequiredTitle"), t("service.projectNotReady"))
            return
        if not docker_compose_available():
            QMessageBox.critical(self, t("service.errorTitle"), self._compose_help_text())
            return
        if not self._apply_compose_variant(self.install_variant):
            QMessageBox.critical(self, t("service.errorTitle"), t("service.composeTemplateNotFound"))
            return
        self._clear_conflicting_containers()
        self._show_preload_dialog(t("service.preparing"))
        self._ensure_service_running("START", restart_if_running=False, on_finish=self._on_start_finished)

    def _on_start_finished(self, exit_code: int, *_):
        if exit_code == 0:
            self.append_log("[START] 완료")
            self.update_state_label()
            self._wait_for_services_ready(self.load_ui)
        else:
            self._hide_preload_dialog()
            QMessageBox.critical(self, t("service.errorTitle"), t("service.startFailed", code=exit_code))

    def on_stop(self):
        if not self._is_valid_project_root(self.project_root):
            QMessageBox.warning(self, t("service.projectRequiredTitle"), t("service.projectNotReady"))
            return
        if not docker_compose_available():
            QMessageBox.critical(self, t("service.errorTitle"), self._compose_help_text())
            return
        self._run_backend_kill()
        self.append_log("[STOP] docker compose stop ...")
        self.run_compose(["stop"], on_finish=self._on_stop_finished)

    def _on_stop_finished(self, exit_code: int):
        if exit_code == 0:
            self.append_log("[STOP] 완료")
            self.update_state_label()
        else:
            QMessageBox.critical(self, t("service.errorTitle"), t("service.stopFailed", code=exit_code))

    def _on_restart_button_clicked(self):
        if not docker_compose_available() or not self._is_valid_project_root(self.project_root):
            try:
                QMessageBox.critical(self, t("service.errorTitle"), self._compose_help_text())
            except Exception:
                pass
            return
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            return
        try:
            if self._preload_dialog:
                self._preload_dialog.show_restart(True, enabled=False)
        except Exception:
            pass
        self.append_log("[RESTART] 컨테이너를 중지/삭제 후 재시작합니다...")
        self._restart_service_container()

    def _restart_service_container(self):
        self._show_preload_dialog(t("service.restarting"))
        self._set_preload_detail(t("service.restarting"))
        self._clear_conflicting_containers()
        self._restart_in_progress = True
        self._restart_phase = "stopping"
        self._run_backend_kill()

        def _after_down(ec: int, *_):
            if ec != 0:
                self.append_log(f"[RESTART][ERROR] 종료 실패 (code={ec})")
                self._enable_restart_button()
                self._hide_preload_dialog()
                return
            self._restart_phase = "starting"
            self.append_log("[RESTART] 컨테이너를 다시 시작합니다...")
            self.run_compose(["up", "-d"], on_finish=_after_up)

        def _after_up(ec: int, *_):
            if ec == 0:
                self.append_log("[RESTART] 완료. 준비 상태를 확인합니다.")
                self._wait_for_services_ready(self.load_ui)
            else:
                self.append_log(f"[RESTART][ERROR] 재시작 실패 (code={ec})")
                self._enable_restart_button()
                self._hide_preload_dialog()

        self.append_log("[RESTART] docker compose down --remove-orphans --volumes ...")
        self.run_compose(["down", "--remove-orphans", "--volumes"], on_finish=_after_down)

    def _run_backend_kill(self, keep_backend: bool = False, label: str = "STOP"):
        """Best-effort: run kill.sh inside the service container to clean ROS/backend processes."""
        container = "easytrainer_backend"
        running = self._get_running_services()
        if "backend" not in running and "service" not in running:
            return
        label = label or "STOP"
        cmd = "/root/src/kill.sh"
        if keep_backend:
            cmd = "EC_KILL_BACKEND=0 /root/src/kill.sh"
        try:
            result = subprocess.run(
                ["docker", "exec", "-i", container, "bash", "-lc", cmd],
                capture_output=True,
                text=True,
                timeout=10,
            )
            msg = result.stdout.strip() if result.stdout else ""
            if result.returncode == 0:
                suffix = " (keep backend)" if keep_backend else ""
                self.append_log(f"[{label}] kill.sh executed inside container{suffix}.")
                if msg:
                    self.append_log(f"[{label}][kill.sh] {msg}")
            else:
                err = result.stderr.strip() if result.stderr else ""
                self.append_log(f"[{label}][WARN] kill.sh returned {result.returncode}: {err or 'no output'}")
        except FileNotFoundError:
            self.append_log(f"[{label}][WARN] Docker not found; skip kill.sh.")
        except Exception as e:
            self.append_log(f"[{label}][WARN] kill.sh failed: {e}")

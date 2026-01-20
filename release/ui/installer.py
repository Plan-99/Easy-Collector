from __future__ import annotations

import shutil
import time
from typing import TYPE_CHECKING

from app_context import (
    MARKER_FILE,
    QCheckBox,
    QDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPixmap,
    QProcess,
    QProgressBar,
    QPushButton,
    QRadioButton,
    QTextEdit,
    QTimer,
    QVBoxLayout,
    QWidget,
    Qt,
    _app_icon_path,
)
from service import docker_compose_available, get_compose_cmd

if TYPE_CHECKING:
    from launcher import MainWindow


def run_setup_wizard(self: "MainWindow") -> bool:
    """Run the installer dialog using the launcher context."""
    if self.is_installed():
        return True

    dlg = QDialog(self)
    dlg.setWindowTitle("Easy Trainer Installer")
    dlg.resize(980, 680)

    # Header (icon + title + divider)
    title = QLabel("Easy Trainer")
    f = title.font(); f.setPointSize(f.pointSize() + 6); f.setBold(True); title.setFont(f)
    # small icon left of title (if available)
    icon_row = QHBoxLayout()
    icon_row.setContentsMargins(0, 0, 0, 0)
    icon_row.setSpacing(8)
    icon_label = QLabel()
    try:
        ip = _app_icon_path()
        if ip:
            pm = QPixmap(ip)
            if not pm.isNull():
                icon_label.setPixmap(pm.scaled(30, 30, Qt.KeepAspectRatio, Qt.SmoothTransformation))
    except Exception:
        pass
    icon_row.addWidget(icon_label)
    icon_row.addWidget(title)
    icon_row.addStretch(1)

    line = QFrame(); line.setFrameShape(QFrame.HLine); line.setFrameShadow(QFrame.Sunken); line.setObjectName("Divider"); line.setFixedHeight(1)

    header = QVBoxLayout(); header.addLayout(icon_row); header.addWidget(line)

    # Sidebar
    steps = QListWidget(); steps.setFixedWidth(170); steps.setEnabled(False)
    for s in ["시작", "설치 준비", "설치 옵션", "설치 중", "완료"]:
        QListWidgetItem(s, steps)

    # Pages inside a bordered content box
    container = QFrame(); container.setFrameShape(QFrame.Box); container.setFrameShadow(QFrame.Plain); container.setObjectName("ContentBox")
    container_layout = QVBoxLayout(); container.setLayout(container_layout)

    page_start = QWidget(); v1 = QVBoxLayout()
    v1.addWidget(QLabel("Easy Trainer는 ~ 입니다."))
    v1.addStretch(1)
    v1.addWidget(QLabel("'다음'을 눌러 설치를 진행하세요."))
    page_start.setLayout(v1)

    page_prepare = QWidget(); v2 = QVBoxLayout()
    lbl_space = QLabel("")
    # 안내 문구 아래에 현재 여유 공간을 표시
    v2.addWidget(QLabel("설치를 위한 저장 공간을 확인합니다(권장 20GB 이상)."))
    v2.addWidget(lbl_space)
    v2.addStretch(1)
    page_prepare.setLayout(v2)

    variant_choice = {"value": None}

    page_variant = QWidget(); v2b = QVBoxLayout()
    hdr_variant = QLabel("설치 방식을 선택하세요. 이후에도 동일한 모드로 서비스가 실행됩니다.")
    hdr_variant.setWordWrap(True)
    v2b.addWidget(hdr_variant)
    v2b.addSpacing(10)
    rb_cpu = QRadioButton("CPU 버전 : 일부 모델은 CPU에서 느릴 수 있습니다.")
    rb_gpu = QRadioButton("GPU 버전 (권장) : CUDA 가속을 사용할 수 있습니다. *NVIDIA 드라이버 사전 설치 필요")
    v2b.addWidget(rb_cpu)
    v2b.addWidget(rb_gpu)
    v2b.addStretch(1)
    page_variant.setLayout(v2b)

    page_install = QWidget(); v3 = QVBoxLayout()
    log = QTextEdit(); log.setReadOnly(True)
    bar = QProgressBar(); bar.setRange(0, 100); bar.setValue(0); bar.setTextVisible(True); bar.setFormat("0.00%")
    # 설치 상태: 왼쪽 문구만 표시
    status_row = QHBoxLayout(); status_row.setContentsMargins(0,0,0,0); status_row.setSpacing(6)
    lbl_status_left = QLabel("설치 중...")
    status_row.addWidget(lbl_status_left)
    status_row.addStretch(1)
    progress_row = QHBoxLayout(); progress_row.setContentsMargins(0,0,0,0); progress_row.setSpacing(6)
    lbl_elapsed = QLabel("경과 00:00")
    lbl_remaining = QLabel("잔여 --:--")
    progress_row.addWidget(lbl_elapsed)
    progress_row.addStretch(1)
    progress_row.addWidget(lbl_remaining)
    v3.addLayout(status_row); v3.addWidget(log, 1); v3.addWidget(bar); v3.addLayout(progress_row)
    page_install.setLayout(v3)

    page_done = QWidget(); v4 = QVBoxLayout()
    v4.addWidget(QLabel("설치가 완료되었습니다. ‘완료’를 눌러 시작하세요."))
    chk_launch = QCheckBox("Easy Trainer 실행하기")
    chk_launch.setChecked(True)
    v4.addWidget(chk_launch)
    v4.addStretch(1)
    page_done.setLayout(v4)

    pages = [page_start, page_prepare, page_variant, page_install, page_done]

    def set_page(idx: int):
        steps.setCurrentRow(idx)
        for i in reversed(range(container_layout.count())):
            w = container_layout.itemAt(i).widget()
            if w is not None:
                w.setParent(None)
        container_layout.addWidget(pages[idx])
        btn_prev.setVisible(idx in (1, 2))
        btn_prev.setEnabled(idx in (1, 2))
        btn_next.setVisible(idx in (0, 1, 3))
        btn_install.setVisible(idx == 2)
        btn_finish.setVisible(idx == 4)
        if idx == 0:
            btn_next.setEnabled(True)
        elif idx == 1:
            try:
                target = self._disk_usage_target()
                usage = shutil.disk_usage(str(target))
                lbl_space.setText(f"현재 여유 공간: {usage.free / (1024**3):.1f} GB")
                has_space = usage.free >= 20 * 1024**3
            except Exception:
                lbl_space.setText("현재 여유 공간: 확인 불가")
                has_space = True
            btn_next.setEnabled(has_space and docker_compose_available())
        elif idx == 3:
            btn_next.setEnabled(False)
        else:
            btn_next.setEnabled(True)

    # Body (sidebar + content) with explicit spacer to ensure visible gap
    body = QHBoxLayout()
    body.setSpacing(6)  # base spacing
    body.addWidget(steps)
    body.addSpacing(6)  # explicit vertical gap between sidebar and content box

    # Footer buttons (placed under content inside the right panel)
    btn_prev = QPushButton("이전"); btn_next = QPushButton("다음"); btn_install = QPushButton("설치"); btn_finish = QPushButton("완료")
    btn_install.setEnabled(False)
    footer = QHBoxLayout()
    footer_left = QHBoxLayout(); footer_left.setSpacing(6)
    footer_right = QHBoxLayout(); footer_right.setSpacing(6)
    # Place prev on the left side (near sidebar), others on the right side by default
    footer_left.addWidget(btn_prev)
    footer_right.addWidget(btn_next)
    footer_right.addWidget(btn_install)
    footer_right.addWidget(btn_finish)
    footer.addLayout(footer_left)
    footer.addStretch(1)
    footer.addLayout(footer_right)

    # Right panel = content box (pages) + footer buttons stacked vertically
    right_panel = QWidget()
    right_layout = QVBoxLayout(right_panel)
    # 외부 마진 제거 (콘텐츠 박스의 바깥 여백 0)
    right_layout.setContentsMargins(0, 0, 0, 0)
    right_layout.setSpacing(6)
    right_layout.addWidget(container, 1)
    right_layout.addSpacing(6)
    right_layout.addLayout(footer)

    body.addWidget(right_panel, 1)

    def _update_variant_choice():
        val = None
        if rb_gpu.isChecked():
            val = "gpu"
        elif rb_cpu.isChecked():
            val = "cpu"
        variant_choice["value"] = val
        btn_install.setEnabled(val is not None)

    rb_cpu.toggled.connect(lambda _: _update_variant_choice())
    rb_gpu.toggled.connect(lambda _: _update_variant_choice())
    _update_variant_choice()

    # Wire actions
    def on_prev():
        nonlocal current
        if current > 0:
            current -= 1; set_page(current)
    def on_next():
        nonlocal current
        if current in (0, 1):
            current += 1; set_page(current)
        elif current == 3:
            current = 4; set_page(current)
    def _record_launch_choice():
        try:
            self._auto_launch_after_install = chk_launch.isChecked()
        except Exception:
            self._auto_launch_after_install = True

    def on_finish():
        _record_launch_choice()
        dlg.accept()

    btn_prev.clicked.connect(on_prev)
    btn_next.clicked.connect(on_next)
    btn_finish.clicked.connect(on_finish)

    INSTALL_LOG_TARGET = 14954
    progress_state = {
        "lines": 0,
        "target": INSTALL_LOG_TARGET,
        "start": None,
        "tracking": False,
        "complete": False,
    }
    progress_timer = QTimer(dlg)
    progress_timer.setInterval(1000)
    progress_timer.timeout.connect(lambda: _update_progress_display())

    def _format_duration(seconds: float | None) -> str:
        if seconds is None or seconds < 0:
            return "--:--"
        seconds = int(seconds)
        m, s = divmod(seconds, 60)
        h, m = divmod(m, 60)
        if h:
            return f"{h:02d}:{m:02d}:{s:02d}"
        return f"{m:02d}:{s:02d}"

    def _current_percent(force_complete: bool = False) -> float:
        if force_complete or progress_state["complete"]:
            return 100.0
        target = max(1, progress_state["target"])
        pct = (progress_state["lines"] / target) * 100.0
        return min(pct, 99.99)

    def _update_progress_display(force_complete: bool = False):
        pct = _current_percent(force_complete)
        bar.setValue(int(pct))
        bar.setFormat(f"{pct:05.2f}%")
        elapsed = None
        if progress_state["start"] is not None:
            elapsed = max(0.0, time.monotonic() - progress_state["start"])
        elapsed_str = _format_duration(elapsed)
        if force_complete or progress_state["complete"]:
            remain_str = "00:00"
        elif pct <= 0.0:
            remain_str = "--:--"
        else:
            remaining = None
            if elapsed is not None:
                remaining = elapsed * max(0.0, 100.0 - pct) / max(pct, 0.01)
            remain_str = _format_duration(remaining)
        lbl_elapsed.setText(f"경과 {elapsed_str}")
        lbl_remaining.setText(f"잔여 {remain_str}")

    def _increment_progress(lines: int):
        if lines <= 0 or not progress_state["tracking"] or progress_state["complete"]:
            return
        progress_state["lines"] = min(progress_state["lines"] + lines, progress_state["target"])
        _update_progress_display()

    def _mark_progress_complete():
        progress_state["tracking"] = False
        progress_state["complete"] = True
        progress_state["lines"] = progress_state["target"]
        _update_progress_display(force_complete=True)
        try:
            lbl_status_left.setText("설치 완료")
        except Exception:
            pass
        try:
            progress_timer.stop()
        except Exception:
            pass

    _update_progress_display()

    def on_build_finished(code: int):
        progress_state["tracking"] = False
        if code == 0:
            run_post_install_steps()
        else:
            try:
                progress_timer.stop()
            except Exception:
                pass
            _update_progress_display()
            QMessageBox.critical(dlg, "오류", f"설치 실패 (code={code})")

    def _start_docker_build():
        log.append("[INSTALL] docker compose build --no-cache ...")
        proc = QProcess(dlg)
        program, prefix = get_compose_cmd()
        proc.setProgram(program); proc.setArguments([*prefix, "build", "--no-cache"]); proc.setWorkingDirectory(str(self.project_root))
        progress_state["tracking"] = True
        def _append(is_err=False):
            try:
                data = proc.readAllStandardError() if is_err else proc.readAllStandardOutput()
                text = bytes(data).decode(errors="ignore")
                if text:
                    log.append(text.rstrip("\n"))
                    line_count = len(text.splitlines())
                    _increment_progress(line_count)
            except Exception:
                pass
        proc.readyReadStandardOutput.connect(lambda: _append(False))
        proc.readyReadStandardError.connect(lambda: _append(True))
        proc.finished.connect(on_build_finished)
        proc.start()

    def _ensure_gpu_runtime_then_build():
        script = self._resolve_nvidia_script()
        if self._has_nvidia_runtime():
            log.append("[GPU] NVIDIA Container Toolkit이 감지되어 바로 설치를 진행합니다.")
            _start_docker_build()
            return
        if not script:
            log.append("[GPU][WARN] 설치 스크립트를 찾을 수 없어 자동 구성을 건너뜁니다. README의 NVIDIA 설치 절차를 참고하세요.")
            _start_docker_build()
            return
        cmd = self._nvidia_setup_command(script)
        if not cmd:
            log.append(f"[GPU][WARN] pkexec를 찾을 수 없어 자동 구성을 진행할 수 없습니다.\n"
                       f"터미널에서 아래 명령을 실행한 뒤 다시 시도하세요:\n  sudo bash {script}")
            _start_docker_build()
            return
        log.append("[GPU] NVIDIA Container Toolkit 설치/구성을 진행합니다 (관리자 권한 필요)...")
        gpu_proc = QProcess(dlg)
        gpu_proc.setProgram(cmd[0]); gpu_proc.setArguments(cmd[1:]); gpu_proc.setWorkingDirectory(str(script.parent))
        def _append_gpu(is_err=False):
            try:
                data = gpu_proc.readAllStandardError() if is_err else gpu_proc.readAllStandardOutput()
                text = bytes(data).decode(errors="ignore")
                if text:
                    for line in text.rstrip("\n").splitlines():
                        log.append(line)
            except Exception:
                pass
        gpu_proc.readyReadStandardOutput.connect(lambda: _append_gpu(False))
        gpu_proc.readyReadStandardError.connect(lambda: _append_gpu(True))
        def _after_gpu(exit_code: int, *_):
            if exit_code != 0:
                QMessageBox.critical(dlg, "오류", f"NVIDIA Container Toolkit 설치 실패 (code={exit_code}). 로그를 확인하세요.")
                return
            if self._has_nvidia_runtime():
                log.append("[GPU] NVIDIA Container Toolkit 설치가 완료되었습니다.")
            else:
                log.append("[GPU][WARN] 설치 후 docker info에서 NVIDIA 런타임을 확인하지 못했습니다. 필요 시 수동으로 확인하세요.")
            _start_docker_build()
        gpu_proc.finished.connect(_after_gpu)
        gpu_proc.start()

    def on_install_click():
        if not docker_compose_available():
            QMessageBox.critical(dlg, "오류", self._compose_help_text()); return
        variant = variant_choice["value"]
        if variant not in ("cpu", "gpu"):
            QMessageBox.warning(dlg, "설치 옵션 필요", "CPU 또는 GPU 버전을 선택하세요.")
            return
        if variant == "gpu" and not self._has_host_nvidia_driver():
            QMessageBox.critical(
                dlg,
                "GPU 드라이버 필요",
                "GPU 버전을 설치하기 전에 호스트에 NVIDIA 드라이버를 설치하고 'nvidia-smi'가 정상 동작하는지 확인하세요.",
            )
            return
        self._set_install_variant(variant)
        if not self._apply_compose_variant(variant):
            QMessageBox.critical(dlg, "오류", "선택한 설치 옵션에 맞는 docker-compose 템플릿을 찾을 수 없습니다.")
            return
        nonlocal current
        current = 3; set_page(current)
        log.clear()

        progress_state.update({
            "lines": 0,
            "target": INSTALL_LOG_TARGET,
            "start": time.monotonic(),
            "tracking": False,
            "complete": False,
        })
        _update_progress_display()
        try:
            progress_timer.start()
        except Exception:
            pass
        try:
            lbl_status_left.setText("설치 중...")
        except Exception:
            pass

        if variant == "gpu":
            _ensure_gpu_runtime_then_build()
        else:
            log.append("[CPU] GPU 없이 CPU 전용 모드로 설치합니다.")
            _start_docker_build()

    def run_post_install_steps():
        # Run post steps in a single container session

        # Post-install strategy:
        # 1) Run migrations in an ephemeral container (no concurrent backend).
        # 2) Start the service normally.
        svc = "service"

        def _exec_to_log(args: list[str], on_finish=None):
            p = QProcess(dlg)
            program, prefix = get_compose_cmd()
            p.setProgram(program)
            p.setArguments([*prefix, *args])
            p.setWorkingDirectory(str(self.project_root))
            def _r(is_err=False):
                try:
                    data = p.readAllStandardError() if is_err else p.readAllStandardOutput()
                    text = bytes(data).decode(errors="ignore")
                    if text:
                        log.append(text.rstrip("\n"))
                except Exception:
                    pass
            p.readyReadStandardOutput.connect(lambda: _r(False))
            p.readyReadStandardError.connect(lambda: _r(True))
            if on_finish:
                p.finished.connect(on_finish)
            p.start()

        # Step 1: migrate in an ephemeral run container with verbose diagnostics
        log.append(f"[POST] Running DB migrations in one-off container for '{svc}' ...")
        migrate_cmd = (
            "set -euxo pipefail; cd ~/src/backend/database; "
            "echo '[DBG] python:' $(python3 -V); "
            "python3 - <<'PY'\n"
            "import os, sys\n"
            "import importlib, importlib.metadata as m\n"
            "def v(p):\n"
            "  try:\n"
            "    print(f'[DBG] pkg {p}=', m.version(p))\n"
            "  except Exception as e:\n"
            "    print(f'[DBG] pkg {p}=n/a ({e})')\n"
            "for p in ['orator','cleo','inflection','pyyaml','faker','pendulum','backpack']:\n"
            "  v(p)\n"
            "print('[DBG] cwd=', os.getcwd())\n"
            "print('[DBG] files:', os.listdir())\n"
            "PY\n"
            "python3 -m pip show orator >/dev/null 2>&1 || ("
            "python3 -m pip install --no-deps --no-input -q "
            "backpack==0.1 simplejson faker lazy-object-proxy cleo==0.6.8 inflection "
            "pendulum==1.5.1 pytzdata python-dateutil && "
            "python3 -m pip install --no-deps --no-input -q orator==0.9.9); "
            "ls -l; echo '[DBG] db before:'; ls -l main.db || true; "
            # Run migrations via Orator Python API to avoid CLI/__main__ issues
            "python3 - <<'PY'\n"
            "import os, sys\n"
            "sys.path.insert(0, os.getcwd())\n"
            "try:\n"
            "  from importlib import import_module\n"
            "  cfg = import_module('config.database')\n"
            "  if hasattr(cfg, 'DATABASES'):\n"
            "    connections = {k:v for k,v in cfg.DATABASES.items() if isinstance(v, dict)}\n"
            "  elif hasattr(cfg, 'config'):\n"
            "    connections = cfg.config\n"
            "  else:\n"
            "    raise RuntimeError('No DATABASES/config mapping in config/database.py')\n"
            "  from orator import DatabaseManager\n"
            "  from orator.migrations import Migrator, DatabaseMigrationRepository\n"
            "  db = DatabaseManager(connections)\n"
            "  repo = DatabaseMigrationRepository(db, 'migrations')\n"
            "  if not repo.repository_exists():\n"
            "    print('[DBG] creating migrations repository table ...')\n"
            "    repo.create_repository()\n"
            "  migrator = Migrator(repo, db)\n"
            "  path = os.path.join(os.getcwd(), 'migrations')\n"
            "  print('[DBG] running migrations from', path)\n"
            "  migrator.run(path)\n"
            "  print('[DBG] migration complete')\n"
            "except Exception as e:\n"
            "  import traceback\n"
            "  traceback.print_exc()\n"
            "  sys.exit(1)\n"
            "PY\n"
            "echo '[DBG] db after:'; ls -l main.db || true"
        )

        def _after_migrate(rc: int):
            if rc != 0:
                QMessageBox.critical(dlg, "오류", f"마이그레이션 실패 (code={rc}) — 로그를 확인하세요.")
                return
            # Step 2: bring up service
            log.append(f"[POST] Bringing up service '{svc}' ...")
            def _after_up(exit_code: int):
                if exit_code != 0:
                    try:
                        running = "service" in self._get_running_services()
                    except Exception:
                        running = False
                    if not running:
                        QMessageBox.critical(dlg, "오류", f"서비스 시작 실패 (code={exit_code})")
                        return
                    log.append(f"[POST][WARN] 서비스 시작 exit_code={exit_code}, 하지만 컨테이너가 실행 중입니다.")
                _mark_progress_complete()
                try:
                    MARKER_FILE.write_text("ok")
                except Exception:
                    pass
                btn_next.setEnabled(True)
            self.run_compose_to_widget(["up", "-d", svc], log, on_finish=_after_up)

        self.run_compose_to_widget(
            ["run", "--rm", "--entrypoint", "bash", svc, "-c", migrate_cmd], 
            log, 
            on_finish=_after_migrate
        )

    btn_install.clicked.connect(on_install_click)

    # Layout on dialog (footer already inside right panel)
    layout = QVBoxLayout(dlg); layout.addLayout(header); layout.addLayout(body, 1)
    layout.setContentsMargins(6, 6, 6, 6)
    layout.setSpacing(6)
    # 외부(헤더/바디) 여백만 유지, 콘텐츠 박스 내부는 소량의 안쪽 여백만 부여
    for lay in (header, body):
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)
    container_layout.setContentsMargins(6, 6, 6, 6)
    container_layout.setSpacing(6)
    steps.setFixedWidth(220)

    # Let container expand naturally: it has stretch=1 above footer
    # Also normalize button sizes (width ~ 1/8 of panel, height ~ 1/3 of a sidebar row)
    def _adjust_button_sizes():
        try:
            rp = body.itemAt(1).widget()  # right_panel
            rp_w = max(1, rp.width())
            btn_w = max(60, rp_w // 8)
            row_h = steps.sizeHintForRow(0)
            if row_h <= 0:
                row_h = max(steps.sizeHint().height() // max(1, steps.count()), 24)
            btn_h = max(24, row_h // 3)
            for b in (btn_prev, btn_next, btn_install, btn_finish):
                b.setMinimumWidth(btn_w)
                b.setMinimumHeight(btn_h)
        except Exception:
            pass
    QTimer.singleShot(0, _adjust_button_sizes)
    current = 0; set_page(current)

    # Intercept close (X) / reject: confirm if not installed
    def _confirm_exit() -> bool:
        if not self.is_installed():
            res = QMessageBox.question(
                dlg,
                "종료 확인",
                "아직 설치되지 않았습니다. 정말 종료하시겠습니까?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            return res == QMessageBox.Yes
        return True

    def _on_close(ev):
        try:
            if _confirm_exit():
                if current == 4:
                    _record_launch_choice()
                ev.accept()
            else:
                ev.ignore()
        except Exception:
            ev.ignore()

    def _on_reject():
        if _confirm_exit():
            if current == 4:
                _record_launch_choice()
            QDialog.reject(dlg)

    dlg.closeEvent = _on_close
    dlg.reject = _on_reject

    # Exec modal; abort app if not completed
    result = dlg.exec()
    return result == QDialog.Accepted and self.is_installed()

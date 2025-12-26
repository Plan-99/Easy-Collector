from __future__ import annotations

import os
import sys
import json
import shlex
import shutil
import subprocess
import time
import pwd
import grp
from pathlib import Path

# Apply safe Qt WebEngine defaults BEFORE importing Qt modules
_force_external = os.environ.get("EASYCOLLECTOR_FORCE_EXTERNAL_BROWSER", "0") == "1"
if not _force_external:
    # Disable sandbox and GPU to avoid common crashes
    os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
    os.environ.setdefault("QT_OPENGL", "software")
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    # Force X11 on Wayland sessions
    if os.environ.get("XDG_SESSION_TYPE", "").lower() == "wayland":
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    # Chromium flags for WebEngine (avoid single-process/in-process-gpu which can crash)
    _chromium_flags = os.environ.get("QTWEBENGINE_CHROMIUM_FLAGS", "").strip()
    _safe_flags = "--disable-gpu --disable-gpu-compositing"
    if _safe_flags not in _chromium_flags:
        os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = (f"{_chromium_flags} " if _chromium_flags else "") + _safe_flags

# Single source of truth for data/log locations (defaults to /opt/easytrainer)
DATA_ROOT = Path(os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer")).expanduser()

# Ensure QtWebEngine dictionary path is writable to avoid base::dir_app_dictionaries warnings
dict_dir_env = os.environ.get("QTWEBENGINE_DICTIONARIES_PATH")
dict_dir = Path(dict_dir_env) if dict_dir_env else DATA_ROOT / "qtwebengine_dictionaries"
try:
    dict_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("QTWEBENGINE_DICTIONARIES_PATH", str(dict_dir))
except Exception:
    pass

# Frontend URL (override with EASYCOLLECTOR_FRONTEND_URL if needed)
FRONTEND_URL = os.environ.get("EASYCOLLECTOR_FRONTEND_URL", "http://localhost:5173/")

# Try PySide6 first, then PyQt6, and adapt minor API differences
HAS_WEBENGINE = False

try:
    from PySide6.QtCore import Qt, QUrl, QProcess, QTimer, QPoint, QEvent
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QPushButton, QTextEdit, QLabel,
        QVBoxLayout, QHBoxLayout, QMessageBox, QTabWidget, QFileDialog, QLineEdit,
        QDialog, QFrame, QListWidget, QListWidgetItem, QProgressBar, QToolButton,
        QMenu, QPlainTextEdit, QSizePolicy, QRadioButton, QCheckBox
    )
    try:
        from PySide6.QtWebEngineWidgets import QWebEngineView  # optional
        from PySide6.QtWebEngineCore import QWebEngineProfile
        HAS_WEBENGINE = not _force_external
    except Exception:
        QWebEngineView = None  # type: ignore
        QWebEngineProfile = None  # type: ignore
        HAS_WEBENGINE = False
    from PySide6.QtGui import QDesktopServices, QIcon, QPixmap, QFont
except Exception:
    from PyQt6.QtCore import Qt, QUrl, QProcess, QTimer, QPoint, QEvent
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QPushButton, QTextEdit, QLabel,
        QVBoxLayout, QHBoxLayout, QMessageBox, QTabWidget, QFileDialog, QLineEdit,
        QDialog, QFrame, QListWidget, QListWidgetItem, QProgressBar, QToolButton,
        QMenu, QPlainTextEdit, QSizePolicy, QRadioButton, QCheckBox
    )
    try:
        from PyQt6.QtWebEngineWidgets import QWebEngineView  # optional
        from PyQt6.QtWebEngineCore import QWebEngineProfile
        HAS_WEBENGINE = not _force_external
    except Exception:
        QWebEngineView = None  # type: ignore
        QWebEngineProfile = None  # type: ignore
        HAS_WEBENGINE = False
    from PyQt6.QtGui import QDesktopServices, QIcon, QPixmap, QFont


APP_HOME = DATA_ROOT
MARKER_FILE = APP_HOME / ".installed"
CONFIG_FILE = APP_HOME / "config.json"
LEGACY_CONFIG_LOCATIONS = [
    Path.home() / "EasyTrainer" / "config.json",
    Path(os.environ.get("XDG_DATA_HOME", str(Path.home() / ".local" / "share"))) / "EasyTrainer" / "config.json",
]
DEFAULT_PROJECT_PATH = DATA_ROOT / "project"
SYSTEM_PAYLOAD_DIR = Path("/usr/share/easytrainer-project")
REPO_ROOT_CANDIDATE = Path(__file__).resolve().parents[2]

# Unified log persistence (force /tmp to avoid host permission issues)
SERVICE_LOG_DIR = Path(os.environ.get("EASYTRAINER_LOG_DIR", "/tmp/easytrainer/logs"))
UI_LOG_DIR = SERVICE_LOG_DIR
UI_LOG_FILE = UI_LOG_DIR / "ui.log"


class ServiceSplash(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setModal(True)
        self.setFixedSize(320, 150)
        flags = self.windowFlags()
        flags = (flags & ~Qt.WindowContextHelpButtonHint) | Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint
        self.setWindowFlags(flags)
        layout = QVBoxLayout(self)
        self.message_label = QLabel("서비스 준비중...", self)
        self.message_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.message_label)
        self.progress = QProgressBar(self)
        self.progress.setRange(0, 0)  # indefinite
        layout.addWidget(self.progress)
        self.detail_label = QLabel("", self)
        self.detail_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.detail_label)

    def showEvent(self, event):
        super().showEvent(event)
        self._center_on_parent()

    def _center_on_parent(self):
        parent = self.parent()
        if parent and parent.isVisible():
            geo = parent.frameGeometry()
            center = geo.center()
        else:
            screen = QApplication.primaryScreen()
            if screen:
                geo = screen.availableGeometry()
                center = geo.center()
            else:
                center = QPoint(0, 0)
        self.move(int(center.x() - self.width() / 2), int(center.y() - self.height() / 2))

    def set_message(self, text: str):
        self.message_label.setText(text)

    def set_detail(self, text: str):
        self.detail_label.setText(text)


def _app_icon_path() -> str | None:
    try:
        sys_icon = Path("/opt/easytrainer/app_icon.png")
        if sys_icon.exists():
            return str(sys_icon)
        repo_icon = REPO_ROOT_CANDIDATE / "app_icon.png"
        if repo_icon.exists():
            return str(repo_icon)
    except Exception:
        pass
    return None


def _window_icon() -> QIcon | None:
    """Load the app icon and downscale to safe sizes to avoid oversized XCB transfers."""
    ip = _app_icon_path()
    if not ip:
        return None
    try:
        pm = QPixmap(ip)
        if pm.isNull():
            return None
        icon = QIcon()
        for size in (16, 24, 32, 48, 64, 128, 256):
            icon.addPixmap(pm.scaled(size, size, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        return icon
    except Exception:
        return None


def _configure_webengine_profile():
    """Disable spellcheck (to avoid dictionary path overrides) and apply custom paths."""
    if not HAS_WEBENGINE or QWebEngineProfile is None:
        return
    try:
        profile = QWebEngineProfile.defaultProfile()
    except Exception:
        return
    try:
        profile.setSpellCheckEnabled(False)
    except Exception:
        pass
    dict_path = os.environ.get("QTWEBENGINE_DICTIONARIES_PATH")
    if dict_path and hasattr(profile, "setSpellCheckPath"):
        try:
            profile.setSpellCheckPath(dict_path)
        except Exception:
            pass

def docker_compose_available() -> bool:
    """Detect docker compose capability (prefer v2 plugin)."""
    # Prefer docker CLI with compose plugin (v2)
    if shutil.which("docker"):
        return True
    # Fallback to legacy docker-compose v1 binary
    return shutil.which("docker-compose") is not None

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

def _maybe_migrate_legacy_config():
    if CONFIG_FILE.exists():
        return
    for legacy in LEGACY_CONFIG_LOCATIONS:
        if legacy == CONFIG_FILE:
            continue
        try:
            if legacy.exists():
                CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(legacy, CONFIG_FILE)
                break
        except Exception:
            continue


def load_config() -> dict:
    _maybe_migrate_legacy_config()
    if CONFIG_FILE.exists():
        try:
            return json.loads(CONFIG_FILE.read_text() or '{}')
        except Exception:
            return {}
    return {}

def save_config(cfg: dict):
    try:
        APP_HOME.mkdir(parents=True, exist_ok=True)
        CONFIG_FILE.write_text(json.dumps(cfg, ensure_ascii=False, indent=2))
    except Exception:
        pass


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.install_variant = "gpu"
        self.setWindowTitle("Easy Trainer (GPU)")
        try:
            icon = _window_icon()
            if icon:
                self.setWindowIcon(icon)
        except Exception:
            pass
        self.resize(1000, 700)

        self.process: QProcess | None = None
        self._preload_dialog: ServiceSplash | None = None
        self._ready_check_timer: QTimer | None = None
        self._ready_timeout_timer: QTimer | None = None
        self._main_visible = False
        self._fullscreen_pref = False
        self._closing = False
        self._log_windows: list[QDialog] = []
        self._auto_launch_after_install = True

        # UI elements
        self.status_label = QLabel(self)
        # PyQt6 moved flags under TextInteractionFlag
        try:
            self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        except AttributeError:
            self.status_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.status_label.hide()


        # Hidden log buffer for append_log helper
        self.log = QTextEdit(self)
        self.log.setReadOnly(True)
        self.log.setVisible(False)

        # Minimal dev controls (icon buttons)
        # Web view (fills remaining area)
        if HAS_WEBENGINE:
            self.web_view = QWebEngineView(self)
            try:
                self.web_view.urlChanged.connect(lambda *_: self._update_nav_buttons())
            except Exception:
                pass
            _configure_webengine_profile()
        else:
            self.web_view = None
        self.web_container = QWidget(self)
        web_layout = QVBoxLayout(self.web_container)
        web_layout.setContentsMargins(0, 0, 0, 0)
        web_layout.setSpacing(0)
        if self.web_view:
            web_layout.addWidget(self.web_view)
        else:
            placeholder = QLabel("Qt WebEngine을 사용할 수 없어 UI를 표시할 수 없습니다.")
            placeholder.setAlignment(Qt.AlignCenter)
            web_layout.addWidget(placeholder)

        # Navigation bar
        nav = QWidget(self)
        nav_layout = QHBoxLayout(nav)
        nav_layout.setContentsMargins(8, 4, 8, 4)
        nav_layout.setSpacing(8)
        self._nav_button_size = 34
        def _mk_btn(text: str, tooltip: str):
            b = QPushButton(text)
            b.setToolTip(tooltip)
            size = self._nav_button_size
            b.setFixedSize(size, size)
            try:
                b.setMinimumSize(size, size)
                b.setMaximumSize(size, size)
                b.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            except Exception:
                pass
            try:
                b.setStyleSheet(f"QPushButton {{ padding: 0px; min-width: {size}px; max-width: {size}px; min-height: {size}px; max-height: {size}px; }}")
            except Exception:
                pass
            return b
        self.btn_nav_back = _mk_btn("←", "뒤로가기")
        self.btn_nav_forward = _mk_btn("→", "앞으로가기")
        self.btn_nav_back.clicked.connect(lambda: self._nav_action("back"))
        self.btn_nav_forward.clicked.connect(lambda: self._nav_action("forward"))
        nav_layout.addWidget(self.btn_nav_back)
        nav_layout.addWidget(self.btn_nav_forward)

        self.project_path_display = QLineEdit(self)
        self.project_path_display.setReadOnly(True)
        self.project_path_display.setPlaceholderText("원본 프로젝트 경로 (클릭하여 변경)")
        self.project_path_display.setFixedHeight(self._nav_button_size)
        self.project_path_display.installEventFilter(self)
        self.project_path_display.setContextMenuPolicy(Qt.CustomContextMenu)
        self.project_path_display.customContextMenuRequested.connect(self._show_path_menu)
        try:
            self.project_path_display.setCursor(Qt.PointingHandCursor)
        except AttributeError:
            try:
                self.project_path_display.setCursor(Qt.CursorShape.PointingHandCursor)  # type: ignore[attr-defined]
            except Exception:
                pass
        nav_layout.addWidget(self.project_path_display, 1)

        self.btn_quick_apply = _mk_btn("⇆", "빠른 동기화 및 새로고침")
        self.btn_quick_apply.clicked.connect(self._on_quick_apply_clicked)
        nav_layout.addWidget(self.btn_quick_apply)

        self.btn_logs = _mk_btn("Log", "서비스 로그 보기")
        self.btn_logs.clicked.connect(self.open_logs_window)
        nav_layout.addWidget(self.btn_logs)

        # Main layout
        main = QWidget(self)
        v = QVBoxLayout()
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(4)
        v.addWidget(nav)
        v.addWidget(self.web_container, 1)
        main.setLayout(v)
        self.setCentralWidget(main)
        self._update_nav_buttons()

        # state init
        try:
            APP_HOME.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
        try:
            UI_LOG_DIR.mkdir(parents=True, exist_ok=True)
            # Start a new session section in the UI log file
            with UI_LOG_FILE.open("a", encoding="utf-8") as f:
                from datetime import datetime
                f.write("\n=== EasyTrainer UI session start: %s ===\n" % datetime.now().isoformat(timespec="seconds"))
        except Exception:
            pass
        try:
            # Inform in-app that log persistence is active
            self.append_log(f"[LOG] UI 로그 저장 위치: {UI_LOG_FILE}")
        except Exception:
            pass

        # Resolve project root: env > config > default under data root (no HOME fallback)
        cfg = load_config()
        self.install_variant = cfg.get("install_variant", "gpu")
        self._update_window_title()
        env_root = os.environ.get("EASYCOLLECTOR_PROJECT_ROOT", "").strip()
        cfg_root = cfg.get("project_root")
        if env_root:
            self.project_root = Path(env_root).expanduser()
        elif cfg_root:
            candidate = Path(cfg_root).expanduser()
            if self._is_home_scoped(candidate):
                candidate = DEFAULT_PROJECT_PATH
                cfg["project_root"] = str(candidate)
                save_config(cfg)
            self.project_root = candidate
        else:
            self.project_root = DEFAULT_PROJECT_PATH
            cfg["project_root"] = str(self.project_root)
            save_config(cfg)

        # Ensure project exists; if not, copy from system payload or repo
        self._ensure_project_present()
        # Always request auth on startup, then ensure the project root is writable.
        self._ensure_project_root_writable(force_auth=True)
        # Unify compose service name to 'service' inside project compose files
        self._unify_compose_service()
        # Ensure docker-compose.yml matches the selected variant
        self._apply_compose_variant(self.install_variant)

        # Initialize developer source path
        self.dev_src_root = None
        cfg = load_config()
        dev_src_saved = cfg.get("dev_src_root")
        if dev_src_saved:
            self.dev_src_root = Path(dev_src_saved).expanduser()
        self.update_dev_src_label()

        # Always sync latest from dev source on startup (if configured).
        try:
            QTimer.singleShot(0, self._auto_sync_on_start)
        except Exception:
            pass

        self.update_state_label()
        self.update_buttons()
        self.update_project_label()

        # Periodic status update
        self.timer = QTimer(self)
        self.timer.setInterval(3000)
        self.timer.timeout.connect(self.update_state_label)
        self.timer.start()

        self.hide()

    def set_display_mode(self, fullscreen: bool):
        self._fullscreen_pref = fullscreen

    def eventFilter(self, obj, event):
        if obj is self.project_path_display and event is not None:
            try:
                etype = event.type()
            except Exception:
                etype = None
            if etype == QEvent.MouseButtonPress:
                try:
                    button = event.button()
                except Exception:
                    button = None
                if button == Qt.LeftButton:
                    self.on_select_dev_src()
                    return True
        return super().eventFilter(obj, event)

    def _show_path_menu(self, pos):
        menu = QMenu(self.project_path_display)
        change_action = menu.addAction("원본 경로 변경...")
        open_action = None
        if self.dev_src_root and self.dev_src_root.exists():
            open_action = menu.addAction("폴더 열기")
        action = menu.exec(self.project_path_display.mapToGlobal(pos))
        if action == change_action:
            self.on_select_dev_src()
        elif action == open_action and self.dev_src_root:
            try:
                QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.dev_src_root)))
            except Exception:
                pass

    def _ensure_main_window_visible(self):
        if self._main_visible:
            return
        if self._fullscreen_pref:
            self.showFullScreen()
        else:
            self.showMaximized()
        self._main_visible = True

    def closeEvent(self, event):
        """Ensure services stop gracefully before exiting."""
        if self._closing:
            try:
                super().closeEvent(event)
            except Exception:
                event.accept()
            return
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            # Wait for in-flight compose command to finish first
            self.process.finished.connect(lambda *_: self.close())
            event.ignore()
            return
        event.ignore()
        self._closing = True
        self.hide()
        if not docker_compose_available() or not self._is_valid_project_root(self.project_root):
            QApplication.instance().quit()
            return
        def _after_down(_code: int, *_):
            QApplication.instance().quit()
        self.run_compose(["down", "--remove-orphans"], on_finish=_after_down)

    def run_compose_blocking(self, args: list[str]) -> int:
        program, prefix = get_compose_cmd()
        if not program:
            return 1
        import subprocess
        try:
            res = subprocess.run([program, *prefix, *args], cwd=str(self.project_root), text=True)
            return res.returncode
        except Exception:
            return 1

    # ------------------------ Setup Wizard ------------------------
    def maybe_run_setup_wizard(self) -> bool:
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
        # 설치 상태: 왼쪽은 고정 문구, 오른쪽은 단계 설명 + (N/7)
        status_row = QHBoxLayout(); status_row.setContentsMargins(0,0,0,0); status_row.setSpacing(6)
        lbl_status_left = QLabel("설치 중...")
        lbl_status_right = QLabel("시스템을 준비하는 중입니다 (1/7)")
        status_row.addWidget(lbl_status_left)
        status_row.addStretch(1)
        status_row.addWidget(lbl_status_right)
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

        # Right-side status messages in "~하는 중입니다" style
        stage_messages = [
            "시스템을 준비하는 중입니다",
            "로봇 기능을 구성하는 중입니다",
            "인터페이스를 설치하는 중입니다",
            "AI 실행 환경을 구성하는 중입니다",
            "최적화된 실행 환경을 구성하는 중입니다",
            "하드웨어 장치를 구성하는 중입니다",
            "프로그램 구성을 마무리하는 중입니다",
        ]
        current_stage = {"idx": 1}

        def _advance_stage(new_stage: int):
            if new_stage > current_stage["idx"] and 1 <= new_stage <= len(stage_messages):
                current_stage["idx"] = new_stage
                try:
                    lbl_status_right.setText(f"{stage_messages[new_stage-1]} ({new_stage}/7)")
                except Exception:
                    pass

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
                progress_timer.stop()
            except Exception:
                pass

        _update_progress_display()

        def on_build_finished(code: int):
            progress_state["tracking"] = False
            if code == 0:
                _advance_stage(6)
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
                        import re
                        m = re.search(r"Step\s+(\d+)\s*/\s*(\d+)", text)
                        if m:
                            cur = int(m.group(1)); total = max(1, int(m.group(2)))
                            est_stage = max(1, min(6, int((cur - 1) * 6 / total) + 1))
                            _advance_stage(est_stage)
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
            current_stage["idx"] = 1
            try:
                lbl_status_right.setText(f"{stage_messages[0]} (1/7)")
            except Exception:
                pass

            if variant == "gpu":
                _ensure_gpu_runtime_then_build()
            else:
                log.append("[CPU] GPU 없이 CPU 전용 모드로 설치합니다.")
                _start_docker_build()

        def run_post_install_steps():
            # Move to stage 7 (100% cap) and run post steps in a single container session
            _advance_stage(7)

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
                        QMessageBox.critical(dlg, "오류", f"서비스 시작 실패 (code={exit_code})")
                        return
                    _mark_progress_complete()
                    try:
                        MARKER_FILE.write_text("ok")
                    except Exception:
                        pass
                    btn_next.setEnabled(True)
                self.run_compose_to_widget(["up", "-d", svc], log, on_finish=_after_up)

            self.run_compose_to_widget(["run", "--rm", svc, "bash", "-lc", migrate_cmd], log, on_finish=_after_migrate)

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

    # ------------------------ UI state helpers ------------------------
    def is_installed(self) -> bool:
        # Must have marker AND a usable Docker/compose environment with built image
        if not MARKER_FILE.exists():
            return False
        if not docker_compose_available():
            return False
        if not self._is_valid_project_root(self.project_root):
            return False
        # Check that the compose image exists (defined in docker-compose.yml)
        try:
            return self._image_exists("easy-collector:latest")
        except Exception:
            return False

    def update_buttons(self):
        has_root = self._is_valid_project_root(self.project_root)
        valid_src = self._is_valid_dev_src(self.dev_src_root)
        if not has_root:
            placeholder = "프로젝트 루트를 먼저 설정하세요"
        elif not valid_src:
            placeholder = "원본 프로젝트 경로 (클릭하여 선택)"
        else:
            placeholder = ""
        try:
            self.project_path_display.setPlaceholderText(placeholder)
        except Exception:
            pass
        try:
            self.btn_quick_apply.setEnabled(valid_src)
        except Exception:
            pass

    def update_state_label(self):
        parts = []
        if self.is_installed():
            parts.append("설치 상태: 설치됨")
        else:
            parts.append("설치 상태: 미설치")

        running = self._get_running_services()
        if running:
            parts.append("실행 중: " + ", ".join(running))
        else:
            parts.append("실행 중: 없음")

        if self.status_label.isVisible():
            self.status_label.setText(" | ".join(parts))
        self.update_buttons()

    def update_dev_src_label(self):
        self.update_project_label()

    def update_project_label(self):
        text = str(self.dev_src_root) if self.dev_src_root else ""
        try:
            self.project_path_display.setText(text)
            tooltip = text or "원본 경로 미설정 - 더블클릭하여 선택하세요."
            self.project_path_display.setToolTip(tooltip)
        except Exception:
            pass

    def _is_valid_project_root(self, path: Path) -> bool:
        try:
            return bool(path) and path.is_dir() and (path / "docker-compose.yml").exists()
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

    def _is_valid_dev_src(self, path: Path | None) -> bool:
        try:
            return bool(path) and path.is_dir() and (path / "src" / "backend").exists() and (path / "src" / "ui").exists()
        except Exception:
            return False

    def _get_running_services(self) -> list[str]:
        """Return a short list of running compose services based on container names."""
        try:
            import subprocess
            out = subprocess.check_output([
                "docker", "ps", "--format", "{{.Names}}"
            ], text=True)
            names = [n.strip() for n in out.splitlines() if n.strip()]
            svc = []
            if "easy_collector_service" in names:
                svc.append("service")
            return svc
        except Exception:
            return []

        # Dynamic service detection removed; we standardize on 'service'.


    def _image_exists(self, image: str) -> bool:
        """Return True if a local Docker image exists, without noisy stderr."""
        try:
            import subprocess
            res = subprocess.run(
                ["docker", "image", "ls", "-q", image],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                check=False,
            )
            return bool((res.stdout or "").strip())
        except Exception:
            return False

    def _has_nvidia_runtime(self) -> bool:
        """Best-effort detection of NVIDIA Docker runtime availability."""
        try:
            import subprocess, json as _json
            res = subprocess.run(
                ["docker", "info", "--format", "{{json .Runtimes}}"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                check=False,
            )
            payload = (res.stdout or "").strip()
            if payload:
                try:
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
            import subprocess
            subprocess.run(
                ["nvidia-ctk", "--version"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True,
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
        try:
            self.setWindowTitle(f"Easy Trainer ({label})")
        except Exception:
            pass

    def _apply_compose_variant(self, variant: str | None = None) -> bool:
        variant = (variant or self._current_variant())
        src_name = "docker-compose.gpu.yml" if variant == "gpu" else "docker-compose.cpu.yml"
        src = self.project_root / src_name
        dst = self.project_root / "docker-compose.yml"
        if not src.exists():
            try:
                self.append_log(f"[VARIANT] {src_name} 파일이 없어 기본 compose를 유지합니다.")
            except Exception:
                pass
            return False
        if not self._ensure_project_root_writable():
            try:
                self.append_log("[VARIANT][ERROR] 프로젝트 경로 권한 확보에 실패했습니다.")
            except Exception:
                pass
            return False
        try:
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            return True
        except Exception as e:
            try:
                if isinstance(e, PermissionError):
                    if self._ensure_project_root_writable():
                        shutil.copy2(src, dst)
                        return True
            except Exception:
                pass
            try:
                self.append_log(f"[VARIANT][ERROR] compose 템플릿 적용 실패: {e}")
            except Exception:
                pass
            return False

    def _ensure_project_root_writable(self, force_auth: bool = False) -> bool:
        if not force_auth and getattr(self, "_project_root_writable_fixed", False):
            return True
        path = getattr(self, "project_root", None)
        if not path:
            return False
        if force_auth:
            if self._run_pkexec_chown(path):
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
        if self._run_pkexec_chown(path):
            self._project_root_writable_fixed = True
            return True
        return False

    def _run_pkexec_chown(self, path: Path) -> bool:
        if not shutil.which("pkexec"):
            return False
        try:
            user = pwd.getpwuid(os.getuid()).pw_name
            group = grp.getgrgid(os.getgid()).gr_name
            cmd = "chown -R %s:%s %s" % (
                shlex.quote(user),
                shlex.quote(group),
                shlex.quote(str(path)),
            )
            res = subprocess.run(
                ["pkexec", "/bin/sh", "-c", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            return res.returncode == 0
        except Exception:
            return False

    def _auto_sync_on_start(self):
        if not self._is_valid_dev_src(self.dev_src_root):
            try:
                self.append_log("[SYNC][AUTO] 원본 경로가 없어 자동 동기화를 건너뜁니다.")
            except Exception:
                pass
            return
        try:
            self.append_log("[SYNC][AUTO] 최신 코드 자동 동기화를 시작합니다.")
        except Exception:
            pass
        if not self._sync_dev_files(show_errors=False):
            try:
                self.append_log("[SYNC][AUTO][WARN] 자동 동기화에 실패했습니다.")
            except Exception:
                pass
            return
        try:
            self.append_log("[SYNC][AUTO] 자동 동기화 완료.")
        except Exception:
            pass

    def _has_host_nvidia_driver(self) -> bool:
        smi = shutil.which("nvidia-smi")
        if not smi:
            return False
        try:
            import subprocess
            res = subprocess.run(
                [smi],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True,
            )
            return res.returncode == 0
        except Exception:
            return False

    def auto_launch_enabled(self) -> bool:
        return bool(getattr(self, "_auto_launch_after_install", True))

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
        """Ensure docker-compose service name is 'service' and container is unified.
        Modifies docker-compose.yml and docker-compose.dev.yml in project_root if needed.
        """
        try:
            import re
            for fn in ("docker-compose.yml", "docker-compose.dev.yml"):
                path = self.project_root / fn
                if not path.exists():
                    continue
                try:
                    txt = path.read_text(encoding="utf-8", errors="ignore")
                except Exception:
                    continue
                orig = txt
                # Rename top-level service key 'backend:' -> 'service:'
                txt = re.sub(r"(?m)^(\s{2,})backend\s*:", r"\1service:", txt)
                # Update container_name if present
                txt = txt.replace("easy_collector_backend", "easy_collector_service")
                if txt != orig:
                    try:
                        path.write_text(txt, encoding="utf-8")
                    except Exception:
                        pass
            # Best-effort remove legacy container if it exists
            try:
                import subprocess
                subprocess.run(["docker", "rm", "-f", "easy_collector_backend"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        except Exception:
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
        """Remove stale containers that can block compose up due to duplicate names."""
        targets = [
            "easy_collector_service",
            "easy_collector_frontend",
            "easy_collector_backend",
        ]
        try:
            import subprocess
            for name in targets:
                subprocess.run(
                    ["docker", "rm", "-f", name],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
        except Exception:
            # Non-fatal; compose will still emit the conflict if removal fails
            pass

    # ------------------------ Actions ------------------------
    def on_install(self):
        if not docker_compose_available():
            QMessageBox.critical(self, "오류", self._compose_help_text())
            return
        if not self._is_valid_project_root(self.project_root):
            QMessageBox.warning(self, "프로젝트 필요", "프로젝트가 아직 준비되지 않았습니다. 설치를 먼저 진행하세요.")
            return
        if not self._apply_compose_variant(self.install_variant):
            QMessageBox.critical(self, "오류", "선택한 설치 옵션에 맞는 docker-compose 템플릿을 찾을 수 없습니다.")
            return

        reply = QMessageBox.question(
            self, "초기 설치",
            "Docker 이미지를 빌드합니다. 시간이 다소 걸릴 수 있습니다. 진행할까요?",
        )
        if reply != QMessageBox.StandardButton.Yes:
            return

        self.append_log("[INSTALL] docker compose build --no-cache ...")
        self.run_compose(["build", "--no-cache"], on_finish=self._on_install_finished)

    def _on_install_finished(self, exit_code: int):
        if exit_code == 0:
            MARKER_FILE.write_text("ok")
            self.append_log("[INSTALL] 완료")
            self.update_state_label()
        else:
            QMessageBox.critical(self, "오류", f"빌드 실패 (code={exit_code})")

    def on_start(self):
        if not self.is_installed():
            QMessageBox.warning(self, "경고", "먼저 초기 설치를 진행하세요.")
            return
        if not self._is_valid_project_root(self.project_root):
            QMessageBox.warning(self, "프로젝트 필요", "프로젝트가 아직 준비되지 않았습니다. 설치를 먼저 진행하세요.")
            return
        if not docker_compose_available():
            QMessageBox.critical(self, "오류", self._compose_help_text())
            return
        if not self._apply_compose_variant(self.install_variant):
            QMessageBox.critical(self, "오류", "선택한 설치 옵션에 맞는 docker-compose 템플릿을 찾을 수 없습니다.")
            return
        self._clear_conflicting_containers()
        self._show_preload_dialog("서비스 준비중...")
        self.append_log("[START] docker compose clean start (down -> up -d) ...")
        self._run_compose_sequence(
            [
                ["down", "--remove-orphans"],
                ["up", "-d", "service"],
            ],
            on_finish=self._on_start_finished,
        )

    def _on_start_finished(self, exit_code: int, *_):
        if exit_code == 0:
            self.append_log("[START] 완료")
            self.update_state_label()
            self._wait_for_services_ready(self.load_ui)
        else:
            self._hide_preload_dialog()
            QMessageBox.critical(self, "오류", f"시작 실패 (code={exit_code})")

    def on_stop(self):
        if not self._is_valid_project_root(self.project_root):
            QMessageBox.warning(self, "프로젝트 필요", "프로젝트가 아직 준비되지 않았습니다. 설치를 먼저 진행하세요.")
            return
        if not docker_compose_available():
            QMessageBox.critical(self, "오류", self._compose_help_text())
            return
        self.append_log("[STOP] docker compose down --remove-orphans ...")
        self.run_compose(["down", "--remove-orphans"], on_finish=self._on_stop_finished)

    def _on_stop_finished(self, exit_code: int):
        if exit_code == 0:
            self.append_log("[STOP] 완료")
            self.update_state_label()
        else:
            QMessageBox.critical(self, "오류", f"중지 실패 (code={exit_code})")

    def on_open_ui(self):
        if not self._is_valid_project_root(self.project_root):
            QMessageBox.warning(self, "프로젝트 필요", "프로젝트가 아직 준비되지 않았습니다. 설치를 먼저 진행하세요.")
            return
        self.load_ui()

    def on_select_dev_src(self):
        folder = QFileDialog.getExistingDirectory(self, "원본 프로젝트 루트 선택 (src/backend, src/ui 포함)", str(self.dev_src_root or Path.home()))
        if not folder:
            return
        path = Path(folder)
        if not self._is_valid_dev_src(path):
            QMessageBox.warning(self, "잘못된 경로", "선택한 폴더에 src/backend 또는 src/ui가 없습니다.")
            return
        self.dev_src_root = path
        cfg = load_config()
        cfg["dev_src_root"] = str(self.dev_src_root)
        save_config(cfg)
        self.update_dev_src_label()
        self.update_buttons()

    def on_apply_and_restart(self):
        self.append_log("[SYNC] 적용 중: backend/ui/compose 복사...")
        if not self._sync_dev_files():
            return
        if not self._apply_compose_variant(self.install_variant):
            QMessageBox.critical(self, "오류", "선택한 설치 옵션에 맞는 docker-compose 템플릿을 찾을 수 없습니다.")
            return
        self.append_log("[SYNC] 완료. 서비스 재시작 중...")
        self._show_preload_dialog("서비스 준비중...")
        def _after_restart(exit_code: int, *_):
            self.append_log("[RESTART] 완료")
            if exit_code == 0:
                self._wait_for_services_ready(self.load_ui)
            else:
                self._hide_preload_dialog()
        self._clear_conflicting_containers()
        self._run_compose_sequence(
            [
                ["down", "--remove-orphans"],
                ["up", "-d", "service"],
            ],
            on_finish=_after_restart,
        )

    def _quick_apply_script_candidates(self) -> list[Path]:
        return [
            REPO_ROOT_CANDIDATE / "scripts" / "quick_apply.sh",
            Path("/opt/easytrainer/scripts/quick_apply.sh"),
            APP_HOME / "scripts" / "quick_apply.sh",
            self.project_root / "scripts" / "quick_apply.sh",
        ]

    def _resolve_quick_apply_script(self) -> Path | None:
        for cand in self._quick_apply_script_candidates():
            try:
                if cand.is_file() and os.access(cand, os.X_OK):
                    return cand
            except Exception:
                continue
        return None

    def _sync_dirs(self, src: Path, dst: Path):
        dst.mkdir(parents=True, exist_ok=True)
        for item in src.rglob("*"):
            rel = item.relative_to(src)
            target = dst / rel
            if item.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(item, target)

    def _sync_core_files(self):
        if not self.dev_src_root:
            return
        files = [
            "docker-compose.yml",
            "docker-compose.dev.yml",
            "docker-compose.cpu.yml",
            "docker-compose.gpu.yml",
            "start_services.sh",
            "Dockerfile",
            ".dockerignore",
            "requirements.txt",
            "requirements.min.txt",
        ]
        for name in files:
            src = self.dev_src_root / name
            if not src.exists():
                continue
            dst = self.project_root / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)

    def _sync_dev_files(self, show_errors: bool = True) -> bool:
        if not self._is_valid_project_root(self.project_root):
            if show_errors:
                QMessageBox.warning(self, "프로젝트 필요", "프로젝트가 아직 준비되지 않았습니다.")
            return False
        if not self._is_valid_dev_src(self.dev_src_root):
            if show_errors:
                QMessageBox.warning(self, "원본 필요", "원본 프로젝트 경로를 먼저 선택하세요.")
            return False
        script = self._resolve_quick_apply_script()
        if script:
            try:
                self.append_log(f"[SYNC] quick_apply 실행: {script}")
            except Exception:
                pass
            try:
                result = subprocess.run(
                    [str(script), str(self.dev_src_root), str(self.project_root)],
                    capture_output=True,
                    text=True,
                )
            except Exception as e:
                if show_errors:
                    QMessageBox.critical(self, "오류", f"동기화 스크립트 실행 실패: {e}")
                else:
                    self.append_log(f"[SYNC][ERROR] 스크립트 실행 실패: {e}")
                return False
            for stream_label, payload in (("OUT", result.stdout), ("ERR", result.stderr)):
                if not payload:
                    continue
                for line in payload.splitlines():
                    self.append_log(f"[SYNC][{stream_label}] {line}")
            if result.returncode == 0:
                self._apply_compose_variant(self.install_variant)
                return True
            if show_errors:
                QMessageBox.critical(self, "오류", "[SYNC] quick_apply 스크립트가 실패했습니다. 로그를 확인하세요.")
            else:
                self.append_log("[SYNC][ERROR] quick_apply 스크립트가 실패했습니다.")
            return False
        # Fallback to in-app copy when script is missing (legacy behaviour)
        try:
            self.append_log("[SYNC][WARN] quick_apply 스크립트를 찾을 수 없어 내부 복사를 사용합니다.")
            self._sync_dirs(self.dev_src_root / "src" / "backend", self.project_root / "src" / "backend")
            self._sync_dirs(self.dev_src_root / "src" / "ui", self.project_root / "src" / "ui")
            self._sync_core_files()
            self._apply_compose_variant(self.install_variant)
            return True
        except Exception as e:
            if show_errors:
                QMessageBox.critical(self, "오류", f"동기화 실패: {e}")
            else:
                self.append_log(f"[SYNC][ERROR] {e}")
            return False

    def _show_preload_dialog(self, message: str):
        if self._preload_dialog is None:
            self._preload_dialog = ServiceSplash(self)
        text = message or "서비스 준비중..."
        self._preload_dialog.set_message(text)
        self._preload_dialog.set_detail("프론트엔드: 대기중 | 백엔드: 대기중")
        self._preload_dialog.show()
        self._preload_dialog.raise_()

    def _set_preload_detail(self, text: str):
        if self._preload_dialog:
            self._preload_dialog.set_detail(text)

    def _hide_preload_dialog(self, ready: bool = False):
        self._stop_ready_timer()
        self._stop_ready_timeout()
        if self._preload_dialog:
            self._preload_dialog.hide()
        if ready:
            self._ensure_main_window_visible()

    def _stop_ready_timer(self):
        if self._ready_check_timer:
            self._ready_check_timer.stop()
            self._ready_check_timer = None

    def _stop_ready_timeout(self):
        if self._ready_timeout_timer:
            try:
                self._ready_timeout_timer.stop()
            except Exception:
                pass
            self._ready_timeout_timer = None

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
        return 45000  # default 45s

    def _wait_for_services_ready(self, on_ready=None):
        if on_ready is None:
            on_ready = self.load_ui
        self._stop_ready_timer()
        self._stop_ready_timeout()
        self._show_preload_dialog("서비스 준비중...")

        def poll():
            backend_ok = self._check_backend_ready()
            frontend_ok = self._is_frontend_up()
            detail = f"프론트엔드: {'준비완료' if frontend_ok else '대기중'} | 백엔드: {'준비완료' if backend_ok else '대기중'}"
            self._set_preload_detail(detail)
            if backend_ok and frontend_ok:
                self._stop_ready_timer()
                self._stop_ready_timeout()
                self._hide_preload_dialog(ready=True)
                try:
                    on_ready()
                except Exception:
                    pass

        self._ready_check_timer = QTimer(self)
        self._ready_check_timer.setInterval(1000)
        self._ready_check_timer.timeout.connect(poll)
        poll()
        if self._ready_check_timer is not None:
            self._ready_check_timer.start()

        # Fail fast if services never become healthy
        timeout_ms = self._resolve_ready_timeout_ms()
        self._ready_timeout_timer = QTimer(self)
        self._ready_timeout_timer.setSingleShot(True)
        self._ready_timeout_timer.setInterval(timeout_ms)
        self._ready_timeout_timer.timeout.connect(self._on_ready_timeout)
        self._ready_timeout_timer.start()

    def _check_backend_ready(self) -> bool:
        try:
            from urllib import request
            req = request.Request("http://127.0.0.1:5000/api/healthz", method="GET")
            with request.urlopen(req, timeout=1.0):
                return True
        except Exception:
            return False

    def _is_frontend_up(self) -> bool:
        try:
            from urllib.parse import urlparse
            import socket
            u = urlparse(FRONTEND_URL)
            host = u.hostname or "localhost"
            port = u.port or (443 if (u.scheme or "http").lower() == "https" else 80)
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except Exception:
            return False

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

        stop_triggered = {"done": False}

        def _stop_services():
            if stop_triggered["done"]:
                return
            stop_triggered["done"] = True
            self.append_log("[STOP] 로그 창 종료로 서비스 중지 중 ...")
            # Try synchronous compose down first
            try:
                subprocess.run(
                    ["docker", "compose", "down", "--remove-orphans"],
                    cwd=str(self.project_root),
                    check=False,
                    timeout=20,
                    capture_output=True,
                    text=True,
                )
            except Exception:
                pass
            # Ensure container is gone even if compose down failed/wasn't running
            try:
                subprocess.run(
                    ["docker", "rm", "-f", "easy_collector_service"],
                    check=False,
                    timeout=10,
                    capture_output=True,
                    text=True,
                )
            except Exception:
                pass

        def _cleanup(*_):
            try:
                proc.terminate()
                proc.waitForFinished(2000)
                proc.kill()
            except Exception:
                pass
            _stop_services()
            if on_close and not stop_triggered.get("closed"):
                stop_triggered["closed"] = True
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
        self._hide_preload_dialog()
        self.append_log("[ERROR] 서비스가 제시간에 준비되지 않아 중지되었습니다.")
        logs = self._collect_docker_logs("easy_collector_service", tail=200)
        self._show_docker_logs_follow(
            "서비스 시작 실패",
            container="easy_collector_service",
            tail=200,
            initial_text=logs,
            on_close=lambda: QApplication.quit(),
        )

    def load_ui(self):
        url = QUrl(FRONTEND_URL)

        def _open():
            if HAS_WEBENGINE and self.web_view is not None:
                self.web_view.setUrl(url)
            else:
                QDesktopServices.openUrl(url)
            self.append_log(f"[WEB] Open {url.toString()}")

        if self._is_frontend_up():
            _open()
            try:
                self._update_nav_buttons()
            except Exception:
                pass
            return

        self.append_log(f"[WEB] Frontend not ready. Waiting for {FRONTEND_URL} ...")
        # Poll until server is up, then open once
        self._frontend_timer = QTimer(self)
        self._frontend_timer.setInterval(1000)
        def _poll():
            if self._is_frontend_up():
                try:
                    self._frontend_timer.stop()
                except Exception:
                    pass
                _open()
                try:
                    self._update_nav_buttons()
                except Exception:
                    pass
        self._frontend_timer.timeout.connect(_poll)
        self._frontend_timer.start()

    def _quick_apply_from_dev_src(self):
        if not self._is_valid_dev_src(self.dev_src_root):
            self.load_ui()
            return
        self.append_log("[SYNC] 원본에서 빠른 적용 중 (재시작 없음)...")
        if not self._sync_dev_files(show_errors=False):
            self.append_log("[SYNC][WARN] 원본 적용에 실패했습니다. 경로를 확인하세요.")
            return
        self.append_log("[SYNC] 빠른 적용 완료. UI 새로고침 중...")
        self.load_ui()

    def _nav_action(self, action: str):
        try:
            if not HAS_WEBENGINE or self.web_view is None:
                return
            if action == "back":
                try:
                    self.web_view.back()
                except Exception:
                    self.web_view.history().back()
            elif action == "forward":
                try:
                    self.web_view.forward()
                except Exception:
                    self.web_view.history().forward()
        finally:
            try:
                self._update_nav_buttons()
            except Exception:
                pass

    def _on_quick_apply_clicked(self):
        if not self._is_valid_dev_src(self.dev_src_root):
            QMessageBox.information(self, "원본 필요", "먼저 원본 프로젝트 경로를 선택하세요.")
            self.on_select_dev_src()
            if not self._is_valid_dev_src(self.dev_src_root):
                return
        self._quick_apply_from_dev_src()

    def _update_nav_buttons(self):
        try:
            if not HAS_WEBENGINE or self.web_view is None:
                for b in (self.btn_nav_back, self.btn_nav_forward):
                    b.setEnabled(False)
                return
            hist = self.web_view.history()
            self.btn_nav_back.setEnabled(hist.canGoBack())
            self.btn_nav_forward.setEnabled(hist.canGoForward())
        except Exception:
            pass

    def _dev_apply_and_restart_and_reload(self):
        self.append_log("[SYNC] 개발 원본에서 backend/ui/compose 반영...")
        if not self._sync_dev_files():
            return
        self.append_log("[SYNC] 완료. 서비스 재시작 중...")
        self._show_preload_dialog("서비스 준비중...")
        def _after_restart(exit_code: int, *_):
            self.append_log("[RESTART] 완료")
            if exit_code == 0:
                self._wait_for_services_ready(self.load_ui)
            else:
                self._hide_preload_dialog()
        self._clear_conflicting_containers()
        self._run_compose_sequence(
            [
                ["down", "--remove-orphans"],
                ["up", "-d", "service"],
            ],
            on_finish=_after_restart,
        )

    def open_logs_window(self):
        svc = "service"
        backend_log = shlex.quote(str(SERVICE_LOG_DIR / "backend.log"))
        frontend_log = shlex.quote(str(SERVICE_LOG_DIR / "frontend.log"))
        tail_backend = ["exec", "-T", svc, "bash", "-lc", f"tail -n 200 -F {backend_log}"]
        tail_frontend = ["exec", "-T", svc, "bash", "-lc", f"tail -n 200 -F {frontend_log}"]
        backend = self._create_log_dialog("Backend Logs", tail_backend)
        frontend = self._create_log_dialog("Frontend Logs", tail_frontend)
        ui_log = self._create_ui_log_dialog("Launcher Logs", UI_LOG_FILE)

        ordered = []
        for dlg in (ui_log, frontend, backend):
            if dlg is None:
                continue
            self._register_log_window(dlg)
            ordered.append(dlg)

        self._arrange_log_windows_side_by_side(ordered)
        for dlg in ordered:
            dlg.show()

    def _create_log_dialog(self, title: str, compose_args: list[str]) -> QDialog | None:
        dlg = QDialog(self)
        dlg.setWindowTitle(title)
        dlg.resize(600, 420)
        log_widget = QTextEdit(dlg)
        log_widget.setReadOnly(True)
        self._apply_log_font(log_widget)
        layout = QVBoxLayout(dlg)
        layout.addWidget(log_widget)
        self.run_compose_to_widget_parent(dlg, compose_args, log_widget)
        return dlg

    def _register_log_window(self, dlg: QDialog):
        self._log_windows.append(dlg)
        def _cleanup(*_):
            try:
                self._log_windows.remove(dlg)
            except ValueError:
                pass
        try:
            dlg.finished.connect(_cleanup)
        except Exception:
            pass
        dlg.destroyed.connect(_cleanup)

    def _create_ui_log_dialog(self, title: str, log_file: Path) -> QDialog | None:
        dlg = QDialog(self)
        dlg.setWindowTitle(title)
        dlg.resize(600, 420)
        viewer = QPlainTextEdit(dlg)
        viewer.setReadOnly(True)
        self._apply_log_font(viewer)
        layout = QVBoxLayout(dlg)
        layout.addWidget(viewer)
        state = {"pos": 0}

        def _move_to_end():
            try:
                from PySide6.QtGui import QTextCursor as _QC
                viewer.moveCursor(_QC.End)
            except Exception:
                try:
                    from PyQt6.QtGui import QTextCursor as _QC2
                    viewer.moveCursor(_QC2.MoveOperation.End)
                except Exception:
                    pass

        def load_initial():
            try:
                with log_file.open("r", encoding="utf-8", errors="ignore") as f:
                    data = f.read()
                    state["pos"] = f.tell()
            except FileNotFoundError:
                data = "[UI LOG] 로그 파일이 아직 생성되지 않았습니다."
            except Exception as e:
                data = f"[UI LOG] 로그를 읽을 수 없습니다: {e}"
            viewer.setPlainText(data)
            _move_to_end()

        def tail_update():
            try:
                with log_file.open("r", encoding="utf-8", errors="ignore") as f:
                    f.seek(state["pos"])
                    chunk = f.read()
                    state["pos"] = f.tell()
                    if chunk:
                        _move_to_end()
                        viewer.insertPlainText(chunk)
                        _move_to_end()
            except FileNotFoundError:
                pass
            except Exception:
                pass

        load_initial()
        timer = QTimer(dlg)
        timer.setInterval(1000)
        timer.timeout.connect(tail_update)
        timer.start()
        dlg._ui_log_timer = timer  # type: ignore[attr-defined]
        return dlg

    def _apply_log_font(self, widget):
        try:
            font = QFont("Monospace")
            font.setStyleHint(QFont.Monospace)
            size = max(font.pointSize() - 2, 9)
            if size <= 0:
                size = 9
            font.setPointSize(size)
            widget.setFont(font)
        except Exception:
            pass

    def _arrange_log_windows_side_by_side(self, dialogs: list[QDialog]):
        if not dialogs:
            return
        spacing = 24
        if self.isVisible():
            base_geo = self.frameGeometry()
            start_x = base_geo.left()
            y = base_geo.top()
        else:
            screen = QApplication.primaryScreen()
            geo = screen.availableGeometry() if screen else None
            start_x = geo.left() if geo else 100
            y = geo.top() if geo else 100
        for dlg in dialogs:
            dlg.move(max(start_x, 0), max(y, 0))
            start_x += dlg.width() + spacing

    # ------------------------ Process helpers ------------------------
    def run_compose(self, args: list[str], on_finish=None):
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            QMessageBox.information(self, "실행중", "다른 작업이 실행 중입니다. 잠시만 기다려 주세요.")
            return

        program, prefix = get_compose_cmd()
        if not program:
            QMessageBox.critical(self, "오류", self._compose_help_text())
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

    def run_compose_with_files(self, compose_files: list[str], args: list[str], on_finish=None):
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            QMessageBox.information(self, "실행중", "다른 작업이 실행 중입니다. 잠시만 기다려 주세요.")
            return
        program, prefix = get_compose_cmd()
        if not program:
            QMessageBox.critical(self, "오류", self._compose_help_text())
            return
        file_args: list[str] = []
        for f in compose_files:
            file_args.extend(["-f", f])
        self.process = QProcess(self)
        self.process.setProgram(program)
        self.process.setArguments([*prefix, *file_args, *args])
        self.process.setWorkingDirectory(str(self.project_root))
        self.process.readyReadStandardOutput.connect(lambda: self._read_stream(self.process, False))
        self.process.readyReadStandardError.connect(lambda: self._read_stream(self.process, True))
        if on_finish:
            self.process.finished.connect(on_finish)
        self.process.finished.connect(lambda *_: self.update_state_label())
        self.process.start()

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

    def run_compose_to_widget_with_files(self, compose_files: list[str], args: list[str], log_widget: QTextEdit, on_finish=None):
        proc = QProcess(self)
        program, prefix = get_compose_cmd()
        if not program:
            log_widget.append(self._compose_help_text())
            return
        file_args: list[str] = []
        for f in compose_files:
            file_args.extend(["-f", f])
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
        return (
            "[ERROR] Docker Compose(v2)를 찾을 수 없습니다.\n"
            "설치 방법(권장):\n"
            "  sudo apt-get update\n"
            "  sudo apt-get install -y docker.io docker-compose-plugin\n"
            "버전 확인: docker compose version\n"
            "레거시(v1) 바이너리 사용 시: docker-compose 설치 또는 PATH 설정이 필요합니다.\n"
            "사용자를 docker 그룹에 추가:\n"
            "  sudo usermod -aG docker $USER && newgrp docker\n"
        )

    def _read_stream(self, proc: QProcess, is_err: bool):
        try:
            data = proc.readAllStandardError() if is_err else proc.readAllStandardOutput()
            text = bytes(data).decode(errors="ignore")
            if text:
                self.append_log(text.rstrip("\n"))
        except Exception:
            pass

    def append_log(self, msg: str):
        self.log.append(msg)
        # Persist to text file (best-effort)
        try:
            UI_LOG_DIR.mkdir(parents=True, exist_ok=True)
            with UI_LOG_FILE.open("a", encoding="utf-8") as f:
                f.write(msg + "\n")
        except Exception:
            pass
        # Move cursor to end (Qt5/Qt6 compatible)
        try:
            from PySide6.QtGui import QTextCursor as _QC
            self.log.moveCursor(_QC.End)
        except Exception:
            try:
                from PyQt6.QtGui import QTextCursor as _QC2
                try:
                    self.log.moveCursor(_QC2.MoveOperation.End)
                except Exception:
                    pass
            except Exception:
                pass


def main():
    os.environ.setdefault("QT_QPA_PLATFORMTHEME", "")
    os.environ.setdefault("QT_STYLE_OVERRIDE", "Fusion")
    os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
    app = QApplication(sys.argv)
    # Dark theme ~ rgb(30,30,30) with white accents
    app.setStyleSheet(
        """
        QWidget { background-color: #1E1E1E; color: #EEEEEE; }
        QFrame#Divider { background-color: #FFFFFF; max-height: 1px; }
        QFrame#ContentBox { border: 1px solid #FFFFFF; border-radius: 6px; }
        QListWidget { background-color: #232323; border: 1px solid #333333; padding: 8px; }
        QLineEdit, QTextEdit { background-color: #232323; color: #EEEEEE; border: 1px solid #3A3A3A; border-radius: 4px; }
        QPushButton { background-color: #2A2A2A; color: #EEEEEE; border: 1px solid #555555; border-radius: 4px; padding: 4px 10px; min-width: 60px; min-height: 24px; }
        QPushButton:disabled { color: #888888; border-color: #333333; }
        QProgressBar { background-color: #232323; border: 1px solid #333333; color: #EEEEEE; text-align: center; }
        QProgressBar::chunk { background-color: #4a90e2; }
        """
    )
    # App-wide icon
    try:
        icon = _window_icon()
        if icon:
            app.setWindowIcon(icon)
    except Exception:
        pass
    if not docker_compose_available():
        QMessageBox.critical(None, "오류", "Docker가 설치되어 있지 않거나 PATH에 없습니다.")
        return 1

    win = MainWindow()
    # Run setup wizard if needed before showing main window; abort if not completed
    if not win.maybe_run_setup_wizard():
        return 0
    fullscreen = os.environ.get("EASYCOLLECTOR_FULLSCREEN", "0") == "1"
    win.set_display_mode(fullscreen)
    if not win.auto_launch_enabled():
        return 0
    QTimer.singleShot(0, win.on_start)
    return app.exec()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        # As a last resort, print to stdout (captured by launcher log)
        print(f"[FATAL] Launcher crashed: {e}", file=sys.stderr)
        raise

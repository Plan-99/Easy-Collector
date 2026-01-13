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
import sqlite3
from pathlib import Path

# Quiet noisy GTK module warnings inherited from some environments
_gtk_modules = os.environ.get("GTK_MODULES", "")
if _gtk_modules:
    _filtered = [m for m in _gtk_modules.split(":") if m not in ("atk-bridge", "atkbridge")]
    if _filtered != _gtk_modules.split(":"):
        if _filtered:
            os.environ["GTK_MODULES"] = ":".join(_filtered)
        else:
            os.environ.pop("GTK_MODULES", None)

# Apply safe Qt WebEngine defaults BEFORE importing Qt modules
_force_external_env = os.environ.get("EASYCOLLECTOR_FORCE_EXTERNAL_BROWSER")
# Default to using an external browser (no embedded WebView) unless explicitly disabled
_force_external = True if _force_external_env is None else (_force_external_env == "1")
# On Wayland, default to X11/xcb to avoid noisy activation token warnings
if os.environ.get("XDG_SESSION_TYPE", "").lower() == "wayland" and not os.environ.get("QT_QPA_PLATFORM"):
    os.environ["QT_QPA_PLATFORM"] = "xcb"
if not _force_external:
    # Disable sandbox and GPU to avoid common crashes
    os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
    os.environ.setdefault("QT_OPENGL", "software")
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    # Force X11 on Wayland sessions
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
QWebEngineView = None  # type: ignore
QWebEngineProfile = None  # type: ignore
QWebEnginePage = None  # type: ignore

try:
    from PySide6.QtCore import Qt, QUrl, QProcess, QTimer, QPoint, QEvent, QVariantAnimation, QEasingCurve
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QPushButton, QTextEdit, QLabel,
        QVBoxLayout, QHBoxLayout, QMessageBox, QTabWidget, QFileDialog, QLineEdit,
        QDialog, QFrame, QListWidget, QListWidgetItem, QProgressBar, QToolButton,
        QMenu, QPlainTextEdit, QSizePolicy, QRadioButton, QCheckBox, QStackedLayout, QButtonGroup, QInputDialog
    )
    if not _force_external:
        try:
            from PySide6.QtWebEngineWidgets import QWebEngineView  # optional
            from PySide6.QtWebEngineCore import QWebEngineProfile, QWebEnginePage
            HAS_WEBENGINE = True
        except Exception:
            HAS_WEBENGINE = False
    from PySide6.QtGui import (
        QDesktopServices,
        QIcon,
        QPixmap,
        QFont,
        QCursor,
        QPainterPath,
        QRegion,
        QPainter,
        QColor,
        QPen,
    )
except Exception:
    from PyQt6.QtCore import Qt, QUrl, QProcess, QTimer, QPoint, QEvent, QVariantAnimation, QEasingCurve
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QPushButton, QTextEdit, QLabel,
        QVBoxLayout, QHBoxLayout, QMessageBox, QTabWidget, QFileDialog, QLineEdit,
        QDialog, QFrame, QListWidget, QListWidgetItem, QProgressBar, QToolButton,
        QMenu, QPlainTextEdit, QSizePolicy, QRadioButton, QCheckBox, QStackedLayout, QButtonGroup, QInputDialog
    )
    if not _force_external:
        try:
            from PyQt6.QtWebEngineWidgets import QWebEngineView  # optional
            from PyQt6.QtWebEngineCore import QWebEngineProfile, QWebEnginePage
            HAS_WEBENGINE = True
        except Exception:
            HAS_WEBENGINE = False
    from PyQt6.QtGui import (
        QDesktopServices,
        QIcon,
        QPixmap,
        QFont,
        QCursor,
        QPainterPath,
        QRegion,
        QPainter,
        QColor,
        QPen,
    )


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


def _load_app_version() -> str:
    """Resolve app version from bundled file or repo root."""
    candidates = [
        DATA_ROOT / "VERSION",
        Path(__file__).resolve().parents[1] / "VERSION",
        REPO_ROOT_CANDIDATE / "VERSION",
    ]
    for p in candidates:
        try:
            if p.is_file():
                ver = p.read_text(encoding="utf-8").strip()
                if ver:
                    return ver
        except Exception:
            continue
    return "0.0.0"


APP_VERSION = _load_app_version()


class ServiceSplash(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setModal(True)
        self.setFixedSize(360, 150)
        flags = self.windowFlags()
        flags = (flags & ~Qt.WindowContextHelpButtonHint) | Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint
        self.setWindowFlags(flags)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 2, 6, 4)
        layout.setSpacing(2)
        self.message_label = QLabel("Easy Trainer 준비중...", self)
        self.message_label.setAlignment(Qt.AlignCenter)
        self.message_label.setStyleSheet("font-weight: 600; margin: 0; padding: 0;")
        layout.addWidget(self.message_label)
        try:
            self.restart_button.setEnabled(True)
        except Exception:
            pass
        self.progress = QProgressBar(self)
        self.progress.setRange(0, 0)  # indefinite
        self.progress.setFixedHeight(32)
        layout.addWidget(self.progress)
        self.detail_label = QLabel("", self)
        self.detail_label.setAlignment(Qt.AlignCenter)
        self.detail_label.setStyleSheet("color: #aaa; font-size: 12px; font-weight: 600; margin: 0; padding: 0;")
        layout.addWidget(self.detail_label)
        self.restart_button = QPushButton("서비스 재실행", self)
        self.restart_button.setVisible(False)
        self.restart_button.setFixedHeight(22)
        self.restart_button.setStyleSheet("padding: 2px 6px; font-size: 11px;")
        layout.addWidget(self.restart_button)
        self._restart_handler = None

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

    def set_restart_handler(self, handler):
        try:
            if self._restart_handler:
                self.restart_button.clicked.disconnect(self._restart_handler)
        except Exception:
            pass
        try:
            self.restart_button.clicked.connect(handler)
            self._restart_handler = handler
        except Exception:
            pass

    def show_restart(self, show: bool, enabled: bool = True):
        self.restart_button.setVisible(show)
        self.restart_button.setEnabled(enabled)


class PillLoadingRing(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._dash_offset = 0.0
        self._timer = QTimer(self)
        self._timer.setInterval(30)
        self._timer.timeout.connect(self._tick)
        try:
            self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            self.setAttribute(Qt.WA_TranslucentBackground, True)
        except Exception:
            pass
        self.hide()

    def start(self):
        if not self._timer.isActive():
            self._timer.start()
        self.show()
        try:
            self.raise_()
        except Exception:
            pass

    def stop(self):
        if self._timer.isActive():
            self._timer.stop()
        self.hide()

    def _tick(self):
        self._dash_offset = (self._dash_offset + 1.8) % 200.0
        self.update()

    def paintEvent(self, event):
        rect = self.rect()
        if rect.width() <= 2 or rect.height() <= 2:
            return
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing, True)
        except Exception:
            pass
        pen_width = 2.0
        pen = QPen(QColor("#7dd3fc"), pen_width)
        try:
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)
            pen.setDashPattern([6.0, 6.0])
            pen.setDashOffset(self._dash_offset)
        except Exception:
            pass
        painter.setPen(pen)
        try:
            inset = int(pen_width / 2) + 1
            inner = rect.adjusted(inset, inset, -inset, -inset)
            radius = min(24.0, inner.width() / 2.0, inner.height() / 2.0)
            painter.drawRoundedRect(inner, radius, radius)
        finally:
            painter.end()


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
        self.setObjectName("LauncherWindow")
        self.install_variant = "gpu"
        self.project_root: Path = DEFAULT_PROJECT_PATH
        self._update_window_title()
        try:
            icon = _window_icon()
            if icon:
                self.setWindowIcon(icon)
        except Exception:
            pass
        self.resize(240, 480)

        self.process: QProcess | None = None
        self._preload_dialog: ServiceSplash | None = None
        self._ready_check_timer: QTimer | None = None
        self._ready_timeout_timer: QTimer | None = None
        self._restart_btn_timer: QTimer | None = None
        self._inline_log_proc: QProcess | None = None
        self._inline_log_retry: QTimer | None = None
        self._inline_log_initialized: bool = False
        self._main_visible = False
        self._fullscreen_pref = False
        self._closing = False
        self._pending_quick_apply = False
        self._log_windows: list[QDialog] = []
        self._auto_launch_after_install = True
        self._devtools_view: QWebEngineView | None = None
        self._devtools_dialog: QDialog | None = None
        self._floating_user_moved = False
        self._dragging = False
        self._drag_offset = QPoint(0, 0)
        self._anchor_left: int | None = None  # screen-left anchor for animations
        self._pill_anim: QVariantAnimation | None = None
        self._pill_expanded: bool = False
        self._pill_loading: bool = False
        self._pill_loading_ring: PillLoadingRing | None = None
        self._pad_loading_panel: QWidget | None = None
        self._pad_loading_front_label: QLabel | None = None
        self._pad_loading_back_label: QLabel | None = None
        self._pad_loading_title: QLabel | None = None
        self._pad_status_front_label: QLabel | None = None
        self._pad_status_back_label: QLabel | None = None
        self._pad_status_front_dot: QLabel | None = None
        self._pad_status_back_dot: QLabel | None = None
        self._preload_message: str = ""
        self._preload_detail: str = ""
        self._preload_auto_open_new: bool = True
        self._restart_in_progress: bool = False
        self._restart_phase: str = ""
        self._app_home_writable_fixed = False
        self._collapse_timer = QTimer(self)
        self._collapse_timer.setSingleShot(True)
        self._collapse_timer.timeout.connect(self._collapse_if_needed)
        self._hover_watch_timer = QTimer(self)
        self._hover_watch_timer.setInterval(50)
        self._hover_watch_timer.timeout.connect(self._sync_hover_state)
        self._hover_watch_timer.start()
        self._pad_desc_timer = QTimer(self)
        self._pad_desc_timer.setSingleShot(True)
        self._pad_desc_timer.timeout.connect(self._on_pad_desc_timeout)
        self._pad_desc_window_active = False
        self._pill_hover_timer = QTimer(self)
        self._pill_hover_timer.setSingleShot(True)
        self._pill_hover_timer.timeout.connect(self._on_pill_hover_timeout)
        self._pill_hover_target = None
        self._pill_hover_target_index: int | None = None
        self._pad_force_mode_index: int | None = None
        self._pad_notice_timer = QTimer(self)
        self._pad_notice_timer.setSingleShot(True)
        self._pad_notice_timer.timeout.connect(self._clear_pad_notice)
        self._pad_notice_overrides: dict[int, tuple[str, str]] = {}
        self._dev_src_prompt_timer = QTimer(self)
        self._dev_src_prompt_timer.setSingleShot(True)
        self._dev_src_prompt_timer.timeout.connect(self._prompt_dev_src_then_apply)
        self._pad_import_choice = "DB"
        self._pad_export_choice = "DB"
        self._exit_action = "EXIT"
        self._open_ui_mode = "NEW"
        self._disable_circle_tooltips = True
        self._pad_width = 300
        self._button_size = 60
        self._pad_item_height = self._button_size
        self.dev_src_root: Path | None = None

        # UI elements
        self.status_label = QLabel(self)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #cfd8dc; font-size: 11px; font-weight: 600;")
        self.status_label.hide()

        # Hidden log buffer for append_log helper
        self.log = QTextEdit(self)
        self.log.setReadOnly(True)
        self.log.setVisible(False)

        # No embedded web view; always prefer external browser
        self.web_view = None

        # Floating pill bar UI
        self.pill = QFrame(self)
        self.pill.setObjectName("FloatingBar")
        pill_layout = QVBoxLayout(self.pill)
        pill_layout.setContentsMargins(7, 8, 7, 8)
        pill_layout.setSpacing(6)
        pill_layout.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.pill.installEventFilter(self)
        self.pill.setStyleSheet("QFrame#FloatingBar { background: rgb(30,30,30); border-radius: 24px; }")

        self._frontend_light = QLabel(self.pill)
        self._backend_light = QLabel(self.pill)
        for light in (self._frontend_light, self._backend_light):
            light.setFixedSize(8, 8)
            try:
                light.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            except Exception:
                pass
        self._position_status_lights()
        self._pill_loading_ring = PillLoadingRing(self.pill)
        self._position_loading_ring()

        # Hidden labels kept for compatibility with existing update methods
        self.title_label = QLabel("", self.pill)
        self.title_label.hide()
        self.path_hint_label = QLabel("", self.pill)
        self.path_hint_label.hide()

        circle_icons = self._circle_icon_paths()
        self.btn_open_browser = self._create_circle_button(
            "↗",
            "UI 열기",
            circle_icons[0] if len(circle_icons) > 0 else None,
        )
        self.btn_open_browser.clicked.connect(self.on_open_ui)
        pill_layout.addWidget(self.btn_open_browser, 0, Qt.AlignHCenter)

        self.btn_quick_apply = self._create_circle_button(
            "⇆",
            "빠른 동기화",
            circle_icons[1] if len(circle_icons) > 1 else None,
        )
        self.btn_quick_apply.clicked.connect(self._on_quick_apply_clicked)
        pill_layout.addWidget(self.btn_quick_apply, 0, Qt.AlignHCenter)

        self.btn_folder = self._create_circle_button(
            "📁",
            "불러오기",
            circle_icons[2] if len(circle_icons) > 2 else None,
        )
        self.btn_folder.clicked.connect(self._on_import_clicked)
        pill_layout.addWidget(self.btn_folder, 0, Qt.AlignHCenter)

        self.btn_logs = self._create_circle_button(
            "🧾",
            "내보내기",
            circle_icons[3] if len(circle_icons) > 3 else None,
        )
        self.btn_logs.clicked.connect(self._on_export_clicked)
        pill_layout.addWidget(self.btn_logs, 0, Qt.AlignHCenter)

        self.btn_settings = self._create_circle_button(
            "⚙",
            "로그",
            circle_icons[4] if len(circle_icons) > 4 else None,
        )
        self.btn_settings.clicked.connect(self.open_logs_window)
        pill_layout.addWidget(self.btn_settings, 0, Qt.AlignHCenter)

        self.btn_exit = self._create_circle_button(
            "✕",
            "서비스 종료",
            circle_icons[5] if len(circle_icons) > 5 else None,
        )
        self.btn_exit.clicked.connect(self._on_exit_action)
        pill_layout.addWidget(self.btn_exit, 0, Qt.AlignHCenter)

        self.state_label = QLabel("", self.pill)
        self.state_label.hide()

        wrapper = QWidget(self)
        wrapper.setObjectName("PillWrapper")
        wrapper_layout = QHBoxLayout(wrapper)
        wrapper_layout.setContentsMargins(0, 0, 0, 0)
        wrapper_layout.setSpacing(0)
        wrapper_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        # Pad grows right from the pill center
        self._pad_bridge = QFrame(wrapper)
        self._pad_bridge.setObjectName("FloatingBarBridge")
        self._pad_bridge.setVisible(False)
        try:
            self._pad_bridge.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        except Exception:
            pass
        self._bridge_segments: list[QFrame] = []
        for _ in range(6):
            seg = QFrame(self._pad_bridge)
            seg.setObjectName("FloatingBarBridgeSegment")
            seg.setVisible(False)
            try:
                seg.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            except Exception:
                pass
            self._bridge_segments.append(seg)
        self._right_pad = QFrame(wrapper)
        self._right_pad.setObjectName("FloatingBarPad")
        try:
            # Allow clicks inside the pad (e.g., "변경" button)
            self._right_pad.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        except Exception:
            pass
        self._right_pad.setFixedWidth(0)
        self._right_pad.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        try:
            self._frontend_light.setParent(self._right_pad)
            self._backend_light.setParent(self._right_pad)
            self._position_status_lights()
            self._frontend_light.hide()
            self._backend_light.hide()
        except Exception:
            pass
        pad_layout = QVBoxLayout(self._right_pad)
        pad_layout.setContentsMargins(46, 15, 15, 15)
        pad_layout.setSpacing(6)  # match pill button spacing
        pad_layout.setAlignment(Qt.AlignTop)
        (
            self.pad_box_open_ui,
            self.pad_label_open_ui,
            self.pad_stack_open_ui,
            self._open_ui_buttons,
        ) = self._create_pad_open_mode_box(
            "1. UI 열기",
            self._open_ui_mode,
            self._set_open_ui_mode,
        )
        (
            self.pad_box_sync,
            self.pad_label_sync,
            self.pad_stack_sync,
            self.pad_path_label,
            self.pad_path_change_btn,
        ) = self._create_pad_path_box("2. 코드 동기화")
        (
            self.pad_box_folder,
            self.pad_label_folder,
            self.pad_stack_folder,
            self.pad_import_buttons,
        ) = self._create_pad_choice_box("3. 불러오기", self._pad_import_choice, self._set_import_choice)
        (
        self.pad_box_logs,
        self.pad_label_logs,
        self.pad_stack_logs,
        self.pad_export_buttons,
        ) = self._create_pad_choice_box("4. 내보내기", self._pad_export_choice, self._set_export_choice)
        self.pad_box_settings, self.pad_label_settings, self.pad_stack_settings = self._create_pad_simple_box("5. 로그")
        (
            self.pad_box_exit,
            self.pad_label_exit,
            self.pad_stack_exit,
            self._exit_action_buttons,
        ) = self._create_pad_exit_action_box("6. 종료", self._exit_action, self._set_exit_action)
        self._pad_boxes = [
            self.pad_box_open_ui,
            self.pad_box_sync,
            self.pad_box_folder,
            self.pad_box_logs,
            self.pad_box_settings,
            self.pad_box_exit,
        ]
        self._pad_loading_panel = self._create_pad_loading_panel()
        self._setup_settings_status_mode()
        self._pad_desc_labels = [
            self.pad_label_open_ui,
            self.pad_label_sync,
            self.pad_label_folder,
            self.pad_label_logs,
            self.pad_label_settings,
            self.pad_label_exit,
        ]
        self._pad_desc_stacks = [
            self.pad_stack_open_ui,
            self.pad_stack_sync,
            self.pad_stack_folder,
            self.pad_stack_logs,
            self.pad_stack_settings,
            self.pad_stack_exit,
        ]
        self._update_pad_choice_labels()
        self._update_pad_choice_tooltips()
        self._update_exit_label()
        self._set_pad_description_indices(set(range(len(self._pad_desc_labels))))
        pad_layout.addWidget(self.pad_box_open_ui)
        pad_layout.addWidget(self.pad_box_sync)
        pad_layout.addWidget(self.pad_box_folder)
        pad_layout.addWidget(self.pad_box_logs)
        pad_layout.addWidget(self.pad_box_settings)
        pad_layout.addWidget(self.pad_box_exit)
        try:
            self._right_pad.lower()  # keep the pad behind the pill so pill corners stay round
        except Exception:
            pass
        wrapper_layout.addWidget(self.pill, 0, Qt.AlignLeft | Qt.AlignVCenter)
        wrapper_layout.addStretch(1)  # space for pad to overlap into
        # pad is positioned manually; keep it out of the layout to allow overlap
        self.setCentralWidget(wrapper)
        try:
            # z-order: pad (bottom) -> bridge -> pill (top)
            self._pad_bridge.raise_()
            self.pill.raise_()
        except Exception:
            pass
        self._register_drag_targets(self, wrapper, self.pill, self._right_pad)

        # Use a normal window type (not Qt.Tool) so the icon shows in the taskbar even when floating
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint | Qt.Window)
        try:
            self.setAttribute(Qt.WA_TranslucentBackground, True)
            self.setAttribute(Qt.WA_Hover, True)
        except Exception:
            pass
        self.setStyleSheet("""
            QWidget#LauncherWindow { background: transparent; }
            QPushButton#CircleButton {
                background: #000;
                color: #fff;
                border: none;
                border-radius: 30px;
                font-weight: 700;
            }
            QPushButton#CircleButton:hover { border: none; }
            QPushButton#CircleButton:disabled { background: #3a3a3a; color: #9e9e9e; }
        """)
        self.adjustSize()
        try:
            bar_height = self.pill.sizeHint().height() or self.pill.height() or self.height()
        except Exception:
            bar_height = self.height()
        self._bar_height = max(1, bar_height)
        try:
            self.pill.setFixedHeight(self._bar_height)
            self._right_pad.setFixedHeight(self._bar_height)
            self.setMinimumHeight(self._bar_height)
            self.setMaximumHeight(self._bar_height)
            self.resize(self.width(), self._bar_height)
        except Exception:
            pass
        try:
            collapsed = self.pill.sizeHint().width() or self.pill.width() or self.width()
        except Exception:
            collapsed = self.width()
        self._pill_collapsed_width = max(1, collapsed)
        self.pill.setFixedWidth(self._pill_collapsed_width)
        total_width = self._pill_collapsed_width + self._pad_width
        try:
            self.setMinimumWidth(self._pill_collapsed_width)
            self.setMaximumWidth(total_width)
            self.resize(self._pill_collapsed_width, self.height())
        except Exception:
            pass
        self._apply_pill_width(0, keep_right=False)
        self._position_floating_bar(force=True)
        self._position_loading_panel()
        self._position_loading_ring()

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

        # Always request auth on startup, then ensure app data/project roots are writable.
        if not self._ensure_app_and_project_writable(force_auth=True):
            try:
                QMessageBox.critical(self, "권한 필요", "데이터/프로젝트 경로 권한을 획득하지 못해 종료합니다.")
            except Exception:
                pass
            try:
                QTimer.singleShot(0, lambda: QApplication.instance().quit())
            except Exception:
                sys.exit(1)
            return
        # Ensure project exists; if not, copy from system payload or repo
        self._ensure_project_present()
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

        self._status_timer = QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._update_status_lights)
        self._status_timer.start()
        self._update_status_lights()

        # Periodic status update
        self.timer = QTimer(self)
        self.timer.setInterval(3000)
        self.timer.timeout.connect(self.update_state_label)
        self.timer.start()

        self.hide()

    def _cursor_over_pill(self, strict: bool = False) -> bool:
        try:
            from PySide6.QtGui import QCursor
        except Exception:
            try:
                from PyQt6.QtGui import QCursor
            except Exception:
                return False
        try:
            global_pos = QCursor.pos()
            if not self._cursor_over_window(global_pos):
                return False
            if strict:
                bounds = self._content_hover_bounds()
                if not bounds:
                    return False
                x = int(global_pos.x())
                y = int(global_pos.y())
                left, top, right, bottom = bounds
                return left <= x < right and top <= y < bottom
            local = self.pill.mapFromGlobal(global_pos)
            if self.pill.rect().contains(local):
                return True
            pad = getattr(self, "_right_pad", None)
            if pad is not None and pad.isVisible() and pad.width() > 0:
                pad_local = pad.mapFromGlobal(global_pos)
                if pad.rect().contains(pad_local):
                    return True
            return False
        except Exception:
            return False

    def _cursor_over_window(self, global_pos) -> bool:
        try:
            local = self.mapFromGlobal(global_pos)
            if not self.rect().contains(local):
                return False
        except Exception:
            try:
                if not self.frameGeometry().contains(global_pos):
                    return False
            except Exception:
                pass
        widget = None
        try:
            widget = QApplication.widgetAt(global_pos)
        except Exception:
            try:
                app = QApplication.instance()
                if app is not None:
                    widget = app.widgetAt(global_pos)
            except Exception:
                widget = None
        if widget is None:
            return False
        try:
            if widget is self or self.isAncestorOf(widget):
                return True
        except Exception:
            pass
        return False

    def _content_hover_bounds(self) -> list[int] | None:
        widgets = [
            getattr(self, "btn_open_browser", None),
            getattr(self, "btn_quick_apply", None),
            getattr(self, "btn_folder", None),
            getattr(self, "btn_logs", None),
            getattr(self, "btn_settings", None),
            getattr(self, "btn_exit", None),
            getattr(self, "pad_box_open_ui", None),
            getattr(self, "pad_box_sync", None),
            getattr(self, "pad_box_folder", None),
            getattr(self, "pad_box_logs", None),
            getattr(self, "pad_box_settings", None),
            getattr(self, "pad_box_exit", None),
        ]
        bounds = None
        for w in widgets:
            if w is None:
                continue
            try:
                if not w.isVisible():
                    continue
            except Exception:
                pass
            try:
                pos = w.mapToGlobal(QPoint(0, 0))
                x1 = int(pos.x())
                y1 = int(pos.y())
                x2 = x1 + int(w.width())
                y2 = y1 + int(w.height())
            except Exception:
                continue
            if x2 <= x1 or y2 <= y1:
                continue
            if bounds is None:
                bounds = [x1, y1, x2, y2]
            else:
                if x1 < bounds[0]:
                    bounds[0] = x1
                if y1 < bounds[1]:
                    bounds[1] = y1
                if x2 > bounds[2]:
                    bounds[2] = x2
                if y2 > bounds[3]:
                    bounds[3] = y2
        return bounds

    def _hovered_pad_box_index(self) -> int | None:
        try:
            from PySide6.QtGui import QCursor
        except Exception:
            try:
                from PyQt6.QtGui import QCursor
            except Exception:
                return None
        try:
            global_pos = QCursor.pos()
            pad = getattr(self, "_right_pad", None)
            if pad is None or not pad.isVisible() or pad.width() <= 0:
                return None
            boxes = [
                getattr(self, "pad_box_open_ui", None),
                getattr(self, "pad_box_sync", None),
                getattr(self, "pad_box_folder", None),
                getattr(self, "pad_box_logs", None),
                getattr(self, "pad_box_settings", None),
                getattr(self, "pad_box_exit", None),
            ]
            for idx, box in enumerate(boxes):
                if box is None or not box.isVisible():
                    continue
                local = box.mapFromGlobal(global_pos)
                if box.rect().contains(local):
                    return idx
            return None
        except Exception:
            return None

    def _cursor_in_pad_box_column(self) -> bool:
        try:
            from PySide6.QtGui import QCursor
        except Exception:
            try:
                from PyQt6.QtGui import QCursor
            except Exception:
                return False
        try:
            global_pos = QCursor.pos()
            pad = getattr(self, "_right_pad", None)
            if pad is None or not pad.isVisible() or pad.width() <= 0:
                return False
            pad_pos = pad.mapToGlobal(QPoint(0, 0))
            pad_x1 = int(pad_pos.x())
            pad_y1 = int(pad_pos.y())
            pad_x2 = pad_x1 + int(pad.width())
            pad_y2 = pad_y1 + int(pad.height())
            gx = int(global_pos.x())
            gy = int(global_pos.y())
            if gx < pad_x1 or gx >= pad_x2 or gy < pad_y1 or gy >= pad_y2:
                return False
            boxes = [
                getattr(self, "pad_box_open_ui", None),
                getattr(self, "pad_box_sync", None),
                getattr(self, "pad_box_folder", None),
                getattr(self, "pad_box_logs", None),
                getattr(self, "pad_box_settings", None),
                getattr(self, "pad_box_exit", None),
            ]
            x_min = None
            x_max = None
            for box in boxes:
                if box is None:
                    continue
                try:
                    if not box.isVisible():
                        continue
                except Exception:
                    continue
                try:
                    pos = box.mapToGlobal(QPoint(0, 0))
                    bx1 = int(pos.x())
                    bx2 = bx1 + int(box.width())
                except Exception:
                    continue
                if x_min is None or bx1 < x_min:
                    x_min = bx1
                if x_max is None or bx2 > x_max:
                    x_max = bx2
            if x_min is None or x_max is None:
                return False
            return x_min <= gx < x_max
        except Exception:
            return False

    def _cursor_over_pad_area(self) -> bool:
        try:
            from PySide6.QtGui import QCursor
        except Exception:
            try:
                from PyQt6.QtGui import QCursor
            except Exception:
                return False
        try:
            global_pos = QCursor.pos()
            pad = getattr(self, "_right_pad", None)
            if pad is None or not pad.isVisible() or pad.width() <= 0:
                return False
            local = pad.mapFromGlobal(global_pos)
            return pad.rect().contains(local)
        except Exception:
            return False

    def _set_pad_description_indices(self, indices: set[int]):
        stacks = getattr(self, "_pad_desc_stacks", [])
        for idx, stack in enumerate(stacks):
            try:
                if stack is not None:
                    stack.setCurrentIndex(0 if idx in indices else 1)
            except Exception:
                pass

    def _start_pad_description_window(self):
        self._pad_desc_window_active = True
        try:
            self._pad_desc_timer.stop()
        except Exception:
            pass
        self._pad_desc_timer.start(500)
        self._update_pad_description_mode()

    def _on_pad_desc_timeout(self):
        self._pad_desc_window_active = False
        self._update_pad_description_mode()

    def _update_pad_description_mode(self):
        labels = getattr(self, "_pad_desc_labels", [])
        all_indices = set(range(len(labels)))
        if getattr(self, "_pad_desc_window_active", False):
            self._set_pad_description_indices(all_indices)
            return
        idx = self._hovered_pad_box_index()
        if idx is not None:
            self._set_pad_description_indices(all_indices - {idx})
            return
        if self._cursor_in_pad_box_column():
            if self._pad_force_mode_index is not None:
                self._pad_force_mode_index = None
            self._set_pad_description_indices(all_indices)
            return
        forced_idx = getattr(self, "_pad_force_mode_index", None)
        if forced_idx is not None and forced_idx in all_indices:
            self._set_pad_description_indices(all_indices - {forced_idx})
            return
        self._set_pad_description_indices(all_indices)

    def _pill_button_entry(self, obj) -> tuple[int, QPushButton] | None:
        buttons = [
            getattr(self, "btn_open_browser", None),
            getattr(self, "btn_quick_apply", None),
            getattr(self, "btn_folder", None),
            getattr(self, "btn_logs", None),
            getattr(self, "btn_settings", None),
            getattr(self, "btn_exit", None),
        ]
        for idx, btn in enumerate(buttons):
            if btn is None:
                continue
            try:
                if obj is btn or btn.isAncestorOf(obj):
                    return idx, btn
            except Exception:
                continue
        return None

    def _start_pill_hover_timer(self, idx: int, target):
        try:
            if self._pill_hover_target is target:
                return
        except Exception:
            pass
        try:
            self._pill_hover_timer.stop()
        except Exception:
            pass
        self._pill_hover_target = target
        self._pill_hover_target_index = idx
        self._on_pill_hover_timeout()

    def _clear_pill_hover(self, target=None):
        try:
            if target is not None and self._pill_hover_target is not target:
                return
        except Exception:
            pass
        try:
            self._pill_hover_timer.stop()
        except Exception:
            pass
        keep_forced = False
        try:
            keep_forced = self._cursor_over_pill(strict=False)
        except Exception:
            keep_forced = False
        self._pill_hover_target = None
        self._pill_hover_target_index = None
        if keep_forced:
            return
        if self._pad_force_mode_index is not None:
            self._pad_force_mode_index = None
            self._update_pad_description_mode()

    def _on_pill_hover_timeout(self):
        idx = getattr(self, "_pill_hover_target_index", None)
        target = getattr(self, "_pill_hover_target", None)
        if idx is None or target is None:
            return
        try:
            from PySide6.QtGui import QCursor
        except Exception:
            try:
                from PyQt6.QtGui import QCursor
            except Exception:
                QCursor = None  # type: ignore
        if QCursor is not None:
            try:
                gp = QCursor.pos()
                local = target.mapFromGlobal(gp)
                if not target.rect().contains(local):
                    return
            except Exception:
                pass
        self._pad_force_mode_index = idx
        self._update_pad_description_mode()

    def _set_import_choice(self, choice: str):
        if choice not in ("DB", "DATASET", "MODEL"):
            return
        self._pad_import_choice = choice
        self._update_pad_choice_labels()
        self._update_pad_choice_tooltips()
        self._style_choice_buttons(getattr(self, "pad_import_buttons", {}))

    def _set_export_choice(self, choice: str):
        if choice not in ("DB", "DATASET", "MODEL"):
            return
        self._pad_export_choice = choice
        self._update_pad_choice_labels()
        self._update_pad_choice_tooltips()
        self._style_choice_buttons(getattr(self, "pad_export_buttons", {}))

    def _set_exit_action(self, action: str):
        if action not in ("EXIT", "RESTART"):
            return
        self._exit_action = action
        self._update_exit_label()
        self._style_choice_buttons(getattr(self, "_exit_action_buttons", {}))

    def _set_open_ui_mode(self, mode: str):
        if mode not in ("NEW", "CURRENT"):
            return
        self._open_ui_mode = mode
        self._update_open_ui_tooltip()
        self._style_choice_buttons(getattr(self, "_open_ui_buttons", {}))

    def _update_pad_choice_labels(self):
        try:
            self.pad_label_folder.setText(f"3. {self._pad_import_choice} 불러오기")
        except Exception:
            pass
        try:
            self.pad_label_logs.setText(f"4. {self._pad_export_choice} 내보내기")
        except Exception:
            pass

    def _update_pad_choice_tooltips(self):
        if getattr(self, "_disable_circle_tooltips", False):
            return
        try:
            self.btn_folder.setToolTip(f"{self._pad_import_choice} 불러오기")
        except Exception:
            pass
        try:
            self.btn_logs.setToolTip(f"{self._pad_export_choice} 내보내기")
        except Exception:
            pass

    def _open_ui_mode_tooltip(self, mode: str) -> str:
        if mode == "NEW":
            return "새 창으로 UI 열기"
        return "새 탭으로 UI 열기"

    def _open_ui_mode_label(self, mode: str) -> str:
        if mode == "NEW":
            return "1. 새 창으로 UI 열기"
        return "1. 새 탭으로 UI 열기"

    def _exit_action_label(self, action: str) -> str:
        if action == "RESTART":
            return "6. 재실행"
        return "6. 종료"

    def _update_open_ui_tooltip(self):
        mode = getattr(self, "_open_ui_mode", "NEW")
        try:
            self.pad_label_open_ui.setText(self._open_ui_mode_label(mode))
        except Exception:
            pass
        if getattr(self, "_disable_circle_tooltips", False):
            return
        try:
            self.btn_open_browser.setToolTip(self._open_ui_mode_tooltip(mode))
        except Exception:
            pass

    def _update_exit_label(self):
        action = getattr(self, "_exit_action", "EXIT")
        try:
            self.pad_label_exit.setText(self._exit_action_label(action))
        except Exception:
            pass

    def _current_choice_from_buttons(self, buttons: dict[str, QPushButton], fallback: str) -> str:
        if not buttons:
            return fallback
        for key, btn in buttons.items():
            try:
                if btn.isChecked():
                    return key
            except Exception:
                continue
        return fallback

    def _on_import_clicked(self):
        choice = self._current_choice_from_buttons(
            getattr(self, "pad_import_buttons", {}),
            self._pad_import_choice,
        )
        self._run_import_selected(choice)

    def _on_export_clicked(self):
        choice = self._current_choice_from_buttons(
            getattr(self, "pad_export_buttons", {}),
            self._pad_export_choice,
        )
        self._run_export_selected(choice)

    def _map_backend_db_path(self, raw_path: str) -> Path | None:
        try:
            path = Path(raw_path)
        except Exception:
            return None
        if not path.is_absolute():
            return None
        if path == Path("/root/src") or str(path).startswith("/root/src/"):
            try:
                rel = path.relative_to("/root/src")
            except Exception:
                return None
            for base in (getattr(self, "project_root", None), getattr(self, "dev_src_root", None)):
                if isinstance(base, Path):
                    return base / "src" / rel
        return path

    def _fetch_backend_db_path(self) -> Path | None:
        try:
            from urllib import request
            req = request.Request("http://127.0.0.1:5000/api/db/path", method="GET")
            with request.urlopen(req, timeout=0.8) as resp:
                payload = json.loads(resp.read().decode("utf-8") or "{}")
            raw_path = payload.get("path")
            if not raw_path:
                return None
            return self._map_backend_db_path(str(raw_path))
        except Exception:
            return None

    def _db_main_path(self) -> Path:
        data_db = DATA_ROOT / "database" / "main.db"
        backend_path = self._fetch_backend_db_path()
        if backend_path is not None:
            return backend_path
        running = False
        try:
            running = "service" in self._get_running_services()
        except Exception:
            running = False
        if running:
            return data_db
        project_under_data = False
        try:
            if isinstance(self.project_root, Path):
                pr = self.project_root.resolve()
                dr = DATA_ROOT.resolve()
                project_under_data = pr == dr or dr in pr.parents
        except Exception:
            project_under_data = False
        if self._is_valid_dev_src(self.dev_src_root) and not project_under_data:
            return self.dev_src_root / "src" / "backend" / "database" / "main.db"
        if self._is_valid_project_root(self.project_root) and not project_under_data:
            return self.project_root / "src" / "backend" / "database" / "main.db"
        if docker_compose_available() and self._is_valid_project_root(self.project_root):
            return data_db
        if self._is_valid_project_root(self.project_root):
            return self.project_root / "src" / "backend" / "database" / "main.db"
        return data_db

    def _cleanup_sqlite_sidecars(self, db_path: Path):
        for suffix in ("-wal", "-shm"):
            sidecar = db_path.with_name(db_path.name + suffix)
            try:
                if sidecar.exists():
                    sidecar.unlink()
            except Exception:
                pass

    def _copy_file_best_effort(self, src: Path, dest: Path, context: str):
        try:
            shutil.copy2(src, dest)
            return
        except OSError as e:
            try:
                shutil.copyfile(src, dest)
            except Exception:
                raise
            self.append_log(f"[{context}][INFO] 메타데이터 복사 실패로 기본 복사로 진행: {e}")

    def _unique_import_path(self, dest: Path) -> Path:
        if not dest.exists():
            return dest
        parent = dest.parent
        suffix = dest.suffix
        stem = dest.stem if suffix else dest.name
        for i in range(1, 1000):
            candidate = parent / f"{stem}_{i}{suffix}"
            if not candidate.exists():
                return candidate
        return parent / f"{stem}_{int(time.time())}{suffix}"

    def _reload_backend_db(self) -> bool:
        try:
            from urllib import request
            req = request.Request("http://127.0.0.1:5000/api/db/reload", method="POST")
            with request.urlopen(req, timeout=2.0) as resp:
                payload = json.loads(resp.read().decode("utf-8") or "{}")
            return payload.get("status") == "success"
        except Exception as e:
            self.append_log(f"[IMPORT][WARN] DB 갱신 요청 실패: {e}")
            return False

    def _resolve_backend_subdir(self, name: str) -> Path | None:
        roots: list[Path] = []
        root = getattr(self, "project_root", None)
        dev_root = getattr(self, "dev_src_root", None)
        prefer_dev = False
        project_under_data = False
        try:
            if isinstance(root, Path):
                pr = root.resolve()
                dr = DATA_ROOT.resolve()
                project_under_data = pr == dr or dr in pr.parents
        except Exception:
            project_under_data = False
        try:
            prefer_dev = ("service" not in self._get_running_services()) and not project_under_data
        except Exception:
            prefer_dev = False
        ordered_roots = [dev_root, root] if prefer_dev else [root, dev_root]
        for candidate in ordered_roots:
            if isinstance(candidate, Path) and candidate not in roots:
                roots.append(candidate)
        for base in roots:
            try:
                if base.exists():
                    return base / "src" / "backend" / name
            except Exception:
                continue
        return None

    def _db_table_columns(self, conn, table: str) -> set[str]:
        try:
            cur = conn.execute(f"PRAGMA table_info({table})")
            return {row[1] for row in cur.fetchall()}
        except Exception:
            return set()

    def _fetch_tasks_from_db(self) -> list[tuple[int, str]]:
        db_path = self._db_main_path()
        if not db_path.exists():
            return []
        conn = None
        try:
            conn = sqlite3.connect(str(db_path))
            cur = conn.execute("SELECT id, name FROM tasks WHERE deleted_at IS NULL ORDER BY id")
            tasks = []
            for row in cur.fetchall():
                try:
                    tid = int(row[0])
                except Exception:
                    continue
                name = row[1] or f"Task {tid}"
                tasks.append((tid, name))
            return tasks
        except Exception:
            return []
        finally:
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass

    def _select_task_id(self) -> int | None:
        tasks = self._fetch_tasks_from_db()
        if not tasks:
            return None
        if len(tasks) == 1:
            return tasks[0][0]
        items = [f"{tid}: {name}" for tid, name in tasks]
        choice, ok = QInputDialog.getItem(
            self,
            "작업 선택",
            "연결할 작업을 선택하세요.",
            items,
            0,
            False,
        )
        if not ok or not choice:
            return None
        try:
            return int(str(choice).split(":", 1)[0].strip())
        except Exception:
            return None

    def _fetch_policies_from_db(self) -> list[tuple[int, str]]:
        db_path = self._db_main_path()
        if not db_path.exists():
            return []
        conn = None
        try:
            conn = sqlite3.connect(str(db_path))
            cur = conn.execute("SELECT id, name, type FROM policies WHERE deleted_at IS NULL ORDER BY id")
            policies = []
            for row in cur.fetchall():
                try:
                    pid = int(row[0])
                except Exception:
                    continue
                name = row[1] or f"Policy {pid}"
                ptype = row[2] or ""
                label = f"{name} ({ptype})" if ptype else name
                policies.append((pid, label))
            return policies
        except Exception:
            return []
        finally:
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass

    def _select_policy_id(self) -> int | None:
        policies = self._fetch_policies_from_db()
        if not policies:
            return None
        if len(policies) == 1:
            return policies[0][0]
        items = [f"{pid}: {label}" for pid, label in policies]
        choice, ok = QInputDialog.getItem(
            self,
            "정책 선택",
            "연결할 정책을 선택하세요.",
            items,
            0,
            False,
        )
        if not ok or not choice:
            return None
        try:
            return int(str(choice).split(":", 1)[0].strip())
        except Exception:
            return None

    def _insert_dataset_record(self, name: str, task_id: int | None) -> int | None:
        db_path = self._db_main_path()
        if not db_path.exists():
            return None
        conn = None
        try:
            conn = sqlite3.connect(str(db_path))
            columns = self._db_table_columns(conn, "datasets")
            if not columns:
                return None
            fields = []
            values = []
            if "name" in columns:
                fields.append("name")
                values.append(name)
            if "task_id" in columns:
                fields.append("task_id")
                values.append(task_id)
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            if "created_at" in columns:
                fields.append("created_at")
                values.append(timestamp)
            if "updated_at" in columns:
                fields.append("updated_at")
                values.append(timestamp)
            if "deleted_at" in columns:
                fields.append("deleted_at")
                values.append(None)
            if not fields:
                return None
            placeholders = ", ".join(["?"] * len(fields))
            sql = f"INSERT INTO datasets ({', '.join(fields)}) VALUES ({placeholders})"
            cur = conn.execute(sql, values)
            conn.commit()
            return cur.lastrowid
        except Exception:
            return None
        finally:
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass

    def _insert_checkpoint_record(
        self,
        name: str,
        policy_id: int | None,
        task_id: int | None = None,
        is_base_model: bool = True,
    ) -> int | None:
        if policy_id is None:
            return None
        db_path = self._db_main_path()
        if not db_path.exists():
            return None
        conn = None
        try:
            conn = sqlite3.connect(str(db_path))
            columns = self._db_table_columns(conn, "checkpoints")
            if not columns:
                return None
            fields = []
            values = []
            if "name" in columns:
                fields.append("name")
                values.append(name)
            if "policy_id" in columns:
                fields.append("policy_id")
                values.append(policy_id)
            if "task_id" in columns:
                fields.append("task_id")
                values.append(task_id)
            if "dataset_info" in columns:
                fields.append("dataset_info")
                values.append(json.dumps({}))
            if "status" in columns:
                fields.append("status")
                values.append("finished")
            if "is_base_model" in columns:
                fields.append("is_base_model")
                values.append(1 if is_base_model else 0)
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            if "created_at" in columns:
                fields.append("created_at")
                values.append(timestamp)
            if "updated_at" in columns:
                fields.append("updated_at")
                values.append(timestamp)
            if "deleted_at" in columns:
                fields.append("deleted_at")
                values.append(None)
            if not fields:
                return None
            placeholders = ", ".join(["?"] * len(fields))
            sql = f"INSERT INTO checkpoints ({', '.join(fields)}) VALUES ({placeholders})"
            cur = conn.execute(sql, values)
            conn.commit()
            return cur.lastrowid
        except Exception:
            return None
        finally:
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass

    def _run_import_selected(self, choice: str):
        try:
            if choice == "DB":
                result = QFileDialog.getOpenFileName(
                    self,
                    "DB 불러오기",
                    str(Path.home()),
                    "DB Files (*.db *.sqlite *.sqlite3);;All Files (*)",
                )
                src_path = result[0] if isinstance(result, (tuple, list)) else result
            else:
                src_path = QFileDialog.getExistingDirectory(
                    self,
                    f"{choice} 불러오기",
                    str(Path.home()),
                )
        except Exception:
            return
        if not src_path:
            return
        src = Path(src_path)
        try:
            if choice == "DB":
                dest = self._db_main_path()
                dest.parent.mkdir(parents=True, exist_ok=True)
                if src.is_dir():
                    raise ValueError("DB 파일이 디렉터리입니다.")
                self._cleanup_sqlite_sidecars(dest)
                self._copy_file_best_effort(src, dest, "IMPORT")
                self._cleanup_sqlite_sidecars(dest)
                self.append_log(f"[IMPORT] {choice} -> {dest}")
                if docker_compose_available() and self._is_valid_project_root(self.project_root):
                    running = False
                    try:
                        running = "service" in self._get_running_services()
                    except Exception:
                        running = False
                    if running:
                        if self._reload_backend_db():
                            self.append_log("[IMPORT] DB 연결을 갱신했습니다.")
                        else:
                            self.append_log("[IMPORT][WARN] DB 갱신에 실패했습니다. UI만 새로고침합니다.")
                    else:
                        self.append_log("[IMPORT][INFO] 서비스가 실행 중이 아니어서 UI만 새로고침합니다.")
                    self.load_ui(open_mode="CURRENT", force_refresh=True)
                else:
                    if self._reload_backend_db():
                        self.append_log("[IMPORT] DB 연결을 갱신했습니다.")
                    self.load_ui(open_mode="CURRENT", force_refresh=True)
                return
            elif choice == "DATASET":
                dest_root = self._resolve_backend_subdir("datasets")
                if dest_root is None:
                    self.append_log("[IMPORT][WARN] DATASET 경로를 찾을 수 없습니다.")
                    return
                dest_root.mkdir(parents=True, exist_ok=True)
                tasks = self._fetch_tasks_from_db()
                if not tasks:
                    self.append_log("[IMPORT][WARN] 작업이 없어 DATASET을 적용할 수 없습니다. 작업을 먼저 생성하세요.")
                    return
                task_id = tasks[0][0] if len(tasks) == 1 else self._select_task_id()
                if task_id is None:
                    self.append_log("[IMPORT][INFO] DATASET 불러오기를 취소했습니다.")
                    return
                dataset_name = src.stem if src.is_file() else src.name
                dataset_id = self._insert_dataset_record(dataset_name, task_id)
                if dataset_id is None:
                    self.append_log("[IMPORT][WARN] DB 등록 실패로 파일만 복사합니다.")
                    dest = self._unique_import_path(dest_root / src.name)
                    if src.is_dir():
                        shutil.copytree(src, dest, dirs_exist_ok=True)
                    else:
                        shutil.copy2(src, dest)
                else:
                    dest = dest_root / str(dataset_id)
                    if src.is_dir():
                        shutil.copytree(src, dest, dirs_exist_ok=True)
                    else:
                        dest.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(src, dest / src.name)
            elif choice == "MODEL":
                dest_root = self._resolve_backend_subdir("checkpoints") or self._resolve_backend_subdir("models")
                if dest_root is None:
                    self.append_log("[IMPORT][WARN] MODEL 경로를 찾을 수 없습니다.")
                    return
                dest_root.mkdir(parents=True, exist_ok=True)
                policies = self._fetch_policies_from_db()
                if not policies:
                    self.append_log("[IMPORT][WARN] 정책이 없어 MODEL을 적용할 수 없습니다. 정책을 먼저 생성하세요.")
                    return
                policy_id = policies[0][0] if len(policies) == 1 else self._select_policy_id()
                if policy_id is None:
                    self.append_log("[IMPORT][INFO] MODEL 불러오기를 취소했습니다.")
                    return
                tasks = self._fetch_tasks_from_db()
                task_id = None
                if tasks:
                    task_id = tasks[0][0] if len(tasks) == 1 else self._select_task_id()
                is_base_model = task_id is None
                model_name = src.stem if src.is_file() else src.name
                checkpoint_id = self._insert_checkpoint_record(
                    model_name,
                    policy_id,
                    task_id=task_id,
                    is_base_model=is_base_model,
                )
                if checkpoint_id is None:
                    self.append_log("[IMPORT][WARN] DB 등록 실패로 파일만 복사합니다.")
                    dest = self._unique_import_path(dest_root / src.name)
                    if src.is_dir():
                        shutil.copytree(src, dest, dirs_exist_ok=True)
                    else:
                        shutil.copy2(src, dest)
                else:
                    dest = dest_root / str(checkpoint_id)
                    if src.is_dir():
                        shutil.copytree(src, dest, dirs_exist_ok=True)
                    else:
                        dest.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(src, dest / src.name)
            else:
                return
            self.append_log(f"[IMPORT] {choice} -> {dest}")
            self.load_ui(open_mode="CURRENT", force_refresh=True)
        except Exception as e:
            self.append_log(f"[IMPORT][ERROR] {choice} 불러오기 실패: {e}")

    def _resolve_export_source(self, choice: str) -> Path | None:
        if choice == "DB":
            db_path = self._db_main_path()
            try:
                return db_path if db_path.exists() else None
            except Exception:
                return None
        if choice == "DATASET":
            root = self._resolve_backend_subdir("datasets")
            try:
                return root if root and root.exists() else None
            except Exception:
                return None
        if choice == "MODEL":
            for name in ("checkpoints", "models"):
                root = self._resolve_backend_subdir(name)
                try:
                    if root and root.exists():
                        return root
                except Exception:
                    continue
        return None

    def _run_export_selected(self, choice: str):
        src = self._resolve_export_source(choice)
        if not src:
            self.append_log(f"[EXPORT][WARN] {choice} 내보낼 항목을 찾지 못했습니다.")
            return
        try:
            if choice == "DB":
                default_name = src.name if src.name else "main.db"
                result = QFileDialog.getSaveFileName(
                    self,
                    "DB 저장 위치 선택",
                    str(Path.home() / default_name),
                    "DB Files (*.db *.sqlite *.sqlite3);;All Files (*)",
                )
                dest_path = result[0] if isinstance(result, (tuple, list)) else result
                if not dest_path:
                    return
                dest = Path(dest_path)
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dest)
            else:
                dest_dir = QFileDialog.getExistingDirectory(
                    self,
                    f"{choice} 저장 위치 선택",
                    str(Path.home()),
                )
                if not dest_dir:
                    return
                dest = Path(dest_dir) / src.name
                if src.is_dir():
                    shutil.copytree(src, dest, dirs_exist_ok=True)
                else:
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(src, dest)
            self.append_log(f"[EXPORT] {choice} -> {dest}")
        except Exception as e:
            self.append_log(f"[EXPORT][ERROR] {choice} 내보내기 실패: {e}")

    def _show_pad_notice(self, index: int, text: str, duration_ms: int = 1000, color: str | None = None):
        labels = getattr(self, "_pad_desc_labels", [])
        if index < 0 or index >= len(labels):
            return
        label = labels[index]
        if label is None:
            return
        if index not in self._pad_notice_overrides:
            try:
                self._pad_notice_overrides[index] = (label.text(), label.styleSheet())
            except Exception:
                self._pad_notice_overrides[index] = ("", "")
        try:
            label.setText(text)
        except Exception:
            return
        if color:
            try:
                base_style = label.styleSheet() or ""
                label.setStyleSheet(f"{base_style} color: {color};")
            except Exception:
                pass
        if not self._pill_expanded:
            self._animate_pill(True)
        else:
            self._start_pad_description_window()
        try:
            self._pad_notice_timer.stop()
        except Exception:
            pass
        self._pad_notice_timer.start(duration_ms)

    def _clear_pad_notice(self):
        labels = getattr(self, "_pad_desc_labels", [])
        for idx, original in list(self._pad_notice_overrides.items()):
            if idx < 0 or idx >= len(labels):
                continue
            lbl = labels[idx]
            if lbl is None:
                continue
            try:
                original_text, original_style = original
                lbl.setText(original_text)
                if original_style:
                    lbl.setStyleSheet(original_style)
            except Exception:
                pass
        self._pad_notice_overrides.clear()

    def _prompt_dev_src_then_apply(self):
        if self._is_valid_dev_src(self.dev_src_root):
            return
        self.on_select_dev_src()
        if self._is_valid_dev_src(self.dev_src_root):
            self._quick_apply_from_dev_src()

    def _collapse_if_needed(self):
        if getattr(self, "_pill_loading", False):
            return
        if self._cursor_over_pill(strict=True):
            return
        self._animate_pill(False)

    def _sync_hover_state(self):
        if self._dragging or not self.isVisible():
            return
        if getattr(self, "_pill_loading", False):
            return
        hovered = self._cursor_over_pill(strict=True)
        if hovered and not self._pill_expanded:
            self._animate_pill(True)
        elif not hovered and self._pill_expanded:
            self._animate_pill(False)

    def _place_pad(self, width: int | None = None):
        pad = getattr(self, "_right_pad", None)
        if pad is None:
            return
        if width is None:
            width = pad.width()
        try:
            pill_geom = self.pill.geometry()
            w = max(0, int(width))
            h = pill_geom.height()
            # Anchor the pad's left edge to the pill center and grow only rightwards
            center_x = pill_geom.x() + pill_geom.width() // 2
            x = center_x
            y = pill_geom.y()
            pad.setGeometry(x, y, w, h)
            pad.setVisible(w > 0)
            bridge = getattr(self, "_pad_bridge", None)
            if bridge is not None:
                bridge.setVisible(False)
                for seg in getattr(self, "_bridge_segments", []):
                    seg.setVisible(False)
            self._position_loading_panel()
            self._position_status_lights()
        except Exception:
            pass

    def set_display_mode(self, fullscreen: bool):
        self._fullscreen_pref = fullscreen

    def _circle_icon_paths(self) -> list[Path]:
        img_dir = Path(__file__).resolve().parent / "img"
        icons = [img_dir / f"Plugin icon - {idx}.png" for idx in range(1, 7)]
        if all(p.is_file() for p in icons):
            return icons
        return []

    def _create_circle_button(self, text: str, tooltip: str, icon_path: Path | str | None = None) -> QPushButton:
        btn = QPushButton(text, self.pill)
        btn.setObjectName("CircleButton")
        if getattr(self, "_disable_circle_tooltips", False):
            btn.setToolTip("")
        else:
            btn.setToolTip(tooltip)
        btn.setFixedSize(self._button_size, self._button_size)
        if icon_path:
            try:
                pm = QPixmap(str(icon_path))
                if not pm.isNull():
                    base_icon_size = int(self._button_size * 0.5)
                    icon_size = max(18, int(base_icon_size * (2 / 3)))
                    pm = pm.scaled(icon_size, icon_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    btn.setIcon(QIcon(pm))
                    btn.setIconSize(pm.size())
                    btn.setText("")
            except Exception:
                pass
        try:
            btn.setCursor(Qt.PointingHandCursor)
        except Exception:
            pass
        return btn

    def _create_pad_simple_box(self, text: str, mode2_text: str | None = None) -> tuple[QFrame, QLabel, QStackedLayout]:
        box = QFrame(self._right_pad)
        box.setObjectName("PadInfoBox")
        box.setMinimumHeight(self._pad_item_height)
        box.setMaximumHeight(self._pad_item_height)
        box.setStyleSheet("background-color: #2d2d2d; border: none; border-radius: 12px;")
        stack = QStackedLayout(box)
        stack.setContentsMargins(0, 0, 0, 0)
        stack.setSpacing(0)

        desc_page = QWidget(box)
        desc_layout = QHBoxLayout(desc_page)
        desc_layout.setContentsMargins(12, 0, 12, 0)
        desc_layout.setSpacing(0)
        desc_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        label = QLabel(text, desc_page)
        label.setObjectName("PadInfoLabel")
        label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        label.setStyleSheet("color: #f5f5f5; font-weight: 800; font-size: 16px; background-color: transparent; border: none;")
        desc_layout.addWidget(label, 1, Qt.AlignLeft | Qt.AlignVCenter)
        stack.addWidget(desc_page)

        mode2_page = QWidget(box)
        mode2_layout = QHBoxLayout(mode2_page)
        mode2_layout.setContentsMargins(12, 0, 12, 0)
        mode2_layout.setSpacing(0)
        mode2_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        if mode2_text is not None:
            mode2_label = QLabel(mode2_text, mode2_page)
            mode2_label.setObjectName("PadMode2Label")
            mode2_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            mode2_layout.addWidget(mode2_label, 1, Qt.AlignLeft | Qt.AlignVCenter)
        stack.addWidget(mode2_page)
        stack.setCurrentIndex(0)
        return box, label, stack

    def _create_pad_open_mode_box(
        self,
        text: str,
        default_choice: str,
        on_change,
    ) -> tuple[QFrame, QLabel, QStackedLayout, dict[str, QPushButton]]:
        box = QFrame(self._right_pad)
        box.setObjectName("PadInfoBox")
        box.setMinimumHeight(self._pad_item_height)
        box.setMaximumHeight(self._pad_item_height)
        box.setStyleSheet("background-color: #2d2d2d; border: none; border-radius: 12px;")
        stack = QStackedLayout(box)
        stack.setContentsMargins(0, 0, 0, 0)
        stack.setSpacing(0)

        desc_page = QWidget(box)
        desc_layout = QHBoxLayout(desc_page)
        desc_layout.setContentsMargins(12, 0, 12, 0)
        desc_layout.setSpacing(0)
        desc_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        desc_label = QLabel(text, desc_page)
        desc_label.setObjectName("PadInfoLabel")
        desc_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        desc_label.setStyleSheet("color: #f5f5f5; font-weight: 800; font-size: 16px; background-color: transparent; border: none;")
        desc_layout.addWidget(desc_label, 1, Qt.AlignLeft | Qt.AlignVCenter)
        stack.addWidget(desc_page)

        mode2_page = QWidget(box)
        mode2_layout = QHBoxLayout(mode2_page)
        mode2_layout.setContentsMargins(5, 5, 5, 5)
        mode2_layout.setSpacing(5)
        mode2_layout.setAlignment(Qt.AlignVCenter)
        group = QButtonGroup(mode2_page)
        group.setExclusive(True)
        buttons: dict[str, QPushButton] = {}
        choices = [
            ("NEW", "새 브라우저", "새 창으로 UI 열기"),
            ("CURRENT", "현재 브라우저", "새 탭으로 UI 열기"),
        ]
        for key, label, tooltip in choices:
            btn = QPushButton(label, mode2_page)
            btn.setObjectName("PadChoiceButton")
            btn.setCheckable(True)
            btn.setChecked(key == default_choice)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            try:
                btn.setAutoFillBackground(True)
            except Exception:
                pass
            btn.setToolTip(tooltip)
            btn.clicked.connect(lambda _=False, c=key: on_change(c))
            btn._choice_indicator_hide_text = True
            group.addButton(btn)
            buttons[key] = btn
            mode2_layout.addWidget(btn, 1)
        self._style_choice_buttons(buttons)
        stack.addWidget(mode2_page)
        stack.setCurrentIndex(0)
        return box, desc_label, stack, buttons

    def _create_pad_exit_action_box(
        self,
        text: str,
        default_choice: str,
        on_change,
    ) -> tuple[QFrame, QLabel, QStackedLayout, dict[str, QPushButton]]:
        box = QFrame(self._right_pad)
        box.setObjectName("PadInfoBox")
        box.setMinimumHeight(self._pad_item_height)
        box.setMaximumHeight(self._pad_item_height)
        box.setStyleSheet("background-color: #2d2d2d; border: none; border-radius: 12px;")
        stack = QStackedLayout(box)
        stack.setContentsMargins(0, 0, 0, 0)
        stack.setSpacing(0)

        desc_page = QWidget(box)
        desc_layout = QHBoxLayout(desc_page)
        desc_layout.setContentsMargins(12, 0, 12, 0)
        desc_layout.setSpacing(0)
        desc_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        desc_label = QLabel(text, desc_page)
        desc_label.setObjectName("PadInfoLabel")
        desc_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        desc_label.setStyleSheet("color: #f5f5f5; font-weight: 800; font-size: 16px; background-color: transparent; border: none;")
        desc_layout.addWidget(desc_label, 1, Qt.AlignLeft | Qt.AlignVCenter)
        stack.addWidget(desc_page)

        mode2_page = QWidget(box)
        mode2_layout = QHBoxLayout(mode2_page)
        mode2_layout.setContentsMargins(5, 5, 5, 5)
        mode2_layout.setSpacing(5)
        mode2_layout.setAlignment(Qt.AlignVCenter)
        group = QButtonGroup(mode2_page)
        group.setExclusive(True)
        buttons: dict[str, QPushButton] = {}
        choices = [
            ("EXIT", "종료"),
            ("RESTART", "재실행"),
        ]
        for key, label in choices:
            btn = QPushButton(label, mode2_page)
            btn.setObjectName("PadChoiceButton")
            btn.setCheckable(True)
            btn.setChecked(key == default_choice)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            try:
                btn.setAutoFillBackground(True)
            except Exception:
                pass
            btn.clicked.connect(lambda _=False, c=key: on_change(c))
            btn._choice_indicator_hide_text = True
            group.addButton(btn)
            buttons[key] = btn
            mode2_layout.addWidget(btn, 1)
        stack.addWidget(mode2_page)
        self._style_choice_buttons(buttons)
        stack.setCurrentIndex(0)
        return box, desc_label, stack, buttons

    def _create_pad_path_box(self, text: str) -> tuple[QFrame, QLabel, QStackedLayout, QLabel, QPushButton]:
        box = QFrame(self._right_pad)
        box.setObjectName("PadInfoBox")
        box.setMinimumHeight(self._pad_item_height)
        box.setMaximumHeight(self._pad_item_height)
        box.setStyleSheet("background-color: #2d2d2d; border: none; border-radius: 12px;")
        stack = QStackedLayout(box)
        stack.setContentsMargins(0, 0, 0, 0)
        stack.setSpacing(0)

        desc_page = QWidget(box)
        desc_layout = QHBoxLayout(desc_page)
        desc_layout.setContentsMargins(12, 0, 12, 0)
        desc_layout.setSpacing(0)
        desc_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        desc_label = QLabel(text, desc_page)
        desc_label.setObjectName("PadInfoLabel")
        desc_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        desc_label.setStyleSheet("color: #f5f5f5; font-weight: 800; font-size: 16px; background-color: transparent; border: none;")
        desc_layout.addWidget(desc_label, 1, Qt.AlignLeft | Qt.AlignVCenter)
        stack.addWidget(desc_page)

        mode2_page = QWidget(box)
        mode2_layout = QHBoxLayout(mode2_page)
        mode2_layout.setContentsMargins(12, 0, 0, 0)
        mode2_layout.setSpacing(6)
        mode2_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        path_label = QLabel("", mode2_page)
        path_label.setObjectName("PadPathLabel")
        path_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        path_label.setWordWrap(False)
        path_label.setStyleSheet("color: #cfd8dc; font-size: 12px; font-weight: 700; background-color: transparent; border: none;")
        try:
            flag = getattr(Qt, "TextSelectableByMouse", None)
            if flag is None:
                interaction_flags = getattr(Qt, "TextInteractionFlag", None)
                if interaction_flags is not None:
                    flag = getattr(interaction_flags, "TextSelectableByMouse", None)
            if flag is not None:
                path_label.setTextInteractionFlags(flag)
        except Exception:
            pass
        change_btn = QPushButton("변경", mode2_page)
        change_btn.setObjectName("PadPathChangeButton")
        btn_size = max(10, self._pad_item_height - 10)
        change_btn.setFixedSize(btn_size, btn_size)
        try:
            change_btn.setCursor(Qt.PointingHandCursor)
        except Exception:
            pass
        try:
            change_btn.setAutoFillBackground(True)
        except Exception:
            pass
        change_btn.setStyleSheet(
            "background-color: rgb(30,30,30); color: #eaeaea; border: none; border-radius: 6px;"
        )
        change_btn.clicked.connect(self.on_select_dev_src)
        btn_wrap = QWidget(mode2_page)
        btn_wrap.setFixedSize(self._pad_item_height, self._pad_item_height)
        btn_layout = QHBoxLayout(btn_wrap)
        btn_layout.setContentsMargins(5, 5, 5, 5)
        btn_layout.setSpacing(0)
        btn_layout.setAlignment(Qt.AlignCenter)
        btn_layout.addWidget(change_btn)
        mode2_layout.addWidget(path_label, 1, Qt.AlignLeft | Qt.AlignVCenter)
        mode2_layout.addWidget(btn_wrap, 0, Qt.AlignRight | Qt.AlignVCenter)
        stack.addWidget(mode2_page)
        stack.setCurrentIndex(0)
        return box, desc_label, stack, path_label, change_btn

    def _create_pad_choice_box(
        self,
        desc_text: str,
        default_choice: str,
        on_change,
    ) -> tuple[QFrame, QLabel, QStackedLayout, dict[str, QPushButton]]:
        box = QFrame(self._right_pad)
        box.setObjectName("PadInfoBox")
        box.setMinimumHeight(self._pad_item_height)
        box.setMaximumHeight(self._pad_item_height)
        box.setStyleSheet("background-color: #2d2d2d; border: none; border-radius: 12px;")
        stack = QStackedLayout(box)
        stack.setContentsMargins(0, 0, 0, 0)
        stack.setSpacing(0)

        desc_page = QWidget(box)
        desc_layout = QHBoxLayout(desc_page)
        desc_layout.setContentsMargins(12, 0, 12, 0)
        desc_layout.setSpacing(0)
        desc_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        desc_label = QLabel(desc_text, desc_page)
        desc_label.setObjectName("PadInfoLabel")
        desc_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        desc_label.setStyleSheet("color: #f5f5f5; font-weight: 800; font-size: 16px; background-color: transparent; border: none;")
        desc_layout.addWidget(desc_label, 1, Qt.AlignLeft | Qt.AlignVCenter)
        stack.addWidget(desc_page)

        mode2_page = QWidget(box)
        mode2_layout = QHBoxLayout(mode2_page)
        mode2_layout.setContentsMargins(5, 5, 5, 5)
        mode2_layout.setSpacing(5)
        mode2_layout.setAlignment(Qt.AlignVCenter)
        group = QButtonGroup(mode2_page)
        group.setExclusive(True)
        buttons: dict[str, QPushButton] = {}
        for choice in ("DB", "DATASET", "MODEL"):
            btn = QPushButton(choice, mode2_page)
            btn.setObjectName("PadChoiceButton")
            btn.setCheckable(True)
            btn.setChecked(choice == default_choice)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            try:
                btn.setAutoFillBackground(True)
            except Exception:
                pass
            btn.clicked.connect(lambda _=False, c=choice: on_change(c))
            btn._choice_indicator_hide_text = True
            group.addButton(btn)
            buttons[choice] = btn
            mode2_layout.addWidget(btn, 1)
        self._style_choice_buttons(buttons)
        stack.addWidget(mode2_page)
        stack.setCurrentIndex(0)
        return box, desc_label, stack, buttons

    def _create_pad_loading_panel(self) -> QWidget:
        panel = QWidget(self._right_pad)
        panel.setObjectName("PadLoadingPanel")
        panel.setStyleSheet(
            "background-color: rgb(30,30,30); border: none;"
            "border-top-left-radius: 0px; border-bottom-left-radius: 0px;"
            "border-top-right-radius: 24px; border-bottom-right-radius: 24px;"
        )
        try:
            panel.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        except Exception:
            pass
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(45, 13, 13, 13)
        layout.setSpacing(4)
        layout.setAlignment(Qt.AlignTop | Qt.AlignHCenter)

        icon_label = QLabel(panel)
        icon_label.setAlignment(Qt.AlignCenter)
        try:
            ip = _app_icon_path()
            if ip:
                pm = QPixmap(ip)
                if not pm.isNull():
                    icon_label.setPixmap(pm.scaled(132, 132, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except Exception:
            pass

        title = QLabel("Easy Trainer 준비중...", panel)
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #f5f5f5; font-size: 12px; font-weight: 700;")

        bar = QProgressBar(panel)
        bar.setObjectName("PadLoadingBar")
        bar.setRange(0, 0)
        bar.setTextVisible(False)
        bar.setFixedHeight(8)
        bar.setStyleSheet(
            "QProgressBar { background-color: #2b2b2b; border: 1px solid #3a3a3a; border-radius: 4px; }"
            "QProgressBar::chunk { background-color: #7dd3fc; }"
        )

        status_row = QHBoxLayout()
        status_row.setContentsMargins(0, 0, 0, 0)
        status_row.setSpacing(10)
        front = QLabel("프론트엔드: 대기중", panel)
        back = QLabel("백엔드: 대기중", panel)
        for lbl in (front, back):
            lbl.setStyleSheet("color: #e0e0e0; font-size: 11px; font-weight: 700;")
            lbl.setAlignment(Qt.AlignVCenter)
        status_row.addWidget(front, 0, Qt.AlignLeft | Qt.AlignVCenter)
        status_row.addStretch(1)
        status_row.addWidget(back, 0, Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(icon_label)
        layout.addWidget(title)
        layout.addStretch(1)
        layout.addLayout(status_row)
        layout.addWidget(bar)

        self._pad_loading_front_label = front
        self._pad_loading_back_label = back
        self._pad_loading_title = title
        panel.hide()
        return panel

    def _setup_settings_status_mode(self):
        stack = getattr(self, "pad_stack_settings", None)
        if stack is None:
            return
        mode2_page = None
        try:
            mode2_page = stack.widget(1)
        except Exception:
            try:
                item = stack.itemAt(1)
                if item is not None:
                    mode2_page = item.widget()
            except Exception:
                mode2_page = None
        if mode2_page is None:
            return
        mode2_layout = mode2_page.layout()
        if mode2_layout is None:
            mode2_layout = QHBoxLayout(mode2_page)
            mode2_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        mode2_layout.setContentsMargins(5, 5, 5, 5)
        mode2_layout.setSpacing(5)
        container = QWidget(mode2_page)
        vlayout = QVBoxLayout(container)
        vlayout.setContentsMargins(0, 0, 0, 0)
        vlayout.setSpacing(5)

        def _row(text: str):
            row = QWidget(container)
            row.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(5, 0, 0, 0)
            row_layout.setSpacing(5)
            label = QLabel(text, row)
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            label.setStyleSheet("color: #e0e0e0; font-size: 11px; font-weight: 700;")
            dot = QLabel(row)
            dot.setFixedSize(10, 10)
            dot.setStyleSheet("background-color: #e74c3c; border: none; border-radius: 5px;")
            row_layout.addWidget(dot, 0, Qt.AlignVCenter)
            row_layout.addWidget(label, 1, Qt.AlignLeft | Qt.AlignVCenter)
            return row, label, dot

        row_front, front_label, front_dot = _row("프론트엔드")
        row_back, back_label, back_dot = _row("백엔드")
        vlayout.addWidget(row_front, 1)
        vlayout.addWidget(row_back, 1)
        mode2_layout.addWidget(container, 1)

        self._pad_status_front_label = front_label
        self._pad_status_back_label = back_label
        self._pad_status_front_dot = front_dot
        self._pad_status_back_dot = back_dot

    def _ensure_choice_indicator(self, btn: QPushButton):
        indicator = getattr(btn, "_choice_indicator", None)
        if indicator is not None:
            try:
                self._apply_choice_indicator_text_visibility(btn, indicator)
            except Exception:
                pass
            return indicator
        indicator = QWidget(btn)
        indicator.setObjectName("PadChoiceIndicator")
        try:
            indicator.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        except Exception:
            pass
        layout = QHBoxLayout(indicator)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        dot = QLabel("✓", indicator)
        dot.setObjectName("PadChoiceIndicatorDot")
        dot.setAlignment(Qt.AlignCenter)
        dot.setFixedSize(10, 10)
        dot.setStyleSheet(
            "background-color: #2ecc71; color: #ffffff; border-radius: 5px; font-size: 8px; font-weight: 700;"
        )
        text = QLabel("선택됨", indicator)
        text.setObjectName("PadChoiceIndicatorText")
        text.setStyleSheet("color: #2ecc71; font-size: 8px; font-weight: 600;")
        layout.addWidget(text, 0, Qt.AlignVCenter)
        layout.addWidget(dot, 0, Qt.AlignVCenter)
        try:
            dot.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            text.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        except Exception:
            pass
        indicator.adjustSize()
        btn._choice_indicator = indicator
        self._apply_choice_indicator_text_visibility(btn, indicator, text)
        return indicator

    def _apply_choice_indicator_text_visibility(
        self,
        btn: QPushButton,
        indicator: QWidget,
        text_label: QLabel | None = None,
    ):
        hide_text = bool(getattr(btn, "_choice_indicator_hide_text", False))
        label = text_label
        if label is None:
            try:
                label = indicator.findChild(QLabel, "PadChoiceIndicatorText")
            except Exception:
                label = None
        if label is not None:
            try:
                label.setVisible(not hide_text)
            except Exception:
                pass
        try:
            layout = indicator.layout()
            if layout is not None:
                layout.setSpacing(0 if hide_text else 2)
        except Exception:
            pass
        try:
            indicator.adjustSize()
        except Exception:
            pass

    def _position_choice_indicator(self, btn: QPushButton):
        indicator = self._ensure_choice_indicator(btn)
        try:
            indicator.adjustSize()
        except Exception:
            pass
        margin = 3
        try:
            x = max(0, btn.width() - indicator.width() - margin)
            y = max(0, btn.height() - indicator.height() - margin)
            indicator.move(x, y)
        except Exception:
            pass

    def _style_choice_buttons(self, buttons: dict[str, QPushButton]):
        if not buttons:
            return
        for btn in buttons.values():
            try:
                btn.setAttribute(Qt.WA_StyledBackground, True)
            except Exception:
                pass
            try:
                checked = btn.isChecked()
            except Exception:
                checked = False
            bg = "rgb(230,230,230)" if checked else "rgb(30,30,30)"
            fg = "#000000" if checked else "rgb(230,230,230)"
            btn.setStyleSheet(
                "background-color: "
                + bg
                + "; color: "
                + fg
                + "; border: none; border-radius: 8px; font-weight: 700; font-size: 11px; padding: 0px;"
            )
            indicator = self._ensure_choice_indicator(btn)
            try:
                indicator.setVisible(bool(checked))
            except Exception:
                pass
            self._position_choice_indicator(btn)

    def _position_floating_bar(self, force: bool = False):
        if self._floating_user_moved and not force:
            return
        try:
            screen = QApplication.primaryScreen()
            geo = screen.availableGeometry() if screen else None
            if not geo:
                return
            margin = 12
            if force or self._anchor_left is None:
                self._anchor_left = geo.x() + margin
            anchor_left = self._anchor_left
            x = anchor_left
            y = geo.y() + geo.height() - self.height() - margin
            self.move(x, y)
            if force:
                self._floating_user_moved = False
        except Exception:
            pass

    def _apply_pill_width(self, width: int, keep_right: bool = False):
        try:
            pad = getattr(self, "_right_pad", None)
            if pad is None:
                return
            new_width = max(0, int(width))
            pad.setFixedWidth(new_width)
            total_width = max(1, int(getattr(self, "_pill_collapsed_width", 0)) + new_width)
            right_edge = None
            if keep_right:
                try:
                    right_edge = self.x() + self.width()
                except Exception:
                    right_edge = None
            try:
                if self.width() != total_width:
                    self.resize(total_width, self.height())
            except Exception:
                pass
            if right_edge is not None:
                try:
                    self.move(right_edge - total_width, self.y())
                except Exception:
                    pass
            try:
                self._anchor_left = self.x()
            except Exception:
                pass
            self._place_pad(new_width)
        except Exception:
            pass

    def _animate_pill(self, expand: bool):
        if getattr(self, "_pill_loading", False):
            return
        target = self._pad_width if expand else 0
        prev_expanded = self._pill_expanded
        if expand == prev_expanded and self._pill_anim:
            return
        self._pill_expanded = expand
        if expand and not prev_expanded:
            self._start_pad_description_window()
        elif not expand:
            try:
                self._pad_desc_timer.stop()
            except Exception:
                pass
            self._pad_desc_window_active = False
            labels = getattr(self, "_pad_desc_labels", [])
            self._set_pad_description_indices(set(range(len(labels))))
        try:
            current = max(0, getattr(self, "_right_pad", self.pill).width())
        except Exception:
            current = 0
        if self._pill_anim:
            try:
                self._pill_anim.stop()
            except Exception:
                pass
        anim = QVariantAnimation(self)
        anim.setDuration(200)
        anim.setStartValue(current)
        anim.setEndValue(target)
        try:
            anim.setEasingCurve(QEasingCurve.InOutCubic)
        except Exception:
            pass
        anim.valueChanged.connect(lambda v: self._apply_pill_width(int(v), keep_right=False))
        def _finish():
            self._apply_pill_width(target, keep_right=False)
        anim.finished.connect(_finish)
        self._pill_anim = anim
        anim.start()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        try:
            pad = getattr(self, "_right_pad", None)
            if pad:
                self._place_pad(pad.width())
        except Exception:
            pass
        self._position_loading_ring()
        self._position_loading_panel()
        self._position_status_lights()

    def _set_status_light(self, light: QLabel, ok: bool):
        color = "#2ecc71" if ok else "#e74c3c"
        radius = 4
        try:
            radius = max(1, min(int(light.width()), int(light.height())) // 2)
        except Exception:
            radius = 4
        light.setStyleSheet(
            f"background-color: {color}; border: none; border-radius: {radius}px;"
        )

    def _position_status_lights(self):
        for light in (self._frontend_light, self._backend_light):
            try:
                light.hide()
            except Exception:
                pass
        return

    def _position_loading_ring(self):
        ring = getattr(self, "_pill_loading_ring", None)
        pill = getattr(self, "pill", None)
        if ring is None or pill is None:
            return
        try:
            ring.setGeometry(pill.rect())
            ring.raise_()
        except Exception:
            pass

    def _position_loading_panel(self):
        panel = getattr(self, "_pad_loading_panel", None)
        pad = getattr(self, "_right_pad", None)
        if panel is None or pad is None:
            return
        try:
            panel.setGeometry(pad.rect())
            panel.raise_()
        except Exception:
            pass

    def _set_pad_boxes_visible(self, visible: bool):
        for box in getattr(self, "_pad_boxes", []):
            if box is None:
                continue
            try:
                box.setVisible(visible)
            except Exception:
                pass

    def _update_loading_panel_status(self, frontend_ok: bool, backend_ok: bool):
        stopping = bool(getattr(self, "_restart_in_progress", False)) and getattr(self, "_restart_phase", "") == "stopping"
        front = getattr(self, "_pad_loading_front_label", None)
        back = getattr(self, "_pad_loading_back_label", None)
        if front is not None:
            if stopping:
                state = "종료중"
            else:
                state = "준비완료" if frontend_ok else "대기중"
            text = f"프론트엔드: {state}"
            color = "#2ecc71" if state == "준비완료" else "#e74c3c"
            front.setText(text)
            front.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: 700;")
        if back is not None:
            if stopping:
                state = "종료중"
            else:
                state = "준비완료" if backend_ok else "대기중"
            text = f"백엔드: {state}"
            color = "#2ecc71" if state == "준비완료" else "#e74c3c"
            back.setText(text)
            back.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: 700;")

    def _update_pad_status_indicators(self, frontend_ok: bool, backend_ok: bool):
        front_label = getattr(self, "_pad_status_front_label", None)
        back_label = getattr(self, "_pad_status_back_label", None)
        front_dot = getattr(self, "_pad_status_front_dot", None)
        back_dot = getattr(self, "_pad_status_back_dot", None)
        if front_dot is not None:
            self._set_status_light(front_dot, frontend_ok)
        if back_dot is not None:
            self._set_status_light(back_dot, backend_ok)
        if front_label is not None:
            front_label.setStyleSheet("color: #e0e0e0; font-size: 11px; font-weight: 700;")
        if back_label is not None:
            back_label.setStyleSheet("color: #e0e0e0; font-size: 11px; font-weight: 700;")

    def _update_status_lights(self):
        try:
            frontend_ok = self._is_frontend_up(timeout=0.3)
        except Exception:
            frontend_ok = False
        try:
            backend_ok = self._check_backend_ready(timeout=0.4)
        except Exception:
            backend_ok = False
        try:
            self._set_status_light(self._frontend_light, frontend_ok)
            self._set_status_light(self._backend_light, backend_ok)
        except Exception:
            pass
        self._update_loading_panel_status(frontend_ok, backend_ok)
        self._update_pad_status_indicators(frontend_ok, backend_ok)

    def _set_pill_buttons_enabled(self, enabled: bool):
        for btn in (
            getattr(self, "btn_open_browser", None),
            getattr(self, "btn_quick_apply", None),
            getattr(self, "btn_folder", None),
            getattr(self, "btn_logs", None),
            getattr(self, "btn_settings", None),
            getattr(self, "btn_exit", None),
        ):
            if btn is None:
                continue
            try:
                btn.setEnabled(enabled)
            except Exception:
                pass

    def _set_pill_loading(self, loading: bool):
        if loading == getattr(self, "_pill_loading", False):
            return
        self._pill_loading = loading
        self._set_pill_buttons_enabled(not loading)
        if loading:
            exit_btn = getattr(self, "btn_exit", None)
            if exit_btn is not None:
                try:
                    exit_btn.setEnabled(True)
                except Exception:
                    pass
        if loading:
            try:
                if self._pill_anim:
                    self._pill_anim.stop()
            except Exception:
                pass
            try:
                self._pad_desc_timer.stop()
            except Exception:
                pass
            self._pad_desc_window_active = False
            self._pill_expanded = True
            self._apply_pill_width(self._pad_width, keep_right=False)
            self._set_pad_boxes_visible(False)
            panel = getattr(self, "_pad_loading_panel", None)
            if panel is not None:
                panel.setVisible(True)
                self._position_loading_panel()
            self._update_loading_panel_status(
                self._is_frontend_up(timeout=0.3),
                self._check_backend_ready(timeout=0.4),
            )
        else:
            self._set_pad_boxes_visible(True)
            panel = getattr(self, "_pad_loading_panel", None)
            if panel is not None:
                panel.setVisible(False)
            self._pill_expanded = False
            self._apply_pill_width(0, keep_right=False)
        ring = getattr(self, "_pill_loading_ring", None)
        if ring is not None:
            ring.stop()

    def _apply_pill_mask(self):
        try:
            pill = getattr(self, "pill", None)
            if pill is None:
                return
            rect = pill.rect()
            if rect.width() <= 0 or rect.height() <= 0:
                return
            radius = max(1, min(rect.width(), rect.height()) // 2)
            path = QPainterPath()
            path.addRoundedRect(rect, float(radius), float(radius))
            region = QRegion(path.toFillPolygon().toPolygon())
            pill.setMask(region)
            origin = pill.mapTo(self, QPoint(0, 0))
            self.setMask(region.translated(origin.x(), origin.y()))
        except Exception:
            pass

    def _global_pos_from_event(self, event):
        try:
            return event.globalPosition().toPoint()
        except Exception:
            try:
                return event.globalPos()
            except Exception:
                return None

    def _register_drag_targets(self, *widgets):
        for w in widgets:
            if not w:
                continue
            try:
                w.installEventFilter(self)
            except Exception:
                pass
            try:
                for child in w.findChildren(QWidget):
                    child.installEventFilter(self)
            except Exception:
                pass

    def _is_drag_target(self, obj) -> bool:
        try:
            for btn in (
                getattr(self, "btn_folder", None),
                getattr(self, "btn_logs", None),
            ):
                if btn is not None:
                    try:
                        if obj is btn or btn.isAncestorOf(obj):
                            return False
                    except Exception:
                        pass
            if obj is self or obj is getattr(self, "pill", None):
                return True
            pill = getattr(self, "pill", None)
            if pill is not None:
                try:
                    if pill.isAncestorOf(obj):
                        return True
                except Exception:
                    pass
            cw = self.centralWidget()
            if cw is not None and cw is obj:
                return True
        except Exception:
            pass
        return False

    def eventFilter(self, obj, event):
        try:
            etype = event.type()
        except Exception:
            etype = None
        if etype == QEvent.Resize:
            try:
                if isinstance(obj, QPushButton) and hasattr(obj, "_choice_indicator"):
                    self._position_choice_indicator(obj)
            except Exception:
                pass
        if etype in (QEvent.Enter, QEvent.HoverEnter, QEvent.Leave, QEvent.HoverLeave):
            btn_entry = self._pill_button_entry(obj)
            if btn_entry is not None:
                idx, btn = btn_entry
                if etype in (QEvent.Enter, QEvent.HoverEnter):
                    self._start_pill_hover_timer(idx, btn)
                else:
                    self._clear_pill_hover(btn)
            self._update_pad_description_mode()
            if etype in (QEvent.Enter, QEvent.HoverEnter):
                if self._cursor_over_pill(strict=True):
                    self._animate_pill(True)
            else:
                if not self._cursor_over_pill(strict=True):
                    self._animate_pill(False)
        if self._is_drag_target(obj):
            if etype == QEvent.MouseButtonPress:
                try:
                    if event.button() == Qt.LeftButton:
                        gp = self._global_pos_from_event(event)
                        if gp:
                            self._dragging = True
                            self._drag_offset = gp - self.pos()
                            self._floating_user_moved = True
                            return False  # allow clicks to propagate
                except Exception:
                    pass
            elif etype == QEvent.MouseMove:
                if self._dragging:
                    gp = self._global_pos_from_event(event)
                    if gp:
                        self.move(gp - self._drag_offset)
                        return True
            elif etype in (QEvent.MouseButtonRelease, QEvent.MouseButtonDblClick):
                if self._dragging:
                    self._dragging = False
                    try:
                        self._anchor_left = self.x()
                    except Exception:
                        pass
                    return False
            elif etype in (QEvent.Enter, QEvent.HoverEnter):
                try:
                    self._collapse_timer.stop()
                except Exception:
                    pass
                if self._cursor_over_pill(strict=True):
                    self._animate_pill(True)
            elif etype in (QEvent.Leave, QEvent.HoverLeave):
                try:
                    self._collapse_timer.stop()
                except Exception:
                    pass
                if not self._cursor_over_pill(strict=True):
                    self._animate_pill(False)
        return super().eventFilter(obj, event)

    def _ensure_main_window_visible(self):
        if self._main_visible:
            self._position_floating_bar()
            if not self.isVisible():
                self.show()
            return
        self._position_floating_bar()
        self.show()
        try:
            self.raise_()
        except Exception:
            pass
        self._main_visible = True

    def _close_log_windows(self):
        """Close any opened log dialogs when main UI exits."""
        for dlg in list(self._log_windows):
            try:
                dlg.close()
            except Exception:
                pass
        self._log_windows.clear()
        self._close_devtools()

    def closeEvent(self, event):
        """Exit and stop the service container so next launch starts cleanly."""
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
        self._close_log_windows()
        self._stop_inline_logs()
        self._restart_in_progress = True
        self._restart_phase = "stopping"
        self._show_preload_dialog("Easy Trainer 종료중...", auto_open_new=False)
        try:
            self._update_loading_panel_status(
                self._is_frontend_up(timeout=0.3),
                self._check_backend_ready(timeout=0.4),
            )
        except Exception:
            pass
        if not docker_compose_available() or not self._is_valid_project_root(self.project_root):
            QApplication.instance().quit()
            return
        def _after_down(_code: int, *_):
            QApplication.instance().quit()
        self._run_backend_kill()
        self.append_log("[EXIT] docker compose down --remove-orphans --volumes ...")
        self.run_compose(["down", "--remove-orphans", "--volumes"], on_finish=_after_down)

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
                        QMessageBox.critical(dlg, "오류", f"서비스 시작 실패 (code={exit_code})")
                        return
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
        if getattr(self, "_pill_loading", False):
            try:
                self.btn_quick_apply.setEnabled(False)
            except Exception:
                pass
            return
        valid_src = self._is_valid_dev_src(self.dev_src_root)
        try:
            self.btn_quick_apply.setEnabled(True)
            if not valid_src:
                self.btn_quick_apply.setToolTip("원본 프로젝트 경로를 먼저 선택하세요.")
        except Exception:
            pass
        self._update_open_ui_tooltip()
        self._update_pad_choice_tooltips()

    def update_state_label(self):
        installed = self.is_installed()
        running = self._get_running_services()
        loading = getattr(self, "_pill_loading", False)
        status_parts = []
        status_parts.append("설치됨" if installed else "미설치")
        status_parts.append("실행 중" if running else "중지됨")
        text = " · ".join(status_parts)
        try:
            self.state_label.setText(text)
        except Exception:
            pass
        try:
            self.btn_open_browser.setEnabled(bool(running) and not loading)
        except Exception:
            pass
        if loading:
            try:
                self.btn_quick_apply.setEnabled(False)
            except Exception:
                pass
        if self.status_label.isVisible():
            self.status_label.setText(text)
        if not loading:
            self.update_buttons()
        self._position_floating_bar()

    def update_dev_src_label(self):
        self.update_project_label()

    def update_project_label(self):
        full_text = str(self.dev_src_root) if self.dev_src_root else ""
        display = full_text
        if not display:
            display = "원본 경로 미설정"
        elif len(display) > 28:
            display = "..." + display[-25:]
        try:
            self.path_hint_label.setText(display)
            self.path_hint_label.setToolTip(full_text or "원본 경로 미설정 - 클릭하여 선택하세요.")
        except Exception:
            pass
        pad_label = getattr(self, "pad_path_label", None)
        if pad_label is not None:
            try:
                pad_display = str(self.dev_src_root) if self.dev_src_root else "미정"
                pad_label.setText(pad_display if pad_display else "미정")
                pad_label.setToolTip(full_text or "원본 경로 미설정 - 클릭하여 선택하세요.")
            except Exception:
                pass
        self._position_floating_bar()

    def _is_valid_project_root(self, path: Path) -> bool:
        try:
            if not path or not path.is_dir():
                return False
            required = [
                path / "docker-compose.yml",
                path / "Dockerfile",
                path / "start_services.sh",
                path / "src" / "backend",
                path / "src" / "ui",
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

    def _service_exists(self) -> bool:
        """Check whether the service container exists (running or stopped)."""
        try:
            import subprocess
            out = subprocess.check_output(
                ["docker", "ps", "-a", "--format", "{{.Names}}"],
                text=True,
            )
            names = [n.strip() for n in out.splitlines() if n.strip()]
            return "easy_collector_service" in names
        except Exception:
            return False


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
        version = APP_VERSION or "0.0.0"
        title = f"Easy Trainer v{version} ({label})"
        try:
            self.setWindowTitle(title)
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
            res = subprocess.run(
                ["pkexec", "/bin/sh", "-c", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
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
            )
            return res.returncode == 0
        except Exception:
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
        """Remove stale legacy containers that can block compose up due to duplicate names."""
        targets = [
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
            self.append_log(f"[{reason}] docker compose restart service ...")
            cmd = ["restart", "service"]
        elif exists:
            self.append_log(f"[{reason}] docker compose start service ...")
            cmd = ["start", "service"]
        else:
            self.append_log(f"[{reason}] docker compose up -d --no-recreate service ...")
            cmd = ["up", "-d", "--no-recreate", "service"]
        self.run_compose(cmd, on_finish=on_finish)

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
        self._show_preload_dialog("Easy Trainer 준비중...")
        self._ensure_service_running("START", restart_if_running=False, on_finish=self._on_start_finished)

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
        self._run_backend_kill()
        self.append_log("[STOP] docker compose stop service ...")
        self.run_compose(["stop", "service"], on_finish=self._on_stop_finished)

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
        self.load_ui(open_mode=self._open_ui_mode)

    def _on_exit_action(self):
        action = getattr(self, "_exit_action", "EXIT")
        if action == "RESTART":
            self._restart_service_container()
            return
        self.close()

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
        self._show_preload_dialog("Easy Trainer 준비중...")
        def _after_restart(exit_code: int, *_):
            self.append_log("[RESTART] 완료")
            if exit_code == 0:
                self._wait_for_services_ready(self.load_ui)
            else:
                self._hide_preload_dialog()
        self._clear_conflicting_containers()
        self._ensure_service_running("RESTART", restart_if_running=True, on_finish=_after_restart)

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
            self._sync_dirs(self.dev_src_root / "ros2_ws" / "src", self.project_root / "ros2_ws" / "src")
            self._sync_core_files()
            self._apply_compose_variant(self.install_variant)
            return True
        except Exception as e:
            if show_errors:
                QMessageBox.critical(self, "오류", f"동기화 실패: {e}")
            else:
                self.append_log(f"[SYNC][ERROR] {e}")
            return False

    def _show_preload_dialog(self, message: str, auto_open_new: bool | None = None):
        text = message or "Easy Trainer 준비중..."
        self._preload_message = text
        self._preload_detail = "프론트엔드: 대기중 | 백엔드: 대기중"
        if auto_open_new is None:
            auto_open_new = True
        self._preload_auto_open_new = auto_open_new
        title = getattr(self, "_pad_loading_title", None)
        if title is not None:
            title.setText(text)
        self._set_pill_loading(True)
        self._ensure_main_window_visible()
        self._start_inline_logs()

    def _set_preload_detail(self, text: str):
        self._preload_detail = text
        # Show restart button if waiting long (timer handles when to enable)
        if self._restart_btn_timer is None:
            self._start_restart_timer()

    def _hide_preload_dialog(self, ready: bool = False):
        self._stop_ready_timer()
        self._stop_ready_timeout()
        self._stop_restart_timer()
        self._stop_inline_logs()
        self._restart_in_progress = False
        self._restart_phase = ""
        self._set_pill_loading(False)
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

    def _start_restart_timer(self):
        try:
            if self._restart_btn_timer is not None:
                return
            self._restart_btn_timer = QTimer(self)
            self._restart_btn_timer.setSingleShot(True)
            self._restart_btn_timer.setInterval(60000)  # 60s
            self._restart_btn_timer.timeout.connect(self._enable_restart_button)
            self._restart_btn_timer.start()
        except Exception:
            self._restart_btn_timer = None

    def _stop_restart_timer(self):
        if self._restart_btn_timer:
            try:
                self._restart_btn_timer.stop()
            except Exception:
                pass
            self._restart_btn_timer = None

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

    def _schedule_inline_log_retry(self):
        try:
            if self._inline_log_retry is None:
                self._inline_log_retry = QTimer(self)
                self._inline_log_retry.setSingleShot(True)
                self._inline_log_retry.setInterval(1000)
                self._inline_log_retry.timeout.connect(self._start_inline_logs)
            self._inline_log_retry.start()
        except Exception:
            self._inline_log_retry = None

    def _stop_inline_log_retry(self):
        if self._inline_log_retry:
            try:
                self._inline_log_retry.stop()
            except Exception:
                pass
            self._inline_log_retry = None

    def _enable_restart_button(self):
        try:
            if self._preload_dialog:
                self._preload_dialog.show_restart(True, enabled=True)
        except Exception:
            pass

    def _on_restart_button_clicked(self):
        if not docker_compose_available() or not self._is_valid_project_root(self.project_root):
            try:
                QMessageBox.critical(self, "오류", self._compose_help_text())
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
        self._show_preload_dialog("서비스 재시작 중...")
        self._set_preload_detail("서비스 재시작 중...")
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
            self.run_compose(["up", "-d", "service"], on_finish=_after_up)

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
        self._show_preload_dialog("Easy Trainer 준비중...", auto_open_new=auto_open_new)

        def poll():
            backend_ok = self._check_backend_ready()
            frontend_ok = self._is_frontend_ready()
            detail = f"프론트엔드: {'준비완료' if frontend_ok else '대기중'} | 백엔드: {'준비완료' if backend_ok else '대기중'}"
            self._set_preload_detail(detail)
            if backend_ok and frontend_ok:
                self._stop_ready_timer()
                self._stop_ready_timeout()
                self._hide_preload_dialog(ready=True)
                try:
                    if auto_open_new:
                        self.load_ui(open_mode="NEW")
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

    def _run_backend_kill(self):
        """Best-effort: run kill.sh inside the service container to clean ROS/backend processes."""
        container = "easy_collector_service"
        if "service" not in self._get_running_services():
            return
        try:
            import subprocess
            result = subprocess.run(
                ["docker", "exec", "-T", container, "bash", "-lc", "/root/src/kill.sh"],
                capture_output=True,
                text=True,
                timeout=10,
            )
            msg = result.stdout.strip() if result.stdout else ""
            if result.returncode == 0:
                self.append_log("[STOP] kill.sh executed inside container.")
                if msg:
                    self.append_log(f"[STOP][kill.sh] {msg}")
            else:
                err = result.stderr.strip() if result.stderr else ""
                self.append_log(f"[STOP][WARN] kill.sh returned {result.returncode}: {err or 'no output'}")
        except FileNotFoundError:
            self.append_log("[STOP][WARN] Docker not found; skip kill.sh.")
        except Exception as e:
            self.append_log(f"[STOP][WARN] kill.sh failed: {e}")

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
        logs = self._collect_docker_logs("easy_collector_service", tail=200)
        self._show_docker_logs_follow(
            "서비스 시작 실패",
            container="easy_collector_service",
            tail=200,
            initial_text=logs,
        )
        self._enable_restart_button()
        if docker_compose_available() and self._is_valid_project_root(self.project_root):
            self.append_log("[STOP] docker compose stop service ...")
            self.run_compose(["stop", "service"], on_finish=self._on_stop_finished)

    def _spawn_browser(self, program: str, args: list[str]) -> bool:
        try:
            proc = subprocess.Popen(
                [program, *args],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            try:
                proc.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                return True
            if proc.returncode == 0:
                return True
            try:
                self.append_log(f"[WEB][ERROR] Browser exited ({proc.returncode}): {program}")
            except Exception:
                pass
            return False
        except Exception as e:
            try:
                self.append_log(f"[WEB][ERROR] Browser spawn failed: {program} ({e})")
            except Exception:
                pass
            return False

    def _strip_browser_placeholders(self, args: list[str]) -> list[str]:
        placeholders = ("%u", "%U", "%f", "%F", "%s", "%S", "%i", "%c", "%k")
        cleaned: list[str] = []
        for arg in args:
            if any(ph in arg for ph in placeholders):
                continue
            cleaned.append(arg)
        return cleaned

    def _strip_new_window_args(self, args: list[str]) -> list[str]:
        return [arg for arg in args if arg not in ("--new-window", "-new-window")]

    def _with_new_window_args(self, program: str, args: list[str]) -> list[str]:
        for flag in ("--new-window", "-new-window"):
            if flag in args:
                return args
        base = Path(program).name
        known = {
            "google-chrome",
            "google-chrome-stable",
            "chromium",
            "chromium-browser",
            "microsoft-edge",
            "microsoft-edge-stable",
            "brave-browser",
            "firefox",
        }
        if base in known:
            return ["--new-window", *args]
        return args

    def _with_new_tab_args(self, program: str, args: list[str]) -> list[str]:
        for flag in ("--new-tab", "-new-tab"):
            if flag in args:
                return args
        base = Path(program).name
        if base in ("firefox", "firefox-bin"):
            return ["-new-tab", *args]
        if base in (
            "google-chrome",
            "google-chrome-stable",
            "chromium",
            "chromium-browser",
            "microsoft-edge",
            "microsoft-edge-stable",
            "brave-browser",
            "brave",
        ):
            return ["--new-tab", *args]
        return args

    def _parse_browser_env(self, value: str) -> tuple[str, list[str]] | None:
        if not value:
            return None
        for entry in value.split(":"):
            entry = entry.strip()
            if not entry:
                continue
            try:
                parts = shlex.split(entry)
            except Exception:
                parts = entry.split()
            if not parts:
                continue
            cmd = parts[0]
            args = self._strip_browser_placeholders(parts[1:])
            if shutil.which(cmd) or Path(cmd).is_file():
                return cmd, args
        return None

    def _read_desktop_exec(self, desktop_id: str) -> tuple[str, list[str]] | None:
        if not desktop_id.endswith(".desktop"):
            return None
        candidates = [
            Path.home() / ".local" / "share" / "applications" / desktop_id,
            Path("/usr/share/applications") / desktop_id,
        ]
        for path in candidates:
            try:
                if not path.exists():
                    continue
                for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
                    if not line.startswith("Exec="):
                        continue
                    cmdline = line[len("Exec="):].strip()
                    if not cmdline:
                        continue
                    parts = shlex.split(cmdline)
                    if not parts:
                        continue
                    cmd = parts[0]
                    args = self._strip_browser_placeholders(parts[1:])
                    return cmd, args
            except Exception:
                continue
        return None

    def _resolve_default_browser_cmd(self) -> tuple[str, list[str]] | None:
        env_browser = os.environ.get("BROWSER", "").strip()
        parsed = self._parse_browser_env(env_browser)
        if parsed:
            return parsed
        desktop_id = ""
        try:
            if shutil.which("xdg-settings"):
                res = subprocess.run(
                    ["xdg-settings", "get", "default-web-browser"],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                desktop_id = (res.stdout or "").strip()
        except Exception:
            desktop_id = ""
        if not desktop_id:
            try:
                if shutil.which("xdg-mime"):
                    res = subprocess.run(
                        ["xdg-mime", "query", "default", "x-scheme-handler/http"],
                        capture_output=True,
                        text=True,
                        check=False,
                    )
                    desktop_id = (res.stdout or "").strip()
            except Exception:
                desktop_id = ""
        if desktop_id:
            resolved = self._read_desktop_exec(desktop_id)
            if resolved:
                cmd, args = resolved
                if shutil.which(cmd) or Path(cmd).is_file():
                    return cmd, args
        return None

    def _browser_process_candidates(self, program: str) -> list[str]:
        base = Path(program).name
        aliases = {
            "google-chrome": ["chrome", "google-chrome", "google-chrome-stable"],
            "google-chrome-stable": ["chrome", "google-chrome", "google-chrome-stable"],
            "chromium": ["chromium", "chromium-browser"],
            "chromium-browser": ["chromium", "chromium-browser"],
            "microsoft-edge": ["msedge", "microsoft-edge", "microsoft-edge-stable"],
            "microsoft-edge-stable": ["msedge", "microsoft-edge", "microsoft-edge-stable"],
            "brave-browser": ["brave", "brave-browser"],
            "firefox": ["firefox"],
        }
        return aliases.get(base, [base])

    def _fallback_browser_programs(self) -> list[str]:
        candidates = [
            "google-chrome",
            "google-chrome-stable",
            "chromium",
            "chromium-browser",
            "microsoft-edge",
            "microsoft-edge-stable",
            "brave-browser",
            "firefox",
        ]
        return [c for c in candidates if shutil.which(c)]

    def _open_url_fallback_browser(self, url: QUrl, new_window: bool) -> bool:
        url_str = url.toString()
        for program in self._fallback_browser_programs():
            args: list[str] = []
            if new_window:
                args = self._with_new_window_args(program, args)
            else:
                args = self._with_new_tab_args(program, args)
            if self._spawn_browser(program, args + [url_str]):
                return True
        return False

    def _running_browser_cmd(self) -> tuple[str, list[str]] | None:
        cmd_env = os.environ.get("EASYCOLLECTOR_BROWSER_CMD", "").strip()
        args_env = os.environ.get("EASYCOLLECTOR_BROWSER_ARGS", "").strip()
        if cmd_env and self._is_any_process_running(self._browser_process_candidates(cmd_env)):
            args = shlex.split(args_env) if args_env else []
            args = self._strip_browser_placeholders(args)
            args = self._strip_new_window_args(args)
            args = self._with_new_tab_args(cmd_env, args)
            return cmd_env, args
        resolved = self._resolve_default_browser_cmd()
        if resolved:
            program, args = resolved
            if self._is_any_process_running(self._browser_process_candidates(program)):
                args = self._strip_new_window_args(args)
                args = self._with_new_tab_args(program, args)
                return program, args
        for program in self._fallback_browser_programs():
            if self._is_any_process_running(self._browser_process_candidates(program)):
                args = self._with_new_tab_args(program, [])
                return program, args
        return None

    def _is_any_process_running(self, names: list[str]) -> bool:
        if not names:
            return False
        if shutil.which("pgrep"):
            for name in names:
                try:
                    res = subprocess.run(
                        ["pgrep", "-x", name],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        check=False,
                    )
                    if res.returncode == 0:
                        return True
                except Exception:
                    continue
        try:
            proc_root = Path("/proc")
            for pid in proc_root.iterdir():
                if not pid.name.isdigit():
                    continue
                try:
                    comm = (pid / "comm").read_text(encoding="utf-8", errors="ignore").strip()
                except Exception:
                    continue
                if comm in names:
                    return True
        except Exception:
            pass
        return False

    def _is_primary_browser_running(self) -> bool:
        cmd_env = os.environ.get("EASYCOLLECTOR_BROWSER_CMD", "").strip()
        if cmd_env:
            names = self._browser_process_candidates(cmd_env)
            return self._is_any_process_running(names)
        resolved = self._resolve_default_browser_cmd()
        if not resolved:
            return False
        program, _args = resolved
        return self._is_any_process_running(self._browser_process_candidates(program))

    def _open_url_smart(self, url: QUrl) -> bool:
        url_str = url.toString()
        cmd_env = os.environ.get("EASYCOLLECTOR_BROWSER_CMD", "").strip()
        args_env = os.environ.get("EASYCOLLECTOR_BROWSER_ARGS", "").strip()
        running = self._is_primary_browser_running()
        if cmd_env:
            args = shlex.split(args_env) if args_env else []
            args = self._strip_browser_placeholders(args)
            if running:
                args = self._strip_new_window_args(args)
            else:
                args = self._with_new_window_args(cmd_env, args)
            if self._spawn_browser(cmd_env, args + [url_str]):
                return True
        resolved = self._resolve_default_browser_cmd()
        if resolved:
            program, args = resolved
            if running:
                args = self._strip_new_window_args(args)
            else:
                args = self._with_new_window_args(program, args)
            if self._spawn_browser(program, args + [url_str]):
                return True
        if running:
            return self._open_url_current_window(url)
        return self._open_url_new_window(url)

    def _open_url_new_window(self, url: QUrl) -> bool:
        url_str = url.toString()
        cmd_env = os.environ.get("EASYCOLLECTOR_BROWSER_CMD", "").strip()
        args_env = os.environ.get("EASYCOLLECTOR_BROWSER_ARGS", "").strip()
        if cmd_env:
            args = shlex.split(args_env) if args_env else []
            args = self._strip_browser_placeholders(args)
            args = self._with_new_window_args(cmd_env, args)
            if self._spawn_browser(cmd_env, args + [url_str]):
                return True
        resolved = self._resolve_default_browser_cmd()
        if resolved:
            program, args = resolved
            args = self._with_new_window_args(program, args)
            if self._spawn_browser(program, args + [url_str]):
                return True
        if self._open_url_fallback_browser(url, new_window=True):
            return True
        return self._open_url_current_window(url)

    def _open_url_current_window(self, url: QUrl) -> bool:
        url_str = url.toString()
        running = self._running_browser_cmd()
        if running:
            program, args = running
            if self._spawn_browser(program, args + [url_str]):
                return True
        cmd_env = os.environ.get("EASYCOLLECTOR_BROWSER_CMD", "").strip()
        args_env = os.environ.get("EASYCOLLECTOR_BROWSER_ARGS", "").strip()
        if cmd_env:
            args = shlex.split(args_env) if args_env else []
            args = self._strip_browser_placeholders(args)
            args = self._strip_new_window_args(args)
            args = self._with_new_tab_args(cmd_env, args)
            if self._spawn_browser(cmd_env, args + [url_str]):
                return True
        resolved = self._resolve_default_browser_cmd()
        if resolved:
            program, args = resolved
            args = self._strip_new_window_args(args)
            args = self._with_new_tab_args(program, args)
            if self._spawn_browser(program, args + [url_str]):
                return True
        if self._open_url_fallback_browser(url, new_window=False):
            return True
        try:
            return QDesktopServices.openUrl(url)
        except Exception:
            return False

    def _open_url_with_mode(self, url: QUrl, mode: str) -> bool:
        if mode == "CURRENT":
            return self._open_url_current_window(url)
        if self._open_url_new_window(url):
            return True
        return self._open_url_current_window(url)

    def _ensure_ui_wait_page(self, target_url: str) -> QUrl | None:
        try:
            UI_LOG_DIR.mkdir(parents=True, exist_ok=True)
            wait_path = UI_LOG_DIR / "ui_wait.html"
            target_js = json.dumps(target_url)
            html = (
                "<!doctype html>\n"
                "<html>\n"
                "<head>\n"
                "  <meta charset=\"utf-8\"/>\n"
                "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"/>\n"
                "  <title>EasyCollector UI</title>\n"
                "  <style>\n"
                "    body { margin: 0; padding: 0; background: #0f1113; color: #f5f5f5; }\n"
                "    .wrap { min-height: 100vh; display: flex; align-items: center; justify-content: center; }\n"
                "    .card { background: #1b1f23; border-radius: 12px; padding: 24px 28px; max-width: 420px; }\n"
                "    .title { font-size: 18px; font-weight: 700; margin-bottom: 8px; }\n"
                "    .hint { font-size: 13px; color: #b0b6bb; }\n"
                "    .link { margin-top: 16px; font-size: 13px; }\n"
                "    a { color: #8ab4f8; text-decoration: none; }\n"
                "    a:hover { text-decoration: underline; }\n"
                "  </style>\n"
                "</head>\n"
                "<body>\n"
                "  <div class=\"wrap\">\n"
                "    <div class=\"card\">\n"
                "      <div class=\"title\">UI is starting...</div>\n"
                "      <div class=\"hint\">The app will open when it is ready.</div>\n"
                "      <div class=\"link\"><a id=\"open-link\" href=\"#\">Open now</a></div>\n"
                "    </div>\n"
                "  </div>\n"
                "  <script>\n"
                f"    const target = {target_js};\n"
                "    const link = document.getElementById(\"open-link\");\n"
                "    if (link) link.href = target;\n"
                "    // The launcher will open the UI once it is fully ready.\n"
                "  </script>\n"
                "</body>\n"
                "</html>\n"
            )
            wait_path.write_text(html, encoding="utf-8")
            return QUrl.fromLocalFile(str(wait_path))
        except Exception:
            return None

    def load_ui(self, open_mode: str | None = None, force_refresh: bool = False):
        url_str = FRONTEND_URL
        if force_refresh:
            try:
                from urllib.parse import urlsplit, urlunsplit, parse_qsl, urlencode
                parts = urlsplit(url_str)
                query = dict(parse_qsl(parts.query, keep_blank_values=True))
                query["_ts"] = str(int(time.time() * 1000))
                url_str = urlunsplit(
                    (parts.scheme, parts.netloc, parts.path, urlencode(query), parts.fragment)
                )
            except Exception:
                sep = "&" if "?" in url_str else "?"
                url_str = f"{url_str}{sep}_ts={int(time.time() * 1000)}"
        url = QUrl(url_str)
        mode = open_mode or getattr(self, "_open_ui_mode", "NEW")
        if mode not in ("NEW", "CURRENT"):
            mode = "NEW"

        def _open():
            if self._open_url_with_mode(url, mode):
                if mode == "CURRENT":
                    self.append_log(f"[WEB] Open current {url.toString()}")
                else:
                    self.append_log(f"[WEB] Open UI {url.toString()}")
                return
            self.append_log(f"[WEB][ERROR] Failed to open {url.toString()}")

        if self._is_ui_ready():
            _open()
            return

        wait_url = self._ensure_ui_wait_page(url_str)
        if wait_url and self._open_url_with_mode(wait_url, mode):
            self.append_log("[WEB] Frontend not ready. Opened waiting page.")

        self.append_log(f"[WEB] Frontend not ready. Waiting for {FRONTEND_URL} ...")
        # Poll until server is up, then open once
        try:
            if getattr(self, "_frontend_timer", None) is not None:
                self._frontend_timer.stop()
        except Exception:
            pass
        self._frontend_timer = QTimer(self)
        self._frontend_timer.setInterval(1000)
        def _poll():
            if self._is_ui_ready():
                try:
                    self._frontend_timer.stop()
                except Exception:
                    pass
                _open()
        self._frontend_timer.timeout.connect(_poll)
        self._frontend_timer.start()

    def _quick_apply_from_dev_src(self):
        if not self._is_valid_dev_src(self.dev_src_root):
            self.load_ui(open_mode="CURRENT")
            return
        self.append_log("[SYNC] 원본에서 빠른 적용 중...")
        if not self._sync_dev_files(show_errors=False):
            self.append_log("[SYNC][WARN] 원본 적용에 실패했습니다. 경로를 확인하세요.")
            return
        # Default to fast path: assume backend autoreload is on (container default) to avoid restarts
        fast_backend_reload = os.environ.get("EC_BACKEND_AUTORELOAD", "1") != "0"
        running = "service" in self._get_running_services()
        if docker_compose_available() and self._is_valid_project_root(self.project_root):
            if self.process is not None and self.process.state() != QProcess.NotRunning:
                if self._pending_quick_apply:
                    self.append_log("[SYNC][INFO] 다른 작업이 끝나면 대기 중인 빠른 동기화를 실행합니다.")
                    return
                self._pending_quick_apply = True
                self.append_log("[SYNC][INFO] 다른 작업 종료 후 빠른 동기화를 바로 실행합니다.")
                def _after_current(*_):
                    try:
                        self.process.finished.disconnect(_after_current)
                    except Exception:
                        pass
                    self._pending_quick_apply = False
                    self._quick_apply_from_dev_src()
                self.process.finished.connect(_after_current)
                return
            if running:
                if fast_backend_reload:
                    self.append_log("[SYNC] 빠른 적용 완료. 컨테이너는 유지하고 autoreload/HMR로 바로 반영합니다.")
                    self.load_ui(open_mode="CURRENT")
                    return
                self.append_log("[SYNC][INFO] 컨테이너 유지 (autreload=off). 필요하면 수동 재시작해 주세요.")
                self.load_ui(open_mode="CURRENT")
                return
            self.append_log("[SYNC] 빠른 적용 완료. 서비스 시작 중...")
            self._show_preload_dialog("서비스 시작 중...")
            self._clear_conflicting_containers()
            def _after_restart(exit_code: int, *_):
                if exit_code == 0:
                    self.append_log("[START] 완료, UI 새로고침 중...")
                    self._wait_for_services_ready(lambda: self.load_ui(open_mode="CURRENT"))
                else:
                    self.append_log(f"[START][ERROR] 시작 실패 (code={exit_code})")
                    self._hide_preload_dialog()
            self._ensure_service_running("SYNC", restart_if_running=False, on_finish=_after_restart)
            return
        self.append_log("[SYNC] 빠른 적용 완료. UI 새로고침 중...")
        self.load_ui(open_mode="CURRENT")

    def _nav_action(self, action: str):
        return

    def keyPressEvent(self, event):
        try:
            key = event.key()
        except Exception:
            key = None
        if key == Qt.Key_Escape:
            try:
                self.close()
                return
            except Exception:
                pass
        try:
            super().keyPressEvent(event)
        except Exception:
            pass

    def _on_quick_apply_clicked(self):
        if not self._is_valid_dev_src(self.dev_src_root):
            self._show_pad_notice(1, "프로젝트 경로 지정", color="#ff6b6b", duration_ms=1000)
            try:
                self._dev_src_prompt_timer.stop()
            except Exception:
                pass
            self._dev_src_prompt_timer.start(1000)
            return
        self._quick_apply_from_dev_src()

    def _update_nav_buttons(self):
        # Navigation buttons removed with floating bar UI
        return

    def _dev_apply_and_restart_and_reload(self):
        self.append_log("[SYNC] 개발 원본에서 backend/ui/compose 반영...")
        if not self._sync_dev_files():
            return
        self.append_log("[SYNC] 완료. 서비스 재시작 중...")
        self._show_preload_dialog("Easy Trainer 준비중...")
        def _after_restart(exit_code: int, *_):
            self.append_log("[RESTART] 완료")
            if exit_code == 0:
                self._wait_for_services_ready(self.load_ui)
            else:
                self._hide_preload_dialog()
        self._clear_conflicting_containers()
        self._ensure_service_running("RESTART", restart_if_running=True, on_finish=_after_restart)

    def open_settings_dialog(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("설정")
        dlg.resize(380, 260)
        layout = QVBoxLayout(dlg)
        layout.setSpacing(10)

        project_text = str(self.project_root) if getattr(self, "project_root", None) else "미설정"
        dev_text = str(self.dev_src_root) if self.dev_src_root else "미설정"
        mode_label = "GPU" if self._current_variant() == "gpu" else "CPU"

        info = QLabel(f"프로젝트: {project_text}\n원본 경로: {dev_text}\n모드: {mode_label}", dlg)
        info.setWordWrap(True)
        layout.addWidget(info)

        btn_logs = QPushButton("로그 보기", dlg)
        btn_restart = QPushButton("서비스 재시작", dlg)
        btn_stop = QPushButton("서비스 중지", dlg)
        btn_open_project = QPushButton("프로젝트 폴더 열기", dlg)

        row1 = QHBoxLayout()
        row1.setSpacing(8)
        row1.addWidget(btn_logs)
        row1.addWidget(btn_open_project)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.setSpacing(8)
        row2.addWidget(btn_restart)
        row2.addWidget(btn_stop)
        layout.addLayout(row2)

        layout.addStretch(1)
        btn_close = QPushButton("닫기", dlg)
        row_close = QHBoxLayout()
        row_close.addStretch(1)
        row_close.addWidget(btn_close)
        layout.addLayout(row_close)

        btn_close.clicked.connect(dlg.accept)
        btn_logs.clicked.connect(self.open_logs_window)
        btn_restart.clicked.connect(self._restart_service_container)
        btn_stop.clicked.connect(self.on_stop)
        def _open_project():
            try:
                if self.project_root:
                    QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.project_root)))
            except Exception:
                pass
        btn_open_project.clicked.connect(_open_project)

        dlg.exec()

    def open_logs_window(self):
        svc = "service"
        backend_log = shlex.quote(str(SERVICE_LOG_DIR / "backend.log"))
        frontend_log = shlex.quote(str(SERVICE_LOG_DIR / "frontend.log"))
        tail_backend = ["exec", "-T", svc, "bash", "-lc", f"tail -n 200 -F {backend_log}"]
        tail_frontend = ["exec", "-T", svc, "bash", "-lc", f"tail -n 200 -F {frontend_log}"]
        self._close_log_windows()
        dialogs: list[QDialog] = []

        dlg_launcher = self._create_ui_log_dialog("Launcher Logs", UI_LOG_FILE)
        if dlg_launcher:
            self._register_log_window_append(dlg_launcher)
            dlg_launcher.show()
            dialogs.append(dlg_launcher)

        dlg_front = self._create_log_dialog("Frontend Logs", tail_frontend)
        if dlg_front:
            self._register_log_window_append(dlg_front)
            dlg_front.show()
            dialogs.append(dlg_front)

        dlg_back = self._create_log_dialog("Backend Logs", tail_backend)
        if dlg_back:
            self._register_log_window_append(dlg_back)
            dlg_back.show()
            dialogs.append(dlg_back)

        self._arrange_log_windows_side_by_side(dialogs)

    def _open_service_log_dialog(self, title: str, log_path: Path):
        svc = "service"
        log_path = shlex.quote(str(log_path))
        tail_args = ["exec", "-T", svc, "bash", "-lc", f"tail -n 200 -F {log_path}"]
        self._close_log_windows()
        dlg = self._create_log_dialog(title, tail_args)
        if dlg:
            self._register_log_window(dlg)
            dlg.show()

    def _open_frontend_logs(self):
        self._open_service_log_dialog("Frontend Logs", SERVICE_LOG_DIR / "frontend.log")

    def _open_backend_logs(self):
        self._open_service_log_dialog("Backend Logs", SERVICE_LOG_DIR / "backend.log")

    def _open_frontend_backend_logs_side_by_side(self):
        svc = "service"
        backend_log = shlex.quote(str(SERVICE_LOG_DIR / "backend.log"))
        frontend_log = shlex.quote(str(SERVICE_LOG_DIR / "frontend.log"))
        tail_backend = ["exec", "-T", svc, "bash", "-lc", f"tail -n 200 -F {backend_log}"]
        tail_frontend = ["exec", "-T", svc, "bash", "-lc", f"tail -n 200 -F {frontend_log}"]
        self._close_log_windows()
        dlg_front = self._create_log_dialog("Frontend Logs", tail_frontend)
        dlg_back = self._create_log_dialog("Backend Logs", tail_backend)
        dialogs: list[QDialog] = []
        if dlg_front:
            self._register_log_window_append(dlg_front)
            dlg_front.show()
            dialogs.append(dlg_front)
        if dlg_back:
            self._register_log_window_append(dlg_back)
            dlg_back.show()
            dialogs.append(dlg_back)
        self._arrange_log_windows_side_by_side(dialogs)

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
        self._close_log_windows()
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

    def _register_log_window_append(self, dlg: QDialog):
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

    def _open_devtools(self):
        """DevTools are not available when using the external browser."""
        return None

    def _close_devtools(self):
        if self._devtools_dialog:
            try:
                self._devtools_dialog.close()
            except Exception:
                pass
        self._devtools_view = None
        self._devtools_dialog = None

    # ------------------------ Process helpers ------------------------
    def run_compose(self, args: list[str], on_finish=None):
        if self.process is not None and self.process.state() != QProcess.NotRunning:
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
        QMainWindow#LauncherWindow, QWidget#PillWrapper { background-color: transparent; }
        QWidget { background-color: #1E1E1E; color: #EEEEEE; }
        QFrame#Divider { background-color: #FFFFFF; max-height: 1px; }
        QFrame#ContentBox { border: 1px solid #FFFFFF; border-radius: 6px; }
        QFrame#FloatingBar { background-color: rgb(30,30,30); border: none; border-radius: 24px; padding: 7px; }
        QFrame#FloatingBarPad {
            background-color: rgb(30,30,30);
            border: none;
            border-top-left-radius: 0px;
            border-bottom-left-radius: 0px;
            border-top-right-radius: 24px;
            border-bottom-right-radius: 24px;
        }
        QFrame#PadInfoBox {
            background-color: #2d2d2d;
            border: none;
            border-radius: 12px;
        }
        QLabel#PadInfoLabel {
            color: #f5f5f5;
            font-weight: 800;
            font-size: 16px;
            letter-spacing: 0.2px;
            background-color: transparent;
            border: none;
        }
        QLabel#PadPathLabel {
            color: #cfd8dc;
            font-size: 11px;
            font-weight: 600;
            background-color: transparent;
            border: none;
        }
        QLabel#PadMode2Label {
            color: #cfd8dc;
            font-size: 14px;
            font-weight: 700;
            background-color: transparent;
            border: none;
        }
        QPushButton#PadChoiceButton {
            background-color: rgb(30,30,30);
            color: rgb(230,230,230);
            border: none;
            border-radius: 8px;
            font-weight: 700;
            font-size: 11px;
        }
        QPushButton#PadChoiceButton:hover { background-color: rgb(36,36,36); }
        QPushButton#PadChoiceButton:checked { background-color: rgb(230,230,230); color: #000000; }
        QPushButton#PadPathChangeButton {
            background-color: rgb(30,30,30);
            color: #eaeaea;
            border: none;
            border-radius: 6px;
            font-weight: 700;
            font-size: 11px;
            padding: 0px;
            margin: 0px;
            min-width: 50px;
            max-width: 50px;
            min-height: 50px;
            max-height: 50px;
        }
        QPushButton#PadPathChangeButton:hover { background-color: rgb(36,36,36); }
        QPushButton#PadPathChangeButton:pressed { background-color: rgb(24,24,24); }
        QFrame#FloatingBarBridge { background-color: transparent; border: none; }
        QFrame#FloatingBarBridgeSegment { background-color: rgb(45,45,45); border: none; border-radius: 12px; }
        QListWidget { background-color: #232323; border: 1px solid #333333; padding: 8px; }
        QLineEdit, QTextEdit { background-color: #232323; color: #EEEEEE; border: 1px solid #3A3A3A; border-radius: 4px; }
        QPushButton { background-color: #2A2A2A; color: #EEEEEE; border: 1px solid #555555; border-radius: 8px; padding: 6px 12px; min-width: 60px; min-height: 26px; }
        QPushButton#CircleButton { background-color: #000000; color: #f5f5f5; border-radius: 30px; border: none; font-weight: 700; font-size: 18px; min-width: 60px; min-height: 60px; padding: 0px; }
        QPushButton#CircleButton:hover { border: none; }
        QPushButton#CircleButton:pressed { background-color: #0d0d0d; }
        QPushButton#CircleButton:disabled { background-color: #3a3a3a; color: #9e9e9e; }
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
    # Show preload immediately while services start
    win._show_preload_dialog("Easy Trainer 준비중...")
    QTimer.singleShot(0, win.on_start)
    return app.exec()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        # As a last resort, print to stdout (captured by launcher log)
        print(f"[FATAL] Launcher crashed: {e}", file=sys.stderr)
        raise

from __future__ import annotations

import json
import re
import os
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

from app_context import (
    APP_HOME,
    DEFAULT_PROJECT_PATH,
    FRONTEND_URL,
    SERVICE_LOG_DIR,
    UI_LOG_DIR,
    UI_LOG_FILE,
    QApplication,
    QButtonGroup,
    QCheckBox,
    QDialog,
    QEvent,
    QFont,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPlainTextEdit,
    QPoint,
    QProcess,
    QProgressBar,
    QPushButton,
    QRadioButton,
    QRegion,
    QSizePolicy,
    QStackedLayout,
    QTabWidget,
    QTextEdit,
    QTimer,
    QToolButton,
    QUrl,
    QVBoxLayout,
    QVariantAnimation,
    QWidget,
    QDesktopServices,
    QColor,
    QCursor,
    QIcon,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    Qt,
    QWebEngineView,
    load_config,
    resolve_project_root,
    save_config,
    _app_icon_path,
    _window_icon,
)
from service import ComposeServiceMixin, HealthServiceMixin, RuntimeServiceMixin, docker_compose_available
from tools import ToolingMixin
from update import UpdateManager, CONFIG_UPGRADE_KEY


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


class MainWindow(ToolingMixin, HealthServiceMixin, RuntimeServiceMixin, ComposeServiceMixin, QMainWindow):
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
        self._skip_shutdown_on_exit = False
        self._pill_expanded: bool = False
        self._pill_loading: bool = False
        self._pill_loading_ring: PillLoadingRing | None = None
        self._pad_loading_panel: QWidget | None = None
        self._pad_loading_front_label: QLabel | None = None
        self._pad_loading_back_label: QLabel | None = None
        self._pad_loading_title: QLabel | None = None
        self._pad_update_panel: QWidget | None = None
        self._pad_update_title: QLabel | None = None
        self._pad_update_header_row: QWidget | None = None
        self._pad_update_version_row: QWidget | None = None
        self._pad_update_current_label: QLabel | None = None
        self._pad_update_latest_label: QLabel | None = None
        self._pad_update_eta_label: QLabel | None = None
        self._pad_update_detail: QLabel | None = None
        self._pad_update_bar: QProgressBar | None = None
        self._pad_update_progress_label: QLabel | None = None
        self._pad_update_status_row: QWidget | None = None
        self._pad_update_front_label: QLabel | None = None
        self._pad_update_back_label: QLabel | None = None
        self._pad_update_log_view: QPlainTextEdit | None = None
        self._pad_update_primary_btn: QPushButton | None = None
        self._pad_update_secondary_btn: QPushButton | None = None
        self._pad_update_tertiary_btn: QPushButton | None = None
        self._update_panel_actions: dict[str, callable] = {}
        self._update_panel_visible = False
        self._upgrade_in_progress = False
        self._ros_domain_change_active = False
        self._ros_domain_wait_timer: QTimer | None = None
        self._update_log_lines: list[str] = []
        self._pad_status_front_label: QLabel | None = None
        self._pad_status_back_label: QLabel | None = None
        self._pad_status_front_dot: QLabel | None = None
        self._pad_status_back_dot: QLabel | None = None
        self._ros_domain_input: QLineEdit | None = None
        self._ros_domain_apply_btn: QLabel | None = None
        self._ros_domain_saved_value: int = 0
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
        self._pad_update_panel = self._create_pad_update_panel()
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
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Window)
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
        self._ros_domain_saved_value = self._sanitize_ros_domain_value(cfg.get("ros_domain_id", 0))
        os.environ["ROS_DOMAIN_ID"] = str(self._ros_domain_saved_value)
        if self._ros_domain_input is not None:
            try:
                self._ros_domain_input.blockSignals(True)
                self._ros_domain_input.setText(str(self._ros_domain_saved_value))
                self._ros_domain_input.blockSignals(False)
            except Exception:
                pass
        if self._ros_domain_apply_btn is not None:
            try:
                self._ros_domain_apply_btn.setVisible(True)
            except Exception:
                pass
        self._update_window_title()
        self.project_root = resolve_project_root(cfg)
        # Ensure project exists; if not, copy from system payload or repo
        self._ensure_project_present()
        # Unify compose service name to 'service' inside project compose files
        self._unify_compose_service()
        # Ensure docker-compose.yml matches the selected variant
        self._apply_compose_variant(self.install_variant)
        self._sync_ros_domain_compose_files(self._ros_domain_saved_value)

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

        self._update_manager = UpdateManager(self)

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

    def _is_ros_domain_editing(self) -> bool:
        widget = getattr(self, "_ros_domain_input", None)
        if widget is None:
            return False
        try:
            if not widget.isVisible():
                return False
            if widget.hasFocus():
                return True
            focus = QApplication.focusWidget()
            if focus is None:
                return False
            return focus is widget or widget.isAncestorOf(focus)
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

    def _collapse_if_needed(self):
        if getattr(self, "_pill_loading", False):
            return
        if getattr(self, "_update_panel_visible", False):
            return
        if self._is_ros_domain_editing():
            return
        if self._cursor_over_pill(strict=True):
            return
        self._animate_pill(False)

    def _sync_hover_state(self):
        if self._dragging or not self.isVisible():
            return
        if getattr(self, "_pill_loading", False):
            return
        if getattr(self, "_update_panel_visible", False):
            return
        if self._is_ros_domain_editing():
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
            self._position_update_panel()
            self._position_status_lights()
        except Exception:
            pass

    def set_display_mode(self, fullscreen: bool):
        self._fullscreen_pref = fullscreen

    def _circle_icon_paths(self) -> list[Path]:
        img_dirs = [Path(__file__).resolve().parent / "img"]
        meipass = getattr(sys, "_MEIPASS", None)
        if meipass:
            img_dirs.append(Path(meipass) / "img")
            img_dirs.append(Path(meipass) / "release" / "ui" / "img")
        for img_dir in img_dirs:
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

    def _create_pad_update_panel(self) -> QWidget:
        panel = QWidget(self._right_pad)
        panel.setObjectName("PadUpdatePanel")
        panel.setStyleSheet(
            "background-color: rgb(30,30,30); border: none;"
            "border-top-left-radius: 0px; border-bottom-left-radius: 0px;"
            "border-top-right-radius: 24px; border-bottom-right-radius: 24px;"
        )
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(45, 13, 13, 13)
        layout.setSpacing(6)
        layout.setAlignment(Qt.AlignTop | Qt.AlignHCenter)

        icon_label = QLabel(panel)
        icon_label.setAlignment(Qt.AlignCenter)
        try:
            ip = _app_icon_path()
            if ip:
                pm = QPixmap(ip)
                if not pm.isNull():
                    icon_label.setPixmap(pm.scaled(120, 120, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except Exception:
            pass

        title = QLabel("업데이트 확인 중...", panel)
        title.setObjectName("PadUpdateTitle")
        title.setAlignment(Qt.AlignCenter)

        version_row = QWidget(panel)
        version_row_layout = QHBoxLayout(version_row)
        version_row_layout.setContentsMargins(0, 0, 0, 0)
        version_row_layout.setSpacing(10)
        version_row_layout.setAlignment(Qt.AlignVCenter)
        current_label = QLabel("현재 버전: -", panel)
        current_label.setObjectName("PadUpdateVersionLabel")
        current_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        latest_label = QLabel("최신 버전: -", panel)
        latest_label.setObjectName("PadUpdateVersionLabel")
        latest_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        version_row_layout.addWidget(current_label, 1)
        version_row_layout.addWidget(latest_label, 1)
        version_row.setVisible(False)

        eta_label = QLabel("", panel)
        eta_label.setObjectName("PadUpdateEtaLabel")
        eta_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        eta_label.setVisible(False)

        header_row = QWidget(panel)
        header_layout = QHBoxLayout(header_row)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(10)
        header_layout.addWidget(version_row, 1)
        header_layout.addWidget(eta_label, 0, Qt.AlignRight | Qt.AlignVCenter)
        header_layout.setStretch(0, 1)
        header_row.setVisible(False)

        detail = QLabel("", panel)
        detail.setObjectName("PadUpdateDetail")
        detail.setAlignment(Qt.AlignCenter)
        detail.setWordWrap(True)

        log_view = QPlainTextEdit(panel)
        log_view.setObjectName("PadUpdateLog")
        log_view.setReadOnly(True)
        try:
            log_view.setLineWrapMode(QPlainTextEdit.WidgetWidth)
        except Exception:
            pass
        try:
            log_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            log_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        except Exception:
            try:
                log_view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
                log_view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            except Exception:
                pass
        try:
            line_height = log_view.fontMetrics().lineSpacing()
            log_view.setFixedHeight(int(line_height * 3 + 6))
        except Exception:
            log_view.setFixedHeight(54)
        log_view.setVisible(False)

        bar = QProgressBar(panel)
        bar.setObjectName("PadUpdateBar")
        bar.setRange(0, 0)
        bar.setTextVisible(False)
        bar.setFixedHeight(8)

        progress_label = QLabel("", panel)
        progress_label.setObjectName("PadUpdateProgressLabel")
        progress_label.setAlignment(Qt.AlignCenter)
        progress_label.setVisible(False)

        status_row_wrap = QWidget(panel)
        status_row_wrap.setVisible(False)
        status_row = QHBoxLayout(status_row_wrap)
        status_row.setContentsMargins(0, 0, 0, 0)
        status_row.setSpacing(10)
        status_front = QLabel("프론트엔드: 대기중", panel)
        status_back = QLabel("백엔드: 대기중", panel)
        for lbl in (status_front, status_back):
            lbl.setStyleSheet("color: #e0e0e0; font-size: 11px; font-weight: 700;")
            lbl.setAlignment(Qt.AlignVCenter)
        status_row.addWidget(status_front, 0, Qt.AlignLeft | Qt.AlignVCenter)
        status_row.addStretch(1)
        status_row.addWidget(status_back, 0, Qt.AlignRight | Qt.AlignVCenter)

        button_col = QVBoxLayout()
        button_col.setContentsMargins(0, 0, 0, 0)
        button_col.setSpacing(6)

        btn_primary = QPushButton("설치하기", panel)
        btn_primary.setObjectName("PadUpdatePrimaryButton")
        btn_secondary = QPushButton("해당 버전 건너뛰기", panel)
        btn_secondary.setObjectName("PadUpdateSecondaryButton")
        btn_tertiary = QPushButton("나중에 하기", panel)
        btn_tertiary.setObjectName("PadUpdateTertiaryButton")

        for btn in (btn_tertiary, btn_secondary, btn_primary):
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.setMinimumHeight(26)
            button_col.addWidget(btn)

        layout.addWidget(header_row)
        layout.addWidget(icon_label)
        layout.addWidget(title)
        layout.addWidget(detail)
        layout.addStretch(1)
        layout.addWidget(log_view)
        layout.addWidget(progress_label)
        layout.addWidget(status_row_wrap)
        layout.addWidget(bar)
        layout.addLayout(button_col)

        btn_primary.clicked.connect(lambda: self._on_update_panel_action("primary"))
        btn_secondary.clicked.connect(lambda: self._on_update_panel_action("secondary"))
        btn_tertiary.clicked.connect(lambda: self._on_update_panel_action("tertiary"))

        self._pad_update_title = title
        self._pad_update_header_row = header_row
        self._pad_update_version_row = version_row
        self._pad_update_current_label = current_label
        self._pad_update_latest_label = latest_label
        self._pad_update_eta_label = eta_label
        self._pad_update_detail = detail
        self._pad_update_bar = bar
        self._pad_update_progress_label = progress_label
        self._pad_update_status_row = status_row_wrap
        self._pad_update_front_label = status_front
        self._pad_update_back_label = status_back
        self._pad_update_log_view = log_view
        self._pad_update_primary_btn = btn_primary
        self._pad_update_secondary_btn = btn_secondary
        self._pad_update_tertiary_btn = btn_tertiary

        panel.hide()
        return panel

    def _sanitize_ros_domain_value(self, raw) -> int:
        try:
            value = int(str(raw).strip())
        except Exception:
            return 0
        if value < 0:
            return 0
        if value > 232:
            return 232
        return value

    def _sync_ros_domain_compose_files(self, value: int) -> bool:
        value = self._sanitize_ros_domain_value(value)
        if not self._is_valid_project_root(self.project_root):
            return False
        if not self._ensure_project_root_writable():
            return False
        found_any = False
        for name in ("docker-compose.yml", "docker-compose.gpu.yml", "docker-compose.cpu.yml"):
            path = self.project_root / name
            if not path.exists():
                continue
            try:
                text = path.read_text(encoding="utf-8", errors="ignore")
            except Exception:
                continue
            updated_text, count = re.subn(
                r"(?m)^(\s*-\s*ROS_DOMAIN_ID\s*=).*$",
                f"\\g<1>{value}",
                text,
            )
            if count == 0:
                updated_text, count = re.subn(
                    r"(?m)^(\s*ROS_DOMAIN_ID\s*:\s*).*$",
                    f"\\g<1>{value}",
                    text,
                )
            if count > 0:
                found_any = True
            if updated_text != text:
                try:
                    path.write_text(updated_text, encoding="utf-8")
                except Exception:
                    pass
        if not found_any:
            try:
                self.append_log("[SETTINGS][WARN] compose 파일에서 ROS_DOMAIN_ID 항목을 찾지 못했습니다.")
            except Exception:
                pass
        return found_any

    def _apply_ros_domain_value(self, value: int):
        value = self._sanitize_ros_domain_value(value)
        self._ros_domain_saved_value = value
        os.environ["ROS_DOMAIN_ID"] = str(value)
        try:
            cfg = load_config()
            cfg["ros_domain_id"] = value
            save_config(cfg)
        except Exception:
            pass
        synced = self._sync_ros_domain_compose_files(value)
        if synced:
            self.append_log(f"[SETTINGS] ROS_DOMAIN_ID={value} 저장 완료.")
        else:
            self.append_log(f"[SETTINGS] ROS_DOMAIN_ID={value} 저장 완료 (compose 적용 대기).")
        running = False
        try:
            running = "service" in self._get_running_services()
        except Exception:
            running = False
        if not running:
            return
        if not docker_compose_available() or not self._is_valid_project_root(self.project_root):
            self.append_log("[SETTINGS][WARN] compose 사용 불가: 다음 시작 때 적용됩니다.")
            return
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            self.append_log("[SETTINGS][WARN] 다른 작업 중이라 적용을 보류합니다.")
            return
        self.append_log("[SETTINGS] ROS DOMAIN 적용을 위해 서비스를 다시 시작합니다...")
        self._show_ros_domain_change_panel(value)

        def _after_restart(exit_code: int, *_):
            if exit_code != 0:
                self.append_log(f"[SETTINGS][WARN] ROS DOMAIN 적용 실패 (code={exit_code}).")
                self._hide_ros_domain_change_panel()
                return
            self.append_log("[SETTINGS] ROS DOMAIN 적용 중... 서비스 준비를 기다립니다.")
            self._wait_for_ros_domain_ready(value)

        self.run_compose(["up", "-d", "--force-recreate", "--no-deps", "service"], on_finish=_after_restart)

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
        mode2_layout.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        container = QWidget(mode2_page)
        container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
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

        right_container = QWidget(mode2_page)
        right_container.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(0, 2, 0, 2)
        right_layout.setSpacing(4)
        right_layout.setAlignment(Qt.AlignTop)
        top_row = QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)
        top_row.setSpacing(6)
        ros_label = QLabel("ROS DOMAIN", right_container)
        ros_label.setStyleSheet("color: #cfd8dc; font-size: 10px; font-weight: 700;")
        ros_input = QLineEdit(right_container)
        ros_input.setFixedWidth(32)
        ros_input.setMaxLength(3)
        ros_input.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        ros_input.setStyleSheet(
            "background-color: rgb(30,30,30); color: #e0e0e0; border: 1px solid #3a3a3a; "
            "border-radius: 6px; padding: 1px 4px; font-weight: 700; font-size: 11px;"
        )
        ros_input.setText(str(self._ros_domain_saved_value))
        top_row.addWidget(ros_label, 0, Qt.AlignLeft | Qt.AlignVCenter)
        top_row.addStretch(1)
        top_row.addWidget(ros_input, 0, Qt.AlignRight | Qt.AlignVCenter)
        right_layout.addLayout(top_row)
        right_layout.addStretch(1)
        ros_apply = QLabel("적용 및 저장", right_container)
        ros_apply.setFixedHeight(14)
        ros_apply.setAlignment(Qt.AlignCenter)
        ros_apply.setStyleSheet(
            "background-color: rgb(30,30,30); color: #e0e0e0; border: none; border-radius: 6px; "
            "font-weight: 700; font-size: 11px; padding: 1px 6px; border-radius: 4px;"
        )
        try:
            ros_apply.setCursor(Qt.PointingHandCursor)
        except Exception:
            pass
        apply_placeholder = QWidget(right_container)
        apply_placeholder.setFixedHeight(14)
        apply_stack_wrap = QWidget(right_container)
        apply_stack = QStackedLayout(apply_stack_wrap)
        apply_stack.setContentsMargins(0, 0, 0, 0)
        apply_stack.addWidget(apply_placeholder)
        apply_stack.addWidget(ros_apply)
        apply_stack.setCurrentWidget(apply_placeholder)
        right_layout.addWidget(apply_stack_wrap, 0, Qt.AlignLeft | Qt.AlignBottom)
        mode2_layout.addWidget(right_container, 0)

        def _parse_ros_domain(text: str) -> int | None:
            text = str(text).strip()
            if not text or not text.isdigit():
                return None
            try:
                value = int(text)
            except Exception:
                return None
            if value < 0 or value > 232:
                return None
            return value

        def _refresh_ros_domain_button():
            value = _parse_ros_domain(ros_input.text())
            dirty = value is not None and value != self._ros_domain_saved_value
            try:
                apply_stack.setCurrentWidget(ros_apply if dirty else apply_placeholder)
            except Exception:
                pass

        def _apply_ros_domain():
            value = _parse_ros_domain(ros_input.text())
            if value is None:
                QMessageBox.warning(self, "입력 오류", "ROS DOMAIN 값은 0~232 사이 숫자여야 합니다.")
                return
            if value == self._ros_domain_saved_value:
                ros_apply.setVisible(False)
                return
            self._apply_ros_domain_value(value)
            try:
                ros_input.blockSignals(True)
                ros_input.setText(str(value))
                ros_input.blockSignals(False)
            except Exception:
                pass
            _refresh_ros_domain_button()

        def _on_apply_mouse(event):
            try:
                if event is not None and event.button() != Qt.LeftButton:
                    return
            except Exception:
                pass
            _apply_ros_domain()
            try:
                event.accept()
            except Exception:
                pass

        ros_input.textChanged.connect(lambda *_: _refresh_ros_domain_button())
        ros_apply.mousePressEvent = _on_apply_mouse
        _refresh_ros_domain_button()

        self._pad_status_front_label = front_label
        self._pad_status_back_label = back_label
        self._pad_status_front_dot = front_dot
        self._pad_status_back_dot = back_dot
        self._ros_domain_input = ros_input
        self._ros_domain_apply_btn = ros_apply

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
        if getattr(self, "_update_panel_visible", False):
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
        self._position_update_panel()
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

    def _position_update_panel(self):
        panel = getattr(self, "_pad_update_panel", None)
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
            getattr(self, "btn_exit", None),
        ):
            if btn is None:
                continue
            try:
                btn.setEnabled(enabled)
            except Exception:
                pass
        log_btn = getattr(self, "btn_settings", None)
        if log_btn is not None:
            try:
                log_btn.setEnabled(True)
            except Exception:
                pass

    def _set_pill_loading(self, loading: bool):
        if loading == getattr(self, "_pill_loading", False):
            return
        self._pill_loading = loading
        self._set_pill_buttons_enabled(not loading)
        if loading:
            if getattr(self, "_update_panel_visible", False):
                self._set_update_panel_visible(False)
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

    def _on_update_panel_action(self, key: str):
        handler = self._update_panel_actions.get(key)
        if not handler:
            return
        try:
            self._set_update_panel_buttons_enabled(False)
        except Exception:
            pass
        try:
            handler()
        except Exception:
            pass

    def _set_update_panel_buttons_enabled(self, enabled: bool):
        for btn in (
            getattr(self, "_pad_update_primary_btn", None),
            getattr(self, "_pad_update_secondary_btn", None),
            getattr(self, "_pad_update_tertiary_btn", None),
        ):
            if btn is None:
                continue
            try:
                btn.setEnabled(enabled)
            except Exception:
                pass

    def _set_update_status_row_visible(self, visible: bool):
        row = getattr(self, "_pad_update_status_row", None)
        if row is None:
            return
        try:
            row.setVisible(visible)
        except Exception:
            pass

    def _set_update_status_row(self, frontend_ok: bool, backend_ok: bool):
        front = getattr(self, "_pad_update_front_label", None)
        back = getattr(self, "_pad_update_back_label", None)
        if front is not None:
            state = "준비완료" if frontend_ok else "대기중"
            color = "#2ecc71" if frontend_ok else "#e74c3c"
            front.setText(f"프론트엔드: {state}")
            front.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: 700;")
        if back is not None:
            state = "준비완료" if backend_ok else "대기중"
            color = "#2ecc71" if backend_ok else "#e74c3c"
            back.setText(f"백엔드: {state}")
            back.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: 700;")

    def _set_update_panel_visible(self, visible: bool):
        if visible == getattr(self, "_update_panel_visible", False):
            return
        self._update_panel_visible = visible
        if visible:
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
            self._set_pill_buttons_enabled(False)
            exit_btn = getattr(self, "btn_exit", None)
            if exit_btn is not None:
                try:
                    exit_btn.setEnabled(True)
                except Exception:
                    pass
            panel = getattr(self, "_pad_update_panel", None)
            if panel is not None:
                panel.setVisible(True)
                self._position_update_panel()
            self._ensure_main_window_visible()
        else:
            panel = getattr(self, "_pad_update_panel", None)
            if panel is not None:
                panel.setVisible(False)
            self._set_pad_boxes_visible(True)
            self._set_pill_buttons_enabled(True)
            self._pill_expanded = False
            self._apply_pill_width(0, keep_right=False)

    def _configure_update_panel(
        self,
        title: str,
        detail: str,
        show_progress: bool,
        current_version: str | None = None,
        latest_version: str | None = None,
    ):
        title_label = getattr(self, "_pad_update_title", None)
        detail_label = getattr(self, "_pad_update_detail", None)
        bar = getattr(self, "_pad_update_bar", None)
        version_row = getattr(self, "_pad_update_version_row", None)
        current_label = getattr(self, "_pad_update_current_label", None)
        latest_label = getattr(self, "_pad_update_latest_label", None)
        if title_label is not None:
            title_label.setText(title)
        if detail_label is not None:
            detail_label.setText(detail)
        if bar is not None:
            bar.setVisible(show_progress)
            if show_progress:
                bar.setRange(0, 0)
                bar.setValue(0)
                bar.setTextVisible(False)
        progress_label = getattr(self, "_pad_update_progress_label", None)
        if progress_label is not None and not show_progress:
            progress_label.setVisible(False)
        if version_row is not None:
            if current_version and latest_version:
                if current_label is not None:
                    current_label.setText(f"현재 버전: {current_version}")
                if latest_label is not None:
                    latest_label.setText(f"최신 버전: {latest_version}")
                version_row.setVisible(True)
            else:
                version_row.setVisible(False)
        self._set_update_status_row_visible(False)
        self._update_update_header_row_visibility()
        self._set_update_panel_buttons_enabled(True)

    def _set_update_progress(self, percent: int | None, text: str | None = None):
        bar = getattr(self, "_pad_update_bar", None)
        label = getattr(self, "_pad_update_progress_label", None)
        if bar is None:
            return
        if percent is None:
            bar.setRange(0, 0)
            bar.setValue(0)
            bar.setTextVisible(False)
            if label is not None:
                label.setVisible(bool(text))
                if text:
                    label.setText(text)
            return
        value = max(0, min(100, int(percent)))
        bar.setRange(0, 100)
        bar.setValue(value)
        bar.setTextVisible(False)
        if label is not None:
            label.setText(text or f"{value}%")
            label.setVisible(True)

    def _set_update_eta(self, text: str | None):
        label = getattr(self, "_pad_update_eta_label", None)
        if label is None:
            return
        if text:
            label.setText(text)
            label.setVisible(True)
        else:
            label.setVisible(False)
        self._update_update_header_row_visibility()

    def _update_update_header_row_visibility(self):
        header_row = getattr(self, "_pad_update_header_row", None)
        version_row = getattr(self, "_pad_update_version_row", None)
        eta_label = getattr(self, "_pad_update_eta_label", None)
        if header_row is None:
            return
        show = False
        try:
            show = (version_row is not None and version_row.isVisible()) or (
                eta_label is not None and eta_label.isVisible()
            )
        except Exception:
            show = False
        header_row.setVisible(show)

    def _set_update_log_visible(self, visible: bool):
        log_view = getattr(self, "_pad_update_log_view", None)
        if log_view is None:
            return
        log_view.setVisible(visible)
        if not visible:
            self._update_log_lines.clear()
            log_view.setPlainText("")

    def _append_update_log_line(self, msg: str):
        log_view = getattr(self, "_pad_update_log_view", None)
        if log_view is None or not log_view.isVisible():
            return
        for line in msg.splitlines():
            if not line:
                continue
            self._update_log_lines.append(line)
        max_lines = 3
        if len(self._update_log_lines) > max_lines:
            self._update_log_lines = self._update_log_lines[-max_lines:]
        log_view.setPlainText("\n".join(self._update_log_lines))

    def _set_update_panel_button(self, key: str, text: str | None, visible: bool, handler=None):
        btn = None
        if key == "primary":
            btn = getattr(self, "_pad_update_primary_btn", None)
        elif key == "secondary":
            btn = getattr(self, "_pad_update_secondary_btn", None)
        elif key == "tertiary":
            btn = getattr(self, "_pad_update_tertiary_btn", None)
        if btn is None:
            return
        if text is not None:
            btn.setText(text)
        btn.setVisible(visible)
        if handler:
            self._update_panel_actions[key] = handler
        else:
            self._update_panel_actions.pop(key, None)

    def show_update_check_panel(self, detail: str = "GitHub 릴리즈를 확인합니다."):
        self._update_panel_actions.clear()
        self._set_update_eta(None)
        self._configure_update_panel("업데이트 확인 중...", detail, True)
        self._set_update_log_visible(False)
        self._set_update_panel_button("primary", None, False)
        self._set_update_panel_button("secondary", None, False)
        self._set_update_panel_button("tertiary", None, False)
        self._set_update_panel_visible(True)

    def show_update_prompt_panel(
        self,
        current_version: str,
        latest_version: str,
        detail: str,
        on_install,
        on_skip,
        on_later,
    ):
        self._update_panel_actions.clear()
        self._set_update_eta(None)
        self._configure_update_panel("업데이트가 있습니다", detail, False, current_version, latest_version)
        self._set_update_log_visible(False)
        self._set_update_panel_button("primary", "설치하기", True, on_install)
        self._set_update_panel_button("secondary", "해당 버전 건너뛰기", True, on_skip)
        self._set_update_panel_button("tertiary", "나중에 하기", True, on_later)
        self._set_update_panel_visible(True)

    def show_update_progress_panel(self, detail: str):
        self._update_panel_actions.clear()
        if not getattr(self, "_upgrade_in_progress", False):
            self._set_update_eta(None)
        self._configure_update_panel("업데이트 진행 중...", detail, True)
        self._set_update_progress(None, None)
        if not getattr(self, "_upgrade_in_progress", False):
            self._set_update_log_visible(False)
        self._set_update_panel_button("primary", None, False)
        self._set_update_panel_button("secondary", None, False)
        self._set_update_panel_button("tertiary", None, False)
        self._set_update_panel_visible(True)
        self._update_update_header_row_visibility()

    def set_update_progress(self, percent: int | None, text: str | None = None):
        self._set_update_progress(percent, text)

    def show_update_done_panel(
        self,
        detail: str,
        on_close,
        current_version: str | None = None,
        latest_version: str | None = None,
        button_text: str = "닫기",
        title: str = "업데이트 완료",
    ):
        self._update_panel_actions.clear()
        self._set_update_eta(None)
        self._configure_update_panel(title, detail, False, current_version, latest_version)
        self._set_update_log_visible(False)
        self._set_update_panel_button("primary", button_text, True, on_close)
        self._set_update_panel_button("secondary", None, False)
        self._set_update_panel_button("tertiary", None, False)
        self._set_update_panel_visible(True)

    def show_update_error_panel(self, detail: str, on_continue, on_later):
        self._update_panel_actions.clear()
        if not getattr(self, "_upgrade_in_progress", False):
            self._set_update_eta(None)
        self._configure_update_panel("업데이트 실패", detail, False)
        self._set_update_panel_button("primary", "서비스 시작", True, on_continue)
        self._set_update_panel_button("secondary", None, False)
        self._set_update_panel_button("tertiary", "나중에", True, on_later)
        self._set_update_panel_visible(True)

    def hide_update_panel(self):
        self._update_panel_actions.clear()
        self._set_update_log_visible(False)
        self._set_update_eta(None)
        self._set_update_panel_visible(False)

    def _show_ros_domain_change_panel(self, value: int):
        self._ros_domain_change_active = True
        self._update_panel_actions.clear()
        self._set_update_eta(None)
        detail = f"ROS DOMAIN ID를 {value}로 적용하는 중입니다."
        self._configure_update_panel("ROS DOMAIN 변경 중...", detail, True)
        self._set_update_log_visible(False)
        self._set_update_progress(None, None)
        self._set_update_status_row_visible(True)
        self._set_update_status_row(False, False)
        self._set_update_panel_button("primary", None, False)
        self._set_update_panel_button("secondary", None, False)
        self._set_update_panel_button("tertiary", None, False)
        self._set_update_panel_visible(True)
        self._set_pill_buttons_enabled(False)
        for btn in (getattr(self, "btn_open_browser", None), getattr(self, "btn_exit", None)):
            if btn is None:
                continue
            try:
                btn.setEnabled(True)
            except Exception:
                pass

    def _stop_ros_domain_wait_timer(self):
        timer = getattr(self, "_ros_domain_wait_timer", None)
        if timer is None:
            return
        try:
            timer.stop()
        except Exception:
            pass
        self._ros_domain_wait_timer = None

    def _wait_for_ros_domain_ready(self, value: int):
        self._stop_ros_domain_wait_timer()

        def _poll():
            try:
                frontend_ok = self._is_frontend_ready(timeout=0.8)
            except Exception:
                frontend_ok = False
            try:
                backend_ok = self._check_backend_ready(timeout=0.8)
            except Exception:
                backend_ok = False
            self._set_update_status_row(frontend_ok, backend_ok)
            if frontend_ok and backend_ok:
                self._stop_ros_domain_wait_timer()
                self._hide_ros_domain_change_panel()

        timer = QTimer(self)
        timer.setInterval(1000)
        timer.timeout.connect(_poll)
        self._ros_domain_wait_timer = timer
        _poll()
        timer.start()

    def _hide_ros_domain_change_panel(self):
        self._ros_domain_change_active = False
        self._stop_ros_domain_wait_timer()
        self._set_update_status_row_visible(False)
        self.hide_update_panel()

    def start_upgrade_flow(self, version: str | None = None):
        if getattr(self, "_upgrade_in_progress", False):
            return
        self._upgrade_in_progress = True
        ver = str(version or "").strip() or "?"
        self._update_log_lines.clear()
        self._set_update_log_visible(True)
        self._set_update_eta("예상 소요: 약 30분")
        self._ensure_main_window_visible()
        self.show_update_progress_panel(f"업그레이드 준비 중... (v{ver})")
        if not docker_compose_available():
            self._show_upgrade_error("Docker 또는 Compose를 찾을 수 없습니다.")
            return
        if not self._is_valid_project_root(self.project_root):
            self._show_upgrade_error("프로젝트 경로가 아직 준비되지 않았습니다.")
            return
        if not self._apply_compose_variant(self.install_variant):
            self._show_upgrade_error("선택한 설치 옵션에 맞는 docker-compose 템플릿을 찾을 수 없습니다.")
            return

        def _after_build(exit_code: int, *_):
            if exit_code != 0:
                self._show_upgrade_error(f"업그레이드 빌드 실패 (code={exit_code})")
                return
            self.show_update_progress_panel("DB 마이그레이션 중...")
            cmd = self._upgrade_migration_cmd()
            self.run_compose(
                ["run", "--rm", "--entrypoint", "bash", "service", "-c", cmd],
                on_finish=_after_migrate,
            )

        def _after_migrate(exit_code: int, *_):
            if exit_code != 0:
                self._show_upgrade_error(f"마이그레이션 실패 (code={exit_code})")
                return
            self.show_update_progress_panel("서비스 시작 중...")
            self.run_compose(["up", "-d", "service"], on_finish=_after_up)

        def _after_up(exit_code: int, *_):
            if exit_code != 0:
                self._show_upgrade_error(f"서비스 시작 실패 (code={exit_code})")
                return
            try:
                cfg = load_config()
                if CONFIG_UPGRADE_KEY in cfg:
                    cfg.pop(CONFIG_UPGRADE_KEY, None)
                    save_config(cfg)
            except Exception:
                pass
            self._upgrade_in_progress = False
            detail = "업그레이드가 완료되었습니다.\n프로그램을 다시 실행해 주세요."
            def _close_panel():
                try:
                    self.close()
                except Exception:
                    try:
                        QApplication.instance().quit()
                    except Exception:
                        pass
            try:
                self.show_update_done_panel(
                    detail,
                    _close_panel,
                    None,
                    None,
                    "닫기",
                    "업그레이드 완료",
                )
            except Exception:
                try:
                    self.close()
                except Exception:
                    pass

        self.show_update_progress_panel("Docker 이미지를 빌드하는 중입니다...")
        self.run_compose(["build", "--no-cache"], on_finish=_after_build)

    def _show_upgrade_error(self, detail: str):
        self._upgrade_in_progress = False
        self._set_update_log_visible(True)

        def _continue():
            try:
                self.hide_update_panel()
            except Exception:
                pass
            try:
                self.on_start()
            except Exception:
                pass

        def _later():
            try:
                self.hide_update_panel()
            except Exception:
                pass

        try:
            self.show_update_error_panel(detail, _continue, _later)
        except Exception:
            pass

    def _upgrade_migration_cmd(self) -> str:
        return (
            "set -euxo pipefail; cd ~/src/backend/database; "
            "python3 -m pip show orator >/dev/null 2>&1 || ("
            "python3 -m pip install --no-deps --no-input -q "
            "backpack==0.1 simplejson faker lazy-object-proxy cleo==0.6.8 inflection "
            "pendulum==1.5.1 pytzdata python-dateutil && "
            "python3 -m pip install --no-deps --no-input -q orator==0.9.9); "
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
            "    repo.create_repository()\n"
            "  migrator = Migrator(repo, db)\n"
            "  path = os.path.join(os.getcwd(), 'migrations')\n"
            "  migrator.run(path)\n"
            "except Exception:\n"
            "  import traceback\n"
            "  traceback.print_exc()\n"
            "  sys.exit(1)\n"
            "PY\n"
        )

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
                if not self._cursor_over_pill(strict=True) and not self._is_ros_domain_editing():
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
                if not self._cursor_over_pill(strict=True) and not self._is_ros_domain_editing():
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
        if getattr(self, "_skip_shutdown_on_exit", False):
            try:
                self._close_log_windows()
                self._stop_inline_logs()
            except Exception:
                pass
            try:
                event.accept()
            except Exception:
                pass
            return
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

    def update_buttons(self):
        if getattr(self, "_pill_loading", False):
            try:
                self.btn_quick_apply.setEnabled(False)
            except Exception:
                pass
            return
        if getattr(self, "_update_panel_visible", False):
            try:
                self.btn_quick_apply.setEnabled(False)
                self.btn_quick_apply.setToolTip("업데이트 확인/진행 중에는 사용할 수 없습니다.")
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
        update_panel_visible = getattr(self, "_update_panel_visible", False)
        status_parts = []
        status_parts.append("설치됨" if installed else "미설치")
        status_parts.append("실행 중" if running else "중지됨")
        text = " · ".join(status_parts)
        try:
            self.state_label.setText(text)
        except Exception:
            pass
        try:
            self.btn_open_browser.setEnabled(bool(running) and not loading and not update_panel_visible)
        except Exception:
            pass
        if loading or update_panel_visible:
            try:
                self.btn_quick_apply.setEnabled(False)
            except Exception:
                pass
        if self.status_label.isVisible():
            self.status_label.setText(text)
        if not loading and not update_panel_visible:
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

    def auto_launch_enabled(self) -> bool:
        return bool(getattr(self, "_auto_launch_after_install", True))

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

    def _enable_restart_button(self):
        try:
            if self._preload_dialog:
                self._preload_dialog.show_restart(True, enabled=True)
        except Exception:
            pass

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
        if any(getattr(dlg, "isVisible", lambda: False)() for dlg in self._log_windows):
            self._close_log_windows()
            return
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

    def _close_devtools(self):
        if self._devtools_dialog:
            try:
                self._devtools_dialog.close()
            except Exception:
                pass
        self._devtools_view = None
        self._devtools_dialog = None

    # ------------------------ Process helpers ------------------------
    def append_log(self, msg: str):
        self.log.append(msg)
        # Persist to text file (best-effort)
        try:
            UI_LOG_DIR.mkdir(parents=True, exist_ok=True)
            with UI_LOG_FILE.open("a", encoding="utf-8") as f:
                f.write(msg + "\n")
        except Exception:
            pass
        try:
            if getattr(self, "_upgrade_in_progress", False) or (
                getattr(self, "_pad_update_log_view", None) is not None
                and getattr(self, "_pad_update_log_view").isVisible()
            ):
                self._append_update_log_line(msg)
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

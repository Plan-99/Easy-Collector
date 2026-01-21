from __future__ import annotations

import json
import os
import shutil
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
        os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = (
            f"{_chromium_flags} " if _chromium_flags else ""
        ) + _safe_flags

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
        DEFAULT_PROJECT_PATH / "VERSION",
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


def is_home_scoped(path: Path | None) -> bool:
    if not path:
        return False
    try:
        resolved = path.expanduser().resolve()
        home = Path.home().resolve()
        return resolved == home or home in resolved.parents
    except Exception:
        return False


def resolve_project_root(cfg: dict | None = None) -> Path:
    cfg = cfg or load_config()
    env_root = os.environ.get("EASYCOLLECTOR_PROJECT_ROOT", "").strip()
    cfg_root = cfg.get("project_root")
    if env_root:
        return Path(env_root).expanduser()
    if cfg_root:
        candidate = Path(cfg_root).expanduser()
        if is_home_scoped(candidate):
            candidate = DEFAULT_PROJECT_PATH
            cfg["project_root"] = str(candidate)
            save_config(cfg)
        return candidate
    candidate = DEFAULT_PROJECT_PATH
    cfg["project_root"] = str(candidate)
    save_config(cfg)
    return candidate

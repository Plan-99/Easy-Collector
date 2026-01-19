from __future__ import annotations

import os
import sys

from app_context import QApplication, QMessageBox, QTimer, _window_icon
from installer import run_setup_wizard
from launcher import MainWindow
from service import docker_compose_available


_APP_STYLE = """
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


def _apply_app_style(app: QApplication) -> None:
    app.setStyleSheet(_APP_STYLE)
    try:
        icon = _window_icon()
        if icon:
            app.setWindowIcon(icon)
    except Exception:
        pass


def main() -> int:
    os.environ.setdefault("QT_QPA_PLATFORMTHEME", "")
    os.environ.setdefault("QT_STYLE_OVERRIDE", "Fusion")
    os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
    app = QApplication(sys.argv)
    _apply_app_style(app)
    if not docker_compose_available():
        QMessageBox.critical(None, "오류", "Docker가 설치되어 있지 않거나 PATH에 없습니다.")
        return 1

    win = MainWindow()
    if not run_setup_wizard(win):
        return 0
    fullscreen = os.environ.get("EASYCOLLECTOR_FULLSCREEN", "0") == "1"
    win.set_display_mode(fullscreen)
    if not win.auto_launch_enabled():
        return 0
    win._show_preload_dialog("Easy Trainer 준비중...")
    QTimer.singleShot(0, win.on_start)
    return app.exec()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        print(f"[FATAL] Launcher crashed: {e}", file=sys.stderr)
        raise

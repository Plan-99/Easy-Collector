"""PoC: PySide6/PyQt 런처 다이얼로그 인터랙션 테스트.

추가 패키지 없이(PySide6에 기본 포함된 QtTest) 클릭/키입력을 시뮬레이션한다.
헤드리스에서 돌리려면 QT_QPA_PLATFORM=offscreen 으로 실행:

    QT_QPA_PLATFORM=offscreen python -m pytest release/ui/tests/test_credential_dialog.py -q

pytest 없이 단독 실행도 가능:
    QT_QPA_PLATFORM=offscreen python release/ui/tests/test_credential_dialog.py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# release/ui 를 import 경로에 추가 (app_context, modules, credential_dialog)
UI_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(UI_DIR))

from app_context import QApplication, Qt, QLineEdit  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402

import credential_dialog as cd  # noqa: E402


# 모든 테스트가 공유하는 단일 QApplication
_app = QApplication.instance() or QApplication(sys.argv)


SPEC = {
    "id": "hf_token",
    "title": "HuggingFace Token",
    "description": "SAM3 다운로드에 필요합니다.",
    "skippable": True,
    "input": {
        "label": "Token",
        "type": "secret",
        "validate": r"^hf_[A-Za-z0-9]+$",
        "validate_message": "hf_ 로 시작해야 합니다.",
    },
}


def test_empty_input_shows_error(monkeypatch=None):
    """빈 값으로 저장 → 에러 표시, accept 안 됨."""
    saved_calls = []
    _patch_save(monkeypatch, lambda spec, val: saved_calls.append(val) or True)

    dlg = cd.CredentialDialog(None, "sam3", SPEC)
    QTest.mouseClick(_save_button(dlg), Qt.MouseButton.LeftButton)

    # 다이얼로그를 show() 하지 않았으므로 isVisible() 대신 isHidden()/text() 확인
    assert not dlg._error_label.isHidden(), "빈 입력엔 에러가 보여야 함"
    assert dlg._error_label.text(), "에러 메시지가 채워져야 함"
    assert not dlg.was_saved
    assert saved_calls == [], "저장 함수가 호출되면 안 됨"


def test_invalid_format_blocks_save(monkeypatch=None):
    """정규식 불일치 → 저장 차단."""
    _patch_save(monkeypatch, lambda spec, val: True)
    dlg = cd.CredentialDialog(None, "sam3", SPEC)
    dlg._input.setText("not-a-token")
    QTest.mouseClick(_save_button(dlg), Qt.MouseButton.LeftButton)
    assert not dlg._error_label.isHidden()
    assert not dlg.was_saved


def test_valid_input_saves_and_accepts(monkeypatch=None):
    """유효한 토큰 입력 → save_credential 호출, was_saved True."""
    received = {}
    _patch_save(monkeypatch, lambda spec, val: received.setdefault("val", val) or True)

    dlg = cd.CredentialDialog(None, "sam3", SPEC)
    # 키 입력 시뮬레이션
    dlg._input.setFocus()
    QTest.keyClicks(dlg._input, "hf_abc123XYZ")
    QTest.mouseClick(_save_button(dlg), Qt.MouseButton.LeftButton)

    assert received.get("val") == "hf_abc123XYZ"
    assert dlg.was_saved
    assert dlg.result() == cd.QDialog.DialogCode.Accepted


def test_secret_input_is_password_echo(monkeypatch=None):
    """type=secret → 비밀번호 에코 모드."""
    dlg = cd.CredentialDialog(None, "sam3", SPEC)
    assert dlg._input.echoMode() == QLineEdit.EchoMode.Password


def test_skip_button_marks_skipped(monkeypatch=None):
    """skippable 스펙 → 스킵 버튼이 was_skipped 설정."""
    dlg = cd.CredentialDialog(None, "sam3", SPEC)
    QTest.mouseClick(_skip_button(dlg), Qt.MouseButton.LeftButton)
    assert dlg.was_skipped
    assert not dlg.was_saved


# -- helpers -----------------------------------------------------------------
def _patch_save(monkeypatch, fn):
    if monkeypatch is not None:
        monkeypatch.setattr(cd, "save_credential", fn)
    else:
        cd.save_credential = fn  # 단독 실행 경로


def _save_button(dlg):
    from app_context import QPushButton
    for btn in dlg.findChildren(QPushButton):
        if btn.isDefault():
            return btn
    # fallback: 마지막 버튼
    return dlg.findChildren(QPushButton)[-1]


def _skip_button(dlg):
    from app_context import QPushButton
    btns = dlg.findChildren(QPushButton)
    for btn in btns:
        if not btn.isDefault() and "🔗" not in btn.text():
            return btn
    raise AssertionError("skip button not found")


# -- 단독 실행 ---------------------------------------------------------------
if __name__ == "__main__":
    import traceback
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"PASS  {t.__name__}")
            passed += 1
        except Exception:
            print(f"FAIL  {t.__name__}")
            traceback.print_exc()
    print(f"\n{passed}/{len(tests)} passed")
    sys.exit(0 if passed == len(tests) else 1)

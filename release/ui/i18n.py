"""Lightweight i18n for the launcher. Mirrors frontend/src/boot/i18n.js.

config.json의 `locale` 값(없으면 ko-KR)을 읽어 t(key)로 번역 문자열을 돌려준다.
지원 로케일: ko-KR, en-US.
"""
from __future__ import annotations

from typing import Optional

from app_context import load_config, save_config

SUPPORTED_LOCALES = ["ko-KR", "en-US"]
DEFAULT_LOCALE = "ko-KR"
_CONFIG_KEY = "locale"

_current_locale: Optional[str] = None
_listeners: list = []


_TRANSLATIONS: dict[str, dict[str, str]] = {
    "ko-KR": {
        # ── 공용 ──
        "common.ok": "확인",
        "common.cancel": "취소",
        "common.close": "닫기",
        "common.copy": "복사",
        "common.copied": "복사됨",
        "common.retry": "재시도",
        "common.installing": "설치 중...",
        "common.installed": "설치됨",
        "common.notInstalled": "미설치",
        "common.failed": "실패",
        "common.install": "설치",
        "common.update": "업데이트",
        "common.remove": "제거",
        "common.search": "검색...",
        "common.language": "언어",
        # ── Pill 버튼 / 패드 라벨 ──
        "pill.codeSync": "코드 동기화",
        "pill.import": "불러오기",
        "pill.export": "내보내기",
        "pill.logs": "로그",
        "pill.modules": "모듈 관리",
        "pill.training": "학습 서버",
        "pill.exit": "서비스 종료",
        "pad.codeSync": "1. 코드 동기화",
        "pad.import": "2. 불러오기",
        "pad.export": "3. 내보내기",
        "pad.logs": "4. 로그",
        "pad.modules": "5. 모듈 관리",
        "pad.training": "6. 학습 서버",
        "pad.exit": "7. 종료",
        "pad.exitRestart": "7. 재실행",
        "pad.importChoice": "{choice} 불러오기",
        "pad.exportChoice": "{choice} 내보내기",
        # ── 모듈 관리 다이얼로그 ──
        "modules.title": "모듈 관리",
        "modules.subtitle": "설치된 모듈을 확인하고 추가/제거할 수 있습니다.",
        "modules.languageLabel": "언어",
        "modules.checkout": "결제 · {price}",
        "modules.installPaid": "설치 (구매 완료)",
        "modules.priceWon": "{amount}원",
        "modules.sizeMb": "{size:.1f} MB",
        "modules.sizeKb": "{size:.0f} KB",
        "modules.restartNeeded": "런처 재시작 필요",
        "modules.restartNeededBody": (
            "모듈이 설치되었습니다.\n"
            "변경 사항을 적용하려면 런처를 재시작해 주세요."
        ),
        # ── 카테고리 ──
        "category.core": "코어 기능 (필수)",
        "category.feature": "기능 모듈",
        "category.robot": "로봇",
        "category.sensor": "센서",
        "category.extension": "확장 기능",
        # ── 결제 다이얼로그 ──
        "checkout.title": "결제 진행",
        "checkout.message": (
            "브라우저에서 모듈 페이지가 열렸습니다.\n"
            "결제가 완료되면 자동으로 설치가 시작됩니다.\n\n"
            "창이 열리지 않았다면 아래 주소를 복사해 직접 여세요:"
        ),
        "checkout.timeout": "시간 초과",
        "checkout.timeoutBody": (
            "결제 확인 대기 시간이 만료되었습니다. "
            "결제를 완료하셨다면 모듈 창을 다시 열어주세요."
        ),
        "checkout.completed": "결제 완료",
        "checkout.completedBody": (
            "결제가 확인되었습니다. 모듈 창을 다시 열어 설치를 진행해 주세요."
        ),
        # ── 언어 설정 ──
        "language.title": "언어 설정",
        "language.label": "런처 표시 언어",
        "language.korean": "한국어",
        "language.english": "English",
        "language.applyHint": "변경 후 다이얼로그를 다시 열면 적용됩니다.",
    },
    "en-US": {
        # ── Common ──
        "common.ok": "OK",
        "common.cancel": "Cancel",
        "common.close": "Close",
        "common.copy": "Copy",
        "common.copied": "Copied",
        "common.retry": "Retry",
        "common.installing": "Installing...",
        "common.installed": "Installed",
        "common.notInstalled": "Not installed",
        "common.failed": "Failed",
        "common.install": "Install",
        "common.update": "Update",
        "common.remove": "Remove",
        "common.search": "Search...",
        "common.language": "Language",
        # ── Pill buttons / pad labels ──
        "pill.codeSync": "Code sync",
        "pill.import": "Import",
        "pill.export": "Export",
        "pill.logs": "Logs",
        "pill.modules": "Modules",
        "pill.training": "Training",
        "pill.exit": "Stop service",
        "pad.codeSync": "1. Code sync",
        "pad.import": "2. Import",
        "pad.export": "3. Export",
        "pad.logs": "4. Logs",
        "pad.modules": "5. Modules",
        "pad.training": "6. Training",
        "pad.exit": "7. Exit",
        "pad.exitRestart": "7. Restart",
        "pad.importChoice": "Import {choice}",
        "pad.exportChoice": "Export {choice}",
        # ── Module dialog ──
        "modules.title": "Modules",
        "modules.subtitle": "Browse, install, and remove modules.",
        "modules.languageLabel": "Language",
        "modules.checkout": "Checkout · {price}",
        "modules.installPaid": "Install (purchased)",
        "modules.priceWon": "₩{amount}",
        "modules.sizeMb": "{size:.1f} MB",
        "modules.sizeKb": "{size:.0f} KB",
        "modules.restartNeeded": "Restart required",
        "modules.restartNeededBody": (
            "Module installed.\n"
            "Please restart the launcher to apply changes."
        ),
        # ── Categories ──
        "category.core": "Core (required)",
        "category.feature": "Feature modules",
        "category.robot": "Robots",
        "category.sensor": "Sensors",
        "category.extension": "Extensions",
        # ── Checkout dialog ──
        "checkout.title": "Payment in progress",
        "checkout.message": (
            "The module page has opened in your browser.\n"
            "Installation will start automatically once payment is complete.\n\n"
            "If the page didn't open, copy the URL below:"
        ),
        "checkout.timeout": "Timed out",
        "checkout.timeoutBody": (
            "Payment confirmation timed out. "
            "If you completed the purchase, reopen the module dialog."
        ),
        "checkout.completed": "Payment complete",
        "checkout.completedBody": (
            "Payment confirmed. Reopen the module dialog to install."
        ),
        # ── Language settings ──
        "language.title": "Language",
        "language.label": "Launcher display language",
        "language.korean": "한국어",
        "language.english": "English",
        "language.applyHint": "Reopen the dialog to see changes applied.",
    },
}


def get_locale() -> str:
    global _current_locale
    if _current_locale is not None:
        return _current_locale
    try:
        cfg = load_config()
        loc = (cfg.get(_CONFIG_KEY) or "").strip()
    except Exception:
        loc = ""
    if loc not in SUPPORTED_LOCALES:
        loc = DEFAULT_LOCALE
    _current_locale = loc
    return loc


def set_locale(locale: str) -> None:
    """Persist locale to config.json and notify listeners."""
    global _current_locale
    if locale not in SUPPORTED_LOCALES:
        return
    if _current_locale == locale:
        return
    _current_locale = locale
    try:
        cfg = load_config()
    except Exception:
        cfg = {}
    cfg[_CONFIG_KEY] = locale
    save_config(cfg)
    for cb in list(_listeners):
        try:
            cb(locale)
        except Exception:
            pass


def on_locale_change(callback) -> None:
    """Register a callback fired when locale changes (best-effort)."""
    if callback not in _listeners:
        _listeners.append(callback)


def t(key: str, **kwargs) -> str:
    locale = get_locale()
    msg = _TRANSLATIONS.get(locale, {}).get(key)
    if msg is None:
        msg = _TRANSLATIONS.get(DEFAULT_LOCALE, {}).get(key, key)
    if kwargs:
        try:
            return msg.format(**kwargs)
        except Exception:
            return msg
    return msg

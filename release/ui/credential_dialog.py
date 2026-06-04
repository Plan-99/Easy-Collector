"""Generic credential prompt for module post-install hooks.

Reads a `credentials` spec dict (see modules.py docstring for schema), shows
the user a modal dialog with description text, clickable instruction links,
and a single input field, and persists the entered value via
`modules.save_credential`.

Designed to be reusable for any module that needs an external secret/license
key — not specific to HuggingFace / SAM 3.
"""
from __future__ import annotations

import re
from typing import Optional

from app_context import (
    Qt, QUrl, QDesktopServices,
    QDialog, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton,
    QVBoxLayout,
)

from modules import save_credential
from i18n import t


class CredentialDialog(QDialog):
    """Modal dialog for one credential spec. Returns saved/skipped status
    via `was_saved` / `was_skipped` after `exec()` completes."""

    def __init__(self, parent, module_id: str, spec: dict):
        super().__init__(parent)
        self._spec = spec
        self._module_id = module_id
        self._saved = False
        self._skipped = False

        self.setWindowTitle(spec.get("title") or t("cred.windowTitle", module_id=module_id))
        self.setModal(True)
        self.setMinimumWidth(500)

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(12)

        # Module attribution line — small + dim, so the user knows which
        # module is asking for this.
        attribution = QLabel(t("cred.moduleAttribution", module_id=module_id))
        attribution.setTextFormat(Qt.TextFormat.RichText)
        root.addWidget(attribution)

        title = QLabel(spec.get("title") or t("cred.titleFallback"))
        title_font = title.font()
        title_font.setPointSize(title_font.pointSize() + 2)
        title_font.setBold(True)
        title.setFont(title_font)
        root.addWidget(title)

        desc_text = spec.get("description") or ""
        if desc_text:
            desc = QLabel(desc_text)
            desc.setWordWrap(True)
            root.addWidget(desc)

        # Instruction links — rendered as a vertical list of clickable
        # hyperlinks. Clicking opens in the user's default browser.
        instructions = spec.get("instructions") or []
        for inst in instructions:
            if not isinstance(inst, dict):
                continue
            url = inst.get("url")
            label = inst.get("label") or url or ""
            if not url:
                continue
            row = QHBoxLayout()
            link_btn = QPushButton(f"🔗  {label}")
            link_btn.setStyleSheet(
                "QPushButton { text-align: left; padding: 6px 10px; "
                "background: #1f2937; color: #93c5fd; border: 1px solid #374151; "
                "border-radius: 6px; }"
                "QPushButton:hover { background: #374151; }"
            )
            link_btn.clicked.connect(lambda _checked=False, u=url: QDesktopServices.openUrl(QUrl(u)))
            row.addWidget(link_btn)
            row.addStretch(1)
            root.addLayout(row)

        # Input field
        input_cfg = spec.get("input") or {}
        input_label_text = input_cfg.get("label") or t("cred.inputLabelFallback")
        input_label = QLabel(input_label_text)
        root.addWidget(input_label)

        self._input = QLineEdit()
        if input_cfg.get("type") == "secret":
            self._input.setEchoMode(QLineEdit.EchoMode.Password)
        placeholder = input_cfg.get("placeholder")
        if placeholder:
            self._input.setPlaceholderText(placeholder)
        root.addWidget(self._input)

        self._error_label = QLabel("")
        self._error_label.setStyleSheet("color: #ef4444;")
        self._error_label.setWordWrap(True)
        self._error_label.hide()
        root.addWidget(self._error_label)

        # Buttons
        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        if spec.get("skippable", False):
            skip_btn = QPushButton(spec.get("skip_label") or t("cred.skipLater"))
            skip_btn.clicked.connect(self._on_skip)
            btn_row.addWidget(skip_btn)
        save_btn = QPushButton(spec.get("save_label") or t("cred.save"))
        save_btn.setDefault(True)
        save_btn.clicked.connect(self._on_save)
        btn_row.addWidget(save_btn)
        root.addLayout(btn_row)

    # -- result accessors ----------------------------------------------------
    @property
    def was_saved(self) -> bool:
        return self._saved

    @property
    def was_skipped(self) -> bool:
        return self._skipped

    # -- handlers ------------------------------------------------------------
    def _on_skip(self) -> None:
        self._skipped = True
        self.accept()

    def _on_save(self) -> None:
        value = self._input.text().strip()
        if not value:
            self._show_error(t("cred.errEmpty"))
            return
        validate = (self._spec.get("input") or {}).get("validate")
        if validate:
            try:
                if not re.match(validate, value):
                    err = (self._spec.get("input") or {}).get(
                        "validate_message",
                        t("cred.errInvalidFormat"),
                    )
                    self._show_error(err)
                    return
            except re.error:
                pass  # bad regex in spec — don't block the user
        if not save_credential(self._spec, value):
            self._show_error(t("cred.errSaveFailed"))
            return
        self._saved = True
        self.accept()

    def _show_error(self, msg: str) -> None:
        self._error_label.setText(msg)
        self._error_label.show()


def prompt_credentials(parent, module_id: str, specs: list[dict]) -> dict:
    """Show one dialog per spec. Returns {cred_id: 'saved'|'skipped'|'cancelled'}.

    Stops the chain if the user closes a dialog (cancelled). Skippable specs
    record 'skipped' and continue. Non-skippable + cancelled → 'cancelled'."""
    results: dict[str, str] = {}
    for spec in specs:
        cred_id = spec.get("id") or "credential"
        dlg = CredentialDialog(parent, module_id, spec)
        dlg.exec()
        if dlg.was_saved:
            results[cred_id] = "saved"
        elif dlg.was_skipped:
            results[cred_id] = "skipped"
        else:
            results[cred_id] = "cancelled"
            if not spec.get("skippable", False):
                # Stop further prompts — user explicitly bailed on a required
                # credential; no point pestering them about the rest.
                break
        # Friendly hint after save (non-blocking).
        if results[cred_id] == "saved":
            try:
                QMessageBox.information(
                    parent,
                    t("cred.savedTitle"),
                    t("cred.savedBody", name=spec.get("title") or cred_id),
                )
            except Exception:
                pass
    return results

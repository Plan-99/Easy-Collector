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
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPixmap,
    QProcess,
    QProgressBar,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QTextEdit,
    QTimer,
    QVBoxLayout,
    QWidget,
    Qt,
    QApplication,
    _app_icon_path,
)
from modules import (
    MODULE_REGISTRY,
    CATEGORY_LABELS,
    VISIBLE_CATEGORIES,
    modules_by_category,
    save_installed_modules,
    get_installed_modules,
    install_modules_batch,
    detect_gpus,
    set_training_mode,
    set_training_server_installed,
    _REGISTRY_MAP,
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
    dlg.resize(1200, 780)

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
    for s in ["소프트웨어 사용권 계약", "설치 준비", "모듈 선택", "학습 서버", "설치 중", "완료"]:
        QListWidgetItem(s, steps)

    # Pages inside a bordered content box
    container = QFrame(); container.setFrameShape(QFrame.Box); container.setFrameShadow(QFrame.Plain); container.setObjectName("ContentBox")
    container_layout = QVBoxLayout(); container.setLayout(container_layout)

    page_eula = QWidget(); v1b = QVBoxLayout()
    v1b.addWidget(QLabel("소프트웨어 사용권 계약을 확인해 주세요."))
    eula_text = QTextEdit()
    eula_text.setReadOnly(True)
    eula_text.setPlainText(
        "Easy Trainer 소프트웨어 사용권 계약 (EULA)\n"
        "\n"
        "최종 업데이트: 2025년 4월\n"
        "\n"
        "본 최종 사용자 사용권 계약(이하 \"본 계약\")은 Vertical Labs(이하 \"회사\")와 본 소프트웨어를 "
        "설치하거나 사용하는 개인 또는 법인(이하 \"사용자\") 사이에 체결되는 법적 구속력이 있는 계약입니다. "
        "본 소프트웨어를 설치, 복사 또는 사용하는 경우 본 계약의 모든 조건에 동의한 것으로 간주됩니다. "
        "동의하지 않으시면 설치를 중단하고 본 소프트웨어를 삭제하십시오.\n"
        "\n"
        "제1조 (정의)\n"
        "1. \"소프트웨어\"란 Easy Trainer 프로그램 및 관련 문서, 업데이트, 패치를 포함한 일체의 파일을 의미합니다.\n"
        "2. \"라이선스 키\"란 회사가 사용자에게 발급하는 고유한 인증 코드를 의미합니다.\n"
        "3. \"기기\"란 소프트웨어가 설치되어 실행되는 하나의 물리적 컴퓨터를 의미합니다.\n"
        "\n"
        "제2조 (사용권 부여)\n"
        "1. 회사는 사용자에게 본 계약 조건에 따라 소프트웨어를 사용할 수 있는 비독점적, 양도 불가능한 사용권을 부여합니다.\n"
        "2. 하나의 라이선스 키는 1대의 기기에만 등록하여 사용할 수 있습니다.\n"
        "3. 사용권은 회사가 정한 요금제(Free, Business)에 따라 기능 범위가 달라질 수 있습니다.\n"
        "\n"
        "제3조 (사용 제한)\n"
        "사용자는 다음 행위를 해서는 안 됩니다.\n"
        "1. 소프트웨어를 역컴파일, 디컴파일, 역어셈블 또는 기타 방법으로 소스 코드를 추출하는 행위\n"
        "2. 소프트웨어를 복제, 수정, 배포, 재판매, 대여 또는 2차 라이선스하는 행위\n"
        "3. 라이선스 키를 제3자와 공유하거나 공개적으로 게시하는 행위\n"
        "4. 소프트웨어의 보안 기능, 인증 메커니즘 또는 사용 제한을 우회하는 행위\n"
        "5. 소프트웨어를 불법적이거나 본 계약에서 허용하지 않는 목적으로 사용하는 행위\n"
        "\n"
        "제4조 (지적재산권)\n"
        "1. 소프트웨어에 대한 저작권 및 기타 지적재산권은 회사에 귀속됩니다.\n"
        "2. 본 계약은 소프트웨어에 대한 소유권을 이전하는 것이 아니며, 사용권만을 부여합니다.\n"
        "3. 소프트웨어에 포함된 상표, 로고, 브랜드명에 대한 권리는 회사에 있습니다.\n"
        "\n"
        "제5조 (개인정보 수집 및 이용)\n"
        "1. 회사는 라이선스 인증 및 서비스 제공을 위해 다음 정보를 수집할 수 있습니다.\n"
        "   - 이메일 주소 (Google OAuth 로그인 시)\n"
        "   - 기기 고유 식별자 (라이선스 바인딩용)\n"
        "   - 소프트웨어 사용 현황 (버전, 접속 시간)\n"
        "2. 수집된 정보는 서비스 제공 및 개선 목적으로만 사용되며, 제3자에게 판매하지 않습니다.\n"
        "3. 사용자는 회사에 개인정보 삭제를 요청할 수 있으며, 이 경우 사용권이 종료될 수 있습니다.\n"
        "\n"
        "제6조 (면책 및 보증의 제한)\n"
        "1. 소프트웨어는 \"있는 그대로(AS IS)\" 제공되며, 명시적이든 묵시적이든 어떠한 종류의 보증도 하지 않습니다.\n"
        "2. 회사는 소프트웨어의 사용 또는 사용 불능으로 인해 발생하는 직접적, 간접적, 부수적, 특별, 결과적 손해에 대해 책임지지 않습니다.\n"
        "3. 로봇 및 하드웨어 제어 과정에서 발생하는 물리적 손해, 장비 파손, 인명 피해에 대해 회사는 일체의 책임을 부담하지 않습니다.\n"
        "4. 사용자는 소프트웨어를 사용하기 전에 충분한 안전 조치를 취할 책임이 있습니다.\n"
        "\n"
        "제7조 (계약의 해지)\n"
        "1. 사용자가 본 계약의 조건을 위반하는 경우, 회사는 별도 통지 없이 사용권을 해지할 수 있습니다.\n"
        "2. 계약이 해지된 경우 사용자는 소프트웨어의 모든 사본을 즉시 삭제해야 합니다.\n"
        "3. 사용자는 언제든지 소프트웨어를 삭제하여 본 계약을 종료할 수 있습니다.\n"
        "\n"
        "제8조 (업데이트 및 변경)\n"
        "1. 회사는 소프트웨어의 기능을 개선하기 위해 업데이트를 제공할 수 있습니다.\n"
        "2. 회사는 본 계약의 내용을 합리적인 범위 내에서 변경할 수 있으며, 변경 시 소프트웨어 내 또는 웹사이트를 통해 고지합니다.\n"
        "3. 변경된 계약에 동의하지 않는 경우, 사용자는 소프트웨어 사용을 중단할 수 있습니다.\n"
        "\n"
        "제9조 (준거법 및 분쟁 해결)\n"
        "1. 본 계약은 대한민국 법률에 따라 해석됩니다.\n"
        "2. 본 계약과 관련된 분쟁은 서울중앙지방법원을 제1심 관할 법원으로 합니다.\n"
        "\n"
        "제10조 (기타)\n"
        "1. 본 계약의 어느 조항이 무효 또는 집행 불능인 경우에도 나머지 조항의 효력에는 영향을 미치지 않습니다.\n"
        "2. 본 계약에 명시되지 않은 사항은 관련 법령 및 상관례에 따릅니다.\n"
        "\n"
        "Copyright (c) 2025 Vertical Labs. All rights reserved.\n"
    )
    v1b.addWidget(eula_text, 1)
    eula_agree_row = QHBoxLayout()
    chk_eula_agree = QCheckBox("동의함")
    eula_agree_row.addWidget(chk_eula_agree)
    eula_agree_row.addStretch(1)
    v1b.addLayout(eula_agree_row)
    page_eula.setLayout(v1b)

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

    # ── Module selection page ──
    page_modules = QWidget()
    v_mod = QVBoxLayout()
    v_mod.addWidget(QLabel("설치할 모듈을 선택하세요."))
    v_mod.addSpacing(6)

    module_checkboxes: dict[str, QCheckBox] = {}
    by_cat = modules_by_category()
    # Auto-include core/feature (hidden)
    for cat in ("core", "feature"):
        for mod in by_cat.get(cat, []):
            chk = QCheckBox(mod.name)
            chk.setChecked(True)
            module_checkboxes[mod.id] = chk

    installed = get_installed_modules()
    columns_row = QHBoxLayout()
    columns_row.setSpacing(12)

    def _make_filter(search_input, widget_list):
        def _do_filter():
            query = search_input.text().strip().lower()
            for widget, key in widget_list:
                widget.setVisible(not query or query in key)
        search_input.textChanged.connect(_do_filter)

    for cat in VISIBLE_CATEGORIES:
        mods = by_cat.get(cat, [])
        if not mods:
            continue
        col = QVBoxLayout()
        col.setSpacing(4)
        cat_label = QLabel(f"■ {CATEGORY_LABELS.get(cat, cat)}")
        cat_label.setStyleSheet("font-weight: bold; font-size: 13px;")
        col.addWidget(cat_label)

        search_input = QLineEdit()
        search_input.setPlaceholderText("검색...")
        search_input.setFixedHeight(28)
        search_input.setStyleSheet("font-size: 11px; padding: 2px 8px;")
        col.addWidget(search_input)

        sorted_mods = sorted(mods, key=lambda m: (0 if m.id in installed else 1, m.name))
        widget_list: list[tuple[QWidget, str]] = []

        for mod in sorted_mods:
            row_widget = QWidget()
            row = QHBoxLayout(row_widget)
            row.setContentsMargins(4, 2, 4, 2)
            row.setSpacing(6)
            chk = QCheckBox(mod.name)
            chk.setChecked(mod.id in installed)
            module_checkboxes[mod.id] = chk
            row.addWidget(chk)
            row.addStretch(1)
            col.addWidget(row_widget)
            search_key = f"{mod.name} {mod.description} {mod.id}".lower()
            widget_list.append((row_widget, search_key))

        _make_filter(search_input, widget_list)
        col.addStretch(1)
        if columns_row.count() > 0:
            sep = QFrame()
            sep.setFrameShape(QFrame.VLine)
            sep.setStyleSheet("color: #444;")
            columns_row.addWidget(sep)
        columns_row.addLayout(col, 1)

    v_mod.addLayout(columns_row, 1)
    page_modules.setLayout(v_mod)

    # ── Training server page ──
    page_training = QWidget()
    v_train = QVBoxLayout()
    v_train.addWidget(QLabel("학습 서버 설정"))
    v_train.addSpacing(6)

    # GPU info display
    gpu_info_label = QLabel("GPU 정보를 확인하는 중...")
    gpu_info_label.setStyleSheet("font-size: 12px; color: #ccc;")
    gpu_info_label.setWordWrap(True)
    v_train.addWidget(gpu_info_label)
    v_train.addSpacing(10)

    def _refresh_gpu_info():
        gpus = detect_gpus()
        if not gpus:
            gpu_info_label.setText("⚠ NVIDIA GPU가 감지되지 않았습니다.")
            gpu_info_label.setStyleSheet("font-size: 12px; color: #ff9800;")
        else:
            lines = []
            for g in gpus:
                lines.append(
                    f"🖥 GPU {g.index}: {g.name}  —  "
                    f"VRAM {g.vram_total_mb} MB (사용: {g.vram_used_mb} MB / 여유: {g.vram_free_mb} MB)"
                )
            gpu_info_label.setText("\n".join(lines))
            gpu_info_label.setStyleSheet("font-size: 12px; color: #86efac;")

    training_choice = {"value": "remote"}

    rb_local = QRadioButton("로컬 학습 서버 설치 (이 PC에서 학습)")
    rb_local.setStyleSheet("font-size: 12px;")
    rb_remote = QRadioButton("원격 학습 서버 사용 (별도 서버에서 학습)")
    rb_remote.setStyleSheet("font-size: 12px;")
    rb_remote.setChecked(True)

    local_desc = QLabel("    이 PC에 학습 서버 Docker 이미지를 설치합니다. GPU VRAM 8GB 이상 권장.")
    local_desc.setStyleSheet("color: #999; font-size: 11px;")
    local_desc.setWordWrap(True)
    remote_desc = QLabel("    별도 GPU 서버에서 학습을 실행합니다. 이 PC에는 학습 서버를 설치하지 않습니다.")
    remote_desc.setStyleSheet("color: #999; font-size: 11px;")
    remote_desc.setWordWrap(True)

    v_train.addWidget(rb_local)
    v_train.addWidget(local_desc)
    v_train.addSpacing(6)
    v_train.addWidget(rb_remote)
    v_train.addWidget(remote_desc)

    def _update_training_choice():
        if rb_local.isChecked():
            training_choice["value"] = "local"
        else:
            training_choice["value"] = "remote"
    rb_local.toggled.connect(lambda _: _update_training_choice())
    rb_remote.toggled.connect(lambda _: _update_training_choice())

    v_train.addStretch(1)
    page_training.setLayout(v_train)

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

    pages = [page_eula, page_prepare, page_modules, page_training, page_install, page_done]

    def set_page(idx: int):
        steps.setCurrentRow(idx)
        for i in reversed(range(container_layout.count())):
            w = container_layout.itemAt(i).widget()
            if w is not None:
                w.setParent(None)
        container_layout.addWidget(pages[idx])
        # Pages: 0=EULA, 1=준비, 2=모듈, 3=학습서버, 4=설치중, 5=완료
        btn_prev.setVisible(idx in (1, 2, 3))
        btn_prev.setEnabled(idx in (1, 2, 3))
        btn_next.setVisible(idx in (0, 1, 4))
        btn_install.setVisible(idx == 3)
        btn_finish.setVisible(idx == 5)
        if idx == 0:
            # EULA
            btn_next.setEnabled(chk_eula_agree.isChecked())
        elif idx == 1:
            # 설치 준비
            try:
                target = self._disk_usage_target()
                usage = shutil.disk_usage(str(target))
                lbl_space.setText(f"현재 여유 공간: {usage.free / (1024**3):.1f} GB")
                has_space = usage.free >= 20 * 1024**3
            except Exception:
                lbl_space.setText("현재 여유 공간: 확인 불가")
                has_space = True
            btn_next.setEnabled(has_space and docker_compose_available())
        elif idx == 2:
            # 모듈 선택 — next goes to 학습서버
            btn_next.setVisible(True)
            btn_next.setEnabled(True)
        elif idx == 3:
            # 학습 서버 — install button
            _refresh_gpu_info()
            btn_install.setEnabled(True)
        elif idx == 4:
            # 설치 중
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
        if current in (0, 1, 2):
            current += 1; set_page(current)
        elif current == 4:
            # 설치중 → 완료
            current = 5; set_page(current)
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

    def _update_eula_next():
        if current == 0:
            btn_next.setEnabled(chk_eula_agree.isChecked())

    chk_eula_agree.toggled.connect(_update_eula_next)

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
        # Auto-detect GPU
        variant = "gpu" if self._has_host_nvidia_driver() else "cpu"
        self._set_install_variant(variant)
        if not self._apply_compose_variant(variant):
            QMessageBox.critical(dlg, "오류", "선택한 설치 옵션에 맞는 docker-compose 템플릿을 찾을 수 없습니다.")
            return
        # Save selected modules to config
        selected = [mid for mid, chk in module_checkboxes.items() if chk.isChecked()]
        save_installed_modules(selected)
        # Save training server choice. Local mode just flips a flag; the backend
        # entrypoint reads EASYTRAINER_LOCAL_TRAINING and spawns training_server
        # in-container, sharing torch/CUDA with the API process.
        mode = training_choice["value"]
        set_training_mode(mode)
        try:
            import training_server_install as _ts
            _ts.set_local_training_enabled(mode == "local")
        except Exception:
            set_training_server_installed(mode == "local")
        nonlocal current
        current = 4; set_page(current)
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
        svc = "backend"

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

        # Step 1: run DB migrations (peewee create_tables)
        log.append(f"[POST] Running DB migrations in one-off container for '{svc}' ...")
        migrate_cmd = (
            "cd /root && python3 -c \""
            "import sys; sys.path.insert(0, '/root'); "
            "from backend.database.models import create_tables; "
            "create_tables(); "
            "print('[DB] Migration complete')"
            "\""
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
                        running = "backend" in self._get_running_services()
                    except Exception:
                        running = False
                    if not running:
                        QMessageBox.critical(dlg, "오류", f"서비스 시작 실패 (code={exit_code})")
                        return
                    log.append(f"[POST][WARN] 서비스 시작 exit_code={exit_code}, 하지만 컨테이너가 실행 중입니다.")
                # Step 3: Install selected modules from GitHub
                # Filter to only visible (installable) modules that user checked
                installable = [mid for mid, chk in module_checkboxes.items()
                               if chk.isChecked() and mid in _REGISTRY_MAP and not _REGISTRY_MAP[mid].required]
                if installable:
                    log.append(f"[POST] 선택한 모듈 {len(installable)}개 설치 중...")
                    def _on_mod_start(mid, i, total):
                        mod = _REGISTRY_MAP.get(mid)
                        name = mod.name if mod else mid
                        log.append(f"[MODULE] ({i+1}/{total}) {name} 다운로드 중...")
                    def _on_mod_done(mid, ok, i, total):
                        mod = _REGISTRY_MAP.get(mid)
                        name = mod.name if mod else mid
                        if ok:
                            log.append(f"[MODULE] ✓ {name} 설치 완료")
                        else:
                            log.append(f"[MODULE] ✗ {name} 설치 실패")
                    try:
                        install_modules_batch(installable, on_module_start=_on_mod_start, on_module_done=_on_mod_done)
                    except Exception as e:
                        log.append(f"[MODULE][WARN] 모듈 설치 중 오류: {e}")
                    log.append("[POST] 모듈 설치 완료")

                # Step 4: download backend/training_server source if user opted in.
                def _finalize():
                    _mark_progress_complete()
                    try:
                        MARKER_FILE.write_text("ok")
                    except Exception:
                        pass
                    btn_next.setEnabled(True)

                if training_choice["value"] == "local":
                    log.append("[POST] 로컬 학습 서버 다운로드 중...")
                    import threading
                    import training_server_install as _ts

                    def _ts_log(msg: str):
                        QTimer.singleShot(0, lambda m=msg: log.append(f"[TRAIN-SERVER] {m}"))

                    def _ts_worker():
                        try:
                            ok, msg = _ts.download_from_release(on_log=_ts_log)
                        except Exception as e:
                            ok, msg = False, str(e)
                        QTimer.singleShot(0, lambda: log.append(
                            f"[TRAIN-SERVER] {'완료' if ok else '실패'}: {msg}"
                        ))
                        QTimer.singleShot(0, _finalize)

                    threading.Thread(target=_ts_worker, daemon=True).start()
                else:
                    _finalize()
            self.run_compose_to_widget(["up", "-d"], log, on_finish=_after_up)

        # Run migration using docker run (bypasses compose deploy GPU requirements)
        def _run_docker_migrate():
            proc = QProcess(dlg)
            proc.setProgram("docker")
            proc.setArguments([
                "run", "--rm",
                "-v", f"{self.project_root}/backend:/root/backend",
                "-v", "/opt/easytrainer:/opt/easytrainer",
                "--entrypoint", "bash",
                "easytrainer-backend:latest",
                "-c", migrate_cmd,
            ])
            proc.setWorkingDirectory(str(self.project_root))
            def _r(is_err=False):
                try:
                    data = proc.readAllStandardError() if is_err else proc.readAllStandardOutput()
                    text = bytes(data).decode(errors="ignore")
                    if text:
                        log.append(text.rstrip("\n"))
                except Exception:
                    pass
            proc.readyReadStandardOutput.connect(lambda: _r(False))
            proc.readyReadStandardError.connect(lambda: _r(True))
            proc.finished.connect(_after_migrate)
            proc.start()
        _run_docker_migrate()

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
                if current == 5:
                    _record_launch_choice()
                ev.accept()
            else:
                ev.ignore()
        except Exception:
            ev.ignore()

    def _on_reject():
        if _confirm_exit():
            if current == 5:
                _record_launch_choice()
            QDialog.reject(dlg)

    dlg.closeEvent = _on_close
    dlg.reject = _on_reject

    def _center_dialog():
        try:
            screen = dlg.screen() or QApplication.primaryScreen()
            if not screen:
                return
            geo = screen.availableGeometry()
            x = int(geo.x() + (geo.width() - dlg.width()) / 2)
            y = int(geo.y() + (geo.height() - dlg.height()) / 2)
            dlg.move(x, y)
        except Exception:
            pass

    # Center the dialog when it first appears.
    _center_dialog()
    QTimer.singleShot(0, _center_dialog)

    # Exec modal; abort app if not completed
    result = dlg.exec()
    return result == QDialog.Accepted and self.is_installed()

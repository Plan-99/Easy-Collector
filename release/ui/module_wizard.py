"""
Local module wizard.

사용자가 직접 만든 ROS 패키지 / vendor SDK를 EasyTrainer에 등록하기 위한 PyQt 다이얼로그.
- 작업 디렉터리: ~/.easytrainer/local_modules/<id>/
- 코드 편집은 외부 에디터(xdg-open)로 위임
- 검증: schema + (SDK) controller.py 임포트/서브클래스 체크
- 설치: 기존 마켓 모듈과 동일한 파이프라인(tar.gz → _install_robot_sensor_module)을 통해 진행
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import tarfile
import tempfile
import textwrap
import traceback
from pathlib import Path
from typing import Optional

# 대부분의 위젯은 launcher가 PySide6/PyQt6 폴백을 처리해 둔 app_context에서 가져온다.
from app_context import (
    Qt, QFont,
    QCheckBox, QComboBox, QDialog, QFileDialog, QFrame, QHBoxLayout,
    QInputDialog, QLabel, QLineEdit, QListWidget, QMessageBox,
    QPlainTextEdit, QPushButton, QScrollArea, QVBoxLayout, QWidget,
)
# app_context에 노출되지 않은 심볼은 동일한 PySide6 → PyQt6 폴백 패턴.
try:
    from PySide6.QtCore import QThread, Signal as _Signal
    from PySide6.QtWidgets import QSpinBox, QFormLayout, QGroupBox, QTabWidget
except Exception:
    from PyQt6.QtCore import QThread, pyqtSignal as _Signal
    from PyQt6.QtWidgets import QSpinBox, QFormLayout, QGroupBox, QTabWidget


LOCAL_MODULES_DIR = Path.home() / ".easytrainer" / "local_modules"
ROS2_CONTAINER = "easytrainer_ros2"

# 컨테이너 안에서 ros2_ws/src 가 마운트되는 경로
CONTAINER_ROS2_WS_SRC = "/root/ros2_ws/src"


# ---------------------------------------------------------------------------
# Schema validation
# ---------------------------------------------------------------------------

_ID_RE = re.compile(r"^[a-z][a-z0-9_]{1,40}$")


def validate_module_meta(meta: dict) -> list[str]:
    """module.json 딕셔너리를 검증해 에러 목록을 반환한다. 빈 리스트면 통과."""
    errs: list[str] = []
    mid = meta.get("id", "")
    if not _ID_RE.match(mid):
        errs.append("id는 소문자/숫자/언더스코어만 가능 (예: robot_myarm)")

    if not meta.get("name"):
        errs.append("name 비어 있음")
    if not meta.get("version"):
        errs.append("version 비어 있음")

    cat = meta.get("category", "")
    if cat not in ("robot", "sensor"):
        errs.append(f"category는 robot 또는 sensor여야 함 (현재: {cat})")

    install = meta.get("install", {})
    if not isinstance(install, dict) or not install:
        errs.append("install 섹션 비어 있음")

    if cat == "robot":
        robots = meta.get("robots", [])
        if not robots:
            errs.append("robot 모듈은 robots[] 항목이 최소 1개 필요")
        seen_types: set[str] = set()
        for i, r in enumerate(robots):
            rtype = r.get("type", "")
            if not rtype:
                errs.append(f"robots[{i}].type 비어 있음")
            elif rtype == "custom":
                # frontend가 'custom' type을 외부 ROS 토픽 직접입력 로봇으로 분기하기 때문에
                # 사용자 모듈 type으로는 사용할 수 없다.
                errs.append(f"robots[{i}].type 으로 'custom'은 예약어라 사용할 수 없습니다")
            elif rtype in seen_types:
                errs.append(f"robots[{i}].type 중복: {rtype}")
            seen_types.add(rtype)

            role = r.get("spec", {}).get("role", "")
            if role not in ("single_arm", "tool", "dual_arm"):
                errs.append(f"robots[{i}].spec.role 은 single_arm/tool/dual_arm 중 하나여야 함 (현재: {role})")
            ee_defs = r.get("ik", {}).get("ee_definitions", [])
            if role == "dual_arm":
                names = {e.get("name") for e in ee_defs}
                if not {"L_ee", "R_ee"}.issubset(names):
                    errs.append(f"robots[{i}].ik.ee_definitions 에 L_ee 와 R_ee 가 모두 정의돼야 함 (dual_arm)")

            spec = r.get("spec", {})
            jdim = spec.get("joint_dim")
            jnames = spec.get("joint_names", [])
            if not isinstance(jdim, int) or jdim <= 0:
                errs.append(f"robots[{i}].spec.joint_dim 필요 (양의 정수)")
            if len(jnames) != jdim:
                errs.append(f"robots[{i}].spec.joint_names 개수({len(jnames)})와 joint_dim({jdim}) 불일치")
            lb = spec.get("joint_lower_bounds", [])
            ub = spec.get("joint_upper_bounds", [])
            if len(lb) != jdim or len(ub) != jdim:
                errs.append(f"robots[{i}] joint_lower_bounds/upper_bounds 개수 불일치")
            driver = r.get("driver", {})
            kind = driver.get("kind")
            if kind not in ("topic", "service", "sdk"):
                errs.append(f"robots[{i}].driver.kind는 topic/service/sdk 중 하나")
            if kind == "sdk" and not driver.get("sdk_type"):
                errs.append(f"robots[{i}].driver.sdk_type 필요")
    return errs


# ---------------------------------------------------------------------------
# Templates
# ---------------------------------------------------------------------------

_CONTROLLER_TEMPLATE = '''"""
{name} SDK Controller.
EasyTrainer가 BaseSDKController를 상속한 클래스를 임포트하여 사용한다.
"""
try:
    from base import BaseSDKController  # noqa: F401  (sdk_controllers/base.py가 sys.path에 추가됨)
except ImportError:
    # dispatcher가 BaseSDKController를 모듈 글로벌에 주입하므로 import 없이도 동작.
    BaseSDKController = BaseSDKController  # type: ignore[name-defined]

# 디스패치가 sdk_type을 매칭할 때 참조한다 (생략 시 모듈 폴더명 사용).
SDK_TYPE = "{sdk_type}"


class {class_name}(BaseSDKController):
    """{name} 로봇 SDK 직접 제어."""

    def __init__(self, config: dict):
        # config: module.json + 사용자가 추가한 custom_fields(예: ip_address, can_port)
        self._config = config
        self._connected = False
        # TODO: self._client = vendor_sdk.Client(...)

    def connect(self) -> bool:
        # TODO: SDK 연결 초기화
        self._connected = True
        return True

    def enable(self) -> bool:
        # TODO: 모터 enable
        return self._connected

    def write_joints(self, positions: list, names=None) -> None:
        # TODO: vendor SDK joint command 호출 (라디안 단위 입력)
        pass

    def read_joints(self) -> tuple:
        # TODO: vendor SDK 상태 읽기. (names, positions) 반환 (라디안 단위)
        names = self._config.get("joint_names", [])
        return names, [0.0] * len(names)

    def disconnect(self) -> None:
        self._connected = False
'''


_DESC_PACKAGE_XML = '''<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.0.1</version>
  <description>Description package for {module_id} (auto-generated by EasyTrainer wizard).</description>
  <maintainer email="local@easytrainer">local</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
'''

_DESC_CMAKELISTS = '''cmake_minimum_required(VERSION 3.8)
project({name})
find_package(ament_cmake REQUIRED)
ament_package()
'''


def _to_class_name(module_id: str) -> str:
    parts = re.split(r"[_\-]", module_id)
    return "".join(p.capitalize() for p in parts if p) + "SDKController"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _csv_to_floats(s: str) -> list[float]:
    return [float(c.strip()) for c in s.split(",") if c.strip()]


def _is_float(s: str) -> bool:
    s = s.strip()
    if not s:
        return False
    try:
        float(s)
        return True
    except ValueError:
        return False


def _csv_non_floats(s: str) -> list[str]:
    """CSV 문자열에서 숫자로 파싱 불가능한 토큰들을 반환. 빈 입력이면 [] (선택 입력 허용)."""
    bad: list[str] = []
    for c in s.split(","):
        c = c.strip()
        if c and not _is_float(c):
            bad.append(c)
    return bad


def _req(text: str) -> str:
    """필수 필드 라벨에 빨간 별표를 붙인 HTML. QFormLayout.addRow(str, widget)는
    QLabel 을 만들고 rich text 자동 인식하므로 그대로 사용 가능."""
    return f'{text} <span style="color:#c0392b;font-weight:bold;">*</span>'


# ROS 2 msg/srv 타입 표기 패턴: pkg/Name, pkg/msg/Name, pkg/srv/Name, pkg/action/Name
_MSG_TYPE_RE = re.compile(r"^[a-z][a-z0-9_]*/(?:(?:msg|srv|action)/)?[A-Z][A-Za-z0-9_]*$")


def _looks_like_msg_type(s: str) -> bool:
    return bool(_MSG_TYPE_RE.match(s.strip()))


# 콤보 기본 옵션 (편집 가능 — vendor msg/srv 도 직접 입력 가능)
_COMMON_READ_MSGS = [
    "sensor_msgs/JointState",
    "sensor_msgs/msg/JointState",
    "trajectory_msgs/JointTrajectory",
    "trajectory_msgs/msg/JointTrajectory",
    "std_msgs/Float64MultiArray",
]
_COMMON_WRITE_MSGS = [
    "sensor_msgs/JointState",
    "sensor_msgs/msg/JointState",
    "trajectory_msgs/JointTrajectory",
    "trajectory_msgs/msg/JointTrajectory",
    "std_msgs/Float64MultiArray",
    "std_srvs/srv/Trigger",
]


def _csv_to_ints(s: str) -> list[int]:
    return [int(c.strip()) for c in s.split(",") if c.strip()]


def _csv_to_strs(s: str) -> list[str]:
    return [c.strip() for c in s.split(",") if c.strip()]


def _list_to_csv(values: list) -> str:
    return ", ".join(str(v) for v in values)


def open_in_external(path: Path) -> None:
    """xdg-open으로 파일/폴더를 시스템 기본 앱에서 연다."""
    try:
        subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def _container_path_from_workdir_file(workdir: Path, module_id: str, abs_file: Path) -> Optional[str]:
    """`<workdir>/ros2/src/<pkg>/<rel>` → `/root/ros2_ws/src/<id>/<pkg>/<rel>` 매핑.

    설치 시 `<workdir>/ros2/src/<pkg>/`가 `ros2_ws/src/<id>/<pkg>/`로 복사되므로 그 매핑을 그대로 따른다.
    workdir 밖이거나 ros2/src 트리 밖이면 None.
    """
    try:
        rel = abs_file.resolve().relative_to((workdir / "ros2" / "src").resolve())
    except (ValueError, RuntimeError):
        return None
    return f"{CONTAINER_ROS2_WS_SRC}/{module_id}/{rel.as_posix()}"


def parse_urdf_joints(urdf_path: Path) -> list[tuple[str, float, float]]:
    """URDF 파일을 파싱해 controlled joint 들의 (name, lower, upper) 리스트를 반환.

    - type=fixed 와 mimic 자식이 있는 joint 는 제외.
    - revolute/prismatic 은 <limit lower=.. upper=..> 에서 한계를 읽음.
    - continuous 는 한계가 없으므로 ±2π 로 디폴트.
    - .xacro 파일은 매크로 미확장이라 일부 joint 가 누락될 수 있음 (best-effort).
    실패 시 RuntimeError.
    """
    import math
    import xml.etree.ElementTree as ET

    try:
        tree = ET.parse(str(urdf_path))
    except ET.ParseError as e:
        raise RuntimeError(f"URDF XML 파싱 실패: {e}")
    except OSError as e:
        raise RuntimeError(f"URDF 파일 열기 실패: {e}")

    root = tree.getroot()
    out: list[tuple[str, float, float]] = []
    for j in root.iter("joint"):
        # robot/joint 직속만 고려 (transmission/joint 같은 다른 컨텍스트 제외)
        # ET.iter는 모든 깊이의 joint를 잡으니 parent 가 transmission 이면 제외하기 어려움.
        # 다행히 transmission/joint 는 <hardwareInterface> 등 다른 자식이고, 이름이 robot/joint 와
        # 중복되므로 dedup 으로 처리.
        jname = j.get("name", "").strip()
        if not jname or "$" in jname or "{" in jname:
            continue
        jtype = j.get("type", "")
        if jtype == "fixed":
            continue
        if j.find("mimic") is not None:
            continue
        if any(jname == prev[0] for prev in out):
            continue  # transmission/joint 의 중복 방지

        limit = j.find("limit")
        if limit is not None:
            try:
                lb = float(limit.get("lower", "0"))
                ub = float(limit.get("upper", "0"))
            except ValueError:
                lb, ub = 0.0, 0.0
        elif jtype == "continuous":
            lb, ub = -2 * math.pi, 2 * math.pi
        else:
            lb, ub = 0.0, 0.0
        out.append((jname, lb, ub))
    return out


def detect_ros_packages(folder: Path) -> list[Path]:
    """`folder` 안의 ROS 2 패키지 목록을 반환한다.

    지원 레이아웃:
    1. 단일 패키지: folder/package.xml → [folder]
    2. 표준 워크스페이스: folder/src/<pkg>/package.xml (피퍼 형태)
    3. 평면 워크스페이스: folder/<pkg>/package.xml

    어떤 것도 없으면 [] 반환.
    """
    if not folder.is_dir():
        return []
    if (folder / "package.xml").is_file():
        return [folder]
    src = folder / "src"
    if src.is_dir():
        pkgs = [c for c in src.iterdir() if c.is_dir() and (c / "package.xml").is_file()]
        if pkgs:
            return pkgs
    pkgs = [c for c in folder.iterdir() if c.is_dir() and (c / "package.xml").is_file()]
    return pkgs


# ---------------------------------------------------------------------------
# Launch arg row — one per `key:=value` ros2 launch argument
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Pre-launch script row — driver 시작 전 실행할 bash 스크립트
# ---------------------------------------------------------------------------

class _PreLaunchScriptRow(QWidget):
    """driver.pre_launch 의 type=script 항목 한 개.
    path 는 모듈 루트 기준 상대 (root 콤보로 ros2/sdk 선택)."""

    def __init__(self, on_delete, path: str = "", root: str = "ros2", wait_after: float = 1.0):
        super().__init__()
        self._on_delete = on_delete
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(6)

        self.path_edit = QLineEdit(path)
        self.path_edit.setPlaceholderText("스크립트 path (예: can_activate_main.sh)")
        self.root_combo = QComboBox()
        self.root_combo.addItems(["ros2", "sdk"])
        self.root_combo.setCurrentText(root if root in ("ros2", "sdk") else "ros2")
        self.root_combo.setMaximumWidth(70)
        self.wait_edit = QLineEdit(str(wait_after))
        self.wait_edit.setPlaceholderText("wait_after (s)")
        self.wait_edit.setMaximumWidth(80)
        self.btn_del = QPushButton("✕")
        self.btn_del.setMaximumWidth(28)
        self.btn_del.clicked.connect(lambda: self._on_delete(self))

        layout.addWidget(QLabel("path"))
        layout.addWidget(self.path_edit, 1)
        layout.addWidget(QLabel("root"))
        layout.addWidget(self.root_combo)
        layout.addWidget(QLabel("wait"))
        layout.addWidget(self.wait_edit)
        layout.addWidget(self.btn_del)

    def to_dict(self) -> dict:
        try:
            wa = float(self.wait_edit.text().strip() or 1.0)
        except ValueError:
            wa = 1.0
        return {
            "type": "script",
            "path": self.path_edit.text().strip(),
            "root": self.root_combo.currentText(),
            "wait_after": wa,
        }


# ---------------------------------------------------------------------------
# Post-launch ROS service row — driver 시작 후 호출할 ROS 서비스
# ---------------------------------------------------------------------------

class _PostLaunchServiceRow(QWidget):
    """driver.post_launch 의 type=ros_service 항목 한 개.
    request 는 JSON 텍스트로 입력."""

    def __init__(self, on_delete, service: str = "", service_type: str = "",
                 request: dict | None = None, wait_before: float = 0.0, timeout: float = 5.0):
        super().__init__()
        self._on_delete = on_delete
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(2)

        row1 = QHBoxLayout()
        row1.setSpacing(6)
        self.service_edit = QLineEdit(service)
        self.service_edit.setPlaceholderText("service (예: /jaka_driver/servo_move_enable)")
        self.service_type_edit = QLineEdit(service_type)
        self.service_type_edit.setPlaceholderText("service_type (예: jaka_msgs/srv/ServoMoveEnable)")
        self.btn_del = QPushButton("✕")
        self.btn_del.setMaximumWidth(28)
        self.btn_del.clicked.connect(lambda: self._on_delete(self))
        row1.addWidget(QLabel("service"))
        row1.addWidget(self.service_edit, 2)
        row1.addWidget(self.service_type_edit, 2)
        row1.addWidget(self.btn_del)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.setSpacing(6)
        self.request_edit = QLineEdit(json.dumps(request or {}, ensure_ascii=False))
        self.request_edit.setPlaceholderText('request (JSON, 예: {"enable": true})')
        self.wait_before_edit = QLineEdit(str(wait_before))
        self.wait_before_edit.setMaximumWidth(80)
        self.wait_before_edit.setPlaceholderText("wait_before")
        self.timeout_edit = QLineEdit(str(timeout))
        self.timeout_edit.setMaximumWidth(80)
        self.timeout_edit.setPlaceholderText("timeout")
        row2.addWidget(QLabel("request"))
        row2.addWidget(self.request_edit, 2)
        row2.addWidget(QLabel("wait_before"))
        row2.addWidget(self.wait_before_edit)
        row2.addWidget(QLabel("timeout"))
        row2.addWidget(self.timeout_edit)
        layout.addLayout(row2)

    def to_dict(self) -> dict | None:
        svc = self.service_edit.text().strip()
        stype = self.service_type_edit.text().strip()
        if not svc or not stype:
            return None
        try:
            req = json.loads(self.request_edit.text() or "{}")
            if not isinstance(req, dict):
                req = {}
        except Exception:
            req = {}
        try:
            wb = float(self.wait_before_edit.text().strip() or 0.0)
        except ValueError:
            wb = 0.0
        try:
            to = float(self.timeout_edit.text().strip() or 5.0)
        except ValueError:
            to = 5.0
        return {
            "type": "ros_service",
            "service": svc,
            "service_type": stype,
            "request": req,
            "wait_before": wb,
            "timeout": to,
        }


# ---------------------------------------------------------------------------
# Launch arg row — `key:=value`
# ---------------------------------------------------------------------------

class _LaunchArgRow(QWidget):
    """ros2 launch 의 `key:=value` 한 쌍."""

    def __init__(self, on_delete, key: str = "", value: str = ""):
        super().__init__()
        self._on_delete = on_delete
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(6)

        self.key_edit = QLineEdit(key)
        self.key_edit.setPlaceholderText("key (예: robot_ip)")
        self.value_edit = QLineEdit(value)
        self.value_edit.setPlaceholderText("value (placeholder 가능: {ip_address}, {robot_id}, {namespace})")
        self.btn_del = QPushButton("✕")
        self.btn_del.setMaximumWidth(28)
        self.btn_del.setToolTip("이 launch arg 삭제")
        self.btn_del.clicked.connect(lambda: self._on_delete(self))

        layout.addWidget(self.key_edit, 1)
        layout.addWidget(QLabel(":="))
        layout.addWidget(self.value_edit, 2)
        layout.addWidget(self.btn_del)

    def to_pair(self) -> tuple[str, str]:
        return (self.key_edit.text().strip(), self.value_edit.text().strip())


# ---------------------------------------------------------------------------
# Joint row — one per joint inside a variant
# ---------------------------------------------------------------------------

class _JointRow(QWidget):
    """단일 joint의 name / lower bound / upper bound / is_tool 입력 한 줄."""

    def __init__(self, on_delete, name: str = "", lb: float = -3.14, ub: float = 3.14, is_tool: bool = False):
        super().__init__()
        self._on_delete = on_delete
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(6)

        self.name_edit = QLineEdit(name)
        self.name_edit.setPlaceholderText("joint name")

        self.lb_edit = QLineEdit(f"{lb:g}")
        self.lb_edit.setMaximumWidth(80)
        self.lb_edit.setToolTip("lower bound (rad)")

        self.ub_edit = QLineEdit(f"{ub:g}")
        self.ub_edit.setMaximumWidth(80)
        self.ub_edit.setToolTip("upper bound (rad)")

        self.is_tool_chk = QCheckBox("is_tool")
        self.is_tool_chk.setChecked(is_tool)
        self.is_tool_chk.setToolTip("이 joint가 그리퍼/툴이면 체크 (tool_index에 포함)")

        self.btn_del = QPushButton("✕")
        self.btn_del.setMaximumWidth(28)
        self.btn_del.setToolTip("이 joint 삭제")
        self.btn_del.clicked.connect(lambda: self._on_delete(self))

        layout.addWidget(self.name_edit, 2)
        layout.addWidget(QLabel("lb"))
        layout.addWidget(self.lb_edit)
        layout.addWidget(QLabel("ub"))
        layout.addWidget(self.ub_edit)
        layout.addWidget(self.is_tool_chk)
        layout.addWidget(self.btn_del)

    def to_tuple(self) -> tuple[str, float, float, bool]:
        try:
            lb = float(self.lb_edit.text().strip())
        except ValueError:
            lb = 0.0
        try:
            ub = float(self.ub_edit.text().strip())
        except ValueError:
            ub = 0.0
        return (self.name_edit.text().strip(), lb, ub, self.is_tool_chk.isChecked())


# ---------------------------------------------------------------------------
# Variant widget — one per robot variant tab
# ---------------------------------------------------------------------------

class _VariantWidget(QWidget):
    """단일 로봇 변형의 spec/ik/driver 입력. ROS와 SDK 두 모드를 모두 갖고 있고
    `set_method()`로 보이는 입력만 토글한다."""

    def __init__(self, wizard: "ModuleWizard", default_type: str = ""):
        super().__init__()
        self._wizard = wizard
        # ROS 모드에서 URDF 가 속한 패키지 폴더명 (워크스페이스 지원).
        # SDK 모드에선 None.
        self._urdf_pkg_name: Optional[str] = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)

        # Variant type
        head = QFormLayout()
        self.type_edit = QLineEdit(default_type)
        self.type_edit.setPlaceholderText("예: tm_12, tm_12_robotiq")
        self.type_edit.textChanged.connect(self._on_type_changed)
        head.addRow(_req("type"), self.type_edit)
        layout.addLayout(head)

        # spec
        spec_box = QGroupBox("spec")
        spec_outer = QVBoxLayout(spec_box)

        # 상단 폼: role / company / IK / custom_fields
        spec_form = QFormLayout()
        self.role_combo = QComboBox()
        # frontend가 분기하는 값은 single_arm / tool / dual_arm 셋뿐이다.
        self.role_combo.addItems(["single_arm", "tool", "dual_arm"])
        self.role_combo.setCurrentText("single_arm")
        self.role_combo.currentTextChanged.connect(self._on_role_changed)
        self.company_edit = QLineEdit()
        self.ik_chk = QCheckBox("IK 사용")
        self.ik_chk.setChecked(True)
        self.custom_fields_edit = QLineEdit()
        self.custom_fields_edit.setPlaceholderText("예: ip_address, can_port")
        spec_form.addRow("role", self.role_combo)
        spec_form.addRow("company", self.company_edit)
        spec_form.addRow(self.ik_chk)
        spec_form.addRow("custom_fields", self.custom_fields_edit)
        spec_outer.addLayout(spec_form)
        layout.addWidget(spec_box)

        # IK (role=tool 일 때는 숨김 — 그리퍼/툴은 IK 안 씀)
        self.ik_box = QGroupBox("IK")
        ik_box = self.ik_box
        ik_layout = QVBoxLayout(ik_box)

        ik_hint = QLabel(
            "ROS 모드: 'description 패키지'(URDF 가 들어있는 sub-package)를 선택한 뒤\n"
            "'📂 URDF 선택' 으로 .urdf 파일을 고릅니다. urdf_path 는 그 패키지 기준 상대.\n"
            "SDK 모드: 위에서 'description 폴더'를 선택해두고 그 안에서 URDF 를 고릅니다."
        )
        ik_hint.setStyleSheet("color: #666; font-size: 11px;")
        ik_hint.setWordWrap(True)
        ik_layout.addWidget(ik_hint)

        # URDF 패키지 (ROS 모드 전용) — workspace 안의 어느 sub-package 가 URDF 를 들고 있는지
        self.urdf_pkg_row = QHBoxLayout()
        self.urdf_pkg_row.addWidget(QLabel("description 패키지"))
        self.urdf_pkg_combo = QComboBox()
        self.urdf_pkg_combo.setMinimumWidth(200)
        self.urdf_pkg_combo.setToolTip(
            "ROS 패키지 폴더 안에서 URDF 가 들어있는 sub-package 를 고르세요.\n"
            "(워크스페이스 형태일 때 어느 패키지인지 명시하기 위해 필요)"
        )
        # 콤보 선택 즉시 _urdf_pkg_name 갱신 → 설치 시 manifest 의 urdf_package_dir 반영
        self.urdf_pkg_combo.currentTextChanged.connect(self._on_urdf_pkg_changed)
        btn_refresh_urdf_pkg = QPushButton("🔄")
        btn_refresh_urdf_pkg.setMaximumWidth(34)
        btn_refresh_urdf_pkg.setToolTip("위에서 고른 ROS 패키지 폴더를 다시 스캔")
        btn_refresh_urdf_pkg.clicked.connect(self.refresh_urdf_pkg_combo)
        self.urdf_pkg_row.addWidget(self.urdf_pkg_combo, 1)
        self.urdf_pkg_row.addWidget(btn_refresh_urdf_pkg)
        ik_layout.addLayout(self.urdf_pkg_row)

        urdf_row = QHBoxLayout()
        self.btn_pick_urdf = QPushButton("📂 URDF 선택")
        self.btn_pick_urdf.setToolTip("선택된 description 패키지 안의 .urdf 파일을 골라 상대 경로를 자동 입력합니다.")
        self.btn_pick_urdf.clicked.connect(self._on_pick_urdf)
        self.btn_clear_urdf = QPushButton("지우기")
        self.btn_clear_urdf.setMaximumWidth(60)
        self.btn_clear_urdf.clicked.connect(self._on_clear_urdf)
        urdf_row.addWidget(self.btn_pick_urdf)
        urdf_row.addWidget(self.btn_clear_urdf)
        urdf_row.addStretch()
        ik_layout.addLayout(urdf_row)

        ik_form = QFormLayout()
        self.urdf_path_edit = QLineEdit()
        self.urdf_path_edit.setPlaceholderText("(URDF 선택 후 자동 입력 — 패키지 기준 상대 경로)")
        self.urdf_path_edit.setReadOnly(True)
        self.urdf_path_edit.setStyleSheet("background: #f5f5f5; color: #444;")
        self.joints_lock_edit = QLineEdit()
        ik_form.addRow("urdf_path", self.urdf_path_edit)
        ik_form.addRow("joints_to_lock (CSV)", self.joints_lock_edit)
        ik_layout.addLayout(ik_form)

        # ee 입력 — single_arm/tool 용 (한 행)
        self.ee_form = QFormLayout()
        self.ee_parent_edit = QLineEdit()
        self.ee_parent_edit.setPlaceholderText("예: joint_6")
        self.ee_offset_edit = QLineEdit()
        self.ee_offset_edit.setPlaceholderText("예: 0.0, 0.0, 0.15  (없으면 비워둠)")
        self.ee_form.addRow("ee parent joint", self.ee_parent_edit)
        self.ee_form.addRow("ee offset (xyz)", self.ee_offset_edit)
        # 컨테이너 위젯으로 감싸야 setVisible로 토글 가능
        self.ee_single_box = QWidget()
        self.ee_single_box.setLayout(self.ee_form)
        ik_layout.addWidget(self.ee_single_box)

        # dual_arm 용 (L_ee / R_ee 두 행)
        self.ee_dual_box = QWidget()
        ee_dual_form = QFormLayout(self.ee_dual_box)
        self.l_ee_parent_edit = QLineEdit()
        self.l_ee_parent_edit.setPlaceholderText("왼쪽 팔의 마지막 joint 이름")
        self.l_ee_offset_edit = QLineEdit()
        self.l_ee_offset_edit.setPlaceholderText("예: 0.0, 0.0, 0.15  (없으면 비워둠)")
        self.r_ee_parent_edit = QLineEdit()
        self.r_ee_parent_edit.setPlaceholderText("오른쪽 팔의 마지막 joint 이름")
        self.r_ee_offset_edit = QLineEdit()
        self.r_ee_offset_edit.setPlaceholderText("예: 0.0, 0.0, 0.15  (없으면 비워둠)")
        ee_dual_form.addRow("L_ee parent joint", self.l_ee_parent_edit)
        ee_dual_form.addRow("L_ee offset (xyz)", self.l_ee_offset_edit)
        ee_dual_form.addRow("R_ee parent joint", self.r_ee_parent_edit)
        ee_dual_form.addRow("R_ee offset (xyz)", self.r_ee_offset_edit)
        ik_layout.addWidget(self.ee_dual_box)

        layout.addWidget(ik_box)

        # joints — IK(URDF 선택) 다음에 와야 URDF 추출 결과로 자동 채워지는 흐름이 자연스러움
        joints_box = QGroupBox("joints (joint_dim 은 행 개수에서 자동 산출 — URDF 선택 시 자동 입력)")
        joints_outer = QVBoxLayout(joints_box)
        self._joints: list[_JointRow] = []
        self._joints_layout = QVBoxLayout()
        self._joints_layout.setContentsMargins(0, 0, 0, 0)
        self._joints_layout.setSpacing(2)
        joints_container = QWidget()
        joints_container.setLayout(self._joints_layout)
        joints_outer.addWidget(joints_container)

        add_joint_btn = QPushButton("+ joint 추가")
        add_joint_btn.clicked.connect(lambda: self._add_joint())
        add_joint_row = QHBoxLayout()
        add_joint_row.addWidget(add_joint_btn)
        add_joint_row.addStretch()
        joints_outer.addLayout(add_joint_row)

        # 기본 6개 joint (URDF 픽커가 동작하면 즉시 교체됨)
        for i in range(6):
            self._add_joint(name=f"joint_{i + 1}", lb=-3.14, ub=3.14)

        layout.addWidget(joints_box)

        # ROS driver
        self.ros_box = QGroupBox("ROS 드라이버")
        ros_outer = QVBoxLayout(self.ros_box)

        ros_form = QFormLayout()
        self.driver_kind_combo = QComboBox()
        self.driver_kind_combo.addItems(["topic", "service"])
        self.read_topic_edit = QLineEdit("/joint_states")
        self.read_msg_edit = QComboBox()
        self.read_msg_edit.setEditable(True)
        self.read_msg_edit.addItems(_COMMON_READ_MSGS)
        self.read_msg_edit.setCurrentText("sensor_msgs/JointState")
        self.write_topic_edit = QLineEdit()
        self.write_msg_edit = QComboBox()
        self.write_msg_edit.setEditable(True)
        self.write_msg_edit.addItems(_COMMON_WRITE_MSGS)
        self.write_msg_edit.setCurrentText("")
        ros_form.addRow(_req("kind"), self.driver_kind_combo)
        ros_form.addRow(_req("read_topic"), self.read_topic_edit)
        ros_form.addRow(_req("read_topic_msg"), self.read_msg_edit)
        ros_form.addRow(_req("write_topic"), self.write_topic_edit)
        ros_form.addRow(_req("write_topic_msg"), self.write_msg_edit)
        ros_outer.addLayout(ros_form)

        # ros2 launch 명령 (이 변형의 ROS 드라이버 노드를 띄울 때 사용)
        ros_outer.addWidget(QLabel("ros2 launch 명령 (이 변형의 ROS 드라이버 노드 기동에 사용)"))
        launch_form = QFormLayout()
        self.launch_pkg_edit = QLineEdit()
        self.launch_pkg_edit.setPlaceholderText("예: ur_bringup, tm_driver, jaka_driver")
        self.launch_file_edit = QLineEdit()
        self.launch_file_edit.setPlaceholderText("예: ur_control.launch.py, robot.launch.py")
        launch_form.addRow(_req("launch package"), self.launch_pkg_edit)
        launch_form.addRow(_req("launch file"), self.launch_file_edit)
        ros_outer.addLayout(launch_form)

        # launch args — 동적 key:=value 행
        ros_outer.addWidget(QLabel("launch args (key:=value, 값에 {ip_address}/{robot_id}/{namespace} 등 placeholder 사용 가능)"))
        self._launch_args: list[_LaunchArgRow] = []
        self._launch_args_layout = QVBoxLayout()
        self._launch_args_layout.setContentsMargins(0, 0, 0, 0)
        self._launch_args_layout.setSpacing(2)
        launch_args_holder = QWidget()
        launch_args_holder.setLayout(self._launch_args_layout)
        ros_outer.addWidget(launch_args_holder)

        add_arg_btn = QPushButton("+ launch arg 추가")
        add_arg_btn.clicked.connect(lambda: self._add_launch_arg())
        arg_btn_row = QHBoxLayout()
        arg_btn_row.addWidget(add_arg_btn)
        arg_btn_row.addStretch()
        ros_outer.addLayout(arg_btn_row)

        # 흔히 쓰는 namespace arg 를 기본 1줄 미리 채워둠
        self._add_launch_arg(key="namespace", value="{namespace}")

        layout.addWidget(self.ros_box)

        # SDK driver
        self.sdk_box = QGroupBox("SDK 드라이버")
        sdk_form = QFormLayout(self.sdk_box)
        self.sdk_type_edit = QLineEdit()
        self.sdk_type_edit.setPlaceholderText("vendor 식별자 (예: myarm). 비우면 module id 사용")
        sdk_form.addRow("sdk_type", self.sdk_type_edit)
        layout.addWidget(self.sdk_box)

        # Pre/post launch hooks (선택)
        hooks_box = QGroupBox("driver 훅 (선택) — driver 시작 전후 자동 실행")
        hooks_v = QVBoxLayout(hooks_box)
        hooks_hint = QLabel(
            "pre_launch: driver 시작 직전 bash 스크립트 실행 (예: piper CAN bring-up).\n"
            "    path 는 모듈 루트 기준 상대 (root=ros2: /root/ros2_ws/src/<id>/<path>, root=sdk: /root/robot_sdk/<id>/<path>)\n"
            "post_launch: driver 시작 직후 ROS 서비스 호출 (예: JAKA servo_move_enable)."
        )
        hooks_hint.setStyleSheet("color: #666; font-size: 11px;")
        hooks_hint.setWordWrap(True)
        hooks_v.addWidget(hooks_hint)

        # pre_launch list
        hooks_v.addWidget(QLabel("pre_launch (script)"))
        self._pre_hooks: list[_PreLaunchScriptRow] = []
        self._pre_hooks_layout = QVBoxLayout()
        self._pre_hooks_layout.setContentsMargins(0, 0, 0, 0)
        self._pre_hooks_layout.setSpacing(2)
        pre_holder = QWidget()
        pre_holder.setLayout(self._pre_hooks_layout)
        hooks_v.addWidget(pre_holder)
        btn_add_pre = QPushButton("+ pre_launch script 추가")
        btn_add_pre.clicked.connect(lambda: self._add_pre_hook())
        pre_btn_row = QHBoxLayout()
        pre_btn_row.addWidget(btn_add_pre)
        pre_btn_row.addStretch()
        hooks_v.addLayout(pre_btn_row)

        # post_launch list
        hooks_v.addWidget(QLabel("post_launch (ros_service)"))
        self._post_hooks: list[_PostLaunchServiceRow] = []
        self._post_hooks_layout = QVBoxLayout()
        self._post_hooks_layout.setContentsMargins(0, 0, 0, 0)
        self._post_hooks_layout.setSpacing(2)
        post_holder = QWidget()
        post_holder.setLayout(self._post_hooks_layout)
        hooks_v.addWidget(post_holder)
        btn_add_post = QPushButton("+ post_launch service 추가")
        btn_add_post.clicked.connect(lambda: self._add_post_hook())
        post_btn_row = QHBoxLayout()
        post_btn_row.addWidget(btn_add_post)
        post_btn_row.addStretch()
        hooks_v.addLayout(post_btn_row)

        layout.addWidget(hooks_box)

        layout.addStretch()

        # 초기 토글 상태 적용 (single_arm)
        self._on_role_changed(self.role_combo.currentText())

    # ---- callbacks ----
    def _on_type_changed(self, txt: str) -> None:
        # 부모 탭 라벨 갱신
        self._wizard._refresh_variant_tab_titles()

    def _on_role_changed(self, role: str) -> None:
        is_dual = (role == "dual_arm")
        is_tool = (role == "tool")
        # tool 은 IK 자체가 의미 없음 → 섹션 통째로 숨김
        self.ik_box.setVisible(not is_tool)
        # dual 이면 ee 두 줄, 아니면 한 줄 (tool 때는 어차피 ik_box 가 숨김)
        self.ee_single_box.setVisible(not is_dual)
        self.ee_dual_box.setVisible(is_dual)
        # ik_chk 도 tool 모드에서 자동으로 끔 (이미 꺼져있으면 그대로)
        if is_tool:
            self.ik_chk.setChecked(False)

    # ---- joints ----
    def _add_joint(self, name: str = "", lb: float = -3.14, ub: float = 3.14, is_tool: bool = False) -> None:
        row = _JointRow(self._remove_joint, name=name, lb=lb, ub=ub, is_tool=is_tool)
        self._joints.append(row)
        self._joints_layout.addWidget(row)

    def _remove_joint(self, row: _JointRow) -> None:
        if len(self._joints) <= 1:
            QMessageBox.information(self, "삭제 불가", "최소 1개의 joint는 필요합니다.")
            return
        try:
            self._joints.remove(row)
        except ValueError:
            return
        self._joints_layout.removeWidget(row)
        row.deleteLater()

    # ---- launch args ----
    def _add_launch_arg(self, key: str = "", value: str = "") -> None:
        row = _LaunchArgRow(self._remove_launch_arg, key=key, value=value)
        self._launch_args.append(row)
        self._launch_args_layout.addWidget(row)

    def _remove_launch_arg(self, row: _LaunchArgRow) -> None:
        try:
            self._launch_args.remove(row)
        except ValueError:
            return
        self._launch_args_layout.removeWidget(row)
        row.deleteLater()

    # ---- pre/post launch hooks ----
    def _add_pre_hook(self, path: str = "", root: str = "ros2", wait_after: float = 1.0) -> None:
        row = _PreLaunchScriptRow(self._remove_pre_hook, path=path, root=root, wait_after=wait_after)
        self._pre_hooks.append(row)
        self._pre_hooks_layout.addWidget(row)

    def _remove_pre_hook(self, row: _PreLaunchScriptRow) -> None:
        try:
            self._pre_hooks.remove(row)
        except ValueError:
            return
        self._pre_hooks_layout.removeWidget(row)
        row.deleteLater()

    def _add_post_hook(self, service: str = "", service_type: str = "",
                       request: dict | None = None, wait_before: float = 0.0,
                       timeout: float = 5.0) -> None:
        row = _PostLaunchServiceRow(self._remove_post_hook, service=service,
                                     service_type=service_type, request=request,
                                     wait_before=wait_before, timeout=timeout)
        self._post_hooks.append(row)
        self._post_hooks_layout.addWidget(row)

    def _remove_post_hook(self, row: _PostLaunchServiceRow) -> None:
        try:
            self._post_hooks.remove(row)
        except ValueError:
            return
        self._post_hooks_layout.removeWidget(row)
        row.deleteLater()

    def refresh_urdf_pkg_combo(self) -> None:
        """ROS 패키지 폴더 안의 sub-package 목록으로 콤보 갱신. 선택 유지."""
        keep = self.urdf_pkg_combo.currentText()
        self.urdf_pkg_combo.blockSignals(True)
        self.urdf_pkg_combo.clear()
        ros_pkg = self._wizard._ros_pkg_src
        if ros_pkg is not None:
            pkgs = detect_ros_packages(ros_pkg)
            for p in pkgs:
                self.urdf_pkg_combo.addItem(p.name)
        if keep:
            i = self.urdf_pkg_combo.findText(keep)
            if i >= 0:
                self.urdf_pkg_combo.setCurrentIndex(i)
        self.urdf_pkg_combo.blockSignals(False)
        # 갱신 후 현재 선택을 _urdf_pkg_name 에 반영
        self._on_urdf_pkg_changed(self.urdf_pkg_combo.currentText())

    def _on_urdf_pkg_changed(self, name: str) -> None:
        """description 패키지 콤보 선택 즉시 변형의 _urdf_pkg_name 갱신.
        URDF 파일을 picker 로 다시 안 골라도 설치 시점의 urdf_package_dir 가 이 값에 맞춰진다.
        패키지가 바뀌면 기존 urdf_path 는 다른 패키지 기준이므로 클리어."""
        new_name = (name or "").strip() or None
        if new_name != self._urdf_pkg_name:
            self.urdf_path_edit.clear()
        self._urdf_pkg_name = new_name

    def _on_pick_urdf(self) -> None:
        """사용자가 고른 description 패키지(ROS) / SDK·description 폴더(SDK) 안에서 .urdf 픽.

        - ROS 모드: 'description 패키지' 콤보의 sub-package 안에서만 검색.
            urdf_path = picked relative to that package.
            urdf_pkg_name = 그 sub-package 이름 → urdf_package_dir 결정.
        - SDK 모드: SDK 폴더 또는 description 폴더 안에서 검색 (기존 그대로).
        """
        is_sdk = self._wizard.method_combo.currentIndex() == 1

        if is_sdk:
            sdk = self._wizard._sdk_src
            desc = self._wizard._desc_src
            if sdk is None and desc is None:
                QMessageBox.warning(self, "폴더 미선택",
                                    "먼저 위에서 'SDK 폴더' 또는 'description 폴더'를 선택하세요.")
                return
            open_at = desc or sdk
            label = open_at.name if open_at else ""
        else:
            ros_pkg = self._wizard._ros_pkg_src
            if ros_pkg is None:
                QMessageBox.warning(self, "ROS 패키지 미선택",
                                    "먼저 위에서 'ROS 패키지 폴더'를 선택하세요.")
                return
            # description 패키지 콤보가 비어있으면 자동 갱신 시도
            if self.urdf_pkg_combo.count() == 0:
                self.refresh_urdf_pkg_combo()
            picked_pkg_name = self.urdf_pkg_combo.currentText().strip()
            if not picked_pkg_name:
                QMessageBox.warning(self, "description 패키지 미선택",
                                    "IK 섹션의 'description 패키지' 콤보에서 URDF 가 들어있는 sub-package 를\n"
                                    "먼저 선택하세요. (보이지 않으면 🔄 새로고침)")
                return
            # detect_ros_packages 결과에서 일치하는 패키지 dir 찾기
            pkgs = detect_ros_packages(ros_pkg)
            target_pkg = next((p for p in pkgs if p.name == picked_pkg_name), None)
            if target_pkg is None:
                QMessageBox.warning(self, "패키지 없음",
                                    f"'{picked_pkg_name}' 를 찾지 못했습니다. 🔄 로 갱신.")
                return
            open_at = target_pkg
            label = picked_pkg_name

        path_str, _ = QFileDialog.getOpenFileName(
            self, f"URDF 선택 — {label}", str(open_at),
            "URDF Files (*.urdf *.xacro);;All Files (*)"
        )
        if not path_str:
            return
        picked = Path(path_str).resolve()

        rel_str: Optional[str] = None
        if is_sdk:
            if self._wizard._desc_src is not None:
                try:
                    r = picked.relative_to(self._wizard._desc_src.resolve())
                    rel_str = f"description/{r.as_posix()}"
                except ValueError:
                    pass
            if rel_str is None and self._wizard._sdk_src is not None:
                try:
                    r = picked.relative_to(self._wizard._sdk_src.resolve())
                    rel_str = r.as_posix()
                except ValueError:
                    pass
            self._urdf_pkg_name = None
        else:
            # ROS 모드: 사용자가 고른 description 패키지 기준 상대
            try:
                rel_str = picked.relative_to(open_at.resolve()).as_posix()
                self._urdf_pkg_name = open_at.name
            except ValueError:
                rel_str = None

        if rel_str is None:
            QMessageBox.warning(self, "범위 밖 파일",
                                f"선택한 파일이 '{label}' 안에 있지 않습니다.")
            return
        self.urdf_path_edit.setText(rel_str)

        # URDF 에서 controlled joint 자동 추출 → 사용자에게 교체 여부 확인
        try:
            joints = parse_urdf_joints(picked)
        except RuntimeError as e:
            self._wizard._log(f"[WARN] URDF 파싱 실패: {e}")
            return

        if not joints:
            self._wizard._log("[INFO] URDF 에서 controlled joint 를 찾지 못했습니다 (수동 입력 필요).")
            return

        names_preview = ", ".join(j[0] for j in joints[:8])
        if len(joints) > 8:
            names_preview += f", ... (+{len(joints) - 8})"
        ans = QMessageBox.question(
            self, "Joints 자동 입력?",
            f"URDF 에서 controlled joint {len(joints)}개를 발견했습니다.\n"
            f"이름·하한·상한이 자동 입력되도록 현재 joints 입력을 교체할까요?\n\n"
            f"발견된 joints: {names_preview}\n\n"
            "(is_tool 체크박스는 사용자가 직접 표시해야 합니다.)",
            QMessageBox.Yes | QMessageBox.No
        )
        if ans != QMessageBox.Yes:
            return

        # 기존 joint rows 모두 제거 후 새로 추가
        for row in list(self._joints):
            self._joints_layout.removeWidget(row)
            row.deleteLater()
        self._joints.clear()
        for jname, lb, ub in joints:
            self._add_joint(name=jname, lb=lb, ub=ub)
        self._wizard._log(f"[OK] URDF 에서 joints {len(joints)}개 자동 입력")

    def _on_clear_urdf(self) -> None:
        self.urdf_path_edit.clear()

    # ---- mode toggle ----
    def set_method(self, is_sdk: bool) -> None:
        self.ros_box.setVisible(not is_sdk)
        self.sdk_box.setVisible(is_sdk)
        # description 패키지 콤보는 ROS 모드일 때만 의미 있음 (sub-package 선택)
        for i in range(self.urdf_pkg_row.count()):
            w = self.urdf_pkg_row.itemAt(i).widget()
            if w is not None:
                w.setVisible(not is_sdk)

    # ---- (de)serialize ----
    def to_dict(self) -> dict:
        joint_tuples = [r.to_tuple() for r in self._joints]
        joint_names = [t[0] for t in joint_tuples]
        joint_lb = [t[1] for t in joint_tuples]
        joint_ub = [t[2] for t in joint_tuples]
        tool_index = [i for i, t in enumerate(joint_tuples) if t[3]]
        spec = {
            "company": self.company_edit.text().strip(),
            "role": self.role_combo.currentText(),
            "joint_dim": len(joint_tuples),
            "joint_names": joint_names,
            "joint_lower_bounds": joint_lb,
            "joint_upper_bounds": joint_ub,
            "tool_inner": bool(tool_index),
            "tool_index": tool_index,
            "ik_available": self.ik_chk.isChecked(),
            "custom_fields": _csv_to_strs(self.custom_fields_edit.text()),
        }

        # ee_definitions: dual_arm 이면 L_ee / R_ee 두 항목, 그 외엔 ee 한 항목
        if spec["role"] == "dual_arm":
            l_offset_raw = self.l_ee_offset_edit.text().strip()
            r_offset_raw = self.r_ee_offset_edit.text().strip()
            ee_defs = [
                {"name": "L_ee",
                 "parent": self.l_ee_parent_edit.text().strip(),
                 "offset": _csv_to_floats(l_offset_raw) if l_offset_raw else None},
                {"name": "R_ee",
                 "parent": self.r_ee_parent_edit.text().strip(),
                 "offset": _csv_to_floats(r_offset_raw) if r_offset_raw else None},
            ]
        else:
            ee_offset_raw = self.ee_offset_edit.text().strip()
            ee_defs = [
                {"name": "ee",
                 "parent": self.ee_parent_edit.text().strip()
                           or (spec["joint_names"][-1] if spec["joint_names"] else ""),
                 "offset": _csv_to_floats(ee_offset_raw) if ee_offset_raw else None},
            ]
        ik_block = {
            "urdf_path": self.urdf_path_edit.text().strip(),
            # urdf_package_dir 은 모듈 레벨에서 wizard 가 일괄 주입 (variant 종속 아님)
            "joints_to_lock": _csv_to_strs(self.joints_lock_edit.text()),
            "ee_definitions": ee_defs,
        }
        return {
            "type": self.type_edit.text().strip(),
            "spec": spec,
            "ik": ik_block,
        }

    def _hooks_dicts(self) -> tuple[list[dict], list[dict]]:
        """현재 입력된 pre_launch / post_launch 항목을 list[dict] 로 변환.
        path 가 비어있는 pre 와 service 가 비어있는 post 는 제외."""
        pre = [r.to_dict() for r in self._pre_hooks]
        pre = [d for d in pre if d.get("path")]
        post = [r.to_dict() for r in self._post_hooks]
        post = [d for d in post if d is not None]
        return pre, post

    def driver_dict(self, is_sdk: bool, fallback_sdk_type: str) -> dict:
        pre_hooks, post_hooks = self._hooks_dicts()

        if is_sdk:
            sdk_type = self.sdk_type_edit.text().strip() or fallback_sdk_type
            d = {"kind": "sdk", "interpolation": True, "sdk_control": True, "sdk_type": sdk_type}
            if pre_hooks:
                d["pre_launch"] = pre_hooks
            if post_hooks:
                d["post_launch"] = post_hooks
            return d

        kind = self.driver_kind_combo.currentText()
        # launch args dict — 빈 key 는 제외
        args: dict[str, str] = {}
        for row in self._launch_args:
            k, v = row.to_pair()
            if k:
                args[k] = v
        launch_block = {
            "package": self.launch_pkg_edit.text().strip(),
            "launch_file": self.launch_file_edit.text().strip(),
            "args": args,
        }
        d = {
            "kind": kind,
            # interpolation: True 로 두면 agent 가 ec_joint_cmd → interp_node → write_topic
            # 으로 200Hz 보간 publish. False 면 별도 driver plugin 이 필요해 사용자 모듈에선
            # 사실상 동작 불가능 → 토픽/서비스 모드는 항상 True 가 안전한 기본값.
            "interpolation": True,
            "read_topic": self.read_topic_edit.text().strip(),
            "read_topic_msg": self.read_msg_edit.currentText().strip(),
            "write_type": kind,
            "write_topic": self.write_topic_edit.text().strip(),
            "write_topic_msg": self.write_msg_edit.currentText().strip(),
            "launch": launch_block,
        }
        if pre_hooks:
            d["pre_launch"] = pre_hooks
        if post_hooks:
            d["post_launch"] = post_hooks
        return d


# ---------------------------------------------------------------------------
# Worker thread for install (colcon build can take minutes)
# ---------------------------------------------------------------------------

class _InstallWorker(QThread):
    log = _Signal(str)
    done = _Signal(bool, str)

    def __init__(self, workdir: Path, module_id: str, ros_pkg_names: list[str]):
        super().__init__()
        self.workdir = workdir
        self.module_id = module_id
        self.ros_pkg_names = ros_pkg_names

    def run(self):
        try:
            self._emit(f"[1/3] 패킹 중: {self.workdir}")
            tar_path = self._pack()
            self._emit(f"  → {tar_path}")

            self._emit("[2/3] 프로젝트 트리에 설치")
            self._extract_to_project(tar_path)

            if self.ros_pkg_names:
                self._emit(f"[3/3] colcon build (--packages-select {' '.join(self.ros_pkg_names)})")
                rc = self._colcon_build()
                if rc != 0:
                    self.done.emit(False, f"colcon build 실패 (rc={rc})")
                    return
            else:
                self._emit("[3/3] colcon 대상 없음 (스킵)")

            self.done.emit(True, "설치 완료")
        except Exception:
            self.done.emit(False, traceback.format_exc())

    def _emit(self, s: str) -> None:
        self.log.emit(s)

    def _pack(self) -> str:
        tmp = tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False)
        tmp.close()
        with tarfile.open(tmp.name, "w:gz") as tar:
            tar.add(self.workdir, arcname=self.module_id)
        return tmp.name

    def _extract_to_project(self, tar_path: str) -> None:
        # 마켓 모듈과 같은 파이프라인을 그대로 사용한다.
        from modules import (
            _install_robot_sensor_module,
            _install_deps_in_containers,
            _save_module_manifest,
            set_module_installed,
        )
        meta = json.loads((self.workdir / "module.json").read_text())
        _install_robot_sensor_module(tar_path, self.module_id)
        # project/modules/<id>.json 저장 — 다이얼로그·is_module_installed 가 이걸 본다
        _save_module_manifest(self.module_id, meta)
        _install_deps_in_containers(meta)
        set_module_installed(self.module_id, True, version=meta.get("version"))
        try:
            os.unlink(tar_path)
        except OSError:
            pass

    def _colcon_build(self) -> int:
        cmd = [
            "docker", "exec", "-w", "/root/ros2_ws", ROS2_CONTAINER,
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && "
            f"colcon build --symlink-install --packages-select {' '.join(self.ros_pkg_names)}",
        ]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        except FileNotFoundError:
            self._emit("docker 명령을 찾을 수 없음 — 컨테이너에서 직접 colcon build 하세요.")
            return -1
        assert proc.stdout is not None
        for line in proc.stdout:
            self._emit(line.rstrip())
        return proc.wait()


# ---------------------------------------------------------------------------
# Wizard dialog
# ---------------------------------------------------------------------------

class ModuleWizard(QDialog):
    """로컬 모듈 위자드 — 사용자는 ROS 패키지/SDK 폴더를 가리키고 로봇 정보만 입력하면
    내부적으로 작업폴더 생성, 복사, module.json 생성, 패키징, 설치, colcon build 까지 자동.
    사용자에게 노출되는 건 '재료 선택'과 '로봇 정보 입력' 뿐이다.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("로컬 모듈 만들기")
        self.resize(900, 800)
        # 사용자가 고른 원본 폴더들 (아직 복사 안 됨; 설치 시 복사)
        self._ros_pkg_src: Optional[Path] = None
        self._sdk_src: Optional[Path] = None
        self._desc_src: Optional[Path] = None
        # 내부 작업 폴더 — 사용자에게 노출 X. 설치 시 또는 controller.py 편집 시 lazy 생성
        self._workdir: Optional[Path] = None
        self._worker: Optional[_InstallWorker] = None
        self._build_ui()
        self._add_variant()
        self._update_visibility()

    # ---- UI ----
    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        body = QWidget()
        scroll.setWidget(body)
        layout = QVBoxLayout(body)

        # 0) 작업 액션 — 기존 모듈 불러와 편집 / 삭제
        action_row = QHBoxLayout()
        btn_load = QPushButton("📂 기존 모듈 불러오기")
        btn_load.setToolTip("이전에 위자드로 만든 모듈을 폼에 풀어 편집을 시작합니다.")
        btn_load.clicked.connect(self._on_load_existing)
        btn_delete = QPushButton("🗑 모듈 삭제")
        btn_delete.setToolTip("위자드로 만든 로컬 모듈을 프로젝트 트리·manifest·작업 폴더에서 모두 제거합니다.")
        btn_delete.clicked.connect(self._on_delete_local)
        action_row.addWidget(btn_load)
        action_row.addWidget(btn_delete)
        action_row.addStretch()
        layout.addLayout(action_row)

        # 1) 기본 정보
        common_box = QGroupBox("기본 정보")
        common_form = QFormLayout(common_box)
        self.id_edit = QLineEdit()
        self.id_edit.setPlaceholderText("robot_myarm")
        self.id_edit.textChanged.connect(self._on_id_changed)
        self.name_edit = QLineEdit()
        self.version_edit = QLineEdit("1.0.0")
        self.desc_edit = QLineEdit()
        self.category_combo = QComboBox()
        self.category_combo.addItems(["robot", "sensor"])
        self.category_combo.currentTextChanged.connect(self._update_visibility)
        common_form.addRow(_req("ID"), self.id_edit)
        common_form.addRow(_req("이름"), self.name_edit)
        common_form.addRow("버전", self.version_edit)
        common_form.addRow("설명", self.desc_edit)
        common_form.addRow("카테고리", self.category_combo)
        layout.addWidget(common_box)

        # 1-b) 의존성 (선택) — 모듈 install 시 누락된 apt/pip 패키지 자동 설치
        deps_box = QGroupBox("의존성 (선택)")
        deps_v = QVBoxLayout(deps_box)
        deps_hint = QLabel(
            "모듈이 필요로 하는 시스템 패키지(apt)와 Python 패키지(pip).\n"
            "한 줄에 하나씩. 버전 명시 가능 (예: python-can>=4.3.1, piper_sdk).\n"
            "설치 파이프라인이 누락된 패키지를 자동으로 설치합니다."
        )
        deps_hint.setStyleSheet("color: #666; font-size: 11px;")
        deps_hint.setWordWrap(True)
        deps_v.addWidget(deps_hint)

        deps_form = QFormLayout()
        self.deps_apt_edit = QPlainTextEdit()
        self.deps_apt_edit.setPlaceholderText("(예: ros-humble-moveit, can-utils)\n한 줄에 하나씩")
        self.deps_apt_edit.setMaximumHeight(70)
        self.deps_pip_edit = QPlainTextEdit()
        self.deps_pip_edit.setPlaceholderText("(예: piper_sdk, python-can>=4.3.1)\n한 줄에 하나씩")
        self.deps_pip_edit.setMaximumHeight(70)
        deps_form.addRow("apt", self.deps_apt_edit)
        deps_form.addRow("pip", self.deps_pip_edit)
        deps_v.addLayout(deps_form)
        layout.addWidget(deps_box)

        # 2) 로봇 옵션
        self.robot_box = QGroupBox("로봇 옵션")
        rl = QVBoxLayout(self.robot_box)

        # 2-1) 제어 방식
        method_row = QHBoxLayout()
        method_row.addWidget(QLabel("제어 방식:"))
        self.method_combo = QComboBox()
        self.method_combo.addItems(["ROS (topic/service)", "SDK (직접 제어)"])
        self.method_combo.currentTextChanged.connect(self._on_method_changed)
        method_row.addWidget(self.method_combo)
        warning = QLabel(
            "⚠ ROS는 200Hz로 read/write 토픽·서비스를 호출하므로,\n"
            "    그 부하를 견딜 수 있는 ROS 패키지만 사용 가능합니다."
        )
        warning.setStyleSheet("color: #c0392b; font-size: 11px;")
        warning.setWordWrap(True)
        method_row.addWidget(warning, 1)
        rl.addLayout(method_row)

        # 2-2) ROS 모드 — ROS 패키지 폴더 선택
        self.ros_mode_box = QGroupBox("ROS 패키지")
        rmode_v = QVBoxLayout(self.ros_mode_box)
        rmode_hint = QLabel(
            "이 모듈에 사용할 ROS 패키지 폴더를 골라 주세요.\n"
            "(package.xml 이 폴더 루트에 있어야 합니다. URDF·메쉬 등도 이 폴더 안에 있어야 합니다.)"
        )
        rmode_hint.setStyleSheet("color: #666; font-size: 11px;")
        rmode_hint.setWordWrap(True)
        rmode_v.addWidget(rmode_hint)

        rpick = QHBoxLayout()
        rpick.addWidget(QLabel("패키지 폴더"))
        self.ros_pkg_path_edit = QLineEdit()
        self.ros_pkg_path_edit.setReadOnly(True)
        self.ros_pkg_path_edit.setStyleSheet("background: #f5f5f5; color: #444;")
        self.ros_pkg_path_edit.setPlaceholderText("(폴더 선택 후 자동 입력)")
        rpick.addWidget(self.ros_pkg_path_edit, 1)
        btn_pick_ros = QPushButton("📁 선택")
        btn_pick_ros.clicked.connect(self._on_pick_ros_pkg)
        rpick.addWidget(btn_pick_ros)
        rmode_v.addLayout(rpick)
        rl.addWidget(self.ros_mode_box)

        # 2-3) SDK 모드 — SDK 폴더 + description 폴더 + install_cmd + controller.py 편집
        self.sdk_mode_box = QGroupBox("SDK 설정")
        smode_v = QVBoxLayout(self.sdk_mode_box)
        smode_hint = QLabel(
            "vendor SDK 폴더(setup.py 가 폴더 루트에 있어야 함)와\n"
            "URDF·메쉬가 든 description 폴더(선택)를 골라 주세요."
        )
        smode_hint.setStyleSheet("color: #666; font-size: 11px;")
        smode_hint.setWordWrap(True)
        smode_v.addWidget(smode_hint)

        spick = QHBoxLayout()
        spick.addWidget(QLabel("SDK 폴더"))
        self.sdk_path_edit = QLineEdit()
        self.sdk_path_edit.setReadOnly(True)
        self.sdk_path_edit.setStyleSheet("background: #f5f5f5; color: #444;")
        self.sdk_path_edit.setPlaceholderText("(폴더 선택 후 자동 입력)")
        spick.addWidget(self.sdk_path_edit, 1)
        btn_pick_sdk = QPushButton("📁 선택")
        btn_pick_sdk.clicked.connect(self._on_pick_sdk)
        spick.addWidget(btn_pick_sdk)
        smode_v.addLayout(spick)

        dpick = QHBoxLayout()
        dpick.addWidget(QLabel("description 폴더 (선택)"))
        self.desc_path_edit = QLineEdit()
        self.desc_path_edit.setReadOnly(True)
        self.desc_path_edit.setStyleSheet("background: #f5f5f5; color: #444;")
        self.desc_path_edit.setPlaceholderText("(URDF/메쉬가 든 폴더, 비워둬도 됨)")
        dpick.addWidget(self.desc_path_edit, 1)
        btn_pick_desc = QPushButton("📁 선택")
        btn_pick_desc.clicked.connect(self._on_pick_desc)
        btn_clear_desc = QPushButton("지우기")
        btn_clear_desc.setMaximumWidth(60)
        btn_clear_desc.clicked.connect(self._on_clear_desc)
        dpick.addWidget(btn_pick_desc)
        dpick.addWidget(btn_clear_desc)
        smode_v.addLayout(dpick)

        sform = QFormLayout()
        self.sdk_install_edit = QLineEdit("pip3 install -e .")
        sform.addRow("install_cmd", self.sdk_install_edit)
        smode_v.addLayout(sform)

        ctrl_row = QHBoxLayout()
        self.btn_edit_controller = QPushButton("📝 controller.py 작성/편집")
        self.btn_edit_controller.setToolTip(
            "BaseSDKController 를 상속한 컨트롤러 코드를 외부 에디터에서 작성합니다.\n"
            "처음 누르면 템플릿이 자동 생성됩니다."
        )
        self.btn_edit_controller.clicked.connect(self._on_edit_controller)
        ctrl_row.addWidget(self.btn_edit_controller)
        ctrl_row.addStretch()
        smode_v.addLayout(ctrl_row)

        rl.addWidget(self.sdk_mode_box)

        # 2-4) 변형 탭
        rl.addWidget(QLabel("로봇 변형 (여러 대를 한 모듈로 묶을 수 있습니다)"))
        self.variant_tabs = QTabWidget()
        self.variant_tabs.setTabsClosable(True)
        self.variant_tabs.tabCloseRequested.connect(self._remove_variant)
        add_btn = QPushButton("+ 변형 추가")
        add_btn.clicked.connect(self._add_variant)
        self.variant_tabs.setCornerWidget(add_btn)
        rl.addWidget(self.variant_tabs)

        layout.addWidget(self.robot_box)

        # 3) 로그
        layout.addWidget(self._sep())
        layout.addWidget(QLabel("로그"))
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("monospace"))
        self.log_view.setMinimumHeight(140)
        layout.addWidget(self.log_view)

        outer.addWidget(scroll, 1)

        # 4) 하단 버튼
        bottom = QHBoxLayout()
        self.btn_validate = QPushButton("검증")
        self.btn_validate.clicked.connect(self._on_validate)
        self.btn_install = QPushButton("설치 (colcon build 포함)")
        self.btn_install.clicked.connect(self._on_install)
        self.btn_close = QPushButton("닫기")
        self.btn_close.clicked.connect(self.reject)
        bottom.addWidget(self.btn_validate)
        bottom.addWidget(self.btn_install)
        bottom.addStretch()
        bottom.addWidget(self.btn_close)
        outer.addLayout(bottom)

    def _sep(self) -> QFrame:
        f = QFrame()
        f.setFrameShape(QFrame.HLine)
        f.setFrameShadow(QFrame.Sunken)
        return f

    # ---- variant management ----
    def _add_variant(self) -> None:
        idx = self.variant_tabs.count()
        default_type = f"variant_{idx + 1}"
        v = _VariantWidget(self, default_type=default_type)
        v.set_method(self.method_combo.currentIndex() == 1)
        self.variant_tabs.addTab(v, default_type)
        self.variant_tabs.setCurrentIndex(idx)

    def _remove_variant(self, idx: int) -> None:
        if self.variant_tabs.count() <= 1:
            QMessageBox.information(self, "삭제 불가", "최소 1개의 변형은 필요합니다.")
            return
        w = self.variant_tabs.widget(idx)
        self.variant_tabs.removeTab(idx)
        w.deleteLater()

    def _refresh_variant_tab_titles(self) -> None:
        for i in range(self.variant_tabs.count()):
            v: _VariantWidget = self.variant_tabs.widget(i)  # type: ignore[assignment]
            t = v.type_edit.text().strip() or f"variant_{i + 1}"
            self.variant_tabs.setTabText(i, t)

    def _all_variants(self) -> list[_VariantWidget]:
        return [self.variant_tabs.widget(i) for i in range(self.variant_tabs.count())]  # type: ignore[return-value]

    # ---- callbacks ----
    def _on_id_changed(self, _text: str) -> None:
        # ID 가 바뀌면 미래의 workdir 도 바뀌므로 controller.py 추적은 무효화
        self._workdir = None

    # ---- load existing / delete ----
    def _list_local_modules(self) -> list[tuple[str, dict]]:
        if not LOCAL_MODULES_DIR.is_dir():
            return []
        out: list[tuple[str, dict]] = []
        for d in sorted(LOCAL_MODULES_DIR.iterdir()):
            mj = d / "module.json"
            if not d.is_dir() or not mj.is_file():
                continue
            try:
                meta = json.loads(mj.read_text())
                out.append((d.name, meta))
            except Exception:
                continue
        return out

    def _on_load_existing(self) -> None:
        items = self._list_local_modules()
        if not items:
            QMessageBox.information(
                self, "로컬 모듈 없음",
                f"{LOCAL_MODULES_DIR}\n에 위자드로 만든 모듈이 없습니다."
            )
            return
        labels = [f"{mid}  —  {meta.get('name', '')} v{meta.get('version', '')}"
                  for mid, meta in items]
        choice, ok = QInputDialog.getItem(
            self, "기존 모듈 불러오기", "수정할 모듈을 고르세요:",
            labels, 0, False,
        )
        if not ok or not choice:
            return
        mid, meta = items[labels.index(choice)]
        self._load_meta_into_form(meta, LOCAL_MODULES_DIR / mid)
        self._log(f"[OK] 모듈 불러옴: {mid}")

    def _on_delete_local(self) -> None:
        items = self._list_local_modules()
        if not items:
            QMessageBox.information(self, "삭제 가능 모듈 없음",
                                    "위자드로 만든 로컬 모듈이 없습니다.")
            return
        labels = [f"{mid}  —  {meta.get('name', '')} v{meta.get('version', '')}"
                  for mid, meta in items]
        choice, ok = QInputDialog.getItem(
            self, "모듈 삭제", "삭제할 모듈을 고르세요:", labels, 0, False,
        )
        if not ok or not choice:
            return
        mid = items[labels.index(choice)][0]
        ans = QMessageBox.question(
            self, "삭제 확인",
            f"'{mid}' 모듈을 정말 삭제할까요?\n"
            "다음을 모두 제거합니다:\n"
            f"  · 프로젝트 트리 (ros2/ros2_ws/src/{mid}/, ros2/robot_sdk/{mid}/)\n"
            f"  · manifest (project/modules/{mid}.json)\n"
            f"  · 위자드 작업 폴더 ({LOCAL_MODULES_DIR / mid})",
            QMessageBox.Yes | QMessageBox.Cancel,
        )
        if ans != QMessageBox.Yes:
            return
        self.log_view.clear()
        try:
            from modules import remove_module
            ok = remove_module(mid)
        except Exception as e:
            self._log(f"[ERR] remove_module 실패: {e}")
            return
        if not ok:
            self._log(f"[ERR] 모듈 삭제 실패: {mid}")
            return
        wd = LOCAL_MODULES_DIR / mid
        if wd.is_dir():
            try:
                shutil.rmtree(wd)
            except Exception as e:
                self._log(f"[WARN] 작업 폴더 삭제 실패: {e}")
        self._log(f"[OK] 모듈 삭제 완료: {mid}")
        QMessageBox.information(self, "삭제 완료", f"{mid} 가 삭제되었습니다.")

    def _load_meta_into_form(self, meta: dict, workdir: Path) -> None:
        """module.json 을 폼에 풀어넣는다. workdir 의 트리를 *in-place* 소스로 둔다."""
        self.id_edit.setText(meta.get("id", ""))
        self.name_edit.setText(meta.get("name", ""))
        self.version_edit.setText(meta.get("version", "1.0.0"))
        self.desc_edit.setText(meta.get("description", ""))
        self.category_combo.setCurrentText(meta.get("category", "robot"))
        deps = meta.get("dependencies") or {}
        self.deps_apt_edit.setPlainText("\n".join(deps.get("apt") or []))
        self.deps_pip_edit.setPlainText("\n".join(deps.get("pip") or []))
        self._workdir = workdir

        if (meta.get("category") or "") != "robot":
            self._update_visibility()
            return

        robots = meta.get("robots") or []
        first_kind = (robots[0].get("driver", {}).get("kind") if robots else "topic") or "topic"
        is_sdk = first_kind == "sdk"
        self.method_combo.setCurrentIndex(1 if is_sdk else 0)

        sdk_install = (meta.get("install", {}).get("sdk", {}) or {}).get("install_cmd")
        if sdk_install:
            self.sdk_install_edit.setText(sdk_install)

        # 소스 폴더 — workdir 의 기존 트리를 그대로 in-place 소스로 등록
        # (사용자가 새로 'ROS 패키지 폴더 선택' 누르면 그때 외부 경로로 교체)
        if is_sdk:
            sdk_dir = workdir / "sdk"
            if sdk_dir.is_dir():
                self._sdk_src = sdk_dir
                self.sdk_path_edit.setText(f"{sdk_dir}  (작업 폴더 안 — 재설치 시 그대로 사용)")
            else:
                self._sdk_src = None
                self.sdk_path_edit.clear()
            desc_dir = sdk_dir / "description"
            if desc_dir.is_dir():
                self._desc_src = desc_dir
                self.desc_path_edit.setText(f"{desc_dir}  (작업 폴더 안)")
            else:
                self._desc_src = None
                self.desc_path_edit.clear()
            self._ros_pkg_src = None
            self.ros_pkg_path_edit.clear()
        else:
            ros2_dir = workdir / "ros2"
            if ros2_dir.is_dir():
                self._ros_pkg_src = ros2_dir
                pkgs = detect_ros_packages(ros2_dir)
                self.ros_pkg_path_edit.setText(
                    f"{ros2_dir}  (작업 폴더 안 — {len(pkgs)}개 패키지)"
                )
            else:
                self._ros_pkg_src = None
                self.ros_pkg_path_edit.clear()
            self._sdk_src = None
            self._desc_src = None
            self.sdk_path_edit.clear()
            self.desc_path_edit.clear()

        # variant 탭 모두 비우고 manifest 의 robots[] 로 재구성
        while self.variant_tabs.count() > 0:
            w = self.variant_tabs.widget(0)
            self.variant_tabs.removeTab(0)
            if w is not None:
                w.deleteLater()

        for r in robots:
            self._add_variant()
            v = self._all_variants()[-1]
            self._populate_variant(v, r)

        if self.variant_tabs.count() == 0:
            self._add_variant()  # 최소 1개

        self._refresh_variant_tab_titles()
        self._update_visibility()

    def _populate_variant(self, v: "_VariantWidget", robot: dict) -> None:
        """variant 위젯에 manifest 의 robot dict 를 풀어넣는다."""
        v.type_edit.setText(robot.get("type", ""))
        spec = robot.get("spec", {}) or {}
        role = spec.get("role", "single_arm")
        if role in ("single_arm", "tool", "dual_arm"):
            v.role_combo.setCurrentText(role)
        v.company_edit.setText(spec.get("company", ""))
        v.ik_chk.setChecked(bool(spec.get("ik_available", True)))
        cf = spec.get("custom_fields") or []
        v.custom_fields_edit.setText(", ".join(cf) if isinstance(cf, list) else "")

        # joints 재구성
        for row in list(v._joints):
            v._joints_layout.removeWidget(row)
            row.deleteLater()
        v._joints.clear()

        names = spec.get("joint_names") or []
        lbs = spec.get("joint_lower_bounds") or []
        ubs = spec.get("joint_upper_bounds") or []
        tool_idx = set(spec.get("tool_index") or [])
        for i, name in enumerate(names):
            lb = float(lbs[i]) if i < len(lbs) else 0.0
            ub = float(ubs[i]) if i < len(ubs) else 0.0
            v._add_joint(name=name, lb=lb, ub=ub, is_tool=(i in tool_idx))
        if not v._joints:
            v._add_joint(name="joint_1", lb=-3.14, ub=3.14)

        # IK
        ik = robot.get("ik", {}) or {}
        # 저장된 urdf_path 는 absolute. UI 표시는 패키지 기준 상대로 변환.
        abs_urdf = (ik.get("urdf_path") or "").strip()
        abs_pkg_dir = (ik.get("urdf_package_dir") or "").strip()
        if abs_urdf and abs_pkg_dir and abs_urdf.startswith(abs_pkg_dir):
            rel = abs_urdf[len(abs_pkg_dir):].lstrip("/")
            v.urdf_path_edit.setText(rel)
        else:
            v.urdf_path_edit.setText(abs_urdf)
        # description 패키지 콤보 복원: urdf_package_dir 의 마지막 segment
        v.refresh_urdf_pkg_combo()
        if abs_pkg_dir:
            pkg_name = abs_pkg_dir.rstrip("/").rsplit("/", 1)[-1]
            i = v.urdf_pkg_combo.findText(pkg_name)
            if i >= 0:
                v.urdf_pkg_combo.setCurrentIndex(i)
            v._urdf_pkg_name = pkg_name
        jl = ik.get("joints_to_lock") or []
        v.joints_lock_edit.setText(", ".join(jl) if isinstance(jl, list) else "")

        ee_defs = ik.get("ee_definitions") or []

        def _offset_csv(off):
            if not off:
                return ""
            try:
                return ", ".join(str(x) for x in off)
            except TypeError:
                return ""

        if role == "dual_arm":
            l_ee = next((e for e in ee_defs if e.get("name") == "L_ee"), {}) or {}
            r_ee = next((e for e in ee_defs if e.get("name") == "R_ee"), {}) or {}
            v.l_ee_parent_edit.setText(l_ee.get("parent", "") or "")
            v.l_ee_offset_edit.setText(_offset_csv(l_ee.get("offset")))
            v.r_ee_parent_edit.setText(r_ee.get("parent", "") or "")
            v.r_ee_offset_edit.setText(_offset_csv(r_ee.get("offset")))
        else:
            ee = next((e for e in ee_defs if e.get("name") == "ee"), None)
            if ee is None and ee_defs:
                ee = ee_defs[0]
            ee = ee or {}
            v.ee_parent_edit.setText(ee.get("parent", "") or "")
            v.ee_offset_edit.setText(_offset_csv(ee.get("offset")))

        # driver
        drv = robot.get("driver", {}) or {}
        if drv.get("kind") == "sdk":
            v.sdk_type_edit.setText(drv.get("sdk_type", "") or "")
        else:
            kind = drv.get("kind", "topic")
            if kind in ("topic", "service"):
                v.driver_kind_combo.setCurrentText(kind)
            v.read_topic_edit.setText(drv.get("read_topic", "") or "")
            v.read_msg_edit.setCurrentText(drv.get("read_topic_msg", "") or "")
            v.write_topic_edit.setText(drv.get("write_topic", "") or "")
            v.write_msg_edit.setCurrentText(drv.get("write_topic_msg", "") or "")
            # launch
            launch = drv.get("launch", {}) or {}
            v.launch_pkg_edit.setText(launch.get("package", "") or "")
            v.launch_file_edit.setText(launch.get("launch_file", "") or "")
            for row in list(v._launch_args):
                v._launch_args_layout.removeWidget(row)
                row.deleteLater()
            v._launch_args.clear()
            for k, val in (launch.get("args") or {}).items():
                v._add_launch_arg(key=str(k), value=str(val))
            if not v._launch_args:
                v._add_launch_arg(key="namespace", value="{namespace}")

        # pre/post launch hooks (sdk·ros 양쪽에서 가능)
        for row in list(v._pre_hooks):
            v._pre_hooks_layout.removeWidget(row)
            row.deleteLater()
        v._pre_hooks.clear()
        for h in (drv.get("pre_launch") or []):
            if (h.get("type") or "").lower() != "script":
                continue
            try:
                wa = float(h.get("wait_after", 1.0))
            except (TypeError, ValueError):
                wa = 1.0
            v._add_pre_hook(path=h.get("path", "") or "",
                            root=h.get("root", "ros2") or "ros2",
                            wait_after=wa)

        for row in list(v._post_hooks):
            v._post_hooks_layout.removeWidget(row)
            row.deleteLater()
        v._post_hooks.clear()
        for h in (drv.get("post_launch") or []):
            if (h.get("type") or "").lower() != "ros_service":
                continue
            try:
                wb = float(h.get("wait_before", 0.0))
            except (TypeError, ValueError):
                wb = 0.0
            try:
                to = float(h.get("timeout", 5.0))
            except (TypeError, ValueError):
                to = 5.0
            v._add_post_hook(service=h.get("service", "") or "",
                             service_type=h.get("service_type", "") or "",
                             request=h.get("request") if isinstance(h.get("request"), dict) else None,
                             wait_before=wb, timeout=to)

    def _on_method_changed(self, _text: str) -> None:
        # 모드 전환 시 visibility 갱신 + variant urdf_path 클리어 (경로 의미가 다름)
        self._update_visibility()
        for v in self._all_variants():
            v.urdf_path_edit.clear()

    # ---- visibility ----
    def _update_visibility(self) -> None:
        is_robot = self.category_combo.currentText() == "robot"
        self.robot_box.setVisible(is_robot)
        is_sdk = is_robot and self.method_combo.currentIndex() == 1
        self.ros_mode_box.setVisible(is_robot and not is_sdk)
        self.sdk_mode_box.setVisible(is_sdk)
        for v in self._all_variants():
            v.set_method(is_sdk)

    # ---- log ----
    def _log(self, msg: str) -> None:
        self.log_view.appendPlainText(msg)

    # ---- source pickers ----
    def _on_pick_ros_pkg(self) -> None:
        s = QFileDialog.getExistingDirectory(self, "ROS 패키지 폴더 선택", str(Path.home()))
        if not s:
            return
        src = Path(s)
        pkgs = detect_ros_packages(src)
        if not pkgs:
            QMessageBox.warning(self, "ROS 패키지 아님",
                                f"{src} 안에 package.xml 을 찾지 못했습니다.\n"
                                "다음 중 하나여야 합니다:\n"
                                " · 단일 패키지: 폴더 루트에 package.xml\n"
                                " · 워크스페이스: 폴더/src/<pkg>/package.xml (예: piper)\n"
                                " · 평면 워크스페이스: 폴더/<pkg>/package.xml")
            return
        self._ros_pkg_src = src
        names = [p.name for p in pkgs]
        if len(pkgs) == 1 and pkgs[0] == src:
            display = f"{src}  (단일 패키지)"
        else:
            display = f"{src}  (워크스페이스: {len(pkgs)}개 패키지 — {', '.join(names[:5])}{'...' if len(names) > 5 else ''})"
        self.ros_pkg_path_edit.setText(display)
        for v in self._all_variants():
            v.urdf_path_edit.clear()
            v._urdf_pkg_name = None
            v.refresh_urdf_pkg_combo()
        self._log(f"[OK] ROS 패키지 선택: {src} ({len(pkgs)}개 패키지)")

    def _on_pick_sdk(self) -> None:
        s = QFileDialog.getExistingDirectory(self, "vendor SDK 폴더 선택", str(Path.home()))
        if not s:
            return
        src = Path(s)
        self._sdk_src = src
        self.sdk_path_edit.setText(str(src))
        for v in self._all_variants():
            v.urdf_path_edit.clear()
        if not (src / "setup.py").is_file():
            self._log(f"[WARN] {src}/setup.py 가 없습니다 — install_cmd 가 동작하지 않을 수 있습니다.")
        self._log(f"[OK] SDK 폴더 선택: {src}")

    def _on_pick_desc(self) -> None:
        s = QFileDialog.getExistingDirectory(self, "description 폴더 선택", str(Path.home()))
        if not s:
            return
        self._desc_src = Path(s)
        self.desc_path_edit.setText(str(self._desc_src))
        for v in self._all_variants():
            v.urdf_path_edit.clear()
        self._log(f"[OK] description 폴더 선택: {self._desc_src}")

    def _on_clear_desc(self) -> None:
        self._desc_src = None
        self.desc_path_edit.clear()
        for v in self._all_variants():
            v.urdf_path_edit.clear()

    # ---- controller.py 편집 ----
    def _ensure_workdir(self) -> Optional[Path]:
        """ID 기반 workdir 을 만들고 반환. ID 가 유효하지 않으면 None."""
        mid = self.id_edit.text().strip()
        if not _ID_RE.match(mid):
            QMessageBox.warning(self, "ID 형식",
                                "ID 는 소문자/숫자/언더스코어만 가능합니다 (예: robot_myarm).")
            return None
        wd = LOCAL_MODULES_DIR / mid
        wd.mkdir(parents=True, exist_ok=True)
        (wd / "sdk").mkdir(exist_ok=True)
        self._workdir = wd
        return wd

    def _on_edit_controller(self) -> None:
        wd = self._ensure_workdir()
        if wd is None:
            return
        ctrl = wd / "sdk" / "controller.py"
        if not ctrl.is_file():
            mid = self.id_edit.text().strip()
            # 첫 변형의 sdk_type 추정
            sdk_type = mid
            if self._all_variants():
                t = self._all_variants()[0].sdk_type_edit.text().strip()
                if t:
                    sdk_type = t
            ctrl.write_text(_CONTROLLER_TEMPLATE.format(
                name=self.name_edit.text().strip() or mid,
                sdk_type=sdk_type,
                class_name=_to_class_name(mid),
            ))
            self._log(f"[OK] controller.py 템플릿 생성: {ctrl}")
        open_in_external(ctrl)
        self._log(f"외부 에디터로 열었습니다: {ctrl}")

    # ---- meta ----
    def _collect_meta(self) -> dict:
        mid = self.id_edit.text().strip()
        cat = self.category_combo.currentText()
        meta: dict = {
            "id": mid,
            "name": self.name_edit.text().strip(),
            "version": self.version_edit.text().strip() or "1.0.0",
            "category": cat,
            "description": self.desc_edit.text().strip(),
            "dependencies": {
                "apt": [l.strip() for l in self.deps_apt_edit.toPlainText().splitlines() if l.strip()],
                "pip": [l.strip() for l in self.deps_pip_edit.toPlainText().splitlines() if l.strip()],
            },
            "check": {"type": "path"},
        }
        if cat == "sensor":
            meta["install"] = {"ros2": {"target": "ros2/ros2_ws/src", "build": "colcon"}}
            return meta

        is_sdk = self.method_combo.currentIndex() == 1
        meta["install"] = {"ros2": {"target": "ros2/ros2_ws/src", "build": "colcon"}}
        if is_sdk:
            meta["install"]["sdk"] = {
                "target": f"ros2/robot_sdk/{mid}",
                "install_cmd": self.sdk_install_edit.text().strip(),
            }

        robots: list[dict] = []
        for v in self._all_variants():
            d = v.to_dict()
            d["driver"] = v.driver_dict(is_sdk, fallback_sdk_type=mid)
            # urdf_package_dir 은 placeholder 형태로 저장 — module_loader 가 install
            # 시점에 실제 경로로 치환. id 와 install 폴더가 달라도 portable.
            #   ROS:  {ros2_root}/<sub_pkg>/
            #   SDK:  {sdk_root}/
            if is_sdk:
                upd = "{sdk_root}/"
            else:
                pkg_name = v._urdf_pkg_name
                if not pkg_name:
                    pkg_name = self._ros_pkg_src.name if self._ros_pkg_src else "<pkg>"
                upd = f"{{ros2_root}}/{pkg_name}/"
            ik = d.setdefault("ik", {})
            ik["urdf_package_dir"] = upd
            # urdf_path: UI 의 패키지 기준 상대 경로를 그대로 placeholder 와 합쳐
            # absolute placeholder 형태로 저장 (예: {ros2_root}/<pkg>/urdf/foo.urdf).
            rel = (ik.get("urdf_path") or "").strip()
            if rel:
                if rel.startswith("/") or rel.startswith("{"):
                    # 사용자가 직접 절대 또는 placeholder 입력한 케이스 — 그대로 사용
                    abs_urdf = rel
                else:
                    abs_urdf = upd.rstrip("/") + "/" + rel.lstrip("/")
                ik["urdf_path"] = abs_urdf
            robots.append(d)
        meta["robots"] = robots
        return meta

    # ---- pre-install validation ----
    def _validate_inputs(self) -> list[str]:
        """폼 입력값(메모리)으로부터 module.json dict 를 만들고 schema 검증.
        추가로 사용자가 *재료* 폴더를 선택했는지, 숫자 입력이 실제로 숫자인지도 체크.
        """
        errs: list[str] = []

        # 숫자 입력 검증 — to_tuple 과 _csv_to_floats 가 ValueError 를 던지지 않도록 사전에 잡는다
        is_robot = self.category_combo.currentText() == "robot"
        if is_robot:
            for vi, v in enumerate(self._all_variants()):
                vt = v.type_edit.text().strip() or f"variant_{vi + 1}"
                # joint lower/upper bounds
                for ji, jrow in enumerate(v._joints):
                    jname = jrow.name_edit.text().strip() or f"joint_{ji + 1}"
                    for label, edit in [("lower bound", jrow.lb_edit), ("upper bound", jrow.ub_edit)]:
                        txt = edit.text().strip()
                        if not _is_float(txt):
                            errs.append(f"{vt} '{jname}': {label} 이 숫자가 아닙니다 ('{txt}')")
                # ee offset (CSV)
                if v.role_combo.currentText() == "dual_arm":
                    for label, edit in [("L_ee offset", v.l_ee_offset_edit),
                                        ("R_ee offset", v.r_ee_offset_edit)]:
                        bad = _csv_non_floats(edit.text())
                        if bad:
                            errs.append(f"{vt} {label} 에 숫자가 아닌 값: {bad}")
                else:
                    bad = _csv_non_floats(v.ee_offset_edit.text())
                    if bad:
                        errs.append(f"{vt} ee offset 에 숫자가 아닌 값: {bad}")

        # 숫자 검증 통과해야 _collect_meta 가 안전. 실패면 여기서 멈춤.
        if errs:
            return errs

        try:
            meta = self._collect_meta()
        except Exception as e:
            return [f"입력 파싱 실패: {e}"]
        errs.extend(validate_module_meta(meta))

        if meta["category"] == "robot":
            is_sdk = self.method_combo.currentIndex() == 1
            if is_sdk:
                if self._sdk_src is None:
                    errs.append("SDK 폴더가 선택되지 않았습니다.")
            else:
                if self._ros_pkg_src is None:
                    errs.append("ROS 패키지 폴더가 선택되지 않았습니다.")
                # ROS 드라이버: topic / msg / launch 형식 검증 (variant 단위)
                for vi, v in enumerate(self._all_variants()):
                    vt = v.type_edit.text().strip() or f"variant_{vi + 1}"
                    rt = v.read_topic_edit.text().strip()
                    wt = v.write_topic_edit.text().strip()
                    rm = v.read_msg_edit.currentText().strip()
                    wm = v.write_msg_edit.currentText().strip()
                    if not rt:
                        errs.append(f"{vt}: read_topic 비어 있음")
                    elif not rt.startswith("/"):
                        errs.append(f"{vt}: read_topic 은 '/' 로 시작해야 함 (현재 '{rt}')")
                    if not wt:
                        errs.append(f"{vt}: write_topic 비어 있음")
                    elif not wt.startswith("/"):
                        errs.append(f"{vt}: write_topic 은 '/' 로 시작해야 함 (현재 '{wt}')")
                    if not rm:
                        errs.append(f"{vt}: read_topic_msg 비어 있음")
                    elif not _looks_like_msg_type(rm):
                        errs.append(f"{vt}: read_topic_msg 형식 오류 ('{rm}'). "
                                    "'<pkg>/<Name>' 또는 '<pkg>/msg/<Name>' / '<pkg>/srv/<Name>' 형식이어야 함")
                    if not wm:
                        errs.append(f"{vt}: write_topic_msg 비어 있음")
                    elif not _looks_like_msg_type(wm):
                        errs.append(f"{vt}: write_topic_msg 형식 오류 ('{wm}'). "
                                    "'<pkg>/<Name>' 또는 '<pkg>/msg/<Name>' / '<pkg>/srv/<Name>' 형식이어야 함")

                    # launch package / file
                    lpkg = v.launch_pkg_edit.text().strip()
                    lfile = v.launch_file_edit.text().strip()
                    if not lpkg:
                        errs.append(f"{vt}: launch package 비어 있음")
                    if not lfile:
                        errs.append(f"{vt}: launch file 비어 있음")
                    elif not (lfile.endswith(".py") or lfile.endswith(".xml") or lfile.endswith(".yaml")):
                        errs.append(f"{vt}: launch file 확장자가 .py/.xml/.yaml 이 아닙니다 ('{lfile}')")
                    # launch args 의 key 가 ros2 launch 가 받는 식별자 형태인지
                    seen_keys: set[str] = set()
                    for row in v._launch_args:
                        k, _val = row.to_pair()
                        if not k:
                            continue
                        if not re.match(r"^[a-zA-Z_][a-zA-Z0-9_]*$", k):
                            errs.append(f"{vt}: launch arg key '{k}' 형식이 잘못됨 (영문/숫자/언더스코어만)")
                        elif k in seen_keys:
                            errs.append(f"{vt}: launch arg key '{k}' 가 중복됩니다")
                        seen_keys.add(k)

            # variant 별 urdf_path 가 비어 있으면 경고 (필수는 아님)
            for i, r in enumerate(meta.get("robots", [])):
                if not r["ik"].get("urdf_path"):
                    errs.append(f"robots[{i}] ({r.get('type','?')}): URDF 가 선택되지 않았습니다.")

        return errs

    def _on_validate(self) -> None:
        self.log_view.clear()
        errs = self._validate_inputs()
        if errs:
            for e in errs:
                self._log(f"[ERR] {e}")
            return
        self._log("[OK] 모든 검증 통과 — 설치 가능 상태입니다.")
        # SDK 모드 + controller.py 작성 여부도 안내
        if self.method_combo.currentIndex() == 1:
            wd = LOCAL_MODULES_DIR / self.id_edit.text().strip()
            ctrl = wd / "sdk" / "controller.py"
            if ctrl.is_file():
                ok, info = self._verify_controller(ctrl)
                self._log(("[OK] " if ok else "[ERR] ") + info)
            else:
                self._log("[INFO] controller.py 가 아직 작성되지 않았습니다 — 설치 시 템플릿이 자동 생성됩니다.")

    def _verify_controller(self, ctrl_path: Path) -> tuple[bool, str]:
        rc = subprocess.run(["python3", "-m", "py_compile", str(ctrl_path)],
                            capture_output=True, text=True).returncode
        if rc != 0:
            return False, f"controller.py 문법 오류 (python3 -m py_compile rc={rc})"
        text = ctrl_path.read_text()
        if "BaseSDKController" not in text:
            return False, "controller.py 에 BaseSDKController 상속 코드를 찾을 수 없음"
        return True, "controller.py 문법 OK + BaseSDKController 상속 확인"

    # ---- workdir staging (install 시 호출) ----
    def _stage_workdir(self) -> Path:
        """사용자가 고른 원본 폴더들을 모듈 작업 디렉터리에 복사하여 마켓 모듈과 같은
        레이아웃을 만든다. 소스가 이미 workdir 안이면 (예: '기존 모듈 불러오기' 후
        재설치) in-place 로 두고 재복사하지 않는다 (controller.py 는 항상 보존)."""
        mid = self.id_edit.text().strip()
        wd = LOCAL_MODULES_DIR / mid
        wd.mkdir(parents=True, exist_ok=True)

        is_sdk = self.method_combo.currentIndex() == 1
        ros2_dir = wd / "ros2"
        sdk_dir = wd / "sdk"

        def _is_inside_workdir(p: Optional[Path]) -> bool:
            if p is None:
                return False
            try:
                Path(p).resolve().relative_to(wd.resolve())
                return True
            except (ValueError, RuntimeError):
                return False

        ros_inplace = (not is_sdk) and _is_inside_workdir(self._ros_pkg_src)
        sdk_inplace = is_sdk and _is_inside_workdir(self._sdk_src)
        desc_inplace = is_sdk and _is_inside_workdir(self._desc_src)

        # ros2/ 클린: 외부 소스가 있을 때만 (in-place 면 보존)
        if not is_sdk and not ros_inplace:
            if ros2_dir.exists():
                shutil.rmtree(ros2_dir)
            ros2_dir.mkdir()
        else:
            ros2_dir.mkdir(exist_ok=True)

        # sdk/ 콘텐츠 클린 (controller.py 보존). in-place 면 보존
        if is_sdk and not sdk_inplace:
            if sdk_dir.is_dir():
                for item in sdk_dir.iterdir():
                    if item.name == "controller.py":
                        continue
                    if item.is_dir():
                        shutil.rmtree(item)
                    else:
                        item.unlink()
            else:
                sdk_dir.mkdir()
        else:
            sdk_dir.mkdir(exist_ok=True)

        # ROS 모드: 외부 소스가 있을 때만 복사
        if not is_sdk and self._ros_pkg_src is not None and not ros_inplace:
            pkgs = detect_ros_packages(self._ros_pkg_src)
            if not pkgs:
                raise RuntimeError(f"{self._ros_pkg_src} 안에서 ROS 패키지를 찾지 못했습니다.")
            (ros2_dir / "src").mkdir(parents=True, exist_ok=True)
            for pkg in pkgs:
                dst = ros2_dir / "src" / pkg.name
                shutil.copytree(pkg, dst, symlinks=True)
            self._log(f"  복사: {len(pkgs)}개 ROS 패키지 → ros2/src/ ({', '.join(p.name for p in pkgs)})")
        elif not is_sdk and ros_inplace:
            self._log("  in-place: 작업 폴더의 ros2/ 트리 그대로 사용")

        # SDK 모드: 외부 SDK 소스
        if is_sdk and self._sdk_src is not None and not sdk_inplace:
            for item in self._sdk_src.iterdir():
                if item.name == "controller.py":
                    continue
                d = sdk_dir / item.name
                if item.is_dir():
                    shutil.copytree(item, d, symlinks=True)
                else:
                    shutil.copy2(item, d)
            self._log(f"  복사: SDK 콘텐츠 → sdk/")
        elif is_sdk and sdk_inplace:
            self._log("  in-place: 작업 폴더의 sdk/ 트리 그대로 사용")

        # description: 외부 소스일 때만 복사
        if is_sdk and self._desc_src is not None and not desc_inplace:
            desc_dst = sdk_dir / "description"
            if desc_dst.exists():
                shutil.rmtree(desc_dst)
            shutil.copytree(self._desc_src, desc_dst, symlinks=True)
            self._log(f"  복사: description/ → sdk/description/")

        # SDK 모드에서 controller.py 가 없으면 템플릿 생성
        if is_sdk:
            ctrl = sdk_dir / "controller.py"
            if not ctrl.is_file():
                sdk_type = mid
                if self._all_variants():
                    t = self._all_variants()[0].sdk_type_edit.text().strip()
                    if t:
                        sdk_type = t
                ctrl.write_text(_CONTROLLER_TEMPLATE.format(
                    name=self.name_edit.text().strip() or mid,
                    sdk_type=sdk_type,
                    class_name=_to_class_name(mid),
                ))
                self._log(f"  생성: sdk/controller.py (템플릿)")

        # module.json 작성
        meta = self._collect_meta()
        (wd / "module.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
        self._log(f"  생성: module.json")

        self._workdir = wd
        return wd

    def _scan_ros_packages(self, ros2_dir: Path) -> list[str]:
        if not ros2_dir.is_dir():
            return []
        pkgs: list[str] = []
        for px in ros2_dir.rglob("package.xml"):
            try:
                txt = px.read_text()
                m = re.search(r"<name>\s*([\w\-]+)\s*</name>", txt)
                if m:
                    pkgs.append(m.group(1))
            except Exception:
                continue
        return pkgs

    # ---- install ----
    def _on_install(self) -> None:
        self.log_view.clear()
        errs = self._validate_inputs()
        if errs:
            QMessageBox.warning(self, "검증 실패", "\n".join(errs))
            return

        mid = self.id_edit.text().strip()
        existing = LOCAL_MODULES_DIR / mid
        if existing.exists() and any(existing.iterdir()):
            ans = QMessageBox.question(
                self, "재설치?",
                f"이미 같은 ID('{mid}')의 모듈 작업 폴더가 있습니다. 덮어쓰고 재설치할까요?",
                QMessageBox.Yes | QMessageBox.Cancel,
            )
            if ans != QMessageBox.Yes:
                return

        self._log("[1/4] 모듈 폴더 준비")
        try:
            wd = self._stage_workdir()
        except Exception as e:
            self._log(f"[ERR] 폴더 준비 실패: {e}")
            QMessageBox.critical(self, "오류", str(e))
            return

        ros_pkgs = self._scan_ros_packages(wd / "ros2")
        self._set_buttons_enabled(False)
        self._worker = _InstallWorker(wd, mid, ros_pkgs)
        self._worker.log.connect(self._log)
        self._worker.done.connect(self._on_install_done)
        self._worker.start()

    def _on_install_done(self, ok: bool, msg: str) -> None:
        self._log(("[OK] " if ok else "[ERR] ") + msg)
        self._set_buttons_enabled(True)
        if ok:
            QMessageBox.information(self, "설치 완료", "런처를 재시작하면 모듈이 인식됩니다.")

    def _set_buttons_enabled(self, on: bool) -> None:
        for b in (self.btn_validate, self.btn_install, self.btn_close):
            b.setEnabled(on)

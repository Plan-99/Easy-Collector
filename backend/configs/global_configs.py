import os


# Datasets are persistent runtime data and must NOT live under the bind-mounted
# source tree (/root/src is bind-mounted from the dev checkout). Resolve under
# EASYTRAINER_DATA_DIR when available so writes land on the persistent volume.
_data_root = os.environ.get('EASYTRAINER_DATA_DIR')
DATASET_DIR = os.path.join(_data_root, 'datasets') if _data_root else '/root/src/backend/datasets'
CHECKPOINT_DIR = os.path.join(_data_root, 'checkpoints') if _data_root else '/root/src/backend/checkpoints'


def resolve_checkpoint_dir(checkpoint_id):
    """Locate the on-disk checkpoint directory for a given id.

    Tries, in order:
      1. ``$EASYTRAINER_DATA_DIR/checkpoints/<id>`` — canonical install location
         (where remote_train extracts the downloaded model).
      2. ``$TRAINING_SERVER_DATA_DIR/checkpoints/<machine_id>/<id>`` — raw output
         of local in-container training.
      3. ``/root/src/backend/checkpoints/<id>`` — legacy path from when the
         project lived under ``src/``.

    Returns the first existing directory, or the canonical path (1) if none
    exist so callers attempting writes still get a sensible target.
    """
    cid = str(checkpoint_id)
    candidates = [os.path.join(CHECKPOINT_DIR, cid)]

    training_data = os.environ.get('TRAINING_SERVER_DATA_DIR')
    if training_data:
        try:
            from ..utils.machine_id import machine_id
            candidates.append(os.path.join(training_data, 'checkpoints', machine_id(), cid))
        except Exception:
            pass

    legacy = os.path.join('/root/src/backend/checkpoints', cid)
    if legacy not in candidates:
        candidates.append(legacy)

    for path in candidates:
        if os.path.isdir(path):
            return path
    return candidates[0]

# Trained checkpoints — training_server가 이 경로에 직접 저장하고 (host volume
# share), remote 학습 시 _download_and_install_model이 동일한 경로에 풀어준다.
# machine_id namespace는 lazy하게 평가해야 하므로 함수로 노출.
_checkpoint_base = os.path.join(
    _data_root or '/opt/easytrainer', 'training_data', 'checkpoints'
)


def get_checkpoint_dir(checkpoint_id) -> str:
    """checkpoint_id에 해당하는 로컬 체크포인트 디렉터리 절대경로."""
    from ..utils.machine_id import machine_id
    return os.path.join(_checkpoint_base, machine_id(), str(checkpoint_id))

# 빌트인 / 외부 module manifest 모두 module_loader 가 단일 진실원천으로 노출.
# 이전엔 _ALL_ROBOTS 에 in-code dict 로 박혀있던 9 로봇 + 4 그리퍼 정의를
# `modules/robots/<id>/module.json::robots[]` 로 옮겼다.
#   tutorial_arm                        → backend/configs/tutorial_defaults.py
#   test_arm                            → module_loader._BUILTIN_ROBOTS
#   piper / piper(no gripper)           → modules/robots/piper/module.json
#   tm_12 / tm_12s / tm_12_robotiq      → modules/robots/techman/module.json
#   rb3_730es_u / rb5_850e              → modules/robots/rbpodo/module.json
#   kinova_gen3_7dof_robotiq_2f_85      → modules/robots/kinova/module.json
#   fairino_fr5                         → modules/robots/fairino/module.json
#   jaka_zu12                           → modules/robots/jaka/module.json
#   robotiq_2f_85                       → modules/robots/robotiq/module.json
#   2FG7                                → modules/robots/onrobot/module.json
from .module_loader import load_all_robots as _load_all_robots
from .module_loader import load_all_sensors as _load_all_sensors

# 옛 코드가 `from ...configs.global_configs import SUPPORT_ROBOTS` 외에
# `_ALL_ROBOTS` / `_ALL_SENSORS` 를 직접 import 하는 경로는 없지만, 안전을 위해
# 빈 list 로 alias. 진실원천은 modules/ manifest.
_ALL_ROBOTS: list = []
_ALL_SENSORS: list = []


def _get_support_robots():
    """module_loader 가 manifest 에서 직접 읽어옴 — manifest 자체가 "설치됨"의
    증거이므로 별도 필터링 불필요."""
    return _load_all_robots()

def _get_support_sensors():
    """robot 과 동일 패턴 — sensor 도 manifest 의 sensors[] 가 진실원천.
    sensor 모듈뿐 아니라 robot 모듈(예: kinova)도 vision sensor 를 포함할 수 있다.
    """
    return _load_all_sensors()

# 하위호환: 기존 코드에서 SUPPORT_ROBOTS / SUPPORT_SENSORS를 직접 참조하는 곳 대응
class _DynamicList:
    """매 접근 시 함수를 호출하여 최신 목록을 반환하는 프록시."""
    def __init__(self, fn):
        self._fn = fn
    def __iter__(self):
        return iter(self._fn())
    def __len__(self):
        return len(self._fn())
    def __getitem__(self, idx):
        return self._fn()[idx]
    def __contains__(self, item):
        return item in self._fn()
    def __bool__(self):
        return bool(self._fn())

SUPPORT_ROBOTS = _DynamicList(_get_support_robots)
SUPPORT_SENSORS = _DynamicList(_get_support_sensors)

def get_robot_by_name(name):
    return next((robot for robot in _get_support_robots() if robot.get('name') == name), None)

def get_sensor_by_name(name):
    return next((sensor for sensor in _get_support_sensors() if sensor.get('name') == name), None)
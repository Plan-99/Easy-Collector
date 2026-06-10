from einops import rearrange
import cv2
import numpy as np
import torch
import os
from torch.utils.data import TensorDataset, DataLoader
import json
from types import SimpleNamespace
from lerobot.configs.types import PolicyFeature, FeatureType
from torchvision import transforms
from transformers import AutoImageProcessor
from PIL import Image
from scipy.spatial.transform import Rotation
import pyarrow.parquet as pq
from ..utils.lerobot_io import (
    read_episode, list_episodes, get_dataset_info, get_norm_stats_from_dataset,
    _parse_image_value,
    PARQUET_PATH_TEMPLATE, IMAGE_PATH_TEMPLATE, DEFAULT_CHUNK_SIZE,
)


def _eepos_to_homogeneous(eepos6: np.ndarray) -> np.ndarray:
    """[x, y, z, rx, ry, rz] (axis-angle) → 4x4 homogeneous matrix."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_rotvec(eepos6[3:6]).as_matrix()
    T[:3, 3] = eepos6[:3]
    return T


def _homogeneous_to_eepos(T: np.ndarray) -> np.ndarray:
    """4x4 homogeneous → [x, y, z, rx, ry, rz]."""
    rotvec = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    return np.concatenate([T[:3, 3], rotvec]).astype(np.float32)


def relative_trajectory_in_local_frame_to_world_deltas(
    relative_traj: np.ndarray, current_eepos: np.ndarray
) -> np.ndarray:
    """역변환 (inference). current EE local frame 의 trajectory → world frame
    sequential deltas (move_ee_delta_step 으로 적용 가능한 형태).

    각 waypoint 를 world 로 풀고, 연속 waypoint 간 차분 (회전은 합성) 으로 step
    별 delta 계산. 첫 delta 는 current → world[0], 이후는 world[i-1] → world[i].
    Tool 차원은 absolute 값 그대로 (한 step 내 절대 위치로 보냄).
    """
    relative_traj = np.asarray(relative_traj, dtype=np.float32)
    current_eepos = np.asarray(current_eepos, dtype=np.float32)
    if relative_traj.ndim == 1:
        relative_traj = relative_traj[None, :]
    N, D = relative_traj.shape

    T_curr = _eepos_to_homogeneous(current_eepos)

    # 모든 waypoint 를 world frame 으로 변환
    T_world_list = []
    for i in range(N):
        T_rel = _eepos_to_homogeneous(relative_traj[i, :6])
        T_world_list.append(T_curr @ T_rel)

    deltas = np.zeros_like(relative_traj)
    T_prev = T_curr  # 첫 delta 는 current → world[0]
    for i in range(N):
        T_i = T_world_list[i]
        # Position delta in world
        delta_t = T_i[:3, 3] - T_prev[:3, 3]
        # Rotation delta: R_delta = R_i @ R_prev^T
        R_delta = T_i[:3, :3] @ T_prev[:3, :3].T
        rotvec_delta = Rotation.from_matrix(R_delta).as_rotvec()
        deltas[i, :3] = delta_t
        deltas[i, 3:6] = rotvec_delta
        if D > 6:
            deltas[i, 6:] = relative_traj[i, 6:]  # tool absolute pass-through
        T_prev = T_i
    return deltas


def relative_trajectory_to_delta(waypoints: np.ndarray, current_eepos=None) -> np.ndarray:
    """relative trajectory → sequential delta 역변환 (inference 시 사용).

    current_eepos 가 주어지면 DexUMI 스타일 (current EE local frame trajectory → world
    sequential deltas). None 이면 legacy 동작 (waypoints 를 world cumsum 으로 간주,
    naive 차분). 옛 학습 모델 호환 위해 기본 None.

    Args:
        waypoints: [T, D] relative trajectory. 앞 6 차원이 EE pose, D > 6 일 때 6 이후는
                   tool joint (absolute).
        current_eepos: [D] 현재 EE pose. 주어지면 local-frame waypoints 를 world 로
                       풀고 sequential delta 계산.

    Returns:
        [T, D] sequential deltas (+ tool absolute).
    """
    waypoints = np.asarray(waypoints, dtype=np.float32)
    if current_eepos is not None:
        # DexUMI 스타일 역변환: utils 의 helper 사용.
        return relative_trajectory_in_local_frame_to_world_deltas(waypoints, current_eepos)

    # Legacy 경로 (current_eepos 없을 때) — naive 누적합 모델용.
    T = len(waypoints)
    deltas = np.zeros_like(waypoints)

    # Translation: 연속 차분
    deltas[0, :3] = waypoints[0, :3]
    deltas[1:, :3] = np.diff(waypoints[:, :3], axis=0)

    # Rotation: 연속 회전 차분
    deltas[0, 3:6] = waypoints[0, 3:6]
    for i in range(1, T):
        r_prev = Rotation.from_rotvec(waypoints[i - 1, 3:6])
        r_curr = Rotation.from_rotvec(waypoints[i, 3:6])
        deltas[i, 3:6] = (r_prev.inv() * r_curr).as_rotvec()

    # Tool 차원: 그대로 복사
    if waypoints.shape[1] > 6:
        deltas[:, 6:] = waypoints[:, 6:]

    return deltas


# ─── action_key / obs_state_keys 통일 헬퍼 ───────────────────────────────
#
# 데이터 스키마 정합성:
#   action.joint     ← qaction (절대 joint position target).
#   action.ee_delta  ← ee_delta_action (t+1 shift, legacy 'action' 컬럼과 동일).
#   observation.qpos ← 현재 frame joint 위치 (= legacy observation.state).
#   observation.qvel/qeffort/eepos ← 그 외 관측치.
#
# action_key 값:
#   - 'joint' 또는 'qaction' → action.joint 컬럼 사용.
#   - 'ee_delta' 또는 'ee_delta_action' → action.ee_delta 컬럼 사용.
#   - 'relative_ee_pos' / 기타 → 기존 legacy 'action' 컬럼 (특수 전처리 분기 유지).
#
# obs_state_keys 값: ['qpos', 'qvel', 'qeffort', 'eepos'] 중 임의 부분집합.
# 학습/추론 모두 같은 순서로 concat 해서 observation state 구성.

_ACTION_KEY_ALIASES = {
    'qaction': 'joint',
    'joint': 'joint',
    'ee_delta_action': 'ee_delta',
    'ee_delta': 'ee_delta',
    'relative_ee_pos': 'relative_ee_pos',
    # relative_joint_pos: action target = chunk-anchored delta from observation.qpos.
    # Anchor 출처는 obs_state_keys와 독립적인 별도 채널 (parquet의 observation.qpos
    # 컬럼). relative_ee_pos가 observation.eepos를 anchor로 쓰는 것과 대칭.
    'relative_joint_pos': 'relative_joint_pos',
}


def _absolute_action_dims_from_features(action_names, has_succeed: bool):
    """action.joint feature names + has_succeed 로부터 absolute dim 인덱스 자동 추출.

    relative_joint_pos 모드에서 어떤 dim이 delta로 변환돼선 안 되는지 결정.
    - 이름에 'gripper' 또는 'tool' 포함 → absolute (개폐 명령 / 도구 joint)
    - has_succeed=True → action의 마지막 dim이 succeed/done flag → absolute

    relative_ee_pos가 EE pose의 처음 6 dim을 transform하고 나머지(tool)는 그대로
    두는 패턴과 같은 논리. user가 따로 설정할 필요 없음.

    Args:
        action_names: dataset features['action.joint']['names'] (list of str).
                       Empty/None이면 빈 list 반환 (호출자가 fallback).
        has_succeed: True면 action 마지막 dim이 succeed bit로 append됨.

    Returns:
        list of int — absolute로 유지할 action dim 인덱스.
    """
    if not action_names:
        return []
    abs_dims = []
    for i, name in enumerate(action_names):
        nl = (name or '').lower()
        if 'gripper' in nl or 'tool' in nl:
            abs_dims.append(i)
    if has_succeed:
        abs_dims.append(len(action_names))
    return abs_dims


def process_image(image, vision_backbone='resnet18', to_cuda=False, pixel_range='01', image_resolution=None):
    """Preprocess an image into a model-ready tensor.

    pixel_range: '01' → standard torchvision ToTensor output in [0, 1].
                 '-11' → scaled to [-1, 1] (mandatory for PI05 / PaliGemma / SigLIP —
                         openpi preprocess_observation explicitly does `img / 255 * 2 - 1`).
                         Feeding [0, 1] into pi05_base shifts the vision feature
                         distribution and wrecks pretrained visual grounding; LoRA
                         can't fully recover it, which matches the "moves but misses"
                         symptom during inference.

    image_resolution: (H, W) target resize used to UNIFY heterogeneous source resolutions.
                      Defaults to (224, 224) — backwards compatible with the previous
                      hard-coded resize. At inference time the checkpoint metadata's
                      ``image_resolution`` (saved at training time) should be passed in
                      so the runtime preprocessing matches what the model was trained on.
    """
    if image_resolution is None:
        image_resolution = (224, 224)
    if isinstance(image_resolution, int):
        image_resolution = (image_resolution, image_resolution)
    elif isinstance(image_resolution, (list, tuple)) and len(image_resolution) == 2:
        image_resolution = (int(image_resolution[0]), int(image_resolution[1]))
    else:
        raise ValueError(f"image_resolution must be int or (H, W); got {image_resolution!r}")

    if not isinstance(image, Image.Image):
        image = Image.fromarray(np.array(image))
    if vision_backbone not in VISION_BACKBONE_MAP:
        image_transform = transforms.Compose([
            transforms.Resize(image_resolution),
            transforms.ToTensor(),
        ])
        image = image_transform(image)
        if pixel_range == '-11':
            image = image * 2.0 - 1.0
    else:
        image_processor = AutoImageProcessor.from_pretrained(VISION_BACKBONE_MAP[vision_backbone])
        h, w = image_resolution
        image = image_processor(
            image,
            do_resize=True, do_center_crop=False,
            size={"height": h, "width": w},
        )['pixel_values'][0]  # backbone's own normalization

    return image.cuda() if to_cuda else image


# Computes the mean of a list of dictionaries, where each dictionary represents an epoch's metrics.


# Detaches all tensors in a dictionary.


def zoom_image(img, point, size):
    height, width = img.shape[:2]

    
    # 중심 좌표를 기준으로 관심 영역 크기 계산
    half_width = int(size[0] / 2)
    half_height = int(size[1] / 2)

    x1 = max(point[0] - half_width, 0)
    y1 = max(point[1] - half_height, 0)
    x2 = min(point[0] + half_width, width)
    y2 = min(point[1] + half_height, height)

    # 관심 영역 크롭
    cropped = img[y1:y2, x1:x2]
    
    return cropped


def input_caching(prompt):
    cache_file_path = "input_cache.json"
    if os.path.exists(cache_file_path):
        with open(cache_file_path, "r") as f:
            cache = json.load(f)
    else:
        cache = {}
    default = cache.get(prompt, "")
    def prefill_hook():
        readline.insert_text(default)  # 기본값 입력
        readline.redisplay()          # 화면에 표시
    readline.set_pre_input_hook(prefill_hook)

    answer = input(prompt)

    cache[prompt] = answer

    with open(cache_file_path, "w") as f:
        json.dump(cache, f, indent=4)

    return answer


def ros_image_to_numpy(image_msg):
    if isinstance(image_msg, CompressedImage):
        # 압축 이미지 처리
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        image_array = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 기본 BGR 형태로 디코딩됨
        image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)  # RGB로 변환
        image_array = image_array[:, :, ::-1]  # BGR -> RGB
        return image_array

    # 일반 Image 메시지 처리
    encoding_to_dtype = {
        'rgb8': ('uint8', 3),
        'bgr8': ('uint8', 3),
        'mono8': ('uint8', 1),
        'mono16': ('uint16', 1),
        'rgba8': ('uint8', 4),
        'bgra8': ('uint8', 4),
    }

    if image_msg.encoding not in encoding_to_dtype:
        raise ValueError(f"Unsupported encoding: {image_msg.encoding}")
    
    dtype, channels = encoding_to_dtype[image_msg.encoding]
    data = np.frombuffer(image_msg.data, dtype=dtype)
    image_array = data.reshape((image_msg.height, image_msg.width, channels))
    
    if image_msg.encoding == 'bgr8':
        image_array = image_array[:, :, ::-1]  # BGR -> RGB
    elif image_msg.encoding == 'bgra8':
        image_array = image_array[:, :, [2, 1, 0, 3]]  # BGRA -> RGBA
    
    return image_array


def rescale_val(val, origin_rng, rescaled_rng):
    return rescaled_rng[0] + (rescaled_rng[1] - rescaled_rng[0]) * ((val - origin_rng[0]) / (origin_rng[1] - origin_rng[0]))


_pi05_tokenizer = None

def _get_pi05_tokenizer():
    """Lazy-load PaliGemma tokenizer for PI05."""
    global _pi05_tokenizer
    if _pi05_tokenizer is None:
        from transformers import AutoTokenizer
        _pi05_tokenizer = AutoTokenizer.from_pretrained("google/paligemma-3b-pt-224")
    return _pi05_tokenizer


def prepare_pi05_language_tokens(batch, config, norm_stats=None):
    """Convert language_instruction + state → tokenized language tokens for PI05.

    Replicates the logic of Pi05PrepareStateTokenizerProcessorStep + TokenizerProcessorStep.
    """
    tokenizer = _get_pi05_tokenizer()
    max_length = getattr(config, 'tokenizer_max_length', 200)
    max_state_dim = getattr(config, 'max_state_dim', 32)

    # Get language instruction (string or list of strings)
    lang = batch.get('language_instruction', '')
    if isinstance(lang, str):
        lang = [lang]

    # Get state for discretization
    state = batch.get('observation.state')
    prompts = []
    for i, task_text in enumerate(lang):
        cleaned = task_text.strip().replace("_", " ").replace("\n", " ") if task_text else ""
        state_str = ""
        if state is not None:
            s = state[i] if state.dim() > 1 else state
            s_np = s.cpu().numpy().flatten()
            # Normalize to [-1, 1] using min-max if norm_stats available
            if norm_stats and 'observation.state' in norm_stats:
                s_min = norm_stats['observation.state']['min']
                s_max = norm_stats['observation.state']['max']
                s_range = s_max - s_min
                s_range[s_range < 1e-6] = 1.0
                s_np = 2.0 * (s_np - s_min) / s_range - 1.0
                s_np = np.clip(s_np, -1.0, 1.0)
            # Pad to max_state_dim
            if len(s_np) < max_state_dim:
                s_np = np.concatenate([s_np, np.zeros(max_state_dim - len(s_np))])
            # Discretize into 256 bins
            bins = np.linspace(-1, 1, 257)[:-1]
            discretized = np.digitize(s_np, bins) - 1
            state_str = " ".join(map(str, discretized.astype(int)))
        prompt = f"Task: {cleaned}, State: {state_str};\nAction: "
        prompts.append(prompt)

    tokenized = tokenizer(
        prompts,
        max_length=max_length,
        truncation=True,
        padding="max_length",
        return_tensors="pt",
    )

    device = state.device if state is not None else torch.device('cuda')
    batch['observation.language.tokens'] = tokenized['input_ids'].to(device)
    batch['observation.language.attention_mask'] = tokenized['attention_mask'].to(dtype=torch.bool, device=device)

    return batch


def make_easytrainer_processors(policy_type, cfg, dataset_stats=None, pretrained_path=None):
    """Per-policy dispatch for building / loading the LeRobot Normalize+Unnormalize pipeline.

    Bypasses ``lerobot.policies.factory.make_pre_post_processors`` because that module
    eagerly imports ``lerobot.envs.configs`` → robots → motors, none of which we install
    in the EasyTrainer container. We just call the per-policy ``make_*_pre_post_processors``
    builders directly so the dependency surface stays minimal.

    Args:
        policy_type: 'ACT' | 'Diffusion' | 'PI05'
        cfg: the policy config (ACTConfig / DiffusionConfig / PI05Config)
        dataset_stats: numpy stats dict from get_norm_stats() — used when building from scratch.
        pretrained_path: when set, load saved processor json/safetensors from this dir
            instead of building. ``dataset_stats`` is ignored in this branch.

    Returns:
        (preprocessor, postprocessor) PolicyProcessorPipeline tuple, or (None, None)
        on a load failure when pretrained_path is set (caller falls back to raw I/O).
    """
    from lerobot.processor.pipeline import PolicyProcessorPipeline
    from lerobot.processor.converters import (
        batch_to_transition,
        transition_to_batch,
        policy_action_to_transition,
        transition_to_policy_action,
    )
    from lerobot.utils.constants import (
        POLICY_PREPROCESSOR_DEFAULT_NAME,
        POLICY_POSTPROCESSOR_DEFAULT_NAME,
    )

    if pretrained_path is not None:
        # Force-import the policy-specific processor modules before deserializing the
        # saved pipeline JSON. ProcessorStepRegistry is populated by @register decorators
        # at import time — without this, loading a PI05 checkpoint fails with
        # "Processor step 'pi05_prepare_state_tokenizer_processor_step' not found in
        # registry" and the code silently falls back to raw I/O (no normalization).
        # That skipped normalization is what makes the robot behave erratically at
        # inference even though training looked fine.
        if policy_type == 'PI05':
            import lerobot.policies.pi05.processor_pi05  # noqa: F401
        elif policy_type == 'ACT':
            import lerobot.policies.act.processor_act  # noqa: F401
        elif policy_type == 'Diffusion':
            import lerobot.policies.diffusion.processor_diffusion  # noqa: F401
        try:
            preprocessor = PolicyProcessorPipeline.from_pretrained(
                pretrained_model_name_or_path=pretrained_path,
                config_filename=f"{POLICY_PREPROCESSOR_DEFAULT_NAME}.json",
                to_transition=batch_to_transition,
                to_output=transition_to_batch,
            )
            postprocessor = PolicyProcessorPipeline.from_pretrained(
                pretrained_model_name_or_path=pretrained_path,
                config_filename=f"{POLICY_POSTPROCESSOR_DEFAULT_NAME}.json",
                to_transition=policy_action_to_transition,
                to_output=transition_to_policy_action,
            )
            return preprocessor, postprocessor
        except Exception as e:
            print(f'[WARN] make_easytrainer_processors: failed to load processors from '
                  f'{pretrained_path} ({type(e).__name__}: {e}). Returning (None, None).')
            return None, None

    # Build from scratch using per-policy factories
    if policy_type == 'ACT':
        from lerobot.policies.act.processor_act import make_act_pre_post_processors
        return make_act_pre_post_processors(config=cfg, dataset_stats=dataset_stats)
    if policy_type == 'Diffusion':
        from lerobot.policies.diffusion.processor_diffusion import make_diffusion_pre_post_processors
        return make_diffusion_pre_post_processors(config=cfg, dataset_stats=dataset_stats)
    if policy_type == 'PI05':
        from lerobot.policies.pi05.processor_pi05 import make_pi05_pre_post_processors
        return make_pi05_pre_post_processors(config=cfg, dataset_stats=dataset_stats)
    raise ValueError(f'make_easytrainer_processors: unsupported policy_type {policy_type!r}')


def convert_lists_to_tuples(obj):
    """
    딕셔너리나 리스트 내부의 모든 리스트를 재귀적으로 튜플로 변환합니다.
    """
    # 입력된 객체가 딕셔너리일 경우
    if isinstance(obj, dict):
        return {key: convert_lists_to_tuples(value) for key, value in obj.items()}
    # 입력된 객체가 리스트일 경우
    elif isinstance(obj, list):
        return tuple(convert_lists_to_tuples(item) for item in obj)
    # 딕셔너리나 리스트가 아니면 그대로 반환
    else:
        return obj
    

VISION_BACKBONE_MAP = {
    'dinov2': 'facebook/dinov2-base',
    'dinov3': 'facebook/dinov3-vitb16-pretrain-lvd1689m',
}
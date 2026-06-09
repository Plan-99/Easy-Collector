"""Curriculum 자가 학습 API.

Planner 위에 얹힌 Curriculum → CheckpointGroup → Stage → Rollout 계층의 CRUD 와
롤아웃 시작/정지/상태, 초기화(reset), 실패 데이터 버리기(discard failure)를 제공한다.
docs/design-docs/2026-05-29_curriculum-self-training.md 참고.
"""
import os
import shutil

from flask import Blueprint, request, current_app

from ...configs.global_configs import DATASET_DIR
from ...database.models.curriculum_model import Curriculum as CurriculumModel
from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel
from ...database.models.stage_model import Stage as StageModel
from ...database.models.rollout_model import Rollout as RolloutModel
from ...database.models.rollout_result_model import RolloutResult as RolloutResultModel
from ...database.models.dataset_model import Dataset as DatasetModel
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel

curriculum_bp = Blueprint('curriculum_bp', __name__)

DATASET_ROLES = ('success', 'failure', 'dagger')

# 그룹 색상 팔레트(Quasar color name). 그룹 index 순으로 부여.
GROUP_COLORS = [
    'deep-purple', 'teal', 'deep-orange', 'indigo',
    'pink', 'green', 'cyan', 'amber', 'light-blue', 'red',
]

DEFAULT_MISSION = {
    'target_success_count': 20,
    'failure_save_prob': 0.3,
}


def _rollout_process_name(curriculum_id):
    return f'curriculum_rollout_{curriculum_id}'


def _next_group_color(curriculum):
    """이미 쓰인 색을 피해 팔레트에서 다음 색을 고른다."""
    used = {g.color for g in curriculum.checkpoint_groups if g.color}
    for c in GROUP_COLORS:
        if c not in used:
            return c
    # 팔레트 소진 시 그룹 수 기준 순환.
    return GROUP_COLORS[len(list(curriculum.checkpoint_groups)) % len(GROUP_COLORS)]


def _planner_checkpoint_ids(planner):
    """플래너의 모든 체크포인트 블록에서 checkpoint_id 수집(중복 제거, 순서 유지)."""
    from ..routes.planner import _ensure_plans_loaded
    plans = _ensure_plans_loaded(planner) or []
    seen, out = set(), []
    for grp in plans:
        for blk in grp.get('blocks') or []:
            if blk.get('type') == 'checkpoint':
                cid = blk.get('checkpoint_id')
                if cid is not None and cid not in seen:
                    seen.add(cid)
                    out.append(cid)
    return out


# ── 내부 헬퍼 ──────────────────────────────────────────────────────────────

def _group_checkpoint_blocks(curriculum, group):
    """그룹 멤버 cp 를 참조하는 플랜의 **체크포인트 블록들**.

    반환: ``[{block_id, checkpoint_id, name, task_id}]`` (plan 등장 순서, 블록 단위).
    같은 checkpoint_id 가 여러 블록으로 등장하면 각각 별도 항목이 된다. 그룹 멤버십은
    기존대로 ``checkpoint_ids`` 기준 — 그 cp 를 쓰는 블록만 이 그룹의 행이 된다.
    커리큘럼 기록(데이터셋/판정조건/대시보드)의 블록 단위 키 source-of-truth.
    """
    from ..routes.planner import _ensure_plans_loaded
    member_cps = set(group._get_json_field('checkpoint_ids') or [])
    planner = curriculum.planner if curriculum else None
    plans = (_ensure_plans_loaded(planner) or []) if planner else []
    out, seen = [], set()
    for grp in plans:
        for blk in grp.get('blocks') or []:
            if blk.get('type') != 'checkpoint':
                continue
            cid, bid = blk.get('checkpoint_id'), blk.get('id')
            if cid is None or bid is None or bid in seen or cid not in member_cps:
                continue
            seen.add(bid)
            ckpt = CheckpointModel.find(cid)
            out.append({
                'block_id': bid,
                'checkpoint_id': cid,
                'name': blk.get('name') or (ckpt.name if ckpt else f'CP #{cid}'),
                'task_id': blk.get('workspace_id') or (ckpt.task_id if ckpt else None),
            })
    return out


def _create_stage_datasets_for(stage, group, blocks):
    """주어진 체크포인트 **블록들**에 대해 success/failure/dagger 데이터셋 3개씩 생성.

    task_id 는 블록의 워크스페이스로 설정 + stage_id 로 Stage 종속. block_id 로 블록 단위
    분리, checkpoint_id 도 보존(라벨·학습용). origin='curriculum' 으로 표식하여
    워크스페이스 UI 편집/병합/삭제를 차단한다.
    """
    for blk in blocks:
        for role in DATASET_ROLES:
            ds = DatasetModel.create(
                name=f"blk{blk['block_id']}_stage{stage.index}_{role}",
                task_id=blk.get('task_id'),
                origin='curriculum',
                stage_id=stage.id,
                checkpoint_group_id=group.id,
                checkpoint_id=blk.get('checkpoint_id'),
                block_id=blk['block_id'],
                role=role,
            )
            # 빈 폴더를 미리 만들지 않는다 — record_episode 가 첫 에피소드 기록 시
            # 생성하므로(불필요한 빈 폴더 방지). 초기화 직후 datasets 디렉터리가 깨끗.


def _create_stage_datasets(stage, group):
    """Stage 생성 시 그룹 내 전 체크포인트 블록에 대해 데이터셋 생성."""
    curriculum = CurriculumModel.find(group.curriculum_id)
    blocks = _group_checkpoint_blocks(curriculum, group) if curriculum else []
    _create_stage_datasets_for(stage, group, blocks)


def _delete_dataset_rows(datasets):
    """데이터셋 행 + 디스크 폴더 삭제."""
    for ds in datasets:
        folder = os.path.join(DATASET_DIR, str(ds.id))
        if os.path.isdir(folder):
            shutil.rmtree(folder, ignore_errors=True)
        ds.delete_instance()


# ── Curriculum CRUD ────────────────────────────────────────────────────────

@curriculum_bp.route('/curriculums', methods=['GET'])
def list_curriculums():
    query = CurriculumModel.all_active()
    planner_id = request.args.get('planner_id')
    if planner_id is not None:
        query = query.where(CurriculumModel.planner_id == int(planner_id))
    return {'status': 'success', 'curriculums': [c.to_dict() for c in query]}, 200


@curriculum_bp.route('/curriculum/<id>', methods=['GET'])
def get_curriculum(id):
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    return {'status': 'success', 'curriculum': curriculum.to_dict()}, 200


@curriculum_bp.route('/curriculum', methods=['POST'])
def create_curriculum():
    data = request.json or {}
    if not data.get('planner_id'):
        return {'status': 'error', 'message': 'planner_id is required'}, 400
    # Curriculum 과 Planner 는 1:1 — 이미 있으면 그것을 반환(idempotent).
    existing = (
        CurriculumModel.all_active()
        .where(CurriculumModel.planner_id == int(data['planner_id']))
        .first()
    )
    if existing:
        return {'status': 'success', 'curriculum_id': existing.id, 'existed': True}, 200
    curriculum = CurriculumModel.create(
        name=data.get('name'),
        planner_id=data.get('planner_id'),
        status=CurriculumModel.STATUS_IDLE,
    )
    # 최초 그룹 1개 자동 생성 — 플래너의 모든 체크포인트를 한 그룹에 넣는다.
    planner = curriculum.planner
    cp_ids = _planner_checkpoint_ids(planner) if planner else []
    group = CheckpointGroupModel.create(
        name='Group 1',
        curriculum_id=curriculum.id,
        color=GROUP_COLORS[0],
        checkpoint_ids=cp_ids,
        motion_block_ids=[],
        checkpoint_settings={},
        block_noise={},
        mission=dict(DEFAULT_MISSION),
    )
    # 첫 Stage 자동 생성(+ 데이터셋).
    stage = StageModel.create(
        checkpoint_group_id=group.id,
        index=0,
        status=StageModel.STATUS_ACTIVE,
        rollout_noise={},
        success_criteria={},
        success_count=0,
        failure_count=0,
    )
    _create_stage_datasets(stage, group)
    return {'status': 'success', 'curriculum_id': curriculum.id}, 200


@curriculum_bp.route('/curriculum/<id>', methods=['PUT'])
def update_curriculum(id):
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    data = request.json or {}
    if 'name' in data:
        curriculum.name = data['name']
    curriculum.save()
    return {'status': 'success', 'message': 'Curriculum updated'}, 200


@curriculum_bp.route('/curriculum/<id>', methods=['DELETE'])
def delete_curriculum(id):
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    # 종속된 그룹/스테이지/데이터셋 정리.
    for group in curriculum.checkpoint_groups:
        _delete_group_cascade(group)
    curriculum.delete_instance()
    return {'status': 'success', 'message': 'Curriculum deleted'}, 200


# ── CheckpointGroup CRUD ───────────────────────────────────────────────────

def _delete_group_cascade(group):
    for stage in group.stages:
        _delete_dataset_rows(stage.datasets)
        stage.delete_instance()
    group.delete_instance()


@curriculum_bp.route('/curriculum/<id>/checkpoint_group', methods=['POST'])
def create_checkpoint_group(id):
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    data = request.json or {}
    group = CheckpointGroupModel.create(
        name=data.get('name'),
        curriculum_id=curriculum.id,
        checkpoint_ids=data.get('checkpoint_ids', []),
        # 체크포인트별 학습 설정 { cp_id: {base_dataset_ids, train_settings} }
        checkpoint_settings=data.get('checkpoint_settings', {}),
        # 그룹 정책(승급 목표·저장확률·상승률)
        mission=data.get('mission', {}),
    )
    return {'status': 'success', 'checkpoint_group_id': group.id}, 200


@curriculum_bp.route('/checkpoint_group/<id>', methods=['PUT'])
def update_checkpoint_group(id):
    group = CheckpointGroupModel.find(id)
    if not group:
        return {'status': 'error', 'message': 'Checkpoint group not found'}, 404
    data = request.json or {}
    for field in ('name', 'color', 'checkpoint_ids', 'motion_block_ids',
                  'checkpoint_settings', 'block_noise', 'mission'):
        if field in data:
            setattr(group, field, data[field])
    group.save()
    return {'status': 'success', 'message': 'Checkpoint group updated'}, 200


# ── 블록 ↔ 그룹 배정 (플래너 탭 블록 설정 다이얼로그) ──────────────────────

def _group_of_checkpoint(curriculum, checkpoint_id):
    for g in curriculum.checkpoint_groups:
        if checkpoint_id in (g._get_json_field('checkpoint_ids') or []):
            return g
    return None


@curriculum_bp.route('/curriculum/<id>/:set_checkpoint_settings', methods=['POST'])
def set_checkpoint_settings(id):
    """체크포인트별 설정을 그 체크포인트가 속한 그룹에 저장.

    base_dataset_ids, train_settings 외에 판정 조건 정책:
      - initial_max_steps: 최초(stage 0) 길이 제한
      - length_limit_rate: 다음 stage 길이 제한 = 성공 에피소드 평균 길이 × rate
      - success_threshold: 성공 토큰 임계값
    현재 active stage 의 success_criteria 도 initial_max_steps / threshold 로 시드한다.
    """
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    data = request.json or {}
    cp_id = data.get('checkpoint_id')
    block_id = data.get('block_id')
    group = _group_of_checkpoint(curriculum, cp_id)
    if group is None:
        return {'status': 'error', 'message': 'Checkpoint not in any group'}, 404
    # checkpoint_settings 는 학습(그룹/cp 단위) config — cp 키 유지.
    settings = group._get_json_field('checkpoint_settings') or {}
    settings[str(cp_id)] = {
        'base_dataset_ids': data.get('base_dataset_ids', []),
        'train_settings': data.get('train_settings', {}),
        'initial_max_steps': data.get('initial_max_steps', 600),
        'length_limit_rate': data.get('length_limit_rate', 1.5),
        'success_threshold': data.get('success_threshold', 0.5),
        'succeed_done_frames': int(data.get('succeed_done_frames') or 3),
        # 성공 데이터 다운샘플: enabled 면 성공 에피소드를 rate(=stride) 로 솎아서 기록.
        'success_downsample': bool(data.get('success_downsample')),
        'success_downsample_rate': max(1, int(data.get('success_downsample_rate') or 1)),
    }
    group.checkpoint_settings = settings
    group.save()

    # 현재 stage 의 success_criteria 를 **블록 단위**로 시드. block_id 미제공(구버전)
    # 이면 그 cp 를 쓰는 모든 블록에 적용.
    stage = group.current_stage
    if stage is not None:
        target_blocks = [block_id] if block_id else [
            b['block_id'] for b in _group_checkpoint_blocks(curriculum, group)
            if b['checkpoint_id'] == cp_id
        ]
        crit = stage._get_json_field('success_criteria') or {}
        for bid in target_blocks:
            entry = dict(crit.get(bid) or {})
            entry['max_steps'] = data.get('initial_max_steps', 600)
            entry['success_threshold'] = data.get('success_threshold', 0.5)
            # succeed 토큰 done debounce 프레임 수 (블록 단위). 기본 3.
            entry['succeed_done_frames'] = int(data.get('succeed_done_frames') or 3)
            # 성공 데이터 다운샘플 (블록 단위) — 롤아웃 기록 시 적용.
            entry['success_downsample'] = bool(data.get('success_downsample'))
            entry['success_downsample_rate'] = max(1, int(data.get('success_downsample_rate') or 1))
            crit[bid] = entry
        stage.success_criteria = crit
        stage.save()
    return {'status': 'success'}, 200


@curriculum_bp.route('/curriculum/<id>/:assign_checkpoint', methods=['POST'])
def assign_checkpoint(id):
    """체크포인트를 다른 그룹으로 이동(그룹 변경). 체크포인트별 설정도 함께 옮긴다."""
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    data = request.json or {}
    cp_id = data.get('checkpoint_id')
    target_group_id = data.get('group_id')
    src = _group_of_checkpoint(curriculum, cp_id)
    dst = CheckpointGroupModel.find(target_group_id)
    if dst is None:
        return {'status': 'error', 'message': 'Target group not found'}, 404
    if src and src.id == dst.id:
        return {'status': 'success', 'message': 'No-op'}, 200

    moved_setting = None
    if src:
        s_ids = src._get_json_field('checkpoint_ids') or []
        s_set = src._get_json_field('checkpoint_settings') or {}
        moved_setting = s_set.pop(str(cp_id), None)
        src.checkpoint_ids = [c for c in s_ids if c != cp_id]
        src.checkpoint_settings = s_set
        src.save()

    d_ids = dst._get_json_field('checkpoint_ids') or []
    if cp_id not in d_ids:
        d_ids.append(cp_id)
    dst.checkpoint_ids = d_ids
    if moved_setting is not None:
        d_set = dst._get_json_field('checkpoint_settings') or {}
        d_set[str(cp_id)] = moved_setting
        dst.checkpoint_settings = d_set
    dst.save()
    _sync_group_stages(dst)
    if src:
        _sync_group_stages(src)
    return {'status': 'success'}, 200


@curriculum_bp.route('/curriculum/<id>/:split_checkpoint', methods=['POST'])
def split_checkpoint(id):
    """체크포인트를 현재 그룹에서 떼어 새 그룹을 만든다(그룹 분리)."""
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    data = request.json or {}
    cp_id = data.get('checkpoint_id')
    new_name = data.get('name') or 'New Group'
    src = _group_of_checkpoint(curriculum, cp_id)
    if src is None:
        return {'status': 'error', 'message': 'Checkpoint not in any group'}, 404

    s_ids = src._get_json_field('checkpoint_ids') or []
    s_set = src._get_json_field('checkpoint_settings') or {}
    moved_setting = s_set.pop(str(cp_id), None)
    src.checkpoint_ids = [c for c in s_ids if c != cp_id]
    src.checkpoint_settings = s_set
    src.save()

    new_group = CheckpointGroupModel.create(
        name=new_name,
        curriculum_id=curriculum.id,
        color=_next_group_color(curriculum),
        checkpoint_ids=[cp_id],
        motion_block_ids=[],
        checkpoint_settings={str(cp_id): moved_setting} if moved_setting else {},
        block_noise={},
        mission=src._get_json_field('mission') or dict(DEFAULT_MISSION),
    )
    stage = StageModel.create(
        checkpoint_group_id=new_group.id,
        index=0,
        status=StageModel.STATUS_ACTIVE,
        rollout_noise={},
        success_criteria={},
        success_count=0,
        failure_count=0,
    )
    _create_stage_datasets(stage, new_group)
    _sync_group_stages(src)
    return {'status': 'success', 'checkpoint_group_id': new_group.id}, 200


@curriculum_bp.route('/curriculum/<id>/:assign_motion_block', methods=['POST'])
def assign_motion_block(id):
    """모션 블록을 한 그룹에 배정(+ noise rate/offset 저장). group_id=null 이면 배정 해제."""
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    data = request.json or {}
    block_id = data.get('block_id')
    target_group_id = data.get('group_id')  # None → 해제
    noise = data.get('block_noise')  # { rate:{...}, offset:{...} }

    # 모든 그룹에서 이 블록을 제거.
    for g in curriculum.checkpoint_groups:
        ids = g._get_json_field('motion_block_ids') or []
        bn = g._get_json_field('block_noise') or {}
        changed = False
        if block_id in ids:
            g.motion_block_ids = [b for b in ids if b != block_id]
            changed = True
        if str(block_id) in bn and (target_group_id is None or g.id != target_group_id):
            bn.pop(str(block_id), None)
            g.block_noise = bn
            changed = True
        if changed:
            g.save()

    if target_group_id is not None:
        dst = CheckpointGroupModel.find(target_group_id)
        if dst is None:
            return {'status': 'error', 'message': 'Target group not found'}, 404
        ids = dst._get_json_field('motion_block_ids') or []
        if block_id not in ids:
            ids.append(block_id)
        dst.motion_block_ids = ids
        if noise is not None:
            bn = dst._get_json_field('block_noise') or {}
            bn[str(block_id)] = noise
            dst.block_noise = bn
        dst.save()
    return {'status': 'success'}, 200


def _sync_group_stages(group):
    """그룹 멤버십이 바뀌면 현재 Stage 의 데이터셋을 멤버 체크포인트와 맞춘다.

    아직 롤아웃 전(데이터 없음)인 active stage 에 대해서만 누락 데이터셋을 보충한다.
    이미 데이터가 쌓인 경우는 건드리지 않는다(reset 으로만).
    """
    stage = group.current_stage
    if stage is None:
        return
    curriculum = CurriculumModel.find(group.curriculum_id)
    blocks = _group_checkpoint_blocks(curriculum, group) if curriculum else []
    existing_bids = {ds.block_id for ds in stage.datasets if ds.block_id}
    missing = [b for b in blocks if b['block_id'] not in existing_bids]
    if missing:
        _create_stage_datasets_for(stage, group, missing)


@curriculum_bp.route('/checkpoint_group/<id>', methods=['DELETE'])
def delete_checkpoint_group(id):
    group = CheckpointGroupModel.find(id)
    if not group:
        return {'status': 'error', 'message': 'Checkpoint group not found'}, 404
    _delete_group_cascade(group)
    return {'status': 'success', 'message': 'Checkpoint group deleted'}, 200


# ── Stage ──────────────────────────────────────────────────────────────────

@curriculum_bp.route('/checkpoint_group/<id>/stage', methods=['POST'])
def create_stage(id):
    group = CheckpointGroupModel.find(id)
    if not group:
        return {'status': 'error', 'message': 'Checkpoint group not found'}, 404
    data = request.json or {}
    existing = group.stages
    next_index = (max((s.index or 0 for s in existing), default=-1) + 1) if existing else 0
    stage = StageModel.create(
        checkpoint_group_id=group.id,
        index=data.get('index', next_index),
        status=StageModel.STATUS_ACTIVE,
        rollout_noise=data.get('rollout_noise', {}),
        success_criteria=data.get('success_criteria', {}),
        success_count=0,
        failure_count=0,
    )
    _create_stage_datasets(stage, group)
    return {'status': 'success', 'stage_id': stage.id}, 200


@curriculum_bp.route('/stage/<id>', methods=['PUT'])
def update_stage(id):
    stage = StageModel.find(id)
    if not stage:
        return {'status': 'error', 'message': 'Stage not found'}, 404
    data = request.json or {}
    for field in ('rollout_noise', 'success_criteria', 'status'):
        if field in data:
            setattr(stage, field, data[field])
    stage.save()
    return {'status': 'success', 'message': 'Stage updated'}, 200


@curriculum_bp.route('/stage/<id>/:discard_failure', methods=['POST'])
def discard_failure(id):
    """실패 데이터 버리기 — 이 Stage 의 failure 데이터셋 내용 비우고 failure_count 리셋."""
    stage = StageModel.find(id)
    if not stage:
        return {'status': 'error', 'message': 'Stage not found'}, 404
    for ds in stage.datasets:
        if ds.role != 'failure':
            continue
        folder = os.path.join(DATASET_DIR, str(ds.id))
        if os.path.isdir(folder):
            shutil.rmtree(folder, ignore_errors=True)
        os.makedirs(folder, exist_ok=True)
    stage.failure_count = 0
    stage.save()
    return {'status': 'success', 'message': 'Failure data discarded'}, 200


@curriculum_bp.route('/stage/<id>/:reset', methods=['POST'])
def reset_stage(id):
    """스테이지 초기화 — 이 Stage 의 모든 데이터셋(success/failure/dagger) 내용을 비우고
    success_count / failure_count 를 리셋. 데이터셋 행/구조는 유지한다."""
    stage = StageModel.find(id)
    if not stage:
        return {'status': 'error', 'message': 'Stage not found'}, 404

    # 롤아웃 진행 중엔 거부 — stop_rollout 먼저.
    group = CheckpointGroupModel.find(stage.checkpoint_group_id)
    if group:
        proc_name = _rollout_process_name(group.curriculum_id)
        if proc_name in current_app.pm.processes:
            return {'status': 'error', 'message': 'Stop the rollout before resetting'}, 409

    for ds in stage.datasets:
        folder = os.path.join(DATASET_DIR, str(ds.id))
        if os.path.isdir(folder):
            shutil.rmtree(folder, ignore_errors=True)
        os.makedirs(folder, exist_ok=True)
    # 이 stage 의 RolloutResult 도 삭제. 대시보드의 평균길이/성공률 그래프는
    # rollout_results 의 episodes(steps) 에서 계산되므로 안 지우면 초기화 후에도
    # 평균길이가 남는다. 또한 _recompute_stage_counts(매 startup)가 rollout_results
    # 로 success/failure_count 를 되살리므로 카운트 리셋도 무효가 된다.
    for res in RolloutResultModel.all_active().where(RolloutResultModel.stage_id == stage.id):
        res.delete_instance()
    stage.success_count = 0
    stage.failure_count = 0
    stage.save()
    return {'status': 'success', 'message': 'Stage reset'}, 200


@curriculum_bp.route('/checkpoint_group/<id>/:revert_stage', methods=['POST'])
def revert_stage(id):
    """현재(최신) 스테이지를 삭제하고 이전 스테이지로 복귀 — 승급(_graduate_group)의
    역연산.

      1) 그룹 체크포인트를 부모(load_model_id)로 되돌린다. 승급 시 새 cp 는
         load_model_id=직전 cp 로 만들어지므로, 현재 cp 의 부모가 곧 이전 스테이지의
         체크포인트다.
      2) 플래너 블록의 checkpoint_id 도 같은 매핑으로 되돌린다.
      3) 현재 스테이지(+데이터셋/롤아웃결과)와 승급으로 생긴(이제 미참조) 체크포인트를
         삭제한다.
      4) 그룹 status=collecting, 이전 스테이지를 active 로.

    최초(index 0) 스테이지면 복귀할 이전 스테이지가 없어 거부.
    """
    group = CheckpointGroupModel.find(id)
    if not group:
        return {'status': 'error', 'message': 'Group not found'}, 404

    # 롤아웃/학습 진행 중엔 거부 — 먼저 멈춰야 한다.
    proc_name = _rollout_process_name(group.curriculum_id)
    if proc_name in current_app.pm.processes:
        return {'status': 'error', 'message': 'Stop the rollout before reverting'}, 409
    if group.status == CheckpointGroupModel.STATUS_TRAINING:
        return {'status': 'error', 'message': 'Cannot revert while training'}, 409

    cur = group.current_stage
    if cur is None or (cur.index or 0) <= 0:
        return {'status': 'error', 'message': 'No previous stage to revert to'}, 400

    # 현재 그룹 체크포인트 → 부모(load_model_id) 매핑 (str(current) → parent).
    cur_cp_ids = group._get_json_field('checkpoint_ids') or []
    mapping = {}
    orphan_ids = []  # 승급으로 생긴 현재 cp — revert 후 미참조 → soft-delete.
    for cp in cur_cp_ids:
        ck = CheckpointModel.find(cp)
        parent = getattr(ck, 'load_model_id', None) if ck else None
        if parent is not None:
            mapping[str(cp)] = parent
            orphan_ids.append(cp)
    if not mapping:
        return {'status': 'error',
                'message': '복귀할 이전 체크포인트(load_model_id)를 찾을 수 없습니다'}, 400

    # 1) 그룹 체크포인트/설정 되돌리기.
    new_ids = [mapping.get(str(cp), cp) for cp in cur_cp_ids]
    cs = group._get_json_field('checkpoint_settings') or {}
    new_cs = {}
    if isinstance(cs, dict):
        for cp in cur_cp_ids:
            conf = cs.get(str(cp))
            if conf is None:
                conf = cs.get(cp)
            if conf is not None:
                new_cs[str(mapping.get(str(cp), cp))] = conf
    group.checkpoint_ids = new_ids
    group.checkpoint_settings = new_cs
    group.status = CheckpointGroupModel.STATUS_COLLECTING
    group.training_map = {}
    group.save()

    # 2) 플래너 블록 checkpoint_id 되돌리기 (current → parent). curriculum_train 의
    #    교체 헬퍼 재사용. 순환 import 회피 위해 함수 내 import.
    from ..process.curriculum_train import _replace_planner_checkpoints
    curriculum = CurriculumModel.find(group.curriculum_id)
    planner = curriculum.planner if curriculum else None
    if planner is not None:
        plans = planner._get_json_field('plans') or []
        _replace_planner_checkpoints((plans, planner), mapping)

    # 3) 현재 스테이지 + 데이터셋 + 롤아웃 결과 삭제.
    _delete_dataset_rows(cur.datasets)
    for res in RolloutResultModel.all_active().where(RolloutResultModel.stage_id == cur.id):
        res.delete_instance()
    cur.delete_instance()

    # 승급으로 생긴 체크포인트(이제 어디서도 참조 안 함) soft-delete — 이전 cp 로
    #    되돌렸으므로 현재 cp 는 버린다.
    for cp in orphan_ids:
        ck = CheckpointModel.find(cp)
        if ck:
            try:
                ck.delete_instance()
            except Exception as e:
                print(f'[curriculum] revert: checkpoint {cp} delete failed: {e}')

    # 4) 이전 스테이지가 현재가 됨 — active 로 보장.
    prev = group.current_stage
    if prev is not None:
        prev.status = StageModel.STATUS_ACTIVE
        prev.save()

    return {'status': 'success', 'message': 'Reverted to previous stage',
            'stage_index': prev.index if prev else None}, 200


# ── Rollout 시작 / 정지 / 상태 ─────────────────────────────────────────────

@curriculum_bp.route('/curriculum/<id>/:start_rollout', methods=['POST'])
def start_rollout(id):
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404

    proc_name = _rollout_process_name(curriculum.id)
    if proc_name in current_app.pm.processes:
        return {'status': 'error', 'message': 'A rollout is already in progress'}, 409

    data = request.json or {}
    target_group_ids = data.get('target_group_ids') or []
    if not target_group_ids:
        return {'status': 'error', 'message': 'target_group_ids is required'}, 400

    try:
        repeat_count = int(data.get('repeat_count', 1))
    except (TypeError, ValueError):
        repeat_count = 1

    # 즉시 DAgger 모드 — 체크포인트 진입 즉시 expert 데모 수집 (모델 실행 skip).
    force_dagger = bool(data.get('force_dagger', False))

    # Phase 3 엔진 — lazy import 로 순환참조/미구현 시점 회피.
    from ..process.curriculum_rollout import curriculum_rollout

    curriculum.status = CurriculumModel.STATUS_RUNNING
    curriculum.save()

    current_app.pm.start_function(
        name=proc_name,
        func=curriculum_rollout,
        curriculum_id=curriculum.id,
        target_group_ids=target_group_ids,
        repeat_count=repeat_count,
        app=current_app._get_current_object(),
        socketio_instance=current_app.pm.socketio,
        force_dagger=force_dagger,
    )
    return {'status': 'success', 'message': 'Rollout started'}, 200


@curriculum_bp.route('/curriculum/<id>/:stop_rollout', methods=['POST'])
def stop_rollout(id):
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    proc_name = _rollout_process_name(curriculum.id)
    if proc_name in current_app.pm.processes:
        current_app.pm.stop_function(proc_name)
    curriculum.status = CurriculumModel.STATUS_STOPPED
    curriculum.save()
    return {'status': 'success', 'message': 'Rollout stopped'}, 200


@curriculum_bp.route('/curriculum/<id>/:upgrade_now', methods=['POST'])
def upgrade_now(id):
    """현재 collecting 상태인 모든 그룹을 즉시 학습으로 전환 (강제 승급).

    승급 조건 (mission target 도달) 을 무시하고 사용자 명시적 트리거. 그룹의
    모든 stage 에 걸쳐 누적된 ``success`` + ``dagger`` 데이터셋 + base
    데이터셋으로 학습 시작. 학습 끝나면 평소 graduation 흐름 그대로 다음 stage.

    rollout 이 돌고 있는 동안 호출하면 안전상 거부 — stop_rollout 먼저.
    """
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    proc_name = _rollout_process_name(curriculum.id)
    if proc_name in current_app.pm.processes:
        return {'status': 'error', 'message': 'Stop the rollout first'}, 409

    from ..process.curriculum_train import _start_group_training
    from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel
    promoted = []
    for group in curriculum.checkpoint_groups:
        if group.status != CheckpointGroupModel.STATUS_COLLECTING:
            continue
        stage = group.current_stage
        if stage is None or stage.status != stage.STATUS_ACTIVE:
            continue
        try:
            _start_group_training(
                curriculum, group, stage, current_app.pm.socketio,
                app=current_app._get_current_object(),
            )
            promoted.append(group.id)
        except Exception as e:
            print(f"[upgrade_now] group {group.id} failed: {e}")
    return {
        'status': 'success',
        'promoted_group_ids': promoted,
        'message': f'Triggered training for {len(promoted)} group(s)',
    }, 200


@curriculum_bp.route('/curriculum/<id>/:resume_after_failure', methods=['POST'])
def resume_after_failure(id):
    """체크포인트 실패로 pause 된 롤아웃에 사용자 결정을 전달.

    body: { block_id: str, action: 'fallback' | 'abort' | 'next' }
      - 'fallback' (기본): fallback 블록으로 점프 후 plan 계속 진행
      - 'abort' / 'next':  fallback 점프 안 함. _run_group_once 가 다음 블록으로
                           순차 진행 (또는 stop_rollout 호출 시점에 종료).
                           'next' 는 "교정 데이터 기록 후 다음 블록으로 진행"
                           의 의미상 alias. 'abort' 는 X 버튼 등 명시적 닫기.

    교정 텔레옵 → record_episode 흐름에서는 사용자가 누른 버튼에 따라:
      - Done   → 'next'    (다음 블록으로 진행)
      - Throw  → 'fallback' (fallback 블록으로 점프)
      - Stop   → :stop_rollout (전체 롤아웃 중지) — 이 API 가 아닌 별도 호출
    """
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    proc_name = _rollout_process_name(curriculum.id)
    proc = current_app.pm.processes.get(proc_name)
    if proc is None:
        return {'status': 'error', 'message': 'Rollout not running'}, 409
    obj = proc.get('obj') if isinstance(proc, dict) else None
    if obj is None:
        return {'status': 'error', 'message': 'Internal: no task_control'}, 500
    body = request.json or {}
    block_id = body.get('block_id')
    action = body.get('action') or 'fallback'
    if not block_id:
        return {'status': 'error', 'message': 'block_id required'}, 400
    if action not in ('fallback', 'abort', 'next'):
        return {'status': 'error', 'message': f'invalid action {action!r}'}, 400
    # 이중 클라이언트(런처 + 별도 브라우저 등)가 같은 record_episode 종료 이벤트에
    # 반응해 각자 resume 신호를 쏘면, 교정을 안 몬 쪽이 기본값 'fallback' 을 보내
    # 정상 'next' 를 가로채 첫 home 블록으로 점프시키는 버그가 있었다. 두 겹 가드:
    #  (1) awaiting-match: 롤아웃이 지금 이 block_id 의 결정을 기다리는 중이 아니면
    #      stale(이미 소비됐거나 다른 블록) → 무시. 소비 후 늦게 온 신호 차단.
    #  (2) non-override: 이미 명시적 결정(next/abort)이 들어왔으면 'fallback' 으로
    #      덮어쓰지 않는다. 반대로 'fallback' 위에 'next' 는 덮을 수 있다.
    awaiting = obj.get('awaiting_correction')
    if awaiting != block_id:
        return {'status': 'success',
                'message': f'ignored stale signal (not awaiting {block_id!r}, awaiting={awaiting!r})'}, 200
    decisions = obj.setdefault('correction_decisions', {})
    existing = decisions.get(block_id)
    if existing in ('next', 'abort') and action == 'fallback':
        return {'status': 'success',
                'message': f'kept {existing!r}, ignored stale fallback'}, 200
    decisions[block_id] = action
    return {'status': 'success', 'message': f'resume signal sent ({action})'}, 200


@curriculum_bp.route('/curriculum/<id>/:stop_current_block', methods=['POST'])
def stop_current_block(id):
    """현재 진행 중인 체크포인트 블록만 중단 — task_control 의 stop 은 건드리지
    않아 rollout 자체는 살려둔다. ``ctx['stop_current_block']`` 플래그를 set
    하면 curriculum_rollout 의 폴링 루프가 이를 감지해 max_steps 초과와 같은
    실패 처리로 빠져나오고 (curriculum_checkpoint_failed emit), 사용자가 교정
    다이얼로그를 받게 된다.
    """
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    proc_name = _rollout_process_name(curriculum.id)
    proc = current_app.pm.processes.get(proc_name)
    if proc is None:
        return {'status': 'error', 'message': 'Rollout not running'}, 409
    # task_control 자체는 process 의 'obj' (FunctionTask.control) — 거기로 신호.
    # rollout 함수는 ctx 객체를 별도로 갖지만, task_control 과 ctx 가 같은
    # process 안에서 공유돼 있다. 안전한 채널은 task_control 인데, 여기는
    # 평소엔 {'stop': bool} 만 들고 있고 우리가 별도 키를 추가하는 것은
    # task_control 그대로는 안 됨 → process 의 ``kwargs`` 에 ctx 참조가 없으니
    # 가장 간단한 방법은 task_control 에 같이 얹기. curriculum_rollout 에서
    # ``ctx['stop_current_block']`` 대신 task_control 의 키도 같이 본다.
    obj = proc.get('obj') if isinstance(proc, dict) else None
    if obj is None:
        return {'status': 'error', 'message': 'Internal: no task_control'}, 500
    obj['stop_current_block'] = True
    return {'status': 'success', 'message': 'Block stop requested'}, 200


@curriculum_bp.route('/curriculum/<id>/:rollout_status', methods=['GET'])
def rollout_status(id):
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404
    proc_name = _rollout_process_name(curriculum.id)
    is_running = proc_name in current_app.pm.processes
    # 그룹별 현재 stage 의 성공/실패 카운트 요약.
    groups = []
    for group in curriculum.checkpoint_groups:
        stage = group.current_stage
        groups.append({
            'checkpoint_group_id': group.id,
            'name': group.name,
            'current_stage_id': stage.id if stage else None,
            'stage_index': stage.index if stage else None,
            'success_count': stage.success_count if stage else 0,
            'failure_count': stage.failure_count if stage else 0,
            'correction_count': stage.correction_count if stage else 0,
        })
    return {
        'status': 'success',
        'is_running': is_running,
        'curriculum_status': curriculum.status,
        'groups': groups,
    }, 200


# ── 대시보드 (그룹별 stage 수치) ────────────────────────────────────────────

@curriculum_bp.route('/curriculum/<id>/:dashboard', methods=['GET'])
def curriculum_dashboard(id):
    """그룹별 / stage별 / 체크포인트별 수치 집계.

    각 stage: success/failure count + 성공률, 체크포인트별 success/failure/dagger 에피소드 수
    및 성공 에피소드 평균 길이(RolloutResult steps 기준).
    """
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404

    out_groups = []
    for group in curriculum.checkpoint_groups:
        cp_ids = group._get_json_field('checkpoint_ids') or []
        # 기록의 행 단위 = 그룹 멤버 cp 를 쓰는 플랜의 체크포인트 블록들.
        blocks = _group_checkpoint_blocks(curriculum, group)
        block_ids = [b['block_id'] for b in blocks]
        from ...database.models.task_model import Task as TaskModel
        # 체크포인트 → "체크포인트 이름 (워크스페이스 이름)" 라벨 (하위호환 유지).
        cp_labels = {}
        for cp in cp_ids:
            ckpt = CheckpointModel.find(cp)
            name = (ckpt.name if ckpt else None) or f'CP #{cp}'
            ws = TaskModel.find(ckpt.task_id) if (ckpt and ckpt.task_id) else None
            cp_labels[str(cp)] = f'{name} ({ws.name})' if (ws and ws.name) else name
        # 블록 라벨: "<워크스페이스명> · #<cp id>". cp 이름(모델 체크포인트명)은
        # 길고 불명확해서 워크스페이스명 + cp id 조합으로 표기한다.
        block_labels = {}
        for b in blocks:
            ws = TaskModel.find(b['task_id']) if b['task_id'] else None
            ws_name = (ws.name if ws else None) or (b['name'] or f"WS #{b['task_id']}")
            block_labels[b['block_id']] = f"{ws_name} · #{b['checkpoint_id']}"
        stages_out = []
        for stage in group.stages:  # index asc
            # 데이터셋 에피소드 수: (block_id, role) → count
            ep_counts = {}
            for ds in stage.datasets:
                ep_counts[(ds.block_id, ds.role)] = len(ds.episodes or [])
            # 성공 에피소드 평균 길이: RolloutResult(stage) 의 success steps 평균(block별)
            steps_by_block = {}
            for res in RolloutResultModel.all_active().where(RolloutResultModel.stage_id == stage.id):
                for ep in (res._get_json_field('episodes') or []):
                    if ep.get('role') == 'success' and ep.get('steps'):
                        steps_by_block.setdefault(ep.get('block_id'), []).append(int(ep['steps']))
            checkpoints = {}
            for b in blocks:
                bid = b['block_id']
                lens = steps_by_block.get(bid) or []
                checkpoints[bid] = {
                    'checkpoint_id': b['checkpoint_id'],
                    'success_eps': ep_counts.get((bid, 'success'), 0),
                    'failure_eps': ep_counts.get((bid, 'failure'), 0),
                    'dagger_eps': ep_counts.get((bid, 'dagger'), 0),
                    'avg_success_len': round(sum(lens) / len(lens), 1) if lens else 0,
                }
            succ = stage.success_count or 0
            fail = stage.failure_count or 0
            corr = stage.correction_count  # property — dagger 데이터셋 에피소드 합
            # 성공률 분모는 **그룹 단위 rollout 횟수** = success_count + failure_count.
            # correction_count 는 같은 실패 rollout 에 부속된 사용자 시연 수(체크포인트
            # 단위 에피소드 합) 라 분모에 포함하면 한 사건을 두 번 세게 된다.
            denom = succ + fail
            stages_out.append({
                'id': stage.id,
                'index': stage.index,
                'status': stage.status,
                'success_count': succ,
                'failure_count': fail,
                'correction_count': corr,
                'success_rate': round(succ / denom, 4) if denom > 0 else 0,
                'success_criteria': stage._get_json_field('success_criteria') or {},  # block 키
                'checkpoints': checkpoints,  # block 키
            })
        out_groups.append({
            'checkpoint_group_id': group.id,
            'name': group.name,
            'color': group.color,
            'status': group.status or 'collecting',  # collecting | training
            'checkpoint_ids': cp_ids,        # 하위호환
            'cp_labels': cp_labels,          # 하위호환
            'cp_blocks': blocks,             # [{block_id, checkpoint_id, name, task_id}]
            'block_ids': block_ids,
            'block_labels': block_labels,
            'stages': stages_out,
            # 모션 블록별 노이즈 spec (rate/offset) — 프론트 상세 패널에서
            # stage 의 성공률을 곱해 효과적인 ± 범위를 표시.
            'block_noise': group._get_json_field('block_noise') or {},
            'motion_block_ids': group._get_json_field('motion_block_ids') or [],
        })
    return {'status': 'success', 'groups': out_groups}, 200


# ── 초기화 (reset) ─────────────────────────────────────────────────────────

@curriculum_bp.route('/curriculum/<id>/:reset', methods=['POST'])
def reset_curriculum(id):
    """파이프라인 초기화 — 모든 롤아웃/결과/Stage/curriculum 데이터셋 삭제.

    그룹 정의(checkpoint_ids)는 유지하고, 각 그룹의 첫 Stage(index 0)만 빈 상태로 재생성한다.
    """
    curriculum = CurriculumModel.find(id)
    if not curriculum:
        return {'status': 'error', 'message': 'Curriculum not found'}, 404

    proc_name = _rollout_process_name(curriculum.id)
    if proc_name in current_app.pm.processes:
        # 롤아웃이 도는 중이면 409 로 막지 말고 **먼저 정지**한다 — "데이터 초기화"가
        # 항상 동작하도록. 협력적 stop 신호 후 프로세스가 실제로 정리될 때까지 대기.
        import time as _t
        current_app.pm.stop_function(proc_name, wait_timeout=10)
        for _ in range(20):
            if proc_name not in current_app.pm.processes:
                break
            _t.sleep(0.5)
        if proc_name in current_app.pm.processes:
            return {'status': 'error',
                    'message': '롤아웃이 정지되지 않아 초기화할 수 없습니다. 잠시 후 다시 시도하세요.'}, 409
        curriculum.status = CurriculumModel.STATUS_STOPPED

    # 롤아웃/결과 삭제.
    for rollout in RolloutModel.all_active().where(RolloutModel.curriculum_id == curriculum.id):
        for res in rollout.results:
            res.delete_instance()
        rollout.delete_instance()

    # Stage + 데이터셋 삭제 후 각 그룹 첫 Stage 재생성.
    for group in curriculum.checkpoint_groups:
        # 초기화는 첫 Stage(index 0)의 초기 난이도로 되돌린다(승급으로 올라간 최신값이 아니라).
        stages = group.stages  # index asc
        first_stage = stages[0] if stages else None
        seed_noise = first_stage._get_json_field('rollout_noise') if first_stage else {}
        seed_criteria = first_stage._get_json_field('success_criteria') if first_stage else {}
        for stage in stages:
            _delete_dataset_rows(stage.datasets)
            stage.delete_instance()
        # 학습 상태도 초기화(collecting + training_map 비움).
        group.status = CheckpointGroupModel.STATUS_COLLECTING
        group.training_map = {}
        group.save()
        fresh = StageModel.create(
            checkpoint_group_id=group.id,
            index=0,
            status=StageModel.STATUS_ACTIVE,
            rollout_noise=seed_noise,
            success_criteria=seed_criteria,
            success_count=0,
            failure_count=0,
        )
        _create_stage_datasets(fresh, group)

    curriculum.status = CurriculumModel.STATUS_IDLE
    curriculum.save()
    return {'status': 'success', 'message': 'Curriculum reset'}, 200

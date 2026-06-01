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

def _create_stage_datasets_for(stage, group, checkpoint_ids):
    """주어진 체크포인트들에 대해 success/failure/dagger 데이터셋 3개씩 생성.

    task_id 는 체크포인트의 워크스페이스로 설정(워크스페이스 종속) + stage_id 로 Stage 종속.
    origin='curriculum' 으로 표식하여 워크스페이스 UI 편집/병합/삭제를 차단한다.
    """
    for cp_id in checkpoint_ids:
        checkpoint = CheckpointModel.find(cp_id)
        task_id = checkpoint.task_id if checkpoint else None
        for role in DATASET_ROLES:
            ds = DatasetModel.create(
                name=f'cp{cp_id}_stage{stage.index}_{role}',
                task_id=task_id,
                origin='curriculum',
                stage_id=stage.id,
                checkpoint_group_id=group.id,
                checkpoint_id=cp_id,
                role=role,
            )
            os.makedirs(os.path.join(DATASET_DIR, str(ds.id)), exist_ok=True)


def _create_stage_datasets(stage, group):
    """Stage 생성 시 그룹 내 전 체크포인트에 대해 데이터셋 생성."""
    checkpoint_ids = group._get_json_field('checkpoint_ids') or []
    if not isinstance(checkpoint_ids, list):
        checkpoint_ids = []
    _create_stage_datasets_for(stage, group, checkpoint_ids)


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
    group = _group_of_checkpoint(curriculum, cp_id)
    if group is None:
        return {'status': 'error', 'message': 'Checkpoint not in any group'}, 404
    settings = group._get_json_field('checkpoint_settings') or {}
    settings[str(cp_id)] = {
        'base_dataset_ids': data.get('base_dataset_ids', []),
        'train_settings': data.get('train_settings', {}),
        'initial_max_steps': data.get('initial_max_steps', 600),
        'length_limit_rate': data.get('length_limit_rate', 1.5),
        'success_threshold': data.get('success_threshold', 0.5),
    }
    group.checkpoint_settings = settings
    group.save()

    # 현재 stage 의 success_criteria 를 초기 길이 제한/임계값으로 시드.
    stage = group.current_stage
    if stage is not None:
        crit = stage._get_json_field('success_criteria') or {}
        crit[str(cp_id)] = {
            'max_steps': data.get('initial_max_steps', 600),
            'success_threshold': data.get('success_threshold', 0.5),
        }
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
    existing_cp = {ds.checkpoint_id for ds in stage.datasets}
    member_cp = set(group._get_json_field('checkpoint_ids') or [])
    missing = member_cp - existing_cp
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
    obj.setdefault('correction_decisions', {})[block_id] = action
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
        # 체크포인트 → "체크포인트 이름 (워크스페이스 이름)" 라벨.
        from ...database.models.task_model import Task as TaskModel
        cp_labels = {}
        for cp in cp_ids:
            ckpt = CheckpointModel.find(cp)
            name = (ckpt.name if ckpt else None) or f'CP #{cp}'
            ws = TaskModel.find(ckpt.task_id) if (ckpt and ckpt.task_id) else None
            cp_labels[str(cp)] = f'{name} ({ws.name})' if (ws and ws.name) else name
        stages_out = []
        for stage in group.stages:  # index asc
            # 데이터셋 에피소드 수: (checkpoint_id, role) → count
            ep_counts = {}
            for ds in stage.datasets:
                ep_counts[(ds.checkpoint_id, ds.role)] = len(ds.episodes or [])
            # 성공 에피소드 평균 길이: RolloutResult(stage) 의 success steps 평균(cp별)
            steps_by_cp = {}
            for res in RolloutResultModel.all_active().where(RolloutResultModel.stage_id == stage.id):
                for ep in (res._get_json_field('episodes') or []):
                    if ep.get('role') == 'success' and ep.get('steps'):
                        steps_by_cp.setdefault(ep.get('checkpoint_id'), []).append(int(ep['steps']))
            checkpoints = {}
            for cp in cp_ids:
                lens = steps_by_cp.get(cp) or steps_by_cp.get(str(cp)) or []
                checkpoints[str(cp)] = {
                    'success_eps': ep_counts.get((cp, 'success'), 0),
                    'failure_eps': ep_counts.get((cp, 'failure'), 0),
                    'dagger_eps': ep_counts.get((cp, 'dagger'), 0),
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
                'index': stage.index,
                'status': stage.status,
                'success_count': succ,
                'failure_count': fail,
                'correction_count': corr,
                'success_rate': round(succ / denom, 4) if denom > 0 else 0,
                'success_criteria': stage._get_json_field('success_criteria') or {},
                'checkpoints': checkpoints,
            })
        out_groups.append({
            'checkpoint_group_id': group.id,
            'name': group.name,
            'color': group.color,
            'status': group.status or 'collecting',  # collecting | training
            'checkpoint_ids': cp_ids,
            'cp_labels': cp_labels,
            'stages': stages_out,
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
        return {'status': 'error', 'message': 'Stop the rollout before resetting'}, 409

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

import json

from flask import Blueprint, request, current_app

from ...database.models.planner_model import Planner as PlannerModel
from ...database.models.task_model import Task as TaskModel
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
from ..process.planner_run import planner_run


planner_bp = Blueprint('planner_bp', __name__)


PLANNER_RUN_PROCESS_NAME = 'planner_run'


BLOCK_CONFIGS = {
    'joint_position': {
        'label': 'Joint Position',
        'icon': 'precision_manufacturing',
        'color': 'blue',
        'keys': ['workspace_id', 'positions', 'duration', 'name'],
    },
    'checkpoint': {
        'label': 'Checkpoint',
        'icon': 'psychology',
        'color': 'purple',
        'keys': ['workspace_id', 'checkpoint_id', 'checkpoint_name', 'duration', 'until_done', 'done_threshold', 'hz', 're_inference_steps', 'temporal_ensemble_coeff', 'move_homepose', 'move_homepose_duration', 'move_homepose_settle_sec', 'name'],
    },
    'timesleep': {
        'label': 'Time Sleep',
        'icon': 'hourglass_empty',
        'color': 'orange',
        'keys': ['duration', 'name'],
    },
    'sync': {
        'label': 'Sync',
        'icon': 'sync',
        'color': 'teal',
        'keys': ['sync_id', 'name'],
    },
}


@planner_bp.route('/planner/block_configs', methods=['GET'])
def get_block_configs():
    return {
        'status': 'success',
        'block_configs': BLOCK_CONFIGS,
    }, 200


# ---------------------------------------------------------------------------
# Group computation: connected components by shared-robot edges
# ---------------------------------------------------------------------------

def _group_id_for(workspace_ids):
    """Deterministic group identifier from a set of workspace IDs."""
    return '+'.join(str(w) for w in sorted(workspace_ids))


def _compute_groups(task_ids):
    """Partition workspaces into groups whose assemblies share at least one robot.

    Returns ``[{ id, workspace_ids: [...] }, ...]`` ordered by smallest workspace_id.
    Workspaces that no longer exist (or have no assembly) get their own singleton
    group so the UI still shows them.
    """
    if not task_ids:
        return []

    workspaces = []
    for ws_id in task_ids:
        ws = TaskModel.find(ws_id)
        if ws is None:
            continue
        ws_dict = ws.to_dict()
        robots = (ws_dict.get('assembly') or {}).get('robots') or []
        robot_ids = {int(r['id']) for r in robots if r.get('id') is not None}
        workspaces.append({'id': ws_dict['id'], 'robots': robot_ids})

    # Union-find on workspace indices, joined when robot sets intersect.
    parent = list(range(len(workspaces)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for i in range(len(workspaces)):
        for j in range(i + 1, len(workspaces)):
            if workspaces[i]['robots'] & workspaces[j]['robots']:
                union(i, j)

    buckets = {}
    for idx, ws in enumerate(workspaces):
        root = find(idx)
        buckets.setdefault(root, []).append(ws['id'])

    groups = []
    for ws_ids in buckets.values():
        ws_ids_sorted = sorted(ws_ids)
        groups.append({
            'id': _group_id_for(ws_ids_sorted),
            'workspace_ids': ws_ids_sorted,
        })
    groups.sort(key=lambda g: g['workspace_ids'][0] if g['workspace_ids'] else 0)
    return groups


def _rebalance_plans(planner):
    """Recompute group structure from the planner's current task_ids and redistribute
    existing blocks. Blocks whose workspace_id no longer maps to any group are dropped.

    Idempotent — safe to call after any task_ids/plans/plan write.
    Persists the result if it changes.
    """
    task_ids = planner._get_json_field('task_ids') or []
    groups = _compute_groups(task_ids)

    # Collect blocks from existing storage (new `plans`, fall back to legacy `plan`).
    existing_plans = planner._get_json_field('plans')
    legacy_plan = planner._get_json_field('plan') or []

    # (block, source_group_id) pairs so sync blocks can stay in their original
    # group even though they have no workspace_id.
    all_blocks = []
    if isinstance(existing_plans, list) and existing_plans:
        for grp in existing_plans:
            for b in grp.get('blocks') or []:
                all_blocks.append((b, grp.get('id')))
    elif isinstance(legacy_plan, list):
        all_blocks = [(b, None) for b in legacy_plan]

    # Index workspace_id -> group_id for redistribution.
    ws_to_group = {}
    for grp in groups:
        for ws_id in grp['workspace_ids']:
            ws_to_group[ws_id] = grp['id']

    new_plans = [{'id': g['id'], 'workspace_ids': g['workspace_ids'], 'blocks': []}
                 for g in groups]
    new_plans_by_id = {p['id']: p for p in new_plans}

    # timesleep has no workspace — pin to first group (or skip if no groups left).
    fallback_group_id = new_plans[0]['id'] if new_plans else None

    for block, source_group_id in all_blocks:
        ws_id = block.get('workspace_id')
        if ws_id is not None:
            group_id = ws_to_group.get(ws_id)
            if group_id is None:
                # workspace was removed from planner — drop the block
                continue
            new_plans_by_id[group_id]['blocks'].append(block)
        else:
            # Sync blocks were placed in a specific group by the user; keep
            # them there if the group still exists. Other workspace-less
            # blocks (timesleep) fall back to the first group.
            target = source_group_id if (block.get('type') == 'sync' and source_group_id in new_plans_by_id) else fallback_group_id
            if target is not None:
                new_plans_by_id[target]['blocks'].append(block)

    planner.plans = new_plans
    planner.plan = []  # legacy field cleared once plans is canonical
    planner.save()
    return new_plans


def _ensure_plans_loaded(planner):
    """Return planner.plans, lazily migrating from legacy `plan` field on first read."""
    plans = planner._get_json_field('plans')
    if isinstance(plans, list) and plans:
        return plans
    # First read after migration — rebalance from whatever's there.
    return _rebalance_plans(planner)


@planner_bp.route('/planners', methods=['GET'])
def get_planners():
    planners = PlannerModel.select().where(PlannerModel.deleted_at.is_null())
    result = []
    for p in planners:
        # Lazy migration so the UI always sees `plans`.
        _ensure_plans_loaded(p)
        result.append(p.to_dict())
    return {
        'status': 'success',
        'planners': result,
    }, 200


@planner_bp.route('/planner', methods=['POST'])
def create_planner():
    data = request.json or {}
    if not data.get('name'):
        return {'status': 'error', 'message': 'Name is a required field.'}, 400

    planner = PlannerModel.create(
        name=data.get('name'),
        task_ids=data.get('task_ids', []),
        plans=data.get('plans', []),
    )
    _rebalance_plans(planner)

    return {
        'status': 'success',
        'message': 'Planner Created',
        'id': planner.id,
    }, 201


@planner_bp.route('/planner/<id>', methods=['PUT'])
def update_planner(id):
    planner = PlannerModel.find(id)
    if not planner:
        return {'status': 'error', 'message': 'Planner not found'}, 404

    data = request.json or {}
    structural_change = False  # task_ids 변경 시에만 그룹 재계산

    if 'name' in data:
        planner.name = data['name']

    if 'workspace_ids' in data:
        planner.task_ids = json.dumps(data['workspace_ids'])
        structural_change = True
    elif 'task_ids' in data:
        planner.task_ids = json.dumps(data['task_ids'])
        structural_change = True

    # 새 클라이언트는 plans를 통째로 보냄. 한 그룹의 블록만 갱신하려면 group_id+blocks.
    if 'plans' in data:
        planner.plans = json.dumps(data['plans'])
    elif 'group_id' in data and 'blocks' in data:
        existing = planner._get_json_field('plans') or []
        for grp in existing:
            if grp.get('id') == data['group_id']:
                grp['blocks'] = data['blocks']
                break
        planner.plans = json.dumps(existing)
    elif 'plan' in data:
        # 레거시 호환: 단일 flat plan을 보내면 다음 rebalance가 그룹화.
        planner.plan = json.dumps(data['plan'])
        planner.plans = None  # 강제 재계산
        structural_change = True

    if 'blocks' in data and 'group_id' not in data:
        planner.blocks = json.dumps(data['blocks'])

    planner.save()

    if structural_change:
        _rebalance_plans(planner)

    return {
        'status': 'success',
        'message': 'Planner Updated',
    }, 200


@planner_bp.route('/planner/<id>', methods=['DELETE'])
def delete_planner(id):
    planner = PlannerModel.find(id)
    if not planner:
        return {'status': 'error', 'message': 'Planner not found'}, 404

    planner.soft_delete()

    return {
        'status': 'success',
        'message': 'Planner Deleted',
    }, 200


def _validate_plan_blocks(blocks, workspaces_by_id, agents, prefix_offset=0):
    """Pre-flight checks for a sequence of blocks. ``prefix_offset`` lets callers
    number errors continuously across multiple groups when validating 전체 재생.
    """
    errors = []
    for index, block in enumerate(blocks):
        prefix = f"Block {index + 1 + prefix_offset} ({block.get('name') or block.get('type')})"
        btype = block.get('type')

        if btype == 'joint_position':
            ws = workspaces_by_id.get(block.get('workspace_id'))
            if ws is None:
                errors.append(f"{prefix}: workspace not found")
                continue
            for robot in (ws.get('assembly') or {}).get('robots') or []:
                if int(robot['id']) not in agents:
                    errors.append(f"{prefix}: robot '{robot.get('name')}' is not running")

        elif btype == 'checkpoint':
            ws = workspaces_by_id.get(block.get('workspace_id'))
            if ws is None:
                errors.append(f"{prefix}: workspace not found")
                continue
            ckpt = CheckpointModel.find(block.get('checkpoint_id'))
            if ckpt is None:
                errors.append(f"{prefix}: checkpoint not found")
                continue
            if ckpt.policy_id is None:
                errors.append(f"{prefix}: checkpoint has no associated policy")
            for robot in (ws.get('assembly') or {}).get('robots') or []:
                if int(robot['id']) not in agents:
                    errors.append(f"{prefix}: robot '{robot.get('name')}' is not running")
            if block.get('until_done'):
                train_settings = (ckpt.to_dict() or {}).get('train_settings') or {}
                if not train_settings.get('has_succeed'):
                    errors.append(f"{prefix}: until_done requires a checkpoint trained with has_succeed")
                try:
                    thr = float(block.get('done_threshold') if block.get('done_threshold') is not None else 0.5)
                    if not (0.0 < thr < 1.0):
                        errors.append(f"{prefix}: done_threshold must be between 0 and 1")
                except (TypeError, ValueError):
                    errors.append(f"{prefix}: done_threshold is invalid")

        elif btype == 'timesleep':
            try:
                if float(block.get('duration') or 0) <= 0:
                    errors.append(f"{prefix}: duration must be > 0")
            except (TypeError, ValueError):
                errors.append(f"{prefix}: duration is invalid")

        elif btype == 'sync':
            sync_id = (block.get('sync_id') or '').strip()
            if not sync_id:
                errors.append(f"{prefix}: sync_id is required")

        else:
            errors.append(f"{prefix}: unknown block type '{btype}'")

    return errors


def _load_workspaces_for_blocks(blocks):
    """Load every workspace (task) referenced by any block in ``blocks``."""
    workspace_ids = {b.get('workspace_id') for b in blocks if b.get('workspace_id') is not None}
    workspaces = []
    for ws_id in workspace_ids:
        ws = TaskModel.find(ws_id)
        if ws is not None:
            workspaces.append(ws.to_dict())
    return workspaces


@planner_bp.route('/planner/<id>/:start_run', methods=['POST'])
def start_planner_run(id):
    planner = PlannerModel.find(id)
    if not planner:
        return {'status': 'error', 'message': 'Planner not found'}, 404

    if PLANNER_RUN_PROCESS_NAME in current_app.pm.processes:
        return {'status': 'error', 'message': 'A planner run is already in progress'}, 409

    plans = _ensure_plans_loaded(planner)
    if not plans:
        return {'status': 'error', 'message': 'Planner has no groups to run'}, 400

    body = request.json or {}
    group_id = body.get('group_id')  # None → 전체 재생, 값 있으면 해당 그룹만

    # 실행할 그룹들을 결정.
    if group_id is None:
        run_groups = [g for g in plans if g.get('blocks')]
    else:
        run_groups = [g for g in plans if g.get('id') == group_id]
        if not run_groups:
            return {'status': 'error', 'message': f'Group {group_id} not found'}, 404
        if not run_groups[0].get('blocks'):
            return {'status': 'error', 'message': 'Group has no blocks to run'}, 400

    if not run_groups:
        return {'status': 'error', 'message': 'No blocks to run in any group'}, 400

    # 모든 블록이 참조하는 워크스페이스를 한 번에 로드.
    all_blocks = [b for g in run_groups for b in g['blocks']]
    workspaces = _load_workspaces_for_blocks(all_blocks)
    workspaces_by_id = {ws['id']: ws for ws in workspaces}
    agents = getattr(current_app, 'agents', {}) or {}

    errors = []
    offset = 0
    for grp in run_groups:
        errors.extend(_validate_plan_blocks(grp['blocks'], workspaces_by_id, agents, prefix_offset=offset))
        offset += len(grp['blocks'])
    if errors:
        return {
            'status': 'error',
            'message': 'Planner validation failed',
            'errors': errors,
        }, 400

    try:
        repeat_count = int(body.get('repeat_count', 1))
    except (TypeError, ValueError):
        repeat_count = 1

    current_app.pm.start_function(
        name=PLANNER_RUN_PROCESS_NAME,
        func=planner_run,
        groups=run_groups,
        workspaces=workspaces,
        app=current_app._get_current_object(),
        socketio_instance=current_app.pm.socketio,
        repeat_count=repeat_count,
    )

    return {
        'status': 'success',
        'message': 'Planner run started',
        'group_ids': [g['id'] for g in run_groups],
    }, 200


@planner_bp.route('/planner/<id>/:stop_run', methods=['POST'])
def stop_planner_run(id):
    if PLANNER_RUN_PROCESS_NAME not in current_app.pm.processes:
        return {'status': 'success', 'message': 'No planner run in progress'}, 200
    current_app.pm.stop_function(PLANNER_RUN_PROCESS_NAME)
    return {'status': 'success', 'message': 'Planner run stopped'}, 200


@planner_bp.route('/planner/<id>/:run_status', methods=['GET'])
def planner_run_status(id):
    """Lightweight status probe so the UI can recover state on reload."""
    is_running = PLANNER_RUN_PROCESS_NAME in current_app.pm.processes
    return {
        'status': 'success',
        'is_running': is_running,
    }, 200

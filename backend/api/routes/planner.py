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
        'keys': ['workspace_id', 'checkpoint_id', 'checkpoint_name', 'duration', 'hz', 're_inference_steps', 'temporal_ensemble_coeff', 'name'],
    },
    'timesleep': {
        'label': 'Time Sleep',
        'icon': 'hourglass_empty',
        'color': 'orange',
        'keys': ['duration', 'name'],
    },
}


@planner_bp.route('/planner/block_configs', methods=['GET'])
def get_block_configs():
    return {
        'status': 'success',
        'block_configs': BLOCK_CONFIGS,
    }, 200


@planner_bp.route('/planners', methods=['GET'])
def get_planners():
    planners = PlannerModel.select().where(PlannerModel.deleted_at.is_null())
    return {
        'status': 'success',
        'planners': [p.to_dict() for p in planners],
    }, 200


@planner_bp.route('/planner', methods=['POST'])
def create_planner():
    data = request.json or {}
    if not data.get('name'):
        return {'status': 'error', 'message': 'Name is a required field.'}, 400

    planner = PlannerModel.create(
        name=data.get('name'),
        task_ids=data.get('task_ids', []),
        plan=data.get('plan', []),
    )

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

    if 'name' in data:
        planner.name = data['name']
    if 'workspace_ids' in data:
        planner.task_ids = json.dumps(data['workspace_ids'])
    elif 'task_ids' in data:
        planner.task_ids = json.dumps(data['task_ids'])
    if 'plan' in data:
        planner.plan = json.dumps(data['plan'])
    if 'blocks' in data:
        planner.blocks = json.dumps(data['blocks'])

    planner.save()

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


def _validate_plan(plan, workspaces_by_id, agents):
    """Pre-flight checks before launching a planner run.

    Returns a list of error strings; empty list means OK.
    Catches issues that would otherwise surface mid-run as exceptions:
    missing workspaces, agents not running, missing checkpoints, bad durations.
    """
    errors = []
    for index, block in enumerate(plan):
        prefix = f"Block {index + 1} ({block.get('name') or block.get('type')})"
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

        elif btype == 'timesleep':
            try:
                if float(block.get('duration') or 0) <= 0:
                    errors.append(f"{prefix}: duration must be > 0")
            except (TypeError, ValueError):
                errors.append(f"{prefix}: duration is invalid")

        else:
            errors.append(f"{prefix}: unknown block type '{btype}'")

    return errors


def _load_workspaces_for_plan(plan):
    """Load every workspace (task) referenced by the plan, as dicts."""
    workspace_ids = {block.get('workspace_id') for block in plan if block.get('workspace_id') is not None}
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

    plan = planner._get_json_field('plan') or []
    if not plan:
        return {'status': 'error', 'message': 'Planner has no blocks to run'}, 400

    workspaces = _load_workspaces_for_plan(plan)
    workspaces_by_id = {ws['id']: ws for ws in workspaces}
    agents = getattr(current_app, 'agents', {}) or {}

    errors = _validate_plan(plan, workspaces_by_id, agents)
    if errors:
        return {
            'status': 'error',
            'message': 'Planner validation failed',
            'errors': errors,
        }, 400

    body = request.json or {}
    # repeat_count: 양수면 그 횟수만큼 반복, <=0이면 무한 반복 (정지 누를 때까지).
    try:
        repeat_count = int(body.get('repeat_count', 1))
    except (TypeError, ValueError):
        repeat_count = 1

    current_app.pm.start_function(
        name=PLANNER_RUN_PROCESS_NAME,
        func=planner_run,
        plan=plan,
        workspaces=workspaces,
        app=current_app._get_current_object(),
        socketio_instance=current_app.pm.socketio,
        repeat_count=repeat_count,
    )

    return {
        'status': 'success',
        'message': 'Planner run started',
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

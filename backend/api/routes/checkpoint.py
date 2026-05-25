from flask import Blueprint, request, current_app, send_file
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
from ...configs.global_configs import get_checkpoint_dir
import os
import shutil

from ..process.checkpoint_test import checkpoint_test
from ..process.failure_detection import failure_detection
from ..process.generate_ood_features import generate_ood_features
from ..process.export_checkpoint import bundle_checkpoint_zip
from ..process.vision_map import compute_vision_map, compute_vision_map_episode_stream
from ...configs.global_configs import resolve_checkpoint_dir


checkpoint_bp = Blueprint('checkpoint_bp', __name__)

@checkpoint_bp.route('/checkpoints', methods=['GET'])
def get_checkpoints():
    query_str = request.args.get('where', None)
    query_parts = query_str.split('|') if query_str else []

    q = CheckpointModel.select().where(CheckpointModel.deleted_at.is_null())
    for part in query_parts:
        qarr = part.split(',')
        field = getattr(CheckpointModel, qarr[0], None)
        if field:
            op = qarr[1]
            val = qarr[2]
            if op == '=':
                q = q.where(field == val)
            elif op == '!=':
                q = q.where(field != val)
            elif op == '>':
                q = q.where(field > val)
            elif op == '<':
                q = q.where(field < val)

    checkpoints = [checkpoint.to_dict() for checkpoint in q]
    return {
        'status': 'success', 'checkpoints': checkpoints}, 200


@checkpoint_bp.route('/checkpoints/<folder_name>', methods=['GET'])
def get_checkpoint_files(id):
    folder_path = resolve_checkpoint_dir(id)
    if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
        return {'status': 'error', 'message': 'Folder not found'}, 404

    files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    return {'status': 'success', 'files': files}, 200


@checkpoint_bp.route('/checkpoint', methods=['POST'])
def create_checkpoint():
    data = request.json
    new_checkpoint = CheckpointModel.create(
        name=data.get('name'),
        task_id=data.get('task_id'),
        policy_id=data.get('policy_id'),
        dataset_info=data.get('dataset_info', []),
        num_epochs=data.get('num_epochs'),
        batch_size=data.get('batch_size'),
    )
    return {'status': 'success', 'message': 'Checkpoint Created', 'id': new_checkpoint.id}, 200


@checkpoint_bp.route('/checkpoint/<id>', methods=['GET'])
def get_checkpoint(id):
    """Single checkpoint dict — TrainPage가 enqueue 직후 다이얼로그를 자동
    열기 위해 사용."""
    ckpt = CheckpointModel.find(id)
    if ckpt is None:
        return {'status': 'error', 'message': 'Checkpoint not found'}, 404
    return {'status': 'success', 'checkpoint': ckpt.to_dict()}, 200


@checkpoint_bp.route('/checkpoint/<id>/:check_create_successed', methods=['GET'])
def check_create_successed(id):
    checkpoint = CheckpointModel.find(id)
    if not checkpoint:
        return {'check_create_successed': False, 'message': 'Checkpoint creation failed'}, 200

    folder_path = resolve_checkpoint_dir(checkpoint.id)
    if os.path.exists(folder_path) and os.path.isdir(folder_path):
        return {'check_create_successed': True, 'message': 'Checkpoint created successfully'}, 200
    else:
        return {'check_create_successed': False, 'message': 'Checkpoint creation failed'}, 200


@checkpoint_bp.route('/checkpoint/<id>/:start_test', methods=['POST'])
def start_test(id):
    data = request.json
    checkpoint = CheckpointModel.find(id).to_dict()
    agents = [current_app.agents[agent_id] for agent_id in data.get('robot_ids', [])]
    current_app.pm.start_function(
        func=checkpoint_test,
        node=current_app.node,
        checkpoint=checkpoint,
        task=data.get('task'),
        policy_obj=data.get('policy'),
        agents=agents,
        sensors=data.get('sensors'),
        max_timesteps=data.get('timesteps', 100),
        socketio_instance=current_app.pm.socketio,
        name=f"checkpoint_test",
        move_homepose=data.get('move_homepose', False),
        move_homepose_duration=float(data.get('move_homepose_duration') or 5.0),
        move_homepose_settle_sec=float(data.get('move_homepose_settle_sec') or 0.0),
        hz=data.get('hz', 10),
        re_inference_steps=data.get('re_inference_steps', 1),
        temporal_ensemble_coeff=data.get('temporal_ensemble_coeff', 0.01),
        action_type=data.get('action_type', None),
        # Optional natural-language prompt for VLA policies (PI0.5). If omitted, falls
        # back to task.name at the call site. Empty string is treated as "use task.name".
        language_instruction=data.get('language_instruction', None),
        # Planner feature (origin/integration_2 추가): inference 시 episode_len 별도 지정.
        inference_episode_len=data.get('inference_episode_len', None),
    )

    return {'status': 'success', 'message': 'Checkpoint test started'}, 200

@checkpoint_bp.route('/checkpoint/<id>/:stop_test', methods=['POST'])
def stop_test(id):
    current_app.pm.stop_function(
        name=f"checkpoint_test",
    )
    return {'status': 'success', 'message': 'Checkpoint test stopped'}, 200


@checkpoint_bp.route('/checkpoint/<id>/:start_failure_detection', methods=['POST'])
def start_failure_detection(id):
    data = request.json
    task = data.get('task')
    robots = data.get('robots')
    sensors = data.get('sensors')
    checkpoint = CheckpointModel.find(id)
    if checkpoint:
        current_app.pm.start_function(
            func=failure_detection,
            node=current_app.node,
            checkpoint=checkpoint,
            robots=robots,
            sensors=sensors,
            task_obj=task,
            name=f"failure_detection",
            socketio_instance=current_app.pm.socketio,
    )
    return {'status': 'success', 'message': 'Failure detection started'}, 200

@checkpoint_bp.route('/checkpoint/<id>/:stop_failure_detection', methods=['POST'])
def stop_failure_detection(id):
    current_app.pm.stop_function(
        name=f"failure_detection",
    )
    return {'status': 'success', 'message': 'Failure detection stopped'}, 200


@checkpoint_bp.route('/checkpoint/<id>/:export', methods=['POST', 'GET'])
def export_checkpoint(id):
    """Bundle checkpoint files + standalone inference code into a zip download."""
    checkpoint = CheckpointModel.find(id)
    if not checkpoint:
        return {'status': 'error', 'message': 'Checkpoint not found'}, 404
    if not checkpoint.policy:
        return {'status': 'error', 'message': 'Checkpoint has no associated policy'}, 400
    if not checkpoint.task:
        return {'status': 'error', 'message': 'Checkpoint has no associated task'}, 400

    try:
        buf, filename = bundle_checkpoint_zip(
            checkpoint=checkpoint.to_dict(),
            task=checkpoint.task.to_dict(),
            policy=checkpoint.policy.to_dict(),
        )
    except FileNotFoundError as e:
        return {'status': 'error', 'message': str(e)}, 404
    except Exception as e:
        import traceback
        traceback.print_exc()
        return {'status': 'error', 'message': f'Export failed: {e}'}, 500

    return send_file(
        buf,
        mimetype='application/zip',
        as_attachment=True,
        download_name=filename,
    )


@checkpoint_bp.route('/checkpoint/<id>/:generate_ood_features', methods=['POST'])
def generate_ood(id):
    checkpoint = CheckpointModel.find(id)
    if not checkpoint:
        return {'status': 'error', 'message': 'Checkpoint not found'}, 404

    checkpoint_dict = checkpoint.to_dict()
    policy = checkpoint.policy.to_dict()
    task = checkpoint.task.to_dict()

    current_app.pm.start_function(
        func=generate_ood_features,
        checkpoint=checkpoint_dict,
        policy_obj=policy,
        task=task,
        name='generate_ood_features',
    )
    return {'status': 'success', 'message': 'OOD feature generation started'}, 200


@checkpoint_bp.route('/checkpoint/<id>/:vision_map', methods=['POST'])
def vision_map(id):
    """Compute a per-camera saliency heatmap for a single dataset frame.

    Body: { dataset_id, episode_idx, frame_idx, method ('attention'|'gradcam') }
    Returns: { heatmaps: { sensor_name: 'data:image/png;base64,...' } }

    Used by the WorkspacePage Vision Map overlay — frontend fetches one heatmap
    per visible frame (cached on the policy side so the model is loaded once).
    """
    checkpoint = CheckpointModel.find(id)
    if checkpoint is None:
        return {'status': 'error', 'message': 'Checkpoint not found'}, 404
    if checkpoint.policy is None:
        return {'status': 'error', 'message': 'Checkpoint has no associated policy'}, 400

    data = request.json or {}
    try:
        dataset_id = str(data['dataset_id'])
        episode_idx = int(data['episode_idx'])
        frame_idx = int(data['frame_idx'])
    except (KeyError, TypeError, ValueError) as e:
        return {'status': 'error', 'message': f'Invalid request: {e}'}, 400
    method = data.get('method', 'attention')

    try:
        heatmaps = compute_vision_map(
            checkpoint=checkpoint.to_dict(),
            policy_obj=checkpoint.policy.to_dict(),
            dataset_id=dataset_id,
            episode_idx=episode_idx,
            frame_idx=frame_idx,
            method=method,
        )
    except FileNotFoundError as e:
        return {'status': 'error', 'message': str(e)}, 404
    except Exception as e:
        import traceback; traceback.print_exc()
        return {'status': 'error', 'message': f'Vision map failed: {e}'}, 500

    return {'status': 'success', 'heatmaps': heatmaps}, 200


@checkpoint_bp.route('/checkpoint/<id>/:vision_map_episode_start', methods=['POST'])
def vision_map_episode_start(id):
    """Kick off precompute of vision-map heatmaps for an entire episode.

    Body: { dataset_id, episode_idx, method, session_id? }
    Returns: { session_id }

    Frontend listens for vision_map_episode_* socket events and caches the
    per-frame heatmaps to overlay during playback. A new call cancels any
    in-flight precompute (single global slot — only one episode at a time).
    """
    checkpoint = CheckpointModel.find(id)
    if checkpoint is None:
        return {'status': 'error', 'message': 'Checkpoint not found'}, 404
    if checkpoint.policy is None:
        return {'status': 'error', 'message': 'Checkpoint has no associated policy'}, 400

    data = request.json or {}
    try:
        dataset_id = str(data['dataset_id'])
        episode_idx = int(data['episode_idx'])
    except (KeyError, TypeError, ValueError) as e:
        return {'status': 'error', 'message': f'Invalid request: {e}'}, 400
    method = data.get('method', 'attention')
    # Frontend provides a fresh session_id per request so it can ignore
    # straggler events from a prior request that was cancelled.
    session_id = str(data.get('session_id') or f'{id}-{dataset_id}-{episode_idx}-{method}')

    # Cancel any prior episode computation before starting a new one. wait so
    # the prior worker has a chance to release its GPU/policy handles.
    current_app.pm.stop_function('vision_map_episode', wait_timeout=2.0)

    current_app.pm.start_function(
        name='vision_map_episode',
        func=compute_vision_map_episode_stream,
        checkpoint=checkpoint.to_dict(),
        policy_obj=checkpoint.policy.to_dict(),
        dataset_id=dataset_id,
        episode_idx=episode_idx,
        method=method,
        socketio_instance=current_app.pm.socketio,
        session_id=session_id,
    )
    return {'status': 'success', 'session_id': session_id}, 200


@checkpoint_bp.route('/checkpoint/<id>/:vision_map_episode_stop', methods=['POST'])
def vision_map_episode_stop(id):
    """Cancel in-flight episode precompute. Idempotent."""
    current_app.pm.stop_function('vision_map_episode')
    return {'status': 'success'}, 200


@checkpoint_bp.route('/checkpoint/<id>/:set_inference_vision_map', methods=['POST'])
def set_inference_vision_map(id):
    """Live-toggle the vision-map method on a running inference (checkpoint_test).

    Body: { method: 'attention' | 'gradcam' | null }
    The inference loop reads ``task_control['vision_map_method']`` each step and
    emits an ``inference_vision_map`` socket event when the method is set.
    """
    task = current_app.pm._get('checkpoint_test')
    if task is None:
        return {'status': 'error', 'message': 'No inference running'}, 400

    data = request.json or {}
    method = data.get('method')
    if method not in ('attention', 'gradcam', None):
        return {'status': 'error', 'message': f'Invalid method: {method}'}, 400
    task.control['vision_map_method'] = method
    return {'status': 'success', 'method': method}, 200


@checkpoint_bp.route('/checkpoint/<id>', methods=['PUT'])
def update_checkpoint(id):
    data = request.json
    checkpoint = CheckpointModel.find(id)
    if not checkpoint:
        return {'status': 'error', 'message': 'Checkpoint not found'}, 404

    checkpoint.name = data.get('name', checkpoint.name)
    checkpoint.save()

    return {'status': 'success', 'message': 'Checkpoint Updated'}, 200


@checkpoint_bp.route('/checkpoint/<id>', methods=['DELETE'])
def delete_checkpoint(id):
    checkpoint = CheckpointModel.find(id)
    if not checkpoint:
        return {'status': 'error', 'message': 'Checkpoint not found'}, 404

    folder_path = resolve_checkpoint_dir(checkpoint.id)
    if os.path.exists(folder_path) and os.path.isdir(folder_path):
        shutil.rmtree(folder_path)

    checkpoint.delete_instance()
    return {'status': 'success', 'message': 'Checkpoint Deleted'}, 200

import json

from flask import Blueprint, request, current_app, send_file

from ...database.models.planner_model import Planner as PlannerModel
from ...database.models.task_model import Task as TaskModel
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
from ..process.planner_run import planner_run
from ..process.export_planner import bundle_planner_zip


planner_bp = Blueprint('planner_bp', __name__)


PLANNER_RUN_PROCESS_NAME = 'planner_run'


BLOCK_CONFIGS = {
    'joint_position': {
        'label': 'Joint Position',
        'icon': 'precision_manufacturing',
        'color': 'blue',
        'keys': ['workspace_id', 'positions', 'duration', 'name'],
    },
    'move_relative_ee': {
        # Arm 은 현재 EE 기준 6-DOF 상대 이동 (dx, dy, dz, drx, dry, drz).
        # Tool (gripper 등 IK 없는 agent) 은 절대 joint 위치 — 두 dict 가
        # 같이 들어간다: ``deltas`` (arm 별) + ``tool_positions`` (tool 별).
        'label': 'Move Relative EE',
        'icon': 'open_with',
        'color': 'cyan',
        'keys': ['workspace_id', 'deltas', 'tool_positions', 'duration', 'name'],
    },
    'checkpoint': {
        'label': 'Checkpoint',
        'icon': 'psychology',
        'color': 'purple',
        'keys': ['workspace_id', 'checkpoint_id', 'checkpoint_name', 'duration', 'until_done', 'done_threshold', 'hz', 're_inference_steps', 'temporal_ensemble_coeff', 'succeed_done_frames', 'move_homepose', 'move_homepose_duration', 'move_homepose_settle_sec', 'max_steps', 'fallback_block_id', 'name'],
    },
    'replay_episode': {
        # 데이터셋의 특정 에피소드를 frame 별 qaction 으로 직접 replay.
        # 첫 프레임 qpos 로 이동 후 hz 에 맞춰 ``move_joint_step`` 으로 재생.
        # 정책 추론과 무관 — 녹화된 trajectory 를 그대로 따라가는 deterministic 블록.
        'label': 'Replay Episode',
        'icon': 'replay',
        'color': 'green',
        'keys': ['workspace_id', 'dataset_id', 'dataset_name', 'episode_index', 'hz', 'move_to_first', 'settle_sec', 'name'],
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
    'query_pose': {
        'label': 'Query Pose',
        'icon': 'travel_explore',
        'color': 'indigo',
        'keys': ['workspace_id', 'service_name', 'pose_type', 'duration', 'name'],
    },
    'visual_reach': {
        # Wrist depth-camera로 타겟을 보고 EE를 타겟 위(살짝 높게)로 이동.
        # 타겟 지정: text_prompts(SAM3) 또는 boxes(바운딩박스), 없으면 색상 fallback.
        # observe_positions: 관찰 자세(joint preset). rgbd_service: wrist RGB-D 소스.
        'label': 'Wrist View Reach',
        'icon': 'my_location',
        'color': 'pink',
        # cam_pose_mode: 'service'(센서가 카메라 월드 pose 제공) | 'manual_ee'(캘리브레이션
        # 없이 EE 기준 대략적 카메라 마운트를 사람이 입력). manual_ee 시 cam_offset[x,y,z]
        # (EE 프레임, m) + cam_pitch/cam_yaw/cam_roll(deg)로 카메라 pose를 EE FK와 합성.
        'keys': ['workspace_id', 'sensor_id', 'rgbd_service', 'target_mode', 'text_prompts', 'boxes',
                 'exemplar_image', 'exemplar_box', 'references',
                 'target_color', 'observe_positions', 'hover', 'duration', 'settle_sec',
                 'cam_pose_mode', 'cam_offset', 'cam_pitch', 'cam_yaw', 'cam_roll',
                 'cam_convention', 'name'],
    },
}


@planner_bp.route('/planner/block_configs', methods=['GET'])
def get_block_configs():
    return {
        'status': 'success',
        'block_configs': BLOCK_CONFIGS,
    }, 200


@planner_bp.route('/planner/:test_wrist_reach', methods=['POST'])
def test_wrist_reach():
    """Run the Wrist View Reach target mask on the LIVE wrist RGB and return an
    overlay image so the user can see exactly what was detected — isolates a
    detection problem (wrong/empty mask) from a camera-pose problem (mask right
    but the EE goes elsewhere → tune cam_offset/pitch). Uses the same RGB-D
    source + mask logic as the block handler."""
    import base64
    import json
    import numpy as np
    from ..process.planner_run import (_visual_reach_decode_rgb, _visual_reach_mask,
                                        _resolve_rgbd_service, _mask_overlay_b64,
                                        _visual_reach_backproject, _is_tool_agent)
    from ...bridge.client import get_bridge_client
    from ...bridge.generated import robot_bridge_pb2 as pb

    body = request.json or {}
    sid = body.get('sensor_id')
    service_name = _resolve_rgbd_service({'rgbd_service': body.get('rgbd_service'), 'sensor_id': sid})
    text_prompts = body.get('text_prompts') or []
    boxes = body.get('boxes') or []
    # Carry the camera-pose params so the readout uses the SAME geometry as a real run.
    block = {
        'target_color': body.get('target_color'),
        'references': body.get('references'),           # 멀티 레퍼런스(여러 각도) [{image,box},...]
        'exemplar_image': body.get('exemplar_image'),  # box-defined visual exemplar (refer frame)
        'exemplar_box': body.get('exemplar_box'),
        'cam_pose_mode': body.get('cam_pose_mode'),
        'cam_offset': body.get('cam_offset'),
        'cam_pitch': body.get('cam_pitch'),
        'cam_yaw': body.get('cam_yaw'),
        'cam_roll': body.get('cam_roll'),
        'cam_convention': body.get('cam_convention'),
    }

    try:
        client = get_bridge_client()
        resp = client.ros_proxy.CallService(pb.ROSServiceRequest(
            service_type='std_srvs/srv/Trigger', service_name=service_name, request_json=''))
    except Exception as e:
        return {'status': 'error', 'message': f'RGBD service call failed: {e}', 'service': service_name}, 500
    if not resp.success:
        return {'status': 'error', 'message': f"wrist RGBD '{service_name}' unavailable: {resp.response_json}",
                'service': service_name}, 400
    try:
        outer = json.loads(resp.response_json)
        payload = json.loads(outer.get('message') or '{}')
    except Exception as e:
        return {'status': 'error', 'message': f'bad RGBD payload: {e}', 'service': service_name}, 500
    if not payload.get('ok'):
        return {'status': 'error', 'message': f"wrist RGBD: {payload.get('error')}", 'service': service_name}, 400

    rgb = _visual_reach_decode_rgb(payload['rgb_jpeg_b64'])  # HxWx3 RGB
    mask = _visual_reach_mask(rgb, text_prompts, boxes, block)
    mask = np.asarray(mask).astype(bool) if mask is not None else np.zeros(rgb.shape[:2], bool)
    ys, xs = np.where(mask)
    detected = len(xs) >= 8
    centroid = [float(xs.mean()), float(ys.mean())] if detected else None

    # Transparent, mask-ONLY overlay (not a copy of the RGB) so it composites
    # cleanly over the live wrist stream.
    img_b64, _ = _mask_overlay_b64(mask)

    # Calibration readout: if the workspace's arm is running, back-project the
    # detection to world XYZ with the EXACT geometry a real run uses, so the user
    # can tune cam_offset/pitch/yaw/roll until target_xyz lands on the object —
    # without running the whole plan. Best-effort: skipped if no robot/EE pose.
    target_xyz = None
    target_note = None
    if detected:
        try:
            ws_id = body.get('workspace_id')
            agents = getattr(current_app, 'agents', {}) or {}
            ee_pose = None
            if ws_id is not None:
                ws = TaskModel.find(ws_id)
                robots = ((ws.to_dict().get('assembly') or {}).get('robots') or []) if ws else []
                for rb in robots:
                    ag = agents.get(int(rb['id']))
                    if ag is not None and not _is_tool_agent(ag):
                        poses = ag.get_ee_position() or {}
                        if poses:
                            ee_pose = [float(v) for v in next(iter(poses.values()))]
                        break
            cam_mode = block.get('cam_pose_mode') or ('manual_ee' if block.get('cam_offset') is not None else 'service')
            if cam_mode == 'manual_ee' and ee_pose is None:
                target_note = 'robot_off'   # need a running arm for the EE pose
            else:
                bp = _visual_reach_backproject(payload, mask, block, ee_pose=ee_pose)
                if bp:
                    target_xyz = bp['target_xyz']
        except Exception as e:
            target_note = f'target calc skipped: {e}'

    return {
        'status': 'success',
        'detected': detected,
        'centroid': centroid,
        'mask_pixels': int(mask.sum()),
        'service': service_name,
        'image': f'data:image/png;base64,{img_b64}' if img_b64 else None,
        'target_xyz': target_xyz,
        'target_note': target_note,
    }, 200


@planner_bp.route('/planner/:define_wrist_exemplar', methods=['POST'])
def define_wrist_exemplar():
    """Define a Wrist View Reach target by a DRAGGED box on the live wrist view.
    Stores the reference frame + box; YOLOE re-finds that object in later views even
    when the camera pose changed (cross-view). `box_norm` is [xn1,yn1,xn2,yn2] in
    [0,1] (drag fractions) so the UI display size is decoupled from the RGB-D
    resolution. Returns the stored exemplar (image + pixel box) + a preview overlay."""
    import json as _json
    import numpy as np
    from ..process.planner_run import (_visual_reach_decode_rgb, _resolve_rgbd_service,
                                        _mask_overlay_b64)
    from ...utils import yoloe_helper
    from ...bridge.client import get_bridge_client
    from ...bridge.generated import robot_bridge_pb2 as pb

    body = request.json or {}
    sid = body.get('sensor_id')
    bn = body.get('box_norm')
    if not (isinstance(bn, (list, tuple)) and len(bn) == 4):
        return {'status': 'error', 'message': 'box_norm [xn1,yn1,xn2,yn2] (0-1) is required'}, 400
    if not yoloe_helper.is_extension_installed():
        return {'status': 'error',
                'message': "YOLOE 확장이 설치되지 않았습니다. 모듈 관리에서 'YOLOE Visual-Prompt Detection'을 설치하세요.",
                'need_install': 'yoloe'}, 409
    service_name = _resolve_rgbd_service({'rgbd_service': body.get('rgbd_service'), 'sensor_id': sid})
    try:
        client = get_bridge_client()
        resp = client.ros_proxy.CallService(pb.ROSServiceRequest(
            service_type='std_srvs/srv/Trigger', service_name=service_name, request_json=''))
    except Exception as e:
        return {'status': 'error', 'message': f'RGBD service call failed: {e}', 'service': service_name}, 500
    if not resp.success:
        return {'status': 'error', 'message': f"wrist RGBD '{service_name}' unavailable: {resp.response_json}"}, 400
    try:
        payload = _json.loads(_json.loads(resp.response_json).get('message') or '{}')
    except Exception as e:
        return {'status': 'error', 'message': f'bad RGBD payload: {e}'}, 500
    if not payload.get('ok'):
        return {'status': 'error', 'message': f"wrist RGBD: {payload.get('error')}"}, 400

    H, W = int(payload['height']), int(payload['width'])
    xs = sorted((float(bn[0]), float(bn[2])))
    ys = sorted((float(bn[1]), float(bn[3])))
    box = [round(xs[0] * W, 1), round(ys[0] * H, 1), round(xs[1] * W, 1), round(ys[1] * H, 1)]
    if (box[2] - box[0]) < 4 or (box[3] - box[1]) < 4:
        return {'status': 'error', 'message': '박스가 너무 작습니다. 객체를 충분히 감싸도록 드래그하세요.'}, 422

    refer_b64 = payload['rgb_jpeg_b64']
    rgb = _visual_reach_decode_rgb(refer_b64)
    # validate + preview: detect on the SAME frame (refer == target)
    try:
        mask = yoloe_helper.detect_exemplar(rgb, rgb, box)
    except Exception as e:
        return {'status': 'error', 'message': f'YOLOE detect failed: {e}'}, 500
    detected = bool(np.asarray(mask).any())
    img_b64, _ = _mask_overlay_b64(mask if detected else np.zeros((H, W), bool))
    return {
        'status': 'success',
        'detected': detected,
        'exemplar_image': refer_b64,   # store on the block
        'exemplar_box': box,
        'mask_pixels': int(np.asarray(mask).sum()),
        'image': f'data:image/png;base64,{img_b64}' if img_b64 else None,
    }, 200


@planner_bp.route('/planner/:test_query_pose', methods=['POST'])
def test_query_pose():
    """Run a single Query Pose block end-to-end from the block edit form: call
    the ROS service, then actually drive the workspace's robots to the returned
    pose. Returns the pose so the user can see what came back."""
    body = request.json or {}
    service_name = (body.get('service_name') or '').strip()
    pose_type = body.get('pose_type')
    workspace_id = body.get('workspace_id')
    if not service_name:
        return {'status': 'error', 'message': 'service_name is required'}, 400
    if pose_type not in ('joint_position', 'end_effector_position'):
        return {'status': 'error', 'message': "pose_type must be 'joint_position' or 'end_effector_position'"}, 400
    if workspace_id is None:
        return {'status': 'error', 'message': 'workspace_id is required'}, 400

    if PLANNER_RUN_PROCESS_NAME in current_app.pm.processes:
        return {'status': 'error', 'message': 'A planner run is in progress — stop it before testing'}, 409

    ws = TaskModel.find(workspace_id)
    if ws is None:
        return {'status': 'error', 'message': 'workspace not found'}, 404
    ws_dict = ws.to_dict()

    agents = getattr(current_app, 'agents', {}) or {}
    for robot in (ws_dict.get('assembly') or {}).get('robots') or []:
        if int(robot['id']) not in agents:
            return {'status': 'error',
                    'message': f"robot '{robot.get('name')}' is not running — turn it on first"}, 409

    try:
        duration = float(body.get('duration') or 5.0)
    except (TypeError, ValueError):
        duration = 5.0

    block = {
        'type': 'query_pose',
        'name': 'test',
        'workspace_id': workspace_id,
        'service_name': service_name,
        'pose_type': pose_type,
        'duration': duration,
    }
    ctx = {'workspaces_by_id': {workspace_id: ws_dict}, 'agents': agents}
    task_control = {'stop': False}

    from ..process.planner_run import _run_query_pose

    try:
        pose = _run_query_pose(block, ctx, task_control)
    except RuntimeError as e:
        return {'status': 'error', 'message': str(e)}, 502
    except Exception as e:
        return {'status': 'error', 'message': f'query_pose test failed: {e}'}, 500

    return {
        'status': 'success',
        'message': 'Robot moved to the pose returned by the service',
        'pose': pose,
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

        elif btype == 'move_relative_ee':
            ws = workspaces_by_id.get(block.get('workspace_id'))
            if ws is None:
                errors.append(f"{prefix}: workspace not found")
                continue
            for robot in (ws.get('assembly') or {}).get('robots') or []:
                if int(robot['id']) not in agents:
                    errors.append(f"{prefix}: robot '{robot.get('name')}' is not running")
            # Arm 은 deltas (6-DOF), tool (role='tool' or no IK) 은 tool_positions
            # (절대 joint 위치). 둘 중 어느 한 쪽이라도 비어있지 않으면 통과.
            deltas = block.get('deltas') or {}
            tool_positions = block.get('tool_positions') or {}
            if (not isinstance(deltas, dict) or not deltas) and (
                not isinstance(tool_positions, dict) or not tool_positions
            ):
                errors.append(
                    f"{prefix}: at least one of deltas (arm 6-DOF) or "
                    f"tool_positions (tool absolute joints) is required"
                )
            if isinstance(deltas, dict):
                for rid, vec in deltas.items():
                    if not isinstance(vec, (list, tuple)) or len(vec) != 6:
                        errors.append(
                            f"{prefix}: deltas[{rid}] must be a list of 6 floats "
                            f"(dx, dy, dz, drx, dry, drz)"
                        )
                        continue
                    try:
                        [float(v) for v in vec]
                    except (TypeError, ValueError):
                        errors.append(f"{prefix}: deltas[{rid}] contains non-numeric values")
            if isinstance(tool_positions, dict):
                for rid, vec in tool_positions.items():
                    if not isinstance(vec, (list, tuple)) or len(vec) == 0:
                        errors.append(
                            f"{prefix}: tool_positions[{rid}] must be a non-empty list of joint values"
                        )
                        continue
                    try:
                        [float(v) for v in vec]
                    except (TypeError, ValueError):
                        errors.append(f"{prefix}: tool_positions[{rid}] contains non-numeric values")
            try:
                if float(block.get('duration') or 0) <= 0:
                    errors.append(f"{prefix}: duration must be > 0")
            except (TypeError, ValueError):
                errors.append(f"{prefix}: duration is invalid")

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
            # max_steps: 실패 판정 step 한도. until_done 일 때만 의미 — done 신호
            # 없이 이 step 에 도달하면 실패로 보고 fallback 블록을 실행.
            if block.get('max_steps') is not None:
                try:
                    ms = int(block.get('max_steps'))
                    if ms <= 0:
                        errors.append(f"{prefix}: max_steps must be > 0")
                except (TypeError, ValueError):
                    errors.append(f"{prefix}: max_steps is invalid")

        elif btype == 'replay_episode':
            ws = workspaces_by_id.get(block.get('workspace_id'))
            if ws is None:
                errors.append(f"{prefix}: workspace not found")
                continue
            for robot in (ws.get('assembly') or {}).get('robots') or []:
                if int(robot['id']) not in agents:
                    errors.append(f"{prefix}: robot '{robot.get('name')}' is not running")
            try:
                if block.get('dataset_id') is None or int(block['dataset_id']) <= 0:
                    errors.append(f"{prefix}: dataset_id is required")
            except (TypeError, ValueError):
                errors.append(f"{prefix}: dataset_id is invalid")
            try:
                ep_idx = int(block.get('episode_index'))
                if ep_idx < 0:
                    errors.append(f"{prefix}: episode_index must be >= 0")
            except (TypeError, ValueError):
                errors.append(f"{prefix}: episode_index is required")
            try:
                if float(block.get('hz') or 0) <= 0:
                    errors.append(f"{prefix}: hz must be > 0")
            except (TypeError, ValueError):
                errors.append(f"{prefix}: hz is invalid")

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

        elif btype == 'query_pose':
            ws = workspaces_by_id.get(block.get('workspace_id'))
            if ws is None:
                errors.append(f"{prefix}: workspace not found")
                continue
            service_name = (block.get('service_name') or '').strip()
            if not service_name:
                errors.append(f"{prefix}: service_name is required")
            pose_type = block.get('pose_type')
            if pose_type not in ('joint_position', 'end_effector_position'):
                errors.append(f"{prefix}: pose_type must be 'joint_position' or 'end_effector_position'")
            for robot in (ws.get('assembly') or {}).get('robots') or []:
                if int(robot['id']) not in agents:
                    errors.append(f"{prefix}: robot '{robot.get('name')}' is not running")
            try:
                if float(block.get('duration') or 0) <= 0:
                    errors.append(f"{prefix}: duration must be > 0")
            except (TypeError, ValueError):
                errors.append(f"{prefix}: duration is invalid")

        elif btype == 'visual_reach':
            ws = workspaces_by_id.get(block.get('workspace_id'))
            if ws is None:
                errors.append(f"{prefix}: workspace not found")
                continue
            # needs an arm agent (IK) running in the workspace
            arm_running = False
            for robot in (ws.get('assembly') or {}).get('robots') or []:
                if int(robot['id']) not in agents:
                    errors.append(f"{prefix}: robot '{robot.get('name')}' is not running")
                else:
                    arm_running = True
            if not arm_running:
                errors.append(f"{prefix}: needs a running arm to move the end-effector")
            # RGB-D source: an explicit rgbd_service (sim/custom) OR a selected
            # sensor (real use_depth → handler derives /ec_sensor_<id>/wrist_rgbd).
            if not (block.get('rgbd_service') or '').strip() and block.get('sensor_id') in (None, ''):
                errors.append(f"{prefix}: rgbd_service or a depth sensor is required")
            try:
                if float(block.get('duration') or 0) <= 0:
                    errors.append(f"{prefix}: duration must be > 0")
            except (TypeError, ValueError):
                errors.append(f"{prefix}: duration is invalid")

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
    group_id = body.get('group_id')    # None → 전체 재생, 값 있으면 해당 그룹만
    block_id = body.get('block_id')    # 값 있으면 그 블록 하나만 실행 ("이 블록만 실행")

    # 실행할 그룹들을 결정.
    if block_id is not None:
        # 단일 블록 실행 — 어느 그룹에 있든 그 블록 하나만 담은 합성 그룹을 만든다.
        found = None
        for g in plans:
            for b in (g.get('blocks') or []):
                if b.get('id') == block_id:
                    found = (g, b)
                    break
            if found:
                break
        if not found:
            return {'status': 'error', 'message': f'Block {block_id} not found'}, 404
        src_group, blk = found
        run_groups = [{**src_group, 'blocks': [blk]}]
    elif group_id is None:
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


@planner_bp.route('/planner/<id>/:export', methods=['POST', 'GET'])
def export_planner(id):
    """Bundle the planner + every checkpoint it references + standalone
    execution code (run_planner.py / ros_planner_service.py) into a zip
    download that runs outside the EasyTrainer container."""
    planner = PlannerModel.find(id)
    if not planner:
        return {'status': 'error', 'message': 'Planner not found'}, 404

    plans = _ensure_plans_loaded(planner)
    if not plans:
        return {'status': 'error', 'message': 'Planner has no groups to export'}, 400

    try:
        buf, filename = bundle_planner_zip(planner=planner.to_dict(), groups=plans)
    except (ValueError, FileNotFoundError) as e:
        return {'status': 'error', 'message': str(e)}, 400
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

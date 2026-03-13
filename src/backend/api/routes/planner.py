from flask import Blueprint, request
from ...database.models.planner_model import Planner as PlannerModel

# 1. Blueprint 생성
# 이 블루프린트는 플래너와 관련된 'HTTP' 라우트를 관리합니다.
planner_bp = Blueprint('planner_bp', __name__)


@planner_bp.route('/planners', methods=['GET'])
def get_planners():
    planners = PlannerModel.all()
    planners = [planner.to_dict() for planner in planners]
    return {
        'status': 'success',
        'planners': planners
    }, 200


@planner_bp.route('/planner', methods=['POST'])
def create_planner():
    """
    새로운 플래너를 생성합니다.
    """
    data = request.json
    
    # 필수 필드 확인
    if not data or not data.get('name'):
        return {'status': 'error', 'message': 'Name is a required field.'}, 400

    new_planner = PlannerModel.create(
        name=data.get('name'),
        task_ids=data.get('task_ids', []),
        plan=data.get('plan', {})
    )
    
    return {
        'status': 'success',
        'message': 'Planner Created',
        'id': new_planner.id
    }, 201

@planner_bp.route('/planner/<id>', methods=['PUT'])
def update_planner(id):
    """
    기존 플래너를 ID로 찾아 업데이트합니다.
    """
    planner = PlannerModel.find(id)
    if not planner:
        return {'status': 'error', 'message': 'Planner not found'}, 404

    data = request.json
    
    planner.name = data.get('name', planner.name)
    planner.task_ids = data.get('workspace_ids', planner.task_ids)
    planner.plan = data.get('plan', planner.plan)
    
    planner.save()

    print(planner.to_dict())  # 업데이트된 플래너 정보 출력 (디버깅용)
    
    return {
        'status': 'success',
        'message': 'Planner Updated'
    }, 200

@planner_bp.route('/planner/<id>', methods=['DELETE'])
def delete_planner(id):
    """
    플래너를 ID로 찾아 삭제합니다. (Soft delete)
    """
    planner = PlannerModel.find(id)
    if not planner:
        return {'status': 'error', 'message': 'Planner not found'}, 404

    planner.delete()
    
    return {
        'status': 'success',
        'message': 'Planner Deleted'
    }, 200

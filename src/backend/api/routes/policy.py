from flask import Blueprint, request
from ...database.models.policy_model import Policy as PolicyModel

policy_bp = Blueprint('policy_bp', __name__)

@policy_bp.route('/policies', methods=['GET'])
def get_policies():
    policies = PolicyModel.all()
    policies = [policy.to_dict() for policy in policies]
    return {
        'status': 'success', 'policies': policies}, 200

@policy_bp.route('/policy', methods=['POST'])
def create_policy():
    data = request.json
    new_policy = PolicyModel.create(
        name=data.get('name'),
        type=data.get('type'),
        settings=data.get('settings')
    )
    return {'status': 'success', 'message': 'Policy Created'}, 200

@policy_bp.route('/policy/<id>', methods=['PUT'])
def update_policy(id):
    data = request.json
    policy = PolicyModel.find(id)
    if not policy:
        return {'status': 'error', 'message': 'Policy not found'}, 404

    policy.name = data.get('name', policy.name)
    policy.type = data.get('type', policy.type)
    policy.settings = data.get('settings', policy.settings)
    policy.save()
    
    return {'status': 'success', 'message': 'Policy Updated'}, 200

@policy_bp.route('/policy/<id>', methods=['DELETE'])
def delete_policy(id):
    policy = PolicyModel.find(id)
    if not policy:
        return {'status': 'error', 'message': 'Policy not found'}, 404

    policy.delete()
    return {'status': 'success', 'message': 'Policy Deleted'}, 200

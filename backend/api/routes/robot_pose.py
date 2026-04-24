# -*- coding: utf-8 -*-
from flask import Blueprint, request, jsonify
from ...database.models.robot_pose_model import RobotPose
import json
import random

BRIGHT_COLORS = [
    '#4FC3F7', '#81C784', '#FFB74D', '#FF8A65', '#BA68C8',
    '#4DD0E1', '#AED581', '#FFD54F', '#F06292', '#7986CB',
    '#4DB6AC', '#DCE775', '#FF8A80', '#80DEEA', '#CE93D8',
]

robot_pose_bp = Blueprint('robot_pose', __name__)


@robot_pose_bp.route('/robot/<int:robot_id>/poses', methods=['GET'])
def list_poses(robot_id):
    poses = RobotPose.select().where(RobotPose.robot_id == robot_id).order_by(RobotPose.created_at)
    return jsonify({
        'status': 'success',
        'poses': [p.to_dict() for p in poses],
    })


@robot_pose_bp.route('/robot/<int:robot_id>/pose', methods=['POST'])
def create_pose(robot_id):
    data = request.json or {}
    name = data.get('name', 'home')
    pose = data.get('pose', [])
    color = data.get('color') or random.choice(BRIGHT_COLORS)
    is_default = data.get('is_default', False)

    # is_default이면 기존 default 해제
    if is_default:
        RobotPose.update(is_default=False).where(
            RobotPose.robot_id == robot_id,
            RobotPose.is_default == True,
        ).execute()

    rp = RobotPose.create(
        robot_id=robot_id,
        name=name,
        pose=json.dumps(pose),
        color=color,
        is_default=is_default,
    )
    return jsonify({'status': 'success', 'pose': rp.to_dict()}), 201


@robot_pose_bp.route('/robot/pose/<int:pose_id>', methods=['PUT'])
def update_pose(pose_id):
    rp = RobotPose.get_or_none(RobotPose.id == pose_id)
    if not rp:
        return jsonify({'error': 'Pose not found'}), 404

    data = request.json or {}
    if 'name' in data:
        rp.name = data['name']
    if 'pose' in data:
        rp.pose = json.dumps(data['pose'])
    if 'is_default' in data:
        if data['is_default']:
            RobotPose.update(is_default=False).where(
                RobotPose.robot_id == rp.robot_id,
                RobotPose.is_default == True,
            ).execute()
        rp.is_default = data['is_default']

    rp.save()
    return jsonify({'status': 'success', 'pose': rp.to_dict()})


@robot_pose_bp.route('/robot/pose/<int:pose_id>', methods=['DELETE'])
def delete_pose(pose_id):
    rp = RobotPose.get_or_none(RobotPose.id == pose_id)
    if not rp:
        return jsonify({'error': 'Pose not found'}), 404
    rp.delete_instance()
    return jsonify({'status': 'success'})

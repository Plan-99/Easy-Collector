from orator import Model


class LeaderRobotPreset(Model):
    __fillable__ = [
        'name',
        'robot_id',
        'gripper_id',
        'origin',
        'gripper_dxl_range',
        'dxl_ids',
        'sign_corrector'
    ]

    __casts__ = {
        'origin': 'json',
        'gripper_dxl_range': 'json',
        'dxl_ids': 'json',
        'sign_corrector': 'json',
    }
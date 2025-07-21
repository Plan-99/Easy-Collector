from orator import Model, SoftDeletes

GRIPPER_CONFIGS = {
    'robotiq': {
        'usb_port': '/dev/ttyUSB0'
    }
}

class GripperObserver:
    def creating(self, gripper):
        if not getattr(gripper, 'settings', None):
            gripper_type = gripper.type
            if gripper_type in GRIPPER_CONFIGS:
                gripper.settings = GRIPPER_CONFIGS[gripper_type]
            else:
                gripper.settings = {}
                

class Gripper(Model, SoftDeletes):
    __fillable__ = [
        'name',
        'type',
        'read_topic',
        'write_topic',
        'settings',
        'image'
    ]

    __casts__ = {
        'settings': 'json',
    }

    __timestamps__ = False
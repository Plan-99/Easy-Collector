# Package marker for database models
from ..config.database import db

from .robot_model import Robot
from .sensor_model import Sensor
from .policy_model import Policy
from .gripper_model import Gripper
from .leader_robot_preset_model import LeaderRobotPreset
from .task_model import Task
from .dataset_model import Dataset
from .checkpoint_model import Checkpoint
from .assembly_model import Assembly
from .teleoperator_model import Teleoperator

ALL_MODELS = [
    Robot, Sensor, Policy, Gripper, LeaderRobotPreset,
    Task, Dataset, Checkpoint, Assembly, Teleoperator
]


def create_tables():
    """Create all tables if they don't exist."""
    db.connect(reuse_if_open=True)
    db.create_tables(ALL_MODELS, safe=True)
    print(f"Migration complete. Ensured {len(ALL_MODELS)} tables exist.")

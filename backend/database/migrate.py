"""
Peewee migration script - creates all tables if they don't exist.
Run this to initialize a fresh database or to ensure schema is up to date.

Usage:
    python -m backend.database.migrate
"""
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from backend.database.config.database import db
from backend.database.models.robot_model import Robot
from backend.database.models.sensor_model import Sensor
from backend.database.models.policy_model import Policy
from backend.database.models.gripper_model import Gripper
from backend.database.models.leader_robot_preset_model import LeaderRobotPreset
from backend.database.models.task_model import Task
from backend.database.models.dataset_model import Dataset
from backend.database.models.checkpoint_model import Checkpoint
from backend.database.models.assembly_model import Assembly
from backend.database.models.teleoperator_model import Teleoperator


ALL_MODELS = [
    Robot, Sensor, Policy, Gripper, LeaderRobotPreset,
    Task, Dataset, Checkpoint, Assembly, Teleoperator
]


def migrate():
    """Create tables that don't already exist (safe to run multiple times)."""
    db.connect(reuse_if_open=True)
    db.create_tables(ALL_MODELS, safe=True)
    print(f"Migration complete. Ensured {len(ALL_MODELS)} tables exist.")


if __name__ == '__main__':
    migrate()

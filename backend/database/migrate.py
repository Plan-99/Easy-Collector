"""
Peewee migration script - creates all tables if they don't exist,
then ensures all columns match the model definitions.

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
from backend.database.models.robot_pose_model import RobotPose
from backend.database.models.planner_model import Planner
from backend.database.models.curriculum_model import Curriculum
from backend.database.models.checkpoint_group_model import CheckpointGroup
from backend.database.models.stage_model import Stage
from backend.database.models.rollout_model import Rollout
from backend.database.models.rollout_result_model import RolloutResult


ALL_MODELS = [
    Robot, Sensor, Policy, Gripper, LeaderRobotPreset,
    Task, Dataset, Checkpoint, Assembly, Teleoperator,
    RobotPose, Planner,
    Curriculum, CheckpointGroup, Stage, Rollout, RolloutResult,
]


def _get_existing_columns(table_name):
    """Get set of column names for a table."""
    cursor = db.execute_sql(f'PRAGMA table_info("{table_name}")')
    return {row[1] for row in cursor.fetchall()}


def _ensure_columns():
    """Add missing columns to existing tables (equivalent of ALTER TABLE ADD COLUMN)."""
    from peewee import AutoField

    for model in ALL_MODELS:
        table_name = model._meta.table_name
        try:
            existing = _get_existing_columns(table_name)
        except Exception:
            continue  # table doesn't exist yet, create_tables will handle it

        for field_name, field_obj in model._meta.fields.items():
            column_name = field_obj.column_name
            if column_name in existing:
                continue
            if isinstance(field_obj, AutoField):
                continue

            # Build column type and constraints manually
            field_type = field_obj.field_type
            parts = [f'"{column_name}"', field_type]
            if field_obj.default is not None:
                default = field_obj.default
                if callable(default):
                    pass  # skip callable defaults for ALTER TABLE
                elif isinstance(default, str):
                    parts.append(f"DEFAULT '{default}'")
                elif isinstance(default, bool):
                    parts.append(f"DEFAULT {1 if default else 0}")
                else:
                    parts.append(f"DEFAULT {default}")
            if field_obj.null:
                parts.append("NULL")

            sql = f'ALTER TABLE "{table_name}" ADD COLUMN {" ".join(parts)}'
            try:
                db.execute_sql(sql)
                print(f"  Added column: {table_name}.{column_name}")
            except Exception as e:
                print(f"  Warning: {table_name}.{column_name}: {e}")


def _migrate_checkpoint_status():
    """One-time data migration for the training queue state machine.

    이전 status 'training' (단일 슬롯 시절) 을 'running' 으로 정규화한다.
    부팅 시점의 ``running`` row 는 **자동으로 'failed' 처리하지 않는다** —
    training_server (별도 프로세스) 에서 학습이 계속 진행 중일 가능성이 높기
    때문. 대신 ``backend/api/app.py`` 의 startup hook 이 ``running`` 체크포인트
    들에 대해 training_server 에 polling 을 재개해서 실제 상태를 확인 + 결과
    처리한다 (``_resume_inflight_trainings`` 참고).
    """
    try:
        db.execute_sql(
            "UPDATE checkpoints SET status='running' WHERE status='training'"
        )
    except Exception as e:
        print(f"  Warning: checkpoint status migration: {e}")


def _recompute_stage_counts():
    """Stage.success_count / failure_count 를 rollout_results 의 ground truth
    로 재계산.

    과거 per-cp 카운팅 (한 rollout 안에서 cp 마다 +1) 코드로 부풀어 있던
    카운트를 그룹 rollout 단위 (rollout_results 의 row 수) 로 정규화한다.
    rollout_results 는 항상 1 rollout × 1 group = 1 row 로 기록되므로
    재계산은 안전하고 idempotent. 매 startup 에 실행해도 부담 없음
    (스테이지 수 × 가벼운 COUNT 쿼리 한 쌍).
    """
    try:
        # success=True 행 개수 → success_count, success=False 행 개수 → failure_count.
        # NULL stage_id 인 row 는 제외, soft-deleted 도 제외.
        db.execute_sql(
            "UPDATE stages SET success_count = ("
            "  SELECT COUNT(*) FROM rollout_results r"
            "  WHERE r.stage_id = stages.id"
            "    AND r.success = 1 AND r.deleted_at IS NULL"
            ") WHERE deleted_at IS NULL"
        )
        db.execute_sql(
            "UPDATE stages SET failure_count = ("
            "  SELECT COUNT(*) FROM rollout_results r"
            "  WHERE r.stage_id = stages.id"
            "    AND r.success = 0 AND r.deleted_at IS NULL"
            ") WHERE deleted_at IS NULL"
        )
    except Exception as e:
        print(f"  Warning: stage count recompute: {e}")


def migrate():
    """Create tables that don't already exist, then ensure all columns are present."""
    db.connect(reuse_if_open=True)
    db.create_tables(ALL_MODELS, safe=True)
    _ensure_columns()
    _migrate_checkpoint_status()
    _recompute_stage_counts()
    print(f"Migration complete. Ensured {len(ALL_MODELS)} tables exist with all columns.")


if __name__ == '__main__':
    migrate()

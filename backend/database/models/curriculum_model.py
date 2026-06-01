from peewee import CharField, IntegerField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


class Curriculum(SoftDeleteModel):
    """플래너와 1:1인 자가 학습 단위.

    base 데이터셋 + training param 을 보유하고, 그 아래 CheckpointGroup → Stage →
    Rollout 계층으로 자가 수집/재학습을 수행한다.
    docs/design-docs/2026-05-29_curriculum-self-training.md 참고.
    """

    class Meta:
        table_name = 'curriculums'

    STATUS_IDLE = 'idle'
    STATUS_RUNNING = 'running'
    STATUS_STOPPED = 'stopped'

    # base 데이터셋·training 파라미터는 커리큘럼이 아니라 체크포인트별 설정으로 이동했다
    # (CheckpointGroup.checkpoint_settings). Curriculum 은 플래너↔그룹을 묶는 얇은 컨테이너.
    name = CharField(null=True)
    planner_id = IntegerField(null=True)
    status = CharField(null=True, default=STATUS_IDLE)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        return super().save(*args, **kwargs)

    @property
    def planner(self):
        from .planner_model import Planner
        return Planner.find(self.planner_id) if self.planner_id else None

    @property
    def checkpoint_groups(self):
        from .checkpoint_group_model import CheckpointGroup
        return list(
            CheckpointGroup.all_active().where(CheckpointGroup.curriculum_id == self.id)
        )

    def to_dict(self):
        data = super().to_dict()
        data['checkpoint_groups'] = [g.to_dict() for g in self.checkpoint_groups]
        return data

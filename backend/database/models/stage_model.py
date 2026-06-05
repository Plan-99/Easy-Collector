from peewee import CharField, IntegerField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


class Stage(SoftDeleteModel):
    """체크포인트 그룹에 종속된 난이도/판정/학습 단위.

    - rollout_noise: 모션 블록별 노이즈 스펙(현재 스냅샷) — { block_key: {xyz, axyz delta 범위}, ... }
    - success_criteria: 체크포인트별 판정 조건(현재 스냅샷) — { checkpoint_id: {max_steps, success_threshold, fallback_block} }
    - 정책(승급 목표·저장확률·상승률)은 CheckpointGroup.mission 으로 이동.
    - success_count / failure_count: 그룹 rollout 단위로 누적 (저장 확률과 무관).
      1 파이프라인 한 바퀴 = 정확히 한 쪽 +1. 그룹의 모든 cp 가 성공해야 success.
    - trained_checkpoint_id: 승급 시 재학습으로 생긴 새 체크포인트.
    docs/design-docs/2026-05-29_curriculum-self-training.md 참고.
    """

    class Meta:
        table_name = 'stages'

    STATUS_ACTIVE = 'active'
    STATUS_TRAINING = 'training'
    STATUS_COMPLETED = 'completed'

    # 정책(mission)은 그룹으로 이동. Stage 는 정책으로부터 계산된 현재 난이도 스냅샷 +
    # 런타임 상태(카운트)만 보유한다.
    __casts__ = {
        'rollout_noise': 'json',
        'success_criteria': 'json',
    }

    checkpoint_group_id = IntegerField(null=True)
    index = IntegerField(null=True, default=0)
    status = CharField(null=True, default=STATUS_ACTIVE)
    rollout_noise = TextField(null=True)
    success_criteria = TextField(null=True)
    success_count = IntegerField(null=True, default=0)
    failure_count = IntegerField(null=True, default=0)
    trained_checkpoint_id = IntegerField(null=True)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        for field_name in self.__casts__:
            val = getattr(self, field_name, None)
            if val is not None and not isinstance(val, str):
                setattr(self, field_name, json.dumps(val))
        return super().save(*args, **kwargs)

    @classmethod
    def create(cls, **kwargs):
        for field_name in ('rollout_noise', 'success_criteria'):
            if field_name in kwargs and isinstance(kwargs[field_name], (list, dict)):
                kwargs[field_name] = json.dumps(kwargs[field_name])
        return super().create(**kwargs)

    @property
    def datasets(self):
        from .dataset_model import Dataset
        return list(Dataset.all_active().where(Dataset.stage_id == self.id))

    @property
    def correction_count(self):
        """이 Stage 의 모든 교정(dagger) 데이터셋의 에피소드 합. 성공률 계산
        시 ``dagger == 실패`` 로 분모에 포함되도록 노출. 별도 컬럼 없이 데이터셋에서
        파생 (사용자 교정 이후 record_episode 가 dagger 데이터셋에 직접 적재)."""
        return sum(len(d.episodes or []) for d in self.datasets if d.role == 'dagger')

    def to_dict(self):
        data = super().to_dict()
        data['datasets'] = [d.to_dict() for d in self.datasets]
        data['correction_count'] = self.correction_count
        return data

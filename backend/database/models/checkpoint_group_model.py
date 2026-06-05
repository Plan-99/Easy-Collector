from peewee import CharField, IntegerField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


class CheckpointGroup(SoftDeleteModel):
    """체크포인트들의 묶음 — 롤아웃의 성공/실패 판정 단위.

    한 그룹의 성공 = 그룹 내 전 체크포인트 성공. 그룹마다 독립적인 Stage 시퀀스를 가진다.
    docs/design-docs/2026-05-29_curriculum-self-training.md 참고.
    """

    class Meta:
        table_name = 'checkpoint_groups'

    _JSON_FIELDS = ('checkpoint_ids', 'motion_block_ids', 'checkpoint_settings', 'block_noise', 'mission', 'training_map')

    # 그룹 학습 라이프사이클 상태.
    STATUS_COLLECTING = 'collecting'  # 롤아웃으로 데이터 수집 중(기본)
    STATUS_TRAINING = 'training'      # 미션 달성 → 학습 진행 중(수집 중단)

    __casts__ = {
        'checkpoint_ids': 'json',          # 소속 체크포인트 id 목록
        'motion_block_ids': 'json',        # 이 그룹의 롤아웃에 영향 주는 모션 블록 id 목록
        # 체크포인트별 학습 설정: { "<checkpoint_id>": { base_dataset_ids: [...], train_settings: {...} } }
        'checkpoint_settings': 'json',
        # 모션 블록별 noise base: { "<block_id>": { rate: {x,y,z,ax,ay,az}, offset: {...} } }
        'block_noise': 'json',
        # 그룹 정책(모든 Stage 공통): target_success_count, failure_save_prob
        'mission': 'json',
        # 학습 중인 체크포인트 매핑 { "<old_cp_id>": <new_cp_id> } — 학습 완료 시 졸업에 사용.
        'training_map': 'json',
    }

    name = CharField(null=True)
    curriculum_id = IntegerField(null=True)
    color = CharField(null=True)           # 그룹 색상(Quasar color name)
    status = CharField(null=True, default=STATUS_COLLECTING)
    checkpoint_ids = TextField(null=True)
    motion_block_ids = TextField(null=True)
    checkpoint_settings = TextField(null=True)
    block_noise = TextField(null=True)
    mission = TextField(null=True)
    training_map = TextField(null=True)
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
        for field_name in cls._JSON_FIELDS:
            if field_name in kwargs and isinstance(kwargs[field_name], (list, dict)):
                kwargs[field_name] = json.dumps(kwargs[field_name])
        return super().create(**kwargs)

    @property
    def stages(self):
        from .stage_model import Stage
        return list(
            Stage.all_active()
            .where(Stage.checkpoint_group_id == self.id)
            .order_by(Stage.index)
        )

    @property
    def current_stage(self):
        from .stage_model import Stage
        return (
            Stage.all_active()
            .where(Stage.checkpoint_group_id == self.id)
            .order_by(Stage.index.desc())
            .first()
        )

    def to_dict(self):
        data = super().to_dict()
        data['stages'] = [s.to_dict() for s in self.stages]
        return data

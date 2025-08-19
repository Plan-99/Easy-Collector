from orator.migrations import Migration
import json

class AddIsVlaToCheckpointsTable(Migration):

    def up(self):
        """
        마이그레이션을 실행합니다.
        """
        # 1. 스키마 변경: is_vla 컬럼 추가 및 task_id를 nullable로 변경
        with self.schema.table('checkpoints') as table:
            table.boolean('is_vla').default(False)
            # task_id 컬럼을 nullable로 변경합니다.
            table.integer('task_id').nullable().change()

        # 2. 데이터 추가: VLA 기본 체크포인트 데이터를 모든 컬럼에 맞게 삽입합니다.
        self.db.table('checkpoints').insert({
            'name': 'VLA Default Checkpoint',
            'is_vla': True,
            
            # Foreign Key 및 상태 값
            'task_id': None, # nullable로 변경했으므로 None(NULL) 값 지정
            'policy_id': None,
            'load_model_id': None,
            'is_training': False, # 기본 모델이므로 학습 중이 아님
            
            # 학습 파라미터 (일반적인 기본값)
            'batch_size': 32,
            'num_workers': 4,
            
            # JSON 형태의 설정 값 (빈 JSON 객체로 초기화)
            'dataset_info': json.dumps({}),
            'train_settings': json.dumps({}),

            # 성능 지표
            'loss': None,
            'best_epoch': 0,

            # 타임스탬프
            'created_at': '',
            'updated_at': '',
            'deleted_at': None # Soft delete 컬럼이므로 기본값은 None
        })

    def down(self):
        """
        마이그레이션을 되돌립니다. (up 메소드 실행의 역순)
        """
        # 1. 데이터 삭제: up()에서 추가했던 데이터를 먼저 삭제합니다.
        self.db.table('checkpoints').where('name', 'VLA Default Checkpoint').delete()

        # 2. 스키마 변경 되돌리기
        with self.schema.table('checkpoints') as table:
            # is_vla 컬럼 삭제
            table.drop_column('is_vla')
            # task_id 컬럼을 다시 NOT NULL로 변경
            table.integer('task_id').nullable(False).change()

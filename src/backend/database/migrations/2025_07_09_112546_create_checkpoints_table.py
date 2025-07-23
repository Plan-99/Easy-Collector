from orator.migrations import Migration
import os
import shutil


class CreateCheckpointsTable(Migration):
    def up(self):
        with self.schema.create('checkpoints') as table:
            table.increments('id')
            table.string('name')
            table.integer('task_id').unsigned()
            table.integer('policy_id').unsigned()
            table.timestamps()
            table.soft_deletes()

    def down(self):
        checkpoint_dir = os.path.join('/root/src/backend', 'checkpoints')

        # 2. 폴더가 존재하는지 확인하고 삭제합니다.
        if os.path.exists(checkpoint_dir):
            try:
                # shutil.rmtree()는 폴더와 그 안의 모든 내용을 삭제합니다.
                shutil.rmtree(checkpoint_dir)
                print(f"✅ Directory '{checkpoint_dir}' successfully deleted.")
            except Exception as e:
                print(f"❌ Error deleting directory '{checkpoint_dir}': {e}")
                
        self.schema.drop('checkpoints')


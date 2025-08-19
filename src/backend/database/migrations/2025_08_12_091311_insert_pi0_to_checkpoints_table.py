from orator.migrations import Migration
import json
from datetime import datetime


class InsertPi0ToCheckpointsTable(Migration):

    def up(self):
        self.db.table('checkpoints').insert({
            'name': 'pi0_basic',
            'policy_id': 43,
            'dataset_info': json.dumps({}),
            'num_epochs': 0,
            'batch_size': 0,
            'is_training': False,
            'train_settings': json.dumps({}),
            'best_epoch': 0,
            'num_workers': 0,
            'load_model_id': None,
            'loss': 0.0,
            'is_vla': True,
            'deleted_at': None,
            'created_at': datetime.now(),
            'updated_at': datetime.now()
        })

    def down(self):
        self.db.table('checkpoints').where('name', 'pi0_basic').delete()

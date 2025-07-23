from orator.migrations import Migration
import json


class AddDatasetInfoToCheckpointsTable(Migration):

    def up(self):
        with self.schema.table('checkpoints') as table:
            table.json('dataset_info').default=json.dumps({})
            
    def down(self):
        with self.schema.table('checkpoints') as table:
            table.drop_column('dataset_info')

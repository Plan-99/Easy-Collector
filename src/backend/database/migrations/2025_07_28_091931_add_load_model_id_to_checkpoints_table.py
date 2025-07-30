from orator.migrations import Migration


class AddLoadModelIdToCheckpointsTable(Migration):

    def up(self):
        with self.schema.table('checkpoints') as table:
            table.integer('load_model_id').unsigned().nullable()

    def down(self):
        with self.schema.table('checkpoints') as table:
            table.drop_column('load_model_id')

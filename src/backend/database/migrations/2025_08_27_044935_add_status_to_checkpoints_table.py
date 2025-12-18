from orator.migrations import Migration


class AddStatusToCheckpointsTable(Migration):

    def up(self):
        with self.schema.table('checkpoints') as table:
            table.enum('status', ['waiting', 'training', 'finished']).default('finished')

    def down(self):
        with self.schema.table('checkpoints') as table:
            table.drop_column('status')

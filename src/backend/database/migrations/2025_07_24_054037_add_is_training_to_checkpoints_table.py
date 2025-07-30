from orator.migrations import Migration


class AddIsTrainingToCheckpointsTable(Migration):

    def up(self):
        with self.schema.table('checkpoints') as table:
            table.boolean('is_training').default(False)

    def down(self):
        with self.schema.table('checkpoints') as table:
            table.drop_column('is_training')

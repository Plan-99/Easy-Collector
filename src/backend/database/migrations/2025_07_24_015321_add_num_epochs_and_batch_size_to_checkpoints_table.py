from orator.migrations import Migration


class AddNumEpochsAndBatchSizeToCheckpointsTable(Migration):

    def up(self):
        with self.schema.table('checkpoints') as table:
            table.integer('num_epochs').default(1000)
            table.integer('batch_size').default(32)

    def down(self):
        with self.schema.table('checkpoints') as table:
            table.drop_column('num_epochs')
            table.drop_column('batch_size')

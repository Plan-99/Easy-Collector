from orator.migrations import Migration


class AddLearningRateAndLrBackboneToCheckpointTable(Migration):

    def up(self):
        with self.schema.table('checkpoints') as table:
            table.integer('learning_rate').default(1e-5)
            table.integer('lr_backbone').default(1e-6)

    def down(self):
        with self.schema.table('checkpoints') as table:
            table.drop_column('learning_rate')
            table.drop_column('lr_backbone')
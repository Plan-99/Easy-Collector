from orator.migrations import Migration


class AddTrainSettingsToCheckpointsTable(Migration):

    def up(self):
        """
        Run the migrations.
        """
        with self.schema.table('checkpoints') as table:
            table.text('train_settings').nullable()
            table.integer('num_workers').default(4)
            table.drop_column('lr_backbone')

    def down(self):
        """
        Revert the migrations.
        """
        with self.schema.table('checkpoints') as table:
            table.drop_column('train_settings')
            table.drop_column('num_workers')
            table.float('lr_backbone').default(1e-5)


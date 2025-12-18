from orator.migrations import Migration


class AddIsBaseModelToCheckpointsTable(Migration):

    def up(self):
        """
        Apply the migrations.
        """
        with self.schema.table('checkpoints') as table:
            table.boolean('is_base_model').default(False)

    def down(self):
        """
        Revert the migrations.
        """
        with self.schema.table('checkpoints') as table:
            table.drop_column('is_base_model')
from orator.migrations import Migration


class CreateCheckpointsTable(Migration):
    def up(self):
        with self.schema.create('checkpoints') as table:
            table.increments('id')
            table.string('name').unique()
            table.integer('dir')
            table.string('file_name')
            table.integer('task_id').unsigned()
            table.integer('policy_id').unsigned()
            table.boolean('is_deleted').default(False)
            table.timestamps()
            table.soft_deletes()
    def down(self):
        self.schema.drop('checkpoints')


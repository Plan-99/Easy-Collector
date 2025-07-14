from orator.migrations import Migration


class CreateDatasetsTable(Migration):

    def up(self):
        with self.schema.create('datasets') as table:
            table.increments('id')
            table.string('name').unique()
            table.integer('task_id').unsigned()
            table.timestamps()
            table.soft_deletes()
            
    def down(self):
        self.schema.drop('datasets')

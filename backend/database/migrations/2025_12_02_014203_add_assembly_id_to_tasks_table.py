from orator.migrations import Migration


class AddAssemblyIdToTasksTable(Migration):

    def up(self):
        with self.schema.table('tasks') as table:
            table.integer('assembly_id').nullable()

    def down(self):
        with self.schema.table('tasks') as table:
            table.drop_column('assembly_id')

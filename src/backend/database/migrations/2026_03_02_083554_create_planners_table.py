from orator.migrations import Migration


class CreatePlannersTable(Migration):

    def up(self):
        with self.schema.create('planners') as table:
            table.increments('id')
            table.string('name')
            table.json('task_ids').nullable()
            table.json('plan').nullable()
            table.timestamps()
            table.soft_deletes()

    def down(self):
        self.schema.drop('planners')
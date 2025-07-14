from orator.migrations import Migration


class CreateRobotsTable(Migration):
    def up(self):
        with self.schema.create('robots') as table:
            table.increments('id')
            table.string('name').unique()
            table.string('type')
            table.json('settings').nullable()
            table.timestamps()
            table.soft_deletes()
    def down(self):
        self.schema.drop('robots')

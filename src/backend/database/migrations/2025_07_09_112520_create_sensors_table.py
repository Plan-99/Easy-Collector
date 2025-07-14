from orator.migrations import Migration
import json


class CreateSensorsTable(Migration):
    def up(self):
        with self.schema.create('sensors') as table:
            table.increments('id')
            table.string('name').unique()
            table.string('type')
            table.json('settings').default(json.dumps({}))
            table.timestamps()
            table.soft_deletes()
    def down(self):
        self.schema.drop('sensors')

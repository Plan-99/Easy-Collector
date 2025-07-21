from orator.migrations import Migration
import json


class CreatePoliciesTable(Migration):
    def up(self):
        with self.schema.create('policies') as table:
            table.increments('id')
            table.string('name')
            table.string('type')
            table.json('settings').default(json.dumps({}))
            table.timestamps()
            table.soft_deletes()
    def down(self):
        self.schema.drop('policies')
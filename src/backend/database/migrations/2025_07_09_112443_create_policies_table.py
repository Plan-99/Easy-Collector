from orator.migrations import Migration
import json


class CreatePoliciesTable(Migration):
    def up(self):
        with self.schema.create('policies') as table:
            table.increments('id')
            table.string('name').unique()
            table.string('type')
            table.integer('batch_size').default(20)
            table.integer('num_epochs').default(2000)
            table.json('settings').default(json.dumps({}))
            table.boolean('is_deleted').default(False)
            table.timestamps()
            table.soft_deletes()
    def down(self):
        self.schema.drop('policies')
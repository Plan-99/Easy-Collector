from orator.migrations import Migration
import json


class CreateGrippersTable(Migration):
    def up(self):
        with self.schema.create('grippers') as table:
            table.increments('id')
            table.string('name').unique()
            table.string('type')
            table.string('read_topic').nullable()
            table.string('write_topic').nullable()
            table.json('settings').default(json.dumps({}))
            table.string('image').nullable()
            table.timestamps()
            table.soft_deletes()
    def down(self):
        self.schema.drop('grippers')

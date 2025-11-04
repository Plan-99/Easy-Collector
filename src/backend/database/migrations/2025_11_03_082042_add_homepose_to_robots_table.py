from orator.migrations import Migration
import json

class AddHomeposeToRobotsTable(Migration):

    def up(self):
        with self.schema.table('robots') as table:
            table.json('homepose').nullable()

    def down(self):
        with self.schema.table('robots') as table:
            table.drop_column('homepose')

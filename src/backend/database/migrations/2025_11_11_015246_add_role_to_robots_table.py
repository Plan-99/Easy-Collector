from orator.migrations import Migration


class AddRoleToRobotsTable(Migration):

    def up(self):
        with self.schema.table('robots') as table:
            table.string('role').default('manipulator')

    def down(self):
        with self.schema.table('robots') as table:
            table.drop_column('role')

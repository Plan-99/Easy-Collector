from orator.migrations import Migration


class AddRoleToRobotsTable(Migration):

    def up(self):
        with self.schema.table('robots') as table:
            table.string('role').default('single_arm')

    def down(self):
        with self.schema.table('robots') as table:
            table.drop_column('role')

from orator.migrations import Migration


class AddHideToSensorAndRobotTable(Migration):

    def up(self):
        with self.schema.table('sensors') as table:
            table.boolean('hide').default(False)

        with self.schema.table('robots') as table:
            table.boolean('hide').default(False)

    def down(self):
        with self.schema.table('sensors') as table:
            table.drop_column('hide')
        with self.schema.table('robots') as table:
            table.drop_column('hide')

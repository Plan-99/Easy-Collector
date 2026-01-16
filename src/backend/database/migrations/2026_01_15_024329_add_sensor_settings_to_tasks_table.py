from orator.migrations import Migration


class AddSensorSettingsToTasksTable(Migration):

    def up(self):
        with self.schema.table('tasks') as table:
            table.json('sensor_settings').nullable()

    def down(self):
        with self.schema.table('tasks') as table:
            table.drop_column('sensor_settings')

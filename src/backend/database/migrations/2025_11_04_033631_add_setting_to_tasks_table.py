from orator.migrations import Migration


class AddSettingToTasksTable(Migration):
    def up(self):
        default_settings = '{"sensors": {}, "robots": {}}'
        with self.schema.table('tasks') as table:
            table.json('settings').nullable(False).default(default_settings)

    def down(self):
        with self.schema.table('tasks') as table:
            table.drop_column('settings')        

from orator.migrations import Migration


class AddEmaToLeaderRobotPresetsTable(Migration):

    def up(self):
        with self.schema.table('leader_robot_presets') as table:
            table.float('ema').default(0.5)

    def down(self):
        with self.schema.table('leader_robot_presets') as table:
            table.drop_column('ema')

from orator.migrations import Migration


class AddPortNameToLeaderTobotPresetsTable(Migration):

    def up(self):
        with self.schema.table('leader_robot_presets') as table:
            table.string('port_name').default('/dev/ttyUSB0')
            
    def down(self):
        with self.schema.table('leader_tobot_presets') as table:
            table.drop_column('port_name')

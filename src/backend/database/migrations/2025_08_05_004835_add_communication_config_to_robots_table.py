from orator.migrations import Migration


class AddCommunicationConfigToRobotsTable(Migration):

    def up(self):
        with self.schema.table('robots') as table:
            table.string('communication_config').nullable().comment('Communication configuration for the robot, e.g., IP address, port, etc.')
            table.string('communication_type').nullable().comment('Type of communication used by the robot, e.g., TCP, UDP, Serial, etc.')

    def down(self):
        with self.schema.table('robots') as table:
            table.drop_column('communication_config')
            table.drop_column('communication_type')

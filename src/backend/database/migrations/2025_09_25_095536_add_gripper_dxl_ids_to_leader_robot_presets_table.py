from orator.migrations import Migration
import json

class AddGripperDxlIdsToLeaderRobotPresetsTable(Migration):

    def up(self):
        with self.schema.table('leader_robot_presets') as table:
            table.json('gripper_dxl_ids').default(json.dumps([]))

    def down(self):
        with self.schema.table('leader_robot_presets') as table:
            table.drop_column('gripper_dxl_ids')
from orator.migrations import Migration
import json

class CreateLeaderRobotPresetsTable(Migration):
    def up(self):
        with self.schema.create('leader_robot_presets') as table:
            table.increments('id')
            table.string('name')
            table.integer('robot_id').unsigned().unique()
            table.integer('gripper_id').unsigned().nullable()
            table.json('origin').default(json.dumps([0.0] * 7))
            table.json('gripper_dxl_range').default(json.dumps([0.0, 0.0]))
            table.json('dxl_ids').default(json.dumps([]))
            table.json('sign_corrector').default(json.dumps([1.0] * 7))
            table.timestamps()
            table.soft_deletes()
    def down(self):
        self.schema.drop('leader_robot_presets')

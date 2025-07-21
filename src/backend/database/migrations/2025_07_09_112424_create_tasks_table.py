from orator.migrations import Migration
import json


class CreateTasksTable(Migration):
    def up(self):
        with self.schema.create('tasks') as table:
            table.increments('id')
            table.string('name')
            table.json('robot_ids').default(json.dumps([]))
            table.json('sensor_ids').default(json.dumps([]))
            table.json('home_pose').default(json.dumps(['0.0'] * 7))
            table.json('end_pose').default(json.dumps(['0.0'] * 7))
            table.string('image').nullable()
            table.integer('episode_len').default(100)
            table.json('sensor_img_size').default(json.dumps([640, 480]))
            table.timestamps()
            table.soft_deletes()
    def down(self):
        self.schema.drop('tasks')

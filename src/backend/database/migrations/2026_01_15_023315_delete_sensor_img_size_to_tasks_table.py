from orator.migrations import Migration


class DeleteSensorImgSizeToTasksTable(Migration):

    def up(self):
        """
        Run the migrations.
        """
        with self.schema.table('tasks') as table:
            table.drop_column('sensor_img_size')

    def down(self):
        """
        Reverse the migrations.
        """
        with self.schema.table('tasks') as table:
            table.integer('sensor_img_size').nullable()

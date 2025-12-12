from orator.migrations import Migration


class CreateAssembliesTable(Migration):

    def up(self):
        with self.schema.create('assemblies') as table:
            table.increments('id')
            table.string('name')
            table.integer('left_arm_id').unsigned().nullable()
            table.integer('right_arm_id').unsigned().nullable()
            table.integer('left_tool_id').unsigned().nullable()
            table.integer('right_tool_id').unsigned().nullable()
            table.integer('mobile_base_id').unsigned().nullable()
            table.boolean('hide').default(False)
            table.timestamps()
            table.soft_deletes()

    def down(self):
        self.schema.drop('assemblies')

from orator.migrations import Migration


class CreateTeleoperatorTable(Migration):

    def up(self):
        with self.schema.create('teleoperators') as table:
            table.increments('id')
            table.string('type')
            table.json('settings').nullable()
            table.integer('assembly_id').unsigned()
            table.timestamps()
            table.soft_deletes()

    def down(self):
        self.schema.drop('teleoperators')
        

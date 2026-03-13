from orator.migrations import Migration


class AddBlocksToPlannersTable(Migration):

    def up(self):
        with self.schema.table('planners') as table:
            table.json('blocks').nullable()

    def down(self):
        with self.schema.table('planners') as table:
            table.drop_column('blocks')
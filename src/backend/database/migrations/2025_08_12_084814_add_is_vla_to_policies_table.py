from orator.migrations import Migration


class AddIsVlaToPoliciesTable(Migration):

    def up(self):
        with self.schema.table('policies') as table:
            table.boolean('is_vla').default(False)

    def down(self):
        with self.schema.table('policies') as table:
            table.drop_column('is_vla')

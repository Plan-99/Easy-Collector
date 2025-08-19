import json
from orator.migrations import Migration
from datetime import datetime


class InsertPi0ToPolicyTable(Migration):

    def up(self):
        self.db.table('policies').insert({
            'name': 'pi0_basic',
            'type': 'PI0',
            'settings': json.dumps({}),
            'is_vla': True,
            'deleted_at': None,
            'created_at': datetime.now(),
            'updated_at': datetime.now()
        })

    def down(self):

        self.db.table('policy').where('name', 'pi0_basic').delete()
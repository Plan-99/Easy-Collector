import os

# Compute persistent DB location. If EASYTRAINER_DATA_DIR is set (container runtime),
# store DB under <data_root>/database/main.db to avoid overwrites from code sync.
# Otherwise fall back to the repo-local DB path.
data_root = os.environ.get('EASYTRAINER_DATA_DIR')
if data_root:
    db_dir = os.path.join(data_root, 'database')
    os.makedirs(db_dir, exist_ok=True)
    DB_PATH = os.path.join(db_dir, 'main.db')
else:
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    DB_PATH = os.path.join(base_dir, 'main.db')

DATABASES = {
    'default': 'sqlite',
    'sqlite': {
        'driver': 'sqlite',
        'database': DB_PATH,
        'prefix': ''
    }
}

import os
from peewee import SqliteDatabase, Model
from playhouse.shortcuts import model_to_dict
import datetime
import json

# Compute persistent DB location
def _ensure_db_permissions(db_dir: str, db_path: str):
    try:
        os.makedirs(db_dir, exist_ok=True)
    except Exception:
        return
    try:
        os.chmod(db_dir, 0o777)
    except Exception:
        pass
    try:
        if os.path.exists(db_path):
            os.chmod(db_path, 0o666)
    except Exception:
        pass


data_root = os.environ.get('EASYTRAINER_DATA_DIR')
if data_root:
    db_dir = os.path.join(data_root, 'database')
    DB_PATH = os.path.join(db_dir, 'main.db')
    _ensure_db_permissions(db_dir, DB_PATH)
else:
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    DB_PATH = os.path.join(base_dir, 'main.db')

db = SqliteDatabase(DB_PATH, pragmas={
    'journal_mode': 'wal',
    'foreign_keys': 1,
    # 동시 쓰기(스케줄러 동시 학습 finalize + flask 요청 + resume_polling 등)에서
    # 즉시 'database is locked' 로 실패하지 않고 최대 5초 대기 후 재시도하도록.
    'busy_timeout': 5000,
})


class JSONField(object):
    """Mixin-style descriptor that auto-serializes/deserializes JSON fields."""
    pass


class BaseModel(Model):
    """Base model with common functionality."""

    class Meta:
        database = db

    def to_dict(self):
        """Convert model to dictionary, respecting __appends__ and __casts__."""
        # Get basic fields
        data = model_to_dict(self, recurse=False, backrefs=False)

        # Parse JSON casts
        casts = getattr(self, '__casts__', {})
        for field_name, cast_type in casts.items():
            if cast_type == 'json' and field_name in data:
                val = data[field_name]
                if isinstance(val, str):
                    try:
                        data[field_name] = json.loads(val)
                    except (json.JSONDecodeError, TypeError):
                        pass

        # Handle timestamps
        for key in ('created_at', 'updated_at', 'deleted_at'):
            if key in data and data[key] is not None:
                if isinstance(data[key], datetime.datetime):
                    data[key] = data[key].isoformat()

        # Add appended properties
        appends = getattr(self, '__appends__', [])
        for prop_name in appends:
            if hasattr(self, prop_name):
                val = getattr(self, prop_name)
                data[prop_name] = val

        return data

    @classmethod
    def find(cls, id):
        """Find a model by primary key, returns None if not found."""
        try:
            return cls.get_by_id(id)
        except cls.DoesNotExist:
            return None

    def _get_json_field(self, field_name):
        """Get a JSON field value, parsing from string if needed."""
        val = getattr(self, field_name, None)
        if isinstance(val, str):
            try:
                return json.loads(val)
            except (json.JSONDecodeError, TypeError):
                return val
        return val

    def _set_json_field(self, field_name, value):
        """Set a JSON field, serializing to string."""
        if value is not None and not isinstance(value, str):
            setattr(self, field_name, json.dumps(value))
        else:
            setattr(self, field_name, value)


class SoftDeleteModel(BaseModel):
    """Base model with soft delete support."""

    @classmethod
    def all_active(cls):
        """Get all records that are not soft-deleted."""
        return cls.select().where(cls.deleted_at.is_null())

    @classmethod
    def all(cls):
        """Get all records that are not soft-deleted (default behavior)."""
        return cls.select().where(cls.deleted_at.is_null())

    def soft_delete(self):
        """Soft delete this record."""
        self.deleted_at = datetime.datetime.now()
        self.save()

    def delete_instance(self, *args, **kwargs):
        """Override delete to perform soft delete."""
        self.soft_delete()

    def restore(self):
        """Restore a soft-deleted record."""
        self.deleted_at = None
        self.save()

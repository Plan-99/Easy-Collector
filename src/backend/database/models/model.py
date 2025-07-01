# app/models/base_model.py
from ..db_init import get_db_connection
import json
import sqlite3

from ...configs.policy_config import POLICY_CONFIGS
from ...configs.sensor_config import SENSOR_CONFIGS
from ...configs.gripper_config import GRIPPER_CONFIGS
from ...configs.robot_config import ROBOT_CONFIGS


def data_transform_for_db(model):
    columns = getattr(model, 'COLUMNS', {}).keys()
    data = model.__dict__

    transformed_data = {
        'id': data['id']
    }
    for column in columns:
        value = data[column]
        if isinstance(value, (dict, list)):
            transformed_data[column] = json.dumps(value)
        else:
            transformed_data[column] = value
    return transformed_data

def data_transform_for_model(row):
    data_dict = dict(row)
    for key, value in data_dict.items():
        if isinstance(value, str):
            if value.strip().startswith(('{', '[')):
                try:
                    data_dict[key] = json.loads(value)
                except json.JSONDecodeError:
                    pass
    return data_dict

# DBModel 클래스 내부
class DBModel:

    def __init__(self, table_name, **kwargs):
        if not table_name:
            raise ValueError("테이블 이름은 필수입니다.")
        self.table_name = table_name
        self.args = kwargs
        
        if self.table_name == 'sensors':
            self.configs = SENSOR_CONFIGS
        elif self.table_name == 'policies':
            self.configs = POLICY_CONFIGS
        elif self.table_name == 'grippers':
            self.configs = GRIPPER_CONFIGS
        elif self.table_name == 'robots':
            self.configs = ROBOT_CONFIGS
        
        self.id = None
        schema = getattr(self, 'COLUMNS', {})
        if schema and isinstance(schema, (dict, list)):
            for col_name, properties in schema.items():
                if 'default' in properties:
                    setattr(self, col_name, properties['default'])
                    # self.processed_data[col_name] = properties['default']
                    
        for key, val in self.args.items():
            setattr(self, key, val)

        if hasattr(self, 'type'):
            self.match_config()

    def set_data(self):
        pass
            
        
    def match_config(self):
        if self.type not in self.configs:
            raise ValueError(f"유효하지 않거나 지원되지 않는 policy_type입니다: '{self.type}'")
        schema = self.configs[self.type]
        
        final_settings = {}
        for param_name, properties in schema.items():
            if 'default' in properties:
                final_settings[param_name] = properties['default']

        user_settings = self.args.get('settings', {})
        for user_param, user_value in user_settings.items():
            if user_param in schema:
                final_settings[user_param] = user_value
            else:
                print(f"경고: '{self.type}' 정책에 '{user_param}'는 정의되지 않은 하이퍼파라미터입니다. 무시됩니다.")
        
        setattr(self, 'settings', final_settings)
        

    def create(self):
        data = data_transform_for_db(self)

        keys = data.keys()
        values = tuple(data.values())

        columns = ', '.join(keys)
        placeholders = ', '.join(['?'] * len(keys))
        sql = f'INSERT INTO {self.table_name} ({columns}) VALUES ({placeholders})'
        print(sql, values)
        self.apply_sql(sql, values)
        
        
    @classmethod
    def find_one(cls, conditions: dict={}, to: str='model'):
        if not hasattr(cls, 'TABLE_NAME'):
            raise NotImplementedError("이 메서드를 사용하려면 자식 클래스에 TABLE_NAME 속성이 정의되어야 합니다.")
        table_name = cls.TABLE_NAME
        querys = [f"{key} = ?" for key in conditions.keys()]
        if len(conditions) > 0:
            where_clause = " WHERE " + " AND ".join(querys)
        else:
            where_clause = ""
        sql = f"SELECT * FROM {table_name}{where_clause}"
        values = tuple(conditions.values())
        conn = None
        try:
            conn = get_db_connection()
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            cursor.execute(sql, values)
            row = cursor.fetchone() # 
            if row:
                data_dict = data_transform_for_model(row)
                model = cls(**data_dict)
                model.set_data()
                if to == 'dict':
                    return model.to_dict()
                return model
            else:
                return None
        finally:
            if conn:
                conn.close()

    @classmethod
    def find_all(cls, conditions: dict={}, to: str='model'):
        result = []
        if not hasattr(cls, 'TABLE_NAME'):
            raise NotImplementedError("이 메서드를 사용하려면 자식 클래스에 TABLE_NAME 속성이 정의되어야 합니다.")
        table_name = cls.TABLE_NAME
        querys = [f"{key} = ?" for key in conditions.keys()]

        if len(conditions) > 0:
            where_clause = " WHERE " + " AND ".join(querys)
        else:
            where_clause = ""
        sql = f"SELECT * FROM {table_name}{where_clause}"
        values = tuple(conditions.values())
        conn = None
        try:
            conn = get_db_connection()
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            cursor.execute(sql, values)
            rows = cursor.fetchall() # 
            for row in rows:
                data_dict = data_transform_for_model(row)
                model = cls(**data_dict)
                model.set_data()
                if to == 'dict':
                    model = model.to_dict()
                result.append(model)
            return result

        finally:
            if conn:
                conn.close()


    def update(self):
        if id is None:
            raise ValueError("업데이트할 데이터의 id가 없습니다. 먼저 데이터를 조회해야 합니다.")
        else:
            data = data_transform_for_db(self)
            keys = data.keys()
            values = tuple(data.values()) + (self.id,)
            set_clause = ', '.join([f"{key} = ?" for key in keys])
            sql = f'UPDATE {self.table_name} SET {set_clause} WHERE id = ?'
            print(sql)
            return self.apply_sql(sql, values)
        
        
    def delete(self):
        if self.id is None:
            raise ValueError("삭제할 데이터의 id가 없습니다. 먼저 데이터를 조회해야 합니다.")
        sql = f'DELETE FROM {self.table_name} WHERE id = ?'
        return self.apply_sql(sql, (self.id,))
    
    
    def apply_sql(self, sql: str, values: tuple = ()):
        conn = None
        try:
            conn = get_db_connection()
            cursor = conn.cursor()
            cursor.execute(sql, values)
            conn.commit()
            last_id = cursor.lastrowid
            return last_id
        except Exception as e:
            if conn:
                conn.rollback()
            raise e
        finally:
            if conn:
                conn.close()
                
    
    #  테이블 생성 SQL 쿼리를 생성하는 클래스 메서드
    @classmethod
    def generate_create_table_sql(cls):
        sql_parts = [
            f"CREATE TABLE IF NOT EXISTS {cls.TABLE_NAME} (",
            "    id INTEGER PRIMARY KEY AUTOINCREMENT"
        ]

        for col_name, col_info in cls.COLUMNS.items():
            default_value = col_info.get('default')

            if isinstance(default_value, int):
                col_type = "INTEGER"
                formatted_default = str(default_value)
            elif isinstance(default_value, float):
                col_type = "REAL"
                formatted_default = str(default_value)
            elif isinstance(default_value, bool):
                col_type = "INTEGER"
                formatted_default = "1" if default_value else "0"
            else:
                col_type = "TEXT"
                if isinstance(default_value, (list, dict)):
                    value_str = json.dumps(default_value, ensure_ascii=False)
                else:
                    value_str = str(default_value)
                    
                escaped_default = value_str.replace("'", "''")
                formatted_default = f"'{escaped_default}'"
                
            if default_value is None:
                column_definition = f"    {col_name} {col_type}" # NOT NULL 제외
            else:
                column_definition = f"    {col_name} {col_type} NOT NULL DEFAULT {formatted_default}"
            
            sql_parts.append(column_definition)

        final_sql = sql_parts[0] + "\n" + ",\n".join(sql_parts[1:]) + "\n);"
        
        return final_sql
    
    
    def get_data(self):
        return self
    
    def to_dict(self):
        result = {}
        columns = list(getattr(self, 'COLUMNS', {}).keys()) + getattr(self, 'NEW_COLS') + ['id']
        for col in columns:
            result[col] = getattr(self, col)
        return result
    
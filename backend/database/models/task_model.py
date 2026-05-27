from peewee import CharField, IntegerField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


class Task(SoftDeleteModel):
    class Meta:
        table_name = 'tasks'

    __casts__ = {
        'home_pose': 'json',
        'end_pose': 'json',
        'settings': 'json',
        'sensor_ids': 'json',
        'sensor_img_size': 'json',
        'sensor_cropped_area': 'json',
        'sensor_rotate': 'json',
    }

    __appends__ = [
        'joint_dim',
        'sensors',
        'sensor_img_size',
        'sensor_cropped_area',
        'sensor_rotate',
        'sensor_sam3'
    ]

    name = CharField(null=True)
    home_pose = TextField(null=True)
    end_pose = TextField(null=True)
    image = CharField(null=True)
    episode_len = IntegerField(null=True)
    settings = TextField(null=True)
    assembly_id = IntegerField(null=True)
    sensor_ids = TextField(null=True)
    sensor_img_size = TextField(null=True)
    sensor_cropped_area = TextField(null=True)
    sensor_rotate = TextField(null=True)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        for field_name in self.__casts__:
            val = getattr(self, field_name, None)
            if val is not None and not isinstance(val, str):
                setattr(self, field_name, json.dumps(val))
        return super().save(*args, **kwargs)

    def _get_json(self, field_name):
        val = getattr(self, field_name, None)
        if isinstance(val, str):
            try:
                return json.loads(val)
            except (json.JSONDecodeError, TypeError):
                return val
        return val

    @property
    def _settings(self):
        return self._get_json('settings') or {}

    @property
    def _sensor_ids(self):
        return self._get_json('sensor_ids') or []

    @property
    def assembly(self):
        # 메모이즈 — to_dict() 한 번에 assembly 가 여러 번 (home_pose_computed,
        # joint_dim, 직접) 접근되는데 매번 Assembly.find() 쿼리가 나가던 것을 1회로.
        # 모델 인스턴스는 요청마다 새로 생성되므로 캐시는 요청 범위로 안전.
        if not hasattr(self, '_assembly_cache'):
            from .assembly_model import Assembly
            self._assembly_cache = (
                Assembly.find(self.assembly_id) if self.assembly_id else None
            )
        return self._assembly_cache

    @property
    def _resolved_robots(self):
        """assembly.robots 의 robot dict 들을 Robot 객체로 1회만 resolve (메모이즈).
        home_pose_computed 와 joint_dim 이 각각 Robot.find() 루프를 따로 돌며 같은
        robot 을 중복 쿼리하던 것을 통합한다."""
        if not hasattr(self, '_resolved_robots_cache'):
            from .robot_model import Robot
            robots = []
            assembly = self.assembly
            if assembly:
                for robot_dict in assembly.robots:
                    r = Robot.find(robot_dict['id'])
                    if r is not None:
                        robots.append(r)
            self._resolved_robots_cache = robots
        return self._resolved_robots_cache

    @property
    def home_pose_computed(self):
        home_pose = {}
        settings_robots = self._settings.get('robots', {})
        for robot in self._resolved_robots:
            rid = str(robot.id)
            default = [0.0] * len(robot.joint_names)
            home_pose[rid] = (
                settings_robots[rid].get('home_pose', default)
                if rid in settings_robots else default
            )
        return home_pose

    @property
    def joint_dim(self):
        return sum(robot.joint_dim for robot in self._resolved_robots)

    @property
    def sensors(self):
        # Multi-view: sensor_ids 에 같은 sensor_id 가 N 번 들어오면 sensor 객체도
        # 그대로 N 번 리턴 (인덱스가 view 순서를 결정). frontend 의 selectedSensors
        # computed 가 first-occurrence dedup 으로 카드 개수를 물리 sensor 단위로
        # 줄이고, selectedViews computed 가 enumerateViews 로 chip 을 펼친다.
        from .sensor_model import Sensor
        sensors = []
        for sensor_id in self._sensor_ids:
            sensor = Sensor.find(sensor_id)
            if sensor:
                sensors.append(sensor.to_dict())
        return sensors

    def _enumerate_view_keys(self):
        """``sensor_ids`` 를 view_key 들로 펼침. ``sensor_view.enumerate_views``
        와 같은 규칙 + (sensor_id, occurrence) 오름차순 정렬 — frontend, 데이터셋
        features 순서, 학습/추론 image_features 가 모두 같은 ordering 으로 정합.
        """
        seen = {}
        items = []
        for sensor_id in self._sensor_ids:
            sid = int(sensor_id)
            idx = seen.get(sid, 0)
            seen[sid] = idx + 1
            vkey = str(sid) if idx == 0 else f"{sid}_{idx + 1}"
            items.append((sid, idx, vkey))
        items.sort(key=lambda t: (t[0], t[1]))
        return [(sid, vkey) for sid, _occ, vkey in items]

    @property
    def sensor_img_size_computed(self):
        # Multi-view: view_key 별로 별도 entry. 같은 sensor 의 두 번째 view
        # ("5_2") 가 설정에 없으면 첫 view ("5") 값으로 fallback — 새로 추가된
        # view 가 아직 설정 안 잡혔을 때 깨지지 않게.
        img_size = {}
        sensors_settings = self._settings.get('sensors', {})
        for sid, vkey in self._enumerate_view_keys():
            sid_str = str(sid)
            stored = (
                sensors_settings.get(vkey, {}).get('img_size')
                if vkey in sensors_settings else None
            ) or (
                sensors_settings.get(sid_str, {}).get('img_size')
                if sid_str in sensors_settings else None
            )
            img_size[vkey] = stored if stored else [640, 480]
        return img_size

    @property
    def sensor_cropped_area_computed(self):
        cropped_area = {}
        sensors_settings = self._settings.get('sensors', {})
        for sid, vkey in self._enumerate_view_keys():
            sid_str = str(sid)
            stored = (
                sensors_settings.get(vkey, {}).get('cropped_area')
                if vkey in sensors_settings else None
            ) or (
                sensors_settings.get(sid_str, {}).get('cropped_area')
                if sid_str in sensors_settings else None
            )
            cropped_area[vkey] = stored if stored else [0, 0, 640, 480]
        return cropped_area

    @property
    def sensor_rotate_computed(self):
        rotation = {}
        sensors_settings = self._settings.get('sensors', {})
        for sid, vkey in self._enumerate_view_keys():
            sid_str = str(sid)
            stored = (
                sensors_settings.get(vkey, {}).get('rotate')
                if vkey in sensors_settings else None
            )
            if stored is None and sid_str in sensors_settings:
                stored = sensors_settings.get(sid_str, {}).get('rotate')
            rotation[vkey] = stored if stored is not None else 0
        return rotation

    @property
    def sensor_sam3_computed(self):
        # SAM3 per-view segmentation config. Default = disabled / no-op so
        # the rest of the pipeline can read this dict unconditionally.
        default = {
            'enabled': False,
            'text_prompts': [],
            'boxes': [],
            'mode': 'background',  # 'off' | 'background' | 'object'
            'color': [0, 0, 0],
        }
        out = {}
        sensors_settings = self._settings.get('sensors', {})
        for sid, vkey in self._enumerate_view_keys():
            sid_str = str(sid)
            stored = (
                sensors_settings.get(vkey, {}).get('sam3')
                if vkey in sensors_settings else None
            ) or (
                sensors_settings.get(sid_str, {}).get('sam3')
                if sid_str in sensors_settings else None
            ) or {}
            out[vkey] = {**default, **stored}
        return out

    def to_dict(self):
        data = super().to_dict()
        # Override appended properties
        data['home_pose'] = self.home_pose_computed
        data['joint_dim'] = self.joint_dim
        data['sensors'] = self.sensors
        data['sensor_img_size'] = self.sensor_img_size_computed
        data['sensor_cropped_area'] = self.sensor_cropped_area_computed
        data['sensor_rotate'] = self.sensor_rotate_computed
        data['sensor_sam3'] = self.sensor_sam3_computed
        data['sensor_ids'] = self._sensor_ids
        data['settings'] = self._settings
        # Include assembly
        assembly = self.assembly
        data['assembly'] = assembly.to_dict() if assembly else None
        return data

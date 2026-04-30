from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class StatusResponse(_message.Message):
    __slots__ = ("success", "message")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    success: bool
    message: str
    def __init__(self, success: bool = ..., message: _Optional[str] = ...) -> None: ...

class DriverConfig(_message.Message):
    __slots__ = ("process_id", "robot_id", "type", "company", "settings_json")
    PROCESS_ID_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    COMPANY_FIELD_NUMBER: _ClassVar[int]
    SETTINGS_JSON_FIELD_NUMBER: _ClassVar[int]
    process_id: str
    robot_id: int
    type: str
    company: str
    settings_json: str
    def __init__(self, process_id: _Optional[str] = ..., robot_id: _Optional[int] = ..., type: _Optional[str] = ..., company: _Optional[str] = ..., settings_json: _Optional[str] = ...) -> None: ...

class SensorDriverConfig(_message.Message):
    __slots__ = ("process_id", "sensor_id", "type", "company", "settings_json")
    PROCESS_ID_FIELD_NUMBER: _ClassVar[int]
    SENSOR_ID_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    COMPANY_FIELD_NUMBER: _ClassVar[int]
    SETTINGS_JSON_FIELD_NUMBER: _ClassVar[int]
    process_id: str
    sensor_id: int
    type: str
    company: str
    settings_json: str
    def __init__(self, process_id: _Optional[str] = ..., sensor_id: _Optional[int] = ..., type: _Optional[str] = ..., company: _Optional[str] = ..., settings_json: _Optional[str] = ...) -> None: ...

class DriverStatus(_message.Message):
    __slots__ = ("success", "message", "pid")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    PID_FIELD_NUMBER: _ClassVar[int]
    success: bool
    message: str
    pid: int
    def __init__(self, success: bool = ..., message: _Optional[str] = ..., pid: _Optional[int] = ...) -> None: ...

class ProcessId(_message.Message):
    __slots__ = ("name",)
    NAME_FIELD_NUMBER: _ClassVar[int]
    name: str
    def __init__(self, name: _Optional[str] = ...) -> None: ...

class ProcessList(_message.Message):
    __slots__ = ("names",)
    NAMES_FIELD_NUMBER: _ClassVar[int]
    names: _containers.RepeatedScalarFieldContainer[str]
    def __init__(self, names: _Optional[_Iterable[str]] = ...) -> None: ...

class TopicInfo(_message.Message):
    __slots__ = ("name", "type")
    NAME_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    name: str
    type: str
    def __init__(self, name: _Optional[str] = ..., type: _Optional[str] = ...) -> None: ...

class TopicList(_message.Message):
    __slots__ = ("topics",)
    TOPICS_FIELD_NUMBER: _ClassVar[int]
    topics: _containers.RepeatedCompositeFieldContainer[TopicInfo]
    def __init__(self, topics: _Optional[_Iterable[_Union[TopicInfo, _Mapping]]] = ...) -> None: ...

class LaunchConfig(_message.Message):
    __slots__ = ("process_id", "package", "launch_file", "args_json")
    PROCESS_ID_FIELD_NUMBER: _ClassVar[int]
    PACKAGE_FIELD_NUMBER: _ClassVar[int]
    LAUNCH_FILE_FIELD_NUMBER: _ClassVar[int]
    ARGS_JSON_FIELD_NUMBER: _ClassVar[int]
    process_id: str
    package: str
    launch_file: str
    args_json: str
    def __init__(self, process_id: _Optional[str] = ..., package: _Optional[str] = ..., launch_file: _Optional[str] = ..., args_json: _Optional[str] = ...) -> None: ...

class RobotConfig(_message.Message):
    __slots__ = ("robot_json",)
    ROBOT_JSON_FIELD_NUMBER: _ClassVar[int]
    robot_json: str
    def __init__(self, robot_json: _Optional[str] = ...) -> None: ...

class AgentId(_message.Message):
    __slots__ = ("id",)
    ID_FIELD_NUMBER: _ClassVar[int]
    id: int
    def __init__(self, id: _Optional[int] = ...) -> None: ...

class JointValues(_message.Message):
    __slots__ = ("values",)
    VALUES_FIELD_NUMBER: _ClassVar[int]
    values: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, values: _Optional[_Iterable[float]] = ...) -> None: ...

class MoveJointRequest(_message.Message):
    __slots__ = ("agent_id", "action", "from_ee", "velocity_arg")
    AGENT_ID_FIELD_NUMBER: _ClassVar[int]
    ACTION_FIELD_NUMBER: _ClassVar[int]
    FROM_EE_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_ARG_FIELD_NUMBER: _ClassVar[int]
    agent_id: int
    action: _containers.RepeatedScalarFieldContainer[float]
    from_ee: bool
    velocity_arg: float
    def __init__(self, agent_id: _Optional[int] = ..., action: _Optional[_Iterable[float]] = ..., from_ee: bool = ..., velocity_arg: _Optional[float] = ...) -> None: ...

class MoveEERequest(_message.Message):
    __slots__ = ("agent_id", "target_ee_json", "velocity_arg")
    AGENT_ID_FIELD_NUMBER: _ClassVar[int]
    TARGET_EE_JSON_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_ARG_FIELD_NUMBER: _ClassVar[int]
    agent_id: int
    target_ee_json: str
    velocity_arg: float
    def __init__(self, agent_id: _Optional[int] = ..., target_ee_json: _Optional[str] = ..., velocity_arg: _Optional[float] = ...) -> None: ...

class MoveEEDeltaRequest(_message.Message):
    __slots__ = ("agent_id", "delta_ee_json", "velocity_arg", "tool_positions_json")
    AGENT_ID_FIELD_NUMBER: _ClassVar[int]
    DELTA_EE_JSON_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_ARG_FIELD_NUMBER: _ClassVar[int]
    TOOL_POSITIONS_JSON_FIELD_NUMBER: _ClassVar[int]
    agent_id: int
    delta_ee_json: str
    velocity_arg: float
    tool_positions_json: str
    def __init__(self, agent_id: _Optional[int] = ..., delta_ee_json: _Optional[str] = ..., velocity_arg: _Optional[float] = ..., tool_positions_json: _Optional[str] = ...) -> None: ...

class MoveToRequest(_message.Message):
    __slots__ = ("agent_id", "target_pos", "step_size", "duration")
    AGENT_ID_FIELD_NUMBER: _ClassVar[int]
    TARGET_POS_FIELD_NUMBER: _ClassVar[int]
    STEP_SIZE_FIELD_NUMBER: _ClassVar[int]
    DURATION_FIELD_NUMBER: _ClassVar[int]
    agent_id: int
    target_pos: _containers.RepeatedScalarFieldContainer[float]
    step_size: float
    duration: float
    def __init__(self, agent_id: _Optional[int] = ..., target_pos: _Optional[_Iterable[float]] = ..., step_size: _Optional[float] = ..., duration: _Optional[float] = ...) -> None: ...

class ResetIKRequest(_message.Message):
    __slots__ = ("agent_id", "q")
    AGENT_ID_FIELD_NUMBER: _ClassVar[int]
    Q_FIELD_NUMBER: _ClassVar[int]
    agent_id: int
    q: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, agent_id: _Optional[int] = ..., q: _Optional[_Iterable[float]] = ...) -> None: ...

class RobotState(_message.Message):
    __slots__ = ("agent_id", "joint_states", "joint_actions", "ee_pos_json", "ee_target_json", "connected", "joint_vel", "joint_effort")
    AGENT_ID_FIELD_NUMBER: _ClassVar[int]
    JOINT_STATES_FIELD_NUMBER: _ClassVar[int]
    JOINT_ACTIONS_FIELD_NUMBER: _ClassVar[int]
    EE_POS_JSON_FIELD_NUMBER: _ClassVar[int]
    EE_TARGET_JSON_FIELD_NUMBER: _ClassVar[int]
    CONNECTED_FIELD_NUMBER: _ClassVar[int]
    JOINT_VEL_FIELD_NUMBER: _ClassVar[int]
    JOINT_EFFORT_FIELD_NUMBER: _ClassVar[int]
    agent_id: int
    joint_states: _containers.RepeatedScalarFieldContainer[float]
    joint_actions: _containers.RepeatedScalarFieldContainer[float]
    ee_pos_json: str
    ee_target_json: str
    connected: bool
    joint_vel: _containers.RepeatedScalarFieldContainer[float]
    joint_effort: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, agent_id: _Optional[int] = ..., joint_states: _Optional[_Iterable[float]] = ..., joint_actions: _Optional[_Iterable[float]] = ..., ee_pos_json: _Optional[str] = ..., ee_target_json: _Optional[str] = ..., connected: bool = ..., joint_vel: _Optional[_Iterable[float]] = ..., joint_effort: _Optional[_Iterable[float]] = ...) -> None: ...

class SubscribeRequest(_message.Message):
    __slots__ = ("agent_id",)
    AGENT_ID_FIELD_NUMBER: _ClassVar[int]
    agent_id: int
    def __init__(self, agent_id: _Optional[int] = ...) -> None: ...

class FetchJointMapRequest(_message.Message):
    __slots__ = ("agent_id", "joint_map_json")
    AGENT_ID_FIELD_NUMBER: _ClassVar[int]
    JOINT_MAP_JSON_FIELD_NUMBER: _ClassVar[int]
    agent_id: int
    joint_map_json: str
    def __init__(self, agent_id: _Optional[int] = ..., joint_map_json: _Optional[str] = ...) -> None: ...

class EnvConfig(_message.Message):
    __slots__ = ("agent_ids", "sensors_json", "language_instruction", "virtual_agents")
    AGENT_IDS_FIELD_NUMBER: _ClassVar[int]
    SENSORS_JSON_FIELD_NUMBER: _ClassVar[int]
    LANGUAGE_INSTRUCTION_FIELD_NUMBER: _ClassVar[int]
    VIRTUAL_AGENTS_FIELD_NUMBER: _ClassVar[int]
    agent_ids: _containers.RepeatedScalarFieldContainer[int]
    sensors_json: str
    language_instruction: str
    virtual_agents: bool
    def __init__(self, agent_ids: _Optional[_Iterable[int]] = ..., sensors_json: _Optional[str] = ..., language_instruction: _Optional[str] = ..., virtual_agents: bool = ...) -> None: ...

class EnvId(_message.Message):
    __slots__ = ("id",)
    ID_FIELD_NUMBER: _ClassVar[int]
    id: int
    def __init__(self, id: _Optional[int] = ...) -> None: ...

class ImageData(_message.Message):
    __slots__ = ("sensor_key", "width", "height", "channels", "shm_name", "data")
    SENSOR_KEY_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    CHANNELS_FIELD_NUMBER: _ClassVar[int]
    SHM_NAME_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    sensor_key: str
    width: int
    height: int
    channels: int
    shm_name: str
    data: bytes
    def __init__(self, sensor_key: _Optional[str] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., channels: _Optional[int] = ..., shm_name: _Optional[str] = ..., data: _Optional[bytes] = ...) -> None: ...

class Observation(_message.Message):
    __slots__ = ("robot_states_json", "images", "language_instruction")
    ROBOT_STATES_JSON_FIELD_NUMBER: _ClassVar[int]
    IMAGES_FIELD_NUMBER: _ClassVar[int]
    LANGUAGE_INSTRUCTION_FIELD_NUMBER: _ClassVar[int]
    robot_states_json: str
    images: _containers.RepeatedCompositeFieldContainer[ImageData]
    language_instruction: str
    def __init__(self, robot_states_json: _Optional[str] = ..., images: _Optional[_Iterable[_Union[ImageData, _Mapping]]] = ..., language_instruction: _Optional[str] = ...) -> None: ...

class SensorConfig(_message.Message):
    __slots__ = ("sensors_json",)
    SENSORS_JSON_FIELD_NUMBER: _ClassVar[int]
    sensors_json: str
    def __init__(self, sensors_json: _Optional[str] = ...) -> None: ...

class SensorSessionId(_message.Message):
    __slots__ = ("id",)
    ID_FIELD_NUMBER: _ClassVar[int]
    id: int
    def __init__(self, id: _Optional[int] = ...) -> None: ...

class ROSServiceRequest(_message.Message):
    __slots__ = ("service_type", "service_name", "request_json")
    SERVICE_TYPE_FIELD_NUMBER: _ClassVar[int]
    SERVICE_NAME_FIELD_NUMBER: _ClassVar[int]
    REQUEST_JSON_FIELD_NUMBER: _ClassVar[int]
    service_type: str
    service_name: str
    request_json: str
    def __init__(self, service_type: _Optional[str] = ..., service_name: _Optional[str] = ..., request_json: _Optional[str] = ...) -> None: ...

class ROSServiceResponse(_message.Message):
    __slots__ = ("success", "response_json")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    RESPONSE_JSON_FIELD_NUMBER: _ClassVar[int]
    success: bool
    response_json: str
    def __init__(self, success: bool = ..., response_json: _Optional[str] = ...) -> None: ...

class ViveConfig(_message.Message):
    __slots__ = ("agent_ids", "move_robot", "role", "scale_factor", "rotation_scale_factor", "step_rate", "smoothing_alpha", "deadzone_pos", "deadzone_rot")
    AGENT_IDS_FIELD_NUMBER: _ClassVar[int]
    MOVE_ROBOT_FIELD_NUMBER: _ClassVar[int]
    ROLE_FIELD_NUMBER: _ClassVar[int]
    SCALE_FACTOR_FIELD_NUMBER: _ClassVar[int]
    ROTATION_SCALE_FACTOR_FIELD_NUMBER: _ClassVar[int]
    STEP_RATE_FIELD_NUMBER: _ClassVar[int]
    SMOOTHING_ALPHA_FIELD_NUMBER: _ClassVar[int]
    DEADZONE_POS_FIELD_NUMBER: _ClassVar[int]
    DEADZONE_ROT_FIELD_NUMBER: _ClassVar[int]
    agent_ids: _containers.RepeatedScalarFieldContainer[int]
    move_robot: bool
    role: str
    scale_factor: float
    rotation_scale_factor: float
    step_rate: float
    smoothing_alpha: float
    deadzone_pos: float
    deadzone_rot: float
    def __init__(self, agent_ids: _Optional[_Iterable[int]] = ..., move_robot: bool = ..., role: _Optional[str] = ..., scale_factor: _Optional[float] = ..., rotation_scale_factor: _Optional[float] = ..., step_rate: _Optional[float] = ..., smoothing_alpha: _Optional[float] = ..., deadzone_pos: _Optional[float] = ..., deadzone_rot: _Optional[float] = ...) -> None: ...

class ViveReadyRequest(_message.Message):
    __slots__ = ("timeout",)
    TIMEOUT_FIELD_NUMBER: _ClassVar[int]
    timeout: float
    def __init__(self, timeout: _Optional[float] = ...) -> None: ...

class DeltaValues(_message.Message):
    __slots__ = ("values",)
    VALUES_FIELD_NUMBER: _ClassVar[int]
    values: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, values: _Optional[_Iterable[float]] = ...) -> None: ...

class UncertaintyScore(_message.Message):
    __slots__ = ("score",)
    SCORE_FIELD_NUMBER: _ClassVar[int]
    score: float
    def __init__(self, score: _Optional[float] = ...) -> None: ...

class StreamConfig(_message.Message):
    __slots__ = ("topic", "msg_type", "config_json")
    TOPIC_FIELD_NUMBER: _ClassVar[int]
    MSG_TYPE_FIELD_NUMBER: _ClassVar[int]
    CONFIG_JSON_FIELD_NUMBER: _ClassVar[int]
    topic: str
    msg_type: str
    config_json: str
    def __init__(self, topic: _Optional[str] = ..., msg_type: _Optional[str] = ..., config_json: _Optional[str] = ...) -> None: ...

class StreamResponse(_message.Message):
    __slots__ = ("sdp", "type", "stream_id")
    SDP_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    STREAM_ID_FIELD_NUMBER: _ClassVar[int]
    sdp: str
    type: str
    stream_id: str
    def __init__(self, sdp: _Optional[str] = ..., type: _Optional[str] = ..., stream_id: _Optional[str] = ...) -> None: ...

class UpdateStreamConfig(_message.Message):
    __slots__ = ("stream_id", "config_json")
    STREAM_ID_FIELD_NUMBER: _ClassVar[int]
    CONFIG_JSON_FIELD_NUMBER: _ClassVar[int]
    stream_id: str
    config_json: str
    def __init__(self, stream_id: _Optional[str] = ..., config_json: _Optional[str] = ...) -> None: ...

class ImageFrame(_message.Message):
    __slots__ = ("jpeg_data", "width", "height", "stream_id")
    JPEG_DATA_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    STREAM_ID_FIELD_NUMBER: _ClassVar[int]
    jpeg_data: bytes
    width: int
    height: int
    stream_id: str
    def __init__(self, jpeg_data: _Optional[bytes] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., stream_id: _Optional[str] = ...) -> None: ...

class SubscribeImageRequest(_message.Message):
    __slots__ = ("topic", "msg_type", "stream_id")
    TOPIC_FIELD_NUMBER: _ClassVar[int]
    MSG_TYPE_FIELD_NUMBER: _ClassVar[int]
    STREAM_ID_FIELD_NUMBER: _ClassVar[int]
    topic: str
    msg_type: str
    stream_id: str
    def __init__(self, topic: _Optional[str] = ..., msg_type: _Optional[str] = ..., stream_id: _Optional[str] = ...) -> None: ...

class UnsubscribeImageRequest(_message.Message):
    __slots__ = ("stream_id",)
    STREAM_ID_FIELD_NUMBER: _ClassVar[int]
    stream_id: str
    def __init__(self, stream_id: _Optional[str] = ...) -> None: ...

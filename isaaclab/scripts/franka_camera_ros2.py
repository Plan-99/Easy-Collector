"""
Franka Panda + ROS2 + MoveIt2 연동

IsaacSim에서 Franka Panda 로봇을 ROS2로 퍼블리쉬하고,
/joint_command 토픽으로 외부(MoveIt2)에서 관절 제어를 받는 예제.

Usage (컨테이너 내부):
    /workspace/isaaclab/_isaac_sim/python.sh scripts/franka_camera_ros2.py

퍼블리쉬 토픽:
    /joint_states          - sensor_msgs/JointState  (로봇 관절 상태)
    /tf                    - tf2_msgs/TFMessage        (좌표 변환)
    /clock                 - rosgraph_msgs/Clock       (시뮬레이션 시간)
    /camera_rgb            - sensor_msgs/Image
    /camera_depth          - sensor_msgs/Image
    /camera_camera_info    - sensor_msgs/CameraInfo
    /camera_pointcloud     - sensor_msgs/PointCloud2

구독 토픽:
    /joint_command         - sensor_msgs/JointState  (MoveIt2로부터 관절 명령)
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import is_prim_path_valid, set_targets
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from pxr import UsdGeom, UsdPhysics, Gf

# DLAA 모드 설정 - GUI의 "auto"와 동일 효과
# DLAA = 네이티브 해상도 DLSS (내부 렌더 해상도 = 출력 해상도, 노이즈 최소)
_s = carb.settings.get_settings()
_s.set("/rtx/post/dlss/enabled", True)
_s.set("/rtx/post/aa/op", 4)          # 4 = DLAA (다운스케일 없음)
_s.set("/rtx/post/dlss/execMode", 3)  # 3 = Quality

# ROS2 브리지 익스텐션 활성화
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
simulation_app.update()


# ---------------------------------------------------------------------------
# ROS2 OmniGraph 퍼블리셔 설정 함수들
# ---------------------------------------------------------------------------

def setup_clock_and_joint_state_publisher(robot_prim_path: str):
    """Clock + 로봇 Joint State를 ROS2로 퍼블리쉬하는 OmniGraph 생성."""
    graph_path = "/RobotStateGraph"

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", "/clock"),
                ("PublishJointState.inputs:topicName", "/joint_states"),
            ],
        },
    )

    # Joint State 퍼블리셔가 Franka 프림을 바라보도록 연결
    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/PublishJointState"),
        "inputs:targetPrim",
        [robot_prim_path],
    )


def setup_robot_tf_publisher(robot_prim_path: str):
    """로봇 링크들의 TF를 ROS2로 퍼블리쉬하는 OmniGraph 생성."""
    graph_path = "/RobotTFGraph"

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishTF.inputs:topicName", "/tf"),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/PublishTF"),
        "inputs:targetPrims",
        [robot_prim_path],
    )


def publish_camera_info(camera: Camera, freq: int):
    """카메라 내부 파라미터(CameraInfo)를 ROS2로 퍼블리쉬."""
    from isaacsim.ros2.bridge import read_camera_info

    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_camera_info"
    frame_id = camera.prim_path.split("/")[-1]

    writer = rep.writers.get("ROS2PublishCameraInfo")
    camera_info = read_camera_info(render_product_path=render_product)
    writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=1,
        topicName=topic_name,
        width=camera_info["width"],
        height=camera_info["height"],
        projectionType=camera_info["projectionType"],
        k=camera_info["k"].reshape([1, 9]),
        r=camera_info["r"].reshape([1, 9]),
        p=camera_info["p"].reshape([1, 12]),
        physicalDistortionModel=camera_info["physicalDistortionModel"],
        physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_rgb(camera: Camera, freq: int):
    """RGB 이미지를 ROS2로 퍼블리쉬."""
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_rgb"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name)
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_depth(camera: Camera, freq: int):
    """깊이 이미지를 ROS2로 퍼블리쉬."""
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_depth"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name)
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_pointcloud(camera: Camera, freq: int):
    """포인트 클라우드를 ROS2로 퍼블리쉬."""
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_pointcloud"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )
    writer = rep.writers.get(rv + "ROS2PublishPointCloud")
    writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name)
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_camera_tf(camera: Camera):
    """카메라 좌표계 TF를 ROS2로 퍼블리쉬."""
    camera_prim = camera.prim_path
    camera_frame_id = camera_prim.split("/")[-1]
    graph_path = "/CameraTFGraph"

    if not is_prim_path_valid(graph_path):
        og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )

    og.Controller.edit(
        graph_path,
        {
            og.Controller.Keys.CREATE_NODES: [
                ("PublishTF_" + camera_frame_id, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                (
                    "PublishRawTF_" + camera_frame_id + "_optical",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishTF_" + camera_frame_id + ".inputs:topicName", "/tf"),
                ("PublishRawTF_" + camera_frame_id + "_optical.inputs:topicName", "/tf"),
                (
                    "PublishRawTF_" + camera_frame_id + "_optical.inputs:parentFrameId",
                    camera_frame_id,
                ),
                (
                    "PublishRawTF_" + camera_frame_id + "_optical.inputs:childFrameId",
                    camera_frame_id + "_optical",
                ),
                (
                    "PublishRawTF_" + camera_frame_id + "_optical.inputs:rotation",
                    [0.5, -0.5, 0.5, 0.5],
                ),
            ],
            og.Controller.Keys.CONNECT: [
                (
                    graph_path + "/OnTick.outputs:tick",
                    "PublishTF_" + camera_frame_id + ".inputs:execIn",
                ),
                (
                    graph_path + "/OnTick.outputs:tick",
                    "PublishRawTF_" + camera_frame_id + "_optical.inputs:execIn",
                ),
                (
                    graph_path + "/IsaacClock.outputs:simulationTime",
                    "PublishTF_" + camera_frame_id + ".inputs:timeStamp",
                ),
                (
                    graph_path + "/IsaacClock.outputs:simulationTime",
                    "PublishRawTF_" + camera_frame_id + "_optical.inputs:timeStamp",
                ),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/PublishTF_" + camera_frame_id),
        "inputs:targetPrims",
        [camera_prim],
    )


# ---------------------------------------------------------------------------
# 오브젝트 관리 클래스
# ---------------------------------------------------------------------------

class TargetObject:
    """테이블 위의 개별 작업 대상 오브젝트."""

    def __init__(self, name: str, stage, size: float = 0.06,
                 color: tuple = (1.0, 0.3, 0.2), shape: str = "cube",
                 height: float | None = None):
        self.name = name
        self.prim_path = f"/World/{name}"
        self.size = size
        self.color = color
        self._stage = stage

        if shape == "cube":
            geom = UsdGeom.Cube.Define(stage, self.prim_path)
            geom.GetSizeAttr().Set(size)
            self.half_height = size / 2.0
        elif shape == "sphere":
            geom = UsdGeom.Sphere.Define(stage, self.prim_path)
            geom.GetRadiusAttr().Set(size / 2.0)
            self.half_height = size / 2.0
        elif shape == "cylinder":
            actual_height = height if height is not None else size
            geom = UsdGeom.Cylinder.Define(stage, self.prim_path)
            geom.GetRadiusAttr().Set(size / 2.0)
            geom.GetHeightAttr().Set(actual_height)
            self.half_height = actual_height / 2.0

        self._xform = UsdGeom.Xformable(stage.GetPrimAtPath(self.prim_path))
        self._translate_op = self._xform.AddTranslateOp()
        geom.CreateDisplayColorAttr([color])

        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(self.prim_path))
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(self.prim_path))

        self._position = np.array([0.0, 0.0, 0.0])

    def set_position(self, position: np.ndarray):
        """오브젝트 위치를 설정."""
        self._position = position.copy()
        self._translate_op.Set(Gf.Vec3d(*position.tolist()))

    def get_position(self) -> np.ndarray:
        """현재 오브젝트 위치를 USD에서 읽어 반환."""
        prim = self._stage.GetPrimAtPath(self.prim_path)
        xform = UsdGeom.Xformable(prim)
        transform = xform.ComputeLocalToWorldTransform(0)
        t = transform.ExtractTranslation()
        return np.array([t[0], t[1], t[2]])


class ObjectManager:
    """여러 오브젝트를 생성, 랜덤 배치, ROS2 퍼블리쉬하는 매니저."""

    def __init__(self, stage, table_center: np.ndarray, table_scale: np.ndarray):
        self._stage = stage
        self.objects: list[TargetObject] = []

        # 테이블 표면 범위 계산
        half_x = table_scale[0] / 2.0
        half_y = table_scale[1] / 2.0
        table_surface_z = table_center[2] + table_scale[2] / 2.0
        coverage_ratio = 0.5 
    
        reduced_half_x = half_x * coverage_ratio
        reduced_half_y = half_y * coverage_ratio

        self._x_range = (table_center[0] - reduced_half_x, table_center[0] + reduced_half_x)
        self._y_range = (table_center[1] - reduced_half_y, table_center[1] + reduced_half_y)
        self._surface_z = table_surface_z

    def add_object(self, name: str, size: float = 0.06,
                   color: tuple = (1.0, 0.3, 0.2), shape: str = "cube",
                   height: float | None = None) -> TargetObject:
        """오브젝트를 생성하고 매니저에 추가."""
        obj = TargetObject(name, self._stage, size, color, shape, height)
        self.objects.append(obj)
        return obj

    def randomize_all(self):
        """모든 오브젝트를 테이블 위 랜덤 위치에 배치. 겹치지 않도록 처리."""
        positions = []
        for obj in self.objects:
            for _ in range(100):  # 최대 100번 시도
                x = np.random.uniform(*self._x_range)
                y = np.random.uniform(*self._y_range)
                z = self._surface_z + obj.half_height
                pos = np.array([x, y, z])

                # 기존 오브젝트와 겹침 검사
                too_close = False
                for prev_pos, prev_obj in zip(positions, self.objects):
                    min_dist = (obj.size + prev_obj.size) / 2.0 + 0.02
                    if np.linalg.norm(pos[:2] - prev_pos[:2]) < min_dist:
                        too_close = True
                        break
                if not too_close:
                    break

            obj.set_position(pos)
            positions.append(pos)

    def setup_tf_publisher(self):
        """모든 오브젝트의 TF를 /tf로 퍼블리쉬하는 OmniGraph 생성."""
        graph_path = "/ObjectTFGraph"

        og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishTF.inputs:topicName", "/tf"),
                ],
            },
        )

        stage = omni.usd.get_context().get_stage()
        set_targets(
            stage.GetPrimAtPath(graph_path + "/PublishTF"),
            "inputs:targetPrims",
            [obj.prim_path for obj in self.objects],
        )

    def setup_pose_publisher(self, robot_prim_path: str):
        """오브젝트별 Marker로 로봇 base 기준 상대 좌표를 퍼블리쉬 (구체 시각화)."""
        import rclpy
        from visualization_msgs.msg import Marker

        self._robot_prim_path = robot_prim_path

        try:
            rclpy.init()
        except RuntimeError:
            pass  # 이미 초기화된 경우

        self._ros_node = rclpy.create_node("object_pose_publisher")
        self._marker_pubs = {}
        for obj in self.objects:
            topic = f"/object_markers/{obj.name}"
            self._marker_pubs[obj.name] = self._ros_node.create_publisher(Marker, topic, 10)

    def _get_robot_base_transform(self):
        """로봇 base 프레임의 월드 변환 행렬(역행렬)을 반환."""
        prim = self._stage.GetPrimAtPath(self._robot_prim_path)
        xform = UsdGeom.Xformable(prim)
        world_tf = xform.ComputeLocalToWorldTransform(0)
        return world_tf.GetInverse()

    def publish_poses(self):
        """오브젝트별 Marker(구체)로 로봇 base 기준 상대 좌표를 퍼블리쉬."""
        from visualization_msgs.msg import Marker

        inv_base_tf = self._get_robot_base_transform()

        for i, obj in enumerate(self.objects):
            pos = obj.get_position()
            rel_pt = inv_base_tf.Transform(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))

            marker = Marker()
            marker.header.frame_id = "panda_link0"
            marker.ns = obj.name
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = rel_pt[0]
            marker.pose.position.y = rel_pt[1]
            marker.pose.position.z = rel_pt[2]
            marker.pose.orientation.w = 1.0
            # 오브젝트 크기에 맞춘 구체
            marker.scale.x = obj.size
            marker.scale.y = obj.size
            marker.scale.z = obj.size
            # 오브젝트 색상 사용
            marker.color.r = float(obj.color[0])
            marker.color.g = float(obj.color[1])
            marker.color.b = float(obj.color[2])
            marker.color.a = 0.8

            self._marker_pubs[obj.name].publish(marker)

    def get_all_positions(self) -> dict:
        """모든 오브젝트의 이름과 위치를 딕셔너리로 반환."""
        return {obj.name: obj.get_position() for obj in self.objects}

    def print_positions(self):
        """현재 모든 오브젝트 위치를 출력."""
        for obj in self.objects:
            pos = obj.get_position()
            print(f"  {obj.name}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")


def setup_joint_command_subscriber(robot_prim_path: str):
    """MoveIt2로부터 /joint_command 토픽을 구독하여 로봇을 제어하는 OmniGraph 생성."""
    graph_path = "/JointCommandGraph"

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubscribeJointState.inputs:topicName", "/joint_command"),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/ArticulationController"),
        "inputs:targetPrim",
        [robot_prim_path],
    )


# ---------------------------------------------------------------------------
# 메인
# ---------------------------------------------------------------------------

def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_ground_plane()

    # 조명
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path="/World/DomeLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 1500.0},
    )

    # 테이블 (간단한 박스로 대체) - Collider만 (고정 물체)
    stage = omni.usd.get_context().get_stage()
    table = UsdGeom.Cube.Define(stage, "/World/Table")
    table.GetSizeAttr().Set(1.0)
    table.AddScaleOp().Set(Gf.Vec3d(0.8, 0.6, 0.05))
    table.AddTranslateOp().Set(Gf.Vec3d(0.4, 0.0, 0.4))
    table.CreateDisplayColorAttr([(0.55, 0.4, 0.25)])
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())

    # 오브젝트 매니저 (테이블 위 오브젝트 관리)
    table_center = np.array([0.4, 0.0, 0.4])
    table_scale = np.array([0.8, 0.6, 0.05])
    obj_manager = ObjectManager(stage, table_center, table_scale)

    obj_manager.add_object("RedCube", size=0.03, color=(1.0, 0.3, 0.2), shape="cube")
    obj_manager.add_object("GreenCube", size=0.05, color=(0.2, 0.9, 0.3), shape="cube")
    obj_manager.add_object("BlueSphere", size=0.05, color=(0.2, 0.4, 1.0), shape="sphere")
    obj_manager.add_object("WhitePlate", size=0.12, color=(0.95, 0.95, 0.95), shape="cylinder", height=0.015)

    obj_manager.randomize_all()

    # Franka Panda 스폰 (USD 자동 로드)
    robot_prim_path = "/World/Franka"
    franka = Franka(prim_path=robot_prim_path, name="franka")

    # 카메라: 로봇 정면 비스듬히 위에서 내려다보는 위치
    camera = Camera(
        prim_path="/World/Camera",
        position=np.array([2.9, 0.0, 2.3]),
        frequency=30,
        resolution=(640, 480),
        orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 40.0, 180.0]), degrees=True),
    )

    # 씬에 추가 후 리셋
    world.scene.add(franka)
    world.reset()
    obj_manager.randomize_all()
    camera.initialize()

    # 렌더링 파이프라인 워밍업
    print("[INFO] 렌더링 워밍업 중...")
    for _ in range(15):
        world.step(render=True)

    # ROS2 퍼블리셔 설정
    print("[INFO] ROS2 퍼블리셔 설정 중...")
    setup_clock_and_joint_state_publisher(robot_prim_path)
    setup_robot_tf_publisher(robot_prim_path)
    setup_joint_command_subscriber(robot_prim_path)
    obj_manager.setup_tf_publisher()
    obj_manager.setup_pose_publisher(robot_prim_path)

    publish_freq = 30
    publish_camera_tf(camera)
    publish_camera_info(camera, publish_freq)
    publish_rgb(camera, publish_freq)
    # publish_depth(camera, publish_freq)
    # publish_pointcloud(camera, publish_freq)

    print("\n" + "=" * 65)
    print("  Franka Panda + MoveIt2 연동 대기 중")
    print("=" * 65)
    print("\n  퍼블리쉬: /joint_states, /tf, /clock, /camera_*, /object_poses")
    print("  구독:     /joint_command (MoveIt2 → IsaacSim)")
    print(f"\n  오브젝트 ({len(obj_manager.objects)}개):")
    obj_manager.print_positions()
    print("\n  오브젝트 TF 프레임: " + ", ".join(obj.name for obj in obj_manager.objects))
    print("\n  MoveIt2 Docker 컨테이너에서 로봇을 제어하세요.")
    print("=" * 65 + "\n")

    # 시뮬레이션 루프 (MoveIt2로부터 /joint_command를 받아 자동 적용)
    was_stopped = False
    while simulation_app.is_running():
        if world.is_stopped():
            was_stopped = True
            world.step(render=True)
            continue
        if was_stopped:
            world.reset()
            obj_manager.randomize_all()
            was_stopped = False
        world.step(render=True)
        obj_manager.publish_poses()

    simulation_app.close()


if __name__ == "__main__":
    main()

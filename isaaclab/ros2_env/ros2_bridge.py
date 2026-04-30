"""
ROS2 OmniGraph 퍼블리셔/구독자 설정 함수들.

모든 isaacsim import는 함수 내부에서 수행 (SimulationApp 생성 후 호출되므로 안전).
"""

# 시뮬레이션 토픽 네임스페이스
NS = "/simulation"


def setup_clock_and_joint_state_publisher(robot_prim_path: str):
    """Clock + Joint State 퍼블리셔 OmniGraph."""
    import omni.graph.core as og
    import omni.usd
    from isaacsim.core.utils.prims import set_targets

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
                ("PublishClock.inputs:topicName", f"{NS}/clock"),
                ("PublishJointState.inputs:topicName", f"{NS}/joint_states"),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/PublishJointState"),
        "inputs:targetPrim",
        [robot_prim_path],
    )


def setup_robot_tf_publisher(robot_prim_path: str):
    """로봇 링크 TF 퍼블리셔 OmniGraph."""
    import omni.graph.core as og
    import omni.usd
    from isaacsim.core.utils.prims import set_targets

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
                ("PublishTF.inputs:topicName", f"{NS}/tf"),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/PublishTF"),
        "inputs:targetPrims",
        [robot_prim_path],
    )


def setup_joint_command_subscriber(robot_prim_path: str):
    """/joint_command 구독 → ArticulationController OmniGraph."""
    import omni.graph.core as og
    import omni.usd
    from isaacsim.core.utils.prims import set_targets

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
                ("SubscribeJointState.inputs:topicName", f"{NS}/joint_command_internal"),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/ArticulationController"),
        "inputs:targetPrim",
        [robot_prim_path],
    )


def setup_object_tf_publisher(object_prim_paths: list[str]):
    """오브젝트 TF 퍼블리셔 OmniGraph."""
    import omni.graph.core as og
    import omni.usd
    from isaacsim.core.utils.prims import set_targets

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
                ("PublishTF.inputs:topicName", f"{NS}/tf"),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/PublishTF"),
        "inputs:targetPrims",
        object_prim_paths,
    )


def publish_camera_tf(camera):
    """카메라 TF 퍼블리셔 OmniGraph."""
    import omni.graph.core as og
    import omni.usd
    from isaacsim.core.utils.prims import is_prim_path_valid, set_targets

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
                ("PublishTF_" + camera_frame_id + ".inputs:topicName", f"{NS}/tf"),
                ("PublishRawTF_" + camera_frame_id + "_optical.inputs:topicName", f"{NS}/tf"),
                ("PublishRawTF_" + camera_frame_id + "_optical.inputs:parentFrameId", camera_frame_id),
                ("PublishRawTF_" + camera_frame_id + "_optical.inputs:childFrameId", camera_frame_id + "_optical"),
                ("PublishRawTF_" + camera_frame_id + "_optical.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),
            ],
            og.Controller.Keys.CONNECT: [
                (graph_path + "/OnTick.outputs:tick", "PublishTF_" + camera_frame_id + ".inputs:execIn"),
                (graph_path + "/OnTick.outputs:tick", "PublishRawTF_" + camera_frame_id + "_optical.inputs:execIn"),
                (graph_path + "/IsaacClock.outputs:simulationTime", "PublishTF_" + camera_frame_id + ".inputs:timeStamp"),
                (graph_path + "/IsaacClock.outputs:simulationTime", "PublishRawTF_" + camera_frame_id + "_optical.inputs:timeStamp"),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    set_targets(
        stage.GetPrimAtPath(graph_path + "/PublishTF_" + camera_frame_id),
        "inputs:targetPrims",
        [camera_prim],
    )


def publish_camera_info(camera, freq: int):
    """카메라 CameraInfo 퍼블리셔."""
    import omni.graph.core as og
    import omni.syntheticdata
    import omni.replicator.core as rep
    from isaacsim.ros2.bridge import read_camera_info

    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = f"{NS}/{camera.name}_camera_info"
    frame_id = camera.prim_path.split("/")[-1]

    writer = rep.writers.get("ROS2PublishCameraInfo")
    camera_info = read_camera_info(render_product_path=render_product)
    writer.initialize(
        frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name,
        width=camera_info["width"], height=camera_info["height"],
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


def publish_rgb(camera, freq: int):
    """RGB 이미지 퍼블리셔."""
    import omni.graph.core as og
    import omni.syntheticdata
    import omni.syntheticdata._syntheticdata as sd
    import omni.replicator.core as rep

    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = f"{NS}/{camera.name}_rgb"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name)
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_depth(camera, freq: int):
    """깊이 이미지 퍼블리셔."""
    import omni.graph.core as og
    import omni.syntheticdata
    import omni.syntheticdata._syntheticdata as sd
    import omni.replicator.core as rep

    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = f"{NS}/{camera.name}_depth"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name)
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_pointcloud(camera, freq: int):
    """포인트 클라우드 퍼블리셔."""
    import omni.graph.core as og
    import omni.syntheticdata
    import omni.syntheticdata._syntheticdata as sd
    import omni.replicator.core as rep

    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = f"{NS}/{camera.name}_pointcloud"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
    writer = rep.writers.get(rv + "ROS2PublishPointCloud")
    writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1, topicName=topic_name)
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

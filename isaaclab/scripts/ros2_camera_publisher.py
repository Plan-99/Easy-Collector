"""
ROS2 Camera Publisher for Isaac Sim (Headless)

Launches Isaac Sim headless, creates a simple scene with a camera,
and publishes RGB, Depth, CameraInfo, and PointCloud to ROS2 topics.

Usage (inside container):
    /workspace/isaaclab/_isaac_sim/python.sh scripts/ros2_camera_publisher.py

Verify with (another terminal):
    ros2 topic list
    ros2 topic hz /camera_rgb
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

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
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from pxr import UsdGeom, Gf

# Enable ROS2 Bridge extension
enable_extension("isaacsim.ros2.bridge")

# Wait for extension to load
simulation_app.update()
simulation_app.update()


def publish_camera_info(camera: Camera, freq: int):
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
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_rgb"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=1,
        topicName=topic_name,
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_depth(camera: Camera, freq: int):
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_depth"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=1,
        topicName=topic_name,
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_pointcloud_from_depth(camera: Camera, freq: int):
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_pointcloud"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )
    writer = rep.writers.get(rv + "ROS2PublishPointCloud")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=1,
        topicName=topic_name,
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_camera_tf(camera: Camera):
    camera_prim = camera.prim_path
    camera_frame_id = camera_prim.split("/")[-1]
    ros_camera_graph_path = "/CameraTFActionGraph"

    if not is_prim_path_valid(ros_camera_graph_path):
        og.Controller.edit(
            {
                "graph_path": ros_camera_graph_path,
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
        ros_camera_graph_path,
        {
            og.Controller.Keys.CREATE_NODES: [
                ("PublishTF_" + camera_frame_id, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                (
                    "PublishRawTF_" + camera_frame_id + "_world",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishTF_" + camera_frame_id + ".inputs:topicName", "/tf"),
                ("PublishRawTF_" + camera_frame_id + "_world.inputs:topicName", "/tf"),
                (
                    "PublishRawTF_" + camera_frame_id + "_world.inputs:parentFrameId",
                    camera_frame_id,
                ),
                (
                    "PublishRawTF_" + camera_frame_id + "_world.inputs:childFrameId",
                    camera_frame_id + "_world",
                ),
                (
                    "PublishRawTF_" + camera_frame_id + "_world.inputs:rotation",
                    [0.5, -0.5, 0.5, 0.5],
                ),
            ],
            og.Controller.Keys.CONNECT: [
                (
                    ros_camera_graph_path + "/OnTick.outputs:tick",
                    "PublishTF_" + camera_frame_id + ".inputs:execIn",
                ),
                (
                    ros_camera_graph_path + "/OnTick.outputs:tick",
                    "PublishRawTF_" + camera_frame_id + "_world.inputs:execIn",
                ),
                (
                    ros_camera_graph_path + "/IsaacClock.outputs:simulationTime",
                    "PublishTF_" + camera_frame_id + ".inputs:timeStamp",
                ),
                (
                    ros_camera_graph_path + "/IsaacClock.outputs:simulationTime",
                    "PublishRawTF_" + camera_frame_id + "_world.inputs:timeStamp",
                ),
            ],
        },
    )

    set_targets(
        prim_path=ros_camera_graph_path + "/PublishTF_" + camera_frame_id,
        attribute="inputs:targetPrims",
        target_prim_paths=[camera_prim],
    )


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_ground_plane()

    # Add light
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path="/World/DomeLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 1000.0},
    )

    # Add objects
    stage = omni.usd.get_context().get_stage()

    cube = UsdGeom.Cube.Define(stage, "/World/Cube")
    cube.GetSizeAttr().Set(0.1)
    cube.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 0.05))
    cube.CreateDisplayColorAttr([(0.2, 0.6, 1.0)])

    sphere = UsdGeom.Sphere.Define(stage, "/World/Sphere")
    sphere.GetRadiusAttr().Set(0.08)
    sphere.AddTranslateOp().Set(Gf.Vec3d(0.3, 0.3, 0.08))
    sphere.CreateDisplayColorAttr([(1.0, 0.3, 0.2)])

    # Create camera
    camera = Camera(
        prim_path="/World/camera",
        position=np.array([1.0, 0.0, 0.5]),
        frequency=30,
        resolution=(640, 480),
        orientation=rot_utils.euler_angles_to_quats(
            np.array([0, 25, 180]), degrees=True
        ),
    )

    world.reset()
    camera.initialize()

    # Wait for render products
    for _ in range(10):
        world.step(render=True)

    # Setup ROS2 publishers
    publish_freq = 30
    publish_camera_tf(camera)
    publish_camera_info(camera, publish_freq)
    publish_rgb(camera, publish_freq)
    publish_depth(camera, publish_freq)
    publish_pointcloud_from_depth(camera, publish_freq)

    print("\n" + "=" * 60)
    print("ROS2 Camera Publisher is running!")
    print("=" * 60)
    print("\nPublishing topics:")
    print("  /camera_rgb           - sensor_msgs/Image (RGB)")
    print("  /camera_depth         - sensor_msgs/Image (Depth)")
    print("  /camera_camera_info   - sensor_msgs/CameraInfo")
    print("  /camera_pointcloud    - sensor_msgs/PointCloud2")
    print("  /tf                   - tf2_msgs/TFMessage")
    print("  /clock                - rosgraph_msgs/Clock")
    print("\nVerify with: ros2 topic list")
    print("=" * 60 + "\n")

    # Simulation loop
    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()

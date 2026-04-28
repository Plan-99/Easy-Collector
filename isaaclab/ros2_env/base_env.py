"""
IsaacSim + ROS2 환경의 베이스 클래스.

서브클래스는 setup_scene()만 오버라이드하여 씬을 구성하면 됨.
모든 isaacsim import는 메서드 내부에서 수행 (SimulationApp 생성 후).
"""

import numpy as np

from ros2_env.objects import ObjectManager
from ros2_env import ros2_bridge


class BaseEnv:
    """IsaacSim + ROS2 환경의 베이스 클래스.

    사용법:
        class MyEnv(BaseEnv):
            def setup_scene(self):
                self.add_robot("franka")
                self.add_table(center=[0.4, 0, 0.4], scale=[0.8, 0.6, 0.05])
                self.add_object("Cube", size=0.05, shape="cube")
                self.add_camera(position=[2.9, 0, 2.3], orientation=[0, 40, 180])

        MyEnv(headless=False).run()
    """

    def __init__(self, headless: bool = False):
        from isaacsim import SimulationApp
        self._app = SimulationApp({"headless": headless})

        self._setup_rendering()
        self._enable_ros2()

        self._robot = None
        self._robot_prim_path = None
        self._cameras = []  # list of dicts: {camera, freq, rgb, depth, pointcloud}
        self._obj_manager = None
        self._table_center = None
        self._table_scale = None
        self._mimic_joints = []  # (source_joint_prim, target_joint_prim, multiplier, offset)
        self._mimic_joint_names = {}  # {mimic_name: (source_name, multiplier, offset)}
        self._relay_node = None
        self._lighting_randomization = None  # dict: intensity_range, color_temp_range 등
        self._background_randomization = None

    def _setup_rendering(self):
        import carb
        s = carb.settings.get_settings()
        s.set("/rtx/post/dlss/enabled", True)
        s.set("/rtx/post/aa/op", 4)          # DLAA
        s.set("/rtx/post/dlss/execMode", 3)  # Quality

    def _enable_ros2(self):
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        self._app.update()
        self._app.update()

    # ── 씬 구성 메서드 (서브클래스에서 호출) ──────────────────


    def add_mimic_joint(self, mimic_name: str, source_name: str,
                        multiplier: float = -1.0, offset: float = 0.0):
        """Mimic joint 매핑을 수동 등록.

        URDF에 mimic 태그가 없는 경우 setup_scene()에서 직접 호출.
        joint_command에 source_name이 있고 mimic_name이 없으면
        mimic_name = multiplier * source_name + offset 으로 자동 추가.

        예: add_mimic_joint("joint8", "joint7", multiplier=-1.0)
        """
        self._mimic_joint_names[mimic_name] = (source_name, multiplier, offset)
        print(f"[INFO] Mimic joint 수동 등록: {mimic_name} = {multiplier} * {source_name} + {offset}")

    def add_robot(self, robot_type: str, prim_path: str | None = None,
                  usd_path: str | None = None):
        """로봇을 씬에 추가.

        기본 제공 로봇: "franka"
        커스텀 로봇:    usd_path로 USD 파일 직접 지정
            add_robot("piper", usd_path="/path/to/piper.usd")
        """
        if robot_type == "franka":
            from isaacsim.robot.manipulators.examples.franka import Franka
            prim_path = prim_path or "/World/Franka"
            self._robot = Franka(prim_path=prim_path, name="robot")
        elif usd_path is not None:
            import omni.usd
            prim_path = prim_path or f"/World/{robot_type.capitalize()}"
            stage = omni.usd.get_context().get_stage()
            prim = stage.DefinePrim(prim_path, "Xform")
            prim.GetReferences().AddReference(usd_path)
            self._robot = None  # 커스텀 로봇은 world.scene.add 하지 않음
        else:
            raise ValueError(f"지원하지 않는 로봇: {robot_type}. usd_path를 지정하세요.")

        self._robot_prim_path = prim_path

    def add_robot_from_urdf(self, name: str, urdf_path: str,
                            prim_path: str | None = None,
                            position: list | None = None,
                            color: tuple | None = None):
        """URDF 파일에서 로봇을 임포트하여 씬에 추가.

        IsaacSim URDF Importer를 사용하여 자동으로 USD로 변환 후 로드.
        package:// 경로는 URDF 파일 기준으로 자동 해석됨.

        Args:
            position: [x, y, z] 월드 좌표. 지정 시 로봇 루트 prim을 해당 위치로 이동.
            color: (R, G, B) 0-1 범위. 지정 시 로봇 전체 메쉬 색상을 변경.
        """
        import omni.kit.commands
        import omni.usd

        prim_path = prim_path or f"/World/{name.capitalize()}"

        # URDFCreateImportConfig로 올바른 기본값의 ImportConfig 생성
        _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.set_fix_base(True)
        import_config.set_make_default_prim(False)
        import_config.set_create_physics_scene(False)
        import_config.set_merge_fixed_joints(False)
        import_config.set_self_collision(False)
        import_config.set_distance_scale(1.0)
        import_config.set_parse_mimic(True)  # mimic → 일반 joint 변환 (물리 constraint 제거)

        # 현재 스테이지에 직접 import
        result = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=import_config,
        )

        imported_path = result[1] if result and result[1] else None
        print(f"[INFO] URDF 임포트 결과: {imported_path}")

        for _ in range(10):
            self._app.update()

        if imported_path:
            prim_path = imported_path

        # ArticulationRootAPI가 없으면 수동으로 적용
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            from pxr import UsdPhysics, PhysxSchema
            if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                print(f"[WARN] ArticulationRootAPI 없음, 수동 적용: {prim_path}")
                UsdPhysics.ArticulationRootAPI.Apply(prim)
                PhysxSchema.PhysxArticulationAPI.Apply(prim)
                for _ in range(5):
                    self._app.update()
            else:
                print(f"[INFO] ArticulationRootAPI 확인됨: {prim_path}")

        # 로봇 위치 설정 (URDF 임포트가 이미 translate op을 생성하므로 기존 op 재사용)
        if position is not None:
            from pxr import UsdGeom, Gf
            xform = UsdGeom.Xformable(prim)
            for op in xform.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:translate":
                    op.Set(Gf.Vec3d(*position))
                    break
            for _ in range(5):
                self._app.update()

        self._robot = None
        self._robot_prim_path = prim_path

        # URDF에서 mimic joint 파싱하여 등록
        self._parse_mimic_joints(urdf_path)

        # 그리퍼 링크에 높은 마찰력 적용
        self._apply_gripper_friction(str(prim_path))

        # joint drive 강성/감쇠 튜닝 (진동 방지)
        self._tune_joint_drives(str(prim_path))

        # 로봇 색상 변경
        if color is not None:
            self._set_robot_color(str(prim_path), color)

        print(f"[INFO] URDF 로봇 로드 완료: {self._robot_prim_path}")

    def _parse_mimic_joints(self, urdf_path: str):
        """URDF에서 mimic joint를 파싱하여 시뮬레이션 루프에서 동기화할 목록 등록."""
        import xml.etree.ElementTree as ET
        import omni.usd
        from pxr import UsdPhysics

        tree = ET.parse(urdf_path)
        root = tree.getroot()
        stage = omni.usd.get_context().get_stage()

        for joint_elem in root.findall(".//joint"):
            mimic_elem = joint_elem.find("mimic")
            if mimic_elem is None:
                continue

            target_name = joint_elem.get("name")
            source_name = mimic_elem.get("joint")
            multiplier = float(mimic_elem.get("multiplier", "1"))
            offset = float(mimic_elem.get("offset", "0"))

            # USD 스테이지에서 조인트 prim 찾기
            source_prim = None
            target_prim = None
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.DriveAPI):
                    prim_name = prim.GetName()
                    if prim_name == source_name:
                        source_prim = prim
                    elif prim_name == target_name:
                        target_prim = prim

            # joint 이름 매핑 (relay용) — prim 찾기와 무관하게 항상 등록
            self._mimic_joint_names[target_name] = (source_name, multiplier, offset)

            if source_prim and target_prim:
                self._mimic_joints.append((source_prim, target_prim, multiplier, offset))
                print(f"[INFO] Mimic joint 등록: {target_name} = {multiplier} * {source_name} + {offset}")
            else:
                print(f"[WARN] Mimic joint prim을 찾을 수 없음: {source_name} -> {target_name}")

    def _tune_joint_drives(self, robot_prim_path: str):
        """URDF import된 joint drive의 stiffness/damping을 높여 진동 방지."""
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()

        # revolute joint: 높은 stiffness + damping
        # prismatic (gripper): 낮은 stiffness + damping
        revolute_stiffness = 10000.0
        revolute_damping = 1000.0
        prismatic_stiffness = 5000.0
        prismatic_damping = 500.0

        for prim in stage.Traverse():
            if not prim.GetPath().HasPrefix(robot_prim_path):
                continue

            # revolute drive
            if prim.HasAPI(UsdPhysics.DriveAPI):
                for drive_type in ["angular", "linear"]:
                    try:
                        drive = UsdPhysics.DriveAPI(prim, drive_type)
                        if drive.GetStiffnessAttr().Get() is not None:
                            if drive_type == "angular":
                                drive.GetStiffnessAttr().Set(revolute_stiffness)
                                drive.GetDampingAttr().Set(revolute_damping)
                            else:
                                drive.GetStiffnessAttr().Set(prismatic_stiffness)
                                drive.GetDampingAttr().Set(prismatic_damping)
                            print(f"[INFO] Drive 튜닝: {prim.GetPath()} "
                                  f"({drive_type}, stiffness={drive.GetStiffnessAttr().Get()}, "
                                  f"damping={drive.GetDampingAttr().Get()})")
                    except Exception:
                        pass

    def _apply_gripper_friction(self, robot_prim_path: str):
        """그리퍼 링크(link7, link8)에 높은 마찰력 물리 머티리얼 적용."""
        import omni.usd
        from pxr import UsdPhysics, UsdShade

        stage = omni.usd.get_context().get_stage()
        gripper_links = ["link7", "link8"]

        for prim in stage.Traverse():
            if prim.GetName() in gripper_links and prim.GetPath().HasPrefix(robot_prim_path):
                mat_path = f"{prim.GetPath()}/GripperMaterial"
                UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(mat_path))
                mat_api = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(mat_path))
                mat_api.CreateStaticFrictionAttr(1.0)
                mat_api.CreateDynamicFrictionAttr(1.0)
                mat_api.CreateRestitutionAttr(0.0)
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    UsdShade.Material(stage.GetPrimAtPath(mat_path)),
                    UsdShade.Tokens.weakerThanDescendants,
                    "physics",
                )
                print(f"[INFO] 그리퍼 마찰력 적용: {prim.GetPath()}")

    def _set_robot_color(self, robot_prim_path: str, color: tuple):
        """로봇의 기존 머티리얼 셰이더의 diffuse 색상을 직접 변경.

        URDF 임포트 시 생성되는 머티리얼은 {robot}/Looks/ 하위에 위치.
        각 머티리얼의 셰이더에서 diffuse 계열 입력을 찾아 색상을 덮어쓴다.
        """
        import omni.usd
        from pxr import UsdShade, Gf

        stage = omni.usd.get_context().get_stage()

        looks_path = f"{robot_prim_path}/Looks"
        looks_prim = stage.GetPrimAtPath(looks_path)
        if not looks_prim.IsValid():
            print(f"[WARN] {looks_path} not found, skipping color change")
            return

        diffuse_names = [
            "diffuse_color_constant", "diffuseColor", "base_color",
            "albedo_color", "color",
        ]

        count = 0
        for mat_prim in looks_prim.GetAllChildren():
            for child in mat_prim.GetAllChildren():
                shader = UsdShade.Shader(child)
                if not shader:
                    continue
                for name in diffuse_names:
                    inp = shader.GetInput(name)
                    if inp and inp.Get() is not None:
                        inp.Set(Gf.Vec3f(*color))
                        count += 1

        print(f"[INFO] 로봇 색상 변경: {color} ({count}개 셰이더 수정)")

    def _sync_mimic_joints(self):
        """mimic joint 위치를 소스 joint에 맞춰 동기화."""
        from pxr import UsdPhysics
        for source_prim, target_prim, multiplier, offset in self._mimic_joints:
            try:
                source_drive = UsdPhysics.DriveAPI(source_prim, "linear")
                target_drive = UsdPhysics.DriveAPI(target_prim, "linear")
                source_pos = source_drive.GetTargetPositionAttr().Get()
                if source_pos is not None:
                    target_pos = multiplier * source_pos + offset
                    target_drive.GetTargetPositionAttr().Set(target_pos)
            except Exception:
                pass

    def add_table(self, center: list, scale: list, color: tuple = (0.55, 0.4, 0.25),
                  spawn_range: list | None = None):
        """테이블(고정 박스 + Collider)을 추가하고 ObjectManager 초기화.

        Args:
            spawn_range: 오브젝트 배치 범위 [[x_min, y_min], [x_max, y_max]].
                         None이면 테이블 면적의 50%를 자동 사용.
        """
        from pxr import UsdGeom, UsdPhysics, Gf
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        table = UsdGeom.Cube.Define(stage, "/World/Table")
        table.GetSizeAttr().Set(1.0)
        table.AddTranslateOp().Set(Gf.Vec3d(*center))
        table.AddScaleOp().Set(Gf.Vec3d(*scale))
        table.CreateDisplayColorAttr([color])
        UsdPhysics.CollisionAPI.Apply(table.GetPrim())

        self._table_center = np.array(center)
        self._table_scale = np.array(scale)
        self._obj_manager = ObjectManager(stage, self._table_center, self._table_scale,
                                          spawn_range=spawn_range)

    def add_object(self, name: str, size: float = 0.06,
                   color: tuple | None = None, shape: str = "cube",
                   height: float | None = None,
                   usd_path: str | None = None, scale: float = 1.0,
                   position_range: list | None = None,
                   orientation_range: list | None = None,
                   deformable: float | None = None,
                   collision_approximation: str = "convexHull",
                   friction: float = 1.0,
                   mass: float | None = None):
        """테이블 위 오브젝트를 추가 (add_table 이후 호출).

        프리미티브: add_object("Cube", size=0.05, shape="cube")
        USD 에셋:  add_object("Mug", usd_path="/path/to/mug.usd", size=0.1, scale=0.01)
        size: 겹침 방지 거리 계산용 (scale 적용 후 실제 크기에 맞게 설정).
        position_range: [[x_max, y_max], [x_min, y_min]]. None이면 테이블 기본 범위.
        orientation_range: [[rx_min, ry_min, rz_min], [rx_max, ry_max, rz_max]] 도 단위.
        deformable: 0.0~1.0 부드러움 정도. 0.0=매우 부드러움, 1.0=단단함. None이면 rigid body.
        collision_approximation: 충돌 근사 방식.
        friction: 마찰 계수 (기본 1.0). 잡기 어려운 물체는 2.0~5.0으로 높이기.
        mass: 질량 (kg). None이면 자동 계산. 가벼울수록 잡기 쉬움.
        """
        if self._obj_manager is None:
            raise RuntimeError("add_table()을 먼저 호출하세요.")
        self._obj_manager.add_object(name, size, color, shape, height, usd_path, scale,
                                     position_range, orientation_range, deformable,
                                     collision_approximation, friction, mass)

    def add_bg_object(self, name: str, usd_path: str | None = None,
                      position: tuple = (0.0, 0.0, 0.0),
                      orientation: tuple | None = None,
                      scale: float = 1.0,
                      box_size: tuple | None = None,
                      color: tuple = (0.5, 0.5, 0.5)):
        """물리 없이 고정 배치되는 배경 오브젝트를 추가.

        USD 에셋:  add_bg_object("Lamp", usd_path="/path/to/lamp.usd", position=(0.5, 0.3, 0.032))
        직육면체:  add_bg_object("Shelf", box_size=(0.4, 0.3, 0.02), position=(0.3, 0, 0.1), color=(0.5, 0.5, 0.5))
        """
        if self._obj_manager is None:
            raise RuntimeError("add_table()을 먼저 호출하세요.")
        self._obj_manager.add_bg_object(name, usd_path, position=position,
                                        orientation=orientation, scale=scale,
                                        box_size=box_size, color=color)

    def add_camera(self, position: list, orientation: list | None = None,
                   resolution: tuple = (640, 480), freq: int = 30,
                   publish_rgb: bool = True, publish_depth: bool = False,
                   publish_pointcloud: bool = False,
                   parent_prim: str | None = None,
                   name: str | None = None,
                   near_plane: float = 0.01,
                   focal_length: float | None = None,
                   position_delta: list | None = None,
                   orientation_delta: list | None = None):
        """카메라를 추가.

        orientation: [roll, pitch, yaw] 도 단위. None이면 회전 없음.
        parent_prim: 로봇 링크 경로 (예: "/piper/link6"). 지정 시 해당 링크에 부착.
        name: 카메라 이름. None이면 자동 생성.
        focal_length: 초점 거리 (mm). 작을수록 광각. 기본 24mm (IsaacSim 기본).
                      예: 5.0=초광각(~120도), 10.0=광각(~90도), 24.0=일반, 50.0=망원.
        position_delta: [dx, dy, dz]. position 기준 ±delta 범위 내 랜덤.
        orientation_delta: [droll, dpitch, dyaw]. orientation 기준 ±delta 범위 내 랜덤. 도 단위.
        """
        import random
        from isaacsim.sensors.camera import Camera

        # position_delta가 있으면 base position에서 ±delta 랜덤
        if position_delta is not None:
            position = [position[i] + random.uniform(-position_delta[i], position_delta[i])
                        for i in range(3)]

        # orientation_delta가 있으면 base orientation에서 ±delta 랜덤
        if orientation_delta is not None and orientation is not None:
            orientation = [orientation[i] + random.uniform(-orientation_delta[i], orientation_delta[i])
                           for i in range(3)]

        cam_name = name or f"camera_{len(self._cameras)}"
        if parent_prim is not None:
            camera_path = f"{parent_prim}/{cam_name}"
        else:
            camera_path = f"/World/{cam_name}"

        camera = Camera(
            prim_path=camera_path,
            name=cam_name,
            frequency=freq,
            resolution=resolution,
        )
        from pxr import Gf, UsdGeom
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(camera_path)
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        if orientation is not None:
            xform.AddRotateXYZOp().Set(Gf.Vec3f(*orientation))

        # 카메라 속성 설정
        cam_prim = UsdGeom.Camera(prim)
        cam_prim.CreateClippingRangeAttr().Set(Gf.Vec2f(near_plane, 1000000.0))
        if focal_length is not None:
            cam_prim.CreateFocalLengthAttr().Set(focal_length)

        self._cameras.append({
            "camera": camera,
            "freq": freq,
            "rgb": publish_rgb,
            "depth": publish_depth,
            "pointcloud": publish_pointcloud,
            "prim_path": camera_path,
            "base_position": list(position) if position_delta else None,
            "position_delta": position_delta,
            "base_orientation": list(orientation) if (orientation_delta and orientation) else None,
            "orientation_delta": orientation_delta,
        })

    def _randomize_cameras(self):
        """position_delta/orientation_delta가 설정된 카메라의 위치를 랜덤화."""
        import random
        from pxr import Gf, UsdGeom
        import omni.usd
        stage = omni.usd.get_context().get_stage()

        for cam_info in self._cameras:
            base_pos = cam_info.get("base_position")
            pos_delta = cam_info.get("position_delta")
            base_ori = cam_info.get("base_orientation")
            ori_delta = cam_info.get("orientation_delta")

            if pos_delta is None and ori_delta is None:
                continue

            prim = stage.GetPrimAtPath(cam_info["prim_path"])
            if not prim.IsValid():
                continue
            xform = UsdGeom.Xformable(prim)

            if pos_delta is not None and base_pos is not None:
                new_pos = [base_pos[i] + random.uniform(-pos_delta[i], pos_delta[i])
                           for i in range(3)]
                for op in xform.GetOrderedXformOps():
                    if op.GetOpName() == "xformOp:translate":
                        op.Set(Gf.Vec3d(*new_pos))
                        break

            if ori_delta is not None and base_ori is not None:
                new_ori = [base_ori[i] + random.uniform(-ori_delta[i], ori_delta[i])
                           for i in range(3)]
                for op in xform.GetOrderedXformOps():
                    if op.GetOpName() == "xformOp:rotateXYZ":
                        op.Set(Gf.Vec3f(*new_ori))
                        break

    def set_lighting_randomization(self,
                                   intensity_range: list | None = None,
                                   color_temp_range: list | None = None,
                                   additional_lights: int = 0,
                                   additional_intensity_range: list | None = None):
        """리셋 시 조명 랜덤화 설정.

        Args:
            intensity_range: DomeLight intensity [min, max]. 예: [800, 2500].
            color_temp_range: 색온도 [min, max] Kelvin. 예: [3500, 6500].
                              3500=따뜻한 노란빛, 6500=차가운 흰빛.
            additional_lights: 추가 DistantLight 개수 (랜덤 방향). 0이면 DomeLight만.
            additional_intensity_range: 추가 조명 intensity [min, max]. 예: [200, 800].
        """
        self._lighting_randomization = {
            "intensity_range": intensity_range or [1000, 2000],
            "color_temp_range": color_temp_range,
            "additional_lights": additional_lights,
            "additional_intensity_range": additional_intensity_range or [200, 800],
        }

    def _randomize_lighting(self):
        """리셋 시 조명 랜덤화 실행."""
        if self._lighting_randomization is None:
            return

        import random
        from pxr import UsdLux
        import omni.usd
        stage = omni.usd.get_context().get_stage()

        cfg = self._lighting_randomization

        # DomeLight intensity 랜덤화
        dome = stage.GetPrimAtPath("/World/DomeLight")
        if dome.IsValid():
            lo, hi = cfg["intensity_range"]
            intensity = random.uniform(lo, hi)
            dome.GetAttribute("inputs:intensity").Set(intensity)

            # 색온도 랜덤화
            if cfg["color_temp_range"] is not None:
                lo_t, hi_t = cfg["color_temp_range"]
                temp = random.uniform(lo_t, hi_t)
                dome.GetAttribute("inputs:enableColorTemperature").Set(True)
                dome.GetAttribute("inputs:colorTemperature").Set(temp)

        # 추가 DistantLight 랜덤화 (방향 + intensity)
        n_lights = cfg["additional_lights"]
        if n_lights > 0:
            from pxr import Gf, UsdGeom
            int_lo, int_hi = cfg["additional_intensity_range"]

            for i in range(n_lights):
                light_path = f"/World/RandomLight_{i}"
                light_prim = stage.GetPrimAtPath(light_path)

                if not light_prim.IsValid():
                    # 첫 리셋 시 생성
                    light_prim = UsdLux.DistantLight.Define(stage, light_path).GetPrim()

                light_prim.GetAttribute("inputs:intensity").Set(
                    random.uniform(int_lo, int_hi)
                )

                xform = UsdGeom.Xformable(light_prim)
                xform.ClearXformOpOrder()
                xform.AddRotateXYZOp().Set(Gf.Vec3f(
                    random.uniform(-60, 60),
                    random.uniform(-60, 60),
                    random.uniform(0, 360),
                ))

    def set_background_randomization(self, brightness_range: list | None = None):
        """리셋 시 배경(바닥+DomeLight) 색상을 랜덤 흑백으로 변경.

        Args:
            brightness_range: [min, max] 0~1 범위. 예: [0.0, 0.3]=검정~진한회색.
                              None이면 비활성화.
        """
        self._background_randomization = brightness_range

    def _randomize_background(self):
        """배경색을 brightness_range 범위 내 랜덤 흑백으로 변경."""
        if self._background_randomization is None:
            return

        import random
        from pxr import Gf, Sdf, UsdLux
        import omni.usd
        stage = omni.usd.get_context().get_stage()

        lo, hi = self._background_randomization
        v = random.uniform(lo, hi)

        # 1) DomeLight color 변경
        for prim in stage.Traverse():
            if prim.IsA(UsdLux.DomeLight):
                tex_attr = prim.GetAttribute("inputs:texture:file")
                if tex_attr and tex_attr.Get():
                    tex_attr.Set(Sdf.AssetPath(""))
                color_attr = prim.GetAttribute("inputs:color")
                if color_attr:
                    color_attr.Set(Gf.Vec3f(v, v, v))

        # 2) 커스텀 바닥 셰이더 색상 변경
        shader = stage.GetPrimAtPath("/World/BackgroundFloor/Material/Shader")
        if shader.IsValid():
            from pxr import UsdShade
            s = UsdShade.Shader(shader)
            dc = s.GetInput("diffuseColor")
            if dc:
                dc.Set(Gf.Vec3f(v, v, v))

    def setup_scene(self):
        """서브클래스에서 오버라이드: 로봇, 테이블, 오브젝트, 카메라를 배치."""
        raise NotImplementedError("setup_scene()을 구현하세요.")

    # ── 실행 ──────────────────────────────────────────────────

    def run(self):
        """전체 시뮬레이션 실행."""
        from isaacsim.core.api import World
        import omni.kit.commands

        world = World(stage_units_in_meters=1.0)

        # 커스텀 바닥 (색상 변경 가능) + 물리 충돌면
        import omni.usd
        from pxr import UsdGeom, UsdPhysics, Gf
        _stage = omni.usd.get_context().get_stage()

        floor = UsdGeom.Cube.Define(_stage, "/World/BackgroundFloor")
        floor.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(floor.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.251))
        xf.AddScaleOp().Set(Gf.Vec3d(20, 20, 0.5))
        # 물리 충돌 (ground plane 대체)
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        # OmniPBR 머티리얼 (RTX 렌더러에서 색상 변경 가능)
        from pxr import UsdShade, Sdf
        mat = UsdShade.Material.Define(_stage, "/World/BackgroundFloor/Material")
        shader = UsdShade.Shader.Define(_stage, "/World/BackgroundFloor/Material/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.3, 0.3, 0.3))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(floor.GetPrim()).Bind(mat)

        # 조명
        omni.kit.commands.execute(
            "CreatePrim",
            prim_path="/World/DomeLight",
            prim_type="DomeLight",
            attributes={"inputs:intensity": 1500.0},
        )

        # 서브클래스에서 씬 구성
        self.setup_scene()

        # 로봇을 world에 추가
        if self._robot is not None:
            world.scene.add(self._robot)

        world.reset()

        # 오브젝트 랜덤 배치
        if self._obj_manager:
            self._obj_manager.randomize_all()

        # 초기 도메인 랜덤화
        self._randomize_lighting()
        self._randomize_background()

        # 카메라 초기화
        for cam_info in self._cameras:
            cam_info["camera"].initialize()

        # 렌더링 워밍업
        print("[INFO] 렌더링 워밍업 중...")
        for _ in range(15):
            world.step(render=True)

        # ROS2 퍼블리셔 설정
        self._setup_ros2_publishers()

        # 환경 리셋 서비스 설정
        self._reset_requested = False
        self._setup_reset_service(world)

        # mimic joint relay 설정 (joint_command → joint_command_internal)
        print(f"[DEBUG] mimic_joint_names = {self._mimic_joint_names}")
        if self._mimic_joint_names:
            self._setup_mimic_relay()
        else:
            print("[WARN] mimic_joint_names가 비어있음! URDF에 mimic 태그가 없거나 add_robot_from_urdf를 사용하지 않음")

        self._print_status()

        # 시뮬레이션 루프
        was_stopped = False
        while self._app.is_running():
            if world.is_stopped():
                was_stopped = True
                world.step(render=True)
                continue
            if was_stopped:
                world.reset()
                if self._obj_manager:
                    self._obj_manager.randomize_all()
                was_stopped = False
            world.step(render=True)
            # mimic joint 동기화는 브릿지에서 처리 (_append_mimic)
            # _sync_mimic_joints의 DriveAPI 경로와 ArticulationController 경로가
            # 충돌하여 joint8이 멈추는 문제가 있으므로 비활성화
            # if self._mimic_joints:
            #     self._sync_mimic_joints()
            if self._obj_manager:
                self._obj_manager.publish_markers()

            # relay + 리셋 서비스 콜백 처리
            import rclpy as _rclpy
            if self._relay_node is not None:
                # 큐에 쌓인 메시지를 모두 처리 (최대 10개)
                for _ in range(10):
                    _rclpy.spin_once(self._relay_node, timeout_sec=0)
            _rclpy.spin_once(self._reset_node, timeout_sec=0)
            if self._reset_requested:
                print("[INFO] 환경 리셋 중 (오브젝트만, 로봇 포즈 유지)...")
                if self._obj_manager:
                    self._obj_manager.randomize_all()
                self._randomize_cameras()
                self._randomize_lighting()
                self._randomize_background()
                for _ in range(15):
                    world.step(render=True)
                self._reset_requested = False
                print("[INFO] 환경 리셋 완료")

        self._cleanup_ros2()
        self._app.close()

    def _cleanup_ros2(self):
        """ROS2 노드 정리 및 shutdown."""
        import rclpy

        print("[INFO] ROS2 정리 중...")
        if self._relay_node is not None:
            self._relay_node.destroy_node()
            self._relay_node = None
        if hasattr(self, '_reset_node') and self._reset_node:
            self._reset_node.destroy_node()
            self._reset_node = None
        if self._obj_manager:
            self._obj_manager.destroy()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("[INFO] ROS2 정리 완료")

    def _setup_reset_service(self, world):
        """환경 리셋 ROS2 서비스 설정."""
        import rclpy
        from std_srvs.srv import Trigger

        try:
            rclpy.init()
        except RuntimeError:
            pass  # 이미 초기화됨

        self._reset_node = rclpy.create_node("env_reset_service")
        self._reset_node.create_service(
            Trigger, "/simulation/reset_env", self._reset_cb
        )
        # 서비스 노드를 몇 번 spin하여 DDS에 등록
        for _ in range(10):
            rclpy.spin_once(self._reset_node, timeout_sec=0)
        print("[INFO] 리셋 서비스 등록 완료: /simulation/reset_env")

    def _reset_cb(self, _request, response):
        self._reset_requested = True
        response.success = True
        response.message = "환경 리셋 요청됨"
        return response

    def _setup_mimic_relay(self):
        """Mimic joint relay 노드 설정.

        /simulation/joint_command 구독 → mimic joint 추가 →
        /simulation/joint_command_internal 발행.
        OmniGraph ArticulationController는 _internal 토픽을 구독.
        시뮬레이션 루프에서 spin_once로 콜백 처리.
        """
        import rclpy
        from sensor_msgs.msg import JointState

        self._relay_node = rclpy.create_node("mimic_joint_relay")
        self._relay_pub = self._relay_node.create_publisher(
            JointState, "/simulation/joint_command_internal", 10
        )

        mimic_map = self._mimic_joint_names  # {mimic: (source, mult, offset)}
        relay_count = [0]

        def relay_cb(msg: JointState):
            names = list(msg.name)
            positions = list(msg.position)
            name_set = set(names)

            added = []
            for mimic_name, (src_name, mult, offset) in mimic_map.items():
                if src_name in name_set and mimic_name not in name_set:
                    src_idx = names.index(src_name)
                    val = mult * positions[src_idx] + offset
                    names.append(mimic_name)
                    positions.append(val)
                    added.append(f"{mimic_name}={val:.4f}")

            out = JointState()
            out.header = msg.header
            out.name = names
            out.position = positions
            out.velocity = msg.velocity
            out.effort = msg.effort
            self._relay_pub.publish(out)

            relay_count[0] += 1
            if relay_count[0] <= 5 or relay_count[0] % 100 == 0:
                print(f"[RELAY #{relay_count[0]}] in={list(msg.name)} added={added} out={names}")

        self._relay_node.create_subscription(
            JointState, "/simulation/joint_command", relay_cb, 10
        )

        # DDS 등록 대기
        for _ in range(10):
            rclpy.spin_once(self._relay_node, timeout_sec=0)

        mimic_str = ", ".join(
            f"{m} = {mult}*{src}+{off}"
            for m, (src, mult, off) in mimic_map.items()
        )
        print(f"[INFO] Mimic relay 설정 완료: {mimic_str}")

    def _setup_ros2_publishers(self):
        """ROS2 OmniGraph 퍼블리셔/구독자 설정."""
        print("[INFO] ROS2 퍼블리셔 설정 중...")

        if self._robot_prim_path:
            ros2_bridge.setup_clock_and_joint_state_publisher(self._robot_prim_path)
            ros2_bridge.setup_robot_tf_publisher(self._robot_prim_path)
            ros2_bridge.setup_joint_command_subscriber(self._robot_prim_path)

        if self._obj_manager and self._obj_manager.objects:
            ros2_bridge.setup_object_tf_publisher(
                [obj.prim_path for obj in self._obj_manager.objects]
            )
            if self._robot_prim_path:
                self._obj_manager.setup_marker_publisher(self._robot_prim_path)

        for cam_info in self._cameras:
            cam = cam_info["camera"]
            freq = cam_info["freq"]
            ros2_bridge.publish_camera_tf(cam)
            ros2_bridge.publish_camera_info(cam, freq)
            if cam_info["rgb"]:
                ros2_bridge.publish_rgb(cam, freq)
            if cam_info["depth"]:
                ros2_bridge.publish_depth(cam, freq)
            if cam_info["pointcloud"]:
                ros2_bridge.publish_pointcloud(cam, freq)

    def _print_status(self):
        """시작 시 상태 출력."""
        print("\n" + "=" * 65)
        print("  IsaacSim + ROS2 환경 실행 중")
        print("=" * 65)
        if self._robot_prim_path:
            print(f"\n  로봇: {self._robot_prim_path}")
            ns = ros2_bridge.NS
            print(f"  퍼블리쉬: {ns}/joint_states, {ns}/tf, {ns}/clock")
            print(f"  구독:     {ns}/joint_command")
        for cam_info in self._cameras:
            cam = cam_info["camera"]
            ns = ros2_bridge.NS
            topics = [f"{ns}/{cam.name}_camera_info", f"{ns}/{cam.name}_tf"]
            if cam_info["rgb"]:
                topics.append(f"{ns}/{cam.name}_rgb")
            if cam_info["depth"]:
                topics.append(f"{ns}/{cam.name}_depth")
            if cam_info["pointcloud"]:
                topics.append(f"{ns}/{cam.name}_pointcloud")
            print(f"  카메라({cam.name}): {', '.join(topics)}")
        if self._obj_manager and self._obj_manager.objects:
            print(f"\n  오브젝트 ({len(self._obj_manager.objects)}개):")
            self._obj_manager.print_positions()
            print("  TF 프레임: " + ", ".join(obj.name for obj in self._obj_manager.objects))
            print(f"  마커:     {ros2_bridge.NS}/object_markers/<name>")
        print("\n" + "=" * 65 + "\n")

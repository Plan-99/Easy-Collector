"""
테이블 위 오브젝트 생성, 랜덤 배치, ROS2 퍼블리쉬.

isaacsim import는 함수/메서드 내부에서 수행.
"""

import numpy as np


class TargetObject:
    """테이블 위의 개별 작업 대상 오브젝트.

    프리미티브(cube/sphere/cylinder) 또는 USD 에셋 파일로 생성 가능.

    Args:
        usd_path: USD 에셋 파일 경로. 지정 시 shape/color 무시.
        scale: USD 에셋의 스케일 (기본 1.0). usd_path 사용 시에만 적용.
        position_range: [[x_max, y_max], [x_min, y_min]]. None이면 테이블 기본 범위.
        orientation_range: [[rx_min, ry_min, rz_min], [rx_max, ry_max, rz_max]] 도 단위.
                           None이면 회전 없음.
    """

    def __init__(self, name: str, stage, size: float = 0.06,
                 color: tuple | None = None, shape: str = "cube",
                 height: float | None = None,
                 usd_path: str | None = None, scale: float = 1.0,
                 position_range: list | None = None,
                 orientation_range: list | None = None,
                 deformable: float | None = None,
                 collision_approximation: str = "convexHull",
                 friction: float = 1.0,
                 mass: float | None = None):
        from pxr import Usd, UsdGeom, UsdPhysics, Gf

        # 프리미티브는 color 기본값 필요, USD는 None 유지
        if color is None and usd_path is None:
            color = (1.0, 0.3, 0.2)

        self.name = name
        self.prim_path = f"/World/{name}"
        self.size = size
        self.color = color
        self._stage = stage
        self.position_range = position_range
        self.orientation_range = orientation_range
        self._scale = scale

        if usd_path is not None:
            # USD 에셋 로드
            prim = stage.DefinePrim(self.prim_path)
            prim.GetReferences().AddReference(usd_path)
            self._xform = UsdGeom.Xformable(prim)
            self._xform.ClearXformOpOrder()
            self.half_height = size / 2.0

            if orientation_range is not None:
                # transform matrix (rotation+scale) + translate
                self._use_transform_op = True
                # 초기값: orientation_range의 중간값
                ori_min, ori_max = orientation_range
                init_ori = tuple((ori_min[i] + ori_max[i]) / 2.0 for i in range(3))
                mat = self._build_transform_matrix(init_ori, scale)
                self._xform.AddTransformOp().Set(mat)
            else:
                self._use_transform_op = False
                self._xform.AddScaleOp().Set(Gf.Vec3d(scale, scale, scale))
            self._xform.AddTranslateOp()

            # USD 오브젝트에 color 지정 시 OmniPBR 머티리얼 생성 후 바인딩
            if color is not None:
                self._apply_color_to_usd(stage, prim, color)

            # USD 참조의 메쉬에 CollisionAPI 적용
            if prim.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(prim)
                mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_collision.CreateApproximationAttr(collision_approximation)
            else:
                for desc in Usd.PrimRange(prim):
                    if desc.IsA(UsdGeom.Mesh):
                        UsdPhysics.CollisionAPI.Apply(desc)
                        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(desc)
                        mesh_collision.CreateApproximationAttr(collision_approximation)
        else:
            # 프리미티브 생성
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

            if orientation_range is not None:
                self._use_transform_op = True
                ori_min, ori_max = orientation_range
                init_ori = tuple((ori_min[i] + ori_max[i]) / 2.0 for i in range(3))
                mat = self._build_transform_matrix(init_ori, 1.0)
                self._xform.AddTransformOp().Set(mat)
            else:
                self._use_transform_op = False

            self._xform.AddTranslateOp()
            if color is not None:
                geom.CreateDisplayColorAttr([color])

        prim = stage.GetPrimAtPath(self.prim_path)

        if deformable is not None:
            # Deformable body (RigidBody와 양립 불가)
            from omni.physx.scripts import deformableUtils
            from pxr import PhysxSchema, UsdShade
            import math

            # 메쉬 prim 찾기
            mesh_prim = prim
            mesh_path = self.prim_path
            if not prim.IsA(UsdGeom.Mesh):
                for desc in Usd.PrimRange(prim):
                    if desc.IsA(UsdGeom.Mesh):
                        mesh_prim = desc
                        mesh_path = str(desc.GetPath())
                        break

            # deformable 0.0=매우 부드러움(100 Pa), 1.0=단단함(100000 Pa)
            young_modulus = 100.0 * math.pow(1000.0, deformable)

            # deformableUtils로 simulation mesh 생성 + API 설정
            deformableUtils.add_physx_deformable_body(
                stage,
                mesh_path,
                collision_simplification=True,
                simulation_hexahedral_resolution=2,
                self_collision=False,
            )

            # Deformable body material 생성 및 바인딩
            deform_mat_path = f"{self.prim_path}/DeformableMaterial"
            deformableUtils.add_deformable_body_material(
                stage,
                deform_mat_path,
                youngs_modulus=young_modulus,
                poissons_ratio=0.45,
                damping_scale=0.1,
                dynamic_friction=1.0,
            )
            from pxr import UsdShade
            mat = UsdShade.Material(stage.GetPrimAtPath(deform_mat_path))
            UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics",
            )
        else:
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.CollisionAPI.Apply(prim)

        # 마찰력 설정 (그리핑 안정성)
        material_path = f"{self.prim_path}/PhysicsMaterial"
        UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(material_path))
        mat_prim = stage.GetPrimAtPath(material_path)
        mat_api = UsdPhysics.MaterialAPI(mat_prim)
        mat_api.CreateStaticFrictionAttr(friction)
        mat_api.CreateDynamicFrictionAttr(friction)
        mat_api.CreateRestitutionAttr(0.0)

        # 질량 설정 (지정 시)
        if mass is not None:
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr(mass)
        from pxr import UsdShade
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            UsdShade.Material(mat_prim),
            UsdShade.Tokens.weakerThanDescendants,
            "physics",
        )

        self._position = np.array([0.0, 0.0, 0.0])

    def _apply_color_to_usd(self, stage, prim, color):
        """USD 오브젝트에 단색 OmniPBR 머티리얼을 생성하여 바인딩.

        원본 텍스처를 무시하고 지정된 color로 덮어씁니다.
        """
        from pxr import UsdShade, Sdf, Gf

        mat_path = f"{self.prim_path}/Looks/ColorMaterial"
        shader_path = f"{mat_path}/Shader"

        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, shader_path)
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        # 모든 하위 메쉬에 바인딩
        from pxr import Usd, UsdGeom
        for desc in Usd.PrimRange(prim):
            if desc.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI.Apply(desc).Bind(mat)

        print(f"[INFO] USD 오브젝트 색상 적용: {self.name} → {color}")

    @staticmethod
    def _euler_to_rotation(orientation):
        """(rx, ry, rz) degrees → 3x3 rotation matrix rows."""
        import math
        rx, ry, rz = [math.radians(a) for a in orientation]
        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)
        return (
            (cy*cz, -cy*sz, sy),
            (sx*sy*cz+cx*sz, -sx*sy*sz+cx*cz, -sx*cy),
            (-cx*sy*cz+sx*sz, cx*sy*sz+sx*cz, cx*cy),
        )

    @staticmethod
    def _build_transform_matrix(orientation, scale, position=(0, 0, 0)):
        """orientation(degrees) + scale + position → Gf.Matrix4d.

        USD는 row-vector convention (p' = p * M)이므로 rotation을 전치하여 저장.
        """
        from pxr import Gf
        rot = TargetObject._euler_to_rotation(orientation)
        r = rot
        s = scale
        # 전치: r[row][col] → r[col][row]
        return Gf.Matrix4d(
            s*r[0][0], s*r[1][0], s*r[2][0], 0,
            s*r[0][1], s*r[1][1], s*r[2][1], 0,
            s*r[0][2], s*r[1][2], s*r[2][2], 0,
            position[0], position[1], position[2], 1,
        )

    def set_position(self, position: np.ndarray):
        from pxr import Gf, UsdGeom
        self._position = position.copy()
        prim = self._stage.GetPrimAtPath(self.prim_path)
        xform = UsdGeom.Xformable(prim)
        for op in xform.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                op.Set(Gf.Vec3d(*position.tolist()))
                return

    def set_orientation(self, orientation: tuple):
        """orientation_range가 설정된 USD 오브젝트의 orientation을 갱신."""
        from pxr import Gf, UsdGeom
        from scipy.spatial.transform import Rotation as R

        if not self._use_transform_op:
            return
        prim = self._stage.GetPrimAtPath(self.prim_path)
        xform = UsdGeom.Xformable(prim)

        # euler → quaternion (scipy: extrinsic XYZ = intrinsic ZYX)
        quat_xyzw = R.from_euler('XYZ', orientation, degrees=True).as_quat()
        quat = Gf.Quatf(float(quat_xyzw[3]), float(quat_xyzw[0]),
                         float(quat_xyzw[1]), float(quat_xyzw[2]))

        for op in xform.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:orient":
                op.Set(quat)
                return
            if op.GetOpName() == "xformOp:transform":
                mat = self._build_transform_matrix(orientation, self._scale)
                op.Set(mat)
                return

    def get_position(self) -> np.ndarray:
        from pxr import UsdGeom
        prim = self._stage.GetPrimAtPath(self.prim_path)
        xform = UsdGeom.Xformable(prim)
        transform = xform.ComputeLocalToWorldTransform(0)
        t = transform.ExtractTranslation()
        return np.array([t[0], t[1], t[2]])


class BgObject:
    """물리 없이 고정 배치되는 배경 오브젝트.

    USD 에셋 또는 프리미티브(cube)로 생성 가능.

    Args:
        usd_path: USD 에셋 파일 경로. None이면 프리미티브 생성.
        position: (x, y, z) 월드 좌표.
        orientation: (roll, pitch, yaw) 도 단위. None이면 회전 없음.
        scale: 스케일 (기본 1.0). USD 에셋 전용.
        box_size: (x, y, z) 직육면체 크기. usd_path=None일 때 사용.
        color: (r, g, b) 프리미티브 색상.
    """

    def __init__(self, name: str, stage, usd_path: str | None = None,
                 position: tuple = (0.0, 0.0, 0.0),
                 orientation: tuple | None = None,
                 scale: float = 1.0,
                 box_size: tuple | None = None,
                 color: tuple = (0.5, 0.5, 0.5)):
        from pxr import UsdGeom, Gf

        self.name = name
        self.prim_path = f"/World/{name}"
        self._stage = stage

        if usd_path is not None:
            prim = stage.DefinePrim(self.prim_path)
            prim.GetReferences().AddReference(usd_path)
        elif box_size is not None:
            # UsdGeom.Cube는 size=1 고정이므로 scale로 각 축 크기 조절
            geom = UsdGeom.Cube.Define(stage, self.prim_path)
            geom.GetSizeAttr().Set(1.0)
            geom.CreateDisplayColorAttr([color])
            scale = 1.0  # box_size가 스케일 역할

        xform = UsdGeom.Xformable(stage.GetPrimAtPath(self.prim_path))
        xform.ClearXformOpOrder()

        if orientation is not None:
            rot = TargetObject._euler_to_rotation(orientation)
            r = rot
            if box_size is not None:
                sx, sy, sz = box_size[0] / 2, box_size[1] / 2, box_size[2] / 2
                mat = Gf.Matrix4d(
                    sx*r[0][0], sx*r[1][0], sx*r[2][0], 0,
                    sy*r[0][1], sy*r[1][1], sy*r[2][1], 0,
                    sz*r[0][2], sz*r[1][2], sz*r[2][2], 0,
                    position[0], position[1], position[2], 1,
                )
            else:
                s = scale
                mat = Gf.Matrix4d(
                    s*r[0][0], s*r[1][0], s*r[2][0], 0,
                    s*r[0][1], s*r[1][1], s*r[2][1], 0,
                    s*r[0][2], s*r[1][2], s*r[2][2], 0,
                    position[0], position[1], position[2], 1,
                )
            xform.AddTransformOp().Set(mat)
        else:
            xform.AddTranslateOp().Set(Gf.Vec3d(*position))
            if box_size is not None:
                xform.AddScaleOp().Set(Gf.Vec3d(box_size[0]/2, box_size[1]/2, box_size[2]/2))
            elif scale != 1.0:
                xform.AddScaleOp().Set(Gf.Vec3d(scale, scale, scale))


class ObjectManager:
    """여러 오브젝트를 생성, 랜덤 배치, ROS2 마커 퍼블리쉬하는 매니저."""

    def __init__(self, stage, table_center: np.ndarray, table_scale: np.ndarray,
                 spawn_range: list | None = None):
        self._stage = stage
        self.objects: list[TargetObject] = []
        self.bg_objects: list[BgObject] = []

        table_surface_z = table_center[2] + table_scale[2] / 2.0

        if spawn_range is not None:
            self._x_range = (spawn_range[0][0], spawn_range[1][0])
            self._y_range = (spawn_range[0][1], spawn_range[1][1])
        else:
            half_x = table_scale[0] / 2.0 * 0.5
            half_y = table_scale[1] / 2.0 * 0.5
            self._x_range = (table_center[0] - half_x, table_center[0] + half_x)
            self._y_range = (table_center[1] - half_y, table_center[1] + half_y)

        self._surface_z = table_surface_z

        self._ros_node = None
        self._marker_pubs = {}
        self._robot_prim_path = None

    def add_object(self, name: str, size: float = 0.06,
                   color: tuple | None = None, shape: str = "cube",
                   height: float | None = None,
                   usd_path: str | None = None, scale: float = 1.0,
                   position_range: list | None = None,
                   orientation_range: list | None = None,
                   deformable: float | None = None,
                   collision_approximation: str = "convexHull",
                   friction: float = 1.0,
                   mass: float | None = None) -> TargetObject:
        obj = TargetObject(name, self._stage, size, color, shape, height,
                           usd_path, scale, position_range, orientation_range,
                           deformable, collision_approximation, friction, mass)
        self.objects.append(obj)
        return obj

    def add_bg_object(self, name: str, usd_path: str | None = None,
                      position: tuple = (0.0, 0.0, 0.0),
                      orientation: tuple | None = None,
                      scale: float = 1.0,
                      box_size: tuple | None = None,
                      color: tuple = (0.5, 0.5, 0.5)) -> BgObject:
        obj = BgObject(name, self._stage, usd_path, position=position,
                       orientation=orientation, scale=scale,
                       box_size=box_size, color=color)
        self.bg_objects.append(obj)
        return obj

    def randomize_all(self):
        """모든 오브젝트를 테이블 위 랜덤 위치에 배치 (겹침 방지)."""
        positions = []
        for obj in self.objects:
            if obj.position_range is not None:
                x_range = (obj.position_range[0][0], obj.position_range[1][0])
                y_range = (obj.position_range[0][1], obj.position_range[1][1])
            else:
                x_range = self._x_range
                y_range = self._y_range
            for _ in range(100):
                x = np.random.uniform(*x_range)
                y = np.random.uniform(*y_range)
                z = self._surface_z + obj.half_height + 0.05
                pos = np.array([x, y, z])

                too_close = False
                for prev_pos, prev_obj in zip(positions, self.objects):
                    min_dist = (obj.size + prev_obj.size) / 2.0 + 0.02
                    if np.linalg.norm(pos[:2] - prev_pos[:2]) < min_dist:
                        too_close = True
                        break
                if not too_close:
                    break

            # orientation 랜덤
            if obj.orientation_range is not None:
                ori_min = obj.orientation_range[0]
                ori_max = obj.orientation_range[1]
                ori = tuple(
                    np.random.uniform(ori_min[i], ori_max[i]) for i in range(3)
                )
                obj.set_orientation(ori)

            obj.set_position(pos)
            positions.append(pos)

    def setup_marker_publisher(self, robot_prim_path: str):
        """오브젝트별 Marker 퍼블리셔 (로봇 base 기준 상대 좌표)."""
        import rclpy
        from visualization_msgs.msg import Marker

        self._robot_prim_path = robot_prim_path

        try:
            rclpy.init()
        except RuntimeError:
            pass

        self._ros_node = rclpy.create_node("object_pose_publisher")
        for obj in self.objects:
            topic = f"/simulation/object_markers/{obj.name}"
            self._marker_pubs[obj.name] = self._ros_node.create_publisher(Marker, topic, 10)

    def publish_markers(self):
        """오브젝트별 Marker(구체)를 로봇 base 기준 상대 좌표로 퍼블리쉬."""
        from pxr import UsdGeom, Gf
        from visualization_msgs.msg import Marker

        if not self._marker_pubs:
            return

        # 로봇 base의 역변환 행렬
        prim = self._stage.GetPrimAtPath(self._robot_prim_path)
        xform = UsdGeom.Xformable(prim)
        inv_base_tf = xform.ComputeLocalToWorldTransform(0).GetInverse()

        for i, obj in enumerate(self.objects):
            pos = obj.get_position()
            rel_pt = inv_base_tf.Transform(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))

            # 오브젝트 orientation (월드 → 로봇 base 기준)
            obj_prim = self._stage.GetPrimAtPath(obj.prim_path)
            obj_xform = UsdGeom.Xformable(obj_prim)
            world_tf = obj_xform.ComputeLocalToWorldTransform(0)
            rel_tf = inv_base_tf * world_tf
            rel_quat = rel_tf.ExtractRotation().GetQuaternion()
            qw = rel_quat.GetReal()
            qi = rel_quat.GetImaginary()

            marker = Marker()
            marker.header.frame_id = "panda_link0"
            marker.ns = obj.name
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = rel_pt[0]
            marker.pose.position.y = rel_pt[1]
            marker.pose.position.z = rel_pt[2]
            marker.pose.orientation.x = qi[0]
            marker.pose.orientation.y = qi[1]
            marker.pose.orientation.z = qi[2]
            marker.pose.orientation.w = qw
            marker.scale.x = obj.size
            marker.scale.y = obj.size
            marker.scale.z = obj.size
            c = obj.color if obj.color is not None else (0.5, 0.5, 0.5)
            marker.color.r = float(c[0])
            marker.color.g = float(c[1])
            marker.color.b = float(c[2])
            marker.color.a = 0.8

            self._marker_pubs[obj.name].publish(marker)

    def get_all_positions(self) -> dict:
        return {obj.name: obj.get_position() for obj in self.objects}

    def destroy(self):
        """ROS2 노드 정리."""
        if self._ros_node:
            self._ros_node.destroy_node()
            self._ros_node = None
        self._marker_pubs.clear()

    def print_positions(self):
        for obj in self.objects:
            pos = obj.get_position()
            print(f"  {obj.name}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

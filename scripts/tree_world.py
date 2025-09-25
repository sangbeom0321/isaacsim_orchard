import math
import omni
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
import omni.isaac.core.utils.numpy.rotations as rot_utils
from pxr import UsdGeom, Gf
from omni.isaac.core.world import World
import yaml
import random
import numpy as np
from ament_index_python.packages import get_package_share_directory

class UnstructuredTreeWorld:
    def __init__(self, stage):
        self.stage = stage
        self.world = World()

        package_name = 'isaac_ml'
        package_share_directory = get_package_share_directory(package_name)
        yaml_path = package_share_directory + "/config/unstructured_orchard.yaml"
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # Ground
        ground_config = config["Ground"]
        self.ground_usdc_path = package_share_directory + ground_config["ground_usdc_path"]
        self.ground_prim_path = ground_config["ground_prim_path"]
        self.ground_position = tuple(ground_config["ground_position"])
        self.ground_scale = tuple(ground_config["ground_scale"])
        
        # Tree
        tree_config = config["Tree"]
        tree_usdc_path1 = package_share_directory + tree_config["tree1_usdc_path"]
        tree_usdc_path2 = package_share_directory + tree_config["tree2_usdc_path"]
        tree_usdc_path3 = package_share_directory + tree_config["tree3_usdc_path"]
        tree_usdc_path4 = package_share_directory + tree_config["tree4_usdc_path"]
        self.tree_usdc_list = [tree_usdc_path1, tree_usdc_path2, tree_usdc_path3, tree_usdc_path4]
        
        self.x_max = float(tree_config["x_max"])
        self.x_min = float(tree_config["x_min"])
        self.y_max = float(tree_config["y_max"])
        self.y_min = float(tree_config["y_min"])
        self.y_step = float(tree_config["y_step"])
        self.x_spacing_range = tuple(map(float, tree_config["x_spacing_range"]))
        self.x_start_range = tuple(map(float, tree_config["x_start_range"]))
        # end는 start와 독립적으로 뽑지 않고, 최소 간격 보장 로직으로 샘플링
        self.x_end_range = [self.x_max - self.x_start_range[1], self.x_max - self.x_start_range[0]]
        
        self.skip_spawn_rate = float(tree_config.get("skip_spawn_rate", 0.0))  # %

        # Layout (전체 회전/중앙 지정)  >>> center_xyz 로 변경 <<<
        layout_cfg = config.get("Layout", {})
        self.layout_center_xyz = tuple(layout_cfg.get("center_xyz", [])) if layout_cfg.get("center_xyz") else None
        self.layout_yaw_deg = float(layout_cfg.get("yaw_deg", 0.0))  # Z축 회전(도)
        self.rotate_trees_with_layout = bool(layout_cfg.get("rotate_trees_with_layout", False))

        # 스케일 설정
        self.scale_range = tuple(map(float, tree_config.get("scale_range", [1.0, 1.0])))
        self.nonuniform_scale_range = tree_config.get("nonuniform_scale_range", None)
        if self.nonuniform_scale_range:
            self.nonuniform_scale_range = [tuple(map(float, r)) for r in self.nonuniform_scale_range]

    # ----------------------------
    # X span 샘플링 (start <= end 보장)
    # ----------------------------
    def _sample_x_span(self):
        start_x = random.uniform(self.x_min + self.x_start_range[0],
                                 self.x_min + self.x_start_range[1])
        end_x   = random.uniform(self.x_max - self.x_start_range[1],
                                 self.x_max - self.x_start_range[0])

        min_span = self.x_spacing_range[0]
        if end_x < start_x + min_span:
            end_x = min(start_x + min_span, self.x_max)
        return start_x, end_x

    # 스케일 샘플러
    def _sample_tree_scale(self):
        if self.nonuniform_scale_range:
            sx = random.uniform(*self.nonuniform_scale_range[0])
            sy = random.uniform(*self.nonuniform_scale_range[1])
            sz = random.uniform(*self.nonuniform_scale_range[2])
            return (float(sx), float(sy), float(sz))
        else:
            s = random.uniform(*self.scale_range)
            return (float(s), float(s), float(s))
        
    def create_ground_from_usdc(self):
        ground_path = self.ground_prim_path
        grass_usdc_path = self.ground_usdc_path
        grass_position = self.ground_position
        
        prim_utils.create_prim(ground_path, prim_type="Xform")
        ground_prim = self.stage.DefinePrim(ground_path, "Xform")
        ground_prim.GetReferences().AddReference(grass_usdc_path)
        ground_prim.GetAttribute("xformOp:translate").Set(grass_position)

    def create_tree_from_usdc(self, position, tree_id, yaw_offset_deg=0.0):
        if self.skip_spawn_rate > 0.0 and random.random() < (self.skip_spawn_rate / 100.0):
            return

        tree_path = f"/World/Trees2/Tree_{tree_id}"
        prim_utils.create_prim(tree_path, prim_type="Xform")
        tree_prim = self.stage.DefinePrim(tree_path, "Xform")

        random_index = random.randint(0, 3)
        tree_usdc = self.tree_usdc_list[int(random_index)]
        tree_prim.GetReferences().AddReference(tree_usdc)
        tree_prim.GetAttribute("xformOp:translate").Set(position)
        
        base_yaw = random.randint(0, 360)
        yaw = base_yaw + (yaw_offset_deg if self.rotate_trees_with_layout else 0.0)
        quaternion_np = rot_utils.euler_angles_to_quats(np.array([0, 0, yaw]), degrees=True)
        quaternion = Gf.Quatd(quaternion_np[0], Gf.Vec3d(quaternion_np[1], quaternion_np[2], quaternion_np[3]))
        tree_prim.GetAttribute("xformOp:orient").Set(quaternion)

        sx, sy, sz = self._sample_tree_scale()
        xform = UsdGeom.Xformable(tree_prim)
        found_scale = False
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3f(float(sx), float(sy), float(sz)))
                found_scale = True
                break
        if not found_scale:
            xform.AddScaleOp().Set(Gf.Vec3f(float(sx), float(sy), float(sz)))
        
        
    def generate_tree_positions(self):
        positions = []
        z_fixed = 0.0

        num_lines = math.floor((self.y_max - self.y_min) / self.y_step) + 1
        
        def generate_line(start_x, start_y, end_x, skip_range=None):
            line_positions = []
            current_x = start_x
            while current_x <= end_x:
                actual_x = current_x + random.uniform(-.3, .3)
                actual_y = start_y + random.uniform(-.5, .5)
                actual_y = max(self.y_min, min(self.y_max, actual_y))

                if skip_range is None or not (skip_range[0] <= actual_x <= skip_range[1]):
                    line_positions.append((actual_x, actual_y, z_fixed))
                current_x += random.uniform(*self.x_spacing_range)
            return line_positions
        
        for i in range(num_lines):
            current_y = self.y_min + i * self.y_step
            if current_y > self.y_max:
                break

            start_x, end_x = self._sample_x_span()
            if random.random() < -1:
                main_line = generate_line(start_x, current_y, end_x)
                if len(main_line) > 4:
                    branch_start_idx = len(main_line) // 2 + random.randint(-2, 2)
                    if 0 <= branch_start_idx < len(main_line):
                        branch_start_x = main_line[branch_start_idx][0]
                        branch_y = current_y + random.choice([-0.4, 0.4]) * self.y_step
                        if self.y_min <= branch_y <= self.y_max:
                            branch_end_x = end_x + random.uniform(-5.0, 5.0)
                            skip_start_x = branch_start_x - random.uniform(1.0, 2.0)
                            skip_end_x = end_x
                            main_line_positions = generate_line(
                                start_x, current_y, end_x, skip_range=(skip_start_x, skip_end_x)
                            )
                            positions.extend(main_line_positions)
                            branch_line = generate_line(branch_start_x, branch_y, branch_end_x)
                            positions.extend(branch_line)
                        else:
                            positions.extend(main_line)
                    else:
                        positions.extend(main_line)
            else:
                positions.extend(generate_line(start_x, current_y, end_x))

        return positions

    # ----------------------------
    # 전체 레이아웃 이동/회전 (center_xyz + yaw)
    # ----------------------------
    def _transform_layout(self, positions):
        """
        positions: [(x,y,z), ...]
        - center_xyz가 주어지면: 현재 centroid(x,y,z)를 target(x,y,z)로 평행이동
        - yaw_deg: target(또는 centroid)을 피벗으로 Z축 회전
        """
        if not positions:
            return positions

        pts = np.array(positions, dtype=float)  # (N,3)
        centroid = pts.mean(axis=0)             # (cx, cy, cz)

        # 1) 평행이동
        if self.layout_center_xyz is not None:
            target = np.array(self.layout_center_xyz, dtype=float)  # (tx, ty, tz)
            delta = target - centroid
            pts = pts + delta
            pivot = target
        else:
            pivot = centroid

        # 2) Z축(yaw) 회전: pivot을 기준
        yaw_rad = math.radians(self.layout_yaw_deg)
        if abs(yaw_rad) > 1e-9:
            c, s = math.cos(yaw_rad), math.sin(yaw_rad)
            Rz = np.array([[c, -s, 0.0],
                           [s,  c, 0.0],
                           [0.0, 0.0, 1.0]], dtype=float)
            pts = (pts - pivot) @ Rz.T + pivot

        return [tuple(p) for p in pts.tolist()]

    def setup_environment(self):
        # self.create_ground_from_usdc()  # 필요시 사용

        tree_positions = self.generate_tree_positions()
        tree_positions = self._transform_layout(tree_positions)

        yaw_offset_deg = float(self.layout_yaw_deg)
        for tree_id, position in enumerate(tree_positions):
            self.create_tree_from_usdc(position, tree_id, yaw_offset_deg=yaw_offset_deg)

def main():
    stage = omni.usd.get_context().get_stage()
    tree_world = UnstructuredTreeWorld(stage)
    tree_world.setup_environment()

if __name__ == "__main__":
    main()

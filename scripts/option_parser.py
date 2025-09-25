# Config & context loader, common constants

import yaml
import numpy as np
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api import World
import isaacsim.core.utils.numpy.rotations as rot_utils
from pxr import Gf
import omni

# ===== constants requested =====
REALSENSE_USD_PATH        = "/Isaac/Sensors/Intel/RealSense/rsd455.usd"
REALSENSE_DEPTH_PRIM_PATH = "/Camera_Pseudo_Depth"
REALSENSE_RGB_PRIM_PATH   = "/Camera_OmniVision_OV9782_Color"

PACKAGE_SHARE_DIR = "/root/ros2_ws/src/isaacsim_orchard"  # absolute

def load_config_and_context():
    cfg_path = PACKAGE_SHARE_DIR + "/config/isaac_sim_settings.yaml"
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)

    cfg["PACKAGE"] = PACKAGE_SHARE_DIR

    usd_ctx = omni.usd.get_context()
    stage   = usd_ctx.get_stage()

    # world
    physics_dt   = 1.0 / cfg["World"]["physics_freq"]
    rendering_dt = 1.0 / cfg["World"]["rendering_freq"]
    world = World(physics_dt=physics_dt, rendering_dt=rendering_dt, stage_units_in_meters=1.0)

    # spawn pose
    pos = cfg["Robot"]["robot_spawn_position"]
    rpy = np.array(cfg["Robot"]["robot_spawn_rotation"])
    q = rot_utils.euler_angles_to_quats(rpy, degrees=True)  # [w,x,y,z]
    rotM = Gf.Matrix3d(Gf.Quatd(q[0], q[1], q[2], q[3]))
    robot_spawn_pose = Gf.Matrix4d().SetRotate(rotM).SetTranslateOnly(Gf.Vec3d(*pos))

    assets_root = get_assets_root_path()

    return cfg, stage, world, assets_root, robot_spawn_pose

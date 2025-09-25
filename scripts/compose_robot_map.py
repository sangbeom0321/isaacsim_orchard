#!/usr/bin/env python3
# Compose a scene with user-specified map USD and robot USD, attach camera and RTX lidar to the robot,
# and publish ROS2 topics using Omnigraph/Replicator writers (Isaac Sim 5.x compatible patterns).

import sys
import os

from isaacsim import SimulationApp

DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")


def load_config(path: str) -> dict:
    try:
        import yaml  # type: ignore
    except Exception as e:
        print(f"[ERROR] PyYAML import failed: {e}. Please install pyyaml.", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(path):
        print(f"[ERROR] config file not found: {path}", file=sys.stderr)
        sys.exit(1)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data


def run_simulation(cfg: dict) -> None:
    # Resolve config values
    
    map_usd = os.path.join(os.path.dirname(__file__),cfg.get("map_usd"))
    robot_usd = os.path.join(os.path.dirname(__file__),cfg.get("robot_usd"))
    robot_prim = cfg.get("robot_prim", "/World/robot")
    robot_xyz = cfg.get("robot_xyz", [0.0, 0.0, 1.0])
    robot_rpy = cfg.get("robot_rpy", [0.0, 0.0, 0.0])
    
    cam_cfg = cfg.get("camera", {})
    cam_link_rel = cam_cfg.get("link_rel", "/camera_link")
    cam_res = cam_cfg.get("resolution", [640, 480])
    
    lidar_cfg = cfg.get("lidar", {})
    lidar_rel = lidar_cfg.get("rel", "/lidar_mount_link/lidar")
    lidar_config = lidar_cfg.get("config", "OS2")
    lidar_variant = lidar_cfg.get("variant", "OS2_REV7_128ch10hz1024res")
    lidar_enable_2d = bool(lidar_cfg.get("enable_2d", True))
    
    app_cfg = cfg.get("app", {})
    show_ui = bool(app_cfg.get("show_ui", False))
    renderer = app_cfg.get("renderer", "HydraStorm")
    sim_cfg = cfg.get("sim", {})

    # Ensure ROS2 env vars for internal rclpy before extensions load
    os.environ.setdefault("ROS_DISTRO", "humble")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ["LD_LIBRARY_PATH"] = f"{os.environ.get('LD_LIBRARY_PATH', '')}:/humble/lib"

    # Build SimulationApp config from sim_cfg + app settings
    sim_app_cfg = {
        "renderer": renderer,
        "headless": bool(sim_cfg.get("headless", not show_ui))
    }
    # Optional window/display overrides if provided
    if "width" in sim_cfg and "height" in sim_cfg:
        sim_app_cfg["width"] = int(sim_cfg["width"])
        sim_app_cfg["height"] = int(sim_cfg["height"])
    if "window_width" in sim_cfg and "window_height" in sim_cfg:
        sim_app_cfg["window_width"] = int(sim_cfg["window_width"])
        sim_app_cfg["window_height"] = int(sim_cfg["window_height"])
    if "hide_ui" in sim_cfg:
        sim_app_cfg["hide_ui"] = bool(sim_cfg["hide_ui"])
    if "display_options" in sim_cfg:
        sim_app_cfg["display_options"] = int(sim_cfg["display_options"])

    app = SimulationApp(sim_app_cfg)

    import carb
    import omni
    import omni.kit.commands
    import omni.graph.core as og  # noqa: F401 (not used after comment-out)
    import omni.replicator.core as rep  # noqa: F401 (not used after comment-out)
    import usdrt.Sdf  # noqa: F401 (not used after comment-out)
    from pxr import Gf, UsdGeom, UsdLux, Sdf
    from isaacsim.core.api import SimulationContext  # noqa: F401 (not used after comment-out)
    from isaacsim.core.utils import stage
    from isaacsim.core.utils.extensions import enable_extension  # noqa: F401 (not used after comment-out)
    from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats

    import warp as wp
    
    # Enable required extensions for ROS2 and Replicator writers
    try:
        enable_extension("isaacsim.ros2.bridge")
    except Exception:
        carb.log_warn("[ext] failed to enable isaacsim.ros2.bridge")
    try:
        enable_extension("omni.syntheticdata")
    except Exception:
        carb.log_warn("[ext] failed to enable omni.syntheticdata")
    try:
        enable_extension("isaacsim.sensors.rtx")
    except Exception:
        carb.log_warn("[ext] failed to enable isaacsim.sensors.rtx")
    try:
        enable_extension("isaacsim.core.nodes")
    except Exception:
        carb.log_warn("[ext] failed to enable isaacsim.core.nodes")
    app.update()

    # Load map USD (existence check)
    if not os.path.exists(map_usd):
        carb.log_error(f"[map] USD not found: {map_usd} {os.path.dirname(os.path.abspath(__file__))}")
        app.close()
        sys.exit(1)
    stage.add_reference_to_stage(map_usd, "/background") # 맵 추가 prim은 /background?
    app.update()

    # Add a distant light similar to start_sim structure
    usd_ctx = omni.usd.get_context() # 컨텍스트
    usd_stage = usd_ctx.get_stage() # 스테이지
    light_path = "/World/Light"
    
    UsdLux.DistantLight.Define(usd_stage, Sdf.Path(light_path)) # 빛 추가

    # Add robot reference (existence check)
    if not os.path.exists(robot_usd):
        carb.log_error(f"[robot] USD not found: {robot_usd}")
    else:
        stage.add_reference_to_stage(robot_usd, robot_prim) # /World/robot
        app.update()
        # Apply spawn pose using TransformPrimCommand with a full transform matrix
        prim = usd_stage.GetPrimAtPath(robot_prim)
        q = euler_angles_to_quats(robot_rpy, degrees=True)
        rot_m = Gf.Matrix3d(Gf.Quatd(q[0], q[1], q[2], q[3]))

        if prim.IsValid():
            # Apply translation only (rotation optional; avoids API differences)
            m4 = Gf.Matrix4d().SetRotate(rot_m).SetTranslateOnly(Gf.Vec3d(*robot_xyz) )
            
            omni.kit.commands.execute(
                "TransformPrimCommand",
                path=robot_prim,
                old_transform_matrix=None,
                new_transform_matrix=m4,
            )
            app.update()
            carb.log_info(f"[robot] placed at XYZ={robot_xyz} RPY(deg)={robot_rpy}")
        else:
            carb.log_warn(f"[robot] prim invalid: {robot_prim}")

    # Resolve full paths
    camera_link_path = robot_prim + cam_link_rel
    lidar_prim_path = robot_prim + lidar_rel
    lidar_2d_prim_path = robot_prim + lidar_rel

    # Camera setup
    camera_prim = UsdGeom.Camera(usd_stage.DefinePrim(camera_link_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_prim)
    xform_api.SetTranslate(Gf.Vec3d(-1, 5, 1))
    xform_api.SetRotate((90, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)# camera의 방향설정하는 부분, 나중에 rviz2로 확인하면서 수정필요
    camera_prim.GetHorizontalApertureAttr().Set(21)
    camera_prim.GetVerticalApertureAttr().Set(16)
    camera_prim.GetProjectionAttr().Set("perspective")
    camera_prim.GetFocalLengthAttr().Set(24)
    camera_prim.GetFocusDistanceAttr().Set(400)
    app.update()

    @wp.kernel
    def image_gaussian_noise_warp(
        data_in: wp.array3d(dtype=wp.uint8), data_out: wp.array3d(dtype=wp.uint8), seed: int, sigma: float = 0.5
    ):
        i, j = wp.tid()
        dim_i = data_out.shape[0]
        dim_j = data_out.shape[1]
        pixel_id = i * dim_i + j
        state_r = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 0))
        state_g = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 1))
        state_b = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 2))

        data_out[i, j, 0] = wp.uint8(float(data_in[i, j, 0]) + (255.0 * sigma * wp.randn(state_r)))
        data_out[i, j, 1] = wp.uint8(float(data_in[i, j, 1]) + (255.0 * sigma * wp.randn(state_g)))
        data_out[i, j, 2] = wp.uint8(float(data_in[i, j, 2]) + (255.0 * sigma * wp.randn(state_b)))
        

    # LiDAR setup (3D for PointCloud2)
    
    _, rtx_lidar = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=lidar_prim_path,
        parent=None,
        config=lidar_config,
        variant=lidar_variant,
        translation=(0.03, 0.0, 0.3),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    app.update()

    # LiDAR setup (2D for LaserScan)
    _, rtx_lidar_2d = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=lidar_2d_prim_path,
        parent=None,
        config="Example_Rotary_2D",
        translation=(0.03, 0.0, 0.3),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    app.update()

    # Replicator render products
    cam_rp = rep.create.render_product(camera_link_path, cam_res, name="CameraRP")
    lidar_rp = rep.create.render_product(rtx_lidar.GetPath(), [1, 1], name="LidarRP")
    lidar2d_rp = rep.create.render_product(rtx_lidar_2d.GetPath(), [1, 1], name="LidarRP2D")

    # ROS2 writers for camera (register custom writer to ensure annotators/time mapping)
    # Register explicit RGB annotator to guarantee 3-channel binding
    try:
        # register new augmented annotator that adds noise to rgba and then outputs to rgb to the ROS publisher can publish
        rep.annotators.register(
            name="rgb_gaussian_noise",
            annotator=rep.annotators.augment_compose(
                source_annotator=rep.annotators.get("rgb", device="cuda"),
                augmentations=[
                    rep.annotators.Augmentation.from_function(
                        image_gaussian_noise_warp, sigma=0.1, seed=1234, data_out_shape=(-1, -1, 3)
                    ),
                ],
            ),
        )
    except Exception as e:
        carb.log_warn(f"[rep] register rgb_gaussian_noise failed: {e}")
    try:
        rep.writers.register_node_writer(
            name="CustomROS2PublishImage",
            node_type_id="isaacsim.ros2.bridge.ROS2PublishImage",
            annotators=[
                "rgb_gaussian_noise",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime",
                    attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"},
                ),
            ],
            category="custom",
        )
        if "CustomROS2PublishImage" not in rep.WriterRegistry._default_writers:
            rep.WriterRegistry._default_writers.append("CustomROS2PublishImage")
    except Exception as e:
        carb.log_warn(f"[ros2] register custom image writer failed: {e}")
    try:
        writer_img = rep.writers.get("CustomROS2PublishImage")
        writer_img.initialize(topicName="/image_raw", frameId="camera")
        writer_img.attach([cam_rp])
    except Exception as e:
        carb.log_warn(f"[ros2] image writer init failed: {e}")
    try:
        rep.writers.register_node_writer(
            name="CustomROS2PublishCameraInfo",
            node_type_id="isaacsim.ros2.bridge.ROS2PublishCameraInfo",
            annotators=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime",
                    attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"},
                ),
            ],
            category="custom",
        )
        if "CustomROS2PublishCameraInfo" not in rep.WriterRegistry._default_writers:
            rep.WriterRegistry._default_writers.append("CustomROS2PublishCameraInfo")
    except Exception as e:
        carb.log_warn(f"[ros2] register custom camera_info writer failed: {e}")
    try:
        writer_info = rep.writers.get("CustomROS2PublishCameraInfo")
        writer_info.initialize(topicName="/camera_info", frameId="camera_optical")
        writer_info.attach([cam_rp])
    except Exception as e:
        carb.log_warn(f"[ros2] camera_info writer init failed: {e}")

    # ROS2 writers for LiDAR
    try:
        writer_pc = rep.writers.get("RtxLidarROS2PublishPointCloud")
        writer_pc.initialize(topicName="/point_cloud", frameId="lidar_link")
        writer_pc.attach([lidar_rp])
    except Exception as e:
        carb.log_warn(f"[ros2] pointcloud writer init failed: {e}")
    try:
        if lidar2d_rp is not None:
            writer_scan = rep.writers.get("RtxLidarROS2PublishLaserScan")
            writer_scan.initialize(topicName="/scan", frameId="lidar_link")
            writer_scan.attach([lidar2d_rp])
    except Exception as e:
        carb.log_warn(f"[ros2] laserscan writer init failed: {e}")

    # Start simulation with SimulationContext to drive sensors/writers
    sim_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
    app.update()
    # Ensure physics and sensors are initialized before play
    sim_context.initialize_physics()
    sim_context.play()

    while app.is_running():
        sim_context.step(render=True)

    sim_context.stop()
    app.close()

if __name__ == "__main__":
    try:
        config = load_config(DEFAULT_CONFIG_PATH)
        run_simulation(config)
    except SystemExit:
        raise
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)



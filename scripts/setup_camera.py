# Camera attach (FixedJoint weld) + ROS2 camera graph

from pxr import Sdf, Usd, UsdGeom, Gf, UsdPhysics
import usdrt.Sdf as usdrt_Sdf
import carb
import omni
import numpy as np
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.nodes.scripts.utils import set_target_prims
from isaacsim.sensors.camera import Camera
from setup_utils import reset_xform_stack, weld_fixed_joint

def set_camera(stage, cfg, assets_root_path, realSense_usd_path, rgb_rel, depth_rel):
    app = omni.kit.app.get_app()
    _ = assets_root_path, realSense_usd_path, rgb_rel, depth_rel

    base_link = cfg["Robot"]["robot_base_link_prim"]
    robot_root = cfg["Robot"].get("robot_container_prim", cfg["Robot"]["robot_prim_path"])

    base_link_prim = stage.GetPrimAtPath(base_link)
    if (not base_link_prim or not base_link_prim.IsValid()) and app:
        for _ in range(60):
            app.update()
            base_link_prim = stage.GetPrimAtPath(base_link)
            if base_link_prim and base_link_prim.IsValid():
                break

    if not base_link_prim or not base_link_prim.IsValid():
        robot_prim = stage.GetPrimAtPath(robot_root)
        sample = []
        if robot_prim and robot_prim.IsValid():
            for idx, prim in enumerate(Usd.PrimRange(robot_prim)):
                if idx > 50:
                    break
                sample.append(prim.GetPath().pathString)
        carb.log_error(
            f"[CAM] Base link prim '{base_link}' not found under '{robot_root}'."
            f" Sample children: {sample}"
        )
        raise RuntimeError(f"[CAM] Base link prim '{base_link}' not found")

    mount_rel = cfg["Camera"]["camera_prim_path"]
    mount_rel = mount_rel.lstrip("/")
    if mount_rel.startswith("base_link"):
        mount_rel = mount_rel[len("base_link") :].lstrip("/")
    mount_path = base_link + ("/" + mount_rel if mount_rel else "")
    mount_prim = stage.GetPrimAtPath(mount_path)
    if not mount_prim or not mount_prim.IsValid():
        mount_prim = stage.DefinePrim(mount_path, "Xform")

    camera_path = mount_path + "/camera_sensor"
    camera_prim = stage.GetPrimAtPath(camera_path)

    if camera_prim and camera_prim.IsValid():
        camera = Camera(prim_path=camera_path)
    else:
        camera = Camera(
            prim_path=camera_path,
            position=np.array([0.0, 0.0, 0.0]),
            frequency=cfg["Camera"].get("camera_freq", 30),
            resolution=(640, 480),
            orientation=rot_utils.euler_angles_to_quats(np.zeros(3), degrees=True),
        )

    camera.initialize()

    translation = np.array(cfg["Camera"].get("camera_translation", [0.0, 0.0, 0.0]), dtype=float)
    orientation_rpy = np.array(cfg["Camera"].get("camera_orientation", [0.0, 0.0, 0.0]), dtype=float)
    orientation_quat = rot_utils.euler_angles_to_quats(orientation_rpy, degrees=True)
    translation_f = tuple(float(v) for v in translation)
    orientation_f = tuple(float(v) for v in orientation_quat)

    reset_xform_stack(stage, camera_path)
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(camera_path))

    # ensure camera prim has rigid body so it can be welded
    camera_prim = stage.GetPrimAtPath(camera_path)
    camera_rb = UsdPhysics.RigidBodyAPI.Get(stage, camera_prim.GetPath())
    if not camera_rb:
        camera_rb = UsdPhysics.RigidBodyAPI.Apply(camera_prim)
        camera_rb.CreateKinematicEnabledAttr(True)
    mass_api = UsdPhysics.MassAPI.Get(stage, camera_prim.GetPath()) or UsdPhysics.MassAPI.Apply(camera_prim)
    if not mass_api.GetMassAttr():
        mass_api.CreateMassAttr(1.0)

    # weld joint between mount link and camera body
    parent_body = mount_path
    parent_rb = UsdPhysics.RigidBodyAPI.Get(stage, Sdf.Path(parent_body))
    if not parent_rb:
        parent_body = base_link
    joint_name = camera_path.strip("/").replace("/", "_") + "_weld"
    joint_path = f"/World/Joints/{joint_name}"
    joint_prim = weld_fixed_joint(stage, parent_body, camera_path, joint_path)

    if joint_prim:
        joint_prim.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint_prim.CreateAttribute("physics:localRot0", Sdf.ValueTypeNames.Quatf).Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint_prim.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*translation_f))
        joint_prim.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf).Set(
            Gf.Quatf(orientation_f[0], orientation_f[1], orientation_f[2], orientation_f[3])
        )

    cfg.setdefault("Camera", {})["camera_sensor_prim"] = camera_path

    print(f"[CAM] Camera sensor created at {camera_path}")
    return camera_path, camera_path, camera


def make_ros2_camera(stage, cfg, color_cam_prim: str, depth_cam_prim: str):
    import omni.graph.core as og
    gp = cfg["Robot"]["graph_prim_path"]

    node_specs = [
        ("CreateRenderRgb",   "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("CreateRenderDepth", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("Context",           "isaacsim.ros2.bridge.ROS2Context"),
        ("Tick",              "omni.graph.action.OnPlaybackTick"),
        ("CamRGB",            "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("CamInfo",           "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ("CamDepth",          "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ]

    nodes_to_create = []
    for name, node_type in node_specs:
        node_path = f"{gp}/{name}"
        prim = stage.GetPrimAtPath(node_path)
        if not prim or not prim.IsValid():
            nodes_to_create.append((name, node_type))

    def attr(node_name, attr_name):
        return f"{gp}/{node_name}.{attr_name}"

    edit_kwargs = {
        og.Controller.Keys.CONNECT: [
            (attr("Tick", "outputs:tick"), attr("CreateRenderRgb", "inputs:execIn")),
            (attr("Tick", "outputs:tick"), attr("CreateRenderDepth", "inputs:execIn")),
            (attr("Context", "outputs:context"), attr("CamRGB", "inputs:context")),
            (attr("Context", "outputs:context"), attr("CamInfo", "inputs:context")),
            (attr("Context", "outputs:context"), attr("CamDepth", "inputs:context")),
            (attr("CreateRenderRgb", "outputs:execOut"), attr("CamRGB", "inputs:execIn")),
            (attr("CreateRenderRgb", "outputs:renderProductPath"), attr("CamRGB", "inputs:renderProductPath")),
            (attr("CreateRenderRgb", "outputs:renderProductPath"), attr("CamInfo", "inputs:renderProductPath")),
            (attr("CreateRenderDepth", "outputs:execOut"), attr("CamDepth", "inputs:execIn")),
            (attr("CreateRenderDepth", "outputs:renderProductPath"), attr("CamDepth", "inputs:renderProductPath")),
        ],
        og.Controller.Keys.SET_VALUES: [
            (attr("CreateRenderRgb", "inputs:width"), 640),
            (attr("CreateRenderRgb", "inputs:height"), 480),
            (attr("CreateRenderDepth", "inputs:width"), 640),
            (attr("CreateRenderDepth", "inputs:height"), 480),

            (attr("CamRGB", "inputs:type"), "rgb"),
            (attr("CamRGB", "inputs:frameId"), cfg["Camera"]["camera_frame_id"]),
            (attr("CamRGB", "inputs:topicName"), cfg["Camera"]["camera_topic_name"] + "/image_raw"),
            (attr("CamRGB", "inputs:frameSkipCount"), 0),

            (attr("CamInfo", "inputs:frameId"), cfg["Camera"]["camera_frame_id"]),
            (attr("CamInfo", "inputs:topicName"), cfg["Camera"]["camera_topic_name"] + "/camera_info"),

            (attr("CamDepth", "inputs:type"), "depth"),
            (attr("CamDepth", "inputs:frameId"), cfg["Camera"]["camera_frame_id"]),
            (attr("CamDepth", "inputs:topicName"), cfg["Camera"]["camera_topic_name"] + "/depth_raw"),
            (attr("CamDepth", "inputs:frameSkipCount"), 0),
        ],
    }

    if nodes_to_create:
        edit_kwargs[og.Controller.Keys.CREATE_NODES] = nodes_to_create

    try:
        og.Controller.edit(
            gp,
            edit_kwargs,
        )
    except og.OmniGraphError as exc:
        if nodes_to_create and "already exists" in str(exc):
            edit_kwargs.pop(og.Controller.Keys.CREATE_NODES, None)
            og.Controller.edit(gp, edit_kwargs)
        else:
            raise

    if color_cam_prim:
        og.Controller.attribute(f"{gp}/CreateRenderRgb.inputs:cameraPrim").set(
            [usdrt_Sdf.Path(color_cam_prim)]
        )
    if depth_cam_prim:
        og.Controller.attribute(f"{gp}/CreateRenderDepth.inputs:cameraPrim").set(
            [usdrt_Sdf.Path(depth_cam_prim)]
        )

    print(f"[ROS2_CAMERA] bound cameraPrim: {color_cam_prim} -> RGB, {depth_cam_prim} -> DEPTH")

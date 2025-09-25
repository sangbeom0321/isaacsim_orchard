# RTX LiDAR attach + ROS2 lidar graphs (PC and Scan)

import omni
def set_lidar(stage, cfg):
    # 5.0: config + variant required
    base_link = cfg["Robot"]["robot_base_link_prim"]
    app = omni.kit.app.get_app()

    target_path = base_link + cfg["LiDAR"]["lidar_prim_path"]

    _, _lidar = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=target_path,
        parent=cfg["LiDAR"]["lidar_parent"] if cfg["LiDAR"]["lidar_parent"] else None,
        config="OS2",
        variant="OS2_REV7_128ch10hz1024res",
        translation=tuple(cfg["LiDAR"]["lidar_translation"]),
        orientation=_to_quat(cfg["LiDAR"]["lidar_orientation"]),
    )

    lidar_path = target_path
    try:
        lidar_path = _lidar.GetPath().pathString
    except AttributeError:
        if isinstance(_lidar, str):
            lidar_path = _lidar

    if app:
        for _ in range(5):
            app.update()

    cfg.setdefault("LiDAR", {})["lidar_sensor_prim"] = lidar_path
    print(f"[LiDAR] sensor created at {lidar_path}")
    return lidar_path

def _to_quat(rpy_deg):
    from pxr import Gf
    import numpy as np
    import isaacsim.core.utils.numpy.rotations as rot_utils
    q = rot_utils.euler_angles_to_quats(np.array(rpy_deg), degrees=True)
    gq = Gf.Quatd(q[0]); gq.SetImaginary(q[1], q[2], q[3]); return gq

def make_ros2_lidar(stage, cfg):
    import omni.graph.core as og
    gp = cfg["Robot"]["graph_prim_path"]
    lidar_cfg = cfg.get("LiDAR", {})
    enable_scan = lidar_cfg.get("enable_flat_scan", False)

    node_specs = [
        ("CreateRenderLidar", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("RunOneFrame",       "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
        ("LidarPcPub",        "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ("Context",           "isaacsim.ros2.bridge.ROS2Context"),
        ("Tick",              "omni.graph.action.OnPlaybackTick"),
    ]

    if enable_scan:
        node_specs.append(("LidarScanPub", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))

    nodes_to_create = []
    for name, node_type in node_specs:
        node_path = f"{gp}/{name}"
        prim = stage.GetPrimAtPath(node_path)
        if not prim or not prim.IsValid():
            nodes_to_create.append((name, node_type))

    def attr(node_name, port):
        return f"{gp}/{node_name}.{port}"

    edit_kwargs = {
        og.Controller.Keys.CONNECT: [
            (attr("Tick", "outputs:tick"), attr("RunOneFrame", "inputs:execIn")),
            (attr("RunOneFrame", "outputs:step"), attr("CreateRenderLidar", "inputs:execIn")),
            (attr("Context", "outputs:context"), attr("LidarPcPub", "inputs:context")),
            (attr("CreateRenderLidar", "outputs:execOut"), attr("LidarPcPub", "inputs:execIn")),
            (attr("CreateRenderLidar", "outputs:renderProductPath"), attr("LidarPcPub", "inputs:renderProductPath")),
        ],
        og.Controller.Keys.SET_VALUES: [
            (attr("LidarPcPub", "inputs:topicName"), cfg["LiDAR"]["lidar_pc_topic"]),
            (attr("LidarPcPub", "inputs:frameId"), cfg["LiDAR"]["lidar_frame_id"]),
            (attr("LidarPcPub", "inputs:fullScan"), True),
            (attr("LidarPcPub", "inputs:nodeNamespace"), cfg["LiDAR"]["lidar_pc_node_name"]),
            (attr("LidarPcPub", "inputs:type"), "point_cloud"),
            (attr("LidarPcPub", "inputs:frameSkipCount"), cfg["LiDAR"]["lidar_frameskip_cnt"]),
        ],
    }

    if enable_scan:
        edit_kwargs[og.Controller.Keys.CONNECT].extend(
            [
                (attr("Context", "outputs:context"), attr("LidarScanPub", "inputs:context")),
                (attr("CreateRenderLidar", "outputs:execOut"), attr("LidarScanPub", "inputs:execIn")),
                (attr("CreateRenderLidar", "outputs:renderProductPath"), attr("LidarScanPub", "inputs:renderProductPath")),
            ]
        )
        edit_kwargs[og.Controller.Keys.SET_VALUES].extend(
            [
                (attr("LidarScanPub", "inputs:topicName"), cfg["LiDAR"]["lidar_scan_topic"]),
                (attr("LidarScanPub", "inputs:frameId"), cfg["LiDAR"]["lidar_frame_id"]),
                (attr("LidarScanPub", "inputs:nodeNamespace"), cfg["LiDAR"]["lidar_scan_node_name"]),
                (attr("LidarScanPub", "inputs:type"), "laser_scan"),
                (attr("LidarScanPub", "inputs:frameSkipCount"), cfg["LiDAR"]["lidar_frameskip_cnt"]),
            ]
        )

    if nodes_to_create:
        edit_kwargs[og.Controller.Keys.CREATE_NODES] = nodes_to_create

    try:
        og.Controller.edit(gp, edit_kwargs)
    except og.OmniGraphError as exc:
        if nodes_to_create and "already exists" in str(exc):
            edit_kwargs.pop(og.Controller.Keys.CREATE_NODES, None)
            og.Controller.edit(gp, edit_kwargs)
        else:
            raise

    lidar_prim = cfg["LiDAR"].get("lidar_sensor_prim") or (
        cfg["Robot"]["robot_base_link_prim"] + cfg["LiDAR"]["lidar_prim_path"]
    )

    og.Controller.attribute(f"{gp}/CreateRenderLidar.inputs:cameraPrim").set([lidar_prim])
    print(f"[LiDAR] render product attached to {lidar_prim}")

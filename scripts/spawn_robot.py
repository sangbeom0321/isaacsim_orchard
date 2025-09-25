# Robot spawn + ROS2 base graphs (clock / TF / joints)

import omni
import carb
from pxr import Usd, UsdPhysics, PhysxSchema, Sdf
from isaacsim.core.api.robots import Robot
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.graph.core as og
import usdrt.Sdf as usdrt_Sdf

def spawn_robot(stage, world, cfg, robot_spawn_pose):
    if not cfg["Robot"]["is_valid"]:
        return
    robot_prim_path = cfg["Robot"]["robot_prim_path"]

    usd_asset_path = cfg["Robot"]["usd_path"].replace("{PACKAGE}", cfg.get("PACKAGE", ""))

    add_reference_to_stage(
        usd_path=usd_asset_path,
        prim_path=robot_prim_path,
    )

    prim = stage.DefinePrim(robot_prim_path, "Xform")
    prim.GetReferences().AddReference(usd_asset_path)
    omni.kit.commands.execute(
        "TransformPrimCommand",
        path=prim.GetPath(),
        old_transform_matrix=None,
        new_transform_matrix=robot_spawn_pose,
    )

    app = omni.kit.app.get_app()
    if app:
        for _ in range(60):
            app.update()

    robot_container = stage.GetPrimAtPath(robot_prim_path)
    articulation_prim = None

    base_link_path = cfg["Robot"]["robot_base_link_prim"]
    base_link_name = base_link_path.rstrip("/").split("/")[-1]
    base_link_prim = stage.GetPrimAtPath(base_link_path)

    if not robot_container or not robot_container.IsValid():
        carb.log_error(f"[ROBOT] Container prim '{robot_prim_path}' is invalid after reference load")
        return

    for prim_it in Usd.PrimRange(robot_container):
        name = prim_it.GetName()
        if not base_link_prim or not base_link_prim.IsValid():
            if name == base_link_name:
                base_link_prim = prim_it
        if not articulation_prim:
            art_api = UsdPhysics.ArticulationRootAPI.Get(stage, prim_it.GetPath())
            if art_api:
                is_valid_fn = getattr(art_api, "IsValid", None)
                is_valid = is_valid_fn() if callable(is_valid_fn) else True
                if is_valid:
                    articulation_prim = prim_it
        if base_link_prim and articulation_prim:
            break

    if base_link_prim and base_link_prim.IsValid():
        base_link_path = str(base_link_prim.GetPath())
        cfg["Robot"]["robot_base_link_prim"] = base_link_path
    else:
        carb.log_warn(
            f"[ROBOT] Cannot locate base link prim '{cfg['Robot']['robot_base_link_prim']}' under '{robot_prim_path}'"
        )
        return

    if articulation_prim and articulation_prim.IsValid():
        articulation_path = str(articulation_prim.GetPath())
    else:
        articulation_path = base_link_path
        carb.log_warn(
            f"[ROBOT] Articulation root not found; defaulting to base link '{articulation_path}'"
        )

    cfg["Robot"]["robot_container_prim"] = robot_prim_path
    cfg["Robot"]["robot_articulation_prim"] = articulation_path

    robot = Robot(articulation_path, name=cfg["Robot"]["robot_name"])
    world.scene.add(robot)

    mass_api = UsdPhysics.MassAPI.Apply(base_link_prim)
    mass_attr = mass_api.GetMassAttr()

    if mass_attr:
        mass_attr.Set(30)
    else:
        mass_api.CreateMassAttr(30)

    _configure_wheel_collisions(stage, base_link_path)

    _configure_collision_offsets(stage, robot_prim_path)

    return {
        "container_prim": robot_prim_path,
        "base_link_prim": base_link_path,
        "articulation_prim": articulation_path,
    }


def _configure_wheel_collisions(stage, base_link_path: str):
    wheel_names = (
        "fl_wheel_link",
        "fr_wheel_link",
        "rl_wheel_link",
        "rr_wheel_link",
    )

    for name in wheel_names:
        link_path = Sdf.Path(base_link_path).GetParentPath().AppendChild(name)
        link_prim = stage.GetPrimAtPath(str(link_path))
        if not link_prim or not link_prim.IsValid():
            continue

        for prim in Usd.PrimRange(link_prim):
            if prim.GetTypeName() in {"Mesh", "Collision"} or "collision" in prim.GetName().lower():
                mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_api.CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)


def _configure_collision_offsets(stage, root_path: str, rest_offset: float = 0.002, contact_offset: float = 0.02):
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return

    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue

        coll_api = UsdPhysics.CollisionAPI(prim)

        rest_attr = coll_api.GetRestOffsetAttr()
        if not rest_attr:
            rest_attr = coll_api.CreateRestOffsetAttr()
        rest_attr.Set(rest_offset)

        contact_attr = coll_api.GetContactOffsetAttr()
        if not contact_attr:
            contact_attr = coll_api.CreateContactOffsetAttr()
        contact_attr.Set(contact_offset)


def make_ros2_bridge(stage, cfg):
    gp = cfg["Robot"]["graph_prim_path"]
    og.Controller.edit(
        {"graph_path": gp, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )


def make_ros2_tf_joint(stage, cfg):
    gp = cfg["Robot"]["graph_prim_path"]
    og.Controller.edit(
        gp,
        {
            og.Controller.Keys.CREATE_NODES: [
                ("PublishJointState",   "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationCtrl",    "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishTF",           "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                (gp + "/OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                (gp + "/OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                (gp + "/SubscribeJointState.outputs:jointNames", "ArticulationCtrl.inputs:jointNames"),
                (gp + "/OnPlaybackTick.outputs:tick", "ArticulationCtrl.inputs:execIn"),
                (gp + "/Context.outputs:context", "PublishJointState.inputs:context"),
                (gp + "/Context.outputs:context", "SubscribeJointState.inputs:context"),
                (gp + "/ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationCtrl.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationCtrl.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand",   "ArticulationCtrl.inputs:effortCommand"),
                (gp + "/OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                (gp + "/ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationCtrl.inputs:robotPath", cfg["Robot"]["robot_base_link_prim"]),
                ("PublishJointState.inputs:topicName", cfg["Robot"]["joint_state_topic_name"]),
                ("SubscribeJointState.inputs:topicName", cfg["Robot"]["joint_command_topic_name"]),
                ("PublishJointState.inputs:targetPrim", [usdrt_Sdf.Path(cfg["Robot"]["robot_base_link_prim"])]),
                ("PublishTF.inputs:topicName", cfg["Robot"]["tf_topic"]),
            ],
        },
    )

    from isaacsim.core.nodes.scripts.utils import set_target_prims
    set_target_prims(
        primPath=gp + "/PublishTF",
        inputName="inputs:targetPrims",
        targetPrimPaths=[cfg["Robot"]["robot_base_link_prim"]],
    )
    set_target_prims(
        primPath=gp + "/PublishTF",
        inputName="inputs:parentPrim",
        targetPrimPaths=[cfg["Robot"]["robot_base_link_prim"]],
    )

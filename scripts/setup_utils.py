# Shared helpers: xform reset, body find, weld joint, IMU, step callbacks (imu/gps/odom publishers)

from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
import omni
import numpy as np
from isaacsim.sensors.physics import IMUSensor

def reset_xform_stack(stage, prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    x = UsdGeom.Xformable(prim)
    ops = list(x.GetOrderedXformOps())
    if hasattr(x, "RemoveXformOp"):
        for op in ops:
            x.RemoveXformOp(op)
    else:
        x.SetXformOpOrder([], resetXformStack=True)
    op = x.AddTransformOp(); op.Set(Gf.Matrix4d(1.0))
    x.SetXformOpOrder([op], resetXformStack=True)

def find_first_rigidbody(stage, root_path: str) -> str | None:
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for p in Usd.PrimRange(root):
        rb = UsdPhysics.RigidBodyAPI.Get(stage, p.GetPath())
        if rb and rb.IsValid():
            return p.GetPath().pathString
    return None

def weld_fixed_joint(stage, parent_body: str, child_body: str, joint_path: str):
    jp = Sdf.Path(joint_path).GetParentPath()
    if not stage.GetPrimAtPath(jp).IsValid():
        stage.DefinePrim(jp, "Xform")
    joint = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(joint_path))
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_body)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_body)])
    return joint.GetPrim()

def create_imu_sensor(cfg) -> IMUSensor:
    base_link = cfg["Robot"]["robot_base_link_prim"]
    imu_suffix = cfg["IMU"]["imu_prim_path"]
    if imu_suffix.startswith("/base_link"):
        imu_suffix = imu_suffix[len("/base_link") :]
    prim_path = base_link + imu_suffix

    return IMUSensor(
        prim_path=prim_path,
        name=cfg["IMU"]["imu_name"],
        frequency=cfg["IMU"]["imu_frequency"],
        translation=cfg["IMU"]["imu_translation"],
        orientation=cfg["IMU"]["imu_orientation"],
        linear_acceleration_filter_size=cfg["IMU"]["imu_linear_acceleration_filter_size"],
        angular_velocity_filter_size=cfg["IMU"]["imu_angular_velocity_filter_size"],
        orientation_filter_size=cfg["IMU"]["imu_orientation_filter_size"],
    )

def step_pub_callbacks(stage, world, cfg, imu: IMUSensor, imu_pub, pose_pub, heading_pub, gps_pub):
    """Attach a physics callback that publishes IMU / odom_gt / heading / GPS."""
    from rclpy.time import Time
    from sensor_msgs.msg import Imu, NavSatFix
    from std_msgs.msg import Float64, Float64MultiArray

    robot_base = cfg["Robot"]["robot_base_link_prim"]
    imu_interval = 1
    gps_interval = cfg["GPS"]["gps_frameskip_cnt"]

    state = {"step": 0, "pos": Gf.Vec3d(0,0,0), "rot": Gf.Rotation(),
             "pose_msg": Float64MultiArray(), "heading_msg": Float64(), "gps_msg": NavSatFix()}

    state["gps_msg"].header.frame_id = cfg["GPS"]["gps_frame_id"]
    state["gps_msg"].position_covariance_type = 0
    state["gps_msg"].position_covariance = [0.0]*9
    state["gps_msg"].status.status = 0
    state["gps_msg"].status.service = 0

    a = 6378137.0; b = 6356752.3; ab = a*b
    lat0 = np.radians(cfg["GPS"]["latitude_reference"])
    lon0 = np.radians(cfg["GPS"]["longitude_reference"])
    cos_lat, sin_lat = np.cos(lat0), np.sin(lat0)
    denom = (a*cos_lat)**2 + (b*sin_lat)**2
    rM = ab**2 / denom / np.sqrt(denom)
    rN = a**2 / np.sqrt(denom)

    def on_step(dt):
        state["step"] += 1

        # robot pose
        T = UsdGeom.Xformable(stage.GetPrimAtPath(robot_base)).GetLocalTransformation(0)
        state["pos"] = T.ExtractTranslation()
        state["rot"] = T.ExtractRotation()

        # imu
        if state["step"] % imu_interval == 0:
            data = imu.get_current_frame()
            q = data["orientation"].astype(float)  # w,x,y,z
            lin = data["lin_acc"].astype(float)
            ang = data["ang_vel"].astype(float)
            t = data["time"]; sec = int(t); nsec = int((t-sec)*1e9)
            msg = Imu()
            msg.header.frame_id = cfg["IMU"]["imu_frame_id"]
            msg.header.stamp = Time(seconds=sec, nanoseconds=nsec).to_msg()
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = q
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = lin
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = ang
            imu_pub.publish(msg)

            # odom_gt / heading
            euler = state["rot"].Decompose(Gf.Vec3d(1,0,0), Gf.Vec3d(0,1,0), Gf.Vec3d(0,0,1))
            heading_rad = np.radians(euler[2])
            h = Float64(); h.data = heading_rad
            heading_pub.publish(h)

            p = Float64MultiArray(); p.data = [float(state["pos"][0]), float(state["pos"][1])]
            pose_pub.publish(p)

        # gps
        if state["step"] % gps_interval == 0:
            east, north = state["pos"][0], state["pos"][1]
            dlat = north / rM; dlon = east / rN
            msg = NavSatFix()
            msg.header.frame_id = cfg["GPS"]["gps_frame_id"]
            msg.latitude  = cfg["GPS"]["latitude_reference"]  + np.degrees(dlat)
            msg.longitude = cfg["GPS"]["longitude_reference"] + np.degrees(dlon)
            msg.altitude  = cfg["GPS"]["altitude_reference"]  + float(state["pos"][2])
            msg.position_covariance_type = 0
            msg.position_covariance = [0.0]*9
            gps_pub.publish(msg)

    world.add_physics_callback("ros_pub_cb", on_step)

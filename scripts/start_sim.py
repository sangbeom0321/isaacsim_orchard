#!/isaac-sim/kit/python/bin/python3
# Orchestrator: imports split modules and runs the app

import carb
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Isaac Sim 5.0 extensions
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.core.nodes")
enable_extension("isaacsim.sensors.rtx")
enable_extension("isaacsim.sensors.camera")
enable_extension("isaacsim.sensors.physics")

import omni
omni.timeline.get_timeline_interface().play()

# --------------- our modules ---------------
from option_parser import load_config_and_context, REALSENSE_USD_PATH, REALSENSE_DEPTH_PRIM_PATH, REALSENSE_RGB_PRIM_PATH
from spawn_entity import set_world, setup_sunlight, setup_dome, setup_tonemapper, setup_friction
from spawn_robot import spawn_robot, make_ros2_bridge, make_ros2_tf_joint
from setup_camera import set_camera, make_ros2_camera
from setup_lidar import set_lidar, make_ros2_lidar
from setup_utils import create_imu_sensor, step_pub_callbacks

# ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, Float64MultiArray


class StartSim(Node):
    def __init__(self) -> None:
        super().__init__("isaac_sim")

        # Parse YAML, stage, world, constants
        (self.cfg, self.stage, self.world,
         self.assets_root_path, self.robot_spawn_pose) = load_config_and_context()

        # Short-hands
        self.robot_is_valid       = self.cfg["Robot"]["is_valid"]
        self.robot_prim_path      = self.cfg["Robot"]["robot_prim_path"]
        self.robot_base_link_prim = self.cfg["Robot"]["robot_base_link_prim"]
        self.graph_prim_path      = self.cfg["Robot"]["graph_prim_path"]

        # Camera topics / ids
        self.camera_frame_id   = self.cfg["Camera"]["camera_frame_id"]
        self.camera_topic_name = self.cfg["Camera"]["camera_topic_name"]

        # LiDAR topics / ids
        self.lidar_frame_id      = self.cfg["LiDAR"]["lidar_frame_id"]
        self.lidar_pc_topic      = self.cfg["LiDAR"]["lidar_pc_topic"]
        self.lidar_scan_topic    = self.cfg["LiDAR"]["lidar_scan_topic"]
        self.lidar_frameskip_cnt = self.cfg["LiDAR"]["lidar_frameskip_cnt"]
        self.lidar_pc_node_name  = self.cfg["LiDAR"]["lidar_pc_node_name"]
        self.lidar_scan_node_name= self.cfg["LiDAR"]["lidar_scan_node_name"]

        # IMU / GPS pubs
        self.imu_pub   = self.create_publisher(Imu, self.cfg["IMU"]["imu_topic_name"], 10)
        self.heading_pub = self.create_publisher(Float64, "/heading", 10)
        self.pose_pub    = self.create_publisher(Float64MultiArray, "/odom_gt", 10)
        self.gps_pub     = self.create_publisher(NavSatFix, self.cfg["GPS"]["gps_topic_name"], 10)

        # placeholders to be filled by set_camera()
        self._rs_color_cam = None
        self._rs_depth_cam = None

    # Thin wrappers calling split modules
    def set_world(self):
        set_world(self.stage, self.world, self.cfg)

    def spawn_robot(self):
        resolved = spawn_robot(self.stage, self.world, self.cfg, self.robot_spawn_pose)
        if resolved:
            container = resolved.get("container_prim")
            articulation = resolved.get("articulation_prim")
            base_link = resolved.get("base_link_prim")

            if container:
                self.cfg["Robot"]["robot_container_prim"] = container
                self.robot_prim_path = container

            if articulation:
                self.cfg["Robot"]["robot_articulation_prim"] = articulation

            if base_link:
                self.cfg["Robot"]["robot_base_link_prim"] = base_link
                self.robot_base_link_prim = base_link

    def setup_sensors(self):
        # Camera (B: FixedJoint weld)
        self._rs_color_cam, self._rs_depth_cam, self._camera_sensor = set_camera(
            stage=self.stage,
            cfg=self.cfg,
            assets_root_path=self.assets_root_path,
            realSense_usd_path=REALSENSE_USD_PATH,
            rgb_rel=REALSENSE_RGB_PRIM_PATH,
            depth_rel=REALSENSE_DEPTH_PRIM_PATH
        )
        # LiDAR, IMU
        self._lidar_prim = set_lidar(self.stage, self.cfg)
        self.imu = create_imu_sensor(self.cfg)

    def setup_ros_graphs(self):
        app = omni.kit.app.get_app()
        if app:
            for _ in range(5):
                app.update()
        make_ros2_bridge(self.stage, self.cfg)
        if self._rs_color_cam and self._rs_depth_cam:
            make_ros2_camera(self.stage, self.cfg, self._rs_color_cam, self._rs_depth_cam)
        make_ros2_lidar(self.stage, self.cfg)
        make_ros2_tf_joint(self.stage, self.cfg)

    def attach_callbacks(self):
        step_pub_callbacks(self.stage, self.world, self.cfg,
                           self.imu, self.imu_pub, self.pose_pub, self.heading_pub, self.gps_pub)


def main():
    rclpy.init()
    sim = StartSim()

    sim.set_world()
    sim.spawn_robot()

    # scene polish
    setup_sunlight(sim.stage)
    setup_dome(sim.stage)
    setup_tonemapper(exposure_stops=1.5)
    setup_friction(sim.stage, sim.cfg)

    # sensors & ros
    sim.setup_sensors()
    sim.world.reset()
    sim.setup_ros_graphs()
    sim.attach_callbacks()
    sim.world.reset()

    while simulation_app.is_running():
        sim.world.step(render=True)

    simulation_app.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

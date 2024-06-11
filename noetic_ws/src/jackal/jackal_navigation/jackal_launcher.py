# import argparse
# import numpy as np
# import math
# import carb
# import rospy
# from sensor_msgs.msg import LaserScan
# import tf
# from collections import deque
# from omni.isaac.kit import SimulationApp

# class JackalEnv:
#     metadata = {"render.modes": ["human"]}

#     def __init__(
#         self,
#         args,
#         skip_frame=1,
#         physics_dt=1.0 / 60.0,
#         rendering_dt=1.0 / 60.0,
#         seed=0,
#         headless=True,
#     ) -> None:
#         # This sample enables a livestream server to connect to when running headless
#         CONFIG = {
#             "width": 1280,
#             "height": 720,
#             "window_width": 1920,
#             "window_height": 1080,
#             "headless": headless,
#             "renderer": "RayTracedLighting",
#             "display_options": 3286,  # Set display options to show default grid
#         }

#         # Start the omniverse application
#         self.simulation_app = SimulationApp(launch_config=CONFIG)

#         from omni.isaac.core.utils.extensions import enable_extension

#         # Default Livestream settings
#         self.simulation_app.set_setting("/app/window/drawMouse", True)
#         self.simulation_app.set_setting("/app/livestream/proto", "ws")
#         self.simulation_app.set_setting("/app/livestream/websocket/framerate_limit", 120)
#         self.simulation_app.set_setting("/ngx/enabled", False)
#         self.simulation_app.set_setting("/exts/omni.services.transport.server.http/port", 8201)
#         self.simulation_app.set_setting("/app/livestream/websocket/server_port", 8886)

#         enable_extension("omni.services.streamclient.webrtc")
#         enable_extension("omni.isaac.ros_bridge")

#         self.args = args
#         self._skip_frame = skip_frame
#         self._dt = physics_dt * self._skip_frame

#         from omni.isaac.core import World
#         from omni.isaac.wheeled_robots.robots import WheeledRobot
#         from omni.isaac.sensor import RotatingLidarPhysX
#         from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController

#         self._my_world = World(physics_dt=physics_dt, rendering_dt=rendering_dt, stage_units_in_meters=1.0)
#         self._my_world.scene.add_default_ground_plane()

#         jackal_asset_path = "/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_description/urdf/jackal_1.usda"

#         theta = 90 * np.pi / 180  # 90 degrees
#         q_rot = np.array([np.cos(theta / 2), 0, 0, np.sin(theta / 2)])

#         self.jackal = self._my_world.scene.add(
#             WheeledRobot(
#                 prim_path="/jackal",
#                 name="my_jackal",
#                 wheel_dof_names=["front_right_wheel_joint", "rear_right_wheel_joint",
#                                  "rear_left_wheel_joint", "front_left_wheel_joint"],
#                 create_robot=True,
#                 usd_path=jackal_asset_path,
#                 position=np.array([-1, 0, 0.07]),
#                 orientation=q_rot
#             )
#         )
#         self.jackal_controller = DifferentialController(name="simple_control", wheel_radius=0.098, wheel_base=4.2)

#     #     self.lidar = self._my_world.scene.add(
#     #         RotatingLidarPhysX(
#     #             prim_path="/jackal/base_link/sick_lms1xx_lidar_frame/Lidar",
#     #             name="Lidar",
#     #         )
#     #     )
#     #     rospy.init_node('scan_values')
#     #     self.laser_sub = rospy.Subscriber(
#     #         'laser_scan', LaserScan, self.laser_scan_callback
#     #     )
#     # def laser_scan_callback(self, scan):
#     #     self.scan = np.array(scan.ranges)
#     #     # print("scan type: ",type(scan))
#     #     # self.scan = scan
#     #     # print("ROS lidar size:", self.scan.shape)

#     def play(self):
#         i = 0  # Initialize the variable `i`
#         while self.simulation_app.is_running():
#             self._my_world.step(render=True)
#             if self._my_world.is_playing():
#                 if self._my_world.current_time_step_index == 0:
#                     self._my_world.reset()
#                     self.jackal_controller.reset()
#                 if i >= 0 and i < 1000:
#                     # forward
#                     self.jackal.apply_wheel_actions(self.jackal_controller.forward(command=[0.05, 0]))
#                     print(self.jackal.get_linear_velocity())
#                 elif i >= 1000 and i < 1300:
#                     # rotate
#                     self.jackal.apply_wheel_actions(self.jackal_controller.forward(command=[0.0, np.pi / 12]))
#                     print(self.jackal.get_angular_velocity())
#                 elif i >= 1300 and i < 2000:
#                     # forward
#                     self.jackal.apply_wheel_actions(self.jackal_controller.forward(command=[0.05, 0]))
#                 elif i == 2000:
#                     i = 0
#                 i += 1
#             if self.args.test is True:
#                 break
#         self.simulation_app.close()  # Ensure proper shutdown

# if __name__ == '__main__':
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
#     args, unknown = parser.parse_known_args()

#     try:
#         jackal_launch = JackalEnv(args=args)
#         jackal_launch.play()
#     except rospy.ROSInterruptException:
#         pass

import argparse
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from omni.isaac.kit import SimulationApp
import subprocess

class JackalEnv:

    def __init__(self, args, skip_frame=1, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, seed=0, headless=True) -> None:
        CONFIG = {
            "width": 1280,
            "height": 720,
            "window_width": 1920,
            "window_height": 1080,
            "headless": headless,
            "renderer": "RayTracedLighting",
            "display_options": 3286,
        }

        self.simulation_app = SimulationApp(launch_config=CONFIG)
        from omni.isaac.core.utils.extensions import enable_extension

        self.simulation_app.set_setting("/app/window/drawMouse", True)
        self.simulation_app.set_setting("/ngx/enabled", False)
        self.simulation_app.set_setting("/exts/omni.services.transport.server.http/port", 8201)
        enable_extension("omni.services.streamclient.webrtc")
        enable_extension("omni.isaac.ros_bridge")

        self.args = args
        from omni.isaac.core import World
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
        from omni.isaac.sensor import RotatingLidarPhysX
        from omni.isaac.core.objects import FixedCuboid
        import omni.graph.core as og
        
        self.og = og
        # vel attributes
        self.left_wheel_vel = []
        self.right_wheel_vel = []
        self.linear_vel = 0
        self.angular_vel = 0

        self._my_world = World(stage_units_in_meters=1.0)
        self._my_world.scene.add_default_ground_plane()

        theta = 90 * np.pi / 180
        q_rot = np.array([np.cos(theta / 2), 0, 0, np.sin(theta / 2)])

        jackal_asset_path = "/isaac-sim/IsaacSim-ros_workspaces/noetic_ws/src/jackal/jackal_description/jackal/jackal.usd"
        self.jackal = self._my_world.scene.add(
            WheeledRobot(
                prim_path="/jackal",
                name="my_jackal",
                wheel_dof_names=["front_right_wheel", "rear_right_wheel", "rear_left_wheel", "front_left_wheel"],
                create_robot=True,
                usd_path=jackal_asset_path,
                position=np.array([-1, 0, 0.07]),
                orientation=q_rot
            )
        )
        self.jackal_controller = DifferentialController(name="simple_control", wheel_radius=0.098, wheel_base=4.2)
        self._my_world.reset()

        self.obstacles = self._my_world.scene.add(
            FixedCuboid(
                prim_path="/obstacles_1",
                name="obstacles_cube",
                scale=np.array([3, 3, 20]),
                position=np.array([-4, 0, 0]),
                size=0.1,
                color=np.array([1.0, 1, 1]),
            )
        )

        self.lidar = self._my_world.scene.add(
            RotatingLidarPhysX(
                prim_path="/jackal/front_mount/Lidar",
                name="Lidar",
            )
        )

        self.lidar.enable_visualization(high_lod=False, draw_points=True, draw_lines=True)

        rospy.init_node('scan_values')
        self.laser_sub = rospy.Subscriber('laser_scan', LaserScan, self.laser_scan_callback)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist ,self.cmd_vel_callback)
        self.action_graph_setup()

    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    # def wheel_vel_callback(self, msg):
    #     self.left_wheel_vel = [msg.data[0],msg.data[1]]
    #     self.right_wheel_vel = [msg.data[2],msg.data[3]]
    #     self.right_wheel_vel.extend(left_wheel_vel)
    #     self.final_joints_vel = right_wheel_vel

    def laser_scan_callback(self, scan):
        self.scan = np.array(scan.ranges)

    def action_graph_setup(self):
        self.og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                self.og.Controller.Keys.CREATE_NODES: [
                    # ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ClockPublisher", "omni.isaac.ros_bridge.ROS1PublishClock"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ReadLidarBeams", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                    ("PublishLaserScan", "omni.isaac.ros_bridge.ROS1PublishLaserScan"),
                    ("ComputeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                    ("PublishOdometry", "omni.isaac.ros_bridge.ROS1PublishOdometry"),
                    ("PublishOdometryTfXform", "omni.isaac.ros_bridge.ROS1PublishRawTransformTree"),
                    ("tf_baselink_chassis", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                    ("tf_midmount_down", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                    ("tf_frontmount_lidar", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                    ("tf_chassis_down", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                ],
                self.og.Controller.Keys.CONNECT: [
                    # ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    # ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "tf_baselink_chassis.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "tf_baselink_chassis.inputs:timeStamp"),
                    ("OnPlaybackTick.outputs:tick", "tf_midmount_down.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "tf_midmount_down.inputs:timeStamp"),
                    ("OnPlaybackTick.outputs:tick", "tf_frontmount_lidar.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "tf_frontmount_lidar.inputs:timeStamp"),
                    ("OnPlaybackTick.outputs:tick", "tf_chassis_down.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "tf_chassis_down.inputs:timeStamp"),
                    ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishOdometryTfXform.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishOdometryTfXform.inputs:timeStamp"),
                    ("ComputeOdom.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                    ("ComputeOdom.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                    ("ComputeOdom.outputs:orientation", "PublishOdometry.inputs:orientation"),
                    ("ComputeOdom.outputs:position", "PublishOdometry.inputs:position"),
                    ("ComputeOdom.outputs:orientation", "PublishOdometryTfXform.inputs:rotation"),
                    ("ComputeOdom.outputs:position", "PublishOdometryTfXform.inputs:translation"),
                    ("OnPlaybackTick.outputs:tick", "ClockPublisher.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "ClockPublisher.inputs:timeStamp"),
                    ("OnPlaybackTick.outputs:tick", "ReadLidarBeams.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishLaserScan.inputs:timeStamp"),
                    ("ReadLidarBeams.outputs:execOut", "PublishLaserScan.inputs:execIn"),
                    ("ReadLidarBeams.outputs:azimuthRange", "PublishLaserScan.inputs:azimuthRange"),
                    ("ReadLidarBeams.outputs:depthRange", "PublishLaserScan.inputs:depthRange"),
                    ("ReadLidarBeams.outputs:horizontalFov", "PublishLaserScan.inputs:horizontalFov"),
                    ("ReadLidarBeams.outputs:horizontalResolution", "PublishLaserScan.inputs:horizontalResolution"),
                    ("ReadLidarBeams.outputs:intensitiesData", "PublishLaserScan.inputs:intensitiesData"),
                    ("ReadLidarBeams.outputs:linearDepthData", "PublishLaserScan.inputs:linearDepthData"),
                    ("ReadLidarBeams.outputs:numCols", "PublishLaserScan.inputs:numCols"),
                    ("ReadLidarBeams.outputs:numRows", "PublishLaserScan.inputs:numRows"),
                    ("ReadLidarBeams.outputs:rotationRate", "PublishLaserScan.inputs:rotationRate"),
                ],
                self.og.Controller.Keys.SET_VALUES: [
                    # ("PublishJointState.inputs:targetPrim", "/jackal"),
                    ("ReadLidarBeams.inputs:lidarPrim", "/jackal/front_mount/Lidar"),
                    ("PublishLaserScan.inputs:topicName", "/laser_scan"),
                    ("PublishLaserScan.inputs:frameId", "Lidar"),
                    ("ComputeOdom.inputs:chassisPrim", "/jackal"),
                    ("PublishOdometryTfXform.inputs:parentFrameId", "/odom"),
                    ("PublishOdometryTfXform.inputs:childFrameId", "/base_link"),
                    ("tf_baselink_chassis.inputs:parentPrim", "/jackal/base_link"),
                    ("tf_baselink_chassis.inputs:targetPrims", "/jackal/chassis_link"),
                    ("tf_midmount_down.inputs:parentPrim", "/jackal/mid_mount"),
                    ("tf_midmount_down.inputs:targetPrims", ["/jackal/front_mount","/jackal/rear_mount"]),
                    ("tf_frontmount_lidar.inputs:parentPrim", "/jackal/front_mount"),
                    ("tf_frontmount_lidar.inputs:targetPrims", "/jackal/front_mount/Lidar"),
                    ("tf_chassis_down.inputs:parentPrim", "/jackal/chassis_link"),
                    ("tf_chassis_down.inputs:targetPrims", [
                        "/jackal/front_fender_link","/jackal/front_left_wheel_link","/jackal/front_right_wheel_link",
                        "/jackal/rear_left_wheel_link","/jackal/rear_right_wheel_link","/jackal/imu_link","/jackal/mid_mount",
                        "/jackal/navsat_link","/jackal/rear_fender_link"
                    ]),
                ],
            },
        )

    def play(self):
        i = 0
        while self.simulation_app.is_running():
            self._my_world.step(render=True)
            if self._my_world.is_playing():
                if self._my_world.current_time_step_index == 0:
                    self._my_world.reset()
                    self.jackal_controller.reset()
                    # print("control resetted")
                # if i >= 0 and i < 1000:
                #     self.jackal.apply_wheel_actions(self.jackal_controller.forward(command=[0.05, np.pi / 12]))
                #     print("Linear Velocity: ", self.jackal.get_linear_velocity())
                # elif i >= 1000 and i < 1300:
                #     self.jackal.apply_wheel_actions(self.jackal_controller.forward(command=[0.0, np.pi / 12]))
                #     print("Angular Velocity: ", self.jackal.get_angular_velocity())
                # elif i >= 1300 and i < 2000:
                #     self.jackal.apply_wheel_actions(self.jackal_controller.forward(command=[0.05, 0]))
                # elif i == 2000:
                #     i = 0
                # i += 1
                # if i >= 0 and i < 3000:
                print(self.linear_vel, self.angular_vel)
                self.jackal.apply_wheel_actions(self.jackal_controller.forward(command=[self.linear_vel, self.angular_vel]))
                print("Linear Velocity: ", self.jackal.get_linear_velocity())
                print("Angular Velocity: ", self.jackal.get_angular_velocity())
            if self.args.test is True:
                break
        self.simulation_app.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
    args, unknown = parser.parse_known_args()

    try:
        jackal_launch = JackalEnv(args=args)
        jackal_launch.play()
    except rospy.ROSInterruptException:
        pass

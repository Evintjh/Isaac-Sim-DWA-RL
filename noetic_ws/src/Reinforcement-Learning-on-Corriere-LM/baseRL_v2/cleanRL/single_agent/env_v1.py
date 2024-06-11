# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import sys
import os

# Get the current directory (main_directory/script.py's directory)
current_dir = os.path.dirname(os.path.abspath(__file__))

# Get the parent directory (main_directory)
parent_dir = os.path.dirname(current_dir)

# Add the parent directory to sys.path
sys.path.append(parent_dir)
import gym
from gym import spaces
import numpy as np
import math
import carb
import rospy
from sensor_msgs.msg import LaserScan
import tf
from collections import deque

import time


class JackalEnv(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        args,
        skip_frame=1,
        physics_dt=1.0 / 120.0,
        rendering_dt=1.0 / 60.0,
        # max_episode_length=256,
        seed=0,
        headless=True,
    ) -> None:
        
        # This sample enables a livestream server to connect to when running headless
        CONFIG = {
            # "width": 1280,
            # "height": 720,
            "width": 1920,
            "height": 1080,
            # "window_width": 1920,
            # "window_height": 1080,
            "window_width": 2560,
            "window_height": 1440,
            "headless": True,
            "renderer": "RayTracedLighting",
            "display_options": 3286,  # Set display options to show default grid
        }
        from omni.isaac.kit import SimulationApp
        self.simulation_app = SimulationApp(launch_config=CONFIG)
        from omni.isaac.core.utils.extensions import enable_extension

        # Default Livestream settings
        self.simulation_app.set_setting("/app/window/drawMouse", False)
        self.simulation_app.set_setting("/app/livestream/proto", "ws")
        self.simulation_app.set_setting("/app/livestream/websocket/framerate_limit",40)
        self.simulation_app.set_setting("/ngx/enabled", False)
        
        # set different websocket server port to run RL training on second container
        # self.simulation_app.set_setting("/app/livestream/websocket/server_port",8886)
        # self.simulation_app.set_setting("/exts/omni.services.transport.server.http/port",8201)

        # Default URL: http://localhost:8211/streaming/client/ , change 8211 to customised transport server port
        enable_extension("omni.kit.livestream.native")
        # enable_extension("omni.services.streamclient.websocket")
        enable_extension("omni.isaac.ros_bridge")
        enable_extension("omni.isaac.physics_inspector")

        self.args =args
        self._skip_frame = skip_frame
        self._dt = physics_dt * self._skip_frame
        # self._max_episode_length = max_episode_length
        self._steps_after_reset = int(rendering_dt / physics_dt)
        from omni.isaac.core import World
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        from omni.isaac.sensor import RotatingLidarPhysX
        from env_setup_utils.differential_controller_v1 import DifferentialController
        from omni.isaac.core.objects import VisualCuboid
        from omni.isaac.core.objects import FixedCuboid


        self._my_world = World(physics_dt=physics_dt, rendering_dt=rendering_dt, stage_units_in_meters=1.0)
        self._my_world.scene.add_default_ground_plane()
       
        # jackal_asset_path = "/isaac-sim/src/jackal.usda"
        jackal_asset_path = "src/baseRL_v2/cleanRL/single_agent/jackal.usda"


        theta = 90*np.pi/180  # 90 degrees
        # Create the rotation quaternion
        q_rot = np.array([np.cos(theta/2), 0, 0, np.sin(theta/2)])

        self.jackal = self._my_world.scene.add(
            WheeledRobot(
                prim_path="/jackal",
                name="my_jackal",
                wheel_dof_names=[
                    "front_left_wheel_joint",
                    "front_right_wheel_joint",
                    "rear_left_wheel_joint",
                    "rear_right_wheel_joint",
                ],
                create_robot=True,
                usd_path=jackal_asset_path,
                position=np.array([0, 0, 0.0]),
                orientation=q_rot
            )
        )

        self.jackal_controller = DifferentialController(name="simple_control", 
                                                        wheel_radius=0.0975, wheel_base=0.37559*1.35)

        self.lidar = self._my_world.scene.add(
            RotatingLidarPhysX(
                prim_path="/jackal/base_link/sick_lms1xx_lidar_frame/Lidar",
                name="Lidar",
            )
        )
        rospy.init_node('scan_values')
        self.laser_sub = rospy.Subscriber(
            'laser_scan', LaserScan, self.laser_scan_callback
        )

        self.obstacles = self._my_world.scene.add(
            FixedCuboid(
                prim_path="/obstacles_1",
                name="obstacles_cube1",
                scale=np.array([2, 2, 10]),
                position=np.array([2.5, 0, 0]),
                size=0.25,
                color=np.array([1.0, 1, 1]),
            )
        )
        
        self.goal = self._my_world.scene.add(
            VisualCuboid(
                prim_path="/goal_cube_1",
                name="visual_cube",
                position=np.array([5, 0, 0.05]),
                size=0.1,
                color=np.array([1.0, 0, 0]),
            )
        )

        self.seed(seed)
        self.reward_range = (-float("inf"), float("inf"))
        gym.Env.__init__(self)
        self.action_space = spaces.Box(low=-1, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=float("inf"), high=float("inf"), shape=(364,), dtype=np.float32)

        self.max_velocity = 1
        self.max_angular_velocity = math.pi
        self.reset_counter = 0
        self.last_action = [0,0]
        self.spawn_distance = 0
        self.scan = np.ones((360,))*10
        self.local_goal = np.empty((2,))
        self.success_array = deque(maxlen=100)
        self.success_rate = 0
        self.num_steps = 0  # counting number of iterations for each episode
      
        self.reset()
    
        print("done init")
        return

    def laser_scan_callback(self, scan):
        self.scan = np.array(scan.ranges)
        

    def get_dt(self):
        return self._dt

    def step(self, action):

        self.last_action = action
        action = np.clip(action, [0, -self.args.w_max], [self.args.v_max, self.args.w_max])
      
   
        for i in range(self._skip_frame):
            self.jackal.apply_wheel_actions(
                self.jackal_controller.forward(command=action)
            )
            self._my_world.step(render=False)

        #print(f'\t{self.num_steps} Before obs: collision_distance={self.args.collision_distance} np.amin(self.scan)={np.amin(self.scan)} condition={np.amin(self.scan) < self.args.collision_distance}')
        observations = self.get_observations()
        #print(f'\t{self.num_steps} After  obs: collision_distance={self.args.collision_distance} np.amin(self.scan)={np.amin(self.scan)} condition={np.amin(self.scan) < self.args.collision_distance}')
        info = {}
        done = False

        #print(f'\t{self.num_steps} Before rew: collision_distance={self.args.collision_distance} np.amin(self.scan)={np.amin(self.scan)} condition={np.amin(self.scan) < self.args.collision_distance}')
        reward_new, done_info = self.compute_reward(self.local_goal,self.scan)
        #print(f'\t{self.num_steps} After  rew: collision_distance={self.args.collision_distance} np.amin(self.scan)={np.amin(self.scan)} condition={np.amin(self.scan) < self.args.collision_distance}')
        done = done_info[0]
        info["done_reason"] = done_info
        if done:
            self.success_array.append(done_info[1] == 1)
            self.success_rate = np.mean(self.success_array)
        
        return observations, reward_new, done, info
    
    def compute_reward(self, local_goal, scan):
        goal_reward = 0
        collision_reward = 0
        timeout_reward = 0
        distance_reward = 0
        social_reward = 0

        dist_to_goal = math.hypot(local_goal[0], local_goal[1])

        if dist_to_goal < self.args.goal_radius:
            print("Reached goal")
            done_info = [True, 1]  # Reach Goalum_steps
        # elif np.amin(scan) < self.args.collision_distance and self.num_steps>= 100: #first 10 frame of scan has no data (maybe related to lidar freq? 10hz)
        elif np.amin(scan) < self.args.collision_distance:
            # print(scan)
            print("Collision")
            done_info = [True, 2]  # Collision
        elif self.num_steps > self.args.timeout:
            print("Timeout")
            done_info = [True, 3]  # Timeout
        else:
            done_info = [False, 0]  # Nothing


        if self.args.final_dist_reward:
            if done_info[0]: #from spawn to goalacvt
                distance_reward = self.spawn_distance - dist_to_goal
                # print(distance_reward)
        else:
            distance_reward = (self.last_distance - dist_to_goal)*5
            if abs(distance_reward) > 0.5:
                distance_reward = 0  # To prevent init bug
       
        self.last_distance = dist_to_goal

        if dist_to_goal < self.args.goal_radius:
            goal_reward = 15  # Reach Goal
        if np.amin(scan) < self.args.collision_distance:
            collision_reward = -15  # Collision
        if self.num_steps > self.args.timeout:
            timeout_reward = -15  # Timeout
        if np.amin(scan) < 1.0:
            social_reward = -self.args.social_penalty  # Collision for getting to near

            #big rotation? can just ignore for now

        reward = (distance_reward + collision_reward + goal_reward + timeout_reward + social_reward)
        self.num_steps += 1
        return float(reward), done_info

    def reset(self):
        self._my_world.reset()
        self.scan = np.ones((360,))*10
        self.num_steps = 0
        self.reset_counter = 0
        
        # randomize goal location in circle around robot
        alpha = 2 * math.pi * np.random.rand()
        r = np.random.uniform(low=3.0, high=5.0)
        self.goal.set_world_pose(np.array([math.sin(alpha) * r, math.cos(alpha) * r, 0.05]))

        # randomize obstacle location in circle around robot
        obstacles_r = np.random.uniform(low=1.0, high=2.5)
        obstacles_alpha = alpha + np.random.uniform(low=-30.0, high=30.0)/180* math.pi
        self.obstacles.set_world_pose(np.array([math.sin(obstacles_alpha) * obstacles_r, math.cos(obstacles_alpha) * obstacles_r, 0.0]))
        
        # randomize jackal position

        # jackal_x = np.random.uniform(0, 0)
        # jackal_y = np.random.uniform(-1,1)
        # random_orien = np.random.choice([-1, 1])
        # theta = random_orien*0*np.pi/180  
        # q_rot = np.array([np.cos(theta/2), 0, 0, np.sin(theta/2)])
        # self.jackal.set_world_pose(np.array([jackal_x,jackal_y,0]),q_rot)
        
        observations = self.get_observations()
        self.last_distance = math.hypot(self.local_goal[0], self.local_goal[1])
        self.spawn_distance = math.hypot(self.local_goal[0], self.local_goal[1])
     
        return observations
    
    def get_local_goal(self):
        #robot_odom = [caretsian(x,y,z), quaternion(w,x,y,z)]
        robot_odom = self.jackal.get_world_pose()
        goal_pose, _ = self.goal.get_world_pose()
        Quaternions = robot_odom[1]
        Euler = tf.transformations.euler_from_quaternion(
            [Quaternions[1], Quaternions[2], Quaternions[3], Quaternions[0]]
        )
        x, y, theta = robot_odom[0][0], robot_odom[0][1], Euler[2]
        goal_x, goal_y = goal_pose[0], goal_pose[1]
        local_x = (goal_x - x) * np.cos(theta) + (goal_y - y) * np.sin(theta)
        local_y = -(goal_x - x) * np.sin(theta) + (goal_y - y) * np.cos(theta)
        return local_x, local_y

    def get_observations(self):
        self._my_world.render()
        jackal_linear_velocity = self.jackal.get_linear_velocity()
        jackal_angular_velocity = self.jackal.get_angular_velocity()
        self.local_goal = self.get_local_goal() #relative goalpose to robot

        scan = self.scan/10 -0.5
        state = np.concatenate(
            (
                scan,
                self.local_goal[0],
                self.local_goal[1],
                # self.last_action, can be ignore for now, can be add in later stage
                jackal_linear_velocity[0],
                jackal_angular_velocity[2]
            ), axis = None,dtype=np.float32
        )
        return state
        
    def render(self, mode="human"):
        return

    def close(self):
        self.simulation_app.close()
        return

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        np.random.seed(seed)
        return [seed]

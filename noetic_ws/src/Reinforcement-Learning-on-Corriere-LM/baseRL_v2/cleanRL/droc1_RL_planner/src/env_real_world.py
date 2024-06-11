# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import gym
from gym import spaces
import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
import tf.transformations as tf
from collections import deque
import time
from geometry_msgs.msg import Twist, Quaternion, PointStamped, Point, Vector3
from nav_msgs.msg import Odometry


class JackalEnv(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        args,
        # max_episode_length=256,
        seed=0,
        headless=True,
    ) -> None:
        
        
        rospy.init_node('RL_local_planner')
        self.rate = rospy.Rate(10)
        self.laser_sub = rospy.Subscriber(
             'scan', LaserScan, self.laser_scan_callback
         )
        rospy.Subscriber('/localization/core/proprioceptive_odometry/2d/data/pose',
                         Odometry,self.odom_callback)
        
        rospy.Subscriber('/clicked_point',
                        PointStamped,self.goal_pose_callback)
        
        self.action_pub = rospy.Publisher('/set_vel',Twist,queue_size=10)

      
        self.args = args
        self._skip_frame = 1
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
        self.robot_world_pose_quat = Quaternion()
        self.robot_world_pose_cart = Point()
        self.robot_linear_velo = Vector3()
        self.robot_angular_velo = Vector3()
        self.goal_world_pose = Point()
        self.local_goal = np.empty((2,))
        self.success_array = deque(maxlen=100)
        self.success_rate = 0
        self.num_steps = 0  # counting number of iterations for each episode
        self.reset()

        print("done init")
        return

    def laser_scan_callback(self, scan):
        #pass
        self.scan = np.array(scan.ranges) #If this line is commented out, lidar will work. If this line is uncommented, lidar will NOT work
        print("scan: ",self.scan[180])

    def odom_callback(self,data):
        
        self.robot_world_pose_quat = data.pose.pose.orientation #Quaternion
        self.robot_world_pose_cart = data.pose.pose.position #cartesian
        self.robot_linear_velo = data.twist.twist.linear
        self.robot_angular_velo = data.twist.twist.angular

    def goal_pose_callback(self,data):
        self.goal_world_pose = data.point


    def get_dt(self):
        return self._dt

    def step(self, action):
        self.last_action = action
        action = np.clip(action, [0, -self.args.w_max], [self.args.v_max, self.args.w_max])
        set_vel_msgs = Twist()
        set_vel_msgs.linear.x = action[0]
        set_vel_msgs.angular.z = action[1]
        for i in range(self._skip_frame):
            self.action_pub.publish(set_vel_msgs)
            self.rate.sleep()

        observations = self.get_observations()
        info = {}
        done = False

        reward_new, done_info = self.compute_reward(self.local_goal,self.scan)
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
            
            
            
            print("Reahced goal")
            set_vel_msgs = Twist()
            set_vel_msgs.linear.x = 0
            set_vel_msgs.angular.z = 0
            self.action_pub.publish(set_vel_msgs)
            done_info = [True, 1]  # Reach Goalum_steps
        elif np.amin(scan) < self.args.collision_distance:
            print("Collsion")
            done_info = [True, 2]  # Collision
        elif self.num_steps > self.args.timeout:
            print("Timeout")
            done_info = [True, 3]  # Timeout
        else:
            done_info = [False, 0]  # Nothing


        if self.args.final_dist_reward:
            if done_info[0]: #from spawn to goal
                distance_reward = self.spawn_distance - dist_to_goal
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
        self.scan = np.ones((360,))*10
        self.num_steps = 0
        self.reset_counter = 0
        
        observations = self.get_observations()
        self.last_distance = math.hypot(self.local_goal[0], self.local_goal[1])
        self.spawn_distance = math.hypot(self.local_goal[0], self.local_goal[1])
     
        return observations
    
    def get_local_goal(self):
        #robot_odom = [caretsian(x,y,z), quaternion(w,x,y,z)]
        robot_odom = self.robot_world_pose_cart
        goal_pose = self.goal_world_pose
        Quaternions = self.robot_world_pose_quat
        Euler = tf.euler_from_quaternion(
            [Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w]
        )
        x, y, theta = robot_odom.x, robot_odom.y, Euler[2]

        goal_x, goal_y = goal_pose.x, goal_pose.y
        local_x = (goal_x - x) * np.cos(theta) + (goal_y - y) * np.sin(theta)
        local_y = -(goal_x - x) * np.sin(theta) + (goal_y - y) * np.cos(theta)
        return local_x, local_y

    def get_observations(self):
        jackal_linear_velocity = self.robot_linear_velo
        jackal_angular_velocity = self.robot_angular_velo
    
        self.local_goal = self.get_local_goal() #relative goalpose to robot

        scan = self.scan/10 -0.5
        state = np.concatenate(
            (
                scan,
                self.local_goal[0],
                self.local_goal[1],
                # self.last_action, can be ignore for now, can be add in later stage
                jackal_linear_velocity.x,
                jackal_angular_velocity.z
            ), axis = None,dtype=np.float32
        )
        return state
        
       
    

    def render(self, mode="human"):
        return

    def close(self):
        self._simulation_app.close()
        return

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        np.random.seed(seed)
        return [seed]

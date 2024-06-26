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
import carb
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from collections import deque
import time
from env_setup_utils.setup_simulation_app import setup_simulation_app
from env_setup_utils.env_setup import setup_action_graph




class JackalEnv(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        args,
        seed=0,
    ) -> None:
        self._simulation_app = setup_simulation_app()
        
        from omni.isaac.core import World
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        from omni.isaac.sensor import RotatingLidarPhysX
        from env_setup_utils.differential_controller_multiagent import DifferentialController
        from omni.isaac.core.objects import VisualCuboid
        from omni.isaac.core.objects import FixedCuboid
        from omni.isaac.cloner import GridCloner
        from omni.isaac.core.utils.prims import get_prim_at_path, define_prim
        from omni.isaac.core.utils.stage import get_current_stage, add_reference_to_stage
        from omni.isaac.core.articulations import ArticulationView
        import omni.graph.core as og
        from omni.isaac.core.prims import XFormPrimView, RigidPrimView
        import omni.replicator.isaac as dr
        import omni.replicator.core as rep
        from domain_randomization.randomize import Randomizer



        self.args = args
        self.num_agents = self.args.num_agents
        physics_dt = 1.0/self.args.physics_dt
        rendering_dt = 1.0/self.args.rendering_dt
        # physics_dt = 1.0/10.0
        # rendering_dt = 1.0/60.0
        self._skip_frame = self.args.skip_frame
        self._dt = physics_dt * self._skip_frame
        self._steps_after_reset = int(rendering_dt / physics_dt)
    
        self._my_world = World(physics_dt=physics_dt, rendering_dt=rendering_dt, stage_units_in_meters=1.0)
        self._my_world.scene.add_default_ground_plane()

        setup_action_graph(self.num_agents)

        jackal_asset_path = self.args.jackal_asset_path

        cloner = GridCloner(spacing=self.args.grid_spacing)
        cloner.define_base_env("/envs")
        define_prim("/envs/env_0")
        VisualCuboid(
                prim_path="/envs/env_0/goal",name="visual_cube",
                position=np.array([-3, 0, 0.05]),size=0.1,
                color=np.array([1.0, 0, 0]),
            )
        FixedCuboid(
            prim_path="/envs/env_0/obstacles_1",
            name="obstacles_cube1",
            scale=np.array([2, 2, 10]),
            position=np.array([2.5, 0, 0]),
            size=0.25,
            color=np.array([1.0, 1, 1])
        )
        add_reference_to_stage(
        usd_path=jackal_asset_path, prim_path="/envs/env_0/jackal"
        )

        # clone environments
        
        prim_paths = cloner.generate_paths("/envs/env", self.num_agents)
        self.env_pos = cloner.clone(source_prim_path="/envs/env_0", prim_paths=prim_paths)

        # print("env_pos:  ", self.env_pos)
        self.jackal_articulation = ArticulationView(prim_paths_expr="/envs/*/jackal", name="jackal_view")
        print("friction coeeficients: ",self.jackal_articulation._physics_view)
        self._my_world.scene.add(self.jackal_articulation)
        # self.jackal_prim = XFormPrimView("/envs/*/jackal")
        self.obstacle_prim = XFormPrimView("/envs/*/obstacles_1",reset_xform_properties=False)
        self._my_world.scene.add(self.obstacle_prim)
        self.goal_prim = XFormPrimView(prim_paths_expr="/envs/*/goal", name="goal_view")
        self._my_world.scene.add(self.goal_prim)


        rospy.init_node("RL_agent")
        for i in range(self.num_agents):
            rospy.Subscriber(f"laser_scan_{i}", LaserScan, self.laser_scan_callback,i)
            rospy.Subscriber(f"Odom{i}", Odometry, self.odom_callback, i)


        self.jackal_controller = DifferentialController(name="simple_control", num_agents=self.num_agents,wheel_radius=0.0975, 
                                                        wheel_base=0.37559*1.35)
    

        self.seed(seed)
        self.reward_range = (-float("inf"), float("inf"))
        gym.Env.__init__(self)
        self.action_space = spaces.Box(low=-1, high=1.0, shape=(self.num_agents,2), dtype=np.float32)
        self.observation_space = spaces.Box(low=float("inf"), high=float("inf"), shape=(self.num_agents,self.args.num_stacks,364), dtype=np.float32)

        self.last_action = [0, 0]
        self.spawn_distance = np.zeros((self.num_agents))
        self.last_distance = np.zeros((self.num_agents))
        self.scan = np.ones((self.num_agents,360))*10
        # print("scan:  ", self.scan)
        self.local_goal = np.empty((self.num_agents,2))
        self.num_steps = np.zeros((self.num_agents))  # counting number of iterations for each episode
        self.jackal_prev_linear_velo = 0
        self.jackal_prev_ang_velo = 0
        self.Quaternions = np.zeros((self.num_agents))         # if self.num_agents  = 3 -> 1 x 3 matrix? --> [0 0 0], hprizontal row across
        self.pos = np.zeros((self.num_agents))
        self.robot_odom_ros = np.zeros(2,1)                     # -> creates ([0],[0]) -> 2 by 1 matrix?
        self.stacked_obs = np.zeros((self.num_agents,self.args.num_stacks,364))
        self.rewards = np.zeros((self.num_agents,),dtype=object)
        for i in range(self.num_agents):
            self.rewards[i] = []
        self.dones = np.zeros((self.num_agents)) #[num_agents, true/False]
        self.infos = [{} for _ in range(self.num_agents)]

        self._dr_randomizer = Randomizer(args)
        self._my_world.reset()
        self.reset() 

        
        if self._dr_randomizer.randomize:
            import omni.replicator.isaac as dr
            self.dr = dr
        if self._dr_randomizer.randomize:
            self._dr_randomizer.apply_on_startup_domain_randomization(self)
            self._dr_randomizer.set_up_domain_randomization(self)

        

        print("done init")
        return

    def odom_callback(self, msg, idx):
        self.Quaternions[idx] = np.array(msg.pose.pose.angular.w, msg.pose.pose.angular.x, msg.pose.pose.angular.y, msg.pose.pose.angular.z)
        self.pos[idx] = np.array(msg.pose.pose.linear.x, msg.pose.pose.linear.y, msg.pose.pose.linear.z)
        self.robot_odom_ros = np.array(self.pos[idx], self.Quaternions[idx])

    def laser_scan_callback(self, scan,idx):
        # print(idx)
        # print("scan shape: ",np.array(scan.ranges))
        # print(self.scan)
        self.scan[idx] = np.array(scan.ranges)
        # print("scan type: ",scan)
        # print("ROS lidar size:", self.scan.shape)

    def get_dt(self):
        return self._dt

    def step(self, action:np.ndarray):
        # print(f'joint_velo_jackal: {self.jackal.get_joint_velocities()}')
        # print(f'joint_pos: {self.jackal.get_dof_index("rear_left_wheel_joint")}')
        jackal_linear_velo = self.jackal_articulation.get_linear_velocities()
        jackal_angular_velo = self.jackal_articulation.get_angular_velocities()
        # print(f"acceleration(step_wise):  {np.round(abs(jackal_linear_velo - self.jackal_prev_linear_velo),3)}{np.round(abs(jackal_angular_velo - self.jackal_prev_ang_velo)  ,3)}",end='\t')
        # print(f"velo(step_wise):  {np.round(abs(jackal_linear_velo),3)}{np.round(abs(jackal_angular_velo  ),3)}")
        self.jackal_prev_linear_velo = jackal_linear_velo
        self.jackal_prev_ang_velo = jackal_angular_velo

        # print("num_steps:  ",self.num_steps)
        self.last_action = action
        # print('action:',action)

        for i in range(self.num_agents):
            action[i] = np.clip(action[i], [0, -self.args.w_max], [self.args.v_max, self.args.w_max])
        # action = np.array([[0,0.5]])
        
        # print(f'action: {round(action[0],3)} {round(action[1],3)}',end='\t ')

        # we apply our actions to the jackal
        for i in range(self._skip_frame):
            self.jackal_articulation.set_joint_velocities(self.jackal_controller.forward(command=action))
            self._my_world.step(render=False)

        observations = self.get_observations()
        
        

        reward, done_info = self.compute_reward(self.local_goal, self.scan)
        # print("done info:",done_info)

        self.dones = done_info[:,0]
        # print("done? :  ",done)
        for i in range(self.num_agents):
            self.rewards[i].append(reward[i])
            if self.dones[i]:
                self.infos[i]["done_reason"] = done_info[i]
                ep_rew = sum(self.rewards[i])
                ep_len = len(self.rewards[i])
                ep_info = {"r": round(ep_rew, 6), "l": ep_len}       
    
                self.infos[i]["episode"] = ep_info

                self.rewards[i] = []
                


            
          

          


        

        # print("env step:", reward, done, info)



        return observations, reward, self.dones, self.infos

    def compute_reward(self, local_goal, scan):
        reward = np.zeros((self.num_agents),dtype=np.float32)
        done_info = np.zeros((self.num_agents,2))

        for i in range(self.num_agents):
            goal_reward = 0
            collision_reward = 0
            timeout_reward = 0
            distance_reward = 0
            social_reward = 0
            # print("laser scan:",np.amin(scan[i]),"  ",i)

            dist_to_goal = math.hypot(local_goal[i][0], local_goal[i][1]) 
            # print(self.num_steps)
            # print(scan)

            if dist_to_goal < self.args.goal_radius:
                print(f'Agent_{i} Reahced goal')
                done_info[i] = [True, 1]  # Reach Goalum_steps
            # elif np.amin(scan) < self.args.collision_distance and self.num_steps>= 100: #first 10 frame of scan has no data (maybe related to lidar freq? 10hz)
            elif np.amin(scan[i]) < self.args.collision_distance:
                
                # print("Collsion")
                done_info[i] = [True, 2]  # Collision
            elif self.num_steps[i] > self.args.timeout:
                print("Timeout")
                done_info[i] = [True, 3]  # Timeout
            else:
                done_info[i] = [False, 0]  # Nothing

            if self.args.final_dist_reward:
                if done_info[i][0]:  # from spawn to goal
                    distance_reward = self.spawn_distance - dist_to_goal
                    # print(distance_reward)
            else:
                distance_reward = (self.last_distance[i] - dist_to_goal) * 5
                if abs(distance_reward) > 0.5:
                    distance_reward = 0  # To prevent init bug

            # print(f"dist_to_goal:  {round(dist_to_goal,3)} \t last_dist:  {round(self.last_distance,3)} ")
            self.last_distance[i] = dist_to_goal

            if dist_to_goal < self.args.goal_radius:
                goal_reward = 15  # Reach Goal
            # print(scan)
            if np.amin(scan[i]) < self.args.collision_distance:
                # print("Collision")
                collision_reward = -15  # Collision
            if self.num_steps[i] > self.args.timeout:
                # print("Timeout")
                timeout_reward = -15  # Timeout

            if np.amin(scan[i]) < 1.0:
                social_reward = -self.args.social_penalty  # Collision for getting to near

                # big rotation? can just ignore for now

            reward[i] = distance_reward + collision_reward + goal_reward + timeout_reward + social_reward
            # print(f'reward[{i}]: ',reward[i])
            # reward /= self.cfg.reward_scale

            # print(f"reward:  {round(reward,3)}")

            self.num_steps[i] += 1
        return reward, done_info

    def reset(self, reset_idx=None):
        if reset_idx == None:
            self.rewards = np.empty((self.num_agents,),dtype=object)
            for i in range(self.num_agents):
                self.rewards[i] = []
            self.dones = np.zeros((self.num_agents)) #[num_agents, true/False]
            self.infos = [{} for _ in range(self.num_agents)]


            self._my_world.reset()
            self.scan = np.ones((self.args.num_agents,360))*10
            print("-------------------reset----------------------------.", self.num_steps)
            self.num_steps = np.zeros((self.num_agents))
        else:
            self.dones[reset_idx] = 0
            # print("resetting agent_",reset_idx)
            import omni.replicator.isaac as dr
            # dr.physics_view.step_randomization(reset_inds=reset_idx)'
            reset_indices = list()
            reset_indices.append(reset_idx)

            jackal_alpha = 2 * math.pi * np.random.rand()
            jackal_r = np.random.uniform(low=2.5, high=4.0)
            
            # randomize goal position
            jackal_reset_pos = np.multiply(np.sin(jackal_alpha) ,jackal_r).reshape((1,-1))
            jackal_reset_pos = np.hstack((jackal_reset_pos, np.multiply(np.cos(jackal_alpha),jackal_r).reshape((1,-1))))
            jackal_reset_pos = np.hstack((jackal_reset_pos, np.ones(1).reshape((1,-1))*0.0645))     
            jackal_reset_pos = jackal_reset_pos + np.array(self.env_pos[reset_idx])

            # print("jackal_reset_pos: ",jackal_reset_pos)
            self.jackal_articulation.set_world_poses(positions=jackal_reset_pos,orientations=np.array([1,0,0,0]).reshape((-1,4)),
                                                     indices=np.array(reset_idx))
            # print("jackal_pos",self.jackal_articulation.get_world_poses()[0][reset_idx])
        

            alpha = 3 * math.pi / 4 + np.random.rand() * (5 * math.pi / 4 - 3 * math.pi / 4) + jackal_alpha
            # print("alpha:  ",alpha.shape)
            r = np.random.uniform(low=2.5, high=4.0)
            # print("r:  ",r.shape)
            goal_reset_pos = np.multiply(np.sin(alpha) ,r).reshape((1,-1))
            goal_reset_pos = np.hstack((goal_reset_pos,np.multiply(np.cos(alpha),r).reshape((1,-1))))
            goal_reset_pos = np.hstack((goal_reset_pos,np.ones(1).reshape((1,-1))*0.05))     
            goal_reset_pos = goal_reset_pos + np.array(self.env_pos[reset_idx])
            # print("env_pos:  ", np.array(self.env_pos).shape)
            # print("goal_reset_pos: ",goal_reset_pos.shape)
            self.goal_prim.set_world_poses(goal_reset_pos,orientations=np.array([1,0,0,0]).reshape((-1,4)),indices=np.array([reset_idx]))

            #random obstacles position
            obstacles_reset_pos = np.random.uniform(low=-1, high=1)
            obstacles_reset_pos = np.hstack((obstacles_reset_pos, np.random.uniform(low=-1, high=1)))
            obstacles_reset_pos = np.hstack((obstacles_reset_pos,0.05))
            obstacles_reset_pos = obstacles_reset_pos + np.array(self.env_pos[reset_idx])
            self.obstacle_prim.set_world_poses(obstacles_reset_pos.reshape((-1,3)),orientations=np.array([1,0,0,0]).reshape((-1,4)),indices=np.array([reset_idx]))

            self.scan[reset_idx] = np.ones((360))*10
            self.num_steps[reset_idx] = 0
            self.stacked_obs[reset_idx] = np.zeros((self.args.num_stacks,364))
            self.rewards[reset_idx] = []
            self.infos[reset_idx] = {}

            self.last_distance[reset_idx] = math.hypot(self.local_goal[reset_idx][0], self.local_goal[reset_idx][1])
            # print(self.last_distance)
            self.spawn_distance[reset_idx] = math.hypot(self.local_goal[reset_idx][0], self.local_goal[reset_idx][1])
            if self._dr_randomizer.randomize:
                self.dr.physics_view.step_randomization(reset_indices)

        observations = self.get_observations()
        # time.sleep(1)
       

        return observations


    def get_local_goal(self):
        # robot_odom = ([caretsian(x,y,z)], [quaternion(w,x,y,z])
        robot_odom = self.robot_odom_ros
        goal_pose, _ = self.goal_prim.get_world_poses()
        local_goal = []
        # goal_pose = goal_pose[0]
        # print("robot_odom: ", robot_odom[1])
        for i in range (self.num_agents):
            Quaternions = robot_odom[1][i]
            robot_linear_velo = robot_odom[0][i]
            Euler = tf.transformations.euler_from_quaternion(
                [Quaternions[1], Quaternions[2], Quaternions[3], Quaternions[0]]
            )
            x, y, theta = robot_linear_velo[0], robot_linear_velo[1], Euler[2]
            goal_x, goal_y = goal_pose[i][0], goal_pose[i][1]
            local_x = (goal_x - x) * np.cos(theta) + (goal_y - y) * np.sin(theta)
            local_y = -(goal_x - x) * np.sin(theta) + (goal_y - y) * np.cos(theta)
            local_goal.append((local_x,local_y))
            # print("local_goal :", local_goal)
        return np.array(local_goal)

    def get_observations(self):
        self._my_world.render()
        jackal_linear_velocity = self.jackal_articulation.get_linear_velocities()[:,0].reshape((self.num_agents,-1))
        jackal_angular_velocity = self.jackal_articulation.get_angular_velocities()[:,2].reshape((self.num_agents,-1))
        # print("linear_velo:", jackal_linear_velocity.shape)
        # print("angular velo: ", jackal_angular_velocity)
        # goal_world_position, _ = self.goal.get_world_pose()
        # print("robot_odom: ", robot_odom)
        # print("goal_world_pos: ",goal_world_position)
        self.local_goal = self.get_local_goal()  # relative goalpose to robot
        # print("scan type: ",type(self.scan),self.scan)
        # print("local_goal:  ", self.local_goal.shape)

        scan = self.scan / 10 - 0.5
        state = np.concatenate(
            (
                scan,
                self.local_goal,
                # self.last_action, can be ignore for now, can be add in later stage
                jackal_linear_velocity,
                jackal_angular_velocity,
            ),
            axis=1,
            dtype=np.float32,
        )
        self.stacked_obs = np.roll(self.stacked_obs, 1, axis=1)
        self.stacked_obs[:,0] = state
        # print("stacked obs space: ", self.stacked_obs)
        return self.stacked_obs

    def render(self, mode="human"):
        return

    def close(self):
        self._simulation_app.close()
        return

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        np.random.seed(seed)
        return [seed]
    
  
            
        

    

    


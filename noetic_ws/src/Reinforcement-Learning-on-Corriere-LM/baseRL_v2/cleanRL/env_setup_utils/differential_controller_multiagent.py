# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.core.utils.types import ArticulationActions
from omni.isaac.core.controllers import BaseController
import numpy as np


class DifferentialController(BaseController):
    """Controller uses unicycle model for a differential drive

        Args:
            name (str): [description]
            wheel_radius (float): Radius of left and right wheels in cms
            wheel_base (float): Distance between left and right wheels in cms
        """

    def __init__(
        self,
        name: str,
        num_agents: int,
        wheel_radius: float,
        wheel_base: float,
        max_linear_speed: float = 1.0e20,
        max_angular_speed: float = 1.0e20,
        max_wheel_speed: float = 1.0e20,
    ) -> None:
        super().__init__(name)
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_wheel_speed = max_wheel_speed
        self.num_agents = num_agents

        assert self.max_linear_speed >= 0
        assert self.max_angular_speed >= 0
        assert self.max_wheel_speed >= 0

    def forward(self, command: np.ndarray):
        """Calculating the wheels speeds given the desired speed for the vehicle.

        Args:
            command (np.ndarray): desired vehicle [forward, rotation] speed

        Returns:
            ArticulationAction: [description]
        """
        if isinstance(command, list):
            command = np.array(command)
        if command.shape[1] != 2:
            raise Exception("command should be of length 2")
        
        actions_list = np.zeros((self.num_agents,4))
        
        for i in range(self.num_agents):
        # command = [num_agents,2]
            command[i] = np.clip(
                command[i],
                a_min=[-self.max_linear_speed, -self.max_angular_speed],
                a_max=[self.max_linear_speed, self.max_angular_speed],
            )
            joint_velocities = [0.0, 0.0,0,0]
            joint_velocities[0] = ((2 * command[i][0]) - (command[i][1] * self.wheel_base)) / (2 * self.wheel_radius)
            joint_velocities[1] = ((2 * command[i][0]) + (command[i][1] * self.wheel_base)) / (2 * self.wheel_radius)
            joint_velocities[2] = joint_velocities[0]
            joint_velocities[3] = joint_velocities[1]
            joint_velocities = np.clip(
                joint_velocities,
                a_min=[-self.max_wheel_speed, -self.max_wheel_speed,-self.max_wheel_speed,-self.max_wheel_speed],
                a_max=[self.max_wheel_speed, self.max_wheel_speed,self.max_wheel_speed,self.max_wheel_speed],
            )

            actions_list[i] = joint_velocities
        return actions_list

    def reset(self) -> None:
        """[summary]
        """
        return

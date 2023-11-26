"""
File to contain the Robot class. Contains all methods to control and receive information regarding
the robot.
"""
from typing import List
import math
import numpy as np

from camera import Camera



class Robot:
    """
    Robot class contains information on position, velocity, and can access camera view
    """

    # set some parameters for all robots
    wheel_diameter = 0.2
    track_width = 0.3

    def __init__(self) -> None:
        """
        Initialize the pose and velocity at zero, initialize camera object
        """
        self._pose = [0,0,0] # x,y,theta
        self._vel = [0,0] # l wheel vel, r wheel vel
        self._camera = Camera()

    @property
    def pose(self) -> List[int]:
        """
        Return the pose of the robot as list of [x_pos, y_pos, theta]
        """
        return self._pose
    
    @property
    def vel(self) -> List[int]:
        """
        Return the velocity of the robot as list of [l_vel, r_vel] in rad/sec
        """
        return self._vel
    
    @vel.setter
    def vel(self, vel: List[int]) -> None:
        """
        Set the velocity of the wheels. Input argument as list [l_vel, r_vel] in rad/sec
        """
        self._vel = vel

    def step(self, time_step: float = .05) -> None:
        """
        Iterate a small time step to calculate the new pose of the robot given the wheel velocities
        Input args: time_step: float
        """
        cur_x = self.pose[0]
        cur_y = self.pose[1]
        cur_theta = self.pose[2]

        dt = time_step

        # calculate linear velocities at each wheel
        linear_vel_l = self.vel[0] * self.wheel_diameter
        linear_vel_r = self.vel[1] * self.wheel_diameter

        # calculate the angular velocity
        omega = (linear_vel_r - linear_vel_l) / self.track_width

        # if robot is going in a straight line
        if omega == 0:
            
            # calculate new pose
            new_x = cur_x + linear_vel_l * dt * math.cos(cur_theta)
            new_y = cur_y + linear_vel_l * dt * math.sin(cur_theta)
            new_theta = cur_theta

            # set new pose
            self._pose = [new_x, new_y, new_theta]
        
        # if robot is not going straight, it is going along some arch 
        else:    
            
            # calculate new pose
            ICC_R = self.track_width/2 * (linear_vel_r + linear_vel_l)/(linear_vel_r - linear_vel_l)
            ICC_x = cur_x - ICC_R*math.sin(cur_theta)
            ICC_y = cur_y + ICC_R*math.cos(cur_theta)

            new_pos = np.array([[math.cos(omega*dt), -math.sin(omega*dt), 0],
                                [math.sin(omega*dt), math.cos(omega*dt), 0],
                                [0, 0, 1]]) @ \
                    np.array([[cur_x - ICC_x],
                                [cur_y - ICC_y],
                                [cur_theta]]) + \
                    np.array([[ICC_x],
                                [ICC_y],
                                [omega*dt]])
            
            # set new pose
            self._pose = new_pos.T[0].tolist()

    @property
    def camera_view(self) -> np.ndarray:
        """
        Get access to the camera view from the robot's current position
        """
        return self._camera.view
        

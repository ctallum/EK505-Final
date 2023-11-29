"""
Main file to run the simulation of the robot obstacle avoidance. Includes the Simulation class
"""

import math
from typing import List
from matplotlib import pyplot as plt

from robot import Robot
from world import World
from control import Control
from interface import Interface
from camera import Camera


class Simulation:
    """
    Class to initialize and run everything
    """
    threshold = 0.1
    max_steps = 1000  # set the maximum number of steps the robot can go to

    def __init__(self) -> None:
        """
        Initialize a robot, world, control, and a goal for the robot to reach
        """
        self.robot = Robot()
        self.world = World()
        self.control = Control()
        self.interface = Interface()
        self.camera = Camera(self.world)

        # set the goal point that the robot is trying to reach
        self.goal = [17, 6.25]

    def is_over(self, nb_steps: int) -> bool:
        """
        Determine if the robot is close enough to the goal for the simulation to end or if it has
        reached the maximum number of steps
        """
        dist_to_goal = math.sqrt((self.robot.pose[0] - self.goal[0])**2 +
                                 (self.robot.pose[1] - self.goal[1])**2)

        if dist_to_goal < self.threshold or nb_steps > self.max_steps:
            return True

        return False

    def get_control(self) -> List[int]:
        """
        Get control for robot given current pose of the robot and the world [TODO]
        """
        return self.control.control(self.robot.pose)

    def plot(self) -> None:
        """
        Plot the world and the robot
        """
        robot_x = self.robot.pose[0] * 200
        robot_y = self.robot.pose[1] * 200

        plt.plot(robot_x, robot_y, "ok")

    def force_end(self, event) -> None:
        """
        Create a function that allows the user to force end the simulation by closing the figure
        """
        exit()

    def run(self) -> None:
        """
        Run the simulation until either the robot reaches the end the number of steps expires
        """

        nb_steps = 0
        while not self.is_over(nb_steps):
            # get control for the robot
            control = self.get_control()

            # set the control for the robot
            self.robot.vel = control

            # iterate a single step
            self.robot.step()

            # plot everything
            self.interface.update(self.robot, self.world, self.camera)

            nb_steps += 1


if __name__ == "__main__":
    simulation = Simulation()
    simulation.run()

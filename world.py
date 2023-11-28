"""
File to contain the world class. Contains information on background and objects
"""

import os
import random


class World:
    """
    A class to contain all information about the world and things in the world
    """
    ground_image_dir = "./ground"
    obstacle_image_dir = "./obstacles"

    def __init__(self) -> None:
        """
        Initialize the world with a random floor and a random object placed in front of the robot.
        """

        # get all ground images from folder
        self._ground_images = [
            "ground/" + name for name in os.listdir(self.ground_image_dir)]

        # get all obstacles images from folder
        self._obstacle_images = [
            "obstacles/" + name for name in os.listdir(self.obstacle_image_dir)]

        self.generate_world()

    def list_images(self) -> None:
        """
        Used to list the list of images that the system can read
        """
        print(self._ground_images)
        print(self._obstacle_images)

    def generate_world(self) -> None:
        """
        Using available images, choose a random floor and random obstacle
        200px per ft
        """

        # get random floor
        ground_idx = random.randint(0, len(self._ground_images)-1)
        self._ground = self._ground_images[2]

        # print(floor_img)

    @property
    def ground(self) -> str:
        """
        Return the filename str of the ground image
        """
        return self._ground

"""
File to contain the world class. Contains information on background and objects
"""

import os
import random

import pygame


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

        # set the goal point that the robot is trying to reach
        self.goal = (10,10)
        
        # set position of the obstacle
        self.obstacle_pos = (5,5)

        # generate the world
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
        self.ground = pygame.image.load(self._ground_images[0])

        # position of floor
        self.ground_pos = (0,0)

        # get random piece of wood
        obstacle_idx = random.randint(0, len(self._obstacle_images) - 1)
        self.obstacle = pygame.image.load(self._obstacle_images[obstacle_idx])

        # set random scale of wood between 6in and 1ft
        obstacle_size = random.random()*100 + 100 
        obstacle_width = self.obstacle.get_width()
        scale_factor = obstacle_size/obstacle_width
        self.obstacle = pygame.transform.scale_by(self.obstacle, scale_factor)

        # position of obstacle
        obstacle_pos_x = self.obstacle_pos[0]
        obstacle_pos_y = self.obstacle_pos[1]

        self.obstacle_pos = (obstacle_pos_x *200 ,2500 - obstacle_pos_y*200)

        # rotate obstacle
        obstacle_rot = random.random()*360
        self.obstacle = pygame.transform.rotate(self.obstacle, obstacle_rot)
        
        def rot_center(image, angle, x, y):
            """
            Helper function to rotate object around the center coordinate
            """
            rotated_image = pygame.transform.rotate(image, angle - 90)
            new_rect = rotated_image.get_rect(center=image.get_rect(center=(x, y)).center)
            return rotated_image, new_rect

        self.obstacle, self.obstacle_pos = rot_center(self.obstacle, obstacle_rot, self.obstacle_pos[0], self.obstacle_pos[1])

        self.obstacle.set_colorkey((255,255,255))



        

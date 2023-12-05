"""
File to contain the world class. Contains information on background and objects
"""

import os
import random

import pygame
import convex_hull
import matplotlib.pyplot as plt
from typing import List, Tuple


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

        # get all ground and obstacle images from folder
        self.ground_images = ["ground/" + name for name in os.listdir(self.ground_image_dir)]
        self.obstacle_images = ["obstacles/" + name for name in os.listdir(self.obstacle_image_dir)]

        # set the goal point that the robot is trying to reach
        self.goal = (9.0,9.0)
        
        # set position of the obstacle
        self.obstacle_pos = [(5.0, 5.0), (7.0, 7.0)]

        self.obstacles: List[Obstacle] = []

        # generate the world
        self.generate_world()

    def list_images(self) -> None:
        """
        Used to list the list of images that the system can read
        """
        print(self.ground_images)
        print(self.obstacle_images)

    def generate_world(self) -> None:
        """
        Using available images, choose a random floor and random obstacle
        200px per ft
        """

        # get random floor
        ground_idx = random.randint(0, len(self.ground_images)-1)
        self.ground = pygame.image.load(self.ground_images[0])

        # position of floor
        self.ground_pos = (0,0)

        # generate n random obstacles
        for _,pos in enumerate(self.obstacle_pos):

            # get random piece of wood
            obstacle_idx = random.randint(0, len(self.obstacle_images) - 1)
            obstacle_img_path = self.obstacle_images[obstacle_idx]

            # create an obstacle object
            self.obstacles.append(Obstacle(obstacle_img_path, pos))


class Obstacle():
    """
    Class representing obstacles in the robot environment
    """
    def __init__(self, img_path: str, pos: Tuple[float]) -> None:
        """
        Initialize a obstacle using an image and the location
        """
        self.global_pos = pos
        self.surface, self.surf_pos = self.generate_surface(img_path, pos)

    def generate_surface(self, img_path: str, pos: Tuple[float]) -> Tuple[pygame.Surface, pygame.Rect]:
        """
        Function to generate the obstacle surface and position
        """
        # generate surface
        obstacle_surface = pygame.image.load(img_path)

        # set random scale of wood between 6in and 1ft
        obstacle_size = random.random()*50 + 150 
        obstacle_width = obstacle_surface.get_width()
        scale_factor = obstacle_size/obstacle_width
        obstacle_surface = pygame.transform.scale_by(obstacle_surface, scale_factor)

        # get position of surface
        obstacle_pos_x , obstacle_pos_y = pos
        obstacle_pos = (obstacle_pos_x *200 ,2500 - obstacle_pos_y*200)

        # rotate obstacle
        obstacle_rot = random.random()*360

        def rot_center(image, angle, x, y):
            """
            Helper function to rotate object around the center coordinate
            """
            rotated_image = pygame.transform.rotate(image, angle - 90)
            new_rect = rotated_image.get_rect(center=image.get_rect(center=(x, y)).center)
            return rotated_image, new_rect
        
        # get rotated obstacle around the center
        obstacle_surf, obstacle_surf_pos = rot_center(obstacle_surface, obstacle_rot, obstacle_pos[0], obstacle_pos[1])

        # set transparency
        obstacle_surf.set_colorkey((255,255,255))

        return obstacle_surf, obstacle_surf_pos 



class DetectedObstacle():
    """
    Class representing obstacles in the robots environment that have been detected by the robot
    """
    def __init__(self, pts):
        self.pts = pts
        # Compute convex hull of obstacle points
        self.c_hull = convex_hull.graham_scan(self.pts)

    def densify(self, step):
        """
        Increase density of points in obstacle convex hull 
        at increments 'step' along each edge
        """
        self.c_hull = convex_hull.densify(self.c_hull, step)

    def plot(self, obs_points = False, hull_pts = False, hull = True):
        """
        Plots obstacle
        """
        if obs_points:  # plot individual points
            plt.scatter(self.pts[0,:], self.pts[1,:])
        
        if hull_pts:    # plot hull points
            for elem in self.c_hull:
                plt.scatter(elem[0], elem[1])

        if hull:    # plot convex hull
            hull_x = []
            hull_y = []
            for elem in self.c_hull:
                hull_x.append(elem[0])
                hull_y.append(elem[1])
            plt.plot(hull_x, hull_y,'r-')

        

"""
File to contain the Camera class
"""

from typing import List
import math
import pygame

from world import World


class Camera:
    """
    class to generate a virtual camera. In this case, we are getting our virtual
    camera data from the pygame window
    """

    def __init__(self, world: World) -> None:
        """
        Initialize it's understanding of the world via pygame
        """
        self._world = world

        # setup internal surface for virtual camera tracking
        self._display = pygame.Surface((3570, 2500))
        ground = pygame.image.load(self._world.ground)
        self._display.blit(ground, (0, 0))

    def view(self, pose: List[int]) -> pygame.surface:
        """
        Take the robot's pose and generate a png image that represent's the virtual
        camera's view
        """
        display = self._display

        # set viewing range of camera (200px = 1ft)
        box = 400

        # get pose of the robot in pixels
        robot_x = pose[0] * 200
        robot_y = 2500 - pose[1] * 200
        theta = pose[2]

        # get the center of the camera box
        view_x_center = robot_x + (100 + box/2)*math.cos(theta)
        view_y_center = robot_y - (100 + box/2)*math.sin(theta)

        # crop the display to only the camera view
        sub_crop = display.subsurface((view_x_center - .75*box, view_y_center - .75*box,
                                       1.5*box, 1.5*box))

        # rotate the cropped section to account for the tilt of the robot
        sub_crop = pygame.transform.rotate(sub_crop, -theta*57.2958 + 90) # convert from rad to deg

        # crop image again to get final size
        shape = sub_crop.get_size()
        sub_sub_crop = sub_crop.subsurface((shape[0]/2 - box/2, shape[1]/2 - box/2, box, box))

        return sub_sub_crop

    def into_array(self) -> None:
        """
        Convert camera data from Pygame surface into a camera picture array (nxnx3)
        """

        pass

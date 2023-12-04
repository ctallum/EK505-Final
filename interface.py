"""
File to create a pygame window for plotting
"""

import math
import pygame
from pygame.locals import *
import sys

from world import World
from robot import Robot
from camera import Camera

black = (0, 0, 0)
red = (255, 0, 0)
white = (255, 255, 255)

class Interface:
    """
    Class to contain all the pygame stuff needed to display all our graphs
    """

    def __init__(self, robot: Robot, camera: Camera, world: World) -> None:
        """
        Initialize pygame setup
        """
        pygame.init()

        WINDOW_SIZE = (900, 600)

        self.screen = pygame.display.set_mode(WINDOW_SIZE, pygame.RESIZABLE)
        pygame.display.set_caption("EK505 Final")

        self.display = pygame.Surface((3750, 2500))

        # setup objects
        self.robot = robot
        self.camera = camera
        self.world = world


    def refresh_window(self) -> None:
        """
        Refresh and draw the screen
        """
        self.screen.fill(black)
        new_window_size, center_cords = self.adjust_scale()
        # scale internal display to match window)

        new_disp = pygame.transform.scale(self.display, new_window_size)
        self.screen.blit(new_disp, center_cords)

        pygame.display.update()

    def adjust_scale(self) -> None:
        """
        Adjust internal screen for window scaling

        If the window size is changed, scale the game to the maximum amount while keeping the same
        aspect ratio. Also keep the game centered in the window.

        Returns:
            display_size::tuple (height, width)
                The updated height and width of the internal game display
            cords::tuple (x_cord, y_cord)
                The coordinates of the upper left corner of the internal game display so that when
                it is blit onto window, it is centered.
        """
        window_size = self.screen.get_size()

        # if window is longer than aspect ratio
        if window_size[0] / window_size[1] >= 1.5:
            display_size = (int(1.5 * window_size[1]), window_size[1])

        # if window is taller than aspect ratio
        else:
            display_size = (window_size[0], int(.75 * window_size[0]))

        # find cords so that display is centered
        cords = ((window_size[0] - display_size[0]) / 2,
                 (window_size[1] - display_size[1]) / 2)

        return display_size, cords

    def plot_world(self) -> None:
        """
        Plots the world
        """
        self.display.blit(self.world.ground, self.world.ground_pos)

        self.display.blit(self.world.obstacle, self.world.obstacle_pos)



    def plot_robot(self) -> None:
        """
        Plot's the robot
        """

        # get robot image
        robot_img = pygame.image.load("robot.png")

        # convert coordinates and change size
        loc_x = self.robot.pose[0] * 200
        loc_y = 2500 - self.robot.pose[1] * 200
        robot_img = pygame.transform.scale(robot_img, (200, 200))

        # convert robot theta to degrees
        robot_angle_deg = self.robot.pose[2] * 57.2958

        # rotate robot around the center

        def rot_center(image, angle, x, y):
            """
            Helper function to rotate object around the center coordinate
            """
            rotated_image = pygame.transform.rotate(image, angle - 90)
            new_rect = rotated_image.get_rect(center=image.get_rect(center=(x, y)).center)
            return rotated_image, new_rect

        robot_img, new_center = rot_center(robot_img, robot_angle_deg, loc_x, loc_y)

        # set robot background so that it's clear
        robot_img.set_colorkey((102, 102, 157))

        # display robot
        self.display.blit(robot_img, (new_center))

    def plot_camera_box(self) -> None:
        """
        Plot a box in front of the robot that reflects what the robot can see
        """
        theta = self.robot.pose[2]
        loc_x = self.robot.pose[0] * 200 + 1 + 100*math.cos(theta)
        loc_y = 2500 - self.robot.pose[1] * 200 - 100*math.sin(theta)

        box = 400

        pygame.draw.line(self.display, red, (loc_x + box/2*math.sin(theta),
                                             loc_y + box/2*math.cos(theta)),
                                            (loc_x - box/2*math.sin(theta),
                                             loc_y - box/2*math.cos(theta)),
                                             width=10)

        pygame.draw.line(self.display, red, (loc_x - box/2*math.sin(theta),
                                             loc_y - box/2*math.cos(theta)),
                                            (loc_x - box/2*math.sin(theta) + box*math.cos(theta),
                                             loc_y - box/2*math.cos(theta) - box*math.sin(theta)),
                                             width=10)

        pygame.draw.line(self.display, red, (loc_x + box/2*math.sin(theta),
                                             loc_y + box/2*math.cos(theta)),
                                            (loc_x + box/2*math.sin(theta) + box*math.cos(theta),
                                             loc_y + box/2*math.cos(theta) - box*math.sin(theta)),
                                             width=10)

        pygame.draw.line(self.display, red, (loc_x + box/2*math.sin(theta) + box*math.cos(theta),
                                             loc_y + box/2*math.cos(theta) - box*math.sin(theta)),
                                            (loc_x - box/2*math.sin(theta) + box*math.cos(theta),
                                             loc_y - box/2*math.cos(theta) - box*math.sin(theta)),
                                             width=10)

    def plot_camera(self) -> None:
        """
        Plot the local view from the camera
        """

        camera_size = 1250

        # get view from camera
        camera_view = self.camera.view(self.robot.pose)

        # scale the camera view up
        camera_view = pygame.transform.scale(camera_view, (camera_size, camera_size))

        # draw camera view in the top right corner
        self.display.blit(camera_view, (3750-camera_size, 0))

        # draw black border around camera view
        pygame.draw.line(self.display, black, (3750-camera_size-5,camera_size),
                        (3750, camera_size), width=20)
        pygame.draw.line(self.display, black, (3750-camera_size, camera_size + 5),
                        (3750-camera_size, 0), width=20)


    def plot_goal(self) -> None:
        """
        Plot the goal
        """
        goal_x, goal_y = self.world.goal
        goal_loc = (goal_x * 200, 2500 - goal_y*200)
        pygame.draw.circle(self.display, (0,255,0), goal_loc, radius=20)

    def update(self) -> None:
        """
        Update the pygame window with all new movement
        """

        # check to see if simulation has been canceled
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit()

        # plot everything
        self.plot_world()
        self.plot_robot()
        self.plot_camera()
        self.plot_camera_box()
        self.plot_goal()
        self.refresh_window()


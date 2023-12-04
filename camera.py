"""
File to contain the Camera class
"""

import numpy as np
from typing import List
import math
import pygame
from matplotlib import pyplot as plt
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry
import cv2
import supervision as sv
import time
import imutils

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
        self.world = world
        self.box_view = 400

        # setup internal surface for virtual camera tracking
        self._display = pygame.Surface((3570, 2500))
        ground = self.world.ground
        obstacle = self.world.obstacle
        self._display.blit(ground, self.world.ground_pos)
        self._display.blit(obstacle, self.world.obstacle_pos)

        # setup surface for camera view
        self.camera_view = pygame.Surface((self.box_view, self.box_view))

        self.detects_obstacles = False
        self.obstacle_loc = None
        self.obstacle_center = None

        # setup some SAM stuff
        # self.SAM = sam_model_registry["vit_h"](checkpoint="sam_vit_h_4b8939.pth")
        # self.mask_generator = SamAutomaticMaskGenerator(self.SAM)

    def get_raw_view(self, pose: List[int]):
        """
        Take the robot's pose and generate a png image that represent's the virtual
        camera's view
        """
        display = self._display

        # set viewing range of camera (200px = 1ft)


        # get pose of the robot in pixels
        robot_x = pose[0] * 200
        robot_y = 2500 - pose[1] * 200
        theta = pose[2]

        # get the center of the camera box
        view_x_center = robot_x + (100 + self.box_view/2)*math.cos(theta)
        view_y_center = robot_y - (100 + self.box_view/2)*math.sin(theta)

        # crop the display to only the camera view
        sub_crop = display.subsurface((view_x_center - .75*self.box_view, view_y_center - .75*self.box_view,
                                       1.5*self.box_view, 1.5*self.box_view))

        # rotate the cropped section to account for the tilt of the robot
        sub_crop = pygame.transform.rotate(sub_crop, -theta*57.2958 + 90) # convert from rad to deg

        # crop image again to get final size
        shape = sub_crop.get_size()
        sub_sub_crop = sub_crop.subsurface((shape[0]/2 - self.box_view/2, shape[1]/2 - self.box_view/2, self.box_view, self.box_view))

        return sub_sub_crop

    def get_unprocessed_view(self, pose: List[int]):
        self.camera_view = self.get_raw_view(pose)

    def get_processed_view(self, pose: List[int]):
        surface = self.get_raw_view(pose)
        array = self.into_array(surface)
        masked_image = self.into_color_mask(array)
        self.into_surface(masked_image)
        self.detect_obstacles(masked_image)


    def detect_obstacles(self, array: np.ndarray) -> None:
        lower = np.array([80,80,80])
        upper = np.array([180,180,180])

        

        def find_color(frame, lower, upper):
            mask = cv2.inRange(frame, lower, upper)#create mask with boundaries 
            mask = ~mask

            plt.imshow(mask)
            cnts = cv2.findContours(mask, cv2.RETR_TREE, 
                                cv2.CHAIN_APPROX_SIMPLE) # find contours from mask
            cnts = imutils.grab_contours(cnts)
            for c in cnts:
                area = cv2.contourArea(c) # find how big countour is
                # print(area)
                if 20000 >area > 1000:       # only if countour is big enough, then
                    M = cv2.moments(c)
                    cx = int(M['m10'] / M['m00']) # calculate X position
                    cy = int(M['m01'] / M['m00']) # calculate Y position
                    return c, cx, cy

        if find_color(array, lower, upper):
            c, cx, cy = find_color(array, lower, upper)
            c = np.reshape(c,(-1,2) )
            
            self.obstacle_loc = c
            self.obstacle_center = (cx,cy)

            self.detects_obstacles = True
        else: 
            self.detects_obstacles = False




    def into_array(self, surface) -> np.ndarray:
        """
        Convert camera data from Pygame surface into a camera picture array (nxnx3)
        """
        array = pygame.surfarray.pixels3d(surface)
 
        return array
        


    def into_sam(self) -> None:
        start = time.time()

        print("starting mask")
        output_mask = self.mask_generator.generate(self.camera_view_array)
        print("done mask")
        # print(output_mask)
        end = time.time()
        print(end - start)


        mask_annotator = sv.MaskAnnotator(color_map = "index")
        detections = sv.Detections.from_sam(output_mask)
        annotated_image = mask_annotator.annotate(self.camera_view_array, detections)

        self.annotated_image = annotated_image

        # sv.plot_images_grid(
        #     images=[self.camera_view_array, annotated_image],
        #     grid_size=(1, 2),
        #     titles=['source image', 'segmented image']
        # )

    def into_color_mask(self, img) -> None:

        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)

        
        twoDimage = img.reshape((-1,3))
        twoDimage = np.float32(twoDimage)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 2
        attempts=10

        ret,label,center=cv2.kmeans(twoDimage,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]
        result_image = res.reshape((img.shape))

        return result_image

    def into_surface(self, array) -> None:
        self.camera_view = pygame.surfarray.make_surface(array)


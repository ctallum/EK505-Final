"""
File to contain the control class. This is used to control the robot
"""

from typing import List


class Control:
    """
    Class to control the robot depending on it's pose and what it can see
    """
    @staticmethod
    def control(pose: List[int]) -> List[int]:
        """
        Give controls to the robot. Controls take form of left and right wheel velocities in rad/sec
        Inputs:
            pose: List[x_pos, y_pos, theta]
            world: World class containing location of obstacles
        Return:
            vel: List[left_w, right_w]
        """

        return [0, 0]

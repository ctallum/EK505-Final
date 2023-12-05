"""
File to contain the control class. This is used to control the robot
"""
import cvxopt as cvx
import convex_hull
import matplotlib.pyplot as plt
import numpy as np
cvx.solvers.options['show_progress'] = False
import math

from typing import List

from robot import Robot
from world import World, DetectedObstacle


class Control:
    """
    Class to control the robot depending on it's pose and what it can see
    """
    def __init__(self, robot: Robot, world: World) -> None:
        self.robot = robot
        self.world = world
        self.max_lin_vel = 1
        self.max_angular_vel = 8
        self.prior_obstacles_detected: List[DetectedObstacle]= []
    
    def distance(self, x_eval: np.ndarray, x_hull: np.ndarray) -> float:
        """
        Computes Euclidean distance between point on an obstacle 
        hull and the edge of the robot
        """
        return np.linalg.norm(x_eval - x_hull) - self.robot.radius

    def grad_distance(self, x_eval: np.ndarray, x_hull: np.ndarray) -> float:
        """
        Computes gradient of Euclidean distance between point on obstacle 
        """
        # print("shape",x_hull)
        return (x_eval - x_hull)/np.linalg.norm(x_eval - x_hull)

    def reference(self, x_eval: np.ndarray, p: float) -> np.ndarray:
        """
        Computes references controller for goal location, grad u_attr
        """
        x_goal = np.array(self.world.goal)

        grad_u_attr = p * np.linalg.norm(x_eval - x_goal)**(p-2) * (x_eval - x_goal)
        return - grad_u_attr

    def control(self, step: float=0.2) -> np.ndarray:
        """
        Computes optimal (CBF/CLF) control based on current position of 
        robot (x_eval), returns set of wheel velocities that point robot into
        the direction of u_optimal
        """
        c_h = self.robot.rep_weight # CBF parameter
        x_eval = self.robot.pose[0:2]

        # Compute reference control
        u_ref = self.reference(x_eval, 1)

        # Concatenate all obstacle convex hull points into a single list
        all_hull_pts = []

        if self.robot.detects_obstacles or len(self.prior_obstacles_detected) > 0:
            if self.robot.detects_obstacles:
                obs_pts = np.unique(self.robot.obstacle_loc.T, axis = 1)
                obs = DetectedObstacle(obs_pts)
                obs.densify(step)
                self.prior_obstacles_detected.append(obs)
            else:
                obs = self.prior_obstacles_detected[-1]

            for pt in obs.c_hull:
                all_hull_pts.append(pt)
            tot_hull_pts = len(all_hull_pts)

            a_barrier = np.zeros((tot_hull_pts, 2))
            b_barrier = np.zeros((tot_hull_pts, 1))

            for idx, pt in enumerate(all_hull_pts):
                # pt = pt.reshape((2,1))
                h = self.distance(x_eval, pt)
                # print("x_eval", x_eval, "pt", pt)
                grad_h = self.grad_distance(x_eval, pt)
                a_barrier[idx,:] = - np.transpose(grad_h)
                b_barrier[idx,0] = - c_h * h

            u_opt = qp_supervisor(a_barrier, b_barrier, u_ref)
        else:
            u_opt = u_ref

        # calculate wheel vel
        u_opt_angle = math.atan2(u_opt[1], u_opt[0])

        angular_dif = self.robot.pose[2] - u_opt_angle

        def ddr_ik(v_x: float, omega: float, L=self.robot.track_width, r=self.robot.wheel_diameter):
            #DDR inverse kinematics: calculate wheels speeds from desired velocity
            return [(v_x - (L/2)*omega)/r, (v_x + (L/2)*omega)/r]
        
        # map angular diff to angular_ve
        omega = -2*self.max_angular_vel/math.pi*angular_dif

        return ddr_ik(self.max_lin_vel, omega)

    

def qp_supervisor(a_barrier: np.ndarray, b_barrier:np.ndarray, u_ref: np.ndarray=None, solver='cvxopt') -> np.ndarray:
    """
    Solves the QP min_u ||u-u_ref||^2 subject to a_barrier*u+b_barrier<=0
    For the list of supported solvers, see https://pypi.org/project/qpsolvers/
    """
    dim = 2
    if u_ref is None:
        u_ref = np.zeros((dim, 1))
    p_qp = cvx.matrix(np.eye(2))
    q_qp = cvx.matrix(-u_ref)
    if a_barrier is None:
        g_qp = None
    else:
        g_qp = cvx.matrix(np.double(a_barrier))
    if b_barrier is None:
        h_qp = None
    else:
        h_qp = -cvx.matrix(np.double(b_barrier))
    solution = cvx.solvers.qp(p_qp, q_qp, G=g_qp, h=h_qp, solver=solver)
    return np.array(solution['x'])



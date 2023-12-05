"""
File to contain the control class. This is used to control the robot
"""
import cvxopt as cvx
import convex_hull
import matplotlib.pyplot as plt
import numpy as np
cvx.solvers.options['show_progress'] = False


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
    
    def distance(self, x_eval, x_hull):
        """
        Computes Euclidean distance between point on an obstacle 
        hull and the edge of the robot
        """
        return np.linalg.norm(x_eval - x_hull) - self.robot.radius

    def grad_distance(self, x_eval, x_hull):
        """
        Computes gradient of Euclidean distance between point on obstacle 
        """
        # print("shape",x_hull)
        return (x_eval - x_hull)/np.linalg.norm(x_eval - x_hull)

    def reference(self, x_eval, p):
        """
        Computes references controller for goal location, grad u_attr
        """
        x_goal = np.array(self.world.goal)

        grad_u_attr = p * np.linalg.norm(x_eval - x_goal)**(p-2) * (x_eval - x_goal)
        return - grad_u_attr

    def control(self, step=0.2):
        """
        Computes optimal (CBF/CLF) control based on current position of 
        robot (x_eval)
        """
        c_h = self.robot.rep_weight # CBF parameter
        x_eval = self.robot.pose[0:2]

        # Compute reference control
        u_ref = self.reference(x_eval, 1)

        # Concatenate all obstacle convex hull points
        # into a single list
        all_hull_pts = []

        if self.robot.detects_obstacles:
            
            obs_pts = np.unique(self.robot.obstacle_loc.T, axis = 1)
            obs = DetectedObstacle(obs_pts)

            obs.densify(step)
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

            return u_opt

        else:
            return u_ref
    

def qp_supervisor(a_barrier, b_barrier, u_ref=None, solver='cvxopt'):
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



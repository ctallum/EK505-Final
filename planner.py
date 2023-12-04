"""
Algorithm for CLF-CBF-QP planner for obstacle avoidance
"""
import cvxopt as cvx
import convex_hull
import matplotlib.pyplot as plt
import numpy as np

cvx.solvers.options['show_progress'] = False

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

class Obstacle():
    """
    Class representing obstacles in the robots environment
    """
    def __init__(self, pts):
        self.pts = pts
        # Compute convex hull of obstacle points
        self.c_hull = convex_hull.graham_scan(self.pts)

    def plot(self, points = False, hull = True):
        """
        Plots obstacle
        """
        if points:  # plot individual points
            plt.scatter(self.pts[0,:], self.pts[1,:])

        if hull:    # plot convex hull
            hull_x = []
            hull_y = []
            for elem in self.c_hull:
                hull_x.append(elem[0])
                hull_y.append(elem[1])
            plt.plot(hull_x, hull_y,'r-')\

class World():
    """
    Class representing the robot and its surroundings
    """
    def __init__(self, goal, obstacle) -> None:
        self.obstacles = [obstacle]
        self.goal = goal

    def add_obstacle(self, obstacle):
        """
        Add obstacle, represented by pts
        """
        self.obstacles.append(obstacle)

    def plot(self):
        """
        Plots world with obstacles
        """
        for obs in self.obstacles:
            obs.plot()
        plt.scatter(self.goal[0], self.goal[1], color='g', marker='*')

class Robot():
    """
    Class representing the robot 
    """
    def __init__(self, pose, radius, rep_weight=3) -> None:
        self.center = pose   #[x, y]
        self.radius = radius
        self.rep_weight = rep_weight

    def plot(self):
        """
        Plots robot
        """
        plt.scatter(self.center[0], self.center[1])
        theta = np.linspace(0,2*np.pi, 100)
        x = self.center[0] + self.radius * np.cos(theta)
        y = self.center[1] + self.radius * np.sin(theta)
        plt.plot(x, y)

class ClfCbfControl:
    """
    Class for implementation of CLF-CBF-QP planner 
    """
    def __init__(self, world, robot) -> None:
        self.world = world
        self.robot = robot

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
        return (x_eval - x_hull)/np.linalg.norm(x_eval - x_hull)

    def reference(self, x_eval, p):
        """
        Computes references controller for goal location, grad u_attr
        """
        x_goal = self.world.goal
        grad_u_attr = p * np.linalg.norm(x_eval - x_goal)**(p-2) * (x_eval - x_goal)
        return - grad_u_attr

    def control(self, x_eval):
        """
        Computes optimal (CBF/CLF) control based on current position of 
        robot (x_eval)
        """
        c_h = self.robot.rep_weight # CBF parameter

        # Concatenate all obstacle convex hull points
        # into a single list
        all_hull_pts = []
        for obs in self.world.obstacles:
            for pt in obs.c_hull:
                all_hull_pts.append(pt)
        tot_hull_pts = len(all_hull_pts)

        # Compute reference control
        u_ref = self.reference(x_eval, 1)

        a_barrier = np.zeros((tot_hull_pts, 2))
        b_barrier = np.zeros((tot_hull_pts, 1))

        for idx, pt in enumerate(all_hull_pts):
            pt = pt.reshape((2,1))
            h = self.distance(x_eval, pt)
            grad_h = self.grad_distance(x_eval, pt)
            a_barrier[idx,:] = - np.transpose(grad_h)
            b_barrier[idx,0] = - c_h * h

        u_opt = qp_supervisor(a_barrier, b_barrier, u_ref)
        return u_opt

def planner_test():
    """
    Function for testing CLF-CBF-QP planner
    """
    pts = convex_hull.rand_2d(100,3,4,3,4)
    obs = Obstacle(pts)

    goal = np.array([[10],[10]])
    world = World(goal, obs)

    pose = np.array([[1],[1]])
    robot = Robot(pose, radius=0.5, rep_weight=2)

    epsilon = 0.5
    while np.linalg.norm(robot.center - goal) >= 0.5:
        controller = ClfCbfControl(world, robot)
        u = controller.control(robot.center)
        robot.center = robot.center + epsilon * u
        world.plot()
        robot.plot()
        plt.show()


if __name__ == '__main__':
    planner_test()

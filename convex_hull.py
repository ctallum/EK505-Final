"""
Implementation of convex hull algorithm for finite planar set 
"""

import numpy as np
import matplotlib.pyplot as plt

def rand_2d(n_pts, xmin, xmax, ymin, ymax):
    """
    Generates n random 2d points in the range [xmin, xmax], [ymin, ymax]
    """
    x = np.random.uniform(xmin, xmax, n_pts)
    y = np.random.uniform(ymin, ymax, n_pts)
    points = np.vstack((x,y))

    return points

def sort_cos(pts):
    """
    Sorts an array of points in increasing order of the polar angle) formed between the line 
    from each point to the point p0 (with the lowest y value) and the x axis. 
    """
    # Point with the lowest y value
    p0 = pts[:, np.argmin(pts[1,:])].reshape((2,1))

    nb_pts = pts.shape[1]
    pt_cos_list = []    # list of point, cosine pairs
    
    # Compute polar angle of each point
    for idx in range(nb_pts):
        test_pt = pts[:,idx].reshape((2,1))
        if test_pt[1] == p0[1]:
            nb_pts -= 1
            continue
        else:
            dx = test_pt[0] - p0[0]
            dy = test_pt[1] - p0[1]
            cos = float( dx / np.sqrt(dx**2 + dy**2))
            pt_cos_list.append( (idx, cos) )

    cos_list_sorted = sorted(pt_cos_list, key=lambda x: x[1], reverse=True)

    # Sort points in the array by cosine value
    pts_sorted = np.zeros((2, nb_pts))
    pts_sorted[:,0] = p0.reshape((2,))
    for idx in range(nb_pts -1):
        pt_idx = cos_list_sorted[idx][0]        # point's index from original array
        pts_sorted[:,idx+1] = pts[:, pt_idx]

    return pts_sorted

def orientation(p1, p2, p3):
    """
    Returns 1 if the points make a counterclockwise turn, -1 for a
    clockwise turn, and 0 if they are collinear.
    """
    # Compute the z coordinate of the cross product p1p2 x p2p3
    z = (p2[0]-p1[0]) * (p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0])
    if z > 0:
        return 1
    if z < 0:
        return -1
    if z == 0:
        return 0

def graham_scan(pts):
    """
    Implementation of the Graham scan algorithm for finding the convex hull 
    of a set of points
    """
    # Stack for storing convex hull
    conv_hull = []

    # Sort points by polar angle
    pts_sorted = sort_cos(pts)
    nb_points = pts_sorted.shape[1]

    conv_hull.append(pts_sorted[:,0])
    conv_hull.append(pts_sorted[:,1])

    for idx in range(nb_points):
        pt = pts_sorted[:,idx]
        while len(conv_hull) > 1 and orientation(conv_hull[-2], conv_hull[-1], pt) <= 0:
            conv_hull.pop()
        conv_hull.append(pt)
    conv_hull.append(pts_sorted[:,0])

    return conv_hull

def densify(convex, step):
    """
    Function that adds points along each edge of a convex hull
    at an increment equal to 'step'
    """
    new_hull = []
    for idx, pt in enumerate(convex):
        new_hull.append(pt)
        if idx < len(convex) - 1:
            pt2 = convex[idx+1]
        else:
            pt = convex[-1]
            pt2 = convex[0]
        if (np.linalg.norm(pt2 - pt)) == 0:
            continue
        direction = (pt2 - pt) / np.linalg.norm(pt2 - pt)
        pt_new = pt + direction * step
        while np.linalg.norm(pt2-pt_new) > step:
            new_hull.append(pt_new.copy())
            pt_new += direction * step
    return new_hull

if __name__ == '__main__':
    test_pts = rand_2d(100, -10, 4, -4, 6)
    c_hull = graham_scan(test_pts)
    plt.scatter(test_pts[0,:], test_pts[1,:])
    test_pts_sorted = sort_cos(test_pts)
    hull_x = []
    hull_y = []
    for elem in c_hull:
        hull_x.append(elem[0])
        hull_y.append(elem[1])
    plt.scatter(hull_x, hull_y, color='r')
    plt.plot(hull_x, hull_y,'r-')
    hull_2 = densify(c_hull, 1)
    for val in hull_2:
        plt.scatter(val[0], val[1], color='g', marker='*')
    plt.axis('equal')
    plt.show()

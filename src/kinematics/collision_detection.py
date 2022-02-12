import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d import axes3d
import sys
import kinpy as kp
from rrtnode import RRTNode
from mpl_toolkits.mplot3d import Axes3D

"""****************************************************************************
A script to configure a robot arm and collisions (represented as cubes).
Run the script to see an example visualization of the arm and an obstruction.
Run the script with argument "rand" to see a random configuration and test
the collision detection (result of collision is printed through command line).
TODO: Implement path-planning algo | Integrate with camera's point cloud output
****************************************************************************"""

# Global arm configuration
# chain = kp.build_chain_from_urdf(open("models/SimpleArmModelforURDF.urdf").read())
global ax
global arm


def plot_linear_cube(ax, cube):
    """
    Plot a cube on an instance of matplotlib.axes.

    INSTANCE ARGUMENTS:
    ax   [matplotlib.axes] : figure with 3D axes.
    cube [list]            : specifications of the cube formatted as
                             [<x coord.>, <y coord.>, <z coord.>, <side length>].
                             [<x coord.>, <y coord.>, <z coord.>, <length.>, <width.>, <height.>].

    """
    x, y, z, l = cube
    # x, y, z, l, w, h = prism
    xx = [x, x, x + l, x + l, x]
    yy = [y, y + l, y + l, y, y]

    kwargs = {'alpha': 1, 'color': 'blue'}
    ax.plot3D(xx, yy, [z] * 5, **kwargs)
    ax.plot3D(xx, yy, [z + l] * 5, **kwargs)
    ax.plot3D([x, x], [y, y], [z, z + l], **kwargs)
    ax.plot3D([x, x], [y + l, y + l], [z, z + l], **kwargs)
    ax.plot3D([x + l, x + l], [y + l, y + l], [z, z + l], **kwargs)
    ax.plot3D([x + l, x + l], [y, y], [z, z + l], **kwargs)


def plot_linear_prism(ax, prism):
    """
        Plot a prism on an instance of matplotlib.axes.

        INSTANCE ARGUMENTS:
        ax   [matplotlib.axes] : figure with 3D axes.
        prism [list]            : specifications of the cube formatted as
                                 [<x coord.>, <y coord.>, <z coord.>, <length.>, <width.>, <height.>].

        """
    # x, y, z, l = cube
    x, y, z, l, w, h = prism
    # Each item in the arrays "xx" and "yy" represent either an x or y coordinate
    # For example: (x, y+l), (x, y), (x+w, y), ... etc
    # The order of the coordinates determine which dot is plotted first, and the lines
    # between each are drawn in the same order.
    # Also, the l and w are used as the area of the base of the prism, with the length
    # moving in the positive y direction, width in the positive x, and height is z obv.
    xx = [x, x, x + w, x + w, x]
    yy = [y + l, y, y, y + l, y + l]

    kwargs = {'alpha': 1, 'color': 'blue'}
    ax.plot3D(xx, yy, [z] * 5, **kwargs)
    ax.plot3D(xx, yy, [z + h] * 5, **kwargs)
    ax.plot3D([x, x], [y, y], [z, z + h], **kwargs)
    ax.plot3D([x, x], [y + l, y + l], [z, z + h], **kwargs)
    ax.plot3D([x + w, x + w], [y + l, y + l], [z, z + h], **kwargs)
    ax.plot3D([x + w, x + w], [y, y], [z, z + h], **kwargs)


def plot_points(ax, xs, ys, zs):
    """
    Plot points on an instance of matplotlib.axes.

    INSTANCE ARGUMENTS:
    ax [matplotlib.axes] : figure with 3D axes.
    xs [list]            : x coordinates.
    ys [list]            : y coordinates.
    zs [list]            : z coordinates.

    PRECONDITIONS:
    len(xs) == len(ys) == len(zs)
    """
    ax.scatter(xs, ys, zs)


def plot_lines(ax, xs, ys, zs, line_label):
    """
    Plot a line on an instance of matplotlib.axes.
    Use coordinates as points connecting the line.

    INSTANCE ARGUMENTS:
    ax         [matplotlib.axes] : figure with 3D axes.
    xs         [list]            : x coordinates.
    ys         [list]            : y coordinates.
    zs         [list]            : z coordinates.
    line_label [str]             : label for the line.
    """
    ax.plot(xs, ys, zs, label=line_label)


def plot_arm(ax, node, line_label):
    """
    Plot a robot arm on an instance of matplotlib.axes.

    INSTANCE ARGUMENTS:
    ax         [matplotlib.axes] : figure with 3D axes.
    node        [RRTNode]        : robot arm configuration.
    line_label [str]             : label for the arm plot.
    """
    points = node.joint_positions
    for p in points:
        print("point", p[0], " ", p[1], " ", p[2])
    plot_points(ax, [p[0] for p in points],
                [p[1] for p in points], [p[2] for p in points])
    plot_lines(ax, [p[0] for p in points],
               [p[1] for p in points], [p[2] for p in points], line_label)


def line_is_colliding(line_seg, cube):
    """
    Return True if the line segment is colliding with the cube, else False.
    Check if the line is within the sphere that is centered at the cube's
    center and has a diameter equal to the cube's diagonal.

    INSTANCE ARGUMENTS:
    line_seg [2D list] : line coordinates formatted as
                         [[<point 1 coordinates>], [<point 2 coordinates>]]
    cube     [list]    : specifications of the cube formatted as
                         [<x coord.>, <y coord.>, <z coord.>, <side length>].
    """
    a_vec = np.array([line_seg[0][0], line_seg[0][1], line_seg[0][2]])
    b_vec = np.array([line_seg[1][0], line_seg[1][1], line_seg[1][2]])
    side = cube[3]
    c_vec = np.array([cube[0], cube[1], cube[2]]) + side / 2
    r = np.sqrt(3) * side / 2
    line_vec = b_vec - a_vec
    line_mag = np.linalg.norm(line_vec)
    circle_vec = c_vec - a_vec
    proj = circle_vec.dot(line_vec / line_mag)
    if proj <= 0:
        closest_point = a_vec
    elif proj >= line_mag:
        closest_point = b_vec
    else:
        closest_point = a_vec + line_vec / line_mag * proj
    if np.linalg.norm(closest_point - c_vec) > r:
        return False

    return True


"""
#def line_is_colliding_prism(line_seg, prism):
    
    Return True if the line segment is colliding with the cube, else False.
    Check if the line is within the sphere that is centered at the cube's
    center and has a diameter equal to the cube's diagonal.

    INSTANCE ARGUMENTS:
    line_seg [2D list] : line coordinates formatted as
                         [[<point 1 coordinates>], [<point 2 coordinates>]]
    prism     [list]    : specifications of the cube formatted as
                         [<x coord.>, <y coord.>, <z coord.>, <length.>, <width.>, <height.>].
    
    a_vec = np.array([line_seg[0][0], line_seg[0][1], line_seg[0][2]])
    b_vec = np.array([line_seg[1][0], line_seg[1][1], line_seg[1][2]])
    side = cube[3]
    c_vec = np.array([cube[0], cube[1], cube[2]]) + side/2
    r = np.sqrt(3) * side / 2
    line_vec = b_vec - a_vec
    line_mag = np.linalg.norm(line_vec)
    circle_vec = c_vec - a_vec
    proj = circle_vec.dot(line_vec / line_mag)
    if proj <= 0:
        closest_point = a_vec
    elif proj >= line_mag:
        closest_point = b_vec
    else:
        closest_point = a_vec + line_vec / line_mag * proj
    if np.linalg.norm(closest_point - c_vec) > r:
        return False

    return True
"""


def arm_is_colliding(node, cube):
    """
    Return True if the arm is colliding with the cube, else False.

    INSTANCE ARGUMENTS:
    node  [RRTNode] : Instance of RRTNode representing a robot arm configuration.
    cube [list]     : specifications of the cube formatted as
                      [<x coord.>, <y coord.>, <z coord.>, <side length>].
    """
    points = node.joint_positions
    for i in range(len(node.angles) - 1):
        line = [points[i], points[i + 1]]
        if line_is_colliding(line, cube):
            return True
    return False


def arm_is_colliding_prism(node, prism):
    """
    Return True if the arm is colliding with the cube, else False.

    INSTANCE ARGUMENTS:
    node  [RRTNode] : Instance of RRTNode representing a robot arm configuration.
    cube [list]     : specifications of the cube formatted as
                      [<x coord.>, <y coord.>, <z coord.>, <side length>].
    """
    points = node.joint_positions
    print("points follow: ")
    print(points)
    for i in range(len(node.angles) - 1):
        line = [points[i], points[i + 1]]
        if line_is_colliding(line, prism):
            return True
    return False


def main():
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    # Set the limits for the range of plotted values
    lim = .6
    plt.xlim(-lim, lim)
    plt.ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    # create the prism to be avoided (hard coded)
    prism = [0, .1, 0, .1, .2, .4]
    plot_linear_prism(ax, prism)
    # create the arm and report collision (random)
    arm = RRTNode(None)
    collision_detected = arm_is_colliding_prism(arm, prism)
    print("Arm is colliding:", collision_detected)
    plot_arm(ax, arm, "arm")
    if collision_detected:
        plt.title("Collision Detected")
    else:
        plt.title("No Collision Detected")

    class Index:
        def draw(self, event):
            plt.close()
            main()

    axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
    run = Button(axnext, 'ReRun')
    callback = Index()
    run.on_clicked(callback.draw)
    plt.legend()

    plt.show()

def newLineCollider(line_seg, prism):
        """
        Return True if the line segment is colliding with the prism, else False.

        INSTANCE ARGUMENTS:
        line_seg [2D list] : line coordinates formatted as
                             line_seg[0] = p1x, line_seg[1] = p1y, line_seg[2] = p1z.
                             line_seg[3] = p2x, line_seg[4] = p2y, line_seg[5] = p2z.

        prism     [list]    : specifications of the cube formatted as
                             [<x coord.>, <y coord.>, <z coord.>, <length.>, <width.>, <height.>].

        x = x1 + t(x2-x1)
        y = y1 + t(y2-y2)
        z = z1 + t(z2-z1)

        t = (x-x1) / (x2-x1)

        """
        collision = False
        checkX = False
        checkY = False
        checkZ = False
        # First check if either point is within the prism:
        if (prism[0] <= line_seg[0]) & (line_seg[0] <= prism[0] + prism[4]):
            checkX = True
        if (prism[1] <= line_seg[1]) & (line_seg[1] <= prism[1] + prism[3]):
            checkY= True
        if (prism[2] <= line_seg[2]) & (line_seg[2] <= prism[2] + prism[5]):
            checkZ = True
        if(checkX & checkZ & checkY):
            return True
        checkX = False
        checkY = False
        checkZ = False
        if (prism[0] <= line_seg[3]) & (line_seg[3] <= prism[0] + prism[4]):
            checkX = True
        if (prism[1] <= line_seg[4]) & (line_seg[4] <= prism[1] + prism[3]):
            checkY = True
        if (prism[2] <= line_seg[5]) & (line_seg[5] <= prism[2] + prism[5]):
            checkZ = True
        if(checkX & checkZ & checkY):
            return True

        checkX = False
        checkY = False
        checkZ = False

        # check if the line collides with the x surface
        if(line_seg[3] - line_seg[0] != 0):
            tx = (prism[0] - line_seg[0]) / (line_seg[3] - line_seg[0])
            print("T for X:", tx)
            yt = line_seg[1] + tx * (line_seg[4] - line_seg[1])
            zt = line_seg[2] + tx * (line_seg[5] - line_seg[2])
            if(prism[1] <= yt) and (yt <= prism[1] + prism[3]):
                if(prism[2] <= zt) and (zt <= prism[2] + prism[5]):
                    return True
        else:
            if(prism[0] <= line_seg[0]) & (line_seg[0] <= prism[0] + prism[4]):
                checkX = True

        # check if line collides with y surface
        if (line_seg[4] - line_seg[1] != 0):
            ty = (prism[1] - line_seg[1]) / (line_seg[4] - line_seg[1])
            print("T for Y:", ty)
            zt = line_seg[2] + ty * (line_seg[5] - line_seg[2])
            xt = line_seg[0] + ty * (line_seg[3] - line_seg[0])
            if (prism[2] <= zt) and (zt <= prism[2] + prism[5]):
                if (prism[0] <= xt) & (xt <= prism[0] + prism[4]):
                    return True
        else:
            if (prism[1] <= line_seg[1]) and (line_seg[1] <= prism[1] + prism[3]):
                checkY = True

        # check if line collides with z surface
        if (line_seg[5] - line_seg[2] != 0):
            tz = (prism[2] - line_seg[2]) / (line_seg[5] - line_seg[2])
            print("T for Z:", tz)
            yt = line_seg[1] + tz * (line_seg[4] - line_seg[1])
            xt = line_seg[0] + tz * (line_seg[3] - line_seg[0])
            if (prism[1] <= yt) and (yt <= prism[2] + prism[3]):
                if (prism[0] <= xt) & (xt <= prism[0] + prism[4]):
                    return True
        else:
            if (prism[1] <= line_seg[1]) and (line_seg[1] <= prism[1] + prism[3]):
                checkY = True
        return False


if __name__ == "__main__":
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    # Set the limits for the range of plotted values
    lim = .6
    plt.xlim(-lim, lim)
    plt.ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    # create the prism to be avoided (hard coded)
    prism = ([0, 0, 0, .2, .3, .4])
    plot_linear_prism(ax, prism)
    # create the arm and report collision (random)
    line_seg = ([.1, .1, -.3,
                .1, .1, .5]) #([-.1, .1, .1, .3, .1, .1])
    xs = ([line_seg[0], line_seg[3]])
    ys = ([line_seg[1], line_seg[4]])
    zs = ([line_seg[2], line_seg[5]])
    print("X1", line_seg[0])
    print("X2", line_seg[3])
    print("X Range: ", prism[0], "->", prism[4])
    plot_lines(ax, xs,ys,zs,None)
    plt.title(newLineCollider(line_seg,prism))
    plt.show()
    # main()

"""
    Strategy for 
"""




"""
take code for web use: 


import numpy as np

#line_seg - create fake one
cube = ([11,22,33,1])

a_vec = np.array([line_seg[0][0], line_seg[0][1], line_seg[0][2]])
b_vec = np.array([line_seg[1][0], line_seg[1][1], line_seg[1][2]])
side = cube[3]
c_vec = np.array([cube[0], cube[1], cube[2]]) + side/2
r = np.sqrt(3) * side / 2
line_vec = b_vec - a_vec
line_mag = np.linalg.norm(line_vec)
circle_vec = c_vec - a_vec
proj = circle_vec.dot(line_vec / line_mag)
"""

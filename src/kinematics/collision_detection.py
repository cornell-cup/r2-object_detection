import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import sys
from rrtnode import RRTNode

"""****************************************************************************
A script to configure a robot arm and collisions (represented as cubes).
Run the script to see an example visualization of the arm and an obstruction.
Run the script with argument "rand" to see a random configuration and test
the collision detection (result of collision is printed through command line).
TODO: Implement path-planning algo | Integrate with camera's point cloud output
****************************************************************************"""


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
    xx = [x, x, x+l, x+l, x]
    yy = [y, y+l, y+l, y, y]

    kwargs = {'alpha': 1, 'color': 'blue'}
    ax.plot3D(xx, yy, [z]*5, **kwargs)
    ax.plot3D(xx, yy, [z+l]*5, **kwargs)
    ax.plot3D([x, x], [y, y], [z, z+l], **kwargs)
    ax.plot3D([x, x], [y+l, y+l], [z, z+l], **kwargs)
    ax.plot3D([x+l, x+l], [y+l, y+l], [z, z+l], **kwargs)
    ax.plot3D([x+l, x+l], [y, y], [z, z+l], **kwargs)


def plot_linear_prism(ax, prism):
        """
        Plot a cube on an instance of matplotlib.axes.

        INSTANCE ARGUMENTS:
        ax   [matplotlib.axes] : figure with 3D axes.
        cube [list]            : specifications of the cube formatted as
                                 [<x coord.>, <y coord.>, <z coord.>, <side length>].
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
        xx = [x, x, x+w, x+w, x]
        yy = [y+l, y, y, y+l, y+l]

        kwargs = {'alpha': 1, 'color': 'blue'}
        ax.plot3D(xx, yy, [z] * 5, **kwargs)
        ax.plot3D(xx, yy, [z+h]*5, **kwargs)
        ax.plot3D([x, x], [y, y], [z, z+h], **kwargs)
        ax.plot3D([x, x], [y+l, y+l], [z, z+h], **kwargs)
        ax.plot3D([x+w, x+w], [y+l, y+l], [z, z+h], **kwargs)
        ax.plot3D([x+w, x+w], [y, y], [z, z+h], **kwargs)


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
    points = node.joint_positions()
    for p in points:
        print("point", p[0], " ", p[1], " ", p[2] )
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
        line = [points[i], points[i+1]]
        if line_is_colliding(line, cube):
            return True
    return False


if __name__ == "__main__":
    # Broken, uses deprecated class NLinkArm
    # fig = plt.figure()
    # ax = plt.axes(projection="3d")
    cube = [-0.1, 0.1, 0.15, 0.15]
    # plot_linear_cube(ax, cube)
    # arm = NLinkArm(2, np.array([0.222, 0.3]))
    # print("points, ", len(arm.get_points()))
    # if len(sys.argv) == 2 and sys.argv[1] == "rand":
    #     arm.update_pitch(
    #         [arm.get_pitches()[j] + np.random.randint(-50, 50)/100 for j in range(arm.get_dof())])
    #     arm.update_yaw(
    #         [arm.get_yaws()[j] + np.random.randint(-50, 50)/100 for j in range(arm.get_dof())])
    #
    #     plot_arm(ax, arm, "arm")
    #     print("Arm is colliding:", arm_is_colliding(arm, cube))
    # else:
    #     plot_arm(ax, arm, "arm")
    # plt.legend()
    # plt.show()

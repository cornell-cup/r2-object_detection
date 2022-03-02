import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import random
from rrtnode import RRTNode

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


def plot_linear_prism(ax, prism, color):
    """
        Plot a prism on an instance of matplotlib.axes.

        INSTANCE ARGUMENTS:
        ax   [matplotlib.axes]  : figure with 3D axes.
        prism [list]            : specifications of the cube formatted as
                                  [<x coord.>, <y coord.>, <z coord.>, <length.>, <width.>, <height.>].
        color                   : A color for the prism ('red', 'blue', 'green')
        """
    # x, y, z, l = cube
    x, y, z, l, w, h = prism
    # Each item in the arrays "xx" and "yy" represent either an x or y coordinate
    # For example: (x, y+w), (x, y), (x+l, y), ... etc
    # The order of the coordinates determine which dot is plotted first, and the lines
    # between each are drawn in the same order.
    # Also, the l and w are used as the area of the base of the prism, with the width
    # moving in the positive y direction, length in the positive x, and height is z obv.
    xx = [x, x, x + l, x + l, x]
    yy = [y + w, y, y, y + w, y + w]

    kwargs = {'alpha': 1, 'color': color}
    ax.plot3D(xx, yy, [z] * 5, **kwargs)
    ax.plot3D(xx, yy, [z + h] * 5, **kwargs)
    ax.plot3D([x, x], [y, y], [z, z + h], **kwargs)
    ax.plot3D([x, x], [y + w, y + w], [z, z + h], **kwargs)
    ax.plot3D([x + l, x + l], [y + w, y + w], [z, z + h], **kwargs)
    ax.plot3D([x + l, x + l], [y, y], [z, z + h], **kwargs)


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
    # First check if either point is within the prism: (x,y,z)
    if (prism[0] <= line_seg[0]) & (line_seg[0] <= prism[0] + prism[3]):
        checkX = True
    if (prism[1] <= line_seg[1]) & (line_seg[1] <= prism[1] + prism[4]):
        checkY = True
    if (prism[2] <= line_seg[2]) & (line_seg[2] <= prism[2] + prism[5]):
        checkZ = True
    if (checkX & checkZ & checkY):
        return True
    checkX = False
    checkY = False
    checkZ = False
    if (prism[0] <= line_seg[3]) & (line_seg[3] <= prism[0] + prism[3]):
        checkX = True
    if (prism[1] <= line_seg[4]) & (line_seg[4] <= prism[1] + prism[4]):
        checkY = True
    if (prism[2] <= line_seg[5]) & (line_seg[5] <= prism[2] + prism[5]):
        checkZ = True
    if (checkX & checkZ & checkY):
        print('POINT TYPE COLLISION')
        return True

    checkX = False
    checkY = False
    checkZ = False

    # create some constants
    x = prism[0]
    y = prism[1]
    z = prism[2]
    print('x', x)
    print('y', y)
    print('z', z)
    l = prism[3]  # length goes with x
    w = prism[4]  # width goes with y
    h = prism[5]
    x1 = line_seg[0]
    x2 = line_seg[3]
    y1 = line_seg[1]
    y2 = line_seg[4]
    z1 = line_seg[2]
    z2 = line_seg[5]

    print("p1x: ", x1)
    print("p1y: ", y1)
    print("p1z: ", z1)
    print("p2x: ", x2)
    print("p2y: ", y2)
    print("p2z: ", z2)

    # check if the line collides with either  x surface
    if (x2 - x1 != 0):
        t1 = (x - x1) / (x2 - x1)
        t2 = ((x + l) - x1) / (x2 - x1)
        print("t1", t1)
        print("t2", t2)
        yt1 = y1 + t1 * (y2 - y1)
        yt2 = y1 + t2 * (y2 - y1)
        zt1 = z1 + t1 * (z2 - z1)
        zt2 = z1 + t2 * (z2 - z1)
        xt1 = x1 + t1 * (x2 - x1)
        xt2 = x1 + t2 * (x2 - x1)

        print('xt1:', xt1)
        print('xt2:', xt2)
        print('yt1:', yt1)
        print('yt2:', yt2)
        print('zt1:', zt1)
        print('zt2:', zt2)
        if (y <= yt1 and yt1 <= y + w and z <= zt1 and zt1 <= z + h and t1 >= 0 and t1 <= 1):
            print('collision first side X')
            return True
        elif (y <= yt2 and yt2 <= y + w and z <= zt2 and zt2 <= z + h and t2 >= 0 and t2 <= 1):
            print('collision second side X')
            return True
    print('no collision in x planes')

    # check if the line collides with either y surface
    if (y2 - y1 != 0):
        t1 = (y - y1) / (y2 - y1)
        t2 = ((y + w) - y1) / (y2 - y1)
        xt1 = x1 + t1 * (x2 - x1)
        xt2 = x1 + t2 * (x2 - x1)
        zt1 = z1 + t1 * (z2 - z1)
        zt2 = z1 + t2 * (z2 - z1)
        if (x <= xt1 and xt1 <= x + l and z <= zt1 and zt1 <= z + h and t1 >= 0 and t1 <= 1):
            print('collision first side Y')
            print(xt1)
            print(zt1)
            return True
        elif (x <= xt2 and xt2 <= x + l and z <= zt2 and zt2 <= z + h and t2 >= 0 and t2 <= 1):
            print('collision second side Y')
            print(xt2)
            print(zt2)
            return True
    print('no collision in y planes')

    # check if the line collides with either z surface
    if (z2 - z1 != 0):
        t1 = (z - z1) / (z2 - z1)
        t2 = ((z + h) - z1) / (z2 - z1)
        xt1 = x1 + t1 * (x2 - x1)
        xt2 = x1 + t2 * (x2 - x1)
        yt1 = y1 + t1 * (y2 - y1)
        yt2 = y1 + t2 * (y2 - y1)

        if (x <= xt1 and xt1 <= x + l and y <= yt1 and yt1 <= y + w and t1 >= 0 and t1 <= 1):
            print('collision first side Z')
            return True
        elif (x <= xt2 and xt2 <= x + l and y <= yt2 and yt2 <= y + w and t2 >= 0 and t2 <= 1):
            print('collision second side Z')
            return True
    print('no collision in z planes')


def checkCollision(prism, rrtNode):
    collision = False
    for i in range(len(rrtNode.joint_positions) - 1):
        p1 = ([rrtNode.joint_positions[i]])
        p2 = ([rrtNode.joint_positions[i + 1]])
        # print([p1[0][0],p1[0][1],p1[0][2]])
        line_seg = ([p1[0][0], p1[0][1], p1[0][2], p2[0][0], p2[0][1], p2[0][2]])
        if (newLineCollider(line_seg, prism)):
            print('Collision on line', i, '-', i + 1)
            print("p1", p1)
            print("p2", p2)
            return True
    return False


def main():
    """
    Places a prism randomly in 3D space, and draws a larger prism around it that represents it's
    bounding box. Then checks if a randomly generated arm collides the outer prism at any point.

    """
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    # Set the limits for the range of plotted values
    lim = .6
    plt.xlim(-lim, lim)
    plt.ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    # create the prism to be avoided (hard coded)
    lim = .3
    location = random.uniform(-lim,lim)
    prism = ([location, location, location, .2, .3, .4])
    exoPrism = ([prism[0]-.05,prism[1]-.05,prism[2]-.05,prism[3]+.1,prism[4]+.1,prism[5]+.1])
    plot_linear_prism(ax, prism, 'blue')
    plot_linear_prism(ax, exoPrism, 'red')
    lim = .5
    """
    This code was used to generate a single line, and has since been replaced 
    by code that generates a random N link arm. 
    
        randomX1 = random.uniform(-lim, lim)
        randomY1 = random.uniform(-lim, lim)
        randomZ1 = random.uniform(-lim, lim)
    
        randomX2 = random.uniform(-lim, lim)
        randomY2 = random.uniform(-lim, lim)
        randomZ2 = random.uniform(-lim, lim)
        line_seg = ([randomX1, randomY1, randomZ1,
                     randomX2, randomY2, randomZ2])  # ([-.1, .1, .1, .3, .1, .1])
        # line_seg = ([-.3,.15,-.2,.6,.15,.2]) # intersects x faces
        # line_seg = ([.16, -.3, .2, .16, .3, .2]) # intersects y faces
        # line_seg = ([.2, -.2, -.3, .2, .15, .5]) # intersects z faces
        xs = ([line_seg[0], line_seg[3]])
        ys = ([line_seg[1], line_seg[4]])
        zs = ([line_seg[2], line_seg[5]])
        plot_lines(ax, xs, ys, zs, None)
    """
    # Create a random arm
    arm = RRTNode(None)
    plot_arm(ax,arm,None)
    class Index:
        def draw(self, event):
            plt.close()
            main()

    axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
    run = Button(axnext, 'ReRun')
    callback = Index()
    run.on_clicked(callback.draw)
    if checkCollision(exoPrism,arm):
        plt.title("Collision Detected!")
    else:
        plt.title("Wow no Collision")
    plt.show()


if __name__ == "__main__":
    main()

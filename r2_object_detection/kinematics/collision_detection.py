import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import sys
from visualization import joint_positions

"""****************************************************************************
A script to configure a robot arm and collisions (represented as cubes).
Run the script to see an example visualization of the arm and an obstruction.
Run the script with argument "rand" to see a random configuration and test
the collision detection (result of collision is printed through command line).
TODO: Implement path-planning algo | Integrate with camera's point cloud output
****************************************************************************"""


class NLinkArm(object):
    """
    Class to configure the robot arm.
    There's a point for each joint and one for each of the arm's ends.
    For ex. there will be 7 points for a 6DoF arm.
    The starting point is set at (0,0,0) and does not change.

    INSTANCE ATTRIBUTES:
    n_links      [int]      : degrees of freedom.

    link_lengths [np array] : lengths of each link between joints.
                              len(link_lengths) == n_links.

    yaws         [np array] : yaw angles of joints w.r.t. previous joint.
                              yaw is the counterclockwise angle on the xy plane
                              a.k.a. theta in spherical coordinates.
                              len(yaws) == n_links.

    pitches      [np array] : pitch angles of joints w.r.t. previous joint.
                              pitch is the clockwise angle on the yz plane
                              a.k.a. psi in spherical coordinates.
                              len(pitches) == n_links.

    points       [list]     : coordinates of joints and arm ends.
                              len(points) == n_links + 1.
    """

    def __init__(self, dof, link_lengths):
        """
        Initialize a configuration of a robot arm with [dof] degrees of freedom.
        """
        self.n_links = dof
        self.link_lengths = link_lengths
        self.yaws = np.array([0. for _ in range(dof)])
        self.pitches = np.array([0. for _ in range(dof)])
        self.points = np.array([[0., 0., 0.] for _ in range(dof + 1)])
        self.update_points_alternate()

    def update_yaw(self, yaws):
        """
        Redefine the yaw angles.

        KEYWORD ARGUMENTS:
        yaws [np array] : new yaw angles for each joint.

        PRECONDITIONS:
        len(yaws) == self.n_links.
        """
        self.yaws = np.array(yaws)
        self.update_points()

    def update_pitch(self, pitches):
        """
        Redefine the pitch angles.

        KEYWORD ARGUMENTS:
        pitches [np array] : new pitch angles for each joint.

        PRECONDITIONS:
        len(pitches) == self.n_links.
        """
        self.pitches = np.array(pitches)
        self.update_points()

    def update_points(self):
        """
        Redefine the points according to yaw and pitch angles, to match the method used in visualization.py
        """
        # def joint_positions(theta_1, phi_1, theta_2, phi_2):
        r_1 = self.link_lengths[0]
        r_2 = self.link_lengths[1]
        theta_1, theta_2 = self.get_pitches()
        phi_1, phi_2 = self.get_yaws()
        shoulder_coord, elbow_coord = joint_positions(theta_1, phi_1, theta_2, phi_2)

        self.points[1] = [r_1 * np.sin(theta_1) * np.cos(phi_1), r_1 *
                           np.sin(theta_1) * np.sin(phi_1), r_1 * np.cos(theta_1)]
        self.points[2] = [shoulder_coord[0] + r_2 * np.sin(theta_2) * np.cos(phi_2), shoulder_coord[1] + r_2 *
                           np.sin(theta_2) * np.sin(phi_2), shoulder_coord[2] + r_2 * np.cos(theta_2)]


    def update_points_alternate(self):
        """
        Redefine the points according to yaw and pitch angles.
        """
        for i in range(1, self.n_links + 1):
            yaw = np.sum(self.yaws[i-1])
            pitch = np.sum(self.pitches[i-1])
            r = self.link_lengths[i - 1]
            hyp = r * np.sin(pitch)  # projection of vector on the xy plane
            self.points[i][0] = self.points[i - 1][0] + hyp * np.cos(yaw)
            self.points[i][1] = self.points[i - 1][1] + hyp * np.sin(yaw)
            self.points[i][2] = self.points[i - 1][2] + np.cos(pitch)

    def get_points(self):
        """
        Return the coordinates of the arm's joints and ends.

        RETURNS: np array
        """
        return self.points

    def get_yaws(self):
        """
        Return the arm's yaw angles.

        RETURNS: np array
        """
        return self.yaws

    def get_pitches(self):
        """
        Return the arm's pitch angles.

        RETURNS: np array
        """
        return self.pitches

    def get_dof(self):
        """
        Return the degrees of freedom.

        RETURNS: int
        """
        return self.n_links


def plot_linear_cube(ax, cube):
    """
    Plot a cube on an instance of matplotlib.axes.

    INSTANCE ARGUMENTS:
    ax   [matplotlib.axes] : figure with 3D axes.
    cube [list]            : specifications of the cube formatted as
                             [<x coord.>, <y coord.>, <z coord.>, <side length>].
    """
    x, y, z, l = cube
    xx = [x, x, x+l, x+l, x]
    yy = [y, y+l, y+l, y, y]
    kwargs = {'alpha': 1, 'color': 'red'}
    ax.plot3D(xx, yy, [z]*5, **kwargs)
    ax.plot3D(xx, yy, [z+l]*5, **kwargs)
    ax.plot3D([x, x], [y, y], [z, z+l], **kwargs)
    ax.plot3D([x, x], [y+l, y+l], [z, z+l], **kwargs)
    ax.plot3D([x+l, x+l], [y+l, y+l], [z, z+l], **kwargs)
    ax.plot3D([x+l, x+l], [y, y], [z, z+l], **kwargs)


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


def plot_arm(ax, arm, line_label):
    """
    Plot a robot arm on an instance of matplotlib.axes.

    INSTANCE ARGUMENTS:
    ax         [matplotlib.axes] : figure with 3D axes.
    arm        [NLinkArm]        : robot arm configuration.
    line_label [str]             : label for the arm plot.
    """
    points = arm.get_points()
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


def arm_is_colliding(arm, cube):
    """
    Return True if the arm is colliding with the cube, else False.

    INSTANCE ARGUMENTS:
    arm  [NLinkArm] : robot arm configuration.
    cube [list]     : specifications of the cube formatted as
                      [<x coord.>, <y coord.>, <z coord.>, <side length>].
    """
    points = arm.get_points()
    for i in range(arm.get_dof() - 1):
        line = [points[i], points[i+1]]
        if line_is_colliding(line, cube):
            return True
    return False


if __name__ == "__main__":
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    cube = [0.1, 0, 0, 0.2]
    plot_linear_cube(ax, cube)
    arm = NLinkArm(2, np.array([0.222, 0.3]))
    print("points, ", len(arm.get_points()))
    if len(sys.argv) == 2 and sys.argv[1] == "rand":
        arm.update_pitch(
            [arm.get_pitches()[j] + np.random.randint(-50, 50)/100 for j in range(arm.get_dof())])
        arm.update_yaw(
            [arm.get_yaws()[j] + np.random.randint(-50, 50)/100 for j in range(arm.get_dof())])

        plot_arm(ax, arm, "arm")
        print("Arm is colliding:", arm_is_colliding(arm, cube))
    else:
        plot_arm(ax, arm, "arm")
    plt.legend()
    plt.show()

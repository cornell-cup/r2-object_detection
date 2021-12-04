"""Implementation of a linear arm pathing algorithm that uses RRT to avoid obstacles.

Generates a linear path from the current position of the arm to a desired position. If there is an obstacle in the way
of this path, maneuvers around the obstacle with RRT. For details on the RRT algorithm, see pure_rrt_angles.py.

Written by Simon Kapen, Fall 2021.
"""

import math
import numpy as np
import pure_rrt_angles as rrt
from pure_rrt_angles import random_angle_config
from rrtgraph import Graph
from rrtnode import RRTNode
import line
import collision_detection as cd
import rrtplot
import time


def compute_step_sizes(start_angles, end_angles, num_iter):
    """Computes each arm angle's step size based on how long it needs to travel to go from the start to end pose.

    Depending on whether the anticipated path goes through the no-go zone, switches that path to go the other way
    around.

    Args:
        start_angles: The initial angles of the arm.
        end_angles: The desired angles of the arm.
        num_iter: The number of angle configurations to be generated on the path in between the start and end positions.
        arm: An instance of precision_arm.PrecisionArm, containing the bounds of each arm angle.

    Returns:
        A array containing the step sizes of each angle of the arm.
    """

    # Strategy for bounding angles: split into two quadrants: (1) less than the first bound,
    #                                                         (2) greater than the second bound.
    # If both are in the same quadrant, use the closest angle distance as usual.
    # If start_angles[i] in (1) and end_angles[i] in (2) and angle distance > 0, we must go the other way.
    # If start_angles[i] in (2) and end_angles[i] in (1) and angle distance < 0, we must go the other way.
    step_sizes = []
    true_angle_distances = rrt.true_angle_distances_arm(start_angles, end_angles)

    bounds = RRTNode(None).bounds
    for i in range(len(start_angles)):
        if i >= len(bounds) or bounds[i] is None:
            angle_distance = end_angles[i] - start_angles[i]
            step_size = angle_distance / num_iter
            if angle_distance > math.pi:
                new_angle_dist = (2 * math.pi - end_angles[i] + start_angles[i])
                step_size = -new_angle_dist / num_iter
            elif angle_distance < -math.pi:
                new_angle_dist = (2 * math.pi - start_angles[i] + end_angles[i])
                step_size = new_angle_dist / num_iter
        else:
            bound_1 = bounds[i][0]
            bound_2 = bounds[i][1]
            th_1 = start_angles[i]
            th_2 = end_angles[i]
            angle_dist = true_angle_distances[i]

            if th_1 < bound_1 and th_2 > bound_2:
                if true_angle_distances[i] > 0:
                    angle_dist = -(2 * math.pi - true_angle_distances[i])

            if th_2 < bound_1 and th_1 > bound_2:
                if true_angle_distances[i] < 0:
                    angle_dist = 2 * math.pi + true_angle_distances[i]

            step_size = angle_dist / num_iter

        step_sizes.append(step_size)
    return step_sizes


def generate_linear_path(start_angles, end_angles, num_iter):
    """Generates a linear path of arm configurations that moves an arm configuration from start_angles to end_angles.

    Args:
        start_angles: The initial angles of the arm.
        end_angles: The desired angles of the arm.
        num_iter: The number of angle configurations to be generated on the path in between the start and end positions.
    Returns:
        a path of nodes from start_pos to end_pos, with [num_iter] iterations of equal length.
    """

    step_sizes = compute_step_sizes(start_angles, end_angles, num_iter)

    g = Graph(start_angles, end_angles)

    current_angles = start_angles
    current_idx = 0
    current_node = g.nodes[0]
    for i in range(num_iter):
        # if not rrt.valid_configuration(current_node.angles):
        # print("Iteration {} not valid".format(i))
        #    return g, False
        new_angles = np.add(current_angles, step_sizes)
        new_angles = np.mod(new_angles, math.pi * 2)

        new_node = RRTNode(new_angles)

        new_idx = g.add_vex(new_node)

        dist = line.distance(new_node.end_effector_pos, current_node.end_effector_pos)

        g.add_edge(current_idx, new_idx, dist)

        current_angles = new_angles
        current_idx = new_idx
        current_node = new_node

        if i == num_iter - 1:
            rounded_current = np.around(current_angles, decimals=4)
            rounded_end = np.around(end_angles, decimals=4)


            if not np.array_equal(rounded_current, rounded_end):
                return g, False
    return g, True


def valid_path_configuration(pose, obstacles):
    """Determines whether a the input node is a valid arm configuration.

    Args:
        pose: An RRTNode instance.
        obstacles: An array of float arrays representing cube bounding box obstacles.

    Returns:
         True if pose does not collide with any of the obstacles and is a valid arm configuration. False otherwise.
    """

    for obs in obstacles:
        if cd.arm_is_colliding(pose, obs):
            return False

    # fix valid configuration method, then check that before returning true
    # if not rrt.valid_configuration(pose.angles):
    #     print("not valid")
    return True


def rrt_hd_tl(path, obstacles):
    """Determines the head and tail of a path of arm configurations that passes through an obstacle(s).

    Precondition: The start and end nodes are valid poses that do not collide with any obstacles.

    Args:
        path: An array of RRTNode instances representing the generated linear path.
        obstacles: An array of float arrays representing cube obstacles.

    Returns:
         Two arrays of RRTNode instances representing the longest start and end segments without a collision.
    """
    new_path_hd = [path[0]]
    new_path_tl = [path[-1]]

    # Find first node that collides
    i = 1
    while i < len(path) - 1 and valid_path_configuration(path[i], obstacles):
        new_path_hd.append(path[i])
        i += 1

    # Find last node that collides
    i = len(path) - 2
    while i >= 1 and valid_path_configuration(path[i], obstacles):

        new_path_tl.insert(0, path[i])
        i -= 1

    return new_path_hd, new_path_tl


def replace_with_rrt(path_hd, path_tl, obstacles):
    """Fills in the gap between two lists of arm segments with an RRT path.

    Args:
        path_hd: An array of RRTNode instances representing the head of a linear path.
        path_tl: An array of RRTNode instances representing the tail of a linear path.
        obstacles: An array of float arrays representing cube obstacles.

    Returns:
        A complete path with no colliding arm configurations, or None if such a path is not found.
    """

    rrt_start = path_hd[-1]
    rrt_end = path_tl[0]

    g = rrt.rrt(rrt_start.angles, rrt_end.angles, obstacles, n_iter=1000, radius=.02)
    if g.success:
        print("rrt success :)")
    else:
        print("rrt failed :(")
        for node in path_hd + path_tl:
            g.add_vex(node)
        rrtplot.plot_3d(g, path_hd + path_tl, obstacles)
        return None
    path_mid = rrt.dijkstra(g)
    if g is not None:
        return path_hd + path_mid + path_tl

    else:
        return None


def path_is_colliding(path, obstacles):
    """Determines whether there is at least one node in a path of nodes that collides with an obstacle.

    Args:
        path: An array of RRTNode instances representing a path from a start node to an end node.
        obstacles: An array of float arrays representing cube obstacles.

    Returns:
        True if one or more of the nodes in path collides with an obstacle.
    """

    for obstacle in obstacles:
        for node in path:
            if cd.arm_is_colliding(node, obstacle):
                return True

    return False


def linear_rrt(start_angles, end_angles, num_iter, obstacles):
    """Generates a linear path, and maneuvers around obstacles with RRT if necessary.

    Args:
        start_angles: The initial angles of the arm.
        end_angles: The desired angles of the arm.
        num_iter: The number of angle configurations to be generated on the path in between the start and end positions.
        obstacles: An array of float arrays representing cube obstacles.

    Returns:
        An array of RRTNode instances representing a valid path between the start and end configurations.
    """

    g = generate_linear_path(start_angles, end_angles, num_iter)
    linear_path = g[0].nodes

    if path_is_colliding(linear_path, obstacles):
        new_path_hd, new_path_tl = rrt_hd_tl(linear_path, obstacles)
        linear_path = replace_with_rrt(new_path_hd, new_path_tl, obstacles)

    if linear_path is None:
        return linear_path, False
    return linear_path, True


def linearity_test(num_trials, iter_per_path=50):
    """Runs num_trials of a pure linear arm pathing approach with random start and end configuration.

    Success is defined as converging to the desired end position.

    Returns:
        A float representing the percentage of successful trials.
    """

    s_count = 0
    for i in range(num_trials):
        start_pos = random_angle_config()
        end_pos = random_angle_config()

        while not arm.angles_within_bounds(start_pos) or not arm.angles_within_bounds(end_pos):
            start_pos = random_angle_config()
            end_pos = random_angle_config()

        _, success = generate_linear_path(start_pos, end_pos, iter_per_path)

        if success:
            s_count = s_count + 1
    return (s_count / num_trials) * 100


""" Converts node angles (in radians) to their degree equivalent 
    @return - an array consisting of 6 degree measurements representing
    the node """
def convert_to_degrees(rrtnode):
    degrees = [0 for a in range(6)]
    for ind, val in enumerate(rrtnode.angles):
        degrees[ind] = (val * 180) / math.pi
    return degrees


def linear_rrt_test(num_trials, obstacles, iter_per_path=10):
    """Runs num_trials of a linear rrt pathing approach.

    Success is defined as converging to the desired end position, and any intermediate RRT converging to the desired end
    position.
    """

    start_time = time.time()
    s_count = 0
    for i in range(num_trials):
        start_node = RRTNode(configuration=None)
        end_node = RRTNode(configuration=None)

        while not valid_path_configuration(start_node, obstacles):
            start_node = RRTNode(None)

        while not valid_path_configuration(end_node, obstacles):
            end_node = RRTNode(None)

        if rrt.arm_is_colliding(end_node, obstacles):
            raise Exception("Approved a colliding node")

        _, success = linear_rrt(start_node.angles, end_node.angles, iter_per_path, obstacles)

        if success:
            s_count = s_count + 1
        else:
            print("start_pos =", format(start_node.angles))
            print("end_pos =", format(end_node.angles))

    print("success rate in {t} trials: {r}".format(t=num_trials, r=(s_count / num_trials) * 100))
    print("average time per trial: {}".format((time.time() - start_time) / num_trials))


def plot_random_path(iterations, obstacles):
    """Plots a random path between random start and end configurations.

    Args:
        iterations: Integer representing the number of intermediate poses between the start and end configurations.
        obstacles: Array of int arrays representing cube obstacles.
        arm: A precision_arm.PrecisionArm instance, containing the bounds for arm angles.
    """

    start_node = RRTNode([0, 0, 0, 0, 0])
    end_node = RRTNode([-1.0237, 0.6567117, 0.362883, 1.02, 1.022806])

    # while not valid_path_configuration(start_node, obstacles):
    #     start_node = RRTNode(None)
    # while not valid_path_configuration(end_node, obstacles):
    #     end_node = RRTNode(None)

    path, _ = linear_rrt(start_node.angles, end_node.angles, iterations, obstacles)

    g = Graph(start_node.angles, end_node.angles)

    if path is not None:
        for node in path:
            g.add_vex(node)
        rrtplot.plot_3d(g, path, obstacles)


# Constants for quick testing of certain cases
RRT_TEST_SEED = 1
CLEAR_PATH_TEST_SEED = 20
INCORRECT_END_SEED = 420

if __name__ == '__main__':
    # random.seed(a=INCORRECT_END_SEED)
    np.set_printoptions(precision=20)
    trials = 100
    iterations = 10
    # obstacles = [[-0.1, 0.1, 0.15, 0.2]]
    # THIS METHOD was modified to use plot_linear_prism()
    obstacles = [[.0, .0, -.4, .3, .4, .6]]
    # print("success rate in {t} trials: {r}".format(t=trials, r=linearity_test(trials)))
    # linear_rrt_test(100, obstacles)
    plot_random_path(iterations, obstacles)

"""Implementation of a linear arm pathing algorithm that uses Optimistic Predictive Cost to avoid obstacles.

Generates a linear path from the current position of the arm to a desired position. If there is an obstacle in the way
of this path, maneuvers around the obstacle with OPC. For details on the OPC algorithm, see optimal_min_cost.py.

Written by Simon Kapen '24 and Raj Sinha '25, Fall 2021.

TODO: Refactor the names of the functions in this file, as we use Optimistic Predictive Cost instead of RRT
"""

import math
import numpy as np
from .pure_rrt import dijkstra, rrt
from .arm_graph import Graph
from .arm_node import Node
from .util import line
from .collision_detection import newLineCollider, arm_is_colliding_prism, arm_is_colliding_prisms
from .arm_plot import plot_3d
import time
from ikpy.chain import Chain
from .optimal_min_cost import find_path
from .util.angles import true_angle_distances_arm
from .obstacle_generation import random_start_environment
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d
from typing import List


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
    true_angle_distances = true_angle_distances_arm(start_angles, end_angles)

    bounds = Node(None).bounds
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
    current_node = g.nodes[0]
    for i in range(num_iter):
        new_angles = np.add(current_angles, step_sizes)
        new_angles = np.mod(new_angles, math.pi * 2)

        new_node = Node(new_angles)

        new_idx = g.add_vex(new_node, current_node)

        current_angles = new_angles
        current_node = new_node

        if i == num_iter - 1:
            rounded_current = np.around(current_angles, decimals=4)
            rounded_end = np.around(end_angles, decimals=4)

            if not np.array_equal(rounded_current, rounded_end):
                return g, False
    return g, True

def test_for_alison(start_angles, end_angles, num_iter):
    g = generate_linear_path(start_angles,end_angles,num_iter)
    linear_path = rrt.dijkstra(g)
    list_of_angles = []
    for i in range(linear_path):
        list_of_angles.append(linear_path[i].angles)
    return list_of_angles


def valid_path_configuration(pose, obstacles):
    """Determines whether a the input node is a valid arm configuration.

    Args:
        pose: An Node instance.
        obstacles: An array of float arrays representing cube bounding box obstacles.

    Returns:
         True if pose does not collide with any of the obstacles and is a valid arm configuration. False otherwise.
    """

    for obs in obstacles:
        if arm_is_colliding_prism(pose, obs):
            return False

    # TODO: fix valid configuration method, then check that before returning true

    return True


def path_is_colliding(path, obstacles):
    """Determines whether there is at least one node in a path of nodes that collides with an obstacle.

    Args:
        path: An array of Node instances representing a path from a start node to an end node.
        obstacles: An array of float arrays representing cube obstacles.

    Returns:
        True if one or more of the nodes in path collides with an obstacle.
    """

    for obstacle in obstacles:
        for node in path:
            if arm_is_colliding_prism(node, obstacle):
                return True

    return False


def linear_path_to_angles(start_angles, end_angles, obstacles, num_iter=15):
    """Generates a linear path to a desired end effector position, and maneuvers around obstacles with OPC if necessary.

        Args:
            start_angles: The initial angles of the arm.
            end_angles: The desired end angles of the arm.
            obstacles: An array of float arrays representing cube obstacles.
            num_iter: The number of angle configurations to be generated on the path in between the start and end positions.
            degrees: Whether to convert to and from degrees for motor output.

        Returns:
            An array of Node instances or float arrays representing a valid path between the start and end configurations
        """
    g = generate_linear_path(start_angles, end_angles, num_iter)

    point = Node(end_angles).end_effector_pos
    linear_path = g[0].nodes
    if path_is_colliding(linear_path, obstacles):
        print("Finding OPC path")
        g = opc.find_path((point[0], point[1], point[2]), start_angles, obstacles)
        if g.success:
            linear_path = dijkstra(g)
        else:
            linear_path = None

    if linear_path is None:
        return linear_path, False

    return linear_path, True


def linear_path_to_point(start_angles, end_x, end_y, end_z, obstacles, num_iter=15):
    """Generates a linear path to a desired end effector position, and maneuvers around obstacles with OPC if necessary.

    Args:
        start_angles: The initial angles of the arm.
        end_x, end_y, end_z: The desired end effector position of the arm.
        num_iter: The number of angle configurations to be generated on the path in between the start and end positions.
        obstacles: An array of float arrays representing cube obstacles.
        degrees: Whether to convert to and from degrees for motor output.

    Returns:
        An array of Node instances or float arrays representing a valid path between the start and end configurations
    """
    end_angles = Node.from_point((end_x, end_y, end_z)).angles
    return linear_path_to_angles(start_angles, end_angles, obstacles, num_iter)


def degrees_to_radians(angles: List[float]):
    """Converts an input array in degrees into an output array in radians."""
    radians = [0 for a in range(6)]
    for ind, val in enumerate(angles):
        radians[ind] = (val * math.pi) / 180
    return radians


def radians_to_degrees(node):
    """Converts an Node instance into a degree array to be used in arm encoder movements.

       Returns: An array consisting of 6 degree measurements representing the node. """

    degrees = [0 for a in range(6)]
    for ind, val in enumerate(node.angles):
        degrees[ind] = (val * 180) / math.pi 
    return degrees


def path_radians_to_degrees(path: List[Node]):
    """Converts a path of Node instances into a path of degree arrays to be used in arm encoder movements."""
    return list(map(radians_to_degrees, path))


def linear_path_test(num_trials, obstacles, iter_per_path=10):
    """Runs num_trials of a linear pathing approach.

    Success is defined as converging to the desired end position, and any intermediate OPC converging to the desired end
    position.
    """

    start_time = time.time()
    s_count = 0
    for i in range(num_trials):
        print("Trial", i)
        start_node = Node(configuration=None)
        end_node = Node(configuration=None)

        while not valid_path_configuration(start_node, obstacles):
            start_node = Node(None)

        while not valid_path_configuration(end_node, obstacles):
            end_node = Node(None)

        if arm_is_colliding_prisms(end_node, obstacles):
            raise Exception("Approved a colliding node")

        _, success = linear_path_to_point(start_node.angles, end_node.end_effector_pos, obstacles, iter_per_path)

        if success:
            s_count = s_count + 1
        else:
            print("start_pos =", format(start_node.angles))
            print("end_pos =", format(end_node.angles))

    print("success rate in {t} trials: {r}".format(t=num_trials, r=(s_count / num_trials) * 100))
    print("average time per trial: {}".format((time.time() - start_time) / num_trials))


def plot_random_path(iterations, num_obstacles):
    """Plots a random path between random start and end configurations.

    Args:
        iterations: Integer representing the number of intermediate poses between the start and end configurations.
        obstacles: Array of int arrays representing cube obstacles.
        arm: A precision_arm.PrecisionArm instance, containing the bounds for arm angles.
    """

    start_node, end_node, obstacles, _ = random_start_environment(num_obstacles, [[-.08, .08], [.08, .2], [.12, .2]],
                                                                   [[-.2, .2], [-.04, .2], [-.2, .2]],
                                                                  obstacle_size=.1)

    plot_path(start_node.angles, end_node.angles, iterations, obstacles)


def plot_path(start_angles, end_angles, iterations, obstacles):
    """Plots a path between two given angle configurations."""
    start_node = Node(start_angles)
    end_node = Node(end_angles)

    if (not valid_path_configuration(start_node, obstacles)) or (not valid_path_configuration(end_node, obstacles)):
        raise Exception("Invalid configuration")

    pos = end_node.end_effector_pos
    path, _ = linear_path_to_point(start_node.angles, pos[0], pos[1], pos[2], obstacles, iterations)

    g = Graph(start_node.angles, end_node.angles)

    if path is not None:
        g.add_vex(path[0], g.start_node)
        for i in range(len(path)):
            try:
                g.add_vex(path[i], path[i-1])
            except IndexError:
                pass
        arm_plot.plot_3d(g, path, obstacles)
    else:
        print("No Path Found :(")


# Constants for quick testing of certain cases
RRT_JUMP_SEED = 6869
RRT_TEST_SEED = 120938914
CLEAR_PATH_TEST_SEED = 20
INCORRECT_END_SEED = 420


def path_optimizer(path, prism):
    """

    Args:
        path: refers to the output of dijkstra method from pure_rrt_angles,
              a list of nodes
        prism: the object to be avoided, given in the form
               [<x coord.>, <y coord.>, <z coord.>, <length.>, <width.>, <height.>].

    Returns:
        A new list with some nodes removed where linear paths can be created.
        list length <= path length

    """
    optimizedList = path.copy()
    # To save time we will only check every other node
    for i in range(0, len(path) - 2, 2):
        p1 = path[i].end_effector_pos
        p2 = path[i + 2].end_effector_pos
        line_seg = ([p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]])
        if cd.newLineCollider(line_seg, prism) == False:
            optimizedList.remove(path[i + 1])
    return optimizedList

    # start_point = [0, 0, 0]
    # sucess = 0
    # tests = 10
    # start_time = time.time()
    # fig = plt.figure()
    # ax = plt.axes(projection="3d")
    # for i in range(tests):
    #     fail = False
    #     randomX = random.uniform(-.08, .08)
    #     randomY = random.uniform(-.08, .08)
    #     randomZ = random.uniform(0, .105)
    #     end_point = [randomX, randomY, randomZ]
    #     #print(end_point)
    #     angles = inverse_kinematics(end_point)
    #     matrices = forward_kinematics(angles)
    #     #print(angles)
    #     if not thresholdCheck(randomX, matrices[0][3], .001):
    #         fail = True
    #         print("Fail in X")
    #     if not thresholdCheck(randomY, matrices[1][3], .001):
    #         fail = True
    #         print("Fail in Y")
    #     if not thresholdCheck(randomZ, matrices[2][3], .001):
    #         fail = True
    #         print("Fail in Z")
    #     if not fail:
    #         sucess += 1
    #     plt.plot(end_point[0], end_point[1], end_point[2],'bo',markersize=15)
    #     arm_chain.plot(angles, ax, show=False)
    # #arm_plot.plot_nodes(ax, [end_point])
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # # Set the limits for the range of plotted values
    # lim = .12
    # plt.xlim(-lim, lim)
    # plt.ylim(-lim, lim)
    # ax.set_zlim(-.2, .2)
    # run_time = time.time() - start_time
    # print("Successes: ", sucess)
    # print("Failures: ", tests - sucess)
    # print("Rate: ", sucess / tests)
    # print("Total Run Time: ", run_time)
    # print("Average Run Time: ", run_time / tests)
    # plt.show()
# start_point = [0, 0, 0]
# sucess = 0
# tests = 10
# start_time = time.time()
# fig = plt.figure()
# ax = plt.axes(projection="3d")
# for i in range(tests):
#     fail = False
#     randomX = random.uniform(-.08, .08)
#     randomY = random.uniform(-.08, .08)
#     randomZ = random.uniform(0, .105)
#     end_point = [randomX, randomY, randomZ]
#     #print(end_point)
#     angles = inverse_kinematics(end_point)
#     final_position = forward_kinematics(angles)
#     #print(angles)
#     if not thresholdCheck(randomX, final_position[0][3], .001):
#         fail = True
#         print("Fail in X")
#     if not thresholdCheck(randomY, final_position[1][3], .001):
#         fail = True
#         print("Fail in Y")
#     if not thresholdCheck(randomZ, final_position[2][3], .001):
#         fail = True
#         print("Fail in Z")
#     if not fail:
#         sucess += 1
#     plt.plot(end_point[0], end_point[1], end_point[2],'bo',markersize=15)
#     arm_chain.plot(angles, ax, show=False)
# #arm_plot.plot_nodes(ax, [end_point])
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# # Set the limits for the range of plotted values
# lim = .12
# plt.xlim(-lim, lim)
# plt.ylim(-lim, lim)
# ax.set_zlim(-.2, .2)
# run_time = time.time() - start_time
# print("Successes: ", sucess)
# print("Failures: ", tests - sucess)
# print("Rate: ", sucess / tests)
# print("Total Run Time: ", run_time)
# print("Average Run Time: ", run_time / tests)
# plt.show()


if __name__ == '__main__':
    # random.seed(a=RRT_JUMP_SEED)
    np.set_printoptions(precision=20)
    trials = 100
    iterations = 10
    num_obstacles = 3

    plot_random_path(iterations, num_obstacles)

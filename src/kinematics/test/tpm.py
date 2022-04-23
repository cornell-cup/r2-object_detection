"""
A set of functions that generate Technical Performance Measures (TPMs) based on a list of graphs generated by an arm
pathing algorithm.

Written by Simon Kapen '24, Spring 2022.
"""

from arm_graph import Graph
from util.angles import true_angle_distances_arm
from util import line
from arm_node import Node
import random
from collision_detection import arm_is_colliding_prisms
import obstacle_generation
<<<<<<< HEAD
=======
from arm_plot import plot_3d
>>>>>>> kinematics


def avg_nodes_test(graphs: list[Graph]):
    """ The average amount of nodes generated until the end goal is reached. """
    total_nodes = 0
    for i in range(0, len(graphs)):
        total_nodes += len(graphs[i].nodes)

    return total_nodes / len(graphs)


def converge_test(graphs: list[Graph]):
    """ Counts the amount of successes for a list of RRT graphs. """
    num_success = 0
    for i in range(0, len(graphs)):
        if graphs[i].success:
            num_success += 1

    return num_success


def max_iter_success(graphs):
    """ Determines the maximum iterations for a success in a set of graphs.
        TODO: Currently does not work. Need to find a way to get iterations for a single graph to converge."""
    max_success = -1
    for i in range(0, len(graphs)):
        if graphs[i].success and len(graphs[i].ranking) > max_success:
            max_success = len(graphs[i].ranking)

    return max_success


def print_failed_cases(graphs, failed_obstacles):
    """ Outputs failed start angles, end angles, and obstacles for a list of RRT graphs. """
    print("FAILED CASES")

    for i in range(0, len(graphs)):
        if not graphs[i].success:
            print("start_angles =", graphs[i].start_node.angles)
<<<<<<< HEAD
            ea = graphs[i].end_node.angles
            print("end_angles = [{ea0}, {ea1}, {ea2}, {ea3}, {ea4}]"
                  .format(ea0=ea[0], ea1=ea[1], ea2=ea[2], ea3=ea[3], ea4=ea[4]))
            print("obstacles =", failed_obstacles[i])
            print()
=======
            # ea = graphs[i].end_node.angles
            # print("end_angles = [{ea0}, {ea1}, {ea2}, {ea3}, {ea4}]"
            #       .format(ea0=ea[0], ea1=ea[1], ea2=ea[2], ea3=ea[3], ea4=ea[4]))
            print("obstacles =", failed_obstacles[i])

            print()
            plot_3d(graphs[i], [graphs[i].start_node], failed_obstacles[i])
>>>>>>> kinematics


def avg_distance_traveled_test(paths):
    total_dist = 0
    for path in paths:
        path_dist = 0
        for i in range(1, len(path)):
            path_dist += line.distance(path[i].end_effector_pos, path[i-1].end_effector_pos)
        total_dist += path_dist

    return total_dist / len(paths)

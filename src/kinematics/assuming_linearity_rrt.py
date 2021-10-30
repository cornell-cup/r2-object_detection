"""
Written by Simon Kapen, Fall 2021.
"""

import math
import numpy as np
import pure_rrt_angles as rrt
from rrtgraph import Graph
from rrtnode import RRTNode
import line
import random
import collision_detection as cd
from precision_arm import PrecisionArm

arm = PrecisionArm()


# TODO: calculate path based on correct way to go to avoid no-go zone, not shortest way to go.
def compute_step_sizes(start_angles, end_angles, num_iter, arm):
    """ Computes each arm angle's step size based on how long it needs to travel to go from the start to end pose.
        Strategy for bounding angles: split into two quadrants: (1) less than the first bound,
                                                                (2) greater than the second bound.
        If both are in the same quadrant, use the closest angle distance as usual.
        If start_angles[i] in (1) and end_angles[i] in (2) and angle distance > 0, we must go the other way.
        If start_angles[i] in (2) and end_angles[i] in (1) and angle distance < 0, we must go the other way."""

    step_sizes = []
    true_angle_distances = rrt.true_angle_distances_arm(start_angles, end_angles)
    for i in range(len(start_angles)):
        if i >= len(arm.bounds) or arm.bounds[i] is None:
            angle_distance = end_angles[i] - start_angles[i]
            step_size = angle_distance / num_iter
            if angle_distance > math.pi:
                new_angle_dist = (2 * math.pi - end_angles[i] + start_angles[i])
                step_size = -new_angle_dist / num_iter
            elif angle_distance < -math.pi:
                new_angle_dist = (2 * math.pi - start_angles[i] + end_angles[i])
                step_size = new_angle_dist / num_iter
        else:
            bound_1 = arm.bounds[i][0]
            bound_2 = arm.bounds[i][1]
            th_1 = start_angles[i]
            th_2 = end_angles[i]
            angle_dist = true_angle_distances[i]

            if th_1 < bound_1 and th_2 > bound_2:
                if true_angle_distances[i] > 0:
                    angle_dist = -(2 * math.pi - true_angle_distances[i])

            if th_2 < bound_1 and th_1 > bound_2:
                if true_angle_distances[i] < 0:
                    angle_dist = 2 * math.pi + true_angle_distances[i]

            # print("start: {a1}, end: {a2}, b1: {b1}, b2: {b2}, orig_dist: {d1}, dist: {dist}"
            #       .format(a1=start_angles[i], a2=end_angles[i], b1=bound_1, b2=bound_2,
            #               d1=true_angle_distances[i], dist=angle_dist))

            step_size = angle_dist / num_iter

        step_sizes.append(step_size)
    # print("STEP SIZES: {}".format(step_sizes))
    return step_sizes


def generate_linear_path(start_angles, end_angles, num_iter):
    """ Return a path of nodes from start_pos to end_pos, with [num_iter] iterations of equal length. """
    step_sizes = compute_step_sizes(start_angles, end_angles, num_iter, arm)

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
        # if not arm.angles_within_bounds(new_angles[0:3]):
        #     print("not within bounds: {}".format(new_angles[0:3]))
        #     return g, False
        new_node = RRTNode(new_angles)

        new_idx = g.add_vex(new_node)

        dist = line.distance(new_node.end_effector_pos, current_node.end_effector_pos)

        g.add_edge(current_idx, new_idx, dist)

        current_angles = new_angles
        current_idx = new_idx
        current_node = new_node

        if i == num_iter - 1:
            # print("LAST ANGLES: {}".format(current_angles))
            # print("TARGET END ANGLES: {}".format(end_angles))

            rounded_current = np.around(current_angles, decimals=4)
            rounded_end = np.around(end_angles, decimals=4)

            if not np.array_equal(rounded_current, rounded_end):
                return g, False

    return g, True


def valid_path_configuration(pose, obstacles):
    nlinkarm = pose.to_nlinkarm()
    for obstacle in obstacles:
        if cd.arm_is_colliding(nlinkarm, obstacle):
            return False

    return rrt.valid_configuration(pose.angles)


def rrt_hd_tl(path, obstacles):
    # TODO: determine whether multiple RRTs is worthwhile for some cases
    """ Returns the longest start and end segments without a collision.
        Precondition: the start and end nodes are valid poses that do not collide with any obstacles. """
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
    """ Fills in the gap between two lists of arm segments with an RRT path. """
    n_iter = 10000
    radius = 0.01
    stepSize = .4
    threshold = 4

    rrt_start = path_hd[-1]
    rrt_end = path_tl[0]

    g = rrt.rrt(rrt_start.angles, rrt_end.angles, obstacles, n_iter, radius, stepSize, threshold)
    if g.success:
        print("rrt success :)")
    else:
        print("rrt failed :(")
    path_mid = rrt.dijkstra(g)
    if g is not None:
        return path_hd + path_mid + path_tl

    else:
        return None


def path_is_colliding(path, obstacles):
    """ Returns True if there is a node in [path] that collides with any obstacle in [obstacles]. """
    for obstacle in obstacles:
        for node in path:
            if cd.arm_is_colliding(node.to_nlinkarm(), obstacle):
                return True

    return False


def linear_rrt(start_angles, end_angles, num_iter, obstacles):
    """ Generates a linear path, and maneuvers around obstacles with RRT if necessary. """
    g = generate_linear_path(start_angles, end_angles, num_iter)
    linear_path = g[0].nodes

    new_path_hd, new_path_tl = rrt_hd_tl(linear_path, obstacles)
    # print("HEAD: {}".format(new_path_hd))
    # print("ORIGINAL PATH: {}".format(linear_path))
    if path_is_colliding(linear_path, obstacles):
        linear_path = replace_with_rrt(new_path_hd, new_path_tl, obstacles)

    return linear_path


def random_angle_config():
    """ Returns a set of random angles within a range of the goal state angles. """
    rand_angles = [0, 0, 0, 0, 0, 0]

    while True:
        for a in range(0, 6):
            # Random number from -2pi to 2pi
            rand_angles[a] = random.random() * 2 * math.pi

        if rrt.valid_configuration(rand_angles):
            return rand_angles

    return rand_angles


def linearity_test(num_trials):
    s_count = 0
    for i in range(num_trials):
        startpos = random_angle_config()

        endpos = random_angle_config()
        while not arm.angles_within_bounds(startpos) or not arm.angles_within_bounds(endpos):
            startpos = random_angle_config()
            endpos = random_angle_config()
        print("startpos: ", format(startpos))
        iterations = 10
        _, success = generate_linear_path(startpos, endpos, iterations)

        if success:
            s_count = s_count + 1

    return (s_count / num_trials) * 100


def plot_random_path(iterations, obstacles, arm):
    startpos = random_angle_config()
    endpos = random_angle_config()

    while not (arm.angles_within_bounds(startpos) and arm.angles_within_bounds(endpos)):
        startpos = random_angle_config()
        endpos = random_angle_config()

    path = linear_rrt(startpos, endpos, iterations, obstacles)

    g = Graph(startpos, endpos)

    for node in path:
        g.add_vex(node)
        print(node.angles)

    rrt.plot_3d(g, path, obstacles)


RRT_TEST_SEED = 1
CLEAR_PATH_TEST_SEED = 20
INCORRECT_END_SEED = 420

if __name__ == '__main__':
    #random.seed(a=INCORRECT_END_SEED)

    trials = 1000
    iterations = 10
    obstacles = [[0.1, -0.3, 0.15, 0.2]]
    # print("success rate in {t} trials: {r}".format(t=trials, r=linearity_test(trials)))

    plot_random_path(iterations, obstacles, arm)

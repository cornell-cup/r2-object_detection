from kinematics import kinematics
import math
import numpy as np
import pure_rrt_angles as rrt
from rrtgraph import Graph
from rrtnode import RRTNode
import line
import random


def generate_linear_path(start_angles, end_angles, num_iter):
    """ Return a path of nodes from start_pos to end_pos, with [num_iter] iterations of equal length. """
    step_sizes = []
    for i in range(len(start_angles)):
        angle_distance = end_angles[i] - start_angles[i]
        step_size = angle_distance / num_iter
        if angle_distance > math.pi:
            new_angle_dist = (2 * math.pi - end_angles[i] + start_angles[i])
            step_size = -new_angle_dist / num_iter
        elif angle_distance < -math.pi:
            new_angle_dist = (2 * math.pi - start_angles[i] + end_angles[i])
            step_size = new_angle_dist / num_iter

        step_sizes.append(step_size)

    g = Graph(start_angles, end_angles)

    current_angles = start_angles
    current_idx = 0
    current_node = g.nodes[0]
    for i in range(num_iter):
        if not rrt.valid_configuration(current_node.angles):
            #print("Iteration {} not valid".format(i))
            return g, False
        new_angles = np.add(current_angles, step_sizes)
        new_node = RRTNode(new_angles)

        new_idx = g.add_vex(new_node)

        dist = line.distance(new_node.end_effector_pos, current_node.end_effector_pos)

        g.add_edge(current_idx, new_idx, dist)

        current_angles = new_angles
        current_idx = new_idx
        current_node = new_node

    # print("start angles: {}".format(start_angles))
    # print("end angles: {}".format(end_angles))
    # print("current angles: {}".format(current_angles))
    return g, True


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

        iterations = 10
        _, success = generate_linear_path(startpos, endpos, iterations)

        if(success):
            s_count = s_count + 1

    return (s_count / num_trials) * 100


def plot_random_path(iterations):
    startpos = random_angle_config()

    endpos = random_angle_config()

    g, _ = generate_linear_path(startpos, endpos, iterations)
    path = g.nodes
    rrt.plot_3d(g, path, [])


if __name__ == '__main__':
    random.seed()

    trials = 10000
    iterations = 10
    print("success rate in {t} trials: {r}".format(t=trials, r=linearity_test(trials)))

    #plot_random_path(iterations)


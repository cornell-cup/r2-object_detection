"""
Implementation of an Optimal Minimum Cost algorithm for a 6DOF arm.

Given a start and end configuration, determines a collision-free path with the least possible distance traveled. This is
done by picking nodes that are close to the goal and trying to expand from them in multiple directions. It returns
a set of configurations which is passed to the ECEs.

Written by Simon Kapen and Raj Sinha, Spring 2022.
"""
from arm_node import Node
from arm_graph import Graph
import arm_plot
from util.angles import true_angle_distances_arm
import numpy as np
import matplotlib.pyplot as plt
from util import angles, line
import collision_detection
from pure_rrt import random_start_environment, dijkstra, arm_is_colliding
from arm_plot import plot_3d
import time


def visited(node, g):
    """ Determines whether the algorithm has already explored in the area of the node's end effector position. """
    for n in g.nodes:
        if np.array_equal(np.round(n.end_effector_pos, 3), np.round(node.end_effector_pos, 3)):
            return True

    return False


def expand(node, directions, step_size, g):
    """ Expands from a node in a certain amount of directions for length [step_size]. """
    for i in range(directions):
        new_angles_pos = list.copy(node.angles)
        new_angles_neg = list.copy(node.angles)

        new_angles_pos[i] += step_size
        new_angles_neg[i] -= step_size

        node_pos = Node(new_angles_pos)
        node_neg = Node(new_angles_neg)

        if not visited(node_pos, g) and node_pos.valid_configuration():
            g.add_vex(node_pos, node)

        if not visited(node_neg, g) and node_neg.valid_configuration():
            g.add_vex(node_neg, node)


def find_path(end_node, start_angles, obs, n_iter=200, step_size=.1):
    g = Graph(start_angles, end_node.angles)
    g.ranking.append(g.start_node)

    for i in range(n_iter):
        best_node = g.ranking.pop(0)

        if arm_is_colliding(best_node, obs):
            continue

        dist_to_goal = line.distance(best_node.end_effector_pos, end_node.end_effector_pos)
        print(dist_to_goal)
        if dist_to_goal < 0.03:
            g.end_node = Node.from_point(end_node.end_effector_pos, start_config=best_node.angles)
            g.add_vex(g.end_node, best_node)
            # g.end_node = best_node
            g.success = True
            # print(print(dist_to_goal))
            return g

        # if dist_to_goal < step_size and step_size > .001:
        #     step_size /= 2
        #     print(step_size)

        expand(best_node, 3, step_size, g)

    return g


if __name__ == "__main__":
    obstacle_bounds = [[-.4, .4], [-.2, .4], [-.4, .4]]
    print("time started")
    start_time = time.time()
    start_node, end_node, obstacles = random_start_environment(5, obstacle_bounds, obstacle_size=.4)
    start_angles = start_node.angles
    end_pos = end_node.end_effector_pos

    g = find_path(end_node, start_node.angles, obstacles)

    # start_angles = [5.369371017085593, 1.170449817096421, 1.2196071198743046, 0.5373588543596212, 4.749227044258129]
    # end_angles = [5.79216011, 0.58102332, -0.42800113, 0.15302296, 6.77421087]
    # obstacles = [
    #     [0.3208665645587594, 0.37814481229761404, 0.23882439366195474, 0.016853955529503174, 0.17582898559826995,
    #      0.16858049170914127],
    #     [0.18775580336670283, 0.24715717642614093, 0.1740361031691925, 0.37310350982500556, 0.10699879554315049,
    #      0.39210221856095057],
    #     [0.367780650270748, 0.2385358011780404, 0.30517239147997577, 0.03582676773275471, 0.15362943031993448,
    #      0.08434869814078888],
    #     [0.316037333704863, 0.22237310777884473, -0.04340689128060243, 0.23796543278086038, 0.33430711939413205,
    #      0.3141177486229645],
    #     [-0.3206491982198045, 0.28909375117870534, -0.1589868186492286, 0.24619332779849118, 0.007638981080338692,
    #      0.08744967731817717]]
    #
    # g = find_path(Node(end_angles), start_angles, obstacles)
    if g.success:
        path = dijkstra(g)
        print("\nTime taken: ", (time.time() - start_time))
        plot_3d(Graph(start_angles, end_node.angles), path, obstacles)
    else:
        print("\nTime taken: ", (time.time() - start_time))
        print("Path not found. :(")
        print(start_angles)
        print(end_node.angles)
        print(obstacles)
        plot_3d(g, [Node(start_angles), end_node], obstacles)

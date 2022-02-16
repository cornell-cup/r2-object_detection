"""
Implementation of the Rapidly-Exploring Random Tree algorithm using precision arm configurations as nodes.

Given a start configuration and end configuration received from the ECE subteam, and a set of obstacles received from
the object detection algorithm, the algorithm attempts to find a collision-free path from the start configuration to the
end configuration. This is done by growing a tree pseudo-randomly, and returning a set of configurations which is then
passed to the ECE subteam.

Written by Simon Kapen and Alison Duan, Spring 2021.
Dijkstra algorithm and RRTGraph structure adapted from Fanjin Zeng on github, 2019.
"""

import math
import numpy as np
from random import random
from collections import deque
import time
import random
import collision_detection
import line
from rrtnode import RRTNode
from rrtgraph import Graph
from rrtplot import plot_3d
import kinpy as kp
import obstacle_generation


def arm_is_colliding(node: RRTNode, obstacles):
    """Checks if an arm configuration is colliding with any obstacle in the c-space.

    Args:
        node: An instance of rrtnode.RRTNode.
        obstacles: An array of float arrays representing obstacles.
    """

    for obs in obstacles:
        if collision_detection.arm_is_colliding(node, obs):
            return True
    return False


def nearest(g: Graph, node: RRTNode):
    """ Finds the nearest node to the input node in cartesian space, by end effector position.

     Returns:
         An instance of the nearest node to the input node, as well as its index in the hashtable of the input graph.
     """

    nearest_node = None
    nearest_node_index = None
    min_dist = float("inf")

    neighbors = enumerate(g.nodes)
    # if len(g.nodes) > 100:
    #     neighbors = g.spatial_hash.closest_neighbors(node)
    
    for idx, v in neighbors:
        #print(v)
        dist = line.distance(v.end_effector_pos, node)
        if dist < min_dist:
            min_dist = dist
            nearest_node_index = idx
            nearest_node = v

    return nearest_node, nearest_node_index


def steer(rand_angles, near_angles, step_size):
    """Generates a new node a certain distance along the path between the nearest node to the random node.

    Args:
        rand_angles: The angles of the randomly generated node.
        near_angles: The angles of the node closest to the randomly generated node.
        step_size: The distance from the nearest node for the new node to be generated.

    Returns:
        An instance of RRTNode representing the new node of the tree.
    """

    dirn = true_angle_distances_arm(np.array(near_angles), np.array(rand_angles))
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min(step_size, length)

    new_angles = (near_angles[0] + dirn[0], near_angles[1] + dirn[1], near_angles[2] + dirn[2],
                  near_angles[3] + dirn[3], near_angles[4] + dirn[4])
    return RRTNode(new_angles)


def extend_heuristic(g: Graph, rand_node: RRTNode, step_size: float, threshold: int, obstacles):
    """Extends RRT T from the node closest to the end node, in the direction of rand_node. If the node
    being extended from has failed too many times (generates an colliding configuration or does not get closer
    to the end node), it is removed from the ranking.

    Arguments:
        g: A rrtgraph.Graph instance.
        rand_node: An RRTNode instance representing the randomly generated node.
        step_size: The distance from the nearest node for the new node to be generated.
        threshold: The maximum amount of extension failures per node.
        obstacles: An array of float arrays representing obstacles.
    """
    near_node = g.ranking[0]
    new_node = steer(rand_node.angles, near_node.angles, step_size)

    if g.dist_to_end(new_node) < g.dist_to_end(near_node) and not arm_is_colliding(new_node, obstacles):
        nearest_to_new, nearest_to_new_idx = nearest(g, new_node.end_effector_pos)

        if arm_is_colliding(new_node, obstacles):
            raise Exception("Adding a colliding node")
        newidx = g.add_vex(new_node)
        dist = line.distance(new_node.end_effector_pos, nearest_to_new.end_effector_pos)
        g.add_edge(newidx, nearest_to_new_idx, dist)
        return new_node, g.node_to_index[new_node]

    near_node.inc_fail_count()
    near_idx = g.node_to_index[near_node]
    if near_node.fail_count > threshold:
        g.ranking.remove(near_node)

        parent = g.get_parent(near_idx)
        parent.inc_fail_count()

    return near_node, near_idx


def valid_configuration(angles):
    """ Returns true if the given angle configuration is a valid one. """

    link_lengths = [.222, .3]
    #  for a given a1, an a2 is always valid. However, the a3 is not necessarily valid:
    #  use spherical coordinates for validity
    if link_lengths[0] * math.cos(angles[1]) < 0:
        return False

    if link_lengths[1] * math.cos(angles[2]) + link_lengths[0] * math.cos(angles[1]) < 0:
        return False

    return True, [(angles[0] + math.pi) % math.pi, (angles[1] + math.pi) % math.pi, \
           (angles[2] + math.pi) % math.pi, (angles[3] + math.pi) % math.pi, \
           (angles[4] + math.pi) % math.pi]


def random_angle_config():
    """ Returns a set of random angles within a range of the goal state angles. """
    rand_angles = [0, 0, 0, 0, 0]

    while True:
        for a in range(0, 5):
            # Random number from -2pi to 2pi
            rand_angles[a] = (random.random() * 2 - 1) * 2 * np.pi

        if valid_configuration(rand_angles):
            return rand_angles

    return rand_angles


def true_angle_distance(angle_1, angle_2):
    """ Returns [angle_2] - [angle_1], accounting for angle wrap.
            Example: true_angle_distance(PI/6, 11PI/6) is PI/3, not 5PI/3. """

    difference = angle_2 - angle_1
    if difference > math.pi:
        difference = -(2 * math.pi - angle_2 + angle_1)
    elif difference < -math.pi:
        difference = 2 * math.pi - angle_1 + angle_2

    return difference


def true_angle_distances_arm(angles_1, angles_2):
    """ Returns [angles_2] - [angles_1], accounting for angle wrap. Calculates each angle in an arm configuration.
        Example: true_angle_distance(PI/6, 11PI/6) is PI/3, not 5PI/3. """
    new_angles = []

    for angle1, angle2 in zip(angles_1, angles_2):
        new_angles.append(true_angle_distance(angle1, angle2))

    return new_angles


def rrt(start_angles, end_angles, obstacles, n_iter=300, radius=0.02, angle_threshold=2, stepSize=.05, heuristic_threshold=10):
    """Uses the RRT algorithm to determine a collision-free path from start_angles to end_angles.

    Args:
        start_angles: An array of length 5 representing the initial angle configuration of the arm.
        end_angles: An array of length 5 representing the desired angle configuration.
        obstacles: An array of float arrays representing cube obstacles.
        n_iter: Maximum number of iterations to find a path.
        radius: Maximum distance between the final end effector position and the second-to-last position in a path.
        angle_threshold: Maximum distance between the final angles and the second-to-last angles in a path.
        stepSize: Distance between nodes in the graph.
        heuristic_threshold: Maximum number of times a new node can fail to expand from any given node in the graph.

    Returns:
        An instance of rrtgraph.Graph containing a list of instances of RRTNode, and a path of RRTNode instances between
        the start and end nodes, if successful.
        A boolean indicator representing whether a path was found.
    """
    G = Graph(start_angles, end_angles)

    for i in range(n_iter):
        rand_node = RRTNode(random_angle_config())
        if arm_is_colliding(rand_node, obstacles):
            continue

        if i % 2 == 0 or not G.ranking:
            nearest_node, nearest_node_index = nearest(G, rand_node.end_effector_pos)
            if nearest_node is None:
                continue

            new_node = steer(rand_node.angles, nearest_node.angles, stepSize)

            if arm_is_colliding(new_node, obstacles):
                continue

            nearest_to_new, nearest_to_new_idx = nearest(G, new_node.end_effector_pos)

            if arm_is_colliding(new_node, obstacles):
                raise Exception("Adding a colliding node")

            newidx = G.add_vex(new_node)
            dist = line.distance(new_node.end_effector_pos, nearest_to_new.end_effector_pos)
            G.add_edge(newidx, nearest_to_new_idx, dist)

        else:
            new_node, newidx = extend_heuristic(G, rand_node, stepSize, heuristic_threshold, obstacles)
            if arm_is_colliding(new_node, obstacles):
                continue

        end_eff_dist_to_goal = line.distance(new_node.end_effector_pos, G.end_node.end_effector_pos)
        angle_dist_to_goal = np.linalg.norm(true_angle_distances_arm(new_node.angles, G.end_node.angles))

        if end_eff_dist_to_goal < radius and not G.success:
            if arm_is_colliding(G.end_node, obstacles):
                raise Exception("Adding a colliding node")

            if angle_dist_to_goal > angle_threshold:
                # create new end configuration with inverse kinematics
                G.end_node = RRTNode.from_point(G.end_node.end_effector_pos, start_config=new_node.angles)

            endidx = G.add_vex(G.end_node)
            G.add_edge(newidx, endidx, end_eff_dist_to_goal)
            G.success = True
            print("")
            break

    return G


def dijkstra(G):
    """
    Dijkstra algorithm for finding shortest path from start position to end.
    """
    srcIdx = G.node_to_index[G.start_node]
    dstIdx = G.node_to_index[G.end_node]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.nodes[curNode])
        curNode = prev[curNode]
    path.appendleft(G.nodes[curNode])
    return list(path)


def rrt_graph_list(num_trials, n_iter, radius, step_size, threshold, num_obstacles=7, bounds=[.4, .4, .4]):
    """ Generates a list of RRT graphs. """
    graphs = []
    for i in range(0, num_trials):
        obstacles = obstacle_generation.generate_random_obstacles(num_obstacles, bounds)
        print("Trial: ", i)
        start_node = RRTNode(configuration=None)
        end_node = RRTNode(configuration=None)

        while arm_is_colliding(start_node, obstacles):
            obstacles = obstacle_generation.generate_random_obstacles(num_obstacles, bounds)

        while arm_is_colliding(end_node, obstacles):
            end_node = RRTNode(None)

        if arm_is_colliding(end_node, obstacles):
            raise Exception("Approved a colliding node")

        G = rrt(start_node.angles, end_node.angles, obstacles, n_iter=n_iter, radius=radius, stepSize=step_size,
                heuristic_threshold=threshold)
        graphs.append(G)

    return graphs


def avg_nodes_test(graphs: list[Graph]):
    """ The average amount of nodes generated until the end goal is reached. """
    total_nodes = 0
    for i in range(0, len(graphs)):
        total_nodes += len(graphs[i].nodes)

    return total_nodes/len(graphs)


def converge_test(graphs: list[Graph]):
    """ Counts the amount of successes for a list of graphs. """
    num_success = 0
    for i in range(0, len(graphs)):
        if graphs[i].success:
            num_success += 1

    return num_success


if __name__ == '__main__':
    random.seed()
    startpos = (0., 0., 0., 0., 0.)

    x, y, z = (.2, -.2, -.2)

    endpos = (1.8756746293234707, 0.24887138496377703, 0.33744701903541446, 0.15153332538250205, 0)

    print(valid_configuration(endpos))

    print("endpos: {}".format(endpos))

    # obstacles = [[-0.3, 0.0, 0.15, 0.2, 0.2, 0.2]]
    obstacles = obstacle_generation.generate_random_obstacles(5, [.4, .4, .4])
    print(obstacles)
    n_iter = 1000
    radius = .07
    stepSize = .4
    threshold = 2
    start_time = time.time()
    print("RRT started")
    # G = rrt(startpos, endpos, obstacles, n_iter, radius, stepSize=stepSize)
    #
    # if G.success:
    #     path = dijkstra(G)
    #     print("\nTime taken: ", (time.time() - start_time))
    #     plot_3d(G, path, obstacles)
    # else:
    #     print("\nTime taken: ", (time.time() - start_time))
    #     print("Path not found. :(")
    #     plot_3d(G, [], obstacles)

    trials = 500
    graphs = rrt_graph_list(trials, n_iter, radius, stepSize, threshold)

    print("Average nodes generated: ", avg_nodes_test(graphs))
    print("Num. successes: ", converge_test(graphs))
    total_time = time.time() - start_time
    print("Time taken: ", total_time)
    print("Average time per graph: ", total_time / trials)

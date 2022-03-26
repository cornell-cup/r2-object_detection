"""
Implementation of the Rapidly-Exploring Random Tree algorithm using precision arm configurations as nodes.

Given a start configuration and end configuration received from the ECE subteam, and a set of obstacles received from
the object detection algorithm, the algorithm attempts to find a collision-free path from the start configuration to the
end configuration. This is done by growing a tree pseudo-randomly, and returning a set of configurations which is then
passed to the ECE subteam.

Written by Simon Kapen and Alison Duan, Spring 2021.
Dijkstra algorithm and Graph structure adapted from Fanjin Zeng on github, 2019.
"""

import math
import numpy as np
from random import random
from collections import deque
import time
import random
import collision_detection
from util import line
from arm_node import Node
from arm_graph import Graph
import obstacle_generation
from util.angles import true_angle_distances_arm
import sys

def arm_is_colliding(node: Node, obstacles):
    """Checks if an arm configuration is colliding with any obstacle in the c-space.

    Args:
        node: An instance of arm_node.Node.
        obstacles: An array of float arrays representing obstacles.
    """
    print(node.joint_positions)
    print(obstacles)
    for obs in obstacles:
        if collision_detection.arm_is_colliding_prism(node, obs):
            return True
    return False


def nearest(g: Graph, node: Node):
    """ Finds the nearest node to the input node in Cartesian space, 
        by end effector position.

     Returns:
        An instance of the nearest node to the input node, as well as 
        its index in the hashtable of the input graph.
    """
    # print(g.end_effectors)
    nearest_node_index = np.argmin(np.sum(np.square(g.end_effectors - node), axis=1))

    return g.nodes[nearest_node_index], nearest_node_index


def steer(rand_angles, near_angles, step_size):
    """Generates a new node a certain distance along the path between the nearest node to the random node.

    Args:
        rand_angles: The angles of the randomly generated node.
        near_angles: The angles of the node closest to the randomly generated node.
        step_size: The distance from the nearest node for the new node to be generated.

    Returns:
        An instance of Node representing the new node of the tree.
    """

    dirn = true_angle_distances_arm(np.array(near_angles), np.array(rand_angles))
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min(step_size, length)

    new_angles = (near_angles[0] + dirn[0], near_angles[1] + dirn[1], near_angles[2] + dirn[2],
                  near_angles[3] + dirn[3], near_angles[4] + dirn[4])
    return Node(new_angles)


def extend_heuristic(g: Graph, rand_node: Node, step_size: float, threshold: int, obstacles):
    """Extends RRT T from the node closest to the end node, in the direction of rand_node. If the node
    being extended from has failed too many times (generates an colliding configuration or does not get closer
    to the end node), it is removed from the ranking.

    Arguments:
        g: A rrtgraph.Graph instance.
        rand_node: An Node instance representing the randomly generated node.
        step_size: The distance from the nearest node for the new node to be generated.
        threshold: The maximum amount of extension failures per node.
        obstacles: An array of float arrays representing obstacles.
    """
    near_node = g.ranking[0]
    new_node = steer(rand_node.angles, near_node.angles, step_size)

    if g.dist_to_end(new_node) < g.dist_to_end(near_node) and not arm_is_colliding(new_node, obstacles) \
            and new_node.valid_configuration():
        nearest_to_new, nearest_to_new_idx = nearest(g, new_node.end_effector_pos)

        if arm_is_colliding(new_node, obstacles):
            raise Exception("Adding a colliding node")
        g.add_vex(new_node, nearest_to_new)
        # dist = line.distance(new_node.end_effector_pos, nearest_to_new.end_effector_pos)
        # g.add_edge(newidx, nearest_to_new_idx, dist)
        return new_node, g.node_to_index[new_node]

    near_node.inc_fail_count()
    near_idx = g.node_to_index[near_node]
    if near_node.fail_count > threshold:
        g.ranking.remove(near_node)

        parent = g.get_parent(near_idx)
        parent.inc_fail_count()

    return near_node, near_idx


# Possibly archive/delete. This is unused right now
def valid_configuration(angles):
    """ Returns true if the given angle configuration is a valid one. """
    print(angles)
    link_lengths = [.222, .3]
    #  for a given a1, an a2 is always valid. However, the a3 is not necessarily valid:
    #  use spherical coordinates for validity
    if link_lengths[0] * math.cos(angles[1]) < 0:
        print('invalid condition 1')
        return False

    if link_lengths[1] * math.cos(angles[2]) + link_lengths[0] * math.cos(angles[1]) < 0:
        print('invalid condition 2')
        return False

    return True, [(angles[0] + math.pi) % math.pi, (angles[1] + math.pi) % math.pi, \
                  (angles[2] + math.pi) % math.pi, (angles[3] + math.pi) % math.pi, \
                  (angles[4] + math.pi) % math.pi]




def rrt(start_angles, end_angles, obstacles, n_iter=300, radius=0.02, angle_threshold=1, stepSize=.05,
        heuristic_threshold=10):
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
        An instance of rrtgraph.Graph containing a list of instances of Node, and a path of Node instances between
        the start and end nodes, if successful.
        A boolean indicator representing whether a path was found.
    """
    G = Graph(start_angles, end_angles)

    for i in range(n_iter):
        rand_node = Node(None)
        if arm_is_colliding(rand_node, obstacles):
            continue

        if i % 2 == 0 or not G.ranking:
            nearest_node, nearest_node_index = nearest(G, rand_node.end_effector_pos)
            if nearest_node is None:
                continue

            new_node = steer(rand_node.angles, nearest_node.angles, stepSize)

            if arm_is_colliding(new_node, obstacles) or not new_node.valid_configuration():
                continue

            nearest_to_new, _ = nearest(G, new_node.end_effector_pos)

            G.add_vex(new_node, nearest_to_new)
            # dist = line.distance(new_node.end_effector_pos, nearest_to_new.end_effector_pos)
            # G.add_edge(newidx, nearest_to_new_idx, dist)

        else:
            new_node, newidx = extend_heuristic(G, rand_node, stepSize, heuristic_threshold, obstacles)
            if arm_is_colliding(new_node, obstacles):
                continue

        end_eff_dist_to_goal = line.distance(new_node.end_effector_pos, G.end_node.end_effector_pos)
        angle_dist_to_goal = np.linalg.norm(true_angle_distances_arm(new_node.angles, G.end_node.angles))

        if end_eff_dist_to_goal < radius and not G.success:
            if arm_is_colliding(G.end_node, obstacles):
                raise Exception("Adding a colliding node")
            desired_position = G.end_node.end_effector_pos
            if angle_dist_to_goal > angle_threshold:
                # tries to get a closer arm configuration to the second-to-last arm configration with inverse kinematics
                G.end_node = Node.from_point(desired_position, start_config=new_node.angles)

            endidx = G.add_vex(G.end_node, new_node)
            # G.add_edge(newidx, endidx, end_eff_dist_to_goal)
            G.success = True
            print("Iterations:", i)
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


def random_start_environment(num_obstacles, bounds, obstacle_size=.2):
    """Generates a start environment for a run of RRT.

     Returns:
         An Node representing a valid start configuration.
         An Node representing a valid end configuration.
         A set of [num_obstacles] obstacles that do not collide with the start or end configurations.
    """

    random_start_node = Node(configuration=None)
    random_end_node = Node.from_point([random.uniform(bounds[0][0], bounds[0][1]),
                                          random.uniform(bounds[1][0], bounds[1][1]),
                                          random.uniform(bounds[2][0], bounds[2][1])],
                                         random_start_node.angles)

    max_tries = 10
    tries = 1
    while not random_end_node.valid_configuration():
        random_end_node = Node.from_point([random.uniform(bounds[0][0], bounds[0][1]),
                                              random.uniform(bounds[1][0], bounds[1][1]),
                                              random.uniform(bounds[2][0], bounds[2][1])], random_start_node.angles)
        tries += 1
        if tries > max_tries:
            return None, None, None

    current_obstacles = obstacle_generation.generate_random_obstacles(num_obstacles, bounds,
                                                                      max_side_length=obstacle_size)
    while arm_is_colliding(random_end_node, current_obstacles):
        current_obstacles = obstacle_generation.generate_random_obstacles(num_obstacles, bounds,
                                                                          max_side_length=obstacle_size)

    while arm_is_colliding(random_start_node, current_obstacles) or not random_start_node.valid_configuration():
        random_start_node = Node(None)

    print("start angles:", random_start_node.angles)
    print("end angles:", random_end_node.angles)
    print("obstacles:", current_obstacles)

    return random_start_node, random_end_node, current_obstacles


def rrt_graph_list(num_trials, n_iter, radius, step_size, threshold, bounds, num_obstacles=1):
    """ Generates a list of RRT graphs. """
    print("RUNNING {t} TRIALS OF RRT WITH {o} OBSTACLES\n".format(t=num_trials, o=num_obstacles))
    graphs = []
    failed_obstacles = []
    for i in range(0, num_trials):
        trial_start_time = time.time()

        print("Trial: ", i + 1)
        current_start_node, current_end_node, random_obstacles = random_start_environment(num_obstacles, bounds)
        if current_start_node is None:
            continue
        if not current_start_node.valid_configuration():
            raise Exception("Approved an invalid start node")


        if arm_is_colliding(current_end_node, random_obstacles):
            raise Exception("Approved a colliding node")

        G = rrt(current_start_node.angles, current_end_node.angles, random_obstacles, n_iter=n_iter, radius=radius,
                stepSize=step_size,
                heuristic_threshold=threshold)

        if G.success:
            print("SUCCESS")
        else:
            print("FAIL")

        print("Trial time:", time.time() - trial_start_time)
        print("")
        graphs.append(G)
        failed_obstacles.append(random_obstacles)

    return graphs, failed_obstacles


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


def print_failed_cases(graphs: list[Graph], failed_obstacles):
    """ Outputs failed start angles, end angles, and obstacles for a list of RRT graphs. """
    print("FAILED CASES")

    for i in range(0, len(graphs)):
        if not graphs[i].success:
            print("start_angles =", graphs[i].start_node.angles)
            ea = graphs[i].end_node.angles
            print("end_angles = [{ea0}, {ea1}, {ea2}, {ea3}, {ea4}]"
                  .format(ea0=ea[0], ea1=ea[1], ea2=ea[2], ea3=ea[3], ea4=ea[4]))
            print("obstacles =", failed_obstacles[i])
            print()


if __name__ == '__main__':
    random.seed()

    # obstacles = []
    n_iter = 1000
    radius = .07
    stepSize = .35
    threshold = 2
    num_obstacles = 3
    obstacle_bounds = [[-.4, .4], [-.2, .4], [-.4, .4]]
    start_node, end_node, obstacles = random_start_environment(num_obstacles, obstacle_bounds, obstacle_size=.25)
    # obstacles = []
    start_time = time.time()
    print("RRT started")

    # G = rrt(start_node.angles,
    #         end_node.angles,
    #         obstacles,
    #         n_iter, radius, stepSize=stepSize)
    #
    # if G.success:
    #     path = dijkstra(G)
    #     print("\nTime taken: ", (time.time() - start_time))
    #     plot_3d(Graph(start_node.angles, end_node.angles), path, obstacles)
    # else:
    #     print("\nTime taken: ", (time.time() - start_time))
    #     print("Path not found. :(")
    #     plot_3d(G, [start_node, end_node], obstacles)

    goal_end_effector_bounds = [[-.4, .4], [.05, .4], [-.4, .4]]
    trials = 30
    graphs, failing_obstacles = rrt_graph_list(trials, n_iter, radius, stepSize, threshold,
                                               goal_end_effector_bounds, num_obstacles)
    num_successes = converge_test(graphs)
    print_failed_cases(graphs, failing_obstacles)
    print("Average nodes generated: ", avg_nodes_test(graphs))
    print("Num. successes: ", num_successes)
    print("Convergence rate: ", num_successes / trials)
    total_time = time.time() - start_time
    print("Time taken: ", total_time)
    print("Average time per graph: ", total_time / trials)

"""
Implementation of the Rapidly-Exploring Random Tree algorithm using precision arm configurations as nodes.

Given a start configuration and end configuration received from the ECE subteam, and a set of obstacles received from
the object detection algorithm, the algorithm attempts to find a collision-free path from the start configuration to the
end configuration. This is done by growing a tree pseudo-randomly, and returning a set of configurations which is then
passed to the ECE subteam.

Written by Simon Kapen '24 and Alison Duan '23, Spring 2021.
Dijkstra algorithm adapted from Fanjin Zeng on github, 2019 (gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80).
"""

import math
import numpy as np
from random import random
from collections import deque
import time
import random

import sys

try: 
	from collision_detection import arm_is_colliding_prisms, arm_is_colliding
	from optimizers import path_optimizer_two, path_optimizer_four, checkPath
	import obstacle_generation
	from util import line
	from arm_node import Node
	from arm_graph import Graph
	from matplotlib.widgets import Button
	from arm_plot import plot_3d
	from obstacle_generation import random_start_environment
	from util.angles import true_angle_distances_arm
except: 
	from src.kinematics.collision_detection import arm_is_colliding_prisms, arm_is_colliding
	from src.kinematics.optimizers import path_optimizer_two, path_optimizer_four, checkPath
	from src.kinematics import obstacle_generation
	from src.kinematics.util import line
	from src.kinematics.arm_node import Node
	from src.kinematics.arm_graph import Graph
	from matplotlib.widgets import Button
	from src.kinematics.arm_plot import plot_3d
	from src.kinematics.obstacle_generation import random_start_environment
	from src.kinematics.util.angles import true_angle_distances_arm

def nearest(g: Graph, target_end_effector_pos):
    """ Finds the nearest node to the input node in Cartesian space, 
        by end effector position.

     Returns:
        An instance of the nearest node to the input node, as well as 
        its index in the hashtable of the input graph.
    """
    # print(g.end_effectors)
    nearest_node_index = np.argmin(np.sum(np.square(g.end_effectors - target_end_effector_pos), axis=1))

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

    if g.dist_to_end(new_node) < g.dist_to_end(near_node) and not arm_is_colliding_prisms(new_node, obstacles) \
            and new_node.valid_configuration():
        nearest_to_new, nearest_to_new_idx = nearest(g, new_node.end_effector_pos)

        if arm_is_colliding_prisms(new_node, obstacles):
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
        if arm_is_colliding_prisms(rand_node, obstacles):
            continue

        if i % 2 == 0 or not G.ranking:
            nearest_node, nearest_node_index = nearest(G, rand_node.end_effector_pos)
            if nearest_node is None:
                continue

            new_node = steer(rand_node.angles, nearest_node.angles, stepSize)

            if arm_is_colliding_prisms(new_node, obstacles) or not new_node.valid_configuration():
                continue

            nearest_to_new, _ = nearest(G, new_node.end_effector_pos)

            G.add_vex(new_node, nearest_to_new)
            # dist = line.distance(new_node.end_effector_pos, nearest_to_new.end_effector_pos)
            # G.add_edge(newidx, nearest_to_new_idx, dist)

        else:
            new_node, newidx = extend_heuristic(G, rand_node, stepSize, heuristic_threshold, obstacles)
            if arm_is_colliding_prisms(new_node, obstacles):
                continue

        end_eff_dist_to_goal = line.distance(new_node.end_effector_pos, G.end_node.end_effector_pos)
        angle_dist_to_goal = np.linalg.norm(true_angle_distances_arm(new_node.angles, G.end_node.angles))

        if end_eff_dist_to_goal < radius and not G.success:
            if arm_is_colliding_prisms(G.end_node, obstacles):
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


def dijkstra(G, target_node=None):
    """
    Dijkstra algorithm for finding shortest path from start position to end.
    """
    srcIdx = G.node_to_index[G.start_node]
    dstIdx = G.node_to_index[G.end_node]

    if target_node is not None:
        dstIdx = G.node_to_index[target_node]

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

    #print("start angles:", random_start_node.angles)
    #print("end angles:", random_end_node.angles)
    #print("obstacles:", current_obstacles)

    return random_start_node, random_end_node, current_obstacles

def rrt_graph_list(num_trials, n_iter, radius, step_size, threshold, bounds, num_obstacles=1):
    """ Generates a list of RRT graphs. """
    print("RUNNING {t} TRIALS OF RRT WITH {o} OBSTACLES\n".format(t=num_trials, o=num_obstacles))
    graphs = []
    generated_obstacles = []
    for i in range(0, num_trials):
        trial_start_time = time.time()

        print("Trial: ", i + 1)
        current_start_node, current_end_node, random_obstacles = random_start_environment(num_obstacles, bounds)
        if current_start_node is None:
            continue
        if not current_start_node.valid_configuration():
            raise Exception("Approved an invalid start node")

        if arm_is_colliding_prisms(current_end_node, random_obstacles):
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
        generated_obstacles.append(random_obstacles)

    return graphs, generated_obstacles
def test():
    total_time_two = 0
    total_time_four = 0
    start = time.time()
    runs = 1000
    for i in range(1000):
        print('run')
        n_iter = 2000
        radius = .01
        stepSize = .35
        threshold = 2
        num_obstacles = 1
        bounds = [[-.05, .05], [-.05, .05], [-.05, .05]]
        # start_node = RRTNode([7.4883959080999105, -0.9802836168249124, 2.7119532197892307, 2.690692578970348, 1.4327288698060625])
        # end_node = RRTNode([0.80873032,  0.58529255 , 1.57082885 , 2.15507481 ,-0.80873048])
        start_node, end_node, obstacles, _ = random_start_environment(num_obstacles, bounds)
        location = random.uniform(.1, .1)
        prism = [location, location, location, .2, .2, .2]
        obstacles = [prism]
        start_time = time.time()
        print("RRT started")

        try:
            G = rrt(start_node.angles,
                end_node.angles,
                obstacles,
                n_iter, radius, stepSize=stepSize)
            size1 = 0
            if G.success:
                path = dijkstra(G)
                size1 = len(path)
                runTime = time.time() - start_time

                optimize_start_time2 = time.time()
                path2 = path_optimizer_two(path, prism)
                size2 = len(path2)
                if checkPath(path2, prism):
                    optimizeTime2 = time.time() - optimize_start_time2
                    total_time_two += optimizeTime2

                optimize_start_time4 = time.time()
                path4 = path_optimizer_four(path, prism)
                size4 = len(path4)
                if checkPath(path4, prism):
                    optimizeTime4 = time.time() - optimize_start_time4
                    total_time_four += optimizeTime4
        except Exception:
            print("Exception thrown")
            runs -= 1

    full_runtime = time.time() - start
    print("total time for 2s", total_time_two)
    print("total time for 4s", total_time_four)
    print("total runs", runs)
    print("time per run for 2s", total_time_two/runs)
    print("time per run for 4s", total_time_four/runs)
    print("full run time:", full_runtime)

def optimize(path,prism):
    collision = False
    lastPath = path
    newPath = None
    while not collision:
        newPath = path_optimizer_two(lastPath, prism)
        if newPath == lastPath:
            break
        lastPath = newPath
    return newPath

def multiple_runs():
    n_iter = 1000
    radius = .07
    stepSize = .35
    threshold = 2
    num_obstacles = 1
    bounds = [[-.4, .4], [0, .4], [-.4, .4]]
    # start_node = RRTNode([7.4883959080999105, -0.9802836168249124, 2.7119532197892307, 2.690692578970348, 1.4327288698060625])
    # end_node = RRTNode([0.80873032,  0.58529255 , 1.57082885 , 2.15507481 ,-0.80873048])
    start_node, end_node, obstacles = random_start_environment(num_obstacles, bounds)
    location = random.uniform(.1, .1)
    prism = [location, location, location, .2, .2, .2]
    obstacles = [prism]
    start_time = time.time()
    print("RRT started")

    G = rrt(start_node.angles,
            end_node.angles,
            obstacles,
            n_iter, radius, stepSize=stepSize)
    size1 = 0
    if G.success:
        collision = False
        path = dijkstra(G)
        size1 = len(path)
        print("Original Path Size:", size1)
        plot_3d(G,path,obstacles,None)
        bestPath = optimize(path, prism)
        print("Optimal Path Size:", len(bestPath))
        plot_3d(G,bestPath,obstacles,None)
    else:
        print("Path not found. :(")
        plot_3d(G, [start_node, end_node], obstacles, None)

if __name__ == '__main__':
    #test()
    multiple_runs()
    # n_iter = 1000
    # radius = .07
    # stepSize = .35
    # threshold = 2
    # num_obstacles = 1
    # bounds = [[-.4, .4], [0, .4], [-.4, .4]]
    # # start_node = RRTNode([7.4883959080999105, -0.9802836168249124, 2.7119532197892307, 2.690692578970348, 1.4327288698060625])
    # # end_node = RRTNode([0.80873032,  0.58529255 , 1.57082885 , 2.15507481 ,-0.80873048])
    # start_node, end_node, obstacles = random_start_environment(num_obstacles, bounds)
    # location = random.uniform(.1, .1)
    # prism = [location, location, location, .2, .2, .2]
    # obstacles = [prism]
    # start_time = time.time()
    # print("RRT started")
    #
    # G = rrt(start_node.angles,
    #         end_node.angles,
    #         obstacles,
    #         n_iter, radius, stepSize=stepSize)
    # size1 = 0
    # if G.success:
    #     path = dijkstra(G)
    #     size1 = len(path)
    #     runTime = time.time() - start_time
    #     optimize_start_time2 = time.time()
    #     path2 = path_optimizer_two(path, prism)
    #     size2 = len(path2)
    #     if (path2 == path):
    #         print('No optimizations could be made')
    #     if checkPath(path2, prism):
    #         optimizeTime2 = time.time() - optimize_start_time2
    #         print('Optimization Time for 2 step', optimizeTime2)
    #         print('Generation Runtime', runTime)
    #         print("New Path is valid, Size", size2)
    #         print('Original Path size:', size1)
    #         plot_3d(G, path, obstacles, path2)
    #         plot_3d(G, None, obstacles, path2)
    #     else:
    #         print("Optimized path encounters collisions, and was discarded.")
    #         plot_3d(G, path, obstacles, path2)
    #
    #     optimize_start_time4 = time.time()
    #     path4 = path_optimizer_four(path, prism)
    #     size4 = len(path4)
    #     if path4 == path:
    #         print("bruh they are the same")
    #     if checkPath(path4, prism):
    #         optimizeTime4 = time.time() - optimize_start_time4
    #         print('Optimization Time for 4 step', optimizeTime4)
    #         print('Generation Runtime', runTime)
    #         print("New Path is valid, Size", size4)
    #         print('Original Path size:', size1)
    #         plot_3d(G, path, obstacles, path4)
    #         plot_3d(G, None, obstacles, path4)
    #     else:
    #         print("Optimized path encounters collisions, and was discarded.")
    #         plot_3d(G, path, obstacles, path4)
    #
    # else:
    #     print("Path not found. :(")
    #     plot_3d(G, [start_node, end_node], obstacles, None)

"""
Implementation of an Optimal Minimum Cost algorithm for a 6DOF arm.

Given a start and end configuration, determines a collision-free path with the least possible distance traveled. This is
done by picking nodes that are close to the goal and trying to expand from them in multiple directions. It returns
a set of configurations which is passed to the ECEs.

Written by Simon Kapen '24 and Raj Sinha '25, Spring 2022.
"""
from arm_node import Node
from arm_graph import Graph
import arm_plot
from util.angles import true_angle_distances_arm
import numpy as np
from util import angles, line
import collision_detection
from pure_rrt import dijkstra
from arm_plot import plot_3d
import time
from test import tpm
import random
from obstacle_generation import random_start_environment
from collision_detection import arm_is_colliding_prisms
from matplotlib import pyplot as plt


def visited(node, g):
    """ Determines whether the algorithm has already explored in the area of the node's end effector position. """
    # Left commented for testing
    # for n in g.nodes:
    #     if np.array_equal(np.round(n.end_effector_pos, 3), np.round(node.end_effector_pos, 3)):
    #         return True
    # return False
    nearest_node_index = np.argmin(np.sum(np.square(g.end_effectors - node.end_effector_pos), axis=1))

    nearest_node = g.nodes[nearest_node_index]

    return np.array_equal(np.round(nearest_node.end_effector_pos, 3), np.round(node.end_effector_pos, 3))


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


def find_path(end_node, start_angles, obs, n_iter=150, radius=.03, step_size=.1):
    """Executes the Optimistic Predictive Cost algorithm to find a collision-free path between two configurations.
    Arguments:
        end_node: A Node representing the target arm configuration.
        start_angles: A float array representing the initial angles of the arm.
        obs: An array of float arrays representing obstacles.
        n_iter: The maximum amount of iterations to find a path.
        radius: The maximum distance from the target end effector position needed to converge.
        step_size: The distance, in radians, between each edge in the graph.
    Returns:
        A graph g containing a collision-free path if success = True, or no path if success = False.
      """
    g = Graph(start_angles, end_node.angles)
    g.ranking.append(g.start_node)

    for i in range(n_iter):
        best_node = g.ranking.pop(0)

        if arm_is_colliding_prisms(best_node, obs):
            continue

        dist_to_goal = line.distance(best_node.end_effector_pos, end_node.end_effector_pos)
        if dist_to_goal < radius:
            g.end_node = Node (Node.from_point(end_node.end_effector_pos, start_config=best_node.angles))
            g.add_vex(g.end_node, best_node)
            g.success = True
            return g

        expand(best_node, 3, step_size, g)

    return g


def opc_graph_list(num_trials, n_iter, radius, step_size, bounds, num_obstacles=1):
    """ Generates a list of Optimistic Predictive Cost graphs. """

    print("RUNNING {t} TRIALS OF OPC WITH {o} OBSTACLES\n".format(t=num_trials, o=num_obstacles))
    graphs = []
    failed_obstacles = []
    paths = []

    for i in range(0, num_trials):
        trial_start_time = time.time()

        print("Trial: ", i + 1)
        current_start_node, current_end_node, random_obstacles = \
            random_start_environment(num_obstacles, bounds)
        if current_start_node is None:
            continue
        if not current_start_node.valid_configuration():
            raise Exception("Approved an invalid start node")

        if arm_is_colliding_prisms(current_end_node, random_obstacles):
            raise Exception("Approved a colliding node")

        G = find_path(current_end_node, current_start_node.angles, random_obstacles, n_iter=n_iter, radius=radius,
                step_size=step_size)

        if G.success:
            print("SUCCESS")
            paths.append(dijkstra(G))
        else:
            print("FAIL")

        print("Trial time:", time.time() - trial_start_time)
        print("")

        graphs.append(G)
        failed_obstacles.append(random_obstacles)

    return graphs, paths


if __name__ == "__main__":
    # random.seed(13847)
    obstacle_bounds = [[-.4, .4], [-.2, .4], [-.4, .4]]
    print("time started")
    start_time = time.time()
    start_node, end_node, obstacles = random_start_environment(5, obstacle_bounds, obstacle_size=.4)
    start_angles = start_node.angles
    end_pos = end_node.end_effector_pos

    g = find_path(end_node, start_node.angles, obstacles)

    # start_angles = [0.48403438244460323, 5.326181663215166, 1.5985755384754776, 1.3130704194233174, 0.1646972276800212]
    # end_angles = [0.50680136, 5.41570846, 2.69687345, 1.82939604, -0.50680152]
    # obstacles = [
    #     [0.046264068326599184, 0.2904310706809034, -0.20008977664382757, 0.09162306237980725, 0.08896203158385652,
    #      0.18141438772659008],
    #     [-0.016393651282753274, 0.13647388688724255, -0.08405667897538899, 0.07325748715604971, 0.1719001772835061,
    #      0.12514730978419003],
    #     [0.0036383314905867326, 0.21447615410452847, 0.22043596824095468, 0.08671343867145281, 0.19556976023118172,
    #      0.08797419459934705]]
    # end_node = Node(end_angles)
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

    # goal_end_effector_bounds = [[-.4, .4], [.05, .4], [-.4, .4]]
    # trials = 100
    # graphs, paths = opc_graph_list(trials, n_iter=150, radius=.03, step_size=.1, bounds=goal_end_effector_bounds, num_obstacles=3)
    # num_successes = tpm.converge_test(graphs)
    # # tpm.print_failed_cases(graphs, failing_obstacles)
    # print("Average nodes generated: ", tpm.avg_nodes_test(graphs))
    # print("Num. successes: ", num_successes)
    # print("Convergence rate: ", num_successes / trials)
    # total_time = time.time() - start_time
    # print("Time taken: ", total_time)
    # print("Average time per graph: ", total_time / trials)
    # print("Average distance traveled:", tpm.avg_distance_traveled_test(paths))




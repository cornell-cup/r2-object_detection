"""
Implementation of an Optimal Minimum Cost algorithm for a 6DOF arm.

Given a start and end configuration, determines a collision-free path with the least possible distance traveled. This is
done by picking nodes that are close to the goal and trying to expand from them in multiple directions. It returns
a set of configurations which is passed to the ECEs.

Written by Simon Kapen '24 and Raj Sinha '25, Spring 2022.
"""
from .arm_node import Node
from .arm_graph import Graph
import .arm_plot
from .util.angles import true_angle_distances_arm
import numpy as np
from .util import angles, line
import .collision_detection
from .pure_rrt import dijkstra, nearest
from .arm_plot import plot_3d
import time
from .kinematics_test import tpm
import random
from .obstacle_generation import random_start_environment
from .collision_detection import arm_is_colliding_prisms
from matplotlib import pyplot as plt
from .util.line import distance

stack_empty_error = 0


def visited(node, g):
    """ Determines whether the algorithm has already explored in the area of the node's end effector position. """
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


def find_path(target_end_point, start_angles, obs, n_iter=150, radius=.01, step_size=.1):
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
    g = Graph(start_angles, None, target_end_point)
    g.ranking.append(g.start_node)
    g.end_node = Node.from_point(target_end_point)

    close_to_end = False

    for i in range(n_iter):
        try:
            best_node = g.ranking.pop(0)
        except IndexError:
            global stack_empty_error
            stack_empty_error += 1
            return g

        if arm_is_colliding_prisms(best_node, obs):
            continue

        dist_to_goal = line.distance(best_node.end_effector_pos, target_end_point)
        if dist_to_goal < radius:
            g.end_node = best_node
            g.success = True
            return g

        # if dist_to_goal < 10 * radius and not close_to_end:
        #     step_size /= 10
        #     close_to_end = True

        expand(best_node, 4, step_size, g)

    return g


def opc_graph_list(num_trials, n_iter, radius, step_size, bounds, num_obstacles=1):
    """ Generates a list of Optimistic Predictive Cost graphs. """

    print("RUNNING {t} TRIALS OF OPC WITH {o} OBSTACLES\n".format(t=num_trials, o=num_obstacles))
    graphs = []
    generated_obstacles = []
    paths = []

    for i in range(0, num_trials):
        trial_start_time = time.time()

        print("Trial: ", i + 1)
        current_start_node, current_end_node, random_obstacles, target_end_point = \
            random_start_environment(num_obstacles, bounds, obstacle_bounds, obstacle_size=.03)
        if current_start_node is None:
            continue
        if not current_start_node.valid_configuration():
            raise Exception("Approved an invalid start node")

        if arm_is_colliding_prisms(current_end_node, random_obstacles):
            raise Exception("Approved a colliding node")

        G = find_path(target_end_point, current_start_node.angles, random_obstacles, n_iter=n_iter, radius=radius,
                step_size=step_size)

        if G.success:
            print("SUCCESS")
            paths.append(dijkstra(G))
        else:
            print("FAIL")

        print("Trial time:", time.time() - trial_start_time)
        print("")

        graphs.append(G)
        generated_obstacles.append(random_obstacles)

    return graphs, paths, generated_obstacles


if __name__ == "__main__":
    random.seed(8)
    bounds = [[-.08, .08], [.08, .2], [.12, .2]]

    obstacle_bounds = [[-.08, .08], [-.04, .08], [-.08, .08]]
    print("time started")
    start_time = time.time()
    # start_node, end_node, obstacles, target_point = random_start_environment(0, bounds, obstacle_bounds,
    #                                                                          obstacle_size=.02)
    # start_angles = start_node.angles
    # end_pos = end_node.end_effector_pos
    #
    # g = find_path(target_point, start_node.angles, obstacles, n_iter=200)

    # start_angles = [1.1136998361065094, 0.6419500776001592, 0.27835556277259427, 5.025295680114656, 2.922418772111144]
    # end_angles = [0.70934974, 2.02103071, 1.49368975, 2.76846449, 5.57383542]
    # target_point = [0.0392305850367863, 0.0336901230180235, 0.015662864473418886]
    # obstacles = [[-0.03732420004414908, 0.007675772640742314, -0.038299019006634294, 0.07463221086743226, 0.11296745073528927, 0.09653149032778706], [-0.03493848468307048, 0.016687302804025365, 0.0243820335494229, 0.06962651433533772, 0.09940347921089933, 0.08401565730712937], [-0.028494022494606204, -0.0035408090873946807, -0.020104139523039175, 0.053669251367284995, 0.1854801243294566, 0.06036271907157065]]


    # end_node = Node(end_angles)
    # print("end valid", end_node.valid_configuration())
    # g = find_path(target_point, start_angles, obstacles)
    # if g.success:
    #     path = dijkstra(g)
    #     print("\nTime taken: ", (time.time() - start_time))
    #     plot_3d(Graph(start_angles, g.end_node.angles), path, obstacles)
    # else:
    #     g.nodes.append(g.end_node)
    #     g.node_to_index[g.end_node] = len(g.nodes)
    #
    #     path = dijkstra(g, target_node=nearest(g, g.end_node.end_effector_pos)[0])
    #     # path.append(end_angles)
    #     print("\nTime taken: ", (time.time() - start_time))
    #     print("Path not found. :(")
    #     print(start_angles)
    #     # print(end_node.angles)
    #     print(obstacles)
    #     plot_3d(g, path, obstacles)

    goal_end_effector_bounds = [[-.1, .1], [.05, .15], [.12, .2]]
    trials = 100
    graphs, paths, obstacles = opc_graph_list(trials, n_iter=150, radius=.03, step_size=.1, bounds=goal_end_effector_bounds, num_obstacles=0)
    num_successes = tpm.converge_test(graphs)
    # tpm.print_failed_cases(graphs, failing_obstacles)
    print("Average nodes generated: ", tpm.avg_nodes_test(graphs))
    print("Num. successes: ", num_successes)
    print("Convergence rate: ", num_successes / trials)
    total_time = time.time() - start_time
    print("Time taken: ", total_time)
    print("Average time per graph: ", total_time / trials)
    print("Average distance traveled:", tpm.avg_distance_traveled_test(paths))
    print("Stack empty error occurrences:", stack_empty_error)
    tpm.print_failed_cases(graphs, obstacles)

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
from pure_rrt import dijkstra, arm_is_colliding
from arm_plot import plot_3d
import time
from test import tpm


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


def find_path(end_node, start_angles, obs, n_iter=150, radius=.03, step_size=.1):
    g = Graph(start_angles, end_node.angles)
    g.ranking.append(g.start_node)

    for i in range(n_iter):
        best_node = g.ranking.pop(0)

        if arm_is_colliding(best_node, obs):
            continue

        dist_to_goal = line.distance(best_node.end_effector_pos, end_node.end_effector_pos)
        # print(dist_to_goal)
        if dist_to_goal < radius:
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


def opc_graph_list(num_trials, n_iter, radius, step_size, bounds, num_obstacles=1):
    """ Generates a list of Optimistic Predictive Cost graphs. """

    print("RUNNING {t} TRIALS OF OPC WITH {o} OBSTACLES\n".format(t=num_trials, o=num_obstacles))
    graphs = []
    failed_obstacles = []

    for i in range(0, num_trials):
        trial_start_time = time.time()

        print("Trial: ", i + 1)
        current_start_node, current_end_node, random_obstacles = \
            tpm.random_start_environment(num_obstacles, bounds)
        if current_start_node is None:
            continue
        if not current_start_node.valid_configuration():
            raise Exception("Approved an invalid start node")

        if arm_is_colliding(current_end_node, random_obstacles):
            raise Exception("Approved a colliding node")

        G = find_path(current_end_node, current_start_node.angles, random_obstacles, n_iter=n_iter, radius=radius,
                step_size=step_size)

        if G.success:
            print("SUCCESS")
        else:
            print("FAIL")

        print("Trial time:", time.time() - trial_start_time)
        print("")
        graphs.append(G)
        failed_obstacles.append(random_obstacles)

    return graphs

if __name__ == "__main__":
    obstacle_bounds = [[-.4, .4], [-.2, .4], [-.4, .4]]
    print("time started")
    start_time = time.time()
    # start_node, end_node, obstacles = tpm.random_start_environment(5, obstacle_bounds, obstacle_size=.4)
    # start_angles = start_node.angles
    # end_pos = end_node.end_effector_pos
    #
    # g = find_path(end_node, start_node.angles, obstacles)

    # start_angles = [0.09015071304095702, 6.244120885912649, 1.8464986871137932, 1.2016737110444304, 3.096500467168678]
    # end_angles = [ 0.94745773,  1.05823173, -4.71244326,  8.91195107,  5.33572758]
    # obstacles = [[-0.3754707493365236, 0.32391996277369695, -0.028305024669660672, 0.1453914268814345, 0.15484017011991635, 0.13388306206397582], [-0.13126631342576262, 0.34206600505600937, 0.09143863677824432, 0.09288374422300467, 0.10319476789818907, 0.09304262641577561], [0.21797545254721962, 0.10867226379764039, 0.09461617813673084, 0.0864749163919242, 0.16612954845703914, 0.08036648262339353]]
    # end_node = Node(end_angles)
    # g = find_path(Node(end_angles), start_angles, obstacles)
    # if g.success:
    #     path = dijkstra(g)
    #     print("\nTime taken: ", (time.time() - start_time))
    #     plot_3d(Graph(start_angles, end_node.angles), path, obstacles)
    # else:
    #     print("\nTime taken: ", (time.time() - start_time))
    #     print("Path not found. :(")
    #     print(start_angles)
    #     print(end_node.angles)
    #     print(obstacles)
    #     plot_3d(g, [Node(start_angles), end_node], obstacles)

    goal_end_effector_bounds = [[-.4, .4], [.05, .4], [-.4, .4]]
    trials = 500
    graphs = opc_graph_list(trials, n_iter=150, radius=.03, step_size=.1, bounds=goal_end_effector_bounds, num_obstacles=3)
    num_successes = tpm.converge_test(graphs)
    # tpm.print_failed_cases(graphs, failing_obstacles)
    print("Average nodes generated: ", tpm.avg_nodes_test(graphs))
    print("Num. successes: ", num_successes)
    print("Convergence rate: ", num_successes / trials)
    total_time = time.time() - start_time
    print("Time taken: ", total_time)
    print("Average time per graph: ", total_time / trials)

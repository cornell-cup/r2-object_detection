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

def expand(node, goal, step_size):
    dirn = true_angle_distances_arm(np.array(node.angles), np.array(goal.angles))
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * step_size

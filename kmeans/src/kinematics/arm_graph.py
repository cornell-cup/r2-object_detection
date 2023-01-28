"""A class representing a graph of nodes representing arm configurations.

Written by Simon Kapen '24 and Alison Duan '23, Spring 2021-Spring 2022.
Initially adapted from Fanjin Zeng on github, 2019 (gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80).
"""

from arm_node import Node
from spatial_hashing import SpatialHash
from util import line
import numpy as np


class Graph:
    """A graph representing groups of arm configurations.
    Args:
        start_angles: The initial angles of the arm.
        end_angles: The desired angles of the arm.
    Attributes:
        start_node: Node containing cartesian coordinates and arm angles of the start position.
        end_node: Node containing cartesian coordinates and arm angles of the end position.
        nodes: List of all nodes in the graph.
        edges: List of all pairs (n1, n2) for which there is an edge from node n1 to node n2.
        success: True if there is a valid path from start_node to end_node.
        node_to_index: Maps nodes to indexes that are used to find the distance from start_node of each node.
        neighbors: Maps each node to its neighbors.
        distances: Maps each node to its shortest known distance from the start node.
        ranking: List of all intermediate nodes, ordered by distance between the end effector to the target position.
        sx, sy, sz: The distance between the start and end nodes.
    """
    def __init__(self, start_angles, end_angles, target_end_pos=None):
        self.start_node = Node(start_angles)
        if end_angles is not None:
            self.end_node = Node(end_angles)

        if target_end_pos is not None:
            self.target_end_pos = target_end_pos
        else:
            self.target_end_pos = self.end_node.end_effector_pos

        self.edges = []
        self.success = False

        self.node_to_index = {self.start_node: 0}
        self.neighbors = {0: []}
        self.distances = {0: 0.}
        self.ranking = []
        self.nodes = [self.start_node]
        self.end_effectors = np.array([self.start_node.end_effector_pos])

    def add_vex(self, node, parent):
        try:
            idx = self.node_to_index[node]
        except KeyError:
            parent_idx = self.node_to_index[parent]
            idx = len(self.nodes)

            dist = parent.optimistic_cost + Node.distance(parent, node)
            node.optimistic_cost = dist

            self.neighbors[idx] = []

            self.add_edge(idx, parent_idx, Node.distance(parent, node))

            self.nodes.append(node)
            self.node_to_index[node] = idx
            self.ranking.append(node)
            self.ranking.sort(key=lambda n: dist + line.distance(n.end_effector_pos, self.target_end_pos))
            self.end_effectors = np.vstack([self.end_effectors, node.end_effector_pos])
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def dist_to_end(self, node):
        return line.distance(node.end_effector_pos, self.end_node.end_effector_pos)

    def get_parent(self, idx):
        near_neighbors = self.neighbors[idx]
        parent_idx = near_neighbors[0][0]
        node_list = list(self.node_to_index.keys())
        return node_list[parent_idx]

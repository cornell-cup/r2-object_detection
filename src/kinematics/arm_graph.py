from arm_node import Node
from spatial_hashing import SpatialHash
from util import line

class Graph:
    """An graph representing groups of arm configurations.
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
    def __init__(self, start_angles, end_angles):
        self.start_node = Node(start_angles)
        self.end_node = Node(end_angles)

        self.edges = []
        self.success = False

        self.node_to_index = {self.start_node: 0}
        self.neighbors = {0: []}
        self.distances = {0: 0.}
        self.ranking = []
        self.nodes = [self.start_node]

        self.spatial_hash = SpatialHash(0.2, 0.2, 0.2)
        self.spatial_hash.insert_node(self.start_node.end_effector_pos, self.start_node, 0)

    def add_vex(self, node, parent):
        try:
            idx = self.node_to_index[node]
        except:
            parent_idx = self.node_to_index[parent]
            idx = len(self.nodes)

            dist = parent.optimistic_cost + Node.distance(parent, node)
            node.optimistic_cost = dist

            self.neighbors[idx] = []

            self.add_edge(idx, parent_idx, Node.distance(parent, node))

            self.nodes.append(node)
            self.node_to_index[node] = idx
            self.ranking.append(node)
            self.ranking.sort(key=lambda n: dist + Node.distance(n, self.end_node))

            self.spatial_hash.insert_node(node.end_effector_pos, node, idx)
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

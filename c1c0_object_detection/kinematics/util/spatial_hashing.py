"""A 3D spatial hashing algorithm.

A hashing algorithm that splits 3D points into buckets and determines closest neighbors of certain points through
linear search.

Written by Alison Duan '23, Spring 2022.
"""

from builtins import int, range
from collections import defaultdict
import math

class SpatialHash:
    def __init__(self, x, y, z):
        self.x_cell_length = x
        self.y_cell_length = y
        self.z_cell_length = z

        # set a defaultdict for contents 
        self.contents = defaultdict(list)

    def hash(self, point):
        x = math.floor(point[0]/self.x_cell_length) 
        y = math.floor(point[1]/self.y_cell_length)
        z = math.floor(point[2]/self.z_cell_length)
        return x, y, z

    def insert_node(self, point, node, index):
        self.contents[self.hash(point)].append((index, node))

    def closest_neighbors(self, point):
        """ Finds the closest neighboring cells of a point in the spatial hash.

        Returns: 
            List of nodes that are in the same cell as point or in a 
            neighboring cell. 
        """
        cell = self.hash(point)
        cell_x, cell_y, cell_z = cell[0], cell[1], cell[2]
        neighbors = self.contents[cell][:]
        level = 1
        for x in [cell_x-2, cell_x+3]:
            for y in range (cell_y-2, cell_y+3):
                for z in range (cell_z-2, cell_z+3):
                    neighbors += self.contents[(x, y, z)]
        return neighbors
    # def insert_object_for_box(self, box, object):
    #     # hash the minimum and maximum points
    #     min, max = self.hash(box.min), self.hash(box.max)
    #     # iterate over the rectangular region
    #     for i in range(min[0], max[0]+1):
    #         for j in range(min[1], max[1]+1):
    #             # append to each intersecting cell
    #             self.contents.setdefault((i, j), []).append(object)
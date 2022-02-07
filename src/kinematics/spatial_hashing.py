from builtins import int, range
from collections import defaultdict

class SpatialHash:
    def __init__(self, x, y, z):
        self.x_cell_length = x
        self.y_cell_length = y
        self.z_cell_length = z

        # set a defaultdict for contents 
        self.contents = defaultdict(list)

    def hash(self, point):
        x = point[0]/self.x_cell_length 
        y = point[1]/self.y_cell_length
        z = point[2]/self.z_cell_length
        return x, y, z

    def insert_node(self, point, node):
        self.contents[self.hash(point)].append(node)

    # def insert_object_for_box(self, box, object):
    #     # hash the minimum and maximum points
    #     min, max = self.hash(box.min), self.hash(box.max)
    #     # iterate over the rectangular region
    #     for i in range(min[0], max[0]+1):
    #         for j in range(min[1], max[1]+1):
    #             # append to each intersecting cell
    #             self.contents.setdefault((i, j), []).append(object)
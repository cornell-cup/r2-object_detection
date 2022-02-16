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
        x = int(point[0]/self.x_cell_length) 
        y = int(point[1]/self.y_cell_length)
        z = int(point[2]/self.z_cell_length)
        return x, y, z

    def insert_node(self, point, node, index):
        self.contents[self.hash(point)].append((index, node))

    def closest_neighbors(self, point):
        """ Finds the closest neighboring cells of a point in the spatial hash.

        Returns: 
            List of nodes that are in the same cell as the point or in a 
            neighboring cell. 
        """
        cell = self.hash(point)
        cell_x, cell_y, cell_z = cell[0], cell[1], cell[2]
        neighbors = self.contents[cell]
        level = 1
        while not neighbors:
            # checking on the two x planes
            for x in [cell_x-level, cell_x+level]:
                for y in range (cell_y-level-1, cell_y+level):
                    for z in range (cell_z-level-1, cell_z+level):
                        neighbors += self.contents[(x, y, z)]
        
            # checking on the two y planes
            for x in range (cell_x-level-1, cell_x+level):
                for y in [cell_y-level, cell_y+level]:
                    for z in range (cell_z-level-1, cell_z+level):
                        neighbors += self.contents[(x, y, z)]

            # checking on the two z planes
            for x in range (cell_x-level-1, cell_x+level):
                for y in range (cell_y-level-1, cell_y+level):
                    for z in [cell_z-level-1, cell_z+level]:
                        neighbors += self.contents[(x, y, z)]
            level += 1
        #print("found")
        return neighbors
    # def insert_object_for_box(self, box, object):
    #     # hash the minimum and maximum points
    #     min, max = self.hash(box.min), self.hash(box.max)
    #     # iterate over the rectangular region
    #     for i in range(min[0], max[0]+1):
    #         for j in range(min[1], max[1]+1):
    #             # append to each intersecting cell
    #             self.contents.setdefault((i, j), []).append(object)
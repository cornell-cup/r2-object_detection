from kinematics import kinematics
import math
import numpy as np
import pure_rrt_angles as rrt
from rrtgraph import Graph
from rrtnode import RRTNode
import line


def generate_linear_path(start_angles, end_angles, num_iter):
    """ Return a path of nodes from start_pos to end_pos, with [num_iter] iterations of equal length. """


    step_sizes = np.divide(np.subtract(end_angles, start_angles), num_iter)
    g = Graph(start_angles, end_angles)

    current_angles = start_angles
    current_idx = 0
    current_node = g.nodes[0]
    for i in range(num_iter):
        if not rrt.valid_configuration(current_node.angles):
            print("Iteration {} not valid".format(i))
            return g
        new_angles = (current_angles[0] + step_sizes[0], current_angles[1] + step_sizes[1],
                      current_angles[2] + step_sizes[2], current_angles[3] + step_sizes[3],
                      current_angles[4], current_angles[5])
        new_node = RRTNode(new_angles)

        new_idx = g.add_vex(new_node)

        dist = line.distance(new_node.end_effector_pos, current_node.end_effector_pos)

        g.add_edge(current_idx, new_idx, dist)

        current_angles = new_angles
        current_idx = new_idx
        current_node = new_node
    return g


if __name__ == '__main__':
    startpos = (0., 0., 0., 0., 0., 0.)

    x, y, z = (.2, -.2, 0)

    angles = [round(x[0], 5) for x in kinematics(x, y, z).tolist()]
    iterations = 10
    endpos = (math.radians(angles[1]), math.radians(angles[0]), math.radians(angles[3]), math.radians(angles[2]), 0, 0)
    #endpos = (4.554375325141994, -2.1374669422705335, 0.20145182193493724, -0.6768399716688464, 2.999963235383948, -2.3362358543269828)
    g = generate_linear_path(startpos, endpos, iterations)
    path = g.nodes
    rrt.plot_3d(g, path, [])

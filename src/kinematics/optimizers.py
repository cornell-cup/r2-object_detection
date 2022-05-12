import collision_detection

"""
Written by Yashraj Sinha, Spring 2022.
Optimizes arm paths from rrt. 
"""


def path_optimizer_two(path, prism):
    """
    Args:
        path: refers to the output of dijkstra method from pure_rrt_angles,
              a list of rrtnodes
        prism: the object to be avoided, given in the form
               [<x coord.>, <y coord.>, <z coord.>, <length.>, <width.>, <height.>].
    Returns:
        A new list with some rrtnodes removed where linear paths can be created.
        list length <= path length
    """
    optimizedList = path.copy()
    # This code only checks every other node
    for i in range(0, len(path) - 2, 2):
        p1 = path[i].end_effector_pos
        p2 = path[i + 2].end_effector_pos
        # print('p1',p1)
        line_seg = ([p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]])
        # print(collision_detection.newLineCollider(line_seg, prism))
        if collision_detection.newLineCollider(line_seg, prism) == None:
            optimizedList.remove(path[i + 1])
            # print("non-colliison found")
    return optimizedList

def path_optimizer_two_prismset(path, prism_set):
    """
    Args:
        path: refers to the output of dijkstra method from pure_rrt_angles,
              a list of rrtnodes
        prism_set: the set of prisms to be avoided, given in the form
               [<x coord.>, <y coord.>, <z coord.>, <length.>, <width.>, <height.>].
    Returns:
        A new list with some rrtnodes removed where linear paths can be created.
        list length <= path length
    """
    optimizedList = path.copy()
    # To save time we will only check every other node
    for i in range(0, len(path) - 2, 2):
        p1 = path[i].end_effector_pos
        p2 = path[i + 2].end_effector_pos
        # print('p1',p1)
        line_seg = ([p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]])
        # print(collision_detection.newLineCollider(line_seg, prism))
        valid = True
        for prism in prism_set:
            if not collision_detection.newLineCollider(line_seg, prism) == None:
                valid = False
        if valid:
            optimizedList.remove(path[i + 1])
    return optimizedList

def path_optimizer_four(path, prism):
    """
    Args:
        path: refers to the output of dijkstra method from pure_rrt_angles,
              a list of rrtnodes
        prism: the object to be avoided, given in the form
               [<x coord.>, <y coord.>, <z coord.>, <length.>, <width.>, <height.>].
    Returns:
        A new list with some rrtnodes removed where linear paths can be created.
        list length <= path length
    """
    optimizedList = path.copy()

    for i in range(0, len(path) - 4, 4):
        p1 = path[i].end_effector_pos
        p2 = path[i + 4].end_effector_pos
        # print('p1',p1)
        line_seg = ([p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]])
        # print(collision_detection.newLineCollider(line_seg, prism))
        if collision_detection.newLineCollider(line_seg, prism) == None:
            optimizedList.remove(path[i + 1])
            optimizedList.remove(path[i + 2])
            optimizedList.remove(path[i + 3])
            # print("non-colliison found")
    return optimizedList

def optimize(path,prism_set):
    """
        Optimizes a path by performing a series of two-step optimizations. This
        method takes < .001 seconds to run.

    param path: a given path as a set of nodes for the optimization to be performed on.
    param prism: a prism object which the path must avoid.
    return: lastPath: a path of length less than that of the given path.
    """
    collision = False
    lastPath = path
    newPath = None
    while not collision:
        newPath = path_optimizer_two_prismset(lastPath, prism_set)
        if newPath == lastPath:
            break
        for i in range(len(newPath)):
            if not newPath[i].valid_configuration():
                collision = True
        if not collision:
            lastPath = newPath
    return lastPath

def checkPath(path, prism):
    # This method checks if the passed path collides with the prism
    # between arm motions. Essentially it draws lines to corresponding nodes
    # on the subsequent arm configs.
    for i in range(len(path) - 1):
        for j in range(len(path[i].joint_positions)):
            x1 = path[i].joint_positions[j][0]
            y1 = path[i].joint_positions[j][1]
            z1 = path[i].joint_positions[j][2]
            x2 = path[i + 1].joint_positions[j][0]
            y2 = path[i + 1].joint_positions[j][1]
            z2 = path[i + 1].joint_positions[j][2]
            line_seg = ([x1, y1, z1, x2, y2, z2])
            if (collision_detection.newLineCollider(line_seg, prism)):
                return False
    return True

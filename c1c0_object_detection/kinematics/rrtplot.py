import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d
from . import collision_detection
"""
Written by Simon Kapen, Spring 2021.
Handles all visualizations of multiple arm poses. 
"""


def arr_to_int(arr):
    """ The int array representation of an array of arrays. """
    new_array = []
    for i in range(0, len(arr)):
        new_array.append(arr[i])

    return new_array


def plot_3d(G, path, obstacles):
    ax = plt.axes(projection='3d')

    end_effector_positions = []

    for v in G.nodes:
        end_effector_positions.append(v.end_effector_pos)

    float_vertices = list(map(arr_to_int, end_effector_positions))

    plot_edges(ax, G, float_vertices)

    plot_nodes(ax, float_vertices)

    if path is not None:
        plot_path(ax, path)
        plot_arm_configs(ax, path, obstacles)

    ax.scatter3D(G.start_node.end_effector_pos[0], G.start_node.end_effector_pos[1], G.start_node.end_effector_pos[2], c='black')
    ax.scatter3D(G.end_node.end_effector_pos[0], G.end_node.end_effector_pos[1], G.end_node.end_effector_pos[2], c='black')

    # obstacles
    for obs in obstacles:
        collision_detection.plot_linear_cube(ax, obs)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_xlim3d(-.3, .35)
    ax.set_ylim3d(-.3, .3)
    ax.set_zlim3d(-.3, .4)

    plt.show()


def plot_nodes(ax, float_vertices):
    intermediate_vertices = []
    for i in range(1, len(float_vertices) - 1):
        intermediate_vertices.append(float_vertices[i])

    xdata = [x for x, y, z in intermediate_vertices]
    ydata = [y for x, y, z in intermediate_vertices]
    zdata = [z for x, y, z in intermediate_vertices]

    ax.scatter3D(xdata, ydata, zdata, c=None)


def plot_edges(ax, G, float_vertices):
    lines = [(float_vertices[edge[0]], float_vertices[edge[1]]) for edge in G.edges]

    lc = art3d.Line3DCollection(lines, colors='black', linewidths=1)
    ax.add_collection(lc)


def plot_path(ax, path):
    path_vertices = []
    for i in range(0, len(path)):
        path_vertices.append(path[i].end_effector_pos)
    path_vertices = list(map(arr_to_int, path_vertices))
    paths = [(path_vertices[i], path_vertices[i + 1]) for i in range(len(path_vertices) - 1)]
    lc2 = art3d.Line3DCollection(paths, colors='green', linewidths=3)
    ax.add_collection(lc2)


def plot_arm_configs(ax, path, obstacles):
    color = 'green'
    for i in range(0, len(path)):
        for obstacle in obstacles:
            if collision_detection.arm_is_colliding(path[i].to_nlinkarm(), obstacle):
                color = 'red'
                break
        v1 = []
        v2 = []
        for j in range(3):
            v1.append([0, path[i].joint_positions[0][j]])
            v2.append([path[i].joint_positions[0][j], path[i].joint_positions[1][j]])

        ax.plot(v1[0], v1[1], zs=v1[2], color=color)
        ax.plot(v2[0], v2[1], zs=v2[2], color=color)

"""Handles all visualizations of multiple arm poses.

Written by Simon Kapen '24 and Raj Sinha '25, Spring 2021.
"""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import art3d
import collision_detection
from arm_node import Node
from matplotlib.animation import FuncAnimation
import math
from util.line import distance


def configure_graph(ax, axis_labels=['X', 'Y', 'Z'], axis_limits=[[-.2, .4], [-.2, .4], [-.2, .4]]):
    """ Configures axis lengths and names for a matplotlib graph. """

    ax.set_xlabel(axis_labels[0])
    ax.set_ylabel(axis_labels[1])
    ax.set_zlabel(axis_labels[2])

    ax.set_xlim3d(axis_limits[0][0], axis_limits[0][1])
    ax.set_ylim3d(axis_limits[1][0], axis_limits[1][1])
    ax.set_zlim3d(axis_limits[2][0], axis_limits[2][1])


def arr_to_int(arr):
    """ The int array representation of an array of arrays. """
    new_array = []
    for i in range(0, len(arr)):
        new_array.append(arr[i])

    return new_array


def plot_3d(G, path, obstacles, path2=None):
    fig = plt.figure(figsize=(6, 6))
    ax = plt.axes(projection='3d')

    end_effector_positions = []

    for v in G.nodes:
        end_effector_positions.append(v.end_effector_pos)
    print(end_effector_positions)
    float_vertices = list(map(arr_to_int, end_effector_positions))

    plot_edges(ax, G, float_vertices)

    plot_nodes(ax, float_vertices[1:-2])

    if path is not None:
        plot_path(ax, path)
        plot_arm_configs(ax, path, obstacles)
    if path2 is not None:
        plot_path(ax,path2,'blue')
        plot_arm_configs(ax,path2,obstacles)

    ax.scatter3D(G.start_node.end_effector_pos[0], G.start_node.end_effector_pos[1], G.start_node.end_effector_pos[2], c='red')
    ax.scatter3D(G.end_node.end_effector_pos[0], G.end_node.end_effector_pos[1], G.end_node.end_effector_pos[2], c='purple')

    # collision_detection.plot_linear_prism(ax, (0, 0, 0, .01, .3, .01), "blue")

    for obs in obstacles:
        collision_detection.plot_linear_prism(ax, obs, 'blue')

    configure_graph(ax)

    plt.show()


def plot_nodes(ax, nodes):


    xdata = [x for x, y, z in nodes]
    ydata = [y for x, y, z in nodes]
    zdata = [z for x, y, z in nodes]

    ax.scatter3D(xdata, ydata, zdata, c=None)


def plot_edges(ax, G, nodes):
    lines = [(nodes[edge[0]], nodes[edge[1]]) for edge in G.edges]

    lc = art3d.Line3DCollection(lines, colors='black', linewidths=1)
    ax.add_collection(lc)


def plot_path(ax, path,color='green'):
    path_vertices = []
    for i in range(0, len(path)):
        path_vertices.append(path[i].end_effector_pos)
    path_vertices = list(map(arr_to_int, path_vertices))
    paths = [(path_vertices[i], path_vertices[i + 1]) for i in range(len(path_vertices) - 1)]
    if color != 'green':
        lc2 = art3d.Line3DCollection(paths,color=color,linewidths=3)
    else:
        lc2 = art3d.Line3DCollection(paths, colors=color, linewidths=3)
    ax.add_collection(lc2)


def plot_arm_configs(ax, path, obstacles, color='green'):
    for i in range(0, len(path)):
        if obstacles is not None:
            for obstacle in obstacles:
                if collision_detection.arm_is_colliding_prism(path[i], obstacle):
                    print(path[i].angles)
                    color = 'red'

        v = [[] for j in range(len(path[i].joint_positions))]

        for j in range(3):
            for k in range(len(v)-1):
                v[k].append([path[i].joint_positions[k][j], path[i].joint_positions[k+1][j]])

        for j in range(len(v)-1):
            ax.plot(v[j][0], v[j][1], zs=v[j][2], color=color)

        # ax.plot(v[0][0], v[0][1], zs=v[0][2], color=color)
        # ax.plot(v[1][0], v[1][1], zs=v[1][2], color=color)
        # ax.plot(v[2][0], v[2][1], zs=v[2][2], color=color)
        # ax.plot(v[3][0], v[3][1], zs=v[3][2], color=color)
        # ax.plot(v[4][0], v[4][1], zs=v[4][2], color=color)


def plot_trial_times(graph_list, times):
    for i in range(len(graph_list)):
        if graph_list[i].success:
            plt.scatter(i, times[i], c='green')
        else:
            plt.scatter(i, times[i], c='red')
    plt.yticks(np.arange(0, 11, 1))
    plt.ylim(0, 10)
    plt.xlabel('Trial')
    plt.ylabel('Time (s)')
    plt.show()


def plot_ik_trial(target_point, obtained_config):
    """ Visualizes the distance between a target point and a point obtained through inverse kinematics. """
    ax = plt.axes(projection='3d')
    plot_nodes(ax, [target_point])
    plot_arm_configs(ax, [obtained_config], None)
    print("Link 1 length:", np.linalg.norm(obtained_config.joint_positions[1] - obtained_config.joint_positions[0]))
    print("Link 2 length:", np.linalg.norm(obtained_config.joint_positions[2] - obtained_config.joint_positions[1]))
    print("Link 3 length:", np.linalg.norm(obtained_config.joint_positions[3] - obtained_config.joint_positions[2]))
    print("Link 4 length:", np.linalg.norm(obtained_config.joint_positions[4] - obtained_config.joint_positions[3]))
    print("Link 5 length:", np.linalg.norm(obtained_config.joint_positions[5] - obtained_config.joint_positions[4]))



if __name__ == "__main__":
    # configs = [Node.from_point([-0.05804748277798497, -0.05042320496342255, 0.01512500676344014])]
    # node = Node([0, 0, 0, 0, 0, 0])
    # configs = [Node([0, 0, 0, 0, 0, 0])]
    #
    # for i in range(len(node.joint_positions)-1):
    #     print("Link {} length:".format(i+1), np.linalg.norm(node.joint_positions[i+1] - node.joint_positions[i]))
    #
    # print(node.end_effector_pos)
    # print(node.joint_positions)
    # print(node.angles)
    configs = []
    for i in range(1):
        node = Node(None)
        configs.append(node)

    ax = plt.axes(projection='3d')
    configure_graph(ax)

    plot_arm_configs(ax, configs, [])

    plt.show()

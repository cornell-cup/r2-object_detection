"""Handles all visualizations of multiple arm poses.

Written by Simon Kapen '24 and Raj Sinha '25, Spring 2021.
"""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import art3d
import collision_detection
from arm_node import Node


def arr_to_int(arr):
    """ The int array representation of an array of arrays. """
    new_array = []
    for i in range(0, len(arr)):
        new_array.append(arr[i])

    return new_array


def plot_3d(G, path, obstacles, path2=None):
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
    if path2 is not None:
        plot_path(ax,path2,'blue')
        plot_arm_configs(ax,path2,obstacles)

    ax.scatter3D(G.start_node.end_effector_pos[0], G.start_node.end_effector_pos[1], G.start_node.end_effector_pos[2], c='red')
    ax.scatter3D(G.end_node.end_effector_pos[0], G.end_node.end_effector_pos[1], G.end_node.end_effector_pos[2], c='purple')

    for obs in obstacles:
        collision_detection.plot_linear_prism(ax, obs, 'blue')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_xlim3d(-.4, .4)
    ax.set_ylim3d(-.4, .4)
    ax.set_zlim3d(-.4, .4)

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
        for obstacle in obstacles:
            if collision_detection.arm_is_colliding_prism(path[i], obstacle):
                print(path[i].angles)
                color = 'red'
        v1 = []
        v2 = []
        v3 = []
        v4 = []
        for j in range(3):
            v1.append([path[i].joint_positions[0][j], path[i].joint_positions[1][j]])
            v2.append([path[i].joint_positions[1][j], path[i].joint_positions[2][j]])
            v3.append([path[i].joint_positions[2][j], path[i].joint_positions[3][j]])
            v4.append([path[i].joint_positions[3][j], path[i].joint_positions[4][j]])

        ax.plot(v1[0], v1[1], zs=v1[2], color=color)
        ax.plot(v2[0], v2[1], zs=v2[2], color=color)
        ax.plot(v3[0], v3[1], zs=v3[2], color=color)
        ax.plot(v4[0], v4[1], zs=v4[2], color=color)

    #ax.text(-0.3, -0.3, 0.4, "plot text test")


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


if __name__ == "__main__":
    path = []
    for i in range(50):
        node = Node(None)
        path.append(node)

    ax = plt.axes(projection='3d')
    plot_arm_configs(ax, path, [])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim3d(-.4, .4)
    ax.set_ylim3d(-.4, .4)
    ax.set_zlim3d(-.4, .4)
    plt.show()

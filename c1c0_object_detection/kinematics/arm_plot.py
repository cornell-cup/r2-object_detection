"""Handles all visualizations of multiple arm poses.

Written by Simon Kapen '24 and Raj Sinha '25, Spring 2021.
"""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import art3d
from .collision_detection import *
from .arm_node import Node
from matplotlib.animation import FuncAnimation
import math
from util.line import distance
import random
from os import getenv


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
    ax = plt.axes(projection='3d')

    end_effector_positions = []

    for v in G.nodes:
        end_effector_positions.append(v.end_effector_pos)

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


    for obs in obstacles:
        plot_linear_prism(ax, obs, 'blue')

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
                if arm_is_colliding_prism(path[i], obstacle):
                    color = 'red'
                else:
                    color = 'green'

        v = [[] for j in range(len(path[i].joint_positions))]

        for j in range(3):
            for k in range(len(v)-1):
                v[k].append([path[i].joint_positions[k][j], path[i].joint_positions[k+1][j]])

        for j in range(len(v)-1):
            ax.plot(v[j][0], v[j][1], zs=v[j][2], color=color)

        for j in range(len(v)-1):
            ax.plot(v[j][0], v[j][1], zs=v[j][2], color=color)


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


def plot_config_at_point():
    configs = []
    points = []

    bounds = [[-.08, .08], [.08, .2], [.12, .2]]
    random_point = [random.uniform(bounds[0][0], bounds[0][1]),
                    random.uniform(bounds[1][0], bounds[1][1]),
                    random.uniform(bounds[2][0], bounds[2][1])]
    for i in range(1):
        random_point = [random.uniform(bounds[0][0], bounds[0][1]),
                        random.uniform(bounds[1][0], bounds[1][1]),
                        random.uniform(bounds[2][0], bounds[2][1])]
        points.append(random_point)
        node = Node.from_point(random_point)
        configs.append(node)

    plot_arm_configs(ax, configs, [])
    ax.scatter3D([random_point[0]], [random_point[1]], [random_point[2]])
    plt.show()


def plot_configs_with_obstacle():
    configs = [Node(None) for i in range(10)]
    obstacles = [[.01, .01, .02, .1, .1, .1]]
    plot_arm_configs(ax, configs, obstacles)
    plot_linear_prism(ax, obstacles[0], 'blue')


if __name__ == "__main__":
    ax = plt.axes(projection='3d')
    configure_graph(ax)

    func = getenv('FUNCTION')
    if func == 'PLOT_WITH_OBSTACLE':
        plot_configs_with_obstacle()
    elif func == 'PLOT_AT_POINT':
        plot_config_at_point()

    plt.show()

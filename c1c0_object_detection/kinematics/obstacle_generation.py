"""Functions for randomly generating a test environment for an arm path.

Written by Simon Kapen '24, Spring 2022.
"""


import random
import matplotlib.pyplot as plt
from .collision_detection import plot_linear_prism, arm_is_colliding_prisms
from .arm_node import Node
from .util.line import distance
import numpy as np
import typing


def generate_random_obstacles(num: int, axes_limits: list[list[float]], max_side_length=.2, min_side_length = .05):
    """Randomly generates cuboid obstacles for an environment.

    Args:
        num: The number of obstacles to be generated.
        axes_limits: The [x, y, z] limits of the environment.
        max_side_length: The maximum side length of an obstacle.

    Returns:
        An array of arrays of floats [x, y, z, l, w, h] representing cuboid obstacles.

    """
    obstacles = []

    for i in range(num):
        sides = [
            random.uniform(min_side_length, max_side_length),
            random.uniform(min_side_length, max_side_length),
            random.uniform(min_side_length, max_side_length)
        ]
        obs = [
            random.uniform(axes_limits[0][0], axes_limits[0][1]),
            random.uniform(axes_limits[1][0], axes_limits[1][1]),
            random.uniform(axes_limits[2][0], axes_limits[2][1]),
            sides[0],
            sides[1],
            sides[2]
        ]

        obstacles.append(obs)

    return obstacles


def random_start_environment(num_obstacles, bounds, obstacle_bounds, obstacle_size=.2):
    """Generates a start environment for a run of RRT.

     Returns:
         A Node representing a valid start configuration.
         A Node representing a valid end configuration.
         A set of [num_obstacles] obstacles that do not collide with the start or end configurations.
    """

    random_start_node = Node(configuration=None)
    target_end_pos = [random.uniform(bounds[0][0], bounds[0][1]),
             random.uniform(bounds[1][0], bounds[1][1]),
             random.uniform(bounds[2][0], bounds[2][1])]

    random_end_node = Node.from_point(target_end_pos, random_start_node.angles)

    while (not random_end_node.valid_configuration()):
        target_end_pos = [random.uniform(bounds[0][0], bounds[0][1]),
                          random.uniform(bounds[1][0], bounds[1][1]),
                          random.uniform(bounds[2][0], bounds[2][1])]
        random_end_node = Node.from_point(target_end_pos, random_start_node.angles)\

    current_obstacles = generate_random_obstacles(num_obstacles, obstacle_bounds, max_side_length=obstacle_size)
    while arm_is_colliding_prisms(random_end_node, current_obstacles):
        current_obstacles = generate_random_obstacles(num_obstacles, obstacle_bounds, max_side_length=obstacle_size)

    while arm_is_colliding_prisms(random_start_node, current_obstacles) or not random_start_node.valid_configuration():
        random_start_node = Node(None)

    print("start angles:", random_start_node.angles)
    print("end angles:", random_end_node.angles)
    print("obstacles:", current_obstacles)

    return random_start_node, random_end_node, current_obstacles, target_end_pos





if __name__ == "__main__":
    ax = plt.axes(projection='3d')
    obs_array = generate_random_obstacles(5, [[-.4, .4], [0, .4], [-.4, .4]])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim3d(-.4, .4)
    ax.set_ylim3d(-.4, .4)
    ax.set_zlim3d(-.4, .4)
    # obstacles
    for o in obs_array:
        plot_linear_prism(ax, o)
    plt.show()

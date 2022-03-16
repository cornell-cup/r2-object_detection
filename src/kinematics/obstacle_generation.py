import random
import matplotlib.pyplot as plt
import collision_detection
from typing import List


def generate_random_obstacles(num: int, axes_limits: List[List[float]], max_side_length=.2):
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
            random.uniform(0, max_side_length),
            random.uniform(0, max_side_length),
            random.uniform(0, max_side_length)
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
        collision_detection.plot_linear_prism(ax, o)
    plt.show()

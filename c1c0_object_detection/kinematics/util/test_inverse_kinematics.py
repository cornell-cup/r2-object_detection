import numpy as np
from arm_node import Node
import random
from arm_plot import plot_ik_trial, plot_nodes
import matplotlib.pyplot as plt
import math
import time


def inverse_kinematics_random_test(threshold, num_trials, bounds):
    failed_points = [None]
    error_count = 0
    for i in range(num_trials):
        print("Trial:", i)
        point = [random.uniform(bounds[0][0], bounds[0][1]),
                 random.uniform(bounds[1][0], bounds[1][1]),
                 random.uniform(bounds[2][0], bounds[2][1])]
        start_time = time.time()
        node = Node.from_point(point)
        print("IK time", time.time() - start_time)

        print("Target point:", point)
        print("Obtained point:", node.joint_positions[-2])
        diff = np.linalg.norm(np.array(node.joint_positions[-2]) - np.array(point))
        print("Difference:", diff)

        if diff > threshold:
            print("BAD!!!")
            error_count += 1
            failed_points.append(point)
            print("angles:", node.angles)

        else:
            print("GOOD!!!!")

        plot_ik_trial(point, Node(node.angles))

    # plot_nodes(ax, failed_points)
    print("Success rate", 1 - error_count / num_trials)
    plt.show()


if __name__ == "__main__":
    bounds = [[-.08, .08], [-.08, .08], [.14, .155]]
    inverse_kinematics_random_test(.01, 100, bounds)

"""A black-box test suite of linear RRT.

Written by Simon Kapen, Fall 2021.
"""
# import linear_rrt as lrrt
# import pure_rrt as rrt
import math
import numpy as np
from arm_node import Node
import random


test_count = 0


def test_array(array, expected, description):
    global test_count
    print("TEST {n}: {d}".format(n=test_count, d=description))

    assert np.array_equal(np.round(array, 5), np.round(expected, 5)), description
    test_count += 1


def test_answer():
    start_angles = [0, 0, 0, 0, 0, 0]
    end_angles = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]
    obstacles = []

    path = lrrt.linear_rrt(start_angles, end_angles, obstacles)
    test_array(path[-1].angles, end_angles, "linear rrt converges")
 
    step_sizes = lrrt.compute_step_sizes([0, 1, 1, 2, 5, 2], [1, 0, 2, 1, 2, 5], 100)
    test_array(step_sizes, [0.01, -0.01, -0.05283185307179586, 0.05283185307179586, -0.03, 0.03],
               "step size calculation observes bounds")

    print("TEST {n}: {d}".format(n=test_count, d="arm does not collide on path"))
    end_angles_nonuniform = [1.8756746293234707, -0.24887138496377703, 0.33744701903541446, 0.15153332538250205, 0, 0]
    obstacles = [[0.1, -0.3, 0.15, 0.2]]
    path_through_obstacle = lrrt.linear_rrt(start_angles, end_angles_nonuniform, obstacles)
    for node in path_through_obstacle:
        assert not rrt.arm_is_colliding(node, obstacles), "arm does not collide"

    print("ALL TESTS PASS")


def inverse_kinematics_test(threshold, num_trials, bounds):
    error_count = 0
    for i in range(num_trials):
        print("Trial:", i)
        point = [random.uniform(bounds[0][0], bounds[0][1]),
             random.uniform(bounds[1][0], bounds[1][1]),
             random.uniform(bounds[2][0], bounds[2][1])]

        node = Node.from_point(point)

        print("Target point:", point)
        print("Obtained point:", node.end_effector_pos)
        diff = np.linalg.norm(node.end_effector_pos, point)
        print("Difference:", diff)

        if diff > threshold:
            print("BAD!!!")
            error_count += 1
        else:
            print("GOOD!!!!")

    print("Success rate", error_count / num_trials)


if __name__ == "__main__":
    bounds = [[-.1, .1], [-.1, .1], [-1, .1]]
    inverse_kinematics_test(.03, 100, bounds)
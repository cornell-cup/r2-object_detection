"""Representation of a node in an RRT graph, which represents a single arm configuration.

Represents a single configuration of the precision arm using the five joint angles. Specifications of the arm itself are
held in models/SimpleArmModelForURDF.urdf.

Written by Simon Kapen '24, Spring 2021.
"""
import time

import numpy
import matplotlib.pyplot as plt
from util.error_handling import nostderr
import numpy as np
import math
import random
import kinpy as kp
import ikpy as IKPY
from util import line
from ikpy.chain import Chain

# Global arm configuration - IMPORTANT: wraps with nostderr() to hide command line errors.
ik_py = True #boolean flag: True if ik_py, False if Kinpy
with nostderr():
    chain = kp.build_chain_from_urdf(open("models/XArm.urdf").read())
    serial_chain = kp.build_serial_chain_from_urdf(open("models/XArm.urdf").read(), "link5", "base_link")
    amazon_arm_chain = Chain.from_urdf_file("models/XArm.urdf")
    xarm_joint_names = ['arm1', 'arm2', 'arm3', 'arm4', 'arm5']
    xarm_link_names = ['base_link', 'link1', 'link2', 'link3', 'link4']


class Node(object):
    """
    A node generated in a RRT graph.
    Args:
        configuration: list of joint angles in radians. Corresponds to the 6 degrees of freedom on the arm.
            a0-- the pan angle of the base
            a1-- the lift angle of the base
            a2-- the lift angle of the elbow
            a3-- the pan angle of the elbow
            a4-- the pan angle of the wrist
            (a5-- how big to open the end effector -- not programmed)
    Attributes:
        bounds: An array of float arrays listing the lower and upper bounds of each arm angle.
        end_effector_pos: A np array of the [x, y, z] of the end effector.
        angles: A np array listing the joint angles in radians.
        optimistic_cost: A float representing the optimal path cost traveling through this node.
        fail_count: An integer counting the number of times the extension heuristic has failed from this node.
    """

    null = (2 * math.pi, 0)
    SIMPLE_ARM_BOUNDS = [
        (3 * math.pi / 2, .9 * math.pi / 2),
        (3 * math.pi / 2, 1.22173),
        (1.7 * math.pi, 2.8 * math.pi / 2),
        (0, 2 * math.pi),
        (0, 2 * math.pi),
        (0, 2 * math.pi)
    ]

    XARM_URDF_BOUNDS = [
        (2*math.pi, math.pi),
        (2*math.pi, math.pi/2),
        (17 * math.pi / 16, 15 * math.pi / 16),
        (17 * math.pi / 16, 15 * math.pi / 16),
        (17 * math.pi / 16, 15 * math.pi / 16),
        (17 * math.pi / 16, 15 * math.pi / 16),
    ]

    bounds = XARM_URDF_BOUNDS

    def __init__(self, configuration: list[float]):
        if configuration is None:
            self.angles = self.random_angle_config()
        else:
            self.angles = configuration

        self.joint_positions = self.forward_kinematics(xarm_joint_names, xarm_link_names)
        self.end_effector_pos = self.joint_positions[-1]
        self.optimistic_cost = 0
        self.fail_count = 0

    def forward_kinematics(self, joint_names, link_names):
        """Computes forward kinematics of the arm given the joint angles.

        Returns:
            An array of the [x, y, z] of each joint of the arm based on the node's angle configuration.
        """
        if not ik_py:
            th = {}
            for i in range(len(joint_names)):
                th[joint_names[i]] = self.angles[i]
            ret = chain.forward_kinematics(th)
            position = [ret[name].pos for name in link_names]
        else:
            position = amazon_arm_chain.forward_kinematics(angles)
        return position

    def get_link_lengths(self):
        """ Computes the link lengths of each link in the arm. """
        lengths = []
        for i in range(0, len(self.joint_positions) - 1):
            lengths.append(np.linalg.norm(self.joint_positions[i + 1] - self.joint_positions[i]))
        return lengths

    def inc_fail_count(self):
        self.fail_count = self.fail_count + 1

    def angle_within_bounds(self, angle, joint):
        return (0 < angle < self.bounds[joint][1]) or (self.bounds[joint][0] > angle > 2 * math.pi)

    def angles_within_bounds(self, angles):
        for i, angle in enumerate(angles):
            if not self.angle_within_bounds(angle, i):
                return False
        return True

    def random_angle_config(self):
        """ Returns a set of random angles within the bounds of the arm."""
        rand_angles = [0, 0, 0, 0, 0]

        for a in range(len(rand_angles)):
            rand_angles[a] = random_angle(self.bounds[a][0], self.bounds[a][1])

        return rand_angles

    def valid_configuration(self):
        """Determines if the current arm configuration of the node is a valid one.

         Returns True if none of the joints cross into the negative Y-axis.
         """
        for i in range(1, len(self.joint_positions)):
            if self.joint_positions[i][1] < self.joint_positions[0][1]:
                return False

        if not self.angles_within_bounds(self.angles):
            False
        return True

    @classmethod
    def from_point(cls, end_point, start_config=[0, 0, 0, 0, 0]):
        """Computes Inverse Kinematics from the given point either using IKPY or Kinpy
        :param end_point: the target point from which kinematics will be calculated,
                      formatted as (x,y,z)
        :return: the inverse kinematic angles
        """
        global ik_py
        if ik_py:
            angles = amazon_arm_chain.inverse_kinematics(end_point)
        else:
            angles = kp.ik.inverse_kinematics(serial_chain,
                                          kp.Transform(pos=[end_point[0], end_point[1], end_point[2]]),
                                          initial_state=start_config)
        return Node(angles)

    @classmethod
    def distance(cls, node1, node2):
        return line.distance(node1.end_effector_pos, node2.end_effector_pos)


def random_angle(left_bound, right_bound):
    """ Generates a random angle from (left_bound, 2pi) and (0, right_bound). """

    delta_a = math.pi * 2 - left_bound
    delta_b = right_bound
    if (delta_a == 0 and delta_b == 0) or (np.random.rand() < delta_a / (delta_a + delta_b)):
        return random.uniform(left_bound, math.pi * 2)
    else:
        return random.uniform(0, right_bound)

def thresholdCheck(a, b, threshold):
    dist = abs(a)-abs(b)
    if -threshold < dist and dist < threshold:
        return True
    #print("Off by", dist)
    return False

if __name__ == '__main__':
    start_point = [0, 0, 0]
    sucess = 0
    tests = 1
    start_time = time.time()
    for i in range(tests):
        fail = False
        randomX = random.uniform(-.08,.08)
        randomY = random.uniform(-.08,.08)
        randomZ = random.uniform(0,.105)
        end_point = [randomX, randomY, randomZ]
        angles = amazon_arm_chain.inverse_kinematics(end_point)
        final_position = amazon_arm_chain.forward_kinematics(angles)
        if not thresholdCheck(randomX,final_position[0][3], .001):
            fail = True
            print("Fail in X")
        if not thresholdCheck(randomY,final_position[1][3], .001):
            fail = True
            print("Fail in Y")
        if not thresholdCheck(randomZ,final_position[2][3], .001):
            fail = True
            print("Fail in Z")
        if not fail:
            sucess += 1
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    # Set the limits for the range of plotted values
    lim = .6
    plt.xlim(-lim, lim)
    plt.ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    # create the prism to be avoided (hard coded)
    lim = .3
    run_time = time.time() - start_time
    print("Successes: ", sucess)
    print("Failures: ", tests - sucess)
    print("Rate: ", sucess/tests)
    print("Total Run Time: ", run_time)
    print("Average Run Time: ", run_time/tests)



# you put in angle configs to forward kinematics and get out an ending point
# you put in an ending point to inverse kinematics and get angle configs

"""Representation of a node in an RRT graph, which represents a single arm configuration.

Represents a single configuration of the precision arm using the five joint angles. Specifications of the arm itself are
held in models/SimpleArmModelForURDF.urdf.

Written by Simon Kapen '24, Spring 2021.
"""
from .util.error_handling import nostderr
import numpy as np
import math
import random
import kinpy as kp
from .util import line
from typing import List

# Global arm configuration - IMPORTANT: wraps with nostderr() to hide command line errors.
with nostderr():
    chain = kp.build_chain_from_urdf(open("c1c0_object_detection/kinematics/models/SimpleArmModelforURDF.urdf").read())
    serial_chain = kp.build_serial_chain_from_urdf(open("c1c0_object_detection/kinematics/models/SimpleArmModelforURDF.urdf").read(), "link5", "base_link")
    # chain = kp.build_chain_from_urdf(open("c1c0_object_detection/kinematics/models/XArm.urdf").read())
    # serial_chain = kp.build_serial_chain_from_urdf(open("c1c0_object_detection/kinematics/models/XArm.urdf").read(), "link5", "base_link")

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
    a0_bounds = (3 * math.pi / 2, .9 * math.pi / 2)
    a1_bounds = (3 * math.pi / 2, 1.22173)
    a2_bounds = (1.7 * math.pi, 2.8 * math.pi / 2)
    a3_bounds = (0, 2 * math.pi)
    a4_bounds = (0, 2 * math.pi)

    null = (2 * math.pi, 0)
    bounds = [a0_bounds, a1_bounds, a2_bounds, a3_bounds, a4_bounds]

    def __init__(self, configuration: List[float]):
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
        th = {}
        for i in range(len(joint_names)):
            th[joint_names[i]] = self.angles[i]

        ret = chain.forward_kinematics(th)

        angles = [ret[name].pos for name in link_names]
        return angles

    def get_link_lengths(self):
        """ Computes the link lengths of each link in the arm. """
        lengths = []
        for i in range(0, len(self.joint_positions)-1):
            lengths.append(np.linalg.norm(self.joint_positions[i+1] - self.joint_positions[i]))
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

        for a in range(0, 5):
            rand_angles[a] = random_angle(self.bounds[a][0], self.bounds[a][1])

        return rand_angles

    def valid_configuration(self):
        """Determines if the current arm configuration of the node is a valid one.

         Returns True if none of the joints cross into the negative Y-axis.
         """
        for i in range(1, len(self.joint_positions)):
            if self.joint_positions[i][1] < self.joint_positions[0][1]:
                # print("joint positions not in bounds")
                return False

        if not self.angles_within_bounds(self.angles):
            False
        return True

    @classmethod
    def from_point(cls, point, start_config=[0, 0, 0, 0, 0]):
        """ Uses inverse kinematics to calculate a node given its cartesian coordinates. """
        angle_config = kp.ik.inverse_kinematics(serial_chain, kp.Transform(pos=[point[0], point[1], point[2]]),
                                                initial_state=start_config)

        return Node(angle_config)

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

"""Representation of a node in an RRT graph, which represents a single arm configuration.

Represents a single configuration of the precision arm using the five joint angles. Specifications of the arm itself are
held in models/SimpleArmModelForURDF.urdf.

Written by Simon Kapen '24, Spring 2021.
"""
import numpy

from util.error_handling import nostderr
import numpy as np
import math
import random
import kinpy as kp
import ikpy as IKPY
from util import line
from ikpy.chain import Chain

# Global arm configuration - IMPORTANT: wraps with nostderr() to hide command line errors.
with nostderr():
    chain = kp.build_chain_from_urdf(open("models/SimpleArmModelforURDF.URDF").read())
    serial_chain = kp.build_serial_chain_from_urdf(open("models/SimpleArmModelforURDF.URDF").read(), "hand_1",
                                                   "base_link")
    bruh_chain = Chain.from_urdf_file("models/SimpleArmModelforURDF.URDF")


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

    def __init__(self, configuration: list[float]):
        if configuration is None:
            self.angles = self.random_angle_config()
        else:
            self.angles = configuration

        self.joint_positions = self.forward_kinematics()
        self.end_effector_pos = self.joint_positions[-1]
        self.optimistic_cost = 0
        self.fail_count = 0

    def forward_kinematics(self):
        """Computes forward kinematics of the arm given the joint angles.

        Returns:
            An array of the [x, y, z] of each joint of the arm based on the node's angle configuration.
        """
        th = {'Rev2': self.angles[0], 'Rev3': self.angles[1], 'Rev4': self.angles[2], 'Rev5': self.angles[3],
              'Rev6': self.angles[4]}
        ret = chain.forward_kinematics(th)
        return [ret['link2_1'].pos, ret['link3_1'].pos, ret['link4_1'].pos, ret['link5_1'].pos, ret['hand_1'].pos]

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

        for a in range(0, 5):
            rand_angles[a] = random_angle(self.bounds[a][0], self.bounds[a][1])

        return rand_angles

    def valid_configuration(self):
        """Determines if the current arm configuration of the node is a valid one.

         Returns True if none of the joints cross into the negative Y-axis.
         TODO: also return true if the bounds are correct.
         """
        for i in range(0, len(self.joint_positions)):
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


def test_from_point(point, start_config=[0, 0, 0, 0, 0]):
    # Uses inverse kinematics to calculate a node given its cartesian coordinates.
    angle_config = kp.ik.inverse_kinematics(serial_chain, kp.Transform(pos=[point[0], point[1], point[2]]),
                                            initial_state=start_config)
    return Node(angle_config)


def kinpy_from_point(point, start_config=[0, 0, 0, 0, 0]):
    """ Uses inverse kinematics to calculate a node given its cartesian coordinates. """
    angle_config = kp.ik.inverse_kinematics(serial_chain, kp.Transform(pos=[point[0], point[1], point[2]]),
                                                initial_state=start_config)
    return Node(angle_config)

def ikpy_from_point(point, start_config=[0, 0, 0]):
    """ Uses inverse kinematics to calculate a node given its cartesian coordinates. """
    th = {-1: 'revolute', 0: 'revolute', 1: 'revolute', 2: 'revolute', 3: 'revolute'}
    angle_config = IKPY.chain.ik.inverse_kinematic_optimization(serial_chain,point,start_config)
    return Node(angle_config)

def thresholdCheck(a, b, threshold):
    dist = abs(a)-abs(b)
    if -threshold < dist and dist < threshold:
        return True
    #print("Off by", dist)
    return False


if __name__ == '__main__':
    # this test will involve going from point (0,0,0) to (1,2,3)
    start_point = [0, 0, 0]
    sucess = 0
    tests = 1
    for i in range(tests):
        fail = False
        randomX = random.uniform(-.5,.5)
        randomY = random.uniform(-.5,.5)
        randomZ = random.uniform(-.5,.5)
        #print("Target X: ", randomX)
        #print("Target Y: ", randomY)
        #print("Target Z: ", randomZ)
        end_point = [randomX, randomY, randomZ]
        end_point = [.1,.2,.3]
        angles = bruh_chain.inverse_kinematics(end_point)
        print(angles)
        final = bruh_chain.forward_kinematics(angles)
        print(final)
        #ikpy_angle_configs = ikpy_from_point(end_point)
        #print("Kinpy Angles", kinpy_angle_configs.angles)
        #kinpy_forward = kinpy_angle_configs.forward_kinematics()
    #     if not thresholdCheck(randomX,kinpy_forward[4][0], .05):
    #         fail = True
    #     if not thresholdCheck(randomY,kinpy_forward[4][1], .05):
    #         fail = True
    #     if not thresholdCheck(randomZ,kinpy_forward[4][2], .05):
    #         fail = True
    #     if not fail:
    #         sucess += 1
    # print("Successes: ", sucess)
    # print("Failures: ", tests - sucess)
    # print("Rate: ", sucess/tests)



# you put in angle configs to forward kinematics and get out an ending point
# you put in an ending point to inverse kinematics and get angle configs

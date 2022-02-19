"""
Representation of a node in an RRT graph, which represents a single arm configuration.

Represents a single configuration of the precision arm using the five joint angles. Specifications of the arm itself are
held in the arm's URDF file.

Written by Simon Kapen, Spring 2021.
"""
from error_handling import nostderr
import numpy as np
import math
import random
import kinpy as kp

# Global arm configuration
with nostderr():
    chain = kp.build_chain_from_urdf(open("models/SimpleArmModelforURDF.urdf").read())
    serial_chain = kp.build_serial_chain_from_urdf(open("models/SimpleArmModelforURDF.urdf").read(), "hand_1",  "base_link")


class RRTNode(object):
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
        fail_count: An integer counting the number of times the extension heuristic has failed from this node.
    """

    def __init__(self, configuration: list[float]):
        a0_bounds = (math.pi / 2, 3 * math.pi / 2)
        a1_bounds = (math.pi / 2, 3 * math.pi / 2)
        a2_bounds = (math.pi / 2, 3 * math.pi / 2)
        a3_bounds = (math.pi / 2, 3 * math.pi / 2)
        a4_bounds = (math.pi / 2, 3 * math.pi / 2)

        self.bounds = [a0_bounds, a1_bounds, a2_bounds, a3_bounds, a4_bounds]

        if configuration is None:
            self.angles = self.random_angle_config()
        else:
            self.angles = configuration

        self.joint_positions = self.forward_kinematics()
        self.end_effector_pos = self.joint_positions[-1]
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
        for i in range(0, len(self.joint_positions)-1):
            lengths.append(np.linalg.norm(self.joint_positions[i+1] - self.joint_positions[i]))
        return lengths

    def inc_fail_count(self):
        self.fail_count = self.fail_count + 1

    def angle_within_bounds(self, angle, joint):
        return 0 < angle < self.bounds[joint][0] or self.bounds[joint][1] < angle < 2 * math.pi

    def angles_within_bounds(self, angles):
        for i, angle in enumerate(angles):
            if not self.angle_within_bounds(angle, i):
                return False
        return True

    def random_angle_config(self):
        """ Returns a set of random angles within the bounds of the arm. """
        rand_angles = [0, 0, 0, 0, 0]

        for a in range(0, 5):
            rand_angles[a] = random.uniform(self.bounds[a][0], self.bounds[a][1])

        return rand_angles

    @classmethod
    def from_point(cls, point, start_config=[0, 0, 0, 0, 0]):
        """ Uses inverse kinematics to calculate a node given its cartesian coordinates. """
        angle_config = kp.ik.inverse_kinematics(serial_chain, kp.Transform(pos=[point[0], point[1], point[2]]),
                                                initial_state=start_config)

        return RRTNode(angle_config)



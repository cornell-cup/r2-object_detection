"""
Written by Simon Kapen, Spring 2021.
"""

import numpy as np
import collision_detection
import kinpy as kp

# Global arm configuration
chain = kp.build_chain_from_urdf(open("models/SimpleArmModelforURDF.urdf").read())


class RRTNode(object):
    """
    A node generated in a RRT graph.
    Args:
        configuration: list of joint angles in radians. Corresponds to the 6 degrees of freedom on the arm.
            a1-- the pan angle of the base
            a2-- the lift angle of the base
            a3-- the lift angle of the elbow
            a4-- the pan angle of the elbow
            a5-- the pan angle of the wrist
            (a6-- how big to open the end effector -- not programmed)
    Attributes:
        end_effector_pos [np array] : [x, y, z] of the end effector.
        angles           [np array] : list of joint angles in radians. [a1 ... a6].
        fail_count        [int]      : amount of times extension heuristic has failed from this node.
    """

    def __init__(self, configuration):
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

    def to_nlinkarm(self):
        """ Creates an instance of NLinkArm using the angle configuration. Used for collision detection. """
        arm = collision_detection.NLinkArm(5, self.get_link_lengths(), self.joint_positions)
        arm.update_pitch([self.angles[0], self.angles[2]])
        arm.update_yaw([self.angles[1], self.angles[3]])

        return arm

    def get_link_lengths(self):
        """ Computes the link lengths of each link in the arm. """
        lengths = []
        for i in range(0, len(self.joint_positions)-1):
            lengths.append(np.linalg.norm(self.joint_positions[i+1] - self.joint_positions[i]))
        return lengths

    def inc_fail_count(self):
        self.fail_count = self.fail_count + 1

import numpy as np
from kinematics import FK
import visualization
import collision_detection


class RRTNode(object):
    """
    A node generated in a RRT graph.
    Args:
        configuration: list of joint angles in radians. Corresponds to the 6 degrees of freedom on the arm.
            a1-- the lift angle of the base
            a2-- the pan angle of the base
            a3-- the lift angle of the elbow
            a4-- the pan angle of the elbow
            a5-- the pan angle of the wrist
            a6-- how big to open the end effector
    Instance Attributes:
        end_effector_pos [np array] : [x, y, z] of the end effector.
        angles           [np array] : list of joint angles in radians. [a1 ... a6].
        fail_count        [int]      : amount of times extension heuristic has failed from this node.
    """

    def __init__(self, configuration):
        self.angles = configuration
        self.joint_positions = self.generate_joint_positions()
        self.end_effector_pos = self.joint_positions[1]
        self.fail_count = 0

    def forward_kinematics(self, link_lengths):
        """ Returns the [x, y, z] corresponding the node's angle configuration. """

        angles = np.array([[0], [self.angles[1]], [0], [self.angles[0]], [0]])
        alpha = np.array([[self.angles[3]], [0], [0], [self.angles[2]], [0]])
        r = np.array([[0], [link_lengths[1]], [link_lengths[2]], [0], [0]])
        d = np.array([[link_lengths[0]], [0], [0], [0], [link_lengths[3]]])
        return FK(angles, alpha, r, d)

    def generate_joint_positions(self):
        """ Returns the [x, y, z] of the two joints based on the node's angle configuration. """
        return visualization.joint_positions(*self.angles[0:4])

    def to_nlinkarm(self):
        """ Creates an instance of NLinkArm using the angle configuration. Used for collision detection. """
        arm = collision_detection.NLinkArm(2, [.222, .3])
        arm.update_pitch([self.angles[0], self.angles[2]])
        arm.update_yaw([self.angles[1], self.angles[3]])

        return arm

    def inc_fail_count(self):
        self.fail_count = self.fail_count + 1
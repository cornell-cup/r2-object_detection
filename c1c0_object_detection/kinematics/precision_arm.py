import math


class PrecisionArm:
    """
    Representation of C1C0's precision arm.

    link1_len: length of first segment
    link2_len: length of second segment

    th1: yaw of base
    th2: pitch of base joint
    th3: pitch of elbow joint
    th4: roll of wrist
    th5: up/down of wrist
    th6: closing of claw

    Each angle must be constrained by [bounds[a][0] < angle < 2pi OR 0 < angle < bounds[a][1]]
    For arm path planning purposes we only care about th1, th2, th3.
    """

    def __init__(self):
        # will probably need new measurements of these
        link1_len = 0.066675
        link2_len = 0.104775

        th1_bounds = (math.pi * 3 / 8, math.pi * 5 / 8)
        th2_bounds = (math.pi * 3 / 8, math.pi * 5 / 8)
        th3_bounds = (math.pi * 3 / 8, math.pi * 5 / 8)
        # th4_bounds = (math.pi * 3 / 2, math.pi / 2)

        self.bounds = [th1_bounds, th2_bounds, th3_bounds, th3_bounds, th3_bounds, th3_bounds]

    def angle_within_bounds(self, angle, joint):
        return 0 < angle < self.bounds[joint][0] or self.bounds[joint][1] < angle < 2 * math.pi

    def angles_within_bounds(self, angles):
        for i, angle in enumerate(angles):
            if not self.angle_within_bounds(angle, i):
                return False
        return True

    def fk(self, angles):
        """ Returns the position of the end effector given the angles. """
        # Hold off on this until we have the URDF file?


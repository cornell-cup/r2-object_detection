# README: The purpose of the constraints code is to design a function that, when given the specifications of the
# precision arm.
import math
import sys
import kinematics

#  an ArmSixDOF object is a representation of the precision arm with six degrees of freedom.
#  Requires: 2 arm lengths (valid floats) and 5 joint constraints (valid float degrees).
#  Because the last degree of freedom is for the end effector (the wrist), it isn't necessary to specify here.
#
#  length1 is the length of the arm segment closest to the arm's mount, length 2 is the arm segment that's further away.
#  j1-5 are the joint angle constraints. The class itself also encodes more joint constraints given the current states
#  of the joints.
#  As of March 7th, 2020, joint angles j1, j4, and j5 aren't used to check for valid configurations because the
#  rotations that they create won't make a valid configuration invalid.

# length of the upper arm, the arm segment closest to the base of the robotic arm, is 222 mm as of March 2020.
upper_arm = .222

# length of the forearm, the arm segment furthest away from the base of the robotic arm, is 300 mm as of March 2020.
fore_arm = .300

# current angle constraints of all joints are 0 to 360 degrees. Change the variable below if this changes.
angle_c = 360


class ArmSixDOF():
    def __init__(self, length1, length2, j1, j2, j3, j4, j5, j6):
        self.l1 = length1
        self.l2 = length2
        self.j1 = j1
        self.j2 = j2
        self.j3 = j3
        self.j4 = j4
        self.j5 = j5
        self.j6 = j6

    #  checks to see if a given angle config for the arm is valid/meets constraints.
    #  a5, a6 involve the end effector, which is adjusted by the object detection code. For now, we don't check for
    #  the validation of these joint angles
    #  configuration, so valid_configuration doesn't check for that.
    #  a2's maximum range is from 0-180. However, it's limited by a3's angle.
    #  a3's maximum range is from 0-360. However, it's limited by a2's angle.
    def valid_configuration(self, a1, a2, a3, a4, a5, a6):
        #  for a given a1, an a2 is always valid. However, the a3 is not necessarily valid:
        #  use spherical coordinates for validity
        if self.l1 * math.cos(math.radians(a2)) < 0:
            return False, None
        if self.l2 * math.cos(math.radians(a3)) + self.l1 * math.cos(math.radians(a2)) < 0:
            return False, None
        return True, (a1 + 360) % self.j1, (a2 + 360) % self.j2, \
               (a3 + 360) % self.j3, (a4 + 360) % self.j4, \
               (a5 + 360) % self.j5, (a6 + 360) % self.j6

    def point_in_space(self, a1, a2, a3, a4, a5, a6):
        v_config = self.valid_configuration(a1, a2, a3, a4, a5, a6)
        if not v_config:
            print("Invalid Configuration")
            sys.exit(0)
        else:
            return (self.l1 * math.sin(math.radians(a2)) * math.cos(math.radians(a1)) +
                    self.l2 * math.sin(math.radians(a4)) * math.cos(math.radians(a3)),
                    self.l1 * math.sin(math.radians(a2)) * math.sin(math.radians(a1)) +
                    self.l2 * math.sin(math.radians(a4)) * math.sin(math.radians(a3)),
                    self.l1 * math.cos(math.radians(a2)) + self.l2 * math.cos(math.radians(a4)))


def run_km(x, y, z):
    return [round(x[0], 2) for x in kinematics.kinematics(x, y, z).tolist()]


def in_range(x, y, z):
    return math.sqrt(x*x + y*y + z*z) > 0.522


def constraints(a1, a2, a3, a4, a5, a6, x, y, z):
    arm = ArmSixDOF(upper_arm, fore_arm, angle_c, angle_c, angle_c, angle_c, angle_c, angle_c)
    angles = run_km(x, y, z)
    angles.append(0.0)
    config = arm.valid_configuration(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5])
    if config[0]:
        return config[1]
    else:
        points = [(1, (x - .15, y, z)), (2, (x, y - .15, z)), (3, (x, y, z - .15)),
                  (4, (x + .15, y, z)), (5, (x, y + .15, z)), (6, (x, y, z + .15))]
        while not config[0]:
            pair = points.pop(0)
            point = pair[1]
            x = point[0]
            y = point[1]
            z = point[2]
            if not in_range(x, y, z):
                continue
            config = run_km(x, y, z)
            if pair[0] == 1:
                points.append((1, (x - .15, y, z)))
            elif pair[0] == 2:
                points.append((2, (x, y - .15, z)))
            elif pair[0] == 3:
                points.append((3, (x, y, z - .15)))
            elif pair[0] == 4:
                points.append((4, (x + .15, y, z)))
            elif pair[0] == 5:
                points.append((5, (x, y + .15, z)))
            elif pair[0] == 6:
                points.append((6, (x, y, z + .15)))


# later on, need to input initial angles, or the first 6 inputs, plus the 3 inputs of the coordinates to get to
constraints(0, 0, 0, 0, 0, 0, 0, 0, 0.438)

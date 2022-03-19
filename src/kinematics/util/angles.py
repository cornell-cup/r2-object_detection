"""
A collection of miscellaneous angle math functions.

Written by Simon Kapen, Spring 2022.
"""
import math


def true_angle_distance(angle_1, angle_2):
    """ Returns [angle_2] - [angle_1], accounting for angle wrap.
            Example: true_angle_distance(PI/6, 11PI/6) is PI/3, not 5PI/3. """

    difference = angle_2 - angle_1
    if difference > math.pi:
        difference = -(2 * math.pi - angle_2 + angle_1)
    elif difference < -math.pi:
        difference = 2 * math.pi - angle_1 + angle_2

    return difference


def true_angle_distances_arm(angles_1, angles_2):
    """ Returns [angles_2] - [angles_1], accounting for angle wrap. Calculates each angle in an arm configuration.
        Example: true_angle_distance(PI/6, 11PI/6) is PI/3, not 5PI/3. """
    new_angles = []

    for angle1, angle2 in zip(angles_1, angles_2):
        new_angles.append(true_angle_distance(angle1, angle2))

    return new_angles

# Kinematics

##Overview
The kinematics section of the pipeline aims to find an obstacle-free
path from the arm's current configuration to a configuration that allows it
to grab a target object. It takes as input the current angle configuration of the arm,
the (x, y, z) of the target object, and a list of obstacles.

By default, the algorithm first generates a configuration at the target point with inverse kinematics.
It then tries to generate a linear path between the start (current) configuration and
the generated configuration. If this path runs through an obstacle, it uses the Optimistic
Predictive Cost (OPC) algorithm to generate a new path that navigates around this
obstacle.

## Usage
To generate a path to a desired end effector position, use **linear_path_to_point()**
in linear_pathing.py.

An example of this library being used on an arm is found in just_kinematics.py of the root directory.

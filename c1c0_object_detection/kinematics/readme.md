# Kinematics

## Overview
The kinematics section of the pipeline aims to find an obstacle-free
path from the arm's current configuration to a configuration that allows it
to grab a target object. It takes as input the current angle configuration of the arm,
the (x, y, z) of the target object, and a list of obstacles.

By default, the algorithm first generates a configuration at the target point with inverse kinematics.
It then tries to generate a linear path between the start (current) configuration and
the generated configuration. If this path runs through an obstacle, it uses the Optimistic
Predictive Cost (OPC) algorithm to generate a new path that navigates around this
obstacle.

More information can be found in documentation from past semesters (access restricted to Cornell emails):

<a href="https://docs.google.com/document/d/1DVMR_y_O1Raq-9c0Zfw2L_Gg5T_n_UDTKhcRfCJb1LA/edit?usp=sharing">Fall 2020<a>
Initial setup of forward kinematics, inverse kinematics (deprecated due to ROS dependencies)

<a href="https://docs.google.com/document/d/1uidd3vGvraIsbQjkBHtCVNxngc9dI24RcHmMMUQ-jCU/edit?usp=sharing">Spring 2021<a>
Start of more lightweight solution. Graph structures and RRT algorithm

<a href="https://docs.google.com/document/d/1k5nwPK5oHX843uns-RPDJom2b_Te1UXszvR2c3l71i8/edit?usp=sharing">Fall 2021<a>
Arm bounds setup, and URDF integration with forward kinematics

<a href="https://docs.google.com/document/d/1lKmLhwX3u8OweMDbpeXeGfEl70mxHf-pX8w6sGOIEJo/edit?usp=sharing">Spring 2022<a>
Inverse kinematics, OPC algorithm, path optimizer
<br>
<br>

## Usage
To generate a path to a desired end effector position, use **linear_path_to_point()**
in linear_pathing.py.

An example of this library being used on an arm is found in just_kinematics.py of the root directory.

## Next Steps
1. Determine the correspondences between the joints on the physical arm and the 
joints in the representation described by the Node class (in arm_node.py)
2. Develop test cases with obstacles in various locations, and test on the physical arm
3. Determine the bounds on each angle of the precision arm and update the bounds
in arm_node.py accordingly
4. Test optimizers.py more rigorously to ensure that removing configurations
does not make path pass through object

If the precision arm isn't complete yet, steps 1-3 will have to be done on both the blue
(temporary test) arm, as well as the precision arm when it is complete.

## Contributors
Simon Kapen '24: <a href="sak299@cornell.edu">sak299@cornell.edu<a>

Raj Sinha '25: <a href="ys458@cornell.edu">ys458@cornell.edu<a>

Alison Duan '22: <a href="ad734@cornell.edu">ad734@cornell.edu<a>
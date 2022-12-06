#!/usr/bin/python3

import math
import numpy as np
import sys

import c1c0_object_detection.arm.publish_arm_updates as arm
import c1c0_object_detection.kinematics.linear_pathing as alr
from c1c0_object_detection.kinematics import arm_plot
from c1c0_object_detection.kinematics.arm_node import Node
import matplotlib.pyplot as plt
import math
WIDTH = 640
HEIGHT = 480

TARGET_POINT = [0.0, 0.0, 0.5]

if __name__ == '__main__':
    detections = []
    arm.init_serial()

    for i in range(5):
        
        print("serial port initialized")
        startpos = arm.read_encoder_values()
        startpos = [i*math.pi/180 for i in startpos]
        
        print("arm vals read")
        print("startpos:", startpos)
        # inverse kinematics
        # avg = [(gripper_pt1_arm[i][0] + gripper_pt2_arm[i][0])/2
        #                 for i in range(len(gripper_pt1_arm))]
        avg = TARGET_POINT
        print("target calculated", avg)
        # endpos = RRTNode.from_point(avg_target, startpos)
        # print("endpos:", end_angles)
        # arm_config, success = alr.linear_path_to_point(startpos, avg[2], avg[1], avg[0], [], 5)
        arm_config, success = alr.linear_path_to_angles([0,0,0,0,0,0], [0,0,0,0,0,0], [], 2)
        print([node.angles for node in arm_config])
        print("success:", success)
        # send arm_config to the arm to move
        ax = plt.axes(projection='3d')
        arm_plot.configure_graph(ax)
        arm_plot.plot_arm_configs(ax, arm_config, [])
        plt.savefig('arm_path.png')
        # plt.show()
        print(arm_config)
        if success:
            for config in arm_config:
                converted_array = alr.radians_to_degrees(config)[::-1]
                print("WRITING ARM CONFIG", converted_array)
                arm.publish_updates([0] + converted_array[:-1], 3)
            print("arm config serial written")
            break

    arm.close_serial()
    """
    try:
        if success:
            for config in arm_config:
                print("WRITING ARM CONFIG", config.angles)
                arm.writeToSerial(config.angles.astype(int))
        print("arm config serial written")
        arm.close_serial()
    except Exception as e:
        print("error in writing to arm config")
        print(e) 
        arm.close_serial() 


    """


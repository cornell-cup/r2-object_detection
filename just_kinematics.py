#!/usr/bin/python3

import math
import numpy as np
import sys

sys.path.insert(1, '/usr/local/lib/python3.6')
sys.path.insert(2, '/home/cornellcup-cs-jetson/Desktop/c1c0-modules/r2-object_detection/src/kinematics')
sys.path.insert(3, '/home/cornellcup-cs-jetson/Desktop/c1c0-modules/r2-object_detection/src')

from src.camera import Camera
from src.projections import *
from networking.Client import Client
import src.arm.publish_arm_updates as arm 
import src.kinematics.assuming_linearity_rrt as alr 

WIDTH = 640
HEIGHT = 480

TARGET_POINT = [0.1, 0.1, 0.1]

if __name__ == '__main__':
    detections = []
    arm.init_serial()

    for i in range(5):
        
        print("serial port initialized")
        startpos = arm.read_encoder_values()
        print("arm vals read")
        # inverse kinematics
        # avg = [(gripper_pt1_arm[i][0] + gripper_pt2_arm[i][0])/2
        #                 for i in range(len(gripper_pt1_arm))]
        avg = TARGET_POINT
        print("target calculated", avg)
        # endpos = RRTNode.from_point(avg_target, startpos)
        arm_config, success = alr.linear_rrt_to_point(startpos, avg[0], avg[1], avg[2], [], 1000)
        # send arm_config to the arm to move
        if success:
            for config in arm_config:
                converted_array = alr.radians_to_degrees(config)
                print("WRITING ARM CONFIG", converted_array)
                arm.publish_updates(converted_array, 0.5)
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


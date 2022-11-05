#!/usr/bin/python3
import time

import c1c0_object_detection.kinematics.linear_rrt as alr
from networking.Server import Server
from c1c0_object_detection.object_detection.kmeans import *
import sys 
# for displaying
import math

"""Run object detection pipeline with the intel realsense camera"""

WIDTH = 640
HEIGHT = 480
DISPLAY = False

def print_time(msg, start):
    print(msg, time.time()-start," s")
    return time.time()


def main():
    robot = Server()

    color_img, depth_img, dgr, startpos, bbox, coord1, coord2 = robot.receive_data()
    
    # Identify Obstacles with K-means

    bounds = get_image_bounds(color_img, depth_img)
    # list of bounding boxes, each bounding box has bottom left coordinate, lwh
    collision_coords = bound_to_coor(cam.depth_scale, depth_frame, depth_img, bounds, cam)
    print(collision_coords)

    # --------- Send Arm Configs to the Arm to move ---------

    # inverse kinematics
    avg = [(coord1[i][0] + coord2[i][0])/2
                    for i in range(len(coord1))]
    start_time = print_time("Read Encoder Values: ", start_time)
    print("target calculated", avg)
    arm_config, success = alr.linear_rrt_to_point(startpos, avg[2], avg[1], avg[0], collision_coords, 5)
    robot.send_update([arm_config, success])
    start_time = print_time("Calculated Kinematics: ", start_time)
    print("converted config: ", avg) 
    print(arm_config[0].angles)
    print(arm_config[-1].angles)

    

if __name__ == '__main__':
    main()

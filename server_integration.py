#!/usr/bin/python3
import cv2
import numpy as np
import time

from c1c0_object_detection.object_detection.camera import Camera
from c1c0_object_detection.object_detection.inference import Inference
from c1c0_object_detection.object_detection.grasping import Grasping
import c1c0_object_detection.arm.publish_arm_updates as arm
import c1c0_object_detection.kinematics.linear_rrt as alr
import networking.Server as Server
import sys 
# for displaying
import jetson.utils
import math

"""Run object detection pipeline with the intel realsense camera"""

WIDTH = 640
HEIGHT = 480
DISPLAY = False

def print_time(msg, start):
    print(msg, time.time()-start," s")
    return time.time()


def main():
    grasping = Grasping()

    robot = Server()

    color_img, depth_frame, dgr, startpos, bbox = robot.receive_data()
    depth_img = np.asanyarray(depth_frame.get_data())
    
    start_time = print_time("Detections: ", start_time)
    # --------- Locate where to Grab the Target Object ---------
    isReachable, isGrabbable, coord1, coord2 = grasping.locate_object(
        dgr, bbox, depth_frame, display=False)
    if not isReachable:
        print("object not reachable")
        sys.exit(0)
    if not isGrabbable:
        print("object too large to grasp")
        sys.exit(0)
    print("Grasp coordinates in meters (X, Y, Z): ", coord1, coord2)

    start_time = print_time("Calculated Grasps: ", start_time)
    # Identify Obstacles?

    # - Send Grasp Coordinates to Base Station to compute Arm Configs -
    # robot.send_data()
    # arm_config = robot.listen()
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
    arm_config, success = alr.linear_rrt_to_point(startpos, avg[2], avg[1], avg[0], [], 5)
    robot.send_update([arm_config, success])
    start_time = print_time("Calculated Kinematics: ", start_time)
    print("converted config: ", avg) 
    print(arm_config[0].angles)
    print(arm_config[-1].angles)

    

if __name__ == '__main__':
    main()

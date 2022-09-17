#!/usr/bin/python3
import cv2
import numpy as np

from c1c0_object_detection.object_detection.camera import Camera
from c1c0_object_detection.object_detection.inference import Inference
from c1c0_object_detection.object_detection.grasping import Grasping
import c1c0_object_detection.arm.publish_arm_updates as arm
import c1c0_object_detection.kinematics.linear_rrt as alr
# for displaying
import jetson.utils
import math

"""Run object detection pipeline with the intel realsense camera"""

WIDTH = 640
HEIGHT = 480
DISPLAY = False

def main():
    with Camera(WIDTH, HEIGHT) as cam:
        # TODO: Do we need context managers for these also?
        inf = Inference()
        grasping = Grasping()

        for i in range(5):
            # --------- Get Frames and Numpy Images ---------
            try:
                color_frame, depth_frame = cam.get_frames()
                # Validate that both frames are not None and depth is not all 0
                # TODO: figure out why depth is weird sometimes
                if not depth_frame.is_frame() or not color_frame.is_frame():
                    print("frames not captured")
                    continue
            except RuntimeError:
                print("Couldn't get frames in time")
                continue

            color_img, depth_img, dgr = cam.get_imgs_from_frames(
                color_frame, depth_frame, display=DISPLAY)
            
            # --------- Identify if Target Object is in View ---------
            isFound, top, bot, left, right = inf.detect_object(
                color_img, "bottle", display=DISPLAY)
            if not isFound:
                print("object not found")
                continue

            bbox = (max(0, left-20), min(WIDTH-1, right+20),
            max(0, top), min(HEIGHT-1, bot))
            
            # --------- Locate where to Grab the Target Object ---------
            isReachable, isGrabbable, coord1, coord2 = grasping.locate_object(
                dgr, bbox, depth_frame, display=True)
            if not isReachable:
                print("object not reachable")
                continue
            if not isGrabbable:
                print("object too large to grasp")
                continue
            print("Grasp coordinates in meters (X, Y, Z): ", coord1, coord2)

            key = cv2.waitKey(0) # display results
            # TODO: should this be moved after all the rest of the code?
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                continue
            if key & 0xFF == ord('r'):
                cv2.destroyAllWindows()

            # Identify Obstacles?

            # - Send Grasp Coordinates to Base Station to compute Arm Configs -
            # robot.send_data()
            # arm_config = robot.listen()

            # --------- Send Arm Configs to the Arm to move ---------
            arm.init_serial()
            print("serial port initialized")
            startpos = arm.read_encoder_values()
            startpos = [i*math.pi/180 for i in startpos]
            print("arm vals read")
            # inverse kinematics
            avg = [(coord1[i][0] + coord2[i][0])/2
                          for i in range(len(coord1))]
            print("target calculated", avg)
            arm_config, success = alr.linear_rrt_to_point(startpos, avg[2], avg[1], avg[0], [], 5)
            print("converted config: ", avg) 
            print(arm_config[0].angles)
            print(arm_config[-1].angles)
            # send arm_config to the arm to move
            if success:
                for config in arm_config[2:]:
                    converted_array = alr.radians_to_degrees(config)[::-1]
                    print("WRITING ARM CONFIG", converted_array)
                    converted_array[0] = 10
                    converted_array[1] = 0
                    arm.publish_updates(converted_array, 1)
                c = arm_config[-1]
                conv = alr.radians_to_degrees(c)[::-1]
                conv[0] = 110
                conv[1] = 0
                arm.publish_updates(conv, 1)
                for config in arm_config[::-1][:-2]:
                    converted_array = alr.radians_to_degrees(config)[::-1]
                    print("WRITING ARM CONFIG", converted_array)
                    converted_array[0] = 110

                    converted_array[1] = 0
                    arm.publish_updates(converted_array, 1)
            print("arm config serial written")
            arm.close_serial()
            break

if __name__ == '__main__':
    main()

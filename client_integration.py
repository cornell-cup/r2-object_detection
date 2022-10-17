#!/usr/bin/python3
import time

from c1c0_object_detection.object_detection.camera import Camera
from c1c0_object_detection.object_detection.grasping import Grasping
from c1c0_object_detection.object_detection.inference import Inference
import c1c0_object_detection.arm.publish_arm_updates as arm
import c1c0_object_detection.kinematics.linear_rrt as alr
from networking.Client import Client
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
    start_time = time.time()
    with Camera(WIDTH, HEIGHT) as cam:
        # TODO: Do we need context managers for these also?
        inf = Inference()
        grasping = Grasping()
        robot = Client() 
        start_time = print_time("Loaded Cam, Inf, and Grasp Modules: ", start_time)
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
                color_img, "teddy bear", display=DISPLAY)
            if not isFound:
                print("object not found")
                continue

            bbox = (max(0, left-20), min(WIDTH-1, right+20),
            max(0, top), min(HEIGHT-1, bot))
            
            start_time = print_time("Detections: ", start_time)

            # --------- Locate where to Grab the Target Object ---------
            isReachable, isGrabbable, coord1, coord2 = grasping.locate_object(
                dgr, bbox, depth_frame, display=False)
            if not isReachable:
                continue
            if not isGrabbable:
                continue

            print("Grasp coordinates in meters (X, Y, Z): ", coord1, coord2)

            start_time = print_time("Calculated Grasps: ", start_time)

            print ("Starting arm...")
            arm.init_serial()
            startpos = arm.read_encoder_values()

            startpos = [i*math.pi/180 for i in startpos]
            print("arm vals read")

            kmeans_depth = np.asanyarray(depth_frame.get_data())
            robot.send_data(color_img, kmeans_depth, dgr, startpos, bbox, coord1, coord2)

            arm_config, success = robot.listen()

            # --------- Send Arm Configs to the Arm to move ---------
            if success:
                for config in arm_config:
                    print("Config: ", config)
                    # This array was reversed for the blue arm, 
                    # since it ordered the servos this way
                    converted_array = alr.radians_to_degrees(config)[::-1]
                    print("WRITING ARM CONFIG", converted_array)
                    # Hard coded end effector for demo purposes
                    converted_array[0] = 10
                    converted_array[1] = 0
                    arm.publish_updates(converted_array, 1)
                c = arm_config[-1]
                conv = alr.radians_to_degrees(c)[::-1]
                conv[0] = 110
                conv[1] = 0
                arm.publish_updates(conv, 1)
                #Below is for reversing the path and going backwords 
                for config in arm_config[::-1][:-2]:
                    converted_array = alr.radians_to_degrees(config)[::-1]
                    print("WRITING ARM CONFIG", converted_array)
                    converted_array[0] = 110

                    converted_array[1] = 0
                    arm.publish_updates(converted_array, 1)
            print("arm config serial written")
            arm.close_serial()
            start_time = print_time("Arm Finished Moving: ", start_time)

            break

if __name__ == '__main__':
    main()

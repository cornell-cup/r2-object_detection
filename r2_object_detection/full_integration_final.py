#!/usr/bin/python3
#
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import jetson.inference
import jetson.utils
import math
import cv2
import numpy as np

from camera import Camera
from projections import *

#  this is a test comment
net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = jetson.utils.videoSource("/dev/video1")      # '/dev/video0' for V4L2
display = jetson.utils.videoOutput("my_video.mp4") # 'my_video.mp4' for file

"""Run grasp detection code with the intel realsense camera"""

WIDTH = 640
HEIGHT = 480

if __name__ == '__main__':
    cam = Camera(WIDTH, HEIGHT)
    detections = []
    try:
        for i in range(5):
            while display.IsStreaming():
                img = camera.Capture()
                detections = net.Detect(img)
                display.Render(img)
                display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
                if detections:
                    break;
                #for i in detections:
                    #   print(i.ClassID, i.Confidence, i.Left, i.Right, i.Top, i.Bottom, i.Width, i.Height, i.Area, i.Center)
            try:
                #get frames
                color_frame, depth_frame = cam.get_frames()
                # Validate that both frames are not None and depth is not all 0
                # TODO: figure out why depth is weird sometimes
                if not depth_frame or not color_frame or not np.all(depth_frame == 0):
                    print("frames not captured")
                    continue
            except RuntimeError:
                print("Couldn't get frames in time")
                continue
            
            color_img, depth_img, dgr = cam.get_imgs_from_frames(color_frame, depth_frame)
            
            gripper_h = 200
            width, height = dgr.shape[1], dgr.shape[0]
            # perform grasp prediction to get 2 grasp points (x,y)
            # uses bounding box info - top left corner x,y and width/height
            # from object inference
            bbox_coords = (0, 0, width, height)
            img_pt1, img_pt2 = grasp_coords_rel_img(dgr, bbox_coords)
            print(img_pt1, img_pt2)
            print(dgr[img_pt1[1], img_pt1[0]])

            # squeeze the x,y points towards the midpoint until depth at points
            # is within some distance from the center depth
            clamp_x1, clamp_y1, clamp_x2, clamp_y2, z1, z2 = clamp_z(img_pt1, img_pt2, depth_frame)
            
            # Get 3D camera pts using x,y from grasp prediction and z from
            # clamped points
            #convert grasp coords to cam and arm coords, output is a tuple
            gripper_pt1_cam = proj_pixel_to_point(img_pt1[0], img_pt1[1], z1, depth_frame)
            gripper_pt2_cam = proj_pixel_to_point(img_pt2[0], img_pt2[1], z2, depth_frame)
            print("Grab points at\n\t", gripper_pt1_cam, "and\n\t", gripper_pt2_cam, "\nrelative to the camera")

            # output of proj_grasp_cam_to_arm is a numpy array
            gripper_pt1_arm = proj_grasp_cam_to_arm(gripper_pt1_cam)
            gripper_pt2_arm = proj_grasp_cam_to_arm(gripper_pt2_cam)
            print("Grab points at\n\t", gripper_pt1_arm, "and\n\t", gripper_pt2_arm, "\nrelative to where the arm is connected to C1C0")
            #plot grasp on image
            gripper_w = .1 #10cm
            grabbable(gripper_pt1_arm, gripper_pt2_arm, gripper_w)

            rect_points = calc_pred_rect(
                #grab points will show the canny and dgr with the grasps
                dgr, img_pt1, img_pt2, gripper_h)
            plot_pred_rect(dgr, rect_points)
            cv2.imshow("grasp", dgr)

            # colorized depth image
            cv2.imshow("original", color_img)
            cv2.imshow("white to black depth", depth_img)

            cv2.circle(depth_img, (int(clamp_x1), int(clamp_y1)), 5, (0, 0, 255), -1)
            cv2.circle(depth_img, (int(clamp_x2), int(clamp_y2)), 5, (0, 0, 255), -1)
            cv2.imshow("clamp points", depth_img)
            key = cv2.waitKey(0)

            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        cam.stop()

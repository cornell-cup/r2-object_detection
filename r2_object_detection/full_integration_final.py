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
            try:
                while display.IsStreaming():
                    img = camera.Capture()
                    detections = net.Detect(img)
                    display.Render(img)
                    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
                    if detections:
                        break;
                    #for i in detections:
                     #   print(i.ClassID, i.Confidence, i.Left, i.Right, i.Top, i.Bottom, i.Width, i.Height, i.Area, i.Center)

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
            #perform grasp prediction
            img_pt1, img_pt2 = grasp_coords_rel_img(dgr, (detections[0].Left, detections[0].Bottom, detections[0].Width, detections[0].Height))
            print(img_pt1, img_pt2)
            print(dgr[img_pt1[1], img_pt1[0]])
            #convert grasp coords to cam and arm coords
            cam_pt1 = proj_grasp_img_to_cam(img_pt1, depth_frame)
            cam_pt2 = proj_grasp_img_to_cam(img_pt2, depth_frame)
            print("Grab points at", cam_pt1, "and", cam_pt2, "relative to the camera")
            
            # keep checking pixels closer and closer to the center until the
            # depth distance between the center and edge are 1 inch or less
            distances = np.zeros((HEIGHT, WIDTH))
            # compute a distance array
            for x in range(WIDTH):
                for y in range(HEIGHT):
                    distances[HEIGHT, WIDTH] = depth_frame.get_distance(x,y)
            # unit vector from edge pixel to the midpoint, will need to do some
            ctr_x_img = (img_pt1[0] + img_pt2[0]) / 2
            ctr_y_img = (img_pt1[1] + img_pt2[1]) / 2
            for i in range(1): #calculate unit vector
                ctr_x_img - img_pt1[i]
            # interpolation to search along pixels in distance array

            # while the difference between depths is more than 2cm, move the
            # edge points on the image closer to the center
            while depth_disparity > .02 and iters < 20:
                pass

            arm_pt1 = proj_grasp_cam_to_arm(cam_pt1)
            arm_pt2 = proj_grasp_cam_to_arm(cam_pt2)
            print("Grab points at", arm_pt1, "and", arm_pt2, "relative to where the arm is connected to C1C0")
            #plot grasp on image
            rect_points = calc_pred_rect(
                #grab points will show the canny and dgr with the grasps
                dgr, img_pt1, img_pt2, gripper_h)
            plot_pred_rect(dgr, rect_points)
            cv2.imshow("grasp", dgr)

            # colorized depth image
            cv2.imshow("original", color_img)
            cv2.imshow("white to black depth", depth_img)
            key = cv2.waitKey(0)

            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        cam.stop()

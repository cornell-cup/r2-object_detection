import math
import cv2
import numpy as np

from camera import Camera
from projections import *

"""Run grasp detection code with the intel realsense camera"""

WIDTH = 640
HEIGHT = 480

if __name__ == '__main__':
    cam = Camera(WIDTH, HEIGHT)
    try:
        for i in range(5):
            
            # try:
            #     #get frames
            #     color_frame, depth_frame = cam.get_frames()
            #     # Validate that both frames are not None and depth is not all 0
            #     # TODO: figure out why depth is weird sometimes
            #     if not depth_frame or not color_frame or not np.all(depth_frame == 0):
            #         print("frames not captured")
            #         continue
            # except RuntimeError:
            #     print("Couldn't get frames in time")
            #     continue

            color_frame, depth_frame = cam.get_frames()

            color_img, depth_img, dgr = cam.get_imgs_from_frames(color_frame, depth_frame)
            
            gripper_h = 200
            width, height = dgr.shape[1], dgr.shape[0]
            #perform grasp prediction to get 2 grasp points (x,y)
            img_pt1, img_pt2 = grasp_coords_rel_img(dgr, (0, 0, width, height))
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

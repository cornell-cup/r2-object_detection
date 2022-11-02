import numpy as np
import math
import cv2
import time

from .projections import proj_pixel_to_point, proj_grasp_cam_to_arm
from .grasping_utility import (grasp_coords_rel_img, clamp_z, grabbable, 
    calc_pred_rect, plot_pred_rect)

def print_time(msg, start):
    print(msg, time.time()-start," s")
    return time.time()

class Grasping:
    """
    Attributes
        max_reach is a basic measure of how far the arm can reach. DOES NOT
            take into account the arm configuration needed to avoid obstacles
        gripper_width limits how wide the end effector can open. If the distance
            between two grasping points exceeds this, the grab is not possible.
            points 
    """

    def __init__(self, max_reach=1, gripper_width=.5,
        DIFF_X=0.12, DIFF_Y=0.0, DIFF_Z=0.1,
        # THETA_X=(115*np.pi/180), THETA_Y=(0*np.pi/180), THETA_Z=(0*np.pi/180)):
	THETA_X=0.0, THETA_Y=0.0, THETA_Z=0.0):
        # constants
        self.max_reach = max_reach
        self.gripper_width = gripper_width
        self.gripper_height = 100 #100 pixels, just for visualization
        self.DIFF_X = DIFF_X
        self.DIFF_Y = DIFF_Y
        self.DIFF_Z = DIFF_Z
        self.THETA_X = THETA_X
        self.THETA_Y = THETA_Y
        self.THETA_Z = THETA_Z

        
    def locate_object(self, dgr_img, bbox, depth_frame, display=False): # should this take a cropped image?
        """
        Args
            cropped_img is an image trimmed to a bounding box
            depth_frame is used to find the point coordinates in the real world
        Returns
            isReachable
            isGrabbable
            x, y, z grasping coords (left, right, mid?)
        """
        # TODO: how to go about displaying results?
        # Have a function to draw points on the original image?
        start_time = time.time()
        img_pt1, img_pt2 = grasp_coords_rel_img(dgr_img, bbox)
        start_time = print_time("grasp coords rel image", start_time)

        clamp_x1, clamp_y1, clamp_x2, clamp_y2, z1, z2 = clamp_z(img_pt1, img_pt2, depth_frame)
        start_time = print_time("clamp z", start_time)
        
        # TODO: Should we change grasp representation to midpoint, 
        # gripper_opening, and gripper_closing instead?
        gripper_pt1_cam = proj_pixel_to_point(img_pt1[0], img_pt1[1], z1, depth_frame)
        start_time = print_time("pixel to point", start_time)
        gripper_pt2_cam = proj_pixel_to_point(img_pt2[0], img_pt2[1], z2, depth_frame)
        
        gripper_pt1_arm = proj_grasp_cam_to_arm(gripper_pt1_cam,
            self.DIFF_X, self.DIFF_Y, self.DIFF_Z, self.THETA_X, self.THETA_Y, self.THETA_Z)
        start_time = print_time("grasp cam to arm", start_time)
        gripper_pt2_arm = proj_grasp_cam_to_arm(gripper_pt2_cam,
            self.DIFF_X, self.DIFF_Y, self.DIFF_Z, self.THETA_X, self.THETA_Y, self.THETA_Z)

        # distance from base of arm to the object is within reach?
        # isReachable = math.dist([0,0,0], gripper_pt1_arm) < self.max_reach
        isGrabbable = grabbable(gripper_pt1_arm, gripper_pt2_arm, self.gripper_width)
        start_time = print_time("grababble", start_time)
        
        if display:
            self.display_grasps(dgr_img, (clamp_x1, clamp_y1), (clamp_x2, clamp_y2))

        return True, isGrabbable, gripper_pt1_arm, gripper_pt2_arm

    def display_grasps(self, dgr, img_pt1, img_pt2):
        """Private function. Displays grasping points on a dgr image"""
        #plot grasp on image
        rect_points = calc_pred_rect(dgr, img_pt1, img_pt2, self.gripper_height)
        plot_pred_rect(dgr, rect_points)
        # TODO: Will this cause problems when running? Seems like it will just
        # show a blank screen until cv2.waitKey(0) is pressed, which is fine
        cv2.imshow("grasp on dgr", dgr)

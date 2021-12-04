import numpy as np
import math
from .projections import proj_pixel_to_point, proj_grasp_cam_to_arm
from .grasping_utility import grasp_coords_rel_img, clamp_z, grabbable

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
        DIFF_X=0.0, DIFF_Y=0.0, DIFF_Z=0.24,
        THETA_X=(115*np.pi/180), THETA_Y=(0*np.pi/180), THETA_Z=(0*np.pi/180)):

        self.max_reach = max_reach
        self.gripper_width = gripper_width
        self.DIFF_X = DIFF_X
        self.DIFF_Y = DIFF_Y
        self.DIFF_Z = DIFF_Z
        self.THETA_X = THETA_X
        self.THETA_Y = THETA_Y
        self.THETA_Z = THETA_Z

        
    def locate_object(self, dgr_img, bbox, depth_frame): # should this take a cropped image?
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
        img_pt1, img_pt2 = grasp_coords_rel_img(dgr_img, bbox)

        clamp_x1, clamp_y1, clamp_x2, clamp_y2, z1, z2 = clamp_z(img_pt1, img_pt2, depth_frame)
        
        # TODO: Do I need to pass the clamped points into here?
        gripper_pt1_cam = proj_pixel_to_point(img_pt1[0], img_pt1[1], z1, depth_frame)
        gripper_pt2_cam = proj_pixel_to_point(img_pt2[0], img_pt2[1], z2, depth_frame)
        
        gripper_pt1_arm = proj_grasp_cam_to_arm(gripper_pt1_cam,
            self.DIFF_X, self.DIFF_Y, self.DIFF_Z, self.THETA_X, self.THETA_Y, self.THETA_Z)
        gripper_pt2_arm = proj_grasp_cam_to_arm(gripper_pt2_cam,
            self.DIFF_X, self.DIFF_Y, self.DIFF_Z, self.THETA_X, self.THETA_Y, self.THETA_Z)

        # distance from base of arm to the object is within reach?
        isReachable = math.dist((0,0,0), gripper_pt1_arm) < self.max_reach
        isGrabbable = grabbable(gripper_pt1_arm, gripper_pt2_arm, self.gripper_width)
        return isReachable, isGrabbable, gripper_pt1_arm, gripper_pt2_arm
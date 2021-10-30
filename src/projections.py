import cv2
import numpy as np
import pyrealsense2 as rs
#from builtins import int, len, range, list, float, sorted, max, min
import math
# import matplotlib.pyplot as plt
# from mpl_toolkits import mplot3d #import art3d
from grasp_detection import grab_points

"""Collection of utility functions for grasp detection, such as getting
grasp coordinates relative to a given coordinate system, and plotting grasp
rectangles on the image."""

# Constants based on distance and angle from camera to arm
# (used for cam to arm projection)
DIFF_X = 0.0
DIFF_Y = 0.0
DIFF_Z = 0.24

THETA_X = 115 * np.pi / 180
THETA_Y = 0 * np.pi / 180
THETA_Z = 0 * np.pi / 180


def grasp_coords_rel_img(img, bbox):
    """two grabber points of form (x,y) where x and y are 2D coordinates
    relative to the passed in image. (0,0) represents the top left corner.
    bbox should be a tuple (left, right, top, bottom)
    Calculates these points using the grasp algorithm involving canny edge
    detection"""
    cropped_img = img[bbox[2]: bbox[3], bbox[0]: bbox[1]]
    x1, y1, x2, y2, shortest_dist = grab_points(cropped_img)
    # get loc on full img
    x1 = bbox[0] + x1
    y1 = bbox[2] + y1
    x2 = bbox[0] + x2
    y2 = bbox[2] + y2

    return (x1, y1), (x2, y2)

def clamp_z(img_pt1, img_pt2, depth_frame):
    """Contract img_pt1 and img_pt2 towards center to find the the z coordinates
    just within the object"""
    # keep checking pixels closer and closer to the center until the
    # depth distance between the center and edge are 1 inch or less
    w = depth_frame.get_width()
    h = depth_frame.get_height()
    distances = np.zeros((h, w))
    # compute a distance array
    for x in range(w):
        for y in range(h):
            distances[y, x] = depth_frame.get_distance(x,y)

    # initialize points
    temp_x1, temp_y1 = img_pt1
    temp_x2, temp_y2 = img_pt2
    # find center point
    ctr_x_img = (temp_x1 + temp_x2) / 2
    ctr_y_img = (temp_y1 + temp_y2) / 2
    print("center point: ", ctr_x_img, ",", ctr_y_img, ",", depth_frame.get_distance(int(ctr_x_img), int(ctr_y_img)))
    # find depth disparity between center point and edge points
    depth_disparity_1 =  depth_frame.get_distance(int(temp_x1), int(temp_y1)) - depth_frame.get_distance(int(ctr_x_img), int(ctr_y_img))
    depth_disparity_2 = depth_frame.get_distance(int(temp_x2), int(temp_y2)) - depth_frame.get_distance(int(ctr_x_img), int(ctr_y_img))
    #calculate unit vector
    v1 = np.array([ctr_x_img - temp_x1, ctr_y_img - temp_y1])
    #print("vec1", v1)
    unit_vec1 = v1 / np.sqrt(np.sum(v1**2))
    #print("unit vec1", unit_vec1)
    v2 = np.array([ctr_x_img - temp_x2, ctr_y_img - temp_y2])
    unit_vec2 = v2 / np.sqrt(np.sum(v2**2))
    iters = 0

    # while the difference between depths is more than .02 m, move the
    # edge points on the image closer to the center
    # TODO: will probably be buggy on edge cases
    scale=5
    while (abs(depth_disparity_1) > .02 or abs(depth_disparity_2) > .02) and iters < 10:
        # move in direction of the unit vector
        if abs(depth_disparity_1) > .02:
            #print("moving pt 1 in")
            temp_x1 += unit_vec1[0]*scale
            temp_y1 += unit_vec1[1]*scale
        if abs(depth_disparity_2) > .02:
            #print("moving pt 2 in")
            temp_x2 += unit_vec2[0]*scale
            temp_y2 += unit_vec2[1]*scale
        #print("clamped by 1 iter", temp_x1, temp_y1)
        depth_disparity_1 = depth_frame.get_distance(int(temp_x1), int(temp_y1)) - depth_frame.get_distance(int(ctr_x_img), int(ctr_y_img))
        depth_disparity_2 = depth_frame.get_distance(int(temp_x2), int(temp_y2)) - depth_frame.get_distance(int(ctr_x_img), int(ctr_y_img))
        iters += 1
        
    return temp_x1, temp_y1, temp_x2, temp_y2, depth_frame.get_distance(int(temp_x1), int(temp_y1)), depth_frame.get_distance(int(temp_x2), int(temp_y2))


def proj_grasp_img_to_cam(img_grasp_pt, depth_frame : rs.depth_frame):
    """performs the projection from a 2D pixel in the image to a 3D world point
    in meters using the 2D x, y point, camera intrinsics, and depth at x, y
    pixels.
    """
    grasp_arr = np.asarray(img_grasp_pt) 
    x, y = np.around(grasp_arr).astype(int)
    z = depth_frame.as_depth_frame().get_distance(x, y)
    print("depth", z)

    depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    pt_3D = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], z)

    return pt_3D

def proj_pixel_to_point(x, y, z, depth_frame):
    """Performs pixel to point projections with known x, y pixels and known
    world depth z."""
    depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    pt_3D = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], z)
    return pt_3D

def proj_grasp_cam_to_arm(pt_3D):
    """performs the camera-to-arm projection from (x,y,z) coord relative to the
    camera view to (x,y,z) coord relative to the robot arm."""
    # The default coordinate values ((x,y,z) coordinate in camera)
    x, y, z = pt_3D
    default_coordinate = np.array([[x], [y], [z]])

    # The three translation parameters between two coordinate origins
    # Notice that we put the origin of the camera in the frame of arm coordinate here
    # diff_coordinates = np.array([[DIFF_X], [DIFF_Y], [DIFF_Z]])
    diff_coordinates = np.array([[DIFF_X], [DIFF_Y], [DIFF_Z]])


    # The three rotational angles between two coordinate systems, should be in degree, counter-clockwise
    angles = np.array([THETA_X, THETA_Y, THETA_Z])

    # A scale factor (if needed)
    meu = 1

    # Rotational Matrix
    R = np.dot(np.dot([[np.cos(THETA_Z), np.sin(THETA_Z), 0], [-np.sin(THETA_Z), np.cos(THETA_Z), 0], [0, 0, 1]],
                        [[np.cos(THETA_Y), 0, -np.sin(THETA_Y)], [0, 1, 0], [np.sin(THETA_Y), 0, np.cos(THETA_Y)]]),
                [[1, 0, 0], [0, np.cos(THETA_X), np.sin(THETA_X)], [0, -np.sin(THETA_X), np.cos(THETA_X)]])

    # Transformation formula
    transformed_coordinate = np.add(diff_coordinates, meu * np.dot(R, default_coordinate))

    return (transformed_coordinate)


def calc_pred_rect(img, pt1, pt2, gripper_h):
    """returns array of corner points, polygon of points, and angle"""
    # TODO: Project world measurements of gripper height to pixels for plotting
    # grasp rectangle

    # ================ edge detected grasp point =================
    # gripper_h is taken from labels, just to maintain consistency
    # don't know the specs of the end effector yet so
    x1, y1 = pt1
    x2, y2 = pt2
    diff_x = abs(x1 - x2)
    diff_y = abs(y1 - y2)
    pred_angle = math.atan2(y2-y1, x2-x1)  # in radians

    # TODO: Is my trig correct? T_T

    gripper_w_pred = math.sqrt(diff_x ** 2 + diff_y ** 2) / 2
    dx_w_pred = gripper_w_pred * math.cos(pred_angle)
    dy_w_pred = gripper_w_pred * math.sin(pred_angle)

    dx_h_pred = math.sqrt(gripper_h ** 2 + gripper_w_pred **
                            2) * math.sin(pred_angle)
    dy_h_pred = math.sqrt(gripper_h ** 2 + gripper_w_pred **
                            2) * math.cos(pred_angle)

    # recalculate center by taking average of two points' components
    ctr_x_pred = (x1 + x2) / 2
    ctr_y_pred = (y1 + y2) / 2

    # draw the two gripper dots again (just in case they got covered)
    cv2.circle(img, (int(x1), int(y1)),
                5, (255, 0, 255), -1)
    cv2.circle(img, (int(x2), int(y2)),
                5, (255, 0, 255), -1)

    prect = np.array([[ctr_x_pred - dx_w_pred - dx_h_pred, ctr_y_pred - dy_w_pred + dy_h_pred],
                        [ctr_x_pred - dx_w_pred + dx_h_pred,
                        ctr_y_pred - dy_w_pred - dy_h_pred],
                        [ctr_x_pred + dx_w_pred + dx_h_pred,
                        ctr_y_pred + dy_w_pred - dy_h_pred],
                        [ctr_x_pred + dx_w_pred - dx_h_pred, ctr_y_pred + dy_w_pred + dy_h_pred]], np.int32)

    return prect


def plot_pred_rect(img, prect):
    """input must be an image and nparray of corner points"""
    cv2.polylines(img, [prect], True, (0, 255, 0), 1)
    # cv2.imshow("pred labels", img)
    # cv2.waitKey(0)

def grabbable(pt1, pt2, gripper_max_w):
    """Determines whether two 3D grasping points relative to the arm can be
    grabbed. Specifically, determines whether the object is further than the
    arm can reach, and whether the end effector is wide enough to grab the
    object. """
    # currently just does the end effector part
    #print("input", pt1)
    #print("unwrapped", np.reshape(pt1, (3)))
    pt1 = np.reshape(pt1, 3)
    pt2 = np.reshape(pt2, 3)
    squared = (pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2 + (pt1[2] - pt2[2]) ** 2
    print(squared)
    opening_dist = math.sqrt(squared)
    grabbable = opening_dist <= gripper_max_w
    print("Grasp prediction within end effector opening constraints? ", grabbable)
    print("End Effector Max Opening (m): ", gripper_max_w, "Opening Needed", opening_dist)
    return grabbable

def calc_pts_end_eff(pt1, pt2, gripper_h):

    """Calculates points (3D) for the top and bottom of the end effector
    given grasp points and height of end effector. Assumes Z of both points
    are the same."""

    # ================ edge detected grasp point =================
    x1, y1, z1 = pt1
    x2, y2, z2 = pt2
    diff_x = abs(x1 - x2)
    diff_y = abs(y1 - y2)
    pred_angle = math.atan2(y2-y1, x2-x1)  # in radians

    # TODO: Is my trig correct? T_T

    gripper_w_pred = math.sqrt(diff_x ** 2 + diff_y ** 2) / 2
    dx_w_pred = gripper_w_pred * math.cos(pred_angle)
    dy_w_pred = gripper_w_pred * math.sin(pred_angle)

    dx_h_pred = math.sqrt(gripper_h ** 2 + gripper_w_pred **
                            2) * math.sin(pred_angle)
    dy_h_pred = math.sqrt(gripper_h ** 2 + gripper_w_pred **
                            2) * math.cos(pred_angle)

    # recalculate center by taking average of two points' components
    ctr_x_pred = (x1 + x2) / 2
    ctr_y_pred = (y1 + y2) / 2

    ee1_pt1 = (ctr_x_pred - dx_w_pred - dx_h_pred, ctr_y_pred - dy_w_pred + dy_h_pred, z1)
    ee1_pt2 = (ctr_x_pred - dx_w_pred + dx_h_pred, ctr_y_pred - dy_w_pred - dy_h_pred, z1)
    ee2_pt1 = (ctr_x_pred + dx_w_pred + dx_h_pred, ctr_y_pred + dy_w_pred - dy_h_pred, z2)
    ee2_pt2 = (ctr_x_pred + dx_w_pred - dx_h_pred, ctr_y_pred + dy_w_pred + dy_h_pred, z2)

    return ee1_pt1, ee1_pt2, ee2_pt1, ee2_pt2

def plot_grasp_3D(pt1, pt2, gripper_h):
    """Plots the camera position in at (0,0), the arm position relative to
    the camera, and the two grasp points"""
    pass

    


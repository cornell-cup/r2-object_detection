import numpy as np
import pyrealsense2 as rs

def get_distance(img_grasp_pt, depth_frame : rs.depth_frame):
    """ get the z distance of a pixel at location x, y
    """
    grasp_arr = np.asarray(img_grasp_pt) 
    x, y = np.around(grasp_arr).astype(int)
    z = depth_frame.as_depth_frame().get_distance(x, y)
    print("depth", z)
    return z

def proj_pixel_to_point(x, y, z, depth_frame):
    """Performs pixel to point projections with known x, y pixels and known
    depth z. Note that depth is not the same as the actual z coordinate in
    meters
    Returns the x, y, z coordinates in meters
    """
    depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    pt_3D = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], z)
    return pt_3D

def proj_grasp_cam_to_arm(pt_3D, DIFF_X, DIFF_Y, DIFF_Z, THETA_X, THETA_Y, THETA_Z):
    """performs the camera-to-arm projection from (x,y,z) coord relative to the
    camera view to (x,y,z) coord relative to the robot arm."""
    # The default coordinate values ((x,y,z) coordinate in camera)
    x, y, z = pt_3D
    default_coordinate = np.array([[x], [y], [z]])

    # The three translation parameters between two coordinate origins
    # Notice that we put the origin of the camera in the frame of arm coordinate here
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

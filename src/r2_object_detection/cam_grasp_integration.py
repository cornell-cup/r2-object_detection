from grasp_viz import calc_pred_rect, plot_pred_rect
from final_iris import grab_points
import time
# from io import StringIO
# from collections import defaultdict
# from distutils.version import StrictVersion
import math
import cv2
# import zipfile
# import tensorrt as tf
# import tarfile
import pyrealsense2 as rs
import numpy as np
import os
# import six.moves.urllib as urllib
import sys
print(sys.version_info)

# from PIL import Image


def get_DGR(aligned_frames):
    """forms an rgd image from the camera's color and depth frames."""
    # get rgb frame
    # get d frame, which returns distance
    # use numpy to merge the frames
    pass


def get_grasp_on_image(aligned_frames, bbox_coords):
    """two grabber points of form (x,y,z) where x and y are 2D coordinates
    relative to the camera image. On images, (0,0) represents the top left corner.
    z is the depth distance in [units].
    Calculates these points using the grasp algorithm involving canny edge
    detection"""
    # Get aligned frames
    # aligned_depth_frame is a 640x480 depth image
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        pass

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    print(depth_image)

    color = cv2.resize(color_image, (1920, 1080))
    depth = cv2.resize(depth_image, (1920, 1080))

    # make dgr image
    b, g, r = cv2.split(color)
    colorizer = rs.colorizer(5)
    depth_colorized = np.asanyarray(
        colorizer.colorize(depth_frame).get_data())
    depth_colorized = cv2.resize(b2w_depth, (1920, 1080))

    dgr = cv2.merge((b2w_depth, g, r))
    # grab_points should output some images
    x1, y1, x2, y2, shortest_dist = grab_points(
        bbox_coords[0], bbox_coords[1], bbox_coords[2], bbox_coords[3], dgr)
    # add bbox_x + x1, bbox_y + y1 ... to get loc on full rgd img
    x1 = bbox_coords[0] + x1
    y1 = bbox_coords[1] + y1
    x2 = bbox_coords[0] + x2
    y2 = bbox_coords[1] + y2

    # TODO: translate these coords to depth frame size to get depth
    # need to scale x and y down bc depth frame is smaller scale than dgr
    # distance in meters at a given pixel
    z1 = depth_frame.get_distance(x1*640/1920, y1*480/1080)
    z2 = depth_frame.get_distance(x2*640/1920, y2*480/1080)
    # get confirmation that this output is fine
    return (x1*640/1920, y1*480/1080, z1), (x2*640/1920, y2*480/1080, z2)


def get_grasp_coords_relative_to_arm():
    """performs the camera-to-arm projection from (x,y,z) coords relative to the
    camera view to (x,y,z) coords relative to the robot arm."""
    pass

def main():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 1  # 1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        for i in range(5):
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            # aligned_depth_frame is a 640x480 depth image
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            print(depth_image)

            # Does aligning mean they are the same size?

            test = cv2.resize(color_image, (1920, 1080))
            test_depth = cv2.resize(depth_image, (1920, 1080))
            # cv2.imwrite('color_image.jpg', test)
            # cv2.imwrite('depth_image.jpg', test_depth)

            colorizer = rs.colorizer()
            colorized_depth = np.asanyarray(
                colorizer.colorize(aligned_depth_frame).get_data())
            colorized_depth = cv2.resize(colorized_depth, (1920, 1080))
            # cv2.imwrite('colorized_depth_image.jpg', colorized_depth)

            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            # depth image is 1 channel, color is 3 channels
            depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
            bg_removed = np.where((depth_image_3d > clipping_distance) | (
                depth_image_3d <= 0), grey_color, color_image)
            # Render images
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # grasp prediction code
            b, g, r = cv2.split(test)

            print("colorizer mode: ", 2)
            test_colorizer = rs.colorizer(2)
            test_depth_colorized = np.asanyarray(
                test_colorizer.colorize(aligned_depth_frame).get_data())
            test_depth_colorized = cv2.resize(
                test_depth_colorized, (1920, 1080))
            print(test_depth_colorized.shape)
            #all frames in w2b look the same
            #cv2.imshow("r", (test_depth_colorized[:, :, 0]))
            #cv2.imshow("g", (test_depth_colorized[:, :, 1]))
            #cv2.imshow("b", (test_depth_colorized[:, :, 2]))
            dgr = cv2.merge((test_depth_colorized[:, :, 0], g, r))
            gripper_h = 200
            width, height = dgr.shape[1], dgr.shape[0]
            rect_points = calc_pred_rect(
                #grab points will show the canny and dgr with the grasps
                dgr, grab_points(0, 0, width, height, dgr), gripper_h)
            plot_pred_rect(dgr, rect_points)
            # colorized depth image
            cv2.imshow("original", test)
            cv2.imshow("white to black depth", test_depth_colorized)
            key = cv2.waitKey(0)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

            # for i in range(7):
            #     print("colorizer mode: ", i)
            #     test_colorizer = rs.colorizer(i)
            #     test_depth_colorized = np.asanyarray(
            #         test_colorizer.colorize(aligned_depth_frame).get_data())
            #     test_depth_colorized = cv2.resize(
            #         test_depth_colorized, (1920, 1080))
            #     print(test_depth_colorized.shape)
            #     dgr = cv2.merge((test_depth_colorized[:, :, 0], g, r))
            #     gripper_h = 200
            #     width, height = dgr.shape[1], dgr.shape[0]
            #     rect_points = calc_pred_rect(
            #         dgr, grab_points(0, 0, width, height, dgr), gripper_h)
            #     plot_pred_rect(dgr, rect_points)
            #     cv2.imshow("image with grasp prediction", test_depth_colorized)
            #     key = cv2.waitKey(0)
            #     if key & 0xFF == ord('q') or key == 27:
            #         cv2.destroyAllWindows()
            #         break

            # get_DGR()
            # get_grasp_on_image()

            # print((coordinates[1]*bg_removed.shape[1], coordinates[0]*bg_removed.shape[0]))
            # cv2.rectangle(color_image, (int(coordinates[1]*bg_removed.shape[1]), int(coordinates[0]*bg_removed.shape[0])), (int(coordinates[3]*bg_removed.shape[1]), int(coordinates[2]*bg_removed.shape[0])), 100, 5)
            # print("1")
            # images = np.hstack((depth_colormap, color_image))
            # print("2")
            # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
            # print("3")
            # cv2.imshow('Align Example', color_image)
            # print("4")
            # test2 = cv2.resize(depth_colormap, (1920, 1080))
            # print("5")
            # if(math.sqrt((inx+ 0.06985)**2 + ( iny - 0.10795)**2 + (inz-0.05715)**2) < .7):
            #     kinematics(inx+ 0.06985, iny - 0.10795, inz-0.05715);
            # cv2.imshow('color_image.jpg', test)
            # cv2.imshow('depth_image.jpg', colorized_depth)
            # cv2.imshow("image with grasp prediction", grasp_image)
            # key = cv2.waitKey(0)

            # Press esc or 'q' to close the image window
            # if key & 0xFF == ord('q') or key == 27:
            #     cv2.destroyAllWindows()
            #     break
    finally:
        pipeline.stop()

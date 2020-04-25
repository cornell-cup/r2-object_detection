#!/usr/bin/python
import pyrealsense2 as rs
import numpy as np
import sys
import cv2
import time

import jetson.inference
import jetson.utils

from r2_edge_detection.detection import grab_points

threshold = 0.5
net = jetson.inference.detectNet("ssd-mobilenet-v2", sys.argv, threshold)


# Create a pipeline
pipeline = rs.pipeline()


#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth scale is: ", depth_scale)
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale



#camera = jetson.utils.gstCamera(640, 480, "/dev/video2")
try:
    while True:
        #try:
        #test-block
        t1 = time.time()
        #end test-block


#        camera = jetson.utils.gstCamera(640, 480, "/dev/video2")



        
#        profile = pipeline.start(config)
        frames = pipeline.wait_for_frames()
        color_frames = frames.get_color_frame()
#        pipeline.stop()
        color_image = np.asanyarray(color_frames.get_data()).astype(np.uint8)
        cv2.imshow("image", color_image)
        cv2.waitKey(0)
        detections = []
        while not detections:
            print('Nothing detected')
            cudaImg = jetson.utils.cudaFromNumpy(color_image)
            cudaWidth, cudaHeight, _ = color_image.shape
#            cudaImg, cudaWidth, cudaHeight = camera.CaptureRGBA(zeroCopy=1)
            detections = net.Detect(cudaImg, cudaWidth, cudaHeight, "box,labels,conf")
        print('Object(s) detected.')
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
#        depth_sensor = profile.get_device().first_depth_sensor()
#        depth_scale = depth_sensor.get_depth_scale()
#        print("Depth Scale is: " , depth_scale)
            # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
#        clipping_distance_in_meters = 1 #1 meter
#        clipping_distance = clipping_distance_in_meters / depth_scale
            # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
#        align_to = rs.stream.color
#        align = rs.align(align_to)

        # frames.get_depth_frame() is a 640x360 depth image
        # Align the depth frame to color frame
#        aligned_frames = align.process(frames)
        # Get aligned frames
#        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
#        color_frame = aligned_frames.get_color_frame()
        # Validate that both frames are valid
        #if not aligned_depth_frame or not color_frame:
        #    continue

#        depth_image = np.asanyarray(aligned_depth_frame.get_data())
#        color_image = np.asanyarray(color_frame.get_data())
#       print(cudaImg)
#        color_image = jetson.utils.cudaToNumpy(cudaImg, cudaWidth, cudaHeight, 3).astype(np.uint8)
        print(color_image.shape)
        print('Detected these objects:')
        print(detections)
        print(detections[0])
#       cv2.imwrite('depthimg.png',depth_image)
        cv2.imwrite('colorimg.png',color_image)
        x,y,width,height = (int(detections[0].Left), int(detections[0].Top), int(detections[0].Width), int(detections[0].Height))
        grab_x1, grab_y1, grab_x2, grab_y2, grab_distance = grab_points(x, y, width, height, color_image)
        cv2.circle(color_image, (int(grab_x1), int(grab_y1)), 5, (255,0,255), -1)
        cv2.circle(color_image, (int(grab_x2), int(grab_y2)), 5, (255,0,255), -1)
        cv2.imwrite('grabimg.png',color_image)
        cv2.imshow('frame',color_image)
        if cv2.waitKey(400)==27:
            break
finally:
    #camera.Close()
    #finally:
    pipeline.stop()

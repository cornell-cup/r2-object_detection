import cv2
import numpy as np
import time
import os
import datetime

from camera import Camera
import jetson.inference
# for displaying
import jetson.utils

net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
display = jetson.utils.videoOutput("my_video.mp4") # 'my_video.mp4' for file

WIDTH = 640
HEIGHT = 480

def main():
    detections = []
    start_time = time.time()
    with Camera(WIDTH, HEIGHT) as cam:
        for i in range(5): 
            try:
                #get frames
                color_frame, depth_frame = cam.get_frames()
                # Validate that both frames are not None and depth is not all 0
                # TODO: figure out why depth is weird sometimes
                if not depth_frame.is_frame() or not color_frame.is_frame():
                    print("frames not captured")
                    continue
            except RuntimeError:
                print("Couldn't get frames in time")
                continue

            color_img, depth_img, dgr = cam.get_imgs_from_frames(color_frame, depth_frame)

            color_img_cuda = jetson.utils.cudaFromNumpy(color_img)
            detections = net.Detect(color_img_cuda)
            if not detections:
                print('Nothing detected')
                continue
            # For now selecting first detection, later find object asked for
            detection = detections[0]

            top, bottom, left, right = detection.Top, detection.Bottom, detection.Left,detection.Right
            top, bottom, left, right = round(top), round(bottom), round(left), round(right)

            depth_img = np.asanyarray(depth_frame.get_data())
            color_img = np.asanyarray(color_frame.get_data())

            print("original depth: ", depth_img)

            depth_img = (depth_img/np.float(np.max(depth_img)))*255
            depth_img = depth_img.astype(np.uint8)

            print("normalized depth: ", depth_img)

            color_img_cropped, depth_img_cropped, dgr_cropped = color_img[top:bottom, left:right], depth_img[top:bottom, left:right], dgr[top:bottom, left:right]

            cv2.imshow("Color image", color_img)
            cv2.imshow("Depth image", depth_img)
            cv2.imshow("Cropped color image", color_img_cropped)
            cv2.imshow("Cropped depth image", depth_img_cropped)
            cv2.waitKey()

            parent_dir = "kmeans_test_imgs"
            current_time = datetime.datetime.now()
            directory = str(current_time.year) + "-" + str(current_time.month) + "-" + str(current_time.day) + "-" + str(current_time.hour) + ":" + str(current_time.minute) + ":" + str(current_time.second)
            path = os.path.join(parent_dir, directory)
            os.mkdir(path)

            cv2.imwrite((path + '/Original.jpg'), color_img)
            cv2.imwrite((path + '/Cropped Original.jpg'), color_img_cropped)
            cv2.imwrite((path + '/Depth Frame.jpg'), depth_img)
            cv2.imwrite((path + '/Cropped Depth Frame.jpg'), depth_img_cropped)

if __name__ == "__main__":
    main()
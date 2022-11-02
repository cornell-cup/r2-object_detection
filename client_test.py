from networking.Client import *
import numpy as np
import cv2

if __name__ == "__main__":
    robot = Client()
    cpath = 'c1c0_object_detection/kmeans_test_imgs/04-12-13:10:36/Orignal.jpg'
    dpath = 'c1c0_object_detection/kmeans_test_imgs/04-12-13:10:36/Depth Frame.jpg'

    color_img = cv2.imread(cpath, cv2.IMREAD_GRAYSCALE)
    depth_img = cv2.imread(dpath, cv2.IMREAD_GRAYSCALE)

    try:
        # compute grasp coords and bounding boxes
        grasp_coords = [[[-.1], [-.2], [.5]], [[0], [-.2], [.5]]]

        # send object containing grasp coords to the computer
        data_packet = grasp_coords
        robot.send_data(color_img, depth_img, [.5], [0], [-.2], [.5], 7)
        response = robot.listen()
        print("Server Response: ", response)
    except:
        robot.socket.close()

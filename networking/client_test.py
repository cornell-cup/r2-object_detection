from networking.Client import *
import numpy as np

if __name__ == "__main__":
    # compute grasp coords and bounding boxes
    grasp_coords = np.array([-.1, -.2, .5])

    # send object containing grasp coords to the computer
    robot = Client()
    data_packet = ObjectDetectionObject(["rrt"], [grasp_coords])
    robot.send_data(data_packet)
    print(robot.listen())

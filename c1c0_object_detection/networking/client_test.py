from .Client import Client
import numpy as np

if __name__ == "__main__":
    robot = Client()
    for i in range(5): 
        # compute grasp coords and bounding boxes
        grasp_coords = [[[-.1], [-.2], [.5]], [[0], [-.2], [.5]]]

        # send object containing grasp coords to the computer
        data_packet = grasp_coords
        robot.send_data(data_packet)
        response = robot.listen()
        print(response)

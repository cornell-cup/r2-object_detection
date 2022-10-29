from networking.Client import *
import numpy as np

if __name__ == "__main__":
    robot = Client()
    try:
        for i in range(5): 
            # compute grasp coords and bounding boxes
            grasp_coords = [[[-.1], [-.2], [.5]], [[0], [-.2], [.5]]]

            # send object containing grasp coords to the computer
            data_packet = grasp_coords
            robot.send_data([-.1], [-.2], [.5], [0], [-.2], [.5], 7)
            response = robot.listen()
            print(response)
    except:
        robot.socket.close()

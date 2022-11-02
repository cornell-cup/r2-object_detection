import pickle
from networking.Network import *
from networking.ObjectDetection import *
from networking.SensorState import *
import sys
import json

def jprint(obj):
    # create a formatted string of the Python JSON object
    text = json.dumps(obj, sort_keys=True, indent=4)
    print(text)

class Client(Network):
    def __init__(self):
        super().__init__()
        self.socket.connect(self.server)
        #self.socket.settimeout(4)  # interferes with stopping
        self.receive_ID= 0


    def send_data(self, color_img, depth_frame, dgr, startpos, bbox, c1, c2):
        """ sends json-like nested data containing sensor, accelerometer, etc.
        """
        data = [self.receive_ID, color_img,depth_frame,dgr, startpos, bbox, c1, c2]
        pickled_images = pickle.dumps(data)
        print(self.socket)
        self.socket.sendall(pickled_images)

    def listen(self):
        print("listening to ", self.server)
        # according to pickle docs you shouldn't unpickle from unknown sources, 
        # so we have some validation here
        #while x[1] != self.server:
            # print(x[1], self.server)
        x = self.socket.recv(4096)
        #print(x)
            # print(x[1], self.server)
        y = pickle.loads(x)
        self.receive_ID, content= y[0], y[1]
        # print(y, content)
        # print("PREVIOUS RETURN", str(content))
        return content

# # test to make sure that SensorState object is <= 4096 bytes
# if __name__ == "__main__":
#     robot = Client()
#     data_packet = SensorState()
#     robot.send_data(data_packet)
#     print(robot.listen())

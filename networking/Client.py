import pickle
from networking.Network import *
from networking.ObjectDetection import *
from networking.SensorState import *
import sys
import json
import struct

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


    def send_data(self, data):
        """ sends json-like nested data containing sensor, accelerometer, etc.
        """
        pickled_images = pickle.dumps(data)
        size = len(pickled_images)
        self.socket.sendall(struct.pack(">L", size) + pickled_images)

    def listen(self):
        print("listening to ", self.server)
        try:
            conn = self.socket
            payload_size = struct.calcsize(">L")
            data = b''
            while len(data) < payload_size:
                print("Recv: {}".format(len(data)))
                data += (conn.recv(4096))
        
            print("Done Recv: {}".format(len(data)))
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack(">L", packed_msg_size)[0]
            print("msg_size: {}".format(msg_size))
            while len(data) < msg_size:
                data += conn.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
            return frame
        except:
            print("Couldn't connect to socket in time")

# # test to make sure that SensorState object is <= 4096 bytes
# if __name__ == "__main__":
#     robot = Client()
#     data_packet = SensorState()
#     robot.send_data(data_packet)
#     print(robot.listen())

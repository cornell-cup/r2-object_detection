# import pickle
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
        self.socket.bind((self.get_ip(), 4004))
        #self.socket.settimeout(4)  # interferes with stopping
        self.receive_ID= 0


    def send_data(self, data):
        """ sends json-like nested data containing sensor, accelerometer, etc.
        """
        x= json.dumps({'id': self.receive_ID, 'data': data}).encode('utf-8')
        print("size: ", sys.getsizeof(x))
        print(data)
        print(self.socket)
        self.socket.sendto(x, self.server)

    def listen(self):
        x = ["", ("0.0.0.0", 9999)]
        print("listening")
        # according to pickle docs you shouldn't unpickle from unknown sources, so we have some validation here
        while x[1] != self.server:
            # print(x[1], self.server)
            x = self.socket.recvfrom(4096)
            # print(x[1], self.server)
        y= json.loads(x[0].decode('utf-8'))

        self.receive_ID= y['id']
        print(y, type(y['content']))
        return "content: " + str(y['content'])

# kinematics_test to make sure that SensorState object is <= 4096 bytes
if __name__ == "__main__":
    robot = Client()
    data_packet = SensorState()
    robot.send_data(data_packet)
    print(robot.listen())

import pickle
import json
import time
import struct
from networking.Network import *


class Server(Network):
    def __init__(self):
        super().__init__()
        print((self.get_ip(), self.port))
        # bind the socket to an ip and port
        self.socket.bind((self.get_ip(), self.port))
        print("Server Started")
        self.client = None
        self.last_sent= None
        self.send_ID = 0

    def receive_data(self):
        try:
            self.socket.listen(10)
            conn, _ = self.socket.accept()
            self.client = conn
            conn.settimeout(1)
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

    ##  precondition: must have called receive_data successfully
    def send_update(self, update):
        self.send_ID += 1
        self.last_sent= update
        self.client.sendall(pickle.dumps([self.send_ID,update]))


# test with Client.py main method
if __name__ == "__main__":
    computer = Server()
    while True:
        print("receiving data")
        x = computer.receive_data()
        if x != "no data within listening time":
            print(x)
            computer.send_update(123321)  # placeholder
            break

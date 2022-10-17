import pickle
import json
from networking.Network import *


class Server(Network):
    def __init__(self):
        super().__init__()
        print((self.get_ip(), self.port))
        # bind the socket to an ip and port
        self.socket.bind((self.get_ip(), self.port))
        print("Server Started")
        self.client = ("", 0)
        self.last_sent= None
        self.send_ID = 0

    def receive_data(self):
        try:
            self.socket.listen(10)
            conn, addr = self.socket.accept()
            x = conn.recvfrom(4096)
            print("client", x[1])
            self.client = x[1]
            self.socket.settimeout(1)  # interferes with stopping on further calls

            y = pickle.loads(x[0].decode('utf-8'))

            if y[0] != self.send_ID:
                self.send_update(self.last_sent)  # re-attempt last send operation
                self.socket.settimeout(1)  # interferes with stopping on further calls
                return self.receive_data()

            return y[1:]

        except socket.timeout:
            self.send_update(self.last_sent) # re-attempt last send operation
            self.socket.settimeout(1)  # interferes with stopping on further calls
            return self.receive_data()

    ##  precondition: must have called receive_data successfully
    def send_update(self, update):
        self.send_ID += 1
        self.last_sent= update
        self.socket.sendto(pickle.dumps([self.send_ID,update]).encode('utf-8'), self.client)


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

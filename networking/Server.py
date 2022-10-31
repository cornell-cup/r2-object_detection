import pickle
import json
import time
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
            conn, addr = self.socket.accept()
            start_time = time.time()
            conn.settimeout(1)

            data = []
            try: 
                while True:
                    packet = conn.recv(4096)
                    print("packet", packet)
                    if not packet: break
                    data.append(packet)
            except:
                self.client = conn
                #self.socket.settimeout(10)  # interferes with stopping on further calls
                y = pickle.loads(b"".join(data))

                if y[0] != self.send_ID:
                    self.send_update(self.last_sent)  # re-attempt last send operation
                    conn.settimeout(1)  # interferes with stopping on further calls
                    return self.receive_data()
                #self.socket.close()
                return y[1:]

        except:
            self.send_update(self.last_sent)
            return self.receive_data()

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

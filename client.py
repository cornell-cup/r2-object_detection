"""
Scheduling API for C1C0

March 19, 2021

Purposes of the API:

"""
# import multserver
import socket

class Client(object):

    def __init__(self, process_type):
        """
        Parameter: process_type
        Invariant: process_type is a string in ["path-planning", "object-detection", "locomotion"]
        """
        self.process_type = process_type
        self.handshakeComplete = False
        self.ClientSocket = socket.socket()


    def handshake(self):
        host = '127.0.0.1'
        port = 1233

        print('Waiting for connection')
        try:
            self.ClientSocket.connect((host, port))
            print("connecting")
        except socket.error as e:
            print(str(e))

        Response = self.ClientSocket.recv(32)
        while (not self.handshakeComplete):
            #TODO: handshake timeout mechanism
            self.ClientSocket.send(str.encode("I am "+ self.process_type))
            ResponseSocket = self.ClientSocket.recv(32)
            Response = ResponseSocket.decode('utf-8')
            if (Response == self.process_type + " is recognized"):
                print(Response)
                self.handshakeComplete = True

    def communicate(self, request):
        if self.handshakeComplete:
            self.ClientSocket.send(str.encode(request))
            ResponseSocket = self.ClientSocket.recv(64)
            Response = ResponseSocket.decode('utf-8')
            print(Response)
            return Response

    def close(self):
        self.communicate("kill")
        self.ClientSocket.close()
        print("closed connection")
    
    


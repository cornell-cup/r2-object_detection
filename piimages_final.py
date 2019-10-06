import socket
import numpy as np
from PIL import Image
import picamera
import time
import sys
ip = 192.168.4.102
portnum = 5000
option = 1
def main():
    try:
        input("Please press Enter to take a picture")
    except:
        pass
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((ip,portnum))
    with picamera.PiCamera() as camera:
        camera.capture("image.jpg")
    frame = Image.open("image.jpg")
    frame = np.asarray(frame)
    frameBytes = frame.tobytes()
    byteLength = len(frameBytes)
    shape0 = frame.shape[0]
    shape1 = frame.shape[1]
    print(frame.shape)
    s.send(str(byteLength).encode())
    s.close()
    connected = False       
    while connected == False:
        try:
            s1=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            s1.connect((ip,portnum))
            print("connected")
            connected = True
        except:
            print("cannot reconnect")
    s1.send(frameBytes)
    l=0
    s = 1920*1080*3       

    while l<s:
        if l==0:
            buf=s1.recv(9192)
            l+=len(buf)
        else:
            rec=s1.recv(9192)
            l+=len(rec)
            buf+=rec
    print("loop done")
    arr=np.frombuffer(buf,dtype="uint8")
    print("got array")
    arr=arr.reshape((1080,1920, 3))
    image = Image.fromarray(arr)
    image.show()
    image.save("backimage.png")
    print("showing image")
            
    s1.close()
    time.sleep(.001)



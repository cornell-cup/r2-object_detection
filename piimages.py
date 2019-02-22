import socket
import numpy as np
from PIL import Image
import picamera
import time
import cv2
ip = sys.argv[1]
portnum = sys.argv[2]
def main():	
	s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	s.connect(ip,portnum)
	with picamera.PiCamera as camera():
		camera.capture("image.png")
	frame = cv2.imread("image.png")
	frameBytes = frame.tobytes()
	byteLength = len(bytes)
	shape0 = frame.shape[0]
	shape1 = frame.shape[1]
	s.send(str(byteLength).encode())
	s.send(str(shape0).encode())
	s.send(str(shape1).encode())
	s.close()
	try:	
		s1=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		s1.connect(ip,portnum) 
	except:
		print("cannot reconnect")
	print("connected")
	s1.send(frameBytes)
	s1.close()
	time.sleep(.001)

while true
 main();

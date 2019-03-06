import socket
import numpy as np
from PIL import Image
import picamera
import time
import sys
ip = sys.argv[1]
portnum = int(sys.argv[2])
def main():
	try:
		input()
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
	s1.close()
	time.sleep(.001)

while True :
 main();

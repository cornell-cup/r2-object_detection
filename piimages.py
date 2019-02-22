import socket
import numpy as np
from PIL import Image
import picamera
import time
ip = sys.argv[1]
portnum = sys.argv[2]
def main():	
	s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	s.connect(ip,portnum)
	with picamera.PiCamera as camera():
		camera.capture("image.png")
	img = open("image.png", 'rb')
	bytes = img.read()
	size = len(bytes)
	s.send(size.encode())
	s.sent(bytes.encode())
	time.sleep(.001)

while true
 main();

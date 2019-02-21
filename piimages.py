import socket
import numpy as np
from PIL import Image
import picamera
import time


def main():	s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)						s.connect(('192.168.4.27',portnum))
	with picamera.PiCamera as camera():
		camera.capture("image.png")
	file = open("image.png", 'rb')
	bytes = file.read()
	size = len(bytes)
	s.send(size.encode())
	s.sent(bytes.encode())
	time.sleep(.001)

while true
 main();

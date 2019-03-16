import socket
import numpy as np
from PIL import Image
import picamera
import time
import sys
# ip = sys.argv[1]
# portnum = int(sys.argv[2])
def main():
	# try:
    #     s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    #     s.connect((ip,portnum))
    #     print('connected')
    # except:
    #     print('cannot connect')
    
    
	input("Please press Enter to learn background")
	bg_samples = []
	for i in range(30):
		with picamera.PiCamera() as camera:
			camera.capture("bg.jpg")
		frame = Image.open("bg.jpg")
		background = np.asarray(frame)
		bg_samples.append(background)
		bg_samples = np.array(bg_samples)
	background = np.mean(bg_samples, axis=0)
	
    
    input("Please press Enter to take a hand Image")
    with picamera.PiCamera() as camera:
		camera.capture("new.jpg")
	print("hand image taken")
	frame = Image.open("new.jpg")
    hand = np.asarray(frame)

    #formulate a new picture:
    diff = hand - background
    distance = np.sqrt(np.sum(np.square(diff), axis=2))
    fg_mask = distance > 32
    im = Image.fromarray(numpy.uint8(fg_mask))
    im.save("im.bmp")

	# frameBytes = frame.tobytes()
	# byteLength = len(frameBytes)
	# shape0 = frame.shape[0]
	# shape1 = frame.shape[1]
	# print(frame.shape)
	# s.send(str(byteLength).encode())
	# s.close()
	# connected = False       
	# while connected == False:
	# 	try:
	# 		s1=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	# 		s1.connect((ip,portnum))
	# 		print("connected")
	# 		connected = True
	# 	except:
	# 		print("cannot reconnect")
	# s1.send(frameBytes)
	# s1.close()
	# time.sleep(.001)

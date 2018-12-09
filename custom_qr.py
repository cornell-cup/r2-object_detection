# import the necessary packages
from imutils.video import VideoStream
from pyzbar import pyzbar
import argparse
from PIL import Image
import datetime
import imutils
import time
import cv2
import socket 
import numpy as np

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="barcodes.csv",
	help="path to output CSV file containing barcodes")
args = vars(ap.parse_args())

# initialize the video stream and allow the camera sensor to warm up
portnum=input("enter the port number ")
portnum=int(portnum)
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
# vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)

# open the output CSV file for writing and initialize the set of
# barcodes found thus far
csv = open(args["output"], "w")
found = set()



# loop over the frames from the video stream
while True:
	# grab the frame from the threaded video stream and resize it to
	# have a maximum width of 400 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=400)

	# find the barcodes in the frame and decode each of the barcodes
	barcodes = pyzbar.decode(frame)
	i=0
    	# loop over the detected barcodes
	for barcode in barcodes:
		# extract the bounding box location of the barcode and draw
		# the bounding box surrounding the barcode on the image
		(x, y, w, h) = barcode.rect
		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

		# the barcode data is a bytes object so if we want to draw it
		# on our output image we need to convert it to a string first
		barcodeData = barcode.data.decode("utf-8")
		barcodeType = barcode.type

		# draw the barcode data and barcode type on the image
		text = "{} ({})".format(barcodeData, barcodeType)
		cv2.putText(frame, text, (x, y - 10),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

		if key == ord("s"):
			try:
				
				
				frame=vs.read()
				f =open ('test.txt','w')
				f.write(text)
				f.close()

				img=Image.fromarray(frame,'RGB')
				
				i=i+1
				img.save("out{}.jpg".format(i),"JPEG",quality=200,optimize=True,progressive=True)

		

				s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
				s.connect(('192.168.4.27',portnum))
				

				frameBytes= frame.tobytes()
		
				# # # print(type(frame))
				# # # print(len(frameBytes))	
				# # print(type(len(frameBytes)))
				byteLen=len(frameBytes)
				# # print(byteLen)
				# print(str(byteLen))
				shape0=(frame.shape[0])
				shape1=(frame.shape[1])
				# print(shape0)

				# # print(type(frameBytes.shape[0]))
				s.send(str(byteLen).encode())

				s.send (str(shape0).encode())
				s.send (str(shape1).encode())
				s.close()
			
				try:
					s1=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
					s1.connect(('192.168.4.27',portnum))
				except:
					print("cannot reconnect")
				print("connected!")
				s1.send(frameBytes)

				s1.close()
				s2=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
				s2.connect(('192.168.4.27',portnum))
				print("connected!")
				text=text.rstrip(' (QRCODE)')
				print(text)
				s2.send(text.encode())
				s2.close()
				sys.exit(0)

				# s.send (str(frame.shape[0]).encode())
				# s.send (str(frame.shape[1]).encode())
				
				# path= raw_input('Documents/QR-Code/out.jpg')
				# s.send(('jpg_'+path).encode('utf-8'))
				# reply= s.recv(buf)
				# file = open ('Documents/QR-Code/out.jpg','w')
				# file.write(reply)
				# file.close()
				# print("finished")



			except:
				continue 

		if key == ord("q"):
			break


		# if the barcode text is currently not in our CSV file, write
		# the timestamp + barcode to disk and update the set
		if barcodeData not in found:
			csv.write("{},{}\n".format(datetime.datetime.now(),
				barcodeData))
			csv.flush()
			found.add(barcodeData)

	# show the output frame
	cv2.imshow("Barcode Scanner", frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the `q` key was pressed, break from the loop


# close the output CSV file do a bit of cleanup
print("[INFO] cleaning up...")
csv.close()
cv2.destroyAllWindows()
vs.stop()
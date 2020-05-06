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
        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        s.connect((ip,portnum))
        print('connected')
    except:
        print('cannot connect')
    
    
    input("Please press Enter to learn background")
    bg_samples = []
    camera = picamera.PiCamera()
    camera.framerate = 30
    time.sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    camera.resolution = (160, 120)
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    for i in range(5):
        camera.capture("bg.jpg")
        print(i)
        frame = Image.open("bg.jpg")
        background = np.asarray(frame)
        bg_samples.append(background)
    bg_samples = np.array(bg_samples)
    background = np.mean(bg_samples, axis=0)
    input("Please press Enter to take a hand Image")
    camera.capture("new.jpg")
    print("hand image taken")
    frame = Image.open("new.jpg")
    hand = np.asarray(frame)

    #formulate a new picture:
    diff = hand - background
    diff = np.clip(diff,0, a_max = None)
    #print(diff)
    im = Image.fromarray(np.uint8(diff))
    im.save("diff.bmp")
    distance = np.sqrt(np.sum(np.square(diff), axis=2))
    fg_mask = distance > 32
    fg_mask2 = fg_mask * 255
    im = Image.fromarray(np.uint8(fg_mask2))
    im.save("im.bmp")
    
    frame = np.uint8(fg_mask)
    print(frame.shape)
    frame2 = frame
    #frame2=np.resize(frame,(120,160))
    frame2 = Image.fromarray(np.uint8(frame2))
    frame2.save("frame.bmp")
    frameBytes = frame.tobytes()
    byteLength = len(frameBytes)
    shape0 = frame.shape[0]
    shape1 = frame.shape[1]
    #print(frame.shape)
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
main();
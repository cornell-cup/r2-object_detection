
from networking.Client import *
import numpy as np

import serial
import sys
import codecs
import sys
import time 

sys.path.append('home/cornellcup-cs-jetson/Desktop/r2-object_detection/src/arm/')
import R2Protocol as r2
# Dedicate a /dev port for specifically the precise arm




'''
Arduino to Jetson Communication with R2 Protocol
'''

ser = None
 
def init_serial(port,baud):
    '''
    Initializes the serial port, usually set baud to 9600
    '''
    global ser, startseq, endseq
    ser =serial.Serial(port,baud)
   
    
def close_serial():
    '''
    Closes the serial port.
    '''
    global ser
    ser.close()
    
def read_encoder_values():
    '''
    Returns 6 encoder values (in decimal) as an array.
    If the value is 1452, then the encoder is not powered or there is 
    a wiring issue. 
    '''
    # initialize the array 
    encoderAngle = [0,0,0,0,0,0]
    try:
            while True:
                # length of message plus 16
                        good_data = False
                        while (not good_data):
                                ser_msg = ser.read(28)
                                msgtype, msg, status = r2p.decode(ser_msg)
                                # print(msg.hex())
                                print(status)
                                print(ser_msg)
                                if (status):
                                        good_data = True
                                else:
                                        ser.reset_input_buffer()
                        for i in range(0, 12, 2):
                                encoderAngle[i//2] = (msg[i]<<8) | msg[i+1]
                        print(encoderAngle)
    except KeyboardInterrupt:
            ser.close()
    return encoderAngle


    
def writeToSerial(writeArray):
    '''
    This functions writes a byte array from the Jetson to the Precise
    Arm using R2Protocol. 
    
    The input 'writeArray' is length 6 for each joint on the Precise Arm.
    Each index of the array is the target angle (in degrees) for each 
    joint.
    
    For example, to get Joints 2 and 3 to move to angles 80 and 90. 
    Say the previous writetoSerial command was [10,20,30,40,50,60]. 
    The next command would then be: writeToSerial([10,80,90,40,50,60]).
    So, change the indices you want to change, and keep the previous
    angles for the joints you don't want to move.
    '''
    print("Is serial port open?", ser.isOpen())
    if not ser.isOpen():
        ser.open()
    print("Is serial port open 2?", ser.isOpen())
    # PRM = Precise Arm
    # cast writeArray from int to byte, encode the array using the R2Protocol
    print(writeArray)
    print("SERIAL PORT", ser)
    write_byte = r2.encode(bytes('PRM','utf-8'), bytearray(writeArray))
    # send the encoded array across the Jetson Serial lines
    ser.write(write_byte)


def publish_updates(updates, timeout):
    ''' This function publishes updates to the arm configuration by writing 
        to the serial buffer. We use R2Protocol to write to the arduino, which 
        will connect/write to the motor controllers directly. 

        Args: 
            updates: a list of update integer lists of size 6. Each update 
                     list consists of 6 angle configurations in radians, 
                     which will need to be converted to degrees before being
                     written to the serial buffer. 
            timeout: an integer that represents the time between updates that 
                     the arm is allotted to move to the desired position. 
    '''
    for index, update_array in enumerate(updates): 
        assert len(update_array) == 6
        for index in range(len(update_array)):
            update_array[index] = int(update_array[index]/360*255)
        #new_update = [0]*12
        #for index in range(len(update_array)):
        #    update_array[2*index] = min(255, int(update_array[index]))
        #    update_array[2*index+1] = max(0, int(update_array[index])-255)
        writeToSerial(update_array)
        ser.reset_input_buffer()
        print("array {} sent: {}".format(index, update_array))
        time.sleep(timeout)

#init_serial('/dev/ttyTHS1', 9600)
    
if __name__ =='__main__':
    #init_serial('/dev/ttyTHS1', 38400)
    read_encoder_values()
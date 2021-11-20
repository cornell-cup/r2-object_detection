import R2Protocol as r2
from networking.Client import *
import numpy as np

# Dedicate a /dev port for specifically the precise arm
ser = serial.Serial(
	port = '/dev/ttyTHS1',
	baudrate = 9600,
)

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
	# PARM = Precise Arm
	# cast writeArray from int to byte, encode the array using the R2Protocol
	write_byte = r2.encode(bytes('PARM','utf-8'), bytearray(writeArray))
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
        writeToSerial(update_array)
        print("array {} sent: {}".format(index, update_array))
        time.sleep(timeout)


def read_startpos():
    ''' This function will read from R2Protocol to get the current angle 
        configuration of the precise arm.

        Returns: 
            encoder_readings: List of 6 integer angles (degrees) that 
                              represents the current angle position of 
                              the arm. 
    ''' 
    encoder_readings = None
    return encoder_readings

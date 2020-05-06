#!/usr/bin/env python3
# import serial
import sys
import numpy as np
#from matplotlib import pylab
#from pylab import *
from math import *
from datetime import datetime
#Combined kinematics to perform forward and inverse kinematics to find
#joint angles to desired location

## Forward Kinematics
def FK(angles,alpha,D,R):
    # Forward Kinematics
    # angles - joint angles (radians)
    # alpha - twist
    # d - offset
    # r - np.prod

    #Number of joints
    n=np.size(angles)-1
    p0=np.array([[0],[0],[0],[1]])

    # find np.cos and np.sin of angles and alpha
    st = np.array([])
    ct = np.array([])
    sa = np.array([])
    ca = np.array([])
    r = np.array([])
    d = np.array([])
    for i in range(0,n+1):
        st = np.concatenate((st,np.sin(angles[i])))
        ct = np.concatenate((ct,np.cos(angles[i])))
        sa = np.concatenate((sa,np.sin(alpha[i])))
        ca = np.concatenate((ca,np.cos(alpha[i])))
        r = np.concatenate((r,R[i]))
        d = np.concatenate((d,D[i]))


    # Find the homogenous transform from point 0 to unknown point
    H= (np.array([[ct[n],-st[n]*ca[n],st[n]*sa[n],r[n]*ct[n]],[st[n],ct[n]*ca[n],-ct[n]*sa[n],r[n]*st[n]],[0,sa[n],ca[n],d[n]],[0,0,0,1]]))
    for i in range(n-1,-1,-1):
        Hi = (np.array([[ct[i],-st[i]*ca[i],st[i]*sa[i],r[i]*ct[i]],[st[i],ct[i]*ca[i],- ct[i]*sa[i],r[i]*st[i]],[0,sa[i],ca[i],d[i]],[0,0,0,1]]))
        H = np.matmul(Hi,H)

    point=np.matmul(H,p0)
    point=point[0:3]
    return point

## Jacobian
def Jacobian(angles,alpha,d,r):
    # Find the Jacobian matrix
    # angles - joint angles (radians)
    # alpha - twist
    # d - offset
    # r - np.prod

    #Number of joints
    n=np.prod(angles)

    # Assign variables to theta
    t1 = angles[0]
    t2 = angles[1]
    t3 = angles[2]
    t4 = angles[3]
    t5 = angles[4]

    # Find the derivatives (NEED TO CHANGE THIS FOR VARYING DOFS ARM)
    j11 = -1*np.sin(t1)*(np.sin(t2+t3+t4)+cos(t2+t3)+np.cos(t2))
    j12 = -np.cos(t1)*(np.sin(t2+t3)-np.cos(t2+t3+t4)+np.sin(t2))
    j13 = np.cos(t1)*(np.cos(t2+t3+t4) - np.sin(t2 + t3))
    j14 = np.cos(t2 + t3 + t4)*np.cos(t1)
    j15 = [0]
    j21 = np.cos(t1)*(np.sin(t2+t3+t4)+np.cos(t2+t3)+np.cos(t2))
    j22 = -np.sin(t1)*(np.sin(t2+t3) - np.cos(t2+t3+t4) + np.sin(t2))
    j23 = np.sin(t1)*(np.cos(t2+t3+t4)-np.sin(t2+t3))
    j24 = np.cos(t2+t3+t4)*np.sin(t1)
    j25 = [0]
    j31 = [0]
    j32 = -np.sin(t2 + t3 + t4)-np.cos(t2+t3)-np.cos(t2)
    j33 = -np.sin(t2 + t3 + t4)-np.cos(t2+t3)
    j34 = -np.sin(t2+t3+t4)
    j35 = [0]

    #Finally, make the jacobian matrix
    r1 = np.concatenate((j11,j12,j13,j14,j15))
    r2 = np.concatenate((j21,j22,j23,j24,j25))
    r3 = np.concatenate((j31,j32,j33,j34,j35))
    J = np.concatenate((r1,r2,r3),axis = 0)
    J = np.reshape(J,(3,5))
    return J

## Find distance to a point
def Dist(point1,point2):

    x1=point1[0]
    y1=point1[1]
    z1=point1[2]
    x2=point2[0]
    y2=point2[1]
    z2=point2[2]
    d=sqrt(np.square(x2 - x1) + np.square(y2 - y1) + np.square(z2 - z1))
    return d

## Inverse Kinematics
def IK(angles,e,g,b,alpha,d,r):
    # Inverse Kinematics
    # angles - joint angles (radians)
    # alpha - twist
    # d - offset
    # r - np.prod
    # beta - i don't know what it is - 0 < ? ? 1.
    # g = desired position
    # e = current position

    #1. Compute J(q).
    J=Jacobian(angles,alpha,d,r)

    #2. Select an increment that will move e closer to g, where 0 < ? ? 1.
    de= b*(g - e)

    #3. Compute the change in the joints variables that will achieve the end-effector increment selected in the previous step
    JT = np.transpose(J)
    dangles=np.matmul(JT,de)

    #4. Apply this change to the joint variable vector, as q = q + ?q.
    angles=angles+dangles

    #5. Update e by computing the forward kinematics of the manipulator with the updated q.
    point=FK(angles,alpha,d,r)

    return angles,point

def kinematics(xg,yg,zg):

    # No scientific notations
    np.set_printoptions(suppress=True)

    # Given desired location (x,y,g)
    giv_point=np.array([[xg],[yg],[zg]])
    pi = np.pi

    # Link sizes - L = [l1,l2,l3,l4]
    l1 = 0.066675
    l2 = 0.104775
    l3 = 0.0889
    l4 = 0.1778
    L=np.array([l1,l2,l3,l4])

    #current joint angles (radians)
    angles=np.array([[0],[0],[0],[0],[0]])
    Z=np.array([[0],[- pi / 2],[0],[pi / 2],[0]])
    angles= np.add(angles, Z)
    alpha=np.array([[- pi / 2],[0],[0],[pi / 2],[0]])
    r=np.array([[0],[L[1]],[L[2]],[0],[0]])
    d=np.array([[L[0]],[0],[0],[0],[L[3]]])


    #Decrease the tolerance to increase accuracy
    tolerance=0.001

    # Find current position based on the current joint angles
    point = FK(angles,alpha,d,r)
    error=np.inf
    b=0.1

    # kinematics.m:32
    # Iterate until joint angles found to the desired location
    while error > tolerance:

        angles,point=IK(angles,point,giv_point,b,alpha,d,r)
        error=Dist(point,giv_point)

    angles[3] = angles[3] - pi/2

    print('Desired Loceation')
    print(giv_point)
    print('Calculated position')
    print(point)
    print('Accuracy')
    print(tolerance)
    print('Angles')
    print(np.rad2deg(angles))
    # Correction for convention
    angles[1] = angles[1] + 5*pi/6
    angles[2] = -1*angles[2] + pi/4
    print(np.rad2deg(angles))

    angles_actual = str().join(','.join('%0.3f' %y for y in np.rad2deg(angles)))
    return np.rad2deg(angles)
    # ser = serial.Serial('/dev/ttyTHS1', 9600)
    # ser.write(bytearray(angles_actual, 'utf8'))
    # ser.close()
# Time taken
startTime = datetime.now()

# Call the kinematics function
kinematics(0,0,0.438)      #Claw points straight up (0,0,4)
print('TIME TAKEN:')
print(datetime.now() - startTime)

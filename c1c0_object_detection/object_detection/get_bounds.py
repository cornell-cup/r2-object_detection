import numpy as np
import matplotlib.pyplot as plt
import sys
import time
import cv2
import os
from datetime import datetime
from matplotlib import image


def find_rectangle(vertices):
    """Takes in a list of vertices and finds the bounding rectangle for each
    vertex. Returns a list in the form: 
    [((xy for lower right corner),((xy for upper left corner)))] """
    rect_params_lst = []
    for vertex in vertices:
        min_x, min_y = np.min(vertex[:,0]), np.min(vertex[:,1])
        max_x, max_y = np.max(vertex[:,0]), np.max(vertex[:,1])
        w, h = max_x - min_x, max_y - min_y 
        # rect_params_lst.append((min_x, min_y, w, h))
        rect_params_lst.append(((min_x, min_y), (max_x, max_y)))
    return rect_params_lst

def get_rect_params(img):
    """ """
    img, vertices = approx_shape(img)
    rect_params = find_rectangle(vertices)
    return rect_params


def approx_shape(img):
    """ Approximates shapes into polygons"""
    # Convert to greyscale
    if len(img.shape) == 3:
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        img_gray = img
   
    #cv2.imshow('gray', img_gray)
    # Convert to binary image by thresholding
    _, threshold = cv2.threshold(img_gray, 245, 255, cv2.THRESH_BINARY_INV)
    # Find the contours
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # For each contour approximate the curve and
    # detect the shapes.
    vertices = []
    tmp = img.copy()
    tmp = cv2.cvtColor(tmp, cv2.COLOR_GRAY2RGB)
    for cnt in contours :
        area = cv2.contourArea(cnt)
        # Shortlisting the regions based on there area.
        if area > 600: 
            approx = cv2.approxPolyDP(cnt, 
                                    0.05 * cv2.arcLength(cnt, True), True)
            
            cv2.drawContours(tmp, [approx], 0, (0, 0, 255), 3)
            vertices.append(approx)
            
    
    #cv2.imshow("approximated shape", tmp)
    #cv2.waitKey()
    #print (vertices)
    for i in range (0, len(vertices)):
        vertices[i] = vertices[i].reshape(len(vertices[i]),2)
    # print (vertices)
    
    return img, vertices

def draw_boxes(img):
    """ draw boxes for visualization """
    rect_params = get_rect_params(img)
    tmp = img.copy()
    for param in rect_params:
        cv2.rectangle(tmp,param[0],param[1],(100,100,100),1)
    #cv2.imshow("boxes", tmp)
    #cv2.waitKey()

def erode_and_dilate(img, debug=False):
    # erosion takes the min value of the kernel and 
    # dilation takes the max value of the kernel
    kernel = np.ones((7,7), np.uint8)
    
    img_dilated = cv2.dilate(img, kernel, iterations=4)
    img_erosion = cv2.erode(img_dilated, kernel, iterations=1)
    if debug:
            cv2.imshow("eroded", img_erosion)
            cv2.imshow("dilated", img_dilated)
            cv2.waitKey()
    return img_dilated

def get_bound (img, debug=False):
    # gets xy coordinates of bottom left corner of bounding boxes
    # returns list of tuples in the form (x,y,w,h)
    box_coords = []
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    vals = np.unique(gray)
    for v in vals:
        if v == 255:
            continue
        tmp = gray.copy()
        #create a mask where all values equal to v are 1 and everything else is 0
        label_val = 0
        mask = (tmp == v).astype(int) 
        tmp[mask==0] = 255
        tmp[mask==1] = label_val
        
        # Re update the mask using the dilated image where all 
        # desired pixels (white) are set to 1
        denoised_img = erode_and_dilate(tmp, debug)
        mask = (denoised_img == label_val).astype(int)
        draw_boxes(denoised_img)
        box_coords.append(get_rect_params(denoised_img))
        

        if debug:
            # draw bounding box rectangle
            # cv2.rectangle(tmp,(x,y),(x+w,y+h),(255,0,0),3)
            cv2.imshow("org", img)
            cv2.imshow("tmp", tmp)
            cv2.waitKey()
            draw_boxes(tmp)
    return box_coords



def main():
    path = os.path.join(os.getcwd(), "kmeans_test_imgs", "30-04-15:06:00")
    img = cv2.imread(path+"/Result.jpg")
    #approx_shape(img)
    draw_boxes(img)
    

if __name__ == "__main__":
    main()

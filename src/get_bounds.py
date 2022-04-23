import numpy as np
import matplotlib.pyplot as plt
import sys
import time
import cv2
from sklearn import preprocessing
import os
from datetime import datetime
from matplotlib import image


def connected_component_analysis(img):
    output = cv2.connectedComponentsWithStats(
	img, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
    # loop over the number of unique connected component labels
    for i in range(0, numLabels):
        # if this is the first component then we examine the
        # *background* (typically we would just ignore this
        # component in our loop)
        if i == 0:
            text = "examining component {}/{} (background)".format(
                i + 1, numLabels)
        # otherwise, we are examining an actual connected component
        else:
            text = "examining component {}/{}".format( i + 1, numLabels)
        # print a status message update for the current connected
        # component
        print("[INFO] {}".format(text))
        # extract the connected component statistics and centroid for
        # the current label
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]
        (cX, cY) = centroids[i]

        # clone our original image (so we can draw on it) and then draw
        # a bounding box surrounding the connected component along with
        # a circle corresponding to the centroid
        output = image.copy()
        cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)
        # construct a mask for the current connected component by
        # finding a pixels in the labels array that have the current
        # connected component ID
        componentMask = (labels == i).astype("uint8") * 255
        # show our output image and connected component mask
        cv2.imshow("Output", output)
        cv2.imshow("Connected Component", componentMask)
        cv2.waitKey(0)

    pass


def erode_and_dilate(img, debug=False):

    # erosion takes the min and dilation takes the max
    # since we want to keep white pixels we erode then dilate
    kernel = np.ones((7,7), np.uint8)
    img_erosion = cv2.erode(img, kernel, iterations=4)
    img_dilated = cv2.dilate(img_erosion, kernel, iterations=1)
    if debug:
            #viz_image([img_erosion,img_dilated],["eroded", "dilated"])
            cv2.imshow("eroded", img_erosion)
            cv2.imshow("dilated", img_dilated)
    return img_dilated

def get_bound (img, debug=False):
    # gets xy coordinates of bottom left corner of boudning boxes.
    # returns list of tuples in the form (x,y,w,h)
    box_coords = []
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    vals = np.unique(gray)
    for v in vals:
        if v == 255:
            continue
        tmp = gray.copy()
        #create a mask where all values equal to v are 1 and everything else is 0
        label_val = 150
        mask = (tmp == v).astype(int) 
        tmp[mask==0] = 0
        tmp[mask==1] = label_val
        
        # Re update the mask using the dilated image where all 
        # desired pixels (white) are set to 1
        denoised_img = erode_and_dilate(tmp, debug)
        mask = (denoised_img == label_val).astype(int)
        
        active_px = np.argwhere(mask!=0)
        active_px = active_px[:,[1,0]]
        x,y,w,h = cv2.boundingRect(active_px)
        box_coords.append((x,y,w,h))

        if debug:
            # draw bounding box rectangle
            cv2.rectangle(tmp,(x,y),(x+w,y+h),(255,0,0),1)
            cv2.imshow("org", img)
            cv2.imshow("tmp", tmp)
            cv2.waitKey()
    return box_coords
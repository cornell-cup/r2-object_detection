import math
import cv2
import imutils
import numpy as np


def grab_points(x1, y1, width_box, height_box, image):

    def auto_canny(image, sigma=0.33):

        # compute the median of the single channel pixel intensities
        v = np.median(image)

        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)

        # return the edged image
        return edged

    #  applies Gaussian blur and canny edge detection to the image
    #  parameters are the image file, the midpoint, and canny edge
    #  thresholds

    def canny_edge(image_file, width, height):

        # convert image to grayscale and run canny edge detection on it
        grayscale = cv2.cvtColor(image_file, cv2.IMREAD_GRAYSCALE)

        # blur the grayscale image
        blurred = cv2.GaussianBlur(grayscale, (5, 5), cv2.BORDER_DEFAULT)

        edge = auto_canny(grayscale)
        # find contours on the grayscale image
        cnts = cv2.findContours(edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        #  apply Gaussian blur on source image
        blurred = cv2.GaussianBlur(image_file, (5, 5), cv2.BORDER_DEFAULT)
        #  apply contrast to the blurred image
        l, a, b = cv2.split(blurred)

        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        cl = clahe.apply(l)

        # -----Merge the CLAHE enhanced L-channel with the a and b channel-----------
        limg = cv2.merge((cl, a, b))

        # -----Converting image from LAB Color model to RGB model--------------------
        final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

        # apply edge detection to the contrast image
        edge = auto_canny(final)
        cv2.drawContours(edge, cnts, -1, (255, 0, 0), 5)

        # add more contours to the grayscale canny image from the contrast image
        cnts = cv2.findContours(edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
        k = np.zeros(shape=[height, width, 3], dtype=np.uint8)

        # Draw the largest contour on an all-black image
        cv2.drawContours(k, [cnts], -1, (255, 255, 255), 1)
        M = cv2.moments(cnts)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return k, [cX, cY]

    def distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    #  uses the canny edge image and the midpoint to determine the two points
    #  that the robot arm needs to grab
    def shortest_path(original, edge, mid_contour, w, h):
        pix_val = []
        half_cols = w//2
        total_cols = w
        total_rows = h
        #  range goes from halfway through the x direction and
        #  the whole way in the y direction
        for i in range(half_cols):
            for j in range(total_rows):
                g = edge[j][i][1]
                if g == 255:
                    pix_val.append([i, j])

        min_distance = float("inf")

        val_x1, val_y1, val_x2, val_y2 = -1, -1, -1, -1

        for coor in pix_val:
            col, row = coor[0], coor[1]
            theta = math.atan2((mid_contour[0] - col), (mid_contour[1] - row))
            for radius in range(int(min(total_rows, total_cols)/2)):
                new_col = min(mid_contour[0] +
                              math.sin(theta)*radius, total_cols-1)
                new_row = min(mid_contour[1] +
                              math.cos(theta)*radius, total_rows-1)
                g = edge[int(new_row)][int(new_col)][1]
                if g == 255:
                    dist = distance(new_col, new_row, col, row)
                    if dist < min_distance:
                        val_x1 = col
                        val_y1 = row
                        val_x2 = new_col
                        val_y2 = new_row
                        min_distance = dist
        # changed edge to original to draw on original img
        cv2.circle(original, (int(val_x1), int(val_y1)), 3, (255, 0, 255), -1)
        cv2.circle(original, (int(val_x2), int(val_y2)), 3, (255, 0, 255), -1)
        cv2.circle(original, (int(mid_contour[0]), int(
            mid_contour[1])), 3, (255, 0, 255), -1)

        cv2.circle(edge, (int(val_x1), int(val_y1)), 3, (255, 0, 255), -1)
        cv2.circle(edge, (int(val_x2), int(val_y2)), 3, (255, 0, 255), -1)
        cv2.circle(edge, (int(mid_contour[0]), int(
            mid_contour[1])), 3, (255, 0, 255), -1)

        # Uncomment to see final image/edges/grasping points!
        # cv2.imshow("canny with points", edge)
        # cv2.imshow("points", original)
        # cv2.waitKey(0)

        return val_x1, val_y1, val_x2, val_y2, min_distance

    raw_img = image
    width, height = width_box, height_box
    cropped_image = raw_img[y1:y1+height_box, x1:x1+width_box]

    ceRet = canny_edge(cropped_image, width, height)
    edge_image = ceRet[0]

    shortest_x1, shortest_y1, shortest_x2, shortest_y2, shortest_dist = shortest_path(
        raw_img, edge_image, ceRet[1], width, height)
    return shortest_x1 + x1, shortest_y1 + y1, shortest_x2 + x1, shortest_y2 + y1, shortest_dist


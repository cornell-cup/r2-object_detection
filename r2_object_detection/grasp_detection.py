import cv2
import imutils
import numpy as np

"""
returns the grasping coordinates on the passed in image
"""
def grab_points(image):
    """ 
    Parameters
    - image, an ndarray of pixels
    - sigma
    Returns
    - image with canny edge detection run on each channel's median
    """
    def auto_canny(image, sigma=0.33):

        # compute the median of the single channel pixel intensities
        v = np.median(image)

        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)

        # return the edged image
        return edged

    """ 
    Parameters
    - image
    - width
    - height
    Returns
    - k, a 3D image of white contours on black background
    - [cX, cY], the center of mass coordinates in 2D
    """
    def canny_edge(image):

        width, height = image.shape[1], image.shape[0]
        # convert (height, width, 3) image to grayscale and run canny edge detection on it
        grayscale = cv2.cvtColor(image, cv2.IMREAD_GRAYSCALE)

        # blur the grayscale image
        blurred = cv2.GaussianBlur(grayscale, (5, 5), cv2.BORDER_DEFAULT)

        edge = auto_canny(grayscale)
        # find contours on the grayscale image
        cnts = cv2.findContours(edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        #  apply Gaussian blur on source image
        blurred = cv2.GaussianBlur(image, (5, 5), cv2.BORDER_DEFAULT)
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
        # find contour with greatest area
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
        # 3 chan image of contours
        k = np.zeros(shape=[height, width, 3], dtype=np.uint8)

        # Draw the largest contour on an all-black image
        # But the -1 means all contours are drawn?
        cv2.drawContours(k, [cnts], -1, (255, 255, 255), 1)
        M = cv2.moments(cnts)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return k, [cX, cY]

    """ 
    Parameters
    - original, the original image
    - edge, the edge image
    - mid_contour, the coordinate (x, y) of the center of mass
    Returns
    - coordinates of two grasp points and the distance
    """
    def shortest_path(original, edge, mid_contour):
        g = edge[:, :, 1]

        # array of row indices and array of col indices of contour points
        pix_val = np.where(g == 255)

        min_distance = float("inf")

        val_x1, val_y1, val_x2, val_y2 = -1, -1, -1, -1

        row_numbers, col_numbers = pix_val[0], pix_val[1]
        # coords is the coordinates in pairs
        coords = np.concatenate(
            [row_numbers[:, None], col_numbers[:, None]], axis=1)

        # theta is the angle b/w each coordinate and the centroid of the object
        theta = np.degrees(np.arctan2(
            mid_contour[0] - col_numbers, mid_contour[1] - row_numbers))

        # dictionary mapping angles to coords
        coords_of_theta = {}

        # put thetas with corresponding coordinates into dictionary
        for i in range(len(theta)):
            coords_of_theta[int(theta[i])] = coords[i]

        for i in range(len(theta)):
            # wrap angle ranges
            angle = int(theta[i])
            opposite_angle = angle + 180
            if opposite_angle > 180:
                opposite_angle = -180 + angle
            # look for opposing coords with shortest distance between
            if opposite_angle in coords_of_theta:  # mod to get in range -180 to 180
                dist = np.linalg.norm(
                    coords_of_theta[angle] - coords_of_theta[opposite_angle])
                if dist < min_distance:
                    # row = y coordinate, col = x coordinate
                    val_y1, val_x1 = coords_of_theta[angle]
                    val_y2, val_x2 = coords_of_theta[opposite_angle]
                    min_distance = dist

        cv2.circle(original, (int(val_x1), int(val_y1)), 3, (255, 0, 255), -1)
        cv2.circle(original, (int(val_x2), int(val_y2)), 3, (255, 0, 255), -1)
        cv2.circle(original, (int(mid_contour[0]), int(
            mid_contour[1])), 3, (255, 0, 255), -1)

        cv2.circle(edge, (int(val_x1), int(val_y1)), 3, (255, 0, 255), -1)
        cv2.circle(edge, (int(val_x2), int(val_y2)), 3, (255, 0, 255), -1)
        cv2.circle(edge, (int(mid_contour[0]), int(
            mid_contour[1])), 3, (255, 0, 255), -1)

        # Uncomment to see final image/edges/grasping points!
        cv2.imshow("canny with points", edge)
        cv2.imshow("points", original)

        # cv2.waitKey(0)

        return val_x1, val_y1, val_x2, val_y2, min_distance

    # canny edge return
    ceRet = canny_edge(image)
    edge_image = ceRet[0]

    shortest_x1, shortest_y1, shortest_x2, shortest_y2, shortest_dist = shortest_path(
        image, edge_image, ceRet[1])
    return shortest_x1, shortest_y1, shortest_x2, shortest_y2, shortest_dist


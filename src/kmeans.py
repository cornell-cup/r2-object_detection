from sklearn.cluster import KMeans
import numpy as np
import matplotlib.pyplot as plt
import sys
import get_depth_frame as df
import time
import cv2
from sklearn import preprocessing
import os
from datetime import datetime

def process_features(X, norm_matrix):
    """
    Preprocess features before feeding into algorithm
    """
    c = X.shape[-1]

    X = X.reshape((-1, c))
    #X = preprocessing.normalize(X, norm='l2')
    X = np.divide(X, norm_matrix) #The last value should have a number between 1 and 10
    X = X.reshape((480, 640, c))
    return np.float32(X)



def sel_features(input_matrix):
    """
    Selects the desired features in a depth frame
    TODO: Reformat the depth frame or reshape the output
    """
    coord_list = []
    for y in reversed(range(0, len(input_matrix))):
        for x in range (0, len(input_matrix[0])):
            coord_list.append([input_matrix[y,x],x,abs(y-len(input_matrix)+1)])
    return coord_list




def viz_image(images, names):
    """
    Given a list of images and their corresponding names, display them
    """
    for i in range(len(images)):
        cv2.imshow(names[i], images[i])
    print("press q to save images, press r to quit, press anything else to load a new image")
    
    key = cv2.waitKey(0)
    if key & 0xFF == ord('q') or key == 27:
        print ("Saving images")
        now = datetime.now()
        dt_str = now.strftime("%d-%m-%H:%M:%S")
        pth = "kmeans_test_imgs/" + dt_str
        os.mkdir(pth)
        for i in range(len(images)):
            cv2.imwrite(pth + '/' + names[i]+'.jpg', images[i])

        cv2.destroyAllWindows()

    if key & 0xFF == ord('r'):
        cv2.destroyAllWindows()

def cv_kmeans(input_matrix, k):
    """ Performs kmeans clustering on the input matrix. Expect the input_matrix to have dimension (k, d, c) where k
	and d are the image dimension and c is the number of channels in the image
    """
    
    img = input_matrix
    #img = sel_features(img)
    img = process_features(img, [1,1,1,1.5])
    print ("image shape: ", img.shape,"image: ", img)
    c = img.shape[-1]

    
    twoDimg = img.reshape((-1,c))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    attempts = 10
    ret, label, center = cv2.kmeans(twoDimg, k, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)
    print ("running kmeans")
    center = np.uint8(center)
    
    X_trans = preprocessing.normalize
    res = center[label.flatten()]
    result_image = res.reshape((img.shape))
    return result_image


def main():
    org_image, depth_img, rgbd = df.get_depth_frame()
    print (org_image.shape)
    result_img = cv_kmeans(rgbd, 5)
    viz_image([org_image, result_img, depth_img], ["Orignal", "Result", "Depth Frame"])



if __name__ == "__main__":
    main()

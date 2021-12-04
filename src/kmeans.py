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

def process_features(X):
    """
    Preprocess features before feeding into algorithm
    """
    X_trans = preprocessing.normalize(X.T, norm='l2').T
    return X_trans



def sel_features(input_matrix):
    """
    Selects the desired features in a depth frame
    TODO: Reformat the depth frame or reshape the output
    """
    coord_list = []
    for y in reversed(range(0, len(input_matrix))):
        for x in range (0, len(input_matrix[0])):
            coord_list.append([input_matrix[y,x],x,abs(y-len(input_matrix)+1)])



def visualize(hist):
    plt.plot(hist)
    plt.show()


def viz_image(images, names):
    """
    Given a list of images and their corresponding names, display them
    """
    for i in range(len(images)):
        cv2.imshow(names[i], images[i])
    key = cv2.waitKey(0)
    
    print ("Press q to save image, press anything else to exit")
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

def cv_kmeans(input_matrix):
    """ Performs kmeans clustering on the input matrix. Expect the input_matrix to have dimension (k, d, c) where k
	and d are the image dimension and c is the number of channels in the image
    """
    
    img = input_matrix
    print ("image shape: ", img.shape,"image: ", img)
    c = img.shape[-1]

    
    twoDimg = img.reshape((-1,c))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K=10
    attempts = 10
    ret, label, center = cv2.kmeans(twoDimg, K, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)
    print ("running kmeans")
    center = np.uint8(center)
    
    res = center[label.flatten()]
    result_image = res.reshape((img.shape))
    return result_image


def main():
    org_image, depth_img, rgbd = df.get_depth_frame()
    print (org_image.shape)
    result_img = cv_kmeans(rgbd)
    viz_image([org_image, result_img, depth_img], ["Orignal", "Result", "Depth Frame"])



if __name__ == "__main__":
    main()

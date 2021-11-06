from sklearn.cluster import KMeans
import numpy as np
import matplotlib.pyplot as plt
import sys
import get_depth_frame as df
import time
import cv2

def sel_features(input_matrix):
    """
    Selects the desired features in a depth frame
    TODO: Reformat the depth frame or reshape the output
    """
    tmp =  input_matrix.flatten()
    print ("Shape of input matrix: ", input_matrix.shape)
    input_vec = np.reshape(tmp, (len(tmp), 1))
    print ("shape of input: ", input_vec.shape)
    return input_vec


def kmeans(input_matrix, thresh, n=5,  max_iter=10, debug=False):
    """
    Runs kmeans clustering on point cloud data. Returns a clustering with
    n clusters. n is determined when previous loss - curr loss < threshold 
    Requires that max_iter >= 1.
    Returns a 2D array where the value at index [i,j] is the label assigned
    to that index. So if [0,0] has value 1, it cv2.imshow("segmentation", org_image_cop)means [0,0] has label 1.\
    UPDATE: function now just returns a reshaped matrix containing the
    list of labels
    """
    old_inertia = sys.maxsize
    iters = 0
    hist = []
    if max_iter <= 0:
        raise ("max_iter has to be 1 or greater")

    features = sel_features(input_matrix)
    start = time.time()
    while True:
        iters += 1
        print ("iteration ", iters)
        kmeans = KMeans(n_clusters=n).fit(features)
        if old_inertia - kmeans.inertia_ < thresh or iters >= max_iter:
            print ("clustering took ", time.time()-start, " seconds")
            if debug:
                print ("visualizing hist:")
                visualize(hist)
            """ 
            If max_iter = 1, then return current clustering. Otherwise return
            the previous clustering
            """
            if max_iter == 1:
                labels = kmeans.labels_
            else:
                labels = old_kmeans.labels_
            points = np.reshape(labels, (480,640))
            print (points)
            return points
        n += 1
        old_kmeans = kmeans
        old_inertia = kmeans.inertia_
        hist.append(kmeans.inertia_)

def visualize(hist):
    plt.plot(hist)
    plt.show()


def viz_image(org_image, labels):
    """
    Given the original image and a list of labels that 
    assign each pixel to a label, make a mask of the image
    """
    """cv2.imshow("original", cv2.cvtColor(org_image, cv2.COLOR_RGBA2BGR))"""
    org_image_cop = org_image
    max_label = np.max(labels)
    mul_fact = 255/max_label

    for i in range(0,max_label+1):
        mask = labels == i
        org_image_cop[mask] = i*mul_fact
    cv2.imshow("segmentation", org_image_cop)
    cv2.waitKey()


def main():
    dist_matrix, org_image = df.get_depth_frame()
    print ("displaying image: ")
    print (org_image.shape)
    cv2.imshow("original", cv2.cvtColor(org_image, cv2.COLOR_RGBA2BGR))
    cv2.waitKey()
    points = kmeans(dist_matrix, 2, debug=True)
    viz_image(org_image, points)

if __name__ == "__main__":
    main()

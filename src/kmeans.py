from sklearn.cluster import KMeans
import numpy as np
import matplotlib.pyplot as plt
import sys
import get_depth_frame as df
import time
import cv2
from sklearn import preprocessing


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


    print ("Shape of input matrix: ", input_matrix.shape)
    input_vec = np.array(coord_list)
    print (input_vec)
    print ("shape of input: ", input_vec.shape)
    return input_vec


def kmeans(input_matrix, thresh, n=5,  max_iter=5, debug=False):
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
    features = process_features(features)
    print ("Processed features:  ", features)
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
    cv2.imshow("original", cv2.cvtColor(org_image, cv2.COLOR_RGBA2BGR))
    org_image_cop = org_image
    max_label = np.max(labels)
    mul_fact = 255/max_label

    for i in range(0,max_label+1):
        mask = labels == i
        org_image_cop[mask] = i*mul_fact
    cv2.imshow("segmentation", org_image_cop)
    cv2.waitKey()


def main():
    dist_matrix, org_image, depth_img = df.get_depth_frame()
    print ("displaying image: ")
    print (org_image.shape)
    cv2.imshow("original", cv2.cvtColor(org_image, cv2.COLOR_RGBA2BGR))
    cv2.imshow("depth", depth_img)
    cv2.waitKey()
    img = org_image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    

    twoDimg = img.reshape((-1,3))
    twoDdepth_img = depth_img.reshape((-1,1))
    print ("depth image shape : ", depth_img.shape)
    twoDimg = np.float32(twoDimg)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K=5
    attempts = 10
    ret, label, center = cv2.kmeans(twoDimg, K, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)
    center = np.uint8(center)
    res = center[label.flatten()]
    result_image = res.reshape((img.shape))
    cv2.imshow("original", cv2.cvtColor(org_image, cv2.COLOR_RGBA2BGR))
    cv2.imshow("result: ",result_image)
    cv2.waitKey()
    #points = kmeans(dist_matrix, 2, debug=True)
    #viz_image(org_image, points)

if __name__ == "__main__":
    main()

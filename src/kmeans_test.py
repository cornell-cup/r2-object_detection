
import numpy as np
import matplotlib.pyplot as plt
import sys
import time
import cv2
from sklearn import preprocessing
import os
from datetime import datetime
from matplotlib import image

def process_features(X, norm_matrix):
    """
    Preprocess features before feeding into algorithm
    """
    c = X.shape[-1]

    X = X.reshape((-1, c))
    X = np.divide(X, norm_matrix) #The last value should have a number between 1 and 10
    X = X.reshape((480, 640, c))
    return np.float32(X)



def sel_features(input_matrix):
    """
    Selects the desired features in a depth frame
    """
    coord_list = []
    for y in reversed(range(0, len(input_matrix))):
        for x in range (0, len(input_matrix[0])):
            coord_list.append([input_matrix[y,x],x,abs(y-len(input_matrix)+1)])
    return coord_list


def preprocess_data(depth_image, color_matrix, rgbd):
    rgbd = process_features(rgbd, [1,1,1,1.])
    return rgbd
    

def postprocess_im(depth_img, segmentation, labelled_img):
    """Identify points that violate the depth image and identify the 
    labels of the pixels. If >prop% of pixels of that label violate the depth 
    threshold then remove it from the image """
    """
    for each label:
        get all pixels associated with label. Call this set P
        for each pixel p in P, check if p's depth value is smaller than the threshold
        if the number of violating pixels in P is greater than prop% of |P|,
        white out the segmentation
    """

    # Fixed threshold either has 125 or 100 refer to 04-12-14:49:33 and 04-12-13:10:36
    thresh = max(np.min(depth_img), 100) # set threshold
    labels = np.unique(labelled_img)
    labelled_img = labelled_img.reshape(depth_img.shape)
    prop = 0.3
    
    for label in labels:
        coords = np.count_nonzero(labelled_img == label)
        # a pixel violates if it's part of P and depth value is less than thresh
        violated_px = labelled_img[(labelled_img==label) & (depth_img < thresh)]
        if len(violated_px) > prop * coords:
            print ("label", label,"violated")
            segmentation[labelled_img==label] = [255,255,255,255]

    cv2.imshow('new segmentation', segmentation)
    cv2.waitKey()
    return segmentation



def viz_image(images, names):
    """
    Given a list of images and their corresponding names, display them
    """
    for i in range(len(images)):
        cv2.imshow(names[i], images[i])
    print("press r to save images, press q to quit, press anything else to load a new image")
    
    key = cv2.waitKey(0)
    if key & 0xFF == ord('r') or key == 27:
        print ("Saving images")
        now = datetime.now()
        dt_str = now.strftime("%d-%m-%H:%M:%S")
        pth = "kmeans_test_imgs/" + dt_str
        os.mkdir(pth)
        for i in range(len(images)):
            cv2.imwrite(pth + '/' + names[i]+'.jpg', images[i])

        cv2.destroyAllWindows()

    if key & 0xFF == ord('q'):
        cv2.destroyAllWindows()

def cv_kmeans(img, debug=False):
    """ 
    Performs kmeans clustering on the input matrix. Expect the input_matrix 
    to have dimension (k, d, c) where k and d are the image dimension and c is 
    the number of channels in the image
    """
    c = img.shape[-1]
    twoDimg = img.reshape((-1,c))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    print ("running kmeans")
    distortions = []
    labels=[]
    K_max=6
    for k in range(1,K_max):
        attempts = 10
        ret, label, center = cv2.kmeans(twoDimg, k, None, criteria, attempts, 
        cv2.KMEANS_PP_CENTERS)
        labels.append(label)
        center = np.uint8(center)
        distortions.append(ret)

    if debug:
        plt.plot(range(1,K_max), distortions, 'bx-')
        plt.xlabel('k')
        plt.ylabel('Distortion')
        plt.title('The Elbow Method showing the optimal k')
        plt.show()
    X_trans = preprocessing.normalize
    idx = 0
    for i in range(len(distortions)-1):
        if (distortions[i] - distortions[i+1] < (distortions[0]/10)):
            idx = i
            break
    print("k: ", idx+1)

    res = center[labels[idx].flatten()]
    result_image = res.reshape((img.shape))

    cv2.imshow("res", result_image)
    return result_image, labels[idx]


def get_depth_images(dir):
    path = os.path.join(os.getcwd(), "kmeans_test_imgs", dir)
    org_img = image.imread(path+"/Orignal.jpg")
    RGB_img = cv2.cvtColor(org_img, cv2.COLOR_BGR2RGB)
    depth_img = cv2.imread(path+"/Depth Frame.jpg",0)
    
    
    return RGB_img, depth_img

def create_rgbd(rgb_img, depth_img):
    b, g, r = cv2.split(rgb_img)
    d = depth_img.copy().astype('float32')
    r, g, b = r.astype('float32'), g.astype('float32'), b.astype('float32')
    rgbd = cv2.merge([r, g, b, d])
    
    return rgbd

def get_bound (img, debug=False):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    vals = np.unique(gray)
    for v in vals:
        if v == 255:
            continue
        tmp = gray.copy()
        #create a mask where all values equal to v are 1 and everything else is 0
        mask = (tmp == v).astype(int) 
        
        tmp[mask==0] = 0
        tmp[mask==1] = 150
        
        # erosion takes the min and dilation takes the max
        # since we want to keep white pixels we erode then dilate
        kernel = np.ones((7,7), np.uint8)
        img_erosion = cv2.erode(tmp, kernel, iterations=4)
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=1)

        # Re update the mask using the dilated image where all 
        # desired pixels (white) are set to 1
        mask = (img_dilation == 150).astype(int)
        active_px = np.argwhere(mask!=0)
        active_px = active_px[:,[1,0]]
        x,y,w,h = cv2.boundingRect(active_px)
        
        if debug:
            cv2.imshow("eroded", img_erosion)
            cv2.imshow("dilated", img_dilation)
        cv2.rectangle(tmp,(x,y),(x+w,y+h),(255,0,0),1)
        cv2.imshow("org", img)
        cv2.imshow("tmp", tmp)
        
        cv2.waitKey()
    
    cv2.imshow("gray", gray)




def main():
    org_img, depth_img = get_depth_images("04-12-14:51:37")
    rgbd = create_rgbd(org_img, depth_img)
    cv2.imshow('org',org_img)

    preprocessed_rgbd = preprocess_data(depth_img, org_img, rgbd)
    result_img, labels = cv_kmeans(rgbd)
    result_img = postprocess_im(depth_img, result_img, labels)
    get_bound(result_img, True)
    
    viz_image([org_img, result_img, depth_img], ["Orignal", "Result", "Depth Frame"])



if __name__ == "__main__":
    main()



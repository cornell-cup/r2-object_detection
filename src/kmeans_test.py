
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

def cv_kmeans(input_matrix):
    """ Performs kmeans clustering on the input matrix. Expect the input_matrix to have dimension (k, d, c) where k
    and d are the image dimension and c is the number of channels in the image
    """
    img = input_matrix
    #img = sel_features(img)

    img = process_features(img, [1,1,1,1.2])
    print ("image shape: ", img.shape,"image: ", img)
    c = img.shape[-1]
    twoDimg = img.reshape((-1,c))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    print ("running kmeans")
    distortions = []
    labels=[]
    K_max=5
    for k in range(1,K_max):
        attempts = 10
        ret, label, center = cv2.kmeans(twoDimg, k, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)
        labels.append(label)
        center = np.uint8(center)
        distortions.append(ret)

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
    cv2.imshow("res",result_image)
    cv2.waitKey()
    return result_image


def get_depth_images(dir):
    path = os.path.join(os.getcwd(), "kmeans_test_imgs", dir)
    org_img = image.imread(path+"/Orignal.jpg")

    depth_img = cv2.imread(path+"/Depth Frame.jpg",0)
    b, g, r = cv2.split(org_img)
    depth_img = depth_img.astype('float32')
    r, g, b = r.astype('float32'), g.astype('float32'), b.astype('float32')
    rgbd = cv2.merge([r, g, b, depth_img])
    
    return org_img, depth_img, rgbd


def get_bound (img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    vals = np.unique(gray)
    for v in vals:
        
        tmp = gray.copy()
        #create a mask where all values equal to v are 1 and everything else is 0
        mask = (tmp == v).astype(int) 
        
        tmp[mask==0] = 0
        tmp[mask==1] = 150
        
        # erosion takes the min and dilation takes the max
        # since we want to keep white pixels we erode then dilate
        kernel = np.ones((5,5), np.uint8)
        img_erosion = cv2.erode(tmp, kernel, iterations=4)
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=1)

        # Re update the mask using the dilated image where all 
        # desired pixels (white) are set to 1
        mask = (img_dilation == 150).astype(int)
        active_px = np.argwhere(mask!=0)
        active_px = active_px[:,[1,0]]
        x,y,w,h = cv2.boundingRect(active_px)
        
        cv2.imshow("eroded", img_erosion)
        cv2.imshow("dilated", img_dilation)
        cv2.rectangle(tmp,(x,y),(x+w,y+h),(255,0,0),1)
        cv2.imshow("org", img)
        cv2.imshow("tmp", tmp)
        
        cv2.waitKey()
    
    cv2.imshow("gray", gray)



def main():
    org_image, depth_img, rgbd = get_depth_images("04-12-15:49:55")
    cv2.imshow("org_im", org_image)
    result_img = cv_kmeans(rgbd)
    get_bound(result_img)
    
    viz_image([org_image, result_img, depth_img], ["Orignal", "Result", "Depth Frame"])



if __name__ == "__main__":
    main()


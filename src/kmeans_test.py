
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


def preprocess_im(depth_image, color_matrix):
    """ Apply a mask on the depth map. Basically you look at the pixels
    that have a high depth value then color those parts super white"""
    
    # max_depth = np.min(depth_image) #get max depth value of scene
    # pct_depth = 0.9
    # thresh = 110 # min(max_depth * (1+pct_depth)+10, 50) #set thresh, check if min or max

    # mask = (depth_image > thresh).astype(int) # mask is 0 if depth val > thresh
    # color_matrix[mask==0] = [255, 255, 255]
    # gray = cv2.cvtColor(color_matrix, cv2.COLOR_BGR2GRAY)
    # thresh = cv2.threshold(gray, 11, 255, cv2.THRESH_BINARY)[1]
    # cv2.imshow("gray", gray)
    # cv2.imshow("depth",depth_image)
    # cv2.imshow("color",color_matrix)

    return color_matrix
    
def unique_count_app(a):
    colors, count = np.unique(a.reshape(-1,a.shape[-1]), axis=0, return_counts=True)
    return colors[count.argmax()]

def postprocess_im(depth_img, segmentation, labelled_img):
    """Identify points that violate the depth image and identify the 
    labels of the pixels. If >50% of pixels of that label violate the depth 
    threshold then remove it from the image """


    """
    for each label:
        get all pixels associated with label. Call this set P
        for each pixel p in P, check if p's depth value is smaller than the threshold
        if the number of violating pixels in P is greater than 50% of |P|, white out the segmentation
     """

    thresh = 50
    labels = np.unique(labelled_img)
    labelled_img = labelled_img.reshape(depth_img.shape)
    for label in labels:
        coords = np.where(labelled_img == label)
        violated_px = labelled_img[(labelled_img==label) & (depth_img < thresh)]
        if len(violated_px) > 0.5 * len(labelled_img):
            print ("label ", label,"violated")
            segmentation[labelled_img==label] = [255,255,255,255]

    cv2.imshow('new segmentation', segmentation)
    cv2.waitKey()
    # print(segmentation)
    # print(segmentation[segmentation==common_label][0])
    #cv2.imshow('seg', segmentation)
    #cv2.waitKey()


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

    img = process_features(img, [1,1,1,1.5])
    c = img.shape[-1]
    twoDimg = img.reshape((-1,c))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    print ("running kmeans")
    distortions = []
    labels=[]
    K_max=6
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
    # why is it 4?
    res = center[labels[idx].flatten()]
    result_image = res.reshape((img.shape))
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

def get_bound (img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    vals = np.unique(gray)
    print (vals)
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
    org_img, depth_img = get_depth_images("04-12-14:49:33")
    rgb_img = preprocess_im(depth_img, org_img)
    rgbd = create_rgbd(rgb_img, depth_img)
    


    result_img, labels = cv_kmeans(rgbd)
    postprocess_im(depth_img, result_img, labels)
    

    get_bound(result_img)
    
    viz_image([org_img, result_img, depth_img], ["Orignal", "Result", "Depth Frame"])



if __name__ == "__main__":
    main()


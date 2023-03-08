import numpy as np
import matplotlib.pyplot as plt
import sys
import time
import cv2
import os
from datetime import datetime
from matplotlib import image
from .get_bounds import get_bound as bound
#import get_depth_frame as df

def preprocess_data(depth_img, rgbd, crop):
    width, height = depth_img.shape[0], depth_img.shape[1]
    xy_matrix = np.indices((width, height)).transpose(1,2,0)
    rgbd = np.concatenate((rgbd, xy_matrix), axis=2)

    if not crop:
        norm_matrix = [0.5,0.5,0.5,2.4,0.9,0.9] #rgbdxy for non-cropped
    else:
        norm_matrix = [0.5,0.5,0.5,2.4,0.9,0.9] #rgbdxy for cropped

    rgbd = rgbd*norm_matrix
    print("rgbd",rgbd)
    return np.float32(rgbd)
    

def postprocess_im(depth_img, segmentation, labelled_img, prop=0.4, thresh=110):
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
    thresh = max(np.min(depth_img), thresh) # set threshold
    labels = np.unique(labelled_img)
    labelled_img = labelled_img.reshape(depth_img.shape)

    labels = np.unique(labelled_img)

    # Bug in labelling
    
    for label in labels:
        coords = np.count_nonzero(labelled_img == label)
        # a pixel violates if it's part of P and depth value is less than thresh
        violated_px = labelled_img[(labelled_img==label) & (depth_img < thresh)]
        if len(violated_px) > prop * coords:
            print ("label", label, "violated")
            # TODO: make it a label of varying length
            print (segmentation.shape)
            segmentation[labelled_img==label] = [255]*segmentation.shape[-1]
        else:
            print("label", label, "not violated")
            print(segmentation.shape)
            segmentation[labelled_img==label] = np.random.randint(0,255)*segmentation.shape[-1]

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

def cv_kmeans(input_img, img_shape, crop_add, debug=False):
    """ 
    Performs kmeans clustering on the input matrix. Expect the input_matrix 
    to have dimension (k, d, c) where k and d are the image dimension and c is 
    the number of channels in the image. img_shape is the desired image shape
    """
    
    c = input_img.shape[-1]

    twoDimg = np.float32(input_img.reshape((-1,c)))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    
    distortions = []
    labels=[]
    K_max=6
    for k in range(1,K_max):
        attempts = 10
        ret, label, center = cv2.kmeans(twoDimg, k, None, criteria, attempts, 
        cv2.KMEANS_PP_CENTERS)
        labels.append(label+crop_add)
        center = np.uint8(center)
        distortions.append(ret)

    
    idx = 0
    for i in range(len(distortions)-1):
        if (distortions[i] - distortions[i+1] < (distortions[0]/10)):
            idx = i
            break
    
    res = center[labels[idx-2].flatten()]
    res = res[:,:3]
    result_image = res.reshape((img_shape))

    if debug:
        plt.plot(range(1,K_max), distortions, 'bx-')
        plt.xlabel('k')
        plt.ylabel('Distortion')
        plt.title('The Elbow Method showing the optimal k')
        plt.show()
        cv2.imshow("res", result_image)
        cv2.waitKey()

    
    return result_image, labels[idx]

def stitch(color_img, depth_img, bounding):
    '''
    Given an original color image and depth image, crops both images based on bounding boxes of object. Performs kmeans segmentation on original depth image and cropped depth image. Stitches cropped depth image segmentation on top of original depth image segmentation. Combines labels of both segmentations. Returns one final stitched kmeans segmentation, final labels, and the object label.
    '''
    depth_img = normalize(depth_img)
    print(depth_img)
    print(np.min(depth_img))
    print(np.max(depth_img))
    crop_color_img, crop_depth_img, top, bottom, left, right = crop_images(color_img, depth_img, bounding)
    cv2.imshow("Color Image", color_img)
    cv2.imshow("Depth Image", depth_img)
    cv2.imshow("Cropped Color Image", crop_color_img)
    cv2.imshow("Cropped Depth Image", crop_depth_img)
    #cv2.waitKey()

    result_img, labels = segmentation(color_img, depth_img, 0)
    crop_result_img, crop_labels = segmentation(crop_color_img, crop_depth_img, (np.max(labels)+1))
    object_label = labels
    result_img, labels, object_label = superimpose(result_img, crop_result_img, labels, crop_labels, top, bottom, left, right)
    print('step8')
    cv2.imshow("Result Image", result_img)
    print('step9')
    cv2.waitKey()
    
    return result_img, labels, object_label
    #return crop_result_img, labels, object_label

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

def segmentation(color_img, depth_img, crop_add):
    rgbd = create_rgbd(color_img, depth_img)
    preprocessed_rgbd = preprocess_data(depth_img,rgbd, False)
    result_img, labels = cv_kmeans(preprocessed_rgbd, color_img.shape, crop_add)
    result_img = postprocess_im(depth_img, result_img, labels)
    return result_img, labels

def superimpose(result_img, crop_result_img, labels, crop_labels, top, bottom, left, right): 
    print('SUPERIMPOSINGGGGGGGGGGGGGGGGGGGGG\n\n\n\n\n\n')
    result_img[top:bottom, left:right] = crop_result_img
    print('step1')
    print(crop_labels)
    print(labels)
    print(np.min(labels))
    print(np.max(labels))
    print(np.min(crop_labels))
    print(np.max(crop_labels))
    print(len(labels))
    print(len(crop_labels))
    #for x in crop_labels:
    #    x[0] += np.max(labels)
    print('step2')
    labels = labels.reshape((result_img.shape[0], result_img.shape[1], 1))
    print('step3')
    crop_labels = crop_labels.reshape((crop_result_img.shape[0], crop_result_img.shape[1], 1))
    print('step4')
    object_label = np.max(crop_labels)
    print('step5')
    labels[top:bottom, left:right] = crop_labels
    print('step6')
    labels = labels.reshape((result_img.shape[0]*result_img.shape[1], 1))
    print('step7')

    return result_img, labels, object_label

def get_image_bounds(color_img, depth_img, bounding, debug=False):
    '''
    cv2.imshow('color_img', color_img)
    cv2.imshow('depth_img', depth_img)
    cv2.waitKey()
    rgbd = create_rgbd(color_img, depth_img)
    preprocessed_rgbd = preprocess_data(depth_img,rgbd)
    print ("max of depth image",np.max(depth_img))
    result_img, labels = cv_kmeans(preprocessed_rgbd, color_img.shape)
    result_img = postprocess_im(depth_img, result_img, labels)
    '''
    result_img, labels, object_label = stitch(color_img, depth_img, bounding)
    
    
    return bound(result_img, True)

def crop_images(color_img, depth_img, bounding):
    return color_img[bounding[0]:bounding[1], bounding[2]:bounding[3]], depth_img[bounding[0]:bounding[1], bounding[2]:bounding[3]], bounding[0], bounding[1], bounding[2], bounding[3]

def normalize(depth_array):
    scaled_array = (depth_array/np.float(np.max(depth_array)))*255
    scaled_array = scaled_array.astype(int)
    scaled_array = scaled_array.astype(np.uint8)
    return scaled_array

def main():
    color_img, depth_img = get_depth_images("04-12-13:10:36")
    #org_img, depth_img = get_depth_images("04-12-14:44:58")
    #org_img, depth_img = get_depth_images("04-12-14:49:33")
    #color_img, depth_img = get_depth_images("04-12-15:25:03")

    result_img, labels, object_label = stitch(color_img, depth_img)
    print(object_label)

    # GET IMAGE BOUNDS OF STITCHED

if __name__ == "__main__":
    main()

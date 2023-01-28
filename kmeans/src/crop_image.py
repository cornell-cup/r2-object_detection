import matplotlib.pyplot as plt
import cv2
import os
from matplotlib import image

def get_depth_images(dir):
    path = os.path.join(os.getcwd(), "kmeans_test_imgs", dir)
    org_img = image.imread(path+"/Orignal.jpg")
    RGB_img = cv2.cvtColor(org_img, cv2.COLOR_BGR2RGB)
    depth_img = cv2.imread(path+"/Depth Frame.jpg",0)
    return RGB_img, depth_img

def crop_image(img):
    return img[148:478, 58:639] # cropping with format - [top:bottom, left:right]
    #return img[140:480, 0:495] # cropping with format - [top:bottom, left:right]
    #return img[140:480, 0:495] # cropping with format - [top:bottom, left:right]
    #return img[140:480, 0:495] # cropping with format - [top:bottom, left:right]
    #return img[140:480, 0:495] # cropping with format - [top:bottom, left:right]
    #return img[140:480, 0:495] # cropping with format - [top:bottom, left:right]
    #return img[140:480, 0:495] # cropping with format - [top:bottom, left:right]
    #return img[140:480, 0:495] # cropping with format - [top:bottom, left:right]
    #return img[140:480, 0:495] # cropping with format - [top:bottom, left:right]
    #return img[140:480, 0:495] # cropping with format - [top:bottom, left:right]

def main():
    org_img, depth_img = get_depth_images("04-12-13:10:36")
    #org_img, depth_img = get_depth_images("04-12-14:44:58")
    #org_img, depth_img = get_depth_images("04-12-14:48:07")
    #org_img, depth_img = get_depth_images("04-12-14:49:33")
    #org_img, depth_img = get_depth_images("04-12-14:51:37")
    #org_img, depth_img = get_depth_images("04-12-15:25:03")
    #org_img, depth_img = get_depth_images("04-12-15:45:10")
    #org_img, depth_img = get_depth_images("04-12-15:49:55")
    #org_img, depth_img = get_depth_images("12-02-12:41:32")
    #org_img, depth_img = get_depth_images("23-04-14:42:35")
    #org_img, depth_img = get_depth_images("30-04-15:06:00")

    cropped_org_img = crop_image(org_img)
    cropped_depth_img = crop_image(depth_img)

    cv2.imshow("org_img", org_img)
    cv2.imshow("depth_img", depth_img)
    cv2.imshow("cropped_org_img", cropped_org_img)
    cv2.imshow("cropped_depth_img", cropped_depth_img)

    cv2.waitKey()

if __name__ == "__main__":
    main()

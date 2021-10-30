#general imports
from grasp_inf import inference as infer
import numpy as np
import tensorflow as tf
import cv2
import os

# points calc imports
import math
import numpy.matlib as npm
#visualize grip points
from matplotlib import pyplot as plt
tf.compat.v1.disable_v2_behavior()
tf.compat.v1.disable_eager_execution()
#define flags for grip point detection
tf.compat.v1.app.flags.DEFINE_integer('image_size', 224,
                            """Provide square images of this size.""")
tf.compat.v1.app.flags.DEFINE_integer('num_preprocess_threads', 12,
                            """Number of preprocessing threads per tower. """
                            """Please make this a multiple of 4.""")
tf.compat.v1.app.flags.DEFINE_integer('num_readers', 12,
                            """Number of parallel readers during train.""")
tf.compat.v1.app.flags.DEFINE_integer('input_queue_memory_factor', 12,
                            """Size of the queue of preprocessed images. """
                            """Default is ideal but try smaller values, e.g. """
                            """4, 2 or 1, if host memory is constrained. See """
                            """comments in code for more details.""")
FLAGS = tf.compat.v1.app.flags.FLAGS
height = FLAGS.image_size
width = FLAGS.image_size


def conv2d_s2(x, W):
    return tf.nn.conv2d(input=x, filters=W, strides=[1,2,2,1], padding='SAME')

def conv2d_s1(x, W):
    return tf.nn.conv2d(input=x, filters=W, strides=[1,1,1,1], padding='SAME')
    
def max_pool_2x2(x):
    return tf.nn.max_pool2d(input=x, ksize=[1,2,2,1], strides=[1,2,2,1], padding='SAME')

def grasp_to_bbox(coord):
    x,y, tan, h, w = coord[0],coord[1],coord[2],coord[3],coord[4]
    theta = math.atan(tan)
    edge1 = (x -w/2*np.cos(theta) +h/2*np.sin(theta), y -w/2*np.sin(theta) -h/2*np.cos(theta))
    edge2 = (x +w/2*np.cos(theta) +h/2*np.sin(theta), y +w/2*np.sin(theta) -h/2*np.cos(theta))
    edge3 = (x +w/2*np.cos(theta) -h/2*np.sin(theta), y +w/2*np.sin(theta) +h/2*np.cos(theta))
    edge4 = (x -w/2*np.cos(theta) -h/2*np.sin(theta), y -w/2*np.sin(theta) +h/2*np.cos(theta))
    edge1=(int(edge1[0]),int(edge1[1]))
    edge2= (int(edge2[0]),int(edge2[1]))
    edge3= (int(edge3[0]),int(edge3[1]))
    edge4= (int(edge4[0]),int(edge4[1]))
    
    
    print('edges: ',[edge1, edge2, edge3, edge4])
    return [edge1, edge2, edge3, edge4]

def convertRectangle2(coord):
    cx_, cy_, a_, h_, w_ = coord[0],coord[1],coord[2],coord[3],coord[4]
    theta = math.radians(a_)
    bbox = npm.repmat([[cx_], [cy_]], 1, 5) + \
    np.matmul([[math.cos(theta), math.sin(theta)],
              [-math.sin(theta), math.cos(theta)]],
             [[-w_ / 2, w_/ 2, w_ / 2, -w_ / 2, w_ / 2 + 8],
              [-h_ / 2, -h_ / 2, h_ / 2, h_ / 2, 0]])
    # add first point
    x1, y1 = bbox[0][0], bbox[1][0]
    # add second point
    x2, y2 = bbox[0][1], bbox[1][1]
    # add third point
    #x3, y3 = bbox[0][4], bbox[1][4]
    # add forth point
    x3, y3 = bbox[0][2], bbox[1][2]
    # add fifth point
    x4, y4 = bbox[0][3], bbox[1][3]

    return list(map(int,[x1, y1, x2, y2, x3, y3, x4, y4]))

def convertRectangle(coord):
    w=float(width)
    h=float(height)
    cx_, cy_, a_, h_, w_ = coord[0]*w+w/2,coord[1]*h+h/2,coord[2],coord[3]*h,coord[4]*w
    theta = a_
    print( cx_, cy_, a_, h_, w_)
    bbox = npm.repmat([[cx_], [cy_]], 1, 4) + \
       np.matmul([[math.cos(theta), math.sin(theta)],
                  [-math.sin(theta), math.cos(theta)]],
                 [[-w_ / 2, w_/ 2, w_ / 2, -w_ / 2, w_ / 2],
                  [-h_ / 2, -h_ / 2, h_ / 2, h_ / 2]])
    # add first point
    x1, y1 = bbox[0][0], bbox[1][0]
    # add second point
    x2, y2 = bbox[0][1], bbox[1][1]
    # add third point
    #x3, y3 = bbox[0][4], bbox[1][4]
    # add forth point
    x3, y3 = bbox[0][2], bbox[1][2]
    # add fifth point
    x4, y4 = bbox[0][3], bbox[1][3]

    return list(map(lambda x:int(x-width/2),[x1, y1, x2, y2, x3, y3, x4, y4]))

def init():
    image_placeholder = tf.compat.v1.placeholder(tf.float32,name="image", shape=(1, height,width,3))
    
    keep_prob = 1.

    w1 = tf.compat.v1.get_variable('w1', shape=[5,5,3,64], trainable=FLAGS.trainable)
    b1 = tf.compat.v1.get_variable('b1', initializer=tf.constant(0.1, shape=[64]), trainable=FLAGS.trainable)
    h1 = tf.nn.relu(conv2d_s2(image_placeholder, w1)+b1)
    print(h1.shape)
    h1_pool = max_pool_2x2(h1)
    print(h1_pool.shape)
    w2 = tf.compat.v1.get_variable('w2', [3,3,64,128], trainable=FLAGS.trainable)
    b2 = tf.compat.v1.get_variable('b2', initializer=tf.constant(0.1, shape=[128]), trainable=FLAGS.trainable)
    h2 = tf.nn.relu(conv2d_s2(h1_pool,w2)+b2)
    h2_pool = max_pool_2x2(h2)

    w3 = tf.compat.v1.get_variable('w3', [3,3,128,128], trainable=FLAGS.trainable)
    b3 = tf.compat.v1.get_variable('b3', initializer=tf.constant(0.1, shape=[128]), trainable=FLAGS.trainable)
    h3 = tf.nn.relu(conv2d_s1(h2_pool,w3)+b3)
    
    w4 = tf.compat.v1.get_variable('w4', [3,3,128,128], trainable=FLAGS.trainable)
    b4 = tf.compat.v1.get_variable('b4', initializer=tf.constant(0.1, shape=[128]), trainable=FLAGS.trainable)
    h4 = tf.nn.relu(conv2d_s1(h3,w4)+b4)

    w5 = tf.compat.v1.get_variable('w5', [3,3,128,256], trainable=FLAGS.trainable)
    b5 = tf.compat.v1.get_variable('b5', initializer=tf.constant(0.1, shape=[256]), trainable=FLAGS.trainable)
    h5 = tf.nn.relu(conv2d_s1(h4,w5)+b5)
    h5_pool = max_pool_2x2(h5)
    print("h5_pool shape")
    print(h5_pool.shape)
    
    w_fc1 = tf.compat.v1.get_variable('w_fc1', [7*7*256,512], trainable=FLAGS.trainable)
    b_fc1 = tf.compat.v1.get_variable('b_fc1', initializer=tf.constant(0.1, shape=[512]), trainable=FLAGS.trainable)
    h5_flat = tf.reshape(h5_pool, [-1, 7*7*256])
    h_fc1 = tf.nn.relu(tf.matmul(h5_flat,w_fc1)+b_fc1)
    #h_fc1_dropout = tf.nn.dropout(h_fc1, 1 - (1 - (keep_prob)))
    h_fc1_dropout = h_fc1
    w_fc2 = tf.compat.v1.get_variable('w_fc2', [512,512], trainable=FLAGS.trainable)
    b_fc2 = tf.compat.v1.get_variable('b_fc2', initializer=tf.constant(0.1, shape=[512]), trainable=FLAGS.trainable)
    h_fc2 = tf.nn.relu(tf.matmul(h_fc1_dropout, w_fc2)+b_fc2)
    #h_fc2_dropout = tf.nn.dropout(h_fc2, 1 - (1 - (keep_prob)))
    h_fc2_dropout = h_fc2
    
    w_output = tf.compat.v1.get_variable('w_output', [512, 5], trainable=FLAGS.trainable)
    b_output = tf.compat.v1.get_variable('b_output', initializer=tf.constant(0.1, shape=[5]), trainable=FLAGS.trainable)
    output = tf.matmul(h_fc2_dropout, w_output)+b_output
    return output,image_placeholder



def edgedetection(imagePath):
    global sess,output,image_placeholder
    startfrac=0.4
    endfrac=0.6
    fillcolor=255
    img = cv2.imread(imagePath)
    img = cv2.resize(img, (int(endfrac*width)-int(startfrac*width),int(endfrac*height)-int(startfrac*height)))
    new_image=np.zeros((width,height,3))
    new_image.fill(fillcolor)
    w=width
    h=height

    #from edgedetection import edgedetection\nedgedetection('images_edited/black.jpg')
    new_image[int(startfrac*w):int(endfrac*w),int(startfrac*h):int(endfrac*h),:]=img
    #print(new_image[int(0.2*w):int(0.8*w),int(0.2*h):int(0.8*h),:])
    #cv2.imshow('img', new_image/255)
    #cv2.waitKey(0)
    cv2.imwrite('save.jpg',new_image)
    img = cv2.imread('save.jpg')
    image = tf.image.convert_image_dtype(img, dtype=tf.float32)
    image = tf.image.resize(image, [height,width])
    image = tf.expand_dims(image, 0)
    npoutput=sess.run(output,feed_dict={image_placeholder:image.eval(session=sess)}).flatten()
    edges = grasp_to_bbox(npoutput)
    display_img = cv2.resize(img,(height,width))
    for i in range(0,4):
        if i==3:
            cv2.line(display_img,edges[3],edges[0],(255,0,0),thickness=3)
        else:
            cv2.line(display_img,edges[i],edges[i+1],(255,0,0),thickness=3)

    cv2.imshow('img', display_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.waitKey(0)

    return

#testing params
modelPath = "models/grasp/m4/m4.ckpt"
#imagePath = "images"
output,image_placeholder=init()

#dg={}
#lg = ['w1', 'b1', 'w2', 'b2', 'w3', 'b3', 'w4', 'b4', 'w5', 'b5', 'w_fc1', 'b_fc1', 'w_fc2', 'b_fc2', 'w_output', 'b_output']
#for i in lg:
#    dg[i] = [v for v in tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES) if v.name == i+':0'][0]
#new_saver = tf.train.import_meta_graph('models/grasp/m4/m4.ckpt.meta')
dg={}
lg = ['w1', 'b1', 'w2', 'b2', 'w3', 'b3', 'w4', 'b4', 'w5', 'b5', 'w_fc1', 'b_fc1', 'w_fc2', 'b_fc2', 'w_output', 'b_output']
for i in lg:
    dg[i] = [v for v in tf.compat.v1.get_collection(tf.compat.v1.GraphKeys.GLOBAL_VARIABLES) if v.name == i+':0'][0]
sess = tf.compat.v1.Session()
#sess.run(tf.global_variables_initializer())
#new_saver = tf.train.Saver(dg)
new_saver = tf.compat.v1.train.Saver(dg)
new_saver.restore(sess, 'models/grasp/m4/m4.ckpt')

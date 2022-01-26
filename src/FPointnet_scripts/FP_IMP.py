from __future__ import print_function

''' Evaluating Frustum PointNets.
Write evaluation results to KITTI format labels.
and [optionally] write results to pickle files.

Author: Charles R. Qi
Date: September 2017
'''

'''
----------------------------------------------------------
    @file: FP_IMP.py
    @date: Aug 2021
    @date_modif: Fri Aug 21, 2021
    @author: Raul David Dominguez Sanchez
    @e-mail: rauldavidds@hotmail.com
    @brief: Set of functions to initialize F-PointNet and carry out a prediction.
            Script based on the work from Charles R. Qi
    @Note: Be careful, do not forget to modify the paths accordingly.
----------------------------------------------------------
'''

import importlib
import numpy as np
import tensorflow as tf
#from model_util import NUM_HEADING_BIN, NUM_SIZE_CLUSTER

# Set configurations
NUM_HEADING_BIN = 12
NUM_SIZE_CLUSTER = 8 # one cluster for each type
BATCH_SIZE = 32
MODEL_PATH = '/home/rauldds/catkin_ws/src/vttc/src/log_v1/model.ckpt'
GPU_INDEX = 0
NUM_POINT = 1024
MODEL = importlib.import_module('frustum_pointnets_v1')
NUM_CLASSES = 2
NUM_CHANNEL = 4

g_type2class={'Buouy':0, 'Marker':1, 'MarkerDos':2}

g_class2type = {g_type2class[t]:t for t in g_type2class}

g_type_mean_size = {'Buouy': np.array([0.4,0.4,0.4]),
                    'Marker': np.array([0.6,0.6,1.7]),
                    'MarkerDos': np.array([2.13586957,2.58549199,2.2520595])}

def class2size(pred_cls, residual):
    '''
        @name: claas2size
        @brief: function to obtain the dimensions of the bounding box.
                F-PointNet outputs residual values for the dimensions of a bounding box,
                these residuals represent the difference with respect to the average size of the object. 
                Therefore, to obtain the dimension of the box, the average size and the residuals should
                be added.
        @param: pred_cls: class predicted by F-PointNet
                residual: residuals predicted by F-PointNet
        @return: the dimensions of the 3D bounding box
    '''
    mean_size = g_type_mean_size[g_class2type[pred_cls]]
    return mean_size + residual

def class2angle(pred_cls, residual, num_class, to_label_format=True):
    angle_per_class = 2*np.pi/float(num_class)
    angle_center = pred_cls * angle_per_class
    angle = angle_center + residual
    if to_label_format and angle>np.pi:
        angle = angle - 2*np.pi
    return angle

def get_session_and_ops(batch_size, num_point):
    ''' 
    @name: get_session_and_ops
        @brief: function to initialize F-PointNet
        @param: pred_cls: the size of the batch that will be given to F-PointNet.
                          By default its value is 32
                residual: the number of points that the cloud given to F-PointNet should have.
                          By default 1024.
        @return: TensorFlow session (sess) and operations (ops)
    '''
    with tf.Graph().as_default():
        with tf.device('/gpu:'+str(GPU_INDEX)):
            pointclouds_pl, one_hot_vec_pl, labels_pl, centers_pl, \
            heading_class_label_pl, heading_residual_label_pl, \
            size_class_label_pl, size_residual_label_pl = \
                MODEL.placeholder_inputs(batch_size, num_point)
            is_training_pl = tf.placeholder(tf.bool, shape=())
            end_points = MODEL.get_model(pointclouds_pl, one_hot_vec_pl,
                is_training_pl)
            loss = MODEL.get_loss(labels_pl, centers_pl,
                heading_class_label_pl, heading_residual_label_pl,
                size_class_label_pl, size_residual_label_pl, end_points)
            saver = tf.train.Saver()

        # Create a session
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        sess = tf.Session(config=config)

        # Restore variables from disk.
        saver.restore(sess, MODEL_PATH)
        ops = {'pointclouds_pl': pointclouds_pl,
               'one_hot_vec_pl': one_hot_vec_pl,
               'labels_pl': labels_pl,
               'centers_pl': centers_pl,
               'heading_class_label_pl': heading_class_label_pl,
               'heading_residual_label_pl': heading_residual_label_pl,
               'size_class_label_pl': size_class_label_pl,
               'size_residual_label_pl': size_residual_label_pl,
               'is_training_pl': is_training_pl,
               'logits': end_points['mask_logits'],
               'center': end_points['center'],
               'end_points': end_points,
               'loss': loss}
        return sess, ops

def softmax(x):
    ''' Numpy function for softmax'''
    shape = x.shape
    probs = np.exp(x - np.max(x, axis=len(shape)-1, keepdims=True))
    probs /= np.sum(probs, axis=len(shape)-1, keepdims=True)
    return probs

def inference(sess, ops, pc, one_hot_vec, batch_size):
    ''' 
    @name: inference
        @brief: function to execute a F-PointNet inference
        @param: sess: TensorFlow session
                ops: TensorFlow operations
                pc: 32 Frustums that comprise 1024 points and 4 channels (32,1024,4)
                one_hot_vec: Classes detected by the 2D detector in one hot format
                batch_size: Size of the batch (by default 32)
        @return: batch_output, batch_center_pred, batch_hclass_pred, 
                 batch_hres_pred, batch_sclass_pred, batch_sres_pred, batch_scores
    '''
    assert pc.shape[0]%batch_size == 0
    num_batches = pc.shape[0]/batch_size
    logits = np.zeros((pc.shape[0], pc.shape[1], NUM_CLASSES))
    centers = np.zeros((pc.shape[0], 3))
    heading_logits = np.zeros((pc.shape[0], NUM_HEADING_BIN))
    heading_residuals = np.zeros((pc.shape[0], NUM_HEADING_BIN))
    size_logits = np.zeros((pc.shape[0], NUM_SIZE_CLUSTER))
    size_residuals = np.zeros((pc.shape[0], NUM_SIZE_CLUSTER, 3))
    scores = np.zeros((pc.shape[0],)) # 3D box score 
   
    ep = ops['end_points'] 
    for i in range(num_batches):
        feed_dict = {\
            ops['pointclouds_pl']: pc[i*batch_size:(i+1)*batch_size,...],
            ops['one_hot_vec_pl']: one_hot_vec[i*batch_size:(i+1)*batch_size,:],
            ops['is_training_pl']: False}

        batch_logits, batch_centers, \
        batch_heading_scores, batch_heading_residuals, \
        batch_size_scores, batch_size_residuals = \
            sess.run([ops['logits'], ops['center'],
                ep['heading_scores'], ep['heading_residuals'],
                ep['size_scores'], ep['size_residuals']],
                feed_dict=feed_dict)

        logits[i*batch_size:(i+1)*batch_size,...] = batch_logits
        centers[i*batch_size:(i+1)*batch_size,...] = batch_centers
        heading_logits[i*batch_size:(i+1)*batch_size,...] = batch_heading_scores
        heading_residuals[i*batch_size:(i+1)*batch_size,...] = batch_heading_residuals
        size_logits[i*batch_size:(i+1)*batch_size,...] = batch_size_scores
        size_residuals[i*batch_size:(i+1)*batch_size,...] = batch_size_residuals

        # Compute scores
        batch_seg_prob = softmax(batch_logits)[:,:,1] # BxN
        batch_seg_mask = np.argmax(batch_logits, 2) # BxN
        mask_mean_prob = np.sum(batch_seg_prob * batch_seg_mask, 1) # B,
        mask_mean_prob = mask_mean_prob / np.sum(batch_seg_mask,1) # B,
        heading_prob = np.max(softmax(batch_heading_scores),1) # B
        size_prob = np.max(softmax(batch_size_scores),1) # B,
        batch_scores = np.log(mask_mean_prob) + np.log(heading_prob) + np.log(size_prob)
        scores[i*batch_size:(i+1)*batch_size] = batch_scores 
        # Finished computing scores

    heading_cls = np.argmax(heading_logits, 1) # B
    size_cls = np.argmax(size_logits, 1) # B
    heading_res = np.array([heading_residuals[i,heading_cls[i]] \
        for i in range(pc.shape[0])])
    size_res = np.vstack([size_residuals[i,size_cls[i],:] \
        for i in range(pc.shape[0])])

    return np.argmax(logits, 2), centers, heading_cls, heading_res, \
        size_cls, size_res, scores

def F_PointNet_Execution(sess, ops, PCB,OHOT):
    ''' 
    @name: F_PointNet_Execution
        @brief: function to call the inference function, give the proper format to 
                the ouputs provided by F-PointNet
        @param: sess: TensorFlow session
                ops: TensorFlow operations
                PCB: 32 Frustums that comprise 1024 points and 4 channels (32,1024,4)
                OHOT: Classes detected by the 2D detector in one hot format
                batch_size: Size of the batch (by default 32)
        @return: predicted class and the center, dimension, and rotation of the 3D bounding.
    '''
    batch_size = 32
    #batch_data_to_feed = np.zeros((32,1024,4),dtype=np.float32)
    batch_data_to_feed = PCB #32X1024X4 --> 32 NUBES DE PUNTOS DE 1024 PUNTOS CON 3 CANALES 
    #print('batch: '+str(batch_data_to_feed.shape))
    batch_one_hot_to_feed = OHOT # 1X3 --> 32 NUBES EN FORMATO ONE HOT

    #sess, ops = get_session_and_ops(batch_size=batch_size, num_point=NUM_POINT)
    
    #run one batch inference
    batch_output, batch_center_pred, \
        batch_hclass_pred, batch_hres_pred, \
        batch_sclass_pred, batch_sres_pred, batch_scores = \
            inference(sess, ops, batch_data_to_feed,
                batch_one_hot_to_feed, batch_size=batch_size)
    #print('class pred: '+str(batch_output[0]))
    #print('prdiction center: '+str(batch_center_pred[0]))
    #MODIFICAR cuando este lo de jorge
    rotation = []
    dimensions = []
    for i in range(len(batch_center_pred)):
        rotation.append(class2angle(batch_hclass_pred[i], batch_hres_pred[i], NUM_HEADING_BIN))
        dimensions.append(class2size(batch_sclass_pred[i],batch_sres_pred[i]))
    #print('prediction heading: '+str(rotation))
    #print('predicition dimensions:'+str(dimensions))
    return batch_sclass_pred,batch_center_pred,dimensions,rotation
	


'''if __name__=='__main__':
    pc_batch =[]
    pc_ori = np.zeros((NUM_POINT, NUM_CHANNEL))
    indices = np.arange(0, len(pc_ori))
    if len(pc_ori) > 1024:
            choice = np.random.choice(indices, size=1024, replace=True)
    else:
            choice = np.random.choice(indices, size=1024, replace=False)

    point_cloud_ds = pc_ori[choice]
    pc_batch.extend(point_cloud_ds,point_cloud_ds)
    print(pc_batch)
    test_from_rgb_detection(sess, ops, point_cloud_ds)'''
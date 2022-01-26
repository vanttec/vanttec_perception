#!/usr/bin/env python
'''
----------------------------------------------------------
    @file: FrustumNode.py
    @date: Aug 2021
    @date_modif: Fri Aug 21, 2021
    @author: Raul David Dominguez Sanchez
    @e-mail: rauldavidds@hotmail.com
    @brief: ROS Node to implement F-PointNet, the node listens to a point clouds topic, converts them to Frustums, 
    provides the Frustums to F-Pointnet, in the end the node publishes Markers.
    @Note: Be careful, remember to have this script and the script with the F-Pointnet functions in the same folder.
           Furthermore, do not forget to modify the paths accordingly.
----------------------------------------------------------
'''

import os
from kitti_object import *
import kitti_util as utils
import cv2
import rospy
import pypcd
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
import numpy as np
from FP_IMP import F_PointNet_Execution, get_session_and_ops
from visualization_msgs.msg import Marker, MarkerArray
from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list


sess, ops = get_session_and_ops(32, 1024)
mrkr_pub=rospy.Publisher("markers_publisher",MarkerArray,queue_size=10)

box_color = {'0': ColorRGBA(255.0, 0.0, 0.0, 0.5),
               '1': ColorRGBA(0.0, 255.0, 0.0, 0.5),
               '2': ColorRGBA(0.0, 0.0, 255.0, 0.5)}

cv_image = np.zeros((560,1000,3),np.uint8)
BB = [[0, 0, 0, 0]]
lngth = 1

def frustums_gen(pointcloud,arr_4):
    global BB,lngth
    '''
        @name: frustums_gen
        @brief: function to generate a frustum based on a bounding box received from a 2D detector and 
                provide this frustum to F-PointNet
        @param: pointcloud: 3 channel point cloud
                arr_c4: 4 channel point cloud
        @return: predicted class, as well as the center, dimensions, and heading of the 3D bounding box
    '''
    #BORRAR CUANDO YAS SE TENGA LO DE JORGE
    dataset = kitti_object(os.path.join('/home/rauldds/Desktop/VantTec/frustum-pointnets', 'dataset/KITTI/object'))
    data_idx = 9
    pc_batch =[]
    pc_batch2 =[]
    oh_batch = []

    # Load data from dataset
    # cambiar objects por nodo jorge
    objects = dataset.get_label_objects(data_idx)
    #objects[0].print_object() 
    pc_velo = pointcloud 
    #calib = dataset.get_calibration(data_idx)
    calib = utils.Calibration('/home/rauldds/catkin_ws/src/vttc/src/calib.txt')
    #print(calib)
    
    # Only display those points that fall into 2d box
    #print(' -------- LiDAR points in a frustum from a 2D box --------')
    print("length: "+str(lngth))
    for i in range (lngth):
        xmin,ymin,xmax,ymax = BB[i]
        boxfov_pc_velo = get_lidar_in_image_fov(pc_velo, calib, xmin, ymin, xmax, ymax)
        result=[]
        for j in range(boxfov_pc_velo.shape[0]):
            result.append(np.where(pointcloud == boxfov_pc_velo[j][0])[0][0]) 
        #arr_c3 = arr_c3[result]
        arr_c4 = arr_4[result]
        print("hola: "+str(arr_c4.shape))
        indices = np.arange(0, len(arr_c4))
        pc_ori = arr_c4
        print("nube de puntos ="+str(i))
        if len(pc_ori) > 1024:
            choice = np.random.choice(indices, size=1024, replace=True)
            point_cloud_ds = np.array(pc_ori[choice])
        #elif len(pc_ori)<20:
        #    continue
            #return None,None,None,None
        elif 14<len(pc_ori)<1024:
            #se podria acelerar con otro metodo en vez de for
            for i in range(1024-len(pc_ori)):
                pc_ori = np.append(pc_ori, [arr_c4[np.random.randint(arr_c4.shape[0])]], axis=0)
            point_cloud_ds = pc_ori
        elif len(pc_ori)<15:
            point_cloud_ds = np.zeros((1,1024,4))
        else:
                choice = np.random.choice(indices, size=1024, replace=False)
                point_cloud_ds = np.array(pc_ori[choice])
        if len(pc_ori)>=15:
            pc_batch2.append(point_cloud_ds)
    if len(pc_batch2) == 0:
        return None,None,None,None
    while (len(pc_batch2)<32):
        pc_batch2.extend(pc_batch2)
    pc_batch2 = pc_batch2[0:32]
    pc_batch2 = np.asarray(pc_batch2, dtype=np.float32)
    print("pc batch2: "+str(pc_batch2.shape))
    

    '''xmin,ymin,xmax,ymax = BB[0]
    boxfov_pc_velo = \
        get_lidar_in_image_fov(pc_velo, calib, xmin, ymin, xmax, ymax)
    #print(('2d box FOV point num: ', boxfov_pc_velo.shape))
    #podria ser acelerado si se usa otro metodo en vez de for
    for i in range(boxfov_pc_velo.shape[0]):
        result.append(np.where(pointcloud == boxfov_pc_velo[i][0])[0][0]) 
    #arr_c3 = arr_c3[result]
    arr_c4 = arr_c4[result]
    print("hola: "+str(arr_c4.shape))
    indices = np.arange(0, len(arr_c4))
    pc_ori = arr_c4
    if len(pc_ori) > 1024:
            choice = np.random.choice(indices, size=1024, replace=True)
            point_cloud_ds = np.array(pc_ori[choice])
    elif len(pc_ori)<20:
        return None,None,None,None
    elif len(pc_ori)<1024:
        #se podria acelerar con otro metodo en vez de for
        for i in range(1024-len(pc_ori)):
            pc_ori = np.append(pc_ori, [arr_c4[np.random.randint(arr_c4.shape[0])]], axis=0)
        point_cloud_ds = pc_ori
    else:
            choice = np.random.choice(indices, size=1024, replace=False)
            point_cloud_ds = np.array(pc_ori[choice])
    pc_batch.extend([point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,
                 point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,
                 point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,
                 point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,
                 point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds])
    pc_batch = np.asarray(pc_batch, dtype=np.float32)
    print("pc batch: "+str(pc_batch.shape))'''
    #print("batch shape:"+str(pc_batch.shape))
    '''CAMBIAR ONE HOTS POR LECTURAS REALES '''
    oh = np.zeros((1, 3))
    oh[0][1]=1
    oh_batch.extend([oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh])
    oh_batch = np.asarray(oh_batch, dtype=np.float32) # 32X3 --> 32 NUBES EN FORMATO ONE HOT
    oh_batch = np.reshape(oh_batch,(32,3))
    pred, center, dim,rot = F_PointNet_Execution(sess,ops,pc_batch2,oh_batch)
    print("dimensions: "+str(len(dim)))
    return pred, center, dim,rot

def MarkerArrayPub(pred,center, dim,original_pc,rot):
    global lngth
    mrkrs = MarkerArray()
    mrkrs.markers = []
    for i in range (lngth):
        mrkrs.markers.append(MarkerPublish(pred[i],center[i], dim[i],original_pc.header.frame_id,rot[0],i))
    mrkr_pub.publish(mrkrs)


def MarkerPublish(pred,center, dim,head_id,rot,obst_num):
    '''
        @name: MarkerPublish
        @brief: function to convert the output from F-PointNet into an obstacle Marker and then publish it
        @param: pred: the class predicted by F-PointNet
                center: The center of the 3D bounding box predicted by F-PointNet
                dim: The dimensions of the 3D bounding box predicted by F-PointNet
                original_pc: The PointCloud2 message retrieved
                rot: The rotation of the 3D bounding box predicted by F-PointNet
        @return: --
    '''

    mrkr = Marker()
    mrkr.header.frame_id = head_id
    mrkr.id = obst_num
    mrkr.ns = "obstacle_"+str(obst_num)
    mrkr.type = 1
    mrkr.action = 0
    mrkr.color = box_color[str(pred)]
    mrkr.scale.x = dim[0]
    mrkr.scale.y = dim[1]
    mrkr.scale.z = dim[2]
    #suma debido a que la caja se pone en el sistema coordenado de la base de bote, cuando son coordenadas respecto al lidar
    mrkr.pose.position.z = center[2]+0.198
    mrkr.pose.position.x = center[0]+0.27
    mrkr.pose.position.y = center[1]+0.4
    mrkr.pose.orientation.x = 0
    mrkr.pose.orientation.y = 0 #probar rot
    mrkr.pose.orientation.z = 0
    mrkr.pose.orientation.w = 1
    mrkr.lifetime = rospy.Duration(1)
    #mrkr_pub.publish(mrkr)
    return mrkr

def PC2CallBack(msg):
    '''
        @name: PC2CallBack
        @brief: Retrieves PointCloud2 messages and rearranges them in the format accepted by F-PointNet,
                it also calls the functions to generate frustums, provide the clouds to F-PointNet and 
                publish the prediction in the form of a Marker
        @param: msg: a PointCloud2 ROS message
        @return: --
    '''

    time = msg.header.stamp
    #print('nube de puntos time: '+str(time.secs))
    rtime =rospy.get_rostime()
    #print('current time:' + str(rtime.secs))
    if (rtime.secs>time.secs):
        return
    elif (rtime.secs==time.secs) and ((rtime.nsecs/1000000)-(time.nsecs/1000000)>500):
        return
    pc = pypcd.PointCloud.from_msg(msg)
    x = pc.pc_data['x']
    y = pc.pc_data['y']
    z = pc.pc_data['z']
    intensity = pc.pc_data['intensity']
    arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
    arr[::4] = x
    arr[1::4] = y
    arr[2::4] = z
    arr[3::4] = intensity
    arr2 = arr.reshape((-1, 4))
    arr_comp_4 = arr2
    arr2 = arr2[:,0:3]
    #print(arr2.shape)
    #rospy.loginfo("Point Cloud Shape: "+str(arr2.shape))
    pred, center, dim,rot = frustums_gen(arr2,arr_comp_4)
    try:
        if (pred==None):
            print("no se publica")
            return
    except:
        pred = pred
    MarkerArrayPub(pred,center,dim, msg,rot)
    #MarkerPublish(pred,center,dim, msg,rot)

def IMGBoxes(msg):
    global cv_image,BB,lngth
    objects = msg.objects
    #print(msg)
    boxes=[]
    objects = msg.objects
    length = msg.len
    if length>0:
        for i in range(length):
            box = [int(objects[i].x*0.64),objects[i].y*48/56-75,(int(objects[i].x*0.64)+int(objects[i].w*0.64)),(objects[i].y*48/56+objects[i].h*48/56)-75]
            for j in box:
                if j<0:
                    box[box.index(j)] =0
            boxes.append(box)
    #print(boxes)
    BB = boxes
    lngth = length

rospy.init_node("F_Node",anonymous=True)
rospy.Subscriber("velodyne_points",PointCloud2,PC2CallBack,queue_size=10)
rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, IMGBoxes,queue_size=10)
rospy.spin()
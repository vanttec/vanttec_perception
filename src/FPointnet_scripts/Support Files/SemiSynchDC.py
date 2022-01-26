#!/usr/bin/env python
import rospy
import pypcd
from sensor_msgs.msg import PointCloud2,Image
import numpy as np
import cv2
from cv_bridge import CvBridgeError, CvBridge

pcsecs=0
pcnsecs=0
imgsecs=0
imgnsecs=0
cntr=80
PC2=0

bridge = CvBridge()

def imgname(x):
    return{
        '1': '00000'+str(cntr),
        '2': '0000'+str(cntr),
        '3': '000'+str(cntr),
        '4': '00'+str(cntr),
        '5': '0'+str(cntr),
        '6': str(cntr)
    }[x]

def PC2CallBack(msg):
    global pcsecs,pcsecs,imgsecs,imgnsecs,PC2
    pcsecs = msg.header.stamp.secs
    pcnsecs = msg.header.stamp.nsecs
    PC2 = msg

def image_callback(msg):
    global pcsecs,pcsecs,imgsecs,imgnsecs,cntr,PC2
    imgsecs = msg.header.stamp.secs
    imgnsecs = msg.header.stamp.nsecs
    if (imgsecs==pcsecs):
        if (abs(imgnsecs-pcnsecs)<100000000):
            cntr=cntr+1
            name = imgname(str(len(str(cntr))))
            #print(cntr)

            pc = pypcd.PointCloud.from_msg(PC2)
            '''x = pc.pc_data['x']
            y = pc.pc_data['y']
            z = pc.pc_data['z']
            intensity = pc.pc_data['intensity']
            arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
            arr[::4] = x
            arr[1::4] = y
            arr[2::4] = z
            arr[3::4] = intensity
            arr2 = arr.reshape((-1, 4))
            arr2 = arr2[:,0:3]
            arr.astype('float32').tofile('/home/rauldds/catkin_ws/src/vttc/Real_Dataset/velodyne/'+name+'.bin')'''
            pc.save('/home/rauldds/catkin_ws/src/vttc/Real_Dataset/velodyne/'+name+'.pcd')
            #print(arr2.shape)
            #rospy.loginfo("current time"+str(rospy.get_rostime()))

            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, e:
                print(e)
            imgpath = '/home/rauldds/catkin_ws/src/vttc/Real_Dataset/image_2/'+name+'.png'
            cv2.imwrite(imgpath, cv2_img)
            print(name)

rospy.init_node("SemiSynched_Node",anonymous=True)
rospy.Subscriber("velodyne_points",PointCloud2,PC2CallBack)
rospy.Subscriber("/zed/zed_node/left_raw/image_raw_color", Image, image_callback)
rospy.spin()
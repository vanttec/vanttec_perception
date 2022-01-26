#!/usr/bin/env python
import rospy
import pypcd
from sensor_msgs.msg import PointCloud2
import numpy as np
import matplotlib.pyplot as plt

def PC2CallBack(msg):
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
    arr2 = arr2[:,0:3]
    #arr.astype('float32').tofile('./src/vttc/dataset/velodyne/testbin.bin')
    print(arr2.shape)
    rospy.loginfo("current time"+str(rospy.get_rostime()))
    rospy.loginfo(msg.header.stamp)

rospy.init_node("bin_generator",anonymous=True)
rospy.Subscriber("velodyne_points",PointCloud2,PC2CallBack)
rospy.spin()
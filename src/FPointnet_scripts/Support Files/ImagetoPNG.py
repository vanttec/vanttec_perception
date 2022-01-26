#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import cv2
import numpy as np

bridge = CvBridge()

T1 = 1.0
T2 = T1

cntr=0
def imgname(x):
    return{
        '1': '00000'+str(cntr),
        '2': '0000'+str(cntr),
        '3': '000'+str(cntr),
        '4': '00'+str(cntr),
        '5': '0'+str(cntr),
        '6': str(cntr)
    }[x]

def image_callback(msg):
    print(msg.header.stamp.nsecs)
    global cntr, T1
    T2 = rospy.get_rostime()
    if (T2.secs-T1.secs)< 1:
        return
    T1 = T2
    print(msg.header.stamp.nsecs)
    cntr=cntr+1
    #print(cntr)
    name = imgname(str(len(str(cntr))))
    #print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a png 
        imgpath = '/home/rauldds/Desktop/VantTec/PruebaROSBAG/'+name+'.png'
        cv2.imwrite(imgpath, cv2_img)
        #print(name)

def main():
    global T1
    rospy.init_node('image_listener')
    T1 = rospy.get_rostime()
    # Define your image topic
    image_topic = "/zed/zed_node/left_raw/image_raw_color"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
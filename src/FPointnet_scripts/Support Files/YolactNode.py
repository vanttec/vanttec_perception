#!/usr/bin/env python
import rospy

from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge =CvBridge()
cv_image = np.zeros((560,1000,3),np.uint8)

def box_callback(msg):
    #print(msg)
    global cv_image
    boxes=[]
    objects = msg.objects
    length = msg.len
    if length>0:
        for i in range(length):
            box = [int(objects[i].x*0.64),objects[i].y*48/56-75,(int(objects[i].x*0.64)+int(objects[i].w*0.64)),(objects[i].y*48/56+objects[i].h*48/56)-75]
            for j in box:
                if j<0:
                    box[box.index(j)] =0
            cv2.rectangle(cv_image,(box[0],box[1]),(box[2],box[3]),(0,0,255),2)
            boxes.append(box)
    print(boxes)
    cv2.imshow("bounding box", cv_image)
    cv2.waitKey(1)
    #cv2.destroyAllWindows()

    ''' 
    TODO: Call Jorge's functions here
    '''
def image_callback(msg):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
    except CvBridgeError as e:
        print(e)

def main():
    rospy.init_node('image_listener')
    list_topic = "/usv_perception/yolo_zed/objects_detected"
    image_topic = "/r200/camera/color/image_raw"
    rospy.Subscriber(list_topic, obj_detected_list, box_callback,queue_size=10)
    rospy.Subscriber(image_topic, Image, image_callback,queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
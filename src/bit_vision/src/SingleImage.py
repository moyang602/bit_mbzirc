#!/usr/bin/env python
#coding=utf-8

import rospy
from bit_hardware_msgs.srv import *
from sensor_msgs.msg import Image
import math
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv

if __name__ == "__main__":
    pub = rospy.Publisher("/CameraMER/SingleImage",Image,queue_size=1)
    rospy.init_node("SingleImage_node",anonymous=False)
    
    rospy.wait_for_service('/CameraMER/GrabMERImage')
    GrabImage = rospy.ServiceProxy('/CameraMER/GrabMERImage',MER_srv)

    rospy.loginfo("Ready to get single image")
    while(not rospy.is_shutdown()):
        try:
            resp = GrabImage(10000, 2.0, 1.6, 2.5)          # 比较好的参数
            pub.publish(resp.MER_image)
            # print(resp.MER_image.encoding) 

        except rospy.ServiceException as e:
            print("Service call failed:",rospy.ServiceException,e)
            # break

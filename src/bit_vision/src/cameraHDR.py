#!/usr/bin/env python3
#coding=utf-8

import rospy
from bit_hardware_msgs.srv import *
from sensor_msgs.msg import Image
import math
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv

def nothing(test):
    pass

if __name__ == "__main__":
    pub1 = rospy.Publisher("/CameraMER/Image",Image,queue_size=1)
    rospy.init_node("Pub",anonymous=False)
    
    rospy.wait_for_service('/CameraMER/GrabMERImage')
    GrabImage1 = rospy.ServiceProxy('/CameraMER/GrabMERImage',MER_srv)
    # rospy.wait_for_service('/CameraMER/GrabHDRImage')
    # GrabImage1 = rospy.ServiceProxy('/CameraMER/GrabHDRImage',MER_hdr)

    while(not rospy.is_shutdown()):
        try:
            respl = GrabImage1(10000, 2.0, 1.6, 2.5)          # 比较好的参数
            # respl = GrabImage1()          # 比较好的参数
            pub1.publish(respl.MER_image)
            print(respl.MER_image.encoding) 

        except rospy.ServiceException as e:
            print("Service call failed:",rospy.ServiceException,e)
            break

#!/usr/bin/env python3
#coding=utf-8

import rospy
from bit_hardware_msgs.srv import *
from sensor_msgs.msg import Image
import math
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv

if __name__ == "__main__":
    pub = rospy.Publisher("/CameraMER/HDRImage",Image,queue_size=1)
    rospy.init_node("HDRImage_node",anonymous=False)
    
    rospy.wait_for_service('/CameraMER/GrabHDRImage')
    GrabImage = rospy.ServiceProxy('/CameraMER/GrabHDRImage',MER_hdr)
    rospy.loginfo("Ready to get HDR image")
    while(not rospy.is_shutdown()):
        try:
            resp = GrabImage()
            pub.publish(resp.MER_image)
        except rospy.ServiceException as e:
            print("Service call failed:",rospy.ServiceException,e)

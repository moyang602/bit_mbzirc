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
    pub = rospy.Publisher("SettingImage",Image,queue_size=1)
    rospy.init_node("CameraSetting_node",anonymous=False)
    
    rospy.wait_for_service('/CameraMER/GrabMERImage')
    GrabImage = rospy.ServiceProxy('/CameraMER/GrabMERImage',MER_srv)

    cv.namedWindow('ImageControl')
    # create trackbars for color change
    cv.createTrackbar('E', 'ImageControl', 10, 50000, nothing)
    cv.createTrackbar('R', 'ImageControl', 50, 500, nothing)
    cv.createTrackbar('G', 'ImageControl', 50, 300, nothing)
    cv.createTrackbar('B', 'ImageControl', 50, 500, nothing)

    while(not rospy.is_shutdown()):
        try:
            cv.waitKey(1)
            exposure_time = cv.getTrackbarPos('E', 'ImageControl')
            r = cv.getTrackbarPos('R', 'ImageControl')
            g = cv.getTrackbarPos('G', 'ImageControl')
            b = cv.getTrackbarPos('B', 'ImageControl')
            resp = GrabImage(exposure_time,r*0.01,g*0.01,b*0.01)
            # resp = GrabImage(exposure_time, 2.0, 1.6, 2.5)          # 比较好的参数
            if resp.success_flag == True:
                pub.publish(resp.MER_image)
                print("Param:", exposure_time, r*0.01, g*0.01, b*0.01) 

        except rospy.ServiceException as e:
            print("Service call failed:",rospy.ServiceException,e)

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
    pub1 = rospy.Publisher("Image1",Image,queue_size=1)
    rospy.init_node("Pub",anonymous=False)
    
    rospy.wait_for_service('/CameraMER_Left/GrabMERImage')
    GrabImage1 = rospy.ServiceProxy('/CameraMER_Left/GrabMERImage',MER_srv)

    cv.namedWindow('image')
    # create trackbars for color change
    cv.createTrackbar('E', 'image', 10, 50000, nothing)
    cv.createTrackbar('R', 'image', 50, 500, nothing)
    cv.createTrackbar('G', 'image', 50, 200, nothing)
    cv.createTrackbar('B', 'image', 50, 500, nothing)

    time = 0
    while(not rospy.is_shutdown()):
        time = time +0.2
        try:
            cv.waitKey(1)
            # respl = GrabImage1(10000*math.sin(time)+10000)
            exposure_time = cv.getTrackbarPos('E', 'image')
            r = cv.getTrackbarPos('R', 'image')
            g = cv.getTrackbarPos('G', 'image')
            b = cv.getTrackbarPos('B', 'image')
            respl = GrabImage1(exposure_time,r*0.01,g*0.01,b*0.01)
            # respl = GrabImage1(exposure_time, 2.0, 1.6, 2.5)          // 比较好的参数

            pub1.publish(respl.MER_image)
            print(respl.MER_image.encoding) 

        except rospy.ServiceException as e:
            print("Service call failed:",rospy.ServiceException,e)

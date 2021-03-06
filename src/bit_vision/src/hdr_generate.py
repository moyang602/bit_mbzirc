#!/usr/bin/env python3
#coding=utf-8
import rospy
from bit_hardware_msgs.srv import *
from sensor_msgs.msg import Image

import math
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge
# from cv_bridge.boost.cv_bridge_boost import CvBridge

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv

def hdr_gen(images,times):

    # 对齐图像
    alignMTB = cv2.createAlignMTB()
    alignMTB.process(images, images)

    # Obtain Camera Response Function (CRF)
    calibrateDebevec = cv2.createCalibrateDebevec()
    responseDebevec = calibrateDebevec.process(images, times)

    # Merge images into an HDR linear image
    mergeDebevec = cv2.createMergeDebevec()
    hdrDebevec = mergeDebevec.process(images, times, responseDebevec)

    ## 使用Reinhard色调映射算法获得24位彩色图像
    tonemapReinhard = cv2.createTonemapReinhard(1.0, 0, 0.1, 0.0)
    ldrReinhard = tonemapReinhard.process(hdrDebevec)
    ldrReinhard = np.int16(ldrReinhard*255)     # convert to signed 16 bit integer to allow overflow
    ldrReinhard = np.uint8(ldrReinhard)
    return ldrReinhard


if __name__ == "__main__":
    pub = rospy.Publisher("HDRImage",Image,queue_size=1)
    rospy.init_node("hdr_node",anonymous=False)
    rospy.wait_for_service('/CameraMER/GrabMERImage')
    GrabImage = rospy.ServiceProxy('/CameraMER/GrabMERImage',MER_srv)

    bridge = CvBridge()

    while(not rospy.is_shutdown()):
        try:
            time = [100,1000,5000,10000,20000,30000]
            times = np.array(time, dtype=np.float32)*0.000001
            img = []

            for ti in time:
                respl = GrabImage(ti,2.0,1.6,2.5)

                ##########################################
                # 暂时未解决python3兼容问题
                ##########################################
                cv_image = bridge.imgmsg_to_cv2(respl.MER_image, desired_encoding="passthrough")
                img.append(cv_image)
                #pub.publish(bridge.cv2_to_imgmsg(cv_image,'rgb8'))

            hdr_image = hdr_gen(img,times)
            pub.publish(bridge.cv2_to_imgmsg(hdr_image,"rgb8"))

        except rospy.ServiceException as e:
            print("Service call failed:",rospy.ServiceException,e)
            
            

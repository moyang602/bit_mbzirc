#!/usr/bin/env python
#coding=utf-8
import sys
import rospy
from bit_hardware_msgs.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import cv2
import numpy as np
from matplotlib import pyplot as plt

def hdr_gen(images,times):

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
    # cv2.imwrite('ldrReinhard.png',ldrReinhard)

    return ldrReinhard


if __name__ == "__main__":
    pub = rospy.Publisher("Image_hdr",Image,queue_size=1)
    rospy.init_node("image2hdr_node",anonymous=False)
    rospy.wait_for_service('/CameraMER_Left/GrabMERImage')
    GrabImage = rospy.ServiceProxy('/CameraMER_Left/GrabMERImage',MER_srv)

    bridge = CvBridge()

    while(not rospy.is_shutdown()):
        try:
            time = [1000,2000,10000,20000]
            times = np.array(time, dtype=np.float32)
            img = []

            for ti in time:
                respl = GrabImage(ti)
                cv_image_L = bridge.imgmsg_to_cv2(respl.MER_image, desired_encoding="passthrough")
                img.append(cv_image_L)
                #pub.publish(bridge.cv2_to_imgmsg(cv_image_L,'rgb8'))

            hdr_image = hdr_gen(img,times)
            pub.publish(bridge.cv2_to_imgmsg(hdr_image,"bgr8"))

        except rospy.ServiceException, e:
            print "Service call failed:",rospy.ServiceException,e
            

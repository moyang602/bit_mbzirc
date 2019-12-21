#!/usr/bin/env python
import sys
import rospy
from bit_hardware_interface.srv import *
from sensor_msgs.msg import Image
import math

if __name__ == "__main__":
    pub1 = rospy.Publisher("Image1",Image,queue_size=1)
    pub2 = rospy.Publisher("Image2",Image,queue_size=1)
    rospy.init_node("Pub",anonymous=False)
    time = 0
    rospy.wait_for_service('/CameraMER_Left/GrabMERImage')
    GrabImage1 = rospy.ServiceProxy('/CameraMER_Left/GrabMERImage',MER_srv)

    rospy.wait_for_service('/CameraMER_Right/GrabMERImage')
    GrabImage2 = rospy.ServiceProxy('/CameraMER_Right/GrabMERImage',MER_srv)
    while 1:
        time = time +0.2
        try:
            respl = GrabImage1(10000*math.sin(time)+10000)
            # print(respl.success_flag)
            pub1.publish(respl.MER_image)

            resp2 = GrabImage2(10000*math.sin(time)+10000)
            # print(respl.success_flag)
            pub2.publish(resp2.MER_image)
        except rospy.ServiceException, e:
            print "Service call failed:",rospy.ServiceException,e

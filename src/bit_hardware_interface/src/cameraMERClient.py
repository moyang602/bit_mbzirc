#!/usr/bin/env python
import sys
import rospy
from bit_hardware_interface.srv import *
from sensor_msgs.msg import Image
import math

if __name__ == "__main__":
    pub = rospy.Publisher("Image",Image,queue_size=1)
    rospy.init_node("Pub",anonymous=False)
    time = 0
    while 1:
        time = time +0.2
        rospy.wait_for_service('GrabMERImage')
        try:
            GrabImage = rospy.ServiceProxy('GrabMERImage',MER_srv)
            respl = GrabImage(10000*math.sin(time)+10000)
            print(respl.success_flag)
            pub.publish(respl.MER_image)
        except rospy.ServiceException, e:
            print "Service call failed:",rospy.ServiceException,e
    

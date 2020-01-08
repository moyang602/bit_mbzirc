#!/usr/bin/env python
# -*- coding: UTF-8 -*-

'''ur_force ROS Node'''
import rospy
from math import pi
import math
import time,sys,logging,os
import numpy as np
import threading

import roslib
import rospy
import actionlib
import urx
import tf
import traceback

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import PointStamped

odom = PoseWithCovarianceStamped()

def attitude_callback(data)：
    odom.pose.


def pointtcallback(data):



if __name__ == '__main__':

#===================定义一些基本的对象======================#


    rospy.init_node('dji_odom', anonymous = False)
    pub_odom = rospy.Publisher('dji_odom',PoseWithCovarianceStamped,queue_size=10)
    rospy.Subscriber("/dji_sdk/attitude", QuaternionStamped, attitude_callback)
    rospy.Subscriber("//dji_sdk/local_position", PointStamped, pointtcallback)

    while(not rospy.is_shutdown()):
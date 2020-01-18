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

import message_filters

from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import UInt8
from dji_sdk.srv import *

odom = Odometry()
global health
global first
first = True

global tft
tft = tf.TransformerROS()

def callback(atti, posi, vel, ang):
    global first, last_time, T_S_D
    odom.header.frame_id = "gps_odom"
    odom.header.stamp = rospy.Time.now()
    
    if first:
        ori = []
        ori.append(atti.quaternion.x)
        ori.append(atti.quaternion.y)
        ori.append(atti.quaternion.z)
        ori.append(atti.quaternion.w)
        T_S_D = tf.transformations.quaternion_matrix(ori)
        # last_time = now
        first =False
    
    ori = []
    ori.append(atti.quaternion.x)
    ori.append(atti.quaternion.y)
    ori.append(atti.quaternion.z)
    ori.append(atti.quaternion.w)
    R = tf.transformations.quaternion_matrix(ori)
    T_N_D = tft.fromTranslationRotation([posi.point.x, posi.point.y, posi.point.z],ori)

    T_N_S = np.dot( np.linalg.pinv(T_S_D) , T_N_D )
    quat = tf.transformations.quaternion_from_matrix(T_N_S)
    trans = tf.transformations.translation_from_matrix(T_N_S)

    odom.pose.pose.position.x = trans[0] #+= veln[0] * (now - last_time) # = posi.point
    odom.pose.pose.position.y = trans[1] #+= veln[1] * (now - last_time) # = posi.point
    odom.pose.pose.position.z = trans[2] #+= veln[2] * (now - last_time) # = posi.point
    odom.pose.pose.orientation.x = quat[0]
    odom.pose.pose.orientation.y = quat[1]
    odom.pose.pose.orientation.z = quat[2]
    odom.pose.pose.orientation.w = quat[3]
    # last_time = now

    veln = []
    veln.append(vel.vector.x)
    veln.append(vel.vector.y)
    veln.append(vel.vector.z)
    veln.append(1)
    veln = np.dot(R, veln)

    odom.twist.twist.linear.x = veln[0]
    odom.twist.twist.linear.y = veln[1]
    odom.twist.twist.linear.z = veln[2]

    odom.twist.twist.angular.x = ang.vector.x
    odom.twist.twist.angular.y = ang.vector.y
    odom.twist.twist.angular.z = ang.vector.z

    std = 0.5
    a = [std ,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]*5 + [std]
    odom.pose.covariance = a
    odom.twist.covariance = a

    try:
        print(health)
        if health >= 2:
            pub_odom.publish(odom)
        # print(atti,posi)
        # print("end")
    except:
        pass



def health_cb(health_):
    global health
    health = health_.data
    # print(health)

if __name__ == '__main__':

#===================定义一些基本的对象======================#

    rospy.init_node('dji_odom', anonymous = False)
    pub_odom = rospy.Publisher('dji_odom',Odometry,queue_size=10)

    # rospy.wait_for_service('dji_sdk/set_local_position')
    # while True:
    #     try:
    #         get_vision_data = rospy.ServiceProxy('dji_sdk/set_local_position',SetLocalPosRef)
    #         respl = get_vision_data()
    #         if respl.result == True:
    #             break
    #     except rospy.ServiceException as e:
    #         print("Service GetVisionData call failed: %s" %e )

    attitude_sub = message_filters.Subscriber('/dji_sdk/attitude', QuaternionStamped)
    position_sub = message_filters.Subscriber('/dji_sdk/local_position', PointStamped)
    
    vel_sub = message_filters.Subscriber('/dji_sdk/velocity', Vector3Stamped)
    ang_sub = message_filters.Subscriber('/dji_sdk/angular_velocity_fused', Vector3Stamped)
    rospy.Subscriber("/dji_sdk/gps_health", UInt8, health_cb)

    ts = message_filters.ApproximateTimeSynchronizer([attitude_sub, position_sub, vel_sub, ang_sub], 10, 0.1)
    # ts = message_filters.TimeSynchronizer([attitude_sub, position_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()
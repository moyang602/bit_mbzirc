#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''ur_force ROS Node'''
import rospy
from geometry_msgs.msg import WrenchStamped
from math import pi
import time,sys,logging,os
import numpy as np
import threading

import roslib
import rospy
import actionlib
import urx

from geometry_msgs.msg import Twist
from bit_control_tool.msg import EndEffector
from bit_control_tool.msg import heightNow
import bit_motion.msg

from bit_vision.srv import *
from bit_control_tool.srv import SetHeight

if sys.version_info[0] < 3:  # support python v2
    input = raw_input

#=============特征点与运动参数==========
prePickPos = (-1.571, -1.396, -1.745, -1.396, 1.571, 0.0) # [-90.0, -80.0, -100.0, -80.0, 90.0, 0.0]取砖准备位姿
upHeadPos = (-1.57, -1.57, 0, 0, 1.57, 0)
prePutPos = (-1.916, -1.367, 1.621, 1.257, 1.549, -0.344) #(-1.57,-1.29, 1.4, 1.4, 1.57, 0)  # 末端位姿 [0 400 300 0 -180 0]
lookForwardPos = (-1.57, -1.57, -1.57, 0, 1.57, 0)
lookDownPos = (-1.571, -1.396, -1.745, -1.396, 1.571, 0.0)  # 暂时与prePickPos相同
# floorHeight_base = -0.710  # 初始状态机械臂基座离地710mm
# CarHeight_base = -0.140  # 初始状态机械臂基座离车表面140mm
# floorHeight_base = -0.375 - 0.330
# CarHeight_base = 0.180 - 0.330 

global floorHeight_base
global CarHeight_base

posSequence = [] # 随着摆放的过程不断填充这个list来把位置记录下来
l = 0.05
v = 0.05*4
a = 0.3
r = 0.01

#=====================================
GetBrickPos = 1   
GetBrickAngle = 2
GetPutPos = 3
GetPutAngle = 4
GetLPose = 5
GetBrickPos_only = 6

SUCCESS = 1
FAIL_VISION = 2
FAIL_ERROR = 3

TASK_GET = 0
TASK_BUILD = 1
TASK_LOOK_FORWARD = 2
TASK_LOOK_DIRECT_DOWN = 3

global rob
global force 
global ee

def forcecallback(data):
    '''ur_force Callback Function'''
    global force
    force = data.wrench.force.x**2 + data.wrench.force.y**2 + data.wrench.force.z**2
    force = force ** 0.5

def heightcallback(data):
    '''ur_force Callback Function'''
    height = data.x 
    global floorHeight_base
    global CarHeight_base
    floorHeight_base = -0.3737 - height/1000
    CarHeight_base = 0.185 - height /1000

def wait():
    ''' used to debug move one by one step '''
    if do_wait:
        print("Click enter to continue")
        input()


def settle(wait):
    pose = rob.getl()
    pose[4] = -pi
    pose[5] = 0
    rob.movel(pose, acc=a, vel=v, wait=wait)

def GetVisionData_client(ProcAlgorithm, BrickType):
    rospy.wait_for_service('GetVisionData')
    try:
        get_vision_data = rospy.ServiceProxy('GetVisionData',VisionProc)
        respl = get_vision_data(ProcAlgorithm, BrickType)
        return respl.VisionData
    except rospy.ServiceException, e:
        print "Service GetVisionData call failed: %s"%e


if __name__ == '__main__':

#===================定义一些基本的对象======================#

    rospy.init_node('pickputAction', anonymous = False)
    normal = 0
    do_wait = True
    if len(sys.argv) > 1:
        do_wait = False
    
    try :
        global rob
        rob = urx.Robot("192.168.50.60",use_rt = True) 
        normal = 1
        rospy.loginfo('robot ok')
        # Todo 根据实际末端负载与工具中心设置
        rob.set_tcp((0, 0, 0.074, 0, 0, 0))     #TASK2 参数 m,rad
        rob.set_payload(0.96, (0.004, -0.042, 0.011))

        rospy.loginfo("begining")
        pose = rob.getl()
        print("Initial end pose is ", pose)
         # 机械臂移动至取砖准备位姿
        rob.movej((-1.571, -1.571, -1.571, -1.571, 1.571, 0.0),acc=a, vel=0.5*v,wait=True)

        pose = rob.getl()
        pose_t = list(pose)
        record = []
        for i in np.arange(-0.2, 0.2, 0.01):
            for j in np.arange(-0.2, 0.2, 0.01):
                pose_t[0] = pose[0] + i
                pose_t[1] = pose[1] + j
                # print(pose_t)
                # wait()
                rob.movel(pose_t,acc=a, vel=0.5*v,wait=True)
                while(True):
                    VisionData = GetVisionData_client(GetBrickPos, "green")
                    if VisionData.Flag:
                        print(VisionData)
                        if VisionData.Pose.position.x * VisionData.Pose.position.y * VisionData.Pose.position.z != 0:
                            print("ok",VisionData.Pose)
                            record.append([i,j,VisionData.Pose.position.x,VisionData.Pose.position.y,VisionData.Pose.position.z])
                            break
        

    except:
        time.sleep(2.0)
    finally:
        if normal == 1:
            rob.stopl()
            rob.close()
        print(record)
    sys.exit(0)
    
#=========================记录可以使用的api==================

    #rob = urx.Robot("localhost")


    # print("Digital out 0 and 1 are: ", rob.get_digital_out(0), rob.get_digital_out(1))
    # print("Analog inputs are: ", rob.get_analog_inputs())
    #print("force" ,rob.get_force(wait = True))
    # wait()
    #rob.translate((l, 0, 0), acc=a, vel=v)

    # # move to perpendicular to ground
    # pose = rob.getl()
    # pose[3] = -pi
    # pose[4] = 0
    # pose[5] = 0
    # rob.movel(pose, acc=a, vel=v, wait=False)

    # move to first position

    

   
        
            


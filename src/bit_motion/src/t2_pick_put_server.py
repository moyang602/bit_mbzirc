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

v = 0.05*4
a = 0.3

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

## ================================================== ##
## ================================================== ##
##                    Todo List                       ##
##                                                    ##
## 1\ 调用视觉的服务进行砖块精确定位                       ##
## 2\ 根据砖块序号来确定放在车上的位置，并记录到posSequence  ##
## 3\ 建筑任务的欲放置位置需要根据砖块x,y确定               ##
## 4\ 建筑任务的放置砖需结合手眼完成                       ##
##                                                    ##
##                                                    ##
##                                                    ##
##                                                    ##
##                                                    ##
##                                                    ##
##                                                    ##
## ================================================== ##
## ================================================== ##

class pick_put_act(object):
    _feedback = bit_motion.msg.pickputFeedback()
    _result = bit_motion.msg.pickputResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, bit_motion.msg.pickputAction , execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("%s server ready! ", name)

    def show_tell(self, info):
        rospy.loginfo(info)
        self._feedback.move_rightnow = info
        self._as.publish_feedback(self._feedback)

    def execute_cb(self, goal):
        # helper variables
        # r = rospy.Rate(1)
        self._result.finish_state = SUCCESS
        try: 
            # 开始臂车运动
            rospy.loginfo("begining")
            pose = rob.getl()
            print("Initial end pose is ", pose)
            initj = rob.getj()
            print("Initial joint angle is ", initj)

            if goal.task == TASK_GET:
                # 机械臂移动至取砖准备位姿
                rob.movej(prePickPos,acc=a, vel=3*v,wait=True)
                self.show_tell("arrived pre-pick position")

                # 视觉搜索目标砖块位置
                rospy.sleep(0.5)
                while True:         # Todo 避免进入死循环
                    VisionData = GetVisionData_client(GetBrickPos, goal.goal_brick.type)
                    if VisionData.Flag:
                        break

                self.show_tell("Got BrickPos results")
                # 判断是否在工作空间，不是任务返回失败    
                # 工作空间检测函数  Todo
                x = VisionData.Pose.position.x
                y = VisionData.Pose.position.y
                dist = (x**2 + y**2)**0.5
                '''
                if dist>0.668 or x>0.200 or x<-0.200 and y <-0.250: # 待矫正
                    self.show_tell("Brick position is out of workspace")
                    return
                '''

                # 得到识别结果，平移到相机正对砖块上方，
                pose[0] = VisionData.Pose.position.x-0.023      #使ZED正对砖块中心  0.023为zed与magnet偏移
                pose[1] = VisionData.Pose.position.y+0.142      #使ZED正对砖块中心  0.142为zed与magnet偏移
                pose[2] = floorHeight_base + 1.0     # 离地1m处      Todo   偏移值待确定
                pose[3] = 0
                # 下两个坐标使其垂直于地面Brick remembered
                pose[4] = -pi 
                pose[5] = 0 
                rob.movel(pose, acc=a, vel=v, wait=True)

                # 视觉搜索目标砖块角度
                rospy.sleep(0.5)
                while True:         # Todo 避免进入死循环
                    VisionData = GetVisionData_client(GetBrickAngle, goal.goal_brick.type)
                    if VisionData.Flag:
                        break

                # 得到识别结果如下
                theta = VisionData.Pose.orientation.z
                self.show_tell("Got BrickAngle results")

                # 得到识别结果，移动到砖块上方0.1m，旋转角度
                pose[0] = VisionData.Pose.position.x
                pose[1] = VisionData.Pose.position.y
                pose[2] = floorHeight_base + 0.25     # 0.25是离车表面25cm 
                pose[3] = 0.0
                # 下两个坐标使其垂直于地面Brick remembered
                pose[4] = -pi 
                pose[5] = 0.0 
                rob.movel(pose, acc=a, vel=v, wait=True)
                rospy.sleep(0.5)

                rob.movej([0,0,0,0,0,theta],acc=a, vel=1*v,wait=True, relative=True)

                self.show_tell("Arrived block up 0.05m position pependicular to brick")

                # 伪力控下落
                pose[0] = 0.0
                pose[1] = 0.0
                pose[2] = -0.1   
                pose[3] = 0.0
                pose[4] = 0.0 
                pose[5] = 0.0
                rob.movel(pose, acc=a, vel=v*0.1, wait=False, relative=True)     # 相对运动
 
                _force_prenvent_wrongdata_ = 0
                while force < 15:
                    _force_prenvent_wrongdata_ += 1
                    if _force_prenvent_wrongdata_ >150: 
                        _force_prenvent_wrongdata_ = 150 
                    rospy.sleep(0.002)
                    if  _force_prenvent_wrongdata_ >100 and ( not rob.is_program_running() ):
                        rospy.loginfo("did not contact")
                        break
                rob.stopl()
                self.show_tell("Reached Block")

                # 操作末端
                rospy.sleep(1.0)
                ee.MagState = 100
                pub_ee.publish(ee)
                rospy.sleep(1.0)

                # 先提起，后转正
                rob.translate((0,0,0.3), acc=a, vel=v, wait=True)

                # 移动到预放置位置
                rob.movej(prePutPos,acc=a, vel=2*v,wait=True)
                
                self.show_tell("arrived pre-Put position %d" % goal.goal_brick.Sequence)
                delta = (0.0, -0.1+goal.goal_brick.Sequence * 0.21, 0, 0, 0, 0)
                rob.movel(delta, acc=a, vel=v,wait=True, relative=True )

                pose = rob.getl()
                pose[2] = CarHeight_base + 0.32
                rob.movel(pose, acc=a, vel=v,wait=True)
                # 需要加入砖块信息来确定定位
                # 使用砖块的序列信息来计算自己需要放在那个位置，待完成
                # 移动到位，并记录posSequence = f(goal.goal_brick.Sequence)
                # posSequence.append(delta)

                # 伪力控放置
                rob.translate((0, 0, -0.32), acc=a, vel=v*0.15, wait=False)
                _force_prenvent_wrongdata_ = 0
                while force < 15:
                    _force_prenvent_wrongdata_ += 1
                    if _force_prenvent_wrongdata_ >150: 
                        _force_prenvent_wrongdata_ = 150 
                    rospy.sleep(0.002)
                    if  _force_prenvent_wrongdata_ >100 and ( not rob.is_program_running() ):
                        rospy.loginfo("did not contact")
                        break
                rob.stopl()
                self.show_tell("Put Down")

                # 释放末端
                rospy.sleep(1.0)
                ee.MagState = 0
                pub_ee.publish(ee)
                rospy.sleep(1.0)

                # 移动回来
                rob.translate((0, 0, 0.05), acc=a, vel=v*0.3, wait=True)
                rob.movej(prePutPos,acc=a, vel=2*v,wait=True)
                self.show_tell("arrived pre-Put position, finished")

            elif goal.task == TASK_BUILD:
                # 移动至车上取砖位姿
                rob.movej(prePutPos,acc=a, vel=3*v,wait=True)
                self.show_tell("arrived pre-put position")

                # 移动至对应砖块处
                # print(posSequence[goal.goal_brick.Sequence],goal.goal_brick.Sequence)
                delta = (0.0, goal.goal_brick.Sequence * 0.2 +0.1, 0, 0, 0, 0)
                rob.movel(delta, acc=a, vel=0.5*v,wait=True, relative=True )
                #rob.movel(posSequence[goal.goal_brick.Sequence], acc=a, vel=1*v,wait=True, relative=True )
                self.show_tell("arrived Brick remembered position")

                # 进行识别
                ## server ask vision! wait and try some times
                # 视觉搜索目标砖块位置    待修改

                rospy.sleep(0.5)
                while True:         # Todo 避免进入死循环
                    VisionData = GetVisionData_client(GetBrickPos, goal.goal_brick.type)
                    if VisionData.Flag:
                        break

                # 得到识别结果，移动到砖块上方，平移
                pose[0] = VisionData.Pose.position.x
                pose[1] = VisionData.Pose.position.y

                pose[2] = CarHeight_base + 0.25     # 0.25是离车表面25cm
                pose[3] = 0
                # 下两个坐标使其垂直于地面Brick remembered
                pose[4] = -pi 
                pose[5] = 0
                rob.movel(pose, acc=a, vel=v, wait=True)
                
                rospy.sleep(0.5)
                # 伪力控下落
                rob.translate((0,0,-0.1), acc=a, vel=v*0.1, wait=False)
                _force_prenvent_wrongdata_ = 0
                while force < 15:
                    _force_prenvent_wrongdata_ += 1
                    if _force_prenvent_wrongdata_ >150: 
                        _force_prenvent_wrongdata_ = 150 
                    rospy.sleep(0.002)
                    if  _force_prenvent_wrongdata_ >100 and ( not rob.is_program_running() ):
                        rospy.loginfo("did not contact")
                        break
                rob.stopl()
                self.show_tell("Reached Block")

                # 操作末端
                rospy.sleep(1.0)
                ee.MagState = 100
                pub_ee.publish(ee)
                rospy.sleep(1.0)

                # 提起
                rob.translate((0,0,0.25), acc=a, vel=v, wait=True)

                # 根据建筑物位置改变升降台到高度
                # rospy.wait_for_service('Setheight')
                # set_height = rospy.ServiceProxy('Setheight',SetHeight)
                # set_height(400)

                rob.movej(prePickPos,acc=a, vel=3*v,wait=True)     
                self.show_tell("arrived pre-Build position")
                    
                # 配合手眼移动到摆放的位置
                # Todo 有问题，需要移动到砖xy，解算出的位置

                pose[0] = 0.0
                pose[1] = -0.5

                pose[2] = goal.goal_brick.Sequence*0.2 + floorHeight_base + 0.25     # 0.25是离车表面25cm
                pose[3] = 0
                # 下两个坐标使其垂直于地面Brick remembered
                pose[4] = -pi 
                pose[5] = 0
                rob.movel(pose, acc=a, vel=v, wait=True)


                # 伪力控放置
                rob.translate((0, 0, -0.1), acc=a, vel=v*0.1, wait=False)
                _force_prenvent_wrongdata_ = 0
                while force < 15:
                    _force_prenvent_wrongdata_ += 1
                    if _force_prenvent_wrongdata_ >150: 
                        _force_prenvent_wrongdata_ = 150 
                    rospy.sleep(0.002)
                    if  _force_prenvent_wrongdata_ >100 and ( not rob.is_program_running() ):
                        rospy.loginfo("did not contact")
                        break
                        
                rob.stopl()
                self.show_tell("Put Down")

                # 释放末端
                rospy.sleep(1.0)
                ee.MagState = 0
                pub_ee.publish(ee)
                rospy.sleep(1.0)

                rob.translate((0, 0, 0.05), acc=a, vel=v*0.3, wait=true)

                rob.movej(prePickPos,acc=a, vel=3*v,wait=True)
                self.show_tell("arrived pre-Build position, finished")

            elif goal.task == TASK_LOOK_FORWARD:
                rob.movej(lookForwardPos, acc=a, vel=3*v,wait=True)
                rospy.sleep(2.0)
                self.show_tell("arrived look forward position")
            elif goal.task == TASK_LOOK_DIRECT_DOWN:
                rob.movej(lookDownPos, acc=a, vel=3*v,wait=True)
                rospy.sleep(2.0)
                self.show_tell("arrived look forward position")

        except Exception as e:
            print("error", e)
            self._result.finish_state = FAIL_ERROR
            self._as.set_aborted(self._result)
            rob.stopl()
            # rob = urx.Robot("192.168.50.60",use_rt = True)
            # rob.set_tcp((0, 0, 0, 0, 0, 0))
            # rob.set_payload(0.0, (0, 0, 0))
        finally:
            if self._result.finish_state == SUCCESS:
                self._as.set_succeeded(self._result)
            else:
                self._as.set_aborted(self._result)


if __name__ == '__main__':

#===================定义一些基本的对象======================#

    rospy.init_node('pickputAction', anonymous = False)
    pub_ee = rospy.Publisher('endeffCmd',EndEffector,queue_size=1)
    rospy.Subscriber("/wrench", WrenchStamped, forcecallback)
    rospy.Subscriber("/heightNow", heightNow, heightcallback)

    ee = EndEffector()
    normal = 0
    do_wait = True
    if len(sys.argv) > 1:
        do_wait = False
    while(not rospy.is_shutdown()):
        try :
            global rob
            rob = urx.Robot("192.168.50.60",use_rt = True) 
            normal = 1
            rospy.loginfo('robot ok')
            # Todo 根据实际末端负载与工具中心设置
            rob.set_tcp((0, 0, 0.074, 0, 0, 0))     #TASK2 参数 m,rad
            rob.set_payload(0.96, (0.004, -0.042, 0.011))

            pick_put_act("pickputAction")     # rospy.get_name())
            rospy.spin() 
        except:
            time.sleep(2.0)
        finally:
            if normal == 1:
                rob.stopl()
                rob.close()
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

    

   
        
            


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

if sys.version_info[0] < 3:  # support python v2
    input = raw_input

#=============特征点与运动参数==========
prePickPos = (-1.57, -1.57, -1.57, -1.57, 1.57, 0)
upHeadPos = (-1.57, -1.57, 0, 0, 1.57, 0)
prePutPos = (-1.57,-1.29, 1.4, 1.4, 1.57, 0)
lookForwardPos = (-1.57, -1.57, -1.57, 0, 1.57, 0)
# -1.57,-1.29, 1.4, 1.4, 1.57, 0

posSequence = [] # 随着摆放的过程不断填充这个list来把位置记录下来
l = 0.05
v = 0.05*4
a = 0.3
r = 0.01

#=====================================

SUCCESS = 1
FAIL_VISION = 2
FAIL_ERROR = 3

TASK_GET = 0
TASK_BUILD = 1
TASK_LOOK = 2

global rob
global force 
global ee

def forcecallback(data):
    '''ur_force Callback Function'''
    global force
    force = data.wrench.force.x**2 + data.wrench.force.y**2 + data.wrench.force.z**2
    force = force ** 0.5

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

def GetLocateData_client():
    rospy.wait_for_service('GetLocateData')
    try:
        get_locate_data = rospy.ServiceProxy('GetLocateData',BrickLocate)
        respl = get_locate_data()
        return respl.LocateData
    except rospy.ServiceException, e:
        print "Service GetLocateData call failed: %s"%e

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
            rospy.loginfo("begining")
            initj = rob.getl()
            print("Initial joint configuration is ", initj)
            
            t = rob.get_pose()
            print("Transformation from base to tcp is: ", t)
            
            # wait()
            if goal.task == TASK_GET:
                rob.movej(prePickPos,acc=a, vel=3*v,wait=True)
                self.show_tell("arrived pre-pick position")
            elif goal.task == TASK_BUILD:
                rob.movej(prePutPos,acc=a, vel=3*v,wait=True)
                self.show_tell("arrived pre-put position")
                print(posSequence[goal.goal_brick.Sequence],goal.goal_brick.Sequence)
                rob.movel(posSequence[goal.goal_brick.Sequence], acc=a, vel=3*v,wait=True, relative=True )
                self.show_tell("arrived Brick remembered position")
            elif goal.task == TASK_LOOK:
                rob.movej(lookForwardPos, acc=a, vel=3*v,wait=True)
                self.show_tell("arrived look forward position")
                return 0

            # 进行识别
            # client(goal.goal_brick.type)
            ## server ask vision! wait and try some times
            # rospy.sleep(1.0)
            # while True:
            #     LocateData = GetLocateData_client()
            #     if LocateData.flag:
            #         break

            # 得到识别结果如下
            # x = LocateData.position.x
            # y = LocateData.position.y
            # z = LocateData.position.z
            # theta = 0

            theta = -0.5
            x = 0.0
            y = 0.0
            z = 0.2

            self.show_tell("Got recognition results")
            if 0:
                self._result.finish_state = FAIL_VISION
                return 0
            
            # 得到识别结果，移动到砖块上方，平移和旋转末端
            initj = rob.getl()
            initj[0] += -x
            initj[1] += -y

            initj[2] += -z +0.1     # 0.1是在其上的10cm
            initj[3] += theta
            # 下两个坐标使其垂直于地面Brick remembered
            initj[4] = - pi 
            initj[5] = 0 
            rob.movel(initj, acc=a, vel=v, wait=True)
            rospy.sleep(2.0)
            self.show_tell("Arrived block up 0.1m position pependicular to ground")

            # wait()
            # 伪力控下落
            rob.translate((0,0,-0.3), acc=a, vel=v*0.3, wait=False)
            _force_prenvent_wrongdata_ = 0
            while force < 20:
                _force_prenvent_wrongdata_ += 1
                if _force_prenvent_wrongdata_ >150: 
                    _force_prenvent_wrongdata_ = 150 
                rospy.sleep(0.002)
                if  _force_prenvent_wrongdata_ >100 and ( not rob.is_program_running() ):
                    rospy.loginfo("did not contact")
                    break
            rob.stopl()
            self.show_tell("Reached Block")

            # wait()
            # 操作末端
            rospy.sleep(1.0)
            ee.MagState = 100
            pub_ee.publish(ee)
            rospy.sleep(1.0)

            # wait()
            # 先提起，后转正
            rob.translate((0,0,0.3), acc=a, vel=v, wait=True)
            # wait() 
            rob.movej((0, 0, 0, 0, 0, theta),acc=a, vel=v, wait=False , relative=True)

            # wait()
            # 移动到头顶
            # rob.movej(upHeadPos,acc=a, vel=v,wait=True)
            # self.show_tell("arrived Head-up position")

            # wait()
            # 移动到预放置位置
            if goal.task == TASK_GET:
                rob.movej(prePutPos,acc=a, vel=v,wait=True)
                self.show_tell("arrived pre-Put position %d" % goal.goal_brick.Sequence)
                delta = (0.1, goal.goal_brick.Sequence * 0.2 , 0, 0, 0, 0)
                rob.movel(delta, acc=a, vel=v,wait=True, relative=True )
                # 需要加入砖块信息来确定定位
                # 使用砖块的序列信息来计算自己需要放在那个位置，待完成
                # 移动到位，并记录posSequence = f(goal.goal_brick.Sequence)
                posSequence.append(delta)
            elif goal.task == TASK_BUILD:
                rob.movej(prePickPos,acc=a, vel=v,wait=True)      # 有问题，需要移动到砖xy，解算出的位置
                self.show_tell("arrived pre-Build position")
                
                # 配合手眼移动到摆放的位置
                
            # wait()
            # 伪力控放置
            rob.translate((0, 0, -0.3), acc=a, vel=v*0.3, wait=False)
            _force_prenvent_wrongdata_ = 0
            while force < 20:
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

            if goal.task == TASK_GET:
                rob.movej(prePutPos,acc=a, vel=3*v,wait=True)
                self.show_tell("arrived pre-Put position, finished")
            elif goal.task == TASK_BUILD:
                rob.movej(prePickPos,acc=a, vel=3*v,wait=True)
                self.show_tell("arrived pre-Build position, finished")

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
            rob.set_tcp((0, 0, 0, 0, 0, 0))
            rob.set_payload(0.0, (0, 0, 0))

            pick_put_act("pickputAction")     # rospy.get_name())  #
            rospy.loginfo('action server ok')
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

    

   
        
            


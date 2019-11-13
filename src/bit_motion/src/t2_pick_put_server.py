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


if sys.version_info[0] < 3:  # support python v2
    input = raw_input

#=============特征点与运动参数==========
prePickPos = (-1.57, -1.57, -1.57, -1.57, 1.57, 0)
upHeadPos = (-1.57, -1.57, 0, 0, 1.57, 0)
prePutPos = (-1.57, -1.29, 1.47, 1.4, 1.57, 0)
l = 0.05
v = 0.05*4
a = 0.3
r = 0.01

#=====================================

SUCCESS = 1
FAIL_VISION = 2
FAIL_ERROR = 3
FAIL_MOVE = 4

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


class pick_put_act(object):
    _feesback = bit_motion.msg.pickputFeedback()
    _result = bit_motion.msg.pickputResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, bit_motion.msg.pickputAction , execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("%s server ready! ", name)

    def show_tell(self, info):
        rospy.loginfo(info)
        self._feedback.move_rightnow = info
        self._as.publish(self._result)

    def execute_cb(self, goal_brick):
        # helper variables
        # r = rospy.Rate(1)
        self._result.finish_state = SUCCESS
    
        global rob 
        try: 
            
            initj = rob.getl()
            
            rospy.loginfo("Initial joint configuration is ", initj)
            
            rospy.spin()
            t = rob.get_pose()
            rospy.loginfo("Transformation from base to tcp is: ", t)
            
            # wait()
            rob.movej(prePickPos,acc=a, vel=v,wait=True)
            show_tell("arrived pre-pick position")

            # 进行识别
            # client(goal_brick.type)
            ## server ask vision! wait and try some times
            time.sleep(1.0)
            theta = -0.5
            x = 0.1
            y = 0.1
            z = 0.4

            show_tell("Got recognition results")
            if 0:
                self._result.finish_state = FAIL_VISION
                return 0
            

            # 得到识别结果，移动到砖块上方，平移和旋转末端
            initj = rob.getl()
            initj[0] += -x
            initj[1] += -y

            initj[2] += -z +0.1     # 0.1是在其上的10cm
            initj[3] += theta
            # 下两个坐标使其垂直于地面
            initj[4] = - pi 
            initj[5] = 0 
            rob.movel(initj, acc=a, vel=v, wait=True)
            time.sleep(2.0)
            show_tell("Arrived block up 0.1m position,begin settle")

            # wait()
            # 伪力控
            rob.translate((0,0,-0.2), acc=a, vel=v, wait=False)
            _force_prenvent_wrongdata_ = 0
            while force < 15:
                _force_prenvent_wrongdata_ += 1
                if _force_prenvent_wrongdata_ >150: 
                    _force_prenvent_wrongdata_ = 150 
                time.sleep(0.002)
                if  _force_prenvent_wrongdata_ >100 and ( not rob.is_program_running() ):
                    rospy.loginfo("did not contact")
                    break
            rob.stopl()

            # wait()
            # 操作末端
            time.sleep(1.0)
            ee.MagState = 100
            pub_ee.publish(ee)
            time.sleep(1.0)

            # wait()
            # 先提起，后转正
            rob.translate((0,0,0.3), acc=a, vel=v, wait=True)
            # wait() 
            rob.movej((0, 0, 0, 0, 0, theta),acc=a, vel=v, wait=False , relative=True)

            # wait()
            # 移动到头顶
            rob.movej(upHeadPos,acc=a, vel=v,wait=True)
            show_tell("arrived Head-up position")


            # wait()
            # 移动到预放置位置
            rob.movej(prePutPos,acc=a, vel=v,wait=True)
            show_tell("arrived pre-Put position")


            # 需要加入砖块信息来确定定位
            # wait()
            # 伪力控放置
            rob.translate((0, 0, -0.3), acc=a, vel=v, wait=False)
            _force_prenvent_wrongdata_ = 0
            while force < 15:
                _force_prenvent_wrongdata_ += 1
                if _force_prenvent_wrongdata_ >150: 
                    _force_prenvent_wrongdata_ = 150 
                time.sleep(0.002)
                if  _force_prenvent_wrongdata_ >100 and ( not rob.is_program_running() ):
                    rospy.loginfo("did not contact")
                    break
            rob.stopl()

            # wait()
            # 释放末端
            time.sleep(1.0)
            ee.MagState = 0
            pub_ee.publish(ee)
            time.sleep(1.0)

            # wait()
            rob.movej(prePutPos,acc=a, vel=v,wait=True)
            rospy.show_tell("arrived pre-Put position, finished")

        except:
            rospy.loginfo("error")
            self._result.finish_state = FAIL_ERROR
            self._as.set_aborted(self._result)
            rob.close()
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

    rospy.init_node('pickputAction', anonymous = True)
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
            print('robot ok')
            rob.set_tcp((0, 0, 0, 0, 0, 0))
            rob.set_payload(0.0, (0, 0, 0))

            pick_put_act("pickputAction")     # rospy.get_name())  #
            print('action server ok')
            rospy.spin() 
        except:
            time.sleep(2.0)
        finally:
            print('last')
            if normal == 1:
                rob.stopl()
                rob.close()
                print('ok')
                break
            print(rospy.is_shutdown())
    os.kill()
    
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

    

   
        
            


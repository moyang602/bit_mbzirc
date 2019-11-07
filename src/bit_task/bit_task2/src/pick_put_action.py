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
import urx

from geometry_msgs.msg import Twist
from bit_control_tool.msg import EndEffector

if sys.version_info[0] < 3:  # support python v2
    input = raw_input


def callback(data):
    '''ur_force Callback Function'''
    global force
    force = data.wrench.force.x**2 + data.wrench.force.y**2 + data.wrench.force.z**2
    force = force ** 0.5

def wait():
    if do_wait:
        print("Click enter to continue")
        input()

 
def thread_job():
    rospy.spin()
    rob.stopl()
    rob.close()
    sys.exit(0)

def settle(wait):
    pose = rob.getl()
    pose[4] = -pi
    pose[5] = 0
    rob.movel(pose, acc=a, vel=v, wait=wait)


prePickPos = (-1.57, -1.57, -1.57, -1.57, 1.57, 0)
upHeadPos = (-1.57, -1.57, 0, 0, 1.57, 0)
prePutPos = (-1.57, -1.29, 1.47, 1.4, 1.57, 0)

if __name__ == '__main__':

#===================定义一些基本的对象======================#

    rospy.init_node('ur_task',anonymous=True)
    pub_ee = rospy.Publisher('endeffCmd',EndEffector,queue_size=1)
    rospy.Subscriber("/wrench", WrenchStamped, callback)

    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    logging.basicConfig(level=logging.INFO)

    global force 
    ee = EndEffector()
    
    do_wait = True
    if len(sys.argv) > 1:
        do_wait = False
    
    rob = urx.Robot("192.168.50.60",use_rt = True)

#=========================开始控制==================

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

    rob.set_tcp((0, 0, 0, 0, 0, 0))
    rob.set_payload(0.0, (0, 0, 0))
    l = 0.05
    v = 0.05*4
    a = 0.3
    r = 0.01

    cnt = 3     #任务次数

    try:
        while cnt > 0 :
            cnt -= 1
        
            initj = rob.getl()
            print("Initial joint configuration is ", initj)
            t = rob.get_pose()
            print("Transformation from base to tcp is: ", t)
            
            # wait()
            rob.movej(prePickPos,acc=a, vel=v,wait=True)
            print("arrived pre-pick position")

            # 进行识别
            time.sleep(1.0)
            print("Got recognition results")
            theta = -0.5
            x = 0.1
            y = 0.1
            z = 0.4

            # wait()
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
            print("Arrived block up 0.1m position,begin settle")

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
                    print("did not contact")
                    break
            rob.stopl()

            # wait()
            # 操作末端
            time.sleep(0.5)
            ee.MagState = 100
            pub_ee.publish(ee)
            time.sleep(0.5)

            # wait()
            # 先提起，后转正
            rob.translate((0,0,0.3), acc=a, vel=v, wait=True)
            # wait() 
            rob.movej((0, 0, 0, 0, 0, theta),acc=a, vel=v, wait=False , relative=True)

            # wait()
            # 移动到头顶
            rob.movej(upHeadPos,acc=a, vel=v,wait=True)
            print("arrived Head-up position")

            # wait()
            # 移动到预放置位置
            rob.movej(prePutPos,acc=a, vel=v,wait=True)
            print("arrived pre-Put position")
            print("start settle")
            # settle(wait = True)

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
                    print("did not contact")
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
            print("arrived pre-Put position")


        print("finished")
        rob.close()
        print("Wait for Interupt!")
        rospy.spin()
        rob.close()
        # wait()
    finally:
        rob.stopl()
        rospy.spin()
        


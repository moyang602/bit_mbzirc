#!/usr/bin/env python
#coding='utf-8'
'''ur_force ROS Node'''
import rospy
from geometry_msgs.msg import WrenchStamped
from math import pi
import time
import sys
import urx
import logging
import numpy as np

import threading

if sys.version_info[0] < 3:  # support python v2
    input = raw_input

global force

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
    rob.close()
    sys.exit(0)

def settle(wait):
    pose = rob.getl()
    pose[3] = -pi
    pose[4] = 0
    pose[5] = 0
    rob.movel(pose, acc=a, vel=v, wait=wait)


pos1 = (-1.57, -1.57, -1.57, -1.57, 1.57, 1.57)
pos2 = (-1.57, -1.57, 0, 0, 1.57, 1.57)
pos3 = (-1.57, -1.29, 1.47, 1.4, 1.57, 1.57)

if __name__ == '__main__':


    rospy.init_node('ur_task',anonymous=True)
    rospy.Subscriber("/wrench", WrenchStamped, callback)
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()

    global force 

    logging.basicConfig(level=logging.INFO)
    
    do_wait = True
    if len(sys.argv) > 1:
        do_wait = False
    
    rob = urx.Robot("192.168.50.60",use_rt = True)

    #rob = urx.Robot("localhost")
    rob.set_tcp((0, 0, 0, 0, 0, 0))
    rob.set_payload(0.0, (0, 0, 0))
    try:
        l = 0.05
        v = 0.05
        a = 0.3
        r = 0.01
        # print("Digital out 0 and 1 are: ", rob.get_digital_out(0), rob.get_digital_out(1))
        # print("Analog inputs are: ", rob.get_analog_inputs())
       
        initj = rob.getj()
        print("Initial joint configuration is ", initj)
        t = rob.get_pose()
        print("Transformation from base to tcp is: ", t)
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
        print("press to move to first position")
        wait()


        rob.movej(pos1,acc=a, vel=v*4,wait=True)
        print("arrived first position")
        time.sleep(3.0)
        wait()


        rob.movej(pos2,acc=a, vel=v*4,wait=True)
        print("arrived second position")
        time.sleep(3.0)
        wait()

        rob.movej(pos3,acc=a, vel=v*4,wait=True)
        print("arrived first position")
        time.sleep(3.0)
        print("start settle")
        settle(wait = False)
        wait()


        print("finished")
        rob.close()
        print("Wait for Interupt!")
        rospy.spin()
        rob.close()
        # wait()
    finally:
        rospy.spin()
        


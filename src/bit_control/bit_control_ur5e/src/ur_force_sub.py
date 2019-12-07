#!/usr/bin/env python
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

if __name__ == '__main__':

    rospy.init_node('ur_force', anonymous=True)
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
        print("Digital out 0 and 1 are: ", rob.get_digital_out(0), rob.get_digital_out(1))
        print("Analog inputs are: ", rob.get_analog_inputs())
        initj = rob.getj()
        print("Initial joint configuration is ", initj)
        t = rob.get_pose()
        print("Transformation from base to tcp is: ", t)

        #print("force" ,rob.get_force(wait = True))
        wait()
        #rob.translate((l, 0, 0), acc=a, vel=v)
        pose = rob.getl()
        pose[3] = -pi
        pose[4] = 0
        pose[5] = 0
        rob.movel(pose, acc=a, vel=v, wait=True)

        

        print("robot tcp is at: ", pose)
        print("moving in z force")
        wait()

        
        pose[2] = -0.3
        rob.movel(pose, acc=a, vel=v, wait=False)
        print("next")
        i = 0
        while force < 15:
            i = i+1
            if i >150: 
                i = 150 
            print("force",force)
            time.sleep(0.002)
            if  i >100 and ( not rob.is_program_running() ):
                print("no one , out")
                break
        rob.stopl()

        print("finished")
        wait()


    finally:
        rospy.spin()
        rob.close()


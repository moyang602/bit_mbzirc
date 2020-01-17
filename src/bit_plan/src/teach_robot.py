#!/usr/bin/env python

import rospy, time
import sys
import tty
import termios
import tf
from bit_task_msgs.srv import teach_robot

global tf_CarOnMap_trans
global tf_CarOnMap_rot
global tf_StartOnMap_trans
global tf_StartOnMap_rot
global tf_TargetOnMap_trans
global tf_TargetOnMap_rot
global tf_TargetOnStart_trans
global tf_TargetOnStart_rot 

START = 0
REC = 1
GET = 2
FINISH = 3

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)


if __name__ == '__main__':

    rospy.init_node('teach_node', anonymous=False)
    rospy.wait_for_service('Teach_robot')
    teach_point_draw = rospy.ServiceProxy('Teach_robot',teach_robot)

    while(not rospy.is_shutdown()):        

        key=readkey()
        print(key)
        if key=='s':
            teach_point_draw(START)
        if key=='r': 
            teach_point_draw(REC)
        if key=='f': 
            teach_point_draw(FINISH)
        if key=='g': 
            a = teach_point_draw(GET)
            print(a)
        if key=='e':
            break
        
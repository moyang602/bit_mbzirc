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

START   = 0
REC     = 1
FINISH2 = 2
FINISH3 = 3
GET2    = 4
GET3    = 5

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
            print("Task?")
            while True:
                key=readkey()
                if key == '2':
                    teach_point_draw(FINISH2)
                    print("Save Task2 success")
                    break
                elif key == '3':
                    teach_point_draw(FINISH3)
                    print("Save Task3 success")
                    break
                else:
                    print("Wrong input")
                    continue

        if key=='g': 
            print("Task?")
            while True:
                key=readkey()
                if key == '2':
                    print("Get once 2")
                    a = teach_point_draw(GET2)
                    print (a)
                elif key == '3':
                    print("Get once 3")
                    a = teach_point_draw(GET3)
                    print (a)
                elif key == 'g':
                    print("Finish Get")
                    break
                else:
                    print("Wrong input")
                    continue
        # if key=='g': 
        #     a = teach_point_draw(GET)
        #     print(a)
        if key=='e':
            break
        
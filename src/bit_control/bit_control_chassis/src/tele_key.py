#!/usr/bin/env python
#coding=utf-8

'''tele_key ROS Node'''
# license removed for brevity
import roslib
import rospy

from geometry_msgs.msg import Twist
from bit_control_msgs.msg import EndEffector

import sys, select, termios, tty ,os
import time

import numpy as np

msg_remote = '''
------------------------------
请使用遥控器控制：
左遥杆：⇅ 前后\t⇄自旋
右遥杆：⇅ 油门\t⇄横移
组合动作：
⇅ ↘  升降台起落
↗ ↘  开/关泵
↘ ↘  开/关电磁铁
⇇ ⇊  加锁
↘ ↙  解锁
切换模式：
↗ ↙：机器人自主导航
↘ ↙：遥控器控制

空格交给键盘！\r
'''

msg = '''
------------------------------
\rPress to control the car!\r
w/s ：前进后退\r
a/d ：左右平移\r
q/e ：左右转弯\r
 2  ：停止运动\r
1/3 : 减速加速\r
r/f : 上升下降\r
m/p : 电磁水泵\r
l   : 规划模式

  0 ：交给遥控器\r
空格退出！\r
'''


remoterEnable = 0
planEnable = 0

MAX_x = 1
MAX_y = 1
MAX_z = 0.6

r_state = 'r' 
remote_mode = plan_mode = eem_mode = open_m = open_p = eep_mode =  0
eep = eem = planp = remotep = 1




def callback(data):
    global remote_mode,plan_mode,eem_mode,eep_mode,open_m,open_p,eep,eem,planp,remotep,r_state
    global planEnable
    #global turn_cnt
    if remoterEnable ==1:
        if (data.linear.x+data.linear.y+data.angular.x+data.angular.y) == 0:
            os.system('clear')
            print(msg_remote)
            print("遥控器归零")
        else:
            os.system('clear')
            print(msg_remote)
            print("遥控器启动")
        if r_state == 'r':
            print("遥控模式")
        else:
            print("规划模式")

        if data.angular.x > 0 :
            if r_state == 'r':
                twist.linear.x  = data.angular.x * data.linear.y / 450000.0 * MAX_x
                twist.linear.y  = data.angular.x * data.linear.x / 450000.0 * MAX_y 
                twist.linear.z  = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = data.angular.x * data.angular.y / 450000.0 * MAX_z 
                
                pub.publish(twist)  
                print("rp")
            elif r_state == 'p':
                planEnable = 1
        else :
            planEnable = 0
            if data.linear.x > 300:
                if data.linear.y < -300:
                    remote_mode += remotep
                    plan_mode = 0
                elif data.linear.y > 300:
                    plan_mode += planp
                    remote_mode = 0
                else:
                    remote_mode = plan_mode = 0
                    remotep = planp = 1
            elif data.linear.x < -300:
                if abs(data.angular.y ) < 10:
                    if abs( data.linear.y ) > 10:
                        twist.linear.x  = 0
                        twist.linear.y  = 0
                        twist.linear.z  = 0.05 * data.linear.y/abs(data.linear.y)
                        twist.angular.z = 0
                        pub.publish(twist) 
                    else:
                        twist.linear.x  = 0
                        twist.linear.y  = 0
                        twist.linear.z  = 0
                        twist.angular.z = 0
                        pub.publish(twist) 

                elif data.angular.y < -300:
                    if data.linear.y >300:
                        eep_mode += eep
                        eem_mode = 0
                    elif data.linear.y < -300:
                        eem_mode += eem
                        eep_mode = 0
                    else:
                        eep = eem = 1 
                        eep_mode = eem_mode = 0
            else: 
               
                remote_mode = plan_mode = eem_mode = eep_mode =0
                twist.linear.x  = 0
                twist.linear.y  = 0
                twist.linear.z  = 0
                twist.angular.z = 0
                pub.publish(twist) 
            
            if remote_mode > 10:
                remotep = 0
                remote_mode = plan_mode = 0
                r_state = 'r'
            if plan_mode > 10:
                planp = 0
                remote_mode = plan_mode = 0
                r_state = 'p'
            
            if eem_mode > 20:
                eep_mode = eem_mode = 0
                eem = 0
                open_m = 100 - open_m
                if open_m == 100: ee.PumpState = 0 
                ee.MagState = open_m
                pub_ee.publish(ee)
            if eep_mode > 20:
                eep_mode = eem_mode = 0
                eep = 0 
                open_p = 100 - open_p
                if open_p == 100: ee.MagState = 0 
                ee.PumpState = open_p
                pub_ee.publish(ee)
        

            

cmd1 = []
cmd2 = []
cmd3 = []
NUM = 2

def plancallback(data):
    #global turn_cnt
    global cmd1
    global cmd2
    global cmd3
    if planEnable ==1:
        if len(cmd1) < NUM:
            cmd1 = [data.linear.x]*NUM
            cmd2 = [data.linear.y]*NUM
            cmd3 = [data.angular.z ]*NUM
        else:
            cmd1.remove(cmd1[0])
            cmd2.remove(cmd2[0])
            cmd3.remove(cmd3[0])
            cmd1.append(data.linear.x)
            cmd2.append(data.linear.y)
            cmd3.append(data.angular.z)

        twist.linear.x = np.mean(cmd1)
        twist.linear.y = np.mean(cmd2)
        twist.angular.z = np.mean(cmd3)
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        pub.publish(twist)  
        print("pp")


if __name__ == '__main__':

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    pub_ee = rospy.Publisher('endeffCmd',EndEffector,queue_size=1)

    turn_cnt = 0
    rospy.Subscriber("cmd_vel_remo", Twist, callback)
    rospy.Subscriber("cmd_vel_plan", Twist, plancallback)

    rospy.init_node('tele_key_pub')

    x = 0
    y = 0
    z = 0
    h = 0
    delta = 1
    open_p = 0
    open_m = 0
    chassis = 0
    endeff = 0

    try:
        #print(msg)
        rate = rospy.Rate(500) # 20hz
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        twist = Twist()
        ee = EndEffector()
        state = '2'     # 首选遥控器模式
        lastState = '0'


        while not rospy.is_shutdown():

            if state == '1':        # 键盘模式

                if lastState != state:
                    os.system('clear')
                    print(msg)
                    planEnable = 0
                    print("规划失能")
                    key = '2'
                
                else :
                    key = sys.stdin.read(1)

                lastState = state             

                if key == '0':
                    state = '2'

                if key == ' ':
                    state = '0'
                
                if key == '2':
                    x = 0.0
                    y = 0.0 
                    z = 0.0
                    h = 0.0
                    chassis = 1
                else:
                    if key == '3':
                        delta += 0.5
                    elif key == '1':
                        delta -= 0.5

                    if delta > 3:
                        delta = 3
                    elif delta <0:
                        delta = 0

                    if key == 'w' or key == 'W':
                        x = 0.3 * delta
                        z = 0
                        chassis = 1
                    if key == 's' or key == 'S':
                        x = -0.3 * delta
                        z = 0
                        chassis = 1
                    if key == 'a' or key == 'A':
                        y = 0.3 * delta
                        chassis = 1
                    if key == 'd' or key == 'D':
                        y = -0.3 * delta
                        chassis = 1
                    if key == 'q' or key == 'Q':
                        z = 0.2 * delta
                        chassis = 1
                    if key == 'e' or key == 'E':
                        z = -0.2 * delta
                        chassis = 1
                    if key == 'r' or key == 'R':
                        h = 0.01 * delta
                        chassis = 1
                    if key == 'f' or key == 'F':
                        h = -0.01 * delta
                        chassis = 1

                if key == 'l' or key == 'L':
                    planEnable = 1 - planEnable
                    os.system('clear')
                    print(msg)
                    if planEnable == 1:
                        print("规划使能")
                    else:
                        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                        pub.publish(twist)
                        ee.MagState = 0
                        ee.PumpState = 0
                        pub_ee.publish(ee)
                        print("规划失能")

                if key == 'm' or key == 'M':
                    open_m = 100 - open_m
                    ee.MagState = open_m
                    ee.PumpState = 0
                    endeff = 1
                elif key == 'p' or key == 'P':
                    open_p = 100 - open_p
                    ee.PumpState = open_p
                    ee.MagState = 0
                    endeff = 1

                if chassis == 1:
                    twist.linear.x = x; twist.linear.y = y; twist.linear.z = h
                    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = z 
                    pub.publish(twist)
                if endeff == 1:
                    pub_ee.publish(ee)

                endeff = 0
                chassis = 0

                rate.sleep()
            elif state == '2':      # 遥控器模式

                remoterEnable = 1
                if lastState != state:
                    os.system('clear')
                    print(msg_remote)
                    planEnable = 0
                lastState = state

                while 1:
                    key = sys.stdin.read(1)
                    if key == ' ':
                        state = '1'
                        remoterEnable = 0
                        break     
                    
            if state == '0':
                break
            

    except  Exception as e:
        print(e)

    finally:
        twist = Twist()
        ee = EndEffector()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        ee.MagState = 0
        ee.PumpState = 0
        pub_ee.publish(ee)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)


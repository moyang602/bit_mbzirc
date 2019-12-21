#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy,sys
import bit_vision.msg
import bit_motion.msg
from bit_vision.srv import *

import urx
import actionlib

global rob

#=============特征点与运动参数==========
deg2rad = 0.017453
FindFirePos = [-90.0, -80.0, -100.0, -80.0, 90.0, 0.0] # 火源探索位姿
preFightPos = [-90.0, -80.0, -100.0, -80.0, 90.0, 0.0] # 灭火准备位姿
for i in range(0,6):
    FindFirePos[i] = FindFirePos[i]*deg2rad
    preFightPos[i] = preFightPos[i]*deg2rad


v = 0.05*4    # 机械臂关节运动速度
a = 0.3       # 机械臂关节运动加速度

HandEye = 1   
StereoEye = 2

SUCCESS = 1
FAIL_VISION = 2
FAIL_ERROR = 3

### 视觉检测客户端
def GetFireVisionData_client(cameraUsed):
    rospy.wait_for_service('GetFireVisionData')
    try:
        get_vision_data = rospy.ServiceProxy('GetFireVisionData',FirePosition)
        respl = get_vision_data(cameraUsed)
        return respl
    except rospy.ServiceException, e:
        print "Service GetFireVisionData call failed: %s"%e

class fight_fire_act(object):
    _feedback = bit_motion.msg.fightfireFeedback()
    _result = bit_motion.msg.fightfireResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, bit_motion.msg.fightfireAction , execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("%s server ready! ", name)

    def show_tell(self, info):
        rospy.loginfo(info)
        self._feedback.feedback_state = info
        self._as.publish_feedback(self._feedback)

    def execute_cb(self, goal):
        self._result.finish_state = SUCCESS

        try:
            # 机械臂状态初始化
            rospy.loginfo("Start fight fire motion")
            pose = rob.getl()
            print("Initial end pose is ", pose)
            JointAngle = rob.getj()
            print("Initial joint angle is ", JointAngle)

            # 机械臂移动至火源探索位姿
            rob.movej(FindFirePos,acc=a, vel=3*v,wait=True)
            self.show_tell("UR arrived FindFire position")
            JointAngle = rob.getj()
            #--------- 机械臂移动直到找到火源 -------#
            offset = [-30*deg2rad, 0*deg2rad, 30**deg2rad]
            count = 0
            while True:
                MovePos = JointAngle
                MovePos[0] = MovePos[0]+offset[count%3]
                count = count+1
                rob.movej(MovePos,acc=a, vel=1*v,wait=True)
                rospy.sleep(0.5)
                VisionData = GetFireVisionData_client(HandEye)
                if VisionData.flag and count>10:        
                    break
            
            #--------- 机械臂移动直到火源在图像中心线 -------#
            rospy.sleep(0.5)
            while True:
                VisionData = GetFireVisionData_client(HandEye)
                if VisionData.flag:
                    deltax = VisionData.FirePos.point.x
                    if deltax<5:
                        break
                    JointAngle = rob.getj()
                    MovePos = JointAngle
                    MovePos[0] = MovePos[0]+deltax*0.01     # Todo
                    rob.movej(MovePos,acc=a, vel=1*v,wait=True)
                else:
                    break

            #--------- 控制小车移动相应角度，使机械臂朝向正前方 -------#
            JointAngle = rob.getj()
            CarAngle = JointAngle[0]
            # Todo
            # 机械臂移动至灭火准备位姿
            rob.movej(preFightPos,acc=a, vel=1*v,wait=True)

            #--------- 控制机械臂移动，直到火源在画面竖直方向中心 -------#
            rospy.sleep(0.5)
            while True:
                VisionData = GetFireVisionData_client(HandEye)
                if VisionData.flag:
                    deltay = VisionData.FirePos.point.y
                    if deltay<5:
                        break
                    pose = rob.getl()
                    pose[4] = pose[4] + deltay*0.01        # Todo
                    rob.movel(pose, acc=a, vel=v, wait=True)
                else:
                    break

            #--------- 控制车臂同时移动，直到到达火源位置 -------#
            rospy.sleep(0.5)
            while True:
                VisionData = GetFireVisionData_client(HandEye)
                if VisionData.flag:
                    deltax = VisionData.FirePos.point.x
                    deltay = VisionData.FirePos.point.y
                    if deltay<5 and deltay < 5:
                        break
                    pose = rob.getl()
                    pose[4] = pose[4] + deltay*0.01        # Todo
                    rob.movel(pose, acc=a, vel=v, wait=True)
                    # 水平方向通过小车旋转实现
                    CarAngle = deltax*0.01                 # Todo
                else:
                    break


            # 操作末端 开始喷水
            '''
            rospy.sleep(1.0)
            ee.MagState = 100
            pub_ee.publish(ee)
            rospy.sleep(1.0)
            '''

            # 喷5s水
            rospy.sleep(5.0)

            # 操作末端 停止
            '''
            rospy.sleep(1.0)
            ee.MagState = 100
            pub_ee.publish(ee)
            rospy.sleep(1.0)
            '''

            # 机械臂复原
            rob.movej(preFightPos,acc=a, vel=2*v,wait=True)
            self.show_tell("arrived pre-FightFire position, finished")

        except Exception as e:
            print("error", e)
            self._result.finish_state = FAIL_ERROR
            self._as.set_aborted(self._result)
            rob.stopl()
        finally:
            if self._result.finish_state == SUCCESS:
                self._as.set_succeeded(self._result)
            else:
                self._as.set_aborted(self._result)

if __name__ == '__main__':

    rospy.init_node('FightFire_motion_node', anonymous = False)

    normal = 0
    while(not rospy.is_shutdown()):
        try :
            global rob
            rob = urx.Robot("192.168.50.60",use_rt = True) 
            normal = 1
            rospy.loginfo('UR controller connected')
            # Todo 根据实际末端负载与工具中心设置
            rob.set_tcp((0, 0, 0.074, 0, 0, 0))     #TASK2 参数 m,rad     todo 待修改
            rob.set_payload(0.96, (0.004, -0.042, 0.011))

            fight_fire_act("FightFireAction") 
            rospy.spin() 
        except Exception as e:
            print("error", e)
            rospy.sleep(1)
        finally:
            if normal == 1:
                rob.stopl()
                rob.close()
    sys.exit(0)
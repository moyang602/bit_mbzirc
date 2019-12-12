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
preFightPos = (-1.571, -1.396, -1.745, -1.396, 1.571, 0.0) # [-90.0, -80.0, -100.0, -80.0, 90.0, 0.0]灭火准备位姿

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
            initj = rob.getj()
            print("Initial joint angle is ", initj)

            # 机械臂移动至灭火准备位姿
            rob.movej(preFightPos,acc=a, vel=3*v,wait=True)
            self.show_tell("UR arrived pre-FightFire position")

            # 视觉搜索火源位置
            rospy.sleep(0.5)
            while True:
                VisionData = GetFireVisionData_client(HandEye)
                if VisionData.flag:
                    break

            self.show_tell("Got Fire position results")
            # 判断是否在工作空间，不是任务返回失败    
            # 工作空间检测函数  Todo
            x = VisionData.FirePos.point.x
            y = VisionData.FirePos.point.y
            dist = (x**2 + y**2)**0.5
            '''
            if dist>0.668 or x>0.200 or x<-0.200 and y <-0.250: # 待矫正
                self.show_tell("Brick position is out of workspace")
                return
            '''

            # 得到识别结果如下
            pose[0] = VisionData.FirePos.point.x
            pose[1] = VisionData.FirePos.point.y
            pose[2] = floorHeight_base + 1.0     
            pose[3] = 0.0
            # 下两个坐标使其垂直于地面Brick remembered
            pose[4] = -pi 
            pose[5] = 0.0 
            rob.movel(pose, acc=a, vel=v, wait=True)

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

            rob.movej(prePutPos,acc=a, vel=2*v,wait=True)
            self.show_tell("arrived pre-Put position, finished")

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

    # pub_ee = rospy.Publisher('endeffCmd',EndEffector,queue_size=1)

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
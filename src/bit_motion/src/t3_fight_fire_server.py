#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy,sys
import bit_vision.msg
import bit_motion.msg
from bit_vision.srv import *

from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionFeedback
from bit_control_tool.msg import EndEffector

import tf
import urx
import actionlib
import math
global rob
global ee

#=============特征点与运动参数==========
deg2rad = 0.017453
FindFirePos = [-90.0, -60.0, -120.0, -60.0, 90.0, 0.0] # 火源探索位姿
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

global cancel_id

### 视觉检测客户端
def GetFireVisionData_client(cameraUsed):
    rospy.wait_for_service('GetFireVisionData')
    try:
        get_vision_data = rospy.ServiceProxy('GetFireVisionData',FirePosition)
        respl = get_vision_data(cameraUsed)
        return respl
    except rospy.ServiceException, e:
        print "Service GetFireVisionData call failed: %s"%e

# 获取Movebase ID
def move_base_feedback(a):
    global cancel_id
    cancel_id = a.status.goal_id


# 小车移动
def CarMove(x,y,theta,frame_id='car_link'):
    this_target = PoseStamped()
    this_target.header.frame_id = frame_id
    this_target.header.stamp = rospy.Time.now()
    this_target.pose.position.x = x
    this_target.pose.position.y = y
    this_target.pose.position.z = 0.0
    this_target.pose.orientation = tf.createQuaternionMsgFromYaw(theta)
    # 发布位置
    simp_goal_pub.publish(this_target)

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

            # 小车移动相关话题初始化
            simp_goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
            simp_goal_sub = rospy.Subscriber("move_base/feedback",MoveBaseActionFeedback,move_base_feedback)
            simp_cancel = rospy.Publisher('/move_base/cancel',GoalID,1)

            # 末端水枪话题初始化
            pub_ee = rospy.Publisher('endeffCmd',EndEffector,queue_size=1)
            ee = EndEffector()

            ## —————————————————开始灭火作业———————————————##
            # 1. 机械臂移动至灭火准备位姿
            rob.movej(preFightPos,acc=a, vel=3*v,wait=True)
            self.show_tell("UR arrived pre-FightFire position")
            JointAngle = rob.getj()

            # 2.1 小车开始自由移动
            CarMove(2.0, 0.0, 0.0*deg2rad)
    
            # 2.2 机械臂移动直到找到火源
            offset = [0*deg2rad, -20*deg2rad, 0*deg2rad, 20*deg2rad]
            count = 0   #搜索次数
            while True:
                MovePos = JointAngle
                MovePos[0] = JointAngle[0]+offset[count%4]
                count = count+1
                rob.movej(MovePos,acc=a, vel=1*v,wait=True)
                rospy.sleep(0.5)
                VisionData = GetFireVisionData_client(HandEye)
                if VisionData.flag or count>20:  
                    simp_cancel.publish(cancel_id)      # 取消运动
                    break
            
            if VisionData.flag:
                self.show_tell("Fire pos has been found")
            else:
                self.show_tell("Timeout, can't find Fire pos")

            # 3. 机械臂移动直到火源在图像竖直中心线
            rospy.sleep(0.5)
            while True:
                VisionData = GetFireVisionData_client(HandEye)
                if VisionData.flag:
                    deltax = VisionData.FirePos.point.x
                    if math.fabs(deltax)<5:
                        self.show_tell("robot aligned the fire")
                        break
                    MovePos = [deltax*0.2*deg2rad,0,0,0,0,0]
                    rob.movej(MovePos,acc=1*a, vel=0.3*v,wait=False,relative=True)
                    rospy.sleep(2)
                else:
                    break

            # 4. 控制小车移动相应角度，使机械臂朝向正前方
            JointAngle = rob.getj()
            CarMove(0.0, 0.0, JointAngle)                   # 小车转动相应角度
            rob.movej(FindFirePos,acc=a, vel=1*v,wait=True) # 机械臂移动至灭火准备位姿

            # 5. 控制车臂同时移动，直到到达火源位置
            rospy.sleep(0.5)
            while True:
                VisionData = GetFireVisionData_client(HandEye)
                if VisionData.flag:
                    deltax = VisionData.FirePos.point.x
                    deltay = VisionData.FirePos.point.y
                    pose = rob.getl()
                    if math.fabs(pose + pi)<3*deg2rad:          # 当机械臂末端接近竖直向下时停止
                        simp_cancel.publish(cancel_id)
                        break
                    if math.fabs(deltax)<5:
                        deltax = 0
                    if math.fabs(deltay)<5:
                        deltay = 0
                    CarMove(0.2, 0.0, deltax*0.01*deg2rad)      # 偏航角通过小车调整
                    if pose[0]<-0.5:                            # 机械臂一边移动一边往前伸
                        pose = [0, 0 ,0,-deltay*0.2*deg2rad,0,0]      # 俯仰角通过机械臂调整
                    else:
                        pose = [0, -0.1 ,0,-deltay*0.2*deg2rad,0,0]      # 俯仰角通过机械臂调整
                    rob.movel(pose, acc=a, vel=0.3*v, wait=False,relative=True)
                    rospy.sleep(0.5)
                else:
                    break

            # 6. 机械臂变为竖直向下状态
            pose = rob.getl()
            pose[3] = 0
            pose[4] = -pi
            pose[5] = 0
            rob.movel(pose, acc=a, vel=1*v, wait=True,relative=False)

            # 7. 控制机械臂水平移动，对准火源
            rospy.sleep(0.5)
            while True:
                VisionData = GetFireVisionData_client(HandEye)
                if VisionData.flag:
                    deltax = VisionData.FirePos.point.x + 0.0       # Todo 标定水枪位置
                    deltay = VisionData.FirePos.point.y + 0.0
                    if math.fabs(deltax)<5 and math.fabs(deltay)<5:
                        break
                    pose = [deltax*0.001, -deltay*0.001,0,0,0,0]      
                    rob.movel(pose, acc=a, vel=0.3*v, wait=False,relative=True)
                    rospy.sleep(0.5)
                else:
                    break

            # 8. 开始灭火
            '''
            # 操作末端 开始喷水
            
            rospy.sleep(1.0)
            ee.PumpState = 100
            pub_ee.publish(ee)
            rospy.sleep(1.0)
            

            # 喷5s水
            rospy.sleep(5.0)

            # 操作末端 停止
            
            rospy.sleep(1.0)
            ee.PumpState = 100
            pub_ee.publish(ee)
            rospy.sleep(1.0)
            '''

            rospy.sleep(2.0)
            # 9. 机械臂复原
            rob.movej(FindFirePos,acc=a, vel=2*v,wait=True)
            self.show_tell("finished")

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
            rob.set_tcp((0, 0, 0.0, 0, 0, 0))     #TASK3 参数 m,rad     todo 待修改
            rob.set_payload(0.0, (0.000, -0.000, 0.000))

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
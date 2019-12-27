#!/usr/bin/env python
# -*- coding: UTF-8 -*-

'''ur_force ROS Node'''
import rospy
from math import pi
import math
import time,sys,logging,os
import numpy as np
import threading

import roslib
import rospy
import actionlib
import urx
import tf

from bit_control_msgs.msg import EndEffector
from bit_control_msgs.msg import heightNow

from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
# from move_base_msgs.msg import MoveBaseFeedback

import bit_task_msgs.msg

import bit_vision_msgs.srv
from bit_control_msgs.srv import SetHeight

if sys.version_info[0] < 3:  # support python v2
    input = raw_input
do_wait = True

#=============特征点与运动参数==========
deg2rad = 0.017453
prePickPos = (-1.571, -1.396, -1.745, -1.396, 1.571, 0.0) # [-90.0, -80.0, -100.0, -80.0, 90.0, 0.0]取砖准备位姿
upHeadPos = (-1.57, -1.57, 0, 0, 1.57, 0)
prePutPos = (-1.916, -1.367, 1.621, 1.257, 1.549, -0.344) #(-1.57,-1.29, 1.4, 1.4, 1.57, 0)  # 末端位姿 [0 400 300 0 -180 0]
lookForwardPos = (-1.57, -1.57, -1.57, 0, 1.57, 0)
lookDownPos = (-1.571, -1.396, -1.745, -1.396, 1.571, 0.0)  # 暂时与prePickPos相同
# floorHeight_base = -0.710  # 初始状态机械臂基座离地710mm
# CarHeight_base = -0.140  # 初始状态机械臂基座离车表面140mm
# floorHeight_base = -0.375 - 0.330
# CarHeight_base = 0.180 - 0.330 

global floorHeight_base
global CarHeight_base

posSequence = [] # 随着摆放的过程不断填充这个list来把位置记录下来

# 机械臂移动的速度和加速度  
v = 0.05*4
a = 0.3

# 可调用的视觉的接口
GetBrickPos = 1   
GetBrickAngle = 2
GetPutPos = 3
GetPutAngle = 4
GetLPose = 5
GetBrickPos_only = 6
GetBrickLocation = 7

SUCCESS = 1
FAIL_VISION = 2
FAIL_ERROR = 3


TASK_GET = 0
TASK_BUILD = 1
TASK_LOOK_FORWARD = 2
TASK_LOOK_DIRECT_DOWN = 3

ON = 100
OFF = 0

global rob
global force 
global ee
global cancel_id
global status
FinishFlag = 0

def forcecallback(data):
    '''ur_force Callback Function'''
    global force
    force = data.wrench.force.x**2 + data.wrench.force.y**2 + data.wrench.force.z**2
    force = force ** 0.5

def heightcallback(data):
    '''ur_force Callback Function'''
    height = data.x 
    global floorHeight_base
    global CarHeight_base
    floorHeight_base = -0.3737 - height/1000
    CarHeight_base = 0.185 - height /1000

def wait():
    ''' used to debug move one by one step '''
    if do_wait:
        print("Click enter to continue")
        input()


def GetVisionData_client(ProcAlgorithm, BrickType):
    rospy.wait_for_service('GetVisionData')
    try:
        get_vision_data = rospy.ServiceProxy('GetVisionData',VisionProc)
        respl = get_vision_data(ProcAlgorithm, BrickType)
        return respl.VisionData
    except rospy.ServiceException, e:
        print "Service GetVisionData call failed: %s"%e


def Laser_client(LaserAlgoritm):
    rospy.wait_for_service('LaserDeal')
    try:
        get_vision_data = rospy.ServiceProxy('LaserDeal',VisionProc)
        respl = get_vision_data(LaserAlgoritm)
        return respl.VisionData
    except rospy.ServiceException, e:
        print "Service GetVisionData call failed: %s"%e


def move_base_feedback(a):
    global cancel_id
    global status
    cancel_id = a.status_list[0].goal_id.id
    status = a.status_list[0].status

# 小车移动
def CarMove(x,y,theta,frame_id="car_link",wait = False):

    global status
    this_target = PoseStamped()
    this_target.header.frame_id = frame_id
    this_target.header.stamp = rospy.Time.now()
    this_target.pose.position.x = x
    this_target.pose.position.y = y
    this_target.pose.position.z = 0.0
    ori = tf.transformations.quaternion_from_euler(0,0,theta,'ryxz')
    this_target.pose.orientation.x = ori[0]
    this_target.pose.orientation.y = ori[1]
    this_target.pose.orientation.z = ori[2]
    this_target.pose.orientation.w = ori[3]

    # 发布位置
    simp_goal_pub.publish(this_target)
    rospy.sleep(0.1)

    if wait:
        while status != GoalStatus.SUCCEEDED:
            if status == GoalStatus.ABORTED:     # 如果规划器失败的处理
                simp_goal_pub.publish(this_target)
            pass



## ================================================== ##
## ================================================== ##
##                    Todo List                       ##
##                                                    ##
## 1\ 调用视觉的服务进行砖块精确定位                       ##
## 2\ 根据砖块序号来确定放在车上的位置，并记录到posSequence  ##
## 3\ 建筑任务的欲放置位置需要根据砖块x,y确定               ##
## 4\ 建筑任务的放置砖需结合手眼完成                       ##
##                                                    ##
## ================================================== ##
## ================================================== ##

class pick_put_act(object):
    _feedback = bit_task_msgs.msg.buildingFeedback()
    _result = bit_task_msgs.msg.buildingResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, bit_task_msgs.msg.buildingAction , execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("%s server ready! ", name)

        rospy.wait_for_service('Setheight')
        set_height = rospy.ServiceProxy('Setheight',SetHeight)

    def show_tell(self, info):
        rospy.loginfo(info)
        self._feedback.task_feedback = info
        self._as.publish_feedback(self._feedback)

    def execute_cb(self, goal):
        # helper variables
        # r = rospy.Rate(1)
        self._result.finish_state = SUCCESS
        try: 
            # 开始臂车运动
            rospy.loginfo("begining")
            pose = rob.getl()
            print("Initial end pose is ", pose)
            initj = rob.getj()
            print("Initial joint angle is ", initj)

            # 机械臂移动至观察砖堆准备位姿
            rob.movej(lookForwardPos, acc=a, vel=3*v,wait=True)
            # 调用激光雷达找位置，转一个角度
            # CarMove(0,0,theta)
            # -------------------- 找砖 --------------------- #
            '''
            # 找砖：视觉引导，雷达微调 CarMove() tf.TransformListener.lookupTransform()
            while True:         # Todo 避免进入死循环
                VisionData = GetVisionData_client(GetBrickLocation, "O")    # 数据是在base_link坐标系下的
                if VisionData.Flag:     # 能够看到
                    theta = math.atan(VisionData.Pose.position.x,-VisionData.Pose.position.y)
                    CarMove(VisionData.Pose.position.x,VisionData.Pose.position.y,theta,"car_link")
                    # 调用激光雷达检测的服务
                    if 1:# 激光雷达检测到在范围内，并且已经到达
                        break
                else:
                    # 调用激光雷达找位置，来移动
                    CarMove(0,0,theta)
            '''
            CarMove(2.0,0,0,frame_id="car_link",wait=True)
            # ----------- 到达砖堆橙色处，开始取砖 --------------#
            wait()
            '''
            brickIndex = 0
            while brickIndex < goal.Num:
                # 调用激光雷达检测的服务  goal.bricks[brickIndex].type

                for attempt in range(0,3):  # 最大尝试次数30
                    result_ = self.goGetBrick(goal.bricks[brickIndex])
                    if result_ == SUCCESS:
                        # 记录当前点为这种砖的位置
                        self.show_tell("finished !PICK! brick %d,go get next" %brickIndex)
                        brickIndex = brickIndex + 1     # 成功了就取下一块  
                        break
                      
            self.show_tell("Got all bricks, go to build")
            
            # --------------- 寻找L架 ------------------------#
            # 机械臂移动至观察L架位姿
            rob.movej(lookDownPos, acc=a, vel=3*v,wait=True)
            out = 0
            while True:         # 遍历
                VisionData = GetVisionData_client(GetLPose, "none")
                while VisionData.Flag > 0:
                    while VisionData.Flag > 1:
                        while VisionData.Flag > 2:
                            tf_OrignOnMap = PoseCal(tf_BaseOnMap_rot,tf_BaseOnMap_trans,VisionData.Pose.orientation,VisionData.Pose.posotion):
                            # TODO 记录
                            out = 1
                            break
                        if out:
                            break
                        CarMove(0, 0.1, 0)
                        VisionData = GetVisionData_client(GetLPose, "none")
                    if out:
                        break
                    theta = VisionData.Pose.orientation.z        
                    CarMove(0,0,VisionData,"car_link",wait=True)     # 首先转正
                    CarMove(0, -0.1, 0)
                    VisionData = GetVisionData_client(GetLPose, "none")
                if out:
                    break
                # TODO 遍历运动
                VisionData = GetVisionData_client(GetLPose, "none")

            # --------------------找到L架，开始搭建 ------------#

            brickIndex = 0
            while brickIndex < goal.Num:
                
                tf_BrickOnOrign = tf.TransformListener.fromTranslationRotation((goal.bricks[brickIndex].x,goal.bricks[brickIndex].y,0),(0,0,0,1))
                if goal.bricks[brickIndex].x == 0.0:
                    rot_ = tf.transformations.quaternion_from_euler(0,0,0)
                    tf_CarOnBrick = tf.TransformListener.fromTranslationRotation((-0.5,0,0),rot_)      # 砖外0.5m
                elif goal.bricks[brickIndex].y == 0.0:
                    rot_ = tf.transformations.quaternion_from_euler(0,0,3.1415926/2)
                    tf_CarOnBrick = tf.TransformListener.fromTranslationRotation((0,-0.5,0),rot_)      # 砖外0.5m
                target_tf = tf_OrignOnMap * tf_BrickOnOrign * tf_CarOnBrick
                Rotation = tf.transformations.euler_from_quaternion(target_tf.transform.rotation)

                # 动态配置
                CarMove(target_tf.transform.translation.x,target_tf.transform.translation.y,Rotation[3],frame_id="map",wait = True )
                self.show_tell("Got the %d brick place"% brickIndex)

                for attempt in range(0,3):  # 最大尝试次数30
                    result_ = self.goLPutBrick(goal.bricks[brickIndex])
                    if result_ == SUCCESS:
                        # 记录当前点为这种砖的位置
                        self.show_tell("finished !BUILD! brick %d,go get next" %brickIndex)
                        brickIndex = brickIndex + 1     # 成功了就取下一块 
                        break 
            self.show_tell("Build all bricks")
            global FinishFlag
            FinishFlag = 1

            '''

        except Exception as e:
            print("error", e)
            self._result.finish_state = FAIL_ERROR
            self._as.set_aborted(self._result)
            rob.stopl()
            # rob = urx.Robot("192.168.50.60",use_rt = True)
            # rob.set_tcp((0, 0, 0, 0, 0, 0))
            # rob.set_payload(0.0, (0, 0, 0))
        finally:
            if self._result.finish_state == SUCCESS:
                self._as.set_succeeded(self._result)
            else:
                self._as.set_aborted(self._result)


    def forceDown(distance):
        rob.movel([0.0, 0.0, -distance, 0.0, 0.0, 0.0], acc=a, vel=v*0.1, wait=False, relative=True)     # 相对运动

        _force_prenvent_wrongdata_ = 0
        while force < 15:
            _force_prenvent_wrongdata_ += 1
            if _force_prenvent_wrongdata_ >150: 
                _force_prenvent_wrongdata_ = 150 
            rospy.sleep(0.002)
            if  _force_prenvent_wrongdata_ >100 and ( not rob.is_program_running() ):
                rospy.loginfo("did not contact")
                break
        rob.stopl()
        self.show_tell("Reached Block")
    

    def turnEndEffect(state):
        # 操作末端
        # rospy.sleep(1.0)
        ee.MagState = state
        pub_ee.publish(ee)
        rospy.sleep(1.0)

    def goGetBrick(self, goal):

        # ----------- 已经到达橙色砖堆面前---------------- #

        # 机械臂移动至取砖准备位姿
        rob.movej(prePickPos,acc=a, vel=3*v,wait=True)
        self.show_tell("arrived pre-pick position")

        # 视觉搜索目标砖块位置
        while True:         # Todo 避免进入死循环
            VisionData = GetVisionData_client(GetBrickPos, goal.type)
            if VisionData.Flag:
                self.show_tell("Got BrickPos results from Vision")
                # 判断是否在工作空间，不是则动车  
                x = VisionData.Pose.position.x
                y = VisionData.Pose.position.y
                dist = (x**2 + y**2)**0.5
                
                if dist>0.765 or dist<0.45 or x>0.3 or x<-0.3: # 待测试
                    self.show_tell("Brick position is out of workspace")
                    # 动车，再次调用激光雷达的动作
                else:   # 在工作空间，可以抓取
                    break
            
        self.show_tell("In the workspace，Ready to pick")
        wait()
        # 得到识别结果，平移到相机正对砖块上方0.5m（待确定）处
        pose[0] = VisionData.Pose.position.x-0.023      #使ZED正对砖块中心  0.023为zed与magnet偏移
        pose[1] = VisionData.Pose.position.y+0.142      #使ZED正对砖块中心  0.142为zed与magnet偏移
        pose[2] = VisionData.Pose.position.z + 0.5      # 移动到砖上方0.5m处， 偏移值待确定
        pose[3] = 0
        pose[4] = -pi   # 下两个坐标使其垂直于地面Brick remembered
        pose[5] = 0 
        rob.movel(pose, acc=a, vel=v, wait=True)

        # 视觉搜索目标砖块角度
        rospy.sleep(0.5)
        while True:         # Todo 避免进入死循环
            VisionData = GetVisionData_client(GetBrickAngle, goal.type)
            if VisionData.Flag:
                break
        theta = VisionData.Pose.orientation.z      # 得到识别结果
        self.show_tell("Got Brick_Angle results")

        # 得到识别结果，下降0.4m，旋转角度
        rob.translate((0,0,-0.4), acc=a, vel=v, wait=True)
        # rospy.sleep(0.5)
        rob.movej([0,0,0,0,0,theta],acc=a, vel=1*v,wait=True, relative=True)

        self.show_tell("Arrived brick up 0.1m position pependicular to brick")

        self.forceDown(0.3)             # 伪力控下落 0.3m
        self.turnEndEffect(ON)          # 操作末端
        rob.translate((0,0,0.3), acc=a, vel=v, wait=True)       # 先提起，后转正

        # 抬升到抓取准备的位置
        rob.movej(prePickPos,acc=a, vel=1*v,wait=True)
        self.show_tell("Got brick and arrived pre-Pick position" )

        # TODO:需要进行一系列的操作，来放置砖块 @ 周权 % goal.goal_brick.Sequence
        if goal.goal_brick.type == "O":
            # delta = (0.0, -0.1+goal.goal_brick.Sequence * 0.21, 0, 0, 0, 0)
            # rob.movel(delta, acc=a, vel=v,wait=True, relative=True )

            # pose = rob.getl()
            # pose[2] = CarHeight_base + 0.32
            # rob.movel(pose, acc=a, vel=v,wait=True)
            pass
        else:
            pass
            # delta = (0.0, -0.1+goal.goal_brick.Sequence * 0.21, 0, 0, 0, 0)
            # rob.movel(delta, acc=a, vel=v,wait=True, relative=True )

            # pose = rob.getl()
            # pose[2] = CarHeight_base + 0.32
            # rob.movel(pose, acc=a, vel=v,wait=True)
            pass

        self.forceDown(0.32)        # 伪力控放置status
        self.show_tell("Put Down")        
        self.turnEndEffect(OFF)     # 释放末端

        # 移动回放置处
        rob.translate((0, 0, 0.05), acc=a, vel=v*0.3, wait=True)
        rob.movej(prePutPos,acc=a, vel=2*v,wait=True)
        return SUCCESS

    def goLPutBrick(self,goal):

        set_height(400)     # 可能需要提高
        # TODO:需要进行一系列的操作，来放置砖块 @ 周权 % goal.goal_brick.Sequence
        if goal.goal_brick.type == "O":
            # delta = (0.0, -0.1+goal.goal_brick.Sequence * 0.21, 0, 0, 0, 0)
            # rob.movel(delta, acc=a, vel=v,wait=True, relative=True )

            # pose = rob.getl()
            # pose[2] = CarHeight_base + 0.32
            # rob.movel(pose, acc=a, vel=v,wait=True)
            pass
        else:
            pass
            # delta = (0.0, -0.1+goal.goal_brick.Sequence * 0.21, 0, 0, 0, 0)
            # rob.movel(delta, acc=a, vel=v,wait=True, relative=True )

            # pose = rob.getl()
            # pose[2] = CarHeight_base + 0.32
            # rob.movel(pose, acc=a, vel=v,wait=True)
            pass
        self.show_tell("arrived pre-pick position")

        rospy.sleep(0.5)
        while True:         # Todo 避免进入死循环
            VisionData = GetVisionData_client(GetBrickPos, goal.type)
            if VisionData.Flag:
                break

        # 得到识别结果，移动到砖块上方0.1，平移
        pose[0] = VisionData.Pose.position.x
        pose[1] = VisionData.Pose.position.y
        pose[2] = VisionData.Pose.position.z + 0.1     # 0.1是离砖10cm
        pose[3] = 0  # 下两个坐标使其垂直于地面Brick remembered
        pose[4] = -pi 
        pose[5] = 0
        rob.movel(pose, acc=a, vel=v, wait=True)
        # rospy.sleep(0.5)

        self.forceDown(0.2)         # 伪力控下落
        self.turnEndEffect(ON)      # 操作末端
        
        # 提起 或是升降台提升
        rob.translate((0,0,0.25), acc=a, vel=v, wait=True)

        # 根据建筑物位置改变升降台到高度320-670cm
        # set_height(400)

        rob.movej(prePickPos,acc=a, vel=3*v,wait=True)     
        self.show_tell("arrived pre-Build position")
            
        # 配合手眼移动到摆放的位置
        # 调用视觉
        while True:         # Todo 避免进入死循环
            VisionData = GetVisionData_client(GetPutPos, goal.type)
            if VisionData.Flag:
                break

        pose[0] = VisionData.Pose.position.x
        pose[1] = VisionData.Pose.position.y
        pose[2] = floorHeight_base +  goal.z + 0.1     # 0.25是离车表面25cm
        pose[3] = 0     # 下两个坐标使其垂直于地面Brick remembered
        pose[4] = -pi 
        pose[5] = 0
        rob.movel(pose, acc=a, vel=v, wait=True)

        self.forceDown(0.15)
        self.turnEndEffect(OFF)

        rob.movej(prePickPos,acc=a, vel=3*v,wait=True)
        return SUCCESS

def PoseCal(quat1,trans1,quat2,trans2):
    t1 = tf.transformations.quaternion_matrix(quat1)
    t1[0][3] = trans1[0]
    t1[1][3] = trans1[1]
    t1[2][3] = trans1[2]
    t2 = tf.transformations.quaternion_matrix(quat2)
    t2[0][3] = trans2[0]
    t2[1][3] = trans2[1]
    t2[2][3] = trans2[2]

    t3 = np.dot(t2,t1)
    return t3


if __name__ == '__main__':

#===================定义一些基本的对象======================#

    rospy.init_node('pickputAction', anonymous = False)
    pub_ee = rospy.Publisher('endeffCmd',EndEffector,queue_size=1)
    rospy.Subscriber("/wrench", WrenchStamped, forcecallback)
    rospy.Subscriber("/heightNow", heightNow, heightcallback)

    # 小车移动相关话题初始化
    simp_goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
    simp_goal_sub = rospy.Subscriber("move_base/status",GoalStatusArray,move_base_feedback)
    simp_cancel = rospy.Publisher('/move_base/cancel',GoalID,queue_size=1)

    ee = EndEffector()
    normal = 0
  
    while(not rospy.is_shutdown()):
        try :
            global rob
            rob = urx.Robot("192.168.50.60",use_rt = True) 
            normal = 1
            rospy.loginfo('robot ok')
            # Todo 根据实际末端负载与工具中心设置
            rob.set_tcp((0, 0, 0.035, 0, 0, 0))     #TASK2 参数 m,rad
            rob.set_payload(0.76, (0.011, -0.042, 0.003))

            pick_put_act("ugv_building")     # rospy.get_name())
            
            # TF 计算
            listener = tf.TransformListener()
            while not FinishFlag:
                try:
                    (tf_BaseOnMap_trans,tf_BaseOnMap_rot) = listener.lookupTransform("base_link","map", rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
        except:
            time.sleep(2.0)
        finally:
            if normal == 1:
                rob.stopl()
                rob.close()
    sys.exit(0)
    
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

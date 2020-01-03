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
# takePos = (-1.916, -1.367, 1.621, 1.257, 1.549, -0.344) #(-1.57,-1.29, 1.4, 1.4, 1.57, 0)  # 末端位姿 [0 400 300 0 -180 0]
lookForwardPos = (-1.57, -1.57, -1.57, 0, 1.57, 0)
lookDownPos = (-1.571, -1.396, -1.745, -1.396, 1.571, 0.0)  # 暂时与pickPos相同

takePos = list(np.array([-90.0,-96.0,-42.84,-131.15,90.0,0.0])*deg2rad)     # Take动作开始位置
onCarStartPos = list(np.array([-232.52,-66.49,-62.19,-141.32,90.30,34.13])*deg2rad) # 车上操作位置1

putPos = list(np.array([-90.0,-96.0,-42.84,-131.15,90.0,0.0])*deg2rad)      # Put动作开始位置
onCarReadyPos = list(np.array([-249.93,-86.05,-53.04,-130.93,90.36,19.77])*deg2rad) # 车上操作位置2

buildPos = list(np.array([-90.0,-96.0,-42.84,-131.15,90.0,0.0])*deg2rad)      # Build动作开始位置

pickPos = (-1.571, -1.396, -1.745, -1.396, 1.571, 0.0) # 取砖准备位姿 [-90.0, -80.0, -100.0, -80.0, 90.0, 0.0]

RGB = (
    [ 0.3, 0.42, 0.2, pi, 0, 0]    # 红4
    [-0.3, 0.42, 0.2, pi, 0, 0],   # 红3
    [ 0.3, 0.22, 0.2, pi, 0, 0],   # 红2
    [-0.3, 0.22, 0.2, pi, 0, 0],   # 红1
    [ 0.0, 0.42, 0.4, pi, 0, 0],   # 兰1
    [ 0.3, 0.22, 0.4, pi, 0, 0],   # 绿2
    [-0.3, 0.22, 0.4, pi, 0, 0],   # 绿1
)
ORG = (
    [0.0, 0.42, 0.2, pi, 0, 0],     # 橙0
    [0.0, 0.22, 0.2, pi, 0, 0],     # 橙1
    [0.0, 0.42, 0.4, pi, 0, 0],     # 橙2
    [0.0, 0.22, 0.4, pi, 0, 0],     # 橙3
    [0.0, 0.42, 0.6, pi, 0, 0],     # 橙4
    [0.0, 0.22, 0.6, pi, 0, 0],     # 橙5
)
tolerance = 0.02    # 米
RGB_load = (
    [ 0.3, 0.37 + tolerance, 0.2, pi, 0, 0],   # 红2
    [-0.3, 0.37 + tolerance, 0.2, pi, 0, 0],   # 红1
    [ 0.3, 0.57 + 2*tolerance, 0.2, pi, 0, 0],   # 红4
    [-0.3, 0.57 + 2*tolerance, 0.2, pi, 0, 0],   # 红3
    [ 0.3 + 0.5*tolerance, 0.37 + tolerance, 0.4, pi, 0, 0],   # 绿2
    [-0.3 - 0.5*tolerance, 0.37 + tolerance, 0.4, pi, 0, 0],   # 绿1
    [ 0.0, 0.57 + 2*tolerance, 0.4, pi, 0, 0],   # 兰1
)
ORG_load = (
    [0.0, 0.37 + tolerance, 0.2, pi, 0, 0],
    [0.0, 0.57 + 2*tolerance, 0.2, pi, 0, 0],
    [0.0, 0.37 + tolerance, 0.4, pi, 0, 0],
    [0.0, 0.57 + 2*tolerance, 0.4, pi, 0, 0],
    [0.0, 0.37 + tolerance, 0.6, pi, 0, 0],
    [0.0, 0.57 + 2*tolerance, 0.6, pi, 0, 0],
)

global floorHeight_base     # 使用时直接将期望值加上这个值就可以发给机械臂
global CarHeight_base

# 机械臂移动的速度和加速度  
v = 0.05*4
a = 0.3

# 可调用的视觉的接口
GetBrickPos=1   
GetBrickLoc=2
GetPutPos=  3
GetPutAngle=4
GetLPose=   5

SUCCESS = 1
FAIL = 0


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
    # rospy.sleep(0.1)  更换为等待任务开始
    while status != GoalStatus.ACTIVE:
        pass

    if wait:
        while status != GoalStatus.SUCCEEDED:
            if status == GoalStatus.ABORTED:     # TODO 如果规划器失败的处理
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
            #================ 0. 准备 ===================#
            rospy.loginfo("begining")
            pose = rob.getl()
            print("Initial end pose is ", pose)
            initj = rob.getj()
            print("Initial joint angle is ", initj)

            #================ 1. 寻找砖堆 ================#
            # 1.1 机械臂移动至观察砖堆准备位姿
            rob.movej(lookForwardPos, acc=a, vel=3*v,wait=True)

            # 1.2 小车遍历场地， 基于视觉寻找砖堆
            '''
            while True:         # Todo 避免进入死循环
                VisionData = GetVisionData_client(GetBrickLoc, "O")    # 数据是在base_link坐标系下的
                if VisionData.Flag:     # 能够看到
                    theta = math.atan(VisionData.Pose.position.x,-VisionData.Pose.position.y)
                    CarMove(VisionData.Pose.position.x,VisionData.Pose.position.y,theta,"car_link")
                    # TODO 调用激光雷达检测的服务
                    if 1:# TODO 激光雷达检测到在范围内，并且已经到达
                        break
                else:
                    # 遍历场地 TODO
                    pass
            '''
            #================ 2. 到达砖堆橙色处，开始取砖 ================#
            wait()
            brickIndex = 0
            while brickIndex < goal.Num:
                # 2.1 小车运动至期望砖堆处
                # 调用激光雷达检测的服务  goal.bricks[brickIndex].type
                
                # 2.2 机械臂取砖
                for attempt in range(0,3):  # 最大尝试次数3
                    result_ = self.goGetBrick(goal.bricks[brickIndex])
                    if result_ == SUCCESS:
                        # 记录当前点为这种砖的位置
                        self.show_tell("finished !PICK! brick %d,go get next" %brickIndex)
                        brickIndex = brickIndex + 1     # 成功了就取下一块  
                        break

            self.show_tell("Got all bricks, go to build")

            #================ 3. 到达L架 ================#
            # 机械臂移动至观察L架位姿
            # rob.movej(lookDownPos, acc=a, vel=3*v,wait=True)

            # 3.1 移动至L架
            # 3.1.1 如果有信息，直接运动至指定位置

            # 3.1.2 如果无信息，场地遍历，寻找L架


            '''

            #================ 3. 找到L架，开始搭建 ================#

            brickIndex = 0
            while brickIndex < goal.Num:
                
                tf_BrickOnOrign = tft.fromTranslationRotation((goal[brickIndex].x,goal[brickIndex].y,0),(0,0,0,1))
                # print goal[brickIndex]
                if goal[brickIndex].x == 0.0:
                    rot_ = tf.transformations.quaternion_from_euler(0,0,0)
                    tf_CarOnBrick = tft.fromTranslationRotation((-distanceBTcarlink_brick,0,0),rot_)      # 砖外0.5m
                elif goal[brickIndex].y == 0.0:
                    rot_ = tf.transformations.quaternion_from_euler(0,0,3.1415926/2)
                    tf_CarOnBrick = tft.fromTranslationRotation((0,-distanceBTcarlink_brick,0),rot_)      # 砖外0.5m
                target_tf = np.dot(np.dot(tf_OrignOnMap , tf_BrickOnOrign ), tf_CarOnBrick)
                target_rot = tf.transformations.quaternion_from_matrix(target_tf)
                target_trans = tf.transformations.translation_from_matrix(target_tf)

                # 动态配置
                CarMove(target_trans[0],target_trans[1],Rotation[3],frame_id="map",wait = True )
                self.show_tell("Got the %d brick place"% brickIndex)

                for attempt in range(0,3):  # 最大尝试次数3
                    result_ = self.buildWall(goal.bricks[brickIndex])
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
            self._result.finish_state = FAIL
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


    def goGetBrick(self, goal):

        # ----------- 已经到达橙色砖堆面前---------------- #

        # 机械臂移动至取砖准备位姿
        rob.movej(pickPos,acc=a, vel=3*v,wait=True)
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
        pose[2] = VisionData.Pose.position.z + floorHeight_base + 0.5      # 移动到砖上方0.5m处， 偏移值待确定
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
        rob.movej(pickPos,acc=a, vel=1*v,wait=True)
        self.show_tell("Got brick and arrived Ready-Put position" )

        if goal.goal_brick.type in ("R","G","B","O"):
            self.putOneBrickOnCar(goal, False)
        else:
            rospy.logwarn("Wrong Brick Type")
       
        # 执行完 位于可以抓取位姿，优化合并到putOne里面
        # rob.movej(pickPos,acc=a, vel=v,wait=True)      
        self.show_tell("arrived Ready-Pick position")

        return SUCCESS

    def buildWall(self,goal):

        # set_height(320)     # 可能需要提高
        # 进行一系列的操作，来放置砖块 @ 周权 % goal.goal_brick.Sequence
        rob.movej(takePos,acc=a, vel=v,wait=True)      # 到达Take动作开始位置
        if goal.goal_brick.type in ("R","G","B","O"):
            self.takeOneBrickOnCar(goal, False)
        else:
            rospy.logwarn("Wrong Brick Type")
       
        '''takePos的数值已经定了？'''
        rob.movej(buildPos,acc=a, vel=v,wait=True)      
        self.show_tell("arrived Ready-Build position")

        # ************* DEBUG temp stop *********************#
        rospy.sleep(2.0)
        self.turnEndEffect(OFF)
        return SUCCESS
        # ************* DEBUG temp stop *********************#

        # 配合手眼移动到摆放的位置
        # 调用视觉
        while True:         # TODO 避免进入死循环
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

        rob.movej(pickPos,acc=a, vel=3*v,wait=True)
        return SUCCESS


    def takeOneBrickOnCar(self, goal, useVisionToGetBrickOnCar_ = False):
        
        num = goal.Sequence
        try: 
            # 开始臂车运动
            rospy.loginfo("begining %d" %num)
            pose = rob.getl()
            initj = rob.getj()

            if goal.type == "O":
                if num > 3:     # 有第三层的两块
                    set_height(520)     # 可能需要提高,升降台的高度，目前320mm
                    rospy.sleep(2.0)    # TODO 改进时间
                elif num <= 3 and num >= 0:
                    set_height(320)
                    rospy.sleep(2.0)
            elif goal.type in ("R","G","B"):
                set_height(320)     # 升降台的高度到达 320mm
                rospy.sleep(0.5)

            rob.movej(onCarStartPos,acc=a, vel=v,wait=True)    #同时到达相机搜索砖块位置
            
            if: goal.type == "O":
                delta = ORG[num]
                delta[2] += 0.35 + CarHeight_base
                rob.movel(delta, acc=a, vel=v,wait=True)   # 摄像机对准铁片
            elif goal.type in ("R","G","B"):    #颜色类型 R/G/B
                delta = RGB[num]
                delta[2] += 0.35 + CarHeight_base          # 砖块上方0.35处
                rob.movel(delta, acc=a, vel=v,wait=True)   # 摄像机对准铁片

            # 视觉处理
            rospy.sleep(0.5)
            if useVisionToGetBrickOnCar_:
                while True:         # TODO 避免进入死循环
                    VisionData = GetVisionData_client(GetBrickPos, goal.type)
                    if VisionData.Flag:
                        # 工作空间判断 ok TODO 验证
                        if y > 0.2 and y < 0.70 \
                            and x < 0.35 and x>-0.35 \
                            and VisionData.Pose.position.z + CarHeight_base >0.1 \ 
                            and VisionData.Pose.position.z + CarHeight_base <0.7:
                            break

                # 得到识别结果，移动到砖块上方0.1，平移
                pose[0] = VisionData.Pose.position.x
                pose[1] = VisionData.Pose.position.y
                pose[2] = VisionData.Pose.position.z + CarHeight_base + 0.1     # 0.1是离砖10cm
                pose[3] = pi # 下两个坐标使其垂直于地面Brick remembered
                pose[4] = 0 
                pose[5] = 0
                rob.movel(pose, acc=a, vel=v, wait=True)
            else:
                rob.translate((0.0,0.15,-0.30), acc=a, vel=v,wait=True)   #模拟视觉处理后磁体对准铁片
                return FAIL

            self.forceDown(0.2)         # 伪力控下落
            self.turnEndEffect(ON)      # 操作末端
    
            if goal.type == "O":
                rob.movel([0.0,0.37,0.71,pi,0,0],acc=0.7*a, vel=0.8*v,wait=True)
            elif goal.type in ("R","G","B"):
                if num in (5,6):
                    # 只能抬升
                    rob.translate((0,0,0.35), acc=a, vel=v, wait=True)
                elif num  == 4:
                    #直接斜着提过去
                    rob.movel([0.0,0.37,0.71,pi,0,0],acc=a, vel=v,wait=True)
                elif num in (3,2):
                    # 抬升，并到位
                    rob.translate((0,0,0.35), acc=a, vel=v, wait=True)
                    rob.movel([0.0,0.37,0.71,pi,0,0],acc=a, vel=v,wait=True)
                elif num in (1,0):
                    #直接斜着提过去
                    rob.movel([0.0,0.37,0.71,pi,0,0],acc=a, vel=v,wait=True)
            
            return SUCCESS

        except Exception as e:
            rospy.logwarn("error", e)
            rob.stopl()
            return FAIL
        finally:
            pass

    def putOneBrickOnCar(self, goal, useVisionToGetBrickOnCar_ = False):
        num = goal.Sequence
        try: 
            # 开始臂车运动
            rospy.loginfo("begining %d" %num)
            pose = rob.getl()
            initj = rob.getj()

            if goal.type == "O":
                if num > 3:     # 有第三层的两块
                    set_height(520)     # 可能需要提高,升降台的高度，目前320mm
                    rospy.sleep(1.0)    # TODO 改进时间
                elif num <= 3 and num >= 0:
                    set_height(320)
                    rospy.sleep(1.0)
            elif goal.type in ("R","G","B"):
                set_height(320)     # 升降台的高度到达 320mm
                rospy.sleep(1.0)

            rob.movexs("movej",[putPos,onCarStartPos,onCarReadyPos],acc=a, vel=1.2*v,radius = 0.1, wait=True)    # 提升准备位置

            if: goal.type == "O":
                delta = ORG_load[num]
                if num in (0,2,4):
                    delta[2] += 0.05 + CarHeight_base
                    rob.movel(delta, acc=a, vel=v,wait=True)   #磁体对准铁片
                elif num in (1,3,5):
                    delta[2] += 0.25 + CarHeight_base
                    temp = list(delta)
                    temp[2] -= 0.2
                    rob.movels([delta,temp], acc=a, vel=v,radius = 0.03, wait=True)   #磁体对准铁片
                self.forceDown(0.1, 25)         # 伪力控下落
            elif goal.type in ("R","G","B"):    #颜色类型 R/G/B
                delta = RGB_load[num]
                # 部分砖块需要分步走
                if num in (0, 1, 4):
                    delta[2] += 0.05 + CarHeight_base
                    rob.movel(delta, acc=a, vel=v,wait=True)   #磁体对准铁片
                elif num in (2, 3, 5, 6):
                    delta[2] += 0.25 + CarHeight_base
                    temp = list(delta)
                    temp[2] -= 0.2
                    rob.movels([delta,temp], acc=a, vel=v,radius = 0.03, wait=True)   #磁体对准铁片
                self.forceDown(0.1)         # 伪力控下落

            self.turnEndEffect(OFF)      # 操作末端
    
            rob.movexs("movej",[onCarStartPos,prePickPos],acc=a, vel=v,radius = 0.1, wait=True)    # 提升准备位置
            
            return SUCCESS

        except Exception as e:
            rospy.logwarn("error", e)
            rob.stopl()
            return FAIL
        finally:
            pass



    def forceDown(self,distance,forcelimit = 15):
        rob.movel([0.0, 0.0, -distance, 0.0, 0.0, 0.0], acc=a, vel=v*0.1, wait=False, relative=True)     # 相对运动

        _force_prenvent_wrongdata_ = 0
        while force < forcelimit:
            _force_prenvent_wrongdata_ += 1
            if _force_prenvent_wrongdata_ >150: 
                _force_prenvent_wrongdata_ = 150 
            rospy.sleep(0.002)
            if  _force_prenvent_wrongdata_ >100 and ( not rob.is_program_running() ):
                rospy.loginfo("did not contact")
                break
        rob.stopl()
        self.show_tell("Reached Block")
    

    def turnEndEffect(self,state):
        # 操作末端
        # rospy.sleep(1.0)
        ee.MagState = state
        pub_ee.publish(ee)
        rospy.sleep(1.0)

# ================== END CLASS ===========================#

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

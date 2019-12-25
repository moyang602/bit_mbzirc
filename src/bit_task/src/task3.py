#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import urx
import math

# movebase 相关
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback
from actionlib_msgs.msg import GoalID
# 末端工具
from bit_control_msgs.msg import EndEffector
# 视觉相关
from bit_vision_msgs.srv import FirePosition

#---- 变量初始化 ----#
cancel_id = 0  # MoveBase 正在运行任务ID


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
    ori = tf.transformations.quaternion_from_euler(0,0,theta,'ryxz')
    this_target.pose.orientation.x = ori[0]
    this_target.pose.orientation.y = ori[1]
    this_target.pose.orientation.z = ori[2]
    this_target.pose.orientation.w = ori[3]

    # 发布位置
    goal_pub.publish(this_target)

# 视觉检测客户端
def GetFireVisionData_client(cameraUsed):
    rospy.wait_for_service('GetFireVisionData')
    try:
        get_vision_data = rospy.ServiceProxy('GetFireVisionData',FirePosition)
        respl = get_vision_data(cameraUsed)
        return respl
    except rospy.ServiceException, e:
        print "Service GetFireVisionData call failed: %s"%e

# 1. 机器人移动至门口附近
def MoveToBuilding():
    pass

# 2. 机器人寻找门口进入室内
def GoIntoDoor():
    pass

# 3. 机器人室内移动寻找火源
def FightFire():
    rospy.loginfo("Start fight fire motion")
    pose = rob.getl()
    print "Initial end pose is", pose 
    JointAngle = rob.getj()
    print "Initial joint angle is", JointAngle 
    
    #----- 1. 机械臂移动至火源探索位姿 -------#
    rob.movej(FindFirePos, acc=a, vel=3*v, wait=True, relative=False)

    '''
    #----- 2. 车臂移动探索火源 -------#
    # 2.1 小车开始自由移动
    CarMove(2.0, 0.0, 0.0*deg2rad)
    # 2.2 机械臂移动直到找到火源
    offset = [-20*deg2rad, 20*deg2rad, 20*deg2rad, -20*deg2rad]
    count = 0   #搜索次数
    while True:
        MovePos = FindFirePos
        MovePos[0] = MovePos[0]+offset[count%4]
        count = count+1
        rob.movej(MovePos,acc=a, vel=1*v,wait=True)
        rospy.sleep(0.5)
        VisionData = GetFireVisionData_client(HandEye)
        rospy.loginfo("MovePos[0] = ",MovePos[0])
        if VisionData.flag or count>20:  
            # 取消运动
            goal_cancel.publish(cancel_id)
            rospy.loginfo("car motion has been canceled")
            break
    if VisionData.flag:
        rospy.loginfo("Fire pos has been found")
    else:
        rospy.loginfo("Timeout, can't find Fire pos")

    #----- 3. 车臂移动对准火源 -------#
    # 3.1 小车运动
    JointAngle = rob.getj()
    CarMove(0.0, 0.0, JointAngle[0]+90*deg2rad)
    # 3.2 机械臂运动
    rob.movej(FindFirePos,acc=a, vel=1*v,wait=True)
    
    #----- 4. 车臂运动，靠近火源 -------#
    rospy.sleep(0.5)
    while True:
        VisionData = GetFireVisionData_client(HandEye)
        if VisionData.flag:
            deltax = VisionData.FirePos.point.x
            deltay = VisionData.FirePos.point.y
            pose = rob.getl()
            if math.fabs(pose[4] + math.pi)<3*deg2rad:  # 当机械臂末端接近竖直向下时停止
                goal_cancel.publish(cancel_id)
                break
            if math.fabs(deltax)<5:
                deltax = 0
            if math.fabs(deltay)<5:
                deltay = 0
            CarMove(0.2, 0.0, deltax*0.01*deg2rad)      # 偏航角通过小车调整
            if pose[1]<-0.5:                            # 机械臂一边移动一边往前伸
                pose = [0, 0 ,0,-deltay*0.2*deg2rad,0,0]      # 俯仰角通过机械臂调整
            else:
                pose = [0, -0.1 ,0,-deltay*0.2*deg2rad,0,0]      # 俯仰角通过机械臂调整
            rob.movel(pose, acc=a, vel=0.3*v, wait=False,relative=True)
            rospy.sleep(0.5)
        else:
            break
    
    #----- 5. 机械臂变为竖直向下状态 -------#
    pose = rob.getl()
    pose[3] = 0
    pose[4] = -pi
    pose[5] = 0
    rob.movel(pose, acc=a, vel=1*v, wait=True,relative=False)

    #----- 6. 控制机械臂水平移动，对准火源 -------#
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

    #----- 7. 开始灭火-------#
    
    # 操作末端 开始喷水
    
    rospy.sleep(1.0)
    ee.PumpState = 100
    pub_ee.publish(ee)
    rospy.sleep(1.0)
    

    # 喷5s水
    rospy.sleep(15.0)

    # 操作末端 停止
    
    rospy.sleep(1.0)
    ee.PumpState = 100
    pub_ee.publish(ee)
    rospy.sleep(1.0)

    # 等待水停止
    rospy.sleep(10.0)
    
     
    #----- 8. 机械臂复原-------#
    rob.movej(FindFirePos,acc=a, vel=2*v,wait=True)
    '''


if __name__ == '__main__':

    rospy.init_node('task3_node', anonymous=False)

    try:
        #---------- 0. 初始化 --------------#
        # UR 机器人

        # rob = urx.Robot("192.168.50.60", use_rt = True)
        # rospy.loginfo('UR controller connected')
        # rob.set_tcp((0, 0, 0, 0, 0, 0))             # 设置工具中心 Todo
        # rob.set_payload(0.0, (0.0, 0.0, 0.0))       # 设置末端负载 Todo

        # 小车移动相关话题初始化
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        goal_sub = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, move_base_feedback)
        goal_cancel = rospy.Publisher('/move_base/cancel',GoalID,queue_size=1)

        # 末端执行器
        EndEffector_pub = rospy.Publisher('endeffCmd',EndEffector, queue_size=1)

        #---------- 1. 机器人移动至门口附近 -------------#
        MoveToBuilding()

        #---------- 2. 机器人寻找门口进入室内 -------------#
        GoIntoDoor()

        #---------- 3. 机器人室内移动寻找火源 -------------#
        FightFire()


    except Exception as e:
        rospy.logerr('error', e)
    finally:
        # rob.stop()
        # rob.close()
        # rospy.loginfo('Task3 finished!')

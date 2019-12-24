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
import PyKDL
global rob
global ee
global simp_goal_pub
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
    a = tf.transformations.quaternion_from_euler(0,0,theta,'ryxz')
    print(a[0],a[1],a[2],a[3])
    this_target.pose.orientation.x = a[0]
    this_target.pose.orientation.y = a[1]
    this_target.pose.orientation.z = a[2]
    this_target.pose.orientation.w = a[3]

    global simp_goal_pub
    # 发布位置
    simp_goal_pub.publish(this_target)

# class fight_fire_act(object):
#     _feedback = bit_motion.msg.fightfireFeedback()
#     _result = bit_motion.msg.fightfireResult()

#     def __init__(self, name):
#         self._action_name = name
#         self._as = actionlib.SimpleActionServer(self._action_name, bit_motion.msg.fightfireAction , execute_cb=self.execute_cb, auto_start = False)
#         self._as.start()
#         rospy.loginfo("%s server ready! ", name)

#     def show_tell(self, info):
#         rospy.loginfo(info)
#         self._feedback.feedback_state = info
#         self._as.publish_feedback(self._feedback)

def execute_cb():
    
    try:
        # 机械臂状态初始化
        rospy.loginfo("Start fight fire motion")
        pose = rob.getl()
        print("Initial end pose is ", pose)
        JointAngle = rob.getj()
        print("Initial joint angle is ", JointAngle)
        global simp_goal_pub
        # simple_goal 话题发布初始化
        simp_goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
        simp_goal_sub = rospy.Subscriber("move_base/feedback",MoveBaseActionFeedback,move_base_feedback)
        # cancel
        simp_cancel = rospy.Publisher('/move_base/cancel',GoalID,queue_size=1)

        # 末端水枪话题初始化
        pub_ee = rospy.Publisher('endeffCmd',EndEffector,queue_size=1)
        ee = EndEffector()

        # 机械臂移动至火源探索位姿
        rob.movej(FindFirePos,acc=a, vel=3*v,wait=True)
        # self.show_tell("UR arrived FindFire position")
        JointAngle = rob.getj()
       
        #--------- 机械臂移动直到找到火源 -------#
        CarMove(2.0, 0.0, 0.0*deg2rad)
 
        offset = [-20*deg2rad, 20*deg2rad, 20*deg2rad, -20*deg2rad]
        count = 0
        while True:
            MovePos = JointAngle
            MovePos[0] = JointAngle[0]+offset[count%4]
            count = count+1
            rob.movej(MovePos,acc=a, vel=1*v,wait=True)
            rospy.sleep(0.5)
            VisionData = GetFireVisionData_client(HandEye)
            print("MovePos[0] = ",MovePos[0])
            if VisionData.flag or count>20:  
                # 取消运动
                simp_cancel.publish(cancel_id)
                print("car motion has been canceled")
                print("cancel_id = ",cancel_id)
                break
        
        #--------- 机械臂移动直到火源在图像中心线 -------#
        rospy.sleep(0.5)
        while True:
            VisionData = GetFireVisionData_client(HandEye)
            if VisionData.flag:
                deltax = VisionData.FirePos.point.x
                if math.fabs(deltax)<5:
                    break
                MovePos = [deltax*0.2*deg2rad,0,0,0,0,0]
                rob.movej(MovePos,acc=1*a, vel=0.3*v,wait=False,relative=True)
                rospy.sleep(2)
            else:
                break
        
        #--------- 控制小车移动相应角度，使机械臂朝向正前方 -------#
        JointAngle = rob.getj()
        CarMove(0.0, 0.0, JointAngle[0]+90*deg2rad)
        
        # 机械臂移动至灭火准备位姿
        rob.movej(FindFirePos,acc=a, vel=1*v,wait=True)
        
        #--------- 控制机械臂移动，直到火源在画面竖直方向中心 -------#
        # rospy.sleep(0.5)
        # while True:
        #     VisionData = GetFireVisionData_client(HandEye)
        #     if VisionData.flag:
        #         deltay = VisionData.FirePos.point.y
        #         if math.fabs(deltay)<5:
        #             break
        #         pose = [0,0,0,-deltay*0.08*deg2rad,0,0]
        #         rob.movel(pose, acc=a, vel=0.3*v, wait=False,relative=True)
        #     else:
        #         break
        
        #--------- 控制车臂同时移动，直到到达火源位置 -------#
        rospy.sleep(0.5)
        while True:
            VisionData = GetFireVisionData_client(HandEye)
            if VisionData.flag:
                deltax = VisionData.FirePos.point.x
                deltay = VisionData.FirePos.point.y
                pose = rob.getl()
                if math.fabs(pose[4] + math.pi)<3*deg2rad:  # 当机械臂末端接近竖直向下时停止
                    simp_cancel.publish(cancel_id)
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
        '''
        #--------- 机械臂竖直向下 -------#
        pose = rob.getl()
        pose[3] = 0
        pose[4] = -pi
        pose[5] = 0
        rob.movel(pose, acc=a, vel=1*v, wait=True,relative=False)

        #--------- 控制机械臂水平移动，对准火源-------#
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
        # 机械臂复原
        rob.movej(FindFirePos,acc=a, vel=2*v,wait=True)
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
    # while(not rospy.is_shutdown()):
    #     try :
    #         global rob
    #         rob = urx.Robot("192.168.50.60",use_rt = True) 
    #         normal = 1
    #         rospy.loginfo('UR controller connected')
    #         # Todo 根据实际末端负载与工具中心设置
    #         rob.set_tcp((0, 0, 0.0, 0, 0, 0))     #TASK2 参数 m,rad     todo 待修改
    #         rob.set_payload(0.0, (0.000, -0.000, 0.000))

    #         # fight_fire_act("FightFireAction") 
    #         execute_cb()
    #         rospy.spin() 
    #     except Exception as e:
    #         print("error", e)
    #         rospy.sleep(1)
    #     finally:
    #         if normal == 1:
    #             rob.stopl()
    #             rob.close()
    try :
        global rob
        rob = urx.Robot("192.168.50.60",use_rt = True) 
        normal = 1
        rospy.loginfo('UR controller connected')
        # Todo 根据实际末端负载与工具中心设置
        rob.set_tcp((0, 0, 0.0, 0, 0, 0))     #TASK2 参数 m,rad     todo 待修改
        rob.set_payload(0.0, (0.000, -0.000, 0.000))

        # fight_fire_act("FightFireAction") 
        execute_cb()
        rospy.spin() 
    except Exception as e:
        print("error", e)
        rospy.sleep(1)
    finally:
        if normal == 1:
            rob.stopl()
            rob.close()
    sys.exit(0)

    '''
    # base axis
    rob.up(z=0.05, acc=a, vel=v)
    rob.down(z=0.05, acc=a, vel=v)
    rob.translate([0,0,0.1], acc=a, vel=v, wait=True, command="movel")
    rob.movec(pose_via, pose_to, acc=a, vel=v, wait=True, threshold=None)
    rob.movep(tpose, acc=a, vel=v, wait=True, relative=False, threshold=None)
    rob.servoc(tpose, acc=a, vel=v, wait=True, relative=False, threshold=None)  # 运动很抖
    # tool axis
    rob.back(z=0.1, acc=a, vel=v)
    rob.translate_tool([0,0,0.1], acc=a, vel=v, wait=True, threshold=None)

    
    pose = rob.get_pos()
    def _get_lin_dist(self, target):
        pose = URRobot.getl(self, wait=True)
        target = m3d.Transform(target)
        pose = m3d.Transform(pose)
        return pose.dist(target)

    def set_csys(self, transform):
        """
        Set reference coordinate system to use
        """
        self.csys = transform

    def set_orientation(self, orient, acc=0.01, vel=0.01, wait=True, threshold=None):
        """
        set tool orientation using a orientation matric from math3d
        or a orientation vector
        """
        if not isinstance(orient, m3d.Orientation):
            orient = m3d.Orientation(orient)
        trans = self.get_pose()
        trans.orient = orient
        self.set_pose(trans, acc, vel, wait=wait, threshold=threshold)



    def set_pos(self, vect, acc=0.01, vel=0.01, wait=True, threshold=None):
        """
        set tool to given pos, keeping constant orientation
        """
        if not isinstance(vect, m3d.Vector):
            vect = m3d.Vector(vect)
        trans = m3d.Transform(self.get_orientation(), m3d.Vector(vect))
        return self.set_pose(trans, acc, vel, wait=wait, threshold=threshold)

    def set_pose(self, trans, acc=0.01, vel=0.01, wait=True, command="movel", threshold=None):
        """
        move tcp to point and orientation defined by a transformation
        UR robots have several move commands, by default movel is used but it can be changed
        using the command argument
        """
        self.logger.debug("Setting pose to %s", trans.pose_vector)
        t = self.csys * trans
        pose = URRobot.movex(self, command, t.pose_vector, acc=acc, vel=vel, wait=wait, threshold=threshold)
        if pose is not None:
            return self.csys.inverse * m3d.Transform(pose)

    def add_pose_base(self, trans, acc=0.01, vel=0.01, wait=True, command="movel", threshold=None):
        """
        Add transform expressed in base coordinate
        """
        pose = self.get_pose()
        return self.set_pose(trans * pose, acc, vel, wait=wait, command=command, threshold=threshold)

    def add_pose_tool(self, trans, acc=0.01, vel=0.01, wait=True, command="movel", threshold=None):
        """
        Add transform expressed in tool coordinate
        """
        pose = self.get_pose()
        return self.set_pose(pose * trans, acc, vel, wait=wait, command=command, threshold=threshold)

    def get_pose(self, wait=False, _log=True):
        """
        get current transform from base to to tcp
        """
        pose = URRobot.getl(self, wait, _log)
        trans = self.csys.inverse * m3d.Transform(pose)
        if _log:
            self.logger.debug("Returning pose to user: %s", trans.pose_vector)
        return trans

    def get_orientation(self, wait=False):
        """
        get tool orientation in base coordinate system
        """
        trans = self.get_pose(wait)
        return trans.orient

    def get_pos(self, wait=False):
        """
        get tool tip pos(x, y, z) in base coordinate system
        """
        trans = self.get_pose(wait)
        return trans.pos

    def speedl(self, velocities, acc, min_time):
        """
        move at given velocities until minimum min_time seconds
        """
        v = self.csys.orient * m3d.Vector(velocities[:3])
        w = self.csys.orient * m3d.Vector(velocities[3:])
        vels = np.concatenate((v.array, w.array))
        return self.speedx("speedl", vels, acc, min_time)

    def speedj(self, velocities, acc, min_time):
        """
        move at given joint velocities until minimum min_time seconds
        """
        return self.speedx("speedj", velocities, acc, min_time)

    def speedl_tool(self, velocities, acc, min_time):
        """
        move at given velocities in tool csys until minimum min_time seconds
        """
        pose = self.get_pose()
        v = pose.orient * m3d.Vector(velocities[:3])
        w = pose.orient * m3d.Vector(velocities[3:])
        self.speedl(np.concatenate((v.array, w.array)), acc, min_time)

    def movex(self, command, pose, acc=0.01, vel=0.01, wait=True, relative=False, threshold=None):
        """
        Send a move command to the robot. since UR robotene have several methods this one
        sends whatever is defined in 'command' string
        """
        t = m3d.Transform(pose)
        if relative:
            return self.add_pose_base(t, acc, vel, wait=wait, command=command, threshold=threshold)
        else:
            return self.set_pose(t, acc, vel, wait=wait, command=command, threshold=threshold)

    def movexs(self, command, pose_list, acc=0.01, vel=0.01, radius=0.01, wait=True, threshold=None):
        """
        Concatenate several movex commands and applies a blending radius
        pose_list is a list of pose.
        This method is usefull since any new command from python
        to robot make the robot stop
        """
        new_poses = []
        for pose in pose_list:
            t = self.csys * m3d.Transform(pose)
            pose = t.pose_vector
            new_poses.append(pose)
        return URRobot.movexs(self, command, new_poses, acc, vel, radius, wait=wait, threshold=threshold)

    def movel_tool(self, pose, acc=0.01, vel=0.01, wait=True, threshold=None):
        """
        move linear to given pose in tool coordinate
        """
        return self.movex_tool("movel", pose, acc=acc, vel=vel, wait=wait, threshold=threshold)

    def movex_tool(self, command, pose, acc=0.01, vel=0.01, wait=True, threshold=None):
        t = m3d.Transform(pose)
        self.add_pose_tool(t, acc, vel, wait=wait, command=command, threshold=threshold)


    def set_gravity(self, vector):
        if isinstance(vector, m3d.Vector):
            vector = vector.list
        return URRobot.set_gravity(self, vector)


        '''

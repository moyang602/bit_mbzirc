#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import roslib
import rospy
import urx

if sys.version_info[0] < 3:  # support python v2
    input = raw_input

from bit_control_tool.msg import EndEffector

def forcecallback(data):
    '''ur_force Callback Function'''
    global force
    force = data.wrench.force.x**2 + data.wrench.force.y**2 + data.wrench.force.z**2
    force = force ** 0.5

def wait():
    ''' used to debug move one by one step '''
    if do_wait:
        print("Click enter to continue")
        input()
def execute_cb():

    try: 
        # 开始臂车运动
        rospy.loginfo("begining")
        pose = rob.getl()
        print("Initial end pose is ", pose)
        
        t = rob.get_pose()
        print("Transformation from base to tcp is: ", t)
        
        if goal.task == TASK_GET:
            # 机械臂移动至取砖准备位姿
            rob.movej(prePickPos,acc=a, vel=3*v,wait=True)
            self.show_tell("arrived pre-pick position")

            # 视觉搜索目标砖块位置
            rospy.sleep(0.5)
            while True:         # Todo 避免进入死循环
                VisionData = GetVisionData_client(GetBrickPos, goal.goal_brick.type)
                if VisionData.Flag:
                    break

            # 得到识别结果如下

            self.show_tell("Got BrickPos results")

            # 判断是否在工作空间，不是任务返回失败    
            # 工作空间检测函数  Todo
            if 0:
                self._result.finish_state = FAIL_VISION
                return 0
            
            # 得到识别结果，移动到砖块上方，平移
            pose = rob.getl()
            pose[0] += VisionData.Pose.position.x
            pose[1] += VisionData.Pose.position.y

            pose[2] += 0.5     # 0.5是在地上的50cm      Todo   偏移值待确定
            pose[3] += 0
            # 下两个坐标使其垂直于地面Brick remembered
            pose[4] = - pi 
            pose[5] = 0 
            rob.movel(pose, acc=a, vel=v, wait=True)
            rospy.sleep(2.0)
            self.show_tell("Arrived block up 0.5m position pependicular to ground")

            # 视觉搜索目标砖块角度
            rospy.sleep(0.5)
            while True:         # Todo 避免进入死循环
                VisionData = GetVisionData_client(GetBrickAngle, goal.goal_brick.type)
                if VisionData.Flag:
                    break

            # 得到识别结果如下
            theta = VisionData.Pose.orientation.z
            self.show_tell("Got BrickAngle results")

            # 得到识别结果，移动到砖块上方0.1m，旋转角度
            pose = rob.getl()
            pose[0] += VisionData.Pose.position.x
            pose[1] += VisionData.Pose.position.y

            pose[2] += 0.3     # 0.3是在地上的30cm      Todo   偏移值待确定
            pose[3] =  theta
            # 下两个坐标使其垂直于地面Brick remembered
            pose[4] = - pi 
            pose[5] = 0
            rob.movel(pose, acc=a, vel=v, wait=True)
            rospy.sleep(2.0)
            self.show_tell("Arrived block up 0.1m position pependicular to ground")

            # 伪力控下落
            rob.translate((0,0,-0.2), acc=a, vel=v*0.3, wait=False)
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

            # 操作末端
            rospy.sleep(1.0)
            ee.MagState = 100
            pub_ee.publish(ee)
            rospy.sleep(1.0)

            # 先提起，后转正
            rob.translate((0,0,0.3), acc=a, vel=v, wait=True)

            rob.movej((0, 0, 0, 0, 0, theta),acc=a, vel=v, wait=False , relative=True)

            # 移动到预放置位置
            rob.movej(prePutPos,acc=a, vel=v,wait=True)
            self.show_tell("arrived pre-Put position %d" % goal.goal_brick.Sequence)
            delta = (0.1, goal.goal_brick.Sequence * 0.2 , 0, 0, 0, 0)
            rob.movel(delta, acc=a, vel=v,wait=True, relative=True )
            # 需要加入砖块信息来确定定位
            # 使用砖块的序列信息来计算自己需要放在那个位置，待完成
            # 移动到位，并记录posSequence = f(goal.goal_brick.Sequence)
            posSequence.append(delta)

            # 伪力控放置
            rob.translate((0, 0, -0.3), acc=a, vel=v*0.3, wait=False)
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
            self.show_tell("Put Down")

            # 释放末端
            rospy.sleep(1.0)
            ee.MagState = 0
            pub_ee.publish(ee)
            rospy.sleep(1.0)

            # 移动回来
            rob.movej(prePutPos,acc=a, vel=3*v,wait=True)
            self.show_tell("arrived pre-Put position, finished")

        elif goal.task == TASK_BUILD:
            # 移动至车上取砖位姿
            rob.movej(prePutPos,acc=a, vel=3*v,wait=True)
            self.show_tell("arrived pre-put position")

            # 移动至对应砖块处
            print(posSequence[goal.goal_brick.Sequence],goal.goal_brick.Sequence)
            rob.movel(posSequence[goal.goal_brick.Sequence], acc=a, vel=3*v,wait=True, relative=True )
            self.show_tell("arrived Brick remembered position")

            # 进行识别
            ## server ask vision! wait and try some times
            # 视觉搜索目标砖块位置
            rospy.sleep(0.5)
            while True:         # Todo 避免进入死循环
                VisionData = GetVisionData_client(GetBrickPos, goal.goal_brick.type)
                if VisionData.Flag:
                    break

            # 得到识别结果，移动到砖块上方，平移
            pose = rob.getl()
            pose[0] += VisionData.Pose.position.x
            pose[1] += VisionData.Pose.position.y

            pose[2] += 0.5     # 0.5是在地上的50cm      Todo   偏移值待确定
            pose[3] += 0
            # 下两个坐标使其垂直于地面Brick remembered
            pose[4] = - pi 
            pose[5] = 0 
            rob.movel(pose, acc=a, vel=v, wait=True)
            rospy.sleep(2.0)
            self.show_tell("Arrived block up 0.5m position pependicular to ground")

            # 伪力控下落
            rob.translate((0,0,-0.2), acc=a, vel=v*0.3, wait=False)
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

            # 操作末端
            rospy.sleep(1.0)
            ee.MagState = 100
            pub_ee.publish(ee)
            rospy.sleep(1.0)

            # 提起
            rob.translate((0,0,0.3), acc=a, vel=v, wait=True)


            rob.movej(prePickPos,acc=a, vel=v,wait=True)      # Todo 有问题，需要移动到砖xy，解算出的位置
            self.show_tell("arrived pre-Build position")
                
            # 配合手眼移动到摆放的位置
                
                

            # 伪力控放置
            rob.translate((0, 0, -0.3), acc=a, vel=v*0.3, wait=False)
            _force_prenvent_wrongdata_ = 0
            while force < 20:
                _force_prenvent_wrongdata_ += 1
                if _force_prenvent_wrongdata_ >150: 
                    _force_prenvent_wrongdata_ = 150 
                rospy.sleep(0.002)
                if  _force_prenvent_wrongdata_ >100 and ( not rob.is_program_running() ):
                    rospy.loginfo("did not contact")
                    break
                    
            rob.stopl()
            self.show_tell("Put Down")

            # 释放末端
            rospy.sleep(1.0)
            ee.MagState = 0
            pub_ee.publish(ee)
            rospy.sleep(1.0)

            rob.movej(prePickPos,acc=a, vel=3*v,wait=True)
            self.show_tell("arrived pre-Build position, finished")

        elif goal.task == TASK_LOOK:
            rob.movej(lookForwardPos, acc=a, vel=3*v,wait=True)
            self.show_tell("arrived look forward position")

        

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



if __name__ == '__main__':

#===================定义一些基本的对象======================#
    rospy.init_node('motion_test_node', anonymous = False)
    pub_ee = rospy.Publisher('endeffCmd',EndEffector,queue_size=1)
    rospy.Subscriber("/wrench", WrenchStamped, forcecallback)

    ee = EndEffector()
    normal = 0
    do_wait = True
    if len(sys.argv) > 1:
        do_wait = False
    while(not rospy.is_shutdown()):
        try :
            global rob
            rob = urx.Robot("192.168.50.60",use_rt = True) 
            normal = 1
            rospy.loginfo('robot ok')
            # Todo 根据实际末端负载与工具中心设置
            rob.set_tcp((0, 0, 0, 0, 0, 0))
            rob.set_payload(0.0, (0, 0, 0))

            execute_cb()    # 测试程序开始

            rospy.spin() 
        except:
            time.sleep(2.0)
        finally:
            if normal == 1:
                rob.stopl()
                rob.close()
    sys.exit(0)
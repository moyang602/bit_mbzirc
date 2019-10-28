# -*- coding: utf-8 -*-
#builtin
import time
import sys
import json
import os
import math
import numpy as np
#ros and gazebo
import rospy
from std_msgs.msg import String, Float64MultiArray, Int32
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState, ContactState
from libgazebotopic import GazeboTopic
#moveit
from hand_moveit import HandMoveGroup
#Pyqt5
from PyQt5 import QtWidgets,uic
# from ui import Ui_hand_main_window
from ui import hand_main_window
from sub_window import  Identity, Robot_Arm_joint, Robot_Arm_descartes, Load_Save, CyberGlove, QualityMeasure, EigenGrasp
from sub_window import PC_slider

class MainWindow(QtWidgets.QMainWindow, hand_main_window.Ui_MainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self.IdentityWindow = Identity() #定义子窗口，这个子窗口用来控制手指动作
        self.RobotArmJointWindow = Robot_Arm_joint() #定义子窗口，这个子窗口用来控制机械臂关节运动
        self.RobotArmEndWindow = Robot_Arm_descartes(self) #定义子窗口，这个子窗口用来控制机械臂末端运动
        self.LoadSave = Load_Save(self)
        self.CyberGlove = CyberGlove()
        self.QualityMeasure = QualityMeasure(self)
        self.EigenGrasp = EigenGrasp()

        #定义相关变量，变量的定义最好在初始化函数之后，其他函数调用之前，且尽量在初始化函数里，谨防出错
        self.gazebotopic_node = None
        #保存关节位置，便于提取
        self.jointStateMsg = None#type:JointState
        #需要保存的接触信息的link;包括每根手指上的三个指节，45手指的base link;初始值都为None
        #最后保存的变量类型是contacts = ContactState()
        '''
            ContactState格式为：
                string collision1_name
                string collision2_name
                geometry_msgs/Wrench[] wrenches
                geometry_msgs/Wrench total_wrench
                geometry_msgs/Vector3[] contact_positions
                geometry_msgs/Vector3[] contact_normals
        '''
        self.dict_link_contact = {            "f11":None, "f12":None, "f13":None, \
                                              "f21":None, "f22":None, "f23":None, \
                                              "f31":None, "f32":None, "f33":None, \
                                  "f4b":None, "f41":None, "f42":None, "f43":None, \
                                  "f5b":None, "f51":None, "f52":None, "f53":None 
                                  }# type: {str:ContactState}
        self.dict_link_contact_position = {            "f11":None, "f12":None, "f13":None, \
                                                       "f21":None, "f22":None, "f23":None, \
                                                       "f31":None, "f32":None, "f33":None, \
                                           "f4b":None, "f41":None, "f42":None, "f43":None, \
                                           "f5b":None, "f51":None, "f52":None, "f53":None
                                           }# type: {str:ContactState}
        #关节名字与pos框对应关系
        self.dict_jtname_lineEdit_pos = {"jt_f1_b":self.lineEdit_jt_f1_b_p, "jt_f1_0":self.lineEdit_jt_f1_0_p, "jt_f1_1":self.lineEdit_jt_f1_1_p, "jt_f1_2":self.lineEdit_jt_f1_2_p, "jt_f1_3":self.lineEdit_jt_f1_3_p, \
                                         "jt_f2_b":self.lineEdit_jt_f2_b_p, "jt_f2_0":self.lineEdit_jt_f2_0_p, "jt_f2_1":self.lineEdit_jt_f2_1_p, "jt_f2_2":self.lineEdit_jt_f2_2_p, "jt_f2_3":self.lineEdit_jt_f2_3_p, \
                                         "jt_f3_b":self.lineEdit_jt_f3_b_p, "jt_f3_0":self.lineEdit_jt_f3_0_p, "jt_f3_1":self.lineEdit_jt_f3_1_p, "jt_f3_2":self.lineEdit_jt_f3_2_p, "jt_f3_3":self.lineEdit_jt_f3_3_p, \
                                         "jt_f4_b":self.lineEdit_jt_f4_b_p, "jt_f4_0":self.lineEdit_jt_f4_0_p, "jt_f4_1":self.lineEdit_jt_f4_1_p, "jt_f4_2":self.lineEdit_jt_f4_2_p, "jt_f4_3":self.lineEdit_jt_f4_3_p, \
                                         "jt_f5_b":self.lineEdit_jt_f5_b_p, "jt_f5_0":self.lineEdit_jt_f5_0_p, "jt_f5_1":self.lineEdit_jt_f5_1_p, "jt_f5_2":self.lineEdit_jt_f5_2_p, "jt_f5_3":self.lineEdit_jt_f5_3_p 
                                        }
        #关节名字与torque框对应关系
        self.dict_jtname_lineEdit_torque = {"jt_f1_b":self.lineEdit_jt_f1_b_t, "jt_f1_0":self.lineEdit_jt_f1_0_t, "jt_f1_1":self.lineEdit_jt_f1_1_t, "jt_f1_2":self.lineEdit_jt_f1_2_t, "jt_f1_3":self.lineEdit_jt_f1_3_t, \
                                            "jt_f2_b":self.lineEdit_jt_f2_b_t, "jt_f2_0":self.lineEdit_jt_f2_0_t, "jt_f2_1":self.lineEdit_jt_f2_1_t, "jt_f2_2":self.lineEdit_jt_f2_2_t, "jt_f2_3":self.lineEdit_jt_f2_3_t, \
                                            "jt_f3_b":self.lineEdit_jt_f3_b_t, "jt_f3_0":self.lineEdit_jt_f3_0_t, "jt_f3_1":self.lineEdit_jt_f3_1_t, "jt_f3_2":self.lineEdit_jt_f3_2_t, "jt_f3_3":self.lineEdit_jt_f3_3_t, \
                                            "jt_f4_b":self.lineEdit_jt_f4_b_t, "jt_f4_0":self.lineEdit_jt_f4_0_t, "jt_f4_1":self.lineEdit_jt_f4_1_t, "jt_f4_2":self.lineEdit_jt_f4_2_t, "jt_f4_3":self.lineEdit_jt_f4_3_t, \
                                            "jt_f5_b":self.lineEdit_jt_f5_b_t, "jt_f5_0":self.lineEdit_jt_f5_0_t, "jt_f5_1":self.lineEdit_jt_f5_1_t, "jt_f5_2":self.lineEdit_jt_f5_2_t, "jt_f5_3":self.lineEdit_jt_f5_3_t 
                                        }

        #开始启动ros节点，订阅ros消息跟gazebo消息
        self.start_subscriber()

    def set_all_zero(self):
        arm_jt=[0]*7
        hand_name=['jt_f2_0', 'jt_f2_1', 'jt_f2_2', 'jt_f2_3', 'jt_f3_0', 'jt_f3_1', 'jt_f3_2', 'jt_f3_3', 'jt_f1_1', 'jt_f1_2', 'jt_f1_3', 'jt_f4_b', 'jt_f4_0', 'jt_f4_1', 'jt_f4_2', 'jt_f4_3', 'jt_f5_b', 'jt_f5_0', 'jt_f5_1', 'jt_f5_2', 'jt_f5_3']
        hand_angle = [0]*hand_name.__len__()
        hand=dict(zip(hand_name,hand_angle))
        self.RobotArmJointWindow.set_all_angle_by_teach(arm_jt)
        self.IdentityWindow.set_jt_angle_by_teach(hand)

    def move_by_teach(self):
        # 示教点：[0.0018, -0.2231, 0.0008, 0.3261, 1.5681, -0.0005, 1.6731]
        # [-0.2447, -0.4926, 0.4591, 0.4148, 1.394, -0.2024, 1.5008]手指待弯曲
        # [-0.4771, -0.396, 0.5846, 0.0401, 1.4383, -0.1835, 1.2548]待插入 [-0.0003, 1.5712, 0.0005, 0.0003, 0.0003, 1.5708, 0.0001, -0.0001, -0.0002, 0.001, 0.0011, -0.004, 0.0002, 1.5704, -0.0006, -0.0004, -0.0031, -0.0002, 1.5699, -0.0015, -0.0008]
        # [0.0214, -0.5102, 0.277, 0.3917, 1.2923, -0.1018, 1.4358]插入
        # 握住[-0.0033, 1.5711, 1.2219, 0.3491, -0.0033, 1.5708, 1.2217, 0.3491, -0.0002, 0.001, 0.0011, -0.0032, -0.003, 1.5705, 1.0471, 0.8728, -0.0027, -0.0022, 1.57, 0.6972, 0.8726]
        # [-0.0196, -0.2523, 0.3016, 0.2952, 1.301, -0.086, 1.602]拿到桌子外面
        # [0.7115, -0.2398, -0.4243, 0.1481, 1.2784, 0.1144, 1.5305]挪个位置
        #按示教点进行运动

        #到预备位置
        arm_jt_1 = [-0.4771, -0.396, 0.5846, 0.0401, 1.4383, -0.1835, 1.2548]
        hand_name_1=['jt_f2_0', 'jt_f2_1', 'jt_f2_2', 'jt_f2_3', 'jt_f3_0', 'jt_f3_1', 'jt_f3_2', 'jt_f3_3', 'jt_f1_1', 'jt_f1_2', 'jt_f1_3', 'jt_f4_b', 'jt_f4_0', 'jt_f4_1', 'jt_f4_2', 'jt_f4_3', 'jt_f5_b', 'jt_f5_0', 'jt_f5_1', 'jt_f5_2', 'jt_f5_3']
        hand_angle_1 = [-0.0003, 1.5712, 0.0005, 0.0003, 0.0003, 1.5708, 0.0001, -0.0001, -0.0002, 0.001, 0.0011, -0.004, 0.0002, 1.5704, -0.0006, -0.0004, -0.0031, -0.0002, 1.5699, -0.0015, -0.0008]
        hand_1 = dict(zip(hand_name_1, hand_angle_1))

        #进入把手
        arm_jt_2 =  [0.0214, -0.5102, 0.277, 0.3917, 1.2923, -0.1018, 1.4358]

        #握住
        hand_name_2 = ['jt_f2_0', 'jt_f2_1', 'jt_f2_2', 'jt_f2_3', 'jt_f3_0', 'jt_f3_1', 'jt_f3_2', 'jt_f3_3', 'jt_f1_1', 'jt_f1_2', 'jt_f1_3', 'jt_f4_b', 'jt_f4_0', 'jt_f4_1', 'jt_f4_2', 'jt_f4_3', 'jt_f5_b', 'jt_f5_0', 'jt_f5_1', 'jt_f5_2', 'jt_f5_3']
        hand_angle_2=  [-0.0033, 1.5711, 1.2219, 0.3491, -0.0033, 1.5708, 1.2217, 0.3491, -0.0002, 0.001, 0.0011, -0.0032, -0.003, 1.5705, 1.0471, 0.8728, -0.0027, -0.0022, 1.57, 0.6972, 0.8726]
        hand_2 = dict(zip(hand_name_2, hand_angle_2))

        #拿到桌子外面
        arm_jt_3 = [-0.0196, -0.2523, 0.3016, 0.2952, 1.301, -0.086, 1.602]

        #沿着x轴挪个位置
        arm_jt_4 = [0.7115, -0.2398, -0.4243, 0.1481, 1.2784, 0.1144, 1.5305]

        # #沿着y轴移动
        # arm_jt_5 = [-0.4923, 0.2109, 0.4383, -0.8679, 1.6811, 0.0503, 0.8728]
        #
        # #沿着x轴挪回到桌子
        # arm_jt_6 = [-0.2477, -0.1943, 0.6974, -0.5274, 1.1549, 0.201, 0.8864]
        #
        # #放到桌子上
        # arm_jt_7 = [-0.4646, -0.0205, 0.6524, -0.8998, 1.4562, 0.1596, 0.6437]
        #
        # #沿着y轴挪一下，让杯子摆的更正
        # arm_jt_8 = [-0.4458, -0.0257, 0.668, -0.8958, 1.4321, 0.1846, 0.6467]

        self.RobotArmJointWindow.set_all_angle_by_teach(arm_jt_1)#运动到预备位置
        # time.sleep(5)
        self.IdentityWindow.set_jt_angle_by_teach(hand_1)#手指做好准备动作
        time.sleep(2)
        # return
        self.RobotArmJointWindow.set_all_angle_by_teach(arm_jt_2)#进入把手
        # return
        self.IdentityWindow.set_jt_angle_by_teach(hand_2)#握住
        time.sleep(2)#等杯子稳定
        # return
        #拿起杯子，手指不动
        self.RobotArmJointWindow.set_all_angle_by_teach_nstep(arm_jt_3,20)
        # return
        #沿着x轴挪到桌子外面
        # self.RobotArmJointWindow.set_all_angle_by_teach_nstep(arm_jt_4,50)
        return
        # #沿着y轴移动
        # self.RobotArmJointWindow.set_all_angle_by_teach_nstep(arm_jt_5,20)
        # #沿着x轴挪回到桌子
        # self.RobotArmJointWindow.set_all_angle_by_teach_nstep(arm_jt_6,40)
        # #放到桌子上
        # self.RobotArmJointWindow.set_all_angle_by_teach_nstep(arm_jt_7,40)
        # #沿着y轴挪一下，让杯子摆的更正
        # self.RobotArmJointWindow.set_all_angle_by_teach_nstep(arm_jt_8,5)

    def show_PCslider(self):
        self.PC_slider = PC_slider()
        self.PC_slider.show()
    #菜单栏显示子窗口的响应函数
    def show_Identity_grasp_window(self):
        self.IdentityWindow.show()
    #菜单栏显示arm规划子窗口的响应函数
    def show_arm_joint_plan_window(self):
        self.RobotArmJointWindow.show()
    def show_arm_end_plan_window(self):
        self.RobotArmEndWindow.show()
    def show_cyberglove(self):
        self.CyberGlove.show()
    def show_load_save(self):
        self.LoadSave.show()
    def show_quality_measure(self):
        self.QualityMeasure.show()
    def show_eigen_grasp(self):
        self.EigenGrasp.show()

    def showContact(self, FingerName):
        self.textEdit_contact.clear()
        if FingerName == "Finger1":
            fingerName2link =["f11", "f12", "f13"]
        elif FingerName == "Finger2":
            fingerName2link =["f21", "f22", "f23"]
        elif FingerName == "Finger3":
            fingerName2link =["f31", "f32", "f33"]
        elif FingerName == "Finger4":
            fingerName2link =["f41", "f42", "f43"]
        else:
            fingerName2link =["f51", "f52", "f53"]


        if True:#为了不改变原有的代码格式，这里设true
            for link_key in fingerName2link:
                #先检查下是否是NONE或者wrenches列表里没有内容
                if self.dict_link_contact[link_key] is None or self.dict_link_contact[link_key].wrenches.__len__() == 0:
                    content = """\
link         object       total_wrench:             normal:      position:
{lnk: <10}                 no contact
                    """.format(lnk=link_key)
                    self.textEdit_contact.append(content)
                else:
                    #这个量下面要用很多次
                    current_contact = self.dict_link_contact[link_key]
                    total_contact_position = self.dict_link_contact_position[link_key]
                    #先显示下有多少个contact
                    contact_number = """\
--------------------------link:{lnk}   contact_number={num}-----------------\n""".format(lnk=link_key, num=current_contact.wrenches.__len__())
                    self.textEdit_contact.append(contact_number )
                    for i in range(current_contact.wrenches.__len__()):
                        #这里的textEdit控件有问题，显示的空格数量会减少，真的坑爹
                        content = """\
link         object          total_wrench             wrenches             normal                position: 第{i}个接触                position: 合作用点
                                        force:                              force:                          
                                            x:{foc.x:^19.4f}            x:{fos.x:>8.4f}               x:{nor.x:>8.4f}           x:{pos.x:>8.4f}                 x:{total_pos[0]:>8.4f}
                                            y:{foc.y:^19.4f}            y:{fos.y:>8.4f}               y:{nor.y:>8.4f}           y:{pos.y:>8.4f}                 y:{total_pos[1]:>8.4f}
                                            z:{foc.z:^19.4f}            z:{fos.z:>8.4f}               z:{nor.z:>8.4f}           z:{pos.z:>8.4f}                 z:{total_pos[2]:>8.4f}
                                  total_force:{t_foc:^19.4f}
{lnk: <10}   {obj:<3}
                                        torque:                            torque:
                                            x:{tor.x:^19.4f}            x:{fos.x:>8.4f}
                                            y:{tor.y:^19.4f}            y:{fos.y:>8.4f}
                                            z:{tor.z:^19.4f}            z:{fos.z:>8.4f}
                                  total_torqu:{t_tor:^19.4f}
                        """.format(i=i,
                                   lnk=link_key, obj = current_contact.collision1_name[0:3],foc=current_contact.total_wrench.force, 
                                   tor=current_contact.total_wrench.torque,
                                   t_foc=math.sqrt(current_contact.total_wrench.force.x**2 + current_contact.total_wrench.force.y**2 + current_contact.total_wrench.force.z**2),
                                   t_tor=math.sqrt(current_contact.total_wrench.torque.x**2 + current_contact.total_wrench.torque.y**2 + current_contact.total_wrench.torque.z**2),
                                   fos=current_contact.wrenches[i].force, 
                                   nor=current_contact.contact_normals[i], pos=current_contact.contact_positions[i],
                                   total_pos=total_contact_position
                                   )
                        self.textEdit_contact.append(content)

    def update_jt_torque(self, data):
        '''更新关节力矩，实际上是在接受到/pubHand_text消息时，手动调用self.gazebotopic_node.get_msgs()函数然后更新相应的显示框'''
        #先判断这个变量是否为空，因为有可能c++ lib调用失败，导致这个量没有被赋值
        if self.gazebotopic_node is not None:
            #get到json字符串然后load为对象；对象为字典{"joint_names":[type(string), ...], "joint_z_torque":[float...], "updateTime":type(string)}
            gazebo_msgs = json.loads(self.gazebotopic_node.get_msgs()) 
            # print "接收到的消息为：", gazebo_msgs
            #处理消息中每一个关节，按名字处理
            for i in range(gazebo_msgs["joint_names"].__len__()):
                #判断关节名字是否在处理范围只能，依据字典键值self.dict_jtname_lineEdit_torque.keys()
                if gazebo_msgs["joint_names"][i] in self.dict_jtname_lineEdit_torque.keys():
                    #关节在处理范围之内，根据字典self.dict_jtname_lineEdit_torque确定对应的显示框然后显示对应的关节位置值
                    self.dict_jtname_lineEdit_torque[ gazebo_msgs["joint_names"][i] ].setText(  u"%.5f" % gazebo_msgs["joint_z_torque"][i]  )
        else :
            print "fatal error: gazebotopic_node is None"
    
    def update_jt_pos(self, msg):
        '''type(msg) is JointState
        1.根据关节名称更新不同的lineEdit的值，通过字典self.dict_jtname_lineEdit_pos确定对应关系
        2.Sensor_msgs::JointState类型为
                                    std_msgs/Header header
                                    string[] name
                                    float64[] position
                                    float64[] velocity
                                    float64[] effort
        3.这个消息是自己写的插件然后发布出来的，源码为pub_hand_info.cpp
        '''
        # print "准备更新关节位置"
        self.jointStateMsg = msg#保存，便于load_save窗口提取
        for i in range(msg.name.__len__()):
            if msg.name[i] in self.dict_jtname_lineEdit_pos.keys():
                # print "这个关节存在，设置相应关节的位置值"
                angle = msg.position[i]*180.0/3.1415926
                self.dict_jtname_lineEdit_pos[msg.name[i]].setText(  u"%.5f" % angle  )

    def start_subscriber(self):
        '''这里
        1.尝试启动ros节点，然后订阅不同的消息并发给不同的回调函数；
        2.尝试引入c++lib，启动gazebo节点，但是gazebo消息没有回调函数，需要在其他消息更新时顺带更新，目前是依靠/pubHand_text，这个消息由插件pub_hand_info.cpp中发布；
          也就是说在插件pub_hand_info.cpp发布关节位置时，顺带更新了joint_sensor中的关节z轴力矩
        '''
        try:
            rospy.init_node('listener', anonymous = True)#启动ros node
            #订阅不同的ros消息
            # rospy.Subscriber('pubHand1', Float64MultiArray, self.callback)
            rospy.Subscriber('/pubHand_text', Int32, self.callback)
            rospy.Subscriber('/pubHand_jointState', JointState, self.JointStateCallback)
            # rospy.Subscriber('/f13_collision_bumper', ContactsState, self.ContactsCallback)
            # rospy.Subscriber('/box_bumper', ContactsState, self.ContactsCallback)
            # 接收f23的碰撞信息，用来测试
            rospy.Subscriber('/f23_bumper', ContactsState, self.F23BumperCallback)

            #这里不同的回调函数要不同的starttime，否则更新频率的控制会冲突
            self.start_time = time.time()#用于设置显示的更新时间
            self.joint_state_start_time = time.time()#用于设置显示的更新时间
            self.contacts_state_start_time = time.time()#用于设置显示的更新时间
            rospy.loginfo('启动消息订阅成功')
        except rospy.ROSInterruptException:
            print "请启动roscore\n"
        #这里订阅gazebo消息，要导入c++模块，且消息要手动提取然后更新到界面上
        # return #暂时关闭这个模块
        try:
            self.gazebotopic_node = GazeboTopic()
        except :
            print "gazebotopic c++ lib导入出错"
        else:
            print "gazebotopic c++ lib导入成功"

    def callback(self, msg):
        #关节力矩信息通过这个回调函数调用
        '''一定要注意这里有三行涉及到时间设置，如果要修改时间变量一定要考虑到所有的三行'''
        now_time = time.time()
        if (now_time-self.start_time)>1:#每隔１秒更新一次显示
            # print "进入回调函数callback"
            # if isinstance(msg, String):
            #     self.start_time = time.time()
            #     rospy.loginfo(rospy.get_caller_id()+"I heard %s", msg.data)
            # if isinstance(msg, Float64MultiArray):
            #     self.start_time = time.time()
            #     self.update_jt_torque(msg.data)
            if isinstance(msg, Int32):
                #一定要重设下时间，否则更新太快
                self.start_time = time.time()
                self.update_jt_torque(msg.data)
    def JointStateCallback(self, msg):
        '''一定要注意这里有三行涉及到时间设置，如果要修改时间变量一定要考虑到所有的三行'''
        now_time = time.time()
        if (now_time-self.joint_state_start_time )>1:#每隔１秒更新一次显示
            # print "进入回调函数JointStateCallback"
            if isinstance(msg, JointState):
                #一定要重设下时间，否则更新太快
                # print "接受到jointstate消息"
                self.joint_state_start_time  = time.time()
                self.update_jt_pos(msg)
    def F23BumperCallback(self, msg):
        #指定下参数类型
        msg =msg # type: ContactsState
        '''一定要注意这里有三行涉及到时间设置，如果要修改时间变量一定要考虑到所有的三行'''
        if not isinstance(msg, ContactsState):
            print "传入消息的参数类型错误"
            return
        # 指定下参数类型
        msg = msg  # type: ContactsState
        now_time = time.time()

        if (now_time - self.contacts_state_start_time) > 1:  # 每隔１秒更新一次
            '''
               ContactsStates格式为
                 std_msgs/Header header
                 gazebo_msgs/ContactState[] states
               ContactState格式为：
                 string collision1_name
                 string collision2_name
                 geometry_msgs/Wrench[] wrenches
                 geometry_msgs/Wrench total_wrench
                 geometry_msgs/Vector3[] contact_positions
                 geometry_msgs/Vector3[] contact_normals
            '''
            # print "进入回调函数ContactStateCallback"
            # 一定要重设下时间，否则更新速度的限制不起作用
            self.contacts_state_start_time = time.time()
            #如果接触消息为空，就清空这个指节对应的接触
            if msg.states.__len__()==0:
                self.dict_link_contact["f23"]=None
            # 第一步把很多的ContactsState转成ContactState
            #这个函数返回接触ContactState跟接触力的合作用点
            contact, pos = self.ProcessContactMSg(msg)
            #更新各个指节的接触
            for key in self.dict_link_contact.keys():
                if key in contact.collision1_name or key in contact.collision2_name:
                    self.dict_link_contact[key] = contact
                    self.dict_link_contact_position[key] = pos

    def ContactsCallback(self, msg) :
        '''一定要注意这里有三行涉及到时间设置，如果要修改时间变量一定要考虑到所有的三行'''
        if not isinstance(msg, ContactsState):
            print "传入消息的参数类型错误"
            return
        #指定下参数类型
        msg =msg # type: ContactsState
        now_time = time.time()

        if (now_time-self.contacts_state_start_time )>1:#每隔１秒更新一次
            '''
               ContactsStates格式为
                 std_msgs/Header header
                 gazebo_msgs/ContactState[] states
               ContactState格式为：
                 string collision1_name
                 string collision2_name
                 geometry_msgs/Wrench[] wrenches
                 geometry_msgs/Wrench total_wrench
                 geometry_msgs/Vector3[] contact_positions
                 geometry_msgs/Vector3[] contact_normals
            '''
            # print "进入回调函数ContactStateCallback"
            #一定要重设下时间，否则更新速度的限制不起作用
            self.contacts_state_start_time   = time.time()
            #第一步把很多的ContactsState转成ContactState
            contacts = ContactState()
            contacts.info ="info"
            contacts.depths = [0,0]
            for contact in msg.states:
                contacts.collision1_name = contact.collision1_name
                contacts.collision2_name = contact.collision2_name
                contacts.wrenches.extend(contact.wrenches)
                contacts.contact_positions.extend(contact.contact_positions)
                contacts.contact_normals.extend(contact.contact_normals)
                #Vector3类居然没有加法，真的捉急
                contacts.total_wrench.force.x =  contacts.total_wrench.force.x  + contact.total_wrench.force.x
                contacts.total_wrench.force.y =  contacts.total_wrench.force.y  + contact.total_wrench.force.y
                contacts.total_wrench.force.z =  contacts.total_wrench.force.z  + contact.total_wrench.force.z
                contacts.total_wrench.torque.x = contacts.total_wrench.torque.x + contact.total_wrench.torque.x
                contacts.total_wrench.torque.y = contacts.total_wrench.torque.y + contact.total_wrench.torque.y
                contacts.total_wrench.torque.z = contacts.total_wrench.torque.z + contact.total_wrench.torque.z
            #给各个linl赋值接触信息;消息与link name相关，如f11等
            for key in self.dict_link_contact.keys():
                if key in contacts.collision1_name or  key in contacts.collision2_name:
                    self.dict_link_contact[key] = contacts

    def ProcessContactMSg(self, msg) :
        """
        处理msg中的接触信息。目前的缺陷是无法区分一对碰撞体之间距离较远的碰撞点
        :param msg: 要求输入的msg中只包含一对碰撞体
        :return: 返回contacts，类型为ContactState,包含一对碰撞体之间的合碰撞点。
        """
        if not isinstance(msg, ContactsState):
            print "传入消息的参数类型错误"
            return
        #指定下参数类型
        msg =msg # type: ContactsState
        '''
           ContactsStates格式为
             std_msgs/Header header
             gazebo_msgs/ContactState[] states
           ContactState格式为：
             string collision1_name
             string collision2_name
             geometry_msgs/Wrench[] wrenches
             geometry_msgs/Wrench total_wrench
             geometry_msgs/Vector3[] contact_positions
             geometry_msgs/Vector3[] contact_normals
        '''
        # print "进入回调函数ContactStateCallback"
        #第一步把很多的ContactsState转成ContactState
        contacts = ContactState()
        contacts.info ="info"
        # contacts.collision2_name = "collision2"
        contacts.depths = [0,0]
        for contact in msg.states:
            contacts.collision1_name = contact.collision1_name
            contacts.collision2_name = contact.collision2_name
            contacts.wrenches.extend(contact.wrenches)
            contacts.contact_positions.extend(contact.contact_positions)
            contacts.contact_normals.extend(contact.contact_normals)
            #Vector3类居然没有加法，真的捉急
            contacts.total_wrench.force.x =  contacts.total_wrench.force.x  + contact.total_wrench.force.x
            contacts.total_wrench.force.y =  contacts.total_wrench.force.y  + contact.total_wrench.force.y
            contacts.total_wrench.force.z =  contacts.total_wrench.force.z  + contact.total_wrench.force.z
            contacts.total_wrench.torque.x = contacts.total_wrench.torque.x + contact.total_wrench.torque.x
            contacts.total_wrench.torque.y = contacts.total_wrench.torque.y + contact.total_wrench.torque.y
            contacts.total_wrench.torque.z = contacts.total_wrench.torque.z + contact.total_wrench.torque.z
        #删除没有力的那些接触点
        for i in range(contacts.wrenches.__len__()-1,-1,-1):#从尾到头遍历，然后删除
            if abs(contacts.wrenches[i].force.x) < 1e-06 and abs(contacts.wrenches[i].force.y) < 1e-06 and abs(contacts.wrenches[i].force.y) < 1e-06:
                contacts.wrenches.pop(i)
                contacts.contact_positions.pop(i)
                contacts.contact_normals.pop(i)
        #接触点的实际作用位置，是这些小点的合作用点
        total_positions= np.array([0,0,0])#合力作用点，初始为0
        f1 = 0#f1是当前合力大小
        for wrench,pos in zip(contacts.wrenches, contacts.contact_positions):
            f2 = math.sqrt(wrench.force.x**2 + wrench.force.x**2 + wrench.force.x**2)
            p2 = np.array([pos.x, pos.y, pos.z])
            total_positions =( f1/(f1+f2) )*total_positions + ( f2/(f1+f2) )*p2#更新合作用点的公式 f1/(f1+f2)*p1 + f2/(f1+f2)*p2
            f1=f1+f2#合力大小也要更新
        return contacts,total_positions
        # #给各个link赋值接触信息;消息与link name相关，如f11等
        # for key in self.dict_link_contact.keys():
        #     if key in contacts.collision1_name or  key in contacts.collision2_name:
        #         self.dict_link_contact[key] = contacts
                

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    # main_window.RobotArmJointWindow.show()
    # main_window.LoadSave.show()
    sys.exit(app.exec_())

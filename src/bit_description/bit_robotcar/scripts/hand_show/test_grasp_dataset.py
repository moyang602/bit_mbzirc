# -*- coding: utf-8 -*-
import  rospy
import json
from std_msgs.msg import String
import time
from scipy.io import loadmat
rospy.init_node('chatter', anonymous = True)#启动ros node
pub = rospy.Publisher('/set_hand_pos_target', String, queue_size=10)
def pub_angle(anglelist):
    anglelist[0]=0.21-anglelist[0]
    anglelist[4]=-anglelist[4]
    anglelist[8]=-anglelist[8]
    anglelist[12]=-anglelist[12]-0.157
    anglelist[16]=-anglelist[16]-0.4538
    # f10关节指的是拇指的abduction关节
    planned_joint_names = ["jt_f1_0", "jt_f1_1", "jt_f1_2", "jt_f1_3", \
                           "jt_f2_0", "jt_f2_1", "jt_f2_2", "jt_f2_3", \
                           "jt_f3_0", "jt_f3_1", "jt_f3_2", "jt_f3_3", \
                           "jt_f4_0", "jt_f4_1", "jt_f4_2", "jt_f4_3", \
                           "jt_f5_0", "jt_f5_1", "jt_f5_2", "jt_f5_3"]
    # rospy.init_node('set_hand_pos_from_Identity_window')
    rate = rospy.Rate(10)
    #要发送的消息，字典转json即可
    msg = dict()
    for key, pos in zip(planned_joint_names, anglelist) :
        #窗口输入是角度，gazebo接收的是弧度
        # msg[key] = pos/180.0*3.1415926
        msg[key] = pos
    msg = json.dumps(msg)
    print msg
    for i in range(5):
        pub.publish(msg)
        # rate.sleep()
        time.sleep(0.01)
    print "-------------------------------------------over--------------------------------------------------------------------------------------"
    # while True :
    #     pub.publish(msg)
    #     # rate.sleep()
    #     time.sleep(0.01)
datasetfile=loadmat("UNIPI_Dataset.mat")
pos=datasetfile["postures"]#type： ndarray
# 抓key的动作
action=pos[0,11]#type： ndarray
for i in range(0,action.__len__(),10):
    print "第", i, "个"
    pub_angle(action[i,:])
    time.sleep(1)
# print action[100,:]
# action=[0.0]*20
# pub_angle(action)
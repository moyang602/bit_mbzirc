# -*- coding: utf-8 -*-
#built-in
import re,os
import copy
import socket
import json
import time

#ros and gazebo
from sensor_msgs.msg import JointState
import rospy
from std_msgs.msg import String
#moveit
from hand_moveit import HandMoveGroup
#PyQt
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog,QTableWidgetItem
from PyQt5 import  QtCore
#pandas, matlab, quaternion
import pandas as pd
import matlab
import matlab.engine
import numpy as np
import quaternion
import math
import scipy.io as io

from ui import Ui_Identity
from ui import Ui_robot_arm_joint, robot_arm_decarte
from ui import load_save
from ui import cyber_glove_ui
from ui import quality_cal_ui
from ui import eigen_grasp
from ui import PC_slider

#用于手指的模块都不需要
class PC_slider(QtWidgets.QDialog, PC_slider.Ui_Dialog):
    def __init__(self):
        super(PC_slider, self).__init__()
        self.setupUi(self)
        self.dataset=io.loadmat("./dataset/pcamodel.mat")
        self.pub = rospy.Publisher('/set_hand_pos_target', String, queue_size=10)
        #在不同主成分上的坐标值的范围(取均值后)，UNIPI数据集中大致为+-2 +-1.5 +-1，这里的允许取值翻倍
        self.pc1Magnitude=4
        self.pc2Magnitude=3
        self.pc3Magnitude=2
        #设置关闭窗口时销毁自身实例对象
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
    def setPCValue(self):
        pc1=self.PC1Slider.value()/(50.0/self.pc1Magnitude)
        pc2=self.PC2Slider.value()/(50.0/self.pc2Magnitude)
        pc3=self.PC3Slider.value()/(50.0/self.pc3Magnitude)
        try:
            pc1=float( self.lineEdit_pc1.text() )
            temp = int( pc1*(50.0/self.pc1Magnitude) )
            if abs(temp)>50:
                temp=50 if temp>0 else -50
                pc1=self.pc1Magnitude if pc1>0 else -self.pc1Magnitude
                print "设置的值过大，取最大值", pc1
                #更新文本框的值
                self.lineEdit_pc1.setText( str(pc1) )
            self.PC1Slider.setValue( temp )
        except :
            print "PC1的值设置有问题，取当前滑块的值"
        try:
            pc2=float( self.lineEdit_pc2.text() )
            temp = int( pc2*(50.0/self.pc2Magnitude) )
            if abs(temp)>50:
                temp=50 if temp>0 else -50
                pc2=self.pc2Magnitude if pc2>0 else -self.pc2Magnitude
                print "设置的值过大，取最大值", pc2
                #更新文本框的值
                self.lineEdit_pc2.setText( str(pc2) )
            self.PC2Slider.setValue( temp )
        except :
            print "PC2的值设置有问题，取当前滑块的值"
        try:
            pc3=float( self.lineEdit_pc3.text() )
            temp = int( pc3*(50.0/self.pc3Magnitude) )
            if abs(temp)>50:
                temp=50 if temp>0 else -50
                pc3=self.pc3Magnitude if pc3>0 else -self.pc3Magnitude
                print "设置的值过大，取最大值", pc3
                #更新文本框的值
                self.lineEdit_pc3.setText( str(pc3) )
            self.PC3Slider.setValue( temp )
        except :
            print "PC3的值设置有问题，取当前滑块的值"
        #重建样本
        X=np.array([pc1, pc2, pc3])
        Y = np.asmatrix(X)*np.asmatrix(self.dataset["components"]) + self.dataset["mean"]
        Y=np.asarray(Y)
        Y=Y.squeeze()
        self.pub_angle(Y)

    def slide(self):
        # senderName=self.sender().objectName()
        #下面三个值都是+-50
        pc1=self.PC1Slider.value()
        pc2=self.PC2Slider.value()
        pc3=self.PC3Slider.value()
        #slide值映射到pc值
        pc1=pc1/(50.0/self.pc1Magnitude)
        pc2=pc2/(50.0/self.pc2Magnitude)
        pc3=pc3/(50.0/self.pc3Magnitude)
        #更新文本框中的pc值
        self.lineEdit_pc1.setText(str(round(pc1,3)))
        self.lineEdit_pc2.setText(str(round(pc2,3)))
        self.lineEdit_pc3.setText(str(round(pc3,3)))
        #重建样本
        X=np.array([pc1, pc2, pc3])
        Y = np.asmatrix(X)*np.asmatrix(self.dataset["components"]) + self.dataset["mean"]
        Y=np.asarray(Y)
        Y=Y.squeeze()
        self.pub_angle(Y)

    def pub_angle(self, anglelist):
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
        for i in range(10):
            self.pub.publish(msg)
            # rate.sleep()
            time.sleep(0.01)
        print "-------------------------------------------over--------------------------------------------------------------------------------------"

class EigenGrasp(QtWidgets.QDialog, eigen_grasp.Ui_Dialog):
    #TODO
    def __init__(self):
        super(EigenGrasp, self).__init__()
        self.setupUi(self)

class QualityMeasure(QtWidgets.QDialog, quality_cal_ui.Ui_Dialog):
    def __init__(self, parent=None):
        super(QualityMeasure, self).__init__()
        self.setupUi(self)
        self.parent = parent
    def calculate(self):
        print "启动matlab很慢，耐心等待，不要重复点击"
        reply = QtWidgets.QMessageBox.question(self,'提示',"启动matlab很慢，耐心等待，不要重复点击",QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No,QtWidgets.QMessageBox.Yes)
        #启动matlab引擎
        eng=matlab.engine.start_matlab()
        #先做个样子，后面再改
        U = [1,2,3]
        m = 12
        u = 0.2
        quality = eng.quality_cal(U, m, u)
        eng.quit()#关闭matlab
        self.lineEdit_quality.setText( str(quality) )
        reply = QtWidgets.QMessageBox.question(self,'提示',"调用matlab计算完毕",QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No,QtWidgets.QMessageBox.Yes)
    def read(self):
        #临时写的一个
        self.textEdit_contacts.clear()
        if True:
            for link_key in ["f11", "f12", "f13"]:
                # 先检查下是否是NONE或者wrenches列表里没有内容
                if self.parent.dict_link_contact[link_key] is None or self.parent.dict_link_contact[link_key].wrenches.__len__() == 0:
                    content = """\
        link         object       total_wrench:             normal:      position:
        {lnk: <10}                 no contact
                            """.format(lnk=link_key)
                    self.textEdit_contacts.append(content)
                else:
                    # 这个量下面要用很多次
                    current_contact = self.parent.dict_link_contact[link_key]
                    # 先显示下有多少个contact
                    contact_number = """\
        --------------------------link:{lnk}   contact_number={num}-----------------\n""".format(lnk=link_key,
                                                                                                 num=current_contact.wrenches.__len__())
                    self.textEdit_contacts.append(contact_number)
                    for i in range(current_contact.wrenches.__len__()):
                        # 这里的textEdit控件有问题，显示的空格数量会减少，真的坑爹
                        content = """\
        link         object          total_wrench            mwrenches             normal                position: 第{i}个接触
                                                force:                             force:                          
                                                    x:{foc.x:^19.4f}            x:{fos.x:>8.4f}               x:{nor.x:>8.4f}           x:{pos.x:>8.4f}
                                                    y:{foc.y:^19.4f}            y:{fos.y:>8.4f}               y:{nor.y:>8.4f}           y:{pos.y:>8.4f}
                                                    z:{foc.z:^19.4f}            z:{fos.z:>8.4f}               z:{nor.z:>8.4f}           z:{pos.z:>8.4f}
        {lnk: <10}   {obj:<3}
                                                torque:                            torque:
                                                    x:{tor.x:^19.4f}            x:{fos.x:>8.4f}
                                                    y:{tor.y:^19.4f}            y:{fos.y:>8.4f}
                                                    z:{tor.z:^19.4f}            z:{fos.z:>8.4f}
                                """.format(i=i,
                                           lnk=link_key, obj=current_contact.collision1_name[0:3],
                                           foc=current_contact.total_wrench.force,
                                           tor=current_contact.total_wrench.torque,
                                           fos=current_contact.wrenches[i].force,
                                           nor=current_contact.contact_normals[i],
                                           pos=current_contact.contact_positions[i])
                        self.textEdit_contacts.append(content)

class CyberGlove(QtWidgets.QDialog, cyber_glove_ui.Ui_Dialog):
    def __init__(self):
        super(CyberGlove, self).__init__()
        self.setupUi(self)
        self.sock=None# type:socket.socket
        self.addr=None#连接数据手套的sock跟地址
        #字典形式为{'f1_0':0.1,...}
        # self.hand_data=None#type:dict
        #测试，直接用现成的数据
        self.hand_data =  {'f1_0': -4.60200, 'f1_1': -94.38678, 'f1_2': 2.50784,   'f1_3': -8.17611,
                           'f2_0': -3.38959, 'f2_1': -39.81369, 'f2_2': 3.34378,   'f2_3': 0.63089,
                           'f3_0': -5.70491, 'f3_1': -40.52244, 'f3_2': -7.23531,  'f3_3': -0.93763,
                           'f4_0': 14.98345, 'f4_1': -72.48145, 'f4_2': -58.39357, 'f4_3': -9.11706,
                           'f5_0': 11.91010, 'f5_1': -64.79236, 'f5_2': -76.12317, 'f5_3': -19.42298,
                           }
        self.hand_data = {'f1_0': -0.86287, 'f1_1': -92.57165, 'f1_2': 5.85162,   'f1_3': -28.61638,
                          'f2_0': 3.96874,  'f2_1': -28.70289, 'f2_2': -61.02402, 'f2_3': -10.42427,
                          'f3_0': -4.77804, 'f3_1': -36.56903, 'f3_2': -79.58842, 'f3_3': -21.84747,
                          'f4_0': 5.74147,  'f4_1': -23.15380, 'f4_2': -95.27372, 'f4_3': -34.49945,
                          'f5_0': -0.74671, 'f5_1': -32.39618, 'f5_2': -77.16596, 'f5_3': -20.13846,
                          }
        for key in self.hand_data.keys():
            self.hand_data[key] = -self.hand_data[key]#手套的正负跟我的定义正好相反
        #文本框与关节对应关系，方便调用
        self.dict_jtname_lineEdit     = {"f1_0":self.lineEdit_f1_0, "f1_1":self.lineEdit_f1_1, "f1_2":self.lineEdit_f1_2, "f1_3":self.lineEdit_f1_3, \
                                         "f2_0":self.lineEdit_f2_0, "f2_1":self.lineEdit_f2_1, "f2_2":self.lineEdit_f2_2, "f2_3":self.lineEdit_f2_3, \
                                         "f3_0":self.lineEdit_f3_0, "f3_1":self.lineEdit_f3_1, "f3_2":self.lineEdit_f3_2, "f3_3":self.lineEdit_f3_3, \
                                         "f4_0":self.lineEdit_f4_0, "f4_1":self.lineEdit_f4_1, "f4_2":self.lineEdit_f4_2, "f4_3":self.lineEdit_f4_3, \
                                         "f5_0":self.lineEdit_f5_0, "f5_1":self.lineEdit_f5_1, "f5_2":self.lineEdit_f5_2, "f5_3":self.lineEdit_f5_3
                                         }

    #通过tcp连接数据手套
    def connect(self):
        if self.sock!=None:
            print "已经连接手套"
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('10.1.76.53',8888))#绑定本机地址与8888端口
        s.listen(1)
        print "Waiting for connection...."
        sock, addr=s.accept()
        self.sock=sock
        self.addr=addr
        print "accept connection from {}".format(addr)
        sock.send("welcome")
    def get_data(self):
        """
        get_data的方法：先发送一个字符串，包含"getdata"，然后调用sock.recv(2048)，尽量大些，虽然测试的一个数据只有609个bytes大小
        :return: void
        """
        #这句是临时加的，后面正式连接tcp时要去掉
        if self.hand_data!=None:
            for key in self.hand_data.keys():
                self.dict_jtname_lineEdit[key].setText( str(self.hand_data[key]) )#这里两个字典间的关系别搞混了，依靠相同的keys联系在一起
        if self.sock == None:
            return
        self.sock.send("getdata")#发送getdata，等待对方回应
        data=self.sock.recv(2048)
        if not data:
            print "未接收到数据"
            return
        self.hand_data = json.loads(data)#字符串转成字典对象
        #显示获得的数据
        #这里必须保证收到的数据里key与字典self.dict_jtname_lineEdit 的key一样，日后修改程序时一定要注意
        for key in self.hand_data.keys():
            self.dict_jtname_lineEdit[key].setText( str(self.hand_data[key]) )#这里两个字典间的关系别搞混了，依靠相同的keys联系在一起
    def connect_to_rviz(self):
        #TODO
        print "目前不知道moveit里显示关节空间规划结果的python API，所以这个功能暂时放弃"
    def connect_to_gazebo(self):
        #数据手套的数据直接连到rviz跟gazebo
            #角度值可能出现的问题，1数值填的不对，无法转成float;2数值太大，但这里不需要判断，因为hand_moveit.py里有解决；
        planned_group_names=[              "joint_f1_1", "joint_f1_2", "joint_f1_3",\
                             "joint_f2_0", "joint_f2_1", "joint_f2_2", "joint_f2_3",\
                             "joint_f3_0", "joint_f3_1", "joint_f3_2", "joint_f3_3",\
                             "joint_f4_0", "joint_f4_1", "joint_f4_2", "joint_f4_3",\
                             "joint_f5_0", "joint_f5_1", "joint_f5_2", "joint_f5_3" ]
        print "----------------------------------所有待规划的group为：", planned_group_names,"----------------"
        for group_name in planned_group_names:
            #以下关节先不用
            if group_name in ["joint_f2_0", "joint_f3_0", "joint_f4_0", "joint_f5_0",]:
                continue
            #hand_data的形式为：{'f1_0':0.1,...}
            for joint in self.hand_data.keys():
                #joint形式为f1_0,group_name形式为joint_f1_1
                if joint in group_name:#关节名对上了就规划
                    print "planned_grou_names: ", group_name
                    try:
                        jt_moveit = HandMoveGroup(group_name)
                        #手套的数据就是弧度制，所以不用转了--------6.30修改了，要转
                        jt_moveit.go_jt_state([ self.hand_data[joint]/180.0*3.14 ])
                    except:
                        print "规划有问题"
                    else:
                        print "规划完成"
        print "-------------------------------------------over--------------------------------------------------------------------------------------"

class Load_Save(QtWidgets.QDialog, load_save.Ui_Dialog):
    def __init__(self, parent):
        super(Load_Save, self).__init__()
        self.setupUi(self)
        self.parent=parent
        #用于保存打开的csv文件
        self.grasp_csv = None#type:pd.DataFrame
        self.joint_state = None

    def Load(self):
        fileName_choose, filetype = QFileDialog.getOpenFileName(self,
                                                                "选取文件",
                                                                os.getcwd(), # 起始路径
                                                                "All Files (*);;DataBase Files(*.db);;Sqlite(*.sqlite)")   # 设置文件扩展名过滤,用双分号间隔
        if fileName_choose == "":
            print("\n取消选择")
            return
        print fileName_choose
        self.grasp_csv = None#每次代开都确保上一次打开的文件都被delete
        self.grasp_csv=pd.read_csv(fileName_choose)#注意读取数据的类型，特别是整数的时候。表格形式是：f11 f12 ... f53 unit;后期换成sqlite3数据库
        shape_of_csv = self.grasp_csv.shape#类型为tuple[row,colum]
        self.tableWidget.setRowCount(shape_of_csv[0])
        self.tableWidget.setColumnCount(shape_of_csv[1])
        for row in range(shape_of_csv[0]):
            for colum in range(shape_of_csv[1]):
                self.tableWidget.setItem(row, colum, QTableWidgetItem( str(self.grasp_csv.iat[row,colum]) ))
        self.tableWidget.setHorizontalHeaderLabels(["f11","f12","f13","f20","f21","f22","f23","f30","f31","f32","f33","f4b","f40","f41","f42","f43","f5b","f50","f51","f52","f53","unit"])

        print("\n你选择的文件为:")
        print(fileName_choose)

        pass
    def Set(self):
        current_row = self.tableWidget.currentRow()#返回值从0开始算起,选择多行也只返回第一行
        print "选择的行数为：", current_row
        if current_row ==-1:
            print "请选择一行或文件"
            return
        #提取指定行的数字，然后设定相应关节角度
        for joint_name in self.grasp_csv.axes[1].to_list():#axes[1]是dataframe的列label，转成list
            if joint_name not in "f11f12f13f20f21f22f23f30f31f32f33f4bf40f41f42f43f5bf50f51f52f53":
                continue #不是关节位置 的话就返回，比如unit等
            planned_group_name = "joint_"+joint_name[0:-1]+"_"+joint_name[-1]#关节名f11转成规划组的名称joint_f1_1
            print "--------------------------------------set "+planned_group_name+"'s angle----------------------------------------------------------------"
            print "planned_grou_names: ", planned_group_name
            try:
                jt_moveit = HandMoveGroup(planned_group_name)
                if self.grasp_csv.loc[current_row, "unit"]=="rad":
                    angle = self.grasp_csv.loc[current_row, joint_name] #取相应关节角度值
                if self.grasp_csv.loc[current_row, "unit"]=="degree":
                    angle = self.grasp_csv.loc[current_row, joint_name] / 180.0 * 3.1415926 #取相应关节角度值
                jt_moveit.go_jt_state([angle]) #输入的角度值是弧度制
            except:
                print "规划有问题"
            else:
                print "规划完成"
        print "-------------------------------------------over--------------------------------------------------------------------------------------"

    def Read(self):
        ''
        '''
        Sensor_msgs::JointState类型为
                                    std_msgs/Header header
                                    string[] name
                                    float64[] position
                                    float64[] velocity
                                    float64[] effort
        '''
        joint_state = self.parent.jointStateMsg #type:JointState
        #降低精度
        joint_state.position=[round(x, 4) for x in joint_state.position]
        self.joint_state = joint_state
        #设置表格并显示当前关节角度
        num_of_joint = joint_state.name.__len__()
        self.tableWidget_tobesaved.setRowCount(1)#一行
        self.tableWidget_tobesaved.setColumnCount(num_of_joint+1)#最后一行补上单位
        for colum in range(num_of_joint):
            self.tableWidget_tobesaved.setItem(0,colum, QTableWidgetItem(str(round(joint_state.position[colum],4))))
        self.tableWidget_tobesaved.setItem(0,num_of_joint, QTableWidgetItem("degree"))#最后一行补上单位
        self.tableWidget_tobesaved.setHorizontalHeaderLabels(joint_state.name+["unit"])
        #保存到self.to_besaved_grasp
        # dict1={                            "jt_f1_1":0,  "jt_f1_2":1,  "jt_f1_3":2,
        #                      "jt_f2_0":3,  "jt_f2_1":4,  "jt_f2_2":5,  "jt_f2_3":6,
        #                      "jt_f3_0":7,  "jt_f3_1":8,  "jt_f3_2":9,  "jt_f3_3":10,
        #         "jt_f4_b":11,"jt_f4_0":12, "jt_f4_1":13, "jt_f4_2":14, "jt_f4_3":15,
        #         "jt_f5_b":16,"jt_f5_0":17, "jt_f5_1":18, "jt_f5_2":19, "jt_f5_3":20,
        #       }
    def Save(self):
        if self.joint_state ==None:
            return "先read"
        #读取需要保存到的csv文件名
        fileName_choose, filetype = QFileDialog.getOpenFileName(self,
                                                                "选取文件",
                                                                os.getcwd(), # 起始路径
                                                                "All Files (*);;DataBase Files(*.db);;Sqlite(*.sqlite)")   # 设置文件扩展名过滤,用双分号间隔
        grasp_csv=pd.read_csv(fileName_choose)#注意读取数据的类型，特别是整数的时候。表格形式是：f11 f12 ... f53 unit;后期换成sqlite3数据库
        shape_of_csv = grasp_csv.shape#类型为tuple[row,colum]
        #复制一行，然后复制为读取到的关节角度，单位为rad
        row1=copy.deepcopy(grasp_csv.head(1))#复制一行作为样式:f11 f12 ... f4b ..f53 unit
        for (jt_name, pos) in zip(self.joint_state.name, self.joint_state.position) :
            trimed_name = jt_name.replace("jt_","")#原来的名字形式为:jt_f1_1,改为f11
            trimed_name = trimed_name.replace("_","")
            if trimed_name not in ["f11","f12","f13","f20","f21","f22","f23","f30","f31","f32","f33","f4b","f40","f41","f42","f43","f5b","f50","f51","f52","f53","unit"]:
                print "error:修改名字错误"
                return
            row1.loc[0,trimed_name] = pos#这里按名字修改
        row1.loc[0, "unit"]="rad"
        #追加到csv文件中
        row1.to_csv(fileName_choose, index=False, mode="a", header=False)#注意写入的格式，要是追加模式，不要index跟header


class Robot_Arm_descartes(QtWidgets.QDialog, robot_arm_decarte.Ui_Dialog):
    def __init__(self, parent):
        super(Robot_Arm_descartes, self).__init__()
        self.setupUi(self)
        #构造机械臂的规划组;规划组名字为cor_arm,与配置moveit时指定的规划组名字相同
        self.ur5_moveGroup = HandMoveGroup("ur5")
        self.parent=parent
    def increase_angle(self):
        #旋转角为xyz定轴
        angle_x=self.dial_x.value()
        angle_y=self.dial_y.value()
        angle_z=self.dial_z.value()
        #按钮名字,判断增还是减
        sign=1
        btnName = self.sender().objectName()
        if "down" in btnName:
            sign=-1
        angle_x=sign*angle_x/180.0*3.1415926
        angle_y=sign*angle_y/180.0*3.1415926
        angle_z=sign*angle_z/180.0*3.1415926

        #判断是哪个按钮,对应哪个方向的旋转
        #旋转角的四元数
        angle=0
        if "x" in btnName:
            print "*******绕x轴旋转"
            angle=angle_x
            print "角度为：", angle*180.0/3.1415926
            q = np.quaternion( math.cos(angle/2), math.sin(angle/2), 0,                   0                   )
        if "y" in btnName:
            print "*******绕y轴旋转"
            angle=angle_y
            print "角度为：", angle*180.0/3.1415926
            q = np.quaternion( math.cos(angle/2), 0,                   math.sin(angle/2), 0                   )
        if "z" in btnName:
            print "*******绕z轴旋转"
            angle=angle_z
            print "角度为：", angle*180.0/3.1415926
            q = np.quaternion( math.cos(angle/2), 0,                   0,                   math.sin(angle/2) )

        # 规划
        #get当前位姿，位姿为msg，格式为：
        #Pose:position:x y z; orientation: x y z w
        pose_goal = self.ur5_moveGroup.move_group.get_current_pose().pose
        #当前位姿的四元数形式
        pos_quaternion = np.quaternion(pose_goal.orientation.w, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z)
        #开始旋转
        pos_quaternion = q*pos_quaternion
        #更新位姿的四元数
        pose_goal.orientation.w = pos_quaternion.w
        pose_goal.orientation.x = pos_quaternion.x
        pose_goal.orientation.y = pos_quaternion.y
        pose_goal.orientation.z = pos_quaternion.z

        #设置目标位姿,开始规划
        print "开始规划"
        self.ur5_moveGroup.move_group.set_pose_target(pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        plan = self.ur5_moveGroup.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.ur5_moveGroup.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.ur5_moveGroup.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        # current_pose = self.move_group.get_current_pose().pose
        print "--------------over---------------------------"

    def set_angle(self):
        joint=self.ur5_moveGroup.move_group.get_current_joint_values()
        print "机械臂关节角度为：", [round(x, 4) for x in joint]
        pose = self.ur5_moveGroup.move_group.get_current_pose().pose
        print "机械臂末端位置为：", pose.position.x, "  ", pose.position.y, "  ", pose.position.z
        jointState = self.parent.jointStateMsg#type:JointState
        print "手指关节角度："
        print jointState.name
        print [round(x, 4) for x in jointState.position ]
        print "------------------------------------over--------------------------------------"

        return
        #旋转角为xyz定轴
        angle_x=self.dial_x.value()
        angle_y=self.dial_y.value()
        angle_z=self.dial_z.value()
        angle_x=angle_x/180.0*3.1415926
        angle_y=angle_y/180.0*3.1415926
        angle_z=angle_z/180.0*3.1415926
        theta_str=self.lineEdit_theta.text()
        theta = 1
        try:
            theta = float(theta_str)
            theta = 1 if theta>=0 else -1#theat只取-1 1
        except :
            print "theta 值错误，取默认值: ", theta
        # 规划
        # return #先不做，四元数一直搞不清楚，先把这个搞清楚再说
        #get当前位姿，位姿为msg，格式为：
        #Pose:position:x y z; orientation: x y z w
        pose_goal = self.cor_arm_moveGroup.move_group.get_current_pose().pose
        #当前位姿的四元数形式
        pos_quaternion = np.quaternion(pose_goal.orientation.w, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z)
        #旋转角的四元数
        q_x = np.quaternion( math.cos(angle_x/2), math.sin(angle_x/2), 0,                   0                   )
        q_y = np.quaternion( math.cos(angle_x/2), 0,                   math.sin(angle_x/2), 0                   )
        q_z = np.quaternion( math.cos(angle_x/2), 0,                   0,                   math.sin(angle_x/2) )
        #开始旋转
        pos_quaternion = q_x*pos_quaternion*q_x**(-1)
        pos_quaternion = q_y*pos_quaternion*q_y**(-1)
        pos_quaternion = q_z*pos_quaternion*q_z**(-1)
        #更新位姿的四元数
        pose_goal.orientation.w = pos_quaternion.w
        pose_goal.orientation.x = pos_quaternion.x
        pose_goal.orientation.y = pos_quaternion.y
        pose_goal.orientation.z = pos_quaternion.z

        #设置目标位姿,开始规划
        self.cor_arm_moveGroup.move_group.set_pose_target(pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        plan = self.cor_arm_moveGroup.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.cor_arm_moveGroup.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.cor_arm_moveGroup.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        # current_pose = self.cor_arm_moveGroup.move_group.get_current_pose().pose

    def set_xyz_by_teach(self, position):
        #直接设定末端位置，position为[x,y,z],机械臂运动结束才会退出本函数
        if type(position) is not list or position.__len__() is not 3:
            print "fatal error, position 类型或者长度错误"
        ##把需要的中间点append到waypoints里就行了
        waypoints = []

        wpose = self.ur5_moveGroup.move_group.get_current_pose().pose
        # 确定步长，主要是正负号的处理
        # step_normal = 0.1#步长大小默认为0.1
        steps = 10.0  # 步数，定为float，方便做除法
        step=[0,0,0]
        step[0]=position[0]-wpose.position.x# 带正负号的步长，方便添加中间点
        step[1]=position[1]-wpose.position.y
        step[2]=position[2]-wpose.position.z

        for i in range(int(steps)):
            wpose.position.x += step[0]
            wpose.position.y += step[1]
            wpose.position.z += step[2]
            waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.ur5_moveGroup.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        # print "***************计算规划路径结束", time.time()%1000, "********************"
        self.ur5_moveGroup.move_group.execute(plan, wait=True)
        # print "***************执行规划结束", time.time()%1000, "********************"

    def set_xyz(self):
        #虽然名字是set_xyz,但实际是以增量形式设定位置
        # print "***************函数set_xyz开始", time.time()%1000, "********************"
        sender = self.sender()
        increase_or_decrease =1
        btnName=sender.objectName()#获取按钮名字，确定是哪个关节动，初始的名字形式为pushButton_joint_0_up
        if "down" in btnName:       #button名字含down则表示角度要减小
            increase_or_decrease = -1 
            print "坐标减小"
        #获取delta值然后转为数字
        delta_str=self.lineEdit_delta.text()
        delta = 0.1
        try:
            delta  = float(delta_str)
        except :
            print "delta 值错误，取默认值: ", delta
        cordinate_delta = [0, 0, 0]
        for (axis, i) in zip(["Button_x", "Button_y", "Button_z"], [0,1,2]):
            if axis in btnName:
                cordinate_delta[i]=delta*increase_or_decrease
        # 规划
        self.plan_execute_cartesian_path(cordinate_delta)
    
    #这个函数规划并执行机械臂末端的笛卡尔轨迹,参数是末端的位移
    def plan_execute_cartesian_path(self, end_delta):
        # print "***************计算规划路径开始", time.time()%1000, "********************"
        if type(end_delta) is not list and end_delta.__len__()!=3:
            print "end_delta 长度跟类型有问题"
            return
        #确定步长，主要是正负号的处理
        # step_normal = 0.1#步长大小默认为0.1
        steps=10.0#步数，定为float，方便做除法
        step=[x/steps for x in end_delta]#带正负号的步长，方便添加中间点

        ##把需要的中间点append到waypoints里就行了
        waypoints = []
        planned_group_name = "cor_arm"

        wpose = self.ur5_moveGroup.move_group.get_current_pose().pose
        for i in range(int(steps)):
            wpose.position.x += step[0]
            wpose.position.y += step[1]
            wpose.position.z += step[2]
            waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.ur5_moveGroup.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        # print "***************计算规划路径结束", time.time()%1000, "********************"
        self.ur5_moveGroup.move_group.execute(plan, wait=True)
        # print "***************执行规划结束", time.time()%1000, "********************"


        


class Robot_Arm_joint(QtWidgets.QDialog, Ui_robot_arm_joint.Ui_Dialog):
    def __init__(self):
        super(Robot_Arm_joint, self).__init__()
        self.setupUi(self)
        #构造机械臂的规划组;规划组名字为cor_arm,与配置moveit时指定的规划组名字相同
        self.ur5_moveGroup = HandMoveGroup("ur5")

    def increase_joint_angle(self):
        print  "\n\n\n"
        sender = self.sender()
        increase_or_decrease =1
        btnName=sender.objectName()#获取按钮名字，确定是哪个关节动，初始的名字形式为pushButton_joint_0_up
        if "down" in btnName:       #button名字含down则表示角度要减小
            increase_or_decrease = -1 
            print "关节角度减小"
        #匹配正则字符串，去除后面的_up跟_down
        btnName = re.match(r'pushButton_joint[\d]', btnName).group() #type: unicode
        delta_str = self.lineEdit_delta.text()
        delta=10#默认是10度
        try:
            delta = float(delta_str)
        except :
            print "delta有问题，默认角度增量为",delta,"度"
        increase_angle_list=[0]*7#7个关节的角度增量
        joint_num=None#type: int;要规划的关节
        try:
            joint_num=int( re.search(r'\d', btnName).group() )#匹配按钮名字里的数字，从而确定要规划哪个关节
            increase_angle_list[joint_num]=increase_or_decrease * delta / 180.0 * 3.1415926#转为弧度制
        except:
            print "确定关节编号错误"
            return
        # planned_group_name = "cor_arm"
        print "--------------------------------------set arm joint"+str(joint_num)+" angle----------------------------------------------------------------"
        try:
            # print "***************MoveGroup对象构造", time.time()%1000, "********************"
            # jt_moveit = HandMoveGroup(planned_group_name )
            print "***************执行规划开始", time.time()%1000, "********************"
            # jt_moveit.increase_jt_state(increase_angle_list)
            self.ur5_moveGroup.increase_jt_state(increase_angle_list)
            print "***************执行规划结束", time.time()%1000, "********************"
        except:
            print "规划有问题"
        else:
            print "规划完成"
        print "-------------------------------------------over--------------------------------------------------------------------------------------"

    def set_all_angle_by_teach_nstep(self, anglelist, steps = 10, delta = 0.002 ):
        #分小步完成，否则容易仿真出错，delta=0.002验证过没问题
        anglelist_line = []#目标位置做个插值anglelist_line[i]对应一个关节
        current_angle = self.ur5_moveGroup.move_group.get_current_joint_values()
        #做个线性插值,步长0.002
        for cur, target in zip(current_angle, anglelist):
            # temp = np.arange(cur, target, delta).tolist()
            # temp = temp.append(target)
            # anglelist_line.append(temp)
            anglelist_line.append( np.linspace(cur, target, steps, True) )
        for coloum in range(anglelist_line[0].__len__()):
            coloum_value = [x[coloum] for x in anglelist_line]

            self.set_all_angle_by_teach(coloum_value)


    def set_all_angle_by_teach(self, anglelist):
        #直接设定角度，用于示教方式, anglelist为列表，值为弧度制
        if type(anglelist) is not list or anglelist.__len__() is not 7:
            print "erro 输入参数类型或者长度不对"
        try:
            self.ur5_moveGroup.go_jt_state(anglelist)
        except:
            print "规划有问题"
        else:
            print "规划完成"
        print "-------------------------------------------over--------------------------------------------------------------------------------------"

    def set_all_angle(self):
        #角度值可能出现的问题，1数值填的不对，无法转成float;2数值太大，但这里不需要判断，因为hand_moveit.py里有解决；
        anglelist_str = [self.lineEdit_joint0.text(), self.lineEdit_joint1.text(), self.lineEdit_joint2.text(), 
                         self.lineEdit_joint3.text(), self.lineEdit_joint4.text(), self.lineEdit_joint5.text(),
                         self.lineEdit_joint6.text()]
        anglelist=[None]*7
        digit_exist=None#flag:判断是否有一个转float成功
        for i in range(anglelist_str.__len__()):
            try:
                # print anglelist_str
                anglelist[i]=float(anglelist_str[i]) / 180.0 * 3.1415926#直接转弧度制
                digit_exist=True
            except:
                # print "输入数字有问题，请检查设定的角度值."
                anglelist[i]=None
                # return
        if digit_exist==None:
            print "没有设定角度值或者角度值有问题"
            return
        # planned_group_name = "cor_arm"
        print "--------------------------------------set all arm joint angle----------------------------------------------------------------"
        try:
            # jt_moveit = HandMoveGroup(planned_group_name )
            # jt_moveit.go_jt_state(anglelist)
            self.ur5_moveGroup.go_jt_state(anglelist)
        except:
            print "规划有问题"
        else:
            print "规划完成"
        print "-------------------------------------------over--------------------------------------------------------------------------------------"
    



class Identity(QtWidgets.QDialog, Ui_Identity.Ui_Dialog):
    def __init__(self):
        super(Identity, self).__init__()
        self.setupUi(self)
        #按钮名称与要设定的关节名称对应关系
        self.angle_increase_btn = {u'pushButton_jt_f10': "jt_f1_0", u'pushButton_jt_f11': "jt_f1_1", u'pushButton_jt_f12': "jt_f1_2", u'pushButton_jt_f13': "jt_f1_3",\
                                   u'pushButton_jt_f20': "jt_f2_0", u'pushButton_jt_f21': "jt_f2_1", u'pushButton_jt_f22': "jt_f2_2", u'pushButton_jt_f23': "jt_f2_3",\
                                   u'pushButton_jt_f30': "jt_f3_0", u'pushButton_jt_f31': "jt_f3_1", u'pushButton_jt_f32': "jt_f3_2", u'pushButton_jt_f33': "jt_f3_3",\
                                   u'pushButton_jt_f40': "jt_f4_0", u'pushButton_jt_f41': "jt_f4_1", u'pushButton_jt_f42': "jt_f4_2", u'pushButton_jt_f43': "jt_f4_3",\
                                   u'pushButton_jt_f50': "jt_f5_0", u'pushButton_jt_f51': "jt_f5_1", u'pushButton_jt_f52': "jt_f5_2", u'pushButton_jt_f53': "jt_f5_3"}

    def set_jt_angle_by_teach(self, msg):
        #直接设定角度，用于示教方式，msg为字典,pos值为弧度制
        msg = msg#type:dict({"joint":"jt_f1_1", "pos":0})
        #启动一个ros节点，应该是局部变量，启动时间100ms左右
        pub = rospy.Publisher('/set_hand_pos_target', String, queue_size=10)
        #要发送的消息，字典转json即可
        #窗口输入是角度，gazebo接收的是弧度
        msg = json.dumps(msg)
        for i in range(5):
            pub.publish(msg)
            # rate.sleep()
            time.sleep(0.01)
        print "-------------------------------------------设置手关节角度over--------------------------------------------------------------------------------------"
    def increase_jt_angle(self):
        #这里有个设计问题，set按钮应该另开一个槽函数的，日后有机会可以优化下
        sender = self.sender()
        increase_or_decrease =1
        btnName=sender.objectName()#获取按钮名字，确定是哪个关节动，初始的名字形式为pushButton_jt_0_up
        fingerName = self.comboBox_finger.currentText() #获得手指名，确定是哪个手指
        #直接设置关节角度值
        if btnName=="pushButton_set":
            #角度值可能出现的问题，1数值填的不对，无法转成float;2数值太大，但这里不需要判断，因为hand_moveit.py里有解决；
            anglelist_str = [self.lineEdit_jt_b.text(), self.lineEdit_jt_0.text(), self.lineEdit_jt_1.text(), self.lineEdit_jt_2.text(), self.lineEdit_jt_3.text()]
            try:
                # print anglelist_str
                anglelist=[float(x) for x in anglelist_str]
            except:
                print "输入数字有问题，请检查设定的角度值."
                return
            planned_joint_names = None
            if fingerName=="Finger1":
                planned_joint_names=["jt_f1_0", "jt_f1_1", "jt_f1_2", "jt_f1_3"]
            if fingerName=="Finger2":
                planned_joint_names=["jt_f2_0", "jt_f2_1", "jt_f2_2", "jt_f2_3"]
            if fingerName=="Finger3":
                planned_joint_names=["jt_f3_0", "jt_f3_1", "jt_f3_2", "jt_f3_3"]
            if fingerName=="Finger4":
                planned_joint_names=["jt_f4_0", "jt_f4_1", "jt_f4_2", "jt_f4_3"]
            if fingerName=="Finger5":
                planned_joint_names=["jt_f5_0", "jt_f5_1", "jt_f5_2", "jt_f5_3"]
            if planned_joint_names==None:
                print "planned_joint_names有问题，请检查"
                return
            anglelist=anglelist[5-planned_joint_names.__len__()::]#去除不必要的角度值,让角度值跟planned_joint_names相对应
            #启动一个ros节点，应该是局部变量，启动时间100ms左右
            pub = rospy.Publisher('/set_hand_pos_target', String, queue_size=10)
            # rospy.init_node('set_hand_pos_from_Identity_window')
            rate = rospy.Rate(10)
            #要发送的消息，字典转json即可
            msg = dict()
            for key, pos in zip(planned_joint_names, anglelist) :
                #窗口输入是角度，gazebo接收的是弧度
                msg[key] = pos/180.0*3.1415926
            msg = json.dumps(msg)
            for i in range(5):
                pub.publish(msg)
                # rate.sleep()
                time.sleep(0.01)
            print "-------------------------------------------over--------------------------------------------------------------------------------------"
            return #结束这个按钮的响应

        # if "_jt_" not in btnName:
        #     print "不是修改关节角度的按钮，TODO"
        #     return
        # print "button name is:", btnName
        if "down" in btnName:       #button名字含down则表示角度要减小
            increase_or_decrease = -1 
            # print "关节角度减小"
        #匹配正则字符串，去除后面的_up跟_down
        btnName = re.match(r'pushButton_jt_[b\d]', btnName).group() #type: unicode

        if fingerName=="Finger1":   #关节名添上手指f\d；使用正则替换
            btnName = re.sub(r'pushButton_jt_', 'pushButton_jt_f1', btnName)
        if fingerName=="Finger2":
            btnName = re.sub(r'pushButton_jt_', 'pushButton_jt_f2', btnName)
        if fingerName=="Finger3":
            btnName = re.sub(r'pushButton_jt_', 'pushButton_jt_f3', btnName)
        if fingerName=="Finger4":
            btnName = re.sub(r'pushButton_jt_', 'pushButton_jt_f4', btnName)
        if fingerName=="Finger5":
            btnName = re.sub(r'pushButton_jt_', 'pushButton_jt_f5', btnName)
        print "button name is:", btnName
        planned_joint_name = None  #要设定的关节名字初始为None，如果检查要设定的关节时还是NONE，说明中间出问题了
        if btnName in self.angle_increase_btn.keys():
            planned_joint_name = self.angle_increase_btn[btnName]   #确定要设定的关节名称
        print "要设定的关节名称：　", planned_joint_name if planned_joint_name else "要设定的关节名称错误"
        if planned_joint_name==None:
            return
        # return
        # set delta
        delta_str= self.lineEdit_delta_jtAngle.text()
        delta =10 #unit degree，默认值10度
        try:
            delta =float(delta_str)
        except:
            print "输入数字有问题"
            print "delta 取默认值：", delta
        else:
            print "没毛病，老铁","一次按钮点击步长为: " ,delta, " degree"

        #启动一个ros节点，应该是局部变量，启动时间100ms左右
        pub = rospy.Publisher('/increase_hand_pos_target', String, queue_size=10)
        # rospy.init_node('increase_hand_pos_from_Identity_window')
        rate = rospy.Rate(10)
        # plan
        if planned_joint_name !=None:
            # check the angle in [-pi/2, pi/2]
            if abs(delta)>90:
                print "delta is larger than 90degree"
                return
            #要发送的消息，字典转json即可
            msg = dict()
            #窗口输入是角度，gazebo接收的是弧度
            msg[planned_joint_name] =increase_or_decrease*delta/180.0*3.1415926
            msg = json.dumps(msg)
            for i in range(5):
                pub.publish(msg)
                # rate.sleep()
                time.sleep(0.01)
            # print "-------------------------------------------over--------------------------------------------------------------------------------------"

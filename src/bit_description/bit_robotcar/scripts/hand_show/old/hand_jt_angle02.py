# -*- coding: utf-8 -*-

# Form implementation generat ed from reading ui file 'hand_jt_angle.ui'
#
# Created by: PyQt5 UI code generator 5.6
#
# WARNING! All changes made in this file will be lost!
#不需要多线程
from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from std_msgs.msg import String

class Worker(QtCore.QThread):
    sinOut = QtCore.pyqtSignal() # 自定义信号，执行run()函数时，从相关线程发射此信号

    def __init__(self, parent=None):
        print "thread init..."
        super(Worker, self).__init__(parent)
        self.working = True
        self.num = 0

    def run(self):
        # self.listener()
        while self.working == True:
            print "thread run"
            # 发出信号
            self.sinOut.emit()

            # 线程休眠1秒
            self.sleep(1)

    # def listener(self):
    #     rospy.init_node('listener',anonymous=True)
    #     rospy.Subscriber('chatter', String, self.callback)
    #     rospy.spin()
    #
    # def callback(self, data):
    #     rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    #     print "callback函数"

class Ui_MainWindow(object):
    def __init__(self, MainWindow):
        self.setupUi(MainWindow)
        self.setupAllFunction() #把所有的槽函数绑定信号
        #启动订阅器，不需要加rospy.spin()
        self.listener()
        # print "接着启动线程，然后运行主窗口"
        # self.worker.start()
    def setupAllFunction(self):
        #绑定按钮跟响应函数
        self.pushButton_jt_f11_up.clicked.connect(self.jt_f11_angle_up)
        self.pushButton_jt_f11_down.clicked.connect(self.jt_f11_angle_down)
        #绑定新线程里的信号与响应函数,这里创建了一个新的线程
        # self.worker = Worker()
        # self.worker.sinOut.connect(self.slot_update_jtAngle)

    def jt_f11_angle_up(self):
        str=self.lineEdit_jt_f11.text()
        self.lineEdit_jt_f11.setText("角度增加")

    def jt_f11_angle_down(self):
        str=self.lineEdit_jt_f11.text()
        print u"原来的内容是:"+str
        self.lineEdit_jt_f11.setText("角度减少")

    # def slot_update_jtAngle(self):
    #     '''
    #     接收ｒｏｓ消息，更新显示数据
    #     :return:
    #     '''
    #     # self.lineEdit_jt_f11.setText(u"接收ros,update")
    #     print "线程的信号"

    def listener(self):
        rospy.init_node('listener',anonymous=True)
        rospy.Subscriber('chatter', String, self.callback)
        # rospy.spin()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        print "callback函数"


    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1045, 687)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(60, 60, 81, 16))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(170, 60, 81, 16))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(300, 60, 81, 16))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(430, 60, 81, 16))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(540, 60, 81, 16))
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setGeometry(QtCore.QRect(670, 60, 81, 16))
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setGeometry(QtCore.QRect(810, 60, 81, 16))
        self.label_7.setObjectName("label_7")
        self.lineEdit_jt_f11 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_jt_f11.setGeometry(QtCore.QRect(60, 90, 81, 22))
        self.lineEdit_jt_f11.setReadOnly(True)
        self.lineEdit_jt_f11.setObjectName("lineEdit_jt_f11")
        self.lineEdit__jt_f12 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit__jt_f12.setGeometry(QtCore.QRect(170, 90, 81, 22))
        self.lineEdit__jt_f12.setReadOnly(True)
        self.lineEdit__jt_f12.setObjectName("lineEdit__jt_f12")
        self.lineEdit__jt_f13 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit__jt_f13.setGeometry(QtCore.QRect(300, 90, 81, 22))
        self.lineEdit__jt_f13.setReadOnly(True)
        self.lineEdit__jt_f13.setObjectName("lineEdit__jt_f13")
        self.lineEdit__jt_f20 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit__jt_f20.setGeometry(QtCore.QRect(430, 90, 81, 22))
        self.lineEdit__jt_f20.setReadOnly(True)
        self.lineEdit__jt_f20.setObjectName("lineEdit__jt_f20")
        self.lineEdit__jt_f21 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit__jt_f21.setGeometry(QtCore.QRect(540, 90, 81, 22))
        self.lineEdit__jt_f21.setReadOnly(True)
        self.lineEdit__jt_f21.setObjectName("lineEdit__jt_f21")
        self.lineEdit__jt_f22 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit__jt_f22.setGeometry(QtCore.QRect(670, 90, 81, 22))
        self.lineEdit__jt_f22.setReadOnly(True)
        self.lineEdit__jt_f22.setObjectName("lineEdit__jt_f22")
        self.lineEdit__jt_f23 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit__jt_f23.setGeometry(QtCore.QRect(810, 90, 81, 22))
        self.lineEdit__jt_f23.setReadOnly(True)
        self.lineEdit__jt_f23.setObjectName("lineEdit__jt_f23")
        self.pushButton_jt_f11_up = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_jt_f11_up.setGeometry(QtCore.QRect(60, 30, 80, 22))
        self.pushButton_jt_f11_up.setObjectName("pushButton_jt_f11_up")
        self.pushButton_jt_f11_down = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_jt_f11_down.setGeometry(QtCore.QRect(60, 140, 80, 22))
        self.pushButton_jt_f11_down.setObjectName("pushButton_jt_f11_down")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1045, 19))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "jt_f11_angle"))
        self.label_2.setText(_translate("MainWindow", "jt_f12_angle"))
        self.label_3.setText(_translate("MainWindow", "jt_f13_angle"))
        self.label_4.setText(_translate("MainWindow", "jt_f20_angle"))
        self.label_5.setText(_translate("MainWindow", "jt_f21_angle"))
        self.label_6.setText(_translate("MainWindow", "jt_f22_angle"))
        self.label_7.setText(_translate("MainWindow", "jt_f23_angle"))
        self.pushButton_jt_f11_up.setText(_translate("MainWindow", "angle +"))
        self.pushButton_jt_f11_down.setText(_translate("MainWindow", "angle -"))


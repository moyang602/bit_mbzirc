#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
from PyQt5 import QtWidgets,uic, QtCore
import sys
from ui import hand_main_window
import rospy
from std_msgs.msg import String, Float64MultiArray
import time
from hand_moveit import HandMoveGroup

class MainWindow(QtWidgets.QMainWindow, hand_main_window.Ui_MainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self.setmyStyleSheet()

    def setmyStyleSheet(self):
        # self.setStyleSheet('''
        #     QGroupBox{border:none}''')
        self.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.centralwidget.setStyleSheet('''
            #groupBox_2,#groupBox_4,#groupBox_5,#groupBox_6,#groupBox_7{
                        border:none
                        }
            QLineEdit{  border-radius:4px;
                        border:1px solid gray;
                        width:300px;
                        padding:2px 4px;
                                }
            QLabel{
                    }
            ''')
        self.contact_widget.setStyleSheet('''
            QComboBox{border:0px;}
            QTextEdit{
                      border:1px solid gray;
                        }
            ''')
        self.setWindowOpacity(0.9)
        # self.setAttribute(QtCore.Qt.WA_TranslucentBackground)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

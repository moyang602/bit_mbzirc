# -*- coding: utf-8 -*-
from old import hand_jt_angle02
from PyQt5 import QtWidgets
import sys

app=QtWidgets.QApplication(sys.argv) #主应用
MainWindow=QtWidgets.QMainWindow()#初始化一个主窗口
ui= hand_jt_angle02.Ui_MainWindow(MainWindow)#用界面里的类实例化一个窗口
MainWindow.show()
sys.exit(app.exec_())

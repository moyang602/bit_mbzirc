# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'load_save.ui'
#
# Created by: PyQt5 UI code generator 5.6
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(1300, 454)
        Dialog.setStyleSheet("background:white")
        self.widget = QtWidgets.QWidget(Dialog)
        self.widget.setGeometry(QtCore.QRect(10, 260, 1241, 173))
        self.widget.setObjectName("widget")
        self.gridLayout = QtWidgets.QGridLayout(self.widget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton_save = QtWidgets.QPushButton(self.widget)
        self.pushButton_save.setStyleSheet("QPushButton{border:none;border-bottom:1px solid white;font-size:15px;\n"
"font-weight:700;}\n"
"QPushButton:hover{font-size:18px;}")
        icon = QtGui.QIcon.fromTheme("fa.download")
        self.pushButton_save.setIcon(icon)
        self.pushButton_save.setObjectName("pushButton_save")
        self.gridLayout.addWidget(self.pushButton_save, 1, 0, 1, 1)
        self.pushButton_read = QtWidgets.QPushButton(self.widget)
        self.pushButton_read.setMinimumSize(QtCore.QSize(100, 0))
        self.pushButton_read.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.pushButton_read.setStyleSheet("QPushButton{border:none;border-bottom:1px solid white;font-size:15px;\n"
"font-weight:700;}\n"
"QPushButton:hover{font-size:18px;}")
        icon = QtGui.QIcon.fromTheme("fa.download")
        self.pushButton_read.setIcon(icon)
        self.pushButton_read.setObjectName("pushButton_read")
        self.gridLayout.addWidget(self.pushButton_read, 0, 0, 1, 1)
        self.tableWidget_tobesaved = QtWidgets.QTableWidget(self.widget)
        self.tableWidget_tobesaved.setObjectName("tableWidget_tobesaved")
        self.tableWidget_tobesaved.setColumnCount(0)
        self.tableWidget_tobesaved.setRowCount(0)
        self.gridLayout.addWidget(self.tableWidget_tobesaved, 0, 1, 2, 1)
        self.gridLayout.setRowStretch(0, 1)
        self.widget1 = QtWidgets.QWidget(Dialog)
        self.widget1.setGeometry(QtCore.QRect(10, 20, 1241, 261))
        self.widget1.setObjectName("widget1")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.widget1)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setVerticalSpacing(6)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.pushButton_set = QtWidgets.QPushButton(self.widget1)
        self.pushButton_set.setMaximumSize(QtCore.QSize(100, 16777215))
        self.pushButton_set.setStyleSheet("QPushButton{border:none;border-bottom:1px solid white;font-size:15px;\n"
"font-weight:700;}\n"
"QPushButton:hover{font-size:18px;}")
        icon = QtGui.QIcon.fromTheme("fa.download")
        self.pushButton_set.setIcon(icon)
        self.pushButton_set.setObjectName("pushButton_set")
        self.gridLayout_2.addWidget(self.pushButton_set, 1, 0, 1, 1)
        self.pushButton_load = QtWidgets.QPushButton(self.widget1)
        self.pushButton_load.setStyleSheet("QPushButton{border:none;border-bottom:1px solid white;font-size:15px;\n"
"font-weight:700;}\n"
"QPushButton:hover{font-size:18px;}")
        icon = QtGui.QIcon.fromTheme("fa.download")
        self.pushButton_load.setIcon(icon)
        self.pushButton_load.setObjectName("pushButton_load")
        self.gridLayout_2.addWidget(self.pushButton_load, 0, 0, 1, 1)
        self.lineEdit_num_of_grasp = QtWidgets.QLineEdit(self.widget1)
        self.lineEdit_num_of_grasp.setMaximumSize(QtCore.QSize(100, 16777215))
        self.lineEdit_num_of_grasp.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit_num_of_grasp.setObjectName("lineEdit_num_of_grasp")
        self.gridLayout_2.addWidget(self.lineEdit_num_of_grasp, 2, 0, 1, 1)
        self.tableWidget = QtWidgets.QTableWidget(self.widget1)
        self.tableWidget.setObjectName("tableWidget")
        self.tableWidget.setColumnCount(0)
        self.tableWidget.setRowCount(0)
        self.gridLayout_2.addWidget(self.tableWidget, 0, 1, 3, 1)

        self.retranslateUi(Dialog)
        self.pushButton_load.clicked.connect(Dialog.Load)
        self.pushButton_set.clicked.connect(Dialog.Set)
        self.pushButton_read.clicked.connect(Dialog.Read)
        self.pushButton_save.clicked.connect(Dialog.Save)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.pushButton_save.setText(_translate("Dialog", "Save"))
        self.pushButton_read.setText(_translate("Dialog", "Read"))
        self.pushButton_set.setText(_translate("Dialog", "Set"))
        self.pushButton_load.setText(_translate("Dialog", "Load"))
        self.lineEdit_num_of_grasp.setPlaceholderText(_translate("Dialog", "num of grasp"))


# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mantra_hmi.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(266, 142)
        Form.setMinimumSize(QtCore.QSize(0, 0))
        Form.setMaximumSize(QtCore.QSize(500, 500))
        Form.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        Form.setAutoFillBackground(False)
        self.powerButton = QtGui.QPushButton(Form)
        self.powerButton.setGeometry(QtCore.QRect(130, 6, 132, 132))
        self.powerButton.setMinimumSize(QtCore.QSize(30, 30))
        self.powerButton.setMaximumSize(QtCore.QSize(200, 200))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(16)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.powerButton.setFont(font)
        self.powerButton.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.powerButton.setObjectName(_fromUtf8("powerButton"))
        self.resetButton = QtGui.QPushButton(Form)
        self.resetButton.setGeometry(QtCore.QRect(4, 6, 120, 64))
        self.resetButton.setMinimumSize(QtCore.QSize(96, 30))
        self.resetButton.setMaximumSize(QtCore.QSize(120, 120))
        self.resetButton.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.resetButton.setObjectName(_fromUtf8("resetButton"))
        self.setHomeButton = QtGui.QPushButton(Form)
        self.setHomeButton.setGeometry(QtCore.QRect(4, 74, 120, 64))
        self.setHomeButton.setMinimumSize(QtCore.QSize(96, 30))
        self.setHomeButton.setMaximumSize(QtCore.QSize(120, 120))
        self.setHomeButton.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.setHomeButton.setObjectName(_fromUtf8("setHomeButton"))

        self.retranslateUi(Form)
        QtCore.QObject.connect(self.powerButton, QtCore.SIGNAL(_fromUtf8("pressed()")), Form.power)
        QtCore.QObject.connect(self.resetButton, QtCore.SIGNAL(_fromUtf8("pressed()")), Form.reset_arm)
        QtCore.QObject.connect(self.setHomeButton, QtCore.SIGNAL(_fromUtf8("pressed()")), Form.set_home)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Mantra HMI", None))
        self.powerButton.setText(_translate("Form", "Power OFF", None))
        self.resetButton.setText(_translate("Form", "Reset Arm", None))
        self.setHomeButton.setText(_translate("Form", "Set Home", None))


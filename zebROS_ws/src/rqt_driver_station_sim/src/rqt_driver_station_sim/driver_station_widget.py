# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'driverStationSim.ui'
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

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(862, 413)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setSizeConstraint(QtGui.QLayout.SetMinimumSize)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.practice_button = QtGui.QPushButton(self.centralwidget)
        self.practice_button.setMinimumSize(QtCore.QSize(0, 30))
        self.practice_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.practice_button.setFocusPolicy(QtCore.Qt.NoFocus)
        self.practice_button.setStyleSheet(_fromUtf8("background-color: rgba(208, 208, 208, 255);\n"
"selection-background-color: rgba(255, 255, 255, 0);\n"
"selection-color: rgba(255, 255, 255, 0);"))
        self.practice_button.setCheckable(True)
        self.practice_button.setObjectName(_fromUtf8("practice_button"))
        self.verticalLayout_2.addWidget(self.practice_button)
        self.teleop_button = QtGui.QPushButton(self.centralwidget)
        self.teleop_button.setMinimumSize(QtCore.QSize(0, 30))
        self.teleop_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.teleop_button.setFocusPolicy(QtCore.Qt.NoFocus)
        self.teleop_button.setStyleSheet(_fromUtf8("background-color: rgba(208, 208, 208, 255);\n"
"selection-background-color: rgba(255, 255, 255, 0);\n"
"selection-color: rgba(255, 255, 255, 0);\n"
""))
        self.teleop_button.setCheckable(True)
        self.teleop_button.setObjectName(_fromUtf8("teleop_button"))
        self.verticalLayout_2.addWidget(self.teleop_button)
        self.auto_mode = QtGui.QPushButton(self.centralwidget)
        self.auto_mode.setMinimumSize(QtCore.QSize(0, 30))
        self.auto_mode.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.auto_mode.setFocusPolicy(QtCore.Qt.NoFocus)
        self.auto_mode.setStyleSheet(_fromUtf8("background-color: rgba(208, 208, 208, 255);\n"
"selection-color: rgb(66, 255, 183);\n"
"selection-background-color: rgb(255, 249, 69);"))
        self.auto_mode.setCheckable(True)
        self.auto_mode.setObjectName(_fromUtf8("auto_mode"))
        self.verticalLayout_2.addWidget(self.auto_mode)
        self.verticalLayout.addLayout(self.verticalLayout_2)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.enable_button = QtGui.QRadioButton(self.centralwidget)
        self.enable_button.setMinimumSize(QtCore.QSize(0, 70))
        self.enable_button.setSizeIncrement(QtCore.QSize(0, 1))
        self.enable_button.setFocusPolicy(QtCore.Qt.NoFocus)
        self.enable_button.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.enable_button.setAcceptDrops(True)
        self.enable_button.setStyleSheet(_fromUtf8("border-left-color: rgb(131, 255, 229);\n"
"background-color: rgb(61, 255, 8);\n"
"border-top-color: rgb(57, 120, 255);\n"
"selection-color: rgb(255, 112, 248);\n"
"selection-background-color: rgb(76, 148, 255);\n"
"border-color: rgb(255, 161, 129);"))
        self.enable_button.setIconSize(QtCore.QSize(60, 60))
        self.enable_button.setCheckable(True)
        self.enable_button.setObjectName(_fromUtf8("enable_button"))
        self.horizontalLayout_2.addWidget(self.enable_button)
        self.disable_button = QtGui.QRadioButton(self.centralwidget)
        self.disable_button.setMinimumSize(QtCore.QSize(0, 70))
        self.disable_button.setSizeIncrement(QtCore.QSize(0, 1))
        self.disable_button.setFocusPolicy(QtCore.Qt.NoFocus)
        self.disable_button.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.disable_button.setAcceptDrops(True)
        self.disable_button.setStyleSheet(_fromUtf8("background-color: rgb(255, 17, 40);\n"
"selection-background-color: rgba(255, 255, 255, 0);\n"
"selection-color: rgba(255, 255, 255, 0);\n"
""))
        self.disable_button.setIconSize(QtCore.QSize(60, 60))
        self.disable_button.setCheckable(True)
        self.disable_button.setChecked(True)
        self.disable_button.setObjectName(_fromUtf8("disable_button"))
        self.horizontalLayout_2.addWidget(self.disable_button)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.frame = QtGui.QFrame(self.centralwidget)
        self.frame.setMaximumSize(QtCore.QSize(350, 260))
        self.frame.setFrameShape(QtGui.QFrame.NoFrame)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setLineWidth(0)
        self.frame.setObjectName(_fromUtf8("frame"))
        self.textBrowser = QtGui.QTextBrowser(self.frame)
        self.textBrowser.setGeometry(QtCore.QRect(190, 10, 101, 31))
        self.textBrowser.setObjectName(_fromUtf8("textBrowser"))
        self.match_data = QtGui.QLineEdit(self.frame)
        self.match_data.setGeometry(QtCore.QRect(298, 10, 41, 31))
        self.match_data.setObjectName(_fromUtf8("match_data"))
        self.textBrowser_2 = QtGui.QTextBrowser(self.frame)
        self.textBrowser_2.setGeometry(QtCore.QRect(22, 130, 101, 31))
        self.textBrowser_2.setObjectName(_fromUtf8("textBrowser_2"))
        self.textBrowser_3 = QtGui.QTextBrowser(self.frame)
        self.textBrowser_3.setGeometry(QtCore.QRect(22, 170, 101, 31))
        self.textBrowser_3.setObjectName(_fromUtf8("textBrowser_3"))
        self.textBrowser_4 = QtGui.QTextBrowser(self.frame)
        self.textBrowser_4.setGeometry(QtCore.QRect(22, 210, 101, 31))
        self.textBrowser_4.setObjectName(_fromUtf8("textBrowser_4"))
        self.textBrowser_5 = QtGui.QTextBrowser(self.frame)
        self.textBrowser_5.setGeometry(QtCore.QRect(22, 90, 101, 31))
        self.textBrowser_5.setObjectName(_fromUtf8("textBrowser_5"))
        self.mode_0 = QtGui.QSpinBox(self.frame)
        self.mode_0.setGeometry(QtCore.QRect(130, 90, 41, 27))
        self.mode_0.setMaximum(12)
        self.mode_0.setObjectName(_fromUtf8("mode_0"))
        self.mode_1 = QtGui.QSpinBox(self.frame)
        self.mode_1.setGeometry(QtCore.QRect(130, 130, 41, 27))
        self.mode_1.setMaximum(12)
        self.mode_1.setObjectName(_fromUtf8("mode_1"))
        self.mode_2 = QtGui.QSpinBox(self.frame)
        self.mode_2.setGeometry(QtCore.QRect(130, 170, 41, 27))
        self.mode_2.setMaximum(12)
        self.mode_2.setObjectName(_fromUtf8("mode_2"))
        self.mode_3 = QtGui.QSpinBox(self.frame)
        self.mode_3.setGeometry(QtCore.QRect(130, 210, 41, 27))
        self.mode_3.setMaximum(12)
        self.mode_3.setObjectName(_fromUtf8("mode_3"))
        self.textBrowser_6 = QtGui.QTextBrowser(self.frame)
        self.textBrowser_6.setGeometry(QtCore.QRect(18, 10, 101, 31))
        self.textBrowser_6.setObjectName(_fromUtf8("textBrowser_6"))
        self.start_pos = QtGui.QSpinBox(self.frame)
        self.start_pos.setGeometry(QtCore.QRect(128, 10, 41, 27))
        self.start_pos.setMaximum(2)
        self.start_pos.setObjectName(_fromUtf8("start_pos"))
        self.textBrowser_7 = QtGui.QTextBrowser(self.frame)
        self.textBrowser_7.setGeometry(QtCore.QRect(192, 170, 81, 31))
        self.textBrowser_7.setObjectName(_fromUtf8("textBrowser_7"))
        self.delay_1 = QtGui.QDoubleSpinBox(self.frame)
        self.delay_1.setGeometry(QtCore.QRect(280, 130, 51, 27))
        self.delay_1.setDecimals(1)
        self.delay_1.setMaximum(15.0)
        self.delay_1.setSingleStep(0.1)
        self.delay_1.setObjectName(_fromUtf8("delay_1"))
        self.textBrowser_8 = QtGui.QTextBrowser(self.frame)
        self.textBrowser_8.setGeometry(QtCore.QRect(192, 90, 81, 31))
        self.textBrowser_8.setObjectName(_fromUtf8("textBrowser_8"))
        self.delay_3 = QtGui.QDoubleSpinBox(self.frame)
        self.delay_3.setGeometry(QtCore.QRect(280, 210, 51, 27))
        self.delay_3.setDecimals(1)
        self.delay_3.setMaximum(15.0)
        self.delay_3.setSingleStep(0.1)
        self.delay_3.setObjectName(_fromUtf8("delay_3"))
        self.delay_2 = QtGui.QDoubleSpinBox(self.frame)
        self.delay_2.setGeometry(QtCore.QRect(280, 170, 51, 27))
        self.delay_2.setDecimals(1)
        self.delay_2.setMaximum(15.0)
        self.delay_2.setSingleStep(0.1)
        self.delay_2.setObjectName(_fromUtf8("delay_2"))
        self.textBrowser_9 = QtGui.QTextBrowser(self.frame)
        self.textBrowser_9.setGeometry(QtCore.QRect(192, 210, 81, 31))
        self.textBrowser_9.setObjectName(_fromUtf8("textBrowser_9"))
        self.textBrowser_10 = QtGui.QTextBrowser(self.frame)
        self.textBrowser_10.setGeometry(QtCore.QRect(192, 130, 81, 31))
        self.textBrowser_10.setObjectName(_fromUtf8("textBrowser_10"))
        self.delay_0 = QtGui.QDoubleSpinBox(self.frame)
        self.delay_0.setGeometry(QtCore.QRect(280, 90, 51, 27))
        self.delay_0.setDecimals(1)
        self.delay_0.setMaximum(15.0)
        self.delay_0.setSingleStep(0.1)
        self.delay_0.setObjectName(_fromUtf8("delay_0"))
        self.horizontalLayout.addWidget(self.frame)
        self.horizontalLayout_3.addLayout(self.horizontalLayout)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.practice_button.setText(_translate("MainWindow", "Practice", None))
        self.teleop_button.setText(_translate("MainWindow", "TeleOperated", None))
        self.auto_mode.setText(_translate("MainWindow", "Autonomous", None))
        self.enable_button.setText(_translate("MainWindow", "Enable", None))
        self.disable_button.setText(_translate("MainWindow", "Disable", None))
        self.textBrowser.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Alliance Data</p></body></html>", None))
        self.match_data.setText(_translate("MainWindow", "RRR", None))
        self.textBrowser_2.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Auto Mode 1</p></body></html>", None))
        self.textBrowser_3.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Auto Mode 2</p></body></html>", None))
        self.textBrowser_4.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Auto Mode 3</p></body></html>", None))
        self.textBrowser_5.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Auto Mode 0</p></body></html>", None))
        self.textBrowser_6.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Start Pos</p></body></html>", None))
        self.textBrowser_7.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Delay 2</p></body></html>", None))
        self.textBrowser_8.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Delay 0</p></body></html>", None))
        self.textBrowser_9.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Delay 3</p></body></html>", None))
        self.textBrowser_10.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Delay 1</p></body></html>", None))


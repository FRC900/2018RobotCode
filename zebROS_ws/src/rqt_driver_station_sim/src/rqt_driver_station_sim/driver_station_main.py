from PyQt4 import QtGui
import sys
import os
import rospy
import rospkg
import driver_station_widget
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from driver_station_sim import driver_station_sim_data

class DriverStation(QtGui.QMainWindow, driver_station_widget.Ui_MainWindow):
    def __init__(self, parent=None):
        super(DriverStation, self).__init__(parent)
        self.setupUi(self)
        self.enable_button.clicked.connect(self.unclick_enable)
        driver_station_pub = rospy.Publisher("/frcrobot/driver_station_sim", driver_station_sim_data, queue_size=10);   
    def unclick_enable(self):
        self.disable_button.setText("True")

def main():
    app = QtGui.QApplication(sys.argv)
    form = DriverStation()
    form.show()
    app.exec_()


if __name__ == '__main__':
    main()

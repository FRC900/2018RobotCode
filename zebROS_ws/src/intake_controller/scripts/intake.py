#!/usr/bin/env python

import rospy
import time
from threading import Thread
from ros_control_boilerplate.msg import JoystickState
from teleop_joystick_control.msg import RobotState

UPDATE_INTERVAL = 0.05 # 50 milliseconds
TIMEOUT = 1.05 # 5 seconds
intaking = False
cube = False

def updateController(Joystick):
     print('controller updated: ' + str(Joystick.buttonAPress))
     global intaking
     if Joystick.buttonAPress and not intaking:
         Thread(target = intake).start() # so that the controller and cube will continue updating
         intaking = True
     elif Joystick.buttonAPress and intaking: intaking = False

def updateCube(Cubevision):
    global cube
    cube = Cubevision.ifCube

def intake():
    start = time.clock()
    global cube
    global intaking
    intaking = True
    while (not cube):
        print('intaking...')
        if not intaking:
            print('intake canceled!')
            return
        if (time.clock() - start > TIMEOUT):
            print('intake failed')
            return
        time.sleep(UPDATE_INTERVAL)
    print('intake success!')

def intake_subscriber():
     print('plz work')
     rospy.init_node('intake_node', anonymous=True)
     rospy.Subscriber("ros_control_boilerplate", JoystickState, updateController)
     rospy.Subscriber("teleop_joystick_control", RobotState, updateCube)     
     rospy.spin()

if __name__ == '__main__':
    intake_subscriber()



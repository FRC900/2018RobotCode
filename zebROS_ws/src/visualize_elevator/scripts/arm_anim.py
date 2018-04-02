import rospy
import threading
from os.path import expanduser
import Queue
import os
from elevator_controller.msg import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

x = 0
y = 0
up_or_down = 0

fig = plt.gcf()
fig.show()
fig.canvas.draw()



def animate_arm(odom):
	x = odom.x
	y = odom.y
	up_or_down = odom.up_or_down
	plt.scatter(x, y)
	fig.canvas.draw()
def arm_anim_spin_loop():
	rospy.init_node('arm_anim')
	rospy.Subscriber("/frcrobot/elevator_controller/odom", ReturnElevatorCmd, animate_arm)

	rospy.spin()
		
			
	'''
	sign = -1
	if(odom.up_or_down)
	{
		sign = 1
	}
	arm_angle = math.acos(x / arm_length) * sign
	lift_height = math.sin(arm_angle) * arm_length
	'''


if __name__ == '__main__':
	arm_anim_spin_loop()


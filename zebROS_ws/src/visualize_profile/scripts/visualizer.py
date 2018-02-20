import rospy
from os.path import expanduser
import os
from talon_swerve_drive_controller.srv import *
#from trajectory_msgs.msg import *
import pylab as P
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

home = os.path.expanduser("~");
print(home)
filename = home + '/2018RobotCode/zebROS_ws/src/visualize_profile/field.jpg'
print(filename)
def create_plot(req):
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	xs = []
	ys = []
	zs = []

	for i in req.joint_trajectory.points:
		xs.append(i.positions[0])
		ys.append(i.positions[1])
		zs.append(math.sqrt(i.velocities[0]*i.velocities[0] + i.velocities[1]*i.velocities[1]))
	ax.plot(xs, ys, zs, lw=0.5)
	ax.set_xlabel("X Pos")
	ax.set_ylabel("Y Pos")
	ax.set_zlabel("Path Velocity")

	plt.show()

	im = plt.imread(filename)
	implot = plt.imshow(im)
	orient = plt.axes()
	k = 0
	scale = 53.404782735
	arrow_scale  =1
	init_pos_x = 5
	init_pos_y = .5
	#init_orient = -1.5 This will rotate the entire path?
	for i in req.joint_trajectory.points:
		if(k%5 == 0):
			orient.arrow(scale*(i.positions[0]+init_pos_x), scale*(i.positions[1]+init_pos_y), arrow_scale*scale*math.sin(i.positions[2])*.5, arrow_scale*scale*math.cos(i.positions[2])*.5, head_width=scale*arrow_scale*0.1, head_length=scale*arrow_scale*0.25, fc='k', ec='k');
		k+=1
	orient.axis([0, 490, 0, 960])
	plt.show()


	return MotionProfileResponse()

def create_plot_server():
	rospy.init_node('visualize')
	s = rospy.Service('visualize_profile', MotionProfile, create_plot)
	rospy.spin()

if __name__ == "__main__":
	create_plot_server()

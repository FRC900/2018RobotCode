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

#home = os.path.expanduser("~");
#print(home)
#filename = home + '/2018RobotCode/zebROS_ws/src/visualize_profile/field.jpg'
#print(filename)
def create_plot(req):
	print("called")
	#fig = plt.figure()
	#ax = fig.gca(projection='3d')
	flp = []
	frp = []
	blp = []
	brp = []


	flv = []
	frv = []
	blv = []
	brv = []


	fls = []
	frs = []
	bls = []
	brs = []


	index = []
	
	count  = 0

	for i in req.points:
		#print("running")
		flp.append(i.drive_pos[0])
		frp.append(i.drive_pos[1])
		blp.append(i.drive_pos[2])
		brp.append(i.drive_pos[3])
		flv.append(i.drive_vel[0])
		frv.append(i.drive_vel[1])
		blv.append(i.drive_vel[2])
		brv.append(i.drive_vel[3])
		fls.append(i.steer_pos[0])
		frs.append(i.steer_pos[1])
		bls.append(i.steer_pos[2])
		brs.append(i.steer_pos[3])
		index.append(count)
		count+=1
	plt.scatter(index, flp, s=0.2)
	plt.ylabel("Flp")
	plt.show()
	plt.scatter(index, frp, s=0.2)
	plt.ylabel("Frp")
	plt.show()
	plt.scatter(index, blp, s=0.2)
	plt.ylabel("Blp")
	plt.show()
	plt.scatter(index, brp, s=0.2)
	plt.ylabel("Brp")
	plt.show()

	plt.scatter(index, flv, s=0.2)
	plt.ylabel("Flv")
	plt.show()
	plt.scatter(index, frv, s=0.2)
	plt.ylabel("Frv")
	plt.show()
	plt.scatter(index, blv, s=0.2)
	plt.ylabel("Blv")
	plt.show()
	plt.scatter(index, brv, s=0.2)
	plt.ylabel("Brv")
	plt.show()


	plt.scatter(index, fls, s=0.2)
	plt.ylabel("Fls")
	plt.show()
	plt.scatter(index, frs, s=0.2)
	plt.ylabel("Frs")
	plt.show()
	plt.scatter(index, bls, s=0.2)
	plt.ylabel("Bls")
	plt.show()
	plt.scatter(index, brs, s=0.2)
	plt.ylabel("Brs")
	plt.show()

	return MotionProfilePointsResponse()

def create_plot_server():
	rospy.init_node('visualize_swerve')
	s = rospy.Service('visualize_swerve_profile', MotionProfilePoints, create_plot)
	rospy.spin()

if __name__ == "__main__":
	create_plot_server()

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


	flf = []
	frf = []
	blf = []
	brf = []


	fls = []
	frs = []
	bls = []
	brs = []

	flsf = []
	frsf = []
	blsf = []
	brsf = []

	index = []
	
	count  = 0

	for i in req.points:
		#print("running")
		flp.append(i.drive_pos[0])
		frp.append(i.drive_pos[1])
		blp.append(i.drive_pos[2])
		brp.append(i.drive_pos[3])
		flf.append(i.drive_f[0])
		frf.append(i.drive_f[1])
		blf.append(i.drive_f[2])
		brf.append(i.drive_f[3])
		fls.append(i.steer_pos[0])
		frs.append(i.steer_pos[1])
		bls.append(i.steer_pos[2])
		brs.append(i.steer_pos[3])
		flsf.append(i.steer_f[0])
		frsf.append(i.steer_f[1])
		blsf.append(i.steer_f[2])
		brsf.append(i.steer_f[3])
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

	plt.scatter(index, flf, s=0.2)
	plt.ylabel("Flf")
	plt.show()
	plt.scatter(index, frf, s=0.2)
	plt.ylabel("Frf")
	plt.show()
	plt.scatter(index, blf, s=0.2)
	plt.ylabel("Blf")
	plt.show()
	plt.scatter(index, brf, s=0.2)
	plt.ylabel("Brf")
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


	plt.scatter(index, flsf, s=0.2)
	plt.ylabel("Flsf")
	plt.show()
	plt.scatter(index, frsf, s=0.2)
	plt.ylabel("Frsf")
	plt.show()
	plt.scatter(index, blsf, s=0.2)
	plt.ylabel("Blsf")
	plt.show()
	plt.scatter(index, brsf, s=0.2)
	plt.ylabel("Brsf")
	plt.show()


	return MotionProfilePointsResponse()

def create_plot_server():
	rospy.init_node('visualize_swerve')
	s = rospy.Service('visualize_swerve_profile', MotionProfilePoints, create_plot)
	rospy.spin()

if __name__ == "__main__":
	create_plot_server()

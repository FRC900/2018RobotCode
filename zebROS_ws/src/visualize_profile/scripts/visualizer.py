import rospy
from talon_swerve_drive_controller.srv import *
#from trajectory_msgs.msg import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

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

	return MotionProfileResponse()

def create_plot_server():
	rospy.init_node('visualize')
	s = rospy.Service('visualize_profile', MotionProfile, create_plot)
	rospy.spin()

if __name__ == "__main__":
	create_plot_server()

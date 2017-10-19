#! /usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from px_comm.msg import OpticalFlow

pub = rospy.Publisher("px4_odom", Odometry, queue_size=1)

def listener ():
	rospy.Subscriber("/px4flow/opt_flow", OpticalFlow, callback)

def callback (data):
	prev_time = cur_time
	cur_time = rospy.get_rostime()
	dt = cur_time - prev_time
	odom = Odometry()
	odom.header.stamp = data.header.stamp
	odom.header.frame_id = "odom"
	odom.child_frame_id = "base_link"
	odom.pose.pose.orientation.w = 1
	odom.twist.twist.linear.x = data.velocity_x
	odom.twist.twist.linear.y = data.velocity_y
	odom.pose.pose.position.x = data.velocity_x / dt
	odom.pose.pose.position.x = data.velocity_y / dt
	pub.publish(odom)


if __name__ == "__main__":
	rospy.init_node("to_odom")
	prev_time = rospy.get_rostime()
	listener()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()
		rospy.spin()

#! /usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from px_comm.msg import OpticalFlow

x_pos, y_pos = 0, 0
pub = rospy.Publisher("px4_odom", Odometry, queue_size=1)

def listener():
	rospy.Subscriber("/px4flow/opt_flow", OpticalFlow, callback)

def callback(data):
	global cur_time, prev_time, x_pos, y_pos
        # get current time
	cur_time = rospy.get_rostime().to_sec()

        # get current velocities
	odom = Odometry()
	odom.header.stamp = data.header.stamp
	odom.header.frame_id = "odom"
	odom.child_frame_id = "base_link"
	odom.pose.pose.orientation.w = 1
	odom.twist.twist.linear.x = data.velocity_x
	odom.twist.twist.linear.y = data.velocity_y

        # compute dx and dy
        dx = data.velocity_x * (cur_time - prev_time)
        dy = data.velocity_y * (cur_time - prev_time)

        # add dx and dy to current position
	x_pos += dx
        odom.pose.pose.position.x += x_pos
        y_pos += dy
	odom.pose.pose.position.y += y_pos
	
        # update previous times
        prev_time = cur_time

        pub.publish(odom)


if __name__ == "__main__":
	global prev_time
	rospy.init_node("to_odom")
	listener()        
	rate = rospy.Rate(120)

        prev_time = rospy.get_rostime().to_sec()

	while not rospy.is_shutdown():
		rate.sleep()
                rospy.spin()

#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
import numpy as np
import sys
import math


def qv_mult(q, v):
    v_unit = None
    if v == [0, 0, 0]: v_unit = [0.0, 0.0, 0.0]
    else: v_unit = tf.transformations.unit_vector(v)
    qp = list(v_unit)
    qp.append(0.0)
    
    return tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q, qp), tf.transformations.quaternion_conjugate(q))

class TransformOdometry:
    def __init__(self, args):
        self.x_pos_shift = float(args[1])
        self.y_pos_shift = float(args[2])
        self.z_pos_shift = float(args[3])
        
        self.x_ori_shift = float(args[4])
        self.y_ori_shift = float(args[5])
        self.z_ori_shift = float(args[6])
        self.w_ori_shift = float(args[7])

        self.odom_sub_topic = args[8]
        self.odom_pub_topic = args[9]

        self.sub_odom_transform = rospy.Subscriber(self.odom_sub_topic, Odometry, self._transform_callback)
        self.pub_odom_transform = rospy.Publisher(self.odom_pub_topic, Odometry, queue_size=1)

    def get_node_name(self):
        return self.odom_node_name

    def _transform_callback(self, odom_msg):
        #odom_msg.pose.pose.position.x = odom_msg.pose.pose.position.x * math.cos(self.z_ori_shift) - odom_msg.pose.pose.position.y * math.sin(self.z_ori_shift)
        #odom_msg.pose.pose.position.y = odom_msg.pose.pose.position.x * math.sin(self.z_ori_shift) + odom_msg.pose.pose.position.y * math.cos(self.z_ori_shift)
	
        #odom_msg.pose.pose.position.x = odom_msg.pose.pose.position.x * math.cos(self.y_ori_shift) - odom_msg.pose.pose.position.z * math.sin(self.y_ori_shift)
        #odom_msg.pose.pose.position.y = odom_msg.pose.pose.position.x * math.sin(self.y_ori_shift) + odom_msg.pose.pose.position.z * math.cos(self.y_ori_shift)

        #odom_msg.pose.pose.position.z = 0
        
        #q = [odom_msg.pose.pose.orientation.x,
        #     odom_msg.pose.pose.orientation.y,
        #     odom_msg.pose.pose.orientation.z,
        #     odom_msg.pose.pose.orientation.w]

        #new_vec = qv_mult(q, [self.x_pos_shift, self.y_pos_shift, self.z_pos_shift])
        #tx = self.x_pos_shift - new_vec[0]
        #ty = self.y_pos_shift - new_vec[1]
        #tz = self.z_pos_shift - new_vec[2]

        #odom_pub_msg = Odometry()
        #odom_pub_msg.header.seq = odom_msg.header.seq 
        #odom_pub_msg.header.stamp = odom_msg.header.stamp
        #odom_pub_msg.header.frame_id = odom_msg.header.frame_id
        #odom_pub_msg.child_frame_id = ""
        #odom_pub_msg.pose.pose.position.x = odom_msg.pose.pose.position.x + tx - self.x_pos_shift
        #odom_pub_msg.pose.pose.position.y = odom_msg.pose.pose.position.y + ty - self.y_pos_shift
        #odom_pub_msg.pose.pose.position.z = odom_msg.pose.pose.position.z + tz - self.z_pos_shift
        #odom_pub_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
        #odom_pub_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
        #odom_pub_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        #odom_pub_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w
        #odom_pub_msg.pose.covariance = odom_msg.pose.covariance

        self.pub_odom_transform.publish(odom_msg)

if __name__ == "__main__":
    rospy.init_node("odom_transform")
    # Parse input args
    def parse_args(args):
        if len(args) != 10: return False
        for arg in args[1:8]:
            try: float(arg)
            except: return False
        return True
    args = rospy.myargv(argv=sys.argv)
    if not parse_args(args): 
        rospy.logfatal("Invalid commandline arguments specified")
        rospy.signal_shutdown("Invalid commandline arguments specified")
    to = TransformOdometry(args)

    rospy.spin()

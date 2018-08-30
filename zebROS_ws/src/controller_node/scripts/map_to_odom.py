#!/usr/bin/env python

# SciStacvk 
import numpy as np
import matplotlib.pyplot as plt
import cv2

# ROS imports
import rospy
import tf
import tf2_ros
import message_filters

# Transform Odometry
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import TransformStamped
from goal_detection.msg import GoalDetection

# Python Utilities
import math
import time
import sys
from threading import Thread
from time import sleep

def qv_mult(q, v):
    v_unit = None
    if v == [0, 0, 0]: v_unit = [0.0, 0.0, 0.0]
    else: v_unit = tf.transformations.unit_vector(v)
    qp = list(v_unit)
    qp.append(0.0)
    
    return tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q, qp), tf.transformations.quaternion_conjugate(q))

class MapToOdom:
    def __init__(self, args):
        self.is_red = False

        self.x_pos_shift = 0.0
        self.y_pos_shift = 0.0
        self.z_pos_shift = 0.0
        
        self.x_ori_shift = 0.0
        self.y_ori_shift = 0.0
        self.z_ori_shift = 0.0
        self.w_ori_shift = 1.0
        
        self.sub_navx = message_filters.Subscriber("/heading", QuaternionStamped, queue_size=1)
        self.sub_goal = message_filters.Subscriber("/goal_detect_msg", GoalDetection, queue_size=1)
        self.ts = message_filters.TimeSynchronizer([self.sub_navx, self.sub_goal], 10)
        self.ts.registerCallback(self._reset_odom_transform)

        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        
    
    def _reset_odom_transform(self, quat, goal_msg):
        
        
        
        
        
        

        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = "map"
        self.t.child_frame_id = "odom"
        self.t.transform.translation.x = self.x_pos_shift
        self.t.transform.translation.y = self.y_pos_shift
        self.t.transform.translation.z = self.z_pos_shift
        self.t.transform.rotation.x    = self.x_ori_shift
        self.t.transform.rotation.y    = self.y_ori_shift
        self.t.transform.rotation.z    = self.z_ori_shift
        self.t.transform.rotation.w    = self.w_ori_shift

        self.br.sendTransform(self.t)

    def _transform_callback(self, odom_msg):
        self.pub_odom_transform.publish(odom_msg)

if __name__ == "__main__":
    rospy.init_node("odom_transform")
    # Parse input args
    
    mpo = MapToOdom()

    rospy.spin()


# LIDAR LIDAR LIDAR LIDAR LIDAR LIDAR LIDAR
# 6) Look up WPILIB and click on the first link. From there, find the Control System Hardware Overview. What company manufactures the SD540B and SD540C Motor Controllers? Find the name of that company somewhere in our code... 


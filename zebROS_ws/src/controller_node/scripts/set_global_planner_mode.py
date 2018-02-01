#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):
    return

if __name__ == "__main__":
    rospy.init_node("global_planner_set_mode")

    rospy.wait_for_service("/move_base/GlobalPlanner/set_parameters")
    client = dynamic_reconfigure.client.Client("/move_base/GlobalPlanner", timeout=30, config_callback=callback)

    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        client.update_configuration({"orientation_mode":2})
        r.sleep()


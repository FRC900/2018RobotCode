#! /usr/bin/python

import csv
import rospy
import sys
from talon_state_controller.msg import *
from math import pi


import os



def fake_talon_pub():	
	pub = rospy.Publisher('/frcrobot/talon_states', TalonState, queue_size=10000)
	rospy.init_node('fake_talons', anonymous=True)
	rate = rospy.Rate(10000)
	
	#cwd = os.getcwd()

	file_name =  "../Documents/hero_thermal_testing/data/M2/5v_setting_1/out_1.csv" #str(sys.argv[1])
	time_start = 0
	time_end = 10000000000000000000000


	sleeper = rospy.Rate(0.5)
	
	sleeper.sleep()	


	#print(file_name)
	#print(sys.argv[2])
	#print(sys.argv[3])
	#if(len(sys.argv) > 2):
	#	print("s")
	#	time_start  = float(sys.argv[2])
	#	time_end  = float(sys.argv[3])
	with open(file_name, 'rt') as csvfile:
		reader = csv.DictReader(csvfile, delimiter=',')
		
		for row in reader:
			#print("ran row")
			if(row['time_talon'] < time_start):
				#print("continuing")
				continue
			if(float(row['time_talon']) > time_end or rospy.is_shutdown()):
				#print("returning shutdown: ", rospy.is_shutdown())
				return
			msg = TalonState()
			msg.header.stamp	= rospy.Time.from_sec(float(row['time_talon']))
			msg.name			= ["test_talon"]
			msg.speed			= [float(row['RPM']) / 60.0 * 2 * pi]
			msg.bus_voltage		= [float(row['bus_voltage'])]
			msg.output_voltage	= [float(row['voltage'])]
			msg.output_current	= [float(row['current'])]
			
			#print("here")

			rate.sleep()

			pub.publish(msg)

			#Also do something with temps to compare model	
		
if __name__ == "__main__":
	print("called")
	try:
		fake_talon_pub()
	except rospy.ROSInterruptException:
		pass
			
	

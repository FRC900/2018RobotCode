#! /usr/bin/python

from __future__ import print_function
import scipy.optimize as optimize
import csv
import rospy
import sys
from thermal_modeling.srv import *
from thermal_modeling.msg import *
from math import pi
import numpy as np
import matplotlib.pyplot as plt

import os
	

#data = np.array([time, speed, bus_voltage, output_voltage, output_current])

def func(data, e_term1, h_1, e_term2, h_2, v_1, v_2, v_squared_1, v_squared_2):
	

	#v_1, v_2, v_squared_1, v_squared_2 = 0, 0, 0, 0
	#print(type(req))
	#print(callable(req))
	#print(locals(req))

	out_v = []
	

	global start_time_var
	global static_start_time
	global count_var
	
	value_b = True	

	saved_time = time.time()

	for time_h, RPM_h, bus_voltage_h, output_voltage_h, output_current_h, i_temp, true_temp in zip(data[0], data[1],data[2],data[3],data[4],avg_temp, temp):
		p = ModelParams("test_talon", h_1, h_2, e_term1, e_term2, v_1, v_2, v_squared_1, v_squared_2, [i_temp, i_temp])
		#req = ModelTest(p, data[0], data[1], data[2], data[3], data[4])

		out = srv(p, time_h, RPM_h, bus_voltage_h, output_voltage_h, output_current_h)
		
		if(value_b):
			value_b = False
			out_v = out.temps[1].temps
		else:
			out_v = np.hstack((out_v, out.temps[1].temps))

		#print(out.temps[1].temps)
		#if(plot_by_time):
		#	print("founf it", str(start_time_var))	

	
		#print(str(count_var))




	
		if(force_plot_now):
			plt.ylim(290, 340)
			plt.plot(time_h, out.temps[1].temps)
			plt.plot(time_h, true_temp)
			plt.show()
		elif( (plot_by_time and saved_time  - start_time_var > 50) ):
			plt.ylim(290, 340)
			plt.plot(time_h, out.temps[1].temps)
			plt.plot(time_h, true_temp)
			plt.show(block=False)
			time.sleep(3.0)
			#plt.pause(3.0)
		#plt.pause(3.0)
		
	if(saved_time - start_time_var > 50):
		start_time_var = time.time()

	count_var+=1

	

	print("time: ", str( time.time() - static_start_time), " count: ", str(count_var), " compute time: ", str(time.time() - saved_time), end='\r')

	sys.stdout.flush()

	#print(out_v)
	return out_v

	
if __name__ == "__main__":
	
	print(os.getcwd())
	
	file_names =  [ "../Documents/hero_thermal_testing/data/M2/1.25v_stall/out_4.csv","../Documents/hero_thermal_testing/data/M2/1.25v_stall/out_1.csv", "../Documents/hero_thermal_testing/data/M2/out_ramp.csv", "../Documents/hero_thermal_testing/data/M2/out_ramp2.csv", "../Documents/hero_thermal_testing/data/M2/out_ramp3.csv", "../Documents/hero_thermal_testing/data/M2/10v_setting_1/out_1.csv", "../Documents/hero_thermal_testing/data/M2/10v_setting_1/out_2.csv", "../Documents/hero_thermal_testing/data/M2/11v_setting_1/out_1.csv", "../Documents/hero_thermal_testing/data/M2/11v_setting_1/out_3.csv", "../Documents/hero_thermal_testing/data/M2/11v_setting_1/out_5.csv","../Documents/hero_thermal_testing/data/M2/2v_setting_3/out_3.csv", "../Documents/hero_thermal_testing/data/M2/2v_setting_3/out_4.csv", "../Documents/hero_thermal_testing/data/M2/2v_setting_3/out_5.csv"] 
	start_times = [0, 0 , 1000, 700, 700, 0, 0, 0, 0, 0, 0, 0, 0]
	end_times = [1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000]

	time = []
	speed = []
	bus_voltage = []
	output_voltage = []
	output_current = []
	temp = []
	avg_temp = []

	print("calibration: ")	
	for file_name, start_time, end_time in zip(file_names, start_times, end_times):
		print("file: ", file_name)
		t_time = []
		t_speed = []
		t_bus_voltage = []
		t_output_voltage = []
		t_output_current = []
		t_temp = []
		t_avg_temp = []
		max_speed = 0;
		with open(file_name, 'rt') as csvfile:
			reader = csv.DictReader(csvfile, delimiter=',')
			
			for row in reader:
				#print("ran row")
				if(float(row['time_talon']) < start_time):
					#print("continuing")
					continue
				if(float(row['time_talon']) > end_time ):
					#print("returning shutdown: ", rospy.is_shutdown())
					break
				t_time.append(float(row['time_talon']))
				if(abs(float(row['RPM']) / 60.0 * 2 * pi) > max_speed):
					max_speed = abs(float(row['RPM']) / 60.0 * 2 * pi)

				
				t_speed.append(max_speed)#abs(float(row['RPM']) / 60.0 * 2 * pi))
				t_bus_voltage.append(abs(float(row['bus_voltage'])))
				t_output_voltage.append(abs(float(row['voltage'])))
				t_output_current.append(abs(float(row['current'])))
				t_temp.append((float(row['temp1']) + float(row['temp2']) + float(row['temp3'])) / 3.0)

				if(float(row['time_talon']) - start_time < 20.0):
					t_avg_temp.append((float(row['temp1']) + float(row['temp2']) + float(row['temp3'])) / 3.0)
					
		avg_temp.append(np.average(np.array(t_avg_temp)))
	
	
		t_time = np.array(t_time)
		t_temp = np.array(t_temp)
		t_speed = np.array(t_speed)
		t_bus_voltage = np.array(t_bus_voltage)
		t_output_voltage = np.array(t_output_voltage)
		t_output_current = np.array(t_output_current)
		
		time.append(t_time)
		temp.append(t_temp)
		speed.append(t_speed)
		bus_voltage.append(t_bus_voltage)
		output_voltage.append(t_output_voltage)
		output_current.append(t_output_current)

	time = np.array(time)
	avg_temp = np.array(avg_temp)
	temp = np.array(temp)
	speed = np.array(speed)
	bus_voltage = np.array(bus_voltage)
	output_voltage = np.array(output_voltage)
	output_current = np.array(output_current)

	data = np.array([time, speed, bus_voltage, output_voltage, output_current])



	file_names_v =  [ "../Documents/hero_thermal_testing/data/M2/custom_2/out_6.csv"] 
	start_times_v = [0]#, 0 , 1000, 700, 700, 0, 0, 0, 0, 0, 0, 0, 0]
	end_times_v = [1000000000000000000000000000000]#, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000]

	time_v = []
	speed_v = []
	bus_voltage_v = []
	output_voltage_v = []
	output_current_v = []
	temp_v = []
	avg_temp_v = []

	
	print("validation: ")	
	for file_name, start_time, end_time in zip(file_names_v, start_times_v, end_times_v):
		print("file: ", file_name)
		t_time = []
		t_speed = []
		t_bus_voltage = []
		t_output_voltage = []
		t_output_current = []
		t_temp = []
		t_avg_temp = []
		with open(file_name, 'rt') as csvfile:
			reader = csv.DictReader(csvfile, delimiter=',')
			
			for row in reader:
				#print("ran row")
				if(float(row['time_talon']) < start_time):
					#print("continuing")
					continue
				if(float(row['time_talon']) > end_time ):
					#print("returning shutdown: ", rospy.is_shutdown())
					break
				t_time.append(float(row['time_talon']))
				


				
				t_speed.append(abs(float(row['RPM']) / 60.0 * 2 * pi))
				t_bus_voltage.append(abs(float(row['bus_voltage'])))
				t_output_voltage.append(abs(float(row['voltage'])))
				t_output_current.append(abs(float(row['current'])))
				t_temp.append((float(row['temp1']) + float(row['temp2']) + float(row['temp3'])) / 3.0)

				if(float(row['time_talon']) - start_time < 20.0):
					t_avg_temp.append((float(row['temp1']) + float(row['temp2']) + float(row['temp3'])) / 3.0)
					
		avg_temp_v.append(np.average(np.array(t_avg_temp)))
	
	
		t_time = np.array(t_time)
		t_temp = np.array(t_temp)
		t_speed = np.array(t_speed)
		t_bus_voltage = np.array(t_bus_voltage)
		t_output_voltage = np.array(t_output_voltage)
		t_output_current = np.array(t_output_current)
		
		time_v.append(t_time)
		temp_v.append(t_temp)
		speed_v.append(t_speed)
		bus_voltage_v.append(t_bus_voltage)
		output_voltage_v.append(t_output_voltage)
		output_current_v.append(t_output_current)

	time_v = np.array(time_v)
	avg_temp_v = np.array(avg_temp_v)
	temp_v = np.array(temp_v)
	speed_v = np.array(speed_v)
	bus_voltage_v = np.array(bus_voltage_v)
	output_voltage_v = np.array(output_voltage_v)
	output_current_v = np.array(output_current_v)

	data_v = np.array([time_v, speed_v, bus_voltage_v, output_voltage_v, output_current_v])







	'''
	
	print("got to plotting")
	for time_s, temp_s, rpm_s, name in zip(time, temp, speed, file_names): 
		print("about to plot")
		plt.plot(time_s, rpm_s)
		plt.title(name)
		print("about to show")
		print(name)
		plt.show()
	

	print("post_plot")	

	for time_s, temp_s, rpm_s, name in zip(time_v, temp_v, speed_v,  file_names_v): 
		print("about to plot")
		plt.plot(time_s, rpm_s)
		plt.title(name)
		print("about to show")
		plt.show()

	'''


	rospy.wait_for_service('/frcrobot/thermal_test')


	srv = rospy.ServiceProxy('/frcrobot/thermal_test', ModelTest)

	#def func(data, a, c):
#		return data[0] * data[0] *  a + data[1] * c

	guess = ( -6.60520104e-12,  6.04134994e-02,  1.86512289e-08, -1.86403508e+00, -1.94841489e-04,   1.69611900e-03,   1.83011252e-07,  -4.43135529e-07) 

	force_plot_now = True
	plot_by_time = False
	
	import time

	start_time_var = time.time()
	static_start_time = start_time_var

	count_var = 0
	
	print("")

	
	force_plot_now = False
	#plot_now = True


	t_o = temp[0]

	val = False

	for t in temp:
		if (not val):
			val = True
			continue
			
		t_o = np.hstack((t_o, t))
	

	start_time_var = time.time()

	static_start_time = start_time_var

	plot_by_time = False
	
	count_var = 0
	
	print("")
	#para, pcov = optimize.curve_fit(func, data, t_o, guess)
	print("")
	para = guess
	#print(para)
	#print(pcov)


	force_plot_now = True

	start_time_var = time.time()
	static_start_time = start_time_var
	
	count_var = 0
	print("")
	
	#func(data, para[0], para[1],para[2],para[3], para[4], para[5],para[6],para[7])
	
	print("")
	print("validation")
	start_time_var = time.time()
	static_start_time = start_time_var
	
	count_var = 0
	
	avg_temp = avg_temp_v
	temp = temp_v
	
	print("")
	func(data_v, para[0], para[1],para[2],para[3], para[4], para[5],para[6],para[7])
	print("")




			

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
import gc


from multiprocessing.pool import ThreadPool

import os
	

params_last = (1.0, 1.0)

j = 0
b = 0
h = 0
k = 0

def func1(data, a, c):
	global j 
	global b
	global h
	global k
	curr_term = np.where(data[2] > 0.1, data[0] * data[0] *  data[3] / data[2] ** 2, data[0] * data[0] *  data[3] / 0.1 ** 2)
	#print("k: ", k, " h: ", h,  " b: ", b, " a: ", a, " c: ")
	#print(h)
	#print(b)
	#print(a)
	return ((curr_term * a + data[1] * c + data[1] * data[1] * b +  data[0] **abs(h) * data[2] **abs(k) * j) / wl_f - 1)* data[5] +1

#def func(data, e_term1, h_1, e_term2, h_2, e_term3, h_3,v_1, v_2, v_3, v_squared_1, v_squared_2, v_squared_3,custom_v_c, v_squared_term, resistance):
def func(data, e_term1, h_1, e_term2, h_2, e_term3, h_3, v_1, v_2, v_3,v_squared_1, v_squared_2, v_squared_3, custom_v_c, v_squared_term, custom_c_pow, custom_v_pow):

	gc.collect()

	

	global f
	voltage_exponent = 2.0

	f = voltage_exponent

	global j

	j = custom_v_c

	global b

	b = v_squared_term
	
	global h

	h = custom_c_pow
	
	
	global k

	k = custom_v_pow
	#global a

	#a = resistance

	global params_last	

	params, pcov = optimize.curve_fit(func1, independent3, ones_1, params_last,maxfev = 10000 )


#	params_last = params
	waste = np.array([params[0], 0.0, v_squared_term, params[1], 0.0, 0.0, voltage_exponent, 0.0, 0.0, custom_v_c, abs(custom_c_pow), abs(custom_v_pow)])

	#waste = np.array([1.16420618e+00, 0.0, -6.38198699e-05, 1.29578476e-01, 0.0, 0.0, voltage_exponent, 0.0, 0.0, -9.69149234e-02, 1.64252560e+00, 3.71621311e-07])

	loss_resistance_0v, loss_resistance_12v, loss_v_squared_term, loss_v_term, loss_volt_term, loss_volt_squared_term, voltage_exponent,  volt_squared_current, current_squared_volt, custom_v_c, custom_c_pow, custom_v_pow  = waste[0], waste[1], waste[2], waste[3], waste[4], waste[5], waste[6], waste[7], waste[8], waste[9], waste[10], waste[11]

	#0.10480932  0.00315612

	#v_1, v_2, v_squared_1, v_squared_2 = 0, 0, 0, 0
	#print(type(req))
	#print(callable(req))
	#print(locals(req))

	out_v = []
	

	global start_time_var
	global static_start_time
	global count_var
	
	value_b = True	

	saved_time = time_module.time()

	index = 0


	pool = ThreadPool(processes=data.shape[0])

	results = []

	for time_h, RPM_h, bus_voltage_h, output_voltage_h, output_current_h, i_temp in zip(data[0], data[1],data[2],data[3],data[4],avg_temp):
		p = ModelParams("test_talon", h_1, h_2, h_3, e_term1, e_term2, e_term3, v_1, v_2, v_3, v_squared_1, v_squared_2, v_squared_3, loss_v_term,  loss_v_squared_term, loss_volt_term,  loss_volt_squared_term, loss_resistance_12v, loss_resistance_0v, voltage_exponent,  volt_squared_current, current_squared_volt, custom_v_c, custom_c_pow, custom_v_pow, [i_temp, i_temp])		#req = ModelTest(p, data[0], data[1], data[2], data[3], data[4])

		results.append(pool.apply_async(srv[index], (p, time_h, RPM_h, bus_voltage_h, output_voltage_h, output_current_h)))
		
		
		index += 1



		#print(out.temps[1].temps)
		#if(plot_by_time):
		#	print("founf it", str(start_time_var))	

	
		#print(str(count_var))


	for time_h, true_temp, result_yu, file_name, weight in zip(data[0], temp, results, file_names, weights):
		out = result_yu.get()
		if(value_b):
			value_b = False
			out_v = (out.temps[1].temps / true_temp - 1) * weight + 1
		else:
			out_v = np.hstack((out_v, ((out.temps[1].temps / true_temp - 1) * weight + 1) * 10000.0))

	
		if(force_plot_now):
			plt.title(file_name)
			plt.ylim(290, 340)
			plt.plot(time_h, out.temps[1].temps)
			plt.plot(time_h, out.temps[0].temps)
			plt.plot(time_h, true_temp)
			plt.show()
		elif( (plot_by_time and count_var % 500 == 499) ):
			plt.title(file_name)
			plt.ylim(290, 340)
			plt.plot(time_h, out.temps[1].temps)
			plt.plot(time_h, out.temps[0].temps)
			plt.plot(time_h, true_temp)
			plt.show(block=False)
			time_module.sleep(3.0)
			plt.close()
			#plt.pause(3.0)
		#plt.pause(3.0)

		del result_yu

	del results
		
	if(saved_time - start_time_var > 50):
		start_time_var = time_module.time()

	global mse_c

	global ones

	#out_v = np.where(out_v < 10 * 10000.0, out_v, 10 * 10000.0)
	#out_v = np.where(out_v > -10 * 10000.0, out_v, -10 * 10000.0)
	#out_v = np.where(np.isnan(out_v), out_v, 10 * 10000.0)
	
	#mse_v = 0
	if(count_var % 50 == 0):
		mse_c = ((out_v - ones)**2).mean(axis=None)

	count_var+=1


	pool.close()
	pool.join()
	del pool

	#" params: ", params, 


	print("\033[Ktime: ", str( time_module.time() - static_start_time), " count: ", str(count_var), " compute time: ", str(time_module.time() - saved_time), " mse: ", mse_c, " vals: ",e_term1, h_1, e_term2, h_2, v_1, v_2, v_squared_1, v_squared_2, custom_v_c, v_squared_term,  " waste: ", waste[0], waste[1] , waste[2], waste[3], waste[4], waste[5], waste[6], waste[7], waste[8], waste[9], waste[10], waste[11], end='\r')

	#, v_waste_coeff, v_squared_waste_coeff

	sys.stdout.flush()

	

	gc.collect()	
	#print(out_v)
	return out_v

	
if __name__ == "__main__":


	x_a = []
	e_a = []
	w_a = []
	y_a = []
	z_a = []
	k_a = []
	f_a = []
	b_a = []


	weight_local_no_stall = 0.2
	weight_local_stall = 10.0
	weight_curve_data = 1.0

	base = "../Documents/hero_thermal_testing/data/"
	with open(base + '/M2/waste_data_new.csv', 'rt') as csvfile:
		reader = csv.DictReader(csvfile, delimiter=',')
		for row in reader:
			#print(row)


			if(float(row["RPM"]) > 3.0):
				w_a.append(weight_local_no_stall)
			else:
				w_a.append(weight_local_stall)


			e_a.append(1.0)
			#print(row["file_address"])
			x_a.append(float(row["current"]))
			y_a.append(float(row["RPM"]) / 60 * 2 * pi)
			f_a.append(float(row["force"]))
			z_a.append(float(row["waste_watts"]))
			k_a.append(float(row["voltage"]))
			b_a.append((float(row["waste_watts"]) + float(row["RPM"]) *  float(row["force"]) * 2 * pi / 60.0) / float(row["current"]))



	e_p = []
	w_p = []
	s_p = []
	c_p = []
	v_p = []
	b_p = []
	wl_p =[]



	with open('../775pro-motor-curve-data.csv', 'rt') as csvfile:
		reader = csv.DictReader(csvfile, delimiter=',')
		for row in reader:
			w_p.append(weight_curve_data)
			e_p.append(0.0)
			s_p.append(float(row['Speed (RPM)']) / 60 * 2 * pi)
			c_p.append(float(row['Current (A)']))
			wl_p.append(float(row['Power Dissipation (W)']))
			v_p.append(12)
			b_p.append(12)


	w_p = np.array(w_p)
	w_a = np.array(w_a)
	e_p = np.array(e_p)
	e_a = np.array(e_a)
	v_p = np.array(v_p)
	b_p = np.array(b_p)
	x_a = np.array(x_a)
	k_a = np.array(k_a)
	f_a = np.array(f_a)
	y_a = np.array(y_a)
	z_a = np.array(z_a)
	s_p = np.array(s_p)
	c_p = np.array(c_p)
	wl_p =np.array(wl_p)


	eff = 0.95


	wl_f = np.hstack((z_a * eff, wl_p))
	c_f = np.hstack((x_a, c_p))
	s_f = np.hstack((y_a, s_p))
	v_f = np.hstack((k_a,v_p))
	b_f = np.hstack((b_a,b_p))
	e_f = np.hstack((e_a,e_p))
	w_f = np.hstack((w_a,w_p))


	ones_1 = np.full(wl_f.shape, 1.0)

	independent3 = np.array([c_f, s_f, v_f, b_f, e_f, w_f]) 


	
	print(os.getcwd())



	
	file_names =  ["M1/5v_setting_1/out_1.csv", \
				   "M2/out_ramp.csv", \
				   "M2/out_ramp2.csv", \
				   "M2/out_ramp3.csv", \
				   "M1/5v_setting_1/out_2.csv", \
				   "M1/5v_setting_1/out_3.csv", \
				   "M2/1.25v_stall/out_1.csv", \
				   "M2/1.25v_stall/out_2.csv", \
				   "M2/1.25v_stall/out_4.csv", \
				   "M2/10v_setting_1/out_2.csv", \
				   "M2/10v_setting_1/out_7v.csv", \
				   "M2/11v_setting_1/out_1.csv", \
				   "M2/11v_setting_1/out_3.csv", \
				   "M2/11v_setting_1/out_2_non_const_v.csv", \
				   "M2/11v_setting_1/out_4_non_const_v.csv", \
				   "M2/2v_setting_3/out_2v_1.csv", \
				   "M2/2v_setting_3/out_3.csv", \
				   "M2/2v_setting_3/out_4.csv", \
				   "M2/2v_setting_3/out_5.csv", \
				   "M2/5v_setting_1/out_3.csv", \
				   "M2/out_2v_stall_1_min.csv", \
				   "M2/0-3-3-0_stall/out_1.csv", \
				   "M2/0-3-3-0_stall/out_2.csv", \
				   "M2/0-3-3-0_stall/out_3.csv", \
				   "M2/0-4-0_setting_3/out_1.csv", \
				   "M2/0-4-0_setting_3/out_2.csv", \
				   "M2/0-4-0_setting_3/out_3.csv", \
				   "M2/0-4-0_setting_3/out_4.csv", \
				   "M2/0-4-0_setting_3/out_5.csv", \
				   "M2/0-4-0_setting_3/out_6.csv", \
				   "M2/0-4-0_setting_3/out_7.csv", \
				   "M2/0-4-0_setting_3/out_8.csv", \
				   "M2/0-5-0_setting_2/out_1.csv", \
				   "M2/0-5-0_setting_2/out_2.csv", \
				   "M2/out-0-11-11-0.csv", \
				   "M2/out-0-3-0.csv"	
					]
	
	for i in range(len(file_names)):
		file_names[i] = base + file_names[i]

	weights = [3.0] *50
	
	start_times = [0,  1000, 700, 700] + 40 * [0]
	end_times = 40 * [100000000000000000000000000]

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


				t_speed.append(abs(float(row['RPM']) / 60.0 * 2 * pi))
					
				t_bus_voltage.append(abs(float(row['bus_voltage'])))
				t_output_voltage.append(abs(float(row['voltage'])))
				t_output_current.append(abs(float(row['current'])))
				t_temp.append((float(row['temp1']) + float(row['temp2']) + float(row['temp3'])) / 3.0)

				if(float(row['time_talon']) - start_time < 3.0):
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



	
	file_names_v =  [ "M2/custom_1/out_1.csv", \
				      "M2/custom_1/out_2.csv", \
				      "M2/custom_1/out_3.csv", \
				      "M2/custom_2/out_1.csv", \
				      "M2/custom_2/out_2.csv", \
				      "M2/custom_2/out_4.csv", \
				      "M2/custom_2/out_5.csv", \
				      "M2/custom_2/out_6.csv", \
				      "M2/custom_4/out_1.csv", \
				      "M2/custom_4/out_2.csv", \
				      "M2/custom_4/out_3.csv", \
				      "M2/custom_5/out_2.csv", \
				      "M2/custom_5/out_3.csv", \
				      "M2/custom_5/out_4.csv", \
					  "M1/out_smoked_5v.csv" , \
					  "M1/out_smoked_8v.csv"] 
	for i in range(len(file_names_v)):
		file_names_v[i] = base + file_names_v[i]

	

	start_times_v = [0] * 40
	end_times_v = [1000000000000000000000000000000] * 40
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

				if(float(row['time_talon']) - start_time < 3.0):
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
	
	for time_s, temp_s, rpm_s, current_s, voltage_s, name in zip(time, temp, speed, output_current, output_voltage, file_names): 
		plt.plot(time_s, rpm_s)
		plt.title(name)
		plt.show()

		plt.plot(time_s, current_s)
		plt.title(name)
		plt.show()

		plt.plot(time_s, voltage_s)
		plt.title(name)
		plt.show()

	print("post_plot")	

	for time_s, temp_s, rpm_s, current_s, voltage_s, name in zip(time_v, temp_v, speed_v, output_current_v, output_voltage_v, file_names_v): 

		plt.plot(time_s, rpm_s)
		plt.title(name)
		plt.show()

		plt.plot(time_s, current_s)
		plt.title(name)
		plt.show()

		plt.plot(time_s, voltage_s)
		plt.title(name)
		plt.show()

	'''


	srv = []
	for i in range(time.size):
		rospy.wait_for_service('/frcrobot/thermal_test' + str(i))
		srv.append(rospy.ServiceProxy('/frcrobot/thermal_test' + str(i), ModelTest))

	#def func(data, a, c):
#		return data[0] * data[0] *  a + data[1] * c

	guess=(-3.74237248181e-17,0.000465569613927,2.9347475349e-12,0.0154415885479,2.5056548193e-05,0.0134383685593,1.77046588332e-07,-6.56901486569e-06,-0.0969153419586,-6.38187352693e-05, 1.0, 1.0)
	guess=(-6.2801910663e-18,0.0447082181677,2.11180167865e-09,0.00344631641815,0.000169282835049,0.00164693174146, 6.73766667347e-08,-2.246529437e-07, 0.0969149418854, 3.71621381874e-07, 0.338627865961, 4.50600154105e-07)
	guess=(-6.2801910663e-18,0.0447082181677,2.11180167865e-09,0.00344631641815,2.11180167865e-09,0.00344631641815, 0.000169282835049,0.00164693174146,0.00164693174146, 6.73766667347e-08,-2.246529437e-07,2.246529437e-07, 0.0969149418854, 3.71621381874e-07, 0.338627865961, 4.50600154105e-07)
	

	force_plot_now = True
	plot_by_time = False
	
	import time as time_module

	start_time_var = time_module.time()
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
	

	ones = np.full(t_o.shape, 10000.0)
	
	start_time_var = time_module.time()

	static_start_time = start_time_var

	plot_by_time = True
	
	count_var = 0
	
	mse_c = 0
	print("")
	para, pcov = optimize.curve_fit(func, data, ones, guess)
	print("")
	print(para)
	#print(pcov)


	force_plot_now = True

	start_time_var = time_module.time()
	static_start_time = start_time_var

	
	
	mse_c = 0
	count_var = 0
	print("")
	
	func(data, para[0], para[1],para[2],para[3], para[4], para[5],para[6],para[7],para[8],para[9],para[10],para[11], para[12],para[13],para[14], para[15])
	
	print("")
	

	print("validation")
	start_time_var = time_module.time()
	static_start_time = start_time_var
	
	count_var = 0
	
	avg_temp = avg_temp_v
	temp = temp_v
	file_names = file_names_v

	
	
	t_o = temp[0]

	val = False

	for t in temp:
		if (not val):
			val = True
			continue
			
		t_o = np.hstack((t_o, t))
	
	
	ones = np.full(t_o.shape, 1.0)
	
	mse_c = 0
	print("")
	func(data_v, para[0], para[1],para[2],para[3], para[4], para[5],para[6],para[7],para[8], para[9],para[10],para[11], para[12],para[13],para[14], para[15])
	print("")


	


			

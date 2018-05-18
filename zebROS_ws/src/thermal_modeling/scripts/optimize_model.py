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
	

#data = np.array([time, speed, bus_voltage, output_voltage, output_current])

def func(data, e_term1, h_1, e_term2, h_2, v_1, v_2, v_squared_1, v_squared_2):

	gc.collect()	

	waste  = np.array([1.07901658e+00, 8.48587173e-02,  -5.37629287e-07, 1.05476919e-02])#, 6.52164444e+00, 7.06131978e-02])

	#1.07901658e+00  -5.37629287e-07   1.05476919e-02

	
	loss_resistance_0v, loss_resistance_12v, loss_v_squared_term, loss_v_term = waste[0], waste[1], waste[2], waste[3]



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
		p = ModelParams("test_talon", h_1, h_2, e_term1, e_term2, v_1, v_2, v_squared_1, v_squared_2, loss_v_term,  loss_v_squared_term, loss_resistance_12v, loss_resistance_0v,[i_temp, i_temp])
		#req = ModelTest(p, data[0], data[1], data[2], data[3], data[4])

		results.append(pool.apply_async(srv[index], (p, time_h, RPM_h, bus_voltage_h, output_voltage_h, output_current_h)))
		
		
		index += 1



		#print(out.temps[1].temps)
		#if(plot_by_time):
		#	print("founf it", str(start_time_var))	

	
		#print(str(count_var))


	for time_h, true_temp, result_yu in zip(data[0], temp, results):
		out = result_yu.get()
		if(value_b):
			value_b = False
			out_v = out.temps[1].temps
		else:
			out_v = np.hstack((out_v, out.temps[1].temps))

	
		if(force_plot_now):
			plt.ylim(290, 340)
			plt.plot(time_h, out.temps[1].temps)
			plt.plot(time_h, true_temp)
			plt.show()
		elif( (plot_by_time and count_var % 500 == 499) ):
			plt.ylim(290, 340)
			plt.plot(time_h, out.temps[1].temps)
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
	
	#mse_v = 0
	if(count_var % 50 == 0):
		mse_c = ((out_v - t_o)**2).mean(axis=None)

	count_var+=1


	pool.close()
	pool.join()
	del pool


	print("\033[Ktime: ", str( time_module.time() - static_start_time), " count: ", str(count_var), " compute time: ", str(time_module.time() - saved_time), " mse: ", mse_c, " vals: ", e_term1, h_1, e_term2, h_2, v_1, v_2, v_squared_1, v_squared_2, end='\r')

	sys.stdout.flush()

	

	gc.collect()	
	#print(out_v)
	return out_v

	
if __name__ == "__main__":
	
	print(os.getcwd())
	
	file_names =  [ "../Documents/hero_thermal_testing/data/M2/1.25v_stall/out_4.csv", "../Documents/hero_thermal_testing/data/M2/out_ramp.csv", "../Documents/hero_thermal_testing/data/M2/out_ramp2.csv", "../Documents/hero_thermal_testing/data/M2/out_ramp3.csv", "../Documents/hero_thermal_testing/data/M2/10v_setting_1/out_1.csv", "../Documents/hero_thermal_testing/data/M2/10v_setting_1/out_2.csv", "../Documents/hero_thermal_testing/data/M2/11v_setting_1/out_1.csv", "../Documents/hero_thermal_testing/data/M2/11v_setting_1/out_3.csv", "../Documents/hero_thermal_testing/data/M2/11v_setting_1/out_5.csv","../Documents/hero_thermal_testing/data/M2/2v_setting_3/out_3.csv", "../Documents/hero_thermal_testing/data/M2/2v_setting_3/out_4.csv", "../Documents/hero_thermal_testing/data/M2/2v_setting_3/out_5.csv", "../Documents/hero_thermal_testing/data/M2/out_2v_stall_1_min.csv", "../Documents/hero_thermal_testing/data/M2/out_2v_stall_1_min.csv","../Documents/hero_thermal_testing/data/M2/out_2v_stall_1_min.csv",] 
	start_times = [0,  1000, 700, 700, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	end_times = [1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000]

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


				if file_name != 	"../Documents/hero_thermal_testing/data/M2/out_2v_stall_1_min.csv":
					if(abs(float(row['RPM']) / 60.0 * 2 * pi) > max_speed):
						max_speed = abs(float(row['RPM']) / 60.0 * 2 * pi)
					t_speed.append(max_speed)#abs(float(row['RPM']) / 60.0 * 2 * pi))
				else:
					t_speed.append(abs(float(row['RPM']) / 60.0 * 2 * pi))
					
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



	file_names_v =  [ "../Documents/hero_thermal_testing/data/M2/custom_4/out_1.csv", "../Documents/hero_thermal_testing/data/M2/custom_4/out_2.csv", "../Documents/hero_thermal_testing/data/M2/custom_4/out_3.csv"] 
	start_times_v = [0, 0, 0]#, 0 , 1000, 700, 700, 0, 0, 0, 0, 0, 0, 0, 0]
	end_times_v = [1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000] #, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000, 1000000000000000000000000000000]

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

	guess = (-8.50661732936e-10, 0.555232751166, 6.36185394169e-09, -0.49505670238, -0.00199669493354, 0.00344950867011, 1.28466965937e-06, -1.35113514776e-06)



#-6.32446341422e-10, 4.97765036388, 9.70956815248e-09, -1.06062120703, -0.00609532038548, 5.12240309532e-05, 2.00778339947e-06, 7.21247173864e-07)

#(-3.11227070448e-21, 6.17602748899e-05, 1.46867835737e-09, -0.145411776589, 1.39805631561e-08, 0.000772984838754, 3.99094938963e-11, 2.42549306183e-07)#-3.18995504e-21, 1.76800020e-03, 2.64390193e-10, -1.35176212e-02, -2.10478845e-06, 9.55585693e-04, 7.01357332e-10, 1.84737141e-07, 5.20116721e-02, 100) 

	#(  8.20657694e-14, 4.79242215e-02, 2.76380873e-08, -2.78926481e+00, -9.74546040e-05,   8.43715698e-03,   1.18015128e-07,   5.27476615e-06, 0.35)

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
	

	start_time_var = time_module.time()

	static_start_time = start_time_var

	plot_by_time = True
	
	count_var = 0
	
	mse_c = 0
	print("")
	para, pcov = optimize.curve_fit(func, data, t_o, guess)
	print("")
	print(para)
	#print(pcov)


	force_plot_now = True

	start_time_var = time_module.time()
	static_start_time = start_time_var
	
	mse_c = 0
	count_var = 0
	print("")
	
	func(data, para[0], para[1],para[2],para[3], para[4], para[5],para[6],para[7])
	
	print("")
	

	print("validation")
	start_time_var = time_module.time()
	static_start_time = start_time_var
	
	count_var = 0
	
	avg_temp = avg_temp_v
	temp = temp_v


	
	t_o = temp[0]

	val = False

	for t in temp:
		if (not val):
			val = True
			continue
			
		t_o = np.hstack((t_o, t))
	
	mse_c = 0
	print("")
	func(data_v, para[0], para[1],para[2],para[3], para[4], para[5],para[6],para[7])
	print("")


	


			

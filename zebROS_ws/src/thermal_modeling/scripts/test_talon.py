import csv
import rospy
import sys
from talon_state_controller import *




if __name__ == "__main__":
	file_name =  str(sys.argv[1])
	time_start = 0
	time_end = 10000000000000000000000
	if(len(sys.argv) > 2):
		time_start  = float(sys.argv[2])
		time_end  = float(sys.argv[3])
		
	

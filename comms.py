#
# Written by Jesse Nolan
# ELEC3850
# Command control
#
import numpy as np
import serial
import time
import math
ser = serial.Serial('/dev/ttyACM0',9600)
ser.timeout = None

distances = []
angles = []
x_values = []
y_values = []
robot_x = 0
robot_y = 0
robot_r = 0
	
def receive_data():
	
	# format of data packet
	# |Array Size|distances|angles|left speed|right speed|time|
	# after each line is read, a response character is sent back to the robot
	
	line = ser.readline()
	
	print ("array size: " + line)
	
	line.strip()
	arr_size = int(line)
	
	for x in range(0,arr_size):
		line = ser.readline()
		line.strip()
		print ("distance %d: %s" %(x,line))
		print ("size: " + str(arr_size))
		distances.append(int(line))
	
	for x in range(0,arr_size):
		line = ser.readline()
		line.strip
		print ("angle %d: %s" %(x,line))
		print ("size: " + str(arr_size))
		angles.append(float(line))
		
	line = ser.readline()
	line.strip()
	left_speed = int(line)
	print ("left speed: " + line)
	
	line = ser.readline()
	line.strip()
	right_speed = int(line)
	print ("right speed: " + line)
	
	
	line = ser.readline()
	line.strip()
	timer = int(line)
	print ("timer: " + line)
	
	return distances,angles,left_speed,right_speed,timer,arr_size

def send_data(left_speed, right_speed, timer):

	print "sending dating"
	ser.write(str(left_speed) + '\n')
	time.sleep(0.1)
	ser.write(str(right_speed) + '\n')
	time.sleep(0.1)
	ser.write(str(timer) + '\n')
	time.sleep(0.1)
	ser.write(str(timer) + '\n')
	time.sleep(0.1)
	ser.write('\n\n')
	print "data sent"
	return

	
#Send "ready" signal over xigbee to start communicating
ser.flushInput()
ser.write("R\n\n")
ser.flushInput()
self = EKF()
ls = -100;
rs = 100;

while True:

	#Receive data from robot
	distances,angles,left_speed,right_speed,timer,arr_size = receive_data()
	time.sleep(1)

	#---------------------------------
	#Calculate robot x,y,z position here
	#---------------------------------

	#Writing robot data to file for RVIZ software
	file_object = open('/home/silenus/elec3850/rvizdata.txt','w')
	file_object.write(str(arr_size) + "\n")
	file_object.write("%f" %robot_x + "\n")
	file_object.write("%f" %robot_y + "\n")
	file_object.write("%f" %robot_r + "\n")
	for x in range(0,arr_size):
		file_object.write("%f" %x_values[x] + "\n")
	for x in range(0,arr_size):
		file_object.write("%f" %y_values[x] + "\n")
	file_object.close()
	
	#------------------------------------
	# Calculate new speeds and time here
	#------------------------------------
	
	#Send calculated speeds and timer to robot
	send_data(lspeed,rspeed,timer)

	ser.flushInput()

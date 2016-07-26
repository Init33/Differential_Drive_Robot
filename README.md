# Differential_Drive_Robot
This project has code for a self built robot that utilises LIDAR to navigate and runs on a Raspberry Pi. The Pi communicates robot telemetry data via zigbee to a laptop which processes this data and displays it in RVIZ which is a visual tool for the Robot Operating System (ROS) package in linux.

##robot.c/robot.h
These files contains robot peripheral drivers for motors, LIDAR and zigbee comms as well as a main function that the robot operates from. Drivers include serial bus interfacing, data decoding and wireless communications protocol.

##comms.py
This file contains the communications protocol for transferring data from the Raspberry Pi to a laptop

##rviz.cpp
This file contains ROS code that reads robot data from a file, populates data structures for RVIZ application and	transmits data to the RVIZ service

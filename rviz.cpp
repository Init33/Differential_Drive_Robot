/*	
*		Author: Jesse Nolan
*		Year: 2015
*		Description: ROS code that reads robot data from a file, populates data structures for RVIZ application and 
*									transmits data to the service
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#define BUFF 20
#define RESOLUTION 0.1

int read_file(float* size, float* r_x, float* r_y, float* r_r, float** x_values, float** y_values);

int main(int argc, char** argv)
{
	float* x_values;
	float* y_values;
	float size,robot_x,robot_y,robot_r;
	int size1;
	int i;
	
	printf("initialising ros\n");
	
	ros::init(argc,argv,"map_visual");
	ros::NodeHandle n;
	ros::Rate r(1); // 1Hz
	ros::Publisher grid_pub = n.advertise<nav_msgs::GridCells>("grid_topic",2000);
	ros::Publisher rob_pub = n.advertise<nav_msgs::GridCells>("rob_topic",50);
	ros::Publisher occ_pub = n.advertise<nav_msgs::OccupancyGrid>("occ_topic",2000);

		

	while(ros::ok())
	{
		read_file(&size, &robot_x,&robot_y,&robot_r,&x_values,&y_values);
		size1 = int(size);		

		printf("size: %d\n",size1);		

		nav_msgs::GridCells gridcells;
		nav_msgs::GridCells robot;
		nav_msgs::OccupancyGrid occupancygrid;
		
		printf("variables set\n");

		//header info
		gridcells.header.frame_id = "/my_frame";
		gridcells.header.stamp = ros::Time::now();
		robot.header.frame_id = "/my_frame";
		robot.header.stamp = ros::Time::now();

		printf("header set\n");
	
		gridcells.cell_width = 0.3; //100mm per cell
		gridcells.cell_height = 0.3; //100mm per cell

		robot.cell_width = 0.5;
		robot.cell_height = 0.5; 

		printf("cell w and h set\n");

		gridcells.cells.resize(size1);		

		//loads the lidar values into the rviz data structure
		for(i=0;i<size1;i++)
		{	
			gridcells.cells[i].x = x_values[i];
			gridcells.cells[i].y = y_values[i];
			gridcells.cells[i].z = 1.0;	

			printf("%f %f\n", gridcells.cells[i].x,gridcells.cells[i].y);
		}
		robot.cells.resize(1);
		robot.cells[0].x = robot_x;
		robot.cells[0].y = robot_y;
		
		occupancygrid.header = gridcells.header;
		
		printf("second header set\n");

		occupancygrid.info.map_load_time = ros::Time::now();
		occupancygrid.info.resolution = 0.1;
		occupancygrid.info.width = 100;
		occupancygrid.info.height = 100;
		occupancygrid.info.origin.position.x = 0;
		occupancygrid.info.origin.position.y = 0;
		occupancygrid.info.origin.position.z = 0;
		occupancygrid.info.origin.orientation.x = 0;
		occupancygrid.info.origin.orientation.y = 0;
		occupancygrid.info.origin.orientation.z = 0;
		occupancygrid.info.origin.orientation.w = 1;
			
		printf("info set\n");		

		while ((grid_pub.getNumSubscribers() < 1) /*| (occ_pub.getNumSubscribers() < 1)*/)
		{
			if (!ros::ok())
       		{
  	     		return 0;
         	}
         ROS_WARN_ONCE("Please create a subscriber to the marker");
         sleep(1);	
		}
		
		grid_pub.publish(gridcells);
		rob_pub.publish(robot);
		
		r.sleep();
	}	
}

int read_file(float* size, float* r_x, float* r_y, float* r_r, float** x_values, float** y_values)
{
	// data format
	// |data_points|robot_pose|x_values|y_values|
	
	FILE *fp;
	char buffer[BUFF];
	float data_size, robot_x, robot_y, robot_r;
	int i;
	
	//printf("opening the file\n");

	fp = fopen("/home/silenus/elec3850/rvizdata.txt","r");
	while(fp == '\0')
	{
		printf("attempting to open\n");
		fp = fopen("/home/silenus/elec3850/rvizdata.txt","r");
	}
	
	//printf("getting data size\n");
	
	fgets(buffer,BUFF,fp);
	data_size = atof(buffer);
	
	*size = data_size;
	
	//printf("getting robot x\n");
	
	fgets(buffer,BUFF,fp);
	robot_x = atof(buffer);
	*r_x = robot_x;
	
	//printf("getting robot y\n");

	fgets(buffer,BUFF,fp);
	robot_y = atof(buffer);
	*r_y = robot_y;

	//printf("getting robot r\n");
	
	fgets(buffer,BUFF,fp);
	robot_r = atof(buffer);
	*r_r = robot_r;
	
	*x_values = (float*)malloc(sizeof(float)*data_size);
	
	for(i=0;i<data_size;i++)
	{
		//printf("getting %d value of x\n",i);
		fgets(buffer,BUFF,fp);
		(*x_values)[i] = atof(buffer);
	}
	
	*y_values = (float*)malloc(sizeof(float)*data_size);
	
	for(i=0;i<data_size;i++)
	{
		//printf("getting the %d value of y\n", i);
		fgets(buffer,BUFF,fp);
		(*y_values)[i] = atof(buffer);
	}
	fclose(fp);
	return 0;
}

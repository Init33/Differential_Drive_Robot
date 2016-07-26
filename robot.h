/*
*		Author: Jesse Nolan
*		Year: 2015
*	    Description: see readme
*
*/

#ifndef ROBOT_H
#define ROBOT_H

#define ADD_ID							0x03
#define ADD_BAUD						0x04
#define ADD_CW_ANGLE_LIMIT				0x06
#define ADD_CCW_ANGLE_LIMIT				0x08
#define ADD_MOVING_SPEED 				0x20
#define ADD_PRESENT_SPEED 				0x26

#define HEADER							0xFF

#define SET_ID_LENGTH					0x04
#define SET_BAUD_LENGTH					0x04
#define SET_WHEEL_LENGTH				0x07
#define SET_SPEED_LENGTH				0x05
#define READ_SPEED_LENGTH				0x04

#define READ							0x02
#define WRITE							0x03

#define LEFT_WHEEL 1
#define RIGHT_WHEEL 3

#define LIDAR_START 44
#define LIDAR_END 725

#define MIN_THRESHOLD 30
# define MAX_THRESHOLD 100

int xbee_init(void);
int xbee_send_data(int* distances, float* angles, int left_speed, int right_speed, int size_of_array, int time_taken);
int xbee_receive(int* left_speed, int* right_speed, int* time);
int PC_ready(void);
int unexpected_obstacles(int distances[], int data_points, int left_speed, int right_speed);

int LIDAR_init(void);
int laser_on(void);
int get_measurement(int start, int end, unsigned int distances[], float angles[],int* data_points);
unsigned int three_decode(char up, char mid, char bot);
int send_data(char data, int file_name);
int receive_data(char buffer[], int file_name);

int open_UART(void);
int setID(unsigned char ID, unsigned char newID);
int setbaud(unsigned int ID, int baud);
int setwheelmode(unsigned int ID);
int setspeed(unsigned char ID, int speed);

//Struct used to create a bitmask for decoding LIDAR data packet
typedef struct
	{
		unsigned char lower : 6;
		unsigned char middle : 6;
		unsigned char upper : 6;
		unsigned char fill : 6;
		
	} Decode_t;

//Union to overlay manipulated bitmask data into single int size value
union Convert
	{
		unsigned int value;
		Decode_t decode;
	};
#endif

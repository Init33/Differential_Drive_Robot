/*
* Written by Jesse Nolan
* ELEC3850
* Robot peripheral control code
*/
#include "robot.h"
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

int file, file2, file3;

int main(void)
{
	unsigned int distances[800];
	float angles[800];
	unsigned int data_points;
	int left_speed, right_speed, time, msec, stop;
	long int start, diff;
	struct timeval mytime;
	
	left_speed = 0;
	right_speed = 0;
	
	//initialising peripherals
	printf(" --- INITIALISING DYNAMIXEL PORT---\n");
	open_UART();
	usleep(1000000);
	printf(" --- INITIALISING XBEE PORT---\n");
	xbee_init();
	usleep(1000000);
	printf("--- INITIALISING LIDAR PORT ---\n");
	LIDAR_init();
	usleep(1000000);
	printf("--- SETTING WHEEL SPEED TO ZERO ---\n");
	setwheelmode(LEFT_WHEEL);
	setwheelmode(RIGHT_WHEEL);
	usleep(1000000);
	setspeed(LEFT_WHEEL,left_speed);
	setspeed(RIGHT_WHEEL,right_speed);
	printf("--- TURNING LASER ON --- \n");
	laser_on();
	printf("--- WAITING FOR PC --\n");
	PC_ready();
	
	while(1)
	{
		//get lidar data
		printf("--- GETTING LIDAR DATA --- \n");
		get_measurement(LIDAR_START,LIDAR_END,distances,angles,&data_points);	
		//send data
		printf("--- SENDING DATA TO PC ---\n");
		xbee_send_data(distances,angles,left_speed,right_speed,data_points,msec);
		//wait for instruction
		printf("--- WAITING FOR RECEIVE INSTRUCTIONS ---\n");
		xbee_receive(&left_speed,&right_speed,&time);
		
		//begin moving
		setspeed(LEFT_WHEEL,left_speed);
		setspeed(RIGHT_WHEEL,right_speed);
		
		//start clock
		gettimeofday(&mytime,NULL);
		start = mytime.tv_usec;
		diff = 0;
		
		//execute movement command
		while(diff < time*1000000)
		{
			//Read LIDAR data
			get_measurement(LIDAR_START,LIDAR_END,distances,angles,&data_points);	
			//Check if unexpected obstacle in the way
			stop = unexpected_obstacles(distances,data_points,left_speed,right_speed);
			//If obstacle detected, stop robot and record time
			if(stop == 1)
			{
				setspeed(LEFT_WHEEL,0);
				setspeed(RIGHT_WHEEL,0);
				gettimeofday(&mytime,NULL);
				diff = mytime.tv_usec - start;
				break;
			}
			printf("diff: %lu\n",diff);
			gettimeofday(&mytime,NULL);
			//calculate elapsed time
			diff = mytime.tv_usec - start;
		}
		setspeed(254,0);
	}	
}

// Waits for the ready command to be sent from the PC
int PC_ready(void)
{
	char buff[10];
	receive_data(buff,file);
	if(buff[0] == 'R')
	{
		printf("PC ready to start \n");
		return 0;
	}
}

//Detects if there are any obstacles within the allowable distances threshold that the robot may collide with
int unexpected_obstacles(int distances[], int data_points, int left_speed, int right_speed)
{
	int i;
	//if the robot isnt turning on the spot
	if(left_speed != -1*right_speed)
	{
		for(i=0;i<data_points;i++)
		{
			if((distances[i] > MIN_THRESHOLD) && (distances[i] < MAX_THRESHOLD))
			{
				return 1;
			}
		}
	}
	return 0;
}

//Reads data from the xbee serial buffer and converts data to speeds and time
int xbee_receive(int* left_speed, int* right_speed, int* time)
{
	char buff[40];
	char str[10];
	int temp[3];
	int check,i,j,k;
	int lspeed, rspeed, timer;
	
	tcflush(file,TCIOFLUSH);
	//Reads data to buffer
	receive_data(buff,file);
	
	i = 0;
	j = 0;
	k = 0;
	//Processes data separated by \n
	while((buff[i] != '\n') | (buff[i+1] != '\n'))
	{
		if(buff[i] != '\n')
		{
			str[j] = buff[i];
			j++;
		}
		if(buff[i] == '\n')
		{
			str[j] = '\0';
			temp[k] = atoi(str);
			k++;
			j = 0;
		}
		i++;
	}
	
	*left_speed = temp[0];
	*right_speed = temp[1];
	*time = temp[2];
}

//Initialises the xbee
int xbee_init(void)
{
	printf("opening file\n");
	file = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	if(file == -1)
	{
		perror("Unable to open /dev/ttyAMA0\n");
		return -1;
	}

	printf("setting up connection\n");
	struct termios options1;
	tcgetattr(file, &options1);
	options1.c_cflag = B9600 | CS8 | CLOCAL | CREAD;  
	options1.c_iflag = IGNPAR;
	options1.c_oflag = 0;
	options1.c_lflag = 0;
	tcflush(file, TCIFLUSH);
	tcsetattr(file, TCSANOW, &options1);
	fcntl(file, F_SETFL, 0);
	return 0;
}

//Processes data and sends over xbee
int xbee_send_data(int* distances, float* angles, int left_speed, int right_speed, int size_of_array, int time_taken)
{
	int fd,i;
	char buff[6];
	int size;
	char str[30];
	//Sends array dimension data
	sprintf(str,"%d\n",size_of_array);
	size = strlen(str);
	fd = write(file,str,size);
	if(fd == -1)
		{
			perror("error writing data");
			return -1;
		}
	usleep(7000);
	
	//Each loop sends the i'th array element of distances
	for(i=0;i<size_of_array;i++)
	{
		sprintf(str,"%d\n",distances[i]);
		size = strlen(str);
		//write the distance value
		fd = write(file,str,size);
		if(fd == -1)
			{
				perror("error writing data");
				return -1;
			}
		usleep(7000);
	}
	
	//Each loop sends the i'th array element of angles
	for(i=0;i<size_of_array;i++)
	{
		sprintf(str,"%.3f\n",angles[i]);
		size = strlen(str);
		//write the distance value
		fd = write(file,str,size);
		if(fd == -1)
			{
				perror("error writing data");
				return -1;
			}
		usleep(7000);
	}
	
	//Write left speed
	sprintf(str,"%d\n",left_speed);
	size = strlen(str);
	
	fd = write(file,str,size);
	if(fd == -1)
		{
			perror("error writing data");
			return -1;
		}
	usleep(7000);
		
	//Write right speed
	sprintf(str,"%d\n",right_speed);
	size = strlen(str);
	
	fd = write(file,str,size);
	if(fd == -1)
		{
			perror("error writing data");
			return -1;
		}
	usleep(7000);
	
	//Write time taken
	sprintf(str,"%d\n",time_taken);
	size = strlen(str);
	
	fd = write(file,str,size);
	if(fd == -1)
		{
			perror("error writing data");
			return -1;
		}
	usleep(7000);
	return 0;
}

//Initialises the LIDAR
int LIDAR_init(void)
{
	printf("initialising the serial port\n");
	file2 = open("/dev/ttyACM0",O_RDWR | O_NOCTTY | O_NDELAY);
	if(file2 == -1)
	{
		perror("Unable to open /dev/ttyAMA0\n");
		return -1;
	}
	
	struct termios options2;
	tcgetattr(file2, &options2);
	options2.c_cflag = B115200 | CS8 | CLOCAL | CREAD;               //<Set baud rate
	options2.c_iflag = IGNPAR;
	options2.c_oflag = 0;
	options2.c_lflag = 0;
	tcflush(file2, TCIFLUSH);
	tcsetattr(file2, TCSANOW, &options2);

	fcntl(file2, F_SETFL, 0);
	return 0;
}

//Sends commands to turn the LIDAR laser on
int laser_on(void)
{
	int i;
	char status[40] = {0};
	char string[7];
	int check;
	
	//Send the BM command along with check string
	printf("LIDAR: sending BM command\n");
	send_data('B',file2);
	send_data('M',file2);
	send_data(';',file2);
	send_data('c',file2);
	send_data('h',file2);
	send_data('e',file2);
	send_data('c',file2);
	send_data('k',file2);
	send_data('\n',file2);
	//Wait for response
	printf("LIDAR: waiting for response...\n");
	check = receive_data(status,file2);
	while(check < 1)
	{
		check = receive_data(status,file2);
	}
	
	for(i=0;i<6;i++)
	{
		string[i] = status[i+2];
	}
	string[6] = '\0';
	
	printf("LIDAR: response string: %s and error code %x %x \n",string,status[9],status[10]);
	//Check if the response string "check" was valid
	if((strcmp(string,";check") == 0) && ((status[9]==0x30) && (status[10]==0x30)))
	{
		printf("LIDAR: Lidar laser is on\n");
		return 0;
	} else if((strcmp(string,";check") == 0) && ((status[9]==0x30) && (status[10]==0x31)))
	{
		perror("LIDAR: unable to control due to laser malfunction\n");
		return -3;
	}
	else if((strcmp(string,";check") == 0) && ((status[9]==0x30) && (status[10]==0x32))) 
	{
		perror("LIDAR: Laser is already on\n");
		return -2;
	} else
	{
		perror("LIDAR: unknown error\n");
		return -1;
	}
}

//Reads the current data from LIDAR
int get_measurement(int start, int end, unsigned int distances[], float angles[],int* data_points)
{
	char str1[6];
	char str2[6];
	char status[8192] = {0};
	unsigned int dtemp[800];
	float atemp[800];
	char header[30];
	char data[8192] = {0};
	int i, check, j, k;
	
	//Start and end positions for the LIDAR to read from
	sprintf(str1,"%04d",start);
	sprintf(str2,"%04d",end);
	//Send start/end command
	send_data('G',file2);
	send_data('D',file2);
	send_data(str1[0],file2);
	send_data(str1[1],file2);
	send_data(str1[2],file2);
	send_data(str1[3],file2);
	send_data(str2[0],file2);
	send_data(str2[1],file2);
	send_data(str2[2],file2);
	send_data(str2[3],file2);
	
	send_data('0',file2);
	send_data('4',file2);
	send_data(';',file2);
	send_data('c',file2);
	send_data('h',file2);
	send_data('e',file2);
	send_data('c',file2);
	send_data('k',file2);
	send_data('\n',file2);
	
	check = receive_data(status,file2);
	//wait for return status
	while(check < 1)
	{
		check = receive_data(status,file2);
	}
	//Process data
	for(i=0;i<29;i++)
	{
		header[i] = status[i];
	}
	
	j = 0;
	for(i=29;i<check;i++)
	{
		if(status[i] != '\n')
		{
			data[j] = status[i];
			j++;
		} else if (status[i] == '\n')
		{
			j--;
		}
	}
	//Decode LIDAR data	
	for(i=0;i<((j-1)/3);i++)
	{
		dtemp[i] = three_decode(data[(i*3)],data[(i*3)+1],data[(i*3)+2]);
		atemp[i] = (-(384-start)+(i*4))*0.3515625;		
	}
	
	k=0;
	//Gather valid data (over a threshold)
	for(i=0;i<((j-1)/3);i++)
	{
		if(dtemp[i] > 30)
		{
			distances[k] = dtemp[i];
			angles[k] = atemp[i];
			k++;
		}
	}
	
	*data_points = k;
	
	return 0;
}

//Decodes LIDAR data
unsigned int three_decode(char up, char mid, char bot)
{
	union Convert convert;
	//Process individual 6 bit fields	
	up = up - 0x30;
	mid = mid - 0x30;
	bot = bot - 0x30;
	//Populate union struct variable
	convert.decode.fill = 0;
	convert.decode.upper = up;
	convert.decode.middle = mid;
	convert.decode.lower = bot;
	//Return the int overlay of bit field data from union
	return convert.value;
}

//Sends data through xbee
int send_data(char data, int file_name)
{
	int n = write(file_name,(void*)&data,1);
	if(n<0)
	{
		perror("Write failed\n");
		return -1;
	}
	return 0;
}

//Reveives data  from xbee
int receive_data(char buffer[], int file_name)
{
	int bytes = 1;
	char temp;
	int i = 0;
	while(bytes != 0)
	{
		bytes = read(file_name,(void*)&temp,1);
		if (bytes < 0)
		{
			perror("read failed\n");
			return -1;
		}
		buffer[i] = temp;
		if((buffer[i] == 0x0A) && (buffer[i-1] == 0x0A))
		{
			tcflush(file_name,TCIOFLUSH);
			return (i-1);
		}
		
		i++;
	}
	tcflush(file_name,TCIOFLUSH);
	return (i-1);
}

//Opens UART serial port 
int open_UART(void)
{
	file3 = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if(file3 == -1)
	{
		perror("Unable to open /dev/ttyAMA0\n");
		return -1;
	}
	struct termios options3;
	tcgetattr(file3, &options3);
	options3.c_cflag = B115200 | CS8 | CLOCAL | CREAD | CSTOPB;               //<Set baud rate
	options3.c_iflag = IGNPAR;
	options3.c_oflag = 0;
	options3.c_lflag = 0;
	tcflush(file3, TCIFLUSH);
	tcsetattr(file3, TCSANOW, &options3);

	fcntl(file3, F_SETFL, 0);
	
	return 0;
}

// sets the ID of the dynamixel from current ID to new ID
int setID(unsigned char ID, unsigned char newID)
{
	int checksum = ~(ID + SET_ID_LENGTH + WRITE + ADD_ID + newID) & 0xFF;
	
	send_data(HEADER,file3);
	send_data(HEADER,file3);
	send_data(ID,file3);
	send_data(SET_ID_LENGTH,file3);
	send_data(WRITE,file3);
	send_data(ADD_ID,file3);
	send_data(newID,file3);
	send_data((char)checksum,file3);
	usleep(100);
	return 0;
}

// sets the baud rate of the dynamixel
int setbaud(unsigned int ID, int baud)
{
	int BAUD_CMD = (2000000/baud)+1;	
	int checksum = ~(ID + SET_BAUD_LENGTH + WRITE + ADD_BAUD + BAUD_CMD) & 0xFF;
	
	send_data(HEADER,file3);
	send_data(HEADER,file3);
	send_data(ID,file3);
	send_data(SET_BAUD_LENGTH,file3);
	send_data(WRITE,file3);
	send_data(ADD_BAUD,file3);
	send_data(BAUD_CMD,file3);
	send_data((char)checksum,file3);
	usleep(100);
	return 0;
}

//	sets both the CW and CCW direction registers to be zero 
int setwheelmode(unsigned int ID)
{
	int checksum = ~(ID + SET_WHEEL_LENGTH + WRITE + ADD_CW_ANGLE_LIMIT) & 0xFF;
	
	send_data(HEADER,file3);
	send_data(HEADER,file3);
	send_data(ID,file3);
	send_data(SET_WHEEL_LENGTH,file3);
	send_data(WRITE,file3);
	send_data(ADD_CW_ANGLE_LIMIT,file3);
	send_data(0x00,file3); //set CW Low byte
	send_data(0x00,file3); //set CW high byte
	send_data(0x00,file3); //set CCW low byte
	send_data(0x00,file3); //set CCW High byte
	send_data((char)checksum,file3);
	usleep(100);
	return 0;
}

// sets the dynamixel speed where 0-1023 is clockwise
// and 0-(-1023) is counter clockwise
int setspeed(unsigned char ID, int speed)
{
	int SPEED_SET;
	char SPEED_LB, SPEED_HB;
	
	if(ID == 1)
	{
		speed = -speed;
	}
	
	if(speed < 0)
	{
		if(speed < -1023)
		{
			perror("wheel speed set below -1023");
			return -1;
		}
		SPEED_SET = -1*speed;
	} else if (speed >= 0)
	{
		if(speed > 1023)
		{
			perror("wheel speed set above 1023");
			return -1;
		}
		SPEED_SET = speed + 1024;
	}
	SPEED_LB = (char)SPEED_SET;
	SPEED_HB = (char)(SPEED_SET >> 8);
	
	int checksum = ~(ID + SET_SPEED_LENGTH + WRITE + ADD_MOVING_SPEED + SPEED_HB + SPEED_LB) & 0xFF;

	send_data(HEADER,file3);
	send_data(HEADER,file3);
	send_data(ID,file3);
	send_data(SET_SPEED_LENGTH,file3);
	send_data(WRITE,file3);
	send_data(ADD_MOVING_SPEED,file3);
	send_data(SPEED_LB,file3);
	send_data(SPEED_HB,file3);
	send_data(checksum,file3);
	usleep(100);
	return 0;
}

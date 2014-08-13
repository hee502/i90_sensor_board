///////////////////////////////////////////////////////////////////////////////
//Source for i90_sensor node to update target position based on sensor redings/
//v1.0 																																			 //
//First creation 																														 //
//Huseyin Emre Erdem 																												 //
//11.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the sensor values from serial port, calculates the next target 
position to go and publishes this information i90_target topic. 
The topic contain messages of types std_msgs::Float32MultiArray*/

#include "ros/ros.h"
#include "Float32.h"
#include "Float32MultiArray.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/*Prototypes*/
int set_interface_attribs (int fd, int speed, int parity);//Sets serial port parameters

/*Main function*/
int main(int argc, char **argv){

	/*Variables*/
  char cData[15];//Characters to be read from the serial port
  char *cDataP = &data[0];//pointer to data
  uint8_t uiSonar[3];//Sonar values converted from received chars
	char *cPortNameP = "/dev/rfcomm0"
	float fSonarAngle[3] = {-70.00, 0.00, 70.00};//Angles of sonar sensors
	float fIrAngle[6] = {-70.00, -35.00, -15.00, 15.00, 35.00, 70.00};//Angles of infrared sensors

	/*Objects*/
	ros::init(argc, argv, "i90_sensor");//Create node called "i90_sensor"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Publisher targetPub = n.advertise<std_msgs::Float32MultiArray>("i90_target", 1);
	std_msgs::Float32MultiArray targetValue;//values to be published on the i90_ir topic

  /*Opening port*/
  int port = open(cPortNameP, O_RDWR | O_NOCTTY | O_NDELAY);
  if(port == -1){
	  ROS_INFO("Error opening the port");//Inform user on the terminal
  }
  else{
	  ROS_INFO("Serial port is open");//Inform user on the terminal
  }
	set_interface_attribs (port, B115200, 0);//Set

	while (ros::ok()){
		/*Send request and read incoming data*/
		tcflush(port, TCIFLUSH);//Empty the buffer
		write(port,"s",1);
		ROS_INFO("Data request sent");//Inform user on the terminal
		usleep(300000);//sleep for 300ms to allow mbed 
		read(port,dataP,15);//Read 15 bytes from the buffer

		targetValue = 0.1;
		ROS_INFO("Target: %f", targetValue);
		targetPub.publish(targetValue);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

int set_interface_attribs (int fd, int speed, int parity){
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
    printf("error from tcgetattr");
    return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
		                              // no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
		                              // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
    printf("error from tcsetattr");
    return -1;
	}
	return 0;
}


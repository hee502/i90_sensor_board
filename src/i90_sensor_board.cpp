///////////////////////////////////////////////////////////////////////////////
//Source for i90_sensor node to update target position based on sensor redings/
//v1.1 																																			 //
//Based on: v1.0																														 //
//Changelog:																																 //
//-Target calculation added																									 //
//Todo:																																			 //
//-Prevent not publishing new target in case of all obstructed directions		 //
//Huseyin Emre Erdem 																												 //
//13.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the sensor values from serial port, calculates the next target 
position to go and publishes this information i90_target_pos topic. The trigger is 
a new message publishment on i90_current_pos topic.
The topic publishes messages of types of i90_sensor_board::pos*/

#include "ros/ros.h"
#include "pos.h"//header file of special message type (x,y,yaw)
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define RADIUS 0.5//radius of new target position (origin = current position)

/*Variables*/
char *cpPortName = "/dev/rfcomm0";
char cData[15];//Characters to be read from the serial port
char *cpData = &cData[0];//pointer to data
uint8_t iSonar[3];//Sonar distances
float fIr[6];//Ir levels
//uint8_t uSonar[3];//Sonar values converted from received chars
float fSonarAngle[3] = {20.00, 90.00, 160.00};//Angles of sonar sensors
float fIrAngle[6] = {20.00, 55.00, 75.00, 105.00, 125.00, 160.00};//Angles of infrared sensors
float fMaxIr;//Max ir reading level
int iMaxIrNum;//# of ir sensor with the maximum reading
i90_sensor_board::pos targetValue;//values to be published on the i90_ir topic
bool bCalculation;//Flag to check if calculation of the new target is done

/*Prototypes*/
void recalculateTarget(const i90_sensor_board::pos i90CurrentPos);
int setInterfaceAttribs (int iFd, int iSpeed, int iParity);//Sets serial port parameters

/*Opening port*/
int iPort = open(cpPortName, O_RDWR | O_NOCTTY | O_NDELAY);

/*Main function*/
int main(int argc, char **argv){

	/*Objects*/
	ros::init(argc, argv, "i90_sensor");//Create node called "i90_sensor"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Publisher targetPub = n.advertise<i90_sensor_board::pos>("i90_target", 1);
	ros::Subscriber translationSub = n.subscribe("i90_current_pos", 1, recalculateTarget);

	/*Check the port*/
  if(iPort == -1){
	  ROS_INFO("Error opening the port");//Inform user on the terminal
  }
  else{
	  ROS_INFO("Serial port is open");//Inform user on the terminal
  }
	setInterfaceAttribs (iPort, B115200, 0);//Set
	bCalculation = false;

	while (ros::ok()){
		if(bCalculation == true){
			targetPub.publish(targetValue);
			ROS_INFO("Published new target: [%f]/t[%f]/t[%f]", targetValue.fXPos, targetValue.fYPos, targetValue.fYawAngle);
			bCalculation = false;
		}
		usleep(200000);//Wait for 200ms (5Hz)
		ros::spinOnce();
		//loop_rate.sleep();
	}
	return 0;
}

void recalculateTarget(const i90_sensor_board::pos i90CurrentPos){

	/*Read sensors on the mbed board*/
	tcflush(iPort, TCIFLUSH);//Empty the buffer
	write(iPort,"s",1);
	ROS_INFO("Data request sent");//Inform user on the terminal
	usleep(300000);//sleep for 300ms to allow mbed 
	read(iPort,cpData,15);//Read 15 bytes from the buffer

	/*Convert to integer/float values*/const drrobot_i90_player::MotorInfoArray MotorInfoArray);
	for(int i=0;i<3;i++){//Sonar values
		iSonar[i] = cData[i];
	}
	for(int i=0;i<6;i++){//Ir fraction
		fIr[i] += cData[3+i] / 100.00;
	}
	for(int i=0;i<6;i++){//Ir integer
		fIr[i] += (float) cData[3+2*i];
	}

	/*Calculate the direction with max ir and no obstacle*/

	/*Find the ir with maximum level*/
	fMaxIr = 0.00;
	iMaxIrNum = 0;
	for(int i=0;i<6;i++){
		if(fIr[i] >= fMaxIr){
			fMaxIr = fIr[i];
			iMaxIrNum = i;
		}
	}

	/*Check for obstacles*/
	switch(iMaxIrNum){
	case 0:
		if(iSonar[0] > 50){
			targetValue.fYawAngle = fIrAngle[0];
			targetValue.fXPos = i90CurrentPos.fXPos + cos(fIrAngle[0]) * RADIUS;
			targetValue.fYPos = i90CurrentPos.fXPos + sin(fIrAngle[0]) * RADIUS;
		}
		bCalculation = true;//Update flag to publish the calculated target
		break;
	case 1:
		if(iSonar[0] > 50 && iSonar[1] > 50){
			targetValue.fYawAngle = fIrAngle[1];
			targetValue.fXPos = i90CurrentPos.fXPos + cos(fIrAngle[1]) * RADIUS;
			targetValue.fYPos = i90CurrentPos.fXPos + sin(fIrAngle[1]) * RADIUS;
		}
		bCalculation = true;//Update flag to publish the calculated target
		break;
	case 2:
		if(iSonar[1] > 50){
			targetValue.fYawAngle = fIrAngle[2];
			targetValue.fXPos = i90CurrentPos.fXPos + cos(fIrAngle[2]) * RADIUS;
			targetValue.fYPos = i90CurrentPos.fXPos + sin(fIrAngle[2]) * RADIUS;
		}
		bCalculation = true;//Update flag to publish the calculated target
		break;
	case 3:
		if(iSonar[1] > 50){
			targetValue.fYawAngle = fIrAngle[3];
			targetValue.fXPos = i90CurrentPos.fXPos + cos(fIrAngle[3]) * RADIUS;
			targetValue.fYPos = i90CurrentPos.fXPos + sin(fIrAngle[3]) * RADIUS;
		}
		bCalculation = true;//Update flag to publish the calculated target
		break;
	case 4:
		if(iSonar[1] > 50 && iSonar[2] > 50){
			targetValue.fYawAngle = fIrAngle[4];
			targetValue.fXPos = i90CurrentPos.fXPos + cos(fIrAngle[4]) * RADIUS;
			targetValue.fYPos = i90CurrentPos.fXPos + sin(fIrAngle[4]) * RADIUS;
		}
		bCalculation = true;//Update flag to publish the calculated target
		break;
	case 5:
		if(iSonar[2] > 50){
			targetValue.fYawAngle = fIrAngle[5];
			targetValue.fXPos = i90CurrentPos.fXPos + cos(fIrAngle[5]) * RADIUS;
			targetValue.fYPos = i90CurrentPos.fXPos + sin(fIrAngle[5]) * RADIUS;
		}
		bCalculation = true;//Update flag to publish the calculated target
		break;
	}
	
	/*If the direction is obstructed*/
	if(bCalculation == false){
		
	}
}

int setInterfaceAttribs (int iFd, int iSpeed, int iParity){
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (iFd, &tty) != 0)
	{
    printf("error from tcgetattr");
    return -1;
	}

	cfsetospeed (&tty, iSpeed);
	cfsetispeed (&tty, iSpeed);

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
	tty.c_cflag |= iParity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (iFd, TCSANOW, &tty) != 0)
	{
    printf("error from tcsetattr");
    return -1;
	}
	return 0;
}


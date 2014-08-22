///////////////////////////////////////////////////////////////////////////////
//Source for i90_sensor node to update target position based on sensor redings/
//v1.5 																																			 //
//Based on: v1.4																														 //
//Changelog:																																 //
//-Solution for obstructed view added 																			 //
//ToDo:																																			 //
//-Add movement with 3 steps																								 //
//Huseyin Emre Erdem 																												 //
//22.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the sensor values from serial port, calculates the next target 
position to go and publishes this information i90_target_pos topic. The trigger is 
a new message publishment on i90_current_pos topic.
The topic publishes messages of types of i90_sensor_board::pos
Dynamic travel distance: max Ir level - travel distance
3.25 - 3.20: 0.1m
3.20 - 3.15: 0.2m
3.15 - 3.10: 0.3m
3.10 - 3.05: 0.4m
3.05 - ~   : 0.5m
In case of an obstructed view, robot turns right and then left
*/

#include "ros/ros.h"
#include "pos.h"//header file of special message type (x,y,yaw)
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>//for usleep()
#include <visualization_msgs/Marker.h>//for visualization
#include <cmath>
#define PI 3.141593

/*Variables*/
char *cpPortName = "/dev/rfcomm0";
char cData[15];//Characters to be read from the serial port
char *cpData = &cData[0];//pointer to data
uint8_t iSonar[3];//Sonar distances in cm
float fSonar[3];//Sonar distances in m
float fIr[6];//Ir levels
float fSonarAngle[3] = {40.00, 0.00, -40.00};//Angles of sonar sensors
float fIrAngle[6] = {50.00, 30.00, 10.00, -10.00, -30.00, -50.00};//Angles of infrared sensors relative to the robot body
float fMaxIr;//Max ir reading level
float fTravelDist;//Distance to travel during straight movement
int iIrOrder[2] = {};//Order of ir levels (descending). 0:max, 1:second max
float fCurrentPosX;
float fCurrentPosY;
float fCurrentAngleYaw;
int iMaxIrNum;//# of ir sensor with the maximum reading
bool bCalculation;//Flag to check if calculation of the new target is done
bool bBeacon;//0:beacon not found, 1:found
bool bTurnRight;//Shows if the robot checked right side for ir in case of an obstructed state
bool bTurnLeft;
bool bTurnCenter;
bool bOnlyTurn;
uint32_t shape = visualization_msgs::Marker::CUBE;
i90_sensor_board::pos targetValue;//values to be published on the i90_ir topic
int iObstacleNum = 0;
int iCounter = 0;

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
	ros::Publisher targetPub = n.advertise<i90_sensor_board::pos>("i90_target_pos", 1);
	ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("visualization_marker",10);
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
	bBeacon = false;
	bTurnRight = false;
	bTurnLeft = false;
	fTravelDist = 0.5;

	while (ros::ok()){
		if(bCalculation == true){
			/*Visualization*/
			for(int i=0;i<3;i++){
				if(fSonar[i] < 2.54){
					visualization_msgs::Marker marker;
					marker.header.frame_id = "/my_frame";
					marker.header.stamp = ros::Time::now();
					marker.ns = "basic_shapes";
					marker.id = iObstacleNum;
					iObstacleNum++;
					marker.type = shape;
					marker.action = visualization_msgs::Marker::ADD;
					marker.pose.orientation.x = 0.0;
					marker.pose.orientation.y = 0.0;
					marker.pose.orientation.z = 0.0;
					marker.pose.orientation.w = 1.0;
					marker.scale.x = 0.1;
					marker.scale.y = 0.1;
					marker.scale.z = 0.1;
					marker.color.r = 1.0f;
					marker.color.g = 0.0f;
					marker.color.b = 0.0f;
					marker.color.a = 1.0;
					marker.lifetime = ros::Duration();
					marker.pose.position.x = fCurrentPosX + fSonar[i] * cos((fCurrentAngleYaw + fSonarAngle[i]) / 180 * M_PI);
					marker.pose.position.y = fCurrentPosY + fSonar[i] * sin((fCurrentAngleYaw + fSonarAngle[i]) / 180 * M_PI);
					marker.pose.position.z = 0;
					markerPub.publish(marker);
				}
			}

			targetPub.publish(targetValue);
			ROS_INFO("Published new target: %f\t%f\t%f", targetValue.fXPos, targetValue.fYPos, targetValue.fYawAngle);
			bCalculation = false;
			bOnlyTurn = false;
		}
		usleep(200000);//Wait for 200ms (5Hz)
		ros::spinOnce();
		//loop_rate.sleep();
	}
	return 0;
}

void recalculateTarget(const i90_sensor_board::pos i90CurrentPos){

	/*Read current position values*/
	fCurrentPosX = i90CurrentPos.fXPos;
	fCurrentPosY = i90CurrentPos.fYPos;
	fCurrentAngleYaw = i90CurrentPos.fYawAngle;

	/*Request & read sensor values on the mbed board*/
	tcflush(iPort, TCIFLUSH);//Empty the buffer
	write(iPort,"s",1);
	ROS_INFO("Data request sent");//Inform user on the terminal
	usleep(300000);//sleep for 300ms to allow mbed 
	read(iPort,cpData,15);//Read 15 bytes from the buffer

	/*Clean values*/
	for(int i=0; i<6;i++){
		fIr[i] = 0;
	}

	/*Convert to integer/float values*/
	for(int i=0;i<3;i++){//Sonar values
		iSonar[i] = cData[i];
		fSonar[i] = iSonar[i] / 100.00;
	}
	ROS_INFO("-%d- Sonar: %f\t%f\t%f", iCounter, fSonar[0], fSonar[1], fSonar[2] );

	for(int i=0;i<6;i++){//Ir values
		fIr[i] += cData[4+2*i] / 100.00;//Decimal part
		fIr[i] += (float) cData[3+2*i];//Integer part
	}
	ROS_INFO("IR: %f\t%f\t%f\t%f\t%f\t%f", fIr[0], fIr[1], fIr[2], fIr[3], fIr[4], fIr[5] );

	/*Calculate the direction with max ir and no obstacle*/

	/*Find the ir with maximum level*/
	fMaxIr = 0.00;
	for(int i=0;i<6;i++){
		if(fIr[i] >= fMaxIr){
			fMaxIr = fIr[i];
			iIrOrder[0] = i;
		}
	}

	/*Find ir with second max*/
	fMaxIr = 0.00;
	for(int i=0;i<6;i++){
		if(i != iIrOrder[0]){
			if(fIr[i] >= fMaxIr){
				fMaxIr = fIr[i];
				iIrOrder[1] = i;
			}
		}
	}
	if(fIr[iIrOrder[0]] < 0.2){
		iIrOrder[0] = 2;
		iIrOrder[1] = 1;
	}
	ROS_INFO("-%d- maxIr: %u\t%u\t%f\t%f", iCounter, iIrOrder[0], iIrOrder[1], fIr[iIrOrder[0]], fIr[iIrOrder[1]]);

	/*Check if the beacon is reached*/
	if(fIr[iIrOrder[0]] >= 3.25){//If the threshold is reached
		bBeacon = true;
	}
	/*if(fIr[iIrOrder[0]] < 3.25 && fIr[iIrOrder[0]] >= 3.21){
			fTravelDist = 0.1;
	}
	if(fIr[iIrOrder[0]] < 3.21 && fIr[iIrOrder[0]] >= 3.16){
			fTravelDist = 0.2;
	}
	if(fIr[iIrOrder[0]] < 3.16 && fIr[iIrOrder[0]] >= 3.11){
			fTravelDist = 0.3;
	}
	if(fIr[iIrOrder[0]] < 3.11 && fIr[iIrOrder[0]] >= 3.06){
			fTravelDist = 0.4;
	}
	if(fIr[iIrOrder[0]] < 3.06){
		fTravelDist = 0.5;
	}
*/
	/*Check for obstacles to calculate relative turning angle*/

	if(bBeacon == false){
		switch(iIrOrder[0]){//Sensor with maximum IR level
		case 0:
			if(fSonar[0] > fTravelDist){
				if(iIrOrder[1] == 1){//If second max is sensor #1
					targetValue.fYawAngle = fIrAngle[0] - ((fIrAngle[0] - fIrAngle[1]) * fIr[1] / (fIr[0] + fIr[1]));//Use the weighted angle
				}
				else{
					targetValue.fYawAngle = fIrAngle[0];//Use only max Ir angle
				}
				bCalculation = true;//Update flag to publish the calculated target
			}
			break;
		case 1:
			if(fSonar[0] > fTravelDist){
				if(abs(iIrOrder[1] - 1) == 1){//Second max is a neighbour
						targetValue.fYawAngle = fIrAngle[1] - ((fIrAngle[1] - fIrAngle[iIrOrder[1]]) * fIr[iIrOrder[1]] / (fIr[1] + fIr[iIrOrder[1]]));//Use the weighted angle
				}
				else{
					targetValue.fYawAngle = fIrAngle[1];//use the angle directly
				}
				bCalculation = true;//Update flag to publish the calculated target
			}
			break;
		case 2:
			if(fSonar[1] > fTravelDist){
				if(abs(iIrOrder[1] - 2) == 1){//Second max is a neighbour
					targetValue.fYawAngle = fIrAngle[2] - ((fIrAngle[2] - fIrAngle[iIrOrder[1]]) * fIr[iIrOrder[1]] / (fIr[2] + fIr[iIrOrder[1]]));//Use the weighted angle
				}
				else{
					targetValue.fYawAngle = fIrAngle[2];
				}
				bCalculation = true;//Update flag to publish the calculated target
			}
			break;
		case 3:
			if(fSonar[1] > fTravelDist){
				if(abs(iIrOrder[1] - 3) == 1){//Second max is a neighbour
					targetValue.fYawAngle = fIrAngle[3] - ((fIrAngle[3] - fIrAngle[iIrOrder[1]]) * fIr[iIrOrder[1]] / (fIr[3] + fIr[iIrOrder[1]]));//Use the weighted angle
				}
				else{
					targetValue.fYawAngle = fIrAngle[3];
				}
				bCalculation = true;//Update flag to publish the calculated target
			}
			break;
		case 4:
			if(fSonar[2] > fTravelDist){
				if(abs(iIrOrder[1] - 4) == 1){//Second max is a neighbour
					targetValue.fYawAngle = fIrAngle[4] - ((fIrAngle[4] - fIrAngle[iIrOrder[1]]) * fIr[iIrOrder[1]] / (fIr[4] + fIr[iIrOrder[1]]));//Use the weighted angle
				}
				else{
					targetValue.fYawAngle = fIrAngle[4];
				}
				bCalculation = true;//Update flag to publish the calculated target
			}
			break;
		case 5:
			if(fSonar[2] > fTravelDist){
				if(iIrOrder[1] == 4){//Second max is a neighbour
					targetValue.fYawAngle = fIrAngle[5] - ((fIrAngle[5] - fIrAngle[4]) * fIr[4] / (fIr[5] + fIr[4]));//Use the weighted angle
				}
				else{
					targetValue.fYawAngle = fIrAngle[5];
				}
				bCalculation = true;//Update flag to publish the calculated target
			}
			break;
		}

		/*Use the angle of the second max if first is obstructed*/
		if(bCalculation == false){
			switch(iIrOrder[1]){//Sensor with maximum IR level
			case 0:
				if(fSonar[0] > fTravelDist){
					targetValue.fYawAngle = fIrAngle[0];
					bCalculation = true;//Update flag to publish the calculated target
				}
				break;
			case 1:
				if(fSonar[0] > fTravelDist){
					targetValue.fYawAngle = fIrAngle[1];
					bCalculation = true;//Update flag to publish the calculated target
				}
				break;
			case 2:
				if(fSonar[1] > fTravelDist){
					targetValue.fYawAngle = fIrAngle[2];
					bCalculation = true;//Update flag to publish the calculated target
				}
				break;
			case 3:
				if(fSonar[1] > fTravelDist){
					targetValue.fYawAngle = fIrAngle[3];
					bCalculation = true;//Update flag to publish the calculated target
				}
				break;
			case 4:
				if(fSonar[2] > fTravelDist){
					targetValue.fYawAngle = fIrAngle[4];
					bCalculation = true;//Update flag to publish the calculated target
				}
				break;
			case 5:
				if(fSonar[2] > fTravelDist){
					targetValue.fYawAngle = fIrAngle[5];
					bCalculation = true;//Update flag to publish the calculated target
				}
				break;
			}
		}

		/*If all the directions are obstructed, turn right*/
		if(bCalculation == false && bTurnCenter == false && bTurnRight == false){
			targetValue.fYawAngle = -90.00;//Turn right
			bCalculation = true;//Update flag to publish the calculated target
			bTurnRight = true;
			bOnlyTurn = true;
		}

		/*If all the directions are obstructed and also right is checked, turn left to the center*/
		if(bCalculation == false && bTurnRight == true && bTurnCenter == false){
			targetValue.fYawAngle = 90.00;//Turn left
			bCalculation = true;//Update flag to publish the calculated target
			bTurnCenter = true;//Ignore not finding enough ir in the left
			bOnlyTurn = true;
		}
	
		if(bCalculation == false && bTurnCenter == true && bTurnCenter == true){
			targetValue.fYawAngle = 90.00;//Turn left
			bCalculation = true;//Update flag to publish the calculated target
			bTurnLeft = true;//Ignore not finding enough ir in the left		
			bOnlyTurn = true;
		}

		ROS_INFO("-%d- Relative angle: %f", iCounter, targetValue.fYawAngle);
		ROS_INFO("-%d- Current angle: %f", iCounter, fCurrentAngleYaw);
		ROS_INFO("-%d- bCalculation: %d", iCounter, bCalculation);

		/*Transform relative angles to the general coordinate frame*/
		if(bCalculation == true){//If new target is found
			targetValue.fYawAngle += fCurrentAngleYaw;//Add current yaw to transfer to original frame
			if(targetValue.fYawAngle < 0.00) targetValue.fYawAngle += 360.00;//Always keep between 0-360
			if(targetValue.fYawAngle > 360.00) targetValue.fYawAngle -= 360.00;
			if(bOnlyTurn == false){//If target is not obstructed
				targetValue.fXPos = fCurrentPosX + cos(targetValue.fYawAngle * PI / 180.00) * fTravelDist;//Increment on X axis
				targetValue.fYPos = fCurrentPosY + sin(targetValue.fYawAngle * PI / 180.00) * fTravelDist;//Increment on Y axis
				bTurnRight = false;
				bTurnLeft = false;
				bTurnCenter = false;
				iCounter++;
			}
			else{//If target is obstructed
				targetValue.fXPos = fCurrentPosX;//Only turn
				targetValue.fYPos = fCurrentPosY;
			}
		}
	}

	/*If beacon is found*/
	else{
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


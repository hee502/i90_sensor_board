///////////////////////////////////////////////////////////////////////////////
//Source for i90_sensor node to update target position based on sensor redings/
//v3.3 																																			 //
//-Waiting limit (2m) for ardrone is added
//Huseyin Emre Erdem 																												 //
//30.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////
//WORKS WITH V3.1 & V2.9
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
#include <std_msgs/UInt8.h>
#define PI 3.141593
#define ECCENTRIC 0.16//Distance between sensor and robot center
#define THRESHOLD 3.20

/*Variables*/
char *cpPortName = "/dev/rfcomm0";
char *cpDronePortName = "/dev/ttyUSB0";
char *cpMbedPortName = "/dev/ttyACM0";
char cData[15];//Characters to be read from the serial port
char *cpData = &cData[0];//pointer to data
volatile uint8_t iSonar[3];//Sonar distances in cm
volatile float fSonar[3];//Sonar distances in m
volatile float fIr[6];//Ir levels
float fSonarAngle[3] = {40.00, 0.00, -40.00};//Angles of sonar sensors
float fIrAngle[6] = {90.00, 40.00, 10.00, -10.00, -40.00, -90.00};//Angles of infrared sensors relative to the robot body
volatile float fMaxIr;//Max ir reading level
volatile float fTravelDist = 0.75;//Distance to travel during straight movement
volatile int iIrOrder[2] = {};//Order of ir levels (descending). 0:max, 1:second max
volatile float fCurrentPosX;
volatile float fCurrentPosY;
volatile float fCurrentAngleYaw;
volatile int iMaxIrNum;//# of ir sensor with the maximum reading
volatile bool bCalculation;//Flag to check if calculation of the new target is done
volatile bool bBeaconFound;//0:beacon not found, 1:found
volatile bool bTurnRight;//Shows if the robot checked right side for ir in case of an obstructed state
volatile bool bTurnLeft;
volatile bool bTurnCenter;
volatile bool bOnlyTurn;
uint32_t shape = visualization_msgs::Marker::CUBE;
i90_sensor_board::pos targetPos;//Target position to be published on the i90_target_pos topic
volatile int iObstacleNum = 0;
volatile int iCounter = 0;
volatile int iTurncounter = 0;
volatile int iIrObstructed[6] = {};
volatile float fMaxIrAngle;
volatile bool bTurnTowardsIr = 0;
volatile bool bIrReception;
volatile bool bTargetSent = false;
volatile bool bDroneArrival = false;
const float fPassingPoint[2][2] = {{1.60, 0.00}, {0.00, 1.60}, };//Passing point [0] is on Y axis
volatile int iPassingPointNum = 0;
volatile float fDistanceToPassingPoint;
volatile bool bPassingPointReached = false;
volatile bool bHomeReached = false;
volatile float fPassingPointAngle;
volatile char cIncomingChar = '0';
volatile bool bSync = false;

/*Prototypes*/
void recalculateTarget(const i90_sensor_board::pos i90CurrentPos);
//void checkDroneArrival(const std_msgs::UInt8 droneState);
void updatePassingPoint(void);
float findDistance(float x1, float y1, float x2, float y2);
int setInterfaceAttribs (int iFd, int iSpeed, int iParity);//Sets serial port parameters

/*Opening port*/
int iPort = open(cpPortName, O_RDWR | O_NOCTTY | O_NDELAY);
int iDronePort = open(cpDronePortName, O_RDWR | O_NOCTTY | O_NDELAY);
int iMbedPort = open(cpMbedPortName, O_RDWR | O_NOCTTY | O_NDELAY);

/*Main function*/
int main(int argc, char **argv){

	/*Objects*/
	ros::init(argc, argv, "i90_sensor");//Create node called "i90_sensor"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Publisher targetPub = n.advertise<i90_sensor_board::pos>("i90_target_pos", 1);
	//ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("visualization_marker",10);
	ros::Subscriber translationSub = n.subscribe("i90_current_pos", 1, recalculateTarget);
	//ros::Subscriber droneArrivalSub = n.subscribe("drone_arrival", 1, checkDroneArrival);

	/*Prepare the serial ports*/
  if(iPort == -1){
	  printf("Error opening the port\n\r");//Inform user on the terminal
  }
  else{
	  printf("Sensor board serial port is OPEN\n\r");//Inform1409319601.666270183 user on the terminal
  }
	setInterfaceAttribs (iPort, B115200, 0);
	tcflush(iPort, TCIFLUSH);//Empty the buffer

  if(iDronePort == -1){
	  printf("Error opening the drone port\n\r");//Inform user on the terminal
  }
  else{
	  printf("Drone PC serial port is OPEN\n\r");//Inform user on the terminal
  }
	setInterfaceAttribs (iDronePort, B9600, 0);
	tcflush(iDronePort, TCIFLUSH);//Empty the buffer

  if(iMbedPort == -1){
	  printf("Error opening the mbed port\n\r");//Inform user on the terminal
  }
  else{
	  printf("Home beacon serial port is OPEN\n\r");//Inform user on the terminal
  }
	setInterfaceAttribs (iMbedPort, B115200, 0);
	tcflush(iMbedPort, TCIFLUSH);//Empty the buffer

	bCalculation = false;
	bBeaconFound = false;
	bTurnRight = false;
	bTurnLeft = false;
	fMaxIrAngle = 45.00;

	printf("Searching for the beacon.\n\r");

	while (ros::ok()){
		if(bCalculation == true){

			/*Publish the target*/
			targetPub.publish(targetPos);
			//ROS_INFO("Published new target: %f\t%f\t%f", targetPos.fXPos, targetPos.fYPos, targetPos.fYawAngle);
			bCalculation = false;
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
	//ROS_INFO("-%d- Received current pos: %f\t%f\t%f", iCounter, fCurrentPosX, fCurrentPosY, fCurrentAngleYaw);

	/*Request & read sensor values on the mbed board*/
	tcflush(iPort, TCIFLUSH);//Empty the buffer
	write(iPort,"s",1);
	//ROS_INFO("Data request sent");//Inform user on the terminal
	usleep(300000);//sleep for 300ms to allow mbed 
	read(iPort,cpData,15);//Read 15 bytes from the buffer

	/*Clean the variables*/
	for(int i=0; i<6;i++){
		fIr[i] = 0;
	}

	/*Convert to integer/float values*/
	for(int i=0;i<3;i++){//Sonar values
		iSonar[i] = cData[i];
		fSonar[i] = iSonar[i] / 100.00;
	}
	//printf("-%d- Sonar: %f\t%f\t%f\n\r", iCounter, fSonar[0], fSonar[1], fSonar[2]);

	for(int i=0;i<6;i++){//Ir values
		fIr[i] += cData[4+2*i] / 100.00;//Decimal part
		fIr[i] += (float) cData[3+2*i];//Integer part
	}
	//printf("IR: %f\t%f\t%f\t%f\t%f\t%f\n\r", fIr[0], fIr[1], fIr[2], fIr[3], fIr[4], fIr[5]);

	/*Look for the unobstructed directions*/
	if((fSonar[0] < 0.2) || (fSonar[1] < 0.16) || (fSonar[2] < 0.2)){//If front is blocked to do action1
		for(int i=1;i<5;i++){
			iIrObstructed[i] = 1;
		}
		//printf("Action1 cannot be performed.\n\r");
	}
	else{//No obstruction for action1, check for travelDist

		/*Check left sonar*/
		if(fSonar[0] > fTravelDist){//If not obstructed
			iIrObstructed[1] = 0;
		}
		else{//If obstructed
			iIrObstructed[1] = 1;
		}

		/*Check center sonar*/
		if(fSonar[1] > fTravelDist){//If not obstructed
			iIrObstructed[2] = 0;
			iIrObstructed[3] = 0;
		}
		else{//If obstructed
			iIrObstructed[2] = 1;
			iIrObstructed[3] = 1;
		}

		/*Check right sonar*/
		if(fSonar[2] > fTravelDist){//If not obstructed
			iIrObstructed[4] = 0;
		}
		else{//If obstructed
			iIrObstructed[4] = 1;
		}
	}

	/*Find IR sensor with max value on unobstructed directions*/
	if(bBeaconFound == false){
		fMaxIr = 0.00;
		iMaxIrNum = 0;
		for(int i=0;i<6;i++){
			if(fIr[i] >= THRESHOLD){
				if(bPassingPointReached == false){
					bBeaconFound = true;
					printf("Beacon is found.\n\r");
				}
				else{
					//printf("Reached home.REACHED HOME\n\r");
					bBeaconFound = true;
					bHomeReached = true;
				}
			}
			if(iIrObstructed[i] == 0){
				if(fIr[i] >= fMaxIr){
					fMaxIr = fIr[i];
					iMaxIrNum = i;
				}
			}
		}
		if(fMaxIr >= (THRESHOLD - 0.3)){
			fTravelDist = 0.2;
		}
		else{
			if(fMaxIr >= (THRESHOLD - 0.6)){
				fTravelDist = 0.3;
			}
			else{
				if(fMaxIr >= (THRESHOLD - 0.9)){
					fTravelDist = 0.4;
				}
				else{
					fTravelDist = 0.5;
				}
			}
		}

		/*Check if Ir is available*/
		if(fMaxIr <= 0.02){//If ir is not received
			iMaxIrNum = 3;//Go forward
			bIrReception = false;
		}
		else{
			bIrReception = true;
		}

		//printf("Maximum IR: %f, Num: %d\n\r", fMaxIr, iMaxIrNum);
	}

	if(bBeaconFound == false){

		/*If Ir is being received*/
		if(bIrReception == true){

			if(bPassingPointReached == false){
				targetPos.fYawAngle = fIrAngle[iMaxIrNum];
			}
			else{
				if(iMaxIrNum != 0 || iMaxIrNum != 5){
					targetPos.fYawAngle = fIrAngle[iMaxIrNum];
				}
			}

			if(iMaxIrNum == 0 || iMaxIrNum == 5){
				//targetPos.fYawAngle -= fIr[1] / (fIr[0] + fIr[1]) * (fIrAngle[0] - fIrAngle[1]);
			}
			if(iMaxIrNum == 2 || iMaxIrNum == 3){
				if(iMaxIrNum == 2){
					if((fMaxIr > 2.00) && (fIr[3] > fIr[1])){
						targetPos.fYawAngle -= fIr[3] / (fIr[2] + fIr[3]) * (fIrAngle[2] - fIrAngle[3]);
					}
				}
				else{
					if((fMaxIr > 2.00) && (fIr[2] > fIr[4])){
						targetPos.fYawAngle += fIr[2] / (fIr[2] + fIr[3]) * (fIrAngle[2] - fIrAngle[3]);
					}
				}
			}
			if(iMaxIrNum == 1 || iMaxIrNum == 4){
				if(iMaxIrNum == 1){
					if((fMaxIr > 2.00) && (fIr[2] > fIr[0])){
						targetPos.fYawAngle -= fIr[2] / (fIr[1] + fIr[2]) * (fIrAngle[1] - fIrAngle[2]);
					}
				}
				else{
					if((fMaxIr > 2.00) && (fIr[3] > fIr[5])){
						targetPos.fYawAngle += fIr[3] / (fIr[4] + fIr[3]) * (fIrAngle[3] - fIrAngle[4]);
					}
				}
			}
			bOnlyTurn = false;
			fMaxIrAngle = fCurrentAngleYaw + fIrAngle[iMaxIrNum];
			//printf("Ir is increasing. New target angle: %f\n\r", targetPos.fYawAngle);
		}

		/*If Ir is not received*/
		else{
			bool bFlag = false;

			/*If not yet turned to the right*/
			if(bFlag == false && bTurnRight == false && bTurnLeft == false){
				if(rand()%1){
					//printf("No Ir. Turning right.\n\r");
					targetPos.fYawAngle = -90.00;//Turn right
					bTurnRight = true;
					bOnlyTurn = true;
					bFlag = true;
				}
				else{
					//printf("No Ir. Turning left.\n\r");
					targetPos.fYawAngle = 90.00;//Turn right
					bTurnLeft = true;
					bOnlyTurn = true;
					bFlag = true;
				}				
			}

			/*If all the directions are obstructed and also right is checked, turn left to the center*/
			if(bFlag == false && bTurnRight == true && bTurnLeft == false && bTurnCenter == false){
				//printf("No Ir. Turning center.\n\r");
				targetPos.fYawAngle = 90.00;//Turn left
				bTurnCenter = true;//Ignore not finding enough ir in the left
				bOnlyTurn = true;
				bFlag = true;
			}

			if(bFlag == false && bTurnRight == false && bTurnLeft == true && bTurnCenter == false){
				//printf("No Ir. Turning center.\n\r");
				targetPos.fYawAngle = -90.00;//Turn right
				bTurnCenter = true;//Ignore not finding enough ir in the left
				bOnlyTurn = true;
				bFlag = true;
			}

			/*If all the directions are obstructed, right checked and robot is in the center dir, turn left*/
			if(bFlag == false && bTurnRight == true && bTurnCenter == true && bTurnLeft == false){
				//printf("No Ir. Turning left.\n\r");
				targetPos.fYawAngle = 90.00;//Turn left
				bTurnLeft = true;//Ignore not finding enough ir in the left		
				bOnlyTurn = true;
				bFlag = true;

			}

			if(bFlag == false && bTurnRight == false && bTurnCenter == true && bTurnLeft == true){
				//printf("No Ir. Turning right.\n\r");
				targetPos.fYawAngle = -90.00;//Turn right
				bTurnRight = true;//Ignore not finding enough ir in the left		
				bOnlyTurn = true;
				bFlag = true;

			}

			/*If no ir not received in front, right and left sides, go towards the last memory of max ir direction*/
			if(bFlag == false && bTurnRight == true && bTurnCenter == true && bTurnLeft == true){
				//printf("NO IR. Cur %f\t%f\n\r", fCurrentAngleYaw, fMaxIrAngle);
				/*If the angle diff with the last memory is smaller than 90 degrees*/				
				if(abs(fCurrentAngleYaw - fMaxIrAngle) < 90.00){
					targetPos.fYawAngle = fMaxIrAngle - fCurrentAngleYaw;
					bOnlyTurn = false;
				}
				else{
					if(fCurrentAngleYaw > fMaxIrAngle){//Target is on the right
						targetPos.fYawAngle = -90.00;
						bOnlyTurn = false;
					}
					else{//Target direction is on the left
						targetPos.fYawAngle = 90.00;
						bOnlyTurn = false;
					}
				}
			}
		}

		//ROS_INFO("-%d- Relative angle: %f", iCounter, targetPos.fYawAngle);
		//ROS_INFO("-%d- Current angle: %f", iCounter, fCurrentAngleYaw);

		/*Transform relative angles to the general coordinate frame*/
		if(bOnlyTurn == true){//Only Turn
			//printf("Only turning.\n\r");
			targetPos.fYawAngle += fCurrentAngleYaw;//Add current yaw to transfer to original frame
			if(targetPos.fYawAngle < 0.00) targetPos.fYawAngle += 360.00;//Always keep between 0-360
			if(targetPos.fYawAngle > 360.00) targetPos.fYawAngle -= 360.00;
			targetPos.fXPos = fCurrentPosX;
			targetPos.fYPos = fCurrentPosY;
			bCalculation = true;
			iCounter++;
		}

		else{//Go forward & Turn & go forward
			//printf("3 actions\n\r");
			targetPos.fYawAngle += fCurrentAngleYaw;//Add current yaw to transfer to original frame
			if(targetPos.fYawAngle < 0.00) targetPos.fYawAngle += 360.00;//Always keep between 0-360
			if(targetPos.fYawAngle > 360.00) targetPos.fYawAngle -= 360.00;
			/*Calculate turning point*/
			targetPos.fXPos = fCurrentPosX + cos(fCurrentAngleYaw * PI / 180.00) * ECCENTRIC;//Make turning center and sensor center equal
			targetPos.fYPos = fCurrentPosY + sin(fCurrentAngleYaw * PI / 180.00) * ECCENTRIC;
			if(iMaxIrNum != 0 || iMaxIrNum != 5){
				/*Calculate target point*/
				targetPos.fXPos += cos(targetPos.fYawAngle * PI / 180.00) * fTravelDist;//Increment on X axis
				targetPos.fYPos += sin(targetPos.fYawAngle * PI / 180.00) * fTravelDist;//Increment on Y axis
			}
			bCalculation = true;
			bTurnRight = false;
			bTurnLeft = false;
			bTurnCenter = false;
			iCounter++;
		}
	}

	/*If beacon is found*/
	else{

		/*Send the target to the drone*/
		while(bTargetSent == false){
			/*Send "a" and wait for "b" in return*/
			tcflush(iDronePort, TCIFLUSH);//Empty the buffer
			write(iDronePort,"a",1);
			printf("Sync signal has been sent to the Drone.\n\r");
			/*Wait until the reply*/
			char cIncomingChar = 'a';
			char *cpIncomingChar = &cIncomingChar;

			int iCount = 0;
			while(iCount < 120){
				read(iDronePort, cpIncomingChar, 1);//Read 1 byte from the buffer
				if(cIncomingChar != 'b'){
					bSync = true;
					break;
				}
				usleep(1000000);
				iCount++;
				printf("Time passed: %d\n\r", iCount);
			}
			if(iCount == 120){
				printf("Synchronization failed.\n\r");
				bSync = false;
			}

			if(bSync == true){
				/*Calculate the characters to be sent*/
				//printf("%f\t%f\n\r", fCurrentPosX, fCurrentPosY);
				char cToSend[4];
				char *cpToSend = &cToSend[0];
				uint8_t iX = (int) floor(fCurrentPosX);
				uint8_t iY = (int) floor(fCurrentPosY);
				cToSend[0] = iX;
				cToSend[2] = iY;
				float fX = fCurrentPosX - iX;
				float fY = fCurrentPosY - iY;
				fX *= 100.00;
				fY *= 100.00;
				uint8_t iX2 = fX;
				uint8_t iY2 = fY;
				cToSend[1] = iX2;
				cToSend[3] = iY2;

				/*Write on the serial port*/
				tcflush(iDronePort, TCIFLUSH);//Empty the buffer
				write(iDronePort,cpToSend,4);
				bTargetSent = true;
				printf("Target position has been sent to the drone-PC.\n\r");
			}
		}

		if(bSync == true){

			/*If drone not arrived yet*/
			usleep(1000000);
			char cIncomingChar = 'a';
			char *cpIncomingChar = &cIncomingChar;

			printf("Waiting for the Drone to arrive.\n\r");

			int iCount = 0;	

			while(iCount < 120){
				read(iDronePort, cpIncomingChar, 1);//Read 1 byte from the buffer
				if(cIncomingChar == 'c'){
					break;
				}
				if(cIncomingChar == 'e'){
					break;
				}
				usleep(1000000);
				iCount++;
				printf("Time passed: %d\n\r", iCount);
			}

			if(iCount == 120){
				printf("Drone couldn't make it.\n\rGoing back to passing point.\n\r");
			}
			else{
				if(cIncomingChar == 'c'){
					printf("Drone has arrived.\n\r");
				}
				else{
					printf("Drone couldn't make it.\n\rGoing back to passing point.\n\r");	
				}
			}
			bSync = false;
		}

		/*Drone already arrived, going home*/
		/*Go to the passing point if not reached*/
		if(bPassingPointReached == false){
			updatePassingPoint();
		}

		if(bPassingPointReached == false){
			//go to passing point
			//printf("\n\rGOING TO PASSING POINT %d.\n\r", iPassingPointNum);

			/*Turn towards passing point*/
			if(fCurrentAngleYaw < 180.00){
				if(fPassingPointAngle > fCurrentAngleYaw && fPassingPointAngle < (fCurrentAngleYaw + 180.00)){//PP is on the left, turn CCW
					if((fPassingPointAngle - fCurrentAngleYaw) >= 90.00){//If the difference is > 90 degrees
						targetPos.fYawAngle = 90.00;//Turn 90 degrees
						bOnlyTurn = true;
					}
					else{//Turn up to difference
						targetPos.fYawAngle = fPassingPointAngle - fCurrentAngleYaw;
						bOnlyTurn = false;
					}
				}
				else{//Target is on the right, turn CW
					if(fPassingPointAngle < 180.00){//Target between right and 0
						if((fCurrentAngleYaw - fPassingPointAngle) >= 90.00){
							targetPos.fYawAngle = -90.00;
							bOnlyTurn = true;
						}
						else{
							targetPos.fYawAngle = fPassingPointAngle - fCurrentAngleYaw;
							bOnlyTurn = false;
						}
					}
					else{//Target between 360 and current+180
						if((360.00 - fPassingPointAngle + fCurrentAngleYaw) >= 90.00){
							targetPos.fYawAngle = -90.00;
							bOnlyTurn = true;
						}
						else{
							targetPos.fYawAngle = 360.00 - fPassingPointAngle + fCurrentAngleYaw;
							bOnlyTurn = false;
						}
					}
				}
			}
			else{
				if(fPassingPointAngle > (fCurrentAngleYaw - 180.00) && fPassingPointAngle < fCurrentAngleYaw){//Target is on the right, turn CW
					if(abs(fCurrentAngleYaw - fPassingPointAngle) >= 90.00){
						targetPos.fYawAngle = -90.00;
						bOnlyTurn = true;
					}
					else{
						targetPos.fYawAngle = fPassingPointAngle - fCurrentAngleYaw;
						bOnlyTurn = false;
					}
				}
				else{//Target is on the left, turn CCW
					if(fPassingPointAngle < 180.00){//Target is between 0 & current-180
						if(abs(360.00 - fCurrentAngleYaw + fPassingPointAngle) >= 90.00){
							targetPos.fYawAngle = 90.00;
							bOnlyTurn = true;
						}
						else{
							targetPos.fYawAngle = 360.00 - fCurrentAngleYaw + fPassingPointAngle;
							bOnlyTurn = false;
						}
					}
					else{//Target is between current & 360
						if(abs(fPassingPointAngle - fCurrentAngleYaw) >= 90.00){
							targetPos.fYawAngle = 90.00;
							bOnlyTurn = true;
						}
						else{
							targetPos.fYawAngle = fPassingPointAngle - fCurrentAngleYaw;
							bOnlyTurn = false;
						}
					}
				}
			}
			//ROS_INFO("-%d- Relative angle: %f", iCounter, targetPos.fYawAngle);
			//ROS_INFO("-%d- Current angle: %f", iCounter, fCurrentAngleYaw);

			/*Transform relative angles to the general coordinate frame*/
			if(bOnlyTurn == true){//Only Turn
				//printf("Only turning.\n\r");
				targetPos.fYawAngle += fCurrentAngleYaw;//Add current yaw to transfer to original frame
				if(targetPos.fYawAngle < 0.00) targetPos.fYawAngle += 360.00;//Always keep between 0-360
				if(targetPos.fYawAngle > 360.00) targetPos.fYawAngle -= 360.00;
				targetPos.fXPos = fCurrentPosX;
				targetPos.fYPos = fCurrentPosY;
				bCalculation = true;
				iCounter++;
			}

			else{//Go forward & Turn & go forward
				//printf("3 actions\n\r");
				targetPos.fYawAngle += fCurrentAngleYaw;//Add current yaw to transfer to original frame
				if(targetPos.fYawAngle < 0.00) targetPos.fYawAngle += 360.00;//Always keep between 0-360
				if(targetPos.fYawAngle > 360.00) targetPos.fYawAngle -= 360.00;
				/*Calculate turning point*/
				targetPos.fXPos = fCurrentPosX + cos(fCurrentAngleYaw * PI / 180.00) * ECCENTRIC;//Make turning center and sensor center equal
				targetPos.fYPos = fCurrentPosY + sin(fCurrentAngleYaw * PI / 180.00) * ECCENTRIC;
				if(iMaxIrNum != 0 || iMaxIrNum != 5){
					/*Calculate target point*/
					targetPos.fXPos += cos(targetPos.fYawAngle * PI / 180.00) * fTravelDist;//Increment on X axis
					targetPos.fYPos += sin(targetPos.fYawAngle * PI / 180.00) * fTravelDist;//Increment on Y axis
				}
				bCalculation = true;
			}
		}
 
		/*Go home, passingPoint is already reached*/
		else{
			if(bHomeReached == false){
				printf("\n\rReached passing point. Returning home.\n\r");
				write(iMbedPort,"d",1);//Enable home beacon
				bBeaconFound = false;
				targetPos.fYawAngle = fCurrentAngleYaw;
				targetPos.fXPos = fCurrentPosX;
				targetPos.fYPos = fCurrentPosY;
				bCalculation = true;
			}
			else{
				printf("\n\rHOME SWEET HOME.\n\r");
				write(iMbedPort,"d",1);//Disable home beacon
			}
		}
	}
}
/*
void checkDroneArrival(const std_msgs::UInt8 droneState){
	bDroneArrival = true;
}
*/
float findDistance(float x1, float y1, float x2, float y2){
	float fDistX = x1 - x2;
	float fDistY = y1 - y2;
	float fDistance = pow(fDistX, 2);
	fDistance += pow(fDistY ,2);
	fDistance = sqrt(fDistance);
	return fDistance;
}

/*Calculates the distance to the passingPoints, selects proper one*/
void updatePassingPoint(void){
	float fDistances[2];
	fDistances[0] = findDistance(fCurrentPosX, fCurrentPosY, fPassingPoint[0][0], fPassingPoint[0][1]);
	fDistances[1] = findDistance(fCurrentPosX, fCurrentPosY, fPassingPoint[1][0], fPassingPoint[1][1]);
	//printf("Distances: %f\t%f\n\r", fDistances[0], fDistances[1]);

	if(fDistances[0] <= fDistances[1]){
		iPassingPointNum = 0;
		fDistanceToPassingPoint = fDistances[0];
	}
	else{
		iPassingPointNum = 1;
		fDistanceToPassingPoint = fDistances[1];
	}
	if(fDistanceToPassingPoint < 0.5){
		bPassingPointReached = true;
	}
	fPassingPointAngle = atan2((fPassingPoint[iPassingPointNum][1] - fCurrentPosY), (fPassingPoint[iPassingPointNum][0] - fCurrentPosX));
	fPassingPointAngle /= M_PI;
	fPassingPointAngle *= 180.00;
	if(fPassingPointAngle < 0.00) fPassingPointAngle += 360.00;
	if(fPassingPointAngle > 360.00) fPassingPointAngle -= 360.00;
	//printf("PP - num: %d\tdist: %f\tangle: %f\n\r", iPassingPointNum, fDistanceToPassingPoint, fPassingPointAngle);
}

int setInterfaceAttribs (int iFd, int iSpeed, int iParity){
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (iFd, &tty) != 0)
	{
    printf("Serial port error from tcgetattr\n\r");
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
    printf("Serial port error from tcsetattr\n\r");
    return -1;
	}
	return 0;
}


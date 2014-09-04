///////////////////////////////////////////////////////////////////////////////
//Source for i90_sensor node to update target position			     						 //
//v3.3 																																			 //
//-Comments added
//Huseyin Emre Erdem 																												 //
//30.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the sensor values from bluetooth serial port, calculates the 
next target position and publishes the position on i90_target_pos topic. 
The trigger is a new message published on i90_current_pos topic.
The topic publishes messages of types of i90_sensor_board::pos
The distance of target changes dynamically based on IR level:
max Ir level - travel distance
------------------------------
3.25 - 3.20: 0.1m
3.20 - 3.15: 0.2m
3.15 - 3.10: 0.3m
3.10 - 3.05: 0.4m
3.05 - ~   : 0.5m
In case of a fully obstructed view, robot checks right and left.
In terms of no IR reception, robot goes towards the last memory.
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
char *cpPortName = "/dev/rfcomm0";//Port name of bluetooth serial conn. to sensor board
char *cpDronePortName = "/dev/ttyUSB0";//Port name of cabled serial connection with usb converter
char *cpMbedPortName = "/dev/ttyACM0";//Port name for mbed home beacon connection
char cData[15];//Characters to be read from the serial port
char *cpData = &cData[0];//pointer to data
volatile uint8_t iSonar[3];//Sonar distances in cm
volatile float fSonar[3];//Sonar distances in m
volatile float fIr[6];//Ir levels (3.3 max)
float fSonarAngle[3] = {40.00, 0.00, -40.00};//Angles of sonar sensors relative to robot body
float fIrAngle[6] = {90.00, 40.00, 10.00, -10.00, -40.00, -90.00};//Angles of infrared sensors relative to the robot body
volatile float fMaxIr;//Max ir reading level
volatile float fTravelDist = 0.75;//Distance to travel during straight movement
volatile int iIrOrder[2] = {};//Order of ir levels (descending). 0:max, 1:second max
volatile float fCurrentPosX;//Current X pos.
volatile float fCurrentPosY;//Current Y pos
volatile float fCurrentAngleYaw;//Current yaw angle
volatile int iMaxIrNum;//# of ir sensor with the maximum reading
volatile bool bCalculation;//Flag to check if calculation of the new target is done
volatile bool bBeaconFound;//0:beacon not found, 1:found
volatile bool bTurnRight;//Shows if the robot checked right side for ir in case of an obstructed state
volatile bool bTurnLeft;//Flag to check if left checked before
volatile bool bTurnCenter;//Flag to check if center checked before
volatile bool bOnlyTurn;//Flag that defines if target consists only angle change
uint32_t shape = visualization_msgs::Marker::CUBE;//Obstacle definition for visualization
i90_sensor_board::pos targetPos;//Target position to be published on the i90_target_pos topic
volatile int iObstacleNum = 0;//Number of obstacles visualized
volatile int iCounter = 0;
volatile int iIrObstructed[6] = {};//Flag to represent if ir sensor direction is obstructed
volatile float fMaxIrAngle;//Angle of max ir reception
volatile bool bIrReception;//Flag representing if ir is received
volatile bool bTargetSent = false;//Flag representing if target has been sent
volatile bool bDroneArrival = false;//Flag representing if drone has arrived
const float fPassingPoint[2][2] = {{1.60, 0.00}, {0.00, 1.60}, };//Passing point positions
volatile int iPassingPointNum = 0;//# of closer passing point
volatile float fDistanceToPassingPoint;//Distance to passing point
volatile bool bPassingPointReached = false;//Flag represents if pp has been reached
volatile bool bHomeReached = false;//Flag representing if home has been reached
volatile float fPassingPointAngle;//The angle towards the pp from current loc.
volatile char cIncomingChar = '0';//Character read from ar-drone pc serial port
volatile bool bSync = false;//flag for synchronisation with ar-drone pc

/*Prototypes*/
void recalculateTarget(const i90_sensor_board::pos i90CurrentPos);//Provides new target upon a new current pos publishment
void updatePassingPoint(void);//Calculates the distance and angle to closer passing point
float findDistance(float x1, float y1, float x2, float y2);//Finds distance between two positions
int setInterfaceAttribs (int iFd, int iSpeed, int iParity);//Sets serial port parameters

/*Opening port*/
int iPort = open(cpPortName, O_RDWR | O_NOCTTY | O_NDELAY);//Port for sensor board (bt)
int iDronePort = open(cpDronePortName, O_RDWR | O_NOCTTY | O_NDELAY);//Port for drone-pc comm
int iMbedPort = open(cpMbedPortName, O_RDWR | O_NOCTTY | O_NDELAY);//Port for home beacon

/*Main function*/
int main(int argc, char **argv){

	/*Objects*/
	ros::init(argc, argv, "i90_sensor");//Create "i90_sensor" node
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Publisher targetPub = n.advertise<i90_sensor_board::pos>("i90_target_pos", 1);//Publisher for target position
	ros::Subscriber translationSub = n.subscribe("i90_current_pos", 1, recalculateTarget);//Subscriber for current position

	/*Prepare the serial ports*/
  if(iPort == -1){
	  printf("Error opening the port\n\r");//Inform user on the terminal
  }
  else{
	  printf("Sensor board serial port is OPEN\n\r");//Inform user on the terminal
  }
	setInterfaceAttribs (iPort, B115200, 0);//Set parameters
	tcflush(iPort, TCIFLUSH);//Empty the buffer

  if(iDronePort == -1){
	  printf("Error opening the drone port\n\r");//Inform user on the terminal
  }
  else{
	  printf("Drone PC serial port is OPEN\n\r");//Inform user on the terminal
  }
	setInterfaceAttribs (iDronePort, B9600, 0);//Set parameters
	tcflush(iDronePort, TCIFLUSH);//Empty the buffer

  if(iMbedPort == -1){
	  printf("Error opening the mbed port\n\r");//Inform user on the terminal
  }
  else{
	  printf("Home beacon serial port is OPEN\n\r");//Inform user on the terminal
  }
	setInterfaceAttribs (iMbedPort, B115200, 0);//Set parameters
	tcflush(iMbedPort, TCIFLUSH);//Empty the buffer

	/*Reset flags*/
	bCalculation = false;
	bBeaconFound = false;
	bTurnRight = false;
	bTurnLeft = false;
	fMaxIrAngle = 45.00;

	printf("Searching for the beacon.\n\r");

	while (ros::ok()){
		if(bCalculation == true){

			/*Publish the target*/
			targetPub.publish(targetPos);//Publish calculated position
			bCalculation = false;
		}
		usleep(200000);//Wait for 200ms (5Hz)
		ros::spinOnce();
	}
	return 0;
}

/*Provides new target upon a new current pos publishment*/
void recalculateTarget(const i90_sensor_board::pos i90CurrentPos){

	/*Read current position values*/
	fCurrentPosX = i90CurrentPos.fXPos;
	fCurrentPosY = i90CurrentPos.fYPos;
	fCurrentAngleYaw = i90CurrentPos.fYawAngle;

	/*Request & read sensor values on the mbed board*/
	tcflush(iPort, TCIFLUSH);//Empty the buffer
	write(iPort,"s",1);//"send the request byte 's'
	usleep(300000);//sleep for 300ms to allow mbed read and send all the bytes
	read(iPort,cpData,15);//Read 15 bytes from the buffer (both ir and sonars)

	/*Clean the variables*/
	for(int i=0; i<6;i++){
		fIr[i] = 0;
	}

	/*Convert to integer/float values*/
	for(int i=0;i<3;i++){//Sonar values
		iSonar[i] = cData[i];
		fSonar[i] = iSonar[i] / 100.00;
	}

	for(int i=0;i<6;i++){//Ir values
		fIr[i] += cData[4+2*i] / 100.00;//Decimal part
		fIr[i] += (float) cData[3+2*i];//Integer part
	}

	/*Look for the unobstructed directions*/
	if((fSonar[0] < 0.2) || (fSonar[1] < 0.16) || (fSonar[2] < 0.2)){//If front is blocked to do action1
		for(int i=1;i<5;i++){
			iIrObstructed[i] = 1;
		}
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
			if(fIr[i] >= THRESHOLD){//Check if beacon is close enough
				if(bPassingPointReached == false){//If the beacon found is the target beacon
					bBeaconFound = true;
					printf("Beacon is found.\n\r");
				}
				else{//If the beacon found is the home beacon
					bBeaconFound = true;//Set flags
					bHomeReached = true;
				}
			}
			if(iIrObstructed[i] == 0){//Check ir only if not obstructed
				if(fIr[i] >= fMaxIr){
					fMaxIr = fIr[i];//Assign max ir value
					iMaxIrNum = i;//Assign number of ir sensor w/ max ir
				}
			}
		}
		/*Set travel distance (closer to beacon, smaller the movement)*/
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
			iMaxIrNum = 3;//Go forward if no memory is available
			bIrReception = false;//Set the flag
		}
		else{
			bIrReception = true;
		}
	}

	/*If the search for target/home beacon is active*/
	if(bBeaconFound == false){

		/*If Ir is being received*/
		if(bIrReception == true){

			if(bPassingPointReached == false){//If target beacon is being searched
				targetPos.fYawAngle = fIrAngle[iMaxIrNum];//Set target angle as max ir angle
			}
			else{//If the home beacon is being searched
				if(iMaxIrNum != 0 || iMaxIrNum != 5){//Don't consider ir reception further than 50 degrees to the robot's y axis
					targetPos.fYawAngle = fIrAngle[iMaxIrNum];
				}
			}

			/*Calculate angle offsets based on second max ir reading*/
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
			fMaxIrAngle = fCurrentAngleYaw + fIrAngle[iMaxIrNum];//Convert the angle to the general coord. frame
		}

		/*If Ir is not received*/
		else{
			bool bFlag = false;

			/*If not yet turned to the right/left*/
			if(bFlag == false && bTurnRight == false && bTurnLeft == false){
				/*Turn right/left*/
				if(rand()%1){
					targetPos.fYawAngle = -90.00;//Turn right
					bTurnRight = true;
					bOnlyTurn = true;
					bFlag = true;
				}
				else{
					//printf("No Ir. Turning left.\n\r");
					targetPos.fYawAngle = 90.00;//Turn left
					bTurnLeft = true;
					bOnlyTurn = true;
					bFlag = true;
				}				
			}

			/*If all the directions are obstructed and also right/left is checked, turn towards the center*/
			if(bFlag == false && bTurnRight == true && bTurnLeft == false && bTurnCenter == false){
				targetPos.fYawAngle = 90.00;//Turn left
				bTurnCenter = true;
				bOnlyTurn = true;
				bFlag = true;
			}
			if(bFlag == false && bTurnRight == false && bTurnLeft == true && bTurnCenter == false){
				//printf("No Ir. Turning center.\n\r");
				targetPos.fYawAngle = -90.00;//Turn right
				bTurnCenter = true;
				bOnlyTurn = true;
				bFlag = true;
			}

			/*If all the directions are obstructed, right/left checked and robot is in the center dir, turn right/left*/
			if(bFlag == false && bTurnRight == true && bTurnCenter == true && bTurnLeft == false){
				targetPos.fYawAngle = 90.00;//Turn left
				bTurnLeft = true;
				bOnlyTurn = true;
				bFlag = true;
			}
			if(bFlag == false && bTurnRight == false && bTurnCenter == true && bTurnLeft == true){
				targetPos.fYawAngle = -90.00;//Turn right
				bTurnRight = true;//Ignore not finding enough ir in the left		
				bOnlyTurn = true;
				bFlag = true;

			}

			/*If no ir received in front, right and left go towards the last memory of max ir direction*/
			if(bFlag == false && bTurnRight == true && bTurnCenter == true && bTurnLeft == true){
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

		/*Calculate target pos rel. to gen. coord. frame*/
		if(bOnlyTurn == true){//Only Turn
			targetPos.fYawAngle += fCurrentAngleYaw;//Add current yaw to transfer to original frame
			if(targetPos.fYawAngle < 0.00) targetPos.fYawAngle += 360.00;//Cap between 0-360
			if(targetPos.fYawAngle > 360.00) targetPos.fYawAngle -= 360.00;
			targetPos.fXPos = fCurrentPosX;
			targetPos.fYPos = fCurrentPosY;
			bCalculation = true;//Set flag to publish calculated target
			iCounter++;
		}
		else{//Go forward & Turn & go forward (3 actions)
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
			bCalculation = true;//Set flags to enable publishing on required topics
			bTurnRight = false;
			bTurnLeft = false;
			bTurnCenter = false;
			iCounter++;
		}
	}

	/*If beacon has been found*/
	else{
		/*Send the target to the drone*/
		while(bTargetSent == false){
			/*Sync with the other pc by sending "a" and wait for "b" in return*/
			tcflush(iDronePort, TCIFLUSH);//Empty the buffer
			write(iDronePort,"a",1);
			printf("Sync signal has been sent to the Drone.\n\r");
			/*Wait until the reply*/
			char cIncomingChar = 'a';
			char *cpIncomingChar = &cIncomingChar;

			int iCount = 0;//Counter for time limit for reply
			while(iCount < 120){//If time limit is not reached
				read(iDronePort, cpIncomingChar, 1);//Read 1 byte from the buffer
				if(cIncomingChar != 'b'){//If correct handshake char is received
					bSync = true;
					break;
				}
				usleep(1000000);
				iCount++;
				printf("Time passed: %d\n\r", iCount);//Inform user
			}
			if(iCount == 120){//If loop ended because of time limit
				printf("Synchronization failed.\n\r");
				bSync = false;
			}

			/*If sync was succcessful, send target pos*/
			if(bSync == true){
				/*Calculate the characters to be sent*/
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

		/*If target sent wait for drone to arrive up to 2mins*/
		if(bSync == true){

			/*If drone not arrived yet*/
			usleep(1000000);
			char cIncomingChar = 'a';
			char *cpIncomingChar = &cIncomingChar;

			printf("Waiting for the Drone to arrive.\n\r");

			int iCount = 0;	

			/*While time limit is not reached, keep checking for success/failure message*/
			while(iCount < 120){
				read(iDronePort, cpIncomingChar, 1);//Read 1 byte from the buffer
				if(cIncomingChar == 'c'){//success
					break;
				}
				if(cIncomingChar == 'e'){//failure
					break;
				}
				usleep(1000000);
				iCount++;
				printf("Time passed: %d\n\r", iCount);
			}

			/*If ended due to time limit*/
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
			updatePassingPoint();//Update passing point distance & angle
		}

		if(bPassingPointReached == false){
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

			/*Transform relative angles to the general coordinate frame*/
			if(bOnlyTurn == true){//Only Turn
				targetPos.fYawAngle += fCurrentAngleYaw;//Add current yaw to transfer to original frame
				if(targetPos.fYawAngle < 0.00) targetPos.fYawAngle += 360.00;//Always keep between 0-360
				if(targetPos.fYawAngle > 360.00) targetPos.fYawAngle -= 360.00;
				targetPos.fXPos = fCurrentPosX;
				targetPos.fYPos = fCurrentPosY;
				bCalculation = true;
				iCounter++;
			}

			else{//Go forward & Turn & go forward
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

/*Finds distance between two positions*/
float findDistance(float x1, float y1, float x2, float y2){
	float fDistX = x1 - x2;
	float fDistY = y1 - y2;
	float fDistance = pow(fDistX, 2);
	fDistance += pow(fDistY ,2);
	fDistance = sqrt(fDistance);//hypotenus theorem
	return fDistance;
}

/*Calculates the distance and angle to closer passing point*/
void updatePassingPoint(void){
	/*Find distances to 2 PPs*/
	float fDistances[2];
	fDistances[0] = findDistance(fCurrentPosX, fCurrentPosY, fPassingPoint[0][0], fPassingPoint[0][1]);
	fDistances[1] = findDistance(fCurrentPosX, fCurrentPosY, fPassingPoint[1][0], fPassingPoint[1][1]);

	/*Decide the closer PP*/
	if(fDistances[0] <= fDistances[1]){
		iPassingPointNum = 0;
		fDistanceToPassingPoint = fDistances[0];
	}
	else{
		iPassingPointNum = 1;
		fDistanceToPassingPoint = fDistances[1];
	}

	/*Check the distance to PP to flag if PP has been reached*/
	if(fDistanceToPassingPoint < 0.5){
		bPassingPointReached = true;
	}

	/*Calculate angle*/
	fPassingPointAngle = atan2((fPassingPoint[iPassingPointNum][1] - fCurrentPosY), (fPassingPoint[iPassingPointNum][0] - fCurrentPosX));
	fPassingPointAngle /= M_PI;
	fPassingPointAngle *= 180.00;
	if(fPassingPointAngle < 0.00) fPassingPointAngle += 360.00;
	if(fPassingPointAngle > 360.00) fPassingPointAngle -= 360.00;
}

/*Sets serial port parameters*/
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

